#include "heap.h"

#include <cstddef>
#include <cstdint>
#include <new>

#include "pmm.h"

namespace protos {

namespace {

// ASCII "HEAP", stored in every valid HeapBlockHeader to detect invalid frees
// or header corruption.
constexpr uint32_t kHeapBlockMagic = 0x48454150;
// Initial heap arena size in 4 KiB frames (256 * 4 KiB = 1 MiB).
constexpr size_t kInitialHeapFrames = 256;
// Minimum number of 4 KiB frames requested when expanding the heap (64 KiB).
constexpr size_t kMinExpandFrames = 16;

// Intrusive header placed immediately before each heap block's usable payload:
//
//   +-------------------+------------------------------------+
//   | HeapBlockHeader   | Usable Payload (`size` bytes)      |
//   | (32 bytes)        | (16-byte aligned, returned to user)|
//   +-------------------+------------------------------------+
//   ^                   ^
//   block               reinterpret_cast<void*>(block + 1)
//
// Blocks are kept in a doubly linked list ordered by ascending physical
// address (`g_heap_head`). Note that the list may span multiple non-contiguous
// PMM arenas, so `block->next` is not necessarily physically adjacent to
// `block`.
struct alignas(16) HeapBlockHeader {
  uint32_t magic;
  uint32_t is_free;
  size_t size;
  HeapBlockHeader* prev;
  HeapBlockHeader* next;
};

static_assert(sizeof(HeapBlockHeader) == 32);
static_assert(alignof(HeapBlockHeader) == 16);

// Head of the address-sorted doubly linked list of all heap blocks (both free
// and allocated).
HeapBlockHeader* g_heap_head = nullptr;

// Rounds `value` up to the nearest multiple of `alignment` (must be a power
// of two).
static constexpr uintptr_t AlignUp(const uintptr_t value,
                                   const uintptr_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// Returns true if `second` starts at the exact byte immediately following the
// end of `first`'s payload in physical memory (i.e. both blocks belong to the
// same contiguous arena with no gap between them).
static bool HeapBlocksAreAdjacent(const HeapBlockHeader* const first,
                                  const HeapBlockHeader* const second) {
  if (first == nullptr || second == nullptr) {
    return false;
  }
  const uintptr_t first_end = reinterpret_cast<uintptr_t>(first) +
                              sizeof(HeapBlockHeader) + first->size;
  const uintptr_t second_start = reinterpret_cast<uintptr_t>(second);
  return first_end == second_start;
}

// Merges `initial_block` with its predecessor (`prev`) and/or successor
// (`next`) if they are free and physically adjacent in memory. Clears the
// absorbed headers' magic numbers and returns a pointer to the resulting
// merged block.
static HeapBlockHeader* HeapCoalesceBlock(
    HeapBlockHeader* const initial_block) {
  if (initial_block == nullptr) {
    return nullptr;
  }
  HeapBlockHeader* block = initial_block;
  if (block->prev != nullptr && block->prev->is_free == 1 &&
      HeapBlocksAreAdjacent(block->prev, block)) {
    HeapBlockHeader* const prev_block = block->prev;
    prev_block->size += sizeof(HeapBlockHeader) + block->size;
    prev_block->next = block->next;
    if (block->next != nullptr) {
      block->next->prev = prev_block;
    }
    block->magic = 0;
    block = prev_block;
  }

  while (block->next != nullptr && block->next->is_free == 1 &&
         HeapBlocksAreAdjacent(block, block->next)) {
    HeapBlockHeader* const next_block = block->next;
    block->size += sizeof(HeapBlockHeader) + next_block->size;
    block->next = next_block->next;
    if (next_block->next != nullptr) {
      next_block->next->prev = block;
    }
    next_block->magic = 0;
  }

  return block;
}

// Splits a free `block` at offset `sizeof(HeapBlockHeader) + aligned_size`
// into two contiguous blocks if the trailing remainder is large enough to hold
// both a new `HeapBlockHeader` (32 bytes) and at least `kHeapAlignment`
// (16 bytes) of payload:
//
//   Before split:
//   [ Header | <---------------- block->size bytes ----------------> ]
//
//   After split (at `block + sizeof(HeapBlockHeader) + aligned_size`):
//   [ Header | aligned_size bytes ][ New Header | remainder bytes    ]
//   ^                              ^
//   block (shrunk to aligned_size) new_block (marked free, linked as
//   block->next)
//
// If the remainder is smaller than `sizeof(HeapBlockHeader) + kHeapAlignment`,
// `block` is left unsplit to avoid creating unusable fragments.
static void HeapSplitBlock(HeapBlockHeader* const block,
                           const size_t aligned_size) {
  if (block == nullptr) {
    return;
  }
  const size_t min_split_threshold =
      aligned_size + sizeof(HeapBlockHeader) + kHeapAlignment;
  if (block->size < min_split_threshold) {
    return;
  }

  const uintptr_t new_block_addr = reinterpret_cast<uintptr_t>(block) +
                                   sizeof(HeapBlockHeader) + aligned_size;
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(new_block_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 1;
  new_block->size = block->size - aligned_size - sizeof(HeapBlockHeader);
  new_block->prev = block;
  new_block->next = block->next;

  if (block->next != nullptr) {
    block->next->prev = new_block;
  }
  block->next = new_block;
  block->size = aligned_size;
}

// Initializes a newly allocated PMM frame range `[arena_addr, arena_addr +
// arena_bytes)` as a free heap block, inserts it into `g_heap_head` in
// ascending physical address order, and coalesces it with adjacent free blocks
// if contiguous.
static HeapBlockHeader* HeapInsertArena(const uintptr_t arena_addr,
                                        const size_t arena_bytes) {
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(arena_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 1;
  new_block->size = arena_bytes - sizeof(HeapBlockHeader);
  new_block->prev = nullptr;
  new_block->next = nullptr;

  if (g_heap_head == nullptr) {
    g_heap_head = new_block;
    return new_block;
  }

  if (arena_addr < reinterpret_cast<uintptr_t>(g_heap_head)) {
    new_block->next = g_heap_head;
    g_heap_head->prev = new_block;
    g_heap_head = new_block;
    return HeapCoalesceBlock(new_block);
  }

  HeapBlockHeader* curr = g_heap_head;
  while (curr->next != nullptr &&
         reinterpret_cast<uintptr_t>(curr->next) < arena_addr) {
    curr = curr->next;
  }

  new_block->next = curr->next;
  new_block->prev = curr;
  if (curr->next != nullptr) {
    curr->next->prev = new_block;
  }
  curr->next = new_block;

  return HeapCoalesceBlock(new_block);
}

// Requests additional contiguous 4 KiB frames from the PMM to satisfy an
// allocation of `aligned_size` bytes (requesting at least `kMinExpandFrames`
// when possible, falling back to the exact frame count needed), and inserts
// the new arena into the heap list.
static HeapBlockHeader* HeapExpand(const size_t aligned_size) {
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  if (max_phys <= sizeof(HeapBlockHeader) ||
      aligned_size > max_phys - sizeof(HeapBlockHeader)) {
    return nullptr;
  }
  const size_t needed_bytes = sizeof(HeapBlockHeader) + aligned_size;
  const size_t exact_frames = (needed_bytes + kPageSize - 1) / kPageSize;
  const size_t preferred_frames =
      (exact_frames < kMinExpandFrames) ? kMinExpandFrames : exact_frames;

  size_t alloc_frames = preferred_frames;
  uintptr_t arena_addr = PmmAllocFrames(alloc_frames);
  if (arena_addr == 0 && alloc_frames > exact_frames) {
    alloc_frames = exact_frames;
    arena_addr = PmmAllocFrames(alloc_frames);
  }
  if (arena_addr == 0) {
    return nullptr;
  }

  return HeapInsertArena(arena_addr, alloc_frames * kPageSize);
}

// Helper for C++ aligned `operator new` overloads; supports alignments up to
// `kHeapAlignment` (16 bytes).
static void* KmallocAligned(const size_t size, const size_t alignment) {
  if (alignment > kHeapAlignment) {
    return nullptr;
  }
  return Kmalloc(size);
}

}  // namespace

bool HeapInit() {
  const uintptr_t arena_addr = PmmAllocFrames(kInitialHeapFrames);
  if (arena_addr == 0) {
    return false;
  }
  const size_t arena_bytes = kInitialHeapFrames * kPageSize;
  g_heap_head = nullptr;
  HeapInsertArena(arena_addr, arena_bytes);
  return g_heap_head != nullptr;
}

size_t HeapTotalFreeBytes() {
  size_t total = 0;
  const HeapBlockHeader* curr = g_heap_head;
  while (curr != nullptr) {
    if (curr->is_free == 1) {
      total += curr->size;
    }
    curr = curr->next;
  }
  return total;
}

// First-fit allocator over `g_heap_head`:
// 1. Rounds `size` up to a non-zero multiple of `kHeapAlignment` (16 bytes).
// 2. Scans the block list for the first free block with `size >= aligned_size`,
//    splits off any excess tail via `HeapSplitBlock`, marks it used, and
//    returns the payload pointer (`curr + 1`).
// 3. If no existing free block fits, expands the heap via `HeapExpand`.
void* Kmalloc(const size_t size) {
  if (g_heap_head == nullptr) {
    if (!HeapInit()) {
      return nullptr;
    }
  }
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  if (max_phys <= sizeof(HeapBlockHeader) + kHeapAlignment ||
      size > max_phys - sizeof(HeapBlockHeader) - kHeapAlignment) {
    return nullptr;
  }

  const size_t raw_size = (size == 0) ? kHeapAlignment : size;
  const size_t aligned_size = AlignUp(raw_size, kHeapAlignment);

  HeapBlockHeader* curr = g_heap_head;
  while (curr != nullptr) {
    if (curr->is_free == 1 && curr->size >= aligned_size) {
      HeapSplitBlock(curr, aligned_size);
      curr->is_free = 0;
      return reinterpret_cast<void*>(curr + 1);
    }
    curr = curr->next;
  }

  HeapBlockHeader* const expanded = HeapExpand(aligned_size);
  if (expanded == nullptr || expanded->size < aligned_size) {
    return nullptr;
  }
  HeapSplitBlock(expanded, aligned_size);
  expanded->is_free = 0;
  return reinterpret_cast<void*>(expanded + 1);
}

// Validates that `ptr` points to a live, 16-byte-aligned payload preceded by a
// valid in-use `HeapBlockHeader`, marks the block free, and coalesces it with
// physically adjacent free neighbors.
void Kfree(void* const ptr) {
  if (ptr == nullptr) {
    return;
  }
  const uintptr_t ptr_addr = reinterpret_cast<uintptr_t>(ptr);
  if (ptr_addr < kLowerMemoryLimit + sizeof(HeapBlockHeader) ||
      ptr_addr >= PmmMaxPhysicalAddress() ||
      (ptr_addr & (kHeapAlignment - 1)) != 0) {
    return;
  }

  HeapBlockHeader* const block = reinterpret_cast<HeapBlockHeader*>(ptr) - 1;
  if (block->magic != kHeapBlockMagic || block->is_free != 0) {
    return;
  }

  block->is_free = 1;
  HeapCoalesceBlock(block);
}

}  // namespace protos

void* operator new(const size_t size) { return protos::Kmalloc(size); }

void* operator new[](const size_t size) { return protos::Kmalloc(size); }

void* operator new(const size_t size, const std::align_val_t alignment) {
  return protos::KmallocAligned(size, static_cast<size_t>(alignment));
}

void* operator new[](const size_t size, const std::align_val_t alignment) {
  return protos::KmallocAligned(size, static_cast<size_t>(alignment));
}

void operator delete(void* const ptr) noexcept { protos::Kfree(ptr); }

void operator delete[](void* const ptr) noexcept { protos::Kfree(ptr); }

void operator delete(void* const ptr, const size_t size) noexcept {
  (void)size;
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr, const size_t size) noexcept {
  (void)size;
  protos::Kfree(ptr);
}

void operator delete(void* const ptr,
                     const std::align_val_t alignment) noexcept {
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr,
                       const std::align_val_t alignment) noexcept {
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete(void* const ptr,    //
                     const size_t size,  //
                     const std::align_val_t alignment) noexcept {
  (void)size;
  (void)alignment;
  protos::Kfree(ptr);
}

void operator delete[](void* const ptr,    //
                       const size_t size,  //
                       const std::align_val_t alignment) noexcept {
  (void)size;
  (void)alignment;
  protos::Kfree(ptr);
}
