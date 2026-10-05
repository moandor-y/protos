#include "heap.h"

#include <cstddef>
#include <cstdint>
#include <new>

#include "check.h"
#include "pmm.h"
#include "rbtree.h"

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
//   | (64 bytes)        | (16-byte aligned, returned to user)|
//   +-------------------+------------------------------------+
//   ^                   ^
//   block               reinterpret_cast<void*>(block + 1)
//
// Free blocks are stored in an address-ordered Red-Black Tree (`g_free_tree`)
// augmented with `max_free_size` (the largest free block payload `size` in the
// node's subtree) to support O(log n) address-ordered first-fit allocation and
// O(log n) neighbor lookup for immediate physical coalescing.
struct alignas(kHeapAlignment) HeapBlockHeader {
  uint32_t magic;
  uint32_t is_free;
  size_t size;
  size_t max_free_size;
  RbNode rb_node;
};

static_assert(sizeof(HeapBlockHeader) == 64);
static_assert(alignof(HeapBlockHeader) == kHeapAlignment);

struct FreeBlockRbTraits {
  static uintptr_t GetKey(const HeapBlockHeader& block) {
    return reinterpret_cast<uintptr_t>(&block);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }

  static void UpdateAugment(HeapBlockHeader* const block,
                            const HeapBlockHeader* const left,
                            const HeapBlockHeader* const right) {
    size_t max_size = block->size;
    if (left != nullptr && left->max_free_size > max_size) {
      max_size = left->max_free_size;
    }
    if (right != nullptr && right->max_free_size > max_size) {
      max_size = right->max_free_size;
    }
    block->max_free_size = max_size;
  }
};

using FreeBlockTree = RbTree<HeapBlockHeader,            //
                             &HeapBlockHeader::rb_node,  //
                             FreeBlockRbTraits>;

// Intrusive Red-Black Tree containing all currently FREE heap blocks across all
// PMM arenas, keyed by each block header's physical address (`uintptr_t`) and
// augmented with `max_free_size` (the maximum payload `size` among all free
// blocks in the node's subtree). Allocated blocks are removed from this tree
// and re-inserted when freed.
FreeBlockTree g_free_tree;
// Sum of usable payload bytes (`block->size`) across all free blocks in
// `g_free_tree`.
size_t g_total_free_bytes = 0;
// True once `HeapInit()` has successfully allocated and inserted the initial
// heap arena.
bool g_heap_initialized = false;

// Rounds `value` up to the nearest multiple of `alignment` (must be a power
// of two).
static constexpr uintptr_t AlignUp(const uintptr_t value,
                                   const uintptr_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// Returns true if `second` starts at the exact byte immediately following the
// end of `first`'s payload in physical memory (i.e. both blocks are physically
// contiguous with no allocated block or unmapped arena gap between them).
static bool HeapBlocksAreAdjacent(const HeapBlockHeader* const first,
                                  const HeapBlockHeader* const second) {
  DCHECK(first != nullptr);
  DCHECK(second != nullptr);
  const uintptr_t first_end = reinterpret_cast<uintptr_t>(first) +
                              sizeof(HeapBlockHeader) + first->size;
  const uintptr_t second_start = reinterpret_cast<uintptr_t>(second);
  return first_end == second_start;
}

static void FreeTreeInsert(HeapBlockHeader* const block) {
  DCHECK(block != nullptr);
  DCHECK(block->magic == kHeapBlockMagic);
  DCHECK(block->is_free == 0);
  block->is_free = 1;
  block->max_free_size = block->size;
  block->rb_node = {};
  const bool inserted = g_free_tree.Insert(block);
  DCHECK(inserted);
  g_total_free_bytes += block->size;
}

static void FreeTreeRemove(HeapBlockHeader* const block) {
  DCHECK(block != nullptr);
  DCHECK(block->magic == kHeapBlockMagic);
  DCHECK(block->is_free == 1);
  DCHECK(g_total_free_bytes >= block->size);
  g_free_tree.Erase(block);
  block->is_free = 0;
  block->max_free_size = 0;
  g_total_free_bytes -= block->size;
}

// Inserts `initial_block` into `g_free_tree` (if not already present) and
// merges it in O(log n) time with its in-order predecessor (`Prev`) and/or
// successor (`Next`) if they are physically adjacent in memory. Clears absorbed
// headers' magic numbers and returns a pointer to the resulting merged free
// block.
static HeapBlockHeader* HeapCoalesceBlock(
    HeapBlockHeader* const initial_block) {
  DCHECK(initial_block != nullptr);
  HeapBlockHeader* block = initial_block;
  if (block->is_free == 0) {
    FreeTreeInsert(block);
  }

  HeapBlockHeader* const prev_free = FreeBlockTree::Prev(block);
  HeapBlockHeader* const next_free = FreeBlockTree::Next(block);

  if (next_free != nullptr && HeapBlocksAreAdjacent(block, next_free)) {
    const size_t next_size = next_free->size;
    FreeTreeRemove(next_free);
    next_free->magic = 0;
    const size_t gained_bytes = sizeof(HeapBlockHeader) + next_size;
    block->size += gained_bytes;
    g_total_free_bytes += gained_bytes;
    g_free_tree.PropagateAugment(block);
  }

  if (prev_free != nullptr && HeapBlocksAreAdjacent(prev_free, block)) {
    const size_t block_size = block->size;
    FreeTreeRemove(block);
    block->magic = 0;
    const size_t gained_bytes = sizeof(HeapBlockHeader) + block_size;
    prev_free->size += gained_bytes;
    g_total_free_bytes += gained_bytes;
    g_free_tree.PropagateAugment(prev_free);
    block = prev_free;
  }

  return block;
}

// Returns true if `block` has enough trailing bytes beyond `aligned_size` to
// form a separate free block with its own `HeapBlockHeader` (64 bytes) and at
// least `kHeapAlignment` (16 bytes) of payload.
static bool HeapCanSplitBlock(const HeapBlockHeader* const block,
                              const size_t aligned_size) {
  DCHECK(block != nullptr);
  const size_t min_split_threshold =
      aligned_size + sizeof(HeapBlockHeader) + kHeapAlignment;
  return block->size >= min_split_threshold;
}

// Splits `block` at offset `sizeof(HeapBlockHeader) + aligned_size` into two
// contiguous blocks and inserts the trailing remainder into `g_free_tree` in
// O(log n) time. Requires `HeapCanSplitBlock(block, aligned_size)`.
static void HeapSplitBlock(HeapBlockHeader* const block,
                           const size_t aligned_size) {
  DCHECK(block != nullptr);
  DCHECK(HeapCanSplitBlock(block, aligned_size));

  const uintptr_t new_block_addr = reinterpret_cast<uintptr_t>(block) +
                                   sizeof(HeapBlockHeader) + aligned_size;
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(new_block_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 0;
  new_block->size = block->size - aligned_size - sizeof(HeapBlockHeader);
  new_block->max_free_size = 0;
  new_block->rb_node = {};

  if (block->is_free == 1) {
    g_total_free_bytes -= (block->size - aligned_size);
    block->size = aligned_size;
    g_free_tree.PropagateAugment(block);
  } else {
    block->size = aligned_size;
  }

  FreeTreeInsert(new_block);
}

// Initializes a newly allocated PMM frame range `[arena_addr, arena_addr +
// arena_bytes)` as a free heap block, inserts it into `g_free_tree` in O(log n)
// time, and coalesces it with physically adjacent free neighbors if contiguous.
static HeapBlockHeader* HeapInsertArena(const uintptr_t arena_addr,
                                        const size_t arena_bytes) {
  DCHECK(arena_addr != 0);
  DCHECK(arena_bytes >= sizeof(HeapBlockHeader) + kHeapAlignment);
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(arena_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 0;
  new_block->size = arena_bytes - sizeof(HeapBlockHeader);
  new_block->max_free_size = 0;
  new_block->rb_node = {};

  return HeapCoalesceBlock(new_block);
}

// Requests additional contiguous 4 KiB frames from the PMM to satisfy an
// allocation of `aligned_size` bytes (requesting at least `kMinExpandFrames`
// when possible, falling back to the exact frame count needed), and inserts
// the new arena into `g_free_tree`.
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

// Finds the lowest-address free block with `size >= aligned_size` in O(log n)
// time using subtree `max_free_size` augmentation.
static HeapBlockHeader* HeapFindFirstFit(const size_t aligned_size) {
  return g_free_tree.FindFirstAugmented(
      [aligned_size](const HeapBlockHeader& block) {
        return block.max_free_size >= aligned_size;
      },
      [aligned_size](const HeapBlockHeader& block) {
        return block.size >= aligned_size;
      });
}

#if !__STDC_HOSTED__
// Helper for C++ aligned `operator new` overloads; supports alignments up to
// `kHeapAlignment` (16 bytes).
static void* KmallocAligned(const size_t size, const size_t alignment) {
  if (alignment > kHeapAlignment) {
    return nullptr;
  }
  return Kmalloc(size);
}
#endif  // !__STDC_HOSTED__

}  // namespace

bool HeapInit() {
  g_free_tree.Clear();
  g_total_free_bytes = 0;
  g_heap_initialized = false;

  const uintptr_t arena_addr = PmmAllocFrames(kInitialHeapFrames);
  if (arena_addr == 0) {
    return false;
  }
  const size_t arena_bytes = kInitialHeapFrames * kPageSize;
  const HeapBlockHeader* const block = HeapInsertArena(arena_addr, arena_bytes);
  g_heap_initialized = (block != nullptr);
  return g_heap_initialized;
}

size_t HeapTotalFreeBytes() { return g_total_free_bytes; }

// Address-ordered first-fit allocator over `g_free_tree`:
// 1. Rounds `size` up to a non-zero multiple of `kHeapAlignment` (16 bytes).
// 2. Queries `g_free_tree.FindFirstAugmented` in O(log n) time for the lowest-
//    address free block with `size >= aligned_size`, removes it from the free
//    tree, splits off any excess tail via `HeapSplitBlock`, and returns the
//    payload pointer (`candidate + 1`).
// 3. If no existing free block fits, expands the heap via `HeapExpand`.
void* Kmalloc(const size_t size) {
  DCHECK(g_heap_initialized);
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  if (max_phys <= sizeof(HeapBlockHeader) + kHeapAlignment ||
      size > max_phys - sizeof(HeapBlockHeader) - kHeapAlignment) {
    return nullptr;
  }

  const size_t raw_size = (size == 0) ? kHeapAlignment : size;
  const size_t aligned_size = AlignUp(raw_size, kHeapAlignment);

  HeapBlockHeader* candidate = HeapFindFirstFit(aligned_size);
  if (candidate == nullptr) {
    candidate = HeapExpand(aligned_size);
    if (candidate == nullptr) {
      return nullptr;
    }
    DCHECK(candidate->size >= aligned_size);
  }

  FreeTreeRemove(candidate);
  if (HeapCanSplitBlock(candidate, aligned_size)) {
    HeapSplitBlock(candidate, aligned_size);
  }
  return reinterpret_cast<void*>(candidate + 1);
}

// Validates that `ptr` points to a live, 16-byte-aligned payload preceded by a
// valid in-use `HeapBlockHeader`, inserts the block into `g_free_tree`, and
// coalesces it with physically adjacent free neighbors in O(log n) time.
void Kfree(void* const ptr) {
  if (ptr == nullptr) {
    return;
  }
  DCHECK(g_heap_initialized);
  const uintptr_t ptr_addr = reinterpret_cast<uintptr_t>(ptr);
  if (ptr_addr < kLowerMemoryLimit + sizeof(HeapBlockHeader) ||
      ptr_addr >= PmmMaxPhysicalAddress() ||
      (ptr_addr & (kHeapAlignment - 1)) != 0) {
    return;
  }

  HeapBlockHeader* const block = reinterpret_cast<HeapBlockHeader*>(ptr) - 1;
  if (block->magic != kHeapBlockMagic || block->is_free != 0 ||
      block->size < kHeapAlignment ||
      (block->size & (kHeapAlignment - 1)) != 0) {
    return;
  }

  HeapCoalesceBlock(block);
}

}  // namespace protos

#if !__STDC_HOSTED__
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
#endif  // !__STDC_HOSTED__
