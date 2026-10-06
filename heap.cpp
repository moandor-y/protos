#include "heap.h"

#include <cstddef>
#include <cstdint>
#include <memory>
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
constexpr int64_t kInitialHeapFrames = 256;
// Minimum number of 4 KiB frames requested when expanding the heap (64 KiB).
constexpr int64_t kMinExpandFrames = 16;

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
  int64_t size;
  int64_t max_free_size;
  RbNode rb_node;
};

constexpr int64_t kBlockHeaderSize = sizeof(HeapBlockHeader);
static_assert(kBlockHeaderSize == 64);
static_assert(alignof(HeapBlockHeader) == kHeapAlignment);

struct FreeBlockRbTraits {
  static uintptr_t GetKey(const HeapBlockHeader& block) {
    return reinterpret_cast<uintptr_t>(&block);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }

  static void UpdateAugment(HeapBlockHeader* const block,
                            const HeapBlockHeader* const left,
                            const HeapBlockHeader* const right) {
    int64_t max_size = block->size;
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
int64_t g_total_free_bytes = 0;
// True once `HeapInit()` has successfully allocated and inserted the initial
// heap arena.
bool g_heap_initialized = false;

// Rounds `value` up to the nearest multiple of `alignment` (must be a power
// of two).
static constexpr int64_t AlignUp(const int64_t value, const int64_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// Returns true if `second` starts at the exact byte immediately following the
// end of `first`'s payload in physical memory (i.e. both blocks are physically
// contiguous with no allocated block or unmapped arena gap between them).
static bool HeapBlocksAreAdjacent(const HeapBlockHeader* const first,
                                  const HeapBlockHeader* const second) {
  DCHECK(first != nullptr);
  DCHECK(second != nullptr);
  const uintptr_t first_end =
      reinterpret_cast<uintptr_t>(first) + kBlockHeaderSize + first->size;
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
    const int64_t next_size = next_free->size;
    FreeTreeRemove(next_free);
    next_free->magic = 0;
    const int64_t gained_bytes = kBlockHeaderSize + next_size;
    block->size += gained_bytes;
    g_total_free_bytes += gained_bytes;
    g_free_tree.PropagateAugment(block);
  }

  if (prev_free != nullptr && HeapBlocksAreAdjacent(prev_free, block)) {
    const int64_t block_size = block->size;
    FreeTreeRemove(block);
    block->magic = 0;
    const int64_t gained_bytes = kBlockHeaderSize + block_size;
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
                              const int64_t aligned_size) {
  DCHECK(block != nullptr);
  const int64_t min_split_threshold =
      aligned_size + kBlockHeaderSize + kHeapAlignment;
  return block->size >= min_split_threshold;
}

// Splits `block` at offset `sizeof(HeapBlockHeader) + aligned_size` into two
// contiguous blocks and inserts the trailing remainder into `g_free_tree` in
// O(log n) time. Requires `HeapCanSplitBlock(block, aligned_size)`.
static void HeapSplitBlock(HeapBlockHeader* const block,
                           const int64_t aligned_size) {
  DCHECK(block != nullptr);
  DCHECK(HeapCanSplitBlock(block, aligned_size));

  const uintptr_t new_block_addr =
      reinterpret_cast<uintptr_t>(block) + kBlockHeaderSize + aligned_size;
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(new_block_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 0;
  new_block->size = block->size - aligned_size - kBlockHeaderSize;
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
                                        const int64_t arena_bytes) {
  DCHECK(arena_addr != 0);
  DCHECK(arena_bytes >= kBlockHeaderSize + kHeapAlignment);
  HeapBlockHeader* const new_block =
      reinterpret_cast<HeapBlockHeader*>(arena_addr);
  new_block->magic = kHeapBlockMagic;
  new_block->is_free = 0;
  new_block->size = arena_bytes - kBlockHeaderSize;
  new_block->max_free_size = 0;
  new_block->rb_node = {};

  return HeapCoalesceBlock(new_block);
}

// Requests additional contiguous 4 KiB frames from the PMM to satisfy an
// allocation of `aligned_size` bytes (requesting at least `kMinExpandFrames`
// when possible, falling back to the exact frame count needed), and inserts
// the new arena into `g_free_tree`.
static HeapBlockHeader* HeapExpand(const int64_t aligned_size) {
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  if (aligned_size <= 0 || max_phys <= kBlockHeaderSize ||
      static_cast<uintptr_t>(aligned_size) > max_phys - kBlockHeaderSize) {
    return nullptr;
  }
  const int64_t needed_bytes = kBlockHeaderSize + aligned_size;
  const int64_t exact_frames = (needed_bytes + kPageSize - 1) / kPageSize;
  const int64_t preferred_frames =
      (exact_frames < kMinExpandFrames) ? kMinExpandFrames : exact_frames;

  int64_t alloc_frames = preferred_frames;
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
static HeapBlockHeader* HeapFindFirstFit(const int64_t aligned_size) {
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
static void* KmallocAligned(const int64_t size, const int64_t alignment) {
  if (alignment <= 0 || alignment > kHeapAlignment) {
    return nullptr;
  }
  return Kmalloc(size);
}
#endif  // !__STDC_HOSTED__

}  // namespace

void HeapInit() {
  g_free_tree.Clear();
  g_total_free_bytes = 0;
  g_heap_initialized = false;

  const uintptr_t arena_addr = PmmAllocFrames(kInitialHeapFrames);
  CHECK(arena_addr != 0);
  const int64_t arena_bytes = kInitialHeapFrames * kPageSize;
  const HeapBlockHeader* const block = HeapInsertArena(arena_addr, arena_bytes);
  CHECK(block != nullptr);
  g_heap_initialized = true;
}

int64_t HeapTotalFreeBytes() { return g_total_free_bytes; }

// Address-ordered first-fit allocator over `g_free_tree`:
// 1. Rounds `size` up to a non-zero multiple of `kHeapAlignment` (16 bytes).
// 2. Queries `g_free_tree.FindFirstAugmented` in O(log n) time for the lowest-
//    address free block with `size >= aligned_size`, removes it from the free
//    tree, splits off any excess tail via `HeapSplitBlock`, and returns the
//    payload pointer (`candidate + 1`).
// 3. If no existing free block fits, expands the heap via `HeapExpand`.
void* Kmalloc(const int64_t size) {
  DCHECK(g_heap_initialized);
  DCHECK(size >= 0);
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  constexpr uintptr_t kMinOverhead = kBlockHeaderSize + kHeapAlignment;
  if (max_phys <= kMinOverhead ||
      static_cast<uintptr_t>(size) > max_phys - kMinOverhead) {
    return nullptr;
  }

  const int64_t raw_size = (size == 0) ? kHeapAlignment : size;
  const int64_t aligned_size = AlignUp(raw_size, kHeapAlignment);

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
  DCHECK(ptr_addr >= kLowerMemoryLimit + kBlockHeaderSize);
  DCHECK(ptr_addr < PmmMaxPhysicalAddress());
  DCHECK((ptr_addr & (kHeapAlignment - 1)) == 0);

  HeapBlockHeader* const block = reinterpret_cast<HeapBlockHeader*>(ptr) - 1;
  DCHECK(block->magic == kHeapBlockMagic);
  DCHECK(block->is_free == 0);
  DCHECK(block->size >= kHeapAlignment);
  DCHECK((block->size & (kHeapAlignment - 1)) == 0);

  HeapCoalesceBlock(block);
}

}  // namespace protos

#if !__STDC_HOSTED__
void* operator new(const size_t size) {
  if (size > INT64_MAX) {
    return nullptr;
  }
  return protos::Kmalloc(size);
}

void* operator new[](const size_t size) {
  if (size > INT64_MAX) {
    return nullptr;
  }
  return protos::Kmalloc(size);
}

void* operator new(const size_t size, const std::align_val_t alignment) {
  if (size > INT64_MAX || static_cast<size_t>(alignment) > INT64_MAX) {
    return nullptr;
  }
  return protos::KmallocAligned(size, static_cast<int64_t>(alignment));
}

void* operator new[](const size_t size, const std::align_val_t alignment) {
  if (size > INT64_MAX || static_cast<size_t>(alignment) > INT64_MAX) {
    return nullptr;
  }
  return protos::KmallocAligned(size, static_cast<int64_t>(alignment));
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

extern "C" {
// Itanium C++ ABI handler placed in vtable slots for pure virtual functions
// (`= 0`); halts the kernel if an unimplemented pure virtual method is called.
void __cxa_pure_virtual() { CHECK(false); }

// Forces `libstdc++` reference counting (`std::shared_ptr`) to always use
// thread-safe atomic instructions (`lock xadd`) in freestanding kernel builds.
char __libc_single_threaded = 0;
}  // extern "C"

namespace std {
// `std::make_shared`'s control-block vtable (`_Sp_counted_ptr_inplace`)
// references `_Sp_make_shared_tag::_S_eq` (normally provided by `libstdc++.a`)
// as a cross-DSO fallback when `_M_get_deleter` checks whether a `type_info`
// matches `_Sp_make_shared_tag` under `-fno-rtti`. Within a single static
// kernel binary, `_M_get_deleter`'s fast-path pointer comparison against
// `_Sp_make_shared_tag::_S_ti()` already succeeds, so returning `false` here
// satisfies the linker without pulling in `libstdc++`.
bool _Sp_make_shared_tag::_S_eq(const type_info&) noexcept { return false; }
}  // namespace std
#endif  // !__STDC_HOSTED__
