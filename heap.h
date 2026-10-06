#ifndef PROTOS_HEAP_H_
#define PROTOS_HEAP_H_

#include <cstdint>

namespace protos {

// Minimum and default alignment (in bytes) guaranteed for all heap allocations.
constexpr int64_t kHeapAlignment = 16;

// Initializes the kernel heap with an initial arena of physical frames from
// the PMM. Panics via `CHECK` if the initial arena cannot be allocated and
// linked.
void HeapInit();

// Returns the sum of payload bytes across all free blocks currently in the
// heap block list.
int64_t HeapTotalFreeBytes();

// Allocates at least `size` bytes of 16-byte-aligned memory from the kernel
// heap (`size` must be >= 0). Returns nullptr if out of memory. If `size == 0`,
// allocates a minimum 16-byte payload block.
void* Kmalloc(int64_t size);

// Frees a pointer previously returned by `Kmalloc` and coalesces it with any
// physically adjacent free blocks. Safe no-op if `ptr == nullptr`; validates
// with `DCHECK` that non-null `ptr` is a valid, aligned, in-use heap block.
void Kfree(void* ptr);

}  // namespace protos

#endif  // PROTOS_HEAP_H_
