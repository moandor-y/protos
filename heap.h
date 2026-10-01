#ifndef PROTOS_HEAP_H_
#define PROTOS_HEAP_H_

#include <cstddef>
#include <cstdint>
#include <new>

namespace protos {

// Minimum and default alignment (in bytes) guaranteed for all heap allocations.
constexpr size_t kHeapAlignment = 16;

// Initializes the kernel heap with an initial arena of physical frames from
// the PMM. Returns true if the initial arena was allocated and linked.
bool HeapInit();

// Returns the sum of payload bytes across all free blocks currently in the
// heap block list.
size_t HeapTotalFreeBytes();

// Allocates at least `size` bytes of 16-byte-aligned memory from the kernel
// heap. Returns nullptr if out of memory. If `size == 0`, allocates a minimum
// 16-byte payload block.
void* Kmalloc(size_t size);

// Frees a pointer previously returned by `Kmalloc` and coalesces it with any
// physically adjacent free blocks. Safe no-op if `ptr == nullptr`.
void Kfree(void* ptr);

}  // namespace protos

#endif  // PROTOS_HEAP_H_
