#ifndef PROTOS_PMM_H_
#define PROTOS_PMM_H_

#include <cstdint>

namespace protos {

// Physical frame size (4 KiB).
constexpr int64_t kPageSize = 4096;
// Physical addresses below 1 MiB (BIOS, real-mode IVT/BDA, VGA buffer at
// 0xB8000) are always reserved and never handed out by the frame allocator.
constexpr uintptr_t kLowerMemoryLimit = 0x100000;

// Parses the Multiboot memory map, carves out lower memory, the kernel image,
// the framebuffer, and bootloader structures, extends the identity mapping
// across usable physical RAM, and populates the intrusive Red-Black Tree of
// free frame runs. Returns true if at least one usable frame is available.
bool PmmInit(uint32_t multiboot_magic, uint64_t multiboot_info_addr);

// Allocates `count` contiguous 4 KiB physical frames (`count > 0`) and returns
// the base physical address, or 0 if no contiguous run of `count` frames is
// available.
uintptr_t PmmAllocFrames(int64_t count);

// Allocates a single 4 KiB physical frame and returns its physical address,
// or 0 if out of memory.
uintptr_t PmmAllocFrame();

// Frees `count` contiguous 4 KiB physical frames starting at page-aligned
// `base_addr` and coalesces the range with physically adjacent free runs in
// O(log n) time. Validates with `DCHECK` that the range is non-empty,
// page-aligned, within usable RAM, and does not overlap any already-free
// frames.
void PmmFreeFrames(uintptr_t base_addr, int64_t count);

// Frees a single 4 KiB physical frame at page-aligned `frame_addr`.
void PmmFreeFrame(uintptr_t frame_addr);

// Returns true if `[addr, addr + size)` lies strictly within a usable RAM
// region above 1 MiB and does not overlap the kernel image, framebuffer, or
// Multiboot data.
bool PmmRangeIsValidUsableRam(uintptr_t addr, int64_t size);

// Returns the highest page-aligned usable physical RAM address discovered from
// the Multiboot memory map.
uintptr_t PmmMaxPhysicalAddress();

// Returns the total number of 4 KiB frame slots up to the highest usable
// physical address (`PmmMaxPhysicalAddress() / kPageSize`).
int64_t PmmMaxFrameCount();

// Returns the number of currently free 4 KiB frames.
int64_t PmmFreeFrameCount();

// Returns the total number of usable 4 KiB frames available immediately after
// `PmmInit` completed.
int64_t PmmTotalUsableFrameCount();

}  // namespace protos

#endif  // PROTOS_PMM_H_
