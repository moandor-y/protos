#ifndef PROTOS_PMM_H_
#define PROTOS_PMM_H_

#include <cstddef>
#include <cstdint>

namespace protos {

// Physical frame size (4 KiB).
constexpr uintptr_t kPageSize = 4096;
// Physical addresses below 1 MiB (BIOS, real-mode IVT/BDA, VGA buffer at
// 0xB8000) are always reserved and never handed out by the frame allocator.
constexpr uintptr_t kLowerMemoryLimit = 0x100000;

// Parses the Multiboot memory map, dynamically places and initializes the PMM
// frame bitmap, extends the identity mapping across all usable physical RAM,
// and reserves lower memory, the kernel image, the bitmap, and bootloader
// structures. Returns true if at least one usable frame is available.
bool PmmInit(uint32_t multiboot_magic, uint64_t multiboot_info_addr);

// Allocates `count` contiguous 4 KiB physical frames and returns the base
// physical address, or 0 if no contiguous run of `count` frames is available.
uintptr_t PmmAllocFrames(size_t count);

// Allocates a single 4 KiB physical frame and returns its physical address,
// or 0 if out of memory.
uintptr_t PmmAllocFrame();

// Frees `count` contiguous 4 KiB physical frames starting at page-aligned
// `base_addr`. Ignores frames outside usable RAM or overlapping reserved
// ranges.
void PmmFreeFrames(uintptr_t base_addr, size_t count);

// Frees a single 4 KiB physical frame at page-aligned `frame_addr`.
void PmmFreeFrame(uintptr_t frame_addr);

// Returns true if `[addr, addr + size)` lies strictly within a usable RAM
// region above 1 MiB and does not overlap the kernel image, PMM bitmap, or
// Multiboot data.
bool PmmRangeIsValidUsableRam(uintptr_t addr, size_t size);

// Returns the highest page-aligned usable physical RAM address discovered from
// the Multiboot memory map.
uintptr_t PmmMaxPhysicalAddress();

// Returns the total number of 4 KiB frame slots tracked in the PMM bitmap
// (`PmmMaxPhysicalAddress() / kPageSize`).
size_t PmmMaxFrameCount();

// Returns the number of currently free 4 KiB frames.
size_t PmmFreeFrameCount();

// Returns the total number of usable 4 KiB frames available immediately after
// `PmmInit` completed.
size_t PmmTotalUsableFrameCount();

}  // namespace protos

#endif  // PROTOS_PMM_H_
