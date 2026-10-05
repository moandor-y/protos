#ifndef PROTOS_MULTIBOOT_H_
#define PROTOS_MULTIBOOT_H_

#include <cstddef>
#include <cstdint>

namespace protos {

// Multiboot memory map entry type indicating usable RAM.
constexpr uint32_t kMemoryTypeAvailable = 1;
// Maximum number of memory map regions stored in `MultibootMemoryMap`.
constexpr size_t kMaxMemoryRegions = 64;

// Normalized physical memory region from a Multiboot1 or Multiboot2 memory map.
struct MemoryRegion {
  uint64_t base;
  uint64_t length;
  uint32_t type;
};

// Parsed summary of the bootloader memory map and the physical address ranges
// occupied by the Multiboot info structures themselves.
struct MultibootMemoryMap {
  MemoryRegion regions[kMaxMemoryRegions];
  size_t region_count;
  uint64_t total_ram_bytes;
  uint64_t usable_ram_bytes;
  uint64_t reserved_ram_bytes;
  uintptr_t mb_reserved_start;
  uintptr_t mb_reserved_end;
  uintptr_t mb1_mmap_reserved_start;
  uintptr_t mb1_mmap_reserved_end;
};

// Parses the Multiboot1 (`0x2BADB002`) or Multiboot2 (`0x36D76289`) memory map
// at `multiboot_info_addr` (must lie below `max_physical_addr`) and populates
// `*out_map`. Returns true if a valid memory map with usable RAM was found.
bool MultibootParseMemoryMap(uint32_t multiboot_magic,      //
                             uint64_t multiboot_info_addr,  //
                             uintptr_t max_physical_addr,   //
                             MultibootMemoryMap* out_map);

}  // namespace protos

#endif  // PROTOS_MULTIBOOT_H_
