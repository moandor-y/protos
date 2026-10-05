#include "multiboot.h"

#include <cstddef>
#include <cstdint>

#include "paging.h"
#include "uart.h"
#include "vga.h"

namespace protos {

namespace {

constexpr uint32_t kMultiboot1Magic = 0x2BADB002;
constexpr uint32_t kMultiboot2Magic = 0x36D76289;
constexpr uint32_t kMultiboot1FlagMmap = 1 << 6;
constexpr uint32_t kMultiboot2TagEnd = 0;
constexpr uint32_t kMultiboot2TagMmap = 6;
constexpr uint32_t kMultiboot2TagFramebuffer = 8;
constexpr uint8_t kMultiboot2FramebufferTypeRgb = 1;
constexpr uint32_t kMaxMultibootStructureBytes = 4 * 1024 * 1024;

struct [[gnu::packed]] Multiboot1Info {
  uint32_t flags;
  uint32_t mem_lower;
  uint32_t mem_upper;
  uint32_t boot_device;
  uint32_t cmdline;
  uint32_t mods_count;
  uint32_t mods_addr;
  uint32_t syms[4];
  uint32_t mmap_length;
  uint32_t mmap_addr;
};

struct [[gnu::packed]] Multiboot1MmapEntry {
  uint32_t size;
  uint64_t addr;
  uint64_t len;
  uint32_t type;
};

struct [[gnu::packed]] Multiboot2InfoHeader {
  uint32_t total_size;
  uint32_t reserved;
};

struct [[gnu::packed]] Multiboot2Tag {
  uint32_t type;
  uint32_t size;
};

struct [[gnu::packed]] Multiboot2TagMmap {
  uint32_t type;
  uint32_t size;
  uint32_t entry_size;
  uint32_t entry_version;
};

struct [[gnu::packed]] Multiboot2MmapEntry {
  uint64_t addr;
  uint64_t len;
  uint32_t type;
  uint32_t reserved;
};

struct [[gnu::packed]] Multiboot2TagFramebuffer {
  uint32_t type;
  uint32_t size;
  uint64_t framebuffer_addr;
  uint32_t framebuffer_pitch;
  uint32_t framebuffer_width;
  uint32_t framebuffer_height;
  uint8_t framebuffer_bpp;
  uint8_t framebuffer_type;
  uint16_t reserved;
};

static constexpr uintptr_t AlignUp(const uintptr_t value,
                                   const uintptr_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

static void ConsoleWrite(const char* const str) {
  UartWrite(str);
  VgaWrite(str);
}

static void ConsoleWriteHex(const uint64_t value) {
  UartWriteHex(value);
  VgaWriteHex(value);
}

static void ConsoleWriteDec(const uint64_t value) {
  UartWriteDec(value);
  VgaWriteDec(value);
}

static void RecordMmapEntry(MultibootMemoryMap* const out_map,  //
                            const uint64_t base,                //
                            const uint64_t length,              //
                            const uint32_t type) {
  if (out_map == nullptr || length == 0) {
    return;
  }
  if (out_map->region_count < kMaxMemoryRegions) {
    out_map->regions[out_map->region_count].base = base;
    out_map->regions[out_map->region_count].length = length;
    out_map->regions[out_map->region_count].type = type;
    ++out_map->region_count;
  }
  out_map->total_ram_bytes += length;
  if (type == kMemoryTypeAvailable) {
    out_map->usable_ram_bytes += length;
  } else {
    out_map->reserved_ram_bytes += length;
  }
  ConsoleWrite("[PMM] mmap entry: base=");
  ConsoleWriteHex(base);
  ConsoleWrite(" len=");
  ConsoleWriteHex(length);
  ConsoleWrite(" type=");
  ConsoleWriteDec(type);
  ConsoleWrite("\n");
}

}  // namespace

bool MultibootParseMemoryMap(const uint32_t multiboot_magic,
                             const uint64_t multiboot_info_addr,
                             const uintptr_t max_physical_addr,
                             MultibootMemoryMap* const out_map) {
  if (out_map == nullptr) {
    return false;
  }

  out_map->region_count = 0;
  out_map->total_ram_bytes = 0;
  out_map->usable_ram_bytes = 0;
  out_map->reserved_ram_bytes = 0;
  out_map->mb_reserved_start = 0;
  out_map->mb_reserved_end = 0;
  out_map->mb1_mmap_reserved_start = 0;
  out_map->mb1_mmap_reserved_end = 0;
  out_map->fb_addr = 0;
  out_map->fb_pitch = 0;
  out_map->fb_width = 0;
  out_map->fb_height = 0;
  out_map->fb_bpp = 0;
  out_map->fb_type = 0;

  ConsoleWrite("[PMM] Multiboot magic=");
  ConsoleWriteHex(multiboot_magic);
  ConsoleWrite(" info_addr=");
  ConsoleWriteHex(multiboot_info_addr);
  ConsoleWrite("\n");

  if (multiboot_info_addr == 0 || multiboot_info_addr >= max_physical_addr) {
    return false;
  }
  const uintptr_t info_phys = static_cast<uintptr_t>(multiboot_info_addr);

  if (multiboot_magic == kMultiboot2Magic) {
    if (!PagingMapBootstrapRange(info_phys, sizeof(Multiboot2InfoHeader))) {
      return false;
    }
    const Multiboot2InfoHeader* const header =
        reinterpret_cast<const Multiboot2InfoHeader*>(info_phys);
    const uint32_t total_size = header->total_size;
    if (total_size < sizeof(Multiboot2InfoHeader) ||
        total_size > kMaxMultibootStructureBytes ||
        (max_physical_addr - info_phys) < total_size) {
      return false;
    }
    if (!PagingMapBootstrapRange(info_phys, total_size)) {
      return false;
    }
    out_map->mb_reserved_start = info_phys;
    out_map->mb_reserved_end = info_phys + total_size;

    uintptr_t offset = sizeof(Multiboot2InfoHeader);
    while (offset + sizeof(Multiboot2Tag) <= total_size) {
      const Multiboot2Tag* const tag =
          reinterpret_cast<const Multiboot2Tag*>(info_phys + offset);
      if (tag->type == kMultiboot2TagEnd || tag->size < sizeof(Multiboot2Tag)) {
        break;
      }
      if (tag->type == kMultiboot2TagMmap &&
          tag->size >= sizeof(Multiboot2TagMmap)) {
        const Multiboot2TagMmap* const mmap_tag =
            reinterpret_cast<const Multiboot2TagMmap*>(tag);
        if (mmap_tag->entry_size >= sizeof(Multiboot2MmapEntry)) {
          uintptr_t entry_offset = sizeof(Multiboot2TagMmap);
          while (entry_offset + mmap_tag->entry_size <= mmap_tag->size) {
            const Multiboot2MmapEntry* const entry =
                reinterpret_cast<const Multiboot2MmapEntry*>(
                    reinterpret_cast<uintptr_t>(mmap_tag) + entry_offset);
            RecordMmapEntry(out_map, entry->addr, entry->len, entry->type);
            entry_offset += mmap_tag->entry_size;
          }
        }
      } else if (tag->type == kMultiboot2TagFramebuffer &&
                 tag->size >= sizeof(Multiboot2TagFramebuffer)) {
        const Multiboot2TagFramebuffer* const fb_tag =
            reinterpret_cast<const Multiboot2TagFramebuffer*>(tag);
        if (fb_tag->framebuffer_type == kMultiboot2FramebufferTypeRgb &&
            fb_tag->framebuffer_addr < kMaxCanonicalIdentityAddress) {
          out_map->fb_addr = static_cast<uintptr_t>(fb_tag->framebuffer_addr);
          out_map->fb_pitch = fb_tag->framebuffer_pitch;
          out_map->fb_width = fb_tag->framebuffer_width;
          out_map->fb_height = fb_tag->framebuffer_height;
          out_map->fb_bpp = fb_tag->framebuffer_bpp;
          out_map->fb_type = fb_tag->framebuffer_type;
        }
      }
      offset = AlignUp(offset + tag->size, 8);
    }
  } else if (multiboot_magic == kMultiboot1Magic) {
    if ((max_physical_addr - info_phys) < sizeof(Multiboot1Info) ||
        !PagingMapBootstrapRange(info_phys, sizeof(Multiboot1Info))) {
      return false;
    }
    const Multiboot1Info* const info =
        reinterpret_cast<const Multiboot1Info*>(info_phys);
    if ((info->flags & kMultiboot1FlagMmap) == 0 || info->mmap_addr == 0 ||
        info->mmap_length == 0 ||
        info->mmap_length > kMaxMultibootStructureBytes) {
      return false;
    }
    const uintptr_t mmap_phys = static_cast<uintptr_t>(info->mmap_addr);
    if (mmap_phys >= max_physical_addr ||
        (max_physical_addr - mmap_phys) < info->mmap_length ||
        !PagingMapBootstrapRange(mmap_phys, info->mmap_length)) {
      return false;
    }
    out_map->mb_reserved_start = info_phys;
    out_map->mb_reserved_end = info_phys + sizeof(Multiboot1Info);
    out_map->mb1_mmap_reserved_start = mmap_phys;
    out_map->mb1_mmap_reserved_end = mmap_phys + info->mmap_length;

    uintptr_t entry_offset = 0;
    while (entry_offset + sizeof(Multiboot1MmapEntry) <= info->mmap_length) {
      const Multiboot1MmapEntry* const entry =
          reinterpret_cast<const Multiboot1MmapEntry*>(mmap_phys +
                                                       entry_offset);
      if (entry->size < 20) {
        break;
      }
      RecordMmapEntry(out_map, entry->addr, entry->len, entry->type);
      entry_offset += entry->size + sizeof(uint32_t);
    }
  } else {
    return false;
  }

  return out_map->region_count > 0 && out_map->usable_ram_bytes > 0;
}

}  // namespace protos
