#include "pmm.h"

#include <cstddef>
#include <cstdint>

#include "multiboot.h"
#include "paging.h"
#include "uart.h"

extern "C" {
extern const uint8_t _kernel_start[];
extern const uint8_t _kernel_end[];
}

namespace protos {

namespace {

constexpr size_t kBitmapWordBits = 64;

// Dynamically placed bitmap where bit `frame_idx % 64` of
// `g_pmm_bitmap[frame_idx / 64]` is:
//   1 -> frame is USED / RESERVED
//   0 -> frame is FREE
uint64_t* g_pmm_bitmap = nullptr;
uintptr_t g_bitmap_phys_start = 0;
uintptr_t g_bitmap_phys_end = 0;
uintptr_t g_max_managed_phys_addr = 0;
size_t g_max_frames = 0;
size_t g_bitmap_words = 0;
size_t g_pmm_free_frames = 0;
size_t g_pmm_total_usable_frames = 0;
MultibootMemoryMap g_memory_map = {};

// Rounds `value` up to the nearest multiple of `alignment` (power of two).
static constexpr uintptr_t AlignUp(
    const uintptr_t value,     //
    const uintptr_t alignment  //
) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// Rounds `value` down to the nearest multiple of `alignment` (power of two).
static constexpr uintptr_t AlignDown(
    const uintptr_t value,     //
    const uintptr_t alignment  //
) {
  return value & ~(alignment - 1);
}

static bool PmmIsFrameUsed(const size_t frame_idx) {
  if (g_pmm_bitmap == nullptr || frame_idx >= g_max_frames) {
    return true;
  }
  const size_t word_idx = frame_idx / kBitmapWordBits;
  const size_t bit_idx = frame_idx % kBitmapWordBits;
  return (g_pmm_bitmap[word_idx] & (1ULL << bit_idx)) != 0;
}

static void PmmMarkFrameUsed(const size_t frame_idx) {
  if (g_pmm_bitmap == nullptr || frame_idx >= g_max_frames) {
    return;
  }
  const size_t word_idx = frame_idx / kBitmapWordBits;
  const size_t bit_idx = frame_idx % kBitmapWordBits;
  const uint64_t mask = 1ULL << bit_idx;
  if ((g_pmm_bitmap[word_idx] & mask) == 0) {
    g_pmm_bitmap[word_idx] |= mask;
    if (g_pmm_free_frames > 0) {
      --g_pmm_free_frames;
    }
  }
}

static void PmmMarkFrameFree(const size_t frame_idx) {
  if (g_pmm_bitmap == nullptr || frame_idx >= g_max_frames) {
    return;
  }
  const size_t word_idx = frame_idx / kBitmapWordBits;
  const size_t bit_idx = frame_idx % kBitmapWordBits;
  const uint64_t mask = 1ULL << bit_idx;
  if ((g_pmm_bitmap[word_idx] & mask) != 0) {
    g_pmm_bitmap[word_idx] &= ~mask;
    ++g_pmm_free_frames;
  }
}

// Marks only the complete 4 KiB frames strictly inside `[base, base + length)`
// as free (rounding start UP and end DOWN to page boundaries so partial edge
// frames are never freed).
static void PmmFreeRegionInterior(
    const uint64_t base,   //
    const uint64_t length  //
) {
  if (base >= g_max_managed_phys_addr || length < kPageSize) {
    return;
  }
  const uint64_t max_len = g_max_managed_phys_addr - base;
  const uint64_t clamped_len = (length > max_len) ? max_len : length;
  const uintptr_t start_addr = AlignUp(
      static_cast<uintptr_t>(base),  //
      kPageSize                      //
  );
  const uintptr_t end_addr = AlignDown(
      static_cast<uintptr_t>(base + clamped_len),  //
      kPageSize                                    //
  );
  if (start_addr >= end_addr) {
    return;
  }
  for (uintptr_t addr = start_addr; addr < end_addr; addr += kPageSize) {
    PmmMarkFrameFree(addr / kPageSize);
  }
}

// Marks all 4 KiB frames overlapping `[base, base + length)` as used (rounding
// start DOWN and end UP to page boundaries so any partially touched frame is
// conservatively reserved).
static void PmmReserveRegionOutward(
    const uint64_t base,   //
    const uint64_t length  //
) {
  if (length == 0 || base >= g_max_managed_phys_addr) {
    return;
  }
  const uint64_t max_len = g_max_managed_phys_addr - base;
  const uint64_t clamped_len = (length > max_len) ? max_len : length;
  const uintptr_t start_addr = AlignDown(
      static_cast<uintptr_t>(base),  //
      kPageSize                      //
  );
  const uintptr_t end_addr = AlignUp(
      static_cast<uintptr_t>(base + clamped_len),  //
      kPageSize                                    //
  );
  for (uintptr_t addr = start_addr; addr < end_addr; addr += kPageSize) {
    PmmMarkFrameUsed(addr / kPageSize);
  }
}

// Checks whether `[addr, addr + size)` lies inside a usable RAM region within
// the initial bootstrap identity map (`[kLowerMemoryLimit, kBootstrapIdentityMapSize)`)
// and does not overlap the kernel image or Multiboot structures.
static bool IsRangeUsableForBitmap(
    const uintptr_t addr,  //
    const size_t size      //
) {
  if (size == 0 ||
      addr < kLowerMemoryLimit ||
      addr >= kBootstrapIdentityMapSize ||
      (kBootstrapIdentityMapSize - addr) < size) {
    return false;
  }

  const uintptr_t end_addr = addr + size;
  const uintptr_t kernel_start = reinterpret_cast<uintptr_t>(_kernel_start);
  const uintptr_t kernel_end = reinterpret_cast<uintptr_t>(_kernel_end);

  if (addr < kernel_end && end_addr > kernel_start) {
    return false;
  }
  if (g_memory_map.mb_reserved_end > g_memory_map.mb_reserved_start &&
      addr < g_memory_map.mb_reserved_end &&
      end_addr > g_memory_map.mb_reserved_start) {
    return false;
  }
  if (g_memory_map.mb1_mmap_reserved_end >
          g_memory_map.mb1_mmap_reserved_start &&
      addr < g_memory_map.mb1_mmap_reserved_end &&
      end_addr > g_memory_map.mb1_mmap_reserved_start) {
    return false;
  }

  bool inside_available = false;
  for (size_t i = 0; i < g_memory_map.region_count; ++i) {
    const uint64_t reg_start = g_memory_map.regions[i].base;
    const uint64_t reg_end = reg_start + g_memory_map.regions[i].length;
    if (g_memory_map.regions[i].type == kMemoryTypeAvailable) {
      if (addr >= reg_start && end_addr <= reg_end) {
        inside_available = true;
      }
    } else {
      if (addr < reg_end && end_addr > reg_start) {
        return false;
      }
    }
  }
  return inside_available;
}

// Finds a page-aligned physical address range of `bitmap_bytes` bytes within
// the bootstrap-mapped low memory to store `g_pmm_bitmap`.
static uintptr_t FindBitmapPhysicalAddress(const size_t bitmap_bytes) {
  for (size_t i = 0; i < g_memory_map.region_count; ++i) {
    if (g_memory_map.regions[i].type != kMemoryTypeAvailable) {
      continue;
    }
    const uint64_t raw_start = g_memory_map.regions[i].base;
    const uint64_t raw_end = raw_start + g_memory_map.regions[i].length;
    if (raw_end <= kLowerMemoryLimit ||
        raw_start >= kBootstrapIdentityMapSize) {
      continue;
    }
    const uintptr_t clamped_start = static_cast<uintptr_t>(
        (raw_start < kLowerMemoryLimit) ? kLowerMemoryLimit : raw_start);
    const uintptr_t clamped_end = static_cast<uintptr_t>(
        (raw_end > kBootstrapIdentityMapSize)
            ? kBootstrapIdentityMapSize
            : raw_end);
    const uintptr_t aligned_start = AlignUp(
        clamped_start,  //
        kPageSize       //
    );
    const uintptr_t aligned_end = AlignDown(
        clamped_end,  //
        kPageSize     //
    );
    if (aligned_start >= aligned_end ||
        (aligned_end - aligned_start) < bitmap_bytes) {
      continue;
    }
    for (uintptr_t cand = aligned_start;
         cand <= aligned_end - bitmap_bytes;
         cand += kPageSize) {
      if (IsRangeUsableForBitmap(
              cand,         //
              bitmap_bytes  //
          )) {
        return cand;
      }
    }
  }
  return 0;
}

}  // namespace

bool PmmInit(
    const uint32_t multiboot_magic,     //
    const uint64_t multiboot_info_addr  //
) {
  const bool parsed_ok = MultibootParseMemoryMap(
      multiboot_magic,            //
      multiboot_info_addr,        //
      kBootstrapIdentityMapSize,  //
      &g_memory_map               //
  );
  if (!parsed_ok) {
    return false;
  }

  uintptr_t highest_usable_addr = 0;
  for (size_t i = 0; i < g_memory_map.region_count; ++i) {
    if (g_memory_map.regions[i].type == kMemoryTypeAvailable) {
      const uintptr_t region_end = AlignDown(
          static_cast<uintptr_t>(
              g_memory_map.regions[i].base + g_memory_map.regions[i].length),  //
          kPageSize                                                            //
      );
      if (region_end > highest_usable_addr) {
        highest_usable_addr = region_end;
      }
    }
  }
  if (highest_usable_addr <= kLowerMemoryLimit) {
    return false;
  }

  g_max_managed_phys_addr = highest_usable_addr;
  g_max_frames = g_max_managed_phys_addr / kPageSize;
  g_bitmap_words =
      (g_max_frames + kBitmapWordBits - 1) / kBitmapWordBits;
  const size_t bitmap_bytes = AlignUp(
      g_bitmap_words * sizeof(uint64_t),  //
      kPageSize                           //
  );

  const uintptr_t bitmap_phys = FindBitmapPhysicalAddress(bitmap_bytes);
  if (bitmap_phys == 0) {
    return false;
  }

  g_bitmap_phys_start = bitmap_phys;
  g_bitmap_phys_end = bitmap_phys + bitmap_bytes;
  g_pmm_bitmap = reinterpret_cast<uint64_t*>(bitmap_phys);

  for (size_t i = 0; i < g_bitmap_words; ++i) {
    g_pmm_bitmap[i] = ~uint64_t{0};
  }

  for (size_t i = 0; i < g_memory_map.region_count; ++i) {
    if (g_memory_map.regions[i].type == kMemoryTypeAvailable) {
      PmmFreeRegionInterior(
          g_memory_map.regions[i].base,   //
          g_memory_map.regions[i].length  //
      );
    }
  }

  for (size_t i = 0; i < g_memory_map.region_count; ++i) {
    if (g_memory_map.regions[i].type != kMemoryTypeAvailable) {
      PmmReserveRegionOutward(
          g_memory_map.regions[i].base,   //
          g_memory_map.regions[i].length  //
      );
    }
  }

  PmmReserveRegionOutward(
      0,                 //
      kLowerMemoryLimit  //
  );

  const uintptr_t kernel_start = reinterpret_cast<uintptr_t>(_kernel_start);
  const uintptr_t kernel_end = reinterpret_cast<uintptr_t>(_kernel_end);
  PmmReserveRegionOutward(
      kernel_start,              //
      kernel_end - kernel_start  //
  );

  PmmReserveRegionOutward(
      g_bitmap_phys_start,                       //
      g_bitmap_phys_end - g_bitmap_phys_start    //
  );

  if (g_memory_map.mb_reserved_end > g_memory_map.mb_reserved_start) {
    PmmReserveRegionOutward(
        g_memory_map.mb_reserved_start,                                //
        g_memory_map.mb_reserved_end - g_memory_map.mb_reserved_start  //
    );
  }
  if (g_memory_map.mb1_mmap_reserved_end >
      g_memory_map.mb1_mmap_reserved_start) {
    PmmReserveRegionOutward(
        g_memory_map.mb1_mmap_reserved_start,     //
        g_memory_map.mb1_mmap_reserved_end -      //
            g_memory_map.mb1_mmap_reserved_start  //
    );
  }

  if (!PagingExtendIdentityMap(g_max_managed_phys_addr)) {
    return false;
  }

  g_pmm_total_usable_frames = g_pmm_free_frames;

  UartWrite("[PMM] Kernel range: ");
  UartWriteHex(kernel_start);
  UartWrite(" .. ");
  UartWriteHex(kernel_end);
  UartWrite(", Bitmap: ");
  UartWriteHex(g_bitmap_phys_start);
  UartWrite(" .. ");
  UartWriteHex(g_bitmap_phys_end);
  UartWrite(", Identity-mapped: 0x0 .. ");
  UartWriteHex(PagingIdentityMappedLimit());
  UartWrite("\n");

  UartWrite("[PMM] Total RAM: ");
  UartWriteDec(g_memory_map.total_ram_bytes / 1024);
  UartWrite(" KiB, Usable RAM: ");
  UartWriteDec(g_memory_map.usable_ram_bytes / 1024);
  UartWrite(" KiB, Reserved RAM: ");
  UartWriteDec(g_memory_map.reserved_ram_bytes / 1024);
  UartWrite(" KiB, Free 4KiB frames: ");
  UartWriteDec(g_pmm_free_frames);
  UartWrite("\n");

  return g_pmm_free_frames > 0;
}

uintptr_t PmmAllocFrames(const size_t count) {
  if (g_pmm_bitmap == nullptr || count == 0 || count > g_pmm_free_frames) {
    return 0;
  }

  const size_t min_frame = kLowerMemoryLimit / kPageSize;
  size_t run_start = min_frame;
  size_t run_length = 0;

  size_t frame_idx = min_frame;
  while (frame_idx < g_max_frames) {
    if (run_length == 0 && (frame_idx % kBitmapWordBits) == 0) {
      const size_t word_idx = frame_idx / kBitmapWordBits;
      if (g_pmm_bitmap[word_idx] == ~uint64_t{0}) {
        frame_idx += kBitmapWordBits;
        run_start = frame_idx;
        continue;
      }
    }

    if (!PmmIsFrameUsed(frame_idx)) {
      if (run_length == 0) {
        run_start = frame_idx;
      }
      ++run_length;
      if (run_length == count) {
        for (size_t i = 0; i < count; ++i) {
          PmmMarkFrameUsed(run_start + i);
        }
        return run_start * kPageSize;
      }
    } else {
      run_length = 0;
    }
    ++frame_idx;
  }

  return 0;
}

uintptr_t PmmAllocFrame() {
  return PmmAllocFrames(1);
}

bool PmmRangeIsValidUsableRam(
    const uintptr_t addr,  //
    const size_t size      //
) {
  if (size == 0 ||
      addr < kLowerMemoryLimit ||
      addr >= g_max_managed_phys_addr ||
      (g_max_managed_phys_addr - addr) < size) {
    return false;
  }

  const uintptr_t end_addr = addr + size;
  const uintptr_t kernel_start = reinterpret_cast<uintptr_t>(_kernel_start);
  const uintptr_t kernel_end = reinterpret_cast<uintptr_t>(_kernel_end);

  if (addr < kernel_end && end_addr > kernel_start) {
    return false;
  }
  if (g_bitmap_phys_end > g_bitmap_phys_start &&
      addr < g_bitmap_phys_end &&
      end_addr > g_bitmap_phys_start) {
    return false;
  }
  if (g_memory_map.mb_reserved_end > g_memory_map.mb_reserved_start &&
      addr < g_memory_map.mb_reserved_end &&
      end_addr > g_memory_map.mb_reserved_start) {
    return false;
  }
  if (g_memory_map.mb1_mmap_reserved_end >
          g_memory_map.mb1_mmap_reserved_start &&
      addr < g_memory_map.mb1_mmap_reserved_end &&
      end_addr > g_memory_map.mb1_mmap_reserved_start) {
    return false;
  }

  bool inside_available = false;
  for (size_t i = 0; i < g_memory_map.region_count; ++i) {
    const uint64_t reg_start = g_memory_map.regions[i].base;
    const uint64_t reg_end = reg_start + g_memory_map.regions[i].length;
    if (g_memory_map.regions[i].type == kMemoryTypeAvailable) {
      if (addr >= reg_start && end_addr <= reg_end) {
        inside_available = true;
      }
    } else {
      if (addr < reg_end && end_addr > reg_start) {
        return false;
      }
    }
  }
  return inside_available;
}

void PmmFreeFrames(
    const uintptr_t base_addr,  //
    const size_t count          //
) {
  if (base_addr == 0 ||
      (base_addr & (kPageSize - 1)) != 0 ||
      base_addr < kLowerMemoryLimit ||
      base_addr >= g_max_managed_phys_addr ||
      count == 0) {
    return;
  }

  const size_t start_frame = base_addr / kPageSize;

  for (size_t i = 0; i < count; ++i) {
    const size_t frame_idx = start_frame + i;
    if (frame_idx >= g_max_frames) {
      break;
    }
    const uintptr_t addr = frame_idx * kPageSize;
    if (!PmmRangeIsValidUsableRam(
            addr,      //
            kPageSize  //
        )) {
      continue;
    }
    if (PmmIsFrameUsed(frame_idx)) {
      PmmMarkFrameFree(frame_idx);
    }
  }
}

void PmmFreeFrame(const uintptr_t frame_addr) {
  PmmFreeFrames(
      frame_addr,  //
      1            //
  );
}

uintptr_t PmmMaxPhysicalAddress() {
  return g_max_managed_phys_addr;
}

size_t PmmMaxFrameCount() {
  return g_max_frames;
}

size_t PmmFreeFrameCount() {
  return g_pmm_free_frames;
}

size_t PmmTotalUsableFrameCount() {
  return g_pmm_total_usable_frames;
}

}  // namespace protos
