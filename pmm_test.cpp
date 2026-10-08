#include "pmm.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "multiboot.h"
#include "paging.h"
#include "uart.h"
#include "vga.h"

extern "C" {
extern const uint8_t g_kernel_start[];
extern const uint8_t g_kernel_end[];
alignas(protos::kPageSize) uint8_t
    g_fake_ram[4096 * protos::kPageSize];  // 16 MiB
}

// Place the simulated kernel image at frames [16, 20) inside `g_fake_ram` so
// that pages both below `g_kernel_start` and at or above `g_kernel_end` are
// backed by writable host memory during region-carving tests.
asm(".globl g_kernel_start\n"
    ".set g_kernel_start, g_fake_ram + (16 * 4096)\n"
    ".globl g_kernel_end\n"
    ".set g_kernel_end, g_fake_ram + (20 * 4096)\n");

namespace protos {
namespace {

namespace t = ::testing;

constexpr int64_t kFakeRamFrames = 4096;
constexpr int64_t kFakeRamBytes = kFakeRamFrames * kPageSize;

struct FakeEnvState {
  MultibootMemoryMap map = {};
  bool parse_ok = true;
  bool fail_extend_map = false;
  int extend_map_calls = 0;
  uintptr_t last_extend_max_phys = 0;
  uintptr_t identity_mapped_limit = kBootstrapIdentityMapSize;
  int64_t free_frames_during_extend = -1;
  bool fake_ram_clean_during_extend = false;
  int check_clean_frames = 0;
  bool fb_attached = false;
  uintptr_t attached_fb_addr = 0;
  int attached_fb_pitch = 0;
  int attached_fb_width = 0;
  int attached_fb_height = 0;
  int attached_fb_bpp = 0;
  std::string uart_log;
  std::string vga_log;
};

FakeEnvState g_env;

static uintptr_t FakeRamBase() {
  return reinterpret_cast<uintptr_t>(g_fake_ram);
}

static uintptr_t FrameAddr(const int64_t frame_index) {
  return FakeRamBase() + frame_index * kPageSize;
}

static void ResetFakeEnv(const int64_t zero_frames = 256) {
  g_env = FakeEnvState{};
  const int64_t bytes_to_zero = (zero_frames <= kFakeRamFrames)
                                    ? (zero_frames * kPageSize)
                                    : kFakeRamBytes;
  if (bytes_to_zero > 0) {
    std::memset(g_fake_ram, 0, bytes_to_zero);
  }
}

static void AddRegion(const uintptr_t base,  //
                      const int64_t length,  //
                      const uint32_t type) {
  const int idx = g_env.map.region_count;
  g_env.map.regions[idx] = {base, length, type};
  ++g_env.map.region_count;
  g_env.map.total_ram_bytes += length;
  if (type == kMemoryTypeAvailable) {
    g_env.map.usable_ram_bytes += length;
  } else {
    g_env.map.reserved_ram_bytes += length;
  }
}

}  // namespace

bool MultibootParseMemoryMap(const uint32_t multiboot_magic,
                             const uint64_t multiboot_info_addr,
                             const uintptr_t max_physical_addr,
                             MultibootMemoryMap* const out_map) {
  if (multiboot_magic != 0 || multiboot_info_addr != 0) {
    return MultibootParseMemoryMapFromBuffer(multiboot_magic,      //
                                             multiboot_info_addr,  //
                                             max_physical_addr,    //
                                             out_map);
  }
  if (!g_env.parse_ok || out_map == nullptr) {
    return false;
  }
  *out_map = g_env.map;
  return true;
}

bool PagingMapBootstrapRange(const uintptr_t phys_addr, const int64_t size) {
  (void)phys_addr;
  (void)size;
  return true;
}

bool PagingExtendIdentityMap(const uintptr_t max_physical_addr) {
  ++g_env.extend_map_calls;
  g_env.last_extend_max_phys = max_physical_addr;
  g_env.free_frames_during_extend = PmmFreeFrameCount();

  if (g_env.check_clean_frames > 0) {
    bool all_zero = true;
    const int total_bytes = g_env.check_clean_frames * kPageSize;
    for (int i = 0; i < total_bytes; ++i) {
      if (g_fake_ram[i] != 0) {
        all_zero = false;
        break;
      }
    }
    g_env.fake_ram_clean_during_extend = all_zero;
  }

  if (g_env.fail_extend_map) {
    return false;
  }
  g_env.identity_mapped_limit =
      (max_physical_addr + kHugePageSize - 1) & ~(kHugePageSize - 1);
  return true;
}

uintptr_t PagingIdentityMappedLimit() { return g_env.identity_mapped_limit; }

void UartWrite(const char* const str) { g_env.uart_log.append(str); }

void UartWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llx",     //
                static_cast<unsigned long long>(value));
  g_env.uart_log.append(buf);
}

void UartWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  g_env.uart_log.append(buf);
}

void VgaWrite(const char* const str) { g_env.vga_log.append(str); }

void VgaWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llx",     //
                static_cast<unsigned long long>(value));
  g_env.vga_log.append(buf);
}

void VgaWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  g_env.vga_log.append(buf);
}

void VgaAttachFramebuffer(const uintptr_t fb_phys_addr,  //
                          const int pitch,               //
                          const int width,               //
                          const int height,              //
                          const int bpp) {
  g_env.fb_attached = true;
  g_env.attached_fb_addr = fb_phys_addr;
  g_env.attached_fb_pitch = pitch;
  g_env.attached_fb_width = width;
  g_env.attached_fb_height = height;
  g_env.attached_fb_bpp = bpp;
}

namespace {

TEST(PmmTest, InitFailureModesAndEmptyRegions) {
  // Calling PmmAllocFrame before PmmInit succeeds must trigger DCHECK in debug
  // builds and return 0 under NDEBUG.
  ResetFakeEnv();
#ifdef NDEBUG
  EXPECT_THAT(PmmAllocFrame(), t::Eq(0u));
#else
  EXPECT_DEATH(PmmAllocFrame(), "Check failed");
#endif

  // MultibootParseMemoryMap failure must cause PmmInit to panic via CHECK.
  ResetFakeEnv();
  g_env.parse_ok = false;
  EXPECT_DEATH(PmmInit(0, 0), "Check failed");

  // Available RAM strictly below kLowerMemoryLimit (1 MiB) is completely
  // excluded and must cause PmmInit to panic via CHECK.
  ResetFakeEnv();
  AddRegion(0, kLowerMemoryLimit, kMemoryTypeAvailable);
  EXPECT_DEATH(PmmInit(0, 0), "Check failed");

  // Sub-page available fragments (< kPageSize) or unaligned fragments that
  // contain no complete page must be rejected via CHECK.
  ResetFakeEnv();
  AddRegion(FrameAddr(20) + 1024, 2048, kMemoryTypeAvailable);
  EXPECT_DEATH(PmmInit(0, 0), "Check failed");

  // Available region completely covered by a non-available reserved region
  // leaves zero usable frames and must panic via CHECK.
  ResetFakeEnv();
  AddRegion(FrameAddr(20), 8 * kPageSize, kMemoryTypeAvailable);
  AddRegion(FrameAddr(20), 8 * kPageSize, 2);
  EXPECT_DEATH(PmmInit(0, 0), "Check failed");

  // Failure in PagingExtendIdentityMap must cause PmmInit to panic via CHECK.
  ResetFakeEnv(64);
  g_env.fail_extend_map = true;
  AddRegion(FrameAddr(20), 16 * kPageSize, kMemoryTypeAvailable);
  EXPECT_DEATH(PmmInit(0, 0), "Check failed");
}

TEST(PmmTest, InitCarvesReservedKernelMultibootAndFramebuffer) {
  ResetFakeEnv(140);
  g_env.check_clean_frames = 140;

  // Available region spanning frames [0, 128) with unaligned sub-page edges on
  // both ends that must be rounded inward to [FrameAddr(0), FrameAddr(128)).
  AddRegion(FrameAddr(0) - 512, 128 * kPageSize + 1024, kMemoryTypeAvailable);

  // Non-available region inside frames [30, 34) with unaligned edges that must
  // be rounded outward to carve out frames [30, 34).
  AddRegion(FrameAddr(30) + 256, 3 * kPageSize, 2);

  // Trailing non-available region covering frames [120, 128) so that
  // PmmMaxPhysicalAddress() must be clamped downward to FrameAddr(120).
  AddRegion(FrameAddr(120), 8 * kPageSize, 2);

  // Unaligned Multiboot info reservation touching frames 45 and 46 -> [45, 47).
  g_env.map.mb_reserved_start = FrameAddr(45) + 100;
  g_env.map.mb_reserved_end = FrameAddr(46) + 200;

  // Unaligned Multiboot1 mmap reservation inside frame 60 -> [60, 61).
  g_env.map.mb1_mmap_reserved_start = FrameAddr(60) + 16;
  g_env.map.mb1_mmap_reserved_end = FrameAddr(60) + 512;

  // Framebuffer at frame 80 spanning 3 frames [80, 83).
  g_env.map.fb_addr = FrameAddr(80);
  g_env.map.fb_pitch = 2048;
  g_env.map.fb_width = 512;
  g_env.map.fb_height = 6;
  g_env.map.fb_bpp = 32;

  PmmInit(0, 0);

  // Verify framebuffer attachment and dual UART + VGA console logging.
  EXPECT_TRUE(g_env.fb_attached);
  EXPECT_THAT(g_env.attached_fb_addr, t::Eq(FrameAddr(80)));
  EXPECT_THAT(g_env.attached_fb_pitch, t::Eq(2048));
  EXPECT_THAT(g_env.attached_fb_width, t::Eq(512));
  EXPECT_THAT(g_env.attached_fb_height, t::Eq(6));
  EXPECT_THAT(g_env.attached_fb_bpp, t::Eq(32));
  EXPECT_THAT(g_env.uart_log, t::HasSubstr("[PMM] Framebuffer: addr="));
  EXPECT_THAT(g_env.uart_log, t::HasSubstr("[PMM] Kernel range: "));
  EXPECT_THAT(g_env.uart_log, t::HasSubstr("[PMM] Total RAM: "));
  EXPECT_THAT(g_env.vga_log, t::Eq(g_env.uart_log));

  // Verify PagingExtendIdentityMap was called with the clamped max address
  // BEFORE any FreeRunNode was written into high memory (`g_fake_ram`).
  EXPECT_THAT(g_env.extend_map_calls, t::Eq(1));
  EXPECT_THAT(g_env.last_extend_max_phys, t::Eq(FrameAddr(120)));
  EXPECT_THAT(g_env.free_frames_during_extend, t::Eq(0));
  EXPECT_TRUE(g_env.fake_ram_clean_during_extend);

  // Expected surviving usable intervals:
  //   [0, 16)   -> 16 frames (before kernel [16, 20))
  //   [20, 30)  -> 10 frames (between kernel and E820 reserved [30, 34))
  //   [34, 45)  -> 11 frames (before mb_reserved [45, 47))
  //   [47, 60)  -> 13 frames (before mb1_mmap_reserved [60, 61))
  //   [61, 80)  -> 19 frames (before framebuffer [80, 83))
  //   [83, 120) -> 37 frames (before trailing E820 reserved [120, 128))
  // Total = 16 + 10 + 11 + 13 + 19 + 37 = 106 frames.
  constexpr int64_t kExpectedTotalFrames = 106;
  EXPECT_THAT(PmmMaxPhysicalAddress(), t::Eq(FrameAddr(120)));
  EXPECT_THAT(PmmMaxFrameCount(), t::Eq(FrameAddr(120) / kPageSize));
  EXPECT_THAT(PmmTotalUsableFrameCount(), t::Eq(kExpectedTotalFrames));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(kExpectedTotalFrames));

  struct IntervalRange {
    int start_frame;
    int end_frame;
  };
  constexpr IntervalRange kExpectedRanges[] = {
      {0, 16}, {20, 30}, {34, 45}, {47, 60}, {61, 80}, {83, 120},
  };
  constexpr int kNumRanges =
      sizeof(kExpectedRanges) / sizeof(kExpectedRanges[0]);

  for (int r = 0; r < kNumRanges; ++r) {
    for (int f = kExpectedRanges[r].start_frame;
         f < kExpectedRanges[r].end_frame; ++f) {
      const uintptr_t addr = PmmAllocFrame();
      ASSERT_THAT(addr, t::Eq(FrameAddr(f)));
    }
  }
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));
  EXPECT_THAT(PmmAllocFrame(), t::Eq(0));
}

TEST(PmmTest, InitNormalizesOverlappingAndUnsortedAvailableRegions) {
  ResetFakeEnv(100);

  // Add unsorted, overlapping, and abutting available regions in [20, 60):
  //   [40, 60), [24, 35), [30, 40), [20, 24)
  AddRegion(FrameAddr(40), 20 * kPageSize, kMemoryTypeAvailable);
  AddRegion(FrameAddr(24), 11 * kPageSize, kMemoryTypeAvailable);
  AddRegion(FrameAddr(30), 10 * kPageSize, kMemoryTypeAvailable);
  AddRegion(FrameAddr(20), 4 * kPageSize, kMemoryTypeAvailable);

  PmmInit(0, 0);
  EXPECT_THAT(PmmTotalUsableFrameCount(), t::Eq(40));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(40));
  EXPECT_THAT(PmmMaxPhysicalAddress(), t::Eq(FrameAddr(60)));

  // Because all four regions merged into a single contiguous interval [20, 60),
  // allocating all 40 frames in one contiguous request must succeed.
  const uintptr_t base = PmmAllocFrames(40);
  EXPECT_THAT(base, t::Eq(FrameAddr(20)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));
  PmmFreeFrames(base, 40);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(40));
}

TEST(PmmTest, LowestAddressFirstFitAndFindFirstAugmentedSelection) {
  ResetFakeEnv(120);
  AddRegion(FrameAddr(20), 60 * kPageSize, kMemoryTypeAvailable);
  PmmInit(0, 0);

  // Carve 4 free holes at ascending addresses separated by 1-frame barriers:
  //   run0 (2 frames) < run1 (8 frames) < run2 (32 frames) < run3 (8 frames)
  const uintptr_t run0 = PmmAllocFrames(2);
  const uintptr_t bar0 = PmmAllocFrame();
  const uintptr_t run1 = PmmAllocFrames(8);
  const uintptr_t bar1 = PmmAllocFrame();
  const uintptr_t run2 = PmmAllocFrames(32);
  const uintptr_t bar2 = PmmAllocFrame();
  const uintptr_t run3 = PmmAllocFrames(8);
  const uintptr_t tail = PmmAllocFrames(PmmFreeFrameCount());
  ASSERT_THAT(run0, t::Eq(FrameAddr(20)));
  ASSERT_THAT(bar0, t::Eq(FrameAddr(22)));
  ASSERT_THAT(run1, t::Eq(FrameAddr(23)));
  ASSERT_THAT(bar1, t::Eq(FrameAddr(31)));
  ASSERT_THAT(run2, t::Eq(FrameAddr(32)));
  ASSERT_THAT(bar2, t::Eq(FrameAddr(64)));
  ASSERT_THAT(run3, t::Eq(FrameAddr(65)));
  ASSERT_THAT(tail, t::Eq(FrameAddr(73)));
  ASSERT_THAT(PmmFreeFrameCount(), t::Eq(0));

  PmmFreeFrames(run0, 2);
  PmmFreeFrames(run1, 8);
  PmmFreeFrames(run2, 32);
  PmmFreeFrames(run3, 8);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(50));

  // Request 4 frames: run0 (2 frames) is too small; run1 (8 frames) is the
  // lowest-address qualifying run and must be chosen over larger run2 (32)
  // and equal higher-address run3 (8).
  const uintptr_t pick1 = PmmAllocFrames(4);
  EXPECT_THAT(pick1, t::Eq(run1));

  // Request 8 frames: run1's remainder now has only 4 frames; run2 (32 frames)
  // and run3 (8 frames) both qualify. Lowest-address first-fit must select
  // run2 over exact-fit run3.
  const uintptr_t pick2 = PmmAllocFrames(8);
  EXPECT_THAT(pick2, t::Eq(run2));

  // run2 was split into 8 allocated frames + 24 free frames at `run2 + 8
  // pages`. Requesting 16 frames must select the 24-frame remainder of run2.
  const uintptr_t pick3 = PmmAllocFrames(16);
  EXPECT_THAT(pick3, t::Eq(run2 + 8 * kPageSize));

  // Request 4 frames: run1's 4-frame remainder at `run1 + 4 pages` is the
  // lowest-address run with >= 4 frames.
  const uintptr_t pick4 = PmmAllocFrames(4);
  EXPECT_THAT(pick4, t::Eq(run1 + 4 * kPageSize));

  // Request 8 frames: run2's remaining 8 frames at `run2 + 24 pages` is lower
  // in address than run3 (8 frames) and must be selected first.
  const uintptr_t pick5 = PmmAllocFrames(8);
  EXPECT_THAT(pick5, t::Eq(run2 + 24 * kPageSize));

  // Next 8-frame request selects run3.
  const uintptr_t pick6 = PmmAllocFrames(8);
  EXPECT_THAT(pick6, t::Eq(run3));

  // Finally, a 2-frame request selects run0 at the lowest address.
  const uintptr_t pick7 = PmmAllocFrames(2);
  EXPECT_THAT(pick7, t::Eq(run0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));

  PmmFreeFrames(pick1, 4);
  PmmFreeFrames(pick2, 8);
  PmmFreeFrames(pick3, 16);
  PmmFreeFrames(pick4, 4);
  PmmFreeFrames(pick5, 8);
  PmmFreeFrames(pick6, 8);
  PmmFreeFrames(pick7, 2);
  PmmFreeFrame(bar0);
  PmmFreeFrame(bar1);
  PmmFreeFrame(bar2);
  PmmFreeFrames(tail, 7);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(60));
}

TEST(PmmTest, RunSplittingAndExactFitWithoutPerFrameLoops) {
  ResetFakeEnv(64);
  AddRegion(FrameAddr(20), 20 * kPageSize, kMemoryTypeAvailable);
  PmmInit(0, 0);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(20));

  const uintptr_t a1 = PmmAllocFrame();
  EXPECT_THAT(a1, t::Eq(FrameAddr(20)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(19));

  const uintptr_t a2 = PmmAllocFrames(3);
  EXPECT_THAT(a2, t::Eq(FrameAddr(21)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(16));

  const uintptr_t a3 = PmmAllocFrames(10);
  EXPECT_THAT(a3, t::Eq(FrameAddr(24)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(6));

  // Exact-fit allocation of the remaining 6 frames removes the run completely.
  const uintptr_t a4 = PmmAllocFrames(6);
  EXPECT_THAT(a4, t::Eq(FrameAddr(34)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));
  EXPECT_THAT(PmmAllocFrame(), t::Eq(0));

  // Verify payload isolation across all allocated frames.
  for (int i = 0; i < 20; ++i) {
    uint64_t* const page_word = reinterpret_cast<uint64_t*>(FrameAddr(20 + i));
    *page_word = 0xA5A5000000000000ULL | i;
  }
  for (int i = 0; i < 20; ++i) {
    const uint64_t* const page_word =
        reinterpret_cast<const uint64_t*>(FrameAddr(20 + i));
    EXPECT_THAT(*page_word, t::Eq(0xA5A5000000000000ULL | i));
  }

  PmmFreeFrame(a1);
  PmmFreeFrames(a2, 3);
  PmmFreeFrames(a3, 10);
  PmmFreeFrames(a4, 6);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(20));
}

TEST(PmmTest, CoalescingForwardBackwardAndThreeWay) {
  ResetFakeEnv(64);
  AddRegion(FrameAddr(20), 10 * kPageSize, kMemoryTypeAvailable);
  PmmInit(0, 0);

  const uintptr_t a = PmmAllocFrames(2);
  const uintptr_t b = PmmAllocFrames(3);
  const uintptr_t c = PmmAllocFrames(4);
  const uintptr_t barrier = PmmAllocFrame();
  ASSERT_THAT(a, t::Eq(FrameAddr(20)));
  ASSERT_THAT(b, t::Eq(FrameAddr(22)));
  ASSERT_THAT(c, t::Eq(FrameAddr(25)));
  ASSERT_THAT(barrier, t::Eq(FrameAddr(29)));
  ASSERT_THAT(PmmFreeFrameCount(), t::Eq(0));

  // Backward coalescing: free `a` (2 frames), then free `b` (3 frames) ->
  // `b` merges into predecessor `a` to form a 5-frame run at `a`.
  PmmFreeFrames(a, 2);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(2));
  PmmFreeFrames(b, 3);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(5));

  // The absorbed node header at `b` must have its `frame_count` cleared to 0.
  const uint64_t* const b_header = reinterpret_cast<const uint64_t*>(b);
  EXPECT_THAT(*b_header, t::Eq(0));

  const uintptr_t ab = PmmAllocFrames(5);
  EXPECT_THAT(ab, t::Eq(a));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));

  // Forward coalescing: free `b` first, then free `a` -> `a` merges forward
  // with successor `b` to form a 5-frame run at `a`.
  PmmFreeFrames(b, 3);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(3));
  PmmFreeFrames(a, 2);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(5));
  EXPECT_THAT(*b_header, t::Eq(0));

  const uintptr_t ab2 = PmmAllocFrames(5);
  EXPECT_THAT(ab2, t::Eq(a));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));

  // Three-way coalescing: free `a` (2 frames) and `c` (4 frames) first, then
  // free middle run `b` (3 frames) -> `a + b + c` merge into a single 9-frame
  // run at `a`.
  PmmFreeFrames(a, 2);
  PmmFreeFrames(c, 4);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(6));
  EXPECT_THAT(PmmAllocFrames(5), t::Eq(0));

  PmmFreeFrames(b, 3);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(9));
  const uint64_t* const c_header = reinterpret_cast<const uint64_t*>(c);
  EXPECT_THAT(*b_header, t::Eq(0));
  EXPECT_THAT(*c_header, t::Eq(0));

  const uintptr_t abc = PmmAllocFrames(9);
  EXPECT_THAT(abc, t::Eq(a));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));

  PmmFreeFrames(abc, 9);
  PmmFreeFrame(barrier);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(10));
  const uintptr_t all_ten = PmmAllocFrames(10);
  EXPECT_THAT(all_ten, t::Eq(FrameAddr(20)));
  PmmFreeFrames(all_ten, 10);
}

TEST(PmmTest, NonContiguousRegionsNeverCoalesceAcrossReservedHoles) {
  ResetFakeEnv(64);
  // Region [12, 30) straddles the kernel image [16, 20) and an E820 reserved
  // frame at [24, 25), producing three disjoint usable intervals:
  //   [12, 16) (4 frames), [20, 24) (4 frames), [25, 30) (5 frames).
  AddRegion(FrameAddr(12), 18 * kPageSize, kMemoryTypeAvailable);
  AddRegion(FrameAddr(24), kPageSize, 2);
  PmmInit(0, 0);
  EXPECT_THAT(PmmTotalUsableFrameCount(), t::Eq(13));

  const uintptr_t r1 = PmmAllocFrames(4);
  const uintptr_t r2 = PmmAllocFrames(4);
  const uintptr_t r3 = PmmAllocFrames(5);
  ASSERT_THAT(r1, t::Eq(FrameAddr(12)));
  ASSERT_THAT(r2, t::Eq(FrameAddr(20)));
  ASSERT_THAT(r3, t::Eq(FrameAddr(25)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));

  // Free all three regions back into the tree. Because reserved holes lie
  // between them, they must remain separate runs and never coalesce.
  PmmFreeFrames(r1, 4);
  PmmFreeFrames(r2, 4);
  PmmFreeFrames(r3, 5);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(13));
  EXPECT_THAT(PmmAllocFrames(6), t::Eq(0));
}

TEST(PmmTest, OomHandling) {
  ResetFakeEnv(64);
  AddRegion(FrameAddr(20), 8 * kPageSize, kMemoryTypeAvailable);
  PmmInit(0, 0);

  // Request exceeding total free frames must return 0 immediately.
  EXPECT_THAT(PmmAllocFrames(9), t::Eq(0));
  EXPECT_THAT(PmmAllocFrames(PmmMaxFrameCount() + 1), t::Eq(0));

  // Fragment the 8 frames into two 3-frame free runs separated by a 2-frame
  // allocated barrier: total free frames = 6, max contiguous run = 3.
  const uintptr_t low_run = PmmAllocFrames(3);
  const uintptr_t mid_bar = PmmAllocFrames(2);
  const uintptr_t high_run = PmmAllocFrames(3);
  ASSERT_THAT(low_run, t::Ne(0));
  ASSERT_THAT(mid_bar, t::Ne(0));
  ASSERT_THAT(high_run, t::Ne(0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));
  EXPECT_THAT(PmmAllocFrame(), t::Eq(0));

  PmmFreeFrames(low_run, 3);
  PmmFreeFrames(high_run, 3);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(6));

  // Requesting 4 contiguous frames must return 0 because max_subtree_frames is
  // 3 even though PmmFreeFrameCount() is 6.
  EXPECT_THAT(PmmAllocFrames(4), t::Eq(0));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(6));

  // Freeing the middle barrier coalesces all 8 frames so a 6-frame allocation
  // now succeeds.
  PmmFreeFrames(mid_bar, 2);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(8));
  const uintptr_t recovered = PmmAllocFrames(6);
  EXPECT_THAT(recovered, t::Eq(FrameAddr(20)));
  PmmFreeFrames(recovered, 6);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(8));
}

TEST(PmmTest, RangeIsValidUsableRamValidation) {
  ResetFakeEnv(100);
  AddRegion(FrameAddr(10), 60 * kPageSize, kMemoryTypeAvailable);
  AddRegion(FrameAddr(30), 2 * kPageSize, 2);
  g_env.map.mb_reserved_start = FrameAddr(40);
  g_env.map.mb_reserved_end = FrameAddr(42);
  g_env.map.mb1_mmap_reserved_start = FrameAddr(48);
  g_env.map.mb1_mmap_reserved_end = FrameAddr(49);
  g_env.map.fb_addr = FrameAddr(55);
  g_env.map.fb_pitch = kPageSize;
  g_env.map.fb_width = 64;
  g_env.map.fb_height = 2;
  g_env.map.fb_bpp = 32;
  PmmInit(0, 0);

  // Valid ranges strictly inside usable intervals.
  EXPECT_TRUE(PmmRangeIsValidUsableRam(FrameAddr(10), 6 * kPageSize));
  EXPECT_TRUE(PmmRangeIsValidUsableRam(FrameAddr(10) + 128, 512));
  EXPECT_TRUE(PmmRangeIsValidUsableRam(FrameAddr(20), 10 * kPageSize));
  EXPECT_TRUE(PmmRangeIsValidUsableRam(FrameAddr(32), 8 * kPageSize));
  EXPECT_TRUE(PmmRangeIsValidUsableRam(FrameAddr(57), 13 * kPageSize));

  // Invalid sizes, lower memory, and out-of-bounds addresses.
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(10), 0));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(10), -kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(0, kPageSize));
  EXPECT_FALSE(
      PmmRangeIsValidUsableRam(kLowerMemoryLimit - kPageSize, kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(PmmMaxPhysicalAddress(), kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(69), 2 * kPageSize));

  // Ranges overlapping kernel [16, 20), E820 reserved [30, 32), mb_reserved
  // [40, 42), mb1_mmap_reserved [48, 49), or framebuffer [55, 57) must fail.
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(15), 2 * kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(16), kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(19), 2 * kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(29), 2 * kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(30), kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(39), 4 * kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(48), kPageSize));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(54), 4 * kPageSize));
}

TEST(PmmTest, HighFragmentationStressAndFullCoalescence) {
  ResetFakeEnv(2048);
  constexpr int64_t kArenaStartFrame = 32;
  constexpr int64_t kArenaFrames = 1600;
  AddRegion(FrameAddr(kArenaStartFrame),  //
            kArenaFrames * kPageSize,     //
            kMemoryTypeAvailable);
  PmmInit(0, 0);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(kArenaFrames));

  constexpr int kNumRuns = 256;
  uintptr_t targets[kNumRuns];
  uintptr_t barriers[kNumRuns];
  int64_t target_counts[kNumRuns];

  for (int i = 0; i < kNumRuns; ++i) {
    target_counts[i] = (i % 4) + 1;
    targets[i] = PmmAllocFrames(target_counts[i]);
    barriers[i] = PmmAllocFrame();
    ASSERT_THAT(targets[i], t::Ne(0));
    ASSERT_THAT(barriers[i], t::Ne(0));

    for (int64_t f = 0; f < target_counts[i]; ++f) {
      uint64_t* const word =
          reinterpret_cast<uint64_t*>(targets[i] + f * kPageSize);
      *word = 0x1111000000000000ULL | (static_cast<uint64_t>(i) << 16) | f;
    }
    uint64_t* const bar_word = reinterpret_cast<uint64_t*>(barriers[i]);
    *bar_word = 0xBADC0FFE00000000ULL | i;
  }

  // Free all 256 target runs in pseudo-random order (gcd(73, 256) == 1) while
  // keeping all 256 barrier frames allocated so no two target runs coalesce.
  for (int step = 0; step < kNumRuns; ++step) {
    const int idx = (step * 73 + 19) & (kNumRuns - 1);
    PmmFreeFrames(targets[idx], target_counts[idx]);
  }

  // Re-allocate and free across the 256 separated free runs in multiple
  // rounds. Because we request `target_counts[i]` in ascending index order,
  // lowest-address first-fit via FindFirstAugmented must return the exact
  // original address `targets[i]` for every run.
  for (int round = 0; round < 3; ++round) {
    for (int i = 0; i < kNumRuns; ++i) {
      const uintptr_t realloc_addr = PmmAllocFrames(target_counts[i]);
      ASSERT_THAT(realloc_addr, t::Eq(targets[i]));
    }
    for (int step = 0; step < kNumRuns; ++step) {
      const int idx = (step * 109 + 43) & (kNumRuns - 1);
      PmmFreeFrames(targets[idx], target_counts[idx]);
    }
  }

  // Verify barrier canaries were never corrupted across all fragmentation
  // rounds, then free all barriers in pseudo-random order.
  for (int step = 0; step < kNumRuns; ++step) {
    const int idx = (step * 83 + 11) & (kNumRuns - 1);
    const uint64_t* const bar_word =
        reinterpret_cast<const uint64_t*>(barriers[idx]);
    ASSERT_THAT(*bar_word, t::Eq(0xBADC0FFE00000000ULL | idx));
    PmmFreeFrame(barriers[idx]);
  }

  // All 256 target runs, 256 barriers, and the trailing arena remainder must
  // have coalesced back into a single contiguous run of `kArenaFrames` frames.
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(kArenaFrames));
  const uintptr_t whole_arena = PmmAllocFrames(kArenaFrames);
  EXPECT_THAT(whole_arena, t::Eq(FrameAddr(kArenaStartFrame)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));
  PmmFreeFrames(whole_arena, kArenaFrames);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(kArenaFrames));
}

TEST(PmmTest, InvalidAndDoubleFreeTriggerDcheck) {
  ResetFakeEnv(64);
  AddRegion(FrameAddr(20), 8 * kPageSize, kMemoryTypeAvailable);
  PmmInit(0, 0);

  const uintptr_t allocated = PmmAllocFrames(4);
  ASSERT_THAT(allocated, t::Eq(FrameAddr(20)));

#ifdef NDEBUG
  EXPECT_THAT(PmmAllocFrames(0), t::Eq(0u));
  EXPECT_THAT(PmmAllocFrames(-1), t::Eq(0u));
  EXPECT_THAT(PmmAllocFrames(INT64_MIN), t::Eq(0u));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(4));
  PmmFreeFrames(allocated, 4);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(8));
#else
  // Non-positive allocation frame counts must trigger DCHECK failures.
  EXPECT_DEATH(PmmAllocFrames(0), "Check failed");
  EXPECT_DEATH(PmmAllocFrames(-1), "Check failed");

  // Null, unaligned, non-positive count, out-of-bounds, and reserved-range
  // frees must trigger DCHECK failures.
  EXPECT_DEATH(PmmFreeFrame(0), "Check failed");
  EXPECT_DEATH(PmmFreeFrame(allocated + 64), "Check failed");
  EXPECT_DEATH(PmmFreeFrames(allocated, 0), "Check failed");
  EXPECT_DEATH(PmmFreeFrames(allocated, -2), "Check failed");
  EXPECT_DEATH(PmmFreeFrame(PmmMaxPhysicalAddress()), "Check failed");
  EXPECT_DEATH(PmmFreeFrame(FrameAddr(16)), "Check failed");

  // Direct double-free of an already-free frame (FrameAddr(24) is still in the
  // free tree) and partial overlap spanning allocated + free frames must
  // trigger DCHECK failures.
  EXPECT_DEATH(PmmFreeFrame(FrameAddr(24)), "Check failed");
  EXPECT_DEATH(PmmFreeFrames(FrameAddr(22), 4), "Check failed");

  PmmFreeFrames(allocated, 4);
  EXPECT_DEATH(PmmFreeFrame(allocated), "Check failed");
#endif
}

TEST(PmmTest, ConcurrentMultiThreadedAllocAndFreeStress) {
  ResetFakeEnv(2560);
  constexpr int64_t kArenaStartFrame = 32;
  constexpr int64_t kArenaFrames = 2048;
  AddRegion(FrameAddr(kArenaStartFrame),  //
            kArenaFrames * kPageSize,     //
            kMemoryTypeAvailable);
  PmmInit(0, 0);
  ASSERT_THAT(PmmFreeFrameCount(), t::Eq(kArenaFrames));

  constexpr int kNumThreads = 8;
  constexpr int kIterationsPerThread = 200;
  std::atomic<int> ready_threads{0};
  std::atomic<bool> start_gate{false};

  std::vector<std::thread> workers;
  workers.reserve(kNumThreads);
  for (int tid = 0; tid < kNumThreads; ++tid) {
    workers.emplace_back([&, tid]() {
      ready_threads.fetch_add(1, std::memory_order_acq_rel);
      while (!start_gate.load(std::memory_order_acquire)) {
        asm volatile("pause" : : : "memory");
      }

      for (int iter = 0; iter < kIterationsPerThread; ++iter) {
        const int64_t run_len = ((iter + tid) % 4) + 1;
        const uintptr_t single = PmmAllocFrame();
        const uintptr_t multi = PmmAllocFrames(run_len);
        ASSERT_THAT(single, t::Ne(0u));
        ASSERT_THAT(multi, t::Ne(0u));
        ASSERT_THAT(single, t::Ne(multi));
        ASSERT_THAT(single & (kPageSize - 1), t::Eq(0u));
        ASSERT_THAT(multi & (kPageSize - 1), t::Eq(0u));
        ASSERT_TRUE(PmmRangeIsValidUsableRam(single, kPageSize));
        ASSERT_TRUE(PmmRangeIsValidUsableRam(multi, run_len * kPageSize));

        const uint64_t base_tag = 0xCAFE000000000000ULL |
                                  (static_cast<uint64_t>(tid) << 32) |
                                  (static_cast<uint64_t>(iter) << 8);
        uint64_t* const single_words = reinterpret_cast<uint64_t*>(single);
        single_words[0] = base_tag;
        single_words[kPageSize / sizeof(uint64_t) - 1] = ~base_tag;

        for (int64_t f = 0; f < run_len; ++f) {
          uint64_t* const page_words =
              reinterpret_cast<uint64_t*>(multi + f * kPageSize);
          const uint64_t page_tag = base_tag | static_cast<uint64_t>(f + 1);
          page_words[0] = page_tag;
          page_words[kPageSize / sizeof(uint64_t) - 1] = ~page_tag;
        }

        const int64_t observed_free = PmmFreeFrameCount();
        EXPECT_GE(observed_free, 0);
        EXPECT_LE(observed_free, kArenaFrames);

        ASSERT_THAT(single_words[0], t::Eq(base_tag));
        ASSERT_THAT(single_words[kPageSize / sizeof(uint64_t) - 1],
                    t::Eq(~base_tag));
        for (int64_t f = 0; f < run_len; ++f) {
          const uint64_t* const page_words =
              reinterpret_cast<const uint64_t*>(multi + f * kPageSize);
          const uint64_t page_tag = base_tag | static_cast<uint64_t>(f + 1);
          ASSERT_THAT(page_words[0], t::Eq(page_tag));
          ASSERT_THAT(page_words[kPageSize / sizeof(uint64_t) - 1],
                      t::Eq(~page_tag));
        }

        if ((iter & 1) == 0) {
          PmmFreeFrame(single);
          PmmFreeFrames(multi, run_len);
        } else {
          PmmFreeFrames(multi, run_len);
          PmmFreeFrame(single);
        }
      }
    });
  }

  while (ready_threads.load(std::memory_order_acquire) < kNumThreads) {
    std::this_thread::yield();
  }
  start_gate.store(true, std::memory_order_release);

  for (std::thread& worker : workers) {
    worker.join();
  }

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(kArenaFrames));
  const uintptr_t whole_arena = PmmAllocFrames(kArenaFrames);
  EXPECT_THAT(whole_arena, t::Eq(FrameAddr(kArenaStartFrame)));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(0));
  PmmFreeFrames(whole_arena, kArenaFrames);
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(kArenaFrames));
}

TEST(PmmTest, MultibootMemoryMapSaturatesByteTotalsWithoutSignedOverflow) {
  ResetFakeEnv(64);
  MultibootMemoryMap map = {};
  constexpr uint64_t kHugeLen = static_cast<uint64_t>(INT64_MAX) + 4096ULL;

  ASSERT_TRUE(
      MultibootRecordMmapEntry(&map, 0x100000, kHugeLen, kMemoryTypeAvailable));
  EXPECT_THAT(map.usable_ram_bytes, t::Eq(INT64_MAX));
  EXPECT_THAT(map.total_ram_bytes, t::Eq(INT64_MAX));

  ASSERT_TRUE(MultibootRecordMmapEntry(&map, 0x200000, kHugeLen, 2));
  EXPECT_THAT(map.reserved_ram_bytes, t::Eq(INT64_MAX));
  EXPECT_THAT(map.total_ram_bytes, t::Eq(INT64_MAX));
}

TEST(PmmTest,
     MultibootParseMemoryMapCoalescesAndRejectsUncoalesceableOverflow) {
  ResetFakeEnv(128);

  struct [[gnu::packed]] SyntheticMb2Buffer {
    uint32_t total_size;
    uint32_t reserved;
    uint32_t mmap_type;
    uint32_t mmap_size;
    uint32_t entry_size;
    uint32_t entry_version;
    struct [[gnu::packed]] Entry {
      uint64_t addr;
      uint64_t len;
      uint32_t type;
      uint32_t reserved;
    } entries[kMaxMemoryRegions + 16];
    uint32_t end_type;
    uint32_t end_size;
  };

  const auto buf = std::make_unique<SyntheticMb2Buffer>();
  std::memset(buf.get(), 0, sizeof(SyntheticMb2Buffer));
  buf->total_size = sizeof(SyntheticMb2Buffer);
  buf->mmap_type = 6;
  buf->mmap_size = 16 + static_cast<uint32_t>(sizeof(buf->entries));
  buf->entry_size = sizeof(SyntheticMb2Buffer::Entry);
  buf->entry_version = 0;
  buf->end_type = 0;
  buf->end_size = 8;

  // 1) Populate > kMaxMemoryRegions contiguous available slices in [20, 84)
  // followed by a reserved region at frames [30, 34). Coalescing must merge the
  // contiguous available slices so the trailing reserved region is preserved
  // and carved out by PmmInit.
  constexpr int kTotalEntries = kMaxMemoryRegions + 16;
  for (int i = 0; i < kTotalEntries - 1; ++i) {
    const uintptr_t base = FrameAddr(20) + (i % 64) * kPageSize;
    buf->entries[i] = {base, kPageSize, kMemoryTypeAvailable, 0};
  }
  buf->entries[kTotalEntries - 1] = {FrameAddr(30), 4 * kPageSize, 2, 0};

  const uintptr_t buf_addr = reinterpret_cast<uintptr_t>(buf.get());
  PmmInit(0x36D76289, buf_addr);
  // Frames [20, 84) = 64 frames minus reserved [30, 34) (4 frames) = 60 frames.
  EXPECT_THAT(PmmTotalUsableFrameCount(), t::Eq(60));
  EXPECT_FALSE(PmmRangeIsValidUsableRam(FrameAddr(30), kPageSize));

  // 2) Populate > kMaxMemoryRegions disjoint regions that cannot be coalesced;
  // MultibootParseMemoryMap must return false rather than silently dropping the
  // trailing reserved region.
  for (int i = 0; i < kTotalEntries; ++i) {
    const uintptr_t base =
        FrameAddr(20) + static_cast<uintptr_t>(i * 2) * kPageSize;
    buf->entries[i] = {base, kPageSize, kMemoryTypeAvailable, 0};
  }
  MultibootMemoryMap parsed = {};
  EXPECT_FALSE(MultibootParseMemoryMap(0x36D76289,   //
                                       buf_addr,     //
                                       UINTPTR_MAX,  //
                                       &parsed));
}

TEST(PmmTest, Multiboot2MmapTagSizeExceedingTotalSize) {
  ResetFakeEnv(64);

  struct [[gnu::packed]] TruncatedMb2MmapBuffer {
    uint32_t total_size;
    uint32_t reserved;
    uint32_t mmap_type;
    uint32_t mmap_size;
    uint32_t entry_size;
    uint32_t entry_version;
    uint64_t addr;
    uint64_t len;
    uint32_t type;
    uint32_t entry_reserved;
  };

  constexpr int64_t kExactSize = sizeof(TruncatedMb2MmapBuffer);
  const auto raw_mb2 = std::make_unique<uint8_t[]>(kExactSize);
  std::memset(raw_mb2.get(), 0, kExactSize);
  TruncatedMb2MmapBuffer* const mb2 =
      reinterpret_cast<TruncatedMb2MmapBuffer*>(raw_mb2.get());
  mb2->total_size = static_cast<uint32_t>(kExactSize);
  mb2->mmap_type = 6;
  mb2->mmap_size = 4096;  // Exceeds total_size - offset (40 bytes).
  mb2->entry_size = 24;
  mb2->entry_version = 0;
  mb2->addr = FrameAddr(20);
  mb2->len = 8 * kPageSize;
  mb2->type = kMemoryTypeAvailable;

  MultibootMemoryMap parsed = {};
  EXPECT_FALSE(
      MultibootParseMemoryMap(0x36D76289,                                  //
                              reinterpret_cast<uintptr_t>(raw_mb2.get()),  //
                              UINTPTR_MAX,                                 //
                              &parsed));
  EXPECT_THAT(parsed.region_count, t::Eq(0));

  // Also verify Multiboot1 rejects an entry whose `size + 4` exceeds remaining
  // `mmap_length` without overreading past the buffer.
  struct [[gnu::packed]] Mb1HeaderOnly {
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
  struct [[gnu::packed]] Mb1SingleEntry {
    uint32_t size;
    uint64_t addr;
    uint64_t len;
    uint32_t type;
  };

  const auto raw_mb1_entry = std::make_unique<Mb1SingleEntry>();
  raw_mb1_entry->size = 1024;  // Exceeds mmap_length (24 bytes).
  raw_mb1_entry->addr = FrameAddr(20);
  raw_mb1_entry->len = 8 * kPageSize;
  raw_mb1_entry->type = kMemoryTypeAvailable;

  Mb1HeaderOnly mb1_info = {};
  mb1_info.flags = 1u << 6;
  mb1_info.mmap_length = sizeof(Mb1SingleEntry);
  mb1_info.mmap_addr =
      static_cast<uint32_t>(reinterpret_cast<uintptr_t>(raw_mb1_entry.get()));
  if (reinterpret_cast<uintptr_t>(raw_mb1_entry.get()) <= UINT32_MAX) {
    EXPECT_FALSE(
        MultibootParseMemoryMap(0x2BADB002,                              //
                                reinterpret_cast<uintptr_t>(&mb1_info),  //
                                UINTPTR_MAX,                             //
                                &parsed));
  }
}

TEST(PmmTest, MultibootRecordMmapEntryBridgeEntriesMergeTransitively) {
  ResetFakeEnv(64);
  MultibootMemoryMap map = {};

  constexpr int kPairs = 260;
  for (int i = 0; i < kPairs; ++i) {
    const uintptr_t even_base =
        FrameAddr(20) + static_cast<uintptr_t>(2 * i) * kPageSize;
    ASSERT_TRUE(MultibootRecordMmapEntry(&map,       //
                                         even_base,  //
                                         kPageSize,  //
                                         kMemoryTypeAvailable));
  }
  EXPECT_THAT(map.region_count, t::Eq(kPairs));

  for (int i = 0; i < kPairs; ++i) {
    const uintptr_t odd_base =
        FrameAddr(20) + static_cast<uintptr_t>(2 * i + 1) * kPageSize;
    ASSERT_TRUE(MultibootRecordMmapEntry(&map,       //
                                         odd_base,   //
                                         kPageSize,  //
                                         kMemoryTypeAvailable));
  }
  EXPECT_THAT(map.region_count, t::Eq(1));
  EXPECT_THAT(map.regions[0].base, t::Eq(FrameAddr(20)));
  EXPECT_THAT(map.regions[0].length, t::Eq(2 * kPairs * kPageSize));

  // Verify TryMergeRegions does not merge and truncate when combined span
  // exceeds INT64_MAX.
  MultibootMemoryMap span_map = {};
  constexpr uint64_t kMaxSpan = static_cast<uint64_t>(INT64_MAX);
  ASSERT_TRUE(MultibootRecordMmapEntry(&span_map,  //
                                       0,          //
                                       kMaxSpan,   //
                                       kMemoryTypeAvailable));
  ASSERT_TRUE(MultibootRecordMmapEntry(&span_map,  //
                                       kMaxSpan,   //
                                       kPageSize,  //
                                       kMemoryTypeAvailable));
  EXPECT_THAT(span_map.region_count, t::Eq(2));
  EXPECT_THAT(span_map.regions[0].length, t::Eq(INT64_MAX));
  EXPECT_THAT(span_map.regions[1].length, t::Eq(kPageSize));
}

TEST(PmmTest, AllocFramesBoundsAndNdebugNonPositiveCountSafety) {
  ResetFakeEnv(64);
  AddRegion(FrameAddr(20), 8 * kPageSize, kMemoryTypeAvailable);
  PmmInit(0, 0);
  ASSERT_THAT(PmmFreeFrameCount(), t::Eq(8));

  EXPECT_THAT(PmmAllocFrames(9), t::Eq(0u));
  EXPECT_THAT(PmmAllocFrames(INT64_MAX), t::Eq(0u));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(8));

#ifdef NDEBUG
  EXPECT_THAT(PmmAllocFrames(0), t::Eq(0u));
  EXPECT_THAT(PmmAllocFrames(-1), t::Eq(0u));
  EXPECT_THAT(PmmAllocFrames(INT64_MIN), t::Eq(0u));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(8));
#else
  EXPECT_DEATH(PmmAllocFrames(0), "Check failed");
  EXPECT_DEATH(PmmAllocFrames(-1), "Check failed");
#endif
}

}  // namespace
}  // namespace protos
