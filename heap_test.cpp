#include "heap.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "pmm.h"

namespace protos {
namespace {

namespace t = ::testing;

constexpr int64_t kInitialHeapFrames = 256;
constexpr int64_t kMinExpandFrames = 16;
constexpr int64_t kFakeRamFrames = 4096;  // 16 MiB
constexpr int64_t kFakeRamBytes = kFakeRamFrames * kPageSize;

alignas(kPageSize) uint8_t g_fake_ram[kFakeRamBytes];

struct FakePmmState {
  int64_t next_frame = 0;
  int64_t limit_frame = kFakeRamFrames;
  int64_t gap_frames = 0;
  bool force_oom = false;
  uintptr_t max_phys_override = 0;
  std::vector<int64_t> scripted_start_frames;
  int alloc_call_count = 0;
  std::vector<int64_t> requested_counts;
};

FakePmmState g_pmm;

static uintptr_t FakeRamBase() {
  return reinterpret_cast<uintptr_t>(g_fake_ram);
}

static void ResetFakePmm(const int64_t zero_frames = 512) {
  g_pmm = FakePmmState{};
  const int64_t bytes_to_zero = (zero_frames <= kFakeRamFrames)
                                    ? (zero_frames * kPageSize)
                                    : kFakeRamBytes;
  std::memset(g_fake_ram, 0, bytes_to_zero);
}

}  // namespace

uintptr_t PmmMaxPhysicalAddress() {
  if (g_pmm.max_phys_override != 0) {
    return g_pmm.max_phys_override;
  }
  return FakeRamBase() + kFakeRamBytes;
}

uintptr_t PmmAllocFrames(const int64_t count) {
  ++g_pmm.alloc_call_count;
  g_pmm.requested_counts.push_back(count);
  if (g_pmm.force_oom || count <= 0) {
    return 0;
  }
  if (!g_pmm.scripted_start_frames.empty()) {
    const int64_t start_frame = g_pmm.scripted_start_frames.front();
    if (start_frame < 0 || start_frame > g_pmm.limit_frame ||
        count > g_pmm.limit_frame - start_frame) {
      return 0;
    }
    g_pmm.scripted_start_frames.erase(g_pmm.scripted_start_frames.begin());
    return FakeRamBase() + start_frame * kPageSize;
  }
  const int64_t start_frame =
      (g_pmm.next_frame == 0) ? 0 : (g_pmm.next_frame + g_pmm.gap_frames);
  if (start_frame < 0 || start_frame > g_pmm.limit_frame ||
      count > g_pmm.limit_frame - start_frame) {
    return 0;
  }
  g_pmm.next_frame = start_frame + count;
  return FakeRamBase() + start_frame * kPageSize;
}

namespace {

TEST(HeapTest, InitZeroSizeAndAlignment) {
  // Verify explicit HeapInit() and Kmalloc(0) minimum 16-byte allocation.
  ResetFakePmm();
  ASSERT_TRUE(HeapInit());
  EXPECT_THAT(g_pmm.alloc_call_count, t::Eq(1));
  ASSERT_THAT(g_pmm.requested_counts, t::ElementsAre(kInitialHeapFrames));

  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;
  EXPECT_THAT(header_size, t::Eq(64));
  EXPECT_THAT(header_size % kHeapAlignment, t::Eq(0));

  void* const zero_ptr = Kmalloc(0);
  ASSERT_THAT(zero_ptr, t::NotNull());
  EXPECT_THAT(reinterpret_cast<uintptr_t>(zero_ptr) & (kHeapAlignment - 1),
              t::Eq(0));
  EXPECT_THAT(HeapTotalFreeBytes(),
              t::Eq(initial_free - kHeapAlignment - header_size));
  Kfree(zero_ptr);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));

  // Verify varied allocation sizes, 16-byte alignment, payload isolation, and
  // full free-byte restoration.
  constexpr int64_t kSizes[] = {1,  7,  15, 16,  17,  31,   32,
                                33, 48, 64, 127, 256, 1024, 4096};
  constexpr int kNumSizes = sizeof(kSizes) / sizeof(kSizes[0]);
  uint8_t* ptrs[kNumSizes];

  for (int i = 0; i < kNumSizes; ++i) {
    const int64_t req_size = kSizes[i];
    ptrs[i] = static_cast<uint8_t*>(Kmalloc(req_size));
    ASSERT_THAT(ptrs[i], t::NotNull());
    const uintptr_t addr = reinterpret_cast<uintptr_t>(ptrs[i]);
    EXPECT_THAT(addr & (kHeapAlignment - 1), t::Eq(0));
    std::memset(ptrs[i], (i + 1) * 19, req_size);
  }

  for (int i = 0; i < kNumSizes; ++i) {
    const uintptr_t start_i = reinterpret_cast<uintptr_t>(ptrs[i]);
    const uintptr_t end_i = start_i + kSizes[i];
    for (int j = i + 1; j < kNumSizes; ++j) {
      const uintptr_t start_j = reinterpret_cast<uintptr_t>(ptrs[j]);
      const uintptr_t end_j = start_j + kSizes[j];
      EXPECT_TRUE(end_i <= start_j || end_j <= start_i);
    }
    const uint8_t expected_byte = (i + 1) * 19;
    for (int64_t b = 0; b < kSizes[i]; ++b) {
      ASSERT_THAT(ptrs[i][b], t::Eq(expected_byte));
    }
  }

  for (int i = 0; i < kNumSizes; ++i) {
    Kfree(ptrs[i]);
  }
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, BlockSplittingThresholds) {
  ResetFakePmm();
  ASSERT_TRUE(HeapInit());
  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  constexpr int64_t kTargetBlockSize = 256;
  void* const front_block = Kmalloc(kTargetBlockSize);
  void* const barrier = Kmalloc(64);
  ASSERT_THAT(front_block, t::NotNull());
  ASSERT_THAT(barrier, t::NotNull());

  Kfree(front_block);
  const int64_t free_with_hole = HeapTotalFreeBytes();

  // Exact fit (256 bytes) does not split.
  void* const exact_fit = Kmalloc(kTargetBlockSize);
  EXPECT_THAT(exact_fit, t::Eq(front_block));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole - kTargetBlockSize));
  Kfree(exact_fit);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole));

  // Remainder is `header_size` (64 bytes), which is 16 bytes below the minimum
  // split threshold (`header_size + kHeapAlignment = 80` bytes). Must NOT
  // split; the full 256-byte block is consumed and later restored.
  const int64_t no_split_req = kTargetBlockSize - header_size;
  void* const unsplit = Kmalloc(no_split_req);
  EXPECT_THAT(unsplit, t::Eq(front_block));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole - kTargetBlockSize));
  Kfree(unsplit);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole));

  // Remainder is exactly `header_size + kHeapAlignment` (80 bytes). Must split
  // into `exact_split_req` (176 bytes) + a new 16-byte free block.
  const int64_t exact_split_req =
      kTargetBlockSize - header_size - kHeapAlignment;
  void* const split_head = Kmalloc(exact_split_req);
  EXPECT_THAT(split_head, t::Eq(front_block));
  EXPECT_THAT(HeapTotalFreeBytes(),
              t::Eq(free_with_hole - exact_split_req - header_size));

  // The 16-byte split remainder has a lower address than the main arena tail
  // after `barrier`, so Kmalloc(16) must return it via FindFirstAugmented.
  void* const split_tail = Kmalloc(kHeapAlignment);
  const uintptr_t expected_tail_addr =
      reinterpret_cast<uintptr_t>(split_head) + exact_split_req + header_size;
  EXPECT_THAT(reinterpret_cast<uintptr_t>(split_tail),
              t::Eq(expected_tail_addr));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_with_hole - kTargetBlockSize));

  Kfree(split_head);
  Kfree(split_tail);
  Kfree(barrier);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, AddressOrderedFirstFitSelectionViaFindFirstAugmented) {
  ResetFakePmm();
  ASSERT_TRUE(HeapInit());
  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  // Create 4 free holes at ascending addresses:
  //   h0 (32B) < h1 (128B) < h2 (512B) < h3 (128B)
  // separated by allocated barrier blocks b0..b3.
  void* const h0 = Kmalloc(32);
  void* const b0 = Kmalloc(32);
  void* const h1 = Kmalloc(128);
  void* const b1 = Kmalloc(32);
  void* const h2 = Kmalloc(512);
  void* const b2 = Kmalloc(32);
  void* const h3 = Kmalloc(128);
  void* const b3 = Kmalloc(32);
  ASSERT_THAT(h0, t::NotNull());
  ASSERT_THAT(b0, t::NotNull());
  ASSERT_THAT(h1, t::NotNull());
  ASSERT_THAT(b1, t::NotNull());
  ASSERT_THAT(h2, t::NotNull());
  ASSERT_THAT(b2, t::NotNull());
  ASSERT_THAT(h3, t::NotNull());
  ASSERT_THAT(b3, t::NotNull());

  Kfree(h0);
  Kfree(h1);
  Kfree(h2);
  Kfree(h3);

  // Request 64B: h0 (32B) is too small; h1 (128B) is the lowest-address
  // qualifying free block and must be chosen over larger h2 (512B) and equal
  // higher-address h3 (128B).
  void* const pick1 = Kmalloc(64);
  EXPECT_THAT(pick1, t::Eq(h1));

  // Request 128B: h1 is now in use; h2 (512B) and h3 (128B) both qualify.
  // Address-ordered first-fit must choose h2 (lower address) over exact-fit h3.
  void* const pick2 = Kmalloc(128);
  EXPECT_THAT(pick2, t::Eq(h2));

  // h2 (512B) was split into 128B + (512 - 128 - header_size = 320B).
  // Requesting 256B must choose the 320B split remainder of h2.
  void* const pick3 = Kmalloc(256);
  const uintptr_t expected_h2_rem =
      reinterpret_cast<uintptr_t>(h2) + 128 + header_size;
  EXPECT_THAT(reinterpret_cast<uintptr_t>(pick3), t::Eq(expected_h2_rem));

  // Request 128B: now h3 (128B) is the lowest-address block with size >= 128.
  void* const pick4 = Kmalloc(128);
  EXPECT_THAT(pick4, t::Eq(h3));

  // Request 32B: h0 (32B) at the very lowest address is still free and fits.
  void* const pick5 = Kmalloc(32);
  EXPECT_THAT(pick5, t::Eq(h0));

  Kfree(pick1);
  Kfree(pick2);
  Kfree(pick3);
  Kfree(pick4);
  Kfree(pick5);
  Kfree(b0);
  Kfree(b1);
  Kfree(b2);
  Kfree(b3);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, CoalescingForwardBackwardAndThreeWay) {
  ResetFakePmm();
  ASSERT_TRUE(HeapInit());
  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  void* a = Kmalloc(64);
  void* b = Kmalloc(128);
  void* const c = Kmalloc(256);
  void* const d = Kmalloc(64);
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());
  ASSERT_THAT(c, t::NotNull());
  ASSERT_THAT(d, t::NotNull());
  const int64_t free_after_abcd = HeapTotalFreeBytes();

  // Backward coalescing: free `a` first, then free `b` -> `b` merges into
  // predecessor `a`, reclaiming `b`'s header.
  Kfree(a);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + 64));
  Kfree(b);
  const int64_t merged_ab_size = 64 + 128 + header_size;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + merged_ab_size));

  // Absorbed header magic at `b` must be cleared to 0.
  const uint32_t* const b_magic_ptr = reinterpret_cast<const uint32_t*>(
      reinterpret_cast<uintptr_t>(b) - header_size);
  EXPECT_THAT(*b_magic_ptr, t::Eq(0));

  void* const ab = Kmalloc(merged_ab_size);
  EXPECT_THAT(ab, t::Eq(a));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  // Restore `a` (64B) and `b` (128B).
  Kfree(ab);
  a = Kmalloc(64);
  b = Kmalloc(128);
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());
  ASSERT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  // Forward coalescing: free `b` first, then free `a` -> `a` merges forward
  // with successor `b`.
  Kfree(b);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + 128));
  Kfree(a);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + merged_ab_size));
  EXPECT_THAT(*b_magic_ptr, t::Eq(0));

  void* const ab2 = Kmalloc(merged_ab_size);
  EXPECT_THAT(ab2, t::Eq(a));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  // Restore `a` (64B) and `b` (128B).
  Kfree(ab2);
  a = Kmalloc(64);
  b = Kmalloc(128);
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());

  // Three-way coalescing: free `a` and `c` first, then free middle block `b`
  // -> `a + b + c` merge into a single free block at `a`.
  Kfree(a);
  Kfree(c);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + 64 + 256));

  Kfree(b);
  const int64_t merged_abc_size = 64 + 128 + 256 + 2 * header_size;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd + merged_abc_size));

  const uint32_t* const c_magic_ptr = reinterpret_cast<const uint32_t*>(
      reinterpret_cast<uintptr_t>(c) - header_size);
  EXPECT_THAT(*b_magic_ptr, t::Eq(0));
  EXPECT_THAT(*c_magic_ptr, t::Eq(0));

  void* const abc = Kmalloc(merged_abc_size);
  EXPECT_THAT(abc, t::Eq(a));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_after_abcd));

  Kfree(abc);
  Kfree(d);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapTest, MultiArenaExpansionContiguousAndNonContiguousIsolation) {
  // Non-contiguous arena expansion with a 1-frame gap. Also verifies that
  // exhausting Arena 1 (`g_free_tree.Empty() == true`) triggers HeapExpand
  // rather than re-running HeapInit().
  ResetFakePmm(1024);
  g_pmm.gap_frames = 1;
  ASSERT_TRUE(HeapInit());

  const int64_t arena1_payload = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - arena1_payload;

  void* const full_arena1 = Kmalloc(arena1_payload);
  ASSERT_THAT(full_arena1, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));

  void* const arena2_alloc = Kmalloc(1024);
  ASSERT_THAT(arena2_alloc, t::NotNull());
  ASSERT_THAT(g_pmm.requested_counts,
              t::ElementsAre(kInitialHeapFrames, kMinExpandFrames));

  const uintptr_t expected_arena2_addr =
      FakeRamBase() + (kInitialHeapFrames + 1) * kPageSize + header_size;
  EXPECT_THAT(reinterpret_cast<uintptr_t>(arena2_alloc),
              t::Eq(expected_arena2_addr));

  Kfree(full_arena1);
  Kfree(arena2_alloc);

  const int64_t arena2_payload = kMinExpandFrames * kPageSize - header_size;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(arena1_payload + arena2_payload));

  // Because of the 1-frame gap, Arena 1 and Arena 2 must NOT have coalesced.
  g_pmm.force_oom = true;
  EXPECT_THAT(Kmalloc(arena1_payload + 16), t::IsNull());
  g_pmm.force_oom = false;

  // Non-contiguous arenas inserted in descending & interleaved address order
  // (frame 512, then frame 100, then frame 300).
  ResetFakePmm(1024);
  g_pmm.scripted_start_frames = {512, 100, 300};
  ASSERT_TRUE(HeapInit());

  void* const high_arena = Kmalloc(arena1_payload);
  ASSERT_THAT(high_arena, t::NotNull());
  void* const low_arena = Kmalloc(arena2_payload);
  ASSERT_THAT(low_arena, t::NotNull());
  void* const mid_arena = Kmalloc(arena2_payload);
  ASSERT_THAT(mid_arena, t::NotNull());

  EXPECT_LT(reinterpret_cast<uintptr_t>(low_arena),
            reinterpret_cast<uintptr_t>(mid_arena));
  EXPECT_LT(reinterpret_cast<uintptr_t>(mid_arena),
            reinterpret_cast<uintptr_t>(high_arena));

  Kfree(high_arena);
  Kfree(low_arena);
  Kfree(mid_arena);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(arena1_payload + 2 * arena2_payload));

  g_pmm.force_oom = true;
  // Small request must select `low_arena` (lowest physical address).
  void* const first_fit_low = Kmalloc(64);
  EXPECT_THAT(first_fit_low, t::Eq(low_arena));
  // Request larger than `arena2_payload` must skip `low_arena` and `mid_arena`
  // and select `high_arena`.
  void* const first_fit_high = Kmalloc(arena2_payload + 64);
  EXPECT_THAT(first_fit_high, t::Eq(high_arena));
  Kfree(first_fit_low);
  Kfree(first_fit_high);

  // Contiguous arena expansion + fallback from `kMinExpandFrames` (16) to
  // `exact_frames` (1) when PMM has only 4 frames remaining.
  ResetFakePmm(512);
  g_pmm.limit_frame = kInitialHeapFrames + 4;
  ASSERT_TRUE(HeapInit());

  void* const p1 = Kmalloc(arena1_payload);
  ASSERT_THAT(p1, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));

  void* const p2 = Kmalloc(2048);
  ASSERT_THAT(p2, t::NotNull());
  // HeapExpand tried kMinExpandFrames (16), failed, and fell back to
  // exact_frames (1).
  ASSERT_THAT(g_pmm.requested_counts,
              t::ElementsAre(kInitialHeapFrames, kMinExpandFrames, 1));

  Kfree(p1);
  Kfree(p2);
  // Because Arena 2 at frame 256 is physically contiguous with Arena 1
  // [0, 256), the two arenas coalesce into a single free block of size
  // `arena1_payload + kPageSize`.
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(arena1_payload + kPageSize));

  g_pmm.force_oom = true;
  void* const cross_arena = Kmalloc(arena1_payload + 2048);
  EXPECT_THAT(cross_arena, t::Eq(p1));
  Kfree(cross_arena);
}

TEST(HeapTest, OomAndNullFreeResilience) {
  // OOM during HeapInit().
  ResetFakePmm();
  g_pmm.force_oom = true;
  EXPECT_FALSE(HeapInit());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));

  // Recovery once PMM frames are available.
  g_pmm.force_oom = false;
  ASSERT_TRUE(HeapInit());
  void* const recovered = Kmalloc(64);
  ASSERT_THAT(recovered, t::NotNull());
  Kfree(recovered);

  const int64_t initial_free = HeapTotalFreeBytes();
  const int64_t header_size = kInitialHeapFrames * kPageSize - initial_free;

  // Overflow and out-of-bounds Kmalloc sizes and PMM exhaustion.
  EXPECT_THAT(Kmalloc(INT64_MAX), t::IsNull());
  EXPECT_THAT(Kmalloc(PmmMaxPhysicalAddress()), t::IsNull());
  EXPECT_THAT(Kmalloc(PmmMaxPhysicalAddress() - header_size), t::IsNull());

  g_pmm.force_oom = true;
  EXPECT_THAT(Kmalloc(initial_free + 64), t::IsNull());
  g_pmm.force_oom = false;
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));

  // Kfree(nullptr) is a safe no-op.
  uint8_t* const a = static_cast<uint8_t*>(Kmalloc(128));
  ASSERT_THAT(a, t::NotNull());
  const int64_t free_baseline = HeapTotalFreeBytes();
  Kfree(nullptr);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(free_baseline));
  Kfree(a);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

TEST(HeapDeathTest, InvalidOrDoubleFreeTriggersDcheck) {
  ResetFakePmm();
  ASSERT_TRUE(HeapInit());

  uint8_t* const a = static_cast<uint8_t*>(Kmalloc(128));
  uint8_t* const b = static_cast<uint8_t*>(Kmalloc(128));
  ASSERT_THAT(a, t::NotNull());
  ASSERT_THAT(b, t::NotNull());
  std::memset(a, 0, 128);

  // Negative Kmalloc size triggers DCHECK.
  EXPECT_DEATH(Kmalloc(-1), "Check failed");

  // Out-of-bounds and misaligned pointers trigger DCHECK in Kfree.
  EXPECT_DEATH(Kfree(reinterpret_cast<void*>(kLowerMemoryLimit)),
               "Check failed");
  EXPECT_DEATH(Kfree(reinterpret_cast<void*>(PmmMaxPhysicalAddress())),
               "Check failed");
  EXPECT_DEATH(Kfree(a + 1), "Check failed");
  EXPECT_DEATH(Kfree(a + 64), "Check failed");

  // Double-free of a standalone free block triggers DCHECK (`is_free == 0`).
  Kfree(b);
  EXPECT_DEATH(Kfree(b), "Check failed");

  // Double-free of an absorbed/coalesced block triggers DCHECK
  // (`magic == kHeapBlockMagic`).
  Kfree(a);
  EXPECT_DEATH(Kfree(b), "Check failed");
}

TEST(HeapTest, HighFragmentationStressAndFullCoalescence) {
  ResetFakePmm(kFakeRamFrames);
  ASSERT_TRUE(HeapInit());
  const int64_t initial_free = HeapTotalFreeBytes();

  constexpr int kNumBlocks = 512;
  uint8_t* targets[kNumBlocks];
  uint8_t* barriers[kNumBlocks];
  int64_t target_sizes[kNumBlocks];

  for (int i = 0; i < kNumBlocks; ++i) {
    target_sizes[i] = ((i % 16) + 1) * kHeapAlignment;
    targets[i] = static_cast<uint8_t*>(Kmalloc(target_sizes[i]));
    barriers[i] = static_cast<uint8_t*>(Kmalloc(kHeapAlignment));
    ASSERT_THAT(targets[i], t::NotNull());
    ASSERT_THAT(barriers[i], t::NotNull());

    std::memset(targets[i], (i + 3) & 0xFF, target_sizes[i]);
    std::memset(barriers[i], (i ^ 0x5A) & 0xFF, kHeapAlignment);
  }

  // Free all 512 target blocks in pseudo-random order (gcd(73, 512) == 1)
  // while keeping all 512 barriers allocated so no two free blocks coalesce.
  for (int step = 0; step < kNumBlocks; ++step) {
    const int idx = (step * 73 + 19) & (kNumBlocks - 1);
    Kfree(targets[idx]);
  }

  // Re-allocate and free across the 512 separated free blocks in multiple
  // rounds. Because we request `target_sizes[i]` in ascending index order,
  // address-ordered first-fit via FindFirstAugmented must return the exact
  // original hole address `targets[i]` for every block.
  for (int round = 0; round < 3; ++round) {
    for (int i = 0; i < kNumBlocks; ++i) {
      void* const realloc_ptr = Kmalloc(target_sizes[i]);
      ASSERT_THAT(realloc_ptr, t::Eq(targets[i]));
    }
    for (int step = 0; step < kNumBlocks; ++step) {
      const int idx = (step * 109 + 43) & (kNumBlocks - 1);
      Kfree(targets[idx]);
    }
  }

  // Verify barrier payloads were never corrupted across all fragmentation
  // rounds, then free all barriers in pseudo-random order.
  for (int step = 0; step < kNumBlocks; ++step) {
    const int idx = (step * 83 + 11) & (kNumBlocks - 1);
    const uint8_t expected = (idx ^ 0x5A) & 0xFF;
    for (int64_t b = 0; b < kHeapAlignment; ++b) {
      ASSERT_THAT(barriers[idx][b], t::Eq(expected));
    }
    Kfree(barriers[idx]);
  }

  // All 1024 blocks + the arena tail must have coalesced back into a single
  // contiguous free block equal to `initial_free`.
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
  void* const whole_arena = Kmalloc(initial_free);
  ASSERT_THAT(whole_arena, t::NotNull());
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(0));
  Kfree(whole_arena);
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(initial_free));
}

}  // namespace
}  // namespace protos
