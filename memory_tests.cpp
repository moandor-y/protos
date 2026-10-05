#include "memory_tests.h"

#include <cstddef>
#include <cstdint>
#include <new>

#include "heap.h"
#include "paging.h"
#include "pmm.h"
#include "uart.h"

namespace protos {

namespace {

size_t g_widget_ctor_count = 0;
size_t g_widget_dtor_count = 0;

class TestWidget {
 public:
  TestWidget()
      : id_(g_widget_ctor_count + 1),
        checksum_((g_widget_ctor_count + 1) ^ 0x5A5A5A5A5A5A5A5A) {
    ++g_widget_ctor_count;
  }

  explicit TestWidget(const uint64_t seed)
      : id_(seed), checksum_(seed ^ 0x5A5A5A5A5A5A5A5A) {
    ++g_widget_ctor_count;
  }

  ~TestWidget() {
    if (IsValid()) {
      ++g_widget_dtor_count;
    }
    id_ = 0;
    checksum_ = 0;
  }

  bool IsValid() const { return (id_ ^ 0x5A5A5A5A5A5A5A5A) == checksum_; }

  uint64_t id() const { return id_; }

 private:
  uint64_t id_;
  uint64_t checksum_;
};

static void LogTestResult(const char* const test_name, const bool passed) {
  UartWrite("[TEST] ");
  UartWrite(test_name);
  if (passed) {
    UartWrite(": PASS\n");
  } else {
    UartWrite(": FAIL\n");
  }
}

static bool TestPmmAllocAndBounds(uintptr_t* const out_first_frame,   //
                                  uintptr_t* const out_second_frame,  //
                                  uintptr_t* const out_multi_frames) {
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  if (max_phys <= kBootstrapIdentityMapSize ||
      PagingIdentityMappedLimit() < max_phys ||
      !PagingIsIdentityMapped(max_phys - kPageSize)) {
    return false;
  }

  const uintptr_t frame1 = PmmAllocFrame();
  const uintptr_t frame2 = PmmAllocFrame();
  const uintptr_t multi = PmmAllocFrames(4);

  *out_first_frame = frame1;
  *out_second_frame = frame2;
  *out_multi_frames = multi;

  if (frame1 == 0 || frame2 == 0 || multi == 0) {
    return false;
  }
  if (frame1 == frame2 || frame1 == multi || frame2 == multi) {
    return false;
  }
  if ((frame1 & (kPageSize - 1)) != 0 || (frame2 & (kPageSize - 1)) != 0 ||
      (multi & (kPageSize - 1)) != 0) {
    return false;
  }
  if (!PagingIsIdentityMapped(frame1) || !PagingIsIdentityMapped(frame2) ||
      !PagingIsIdentityMapped(multi) ||
      !PmmRangeIsValidUsableRam(frame1, kPageSize) ||
      !PmmRangeIsValidUsableRam(frame2, kPageSize) ||
      !PmmRangeIsValidUsableRam(multi, 4 * kPageSize)) {
    return false;
  }

  volatile uint64_t* const p1 = reinterpret_cast<volatile uint64_t*>(frame1);
  volatile uint64_t* const p2 = reinterpret_cast<volatile uint64_t*>(frame2);
  volatile uint64_t* const pm =
      reinterpret_cast<volatile uint64_t*>(multi + 3 * kPageSize);
  volatile uint64_t* const p_top =
      reinterpret_cast<volatile uint64_t*>(max_phys - kPageSize);
  *p1 = 0xCAFEBABE11112222;
  *p2 = 0xDEADBEEF33334444;
  *pm = 0x0123456789ABCDEF;
  *p_top = 0xFEDCBA9876543210;

  return *p1 == 0xCAFEBABE11112222 && *p2 == 0xDEADBEEF33334444 &&
         *pm == 0x0123456789ABCDEF && *p_top == 0xFEDCBA9876543210;
}

static bool TestPmmFreeAndReuse(const uintptr_t frame1,  //
                                const uintptr_t frame2,  //
                                const uintptr_t multi_frames) {
  const size_t free_with_allocs = PmmFreeFrameCount();
  const size_t total_usable = PmmTotalUsableFrameCount();
  if (total_usable == 0 || free_with_allocs + 6 != total_usable) {
    return false;
  }

  PmmFreeFrame(frame1);
  PmmFreeFrame(frame2);
  PmmFreeFrames(multi_frames, 4);

  if (PmmFreeFrameCount() != free_with_allocs + 6) {
    return false;
  }

  const uintptr_t reused_frame1 = PmmAllocFrame();
  const uintptr_t reused_frame2 = PmmAllocFrame();
  const bool reused_match =
      (reused_frame1 == frame1) && (reused_frame2 == frame2);

  PmmFreeFrame(reused_frame1);
  PmmFreeFrame(reused_frame2);

  const uintptr_t oom_frame = PmmAllocFrames(PmmMaxFrameCount() + 1);
  if (oom_frame != 0) {
    return false;
  }

  return reused_match && (PmmFreeFrameCount() == free_with_allocs + 6);
}

static bool TestHeapVariedSizesAndAlignment() {
  constexpr size_t kSizes[] = {
      1,      //
      7,      //
      15,     //
      16,     //
      31,     //
      64,     //
      256,    //
      1024,   //
      4096,   //
      16384,  //
      65536,  //
  };
  constexpr size_t kNumSizes = sizeof(kSizes) / sizeof(kSizes[0]);

  void* kmalloc_ptrs[kNumSizes];
  uint8_t* new_ptrs[kNumSizes];

  for (size_t i = 0; i < kNumSizes; ++i) {
    const size_t req_size = kSizes[i];
    kmalloc_ptrs[i] = Kmalloc(req_size);
    new_ptrs[i] = new uint8_t[req_size];

    const uintptr_t k_addr = reinterpret_cast<uintptr_t>(kmalloc_ptrs[i]);
    const uintptr_t n_addr = reinterpret_cast<uintptr_t>(new_ptrs[i]);
    if (kmalloc_ptrs[i] == nullptr || new_ptrs[i] == nullptr ||
        (k_addr & (kHeapAlignment - 1)) != 0 ||
        (n_addr & (kHeapAlignment - 1)) != 0 ||
        !PmmRangeIsValidUsableRam(k_addr, req_size) ||
        !PmmRangeIsValidUsableRam(n_addr, req_size)) {
      return false;
    }
  }

  for (size_t i = 0; i < kNumSizes; ++i) {
    Kfree(kmalloc_ptrs[i]);
    delete[] new_ptrs[i];
  }

  return true;
}

static bool TestHeapPatternIsolation() {
  constexpr size_t kSizes[] = {
      24,     //
      128,    //
      512,    //
      2048,   //
      8192,   //
      16384,  //
  };
  constexpr size_t kCount = sizeof(kSizes) / sizeof(kSizes[0]);

  uint8_t* buffers[kCount];
  for (size_t i = 0; i < kCount; ++i) {
    buffers[i] = static_cast<uint8_t*>(Kmalloc(kSizes[i]));
    if (buffers[i] == nullptr) {
      return false;
    }
    for (size_t b = 0; b < kSizes[i]; ++b) {
      const uint8_t pattern =
          static_cast<uint8_t>(((i + 1) * 41 + b * 17) ^ 0xA5);
      buffers[i][b] = pattern;
    }
  }

  for (size_t i = 0; i < kCount; ++i) {
    const uintptr_t start_i = reinterpret_cast<uintptr_t>(buffers[i]);
    const uintptr_t end_i = start_i + kSizes[i];
    for (size_t j = i + 1; j < kCount; ++j) {
      const uintptr_t start_j = reinterpret_cast<uintptr_t>(buffers[j]);
      const uintptr_t end_j = start_j + kSizes[j];
      if (start_i < end_j && end_i > start_j) {
        return false;
      }
    }
    for (size_t b = 0; b < kSizes[i]; ++b) {
      const uint8_t expected =
          static_cast<uint8_t>(((i + 1) * 41 + b * 17) ^ 0xA5);
      if (buffers[i][b] != expected) {
        return false;
      }
    }
  }

  for (size_t i = 0; i < kCount; ++i) {
    Kfree(buffers[i]);
  }
  return true;
}

static bool TestCppNewDeleteLifecycle() {
  g_widget_ctor_count = 0;
  g_widget_dtor_count = 0;
  const size_t free_before = HeapTotalFreeBytes();

  TestWidget* const single = new TestWidget(0x13572468);
  if (single == nullptr || !single->IsValid() || single->id() != 0x13572468 ||
      g_widget_ctor_count != 1 || g_widget_dtor_count != 0) {
    return false;
  }
  delete single;
  if (g_widget_dtor_count != 1) {
    return false;
  }

  constexpr size_t kArrayLen = 12;
  TestWidget* const array = new TestWidget[kArrayLen];
  if (array == nullptr || g_widget_ctor_count != (1 + kArrayLen)) {
    return false;
  }
  for (size_t i = 0; i < kArrayLen; ++i) {
    if (!array[i].IsValid()) {
      return false;
    }
  }
  delete[] array;
  if (g_widget_dtor_count != (1 + kArrayLen)) {
    return false;
  }

  const size_t free_after = HeapTotalFreeBytes();
  return free_after == free_before;
}

static bool TestHeapStressReuse() {
  const size_t free_before = HeapTotalFreeBytes();
  const size_t pmm_before = PmmFreeFrameCount();

  constexpr size_t kIterations = 256;
  constexpr size_t kChunkSizes[] = {
      4096,   //
      8192,   //
      16384,  //
      32768,  //
  };
  constexpr size_t kNumChunks = sizeof(kChunkSizes) / sizeof(kChunkSizes[0]);

  for (size_t iter = 0; iter < kIterations; ++iter) {
    uint8_t* chunks[kNumChunks];
    for (size_t c = 0; c < kNumChunks; ++c) {
      const size_t sz = kChunkSizes[c];
      chunks[c] = static_cast<uint8_t*>(Kmalloc(sz));
      if (chunks[c] == nullptr) {
        return false;
      }
      const uint8_t tag = static_cast<uint8_t>((iter + c) & 0xFF);
      chunks[c][0] = tag;
      chunks[c][sz / 2] = static_cast<uint8_t>(tag ^ 0x55);
      chunks[c][sz - 1] = static_cast<uint8_t>(tag ^ 0xAA);
    }

    for (size_t c = 0; c < kNumChunks; ++c) {
      const size_t sz = kChunkSizes[c];
      const uint8_t tag = static_cast<uint8_t>((iter + c) & 0xFF);
      if (chunks[c][0] != tag ||
          chunks[c][sz / 2] != static_cast<uint8_t>(tag ^ 0x55) ||
          chunks[c][sz - 1] != static_cast<uint8_t>(tag ^ 0xAA)) {
        return false;
      }
    }

    if ((iter & 1) == 0) {
      for (size_t c = 0; c < kNumChunks; ++c) {
        Kfree(chunks[c]);
      }
    } else {
      for (size_t c = kNumChunks; c > 0; --c) {
        Kfree(chunks[c - 1]);
      }
    }
  }

  if (HeapTotalFreeBytes() != free_before ||
      PmmFreeFrameCount() != pmm_before) {
    return false;
  }

  constexpr size_t kLargeChunkSize = 768 * 1024;
  constexpr size_t kLargeIterations = 16;
  size_t free_after_first_expand = 0;
  size_t pmm_after_first_expand = 0;

  for (size_t iter = 0; iter < kLargeIterations; ++iter) {
    uint8_t* const big1 = static_cast<uint8_t*>(Kmalloc(kLargeChunkSize));
    uint8_t* const big2 = static_cast<uint8_t*>(Kmalloc(kLargeChunkSize));
    if (big1 == nullptr || big2 == nullptr) {
      return false;
    }
    const uint8_t s1 = static_cast<uint8_t>((iter + 0x11) & 0xFF);
    const uint8_t s2 = static_cast<uint8_t>((iter + 0x77) & 0xFF);
    big1[0] = s1;
    big1[kLargeChunkSize - 1] = static_cast<uint8_t>(s1 ^ 0xFF);
    big2[0] = s2;
    big2[kLargeChunkSize - 1] = static_cast<uint8_t>(s2 ^ 0xFF);

    if (big1[0] != s1 ||
        big1[kLargeChunkSize - 1] != static_cast<uint8_t>(s1 ^ 0xFF) ||
        big2[0] != s2 ||
        big2[kLargeChunkSize - 1] != static_cast<uint8_t>(s2 ^ 0xFF)) {
      return false;
    }

    Kfree(big1);
    Kfree(big2);

    if (iter == 0) {
      free_after_first_expand = HeapTotalFreeBytes();
      pmm_after_first_expand = PmmFreeFrameCount();
    } else if (HeapTotalFreeBytes() != free_after_first_expand ||
               PmmFreeFrameCount() != pmm_after_first_expand) {
      return false;
    }
  }

  return free_after_first_expand >= free_before;
}

static bool TestEdgeCasesAndOom() {
  const size_t free_before = HeapTotalFreeBytes();

  Kfree(nullptr);
  int* const null_single = nullptr;
  delete null_single;
  int* const null_array = nullptr;
  delete[] null_array;

  void* const zero_alloc = Kmalloc(0);
  if (zero_alloc == nullptr ||
      (reinterpret_cast<uintptr_t>(zero_alloc) & (kHeapAlignment - 1)) != 0) {
    return false;
  }
  Kfree(zero_alloc);

  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  void* const oom1 = Kmalloc(static_cast<size_t>(-1));
  void* const oom2 = Kmalloc(max_phys);
  void* oom3 = operator new(max_phys);
  asm volatile("" : "+r"(oom3));
  if (oom1 != nullptr || oom2 != nullptr || oom3 != nullptr) {
    return false;
  }

  void* const recovery = Kmalloc(128);
  if (recovery == nullptr) {
    return false;
  }
  Kfree(recovery);

  const size_t free_after = HeapTotalFreeBytes();
  return free_after == free_before;
}

}  // namespace

void RunBootVerificationSuite(const uint32_t multiboot_magic,
                              const uint64_t multiboot_info_addr) {
  const bool pmm_ok = PmmInit(multiboot_magic, multiboot_info_addr);
  LogTestResult("pmm_memory_map_init", pmm_ok);
  if (!pmm_ok) {
    UartWrite("[TEST] MEMORY VERIFICATION FAILED\n");
    return;
  }

  uintptr_t frame1 = 0;
  uintptr_t frame2 = 0;
  uintptr_t multi_frames = 0;
  const bool alloc_bounds_ok = TestPmmAllocAndBounds(&frame1,  //
                                                     &frame2,  //
                                                     &multi_frames);
  LogTestResult("pmm_alloc_and_bounds", alloc_bounds_ok);

  const bool free_reuse_ok = TestPmmFreeAndReuse(frame1,  //
                                                 frame2,  //
                                                 multi_frames);
  LogTestResult("pmm_free_and_reuse", free_reuse_ok);

  const bool heap_init_ok = HeapInit();
  const bool varied_ok = heap_init_ok && TestHeapVariedSizesAndAlignment();
  LogTestResult("heap_varied_sizes_and_alignment", varied_ok);

  const bool pattern_ok = TestHeapPatternIsolation();
  LogTestResult("heap_pattern_isolation", pattern_ok);

  const bool cpp_ok = TestCppNewDeleteLifecycle();
  LogTestResult("cpp_new_delete_lifecycle", cpp_ok);

  const bool stress_ok = TestHeapStressReuse();
  LogTestResult("heap_stress_reuse", stress_ok);

  const bool edge_ok = TestEdgeCasesAndOom();
  LogTestResult("edge_cases_and_oom", edge_ok);

  if (pmm_ok && alloc_bounds_ok && free_reuse_ok && varied_ok && pattern_ok &&
      cpp_ok && stress_ok && edge_ok) {
    UartWrite("[TEST] ALL MEMORY TESTS PASSED\n");
  } else {
    UartWrite("[TEST] MEMORY VERIFICATION FAILED\n");
  }
}

}  // namespace protos
