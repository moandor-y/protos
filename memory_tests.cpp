#include "memory_tests.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <new>

#include "heap.h"
#include "multiboot.h"
#include "paging.h"
#include "pmm.h"
#include "rbtree.h"
#include "smp.h"
#include "spinlock.h"
#include "uart.h"
#include "vga.h"

namespace protos {

namespace {

int g_widget_ctor_count = 0;
int g_widget_dtor_count = 0;
int g_poly_base_dtor_count = 0;
int g_poly_derived_dtor_count = 0;

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

class PolymorphicBase {
 public:
  [[gnu::noinline]] explicit PolymorphicBase(const uint64_t base_tag)
      : base_tag_(base_tag) {}

  [[gnu::noinline]] virtual ~PolymorphicBase() {
    if (base_tag_ != 0) {
      ++g_poly_base_dtor_count;
    }
    base_tag_ = 0;
  }

  virtual uint64_t Compute(uint64_t input) const = 0;

  virtual uint64_t Tag() const { return base_tag_; }

 protected:
  uint64_t base_tag_;
};

class AddDevice final : public PolymorphicBase {
 public:
  AddDevice(const uint64_t base_tag, const uint64_t delta)
      : PolymorphicBase(base_tag), delta_(delta) {}

  ~AddDevice() override {
    if (delta_ != 0) {
      ++g_poly_derived_dtor_count;
    }
    delta_ = 0;
  }

  uint64_t Compute(const uint64_t input) const override {
    return input + delta_ + base_tag_;
  }

 private:
  uint64_t delta_;
};

class XorDevice final : public PolymorphicBase {
 public:
  XorDevice(const uint64_t base_tag, const uint64_t mask)
      : PolymorphicBase(base_tag), mask_(mask), extra_padding_{mask, ~mask} {}

  ~XorDevice() override {
    if (extra_padding_[0] == mask_ && extra_padding_[1] == ~mask_) {
      ++g_poly_derived_dtor_count;
    }
    mask_ = 0;
  }

  uint64_t Compute(const uint64_t input) const override {
    return (input ^ mask_) + base_tag_;
  }

  uint64_t Tag() const override { return base_tag_ ^ mask_; }

 private:
  uint64_t mask_;
  uint64_t extra_padding_[2];
};

static void ConsoleWrite(const char* const str) {
  UartWrite(str);
  VgaWrite(str);
}

static void LogTestResult(const char* const test_name, const bool passed) {
  ConsoleWrite("[TEST] ");
  ConsoleWrite(test_name);
  if (passed) {
    ConsoleWrite(": PASS\n");
  } else {
    ConsoleWrite(": FAIL\n");
  }
}

// Verifies that PMM physical frame allocation returns valid, page-aligned,
// identity-mapped usable RAM frames and that the identity map covers all
// discovered usable physical RAM. Outputs the allocated frame addresses so the
// subsequent free-and-reuse test can release them.
static bool TestPmmAllocAndBounds(uintptr_t* const out_first_frame,
                                  uintptr_t* const out_second_frame,
                                  uintptr_t* const out_multi_frames) {
  // Verify that usable physical RAM extends beyond the initial 64 MiB
  // bootstrap mapping and that PmmInit extended the identity map all the way
  // up to the highest usable physical page.
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  if (max_phys <= kBootstrapIdentityMapSize ||
      PagingIdentityMappedLimit() < max_phys ||
      !PagingIsIdentityMapped(max_phys - kPageSize) ||
      !PmmRangeIsValidUsableRam(max_phys - kPageSize, kPageSize)) {
    return false;
  }

  // Allocate two single 4 KiB frames and one contiguous run of four 4 KiB
  // frames, recording their addresses for the caller to free in the next test.
  const uintptr_t frame1 = PmmAllocFrame();
  const uintptr_t frame2 = PmmAllocFrame();
  const uintptr_t multi = PmmAllocFrames(4);

  *out_first_frame = frame1;
  *out_second_frame = frame2;
  *out_multi_frames = multi;

  // Ensure all allocations succeeded (non-zero physical address).
  if (frame1 == 0 || frame2 == 0 || multi == 0) {
    return false;
  }
  // Ensure each allocation returned a distinct starting physical address.
  if (frame1 == frame2 || frame1 == multi || frame2 == multi) {
    return false;
  }
  // Ensure all returned physical addresses are 4 KiB page-aligned.
  if ((frame1 & (kPageSize - 1)) != 0 || (frame2 & (kPageSize - 1)) != 0 ||
      (multi & (kPageSize - 1)) != 0) {
    return false;
  }
  // Verify that all allocated frames are identity-mapped in the active page
  // tables and lie strictly within usable RAM (above 1 MiB and not overlapping
  // the kernel image, PMM bitmap, or Multiboot structures).
  if (!PagingIsIdentityMapped(frame1) || !PagingIsIdentityMapped(frame2) ||
      !PagingIsIdentityMapped(multi) ||
      !PmmRangeIsValidUsableRam(frame1, kPageSize) ||
      !PmmRangeIsValidUsableRam(frame2, kPageSize) ||
      !PmmRangeIsValidUsableRam(multi, 4 * kPageSize)) {
    return false;
  }

  // Write distinct 64-bit test patterns to the allocated frames (including the
  // last frame of the 4-frame contiguous block) as well as the highest usable
  // physical page (restoring its prior value afterward), and verify that they
  // read back identically.
  volatile uint64_t* const p1 = reinterpret_cast<volatile uint64_t*>(frame1);
  volatile uint64_t* const p2 = reinterpret_cast<volatile uint64_t*>(frame2);
  volatile uint64_t* const pm =
      reinterpret_cast<volatile uint64_t*>(multi + 3 * kPageSize);
  volatile uint64_t* const p_top =
      reinterpret_cast<volatile uint64_t*>(max_phys - kPageSize);
  const uint64_t saved_top = *p_top;
  *p1 = 0xCAFEBABE11112222;
  *p2 = 0xDEADBEEF33334444;
  *pm = 0x0123456789ABCDEF;
  *p_top = 0xFEDCBA9876543210;

  const bool patterns_ok =
      *p1 == 0xCAFEBABE11112222 && *p2 == 0xDEADBEEF33334444 &&
      *pm == 0x0123456789ABCDEF && *p_top == 0xFEDCBA9876543210;
  *p_top = saved_top;
  return patterns_ok;
}

static bool TestPmmFreeAndReuse(const int64_t free_before_allocs,  //
                                const uintptr_t frame1,            //
                                const uintptr_t frame2,            //
                                const uintptr_t multi_frames) {
  const int64_t free_with_allocs = PmmFreeFrameCount();
  const int64_t total_usable = PmmTotalUsableFrameCount();
  if (total_usable == 0 || free_with_allocs + 6 != free_before_allocs ||
      free_before_allocs > total_usable) {
    return false;
  }

  PmmFreeFrame(frame1);
  PmmFreeFrame(frame2);
  PmmFreeFrames(multi_frames, 4);

  if (PmmFreeFrameCount() != free_before_allocs) {
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

  return reused_match && (PmmFreeFrameCount() == free_before_allocs);
}

struct alignas(64) CacheLineAlignedWidget {
  uint64_t id;
  uint64_t checksum;

  CacheLineAlignedWidget()
      : id(g_widget_ctor_count + 1),
        checksum((g_widget_ctor_count + 1) ^ 0xA5A5A5A5A5A5A5A5ULL) {
    ++g_widget_ctor_count;
  }

  explicit CacheLineAlignedWidget(const uint64_t seed)
      : id(seed), checksum(seed ^ 0xA5A5A5A5A5A5A5A5ULL) {
    ++g_widget_ctor_count;
  }

  ~CacheLineAlignedWidget() {
    if ((id ^ 0xA5A5A5A5A5A5A5A5ULL) == checksum) {
      ++g_widget_dtor_count;
    }
    id = 0;
    checksum = 0;
  }

  bool IsValid() const { return (id ^ 0xA5A5A5A5A5A5A5A5ULL) == checksum; }
};

struct alignas(256) PageSubblockAlignedWidget {
  uint64_t tag;
  uint64_t mirror;

  explicit PageSubblockAlignedWidget(const uint64_t seed = 0xCAFE0000ULL)
      : tag(seed), mirror(~seed) {
    ++g_widget_ctor_count;
  }

  ~PageSubblockAlignedWidget() {
    if (tag == ~mirror) {
      ++g_widget_dtor_count;
    }
    tag = 0;
    mirror = 0;
  }

  bool IsValid() const { return tag == ~mirror; }
};

static bool TestHeapVariedSizesAndAlignment() {
  const int64_t free_before = HeapTotalFreeBytes();
  constexpr int64_t kSizes[] = {1,   7,    15,   16,    31,   64,
                                256, 1024, 4096, 16384, 65536};
  constexpr int kNumSizes = sizeof(kSizes) / sizeof(kSizes[0]);

  void* kmalloc_ptrs[kNumSizes];
  uint8_t* new_ptrs[kNumSizes];

  for (int i = 0; i < kNumSizes; ++i) {
    const int64_t req_size = kSizes[i];
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

  for (int i = 0; i < kNumSizes; ++i) {
    Kfree(kmalloc_ptrs[i]);
    delete[] new_ptrs[i];
  }

  // Verify KmallocAligned across all power-of-two alignments in [1, kPageSize],
  // exercising both header-shifting (0 < offset < 80) and front-splitting
  // (offset >= 80).
  constexpr int64_t kAlignments[] = {1,   2,   4,   8,    16,   32,  64,
                                     128, 256, 512, 1024, 2048, 4096};
  constexpr int kNumAlignments = sizeof(kAlignments) / sizeof(kAlignments[0]);
  uint8_t* aligned_ptrs[kNumAlignments];
  for (int i = 0; i < kNumAlignments; ++i) {
    const int64_t align = kAlignments[i];
    const int64_t eff_align = (align < kHeapAlignment) ? kHeapAlignment : align;
    const int64_t req_size = 48 + i * 16;
    aligned_ptrs[i] = static_cast<uint8_t*>(KmallocAligned(req_size, align));
    const uintptr_t addr = reinterpret_cast<uintptr_t>(aligned_ptrs[i]);
    if (aligned_ptrs[i] == nullptr ||
        (addr & static_cast<uintptr_t>(eff_align - 1)) != 0 ||
        !PmmRangeIsValidUsableRam(addr, req_size)) {
      return false;
    }
    for (int64_t b = 0; b < req_size; ++b) {
      aligned_ptrs[i][b] = static_cast<uint8_t>((i * 31 + b) & 0xFF);
    }
  }
  for (int i = 0; i < kNumAlignments; ++i) {
    const int64_t req_size = 48 + i * 16;
    for (int64_t b = 0; b < req_size; ++b) {
      if (aligned_ptrs[i][b] != static_cast<uint8_t>((i * 31 + b) & 0xFF)) {
        return false;
      }
    }
    Kfree(aligned_ptrs[i]);
  }

  return HeapTotalFreeBytes() == free_before;
}

static bool TestHeapPatternIsolation() {
  constexpr int64_t kSizes[] = {24, 128, 512, 2048, 8192, 16384};
  constexpr int kCount = sizeof(kSizes) / sizeof(kSizes[0]);

  uint8_t* buffers[kCount];
  for (int i = 0; i < kCount; ++i) {
    buffers[i] = static_cast<uint8_t*>(Kmalloc(kSizes[i]));
    if (buffers[i] == nullptr) {
      return false;
    }
    for (int64_t b = 0; b < kSizes[i]; ++b) {
      const uint8_t pattern = ((i + 1) * 41 + b * 17) ^ 0xA5;
      buffers[i][b] = pattern;
    }
  }

  for (int i = 0; i < kCount; ++i) {
    const uintptr_t start_i = reinterpret_cast<uintptr_t>(buffers[i]);
    const uintptr_t end_i = start_i + kSizes[i];
    for (int j = i + 1; j < kCount; ++j) {
      const uintptr_t start_j = reinterpret_cast<uintptr_t>(buffers[j]);
      const uintptr_t end_j = start_j + kSizes[j];
      if (start_i < end_j && end_i > start_j) {
        return false;
      }
    }
    for (int64_t b = 0; b < kSizes[i]; ++b) {
      const uint8_t expected = ((i + 1) * 41 + b * 17) ^ 0xA5;
      if (buffers[i][b] != expected) {
        return false;
      }
    }
  }

  for (int i = 0; i < kCount; ++i) {
    Kfree(buffers[i]);
  }
  return true;
}

static bool TestCppNewDeleteLifecycle() {
  g_widget_ctor_count = 0;
  g_widget_dtor_count = 0;
  g_poly_base_dtor_count = 0;
  g_poly_derived_dtor_count = 0;
  const int64_t free_before = HeapTotalFreeBytes();

  {
    const std::unique_ptr<TestWidget> single =
        std::make_unique<TestWidget>(0x13572468);
    if (single == nullptr || !single->IsValid() || single->id() != 0x13572468 ||
        g_widget_ctor_count != 1 || g_widget_dtor_count != 0) {
      return false;
    }
  }
  if (g_widget_dtor_count != 1) {
    return false;
  }

  constexpr int kArrayLen = 12;
  {
    const std::unique_ptr<TestWidget[]> array =
        std::make_unique<TestWidget[]>(kArrayLen);
    if (array == nullptr || g_widget_ctor_count != (1 + kArrayLen)) {
      return false;
    }
    for (int i = 0; i < kArrayLen; ++i) {
      if (!array[i].IsValid()) {
        return false;
      }
    }
  }
  if (g_widget_dtor_count != (1 + kArrayLen)) {
    return false;
  }

  // Verify over-aligned C++17 `operator new(size_t, std::align_val_t)` and
  // `operator new[](size_t, std::align_val_t)` with `alignas(64)` and
  // `alignas(256)` types.
  const int base_ctor_count = g_widget_ctor_count;
  {
    const std::unique_ptr<CacheLineAlignedWidget> aligned64 =
        std::make_unique<CacheLineAlignedWidget>(0x2468ACE0ULL);
    const std::unique_ptr<PageSubblockAlignedWidget> aligned256 =
        std::make_unique<PageSubblockAlignedWidget>(0xF00DBABEULL);
    constexpr int kAlignedArrayLen = 4;
    const std::unique_ptr<CacheLineAlignedWidget[]> aligned64_arr =
        std::make_unique<CacheLineAlignedWidget[]>(kAlignedArrayLen);
    if (aligned64 == nullptr || aligned256 == nullptr ||
        aligned64_arr == nullptr ||
        (reinterpret_cast<uintptr_t>(aligned64.get()) & 63u) != 0 ||
        (reinterpret_cast<uintptr_t>(aligned256.get()) & 255u) != 0 ||
        (reinterpret_cast<uintptr_t>(aligned64_arr.get()) & 63u) != 0 ||
        !aligned64->IsValid() || !aligned256->IsValid() ||
        g_widget_ctor_count != base_ctor_count + 2 + kAlignedArrayLen) {
      return false;
    }
    for (int i = 0; i < kAlignedArrayLen; ++i) {
      if (!aligned64_arr[i].IsValid() ||
          (reinterpret_cast<uintptr_t>(&aligned64_arr[i]) & 63u) != 0) {
        return false;
      }
    }
  }
  if (g_widget_dtor_count != g_widget_ctor_count) {
    return false;
  }

  // Verify dynamic allocation, pure-virtual and overridden vtable dispatch, and
  // polymorphic deletion through smart pointers (`std::unique_ptr` and
  // `std::shared_ptr` / `std::make_unique` / `std::make_shared`).
  {
    const std::unique_ptr<PolymorphicBase> dev0 =
        std::make_unique<AddDevice>(10, 25);
    const std::shared_ptr<PolymorphicBase> dev1 =
        std::make_shared<XorDevice>(7, 0x0F000F00);
    if (dev0 == nullptr || dev1 == nullptr ||
        HeapTotalFreeBytes() >= free_before) {
      return false;
    }
    std::shared_ptr<PolymorphicBase> dev1_alias = dev1;
    if (dev1.use_count() != 2 || dev0->Compute(100) != 135 ||
        dev0->Tag() != 10 ||
        dev1_alias->Compute(0x00FF00FF) != (0x0FFF0FFF + 7) ||
        dev1_alias->Tag() != (7 ^ 0x0F000F00)) {
      return false;
    }
    dev1_alias.reset();
    if (dev1.use_count() != 1 || g_poly_derived_dtor_count != 0) {
      return false;
    }
  }
  if (g_poly_derived_dtor_count != 2 || g_poly_base_dtor_count != 2) {
    return false;
  }

  const int64_t free_after = HeapTotalFreeBytes();
  return free_after == free_before;
}

static bool TestHeapStressReuse() {
  const int64_t free_before = HeapTotalFreeBytes();
  const int64_t pmm_before = PmmFreeFrameCount();

  constexpr int kIterations = 256;
  constexpr int64_t kChunkSizes[] = {4096, 8192, 16384, 32768};
  constexpr int kNumChunks = sizeof(kChunkSizes) / sizeof(kChunkSizes[0]);

  for (int iter = 0; iter < kIterations; ++iter) {
    uint8_t* chunks[kNumChunks];
    for (int c = 0; c < kNumChunks; ++c) {
      const int64_t sz = kChunkSizes[c];
      chunks[c] = static_cast<uint8_t*>(Kmalloc(sz));
      if (chunks[c] == nullptr) {
        return false;
      }
      const uint8_t tag = (iter + c) & 0xFF;
      chunks[c][0] = tag;
      chunks[c][sz / 2] = tag ^ 0x55;
      chunks[c][sz - 1] = tag ^ 0xAA;
    }

    for (int c = 0; c < kNumChunks; ++c) {
      const int64_t sz = kChunkSizes[c];
      const uint8_t tag = (iter + c) & 0xFF;
      if (chunks[c][0] != tag || chunks[c][sz / 2] != (tag ^ 0x55) ||
          chunks[c][sz - 1] != (tag ^ 0xAA)) {
        return false;
      }
    }

    if ((iter & 1) == 0) {
      for (int c = 0; c < kNumChunks; ++c) {
        Kfree(chunks[c]);
      }
    } else {
      for (int c = kNumChunks; c > 0; --c) {
        Kfree(chunks[c - 1]);
      }
    }
  }

  if (HeapTotalFreeBytes() != free_before ||
      PmmFreeFrameCount() != pmm_before) {
    return false;
  }

  constexpr int64_t kLargeChunkSize = 768 * 1024;
  constexpr int kLargeIterations = 16;
  int64_t free_after_first_expand = 0;
  int64_t pmm_after_first_expand = 0;

  for (int iter = 0; iter < kLargeIterations; ++iter) {
    uint8_t* const big1 = static_cast<uint8_t*>(Kmalloc(kLargeChunkSize));
    uint8_t* const big2 = static_cast<uint8_t*>(Kmalloc(kLargeChunkSize));
    if (big1 == nullptr || big2 == nullptr) {
      return false;
    }
    const uint8_t s1 = (iter + 0x11) & 0xFF;
    const uint8_t s2 = (iter + 0x77) & 0xFF;
    big1[0] = s1;
    big1[kLargeChunkSize - 1] = s1 ^ 0xFF;
    big2[0] = s2;
    big2[kLargeChunkSize - 1] = s2 ^ 0xFF;

    if (big1[0] != s1 || big1[kLargeChunkSize - 1] != (s1 ^ 0xFF) ||
        big2[0] != s2 || big2[kLargeChunkSize - 1] != (s2 ^ 0xFF)) {
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

struct KernelRbAugNode {
  RbNode hook;
  int64_t span = 0;
  int64_t max_subtree_span = 0;

  explicit KernelRbAugNode(const int64_t s) : span(s), max_subtree_span(s) {}
  KernelRbAugNode(const KernelRbAugNode&) = delete;
  KernelRbAugNode& operator=(const KernelRbAugNode&) = delete;
};

struct KernelRbAugTraits {
  static RbNode* GetNode(KernelRbAugNode& item) { return &item.hook; }
  static const RbNode* GetNode(const KernelRbAugNode& item) {
    return &item.hook;
  }
  static uintptr_t GetKey(const KernelRbAugNode& item) {
    return reinterpret_cast<uintptr_t>(&item);
  }
  static bool UpdateAugment(KernelRbAugNode& item,              //
                            const KernelRbAugNode* const left,  //
                            const KernelRbAugNode* const right) {
    int64_t max_val = item.span;
    if (left != nullptr && left->max_subtree_span > max_val) {
      max_val = left->max_subtree_span;
    }
    if (right != nullptr && right->max_subtree_span > max_val) {
      max_val = right->max_subtree_span;
    }
    if (item.max_subtree_span == max_val) {
      return false;
    }
    item.max_subtree_span = max_val;
    return true;
  }
};

static bool TestRbTreeEdgeCases() {
  struct NodeGroup {
    KernelRbAugNode n0{4};
    KernelRbAugNode n1{8};
    KernelRbAugNode n2{16};
    KernelRbAugNode uninserted{32};
  } group;

  RbTree<KernelRbAugNode, &KernelRbAugNode::hook, KernelRbAugTraits> tree;
  tree.Insert(group.n0);
  tree.Insert(group.n1);
  tree.Insert(group.n2);

  // Erasing an uninserted node must not corrupt root() or existing nodes.
  tree.Erase(group.uninserted);
  if (tree.root() == nullptr) {
    return false;
  }

  const auto& const_tree = tree;
  bool saw_foreign_address = false;
  const KernelRbAugNode* const match16 =
      const_tree.FindFirstAugmented([&](const KernelRbAugNode& node) {
        if (&node != &group.n0 && &node != &group.n1 && &node != &group.n2) {
          saw_foreign_address = true;
        }
        return node.max_subtree_span >= 12;
      });
  if (saw_foreign_address || match16 != &group.n2) {
    return false;
  }

  // Erasing n1 twice must leave n0 and n2 intact, and right-subtree augmented
  // search from root n0 (span=4, max_subtree_span=16) must return &group.n2.
  tree.Erase(group.n1);
  tree.Erase(group.n1);
  const KernelRbAugNode* const match_after_erase =
      const_tree.FindFirstAugmented([&](const KernelRbAugNode& node) {
        if (&node != &group.n0 && &node != &group.n2) {
          saw_foreign_address = true;
        }
        return node.max_subtree_span >= 12;
      });
  if (saw_foreign_address || match_after_erase != &group.n2) {
    return false;
  }

  tree.Erase(group.n0);
  tree.Erase(group.n2);
  return tree.root() == nullptr;
}

static void ResetTestMemoryMap(MultibootMemoryMap* const map) {
  map->region_count = 0;
  map->total_ram_bytes = 0;
  map->usable_ram_bytes = 0;
  map->reserved_ram_bytes = 0;
  map->mb_reserved_start = 0;
  map->mb_reserved_end = 0;
  map->mb1_mmap_reserved_start = 0;
  map->mb1_mmap_reserved_end = 0;
  map->fb_addr = 0;
  map->fb_pitch = 0;
  map->fb_width = 0;
  map->fb_height = 0;
  map->fb_bpp = 0;
  map->fb_type = 0;
}

static bool TestMultibootMmapEdgeCases() {
  const std::unique_ptr<MultibootMemoryMap> map(
      static_cast<MultibootMemoryMap*>(operator new(
          sizeof(MultibootMemoryMap))));
  if (map == nullptr) {
    return false;
  }
  ResetTestMemoryMap(map.get());

  constexpr uint64_t kHugeLen = static_cast<uint64_t>(INT64_MAX) + 4096ULL;
  if (!MultibootRecordMmapEntry(map.get(),  //
                                0x100000,   //
                                kHugeLen,   //
                                kMemoryTypeAvailable) ||
      map->usable_ram_bytes != INT64_MAX || map->total_ram_bytes != INT64_MAX) {
    return false;
  }
  if (!MultibootRecordMmapEntry(map.get(), 0x200000, kHugeLen, 2) ||
      map->reserved_ram_bytes != INT64_MAX ||
      map->total_ram_bytes != INT64_MAX) {
    return false;
  }

  ResetTestMemoryMap(map.get());
  if (!MultibootRecordMmapEntry(map.get(),  //
                                0x200000,   //
                                kPageSize,  //
                                kMemoryTypeAvailable) ||
      !MultibootRecordMmapEntry(map.get(),  //
                                0x202000,   //
                                kPageSize,  //
                                kMemoryTypeAvailable) ||
      map->region_count != 2) {
    return false;
  }
  if (!MultibootRecordMmapEntry(map.get(),  //
                                0x201000,   //
                                kPageSize,  //
                                kMemoryTypeAvailable) ||
      map->region_count != 1 || map->regions[0].base != 0x200000 ||
      map->regions[0].length != 3 * kPageSize) {
    return false;
  }
  return true;
}

static bool TestEdgeCasesAndOom() {
  const int64_t free_before = HeapTotalFreeBytes();

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
  void* const oom1 = Kmalloc(INT64_MAX);
  void* const oom2 = Kmalloc(max_phys);
  void* oom3 = operator new(max_phys);
  asm volatile("" : "+r"(oom3));
  void* const bad_align1 = KmallocAligned(64, 0);
  void* const bad_align2 = KmallocAligned(64, 24);
  void* const bad_align3 = KmallocAligned(64, kPageSize * 2);
  void* bad_align4 = operator new(64,
                                  static_cast<std::align_val_t>(kPageSize * 2));
  asm volatile("" : "+r"(bad_align4));
  if (oom1 != nullptr || oom2 != nullptr || oom3 != nullptr ||
      bad_align1 != nullptr || bad_align2 != nullptr || bad_align3 != nullptr ||
      bad_align4 != nullptr) {
    return false;
  }

  void* const recovery = Kmalloc(128);
  if (recovery == nullptr) {
    return false;
  }
  Kfree(recovery);

#ifdef NDEBUG
  if (PmmAllocFrames(0) != 0 || PmmAllocFrames(-1) != 0) {
    return false;
  }
#endif

  if (!TestRbTreeEdgeCases() || !TestMultibootMmapEdgeCases()) {
    return false;
  }

  const int64_t free_after = HeapTotalFreeBytes();
  return free_after == free_before;
}

static bool TestSmpDiscoveryAndApBringup() {
  const int cpu_count = SmpCpuCount();
  const int online_count = SmpOnlineCpuCount();
  if (cpu_count < 1 || online_count != cpu_count ||
      SmpLocalApicPhysAddr() == 0 ||
      !PagingIsIdentityMapped(SmpLocalApicPhysAddr())) {
    return false;
  }

  const CpuInfo* const bsp = SmpGetCpuInfo(0);
  if (bsp == nullptr || !bsp->is_bsp || !bsp->online ||
      !bsp->long_mode_active || bsp->observed_apic_id != bsp->apic_id ||
      bsp->stack_base == 0 || bsp->stack_top <= bsp->stack_base ||
      (bsp->stack_top & 0xF) != 0 || bsp->observed_rsp <= bsp->stack_base ||
      bsp->observed_rsp > bsp->stack_top || bsp->observed_cr3 == 0) {
    return false;
  }

  for (int i = 1; i < cpu_count; ++i) {
    const CpuInfo* const ap = SmpGetCpuInfo(i);
    if (ap == nullptr || ap->is_bsp || !ap->online || !ap->long_mode_active ||
        ap->observed_apic_id != ap->apic_id || ap->stack_base == 0 ||
        ap->stack_top != ap->stack_base + kApStackSize ||
        (ap->stack_top & 0xF) != 0 || ap->observed_rsp <= ap->stack_base ||
        ap->observed_rsp > ap->stack_top ||
        ap->observed_cr3 != bsp->observed_cr3 ||
        !PmmRangeIsValidUsableRam(ap->stack_base, kApStackSize)) {
      return false;
    }

    for (int j = 0; j < i; ++j) {
      const CpuInfo* const other = SmpGetCpuInfo(j);
      if (other == nullptr || ap->apic_id == other->apic_id ||
          (ap->stack_base < other->stack_top &&
           ap->stack_top > other->stack_base)) {
        return false;
      }
    }
  }

  return true;
}

struct ConcurrentHeapItem {
  uint64_t tag;
  uint64_t inverted_tag;

  explicit ConcurrentHeapItem(const uint64_t seed)
      : tag(seed), inverted_tag(~seed) {}

  bool IsValid(const uint64_t expected_seed) const {
    return tag == expected_seed && inverted_tag == ~expected_seed;
  }
};

struct KfreeDeleter {
  void operator()(void* const ptr) const { Kfree(ptr); }
};

class ScopedPmmFrames {
 public:
  ScopedPmmFrames(const uintptr_t addr, const int64_t count)
      : addr_(addr), count_(count) {}

  ~ScopedPmmFrames() {
    if (addr_ != 0) {
      PmmFreeFrames(addr_, count_);
    }
  }

  ScopedPmmFrames(const ScopedPmmFrames&) = delete;
  ScopedPmmFrames& operator=(const ScopedPmmFrames&) = delete;

  uintptr_t get() const { return addr_; }

 private:
  uintptr_t addr_;
  int64_t count_;
};

struct MultiCpuStressContext {
  IrqSpinLock counter_lock;
  int64_t shared_counter = 0;
  int64_t invariant_a = 0;
  int64_t invariant_b = 0;
  int expected_cpus = 0;
  std::atomic<int> start_barrier_count{0};
  int second_pass_count = 0;
  bool per_cpu_ok[kMaxCpus] = {};
};

constexpr int kMultiCpuStressIterations = 64;

static void MultiCpuStressWorker(const int cpu_index, void* const raw_context) {
  MultiCpuStressContext* const ctx =
      static_cast<MultiCpuStressContext*>(raw_context);
  if (ctx == nullptr || cpu_index < 0 || cpu_index >= ctx->expected_cpus) {
    return;
  }

  ctx->start_barrier_count.fetch_add(1, std::memory_order_acq_rel);
  while (ctx->start_barrier_count.load(std::memory_order_acquire) <
         ctx->expected_cpus) {
    asm volatile("pause" : : : "memory");
  }

  bool ok = true;
  for (int iter = 0; iter < kMultiCpuStressIterations; ++iter) {
    // 1. Increment shared counter and update two-part invariant under
    //    IrqSpinLock.
    {
      const IrqSpinLockGuard guard(ctx->counter_lock);
      if (AreInterruptsEnabled() || !ctx->counter_lock.IsLockedByCurrentCpu() ||
          (ctx->invariant_a + ctx->invariant_b) != 0) {
        ok = false;
      }
      const int64_t delta = (cpu_index + 1) * 17 + iter;
      ctx->invariant_a += delta;
      ++ctx->shared_counter;
      ctx->invariant_b -= delta;
    }

    // 2. Concurrent PMM single-frame and multi-frame allocation + pattern
    //    verification.
    const ScopedPmmFrames frame_single(PmmAllocFrame(), 1);
    const ScopedPmmFrames frame_pair(PmmAllocFrames(2), 2);
    if (frame_single.get() == 0 || frame_pair.get() == 0 ||
        frame_single.get() == frame_pair.get() ||
        (frame_single.get() & (kPageSize - 1)) != 0 ||
        (frame_pair.get() & (kPageSize - 1)) != 0 ||
        !PmmRangeIsValidUsableRam(frame_single.get(), kPageSize) ||
        !PmmRangeIsValidUsableRam(frame_pair.get(), 2 * kPageSize)) {
      ok = false;
      break;
    }

    const uint64_t frame_pat1 = 0xC0DEC0DE00000000ULL |
                                (static_cast<uint64_t>(cpu_index) << 16) |
                                static_cast<uint64_t>(iter);
    const uint64_t frame_pat2 = frame_pat1 ^ 0x55AA55AA55AA55AAULL;
    volatile uint64_t* const f1_words =
        reinterpret_cast<volatile uint64_t*>(frame_single.get());
    volatile uint64_t* const f2_first =
        reinterpret_cast<volatile uint64_t*>(frame_pair.get());
    volatile uint64_t* const f2_second =
        reinterpret_cast<volatile uint64_t*>(frame_pair.get() + kPageSize);
    f1_words[0] = frame_pat1;
    f1_words[kPageSize / sizeof(uint64_t) - 1] = ~frame_pat1;
    f2_first[0] = frame_pat2;
    f2_second[kPageSize / sizeof(uint64_t) - 1] = ~frame_pat2;

    // 3. Concurrent Kmalloc/Kfree and C++ new/delete allocations with per-CPU
    //    payload verification.
    const int64_t size_a = 32 + ((iter + cpu_index) % 8) * 32;
    const int64_t size_b = 128 + ((iter * 3 + cpu_index) % 8) * 64;
    const std::unique_ptr<uint8_t[], KfreeDeleter> buf_a(
        static_cast<uint8_t*>(Kmalloc(size_a)));
    const std::unique_ptr<uint8_t[], KfreeDeleter> buf_b(
        static_cast<uint8_t*>(Kmalloc(size_b)));
    const uint64_t item_seed =
        0xA5A5000000000000ULL | (static_cast<uint64_t>(cpu_index) << 24) | iter;
    const std::unique_ptr<ConcurrentHeapItem> heap_item =
        std::make_unique<ConcurrentHeapItem>(item_seed);
    constexpr int kArrayWords = 8;
    const std::unique_ptr<uint64_t[]> heap_array =
        std::make_unique<uint64_t[]>(kArrayWords);

    if (buf_a == nullptr || buf_b == nullptr || heap_item == nullptr ||
        heap_array == nullptr ||
        (reinterpret_cast<uintptr_t>(buf_a.get()) & (kHeapAlignment - 1)) !=
            0 ||
        (reinterpret_cast<uintptr_t>(buf_b.get()) & (kHeapAlignment - 1)) !=
            0) {
      ok = false;
      break;
    }

    const uint8_t byte_a =
        static_cast<uint8_t>(((cpu_index + 1) * 29 + iter * 7) & 0xFF);
    const uint8_t byte_b = static_cast<uint8_t>(byte_a ^ 0x3Cu);
    for (int64_t i = 0; i < size_a; ++i) {
      buf_a[i] = static_cast<uint8_t>(byte_a + (i & 0xF));
    }
    for (int64_t i = 0; i < size_b; ++i) {
      buf_b[i] = static_cast<uint8_t>(byte_b ^ (i & 0xF));
    }
    for (int i = 0; i < kArrayWords; ++i) {
      heap_array[i] = item_seed + i;
    }

    // Acquire the shared spinlock a second time while holding live PMM frames
    // and Heap allocations so allocations across CPUs overlap in time.
    {
      const IrqSpinLockGuard guard(ctx->counter_lock);
      if ((ctx->invariant_a + ctx->invariant_b) != 0) {
        ok = false;
      }
      const int64_t delta = (cpu_index + 3) * 31 + iter;
      ctx->invariant_a -= delta;
      ++ctx->shared_counter;
      ctx->invariant_b += delta;
    }

    if (f1_words[0] != frame_pat1 ||
        f1_words[kPageSize / sizeof(uint64_t) - 1] != ~frame_pat1 ||
        f2_first[0] != frame_pat2 ||
        f2_second[kPageSize / sizeof(uint64_t) - 1] != ~frame_pat2 ||
        !heap_item->IsValid(item_seed)) {
      ok = false;
    }
    for (int64_t i = 0; i < size_a; ++i) {
      if (buf_a[i] != static_cast<uint8_t>(byte_a + (i & 0xF))) {
        ok = false;
        break;
      }
    }
    for (int64_t i = 0; i < size_b; ++i) {
      if (buf_b[i] != static_cast<uint8_t>(byte_b ^ (i & 0xF))) {
        ok = false;
        break;
      }
    }
    for (int i = 0; i < kArrayWords; ++i) {
      if (heap_array[i] != item_seed + i) {
        ok = false;
        break;
      }
    }

    // 4. Concurrent UART and VGA console logging on first and last iterations.
    //    Pass a complete string per UartWrite / VgaWrite call so g_uart_lock
    //    and g_vga_lock serialize the entire message atomically, and use '\r'
    //    on VGA so Row 0 ("Hello, x86-64 Kernel World!") at 0xB8000 is never
    //    scrolled off screen.
    if (iter == 0 || iter + 1 == kMultiCpuStressIterations) {
      const char cpu_digit = static_cast<char>('0' + (cpu_index % 10));
      const char pass_digit = (iter == 0) ? '0' : '1';
      const char uart_msg[] = {
          '[', 'S',       'M',        'P', ']', ' ', 'C',  'P', 'U',
          ' ', cpu_digit, ' ',        's', 'y', 'n', 'c',  ' ', 's',
          't', 'r',       'e',        's', 's', ' ', 'p',  'a', 's',
          's', ' ',       pass_digit, ' ', 'o', 'k', '\n', '\0'};
      const char vga_msg[] = {'[', 'S', 'M',       'P', ']', ' ', 'C',  'P',
                              'U', ' ', cpu_digit, ' ', 'o', 'k', '\r', '\0'};
      UartWrite(uart_msg);
      VgaWrite(vga_msg);
    }
  }

  ctx->per_cpu_ok[cpu_index] = ok;
}

static void MultiCpuSecondPassWorker(const int cpu_index,
                                     void* const raw_context) {
  MultiCpuStressContext* const ctx =
      static_cast<MultiCpuStressContext*>(raw_context);
  if (ctx == nullptr || cpu_index < 0 || cpu_index >= ctx->expected_cpus) {
    return;
  }
  const IrqSpinLockGuard guard(ctx->counter_lock);
  ctx->second_pass_count += (cpu_index + 1);
}

static bool TestSmpMultiCpuSyncStress() {
  const int cpu_count = SmpCpuCount();
  if (cpu_count < 1 || SmpOnlineCpuCount() != cpu_count) {
    return false;
  }

  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  if (pmm_free_before <= 0 || heap_free_before <= 0) {
    return false;
  }

  MultiCpuStressContext ctx = {};
  ctx.expected_cpus = cpu_count;

  SmpRunOnAllCpus(MultiCpuStressWorker, &ctx);
  // Ensure cursor is at column 0 before LogTestResult writes the test summary.
  VgaPutc('\r');

  // Dispatch a second round of work across all CPUs to verify APs returned
  // cleanly to their halted state after the first round and can be re-woken.
  SmpRunOnAllCpus(MultiCpuSecondPassWorker, &ctx);

  const int64_t expected_counter =
      static_cast<int64_t>(cpu_count) * kMultiCpuStressIterations * 2;
  const int expected_second_pass = (cpu_count * (cpu_count + 1)) / 2;
  if (ctx.shared_counter != expected_counter ||
      (ctx.invariant_a + ctx.invariant_b) != 0 ||
      ctx.second_pass_count != expected_second_pass) {
    return false;
  }

  for (int i = 0; i < cpu_count; ++i) {
    if (!ctx.per_cpu_ok[i]) {
      return false;
    }
  }

  return PmmFreeFrameCount() == pmm_free_before &&
         HeapTotalFreeBytes() == heap_free_before &&
         SmpOnlineCpuCount() == cpu_count;
}

}  // namespace

void RunBootVerificationSuite() {
  const bool pmm_ok = PmmTotalUsableFrameCount() > 0 &&
                      PmmFreeFrameCount() > 0 &&
                      PmmMaxPhysicalAddress() > kBootstrapIdentityMapSize;
  LogTestResult("pmm_memory_map_init", pmm_ok);
  if (!pmm_ok) {
    ConsoleWrite("[TEST] MEMORY VERIFICATION FAILED\n");
    return;
  }

  const int64_t free_before_allocs = PmmFreeFrameCount();
  uintptr_t frame1 = 0;
  uintptr_t frame2 = 0;
  uintptr_t multi_frames = 0;
  const bool alloc_bounds_ok =
      TestPmmAllocAndBounds(&frame1, &frame2, &multi_frames);
  LogTestResult("pmm_alloc_and_bounds", alloc_bounds_ok);

  const bool free_reuse_ok =
      TestPmmFreeAndReuse(free_before_allocs, frame1, frame2, multi_frames);
  LogTestResult("pmm_free_and_reuse", free_reuse_ok);

  const bool varied_ok =
      HeapTotalFreeBytes() > 0 && TestHeapVariedSizesAndAlignment();
  LogTestResult("heap_varied_sizes_and_alignment", varied_ok);

  const bool pattern_ok = TestHeapPatternIsolation();
  LogTestResult("heap_pattern_isolation", pattern_ok);

  const bool cpp_ok = TestCppNewDeleteLifecycle();
  LogTestResult("cpp_new_delete_lifecycle", cpp_ok);

  const bool stress_ok = TestHeapStressReuse();
  LogTestResult("heap_stress_reuse", stress_ok);

  const bool edge_ok = TestEdgeCasesAndOom();
  LogTestResult("edge_cases_and_oom", edge_ok);

  const bool smp_ok = TestSmpDiscoveryAndApBringup();
  LogTestResult("smp_discovery_and_ap_bringup", smp_ok);

  const bool smp_sync_ok = TestSmpMultiCpuSyncStress();
  LogTestResult("smp_multicpu_sync_stress", smp_sync_ok);

  if (pmm_ok && alloc_bounds_ok && free_reuse_ok && varied_ok && pattern_ok &&
      cpp_ok && stress_ok && edge_ok && smp_ok && smp_sync_ok) {
    UartWrite("[TEST] ALL MEMORY TESTS PASSED\n");
    VgaWrite("[TEST] ALL MEMORY TESTS PASSED");
  } else {
    UartWrite("[TEST] MEMORY VERIFICATION FAILED\n");
    VgaWrite("[TEST] MEMORY VERIFICATION FAILED");
  }
}

}  // namespace protos
