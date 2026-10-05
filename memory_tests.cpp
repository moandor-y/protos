#include "memory_tests.h"

#include <cstddef>
#include <cstdint>
#include <new>

#include "heap.h"
#include "paging.h"
#include "pmm.h"
#include "rbtree.h"
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

// Verifies that PMM physical frame allocation returns valid, page-aligned,
// identity-mapped usable RAM frames and that the identity map covers all
// discovered usable physical RAM. Outputs the allocated frame addresses so the
// subsequent free-and-reuse test can release them.
static bool TestPmmAllocAndBounds(uintptr_t* const out_first_frame,   //
                                  uintptr_t* const out_second_frame,  //
                                  uintptr_t* const out_multi_frames) {
  // Verify that usable physical RAM extends beyond the initial 64 MiB
  // bootstrap mapping and that PmmInit extended the identity map all the way
  // up to the highest usable physical page.
  const uintptr_t max_phys = PmmMaxPhysicalAddress();
  if (max_phys <= kBootstrapIdentityMapSize ||
      PagingIdentityMappedLimit() < max_phys ||
      !PagingIsIdentityMapped(max_phys - kPageSize)) {
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
  // physical page, and verify that they read back identically.
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

struct BasicRbTestNode {
  uint64_t key;
  uint64_t value;
  RbNode node;
};

struct AugmentedRbTestNode {
  uint64_t key;
  size_t payload_size;
  size_t subtree_max_size;
  size_t subtree_count;
  uint64_t subtree_key_sum;
  RbNode node;
};

struct AugmentedRbTestTraits {
  static uint64_t GetKey(const AugmentedRbTestNode& item) { return item.key; }

  static bool Less(const uint64_t a, const uint64_t b) { return a < b; }

  static void UpdateAugment(AugmentedRbTestNode* const item,        //
                            const AugmentedRbTestNode* const left,  //
                            const AugmentedRbTestNode* const right) {
    size_t max_size = item->payload_size;
    size_t count = 1;
    uint64_t key_sum = item->key;

    if (left != nullptr) {
      if (left->subtree_max_size > max_size) {
        max_size = left->subtree_max_size;
      }
      count += left->subtree_count;
      key_sum += left->subtree_key_sum;
    }
    if (right != nullptr) {
      if (right->subtree_max_size > max_size) {
        max_size = right->subtree_max_size;
      }
      count += right->subtree_count;
      key_sum += right->subtree_key_sum;
    }

    item->subtree_max_size = max_size;
    item->subtree_count = count;
    item->subtree_key_sum = key_sum;
  }
};

constexpr size_t kMaxVerifyDepth = 64;
constexpr size_t kBulkTestNodeCount = 128;
constexpr size_t kSearchTestNodeCount = 256;

BasicRbTestNode g_basic_test_nodes[kBulkTestNodeCount];
AugmentedRbTestNode g_aug_test_nodes[kSearchTestNodeCount];

static bool VerifyNodeAugment(const BasicRbTestNode& item,        //
                              const BasicRbTestNode& recomputed,  //
                              const BasicRbTestNode* const left,  //
                              const BasicRbTestNode* const right) {
  (void)left;
  (void)right;
  return item.key == recomputed.key && item.value == recomputed.value;
}

static bool VerifyNodeAugment(const AugmentedRbTestNode& item,        //
                              const AugmentedRbTestNode& recomputed,  //
                              const AugmentedRbTestNode* const left,  //
                              const AugmentedRbTestNode* const right) {
  const size_t left_max = (left != nullptr) ? left->subtree_max_size : 0;
  const size_t right_max = (right != nullptr) ? right->subtree_max_size : 0;
  size_t expected_max = item.payload_size;
  if (left_max > expected_max) {
    expected_max = left_max;
  }
  if (right_max > expected_max) {
    expected_max = right_max;
  }
  const size_t expected_count = 1 +
                                ((left != nullptr) ? left->subtree_count : 0) +
                                ((right != nullptr) ? right->subtree_count : 0);
  const uint64_t expected_sum =
      item.key + ((left != nullptr) ? left->subtree_key_sum : 0) +
      ((right != nullptr) ? right->subtree_key_sum : 0);

  return item.subtree_max_size == recomputed.subtree_max_size &&
         item.subtree_count == recomputed.subtree_count &&
         item.subtree_key_sum == recomputed.subtree_key_sum &&
         item.subtree_max_size == expected_max &&
         item.subtree_count == expected_count &&
         item.subtree_key_sum == expected_sum;
}

template <typename T, RbNode T::* kNodeMember, typename Traits>
static bool VerifyRbSubtree(const RbTree<T, kNodeMember, Traits>& tree,  //
                            const T* const curr,                         //
                            const T* const expected_parent,              //
                            const T* const min_node,                     //
                            const T* const max_node,                     //
                            const size_t depth,                          //
                            size_t* const out_black_height,              //
                            size_t* const io_visited_count) {
  if (curr == nullptr) {
    *out_black_height = 1;
    return true;
  }
  if (depth > kMaxVerifyDepth) {
    return false;
  }

  ++(*io_visited_count);
  if (tree.Parent(curr) != expected_parent) {
    return false;
  }

  if (min_node != nullptr && !(min_node->key < curr->key)) {
    return false;
  }
  if (max_node != nullptr && !(curr->key < max_node->key)) {
    return false;
  }

  const T* const left = tree.Left(curr);
  const T* const right = tree.Right(curr);
  const RbColor color = (curr->*kNodeMember).color;
  if (color != RbColor::kRed && color != RbColor::kBlack) {
    return false;
  }
  if (tree.Color(curr) != color) {
    return false;
  }

  // Invariant (2): No red node has a red child.
  if (color == RbColor::kRed) {
    if (left != nullptr && (left->*kNodeMember).color == RbColor::kRed) {
      return false;
    }
    if (right != nullptr && (right->*kNodeMember).color == RbColor::kRed) {
      return false;
    }
  }

  size_t left_bh = 0;
  size_t right_bh = 0;
  if (!VerifyRbSubtree(tree,       //
                       left,       //
                       curr,       //
                       min_node,   //
                       curr,       //
                       depth + 1,  //
                       &left_bh,   //
                       io_visited_count) ||
      !VerifyRbSubtree(tree,       //
                       right,      //
                       curr,       //
                       curr,       //
                       max_node,   //
                       depth + 1,  //
                       &right_bh,  //
                       io_visited_count)) {
    return false;
  }

  // Invariant (3): All root-to-null paths have identical black-height.
  if (left_bh != right_bh) {
    return false;
  }
  *out_black_height = left_bh + ((color == RbColor::kBlack) ? 1 : 0);

  // Invariant (5): Every node's augmented value matches the freshly recomputed
  // value of its left and right children.
  T recomputed = *curr;
  Traits::UpdateAugment(&recomputed,  //
                        left,         //
                        right);
  if (!VerifyNodeAugment(*curr,       //
                         recomputed,  //
                         left,        //
                         right)) {
    return false;
  }

  return true;
}

template <typename T, RbNode T::* kNodeMember, typename Traits>
static bool VerifyRbTree(const RbTree<T, kNodeMember, Traits>& tree,
                         const size_t expected_count) {
  const T* const root = tree.root();
  if (root == nullptr) {
    return tree.Empty() && tree.First() == nullptr && tree.Last() == nullptr &&
           expected_count == 0;
  }
  if (tree.Empty() || expected_count == 0) {
    return false;
  }

  // Invariant (1): Root is black and has no parent.
  if (tree.Parent(root) != nullptr ||
      (root->*kNodeMember).color != RbColor::kBlack ||
      tree.Color(root) != RbColor::kBlack) {
    return false;
  }

  size_t black_height = 0;
  size_t visited_count = 0;
  const T* const null_node = nullptr;
  if (!VerifyRbSubtree(tree,           //
                       root,           //
                       null_node,      //
                       null_node,      //
                       null_node,      //
                       0,              //
                       &black_height,  //
                       &visited_count)) {
    return false;
  }
  if (visited_count != expected_count) {
    return false;
  }

  const T* leftmost = root;
  while (tree.Left(leftmost) != nullptr) {
    leftmost = tree.Left(leftmost);
  }
  const T* rightmost = root;
  while (tree.Right(rightmost) != nullptr) {
    rightmost = tree.Right(rightmost);
  }
  if (tree.First() != leftmost || tree.Last() != rightmost) {
    return false;
  }
  if (tree.Prev(tree.First()) != nullptr || tree.Next(tree.Last()) != nullptr) {
    return false;
  }

  // Invariant (4): In-order traversal keys are strictly ordered, and
  // Next()/Prev() are exact inverses across all nodes.
  size_t iter_count = 0;
  const T* prev = nullptr;
  const T* curr = tree.First();
  while (curr != nullptr) {
    ++iter_count;
    if (iter_count > expected_count) {
      return false;
    }
    if (prev != nullptr) {
      if (!(prev->key < curr->key)) {
        return false;
      }
      if (tree.Prev(curr) != prev) {
        return false;
      }
    }
    prev = curr;
    curr = tree.Next(curr);
  }

  return iter_count == expected_count && prev == tree.Last();
}

static bool TestThreeNodeRotationCase(const uint64_t k0,  //
                                      const uint64_t k1,  //
                                      const uint64_t k2) {
  RbTree<BasicRbTestNode, &BasicRbTestNode::node> tree;
  g_basic_test_nodes[0] = {k0, k0 * 10, {}};
  g_basic_test_nodes[1] = {k1, k1 * 10, {}};
  g_basic_test_nodes[2] = {k2, k2 * 10, {}};

  if (!tree.Insert(&g_basic_test_nodes[0]) || !VerifyRbTree(tree, 1)) {
    return false;
  }
  if (!tree.Insert(&g_basic_test_nodes[1]) || !VerifyRbTree(tree, 2)) {
    return false;
  }
  if (!tree.Insert(&g_basic_test_nodes[2]) || !VerifyRbTree(tree, 3)) {
    return false;
  }

  BasicRbTestNode* const root = tree.root();
  if (root == nullptr || root->key != 20 ||
      tree.Color(root) != RbColor::kBlack) {
    return false;
  }
  const BasicRbTestNode* const left = tree.Left(root);
  const BasicRbTestNode* const right = tree.Right(root);
  if (left == nullptr || right == nullptr || left->key != 10 ||
      right->key != 30 || tree.Color(left) != RbColor::kRed ||
      tree.Color(right) != RbColor::kRed) {
    return false;
  }

  tree.Erase(root);
  if (!VerifyRbTree(tree, 2)) {
    return false;
  }
  tree.Erase(tree.First());
  if (!VerifyRbTree(tree, 1)) {
    return false;
  }
  tree.Erase(tree.root());
  return VerifyRbTree(tree, 0);
}

static bool TestRbTreeUnaugmentedInsertAndRotations() {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  if (!tree.Empty() || tree.root() != nullptr || tree.First() != nullptr ||
      tree.Last() != nullptr || tree.Find(42) != nullptr ||
      tree.LowerBound(42) != nullptr || tree.UpperBound(42) != nullptr ||
      Tree::Next(static_cast<BasicRbTestNode*>(nullptr)) != nullptr ||
      Tree::Prev(static_cast<BasicRbTestNode*>(nullptr)) != nullptr ||
      Tree::Left(static_cast<BasicRbTestNode*>(nullptr)) != nullptr ||
      Tree::Right(static_cast<BasicRbTestNode*>(nullptr)) != nullptr ||
      Tree::Parent(static_cast<BasicRbTestNode*>(nullptr)) != nullptr ||
      Tree::Color(nullptr) != RbColor::kBlack || tree.Insert(nullptr) ||
      !VerifyRbTree(tree, 0)) {
    return false;
  }
  tree.Erase(nullptr);
  tree.PropagateAugment(nullptr);
  if (!VerifyRbTree(tree, 0)) {
    return false;
  }

  // Single-node lifecycle and duplicate key rejection.
  g_basic_test_nodes[0] = {50, 500, {}};
  g_basic_test_nodes[1] = {50, 501, {}};
  if (!tree.Insert(g_basic_test_nodes[0]) || !VerifyRbTree(tree, 1)) {
    return false;
  }
  if (tree.Insert(&g_basic_test_nodes[1]) || !VerifyRbTree(tree, 1)) {
    return false;
  }
  if (tree.root() != &g_basic_test_nodes[0] ||
      tree.First() != &g_basic_test_nodes[0] ||
      tree.Last() != &g_basic_test_nodes[0] ||
      Tree::Left(&g_basic_test_nodes[0]) != nullptr ||
      Tree::Right(&g_basic_test_nodes[0]) != nullptr ||
      Tree::Parent(&g_basic_test_nodes[0]) != nullptr ||
      Tree::Color(&g_basic_test_nodes[0]) != RbColor::kBlack ||
      tree.Find(50) != &g_basic_test_nodes[0]) {
    return false;
  }
  tree.Erase(g_basic_test_nodes[0]);
  if (!VerifyRbTree(tree, 0)) {
    return false;
  }

  // LL (right rotation), RR (left rotation), LR (left-right double rotation),
  // and RL (right-left double rotation) cases.
  if (!TestThreeNodeRotationCase(30, 20, 10) ||
      !TestThreeNodeRotationCase(10, 20, 30) ||
      !TestThreeNodeRotationCase(30, 10, 20) ||
      !TestThreeNodeRotationCase(10, 30, 20)) {
    return false;
  }

  // Uncle-red recoloring and multi-level fixup sequence.
  constexpr uint64_t kFixupKeys[] = {
      20,  //
      10,  //
      30,  //
      5,   //
      15,  //
      25,  //
      35,  //
      2,   //
      7,   //
      12,  //
      18,  //
      1,   //
      3,   //
      6,   //
      8,   //
  };
  constexpr size_t kFixupCount = sizeof(kFixupKeys) / sizeof(kFixupKeys[0]);
  for (size_t i = 0; i < kFixupCount; ++i) {
    g_basic_test_nodes[i] = {kFixupKeys[i], kFixupKeys[i] * 3, {}};
    if (!tree.Insert(&g_basic_test_nodes[i]) || !VerifyRbTree(tree, i + 1)) {
      return false;
    }
  }

  for (size_t i = 0; i < kFixupCount; ++i) {
    BasicRbTestNode* const victim = ((i & 1) == 0) ? tree.root() : tree.First();
    tree.Erase(victim);
    if (!VerifyRbTree(tree, kFixupCount - 1 - i)) {
      return false;
    }
  }

  return tree.Empty();
}

static bool TestRbTreeQueriesAndIteration() {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  constexpr uint64_t kInsertOrder[] = {
      80,   //
      40,   //
      120,  //
      20,   //
      60,   //
      100,  //
      140,  //
      10,   //
      30,   //
      50,   //
      70,   //
      90,   //
      110,  //
      130,  //
      150,  //
      160,  //
  };
  constexpr size_t kCount = sizeof(kInsertOrder) / sizeof(kInsertOrder[0]);

  for (size_t i = 0; i < kCount; ++i) {
    g_basic_test_nodes[i] = {kInsertOrder[i], i + 100, {}};
    if (!tree.Insert(&g_basic_test_nodes[i]) || !VerifyRbTree(tree, i + 1)) {
      return false;
    }
  }

  const Tree& const_tree = tree;

  // Below minimum key (10).
  if (tree.Find(0) != nullptr || const_tree.Find(9) != nullptr ||
      tree.LowerBound(0) == nullptr || tree.LowerBound(0)->key != 10 ||
      const_tree.LowerBound(9) == nullptr ||
      const_tree.LowerBound(9)->key != 10 || tree.UpperBound(0) == nullptr ||
      tree.UpperBound(0)->key != 10 || const_tree.UpperBound(9) == nullptr ||
      const_tree.UpperBound(9)->key != 10) {
    return false;
  }

  // Exact keys (10, 20, ..., 160) and missing odd midpoint keys (15, 25, ...).
  for (size_t idx = 1; idx <= kCount; ++idx) {
    const uint64_t exact_key = idx * 10;
    const BasicRbTestNode* const found = tree.Find(exact_key);
    const BasicRbTestNode* const cfound = const_tree.Find(exact_key);
    const BasicRbTestNode* const lb = tree.LowerBound(exact_key);
    const BasicRbTestNode* const clb = const_tree.LowerBound(exact_key);
    const BasicRbTestNode* const ub = tree.UpperBound(exact_key);
    const BasicRbTestNode* const cub = const_tree.UpperBound(exact_key);

    if (found == nullptr || found->key != exact_key || cfound != found ||
        lb != found || clb != found || ub != cub) {
      return false;
    }
    if (idx < kCount) {
      if (ub == nullptr || ub->key != exact_key + 10) {
        return false;
      }
      const uint64_t mid_key = exact_key + 5;
      if (tree.Find(mid_key) != nullptr ||
          const_tree.Find(mid_key) != nullptr ||
          tree.LowerBound(mid_key) != ub ||
          const_tree.LowerBound(mid_key) != ub ||
          tree.UpperBound(mid_key) != ub ||
          const_tree.UpperBound(mid_key) != ub) {
        return false;
      }
    } else if (ub != nullptr) {
      return false;
    }
  }

  // Above maximum key (160).
  if (tree.Find(161) != nullptr || const_tree.Find(1000) != nullptr ||
      tree.LowerBound(161) != nullptr ||
      const_tree.LowerBound(1000) != nullptr ||
      tree.UpperBound(161) != nullptr ||
      const_tree.UpperBound(1000) != nullptr) {
    return false;
  }

  // Forward and backward full iteration checks.
  size_t forward_idx = 0;
  for (const BasicRbTestNode* curr = const_tree.First(); curr != nullptr;
       curr = Tree::Next(curr)) {
    ++forward_idx;
    if (curr->key != forward_idx * 10) {
      return false;
    }
  }
  if (forward_idx != kCount) {
    return false;
  }

  size_t backward_idx = kCount;
  for (const BasicRbTestNode* curr = const_tree.Last(); curr != nullptr;
       curr = Tree::Prev(curr)) {
    if (curr->key != backward_idx * 10) {
      return false;
    }
    --backward_idx;
  }
  if (backward_idx != 0) {
    return false;
  }

  for (size_t i = 0; i < kCount; ++i) {
    tree.Erase(&g_basic_test_nodes[i]);
    if (!VerifyRbTree(tree, kCount - 1 - i)) {
      return false;
    }
  }

  return true;
}

static void InitAugmentedTestNode(AugmentedRbTestNode* const item,
                                  const size_t index) {
  const uint64_t key = (index + 1) * 16;
  const size_t payload_size = ((index * 97 + 31) % 500) + 16;
  item->key = key;
  item->payload_size = payload_size;
  item->subtree_max_size = 0;
  item->subtree_count = 0;
  item->subtree_key_sum = 0;
  item->node = {};
}

static bool TestRbTreeAugmentedBulkInsertErase() {
  using AugTree = RbTree<AugmentedRbTestNode,         //
                         &AugmentedRbTestNode::node,  //
                         AugmentedRbTestTraits>;
  AugTree tree;

  // Phase 1: Ascending insert (0..127) and ascending erase (0..127).
  for (size_t i = 0; i < kBulkTestNodeCount; ++i) {
    InitAugmentedTestNode(&g_aug_test_nodes[i], i);
    if (!tree.Insert(&g_aug_test_nodes[i]) || !VerifyRbTree(tree, i + 1)) {
      return false;
    }
  }
  for (size_t i = 0; i < kBulkTestNodeCount; ++i) {
    tree.Erase(&g_aug_test_nodes[i]);
    if (!VerifyRbTree(tree, kBulkTestNodeCount - 1 - i)) {
      return false;
    }
  }

  // Phase 2: Descending insert (127..0) and descending erase (127..0).
  for (size_t i = kBulkTestNodeCount; i > 0; --i) {
    const size_t idx = i - 1;
    InitAugmentedTestNode(&g_aug_test_nodes[idx], idx);
    const size_t expected_count = kBulkTestNodeCount - idx;
    if (!tree.Insert(&g_aug_test_nodes[idx]) ||
        !VerifyRbTree(tree, expected_count)) {
      return false;
    }
  }
  for (size_t i = kBulkTestNodeCount; i > 0; --i) {
    const size_t idx = i - 1;
    tree.Erase(&g_aug_test_nodes[idx]);
    if (!VerifyRbTree(tree, idx)) {
      return false;
    }
  }

  // Phase 3: Interleaved outside-in insert + continuous root deletion (forcing
  // two-child successor transplants and fixups at every step).
  for (size_t step = 0; step < kBulkTestNodeCount; ++step) {
    const size_t idx =
        ((step & 1) == 0) ? (step / 2) : (kBulkTestNodeCount - 1 - step / 2);
    InitAugmentedTestNode(&g_aug_test_nodes[idx], idx);
    if (!tree.Insert(&g_aug_test_nodes[idx]) || !VerifyRbTree(tree, step + 1)) {
      return false;
    }
  }
  for (size_t step = 0; step < kBulkTestNodeCount; ++step) {
    AugmentedRbTestNode* const current_root = tree.root();
    if (current_root == nullptr) {
      return false;
    }
    tree.Erase(current_root);
    if (!VerifyRbTree(tree, kBulkTestNodeCount - 1 - step)) {
      return false;
    }
  }

  // Phase 4: Pseudo-random permutation insert + independent pseudo-random
  // permutation erase (gcd(37, 128) == 1 and gcd(83, 128) == 1).
  for (size_t step = 0; step < kBulkTestNodeCount; ++step) {
    const size_t idx = (step * 37 + 11) & (kBulkTestNodeCount - 1);
    InitAugmentedTestNode(&g_aug_test_nodes[idx], idx);
    if (!tree.Insert(&g_aug_test_nodes[idx]) || !VerifyRbTree(tree, step + 1)) {
      return false;
    }
  }
  for (size_t step = 0; step < kBulkTestNodeCount; ++step) {
    const size_t idx = (step * 83 + 59) & (kBulkTestNodeCount - 1);
    tree.Erase(&g_aug_test_nodes[idx]);
    if (!VerifyRbTree(tree, kBulkTestNodeCount - 1 - step)) {
      return false;
    }
  }

  return tree.Empty();
}

static bool TestRbTreePropagateAugmentAndSearch() {
  using AugTree = RbTree<AugmentedRbTestNode,         //
                         &AugmentedRbTestNode::node,  //
                         AugmentedRbTestTraits>;
  AugTree tree;

  // Verify FindFirstAugmented on an empty tree returns nullptr with 0 calls.
  size_t empty_calls = 0;
  const AugmentedRbTestNode* const empty_res =
      tree.FindFirstAugmented([&empty_calls](const AugmentedRbTestNode& node) {
        ++empty_calls;
        return node.subtree_max_size >= 1;
      });
  if (empty_res != nullptr || empty_calls != 0) {
    return false;
  }

  // Populate 256 nodes in pseudo-random order (gcd(73, 256) == 1).
  for (size_t step = 0; step < kSearchTestNodeCount; ++step) {
    const size_t idx = (step * 73 + 19) & (kSearchTestNodeCount - 1);
    InitAugmentedTestNode(&g_aug_test_nodes[idx], idx);
    if (!tree.Insert(&g_aug_test_nodes[idx])) {
      return false;
    }
  }
  if (!VerifyRbTree(tree, kSearchTestNodeCount)) {
    return false;
  }

  // Test PropagateAugment after in-place payload_size mutations.
  const size_t original_size_42 = g_aug_test_nodes[42].payload_size;
  g_aug_test_nodes[42].payload_size = 50000;
  tree.PropagateAugment(&g_aug_test_nodes[42]);
  if (tree.root()->subtree_max_size != 50000 ||
      !VerifyRbTree(tree, kSearchTestNodeCount)) {
    return false;
  }

  g_aug_test_nodes[42].payload_size = 8;
  tree.PropagateAugment(g_aug_test_nodes[42]);
  if (tree.root()->subtree_max_size >= 50000 ||
      !VerifyRbTree(tree, kSearchTestNodeCount)) {
    return false;
  }

  g_aug_test_nodes[42].payload_size = original_size_42;
  tree.PropagateAugment(&g_aug_test_nodes[42]);
  if (!VerifyRbTree(tree, kSearchTestNodeCount)) {
    return false;
  }

  // Mutate root and several internal nodes in-place, propagating each time.
  AugmentedRbTestNode* const root_node = tree.root();
  const size_t old_root_size = root_node->payload_size;
  root_node->payload_size = 12345;
  tree.PropagateAugment(root_node);
  if (tree.root()->subtree_max_size != 12345 ||
      !VerifyRbTree(tree, kSearchTestNodeCount)) {
    return false;
  }
  root_node->payload_size = old_root_size;
  tree.PropagateAugment(root_node);

  // Place a unique peak size (20000) at the maximum-key node (index 255) so
  // any unpruned search for 20000 would have to visit all 256 nodes.
  g_aug_test_nodes[kSearchTestNodeCount - 1].payload_size = 20000;
  tree.PropagateAugment(&g_aug_test_nodes[kSearchTestNodeCount - 1]);
  if (tree.root()->subtree_max_size != 20000 ||
      !VerifyRbTree(tree, kSearchTestNodeCount)) {
    return false;
  }

  constexpr size_t kTargetSizes[] = {
      1,      //
      8,      //
      16,     //
      100,    //
      250,    //
      400,    //
      500,    //
      515,    //
      20000,  //
      20001,  //
      99999,  //
  };
  constexpr size_t kNumTargets = sizeof(kTargetSizes) / sizeof(kTargetSizes[0]);
  const AugTree& const_tree = tree;

  for (size_t t = 0; t < kNumTargets; ++t) {
    const size_t target = kTargetSizes[t];

    // Ground-truth linear scan in key order (First -> Next).
    const AugmentedRbTestNode* expected_match = nullptr;
    for (const AugmentedRbTestNode* curr = const_tree.First(); curr != nullptr;
         curr = AugTree::Next(curr)) {
      if (curr->payload_size >= target) {
        expected_match = curr;
        break;
      }
    }

    size_t subtree_pred_calls = 0;
    size_t node_pred_calls = 0;
    const AugmentedRbTestNode* const found_two_pred = tree.FindFirstAugmented(
        [&subtree_pred_calls, target](const AugmentedRbTestNode& node) {
          ++subtree_pred_calls;
          return node.subtree_max_size >= target;
        },
        [&node_pred_calls, target](const AugmentedRbTestNode& node) {
          ++node_pred_calls;
          return node.payload_size >= target;
        });

    if (found_two_pred != expected_match) {
      return false;
    }

    // Verify O(log n) subtree pruning (tree height <= 18 for N = 256).
    if (subtree_pred_calls > 40 || node_pred_calls > 20) {
      return false;
    }
    if (target > const_tree.root()->subtree_max_size) {
      if (found_two_pred != nullptr || subtree_pred_calls != 1 ||
          node_pred_calls != 0) {
        return false;
      }
    }

    // Also verify the single-predicate overload on const_tree.
    size_t single_pred_calls = 0;
    const AugmentedRbTestNode* const found_one_pred =
        const_tree.FindFirstAugmented(
            [&single_pred_calls, target](const AugmentedRbTestNode& node) {
              ++single_pred_calls;
              return node.subtree_max_size >= target;
            });
    if (found_one_pred != expected_match || single_pred_calls > 60) {
      return false;
    }
  }

  return VerifyRbTree(tree, kSearchTestNodeCount);
}

}  // namespace

bool RunRbTreeUnitTests() {
  const bool unaug_ok = TestRbTreeUnaugmentedInsertAndRotations();
  LogTestResult("rbtree_unaugmented_insert_and_rotations", unaug_ok);

  const bool query_ok = TestRbTreeQueriesAndIteration();
  LogTestResult("rbtree_queries_and_iteration", query_ok);

  const bool aug_bulk_ok = TestRbTreeAugmentedBulkInsertErase();
  LogTestResult("rbtree_augmented_bulk_insert_erase", aug_bulk_ok);

  const bool search_ok = TestRbTreePropagateAugmentAndSearch();
  LogTestResult("rbtree_propagate_augment_and_search", search_ok);

  return unaug_ok && query_ok && aug_bulk_ok && search_ok;
}

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

  const bool rbtree_ok = RunRbTreeUnitTests();

  if (pmm_ok && alloc_bounds_ok && free_reuse_ok && varied_ok && pattern_ok &&
      cpp_ok && stress_ok && edge_ok && rbtree_ok) {
    UartWrite("[TEST] ALL MEMORY TESTS PASSED\n");
  } else {
    UartWrite("[TEST] MEMORY VERIFICATION FAILED\n");
  }
}

}  // namespace protos
