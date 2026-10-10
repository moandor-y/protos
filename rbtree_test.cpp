#include "rbtree.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

namespace protos {
namespace {

namespace t = ::testing;

struct BasicRbTestNode {
  int key;
  int value;
  RbNode node;
};

struct AugmentedRbTestNode {
  int key;
  int64_t payload_size;
  int64_t subtree_max_size;
  int64_t subtree_count;
  int subtree_key_sum;
  RbNode node;
};

struct AugmentedRbTestTraits {
  static int GetKey(const AugmentedRbTestNode& item) { return item.key; }

  static bool Less(const int a, const int b) { return a < b; }

  static void UpdateAugment(AugmentedRbTestNode* const item,
                            const AugmentedRbTestNode* const left,
                            const AugmentedRbTestNode* const right) {
    int64_t max_size = item->payload_size;
    int64_t count = 1;
    int key_sum = item->key;

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

constexpr int64_t kMaxVerifyDepth = 64;
constexpr int64_t kBulkTestNodeCount = 128;
constexpr int64_t kSearchTestNodeCount = 256;

static bool VerifyNodeAugment(const BasicRbTestNode& item,
                              const BasicRbTestNode& recomputed,
                              const BasicRbTestNode* const left,
                              const BasicRbTestNode* const right) {
  (void)left;
  (void)right;
  return item.key == recomputed.key && item.value == recomputed.value;
}

static bool VerifyNodeAugment(const AugmentedRbTestNode& item,
                              const AugmentedRbTestNode& recomputed,
                              const AugmentedRbTestNode* const left,
                              const AugmentedRbTestNode* const right) {
  const int64_t left_max = (left != nullptr) ? left->subtree_max_size : 0;
  const int64_t right_max = (right != nullptr) ? right->subtree_max_size : 0;
  int64_t expected_max = item.payload_size;
  if (left_max > expected_max) {
    expected_max = left_max;
  }
  if (right_max > expected_max) {
    expected_max = right_max;
  }
  const int64_t expected_count =
      1 + ((left != nullptr) ? left->subtree_count : 0) +
      ((right != nullptr) ? right->subtree_count : 0);
  const int expected_sum = item.key +
                           ((left != nullptr) ? left->subtree_key_sum : 0) +
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
                            const int64_t depth,                         //
                            int64_t* const out_black_height,             //
                            int64_t* const io_visited_count) {
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
  const RbColor color = (curr->*kNodeMember).color();
  if (color != RbColor::kRed && color != RbColor::kBlack) {
    return false;
  }
  if (tree.Color(curr) != color) {
    return false;
  }

  // Invariant (2): No red node has a red child.
  if (color == RbColor::kRed) {
    if (left != nullptr && (left->*kNodeMember).color() == RbColor::kRed) {
      return false;
    }
    if (right != nullptr && (right->*kNodeMember).color() == RbColor::kRed) {
      return false;
    }
  }

  int64_t left_bh = 0;
  int64_t right_bh = 0;
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
  Traits::UpdateAugment(&recomputed, left, right);
  if (!VerifyNodeAugment(*curr, recomputed, left, right)) {
    return false;
  }

  return true;
}

template <typename T, RbNode T::* kNodeMember, typename Traits>
static t::AssertionResult VerifyRbTree(
    const RbTree<T, kNodeMember, Traits>& tree, const int64_t expected_count) {
  const T* const root = tree.root();
  if (root == nullptr) {
    if (tree.Empty() && tree.First() == nullptr && tree.Last() == nullptr &&
        expected_count == 0) {
      return t::AssertionSuccess();
    }
    return t::AssertionFailure()
           << "Empty tree state mismatch for expected_count=" << expected_count;
  }
  if (tree.Empty() || expected_count == 0) {
    return t::AssertionFailure()
           << "Non-null root when expected_count=" << expected_count;
  }

  // Invariant (1): Root is black and has no parent.
  if (tree.Parent(root) != nullptr ||
      (root->*kNodeMember).color() != RbColor::kBlack ||
      tree.Color(root) != RbColor::kBlack) {
    return t::AssertionFailure() << "Root must be black with null parent";
  }

  int64_t black_height = 0;
  int64_t visited_count = 0;
  const T* const null_node = nullptr;
  if (!VerifyRbSubtree(tree,           //
                       root,           //
                       null_node,      //
                       null_node,      //
                       null_node,      //
                       0,              //
                       &black_height,  //
                       &visited_count)) {
    return t::AssertionFailure() << "Subtree Red-Black or augment invariant "
                                    "failed";
  }
  if (visited_count != expected_count) {
    return t::AssertionFailure() << "Visited node count " << visited_count
                                 << " != expected " << expected_count;
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
    return t::AssertionFailure() << "First()/Last() mismatch with tree extrema";
  }
  if (tree.Prev(tree.First()) != nullptr || tree.Next(tree.Last()) != nullptr) {
    return t::AssertionFailure()
           << "Prev(First()) and Next(Last()) must be null";
  }

  // Invariant (4): In-order traversal keys are strictly ordered, and
  // Next()/Prev() are exact inverses across all nodes.
  int64_t iter_count = 0;
  const T* prev = nullptr;
  const T* curr = tree.First();
  while (curr != nullptr) {
    ++iter_count;
    if (iter_count > expected_count) {
      return t::AssertionFailure() << "Iteration cycle detected";
    }
    if (prev != nullptr) {
      if (!(prev->key < curr->key)) {
        return t::AssertionFailure() << "Out-of-order keys during traversal";
      }
      if (tree.Prev(curr) != prev) {
        return t::AssertionFailure() << "Prev(curr) != prev during traversal";
      }
    }
    prev = curr;
    curr = tree.Next(curr);
  }

  if (iter_count != expected_count || prev != tree.Last()) {
    return t::AssertionFailure() << "In-order traversal count/end mismatch";
  }
  return t::AssertionSuccess();
}

template <typename Tree>
static std::vector<int> CollectKeys(const Tree& tree) {
  std::vector<int> keys;
  for (auto* curr = tree.First(); curr != nullptr; curr = Tree::Next(curr)) {
    keys.push_back(curr->key);
  }
  return keys;
}

template <typename Tree>
static std::vector<int> CollectKeysReverse(const Tree& tree) {
  std::vector<int> keys;
  for (auto* curr = tree.Last(); curr != nullptr; curr = Tree::Prev(curr)) {
    keys.push_back(curr->key);
  }
  return keys;
}

static void InitAugmentedTestNode(AugmentedRbTestNode* const item,
                                  const int64_t index) {
  const int key = (index + 1) * 16;
  const int64_t payload_size = ((index * 97 + 31) % 500) + 16;
  item->key = key;
  item->payload_size = payload_size;
  item->subtree_max_size = 0;
  item->subtree_count = 0;
  item->subtree_key_sum = 0;
  item->node = {};
}

static void VerifyThreeNodeRotationCase(const int k0,  //
                                        const int k1,  //
                                        const int k2) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;
  BasicRbTestNode nodes[3] = {
      {k0, k0 * 10, {}},
      {k1, k1 * 10, {}},
      {k2, k2 * 10, {}},
  };

  ASSERT_TRUE(tree.Insert(&nodes[0]));
  ASSERT_TRUE(VerifyRbTree(tree, 1));
  ASSERT_TRUE(tree.Insert(&nodes[1]));
  ASSERT_TRUE(VerifyRbTree(tree, 2));
  ASSERT_TRUE(tree.Insert(&nodes[2]));
  ASSERT_TRUE(VerifyRbTree(tree, 3));

  BasicRbTestNode* const root = tree.root();
  ASSERT_THAT(root, t::NotNull());
  EXPECT_THAT(root->key, t::Eq(20));
  EXPECT_THAT(tree.Color(root), t::Eq(RbColor::kBlack));

  const BasicRbTestNode* const left = tree.Left(root);
  const BasicRbTestNode* const right = tree.Right(root);
  ASSERT_THAT(left, t::NotNull());
  ASSERT_THAT(right, t::NotNull());
  EXPECT_THAT(left->key, t::Eq(10));
  EXPECT_THAT(right->key, t::Eq(30));
  EXPECT_THAT(tree.Color(left), t::Eq(RbColor::kRed));
  EXPECT_THAT(tree.Color(right), t::Eq(RbColor::kRed));
  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(10, 20, 30));

  tree.Erase(root);
  ASSERT_TRUE(VerifyRbTree(tree, 2));
  tree.Erase(tree.First());
  ASSERT_TRUE(VerifyRbTree(tree, 1));
  tree.Erase(tree.root());
  EXPECT_TRUE(VerifyRbTree(tree, 0));
}

TEST(RbTreeTest, EmptyTreeAndNullPointerSafety) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  EXPECT_TRUE(tree.Empty());
  EXPECT_THAT(tree.root(), t::IsNull());
  EXPECT_THAT(tree.First(), t::IsNull());
  EXPECT_THAT(tree.Last(), t::IsNull());
  EXPECT_THAT(tree.Find(42), t::IsNull());
  EXPECT_THAT(tree.LowerBound(42), t::IsNull());
  EXPECT_THAT(tree.UpperBound(42), t::IsNull());
  EXPECT_THAT(Tree::Next(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Prev(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Left(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Right(static_cast<BasicRbTestNode*>(nullptr)), t::IsNull());
  EXPECT_THAT(Tree::Parent(static_cast<BasicRbTestNode*>(nullptr)),
              t::IsNull());
  EXPECT_THAT(Tree::Color(nullptr), t::Eq(RbColor::kBlack));
  EXPECT_FALSE(tree.Insert(nullptr));
  EXPECT_THAT(CollectKeys(tree), t::IsEmpty());
  EXPECT_TRUE(VerifyRbTree(tree, 0));

  tree.Erase(nullptr);
  tree.PropagateAugment(nullptr);
  EXPECT_TRUE(VerifyRbTree(tree, 0));
}

TEST(RbTreeTest, SingleNodeLifecycleAndDuplicateKeyRejection) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  BasicRbTestNode first_node = {50, 500, {}};
  BasicRbTestNode duplicate_node = {50, 501, {}};

  ASSERT_TRUE(tree.Insert(first_node));
  ASSERT_TRUE(VerifyRbTree(tree, 1));

  EXPECT_FALSE(tree.Insert(&duplicate_node));
  ASSERT_TRUE(VerifyRbTree(tree, 1));

  EXPECT_THAT(tree.root(), t::Eq(&first_node));
  EXPECT_THAT(tree.First(), t::Eq(&first_node));
  EXPECT_THAT(tree.Last(), t::Eq(&first_node));
  EXPECT_THAT(Tree::Left(&first_node), t::IsNull());
  EXPECT_THAT(Tree::Right(&first_node), t::IsNull());
  EXPECT_THAT(Tree::Parent(&first_node), t::IsNull());
  EXPECT_THAT(Tree::Color(&first_node), t::Eq(RbColor::kBlack));
  EXPECT_THAT(tree.Find(50), t::Eq(&first_node));
  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(50));

  tree.Erase(first_node);
  EXPECT_TRUE(VerifyRbTree(tree, 0));
  EXPECT_THAT(CollectKeys(tree), t::IsEmpty());
}

TEST(RbTreeTest, RotationsAndMultiLevelInsertEraseFixup) {
  // LL (right rotation), RR (left rotation), LR (left-right double rotation),
  // and RL (right-left double rotation) cases.
  VerifyThreeNodeRotationCase(30, 20, 10);
  VerifyThreeNodeRotationCase(10, 20, 30);
  VerifyThreeNodeRotationCase(30, 10, 20);
  VerifyThreeNodeRotationCase(10, 30, 20);

  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;
  constexpr int kFixupKeys[] = {20, 10, 30, 5, 15, 25, 35, 2,
                                7,  12, 18, 1, 3,  6,  8};
  constexpr int64_t kFixupCount = sizeof(kFixupKeys) / sizeof(kFixupKeys[0]);
  BasicRbTestNode nodes[kFixupCount];

  for (int64_t i = 0; i < kFixupCount; ++i) {
    nodes[i] = {kFixupKeys[i], kFixupKeys[i] * 3, {}};
    ASSERT_TRUE(tree.Insert(&nodes[i]));
    ASSERT_TRUE(VerifyRbTree(tree, i + 1));
  }

  EXPECT_THAT(CollectKeys(tree), t::ElementsAre(1, 2, 3, 5, 6, 7, 8, 10, 12, 15,
                                                18, 20, 25, 30, 35));

  for (int64_t i = 0; i < kFixupCount; ++i) {
    BasicRbTestNode* const victim = ((i & 1) == 0) ? tree.root() : tree.First();
    tree.Erase(victim);
    ASSERT_TRUE(VerifyRbTree(tree, kFixupCount - 1 - i));
  }

  EXPECT_TRUE(tree.Empty());
}

TEST(RbTreeTest, QueriesAndBidirectionalIteration) {
  using Tree = RbTree<BasicRbTestNode, &BasicRbTestNode::node>;
  Tree tree;

  constexpr int kInsertOrder[] = {80, 40, 120, 20, 60,  100, 140, 10,
                                  30, 50, 70,  90, 110, 130, 150, 160};
  constexpr int64_t kCount = sizeof(kInsertOrder) / sizeof(kInsertOrder[0]);
  BasicRbTestNode nodes[kCount];

  for (int64_t i = 0; i < kCount; ++i) {
    nodes[i] = {kInsertOrder[i], static_cast<int>(i + 100), {}};
    ASSERT_TRUE(tree.Insert(&nodes[i]));
    ASSERT_TRUE(VerifyRbTree(tree, i + 1));
  }

  const Tree& const_tree = tree;

  // Below minimum key (10).
  EXPECT_THAT(tree.Find(0), t::IsNull());
  EXPECT_THAT(const_tree.Find(9), t::IsNull());
  ASSERT_THAT(tree.LowerBound(0), t::NotNull());
  EXPECT_THAT(tree.LowerBound(0)->key, t::Eq(10));
  ASSERT_THAT(const_tree.LowerBound(9), t::NotNull());
  EXPECT_THAT(const_tree.LowerBound(9)->key, t::Eq(10));
  ASSERT_THAT(tree.UpperBound(0), t::NotNull());
  EXPECT_THAT(tree.UpperBound(0)->key, t::Eq(10));
  ASSERT_THAT(const_tree.UpperBound(9), t::NotNull());
  EXPECT_THAT(const_tree.UpperBound(9)->key, t::Eq(10));

  // Exact keys (10, 20, ..., 160) and missing odd midpoint keys (15, 25, ...).
  for (int64_t idx = 1; idx <= kCount; ++idx) {
    const int exact_key = idx * 10;
    const BasicRbTestNode* const found = tree.Find(exact_key);
    const BasicRbTestNode* const cfound = const_tree.Find(exact_key);
    const BasicRbTestNode* const lb = tree.LowerBound(exact_key);
    const BasicRbTestNode* const clb = const_tree.LowerBound(exact_key);
    const BasicRbTestNode* const ub = tree.UpperBound(exact_key);
    const BasicRbTestNode* const cub = const_tree.UpperBound(exact_key);

    ASSERT_THAT(found, t::NotNull());
    EXPECT_THAT(found->key, t::Eq(exact_key));
    EXPECT_THAT(cfound, t::Eq(found));
    EXPECT_THAT(lb, t::Eq(found));
    EXPECT_THAT(clb, t::Eq(found));
    EXPECT_THAT(ub, t::Eq(cub));

    if (idx < kCount) {
      ASSERT_THAT(ub, t::NotNull());
      EXPECT_THAT(ub->key, t::Eq(exact_key + 10));

      const int mid_key = exact_key + 5;
      EXPECT_THAT(tree.Find(mid_key), t::IsNull());
      EXPECT_THAT(const_tree.Find(mid_key), t::IsNull());
      EXPECT_THAT(tree.LowerBound(mid_key), t::Eq(ub));
      EXPECT_THAT(const_tree.LowerBound(mid_key), t::Eq(ub));
      EXPECT_THAT(tree.UpperBound(mid_key), t::Eq(ub));
      EXPECT_THAT(const_tree.UpperBound(mid_key), t::Eq(ub));
    } else {
      EXPECT_THAT(ub, t::IsNull());
    }
  }

  // Above maximum key (160).
  EXPECT_THAT(tree.Find(161), t::IsNull());
  EXPECT_THAT(const_tree.Find(1000), t::IsNull());
  EXPECT_THAT(tree.LowerBound(161), t::IsNull());
  EXPECT_THAT(const_tree.LowerBound(1000), t::IsNull());
  EXPECT_THAT(tree.UpperBound(161), t::IsNull());
  EXPECT_THAT(const_tree.UpperBound(1000), t::IsNull());

  EXPECT_THAT(CollectKeys(const_tree),
              t::ElementsAre(10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120,
                             130, 140, 150, 160));

  EXPECT_THAT(CollectKeysReverse(const_tree),
              t::ElementsAre(160, 150, 140, 130, 120, 110, 100, 90, 80, 70, 60,
                             50, 40, 30, 20, 10));

  for (int64_t i = 0; i < kCount; ++i) {
    tree.Erase(&nodes[i]);
    ASSERT_TRUE(VerifyRbTree(tree, kCount - 1 - i));
  }
}

TEST(RbTreeTest, AugmentedBulkInsertAndErase) {
  using AugTree = RbTree<AugmentedRbTestNode,         //
                         &AugmentedRbTestNode::node,  //
                         AugmentedRbTestTraits>;
  AugTree tree;
  AugmentedRbTestNode nodes[kBulkTestNodeCount];

  // Phase 1: Ascending insert (0..127) and ascending erase (0..127).
  for (int64_t i = 0; i < kBulkTestNodeCount; ++i) {
    InitAugmentedTestNode(&nodes[i], i);
    ASSERT_TRUE(tree.Insert(&nodes[i]));
    ASSERT_TRUE(VerifyRbTree(tree, i + 1));
  }
  for (int64_t i = 0; i < kBulkTestNodeCount; ++i) {
    tree.Erase(&nodes[i]);
    ASSERT_TRUE(VerifyRbTree(tree, kBulkTestNodeCount - 1 - i));
  }

  // Phase 2: Descending insert (127..0) and descending erase (127..0).
  for (int64_t i = kBulkTestNodeCount; i > 0; --i) {
    const int64_t idx = i - 1;
    InitAugmentedTestNode(&nodes[idx], idx);
    const int64_t expected_count = kBulkTestNodeCount - idx;
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
    ASSERT_TRUE(VerifyRbTree(tree, expected_count));
  }
  for (int64_t i = kBulkTestNodeCount; i > 0; --i) {
    const int64_t idx = i - 1;
    tree.Erase(&nodes[idx]);
    ASSERT_TRUE(VerifyRbTree(tree, idx));
  }

  // Phase 3: Interleaved outside-in insert + continuous root deletion (forcing
  // two-child successor transplants and fixups at every step).
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    const int64_t idx =
        ((step & 1) == 0) ? (step / 2) : (kBulkTestNodeCount - 1 - step / 2);
    InitAugmentedTestNode(&nodes[idx], idx);
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
    ASSERT_TRUE(VerifyRbTree(tree, step + 1));
  }
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    AugmentedRbTestNode* const current_root = tree.root();
    ASSERT_THAT(current_root, t::NotNull());
    tree.Erase(current_root);
    ASSERT_TRUE(VerifyRbTree(tree, kBulkTestNodeCount - 1 - step));
  }

  // Phase 4: Pseudo-random permutation insert + independent pseudo-random
  // permutation erase (gcd(37, 128) == 1 and gcd(83, 128) == 1).
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    const int64_t idx = (step * 37 + 11) & (kBulkTestNodeCount - 1);
    InitAugmentedTestNode(&nodes[idx], idx);
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
    ASSERT_TRUE(VerifyRbTree(tree, step + 1));
  }
  for (int64_t step = 0; step < kBulkTestNodeCount; ++step) {
    const int64_t idx = (step * 83 + 59) & (kBulkTestNodeCount - 1);
    tree.Erase(&nodes[idx]);
    ASSERT_TRUE(VerifyRbTree(tree, kBulkTestNodeCount - 1 - step));
  }

  EXPECT_TRUE(tree.Empty());
}

TEST(RbTreeTest, PropagateAugmentAndAugmentedSearchWithMockPredicates) {
  using AugTree = RbTree<AugmentedRbTestNode,         //
                         &AugmentedRbTestNode::node,  //
                         AugmentedRbTestTraits>;
  AugTree tree;
  AugmentedRbTestNode nodes[kSearchTestNodeCount];

  // Verify FindFirstAugmented on an empty tree never invokes predicates.
  {
    t::MockFunction<bool(const AugmentedRbTestNode&)> empty_pred;
    EXPECT_CALL(empty_pred, Call).Times(0);
    EXPECT_THAT(tree.FindFirstAugmented(empty_pred.AsStdFunction(),
                                        empty_pred.AsStdFunction()),
                t::IsNull());
  }

  // Populate 256 nodes in pseudo-random order (gcd(73, 256) == 1).
  for (int64_t step = 0; step < kSearchTestNodeCount; ++step) {
    const int64_t idx = (step * 73 + 19) & (kSearchTestNodeCount - 1);
    InitAugmentedTestNode(&nodes[idx], idx);
    ASSERT_TRUE(tree.Insert(&nodes[idx]));
  }
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  // Test PropagateAugment after in-place payload_size mutations.
  const int64_t original_size_42 = nodes[42].payload_size;
  nodes[42].payload_size = 50000;
  tree.PropagateAugment(&nodes[42]);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Eq(50000));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  nodes[42].payload_size = 8;
  tree.PropagateAugment(nodes[42]);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Lt(50000));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  nodes[42].payload_size = original_size_42;
  tree.PropagateAugment(&nodes[42]);
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  // Mutate root in-place and propagate.
  AugmentedRbTestNode* const root_node = tree.root();
  const int64_t old_root_size = root_node->payload_size;
  root_node->payload_size = 12345;
  tree.PropagateAugment(root_node);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Eq(12345));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));
  root_node->payload_size = old_root_size;
  tree.PropagateAugment(root_node);

  // Place a unique peak size (20000) at the maximum-key node (index 255) so
  // any unpruned search for 20000 would have to visit all 256 nodes.
  nodes[kSearchTestNodeCount - 1].payload_size = 20000;
  tree.PropagateAugment(&nodes[kSearchTestNodeCount - 1]);
  EXPECT_THAT(tree.root()->subtree_max_size, t::Eq(20000));
  ASSERT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));

  constexpr int64_t kTargetSizes[] = {1,   8,   16,    100,   250,  400,
                                      500, 515, 20000, 20001, 99999};
  constexpr int64_t kNumTargets =
      sizeof(kTargetSizes) / sizeof(kTargetSizes[0]);
  const AugTree& const_tree = tree;

  for (int64_t t_idx = 0; t_idx < kNumTargets; ++t_idx) {
    const int64_t target = kTargetSizes[t_idx];

    // Ground-truth linear scan in key order (First -> Next).
    const AugmentedRbTestNode* expected_match = nullptr;
    for (const AugmentedRbTestNode* curr = const_tree.First(); curr != nullptr;
         curr = AugTree::Next(curr)) {
      if (curr->payload_size >= target) {
        expected_match = curr;
        break;
      }
    }

    t::MockFunction<bool(const AugmentedRbTestNode&)> subtree_pred;
    t::MockFunction<bool(const AugmentedRbTestNode&)> node_pred;

    if (target > const_tree.root()->subtree_max_size) {
      EXPECT_CALL(subtree_pred, Call)
          .WillOnce([&const_tree, target](const AugmentedRbTestNode& node) {
            EXPECT_THAT(&node, t::Eq(const_tree.root()));
            return node.subtree_max_size >= target;
          });
      EXPECT_CALL(node_pred, Call).Times(0);
    } else {
      // Verify O(log n) subtree pruning (tree height <= 18 for N = 256).
      EXPECT_CALL(subtree_pred, Call)
          .Times(t::Between(1, 40))
          .WillRepeatedly([target](const AugmentedRbTestNode& node) {
            EXPECT_THAT(node.subtree_count, t::Ge(1));
            return node.subtree_max_size >= target;
          });
      EXPECT_CALL(node_pred, Call)
          .Times(t::Between(1, 20))
          .WillRepeatedly([target](const AugmentedRbTestNode& node) {
            return node.payload_size >= target;
          });
    }

    const AugmentedRbTestNode* const found_two_pred = tree.FindFirstAugmented(
        subtree_pred.AsStdFunction(), node_pred.AsStdFunction());
    EXPECT_THAT(found_two_pred, t::Eq(expected_match));

    // Also verify the const overload on const_tree.
    const AugmentedRbTestNode* const found_const =
        const_tree.FindFirstAugmented(
            [target](const AugmentedRbTestNode& node) {
              return node.subtree_max_size >= target;
            },
            [target](const AugmentedRbTestNode& node) {
              return node.payload_size >= target;
            });
    EXPECT_THAT(found_const, t::Eq(expected_match));
  }

  EXPECT_TRUE(VerifyRbTree(tree, kSearchTestNodeCount));
}

class MockAugmentObserver {
 public:
  MOCK_METHOD(void, OnUpdateAugment,
              (int node_key, const int* left_key, const int* right_key));
};

struct ObservedRbTestNode {
  int key;
  std::shared_ptr<MockAugmentObserver> observer;
  RbNode node;
};

struct ObservedRbTestTraits {
  static int GetKey(const ObservedRbTestNode& item) { return item.key; }

  static void UpdateAugment(ObservedRbTestNode* const item,
                            const ObservedRbTestNode* const left,
                            const ObservedRbTestNode* const right) {
    if (item->observer != nullptr) {
      const int* const left_key = (left != nullptr) ? &left->key : nullptr;
      const int* const right_key = (right != nullptr) ? &right->key : nullptr;
      item->observer->OnUpdateAugment(item->key, left_key, right_key);
    }
  }
};

TEST(RbTreeTest, CustomTraitsInvokeAugmentCallbacksOnInsertAndPropagate) {
  const auto observer = std::make_shared<MockAugmentObserver>();
  RbTree<ObservedRbTestNode, &ObservedRbTestNode::node, ObservedRbTestTraits>
      tree;

  ObservedRbTestNode n10 = {10, observer, {}};
  ObservedRbTestNode n20 = {20, observer, {}};

  EXPECT_CALL(*observer, OnUpdateAugment)
      .WillOnce([](const int node_key,         //
                   const int* const left_key,  //
                   const int* const right_key) {
        EXPECT_THAT(node_key, t::Eq(10));
        EXPECT_THAT(left_key, t::IsNull());
        EXPECT_THAT(right_key, t::IsNull());
      })
      .WillRepeatedly([](const int node_key,         //
                         const int* const left_key,  //
                         const int* const right_key) {
        EXPECT_THAT(node_key, t::AnyOf(t::Eq(10), t::Eq(20)));
        (void)left_key;
        (void)right_key;
      });

  ASSERT_TRUE(tree.Insert(&n10));
  ASSERT_TRUE(tree.Insert(&n20));

  EXPECT_CALL(*observer, OnUpdateAugment)
      .WillOnce([](const int node_key,         //
                   const int* const left_key,  //
                   const int* const right_key) {
        EXPECT_THAT(node_key, t::Eq(20));
        EXPECT_THAT(left_key, t::IsNull());
        EXPECT_THAT(right_key, t::IsNull());
      })
      .WillOnce([](const int node_key,         //
                   const int* const left_key,  //
                   const int* const right_key) {
        EXPECT_THAT(node_key, t::Eq(10));
        EXPECT_THAT(left_key, t::IsNull());
        ASSERT_THAT(right_key, t::NotNull());
        EXPECT_THAT(*right_key, t::Eq(20));
      });

  tree.PropagateAugment(&n20);

  // Read-only FindFirstAugmented queries must not trigger observer callbacks.
  EXPECT_CALL(*observer, OnUpdateAugment).Times(0);
  const auto& const_tree = tree;
  EXPECT_THAT(const_tree.FindFirstAugmented(
                  [](const ObservedRbTestNode&) { return true; },
                  [](const ObservedRbTestNode& n) { return n.key >= 10; }),
              t::Eq(&n10));
  EXPECT_THAT(const_tree.FindFirstAugmented(
                  [](const ObservedRbTestNode&) { return true; },
                  [](const ObservedRbTestNode& n) { return n.key > 100; }),
              t::IsNull());
}

TEST(RbTreeTest, EraseUninsertedOrAlreadyErasedNodePreservesTreeInvariants) {
  RbTree<BasicRbTestNode, &BasicRbTestNode::node> tree;
  BasicRbTestNode inserted[8];
  for (int i = 0; i < 8; ++i) {
    inserted[i] = {(i + 1) * 10, i, {}};
    ASSERT_TRUE(tree.Insert(&inserted[i]));
  }
  ASSERT_TRUE(VerifyRbTree(tree, 8));

  BasicRbTestNode uninserted_red = {999, 99, {}};
  tree.Erase(&uninserted_red);
  tree.Erase(uninserted_red);
  ASSERT_TRUE(VerifyRbTree(tree, 8));

  BasicRbTestNode uninserted_black = {888, 88, {}};
  uninserted_black.node.set_color(RbColor::kBlack);
  tree.Erase(&uninserted_black);
  ASSERT_TRUE(VerifyRbTree(tree, 8));

  // Erase a real node, then erase it a second time after it has been removed.
  tree.Erase(&inserted[3]);
  ASSERT_TRUE(VerifyRbTree(tree, 7));
  tree.Erase(&inserted[3]);
  ASSERT_TRUE(VerifyRbTree(tree, 7));
}

struct NonCopyableAugmentedNode {
  int key = 0;
  int64_t payload_size = 0;
  int64_t subtree_max_size = 0;
  RbNode node = {};

  NonCopyableAugmentedNode() = default;
  NonCopyableAugmentedNode(const NonCopyableAugmentedNode&) = delete;
  NonCopyableAugmentedNode& operator=(const NonCopyableAugmentedNode&) = delete;
};

struct NonCopyableAugmentedTraits {
  static int GetKey(const NonCopyableAugmentedNode& item) { return item.key; }

  static bool Less(const int a, const int b) { return a < b; }

  static void UpdateAugment(NonCopyableAugmentedNode* const item,
                            const NonCopyableAugmentedNode* const left,
                            const NonCopyableAugmentedNode* const right) {
    int64_t max_size = item->payload_size;
    if (left != nullptr && left->subtree_max_size > max_size) {
      max_size = left->subtree_max_size;
    }
    if (right != nullptr && right->subtree_max_size > max_size) {
      max_size = right->subtree_max_size;
    }
    item->subtree_max_size = max_size;
  }
};

TEST(RbTreeTest, FindFirstAugmentedOnNonCopyableNodesIsConstAndThreadSafe) {
  using NonCopyTree = RbTree<NonCopyableAugmentedNode,         //
                             &NonCopyableAugmentedNode::node,  //
                             NonCopyableAugmentedTraits>;
  NonCopyTree tree;
  constexpr int kNodeCount = 64;
  NonCopyableAugmentedNode nodes[kNodeCount];
  for (int i = 0; i < kNodeCount; ++i) {
    nodes[i].key = (i + 1) * 10;
    nodes[i].payload_size = ((i * 37 + 11) % 200) + 10;
    ASSERT_TRUE(tree.Insert(&nodes[i]));
  }
  nodes[kNodeCount - 1].payload_size = 5000;
  tree.PropagateAugment(&nodes[kNodeCount - 1]);

  const NonCopyTree& const_tree = tree;
  constexpr int kNumThreads = 4;
  std::vector<std::thread> readers;
  readers.reserve(kNumThreads);
  for (int tid = 0; tid < kNumThreads; ++tid) {
    readers.emplace_back([&const_tree, &nodes]() {
      for (int iter = 0; iter < 50; ++iter) {
        const NonCopyableAugmentedNode* const match_max =
            const_tree.FindFirstAugmented(
                [](const NonCopyableAugmentedNode& n) {
                  return n.subtree_max_size >= 5000;
                },
                [](const NonCopyableAugmentedNode& n) {
                  return n.payload_size >= 5000;
                });
        EXPECT_THAT(match_max, t::Eq(&nodes[kNodeCount - 1]));

        const NonCopyableAugmentedNode* const match_min =
            const_tree.FindFirstAugmented(
                [](const NonCopyableAugmentedNode& n) {
                  return n.subtree_max_size >= 10;
                },
                [](const NonCopyableAugmentedNode& n) {
                  return n.payload_size >= 10;
                });
        EXPECT_THAT(match_min, t::Eq(&nodes[0]));

        const NonCopyableAugmentedNode* const match_none =
            const_tree.FindFirstAugmented(
                [](const NonCopyableAugmentedNode& n) {
                  return n.subtree_max_size >= 99999;
                },
                [](const NonCopyableAugmentedNode& n) {
                  return n.payload_size >= 99999;
                });
        EXPECT_THAT(match_none, t::IsNull());
      }
    });
  }
  for (auto& reader : readers) {
    reader.join();
  }
}

struct AddressKeyedAugmentedNode {
  int64_t payload_size = 0;
  int64_t subtree_max_size = 0;
  uintptr_t subtree_max_addr = 0;
  RbNode node = {};
};

struct AddressKeyedAugmentedTraits {
  static uintptr_t GetKey(const AddressKeyedAugmentedNode& item) {
    return reinterpret_cast<uintptr_t>(&item);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }

  static void UpdateAugment(AddressKeyedAugmentedNode* const item,
                            const AddressKeyedAugmentedNode* const left,
                            const AddressKeyedAugmentedNode* const right) {
    int64_t max_size = item->payload_size;
    uintptr_t max_addr = GetKey(*item);
    if (left != nullptr) {
      if (left->subtree_max_size > max_size) {
        max_size = left->subtree_max_size;
      }
      if (left->subtree_max_addr > max_addr) {
        max_addr = left->subtree_max_addr;
      }
    }
    if (right != nullptr) {
      if (right->subtree_max_size > max_size) {
        max_size = right->subtree_max_size;
      }
      if (right->subtree_max_addr > max_addr) {
        max_addr = right->subtree_max_addr;
      }
    }
    item->subtree_max_size = max_size;
    item->subtree_max_addr = max_addr;
  }
};

TEST(RbTreeTest, FindFirstAugmentedWorksWithAddressKeyedTraits) {
  using AddrTree = RbTree<AddressKeyedAugmentedNode,         //
                          &AddressKeyedAugmentedNode::node,  //
                          AddressKeyedAugmentedTraits>;
  AddrTree tree;
  constexpr int kCount = 32;
  AddressKeyedAugmentedNode nodes[kCount];
  for (int i = 0; i < kCount; ++i) {
    nodes[i].payload_size = (i + 1) * 16;
    ASSERT_TRUE(tree.Insert(&nodes[i]));
  }

  const AddrTree& const_tree = tree;
  for (int i = 0; i < kCount; ++i) {
    const int64_t target_size = (i + 1) * 16;
    const uintptr_t target_addr = reinterpret_cast<uintptr_t>(&nodes[i]);

    const AddressKeyedAugmentedNode* const by_size =
        const_tree.FindFirstAugmented(
            [target_size](const AddressKeyedAugmentedNode& n) {
              return n.subtree_max_size >= target_size;
            },
            [target_size](const AddressKeyedAugmentedNode& n) {
              return n.payload_size >= target_size;
            });
    EXPECT_THAT(by_size, t::Eq(&nodes[i]));

    const AddressKeyedAugmentedNode* const by_addr =
        const_tree.FindFirstAugmented(
            [target_addr](const AddressKeyedAugmentedNode& n) {
              return n.subtree_max_addr >= target_addr;
            },
            [target_addr](const AddressKeyedAugmentedNode& n) {
              return reinterpret_cast<uintptr_t>(&n) >= target_addr;
            });
    EXPECT_THAT(by_addr, t::Eq(&nodes[i]));
  }
}

struct NonTrivialDestructorAugmentedNode {
  int key = 0;
  int64_t payload_size = 0;
  int64_t subtree_max_size = 0;
  std::shared_ptr<int> lifetime_token;
  RbNode node = {};
};

struct NonTrivialDestructorAugmentedTraits {
  static int GetKey(const NonTrivialDestructorAugmentedNode& item) {
    return item.key;
  }

  static bool Less(const int a, const int b) { return a < b; }

  static void UpdateAugment(
      NonTrivialDestructorAugmentedNode* const item,
      const NonTrivialDestructorAugmentedNode* const left,
      const NonTrivialDestructorAugmentedNode* const right) {
    int64_t max_size = item->payload_size;
    if (left != nullptr && left->subtree_max_size > max_size) {
      max_size = left->subtree_max_size;
    }
    if (right != nullptr && right->subtree_max_size > max_size) {
      max_size = right->subtree_max_size;
    }
    item->subtree_max_size = max_size;
  }
};

struct TrivialObserverAugmentedNode {
  int key = 0;
  int64_t payload_size = 0;
  int64_t subtree_max_size = 0;
  int* observer_calls = nullptr;
  RbNode node = {};
};

struct TrivialObserverAugmentedTraits {
  static int GetKey(const TrivialObserverAugmentedNode& item) {
    return item.key;
  }

  static bool Less(const int a, const int b) { return a < b; }

  static void UpdateAugment(TrivialObserverAugmentedNode* const item,
                            const TrivialObserverAugmentedNode* const left,
                            const TrivialObserverAugmentedNode* const right) {
    if (item->observer_calls != nullptr) {
      ++(*item->observer_calls);
    }
    int64_t max_size = item->payload_size;
    if (left != nullptr && left->subtree_max_size > max_size) {
      max_size = left->subtree_max_size;
    }
    if (right != nullptr && right->subtree_max_size > max_size) {
      max_size = right->subtree_max_size;
    }
    item->subtree_max_size = max_size;
  }
};

TEST(RbTreeTest,
     FindFirstAugmentedHandlesNonTriviallyDestructibleAndTrivialObserverNodes) {
  // 1) Non-trivially-destructible augmented node (holding std::shared_ptr<int>)
  // must descend into right child when root's own payload_size is too small.
  using NonTrivialTree = RbTree<NonTrivialDestructorAugmentedNode,         //
                                &NonTrivialDestructorAugmentedNode::node,  //
                                NonTrivialDestructorAugmentedTraits>;
  NonTrivialTree nt_tree;
  const auto token = std::make_shared<int>(42);
  NonTrivialDestructorAugmentedNode nt10 = {10, 10, 10, token, {}};
  NonTrivialDestructorAugmentedNode nt20 = {20, 100, 100, token, {}};
  ASSERT_TRUE(nt_tree.Insert(&nt10));
  ASSERT_TRUE(nt_tree.Insert(&nt20));

  const NonTrivialTree& const_nt_tree = nt_tree;
  const NonTrivialDestructorAugmentedNode* const nt_found =
      const_nt_tree.FindFirstAugmented(
          [](const NonTrivialDestructorAugmentedNode& n) {
            return n.subtree_max_size >= 100;
          },
          [](const NonTrivialDestructorAugmentedNode& n) {
            return n.payload_size >= 100;
          });
  EXPECT_THAT(nt_found, t::Eq(&nt20));
  EXPECT_THAT(token.use_count(), t::Eq(3));

  // 2) Trivially-destructible node with raw observer counter pointer must never
  // invoke Traits::UpdateAugment during read-only FindFirstAugmented queries.
  using TrivialObsTree = RbTree<TrivialObserverAugmentedNode,         //
                                &TrivialObserverAugmentedNode::node,  //
                                TrivialObserverAugmentedTraits>;
  TrivialObsTree obs_tree;
  int observer_calls = 0;
  TrivialObserverAugmentedNode obs10 = {10, 10, 10, &observer_calls, {}};
  TrivialObserverAugmentedNode obs20 = {20, 100, 100, &observer_calls, {}};
  ASSERT_TRUE(obs_tree.Insert(&obs10));
  ASSERT_TRUE(obs_tree.Insert(&obs20));

  observer_calls = 0;
  const TrivialObsTree& const_obs_tree = obs_tree;
  const TrivialObserverAugmentedNode* const obs_found =
      const_obs_tree.FindFirstAugmented(
          [](const TrivialObserverAugmentedNode& n) {
            return n.subtree_max_size >= 100;
          },
          [](const TrivialObserverAugmentedNode& n) {
            return n.payload_size >= 100;
          });
  EXPECT_THAT(obs_found, t::Eq(&obs20));
  EXPECT_THAT(observer_calls, t::Eq(0));
}

struct DerivedAddressAugmentedNode {
  int64_t payload_size = 0;
  uintptr_t subtree_max_end = 0;
  RbNode node = {};
};

struct DerivedAddressAugmentedTraits {
  static uintptr_t GetKey(const DerivedAddressAugmentedNode& item) {
    return reinterpret_cast<uintptr_t>(&item);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }

  static void UpdateAugment(DerivedAddressAugmentedNode* const item,
                            const DerivedAddressAugmentedNode* const left,
                            const DerivedAddressAugmentedNode* const right) {
    uintptr_t max_end =
        GetKey(*item) + static_cast<uintptr_t>(item->payload_size);
    if (left != nullptr && left->subtree_max_end > max_end) {
      max_end = left->subtree_max_end;
    }
    if (right != nullptr && right->subtree_max_end > max_end) {
      max_end = right->subtree_max_end;
    }
    item->subtree_max_end = max_end;
  }
};

struct NonCopyableAddressKeyedNode {
  int64_t payload_size = 0;
  int64_t subtree_max_size = 0;
  RbNode node = {};

  NonCopyableAddressKeyedNode() = default;
  NonCopyableAddressKeyedNode(const NonCopyableAddressKeyedNode&) = delete;
  NonCopyableAddressKeyedNode& operator=(const NonCopyableAddressKeyedNode&) =
      delete;
};

struct NonCopyableAddressKeyedTraits {
  static uintptr_t GetKey(const NonCopyableAddressKeyedNode& item) {
    return reinterpret_cast<uintptr_t>(&item);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }

  static void UpdateAugment(NonCopyableAddressKeyedNode* const item,
                            const NonCopyableAddressKeyedNode* const left,
                            const NonCopyableAddressKeyedNode* const right) {
    int64_t max_size = item->payload_size;
    if (left != nullptr && left->subtree_max_size > max_size) {
      max_size = left->subtree_max_size;
    }
    if (right != nullptr && right->subtree_max_size > max_size) {
      max_size = right->subtree_max_size;
    }
    item->subtree_max_size = max_size;
  }
};

TEST(RbTreeTest,
     FindFirstAugmentedSupportsAddressDerivedEndAndNonCopyableAddrPredicates) {
  // 1) Address-keyed derived augmentation (`GetKey(*item) + payload_size`).
  using DerivedAddrTree = RbTree<DerivedAddressAugmentedNode,         //
                                 &DerivedAddressAugmentedNode::node,  //
                                 DerivedAddressAugmentedTraits>;
  DerivedAddrTree derived_tree;
  const auto derived_nodes = std::make_unique<DerivedAddressAugmentedNode[]>(2);
  derived_nodes[0].payload_size = 16;
  derived_nodes[1].payload_size = 64;
  ASSERT_TRUE(derived_tree.Insert(&derived_nodes[0]));
  ASSERT_TRUE(derived_tree.Insert(&derived_nodes[1]));

  const DerivedAddrTree& const_derived_tree = derived_tree;
  const uintptr_t target_end =
      reinterpret_cast<uintptr_t>(&derived_nodes[1]) + 64;
  const DerivedAddressAugmentedNode* const derived_found =
      const_derived_tree.FindFirstAugmented(
          [target_end](const DerivedAddressAugmentedNode& n) {
            return n.subtree_max_end >= target_end;
          },
          [target_end](const DerivedAddressAugmentedNode& n) {
            return reinterpret_cast<uintptr_t>(&n) +
                       static_cast<uintptr_t>(n.payload_size) >=
                   target_end;
          });
  EXPECT_THAT(derived_found, t::Eq(&derived_nodes[1]));

  // 2) Non-copyable address-keyed predicate inspecting `&n` must never receive
  // a stack copy address while locating the matching right-subtree node.
  using NonCopyAddrTree = RbTree<NonCopyableAddressKeyedNode,         //
                                 &NonCopyableAddressKeyedNode::node,  //
                                 NonCopyableAddressKeyedTraits>;
  NonCopyAddrTree nc_tree;
  constexpr int kCount = 8;
  NonCopyableAddressKeyedNode nc_nodes[kCount];
  for (int i = 0; i < kCount; ++i) {
    nc_nodes[i].payload_size = (i + 1) * 10;
    ASSERT_TRUE(nc_tree.Insert(&nc_nodes[i]));
  }

  const NonCopyAddrTree& const_nc_tree = nc_tree;
  bool saw_foreign_stack_copy = false;
  const NonCopyableAddressKeyedNode* const nc_found =
      const_nc_tree.FindFirstAugmented(
          [&nc_nodes,
           &saw_foreign_stack_copy](const NonCopyableAddressKeyedNode& n) {
            if (&n < &nc_nodes[0] || &n >= &nc_nodes[kCount]) {
              saw_foreign_stack_copy = true;
            }
            return n.subtree_max_size >= 80;
          },
          [&nc_nodes,
           &saw_foreign_stack_copy](const NonCopyableAddressKeyedNode& n) {
            if (&n < &nc_nodes[0] || &n >= &nc_nodes[kCount]) {
              saw_foreign_stack_copy = true;
            }
            return n.payload_size >= 80;
          });
  EXPECT_FALSE(saw_foreign_stack_copy);
  EXPECT_THAT(nc_found, t::Eq(&nc_nodes[kCount - 1]));
}

}  // namespace
}  // namespace protos
