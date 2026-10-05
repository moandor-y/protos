#ifndef PROTOS_RBTREE_H_
#define PROTOS_RBTREE_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace protos {

enum class RbColor : uint8_t {
  kRed = 0,
  kBlack = 1,
};

struct RbNode {
  RbNode* parent = nullptr;
  RbNode* left = nullptr;
  RbNode* right = nullptr;
  RbColor color = RbColor::kRed;
};

struct DefaultRbTraits {
  template <typename T>
  static constexpr auto GetKey(const T& item) {
    if constexpr (requires { item.key; }) {
      return item.key;
    } else if constexpr (requires { item.key(); }) {
      return item.key();
    } else {
      return item;
    }
  }

  template <typename Key>
  static constexpr bool Less(const Key& a, const Key& b) {
    return a < b;
  }

  template <typename T>
  static constexpr bool UpdateAugment(T& node,              //
                                      const T* const left,  //
                                      const T* const right) {
    (void)node;
    (void)left;
    (void)right;
    return false;
  }

  template <typename T>
  static constexpr bool UpdateAugment(T* const node,        //
                                      const T* const left,  //
                                      const T* const right) {
    (void)node;
    (void)left;
    (void)right;
    return false;
  }
};

template <typename T, RbNode T::* kNodeMember,
          typename Traits = DefaultRbTraits>
class RbTree {
 private:
  static constexpr decltype(auto) ExtractKey(const T& item) {
    if constexpr (requires { Traits::GetKey(item); }) {
      return Traits::GetKey(item);
    } else if constexpr (requires { Traits::GetKey(&item); }) {
      return Traits::GetKey(&item);
    } else {
      return DefaultRbTraits::GetKey(item);
    }
  }

 public:
  using KeyType = std::remove_cvref_t<decltype(ExtractKey(
      *static_cast<const T*>(nullptr)))>;

  constexpr RbTree() = default;
  RbTree(const RbTree&) = delete;
  RbTree& operator=(const RbTree&) = delete;

  bool Empty() const { return root_ == nullptr; }

  void Clear() { root_ = nullptr; }

  T* root() { return NodeToItem(root_); }
  const T* root() const { return NodeToItem(root_); }

  static T* Left(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->left);
  }
  static const T* Left(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->left);
  }

  static T* Right(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->right);
  }
  static const T* Right(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->right);
  }

  static T* Parent(T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->parent);
  }
  static const T* Parent(const T* const item) {
    return (item == nullptr) ? nullptr : NodeToItem(ItemToNode(item)->parent);
  }

  static RbColor Color(const T* const item) {
    return (item == nullptr) ? RbColor::kBlack : ItemToNode(item)->color;
  }

  T* First() { return NodeToItem(MinimumNode(root_)); }
  const T* First() const { return NodeToItem(MinimumNode(root_)); }

  T* Last() { return NodeToItem(MaximumNode(root_)); }
  const T* Last() const { return NodeToItem(MaximumNode(root_)); }

  static T* Next(T* const item) {
    if (item == nullptr) {
      return nullptr;
    }
    return NodeToItem(NextNode(ItemToNode(item)));
  }
  static const T* Next(const T* const item) {
    if (item == nullptr) {
      return nullptr;
    }
    return NodeToItem(NextNode(ItemToNode(item)));
  }

  static T* Prev(T* const item) {
    if (item == nullptr) {
      return nullptr;
    }
    return NodeToItem(PrevNode(ItemToNode(item)));
  }
  static const T* Prev(const T* const item) {
    if (item == nullptr) {
      return nullptr;
    }
    return NodeToItem(PrevNode(ItemToNode(item)));
  }

  bool Insert(T& item) { return Insert(&item); }

  bool Insert(T* const item) {
    if (item == nullptr) {
      return false;
    }
    const auto z_key = ExtractKey(*item);
    RbNode* parent = nullptr;
    RbNode* curr = root_;
    bool insert_left = false;

    while (curr != nullptr) {
      parent = curr;
      const T* const curr_item = NodeToItem(curr);
      const auto curr_key = ExtractKey(*curr_item);
      if (KeyLess(z_key, curr_key)) {
        curr = curr->left;
        insert_left = true;
      } else if (KeyLess(curr_key, z_key)) {
        curr = curr->right;
        insert_left = false;
      } else {
        return false;
      }
    }

    RbNode* const z = ItemToNode(item);
    z->parent = parent;
    z->left = nullptr;
    z->right = nullptr;
    z->color = RbColor::kRed;

    if (parent == nullptr) {
      root_ = z;
    } else if (insert_left) {
      parent->left = z;
    } else {
      parent->right = z;
    }

    PropagateNodeAugmentToRoot(z);
    InsertFixup(z);
    return true;
  }

  void Erase(T& item) { Erase(&item); }

  void Erase(T* const item) {
    if (item == nullptr || root_ == nullptr) {
      return;
    }
    RbNode* const z = ItemToNode(item);
    RbNode* y = z;
    RbColor y_original_color = y->color;
    RbNode* x = nullptr;
    RbNode* x_parent = nullptr;

    if (z->left == nullptr) {
      x = z->right;
      x_parent = z->parent;
      Transplant(z, z->right);
    } else if (z->right == nullptr) {
      x = z->left;
      x_parent = z->parent;
      Transplant(z, z->left);
    } else {
      y = MinimumNode(z->right);
      y_original_color = y->color;
      x = y->right;
      if (y->parent == z) {
        x_parent = y;
      } else {
        x_parent = y->parent;
        Transplant(y, y->right);
        y->right = z->right;
        y->right->parent = y;
      }
      Transplant(z, y);
      y->left = z->left;
      y->left->parent = y;
      y->color = z->color;
    }

    z->parent = nullptr;
    z->left = nullptr;
    z->right = nullptr;
    z->color = RbColor::kRed;

    PropagateNodeAugmentToRoot(x_parent);

    if (y_original_color == RbColor::kBlack) {
      EraseFixup(x, x_parent);
    }
  }

  // Recomputes and propagates subtree augmentation data from `item` up to the
  // root.
  //
  // NOTE: `Insert` and `Erase` automatically maintain and propagate subtree
  // augmentation across all affected ancestors and rotations, so callers do NOT
  // need to call `PropagateAugment` after `Insert` or `Erase`.
  //
  // Call `PropagateAugment` manually ONLY after mutating an already-inserted
  // node's augmented/payload fields in place (without changing its ordering
  // key) while the node remains in the tree. If a node's ordering key changes,
  // it must instead be removed via `Erase` and re-inserted via `Insert`.
  void PropagateAugment(T& item) { PropagateAugment(&item); }

  void PropagateAugment(T* const item) {
    if (item == nullptr) {
      return;
    }
    PropagateNodeAugmentToRoot(ItemToNode(item));
  }

  T* Find(const KeyType& key) {
    return const_cast<T*>(static_cast<const RbTree*>(this)->Find(key));
  }

  const T* Find(const KeyType& key) const {
    const RbNode* curr = root_;
    while (curr != nullptr) {
      const T* const curr_item = NodeToItem(curr);
      const KeyType curr_key = ExtractKey(*curr_item);
      if (KeyLess(key, curr_key)) {
        curr = curr->left;
      } else if (KeyLess(curr_key, key)) {
        curr = curr->right;
      } else {
        return curr_item;
      }
    }
    return nullptr;
  }

  T* LowerBound(const KeyType& key) {
    return const_cast<T*>(static_cast<const RbTree*>(this)->LowerBound(key));
  }

  const T* LowerBound(const KeyType& key) const {
    const RbNode* curr = root_;
    const T* result = nullptr;
    while (curr != nullptr) {
      const T* const curr_item = NodeToItem(curr);
      const KeyType curr_key = ExtractKey(*curr_item);
      if (!KeyLess(curr_key, key)) {
        result = curr_item;
        curr = curr->left;
      } else {
        curr = curr->right;
      }
    }
    return result;
  }

  T* UpperBound(const KeyType& key) {
    return const_cast<T*>(static_cast<const RbTree*>(this)->UpperBound(key));
  }

  const T* UpperBound(const KeyType& key) const {
    const RbNode* curr = root_;
    const T* result = nullptr;
    while (curr != nullptr) {
      const T* const curr_item = NodeToItem(curr);
      const KeyType curr_key = ExtractKey(*curr_item);
      if (KeyLess(key, curr_key)) {
        result = curr_item;
        curr = curr->left;
      } else {
        curr = curr->right;
      }
    }
    return result;
  }

  template <typename SubtreePred, typename NodePred>
  T* FindFirstAugmented(const SubtreePred& subtree_pred,
                        const NodePred& node_pred) {
    return const_cast<T*>(static_cast<const RbTree*>(this)->FindFirstAugmented(
        subtree_pred, node_pred));
  }

  template <typename SubtreePred, typename NodePred>
  const T* FindFirstAugmented(const SubtreePred& subtree_pred,
                              const NodePred& node_pred) const {
    return FindFirstAugmentedTwoPred(root_, subtree_pred, node_pred);
  }

  template <typename SubtreePred>
  T* FindFirstAugmented(const SubtreePred& subtree_pred) {
    return const_cast<T*>(
        static_cast<const RbTree*>(this)->FindFirstAugmented(subtree_pred));
  }

  template <typename SubtreePred>
  const T* FindFirstAugmented(const SubtreePred& subtree_pred) const {
    const auto node_pred = [&subtree_pred](const T* const item) -> bool {
      if constexpr (std::is_copy_constructible_v<T>) {
        T single = *item;
        InvokeUpdateAugmentOnItem(&single, nullptr, nullptr);
        return InvokePred(subtree_pred, &single);
      } else {
        T* const mutable_item = const_cast<T*>(item);
        const RbNode* const node = ItemToNode(item);
        InvokeUpdateAugmentOnItem(mutable_item, nullptr, nullptr);
        const bool matched = InvokePred(subtree_pred, mutable_item);
        InvokeUpdateAugmentOnItem(mutable_item,            //
                                  NodeToItem(node->left),  //
                                  NodeToItem(node->right));
        return matched;
      }
    };
    return FindFirstAugmentedTwoPred(root_, subtree_pred, node_pred);
  }

 private:
  RbNode* root_ = nullptr;

  static uintptr_t NodeOffset() {
    constexpr uintptr_t kDummyBase = 0x10000;
    const T* const dummy = reinterpret_cast<const T*>(kDummyBase);
    const RbNode* const member = &(dummy->*kNodeMember);
    return reinterpret_cast<uintptr_t>(member) - kDummyBase;
  }

  static RbNode* ItemToNode(T* const item) { return &(item->*kNodeMember); }

  static const RbNode* ItemToNode(const T* const item) {
    return &(item->*kNodeMember);
  }

  static T* NodeToItem(RbNode* const node) {
    if (node == nullptr) {
      return nullptr;
    }
    return reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(node) -
                                NodeOffset());
  }

  static const T* NodeToItem(const RbNode* const node) {
    if (node == nullptr) {
      return nullptr;
    }
    return reinterpret_cast<const T*>(reinterpret_cast<uintptr_t>(node) -
                                      NodeOffset());
  }

  template <typename KeyA, typename KeyB>
  static constexpr bool KeyLess(const KeyA& a, const KeyB& b) {
    if constexpr (requires { Traits::Less(a, b); }) {
      return Traits::Less(a, b);
    } else if constexpr (requires { Traits::Compare(a, b); }) {
      return Traits::Compare(a, b) < 0;
    } else {
      return a < b;
    }
  }

  static bool IsRed(const RbNode* const node) {
    return node != nullptr && node->color == RbColor::kRed;
  }

  static bool IsBlack(const RbNode* const node) {
    return node == nullptr || node->color == RbColor::kBlack;
  }

  static RbNode* MinimumNode(RbNode* const start) {
    RbNode* curr = start;
    if (curr == nullptr) {
      return nullptr;
    }
    while (curr->left != nullptr) {
      curr = curr->left;
    }
    return curr;
  }

  static const RbNode* MinimumNode(const RbNode* const start) {
    const RbNode* curr = start;
    if (curr == nullptr) {
      return nullptr;
    }
    while (curr->left != nullptr) {
      curr = curr->left;
    }
    return curr;
  }

  static RbNode* MaximumNode(RbNode* const start) {
    RbNode* curr = start;
    if (curr == nullptr) {
      return nullptr;
    }
    while (curr->right != nullptr) {
      curr = curr->right;
    }
    return curr;
  }

  static const RbNode* MaximumNode(const RbNode* const start) {
    const RbNode* curr = start;
    if (curr == nullptr) {
      return nullptr;
    }
    while (curr->right != nullptr) {
      curr = curr->right;
    }
    return curr;
  }

  static RbNode* NextNode(RbNode* const start) {
    RbNode* curr = start;
    if (curr->right != nullptr) {
      return MinimumNode(curr->right);
    }
    RbNode* parent = curr->parent;
    while (parent != nullptr && curr == parent->right) {
      curr = parent;
      parent = parent->parent;
    }
    return parent;
  }

  static const RbNode* NextNode(const RbNode* const start) {
    const RbNode* curr = start;
    if (curr->right != nullptr) {
      return MinimumNode(curr->right);
    }
    const RbNode* parent = curr->parent;
    while (parent != nullptr && curr == parent->right) {
      curr = parent;
      parent = parent->parent;
    }
    return parent;
  }

  static RbNode* PrevNode(RbNode* const start) {
    RbNode* curr = start;
    if (curr->left != nullptr) {
      return MaximumNode(curr->left);
    }
    RbNode* parent = curr->parent;
    while (parent != nullptr && curr == parent->left) {
      curr = parent;
      parent = parent->parent;
    }
    return parent;
  }

  static const RbNode* PrevNode(const RbNode* const start) {
    const RbNode* curr = start;
    if (curr->left != nullptr) {
      return MaximumNode(curr->left);
    }
    const RbNode* parent = curr->parent;
    while (parent != nullptr && curr == parent->left) {
      curr = parent;
      parent = parent->parent;
    }
    return parent;
  }

  static bool InvokeUpdateAugmentOnItem(T* const item,        //
                                        const T* const left,  //
                                        const T* const right) {
    if constexpr (requires { Traits::UpdateAugment(*item, left, right); }) {
      if constexpr (std::is_same_v<decltype(Traits::UpdateAugment(*item,  //
                                                                  left,   //
                                                                  right)),
                                   bool>) {
        return Traits::UpdateAugment(*item, left, right);
      } else {
        Traits::UpdateAugment(*item, left, right);
        return true;
      }
    } else if constexpr (requires {
                           Traits::UpdateAugment(item, left, right);
                         }) {
      if constexpr (std::is_same_v<decltype(Traits::UpdateAugment(item,  //
                                                                  left,  //
                                                                  right)),
                                   bool>) {
        return Traits::UpdateAugment(item, left, right);
      } else {
        Traits::UpdateAugment(item, left, right);
        return true;
      }
    } else if constexpr (requires { Traits::UpdateAugment(*item); }) {
      if constexpr (std::is_same_v<decltype(Traits::UpdateAugment(*item)),
                                   bool>) {
        return Traits::UpdateAugment(*item);
      } else {
        Traits::UpdateAugment(*item);
        return true;
      }
    } else if constexpr (requires { Traits::UpdateAugment(item); }) {
      if constexpr (std::is_same_v<decltype(Traits::UpdateAugment(item)),
                                   bool>) {
        return Traits::UpdateAugment(item);
      } else {
        Traits::UpdateAugment(item);
        return true;
      }
    } else {
      return false;
    }
  }

  static bool UpdateNodeAugment(RbNode* const node) {
    if (node == nullptr) {
      return false;
    }
    return InvokeUpdateAugmentOnItem(NodeToItem(node),        //
                                     NodeToItem(node->left),  //
                                     NodeToItem(node->right));
  }

  static void PropagateNodeAugmentToRoot(RbNode* const start) {
    RbNode* curr = start;
    while (curr != nullptr) {
      UpdateNodeAugment(curr);
      curr = curr->parent;
    }
  }

  void Transplant(RbNode* const u, RbNode* const v) {
    if (u->parent == nullptr) {
      root_ = v;
    } else if (u == u->parent->left) {
      u->parent->left = v;
    } else {
      u->parent->right = v;
    }
    if (v != nullptr) {
      v->parent = u->parent;
    }
  }

  void RotateLeft(RbNode* const x) {
    RbNode* const y = x->right;
    x->right = y->left;
    if (y->left != nullptr) {
      y->left->parent = x;
    }
    y->parent = x->parent;
    if (x->parent == nullptr) {
      root_ = y;
    } else if (x == x->parent->left) {
      x->parent->left = y;
    } else {
      x->parent->right = y;
    }
    y->left = x;
    x->parent = y;

    UpdateNodeAugment(x);
    UpdateNodeAugment(y);
  }

  void RotateRight(RbNode* const x) {
    RbNode* const y = x->left;
    x->left = y->right;
    if (y->right != nullptr) {
      y->right->parent = x;
    }
    y->parent = x->parent;
    if (x->parent == nullptr) {
      root_ = y;
    } else if (x == x->parent->right) {
      x->parent->right = y;
    } else {
      x->parent->left = y;
    }
    y->right = x;
    x->parent = y;

    UpdateNodeAugment(x);
    UpdateNodeAugment(y);
  }

  void InsertFixup(RbNode* const start) {
    RbNode* z = start;
    while (IsRed(z->parent)) {
      RbNode* const p = z->parent;
      RbNode* const g = p->parent;
      if (p == g->left) {
        RbNode* const uncle = g->right;
        if (IsRed(uncle)) {
          p->color = RbColor::kBlack;
          uncle->color = RbColor::kBlack;
          g->color = RbColor::kRed;
          z = g;
        } else {
          if (z == p->right) {
            z = p;
            RotateLeft(z);
          }
          z->parent->color = RbColor::kBlack;
          g->color = RbColor::kRed;
          RotateRight(g);
        }
      } else {
        RbNode* const uncle = g->left;
        if (IsRed(uncle)) {
          p->color = RbColor::kBlack;
          uncle->color = RbColor::kBlack;
          g->color = RbColor::kRed;
          z = g;
        } else {
          if (z == p->left) {
            z = p;
            RotateRight(z);
          }
          z->parent->color = RbColor::kBlack;
          g->color = RbColor::kRed;
          RotateLeft(g);
        }
      }
    }
    root_->color = RbColor::kBlack;
    PropagateNodeAugmentToRoot(z);
  }

  void EraseFixup(RbNode* const start_x, RbNode* const start_x_parent) {
    RbNode* x = start_x;
    RbNode* x_parent = start_x_parent;
    RbNode* last_touched = x_parent;
    while (x != root_ && IsBlack(x)) {
      if (x == x_parent->left) {
        RbNode* w = x_parent->right;
        if (IsRed(w)) {
          w->color = RbColor::kBlack;
          x_parent->color = RbColor::kRed;
          RotateLeft(x_parent);
          w = x_parent->right;
        }
        if (IsBlack(w->left) && IsBlack(w->right)) {
          w->color = RbColor::kRed;
          x = x_parent;
          x_parent = x->parent;
          last_touched = x;
        } else {
          if (IsBlack(w->right)) {
            if (w->left != nullptr) {
              w->left->color = RbColor::kBlack;
            }
            w->color = RbColor::kRed;
            RotateRight(w);
            w = x_parent->right;
          }
          w->color = x_parent->color;
          x_parent->color = RbColor::kBlack;
          if (w->right != nullptr) {
            w->right->color = RbColor::kBlack;
          }
          RotateLeft(x_parent);
          last_touched = w;
          x = root_;
          x_parent = nullptr;
        }
      } else {
        RbNode* w = x_parent->left;
        if (IsRed(w)) {
          w->color = RbColor::kBlack;
          x_parent->color = RbColor::kRed;
          RotateRight(x_parent);
          w = x_parent->left;
        }
        if (IsBlack(w->right) && IsBlack(w->left)) {
          w->color = RbColor::kRed;
          x = x_parent;
          x_parent = x->parent;
          last_touched = x;
        } else {
          if (IsBlack(w->left)) {
            if (w->right != nullptr) {
              w->right->color = RbColor::kBlack;
            }
            w->color = RbColor::kRed;
            RotateLeft(w);
            w = x_parent->left;
          }
          w->color = x_parent->color;
          x_parent->color = RbColor::kBlack;
          if (w->left != nullptr) {
            w->left->color = RbColor::kBlack;
          }
          RotateRight(x_parent);
          last_touched = w;
          x = root_;
          x_parent = nullptr;
        }
      }
    }
    if (x != nullptr) {
      x->color = RbColor::kBlack;
    }
    PropagateNodeAugmentToRoot(last_touched);
  }

  template <typename Pred>
  static bool InvokePred(const Pred& pred, const T* const item) {
    if constexpr (requires { pred(*item); }) {
      return static_cast<bool>(pred(*item));
    } else {
      return static_cast<bool>(pred(item));
    }
  }

  template <typename SubtreePred, typename NodePred>
  static const T* FindFirstAugmentedTwoPred(const RbNode* const node,
                                            const SubtreePred& subtree_pred,
                                            const NodePred& node_pred) {
    if (node == nullptr) {
      return nullptr;
    }
    const T* const item = NodeToItem(node);
    if (!InvokePred(subtree_pred, item)) {
      return nullptr;
    }
    if (node->left != nullptr) {
      const T* const left_match =
          FindFirstAugmentedTwoPred(node->left, subtree_pred, node_pred);
      if (left_match != nullptr) {
        return left_match;
      }
    }
    if (InvokePred(node_pred, item)) {
      return item;
    }
    return FindFirstAugmentedTwoPred(node->right, subtree_pred, node_pred);
  }
};

}  // namespace protos

#endif  // PROTOS_RBTREE_H_
