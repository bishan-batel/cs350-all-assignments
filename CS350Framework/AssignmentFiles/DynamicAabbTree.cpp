/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: DynamicAabbTree.cpp
Purpose: Dynamic AABB tree impl1
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_3
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"
#include "Precompiled.hpp"

#include <cassert>
#include <queue>

DynamicAabbTree::DynamicAabbTree() { mType = SpatialPartitionTypes::AabbTree; }

DynamicAabbTree::~DynamicAabbTree() = default;

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) {
  key.mVoidKey = data.mClientData;

  std::unique_ptr<Node> node = create_leaf(key, data);

  // trivial case, tree is empty
  if (root == nullptr) {
    root = std::move(node);
    return;
  }

  // if root is the only node then we need to split it
  if (root->is_leaf()) {
    auto split_root = std::make_unique<Node>();
    split_root->left = std::move(root);

    split_root->left->parent = split_root.get();

    split_root->right = std::move(node);
    split_root->right->parent = split_root.get();

    root = std::move(split_root);

    root->parent = nullptr;
    root->recalculate_aabb();
    root->propagate_depth();
    return;
  }

  Node& node_ref = *node;

  root->insert(node);
  root->recalculate_aabb();
  root->propagate_depth();
  rebalance(node_ref);
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) {
  const Node* node = get_node(key);

  if (node->aabb.Contains(data.mAabb)) { return; }

  RemoveData(key);
  InsertData(key, data);
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key) {
  Node* to_remove = get_node(key);

  // if only one node then only one choice to remove
  if (to_remove == root.get()) {
    root = {};
    nodes.clear();
    return;
  }

  // if the root isn't a leaf it will never contain an object's data so this is safe
  Node& parent = *to_remove->parent;

  if (parent.parent == nullptr) {
    root = std::move(to_remove->sibling());
    root->parent = nullptr;
    root->propagate_depth();
    return;
  }

  // transfer all properties of sibling to the parent
  std::unique_ptr<Node> sibling = std::move(to_remove->sibling());

  sibling->parent = nullptr;
  parent.aabb = sibling->aabb;
  parent.data = sibling->data;

  if (sibling->is_leaf()) {
    parent.left = {};
    parent.right = {};
    nodes.at(sibling->data) = &parent;
  } else {
    parent.left = std::move(sibling->left);
    parent.left->parent = &parent;
    parent.right = std::move(sibling->right);
    parent.right->parent = &parent;
  }

  // after this point the original 'to_remove' node will have been released
  nodes.erase(nodes.find(key.mVoidKey));
  root->propagate_depth();
  root->recalculate_aabb();
  rebalance(parent);
}

void DynamicAabbTree::DebugDraw(int level, const Matrix4& transform, const Vector4& color, int bitMask) {
  if (!root) { return; }
  if (level < 0) { level = INT_MAX; }
  root->debug_draw(level, transform, color, bitMask);
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results) {
  if (root) { root->cast_ray(ray, results); }
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results) {
  if (root) { root->cast_frustum(frustum, results); }
}

void DynamicAabbTree::SelfQuery(QueryResults& results) {
  if (!root) { return; }
  root->self_query(results);
}

void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const {
  if (root) { root->fillout_data(results); }
}

void DynamicAabbTree::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const {
  const auto node = get_node(key);
  data.mClientData = node->data;
  data.mAabb = node->aabb;
}

auto DynamicAabbTree::Node::propagate_depth() -> void {
  if (parent == nullptr) {
    height = 0;
    depth = 0;
  } else {
    depth = parent->depth + 1;
  }

  if (is_leaf()) {
    height = 0;
    return;
  }

  data = nullptr;
  left->propagate_depth();
  right->propagate_depth();

  height = 1 + std::max(left->height, right->height);
}

auto DynamicAabbTree::Node::recalculate_aabb() -> void {
  if (is_leaf()) { return; }

  left->recalculate_aabb();
  right->recalculate_aabb();

  aabb = Aabb::Combine(left->aabb, right->aabb);
}

auto DynamicAabbTree::Node::is_leaf() const -> bool { return left == nullptr && right == nullptr; }

auto DynamicAabbTree::Node::select_node(Node& choice1, Node& choice2) const -> Node& {
  const float delta1{
    Aabb::Combine(choice1.aabb, aabb).GetSurfaceArea() - choice1.aabb.GetSurfaceArea(),
  };
  const float delta2{
    Aabb::Combine(choice2.aabb, aabb).GetSurfaceArea() - choice2.aabb.GetSurfaceArea(),
  };

  if (delta1 < delta2) { return choice1; }
  return choice2;
}

auto DynamicAabbTree::Node::insert(std::unique_ptr<Node>& node) -> void {
  assert(!(is_leaf() && parent == nullptr));

  if (is_leaf()) {
    // parent of the node we are splitting
    Node& p = *parent;

    // position of the node we splitting
    std::unique_ptr<Node>& split = p.child(*this);

    // old leaf
    std::unique_ptr<Node> old_leaf = std::move(split);

    // where we were is replaced with a non-leaf node
    split = std::make_unique<Node>();  // reassigns link in 'p'
    split->left = std::move(old_leaf); // left of new node is the old one
    split->left->parent = split.get();

    split->right = std::move(node); // right of new node is the inserted
    split->right->parent = split.get();
    split->parent = &p;

    p.propagate_depth();
    p.recalculate_aabb();
    return;
  }

  node->select_node(*left, *right).insert(node);
}

auto DynamicAabbTree::Node::debug_draw(int level, const Matrix4& transform, const Vector4& color, int bitMask) const
  -> void {
  if (static_cast<int>(depth) > level) { return; }

  gDebugDrawer->DrawAabb(aabb)
    .SetTransform(transform)
    .Color(is_leaf() ? color : Vector4{1.0f, 0.0f, 0.0f, 1.0f})
    .SetMaskBit(bitMask);

  if (left) { left->debug_draw(level, transform, color, bitMask); }
  if (right) { right->debug_draw(level, transform, color, bitMask); }
}

auto DynamicAabbTree::Node::cast_frustum(const Frustum& frustum, CastResults& results) const -> void {
  IntersectionType::Type type = FrustumAabb(frustum.GetPlanes(), aabb.GetMin(), aabb.GetMax(), last_axis);

  if (type == IntersectionType::Inside) {
    cast_frustum_fully_contained(results);
    return;
  }

  if (type == IntersectionType::Outside) { return; }

  if (is_leaf()) {
    cast_frustum_fully_contained(results);
    return;
  }

  left->cast_frustum(frustum, results);
  right->cast_frustum(frustum, results);
}

auto DynamicAabbTree::Node::cast_frustum_fully_contained(CastResults& results) const -> void {
  if (is_leaf()) {
    results.AddResult(CastResult{data});
    return;
  }
  left->cast_frustum_fully_contained(results);
  right->cast_frustum_fully_contained(results);
}

auto DynamicAabbTree::Node::cast_ray(const Ray& ray, CastResults& results) const -> bool {
  float time = 0;

  if (!ray_aabb(ray, time)) { return false; }

  if (is_leaf()) {
    results.AddResult(CastResult{data, time});
    return true;
  }

  left->cast_ray(ray, results);
  right->cast_ray(ray, results);
  return false;
}

auto DynamicAabbTree::Node::ray_aabb(const Ray& ray, float& t) const -> bool {
  return RayAabb(ray.mStart, ray.mDirection, aabb.GetMin(), aabb.GetMax(), t);
}

auto DynamicAabbTree::Node::overlaps(const Node& other) const -> bool {
  return AabbAabb(aabb.GetMin(), aabb.GetMax(), other.aabb.GetMin(), other.aabb.GetMax());
}

auto DynamicAabbTree::Node::sibling() const -> std::unique_ptr<Node>& {
  assert(parent);
  if (parent->left.get() == this) { return parent->right; }
  return parent->left;
}

auto DynamicAabbTree::Node::grow() -> void {
  const Vector3 center = aabb.GetCenter();
  const Vector3 size = aabb.GetHalfSize() * mFatteningFactor;
  aabb = Aabb::BuildFromCenterAndHalfExtents(center, size);
}

void DynamicAabbTree::Node::fillout_data(std::vector<SpatialPartitionQueryData>& results) const {
  SpatialPartitionQueryData query;
  query.mDepth = static_cast<int>(depth);
  query.mAabb = aabb;
  query.mClientData = data;
  results.push_back(query);

  if (left) { left->fillout_data(results); }
  if (right) { right->fillout_data(results); }
}

auto DynamicAabbTree::Node::subtree_size() const -> size_t {
  if (is_leaf()) { return 1; }
  return left->subtree_size() + right->subtree_size() + 1;
}

auto DynamicAabbTree::Node::child(const Node& node) -> std::unique_ptr<Node>& {
  if (left.get() == &node) { return left; }
  return right;
}

auto DynamicAabbTree::Node::balance() const -> int {
  if (is_leaf()) { return 0; }

  int balance = static_cast<int>(right->height);
  balance -= static_cast<int>(left->height);

  return balance;
}

auto DynamicAabbTree::Node::self_query(const Node* left, const Node* right, QueryResults& results) -> void {
  if (!left->overlaps(*right)) { return; }

  if (left->is_leaf() && right->is_leaf()) {
    results.AddResult(QueryResult{left->data, right->data});
    return;
  }

  split_nodes(left, right, results);
}

auto DynamicAabbTree::Node::split_nodes(const Node* left, const Node* right, QueryResults& results) -> void {
  if (right->is_leaf()) {
    self_query(left->left.get(), right, results);
    self_query(left->right.get(), right, results);
    return;
  }

  if (left->is_leaf()) {
    self_query(left, right->left.get(), results);
    self_query(left, right->right.get(), results);
    return;
  }

  if (left->aabb.GetVolume() < right->aabb.GetVolume()) {
    self_query(left, right->left.get(), results);
    self_query(left, right->right.get(), results);
  } else {
    self_query(left->left.get(), right, results);
    self_query(left->right.get(), right, results);
  }
}

auto DynamicAabbTree::Node::self_query(QueryResults& results) const -> void {
  if (is_leaf()) { return; }

  self_query(left.get(), right.get(), results);
  left->self_query(results);
  right->self_query(results);
}

auto DynamicAabbTree::get_node(const SpatialPartitionKey& key) const -> Node* { return nodes.at(key.mVoidKey); }

auto DynamicAabbTree::create_leaf(const SpatialPartitionKey& key, const SpatialPartitionData& data)
  -> std::unique_ptr<Node> {
  // create new node
  auto node{std::make_unique<Node>()};

  node->aabb = data.mAabb;
  node->data = data.mClientData;
  node->grow();

  // add it to the list of all nodes
  nodes.insert_or_assign(key.mVoidKey, node.get());

  return node;
}

auto DynamicAabbTree::rotate(Node& pivot_ref) -> void {
  Node* small_ref = pivot_ref.right.get();
  Node* large_ref = pivot_ref.left.get();

  // identify pivot and its small & large children
  if (small_ref->height > large_ref->height) { std::swap(small_ref, large_ref); }

  // get old parent
  Node& old_parent_ref = *pivot_ref.parent;

  // get grandparent connection
  Node* grandparent = old_parent_ref.parent;
  std::unique_ptr<Node>& grandparent_conn{old_parent_ref.parent ? old_parent_ref.parent->child(old_parent_ref) : root};

  // detach small child from pivot
  std::unique_ptr<Node>& small_loc = pivot_ref.child(*small_ref);
  std::unique_ptr<Node> small_owned = std::move(small_loc);
  small_owned->parent = nullptr;

  // detach pivot from old parent
  std::unique_ptr<Node>& pivot_loc = old_parent_ref.child(pivot_ref);
  std::unique_ptr<Node> pivot_owned = std::move(pivot_loc);
  pivot_owned->parent = nullptr;

  // replace grandparent connection of the old parent with the pivot
  std::unique_ptr<Node> old_parent_owned = std::exchange(grandparent_conn, std::move(pivot_owned));

  // attach the old parent under the pivot where the small child was
  old_parent_owned->parent = &pivot_ref;
  small_loc = std::move(old_parent_owned);

  // attach the small child to the old parent where the pivot node was
  small_owned->parent = &old_parent_ref;
  pivot_loc = std::move(small_owned);

  pivot_ref.parent = grandparent;
  old_parent_ref.parent = &pivot_ref;
  small_ref->parent = &old_parent_ref;
  large_ref->parent = &pivot_ref;

  // pivot_ref.recalculate_aabb();
  // pivot_ref.propagate_depth();
  root->propagate_depth();
  root->recalculate_aabb();
}

auto DynamicAabbTree::rebalance(const Node& start) -> void {
  for (Node* pos = start.parent; pos != nullptr; pos = pos->parent) {
    int balance = pos->balance();
    if (std::abs(balance) <= 1) { continue; }

    if (balance < 0) {
      rotate(*pos->left);
    } else {
      rotate(*pos->right);
    }
  }
}
