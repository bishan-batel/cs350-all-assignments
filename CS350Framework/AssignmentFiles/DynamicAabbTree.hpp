/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: DynamicAabbTree.hpp
Purpose: Dynamic AABB tree impl1
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_3
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#pragma once

#include <memory>

#include "SpatialPartition.hpp"
#include "Shapes.hpp"

#include <unordered_map>

/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition {
public:

  DynamicAabbTree();

  ~DynamicAabbTree() override;

  // Spatial Partition Interface
  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;

  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;

  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;

  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  void GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const override;

  static constexpr float mFatteningFactor = 1.1;

private:

  struct Node final {
    Aabb aabb;
    void* data{};
    std::unique_ptr<Node> left{}, right{};
    Node* parent{};
    size_t depth{0};
    size_t height{0};
    mutable size_t last_axis{6};

    auto propagate_depth() -> void;

    auto recalculate_aabb() -> void;

    auto is_leaf() const -> bool;

    auto select_node(Node& choice1, Node& choice2) const -> Node&;

    auto insert(std::unique_ptr<Node>& node) -> void;

    auto debug_draw(int level, const Matrix4& transform, const Vector4& color, int bitMask) const -> void;

    auto cast_frustum(const Frustum& ray, CastResults& results) const -> void;

    auto cast_frustum_fully_contained(CastResults& results) const -> void;

    auto cast_ray(const Ray& ray, CastResults& results) const -> bool;

    auto ray_aabb(const Ray& ray, float& t) const -> bool;

    auto overlaps(const Node& other) const -> bool;

    auto sibling() const -> std::unique_ptr<Node>&;

    auto grow() -> void;

    void fillout_data(std::vector<SpatialPartitionQueryData>& results) const;

    auto subtree_size() const -> size_t;

    auto child(const Node& node) -> std::unique_ptr<Node>&;

    auto balance() const -> int;

    static auto self_query(const Node* left, const Node* right, QueryResults& results) -> void;

    static void split_nodes(const Node* left, const Node* right, QueryResults& results);

    auto self_query(QueryResults& results) const -> void;
  };

  auto get_node(const SpatialPartitionKey& key) const -> Node*;

  auto create_leaf(const SpatialPartitionKey& key, const SpatialPartitionData& data) -> std::unique_ptr<Node>;

  auto rotate(Node& pivot_ref) -> void;
  auto rebalance(const Node& start) -> void;

  std::unique_ptr<Node> root{};

  std::unordered_map<void*, Node*> nodes;

  // Add your implementation here
};
