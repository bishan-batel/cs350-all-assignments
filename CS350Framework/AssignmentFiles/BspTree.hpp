///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Shapes.hpp"

#include <mutex>
#include <unordered_map>

typedef std::vector<Triangle> TriangleList;

// Result data for the bsp tree. Should be filled out with all of the triangles at the given tree depth.
class BspTreeQueryData {
public:

  BspTreeQueryData();

  std::vector<Triangle> mTriangles;
  int mDepth;
};

/******Student:Assignment4******/
// Implement a bsp-tree and the CSG operation of subtraction
class BspTree {
public:

  // Splits the triangle by the given plane into 4 different lists. The front and back side list should be self
  // explanatory (using the proper clipping table described in class). The coplanar front list should be added to when a
  // triangle is coplanar and the normal points the same direction as the plane normal. When the normal points in the
  // opposite direction then the coplanar back list should be added to. Note: When more than 3 points need to be added
  // to a side you should split your polygon into a triangle using a triangle fan. That is (i0, i1, i2), (i0, i2, i3) in
  // order to match my output.
  static void SplitTriangle(
    const Plane& plane,
    const Triangle& tri,
    TriangleList& coplanar_front,
    TriangleList& coplanar_back,
    TriangleList& front,
    TriangleList& back,
    float epsilon
  );

  // Calculate the score of a triangle (using the heuristic described in class) defined by the given test index.
  // Note: Do not count coplanar triangles in the score!
  static float CalculateScore(const TriangleList& triangles, size_t test_index, float k, float epsilon);

  // Return the index of the triangle to use as the split plane. You should use the passed in
  // k-value in your heuristic to balance the cost of balancing the tree with splitting triangles.
  size_t PickSplitPlane(const TriangleList& triangles, float k, float epsilon);

  static size_t SPickSplitPlane(const TriangleList& triangles, float k, float epsilon);

  // Given a list of triangles, construct a bsp-tree. You should use the heuristic shown in class
  // with the given weight (k) to choose between less splits and a more balanced tree.
  void Construct(const TriangleList& triangles, float k, float epsilon);

  // Given a ray determine if and and what time the ray hits this tree. This should return the t-first value.
  // This method should be an optimized ray-cast that computes a tMin and tMax as discussed in class in order to not
  // check every triangle!
  bool RayCast(const Ray& ray, float& t, float plane_epsilon, float tri_expansion_epsilon, int debugging_index) const;

  // Just a simple helper to return all triangles in this tree.
  void AllTriangles(TriangleList& triangles) const;

  // Invert the tree. This requires swapping all positive and negative sides of the tree
  // (split planes, inside/outside pointers, triangle winding order).
  void Invert();

  // Clips all triangles in this tree against all the split planes of the passed in tree.
  // Use the provided epsilon for triangle splitting.
  void ClipTo(const BspTree* tree, float epsilon);

  // Update this tree so that it is a union of the current tree with the passed in tree.
  void Union(const BspTree* tree, float k, float epsilon);

  // Update this tree to be the intersection of the current tree with the passed in tree.
  void Intersection(BspTree* tree, float k, float epsilon);

  // Update this tree to be the subtraction of the other tree from this tree.
  // That is, when A = this and B = other compute A - B.
  void Subtract(const BspTree* tree, float k, float epsilon);

  // Fillout the array of triangles with the contents of the tree. The tree should do a pre-order depth-first insertion
  // into the array (where pre-order means triangle in the node, then recurse down the front of the plane, then the
  // back). Note: When building the tree make sure to add the split triangle as the first on inside the node and then
  // all other coplanar triangles in order afterwards.
  void FilloutData(std::vector<BspTreeQueryData>& results) const;

  // Draw the tree at the given level. Level of -1 means draw the entire tree. You should set the color and
  // bit-mask on the debug shape returned from a debug drawing operation using ".Color(float4)" and ".MaskBit(size_t)".
  // The mask bit is just a helper to allow run-time toggling of debug shapes.
  void DebugDraw(int level, const Vector4& color, int bitMask = 0);

  auto clone() const -> BspTree;

  // Add your implementation here
private:

  struct Node final {
    Node* parent;
    Node* front;
    Node* back;

    Plane split_plane{};

    TriangleList coplanar_front{}, coplanar_back{};

    auto is_root() const -> bool;

    auto is_leaf() const -> bool;

    auto fillout_data(std::vector<BspTreeQueryData>& results) const -> void;

    auto raycast(
      const Ray& ray,
      float& t,
      float t_min,
      float t_max,
      float plane_epsilon,
      float triangle_expansion_epsilon,
      int debugging_index
    ) const -> bool;
    
    auto raycast_dumb(const Ray&, float& t, float plane_epsilon, float triangle_expansion_epsilon) const -> bool;

    auto get_triangles() const -> TriangleList;

    auto clip_triangles(const TriangleList& triangles, float epsilon) const -> TriangleList;

    auto clip_to(const Node* node, float epsilon) -> void;

    auto depth() const -> size_t;

    auto invert() -> void;

    auto debug_draw(int level, const Vector4& color, int bit_mask = 0) const->void;

    auto clone(BspTree& tree, Node* out_parent) const -> Node*;
  };

  std::unordered_map<Node*, std::unique_ptr<Node>> all_nodes{};

  Node* root{nullptr};

  auto construct_node(Node* parent, const TriangleList& triangles, float k, float epsilon) -> Node*;

  auto alloc_node() -> Node&;

  auto free_node(Node* node) -> void;

  template<typename F>
  static auto for_all_nodes(Node* node, F func) -> void {
    if (node == nullptr) { return; }
    func(node);
    if (node->front) { for_all_nodes(node->front, func); }
    if (node->back) { for_all_nodes(node->back, func); }
  }
};
