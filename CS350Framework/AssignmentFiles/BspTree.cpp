///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include "Precompiled.hpp"

#include <assert.h>
#include <iterator>

#undef near
#undef far

BspTreeQueryData::BspTreeQueryData() { mDepth = 0; }

namespace IntersectionType {
  static constexpr Type Front = Inside;
  static constexpr Type Behind = Outside;
}

namespace {
  auto is_degenerate(const Triangle& triangle, const float epsilon) -> bool {
    const Vector3 p1{triangle.mPoints[0]}, p2{triangle.mPoints[1]}, p3{triangle.mPoints[2]};

    const Vector3 a{p2 - p1};
    const Vector3 b{p3 - p1};

    return a.Cross(b).Length() < epsilon;
  }

  auto tri_normal(const Triangle& triangle) -> Vector3 {
    return (triangle.mPoints[1] - triangle.mPoints[0]).Cross(triangle.mPoints[2] - triangle.mPoints[0]).Normalized();
  }

  auto tri_to_plane(const Triangle& triangle) -> Plane {
    const Vector3 centroid{(triangle.mPoints[0] + triangle.mPoints[1] + triangle.mPoints[2]) / 3.f};
    return {tri_normal(triangle), centroid};
  }
}

void BspTree::SplitTriangle(
  const Plane& plane,
  const Triangle& triangle,
  TriangleList& coplanar_front,
  TriangleList& coplanar_back,
  TriangleList& front,
  TriangleList& back,
  const float epsilon
) {

  const Vector3 p1{triangle.mPoints[0]}, p2{triangle.mPoints[1]}, p3{triangle.mPoints[2]};

  const IntersectionType::Type result{
    PlaneTriangle(plane.mData, p1, p2, p3, epsilon),
  };

  // happy cases yay
  if (result == IntersectionType::Front) {
    front.push_back(triangle);
    return;
  }

  if (result == IntersectionType::Behind) {
    back.push_back(triangle);
    return;
  }

  if (result == IntersectionType::Coplanar) {
    if (tri_normal(triangle).Dot(plane.GetNormal()) > 0) {
      coplanar_front.push_back(triangle);
    } else {
      coplanar_back.push_back(triangle);
    }
    return;
  }

  const auto classify = [&](const Vector3& p) { return PointPlane(p, plane.mData, epsilon); };

  // evil
  std::vector<Vector3> front_output{}, back_output{};

  const auto process = [&](const Vector3& a, const Vector3& b) {
    const IntersectionType::Type a_class = classify(a), b_class = classify(b);

    const auto get_i = [&]() {
      const Ray ray{a, (b - a)};

      float t{0};

      assert(RayPlane(ray.mStart, ray.mDirection, plane.mData, t, epsilon));
      return ray.GetPoint(t);
    };

    if (a_class == IntersectionType::Front && b_class == IntersectionType::Front) {
      front_output.push_back(b);
      return;
    }

    if (a_class == IntersectionType::Coplanar && b_class == IntersectionType::Front) {
      front_output.push_back(b);
      return;
    }

    if (a_class == IntersectionType::Behind && b_class == IntersectionType::Front) {
      const Vector3 i{get_i()};
      front_output.push_back(i);
      front_output.push_back(b);

      back_output.push_back(i);
      return;
    }

    if (a_class == IntersectionType::Front && b_class == IntersectionType::Coplanar) {
      front_output.push_back(b);
      return;
    }

    if (a_class == IntersectionType::Coplanar && b_class == IntersectionType::Coplanar) {
      front_output.push_back(b);
      return;
    }

    if (a_class == IntersectionType::Behind && b_class == IntersectionType::Coplanar) {
      front_output.push_back(b);
      back_output.push_back(b);
      return;
    }

    if (a_class == IntersectionType::Front && b_class == IntersectionType::Behind) {
      const Vector3 i = get_i();
      front_output.push_back(i);

      back_output.push_back(i);
      back_output.push_back(b);
      return;
    }

    if (a_class == IntersectionType::Coplanar && b_class == IntersectionType::Behind) {
      back_output.push_back(a);
      back_output.push_back(b);
      return;
    }

    if (a_class == IntersectionType::Behind && b_class == IntersectionType::Behind) {
      back_output.push_back(b);
      return;
    }

    assert(false && "Unreachable");
  };

  process(p1, p2);
  process(p2, p3);
  process(p3, p1);

  const auto make_triangle = [&](TriangleList& triangles, const std::vector<Vector3>& points) {
    if (points.empty()) { return; }

    assert(points.size() == 3 || points.size() == 4);

    if (points.size() == 3) {
      triangles.emplace_back(points[0], points[1], points[2]);
      return;
    }

    triangles.emplace_back(points[0], points[1], points[2]);
    triangles.emplace_back(points[0], points[2], points[3]);
  };

  make_triangle(front, front_output);
  make_triangle(back, back_output);
}

float BspTree::CalculateScore(
  const TriangleList& triangles,
  const size_t test_index,
  const float k,
  const float epsilon
) {
  /******Student:Assignment4******/

  const Triangle& tri{triangles.at(test_index)};

  if (is_degenerate(tri, Math::DebugEpsilon())) { return Math::PositiveMax(); }

  const Plane plane{tri_to_plane(tri)};

  size_t num_front{0}, numoverlaps{0}, num_behind{0};

  for (const Triangle& other: triangles) {

    const IntersectionType::Type result{
      PlaneTriangle(plane.mData, other.mPoints[0], other.mPoints[1], other.mPoints[2], epsilon)
    };

    if (result == IntersectionType::Front) {
      num_front++;
      continue;
    }

    if (result == IntersectionType::Behind) {
      num_behind++;
      continue;
    }

    if (result == IntersectionType::Overlaps) {
      numoverlaps++;
      continue;
    }
  }

  return k * static_cast<float>(numoverlaps)
       + (1 - k) * Math::Abs(static_cast<float>(num_front) - static_cast<float>(num_behind));
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, const float k, const float epsilon) {
  /******Student:Assignment4******/
  return SPickSplitPlane(triangles, k, epsilon);
}

size_t BspTree::SPickSplitPlane(const TriangleList& triangles, const float k, const float epsilon) {

  size_t lowest_index{0};
  float lowest_score = Math::PositiveMax();

  for (size_t i = 0; i < triangles.size(); i++) {
    const float score = CalculateScore(triangles, i, k, epsilon);

    if (score < lowest_score) {
      lowest_score = score;
      lowest_index = i;
    }
  }

  return lowest_index;
}

void BspTree::Construct(const TriangleList& triangles, const float k, const float epsilon) {
  /******Student:Assignment4******/
  all_nodes.clear();
  root = construct_node(nullptr, triangles, k, epsilon);
}

bool BspTree::RayCast(
  const Ray& ray,
  float& t,
  const float plane_epsilon,
  const float tri_expansion_epsilon,
  const int debugging_index
) const {
  /******Student:Assignment4******/

  t = Math::PositiveMax();
  if (root == nullptr) { return false; }

  (void)debugging_index;

  // return root->raycast_dumb(ray, t, plane_epsilon, tri_expansion_epsilon);
  return root
    ->raycast(ray, t, -Math::PositiveMax(), Math::PositiveMax(), plane_epsilon, tri_expansion_epsilon, debugging_index);
}

void BspTree::AllTriangles(TriangleList& triangles) const {
  /******Student:Assignment4******/
  for_all_nodes(root, [&](Node* node) {
    std::copy(node->coplanar_back.begin(), node->coplanar_back.end(), std::back_inserter(triangles));
    std::copy(node->coplanar_front.begin(), node->coplanar_front.end(), std::back_inserter(triangles));
  });
}

void BspTree::Invert() {
  /******Student:Assignment4******/
  if (root) { root->invert(); }
}

void BspTree::ClipTo(const BspTree* tree, const float epsilon) {
  /******Student:Assignment4******/
  if (!root || !tree || !tree->root) { return; }

  root->clip_to(tree->root, epsilon);
}

void BspTree::Union(const BspTree* tree, const float k, const float epsilon) {
  /******Student:Assignment4******/
  if (!tree || !root) { return; }
  BspTree b{tree->clone()};

  ClipTo(&b, epsilon);
  b.ClipTo(this, epsilon);

  b.Invert();
  b.ClipTo(this, epsilon);
  b.Invert();

  TriangleList result;
  AllTriangles(result);
  b.AllTriangles(result);

  Construct(result, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, const float k, const float epsilon) {
  /******Student:Assignment4******/
  if (!tree || !root) { return; }

  BspTree b{tree->clone()};

  // ~(~A U ~B)
  Invert();

  b.Invert();

  Union(&b, k, epsilon);

  Invert();
}

void BspTree::Subtract(const BspTree* tree, const float k, const float epsilon) {
  if (!tree || !root) { return; }

  BspTree b{tree->clone()};

  b.Invert();

  Intersection(&b, k, epsilon);
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const {
  /******Student:Assignment4******/
  if (root) { root->fillout_data(results); }
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask) {
  /******Student:Assignment4******/
  if (root) { root->debug_draw(level, color, bitMask); }
}

auto BspTree::clone() const -> BspTree {
  BspTree out;

  if (!root) { return out; }

  out.root = root->clone(out, nullptr);
  return out;
}

auto BspTree::Node::is_root() const -> bool { return parent == nullptr; }

auto BspTree::Node::is_leaf() const -> bool { return front == nullptr && back == nullptr; }

auto BspTree::Node::fillout_data(std::vector<BspTreeQueryData>& results) const -> void {
  BspTreeQueryData data;
  data.mDepth = static_cast<int>(depth());

  std::copy(coplanar_front.begin(), coplanar_front.end(), std::back_inserter(data.mTriangles));
  std::copy(coplanar_back.begin(), coplanar_back.end(), std::back_inserter(data.mTriangles));

  results.push_back(data);

  if (front) { front->fillout_data(results); }
  if (back) { back->fillout_data(results); }
}

auto BspTree::Node::raycast(
  const Ray& ray,
  float& t,
  float t_min,
  float t_max,
  const float plane_epsilon,
  const float triangle_expansion_epsilon,
  const int debugging_index
) const -> bool {
  const auto visit_geometry{
    [&](const TriangleList& list, const float t_min, const float t_max) -> bool {
      bool wrote = false;

      for (const Triangle& tri: list) {
        float t_hit{0.f};

        const bool hit{RayTriangle(
          ray.mStart,
          ray.mDirection.Normalized(),
          tri.mPoints[0],
          tri.mPoints[1],
          tri.mPoints[2],
          t_hit,
          triangle_expansion_epsilon
        )};

        if (hit && t_hit >= t_min && t_hit < t_max && t < t_hit) {
          t = Math::Min(t, t_hit);
          wrote = true;
        }
      }

      return wrote;
    },
  };

  const auto visit_side{
    [&ray,
     &t,
     plane_epsilon,
     triangle_expansion_epsilon,
     debugging_index](const Node* side, const float min, const float max) -> bool {
      if (!side) { return false; }
      return side->raycast(ray, t, min, max, plane_epsilon, triangle_expansion_epsilon, debugging_index);
    }
  };

  const IntersectionType::Type ray_start_type = PointPlane(ray.mStart, split_plane.mData, plane_epsilon);

  // edge case 1
  if (ray_start_type == IntersectionType::Coplanar) {
    // visit both sides & geometry in plane
    bool r = false;
    r = r || visit_geometry(coplanar_back, t_min, t_max);
    r = r || visit_side(back, t_min, t_max);
    r = r || visit_side(front, t_min, t_max);
    r = r || visit_geometry(coplanar_front, t_min, t_max);
    return r;
  }

  Node* near_side;
  Node* far_side;
  const TriangleList *coplanar_near, *coplanar_far;

  if (ray_start_type == IntersectionType::Behind) {
    near_side = front;
    far_side = back;
    coplanar_near = &coplanar_front;
    coplanar_far = &coplanar_back;
  } else {
    near_side = back;
    far_side = front;
    coplanar_near = &coplanar_back;
    coplanar_far = &coplanar_front;
  }

  float t_plane = 0.f;
  const bool did_hit = RayPlane(ray.mStart, ray.mDirection, split_plane.mData, t_plane, plane_epsilon);

  // edge case 2
  if (!did_hit) {
    // recurse down the near side
    bool r = visit_side(near_side, t_min, t_max);
    // r = r || visit_geometry(*coplanar_near, t_min, t_max);
    return r;
  }

  // case2
  if (t_plane < 0.f) {
    bool r = visit_side(near_side, t_min, t_max);
    // r = r || visit_geometry(*coplanar_near, t_min, t_max);
    return r;
  }

  // case3
  if (t_max < t_plane) {
    bool r = visit_side(near_side, t_min, t_max);
    // r = r || visit_geometry(*coplanar_near, t_min, t_max);
    return r;
  }

  // case4
  if (t_plane < t_min && 0 < t_plane) {
    bool r = visit_side(far_side, t_min, t_max);
    // r = r || visit_geometry(*coplanar_far, t_min, t_max);
    return r;
  }

  // case1

  assert(t_min <= t_plane && t_plane <= t_max);

  float t_epsilon = Math::Abs(plane_epsilon / split_plane.GetNormal().Normalized().Dot(ray.mDirection.Normalized()));

  bool r = false;
  r = r || visit_side(near_side, t_min, t_plane + t_epsilon); //
  r = r || visit_side(far_side, t_plane - t_epsilon, t_max);
  r = r || visit_geometry(*coplanar_near, t_min, t_max);      //
  r = r || visit_geometry(*coplanar_far, t_min, t_max);       //
  return r;
}

auto BspTree::Node::raycast_dumb(const Ray& ray, float& t, float plane_epsilon, float triangle_expansion_epsilon) const
  -> bool {

  TriangleList triangles = get_triangles();

  bool returns = false;

  for (const Triangle& tri: triangles) {
    float t_tmp = 0;
    returns |= RayTriangle(
      ray.mStart,
      ray.mDirection,
      tri.mPoints[0],
      tri.mPoints[1],
      tri.mPoints[2],
      t_tmp,
      triangle_expansion_epsilon
    );
    if (t_tmp > plane_epsilon) { t = Math::Min(t_tmp, t); }
  }

  if (front) { returns |= front->raycast_dumb(ray, t, plane_epsilon, triangle_expansion_epsilon); }
  if (back) { returns |= back->raycast_dumb(ray, t, plane_epsilon, triangle_expansion_epsilon); }

  return returns;
}

auto BspTree::Node::get_triangles() const -> TriangleList {
  TriangleList triangles{};
  triangles.reserve(coplanar_front.size() + coplanar_back.size());

  triangles.insert(triangles.end(), coplanar_front.begin(), coplanar_front.end());

  triangles.insert(triangles.end(), coplanar_back.begin(), coplanar_back.end());

  return triangles;
}

auto BspTree::Node::clip_triangles(const TriangleList& triangles, const float epsilon) const -> TriangleList {
  if (triangles.empty()) { return {}; }

  TriangleList front_tris, back_tris;

  for (const Triangle& tri: triangles) {
    SplitTriangle(split_plane, tri, front_tris, back_tris, front_tris, back_tris, epsilon);
  }

  if (front) { front_tris = front->clip_triangles(front_tris, epsilon); }

  if (back) {
    back_tris = back->clip_triangles(back_tris, epsilon);
  } else {
    back_tris.clear();
  }

  // Combine results
  front_tris.insert(front_tris.end(), back_tris.begin(), back_tris.end());
  return front_tris;
}

auto BspTree::Node::clip_to(const Node* node, const float epsilon) -> void {
  if (!node) { return; }

  coplanar_front = node->clip_triangles(coplanar_front, epsilon);
  coplanar_back = node->clip_triangles(coplanar_back, epsilon);

  if (front) { front->clip_to(node, epsilon); }

  if (back) { back->clip_to(node, epsilon); }
}

auto BspTree::Node::depth() const -> size_t {
  if (is_root()) { return 0; }

  size_t depth{0};

  for (const Node* node = parent; node; node = node->parent) { depth++; }

  return depth;
}

auto BspTree::Node::invert() -> void {
  std::swap(front, back);

  // flip plane
  split_plane.mData *= -1.f;

  for (Triangle& tri: coplanar_back) { std::swap(tri.mPoints[0], tri.mPoints[1]); }
  for (Triangle& tri: coplanar_front) { std::swap(tri.mPoints[0], tri.mPoints[1]); }

  if (front) { front->invert(); }
  if (back) { back->invert(); }
}

auto BspTree::Node::debug_draw(const int level, const Vector4& color, const int bit_mask) const -> void {
  if (level == -1 || level == static_cast<int>(depth())) {
    for (const Triangle& triangle: coplanar_front) { triangle.DebugDraw().Color(color).SetMaskBit(bit_mask); }
    for (const Triangle& triangle: coplanar_back) { triangle.DebugDraw().Color(color).SetMaskBit(bit_mask); }
  }

  if (front) { front->debug_draw(level, color, bit_mask); }
  if (back) { back->debug_draw(level, color, bit_mask); }
}

auto BspTree::Node::clone(BspTree& tree, Node* out_parent) const -> Node* {
  Node& copy = tree.alloc_node();

  copy.parent = out_parent;
  copy.split_plane = split_plane;

  copy.coplanar_front = coplanar_front;
  copy.coplanar_back = coplanar_back;

  if (front) { copy.front = front->clone(tree, &copy); }
  if (back) { copy.back = back->clone(tree, &copy); }

  return &copy;
}

auto BspTree::construct_node(Node* parent, const TriangleList& triangles, const float k, const float epsilon) -> Node* {
  if (triangles.empty()) { return nullptr; }

  // womp womp
  if (triangles.size() == 1) {
    Node& node = alloc_node();
    node.parent = parent;
    node.split_plane = tri_to_plane(triangles[0]);
    node.coplanar_front.push_back(triangles[0]);

    return &node;
  }

  size_t split_plane_index{PickSplitPlane(triangles, k, epsilon)};

  Node& node = alloc_node();
  node.parent = parent;
  node.split_plane = tri_to_plane(triangles.at(split_plane_index));

  TriangleList front{}, back{};

  for (const Triangle& tri: triangles) {
    SplitTriangle(node.split_plane, tri, node.coplanar_front, node.coplanar_back, front, back, epsilon);
  }

  node.front = construct_node(&node, front, k, epsilon);
  node.back = construct_node(&node, back, k, epsilon);

  return &node;
}

auto BspTree::alloc_node() -> Node& {
  auto node = std::make_unique<Node>();

  Node* ptr = node.get();

  all_nodes.insert_or_assign(ptr, std::move(node));

  return *ptr;
}

auto BspTree::free_node(Node* node) -> void { all_nodes[node].reset(); }
