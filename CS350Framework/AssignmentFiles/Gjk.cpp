/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: Gjk.cpp
Purpose: GJK Implementation
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_5
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"

#include <numeric>

//-----------------------------------------------------------------------------SupportShape
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints, const Matrix4& transform) const {
  Vector3 centroid{};

  for (const Vector3& point: localPoints) { centroid += point; }

  return Math::TransformPoint(transform, centroid / static_cast<float>(localPoints.size()));
}

Vector3 SupportShape::Support(
  const Vector3& world_direction,
  const std::vector<Vector3>& local_points,
  const Matrix4& local_to_world_transform
) const {

  const Vector3 dir = Math::TransformNormal(local_to_world_transform.Inverted(), world_direction);

  Vector3 support_point{};

  float max_dot{std::numeric_limits<float>::lowest()};

  // get max support piotn
  for (const Vector3& point: local_points) {
    const float dot{point.Dot(dir)};

    if (dot >= max_dot) {
      max_dot = dot;
      support_point = point;
    }
  }

  return Math::TransformPoint(local_to_world_transform, support_point);
}

void SupportShape::DebugDraw(
  const std::vector<Vector3>& localPoints,
  const Matrix4& localToWorldTransform,
  const Vector4& color
) const {
  for (const Vector3& p: localPoints) {
    gDebugDrawer->DrawPoint(Math::TransformPoint(localToWorldTransform, p)).Color(color);
  }
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const {
  return SupportShape::GetCenter(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const {
  return SupportShape::Support(
    worldDirection,
    mModel->mMesh->mVertices,
    mModel->mOwner->has(Transform)->GetTransform()
  );
}

void ModelSupportShape::DebugDraw(const Vector4& color) const {
  SupportShape::DebugDraw(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape() {
  mScale = Vector3(1);
  mRotation = Matrix3::cIdentity;
  mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const {
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const {
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const {
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const { return mSphere.mCenter; }

Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const {
  return GetCenter() + (worldDirection.Normalized() * mSphere.GetRadius());
}

void SphereSupportShape::DebugDraw(const Vector4& color) const {
  DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
  shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const { return mTranslation; }

Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const {
  Vector3 dir{Math::Transform(mRotation.Transposed(), worldDirection)};

  Vector3 obb_local{Vector3(Math::GetSign(dir.x), Math::GetSign(dir.y), Math::GetSign(dir.z)) * mScale * 0.5f};

  return Math::Transform(mRotation, obb_local) + mTranslation;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const {
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
  shape.Color(color);
  shape.SetTransform(transform);
}

//------------------------------------------------------------ Voronoi Region Tests
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
  const Vector3& q,
  const Vector3& p0,
  size_t& newSize,
  int newIndices[4],
  Vector3& closestPoint,
  Vector3& searchDirection
) {
  newSize = 1;
  newIndices[0] = 0;
  closestPoint = p0;
  searchDirection = q - p0;

  return VoronoiRegion::Point0;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
  const Vector3& q,
  const Vector3& p0,
  const Vector3& p1,
  size_t& newSize,
  int newIndices[4],
  Vector3& closestPoint,
  Vector3& searchDirection
) {
  Vector2 uv{};

  BarycentricCoordinates(q, p0, p1, uv.x, uv.y);

  VoronoiRegion::Type output;

  if (uv.x <= 0.f) {
    closestPoint = p1;
    newIndices[0] = 1;
    newSize = 1;
    output = VoronoiRegion::Point1;
  } else if (uv.y <= 0.f) {
    closestPoint = p0;
    newIndices[0] = 0;
    newSize = 1;
    output = VoronoiRegion::Point0;
  } else {
    closestPoint = p0 * uv.x + p1 * uv.y;
    newIndices[0] = 0;
    newIndices[1] = 1;
    newSize = 2;
    output = VoronoiRegion::Edge01;
  }

  searchDirection = q - closestPoint;
  return output;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
  const Vector3& q,
  const Vector3& p0,
  const Vector3& p1,
  const Vector3& p2,
  size_t& new_size,
  int new_indices[4],
  Vector3& closest_point,
  Vector3& search_direction
) {

  float u12 = 0, v12 = 0;
  float u20 = 0, v20 = 0;
  float u01 = 0, v01 = 0;

  BarycentricCoordinates(q, p1, p2, u12, v12);
  BarycentricCoordinates(q, p2, p0, u20, v20);
  BarycentricCoordinates(q, p0, p1, u01, v01);

  // edge case

  const Vector3 ps[3]{p0, p1, p2};

  auto simple_case = [&](const int i) -> VoronoiRegion::Type {
    new_indices[0] = i;
    new_size = 1;
    closest_point = ps[i];
    search_direction = q - closest_point;
    return static_cast<VoronoiRegion::Type>(VoronoiRegion::Point0 + i);
  };

  if (u20 <= 0.f && v01 <= 0.f) { return simple_case(0); }
  if (u01 <= 0.f && v12 <= 0.f) { return simple_case(1); }
  if (u12 <= 0.f && v20 <= 0.f) { return simple_case(2); }

  ///

  float u, v, w;

  BarycentricCoordinates(q, p0, p1, p2, u, v, w);

  if (u01 > 0.f && v01 > 0.f && w <= 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 1;
    new_size = 2;

    closest_point = u01 * p0 + v01 * p1;
    search_direction = q - closest_point;

    return VoronoiRegion::Edge01;
  }

  if (u12 > 0.f && v12 > 0.f && u <= 0.f) {
    new_indices[0] = 1;
    new_indices[1] = 2;
    new_size = 2;

    closest_point = v12 * p2 + p1 * u12;
    search_direction = q - closest_point;

    return VoronoiRegion::Edge12;
  }

  if (u20 > 0.f && v20 > 0.f && v <= 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 2;
    new_size = 2;

    closest_point = (u20 * p2) + (v20 * p0);
    search_direction = q - closest_point;

    //
    return VoronoiRegion::Edge02;
  }

  closest_point = (u * p0) + (v * p1) + (w * p2);
  search_direction = q - closest_point;
  new_indices[0] = 0;
  new_indices[1] = 1;
  new_indices[2] = 2;
  new_size = 3;

  return VoronoiRegion::Triangle012;
}

namespace {
  // helper for the next function1
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
  const Vector3& q,
  const Vector3& p0,
  const Vector3& p1,
  const Vector3& p2,
  const Vector3& p3,
  size_t& new_size,
  int new_indices[4],
  Vector3& closest_point,
  Vector3& search_direction
) {
  const auto get_side{
    [](const Vector3& q, const Vector3& point0, const Vector3& point1, const Vector3& point2, const Vector3& back)
      -> float {
      Vector3 normal{(point1 - point0).Cross(point2 - point0)};

      if (normal.Dot(back - point0) < 0.f) { normal *= -1.f; }

      return normal.Dot(q - point0);
    }
  };

  float u01, v01, u20, v20, u03, v03, u12, v12, u13, v13, u32, v32;

  BarycentricCoordinates(q, p0, p1, u01, v01);
  BarycentricCoordinates(q, p2, p0, u20, v20);
  BarycentricCoordinates(q, p0, p3, u03, v03);
  BarycentricCoordinates(q, p1, p2, u12, v12);
  BarycentricCoordinates(q, p1, p3, u13, v13);
  BarycentricCoordinates(q, p3, p2, u32, v32);

  // degenerate case
  if (v01 <= 0.f && u20 <= 0.f && v03 <= 0.f) {
    new_indices[0] = 0;
    new_size = 1;
    closest_point = p0;
    search_direction = q - closest_point;
    return VoronoiRegion::Point0;
  }

  // same as the prev function but just more of an nightamre
  if (u01 <= 0.f && v12 <= 0.f && v13 <= 0.f) {
    new_indices[0] = 1;
    new_size = 1;
    closest_point = p1;
    search_direction = q - closest_point;
    return VoronoiRegion::Point1;
  }

  if (u12 <= 0.f && v20 <= 0.f && u32 <= 0.f) {
    new_indices[0] = 2;
    new_size = 1;
    closest_point = p2;
    search_direction = q - closest_point;
    return VoronoiRegion::Point2;
  }

  if (u13 <= 0.f && v32 <= 0.f && u03 <= 0.f) {
    new_indices[0] = 3;
    new_size = 1;
    closest_point = p3;
    search_direction = q - closest_point;
    return VoronoiRegion::Point3;
  }

  // float u031, v031, w031;
  // float u023, v023, w023;
  // float u213, v213, w213;
  // float u012, v012, w012;

  float u031, v031, w031, u023, v023, w023, u213, v213, w213, u012, v012, w012;

  BarycentricCoordinates(q, p0, p1, p2, u012, v012, w012);
  BarycentricCoordinates(q, p2, p1, p3, u213, v213, w213);
  BarycentricCoordinates(q, p0, p3, p1, u031, v031, w031);
  BarycentricCoordinates(q, p0, p2, p3, u023, v023, w023);

  if (w012 <= 0.f && v031 <= 0.f && u01 > 0.f && v01 > 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 1;
    new_size = 2;
    closest_point = u01 * p0 + v01 * p1;
    search_direction = q - closest_point;
    return VoronoiRegion::Edge01;
  }

  if (u012 <= 0.f && w213 <= 0.f && u12 > 0.f && v12 > 0.f) {
    new_indices[0] = 1;
    new_indices[1] = 2;
    new_size = 2;
    closest_point = u12 * p1 + v12 * p2;
    search_direction = q - closest_point;
    return VoronoiRegion::Edge12;
  }

  if (v012 <= 0.f && w023 <= 0.f && u20 > 0.f && v20 > 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 2;
    new_size = 2;
    closest_point = u20 * p2 + p0 * v20;
    search_direction = q - closest_point;
    return VoronoiRegion::Edge02;
  }

  if (v213 <= 0.f && u023 <= 0.f && u32 > 0.f && v32 > 0.f) {
    new_indices[0] = 2;
    new_indices[1] = 3;
    new_size = 2;
    closest_point = p3 * u32 + p2 * v32;
    search_direction = q - closest_point;
    return VoronoiRegion::Edge23;
  }

  if (v023 <= 0.f && w031 <= 0.f && u03 > 0.f && v03 > 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 3;
    new_size = 2;
    closest_point = u03 * p0 + v03 * p3;
    search_direction = q - closest_point;
    // return VoronoiRegion::Edge02;
    return VoronoiRegion::Edge03;
  }

  if (u213 <= 0.f && u031 <= 0.f && u13 > 0.f && v13 > 0.f) {
    new_indices[0] = 1;
    new_indices[1] = 3;
    new_size = 2;

    closest_point = p1 * u13 + p3 * v13;
    search_direction = q - closest_point;

    return VoronoiRegion::Edge13;
  }

  float t{get_side(q, p0, p1, p2, p3)};
  if (t < 0.f && u012 > 0.f && v012 > 0.f && w012 > 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 1;
    new_indices[2] = 2;
    new_size = 3;

    closest_point = p0 * u012 + p1 * v012 + p2 * w012;
    search_direction = q - closest_point;

    return VoronoiRegion::Triangle012;
  }

  float u = get_side(q, p2, p1, p3, p0);

  if (u < 0.f && u213 > 0.f && v213 > 0.f && w213 > 0.f) {
    new_indices[0] = 1;
    new_indices[1] = 2;
    new_indices[2] = 3;
    new_size = 3;
    closest_point = u213 * p2 + v213 * p1 + w213 * p3;
    search_direction = q - closest_point;
    return VoronoiRegion::Triangle123;
  }

  const float v{
    get_side(q, p0, p2, p3, p1),
  };

  if (v < 0.f && u023 > 0.f && v023 > 0.f && w023 > 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 2;
    new_indices[2] = 3;
    new_size = 3;
    closest_point = p0 * u023 + p2 * v023 + p3 * w023;
    search_direction = q - closest_point;
    return VoronoiRegion::Triangle023;
  }

  const float w{
    get_side(q, p0, p3, p1, p2),
  };

  if (w < 0.f && u031 > 0.f && v031 > 0.f && w031 > 0.f) {
    new_indices[0] = 0;
    new_indices[1] = 1;
    new_indices[2] = 3;
    new_size = 3;
    closest_point = u031 * p0 + v031 * p3 + p1 * w031;
    search_direction = q - closest_point;
    return VoronoiRegion::Triangle013;
  }

  // new_indices[0] = 3;
  // new_indices[1] = 2;
  // new_indices[2] = 1;
  // new_indices[3] = 0;
  new_indices[0u] = 0u;
  new_indices[1u] = 1u;
  new_indices[2u] = 2u;
  new_indices[3u] = 3u;
  new_size = 4;
  closest_point = q;
  search_direction = Vector3::cZero;
  return VoronoiRegion::Tetrahedra0123;
}

Gjk::Gjk() {}

bool Gjk::Intersect(
  const SupportShape* shape_a,
  const SupportShape* shape_b,
  const unsigned int max_iterations,
  CsoPoint& closest_point,
  const float epsilon,
  const int debugging_index,
  const bool debug_draw
) {
  // using '0u' notation in this function cause i had a weird signed integer bug

  (void)debugging_index;
  (void)debug_draw;

  Vector3 search_dir = shape_b->GetCenter() - shape_a->GetCenter();

  // if search direction is basically zero
  if (search_dir.LengthSq() < Math::DebugEpsilon() * Math::DebugEpsilon()) { search_dir = -Vector3::cXAxis; }

  std::vector<CsoPoint> simplex{};

  closest_point = ComputeSupport(shape_a, shape_b, search_dir);
  simplex.push_back(closest_point);

  // const Vector3 q{Vector3::cZero};

  for (size_t i{0u}; i < max_iterations; i++) {
    const size_t curr_size{simplex.size()};

    Vector3 p;
    int indices[4u];
    size_t new_size{0u};

    switch (curr_size) {
      case 1u:
        {
          IdentifyVoronoiRegion(Vector3(), simplex[0].mCsoPoint, new_size, indices, p, search_dir);
          break;
        }
      case 2u:
        {
          IdentifyVoronoiRegion(
            Vector3(),
            simplex[0].mCsoPoint,
            simplex[1].mCsoPoint,
            new_size,
            indices,
            p,
            search_dir
          );
          break;
        }
      case 3u:
        {
          IdentifyVoronoiRegion(
            Vector3(),
            simplex[0].mCsoPoint,
            simplex[1].mCsoPoint,
            simplex[2].mCsoPoint,
            new_size,
            indices,
            p,
            search_dir
          );
          break;
        }
      case 4u:
        {
          IdentifyVoronoiRegion(
            Vector3(),
            simplex[0u].mCsoPoint,
            simplex[1u].mCsoPoint,
            simplex[2u].mCsoPoint,
            simplex[3u].mCsoPoint,
            new_size,
            indices,
            p,
            search_dir
          );
          break;
        }
      default:
        // invalid
        return false;
    }

    if (new_size == 4u) { return true; }
    if (p.LengthSq() < epsilon * epsilon) { return true; }

    for (size_t c = 0u; c < new_size; c++) { simplex.at(c) = simplex.at(indices[c]); }
    simplex.resize(new_size);

    closest_point = ComputeSupport(shape_a, shape_b, search_dir);
    simplex.push_back(closest_point);

    if (search_dir.Dot(closest_point.mCsoPoint - p) <= epsilon) {
      std::vector<float> barycentric;

      switch (new_size) {
        case 1u:
          {
            closest_point = simplex[0u];
            return false;
          }
        case 2u:
          {
            barycentric.resize(2u, 0.f);
            BarycentricCoordinates(p, simplex[0u].mCsoPoint, simplex[1u].mCsoPoint, barycentric[0u], barycentric[1u]);
            break;
          }
        case 3u:
          {
            barycentric.resize(3u, 0.f);
            BarycentricCoordinates(
              p,
              simplex[0u].mCsoPoint,
              simplex[1u].mCsoPoint,
              simplex[2u].mCsoPoint,
              barycentric[0u],
              barycentric[1u],
              barycentric[2u]
            );
            break;
          }
        default: // invalid, should never happen?
          break;
      }

      closest_point.mCsoPoint = p;

      // accumulate simplexes
      closest_point.mPointA = Vector3::cZero;
      closest_point.mPointB = Vector3::cZero;

      for (size_t j = 0u; j < barycentric.size(); j++) {
        closest_point.mPointA += simplex[j].mPointA * barycentric[j];
        closest_point.mPointB += simplex[j].mPointB * barycentric[j];
      }

      return false;
    }
  }

  return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shape_a, const SupportShape* shape_b, const Vector3& direction) {
  Vector3 point_a{shape_a->Support(direction)}, point_b{shape_b->Support(-direction)};

  return CsoPoint{point_a, point_b, point_a - point_b};
}
