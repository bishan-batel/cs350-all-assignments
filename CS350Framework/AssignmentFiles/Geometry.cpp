/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: Geometry.cpp
Purpose: Pair Tests between primitives
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_1
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"

#include <cassert>

Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal, float planeDistance) {
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  const float distance{point.Dot(normal) - planeDistance};

  return point - normal * distance;
}

bool BarycentricCoordinates(
  const Vector3& point,
  const Vector3& a,
  const Vector3& b,
  float& u,
  float& v,
  float epsilon
) {
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  const Vector3 a_to_b{b - a};

  const float a_to_b_len2{a_to_b.LengthSq()};

  // line is degenerate
  if (a_to_b_len2 < epsilon * epsilon) {
    u = 0.f;
    v = 0.f;
    return false;
  }

  // project point onto line
  v = a_to_b.Dot(point - a) / a_to_b_len2;
  u = 1.0f - v;

  return v > -epsilon && v < 1.0f + epsilon;
}

bool BarycentricCoordinates(
  const Vector3& point,
  const Vector3& a,
  const Vector3& b,
  const Vector3& c,
  float& u,
  float& v,
  float& w,
  float epsilon
) {
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  Vector3 abc_normal{(b - a).Cross(c - a)};

  float abc_area{abc_normal.LengthSq()};

  // triangle is degenerate
  if (abc_area < epsilon * epsilon) {
    u = 0.f;
    v = 0.f;
    w = 0.f;
    return false;
  }

  abc_area = Math::Sqrt(abc_area);

  abc_normal /= abc_area;

  const float area_pbc{(b - point).Cross(c - point).Dot(abc_normal)};
  const float area_pca{(c - point).Cross(a - point).Dot(abc_normal)};

  // u = area(PBC) / area(ABC);
  // v = area(PCA) / area(ABC);
  // w = 1.f - u - v

  u = area_pbc / abc_area;
  v = area_pca / abc_area;
  w = 1.f - u - v;

  return u >= -epsilon && v >= -epsilon && w >= -epsilon;
}

IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon) {
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  const auto projected{
    ProjectPointOnPlane(point, Vector3{plane.x, plane.y, plane.z}, plane.w),
  };

  Vector3 plane_to_point = (point - projected);

  // if point is close enough to plane, consider it coplanar
  if (plane_to_point.LengthSq() < epsilon * epsilon) { return IntersectionType::Coplanar; }

  if (plane_to_point.Dot(Vector3{plane.x, plane.y, plane.z}) < -epsilon) { return IntersectionType::Outside; }

  return IntersectionType::Inside;
}

bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius) {
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  return (point - sphereCenter).LengthSq() <= sphereRadius * sphereRadius;
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax) {
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  if (point.x < aabbMin.x || point.x > aabbMax.x) { return false; }

  if (point.y < aabbMin.y || point.y > aabbMax.y) { return false; }

  if (point.z < aabbMin.z || point.z > aabbMax.z) { return false; }

  return true;
}

bool RayPlane(const Vector3& rayStart, const Vector3& rayDir, const Vector4& plane, float& t, float epsilon) {
  ++Application::mStatistics.mRayPlaneTests;
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  const Vector3 normal{plane.x, plane.y, plane.z};

  // normal & plane are perpendicular, therefore ray & plane are parallel
  if (Math::Abs(rayDir.Dot(normal)) < epsilon) { return false; }

  t = (plane.w - rayStart.Dot(normal)) / (rayDir.Dot(normal));

  return t > -epsilon;
}

bool RayTriangle(
  const Vector3& rayStart,
  const Vector3& rayDir,
  const Vector3& triP0,
  const Vector3& triP1,
  const Vector3& triP2,
  float& t,
  float triExpansionEpsilon
) {
  ++Application::mStatistics.mRayTriangleTests;
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  Plane plane{};
  plane.Set(triP0, triP1, triP2);

  if (!RayPlane(rayStart, rayDir, plane.mData, t, triExpansionEpsilon)) { return false; }

  const Vector3 point{rayStart + rayDir * t};

  float u, v, w;
  return BarycentricCoordinates(point, triP0, triP1, triP2, u, v, w, triExpansionEpsilon);
}

bool RaySphere(
  const Vector3& rayStart,
  const Vector3& rayDir,
  const Vector3& sphereCenter,
  float sphereRadius,
  float& t
) {
  ++Application::mStatistics.mRaySphereTests;
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  if (PointSphere(rayStart, sphereCenter, sphereRadius)) {
    t = 0.f;
    return true;
  }

  const Vector3 sphere_to_ray_start = rayStart - sphereCenter;

  const float a = rayDir.Dot(rayDir);
  const float b = 2.f * sphere_to_ray_start.Dot(rayDir);
  const float c = sphere_to_ray_start.Dot(sphere_to_ray_start) - sphereRadius * sphereRadius;

  const float discriminant2{b * b - 4.f * a * c};

  if (discriminant2 < 0.f) { return false; }

  const float discriminant = Math::Sqrt(discriminant2);

  const float t0 = (-b + discriminant) / (2.f * a);

  if (discriminant == 0.f) {
    t = t0;
    return t >= 0.f;
  }

  const float t1 = (-b - discriminant) / (2.f * a);

  if (t0 < 0.f) {
    t = t1;
    return t >= 0.f;
  }

  if (t1 < 0.f) {
    t = t0;
    return t >= 0.f;
  }

  t = Math::Min(t0, t1);
  return t >= 0.f;
}

bool RayAabb(const Vector3& rayStart, const Vector3& rayDir, const Vector3& aabbMin, const Vector3& aabbMax, float& t) {
  ++Application::mStatistics.mRayAabbTests;
  /******Student:Assignment1******/

  // ray starts inside
  if (PointAabb(rayStart, aabbMin, aabbMax)) {
    t = 0.f;
    return true;
  }

  float t_x_min = (aabbMin.x - rayStart.x) / rayDir.x;
  float t_x_max = (aabbMax.x - rayStart.x) / rayDir.x;

  if (rayDir.x < 0.f) { std::swap(t_x_min, t_x_max); }

  float t_y_min = (aabbMin.y - rayStart.y) / rayDir.y;
  float t_y_max = (aabbMax.y - rayStart.y) / rayDir.y;

  if (rayDir.y < 0.f) { std::swap(t_y_min, t_y_max); }

  float t_z_min = (aabbMin.z - rayStart.z) / rayDir.z;
  float t_z_max = (aabbMax.z - rayStart.z) / rayDir.z;

  if (rayDir.z < 0.f) { std::swap(t_z_min, t_z_max); }

  float t_min = -1.f;
  float t_max = FLT_MAX;

  if (!(rayDir.x == 0.f && Math::InRange(rayStart.x, aabbMin.x, aabbMax.x))) {
    t_min = std::max(t_min, t_x_min);
    t_max = std::min(t_max, t_x_max);
  }

  if (!(rayDir.y == 0.f && Math::InRange(rayStart.y, aabbMin.y, aabbMax.y))) {
    t_min = std::max(t_min, t_y_min);
    t_max = std::min(t_max, t_y_max);
  }

  if (!(rayDir.z == 0.f && Math::InRange(rayStart.z, aabbMin.z, aabbMax.z))) {
    t_min = std::max(t_min, t_z_min);
    t_max = std::min(t_max, t_z_max);
  }

  // t_min = std::max(t_x_min, std::max(t_y_min, t_z_min));
  // t_max = std::min(t_x_max, std::min(t_y_max, t_z_max));

  if (t_min > t_max) { return false; }

  t = t_min;

  return t > 0.f;
}

IntersectionType::Type PlaneTriangle(
  const Vector4& plane,
  const Vector3& triP0,
  const Vector3& triP1,
  const Vector3& triP2,
  float epsilon
) {
  ++Application::mStatistics.mPlaneTriangleTests;
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  const IntersectionType::Type p0 = PointPlane(triP0, plane, epsilon);
  const IntersectionType::Type p1 = PointPlane(triP1, plane, epsilon);
  const IntersectionType::Type p2 = PointPlane(triP2, plane, epsilon);

  // if (p0 != p1 || p2 != p0 || p1 != p2)
  // {
  //     return IntersectionType::Overlaps;
  // }

  if (p0 == p1 && p0 == p2 && p0 == IntersectionType::Coplanar) { return IntersectionType::Coplanar; }

  unsigned numFront = 0, numBehind = 0;

  numFront += (p0 == IntersectionType::Inside);
  numFront += (p1 == IntersectionType::Inside);
  numFront += (p2 == IntersectionType::Inside);

  numBehind += (p0 == IntersectionType::Outside);
  numBehind += (p1 == IntersectionType::Outside);
  numBehind += (p2 == IntersectionType::Outside);
  
  if (numFront > 0 && numBehind > 0) { return IntersectionType::Overlaps; }

  if (numFront > numBehind) { return IntersectionType::Inside; }

  if (numFront < numBehind) { return IntersectionType::Outside; }
  
  assert(false);
  return IntersectionType::Overlaps;
}

IntersectionType::Type PlaneSphere(const Vector4& plane, const Vector3& sphereCenter, float sphereRadius) {
  ++Application::mStatistics.mPlaneSphereTests;
  /******Student:Assignment1******/

  // Warn("Assignment1: Required function un-implemented");

  const Vector3 plane_normal{plane.x, plane.y, plane.z};

  const Vector3 projected{
    ProjectPointOnPlane(sphereCenter, plane_normal, plane.w),
  };

  Vector3 plane_to_point = (sphereCenter - projected);

  if (plane_to_point.LengthSq() <= sphereRadius * sphereRadius) { return IntersectionType::Overlaps; }

  if (plane_to_point.Dot(plane_normal) < 0) { return IntersectionType::Outside; }

  return IntersectionType::Inside;
  // return IntersectionType::NotImplemented;
}

IntersectionType::Type PlaneAabb(const Vector4& plane, const Vector3& aabbMin, const Vector3& aabbMax) {
  ++Application::mStatistics.mPlaneAabbTests;
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  const Vector3 center{(aabbMin + aabbMax) / 2.f};

  Vector3 extrema1{
    plane.x < 0.f ? aabbMin.x : aabbMax.x,
    plane.y < 0.f ? aabbMin.y : aabbMax.y,
    plane.z < 0.f ? aabbMin.z : aabbMax.z,
  };

  Vector3 extrema2 = center - (extrema1 - center);

  const IntersectionType::Type min = PointPlane(extrema1, plane, -1.f);
  const IntersectionType::Type max = PointPlane(extrema2, plane, -1.f);

  if (min == max) { return min; }

  return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumTriangle(
  const Vector4 planes[6],
  const Vector3& triP0,
  const Vector3& triP1,
  const Vector3& triP2,
  float epsilon
) {
  ++Application::mStatistics.mFrustumTriangleTests;
  /******Student:Assignment1******/
  size_t inside_count{0};

  for (size_t i = 0; i < 6; i++) {
    const IntersectionType::Type type = PlaneTriangle(planes[i], triP0, triP1, triP2, epsilon);

    if (type == IntersectionType::Outside) { return type; }

    if (type == IntersectionType::Inside) { inside_count++; }
  }

  if (inside_count == 6) { return IntersectionType::Inside; }

  return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumSphere(
  const Vector4 planes[6],
  const Vector3& sphereCenter,
  float sphereRadius,
  size_t& lastAxis
) {
  ++Application::mStatistics.mFrustumSphereTests;

  // /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  // TODO: lastAxis

  size_t inside_count{0};

  for (size_t i = 0; i < 6; i++) {
    const IntersectionType::Type type = PlaneSphere(planes[i], sphereCenter, sphereRadius);

    if (type == IntersectionType::Outside) { return type; }

    if (type == IntersectionType::Inside) { inside_count++; }
  }

  if (inside_count == 6) { return IntersectionType::Inside; }

  return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumAabb(
  const Vector4 planes[6],
  const Vector3& aabbMin,
  const Vector3& aabbMax,
  size_t& lastAxis
) {
  ++Application::mStatistics.mFrustumAabbTests;
  /******Student:Assignment1******/
  // Warn("Assignment1: Required function un-implemented");

  if (lastAxis < 6) {
    const IntersectionType::Type result{PlaneAabb(planes[lastAxis], aabbMin, aabbMax)};

    if (result == IntersectionType::Outside) { return IntersectionType::Outside; }
  }

  size_t inside_count{0};

  for (size_t i = 0; i < 6; i++) {
    if (i == lastAxis) { continue; }
    const IntersectionType::Type type = PlaneAabb(planes[i], aabbMin, aabbMax);

    if (type == IntersectionType::Outside) {
      lastAxis = i;
      return type;
    }

    if (type == IntersectionType::Inside) { inside_count++; }
  }

  if (inside_count == 6) { return IntersectionType::Inside; }

  return IntersectionType::Overlaps;
}

bool SphereSphere(
  const Vector3& sphereCenter0,
  const float sphereRadius0,
  const Vector3& sphereCenter1,
  const float sphereRadius1
) {
  ++Application::mStatistics.mSphereSphereTests;

  /******Student:Assignment1******/

  const float distance2{(sphereCenter0 - sphereCenter1).LengthSq()};

  const float radius = sphereRadius0 + sphereRadius1;
  ;

  return distance2 <= radius * radius;
}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0, const Vector3& aabbMin1, const Vector3& aabbMax1) {
  ++Application::mStatistics.mAabbAabbTests;
  /******Student:Assignment1******/

  const auto range_overlaps{[](const float start1, const float end1, const float start2, const float end2) {
    return !(end1 < start2 || end2 < start1);
  }};

  const bool x = range_overlaps(aabbMin0.x, aabbMax0.x, aabbMin1.x, aabbMax1.x);

  if (!x) { return false; }

  const bool y = range_overlaps(aabbMin0.y, aabbMax0.y, aabbMin1.y, aabbMax1.y);

  if (!y) { return false; }

  const bool z = range_overlaps(aabbMin0.z, aabbMax0.z, aabbMin1.z, aabbMax1.z);

  return z;
}
