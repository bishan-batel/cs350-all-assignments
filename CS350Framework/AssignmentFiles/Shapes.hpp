///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Math/Math.hpp"

class DebugShape;

//-----------------------------------------------------------------------------LineSegment
class LineSegment
{
public:
  LineSegment();
  LineSegment(Math::Vec3Param start, Math::Vec3Param end);

  DebugShape& DebugDraw() const;

  Vector3 mStart;
  Vector3 mEnd;
};

//-----------------------------------------------------------------------------Ray
class Ray
{
public:
  Ray();
  Ray(Math::Vec3Param start, Math::Vec3Param dir);

  // Transforms this ray to another space by the given matrix 4
  Ray Transform(const Matrix4& transform) const;
  // Returns the a point at the given t value.
  Vector3 GetPoint(float t) const;
  // Draws this ray up to a given t value.
  DebugShape& DebugDraw(float t) const;

  Vector3 mStart;
  Vector3 mDirection;
};


//-----------------------------------------------------------------------------PCA Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points);
Matrix3 ComputeJacobiRotation(const Matrix3& matrix);
void ComputeEigenValuesAndVectors(const Matrix3& covariance, Vector3& eigenValues, Matrix3& eigenVectors, int maxIterations);

//-----------------------------------------------------------------------------Sphere
class Sphere
{
public:
  Sphere();
  Sphere(const Vector3& center, float radius);

  void ComputeCentroid(const std::vector<Vector3>& points);
  void ComputeRitter(const std::vector<Vector3>& points);
  void ComputePCA(const std::vector<Vector3>& points);

  // Does the sphere contain the given point?
  bool ContainsPoint(const Vector3& point);

  Vector3 GetCenter() const;
  float GetRadius() const;

  // See if this aabb is equal to another (with epsilon). Used for unit testing.
  bool Compare(const Sphere& rhs, float epsilon) const;

  DebugShape& DebugDraw() const;

  Vector3 mCenter;
  float mRadius;
};

//-----------------------------------------------------------------------------Aabb
class Aabb
{
public:
  Aabb();
  Aabb(const Vector3& min, const Vector3& max);

  static Aabb BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents);

  // Computes the volume of this aabb.
  float GetVolume() const;
  // Computes the surface area of this aabb.
  float GetSurfaceArea() const;

  // Does this aabb completely contain the given aabb (not an intersection test).
  bool Contains(const Aabb& aabb) const;
  // Expand the to include the given point.
  void Expand(const Vector3& point);
  // Combine the two aabbs into a new one
  static Aabb Combine(const Aabb& lhs, const Aabb& rhs);
  // See if this aabb is equal to another (with epsilon). Used for unit testing.
  bool Compare(const Aabb& rhs, float epsilon) const;

  void Transform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation);
  
  Vector3 GetMin() const;
  Vector3 GetMax() const;
  Vector3 GetCenter() const;
  Vector3 GetHalfSize() const;

  DebugShape& DebugDraw() const;

  Vector3 mMin;
  Vector3 mMax;
};

//-----------------------------------------------------------------------------Triangle
class Triangle
{
public:
  Triangle();
  Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2);

  DebugShape& DebugDraw() const;

  Vector3 mPoints[3];
};

//-----------------------------------------------------------------------------Plane
class Plane
{
public:
  Plane();
  // Create the plane from a triangle
  Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2);
  // Create the plane from a point and a normal
  Plane(const Vector3& normal, const Vector3& point);

  // Create a plane from a triangle. The plane's normal should be normalized.
  void Set(const Vector3& p0, const Vector3& p1, const Vector3& p2);
  // Create a plane from a point and a normal. The plane's normal should be normalized.
  void Set(const Vector3& normal, const Vector3& point);

  // Get the normal of this plane.
  Vector3 GetNormal() const;
  // Get the 'd' value (the distance from the origin) which can be computed as Dot(origin - pointOnPlane, normal)
  float GetDistance() const;

  DebugShape& DebugDraw(float size) const;
  DebugShape& DebugDraw(float sizeX, float sizeY) const;

  Math::Vector4 mData;
};

//-----------------------------------------------------------------------------Frustum
class Frustum
{
public:

  void Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
           const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf);

  // Returns the start of an array of the 6 planes of the frustum.
  Math::Vector4* GetPlanes() const;

  DebugShape& DebugDraw() const;

  // The normals all point inwards (towards the centroid of the frustum).
  Plane mPlanes[6];

  // To easily draw the frustum the original points it was constructed from are stored.
  Math::Vector3 mPoints[8];
};
