/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: Shapes.cpp
Purpose: Shape primitives implementatiosn
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_2
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"

#include <numeric>

//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
    mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
    mStart = start;
    mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
    return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
    mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
    mStart = start;
    mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
    Ray transformedRay;
    transformedRay.mStart = Math::TransformPoint(transform, mStart);
    transformedRay.mDirection = Math::TransformNormal(transform, mDirection);
    return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
    return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
    return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------PCA Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points)
{
    /******Student:Assignment2******/

    const Vector3 mean{
        std::accumulate(
            points.begin(), points.end(), Vector3{}, std::plus<Vector3>{}
        ) / static_cast<float>(points.size()),
    };

    Matrix3 output{};

    for (const auto& point : points)
    {
        const auto p = point - mean;
        output.m00 += p.x * p.x;
        output.m01 += p.x * p.y;
        output.m02 += p.x * p.z;

        output.m10 += p.y * p.x;
        output.m11 += p.y * p.y;
        output.m12 += p.y * p.z;

        output.m11 += p.z * p.x;
        output.m12 += p.z * p.y;
        output.m22 += p.z * p.z;
    }

    output /= (static_cast<float>(points.size()) - 1.f);

    return Matrix3::cIdentity;
}

Matrix3 ComputeJacobiRotation(const Matrix3& matrix)
{
    /******Student:Assignment2******/
    // Compute the jacobi rotation matrix that will turn the largest (magnitude) off-diagonal element of the input
    // matrix into zero. Note: the input matrix should always be (near) symmetric.

    // Find largest off-diagonal element
    int p = 0, q = 1;
    float max_val = Math::Abs(matrix.m01);

    if (Math::Abs(matrix.m02) > max_val)
    {
        p = 0;
        q = 2;
        max_val = Math::Abs(matrix.m02);
    }
    if (Math::Abs(matrix.m12) > max_val)
    {
        p = 1;
        q = 2;
        max_val = Math::Abs(matrix.m12);
    }

    Matrix3 jacobi = Matrix3::cIdentity;

    // If already nearly diagonal
    if (max_val < Math::DebugEpsilon()) return jacobi;

    const float app = matrix.array[p * 3 + p];
    const float aqq = matrix.array[q * 3 + q];
    const float apq = matrix.array[p * 3 + q];

    // Compute Jacobi rotation angle
    float tau = (aqq - app) / (2.0f * apq);
    float t;

    if (tau >= 0)
    {
        t = 1.0f / (tau + sqrt(1.0f + tau * tau));
    }
    else
    {
        t = -1.0f / (-tau + sqrt(1.0f + tau * tau));
    }

    const float c = 1.0f / sqrt(1.0f + t * t);
    const float s = t * c;

    // Fill rotation matrix
    jacobi.array[p * 3 + p] = c;
    jacobi.array[q * 3 + q] = c;
    jacobi.array[p * 3 + q] = s;
    jacobi.array[q * 3 + p] = -s;

    return jacobi;
}

void ComputeEigenValuesAndVectors(
    const Matrix3& covariance,
    Vector3& eigenValues,
    Matrix3& eigenVectors,
    const int maxIterations
)
{
    /******Student:Assignment2******/
    // Iteratively rotate off the largest off-diagonal elements until the resultant matrix is diagonal or maxIterations.

    Matrix3 values = covariance;
    eigenVectors = Matrix3::cIdentity;

    for (int i = 0; i < maxIterations; ++i)
    {
        // Check if nearly diagonal
        const float off_diagonal = Math::Abs(values.m01) + Math::Abs(values.m02) + Math::Abs(values.m12);

        if (off_diagonal < Math::DebugEpsilon()) break;

        // Compute Jacobi rotation
        Matrix3 jacobi = ComputeJacobiRotation(values);

        values = jacobi.Transposed() * values * jacobi;

        eigenVectors = eigenVectors * jacobi;
    }

    // Diagonal elements are eigenvalues
    eigenValues[0] = values.m00;
    eigenValues[1] = values.m11;
    eigenValues[2] = values.m22;
}


//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
    mCenter = Vector3::cZero;
    mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
    mCenter = center;
    mRadius = radius;
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
    Vector3 min = points.at(0);
    Vector3 max = points.at(0);

    for (const Vector3& point : points)
    {
        min = Math::Min(min, point);
        max = Math::Max(max, point);
    }

    const Vector3 centroid{(min + max) / 2.f};

    float max_distance{0.f};


    for (const Vector3& point : points)
    {
        const float distance2 = (point - centroid).LengthSq();
        if (distance2 <= max_distance) continue;
        max_distance = distance2;
    }

    mCenter = centroid;
    mRadius = Math::Sqrt(max_distance);

    /******Student:Assignment2******/
    // The centroid method is roughly describe as: find the centroid (not mean) of all
    // points and then find the furthest away point from the centroid.
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
    const Vector3 p0 = points.at(0);

    Vector3 min_x = p0, max_x = p0;
    Vector3 min_y = p0, max_y = p0;
    Vector3 min_z = p0, max_z = p0;

    for (const Vector3& point : points)
    {
        if (point.x < min_x.x) min_x = point;
        if (point.y < min_y.y) min_y = point;
        if (point.z < min_z.z) min_z = point;

        if (point.x > max_x.x) max_x = point;
        if (point.y > max_y.y) max_y = point;
        if (point.z > max_z.z) max_z = point;
    }

    Vector3 spread{
        (max_x - min_x).LengthSq(),
        (max_y - min_y).LengthSq(),
        (max_z - min_z).LengthSq(),
    };

    Vector3 max, min;

    if (spread.x > spread.y && spread.x > spread.z)
    {
        min = min_x;
        max = max_x;
    }
    else if (spread.y > spread.x && spread.y > spread.z)
    {
        min = min_y;
        max = max_y;
    }
    else
    {
        min = min_z;
        max = max_z;
    }

    mCenter = (max + min) / 2.f;
    // mRadius = (max - min).Length() / 2.f;
    mRadius = 0.0f;

    for (const Vector3& point : points)
    {
        const float dist = (mCenter - point).Length();

        // if the distance is less than our radius, skip
        if (dist < mRadius)
        {
            continue;
        }

        const Vector3 b = mCenter - (point - mCenter).Normalized() * mRadius;
        const Vector3 c_prime = (point + b) / 2.f;

        mCenter = c_prime;
        mRadius = (b - c_prime).Length();
    }

    /******Student:Assignment2******/
    // The ritter method:
    // Find the largest spread on each axis.
    // Find which axis' pair of points are the furthest (euclidean distance) apart.
    // Choose the center of this line as the sphere center. Now incrementally expand the sphere.
}

void Sphere::ComputePCA(const std::vector<Vector3>& points)
{
    // The PCA method:
    // Compute the eigen values and vectors. Take the largest eigen vector as the axis of largest spread.
    // Compute the sphere center as the center of this axis then expand by all points.
    const Matrix3 covariance = ComputeCovarianceMatrix(points);
    Vector3 eigenvalues;
    Matrix3 eigenvectors;
    ComputeEigenValuesAndVectors(covariance, eigenvalues, eigenvectors, 10);

    // find the axis of larg spread, biggest eigenvalue
    int axis_index = 0;
    if (eigenvalues.y > eigenvalues[axis_index]) axis_index = 1;
    if (eigenvalues.z > eigenvalues[axis_index]) axis_index = 2;

    Vector3 axis = eigenvectors.Basis(axis_index);
    axis.Normalize();
    float min_projection = axis.Dot(points[0]);
    float max_projection = min_projection;

    for (size_t i = 1; i < points.size(); ++i)
    {
        float proj = axis.Dot(points[i]);
        min_projection = std::min(proj, min_projection);
        max_projection = std::max(proj, max_projection);
    }

    // midpoint is the sphere's starting center
    const float midpoint = 0.5f * (min_projection + max_projection);
    mCenter = axis * midpoint;

    // iterative expansion

    float max_dist_sq = 0.0f;
    for (const Vector3& point : points)
    {
        const float distSq = (point - mCenter).LengthSq();
        max_dist_sq = std::max(distSq, max_dist_sq);
    }

    mRadius = sqrt(max_dist_sq);
}


bool Sphere::ContainsPoint(const Vector3& point)
{
    return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
    return mCenter;
}

float Sphere::GetRadius() const
{
    return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
    float posDiff = Math::Length(mCenter - rhs.mCenter);
    float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

    return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
    return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
    //set the aabb to an initial bad value (where the min is smaller than the max)
    mMin.Splat(Math::PositiveMax());
    mMax.Splat(-Math::PositiveMax());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
    mMin = min;
    mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
    return Aabb(center - halfExtents, center + halfExtents);
}

float Aabb::GetVolume() const
{
    /******Student:Assignment2******/
    // Return the aabb's volume
    const float length = mMax.x - mMin.x;
    const float width = mMax.y - mMin.y;
    const float height = mMax.z - mMin.z;
    return length * width * height;
}

float Aabb::GetSurfaceArea() const
{
    /******Student:Assignment2******/
    // Return the aabb's surface area
    const float length = mMax.x - mMin.x;
    const float width = mMax.y - mMin.y;
    const float height = mMax.z - mMin.z;
    return 2.0f * (length * width + length * height + width * height);
}

bool Aabb::Contains(const Aabb& aabb) const
{
    /******Student:Assignment2******/
    // Return if aabb is completely contained in this
    return PointAabb(aabb.mMin, mMin, mMax) && PointAabb(aabb.mMax, mMin, mMax);
}

void Aabb::Expand(const Vector3& point)
{
    for (uint32_t i = 0; i < 3; ++i)
    {
        mMin[i] = Math::Min(mMin[i], point[i]);
        mMax[i] = Math::Max(mMax[i], point[i]);
    }
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
    Aabb result;
    for (uint32_t i = 0; i < 3; ++i)
    {
        result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
        result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
    }
    return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
    float pos1Diff = Math::Length(mMin - rhs.mMin);
    float pos2Diff = Math::Length(mMax - rhs.mMax);

    return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation)
{
    const Vector3 center = GetCenter();

    const Vector3 half_size = GetHalfSize();
    const Vector3 r{
        half_size.x * scale.x,
        half_size.y * scale.y,
        half_size.z * scale.z,
    };

    const Vector3 r_dir{
        abs(rotation.m00 * r.x) + abs(rotation.m01 * r.y) + abs(rotation.m02 * r.z),
        abs(rotation.m10 * r.x) + abs(rotation.m11 * r.y) + abs(rotation.m12 * r.z),
        abs(rotation.m20 * r.x) + abs(rotation.m21 * r.y) + abs(rotation.m22 * r.z),
    };

    const Vector3 new_center = translation + Math::Transform(rotation, scale * center);

    mMin = new_center - r_dir;
    mMax = new_center + r_dir;

    /******Student:Assignment2******/
    // Compute aabb of the this aabb after it is transformed.
    // You should use the optimize method discussed in class (not transforming all 8 points).
}

Vector3 Aabb::GetMin() const
{
    return mMin;
}

Vector3 Aabb::GetMax() const
{
    return mMax;
}

Vector3 Aabb::GetCenter() const
{
    return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
    return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
    return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
    mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    mPoints[0] = p0;
    mPoints[1] = p1;
    mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
    return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
    mData = Vector4::cZero;
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
    Set(normal, point);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    /******Student:Assignment1******/
    // Set mData from the 3 points. Note: You should most likely normalize the plane normal.

    Vector3 span1 = (p1 - p0);
    Vector3 span2 = (p2 - p0);

    Set(span1.Cross(span2), p0);
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
    /******Student:Assignment1******/
    // Set mData from the normal and point. Note: You should most likely normalize the plane normal.

    Vector3 n_normal{normal};

    if (n_normal.LengthSq() != 1.f)
    {
        n_normal.Normalize();
    }

    const float d{point.Dot(n_normal)};

    mData = Vector4{n_normal.x, n_normal.y, n_normal.z, d};
}

Vector3 Plane::GetNormal() const
{
    return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
    return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
    return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
    return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
                  const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
    mPoints[0] = lbn;
    mPoints[1] = rbn;
    mPoints[2] = rtn;
    mPoints[3] = ltn;
    mPoints[4] = lbf;
    mPoints[5] = rbf;
    mPoints[6] = rtf;
    mPoints[7] = ltf;

    //left
    mPlanes[0].Set(lbf, ltf, lbn);
    //right
    mPlanes[1].Set(rbn, rtf, rbf);
    //top
    mPlanes[2].Set(ltn, ltf, rtn);
    //bot
    mPlanes[3].Set(rbn, lbf, lbn);
    //near
    mPlanes[4].Set(lbn, ltn, rbn);
    //far
    mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
    return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
    return gDebugDrawer->DrawFrustum(*this);
}
