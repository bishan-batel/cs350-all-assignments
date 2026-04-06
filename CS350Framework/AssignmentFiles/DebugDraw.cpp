/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: DebugDraw.cpp
Purpose: Debug Draw Implementation
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_2
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"

#define ShowDebugDrawWarnings true

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape() {
  mColor = Vector4(.6f);
  mMask = (unsigned int)-1;
  mTimer = 0;
  mOnTop = false;
  mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color) {
  mColor = color;
  return *this;
}

DebugShape& DebugShape::OnTop(bool state) {
  mOnTop = state;
  return *this;
}

DebugShape& DebugShape::Time(float time) {
  mTimer = time;
  return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex) {
  mMask = 1 << bitIndex;
  return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform) {
  mTransform = transform;
  return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer() {
  mActiveMask = (unsigned int)-1;
  mApplication = NULL;
}

void DebugDrawer::Update(float dt) {
  std::vector<DebugShape> newShapes;
  for (size_t i = 0; i < mShapes.size(); ++i) {
    DebugShape& shape = mShapes[i];
    shape.mTimer -= dt;

    // If the shape still has time left then add it to the list of shapes to keep drawing,
    // anything that has a timer that ran out will not be in the new list
    if (shape.mTimer >= 0) { newShapes.push_back(shape); }
  }

  mShapes.swap(newShapes);
}

void DebugDrawer::Draw() {
  for (size_t i = 0; i < mShapes.size(); ++i) {
    DebugShape& shape = mShapes[i];

    // If the shape doesn't have one of the active mask bits set then don't draw it
    if ((shape.mMask & mActiveMask) == 0) { continue; }

    // If this shape always draws on top then disable depth testing
    if (shape.mOnTop) { glDisable(GL_DEPTH_TEST); }

    // Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
    float radians;
    Vector3 scale, translation, axis;
    Matrix3 rotationMat;
    shape.mTransform.Decompose(&scale, &rotationMat, &translation);
    Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
    glPushMatrix();
    // Set the transform
    glTranslatef(translation.x, translation.y, translation.z);
    glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
    glScalef(scale.x, scale.y, scale.z);

    glBegin(GL_LINES);
    glColor3fv(shape.mColor.array);

    // Draw all of the line segments of this shape
    for (size_t j = 0; j < shape.mSegments.size(); ++j) {
      LineSegment& segment = shape.mSegments[j];

      glVertex3fv(segment.mStart.array);
      glVertex3fv(segment.mEnd.array);
    }

    glEnd();
    glPopMatrix();

    // Make sure to re-enable depth testing
    if (shape.mOnTop) { glEnable(GL_DEPTH_TEST); }
  }
}

DebugShape& DebugDrawer::GetNewShape() {
  mShapes.push_back(DebugShape());
  return mShapes.back();
}

DebugShape& DebugDrawer::DrawPoint(const Vector3& point) { return DrawSphere(Sphere(point, 0.1f)); }

DebugShape& DebugDrawer::DrawLine(const LineSegment& line) {
  /******Student:Assignment2******/
  // Draw a simple line
  DebugShape& shape = GetNewShape();

  shape.mSegments.emplace_back(line);

  return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t) {
  /******Student:Assignment2******/
  // Draw a ray to a given t-length. The ray must have an arrow head for visualization
  DebugShape& shape = GetNewShape();

  const Vector3 dir = ray.mDirection;
  Vector3 end_point = ray.mStart + dir * t;
  shape.mSegments.emplace_back(ray.mStart, end_point);

  Vector3 basis1{Vector3::cXAxis.Cross(dir)};

  if (basis1.Length() < 0.01f) { basis1 = Vector3::cYAxis.Cross(dir); }

  Vector3 basis2{basis1.Cross(dir)};

  basis1.Normalize();
  basis2.Normalize();

  const Vector3 arrow_base = ray.mStart + (dir * (t - 0.2f));

  constexpr float thickness = 0.1f;
  Vector3 p1 = arrow_base + basis1 * thickness + basis2 * thickness;
  Vector3 p2 = arrow_base + basis1 * thickness - basis2 * thickness;
  Vector3 p3 = arrow_base - basis1 * thickness - basis2 * thickness;
  Vector3 p4 = arrow_base - basis1 * thickness + basis2 * thickness;

  shape.mSegments.emplace_back(p1, p2);
  shape.mSegments.emplace_back(p2, p3);
  shape.mSegments.emplace_back(p3, p4);
  shape.mSegments.emplace_back(p4, p1);

  shape.mSegments.emplace_back(p1, end_point);
  shape.mSegments.emplace_back(p2, end_point);
  shape.mSegments.emplace_back(p3, end_point);
  shape.mSegments.emplace_back(p4, end_point);

  return shape;
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere) {
  /******Student:Assignment2******/
  // Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
  // Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
  DebugShape& shape = GetNewShape();

  Vector3 center = sphere.GetCenter();
  float radius = sphere.GetRadius();

  const auto draw_disc = [&](Vector3 basis1, Vector3 basis2) {
    constexpr float segments = 64.0f;

    for (float theta = 0.0f; theta < Math::cTwoPi; theta += Math::cTwoPi / segments) {
      const float x1 = Math::Cos(theta);
      const float y1 = Math::Sin(theta);
      const float x2 = Math::Cos(theta + Math::cTwoPi / segments);
      const float y2 = Math::Sin(theta + Math::cTwoPi / segments);

      shape.mSegments.emplace_back(
        center + (x1 * basis1 + y1 * basis2) * radius,
        center + (x2 * basis1 + y2 * basis2) * radius
      );
    }
  };

  draw_disc(Vector3::cXAxis, Vector3::cYAxis);
  draw_disc(Vector3::cXAxis, Vector3::cZAxis);
  draw_disc(Vector3::cYAxis, Vector3::cZAxis);

  if (mApplication == nullptr) { return shape; }
  const Vector3 camera_pos = mApplication->mCamera.mTranslation;

  const Vector3 cam_to_sphere = (center - camera_pos).Normalized();
  const float distance = (camera_pos - sphere.GetCenter()).Length();
  const float len = Math::Sqrt(distance * distance - radius * radius);
  const float r_prime = radius * len / distance;

  center = center - Math::Sqrt(radius * radius - r_prime * r_prime) * cam_to_sphere;
  radius = r_prime;

  Vector3 horizon_basis1{Vector3::cXAxis.Cross(cam_to_sphere)};

  if (horizon_basis1.Length() < 0.01f) { horizon_basis1 = Vector3::cYAxis.Cross(cam_to_sphere); }

  Vector3 horizon_basis2{horizon_basis1.Cross(cam_to_sphere)};

  horizon_basis1.Normalize();
  horizon_basis2.Normalize();

  draw_disc(horizon_basis1, horizon_basis2);

  return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb) {
  /******Student:Assignment2******/
  // Draw all edges of an aabb. Make sure to not mis-match edges!
  DebugShape& shape = GetNewShape();

  const Vector3 min = aabb.GetMin();
  const Vector3 max = aabb.GetMax();

  shape.mSegments.reserve(12);
  shape.mSegments.emplace_back(Vector3{min.x, min.y, min.z}, Vector3{max.x, min.y, min.z});

  shape.mSegments.emplace_back(Vector3{max.x, min.y, min.z}, Vector3{max.x, max.y, min.z});

  shape.mSegments.emplace_back(Vector3{max.x, max.y, min.z}, Vector3{min.x, max.y, min.z});

  shape.mSegments.emplace_back(Vector3{min.x, max.y, min.z}, Vector3{min.x, min.y, min.z});

  shape.mSegments.emplace_back(Vector3{min.x, min.y, max.z}, Vector3{max.x, min.y, max.z});

  shape.mSegments.emplace_back(Vector3{max.x, min.y, max.z}, Vector3{max.x, max.y, max.z});

  shape.mSegments.emplace_back(Vector3{max.x, max.y, max.z}, Vector3{min.x, max.y, max.z});

  shape.mSegments.emplace_back(Vector3{min.x, max.y, max.z}, Vector3{min.x, min.y, max.z});

  shape.mSegments.emplace_back(Vector3{min.x, min.y, min.z}, Vector3{min.x, min.y, max.z});

  shape.mSegments.emplace_back(Vector3{max.x, min.y, min.z}, Vector3{max.x, min.y, max.z});

  shape.mSegments.emplace_back(Vector3{max.x, max.y, min.z}, Vector3{max.x, max.y, max.z});

  shape.mSegments.emplace_back(Vector3{min.x, max.y, min.z}, Vector3{min.x, max.y, max.z});

  return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle) {
  /******Student:Assignment2******/
  // Draw the 3 edges of a triangles
  DebugShape& shape = GetNewShape();

  shape.mSegments = {
    {triangle.mPoints[0], triangle.mPoints[1]},
    {triangle.mPoints[1], triangle.mPoints[2]},
    {triangle.mPoints[2], triangle.mPoints[0]},
  };

  return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY) {
  Vector3 center = plane.GetDistance() * plane.GetNormal();

  Vector3 basis1{Vector3::cXAxis.Cross(plane.GetNormal())};

  if (basis1.Length() < 0.01f) { basis1 = Vector3::cYAxis.Cross(plane.GetNormal()); }

  Vector3 basis2{basis1.Cross(plane.GetNormal())};

  basis1.Normalize();
  basis2.Normalize();

  sizeX /= 2;
  sizeY /= 2;
  DebugShape& quad = DrawQuad(
    center + basis1 * sizeX + basis2 * sizeY,
    center + basis1 * sizeX - basis2 * sizeY,
    center - basis1 * sizeX - basis2 * sizeY,
    center - basis1 * sizeX + basis2 * sizeY
  );
  DrawRay(Ray{center, plane.GetNormal()}, 2.0f);
  return quad;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3) {
  /******Student:Assignment2******/
  // Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
  DebugShape& shape = GetNewShape();

  const Vector3 centroid = (p0 + p1 + p2 + p3) / 4.0f;

  const Vector3 basis1 = (p0 - centroid).Normalized();
  const Vector3 basis2 = (p1 - centroid).Normalized();

  std::vector<Vector3> vertices{p0, p1, p2, p3};

  const auto angle_of = [&](Vector3 v) {
    const Vector3 to_v = (v - centroid);
    const Vector2 project_coord{basis1.Dot(to_v), basis2.Dot(to_v)};
    return Math::ArcTan2(project_coord.y, project_coord.x);
  };

  std::sort(vertices.begin(), vertices.end(), [&](Vector3 a, Vector3 b) { return angle_of(a) < angle_of(b); });

  shape.mSegments = {
    {vertices[0], vertices[1]},
    {vertices[1], vertices[2]},
    {vertices[2], vertices[3]},
    {vertices[3], vertices[0]},
  };

  return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum) {
  /******Student:Assignment2******/
  // Draw the 6 faces of the frustum using the 8 frustum points.
  // See Frustum.Set for the point order. For example, Points[4] is left-bottom-front.
  DebugShape& shape = GetNewShape();
  const Vector3(&points)[8] = frustum.mPoints;

  shape.mSegments = {
    // near plane
    {points[0], points[1]},
    {points[1], points[2]},
    {points[2], points[3]},
    {points[3], points[0]},

    // far plane
    {points[4], points[5]},
    {points[5], points[6]},
    {points[6], points[7]},
    {points[7], points[4]},

    // extrusion lines
    {points[0], points[4]},
    {points[1], points[5]},
    {points[2], points[6]},
    {points[3], points[7]},
  };

  return shape;
}
