#include "Precompiled.hpp"
#include "Shapes.hpp"
#include "Geometry.hpp"
#include "Math/Utilities.hpp"
#include "DebugDraw.hpp"
#include "Components.hpp"
#include "Application.hpp"
#include "UnitTests.hpp"

void TestBarycentricLine(const Vector3& p0, const Vector3& p1, float expectedU, float expectedV, FILE* outFile)
{
  float epsilon = 0.05f;
  Vector3 testPoint = p0 * expectedU + p1 * expectedV;
  float u, v;
  bool result = BarycentricCoordinates(testPoint, p0, p1, u, v, epsilon);

  if(outFile != NULL)
    fprintf(outFile, "  Result:%s (u, v):%s\n", result ? "true" : "false", PrintVector2(u, v).c_str());

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  gDebugDrawer->DrawPoint(testPoint).Color(Vector4(0, 1, 0, 1));
  gDebugDrawer->DrawPoint(p0).Color(color);
  gDebugDrawer->DrawPoint(p1).Color(color);
  gDebugDrawer->DrawLine(LineSegment(p0, p1)).Color(color);
}

void TestBarycentricTriangle(const Vector3& p0, const Vector3& p1, const Vector3& p2,
                             float expectedU, float expectedV, float expectedW,
                             FILE* outFile)
{
  float epsilon = 0.05f;
  Vector3 testPoint = p0 * expectedU + p1 * expectedV + p2 * expectedW;

  float u, v, w;
  bool result = BarycentricCoordinates(testPoint, p0, p1, p2, u, v, w, epsilon);

  if(outFile != NULL)
    fprintf(outFile, "  Result:%s (u, v, w): %s\n", result ? "true" : "false", PrintVector3(u, v, w).c_str());

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  gDebugDrawer->DrawPoint(testPoint).Color(Vector4(0, 1, 0, 1));
  gDebugDrawer->DrawPoint(p0).Color(color);
  gDebugDrawer->DrawPoint(p1).Color(color);
  gDebugDrawer->DrawPoint(p2).Color(color);
  gDebugDrawer->DrawTriangle(Triangle(p0, p1, p2)).Color(color);
}

void TestRayPlane(const Ray& ray, const Plane& plane, FILE* outFile)
{
  float t;
  bool result = RayPlane(ray.mStart, ray.mDirection, plane.mData, t);


  if(outFile != NULL)
  {
    if(result)
      fprintf(outFile, "  Result:true t:%s\n", PrintFloat(t).c_str());
    else
      fprintf(outFile, "  Result:false\n");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  ray.DebugDraw(5).Color(color);
  plane.DebugDraw(10).Color(color);
  gDebugDrawer->DrawPoint(ray.GetPoint(t)).Color(color);
}

void TestRayTriangle(const Ray& ray, const Triangle& tri, FILE* outFile)
{
  float epsilon = 0.0f;
  float t;
  bool result = RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], t, epsilon);


  if(outFile != NULL)
  {
    if(result)
      fprintf(outFile, "  Result:true t:%s\n", PrintFloat(t).c_str());
    else
      fprintf(outFile, "  Result:false\n");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  ray.DebugDraw(5).Color(color);
  tri.DebugDraw().Color(color);
  gDebugDrawer->DrawPoint(ray.GetPoint(t)).Color(color);
}

void TestRaySphere(const Ray& ray, const Sphere& sphere, FILE* outFile)
{
  float t = 0;
  bool result = RaySphere(ray.mStart, ray.mDirection, sphere.mCenter, sphere.mRadius, t);

  if(outFile != NULL)
  {
    if(result)
      fprintf(outFile, "  Result:true t:%s\n", PrintFloat(t).c_str());
    else
      fprintf(outFile, "  Result:false\n");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  ray.DebugDraw(5).Color(color);
  sphere.DebugDraw().Color(color);
  gDebugDrawer->DrawPoint(ray.GetPoint(t)).Color(color);
}

void TestRayAabb(const Ray& ray, const Aabb& aabb, FILE* outFile)
{
  float t = 0;
  bool result = RayAabb(ray.mStart, ray.mDirection, aabb.mMin, aabb.mMax, t);

  if(outFile != NULL)
  {
    if(result)
      fprintf(outFile, "  Result:true t:%s\n", PrintFloat(t).c_str());
    else
      fprintf(outFile, "  Result:false\n");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  ray.DebugDraw(5).Color(color);
  aabb.DebugDraw().Color(color);
  gDebugDrawer->DrawPoint(ray.GetPoint(t)).Color(color);
}

void TestPlaneSet(const Plane& plane, FILE* outFile)
{
  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", PrintVector4(plane.mData).c_str());
  }
  plane.DebugDraw(10);
}

void TestPlaneSet(const Vector3& normal, const Vector3& point, FILE* outFile)
{
  Plane plane;
  plane.Set(normal, point);
  TestPlaneSet(plane, outFile);
}

void TestPlaneSet(const Vector3& pointA, const Vector3& pointB, const Vector3& pointC, FILE* outFile)
{
  Plane plane;
  plane.Set(pointA, pointB, pointC);
  TestPlaneSet(plane, outFile);
}

void TestPointPlane(const Vector3& point, const Plane& plane, float epsilon, FILE* outFile)
{
  int result = PointPlane(point, plane.mData, epsilon);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", IntersectionType::Names[result]);
  }

  Vector4 color = Vector4(1, 1, 1, 1);
  if(result == IntersectionType::Inside)
    color = Vector4(1, 0, 0, 1);
  else if(result == IntersectionType::Overlaps)
    color = Vector4(0, 1, 0, 1);
  else if(result == IntersectionType::Outside)
    color = Vector4(0, 0, 1, 1);

  gDebugDrawer->DrawPoint(point).Color(color);
  plane.DebugDraw(10).Color(color);
}

void TestPointSphere(const Vector3& point, const Sphere& sphere, FILE* outFile)
{
  bool result = PointSphere(point, sphere.mCenter, sphere.mRadius);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", result ? "true" : "false");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  gDebugDrawer->DrawPoint(point).Color(color);
  sphere.DebugDraw().Color(color);
}

void TestPointAabb(const Vector3& point, const Aabb& aabb, FILE* outFile)
{
  bool result = PointAabb(point, aabb.mMin, aabb.mMax);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", result ? "true" : "false");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  gDebugDrawer->DrawPoint(point).Color(color);
  aabb.DebugDraw().Color(color);
}

void TestProjectPointOnPlane(const Vector3& point, const Plane& plane, FILE* outFile)
{
  Vector3 result = ProjectPointOnPlane(point, plane.GetNormal(), plane.GetDistance());
  if (outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", PrintVector3(result).c_str());
  }

  gDebugDrawer->DrawPoint(point).Color(Vector4(1, 1, 1, 1));
  gDebugDrawer->DrawPoint(result).Color(Vector4(0, 1, 0, 1));
  plane.DebugDraw(10).Color(Vector4(0, 0, 1, 1));
}

void TestPlaneSphere(const Plane& plane, const Sphere& sphere, FILE* outFile)
{
  int result = PlaneSphere(plane.mData, sphere.mCenter, sphere.mRadius);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", IntersectionType::Names[result]);
  }

  Vector4 color = Vector4(1, 1, 1, 1);
  if(result == IntersectionType::Inside)
    color = Vector4(1, 0, 0, 1);
  else if(result == IntersectionType::Overlaps)
    color = Vector4(0, 1, 0, 1);
  else if(result == IntersectionType::Outside)
    color = Vector4(0, 0, 1, 1);

  sphere.DebugDraw().Color(color);
  plane.DebugDraw(10).Color(color);

  Vector3 closestPoint = ProjectPointOnPlane(sphere.mCenter, plane.GetNormal(), plane.GetDistance());
  gDebugDrawer->DrawPoint(closestPoint).Color(color);
}

void TestPlaneAabb(const Plane& plane, const Aabb& aabb, FILE* outFile)
{
  int result = PlaneAabb(plane.mData, aabb.GetMin(), aabb.GetMax());

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", IntersectionType::Names[result]);
  }

  Vector4 color = Vector4(1, 1, 1, 1);
  if(result == IntersectionType::Inside)
    color = Vector4(1, 0, 0, 1);
  else if(result == IntersectionType::Overlaps)
    color = Vector4(0, 1, 0, 1);
  else if(result == IntersectionType::Outside)
    color = Vector4(0, 0, 1, 1);

  aabb.DebugDraw().Color(color);
  plane.DebugDraw(10).Color(color);
}

void TestPlaneTriangle(const Plane& plane, const Triangle& tri, float epsilon, FILE* outFile)
{
  int result = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", IntersectionType::Names[result]);
  }

  Vector4 color = Vector4(1, 1, 1, 1);
  if(result == IntersectionType::Inside)
    color = Vector4(1, 0, 0, 1);
  else if(result == IntersectionType::Overlaps)
    color = Vector4(0, 1, 0, 1);
  else if(result == IntersectionType::Outside)
    color = Vector4(0, 0, 1, 1);
  else if(result == IntersectionType::Coplanar)
    color = Vector4(0.5f);

  tri.DebugDraw().Color(color);
  plane.DebugDraw(10).Color(color);
}

void TestSphereSphere(const Sphere& sphere0, const Sphere& sphere1, FILE* outFile)
{
  bool result = SphereSphere(sphere0.mCenter, sphere0.mRadius, sphere1.mCenter, sphere1.mRadius);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", result ? "true" : "false");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  sphere0.DebugDraw().Color(color);
  sphere1.DebugDraw().Color(color);
}

void TestAabbAabb(const Aabb& aabb0, const Aabb& aabb1, FILE* outFile)
{
  bool result = AabbAabb(aabb0.mMin, aabb0.mMax, aabb1.mMin, aabb1.mMax);
  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", result ? "true" : "false");
  }

  Vector4 color = Vector4(0, 0, 1, 1);
  if(result == true)
    color = Vector4(1, 0, 0, 1);

  aabb0.DebugDraw().Color(color);
  aabb1.DebugDraw().Color(color);
}

void TestFrustumTriangle(const Frustum& frustum, const Triangle& tri, FILE* outFile)
{
  float epsilon = 0.0001f;
  IntersectionType::Type result = FrustumTriangle(frustum.GetPlanes(), tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);

  if(outFile != NULL)
  {
    fprintf(outFile, " Result:%s\n", IntersectionType::Names[result]);
  }

  Vector4 color = Vector4(1, 1, 1, 1);
  if(result == IntersectionType::Inside)
    color = Vector4(1, 0, 0, 1);
  else if(result == IntersectionType::Overlaps)
    color = Vector4(0, 1, 0, 1);
  else if(result == IntersectionType::Outside)
    color = Vector4(0, 0, 1, 1);

  frustum.DebugDraw().Color(color);
  tri.DebugDraw().Color(color);
}

void TestFrustumSphere(const Frustum& frustum, const Sphere& sphere, FILE* outFile)
{
  size_t axis = 0;
  IntersectionType::Type result = FrustumSphere(frustum.GetPlanes(), sphere.mCenter, sphere.mRadius, axis);

  if(outFile != NULL)
  {
    fprintf(outFile, " Result:%s\n", IntersectionType::Names[result]);
  }

  Vector4 color = Vector4(1, 1, 1, 1);
  if(result == IntersectionType::Inside)
    color = Vector4(1, 0, 0, 1);
  else if(result == IntersectionType::Overlaps)
    color = Vector4(0, 1, 0, 1);
  else if(result == IntersectionType::Outside)
    color = Vector4(0, 0, 1, 1);

  frustum.DebugDraw().Color(color);
  sphere.DebugDraw().Color(color);
}

void TestFrustumAabb(const Frustum& frustum, const Aabb& aabb, FILE* outFile)
{
  size_t axis = 0;
  IntersectionType::Type result = FrustumAabb(frustum.GetPlanes(), aabb.GetMin(), aabb.GetMax(), axis);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Result:%s\n", IntersectionType::Names[result]);
  }

  Vector4 color = Vector4(1, 1, 1, 1);
  if(result == IntersectionType::Inside)
    color = Vector4(1, 0, 0, 1);
  else if(result == IntersectionType::Overlaps)
    color = Vector4(0, 1, 0, 1);
  else if(result == IntersectionType::Outside)
    color = Vector4(0, 0, 1, 1);

  frustum.DebugDraw().Color(color);
  aabb.DebugDraw().Color(color);
}
void ProjectPointOnPlaneTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 1, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-1, 0, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 1, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-1, 0, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, -1, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 0, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, -1, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 0, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2.41421f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.70711f, 1.70711f, 1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.414214f, 1, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.292893f, 0.292893f, 1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(2.41421f, 1, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.70711f, 1.70711f, -1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, -0.414214f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}

void ProjectPointOnPlaneTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.292893f, 0.292893f, -1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestProjectPointOnPlane(point, plane, file);
}


void BarycentricLineTest0(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 1, 1);
  Vector3 p1 = Vector3(-1, -1, -1);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 1, 1);
  Vector3 p1 = Vector3(-1, -1, -1);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 0.25f;
  float v = 0.75f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 0.75f;
  float v = 0.25f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = -0.1f;
  float v = 1.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 1.1f;
  float v = -0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 0.5f;
  float v = 0.5f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 0.1f;
  float v = 0.9f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -1, -1);
  Vector3 p1 = Vector3(1, 1, 1);
  float u = 0.9f;
  float v = 0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 2, 0);
  Vector3 p1 = Vector3(0, -2, 0);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 2, 0);
  Vector3 p1 = Vector3(0, -2, 0);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 0.25f;
  float v = 0.75f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 0.75f;
  float v = 0.25f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = -0.1f;
  float v = 1.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 1.1f;
  float v = -0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 0.5f;
  float v = 0.5f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 0.1f;
  float v = 0.9f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, -2, 0);
  Vector3 p1 = Vector3(0, 2, 0);
  float u = 0.9f;
  float v = 0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 2, 3);
  Vector3 p1 = Vector3(-1, -2, -3);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 2, 3);
  Vector3 p1 = Vector3(-1, -2, -3);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 0.25f;
  float v = 0.75f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 0.75f;
  float v = 0.25f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = -0.1f;
  float v = 1.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 1.1f;
  float v = -0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 0.5f;
  float v = 0.5f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 0.1f;
  float v = 0.9f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(1, 2, 3);
  float u = 0.9f;
  float v = 0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(2, 0, 0);
  Vector3 p1 = Vector3(-2, 0, 0);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(2, 0, 0);
  Vector3 p1 = Vector3(-2, 0, 0);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 0.25f;
  float v = 0.75f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 0.75f;
  float v = 0.25f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = -0.1f;
  float v = 1.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 1.1f;
  float v = -0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 0.5f;
  float v = 0.5f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 0.1f;
  float v = 0.9f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-2, 0, 0);
  Vector3 p1 = Vector3(2, 0, 0);
  float u = 0.9f;
  float v = 0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 2);
  Vector3 p1 = Vector3(0, 0, -2);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 2);
  Vector3 p1 = Vector3(0, 0, -2);
  float u = 1;
  float v = 0;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 0.25f;
  float v = 0.75f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 0.75f;
  float v = 0.25f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = -0.1f;
  float v = 1.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 1.1f;
  float v = -0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 0.5f;
  float v = 0.5f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 0.1f;
  float v = 0.9f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

void BarycentricLineTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = 0.9f;
  float v = 0.1f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

// Expected Result: false. Testing a degenerate line
void BarycentricLineTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(-1, -2, -3);
  float u = 0;
  float v = 1;
  
  TestBarycentricLine(p0, p1, u, v, file);
}

// Expected Result: true. Testing the use of epsilon
void BarycentricLineTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, -2);
  Vector3 p1 = Vector3(0, 0, 2);
  float u = -0.025f;
  float v = 1.025f;
  
  TestBarycentricLine(p0, p1, u, v, file);
}


void BarycentricTriangleTest0(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.33f;
  float v = 0.33f;
  float w = 0.33f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Halfway in-between p0 and p1
void BarycentricTriangleTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.5f;
  float v = 0.5f;
  float w = 0;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Halfway in-between p0 and p2
void BarycentricTriangleTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.5f;
  float v = 0;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Halfway in-between p1 and p2
void BarycentricTriangleTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0;
  float v = 0.5f;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Coordinate at p0
void BarycentricTriangleTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 1;
  float v = 0;
  float w = 0;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Coordinate at p1
void BarycentricTriangleTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0;
  float v = 1;
  float w = 0;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Coordinate at p2
void BarycentricTriangleTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0;
  float v = 0;
  float w = 1;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.25f;
  float v = 0.5f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.5f;
  float v = 0.25f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.25f;
  float v = 0.25f;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.25f;
  float v = 0.25f;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.5f;
  float v = 0.25f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.25f;
  float v = 0.5f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.1f;
  float v = 0.3f;
  float w = 0.6f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.3f;
  float v = 0.6f;
  float w = 0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.6f;
  float v = 0.1f;
  float w = 0.3f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.1f;
  float v = 0.6f;
  float w = 0.3f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.3f;
  float v = 0.1f;
  float w = 0.6f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.6f;
  float v = 0.3f;
  float w = 0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = -0.1f;
  float v = 0.45f;
  float w = 0.65f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.45f;
  float v = 0.65f;
  float w = -0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.65f;
  float v = -0.1f;
  float w = 0.45f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = -0.1f;
  float v = 0.65f;
  float w = 0.45f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.45f;
  float v = -0.1f;
  float w = 0.65f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 0.65f;
  float v = 0.45f;
  float w = -0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = -0.15f;
  float v = -0.05f;
  float w = 1.2f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = -0.05f;
  float v = 1.2f;
  float w = -0.15f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 1.2f;
  float v = -0.15f;
  float w = -0.05f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = -0.15f;
  float v = 1.2f;
  float w = -0.05f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = -0.05f;
  float v = -0.15f;
  float w = 1.2f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(0, 0, 0);
  Vector3 p1 = Vector3(1, 0, 0);
  Vector3 p2 = Vector3(0, 1, 0);
  float u = 1.2f;
  float v = -0.05f;
  float w = -0.15f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.33f;
  float v = 0.33f;
  float w = 0.33f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Halfway in-between p0 and p1
void BarycentricTriangleTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.5f;
  float v = 0.5f;
  float w = 0;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Halfway in-between p0 and p2
void BarycentricTriangleTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.5f;
  float v = 0;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Halfway in-between p1 and p2
void BarycentricTriangleTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0;
  float v = 0.5f;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Coordinate at p0
void BarycentricTriangleTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 1;
  float v = 0;
  float w = 0;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Coordinate at p1
void BarycentricTriangleTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0;
  float v = 1;
  float w = 0;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Coordinate at p2
void BarycentricTriangleTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0;
  float v = 0;
  float w = 1;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.25f;
  float v = 0.5f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.5f;
  float v = 0.25f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.25f;
  float v = 0.25f;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.25f;
  float v = 0.25f;
  float w = 0.5f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.5f;
  float v = 0.25f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.25f;
  float v = 0.5f;
  float w = 0.25f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.1f;
  float v = 0.3f;
  float w = 0.6f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.3f;
  float v = 0.6f;
  float w = 0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.6f;
  float v = 0.1f;
  float w = 0.3f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.1f;
  float v = 0.6f;
  float w = 0.3f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.3f;
  float v = 0.1f;
  float w = 0.6f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.6f;
  float v = 0.3f;
  float w = 0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = -0.1f;
  float v = 0.45f;
  float w = 0.65f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.45f;
  float v = 0.65f;
  float w = -0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.65f;
  float v = -0.1f;
  float w = 0.45f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = -0.1f;
  float v = 0.65f;
  float w = 0.45f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.45f;
  float v = -0.1f;
  float w = 0.65f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.65f;
  float v = 0.45f;
  float w = -0.1f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = -0.15f;
  float v = -0.05f;
  float w = 1.2f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = -0.05f;
  float v = 1.2f;
  float w = -0.15f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 1.2f;
  float v = -0.15f;
  float w = -0.05f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = -0.15f;
  float v = 1.2f;
  float w = -0.05f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = -0.05f;
  float v = -0.15f;
  float w = 1.2f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void BarycentricTriangleTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 1.2f;
  float v = -0.05f;
  float w = -0.15f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: false. Testing a degenerate triangle
void BarycentricTriangleTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(-1, -2, -3);
  Vector3 p1 = Vector3(-1, -2, -3);
  Vector3 p2 = Vector3(0, 0, 0);
  float u = 0;
  float v = 1;
  float w = 0;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Testing the use of epsilon
void BarycentricTriangleTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = -0.025f;
  float v = -0.025f;
  float w = 1.05f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

// Expected Result: true. Testing the use of epsilon
void BarycentricTriangleTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 p0 = Vector3(1, 0, -1);
  Vector3 p1 = Vector3(1, 0, 1);
  Vector3 p2 = Vector3(-1, 0, 0);
  float u = 0.52f;
  float v = 0.52f;
  float w = -0.04f;
  
  TestBarycentricTriangle(p0, p1, p2, u, v, w, file);
}

void PlaneSetTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 normal = Vector3(-1, -0, -0);
  Vector3 point = Vector3(2, 0, 0);
  TestPlaneSet(normal, point, file);
}

void PlaneSetTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 normal = Vector3(1, 1, 1);
  Vector3 point = Vector3(1, 2, 3);
  TestPlaneSet(normal, point, file);
}

void PlaneSetTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 pointA = Vector3(-1, 0, 0);
  Vector3 pointB = Vector3(0, 1, 0);
  Vector3 pointC = Vector3(1, 0, 0);
  TestPlaneSet(pointA, pointB, pointC, file);
}

void PlaneSetTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 pointA = Vector3(0, 0, 0);
  Vector3 pointB = Vector3(-1, -10, -1);
  Vector3 pointC = Vector3(1, -10, -1);
  TestPlaneSet(pointA, pointB, pointC, file);
}

// In front of plane
void PointPlaneTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, 1, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane
void PointPlaneTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(-1, 0, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, 1, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(-1, 0, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, 1, 1);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(-1, 0, 1);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, 1, 1.25f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, 1, 0.75f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane
void PointPlaneTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, -1, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane
void PointPlaneTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 0, 2);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, -1, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 0, 0);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, -1, 1);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 0, 1);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, -1, 1.25f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0, -1, 0.75f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(-1, 0, 1.25f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(-1, 0, 0.75f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 0, 1.25f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 0, 0.75f);
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 1));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane
void PointPlaneTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 2.41421f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane
void PointPlaneTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1.70711f, 1.70711f, 1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(-0.414214f, 1, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0.292893f, 0.292893f, 1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0.292893f, 1.70711f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 1, 1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0.46967f, 1.88388f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0.116116f, 1.53033f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane
void PointPlaneTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(2.41421f, 1, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane
void PointPlaneTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1.70711f, 1.70711f, -1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, -0.414214f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind Plane
void PointPlaneTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0.292893f, 0.292893f, -1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1.70711f, 0.292893f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Directly on Plane
void PointPlaneTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1, 1, -1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1.88388f, 0.46967f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1.53033f, 0.116116f, 0);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1.17678f, 1.17678f, 1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// In front of plane within epsilon
void PointPlaneTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0.823223f, 0.823223f, 1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(1.17678f, 1.17678f, -1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

// Behind of plane within epsilon
void PointPlaneTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Vector3 point = Vector3(0.823223f, 0.823223f, -1);
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 1, 0));
  
  TestPointPlane(point, plane, epsilon, file);
}

void PointSphereTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(4, 2, 3);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(2.5f, 2, 3);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.75f, 2, 3);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 5, 3);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 3.5f, 3);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2.75f, 3);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 6);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 4.5f);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 3.75f);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(2.73205f, 3.73205f, 4.73205f);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.86603f, 2.86603f, 3.86603f);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.43301f, 2.43301f, 3.43301f);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 3);
  Sphere sphere(Vector3(1, 2, 3), 1.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 0, 0);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.5f, 0, 0);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.25f, 0, 0);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 1, 0);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0.5f, 0);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0.25f, 0);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, 1);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, 0.5f);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, 0.25f);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.57735f, 0.57735f, 0.57735f);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.288675f, 0.288675f, 0.288675f);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.144338f, 0.144338f, 0.144338f);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointSphereTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, 0);
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestPointSphere(point, sphere, file);
}

void PointAabbTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 0, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.5f, 0, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.25f, 0, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.25f, 0, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.5f, 0, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-1, 0, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 1, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0.5f, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0.25f, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, -0.25f, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, -0.5f, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, -1, 0);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, 1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, 0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, 0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, -0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, -0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0, 0, -1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 1, 1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.5f, 0.5f, 0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.25f, 0.25f, 0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.25f, -0.25f, -0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.5f, -0.5f, -0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-1, -1, -1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 1, -1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.5f, 0.5f, -0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.25f, 0.25f, -0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.25f, -0.25f, 0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.5f, -0.5f, 0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-1, -1, 1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, -1, -1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.5f, -0.5f, -0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.25f, -0.25f, -0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.25f, 0.25f, 0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.5f, 0.5f, 0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-1, 1, 1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, -1, 1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.5f, -0.5f, 0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.25f, -0.25f, 0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.25f, 0.25f, -0.25f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-0.5f, 0.5f, -0.5f);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(-1, 1, -1);
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.2f, 2, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.1f, 2, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.05f, 2, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.95f, 2, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.9f, 2, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.8f, 2, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 6, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 4, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 3, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 1, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 0, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, -2, 3);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 4.4f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 3.7f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 3.35f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 2.65f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 2.3f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1, 2, 1.6f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.2f, 6, 4.4f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.1f, 4, 3.7f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.05f, 3, 3.35f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.95f, 1, 2.65f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.9f, 0, 2.3f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.8f, -2, 1.6f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.2f, 6, 1.6f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.1f, 4, 2.3f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.05f, 3, 2.65f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.95f, 1, 3.35f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.9f, 0, 3.7f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.8f, -2, 4.4f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.2f, -2, 1.6f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.1f, 0, 2.3f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.05f, 1, 2.65f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.95f, 3, 3.35f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.9f, 4, 3.7f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.8f, 6, 4.4f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.2f, -2, 4.4f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.1f, 0, 3.7f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(1.05f, 1, 3.35f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.95f, 3, 2.65f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.9f, 4, 2.3f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}

void PointAabbTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 point = Vector3(0.8f, 6, 1.6f);
  Aabb aabb(Vector3(0.9f, 0, 2.3f), Vector3(1.1f, 4, 3.7f));
  
  TestPointAabb(point, aabb, file);
}


// Ray in front of plane pointing forward
void RayPlaneTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 1, 0), Vector3(1, 0, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 1, 0), Vector3(1, 0, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 1, 0), Vector3(-1, -0, -0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 1, 0), Vector3(-1, -0, -0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 1, 0), Vector3(-0, 1, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 1, 0), Vector3(-0, 1, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 1, 0), Vector3(0, -0, 1));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 1, 0), Vector3(0, -0, 1));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(0, 1, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -1, -1), Vector3(0, 1, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(-0, -1, -0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -1, -1), Vector3(-0, -1, -0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(0, 0, -1));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -1, -1), Vector3(0, 0, -1));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -1, -1), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 1), Vector3(0, 0, 1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(0, 0, 1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 1), Vector3(-0, -0, -1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(-0, -0, -1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 1), Vector3(0, 1, -0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(0, 1, -0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 1), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -1), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3, 1, 0), Vector3(1, 0, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3, 1, 0), Vector3(1, 0, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3, 1, 0), Vector3(-1, -0, -0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3, 1, 0), Vector3(-1, -0, -0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3, 1, 0), Vector3(-0, 1, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3, 1, 0), Vector3(-0, 1, 0));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3, 1, 0), Vector3(0, -0, 1));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3, 1, 0), Vector3(0, -0, 1));
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 3, -1), Vector3(0, 1, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -3, -1), Vector3(0, 1, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 3, -1), Vector3(-0, -1, -0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -3, -1), Vector3(-0, -1, -0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 3, -1), Vector3(0, 0, -1));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -3, -1), Vector3(0, 0, -1));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 3, -1), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -3, -1), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 3), Vector3(0, 0, 1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -3), Vector3(0, 0, 1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 3), Vector3(-0, -0, -1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -3), Vector3(-0, -0, -1));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 3), Vector3(0, 1, -0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -3), Vector3(0, 1, -0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 3), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, -3), Vector3(-1, 0, 0));
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.29289f, 3.70711f, 3), Vector3(1, 1, 1));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3.70711f, -2.29289f, -3), Vector3(1, 1, 1));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.29289f, 3.70711f, 3), Vector3(-1, -1, -1));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3.70711f, -2.29289f, -3), Vector3(-1, -1, -1));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.29289f, 3.70711f, 3), Vector3(-0.707107f, 0.707107f, 0));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3.70711f, -2.29289f, -3), Vector3(-0.707107f, 0.707107f, 0));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.29289f, 3.70711f, 3), Vector3(-0.408248f, -0.408248f, 0.816497f));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3.70711f, -2.29289f, -3), Vector3(-0.408248f, -0.408248f, 0.816497f));
  Plane plane(Vector3(1, 1, 1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -3.70711f, -3), Vector3(-1, -1, -1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 2.29289f, 3), Vector3(-1, -1, -1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -3.70711f, -3), Vector3(1, 1, 1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 2.29289f, 3), Vector3(1, 1, 1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -3.70711f, -3), Vector3(0.707107f, -0.707107f, 0));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 2.29289f, 3), Vector3(0.707107f, -0.707107f, 0));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -3.70711f, -3), Vector3(-0.408248f, -0.408248f, 0.816497f));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 2.29289f, 3), Vector3(-0.408248f, -0.408248f, 0.816497f));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 0, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing forward
void RayPlaneTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -2.70711f, -3), Vector3(-1, -1, -1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing forward
void RayPlaneTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 3.29289f, 3), Vector3(-1, -1, -1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray in front of plane pointing backwards
void RayPlaneTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -2.70711f, -3), Vector3(1, 1, 1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray behind plane pointing backwards
void RayPlaneTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 3.29289f, 3), Vector3(1, 1, 1));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -2.70711f, -3), Vector3(0.707107f, -0.707107f, 0));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 3.29289f, 3), Vector3(0.707107f, -0.707107f, 0));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2.29289f, -2.70711f, -3), Vector3(-0.408248f, -0.408248f, 0.816497f));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}

// Ray parallel to plane
void RayPlaneTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.70711f, 3.29289f, 3), Vector3(-0.408248f, -0.408248f, 0.816497f));
  Plane plane(Vector3(-1, -1, -1), Vector3(0, 1, 0));
  
  TestRayPlane(ray, plane, file);
}


// Ray behind triangle pointing forwards
void RayTriangleTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -1, 0), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind triangle pointing backwards
void RayTriangleTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -1, 0), Vector3(-0, -1, -0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray in front of triangle pointing backwards
void RayTriangleTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 0), Vector3(-0, -1, -0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray in front of triangle pointing forwards
void RayTriangleTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 1, 0), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 1
void RayTriangleTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1.1f, -1, -1.1f), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 2
void RayTriangleTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1.1f, -1, 1.1f), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 3
void RayTriangleTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1.1f, -1, 0), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge12
void RayTriangleTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1.1f, -1, 0), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge23
void RayTriangleTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.22f, -1, 0.44f), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge31
void RayTriangleTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.22f, -1, -0.44f), Vector3(0, 1, 0));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind triangle pointing forwards
void RayTriangleTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 0, 0), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind triangle pointing backwards
void RayTriangleTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 0, 0), Vector3(-1, -0, -0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray in front of triangle pointing backwards
void RayTriangleTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 0, 0), Vector3(-1, -0, -0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray in front of triangle pointing forwards
void RayTriangleTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 0, 0), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 1
void RayTriangleTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 1.1f, -1.1f), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 2
void RayTriangleTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, -1.1f, -1.1f), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 3
void RayTriangleTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 0, 1.1f), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge12
void RayTriangleTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 0, -1.1f), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge23
void RayTriangleTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, -0.44f, 0.22f), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge31
void RayTriangleTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1, 0.44f, 0.22f), Vector3(1, 0, 0));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind triangle pointing forwards
void RayTriangleTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, -1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind triangle pointing backwards
void RayTriangleTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, -1), Vector3(-0, -0, -1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray in front of triangle pointing backwards
void RayTriangleTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 1), Vector3(-0, -0, -1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray in front of triangle pointing forwards
void RayTriangleTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 1
void RayTriangleTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1.1f, 1.1f, -1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 2
void RayTriangleTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1.1f, -1.1f, -1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray behind point 3
void RayTriangleTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-1.1f, 0, -1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge12
void RayTriangleTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1.1f, 0, -1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge23
void RayTriangleTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.22f, -0.44f, -1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}

// Ray outside edge31
void RayTriangleTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.22f, 0.44f, -1), Vector3(0, 0, 1));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestRayTriangle(ray, tri, file);
}


// Ray behind sphere pointing backwards
void RaySphereTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, -1, 0), Vector3(-1, -0, -0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing forwards
void RaySphereTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, -1, 0), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing but not going through the sphere center
void RaySphereTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, -0.333333f, 0), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray at sphere center
void RaySphereTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 0), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing backwards
void RaySphereTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(11, -1, 0), Vector3(-1, -0, -0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing forwards
void RaySphereTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(11, -1, 0), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far behind sphere pointing forwards
void RaySphereTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, -1, 0), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far in front of sphere pointing backwards
void RaySphereTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(11, -1, 0), Vector3(-1, -0, -0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, 0.98f, 0), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, -2.98f, 0), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, -1, 1.98f), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-9, -1, -1.98f), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-29, -1, 1.98f), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(31, -1, 1.98f), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-29, -1, -1.98f), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(31, -1, -1.98f), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray misses sphere
void RaySphereTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, -4), Vector3(1, 0, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing backwards
void RaySphereTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -11, 0), Vector3(-0, -1, -0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing forwards
void RaySphereTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -11, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing but not going through the sphere center
void RaySphereTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -11, -0.666667f), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray at sphere center
void RaySphereTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing backwards
void RaySphereTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 9, 0), Vector3(-0, -1, -0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing forwards
void RaySphereTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 9, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far behind sphere pointing forwards
void RaySphereTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -11, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far in front of sphere pointing backwards
void RaySphereTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 9, 0), Vector3(-0, -1, -0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -11, -1.98f), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -11, 1.98f), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.98f, -11, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.98f, -11, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.98f, -31, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.98f, 29, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.98f, -31, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.98f, 29, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray misses sphere
void RaySphereTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(5, -1, 0), Vector3(0, 1, 0));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing backwards
void RaySphereTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, -10), Vector3(-0, -0, -1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing forwards
void RaySphereTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, -10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing but not going through the sphere center
void RaySphereTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -0.333333f, -10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray at sphere center
void RaySphereTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 0), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing backwards
void RaySphereTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 10), Vector3(-0, -0, -1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing forwards
void RaySphereTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far behind sphere pointing forwards
void RaySphereTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, -10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far in front of sphere pointing backwards
void RaySphereTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 10), Vector3(-0, -0, -1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, 0.98f, -10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -2.98f, -10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.98f, -1, -10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.98f, -1, -10), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.98f, -1, -30), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.98f, -1, 30), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.98f, -1, -30), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.98f, -1, 30), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray misses sphere
void RaySphereTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(5, -1, 0), Vector3(0, 0, 1));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing backwards
void RaySphereTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4.7735f, -6.7735f, -5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing forwards
void RaySphereTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4.7735f, -6.7735f, -5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing but not going through the sphere center
void RaySphereTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-5.24491f, -6.3021f, -5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray at sphere center
void RaySphereTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 0), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing backwards
void RaySphereTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(6.7735f, 4.7735f, 5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing forwards
void RaySphereTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(6.7735f, 4.7735f, 5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far behind sphere pointing forwards
void RaySphereTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4.7735f, -6.7735f, -5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far in front of sphere pointing backwards
void RaySphereTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(6.7735f, 4.7735f, 5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-6.17357f, -5.37343f, -5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3.37343f, -8.17357f, -5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-5.58183f, -7.58183f, -4.15684f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3.96517f, -5.96517f, -7.39017f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-17.1288f, -19.1288f, -15.7038f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(17.5122f, 15.5122f, 18.9372f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-15.5122f, -17.5122f, -18.9372f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(19.1288f, 17.1288f, 15.7038f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray misses sphere
void RaySphereTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.63299f, 0.632993f, -3.26599f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing backwards
void RaySphereTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(6.7735f, 4.7735f, 5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing forwards
void RaySphereTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(6.7735f, 4.7735f, 5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray behind sphere pointing but not going through the sphere center
void RaySphereTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(7.24491f, 4.3021f, 5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray at sphere center
void RaySphereTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(1, -1, 0), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing backwards
void RaySphereTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4.7735f, -6.7735f, -5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray in front of sphere pointing forwards
void RaySphereTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4.7735f, -6.7735f, -5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far behind sphere pointing forwards
void RaySphereTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(6.7735f, 4.7735f, 5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray far in front of sphere pointing backwards
void RaySphereTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4.7735f, -6.7735f, -5.7735f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(8.17357f, 3.37343f, 5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(5.37343f, 6.17357f, 5.7735f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(5.96517f, 3.96517f, 7.39017f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(7.58183f, 5.58183f, 4.15684f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(17.5122f, 15.5122f, 18.9372f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-17.1288f, -19.1288f, -15.7038f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(19.1288f, 17.1288f, 15.7038f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray Tangent to sphere
void RaySphereTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-15.5122f, -17.5122f, -18.9372f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}

// Ray misses sphere
void RaySphereTest85(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2.63299f, 0.632993f, -3.26599f), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Sphere sphere(Vector3(1, -1, 0), 2);
  
  TestRaySphere(ray, sphere, file);
}


// Ray behind aabb pointing backwards
void RayAabbTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 0, 0), Vector3(-1, -0, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 0, 0), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray at aabb center
void RayAabbTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 0), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 0, 0), Vector3(-1, -0, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 0, 0), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 0, 0), Vector3(-1, -0, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 0, 0), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, 0, 0), Vector3(-1, -0, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, 0, 0), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 2, 0), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, 0), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 0, 2), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 0, -2), Vector3(1, 0, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -2, 0), Vector3(-0, -1, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -2, 0), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray at aabb center
void RayAabbTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 0), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 2, 0), Vector3(-0, -1, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 2, 0), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 4, 0), Vector3(-0, -1, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 4, 0), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -4, 0), Vector3(-0, -1, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -4, 0), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -2, -2), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -2, 2), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, 0), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, -2, 0), Vector3(0, 1, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, -2), Vector3(-0, -0, -1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, -2), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray at aabb center
void RayAabbTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 0), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 2), Vector3(-0, -0, -1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 2), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 4), Vector3(-0, -0, -1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 4), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, -4), Vector3(-0, -0, -1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, -4), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 2, -2), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -2, -2), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 0, -2), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 0, -2), Vector3(0, 0, 1));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, 0), Vector3(-0.707107f, -0.707107f, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, 0), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray at aabb center
void RayAabbTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 0), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 2, 0), Vector3(-0.707107f, -0.707107f, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 2, 0), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 4, 0), Vector3(-0.707107f, -0.707107f, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 4, 0), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, -4, 0), Vector3(-0.707107f, -0.707107f, -0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, -4, 0), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, 3.57628e-07f, 0), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.57628e-07f, -4, 0), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, 2), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, -2), Vector3(0.707107f, 0.707107f, 0));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 0, -2), Vector3(-0.707107f, -0, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 0, -2), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray at aabb center
void RayAabbTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 0), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 0, 2), Vector3(-0.707107f, -0, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 0, 2), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 0, 4), Vector3(-0.707107f, -0, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 0, 4), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, 0, -4), Vector3(-0.707107f, -0, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, 0, -4), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, 2, -2), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, -2), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, 0, 3.57628e-07f), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(3.57628e-07f, 0, -4), Vector3(0.707107f, 0, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -2, -2), Vector3(-0, -0.707107f, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -2, -2), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray at aabb center
void RayAabbTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 0), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 2, 2), Vector3(-0, -0.707107f, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 2, 2), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 4, 4), Vector3(-0, -0.707107f, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 4, 4), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -4, -4), Vector3(-0, -0.707107f, -0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -4, -4), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 3.57628e-07f, -4), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -4, 3.57628e-07f), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, -2), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, -2, -2), Vector3(0, 0.707107f, 0.707107f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, -2), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-2, -2, -2), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray at aabb center
void RayAabbTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, 0, 0), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 2, 2), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(2, 2, 2), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing backwards
void RayAabbTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 4, 4), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray in front of aabb pointing forwards
void RayAabbTest85(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(4, 4, 4), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing backwards
void RayAabbTest86(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, -4, -4), Vector3(-0.57735f, -0.57735f, -0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Ray behind aabb pointing forwards
void RayAabbTest87(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, -4, -4), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest88(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-4, 0, -2), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest89(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(0, -4, -2), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest90(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-3.33333f, -3.33333f, 0.666667f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

void RayAabbTest91(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Ray ray(Vector3(-0.666667f, -0.666667f, -4.66667f), Vector3(0.57735f, 0.57735f, 0.57735f));
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestRayAabb(ray, aabb, file);
}

// Plane of edge edge10. Triangle in front of plane
void PlaneTriangleTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0, -0.707107f), Vector3(1.35355f, -0, 1.35355f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0, -0.707107f), Vector3(0.681802f, -0, 0.681802f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. In the middle of the triangle
void PlaneTriangleTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0, -0.707107f), Vector3(0.5f, -0, 0.5f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0, -0.707107f), Vector3(-0.181802f, 0, -0.181802f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle behind the plane
void PlaneTriangleTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0, -0.707107f), Vector3(-0.853553f, 0, -0.853553f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -1, 0), Vector3(0.25f, -0.45f, 0.25f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -1, 0), Vector3(0.25f, 0.45f, 0.25f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle in front of plane
void PlaneTriangleTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0, -0.316228f), Vector3(-1.37434f, 0, 0.458114f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0, -0.316228f), Vector3(-0.473093f, 0, 0.157698f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. In the middle of the triangle
void PlaneTriangleTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0, -0.316228f), Vector3(0.3f, 0, -0.1f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0, -0.316228f), Vector3(0.773093f, 0, -0.257698f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle behind the plane
void PlaneTriangleTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0, -0.316228f), Vector3(1.67434f, 0, -0.558114f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -1, 0), Vector3(0.15f, -0.45f, -0.05f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -1, 0), Vector3(0.15f, 0.45f, -0.05f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle in front of plane
void PlaneTriangleTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, 0, 0.992278f), Vector3(0.200479f, -0, -1.60383f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, 0, 0.992278f), Vector3(0.0826459f, -0, -0.661167f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. In the middle of the triangle
void PlaneTriangleTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, 0, 0.992278f), Vector3(0.0307692f, -0, -0.246154f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, 0, 0.992278f), Vector3(-0.0518767f, 0, 0.415013f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle behind the plane
void PlaneTriangleTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, 0, 0.992278f), Vector3(-0.16971f, 0, 1.35768f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -1, 0), Vector3(0.0153846f, -0.45f, -0.123077f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -1, 0), Vector3(0.0153846f, 0.45f, -0.123077f));
  Triangle tri(Vector3(1, 0, -1), Vector3(1, 0, 1), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle in front of plane
void PlaneTriangleTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(-0, -1.35355f, -1.35355f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(-0, -0.681802f, -0.681802f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. In the middle of the triangle
void PlaneTriangleTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(-0, -0.5f, -0.5f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0.181802f, 0.181802f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle behind the plane
void PlaneTriangleTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0.853553f, 0.853553f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-1, 0, 0), Vector3(-0.45f, -0.25f, -0.25f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-1, 0, 0), Vector3(0.45f, -0.25f, -0.25f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle in front of plane
void PlaneTriangleTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.316228f, -0.948683f), Vector3(0, -0.458114f, 1.37434f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.316228f, -0.948683f), Vector3(0, -0.157698f, 0.473093f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. In the middle of the triangle
void PlaneTriangleTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.316228f, -0.948683f), Vector3(0, 0.1f, -0.3f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.316228f, -0.948683f), Vector3(0, 0.257698f, -0.773093f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle behind the plane
void PlaneTriangleTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, 0.316228f, -0.948683f), Vector3(0, 0.558114f, -1.67434f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-1, 0, 0), Vector3(-0.45f, 0.05f, -0.15f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-1, 0, 0), Vector3(0.45f, 0.05f, -0.15f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle in front of plane
void PlaneTriangleTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0.992278f, 0.124035f), Vector3(-0, 1.60383f, -0.200479f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0.992278f, 0.124035f), Vector3(-0, 0.661167f, -0.0826459f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. In the middle of the triangle
void PlaneTriangleTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0.992278f, 0.124035f), Vector3(-0, 0.246154f, -0.0307692f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0.992278f, 0.124035f), Vector3(0, -0.415013f, 0.0518767f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle behind the plane
void PlaneTriangleTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0.992278f, 0.124035f), Vector3(0, -1.35768f, 0.16971f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-1, 0, 0), Vector3(-0.45f, 0.123077f, -0.0153846f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-1, 0, 0), Vector3(0.45f, 0.123077f, -0.0153846f));
  Triangle tri(Vector3(0, 1, -1), Vector3(0, -1, -1), Vector3(0, 0, 1));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle in front of plane
void PlaneTriangleTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0.707107f, 0), Vector3(1.35355f, -1.35355f, -0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0.707107f, 0), Vector3(0.681802f, -0.681802f, -0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. In the middle of the triangle
void PlaneTriangleTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0.707107f, 0), Vector3(0.5f, -0.5f, -0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0.707107f, 0), Vector3(-0.181802f, 0.181802f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge10. Triangle behind the plane
void PlaneTriangleTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.707107f, 0.707107f, 0), Vector3(-0.853553f, 0.853553f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0, -1), Vector3(0.25f, -0.25f, -0.45f));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0, -1), Vector3(0.25f, -0.25f, 0.45f));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle in front of plane
void PlaneTriangleTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0.316228f, 0), Vector3(-1.37434f, -0.458114f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0.316228f, 0), Vector3(-0.473093f, -0.157698f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. In the middle of the triangle
void PlaneTriangleTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0.316228f, 0), Vector3(0.3f, 0.1f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0.316228f, 0), Vector3(0.773093f, 0.257698f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge21. Triangle behind the plane
void PlaneTriangleTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0.948683f, 0.316228f, 0), Vector3(1.67434f, 0.558114f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0, -1), Vector3(0.15f, 0.05f, -0.45f));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0, -1), Vector3(0.15f, 0.05f, 0.45f));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle in front of plane
void PlaneTriangleTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, -0.992278f, 0), Vector3(0.200479f, 1.60383f, -0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle in front of plane (one point within epsilon behind)
void PlaneTriangleTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, -0.992278f, 0), Vector3(0.0826459f, 0.661167f, -0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. In the middle of the triangle
void PlaneTriangleTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, -0.992278f, 0), Vector3(0.0307692f, 0.246154f, -0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle behind the plane (one point within epsilon in front)
void PlaneTriangleTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, -0.992278f, 0), Vector3(-0.0518767f, -0.415013f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Plane of edge edge02. Triangle behind the plane
void PlaneTriangleTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(-0.124035f, -0.992278f, 0), Vector3(-0.16971f, -1.35768f, 0));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle in front but coplanar
void PlaneTriangleTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0, -1), Vector3(0.0153846f, 0.123077f, -0.45f));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}

// Triangle behind but coplanar
void PlaneTriangleTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  float epsilon = 0.5f;
  Plane plane(Vector3(0, -0, -1), Vector3(0.0153846f, 0.123077f, 0.45f));
  Triangle tri(Vector3(1, 1, 0), Vector3(1, -1, 0), Vector3(-1, 0, 0));
  
  TestPlaneTriangle(plane, tri, epsilon, file);
}


// Sphere in front of plane
void PlaneSphereTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(2, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the front)
void PlaneSphereTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(1, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere centered at plane
void PlaneSphereTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the back)
void PlaneSphereTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(-1, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere behind plane
void PlaneSphereTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(-2, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere in front of plane
void PlaneSphereTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 2, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the front)
void PlaneSphereTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 1, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere centered at plane
void PlaneSphereTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the back)
void PlaneSphereTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, -1, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere behind plane
void PlaneSphereTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, -2, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere in front of plane
void PlaneSphereTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, 2), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the front)
void PlaneSphereTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, 1), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere centered at plane
void PlaneSphereTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the back)
void PlaneSphereTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, -1), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere behind plane
void PlaneSphereTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, -2), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere in front of plane
void PlaneSphereTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Sphere sphere(Vector3(1.1547f, 1.1547f, 1.1547f), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the front)
void PlaneSphereTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0.57735f, 0.57735f, 0.57735f), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere centered at plane
void PlaneSphereTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Sphere sphere(Vector3(0, 0, 0), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere just touching plane (on the back)
void PlaneSphereTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Sphere sphere(Vector3(-0.57735f, -0.57735f, -0.57735f), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Sphere behind plane
void PlaneSphereTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Sphere sphere(Vector3(-1.1547f, -1.1547f, -1.1547f), 1);
  
  TestPlaneSphere(plane, sphere, file);
}

// Aabb in front of plane
void PlaneAabbTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, -0.5f, -0.5f), Vector3(1.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, -0.5f, -0.5f), Vector3(1.05f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.5f, -0.5f), Vector3(0.95f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.5f, -0.5f), Vector3(0.875f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.5f, -0.5f), Vector3(0.125f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.5f, -0.5f), Vector3(0.05f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, -0.5f, -0.5f), Vector3(-0.05f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, -0.5f, -0.5f), Vector3(-0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, 0.5f, -0.5f), Vector3(0.5f, 1.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, 0.05f, -0.5f), Vector3(0.5f, 1.05f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.05f, -0.5f), Vector3(0.5f, 0.95f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.125f, -0.5f), Vector3(0.5f, 0.875f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.875f, -0.5f), Vector3(0.5f, 0.125f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.95f, -0.5f), Vector3(0.5f, 0.05f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -1.05f, -0.5f), Vector3(0.5f, -0.05f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -1.5f, -0.5f), Vector3(0.5f, -0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, 0.5f), Vector3(0.5f, 0.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, 0.05f), Vector3(0.5f, 0.5f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.05f), Vector3(0.5f, 0.5f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.125f), Vector3(0.5f, 0.5f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.875f), Vector3(0.5f, 0.5f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.95f), Vector3(0.5f, 0.5f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -1.05f), Vector3(0.5f, 0.5f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -1.5f), Vector3(0.5f, 0.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, 0.5f, -0.5f), Vector3(1.5f, 1.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, 0.05f, -0.5f), Vector3(1.05f, 1.05f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.05f, -0.5f), Vector3(0.95f, 0.95f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.125f, -0.5f), Vector3(0.875f, 0.875f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.875f, -0.5f), Vector3(0.125f, 0.125f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.95f, -0.5f), Vector3(0.05f, 0.05f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, -1.05f, -0.5f), Vector3(-0.05f, -0.05f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, -1.5f, -0.5f), Vector3(-0.5f, -0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, -0.5f, 0.5f), Vector3(1.5f, 0.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, -0.5f, 0.05f), Vector3(1.05f, 0.5f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.5f, -0.05f), Vector3(0.95f, 0.5f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.5f, -0.125f), Vector3(0.875f, 0.5f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.5f, -0.875f), Vector3(0.125f, 0.5f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.5f, -0.95f), Vector3(0.05f, 0.5f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, -0.5f, -1.05f), Vector3(-0.05f, 0.5f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, -0.5f, -1.5f), Vector3(-0.5f, 0.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, 0.5f, 0.5f), Vector3(0.5f, 1.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, 0.05f, 0.05f), Vector3(0.5f, 1.05f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.05f, -0.05f), Vector3(0.5f, 0.95f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.125f, -0.125f), Vector3(0.5f, 0.875f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.875f, -0.875f), Vector3(0.5f, 0.125f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.95f, -0.95f), Vector3(0.5f, 0.05f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -1.05f, -1.05f), Vector3(0.5f, -0.05f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -1.5f, -1.5f), Vector3(0.5f, -0.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, 0.05f, 0.05f), Vector3(1.05f, 1.05f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.05f, -0.05f), Vector3(0.95f, 0.95f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.125f, -0.125f), Vector3(0.875f, 0.875f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.875f, -0.875f), Vector3(0.125f, 0.125f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.95f, -0.95f), Vector3(0.05f, 0.05f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, -1.05f, -1.05f), Vector3(-0.05f, -0.05f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, -1.5f, -1.5f), Vector3(-0.5f, -0.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, 0.5f, -1.5f), Vector3(1.5f, 1.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, 0.05f, -1.05f), Vector3(1.05f, 1.05f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.05f, -0.95f), Vector3(0.95f, 0.95f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.125f, -0.875f), Vector3(0.875f, 0.875f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.875f, -0.125f), Vector3(0.125f, 0.125f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.95f, -0.05f), Vector3(0.05f, 0.05f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, -1.05f, 0.05f), Vector3(-0.05f, -0.05f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, -1.5f, 0.5f), Vector3(-0.5f, -0.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, -1.5f, 0.5f), Vector3(1.5f, -0.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, -1.05f, 0.05f), Vector3(1.05f, -0.05f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.95f, -0.05f), Vector3(0.95f, 0.05f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.875f, -0.125f), Vector3(0.875f, 0.125f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.125f, -0.875f), Vector3(0.125f, 0.875f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.05f, -0.95f), Vector3(0.05f, 0.95f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, 0.05f, -1.05f), Vector3(-0.05f, 1.05f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, 0.5f, -1.5f), Vector3(-0.5f, 1.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, -1.5f, -1.5f), Vector3(1.5f, -0.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, -1.05f, -1.05f), Vector3(1.05f, -0.05f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.95f, -0.95f), Vector3(0.95f, 0.05f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest85(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.875f, -0.875f), Vector3(0.875f, 0.125f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest86(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest87(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.125f, -0.125f), Vector3(0.125f, 0.875f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest88(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.05f, -0.05f), Vector3(0.05f, 0.95f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest89(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, 0.05f, 0.05f), Vector3(-0.05f, 1.05f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest90(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, 0.5f, 0.5f), Vector3(-0.5f, 1.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest91(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, -1.5f, 0.5f), Vector3(-0.5f, -0.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest92(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, -1.05f, 0.05f), Vector3(-0.05f, -0.05f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest93(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.95f, -0.05f), Vector3(0.05f, 0.05f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest94(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.875f, -0.125f), Vector3(0.125f, 0.125f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest95(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest96(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.125f, -0.875f), Vector3(0.875f, 0.875f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest97(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.05f, -0.95f), Vector3(0.95f, 0.95f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest98(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, 0.05f, -1.05f), Vector3(1.05f, 1.05f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest99(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, 0.5f, -1.5f), Vector3(1.5f, 1.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest100(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.5f, -1.5f, -1.5f), Vector3(-0.5f, -0.5f, -0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest101(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-1.05f, -1.05f, -1.05f), Vector3(-0.05f, -0.05f, -0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest102(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.95f, -0.95f, -0.95f), Vector3(0.05f, 0.05f, 0.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest103(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.875f, -0.875f, -0.875f), Vector3(0.125f, 0.125f, 0.125f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest104(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest105(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.125f, -0.125f, -0.125f), Vector3(0.875f, 0.875f, 0.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest106(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(-0.05f, -0.05f, -0.05f), Vector3(0.95f, 0.95f, 0.95f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest107(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.05f, 0.05f, 0.05f), Vector3(1.05f, 1.05f, 1.05f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest108(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(0, 0, 0));
  Aabb aabb(Vector3(0.5f, 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest109(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.5f, 1.5f, 2.9f), Vector3(2.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest110(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.05f, 1.5f, 2.9f), Vector3(2.05f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest111(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.95f, 1.5f, 2.9f), Vector3(1.95f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest112(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.875f, 1.5f, 2.9f), Vector3(1.875f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest113(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest114(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.125f, 1.5f, 2.9f), Vector3(1.125f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest115(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.05f, 1.5f, 2.9f), Vector3(1.05f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest116(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.05f, 1.5f, 2.9f), Vector3(0.95f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest117(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(1, 0, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.5f, 1.5f, 2.9f), Vector3(0.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest118(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 2.5f, 2.9f), Vector3(1.5f, 3.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest119(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 2.05f, 2.9f), Vector3(1.5f, 3.05f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest120(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.95f, 2.9f), Vector3(1.5f, 2.95f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest121(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.875f, 2.9f), Vector3(1.5f, 2.875f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest122(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest123(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.125f, 2.9f), Vector3(1.5f, 2.125f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest124(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.05f, 2.9f), Vector3(1.5f, 2.05f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest125(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 0.95f, 2.9f), Vector3(1.5f, 1.95f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest126(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 1, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 0.5f, 2.9f), Vector3(1.5f, 1.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest127(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 3.1f), Vector3(1.5f, 2.5f, 3.3f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest128(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 3.01f), Vector3(1.5f, 2.5f, 3.21f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest129(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.99f), Vector3(1.5f, 2.5f, 3.19f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest130(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.975f), Vector3(1.5f, 2.5f, 3.175f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest131(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest132(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.825f), Vector3(1.5f, 2.5f, 3.025f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest133(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.81f), Vector3(1.5f, 2.5f, 3.01f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest134(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.79f), Vector3(1.5f, 2.5f, 2.99f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest135(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0, 1), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.7f), Vector3(1.5f, 2.5f, 2.9f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest136(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.5f, 2.5f, 2.9f), Vector3(2.5f, 3.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest137(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.05f, 2.05f, 2.9f), Vector3(2.05f, 3.05f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest138(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.95f, 1.95f, 2.9f), Vector3(1.95f, 2.95f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest139(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.875f, 1.875f, 2.9f), Vector3(1.875f, 2.875f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest140(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest141(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.125f, 1.125f, 2.9f), Vector3(1.125f, 2.125f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest142(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.0500001f, 1.05f, 2.9f), Vector3(1.05f, 2.05f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest143(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.05f, 0.95f, 2.9f), Vector3(0.95f, 1.95f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest144(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0.707107f, 0), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.5f, 0.5f, 2.9f), Vector3(0.5f, 1.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest145(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.1f, 1.5f, 3.5f), Vector3(2.1f, 2.5f, 3.7f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest146(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.83f, 1.5f, 3.23f), Vector3(1.83f, 2.5f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest147(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.77f, 1.5f, 3.17f), Vector3(1.77f, 2.5f, 3.37f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest148(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.725f, 1.5f, 3.125f), Vector3(1.725f, 2.5f, 3.325f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest149(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest150(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.275f, 1.5f, 2.675f), Vector3(1.275f, 2.5f, 2.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest151(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.23f, 1.5f, 2.63f), Vector3(1.23f, 2.5f, 2.83f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest152(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.17f, 1.5f, 2.57f), Vector3(1.17f, 2.5f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest153(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.707107f, 0, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.1f, 1.5f, 2.3f), Vector3(0.9f, 2.5f, 2.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest154(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 2.1f, 3.5f), Vector3(1.5f, 3.1f, 3.7f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest155(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.83f, 3.23f), Vector3(1.5f, 2.83f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest156(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.77f, 3.17f), Vector3(1.5f, 2.77f, 3.37f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest157(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.725f, 3.125f), Vector3(1.5f, 2.725f, 3.325f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest158(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest159(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.275f, 2.675f), Vector3(1.5f, 2.275f, 2.875f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest160(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.23f, 2.63f), Vector3(1.5f, 2.23f, 2.83f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest161(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.17f, 2.57f), Vector3(1.5f, 2.17f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest162(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0, 0.707107f, 0.707107f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 0.9f, 2.3f), Vector3(1.5f, 1.9f, 2.5f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest163(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.23333f, 2.23333f, 3.63333f), Vector3(2.23333f, 3.23333f, 3.83333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest164(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.903333f, 1.90333f, 3.30333f), Vector3(1.90333f, 2.90333f, 3.50333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest165(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.83f, 1.83f, 3.23f), Vector3(1.83f, 2.83f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest166(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.775f, 1.775f, 3.175f), Vector3(1.775f, 2.775f, 3.375f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest167(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest168(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.225f, 1.225f, 2.625f), Vector3(1.225f, 2.225f, 2.825f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest169(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.17f, 1.17f, 2.57f), Vector3(1.17f, 2.17f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest170(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.0966667f, 1.09667f, 2.49667f), Vector3(1.09667f, 2.09667f, 2.69667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest171(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.233333f, 0.766667f, 2.16667f), Vector3(0.766667f, 1.76667f, 2.36667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest172(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.23333f, 2.23333f, 2.16667f), Vector3(2.23333f, 3.23333f, 2.36667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest173(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.903333f, 1.90333f, 2.49667f), Vector3(1.90333f, 2.90333f, 2.69667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest174(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.83f, 1.83f, 2.57f), Vector3(1.83f, 2.83f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest175(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.775f, 1.775f, 2.625f), Vector3(1.775f, 2.775f, 2.825f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest176(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest177(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.225f, 1.225f, 3.175f), Vector3(1.225f, 2.225f, 3.375f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest178(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.17f, 1.17f, 3.23f), Vector3(1.17f, 2.17f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest179(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.0966667f, 1.09667f, 3.30333f), Vector3(1.09667f, 2.09667f, 3.50333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest180(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, 0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.233333f, 0.766667f, 3.63333f), Vector3(0.766667f, 1.76667f, 3.83333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest181(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.23333f, 0.766667f, 3.63333f), Vector3(2.23333f, 1.76667f, 3.83333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest182(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.903333f, 1.09667f, 3.30333f), Vector3(1.90333f, 2.09667f, 3.50333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest183(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.83f, 1.17f, 3.23f), Vector3(1.83f, 2.17f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest184(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.775f, 1.225f, 3.175f), Vector3(1.775f, 2.225f, 3.375f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest185(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest186(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.225f, 1.775f, 2.625f), Vector3(1.225f, 2.775f, 2.825f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest187(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.17f, 1.83f, 2.57f), Vector3(1.17f, 2.83f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest188(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.0966667f, 1.90333f, 2.49667f), Vector3(1.09667f, 2.90333f, 2.69667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest189(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.233333f, 2.23333f, 2.16667f), Vector3(0.766667f, 3.23333f, 2.36667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest190(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.23333f, 0.766667f, 2.16667f), Vector3(2.23333f, 1.76667f, 2.36667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest191(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.903333f, 1.09667f, 2.49667f), Vector3(1.90333f, 2.09667f, 2.69667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest192(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.83f, 1.17f, 2.57f), Vector3(1.83f, 2.17f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest193(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.775f, 1.225f, 2.625f), Vector3(1.775f, 2.225f, 2.825f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest194(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest195(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.225f, 1.775f, 3.175f), Vector3(1.225f, 2.775f, 3.375f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest196(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.17f, 1.83f, 3.23f), Vector3(1.17f, 2.83f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest197(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.0966667f, 1.90333f, 3.30333f), Vector3(1.09667f, 2.90333f, 3.50333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest198(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.233333f, 2.23333f, 3.63333f), Vector3(0.766667f, 3.23333f, 3.83333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest199(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.233333f, 0.766667f, 3.63333f), Vector3(0.766667f, 1.76667f, 3.83333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest200(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.0966667f, 1.09667f, 3.30333f), Vector3(1.09667f, 2.09667f, 3.50333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest201(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.17f, 1.17f, 3.23f), Vector3(1.17f, 2.17f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest202(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.225f, 1.225f, 3.175f), Vector3(1.225f, 2.225f, 3.375f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest203(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest204(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.775f, 1.775f, 2.625f), Vector3(1.775f, 2.775f, 2.825f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest205(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.83f, 1.83f, 2.57f), Vector3(1.83f, 2.83f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest206(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.903333f, 1.90333f, 2.49667f), Vector3(1.90333f, 2.90333f, 2.69667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest207(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, 0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.23333f, 2.23333f, 2.16667f), Vector3(2.23333f, 3.23333f, 2.36667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb in front of plane
void PlaneAabbTest208(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(-0.233333f, 0.766667f, 2.16667f), Vector3(0.766667f, 1.76667f, 2.36667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely in front of plane
void PlaneAabbTest209(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.0966667f, 1.09667f, 2.49667f), Vector3(1.09667f, 2.09667f, 2.69667f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb on front)
void PlaneAabbTest210(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.17f, 1.17f, 2.57f), Vector3(1.17f, 2.17f, 2.77f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb slightly touching plane (aabb on front)
void PlaneAabbTest211(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.225f, 1.225f, 2.625f), Vector3(1.225f, 2.225f, 2.825f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb centered on plane
void PlaneAabbTest212(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.5f, 1.5f, 2.9f), Vector3(1.5f, 2.5f, 3.1f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb signtly touching plane (aabb in back)
void PlaneAabbTest213(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.775f, 1.775f, 3.175f), Vector3(1.775f, 2.775f, 3.375f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely touching plane (aabb in back)
void PlaneAabbTest214(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.83f, 1.83f, 3.23f), Vector3(1.83f, 2.83f, 3.43f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb just barely behind plane
void PlaneAabbTest215(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(0.903333f, 1.90333f, 3.30333f), Vector3(1.90333f, 2.90333f, 3.50333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Aabb behind plane
void PlaneAabbTest216(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Plane plane(Vector3(-0.57735f, -0.57735f, -0.57735f), Vector3(1, 2, 3));
  Aabb aabb(Vector3(1.23333f, 2.23333f, 3.63333f), Vector3(2.23333f, 3.23333f, 3.83333f));
  
  TestPlaneAabb(plane, aabb, file);
}

// Triangle inside of frustum.
void FrustumTriangleTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 1, 0), Vector3(0.866025f, -0.5f, 0), Vector3(-0.866025f, -0.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(5.33154f, 0, -1), Vector3(6.19756f, 0, 0.5f), Vector3(4.46551f, 0, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(7.33154f, 0, -1), Vector3(8.19756f, 0, 0.5f), Vector3(6.46551f, 0, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-5.33154f, 0, 1), Vector3(-4.46551f, 0, -0.5f), Vector3(-6.19756f, 0, -0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-7.33154f, 0, 1), Vector3(-6.46551f, 0, -0.5f), Vector3(-8.19756f, 0, -0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 6.33154f, 0), Vector3(0.866025f, 4.83154f, 0), Vector3(-0.866025f, 4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 8.33154f, 0), Vector3(0.866025f, 6.83154f, 0), Vector3(-0.866025f, 6.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -6.33154f, 0), Vector3(0.866025f, -4.83154f, 0), Vector3(-0.866025f, -4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -8.33154f, 0), Vector3(0.866025f, -6.83154f, 0), Vector3(-0.866025f, -6.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 1, 5), Vector3(0, -0.5f, 4.13397f), Vector3(0, -0.5f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 1, 7), Vector3(0, -0.5f, 6.13397f), Vector3(0, -0.5f, 7.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -1, -5), Vector3(0, 0.5f, -5.86603f), Vector3(0, 0.5f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -1, -7), Vector3(0, 0.5f, -7.86603f), Vector3(0, 0.5f, -6.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(5.33154f, 15.9946f, -1), Vector3(6.19756f, 15.9946f, 0.5f), Vector3(4.46551f, 15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(5.33154f, 0, 14), Vector3(6.19756f, 0, 15.5f), Vector3(4.46551f, 0, 15.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(5.33154f, -15.9946f, -1), Vector3(6.19756f, -15.9946f, 0.5f), Vector3(4.46551f, -15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(5.33154f, 0, -16), Vector3(6.19756f, 0, -14.5f), Vector3(4.46551f, 0, -14.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-5.33154f, 15.9946f, -1), Vector3(-4.46551f, 15.9946f, 0.5f), Vector3(-6.19756f, 15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-5.33154f, 0, 14), Vector3(-4.46551f, 0, 15.5f), Vector3(-6.19756f, 0, 15.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-5.33154f, -15.9946f, -1), Vector3(-4.46551f, -15.9946f, 0.5f), Vector3(-6.19756f, -15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-5.33154f, 0, -16), Vector3(-4.46551f, 0, -14.5f), Vector3(-6.19756f, 0, -14.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 6.33154f, 15), Vector3(0.866025f, 4.83154f, 15), Vector3(-0.866025f, 4.83154f, 15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(15.9946f, 6.33154f, 0), Vector3(16.8606f, 4.83154f, 0), Vector3(15.1286f, 4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 6.33154f, -15), Vector3(0.866025f, 4.83154f, -15), Vector3(-0.866025f, 4.83154f, -15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-15.9946f, 6.33154f, 0), Vector3(-15.1286f, 4.83154f, 0), Vector3(-16.8606f, 4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -4.33154f, 15), Vector3(0.866025f, -5.83154f, 15), Vector3(-0.866025f, -5.83154f, 15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(15.9946f, -4.33154f, 0), Vector3(16.8606f, -5.83154f, 0), Vector3(15.1286f, -5.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -4.33154f, -15), Vector3(0.866025f, -5.83154f, -15), Vector3(-0.866025f, -5.83154f, -15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-15.9946f, -4.33154f, 0), Vector3(-15.1286f, -5.83154f, 0), Vector3(-16.8606f, -5.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(15.9946f, 1, 5), Vector3(15.9946f, -0.5f, 4.13397f), Vector3(15.9946f, -0.5f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 16.9946f, 5), Vector3(0, 15.4946f, 4.13397f), Vector3(0, 15.4946f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-15.9946f, 1, 5), Vector3(-15.9946f, -0.5f, 4.13397f), Vector3(-15.9946f, -0.5f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -14.9946f, 5), Vector3(0, -16.4946f, 4.13397f), Vector3(0, -16.4946f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(15.9946f, 1, -5), Vector3(15.9946f, -0.5f, -5.86603f), Vector3(15.9946f, -0.5f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, 16.9946f, -5), Vector3(0, 15.4946f, -5.86603f), Vector3(0, 15.4946f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(-15.9946f, 1, -5), Vector3(-15.9946f, -0.5f, -5.86603f), Vector3(-15.9946f, -0.5f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Triangle triangle(Vector3(0, -14.9946f, -5), Vector3(0, -16.4946f, -5.86603f), Vector3(0, -16.4946f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

// Triangle inside of frustum.
void FrustumTriangleTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 1, 0), Vector3(0, -0.5f, -0.866025f), Vector3(0, -0.5f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 6.33154f, 0), Vector3(0.866025f, 4.83154f, 0), Vector3(-0.866025f, 4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 8.33154f, 0), Vector3(0.866025f, 6.83154f, 0), Vector3(-0.866025f, 6.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -6.33154f, 0), Vector3(0.866025f, -4.83154f, 0), Vector3(-0.866025f, -4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -8.33154f, 0), Vector3(0.866025f, -6.83154f, 0), Vector3(-0.866025f, -6.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 1, 5.33154f), Vector3(0, -0.5f, 4.46551f), Vector3(0, -0.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 1, 7.33154f), Vector3(0, -0.5f, 6.46551f), Vector3(0, -0.5f, 8.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -1, -5.33154f), Vector3(0, 0.5f, -6.19756f), Vector3(0, 0.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -1, -7.33154f), Vector3(0, 0.5f, -8.19756f), Vector3(0, 0.5f, -6.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(5, 0, -1), Vector3(5.86603f, 0, 0.5f), Vector3(4.13397f, 0, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(7, 0, -1), Vector3(7.86603f, 0, 0.5f), Vector3(6.13397f, 0, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-5, 0, 1), Vector3(-4.13397f, 0, -0.5f), Vector3(-5.86603f, 0, -0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-7, 0, 1), Vector3(-6.13397f, 0, -0.5f), Vector3(-7.86603f, 0, -0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 6.33154f, 15.9946f), Vector3(0.866025f, 4.83154f, 15.9946f), Vector3(-0.866025f, 4.83154f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(15, 6.33154f, 0), Vector3(15.866f, 4.83154f, 0), Vector3(14.134f, 4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 6.33154f, -15.9946f), Vector3(0.866025f, 4.83154f, -15.9946f), Vector3(-0.866025f, 4.83154f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-15, 6.33154f, 0), Vector3(-14.134f, 4.83154f, 0), Vector3(-15.866f, 4.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -4.33154f, 15.9946f), Vector3(0.866025f, -5.83154f, 15.9946f), Vector3(-0.866025f, -5.83154f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(15, -4.33154f, 0), Vector3(15.866f, -5.83154f, 0), Vector3(14.134f, -5.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -4.33154f, -15.9946f), Vector3(0.866025f, -5.83154f, -15.9946f), Vector3(-0.866025f, -5.83154f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-15, -4.33154f, 0), Vector3(-14.134f, -5.83154f, 0), Vector3(-15.866f, -5.83154f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(15, 1, 5.33154f), Vector3(15, -0.5f, 4.46551f), Vector3(15, -0.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 16.9946f, 5.33154f), Vector3(0, 15.4946f, 4.46551f), Vector3(0, 15.4946f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-15, 1, 5.33154f), Vector3(-15, -0.5f, 4.46551f), Vector3(-15, -0.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -14.9946f, 5.33154f), Vector3(0, -16.4946f, 4.46551f), Vector3(0, -16.4946f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(15, 1, -5.33154f), Vector3(15, -0.5f, -6.19756f), Vector3(15, -0.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, 16.9946f, -5.33154f), Vector3(0, 15.4946f, -6.19756f), Vector3(0, 15.4946f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-15, 1, -5.33154f), Vector3(-15, -0.5f, -6.19756f), Vector3(-15, -0.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(0, -14.9946f, -5.33154f), Vector3(0, -16.4946f, -6.19756f), Vector3(0, -16.4946f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(5, 15.9946f, -1), Vector3(5.86603f, 15.9946f, 0.5f), Vector3(4.13397f, 15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(5, 0, 14.9946f), Vector3(5.86603f, 0, 16.4946f), Vector3(4.13397f, 0, 16.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(5, -15.9946f, -1), Vector3(5.86603f, -15.9946f, 0.5f), Vector3(4.13397f, -15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(5, 0, -16.9946f), Vector3(5.86603f, 0, -15.4946f), Vector3(4.13397f, 0, -15.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-5, 15.9946f, -1), Vector3(-4.13397f, 15.9946f, 0.5f), Vector3(-5.86603f, 15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-5, 0, 14.9946f), Vector3(-4.13397f, 0, 16.4946f), Vector3(-5.86603f, 0, 16.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-5, -15.9946f, -1), Vector3(-4.13397f, -15.9946f, 0.5f), Vector3(-5.86603f, -15.9946f, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Triangle triangle(Vector3(-5, 0, -16.9946f), Vector3(-4.13397f, 0, -15.4946f), Vector3(-5.86603f, 0, -15.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

// Triangle inside of frustum.
void FrustumTriangleTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 0, -1), Vector3(0.866025f, 0, 0.5f), Vector3(-0.866025f, 0, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 1, 5.33154f), Vector3(0, -0.5f, 4.46551f), Vector3(0, -0.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 1, 7.33154f), Vector3(0, -0.5f, 6.46551f), Vector3(0, -0.5f, 8.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -1, -5.33154f), Vector3(0, 0.5f, -6.19756f), Vector3(0, 0.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -1, -7.33154f), Vector3(0, 0.5f, -8.19756f), Vector3(0, 0.5f, -6.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(5.33154f, 0, -1), Vector3(6.19756f, 0, 0.5f), Vector3(4.46551f, 0, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(7.33154f, 0, -1), Vector3(8.19756f, 0, 0.5f), Vector3(6.46551f, 0, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-5.33154f, 0, 1), Vector3(-4.46551f, 0, -0.5f), Vector3(-6.19756f, 0, -0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-7.33154f, 0, 1), Vector3(-6.46551f, 0, -0.5f), Vector3(-8.19756f, 0, -0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 6, 0), Vector3(0.866025f, 4.5f, 0), Vector3(-0.866025f, 4.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest85(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 8, 0), Vector3(0.866025f, 6.5f, 0), Vector3(-0.866025f, 6.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest86(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -6, 0), Vector3(0.866025f, -4.5f, 0), Vector3(-0.866025f, -4.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest87(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -8, 0), Vector3(0.866025f, -6.5f, 0), Vector3(-0.866025f, -6.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest88(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(15.9946f, 1, 5.33154f), Vector3(15.9946f, -0.5f, 4.46551f), Vector3(15.9946f, -0.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest89(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 16, 5.33154f), Vector3(0, 14.5f, 4.46551f), Vector3(0, 14.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest90(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-15.9946f, 1, 5.33154f), Vector3(-15.9946f, -0.5f, 4.46551f), Vector3(-15.9946f, -0.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest91(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -14, 5.33154f), Vector3(0, -15.5f, 4.46551f), Vector3(0, -15.5f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest92(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(15.9946f, 1, -5.33154f), Vector3(15.9946f, -0.5f, -6.19756f), Vector3(15.9946f, -0.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest93(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 16, -5.33154f), Vector3(0, 14.5f, -6.19756f), Vector3(0, 14.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest94(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-15.9946f, 1, -5.33154f), Vector3(-15.9946f, -0.5f, -6.19756f), Vector3(-15.9946f, -0.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest95(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -14, -5.33154f), Vector3(0, -15.5f, -6.19756f), Vector3(0, -15.5f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest96(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(5.33154f, 15, -1), Vector3(6.19756f, 15, 0.5f), Vector3(4.46551f, 15, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest97(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(5.33154f, 0, 14.9946f), Vector3(6.19756f, 0, 16.4946f), Vector3(4.46551f, 0, 16.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest98(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(5.33154f, -15, -1), Vector3(6.19756f, -15, 0.5f), Vector3(4.46551f, -15, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest99(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(5.33154f, 0, -16.9946f), Vector3(6.19756f, 0, -15.4946f), Vector3(4.46551f, 0, -15.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest100(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-5.33154f, 15, -1), Vector3(-4.46551f, 15, 0.5f), Vector3(-6.19756f, 15, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest101(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-5.33154f, 0, 14.9946f), Vector3(-4.46551f, 0, 16.4946f), Vector3(-6.19756f, 0, 16.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest102(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-5.33154f, -15, -1), Vector3(-4.46551f, -15, 0.5f), Vector3(-6.19756f, -15, 0.5f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest103(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-5.33154f, 0, -16.9946f), Vector3(-4.46551f, 0, -15.4946f), Vector3(-6.19756f, 0, -15.4946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest104(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 6, 15.9946f), Vector3(0.866025f, 4.5f, 15.9946f), Vector3(-0.866025f, 4.5f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest105(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(15.9946f, 6, 0), Vector3(16.8606f, 4.5f, 0), Vector3(15.1286f, 4.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest106(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, 6, -15.9946f), Vector3(0.866025f, 4.5f, -15.9946f), Vector3(-0.866025f, 4.5f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest107(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-15.9946f, 6, 0), Vector3(-15.1286f, 4.5f, 0), Vector3(-16.8606f, 4.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest108(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -4, 15.9946f), Vector3(0.866025f, -5.5f, 15.9946f), Vector3(-0.866025f, -5.5f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest109(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(15.9946f, -4, 0), Vector3(16.8606f, -5.5f, 0), Vector3(15.1286f, -5.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest110(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(0, -4, -15.9946f), Vector3(0.866025f, -5.5f, -15.9946f), Vector3(-0.866025f, -5.5f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest111(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Triangle triangle(Vector3(-15.9946f, -4, 0), Vector3(-15.1286f, -5.5f, 0), Vector3(-16.8606f, -5.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

// Triangle inside of frustum.
void FrustumTriangleTest112(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(0, 1, 0), Vector3(0.866025f, -0.5f, 0), Vector3(-0.866025f, -0.5f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest113(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(3.06286f, 3.06286f, 0), Vector3(4.12352f, 4.12352f, -0.866025f), Vector3(4.12352f, 4.12352f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest114(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(4.47707f, 4.47707f, 0), Vector3(5.53773f, 5.53773f, -0.866025f), Vector3(5.53773f, 5.53773f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest115(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-3.06286f, -3.06286f, 0), Vector3(-4.12352f, -4.12352f, -0.866025f), Vector3(-4.12352f, -4.12352f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest116(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-4.47707f, -4.47707f, 0), Vector3(-5.53773f, -5.53773f, -0.866025f), Vector3(-5.53773f, -5.53773f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest117(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-3.76997f, 4.76997f, 0), Vector3(-2.90394f, 3.26997f, 0), Vector3(-4.63599f, 3.26997f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest118(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-5.18418f, 6.18418f, 0), Vector3(-4.31816f, 4.68418f, 0), Vector3(-6.05021f, 4.68418f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest119(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(3.76997f, -4.76997f, 0), Vector3(4.63599f, -3.26997f, 0), Vector3(2.90394f, -3.26997f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest120(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(5.18418f, -6.18418f, 0), Vector3(6.05021f, -4.68418f, 0), Vector3(4.31816f, -4.68418f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest121(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-0.707107f, 0.707107f, 5), Vector3(0.353553f, -0.353553f, 4.13397f), Vector3(0.353553f, -0.353553f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest122(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-0.707107f, 0.707107f, 7), Vector3(0.353553f, -0.353553f, 6.13397f), Vector3(0.353553f, -0.353553f, 7.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest123(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(0.707107f, -0.707107f, -5), Vector3(-0.353553f, 0.353553f, -5.86603f), Vector3(-0.353553f, 0.353553f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest124(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(0.707107f, -0.707107f, -7), Vector3(-0.353553f, 0.353553f, -7.86603f), Vector3(-0.353553f, 0.353553f, -6.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest125(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-8.24704f, 14.3728f, 0), Vector3(-7.18638f, 15.4334f, -0.866025f), Vector3(-7.18638f, 15.4334f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest126(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(3.06286f, 3.06286f, 15), Vector3(4.12352f, 4.12352f, 14.134f), Vector3(4.12352f, 4.12352f, 15.866f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest127(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(14.3728f, -8.24704f, 0), Vector3(15.4334f, -7.18638f, -0.866025f), Vector3(15.4334f, -7.18638f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest128(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(3.06286f, 3.06286f, -15), Vector3(4.12352f, 4.12352f, -15.866f), Vector3(4.12352f, 4.12352f, -14.134f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest129(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-15.787f, 6.83283f, 0), Vector3(-14.7263f, 7.89349f, -0.866025f), Vector3(-14.7263f, 7.89349f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest130(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-4.47707f, -4.47707f, 15), Vector3(-3.41641f, -3.41641f, 14.134f), Vector3(-3.41641f, -3.41641f, 15.866f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest131(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(6.83283f, -15.787f, 0), Vector3(7.89349f, -14.7263f, -0.866025f), Vector3(7.89349f, -14.7263f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest132(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-4.47707f, -4.47707f, -15), Vector3(-3.41641f, -3.41641f, -15.866f), Vector3(-3.41641f, -3.41641f, -14.134f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest133(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-3.76997f, 4.76997f, 15), Vector3(-2.90394f, 3.26997f, 15), Vector3(-4.63599f, 3.26997f, 15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest134(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(7.53993f, 16.0799f, 0), Vector3(8.40596f, 14.5799f, 0), Vector3(6.67391f, 14.5799f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest135(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-3.76997f, 4.76997f, -15), Vector3(-2.90394f, 3.26997f, -15), Vector3(-4.63599f, 3.26997f, -15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest136(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-15.0799f, -6.53993f, 0), Vector3(-14.2138f, -8.03993f, 0), Vector3(-15.9459f, -8.03993f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest137(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(3.76997f, -2.76997f, 15), Vector3(4.63599f, -4.26997f, 15), Vector3(2.90394f, -4.26997f, 15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest138(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(15.0799f, 8.53993f, 0), Vector3(15.9459f, 7.03993f, 0), Vector3(14.2138f, 7.03993f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest139(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(3.76997f, -2.76997f, -15), Vector3(4.63599f, -4.26997f, -15), Vector3(2.90394f, -4.26997f, -15));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest140(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-7.53993f, -14.0799f, 0), Vector3(-6.67391f, -15.5799f, 0), Vector3(-8.40596f, -15.5799f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest141(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(10.6028f, 12.017f, 5), Vector3(11.6635f, 10.9563f, 4.13397f), Vector3(11.6635f, 10.9563f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest142(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-12.017f, 12.017f, 5), Vector3(-10.9563f, 10.9563f, 4.13397f), Vector3(-10.9563f, 10.9563f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest143(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-12.017f, -10.6028f, 5), Vector3(-10.9563f, -11.6635f, 4.13397f), Vector3(-10.9563f, -11.6635f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest144(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(10.6028f, -10.6028f, 5), Vector3(11.6635f, -11.6635f, 4.13397f), Vector3(11.6635f, -11.6635f, 5.86603f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest145(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(10.6028f, 12.017f, -5), Vector3(11.6635f, 10.9563f, -5.86603f), Vector3(11.6635f, 10.9563f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest146(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-12.017f, 12.017f, -5), Vector3(-10.9563f, 10.9563f, -5.86603f), Vector3(-10.9563f, 10.9563f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest147(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(-12.017f, -10.6028f, -5), Vector3(-10.9563f, -11.6635f, -5.86603f), Vector3(-10.9563f, -11.6635f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest148(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Triangle triangle(Vector3(10.6028f, -10.6028f, -5), Vector3(11.6635f, -11.6635f, -5.86603f), Vector3(11.6635f, -11.6635f, -4.13397f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

// Triangle inside of frustum.
void FrustumTriangleTest149(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-0.707107f, 0.707107f, 0), Vector3(0.353553f, -0.353553f, -0.866025f), Vector3(0.353553f, -0.353553f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest150(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-3.76997f, 4.76997f, 0), Vector3(-2.90394f, 3.26997f, 0), Vector3(-4.63599f, 3.26997f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest151(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-5.18418f, 6.18418f, 0), Vector3(-4.31816f, 4.68418f, 0), Vector3(-6.05021f, 4.68418f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest152(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(3.76997f, -4.76997f, 0), Vector3(4.63599f, -3.26997f, 0), Vector3(2.90394f, -3.26997f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest153(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(5.18418f, -6.18418f, 0), Vector3(6.05021f, -4.68418f, 0), Vector3(4.31816f, -4.68418f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest154(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-0.707107f, 0.707107f, 5.33154f), Vector3(0.353553f, -0.353553f, 4.46551f), Vector3(0.353553f, -0.353553f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest155(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-0.707107f, 0.707107f, 7.33154f), Vector3(0.353553f, -0.353553f, 6.46551f), Vector3(0.353553f, -0.353553f, 8.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest156(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(0.707107f, -0.707107f, -5.33154f), Vector3(-0.353553f, 0.353553f, -6.19756f), Vector3(-0.353553f, 0.353553f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest157(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(0.707107f, -0.707107f, -7.33154f), Vector3(-0.353553f, 0.353553f, -8.19756f), Vector3(-0.353553f, 0.353553f, -6.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest158(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(2.82843f, 2.82843f, 0), Vector3(3.88909f, 3.88909f, -0.866025f), Vector3(3.88909f, 3.88909f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest159(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(4.24264f, 4.24264f, 0), Vector3(5.3033f, 5.3033f, -0.866025f), Vector3(5.3033f, 5.3033f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest160(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-2.82843f, -2.82843f, 0), Vector3(-3.88909f, -3.88909f, -0.866025f), Vector3(-3.88909f, -3.88909f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest161(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-4.24264f, -4.24264f, 0), Vector3(-5.3033f, -5.3033f, -0.866025f), Vector3(-5.3033f, -5.3033f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest162(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-3.76997f, 4.76997f, 15.9946f), Vector3(-2.90394f, 3.26997f, 15.9946f), Vector3(-4.63599f, 3.26997f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest163(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(6.83663f, 15.3766f, 0), Vector3(7.70266f, 13.8766f, 0), Vector3(5.97061f, 13.8766f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest164(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-3.76997f, 4.76997f, -15.9946f), Vector3(-2.90394f, 3.26997f, -15.9946f), Vector3(-4.63599f, 3.26997f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest165(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-14.3766f, -5.83663f, 0), Vector3(-13.5105f, -7.33663f, 0), Vector3(-15.2426f, -7.33663f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest166(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(3.76997f, -2.76997f, 15.9946f), Vector3(4.63599f, -4.26997f, 15.9946f), Vector3(2.90394f, -4.26997f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest167(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(14.3766f, 7.83663f, 0), Vector3(15.2426f, 6.33663f, 0), Vector3(13.5105f, 6.33663f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest168(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(3.76997f, -2.76997f, -15.9946f), Vector3(4.63599f, -4.26997f, -15.9946f), Vector3(2.90394f, -4.26997f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest169(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-6.83663f, -13.3766f, 0), Vector3(-5.97061f, -14.8766f, 0), Vector3(-7.70266f, -14.8766f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest170(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(9.8995f, 11.3137f, 5.33154f), Vector3(10.9602f, 10.253f, 4.46551f), Vector3(10.9602f, 10.253f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest171(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-12.017f, 12.017f, 5.33154f), Vector3(-10.9563f, 10.9563f, 4.46551f), Vector3(-10.9563f, 10.9563f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest172(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-11.3137f, -9.8995f, 5.33154f), Vector3(-10.253f, -10.9602f, 4.46551f), Vector3(-10.253f, -10.9602f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest173(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(10.6028f, -10.6028f, 5.33154f), Vector3(11.6635f, -11.6635f, 4.46551f), Vector3(11.6635f, -11.6635f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest174(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(9.8995f, 11.3137f, -5.33154f), Vector3(10.9602f, 10.253f, -6.19756f), Vector3(10.9602f, 10.253f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest175(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-12.017f, 12.017f, -5.33154f), Vector3(-10.9563f, 10.9563f, -6.19756f), Vector3(-10.9563f, 10.9563f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest176(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-11.3137f, -9.8995f, -5.33154f), Vector3(-10.253f, -10.9602f, -6.19756f), Vector3(-10.253f, -10.9602f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest177(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(10.6028f, -10.6028f, -5.33154f), Vector3(11.6635f, -11.6635f, -6.19756f), Vector3(11.6635f, -11.6635f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest178(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-8.48147f, 14.1383f, 0), Vector3(-7.42081f, 15.199f, -0.866025f), Vector3(-7.42081f, 15.199f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest179(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(2.82843f, 2.82843f, 15.9946f), Vector3(3.88909f, 3.88909f, 15.1286f), Vector3(3.88909f, 3.88909f, 16.8606f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest180(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(14.1383f, -8.48147f, 0), Vector3(15.199f, -7.42081f, -0.866025f), Vector3(15.199f, -7.42081f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest181(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(2.82843f, 2.82843f, -15.9946f), Vector3(3.88909f, 3.88909f, -16.8606f), Vector3(3.88909f, 3.88909f, -15.1286f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest182(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-15.5525f, 7.06726f, 0), Vector3(-14.4919f, 8.12792f, -0.866025f), Vector3(-14.4919f, 8.12792f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest183(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-4.24264f, -4.24264f, 15.9946f), Vector3(-3.18198f, -3.18198f, 15.1286f), Vector3(-3.18198f, -3.18198f, 16.8606f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest184(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(7.06726f, -15.5525f, 0), Vector3(8.12792f, -14.4919f, -0.866025f), Vector3(8.12792f, -14.4919f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest185(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Triangle triangle(Vector3(-4.24264f, -4.24264f, -15.9946f), Vector3(-3.18198f, -3.18198f, -16.8606f), Vector3(-3.18198f, -3.18198f, -15.1286f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

// Triangle inside of frustum.
void FrustumTriangleTest186(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-0.707107f, -0.707107f, 0), Vector3(0.353553f, 0.353553f, -0.866025f), Vector3(0.353553f, 0.353553f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest187(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-0.707107f, 0.707107f, 5.33154f), Vector3(0.353553f, -0.353553f, 4.46551f), Vector3(0.353553f, -0.353553f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest188(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-0.707107f, 0.707107f, 7.33154f), Vector3(0.353553f, -0.353553f, 6.46551f), Vector3(0.353553f, -0.353553f, 8.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest189(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(0.707107f, -0.707107f, -5.33154f), Vector3(-0.353553f, 0.353553f, -6.19756f), Vector3(-0.353553f, 0.353553f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest190(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(0.707107f, -0.707107f, -7.33154f), Vector3(-0.353553f, 0.353553f, -8.19756f), Vector3(-0.353553f, 0.353553f, -6.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest191(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(3.06286f, 3.06286f, 0), Vector3(4.12352f, 4.12352f, -0.866025f), Vector3(4.12352f, 4.12352f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest192(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(4.47707f, 4.47707f, 0), Vector3(5.53773f, 5.53773f, -0.866025f), Vector3(5.53773f, 5.53773f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest193(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-3.06286f, -3.06286f, 0), Vector3(-4.12352f, -4.12352f, -0.866025f), Vector3(-4.12352f, -4.12352f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest194(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-4.47707f, -4.47707f, 0), Vector3(-5.53773f, -5.53773f, -0.866025f), Vector3(-5.53773f, -5.53773f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest195(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-3.53553f, 4.53553f, 0), Vector3(-2.66951f, 3.03553f, 0), Vector3(-4.40156f, 3.03553f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest196(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-4.94975f, 5.94975f, 0), Vector3(-4.08372f, 4.44975f, 0), Vector3(-5.81577f, 4.44975f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest197(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(3.53553f, -4.53553f, 0), Vector3(4.40156f, -3.03553f, 0), Vector3(2.66951f, -3.03553f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest198(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(4.94975f, -5.94975f, 0), Vector3(5.81577f, -4.44975f, 0), Vector3(4.08372f, -4.44975f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest199(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(10.6028f, 12.017f, 5.33154f), Vector3(11.6635f, 10.9563f, 4.46551f), Vector3(11.6635f, 10.9563f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest200(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-11.3137f, 11.3137f, 5.33154f), Vector3(-10.253f, 10.253f, 4.46551f), Vector3(-10.253f, 10.253f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest201(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-12.017f, -10.6028f, 5.33154f), Vector3(-10.9563f, -11.6635f, 4.46551f), Vector3(-10.9563f, -11.6635f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest202(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(9.8995f, -9.8995f, 5.33154f), Vector3(10.9602f, -10.9602f, 4.46551f), Vector3(10.9602f, -10.9602f, 6.19756f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest203(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(10.6028f, 12.017f, -5.33154f), Vector3(11.6635f, 10.9563f, -6.19756f), Vector3(11.6635f, 10.9563f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest204(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-11.3137f, 11.3137f, -5.33154f), Vector3(-10.253f, 10.253f, -6.19756f), Vector3(-10.253f, 10.253f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest205(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-12.017f, -10.6028f, -5.33154f), Vector3(-10.9563f, -11.6635f, -6.19756f), Vector3(-10.9563f, -11.6635f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest206(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(9.8995f, -9.8995f, -5.33154f), Vector3(10.9602f, -10.9602f, -6.19756f), Vector3(10.9602f, -10.9602f, -4.46551f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest207(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-7.54374f, 13.6695f, 0), Vector3(-6.48308f, 14.7301f, -0.866025f), Vector3(-6.48308f, 14.7301f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest208(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(3.06286f, 3.06286f, 15.9946f), Vector3(4.12352f, 4.12352f, 15.1286f), Vector3(4.12352f, 4.12352f, 16.8606f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest209(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(13.6695f, -7.54374f, 0), Vector3(14.7301f, -6.48308f, -0.866025f), Vector3(14.7301f, -6.48308f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest210(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(3.06286f, 3.06286f, -15.9946f), Vector3(4.12352f, 4.12352f, -16.8606f), Vector3(4.12352f, 4.12352f, -15.1286f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest211(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-15.0837f, 6.12953f, 0), Vector3(-14.023f, 7.19019f, -0.866025f), Vector3(-14.023f, 7.19019f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest212(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-4.47707f, -4.47707f, 15.9946f), Vector3(-3.41641f, -3.41641f, 15.1286f), Vector3(-3.41641f, -3.41641f, 16.8606f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest213(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(6.12953f, -15.0837f, 0), Vector3(7.19019f, -14.023f, -0.866025f), Vector3(7.19019f, -14.023f, 0.866025f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest214(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-4.47707f, -4.47707f, -15.9946f), Vector3(-3.41641f, -3.41641f, -16.8606f), Vector3(-3.41641f, -3.41641f, -15.1286f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest215(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-3.53553f, 4.53553f, 15.9946f), Vector3(-2.66951f, 3.03553f, 15.9946f), Vector3(-4.40156f, 3.03553f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest216(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(7.77437f, 15.8454f, 0), Vector3(8.64039f, 14.3454f, 0), Vector3(6.90834f, 14.3454f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest217(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-3.53553f, 4.53553f, -15.9946f), Vector3(-2.66951f, 3.03553f, -15.9946f), Vector3(-4.40156f, 3.03553f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest218(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-14.8454f, -6.77437f, 0), Vector3(-13.9794f, -8.27437f, 0), Vector3(-15.7115f, -8.27437f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest219(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(3.53553f, -2.53553f, 15.9946f), Vector3(4.40156f, -4.03553f, 15.9946f), Vector3(2.66951f, -4.03553f, 15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest220(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(14.8454f, 8.77437f, 0), Vector3(15.7115f, 7.27437f, 0), Vector3(13.9794f, 7.27437f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest221(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(3.53553f, -2.53553f, -15.9946f), Vector3(4.40156f, -4.03553f, -15.9946f), Vector3(2.66951f, -4.03553f, -15.9946f));
  
  TestFrustumTriangle(frustum, triangle, file);
}

void FrustumTriangleTest222(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Triangle triangle(Vector3(-7.77437f, -13.8454f, 0), Vector3(-6.90834f, -15.3454f, 0), Vector3(-8.64039f, -15.3454f, 0));
  
  TestFrustumTriangle(frustum, triangle, file);
}

// Aabb inside of frustum.
void FrustumAabbTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on right of frustum.
void FrustumAabbTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(5.83154f, -0.5f, -0.5f), Vector3(6.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on left of frustum.
void FrustumAabbTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-6.83154f, -0.5f, -0.5f), Vector3(-5.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on top of frustum.
void FrustumAabbTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, 5.83154f, -0.5f), Vector3(0.5f, 6.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on bottom of frustum.
void FrustumAabbTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -6.83154f, -0.5f), Vector3(0.5f, -5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb in front of frustum.
void FrustumAabbTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.5f), Vector3(0.5f, 0.5f, 6.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb behind frustum.
void FrustumAabbTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.5f), Vector3(0.5f, 0.5f, -5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting right side of frustum.
void FrustumAabbTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(4.83154f, -0.5f, -0.5f), Vector3(5.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting left side of frustum.
void FrustumAabbTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-5.83154f, -0.5f, -0.5f), Vector3(-4.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting top side of frustum.
void FrustumAabbTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, 4.83154f, -0.5f), Vector3(0.5f, 5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting bottom side of frustum.
void FrustumAabbTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -5.83154f, -0.5f), Vector3(0.5f, -4.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting front of frustum.
void FrustumAabbTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 4.5f), Vector3(0.5f, 0.5f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting back of frustum.
void FrustumAabbTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -5.5f), Vector3(0.5f, 0.5f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(4.83154f, 15.4946f, -0.5f), Vector3(5.83154f, 16.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(4.83154f, -16.4946f, -0.5f), Vector3(5.83154f, -15.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(4.83154f, -0.5f, 14.5f), Vector3(5.83154f, 0.5f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(4.83154f, -0.5f, -15.5f), Vector3(5.83154f, 0.5f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-5.83154f, 15.4946f, -0.5f), Vector3(-4.83154f, 16.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-5.83154f, -16.4946f, -0.5f), Vector3(-4.83154f, -15.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-5.83154f, -0.5f, 14.5f), Vector3(-4.83154f, 0.5f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-5.83154f, -0.5f, -15.5f), Vector3(-4.83154f, 0.5f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, 4.83154f, 14.5f), Vector3(0.5f, 5.83154f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, 4.83154f, -15.5f), Vector3(0.5f, 5.83154f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(15.4946f, 4.83154f, -0.5f), Vector3(16.4946f, 5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-16.4946f, 4.83154f, -0.5f), Vector3(-15.4946f, 5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -5.83154f, 14.5f), Vector3(0.5f, -4.83154f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -5.83154f, -15.5f), Vector3(0.5f, -4.83154f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(15.4946f, -5.83154f, -0.5f), Vector3(16.4946f, -4.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-16.4946f, -5.83154f, -0.5f), Vector3(-15.4946f, -4.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(15.4946f, -0.5f, 4.5f), Vector3(16.4946f, 0.5f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-16.4946f, -0.5f, 4.5f), Vector3(-15.4946f, 0.5f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, 15.4946f, 4.5f), Vector3(0.5f, 16.4946f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -16.4946f, 4.5f), Vector3(0.5f, -15.4946f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(15.4946f, -0.5f, -5.5f), Vector3(16.4946f, 0.5f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-16.4946f, -0.5f, -5.5f), Vector3(-15.4946f, 0.5f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, 15.4946f, -5.5f), Vector3(0.5f, 16.4946f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -16.4946f, -5.5f), Vector3(0.5f, -15.4946f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's near plane.
void FrustumAabbTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.05f), Vector3(0.5f, 0.5f, 6.05f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's near plane.
void FrustumAabbTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 3.95f), Vector3(0.5f, 0.5f, 4.95f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's far plane.
void FrustumAabbTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.05f), Vector3(0.5f, 0.5f, -5.05f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's far plane.
void FrustumAabbTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -4.95f), Vector3(0.5f, 0.5f, -3.95f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb inside of frustum.
void FrustumAabbTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on right of frustum.
void FrustumAabbTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, 5.83154f, -0.5f), Vector3(0.5f, 6.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on left of frustum.
void FrustumAabbTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -6.83154f, -0.5f), Vector3(0.5f, -5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on top of frustum.
void FrustumAabbTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.83154f), Vector3(0.5f, 0.5f, 6.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on bottom of frustum.
void FrustumAabbTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.83154f), Vector3(0.5f, 0.5f, -5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb in front of frustum.
void FrustumAabbTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(5.5f, -0.5f, -0.5f), Vector3(6.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb behind frustum.
void FrustumAabbTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-6.5f, -0.5f, -0.5f), Vector3(-5.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting right side of frustum.
void FrustumAabbTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, 4.83154f, -0.5f), Vector3(0.5f, 5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting left side of frustum.
void FrustumAabbTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -5.83154f, -0.5f), Vector3(0.5f, -4.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting top side of frustum.
void FrustumAabbTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 4.83154f), Vector3(0.5f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting bottom side of frustum.
void FrustumAabbTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -5.83154f), Vector3(0.5f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting front of frustum.
void FrustumAabbTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(4.5f, -0.5f, -0.5f), Vector3(5.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting back of frustum.
void FrustumAabbTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-5.5f, -0.5f, -0.5f), Vector3(-4.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, 4.83154f, 15.4946f), Vector3(0.5f, 5.83154f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, 4.83154f, -16.4946f), Vector3(0.5f, 5.83154f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(14.5f, 4.83154f, -0.5f), Vector3(15.5f, 5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-15.5f, 4.83154f, -0.5f), Vector3(-14.5f, 5.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -5.83154f, 15.4946f), Vector3(0.5f, -4.83154f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -5.83154f, -16.4946f), Vector3(0.5f, -4.83154f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(14.5f, -5.83154f, -0.5f), Vector3(15.5f, -4.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-15.5f, -5.83154f, -0.5f), Vector3(-14.5f, -4.83154f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(14.5f, -0.5f, 4.83154f), Vector3(15.5f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-15.5f, -0.5f, 4.83154f), Vector3(-14.5f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, 15.4946f, 4.83154f), Vector3(0.5f, 16.4946f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -16.4946f, 4.83154f), Vector3(0.5f, -15.4946f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(14.5f, -0.5f, -5.83154f), Vector3(15.5f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-15.5f, -0.5f, -5.83154f), Vector3(-14.5f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, 15.4946f, -5.83154f), Vector3(0.5f, 16.4946f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-0.5f, -16.4946f, -5.83154f), Vector3(0.5f, -15.4946f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(4.5f, 15.4946f, -0.5f), Vector3(5.5f, 16.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(4.5f, -16.4946f, -0.5f), Vector3(5.5f, -15.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(4.5f, -0.5f, 15.4946f), Vector3(5.5f, 0.5f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(4.5f, -0.5f, -16.4946f), Vector3(5.5f, 0.5f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-5.5f, 15.4946f, -0.5f), Vector3(-4.5f, 16.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-5.5f, -16.4946f, -0.5f), Vector3(-4.5f, -15.4946f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-5.5f, -0.5f, 15.4946f), Vector3(-4.5f, 0.5f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-5.5f, -0.5f, -16.4946f), Vector3(-4.5f, 0.5f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's near plane.
void FrustumAabbTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(5.05f, -0.5f, -0.5f), Vector3(6.05f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's near plane.
void FrustumAabbTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(3.95f, -0.5f, -0.5f), Vector3(4.95f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's far plane.
void FrustumAabbTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-6.05f, -0.5f, -0.5f), Vector3(-5.05f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's far plane.
void FrustumAabbTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Aabb aabb(Vector3(-4.95f, -0.5f, -0.5f), Vector3(-3.95f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb inside of frustum.
void FrustumAabbTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on right of frustum.
void FrustumAabbTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.83154f), Vector3(0.5f, 0.5f, 6.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on left of frustum.
void FrustumAabbTest85(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.83154f), Vector3(0.5f, 0.5f, -5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on top of frustum.
void FrustumAabbTest86(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(5.83154f, -0.5f, -0.5f), Vector3(6.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on bottom of frustum.
void FrustumAabbTest87(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-6.83154f, -0.5f, -0.5f), Vector3(-5.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb in front of frustum.
void FrustumAabbTest88(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 5.5f, -0.5f), Vector3(0.5f, 6.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb behind frustum.
void FrustumAabbTest89(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -6.5f, -0.5f), Vector3(0.5f, -5.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting right side of frustum.
void FrustumAabbTest90(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 4.83154f), Vector3(0.5f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting left side of frustum.
void FrustumAabbTest91(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -5.83154f), Vector3(0.5f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting top side of frustum.
void FrustumAabbTest92(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(4.83154f, -0.5f, -0.5f), Vector3(5.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting bottom side of frustum.
void FrustumAabbTest93(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-5.83154f, -0.5f, -0.5f), Vector3(-4.83154f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting front of frustum.
void FrustumAabbTest94(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 4.5f, -0.5f), Vector3(0.5f, 5.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting back of frustum.
void FrustumAabbTest95(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -5.5f, -0.5f), Vector3(0.5f, -4.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest96(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(15.4946f, -0.5f, 4.83154f), Vector3(16.4946f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest97(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-16.4946f, -0.5f, 4.83154f), Vector3(-15.4946f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest98(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 14.5f, 4.83154f), Vector3(0.5f, 15.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest99(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -15.5f, 4.83154f), Vector3(0.5f, -14.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest100(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(15.4946f, -0.5f, -5.83154f), Vector3(16.4946f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest101(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-16.4946f, -0.5f, -5.83154f), Vector3(-15.4946f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest102(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 14.5f, -5.83154f), Vector3(0.5f, 15.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest103(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -15.5f, -5.83154f), Vector3(0.5f, -14.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest104(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(4.83154f, 14.5f, -0.5f), Vector3(5.83154f, 15.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest105(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(4.83154f, -15.5f, -0.5f), Vector3(5.83154f, -14.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest106(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(4.83154f, -0.5f, 15.4946f), Vector3(5.83154f, 0.5f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest107(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(4.83154f, -0.5f, -16.4946f), Vector3(5.83154f, 0.5f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest108(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-5.83154f, 14.5f, -0.5f), Vector3(-4.83154f, 15.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest109(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-5.83154f, -15.5f, -0.5f), Vector3(-4.83154f, -14.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest110(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-5.83154f, -0.5f, 15.4946f), Vector3(-4.83154f, 0.5f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest111(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-5.83154f, -0.5f, -16.4946f), Vector3(-4.83154f, 0.5f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest112(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 4.5f, 15.4946f), Vector3(0.5f, 5.5f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest113(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 4.5f, -16.4946f), Vector3(0.5f, 5.5f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest114(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(15.4946f, 4.5f, -0.5f), Vector3(16.4946f, 5.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest115(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-16.4946f, 4.5f, -0.5f), Vector3(-15.4946f, 5.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest116(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -5.5f, 15.4946f), Vector3(0.5f, -4.5f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest117(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -5.5f, -16.4946f), Vector3(0.5f, -4.5f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest118(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(15.4946f, -5.5f, -0.5f), Vector3(16.4946f, -4.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest119(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-16.4946f, -5.5f, -0.5f), Vector3(-15.4946f, -4.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's near plane.
void FrustumAabbTest120(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 5.05f, -0.5f), Vector3(0.5f, 6.05f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's near plane.
void FrustumAabbTest121(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, 3.95f, -0.5f), Vector3(0.5f, 4.95f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's far plane.
void FrustumAabbTest122(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -6.05f, -0.5f), Vector3(0.5f, -5.05f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's far plane.
void FrustumAabbTest123(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Aabb aabb(Vector3(-0.5f, -4.95f, -0.5f), Vector3(0.5f, -3.95f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb inside of frustum.
void FrustumAabbTest124(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on right of frustum.
void FrustumAabbTest125(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(4.26997f, 4.26997f, -0.5f), Vector3(5.26997f, 5.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on left of frustum.
void FrustumAabbTest126(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-5.26997f, -5.26997f, -0.5f), Vector3(-4.26997f, -4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on top of frustum.
void FrustumAabbTest127(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-5.26997f, 4.26997f, -0.5f), Vector3(-4.26997f, 5.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on bottom of frustum.
void FrustumAabbTest128(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(4.26997f, -5.26997f, -0.5f), Vector3(5.26997f, -4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb in front of frustum.
void FrustumAabbTest129(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.5f), Vector3(0.5f, 0.5f, 6.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb behind frustum.
void FrustumAabbTest130(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.5f), Vector3(0.5f, 0.5f, -5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting right side of frustum.
void FrustumAabbTest131(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(3.26997f, 3.26997f, -0.5f), Vector3(4.26997f, 4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting left side of frustum.
void FrustumAabbTest132(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-4.26997f, -4.26997f, -0.5f), Vector3(-3.26997f, -3.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting top side of frustum.
void FrustumAabbTest133(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-4.26997f, 3.26997f, -0.5f), Vector3(-3.26997f, 4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting bottom side of frustum.
void FrustumAabbTest134(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(3.26997f, -4.26997f, -0.5f), Vector3(4.26997f, -3.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting front of frustum.
void FrustumAabbTest135(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 4.5f), Vector3(0.5f, 0.5f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting back of frustum.
void FrustumAabbTest136(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -5.5f), Vector3(0.5f, 0.5f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest137(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-8.03993f, 14.5799f, -0.5f), Vector3(-7.03993f, 15.5799f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest138(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(14.5799f, -8.03993f, -0.5f), Vector3(15.5799f, -7.03993f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest139(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(3.26997f, 3.26997f, 14.5f), Vector3(4.26997f, 4.26997f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest140(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(3.26997f, 3.26997f, -15.5f), Vector3(4.26997f, 4.26997f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest141(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-15.5799f, 7.03993f, -0.5f), Vector3(-14.5799f, 8.03993f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest142(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(7.03993f, -15.5799f, -0.5f), Vector3(8.03993f, -14.5799f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest143(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-4.26997f, -4.26997f, 14.5f), Vector3(-3.26997f, -3.26997f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest144(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-4.26997f, -4.26997f, -15.5f), Vector3(-3.26997f, -3.26997f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest145(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-4.26997f, 3.26997f, 14.5f), Vector3(-3.26997f, 4.26997f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest146(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-4.26997f, 3.26997f, -15.5f), Vector3(-3.26997f, 4.26997f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest147(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(7.03993f, 14.5799f, -0.5f), Vector3(8.03993f, 15.5799f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest148(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-15.5799f, -8.03993f, -0.5f), Vector3(-14.5799f, -7.03993f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest149(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(3.26997f, -4.26997f, 14.5f), Vector3(4.26997f, -3.26997f, 15.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest150(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(3.26997f, -4.26997f, -15.5f), Vector3(4.26997f, -3.26997f, -14.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest151(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(14.5799f, 7.03993f, -0.5f), Vector3(15.5799f, 8.03993f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest152(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-8.03993f, -15.5799f, -0.5f), Vector3(-7.03993f, -14.5799f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest153(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(10.8099f, 10.8099f, 4.5f), Vector3(11.8099f, 11.8099f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest154(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-11.8099f, -11.8099f, 4.5f), Vector3(-10.8099f, -10.8099f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest155(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-11.8099f, 10.8099f, 4.5f), Vector3(-10.8099f, 11.8099f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest156(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(10.8099f, -11.8099f, 4.5f), Vector3(11.8099f, -10.8099f, 5.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest157(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(10.8099f, 10.8099f, -5.5f), Vector3(11.8099f, 11.8099f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest158(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-11.8099f, -11.8099f, -5.5f), Vector3(-10.8099f, -10.8099f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest159(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-11.8099f, 10.8099f, -5.5f), Vector3(-10.8099f, 11.8099f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest160(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(10.8099f, -11.8099f, -5.5f), Vector3(11.8099f, -10.8099f, -4.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's near plane.
void FrustumAabbTest161(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.05f), Vector3(0.5f, 0.5f, 6.05f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's near plane.
void FrustumAabbTest162(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, 3.95f), Vector3(0.5f, 0.5f, 4.95f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's far plane.
void FrustumAabbTest163(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.05f), Vector3(0.5f, 0.5f, -5.05f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's far plane.
void FrustumAabbTest164(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Aabb aabb(Vector3(-0.5f, -0.5f, -4.95f), Vector3(0.5f, 0.5f, -3.95f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb inside of frustum.
void FrustumAabbTest165(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on right of frustum.
void FrustumAabbTest166(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-5.26997f, 4.26997f, -0.5f), Vector3(-4.26997f, 5.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on left of frustum.
void FrustumAabbTest167(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(4.26997f, -5.26997f, -0.5f), Vector3(5.26997f, -4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on top of frustum.
void FrustumAabbTest168(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.83154f), Vector3(0.5f, 0.5f, 6.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on bottom of frustum.
void FrustumAabbTest169(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.83154f), Vector3(0.5f, 0.5f, -5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb in front of frustum.
void FrustumAabbTest170(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(4.03553f, 4.03553f, -0.5f), Vector3(5.03553f, 5.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb behind frustum.
void FrustumAabbTest171(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-5.03553f, -5.03553f, -0.5f), Vector3(-4.03553f, -4.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting right side of frustum.
void FrustumAabbTest172(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-4.26997f, 3.26997f, -0.5f), Vector3(-3.26997f, 4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting left side of frustum.
void FrustumAabbTest173(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(3.26997f, -4.26997f, -0.5f), Vector3(4.26997f, -3.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting top side of frustum.
void FrustumAabbTest174(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 4.83154f), Vector3(0.5f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting bottom side of frustum.
void FrustumAabbTest175(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -5.83154f), Vector3(0.5f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting front of frustum.
void FrustumAabbTest176(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(3.03553f, 3.03553f, -0.5f), Vector3(4.03553f, 4.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting back of frustum.
void FrustumAabbTest177(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-4.03553f, -4.03553f, -0.5f), Vector3(-3.03553f, -3.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest178(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-4.26997f, 3.26997f, 15.4946f), Vector3(-3.26997f, 4.26997f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest179(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-4.26997f, 3.26997f, -16.4946f), Vector3(-3.26997f, 4.26997f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest180(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(6.33663f, 13.8766f, -0.5f), Vector3(7.33663f, 14.8766f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest181(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-14.8766f, -7.33663f, -0.5f), Vector3(-13.8766f, -6.33663f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest182(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(3.26997f, -4.26997f, 15.4946f), Vector3(4.26997f, -3.26997f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest183(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(3.26997f, -4.26997f, -16.4946f), Vector3(4.26997f, -3.26997f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest184(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(13.8766f, 6.33663f, -0.5f), Vector3(14.8766f, 7.33663f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest185(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-7.33663f, -14.8766f, -0.5f), Vector3(-6.33663f, -13.8766f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest186(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(10.1066f, 10.1066f, 4.83154f), Vector3(11.1066f, 11.1066f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest187(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-11.1066f, -11.1066f, 4.83154f), Vector3(-10.1066f, -10.1066f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest188(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-11.8099f, 10.8099f, 4.83154f), Vector3(-10.8099f, 11.8099f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest189(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(10.8099f, -11.8099f, 4.83154f), Vector3(11.8099f, -10.8099f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest190(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(10.1066f, 10.1066f, -5.83154f), Vector3(11.1066f, 11.1066f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest191(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-11.1066f, -11.1066f, -5.83154f), Vector3(-10.1066f, -10.1066f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest192(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-11.8099f, 10.8099f, -5.83154f), Vector3(-10.8099f, 11.8099f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest193(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(10.8099f, -11.8099f, -5.83154f), Vector3(11.8099f, -10.8099f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest194(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-8.27437f, 14.3454f, -0.5f), Vector3(-7.27437f, 15.3454f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest195(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(14.3454f, -8.27437f, -0.5f), Vector3(15.3454f, -7.27437f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest196(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(3.03553f, 3.03553f, 15.4946f), Vector3(4.03553f, 4.03553f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest197(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(3.03553f, 3.03553f, -16.4946f), Vector3(4.03553f, 4.03553f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest198(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-15.3454f, 7.27437f, -0.5f), Vector3(-14.3454f, 8.27437f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest199(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(7.27437f, -15.3454f, -0.5f), Vector3(8.27437f, -14.3454f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest200(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-4.03553f, -4.03553f, 15.4946f), Vector3(-3.03553f, -3.03553f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest201(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-4.03553f, -4.03553f, -16.4946f), Vector3(-3.03553f, -3.03553f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's near plane.
void FrustumAabbTest202(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(3.58553f, 3.58553f, -0.5f), Vector3(4.58553f, 4.58553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's near plane.
void FrustumAabbTest203(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(2.48553f, 2.48553f, -0.5f), Vector3(3.48553f, 3.48553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's far plane.
void FrustumAabbTest204(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-4.58553f, -4.58553f, -0.5f), Vector3(-3.58553f, -3.58553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's far plane.
void FrustumAabbTest205(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Aabb aabb(Vector3(-3.48553f, -3.48553f, -0.5f), Vector3(-2.48553f, -2.48553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb inside of frustum.
void FrustumAabbTest206(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on right of frustum.
void FrustumAabbTest207(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 5.83154f), Vector3(0.5f, 0.5f, 6.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on left of frustum.
void FrustumAabbTest208(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -6.83154f), Vector3(0.5f, 0.5f, -5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on top of frustum.
void FrustumAabbTest209(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(4.26997f, 4.26997f, -0.5f), Vector3(5.26997f, 5.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb on bottom of frustum.
void FrustumAabbTest210(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-5.26997f, -5.26997f, -0.5f), Vector3(-4.26997f, -4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb in front of frustum.
void FrustumAabbTest211(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-5.03553f, 4.03553f, -0.5f), Vector3(-4.03553f, 5.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb behind frustum.
void FrustumAabbTest212(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(4.03553f, -5.03553f, -0.5f), Vector3(5.03553f, -4.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting right side of frustum.
void FrustumAabbTest213(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, 4.83154f), Vector3(0.5f, 0.5f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting left side of frustum.
void FrustumAabbTest214(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-0.5f, -0.5f, -5.83154f), Vector3(0.5f, 0.5f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting top side of frustum.
void FrustumAabbTest215(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(3.26997f, 3.26997f, -0.5f), Vector3(4.26997f, 4.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting bottom side of frustum.
void FrustumAabbTest216(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-4.26997f, -4.26997f, -0.5f), Vector3(-3.26997f, -3.26997f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting front of frustum.
void FrustumAabbTest217(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-4.03553f, 3.03553f, -0.5f), Vector3(-3.03553f, 4.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting back of frustum.
void FrustumAabbTest218(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(3.03553f, -4.03553f, -0.5f), Vector3(4.03553f, -3.03553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest219(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(10.8099f, 10.8099f, 4.83154f), Vector3(11.8099f, 11.8099f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest220(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-11.8099f, -11.8099f, 4.83154f), Vector3(-10.8099f, -10.8099f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest221(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-11.1066f, 10.1066f, 4.83154f), Vector3(-10.1066f, 11.1066f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest222(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(10.1066f, -11.1066f, 4.83154f), Vector3(11.1066f, -10.1066f, 5.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest223(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(10.8099f, 10.8099f, -5.83154f), Vector3(11.8099f, 11.8099f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest224(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-11.8099f, -11.8099f, -5.83154f), Vector3(-10.8099f, -10.8099f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest225(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-11.1066f, 10.1066f, -5.83154f), Vector3(-10.1066f, 11.1066f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest226(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(10.1066f, -11.1066f, -5.83154f), Vector3(11.1066f, -10.1066f, -4.83154f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest227(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-7.33664f, 13.8766f, -0.5f), Vector3(-6.33664f, 14.8766f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest228(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(13.8766f, -7.33664f, -0.5f), Vector3(14.8766f, -6.33664f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest229(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(3.26997f, 3.26997f, 15.4946f), Vector3(4.26997f, 4.26997f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest230(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(3.26997f, 3.26997f, -16.4946f), Vector3(4.26997f, 4.26997f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest231(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-14.8766f, 6.33664f, -0.5f), Vector3(-13.8766f, 7.33664f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest232(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(6.33664f, -14.8766f, -0.5f), Vector3(7.33664f, -13.8766f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest233(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-4.26997f, -4.26997f, 15.4946f), Vector3(-3.26997f, -3.26997f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest234(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-4.26997f, -4.26997f, -16.4946f), Vector3(-3.26997f, -3.26997f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest235(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-4.03553f, 3.03553f, 15.4946f), Vector3(-3.03553f, 4.03553f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest236(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-4.03553f, 3.03553f, -16.4946f), Vector3(-3.03553f, 4.03553f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest237(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(7.27437f, 14.3454f, -0.5f), Vector3(8.27437f, 15.3454f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest238(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-15.3454f, -8.27437f, -0.5f), Vector3(-14.3454f, -7.27437f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest239(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(3.03553f, -4.03553f, 15.4946f), Vector3(4.03553f, -3.03553f, 16.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest240(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(3.03553f, -4.03553f, -16.4946f), Vector3(4.03553f, -3.03553f, -15.4946f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest241(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(14.3454f, 7.27437f, -0.5f), Vector3(15.3454f, 8.27437f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb intersecting one plane but outside another.
void FrustumAabbTest242(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-8.27437f, -15.3454f, -0.5f), Vector3(-7.27437f, -14.3454f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's near plane.
void FrustumAabbTest243(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-4.58553f, 3.58553f, -0.5f), Vector3(-3.58553f, 4.58553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's near plane.
void FrustumAabbTest244(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(-3.48553f, 2.48553f, -0.5f), Vector3(-2.48553f, 3.48553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just outside frustum's far plane.
void FrustumAabbTest245(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(3.58553f, -4.58553f, -0.5f), Vector3(4.58553f, -3.58553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Aabb just inside frustum's far plane.
void FrustumAabbTest246(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Aabb aabb(Vector3(2.48553f, -3.48553f, -0.5f), Vector3(3.48553f, -2.48553f, 0.5f));
  
  TestFrustumAabb(frustum, aabb, file);
}

// Sphere inside of frustum.
void FrustumSphereTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on right of frustum.
void FrustumSphereTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(6.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on left of frustum.
void FrustumSphereTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-6.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on top of frustum.
void FrustumSphereTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 6.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on bottom of frustum.
void FrustumSphereTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, -6.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere in front of frustum.
void FrustumSphereTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 0, 6), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere behind frustum.
void FrustumSphereTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 0, -6), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting right side of frustum.
void FrustumSphereTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(5.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting left side of frustum.
void FrustumSphereTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-5.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting top side of frustum.
void FrustumSphereTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting bottom side of frustum.
void FrustumSphereTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, -5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting front of frustum.
void FrustumSphereTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 0, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting back of frustum.
void FrustumSphereTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 0, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(5.33154f, 15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(5.33154f, -15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(5.33154f, 0, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(5.33154f, 0, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-5.33154f, 15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-5.33154f, -15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-5.33154f, 0, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-5.33154f, 0, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 5.33154f, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 5.33154f, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(15.9946f, 5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-15.9946f, 5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, -5.33154f, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, -5.33154f, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(15.9946f, -5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-15.9946f, -5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(15.9946f, 0, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-15.9946f, 0, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 15.9946f, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, -15.9946f, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(15.9946f, 0, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(-15.9946f, 0, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, 15.9946f, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, -7.66308f, 5), Vector3(7.66308f, -7.66308f, 5), Vector3(7.66308f, 7.66308f, 5), Vector3(-7.66308f, 7.66308f, 5), Vector3(-3, -3, -5), Vector3(3, -3, -5), Vector3(3, 3, -5), Vector3(-3, 3, -5));
  Sphere sphere(Vector3(0, -15.9946f, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere inside of frustum.
void FrustumSphereTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on right of frustum.
void FrustumSphereTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 6.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on left of frustum.
void FrustumSphereTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, -6.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on top of frustum.
void FrustumSphereTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 0, 6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on bottom of frustum.
void FrustumSphereTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 0, -6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere in front of frustum.
void FrustumSphereTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(6, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere behind frustum.
void FrustumSphereTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-6, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting right side of frustum.
void FrustumSphereTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting left side of frustum.
void FrustumSphereTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, -5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting top side of frustum.
void FrustumSphereTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting bottom side of frustum.
void FrustumSphereTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting front of frustum.
void FrustumSphereTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(5, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting back of frustum.
void FrustumSphereTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-5, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 5.33154f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 5.33154f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(15, 5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-15, 5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, -5.33154f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, -5.33154f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(15, -5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-15, -5.33154f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(15, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-15, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 15.9946f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, -15.9946f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(15, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-15, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, 15.9946f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(0, -15.9946f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(5, 15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(5, -15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(5, 0, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(5, 0, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-5, 15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-5, -15.9946f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-5, 0, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(5, -7.66308f, -7.66308f), Vector3(5, 7.66308f, -7.66308f), Vector3(5, 7.66308f, 7.66308f), Vector3(5, -7.66308f, 7.66308f), Vector3(-5, -3, -3), Vector3(-5, 3, -3), Vector3(-5, 3, 3), Vector3(-5, -3, 3));
  Sphere sphere(Vector3(-5, 0, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere inside of frustum.
void FrustumSphereTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on right of frustum.
void FrustumSphereTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 0, 6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on left of frustum.
void FrustumSphereTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 0, -6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on top of frustum.
void FrustumSphereTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(6.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on bottom of frustum.
void FrustumSphereTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-6.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere in front of frustum.
void FrustumSphereTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 6, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere behind frustum.
void FrustumSphereTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, -6, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting right side of frustum.
void FrustumSphereTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting left side of frustum.
void FrustumSphereTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting top side of frustum.
void FrustumSphereTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(5.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting bottom side of frustum.
void FrustumSphereTest85(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-5.33154f, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting front of frustum.
void FrustumSphereTest86(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 5, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting back of frustum.
void FrustumSphereTest87(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, -5, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest88(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(15.9946f, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest89(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-15.9946f, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest90(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 15, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest91(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, -15, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest92(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(15.9946f, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest93(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-15.9946f, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest94(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 15, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest95(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, -15, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest96(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(5.33154f, 15, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest97(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(5.33154f, -15, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest98(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(5.33154f, 0, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest99(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(5.33154f, 0, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest100(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-5.33154f, 15, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest101(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-5.33154f, -15, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest102(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-5.33154f, 0, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest103(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-5.33154f, 0, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest104(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 5, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest105(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, 5, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest106(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(15.9946f, 5, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest107(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-15.9946f, 5, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest108(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, -5, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest109(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(0, -5, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest110(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(15.9946f, -5, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest111(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-7.66308f, 5, -7.66308f), Vector3(-7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, 7.66308f), Vector3(7.66308f, 5, -7.66308f), Vector3(-3, -5, -3), Vector3(-3, -5, 3), Vector3(3, -5, 3), Vector3(3, -5, -3));
  Sphere sphere(Vector3(-15.9946f, -5, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere inside of frustum.
void FrustumSphereTest112(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on right of frustum.
void FrustumSphereTest113(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(4.76997f, 4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on left of frustum.
void FrustumSphereTest114(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-4.76997f, -4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on top of frustum.
void FrustumSphereTest115(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-4.76997f, 4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on bottom of frustum.
void FrustumSphereTest116(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(4.76997f, -4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere in front of frustum.
void FrustumSphereTest117(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(0, 0, 6), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere behind frustum.
void FrustumSphereTest118(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(0, 0, -6), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting right side of frustum.
void FrustumSphereTest119(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(3.76997f, 3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting left side of frustum.
void FrustumSphereTest120(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-3.76997f, -3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting top side of frustum.
void FrustumSphereTest121(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-3.76997f, 3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting bottom side of frustum.
void FrustumSphereTest122(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(3.76997f, -3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting front of frustum.
void FrustumSphereTest123(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(0, 0, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting back of frustum.
void FrustumSphereTest124(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(0, 0, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest125(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-7.53993f, 15.0799f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest126(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(15.0799f, -7.53993f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest127(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(3.76997f, 3.76997f, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest128(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(3.76997f, 3.76997f, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest129(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-15.0799f, 7.53993f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest130(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(7.53993f, -15.0799f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest131(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-3.76997f, -3.76997f, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest132(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-3.76997f, -3.76997f, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest133(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-3.76997f, 3.76997f, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest134(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-3.76997f, 3.76997f, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest135(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(7.53993f, 15.0799f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest136(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-15.0799f, -7.53993f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest137(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(3.76997f, -3.76997f, 15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest138(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(3.76997f, -3.76997f, -15), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest139(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(15.0799f, 7.53993f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest140(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-7.53993f, -15.0799f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest141(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(11.3099f, 11.3099f, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest142(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-11.3099f, -11.3099f, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest143(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-11.3099f, 11.3099f, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest144(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(11.3099f, -11.3099f, 5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest145(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(11.3099f, 11.3099f, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest146(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-11.3099f, -11.3099f, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest147(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(-11.3099f, 11.3099f, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest148(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(4.76837e-07f, -10.8372f, 5), Vector3(10.8372f, -4.76837e-07f, 5), Vector3(-4.76837e-07f, 10.8372f, 5), Vector3(-10.8372f, 4.76837e-07f, 5), Vector3(2.38419e-07f, -4.24264f, -5), Vector3(4.24264f, -2.38419e-07f, -5), Vector3(-2.38419e-07f, 4.24264f, -5), Vector3(-4.24264f, 2.38419e-07f, -5));
  Sphere sphere(Vector3(11.3099f, -11.3099f, -5), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere inside of frustum.
void FrustumSphereTest149(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on right of frustum.
void FrustumSphereTest150(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-4.76997f, 4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on left of frustum.
void FrustumSphereTest151(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(4.76997f, -4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on top of frustum.
void FrustumSphereTest152(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(0, 0, 6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on bottom of frustum.
void FrustumSphereTest153(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(0, 0, -6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere in front of frustum.
void FrustumSphereTest154(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(4.53553f, 4.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere behind frustum.
void FrustumSphereTest155(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-4.53553f, -4.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting right side of frustum.
void FrustumSphereTest156(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-3.76997f, 3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting left side of frustum.
void FrustumSphereTest157(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(3.76997f, -3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting top side of frustum.
void FrustumSphereTest158(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(0, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting bottom side of frustum.
void FrustumSphereTest159(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(0, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting front of frustum.
void FrustumSphereTest160(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(3.53553f, 3.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting back of frustum.
void FrustumSphereTest161(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-3.53553f, -3.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest162(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-3.76997f, 3.76997f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest163(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-3.76997f, 3.76997f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest164(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(6.83663f, 14.3766f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest165(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-14.3766f, -6.83663f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest166(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(3.76997f, -3.76997f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest167(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(3.76997f, -3.76997f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest168(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(14.3766f, 6.83663f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest169(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-6.83663f, -14.3766f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest170(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(10.6066f, 10.6066f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest171(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-10.6066f, -10.6066f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest172(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-11.3099f, 11.3099f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest173(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(11.3099f, -11.3099f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest174(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(10.6066f, 10.6066f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest175(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-10.6066f, -10.6066f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest176(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-11.3099f, 11.3099f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest177(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(11.3099f, -11.3099f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest178(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-7.77437f, 14.8454f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest179(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(14.8454f, -7.77437f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest180(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(3.53553f, 3.53553f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest181(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(3.53553f, 3.53553f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest182(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-14.8454f, 7.77437f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest183(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(7.77437f, -14.8454f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest184(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-3.53553f, -3.53553f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest185(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(8.95415f, -1.88308f, -7.66308f), Vector3(-1.88308f, 8.95415f, -7.66308f), Vector3(-1.88308f, 8.95415f, 7.66308f), Vector3(8.95415f, -1.88308f, 7.66308f), Vector3(-1.41421f, -5.65685f, -3), Vector3(-5.65685f, -1.41421f, -3), Vector3(-5.65685f, -1.41421f, 3), Vector3(-1.41421f, -5.65685f, 3));
  Sphere sphere(Vector3(-3.53553f, -3.53553f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere inside of frustum.
void FrustumSphereTest186(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(0, 0, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on right of frustum.
void FrustumSphereTest187(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(0, 0, 6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on left of frustum.
void FrustumSphereTest188(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(0, 0, -6.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on top of frustum.
void FrustumSphereTest189(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(4.76997f, 4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere on bottom of frustum.
void FrustumSphereTest190(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-4.76997f, -4.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere in front of frustum.
void FrustumSphereTest191(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-4.53553f, 4.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere behind frustum.
void FrustumSphereTest192(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(4.53553f, -4.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting right side of frustum.
void FrustumSphereTest193(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(0, 0, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting left side of frustum.
void FrustumSphereTest194(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(0, 0, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting top side of frustum.
void FrustumSphereTest195(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(3.76997f, 3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting bottom side of frustum.
void FrustumSphereTest196(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-3.76997f, -3.76997f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting front of frustum.
void FrustumSphereTest197(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-3.53553f, 3.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting back of frustum.
void FrustumSphereTest198(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(3.53553f, -3.53553f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest199(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(11.3099f, 11.3099f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest200(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-11.3099f, -11.3099f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest201(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-10.6066f, 10.6066f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest202(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(10.6066f, -10.6066f, 5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest203(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(11.3099f, 11.3099f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest204(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-11.3099f, -11.3099f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest205(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-10.6066f, 10.6066f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest206(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(10.6066f, -10.6066f, -5.33154f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest207(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-6.83664f, 14.3766f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest208(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(14.3766f, -6.83664f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest209(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(3.76997f, 3.76997f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest210(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(3.76997f, 3.76997f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest211(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-14.3766f, 6.83664f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest212(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(6.83664f, -14.3766f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest213(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-3.76997f, -3.76997f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest214(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-3.76997f, -3.76997f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest215(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-3.53553f, 3.53553f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest216(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-3.53553f, 3.53553f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest217(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(7.77437f, 14.8454f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest218(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-14.8454f, -7.77437f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest219(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(3.53553f, -3.53553f, 15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest220(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(3.53553f, -3.53553f, -15.9946f), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest221(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(14.8454f, 7.77437f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}

// Sphere intersecting one plane but outside another.
void FrustumSphereTest222(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Frustum frustum;
  frustum.Set(Vector3(-8.95415f, -1.88308f, -7.66308f), Vector3(-8.95415f, -1.88308f, 7.66308f), Vector3(1.88308f, 8.95415f, 7.66308f), Vector3(1.88308f, 8.95415f, -7.66308f), Vector3(1.41421f, -5.65685f, -3), Vector3(1.41421f, -5.65685f, 3), Vector3(5.65685f, -1.41421f, 3), Vector3(5.65685f, -1.41421f, -3));
  Sphere sphere(Vector3(-7.77437f, -14.8454f, 0), 0.5f);
  
  TestFrustumSphere(frustum, sphere, file);
}


// Spheres separating
void SphereSphereTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(-2, 0, 0), 1);
  Sphere sphere1(Vector3(2, 0, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres separating
void SphereSphereTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(-1.5f, 0, 0), 1);
  Sphere sphere1(Vector3(1.5f, 0, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres just touching
void SphereSphereTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(-1, 0, 0), 1);
  Sphere sphere1(Vector3(1, 0, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres overlapping a bit
void SphereSphereTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(-0.5f, 0, 0), 1);
  Sphere sphere1(Vector3(0.5f, 0, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres at the same center
void SphereSphereTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, 0, 0), 1);
  Sphere sphere1(Vector3(0, 0, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres separating
void SphereSphereTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, -2, 0), 1);
  Sphere sphere1(Vector3(0, 2, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres separating
void SphereSphereTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, -1.5f, 0), 1);
  Sphere sphere1(Vector3(0, 1.5f, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres just touching
void SphereSphereTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, -1, 0), 1);
  Sphere sphere1(Vector3(0, 1, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres overlapping a bit
void SphereSphereTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, -0.5f, 0), 1);
  Sphere sphere1(Vector3(0, 0.5f, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres at the same center
void SphereSphereTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, 0, 0), 1);
  Sphere sphere1(Vector3(0, 0, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres separating
void SphereSphereTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, 0, -2), 1);
  Sphere sphere1(Vector3(0, 0, 2), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres separating
void SphereSphereTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, 0, -1.5f), 1);
  Sphere sphere1(Vector3(0, 0, 1.5f), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres just touching
void SphereSphereTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, 0, -1), 1);
  Sphere sphere1(Vector3(0, 0, 1), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres overlapping a bit
void SphereSphereTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, 0, -0.5f), 1);
  Sphere sphere1(Vector3(0, 0, 0.5f), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}

// Spheres at the same center
void SphereSphereTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Sphere sphere0(Vector3(0, 0, 0), 1);
  Sphere sphere1(Vector3(0, 0, 0), 1);
  
  TestSphereSphere(sphere0, sphere1, file);
}


// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1.5f, -0.5f, -0.5f), Vector3(-0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(0.5f, -0.5f, -0.5f), Vector3(1.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0.5f, -0.5f, -0.5f), Vector3(1.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-1.5f, -0.5f, -0.5f), Vector3(-0.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1, -0.5f, -0.5f), Vector3(0, 0.5f, 0.5f));
  Aabb aabb1(Vector3(0, -0.5f, -0.5f), Vector3(1, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0, -0.5f, -0.5f), Vector3(1, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-1, -0.5f, -0.5f), Vector3(0, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.75f, -0.5f, -0.5f), Vector3(0.25f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.25f, -0.5f, -0.5f), Vector3(0.75f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.25f, -0.5f, -0.5f), Vector3(0.75f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.75f, -0.5f, -0.5f), Vector3(0.25f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -1.5f, -0.5f), Vector3(0.5f, -0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, 0.5f, -0.5f), Vector3(0.5f, 1.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, 0.5f, -0.5f), Vector3(0.5f, 1.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -1.5f, -0.5f), Vector3(0.5f, -0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -1, -0.5f), Vector3(0.5f, 0, 0.5f));
  Aabb aabb1(Vector3(-0.5f, 0, -0.5f), Vector3(0.5f, 1, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, 0, -0.5f), Vector3(0.5f, 1, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -1, -0.5f), Vector3(0.5f, 0, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.75f, -0.5f), Vector3(0.5f, 0.25f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.25f, -0.5f), Vector3(0.5f, 0.75f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.25f, -0.5f), Vector3(0.5f, 0.75f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.75f, -0.5f), Vector3(0.5f, 0.25f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -1.5f), Vector3(0.5f, 0.5f, -0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, 0.5f), Vector3(0.5f, 0.5f, 1.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, 0.5f), Vector3(0.5f, 0.5f, 1.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -1.5f), Vector3(0.5f, 0.5f, -0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -1), Vector3(0.5f, 0.5f, 0));
  Aabb aabb1(Vector3(-0.5f, -0.5f, 0), Vector3(0.5f, 0.5f, 1));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, 0), Vector3(0.5f, 0.5f, 1));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -1), Vector3(0.5f, 0.5f, 0));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.75f), Vector3(0.5f, 0.5f, 0.25f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.25f), Vector3(0.5f, 0.5f, 0.75f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.25f), Vector3(0.5f, 0.5f, 0.75f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.75f), Vector3(0.5f, 0.5f, 0.25f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1.5f, -1.5f, -0.5f), Vector3(-0.5f, -0.5f, 0.5f));
  Aabb aabb1(Vector3(0.5f, 0.5f, -0.5f), Vector3(1.5f, 1.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0.5f, 0.5f, -0.5f), Vector3(1.5f, 1.5f, 0.5f));
  Aabb aabb1(Vector3(-1.5f, -1.5f, -0.5f), Vector3(-0.5f, -0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1, -1, -0.5f), Vector3(2.98023e-08f, 2.98023e-08f, 0.5f));
  Aabb aabb1(Vector3(-2.98023e-08f, -2.98023e-08f, -0.5f), Vector3(1, 1, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-2.98023e-08f, -2.98023e-08f, -0.5f), Vector3(1, 1, 0.5f));
  Aabb aabb1(Vector3(-1, -1, -0.5f), Vector3(2.98023e-08f, 2.98023e-08f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.75f, -0.75f, -0.5f), Vector3(0.25f, 0.25f, 0.5f));
  Aabb aabb1(Vector3(-0.25f, -0.25f, -0.5f), Vector3(0.75f, 0.75f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.25f, -0.25f, -0.5f), Vector3(0.75f, 0.75f, 0.5f));
  Aabb aabb1(Vector3(-0.75f, -0.75f, -0.5f), Vector3(0.25f, 0.25f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1.5f, -0.5f, -1.5f), Vector3(-0.5f, 0.5f, -0.5f));
  Aabb aabb1(Vector3(0.5f, -0.5f, 0.5f), Vector3(1.5f, 0.5f, 1.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0.5f, -0.5f, 0.5f), Vector3(1.5f, 0.5f, 1.5f));
  Aabb aabb1(Vector3(-1.5f, -0.5f, -1.5f), Vector3(-0.5f, 0.5f, -0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1, -0.5f, -1), Vector3(2.98023e-08f, 0.5f, 2.98023e-08f));
  Aabb aabb1(Vector3(-2.98023e-08f, -0.5f, -2.98023e-08f), Vector3(1, 0.5f, 1));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-2.98023e-08f, -0.5f, -2.98023e-08f), Vector3(1, 0.5f, 1));
  Aabb aabb1(Vector3(-1, -0.5f, -1), Vector3(2.98023e-08f, 0.5f, 2.98023e-08f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.75f, -0.5f, -0.75f), Vector3(0.25f, 0.5f, 0.25f));
  Aabb aabb1(Vector3(-0.25f, -0.5f, -0.25f), Vector3(0.75f, 0.5f, 0.75f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.25f, -0.5f, -0.25f), Vector3(0.75f, 0.5f, 0.75f));
  Aabb aabb1(Vector3(-0.75f, -0.5f, -0.75f), Vector3(0.25f, 0.5f, 0.25f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -1.5f, -1.5f), Vector3(0.5f, -0.5f, -0.5f));
  Aabb aabb1(Vector3(-0.5f, 0.5f, 0.5f), Vector3(0.5f, 1.5f, 1.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, 0.5f, 0.5f), Vector3(0.5f, 1.5f, 1.5f));
  Aabb aabb1(Vector3(-0.5f, -1.5f, -1.5f), Vector3(0.5f, -0.5f, -0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -1, -1), Vector3(0.5f, 2.98023e-08f, 2.98023e-08f));
  Aabb aabb1(Vector3(-0.5f, -2.98023e-08f, -2.98023e-08f), Vector3(0.5f, 1, 1));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -2.98023e-08f, -2.98023e-08f), Vector3(0.5f, 1, 1));
  Aabb aabb1(Vector3(-0.5f, -1, -1), Vector3(0.5f, 2.98023e-08f, 2.98023e-08f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.75f, -0.75f), Vector3(0.5f, 0.25f, 0.25f));
  Aabb aabb1(Vector3(-0.5f, -0.25f, -0.25f), Vector3(0.5f, 0.75f, 0.75f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.25f, -0.25f), Vector3(0.5f, 0.75f, 0.75f));
  Aabb aabb1(Vector3(-0.5f, -0.75f, -0.75f), Vector3(0.5f, 0.25f, 0.25f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1.5f, -0.5f, -0.1f), Vector3(-0.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(0.5f, -0.5f, -0.1f), Vector3(1.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0.5f, -0.5f, -0.1f), Vector3(1.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-1.5f, -0.5f, -0.1f), Vector3(-0.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1, -0.5f, -0.1f), Vector3(0, 0.5f, 0.1f));
  Aabb aabb1(Vector3(0, -0.5f, -0.1f), Vector3(1, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0, -0.5f, -0.1f), Vector3(1, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-1, -0.5f, -0.1f), Vector3(0, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.75f, -0.5f, -0.1f), Vector3(0.25f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.25f, -0.5f, -0.1f), Vector3(0.75f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.25f, -0.5f, -0.1f), Vector3(0.75f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.75f, -0.5f, -0.1f), Vector3(0.25f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -1.5f, -0.1f), Vector3(0.5f, -0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, 0.5f, -0.1f), Vector3(0.5f, 1.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, 0.5f, -0.1f), Vector3(0.5f, 1.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -1.5f, -0.1f), Vector3(0.5f, -0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -1, -0.1f), Vector3(0.5f, 0, 0.1f));
  Aabb aabb1(Vector3(-0.5f, 0, -0.1f), Vector3(0.5f, 1, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, 0, -0.1f), Vector3(0.5f, 1, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -1, -0.1f), Vector3(0.5f, 0, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.75f, -0.1f), Vector3(0.5f, 0.25f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.25f, -0.1f), Vector3(0.5f, 0.75f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.25f, -0.1f), Vector3(0.5f, 0.75f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.75f, -0.1f), Vector3(0.5f, 0.25f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.3f), Vector3(0.5f, 0.5f, -0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, 0.1f), Vector3(0.5f, 0.5f, 0.3f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, 0.1f), Vector3(0.5f, 0.5f, 0.3f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.3f), Vector3(0.5f, 0.5f, -0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.2f), Vector3(0.5f, 0.5f, 0));
  Aabb aabb1(Vector3(-0.5f, -0.5f, 0), Vector3(0.5f, 0.5f, 0.2f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, 0), Vector3(0.5f, 0.5f, 0.2f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.2f), Vector3(0.5f, 0.5f, 0));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.15f), Vector3(0.5f, 0.5f, 0.05f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.05f), Vector3(0.5f, 0.5f, 0.15f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.05f), Vector3(0.5f, 0.5f, 0.15f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.15f), Vector3(0.5f, 0.5f, 0.05f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1.5f, -1.5f, -0.1f), Vector3(-0.5f, -0.5f, 0.1f));
  Aabb aabb1(Vector3(0.5f, 0.5f, -0.1f), Vector3(1.5f, 1.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0.5f, 0.5f, -0.1f), Vector3(1.5f, 1.5f, 0.1f));
  Aabb aabb1(Vector3(-1.5f, -1.5f, -0.1f), Vector3(-0.5f, -0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1, -1, -0.1f), Vector3(2.98023e-08f, 2.98023e-08f, 0.1f));
  Aabb aabb1(Vector3(-2.98023e-08f, -2.98023e-08f, -0.1f), Vector3(1, 1, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-2.98023e-08f, -2.98023e-08f, -0.1f), Vector3(1, 1, 0.1f));
  Aabb aabb1(Vector3(-1, -1, -0.1f), Vector3(2.98023e-08f, 2.98023e-08f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.75f, -0.75f, -0.1f), Vector3(0.25f, 0.25f, 0.1f));
  Aabb aabb1(Vector3(-0.25f, -0.25f, -0.1f), Vector3(0.75f, 0.75f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.25f, -0.25f, -0.1f), Vector3(0.75f, 0.75f, 0.1f));
  Aabb aabb1(Vector3(-0.75f, -0.75f, -0.1f), Vector3(0.25f, 0.25f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1.1f, -0.5f, -0.7f), Vector3(-0.1f, 0.5f, -0.5f));
  Aabb aabb1(Vector3(0.1f, -0.5f, 0.5f), Vector3(1.1f, 0.5f, 0.7f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0.1f, -0.5f, 0.5f), Vector3(1.1f, 0.5f, 0.7f));
  Aabb aabb1(Vector3(-1.1f, -0.5f, -0.7f), Vector3(-0.1f, 0.5f, -0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.8f, -0.5f, -0.4f), Vector3(0.2f, 0.5f, -0.2f));
  Aabb aabb1(Vector3(-0.2f, -0.5f, 0.2f), Vector3(0.8f, 0.5f, 0.4f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.2f, -0.5f, 0.2f), Vector3(0.8f, 0.5f, 0.4f));
  Aabb aabb1(Vector3(-0.8f, -0.5f, -0.4f), Vector3(0.2f, 0.5f, -0.2f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.65f, -0.5f, -0.25f), Vector3(0.35f, 0.5f, -0.05f));
  Aabb aabb1(Vector3(-0.35f, -0.5f, 0.05f), Vector3(0.65f, 0.5f, 0.25f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.35f, -0.5f, 0.05f), Vector3(0.65f, 0.5f, 0.25f));
  Aabb aabb1(Vector3(-0.65f, -0.5f, -0.25f), Vector3(0.35f, 0.5f, -0.05f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb0 behind aabb1)
void AabbAabbTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -1.1f, -0.7f), Vector3(0.5f, -0.1f, -0.5f));
  Aabb aabb1(Vector3(-0.5f, 0.1f, 0.5f), Vector3(0.5f, 1.1f, 0.7f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs separating (with aabb1 behind aabb0)
void AabbAabbTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, 0.1f, 0.5f), Vector3(0.5f, 1.1f, 0.7f));
  Aabb aabb1(Vector3(-0.5f, -1.1f, -0.7f), Vector3(0.5f, -0.1f, -0.5f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb0 behind aabb1)
void AabbAabbTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.8f, -0.4f), Vector3(0.5f, 0.2f, -0.2f));
  Aabb aabb1(Vector3(-0.5f, -0.2f, 0.2f), Vector3(0.5f, 0.8f, 0.4f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs just touching (with aabb1 behind aabb0)
void AabbAabbTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.2f, 0.2f), Vector3(0.5f, 0.8f, 0.4f));
  Aabb aabb1(Vector3(-0.5f, -0.8f, -0.4f), Vector3(0.5f, 0.2f, -0.2f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb0 behind aabb1)
void AabbAabbTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.65f, -0.25f), Vector3(0.5f, 0.35f, -0.05f));
  Aabb aabb1(Vector3(-0.5f, -0.35f, 0.05f), Vector3(0.5f, 0.65f, 0.25f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs overlapping (with aabb1 behind aabb0)
void AabbAabbTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.35f, 0.05f), Vector3(0.5f, 0.65f, 0.25f));
  Aabb aabb1(Vector3(-0.5f, -0.65f, -0.25f), Vector3(0.5f, 0.35f, -0.05f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

// Aabbs at the same center
void AabbAabbTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.1f), Vector3(0.5f, 0.5f, 0.1f));
  
  TestAabbAabb(aabb0, aabb1, file);
}

void RegisterProjectPointOnPlaneTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest1, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest2, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest3, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest4, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest5, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest6, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest7, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest8, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest9, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest10, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest11, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest12, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest13, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest14, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest15, list);
  DeclareSimpleUnitTest(ProjectPointOnPlaneTest16, list);
}

void RegisterBarycentricLineTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BarycentricLineTest0, list);
  DeclareSimpleUnitTest(BarycentricLineTest1, list);
  DeclareSimpleUnitTest(BarycentricLineTest2, list);
  DeclareSimpleUnitTest(BarycentricLineTest3, list);
  DeclareSimpleUnitTest(BarycentricLineTest4, list);
  DeclareSimpleUnitTest(BarycentricLineTest5, list);
  DeclareSimpleUnitTest(BarycentricLineTest6, list);
  DeclareSimpleUnitTest(BarycentricLineTest7, list);
  DeclareSimpleUnitTest(BarycentricLineTest8, list);
  DeclareSimpleUnitTest(BarycentricLineTest9, list);
  DeclareSimpleUnitTest(BarycentricLineTest10, list);
  DeclareSimpleUnitTest(BarycentricLineTest11, list);
  DeclareSimpleUnitTest(BarycentricLineTest12, list);
  DeclareSimpleUnitTest(BarycentricLineTest13, list);
  DeclareSimpleUnitTest(BarycentricLineTest14, list);
  DeclareSimpleUnitTest(BarycentricLineTest15, list);
  DeclareSimpleUnitTest(BarycentricLineTest16, list);
  DeclareSimpleUnitTest(BarycentricLineTest17, list);
  DeclareSimpleUnitTest(BarycentricLineTest18, list);
  DeclareSimpleUnitTest(BarycentricLineTest19, list);
  DeclareSimpleUnitTest(BarycentricLineTest20, list);
  DeclareSimpleUnitTest(BarycentricLineTest21, list);
  DeclareSimpleUnitTest(BarycentricLineTest22, list);
  DeclareSimpleUnitTest(BarycentricLineTest23, list);
  DeclareSimpleUnitTest(BarycentricLineTest24, list);
  DeclareSimpleUnitTest(BarycentricLineTest25, list);
  DeclareSimpleUnitTest(BarycentricLineTest26, list);
  DeclareSimpleUnitTest(BarycentricLineTest27, list);
  DeclareSimpleUnitTest(BarycentricLineTest28, list);
  DeclareSimpleUnitTest(BarycentricLineTest29, list);
  DeclareSimpleUnitTest(BarycentricLineTest30, list);
  DeclareSimpleUnitTest(BarycentricLineTest31, list);
  DeclareSimpleUnitTest(BarycentricLineTest32, list);
  DeclareSimpleUnitTest(BarycentricLineTest33, list);
  DeclareSimpleUnitTest(BarycentricLineTest34, list);
  DeclareSimpleUnitTest(BarycentricLineTest35, list);
  DeclareSimpleUnitTest(BarycentricLineTest36, list);
  DeclareSimpleUnitTest(BarycentricLineTest37, list);
  DeclareSimpleUnitTest(BarycentricLineTest38, list);
  DeclareSimpleUnitTest(BarycentricLineTest39, list);
  DeclareSimpleUnitTest(BarycentricLineTest40, list);
  DeclareSimpleUnitTest(BarycentricLineTest41, list);
  DeclareSimpleUnitTest(BarycentricLineTest42, list);
  DeclareSimpleUnitTest(BarycentricLineTest43, list);
  DeclareSimpleUnitTest(BarycentricLineTest44, list);
  DeclareSimpleUnitTest(BarycentricLineTest45, list);
  DeclareSimpleUnitTest(BarycentricLineTest46, list);
  DeclareSimpleUnitTest(BarycentricLineTest47, list);
  DeclareSimpleUnitTest(BarycentricLineTest48, list);
  DeclareSimpleUnitTest(BarycentricLineTest49, list);
  DeclareSimpleUnitTest(BarycentricLineTest50, list);
  DeclareSimpleUnitTest(BarycentricLineTest51, list);
  DeclareSimpleUnitTest(BarycentricLineTest52, list);
  DeclareSimpleUnitTest(BarycentricLineTest53, list);
  DeclareSimpleUnitTest(BarycentricLineTest54, list);
  DeclareSimpleUnitTest(BarycentricLineTest55, list);
  DeclareSimpleUnitTest(BarycentricLineTest56, list);
}

void RegisterBarycentricTriangleTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BarycentricTriangleTest0, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest1, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest2, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest3, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest4, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest5, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest6, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest7, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest8, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest9, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest10, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest11, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest12, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest13, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest14, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest15, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest16, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest17, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest18, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest19, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest20, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest21, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest22, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest23, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest24, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest25, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest26, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest27, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest28, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest29, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest30, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest31, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest32, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest33, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest34, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest35, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest36, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest37, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest38, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest39, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest40, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest41, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest42, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest43, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest44, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest45, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest46, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest47, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest48, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest49, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest50, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest51, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest52, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest53, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest54, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest55, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest56, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest57, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest58, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest59, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest60, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest61, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest62, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest63, list);
  DeclareSimpleUnitTest(BarycentricTriangleTest64, list);
}

void RegisterPlaneSetTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PlaneSetTest1, list);
  DeclareSimpleUnitTest(PlaneSetTest2, list);
  DeclareSimpleUnitTest(PlaneSetTest3, list);
  DeclareSimpleUnitTest(PlaneSetTest4, list);
}

void RegisterPointPlaneTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PointPlaneTest1, list);
  DeclareSimpleUnitTest(PointPlaneTest2, list);
  DeclareSimpleUnitTest(PointPlaneTest3, list);
  DeclareSimpleUnitTest(PointPlaneTest4, list);
  DeclareSimpleUnitTest(PointPlaneTest5, list);
  DeclareSimpleUnitTest(PointPlaneTest6, list);
  DeclareSimpleUnitTest(PointPlaneTest7, list);
  DeclareSimpleUnitTest(PointPlaneTest8, list);
  DeclareSimpleUnitTest(PointPlaneTest9, list);
  DeclareSimpleUnitTest(PointPlaneTest10, list);
  DeclareSimpleUnitTest(PointPlaneTest11, list);
  DeclareSimpleUnitTest(PointPlaneTest12, list);
  DeclareSimpleUnitTest(PointPlaneTest13, list);
  DeclareSimpleUnitTest(PointPlaneTest14, list);
  DeclareSimpleUnitTest(PointPlaneTest15, list);
  DeclareSimpleUnitTest(PointPlaneTest16, list);
  DeclareSimpleUnitTest(PointPlaneTest17, list);
  DeclareSimpleUnitTest(PointPlaneTest18, list);
  DeclareSimpleUnitTest(PointPlaneTest19, list);
  DeclareSimpleUnitTest(PointPlaneTest20, list);
  DeclareSimpleUnitTest(PointPlaneTest21, list);
  DeclareSimpleUnitTest(PointPlaneTest22, list);
  DeclareSimpleUnitTest(PointPlaneTest23, list);
  DeclareSimpleUnitTest(PointPlaneTest24, list);
  DeclareSimpleUnitTest(PointPlaneTest25, list);
  DeclareSimpleUnitTest(PointPlaneTest26, list);
  DeclareSimpleUnitTest(PointPlaneTest27, list);
  DeclareSimpleUnitTest(PointPlaneTest28, list);
  DeclareSimpleUnitTest(PointPlaneTest29, list);
  DeclareSimpleUnitTest(PointPlaneTest30, list);
  DeclareSimpleUnitTest(PointPlaneTest31, list);
  DeclareSimpleUnitTest(PointPlaneTest32, list);
  DeclareSimpleUnitTest(PointPlaneTest33, list);
  DeclareSimpleUnitTest(PointPlaneTest34, list);
  DeclareSimpleUnitTest(PointPlaneTest35, list);
  DeclareSimpleUnitTest(PointPlaneTest36, list);
  DeclareSimpleUnitTest(PointPlaneTest37, list);
  DeclareSimpleUnitTest(PointPlaneTest38, list);
  DeclareSimpleUnitTest(PointPlaneTest39, list);
  DeclareSimpleUnitTest(PointPlaneTest40, list);
}

void RegisterPointSphereTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PointSphereTest1, list);
  DeclareSimpleUnitTest(PointSphereTest2, list);
  DeclareSimpleUnitTest(PointSphereTest3, list);
  DeclareSimpleUnitTest(PointSphereTest4, list);
  DeclareSimpleUnitTest(PointSphereTest5, list);
  DeclareSimpleUnitTest(PointSphereTest6, list);
  DeclareSimpleUnitTest(PointSphereTest7, list);
  DeclareSimpleUnitTest(PointSphereTest8, list);
  DeclareSimpleUnitTest(PointSphereTest9, list);
  DeclareSimpleUnitTest(PointSphereTest10, list);
  DeclareSimpleUnitTest(PointSphereTest11, list);
  DeclareSimpleUnitTest(PointSphereTest12, list);
  DeclareSimpleUnitTest(PointSphereTest13, list);
  DeclareSimpleUnitTest(PointSphereTest14, list);
  DeclareSimpleUnitTest(PointSphereTest15, list);
  DeclareSimpleUnitTest(PointSphereTest16, list);
  DeclareSimpleUnitTest(PointSphereTest17, list);
  DeclareSimpleUnitTest(PointSphereTest18, list);
  DeclareSimpleUnitTest(PointSphereTest19, list);
  DeclareSimpleUnitTest(PointSphereTest20, list);
  DeclareSimpleUnitTest(PointSphereTest21, list);
  DeclareSimpleUnitTest(PointSphereTest22, list);
  DeclareSimpleUnitTest(PointSphereTest23, list);
  DeclareSimpleUnitTest(PointSphereTest24, list);
  DeclareSimpleUnitTest(PointSphereTest25, list);
  DeclareSimpleUnitTest(PointSphereTest26, list);
}

void RegisterPointAabbTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PointAabbTest1, list);
  DeclareSimpleUnitTest(PointAabbTest2, list);
  DeclareSimpleUnitTest(PointAabbTest3, list);
  DeclareSimpleUnitTest(PointAabbTest4, list);
  DeclareSimpleUnitTest(PointAabbTest5, list);
  DeclareSimpleUnitTest(PointAabbTest6, list);
  DeclareSimpleUnitTest(PointAabbTest7, list);
  DeclareSimpleUnitTest(PointAabbTest8, list);
  DeclareSimpleUnitTest(PointAabbTest9, list);
  DeclareSimpleUnitTest(PointAabbTest10, list);
  DeclareSimpleUnitTest(PointAabbTest11, list);
  DeclareSimpleUnitTest(PointAabbTest12, list);
  DeclareSimpleUnitTest(PointAabbTest13, list);
  DeclareSimpleUnitTest(PointAabbTest14, list);
  DeclareSimpleUnitTest(PointAabbTest15, list);
  DeclareSimpleUnitTest(PointAabbTest16, list);
  DeclareSimpleUnitTest(PointAabbTest17, list);
  DeclareSimpleUnitTest(PointAabbTest18, list);
  DeclareSimpleUnitTest(PointAabbTest19, list);
  DeclareSimpleUnitTest(PointAabbTest20, list);
  DeclareSimpleUnitTest(PointAabbTest21, list);
  DeclareSimpleUnitTest(PointAabbTest22, list);
  DeclareSimpleUnitTest(PointAabbTest23, list);
  DeclareSimpleUnitTest(PointAabbTest24, list);
  DeclareSimpleUnitTest(PointAabbTest25, list);
  DeclareSimpleUnitTest(PointAabbTest26, list);
  DeclareSimpleUnitTest(PointAabbTest27, list);
  DeclareSimpleUnitTest(PointAabbTest28, list);
  DeclareSimpleUnitTest(PointAabbTest29, list);
  DeclareSimpleUnitTest(PointAabbTest30, list);
  DeclareSimpleUnitTest(PointAabbTest31, list);
  DeclareSimpleUnitTest(PointAabbTest32, list);
  DeclareSimpleUnitTest(PointAabbTest33, list);
  DeclareSimpleUnitTest(PointAabbTest34, list);
  DeclareSimpleUnitTest(PointAabbTest35, list);
  DeclareSimpleUnitTest(PointAabbTest36, list);
  DeclareSimpleUnitTest(PointAabbTest37, list);
  DeclareSimpleUnitTest(PointAabbTest38, list);
  DeclareSimpleUnitTest(PointAabbTest39, list);
  DeclareSimpleUnitTest(PointAabbTest40, list);
  DeclareSimpleUnitTest(PointAabbTest41, list);
  DeclareSimpleUnitTest(PointAabbTest42, list);
  DeclareSimpleUnitTest(PointAabbTest43, list);
  DeclareSimpleUnitTest(PointAabbTest44, list);
  DeclareSimpleUnitTest(PointAabbTest45, list);
  DeclareSimpleUnitTest(PointAabbTest46, list);
  DeclareSimpleUnitTest(PointAabbTest47, list);
  DeclareSimpleUnitTest(PointAabbTest48, list);
  DeclareSimpleUnitTest(PointAabbTest49, list);
  DeclareSimpleUnitTest(PointAabbTest50, list);
  DeclareSimpleUnitTest(PointAabbTest51, list);
  DeclareSimpleUnitTest(PointAabbTest52, list);
  DeclareSimpleUnitTest(PointAabbTest53, list);
  DeclareSimpleUnitTest(PointAabbTest54, list);
  DeclareSimpleUnitTest(PointAabbTest55, list);
  DeclareSimpleUnitTest(PointAabbTest56, list);
  DeclareSimpleUnitTest(PointAabbTest57, list);
  DeclareSimpleUnitTest(PointAabbTest58, list);
  DeclareSimpleUnitTest(PointAabbTest59, list);
  DeclareSimpleUnitTest(PointAabbTest60, list);
  DeclareSimpleUnitTest(PointAabbTest61, list);
  DeclareSimpleUnitTest(PointAabbTest62, list);
  DeclareSimpleUnitTest(PointAabbTest63, list);
  DeclareSimpleUnitTest(PointAabbTest64, list);
  DeclareSimpleUnitTest(PointAabbTest65, list);
  DeclareSimpleUnitTest(PointAabbTest66, list);
  DeclareSimpleUnitTest(PointAabbTest67, list);
  DeclareSimpleUnitTest(PointAabbTest68, list);
  DeclareSimpleUnitTest(PointAabbTest69, list);
  DeclareSimpleUnitTest(PointAabbTest70, list);
  DeclareSimpleUnitTest(PointAabbTest71, list);
  DeclareSimpleUnitTest(PointAabbTest72, list);
  DeclareSimpleUnitTest(PointAabbTest73, list);
  DeclareSimpleUnitTest(PointAabbTest74, list);
  DeclareSimpleUnitTest(PointAabbTest75, list);
  DeclareSimpleUnitTest(PointAabbTest76, list);
  DeclareSimpleUnitTest(PointAabbTest77, list);
  DeclareSimpleUnitTest(PointAabbTest78, list);
  DeclareSimpleUnitTest(PointAabbTest79, list);
  DeclareSimpleUnitTest(PointAabbTest80, list);
  DeclareSimpleUnitTest(PointAabbTest81, list);
  DeclareSimpleUnitTest(PointAabbTest82, list);
  DeclareSimpleUnitTest(PointAabbTest83, list);
  DeclareSimpleUnitTest(PointAabbTest84, list);
}

void RegisterRayPlaneTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(RayPlaneTest1, list);
  DeclareSimpleUnitTest(RayPlaneTest2, list);
  DeclareSimpleUnitTest(RayPlaneTest3, list);
  DeclareSimpleUnitTest(RayPlaneTest4, list);
  DeclareSimpleUnitTest(RayPlaneTest5, list);
  DeclareSimpleUnitTest(RayPlaneTest6, list);
  DeclareSimpleUnitTest(RayPlaneTest7, list);
  DeclareSimpleUnitTest(RayPlaneTest8, list);
  DeclareSimpleUnitTest(RayPlaneTest9, list);
  DeclareSimpleUnitTest(RayPlaneTest10, list);
  DeclareSimpleUnitTest(RayPlaneTest11, list);
  DeclareSimpleUnitTest(RayPlaneTest12, list);
  DeclareSimpleUnitTest(RayPlaneTest13, list);
  DeclareSimpleUnitTest(RayPlaneTest14, list);
  DeclareSimpleUnitTest(RayPlaneTest15, list);
  DeclareSimpleUnitTest(RayPlaneTest16, list);
  DeclareSimpleUnitTest(RayPlaneTest17, list);
  DeclareSimpleUnitTest(RayPlaneTest18, list);
  DeclareSimpleUnitTest(RayPlaneTest19, list);
  DeclareSimpleUnitTest(RayPlaneTest20, list);
  DeclareSimpleUnitTest(RayPlaneTest21, list);
  DeclareSimpleUnitTest(RayPlaneTest22, list);
  DeclareSimpleUnitTest(RayPlaneTest23, list);
  DeclareSimpleUnitTest(RayPlaneTest24, list);
  DeclareSimpleUnitTest(RayPlaneTest25, list);
  DeclareSimpleUnitTest(RayPlaneTest26, list);
  DeclareSimpleUnitTest(RayPlaneTest27, list);
  DeclareSimpleUnitTest(RayPlaneTest28, list);
  DeclareSimpleUnitTest(RayPlaneTest29, list);
  DeclareSimpleUnitTest(RayPlaneTest30, list);
  DeclareSimpleUnitTest(RayPlaneTest31, list);
  DeclareSimpleUnitTest(RayPlaneTest32, list);
  DeclareSimpleUnitTest(RayPlaneTest33, list);
  DeclareSimpleUnitTest(RayPlaneTest34, list);
  DeclareSimpleUnitTest(RayPlaneTest35, list);
  DeclareSimpleUnitTest(RayPlaneTest36, list);
  DeclareSimpleUnitTest(RayPlaneTest37, list);
  DeclareSimpleUnitTest(RayPlaneTest38, list);
  DeclareSimpleUnitTest(RayPlaneTest39, list);
  DeclareSimpleUnitTest(RayPlaneTest40, list);
  DeclareSimpleUnitTest(RayPlaneTest41, list);
  DeclareSimpleUnitTest(RayPlaneTest42, list);
  DeclareSimpleUnitTest(RayPlaneTest43, list);
  DeclareSimpleUnitTest(RayPlaneTest44, list);
  DeclareSimpleUnitTest(RayPlaneTest45, list);
  DeclareSimpleUnitTest(RayPlaneTest46, list);
  DeclareSimpleUnitTest(RayPlaneTest47, list);
  DeclareSimpleUnitTest(RayPlaneTest48, list);
  DeclareSimpleUnitTest(RayPlaneTest49, list);
  DeclareSimpleUnitTest(RayPlaneTest50, list);
  DeclareSimpleUnitTest(RayPlaneTest51, list);
  DeclareSimpleUnitTest(RayPlaneTest52, list);
  DeclareSimpleUnitTest(RayPlaneTest53, list);
  DeclareSimpleUnitTest(RayPlaneTest54, list);
  DeclareSimpleUnitTest(RayPlaneTest55, list);
  DeclareSimpleUnitTest(RayPlaneTest56, list);
  DeclareSimpleUnitTest(RayPlaneTest57, list);
  DeclareSimpleUnitTest(RayPlaneTest58, list);
  DeclareSimpleUnitTest(RayPlaneTest59, list);
  DeclareSimpleUnitTest(RayPlaneTest60, list);
  DeclareSimpleUnitTest(RayPlaneTest61, list);
  DeclareSimpleUnitTest(RayPlaneTest62, list);
  DeclareSimpleUnitTest(RayPlaneTest63, list);
  DeclareSimpleUnitTest(RayPlaneTest64, list);
  DeclareSimpleUnitTest(RayPlaneTest65, list);
  DeclareSimpleUnitTest(RayPlaneTest66, list);
  DeclareSimpleUnitTest(RayPlaneTest67, list);
  DeclareSimpleUnitTest(RayPlaneTest68, list);
  DeclareSimpleUnitTest(RayPlaneTest69, list);
  DeclareSimpleUnitTest(RayPlaneTest70, list);
  DeclareSimpleUnitTest(RayPlaneTest71, list);
  DeclareSimpleUnitTest(RayPlaneTest72, list);
}

void RegisterRayTriangleTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(RayTriangleTest1, list);
  DeclareSimpleUnitTest(RayTriangleTest2, list);
  DeclareSimpleUnitTest(RayTriangleTest3, list);
  DeclareSimpleUnitTest(RayTriangleTest4, list);
  DeclareSimpleUnitTest(RayTriangleTest5, list);
  DeclareSimpleUnitTest(RayTriangleTest6, list);
  DeclareSimpleUnitTest(RayTriangleTest7, list);
  DeclareSimpleUnitTest(RayTriangleTest8, list);
  DeclareSimpleUnitTest(RayTriangleTest9, list);
  DeclareSimpleUnitTest(RayTriangleTest10, list);
  DeclareSimpleUnitTest(RayTriangleTest11, list);
  DeclareSimpleUnitTest(RayTriangleTest12, list);
  DeclareSimpleUnitTest(RayTriangleTest13, list);
  DeclareSimpleUnitTest(RayTriangleTest14, list);
  DeclareSimpleUnitTest(RayTriangleTest15, list);
  DeclareSimpleUnitTest(RayTriangleTest16, list);
  DeclareSimpleUnitTest(RayTriangleTest17, list);
  DeclareSimpleUnitTest(RayTriangleTest18, list);
  DeclareSimpleUnitTest(RayTriangleTest19, list);
  DeclareSimpleUnitTest(RayTriangleTest20, list);
  DeclareSimpleUnitTest(RayTriangleTest21, list);
  DeclareSimpleUnitTest(RayTriangleTest22, list);
  DeclareSimpleUnitTest(RayTriangleTest23, list);
  DeclareSimpleUnitTest(RayTriangleTest24, list);
  DeclareSimpleUnitTest(RayTriangleTest25, list);
  DeclareSimpleUnitTest(RayTriangleTest26, list);
  DeclareSimpleUnitTest(RayTriangleTest27, list);
  DeclareSimpleUnitTest(RayTriangleTest28, list);
  DeclareSimpleUnitTest(RayTriangleTest29, list);
  DeclareSimpleUnitTest(RayTriangleTest30, list);
}

void RegisterRaySphereTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(RaySphereTest1, list);
  DeclareSimpleUnitTest(RaySphereTest2, list);
  DeclareSimpleUnitTest(RaySphereTest3, list);
  DeclareSimpleUnitTest(RaySphereTest4, list);
  DeclareSimpleUnitTest(RaySphereTest5, list);
  DeclareSimpleUnitTest(RaySphereTest6, list);
  DeclareSimpleUnitTest(RaySphereTest7, list);
  DeclareSimpleUnitTest(RaySphereTest8, list);
  DeclareSimpleUnitTest(RaySphereTest9, list);
  DeclareSimpleUnitTest(RaySphereTest10, list);
  DeclareSimpleUnitTest(RaySphereTest11, list);
  DeclareSimpleUnitTest(RaySphereTest12, list);
  DeclareSimpleUnitTest(RaySphereTest13, list);
  DeclareSimpleUnitTest(RaySphereTest14, list);
  DeclareSimpleUnitTest(RaySphereTest15, list);
  DeclareSimpleUnitTest(RaySphereTest16, list);
  DeclareSimpleUnitTest(RaySphereTest17, list);
  DeclareSimpleUnitTest(RaySphereTest18, list);
  DeclareSimpleUnitTest(RaySphereTest19, list);
  DeclareSimpleUnitTest(RaySphereTest20, list);
  DeclareSimpleUnitTest(RaySphereTest21, list);
  DeclareSimpleUnitTest(RaySphereTest22, list);
  DeclareSimpleUnitTest(RaySphereTest23, list);
  DeclareSimpleUnitTest(RaySphereTest24, list);
  DeclareSimpleUnitTest(RaySphereTest25, list);
  DeclareSimpleUnitTest(RaySphereTest26, list);
  DeclareSimpleUnitTest(RaySphereTest27, list);
  DeclareSimpleUnitTest(RaySphereTest28, list);
  DeclareSimpleUnitTest(RaySphereTest29, list);
  DeclareSimpleUnitTest(RaySphereTest30, list);
  DeclareSimpleUnitTest(RaySphereTest31, list);
  DeclareSimpleUnitTest(RaySphereTest32, list);
  DeclareSimpleUnitTest(RaySphereTest33, list);
  DeclareSimpleUnitTest(RaySphereTest34, list);
  DeclareSimpleUnitTest(RaySphereTest35, list);
  DeclareSimpleUnitTest(RaySphereTest36, list);
  DeclareSimpleUnitTest(RaySphereTest37, list);
  DeclareSimpleUnitTest(RaySphereTest38, list);
  DeclareSimpleUnitTest(RaySphereTest39, list);
  DeclareSimpleUnitTest(RaySphereTest40, list);
  DeclareSimpleUnitTest(RaySphereTest41, list);
  DeclareSimpleUnitTest(RaySphereTest42, list);
  DeclareSimpleUnitTest(RaySphereTest43, list);
  DeclareSimpleUnitTest(RaySphereTest44, list);
  DeclareSimpleUnitTest(RaySphereTest45, list);
  DeclareSimpleUnitTest(RaySphereTest46, list);
  DeclareSimpleUnitTest(RaySphereTest47, list);
  DeclareSimpleUnitTest(RaySphereTest48, list);
  DeclareSimpleUnitTest(RaySphereTest49, list);
  DeclareSimpleUnitTest(RaySphereTest50, list);
  DeclareSimpleUnitTest(RaySphereTest51, list);
  DeclareSimpleUnitTest(RaySphereTest52, list);
  DeclareSimpleUnitTest(RaySphereTest53, list);
  DeclareSimpleUnitTest(RaySphereTest54, list);
  DeclareSimpleUnitTest(RaySphereTest55, list);
  DeclareSimpleUnitTest(RaySphereTest56, list);
  DeclareSimpleUnitTest(RaySphereTest57, list);
  DeclareSimpleUnitTest(RaySphereTest58, list);
  DeclareSimpleUnitTest(RaySphereTest59, list);
  DeclareSimpleUnitTest(RaySphereTest60, list);
  DeclareSimpleUnitTest(RaySphereTest61, list);
  DeclareSimpleUnitTest(RaySphereTest62, list);
  DeclareSimpleUnitTest(RaySphereTest63, list);
  DeclareSimpleUnitTest(RaySphereTest64, list);
  DeclareSimpleUnitTest(RaySphereTest65, list);
  DeclareSimpleUnitTest(RaySphereTest66, list);
  DeclareSimpleUnitTest(RaySphereTest67, list);
  DeclareSimpleUnitTest(RaySphereTest68, list);
  DeclareSimpleUnitTest(RaySphereTest69, list);
  DeclareSimpleUnitTest(RaySphereTest70, list);
  DeclareSimpleUnitTest(RaySphereTest71, list);
  DeclareSimpleUnitTest(RaySphereTest72, list);
  DeclareSimpleUnitTest(RaySphereTest73, list);
  DeclareSimpleUnitTest(RaySphereTest74, list);
  DeclareSimpleUnitTest(RaySphereTest75, list);
  DeclareSimpleUnitTest(RaySphereTest76, list);
  DeclareSimpleUnitTest(RaySphereTest77, list);
  DeclareSimpleUnitTest(RaySphereTest78, list);
  DeclareSimpleUnitTest(RaySphereTest79, list);
  DeclareSimpleUnitTest(RaySphereTest80, list);
  DeclareSimpleUnitTest(RaySphereTest81, list);
  DeclareSimpleUnitTest(RaySphereTest82, list);
  DeclareSimpleUnitTest(RaySphereTest83, list);
  DeclareSimpleUnitTest(RaySphereTest84, list);
  DeclareSimpleUnitTest(RaySphereTest85, list);
}

void RegisterRayAabbTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(RayAabbTest1, list);
  DeclareSimpleUnitTest(RayAabbTest2, list);
  DeclareSimpleUnitTest(RayAabbTest3, list);
  DeclareSimpleUnitTest(RayAabbTest4, list);
  DeclareSimpleUnitTest(RayAabbTest5, list);
  DeclareSimpleUnitTest(RayAabbTest6, list);
  DeclareSimpleUnitTest(RayAabbTest7, list);
  DeclareSimpleUnitTest(RayAabbTest8, list);
  DeclareSimpleUnitTest(RayAabbTest9, list);
  DeclareSimpleUnitTest(RayAabbTest10, list);
  DeclareSimpleUnitTest(RayAabbTest11, list);
  DeclareSimpleUnitTest(RayAabbTest12, list);
  DeclareSimpleUnitTest(RayAabbTest13, list);
  DeclareSimpleUnitTest(RayAabbTest14, list);
  DeclareSimpleUnitTest(RayAabbTest15, list);
  DeclareSimpleUnitTest(RayAabbTest16, list);
  DeclareSimpleUnitTest(RayAabbTest17, list);
  DeclareSimpleUnitTest(RayAabbTest18, list);
  DeclareSimpleUnitTest(RayAabbTest19, list);
  DeclareSimpleUnitTest(RayAabbTest20, list);
  DeclareSimpleUnitTest(RayAabbTest21, list);
  DeclareSimpleUnitTest(RayAabbTest22, list);
  DeclareSimpleUnitTest(RayAabbTest23, list);
  DeclareSimpleUnitTest(RayAabbTest24, list);
  DeclareSimpleUnitTest(RayAabbTest25, list);
  DeclareSimpleUnitTest(RayAabbTest26, list);
  DeclareSimpleUnitTest(RayAabbTest27, list);
  DeclareSimpleUnitTest(RayAabbTest28, list);
  DeclareSimpleUnitTest(RayAabbTest29, list);
  DeclareSimpleUnitTest(RayAabbTest30, list);
  DeclareSimpleUnitTest(RayAabbTest31, list);
  DeclareSimpleUnitTest(RayAabbTest32, list);
  DeclareSimpleUnitTest(RayAabbTest33, list);
  DeclareSimpleUnitTest(RayAabbTest34, list);
  DeclareSimpleUnitTest(RayAabbTest35, list);
  DeclareSimpleUnitTest(RayAabbTest36, list);
  DeclareSimpleUnitTest(RayAabbTest37, list);
  DeclareSimpleUnitTest(RayAabbTest38, list);
  DeclareSimpleUnitTest(RayAabbTest39, list);
  DeclareSimpleUnitTest(RayAabbTest40, list);
  DeclareSimpleUnitTest(RayAabbTest41, list);
  DeclareSimpleUnitTest(RayAabbTest42, list);
  DeclareSimpleUnitTest(RayAabbTest43, list);
  DeclareSimpleUnitTest(RayAabbTest44, list);
  DeclareSimpleUnitTest(RayAabbTest45, list);
  DeclareSimpleUnitTest(RayAabbTest46, list);
  DeclareSimpleUnitTest(RayAabbTest47, list);
  DeclareSimpleUnitTest(RayAabbTest48, list);
  DeclareSimpleUnitTest(RayAabbTest49, list);
  DeclareSimpleUnitTest(RayAabbTest50, list);
  DeclareSimpleUnitTest(RayAabbTest51, list);
  DeclareSimpleUnitTest(RayAabbTest52, list);
  DeclareSimpleUnitTest(RayAabbTest53, list);
  DeclareSimpleUnitTest(RayAabbTest54, list);
  DeclareSimpleUnitTest(RayAabbTest55, list);
  DeclareSimpleUnitTest(RayAabbTest56, list);
  DeclareSimpleUnitTest(RayAabbTest57, list);
  DeclareSimpleUnitTest(RayAabbTest58, list);
  DeclareSimpleUnitTest(RayAabbTest59, list);
  DeclareSimpleUnitTest(RayAabbTest60, list);
  DeclareSimpleUnitTest(RayAabbTest61, list);
  DeclareSimpleUnitTest(RayAabbTest62, list);
  DeclareSimpleUnitTest(RayAabbTest63, list);
  DeclareSimpleUnitTest(RayAabbTest64, list);
  DeclareSimpleUnitTest(RayAabbTest65, list);
  DeclareSimpleUnitTest(RayAabbTest66, list);
  DeclareSimpleUnitTest(RayAabbTest67, list);
  DeclareSimpleUnitTest(RayAabbTest68, list);
  DeclareSimpleUnitTest(RayAabbTest69, list);
  DeclareSimpleUnitTest(RayAabbTest70, list);
  DeclareSimpleUnitTest(RayAabbTest71, list);
  DeclareSimpleUnitTest(RayAabbTest72, list);
  DeclareSimpleUnitTest(RayAabbTest73, list);
  DeclareSimpleUnitTest(RayAabbTest74, list);
  DeclareSimpleUnitTest(RayAabbTest75, list);
  DeclareSimpleUnitTest(RayAabbTest76, list);
  DeclareSimpleUnitTest(RayAabbTest77, list);
  DeclareSimpleUnitTest(RayAabbTest78, list);
  DeclareSimpleUnitTest(RayAabbTest79, list);
  DeclareSimpleUnitTest(RayAabbTest80, list);
  DeclareSimpleUnitTest(RayAabbTest81, list);
  DeclareSimpleUnitTest(RayAabbTest82, list);
  DeclareSimpleUnitTest(RayAabbTest83, list);
  DeclareSimpleUnitTest(RayAabbTest84, list);
  DeclareSimpleUnitTest(RayAabbTest85, list);
  DeclareSimpleUnitTest(RayAabbTest86, list);
  DeclareSimpleUnitTest(RayAabbTest87, list);
  DeclareSimpleUnitTest(RayAabbTest88, list);
  DeclareSimpleUnitTest(RayAabbTest89, list);
  DeclareSimpleUnitTest(RayAabbTest90, list);
  DeclareSimpleUnitTest(RayAabbTest91, list);
}

void RegisterPlaneTriangleTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PlaneTriangleTest1, list);
  DeclareSimpleUnitTest(PlaneTriangleTest2, list);
  DeclareSimpleUnitTest(PlaneTriangleTest3, list);
  DeclareSimpleUnitTest(PlaneTriangleTest4, list);
  DeclareSimpleUnitTest(PlaneTriangleTest5, list);
  DeclareSimpleUnitTest(PlaneTriangleTest6, list);
  DeclareSimpleUnitTest(PlaneTriangleTest7, list);
  DeclareSimpleUnitTest(PlaneTriangleTest8, list);
  DeclareSimpleUnitTest(PlaneTriangleTest9, list);
  DeclareSimpleUnitTest(PlaneTriangleTest10, list);
  DeclareSimpleUnitTest(PlaneTriangleTest11, list);
  DeclareSimpleUnitTest(PlaneTriangleTest12, list);
  DeclareSimpleUnitTest(PlaneTriangleTest13, list);
  DeclareSimpleUnitTest(PlaneTriangleTest14, list);
  DeclareSimpleUnitTest(PlaneTriangleTest15, list);
  DeclareSimpleUnitTest(PlaneTriangleTest16, list);
  DeclareSimpleUnitTest(PlaneTriangleTest17, list);
  DeclareSimpleUnitTest(PlaneTriangleTest18, list);
  DeclareSimpleUnitTest(PlaneTriangleTest19, list);
  DeclareSimpleUnitTest(PlaneTriangleTest20, list);
  DeclareSimpleUnitTest(PlaneTriangleTest21, list);
  DeclareSimpleUnitTest(PlaneTriangleTest22, list);
  DeclareSimpleUnitTest(PlaneTriangleTest23, list);
  DeclareSimpleUnitTest(PlaneTriangleTest24, list);
  DeclareSimpleUnitTest(PlaneTriangleTest25, list);
  DeclareSimpleUnitTest(PlaneTriangleTest26, list);
  DeclareSimpleUnitTest(PlaneTriangleTest27, list);
  DeclareSimpleUnitTest(PlaneTriangleTest28, list);
  DeclareSimpleUnitTest(PlaneTriangleTest29, list);
  DeclareSimpleUnitTest(PlaneTriangleTest30, list);
  DeclareSimpleUnitTest(PlaneTriangleTest31, list);
  DeclareSimpleUnitTest(PlaneTriangleTest32, list);
  DeclareSimpleUnitTest(PlaneTriangleTest33, list);
  DeclareSimpleUnitTest(PlaneTriangleTest34, list);
  DeclareSimpleUnitTest(PlaneTriangleTest35, list);
  DeclareSimpleUnitTest(PlaneTriangleTest36, list);
  DeclareSimpleUnitTest(PlaneTriangleTest37, list);
  DeclareSimpleUnitTest(PlaneTriangleTest38, list);
  DeclareSimpleUnitTest(PlaneTriangleTest39, list);
  DeclareSimpleUnitTest(PlaneTriangleTest40, list);
  DeclareSimpleUnitTest(PlaneTriangleTest41, list);
  DeclareSimpleUnitTest(PlaneTriangleTest42, list);
  DeclareSimpleUnitTest(PlaneTriangleTest43, list);
  DeclareSimpleUnitTest(PlaneTriangleTest44, list);
  DeclareSimpleUnitTest(PlaneTriangleTest45, list);
  DeclareSimpleUnitTest(PlaneTriangleTest46, list);
  DeclareSimpleUnitTest(PlaneTriangleTest47, list);
  DeclareSimpleUnitTest(PlaneTriangleTest48, list);
  DeclareSimpleUnitTest(PlaneTriangleTest49, list);
  DeclareSimpleUnitTest(PlaneTriangleTest50, list);
  DeclareSimpleUnitTest(PlaneTriangleTest51, list);
  DeclareSimpleUnitTest(PlaneTriangleTest52, list);
  DeclareSimpleUnitTest(PlaneTriangleTest53, list);
  DeclareSimpleUnitTest(PlaneTriangleTest54, list);
  DeclareSimpleUnitTest(PlaneTriangleTest55, list);
  DeclareSimpleUnitTest(PlaneTriangleTest56, list);
  DeclareSimpleUnitTest(PlaneTriangleTest57, list);
  DeclareSimpleUnitTest(PlaneTriangleTest58, list);
  DeclareSimpleUnitTest(PlaneTriangleTest59, list);
  DeclareSimpleUnitTest(PlaneTriangleTest60, list);
  DeclareSimpleUnitTest(PlaneTriangleTest61, list);
  DeclareSimpleUnitTest(PlaneTriangleTest62, list);
  DeclareSimpleUnitTest(PlaneTriangleTest63, list);
}

void RegisterPlaneSphereTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PlaneSphereTest1, list);
  DeclareSimpleUnitTest(PlaneSphereTest2, list);
  DeclareSimpleUnitTest(PlaneSphereTest3, list);
  DeclareSimpleUnitTest(PlaneSphereTest4, list);
  DeclareSimpleUnitTest(PlaneSphereTest5, list);
  DeclareSimpleUnitTest(PlaneSphereTest6, list);
  DeclareSimpleUnitTest(PlaneSphereTest7, list);
  DeclareSimpleUnitTest(PlaneSphereTest8, list);
  DeclareSimpleUnitTest(PlaneSphereTest9, list);
  DeclareSimpleUnitTest(PlaneSphereTest10, list);
  DeclareSimpleUnitTest(PlaneSphereTest11, list);
  DeclareSimpleUnitTest(PlaneSphereTest12, list);
  DeclareSimpleUnitTest(PlaneSphereTest13, list);
  DeclareSimpleUnitTest(PlaneSphereTest14, list);
  DeclareSimpleUnitTest(PlaneSphereTest15, list);
  DeclareSimpleUnitTest(PlaneSphereTest16, list);
  DeclareSimpleUnitTest(PlaneSphereTest17, list);
  DeclareSimpleUnitTest(PlaneSphereTest18, list);
  DeclareSimpleUnitTest(PlaneSphereTest19, list);
  DeclareSimpleUnitTest(PlaneSphereTest20, list);
}

void RegisterPlaneAabbTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PlaneAabbTest1, list);
  DeclareSimpleUnitTest(PlaneAabbTest2, list);
  DeclareSimpleUnitTest(PlaneAabbTest3, list);
  DeclareSimpleUnitTest(PlaneAabbTest4, list);
  DeclareSimpleUnitTest(PlaneAabbTest5, list);
  DeclareSimpleUnitTest(PlaneAabbTest6, list);
  DeclareSimpleUnitTest(PlaneAabbTest7, list);
  DeclareSimpleUnitTest(PlaneAabbTest8, list);
  DeclareSimpleUnitTest(PlaneAabbTest9, list);
  DeclareSimpleUnitTest(PlaneAabbTest10, list);
  DeclareSimpleUnitTest(PlaneAabbTest11, list);
  DeclareSimpleUnitTest(PlaneAabbTest12, list);
  DeclareSimpleUnitTest(PlaneAabbTest13, list);
  DeclareSimpleUnitTest(PlaneAabbTest14, list);
  DeclareSimpleUnitTest(PlaneAabbTest15, list);
  DeclareSimpleUnitTest(PlaneAabbTest16, list);
  DeclareSimpleUnitTest(PlaneAabbTest17, list);
  DeclareSimpleUnitTest(PlaneAabbTest18, list);
  DeclareSimpleUnitTest(PlaneAabbTest19, list);
  DeclareSimpleUnitTest(PlaneAabbTest20, list);
  DeclareSimpleUnitTest(PlaneAabbTest21, list);
  DeclareSimpleUnitTest(PlaneAabbTest22, list);
  DeclareSimpleUnitTest(PlaneAabbTest23, list);
  DeclareSimpleUnitTest(PlaneAabbTest24, list);
  DeclareSimpleUnitTest(PlaneAabbTest25, list);
  DeclareSimpleUnitTest(PlaneAabbTest26, list);
  DeclareSimpleUnitTest(PlaneAabbTest27, list);
  DeclareSimpleUnitTest(PlaneAabbTest28, list);
  DeclareSimpleUnitTest(PlaneAabbTest29, list);
  DeclareSimpleUnitTest(PlaneAabbTest30, list);
  DeclareSimpleUnitTest(PlaneAabbTest31, list);
  DeclareSimpleUnitTest(PlaneAabbTest32, list);
  DeclareSimpleUnitTest(PlaneAabbTest33, list);
  DeclareSimpleUnitTest(PlaneAabbTest34, list);
  DeclareSimpleUnitTest(PlaneAabbTest35, list);
  DeclareSimpleUnitTest(PlaneAabbTest36, list);
  DeclareSimpleUnitTest(PlaneAabbTest37, list);
  DeclareSimpleUnitTest(PlaneAabbTest38, list);
  DeclareSimpleUnitTest(PlaneAabbTest39, list);
  DeclareSimpleUnitTest(PlaneAabbTest40, list);
  DeclareSimpleUnitTest(PlaneAabbTest41, list);
  DeclareSimpleUnitTest(PlaneAabbTest42, list);
  DeclareSimpleUnitTest(PlaneAabbTest43, list);
  DeclareSimpleUnitTest(PlaneAabbTest44, list);
  DeclareSimpleUnitTest(PlaneAabbTest45, list);
  DeclareSimpleUnitTest(PlaneAabbTest46, list);
  DeclareSimpleUnitTest(PlaneAabbTest47, list);
  DeclareSimpleUnitTest(PlaneAabbTest48, list);
  DeclareSimpleUnitTest(PlaneAabbTest49, list);
  DeclareSimpleUnitTest(PlaneAabbTest50, list);
  DeclareSimpleUnitTest(PlaneAabbTest51, list);
  DeclareSimpleUnitTest(PlaneAabbTest52, list);
  DeclareSimpleUnitTest(PlaneAabbTest53, list);
  DeclareSimpleUnitTest(PlaneAabbTest54, list);
  DeclareSimpleUnitTest(PlaneAabbTest55, list);
  DeclareSimpleUnitTest(PlaneAabbTest56, list);
  DeclareSimpleUnitTest(PlaneAabbTest57, list);
  DeclareSimpleUnitTest(PlaneAabbTest58, list);
  DeclareSimpleUnitTest(PlaneAabbTest59, list);
  DeclareSimpleUnitTest(PlaneAabbTest60, list);
  DeclareSimpleUnitTest(PlaneAabbTest61, list);
  DeclareSimpleUnitTest(PlaneAabbTest62, list);
  DeclareSimpleUnitTest(PlaneAabbTest63, list);
  DeclareSimpleUnitTest(PlaneAabbTest64, list);
  DeclareSimpleUnitTest(PlaneAabbTest65, list);
  DeclareSimpleUnitTest(PlaneAabbTest66, list);
  DeclareSimpleUnitTest(PlaneAabbTest67, list);
  DeclareSimpleUnitTest(PlaneAabbTest68, list);
  DeclareSimpleUnitTest(PlaneAabbTest69, list);
  DeclareSimpleUnitTest(PlaneAabbTest70, list);
  DeclareSimpleUnitTest(PlaneAabbTest71, list);
  DeclareSimpleUnitTest(PlaneAabbTest72, list);
  DeclareSimpleUnitTest(PlaneAabbTest73, list);
  DeclareSimpleUnitTest(PlaneAabbTest74, list);
  DeclareSimpleUnitTest(PlaneAabbTest75, list);
  DeclareSimpleUnitTest(PlaneAabbTest76, list);
  DeclareSimpleUnitTest(PlaneAabbTest77, list);
  DeclareSimpleUnitTest(PlaneAabbTest78, list);
  DeclareSimpleUnitTest(PlaneAabbTest79, list);
  DeclareSimpleUnitTest(PlaneAabbTest80, list);
  DeclareSimpleUnitTest(PlaneAabbTest81, list);
  DeclareSimpleUnitTest(PlaneAabbTest82, list);
  DeclareSimpleUnitTest(PlaneAabbTest83, list);
  DeclareSimpleUnitTest(PlaneAabbTest84, list);
  DeclareSimpleUnitTest(PlaneAabbTest85, list);
  DeclareSimpleUnitTest(PlaneAabbTest86, list);
  DeclareSimpleUnitTest(PlaneAabbTest87, list);
  DeclareSimpleUnitTest(PlaneAabbTest88, list);
  DeclareSimpleUnitTest(PlaneAabbTest89, list);
  DeclareSimpleUnitTest(PlaneAabbTest90, list);
  DeclareSimpleUnitTest(PlaneAabbTest91, list);
  DeclareSimpleUnitTest(PlaneAabbTest92, list);
  DeclareSimpleUnitTest(PlaneAabbTest93, list);
  DeclareSimpleUnitTest(PlaneAabbTest94, list);
  DeclareSimpleUnitTest(PlaneAabbTest95, list);
  DeclareSimpleUnitTest(PlaneAabbTest96, list);
  DeclareSimpleUnitTest(PlaneAabbTest97, list);
  DeclareSimpleUnitTest(PlaneAabbTest98, list);
  DeclareSimpleUnitTest(PlaneAabbTest99, list);
  DeclareSimpleUnitTest(PlaneAabbTest100, list);
  DeclareSimpleUnitTest(PlaneAabbTest101, list);
  DeclareSimpleUnitTest(PlaneAabbTest102, list);
  DeclareSimpleUnitTest(PlaneAabbTest103, list);
  DeclareSimpleUnitTest(PlaneAabbTest104, list);
  DeclareSimpleUnitTest(PlaneAabbTest105, list);
  DeclareSimpleUnitTest(PlaneAabbTest106, list);
  DeclareSimpleUnitTest(PlaneAabbTest107, list);
  DeclareSimpleUnitTest(PlaneAabbTest108, list);
  DeclareSimpleUnitTest(PlaneAabbTest109, list);
  DeclareSimpleUnitTest(PlaneAabbTest110, list);
  DeclareSimpleUnitTest(PlaneAabbTest111, list);
  DeclareSimpleUnitTest(PlaneAabbTest112, list);
  DeclareSimpleUnitTest(PlaneAabbTest113, list);
  DeclareSimpleUnitTest(PlaneAabbTest114, list);
  DeclareSimpleUnitTest(PlaneAabbTest115, list);
  DeclareSimpleUnitTest(PlaneAabbTest116, list);
  DeclareSimpleUnitTest(PlaneAabbTest117, list);
  DeclareSimpleUnitTest(PlaneAabbTest118, list);
  DeclareSimpleUnitTest(PlaneAabbTest119, list);
  DeclareSimpleUnitTest(PlaneAabbTest120, list);
  DeclareSimpleUnitTest(PlaneAabbTest121, list);
  DeclareSimpleUnitTest(PlaneAabbTest122, list);
  DeclareSimpleUnitTest(PlaneAabbTest123, list);
  DeclareSimpleUnitTest(PlaneAabbTest124, list);
  DeclareSimpleUnitTest(PlaneAabbTest125, list);
  DeclareSimpleUnitTest(PlaneAabbTest126, list);
  DeclareSimpleUnitTest(PlaneAabbTest127, list);
  DeclareSimpleUnitTest(PlaneAabbTest128, list);
  DeclareSimpleUnitTest(PlaneAabbTest129, list);
  DeclareSimpleUnitTest(PlaneAabbTest130, list);
  DeclareSimpleUnitTest(PlaneAabbTest131, list);
  DeclareSimpleUnitTest(PlaneAabbTest132, list);
  DeclareSimpleUnitTest(PlaneAabbTest133, list);
  DeclareSimpleUnitTest(PlaneAabbTest134, list);
  DeclareSimpleUnitTest(PlaneAabbTest135, list);
  DeclareSimpleUnitTest(PlaneAabbTest136, list);
  DeclareSimpleUnitTest(PlaneAabbTest137, list);
  DeclareSimpleUnitTest(PlaneAabbTest138, list);
  DeclareSimpleUnitTest(PlaneAabbTest139, list);
  DeclareSimpleUnitTest(PlaneAabbTest140, list);
  DeclareSimpleUnitTest(PlaneAabbTest141, list);
  DeclareSimpleUnitTest(PlaneAabbTest142, list);
  DeclareSimpleUnitTest(PlaneAabbTest143, list);
  DeclareSimpleUnitTest(PlaneAabbTest144, list);
  DeclareSimpleUnitTest(PlaneAabbTest145, list);
  DeclareSimpleUnitTest(PlaneAabbTest146, list);
  DeclareSimpleUnitTest(PlaneAabbTest147, list);
  DeclareSimpleUnitTest(PlaneAabbTest148, list);
  DeclareSimpleUnitTest(PlaneAabbTest149, list);
  DeclareSimpleUnitTest(PlaneAabbTest150, list);
  DeclareSimpleUnitTest(PlaneAabbTest151, list);
  DeclareSimpleUnitTest(PlaneAabbTest152, list);
  DeclareSimpleUnitTest(PlaneAabbTest153, list);
  DeclareSimpleUnitTest(PlaneAabbTest154, list);
  DeclareSimpleUnitTest(PlaneAabbTest155, list);
  DeclareSimpleUnitTest(PlaneAabbTest156, list);
  DeclareSimpleUnitTest(PlaneAabbTest157, list);
  DeclareSimpleUnitTest(PlaneAabbTest158, list);
  DeclareSimpleUnitTest(PlaneAabbTest159, list);
  DeclareSimpleUnitTest(PlaneAabbTest160, list);
  DeclareSimpleUnitTest(PlaneAabbTest161, list);
  DeclareSimpleUnitTest(PlaneAabbTest162, list);
  DeclareSimpleUnitTest(PlaneAabbTest163, list);
  DeclareSimpleUnitTest(PlaneAabbTest164, list);
  DeclareSimpleUnitTest(PlaneAabbTest165, list);
  DeclareSimpleUnitTest(PlaneAabbTest166, list);
  DeclareSimpleUnitTest(PlaneAabbTest167, list);
  DeclareSimpleUnitTest(PlaneAabbTest168, list);
  DeclareSimpleUnitTest(PlaneAabbTest169, list);
  DeclareSimpleUnitTest(PlaneAabbTest170, list);
  DeclareSimpleUnitTest(PlaneAabbTest171, list);
  DeclareSimpleUnitTest(PlaneAabbTest172, list);
  DeclareSimpleUnitTest(PlaneAabbTest173, list);
  DeclareSimpleUnitTest(PlaneAabbTest174, list);
  DeclareSimpleUnitTest(PlaneAabbTest175, list);
  DeclareSimpleUnitTest(PlaneAabbTest176, list);
  DeclareSimpleUnitTest(PlaneAabbTest177, list);
  DeclareSimpleUnitTest(PlaneAabbTest178, list);
  DeclareSimpleUnitTest(PlaneAabbTest179, list);
  DeclareSimpleUnitTest(PlaneAabbTest180, list);
  DeclareSimpleUnitTest(PlaneAabbTest181, list);
  DeclareSimpleUnitTest(PlaneAabbTest182, list);
  DeclareSimpleUnitTest(PlaneAabbTest183, list);
  DeclareSimpleUnitTest(PlaneAabbTest184, list);
  DeclareSimpleUnitTest(PlaneAabbTest185, list);
  DeclareSimpleUnitTest(PlaneAabbTest186, list);
  DeclareSimpleUnitTest(PlaneAabbTest187, list);
  DeclareSimpleUnitTest(PlaneAabbTest188, list);
  DeclareSimpleUnitTest(PlaneAabbTest189, list);
  DeclareSimpleUnitTest(PlaneAabbTest190, list);
  DeclareSimpleUnitTest(PlaneAabbTest191, list);
  DeclareSimpleUnitTest(PlaneAabbTest192, list);
  DeclareSimpleUnitTest(PlaneAabbTest193, list);
  DeclareSimpleUnitTest(PlaneAabbTest194, list);
  DeclareSimpleUnitTest(PlaneAabbTest195, list);
  DeclareSimpleUnitTest(PlaneAabbTest196, list);
  DeclareSimpleUnitTest(PlaneAabbTest197, list);
  DeclareSimpleUnitTest(PlaneAabbTest198, list);
  DeclareSimpleUnitTest(PlaneAabbTest199, list);
  DeclareSimpleUnitTest(PlaneAabbTest200, list);
  DeclareSimpleUnitTest(PlaneAabbTest201, list);
  DeclareSimpleUnitTest(PlaneAabbTest202, list);
  DeclareSimpleUnitTest(PlaneAabbTest203, list);
  DeclareSimpleUnitTest(PlaneAabbTest204, list);
  DeclareSimpleUnitTest(PlaneAabbTest205, list);
  DeclareSimpleUnitTest(PlaneAabbTest206, list);
  DeclareSimpleUnitTest(PlaneAabbTest207, list);
  DeclareSimpleUnitTest(PlaneAabbTest208, list);
  DeclareSimpleUnitTest(PlaneAabbTest209, list);
  DeclareSimpleUnitTest(PlaneAabbTest210, list);
  DeclareSimpleUnitTest(PlaneAabbTest211, list);
  DeclareSimpleUnitTest(PlaneAabbTest212, list);
  DeclareSimpleUnitTest(PlaneAabbTest213, list);
  DeclareSimpleUnitTest(PlaneAabbTest214, list);
  DeclareSimpleUnitTest(PlaneAabbTest215, list);
  DeclareSimpleUnitTest(PlaneAabbTest216, list);
}

void RegisterFrustumTriangleTestsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(FrustumTriangleTest1, list);
  DeclareSimpleUnitTest(FrustumTriangleTest2, list);
  DeclareSimpleUnitTest(FrustumTriangleTest3, list);
  DeclareSimpleUnitTest(FrustumTriangleTest4, list);
  DeclareSimpleUnitTest(FrustumTriangleTest5, list);
  DeclareSimpleUnitTest(FrustumTriangleTest6, list);
  DeclareSimpleUnitTest(FrustumTriangleTest7, list);
  DeclareSimpleUnitTest(FrustumTriangleTest8, list);
  DeclareSimpleUnitTest(FrustumTriangleTest9, list);
  DeclareSimpleUnitTest(FrustumTriangleTest10, list);
  DeclareSimpleUnitTest(FrustumTriangleTest11, list);
  DeclareSimpleUnitTest(FrustumTriangleTest12, list);
  DeclareSimpleUnitTest(FrustumTriangleTest13, list);
  DeclareSimpleUnitTest(FrustumTriangleTest14, list);
  DeclareSimpleUnitTest(FrustumTriangleTest15, list);
  DeclareSimpleUnitTest(FrustumTriangleTest16, list);
  DeclareSimpleUnitTest(FrustumTriangleTest17, list);
  DeclareSimpleUnitTest(FrustumTriangleTest18, list);
  DeclareSimpleUnitTest(FrustumTriangleTest19, list);
  DeclareSimpleUnitTest(FrustumTriangleTest20, list);
  DeclareSimpleUnitTest(FrustumTriangleTest21, list);
  DeclareSimpleUnitTest(FrustumTriangleTest22, list);
  DeclareSimpleUnitTest(FrustumTriangleTest23, list);
  DeclareSimpleUnitTest(FrustumTriangleTest24, list);
  DeclareSimpleUnitTest(FrustumTriangleTest25, list);
  DeclareSimpleUnitTest(FrustumTriangleTest26, list);
  DeclareSimpleUnitTest(FrustumTriangleTest27, list);
  DeclareSimpleUnitTest(FrustumTriangleTest28, list);
  DeclareSimpleUnitTest(FrustumTriangleTest29, list);
  DeclareSimpleUnitTest(FrustumTriangleTest30, list);
  DeclareSimpleUnitTest(FrustumTriangleTest31, list);
  DeclareSimpleUnitTest(FrustumTriangleTest32, list);
  DeclareSimpleUnitTest(FrustumTriangleTest33, list);
  DeclareSimpleUnitTest(FrustumTriangleTest34, list);
  DeclareSimpleUnitTest(FrustumTriangleTest35, list);
  DeclareSimpleUnitTest(FrustumTriangleTest36, list);
  DeclareSimpleUnitTest(FrustumTriangleTest37, list);
  DeclareSimpleUnitTest(FrustumTriangleTest38, list);
  DeclareSimpleUnitTest(FrustumTriangleTest39, list);
  DeclareSimpleUnitTest(FrustumTriangleTest40, list);
  DeclareSimpleUnitTest(FrustumTriangleTest41, list);
  DeclareSimpleUnitTest(FrustumTriangleTest42, list);
  DeclareSimpleUnitTest(FrustumTriangleTest43, list);
  DeclareSimpleUnitTest(FrustumTriangleTest44, list);
  DeclareSimpleUnitTest(FrustumTriangleTest45, list);
  DeclareSimpleUnitTest(FrustumTriangleTest46, list);
  DeclareSimpleUnitTest(FrustumTriangleTest47, list);
  DeclareSimpleUnitTest(FrustumTriangleTest48, list);
  DeclareSimpleUnitTest(FrustumTriangleTest49, list);
  DeclareSimpleUnitTest(FrustumTriangleTest50, list);
  DeclareSimpleUnitTest(FrustumTriangleTest51, list);
  DeclareSimpleUnitTest(FrustumTriangleTest52, list);
  DeclareSimpleUnitTest(FrustumTriangleTest53, list);
  DeclareSimpleUnitTest(FrustumTriangleTest54, list);
  DeclareSimpleUnitTest(FrustumTriangleTest55, list);
  DeclareSimpleUnitTest(FrustumTriangleTest56, list);
  DeclareSimpleUnitTest(FrustumTriangleTest57, list);
  DeclareSimpleUnitTest(FrustumTriangleTest58, list);
  DeclareSimpleUnitTest(FrustumTriangleTest59, list);
  DeclareSimpleUnitTest(FrustumTriangleTest60, list);
  DeclareSimpleUnitTest(FrustumTriangleTest61, list);
  DeclareSimpleUnitTest(FrustumTriangleTest62, list);
  DeclareSimpleUnitTest(FrustumTriangleTest63, list);
  DeclareSimpleUnitTest(FrustumTriangleTest64, list);
  DeclareSimpleUnitTest(FrustumTriangleTest65, list);
  DeclareSimpleUnitTest(FrustumTriangleTest66, list);
  DeclareSimpleUnitTest(FrustumTriangleTest67, list);
  DeclareSimpleUnitTest(FrustumTriangleTest68, list);
  DeclareSimpleUnitTest(FrustumTriangleTest69, list);
  DeclareSimpleUnitTest(FrustumTriangleTest70, list);
  DeclareSimpleUnitTest(FrustumTriangleTest71, list);
  DeclareSimpleUnitTest(FrustumTriangleTest72, list);
  DeclareSimpleUnitTest(FrustumTriangleTest73, list);
  DeclareSimpleUnitTest(FrustumTriangleTest74, list);
  DeclareSimpleUnitTest(FrustumTriangleTest75, list);
  DeclareSimpleUnitTest(FrustumTriangleTest76, list);
  DeclareSimpleUnitTest(FrustumTriangleTest77, list);
  DeclareSimpleUnitTest(FrustumTriangleTest78, list);
  DeclareSimpleUnitTest(FrustumTriangleTest79, list);
  DeclareSimpleUnitTest(FrustumTriangleTest80, list);
  DeclareSimpleUnitTest(FrustumTriangleTest81, list);
  DeclareSimpleUnitTest(FrustumTriangleTest82, list);
  DeclareSimpleUnitTest(FrustumTriangleTest83, list);
  DeclareSimpleUnitTest(FrustumTriangleTest84, list);
  DeclareSimpleUnitTest(FrustumTriangleTest85, list);
  DeclareSimpleUnitTest(FrustumTriangleTest86, list);
  DeclareSimpleUnitTest(FrustumTriangleTest87, list);
  DeclareSimpleUnitTest(FrustumTriangleTest88, list);
  DeclareSimpleUnitTest(FrustumTriangleTest89, list);
  DeclareSimpleUnitTest(FrustumTriangleTest90, list);
  DeclareSimpleUnitTest(FrustumTriangleTest91, list);
  DeclareSimpleUnitTest(FrustumTriangleTest92, list);
  DeclareSimpleUnitTest(FrustumTriangleTest93, list);
  DeclareSimpleUnitTest(FrustumTriangleTest94, list);
  DeclareSimpleUnitTest(FrustumTriangleTest95, list);
  DeclareSimpleUnitTest(FrustumTriangleTest96, list);
  DeclareSimpleUnitTest(FrustumTriangleTest97, list);
  DeclareSimpleUnitTest(FrustumTriangleTest98, list);
  DeclareSimpleUnitTest(FrustumTriangleTest99, list);
  DeclareSimpleUnitTest(FrustumTriangleTest100, list);
  DeclareSimpleUnitTest(FrustumTriangleTest101, list);
  DeclareSimpleUnitTest(FrustumTriangleTest102, list);
  DeclareSimpleUnitTest(FrustumTriangleTest103, list);
  DeclareSimpleUnitTest(FrustumTriangleTest104, list);
  DeclareSimpleUnitTest(FrustumTriangleTest105, list);
  DeclareSimpleUnitTest(FrustumTriangleTest106, list);
  DeclareSimpleUnitTest(FrustumTriangleTest107, list);
  DeclareSimpleUnitTest(FrustumTriangleTest108, list);
  DeclareSimpleUnitTest(FrustumTriangleTest109, list);
  DeclareSimpleUnitTest(FrustumTriangleTest110, list);
  DeclareSimpleUnitTest(FrustumTriangleTest111, list);
  DeclareSimpleUnitTest(FrustumTriangleTest112, list);
  DeclareSimpleUnitTest(FrustumTriangleTest113, list);
  DeclareSimpleUnitTest(FrustumTriangleTest114, list);
  DeclareSimpleUnitTest(FrustumTriangleTest115, list);
  DeclareSimpleUnitTest(FrustumTriangleTest116, list);
  DeclareSimpleUnitTest(FrustumTriangleTest117, list);
  DeclareSimpleUnitTest(FrustumTriangleTest118, list);
  DeclareSimpleUnitTest(FrustumTriangleTest119, list);
  DeclareSimpleUnitTest(FrustumTriangleTest120, list);
  DeclareSimpleUnitTest(FrustumTriangleTest121, list);
  DeclareSimpleUnitTest(FrustumTriangleTest122, list);
  DeclareSimpleUnitTest(FrustumTriangleTest123, list);
  DeclareSimpleUnitTest(FrustumTriangleTest124, list);
  DeclareSimpleUnitTest(FrustumTriangleTest125, list);
  DeclareSimpleUnitTest(FrustumTriangleTest126, list);
  DeclareSimpleUnitTest(FrustumTriangleTest127, list);
  DeclareSimpleUnitTest(FrustumTriangleTest128, list);
  DeclareSimpleUnitTest(FrustumTriangleTest129, list);
  DeclareSimpleUnitTest(FrustumTriangleTest130, list);
  DeclareSimpleUnitTest(FrustumTriangleTest131, list);
  DeclareSimpleUnitTest(FrustumTriangleTest132, list);
  DeclareSimpleUnitTest(FrustumTriangleTest133, list);
  DeclareSimpleUnitTest(FrustumTriangleTest134, list);
  DeclareSimpleUnitTest(FrustumTriangleTest135, list);
  DeclareSimpleUnitTest(FrustumTriangleTest136, list);
  DeclareSimpleUnitTest(FrustumTriangleTest137, list);
  DeclareSimpleUnitTest(FrustumTriangleTest138, list);
  DeclareSimpleUnitTest(FrustumTriangleTest139, list);
  DeclareSimpleUnitTest(FrustumTriangleTest140, list);
  DeclareSimpleUnitTest(FrustumTriangleTest141, list);
  DeclareSimpleUnitTest(FrustumTriangleTest142, list);
  DeclareSimpleUnitTest(FrustumTriangleTest143, list);
  DeclareSimpleUnitTest(FrustumTriangleTest144, list);
  DeclareSimpleUnitTest(FrustumTriangleTest145, list);
  DeclareSimpleUnitTest(FrustumTriangleTest146, list);
  DeclareSimpleUnitTest(FrustumTriangleTest147, list);
  DeclareSimpleUnitTest(FrustumTriangleTest148, list);
  DeclareSimpleUnitTest(FrustumTriangleTest149, list);
  DeclareSimpleUnitTest(FrustumTriangleTest150, list);
  DeclareSimpleUnitTest(FrustumTriangleTest151, list);
  DeclareSimpleUnitTest(FrustumTriangleTest152, list);
  DeclareSimpleUnitTest(FrustumTriangleTest153, list);
  DeclareSimpleUnitTest(FrustumTriangleTest154, list);
  DeclareSimpleUnitTest(FrustumTriangleTest155, list);
  DeclareSimpleUnitTest(FrustumTriangleTest156, list);
  DeclareSimpleUnitTest(FrustumTriangleTest157, list);
  DeclareSimpleUnitTest(FrustumTriangleTest158, list);
  DeclareSimpleUnitTest(FrustumTriangleTest159, list);
  DeclareSimpleUnitTest(FrustumTriangleTest160, list);
  DeclareSimpleUnitTest(FrustumTriangleTest161, list);
  DeclareSimpleUnitTest(FrustumTriangleTest162, list);
  DeclareSimpleUnitTest(FrustumTriangleTest163, list);
  DeclareSimpleUnitTest(FrustumTriangleTest164, list);
  DeclareSimpleUnitTest(FrustumTriangleTest165, list);
  DeclareSimpleUnitTest(FrustumTriangleTest166, list);
  DeclareSimpleUnitTest(FrustumTriangleTest167, list);
  DeclareSimpleUnitTest(FrustumTriangleTest168, list);
  DeclareSimpleUnitTest(FrustumTriangleTest169, list);
  DeclareSimpleUnitTest(FrustumTriangleTest170, list);
  DeclareSimpleUnitTest(FrustumTriangleTest171, list);
  DeclareSimpleUnitTest(FrustumTriangleTest172, list);
  DeclareSimpleUnitTest(FrustumTriangleTest173, list);
  DeclareSimpleUnitTest(FrustumTriangleTest174, list);
  DeclareSimpleUnitTest(FrustumTriangleTest175, list);
  DeclareSimpleUnitTest(FrustumTriangleTest176, list);
  DeclareSimpleUnitTest(FrustumTriangleTest177, list);
  DeclareSimpleUnitTest(FrustumTriangleTest178, list);
  DeclareSimpleUnitTest(FrustumTriangleTest179, list);
  DeclareSimpleUnitTest(FrustumTriangleTest180, list);
  DeclareSimpleUnitTest(FrustumTriangleTest181, list);
  DeclareSimpleUnitTest(FrustumTriangleTest182, list);
  DeclareSimpleUnitTest(FrustumTriangleTest183, list);
  DeclareSimpleUnitTest(FrustumTriangleTest184, list);
  DeclareSimpleUnitTest(FrustumTriangleTest185, list);
  DeclareSimpleUnitTest(FrustumTriangleTest186, list);
  DeclareSimpleUnitTest(FrustumTriangleTest187, list);
  DeclareSimpleUnitTest(FrustumTriangleTest188, list);
  DeclareSimpleUnitTest(FrustumTriangleTest189, list);
  DeclareSimpleUnitTest(FrustumTriangleTest190, list);
  DeclareSimpleUnitTest(FrustumTriangleTest191, list);
  DeclareSimpleUnitTest(FrustumTriangleTest192, list);
  DeclareSimpleUnitTest(FrustumTriangleTest193, list);
  DeclareSimpleUnitTest(FrustumTriangleTest194, list);
  DeclareSimpleUnitTest(FrustumTriangleTest195, list);
  DeclareSimpleUnitTest(FrustumTriangleTest196, list);
  DeclareSimpleUnitTest(FrustumTriangleTest197, list);
  DeclareSimpleUnitTest(FrustumTriangleTest198, list);
  DeclareSimpleUnitTest(FrustumTriangleTest199, list);
  DeclareSimpleUnitTest(FrustumTriangleTest200, list);
  DeclareSimpleUnitTest(FrustumTriangleTest201, list);
  DeclareSimpleUnitTest(FrustumTriangleTest202, list);
  DeclareSimpleUnitTest(FrustumTriangleTest203, list);
  DeclareSimpleUnitTest(FrustumTriangleTest204, list);
  DeclareSimpleUnitTest(FrustumTriangleTest205, list);
  DeclareSimpleUnitTest(FrustumTriangleTest206, list);
  DeclareSimpleUnitTest(FrustumTriangleTest207, list);
  DeclareSimpleUnitTest(FrustumTriangleTest208, list);
  DeclareSimpleUnitTest(FrustumTriangleTest209, list);
  DeclareSimpleUnitTest(FrustumTriangleTest210, list);
  DeclareSimpleUnitTest(FrustumTriangleTest211, list);
  DeclareSimpleUnitTest(FrustumTriangleTest212, list);
  DeclareSimpleUnitTest(FrustumTriangleTest213, list);
  DeclareSimpleUnitTest(FrustumTriangleTest214, list);
  DeclareSimpleUnitTest(FrustumTriangleTest215, list);
  DeclareSimpleUnitTest(FrustumTriangleTest216, list);
  DeclareSimpleUnitTest(FrustumTriangleTest217, list);
  DeclareSimpleUnitTest(FrustumTriangleTest218, list);
  DeclareSimpleUnitTest(FrustumTriangleTest219, list);
  DeclareSimpleUnitTest(FrustumTriangleTest220, list);
  DeclareSimpleUnitTest(FrustumTriangleTest221, list);
  DeclareSimpleUnitTest(FrustumTriangleTest222, list);
}

void RegisterFrustumAabbTestsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(FrustumAabbTest1, list);
  DeclareSimpleUnitTest(FrustumAabbTest2, list);
  DeclareSimpleUnitTest(FrustumAabbTest3, list);
  DeclareSimpleUnitTest(FrustumAabbTest4, list);
  DeclareSimpleUnitTest(FrustumAabbTest5, list);
  DeclareSimpleUnitTest(FrustumAabbTest6, list);
  DeclareSimpleUnitTest(FrustumAabbTest7, list);
  DeclareSimpleUnitTest(FrustumAabbTest8, list);
  DeclareSimpleUnitTest(FrustumAabbTest9, list);
  DeclareSimpleUnitTest(FrustumAabbTest10, list);
  DeclareSimpleUnitTest(FrustumAabbTest11, list);
  DeclareSimpleUnitTest(FrustumAabbTest12, list);
  DeclareSimpleUnitTest(FrustumAabbTest13, list);
  DeclareSimpleUnitTest(FrustumAabbTest14, list);
  DeclareSimpleUnitTest(FrustumAabbTest15, list);
  DeclareSimpleUnitTest(FrustumAabbTest16, list);
  DeclareSimpleUnitTest(FrustumAabbTest17, list);
  DeclareSimpleUnitTest(FrustumAabbTest18, list);
  DeclareSimpleUnitTest(FrustumAabbTest19, list);
  DeclareSimpleUnitTest(FrustumAabbTest20, list);
  DeclareSimpleUnitTest(FrustumAabbTest21, list);
  DeclareSimpleUnitTest(FrustumAabbTest22, list);
  DeclareSimpleUnitTest(FrustumAabbTest23, list);
  DeclareSimpleUnitTest(FrustumAabbTest24, list);
  DeclareSimpleUnitTest(FrustumAabbTest25, list);
  DeclareSimpleUnitTest(FrustumAabbTest26, list);
  DeclareSimpleUnitTest(FrustumAabbTest27, list);
  DeclareSimpleUnitTest(FrustumAabbTest28, list);
  DeclareSimpleUnitTest(FrustumAabbTest29, list);
  DeclareSimpleUnitTest(FrustumAabbTest30, list);
  DeclareSimpleUnitTest(FrustumAabbTest31, list);
  DeclareSimpleUnitTest(FrustumAabbTest32, list);
  DeclareSimpleUnitTest(FrustumAabbTest33, list);
  DeclareSimpleUnitTest(FrustumAabbTest34, list);
  DeclareSimpleUnitTest(FrustumAabbTest35, list);
  DeclareSimpleUnitTest(FrustumAabbTest36, list);
  DeclareSimpleUnitTest(FrustumAabbTest37, list);
  DeclareSimpleUnitTest(FrustumAabbTest38, list);
  DeclareSimpleUnitTest(FrustumAabbTest39, list);
  DeclareSimpleUnitTest(FrustumAabbTest40, list);
  DeclareSimpleUnitTest(FrustumAabbTest41, list);
  DeclareSimpleUnitTest(FrustumAabbTest42, list);
  DeclareSimpleUnitTest(FrustumAabbTest43, list);
  DeclareSimpleUnitTest(FrustumAabbTest44, list);
  DeclareSimpleUnitTest(FrustumAabbTest45, list);
  DeclareSimpleUnitTest(FrustumAabbTest46, list);
  DeclareSimpleUnitTest(FrustumAabbTest47, list);
  DeclareSimpleUnitTest(FrustumAabbTest48, list);
  DeclareSimpleUnitTest(FrustumAabbTest49, list);
  DeclareSimpleUnitTest(FrustumAabbTest50, list);
  DeclareSimpleUnitTest(FrustumAabbTest51, list);
  DeclareSimpleUnitTest(FrustumAabbTest52, list);
  DeclareSimpleUnitTest(FrustumAabbTest53, list);
  DeclareSimpleUnitTest(FrustumAabbTest54, list);
  DeclareSimpleUnitTest(FrustumAabbTest55, list);
  DeclareSimpleUnitTest(FrustumAabbTest56, list);
  DeclareSimpleUnitTest(FrustumAabbTest57, list);
  DeclareSimpleUnitTest(FrustumAabbTest58, list);
  DeclareSimpleUnitTest(FrustumAabbTest59, list);
  DeclareSimpleUnitTest(FrustumAabbTest60, list);
  DeclareSimpleUnitTest(FrustumAabbTest61, list);
  DeclareSimpleUnitTest(FrustumAabbTest62, list);
  DeclareSimpleUnitTest(FrustumAabbTest63, list);
  DeclareSimpleUnitTest(FrustumAabbTest64, list);
  DeclareSimpleUnitTest(FrustumAabbTest65, list);
  DeclareSimpleUnitTest(FrustumAabbTest66, list);
  DeclareSimpleUnitTest(FrustumAabbTest67, list);
  DeclareSimpleUnitTest(FrustumAabbTest68, list);
  DeclareSimpleUnitTest(FrustumAabbTest69, list);
  DeclareSimpleUnitTest(FrustumAabbTest70, list);
  DeclareSimpleUnitTest(FrustumAabbTest71, list);
  DeclareSimpleUnitTest(FrustumAabbTest72, list);
  DeclareSimpleUnitTest(FrustumAabbTest73, list);
  DeclareSimpleUnitTest(FrustumAabbTest74, list);
  DeclareSimpleUnitTest(FrustumAabbTest75, list);
  DeclareSimpleUnitTest(FrustumAabbTest76, list);
  DeclareSimpleUnitTest(FrustumAabbTest77, list);
  DeclareSimpleUnitTest(FrustumAabbTest78, list);
  DeclareSimpleUnitTest(FrustumAabbTest79, list);
  DeclareSimpleUnitTest(FrustumAabbTest80, list);
  DeclareSimpleUnitTest(FrustumAabbTest81, list);
  DeclareSimpleUnitTest(FrustumAabbTest82, list);
  DeclareSimpleUnitTest(FrustumAabbTest83, list);
  DeclareSimpleUnitTest(FrustumAabbTest84, list);
  DeclareSimpleUnitTest(FrustumAabbTest85, list);
  DeclareSimpleUnitTest(FrustumAabbTest86, list);
  DeclareSimpleUnitTest(FrustumAabbTest87, list);
  DeclareSimpleUnitTest(FrustumAabbTest88, list);
  DeclareSimpleUnitTest(FrustumAabbTest89, list);
  DeclareSimpleUnitTest(FrustumAabbTest90, list);
  DeclareSimpleUnitTest(FrustumAabbTest91, list);
  DeclareSimpleUnitTest(FrustumAabbTest92, list);
  DeclareSimpleUnitTest(FrustumAabbTest93, list);
  DeclareSimpleUnitTest(FrustumAabbTest94, list);
  DeclareSimpleUnitTest(FrustumAabbTest95, list);
  DeclareSimpleUnitTest(FrustumAabbTest96, list);
  DeclareSimpleUnitTest(FrustumAabbTest97, list);
  DeclareSimpleUnitTest(FrustumAabbTest98, list);
  DeclareSimpleUnitTest(FrustumAabbTest99, list);
  DeclareSimpleUnitTest(FrustumAabbTest100, list);
  DeclareSimpleUnitTest(FrustumAabbTest101, list);
  DeclareSimpleUnitTest(FrustumAabbTest102, list);
  DeclareSimpleUnitTest(FrustumAabbTest103, list);
  DeclareSimpleUnitTest(FrustumAabbTest104, list);
  DeclareSimpleUnitTest(FrustumAabbTest105, list);
  DeclareSimpleUnitTest(FrustumAabbTest106, list);
  DeclareSimpleUnitTest(FrustumAabbTest107, list);
  DeclareSimpleUnitTest(FrustumAabbTest108, list);
  DeclareSimpleUnitTest(FrustumAabbTest109, list);
  DeclareSimpleUnitTest(FrustumAabbTest110, list);
  DeclareSimpleUnitTest(FrustumAabbTest111, list);
  DeclareSimpleUnitTest(FrustumAabbTest112, list);
  DeclareSimpleUnitTest(FrustumAabbTest113, list);
  DeclareSimpleUnitTest(FrustumAabbTest114, list);
  DeclareSimpleUnitTest(FrustumAabbTest115, list);
  DeclareSimpleUnitTest(FrustumAabbTest116, list);
  DeclareSimpleUnitTest(FrustumAabbTest117, list);
  DeclareSimpleUnitTest(FrustumAabbTest118, list);
  DeclareSimpleUnitTest(FrustumAabbTest119, list);
  DeclareSimpleUnitTest(FrustumAabbTest120, list);
  DeclareSimpleUnitTest(FrustumAabbTest121, list);
  DeclareSimpleUnitTest(FrustumAabbTest122, list);
  DeclareSimpleUnitTest(FrustumAabbTest123, list);
  DeclareSimpleUnitTest(FrustumAabbTest124, list);
  DeclareSimpleUnitTest(FrustumAabbTest125, list);
  DeclareSimpleUnitTest(FrustumAabbTest126, list);
  DeclareSimpleUnitTest(FrustumAabbTest127, list);
  DeclareSimpleUnitTest(FrustumAabbTest128, list);
  DeclareSimpleUnitTest(FrustumAabbTest129, list);
  DeclareSimpleUnitTest(FrustumAabbTest130, list);
  DeclareSimpleUnitTest(FrustumAabbTest131, list);
  DeclareSimpleUnitTest(FrustumAabbTest132, list);
  DeclareSimpleUnitTest(FrustumAabbTest133, list);
  DeclareSimpleUnitTest(FrustumAabbTest134, list);
  DeclareSimpleUnitTest(FrustumAabbTest135, list);
  DeclareSimpleUnitTest(FrustumAabbTest136, list);
  DeclareSimpleUnitTest(FrustumAabbTest137, list);
  DeclareSimpleUnitTest(FrustumAabbTest138, list);
  DeclareSimpleUnitTest(FrustumAabbTest139, list);
  DeclareSimpleUnitTest(FrustumAabbTest140, list);
  DeclareSimpleUnitTest(FrustumAabbTest141, list);
  DeclareSimpleUnitTest(FrustumAabbTest142, list);
  DeclareSimpleUnitTest(FrustumAabbTest143, list);
  DeclareSimpleUnitTest(FrustumAabbTest144, list);
  DeclareSimpleUnitTest(FrustumAabbTest145, list);
  DeclareSimpleUnitTest(FrustumAabbTest146, list);
  DeclareSimpleUnitTest(FrustumAabbTest147, list);
  DeclareSimpleUnitTest(FrustumAabbTest148, list);
  DeclareSimpleUnitTest(FrustumAabbTest149, list);
  DeclareSimpleUnitTest(FrustumAabbTest150, list);
  DeclareSimpleUnitTest(FrustumAabbTest151, list);
  DeclareSimpleUnitTest(FrustumAabbTest152, list);
  DeclareSimpleUnitTest(FrustumAabbTest153, list);
  DeclareSimpleUnitTest(FrustumAabbTest154, list);
  DeclareSimpleUnitTest(FrustumAabbTest155, list);
  DeclareSimpleUnitTest(FrustumAabbTest156, list);
  DeclareSimpleUnitTest(FrustumAabbTest157, list);
  DeclareSimpleUnitTest(FrustumAabbTest158, list);
  DeclareSimpleUnitTest(FrustumAabbTest159, list);
  DeclareSimpleUnitTest(FrustumAabbTest160, list);
  DeclareSimpleUnitTest(FrustumAabbTest161, list);
  DeclareSimpleUnitTest(FrustumAabbTest162, list);
  DeclareSimpleUnitTest(FrustumAabbTest163, list);
  DeclareSimpleUnitTest(FrustumAabbTest164, list);
  DeclareSimpleUnitTest(FrustumAabbTest165, list);
  DeclareSimpleUnitTest(FrustumAabbTest166, list);
  DeclareSimpleUnitTest(FrustumAabbTest167, list);
  DeclareSimpleUnitTest(FrustumAabbTest168, list);
  DeclareSimpleUnitTest(FrustumAabbTest169, list);
  DeclareSimpleUnitTest(FrustumAabbTest170, list);
  DeclareSimpleUnitTest(FrustumAabbTest171, list);
  DeclareSimpleUnitTest(FrustumAabbTest172, list);
  DeclareSimpleUnitTest(FrustumAabbTest173, list);
  DeclareSimpleUnitTest(FrustumAabbTest174, list);
  DeclareSimpleUnitTest(FrustumAabbTest175, list);
  DeclareSimpleUnitTest(FrustumAabbTest176, list);
  DeclareSimpleUnitTest(FrustumAabbTest177, list);
  DeclareSimpleUnitTest(FrustumAabbTest178, list);
  DeclareSimpleUnitTest(FrustumAabbTest179, list);
  DeclareSimpleUnitTest(FrustumAabbTest180, list);
  DeclareSimpleUnitTest(FrustumAabbTest181, list);
  DeclareSimpleUnitTest(FrustumAabbTest182, list);
  DeclareSimpleUnitTest(FrustumAabbTest183, list);
  DeclareSimpleUnitTest(FrustumAabbTest184, list);
  DeclareSimpleUnitTest(FrustumAabbTest185, list);
  DeclareSimpleUnitTest(FrustumAabbTest186, list);
  DeclareSimpleUnitTest(FrustumAabbTest187, list);
  DeclareSimpleUnitTest(FrustumAabbTest188, list);
  DeclareSimpleUnitTest(FrustumAabbTest189, list);
  DeclareSimpleUnitTest(FrustumAabbTest190, list);
  DeclareSimpleUnitTest(FrustumAabbTest191, list);
  DeclareSimpleUnitTest(FrustumAabbTest192, list);
  DeclareSimpleUnitTest(FrustumAabbTest193, list);
  DeclareSimpleUnitTest(FrustumAabbTest194, list);
  DeclareSimpleUnitTest(FrustumAabbTest195, list);
  DeclareSimpleUnitTest(FrustumAabbTest196, list);
  DeclareSimpleUnitTest(FrustumAabbTest197, list);
  DeclareSimpleUnitTest(FrustumAabbTest198, list);
  DeclareSimpleUnitTest(FrustumAabbTest199, list);
  DeclareSimpleUnitTest(FrustumAabbTest200, list);
  DeclareSimpleUnitTest(FrustumAabbTest201, list);
  DeclareSimpleUnitTest(FrustumAabbTest202, list);
  DeclareSimpleUnitTest(FrustumAabbTest203, list);
  DeclareSimpleUnitTest(FrustumAabbTest204, list);
  DeclareSimpleUnitTest(FrustumAabbTest205, list);
  DeclareSimpleUnitTest(FrustumAabbTest206, list);
  DeclareSimpleUnitTest(FrustumAabbTest207, list);
  DeclareSimpleUnitTest(FrustumAabbTest208, list);
  DeclareSimpleUnitTest(FrustumAabbTest209, list);
  DeclareSimpleUnitTest(FrustumAabbTest210, list);
  DeclareSimpleUnitTest(FrustumAabbTest211, list);
  DeclareSimpleUnitTest(FrustumAabbTest212, list);
  DeclareSimpleUnitTest(FrustumAabbTest213, list);
  DeclareSimpleUnitTest(FrustumAabbTest214, list);
  DeclareSimpleUnitTest(FrustumAabbTest215, list);
  DeclareSimpleUnitTest(FrustumAabbTest216, list);
  DeclareSimpleUnitTest(FrustumAabbTest217, list);
  DeclareSimpleUnitTest(FrustumAabbTest218, list);
  DeclareSimpleUnitTest(FrustumAabbTest219, list);
  DeclareSimpleUnitTest(FrustumAabbTest220, list);
  DeclareSimpleUnitTest(FrustumAabbTest221, list);
  DeclareSimpleUnitTest(FrustumAabbTest222, list);
  DeclareSimpleUnitTest(FrustumAabbTest223, list);
  DeclareSimpleUnitTest(FrustumAabbTest224, list);
  DeclareSimpleUnitTest(FrustumAabbTest225, list);
  DeclareSimpleUnitTest(FrustumAabbTest226, list);
  DeclareSimpleUnitTest(FrustumAabbTest227, list);
  DeclareSimpleUnitTest(FrustumAabbTest228, list);
  DeclareSimpleUnitTest(FrustumAabbTest229, list);
  DeclareSimpleUnitTest(FrustumAabbTest230, list);
  DeclareSimpleUnitTest(FrustumAabbTest231, list);
  DeclareSimpleUnitTest(FrustumAabbTest232, list);
  DeclareSimpleUnitTest(FrustumAabbTest233, list);
  DeclareSimpleUnitTest(FrustumAabbTest234, list);
  DeclareSimpleUnitTest(FrustumAabbTest235, list);
  DeclareSimpleUnitTest(FrustumAabbTest236, list);
  DeclareSimpleUnitTest(FrustumAabbTest237, list);
  DeclareSimpleUnitTest(FrustumAabbTest238, list);
  DeclareSimpleUnitTest(FrustumAabbTest239, list);
  DeclareSimpleUnitTest(FrustumAabbTest240, list);
  DeclareSimpleUnitTest(FrustumAabbTest241, list);
  DeclareSimpleUnitTest(FrustumAabbTest242, list);
  DeclareSimpleUnitTest(FrustumAabbTest243, list);
  DeclareSimpleUnitTest(FrustumAabbTest244, list);
  DeclareSimpleUnitTest(FrustumAabbTest245, list);
  DeclareSimpleUnitTest(FrustumAabbTest246, list);
}

void RegisterFrustumSphereTestsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(FrustumSphereTest1, list);
  DeclareSimpleUnitTest(FrustumSphereTest2, list);
  DeclareSimpleUnitTest(FrustumSphereTest3, list);
  DeclareSimpleUnitTest(FrustumSphereTest4, list);
  DeclareSimpleUnitTest(FrustumSphereTest5, list);
  DeclareSimpleUnitTest(FrustumSphereTest6, list);
  DeclareSimpleUnitTest(FrustumSphereTest7, list);
  DeclareSimpleUnitTest(FrustumSphereTest8, list);
  DeclareSimpleUnitTest(FrustumSphereTest9, list);
  DeclareSimpleUnitTest(FrustumSphereTest10, list);
  DeclareSimpleUnitTest(FrustumSphereTest11, list);
  DeclareSimpleUnitTest(FrustumSphereTest12, list);
  DeclareSimpleUnitTest(FrustumSphereTest13, list);
  DeclareSimpleUnitTest(FrustumSphereTest14, list);
  DeclareSimpleUnitTest(FrustumSphereTest15, list);
  DeclareSimpleUnitTest(FrustumSphereTest16, list);
  DeclareSimpleUnitTest(FrustumSphereTest17, list);
  DeclareSimpleUnitTest(FrustumSphereTest18, list);
  DeclareSimpleUnitTest(FrustumSphereTest19, list);
  DeclareSimpleUnitTest(FrustumSphereTest20, list);
  DeclareSimpleUnitTest(FrustumSphereTest21, list);
  DeclareSimpleUnitTest(FrustumSphereTest22, list);
  DeclareSimpleUnitTest(FrustumSphereTest23, list);
  DeclareSimpleUnitTest(FrustumSphereTest24, list);
  DeclareSimpleUnitTest(FrustumSphereTest25, list);
  DeclareSimpleUnitTest(FrustumSphereTest26, list);
  DeclareSimpleUnitTest(FrustumSphereTest27, list);
  DeclareSimpleUnitTest(FrustumSphereTest28, list);
  DeclareSimpleUnitTest(FrustumSphereTest29, list);
  DeclareSimpleUnitTest(FrustumSphereTest30, list);
  DeclareSimpleUnitTest(FrustumSphereTest31, list);
  DeclareSimpleUnitTest(FrustumSphereTest32, list);
  DeclareSimpleUnitTest(FrustumSphereTest33, list);
  DeclareSimpleUnitTest(FrustumSphereTest34, list);
  DeclareSimpleUnitTest(FrustumSphereTest35, list);
  DeclareSimpleUnitTest(FrustumSphereTest36, list);
  DeclareSimpleUnitTest(FrustumSphereTest37, list);
  DeclareSimpleUnitTest(FrustumSphereTest38, list);
  DeclareSimpleUnitTest(FrustumSphereTest39, list);
  DeclareSimpleUnitTest(FrustumSphereTest40, list);
  DeclareSimpleUnitTest(FrustumSphereTest41, list);
  DeclareSimpleUnitTest(FrustumSphereTest42, list);
  DeclareSimpleUnitTest(FrustumSphereTest43, list);
  DeclareSimpleUnitTest(FrustumSphereTest44, list);
  DeclareSimpleUnitTest(FrustumSphereTest45, list);
  DeclareSimpleUnitTest(FrustumSphereTest46, list);
  DeclareSimpleUnitTest(FrustumSphereTest47, list);
  DeclareSimpleUnitTest(FrustumSphereTest48, list);
  DeclareSimpleUnitTest(FrustumSphereTest49, list);
  DeclareSimpleUnitTest(FrustumSphereTest50, list);
  DeclareSimpleUnitTest(FrustumSphereTest51, list);
  DeclareSimpleUnitTest(FrustumSphereTest52, list);
  DeclareSimpleUnitTest(FrustumSphereTest53, list);
  DeclareSimpleUnitTest(FrustumSphereTest54, list);
  DeclareSimpleUnitTest(FrustumSphereTest55, list);
  DeclareSimpleUnitTest(FrustumSphereTest56, list);
  DeclareSimpleUnitTest(FrustumSphereTest57, list);
  DeclareSimpleUnitTest(FrustumSphereTest58, list);
  DeclareSimpleUnitTest(FrustumSphereTest59, list);
  DeclareSimpleUnitTest(FrustumSphereTest60, list);
  DeclareSimpleUnitTest(FrustumSphereTest61, list);
  DeclareSimpleUnitTest(FrustumSphereTest62, list);
  DeclareSimpleUnitTest(FrustumSphereTest63, list);
  DeclareSimpleUnitTest(FrustumSphereTest64, list);
  DeclareSimpleUnitTest(FrustumSphereTest65, list);
  DeclareSimpleUnitTest(FrustumSphereTest66, list);
  DeclareSimpleUnitTest(FrustumSphereTest67, list);
  DeclareSimpleUnitTest(FrustumSphereTest68, list);
  DeclareSimpleUnitTest(FrustumSphereTest69, list);
  DeclareSimpleUnitTest(FrustumSphereTest70, list);
  DeclareSimpleUnitTest(FrustumSphereTest71, list);
  DeclareSimpleUnitTest(FrustumSphereTest72, list);
  DeclareSimpleUnitTest(FrustumSphereTest73, list);
  DeclareSimpleUnitTest(FrustumSphereTest74, list);
  DeclareSimpleUnitTest(FrustumSphereTest75, list);
  DeclareSimpleUnitTest(FrustumSphereTest76, list);
  DeclareSimpleUnitTest(FrustumSphereTest77, list);
  DeclareSimpleUnitTest(FrustumSphereTest78, list);
  DeclareSimpleUnitTest(FrustumSphereTest79, list);
  DeclareSimpleUnitTest(FrustumSphereTest80, list);
  DeclareSimpleUnitTest(FrustumSphereTest81, list);
  DeclareSimpleUnitTest(FrustumSphereTest82, list);
  DeclareSimpleUnitTest(FrustumSphereTest83, list);
  DeclareSimpleUnitTest(FrustumSphereTest84, list);
  DeclareSimpleUnitTest(FrustumSphereTest85, list);
  DeclareSimpleUnitTest(FrustumSphereTest86, list);
  DeclareSimpleUnitTest(FrustumSphereTest87, list);
  DeclareSimpleUnitTest(FrustumSphereTest88, list);
  DeclareSimpleUnitTest(FrustumSphereTest89, list);
  DeclareSimpleUnitTest(FrustumSphereTest90, list);
  DeclareSimpleUnitTest(FrustumSphereTest91, list);
  DeclareSimpleUnitTest(FrustumSphereTest92, list);
  DeclareSimpleUnitTest(FrustumSphereTest93, list);
  DeclareSimpleUnitTest(FrustumSphereTest94, list);
  DeclareSimpleUnitTest(FrustumSphereTest95, list);
  DeclareSimpleUnitTest(FrustumSphereTest96, list);
  DeclareSimpleUnitTest(FrustumSphereTest97, list);
  DeclareSimpleUnitTest(FrustumSphereTest98, list);
  DeclareSimpleUnitTest(FrustumSphereTest99, list);
  DeclareSimpleUnitTest(FrustumSphereTest100, list);
  DeclareSimpleUnitTest(FrustumSphereTest101, list);
  DeclareSimpleUnitTest(FrustumSphereTest102, list);
  DeclareSimpleUnitTest(FrustumSphereTest103, list);
  DeclareSimpleUnitTest(FrustumSphereTest104, list);
  DeclareSimpleUnitTest(FrustumSphereTest105, list);
  DeclareSimpleUnitTest(FrustumSphereTest106, list);
  DeclareSimpleUnitTest(FrustumSphereTest107, list);
  DeclareSimpleUnitTest(FrustumSphereTest108, list);
  DeclareSimpleUnitTest(FrustumSphereTest109, list);
  DeclareSimpleUnitTest(FrustumSphereTest110, list);
  DeclareSimpleUnitTest(FrustumSphereTest111, list);
  DeclareSimpleUnitTest(FrustumSphereTest112, list);
  DeclareSimpleUnitTest(FrustumSphereTest113, list);
  DeclareSimpleUnitTest(FrustumSphereTest114, list);
  DeclareSimpleUnitTest(FrustumSphereTest115, list);
  DeclareSimpleUnitTest(FrustumSphereTest116, list);
  DeclareSimpleUnitTest(FrustumSphereTest117, list);
  DeclareSimpleUnitTest(FrustumSphereTest118, list);
  DeclareSimpleUnitTest(FrustumSphereTest119, list);
  DeclareSimpleUnitTest(FrustumSphereTest120, list);
  DeclareSimpleUnitTest(FrustumSphereTest121, list);
  DeclareSimpleUnitTest(FrustumSphereTest122, list);
  DeclareSimpleUnitTest(FrustumSphereTest123, list);
  DeclareSimpleUnitTest(FrustumSphereTest124, list);
  DeclareSimpleUnitTest(FrustumSphereTest125, list);
  DeclareSimpleUnitTest(FrustumSphereTest126, list);
  DeclareSimpleUnitTest(FrustumSphereTest127, list);
  DeclareSimpleUnitTest(FrustumSphereTest128, list);
  DeclareSimpleUnitTest(FrustumSphereTest129, list);
  DeclareSimpleUnitTest(FrustumSphereTest130, list);
  DeclareSimpleUnitTest(FrustumSphereTest131, list);
  DeclareSimpleUnitTest(FrustumSphereTest132, list);
  DeclareSimpleUnitTest(FrustumSphereTest133, list);
  DeclareSimpleUnitTest(FrustumSphereTest134, list);
  DeclareSimpleUnitTest(FrustumSphereTest135, list);
  DeclareSimpleUnitTest(FrustumSphereTest136, list);
  DeclareSimpleUnitTest(FrustumSphereTest137, list);
  DeclareSimpleUnitTest(FrustumSphereTest138, list);
  DeclareSimpleUnitTest(FrustumSphereTest139, list);
  DeclareSimpleUnitTest(FrustumSphereTest140, list);
  DeclareSimpleUnitTest(FrustumSphereTest141, list);
  DeclareSimpleUnitTest(FrustumSphereTest142, list);
  DeclareSimpleUnitTest(FrustumSphereTest143, list);
  DeclareSimpleUnitTest(FrustumSphereTest144, list);
  DeclareSimpleUnitTest(FrustumSphereTest145, list);
  DeclareSimpleUnitTest(FrustumSphereTest146, list);
  DeclareSimpleUnitTest(FrustumSphereTest147, list);
  DeclareSimpleUnitTest(FrustumSphereTest148, list);
  DeclareSimpleUnitTest(FrustumSphereTest149, list);
  DeclareSimpleUnitTest(FrustumSphereTest150, list);
  DeclareSimpleUnitTest(FrustumSphereTest151, list);
  DeclareSimpleUnitTest(FrustumSphereTest152, list);
  DeclareSimpleUnitTest(FrustumSphereTest153, list);
  DeclareSimpleUnitTest(FrustumSphereTest154, list);
  DeclareSimpleUnitTest(FrustumSphereTest155, list);
  DeclareSimpleUnitTest(FrustumSphereTest156, list);
  DeclareSimpleUnitTest(FrustumSphereTest157, list);
  DeclareSimpleUnitTest(FrustumSphereTest158, list);
  DeclareSimpleUnitTest(FrustumSphereTest159, list);
  DeclareSimpleUnitTest(FrustumSphereTest160, list);
  DeclareSimpleUnitTest(FrustumSphereTest161, list);
  DeclareSimpleUnitTest(FrustumSphereTest162, list);
  DeclareSimpleUnitTest(FrustumSphereTest163, list);
  DeclareSimpleUnitTest(FrustumSphereTest164, list);
  DeclareSimpleUnitTest(FrustumSphereTest165, list);
  DeclareSimpleUnitTest(FrustumSphereTest166, list);
  DeclareSimpleUnitTest(FrustumSphereTest167, list);
  DeclareSimpleUnitTest(FrustumSphereTest168, list);
  DeclareSimpleUnitTest(FrustumSphereTest169, list);
  DeclareSimpleUnitTest(FrustumSphereTest170, list);
  DeclareSimpleUnitTest(FrustumSphereTest171, list);
  DeclareSimpleUnitTest(FrustumSphereTest172, list);
  DeclareSimpleUnitTest(FrustumSphereTest173, list);
  DeclareSimpleUnitTest(FrustumSphereTest174, list);
  DeclareSimpleUnitTest(FrustumSphereTest175, list);
  DeclareSimpleUnitTest(FrustumSphereTest176, list);
  DeclareSimpleUnitTest(FrustumSphereTest177, list);
  DeclareSimpleUnitTest(FrustumSphereTest178, list);
  DeclareSimpleUnitTest(FrustumSphereTest179, list);
  DeclareSimpleUnitTest(FrustumSphereTest180, list);
  DeclareSimpleUnitTest(FrustumSphereTest181, list);
  DeclareSimpleUnitTest(FrustumSphereTest182, list);
  DeclareSimpleUnitTest(FrustumSphereTest183, list);
  DeclareSimpleUnitTest(FrustumSphereTest184, list);
  DeclareSimpleUnitTest(FrustumSphereTest185, list);
  DeclareSimpleUnitTest(FrustumSphereTest186, list);
  DeclareSimpleUnitTest(FrustumSphereTest187, list);
  DeclareSimpleUnitTest(FrustumSphereTest188, list);
  DeclareSimpleUnitTest(FrustumSphereTest189, list);
  DeclareSimpleUnitTest(FrustumSphereTest190, list);
  DeclareSimpleUnitTest(FrustumSphereTest191, list);
  DeclareSimpleUnitTest(FrustumSphereTest192, list);
  DeclareSimpleUnitTest(FrustumSphereTest193, list);
  DeclareSimpleUnitTest(FrustumSphereTest194, list);
  DeclareSimpleUnitTest(FrustumSphereTest195, list);
  DeclareSimpleUnitTest(FrustumSphereTest196, list);
  DeclareSimpleUnitTest(FrustumSphereTest197, list);
  DeclareSimpleUnitTest(FrustumSphereTest198, list);
  DeclareSimpleUnitTest(FrustumSphereTest199, list);
  DeclareSimpleUnitTest(FrustumSphereTest200, list);
  DeclareSimpleUnitTest(FrustumSphereTest201, list);
  DeclareSimpleUnitTest(FrustumSphereTest202, list);
  DeclareSimpleUnitTest(FrustumSphereTest203, list);
  DeclareSimpleUnitTest(FrustumSphereTest204, list);
  DeclareSimpleUnitTest(FrustumSphereTest205, list);
  DeclareSimpleUnitTest(FrustumSphereTest206, list);
  DeclareSimpleUnitTest(FrustumSphereTest207, list);
  DeclareSimpleUnitTest(FrustumSphereTest208, list);
  DeclareSimpleUnitTest(FrustumSphereTest209, list);
  DeclareSimpleUnitTest(FrustumSphereTest210, list);
  DeclareSimpleUnitTest(FrustumSphereTest211, list);
  DeclareSimpleUnitTest(FrustumSphereTest212, list);
  DeclareSimpleUnitTest(FrustumSphereTest213, list);
  DeclareSimpleUnitTest(FrustumSphereTest214, list);
  DeclareSimpleUnitTest(FrustumSphereTest215, list);
  DeclareSimpleUnitTest(FrustumSphereTest216, list);
  DeclareSimpleUnitTest(FrustumSphereTest217, list);
  DeclareSimpleUnitTest(FrustumSphereTest218, list);
  DeclareSimpleUnitTest(FrustumSphereTest219, list);
  DeclareSimpleUnitTest(FrustumSphereTest220, list);
  DeclareSimpleUnitTest(FrustumSphereTest221, list);
  DeclareSimpleUnitTest(FrustumSphereTest222, list);
}

void RegisterSphereSphereTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereSphereTest1, list);
  DeclareSimpleUnitTest(SphereSphereTest2, list);
  DeclareSimpleUnitTest(SphereSphereTest3, list);
  DeclareSimpleUnitTest(SphereSphereTest4, list);
  DeclareSimpleUnitTest(SphereSphereTest5, list);
  DeclareSimpleUnitTest(SphereSphereTest6, list);
  DeclareSimpleUnitTest(SphereSphereTest7, list);
  DeclareSimpleUnitTest(SphereSphereTest8, list);
  DeclareSimpleUnitTest(SphereSphereTest9, list);
  DeclareSimpleUnitTest(SphereSphereTest10, list);
  DeclareSimpleUnitTest(SphereSphereTest11, list);
  DeclareSimpleUnitTest(SphereSphereTest12, list);
  DeclareSimpleUnitTest(SphereSphereTest13, list);
  DeclareSimpleUnitTest(SphereSphereTest14, list);
  DeclareSimpleUnitTest(SphereSphereTest15, list);
}

void RegisterAabbAabbTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(AabbAabbTest1, list);
  DeclareSimpleUnitTest(AabbAabbTest2, list);
  DeclareSimpleUnitTest(AabbAabbTest3, list);
  DeclareSimpleUnitTest(AabbAabbTest4, list);
  DeclareSimpleUnitTest(AabbAabbTest5, list);
  DeclareSimpleUnitTest(AabbAabbTest6, list);
  DeclareSimpleUnitTest(AabbAabbTest7, list);
  DeclareSimpleUnitTest(AabbAabbTest8, list);
  DeclareSimpleUnitTest(AabbAabbTest9, list);
  DeclareSimpleUnitTest(AabbAabbTest10, list);
  DeclareSimpleUnitTest(AabbAabbTest11, list);
  DeclareSimpleUnitTest(AabbAabbTest12, list);
  DeclareSimpleUnitTest(AabbAabbTest13, list);
  DeclareSimpleUnitTest(AabbAabbTest14, list);
  DeclareSimpleUnitTest(AabbAabbTest15, list);
  DeclareSimpleUnitTest(AabbAabbTest16, list);
  DeclareSimpleUnitTest(AabbAabbTest17, list);
  DeclareSimpleUnitTest(AabbAabbTest18, list);
  DeclareSimpleUnitTest(AabbAabbTest19, list);
  DeclareSimpleUnitTest(AabbAabbTest20, list);
  DeclareSimpleUnitTest(AabbAabbTest21, list);
  DeclareSimpleUnitTest(AabbAabbTest22, list);
  DeclareSimpleUnitTest(AabbAabbTest23, list);
  DeclareSimpleUnitTest(AabbAabbTest24, list);
  DeclareSimpleUnitTest(AabbAabbTest25, list);
  DeclareSimpleUnitTest(AabbAabbTest26, list);
  DeclareSimpleUnitTest(AabbAabbTest27, list);
  DeclareSimpleUnitTest(AabbAabbTest28, list);
  DeclareSimpleUnitTest(AabbAabbTest29, list);
  DeclareSimpleUnitTest(AabbAabbTest30, list);
  DeclareSimpleUnitTest(AabbAabbTest31, list);
  DeclareSimpleUnitTest(AabbAabbTest32, list);
  DeclareSimpleUnitTest(AabbAabbTest33, list);
  DeclareSimpleUnitTest(AabbAabbTest34, list);
  DeclareSimpleUnitTest(AabbAabbTest35, list);
  DeclareSimpleUnitTest(AabbAabbTest36, list);
  DeclareSimpleUnitTest(AabbAabbTest37, list);
  DeclareSimpleUnitTest(AabbAabbTest38, list);
  DeclareSimpleUnitTest(AabbAabbTest39, list);
  DeclareSimpleUnitTest(AabbAabbTest40, list);
  DeclareSimpleUnitTest(AabbAabbTest41, list);
  DeclareSimpleUnitTest(AabbAabbTest42, list);
  DeclareSimpleUnitTest(AabbAabbTest43, list);
  DeclareSimpleUnitTest(AabbAabbTest44, list);
  DeclareSimpleUnitTest(AabbAabbTest45, list);
  DeclareSimpleUnitTest(AabbAabbTest46, list);
  DeclareSimpleUnitTest(AabbAabbTest47, list);
  DeclareSimpleUnitTest(AabbAabbTest48, list);
  DeclareSimpleUnitTest(AabbAabbTest49, list);
  DeclareSimpleUnitTest(AabbAabbTest50, list);
  DeclareSimpleUnitTest(AabbAabbTest51, list);
  DeclareSimpleUnitTest(AabbAabbTest52, list);
  DeclareSimpleUnitTest(AabbAabbTest53, list);
  DeclareSimpleUnitTest(AabbAabbTest54, list);
  DeclareSimpleUnitTest(AabbAabbTest55, list);
  DeclareSimpleUnitTest(AabbAabbTest56, list);
  DeclareSimpleUnitTest(AabbAabbTest57, list);
  DeclareSimpleUnitTest(AabbAabbTest58, list);
  DeclareSimpleUnitTest(AabbAabbTest59, list);
  DeclareSimpleUnitTest(AabbAabbTest60, list);
  DeclareSimpleUnitTest(AabbAabbTest61, list);
  DeclareSimpleUnitTest(AabbAabbTest62, list);
  DeclareSimpleUnitTest(AabbAabbTest63, list);
  DeclareSimpleUnitTest(AabbAabbTest64, list);
  DeclareSimpleUnitTest(AabbAabbTest65, list);
  DeclareSimpleUnitTest(AabbAabbTest66, list);
  DeclareSimpleUnitTest(AabbAabbTest67, list);
  DeclareSimpleUnitTest(AabbAabbTest68, list);
  DeclareSimpleUnitTest(AabbAabbTest69, list);
  DeclareSimpleUnitTest(AabbAabbTest70, list);
  DeclareSimpleUnitTest(AabbAabbTest71, list);
  DeclareSimpleUnitTest(AabbAabbTest72, list);
  DeclareSimpleUnitTest(AabbAabbTest73, list);
  DeclareSimpleUnitTest(AabbAabbTest74, list);
  DeclareSimpleUnitTest(AabbAabbTest75, list);
  DeclareSimpleUnitTest(AabbAabbTest76, list);
  DeclareSimpleUnitTest(AabbAabbTest77, list);
  DeclareSimpleUnitTest(AabbAabbTest78, list);
  DeclareSimpleUnitTest(AabbAabbTest79, list);
  DeclareSimpleUnitTest(AabbAabbTest80, list);
  DeclareSimpleUnitTest(AabbAabbTest81, list);
  DeclareSimpleUnitTest(AabbAabbTest82, list);
  DeclareSimpleUnitTest(AabbAabbTest83, list);
  DeclareSimpleUnitTest(AabbAabbTest84, list);
}

void InitializeAssignment1Tests()
{
  mTestFns.push_back(AssignmentUnitTestList());
  AssignmentUnitTestList& list = mTestFns[0];
  
  RegisterProjectPointOnPlaneTests(list);
  RegisterBarycentricLineTests(list);
  RegisterBarycentricTriangleTests(list);
  RegisterPlaneSetTests(list);
  RegisterPointPlaneTests(list);
  RegisterPointSphereTests(list);
  RegisterPointAabbTests(list);
  RegisterRayPlaneTests(list);
  RegisterRayTriangleTests(list);
  RegisterRaySphereTests(list);
  RegisterRayAabbTests(list);
  RegisterPlaneTriangleTests(list);
  RegisterPlaneSphereTests(list);
  RegisterPlaneAabbTests(list);
  RegisterFrustumTriangleTestsTests(list);
  RegisterFrustumAabbTestsTests(list);
  RegisterFrustumSphereTestsTests(list);
  RegisterSphereSphereTests(list);
  RegisterAabbAabbTests(list);
}
