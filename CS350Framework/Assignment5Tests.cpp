#include "Precompiled.hpp"
#include "Application.hpp"
#include "Shapes.hpp"
#include "Geometry.hpp"
#include "Math/Utilities.hpp"
#include "DebugDraw.hpp"
#include "Components.hpp"
#include "SimpleNSquared.hpp"
#include "UnitTests.hpp"
#include "DynamicAabbTree.hpp"
#include "Gjk.hpp"

#include <iostream>

class PointsSupportShapeUnitTests : public PointsSupportShape
{
public:
  PointsSupportShapeUnitTests(PointsSupportShape* base)
  {
    mLocalSpacePoints = base->mLocalSpacePoints;
    mScale = base->mScale;
    mRotation = base->mRotation;
    mTranslation = base->mTranslation;
  }

  void DebugDraw(const Vector4& color) const override
  {
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);

    for (int i = 0; i < static_cast<int>(mLocalSpacePoints.size()); ++i)
    {
      Vector3 worldPoint = Math::TransformPoint(transform, mLocalSpacePoints[i]);
      DebugShape& shape = gDebugDrawer->DrawPoint(worldPoint);
      shape.Color(color);
    }
  }
};

class SphereSupportShapeUnitTests : public SphereSupportShape
{
public:
  SphereSupportShapeUnitTests(SphereSupportShape* base)
  {
    mSphere = base->mSphere;
  }
};

class ObbSupportShapeUnitTests : public ObbSupportShape
{
public:
  ObbSupportShapeUnitTests(ObbSupportShape* base)
  {
    mScale = base->mScale;
    mRotation = base->mRotation;
    mTranslation = base->mTranslation;
  }
};

struct SupportShapeWrapper
{
  SupportShape* mStudentShape;
  SupportShape* mTeacherShape;
};

struct SupportShapeContainer
{
  std::vector<SupportShapeWrapper> mShapes;

  void Add(PointsSupportShape* shape)
  {
    SupportShapeWrapper wrapper;
    wrapper.mStudentShape = shape;
    wrapper.mTeacherShape = new PointsSupportShapeUnitTests(shape);
    mShapes.push_back(wrapper);
  }

  void Add(SphereSupportShape* shape)
  {
    SupportShapeWrapper wrapper;
    wrapper.mStudentShape = shape;
    wrapper.mTeacherShape = new SphereSupportShapeUnitTests(shape);
    mShapes.push_back(wrapper);
  }

  void Add(ObbSupportShape* shape)
  {
    SupportShapeWrapper wrapper;
    wrapper.mStudentShape = shape;
    wrapper.mTeacherShape = new ObbSupportShapeUnitTests(shape);
    mShapes.push_back(wrapper);
  }

  ~SupportShapeContainer()
  {
    for(size_t i = 0; i < mShapes.size(); ++i)
    {
      SupportShapeWrapper& wrapper = mShapes[i];
      delete wrapper.mStudentShape;
      delete wrapper.mTeacherShape;
    }
  }
};
SupportShapeContainer mSupportShapesContainer;

void PrintGjkResults(SupportShape& shape1, SupportShape& shape2, size_t iterations, int debuggingIndex, FILE* file, bool drawResults = false)
{
  float epsilon = 0.001f;
  Gjk gjk;
  Gjk::CsoPoint closestPoint;
  bool result = gjk.Intersect(&shape1, &shape2, static_cast<unsigned int>(iterations), closestPoint, epsilon, debuggingIndex, false);

  if(drawResults)
  {
    shape1.DebugDraw(Vector4(1, 0, 0, 1));
    shape2.DebugDraw(Vector4(0, 1, 0, 1));
    if(result == false)
    {
      gDebugDrawer->DrawLine(LineSegment(closestPoint.mPointA, closestPoint.mPointB));
      gDebugDrawer->DrawPoint(closestPoint.mPointA);
      gDebugDrawer->DrawPoint(closestPoint.mPointB);
    }
  }

  if(file == NULL)
    return;

  if(result == true)
  {
    fprintf(file, "  Result: true");
  }
  else
  {
    float distance = Math::Distance(closestPoint.mPointA, closestPoint.mPointB);
    //fprintf(file, "  Result: false\n  Distance: %s",
    fprintf(file, "  Result: false\n  CsoPoint: %s\n  PointA: %s\n  PointB: %s\n  Distance: %s",
      PrintVector3(closestPoint.mCsoPoint).c_str(),
      PrintVector3(closestPoint.mPointA).c_str(),
      PrintVector3(closestPoint.mPointB).c_str(),
      PrintFloat(distance).c_str());
  }
  fprintf(file, "\n");
}

void PrintSupportShape(SupportShape& shape, const Vector3& dir, FILE* file)
{
  Vector3 result = shape.Support(dir);
  Vector3 center = shape.GetCenter();
  float distance = Math::Dot(dir, result);
  gDebugDrawer->DrawRay(Ray(center, dir.Normalized()), 1.0f);
  gDebugDrawer->DrawPoint(result);
  shape.DebugDraw();

  if(file != nullptr)
  {
    //fprintf(file, "  SearchDir:%s.\n  Result:%s\n  Distance:%s\n",
    fprintf(file, "  SearchDir:%s.\n  Distance:%s\n",
            PrintVector3(dir).c_str(),
            //PrintVector3(result).c_str(),
            PrintFloat(distance).c_str());
    fprintf(file, "\n");
  }
}

void PrintSupportShapeSpherical(SupportShape& shape, float stacks, float slices, FILE* file)
{
  shape.DebugDraw();
  
  if(file != nullptr)
  {
    Vector3 center = shape.GetCenter();
    fprintf(file, "  Center:%s\n\n", PrintVector3(center).c_str());
  }

  for(size_t iStack = 0; iStack < stacks; ++iStack)
  {
    for(size_t iSlice = 0; iSlice < slices; ++iSlice)
    {
      float theta = Math::cTwoPi * (iSlice / (float)slices);
      float phi = Math::cPi * ((iStack + 0.5f) / (float)stacks);

      float x = Math::Sin(theta) * Math::Sin(phi);
      float y = Math::Cos(phi);
      float z = Math::Cos(theta) * Math::Sin(phi);
      Vector3 dir(x, y, z);

      if(file != nullptr)
      {
        Vector3 result = shape.Support(dir);
        float distance = Math::Dot(dir, result);
        //fprintf(file, "  SearchDir:%s.\n  Result:%s\n  Distance:%s\n",
        fprintf(file, "  SearchDir:%s.\n  Distance:%s\n",
                PrintVector3(dir).c_str(),
                //PrintVector3(result).c_str(),
                PrintFloat(distance).c_str());
        fprintf(file, "\n");
      }
    }
  }
}

void ReduceSimplex(std::vector<Vector3>& inPoints, std::vector<Vector3>& outPoints, size_t newSize, int newIndices[4])
{
  for(size_t i = 0; i < newSize; ++i)
    outPoints.push_back(inPoints[newIndices[i]]);
}

bool ContainsIndex(int toFind, size_t size, int indices[4])
{
  for(size_t i = 0; i < size; ++i)
  {
    if(indices[i] == toFind)
      return true;
  }
  return false;
}

void TestVoronoiRegions(const std::string& testName, const Vector3& q, std::vector<Vector3>& points, FILE* file = NULL)
{
  // Print out the test's header
  PrintTestHeader(file, testName);

  size_t newSize = 1;
  int newIndices[4] = {0};
  Vector3 closestPoint, searchDirection = {};
  VoronoiRegion::Type regionType = {};

  if(points.size() == 1)
    regionType = Gjk::IdentifyVoronoiRegion(q, points[0], newSize, newIndices, closestPoint, searchDirection);
  else if(points.size() == 2)
    regionType = Gjk::IdentifyVoronoiRegion(q, points[0], points[1], newSize, newIndices, closestPoint, searchDirection);
  else if(points.size() == 3)
    regionType = Gjk::IdentifyVoronoiRegion(q, points[0], points[1], points[2], newSize, newIndices, closestPoint, searchDirection);
  else if(points.size() == 4)
    regionType = Gjk::IdentifyVoronoiRegion(q, points[0], points[1], points[2], points[3], newSize, newIndices, closestPoint, searchDirection);

  gDebugDrawer->DrawPoint(q).Color(Vector4(1));
  for(int i = 0; i < static_cast<int>(points.size()); ++i)
  {
    Vector4 color = Vector4(1, 0, 0, 1);
    if(ContainsIndex(i, newSize, newIndices))
      color = Vector4(0, 1, 0, 1);
    gDebugDrawer->DrawPoint(points[i]).Color(color);
  }
  gDebugDrawer->DrawPoint(closestPoint).Color(Vector4(0, 0, 1, 1));
  if(searchDirection.Length() > 0.001f)
  {
    Ray searchRay(closestPoint, searchDirection);
    searchRay.DebugDraw(1.0f).Color(Vector4(0, 0, 1, 1));
  }

  if(file != nullptr)
  {
    fprintf(file, "  Voronoi Region: %s\n  Closest Point: %s\n  Search Direction: %s\n",
            VoronoiRegion::Names[regionType], PrintVector3(closestPoint).c_str(), PrintVector3(searchDirection).c_str());
    fprintf(file, "  Reduced to %d indices: ", static_cast<int>(newSize));
    for(size_t i = 0; i < newSize; ++i)
      fprintf(file, "%d ", newIndices[i]);
  }
}

bool closeEqualFloats(float one, float two)
{
  return fabs(fabs(one) - fabs(two)) < 0.01f;
}


void TestVoronoiRegionsStressTest(const std::string& testName, std::vector<Vector3>& points, const Vector3& stepSize, FILE* file = NULL)
{
  // Print out the test's header
  PrintTestHeader(file, testName);

  size_t newSize = 1;
  int newIndices[4] = {0};
  Vector3 closestPoint, searchDirection = {};
  VoronoiRegion::Type regionType = {};

  // Compute the aabb of the input points
  Aabb aabb;
  for(size_t i = 0; i < points.size(); ++i)
    aabb.Expand(points[i]);
  // Expand the aabb a bit to help test extra voronoi regions
  // Expand the aabb a bit to help test extra voronoi regions
  Vector3 center = aabb.GetCenter();
  Vector3 halfSize = aabb.GetHalfSize();
  halfSize *= 1.25f;
  aabb.mMin = center - halfSize;
  aabb.mMax = center + halfSize;

  for(float z = aabb.GetMin().z; z <= aabb.GetMax().z; z += stepSize.z)
  {
    for(float y = aabb.GetMin().y; y <= aabb.GetMax().y; y += stepSize.y)
    {
      for(float x = aabb.GetMin().x; x <= aabb.GetMax().x; x += stepSize.x)
      {
        Vector3 q(x, y, z);
        if(points.size() == 1)
          regionType = Gjk::IdentifyVoronoiRegion(q, points[0], newSize, newIndices, closestPoint, searchDirection);
        else if(points.size() == 2)
          regionType = Gjk::IdentifyVoronoiRegion(q, points[0], points[1], newSize, newIndices, closestPoint, searchDirection);
        else if(points.size() == 3)
          regionType = Gjk::IdentifyVoronoiRegion(q, points[0], points[1], points[2], newSize, newIndices, closestPoint, searchDirection);
        else if(points.size() == 4)
          regionType = Gjk::IdentifyVoronoiRegion(q, points[0], points[1], points[2], points[3], newSize, newIndices, closestPoint, searchDirection);

        searchDirection.AttemptNormalize();
        if(file != nullptr)
        {
          fprintf(file, "  Voronoi Region: %s\n  Closest Point: %s\n  Search Direction: %s\n",
            VoronoiRegion::Names[regionType], PrintVector3(closestPoint).c_str(), PrintVector3(searchDirection).c_str());
          fprintf(file, "  Reduced to %d indices: ", static_cast<int>(newSize));
          for(size_t i = 0; i < newSize; ++i)
            fprintf(file, "%d ", newIndices[i]);
        }
      }
    }
  }

  for(size_t i = 0; i < points.size(); ++i)
  {
    Vector4 color = Vector4(1, 0, 0, 1);
    gDebugDrawer->DrawPoint(points[i]).Color(color);
  }
  aabb.DebugDraw();
}

void PointVoronoiRegions1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(1, -1, 0), points, file);
}

void PointVoronoiRegions2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, -1, 0), points, file);
}

void PointVoronoiRegions3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, -3, 0), points, file);
}

void EdgeVoronoiRegions1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, -3, 0), points, file);
}

void EdgeVoronoiRegions2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, -1, 0), points, file);
}

void EdgeVoronoiRegions3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, -0.5f, 0), points, file);
}

void EdgeVoronoiRegions4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, 0, 0), points, file);
}

void EdgeVoronoiRegions5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.5f, 0), points, file);
}

void EdgeVoronoiRegions6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, 1, 0), points, file);
}

void EdgeVoronoiRegions7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, 3, 0), points, file);
}

void EdgeVoronoiRegions8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, 0, -1), points, file);
}

void EdgeVoronoiRegions9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(0, 0, 1), points, file);
}

void EdgeVoronoiRegions10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(-1, 0, 0), points, file);
}

void EdgeVoronoiRegions11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(0, 1, 0));
  TestVoronoiRegions(testName, Vector3(1, 0, 0), points, file);
}

void TriangleVoronoiRegions1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 1.52573f, 1.85065f), points, file);
}

void TriangleVoronoiRegions2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 1.99995f, 1.01017f), points, file);
}

void TriangleVoronoiRegions3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.561908f, 1.89893f), points, file);
}

void TriangleVoronoiRegions4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 1.52573f, -1.85065f), points, file);
}

void TriangleVoronoiRegions5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 1.99995f, -1.01017f), points, file);
}

void TriangleVoronoiRegions6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.561908f, -1.89893f), points, file);
}

void TriangleVoronoiRegions7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, -2, 0), points, file);
}

void TriangleVoronoiRegions8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, -1.45709f, 0.889421f), points, file);
}

void TriangleVoronoiRegions9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, -1.45709f, -0.889421f), points, file);
}

void TriangleVoronoiRegions10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 2, 0), points, file);
}

void TriangleVoronoiRegions11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, -0.447214f, 1.39443f), points, file);
}

void TriangleVoronoiRegions12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, -0.447214f, -1.39443f), points, file);
}

void TriangleVoronoiRegions13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  points.push_back(Vector3(0, -1, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.333333f, 0), points, file);
}

void TriangleVoronoiRegions14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 2.22975f, 1.97325f), points, file);
}

void TriangleVoronoiRegions15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 2.99991f, 1.01339f), points, file);
}

void TriangleVoronoiRegions16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 1.11164f, 1.45915f), points, file);
}

void TriangleVoronoiRegions17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 2.22975f, -1.97325f), points, file);
}

void TriangleVoronoiRegions18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 2.99991f, -1.01339f), points, file);
}

void TriangleVoronoiRegions19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 1.11164f, -1.45915f), points, file);
}

void TriangleVoronoiRegions20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.5f, 0), points, file);
}

void TriangleVoronoiRegions21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.603509f, 0.443062f), points, file);
}

void TriangleVoronoiRegions22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.603509f, -0.443062f), points, file);
}

void TriangleVoronoiRegions23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 3, 0), points, file);
}

void TriangleVoronoiRegions24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.855573f, 0.947214f), points, file);
}

void TriangleVoronoiRegions25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 0.855573f, -0.947214f), points, file);
}

void TriangleVoronoiRegions26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 2, 1));
  points.push_back(Vector3(0, 2, -1));
  points.push_back(Vector3(0, 1.5f, 0));
  TestVoronoiRegions(testName, Vector3(0, 1.83333f, 0), points, file);
}

void TetrahedonVoronoiRegions1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, 1.9997f, 0.0243867f), points, file);
}

void TetrahedonVoronoiRegions2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, 1.45687f, 0.889533f), points, file);
}

void TetrahedonVoronoiRegions3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-0.870269f, 1.23148f, -0.434795f), points, file);
}

void TetrahedonVoronoiRegions4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0.870269f, 1.23148f, -0.434795f), points, file);
}

void TetrahedonVoronoiRegions5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.83853f, -1.3214f, 1.43997f), points, file);
}

void TetrahedonVoronoiRegions6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.01142f, -0.558722f, 1.8973f), points, file);
}

void TetrahedonVoronoiRegions7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.87807f, -0.786989f, 0.571493f), points, file);
}

void TetrahedonVoronoiRegions8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.01101f, -1.99992f, 1.00578f), points, file);
}

void TetrahedonVoronoiRegions9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.83853f, -1.3214f, 1.43997f), points, file);
}

void TetrahedonVoronoiRegions10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.01142f, -0.558722f, 1.8973f), points, file);
}

void TetrahedonVoronoiRegions11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.87807f, -0.786989f, 0.571493f), points, file);
}

void TetrahedonVoronoiRegions12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.01101f, -1.99992f, 1.00578f), points, file);
}

void TetrahedonVoronoiRegions13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, -1.54241f, -1.84011f), points, file);
}

void TetrahedonVoronoiRegions14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-0.869843f, -0.78993f, -1.44637f), points, file);
}

void TetrahedonVoronoiRegions15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0.869843f, -0.78993f, -1.44637f), points, file);
}

void TetrahedonVoronoiRegions16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, -1.99995f, -1.00997f), points, file);
}

void TetrahedonVoronoiRegions17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.23393f, 0.55951f, 0.88509f), points, file);
}

void TetrahedonVoronoiRegions18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-0.508526f, 0.450604f, 1.39268f), points, file);
}

void TetrahedonVoronoiRegions19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.37533f, 0.223201f, 0.0710715f), points, file);
}

void TetrahedonVoronoiRegions20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.23393f, 0.55951f, 0.88509f), points, file);
}

void TetrahedonVoronoiRegions21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0.508526f, 0.450604f, 1.39268f), points, file);
}

void TetrahedonVoronoiRegions22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.37533f, 0.223201f, 0.0710715f), points, file);
}

void TetrahedonVoronoiRegions23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(2.98023e-08f, 0.447214f, -1.39443f), points, file);
}

void TetrahedonVoronoiRegions24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-0.867645f, 0.222347f, -0.944695f), points, file);
}

void TetrahedonVoronoiRegions25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0.867645f, 0.222347f, -0.944695f), points, file);
}

void TetrahedonVoronoiRegions26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, -1.52573f, 1.85065f), points, file);
}

void TetrahedonVoronoiRegions27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, -0.561908f, 1.89893f), points, file);
}

void TetrahedonVoronoiRegions28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, -1.99995f, 1.01017f), points, file);
}

void TetrahedonVoronoiRegions29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.19806f, -1.62521f, -0.34903f), points, file);
}

void TetrahedonVoronoiRegions30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.37458f, -0.790529f, -0.437292f), points, file);
}

void TetrahedonVoronoiRegions31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-0.508009f, -1.99996f, -0.00400426f), points, file);
}

void TetrahedonVoronoiRegions32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.19806f, -1.62521f, -0.34903f), points, file);
}

void TetrahedonVoronoiRegions33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.37458f, -0.790529f, -0.437292f), points, file);
}

void TetrahedonVoronoiRegions34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0.508009f, -1.99996f, -0.00400426f), points, file);
}

void TetrahedonVoronoiRegions35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, 0.11388f, 1.56109f), points, file);
}

void TetrahedonVoronoiRegions36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(-1.2062f, -0.115115f, -0.436436f), points, file);
}

void TetrahedonVoronoiRegions37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(1.2062f, -0.115115f, -0.436436f), points, file);
}

void TetrahedonVoronoiRegions38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, -2, 0.333333f), points, file);
}

void TetrahedonVoronoiRegions39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 0));
  points.push_back(Vector3(-1, -1, 1));
  points.push_back(Vector3(1, -1, 1));
  points.push_back(Vector3(0, -1, -1));
  TestVoronoiRegions(testName, Vector3(0, -0.5f, 0.25f), points, file);
}

void TetrahedonVoronoiRegions40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 2.19995f, 0.00958677f), points, file);
}

void TetrahedonVoronoiRegions41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 2.18095f, 0.194274f), points, file);
}

void TetrahedonVoronoiRegions42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.885908f, 1.65535f, -0.088472f), points, file);
}

void TetrahedonVoronoiRegions43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.885908f, 1.65535f, -0.088472f), points, file);
}

void TetrahedonVoronoiRegions44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-1.0969f, 1.4289f, 1.10775f), points, file);
}

void TetrahedonVoronoiRegions45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.21113f, 1.98047f, 1.19637f), points, file);
}

void TetrahedonVoronoiRegions46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-1.0911f, 1.44536f, 0.912877f), points, file);
}

void TetrahedonVoronoiRegions47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.219996f, 0.000202835f, 1.0024f), points, file);
}

void TetrahedonVoronoiRegions48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(1.0969f, 1.4289f, 1.10775f), points, file);
}

void TetrahedonVoronoiRegions49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.21113f, 1.98047f, 1.19637f), points, file);
}

void TetrahedonVoronoiRegions50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(1.0911f, 1.44536f, 0.912877f), points, file);
}

void TetrahedonVoronoiRegions51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.219996f, 0.000202835f, 1.0024f), points, file);
}

void TetrahedonVoronoiRegions52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 0.477696f, -1.85276f), points, file);
}

void TetrahedonVoronoiRegions53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.89318f, 1.43745f, -1.10424f), points, file);
}

void TetrahedonVoronoiRegions54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.89318f, 1.43745f, -1.10424f), points, file);
}

void TetrahedonVoronoiRegions55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 5.21541e-05f, -1.01021f), points, file);
}

void TetrahedonVoronoiRegions56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.628762f, 1.94639f, 0.563525f), points, file);
}

void TetrahedonVoronoiRegions57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.105584f, 2.08078f, 0.695039f), points, file);
}

void TetrahedonVoronoiRegions58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.988515f, 1.5504f, 0.412378f), points, file);
}

void TetrahedonVoronoiRegions59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.628762f, 1.94639f, 0.563525f), points, file);
}

void TetrahedonVoronoiRegions60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.105584f, 2.08078f, 0.695039f), points, file);
}

void TetrahedonVoronoiRegions61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.988515f, 1.5504f, 0.412378f), points, file);
}

void TetrahedonVoronoiRegions62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 2.08058f, -0.696116f), points, file);
}

void TetrahedonVoronoiRegions63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.885824f, 1.55501f, -0.591002f), points, file);
}

void TetrahedonVoronoiRegions64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.885824f, 1.55501f, -0.591002f), points, file);
}

void TetrahedonVoronoiRegions65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 0.901462f, 1.99513f), points, file);
}

void TetrahedonVoronoiRegions66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 1.97759f, 1.21053f), points, file);
}

void TetrahedonVoronoiRegions67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 0.000108361f, 1.01472f), points, file);
}

void TetrahedonVoronoiRegions68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.945909f, 0.473424f, -0.0845909f), points, file);
}

void TetrahedonVoronoiRegions69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.995329f, 1.43631f, -0.0895329f), points, file);
}

void TetrahedonVoronoiRegions70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.110112f, 5.16176e-05f, -0.00101117f), points, file);
}

void TetrahedonVoronoiRegions71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.945909f, 0.473424f, -0.0845909f), points, file);
}

void TetrahedonVoronoiRegions72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.995329f, 1.43631f, -0.0895329f), points, file);
}

void TetrahedonVoronoiRegions73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.110112f, 5.16176e-05f, -0.00101117f), points, file);
}

void TetrahedonVoronoiRegions74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 2.04725f, 0.862783f), points, file);
}

void TetrahedonVoronoiRegions75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(-0.957538f, 1.5121f, -0.0890871f), points, file);
}

void TetrahedonVoronoiRegions76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0.957538f, 1.5121f, -0.0890871f), points, file);
}

void TetrahedonVoronoiRegions77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 0, 0.333333f), points, file);
}

void TetrahedonVoronoiRegions78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.2f, 0));
  points.push_back(Vector3(-0.2f, 1, 1));
  points.push_back(Vector3(0.2f, 1, 1));
  points.push_back(Vector3(0, 1, -1));
  TestVoronoiRegions(testName, Vector3(0, 1.05f, 0.25f), points, file);
}

void VoronoiRegionsStress1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1, 2));
  points.push_back(Vector3(-1, -1, -1));
  TestVoronoiRegionsStressTest(testName, points, Vector3(0.2f, 0.2f, 0.2f), file);
}

void VoronoiRegionsStress2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, -1, 0));
  points.push_back(Vector3(-0.5f, 0.5f, -1));
  points.push_back(Vector3(0.6f, 1, 1));
  TestVoronoiRegionsStressTest(testName, points, Vector3(0.2f, 0.2f, 0.2f), file);
}

void VoronoiRegionsStress3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  std::vector<Vector3> points;
  points.push_back(Vector3(0, 1.5f, -0.5f));
  points.push_back(Vector3(-0.6f, 0.9f, 2.5f));
  points.push_back(Vector3(0.6f, 0.9f, 2.5f));
  points.push_back(Vector3(0, 0.9f, -3.5f));
  TestVoronoiRegionsStressTest(testName, points, Vector3(0.2f, 0.2f, 0.2f), file);
}

void SphereSupport1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape; 
  shape.mSphere.mCenter = Vector3(0.577812f, 0.776177f, -0.602959f); 
  shape.mSphere.mRadius = 0.464431f;
  Vector3 dir = Vector3(0.073282f, -2.20507f, -0.643366f); 
  PrintSupportShape(shape, dir, file); 
}

void SphereSupport2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape; 
  shape.mSphere.mCenter = Vector3(-0.403611f, -1.08467f, 0.148383f); 
  shape.mSphere.mRadius = 4.06283f;
  Vector3 dir = Vector3(1.73315f, -2.37631f, -0.118491f); 
  PrintSupportShape(shape, dir, file); 
}

void SphereSupport3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape; 
  shape.mSphere.mCenter = Vector3(0.0539397f, 0.744349f, 0.940242f); 
  shape.mSphere.mRadius = 2.76108f;
  Vector3 dir = Vector3(3.12043f, -1.11756f, 0.67438f); 
  PrintSupportShape(shape, dir, file); 
}

void SphereSupport4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape; 
  shape.mSphere.mCenter = Vector3(0.212669f, -0.991166f, -0.686591f); 
  shape.mSphere.mRadius = 1.45947f;
  Vector3 dir = Vector3(1.85766f, 0.479936f, 0.937636f); 
  PrintSupportShape(shape, dir, file); 
}

void SphereSupport5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape; 
  shape.mSphere.mCenter = Vector3(-0.590297f, 1.09233f, 0.11888f); 
  shape.mSphere.mRadius = 0.157723f;
  Vector3 dir = Vector3(1.38186f, 1.57224f, 1.93472f); 
  PrintSupportShape(shape, dir, file); 
}

void ObbSupport1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape; 
  shape.mTranslation = Vector3(2.04885f, -0.182182f, 0.539079f); 
  shape.mRotation = Math::ToMatrix3(Vector3(-0.847855f, -0.516529f, -0.119746f), 2.58234f);
  shape.mScale = Vector3(2.22778f, 1.44271f, 2.01333f); 
  Vector3 dir = Vector3(-1.37468f, -0.305937f, 1.44087f); 
  PrintSupportShape(shape, dir, file); 
}

void ObbSupport2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape; 
  shape.mTranslation = Vector3(8.59562f, 0.732762f, 4.00405f); 
  shape.mRotation = Math::ToMatrix3(Vector3(0.967342f, -0.22903f, -0.108603f), 2.58217f);
  shape.mScale = Vector3(0.70255f, 0.454588f, 3.34065f); 
  Vector3 dir = Vector3(-1.31201f, -2.53968f, 1.6207f); 
  PrintSupportShape(shape, dir, file); 
}

void ObbSupport3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape; 
  shape.mTranslation = Vector3(-9.20305f, -1.07426f, -2.16728f); 
  shape.mRotation = Math::ToMatrix3(Vector3(0.935599f, 0.233284f, -0.265015f), 1.18551f);
  shape.mScale = Vector3(4.17754f, 2.65673f, 0.438511f); 
  Vector3 dir = Vector3(-1.17353f, -2.12172f, 0.288007f); 
  PrintSupportShape(shape, dir, file); 
}

void ObbSupport4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape; 
  shape.mTranslation = Vector3(4.53315f, 0.672894f, -8.34495f); 
  shape.mRotation = Math::ToMatrix3(Vector3(-0.761855f, 0.585982f, 0.276048f), 1.18183f);
  shape.mScale = Vector3(0.310862f, 2.26194f, 3.74083f); 
  Vector3 dir = Vector3(-1.62005f, -2.74229f, -0.848079f); 
  PrintSupportShape(shape, dir, file); 
}

void ObbSupport5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape; 
  shape.mTranslation = Vector3(-8.25043f, -1.4871f, 4.52245f); 
  shape.mRotation = Math::ToMatrix3(Vector3(0.0560913f, 0.64874f, 0.75894f), 1.89617f);
  shape.mScale = Vector3(3.32782f, 1.00499f, 0.726939f); 
  Vector3 dir = Vector3(-1.01532f, -1.61236f, -1.55618f); 
  PrintSupportShape(shape, dir, file); 
}

void GjkTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape1; 
  shape1.mSphere = Sphere(Vector3(-3, 0, 0), 1); 
  SphereSupportShape shape2; 
  shape2.mSphere = Sphere(Vector3(3, 0, 0), 1); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape1; 
  shape1.mSphere = Sphere(Vector3(-3, 0, 0), 4); 
  SphereSupportShape shape2; 
  shape2.mSphere = Sphere(Vector3(3, 0, 0), 4); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SphereSupportShape shape1; 
  shape1.mSphere = Sphere(Vector3(-3, 0, 0), 1.0f); 
  SphereSupportShape shape2; 
  shape2 = shape1; 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(-2, -2, -2); 
  shape1.mRotation.SetIdentity(); 
  shape1.mScale = Vector3(1); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(2, 2, 2); 
  shape2.mRotation.SetIdentity(); 
  shape2.mScale = Vector3(1); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(2.5, 2.5, 0); 
  shape1.mRotation = Math::ToMatrix3(Vector3(0, 1, 0), Math::DegToRad(45)); 
  shape1.mScale = Vector3(1); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(0, 0, 4.5); 
  shape2.mRotation = Math::ToMatrix3(Vector3(0, 1, 1).Normalized(), Math::DegToRad(45)); 
  shape2.mScale = Vector3(2); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(-1, 0.5, 0); 
  shape1.mRotation.SetIdentity(); 
  shape1.mScale = Vector3(1); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(0, 0, 0); 
  shape2.mRotation.SetIdentity(); 
  shape2.mScale = Vector3(1); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(-2, 0.5, 0); 
  shape1.mRotation.SetIdentity(); 
  shape1.mScale = Vector3(1); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(0, 0, 0); 
  shape2.mRotation.SetIdentity(); 
  shape2.mScale = Vector3(1); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(0, -.5f, 0); 
  shape1.mRotation.SetIdentity(); 
  shape1.mScale = Vector3(2); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(2.58f, 0.5f, 0); 
  Vector3 eulerAngles; 
  eulerAngles.x = Math::DegToRad(79.17f); 
  eulerAngles.y = Math::DegToRad(52.88f); 
  eulerAngles.z = Math::DegToRad(65.66f); 
  shape2.mRotation = Math::ToMatrix3(Math::EulerAngles(eulerAngles, Math::EulerOrders::XYZs)); 
  shape2.mScale = Vector3(2); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PointsSupportShape shape1; 
  shape1.mLocalSpacePoints.push_back(Vector3(0.0f, -0.525731f, 0.850651f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(0.850651f, 0.0f, 0.525731f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(0.850651f, 0.0f, -0.525731f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(-0.850651f, 0.0f, -0.525731f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(-0.850651f, 0.0f, 0.525731f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(-0.525731f, 0.850651f, 0.0f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(0.525731f, 0.850651f, 0.0f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(0.525731f, -0.850651f, 0.0f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(-0.525731f, -0.850651f, 0.0f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(0.0f, -0.525731f, -0.850651f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(0.0f, 0.525731f, -0.850651f)); 
  shape1.mLocalSpacePoints.push_back(Vector3(0.0f, 0.525731f, 0.850651f)); 
  shape1.mTranslation = Vector3(2.5, 2.5, 0); 
  shape1.mRotation = Math::ToMatrix3(Vector3(0, 1, 0), Math::DegToRad(45)); 
  shape1.mScale = Vector3(1); 
  PointsSupportShape shape2; 
  shape2 = shape1; 
  shape2.mTranslation = Vector3(0, 0, 4.5); 
  shape2.mRotation = Math::ToMatrix3(Vector3(0, 1, 1).Normalized(), Math::DegToRad(45)); 
  shape2.mScale = Vector3(2); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkStressTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(0.429898f, 0.439505f, -3.27661f); 
  shape1.mRotation = Math::ToMatrix3(Vector3(-0.84637f, -0.515624f, 0.133374f), 3.87342f);
  shape1.mScale = Vector3(1.14311f, 3.36085f, 1.80895f); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(2.88779f, 3.49773f, -0.474187f); 
  shape2.mRotation = Math::ToMatrix3(Vector3(-0.602763f, 0.67065f, -0.432326f), 1.53475f);
  shape2.mScale = Vector3(1.18476f, 4.54169f, 1.59504f); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkStressTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(0.962417f, 3.79801f, 4.04033f); 
  shape1.mRotation = Math::ToMatrix3(Vector3(0.954417f, -0.22597f, -0.195002f), 2.75392f);
  shape1.mScale = Vector3(3.71282f, 0.350602f, 1.58913f); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(-2.63924f, -3.50654f, -1.23607f); 
  shape2.mRotation = Math::ToMatrix3(Vector3(-0.264508f, 0.699595f, 0.663778f), 2.20307f);
  shape2.mScale = Vector3(0.574085f, 0.203867f, 2.79437f); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkStressTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(2.92009f, -2.5381f, 4.0207f); 
  shape1.mRotation = Math::ToMatrix3(Vector3(0.451166f, 0.112495f, 0.885321f), 2.82445f);
  shape1.mScale = Vector3(0.392874f, 0.111774f, 1.4497f); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(-1.46413f, -4.83395f, 8.27315f); 
  shape2.mRotation = Math::ToMatrix3(Vector3(-0.10314f, 0.577811f, -0.809628f), 2.60387f);
  shape2.mScale = Vector3(3.95746f, 2.23594f, 1.56858f); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkStressTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(-3.53564f, -1.44474f, 4.00018f); 
  shape1.mRotation = Math::ToMatrix3(Vector3(-0.60692f, 0.466814f, -0.643221f), 4.31062f);
  shape1.mScale = Vector3(3.46565f, 1.71712f, 1.293f); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(0.512985f, -8.99022f, -0.197089f); 
  shape2.mRotation = Math::ToMatrix3(Vector3(0.155865f, 0.841624f, 0.517084f), 3.5387f);
  shape2.mScale = Vector3(3.05802f, 2.51123f, 1.57852f); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

void GjkStressTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  ObbSupportShape shape1; 
  shape1.mTranslation = Vector3(1.54144f, -2.57808f, -8.53066f); 
  shape1.mRotation = Math::ToMatrix3(Vector3(0.0560913f, 0.64874f, 0.75894f), 1.89617f);
  shape1.mScale = Vector3(4.02413f, 1.34486f, 0.397839f); 
  ObbSupportShape shape2; 
  shape2.mTranslation = Vector3(-6.23353f, -1.33718f, -2.46555f); 
  shape2.mRotation = Math::ToMatrix3(Vector3(-0.461726f, -0.88229f, -0.0915004f), 2.94653f);
  shape2.mScale = Vector3(2.87957f, 3.80531f, 0.0595694f); 
  PrintGjkResults(shape1, shape2, 100, debuggingIndex, file, true); 
}

PointsSupportShape* CreatePointSupportShape1()
{
  PointsSupportShape* supportShape = new PointsSupportShape();
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.69176f, 1.01155f, 3.05068f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.67761f, 2.45687f, 1.83826f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-5.27128f, 0.447096f, 0.340967f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.01912f, -1.64241f, 1.37828f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.78397f, -1.66919f, -0.464606f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.34766f, -1.59997f, 0.990126f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.10484f, 0.312883f, -1.84281f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.30185f, -1.40366f, -0.131033f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-4.69743f, 0.538645f, 0.778684f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.25864f, 0.559503f, -1.6025f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.2037f, -1.0022f, 1.30673f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.85325f, 0.315807f, -0.936459f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.59093f, 3.18664f, -0.983547f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-4.33416f, 2.39615f, 1.089f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.747492f, 1.45774f, 1.67154f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.34491f, -0.566644f, 0.585025f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.31289f, -0.626764f, -0.204766f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.90569f, -0.161039f, -0.756992f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.06541f, -0.0615771f, 0.835426f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.79906f, -0.11347f, 3.03875f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-4.14893f, 2.66114f, -0.425925f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.90258f, 1.49455f, 2.92583f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-5.04743f, 2.011f, 0.365586f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.9818f, 0.123897f, -2.43865f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.3303f, 2.47804f, 0.319676f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.84489f, 1.73403f, 2.46284f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.674366f, -0.143556f, -0.166007f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.1613f, -1.43788f, 0.131864f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-4.27698f, -0.496694f, -0.403058f));
  supportShape->mScale = Vector3(3.00893f, 3.00893f, 3.00893f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(-0.312281f, 0.128694f, -0.586144f, -0.736447f));
  supportShape->mTranslation = Vector3(2.79197f, 0.987255f, -8.35168f);
  return supportShape;
}

PointsSupportShape* CreatePointSupportShape2()
{
  PointsSupportShape* supportShape = new PointsSupportShape();
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.06271f, 2.24287f, -0.555226f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.24143f, 3.04784f, 0.613466f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.152477f, 3.06494f, -0.030328f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.531133f, 3.14665f, -0.0427798f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.0872791f, 1.70463f, 1.87847f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.440391f, 2.82757f, 0.0358644f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.530273f, 2.13547f, 1.43685f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.56928f, 1.87084f, 0.994171f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.507671f, 0.913748f, 2.19955f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.377539f, 1.45963f, 0.923917f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.879934f, 1.65117f, -0.631197f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.00938f, 3.36199f, 0.816577f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.916056f, 2.01954f, -0.451384f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.746659f, 2.58356f, 2.20995f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.343379f, 1.43809f, 0.946731f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.7657f, 1.72168f, 0.769616f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.682166f, 1.37111f, -0.34138f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.21211f, 0.342066f, 0.840142f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.315835f, 2.04917f, -0.881676f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.0614991f, 0.744519f, 0.0469987f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.470853f, 1.74386f, 1.39881f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.961454f, 1.54481f, 1.22646f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.643269f, 2.69273f, 0.295583f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.1215f, 2.91621f, 0.310534f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.63579f, 1.63814f, -0.376817f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.0491652f, 2.18855f, -0.514605f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.921101f, 3.14133f, 0.233968f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.26488f, 1.12031f, 1.59426f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.135038f, 1.74538f, 2.21944f));
  supportShape->mScale = Vector3(2.92592f, 2.92592f, 2.92592f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(0.895619f, 0.0894486f, 0.104965f, -0.422905f));
  supportShape->mTranslation = Vector3(0.189055f, 0.487791f, 8.4922f);
  return supportShape;
}

PointsSupportShape* CreatePointSupportShape3()
{
  PointsSupportShape* supportShape = new PointsSupportShape();
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.784714f, 3.37132f, 0.159034f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.567703f, 2.7717f, 0.0876543f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.12184f, 2.18f, -0.0384215f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.674532f, 1.99914f, 1.23398f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.661099f, 2.524f, -0.609477f));
  supportShape->mLocalSpacePoints.push_back(Vector3(0.297561f, 2.58333f, 0.117748f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.0158456f, 2.7163f, 1.07297f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.02643f, 2.55547f, 0.449282f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.04793f, 2.72636f, 0.835025f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.144331f, 1.8084f, 0.0274996f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.867693f, 1.91254f, 0.77667f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.20144f, 2.98264f, 0.137955f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.445658f, 3.35821f, 0.244088f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.776026f, 1.9451f, 0.276717f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.76474f, 2.33138f, 0.639772f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.471201f, 2.17351f, -0.256959f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.204379f, 2.23979f, 0.276009f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.27937f, 2.16896f, 0.815423f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.247414f, 2.30661f, 0.303592f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.524691f, 1.85081f, 1.01904f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.12008f, 2.81694f, 0.186862f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.0669f, 2.71243f, -0.396196f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.531972f, 2.60397f, -0.124666f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.15464f, 1.66788f, 0.479037f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-0.905316f, 2.11079f, 0.491934f));
  supportShape->mScale = Vector3(3.62653f, 3.62653f, 3.62653f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(-0.194263f, 0.280126f, -0.551496f, -0.761343f));
  supportShape->mTranslation = Vector3(5.33344f, 0.792733f, -6.36089f);
  return supportShape;
}

PointsSupportShape* CreatePointSupportShape4()
{
  PointsSupportShape* supportShape = new PointsSupportShape();
  supportShape->mLocalSpacePoints.push_back(Vector3(2.06008f, -0.342237f, 0.550275f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.69963f, -0.511025f, 0.541426f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.4846f, -0.0506535f, 1.01459f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.29921f, -0.913248f, 0.530648f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.66591f, -0.113062f, 0.739963f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.07283f, -0.363404f, 0.872166f));
  supportShape->mLocalSpacePoints.push_back(Vector3(1.84383f, -0.752241f, 0.622644f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.58343f, -0.273882f, 0.627958f));
  supportShape->mLocalSpacePoints.push_back(Vector3(1.88728f, -0.192986f, 0.60977f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.73196f, -0.303976f, 0.505714f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.78923f, -0.574103f, 0.792596f));
  supportShape->mLocalSpacePoints.push_back(Vector3(1.92295f, -0.507772f, 0.654814f));
  supportShape->mLocalSpacePoints.push_back(Vector3(1.82792f, -0.297524f, 0.786321f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.1453f, -0.576954f, 1.07835f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.45322f, -0.718743f, 0.58391f));
  supportShape->mLocalSpacePoints.push_back(Vector3(2.16655f, -0.783691f, 0.493809f));
  supportShape->mScale = Vector3(2.83267f, 2.83267f, 2.83267f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(0.184077f, -0.35075f, -0.608354f, 0.687747f));
  supportShape->mTranslation = Vector3(-0.44f, 5.35234f, 3.76774f);
  return supportShape;
}

PointsSupportShape* CreatePointSupportShape5()
{
  PointsSupportShape* supportShape = new PointsSupportShape();
  supportShape->mLocalSpacePoints.push_back(Vector3(-1.83899f, 0.495612f, 1.113f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.3234f, -0.0309604f, 1.1863f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.70874f, 0.436549f, 1.24821f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.7538f, -0.0184278f, 1.52856f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.51568f, 0.520403f, 0.686613f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.1366f, 0.220042f, 1.42792f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.28151f, 1.11948f, 1.22219f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.88137f, -0.0317611f, 1.55819f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.77409f, 0.0520612f, 0.546954f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-3.14563f, 0.776229f, 1.3242f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.74547f, 0.551608f, 0.703871f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.39859f, 0.309431f, 1.03734f));
  supportShape->mLocalSpacePoints.push_back(Vector3(-2.07316f, 0.382208f, 1.45101f));
  supportShape->mScale = Vector3(2.33395f, 2.33395f, 2.33395f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(0.215937f, -0.672309f, -0.128185f, -0.696377f));
  supportShape->mTranslation = Vector3(-3.14754f, -6.08129f, -2.81667f);
  return supportShape;
}

SphereSupportShape* CreateSphereSupportShape1()
{
  SphereSupportShape* supportShape = new SphereSupportShape(); 
  supportShape->mSphere.mCenter = Vector3(0.743798f, -0.977842f, -7.99672f); 
  supportShape->mSphere.mRadius = 1.39981f;
  return supportShape;
}

SphereSupportShape* CreateSphereSupportShape2()
{
  SphereSupportShape* supportShape = new SphereSupportShape(); 
  supportShape->mSphere.mCenter = Vector3(0.542529f, 4.65361f, -3.74979f); 
  supportShape->mSphere.mRadius = 2.22318f;
  return supportShape;
}

SphereSupportShape* CreateSphereSupportShape3()
{
  SphereSupportShape* supportShape = new SphereSupportShape(); 
  supportShape->mSphere.mCenter = Vector3(-4.7615f, -0.445945f, -6.87806f); 
  supportShape->mSphere.mRadius = 2.40053f;
  return supportShape;
}

SphereSupportShape* CreateSphereSupportShape4()
{
  SphereSupportShape* supportShape = new SphereSupportShape(); 
  supportShape->mSphere.mCenter = Vector3(-6.24734f, 0.754756f, -1.11342f); 
  supportShape->mSphere.mRadius = 4.49994f;
  return supportShape;
}

SphereSupportShape* CreateSphereSupportShape5()
{
  SphereSupportShape* supportShape = new SphereSupportShape(); 
  supportShape->mSphere.mCenter = Vector3(-4.20304f, -5.51349f, 4.73148f); 
  supportShape->mSphere.mRadius = 2.55217f;
  return supportShape;
}

ObbSupportShape* CreateObbSupportShape1()
{
  ObbSupportShape* supportShape = new ObbSupportShape(); 
  supportShape->mScale = Vector3(3.995f, 0.836575f, 1.982f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(-0.339645f, -0.658242f, -0.671241f, -0.0281756f));
  supportShape->mTranslation = Vector3(-0.141365f, 7.54103f, -1.84479f);
  return supportShape;
}

ObbSupportShape* CreateObbSupportShape2()
{
  ObbSupportShape* supportShape = new ObbSupportShape(); 
  supportShape->mScale = Vector3(0.316952f, 3.01208f, 3.28694f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(-0.933367f, -0.219404f, -0.173307f, -0.225059f));
  supportShape->mTranslation = Vector3(-4.25059f, 4.92979f, -2.70318f);
  return supportShape;
}

ObbSupportShape* CreateObbSupportShape3()
{
  ObbSupportShape* supportShape = new ObbSupportShape(); 
  supportShape->mScale = Vector3(2.41758f, 3.6861f, 1.87805f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(-0.446621f, 0.17869f, 0.313052f, -0.818901f));
  supportShape->mTranslation = Vector3(-1.03254f, 5.47122f, 3.30379f);
  return supportShape;
}

ObbSupportShape* CreateObbSupportShape4()
{
  ObbSupportShape* supportShape = new ObbSupportShape(); 
  supportShape->mScale = Vector3(2.5244f, 2.24871f, 2.88051f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(-0.134213f, -0.792952f, 0.453415f, 0.384226f));
  supportShape->mTranslation = Vector3(-6.70118f, -1.47787f, 6.14601f);
  return supportShape;
}

ObbSupportShape* CreateObbSupportShape5()
{
  ObbSupportShape* supportShape = new ObbSupportShape(); 
  supportShape->mScale = Vector3(1.6407f, 3.46498f, 0.0445807f);
  supportShape->mRotation = Math::ToMatrix3(Quaternion(0.207615f, 0.0149273f, 0.870168f, 0.446632f));
  supportShape->mTranslation = Vector3(-8.80512f, -2.68954f, 2.08915f);
  return supportShape;
}

void RegisterFuzzTestSupportShapes()
{
  mSupportShapesContainer.Add(CreatePointSupportShape1());
  mSupportShapesContainer.Add(CreatePointSupportShape2());
  mSupportShapesContainer.Add(CreatePointSupportShape3());
  mSupportShapesContainer.Add(CreatePointSupportShape4());
  mSupportShapesContainer.Add(CreatePointSupportShape5());
  mSupportShapesContainer.Add(CreateSphereSupportShape1());
  mSupportShapesContainer.Add(CreateSphereSupportShape2());
  mSupportShapesContainer.Add(CreateSphereSupportShape3());
  mSupportShapesContainer.Add(CreateSphereSupportShape4());
  mSupportShapesContainer.Add(CreateSphereSupportShape5());
  mSupportShapesContainer.Add(CreateObbSupportShape1());
  mSupportShapesContainer.Add(CreateObbSupportShape2());
  mSupportShapesContainer.Add(CreateObbSupportShape3());
  mSupportShapesContainer.Add(CreateObbSupportShape4());
  mSupportShapesContainer.Add(CreateObbSupportShape5());
}
void ShapeFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[0].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[1].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[2].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[3].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[4].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[5].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[6].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[7].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[8].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[9].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[10].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[11].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[12].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[13].mStudentShape, 7, 7, file);
}

void ShapeFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  PrintSupportShapeSpherical(*mSupportShapesContainer.mShapes[14].mStudentShape, 7, 7, file);
}

void GjkFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[0].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[1].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[2].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[3].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest53(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest54(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest55(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest56(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest57(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest58(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest59(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest60(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[4].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest61(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest62(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest63(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest64(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest65(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest66(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest67(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest68(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest69(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[5].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest70(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest71(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest72(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest73(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest74(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest75(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest76(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest77(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[6].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest78(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest79(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest80(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest81(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest82(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest83(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest84(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[7].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest85(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest86(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest87(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest88(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest89(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest90(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[8].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest91(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest92(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest93(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest94(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest95(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[9].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest96(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest97(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest98(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest99(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[10].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest100(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest101(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest102(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[11].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest103(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest104(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[12].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void GjkFuzzTest105(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  SupportShape& shapeA = *mSupportShapesContainer.mShapes[13].mTeacherShape;
  SupportShape& shapeB = *mSupportShapesContainer.mShapes[14].mTeacherShape;
  PrintGjkResults(shapeA, shapeB, 100, debuggingIndex, file, true);
}

void RegisterPointVoronoiRegionsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(PointVoronoiRegions1, list);
  DeclareSimpleUnitTest(PointVoronoiRegions2, list);
  DeclareSimpleUnitTest(PointVoronoiRegions3, list);
}

void RegisterEdgeVoronoiRegionsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(EdgeVoronoiRegions1, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions2, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions3, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions4, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions5, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions6, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions7, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions8, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions9, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions10, list);
  DeclareSimpleUnitTest(EdgeVoronoiRegions11, list);
}

void RegisterTriangleVoronoiRegionsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(TriangleVoronoiRegions1, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions2, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions3, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions4, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions5, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions6, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions7, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions8, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions9, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions10, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions11, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions12, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions13, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions14, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions15, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions16, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions17, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions18, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions19, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions20, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions21, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions22, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions23, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions24, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions25, list);
  DeclareSimpleUnitTest(TriangleVoronoiRegions26, list);
}

void RegisterTetrahedonVoronoiRegionsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions1, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions2, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions3, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions4, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions5, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions6, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions7, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions8, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions9, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions10, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions11, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions12, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions13, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions14, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions15, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions16, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions17, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions18, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions19, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions20, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions21, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions22, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions23, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions24, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions25, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions26, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions27, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions28, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions29, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions30, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions31, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions32, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions33, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions34, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions35, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions36, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions37, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions38, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions39, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions40, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions41, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions42, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions43, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions44, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions45, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions46, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions47, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions48, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions49, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions50, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions51, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions52, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions53, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions54, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions55, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions56, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions57, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions58, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions59, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions60, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions61, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions62, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions63, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions64, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions65, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions66, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions67, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions68, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions69, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions70, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions71, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions72, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions73, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions74, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions75, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions76, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions77, list);
  DeclareSimpleUnitTest(TetrahedonVoronoiRegions78, list);
}

void RegisterVoronoiRegionsStressTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(VoronoiRegionsStress1, list);
  DeclareSimpleUnitTest(VoronoiRegionsStress2, list);
  DeclareSimpleUnitTest(VoronoiRegionsStress3, list);
}

void RegisterSphereSupportTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereSupport1, list);
  DeclareSimpleUnitTest(SphereSupport2, list);
  DeclareSimpleUnitTest(SphereSupport3, list);
  DeclareSimpleUnitTest(SphereSupport4, list);
  DeclareSimpleUnitTest(SphereSupport5, list);
}

void RegisterObbSupportTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(ObbSupport1, list);
  DeclareSimpleUnitTest(ObbSupport2, list);
  DeclareSimpleUnitTest(ObbSupport3, list);
  DeclareSimpleUnitTest(ObbSupport4, list);
  DeclareSimpleUnitTest(ObbSupport5, list);
}

void RegisterGjkClosestPointsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(GjkTest1, list);
  DeclareSimpleUnitTest(GjkTest2, list);
  DeclareSimpleUnitTest(GjkTest3, list);
  DeclareSimpleUnitTest(GjkTest4, list);
  DeclareSimpleUnitTest(GjkTest5, list);
  DeclareSimpleUnitTest(GjkTest6, list);
  //DeclareSimpleUnitTest(GjkTest7, list);
  DeclareSimpleUnitTest(GjkTest8, list);
  DeclareSimpleUnitTest(GjkTest9, list);
}

void RegisterGjkStressTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(GjkStressTest1, list);
  DeclareSimpleUnitTest(GjkStressTest2, list);
  DeclareSimpleUnitTest(GjkStressTest3, list);
  DeclareSimpleUnitTest(GjkStressTest4, list);
  DeclareSimpleUnitTest(GjkStressTest5, list);
}

void RegisterShapeFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(ShapeFuzzTest1, list);
  DeclareSimpleUnitTest(ShapeFuzzTest2, list);
  DeclareSimpleUnitTest(ShapeFuzzTest3, list);
  DeclareSimpleUnitTest(ShapeFuzzTest4, list);
  DeclareSimpleUnitTest(ShapeFuzzTest5, list);
  DeclareSimpleUnitTest(ShapeFuzzTest6, list);
  DeclareSimpleUnitTest(ShapeFuzzTest7, list);
  DeclareSimpleUnitTest(ShapeFuzzTest8, list);
  DeclareSimpleUnitTest(ShapeFuzzTest9, list);
  DeclareSimpleUnitTest(ShapeFuzzTest10, list);
  DeclareSimpleUnitTest(ShapeFuzzTest11, list);
  DeclareSimpleUnitTest(ShapeFuzzTest12, list);
  DeclareSimpleUnitTest(ShapeFuzzTest13, list);
  DeclareSimpleUnitTest(ShapeFuzzTest14, list);
  DeclareSimpleUnitTest(ShapeFuzzTest15, list);
}

void RegisterGjkFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(GjkFuzzTest1, list);
  DeclareSimpleUnitTest(GjkFuzzTest2, list);
  DeclareSimpleUnitTest(GjkFuzzTest3, list);
  DeclareSimpleUnitTest(GjkFuzzTest4, list);
  DeclareSimpleUnitTest(GjkFuzzTest5, list);
  DeclareSimpleUnitTest(GjkFuzzTest6, list);
  DeclareSimpleUnitTest(GjkFuzzTest7, list);
  DeclareSimpleUnitTest(GjkFuzzTest8, list);
  DeclareSimpleUnitTest(GjkFuzzTest9, list);
  DeclareSimpleUnitTest(GjkFuzzTest10, list);
  DeclareSimpleUnitTest(GjkFuzzTest11, list);
  DeclareSimpleUnitTest(GjkFuzzTest12, list);
  DeclareSimpleUnitTest(GjkFuzzTest13, list);
  DeclareSimpleUnitTest(GjkFuzzTest14, list);
  DeclareSimpleUnitTest(GjkFuzzTest15, list);
  DeclareSimpleUnitTest(GjkFuzzTest16, list);
  DeclareSimpleUnitTest(GjkFuzzTest17, list);
  DeclareSimpleUnitTest(GjkFuzzTest18, list);
  DeclareSimpleUnitTest(GjkFuzzTest19, list);
  DeclareSimpleUnitTest(GjkFuzzTest20, list);
  DeclareSimpleUnitTest(GjkFuzzTest21, list);
  DeclareSimpleUnitTest(GjkFuzzTest22, list);
  DeclareSimpleUnitTest(GjkFuzzTest23, list);
  DeclareSimpleUnitTest(GjkFuzzTest24, list);
  DeclareSimpleUnitTest(GjkFuzzTest25, list);
  DeclareSimpleUnitTest(GjkFuzzTest26, list);
  DeclareSimpleUnitTest(GjkFuzzTest27, list);
  DeclareSimpleUnitTest(GjkFuzzTest28, list);
  DeclareSimpleUnitTest(GjkFuzzTest29, list);
  DeclareSimpleUnitTest(GjkFuzzTest30, list);
  DeclareSimpleUnitTest(GjkFuzzTest31, list);
  DeclareSimpleUnitTest(GjkFuzzTest32, list);
  DeclareSimpleUnitTest(GjkFuzzTest33, list);
  DeclareSimpleUnitTest(GjkFuzzTest34, list);
  DeclareSimpleUnitTest(GjkFuzzTest35, list);
  DeclareSimpleUnitTest(GjkFuzzTest36, list);
  DeclareSimpleUnitTest(GjkFuzzTest37, list);
  DeclareSimpleUnitTest(GjkFuzzTest38, list);
  DeclareSimpleUnitTest(GjkFuzzTest39, list);
  DeclareSimpleUnitTest(GjkFuzzTest40, list);
  DeclareSimpleUnitTest(GjkFuzzTest41, list);
  DeclareSimpleUnitTest(GjkFuzzTest42, list);
  DeclareSimpleUnitTest(GjkFuzzTest43, list);
  DeclareSimpleUnitTest(GjkFuzzTest44, list);
  DeclareSimpleUnitTest(GjkFuzzTest45, list);
  DeclareSimpleUnitTest(GjkFuzzTest46, list);
  DeclareSimpleUnitTest(GjkFuzzTest47, list);
  DeclareSimpleUnitTest(GjkFuzzTest48, list);
  DeclareSimpleUnitTest(GjkFuzzTest49, list);
  DeclareSimpleUnitTest(GjkFuzzTest50, list);
  DeclareSimpleUnitTest(GjkFuzzTest51, list);
  DeclareSimpleUnitTest(GjkFuzzTest52, list);
  DeclareSimpleUnitTest(GjkFuzzTest53, list);
  DeclareSimpleUnitTest(GjkFuzzTest54, list);
  DeclareSimpleUnitTest(GjkFuzzTest55, list);
  DeclareSimpleUnitTest(GjkFuzzTest56, list);
  DeclareSimpleUnitTest(GjkFuzzTest57, list);
  DeclareSimpleUnitTest(GjkFuzzTest58, list);
  DeclareSimpleUnitTest(GjkFuzzTest59, list);
  DeclareSimpleUnitTest(GjkFuzzTest60, list);
  DeclareSimpleUnitTest(GjkFuzzTest61, list);
  DeclareSimpleUnitTest(GjkFuzzTest62, list);
  DeclareSimpleUnitTest(GjkFuzzTest63, list);
  DeclareSimpleUnitTest(GjkFuzzTest64, list);
  DeclareSimpleUnitTest(GjkFuzzTest65, list);
  DeclareSimpleUnitTest(GjkFuzzTest66, list);
  DeclareSimpleUnitTest(GjkFuzzTest67, list);
  DeclareSimpleUnitTest(GjkFuzzTest68, list);
  DeclareSimpleUnitTest(GjkFuzzTest69, list);
  DeclareSimpleUnitTest(GjkFuzzTest70, list);
  DeclareSimpleUnitTest(GjkFuzzTest71, list);
  DeclareSimpleUnitTest(GjkFuzzTest72, list);
  DeclareSimpleUnitTest(GjkFuzzTest73, list);
  DeclareSimpleUnitTest(GjkFuzzTest74, list);
  DeclareSimpleUnitTest(GjkFuzzTest75, list);
  DeclareSimpleUnitTest(GjkFuzzTest76, list);
  DeclareSimpleUnitTest(GjkFuzzTest77, list);
  //DeclareSimpleUnitTest(GjkFuzzTest78, list);
  DeclareSimpleUnitTest(GjkFuzzTest79, list);
  DeclareSimpleUnitTest(GjkFuzzTest80, list);
  DeclareSimpleUnitTest(GjkFuzzTest81, list);
  DeclareSimpleUnitTest(GjkFuzzTest82, list);
  DeclareSimpleUnitTest(GjkFuzzTest83, list);
  DeclareSimpleUnitTest(GjkFuzzTest84, list);
  DeclareSimpleUnitTest(GjkFuzzTest85, list);
  DeclareSimpleUnitTest(GjkFuzzTest86, list);
  DeclareSimpleUnitTest(GjkFuzzTest87, list);
  DeclareSimpleUnitTest(GjkFuzzTest88, list);
  DeclareSimpleUnitTest(GjkFuzzTest89, list);
  DeclareSimpleUnitTest(GjkFuzzTest90, list);
  DeclareSimpleUnitTest(GjkFuzzTest91, list);
  DeclareSimpleUnitTest(GjkFuzzTest92, list);
  DeclareSimpleUnitTest(GjkFuzzTest93, list);
  DeclareSimpleUnitTest(GjkFuzzTest94, list);
  DeclareSimpleUnitTest(GjkFuzzTest95, list);
  DeclareSimpleUnitTest(GjkFuzzTest96, list);
  DeclareSimpleUnitTest(GjkFuzzTest97, list);
  DeclareSimpleUnitTest(GjkFuzzTest98, list);
  DeclareSimpleUnitTest(GjkFuzzTest99, list);
  DeclareSimpleUnitTest(GjkFuzzTest100, list);
  DeclareSimpleUnitTest(GjkFuzzTest101, list);
  DeclareSimpleUnitTest(GjkFuzzTest102, list);
  DeclareSimpleUnitTest(GjkFuzzTest103, list);
  DeclareSimpleUnitTest(GjkFuzzTest104, list);
  DeclareSimpleUnitTest(GjkFuzzTest105, list);
}

void InitializeAssignment5Tests()
{
  mTestFns.push_back(AssignmentUnitTestList());
  AssignmentUnitTestList& list = mTestFns.back();
  
  // Add Gjk Tests
  RegisterFuzzTestSupportShapes();
  RegisterPointVoronoiRegionsTests(list);
  RegisterEdgeVoronoiRegionsTests(list);
  RegisterTriangleVoronoiRegionsTests(list);
  RegisterTetrahedonVoronoiRegionsTests(list);
  RegisterVoronoiRegionsStressTests(list);
  RegisterSphereSupportTests(list);
  RegisterObbSupportTests(list);
  RegisterGjkClosestPointsTests(list);
  RegisterGjkStressTests(list);
  RegisterShapeFuzzTests(list);
  RegisterGjkFuzzTests(list);
}
