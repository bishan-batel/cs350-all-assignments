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
#include "BspTree.hpp"

float splitWeight = 0.8f;
float epsilon = 0.0001f;
float triExpansionEpsilon = 0.0001f;

std::string GetApplicationDirectory()
{
  char temp[MAX_PATH + 1];
  GetModuleFileName(NULL, temp, MAX_PATH);
  std::string fileName = std::string(temp);

  size_t index = fileName.find_last_of('\\');
  if(index == std::string::npos)
    return std::string();

  std::string applicationDir = fileName.substr(0, index);
  return applicationDir;
}

void LoadMesh(const std::string& fileName, const Matrix4& transform, TriangleList& results)
{
  std::string appDir = GetApplicationDirectory();
  std::string filePath = appDir + "\\" + fileName;

  const size_t bufferSize = 1000;
  char buffer[bufferSize];
  size_t size;

  std::string fileData;
  FILE* file;
  fopen_s(&file, filePath.c_str(), "r");
  if(file == NULL)
    return;

  do
  {
    size = fread_s(buffer, bufferSize - 1, 1, bufferSize - 1, file);
    buffer[size] = NULL;
    fileData += buffer;
  } while(size != 0);
  fclose(file);

  std::vector<Vector3> vertices;
  std::vector<size_t> indices;
  Helpers::LoadObjFile(fileData, vertices, indices);
  
  // Add a scale to the mesh to help reduce numerical issues
  float scale = 100.0f;
  for(size_t i = 0; i < indices.size(); i += 3)
  {
    Vector3 p0 = Math::TransformPoint(transform, vertices[indices[i + 0]]) * scale;
    Vector3 p1 = Math::TransformPoint(transform, vertices[indices[i + 1]]) * scale;
    Vector3 p2 = Math::TransformPoint(transform, vertices[indices[i + 2]]) * scale;
    results.push_back(Triangle(p0, p1, p2));
  }
}

void PrintTriangleList(const std::string listName, const TriangleList& triangles, FILE* file)
{
  if(file == nullptr)
    return;

  // sort the triangles to help guarantee a consistent order
  TriangleList sortedTriangles = triangles;
  std::sort(sortedTriangles.begin(), sortedTriangles.end());

  fprintf(file, "  %s:\n", listName.c_str());
  if(sortedTriangles.empty())
  {
    fprintf(file, "    Empty\n");
  }
  else
  {
    for(size_t i = 0; i < sortedTriangles.size(); ++i)
      fprintf(file, "    %s\n", PrintTriangle(sortedTriangles[i]).c_str());
  }
}

void PrintBspTreeQueryData(const std::string listName, const std::vector<BspTreeQueryData>& results, FILE* file)
{
  if(file == nullptr)
    return;

  // sort the triangles to help guarantee a consistent order
  fprintf(file, "  %s:\n", listName.c_str());
  if(results.empty())
  {
    fprintf(file, "    Empty\n");
  }
  else
  {
    for(size_t i = 0; i < results.size(); ++i)
    {
      const BspTreeQueryData& result = results[i];
      TriangleList sortedTriangles = result.mTriangles;
      std::sort(sortedTriangles.begin(), sortedTriangles.end());
      for(size_t triId = 0; triId < result.mTriangles.size(); ++triId)
        fprintf(file, "    Depth(%d) %s\n", result.mDepth, PrintTriangle(sortedTriangles[triId]).c_str());
    }
  }
}

//void SimplifyAndDrawTriangles(TriangleList& tris)
//{
//  std::vector<Helpers::SimpleTriangle> results;
//  for(size_t i = 0; i < tris.size(); ++i)
//  {
//    Helpers::SimpleTriangle tri;
//    tri.mPoints[0] = tris[i].mPoints[0];
//    tri.mPoints[1] = tris[i].mPoints[1];
//    tri.mPoints[2] = tris[i].mPoints[2];
//    results.push_back(tri);
//  }
//
//  std::vector<Helpers::SimplePolygon> mergedResults;
//  Simplify(results, mergedResults);
//
//  for(size_t i = 0; i < mergedResults.size(); ++i)
//  {
//    Helpers::SimplePolygon& poly = mergedResults[i];
//    for(size_t j = 0; j < poly.mPoints.size(); ++j)
//    {
//      Vector3 p0 = poly.mPoints[j];
//      Vector3 p1 = poly.mPoints[(j + 1) % poly.mPoints.size()];
//      gDebugDrawer->DrawLine(LineSegment(p0, p1));
//    }
//  }
//}

void CreateShape1(TriangleList& results)
{
  results.push_back(Triangle(Vector3(0, 3, 0), Vector3(1, 5, 0), Vector3(1, 5, 1)));
  results.push_back(Triangle(Vector3(1, 5, 1), Vector3(0, 3, 0), Vector3(0, 3, 1)));

  results.push_back(Triangle(Vector3(1, 5, 0), Vector3(3, 5, 0), Vector3(3, 5, 1)));
  results.push_back(Triangle(Vector3(3, 5, 1), Vector3(1, 5, 0), Vector3(1, 5, 1)));
  
  results.push_back(Triangle(Vector3(3, 5, 0), Vector3(4, 4, 0), Vector3(4, 4, 1)));
  results.push_back(Triangle(Vector3(4, 4, 1), Vector3(3, 5, 0), Vector3(3, 5, 1)));
  
  results.push_back(Triangle(Vector3(4, 4, 0), Vector3(4, 3, 0), Vector3(4, 3, 1)));
  results.push_back(Triangle(Vector3(4, 3, 1), Vector3(4, 4, 0), Vector3(4, 4, 1)));
  
  results.push_back(Triangle(Vector3(4, 3, 0), Vector3(2, 2, 0), Vector3(2, 2, 1)));
  results.push_back(Triangle(Vector3(2, 2, 1), Vector3(4, 3, 0), Vector3(4, 3, 1)));
  
  results.push_back(Triangle(Vector3(2, 2, 0), Vector3(2, 0, 0), Vector3(2, 0, 1)));
  results.push_back(Triangle(Vector3(2, 0, 1), Vector3(2, 2, 0), Vector3(2, 2, 1)));
  
  results.push_back(Triangle(Vector3(2, 0, 0), Vector3(0, 3, 0), Vector3(0, 3, 1)));
  results.push_back(Triangle(Vector3(0, 3, 1), Vector3(2, 0, 0), Vector3(2, 0, 1)));
}

void CreateShape2(TriangleList& results)
{
  results.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  results.push_back(Triangle(Vector3(0, 1, 1), Vector3(1, 0, 0), Vector3(1, 0, 1)));

  results.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  results.push_back(Triangle(Vector3(1, 2, 1), Vector3(0, 1, 0), Vector3(0, 1, 1)));

  results.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  results.push_back(Triangle(Vector3(3, 2, 1), Vector3(1, 2, 0), Vector3(1, 2, 1)));

  results.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  results.push_back(Triangle(Vector3(4, 1, 1), Vector3(3, 2, 0), Vector3(3, 2, 1)));

  results.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  results.push_back(Triangle(Vector3(3, 0, 1), Vector3(4, 1, 0), Vector3(4, 1, 1)));

  results.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1.5f, 0), Vector3(2, 1.5f, 1)));
  results.push_back(Triangle(Vector3(2, 1.5f, 1), Vector3(3, 0, 0), Vector3(3, 0, 1)));

  results.push_back(Triangle(Vector3(2, 1.5f, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));
  results.push_back(Triangle(Vector3(1, 0, 1), Vector3(2, 1.5f, 0), Vector3(2, 1.5f, 1)));
}

// Just a small helper to shrink a triangle (so it can be more easily viewed after a split)
Triangle ShrinkTriangle(const Triangle& tri, float scaleFactor)
{
  Vector3 center = (tri.mPoints[0] + tri.mPoints[1] + tri.mPoints[2]) / 3.0f;
  Triangle result;
  for(size_t i = 0; i < 3; ++i)
    result.mPoints[i] = tri.mPoints[i] - scaleFactor * Math::Normalized(tri.mPoints[i] - center);
  return result;
}

void TestSplitPick(const std::string& testName, const TriangleList& triangles, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  BspTree bspTree;
  std::vector<float> scores;
  float epsilon = 0.0001f;
  size_t splitTriangleIndex = bspTree.PickSplitPlane(triangles, splitWeight, epsilon);
  for(size_t i = 0; i < triangles.size(); ++i)
  {
    float score = BspTree::CalculateScore(triangles, i, splitWeight, epsilon);
    scores.push_back(score);
  }

  for(size_t i = 0; i < triangles.size(); ++i)
  {
    DebugShape& shape = triangles[i].DebugDraw();
    if(i == splitTriangleIndex)
      shape.Color(Vector4(0, 0, 1, 1));
  }

  if(file != NULL)
  {
    fprintf(file, "    Bsp Triangle Split Index: %d\n", static_cast<int>(splitTriangleIndex));
    fprintf(file, "    Scores: ");
    for(size_t i = 0; i < scores.size(); ++i)
      fprintf(file, "%s, ", PrintFloat(scores[i]).c_str());
    fprintf(file, "\n");
  }
}

void TestTriangleSplit(const std::string& testName, Plane& plane, Triangle& testTri, float epsilon, int debuggingIndex, FILE* file)
{
  PrintTestHeader(file, testName);

  TriangleList coplanarFront, coplanarBack, front, back;
  BspTree::SplitTriangle(plane, testTri, coplanarFront, coplanarBack, front, back, epsilon);

  testTri.DebugDraw();
  plane.DebugDraw(5.0f);

  float scaleFactor = 0.05f;

  PrintTriangleList("Front", front, file);
  PrintTriangleList("Back", back, file);
  PrintTriangleList("CoplanarFront", coplanarFront, file);
  PrintTriangleList("CoplanarBack", coplanarBack, file);

  for(size_t i = 0; i < coplanarFront.size(); ++i)
    ShrinkTriangle(coplanarFront[i], scaleFactor).DebugDraw().Color(Vector4(0.5f));
  for(size_t i = 0; i < coplanarBack.size(); ++i)
    ShrinkTriangle(coplanarBack[i], scaleFactor).DebugDraw().Color(Vector4(0.5f));
  for(size_t i = 0; i < front.size(); ++i)
    ShrinkTriangle(front[i], scaleFactor).DebugDraw().Color(Vector4(1, 0, 0, 1));
  for(size_t i = 0; i < back.size(); ++i)
    ShrinkTriangle(back[i], scaleFactor).DebugDraw().Color(Vector4(0, 0, 1, 1));
}

void TestBspTreeStructure(const std::string& testName, TriangleList& triangles, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  BspTree bspTree;
  bspTree.Construct(triangles, splitWeight, epsilon);

  std::vector<BspTreeQueryData> results;
  bspTree.FilloutData(results);

  bspTree.DebugDraw(-1, Vector4(1));

  PrintBspTreeQueryData("Spatial Partition Structure", results, file);
}

void TestRay(BspTree& tree, Ray& ray, float epsilon, int debuggingIndex, FILE* file)
{
  // Clear the old statistics
  Application::mStatistics.Clear();

  float t = Math::PositiveMax();
  bool result = tree.RayCast(ray, t, epsilon, triExpansionEpsilon, debuggingIndex);

  // Record how many ray-plane and ray-triangle tests were performed
  size_t rayPlaneCount = Application::mStatistics.mRayPlaneTests;
  size_t rayTriangleCount = Application::mStatistics.mRayTriangleTests;

  if(!result)
    t = 100;
  ray.DebugDraw(t);

  if(file != NULL)
  {
    if(result)
      fprintf(file, "  t: %s\n", PrintFloat(t).c_str());
    else
      fprintf(file, "  false\n");
    fprintf(file, "  Statistics: RayPlane(%d) RayTri(%d)\n", static_cast<int>(rayPlaneCount), static_cast<int>(rayTriangleCount));
  }
}

void TestRaycasts(TriangleList& triangles, Vector3& rayStart, float subDivisions, int debuggingIndex, FILE* file)
{
  BspTree bspTree;
  bspTree.Construct(triangles, splitWeight, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  float deltaRad = (Math::cTwoPi / subDivisions);
  for(float rad = 0; rad <= Math::cTwoPi; rad += deltaRad)
  {
    Vector3 dir = Vector3(Math::Cos(rad), Math::Sin(rad), 0);
    Ray ray(rayStart, dir);

    TestRay(bspTree, ray, epsilon * 2, debuggingIndex, file);
  }
}

void TestRaycastsSpherical(const std::string testName, TriangleList& triangles, Vector3& rayStart, float stacks, float slices, int debuggingIndex, FILE* file)
{
  PrintTestHeader(file, testName);

  BspTree bspTree;
  bspTree.Construct(triangles, splitWeight, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  for(size_t iStack = 0; iStack < stacks; ++iStack)
  {
    for(size_t iSlice = 0; iSlice < slices; ++iSlice)
    {
      float theta = Math::cTwoPi * (iSlice / (float)slices);
      float phi = Math::cPi * ((iStack + 0.5f)/ (float)stacks);

      float x = Math::Sin(theta) * Math::Sin(phi);
      float y = Math::Cos(phi);
      float z = Math::Cos(theta) * Math::Sin(phi);

      Ray ray;
      ray.mStart = rayStart;
      ray.mDirection = Vector3(x, y, z);
      TestRay(bspTree, ray, epsilon * 2, debuggingIndex, file);
    }
  }
}

void TriangleSplitTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(-2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2, 0, 0));
  
  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(-2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(2 * epsilon, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(0.5f, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Vector3(-1, 0, 0), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, 1, 0)), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(0, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(-1, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-1, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0));
  Plane plane;
  plane.Set(Math::Normalized(Vector3(1, -1, 0)), Vector3(-2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(2, 0, 0), Vector3(2, 1, 0), Vector3(2, 1, 1));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2, 0, 0));
  
  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(2, 0, 0), Vector3(2, 1, 1), Vector3(2, 1, 0));
  Plane plane;
  plane.Set(Vector3(1, 0, 0), Vector3(2, 0, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-2, 0, 0), Vector3(2, 4, 0), Vector3(0, 2, 2));
  Plane plane;
  plane.Set(Vector3(1, -1, 0).Normalized(), Vector3(0, 2, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(-2, 0, 0), Vector3(0, 2, 2), Vector3(2, 4, 0));
  Plane plane;
  plane.Set(Vector3(1, -1, 0).Normalized(), Vector3(0, 2, 0));

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void TriangleSplitTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  float epsilon = 0.0001f;
  Triangle tri = Triangle(Vector3(1, 0, -1), Vector3(-1, 0, 0), Vector3(-1, 0, 1));
  Plane plane;
  plane.mData = Vector4(0, 0, -1, 0);

  TestTriangleSplit(testName, plane, tri, epsilon, debuggingIndex, file);
}

void SplitPlanePickTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  for(float i = 0; i < 9.0f; i += 1.0f)
    triangles.push_back(Triangle(Vector3(i, 0, 0), Vector3(i, 0, 1), Vector3(i, 1, 0)));

  TestSplitPick(testName, triangles, debuggingIndex, file);
}

void SplitPlanePickTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  for(float i = 0; i < 6.0f; i += 1.0f)
    triangles.push_back(Triangle(Vector3(i, 0, 0), Vector3(i, 0, 1), Vector3(i, 1, 0)));
  triangles.push_back(Triangle(Vector3(2.5f, 2, 0), Vector3(3.5f, 2, 0), Vector3(3, 2, 2)));
  triangles.push_back(Triangle(Vector3(2.5f, -2, 0), Vector3(3.5f, -2, 0), Vector3(3, -2, 2)));

  TestSplitPick(testName, triangles, debuggingIndex, file);
}

void SplitPlanePickTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(1, 0, 1), Vector3(1, 1, 0)));
  triangles.push_back(Triangle(Vector3(4, 0, 0), Vector3(4, 0, 1), Vector3(4, 1, 0)));
  triangles.push_back(Triangle(Vector3(2.75f, 4, 0), Vector3(2.75f, 4, 1), Vector3(2.75f, 5, 0)));
  triangles.push_back(Triangle(Vector3(3.25f,-4, 0), Vector3(3.25f,-4, 1), Vector3(3.25f,-5, 0)));

  triangles.push_back(Triangle(Vector3(2.5f, 0, 0), Vector3(2.5f, 0, 1), Vector3(3.5f, 1, 0)));
  
  TestSplitPick(testName, triangles, debuggingIndex, file);
}

void SplitPlanePickTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  CreateShape1(triangles);

  TestSplitPick(testName, triangles, debuggingIndex, file);
}

void SplitPlanePickTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  triangles.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  triangles.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  triangles.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  triangles.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1, 0), Vector3(2, 1, 1)));
  triangles.push_back(Triangle(Vector3(2, 1, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));

  TestSplitPick(testName, triangles, debuggingIndex, file);
}

void SplitPlanePickTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  triangles.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  triangles.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  triangles.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  triangles.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1, 0), Vector3(2, 1, 1)));
  triangles.push_back(Triangle(Vector3(2, 1, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));
  // Add in a degenerate triangle here. This should not be picked!!
  triangles.push_back(Triangle(Vector3(4, 4, 0), Vector3(4, 4, 0), Vector3(4, 4, 1)));

  TestSplitPick(testName, triangles, debuggingIndex, file);
}

void TestBspTreeStructure1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 1, 1)));
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(1, 2, 0), Vector3(1, 2, 1)));
  triangles.push_back(Triangle(Vector3(1, 2, 0), Vector3(3, 2, 0), Vector3(3, 2, 1)));
  triangles.push_back(Triangle(Vector3(3, 2, 0), Vector3(4, 1, 0), Vector3(4, 1, 1)));
  triangles.push_back(Triangle(Vector3(4, 1, 0), Vector3(3, 0, 0), Vector3(3, 0, 1)));
  triangles.push_back(Triangle(Vector3(3, 0, 0), Vector3(2, 1.5f, 0), Vector3(2, 1.5f, 1)));
  triangles.push_back(Triangle(Vector3(2, 1.5f, 0), Vector3(1, 0, 0), Vector3(1, 0, 1)));

  TestBspTreeStructure(testName, triangles, debuggingIndex, file);
}

void TestBspTreeStructure2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  CreateShape1(triangles);
  TestBspTreeStructure(testName, triangles, debuggingIndex, file);
}

void TestBspTreeStructure3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  CreateShape2(triangles);
  TestBspTreeStructure(testName, triangles, debuggingIndex, file);
}

void TestBspTreeStructure4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  Matrix4 transform = Matrix4::cIdentity;
  LoadMesh("Cube.txt", transform, triangles);
  TestBspTreeStructure(testName, triangles, debuggingIndex, file);
}

void TestBspTreeStructure5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  Matrix4 transform = Matrix4::cIdentity;
  LoadMesh("Sphere.txt", transform, triangles);
  TestBspTreeStructure(testName, triangles, debuggingIndex, file);
}

void TestBspTreeStructure6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  Matrix4 transform = Matrix4::cIdentity;
  LoadMesh("Octohedron.txt", transform, triangles);
  TestBspTreeStructure(testName, triangles, debuggingIndex, file);
}

#ifndef TestAsStudent
void ValidateRay(std::vector<Triangle>& triangles, BspTree& tree, Ray& ray, float epsilon, int debuggingIndex, FILE* file)
{
  float bspT = Math::PositiveMax();
  Application::mStatistics.mRayTriangleTests = 0;
  Application::mStatistics.mRayPlaneTests = 0;
  bool result = tree.RayCast(ray, bspT, epsilon, triExpansionEpsilon, debuggingIndex);


  float bruteT = Math::PositiveMax();
  for(size_t i = 0; i < triangles.size(); ++i)
  {
    Triangle& tri = triangles[i];
    float triT;
    if(RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], triT, 0.0001f) && triT < bruteT)
    {
      bruteT = triT;
    }
  }

  if(Math::Abs(bruteT - bspT) > 0.0001f)
  {
    __debugbreak();
  }

  if(!result)
    bspT = 100;
  ray.DebugDraw(bspT);

  OutputDebugStringA(FormatString("%d %d\n", Application::mStatistics.mRayTriangleTests, Application::mStatistics.mRayPlaneTests).c_str());
}
#endif

void PopulateSimpleTree(BspTree& bspTree, float epsilon)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(-1, 2, 0), Vector3(1, 2, 0)));
  triangles.push_back(Triangle(Vector3(0, 0, 0.5f), Vector3(-1, 0, -3), Vector3(1, 0, -3)));
  triangles.push_back(Triangle(Vector3(0, 3, -0.5f), Vector3(1, 3, 3), Vector3(-1, 3, 3)));
  bspTree.Construct(triangles, splitWeight, epsilon);
}

void PopulateSimpleTree2(BspTree& bspTree, float epsilon)
{
  TriangleList triangles;
  triangles.push_back(Triangle(Vector3(0, 1.4f, 0), Vector3(-0.1f, 1.5f, 0), Vector3(0.1f, 1.5f, 0)));
  triangles.push_back(Triangle(Vector3(0, 1.5f, -0.1f), Vector3(-0.1f, 1.6f, 0.1f), Vector3(0.1f, 1.6f, 0.1f)));
  triangles.push_back(Triangle(Vector3(0, 1.3f, -0.1f), Vector3(0.1f, 1.4f, 0.1f), Vector3(-0.1f, 1.4f, 0.1f)));
  triangles.push_back(Triangle(Vector3(0, 0, 0.5f), Vector3(-1, 0, -3), Vector3(1, 0, -3)));
  triangles.push_back(Triangle(Vector3(0, 3, -0.5f), Vector3(1, 3, 3), Vector3(-1, 3, 3)));
  bspTree.Construct(triangles, splitWeight, epsilon);
}

void PopulateSimpleTree3(BspTree& bspTree, float epsilon, std::vector<Triangle>& triangles)
{
  triangles.push_back(Triangle(Vector3(0, 1, 0), Vector3(-1, 2, 0), Vector3(1, 2, 0)));
  triangles.push_back(Triangle(Vector3(0, 2, 0.2f), Vector3(-1, 0, -3), Vector3(1, 0, -3)));
  triangles.push_back(Triangle(Vector3(0, 1, -0.2f), Vector3(1, 3, 3), Vector3(-1, 3, 3)));
  triangles.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 1.8f, 1), Vector3(-1, 1.8f, 1)));
  triangles.push_back(Triangle(Vector3(0, 2.25f, 2), Vector3(1, 4, 2), Vector3(-1, 4, 2)));
  triangles.push_back(Triangle(Vector3(0, 0, -1), Vector3(1, 1.5f, -1), Vector3(-1, 1.5f, -1)));
  triangles.push_back(Triangle(Vector3(0, 0.75f, -1.5f), Vector3(1, 3, -2), Vector3(-1, 3, -2)));
  bspTree.Construct(triangles, splitWeight, epsilon);
}

void BspSimpleRayCast1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  Ray ray;
  ray.mDirection = Vector3(0, 0, 1);

  ray.mStart = Vector3(0, 4, -2);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 2.5f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 1.5f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 0.5f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = -1.0f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

void BspSimpleRayCast2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  Ray ray;
  ray.mDirection = Vector3(0, 0, -1);

  ray.mStart = Vector3(0, 4, 2);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 2.5f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 1.5f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 0.5f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = -1.0f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

void BspSimpleRayCast3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  Ray ray;
  ray.mDirection = Math::Normalized(Vector3(0, -1, 1));

  ray.mStart = Vector3(0, 5.75f, -2);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 5.0f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 4.25f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 3.5f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 2.75f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 2.0f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 1.25f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

void BspSimpleRayCast4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  Ray ray;
  ray.mDirection = Math::Normalized(Vector3(0, 1, -1));

  ray.mStart = Vector3(0, 1.75f, 2);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 1.0f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = 0.25f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = -0.5;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = -1.25f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = -2.0f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart.y = -2.75f;
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

// Test coplanar start
void BspSimpleRayCast5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  Ray ray;

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.5f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.5f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.5f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.5f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

// Test coplanar triangles with coplanar start. 
void BspSimpleRayCast6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree(bspTree, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));

  Ray ray;

  ray.mDirection = Math::Normalized(Vector3(0, 5, -1));
  ray.mStart = Vector3(0, -0.5f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
  
  ray.mDirection = Math::Normalized(Vector3(0, 5, 1));
  ray.mStart = Vector3(0, -0.5f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
  
  ray.mDirection = Math::Normalized(Vector3(0, -5, -1));
  ray.mStart = Vector3(0, 0.5f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
  
  ray.mDirection = Math::Normalized(Vector3(0, -5, 1));
  ray.mStart = Vector3(0, 0.5f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
  
  // Bottom tri
  ray.mDirection = Math::Normalized(Vector3(0, -5, -1));
  ray.mStart = Vector3(0, 3.5f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
  
  ray.mDirection = Math::Normalized(Vector3(0, -5, 1));
  ray.mStart = Vector3(0, 3.5f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
  
  ray.mDirection = Math::Normalized(Vector3(0, 5, -1));
  ray.mStart = Vector3(0, 2.5f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, 5, 1));
  ray.mStart = Vector3(0, 2.5f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  // center tri
  ray.mDirection = Math::Normalized(Vector3(0, 3, -1));
  ray.mStart = Vector3(0, 0.75f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, 3, 1));
  ray.mStart = Vector3(0, 0.75f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, -3, -1));
  ray.mStart = Vector3(0, 2.25f, .25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, -3, 1));
  ray.mStart = Vector3(0, 2.25f, -.25f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

// Test multiple coplanar triangles
void BspSimpleRayCast7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 1.0f;
  BspTree bspTree;
  PopulateSimpleTree2(bspTree, epsilon);
  bspTree.DebugDraw(-1, Vector4(1));
  
  Ray ray;

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.7f, 0.05f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, -1, 0));
  ray.mStart = Vector3(0, 1.7f, -.05f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.2f, 0.05f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mDirection = Math::Normalized(Vector3(0, 1, 0));
  ray.mStart = Vector3(0, 1.2f, -.05f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

// Test the last two edge cases (expanded t-values and thick plane check values)
void BspSimpleRayCast8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  float epsilon = 0.25f;
  BspTree bspTree;
  std::vector<Triangle> triangles;
  PopulateSimpleTree3(bspTree, epsilon, triangles);
  bspTree.DebugDraw(debuggingIndex, Vector4(1));

  Ray ray;
  ray.mStart = Vector3(0, -2.21259999f, -0.609817982f);
  ray.mDirection = Vector3(0, 0.984807730f, 0.173648179f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart = Vector3(0, 1.91507995f, -4.35225010f);
  ray.mDirection = Vector3(0, -0.374606609f, 0.927183867f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart = Vector3(0, 2.53433990f, 1.40890002f);
  ray.mDirection = Vector3(0, -0.374606609f, 0.927183867f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart = Vector3(0, 4.39364004f, -0.431046009f);
  ray.mDirection = Vector3(0, -0.970295727f, 0.241921857f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart = Vector3(0, 3.49269009f, -0.701030016f);
  ray.mDirection = Vector3(0, -0.990268052f, -0.139173105f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart = Vector3(0, 1.32701004f, 2.93155003f);
  ray.mDirection = Vector3(0, 0.224951208f, -0.974370003f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);

  ray.mStart = Vector3(0, 0.630509973f, -1.86195004f);
  ray.mDirection = Vector3(0, 0.552694738f, 0.833383799f);
  TestRay(bspTree, ray, epsilon, debuggingIndex, file);
}

void BspRayCastStrestTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList triangles;
  CreateShape1(triangles);

  TestRaycasts(triangles, Vector3(2, 3.5f, 0.5f), 20.0f, debuggingIndex, file);
}

void BspRayCastStrestTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList triangles;
  CreateShape2(triangles);

  TestRaycasts(triangles, Vector3(1, 1, 0.5f), 10.0f, debuggingIndex, file);
  TestRaycasts(triangles, Vector3(3, 1, 0.5f), 10.0f, debuggingIndex, file);
}

void BspRayCastStrestTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  Matrix4 transform = Math::BuildTransform(Vector3::cZero, Quaternion::cIdentity, Vector3(3, 3, 3));
  LoadMesh("Cube.txt", transform, triangles);
  TestRaycastsSpherical(testName, triangles, Vector3(0, 0, 0), 5.0f, 10.0f, debuggingIndex, file);
}

void BspRayCastStrestTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  Matrix4 transform = Math::BuildTransform(Vector3::cZero, Quaternion::cIdentity, Vector3(3, 3, 3));
  LoadMesh("Sphere.txt", transform, triangles);
  TestRaycastsSpherical(testName, triangles, Vector3(0, 0, 0), 5.0f, 10.0f, debuggingIndex, file);
}

void BspRayCastStrestTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList triangles;
  Matrix4 transform = Math::BuildTransform(Vector3::cZero, Quaternion::cIdentity, Vector3(3, 3, 3));
  LoadMesh("Octohedron.txt", transform, triangles);
  TestRaycastsSpherical(testName, triangles, Vector3(0, 0, 0), 5.0f, 10.0f, debuggingIndex, file);
}

typedef void (*BspTreeTestFn)(BspTree& tree1, BspTree& tree2, int debuggingIndex, FILE* file);

void TestUnion(BspTree& tree1, BspTree& tree2, int debuggingIndex, FILE* file)
{
  tree1.Union(&tree2, splitWeight, epsilon);

  tree1.DebugDraw(-1, Vector4(0, 0, 1, 1), 3);

  std::vector<BspTreeQueryData> results;
  tree1.FilloutData(results);
  PrintBspTreeQueryData("Union", results, file);
}

void TestIntersection(BspTree& tree1, BspTree& tree2, int debuggingIndex, FILE* file)
{
  tree1.Intersection(&tree2, splitWeight, epsilon);

  tree1.DebugDraw(-1, Vector4(0, 0, 1, 1), 3);

  std::vector<BspTreeQueryData> results;
  tree1.FilloutData(results);
  PrintBspTreeQueryData("Intersection", results, file);
}

void TestSubtraction(BspTree& tree1, BspTree& tree2, int debuggingIndex, FILE* file)
{
  tree1.Subtract(&tree2, splitWeight, epsilon);

  tree1.DebugDraw(-1, Vector4(0, 0, 1, 1), 3);

  std::vector<BspTreeQueryData> results;
  tree1.FilloutData(results);
  PrintBspTreeQueryData("Subtraction", results, file);
}

void TestBspTree(const std::string& testName, TriangleList& tris1, TriangleList& tris2, BspTreeTestFn testFn, int debuggingIndex, FILE* file)
{
  PrintTestHeader(file, testName);

  BspTree tree1;
  tree1.Construct(tris1, splitWeight, epsilon);
  BspTree tree2;
  tree2.Construct(tris2, splitWeight, epsilon);

  tree1.DebugDraw(-1, Vector4(1, 0, 0, 1), 1);
  tree2.DebugDraw(-1, Vector4(0, 1, 0, 1), 2);

  testFn(tree1, tree2, debuggingIndex, file);
}

void PrintBspTree(const std::string& treeName, BspTree& bspTree, FILE* file)
{
  std::vector<BspTreeQueryData> results;
  bspTree.FilloutData(results);
  PrintBspTreeQueryData(treeName, results, file);
}

void TestBspOperations(TriangleList& tris1, TriangleList& tris2, int debuggingIndex, FILE* file)
{
  BspTree tree1;
  tree1.Construct(tris1, splitWeight, epsilon);
  BspTree tree2;
  tree2.Construct(tris2, splitWeight, epsilon);

  tree1.DebugDraw(-1, Vector4(1, 0, 0, 1), 1);
  tree2.DebugDraw(-1, Vector4(0, 1, 0, 1), 1);
  PrintBspTree("Original Tree1", tree1, file);
  PrintBspTree("Original Tree2", tree2, file);

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Invert();
  tree2.Invert();
  PrintBspTree("Tree1 Inverted", tree1, file);
  PrintBspTree("Tree2 Inverted", tree2, file);

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.ClipTo(&tree2, epsilon);
  tree2.ClipTo(&tree1, epsilon);
  tree1.DebugDraw(-1, Vector4(1, 0, 0, 1), 2);
  tree2.DebugDraw(-1, Vector4(1, 0, 0, 1), 3);
  PrintBspTree("Tree1 Clipped to Tree2", tree1, file);
  PrintBspTree("Tree2 Clipped to Tree1", tree2, file);

  PrintBspTree("Union", tree1, file);
  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Union(&tree2, splitWeight, epsilon);
  tree1.DebugDraw(-1, Vector4(1, 0, 0, 1), 4);
  PrintBspTree("Union", tree1, file);

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Intersection(&tree2, splitWeight, epsilon);
  tree1.DebugDraw(-1, Vector4(1, 0, 0, 1), 5);
  PrintBspTree("Intersection", tree1, file);

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree1.Subtract(&tree2, splitWeight, epsilon);
  tree1.DebugDraw(-1, Vector4(1, 0, 0, 1), 6);
  PrintBspTree("Tree1 - Tree2", tree1, file);

  tree1.Construct(tris1, splitWeight, epsilon);
  tree2.Construct(tris2, splitWeight, epsilon);
  tree2.Subtract(&tree1, splitWeight, epsilon);
  tree2.DebugDraw(-1, Vector4(0, 1, 0, 1), 7);
  PrintBspTree("Tree2 - Tree1", tree2, file);
}

void BspCsgSimple1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(-1, 0, 0), Vector3(1, 0, -1), Vector3(-1, 0, 1)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -1, 0), Vector3(1, 1, 0), Vector3(-1, 1, 0)));
  
  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, 0, -2), Vector3(1, 0, 2), Vector3(-1, 0, 2)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -1, 0.5f), Vector3(1, 1, 0.5f), Vector3(-1, 1, 0.5f)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, 1, -2), Vector3(1, 2, -1), Vector3(-1, 2, -1)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -1, -2), Vector3(1, -2, -1), Vector3(-1, -2, -1)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(-1, -3, -1.5f), Vector3(1, 3, -1.5f), Vector3(-1, 3, -1.5f)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0,  1), Vector3( 1,  2, -2), Vector3(-1,  2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0,  1), Vector3(-1, -2, -2), Vector3( 1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 2, -2), Vector3( 1, -2, -2), Vector3(-1, -2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3( 1,  2, 2), Vector3(0, 0, -1), Vector3(-1,  2, 2)));
  tris2.push_back(Triangle(Vector3(-1, -2, 2), Vector3(0, 0, -1), Vector3( 1, -2, 2)));
  tris2.push_back(Triangle(Vector3( 1, -2, 2), Vector3(1, 2,  2), Vector3(-1, -2, 2)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, -1), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(1, 2, 2), Vector3(0, 0, -1), Vector3(-1, 2, 2)));
  tris2.push_back(Triangle(Vector3(-1, -2, 2), Vector3(0, 0, -1), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 1), Vector3(-1, -2, 2), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 1), Vector3(1, 2, 2), Vector3(-1, 2, 2)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(1, 2, -2), Vector3(-1, 2, -2)));
  tris1.push_back(Triangle(Vector3(0, 0, 1), Vector3(-1, -2, -2), Vector3(1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, 0), Vector3(1, -2, -2), Vector3(-1, -2, -2)));
  tris1.push_back(Triangle(Vector3(1, 0, 0), Vector3(-1, 2, -2), Vector3(1, 2, -2)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(1, 2, 2), Vector3(0, 0, -1), Vector3(-1, 2, 2)));
  tris2.push_back(Triangle(Vector3(-1, -2, 2), Vector3(0, 0, -1), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 0), Vector3(-1, -2, 2), Vector3(1, -2, 2)));
  tris2.push_back(Triangle(Vector3(1, 0, 0), Vector3(1, 2, 2), Vector3(-1, 2, 2)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspCsgSimple10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList tris1;
  tris1.push_back(Triangle(Vector3(0, 0, 2.5), Vector3(1, 2, -0.5), Vector3(-1, 2, -0.5)));
  tris1.push_back(Triangle(Vector3(0, 0, 2.5), Vector3(-1, -2, -0.5), Vector3(1, -2, -0.5)));
  tris1.push_back(Triangle(Vector3(1, 0, 0.5), Vector3(1, -2, -0.5), Vector3(-1, -2, -0.5)));
  tris1.push_back(Triangle(Vector3(1, 0, 0.5), Vector3(-1, 2, -0.5), Vector3(1, 2, -0.5)));
  TriangleList tris2;
  tris2.push_back(Triangle(Vector3(1, 2, 0.5), Vector3(0, 0, -2.5), Vector3(-1, 2, 0.5)));
  tris2.push_back(Triangle(Vector3(-1, -2, 0.5), Vector3(0, 0, -2.5), Vector3(1, -2, 0.5)));
  tris2.push_back(Triangle(Vector3(1, 0, -0.5), Vector3(-1, -2, 0.5), Vector3(1, -2, 0.5)));
  tris2.push_back(Triangle(Vector3(1, 0, -0.5), Vector3(1, 2, 0.5), Vector3(-1, 2, 0.5)));

  TestBspOperations(tris1, tris2, debuggingIndex, file);
}

void BspUnion1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 1, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestUnion, debuggingIndex, file);
}

void BspIntersection1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 1, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestIntersection, debuggingIndex, file);
}

void BspSubtraction1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 1, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Octohedron.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestSubtraction, debuggingIndex, file);
}

void BspUnion2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 0, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(.5f, 1, .5f));
  LoadMesh("Cylinder.txt", transform2, tris2);
  
  TestBspTree(testName, tris1, tris2, TestUnion, debuggingIndex, file);
}

void BspIntersection2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 0, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(.5f, 1, .5f));
  LoadMesh("Cylinder.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestIntersection, debuggingIndex, file);
}

void BspSubtraction2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  TriangleList tris1;
  Matrix4 transform1 = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", transform1, tris1);

  TriangleList tris2;
  Matrix4 transform2 = Math::BuildTransform(Vector3(0, 0, 0), Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(.5f, 1, .5f));
  LoadMesh("Cylinder.txt", transform2, tris2);

  TestBspTree(testName, tris1, tris2, TestSubtraction, debuggingIndex, file);
}

void LargeCsg(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);

  TriangleList cubeTris;
  Matrix4 cubeTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1, 1, 1));
  LoadMesh("Cube.txt", cubeTransform, cubeTris);

  TriangleList sphereTris;
  Matrix4 sphereTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 0), 0.0f), Vector3(1.3f));
  LoadMesh("Sphere.txt", sphereTransform, sphereTris);

  TriangleList cylXTris;
  Matrix4 cylXTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 1), 0.5f * Math::cPi), Vector3(0.7f, 1, 0.7f));
  LoadMesh("Cylinder.txt", cylXTransform, cylXTris);

  TriangleList cylYTris;
  Matrix4 cylYTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(0, 0, 1), 0.0f), Vector3(0.7f, 1, 0.7f));
  LoadMesh("Cylinder.txt", cylYTransform, cylYTris);

  TriangleList cylZTris;
  Matrix4 cylZTransform = Math::BuildTransform(Vector3::cZero, Math::ToQuaternion(Vector3(1, 0, 0), 0.5f * Math::cPi), Vector3(0.7f, 1, 0.7f));
  LoadMesh("Cylinder.txt", cylZTransform, cylZTris);

  float epsilon = 0.0001f;

  BspTree cubeTree;
  cubeTree.Construct(cubeTris, splitWeight, epsilon);
  //cubeTree.DebugDraw(-1, Vector4(1), 1);

  BspTree sphereTree;
  sphereTree.Construct(sphereTris, splitWeight, epsilon);

  BspTree cylXTree;
  cylXTree.Construct(cylXTris, splitWeight, epsilon);
  BspTree cylYTree;
  cylYTree.Construct(cylYTris, splitWeight, epsilon);
  BspTree cylZTree;
  cylZTree.Construct(cylZTris, splitWeight, epsilon);
  
  
  cubeTree.Intersection(&sphereTree, splitWeight, epsilon);
  //cubeTree.DebugDraw(-1, Vector4(1), 2);

  cubeTree.Subtract(&cylXTree, splitWeight, epsilon);
  //cubeTree.DebugDraw(-1, Vector4(1), 3);

  cubeTree.Subtract(&cylYTree, splitWeight, epsilon);
  //cubeTree.DebugDraw(-1, Vector4(1), 4);

  cubeTree.Subtract(&cylZTree, splitWeight, epsilon);
  cubeTree.DebugDraw(-1, Vector4(1), 5);

  std::vector<BspTreeQueryData> results;
  cubeTree.FilloutData(results);
  PrintBspTreeQueryData("Triangles", results, file);
}

void RegisterTriangleSplitTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(TriangleSplitTest1, list);
  DeclareSimpleUnitTest(TriangleSplitTest2, list);
  DeclareSimpleUnitTest(TriangleSplitTest3, list);
  DeclareSimpleUnitTest(TriangleSplitTest4, list);
  DeclareSimpleUnitTest(TriangleSplitTest5, list);
  DeclareSimpleUnitTest(TriangleSplitTest6, list);
  DeclareSimpleUnitTest(TriangleSplitTest7, list);
  DeclareSimpleUnitTest(TriangleSplitTest8, list);
  DeclareSimpleUnitTest(TriangleSplitTest9, list);
  DeclareSimpleUnitTest(TriangleSplitTest10, list);
  DeclareSimpleUnitTest(TriangleSplitTest11, list);
  DeclareSimpleUnitTest(TriangleSplitTest12, list);
  DeclareSimpleUnitTest(TriangleSplitTest13, list);
  DeclareSimpleUnitTest(TriangleSplitTest14, list);
  DeclareSimpleUnitTest(TriangleSplitTest15, list);
  DeclareSimpleUnitTest(TriangleSplitTest16, list);
  DeclareSimpleUnitTest(TriangleSplitTest17, list);
  DeclareSimpleUnitTest(TriangleSplitTest18, list);
  DeclareSimpleUnitTest(TriangleSplitTest19, list);
  DeclareSimpleUnitTest(TriangleSplitTest20, list);
  DeclareSimpleUnitTest(TriangleSplitTest21, list);
  DeclareSimpleUnitTest(TriangleSplitTest22, list);
  DeclareSimpleUnitTest(TriangleSplitTest23, list);
  DeclareSimpleUnitTest(TriangleSplitTest24, list);
  DeclareSimpleUnitTest(TriangleSplitTest25, list);
  DeclareSimpleUnitTest(TriangleSplitTest26, list);
  DeclareSimpleUnitTest(TriangleSplitTest27, list);
  DeclareSimpleUnitTest(TriangleSplitTest28, list);
  DeclareSimpleUnitTest(TriangleSplitTest29, list);
  DeclareSimpleUnitTest(TriangleSplitTest30, list);
  DeclareSimpleUnitTest(TriangleSplitTest31, list);
  DeclareSimpleUnitTest(TriangleSplitTest32, list);
  DeclareSimpleUnitTest(TriangleSplitTest33, list);
}

void RegisterSplitPlanePickTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SplitPlanePickTest1, list);
  DeclareSimpleUnitTest(SplitPlanePickTest2, list);
  DeclareSimpleUnitTest(SplitPlanePickTest3, list);
  DeclareSimpleUnitTest(SplitPlanePickTest4, list);
  DeclareSimpleUnitTest(SplitPlanePickTest5, list);
  DeclareSimpleUnitTest(SplitPlanePickTest6, list);
}

void RegisterTreeConstructionTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(TestBspTreeStructure1, list);
  DeclareSimpleUnitTest(TestBspTreeStructure2, list);
  DeclareSimpleUnitTest(TestBspTreeStructure3, list);
  DeclareSimpleUnitTest(TestBspTreeStructure4, list);
  DeclareSimpleUnitTest(TestBspTreeStructure5, list);
  DeclareSimpleUnitTest(TestBspTreeStructure6, list);
}

void RegisterRayCastTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BspSimpleRayCast1, list);
  DeclareSimpleUnitTest(BspSimpleRayCast2, list);
  DeclareSimpleUnitTest(BspSimpleRayCast3, list);
  DeclareSimpleUnitTest(BspSimpleRayCast4, list);
  DeclareSimpleUnitTest(BspSimpleRayCast5, list);
  DeclareSimpleUnitTest(BspSimpleRayCast6, list);
  DeclareSimpleUnitTest(BspSimpleRayCast7, list);
  DeclareSimpleUnitTest(BspSimpleRayCast8, list);
  DeclareSimpleUnitTest(BspRayCastStrestTest1, list);
  DeclareSimpleUnitTest(BspRayCastStrestTest2, list);
  DeclareSimpleUnitTest(BspRayCastStrestTest3, list);
  //DeclareSimpleUnitTest(BspRayCastStrestTest4, list);
  DeclareSimpleUnitTest(BspRayCastStrestTest5, list);
}

void RegisterCsgTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BspCsgSimple1, list);
  DeclareSimpleUnitTest(BspCsgSimple2, list);
  DeclareSimpleUnitTest(BspCsgSimple3, list);
  DeclareSimpleUnitTest(BspCsgSimple4, list);
  DeclareSimpleUnitTest(BspCsgSimple5, list);
  DeclareSimpleUnitTest(BspCsgSimple6, list);
  DeclareSimpleUnitTest(BspCsgSimple7, list);
  DeclareSimpleUnitTest(BspCsgSimple8, list);
  DeclareSimpleUnitTest(BspCsgSimple9, list);
  DeclareSimpleUnitTest(BspCsgSimple10, list);
  DeclareSimpleUnitTest(BspUnion1, list);
  DeclareSimpleUnitTest(BspIntersection1, list);
  DeclareSimpleUnitTest(BspSubtraction1, list);
  DeclareSimpleUnitTest(BspUnion2, list);
  DeclareSimpleUnitTest(BspIntersection2, list);
  DeclareSimpleUnitTest(BspSubtraction2, list);
  //DeclareSimpleUnitTest(LargeCsg, list);
}

void InitializeAssignment4Tests()
{
  mTestFns.push_back(AssignmentUnitTestList());
  AssignmentUnitTestList& list = mTestFns.back();

  RegisterTriangleSplitTests(list);
  RegisterSplitPlanePickTests(list);
  RegisterTreeConstructionTests(list);
  RegisterRayCastTests(list);
  RegisterCsgTests(list);
}
