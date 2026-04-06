#include "Precompiled.hpp"
#include "UnitTests.hpp"
#include "DebugDraw.hpp"
#include <cmath>
std::vector<AssignmentUnitTestList> mTestFns;

//-----------------------------------------------------------------------------UnitTestWrapper
UnitTestWrapper::UnitTestWrapper(UnitTestFn fn, const char* testName)
  : mFn(fn), mTestName(testName)
{

}

void UnitTestWrapper::Run(int debuggingIndex, FILE* file)
{
  Application::mStatistics.Clear();
  mFn(mTestName, debuggingIndex, file);
}

//-----------------------------------------------------------------------------SimpleUnitTesterComponent
SimpleUnitTesterComponent::SimpleUnitTesterComponent(UnitTestFn testFn, const std::string& testName)
{
  mUnitTestFn = testFn;
  mTestName = testName;
}
  
void SimpleUnitTesterComponent::Update(float dt)
{
  int debuggingIndex = mOwner->mApplication->mDebuggingIndex;
  mUnitTestFn(mTestName, debuggingIndex, NULL);
}

//-----------------------------------------------------------------------------SimpleUnitTesterComponent
SimpleUnitTestLevel::SimpleUnitTestLevel(const std::string& levelName, UnitTestFn testFn)
{
  mUnitTestFn = testFn;
  mLevelName = levelName;
}

void SimpleUnitTestLevel::Load(Application* application)
{
  GameObject* obj = application->CreateEmptyObject("Tester");
  obj->Add(new SimpleUnitTesterComponent(mUnitTestFn, mLevelName));
}

std::string SimpleUnitTestLevel::GetName() const
{
  return mLevelName;
}

//-----------------------------------------------------------------------------Unit Test Helper Functions
std::string FormatString(const char* format, ...)
{
  va_list va;
  va_start(va, format);

  int bufferSize;
  bufferSize = _vscprintf(format, va) + 1;
  char* stringBuffer = (char*)alloca((bufferSize + 1)*sizeof(char));
  stringBuffer[bufferSize] = '\0';
  vsprintf_s(stringBuffer, bufferSize, format, va);

  va_end(va);

  return std::string(stringBuffer);
}

float CleanupFloat(float val)
{
  // To deal with the difference between -0.0 and 0.0 if the value is below our
  /// printing epsilon (2 decimal places) then just clamp to 0
  if(Math::Abs(val) < 0.01f)
    val = 0;

  if(val == Math::PositiveMax())
    return val;

  // Round the float to 2 decimal places (0.01)
  float scale = std::pow(10.0f, -2.0f);
  return std::round(val / scale) * scale;

  return val;
}

std::string PrintFloat(float val)
{
  return FormatString("%.2f", CleanupFloat(val));
}

std::string PrintVector2(float x, float y)
{
  return FormatString("(%.2f, %.2f)", CleanupFloat(x), CleanupFloat(y));
}

std::string PrintVector2(const Vector2& vec2)
{
  return PrintVector2(vec2.x, vec2.y);
}

std::string PrintVector3(float x, float y, float z)
{
  return FormatString("(%.2f, %.2f, %.2f)", CleanupFloat(x), CleanupFloat(y), CleanupFloat(z));
}

std::string PrintVector3(const Vector3& vec3)
{
  return PrintVector3(vec3.x, vec3.y, vec3.z);
}
std::string PrintVector4(const Vector4& vec4)
{
  return FormatString("(%.2f, %.2f, %.2f, %.2f)", CleanupFloat(vec4.x), CleanupFloat(vec4.y), CleanupFloat(vec4.z), CleanupFloat(vec4.w));
}

std::string PrintMatrix3(const Matrix3& mat3)
{
  return FormatString("((%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f))",
                      CleanupFloat(mat3.m00), CleanupFloat(mat3.m01), CleanupFloat(mat3.m02),
                      CleanupFloat(mat3.m10), CleanupFloat(mat3.m11), CleanupFloat(mat3.m12),
                      CleanupFloat(mat3.m20), CleanupFloat(mat3.m21), CleanupFloat(mat3.m22));
}

std::string PrintAabb(const Aabb& aabb)
{
  return FormatString("Aabb(%s, %s)", PrintVector3(aabb.mMin).c_str(), PrintVector3(aabb.mMax).c_str());
}

std::string PrintSphere(const Sphere& sphere)
{
  return FormatString("Sphere(%s, %s)", PrintVector3(sphere.mCenter).c_str(), PrintFloat(sphere.mRadius).c_str());
}

std::string PrintTriangle(const Triangle& tri)
{
  return FormatString("Triangle(%s, %s, %s)", PrintVector3(tri.mPoints[0]).c_str(), PrintVector3(tri.mPoints[1]).c_str(), PrintVector3(tri.mPoints[2]).c_str());
}

void PrintAabbData(FILE* outFile, const SpatialPartitionQueryData& rhs)
{
  if(outFile != NULL)
    fprintf(outFile, "    Depth(%d) ClientData(%p) %s\n", rhs.mDepth, rhs.mClientData, PrintAabb(rhs.mAabb).c_str());
}

void PrintSphereData(FILE* outFile, const SpatialPartitionQueryData& rhs)
{
  if(outFile != NULL)
    fprintf(outFile, "    Depth(%d) ClientData(%p) %s\n", rhs.mDepth, rhs.mClientData, PrintSphere(rhs.mBoundingSphere).c_str());
}

void PrintTestHeader(FILE* outFile, const std::string& testName)
{
  if(outFile != NULL)
  {
    fprintf(outFile, "\n");
    fprintf(outFile, "////////////////////////////////////////////////////////////\n");
    fprintf(outFile, "%s\n", testName.c_str());
    fprintf(outFile, "////////////////////////////////////////////////////////////\n");
  }
}

void PrintRayCastResults(SpatialPartition& spatialPartition, const Ray& ray, FILE* outFile)
{
  CastResults castResults;
  spatialPartition.CastRay(ray, castResults);

  // Sort the results to guarantee a consistent ordering
  sort(castResults.mResults.begin(), castResults.mResults.end());

  ray.DebugDraw(10.0f);
  spatialPartition.DebugDraw(-1, Matrix4::cIdentity);

  if(outFile == NULL)
    return;

  fprintf(outFile, "  Test RayCast:\n");
  if(castResults.mResults.empty())
  {
    fprintf(outFile, "    Empty\n");
  }
  else
  {
    for(size_t i = 0; i < castResults.mResults.size(); ++i)
    {
      fprintf(outFile, "    %p %.2f\n", castResults.mResults[i].mClientData, CleanupFloat(castResults.mResults[i].mTime));
    }
  }
}

void PrintFrustumCastResults(SpatialPartition& spatialPartition, const Frustum& frustum, FILE* outFile)
{
  CastResults castResults;
  spatialPartition.CastFrustum(frustum, castResults);

  // Sort the results so we're guaranteed to always produce the same order
  sort(castResults.mResults.begin(), castResults.mResults.end());

  frustum.DebugDraw();
  spatialPartition.DebugDraw(-1, Matrix4::cIdentity);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Test FrustumCast:\n");
    for(size_t i = 0; i < castResults.mResults.size(); ++i)
    {
      fprintf(outFile, "    %p\n", castResults.mResults[i].mClientData);
    }
  }
}

void PrintSpatialPartitionStructure(SpatialPartition& spatialPartition, PrintDataFn printFn, FILE* outFile, bool shouldSort)
{
  std::vector<SpatialPartitionQueryData> results;
  spatialPartition.FilloutData(results);

  if(shouldSort)
    sort(results.begin(), results.end());

  if(outFile != NULL)
  {
    fprintf(outFile, "  Test Spatial Partition Structure:\n");

    if(results.empty())
    {
      fprintf(outFile, "    Empty\n");
    }
    else
    {
      for(size_t i = 0; i < results.size(); ++i)
        printFn(outFile, results[i]);
    }
  }
}

void PrintSpatialPartitionSelfQuery(SpatialPartition& spatialPartition, FILE* outFile)
{
  QueryResults results;
  spatialPartition.SelfQuery(results);
  std::sort(results.mResults.begin(), results.mResults.end());

  spatialPartition.DebugDraw(-1, Matrix4::cIdentity);

  if(outFile != NULL)
  {
    fprintf(outFile, "  Test SelfQuery:\n");
    for(size_t i = 0; i < results.mResults.size(); ++i)
    {
      fprintf(outFile, "    (%p,%p)\n", results.mResults[i].mClientData0, results.mResults[i].mClientData1);
    }
  }
}

bool operator<(const Triangle& rhs, const Triangle& lhs)
{
  // Quick and dirty sorting of triangles. Average their points to compute the
  // centers, round the floats and then sort by x then y then z
  Vector3 centerRhs = (rhs.mPoints[0] + rhs.mPoints[1] + rhs.mPoints[2]) / 3.0f;
  Vector3 centerLhs = (lhs.mPoints[0] + lhs.mPoints[1] + lhs.mPoints[2]) / 3.0f;
  for(uint32_t i = 0; i < 3; ++i)
    centerRhs[i] = CleanupFloat(centerRhs[i]);
  for(uint32_t i = 0; i < 3; ++i)
    centerLhs[i] = CleanupFloat(centerLhs[i]);
  
  for(uint32_t j = 0; j < 3; ++j)
  {
    if(centerRhs[j] != centerLhs[j])
      return centerRhs[j] < centerLhs[j];
  }
  
  return false;
}
