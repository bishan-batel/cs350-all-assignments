#include "Precompiled.hpp"
#include "Application.hpp"
#include "Shapes.hpp"
#include "Geometry.hpp"
#include "Math/Utilities.hpp"
#include "DebugDraw.hpp"
#include "Components.hpp"
#include "SimpleNSquared.hpp"
#include "UnitTests.hpp"

void TestAabbSurfaceArea(const Aabb& aabb, FILE* outFile)
{
  float epsilon = 0.001f;
  float surfaceArea = aabb.GetSurfaceArea();

  if(outFile != NULL)
    fprintf(outFile, "  Result: %s\n", PrintFloat(surfaceArea).c_str());
}

void TestAabbVolume(const Aabb& aabb, FILE* outFile)
{
  float epsilon = 0.001f;
  float volume = aabb.GetVolume();

  if(outFile != NULL)
    fprintf(outFile, "  Result: %s\n", PrintFloat(volume).c_str());
}

void TestAabbContains(const Aabb& aabb0, const Aabb& aabb1, FILE* outFile)
{
  bool result = aabb0.Contains(aabb1);

  if(outFile != NULL)
  {
    if(result)
      fprintf(outFile, "  Result:true\n");
    else
      fprintf(outFile, "  Result:false\n");
  }

  Vector4 color0(1);
  Vector4 color1(0.5f);
  if(result == true)
  {
    color0 = Vector4(1, 0, 0, 1);
    color1 = Vector4(0.5f, 0, 0, 0.5f);
  }

  aabb0.DebugDraw().Color(color0);
  aabb1.DebugDraw().Color(color1);
}

void TestAabbTransform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation, Aabb& inputAabb, FILE* outFile)
{
  Aabb resultAabb = inputAabb;
  resultAabb.Transform(scale, rotation, translation);

  // Draw the original aabb and the transformed aabb
  inputAabb.DebugDraw().Color(Vector4(0.5f));
  resultAabb.DebugDraw().Color(Vector4(1));

  // Also, to make it easier to see what we're after, draw the obb that resulted from this transformation
  Matrix4 transform = Math::BuildTransform(translation, rotation, scale);
  DebugShape& transformedShape = inputAabb.DebugDraw();
  transformedShape.Color(Vector4(1, 0, 0, 1));
  transformedShape.SetTransform(transform);
  
  if(outFile != NULL)
    fprintf(outFile, "  %s\n", PrintAabb(resultAabb).c_str());
}

void DrawSphereComputedFromPoints(const std::vector<Vector3>& input, const Sphere& result)
{
  for(size_t i = 0; i < input.size(); ++i)
    gDebugDrawer->DrawPoint(input[i]);
  gDebugDrawer->DrawSphere(result);
}

void TestSphereCentroid(const std::vector<Vector3>& input, FILE* outFile)
{
  float epsilon = 0.001f;

  Sphere sphere;
  sphere.ComputeCentroid(input);
  DrawSphereComputedFromPoints(input, sphere);

  if(outFile != NULL)
    fprintf(outFile, "  %s\n", PrintSphere(sphere).c_str());
}

void TestSphereRitter(const std::vector<Vector3>& input, FILE* outFile)
{
  float epsilon = 0.001f;

  Sphere sphere;
  sphere.ComputeRitter(input);
  DrawSphereComputedFromPoints(input, sphere);

  if(outFile != NULL)
    fprintf(outFile, "  %s\n", PrintSphere(sphere).c_str());
}


void AabbSurfaceAreaTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbSurfaceArea(aabb, file);
}

void AabbSurfaceAreaTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb(Vector3(-1, -2, -3), Vector3(1, 2, 3));
  
  TestAabbSurfaceArea(aabb, file);
}


void AabbVolumeTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbVolume(aabb, file);
}

void AabbVolumeTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb(Vector3(-1, -2, -3), Vector3(1, 2, 3));
  
  TestAabbVolume(aabb, file);
}


void AabbContainsTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-2.5f, -0.5f, -0.5f), Vector3(-1.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(1.5f, -0.5f, -0.5f), Vector3(2.5f, 0.5f, 0.5f));
  
  TestAabbContains(aabb0, aabb1, file);
}

void AabbContainsTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1.5f, -1.5f, -1.5f), Vector3(-0.5f, -0.5f, -0.5f));
  Aabb aabb1(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbContains(aabb0, aabb1, file);
}

void AabbContainsTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(0.5f, 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f));
  Aabb aabb1(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbContains(aabb0, aabb1, file);
}

void AabbContainsTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  Aabb aabb1(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbContains(aabb0, aabb1, file);
}

void AabbContainsTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Aabb aabb0(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  Aabb aabb1(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
  
  TestAabbContains(aabb0, aabb1, file);
}


void AabbTransformTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 scale(Vector3(2, 2, 2));
  Matrix3 rotation = Math::ToMatrix3(Vector3(Vector3(1, 0, 0)).Normalized(), 0);
  Vector3 translation(Vector3(0, 0, 0));
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbTransform(scale, rotation, translation, aabb, file);
}

void AabbTransformTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 scale(Vector3(1, 1, 1));
  Matrix3 rotation = Math::ToMatrix3(Vector3(Vector3(1, 0, 0)).Normalized(), 0);
  Vector3 translation(Vector3(1, 2, 3));
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbTransform(scale, rotation, translation, aabb, file);
}

// Rotate about (1, 0, 0) by 45 degrees
void AabbTransformTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 scale(Vector3(1, 1, 1));
  Matrix3 rotation = Math::ToMatrix3(Vector3(Vector3(1, 0, 0)).Normalized(), 0.785398f);
  Vector3 translation(Vector3(0, 0, 0));
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbTransform(scale, rotation, translation, aabb, file);
}

// Scale by 2, Rotate about (1, 1, 0) by 45 degrees, translate by (0, 1, 0)
void AabbTransformTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 scale(Vector3(2, 2, 2));
  Matrix3 rotation = Math::ToMatrix3(Vector3(Vector3(1, 1, 0)).Normalized(), 0.785398f);
  Vector3 translation(Vector3(0, 1, 0));
  
  Aabb aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));
  
  TestAabbTransform(scale, rotation, translation, aabb, file);
}

void AabbTransformTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 scale(Vector3(1, 1, 1));
  Matrix3 rotation = Math::ToMatrix3(Vector3(Vector3(0, 1, 0)).Normalized(), 0.785398f);
  Vector3 translation(Vector3(0, 0, 0));
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbTransform(scale, rotation, translation, aabb, file);
}

void AabbTransformTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 scale(Vector3(1, 1, 1));
  Matrix3 rotation = Math::ToMatrix3(Vector3(Vector3(0, 0, 1)).Normalized(), 0.785398f);
  Vector3 translation(Vector3(0, 0, 0));
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbTransform(scale, rotation, translation, aabb, file);
}

void AabbTransformTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Vector3 scale(Vector3(1, 1, 1));
  Matrix3 rotation = Math::ToMatrix3(Vector3(Vector3(1, 0, 0)).Normalized(), 3.14159f);
  Vector3 translation(Vector3(0, 0, 0));
  
  Aabb aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1));
  
  TestAabbTransform(scale, rotation, translation, aabb, file);
}

void SphereCentroidTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.97681f, -5.28855f, 2.96304f));
  points.push_back(Vector3(-8.51253f, -4.59517f, -2.79946f));
  points.push_back(Vector3(-4.89425f, 9.7058f, -3.62163f));
  points.push_back(Vector3(8.68465f, -6.4275f, 7.1514f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.96704f, 7.51335f, -2.51991f));
  points.push_back(Vector3(4.92294f, 6.88284f, 9.18394f));
  points.push_back(Vector3(4.60433f, 0.772424f, 6.76809f));
  points.push_back(Vector3(9.92248f, -6.87857f, 7.28629f));
  points.push_back(Vector3(-1.73986f, 0.666829f, -9.73998f));
  points.push_back(Vector3(-7.37114f, -2.69814f, 2.27271f));
  points.push_back(Vector3(-2.584f, -9.24863f, -2.36f));
  points.push_back(Vector3(-3.29081f, 9.03928f, 7.21915f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.62767f, 2.76528f, -8.93796f));
  points.push_back(Vector3(1.72704f, -2.88003f, -3.37931f));
  points.push_back(Vector3(7.5518f, -2.96121f, 0.00640869f));
  points.push_back(Vector3(-8.00043f, -2.2251f, -8.14386f));
  points.push_back(Vector3(-4.61531f, -2.49001f, -7.08609f));
  points.push_back(Vector3(-8.37458f, -0.0698876f, -5.93066f));
  points.push_back(Vector3(7.37114f, -8.3343f, 2.11035f));
  points.push_back(Vector3(3.0076f, 6.9744f, -4.94797f));
  points.push_back(Vector3(1.03183f, -7.98761f, 3.62651f));
  points.push_back(Vector3(-8.83969f, 3.29936f, 7.06107f));
  points.push_back(Vector3(-7.97174f, 8.00836f, -1.49205f));
  points.push_back(Vector3(8.44172f, -5.90381f, 1.49998f));
  points.push_back(Vector3(3.98907f, -0.987884f, 6.69485f));
  points.push_back(Vector3(-5.2794f, -0.556352f, 2.80496f));
  points.push_back(Vector3(-1.67821f, -9.61242f, 2.08045f));
  points.push_back(Vector3(2.11035f, -9.63805f, 9.59533f));
  points.push_back(Vector3(2.88858f, -8.84274f, 8.53267f));
  points.push_back(Vector3(-0.0332654f, -5.33372f, -5.47044f));
  points.push_back(Vector3(0.150456f, 6.35548f, -7.56462f));
  points.push_back(Vector3(8.4228f, 2.8489f, 7.20267f));
  points.push_back(Vector3(-3.14066f, -7.11539f, 0.357372f));
  points.push_back(Vector3(9.68566f, 1.29917f, -6.84133f));
  points.push_back(Vector3(-9.01608f, 6.78152f, -6.87857f));
  points.push_back(Vector3(-1.75451f, 4.41389f, 9.36888f));
  points.push_back(Vector3(9.89563f, -5.17502f, -7.54936f));
  points.push_back(Vector3(-3.50505f, 1.08982f, 4.65194f));
  points.push_back(Vector3(9.33775f, -3.07047f, -4.08551f));
  points.push_back(Vector3(3.549f, 7.26188f, 5.13535f));
  points.push_back(Vector3(7.90643f, -9.86755f, 2.68716f));
  points.push_back(Vector3(4.77462f, -3.09366f, 2.38746f));
  points.push_back(Vector3(-0.868251f, 2.88308f, -0.955535f));
  points.push_back(Vector3(-9.78332f, 2.01147f, 7.83074f));
  points.push_back(Vector3(5.35325f, 2.09754f, -8.81649f));
  points.push_back(Vector3(-5.80309f, -0.359813f, -6.04602f));
  points.push_back(Vector3(6.80776f, 7.68792f, -3.75164f));
  points.push_back(Vector3(-1.60253f, -5.84948f, -6.79067f));
  points.push_back(Vector3(8.31721f, 2.42531f, 2.96976f));
  points.push_back(Vector3(-0.759606f, 7.17826f, -8.26411f));
  points.push_back(Vector3(7.07816f, 1.61107f, 3.51482f));
  points.push_back(Vector3(9.76257f, -0.25544f, -0.174261f));
  points.push_back(Vector3(-9.1052f, 9.55748f, -9.47325f));
  points.push_back(Vector3(9.04599f, 2.32032f, 6.66067f));
  points.push_back(Vector3(-0.81576f, 0.8652f, 0.400097f));
  points.push_back(Vector3(-8.93735f, -7.72149f, -4.00189f));
  points.push_back(Vector3(-8.20917f, 6.3976f, -1.90039f));
  points.push_back(Vector3(2.00659f, 6.51662f, -5.51683f));
  points.push_back(Vector3(5.53636f, -0.811487f, -7.21122f));
  points.push_back(Vector3(-6.65456f, 2.51259f, 7.76177f));
  points.push_back(Vector3(7.97296f, 6.83828f, -0.372021f));
  points.push_back(Vector3(-0.542924f, -3.69854f, -6.01001f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.24458f, -4.38215f, -1.96081f));
  points.push_back(Vector3(-5.43931f, 6.20899f, -0.0796533f));
  points.push_back(Vector3(5.93616f, -2.90811f, -5.94836f));
  points.push_back(Vector3(7.77703f, -4.87594f, -9.48668f));
  points.push_back(Vector3(-6.94266f, 6.26881f, -6.84194f));
  points.push_back(Vector3(-4.23688f, -0.397046f, -1.301f));
  points.push_back(Vector3(2.83486f, -3.6607f, -2.46437f));
  points.push_back(Vector3(0.484939f, 7.01468f, 6.32313f));
  points.push_back(Vector3(6.61794f, -3.66497f, 9.58617f));
  points.push_back(Vector3(9.99695f, -3.84136f, 1.84851f));
  points.push_back(Vector3(3.18094f, 1.19419f, -0.529496f));
  points.push_back(Vector3(2.64199f, 4.27412f, 0.500809f));
  points.push_back(Vector3(-5.22874f, -5.84765f, 6.292f));
  points.push_back(Vector3(-1.57323f, 8.15302f, -4.77767f));
  points.push_back(Vector3(-8.64681f, 2.68899f, 4.25031f));
  points.push_back(Vector3(-1.40294f, -8.73165f, -2.28736f));
  points.push_back(Vector3(0.226753f, -8.53267f, -1.58238f));
  points.push_back(Vector3(-5.51134f, 8.70785f, 9.17478f));
  points.push_back(Vector3(-2.85623f, -1.09592f, 5.2794f));
  points.push_back(Vector3(-4.8027f, -3.72723f, -3.32072f));
  points.push_back(Vector3(6.1919f, -9.4348f, -6.60634f));
  points.push_back(Vector3(3.02713f, -8.12738f, 5.80004f));
  points.push_back(Vector3(-8.47957f, -4.8381f, 1.89367f));
  points.push_back(Vector3(-1.47374f, -1.23569f, -4.20392f));
  points.push_back(Vector3(-2.99905f, 9.87243f, -8.36665f));
  points.push_back(Vector3(5.72191f, -6.15223f, 2.78115f));
  points.push_back(Vector3(-7.40104f, -6.36708f, 8.04132f));
  points.push_back(Vector3(-4.63729f, 2.53395f, 1.29429f));
  points.push_back(Vector3(-8.86349f, -7.58843f, -1.33152f));
  points.push_back(Vector3(-7.81915f, -2.79458f, 8.21039f));
  points.push_back(Vector3(-1.32786f, 1.73193f, 4.59273f));
  points.push_back(Vector3(3.74859f, 1.49754f, -2.68166f));
  points.push_back(Vector3(8.49849f, -8.5168f, 5.60472f));
  points.push_back(Vector3(6.07105f, -9.99817f, -9.24009f));
  points.push_back(Vector3(-4.80697f, -1.32237f, -2.8367f));
  points.push_back(Vector3(7.02506f, -4.93026f, 7.2924f));
  points.push_back(Vector3(3.80779f, -2.05237f, 4.77279f));
  points.push_back(Vector3(5.65844f, 1.64281f, -0.411084f));
  points.push_back(Vector3(-7.4279f, 9.59288f, -8.6877f));
  points.push_back(Vector3(-4.27107f, -0.00701904f, 5.94348f));
  points.push_back(Vector3(8.02728f, 3.48308f, -2.54616f));
  points.push_back(Vector3(8.14753f, 7.38273f, -1.66417f));
  points.push_back(Vector3(1.67699f, 6.51662f, 7.00797f));
  points.push_back(Vector3(-8.29096f, -4.52498f, -8.62484f));
  points.push_back(Vector3(9.64965f, 4.20026f, 0.544145f));
  points.push_back(Vector3(-2.82571f, 3.33232f, 7.60308f));
  points.push_back(Vector3(3.89508f, 5.70666f, 4.21369f));
  points.push_back(Vector3(-1.15329f, 8.53572f, -3.45378f));
  points.push_back(Vector3(-2.24342f, -1.02512f, 9.04477f));
  points.push_back(Vector3(2.84707f, 3.84991f, 0.283517f));
  points.push_back(Vector3(2.64443f, 0.606403f, 1.49815f));
  points.push_back(Vector3(-5.24766f, -3.58928f, 9.54344f));
  points.push_back(Vector3(-1.29368f, 6.89138f, 1.24241f));
  points.push_back(Vector3(-6.30909f, 3.18766f, 4.11908f));
  points.push_back(Vector3(-6.79189f, -0.149846f, -0.488601f));
  points.push_back(Vector3(7.81549f, -7.09098f, 2.01392f));
  points.push_back(Vector3(8.32575f, -1.84606f, 3.46416f));
  points.push_back(Vector3(9.89502f, -6.72658f, -4.14472f));
  points.push_back(Vector3(-2.23914f, 9.682f, -8.76949f));
  points.push_back(Vector3(-6.02527f, -4.41084f, -4.85763f));
  points.push_back(Vector3(-8.51314f, 2.85745f, 1.27659f));
  points.push_back(Vector3(-2.7897f, -0.501419f, 1.37852f));
  points.push_back(Vector3(2.46254f, -9.83703f, -9.16501f));
  points.push_back(Vector3(-2.39723f, 0.627155f, 2.53883f));
  points.push_back(Vector3(-9.05087f, -8.02118f, -5.99536f));
  points.push_back(Vector3(-9.1052f, 6.39027f, 6.19984f));
  points.push_back(Vector3(3.50322f, 4.54817f, 3.08512f));
  points.push_back(Vector3(-9.14914f, 2.25929f, -0.586871f));
  points.push_back(Vector3(-2.68044f, -5.02609f, -0.555131f));
  points.push_back(Vector3(9.07285f, -4.96078f, -9.33226f));
  points.push_back(Vector3(-1.03183f, -8.80734f, 5.08957f));
  points.push_back(Vector3(-4.14228f, 3.66802f, -2.58034f));
  points.push_back(Vector3(2.03345f, 8.34651f, 5.65233f));
  points.push_back(Vector3(4.87533f, 4.16364f, -5.71581f));
  points.push_back(Vector3(-6.60939f, -2.61818f, 2.32887f));
  points.push_back(Vector3(4.18134f, -3.81207f, -5.46556f));
  points.push_back(Vector3(5.21226f, -0.425733f, -3.33232f));
  points.push_back(Vector3(-3.24503f, -3.32194f, 3.49162f));
  points.push_back(Vector3(-8.11396f, 4.44258f, -9.18943f));
  points.push_back(Vector3(-2.172f, -6.28956f, -0.646687f));
  points.push_back(Vector3(0.279855f, 0.758995f, 9.0698f));
  points.push_back(Vector3(6.33412f, 3.64483f, 7.31803f));
  points.push_back(Vector3(-9.27915f, 3.52458f, 7.23319f));
  points.push_back(Vector3(6.26637f, -0.849941f, -0.827357f));
  points.push_back(Vector3(7.21305f, -3.62529f, 6.21937f));
  points.push_back(Vector3(3.16507f, 7.48772f, -4.75814f));
  points.push_back(Vector3(1.21006f, -8.58272f, -0.970184f));
  points.push_back(Vector3(3.78338f, 9.86816f, -8.76339f));
  points.push_back(Vector3(-3.70281f, -0.500809f, 7.07816f));
  points.push_back(Vector3(5.78784f, -3.67046f, 1.66967f));
  points.push_back(Vector3(-6.19617f, 5.78295f, -3.82305f));
  points.push_back(Vector3(1.57567f, -1.0947f, 6.5276f));
  points.push_back(Vector3(-5.73168f, 0.893277f, -3.53435f));
  points.push_back(Vector3(-1.5598f, -8.64864f, -8.62667f));
  points.push_back(Vector3(-0.75106f, 9.17356f, -3.37138f));
  points.push_back(Vector3(-1.06357f, -1.07944f, 5.46129f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.8114f, -0.7828f, 7.94732f));
  points.push_back(Vector3(-9.48485f, -2.06763f, -3.87493f));
  points.push_back(Vector3(-7.21976f, 1.41209f, -7.15812f));
  points.push_back(Vector3(-2.772f, 6.0802f, 9.38292f));
  points.push_back(Vector3(-7.76544f, 8.39534f, 6.77236f));
  points.push_back(Vector3(-1.94922f, -5.96301f, -0.550249f));
  points.push_back(Vector3(4.57198f, -9.29991f, 1.10324f));
  points.push_back(Vector3(-6.28346f, 3.85662f, 1.16611f));
  points.push_back(Vector3(1.04038f, -8.96359f, 7.10135f));
  points.push_back(Vector3(6.01245f, 2.17139f, 2.39174f));
  points.push_back(Vector3(3.51848f, 1.1301f, -3.75225f));
  points.push_back(Vector3(4.25397f, -1.48961f, -4.1319f));
  points.push_back(Vector3(8.55464f, -9.21995f, 9.57213f));
  points.push_back(Vector3(7.83196f, 6.01245f, -8.00531f));
  points.push_back(Vector3(4.26008f, -8.53389f, 3.9256f));
  points.push_back(Vector3(-3.93719f, 5.8623f, 3.56243f));
  points.push_back(Vector3(1.97302f, 3.79681f, 2.38075f));
  points.push_back(Vector3(-1.20945f, -9.90417f, -6.48976f));
  points.push_back(Vector3(4.46882f, -1.59032f, -5.11765f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.94507f, -0.323191f, -2.58217f));
  points.push_back(Vector3(6.48122f, -3.867f, -4.45235f));
  points.push_back(Vector3(5.50096f, 1.11911f, -2.37587f));
  points.push_back(Vector3(8.64498f, -7.87164f, 7.58171f));
  points.push_back(Vector3(6.89749f, -8.24335f, -0.744346f));
  points.push_back(Vector3(9.035f, -2.64626f, 5.97705f));
  points.push_back(Vector3(9.70763f, 8.6932f, -5.71764f));
  points.push_back(Vector3(0.646077f, 9.8468f, -5.6859f));
  points.push_back(Vector3(3.8731f, -1.19663f, -4.28083f));
  points.push_back(Vector3(-7.73736f, -6.47694f, 8.77438f));
  points.push_back(Vector3(0.799279f, -3.22611f, 6.55873f));
  points.push_back(Vector3(-7.49199f, -6.53554f, -9.97192f));
  points.push_back(Vector3(4.48714f, -8.6932f, 2.31971f));
  points.push_back(Vector3(-1.31565f, -0.0735497f, -0.443434f));
  points.push_back(Vector3(-2.50221f, 2.46925f, -6.68752f));
  points.push_back(Vector3(7.09403f, 6.06311f, 0.472732f));
  points.push_back(Vector3(-3.47636f, 0.827357f, 9.63988f));
  points.push_back(Vector3(-9.24009f, -3.45012f, 6.24683f));
  points.push_back(Vector3(2.39601f, 8.92758f, 2.31727f));
  points.push_back(Vector3(-0.564898f, -2.50404f, 1.22715f));
  points.push_back(Vector3(7.80572f, -9.78271f, 0.165715f));
  points.push_back(Vector3(-2.10303f, -5.41734f, 7.06351f));
  points.push_back(Vector3(-4.52803f, 2.29652f, 6.87613f));
  points.push_back(Vector3(4.37727f, 8.96481f, 1.49876f));
  points.push_back(Vector3(2.30201f, 2.79031f, 8.28669f));
  points.push_back(Vector3(-6.05152f, -8.6053f, 3.09244f));
  points.push_back(Vector3(3.89325f, -0.182195f, 9.04477f));
  points.push_back(Vector3(4.41572f, -3.01004f, 9.67589f));
  points.push_back(Vector3(-9.03012f, 7.30522f, -7.17093f));
  points.push_back(Vector3(-2.88797f, -1.63488f, -7.70745f));
  points.push_back(Vector3(7.47917f, -9.07285f, 3.8255f));
  points.push_back(Vector3(-5.39048f, 6.19312f, -7.48527f));
  points.push_back(Vector3(1.69286f, 5.65294f, 1.51097f));
  points.push_back(Vector3(-5.92944f, -4.05988f, 1.83325f));
  points.push_back(Vector3(9.20957f, 3.23893f, 8.71944f));
  points.push_back(Vector3(-8.56502f, -9.77783f, 9.83886f));
  points.push_back(Vector3(2.69265f, 0.108341f, -3.32255f));
  points.push_back(Vector3(0.277413f, 0.290841f, 5.61449f));
  points.push_back(Vector3(-9.85839f, -4.71358f, -7.00369f));
  points.push_back(Vector3(2.45888f, -9.76501f, -3.39152f));
  points.push_back(Vector3(3.80047f, 7.02933f, 1.7069f));
  points.push_back(Vector3(-1.75695f, -4.05622f, 3.87066f));
  points.push_back(Vector3(-1.87658f, 7.89911f, -8.92392f));
  points.push_back(Vector3(3.71075f, -9.72167f, -5.73779f));
  points.push_back(Vector3(9.94812f, -6.12537f, -2.54677f));
  points.push_back(Vector3(5.70727f, -1.22654f, -8.29218f));
  points.push_back(Vector3(9.6588f, -3.91888f, -5.6212f));
  points.push_back(Vector3(1.6245f, -1.02878f, 3.18461f));
  points.push_back(Vector3(-8.73592f, 6.42079f, 8.1048f));
  points.push_back(Vector3(-4.4322f, 3.70769f, -6.53005f));
  points.push_back(Vector3(-5.67614f, -6.71987f, 6.02954f));
  points.push_back(Vector3(6.42384f, -3.15531f, -1.82104f));
  points.push_back(Vector3(4.14716f, 3.043f, 5.82629f));
  points.push_back(Vector3(-0.555742f, 1.54637f, 9.79614f));
  points.push_back(Vector3(1.0947f, 9.90845f, -7.21061f));
  points.push_back(Vector3(-6.17176f, -1.27842f, -1.17038f));
  points.push_back(Vector3(7.68487f, 2.94961f, -6.42872f));
  points.push_back(Vector3(-3.25663f, -0.749229f, -5.75426f));
  points.push_back(Vector3(-4.04462f, -2.00293f, -1.40904f));
  points.push_back(Vector3(1.95654f, -9.48851f, -3.38176f));
  points.push_back(Vector3(9.80895f, 5.22813f, -4.56587f));
  points.push_back(Vector3(1.11667f, 8.08405f, 6.37867f));
  points.push_back(Vector3(-9.81689f, 3.86517f, -5.9563f));
  points.push_back(Vector3(0.296335f, 9.7937f, -9.32981f));
  points.push_back(Vector3(7.54265f, 5.59374f, -6.09363f));
  points.push_back(Vector3(6.1681f, -6.01794f, -1.85705f));
  points.push_back(Vector3(8.6639f, 0.658895f, 6.29749f));
  points.push_back(Vector3(-2.37526f, 9.56481f, -5.98254f));
  points.push_back(Vector3(-4.26862f, 7.70501f, 3.43608f));
  points.push_back(Vector3(2.29896f, -5.47777f, 0.468459f));
  points.push_back(Vector3(-2.37831f, 2.6603f, 3.62835f));
  points.push_back(Vector3(5.74389f, -5.47349f, 2.23182f));
  points.push_back(Vector3(8.62239f, -4.07758f, -0.362255f));
  points.push_back(Vector3(6.5569f, -3.12906f, 6.92495f));
  points.push_back(Vector3(1.14414f, -6.77602f, -0.872524f));
  points.push_back(Vector3(5.91357f, 9.28587f, 6.32984f));
  points.push_back(Vector3(8.66695f, 6.39454f, -6.44703f));
  points.push_back(Vector3(-2.62978f, 1.08188f, 1.32542f));
  points.push_back(Vector3(5.86535f, 8.22138f, 7.86981f));
  points.push_back(Vector3(-5.14328f, 7.84295f, 1.79724f));
  points.push_back(Vector3(6.9335f, 3.32987f, -9.682f));
  points.push_back(Vector3(-3.95673f, 3.12052f, 6.15589f));
  points.push_back(Vector3(3.18827f, 6.86575f, 1.92602f));
  points.push_back(Vector3(-2.79092f, 3.74737f, -4.3675f));
  points.push_back(Vector3(9.4879f, -2.65297f, 1.62084f));
  points.push_back(Vector3(6.87551f, 1.99011f, -1.31016f));
  points.push_back(Vector3(8.42769f, -8.19758f, -2.38929f));
  points.push_back(Vector3(1.28269f, -9.71007f, 3.26701f));
  points.push_back(Vector3(-5.8684f, -7.23258f, -8.67489f));
  points.push_back(Vector3(-2.20374f, 5.33738f, -8.31538f));
  points.push_back(Vector3(6.23219f, -9.67528f, -0.674765f));
  points.push_back(Vector3(5.03647f, -9.88952f, -2.75918f));
  points.push_back(Vector3(-8.13654f, 0.726646f, 3.63201f));
  points.push_back(Vector3(-6.61855f, -2.09754f, 5.51622f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.36421f, 1.99988f, 3.83404f));
  points.push_back(Vector3(-6.66433f, 8.47835f, -3.88104f));
  points.push_back(Vector3(-8.0456f, 4.29243f, 9.37986f));
  points.push_back(Vector3(-7.0745f, 0.540483f, 8.90072f));
  points.push_back(Vector3(2.30811f, 0.324411f, 0.457473f));
  points.push_back(Vector3(0.823084f, -2.5016f, 3.94757f));
  points.push_back(Vector3(-4.2082f, -5.15976f, -0.513016f));
  points.push_back(Vector3(-6.44398f, -2.84524f, 9.4409f));
  points.push_back(Vector3(-8.15912f, -8.64925f, 9.24375f));
  points.push_back(Vector3(6.6393f, -9.79492f, -4.67513f));
  points.push_back(Vector3(-6.71743f, 7.14408f, 9.30113f));
  points.push_back(Vector3(0.780358f, -5.02121f, 9.00204f));
  points.push_back(Vector3(-0.512406f, -6.86819f, -1.39683f));
  points.push_back(Vector3(7.84722f, -1.47618f, -4.87167f));
  points.push_back(Vector3(-2.16468f, 6.87063f, -5.8507f));
  points.push_back(Vector3(1.60985f, 3.98541f, -5.35203f));
  points.push_back(Vector3(3.10831f, 4.92782f, 2.23182f));
  points.push_back(Vector3(-1.06113f, 2.2483f, 4.16181f));
  points.push_back(Vector3(-3.51848f, -5.40819f, -2.69021f));
  points.push_back(Vector3(1.23997f, -6.50197f, -6.32984f));
  points.push_back(Vector3(5.03891f, -2.89407f, 6.90664f));
  points.push_back(Vector3(-4.06354f, -5.7561f, -8.95321f));
  points.push_back(Vector3(-8.29524f, 7.80938f, 1.42308f));
  points.push_back(Vector3(-3.68816f, 9.91577f, 7.92047f));
  points.push_back(Vector3(6.58864f, -4.85824f, -2.01331f));
  points.push_back(Vector3(7.40288f, 4.90646f, 6.34388f));
  points.push_back(Vector3(-3.41777f, 3.0253f, -0.212104f));
  points.push_back(Vector3(8.52046f, -6.62526f, -8.69137f));
  points.push_back(Vector3(-6.95242f, -7.90094f, -0.522782f));
  points.push_back(Vector3(1.7423f, -2.24403f, 3.26884f));
  points.push_back(Vector3(-9.87609f, -2.77932f, 9.41588f));
  points.push_back(Vector3(-7.04703f, -9.75524f, -0.158391f));
  points.push_back(Vector3(1.07883f, -6.04297f, -4.46089f));
  points.push_back(Vector3(-9.82665f, -1.9602f, -0.606403f));
  points.push_back(Vector3(9.68444f, -1.86865f, -2.28187f));
  points.push_back(Vector3(5.52965f, 8.305f, -6.43239f));
  points.push_back(Vector3(-9.63561f, -7.83258f, -9.67772f));
  points.push_back(Vector3(-5.27146f, 6.21387f, 3.69427f));
  points.push_back(Vector3(7.31376f, 2.678f, 4.70199f));
  points.push_back(Vector3(3.91644f, -8.50154f, -0.176092f));
  points.push_back(Vector3(-8.61324f, 7.81243f, -7.00736f));
  points.push_back(Vector3(0.674764f, -1.81616f, 3.32072f));
  points.push_back(Vector3(6.93716f, -7.05618f, -7.45415f));
  points.push_back(Vector3(6.29322f, 1.13559f, -2.86477f));
  points.push_back(Vector3(-8.78292f, 1.73864f, 4.82894f));
  points.push_back(Vector3(2.72195f, 4.18378f, -5.51988f));
  points.push_back(Vector3(9.22422f, 9.06247f, 4.43648f));
  points.push_back(Vector3(2.3484f, 0.406811f, 9.15891f));
  points.push_back(Vector3(9.1876f, -4.95651f, -4.738f));
  points.push_back(Vector3(4.39802f, 8.15424f, -8.21467f));
  points.push_back(Vector3(1.02756f, 8.39167f, 4.53658f));
  points.push_back(Vector3(0.872524f, 9.10276f, 8.04071f));
  points.push_back(Vector3(4.31806f, -5.71276f, 3.74493f));
  points.push_back(Vector3(-0.0424206f, -0.491653f, 0.461745f));
  points.push_back(Vector3(-3.72784f, -9.64171f, 0.961028f));
  points.push_back(Vector3(8.33186f, 5.39354f, -3.43913f));
  points.push_back(Vector3(6.38111f, 5.80676f, -5.69628f));
  points.push_back(Vector3(-1.79235f, -0.843226f, 5.77746f));
  points.push_back(Vector3(5.16037f, -3.6607f, 3.63628f));
  points.push_back(Vector3(2.7897f, 1.56285f, 8.44722f));
  points.push_back(Vector3(-7.52922f, -6.49525f, -0.330516f));
  points.push_back(Vector3(-2.37465f, -8.33857f, 4.07514f));
  points.push_back(Vector3(-1.42613f, 3.95245f, -1.08493f));
  points.push_back(Vector3(-2.4424f, -9.44395f, -3.78216f));
  points.push_back(Vector3(0.956144f, 4.49629f, 1.85217f));
  points.push_back(Vector3(-7.86676f, 5.40025f, 9.78332f));
  points.push_back(Vector3(4.54634f, 5.14206f, -7.17032f));
  points.push_back(Vector3(-1.05258f, -9.77172f, 5.9508f));
  points.push_back(Vector3(4.68795f, -3.02896f, 6.0332f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.16135f, -2.12317f, 2.46376f));
  points.push_back(Vector3(-1.49632f, -9.729f, -2.55532f));
  points.push_back(Vector3(-7.92108f, -1.04099f, -1.77099f));
  points.push_back(Vector3(-6.08142f, -3.35002f, -1.85278f));
  points.push_back(Vector3(-2.69143f, 0.693686f, 2.44911f));
  points.push_back(Vector3(7.89117f, -9.86328f, 5.60167f));
  points.push_back(Vector3(8.83908f, -8.4875f, -7.50175f));
  points.push_back(Vector3(-5.97034f, -8.57479f, -3.1077f));
  points.push_back(Vector3(6.66189f, -8.54732f, 6.96158f));
  points.push_back(Vector3(4.2552f, -9.48302f, -1.49754f));
  points.push_back(Vector3(0.630207f, 6.80593f, 8.17011f));
  points.push_back(Vector3(1.70202f, 6.34632f, -7.65557f));
  points.push_back(Vector3(-2.40455f, -7.00674f, 0.679647f));
  points.push_back(Vector3(3.98175f, 0.99704f, 1.17038f));
  points.push_back(Vector3(1.04465f, 8.08039f, -6.52516f));
  points.push_back(Vector3(8.36116f, 8.76583f, 7.54875f));
  points.push_back(Vector3(-4.34919f, -5.33738f, 7.65801f));
  points.push_back(Vector3(8.60958f, 5.8562f, 4.0788f));
  points.push_back(Vector3(-1.26682f, -0.825526f, -2.48451f));
  points.push_back(Vector3(-5.14328f, 1.66906f, 6.94021f));
  points.push_back(Vector3(-2.64504f, 7.37663f, 3.65764f));
  points.push_back(Vector3(-4.66353f, -9.94873f, -7.30583f));
  points.push_back(Vector3(6.26575f, 5.45763f, 4.9559f));
  points.push_back(Vector3(-5.11826f, -9.07651f, 2.0835f));
  points.push_back(Vector3(-9.49583f, -9.72655f, -6.32618f));
  points.push_back(Vector3(-8.05963f, -8.03827f, -3.24137f));
  points.push_back(Vector3(9.80346f, -7.69768f, -1.89856f));
  points.push_back(Vector3(-3.76446f, -1.0184f, -1.92358f));
  points.push_back(Vector3(-7.54631f, 3.35063f, -8.46675f));
  points.push_back(Vector3(5.48509f, -5.24094f, 3.65947f));
  points.push_back(Vector3(6.46962f, 0.477614f, 8.23542f));
  points.push_back(Vector3(-5.68163f, 1.89612f, 9.99268f));
  points.push_back(Vector3(-1.71361f, -9.55077f, 8.43806f));
  points.push_back(Vector3(4.19477f, -1.70751f, -9.71679f));
  points.push_back(Vector3(6.292f, 7.82037f, 8.08466f));
  points.push_back(Vector3(2.22877f, -3.46416f, 8.73592f));
  points.push_back(Vector3(0.744346f, 6.96768f, 7.32414f));
  points.push_back(Vector3(1.8656f, 3.35246f, -2.27332f));
  points.push_back(Vector3(5.58763f, -3.42082f, 7.29728f));
  points.push_back(Vector3(-8.16828f, -5.12864f, 5.69262f));
  points.push_back(Vector3(-6.21021f, -3.32682f, 9.92309f));
  points.push_back(Vector3(4.18928f, -9.8059f, 3.22001f));
  points.push_back(Vector3(8.67061f, -4.20331f, 2.80068f));
  points.push_back(Vector3(-9.81445f, -7.79656f, -4.46516f));
  points.push_back(Vector3(1.67455f, -8.00348f, -6.60939f));
  points.push_back(Vector3(6.37196f, 3.19498f, -6.33839f));
  points.push_back(Vector3(4.75204f, -1.89917f, 5.9209f));
  points.push_back(Vector3(5.69079f, 6.21509f, 6.57399f));
  points.push_back(Vector3(5.39293f, 2.16956f, 3.37993f));
  points.push_back(Vector3(1.40721f, 3.51543f, -4.86801f));
  points.push_back(Vector3(-3.08512f, 0.28962f, -1.87353f));
  points.push_back(Vector3(8.33857f, -6.2859f, -1.9779f));
  points.push_back(Vector3(8.39351f, 3.59661f, 0.431226f));
  points.push_back(Vector3(8.39961f, 1.64647f, 2.7488f));
  points.push_back(Vector3(-9.97131f, 7.07938f, -2.70669f));
  points.push_back(Vector3(-1.55675f, 1.95227f, -6.45619f));
  points.push_back(Vector3(-6.58071f, -3.17606f, 6.57033f));
  points.push_back(Vector3(0.293894f, -8.28669f, 5.25681f));
  points.push_back(Vector3(-5.51073f, 7.39006f, 6.33717f));
  points.push_back(Vector3(8.22077f, 1.38523f, -9.52513f));
  points.push_back(Vector3(-7.63604f, 4.48775f, 8.95871f));
  points.push_back(Vector3(0.395215f, -0.58443f, 8.71761f));
  points.push_back(Vector3(-7.34855f, 2.54616f, 6.11805f));
  points.push_back(Vector3(-9.40916f, -3.24564f, -5.92456f));
  points.push_back(Vector3(0.999482f, -5.39171f, 7.5042f));
  points.push_back(Vector3(7.0568f, 5.46678f, -6.32801f));
  points.push_back(Vector3(4.83261f, -4.232f, 4.46577f));
  points.push_back(Vector3(-0.408033f, 3.36161f, -3.80169f));
  points.push_back(Vector3(0.472732f, 3.62102f, 3.28532f));
  points.push_back(Vector3(-4.09955f, -3.57524f, -9.83825f));
  points.push_back(Vector3(1.75878f, 2.32398f, 3.22306f));
  points.push_back(Vector3(-3.24259f, 5.42039f, 7.75262f));
  points.push_back(Vector3(-2.57302f, 3.49895f, -5.32884f));
  points.push_back(Vector3(-1.04099f, 4.7795f, 4.96994f));
  points.push_back(Vector3(0.338451f, 9.7235f, -4.83749f));
  points.push_back(Vector3(7.85882f, 3.34513f, 4.31868f));
  points.push_back(Vector3(4.63729f, 7.35527f, -1.98706f));
  points.push_back(Vector3(-9.90051f, -9.83398f, 5.46556f));
  points.push_back(Vector3(9.62401f, 7.32414f, -7.32902f));
  points.push_back(Vector3(4.36201f, 6.86514f, -5.68285f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.03806f, 4.61592f, -1.52379f));
  points.push_back(Vector3(9.10276f, 0.592974f, 6.03992f));
  points.push_back(Vector3(9.86022f, 8.18598f, 3.05765f));
  points.push_back(Vector3(1.26499f, -8.94711f, -0.186468f));
  points.push_back(Vector3(-4.91623f, 6.83645f, 2.24036f));
  points.push_back(Vector3(3.99579f, 4.97787f, 2.84524f));
  points.push_back(Vector3(-1.88391f, 1.72826f, 4.48225f));
  points.push_back(Vector3(-0.143132f, -0.386059f, 2.33741f));
  points.push_back(Vector3(-0.735801f, 9.93774f, -6.68508f));
  points.push_back(Vector3(7.72515f, 8.66878f, 9.91028f));
  points.push_back(Vector3(-5.80859f, 6.65822f, -4.65499f));
  points.push_back(Vector3(-5.04074f, -2.71584f, 2.22633f));
  points.push_back(Vector3(-4.61104f, -9.28526f, -6.04236f));
  points.push_back(Vector3(5.58092f, 9.61364f, 1.93396f));
  points.push_back(Vector3(-1.65319f, -0.907315f, 6.57948f));
  points.push_back(Vector3(5.91479f, -0.375072f, 2.04871f));
  points.push_back(Vector3(-1.99316f, 8.75423f, 5.71703f));
  points.push_back(Vector3(2.01086f, -2.80313f, -7.40898f));
  points.push_back(Vector3(-5.91601f, 4.96445f, 1.08371f));
  points.push_back(Vector3(-8.05414f, -4.79782f, 7.38334f));
  points.push_back(Vector3(6.54592f, 7.28507f, 4.3614f));
  points.push_back(Vector3(3.32987f, 1.60131f, -6.06067f));
  points.push_back(Vector3(7.40898f, 9.72228f, 2.10791f));
  points.push_back(Vector3(-5.49669f, -3.93292f, 2.88858f));
  points.push_back(Vector3(-5.40269f, -7.17215f, -1.55797f));
  points.push_back(Vector3(9.53734f, -4.13129f, -2.25562f));
  points.push_back(Vector3(0.35432f, 5.26475f, 9.96277f));
  points.push_back(Vector3(-5.36058f, 3.11258f, 3.72784f));
  points.push_back(Vector3(-6.85476f, -6.99332f, -6.63503f));
  points.push_back(Vector3(9.60509f, 7.52434f, 8.0987f));
  points.push_back(Vector3(-9.3646f, -1.52745f, -6.08203f));
  points.push_back(Vector3(4.80697f, 8.19025f, -3.28166f));
  points.push_back(Vector3(4.61959f, 1.89306f, -7.16178f));
  points.push_back(Vector3(4.62203f, 6.65334f, 4.69588f));
  points.push_back(Vector3(-0.682089f, -4.28205f, -3.82244f));
  points.push_back(Vector3(4.44014f, 4.86007f, -4.41511f));
  points.push_back(Vector3(-5.36424f, -7.72698f, -0.411695f));
  points.push_back(Vector3(-6.84072f, 7.29606f, 6.15223f));
  points.push_back(Vector3(-1.64464f, 7.72454f, -5.20554f));
  points.push_back(Vector3(-1.29551f, 9.64904f, 4.68307f));
  points.push_back(Vector3(-1.68249f, 4.20026f, 7.19901f));
  points.push_back(Vector3(3.24442f, 9.03317f, -7.26493f));
  points.push_back(Vector3(-0.0979644f, 9.15464f, 2.41127f));
  points.push_back(Vector3(9.78271f, 5.68835f, -4.41755f));
  points.push_back(Vector3(8.05353f, 9.20713f, 4.96384f));
  points.push_back(Vector3(5.30625f, -4.35163f, -2.11158f));
  points.push_back(Vector3(-0.754723f, -2.69997f, 7.69097f));
  points.push_back(Vector3(0.072329f, -9.38536f, -9.12595f));
  points.push_back(Vector3(-0.419019f, -0.434889f, 7.62139f));
  points.push_back(Vector3(-9.53001f, -2.266f, 7.91376f));
  points.push_back(Vector3(-6.74795f, 4.24543f, -3.67962f));
  points.push_back(Vector3(-3.31889f, 0.599078f, -0.425123f));
  points.push_back(Vector3(3.26151f, 1.9541f, -8.15485f));
  points.push_back(Vector3(8.59127f, -3.88531f, -0.805383f));
  points.push_back(Vector3(6.19678f, 6.88772f, -1.53844f));
  points.push_back(Vector3(-3.42387f, -7.61467f, -1.48839f));
  points.push_back(Vector3(-2.57179f, 4.91684f, -8.63155f));
  points.push_back(Vector3(-9.63012f, -7.03299f, -7.66289f));
  points.push_back(Vector3(4.84359f, 9.17722f, -0.481277f));
  points.push_back(Vector3(1.77892f, -4.78744f, 4.0611f));
  points.push_back(Vector3(-9.31944f, -0.514847f, -7.38151f));
  points.push_back(Vector3(7.96625f, 7.51396f, -7.40226f));
  points.push_back(Vector3(-5.33006f, 0.96408f, 1.10141f));
  points.push_back(Vector3(3.97137f, -5.50645f, 8.60775f));
  points.push_back(Vector3(9.99329f, 2.34107f, 2.72805f));
  points.push_back(Vector3(-1.55675f, 3.15653f, -4.03058f));
  points.push_back(Vector3(-0.912198f, 7.82159f, 9.06247f));
  points.push_back(Vector3(7.89239f, -0.873745f, 6.11011f));
  points.push_back(Vector3(1.52623f, 0.890225f, -8.18903f));
  points.push_back(Vector3(4.45845f, 2.93374f, -9.68322f));
  points.push_back(Vector3(-6.44337f, -3.78521f, -4.52559f));
  points.push_back(Vector3(-7.10318f, -5.68529f, 1.14231f));
  points.push_back(Vector3(1.00497f, 9.63073f, 1.36876f));
  points.push_back(Vector3(-3.4611f, 0.17487f, -9.07956f));
  points.push_back(Vector3(-3.55449f, 8.40815f, -8.56746f));
  points.push_back(Vector3(2.05847f, -8.22871f, 5.78234f));
  points.push_back(Vector3(0.540483f, -2.82388f, 0.70101f));
  points.push_back(Vector3(-8.65535f, 1.21799f, -0.191962f));
  points.push_back(Vector3(8.53755f, -8.54305f, -2.44911f));
  points.push_back(Vector3(-1.82287f, -0.821253f, -1.26499f));
  points.push_back(Vector3(8.77743f, -6.67043f, 3.20414f));
  points.push_back(Vector3(-9.72228f, 7.08121f, 8.25251f));
  points.push_back(Vector3(3.03324f, -5.30381f, 9.02463f));
  points.push_back(Vector3(-3.91522f, 0.779138f, -8.95077f));
  points.push_back(Vector3(-8.77133f, -4.4377f, 4.42549f));
  points.push_back(Vector3(-7.17887f, -9.0521f, -8.19025f));
  points.push_back(Vector3(-2.05786f, 0.0613427f, 2.1427f));
  points.push_back(Vector3(-9.38841f, -9.02097f, -7.18314f));
  points.push_back(Vector3(7.69036f, 4.28327f, -6.25111f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.30723f, -1.02512f, 6.51357f));
  points.push_back(Vector3(6.34693f, -9.30174f, 2.48695f));
  points.push_back(Vector3(-6.59841f, 9.38597f, 2.54433f));
  points.push_back(Vector3(7.85028f, 3.24076f, -3.81573f));
  points.push_back(Vector3(-9.10337f, 3.4611f, -6.33778f));
  points.push_back(Vector3(-4.61959f, -4.75692f, 4.65377f));
  points.push_back(Vector3(7.26981f, 3.35551f, 3.87371f));
  points.push_back(Vector3(4.26679f, 2.43019f, 4.35285f));
  points.push_back(Vector3(-3.98358f, -1.27842f, -2.45521f));
  points.push_back(Vector3(-9.83032f, 5.2739f, -9.76806f));
  points.push_back(Vector3(6.92129f, 5.0444f, 7.4633f));
  points.push_back(Vector3(-4.22956f, -3.62957f, -8.64559f));
  points.push_back(Vector3(6.00085f, 3.41777f, -3.34147f));
  points.push_back(Vector3(1.77648f, -2.05176f, 6.72353f));
  points.push_back(Vector3(2.93008f, -6.49892f, -6.14856f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.60204f, 8.04926f, -7.19352f));
  points.push_back(Vector3(-7.34123f, -1.03793f, 7.77764f));
  points.push_back(Vector3(-7.75262f, -6.18763f, -8.98129f));
  points.push_back(Vector3(3.21757f, -3.39885f, -7.79473f));
  points.push_back(Vector3(-8.95321f, 8.79818f, 8.9996f));
  points.push_back(Vector3(9.19614f, -7.2808f, 0.265816f));
  points.push_back(Vector3(1.89734f, -1.67577f, -7.31254f));
  points.push_back(Vector3(5.84216f, 6.11072f, -9.29075f));
  points.push_back(Vector3(-3.09977f, -0.24018f, -7.29972f));
  points.push_back(Vector3(1.88818f, 2.5895f, 8.80795f));
  points.push_back(Vector3(-4.15998f, -4.28144f, 9.04538f));
  points.push_back(Vector3(-7.48894f, -5.22324f, -9.00937f));
  points.push_back(Vector3(0.300608f, 7.56706f, 3.02713f));
  points.push_back(Vector3(6.34632f, -7.13675f, -2.84158f));
  points.push_back(Vector3(2.91788f, 9.14914f, -8.39778f));
  points.push_back(Vector3(-4.20942f, -6.39332f, 3.9256f));
  points.push_back(Vector3(6.93106f, -9.7589f, -9.2938f));
  points.push_back(Vector3(6.00269f, -3.60088f, 7.28263f));
  points.push_back(Vector3(5.62731f, -4.0434f, 3.82855f));
  points.push_back(Vector3(8.78048f, 2.78298f, 0.521561f));
  points.push_back(Vector3(-0.890836f, 4.80148f, 2.76284f));
  points.push_back(Vector3(-4.44502f, -8.21467f, 5.35508f));
  points.push_back(Vector3(9.61119f, -3.29264f, 1.55736f));
  points.push_back(Vector3(1.39195f, -9.66979f, -6.59169f));
  points.push_back(Vector3(-1.18198f, -4.31684f, 4.74105f));
  points.push_back(Vector3(-8.20246f, -1.63915f, -5.78661f));
  points.push_back(Vector3(-6.19251f, 2.22816f, -0.630817f));
  points.push_back(Vector3(-1.30161f, 4.5793f, -3.03446f));
  points.push_back(Vector3(-5.17441f, -9.13327f, -0.154729f));
  points.push_back(Vector3(-6.29749f, 3.45378f, 9.12534f));
  points.push_back(Vector3(-9.34996f, -9.47325f, 7.33329f));
  points.push_back(Vector3(8.22321f, -0.216987f, 7.95038f));
  points.push_back(Vector3(8.29401f, 2.56142f, -5.63524f));
  points.push_back(Vector3(3.64177f, 6.23219f, 3.75042f));
  points.push_back(Vector3(-4.33271f, -4.52742f, -7.53899f));
  points.push_back(Vector3(-6.94571f, 4.28327f, -1.4835f));
  points.push_back(Vector3(1.22959f, -0.655843f, 8.76705f));
  points.push_back(Vector3(-8.39167f, -7.47917f, 3.18033f));
  points.push_back(Vector3(-4.11603f, -0.890836f, 9.28098f));
  points.push_back(Vector3(-2.34535f, -9.41527f, 3.48491f));
  points.push_back(Vector3(2.16712f, 6.61916f, 3.82672f));
  points.push_back(Vector3(-5.66881f, 4.9797f, -9.08628f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.27732f, -2.62062f, -9.9353f));
  points.push_back(Vector3(6.65334f, 5.13108f, -1.56285f));
  points.push_back(Vector3(1.89673f, 2.58583f, -6.28773f));
  points.push_back(Vector3(-8.43745f, 1.88635f, -3.41227f));
  points.push_back(Vector3(-6.41591f, -5.05295f, -3.16263f));
  points.push_back(Vector3(-5.88427f, 0.769372f, -6.65883f));
  points.push_back(Vector3(4.03058f, -1.26988f, 8.38557f));
  points.push_back(Vector3(5.99902f, -0.105289f, 8.57356f));
  points.push_back(Vector3(9.71068f, -0.0320446f, -7.36992f));
  points.push_back(Vector3(1.00986f, -3.23771f, -3.137f));
  points.push_back(Vector3(-5.60411f, -1.44322f, -4.99435f));
  points.push_back(Vector3(-8.76461f, -1.30528f, -0.771202f));
  points.push_back(Vector3(4.82162f, 8.67367f, -1.41942f));
  points.push_back(Vector3(4.42122f, -9.64476f, 1.74657f));
  points.push_back(Vector3(-8.69015f, 7.45598f, 6.37745f));
  points.push_back(Vector3(-8.06085f, 5.05905f, -4.08124f));
  points.push_back(Vector3(3.76995f, -4.326f, -6.65517f));
  points.push_back(Vector3(-9.08872f, 4.279f, 2.5663f));
  points.push_back(Vector3(-7.24052f, -3.66131f, 5.14512f));
  points.push_back(Vector3(4.03119f, 9.65392f, 1.36387f));
  points.push_back(Vector3(4.24055f, 0.365916f, -8.61385f));
  points.push_back(Vector3(9.14975f, 6.34388f, 4.43403f));
  points.push_back(Vector3(7.77764f, -4.98398f, 1.16794f));
  points.push_back(Vector3(-9.92554f, -1.62999f, 7.13736f));
  points.push_back(Vector3(-6.22791f, 3.36772f, -8.85189f));
  points.push_back(Vector3(-7.10196f, -2.01453f, 4.86923f));
  points.push_back(Vector3(8.32087f, 4.04767f, -6.54347f));
  points.push_back(Vector3(0.954313f, 4.79476f, 3.75774f));
  points.push_back(Vector3(-3.03262f, -7.61834f, -5.01816f));
  points.push_back(Vector3(8.72799f, 8.62545f, -1.78442f));
  points.push_back(Vector3(-6.46046f, 9.45006f, 3.56426f));
  points.push_back(Vector3(3.21635f, 5.68529f, 1.76427f));
  points.push_back(Vector3(9.7821f, 6.84072f, -7.29179f));
  points.push_back(Vector3(-7.17215f, 2.72927f, 1.96265f));
  points.push_back(Vector3(8.30012f, -5.37523f, -2.71279f));
  points.push_back(Vector3(1.44993f, -8.67122f, 8.13532f));
  points.push_back(Vector3(7.74956f, 0.713217f, -9.3115f));
  points.push_back(Vector3(-8.69991f, -3.56304f, -5.63829f));
  points.push_back(Vector3(7.83746f, -6.10523f, 1.57323f));
  points.push_back(Vector3(-4.9205f, -7.34001f, -7.55669f));
  points.push_back(Vector3(5.0383f, -7.14225f, -1.72216f));
  points.push_back(Vector3(8.29951f, -9.3646f, 2.70486f));
  points.push_back(Vector3(-9.57823f, 2.01025f, -6.51235f));
  points.push_back(Vector3(-1.04953f, -2.99173f, 7.10318f));
  points.push_back(Vector3(-8.80978f, 4.28388f, -6.46779f));
  points.push_back(Vector3(-3.59966f, -2.6487f, -8.35688f));
  points.push_back(Vector3(9.90417f, 4.01166f, 8.13593f));
  points.push_back(Vector3(7.07511f, -5.64318f, -3.90118f));
  points.push_back(Vector3(2.4424f, -8.29768f, 0.0564587f));
  points.push_back(Vector3(-6.42689f, 6.34266f, -9.08261f));
  points.push_back(Vector3(-9.98169f, -0.184637f, 2.39723f));
  points.push_back(Vector3(8.92087f, 5.98315f, -3.43059f));
  points.push_back(Vector3(-5.51622f, -3.25358f, -3.1492f));
  points.push_back(Vector3(1.124f, 8.75668f, -8.12006f));
  points.push_back(Vector3(-7.67693f, 7.90399f, 1.36143f));
  points.push_back(Vector3(-2.39051f, -2.64687f, 7.60674f));
  points.push_back(Vector3(-6.46046f, 0.22187f, -6.95181f));
  points.push_back(Vector3(6.99637f, 4.73434f, -7.81854f));
  points.push_back(Vector3(-0.357372f, 2.49245f, 4.32905f));
  points.push_back(Vector3(-0.591754f, 7.15995f, -9.33103f));
  points.push_back(Vector3(-7.3516f, 0.779748f, 9.4055f));
  points.push_back(Vector3(8.79696f, -8.16034f, -2.00964f));
  points.push_back(Vector3(6.89322f, 8.54244f, -2.73782f));
  points.push_back(Vector3(3.88104f, -5.64745f, -4.03546f));
  points.push_back(Vector3(-9.99451f, 7.3748f, -1.57628f));
  points.push_back(Vector3(0.920134f, 1.17649f, 3.13578f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.16889f, 4.91012f, -7.63115f));
  points.push_back(Vector3(-7.33024f, 5.4387f, -9.00571f));
  points.push_back(Vector3(-1.87597f, 9.19919f, -2.98929f));
  points.push_back(Vector3(5.18296f, -8.30683f, -8.46492f));
  points.push_back(Vector3(6.5331f, -9.96643f, 7.8692f));
  points.push_back(Vector3(-2.10852f, 1.60192f, 6.04236f));
  points.push_back(Vector3(5.29527f, 1.95593f, 2.29957f));
  points.push_back(Vector3(-0.459914f, -4.74166f, 9.01547f));
  points.push_back(Vector3(5.30992f, -7.1746f, 3.80291f));
  points.push_back(Vector3(-7.87713f, 7.92901f, -5.35386f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.60958f, -4.91806f, 2.71218f));
  points.push_back(Vector3(6.82547f, -5.87085f, 1.32725f));
  points.push_back(Vector3(-1.70751f, 4.05255f, 9.80102f));
  points.push_back(Vector3(-5.51866f, -8.35627f, 5.59374f));
  points.push_back(Vector3(0.270699f, -1.86193f, -5.58031f));
  points.push_back(Vector3(-0.803552f, 4.185f, 0.705893f));
  points.push_back(Vector3(-1.64647f, 8.76705f, 2.48817f));
  points.push_back(Vector3(-8.64864f, 9.94385f, 2.83242f));
  points.push_back(Vector3(-4.45174f, 1.13254f, 9.54222f));
  points.push_back(Vector3(9.75646f, 0.000915527f, 4.95224f));
  points.push_back(Vector3(7.99249f, 0.339671f, 3.45317f));
  points.push_back(Vector3(9.96399f, 3.92437f, 8.43013f));
  points.push_back(Vector3(5.1561f, 6.03931f, -5.15793f));
  points.push_back(Vector3(-9.84191f, 0.784631f, 3.93658f));
  points.push_back(Vector3(5.12192f, -7.55791f, -0.558184f));
  points.push_back(Vector3(-3.21451f, 4.0556f, -8.63521f));
  points.push_back(Vector3(-8.98495f, -9.47874f, -2.95022f));
  points.push_back(Vector3(-8.93735f, -7.99249f, -1.11362f));
  points.push_back(Vector3(3.12418f, 3.75103f, 7.97357f));
  points.push_back(Vector3(8.62728f, 1.81066f, 1.50059f));
  points.push_back(Vector3(0.675985f, -9.48607f, 2.6072f));
  points.push_back(Vector3(0.401928f, -1.8955f, 1.80395f));
  points.push_back(Vector3(0.0820947f, 7.73614f, -4.54085f));
  points.push_back(Vector3(-4.22834f, 7.77581f, -7.22465f));
  points.push_back(Vector3(5.24277f, 3.94513f, -5.98987f));
  points.push_back(Vector3(-8.15241f, 4.57625f, 6.64663f));
  points.push_back(Vector3(-7.25272f, 8.27815f, -2.1305f));
  points.push_back(Vector3(-2.50771f, -7.40104f, -2.1598f));
  points.push_back(Vector3(2.96487f, -2.54067f, -2.86538f));
  points.push_back(Vector3(0.343333f, -1.11545f, 4.13862f));
  points.push_back(Vector3(-0.399487f, 7.97235f, 3.30302f));
  points.push_back(Vector3(-8.17743f, 5.17808f, -8.9874f));
  points.push_back(Vector3(-2.12989f, 8.02851f, 3.90545f));
  points.push_back(Vector3(-8.41487f, 9.51781f, 2.09265f));
  points.push_back(Vector3(7.39128f, 6.01062f, 5.8562f));
  points.push_back(Vector3(-8.53572f, -7.56462f, 6.43178f));
  points.push_back(Vector3(-7.19352f, 1.31871f, 4.70077f));
  points.push_back(Vector3(2.32337f, -7.4163f, 3.10892f));
  points.push_back(Vector3(5.5327f, -7.49748f, -9.85046f));
  points.push_back(Vector3(7.70074f, -4.9144f, 8.27876f));
  points.push_back(Vector3(6.27613f, -1.3126f, -5.15122f));
  points.push_back(Vector3(-1.64464f, 5.32762f, 1.53966f));
  points.push_back(Vector3(2.72012f, -3.8786f, -4.10077f));
  points.push_back(Vector3(-5.80981f, 3.73882f, -0.0570697f));
  points.push_back(Vector3(-7.56706f, -5.30686f, 9.6118f));
  points.push_back(Vector3(7.09464f, -4.07025f, -8.42158f));
  points.push_back(Vector3(-9.85168f, -8.05841f, 1.89306f));
  points.push_back(Vector3(-7.47429f, 9.74303f, 9.01425f));
  points.push_back(Vector3(-6.37989f, -3.01492f, -7.99188f));
  points.push_back(Vector3(-8.42219f, 8.97763f, 8.36482f));
  points.push_back(Vector3(5.71276f, 6.34999f, -1.23447f));
  points.push_back(Vector3(-8.58211f, -4.8796f, -5.04013f));
  points.push_back(Vector3(4.82101f, -9.55016f, 7.87652f));
  points.push_back(Vector3(2.80374f, -4.03424f, -6.03504f));
  points.push_back(Vector3(3.55205f, 5.90014f, 9.93347f));
  points.push_back(Vector3(1.39134f, -4.01532f, -3.61675f));
  points.push_back(Vector3(-0.604572f, -2.50587f, -7.47429f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.59777f, 1.16977f, -5.38682f));
  points.push_back(Vector3(2.03284f, -8.44722f, -7.42912f));
  points.push_back(Vector3(-3.95306f, -9.76134f, -8.82565f));
  points.push_back(Vector3(-4.28755f, -3.57952f, -7.74102f));
  points.push_back(Vector3(-1.92785f, 8.99655f, -3.91095f));
  points.push_back(Vector3(-9.63988f, 5.4564f, 2.75735f));
  points.push_back(Vector3(4.1319f, 7.04093f, 6.62282f));
  points.push_back(Vector3(4.73983f, 4.43892f, -0.727256f));
  points.push_back(Vector3(-5.2739f, -6.74123f, -1.28819f));
  points.push_back(Vector3(1.99988f, -5.21226f, -6.30848f));
  points.push_back(Vector3(-0.497146f, 1.52074f, 6.05091f));
  points.push_back(Vector3(3.90728f, -3.58013f, 9.37498f));
  points.push_back(Vector3(2.81045f, 4.2674f, 8.6169f));
  points.push_back(Vector3(-2.63466f, -8.14936f, -2.17139f));
  points.push_back(Vector3(6.70156f, 4.34309f, -5.39415f));
  points.push_back(Vector3(-3.64299f, -2.81716f, -4.63912f));
  points.push_back(Vector3(-7.83196f, -2.2074f, -7.42119f));
  points.push_back(Vector3(3.85418f, 1.28086f, -9.21751f));
  points.push_back(Vector3(0.315866f, -2.5663f, -0.572222f));
  points.push_back(Vector3(-8.08832f, -6.45802f, -2.04505f));
  points.push_back(Vector3(7.14835f, -8.75057f, -3.02103f));
  points.push_back(Vector3(-3.54228f, 5.70666f, -8.15363f));
  points.push_back(Vector3(-9.06247f, -6.3805f, -1.76061f));
  points.push_back(Vector3(-1.2009f, 6.62587f, 6.33778f));
  points.push_back(Vector3(-7.50175f, 3.5081f, -4.13739f));
  points.push_back(Vector3(-8.92514f, -6.6747f, -1.23875f));
  points.push_back(Vector3(8.34224f, 4.58235f, 2.97769f));
  points.push_back(Vector3(-9.74059f, -0.448927f, -7.36869f));
  points.push_back(Vector3(-2.57179f, 0.210882f, 2.48512f));
  points.push_back(Vector3(7.38456f, -3.23099f, 8.623f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.30723f, -1.02512f, 6.51357f));
  points.push_back(Vector3(6.34693f, -9.30174f, 2.48695f));
  points.push_back(Vector3(-6.59841f, 9.38597f, 2.54433f));
  points.push_back(Vector3(7.85028f, 3.24076f, -3.81573f));
  points.push_back(Vector3(-9.10337f, 3.4611f, -6.33778f));
  points.push_back(Vector3(-4.61959f, -4.75692f, 4.65377f));
  points.push_back(Vector3(7.26981f, 3.35551f, 3.87371f));
  points.push_back(Vector3(4.26679f, 2.43019f, 4.35285f));
  points.push_back(Vector3(-3.98358f, -1.27842f, -2.45521f));
  points.push_back(Vector3(-9.83032f, 5.2739f, -9.76806f));
  points.push_back(Vector3(6.92129f, 5.0444f, 7.4633f));
  points.push_back(Vector3(-4.22956f, -3.62957f, -8.64559f));
  points.push_back(Vector3(6.00085f, 3.41777f, -3.34147f));
  points.push_back(Vector3(1.77648f, -2.05176f, 6.72353f));
  points.push_back(Vector3(2.93008f, -6.49892f, -6.14856f));
  points.push_back(Vector3(-2.3075f, -1.76183f, -9.84741f));
  points.push_back(Vector3(-5.51012f, 9.03989f, 9.29869f));
  points.push_back(Vector3(7.02445f, -2.33558f, 6.31336f));
  points.push_back(Vector3(-7.40593f, 5.26109f, -1.84729f));
  points.push_back(Vector3(0.541704f, -1.04038f, -9.38902f));
  points.push_back(Vector3(-6.04785f, 2.0011f, -5.23545f));
  points.push_back(Vector3(2.37709f, 1.93701f, 5.74572f));
  points.push_back(Vector3(7.82342f, 8.17804f, -3.95001f));
  points.push_back(Vector3(9.52147f, -3.84259f, -9.83215f));
  points.push_back(Vector3(-8.83114f, -5.31541f, 7.73675f));
  points.push_back(Vector3(-1.68126f, 5.74999f, -9.2407f));
  points.push_back(Vector3(9.31639f, -3.60454f, 6.39393f));
  points.push_back(Vector3(-5.75671f, -7.49565f, -3.73882f));
  points.push_back(Vector3(7.4456f, 2.30384f, -4.81613f));
  points.push_back(Vector3(6.11805f, 8.76278f, -8.01996f));
  points.push_back(Vector3(6.40248f, 8.32209f, 4.76974f));
  points.push_back(Vector3(5.51805f, -7.12821f, -7.60552f));
  points.push_back(Vector3(7.92718f, -7.54082f, -4.80819f));
  points.push_back(Vector3(-1.14658f, -0.261543f, 9.12107f));
  points.push_back(Vector3(-8.07733f, -0.511795f, -7.57439f));
  points.push_back(Vector3(6.07654f, -0.362255f, 9.70458f));
  points.push_back(Vector3(-5.61144f, 4.26862f, 7.07694f));
  points.push_back(Vector3(-1.43223f, 2.57973f, -9.61242f));
  points.push_back(Vector3(-8.47774f, -7.83441f, -5.07981f));
  points.push_back(Vector3(-7.87408f, -4.46333f, -4.08673f));
  points.push_back(Vector3(5.87817f, 3.94086f, -7.83746f));
  points.push_back(Vector3(-8.56929f, -1.66417f, 3.95123f));
  points.push_back(Vector3(-3.26334f, -6.38295f, -5.12803f));
  points.push_back(Vector3(-7.4041f, -6.57643f, 3.54289f));
  points.push_back(Vector3(1.58239f, -6.33168f, -5.39659f));
  points.push_back(Vector3(2.78787f, 7.56462f, 8.36299f));
  points.push_back(Vector3(1.23631f, 4.85031f, -6.80898f));
  points.push_back(Vector3(2.95022f, 2.32521f, 9.2523f));
  points.push_back(Vector3(-9.70031f, 0.719931f, 1.61107f));
  points.push_back(Vector3(-6.03931f, 7.41874f, -1.20701f));
  points.push_back(Vector3(9.61486f, -8.23908f, 3.15714f));
  points.push_back(Vector3(0.450759f, 9.15586f, 6.19373f));
  points.push_back(Vector3(0.886563f, 3.596f, -7.52373f));
  points.push_back(Vector3(-7.3101f, 7.83685f, -4.03363f));
  points.push_back(Vector3(-1.91076f, 0.531328f, -8.27631f));
  points.push_back(Vector3(-2.26112f, 5.47472f, 1.24363f));
  points.push_back(Vector3(-1.94617f, 2.13477f, -4.24177f));
  points.push_back(Vector3(-0.925626f, 1.20518f, -5.33799f));
  points.push_back(Vector3(-3.5081f, 9.15708f, -7.57012f));
  points.push_back(Vector3(1.61168f, -1.34678f, -1.00436f));
  points.push_back(Vector3(-2.10547f, 8.44172f, 4.00372f));
  points.push_back(Vector3(-1.09897f, 9.88098f, -1.33579f));
  points.push_back(Vector3(-9.07895f, -3.01065f, 4.28205f));
  points.push_back(Vector3(-6.13025f, -3.16507f, 4.38459f));
  points.push_back(Vector3(7.50786f, 5.50401f, 5.3856f));
  points.push_back(Vector3(6.23035f, 5.92883f, -0.968353f));
  points.push_back(Vector3(-3.56548f, 4.15754f, 0.989105f));
  points.push_back(Vector3(1.10935f, 7.05863f, 5.81896f));
  points.push_back(Vector3(3.10343f, -9.61547f, 1.63427f));
  points.push_back(Vector3(7.11478f, 7.47124f, 2.5602f));
  points.push_back(Vector3(-0.842006f, 2.7427f, 4.9266f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.08994f, -5.95202f, -5.01328f));
  points.push_back(Vector3(7.23868f, -3.09061f, 3.726f));
  points.push_back(Vector3(0.467849f, -5.36119f, 1.03305f));
  points.push_back(Vector3(-1.17038f, -6.59963f, -0.885342f));
  points.push_back(Vector3(3.75896f, 4.26069f, -9.93103f));
  points.push_back(Vector3(8.85495f, -0.600299f, -9.54772f));
  points.push_back(Vector3(9.06491f, 8.41243f, 3.32804f));
  points.push_back(Vector3(-5.81286f, 1.34129f, -8.97824f));
  points.push_back(Vector3(7.528f, -5.5565f, -4.83322f));
  points.push_back(Vector3(6.27003f, -9.9115f, 6.41652f));
  points.push_back(Vector3(6.56911f, -8.764f, -5.72863f));
  points.push_back(Vector3(6.81936f, -4.07697f, 3.24381f));
  points.push_back(Vector3(2.76589f, -6.39454f, 1.29307f));
  points.push_back(Vector3(2.32948f, 2.77444f, -6.77358f));
  points.push_back(Vector3(9.15586f, 1.5714f, 7.53594f));
  points.push_back(Vector3(-1.44688f, -6.86453f, -6.61306f));
  points.push_back(Vector3(9.92248f, -9.41343f, 1.36998f));
  points.push_back(Vector3(9.93957f, -6.26881f, 7.08609f));
  points.push_back(Vector3(3.13089f, 5.7622f, -1.70324f));
  points.push_back(Vector3(-8.76888f, -4.66659f, 0.745567f));
  points.push_back(Vector3(2.04627f, 3.45256f, -0.450148f));
  points.push_back(Vector3(-8.40938f, 0.628986f, 9.54649f));
  points.push_back(Vector3(-9.84497f, 9.86938f, 5.23789f));
  points.push_back(Vector3(8.211f, 4.23322f, -5.1915f));
  points.push_back(Vector3(-3.24808f, -8.88852f, -6.13819f));
  points.push_back(Vector3(-1.06784f, 1.32725f, -1.37852f));
  points.push_back(Vector3(-8.58699f, -5.3325f, 3.05338f));
  points.push_back(Vector3(4.34126f, 8.47713f, 0.0674462f));
  points.push_back(Vector3(-0.693075f, -8.46248f, -0.951262f));
  points.push_back(Vector3(-8.2519f, -5.57115f, -5.37645f));
  points.push_back(Vector3(7.5988f, 3.18461f, -2.65847f));
  points.push_back(Vector3(8.79574f, -7.35282f, -3.52153f));
  points.push_back(Vector3(-1.26194f, 0.965301f, 6.47755f));
  points.push_back(Vector3(5.7329f, -6.53066f, 5.10361f));
  points.push_back(Vector3(1.59703f, 0.148015f, 3.75164f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.61425f, 8.6877f, 7.38639f));
  points.push_back(Vector3(4.53658f, 1.1887f, -6.60268f));
  points.push_back(Vector3(0.84933f, 4.5323f, -9.44884f));
  points.push_back(Vector3(5.7329f, -2.85745f, -7.95587f));
  points.push_back(Vector3(9.97253f, 8.20368f, 7.72942f));
  points.push_back(Vector3(5.70177f, -5.4912f, -7.20939f));
  points.push_back(Vector3(-4.80697f, -7.82586f, -9.1174f));
  points.push_back(Vector3(9.14975f, -8.8757f, 5.02182f));
  points.push_back(Vector3(3.42265f, -0.738853f, -5.33372f));
  points.push_back(Vector3(1.55248f, 5.99414f, -5.8446f));
  points.push_back(Vector3(4.85031f, -1.68615f, -1.97241f));
  points.push_back(Vector3(-1.67455f, 9.84741f, -4.15876f));
  points.push_back(Vector3(-7.2277f, -2.53578f, 6.25843f));
  points.push_back(Vector3(-6.71194f, -4.09955f, -4.85092f));
  points.push_back(Vector3(-8.4344f, 3.56731f, 2.59255f));
  points.push_back(Vector3(-5.90808f, 2.87881f, 9.61974f));
  points.push_back(Vector3(-8.78109f, 7.58782f, 5.08774f));
  points.push_back(Vector3(-7.55303f, 1.75329f, -3.21879f));
  points.push_back(Vector3(1.56102f, -8.4753f, -2.96854f));
  points.push_back(Vector3(-0.615559f, -9.49461f, 8.22077f));
  points.push_back(Vector3(-5.00595f, 5.45518f, 0.113834f));
  points.push_back(Vector3(-7.15445f, -9.97742f, 5.88f));
  points.push_back(Vector3(5.62914f, 5.97278f, -8.49055f));
  points.push_back(Vector3(9.17051f, 1.44566f, -5.3795f));
  points.push_back(Vector3(-2.22327f, -7.78985f, -8.62362f));
  points.push_back(Vector3(-6.03442f, -6.53371f, 0.569781f));
  points.push_back(Vector3(-9.7943f, -4.83261f, 8.54427f));
  points.push_back(Vector3(4.0141f, -0.336619f, 9.96704f));
  points.push_back(Vector3(7.01651f, 2.83548f, -8.07428f));
  points.push_back(Vector3(-7.34123f, 3.50871f, -9.36949f));
  points.push_back(Vector3(3.79559f, -9.92431f, -4.185f));
  points.push_back(Vector3(9.14365f, -5.34227f, 0.202948f));
  points.push_back(Vector3(-8.44844f, 0.808436f, -8.64193f));
  points.push_back(Vector3(-9.94873f, -2.96426f, -5.38621f));
  points.push_back(Vector3(-6.88345f, -6.58193f, 6.51723f));
  points.push_back(Vector3(-9.09543f, -0.393384f, -0.856044f));
  points.push_back(Vector3(-0.115055f, 0.76632f, -4.67818f));
  points.push_back(Vector3(-9.48485f, 6.97867f, 5.59069f));
  points.push_back(Vector3(-6.64174f, 6.41835f, 6.61977f));
  points.push_back(Vector3(-7.52678f, -8.26472f, 4.87289f));
  points.push_back(Vector3(-1.49693f, -8.94833f, 5.38072f));
  points.push_back(Vector3(-0.415967f, -3.94024f, 7.41264f));
  points.push_back(Vector3(6.9042f, -9.88464f, -1.57994f));
  points.push_back(Vector3(0.694907f, -0.715049f, -3.67351f));
  points.push_back(Vector3(9.60875f, -1.98218f, -6.40004f));
  points.push_back(Vector3(-2.3075f, 3.0839f, -6.65273f));
  points.push_back(Vector3(-5.08591f, 8.13044f, -0.237129f));
  points.push_back(Vector3(5.93738f, 9.46104f, 6.9573f));
  points.push_back(Vector3(-2.36122f, 3.96344f, -3.76385f));
  points.push_back(Vector3(-3.39091f, 1.13254f, 8.9819f));
  points.push_back(Vector3(-3.86883f, 7.74224f, 8.28913f));
  points.push_back(Vector3(6.82547f, 1.73131f, 2.33619f));
  points.push_back(Vector3(3.9555f, 0.579547f, 0.901212f));
  points.push_back(Vector3(-4.55977f, -2.94046f, -8.75362f));
  points.push_back(Vector3(-2.71462f, 6.16871f, -9.88891f));
  points.push_back(Vector3(1.73193f, 0.849941f, -1.4182f));
  points.push_back(Vector3(-0.936003f, 9.77721f, -8.41914f));
  points.push_back(Vector3(3.07779f, -8.36055f, 5.4619f));
  points.push_back(Vector3(9.48424f, -1.0886f, 2.27027f));
  points.push_back(Vector3(-1.09775f, -6.0155f, 8.47652f));
  points.push_back(Vector3(-8.24702f, -6.23035f, 5.87939f));
  points.push_back(Vector3(3.9671f, 9.76623f, -7.74834f));
  points.push_back(Vector3(-5.05905f, -2.95145f, 8.35505f));
  points.push_back(Vector3(7.08792f, 6.3213f, -7.84051f));
  points.push_back(Vector3(4.52437f, 6.29688f, 8.86166f));
  points.push_back(Vector3(-5.92517f, 0.0546277f, 6.22913f));
  points.push_back(Vector3(-1.55492f, -9.65148f, -3.3311f));
  points.push_back(Vector3(-1.44261f, -0.48555f, -0.735801f));
  points.push_back(Vector3(-5.31907f, 6.845f, -2.12378f));
  points.push_back(Vector3(3.92193f, -6.76077f, -1.69652f));
  points.push_back(Vector3(7.91864f, 3.0784f, 0.301828f));
  points.push_back(Vector3(6.37562f, 7.37236f, -0.627155f));
  points.push_back(Vector3(-9.54161f, 9.7589f, 2.7311f));
  points.push_back(Vector3(-4.43892f, -1.26438f, 3.50383f));
  points.push_back(Vector3(-1.53722f, -0.621052f, 8.49666f));
  points.push_back(Vector3(7.80389f, -6.8627f, 7.66839f));
  points.push_back(Vector3(1.57018f, -9.89196f, -4.41023f));
  points.push_back(Vector3(-9.61974f, 8.73592f, 6.17908f));
  points.push_back(Vector3(4.24665f, 5.09445f, 8.67672f));
  points.push_back(Vector3(-4.28327f, 2.74758f, 2.65786f));
  points.push_back(Vector3(-6.46046f, -1.8131f, -5.02426f));
  points.push_back(Vector3(-5.40086f, 9.22483f, 4.45906f));
  points.push_back(Vector3(-7.37419f, 2.90933f, 8.51619f));
  points.push_back(Vector3(9.89807f, -1.55553f, 2.83975f));
  points.push_back(Vector3(5.20005f, 4.5201f, -1.75146f));
  points.push_back(Vector3(-0.669271f, 7.28019f, 5.85009f));
  points.push_back(Vector3(4.86618f, 7.69524f, -2.58217f));
  points.push_back(Vector3(-9.90722f, 7.11356f, -0.809045f));
  points.push_back(Vector3(-1.27293f, 7.25456f, 6.93472f));
  points.push_back(Vector3(2.0951f, -5.95874f, -4.16791f));
  points.push_back(Vector3(-2.1659f, -1.08615f, -3.20963f));
  points.push_back(Vector3(2.69814f, 7.71966f, -9.65087f));
  points.push_back(Vector3(1.59337f, 3.80718f, -0.71749f));
  points.push_back(Vector3(6.65273f, -7.28813f, -8.65474f));
  points.push_back(Vector3(-8.94772f, -3.02408f, -1.5244f));
  points.push_back(Vector3(2.9545f, -7.77398f, -2.62978f));
  points.push_back(Vector3(9.80773f, -1.35655f, -5.57787f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.20673f, 0.263985f, 5.20432f));
  points.push_back(Vector3(-6.38478f, 5.82385f, 5.45763f));
  points.push_back(Vector3(2.0304f, 3.14554f, 5.53148f));
  points.push_back(Vector3(-3.52031f, -6.59169f, -8.9758f));
  points.push_back(Vector3(9.79553f, -1.84912f, -9.48668f));
  points.push_back(Vector3(-3.17301f, -9.39818f, -7.62932f));
  points.push_back(Vector3(4.06476f, 9.14792f, 9.9176f));
  points.push_back(Vector3(0.0125122f, 1.13681f, 7.669f));
  points.push_back(Vector3(-4.03668f, 4.58052f, -3.30607f));
  points.push_back(Vector3(1.05991f, -7.95648f, -1.75268f));
  points.push_back(Vector3(-2.01819f, 6.31031f, 1.02451f));
  points.push_back(Vector3(0.89816f, 9.88403f, 5.14206f));
  points.push_back(Vector3(-1.3889f, -7.19108f, -0.606403f));
  points.push_back(Vector3(3.11014f, -1.46458f, -8.41487f));
  points.push_back(Vector3(7.30155f, 7.03177f, 2.78237f));
  points.push_back(Vector3(-6.02405f, 5.24155f, 6.34999f));
  points.push_back(Vector3(-0.0186163f, -6.79495f, 6.2096f));
  points.push_back(Vector3(-5.9209f, 5.08591f, -7.57195f));
  points.push_back(Vector3(6.68691f, -7.06351f, 3.46965f));
  points.push_back(Vector3(4.95102f, 8.48506f, 2.29957f));
  points.push_back(Vector3(-7.44377f, 1.82775f, -1.55187f));
  points.push_back(Vector3(-8.39473f, 4.12214f, -2.52846f));
  points.push_back(Vector3(-5.87146f, -4.17585f, 0.375683f));
  points.push_back(Vector3(-6.10584f, -6.43422f, -1.37669f));
  points.push_back(Vector3(6.96524f, -5.79638f, -2.71157f));
  points.push_back(Vector3(-1.14414f, -3.98724f, 5.98682f));
  points.push_back(Vector3(0.671102f, 5.99292f, 2.32093f));
  points.push_back(Vector3(5.19639f, 4.76791f, 0.122379f));
  points.push_back(Vector3(5.86535f, 1.1771f, 3.74615f));
  points.push_back(Vector3(2.15369f, 3.69793f, -6.55873f));
  points.push_back(Vector3(-2.95389f, 9.82788f, 4.39863f));
  points.push_back(Vector3(0.83346f, 4.39436f, 5.18662f));
  points.push_back(Vector3(8.1811f, -6.29994f, -8.20856f));
  points.push_back(Vector3(9.1052f, 5.79028f, 9.69359f));
  points.push_back(Vector3(6.60451f, 8.51802f, -0.679037f));
  points.push_back(Vector3(3.8316f, 9.87976f, -8.0285f));
  points.push_back(Vector3(-1.93518f, -6.55568f, 4.01471f));
  points.push_back(Vector3(-0.880459f, -7.47124f, 8.62239f));
  points.push_back(Vector3(3.90423f, -0.217597f, -0.260323f));
  points.push_back(Vector3(8.80306f, 9.68261f, 8.21528f));
  points.push_back(Vector3(2.96304f, -3.88775f, 9.19981f));
  points.push_back(Vector3(8.18049f, -1.03793f, 6.09058f));
  points.push_back(Vector3(8.34529f, -5.51927f, -4.07941f));
  points.push_back(Vector3(6.42445f, -2.65053f, 6.55141f));
  points.push_back(Vector3(-4.84787f, 4.31318f, -8.81283f));
  points.push_back(Vector3(-0.248726f, -0.270699f, -6.90847f));
  points.push_back(Vector3(-7.7929f, 9.97742f, 3.1431f));
  points.push_back(Vector3(-1.26194f, -6.89016f, -2.78054f));
  points.push_back(Vector3(1.13681f, -3.1138f, 8.40938f));
  points.push_back(Vector3(3.68877f, -0.179754f, 0.974456f));
  points.push_back(Vector3(-7.51091f, -7.85882f, 3.20231f));
  points.push_back(Vector3(-7.85211f, -7.60857f, 0.0192261f));
  points.push_back(Vector3(-6.73757f, -8.45149f, -6.5508f));
  points.push_back(Vector3(-4.96017f, 5.68651f, 5.61022f));
  points.push_back(Vector3(5.90503f, 7.85638f, -5.61449f));
  points.push_back(Vector3(4.98459f, -9.37742f, 7.40715f));
  points.push_back(Vector3(7.94122f, 7.73064f, -7.9693f));
  points.push_back(Vector3(-5.40452f, -8.92331f, 7.38029f));
  points.push_back(Vector3(-3.58257f, -0.0930816f, -5.6975f));
  points.push_back(Vector3(-3.48308f, 3.69793f, 0.596637f));
  points.push_back(Vector3(4.84237f, 6.4861f, -9.21506f));
  points.push_back(Vector3(3.0784f, 1.98035f, -6.14246f));
  points.push_back(Vector3(-5.30686f, 6.13025f, 9.94324f));
  points.push_back(Vector3(-5.71642f, -6.51784f, -9.45921f));
  points.push_back(Vector3(-8.55281f, -5.65172f, -6.81387f));
  points.push_back(Vector3(-9.16379f, -4.96323f, 2.06519f));
  points.push_back(Vector3(-1.16001f, -5.30686f, -0.66805f));
  points.push_back(Vector3(6.62038f, -2.26478f, 5.67675f));
  points.push_back(Vector3(7.32047f, 8.00836f, -6.97439f));
  points.push_back(Vector3(-5.0792f, -6.19251f, 7.16788f));
  points.push_back(Vector3(-4.19111f, -2.61513f, -0.833461f));
  points.push_back(Vector3(-2.74453f, -6.18763f, -1.70324f));
  points.push_back(Vector3(-7.51518f, 6.67715f, 4.80148f));
  points.push_back(Vector3(5.42406f, -2.7311f, -5.50462f));
  points.push_back(Vector3(-7.1569f, -7.21549f, 2.79885f));
  points.push_back(Vector3(8.85556f, -4.4554f, -8.56563f));
  points.push_back(Vector3(-5.72069f, 9.03561f, 0.394604f));
  points.push_back(Vector3(-7.05741f, 4.90341f, 1.75817f));
  points.push_back(Vector3(-1.67211f, 1.19297f, 2.95145f));
  points.push_back(Vector3(-7.61589f, -7.35466f, -1.87353f));
  points.push_back(Vector3(-2.93985f, 1.61595f, -8.59066f));
  points.push_back(Vector3(-0.940275f, -7.22282f, 8.3282f));
  points.push_back(Vector3(1.44017f, 3.19254f, 3.52947f));
  points.push_back(Vector3(9.1644f, -7.96319f, -5.06272f));
  points.push_back(Vector3(9.13205f, -4.49141f, 6.55629f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.17072f, -1.65075f, 1.4658f));
  points.push_back(Vector3(-2.01697f, -0.857265f, 8.59798f));
  points.push_back(Vector3(-3.77544f, -9.01486f, 6.9335f));
  points.push_back(Vector3(8.93551f, -8.2165f, -8.49178f));
  points.push_back(Vector3(-6.97989f, -0.0656146f, -5.67614f));
  points.push_back(Vector3(7.30949f, 5.23362f, -5.20371f));
  points.push_back(Vector3(4.17768f, 7.59758f, -4.66842f));
  points.push_back(Vector3(-9.90905f, 6.09424f, 4.73373f));
  points.push_back(Vector3(-3.60332f, 6.07593f, -9.20347f));
  points.push_back(Vector3(2.06702f, 1.82958f, 2.20435f));
  points.push_back(Vector3(-9.04904f, -1.47435f, -5.92517f));
  points.push_back(Vector3(3.45683f, 4.67269f, -9.40916f));
  points.push_back(Vector3(1.19602f, 3.11563f, 9.70031f));
  points.push_back(Vector3(2.28431f, 9.42442f, -2.38685f));
  points.push_back(Vector3(1.35716f, 3.77728f, 9.81323f));
  points.push_back(Vector3(-0.927457f, -2.57363f, 9.27f));
  points.push_back(Vector3(7.11722f, 1.16733f, 3.06558f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.68343f, -7.64946f, -0.714439f));
  points.push_back(Vector3(3.40312f, 1.19541f, -7.35038f));
  points.push_back(Vector3(8.0047f, -9.8413f, -3.08084f));
  points.push_back(Vector3(-6.67715f, -5.01572f, 4.59944f));
  points.push_back(Vector3(0.308542f, 4.47188f, -6.74612f));
  points.push_back(Vector3(7.65069f, -1.44627f, 4.60921f));
  points.push_back(Vector3(-2.9899f, -2.49062f, 4.69222f));
  points.push_back(Vector3(1.74596f, -9.13694f, 4.42183f));
  points.push_back(Vector3(5.76952f, -8.60836f, 8.33125f));
  points.push_back(Vector3(-2.31483f, -5.67003f, 4.59578f));
  points.push_back(Vector3(0.22187f, 3.00882f, 8.84884f));
  points.push_back(Vector3(9.14914f, 3.5258f, -1.66112f));
  points.push_back(Vector3(-1.26926f, -2.92337f, -8.56624f));
  points.push_back(Vector3(6.30116f, -0.48677f, 1.54515f));
  points.push_back(Vector3(-4.88144f, -8.64559f, -6.12049f));
  points.push_back(Vector3(-3.68999f, -2.10242f, -0.191962f));
  points.push_back(Vector3(4.1258f, 0.821863f, -7.59819f));
  points.push_back(Vector3(-9.19309f, -8.30866f, 4.12885f));
  points.push_back(Vector3(1.38218f, -3.57524f, 9.39268f));
  points.push_back(Vector3(-9.31333f, -7.2338f, -1.02206f));
  points.push_back(Vector3(1.96509f, 1.21433f, 9.60692f));
  points.push_back(Vector3(3.69671f, 0.56734f, -8.29402f));
  points.push_back(Vector3(5.52904f, -5.13169f, 6.83645f));
  points.push_back(Vector3(3.74004f, 6.31886f, -6.41469f));
  points.push_back(Vector3(-7.84539f, 9.19492f, 8.26167f));
  points.push_back(Vector3(5.21775f, 7.72881f, 2.50954f));
  points.push_back(Vector3(-6.12964f, -5.26414f, -8.88729f));
  points.push_back(Vector3(3.60637f, -4.38215f, -1.98401f));
  points.push_back(Vector3(4.81002f, 4.60006f, 8.30012f));
  points.push_back(Vector3(7.23991f, -7.44255f, 3.42509f));
  points.push_back(Vector3(0.662557f, -1.47679f, 8.94345f));
  points.push_back(Vector3(-2.50099f, 6.90542f, -0.0985748f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.64415f, -9.7174f, 3.83465f));
  points.push_back(Vector3(4.23017f, 6.75649f, -2.55287f));
  points.push_back(Vector3(-7.64641f, -8.66817f, -0.616169f));
  points.push_back(Vector3(2.02002f, -1.50304f, -8.35933f));
  points.push_back(Vector3(7.28507f, -3.28288f, 4.55367f));
  points.push_back(Vector3(6.96707f, 8.98312f, 4.1026f));
  points.push_back(Vector3(-1.56774f, -3.20048f, 6.37074f));
  points.push_back(Vector3(7.41752f, -6.34083f, 0.801111f));
  points.push_back(Vector3(9.72838f, -1.98523f, -0.419629f));
  points.push_back(Vector3(-9.28709f, -5.49425f, 7.52495f));
  points.push_back(Vector3(-2.62429f, 4.80148f, -9.51476f));
  points.push_back(Vector3(2.85989f, 7.52373f, 7.96686f));
  points.push_back(Vector3(-6.04907f, -7.79107f, 4.33638f));
  points.push_back(Vector3(-9.35728f, 3.49345f, 0.126041f));
  points.push_back(Vector3(3.18644f, 9.61242f, -9.93347f));
  points.push_back(Vector3(-0.154729f, -3.94208f, 3.85296f));
  points.push_back(Vector3(1.93945f, 0.952482f, 1.04099f));
  points.push_back(Vector3(8.56014f, -4.8619f, 0.528276f));
  points.push_back(Vector3(1.39561f, 0.447096f, -9.96094f));
  points.push_back(Vector3(-4.10504f, -0.188299f, -2.53212f));
  points.push_back(Vector3(4.70626f, 7.09037f, 3.49223f));
  points.push_back(Vector3(6.07349f, 5.61571f, 7.19169f));
  points.push_back(Vector3(5.67553f, -0.86581f, 6.39271f));
  points.push_back(Vector3(8.6169f, -0.76632f, -2.34779f));
  points.push_back(Vector3(-4.8265f, 3.52824f, 7.96503f));
  points.push_back(Vector3(-0.614337f, 1.23142f, 6.46046f));
  points.push_back(Vector3(-8.79818f, 7.51579f, 1.48106f));
  points.push_back(Vector3(-2.69692f, 7.37358f, 2.47108f));
  points.push_back(Vector3(-2.50587f, -7.24235f, -7.87225f));
  points.push_back(Vector3(-9.95117f, 3.64666f, 4.39558f));
  points.push_back(Vector3(-3.34147f, 8.94833f, -2.97891f));
  points.push_back(Vector3(-8.55586f, 1.84484f, -9.16684f));
  points.push_back(Vector3(9.69726f, 6.4275f, -6.15833f));
  points.push_back(Vector3(-3.92254f, -5.95508f, 1.77221f));
  points.push_back(Vector3(-3.26029f, -1.7185f, 1.65563f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.04172f, -8.50459f, -3.3311f));
  points.push_back(Vector3(-0.27131f, 8.00226f, 1.24607f));
  points.push_back(Vector3(6.06067f, -8.24152f, 2.902f));
  points.push_back(Vector3(8.7701f, -8.76644f, -0.24018f));
  points.push_back(Vector3(8.05902f, 6.63869f, -4.85031f));
  points.push_back(Vector3(2.83059f, -7.75933f, 0.353709f));
  points.push_back(Vector3(-4.11847f, -6.98843f, -9.45311f));
  points.push_back(Vector3(0.959197f, 1.28513f, -6.22547f));
  points.push_back(Vector3(1.43834f, -3.56243f, 7.304f));
  points.push_back(Vector3(7.61284f, -3.53008f, 5.02609f));
  points.push_back(Vector3(-9.47203f, 0.856655f, -1.66051f));
  points.push_back(Vector3(3.56426f, -4.35896f, 3.8432f));
  points.push_back(Vector3(-7.12088f, -5.98621f, 8.36848f));
  points.push_back(Vector3(-5.4387f, -9.37437f, 1.26438f));
  points.push_back(Vector3(-5.43687f, 3.89874f, 3.57646f));
  points.push_back(Vector3(5.34837f, -3.95123f, -9.38719f));
  points.push_back(Vector3(-7.23075f, 1.20273f, 3.84442f));
  points.push_back(Vector3(4.15876f, -7.68548f, 9.09116f));
  points.push_back(Vector3(-0.604572f, 3.48735f, 5.48448f));
  points.push_back(Vector3(8.81466f, 4.44319f, 9.94995f));
  points.push_back(Vector3(-1.49266f, 0.835901f, -9.85534f));
  points.push_back(Vector3(2.42653f, 7.68059f, 7.44804f));
  points.push_back(Vector3(6.08203f, -7.18986f, 5.42589f));
  points.push_back(Vector3(-2.90384f, -0.227363f, 9.95911f));
  points.push_back(Vector3(0.917081f, 5.00412f, 7.32108f));
  points.push_back(Vector3(-9.74059f, 0.903653f, -6.80349f));
  points.push_back(Vector3(5.82018f, 2.91116f, 6.35426f));
  points.push_back(Vector3(3.0784f, 8.14081f, 8.06146f));
  points.push_back(Vector3(-9.45738f, 3.66314f, -9.27488f));
  points.push_back(Vector3(-4.07636f, -5.79089f, 8.60103f));
  points.push_back(Vector3(-4.98215f, 4.98886f, 3.41227f));
  points.push_back(Vector3(5.11399f, -6.85171f, 7.46941f));
  points.push_back(Vector3(5.70544f, 7.97601f, -1.49693f));
  points.push_back(Vector3(0.0918603f, -9.74548f, 1.65014f));
  points.push_back(Vector3(-8.19941f, 8.36665f, 7.52983f));
  points.push_back(Vector3(3.72356f, 3.30119f, 2.46071f));
  points.push_back(Vector3(-5.81286f, -7.25272f, 1.77343f));
  points.push_back(Vector3(-7.20511f, -1.2186f, -6.37806f));
  points.push_back(Vector3(-9.15403f, 3.49406f, -6.09241f));
  points.push_back(Vector3(3.64421f, 3.36528f, -8.18781f));
  points.push_back(Vector3(-9.57091f, -7.65618f, -5.61632f));
  points.push_back(Vector3(-8.33796f, -0.607623f, -1.7655f));
  points.push_back(Vector3(3.41044f, 9.36888f, -2.73782f));
  points.push_back(Vector3(-4.39802f, 6.54347f, 9.77538f));
  points.push_back(Vector3(0.1059f, -5.54247f, 9.10703f));
  points.push_back(Vector3(0.824305f, 2.34352f, 6.78457f));
  points.push_back(Vector3(-0.577105f, -5.25498f, 5.69811f));
  points.push_back(Vector3(2.18909f, -5.65661f, -6.0387f));
  points.push_back(Vector3(-6.03809f, 6.10096f, -2.83792f));
  points.push_back(Vector3(-0.144352f, -0.788903f, 3.63018f));
  points.push_back(Vector3(-1.46825f, -7.49504f, -4.91195f));
  points.push_back(Vector3(2.21839f, 1.02206f, -0.475173f));
  points.push_back(Vector3(2.78176f, -3.79986f, -2.07129f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.29319f, 4.89669f, 2.83792f));
  points.push_back(Vector3(9.15647f, -5.23301f, -0.736412f));
  points.push_back(Vector3(6.69973f, -3.12113f, -6.91092f));
  points.push_back(Vector3(1.583f, 2.60842f, -3.62774f));
  points.push_back(Vector3(5.48448f, -5.84521f, 8.47774f));
  points.push_back(Vector3(9.45738f, 9.82238f, 3.37504f));
  points.push_back(Vector3(-4.9089f, 3.86395f, -7.35466f));
  points.push_back(Vector3(-9.59166f, 6.58071f, -5.67797f));
  points.push_back(Vector3(-1.59276f, 5.97034f, 8.58516f));
  points.push_back(Vector3(0.561845f, 7.96869f, -2.67373f));
  points.push_back(Vector3(-0.257271f, -4.65011f, 6.98355f));
  points.push_back(Vector3(5.65416f, -7.87835f, 5.69628f));
  points.push_back(Vector3(-5.21653f, 1.87048f, -3.77789f));
  points.push_back(Vector3(0.343944f, -5.59496f, -0.932341f));
  points.push_back(Vector3(-3.82611f, 6.6802f, -5.63646f));
  points.push_back(Vector3(9.67528f, -9.24558f, -9.8236f));
  points.push_back(Vector3(2.82083f, 5.46922f, 5.85376f));
  points.push_back(Vector3(-0.494705f, 4.75143f, 8.56502f));
  points.push_back(Vector3(-5.99536f, -9.56969f, 2.74941f));
  points.push_back(Vector3(-8.49666f, 6.61733f, -8.37092f));
  points.push_back(Vector3(-7.91314f, -5.42894f, -5.47838f));
  points.push_back(Vector3(5.53758f, -9.34019f, -1.53356f));
  points.push_back(Vector3(2.46864f, 7.3693f, -5.562f));
  points.push_back(Vector3(0.445875f, -6.8102f, -4.57991f));
  points.push_back(Vector3(-0.949431f, -7.93023f, 6.6625f));
  points.push_back(Vector3(-4.21125f, -1.87353f, 6.67776f));
  points.push_back(Vector3(-9.81567f, -5.36668f, -0.976897f));
  points.push_back(Vector3(-5.2916f, 4.90707f, 1.09348f));
  points.push_back(Vector3(6.55568f, -4.99313f, 4.42305f));
  points.push_back(Vector3(-5.9978f, -4.63485f, 0.223701f));
  points.push_back(Vector3(-8.93368f, -4.48531f, 1.54027f));
  points.push_back(Vector3(4.4438f, 5.51866f, 4.7673f));
  points.push_back(Vector3(-5.87451f, -8.82992f, -4.63363f));
  points.push_back(Vector3(-5.2916f, -2.86599f, 9.78088f));
  points.push_back(Vector3(-1.76794f, 5.21836f, 2.69448f));
  points.push_back(Vector3(-1.41636f, 5.09384f, -4.3614f));
  points.push_back(Vector3(5.95752f, 2.60964f, -0.571001f));
  points.push_back(Vector3(9.84313f, 2.38014f, 4.24238f));
  points.push_back(Vector3(-2.19703f, -3.02713f, -1.97485f));
  points.push_back(Vector3(-5.16221f, 7.528f, 0.961028f));
  points.push_back(Vector3(-6.51418f, 5.43565f, 7.01651f));
  points.push_back(Vector3(1.96936f, -7.92413f, 4.70321f));
  points.push_back(Vector3(4.45662f, 2.86721f, -7.10746f));
  points.push_back(Vector3(2.22816f, 0.430006f, 3.87066f));
  points.push_back(Vector3(-0.600299f, 5.28916f, -9.89624f));
  points.push_back(Vector3(-1.52623f, 4.1319f, 7.22709f));
  points.push_back(Vector3(-9.38597f, -6.20777f, 0.165105f));
  points.push_back(Vector3(-4.45845f, 9.27366f, 8.44783f));
  points.push_back(Vector3(-0.0332654f, -2.15491f, -1.78137f));
  points.push_back(Vector3(-8.88668f, -7.75018f, -6.2157f));
  points.push_back(Vector3(-8.8641f, -7.14713f, -2.5309f));
  points.push_back(Vector3(-8.93002f, 7.6751f, -3.6314f));
  points.push_back(Vector3(-7.43461f, -6.26759f, 1.18503f));
  points.push_back(Vector3(-4.70748f, 5.59923f, 0.726036f));
  points.push_back(Vector3(-4.60128f, 6.63869f, -9.11252f));
  points.push_back(Vector3(3.01187f, 5.68468f, 9.54649f));
  points.push_back(Vector3(5.28123f, -2.75796f, -9.50621f));
  points.push_back(Vector3(-6.5624f, -3.81451f, 4.17097f));
  points.push_back(Vector3(8.62911f, 8.71334f, 1.98279f));
  points.push_back(Vector3(9.91699f, 7.95648f, 3.11014f));
  points.push_back(Vector3(-4.55367f, -1.8009f, 1.19114f));
  points.push_back(Vector3(-3.14798f, -9.2053f, -8.31721f));
  points.push_back(Vector3(-5.62548f, -0.285959f, 5.00595f));
  points.push_back(Vector3(-9.4586f, 3.67718f, 3.12174f));
  points.push_back(Vector3(-5.99353f, 6.37684f, 6.13697f));
  points.push_back(Vector3(-9.58129f, -5.62243f, -0.386059f));
  points.push_back(Vector3(-0.0204474f, -1.57811f, -8.81466f));
  points.push_back(Vector3(-1.50182f, 0.128483f, -6.61061f));
  points.push_back(Vector3(2.57729f, 7.81854f, 4.5323f));
  points.push_back(Vector3(-5.468f, 1.75451f, 5.48082f));
  points.push_back(Vector3(-3.05826f, 8.18232f, 6.95486f));
  points.push_back(Vector3(-4.93942f, 1.75878f, 5.75671f));
  points.push_back(Vector3(0.070498f, -7.54509f, -3.4257f));
  points.push_back(Vector3(8.63216f, 4.5201f, -5.23667f));
  points.push_back(Vector3(-1.96875f, -6.94082f, 3.00455f));
  points.push_back(Vector3(9.5172f, 6.6332f, -1.75634f));
  points.push_back(Vector3(5.77197f, -7.61528f, 6.11927f));
  points.push_back(Vector3(-9.29014f, -9.98535f, -8.51253f));
  points.push_back(Vector3(3.03873f, 8.08588f, 1.02512f));
  points.push_back(Vector3(6.42994f, 2.14393f, -4.5793f));
  points.push_back(Vector3(-4.26923f, 0.187079f, 3.22794f));
  points.push_back(Vector3(-2.82571f, -0.758385f, -8.02973f));
  points.push_back(Vector3(-4.9675f, -2.01209f, -4.97299f));
  points.push_back(Vector3(9.19248f, -8.14447f, -4.62813f));
  points.push_back(Vector3(-0.645466f, 6.65761f, 8.47163f));
  points.push_back(Vector3(-2.56569f, 0.603961f, -8.89218f));
  points.push_back(Vector3(-0.99826f, 8.77133f, -5.90136f));
  points.push_back(Vector3(-0.760216f, -4.64339f, -2.35267f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.26228f, -3.42448f, -0.0918609f));
  points.push_back(Vector3(2.37709f, 9.54955f, -1.64953f));
  points.push_back(Vector3(8.83908f, -6.82791f, 3.35124f));
  points.push_back(Vector3(1.54943f, -4.06354f, -9.72838f));
  points.push_back(Vector3(-8.55525f, -4.62325f, -8.74691f));
  points.push_back(Vector3(0.522171f, -7.71294f, -2.51381f));
  points.push_back(Vector3(-7.22221f, 7.11478f, -5.17197f));
  points.push_back(Vector3(-4.55489f, 4.53536f, 7.79107f));
  points.push_back(Vector3(6.40126f, -4.41267f, -7.46574f));
  points.push_back(Vector3(-0.506912f, 1.2656f, 9.86999f));
  points.push_back(Vector3(6.69607f, 5.08713f, 2.94534f));
  points.push_back(Vector3(1.36265f, 6.87979f, 7.77581f));
  points.push_back(Vector3(3.4788f, -1.00009f, 1.13865f));
  points.push_back(Vector3(8.83969f, -7.29179f, 2.20801f));
  points.push_back(Vector3(-5.67431f, -5.68407f, 0.735191f));
  points.push_back(Vector3(6.04907f, -4.82406f, 6.25355f));
  points.push_back(Vector3(6.65944f, 7.48711f, -0.0106817f));
  points.push_back(Vector3(4.15632f, -3.26151f, 3.42265f));
  points.push_back(Vector3(1.04404f, -7.74285f, 5.08408f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.61608f, 2.12744f, -3.51726f));
  points.push_back(Vector3(9.84924f, -5.10666f, -8.9996f));
  points.push_back(Vector3(-1.05014f, 6.31886f, 0.473952f));
  points.push_back(Vector3(9.48546f, -2.76711f, -7.98273f));
  points.push_back(Vector3(-3.54045f, -1.8955f, -5.81591f));
  points.push_back(Vector3(-4.88083f, -1.85949f, 1.54454f));
  points.push_back(Vector3(-5.92456f, -2.18421f, 3.91522f));
  points.push_back(Vector3(-0.299386f, 1.9602f, 0.740074f));
  points.push_back(Vector3(-5.49058f, -7.48894f, 1.66051f));
  points.push_back(Vector3(-8.50398f, -0.105289f, 1.71361f));
  points.push_back(Vector3(3.01859f, 5.41307f, 9.52513f));
  points.push_back(Vector3(2.62795f, -0.974456f, -3.35063f));
  points.push_back(Vector3(-8.48262f, 9.11435f, 3.46355f));
  points.push_back(Vector3(7.77886f, -3.59355f, 4.81491f));
  points.push_back(Vector3(9.67406f, -4.02997f, -8.90988f));
  points.push_back(Vector3(3.80902f, 1.09104f, 3.90179f));
  points.push_back(Vector3(-1.39988f, -6.18824f, -5.84887f));
  points.push_back(Vector3(-6.47877f, 9.31272f, 5.03098f));
  points.push_back(Vector3(4.21674f, -9.21384f, -0.76754f));
  points.push_back(Vector3(-2.18177f, -4.87411f, -0.496536f));
  points.push_back(Vector3(0.975066f, -7.7691f, 3.00577f));
  points.push_back(Vector3(-7.60552f, 3.06253f, -7.36625f));
  points.push_back(Vector3(-5.03464f, -2.48329f, 3.16874f));
  points.push_back(Vector3(-9.53368f, -6.70156f, 8.15668f));
  points.push_back(Vector3(-9.06369f, -1.70141f, 5.81591f));
  points.push_back(Vector3(-5.67309f, 5.98437f, -1.70446f));
  points.push_back(Vector3(-7.06107f, 3.99091f, 6.74001f));
  points.push_back(Vector3(-1.76672f, -7.82281f, -7.86615f));
  points.push_back(Vector3(-4.2851f, -1.8363f, -9.3939f));
  points.push_back(Vector3(5.81835f, -3.14859f, 0.88168f));
  points.push_back(Vector3(5.98682f, 3.33415f, -9.43785f));
  points.push_back(Vector3(9.29685f, 7.13675f, -4.42183f));
  points.push_back(Vector3(2.0951f, 3.84991f, -5.80981f));
  points.push_back(Vector3(7.7868f, 8.83663f, 3.09122f));
  points.push_back(Vector3(9.3585f, 9.74242f, 2.19275f));
  points.push_back(Vector3(0.547197f, -1.17283f, -7.41813f));
  points.push_back(Vector3(9.66124f, 1.00375f, -3.58562f));
  points.push_back(Vector3(0.333568f, -7.27897f, 9.32615f));
  points.push_back(Vector3(9.60448f, -5.6975f, 6.17603f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.11335f, 8.59859f, -2.33497f));
  points.push_back(Vector3(3.90851f, 1.713f, -1.89856f));
  points.push_back(Vector3(-8.68465f, -0.827967f, -0.808435f));
  points.push_back(Vector3(0.113224f, 9.16623f, -7.71233f));
  points.push_back(Vector3(4.88266f, -7.1923f, 7.13004f));
  points.push_back(Vector3(-5.80309f, -0.082705f, 0.926237f));
  points.push_back(Vector3(-3.41838f, 3.98846f, -2.61147f));
  points.push_back(Vector3(4.10688f, -8.14081f, 8.89401f));
  points.push_back(Vector3(-5.12803f, 1.8186f, 7.96258f));
  points.push_back(Vector3(-6.31031f, -1.29307f, 3.02408f));
  points.push_back(Vector3(4.06842f, -7.35038f, 0.657064f));
  points.push_back(Vector3(-1.31016f, -1.50182f, 7.14896f));
  points.push_back(Vector3(-2.41676f, 5.27573f, -9.25108f));
  points.push_back(Vector3(2.06458f, -5.25498f, -2.67495f));
  points.push_back(Vector3(-3.77605f, 4.08979f, 3.36161f));
  points.push_back(Vector3(-2.71706f, 9.27976f, -4.91562f));
  points.push_back(Vector3(-6.94266f, 0.661336f, 0.22187f));
  points.push_back(Vector3(3.59294f, 9.81262f, 4.68856f));
  points.push_back(Vector3(1.87231f, 7.64824f, 0.0454724f));
  points.push_back(Vector3(-1.44932f, -8.6755f, -8.01019f));
  points.push_back(Vector3(-8.5406f, 3.37077f, 2.52968f));
  points.push_back(Vector3(-7.1746f, -8.73592f, 0.0277722f));
  points.push_back(Vector3(-4.68184f, -6.74978f, -7.58965f));
  points.push_back(Vector3(-7.03787f, -3.50261f, 2.45949f));
  points.push_back(Vector3(8.45515f, 8.07611f, -1.35472f));
  points.push_back(Vector3(1.87292f, 7.68365f, -7.13797f));
  points.push_back(Vector3(-7.78253f, -8.69747f, 7.11966f));
  points.push_back(Vector3(-3.11014f, -3.39091f, -1.72277f));
  points.push_back(Vector3(-2.96548f, 0.755333f, 5.77929f));
  points.push_back(Vector3(3.66375f, -3.40007f, -8.35444f));
  points.push_back(Vector3(-5.92456f, 6.69485f, -8.71273f));
  points.push_back(Vector3(-6.37745f, -2.125f, 2.54372f));
  points.push_back(Vector3(-7.34123f, -9.23826f, -1.3242f));
  points.push_back(Vector3(-4.44319f, -8.83969f, -8.73653f));
  points.push_back(Vector3(3.25297f, 1.27903f, 2.55287f));
  points.push_back(Vector3(-9.33042f, 6.51235f, 0.389721f));
  points.push_back(Vector3(-1.40477f, 2.30567f, -7.33268f));
  points.push_back(Vector3(-2.3191f, 2.61513f, -6.92679f));
  points.push_back(Vector3(0.357372f, 9.19919f, 0.58443f));
  points.push_back(Vector3(-7.27714f, 7.33634f, -9.32493f));
  points.push_back(Vector3(8.33064f, -5.27879f, 0.360423f));
  points.push_back(Vector3(-9.63317f, 5.50096f, -3.14798f));
  points.push_back(Vector3(5.8916f, -0.554521f, -3.08451f));
  points.push_back(Vector3(-0.332957f, 7.24906f, 3.03934f));
  points.push_back(Vector3(-8.07672f, -9.09299f, 3.8316f));
  points.push_back(Vector3(5.07248f, 2.3954f, -5.9624f));
  points.push_back(Vector3(-1.96753f, -9.98535f, 9.96948f));
  points.push_back(Vector3(8.99594f, 5.47777f, -5.32701f));
  points.push_back(Vector3(3.18277f, 3.29936f, -5.79455f));
  points.push_back(Vector3(-6.85965f, -4.8851f, 1.63121f));
  points.push_back(Vector3(8.4228f, 8.04376f, 2.15552f));
  points.push_back(Vector3(-7.97967f, 2.44972f, -8.80367f));
  points.push_back(Vector3(-5.46617f, -3.24503f, 0.488602f));
  points.push_back(Vector3(6.798f, -0.765099f, 6.14185f));
  points.push_back(Vector3(7.83868f, 0.291451f, 5.91662f));
  points.push_back(Vector3(3.24686f, 9.17722f, 6.01184f));
  points.push_back(Vector3(-3.53618f, -2.26417f, -0.459914f));
  points.push_back(Vector3(-3.24931f, 8.80184f, -3.05887f));
  points.push_back(Vector3(3.62163f, 8.35872f, -4.41511f));
  points.push_back(Vector3(-7.72149f, 9.01853f, 2.50832f));
  points.push_back(Vector3(8.4051f, 9.14548f, 9.04416f));
  points.push_back(Vector3(0.752891f, 8.11945f, -1.58727f));
  points.push_back(Vector3(0.739464f, -7.00491f, -8.05353f));
  points.push_back(Vector3(-2.34779f, -6.29566f, -6.50746f));
  points.push_back(Vector3(-2.05786f, 0.210272f, -4.62813f));
  points.push_back(Vector3(7.53349f, -2.52113f, 0.257882f));
  points.push_back(Vector3(8.35627f, 1.89489f, -3.63506f));
  points.push_back(Vector3(-8.05597f, 5.49242f, 2.22266f));
  points.push_back(Vector3(-4.785f, -9.22361f, -1.80151f));
  points.push_back(Vector3(-9.55931f, 9.71007f, -4.65072f));
  points.push_back(Vector3(3.78521f, 5.33494f, 1.35716f));
  points.push_back(Vector3(-2.44484f, 7.32963f, -0.58504f));
  points.push_back(Vector3(-0.610065f, -0.610675f, 3.74981f));
  points.push_back(Vector3(-4.79782f, 4.42305f, 4.4908f));
  points.push_back(Vector3(-4.30158f, -5.95447f, -5.51256f));
  points.push_back(Vector3(2.52358f, -7.09952f, 0.608845f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.91659f, 4.79476f, 3.58501f));
  points.push_back(Vector3(5.01511f, 4.61959f, -7.76299f));
  points.push_back(Vector3(5.74084f, -0.801111f, -2.19214f));
  points.push_back(Vector3(-7.63665f, 5.54674f, 1.4536f));
  points.push_back(Vector3(-0.654011f, 2.88003f, 8.48689f));
  points.push_back(Vector3(9.51781f, 3.45073f, 8.84335f));
  points.push_back(Vector3(6.27674f, -2.41432f, 9.49828f));
  points.push_back(Vector3(-3.76568f, -1.36265f, 3.50139f));
  points.push_back(Vector3(2.9545f, 1.6715f, 6.66311f));
  points.push_back(Vector3(1.13804f, 0.721152f, 8.8757f));
  points.push_back(Vector3(5.9209f, -6.39088f, -5.98254f));
  points.push_back(Vector3(-7.48161f, -2.59926f, -7.08365f));
  points.push_back(Vector3(-8.07245f, 0.0857568f, 4.44136f));
  points.push_back(Vector3(1.67028f, -1.26377f, -7.63726f));
  points.push_back(Vector3(-6.24256f, 2.50832f, 8.18232f));
  points.push_back(Vector3(-6.8157f, 8.69625f, -9.16868f));
  points.push_back(Vector3(7.74468f, 9.0698f, -7.16056f));
  points.push_back(Vector3(-3.51421f, -3.90362f, 9.35423f));
  points.push_back(Vector3(-7.90948f, -9.98474f, 6.85232f));
  points.push_back(Vector3(7.47002f, -6.64357f, -0.890836f));
  points.push_back(Vector3(1.71789f, -6.03198f, 7.95709f));
  points.push_back(Vector3(-9.129f, 6.19129f, 1.93823f));
  points.push_back(Vector3(-2.11158f, 5.53087f, -9.08078f));
  points.push_back(Vector3(-4.57869f, -6.93655f, -2.772f));
  points.push_back(Vector3(-8.14997f, 1.47069f, -2.35023f));
  points.push_back(Vector3(7.49504f, -7.70745f, -3.54411f));
  points.push_back(Vector3(-6.36219f, 7.05008f, 0.0198376f));
  points.push_back(Vector3(7.2631f, -0.242622f, -8.45454f));
  points.push_back(Vector3(2.53639f, -2.00903f, -6.12354f));
  points.push_back(Vector3(6.88467f, -6.36952f, 2.78909f));
  points.push_back(Vector3(-3.00699f, 9.72655f, -5.65172f));
  points.push_back(Vector3(-4.55123f, 6.96341f, -1.17954f));
  points.push_back(Vector3(1.4951f, -3.62224f, 0.0717187f));
  points.push_back(Vector3(2.78359f, 6.81509f, 7.58538f));
  points.push_back(Vector3(8.58516f, -0.0595111f, -0.0534075f));
  points.push_back(Vector3(-3.70647f, -7.59575f, -4.12824f));
  points.push_back(Vector3(-1.69713f, -2.1836f, -7.05435f));
  points.push_back(Vector3(-5.72924f, -3.01798f, -1.68798f));
  points.push_back(Vector3(7.36869f, 6.75466f, 1.84912f));
  points.push_back(Vector3(8.04804f, -8.71578f, -2.76223f));
  points.push_back(Vector3(-1.09897f, 5.79882f, 8.06757f));
  points.push_back(Vector3(6.50868f, 3.05094f, 5.00107f));
  points.push_back(Vector3(-7.11295f, -7.3809f, -0.55269f));
  points.push_back(Vector3(2.28858f, 9.60082f, -7.27348f));
  points.push_back(Vector3(0.458694f, -0.957976f, 8.60836f));
  points.push_back(Vector3(1.99927f, -8.55098f, -3.43669f));
  points.push_back(Vector3(3.82733f, -4.77889f, 8.46431f));
  points.push_back(Vector3(-4.48775f, -3.12113f, 6.71926f));
  points.push_back(Vector3(0.959808f, 0.228584f, 6.63381f));
  points.push_back(Vector3(-5.77319f, 2.69204f, -1.44688f));
  points.push_back(Vector3(-7.77093f, 2.33131f, 3.89752f));
  points.push_back(Vector3(-2.2074f, 7.69646f, -8.89767f));
  points.push_back(Vector3(7.88995f, 7.43461f, -3.69976f));
  points.push_back(Vector3(5.55406f, 5.18906f, 9.24619f));
  points.push_back(Vector3(2.7488f, -8.46248f, 8.32148f));
  points.push_back(Vector3(5.38438f, -8.64009f, -6.27857f));
  points.push_back(Vector3(0.40498f, 5.13657f, 8.34834f));
  points.push_back(Vector3(1.25217f, 0.652791f, 0.917081f));
  points.push_back(Vector3(-3.67901f, -3.25846f, -0.0936919f));
  points.push_back(Vector3(-5.84948f, -0.860317f, -5.79882f));
  points.push_back(Vector3(9.3469f, 1.66051f, 5.25193f));
  points.push_back(Vector3(1.54393f, -4.51888f, 3.1901f));
  points.push_back(Vector3(7.62322f, -6.7217f, 4.55306f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.16135f, -2.12317f, 2.46376f));
  points.push_back(Vector3(-1.49632f, -9.729f, -2.55532f));
  points.push_back(Vector3(-7.92108f, -1.04099f, -1.77099f));
  points.push_back(Vector3(-6.08142f, -3.35002f, -1.85278f));
  points.push_back(Vector3(-2.69143f, 0.693686f, 2.44911f));
  points.push_back(Vector3(7.89117f, -9.86328f, 5.60167f));
  points.push_back(Vector3(8.83908f, -8.4875f, -7.50175f));
  points.push_back(Vector3(-5.97034f, -8.57479f, -3.1077f));
  points.push_back(Vector3(6.66189f, -8.54732f, 6.96158f));
  points.push_back(Vector3(4.2552f, -9.48302f, -1.49754f));
  points.push_back(Vector3(0.630207f, 6.80593f, 8.17011f));
  points.push_back(Vector3(1.70202f, 6.34632f, -7.65557f));
  points.push_back(Vector3(-2.40455f, -7.00674f, 0.679647f));
  points.push_back(Vector3(3.98175f, 0.99704f, 1.17038f));
  points.push_back(Vector3(1.04465f, 8.08039f, -6.52516f));
  points.push_back(Vector3(8.36116f, 8.76583f, 7.54875f));
  points.push_back(Vector3(-4.34919f, -5.33738f, 7.65801f));
  points.push_back(Vector3(8.60958f, 5.8562f, 4.0788f));
  points.push_back(Vector3(-1.26682f, -0.825526f, -2.48451f));
  points.push_back(Vector3(-5.14328f, 1.66906f, 6.94021f));
  points.push_back(Vector3(-2.64504f, 7.37663f, 3.65764f));
  points.push_back(Vector3(-4.66353f, -9.94873f, -7.30583f));
  points.push_back(Vector3(6.26575f, 5.45763f, 4.9559f));
  points.push_back(Vector3(-5.11826f, -9.07651f, 2.0835f));
  points.push_back(Vector3(-9.49583f, -9.72655f, -6.32618f));
  points.push_back(Vector3(-8.05963f, -8.03827f, -3.24137f));
  points.push_back(Vector3(9.80346f, -7.69768f, -1.89856f));
  points.push_back(Vector3(-3.76446f, -1.0184f, -1.92358f));
  points.push_back(Vector3(-7.54631f, 3.35063f, -8.46675f));
  points.push_back(Vector3(5.48509f, -5.24094f, 3.65947f));
  points.push_back(Vector3(6.46962f, 0.477614f, 8.23542f));
  points.push_back(Vector3(-5.68163f, 1.89612f, 9.99268f));
  points.push_back(Vector3(-1.71361f, -9.55077f, 8.43806f));
  points.push_back(Vector3(4.19477f, -1.70751f, -9.71679f));
  points.push_back(Vector3(6.292f, 7.82037f, 8.08466f));
  points.push_back(Vector3(2.22877f, -3.46416f, 8.73592f));
  points.push_back(Vector3(0.744346f, 6.96768f, 7.32414f));
  points.push_back(Vector3(1.8656f, 3.35246f, -2.27332f));
  points.push_back(Vector3(5.58763f, -3.42082f, 7.29728f));
  points.push_back(Vector3(-8.16828f, -5.12864f, 5.69262f));
  points.push_back(Vector3(-6.21021f, -3.32682f, 9.92309f));
  points.push_back(Vector3(4.18928f, -9.8059f, 3.22001f));
  points.push_back(Vector3(8.67061f, -4.20331f, 2.80068f));
  points.push_back(Vector3(-9.81445f, -7.79656f, -4.46516f));
  points.push_back(Vector3(1.67455f, -8.00348f, -6.60939f));
  points.push_back(Vector3(6.37196f, 3.19498f, -6.33839f));
  points.push_back(Vector3(4.75204f, -1.89917f, 5.9209f));
  points.push_back(Vector3(5.69079f, 6.21509f, 6.57399f));
  points.push_back(Vector3(5.39293f, 2.16956f, 3.37993f));
  points.push_back(Vector3(1.40721f, 3.51543f, -4.86801f));
  points.push_back(Vector3(-3.08512f, 0.28962f, -1.87353f));
  points.push_back(Vector3(8.33857f, -6.2859f, -1.9779f));
  points.push_back(Vector3(8.39351f, 3.59661f, 0.431226f));
  points.push_back(Vector3(8.39961f, 1.64647f, 2.7488f));
  points.push_back(Vector3(-9.97131f, 7.07938f, -2.70669f));
  points.push_back(Vector3(-1.55675f, 1.95227f, -6.45619f));
  points.push_back(Vector3(-6.58071f, -3.17606f, 6.57033f));
  points.push_back(Vector3(0.293894f, -8.28669f, 5.25681f));
  points.push_back(Vector3(-5.51073f, 7.39006f, 6.33717f));
  points.push_back(Vector3(8.22077f, 1.38523f, -9.52513f));
  points.push_back(Vector3(-7.63604f, 4.48775f, 8.95871f));
  points.push_back(Vector3(0.395215f, -0.58443f, 8.71761f));
  points.push_back(Vector3(-7.34855f, 2.54616f, 6.11805f));
  points.push_back(Vector3(-9.40916f, -3.24564f, -5.92456f));
  points.push_back(Vector3(0.999482f, -5.39171f, 7.5042f));
  points.push_back(Vector3(7.0568f, 5.46678f, -6.32801f));
  points.push_back(Vector3(4.83261f, -4.232f, 4.46577f));
  points.push_back(Vector3(-0.408033f, 3.36161f, -3.80169f));
  points.push_back(Vector3(0.472732f, 3.62102f, 3.28532f));
  points.push_back(Vector3(-4.09955f, -3.57524f, -9.83825f));
  points.push_back(Vector3(1.75878f, 2.32398f, 3.22306f));
  points.push_back(Vector3(-3.24259f, 5.42039f, 7.75262f));
  points.push_back(Vector3(-2.57302f, 3.49895f, -5.32884f));
  points.push_back(Vector3(-1.04099f, 4.7795f, 4.96994f));
  points.push_back(Vector3(0.338451f, 9.7235f, -4.83749f));
  points.push_back(Vector3(7.85882f, 3.34513f, 4.31868f));
  points.push_back(Vector3(4.63729f, 7.35527f, -1.98706f));
  points.push_back(Vector3(-9.90051f, -9.83398f, 5.46556f));
  points.push_back(Vector3(9.62401f, 7.32414f, -7.32902f));
  points.push_back(Vector3(4.36201f, 6.86514f, -5.68285f));
  points.push_back(Vector3(-5.9801f, -2.09876f, -0.221869f));
  points.push_back(Vector3(-3.88165f, 3.59661f, -5.57176f));
  points.push_back(Vector3(3.84564f, -5.27146f, 0.0570691f));
  points.push_back(Vector3(-6.00391f, 0.832239f, 9.08139f));
  points.push_back(Vector3(3.91461f, -6.87613f, -2.9899f));
  points.push_back(Vector3(2.86843f, 6.56301f, 0.220039f));
  points.push_back(Vector3(9.39329f, 8.34285f, -8.62911f));
  points.push_back(Vector3(-5.2092f, 0.599688f, 4.21918f));
  points.push_back(Vector3(-7.52007f, -6.42872f, -7.02139f));
  points.push_back(Vector3(-4.19599f, 6.90847f, -3.31645f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.00632f, 9.58129f, -7.06961f));
  points.push_back(Vector3(4.09589f, 1.32176f, 4.38643f));
  points.push_back(Vector3(0.25483f, -0.401319f, 4.30341f));
  points.push_back(Vector3(1.22532f, 9.60936f, 0.243843f));
  points.push_back(Vector3(-8.71639f, 8.42219f, -1.03977f));
  points.push_back(Vector3(-6.68752f, 6.87246f, 2.77932f));
  points.push_back(Vector3(-4.00555f, -8.53877f, -4.03851f));
  points.push_back(Vector3(-8.96237f, 6.24317f, -9.16074f));
  points.push_back(Vector3(1.8717f, -2.06702f, 1.4066f));
  points.push_back(Vector3(8.62056f, 6.2566f, 8.98312f));
  points.push_back(Vector3(3.49773f, -6.92862f, -8.61019f));
  points.push_back(Vector3(6.12232f, -9.5703f, 9.29258f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.49178f, 2.14026f, 6.01489f));
  points.push_back(Vector3(-6.63747f, 5.56505f, 2.73171f));
  points.push_back(Vector3(-9.62584f, -1.36143f, 4.39558f));
  points.push_back(Vector3(-6.91519f, 6.31825f, 7.18009f));
  points.push_back(Vector3(-2.49184f, -6.01672f, -6.42445f));
  points.push_back(Vector3(3.55388f, 9.92065f, 4.21247f));
  points.push_back(Vector3(4.279f, -4.09223f, -6.43117f));
  points.push_back(Vector3(8.83236f, -9.36216f, -4.56709f));
  points.push_back(Vector3(1.41209f, -0.632648f, -3.12174f));
  points.push_back(Vector3(3.05765f, -0.144963f, -0.966521f));
  points.push_back(Vector3(-3.94147f, 1.48961f, 5.12009f));
  points.push_back(Vector3(-3.87249f, 2.39784f, 0.736412f));
  points.push_back(Vector3(-0.81515f, -1.29002f, -0.264596f));
  points.push_back(Vector3(-4.77157f, -9.08078f, -6.30482f));
  points.push_back(Vector3(-3.25175f, 0.664388f, -1.95898f));
  points.push_back(Vector3(3.48979f, 9.55077f, 8.71212f));
  points.push_back(Vector3(-4.4847f, 3.28532f, 2.29957f));
  points.push_back(Vector3(7.6867f, 6.02588f, -7.8515f));
  points.push_back(Vector3(6.44215f, 7.32536f, -1.85705f));
  points.push_back(Vector3(1.0184f, 9.20591f, -4.20698f));
  points.push_back(Vector3(7.80999f, -9.25108f, -8.01385f));
  points.push_back(Vector3(7.04093f, 8.77316f, 3.31095f));
  points.push_back(Vector3(9.2346f, 6.6332f, 7.58843f));
  points.push_back(Vector3(-0.71688f, 8.47713f, -5.81347f));
  points.push_back(Vector3(8.81527f, 4.76302f, 2.09693f));
  points.push_back(Vector3(-9.47203f, 6.03503f, 0.809656f));
  points.push_back(Vector3(-8.50276f, 7.70989f, 4.31806f));
  points.push_back(Vector3(-1.44566f, -5.72802f, 9.99207f));
  points.push_back(Vector3(9.7528f, -6.90054f, -4.99374f));
  points.push_back(Vector3(3.94147f, -8.32331f, -0.669271f));
  points.push_back(Vector3(-9.66063f, 5.74206f, -6.77358f));
  points.push_back(Vector3(2.77139f, 8.90927f, 3.86578f));
  points.push_back(Vector3(-4.16791f, 8.59615f, -3.19742f));
  points.push_back(Vector3(5.21714f, -6.71987f, 1.9364f));
  points.push_back(Vector3(9.14182f, 2.88125f, 0.976897f));
  points.push_back(Vector3(2.6017f, -1.57811f, -6.40553f));
  points.push_back(Vector3(-3.97443f, 7.33573f, 0.242622f));
  points.push_back(Vector3(3.07047f, -6.24439f, 2.73904f));
  points.push_back(Vector3(7.03787f, 7.30216f, -3.68267f));
  points.push_back(Vector3(1.98279f, -9.56175f, 7.96319f));
  points.push_back(Vector3(-1.02817f, 8.43013f, 2.902f));
  points.push_back(Vector3(-9.96338f, 9.7058f, -0.695517f));
  points.push_back(Vector3(-0.785241f, -0.199286f, 7.7868f));
  points.push_back(Vector3(-7.48405f, 8.50642f, -8.72127f));
  points.push_back(Vector3(-3.10953f, 9.78027f, -2.6017f));
  points.push_back(Vector3(-0.694296f, -1.57445f, -0.851161f));
  points.push_back(Vector3(-5.0914f, 7.30888f, 0.674154f));
  points.push_back(Vector3(-9.9176f, -0.263985f, 7.94549f));
  points.push_back(Vector3(9.38047f, 4.18378f, 0.561235f));
  points.push_back(Vector3(4.71969f, -4.58846f, -5.2794f));
  points.push_back(Vector3(9.96948f, -7.30522f, 5.11094f));
  points.push_back(Vector3(-1.93274f, 2.64016f, 6.43788f));
  points.push_back(Vector3(8.97031f, -9.8236f, -1.58483f));
  points.push_back(Vector3(-6.69179f, -5.74999f, -8.77255f));
  points.push_back(Vector3(0.873134f, 0.234077f, -2.82571f));
  points.push_back(Vector3(-8.45088f, -7.9577f, -7.92169f));
  points.push_back(Vector3(0.307322f, 1.96936f, -6.13391f));
  points.push_back(Vector3(9.74181f, -9.2346f, 1.69408f));
  points.push_back(Vector3(-2.94717f, 6.10767f, 4.86862f));
  points.push_back(Vector3(6.85904f, 7.93451f, 7.97174f));
  points.push_back(Vector3(-5.14634f, -7.13797f, -6.04419f));
  points.push_back(Vector3(2.06885f, 3.30241f, -6.38234f));
  points.push_back(Vector3(-1.57384f, -6.67287f, 0.867641f));
  points.push_back(Vector3(-9.15708f, 5.14817f, -6.52516f));
  points.push_back(Vector3(-7.03726f, -3.48613f, 3.55693f));
  points.push_back(Vector3(8.12128f, -8.98984f, 1.60558f));
  points.push_back(Vector3(-5.00778f, 0.442213f, -6.10828f));
  points.push_back(Vector3(-5.74938f, -9.27f, -0.409864f));
  points.push_back(Vector3(-4.78683f, 0.435499f, 2.39357f));
  points.push_back(Vector3(-8.04376f, 2.02124f, -9.64232f));
  points.push_back(Vector3(-4.62752f, 3.41105f, 0.586261f));
  points.push_back(Vector3(3.57891f, 5.87939f, -9.36094f));
  points.push_back(Vector3(-5.12131f, -3.91705f, -7.74285f));
  points.push_back(Vector3(9.71557f, -6.62709f, 2.38258f));
  points.push_back(Vector3(1.1771f, 0.0131226f, 9.78881f));
  points.push_back(Vector3(-4.21857f, -0.664998f, -8.39351f));
  points.push_back(Vector3(-6.59536f, -0.0674456f, -8.70907f));
  points.push_back(Vector3(-5.73473f, -5.42528f, -5.19394f));
  points.push_back(Vector3(-5.94104f, -3.82427f, 1.05136f));
  points.push_back(Vector3(-9.80224f, 9.08261f, -9.48241f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.84313f, -5.74816f, -6.50807f));
  points.push_back(Vector3(-4.47798f, -2.7958f, -2.22144f));
  points.push_back(Vector3(2.38502f, 9.99939f, -8.40388f));
  points.push_back(Vector3(-2.73232f, 7.52434f, 8.95321f));
  points.push_back(Vector3(-3.96527f, 6.8102f, -9.94873f));
  points.push_back(Vector3(8.73409f, -7.8576f, -0.483719f));
  points.push_back(Vector3(6.69423f, 0.967742f, 9.62401f));
  points.push_back(Vector3(2.53578f, -2.77261f, -7.33634f));
  points.push_back(Vector3(-1.56651f, 3.04056f, -0.989715f));
  points.push_back(Vector3(5.11704f, 4.58418f, 3.31889f));
  points.push_back(Vector3(-5.78784f, -5.28367f, 0.202948f));
  points.push_back(Vector3(-6.9097f, 5.36546f, 8.80245f));
  points.push_back(Vector3(8.47896f, -2.82571f, 4.85519f));
  points.push_back(Vector3(-0.323191f, -5.88855f, 6.63686f));
  points.push_back(Vector3(-6.01184f, 9.91516f, -0.101627f));
  points.push_back(Vector3(-8.46736f, -2.74697f, -7.9223f));
  points.push_back(Vector3(0.0747705f, 3.38603f, 7.39799f));
  points.push_back(Vector3(-4.02264f, -8.95993f, 5.50768f));
  points.push_back(Vector3(6.95853f, 6.59291f, -9.90967f));
  points.push_back(Vector3(-0.70101f, 1.85461f, 5.78539f));
  points.push_back(Vector3(2.78481f, 4.65804f, -7.32048f));
  points.push_back(Vector3(0.923185f, 9.56786f, 2.60353f));
  points.push_back(Vector3(-0.68453f, -6.45375f, -7.72454f));
  points.push_back(Vector3(-1.74047f, 4.48592f, -8.80734f));
  points.push_back(Vector3(-8.84762f, -7.68914f, -8.11396f));
  points.push_back(Vector3(-4.47981f, -7.00491f, -0.936003f));
  points.push_back(Vector3(4.50728f, 9.8352f, 1.0593f));
  points.push_back(Vector3(-0.768151f, -1.22471f, -0.837123f));
  points.push_back(Vector3(7.34611f, 5.57054f, 0.142522f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.43013f, 5.50951f, 4.02081f));
  points.push_back(Vector3(8.66268f, 0.726036f, -2.97159f));
  points.push_back(Vector3(9.26511f, 3.25236f, -3.19071f));
  points.push_back(Vector3(-3.24259f, 3.5197f, 8.01385f));
  points.push_back(Vector3(-3.60454f, 7.05496f, -6.52882f));
  points.push_back(Vector3(-8.39412f, -2.65908f, -7.16605f));
  points.push_back(Vector3(-1.08188f, 1.01596f, 9.56114f));
  points.push_back(Vector3(1.74535f, -5.2678f, 8.15607f));
  points.push_back(Vector3(-2.28675f, 8.60958f, 0.055238f));
  points.push_back(Vector3(-5.20798f, 8.93124f, 4.73739f));
  points.push_back(Vector3(-7.16117f, 1.41575f, 8.70785f));
  points.push_back(Vector3(2.75613f, -2.13294f, -4.32234f));
  points.push_back(Vector3(-1.9187f, -2.42958f, 6.37501f));
  points.push_back(Vector3(6.02832f, -4.77218f, -5.92334f));
  points.push_back(Vector3(-4.60006f, -3.82977f, -5.40635f));
  points.push_back(Vector3(2.26661f, 4.9797f, 5.9624f));
  points.push_back(Vector3(6.69362f, -9.66918f, 1.32908f));
  points.push_back(Vector3(-5.61327f, -8.30439f, -3.59539f));
  points.push_back(Vector3(4.11725f, -9.78027f, 9.92737f));
  points.push_back(Vector3(9.56359f, 5.97278f, 6.01489f));
  points.push_back(Vector3(2.40516f, 0.703452f, 2.33863f));
  points.push_back(Vector3(1.03732f, 4.54817f, -6.06738f));
  points.push_back(Vector3(-0.193793f, 8.76583f, 6.16504f));
  points.push_back(Vector3(-0.906094f, 1.04831f, -5.41124f));
  points.push_back(Vector3(0.861537f, -3.95978f, -5.51927f));
  points.push_back(Vector3(-0.673543f, -2.01148f, -8.69747f));
  points.push_back(Vector3(6.77297f, -5.80981f, 0.248725f));
  points.push_back(Vector3(-2.24342f, 6.33778f, 2.81777f));
  points.push_back(Vector3(0.0985754f, -2.07251f, -4.07819f));
  points.push_back(Vector3(-3.99884f, -1.94128f, 1.55065f));
  points.push_back(Vector3(2.42286f, -5.26109f, -3.9317f));
  points.push_back(Vector3(-1.98462f, 2.05664f, 7.22892f));
  points.push_back(Vector3(8.99899f, -5.68224f, 9.00327f));
  points.push_back(Vector3(-4.56954f, 7.46086f, -0.857875f));
  points.push_back(Vector3(5.65416f, -3.16935f, -4.97665f));
  points.push_back(Vector3(3.70769f, 2.584f, -2.98135f));
  points.push_back(Vector3(-7.02933f, -0.0112921f, 6.37562f));
  points.push_back(Vector3(-1.28269f, -4.27229f, 6.95181f));
  points.push_back(Vector3(3.42143f, 2.87515f, -9.93408f));
  points.push_back(Vector3(5.41917f, -2.17322f, 7.45842f));
  points.push_back(Vector3(1.23569f, -7.80694f, -8.46065f));
  points.push_back(Vector3(-0.435499f, -0.874966f, -5.93799f));
  points.push_back(Vector3(4.83016f, 6.48f, 7.59209f));
  points.push_back(Vector3(-7.68609f, 5.24888f, 1.30345f));
  points.push_back(Vector3(-9.92004f, -1.61473f, 3.18522f));
  points.push_back(Vector3(8.77316f, -5.34776f, -8.73775f));
  points.push_back(Vector3(2.15491f, 6.90786f, -8.44172f));
  points.push_back(Vector3(-2.72622f, 1.9364f, -9.90478f));
  points.push_back(Vector3(6.4745f, 2.88186f, 2.68227f));
  points.push_back(Vector3(9.25169f, 2.52113f, 1.11179f));
  points.push_back(Vector3(8.13837f, -5.32701f, -5.79272f));
  points.push_back(Vector3(2.23853f, -3.91705f, -2.78604f));
  points.push_back(Vector3(6.40431f, 9.35606f, -5.87817f));
  points.push_back(Vector3(-6.59658f, 1.48473f, 9.45067f));
  points.push_back(Vector3(8.95749f, -9.86267f, 7.75872f));
  points.push_back(Vector3(0.616169f, 7.25944f, -5.43809f));
  points.push_back(Vector3(-7.68853f, 6.0155f, -3.73455f));
  points.push_back(Vector3(4.78011f, -8.60714f, -4.76608f));
  points.push_back(Vector3(2.22999f, 7.00125f, 1.45909f));
  points.push_back(Vector3(3.63811f, -5.15244f, -5.23606f));
  points.push_back(Vector3(4.01227f, -9.63927f, -4.2143f));
  points.push_back(Vector3(-4.14594f, 7.3516f, 5.55834f));
  points.push_back(Vector3(-0.564898f, 2.53639f, -1.64037f));
  points.push_back(Vector3(7.53349f, 4.01776f, -9.25901f));
  points.push_back(Vector3(7.45964f, 0.379956f, 1.16855f));
  points.push_back(Vector3(3.81512f, -0.144352f, -7.24601f));
  points.push_back(Vector3(2.12012f, 6.46901f, -3.81024f));
  points.push_back(Vector3(8.40083f, 8.61263f, 4.54634f));
  points.push_back(Vector3(-4.26008f, 9.07041f, 6.65639f));
  points.push_back(Vector3(-3.76446f, 5.27573f, 0.435499f));
  points.push_back(Vector3(-8.7286f, -9.64415f, 6.71194f));
  points.push_back(Vector3(1.64891f, 0.326242f, -2.66579f));
  points.push_back(Vector3(6.66799f, 9.14914f, 5.60656f));
  points.push_back(Vector3(8.5052f, 1.07089f, -4.64217f));
  points.push_back(Vector3(-0.769371f, 9.35545f, -2.07678f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.90439f, 4.15693f, 9.00571f));
  points.push_back(Vector3(-6.86331f, 2.39296f, 6.61794f));
  points.push_back(Vector3(-2.86111f, 8.47896f, -1.7246f));
  points.push_back(Vector3(9.84924f, 5.00534f, 1.61534f));
  points.push_back(Vector3(0.420851f, 3.47453f, 9.75707f));
  points.push_back(Vector3(-6.98843f, 1.66112f, -3.68206f));
  points.push_back(Vector3(-7.01895f, 3.73577f, -8.69747f));
  points.push_back(Vector3(-7.07327f, -6.37684f, 9.18943f));
  points.push_back(Vector3(-3.56792f, 2.17017f, 4.69771f));
  points.push_back(Vector3(1.47435f, -2.68349f, 3.52763f));
  points.push_back(Vector3(-3.08939f, -8.98557f, 5.03464f));
  points.push_back(Vector3(6.70522f, 2.3307f, 8.06696f));
  points.push_back(Vector3(-0.544145f, -9.81262f, 1.21006f));
  points.push_back(Vector3(-5.27207f, -4.30097f, -5.62792f));
  points.push_back(Vector3(5.10971f, 8.09015f, -2.8074f));
  points.push_back(Vector3(-5.11643f, -0.575884f, 5.13779f));
  points.push_back(Vector3(3.45622f, -8.27693f, -1.5421f));
  points.push_back(Vector3(-9.95972f, -9.25779f, -0.144352f));
  points.push_back(Vector3(-3.8432f, -5.55345f, -6.35121f));
  points.push_back(Vector3(-3.13456f, 5.63402f, -8.59004f));
  points.push_back(Vector3(5.833f, -6.68569f, -9.39512f));
  points.push_back(Vector3(-6.42018f, 7.95404f, 1.41331f));
  points.push_back(Vector3(1.86987f, -3.73455f, 0.966522f));
  points.push_back(Vector3(7.64336f, 1.94861f, -3.98419f));
  points.push_back(Vector3(-7.10868f, 4.94369f, -8.98557f));
  points.push_back(Vector3(5.32701f, -2.8135f, -9.90051f));
  points.push_back(Vector3(-2.7604f, -5.88977f, -9.15525f));
  points.push_back(Vector3(1.94739f, 4.67391f, -1.45604f));
  points.push_back(Vector3(-9.65514f, 6.02222f, 1.7954f));
  points.push_back(Vector3(7.9284f, -6.42445f, 1.2833f));
  points.push_back(Vector3(3.84808f, -9.82299f, 5.86596f));
  points.push_back(Vector3(-5.47227f, -7.91192f, 6.5685f));
  points.push_back(Vector3(-1.76305f, -1.86926f, 3.0784f));
  points.push_back(Vector3(-3.62713f, -3.98907f, -3.2786f));
  points.push_back(Vector3(-8.8641f, 1.99499f, 5.8916f));
  points.push_back(Vector3(-1.55675f, -2.9191f, -4.75631f));
  points.push_back(Vector3(-0.353099f, -3.60576f, 6.39088f));
  points.push_back(Vector3(-4.63607f, 2.5248f, -4.09894f));
  points.push_back(Vector3(9.89441f, -0.553911f, 4.51033f));
  points.push_back(Vector3(-6.77114f, -9.86694f, -4.15021f));
  points.push_back(Vector3(2.56508f, 1.3657f, 6.51357f));
  points.push_back(Vector3(1.25584f, -8.02911f, 8.50276f));
  points.push_back(Vector3(2.36183f, -8.02362f, -5.10666f));
  points.push_back(Vector3(4.83078f, 7.03421f, -9.84924f));
  points.push_back(Vector3(4.30219f, 3.28837f, -3.82061f));
  points.push_back(Vector3(-4.5555f, -8.63643f, 9.87609f));
  points.push_back(Vector3(3.29447f, 2.88614f, -5.55834f));
  points.push_back(Vector3(9.16257f, 5.69201f, -2.5425f));
  points.push_back(Vector3(-2.1836f, 0.621662f, -1.98828f));
  points.push_back(Vector3(6.07166f, -1.73864f, -8.59737f));
  points.push_back(Vector3(-3.60881f, -2.44728f, 7.59392f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.77355f, 3.86334f, -4.88815f));
  points.push_back(Vector3(9.5703f, -2.45277f, 1.66234f));
  points.push_back(Vector3(8.87448f, 7.46574f, 4.32173f));
  points.push_back(Vector3(5.93127f, 4.36506f, 9.8938f));
  points.push_back(Vector3(8.97275f, 0.277413f, 4.12763f));
  points.push_back(Vector3(-0.884731f, 5.03708f, -6.87857f));
  points.push_back(Vector3(5.80248f, 3.50871f, -6.5154f));
  points.push_back(Vector3(-6.75588f, -2.02124f, 2.51198f));
  points.push_back(Vector3(-9.61364f, -0.718711f, -5.79028f));
  points.push_back(Vector3(-2.92459f, -1.94372f, -1.20884f));
  points.push_back(Vector3(-1.68065f, 6.24683f, -2.19825f));
  points.push_back(Vector3(2.51015f, 4.12091f, 0.509964f));
  points.push_back(Vector3(-7.60613f, 9.43724f, 2.67373f));
  points.push_back(Vector3(-7.48527f, -3.60515f, 8.3581f));
  points.push_back(Vector3(0.207831f, -4.19111f, 2.45766f));
  points.push_back(Vector3(-8.5577f, -0.165105f, -7.80084f));
  points.push_back(Vector3(1.72765f, 5.53453f, -9.82726f));
  points.push_back(Vector3(-1.61901f, 6.47511f, -3.23527f));
  points.push_back(Vector3(-5.98865f, -7.55852f, -6.9274f));
  points.push_back(Vector3(-5.89282f, 0.13947f, -9.12595f));
  points.push_back(Vector3(-6.54286f, 7.50969f, -8.53572f));
  points.push_back(Vector3(-3.27433f, -6.8157f, 6.2096f));
  points.push_back(Vector3(-7.45903f, 9.50255f, 4.21674f));
  points.push_back(Vector3(-7.116f, 9.64782f, -2.54677f));
  points.push_back(Vector3(-9.44029f, -0.763268f, 6.51418f));
  points.push_back(Vector3(2.87332f, -5.12253f, -1.34739f));
  points.push_back(Vector3(8.85067f, 1.02329f, 4.20698f));
  points.push_back(Vector3(1.55675f, 0.784631f, 3.32072f));
  points.push_back(Vector3(2.89773f, 9.08628f, 6.33778f));
  points.push_back(Vector3(5.40757f, -9.09055f, -9.29136f));
  points.push_back(Vector3(-0.935392f, -9.27305f, -4.42122f));
  points.push_back(Vector3(1.41636f, -7.01224f, -9.76745f));
  points.push_back(Vector3(-5.00717f, 4.10688f, -2.06214f));
  points.push_back(Vector3(8.85617f, 1.08615f, -7.20878f));
  points.push_back(Vector3(-7.56157f, -0.644246f, 0.590533f));
  points.push_back(Vector3(2.24525f, -2.74697f, -5.82263f));
  points.push_back(Vector3(1.96081f, -0.273751f, 2.72195f));
  points.push_back(Vector3(-4.05744f, 6.40614f, 4.39741f));
  points.push_back(Vector3(-6.98721f, -2.8019f, -8.86166f));
  points.push_back(Vector3(-9.94324f, 0.409863f, -9.95361f));
  points.push_back(Vector3(2.98379f, -3.17606f, -7.23319f));
  points.push_back(Vector3(-3.71319f, -9.53856f, 7.3925f));
  points.push_back(Vector3(7.25089f, -1.31443f, -7.52861f));
  points.push_back(Vector3(-6.52699f, -6.50136f, 4.00678f));
  points.push_back(Vector3(-8.30073f, 8.07001f, -0.687582f));
  points.push_back(Vector3(-1.5775f, -9.11435f, 9.18394f));
  points.push_back(Vector3(2.02002f, 5.93799f, 0.0595117f));
  points.push_back(Vector3(-9.39451f, -1.3773f, -9.55992f));
  points.push_back(Vector3(-7.12027f, 5.38865f, -2.14087f));
  points.push_back(Vector3(-7.98883f, 0.204779f, -2.34962f));
  points.push_back(Vector3(7.30888f, -1.87841f, -0.992767f));
  points.push_back(Vector3(8.31843f, -1.34678f, 0.332958f));
  points.push_back(Vector3(4.77096f, -9.5587f, 4.23872f));
  points.push_back(Vector3(-2.87088f, 5.48692f, -0.295724f));
  points.push_back(Vector3(8.03705f, -6.48732f, 5.37828f));
  points.push_back(Vector3(-4.24604f, 4.16181f, -2.02185f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.04965f, 5.25437f, -6.94449f));
  points.push_back(Vector3(0.98056f, 2.82022f, -8.34101f));
  points.push_back(Vector3(-1.53844f, -1.0947f, 2.59011f));
  points.push_back(Vector3(3.78033f, -8.4051f, -0.347606f));
  points.push_back(Vector3(-5.99109f, 6.24195f, 0.970184f));
  points.push_back(Vector3(0.501419f, 6.76748f, -4.62996f));
  points.push_back(Vector3(-8.58821f, -4.42183f, 2.67739f));
  points.push_back(Vector3(3.16385f, 4.62813f, -3.35063f));
  points.push_back(Vector3(5.78661f, 9.43907f, -4.71908f));
  points.push_back(Vector3(7.38945f, -7.92657f, -4.74288f));
  points.push_back(Vector3(3.2017f, 9.25352f, 4.32844f));
  points.push_back(Vector3(0.773034f, -7.64519f, 7.07694f));
  points.push_back(Vector3(7.86065f, 0.612507f, -2.81106f));
  points.push_back(Vector3(-7.47734f, -7.34977f, -0.0753808f));
  points.push_back(Vector3(6.99515f, -6.48915f, -2.4308f));
  points.push_back(Vector3(4.21552f, 8.89706f, 7.74224f));
  points.push_back(Vector3(2.2953f, 6.10034f, 0.0985754f));
  points.push_back(Vector3(8.45576f, 2.55104f, 2.09021f));
  points.push_back(Vector3(-9.9823f, 0.533159f, -5.71337f));
  points.push_back(Vector3(2.55043f, 2.9252f, -4.91806f));
  points.push_back(Vector3(2.4308f, 7.93878f, 1.71239f));
  points.push_back(Vector3(0.620441f, -0.161443f, -5.53636f));
  points.push_back(Vector3(3.42753f, -1.01352f, -7.93939f));
  points.push_back(Vector3(2.28187f, 7.18253f, 4.10077f));
  points.push_back(Vector3(-6.44398f, 9.35545f, 5.07797f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.93857f, -7.36869f, 3.64727f));
  points.push_back(Vector3(3.45683f, -4.63118f, 5.8739f));
  points.push_back(Vector3(4.8442f, -1.1478f, 6.95181f));
  points.push_back(Vector3(-6.35914f, 6.53981f, 1.15818f));
  points.push_back(Vector3(-9.29136f, -8.20978f, -0.508744f));
  points.push_back(Vector3(-6.88833f, 3.39824f, 5.1384f));
  points.push_back(Vector3(-6.01489f, -0.355541f, -7.14469f));
  points.push_back(Vector3(-7.70257f, -2.17017f, -3.59416f));
  points.push_back(Vector3(4.91195f, 7.42485f, 3.60027f));
  points.push_back(Vector3(-9.47752f, -6.37013f, -7.98761f));
  points.push_back(Vector3(5.77319f, -8.30012f, 0.48677f));
  points.push_back(Vector3(-0.15595f, -1.63671f, 1.80883f));
  points.push_back(Vector3(-1.87475f, 8.23237f, -6.30177f));
  points.push_back(Vector3(1.06418f, 4.30464f, -1.32115f));
  points.push_back(Vector3(-7.05435f, -1.05869f, 1.66356f));
  points.push_back(Vector3(0.070498f, 9.02768f, 7.93634f));
  points.push_back(Vector3(8.93918f, -2.46315f, -7.46147f));
  points.push_back(Vector3(8.30195f, -0.754113f, 0.101627f));
  points.push_back(Vector3(1.30283f, 1.89123f, -8.94223f));
  points.push_back(Vector3(-9.75646f, 4.18134f, 3.22428f));
  points.push_back(Vector3(7.50664f, 8.50093f, -0.232856f));
  points.push_back(Vector3(5.90503f, 9.62584f, -3.7669f));
  points.push_back(Vector3(0.588702f, -7.48405f, -0.833461f));
  points.push_back(Vector3(9.68261f, 3.44157f, 6.11805f));
  points.push_back(Vector3(-3.39213f, 8.43684f, -3.51848f));
  points.push_back(Vector3(-8.53084f, 9.98657f, -8.5577f));
  points.push_back(Vector3(3.70159f, 4.10504f, 0.17365f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.59737f, -5.5565f, 8.13349f));
  points.push_back(Vector3(-5.05234f, -8.09809f, -4.29304f));
  points.push_back(Vector3(9.69115f, -6.66799f, -9.73266f));
  points.push_back(Vector3(-8.034f, -8.89767f, 5.75549f));
  points.push_back(Vector3(1.34556f, -1.26804f, -4.31013f));
  points.push_back(Vector3(2.69082f, 2.3954f, 8.18171f));
  points.push_back(Vector3(5.05783f, -5.08347f, 4.29304f));
  points.push_back(Vector3(8.04498f, 4.92904f, 8.52046f));
  points.push_back(Vector3(9.02646f, 1.6306f, 7.57622f));
  points.push_back(Vector3(-9.90844f, -3.40373f, -0.395825f));
  points.push_back(Vector3(-1.01779f, -2.25501f, -5.53026f));
  points.push_back(Vector3(4.15021f, 8.85434f, 3.57952f));
  points.push_back(Vector3(-7.3162f, -3.85846f, -8.38923f));
  points.push_back(Vector3(3.21635f, -2.25257f, 5.94592f));
  points.push_back(Vector3(-3.52641f, -1.97546f, 8.4521f));
  points.push_back(Vector3(-1.51585f, -5.21653f, 5.67125f));
  points.push_back(Vector3(6.72719f, -6.82485f, 2.66884f));
  points.push_back(Vector3(4.61776f, 6.65395f, 9.3878f));
  points.push_back(Vector3(7.19047f, 8.18232f, -5.22996f));
  points.push_back(Vector3(-1.97729f, -5.91174f, -6.19862f));
  points.push_back(Vector3(4.79171f, 9.86022f, 5.2562f));
  points.push_back(Vector3(3.11136f, -0.132145f, 1.27903f));
  points.push_back(Vector3(4.06415f, -1.52867f, 5.50645f));
  points.push_back(Vector3(7.99371f, -3.33964f, -8.43684f));
  points.push_back(Vector3(6.28407f, 7.41813f, 7.37541f));
  points.push_back(Vector3(9.68017f, 9.4702f, 0.290231f));
  points.push_back(Vector3(-3.65093f, -4.6617f, 8.69503f));
  points.push_back(Vector3(-7.82342f, -2.48512f, 4.83871f));
  points.push_back(Vector3(-9.22666f, 5.49059f, 5.05417f));
  points.push_back(Vector3(1.38707f, -1.17038f, 2.63283f));
  points.push_back(Vector3(6.45558f, 8.42341f, -5.17991f));
  points.push_back(Vector3(-9.09787f, -9.69726f, -1.23936f));
  points.push_back(Vector3(-5.38804f, 9.78149f, 6.91214f));
  points.push_back(Vector3(5.17502f, -1.28636f, -8.77133f));
  points.push_back(Vector3(9.94202f, 8.06513f, -8.20002f));
  points.push_back(Vector3(-6.386f, -2.88797f, 5.80432f));
  points.push_back(Vector3(-5.84948f, -0.103458f, -1.85461f));
  points.push_back(Vector3(3.41655f, -1.87353f, 0.698569f));
  points.push_back(Vector3(8.05902f, 5.19394f, -7.18925f));
  points.push_back(Vector3(-7.11844f, -6.06494f, 6.89077f));
  points.push_back(Vector3(9.94018f, -5.74572f, -6.70522f));
  points.push_back(Vector3(-6.89749f, -5.75304f, 5.04135f));
  points.push_back(Vector3(-7.80633f, -4.52132f, -8.65474f));
  points.push_back(Vector3(-3.26823f, 1.17283f, -2.63283f));
  points.push_back(Vector3(-3.72417f, -1.06052f, -2.81716f));
  points.push_back(Vector3(0.539262f, -4.15571f, 4.89059f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.73104f, -5.0969f, -2.39662f));
  points.push_back(Vector3(-9.08689f, -9.89746f, -4.87106f));
  points.push_back(Vector3(2.41188f, -6.96158f, -4.9498f));
  points.push_back(Vector3(3.38298f, -2.8489f, 3.95428f));
  points.push_back(Vector3(-3.99152f, 2.09449f, 8.17377f));
  points.push_back(Vector3(-6.32557f, 5.71215f, -5.2916f));
  points.push_back(Vector3(-9.80712f, -7.09037f, -2.52724f));
  points.push_back(Vector3(-5.02609f, -9.08078f, 1.66845f));
  points.push_back(Vector3(-8.14203f, 9.39818f, -3.80657f));
  points.push_back(Vector3(-3.65764f, 7.94916f, 5.98682f));
  points.push_back(Vector3(-3.7376f, -6.61183f, 4.78072f));
  points.push_back(Vector3(-7.59575f, 3.80779f, -2.26051f));
  points.push_back(Vector3(8.61751f, -3.33171f, 4.35896f));
  points.push_back(Vector3(-5.93127f, -8.33857f, -6.4922f));
  points.push_back(Vector3(9.71191f, 9.02707f, -2.16102f));
  points.push_back(Vector3(9.51537f, -5.01572f, 2.58217f));
  points.push_back(Vector3(1.27781f, -9.7943f, 9.92798f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.5876f, 7.2454f, 2.64992f));
  points.push_back(Vector3(8.38313f, 3.37993f, 7.69097f));
  points.push_back(Vector3(-0.810877f, 4.39924f, 0.657064f));
  points.push_back(Vector3(-6.79617f, -9.34873f, 5.88977f));
  points.push_back(Vector3(8.90866f, 9.22788f, 3.41472f));
  points.push_back(Vector3(-4.39802f, 4.23749f, 4.41084f));
  points.push_back(Vector3(-9.35545f, 6.70888f, -0.869473f));
  points.push_back(Vector3(-4.71114f, -9.24924f, 9.92737f));
  points.push_back(Vector3(-6.4098f, -4.62081f, -7.39555f));
  points.push_back(Vector3(0.371411f, 7.09281f, 1.81494f));
  points.push_back(Vector3(8.1402f, 2.24891f, -3.01614f));
  points.push_back(Vector3(2.63833f, 2.96182f, -0.462356f));
  points.push_back(Vector3(-1.04221f, -2.10669f, 5.58519f));
  points.push_back(Vector3(-9.23582f, -4.78378f, -2.37953f));
  points.push_back(Vector3(5.93371f, -3.99091f, 5.96057f));
  points.push_back(Vector3(9.8999f, 3.72417f, -5.74023f));
  points.push_back(Vector3(9.82055f, 2.05359f, -2.64931f));
  points.push_back(Vector3(-0.753502f, 8.85861f, 8.13898f));
  points.push_back(Vector3(-6.08814f, -8.12555f, 3.76751f));
  points.push_back(Vector3(5.85315f, -9.01425f, -2.61452f));
  points.push_back(Vector3(-5.11277f, -4.01837f, -9.20347f));
  points.push_back(Vector3(5.36912f, -5.33006f, 7.50847f));
  points.push_back(Vector3(-2.61818f, 0.75106f, 7.21244f));
  points.push_back(Vector3(1.51158f, -2.60231f, 3.88653f));
  points.push_back(Vector3(0.484939f, -3.02103f, -4.82101f));
  points.push_back(Vector3(7.87347f, 6.88223f, -8.3404f));
  points.push_back(Vector3(2.6841f, -8.77743f, -2.28431f));
  points.push_back(Vector3(1.08066f, -5.05539f, -5.99597f));
  points.push_back(Vector3(7.28141f, 8.85006f, -8.34712f));
  points.push_back(Vector3(-4.40962f, -7.88324f, -8.62239f));
  points.push_back(Vector3(-4.49934f, 2.13294f, 1.08493f));
  points.push_back(Vector3(-9.86511f, 7.90765f, 1.8833f));
  points.push_back(Vector3(1.89734f, -5.42528f, -7.24967f));
  points.push_back(Vector3(-3.50017f, -0.28901f, 8.84335f));
  points.push_back(Vector3(8.7347f, 6.44398f, -6.57949f));
  points.push_back(Vector3(5.40574f, 1.00925f, -1.38524f));
  points.push_back(Vector3(5.27146f, -1.2888f, -7.31742f));
  points.push_back(Vector3(-5.67309f, 9.41282f, 2.02307f));
  points.push_back(Vector3(6.83035f, 5.77075f, -4.97177f));
  points.push_back(Vector3(0.532548f, -0.356761f, -0.932951f));
  points.push_back(Vector3(9.66002f, 3.89386f, -4.66659f));
  points.push_back(Vector3(-7.94183f, -1.65319f, -8.70785f));
  points.push_back(Vector3(3.42265f, 4.94308f, 4.21735f));
  points.push_back(Vector3(-7.81732f, 9.035f, -8.11335f));
  points.push_back(Vector3(6.14551f, -4.18928f, -3.17423f));
  points.push_back(Vector3(-8.25617f, -0.893277f, -7.34916f));
  points.push_back(Vector3(-0.827967f, 5.99353f, -7.14835f));
  points.push_back(Vector3(0.884122f, 9.23337f, 2.03528f));
  points.push_back(Vector3(-5.47533f, 1.03916f, 9.53551f));
  points.push_back(Vector3(9.96094f, -9.14487f, -8.07794f));
  points.push_back(Vector3(-3.32438f, -9.07712f, -4.45845f));
  points.push_back(Vector3(-9.03745f, -7.20634f, 5.23545f));
  points.push_back(Vector3(-2.54128f, -7.90643f, 4.41755f));
  points.push_back(Vector3(-1.67882f, -4.69344f, 4.30219f));
  points.push_back(Vector3(-0.675985f, -7.36015f, -5.66637f));
  points.push_back(Vector3(3.96344f, 2.02979f, 3.71685f));
  points.push_back(Vector3(-9.24863f, 4.09162f, -1.46153f));
  points.push_back(Vector3(8.39229f, -0.527055f, -6.36891f));
  points.push_back(Vector3(-9.02768f, 3.43364f, -9.20774f));
  points.push_back(Vector3(9.91089f, -7.28751f, 2.61513f));
  points.push_back(Vector3(-8.35933f, -2.62001f, 4.67086f));
  points.push_back(Vector3(-9.59899f, -2.96731f, -4.22529f));
  points.push_back(Vector3(3.31584f, 0.358592f, 7.33207f));
  points.push_back(Vector3(-9.19309f, 1.09165f, -3.58196f));
  points.push_back(Vector3(-3.03201f, -9.47264f, 9.8352f));
  points.push_back(Vector3(5.11216f, -9.78149f, 0.472121f));
  points.push_back(Vector3(7.82647f, -3.08267f, 9.6881f));
  points.push_back(Vector3(0.727867f, 1.1063f, 4.8204f));
  points.push_back(Vector3(-6.89261f, -1.96692f, -0.336619f));
  points.push_back(Vector3(-1.7655f, 4.07941f, -3.95611f));
  points.push_back(Vector3(-0.857875f, 0.398266f, -7.60918f));
  points.push_back(Vector3(-3.11014f, 8.67122f, 7.36992f));
  points.push_back(Vector3(9.52879f, -2.85745f, -4.54085f));
  points.push_back(Vector3(-3.24992f, -4.99741f, -8.67672f));
  points.push_back(Vector3(6.77175f, 4.25764f, -0.872524f));
  points.push_back(Vector3(6.07837f, -0.0918609f, -3.72051f));
  points.push_back(Vector3(-2.13355f, -1.86438f, 0.822474f));
  points.push_back(Vector3(-3.4727f, 5.374f, -0.813929f));
  points.push_back(Vector3(1.99683f, -5.08896f, -0.145573f));
  points.push_back(Vector3(0.793176f, 0.194403f, -3.2255f));
  points.push_back(Vector3(8.37458f, 2.47291f, 4.72579f));
  points.push_back(Vector3(0.600909f, 6.23585f, -3.93658f));
  points.push_back(Vector3(-3.15409f, 2.32276f, -3.45012f));
  points.push_back(Vector3(9.50438f, -9.09116f, -5.6444f));
  points.push_back(Vector3(5.63219f, 3.15897f, -2.09387f));
  points.push_back(Vector3(-2.51137f, -0.162663f, 1.79907f));
  points.push_back(Vector3(2.00903f, -6.25416f, -0.529496f));
  points.push_back(Vector3(-4.62264f, -9.25047f, -1.64769f));
  points.push_back(Vector3(-4.6086f, -8.28242f, 9.4586f));
  points.push_back(Vector3(-1.07578f, 3.9378f, -5.75976f));
  points.push_back(Vector3(0.577105f, 0.0161743f, 5.79211f));
  points.push_back(Vector3(7.85394f, -1.50792f, 1.13804f));
  points.push_back(Vector3(4.42854f, 6.13514f, 0.489823f));
  points.push_back(Vector3(6.37745f, 9.74242f, 2.92398f));
  points.push_back(Vector3(4.47493f, -6.25294f, 2.50587f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.75951f, 9.78515f, -8.5638f));
  points.push_back(Vector3(-7.62078f, 1.61595f, -1.56102f));
  points.push_back(Vector3(2.172f, -5.04135f, -5.13352f));
  points.push_back(Vector3(-0.336009f, 3.73272f, -9.91821f));
  points.push_back(Vector3(3.55998f, -9.02951f, -1.05747f));
  points.push_back(Vector3(-6.80837f, -0.384228f, -8.1579f));
  points.push_back(Vector3(-6.37623f, 4.01715f, 2.25745f));
  points.push_back(Vector3(-0.614337f, 2.12867f, -7.51885f));
  points.push_back(Vector3(-7.22282f, 6.53005f, 5.2501f));
  points.push_back(Vector3(7.46757f, 0.75106f, 5.8855f));
  points.push_back(Vector3(-8.85922f, -3.44768f, -2.67861f));
  points.push_back(Vector3(-7.60674f, -0.128483f, -5.14878f));
  points.push_back(Vector3(1.1771f, 7.88934f, 2.23731f));
  points.push_back(Vector3(-8.9172f, -7.14835f, 0.702231f));
  points.push_back(Vector3(-6.54897f, 8.98801f, 2.96915f));
  points.push_back(Vector3(3.42448f, -7.64885f, -7.77642f));
  points.push_back(Vector3(-9.94263f, 1.96387f, 6.72842f));
  points.push_back(Vector3(-9.13816f, -6.43849f, -0.984222f));
  points.push_back(Vector3(-4.57808f, -2.38868f, -2.3307f));
  points.push_back(Vector3(5.06943f, 7.79717f, -8.10785f));
  points.push_back(Vector3(-8.40815f, 0.0796533f, -8.77926f));
  points.push_back(Vector3(-0.113834f, 1.90771f, -1.06967f));
  points.push_back(Vector3(7.1868f, 8.6932f, 2.60476f));
  points.push_back(Vector3(3.80963f, 6.6802f, 2.70608f));
  points.push_back(Vector3(-1.55858f, -3.37809f, 5.43931f));
  points.push_back(Vector3(0.343944f, 7.25456f, -5.42955f));
  points.push_back(Vector3(9.71923f, -0.739464f, -3.16385f));
  points.push_back(Vector3(2.02185f, -6.81387f, 8.15241f));
  points.push_back(Vector3(2.00842f, 1.7893f, -4.42366f));
  points.push_back(Vector3(-6.70827f, -2.48817f, -1.04831f));
  points.push_back(Vector3(3.72845f, -2.07984f, -7.65069f));
  points.push_back(Vector3(0.342723f, 5.63402f, 2.60537f));
  points.push_back(Vector3(1.19236f, 2.81838f, -1.88757f));
  points.push_back(Vector3(4.71053f, -1.5183f, -6.54897f));
  points.push_back(Vector3(-1.25217f, 5.08652f, -9.14121f));
  points.push_back(Vector3(-5.24766f, 2.70913f, 0.112003f));
  points.push_back(Vector3(-6.47084f, -1.93274f, -4.92538f));
  points.push_back(Vector3(7.21793f, 6.20594f, -1.74841f));
  points.push_back(Vector3(-0.706503f, 2.00476f, -5.75732f));
  points.push_back(Vector3(-7.23197f, -7.59941f, -4.90646f));
  points.push_back(Vector3(-9.40855f, -1.68065f, 7.62078f));
  points.push_back(Vector3(6.82608f, 4.20209f, 8.14447f));
  points.push_back(Vector3(-5.02976f, 7.93634f, -9.50865f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.69686f, 6.42872f, 2.96182f));
  points.push_back(Vector3(0.593585f, -2.87332f, -4.12763f));
  points.push_back(Vector3(-5.29344f, 2.66579f, 6.3744f));
  points.push_back(Vector3(-0.409253f, -4.38398f, 4.41084f));
  points.push_back(Vector3(5.7213f, -6.22181f, -1.56163f));
  points.push_back(Vector3(-6.42628f, 3.97504f, 5.88855f));
  points.push_back(Vector3(9.18882f, -2.99905f, -4.08063f));
  points.push_back(Vector3(-4.39619f, 6.71316f, -5.54796f));
  points.push_back(Vector3(3.3787f, 4.1435f, -2.70913f));
  points.push_back(Vector3(7.29423f, -8.36482f, -2.49855f));
  points.push_back(Vector3(7.40043f, -7.29789f, 9.3292f));
  points.push_back(Vector3(-0.735191f, 7.7752f, 3.99762f));
  points.push_back(Vector3(9.94812f, -1.37608f, -8.12983f));
  points.push_back(Vector3(7.7337f, 3.05643f, 9.20164f));
  points.push_back(Vector3(1.87536f, -1.8247f, -6.63259f));
  points.push_back(Vector3(4.32844f, 5.38133f, -0.216376f));
  points.push_back(Vector3(-4.20515f, 4.39253f, -4.15326f));
  points.push_back(Vector3(-1.67333f, 4.60494f, 1.87719f));
  points.push_back(Vector3(-0.0283819f, -8.74508f, 4.79598f));
  points.push_back(Vector3(-0.274972f, 5.10971f, -2.03955f));
  points.push_back(Vector3(3.83099f, 8.64437f, 9.85046f));
  points.push_back(Vector3(0.536211f, -8.15668f, -1.01535f));
  points.push_back(Vector3(-9.11618f, -4.32295f, 8.44844f));
  points.push_back(Vector3(-7.18558f, 9.28648f, 8.33369f));
  points.push_back(Vector3(4.27351f, -8.19086f, 9.33653f));
  points.push_back(Vector3(7.74712f, -4.64827f, 6.5923f));
  points.push_back(Vector3(-6.9982f, -3.502f, -1.51646f));
  points.push_back(Vector3(3.14127f, 3.21573f, -6.8157f));
  points.push_back(Vector3(5.69994f, -8.10358f, -0.939054f));
  points.push_back(Vector3(-0.644856f, 5.95508f, -4.81857f));
  points.push_back(Vector3(-4.00189f, -8.67f, -7.82769f));
  points.push_back(Vector3(-1.42674f, -5.74084f, 7.52922f));
  points.push_back(Vector3(1.76305f, 1.8424f, 8.52473f));
  points.push_back(Vector3(-8.07245f, 8.74569f, -4.90951f));
  points.push_back(Vector3(2.01819f, 4.27656f, -4.40413f));
  points.push_back(Vector3(-4.29853f, -1.85888f, -2.29957f));
  points.push_back(Vector3(2.94839f, -8.25373f, -7.22953f));
  points.push_back(Vector3(-5.69201f, 5.26902f, 7.45293f));
  points.push_back(Vector3(0.342723f, -0.56856f, -9.36522f));
  points.push_back(Vector3(-3.62896f, -3.1431f, 5.12436f));
  points.push_back(Vector3(-7.26005f, -2.13843f, -7.08853f));
  points.push_back(Vector3(3.54228f, -6.75222f, 2.52846f));
  points.push_back(Vector3(-0.0894195f, 0.835291f, 2.62734f));
  points.push_back(Vector3(2.21839f, 2.55654f, -7.83319f));
  points.push_back(Vector3(-2.41676f, -9.77905f, 0.753502f));
  points.push_back(Vector3(8.49605f, 3.22123f, 7.28874f));
  points.push_back(Vector3(-5.94348f, -4.26435f, -1.94433f));
  points.push_back(Vector3(-0.906705f, -8.21833f, -3.07047f));
  points.push_back(Vector3(-9.99695f, 0.768762f, 1.90161f));
  points.push_back(Vector3(-1.95105f, 1.46764f, -5.86718f));
  points.push_back(Vector3(1.86071f, 8.05902f, -8.33979f));
  points.push_back(Vector3(-6.76077f, 4.42793f, -6.14246f));
  points.push_back(Vector3(-5.73107f, 3.87432f, 1.71178f));
  points.push_back(Vector3(0.732139f, -1.39927f, -7.38334f));
  points.push_back(Vector3(0.465407f, -1.06052f, -7.67083f));
  points.push_back(Vector3(-8.89706f, -8.19391f, 7.05191f));
  points.push_back(Vector3(-6.03076f, -8.89401f, 8.4753f));
  points.push_back(Vector3(-7.3217f, -6.80105f, -3.94024f));
  points.push_back(Vector3(-7.24479f, 1.20273f, 7.16422f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.47407f, 1.18259f, 4.14533f));
  points.push_back(Vector3(5.54674f, 2.22449f, 4.30158f));
  points.push_back(Vector3(7.47185f, 2.55898f, -4.90402f));
  points.push_back(Vector3(-0.687582f, 5.50584f, 7.42241f));
  points.push_back(Vector3(-0.879238f, 4.87533f, -4.51888f));
  points.push_back(Vector3(-1.20518f, -2.76406f, 5.42528f));
  points.push_back(Vector3(-5.66454f, 5.13291f, -3.72356f));
  points.push_back(Vector3(-6.12903f, -6.88284f, -6.03504f));
  points.push_back(Vector3(1.62877f, 0.115055f, -6.07044f));
  points.push_back(Vector3(-6.4391f, -5.25193f, -8.98862f));
  points.push_back(Vector3(-7.4572f, -2.40272f, 1.64525f));
  points.push_back(Vector3(-2.59316f, -0.207831f, -6.53859f));
  points.push_back(Vector3(-9.52269f, -6.13697f, 4.88937f));
  points.push_back(Vector3(4.81552f, 6.36403f, 6.70949f));
  points.push_back(Vector3(-6.22364f, 9.03684f, 1.55675f));
  points.push_back(Vector3(-3.96222f, 5.64318f, 0.171819f));
  points.push_back(Vector3(9.08322f, 7.26676f, 0.727867f));
  points.push_back(Vector3(-1.98096f, -2.00537f, -2.09937f));
  points.push_back(Vector3(2.54128f, -6.02832f, -1.66173f));
  points.push_back(Vector3(-4.88815f, 7.622f, -5.75549f));
  points.push_back(Vector3(-6.01794f, 9.76867f, 5.95996f));
  points.push_back(Vector3(-8.89584f, -8.58272f, 2.52358f));
  points.push_back(Vector3(5.20676f, 2.73598f, 2.65908f));
  points.push_back(Vector3(7.61528f, 1.80395f, -7.63237f));
  points.push_back(Vector3(-9.6234f, 9.97314f, -7.85638f));
  points.push_back(Vector3(7.27653f, -6.62404f, 1.27537f));
  points.push_back(Vector3(6.90054f, 8.30073f, 0.555742f));
  points.push_back(Vector3(-9.41954f, 1.64586f, -9.51048f));
  points.push_back(Vector3(-8.5345f, -4.85397f, 6.88528f));
  points.push_back(Vector3(5.50706f, -8.40632f, 7.07266f));
  points.push_back(Vector3(-9.37864f, 6.41896f, 0.50325f));
  points.push_back(Vector3(1.39073f, -3.40312f, 5.48692f));
  points.push_back(Vector3(0.945158f, 1.22471f, -8.6877f));
  points.push_back(Vector3(5.60167f, 7.07511f, 5.64135f));
  points.push_back(Vector3(2.96793f, -4.03668f, -0.106509f));
  points.push_back(Vector3(-4.17463f, 5.43687f, -7.34733f));
  points.push_back(Vector3(8.04254f, 5.20249f, -9.58983f));
  points.push_back(Vector3(-5.29038f, 2.07007f, 9.12351f));
  points.push_back(Vector3(0.826136f, -3.6607f, 0.308542f));
  points.push_back(Vector3(-0.245674f, 8.71273f, 5.88122f));
  points.push_back(Vector3(-5.5327f, 1.78076f, -9.4293f));
  points.push_back(Vector3(-7.84234f, -6.91458f, -5.44359f));
  points.push_back(Vector3(3.4257f, 8.83663f, -9.04416f));
  points.push_back(Vector3(-3.67168f, -5.34288f, -2.58522f));
  points.push_back(Vector3(2.65542f, -3.85113f, 8.75607f));
  points.push_back(Vector3(-0.526444f, 8.29829f, 9.11801f));
  points.push_back(Vector3(4.10932f, -1.1948f, 9.64049f));
  points.push_back(Vector3(0.557573f, 2.95572f, 4.05316f));
  points.push_back(Vector3(4.66597f, -5.22629f, 7.62871f));
  points.push_back(Vector3(-7.51335f, -1.23386f, -6.00452f));
  points.push_back(Vector3(6.21204f, 5.52721f, 0.655233f));
  points.push_back(Vector3(3.14921f, 0.736412f, -3.3372f));
  points.push_back(Vector3(1.12827f, 6.06738f, 1.03977f));
  points.push_back(Vector3(2.36854f, 2.80252f, 8.77682f));
  points.push_back(Vector3(-2.58644f, -3.34269f, -1.04343f));
  points.push_back(Vector3(-4.52803f, -4.83016f, -0.103458f));
  points.push_back(Vector3(9.59899f, -7.17887f, 4.24055f));
  points.push_back(Vector3(2.49489f, -7.11661f, 8.20612f));
  points.push_back(Vector3(6.94327f, -5.8916f, 0.00762939f));
  points.push_back(Vector3(-2.46376f, -2.96182f, -1.0239f));
  points.push_back(Vector3(-8.29402f, 8.26533f, -6.80349f));
  points.push_back(Vector3(8.00653f, 4.47798f, -6.78701f));
  points.push_back(Vector3(0.00946045f, 8.25922f, 4.6556f));
  points.push_back(Vector3(0.849941f, -0.341502f, 0.423292f));
  points.push_back(Vector3(-1.53783f, -2.36366f, 7.37968f));
  points.push_back(Vector3(4.93576f, -3.84075f, 8.06818f));
  points.push_back(Vector3(-6.16443f, 7.35343f, -1.57018f));
  points.push_back(Vector3(8.03644f, 7.53471f, 3.61003f));
  points.push_back(Vector3(3.10831f, -2.86477f, 0.40498f));
  points.push_back(Vector3(-4.2204f, 0.385449f, 5.54186f));
  points.push_back(Vector3(9.66613f, -1.02451f, -5.37767f));
  points.push_back(Vector3(1.0831f, -0.893887f, -9.99756f));
  points.push_back(Vector3(0.882291f, -7.86615f, -7.09281f));
  points.push_back(Vector3(-1.60375f, -5.68224f, -8.04437f));
  points.push_back(Vector3(5.12803f, -9.53307f, 0.537431f));
  points.push_back(Vector3(0.100406f, -2.02246f, 6.9805f));
  points.push_back(Vector3(5.06882f, 6.51967f, -7.99615f));
  points.push_back(Vector3(5.0914f, -9.94995f, -6.01489f));
  points.push_back(Vector3(1.32054f, 5.16282f, -3.72417f));
  points.push_back(Vector3(4.46211f, -3.00089f, -6.90603f));
  points.push_back(Vector3(-1.28513f, 5.0444f, -8.25373f));
  points.push_back(Vector3(-1.21982f, -6.36769f, -0.904874f));
  points.push_back(Vector3(9.00266f, 1.32542f, 5.43931f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.65535f, 4.19477f, -8.06635f));
  points.push_back(Vector3(9.0228f, 9.33287f, 6.2035f));
  points.push_back(Vector3(-5.40025f, 5.14573f, -1.99072f));
  points.push_back(Vector3(0.788293f, -6.27979f, 4.97543f));
  points.push_back(Vector3(9.48363f, 5.85925f, 2.8843f));
  points.push_back(Vector3(-4.19721f, 7.71172f, 2.05115f));
  points.push_back(Vector3(-7.34672f, -1.47435f, 2.2367f));
  points.push_back(Vector3(-5.97095f, -0.836512f, 4.36079f));
  points.push_back(Vector3(-9.45006f, 5.88855f, -9.58983f));
  points.push_back(Vector3(-1.53111f, -0.281075f, -1.21555f));
  points.push_back(Vector3(5.86474f, 3.62041f, 7.88812f));
  points.push_back(Vector3(8.9172f, -4.97177f, 7.022f));
  points.push_back(Vector3(-3.70281f, -6.01856f, -9.43907f));
  points.push_back(Vector3(3.43608f, -7.57378f, 6.23402f));
  points.push_back(Vector3(1.6068f, -2.28797f, -5.0969f));
  points.push_back(Vector3(0.273751f, 2.93008f, -0.143742f));
  points.push_back(Vector3(0.786462f, -6.31886f, 5.51256f));
  points.push_back(Vector3(-4.23139f, 5.86535f, 8.63155f));
  points.push_back(Vector3(4.20331f, 6.76443f, -1.41514f));
  points.push_back(Vector3(-7.39006f, 8.081f, 1.01352f));
  points.push_back(Vector3(-1.76611f, 6.35487f, 9.12107f));
  points.push_back(Vector3(-9.98291f, -1.98645f, -2.85195f));
  points.push_back(Vector3(-5.18052f, -6.74917f, 3.61248f));
  points.push_back(Vector3(5.59008f, 0.383618f, 4.0904f));
  points.push_back(Vector3(7.91803f, 3.96527f, 6.11255f));
  points.push_back(Vector3(0.159001f, -7.51885f, -5.65539f));
  points.push_back(Vector3(-4.39253f, -8.78903f, -3.62774f));
  points.push_back(Vector3(4.53658f, 0.421461f, 7.67937f));
  points.push_back(Vector3(3.03079f, -9.99451f, 6.77786f));
  points.push_back(Vector3(3.00882f, 5.76281f, -0.0888091f));
  points.push_back(Vector3(9.99023f, -7.09098f, 2.48512f));
  points.push_back(Vector3(-4.64888f, -7.80267f, 4.64644f));
  points.push_back(Vector3(0.359813f, -2.02307f, 9.04721f));
  points.push_back(Vector3(-0.508744f, 0.932341f, -2.9313f));
  points.push_back(Vector3(0.945768f, 1.46764f, 6.40187f));
  points.push_back(Vector3(-6.77541f, -5.4912f, -4.49629f));
  points.push_back(Vector3(-2.34657f, 6.76992f, 9.82849f));
  points.push_back(Vector3(8.1341f, 4.66964f, 9.01608f));
  points.push_back(Vector3(-0.81637f, -6.14856f, -0.0509661f));
  points.push_back(Vector3(4.50545f, -7.17032f, 0.265816f));
  points.push_back(Vector3(-4.43587f, 2.34657f, -2.52724f));
  points.push_back(Vector3(-4.84298f, -5.5327f, 4.78378f));
  points.push_back(Vector3(3.06986f, 8.58577f, -3.31095f));
  points.push_back(Vector3(-8.88607f, 3.57524f, -6.84927f));
  points.push_back(Vector3(-8.96603f, 5.08286f, 7.25394f));
  points.push_back(Vector3(-4.44624f, -7.07755f, 3.88165f));
  points.push_back(Vector3(2.19153f, 2.56142f, -1.02206f));
  points.push_back(Vector3(-3.13211f, -7.37358f, -5.48509f));
  points.push_back(Vector3(-0.99765f, -7.85577f, -8.27509f));
  points.push_back(Vector3(9.50682f, -4.03912f, -0.893277f));
  points.push_back(Vector3(6.42628f, -8.66573f, -5.4033f));
  points.push_back(Vector3(5.09751f, -0.0143439f, 4.38398f));
  points.push_back(Vector3(9.3054f, -5.71459f, 7.83563f));
  points.push_back(Vector3(8.53877f, -8.11151f, 6.89688f));
  points.push_back(Vector3(-7.60674f, -2.73843f, 9.82238f));
  points.push_back(Vector3(6.92251f, -7.56279f, -8.0401f));
  points.push_back(Vector3(-4.3498f, -3.57219f, -7.31925f));
  points.push_back(Vector3(-4.23078f, -1.8601f, 4.58724f));
  points.push_back(Vector3(9.16562f, -0.127262f, -4.17829f));
  points.push_back(Vector3(9.57213f, -3.9024f, 5.48753f));
  points.push_back(Vector3(3.53191f, 1.41331f, 9.76012f));
  points.push_back(Vector3(-5.36363f, -7.40837f, 3.96954f));
  points.push_back(Vector3(-4.88754f, -7.16117f, -4.7557f));
  points.push_back(Vector3(4.11542f, -9.28159f, -3.16446f));
  points.push_back(Vector3(-6.02832f, 6.28712f, -2.38502f));
  points.push_back(Vector3(-9.51964f, 6.32435f, 0.50264f));
  points.push_back(Vector3(7.75079f, 1.92053f, 5.87695f));
  points.push_back(Vector3(1.98279f, -2.3777f, 5.1915f));
  points.push_back(Vector3(9.94934f, -9.50194f, 2.73049f));
  points.push_back(Vector3(-0.65218f, -5.2971f, 7.57195f));
  points.push_back(Vector3(-1.52135f, 8.26472f, 8.25373f));
  points.push_back(Vector3(-0.347606f, 3.1486f, -5.77929f));
  points.push_back(Vector3(2.40577f, 5.39293f, 8.56136f));
  points.push_back(Vector3(8.39961f, -6.34327f, -4.84359f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.45433f, -6.48793f, -0.339671f));
  points.push_back(Vector3(-0.497146f, 4.83016f, 5.1323f));
  points.push_back(Vector3(-7.17582f, 1.59886f, -3.21818f));
  points.push_back(Vector3(5.53392f, 9.92126f, -5.80493f));
  points.push_back(Vector3(-9.02829f, -3.87066f, -8.66939f));
  points.push_back(Vector3(-7.71172f, 3.98175f, -7.54021f));
  points.push_back(Vector3(4.58357f, 0.839564f, 8.28059f));
  points.push_back(Vector3(5.05478f, 4.27045f, 7.53166f));
  points.push_back(Vector3(-3.54167f, -0.759606f, -4.87777f));
  points.push_back(Vector3(6.02954f, -6.06861f, 9.52025f));
  points.push_back(Vector3(-8.61995f, -9.61852f, -1.74596f));
  points.push_back(Vector3(-5.85925f, -4.4261f, -8.82748f));
  points.push_back(Vector3(-6.84988f, 5.49242f, 9.84313f));
  points.push_back(Vector3(-5.93921f, -4.59456f, 1.94128f));
  points.push_back(Vector3(2.92398f, -8.6755f, 2.72744f));
  points.push_back(Vector3(-3.25846f, 5.92273f, 7.0397f));
  points.push_back(Vector3(0.710166f, 9.64049f, 0.00213623f));
  points.push_back(Vector3(6.51357f, -2.96854f, -3.20231f));
  points.push_back(Vector3(9.1113f, -9.39146f, 0.990936f));
  points.push_back(Vector3(4.66109f, 0.871303f, 5.56688f));
  points.push_back(Vector3(-3.46904f, 3.40129f, 8.764f));
  points.push_back(Vector3(8.96603f, 6.86148f, 5.5504f));
  points.push_back(Vector3(-1.2833f, 2.44301f, -1.19663f));
  points.push_back(Vector3(5.4564f, -6.75588f, -8.21161f));
  points.push_back(Vector3(4.99374f, 5.18418f, -3.76141f));
  points.push_back(Vector3(5.05844f, -7.94488f, 2.48695f));
  points.push_back(Vector3(-8.43806f, 9.31211f, -7.11844f));
  points.push_back(Vector3(6.47084f, -1.45787f, -3.38664f));
  points.push_back(Vector3(-8.86471f, -3.41472f, -2.48573f));
  points.push_back(Vector3(-0.0894195f, -3.89203f, -9.44761f));
  points.push_back(Vector3(8.52657f, 9.42442f, -3.94879f));
  points.push_back(Vector3(-3.12967f, -3.67229f, -9.82788f));
  points.push_back(Vector3(8.11029f, -2.49001f, 4.77828f));
  points.push_back(Vector3(-8.7524f, -7.01468f, -3.56426f));
  points.push_back(Vector3(-6.20472f, 7.48039f, -7.55669f));
  points.push_back(Vector3(-0.434889f, 1.96081f, 4.11054f));
  points.push_back(Vector3(-2.19092f, 1.80639f, 7.92169f));
  points.push_back(Vector3(5.08835f, 7.55058f, 6.78457f));
  points.push_back(Vector3(-6.29688f, -4.36262f, 2.10059f));
  points.push_back(Vector3(-5.10971f, 3.0607f, -0.300607f));
  points.push_back(Vector3(-5.97766f, 5.28001f, -2.00537f));
  points.push_back(Vector3(2.88125f, 1.65807f, 7.43278f));
  points.push_back(Vector3(6.55751f, 1.54515f, 4.36934f));
  points.push_back(Vector3(7.91681f, 5.07126f, 8.64681f));
  points.push_back(Vector3(7.51701f, 7.96686f, 7.88812f));
  points.push_back(Vector3(-3.03751f, -4.71847f, -2.48878f));
  points.push_back(Vector3(7.80877f, -9.67772f, -0.534379f));
  points.push_back(Vector3(1.26865f, 0.299387f, 3.47392f));
  points.push_back(Vector3(2.39845f, 2.53822f, -5.38743f));
  points.push_back(Vector3(1.20762f, 2.0603f, 0.312204f));
  points.push_back(Vector3(4.95346f, -2.63771f, -2.42897f));
  points.push_back(Vector3(5.3325f, 4.8088f, 4.34004f));
  points.push_back(Vector3(3.14066f, -9.2822f, -2.43629f));
  points.push_back(Vector3(-6.24805f, 8.63216f, 2.78909f));
  points.push_back(Vector3(6.53432f, -1.17588f, -5.15488f));
  points.push_back(Vector3(7.70989f, -2.46193f, -0.814539f));
  points.push_back(Vector3(1.65624f, -0.426954f, -2.8721f));
  points.push_back(Vector3(-1.33946f, -2.87149f, 5.56566f));
  points.push_back(Vector3(-0.381787f, -3.29875f, 5.72985f));
  points.push_back(Vector3(-6.18397f, 6.02039f, 4.07086f));
  points.push_back(Vector3(3.77483f, -0.426344f, 8.02179f));
  points.push_back(Vector3(3.41349f, 0.215156f, -4.67696f));
  points.push_back(Vector3(0.125431f, 5.33006f, -9.08567f));
  points.push_back(Vector3(0.481277f, -6.91824f, 0.588092f));
  points.push_back(Vector3(4.51582f, -3.72539f, -8.26899f));
  points.push_back(Vector3(-0.909147f, 8.04132f, 1.45115f));
  points.push_back(Vector3(-9.61242f, -3.77667f, -9.6588f));
  points.push_back(Vector3(-5.57176f, 8.88852f, -2.78542f));
  points.push_back(Vector3(-8.47652f, -2.48512f, 2.42592f));
  points.push_back(Vector3(0.126041f, -3.52336f, 8.82687f));
  points.push_back(Vector3(8.30317f, 8.09931f, 0.625935f));
  points.push_back(Vector3(-2.47658f, 2.7195f, -8.51131f));
  points.push_back(Vector3(6.04236f, 7.9931f, -9.27305f));
  points.push_back(Vector3(-9.49706f, -3.98053f, 8.60225f));
  points.push_back(Vector3(2.4723f, -1.02817f, 6.26453f));
  points.push_back(Vector3(-9.35789f, -1.15024f, -0.119938f));
  points.push_back(Vector3(0.8005f, -6.89749f, -0.296335f));
  points.push_back(Vector3(-6.72292f, -9.26267f, -1.12094f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.79553f, -8.30073f, -4.82589f));
  points.push_back(Vector3(8.01202f, 8.29707f, -4.70138f));
  points.push_back(Vector3(7.97784f, 7.11905f, -6.53493f));
  points.push_back(Vector3(7.20878f, 5.35813f, 9.59838f));
  points.push_back(Vector3(0.334789f, 9.18821f, -4.86801f));
  points.push_back(Vector3(2.70974f, 4.98459f, 9.4171f));
  points.push_back(Vector3(-6.48915f, 5.56749f, -3.15714f));
  points.push_back(Vector3(9.30723f, -2.82937f, -4.58296f));
  points.push_back(Vector3(-7.65618f, 5.03464f, -8.85311f));
  points.push_back(Vector3(6.46046f, -9.035f, 1.92846f));
  points.push_back(Vector3(-1.82836f, 4.33699f, 4.27045f));
  points.push_back(Vector3(9.8352f, 5.08347f, 9.40184f));
  points.push_back(Vector3(-1.40782f, -2.41737f, -8.07001f));
  points.push_back(Vector3(-8.09137f, 1.96326f, -5.3264f));
  points.push_back(Vector3(-0.603961f, -7.75811f, -4.0611f));
  points.push_back(Vector3(-1.67211f, 0.166326f, 9.30418f));
  points.push_back(Vector3(2.92215f, -5.99841f, 9.87243f));
  points.push_back(Vector3(-9.80285f, 9.62401f, 7.51274f));
  points.push_back(Vector3(3.22367f, 4.31806f, -2.72134f));
  points.push_back(Vector3(-3.11808f, -9.03561f, -5.01083f));
  points.push_back(Vector3(-0.754113f, 2.04199f, 3.27494f));
  points.push_back(Vector3(-8.24091f, -3.38115f, 0.504471f));
  points.push_back(Vector3(-4.75814f, -3.51299f, -7.53655f));
  points.push_back(Vector3(7.1453f, 0.0253308f, 6.34327f));
  points.push_back(Vector3(-4.68246f, 6.20289f, 5.34532f));
  points.push_back(Vector3(6.84805f, -7.42851f, -6.36097f));
  points.push_back(Vector3(-1.08615f, -1.92175f, 4.36079f));
  points.push_back(Vector3(-2.03101f, -1.56102f, 7.1569f));
  points.push_back(Vector3(-1.41881f, -2.30445f, -8.1811f));
  points.push_back(Vector3(-9.84008f, -2.32276f, 3.46965f));
  points.push_back(Vector3(3.16446f, -3.43364f, -2.20313f));
  points.push_back(Vector3(3.10404f, -9.74181f, -0.638142f));
  points.push_back(Vector3(-9.03378f, -2.43934f, 9.09299f));
  points.push_back(Vector3(3.94147f, -9.10703f, 6.04114f));
  points.push_back(Vector3(-8.905f, -1.07761f, -6.97501f));
  points.push_back(Vector3(8.30439f, 8.67977f, 1.99499f));
  points.push_back(Vector3(9.49644f, 2.33375f, -5.26048f));
  points.push_back(Vector3(3.93902f, 9.57762f, 5.48326f));
  points.push_back(Vector3(-8.28486f, 3.93048f, 6.25965f));
  points.push_back(Vector3(-2.77566f, -4.14716f, -0.741905f));
  points.push_back(Vector3(-0.400097f, -8.38252f, -7.71783f));
  points.push_back(Vector3(2.584f, -2.55776f, -2.36061f));
  points.push_back(Vector3(6.54836f, 9.8645f, 4.15326f));
  points.push_back(Vector3(-4.51949f, 8.20307f, -7.93756f));
  points.push_back(Vector3(-2.0127f, 6.95181f, -7.90216f));
  points.push_back(Vector3(-6.2273f, 7.70928f, 8.11152f));
  points.push_back(Vector3(-7.00308f, -8.11396f, 9.10092f));
  points.push_back(Vector3(2.24769f, -0.867641f, -2.58095f));
  points.push_back(Vector3(-8.02423f, 1.33457f, 0.332958f));
  points.push_back(Vector3(-6.37135f, -1.67272f, -5.90747f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.19553f, 6.35182f, -2.89468f));
  points.push_back(Vector3(8.82382f, 3.24747f, -3.29875f));
  points.push_back(Vector3(-0.215766f, 9.33226f, 6.90542f));
  points.push_back(Vector3(-2.28919f, -1.81494f, -2.30995f));
  points.push_back(Vector3(7.59697f, 9.00998f, -7.81671f));
  points.push_back(Vector3(7.99127f, -8.12616f, -5.57787f));
  points.push_back(Vector3(9.84375f, 7.42119f, -5.94836f));
  points.push_back(Vector3(-6.59963f, -4.36811f, 4.10932f));
  points.push_back(Vector3(-4.85885f, -3.29264f, 5.86474f));
  points.push_back(Vector3(-6.69607f, 6.83035f, 6.98782f));
  points.push_back(Vector3(9.49278f, 7.49138f, 3.62163f));
  points.push_back(Vector3(-5.15854f, 2.37953f, 6.08692f));
  points.push_back(Vector3(-3.73455f, -8.96298f, -6.83218f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.29035f, 4.73128f, 7.26066f));
  points.push_back(Vector3(-3.24198f, 1.41209f, 4.79659f));
  points.push_back(Vector3(2.24342f, -1.81433f, 2.26051f));
  points.push_back(Vector3(-5.91601f, -2.80007f, 9.89563f));
  points.push_back(Vector3(2.27027f, -6.01001f, 1.62328f));
  points.push_back(Vector3(-7.63054f, 3.12967f, 0.0442517f));
  points.push_back(Vector3(-2.86538f, 6.09851f, -2.71645f));
  points.push_back(Vector3(3.16202f, -3.76507f, 7.8515f));
  points.push_back(Vector3(1.62023f, 1.09104f, -9.54589f));
  points.push_back(Vector3(-1.29063f, -4.12397f, -4.31928f));
  points.push_back(Vector3(1.05319f, 4.47493f, 3.90545f));
  points.push_back(Vector3(1.5952f, -4.62264f, -0.907315f));
  points.push_back(Vector3(5.91235f, 2.0951f, 2.01147f));
  points.push_back(Vector3(-8.29585f, -0.204779f, -2.48024f));
  points.push_back(Vector3(7.83868f, 7.95892f, -0.2884f));
  points.push_back(Vector3(2.08533f, -9.85717f, 6.20472f));
  points.push_back(Vector3(9.99878f, -5.37278f, 6.87979f));
  points.push_back(Vector3(-0.805383f, 2.56508f, -1.08066f));
  points.push_back(Vector3(-1.77709f, 1.91809f, -4.10932f));
  points.push_back(Vector3(-0.819422f, 2.5425f, -3.80718f));
  points.push_back(Vector3(3.74981f, 6.40614f, -0.0930816f));
  points.push_back(Vector3(-7.35771f, -8.21833f, 1.14475f));
  points.push_back(Vector3(6.25782f, 0.677205f, -9.95361f));
  points.push_back(Vector3(8.34407f, -8.6285f, 7.11051f));
  points.push_back(Vector3(-0.323802f, 9.89257f, 3.73577f));
  points.push_back(Vector3(-5.96728f, 1.75329f, -9.52025f));
  points.push_back(Vector3(-4.54085f, -3.43303f, 6.54408f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.89319f, -9.75585f, 0.906705f));
  points.push_back(Vector3(8.34529f, -0.183416f, -2.13904f));
  points.push_back(Vector3(-5.10727f, -5.33433f, -0.350658f));
  points.push_back(Vector3(-8.92026f, 9.78149f, 8.28059f));
  points.push_back(Vector3(-1.7777f, -5.66698f, -8.57417f));
  points.push_back(Vector3(4.17585f, 2.93252f, -1.6306f));
  points.push_back(Vector3(-1.24058f, 2.00964f, -4.56343f));
  points.push_back(Vector3(6.3152f, 8.11945f, 5.62975f));
  points.push_back(Vector3(-4.3907f, -5.70299f, -6.13269f));
  points.push_back(Vector3(-6.28224f, -7.89727f, -7.73247f));
  points.push_back(Vector3(8.42219f, -7.8045f, 7.63237f));
  points.push_back(Vector3(0.647908f, -5.17441f, 9.0112f));
  points.push_back(Vector3(-2.8898f, 8.41609f, -5.01511f));
  points.push_back(Vector3(1.93579f, 6.76626f, 8.26411f));
  points.push_back(Vector3(6.68935f, -0.00946045f, -7.64397f));
  points.push_back(Vector3(-5.54491f, -7.44804f, 9.13449f));
  points.push_back(Vector3(4.60921f, -1.00558f, -6.01306f));
  points.push_back(Vector3(2.83181f, 0.015564f, -8.24824f));
  points.push_back(Vector3(-6.6509f, 8.12922f, 5.10361f));
  points.push_back(Vector3(0.150456f, -2.63466f, 7.86492f));
  points.push_back(Vector3(-7.69524f, -5.95019f, 4.97726f));
  points.push_back(Vector3(9.63622f, -4.44441f, -8.54366f));
  points.push_back(Vector3(-7.2747f, 2.14942f, 3.74615f));
  points.push_back(Vector3(-9.33042f, 0.798669f, 9.57823f));
  points.push_back(Vector3(0.147405f, 4.50667f, -7.13309f));
  points.push_back(Vector3(4.55428f, 5.93616f, 2.21534f));
  points.push_back(Vector3(-7.16666f, -9.58556f, -4.04645f));
  points.push_back(Vector3(-5.28611f, -8.37458f, -6.66494f));
  points.push_back(Vector3(4.8088f, 8.77377f, 7.14591f));
  points.push_back(Vector3(-5.03159f, -8.53999f, 5.76769f));
  points.push_back(Vector3(-9.48363f, 6.2157f, 0.401928f));
  points.push_back(Vector3(-9.37925f, 1.73559f, -7.24479f));
  points.push_back(Vector3(7.57439f, 6.58132f, 7.87286f));
  points.push_back(Vector3(-7.04093f, 9.12412f, 1.42491f));
  points.push_back(Vector3(6.92984f, -1.19053f, 1.14536f));
  points.push_back(Vector3(0.74984f, -9.51231f, 0.4532f));
  points.push_back(Vector3(8.51802f, -6.05396f, 8.27265f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.52452f, 3.90118f, -1.95959f));
  points.push_back(Vector3(5.45518f, 4.48714f, 1.24851f));
  points.push_back(Vector3(6.33534f, 4.13251f, 4.05622f));
  points.push_back(Vector3(-3.12906f, -6.92007f, -6.74612f));
  points.push_back(Vector3(-1.9657f, 2.66213f, -2.74514f));
  points.push_back(Vector3(1.90649f, -8.91293f, -1.14536f));
  points.push_back(Vector3(5.47533f, -1.70141f, 4.41938f));
  points.push_back(Vector3(-5.65416f, 3.51909f, -2.31666f));
  points.push_back(Vector3(4.50545f, 2.99966f, -0.0772119f));
  points.push_back(Vector3(-5.92944f, 0.459304f, -5.95202f));
  points.push_back(Vector3(7.27348f, -1.1478f, 0.655233f));
  points.push_back(Vector3(4.72152f, -3.18155f, -0.534989f));
  points.push_back(Vector3(9.23521f, -6.76992f, -7.97601f));
  points.push_back(Vector3(1.22288f, -6.87796f, 0.219429f));
  points.push_back(Vector3(-3.29569f, 5.43077f, 0.168157f));
  points.push_back(Vector3(-3.16752f, 3.34086f, 6.91824f));
  points.push_back(Vector3(-0.942106f, 7.4926f, -2.77261f));
  points.push_back(Vector3(4.10932f, 1.59764f, 5.54064f));
  points.push_back(Vector3(2.05786f, 4.75997f, -1.99133f));
  points.push_back(Vector3(9.8529f, 2.58644f, 0.477614f));
  points.push_back(Vector3(5.85864f, 0.549638f, 9.97986f));
  points.push_back(Vector3(-6.83645f, 3.24442f, 1.94433f));
  points.push_back(Vector3(5.4912f, 6.48732f, 6.8627f));
  points.push_back(Vector3(-9.16929f, 8.08283f, 5.52782f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.39595f, -2.96548f, 9.37925f));
  points.push_back(Vector3(-1.65746f, 7.74956f, -2.22755f));
  points.push_back(Vector3(1.5598f, -7.12149f, 8.13349f));
  points.push_back(Vector3(-7.03482f, 1.98523f, 8.47041f));
  points.push_back(Vector3(6.10828f, -1.26133f, 3.73699f));
  points.push_back(Vector3(-8.49422f, -4.39619f, 4.01349f));
  points.push_back(Vector3(-2.08655f, 5.10727f, 8.00775f));
  points.push_back(Vector3(2.37526f, -9.47447f, 0.939054f));
  points.push_back(Vector3(9.23398f, 3.3549f, 1.15207f));
  points.push_back(Vector3(5.74389f, -7.38273f, -3.74798f));
  points.push_back(Vector3(3.97687f, 0.730308f, -6.74429f));
  points.push_back(Vector3(9.6173f, 1.83386f, 1.93579f));
  points.push_back(Vector3(-0.588092f, -0.473342f, -6.11377f));
  points.push_back(Vector3(-0.307932f, 6.62282f, 9.7705f));
  points.push_back(Vector3(7.56401f, 5.31907f, -9.87793f));
  points.push_back(Vector3(-2.92032f, -4.62325f, 3.16385f));
  points.push_back(Vector3(1.21006f, 4.51766f, 7.24845f));
  points.push_back(Vector3(-3.87432f, 3.19315f, -3.84136f));
  points.push_back(Vector3(-1.02878f, 2.77505f, -7.48222f));
  points.push_back(Vector3(-3.81573f, 7.42546f, -5.79943f));
  points.push_back(Vector3(0.731529f, 5.51683f, -6.82363f));
  points.push_back(Vector3(8.71273f, 2.87637f, -0.887173f));
  points.push_back(Vector3(1.08737f, -7.48466f, 7.96503f));
  points.push_back(Vector3(-2.94534f, -0.444655f, 4.48714f));
  points.push_back(Vector3(-2.85501f, -7.45293f, 9.01425f));
  points.push_back(Vector3(-6.81631f, 5.18845f, 9.96033f));
  points.push_back(Vector3(0.310984f, 4.19599f, -9.07956f));
  points.push_back(Vector3(-3.97137f, -6.40065f, 5.9801f));
  points.push_back(Vector3(-7.77642f, -2.65053f, -1.64037f));
  points.push_back(Vector3(-7.70806f, -8.764f, 7.28446f));
  points.push_back(Vector3(5.17808f, 9.35118f, 5.36851f));
  points.push_back(Vector3(-4.59273f, 9.9115f, 5.84765f));
  points.push_back(Vector3(9.76745f, 2.61696f, 0.854824f));
  points.push_back(Vector3(-6.06555f, 6.85049f, -4.97055f));
  points.push_back(Vector3(9.54894f, -0.681478f, 8.53328f));
  points.push_back(Vector3(-0.202338f, -4.16608f, 8.57479f));
  points.push_back(Vector3(6.7803f, -4.04096f, 7.80267f));
  points.push_back(Vector3(-8.18598f, -1.901f, 3.45561f));
  points.push_back(Vector3(7.24479f, 8.83419f, -2.39418f));
  points.push_back(Vector3(3.43303f, 1.23386f, -3.14127f));
  points.push_back(Vector3(8.28364f, -7.0336f, -9.53001f));
  points.push_back(Vector3(8.01508f, 1.06479f, 7.31681f));
  points.push_back(Vector3(-4.99374f, -5.34165f, -8.64437f));
  points.push_back(Vector3(-7.1514f, 7.97845f, -9.3292f));
  points.push_back(Vector3(7.63604f, 3.7492f, -2.02918f));
  points.push_back(Vector3(6.86819f, -2.25562f, 5.64745f));
  points.push_back(Vector3(-9.35545f, 8.62423f, 8.49605f));
  points.push_back(Vector3(-0.718101f, -9.76134f, -6.1449f));
  points.push_back(Vector3(4.23566f, 7.32902f, -8.41304f));
  points.push_back(Vector3(-0.521562f, -0.0314343f, -2.48085f));
  points.push_back(Vector3(3.26273f, -5.5327f, -5.32029f));
  points.push_back(Vector3(5.17136f, -7.51335f, 7.6397f));
  points.push_back(Vector3(0.480666f, 8.25983f, -7.58782f));
  points.push_back(Vector3(8.29524f, -6.80654f, -1.84668f));
  points.push_back(Vector3(2.42225f, 7.82769f, 0.0143433f));
  points.push_back(Vector3(-0.863979f, 2.05542f, 0.440382f));
  points.push_back(Vector3(9.86267f, -0.152898f, 9.19492f));
  points.push_back(Vector3(-8.90927f, 2.05908f, 9.75646f));
  points.push_back(Vector3(3.13334f, 8.7817f, -1.05564f));
  points.push_back(Vector3(3.80718f, 3.15531f, -6.67226f));
  points.push_back(Vector3(-1.93335f, 8.34407f, 3.24137f));
  points.push_back(Vector3(-6.26392f, 9.57213f, -8.53938f));
  points.push_back(Vector3(-6.4629f, 6.29627f, -5.59679f));
  points.push_back(Vector3(-9.12107f, 9.20469f, -9.46837f));
  points.push_back(Vector3(8.9581f, 2.50038f, -2.72195f));
  points.push_back(Vector3(-8.86959f, -8.19758f, 2.73904f));
  points.push_back(Vector3(2.15796f, 3.9671f, -1.90466f));
  points.push_back(Vector3(7.77337f, 0.35432f, -5.63951f));
  points.push_back(Vector3(-2.68105f, 2.83731f, 5.12314f));
  points.push_back(Vector3(5.67919f, -0.0363171f, 4.67147f));
  points.push_back(Vector3(-8.39717f, 6.42323f, 8.78109f));
  points.push_back(Vector3(-9.73266f, 3.08756f, -6.09119f));
  points.push_back(Vector3(0.229194f, -4.97665f, 9.05576f));
  points.push_back(Vector3(2.68044f, -8.25678f, -6.55873f));
  points.push_back(Vector3(-4.41755f, -4.23139f, 0.449538f));
  points.push_back(Vector3(5.48448f, -1.23814f, 6.93411f));
  points.push_back(Vector3(8.94345f, 1.72948f, 1.75939f));
  points.push_back(Vector3(-7.99677f, 3.77606f, -9.57396f));
  points.push_back(Vector3(6.12171f, -2.55959f, 2.24891f));
  points.push_back(Vector3(-0.396435f, -2.02918f, 4.26069f));
  points.push_back(Vector3(-6.84133f, -2.67006f, -0.292062f));
  points.push_back(Vector3(-4.57137f, 7.11539f, 3.27494f));
  points.push_back(Vector3(7.84661f, 4.85397f, 2.04749f));
  points.push_back(Vector3(-5.3264f, 7.60186f, -3.88592f));
  points.push_back(Vector3(-0.462356f, -0.162663f, 4.87167f));
  points.push_back(Vector3(9.02951f, -2.40455f, 2.32582f));
  TestSphereCentroid(points, file);
}

void SphereCentroidTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.41465f, 4.71847f, -2.27149f));
  points.push_back(Vector3(-6.75588f, -9.25962f, -6.93411f));
  points.push_back(Vector3(-9.1821f, 5.86596f, -1.66112f));
  points.push_back(Vector3(-9.51598f, 8.11579f, -5.26719f));
  points.push_back(Vector3(1.22166f, -1.88879f, 2.23121f));
  points.push_back(Vector3(3.93475f, -8.65047f, -2.62307f));
  points.push_back(Vector3(6.93106f, 8.00653f, 7.62993f));
  points.push_back(Vector3(-5.96973f, 7.55669f, -6.84133f));
  points.push_back(Vector3(-5.28306f, -5.76464f, -4.76363f));
  points.push_back(Vector3(7.14835f, -4.08429f, -1.63915f));
  points.push_back(Vector3(8.01324f, 8.39839f, 8.31111f));
  points.push_back(Vector3(8.09503f, -7.99493f, -4.99435f));
  points.push_back(Vector3(-1.75512f, -7.50114f, 5.73901f));
  points.push_back(Vector3(4.25459f, 5.28245f, 8.63948f));
  points.push_back(Vector3(0.763268f, 3.26457f, -7.2393f));
  points.push_back(Vector3(2.40455f, 1.68371f, 1.39439f));
  points.push_back(Vector3(-6.91702f, 5.15366f, -1.26926f));
  points.push_back(Vector3(5.02976f, 5.85131f, -8.19819f));
  points.push_back(Vector3(-4.00189f, 5.3795f, -3.01981f));
  points.push_back(Vector3(-4.0202f, 8.46309f, -0.0967437f));
  points.push_back(Vector3(-3.08512f, 7.88751f, -9.07407f));
  points.push_back(Vector3(-2.00415f, 6.07105f, -9.53246f));
  points.push_back(Vector3(-8.01141f, -8.43989f, 5.62731f));
  points.push_back(Vector3(-0.472121f, -3.80657f, 1.08005f));
  points.push_back(Vector3(1.79785f, 3.42753f, 7.45476f));
  points.push_back(Vector3(-2.16895f, 1.70263f, 7.96625f));
  points.push_back(Vector3(-3.09915f, -7.15201f, 8.96603f));
  points.push_back(Vector3(2.08533f, 8.26167f, -6.72475f));
  points.push_back(Vector3(-2.83486f, -9.97742f, 3.91217f));
  points.push_back(Vector3(-3.27677f, 9.2584f, 5.53331f));
  points.push_back(Vector3(4.70931f, 4.26191f, 1.11057f));
  points.push_back(Vector3(-6.198f, 6.74551f, 2.66518f));
  points.push_back(Vector3(-2.75002f, -3.31462f, 8.13349f));
  points.push_back(Vector3(-3.45378f, -3.02774f, 6.89138f));
  points.push_back(Vector3(8.96542f, 0.995818f, -1.07456f));
  points.push_back(Vector3(6.73086f, -2.45094f, -4.64827f));
  points.push_back(Vector3(2.29041f, -2.93374f, 6.07166f));
  points.push_back(Vector3(8.73165f, -7.30644f, -7.91803f));
  points.push_back(Vector3(8.79025f, -2.05725f, -9.02951f));
  points.push_back(Vector3(5.49486f, 5.89221f, 8.40571f));
  points.push_back(Vector3(-7.09769f, 3.83709f, 6.14856f));
  points.push_back(Vector3(-1.2949f, -1.9425f, -7.56279f));
  points.push_back(Vector3(-8.52962f, -0.597858f, -4.1435f));
  points.push_back(Vector3(9.72228f, -3.48247f, 6.72658f));
  points.push_back(Vector3(6.99454f, -4.54695f, 6.45985f));
  points.push_back(Vector3(1.78015f, 8.33125f, 8.5522f));
  points.push_back(Vector3(6.03259f, -4.12946f, -0.608234f));
  points.push_back(Vector3(0.101627f, 3.00943f, -7.39677f));
  points.push_back(Vector3(-1.41209f, -2.81777f, -0.793176f));
  points.push_back(Vector3(7.35771f, 7.2924f, 3.14493f));
  points.push_back(Vector3(-7.84112f, -5.2324f, 9.89196f));
  points.push_back(Vector3(9.95911f, 0.578326f, 4.84115f));
  points.push_back(Vector3(7.93695f, 8.25251f, -3.27067f));
  points.push_back(Vector3(-1.67028f, -3.47514f, -9.32554f));
  points.push_back(Vector3(-1.15329f, 1.98828f, 1.02878f));
  points.push_back(Vector3(-5.79638f, 6.7101f, -5.66393f));
  points.push_back(Vector3(2.30445f, -2.97769f, -6.48549f));
  points.push_back(Vector3(2.55592f, 8.50093f, -4.4084f));
  points.push_back(Vector3(-2.84829f, 1.14841f, 1.59459f));
  points.push_back(Vector3(-2.45521f, 4.02936f, -7.03116f));
  points.push_back(Vector3(-3.22001f, -3.97565f, 8.5577f));
  points.push_back(Vector3(3.27494f, 2.82754f, 1.09104f));
  points.push_back(Vector3(1.42186f, 7.40043f, 1.55431f));
  points.push_back(Vector3(-6.17115f, 9.77233f, 2.69509f));
  points.push_back(Vector3(-5.48692f, -1.23142f, 2.4485f));
  points.push_back(Vector3(5.34471f, -4.96262f, 0.256661f));
  points.push_back(Vector3(3.37321f, 2.69204f, -6.24012f));
  points.push_back(Vector3(-1.60375f, 6.23158f, 1.70263f));
  points.push_back(Vector3(-4.26557f, -9.81811f, -6.43727f));
  points.push_back(Vector3(4.17707f, -2.71401f, -8.54305f));
  points.push_back(Vector3(-6.60146f, 9.35423f, 0.707114f));
  points.push_back(Vector3(5.31053f, -3.4434f, -5.48204f));
  points.push_back(Vector3(-5.06211f, -7.44865f, 7.72698f));
  points.push_back(Vector3(-5.76159f, 0.340892f, 9.87671f));
  points.push_back(Vector3(-6.52577f, 8.8702f, 5.70605f));
  points.push_back(Vector3(-8.64803f, 0.277413f, 2.93252f));
  points.push_back(Vector3(-4.39192f, -1.14841f, 5.73229f));
  points.push_back(Vector3(4.00189f, -3.76202f, -2.94595f));
  points.push_back(Vector3(-3.95734f, 2.72866f, -2.16285f));
  points.push_back(Vector3(-1.35594f, -7.06839f, 4.62264f));
  points.push_back(Vector3(1.85766f, -4.67025f, 3.08267f));
  points.push_back(Vector3(-5.78906f, -9.02585f, 0.30427f));
  points.push_back(Vector3(-6.74856f, 2.2129f, -9.92431f));
  points.push_back(Vector3(-2.21778f, 4.16608f, -3.66192f));
  points.push_back(Vector3(-3.64849f, 5.22691f, 6.33778f));
  points.push_back(Vector3(6.35182f, 6.61183f, 3.89874f));
  points.push_back(Vector3(-9.08689f, 4.01593f, -0.0473034f));
  points.push_back(Vector3(3.35185f, -1.30711f, 8.54305f));
  TestSphereCentroid(points, file);
}

void SphereRitterTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.97681f, -5.28855f, 2.96304f));
  points.push_back(Vector3(-8.51253f, -4.59517f, -2.79946f));
  points.push_back(Vector3(-4.89425f, 9.7058f, -3.62163f));
  points.push_back(Vector3(8.68465f, -6.4275f, 7.1514f));
  TestSphereRitter(points, file);
}

void SphereRitterTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.96704f, 7.51335f, -2.51991f));
  points.push_back(Vector3(4.92294f, 6.88284f, 9.18394f));
  points.push_back(Vector3(4.60433f, 0.772424f, 6.76809f));
  points.push_back(Vector3(9.92248f, -6.87857f, 7.28629f));
  points.push_back(Vector3(-1.73986f, 0.666829f, -9.73998f));
  points.push_back(Vector3(-7.37114f, -2.69814f, 2.27271f));
  points.push_back(Vector3(-2.584f, -9.24863f, -2.36f));
  points.push_back(Vector3(-3.29081f, 9.03928f, 7.21915f));
  TestSphereRitter(points, file);
}

void SphereRitterTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.62767f, 2.76528f, -8.93796f));
  points.push_back(Vector3(1.72704f, -2.88003f, -3.37931f));
  points.push_back(Vector3(7.5518f, -2.96121f, 0.00640869f));
  points.push_back(Vector3(-8.00043f, -2.2251f, -8.14386f));
  points.push_back(Vector3(-4.61531f, -2.49001f, -7.08609f));
  points.push_back(Vector3(-8.37458f, -0.0698876f, -5.93066f));
  points.push_back(Vector3(7.37114f, -8.3343f, 2.11035f));
  points.push_back(Vector3(3.0076f, 6.9744f, -4.94797f));
  points.push_back(Vector3(1.03183f, -7.98761f, 3.62651f));
  points.push_back(Vector3(-8.83969f, 3.29936f, 7.06107f));
  points.push_back(Vector3(-7.97174f, 8.00836f, -1.49205f));
  points.push_back(Vector3(8.44172f, -5.90381f, 1.49998f));
  points.push_back(Vector3(3.98907f, -0.987884f, 6.69485f));
  points.push_back(Vector3(-5.2794f, -0.556352f, 2.80496f));
  points.push_back(Vector3(-1.67821f, -9.61242f, 2.08045f));
  points.push_back(Vector3(2.11035f, -9.63805f, 9.59533f));
  points.push_back(Vector3(2.88858f, -8.84274f, 8.53267f));
  points.push_back(Vector3(-0.0332654f, -5.33372f, -5.47044f));
  points.push_back(Vector3(0.150456f, 6.35548f, -7.56462f));
  points.push_back(Vector3(8.4228f, 2.8489f, 7.20267f));
  points.push_back(Vector3(-3.14066f, -7.11539f, 0.357372f));
  points.push_back(Vector3(9.68566f, 1.29917f, -6.84133f));
  points.push_back(Vector3(-9.01608f, 6.78152f, -6.87857f));
  points.push_back(Vector3(-1.75451f, 4.41389f, 9.36888f));
  points.push_back(Vector3(9.89563f, -5.17502f, -7.54936f));
  points.push_back(Vector3(-3.50505f, 1.08982f, 4.65194f));
  points.push_back(Vector3(9.33775f, -3.07047f, -4.08551f));
  points.push_back(Vector3(3.549f, 7.26188f, 5.13535f));
  points.push_back(Vector3(7.90643f, -9.86755f, 2.68716f));
  points.push_back(Vector3(4.77462f, -3.09366f, 2.38746f));
  points.push_back(Vector3(-0.868251f, 2.88308f, -0.955535f));
  points.push_back(Vector3(-9.78332f, 2.01147f, 7.83074f));
  points.push_back(Vector3(5.35325f, 2.09754f, -8.81649f));
  points.push_back(Vector3(-5.80309f, -0.359813f, -6.04602f));
  points.push_back(Vector3(6.80776f, 7.68792f, -3.75164f));
  points.push_back(Vector3(-1.60253f, -5.84948f, -6.79067f));
  points.push_back(Vector3(8.31721f, 2.42531f, 2.96976f));
  points.push_back(Vector3(-0.759606f, 7.17826f, -8.26411f));
  points.push_back(Vector3(7.07816f, 1.61107f, 3.51482f));
  points.push_back(Vector3(9.76257f, -0.25544f, -0.174261f));
  points.push_back(Vector3(-9.1052f, 9.55748f, -9.47325f));
  points.push_back(Vector3(9.04599f, 2.32032f, 6.66067f));
  points.push_back(Vector3(-0.81576f, 0.8652f, 0.400097f));
  points.push_back(Vector3(-8.93735f, -7.72149f, -4.00189f));
  points.push_back(Vector3(-8.20917f, 6.3976f, -1.90039f));
  points.push_back(Vector3(2.00659f, 6.51662f, -5.51683f));
  points.push_back(Vector3(5.53636f, -0.811487f, -7.21122f));
  points.push_back(Vector3(-6.65456f, 2.51259f, 7.76177f));
  points.push_back(Vector3(7.97296f, 6.83828f, -0.372021f));
  points.push_back(Vector3(-0.542924f, -3.69854f, -6.01001f));
  TestSphereRitter(points, file);
}

void SphereRitterTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.24458f, -4.38215f, -1.96081f));
  points.push_back(Vector3(-5.43931f, 6.20899f, -0.0796533f));
  points.push_back(Vector3(5.93616f, -2.90811f, -5.94836f));
  points.push_back(Vector3(7.77703f, -4.87594f, -9.48668f));
  points.push_back(Vector3(-6.94266f, 6.26881f, -6.84194f));
  points.push_back(Vector3(-4.23688f, -0.397046f, -1.301f));
  points.push_back(Vector3(2.83486f, -3.6607f, -2.46437f));
  points.push_back(Vector3(0.484939f, 7.01468f, 6.32313f));
  points.push_back(Vector3(6.61794f, -3.66497f, 9.58617f));
  points.push_back(Vector3(9.99695f, -3.84136f, 1.84851f));
  points.push_back(Vector3(3.18094f, 1.19419f, -0.529496f));
  points.push_back(Vector3(2.64199f, 4.27412f, 0.500809f));
  points.push_back(Vector3(-5.22874f, -5.84765f, 6.292f));
  points.push_back(Vector3(-1.57323f, 8.15302f, -4.77767f));
  points.push_back(Vector3(-8.64681f, 2.68899f, 4.25031f));
  points.push_back(Vector3(-1.40294f, -8.73165f, -2.28736f));
  points.push_back(Vector3(0.226753f, -8.53267f, -1.58238f));
  points.push_back(Vector3(-5.51134f, 8.70785f, 9.17478f));
  points.push_back(Vector3(-2.85623f, -1.09592f, 5.2794f));
  points.push_back(Vector3(-4.8027f, -3.72723f, -3.32072f));
  points.push_back(Vector3(6.1919f, -9.4348f, -6.60634f));
  points.push_back(Vector3(3.02713f, -8.12738f, 5.80004f));
  points.push_back(Vector3(-8.47957f, -4.8381f, 1.89367f));
  points.push_back(Vector3(-1.47374f, -1.23569f, -4.20392f));
  points.push_back(Vector3(-2.99905f, 9.87243f, -8.36665f));
  points.push_back(Vector3(5.72191f, -6.15223f, 2.78115f));
  points.push_back(Vector3(-7.40104f, -6.36708f, 8.04132f));
  points.push_back(Vector3(-4.63729f, 2.53395f, 1.29429f));
  points.push_back(Vector3(-8.86349f, -7.58843f, -1.33152f));
  points.push_back(Vector3(-7.81915f, -2.79458f, 8.21039f));
  points.push_back(Vector3(-1.32786f, 1.73193f, 4.59273f));
  points.push_back(Vector3(3.74859f, 1.49754f, -2.68166f));
  points.push_back(Vector3(8.49849f, -8.5168f, 5.60472f));
  points.push_back(Vector3(6.07105f, -9.99817f, -9.24009f));
  points.push_back(Vector3(-4.80697f, -1.32237f, -2.8367f));
  points.push_back(Vector3(7.02506f, -4.93026f, 7.2924f));
  points.push_back(Vector3(3.80779f, -2.05237f, 4.77279f));
  points.push_back(Vector3(5.65844f, 1.64281f, -0.411084f));
  points.push_back(Vector3(-7.4279f, 9.59288f, -8.6877f));
  points.push_back(Vector3(-4.27107f, -0.00701904f, 5.94348f));
  points.push_back(Vector3(8.02728f, 3.48308f, -2.54616f));
  points.push_back(Vector3(8.14753f, 7.38273f, -1.66417f));
  points.push_back(Vector3(1.67699f, 6.51662f, 7.00797f));
  points.push_back(Vector3(-8.29096f, -4.52498f, -8.62484f));
  points.push_back(Vector3(9.64965f, 4.20026f, 0.544145f));
  points.push_back(Vector3(-2.82571f, 3.33232f, 7.60308f));
  points.push_back(Vector3(3.89508f, 5.70666f, 4.21369f));
  points.push_back(Vector3(-1.15329f, 8.53572f, -3.45378f));
  points.push_back(Vector3(-2.24342f, -1.02512f, 9.04477f));
  points.push_back(Vector3(2.84707f, 3.84991f, 0.283517f));
  points.push_back(Vector3(2.64443f, 0.606403f, 1.49815f));
  points.push_back(Vector3(-5.24766f, -3.58928f, 9.54344f));
  points.push_back(Vector3(-1.29368f, 6.89138f, 1.24241f));
  points.push_back(Vector3(-6.30909f, 3.18766f, 4.11908f));
  points.push_back(Vector3(-6.79189f, -0.149846f, -0.488601f));
  points.push_back(Vector3(7.81549f, -7.09098f, 2.01392f));
  points.push_back(Vector3(8.32575f, -1.84606f, 3.46416f));
  points.push_back(Vector3(9.89502f, -6.72658f, -4.14472f));
  points.push_back(Vector3(-2.23914f, 9.682f, -8.76949f));
  points.push_back(Vector3(-6.02527f, -4.41084f, -4.85763f));
  points.push_back(Vector3(-8.51314f, 2.85745f, 1.27659f));
  points.push_back(Vector3(-2.7897f, -0.501419f, 1.37852f));
  points.push_back(Vector3(2.46254f, -9.83703f, -9.16501f));
  points.push_back(Vector3(-2.39723f, 0.627155f, 2.53883f));
  points.push_back(Vector3(-9.05087f, -8.02118f, -5.99536f));
  points.push_back(Vector3(-9.1052f, 6.39027f, 6.19984f));
  points.push_back(Vector3(3.50322f, 4.54817f, 3.08512f));
  points.push_back(Vector3(-9.14914f, 2.25929f, -0.586871f));
  points.push_back(Vector3(-2.68044f, -5.02609f, -0.555131f));
  points.push_back(Vector3(9.07285f, -4.96078f, -9.33226f));
  points.push_back(Vector3(-1.03183f, -8.80734f, 5.08957f));
  points.push_back(Vector3(-4.14228f, 3.66802f, -2.58034f));
  points.push_back(Vector3(2.03345f, 8.34651f, 5.65233f));
  points.push_back(Vector3(4.87533f, 4.16364f, -5.71581f));
  points.push_back(Vector3(-6.60939f, -2.61818f, 2.32887f));
  points.push_back(Vector3(4.18134f, -3.81207f, -5.46556f));
  points.push_back(Vector3(5.21226f, -0.425733f, -3.33232f));
  points.push_back(Vector3(-3.24503f, -3.32194f, 3.49162f));
  points.push_back(Vector3(-8.11396f, 4.44258f, -9.18943f));
  points.push_back(Vector3(-2.172f, -6.28956f, -0.646687f));
  points.push_back(Vector3(0.279855f, 0.758995f, 9.0698f));
  points.push_back(Vector3(6.33412f, 3.64483f, 7.31803f));
  points.push_back(Vector3(-9.27915f, 3.52458f, 7.23319f));
  points.push_back(Vector3(6.26637f, -0.849941f, -0.827357f));
  points.push_back(Vector3(7.21305f, -3.62529f, 6.21937f));
  points.push_back(Vector3(3.16507f, 7.48772f, -4.75814f));
  points.push_back(Vector3(1.21006f, -8.58272f, -0.970184f));
  points.push_back(Vector3(3.78338f, 9.86816f, -8.76339f));
  points.push_back(Vector3(-3.70281f, -0.500809f, 7.07816f));
  points.push_back(Vector3(5.78784f, -3.67046f, 1.66967f));
  points.push_back(Vector3(-6.19617f, 5.78295f, -3.82305f));
  points.push_back(Vector3(1.57567f, -1.0947f, 6.5276f));
  points.push_back(Vector3(-5.73168f, 0.893277f, -3.53435f));
  points.push_back(Vector3(-1.5598f, -8.64864f, -8.62667f));
  points.push_back(Vector3(-0.75106f, 9.17356f, -3.37138f));
  points.push_back(Vector3(-1.06357f, -1.07944f, 5.46129f));
  TestSphereRitter(points, file);
}

void SphereRitterTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.8114f, -0.7828f, 7.94732f));
  points.push_back(Vector3(-9.48485f, -2.06763f, -3.87493f));
  points.push_back(Vector3(-7.21976f, 1.41209f, -7.15812f));
  points.push_back(Vector3(-2.772f, 6.0802f, 9.38292f));
  points.push_back(Vector3(-7.76544f, 8.39534f, 6.77236f));
  points.push_back(Vector3(-1.94922f, -5.96301f, -0.550249f));
  points.push_back(Vector3(4.57198f, -9.29991f, 1.10324f));
  points.push_back(Vector3(-6.28346f, 3.85662f, 1.16611f));
  points.push_back(Vector3(1.04038f, -8.96359f, 7.10135f));
  points.push_back(Vector3(6.01245f, 2.17139f, 2.39174f));
  points.push_back(Vector3(3.51848f, 1.1301f, -3.75225f));
  points.push_back(Vector3(4.25397f, -1.48961f, -4.1319f));
  points.push_back(Vector3(8.55464f, -9.21995f, 9.57213f));
  points.push_back(Vector3(7.83196f, 6.01245f, -8.00531f));
  points.push_back(Vector3(4.26008f, -8.53389f, 3.9256f));
  points.push_back(Vector3(-3.93719f, 5.8623f, 3.56243f));
  points.push_back(Vector3(1.97302f, 3.79681f, 2.38075f));
  points.push_back(Vector3(-1.20945f, -9.90417f, -6.48976f));
  points.push_back(Vector3(4.46882f, -1.59032f, -5.11765f));
  TestSphereRitter(points, file);
}

void SphereRitterTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.94507f, -0.323191f, -2.58217f));
  points.push_back(Vector3(6.48122f, -3.867f, -4.45235f));
  points.push_back(Vector3(5.50096f, 1.11911f, -2.37587f));
  points.push_back(Vector3(8.64498f, -7.87164f, 7.58171f));
  points.push_back(Vector3(6.89749f, -8.24335f, -0.744346f));
  points.push_back(Vector3(9.035f, -2.64626f, 5.97705f));
  points.push_back(Vector3(9.70763f, 8.6932f, -5.71764f));
  points.push_back(Vector3(0.646077f, 9.8468f, -5.6859f));
  points.push_back(Vector3(3.8731f, -1.19663f, -4.28083f));
  points.push_back(Vector3(-7.73736f, -6.47694f, 8.77438f));
  points.push_back(Vector3(0.799279f, -3.22611f, 6.55873f));
  points.push_back(Vector3(-7.49199f, -6.53554f, -9.97192f));
  points.push_back(Vector3(4.48714f, -8.6932f, 2.31971f));
  points.push_back(Vector3(-1.31565f, -0.0735497f, -0.443434f));
  points.push_back(Vector3(-2.50221f, 2.46925f, -6.68752f));
  points.push_back(Vector3(7.09403f, 6.06311f, 0.472732f));
  points.push_back(Vector3(-3.47636f, 0.827357f, 9.63988f));
  points.push_back(Vector3(-9.24009f, -3.45012f, 6.24683f));
  points.push_back(Vector3(2.39601f, 8.92758f, 2.31727f));
  points.push_back(Vector3(-0.564898f, -2.50404f, 1.22715f));
  points.push_back(Vector3(7.80572f, -9.78271f, 0.165715f));
  points.push_back(Vector3(-2.10303f, -5.41734f, 7.06351f));
  points.push_back(Vector3(-4.52803f, 2.29652f, 6.87613f));
  points.push_back(Vector3(4.37727f, 8.96481f, 1.49876f));
  points.push_back(Vector3(2.30201f, 2.79031f, 8.28669f));
  points.push_back(Vector3(-6.05152f, -8.6053f, 3.09244f));
  points.push_back(Vector3(3.89325f, -0.182195f, 9.04477f));
  points.push_back(Vector3(4.41572f, -3.01004f, 9.67589f));
  points.push_back(Vector3(-9.03012f, 7.30522f, -7.17093f));
  points.push_back(Vector3(-2.88797f, -1.63488f, -7.70745f));
  points.push_back(Vector3(7.47917f, -9.07285f, 3.8255f));
  points.push_back(Vector3(-5.39048f, 6.19312f, -7.48527f));
  points.push_back(Vector3(1.69286f, 5.65294f, 1.51097f));
  points.push_back(Vector3(-5.92944f, -4.05988f, 1.83325f));
  points.push_back(Vector3(9.20957f, 3.23893f, 8.71944f));
  points.push_back(Vector3(-8.56502f, -9.77783f, 9.83886f));
  points.push_back(Vector3(2.69265f, 0.108341f, -3.32255f));
  points.push_back(Vector3(0.277413f, 0.290841f, 5.61449f));
  points.push_back(Vector3(-9.85839f, -4.71358f, -7.00369f));
  points.push_back(Vector3(2.45888f, -9.76501f, -3.39152f));
  points.push_back(Vector3(3.80047f, 7.02933f, 1.7069f));
  points.push_back(Vector3(-1.75695f, -4.05622f, 3.87066f));
  points.push_back(Vector3(-1.87658f, 7.89911f, -8.92392f));
  points.push_back(Vector3(3.71075f, -9.72167f, -5.73779f));
  points.push_back(Vector3(9.94812f, -6.12537f, -2.54677f));
  points.push_back(Vector3(5.70727f, -1.22654f, -8.29218f));
  points.push_back(Vector3(9.6588f, -3.91888f, -5.6212f));
  points.push_back(Vector3(1.6245f, -1.02878f, 3.18461f));
  points.push_back(Vector3(-8.73592f, 6.42079f, 8.1048f));
  points.push_back(Vector3(-4.4322f, 3.70769f, -6.53005f));
  points.push_back(Vector3(-5.67614f, -6.71987f, 6.02954f));
  points.push_back(Vector3(6.42384f, -3.15531f, -1.82104f));
  points.push_back(Vector3(4.14716f, 3.043f, 5.82629f));
  points.push_back(Vector3(-0.555742f, 1.54637f, 9.79614f));
  points.push_back(Vector3(1.0947f, 9.90845f, -7.21061f));
  points.push_back(Vector3(-6.17176f, -1.27842f, -1.17038f));
  points.push_back(Vector3(7.68487f, 2.94961f, -6.42872f));
  points.push_back(Vector3(-3.25663f, -0.749229f, -5.75426f));
  points.push_back(Vector3(-4.04462f, -2.00293f, -1.40904f));
  points.push_back(Vector3(1.95654f, -9.48851f, -3.38176f));
  points.push_back(Vector3(9.80895f, 5.22813f, -4.56587f));
  points.push_back(Vector3(1.11667f, 8.08405f, 6.37867f));
  points.push_back(Vector3(-9.81689f, 3.86517f, -5.9563f));
  points.push_back(Vector3(0.296335f, 9.7937f, -9.32981f));
  points.push_back(Vector3(7.54265f, 5.59374f, -6.09363f));
  points.push_back(Vector3(6.1681f, -6.01794f, -1.85705f));
  points.push_back(Vector3(8.6639f, 0.658895f, 6.29749f));
  points.push_back(Vector3(-2.37526f, 9.56481f, -5.98254f));
  points.push_back(Vector3(-4.26862f, 7.70501f, 3.43608f));
  points.push_back(Vector3(2.29896f, -5.47777f, 0.468459f));
  points.push_back(Vector3(-2.37831f, 2.6603f, 3.62835f));
  points.push_back(Vector3(5.74389f, -5.47349f, 2.23182f));
  points.push_back(Vector3(8.62239f, -4.07758f, -0.362255f));
  points.push_back(Vector3(6.5569f, -3.12906f, 6.92495f));
  points.push_back(Vector3(1.14414f, -6.77602f, -0.872524f));
  points.push_back(Vector3(5.91357f, 9.28587f, 6.32984f));
  points.push_back(Vector3(8.66695f, 6.39454f, -6.44703f));
  points.push_back(Vector3(-2.62978f, 1.08188f, 1.32542f));
  points.push_back(Vector3(5.86535f, 8.22138f, 7.86981f));
  points.push_back(Vector3(-5.14328f, 7.84295f, 1.79724f));
  points.push_back(Vector3(6.9335f, 3.32987f, -9.682f));
  points.push_back(Vector3(-3.95673f, 3.12052f, 6.15589f));
  points.push_back(Vector3(3.18827f, 6.86575f, 1.92602f));
  points.push_back(Vector3(-2.79092f, 3.74737f, -4.3675f));
  points.push_back(Vector3(9.4879f, -2.65297f, 1.62084f));
  points.push_back(Vector3(6.87551f, 1.99011f, -1.31016f));
  points.push_back(Vector3(8.42769f, -8.19758f, -2.38929f));
  points.push_back(Vector3(1.28269f, -9.71007f, 3.26701f));
  points.push_back(Vector3(-5.8684f, -7.23258f, -8.67489f));
  points.push_back(Vector3(-2.20374f, 5.33738f, -8.31538f));
  points.push_back(Vector3(6.23219f, -9.67528f, -0.674765f));
  points.push_back(Vector3(5.03647f, -9.88952f, -2.75918f));
  points.push_back(Vector3(-8.13654f, 0.726646f, 3.63201f));
  points.push_back(Vector3(-6.61855f, -2.09754f, 5.51622f));
  TestSphereRitter(points, file);
}

void SphereRitterTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.36421f, 1.99988f, 3.83404f));
  points.push_back(Vector3(-6.66433f, 8.47835f, -3.88104f));
  points.push_back(Vector3(-8.0456f, 4.29243f, 9.37986f));
  points.push_back(Vector3(-7.0745f, 0.540483f, 8.90072f));
  points.push_back(Vector3(2.30811f, 0.324411f, 0.457473f));
  points.push_back(Vector3(0.823084f, -2.5016f, 3.94757f));
  points.push_back(Vector3(-4.2082f, -5.15976f, -0.513016f));
  points.push_back(Vector3(-6.44398f, -2.84524f, 9.4409f));
  points.push_back(Vector3(-8.15912f, -8.64925f, 9.24375f));
  points.push_back(Vector3(6.6393f, -9.79492f, -4.67513f));
  points.push_back(Vector3(-6.71743f, 7.14408f, 9.30113f));
  points.push_back(Vector3(0.780358f, -5.02121f, 9.00204f));
  points.push_back(Vector3(-0.512406f, -6.86819f, -1.39683f));
  points.push_back(Vector3(7.84722f, -1.47618f, -4.87167f));
  points.push_back(Vector3(-2.16468f, 6.87063f, -5.8507f));
  points.push_back(Vector3(1.60985f, 3.98541f, -5.35203f));
  points.push_back(Vector3(3.10831f, 4.92782f, 2.23182f));
  points.push_back(Vector3(-1.06113f, 2.2483f, 4.16181f));
  points.push_back(Vector3(-3.51848f, -5.40819f, -2.69021f));
  points.push_back(Vector3(1.23997f, -6.50197f, -6.32984f));
  points.push_back(Vector3(5.03891f, -2.89407f, 6.90664f));
  points.push_back(Vector3(-4.06354f, -5.7561f, -8.95321f));
  points.push_back(Vector3(-8.29524f, 7.80938f, 1.42308f));
  points.push_back(Vector3(-3.68816f, 9.91577f, 7.92047f));
  points.push_back(Vector3(6.58864f, -4.85824f, -2.01331f));
  points.push_back(Vector3(7.40288f, 4.90646f, 6.34388f));
  points.push_back(Vector3(-3.41777f, 3.0253f, -0.212104f));
  points.push_back(Vector3(8.52046f, -6.62526f, -8.69137f));
  points.push_back(Vector3(-6.95242f, -7.90094f, -0.522782f));
  points.push_back(Vector3(1.7423f, -2.24403f, 3.26884f));
  points.push_back(Vector3(-9.87609f, -2.77932f, 9.41588f));
  points.push_back(Vector3(-7.04703f, -9.75524f, -0.158391f));
  points.push_back(Vector3(1.07883f, -6.04297f, -4.46089f));
  points.push_back(Vector3(-9.82665f, -1.9602f, -0.606403f));
  points.push_back(Vector3(9.68444f, -1.86865f, -2.28187f));
  points.push_back(Vector3(5.52965f, 8.305f, -6.43239f));
  points.push_back(Vector3(-9.63561f, -7.83258f, -9.67772f));
  points.push_back(Vector3(-5.27146f, 6.21387f, 3.69427f));
  points.push_back(Vector3(7.31376f, 2.678f, 4.70199f));
  points.push_back(Vector3(3.91644f, -8.50154f, -0.176092f));
  points.push_back(Vector3(-8.61324f, 7.81243f, -7.00736f));
  points.push_back(Vector3(0.674764f, -1.81616f, 3.32072f));
  points.push_back(Vector3(6.93716f, -7.05618f, -7.45415f));
  points.push_back(Vector3(6.29322f, 1.13559f, -2.86477f));
  points.push_back(Vector3(-8.78292f, 1.73864f, 4.82894f));
  points.push_back(Vector3(2.72195f, 4.18378f, -5.51988f));
  points.push_back(Vector3(9.22422f, 9.06247f, 4.43648f));
  points.push_back(Vector3(2.3484f, 0.406811f, 9.15891f));
  points.push_back(Vector3(9.1876f, -4.95651f, -4.738f));
  points.push_back(Vector3(4.39802f, 8.15424f, -8.21467f));
  points.push_back(Vector3(1.02756f, 8.39167f, 4.53658f));
  points.push_back(Vector3(0.872524f, 9.10276f, 8.04071f));
  points.push_back(Vector3(4.31806f, -5.71276f, 3.74493f));
  points.push_back(Vector3(-0.0424206f, -0.491653f, 0.461745f));
  points.push_back(Vector3(-3.72784f, -9.64171f, 0.961028f));
  points.push_back(Vector3(8.33186f, 5.39354f, -3.43913f));
  points.push_back(Vector3(6.38111f, 5.80676f, -5.69628f));
  points.push_back(Vector3(-1.79235f, -0.843226f, 5.77746f));
  points.push_back(Vector3(5.16037f, -3.6607f, 3.63628f));
  points.push_back(Vector3(2.7897f, 1.56285f, 8.44722f));
  points.push_back(Vector3(-7.52922f, -6.49525f, -0.330516f));
  points.push_back(Vector3(-2.37465f, -8.33857f, 4.07514f));
  points.push_back(Vector3(-1.42613f, 3.95245f, -1.08493f));
  points.push_back(Vector3(-2.4424f, -9.44395f, -3.78216f));
  points.push_back(Vector3(0.956144f, 4.49629f, 1.85217f));
  points.push_back(Vector3(-7.86676f, 5.40025f, 9.78332f));
  points.push_back(Vector3(4.54634f, 5.14206f, -7.17032f));
  points.push_back(Vector3(-1.05258f, -9.77172f, 5.9508f));
  points.push_back(Vector3(4.68795f, -3.02896f, 6.0332f));
  TestSphereRitter(points, file);
}

void SphereRitterTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.16135f, -2.12317f, 2.46376f));
  points.push_back(Vector3(-1.49632f, -9.729f, -2.55532f));
  points.push_back(Vector3(-7.92108f, -1.04099f, -1.77099f));
  points.push_back(Vector3(-6.08142f, -3.35002f, -1.85278f));
  points.push_back(Vector3(-2.69143f, 0.693686f, 2.44911f));
  points.push_back(Vector3(7.89117f, -9.86328f, 5.60167f));
  points.push_back(Vector3(8.83908f, -8.4875f, -7.50175f));
  points.push_back(Vector3(-5.97034f, -8.57479f, -3.1077f));
  points.push_back(Vector3(6.66189f, -8.54732f, 6.96158f));
  points.push_back(Vector3(4.2552f, -9.48302f, -1.49754f));
  points.push_back(Vector3(0.630207f, 6.80593f, 8.17011f));
  points.push_back(Vector3(1.70202f, 6.34632f, -7.65557f));
  points.push_back(Vector3(-2.40455f, -7.00674f, 0.679647f));
  points.push_back(Vector3(3.98175f, 0.99704f, 1.17038f));
  points.push_back(Vector3(1.04465f, 8.08039f, -6.52516f));
  points.push_back(Vector3(8.36116f, 8.76583f, 7.54875f));
  points.push_back(Vector3(-4.34919f, -5.33738f, 7.65801f));
  points.push_back(Vector3(8.60958f, 5.8562f, 4.0788f));
  points.push_back(Vector3(-1.26682f, -0.825526f, -2.48451f));
  points.push_back(Vector3(-5.14328f, 1.66906f, 6.94021f));
  points.push_back(Vector3(-2.64504f, 7.37663f, 3.65764f));
  points.push_back(Vector3(-4.66353f, -9.94873f, -7.30583f));
  points.push_back(Vector3(6.26575f, 5.45763f, 4.9559f));
  points.push_back(Vector3(-5.11826f, -9.07651f, 2.0835f));
  points.push_back(Vector3(-9.49583f, -9.72655f, -6.32618f));
  points.push_back(Vector3(-8.05963f, -8.03827f, -3.24137f));
  points.push_back(Vector3(9.80346f, -7.69768f, -1.89856f));
  points.push_back(Vector3(-3.76446f, -1.0184f, -1.92358f));
  points.push_back(Vector3(-7.54631f, 3.35063f, -8.46675f));
  points.push_back(Vector3(5.48509f, -5.24094f, 3.65947f));
  points.push_back(Vector3(6.46962f, 0.477614f, 8.23542f));
  points.push_back(Vector3(-5.68163f, 1.89612f, 9.99268f));
  points.push_back(Vector3(-1.71361f, -9.55077f, 8.43806f));
  points.push_back(Vector3(4.19477f, -1.70751f, -9.71679f));
  points.push_back(Vector3(6.292f, 7.82037f, 8.08466f));
  points.push_back(Vector3(2.22877f, -3.46416f, 8.73592f));
  points.push_back(Vector3(0.744346f, 6.96768f, 7.32414f));
  points.push_back(Vector3(1.8656f, 3.35246f, -2.27332f));
  points.push_back(Vector3(5.58763f, -3.42082f, 7.29728f));
  points.push_back(Vector3(-8.16828f, -5.12864f, 5.69262f));
  points.push_back(Vector3(-6.21021f, -3.32682f, 9.92309f));
  points.push_back(Vector3(4.18928f, -9.8059f, 3.22001f));
  points.push_back(Vector3(8.67061f, -4.20331f, 2.80068f));
  points.push_back(Vector3(-9.81445f, -7.79656f, -4.46516f));
  points.push_back(Vector3(1.67455f, -8.00348f, -6.60939f));
  points.push_back(Vector3(6.37196f, 3.19498f, -6.33839f));
  points.push_back(Vector3(4.75204f, -1.89917f, 5.9209f));
  points.push_back(Vector3(5.69079f, 6.21509f, 6.57399f));
  points.push_back(Vector3(5.39293f, 2.16956f, 3.37993f));
  points.push_back(Vector3(1.40721f, 3.51543f, -4.86801f));
  points.push_back(Vector3(-3.08512f, 0.28962f, -1.87353f));
  points.push_back(Vector3(8.33857f, -6.2859f, -1.9779f));
  points.push_back(Vector3(8.39351f, 3.59661f, 0.431226f));
  points.push_back(Vector3(8.39961f, 1.64647f, 2.7488f));
  points.push_back(Vector3(-9.97131f, 7.07938f, -2.70669f));
  points.push_back(Vector3(-1.55675f, 1.95227f, -6.45619f));
  points.push_back(Vector3(-6.58071f, -3.17606f, 6.57033f));
  points.push_back(Vector3(0.293894f, -8.28669f, 5.25681f));
  points.push_back(Vector3(-5.51073f, 7.39006f, 6.33717f));
  points.push_back(Vector3(8.22077f, 1.38523f, -9.52513f));
  points.push_back(Vector3(-7.63604f, 4.48775f, 8.95871f));
  points.push_back(Vector3(0.395215f, -0.58443f, 8.71761f));
  points.push_back(Vector3(-7.34855f, 2.54616f, 6.11805f));
  points.push_back(Vector3(-9.40916f, -3.24564f, -5.92456f));
  points.push_back(Vector3(0.999482f, -5.39171f, 7.5042f));
  points.push_back(Vector3(7.0568f, 5.46678f, -6.32801f));
  points.push_back(Vector3(4.83261f, -4.232f, 4.46577f));
  points.push_back(Vector3(-0.408033f, 3.36161f, -3.80169f));
  points.push_back(Vector3(0.472732f, 3.62102f, 3.28532f));
  points.push_back(Vector3(-4.09955f, -3.57524f, -9.83825f));
  points.push_back(Vector3(1.75878f, 2.32398f, 3.22306f));
  points.push_back(Vector3(-3.24259f, 5.42039f, 7.75262f));
  points.push_back(Vector3(-2.57302f, 3.49895f, -5.32884f));
  points.push_back(Vector3(-1.04099f, 4.7795f, 4.96994f));
  points.push_back(Vector3(0.338451f, 9.7235f, -4.83749f));
  points.push_back(Vector3(7.85882f, 3.34513f, 4.31868f));
  points.push_back(Vector3(4.63729f, 7.35527f, -1.98706f));
  points.push_back(Vector3(-9.90051f, -9.83398f, 5.46556f));
  points.push_back(Vector3(9.62401f, 7.32414f, -7.32902f));
  points.push_back(Vector3(4.36201f, 6.86514f, -5.68285f));
  TestSphereRitter(points, file);
}

void SphereRitterTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.03806f, 4.61592f, -1.52379f));
  points.push_back(Vector3(9.10276f, 0.592974f, 6.03992f));
  points.push_back(Vector3(9.86022f, 8.18598f, 3.05765f));
  points.push_back(Vector3(1.26499f, -8.94711f, -0.186468f));
  points.push_back(Vector3(-4.91623f, 6.83645f, 2.24036f));
  points.push_back(Vector3(3.99579f, 4.97787f, 2.84524f));
  points.push_back(Vector3(-1.88391f, 1.72826f, 4.48225f));
  points.push_back(Vector3(-0.143132f, -0.386059f, 2.33741f));
  points.push_back(Vector3(-0.735801f, 9.93774f, -6.68508f));
  points.push_back(Vector3(7.72515f, 8.66878f, 9.91028f));
  points.push_back(Vector3(-5.80859f, 6.65822f, -4.65499f));
  points.push_back(Vector3(-5.04074f, -2.71584f, 2.22633f));
  points.push_back(Vector3(-4.61104f, -9.28526f, -6.04236f));
  points.push_back(Vector3(5.58092f, 9.61364f, 1.93396f));
  points.push_back(Vector3(-1.65319f, -0.907315f, 6.57948f));
  points.push_back(Vector3(5.91479f, -0.375072f, 2.04871f));
  points.push_back(Vector3(-1.99316f, 8.75423f, 5.71703f));
  points.push_back(Vector3(2.01086f, -2.80313f, -7.40898f));
  points.push_back(Vector3(-5.91601f, 4.96445f, 1.08371f));
  points.push_back(Vector3(-8.05414f, -4.79782f, 7.38334f));
  points.push_back(Vector3(6.54592f, 7.28507f, 4.3614f));
  points.push_back(Vector3(3.32987f, 1.60131f, -6.06067f));
  points.push_back(Vector3(7.40898f, 9.72228f, 2.10791f));
  points.push_back(Vector3(-5.49669f, -3.93292f, 2.88858f));
  points.push_back(Vector3(-5.40269f, -7.17215f, -1.55797f));
  points.push_back(Vector3(9.53734f, -4.13129f, -2.25562f));
  points.push_back(Vector3(0.35432f, 5.26475f, 9.96277f));
  points.push_back(Vector3(-5.36058f, 3.11258f, 3.72784f));
  points.push_back(Vector3(-6.85476f, -6.99332f, -6.63503f));
  points.push_back(Vector3(9.60509f, 7.52434f, 8.0987f));
  points.push_back(Vector3(-9.3646f, -1.52745f, -6.08203f));
  points.push_back(Vector3(4.80697f, 8.19025f, -3.28166f));
  points.push_back(Vector3(4.61959f, 1.89306f, -7.16178f));
  points.push_back(Vector3(4.62203f, 6.65334f, 4.69588f));
  points.push_back(Vector3(-0.682089f, -4.28205f, -3.82244f));
  points.push_back(Vector3(4.44014f, 4.86007f, -4.41511f));
  points.push_back(Vector3(-5.36424f, -7.72698f, -0.411695f));
  points.push_back(Vector3(-6.84072f, 7.29606f, 6.15223f));
  points.push_back(Vector3(-1.64464f, 7.72454f, -5.20554f));
  points.push_back(Vector3(-1.29551f, 9.64904f, 4.68307f));
  points.push_back(Vector3(-1.68249f, 4.20026f, 7.19901f));
  points.push_back(Vector3(3.24442f, 9.03317f, -7.26493f));
  points.push_back(Vector3(-0.0979644f, 9.15464f, 2.41127f));
  points.push_back(Vector3(9.78271f, 5.68835f, -4.41755f));
  points.push_back(Vector3(8.05353f, 9.20713f, 4.96384f));
  points.push_back(Vector3(5.30625f, -4.35163f, -2.11158f));
  points.push_back(Vector3(-0.754723f, -2.69997f, 7.69097f));
  points.push_back(Vector3(0.072329f, -9.38536f, -9.12595f));
  points.push_back(Vector3(-0.419019f, -0.434889f, 7.62139f));
  points.push_back(Vector3(-9.53001f, -2.266f, 7.91376f));
  points.push_back(Vector3(-6.74795f, 4.24543f, -3.67962f));
  points.push_back(Vector3(-3.31889f, 0.599078f, -0.425123f));
  points.push_back(Vector3(3.26151f, 1.9541f, -8.15485f));
  points.push_back(Vector3(8.59127f, -3.88531f, -0.805383f));
  points.push_back(Vector3(6.19678f, 6.88772f, -1.53844f));
  points.push_back(Vector3(-3.42387f, -7.61467f, -1.48839f));
  points.push_back(Vector3(-2.57179f, 4.91684f, -8.63155f));
  points.push_back(Vector3(-9.63012f, -7.03299f, -7.66289f));
  points.push_back(Vector3(4.84359f, 9.17722f, -0.481277f));
  points.push_back(Vector3(1.77892f, -4.78744f, 4.0611f));
  points.push_back(Vector3(-9.31944f, -0.514847f, -7.38151f));
  points.push_back(Vector3(7.96625f, 7.51396f, -7.40226f));
  points.push_back(Vector3(-5.33006f, 0.96408f, 1.10141f));
  points.push_back(Vector3(3.97137f, -5.50645f, 8.60775f));
  points.push_back(Vector3(9.99329f, 2.34107f, 2.72805f));
  points.push_back(Vector3(-1.55675f, 3.15653f, -4.03058f));
  points.push_back(Vector3(-0.912198f, 7.82159f, 9.06247f));
  points.push_back(Vector3(7.89239f, -0.873745f, 6.11011f));
  points.push_back(Vector3(1.52623f, 0.890225f, -8.18903f));
  points.push_back(Vector3(4.45845f, 2.93374f, -9.68322f));
  points.push_back(Vector3(-6.44337f, -3.78521f, -4.52559f));
  points.push_back(Vector3(-7.10318f, -5.68529f, 1.14231f));
  points.push_back(Vector3(1.00497f, 9.63073f, 1.36876f));
  points.push_back(Vector3(-3.4611f, 0.17487f, -9.07956f));
  points.push_back(Vector3(-3.55449f, 8.40815f, -8.56746f));
  points.push_back(Vector3(2.05847f, -8.22871f, 5.78234f));
  points.push_back(Vector3(0.540483f, -2.82388f, 0.70101f));
  points.push_back(Vector3(-8.65535f, 1.21799f, -0.191962f));
  points.push_back(Vector3(8.53755f, -8.54305f, -2.44911f));
  points.push_back(Vector3(-1.82287f, -0.821253f, -1.26499f));
  points.push_back(Vector3(8.77743f, -6.67043f, 3.20414f));
  points.push_back(Vector3(-9.72228f, 7.08121f, 8.25251f));
  points.push_back(Vector3(3.03324f, -5.30381f, 9.02463f));
  points.push_back(Vector3(-3.91522f, 0.779138f, -8.95077f));
  points.push_back(Vector3(-8.77133f, -4.4377f, 4.42549f));
  points.push_back(Vector3(-7.17887f, -9.0521f, -8.19025f));
  points.push_back(Vector3(-2.05786f, 0.0613427f, 2.1427f));
  points.push_back(Vector3(-9.38841f, -9.02097f, -7.18314f));
  points.push_back(Vector3(7.69036f, 4.28327f, -6.25111f));
  TestSphereRitter(points, file);
}

void SphereRitterTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.30723f, -1.02512f, 6.51357f));
  points.push_back(Vector3(6.34693f, -9.30174f, 2.48695f));
  points.push_back(Vector3(-6.59841f, 9.38597f, 2.54433f));
  points.push_back(Vector3(7.85028f, 3.24076f, -3.81573f));
  points.push_back(Vector3(-9.10337f, 3.4611f, -6.33778f));
  points.push_back(Vector3(-4.61959f, -4.75692f, 4.65377f));
  points.push_back(Vector3(7.26981f, 3.35551f, 3.87371f));
  points.push_back(Vector3(4.26679f, 2.43019f, 4.35285f));
  points.push_back(Vector3(-3.98358f, -1.27842f, -2.45521f));
  points.push_back(Vector3(-9.83032f, 5.2739f, -9.76806f));
  points.push_back(Vector3(6.92129f, 5.0444f, 7.4633f));
  points.push_back(Vector3(-4.22956f, -3.62957f, -8.64559f));
  points.push_back(Vector3(6.00085f, 3.41777f, -3.34147f));
  points.push_back(Vector3(1.77648f, -2.05176f, 6.72353f));
  points.push_back(Vector3(2.93008f, -6.49892f, -6.14856f));
  TestSphereRitter(points, file);
}

void SphereRitterTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.60204f, 8.04926f, -7.19352f));
  points.push_back(Vector3(-7.34123f, -1.03793f, 7.77764f));
  points.push_back(Vector3(-7.75262f, -6.18763f, -8.98129f));
  points.push_back(Vector3(3.21757f, -3.39885f, -7.79473f));
  points.push_back(Vector3(-8.95321f, 8.79818f, 8.9996f));
  points.push_back(Vector3(9.19614f, -7.2808f, 0.265816f));
  points.push_back(Vector3(1.89734f, -1.67577f, -7.31254f));
  points.push_back(Vector3(5.84216f, 6.11072f, -9.29075f));
  points.push_back(Vector3(-3.09977f, -0.24018f, -7.29972f));
  points.push_back(Vector3(1.88818f, 2.5895f, 8.80795f));
  points.push_back(Vector3(-4.15998f, -4.28144f, 9.04538f));
  points.push_back(Vector3(-7.48894f, -5.22324f, -9.00937f));
  points.push_back(Vector3(0.300608f, 7.56706f, 3.02713f));
  points.push_back(Vector3(6.34632f, -7.13675f, -2.84158f));
  points.push_back(Vector3(2.91788f, 9.14914f, -8.39778f));
  points.push_back(Vector3(-4.20942f, -6.39332f, 3.9256f));
  points.push_back(Vector3(6.93106f, -9.7589f, -9.2938f));
  points.push_back(Vector3(6.00269f, -3.60088f, 7.28263f));
  points.push_back(Vector3(5.62731f, -4.0434f, 3.82855f));
  points.push_back(Vector3(8.78048f, 2.78298f, 0.521561f));
  points.push_back(Vector3(-0.890836f, 4.80148f, 2.76284f));
  points.push_back(Vector3(-4.44502f, -8.21467f, 5.35508f));
  points.push_back(Vector3(9.61119f, -3.29264f, 1.55736f));
  points.push_back(Vector3(1.39195f, -9.66979f, -6.59169f));
  points.push_back(Vector3(-1.18198f, -4.31684f, 4.74105f));
  points.push_back(Vector3(-8.20246f, -1.63915f, -5.78661f));
  points.push_back(Vector3(-6.19251f, 2.22816f, -0.630817f));
  points.push_back(Vector3(-1.30161f, 4.5793f, -3.03446f));
  points.push_back(Vector3(-5.17441f, -9.13327f, -0.154729f));
  points.push_back(Vector3(-6.29749f, 3.45378f, 9.12534f));
  points.push_back(Vector3(-9.34996f, -9.47325f, 7.33329f));
  points.push_back(Vector3(8.22321f, -0.216987f, 7.95038f));
  points.push_back(Vector3(8.29401f, 2.56142f, -5.63524f));
  points.push_back(Vector3(3.64177f, 6.23219f, 3.75042f));
  points.push_back(Vector3(-4.33271f, -4.52742f, -7.53899f));
  points.push_back(Vector3(-6.94571f, 4.28327f, -1.4835f));
  points.push_back(Vector3(1.22959f, -0.655843f, 8.76705f));
  points.push_back(Vector3(-8.39167f, -7.47917f, 3.18033f));
  points.push_back(Vector3(-4.11603f, -0.890836f, 9.28098f));
  points.push_back(Vector3(-2.34535f, -9.41527f, 3.48491f));
  points.push_back(Vector3(2.16712f, 6.61916f, 3.82672f));
  points.push_back(Vector3(-5.66881f, 4.9797f, -9.08628f));
  TestSphereRitter(points, file);
}

void SphereRitterTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.27732f, -2.62062f, -9.9353f));
  points.push_back(Vector3(6.65334f, 5.13108f, -1.56285f));
  points.push_back(Vector3(1.89673f, 2.58583f, -6.28773f));
  points.push_back(Vector3(-8.43745f, 1.88635f, -3.41227f));
  points.push_back(Vector3(-6.41591f, -5.05295f, -3.16263f));
  points.push_back(Vector3(-5.88427f, 0.769372f, -6.65883f));
  points.push_back(Vector3(4.03058f, -1.26988f, 8.38557f));
  points.push_back(Vector3(5.99902f, -0.105289f, 8.57356f));
  points.push_back(Vector3(9.71068f, -0.0320446f, -7.36992f));
  points.push_back(Vector3(1.00986f, -3.23771f, -3.137f));
  points.push_back(Vector3(-5.60411f, -1.44322f, -4.99435f));
  points.push_back(Vector3(-8.76461f, -1.30528f, -0.771202f));
  points.push_back(Vector3(4.82162f, 8.67367f, -1.41942f));
  points.push_back(Vector3(4.42122f, -9.64476f, 1.74657f));
  points.push_back(Vector3(-8.69015f, 7.45598f, 6.37745f));
  points.push_back(Vector3(-8.06085f, 5.05905f, -4.08124f));
  points.push_back(Vector3(3.76995f, -4.326f, -6.65517f));
  points.push_back(Vector3(-9.08872f, 4.279f, 2.5663f));
  points.push_back(Vector3(-7.24052f, -3.66131f, 5.14512f));
  points.push_back(Vector3(4.03119f, 9.65392f, 1.36387f));
  points.push_back(Vector3(4.24055f, 0.365916f, -8.61385f));
  points.push_back(Vector3(9.14975f, 6.34388f, 4.43403f));
  points.push_back(Vector3(7.77764f, -4.98398f, 1.16794f));
  points.push_back(Vector3(-9.92554f, -1.62999f, 7.13736f));
  points.push_back(Vector3(-6.22791f, 3.36772f, -8.85189f));
  points.push_back(Vector3(-7.10196f, -2.01453f, 4.86923f));
  points.push_back(Vector3(8.32087f, 4.04767f, -6.54347f));
  points.push_back(Vector3(0.954313f, 4.79476f, 3.75774f));
  points.push_back(Vector3(-3.03262f, -7.61834f, -5.01816f));
  points.push_back(Vector3(8.72799f, 8.62545f, -1.78442f));
  points.push_back(Vector3(-6.46046f, 9.45006f, 3.56426f));
  points.push_back(Vector3(3.21635f, 5.68529f, 1.76427f));
  points.push_back(Vector3(9.7821f, 6.84072f, -7.29179f));
  points.push_back(Vector3(-7.17215f, 2.72927f, 1.96265f));
  points.push_back(Vector3(8.30012f, -5.37523f, -2.71279f));
  points.push_back(Vector3(1.44993f, -8.67122f, 8.13532f));
  points.push_back(Vector3(7.74956f, 0.713217f, -9.3115f));
  points.push_back(Vector3(-8.69991f, -3.56304f, -5.63829f));
  points.push_back(Vector3(7.83746f, -6.10523f, 1.57323f));
  points.push_back(Vector3(-4.9205f, -7.34001f, -7.55669f));
  points.push_back(Vector3(5.0383f, -7.14225f, -1.72216f));
  points.push_back(Vector3(8.29951f, -9.3646f, 2.70486f));
  points.push_back(Vector3(-9.57823f, 2.01025f, -6.51235f));
  points.push_back(Vector3(-1.04953f, -2.99173f, 7.10318f));
  points.push_back(Vector3(-8.80978f, 4.28388f, -6.46779f));
  points.push_back(Vector3(-3.59966f, -2.6487f, -8.35688f));
  points.push_back(Vector3(9.90417f, 4.01166f, 8.13593f));
  points.push_back(Vector3(7.07511f, -5.64318f, -3.90118f));
  points.push_back(Vector3(2.4424f, -8.29768f, 0.0564587f));
  points.push_back(Vector3(-6.42689f, 6.34266f, -9.08261f));
  points.push_back(Vector3(-9.98169f, -0.184637f, 2.39723f));
  points.push_back(Vector3(8.92087f, 5.98315f, -3.43059f));
  points.push_back(Vector3(-5.51622f, -3.25358f, -3.1492f));
  points.push_back(Vector3(1.124f, 8.75668f, -8.12006f));
  points.push_back(Vector3(-7.67693f, 7.90399f, 1.36143f));
  points.push_back(Vector3(-2.39051f, -2.64687f, 7.60674f));
  points.push_back(Vector3(-6.46046f, 0.22187f, -6.95181f));
  points.push_back(Vector3(6.99637f, 4.73434f, -7.81854f));
  points.push_back(Vector3(-0.357372f, 2.49245f, 4.32905f));
  points.push_back(Vector3(-0.591754f, 7.15995f, -9.33103f));
  points.push_back(Vector3(-7.3516f, 0.779748f, 9.4055f));
  points.push_back(Vector3(8.79696f, -8.16034f, -2.00964f));
  points.push_back(Vector3(6.89322f, 8.54244f, -2.73782f));
  points.push_back(Vector3(3.88104f, -5.64745f, -4.03546f));
  points.push_back(Vector3(-9.99451f, 7.3748f, -1.57628f));
  points.push_back(Vector3(0.920134f, 1.17649f, 3.13578f));
  TestSphereRitter(points, file);
}

void SphereRitterTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.16889f, 4.91012f, -7.63115f));
  points.push_back(Vector3(-7.33024f, 5.4387f, -9.00571f));
  points.push_back(Vector3(-1.87597f, 9.19919f, -2.98929f));
  points.push_back(Vector3(5.18296f, -8.30683f, -8.46492f));
  points.push_back(Vector3(6.5331f, -9.96643f, 7.8692f));
  points.push_back(Vector3(-2.10852f, 1.60192f, 6.04236f));
  points.push_back(Vector3(5.29527f, 1.95593f, 2.29957f));
  points.push_back(Vector3(-0.459914f, -4.74166f, 9.01547f));
  points.push_back(Vector3(5.30992f, -7.1746f, 3.80291f));
  points.push_back(Vector3(-7.87713f, 7.92901f, -5.35386f));
  TestSphereRitter(points, file);
}

void SphereRitterTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.60958f, -4.91806f, 2.71218f));
  points.push_back(Vector3(6.82547f, -5.87085f, 1.32725f));
  points.push_back(Vector3(-1.70751f, 4.05255f, 9.80102f));
  points.push_back(Vector3(-5.51866f, -8.35627f, 5.59374f));
  points.push_back(Vector3(0.270699f, -1.86193f, -5.58031f));
  points.push_back(Vector3(-0.803552f, 4.185f, 0.705893f));
  points.push_back(Vector3(-1.64647f, 8.76705f, 2.48817f));
  points.push_back(Vector3(-8.64864f, 9.94385f, 2.83242f));
  points.push_back(Vector3(-4.45174f, 1.13254f, 9.54222f));
  points.push_back(Vector3(9.75646f, 0.000915527f, 4.95224f));
  points.push_back(Vector3(7.99249f, 0.339671f, 3.45317f));
  points.push_back(Vector3(9.96399f, 3.92437f, 8.43013f));
  points.push_back(Vector3(5.1561f, 6.03931f, -5.15793f));
  points.push_back(Vector3(-9.84191f, 0.784631f, 3.93658f));
  points.push_back(Vector3(5.12192f, -7.55791f, -0.558184f));
  points.push_back(Vector3(-3.21451f, 4.0556f, -8.63521f));
  points.push_back(Vector3(-8.98495f, -9.47874f, -2.95022f));
  points.push_back(Vector3(-8.93735f, -7.99249f, -1.11362f));
  points.push_back(Vector3(3.12418f, 3.75103f, 7.97357f));
  points.push_back(Vector3(8.62728f, 1.81066f, 1.50059f));
  points.push_back(Vector3(0.675985f, -9.48607f, 2.6072f));
  points.push_back(Vector3(0.401928f, -1.8955f, 1.80395f));
  points.push_back(Vector3(0.0820947f, 7.73614f, -4.54085f));
  points.push_back(Vector3(-4.22834f, 7.77581f, -7.22465f));
  points.push_back(Vector3(5.24277f, 3.94513f, -5.98987f));
  points.push_back(Vector3(-8.15241f, 4.57625f, 6.64663f));
  points.push_back(Vector3(-7.25272f, 8.27815f, -2.1305f));
  points.push_back(Vector3(-2.50771f, -7.40104f, -2.1598f));
  points.push_back(Vector3(2.96487f, -2.54067f, -2.86538f));
  points.push_back(Vector3(0.343333f, -1.11545f, 4.13862f));
  points.push_back(Vector3(-0.399487f, 7.97235f, 3.30302f));
  points.push_back(Vector3(-8.17743f, 5.17808f, -8.9874f));
  points.push_back(Vector3(-2.12989f, 8.02851f, 3.90545f));
  points.push_back(Vector3(-8.41487f, 9.51781f, 2.09265f));
  points.push_back(Vector3(7.39128f, 6.01062f, 5.8562f));
  points.push_back(Vector3(-8.53572f, -7.56462f, 6.43178f));
  points.push_back(Vector3(-7.19352f, 1.31871f, 4.70077f));
  points.push_back(Vector3(2.32337f, -7.4163f, 3.10892f));
  points.push_back(Vector3(5.5327f, -7.49748f, -9.85046f));
  points.push_back(Vector3(7.70074f, -4.9144f, 8.27876f));
  points.push_back(Vector3(6.27613f, -1.3126f, -5.15122f));
  points.push_back(Vector3(-1.64464f, 5.32762f, 1.53966f));
  points.push_back(Vector3(2.72012f, -3.8786f, -4.10077f));
  points.push_back(Vector3(-5.80981f, 3.73882f, -0.0570697f));
  points.push_back(Vector3(-7.56706f, -5.30686f, 9.6118f));
  points.push_back(Vector3(7.09464f, -4.07025f, -8.42158f));
  points.push_back(Vector3(-9.85168f, -8.05841f, 1.89306f));
  points.push_back(Vector3(-7.47429f, 9.74303f, 9.01425f));
  points.push_back(Vector3(-6.37989f, -3.01492f, -7.99188f));
  points.push_back(Vector3(-8.42219f, 8.97763f, 8.36482f));
  points.push_back(Vector3(5.71276f, 6.34999f, -1.23447f));
  points.push_back(Vector3(-8.58211f, -4.8796f, -5.04013f));
  points.push_back(Vector3(4.82101f, -9.55016f, 7.87652f));
  points.push_back(Vector3(2.80374f, -4.03424f, -6.03504f));
  points.push_back(Vector3(3.55205f, 5.90014f, 9.93347f));
  points.push_back(Vector3(1.39134f, -4.01532f, -3.61675f));
  points.push_back(Vector3(-0.604572f, -2.50587f, -7.47429f));
  TestSphereRitter(points, file);
}

void SphereRitterTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.59777f, 1.16977f, -5.38682f));
  points.push_back(Vector3(2.03284f, -8.44722f, -7.42912f));
  points.push_back(Vector3(-3.95306f, -9.76134f, -8.82565f));
  points.push_back(Vector3(-4.28755f, -3.57952f, -7.74102f));
  points.push_back(Vector3(-1.92785f, 8.99655f, -3.91095f));
  points.push_back(Vector3(-9.63988f, 5.4564f, 2.75735f));
  points.push_back(Vector3(4.1319f, 7.04093f, 6.62282f));
  points.push_back(Vector3(4.73983f, 4.43892f, -0.727256f));
  points.push_back(Vector3(-5.2739f, -6.74123f, -1.28819f));
  points.push_back(Vector3(1.99988f, -5.21226f, -6.30848f));
  points.push_back(Vector3(-0.497146f, 1.52074f, 6.05091f));
  points.push_back(Vector3(3.90728f, -3.58013f, 9.37498f));
  points.push_back(Vector3(2.81045f, 4.2674f, 8.6169f));
  points.push_back(Vector3(-2.63466f, -8.14936f, -2.17139f));
  points.push_back(Vector3(6.70156f, 4.34309f, -5.39415f));
  points.push_back(Vector3(-3.64299f, -2.81716f, -4.63912f));
  points.push_back(Vector3(-7.83196f, -2.2074f, -7.42119f));
  points.push_back(Vector3(3.85418f, 1.28086f, -9.21751f));
  points.push_back(Vector3(0.315866f, -2.5663f, -0.572222f));
  points.push_back(Vector3(-8.08832f, -6.45802f, -2.04505f));
  points.push_back(Vector3(7.14835f, -8.75057f, -3.02103f));
  points.push_back(Vector3(-3.54228f, 5.70666f, -8.15363f));
  points.push_back(Vector3(-9.06247f, -6.3805f, -1.76061f));
  points.push_back(Vector3(-1.2009f, 6.62587f, 6.33778f));
  points.push_back(Vector3(-7.50175f, 3.5081f, -4.13739f));
  points.push_back(Vector3(-8.92514f, -6.6747f, -1.23875f));
  points.push_back(Vector3(8.34224f, 4.58235f, 2.97769f));
  points.push_back(Vector3(-9.74059f, -0.448927f, -7.36869f));
  points.push_back(Vector3(-2.57179f, 0.210882f, 2.48512f));
  points.push_back(Vector3(7.38456f, -3.23099f, 8.623f));
  TestSphereRitter(points, file);
}

void SphereRitterTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.30723f, -1.02512f, 6.51357f));
  points.push_back(Vector3(6.34693f, -9.30174f, 2.48695f));
  points.push_back(Vector3(-6.59841f, 9.38597f, 2.54433f));
  points.push_back(Vector3(7.85028f, 3.24076f, -3.81573f));
  points.push_back(Vector3(-9.10337f, 3.4611f, -6.33778f));
  points.push_back(Vector3(-4.61959f, -4.75692f, 4.65377f));
  points.push_back(Vector3(7.26981f, 3.35551f, 3.87371f));
  points.push_back(Vector3(4.26679f, 2.43019f, 4.35285f));
  points.push_back(Vector3(-3.98358f, -1.27842f, -2.45521f));
  points.push_back(Vector3(-9.83032f, 5.2739f, -9.76806f));
  points.push_back(Vector3(6.92129f, 5.0444f, 7.4633f));
  points.push_back(Vector3(-4.22956f, -3.62957f, -8.64559f));
  points.push_back(Vector3(6.00085f, 3.41777f, -3.34147f));
  points.push_back(Vector3(1.77648f, -2.05176f, 6.72353f));
  points.push_back(Vector3(2.93008f, -6.49892f, -6.14856f));
  points.push_back(Vector3(-2.3075f, -1.76183f, -9.84741f));
  points.push_back(Vector3(-5.51012f, 9.03989f, 9.29869f));
  points.push_back(Vector3(7.02445f, -2.33558f, 6.31336f));
  points.push_back(Vector3(-7.40593f, 5.26109f, -1.84729f));
  points.push_back(Vector3(0.541704f, -1.04038f, -9.38902f));
  points.push_back(Vector3(-6.04785f, 2.0011f, -5.23545f));
  points.push_back(Vector3(2.37709f, 1.93701f, 5.74572f));
  points.push_back(Vector3(7.82342f, 8.17804f, -3.95001f));
  points.push_back(Vector3(9.52147f, -3.84259f, -9.83215f));
  points.push_back(Vector3(-8.83114f, -5.31541f, 7.73675f));
  points.push_back(Vector3(-1.68126f, 5.74999f, -9.2407f));
  points.push_back(Vector3(9.31639f, -3.60454f, 6.39393f));
  points.push_back(Vector3(-5.75671f, -7.49565f, -3.73882f));
  points.push_back(Vector3(7.4456f, 2.30384f, -4.81613f));
  points.push_back(Vector3(6.11805f, 8.76278f, -8.01996f));
  points.push_back(Vector3(6.40248f, 8.32209f, 4.76974f));
  points.push_back(Vector3(5.51805f, -7.12821f, -7.60552f));
  points.push_back(Vector3(7.92718f, -7.54082f, -4.80819f));
  points.push_back(Vector3(-1.14658f, -0.261543f, 9.12107f));
  points.push_back(Vector3(-8.07733f, -0.511795f, -7.57439f));
  points.push_back(Vector3(6.07654f, -0.362255f, 9.70458f));
  points.push_back(Vector3(-5.61144f, 4.26862f, 7.07694f));
  points.push_back(Vector3(-1.43223f, 2.57973f, -9.61242f));
  points.push_back(Vector3(-8.47774f, -7.83441f, -5.07981f));
  points.push_back(Vector3(-7.87408f, -4.46333f, -4.08673f));
  points.push_back(Vector3(5.87817f, 3.94086f, -7.83746f));
  points.push_back(Vector3(-8.56929f, -1.66417f, 3.95123f));
  points.push_back(Vector3(-3.26334f, -6.38295f, -5.12803f));
  points.push_back(Vector3(-7.4041f, -6.57643f, 3.54289f));
  points.push_back(Vector3(1.58239f, -6.33168f, -5.39659f));
  points.push_back(Vector3(2.78787f, 7.56462f, 8.36299f));
  points.push_back(Vector3(1.23631f, 4.85031f, -6.80898f));
  points.push_back(Vector3(2.95022f, 2.32521f, 9.2523f));
  points.push_back(Vector3(-9.70031f, 0.719931f, 1.61107f));
  points.push_back(Vector3(-6.03931f, 7.41874f, -1.20701f));
  points.push_back(Vector3(9.61486f, -8.23908f, 3.15714f));
  points.push_back(Vector3(0.450759f, 9.15586f, 6.19373f));
  points.push_back(Vector3(0.886563f, 3.596f, -7.52373f));
  points.push_back(Vector3(-7.3101f, 7.83685f, -4.03363f));
  points.push_back(Vector3(-1.91076f, 0.531328f, -8.27631f));
  points.push_back(Vector3(-2.26112f, 5.47472f, 1.24363f));
  points.push_back(Vector3(-1.94617f, 2.13477f, -4.24177f));
  points.push_back(Vector3(-0.925626f, 1.20518f, -5.33799f));
  points.push_back(Vector3(-3.5081f, 9.15708f, -7.57012f));
  points.push_back(Vector3(1.61168f, -1.34678f, -1.00436f));
  points.push_back(Vector3(-2.10547f, 8.44172f, 4.00372f));
  points.push_back(Vector3(-1.09897f, 9.88098f, -1.33579f));
  points.push_back(Vector3(-9.07895f, -3.01065f, 4.28205f));
  points.push_back(Vector3(-6.13025f, -3.16507f, 4.38459f));
  points.push_back(Vector3(7.50786f, 5.50401f, 5.3856f));
  points.push_back(Vector3(6.23035f, 5.92883f, -0.968353f));
  points.push_back(Vector3(-3.56548f, 4.15754f, 0.989105f));
  points.push_back(Vector3(1.10935f, 7.05863f, 5.81896f));
  points.push_back(Vector3(3.10343f, -9.61547f, 1.63427f));
  points.push_back(Vector3(7.11478f, 7.47124f, 2.5602f));
  points.push_back(Vector3(-0.842006f, 2.7427f, 4.9266f));
  TestSphereRitter(points, file);
}

void SphereRitterTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.08994f, -5.95202f, -5.01328f));
  points.push_back(Vector3(7.23868f, -3.09061f, 3.726f));
  points.push_back(Vector3(0.467849f, -5.36119f, 1.03305f));
  points.push_back(Vector3(-1.17038f, -6.59963f, -0.885342f));
  points.push_back(Vector3(3.75896f, 4.26069f, -9.93103f));
  points.push_back(Vector3(8.85495f, -0.600299f, -9.54772f));
  points.push_back(Vector3(9.06491f, 8.41243f, 3.32804f));
  points.push_back(Vector3(-5.81286f, 1.34129f, -8.97824f));
  points.push_back(Vector3(7.528f, -5.5565f, -4.83322f));
  points.push_back(Vector3(6.27003f, -9.9115f, 6.41652f));
  points.push_back(Vector3(6.56911f, -8.764f, -5.72863f));
  points.push_back(Vector3(6.81936f, -4.07697f, 3.24381f));
  points.push_back(Vector3(2.76589f, -6.39454f, 1.29307f));
  points.push_back(Vector3(2.32948f, 2.77444f, -6.77358f));
  points.push_back(Vector3(9.15586f, 1.5714f, 7.53594f));
  points.push_back(Vector3(-1.44688f, -6.86453f, -6.61306f));
  points.push_back(Vector3(9.92248f, -9.41343f, 1.36998f));
  points.push_back(Vector3(9.93957f, -6.26881f, 7.08609f));
  points.push_back(Vector3(3.13089f, 5.7622f, -1.70324f));
  points.push_back(Vector3(-8.76888f, -4.66659f, 0.745567f));
  points.push_back(Vector3(2.04627f, 3.45256f, -0.450148f));
  points.push_back(Vector3(-8.40938f, 0.628986f, 9.54649f));
  points.push_back(Vector3(-9.84497f, 9.86938f, 5.23789f));
  points.push_back(Vector3(8.211f, 4.23322f, -5.1915f));
  points.push_back(Vector3(-3.24808f, -8.88852f, -6.13819f));
  points.push_back(Vector3(-1.06784f, 1.32725f, -1.37852f));
  points.push_back(Vector3(-8.58699f, -5.3325f, 3.05338f));
  points.push_back(Vector3(4.34126f, 8.47713f, 0.0674462f));
  points.push_back(Vector3(-0.693075f, -8.46248f, -0.951262f));
  points.push_back(Vector3(-8.2519f, -5.57115f, -5.37645f));
  points.push_back(Vector3(7.5988f, 3.18461f, -2.65847f));
  points.push_back(Vector3(8.79574f, -7.35282f, -3.52153f));
  points.push_back(Vector3(-1.26194f, 0.965301f, 6.47755f));
  points.push_back(Vector3(5.7329f, -6.53066f, 5.10361f));
  points.push_back(Vector3(1.59703f, 0.148015f, 3.75164f));
  TestSphereRitter(points, file);
}

void SphereRitterTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.61425f, 8.6877f, 7.38639f));
  points.push_back(Vector3(4.53658f, 1.1887f, -6.60268f));
  points.push_back(Vector3(0.84933f, 4.5323f, -9.44884f));
  points.push_back(Vector3(5.7329f, -2.85745f, -7.95587f));
  points.push_back(Vector3(9.97253f, 8.20368f, 7.72942f));
  points.push_back(Vector3(5.70177f, -5.4912f, -7.20939f));
  points.push_back(Vector3(-4.80697f, -7.82586f, -9.1174f));
  points.push_back(Vector3(9.14975f, -8.8757f, 5.02182f));
  points.push_back(Vector3(3.42265f, -0.738853f, -5.33372f));
  points.push_back(Vector3(1.55248f, 5.99414f, -5.8446f));
  points.push_back(Vector3(4.85031f, -1.68615f, -1.97241f));
  points.push_back(Vector3(-1.67455f, 9.84741f, -4.15876f));
  points.push_back(Vector3(-7.2277f, -2.53578f, 6.25843f));
  points.push_back(Vector3(-6.71194f, -4.09955f, -4.85092f));
  points.push_back(Vector3(-8.4344f, 3.56731f, 2.59255f));
  points.push_back(Vector3(-5.90808f, 2.87881f, 9.61974f));
  points.push_back(Vector3(-8.78109f, 7.58782f, 5.08774f));
  points.push_back(Vector3(-7.55303f, 1.75329f, -3.21879f));
  points.push_back(Vector3(1.56102f, -8.4753f, -2.96854f));
  points.push_back(Vector3(-0.615559f, -9.49461f, 8.22077f));
  points.push_back(Vector3(-5.00595f, 5.45518f, 0.113834f));
  points.push_back(Vector3(-7.15445f, -9.97742f, 5.88f));
  points.push_back(Vector3(5.62914f, 5.97278f, -8.49055f));
  points.push_back(Vector3(9.17051f, 1.44566f, -5.3795f));
  points.push_back(Vector3(-2.22327f, -7.78985f, -8.62362f));
  points.push_back(Vector3(-6.03442f, -6.53371f, 0.569781f));
  points.push_back(Vector3(-9.7943f, -4.83261f, 8.54427f));
  points.push_back(Vector3(4.0141f, -0.336619f, 9.96704f));
  points.push_back(Vector3(7.01651f, 2.83548f, -8.07428f));
  points.push_back(Vector3(-7.34123f, 3.50871f, -9.36949f));
  points.push_back(Vector3(3.79559f, -9.92431f, -4.185f));
  points.push_back(Vector3(9.14365f, -5.34227f, 0.202948f));
  points.push_back(Vector3(-8.44844f, 0.808436f, -8.64193f));
  points.push_back(Vector3(-9.94873f, -2.96426f, -5.38621f));
  points.push_back(Vector3(-6.88345f, -6.58193f, 6.51723f));
  points.push_back(Vector3(-9.09543f, -0.393384f, -0.856044f));
  points.push_back(Vector3(-0.115055f, 0.76632f, -4.67818f));
  points.push_back(Vector3(-9.48485f, 6.97867f, 5.59069f));
  points.push_back(Vector3(-6.64174f, 6.41835f, 6.61977f));
  points.push_back(Vector3(-7.52678f, -8.26472f, 4.87289f));
  points.push_back(Vector3(-1.49693f, -8.94833f, 5.38072f));
  points.push_back(Vector3(-0.415967f, -3.94024f, 7.41264f));
  points.push_back(Vector3(6.9042f, -9.88464f, -1.57994f));
  points.push_back(Vector3(0.694907f, -0.715049f, -3.67351f));
  points.push_back(Vector3(9.60875f, -1.98218f, -6.40004f));
  points.push_back(Vector3(-2.3075f, 3.0839f, -6.65273f));
  points.push_back(Vector3(-5.08591f, 8.13044f, -0.237129f));
  points.push_back(Vector3(5.93738f, 9.46104f, 6.9573f));
  points.push_back(Vector3(-2.36122f, 3.96344f, -3.76385f));
  points.push_back(Vector3(-3.39091f, 1.13254f, 8.9819f));
  points.push_back(Vector3(-3.86883f, 7.74224f, 8.28913f));
  points.push_back(Vector3(6.82547f, 1.73131f, 2.33619f));
  points.push_back(Vector3(3.9555f, 0.579547f, 0.901212f));
  points.push_back(Vector3(-4.55977f, -2.94046f, -8.75362f));
  points.push_back(Vector3(-2.71462f, 6.16871f, -9.88891f));
  points.push_back(Vector3(1.73193f, 0.849941f, -1.4182f));
  points.push_back(Vector3(-0.936003f, 9.77721f, -8.41914f));
  points.push_back(Vector3(3.07779f, -8.36055f, 5.4619f));
  points.push_back(Vector3(9.48424f, -1.0886f, 2.27027f));
  points.push_back(Vector3(-1.09775f, -6.0155f, 8.47652f));
  points.push_back(Vector3(-8.24702f, -6.23035f, 5.87939f));
  points.push_back(Vector3(3.9671f, 9.76623f, -7.74834f));
  points.push_back(Vector3(-5.05905f, -2.95145f, 8.35505f));
  points.push_back(Vector3(7.08792f, 6.3213f, -7.84051f));
  points.push_back(Vector3(4.52437f, 6.29688f, 8.86166f));
  points.push_back(Vector3(-5.92517f, 0.0546277f, 6.22913f));
  points.push_back(Vector3(-1.55492f, -9.65148f, -3.3311f));
  points.push_back(Vector3(-1.44261f, -0.48555f, -0.735801f));
  points.push_back(Vector3(-5.31907f, 6.845f, -2.12378f));
  points.push_back(Vector3(3.92193f, -6.76077f, -1.69652f));
  points.push_back(Vector3(7.91864f, 3.0784f, 0.301828f));
  points.push_back(Vector3(6.37562f, 7.37236f, -0.627155f));
  points.push_back(Vector3(-9.54161f, 9.7589f, 2.7311f));
  points.push_back(Vector3(-4.43892f, -1.26438f, 3.50383f));
  points.push_back(Vector3(-1.53722f, -0.621052f, 8.49666f));
  points.push_back(Vector3(7.80389f, -6.8627f, 7.66839f));
  points.push_back(Vector3(1.57018f, -9.89196f, -4.41023f));
  points.push_back(Vector3(-9.61974f, 8.73592f, 6.17908f));
  points.push_back(Vector3(4.24665f, 5.09445f, 8.67672f));
  points.push_back(Vector3(-4.28327f, 2.74758f, 2.65786f));
  points.push_back(Vector3(-6.46046f, -1.8131f, -5.02426f));
  points.push_back(Vector3(-5.40086f, 9.22483f, 4.45906f));
  points.push_back(Vector3(-7.37419f, 2.90933f, 8.51619f));
  points.push_back(Vector3(9.89807f, -1.55553f, 2.83975f));
  points.push_back(Vector3(5.20005f, 4.5201f, -1.75146f));
  points.push_back(Vector3(-0.669271f, 7.28019f, 5.85009f));
  points.push_back(Vector3(4.86618f, 7.69524f, -2.58217f));
  points.push_back(Vector3(-9.90722f, 7.11356f, -0.809045f));
  points.push_back(Vector3(-1.27293f, 7.25456f, 6.93472f));
  points.push_back(Vector3(2.0951f, -5.95874f, -4.16791f));
  points.push_back(Vector3(-2.1659f, -1.08615f, -3.20963f));
  points.push_back(Vector3(2.69814f, 7.71966f, -9.65087f));
  points.push_back(Vector3(1.59337f, 3.80718f, -0.71749f));
  points.push_back(Vector3(6.65273f, -7.28813f, -8.65474f));
  points.push_back(Vector3(-8.94772f, -3.02408f, -1.5244f));
  points.push_back(Vector3(2.9545f, -7.77398f, -2.62978f));
  points.push_back(Vector3(9.80773f, -1.35655f, -5.57787f));
  TestSphereRitter(points, file);
}

void SphereRitterTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.20673f, 0.263985f, 5.20432f));
  points.push_back(Vector3(-6.38478f, 5.82385f, 5.45763f));
  points.push_back(Vector3(2.0304f, 3.14554f, 5.53148f));
  points.push_back(Vector3(-3.52031f, -6.59169f, -8.9758f));
  points.push_back(Vector3(9.79553f, -1.84912f, -9.48668f));
  points.push_back(Vector3(-3.17301f, -9.39818f, -7.62932f));
  points.push_back(Vector3(4.06476f, 9.14792f, 9.9176f));
  points.push_back(Vector3(0.0125122f, 1.13681f, 7.669f));
  points.push_back(Vector3(-4.03668f, 4.58052f, -3.30607f));
  points.push_back(Vector3(1.05991f, -7.95648f, -1.75268f));
  points.push_back(Vector3(-2.01819f, 6.31031f, 1.02451f));
  points.push_back(Vector3(0.89816f, 9.88403f, 5.14206f));
  points.push_back(Vector3(-1.3889f, -7.19108f, -0.606403f));
  points.push_back(Vector3(3.11014f, -1.46458f, -8.41487f));
  points.push_back(Vector3(7.30155f, 7.03177f, 2.78237f));
  points.push_back(Vector3(-6.02405f, 5.24155f, 6.34999f));
  points.push_back(Vector3(-0.0186163f, -6.79495f, 6.2096f));
  points.push_back(Vector3(-5.9209f, 5.08591f, -7.57195f));
  points.push_back(Vector3(6.68691f, -7.06351f, 3.46965f));
  points.push_back(Vector3(4.95102f, 8.48506f, 2.29957f));
  points.push_back(Vector3(-7.44377f, 1.82775f, -1.55187f));
  points.push_back(Vector3(-8.39473f, 4.12214f, -2.52846f));
  points.push_back(Vector3(-5.87146f, -4.17585f, 0.375683f));
  points.push_back(Vector3(-6.10584f, -6.43422f, -1.37669f));
  points.push_back(Vector3(6.96524f, -5.79638f, -2.71157f));
  points.push_back(Vector3(-1.14414f, -3.98724f, 5.98682f));
  points.push_back(Vector3(0.671102f, 5.99292f, 2.32093f));
  points.push_back(Vector3(5.19639f, 4.76791f, 0.122379f));
  points.push_back(Vector3(5.86535f, 1.1771f, 3.74615f));
  points.push_back(Vector3(2.15369f, 3.69793f, -6.55873f));
  points.push_back(Vector3(-2.95389f, 9.82788f, 4.39863f));
  points.push_back(Vector3(0.83346f, 4.39436f, 5.18662f));
  points.push_back(Vector3(8.1811f, -6.29994f, -8.20856f));
  points.push_back(Vector3(9.1052f, 5.79028f, 9.69359f));
  points.push_back(Vector3(6.60451f, 8.51802f, -0.679037f));
  points.push_back(Vector3(3.8316f, 9.87976f, -8.0285f));
  points.push_back(Vector3(-1.93518f, -6.55568f, 4.01471f));
  points.push_back(Vector3(-0.880459f, -7.47124f, 8.62239f));
  points.push_back(Vector3(3.90423f, -0.217597f, -0.260323f));
  points.push_back(Vector3(8.80306f, 9.68261f, 8.21528f));
  points.push_back(Vector3(2.96304f, -3.88775f, 9.19981f));
  points.push_back(Vector3(8.18049f, -1.03793f, 6.09058f));
  points.push_back(Vector3(8.34529f, -5.51927f, -4.07941f));
  points.push_back(Vector3(6.42445f, -2.65053f, 6.55141f));
  points.push_back(Vector3(-4.84787f, 4.31318f, -8.81283f));
  points.push_back(Vector3(-0.248726f, -0.270699f, -6.90847f));
  points.push_back(Vector3(-7.7929f, 9.97742f, 3.1431f));
  points.push_back(Vector3(-1.26194f, -6.89016f, -2.78054f));
  points.push_back(Vector3(1.13681f, -3.1138f, 8.40938f));
  points.push_back(Vector3(3.68877f, -0.179754f, 0.974456f));
  points.push_back(Vector3(-7.51091f, -7.85882f, 3.20231f));
  points.push_back(Vector3(-7.85211f, -7.60857f, 0.0192261f));
  points.push_back(Vector3(-6.73757f, -8.45149f, -6.5508f));
  points.push_back(Vector3(-4.96017f, 5.68651f, 5.61022f));
  points.push_back(Vector3(5.90503f, 7.85638f, -5.61449f));
  points.push_back(Vector3(4.98459f, -9.37742f, 7.40715f));
  points.push_back(Vector3(7.94122f, 7.73064f, -7.9693f));
  points.push_back(Vector3(-5.40452f, -8.92331f, 7.38029f));
  points.push_back(Vector3(-3.58257f, -0.0930816f, -5.6975f));
  points.push_back(Vector3(-3.48308f, 3.69793f, 0.596637f));
  points.push_back(Vector3(4.84237f, 6.4861f, -9.21506f));
  points.push_back(Vector3(3.0784f, 1.98035f, -6.14246f));
  points.push_back(Vector3(-5.30686f, 6.13025f, 9.94324f));
  points.push_back(Vector3(-5.71642f, -6.51784f, -9.45921f));
  points.push_back(Vector3(-8.55281f, -5.65172f, -6.81387f));
  points.push_back(Vector3(-9.16379f, -4.96323f, 2.06519f));
  points.push_back(Vector3(-1.16001f, -5.30686f, -0.66805f));
  points.push_back(Vector3(6.62038f, -2.26478f, 5.67675f));
  points.push_back(Vector3(7.32047f, 8.00836f, -6.97439f));
  points.push_back(Vector3(-5.0792f, -6.19251f, 7.16788f));
  points.push_back(Vector3(-4.19111f, -2.61513f, -0.833461f));
  points.push_back(Vector3(-2.74453f, -6.18763f, -1.70324f));
  points.push_back(Vector3(-7.51518f, 6.67715f, 4.80148f));
  points.push_back(Vector3(5.42406f, -2.7311f, -5.50462f));
  points.push_back(Vector3(-7.1569f, -7.21549f, 2.79885f));
  points.push_back(Vector3(8.85556f, -4.4554f, -8.56563f));
  points.push_back(Vector3(-5.72069f, 9.03561f, 0.394604f));
  points.push_back(Vector3(-7.05741f, 4.90341f, 1.75817f));
  points.push_back(Vector3(-1.67211f, 1.19297f, 2.95145f));
  points.push_back(Vector3(-7.61589f, -7.35466f, -1.87353f));
  points.push_back(Vector3(-2.93985f, 1.61595f, -8.59066f));
  points.push_back(Vector3(-0.940275f, -7.22282f, 8.3282f));
  points.push_back(Vector3(1.44017f, 3.19254f, 3.52947f));
  points.push_back(Vector3(9.1644f, -7.96319f, -5.06272f));
  points.push_back(Vector3(9.13205f, -4.49141f, 6.55629f));
  TestSphereRitter(points, file);
}

void SphereRitterTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.17072f, -1.65075f, 1.4658f));
  points.push_back(Vector3(-2.01697f, -0.857265f, 8.59798f));
  points.push_back(Vector3(-3.77544f, -9.01486f, 6.9335f));
  points.push_back(Vector3(8.93551f, -8.2165f, -8.49178f));
  points.push_back(Vector3(-6.97989f, -0.0656146f, -5.67614f));
  points.push_back(Vector3(7.30949f, 5.23362f, -5.20371f));
  points.push_back(Vector3(4.17768f, 7.59758f, -4.66842f));
  points.push_back(Vector3(-9.90905f, 6.09424f, 4.73373f));
  points.push_back(Vector3(-3.60332f, 6.07593f, -9.20347f));
  points.push_back(Vector3(2.06702f, 1.82958f, 2.20435f));
  points.push_back(Vector3(-9.04904f, -1.47435f, -5.92517f));
  points.push_back(Vector3(3.45683f, 4.67269f, -9.40916f));
  points.push_back(Vector3(1.19602f, 3.11563f, 9.70031f));
  points.push_back(Vector3(2.28431f, 9.42442f, -2.38685f));
  points.push_back(Vector3(1.35716f, 3.77728f, 9.81323f));
  points.push_back(Vector3(-0.927457f, -2.57363f, 9.27f));
  points.push_back(Vector3(7.11722f, 1.16733f, 3.06558f));
  TestSphereRitter(points, file);
}

void SphereRitterTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.68343f, -7.64946f, -0.714439f));
  points.push_back(Vector3(3.40312f, 1.19541f, -7.35038f));
  points.push_back(Vector3(8.0047f, -9.8413f, -3.08084f));
  points.push_back(Vector3(-6.67715f, -5.01572f, 4.59944f));
  points.push_back(Vector3(0.308542f, 4.47188f, -6.74612f));
  points.push_back(Vector3(7.65069f, -1.44627f, 4.60921f));
  points.push_back(Vector3(-2.9899f, -2.49062f, 4.69222f));
  points.push_back(Vector3(1.74596f, -9.13694f, 4.42183f));
  points.push_back(Vector3(5.76952f, -8.60836f, 8.33125f));
  points.push_back(Vector3(-2.31483f, -5.67003f, 4.59578f));
  points.push_back(Vector3(0.22187f, 3.00882f, 8.84884f));
  points.push_back(Vector3(9.14914f, 3.5258f, -1.66112f));
  points.push_back(Vector3(-1.26926f, -2.92337f, -8.56624f));
  points.push_back(Vector3(6.30116f, -0.48677f, 1.54515f));
  points.push_back(Vector3(-4.88144f, -8.64559f, -6.12049f));
  points.push_back(Vector3(-3.68999f, -2.10242f, -0.191962f));
  points.push_back(Vector3(4.1258f, 0.821863f, -7.59819f));
  points.push_back(Vector3(-9.19309f, -8.30866f, 4.12885f));
  points.push_back(Vector3(1.38218f, -3.57524f, 9.39268f));
  points.push_back(Vector3(-9.31333f, -7.2338f, -1.02206f));
  points.push_back(Vector3(1.96509f, 1.21433f, 9.60692f));
  points.push_back(Vector3(3.69671f, 0.56734f, -8.29402f));
  points.push_back(Vector3(5.52904f, -5.13169f, 6.83645f));
  points.push_back(Vector3(3.74004f, 6.31886f, -6.41469f));
  points.push_back(Vector3(-7.84539f, 9.19492f, 8.26167f));
  points.push_back(Vector3(5.21775f, 7.72881f, 2.50954f));
  points.push_back(Vector3(-6.12964f, -5.26414f, -8.88729f));
  points.push_back(Vector3(3.60637f, -4.38215f, -1.98401f));
  points.push_back(Vector3(4.81002f, 4.60006f, 8.30012f));
  points.push_back(Vector3(7.23991f, -7.44255f, 3.42509f));
  points.push_back(Vector3(0.662557f, -1.47679f, 8.94345f));
  points.push_back(Vector3(-2.50099f, 6.90542f, -0.0985748f));
  TestSphereRitter(points, file);
}

void SphereRitterTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.64415f, -9.7174f, 3.83465f));
  points.push_back(Vector3(4.23017f, 6.75649f, -2.55287f));
  points.push_back(Vector3(-7.64641f, -8.66817f, -0.616169f));
  points.push_back(Vector3(2.02002f, -1.50304f, -8.35933f));
  points.push_back(Vector3(7.28507f, -3.28288f, 4.55367f));
  points.push_back(Vector3(6.96707f, 8.98312f, 4.1026f));
  points.push_back(Vector3(-1.56774f, -3.20048f, 6.37074f));
  points.push_back(Vector3(7.41752f, -6.34083f, 0.801111f));
  points.push_back(Vector3(9.72838f, -1.98523f, -0.419629f));
  points.push_back(Vector3(-9.28709f, -5.49425f, 7.52495f));
  points.push_back(Vector3(-2.62429f, 4.80148f, -9.51476f));
  points.push_back(Vector3(2.85989f, 7.52373f, 7.96686f));
  points.push_back(Vector3(-6.04907f, -7.79107f, 4.33638f));
  points.push_back(Vector3(-9.35728f, 3.49345f, 0.126041f));
  points.push_back(Vector3(3.18644f, 9.61242f, -9.93347f));
  points.push_back(Vector3(-0.154729f, -3.94208f, 3.85296f));
  points.push_back(Vector3(1.93945f, 0.952482f, 1.04099f));
  points.push_back(Vector3(8.56014f, -4.8619f, 0.528276f));
  points.push_back(Vector3(1.39561f, 0.447096f, -9.96094f));
  points.push_back(Vector3(-4.10504f, -0.188299f, -2.53212f));
  points.push_back(Vector3(4.70626f, 7.09037f, 3.49223f));
  points.push_back(Vector3(6.07349f, 5.61571f, 7.19169f));
  points.push_back(Vector3(5.67553f, -0.86581f, 6.39271f));
  points.push_back(Vector3(8.6169f, -0.76632f, -2.34779f));
  points.push_back(Vector3(-4.8265f, 3.52824f, 7.96503f));
  points.push_back(Vector3(-0.614337f, 1.23142f, 6.46046f));
  points.push_back(Vector3(-8.79818f, 7.51579f, 1.48106f));
  points.push_back(Vector3(-2.69692f, 7.37358f, 2.47108f));
  points.push_back(Vector3(-2.50587f, -7.24235f, -7.87225f));
  points.push_back(Vector3(-9.95117f, 3.64666f, 4.39558f));
  points.push_back(Vector3(-3.34147f, 8.94833f, -2.97891f));
  points.push_back(Vector3(-8.55586f, 1.84484f, -9.16684f));
  points.push_back(Vector3(9.69726f, 6.4275f, -6.15833f));
  points.push_back(Vector3(-3.92254f, -5.95508f, 1.77221f));
  points.push_back(Vector3(-3.26029f, -1.7185f, 1.65563f));
  TestSphereRitter(points, file);
}

void SphereRitterTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.04172f, -8.50459f, -3.3311f));
  points.push_back(Vector3(-0.27131f, 8.00226f, 1.24607f));
  points.push_back(Vector3(6.06067f, -8.24152f, 2.902f));
  points.push_back(Vector3(8.7701f, -8.76644f, -0.24018f));
  points.push_back(Vector3(8.05902f, 6.63869f, -4.85031f));
  points.push_back(Vector3(2.83059f, -7.75933f, 0.353709f));
  points.push_back(Vector3(-4.11847f, -6.98843f, -9.45311f));
  points.push_back(Vector3(0.959197f, 1.28513f, -6.22547f));
  points.push_back(Vector3(1.43834f, -3.56243f, 7.304f));
  points.push_back(Vector3(7.61284f, -3.53008f, 5.02609f));
  points.push_back(Vector3(-9.47203f, 0.856655f, -1.66051f));
  points.push_back(Vector3(3.56426f, -4.35896f, 3.8432f));
  points.push_back(Vector3(-7.12088f, -5.98621f, 8.36848f));
  points.push_back(Vector3(-5.4387f, -9.37437f, 1.26438f));
  points.push_back(Vector3(-5.43687f, 3.89874f, 3.57646f));
  points.push_back(Vector3(5.34837f, -3.95123f, -9.38719f));
  points.push_back(Vector3(-7.23075f, 1.20273f, 3.84442f));
  points.push_back(Vector3(4.15876f, -7.68548f, 9.09116f));
  points.push_back(Vector3(-0.604572f, 3.48735f, 5.48448f));
  points.push_back(Vector3(8.81466f, 4.44319f, 9.94995f));
  points.push_back(Vector3(-1.49266f, 0.835901f, -9.85534f));
  points.push_back(Vector3(2.42653f, 7.68059f, 7.44804f));
  points.push_back(Vector3(6.08203f, -7.18986f, 5.42589f));
  points.push_back(Vector3(-2.90384f, -0.227363f, 9.95911f));
  points.push_back(Vector3(0.917081f, 5.00412f, 7.32108f));
  points.push_back(Vector3(-9.74059f, 0.903653f, -6.80349f));
  points.push_back(Vector3(5.82018f, 2.91116f, 6.35426f));
  points.push_back(Vector3(3.0784f, 8.14081f, 8.06146f));
  points.push_back(Vector3(-9.45738f, 3.66314f, -9.27488f));
  points.push_back(Vector3(-4.07636f, -5.79089f, 8.60103f));
  points.push_back(Vector3(-4.98215f, 4.98886f, 3.41227f));
  points.push_back(Vector3(5.11399f, -6.85171f, 7.46941f));
  points.push_back(Vector3(5.70544f, 7.97601f, -1.49693f));
  points.push_back(Vector3(0.0918603f, -9.74548f, 1.65014f));
  points.push_back(Vector3(-8.19941f, 8.36665f, 7.52983f));
  points.push_back(Vector3(3.72356f, 3.30119f, 2.46071f));
  points.push_back(Vector3(-5.81286f, -7.25272f, 1.77343f));
  points.push_back(Vector3(-7.20511f, -1.2186f, -6.37806f));
  points.push_back(Vector3(-9.15403f, 3.49406f, -6.09241f));
  points.push_back(Vector3(3.64421f, 3.36528f, -8.18781f));
  points.push_back(Vector3(-9.57091f, -7.65618f, -5.61632f));
  points.push_back(Vector3(-8.33796f, -0.607623f, -1.7655f));
  points.push_back(Vector3(3.41044f, 9.36888f, -2.73782f));
  points.push_back(Vector3(-4.39802f, 6.54347f, 9.77538f));
  points.push_back(Vector3(0.1059f, -5.54247f, 9.10703f));
  points.push_back(Vector3(0.824305f, 2.34352f, 6.78457f));
  points.push_back(Vector3(-0.577105f, -5.25498f, 5.69811f));
  points.push_back(Vector3(2.18909f, -5.65661f, -6.0387f));
  points.push_back(Vector3(-6.03809f, 6.10096f, -2.83792f));
  points.push_back(Vector3(-0.144352f, -0.788903f, 3.63018f));
  points.push_back(Vector3(-1.46825f, -7.49504f, -4.91195f));
  points.push_back(Vector3(2.21839f, 1.02206f, -0.475173f));
  points.push_back(Vector3(2.78176f, -3.79986f, -2.07129f));
  TestSphereRitter(points, file);
}

void SphereRitterTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.29319f, 4.89669f, 2.83792f));
  points.push_back(Vector3(9.15647f, -5.23301f, -0.736412f));
  points.push_back(Vector3(6.69973f, -3.12113f, -6.91092f));
  points.push_back(Vector3(1.583f, 2.60842f, -3.62774f));
  points.push_back(Vector3(5.48448f, -5.84521f, 8.47774f));
  points.push_back(Vector3(9.45738f, 9.82238f, 3.37504f));
  points.push_back(Vector3(-4.9089f, 3.86395f, -7.35466f));
  points.push_back(Vector3(-9.59166f, 6.58071f, -5.67797f));
  points.push_back(Vector3(-1.59276f, 5.97034f, 8.58516f));
  points.push_back(Vector3(0.561845f, 7.96869f, -2.67373f));
  points.push_back(Vector3(-0.257271f, -4.65011f, 6.98355f));
  points.push_back(Vector3(5.65416f, -7.87835f, 5.69628f));
  points.push_back(Vector3(-5.21653f, 1.87048f, -3.77789f));
  points.push_back(Vector3(0.343944f, -5.59496f, -0.932341f));
  points.push_back(Vector3(-3.82611f, 6.6802f, -5.63646f));
  points.push_back(Vector3(9.67528f, -9.24558f, -9.8236f));
  points.push_back(Vector3(2.82083f, 5.46922f, 5.85376f));
  points.push_back(Vector3(-0.494705f, 4.75143f, 8.56502f));
  points.push_back(Vector3(-5.99536f, -9.56969f, 2.74941f));
  points.push_back(Vector3(-8.49666f, 6.61733f, -8.37092f));
  points.push_back(Vector3(-7.91314f, -5.42894f, -5.47838f));
  points.push_back(Vector3(5.53758f, -9.34019f, -1.53356f));
  points.push_back(Vector3(2.46864f, 7.3693f, -5.562f));
  points.push_back(Vector3(0.445875f, -6.8102f, -4.57991f));
  points.push_back(Vector3(-0.949431f, -7.93023f, 6.6625f));
  points.push_back(Vector3(-4.21125f, -1.87353f, 6.67776f));
  points.push_back(Vector3(-9.81567f, -5.36668f, -0.976897f));
  points.push_back(Vector3(-5.2916f, 4.90707f, 1.09348f));
  points.push_back(Vector3(6.55568f, -4.99313f, 4.42305f));
  points.push_back(Vector3(-5.9978f, -4.63485f, 0.223701f));
  points.push_back(Vector3(-8.93368f, -4.48531f, 1.54027f));
  points.push_back(Vector3(4.4438f, 5.51866f, 4.7673f));
  points.push_back(Vector3(-5.87451f, -8.82992f, -4.63363f));
  points.push_back(Vector3(-5.2916f, -2.86599f, 9.78088f));
  points.push_back(Vector3(-1.76794f, 5.21836f, 2.69448f));
  points.push_back(Vector3(-1.41636f, 5.09384f, -4.3614f));
  points.push_back(Vector3(5.95752f, 2.60964f, -0.571001f));
  points.push_back(Vector3(9.84313f, 2.38014f, 4.24238f));
  points.push_back(Vector3(-2.19703f, -3.02713f, -1.97485f));
  points.push_back(Vector3(-5.16221f, 7.528f, 0.961028f));
  points.push_back(Vector3(-6.51418f, 5.43565f, 7.01651f));
  points.push_back(Vector3(1.96936f, -7.92413f, 4.70321f));
  points.push_back(Vector3(4.45662f, 2.86721f, -7.10746f));
  points.push_back(Vector3(2.22816f, 0.430006f, 3.87066f));
  points.push_back(Vector3(-0.600299f, 5.28916f, -9.89624f));
  points.push_back(Vector3(-1.52623f, 4.1319f, 7.22709f));
  points.push_back(Vector3(-9.38597f, -6.20777f, 0.165105f));
  points.push_back(Vector3(-4.45845f, 9.27366f, 8.44783f));
  points.push_back(Vector3(-0.0332654f, -2.15491f, -1.78137f));
  points.push_back(Vector3(-8.88668f, -7.75018f, -6.2157f));
  points.push_back(Vector3(-8.8641f, -7.14713f, -2.5309f));
  points.push_back(Vector3(-8.93002f, 7.6751f, -3.6314f));
  points.push_back(Vector3(-7.43461f, -6.26759f, 1.18503f));
  points.push_back(Vector3(-4.70748f, 5.59923f, 0.726036f));
  points.push_back(Vector3(-4.60128f, 6.63869f, -9.11252f));
  points.push_back(Vector3(3.01187f, 5.68468f, 9.54649f));
  points.push_back(Vector3(5.28123f, -2.75796f, -9.50621f));
  points.push_back(Vector3(-6.5624f, -3.81451f, 4.17097f));
  points.push_back(Vector3(8.62911f, 8.71334f, 1.98279f));
  points.push_back(Vector3(9.91699f, 7.95648f, 3.11014f));
  points.push_back(Vector3(-4.55367f, -1.8009f, 1.19114f));
  points.push_back(Vector3(-3.14798f, -9.2053f, -8.31721f));
  points.push_back(Vector3(-5.62548f, -0.285959f, 5.00595f));
  points.push_back(Vector3(-9.4586f, 3.67718f, 3.12174f));
  points.push_back(Vector3(-5.99353f, 6.37684f, 6.13697f));
  points.push_back(Vector3(-9.58129f, -5.62243f, -0.386059f));
  points.push_back(Vector3(-0.0204474f, -1.57811f, -8.81466f));
  points.push_back(Vector3(-1.50182f, 0.128483f, -6.61061f));
  points.push_back(Vector3(2.57729f, 7.81854f, 4.5323f));
  points.push_back(Vector3(-5.468f, 1.75451f, 5.48082f));
  points.push_back(Vector3(-3.05826f, 8.18232f, 6.95486f));
  points.push_back(Vector3(-4.93942f, 1.75878f, 5.75671f));
  points.push_back(Vector3(0.070498f, -7.54509f, -3.4257f));
  points.push_back(Vector3(8.63216f, 4.5201f, -5.23667f));
  points.push_back(Vector3(-1.96875f, -6.94082f, 3.00455f));
  points.push_back(Vector3(9.5172f, 6.6332f, -1.75634f));
  points.push_back(Vector3(5.77197f, -7.61528f, 6.11927f));
  points.push_back(Vector3(-9.29014f, -9.98535f, -8.51253f));
  points.push_back(Vector3(3.03873f, 8.08588f, 1.02512f));
  points.push_back(Vector3(6.42994f, 2.14393f, -4.5793f));
  points.push_back(Vector3(-4.26923f, 0.187079f, 3.22794f));
  points.push_back(Vector3(-2.82571f, -0.758385f, -8.02973f));
  points.push_back(Vector3(-4.9675f, -2.01209f, -4.97299f));
  points.push_back(Vector3(9.19248f, -8.14447f, -4.62813f));
  points.push_back(Vector3(-0.645466f, 6.65761f, 8.47163f));
  points.push_back(Vector3(-2.56569f, 0.603961f, -8.89218f));
  points.push_back(Vector3(-0.99826f, 8.77133f, -5.90136f));
  points.push_back(Vector3(-0.760216f, -4.64339f, -2.35267f));
  TestSphereRitter(points, file);
}

void SphereRitterTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.26228f, -3.42448f, -0.0918609f));
  points.push_back(Vector3(2.37709f, 9.54955f, -1.64953f));
  points.push_back(Vector3(8.83908f, -6.82791f, 3.35124f));
  points.push_back(Vector3(1.54943f, -4.06354f, -9.72838f));
  points.push_back(Vector3(-8.55525f, -4.62325f, -8.74691f));
  points.push_back(Vector3(0.522171f, -7.71294f, -2.51381f));
  points.push_back(Vector3(-7.22221f, 7.11478f, -5.17197f));
  points.push_back(Vector3(-4.55489f, 4.53536f, 7.79107f));
  points.push_back(Vector3(6.40126f, -4.41267f, -7.46574f));
  points.push_back(Vector3(-0.506912f, 1.2656f, 9.86999f));
  points.push_back(Vector3(6.69607f, 5.08713f, 2.94534f));
  points.push_back(Vector3(1.36265f, 6.87979f, 7.77581f));
  points.push_back(Vector3(3.4788f, -1.00009f, 1.13865f));
  points.push_back(Vector3(8.83969f, -7.29179f, 2.20801f));
  points.push_back(Vector3(-5.67431f, -5.68407f, 0.735191f));
  points.push_back(Vector3(6.04907f, -4.82406f, 6.25355f));
  points.push_back(Vector3(6.65944f, 7.48711f, -0.0106817f));
  points.push_back(Vector3(4.15632f, -3.26151f, 3.42265f));
  points.push_back(Vector3(1.04404f, -7.74285f, 5.08408f));
  TestSphereRitter(points, file);
}

void SphereRitterTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.61608f, 2.12744f, -3.51726f));
  points.push_back(Vector3(9.84924f, -5.10666f, -8.9996f));
  points.push_back(Vector3(-1.05014f, 6.31886f, 0.473952f));
  points.push_back(Vector3(9.48546f, -2.76711f, -7.98273f));
  points.push_back(Vector3(-3.54045f, -1.8955f, -5.81591f));
  points.push_back(Vector3(-4.88083f, -1.85949f, 1.54454f));
  points.push_back(Vector3(-5.92456f, -2.18421f, 3.91522f));
  points.push_back(Vector3(-0.299386f, 1.9602f, 0.740074f));
  points.push_back(Vector3(-5.49058f, -7.48894f, 1.66051f));
  points.push_back(Vector3(-8.50398f, -0.105289f, 1.71361f));
  points.push_back(Vector3(3.01859f, 5.41307f, 9.52513f));
  points.push_back(Vector3(2.62795f, -0.974456f, -3.35063f));
  points.push_back(Vector3(-8.48262f, 9.11435f, 3.46355f));
  points.push_back(Vector3(7.77886f, -3.59355f, 4.81491f));
  points.push_back(Vector3(9.67406f, -4.02997f, -8.90988f));
  points.push_back(Vector3(3.80902f, 1.09104f, 3.90179f));
  points.push_back(Vector3(-1.39988f, -6.18824f, -5.84887f));
  points.push_back(Vector3(-6.47877f, 9.31272f, 5.03098f));
  points.push_back(Vector3(4.21674f, -9.21384f, -0.76754f));
  points.push_back(Vector3(-2.18177f, -4.87411f, -0.496536f));
  points.push_back(Vector3(0.975066f, -7.7691f, 3.00577f));
  points.push_back(Vector3(-7.60552f, 3.06253f, -7.36625f));
  points.push_back(Vector3(-5.03464f, -2.48329f, 3.16874f));
  points.push_back(Vector3(-9.53368f, -6.70156f, 8.15668f));
  points.push_back(Vector3(-9.06369f, -1.70141f, 5.81591f));
  points.push_back(Vector3(-5.67309f, 5.98437f, -1.70446f));
  points.push_back(Vector3(-7.06107f, 3.99091f, 6.74001f));
  points.push_back(Vector3(-1.76672f, -7.82281f, -7.86615f));
  points.push_back(Vector3(-4.2851f, -1.8363f, -9.3939f));
  points.push_back(Vector3(5.81835f, -3.14859f, 0.88168f));
  points.push_back(Vector3(5.98682f, 3.33415f, -9.43785f));
  points.push_back(Vector3(9.29685f, 7.13675f, -4.42183f));
  points.push_back(Vector3(2.0951f, 3.84991f, -5.80981f));
  points.push_back(Vector3(7.7868f, 8.83663f, 3.09122f));
  points.push_back(Vector3(9.3585f, 9.74242f, 2.19275f));
  points.push_back(Vector3(0.547197f, -1.17283f, -7.41813f));
  points.push_back(Vector3(9.66124f, 1.00375f, -3.58562f));
  points.push_back(Vector3(0.333568f, -7.27897f, 9.32615f));
  points.push_back(Vector3(9.60448f, -5.6975f, 6.17603f));
  TestSphereRitter(points, file);
}

void SphereRitterTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.11335f, 8.59859f, -2.33497f));
  points.push_back(Vector3(3.90851f, 1.713f, -1.89856f));
  points.push_back(Vector3(-8.68465f, -0.827967f, -0.808435f));
  points.push_back(Vector3(0.113224f, 9.16623f, -7.71233f));
  points.push_back(Vector3(4.88266f, -7.1923f, 7.13004f));
  points.push_back(Vector3(-5.80309f, -0.082705f, 0.926237f));
  points.push_back(Vector3(-3.41838f, 3.98846f, -2.61147f));
  points.push_back(Vector3(4.10688f, -8.14081f, 8.89401f));
  points.push_back(Vector3(-5.12803f, 1.8186f, 7.96258f));
  points.push_back(Vector3(-6.31031f, -1.29307f, 3.02408f));
  points.push_back(Vector3(4.06842f, -7.35038f, 0.657064f));
  points.push_back(Vector3(-1.31016f, -1.50182f, 7.14896f));
  points.push_back(Vector3(-2.41676f, 5.27573f, -9.25108f));
  points.push_back(Vector3(2.06458f, -5.25498f, -2.67495f));
  points.push_back(Vector3(-3.77605f, 4.08979f, 3.36161f));
  points.push_back(Vector3(-2.71706f, 9.27976f, -4.91562f));
  points.push_back(Vector3(-6.94266f, 0.661336f, 0.22187f));
  points.push_back(Vector3(3.59294f, 9.81262f, 4.68856f));
  points.push_back(Vector3(1.87231f, 7.64824f, 0.0454724f));
  points.push_back(Vector3(-1.44932f, -8.6755f, -8.01019f));
  points.push_back(Vector3(-8.5406f, 3.37077f, 2.52968f));
  points.push_back(Vector3(-7.1746f, -8.73592f, 0.0277722f));
  points.push_back(Vector3(-4.68184f, -6.74978f, -7.58965f));
  points.push_back(Vector3(-7.03787f, -3.50261f, 2.45949f));
  points.push_back(Vector3(8.45515f, 8.07611f, -1.35472f));
  points.push_back(Vector3(1.87292f, 7.68365f, -7.13797f));
  points.push_back(Vector3(-7.78253f, -8.69747f, 7.11966f));
  points.push_back(Vector3(-3.11014f, -3.39091f, -1.72277f));
  points.push_back(Vector3(-2.96548f, 0.755333f, 5.77929f));
  points.push_back(Vector3(3.66375f, -3.40007f, -8.35444f));
  points.push_back(Vector3(-5.92456f, 6.69485f, -8.71273f));
  points.push_back(Vector3(-6.37745f, -2.125f, 2.54372f));
  points.push_back(Vector3(-7.34123f, -9.23826f, -1.3242f));
  points.push_back(Vector3(-4.44319f, -8.83969f, -8.73653f));
  points.push_back(Vector3(3.25297f, 1.27903f, 2.55287f));
  points.push_back(Vector3(-9.33042f, 6.51235f, 0.389721f));
  points.push_back(Vector3(-1.40477f, 2.30567f, -7.33268f));
  points.push_back(Vector3(-2.3191f, 2.61513f, -6.92679f));
  points.push_back(Vector3(0.357372f, 9.19919f, 0.58443f));
  points.push_back(Vector3(-7.27714f, 7.33634f, -9.32493f));
  points.push_back(Vector3(8.33064f, -5.27879f, 0.360423f));
  points.push_back(Vector3(-9.63317f, 5.50096f, -3.14798f));
  points.push_back(Vector3(5.8916f, -0.554521f, -3.08451f));
  points.push_back(Vector3(-0.332957f, 7.24906f, 3.03934f));
  points.push_back(Vector3(-8.07672f, -9.09299f, 3.8316f));
  points.push_back(Vector3(5.07248f, 2.3954f, -5.9624f));
  points.push_back(Vector3(-1.96753f, -9.98535f, 9.96948f));
  points.push_back(Vector3(8.99594f, 5.47777f, -5.32701f));
  points.push_back(Vector3(3.18277f, 3.29936f, -5.79455f));
  points.push_back(Vector3(-6.85965f, -4.8851f, 1.63121f));
  points.push_back(Vector3(8.4228f, 8.04376f, 2.15552f));
  points.push_back(Vector3(-7.97967f, 2.44972f, -8.80367f));
  points.push_back(Vector3(-5.46617f, -3.24503f, 0.488602f));
  points.push_back(Vector3(6.798f, -0.765099f, 6.14185f));
  points.push_back(Vector3(7.83868f, 0.291451f, 5.91662f));
  points.push_back(Vector3(3.24686f, 9.17722f, 6.01184f));
  points.push_back(Vector3(-3.53618f, -2.26417f, -0.459914f));
  points.push_back(Vector3(-3.24931f, 8.80184f, -3.05887f));
  points.push_back(Vector3(3.62163f, 8.35872f, -4.41511f));
  points.push_back(Vector3(-7.72149f, 9.01853f, 2.50832f));
  points.push_back(Vector3(8.4051f, 9.14548f, 9.04416f));
  points.push_back(Vector3(0.752891f, 8.11945f, -1.58727f));
  points.push_back(Vector3(0.739464f, -7.00491f, -8.05353f));
  points.push_back(Vector3(-2.34779f, -6.29566f, -6.50746f));
  points.push_back(Vector3(-2.05786f, 0.210272f, -4.62813f));
  points.push_back(Vector3(7.53349f, -2.52113f, 0.257882f));
  points.push_back(Vector3(8.35627f, 1.89489f, -3.63506f));
  points.push_back(Vector3(-8.05597f, 5.49242f, 2.22266f));
  points.push_back(Vector3(-4.785f, -9.22361f, -1.80151f));
  points.push_back(Vector3(-9.55931f, 9.71007f, -4.65072f));
  points.push_back(Vector3(3.78521f, 5.33494f, 1.35716f));
  points.push_back(Vector3(-2.44484f, 7.32963f, -0.58504f));
  points.push_back(Vector3(-0.610065f, -0.610675f, 3.74981f));
  points.push_back(Vector3(-4.79782f, 4.42305f, 4.4908f));
  points.push_back(Vector3(-4.30158f, -5.95447f, -5.51256f));
  points.push_back(Vector3(2.52358f, -7.09952f, 0.608845f));
  TestSphereRitter(points, file);
}

void SphereRitterTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.91659f, 4.79476f, 3.58501f));
  points.push_back(Vector3(5.01511f, 4.61959f, -7.76299f));
  points.push_back(Vector3(5.74084f, -0.801111f, -2.19214f));
  points.push_back(Vector3(-7.63665f, 5.54674f, 1.4536f));
  points.push_back(Vector3(-0.654011f, 2.88003f, 8.48689f));
  points.push_back(Vector3(9.51781f, 3.45073f, 8.84335f));
  points.push_back(Vector3(6.27674f, -2.41432f, 9.49828f));
  points.push_back(Vector3(-3.76568f, -1.36265f, 3.50139f));
  points.push_back(Vector3(2.9545f, 1.6715f, 6.66311f));
  points.push_back(Vector3(1.13804f, 0.721152f, 8.8757f));
  points.push_back(Vector3(5.9209f, -6.39088f, -5.98254f));
  points.push_back(Vector3(-7.48161f, -2.59926f, -7.08365f));
  points.push_back(Vector3(-8.07245f, 0.0857568f, 4.44136f));
  points.push_back(Vector3(1.67028f, -1.26377f, -7.63726f));
  points.push_back(Vector3(-6.24256f, 2.50832f, 8.18232f));
  points.push_back(Vector3(-6.8157f, 8.69625f, -9.16868f));
  points.push_back(Vector3(7.74468f, 9.0698f, -7.16056f));
  points.push_back(Vector3(-3.51421f, -3.90362f, 9.35423f));
  points.push_back(Vector3(-7.90948f, -9.98474f, 6.85232f));
  points.push_back(Vector3(7.47002f, -6.64357f, -0.890836f));
  points.push_back(Vector3(1.71789f, -6.03198f, 7.95709f));
  points.push_back(Vector3(-9.129f, 6.19129f, 1.93823f));
  points.push_back(Vector3(-2.11158f, 5.53087f, -9.08078f));
  points.push_back(Vector3(-4.57869f, -6.93655f, -2.772f));
  points.push_back(Vector3(-8.14997f, 1.47069f, -2.35023f));
  points.push_back(Vector3(7.49504f, -7.70745f, -3.54411f));
  points.push_back(Vector3(-6.36219f, 7.05008f, 0.0198376f));
  points.push_back(Vector3(7.2631f, -0.242622f, -8.45454f));
  points.push_back(Vector3(2.53639f, -2.00903f, -6.12354f));
  points.push_back(Vector3(6.88467f, -6.36952f, 2.78909f));
  points.push_back(Vector3(-3.00699f, 9.72655f, -5.65172f));
  points.push_back(Vector3(-4.55123f, 6.96341f, -1.17954f));
  points.push_back(Vector3(1.4951f, -3.62224f, 0.0717187f));
  points.push_back(Vector3(2.78359f, 6.81509f, 7.58538f));
  points.push_back(Vector3(8.58516f, -0.0595111f, -0.0534075f));
  points.push_back(Vector3(-3.70647f, -7.59575f, -4.12824f));
  points.push_back(Vector3(-1.69713f, -2.1836f, -7.05435f));
  points.push_back(Vector3(-5.72924f, -3.01798f, -1.68798f));
  points.push_back(Vector3(7.36869f, 6.75466f, 1.84912f));
  points.push_back(Vector3(8.04804f, -8.71578f, -2.76223f));
  points.push_back(Vector3(-1.09897f, 5.79882f, 8.06757f));
  points.push_back(Vector3(6.50868f, 3.05094f, 5.00107f));
  points.push_back(Vector3(-7.11295f, -7.3809f, -0.55269f));
  points.push_back(Vector3(2.28858f, 9.60082f, -7.27348f));
  points.push_back(Vector3(0.458694f, -0.957976f, 8.60836f));
  points.push_back(Vector3(1.99927f, -8.55098f, -3.43669f));
  points.push_back(Vector3(3.82733f, -4.77889f, 8.46431f));
  points.push_back(Vector3(-4.48775f, -3.12113f, 6.71926f));
  points.push_back(Vector3(0.959808f, 0.228584f, 6.63381f));
  points.push_back(Vector3(-5.77319f, 2.69204f, -1.44688f));
  points.push_back(Vector3(-7.77093f, 2.33131f, 3.89752f));
  points.push_back(Vector3(-2.2074f, 7.69646f, -8.89767f));
  points.push_back(Vector3(7.88995f, 7.43461f, -3.69976f));
  points.push_back(Vector3(5.55406f, 5.18906f, 9.24619f));
  points.push_back(Vector3(2.7488f, -8.46248f, 8.32148f));
  points.push_back(Vector3(5.38438f, -8.64009f, -6.27857f));
  points.push_back(Vector3(0.40498f, 5.13657f, 8.34834f));
  points.push_back(Vector3(1.25217f, 0.652791f, 0.917081f));
  points.push_back(Vector3(-3.67901f, -3.25846f, -0.0936919f));
  points.push_back(Vector3(-5.84948f, -0.860317f, -5.79882f));
  points.push_back(Vector3(9.3469f, 1.66051f, 5.25193f));
  points.push_back(Vector3(1.54393f, -4.51888f, 3.1901f));
  points.push_back(Vector3(7.62322f, -6.7217f, 4.55306f));
  TestSphereRitter(points, file);
}

void SphereRitterTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.16135f, -2.12317f, 2.46376f));
  points.push_back(Vector3(-1.49632f, -9.729f, -2.55532f));
  points.push_back(Vector3(-7.92108f, -1.04099f, -1.77099f));
  points.push_back(Vector3(-6.08142f, -3.35002f, -1.85278f));
  points.push_back(Vector3(-2.69143f, 0.693686f, 2.44911f));
  points.push_back(Vector3(7.89117f, -9.86328f, 5.60167f));
  points.push_back(Vector3(8.83908f, -8.4875f, -7.50175f));
  points.push_back(Vector3(-5.97034f, -8.57479f, -3.1077f));
  points.push_back(Vector3(6.66189f, -8.54732f, 6.96158f));
  points.push_back(Vector3(4.2552f, -9.48302f, -1.49754f));
  points.push_back(Vector3(0.630207f, 6.80593f, 8.17011f));
  points.push_back(Vector3(1.70202f, 6.34632f, -7.65557f));
  points.push_back(Vector3(-2.40455f, -7.00674f, 0.679647f));
  points.push_back(Vector3(3.98175f, 0.99704f, 1.17038f));
  points.push_back(Vector3(1.04465f, 8.08039f, -6.52516f));
  points.push_back(Vector3(8.36116f, 8.76583f, 7.54875f));
  points.push_back(Vector3(-4.34919f, -5.33738f, 7.65801f));
  points.push_back(Vector3(8.60958f, 5.8562f, 4.0788f));
  points.push_back(Vector3(-1.26682f, -0.825526f, -2.48451f));
  points.push_back(Vector3(-5.14328f, 1.66906f, 6.94021f));
  points.push_back(Vector3(-2.64504f, 7.37663f, 3.65764f));
  points.push_back(Vector3(-4.66353f, -9.94873f, -7.30583f));
  points.push_back(Vector3(6.26575f, 5.45763f, 4.9559f));
  points.push_back(Vector3(-5.11826f, -9.07651f, 2.0835f));
  points.push_back(Vector3(-9.49583f, -9.72655f, -6.32618f));
  points.push_back(Vector3(-8.05963f, -8.03827f, -3.24137f));
  points.push_back(Vector3(9.80346f, -7.69768f, -1.89856f));
  points.push_back(Vector3(-3.76446f, -1.0184f, -1.92358f));
  points.push_back(Vector3(-7.54631f, 3.35063f, -8.46675f));
  points.push_back(Vector3(5.48509f, -5.24094f, 3.65947f));
  points.push_back(Vector3(6.46962f, 0.477614f, 8.23542f));
  points.push_back(Vector3(-5.68163f, 1.89612f, 9.99268f));
  points.push_back(Vector3(-1.71361f, -9.55077f, 8.43806f));
  points.push_back(Vector3(4.19477f, -1.70751f, -9.71679f));
  points.push_back(Vector3(6.292f, 7.82037f, 8.08466f));
  points.push_back(Vector3(2.22877f, -3.46416f, 8.73592f));
  points.push_back(Vector3(0.744346f, 6.96768f, 7.32414f));
  points.push_back(Vector3(1.8656f, 3.35246f, -2.27332f));
  points.push_back(Vector3(5.58763f, -3.42082f, 7.29728f));
  points.push_back(Vector3(-8.16828f, -5.12864f, 5.69262f));
  points.push_back(Vector3(-6.21021f, -3.32682f, 9.92309f));
  points.push_back(Vector3(4.18928f, -9.8059f, 3.22001f));
  points.push_back(Vector3(8.67061f, -4.20331f, 2.80068f));
  points.push_back(Vector3(-9.81445f, -7.79656f, -4.46516f));
  points.push_back(Vector3(1.67455f, -8.00348f, -6.60939f));
  points.push_back(Vector3(6.37196f, 3.19498f, -6.33839f));
  points.push_back(Vector3(4.75204f, -1.89917f, 5.9209f));
  points.push_back(Vector3(5.69079f, 6.21509f, 6.57399f));
  points.push_back(Vector3(5.39293f, 2.16956f, 3.37993f));
  points.push_back(Vector3(1.40721f, 3.51543f, -4.86801f));
  points.push_back(Vector3(-3.08512f, 0.28962f, -1.87353f));
  points.push_back(Vector3(8.33857f, -6.2859f, -1.9779f));
  points.push_back(Vector3(8.39351f, 3.59661f, 0.431226f));
  points.push_back(Vector3(8.39961f, 1.64647f, 2.7488f));
  points.push_back(Vector3(-9.97131f, 7.07938f, -2.70669f));
  points.push_back(Vector3(-1.55675f, 1.95227f, -6.45619f));
  points.push_back(Vector3(-6.58071f, -3.17606f, 6.57033f));
  points.push_back(Vector3(0.293894f, -8.28669f, 5.25681f));
  points.push_back(Vector3(-5.51073f, 7.39006f, 6.33717f));
  points.push_back(Vector3(8.22077f, 1.38523f, -9.52513f));
  points.push_back(Vector3(-7.63604f, 4.48775f, 8.95871f));
  points.push_back(Vector3(0.395215f, -0.58443f, 8.71761f));
  points.push_back(Vector3(-7.34855f, 2.54616f, 6.11805f));
  points.push_back(Vector3(-9.40916f, -3.24564f, -5.92456f));
  points.push_back(Vector3(0.999482f, -5.39171f, 7.5042f));
  points.push_back(Vector3(7.0568f, 5.46678f, -6.32801f));
  points.push_back(Vector3(4.83261f, -4.232f, 4.46577f));
  points.push_back(Vector3(-0.408033f, 3.36161f, -3.80169f));
  points.push_back(Vector3(0.472732f, 3.62102f, 3.28532f));
  points.push_back(Vector3(-4.09955f, -3.57524f, -9.83825f));
  points.push_back(Vector3(1.75878f, 2.32398f, 3.22306f));
  points.push_back(Vector3(-3.24259f, 5.42039f, 7.75262f));
  points.push_back(Vector3(-2.57302f, 3.49895f, -5.32884f));
  points.push_back(Vector3(-1.04099f, 4.7795f, 4.96994f));
  points.push_back(Vector3(0.338451f, 9.7235f, -4.83749f));
  points.push_back(Vector3(7.85882f, 3.34513f, 4.31868f));
  points.push_back(Vector3(4.63729f, 7.35527f, -1.98706f));
  points.push_back(Vector3(-9.90051f, -9.83398f, 5.46556f));
  points.push_back(Vector3(9.62401f, 7.32414f, -7.32902f));
  points.push_back(Vector3(4.36201f, 6.86514f, -5.68285f));
  points.push_back(Vector3(-5.9801f, -2.09876f, -0.221869f));
  points.push_back(Vector3(-3.88165f, 3.59661f, -5.57176f));
  points.push_back(Vector3(3.84564f, -5.27146f, 0.0570691f));
  points.push_back(Vector3(-6.00391f, 0.832239f, 9.08139f));
  points.push_back(Vector3(3.91461f, -6.87613f, -2.9899f));
  points.push_back(Vector3(2.86843f, 6.56301f, 0.220039f));
  points.push_back(Vector3(9.39329f, 8.34285f, -8.62911f));
  points.push_back(Vector3(-5.2092f, 0.599688f, 4.21918f));
  points.push_back(Vector3(-7.52007f, -6.42872f, -7.02139f));
  points.push_back(Vector3(-4.19599f, 6.90847f, -3.31645f));
  TestSphereRitter(points, file);
}

void SphereRitterTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.00632f, 9.58129f, -7.06961f));
  points.push_back(Vector3(4.09589f, 1.32176f, 4.38643f));
  points.push_back(Vector3(0.25483f, -0.401319f, 4.30341f));
  points.push_back(Vector3(1.22532f, 9.60936f, 0.243843f));
  points.push_back(Vector3(-8.71639f, 8.42219f, -1.03977f));
  points.push_back(Vector3(-6.68752f, 6.87246f, 2.77932f));
  points.push_back(Vector3(-4.00555f, -8.53877f, -4.03851f));
  points.push_back(Vector3(-8.96237f, 6.24317f, -9.16074f));
  points.push_back(Vector3(1.8717f, -2.06702f, 1.4066f));
  points.push_back(Vector3(8.62056f, 6.2566f, 8.98312f));
  points.push_back(Vector3(3.49773f, -6.92862f, -8.61019f));
  points.push_back(Vector3(6.12232f, -9.5703f, 9.29258f));
  TestSphereRitter(points, file);
}

void SphereRitterTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.49178f, 2.14026f, 6.01489f));
  points.push_back(Vector3(-6.63747f, 5.56505f, 2.73171f));
  points.push_back(Vector3(-9.62584f, -1.36143f, 4.39558f));
  points.push_back(Vector3(-6.91519f, 6.31825f, 7.18009f));
  points.push_back(Vector3(-2.49184f, -6.01672f, -6.42445f));
  points.push_back(Vector3(3.55388f, 9.92065f, 4.21247f));
  points.push_back(Vector3(4.279f, -4.09223f, -6.43117f));
  points.push_back(Vector3(8.83236f, -9.36216f, -4.56709f));
  points.push_back(Vector3(1.41209f, -0.632648f, -3.12174f));
  points.push_back(Vector3(3.05765f, -0.144963f, -0.966521f));
  points.push_back(Vector3(-3.94147f, 1.48961f, 5.12009f));
  points.push_back(Vector3(-3.87249f, 2.39784f, 0.736412f));
  points.push_back(Vector3(-0.81515f, -1.29002f, -0.264596f));
  points.push_back(Vector3(-4.77157f, -9.08078f, -6.30482f));
  points.push_back(Vector3(-3.25175f, 0.664388f, -1.95898f));
  points.push_back(Vector3(3.48979f, 9.55077f, 8.71212f));
  points.push_back(Vector3(-4.4847f, 3.28532f, 2.29957f));
  points.push_back(Vector3(7.6867f, 6.02588f, -7.8515f));
  points.push_back(Vector3(6.44215f, 7.32536f, -1.85705f));
  points.push_back(Vector3(1.0184f, 9.20591f, -4.20698f));
  points.push_back(Vector3(7.80999f, -9.25108f, -8.01385f));
  points.push_back(Vector3(7.04093f, 8.77316f, 3.31095f));
  points.push_back(Vector3(9.2346f, 6.6332f, 7.58843f));
  points.push_back(Vector3(-0.71688f, 8.47713f, -5.81347f));
  points.push_back(Vector3(8.81527f, 4.76302f, 2.09693f));
  points.push_back(Vector3(-9.47203f, 6.03503f, 0.809656f));
  points.push_back(Vector3(-8.50276f, 7.70989f, 4.31806f));
  points.push_back(Vector3(-1.44566f, -5.72802f, 9.99207f));
  points.push_back(Vector3(9.7528f, -6.90054f, -4.99374f));
  points.push_back(Vector3(3.94147f, -8.32331f, -0.669271f));
  points.push_back(Vector3(-9.66063f, 5.74206f, -6.77358f));
  points.push_back(Vector3(2.77139f, 8.90927f, 3.86578f));
  points.push_back(Vector3(-4.16791f, 8.59615f, -3.19742f));
  points.push_back(Vector3(5.21714f, -6.71987f, 1.9364f));
  points.push_back(Vector3(9.14182f, 2.88125f, 0.976897f));
  points.push_back(Vector3(2.6017f, -1.57811f, -6.40553f));
  points.push_back(Vector3(-3.97443f, 7.33573f, 0.242622f));
  points.push_back(Vector3(3.07047f, -6.24439f, 2.73904f));
  points.push_back(Vector3(7.03787f, 7.30216f, -3.68267f));
  points.push_back(Vector3(1.98279f, -9.56175f, 7.96319f));
  points.push_back(Vector3(-1.02817f, 8.43013f, 2.902f));
  points.push_back(Vector3(-9.96338f, 9.7058f, -0.695517f));
  points.push_back(Vector3(-0.785241f, -0.199286f, 7.7868f));
  points.push_back(Vector3(-7.48405f, 8.50642f, -8.72127f));
  points.push_back(Vector3(-3.10953f, 9.78027f, -2.6017f));
  points.push_back(Vector3(-0.694296f, -1.57445f, -0.851161f));
  points.push_back(Vector3(-5.0914f, 7.30888f, 0.674154f));
  points.push_back(Vector3(-9.9176f, -0.263985f, 7.94549f));
  points.push_back(Vector3(9.38047f, 4.18378f, 0.561235f));
  points.push_back(Vector3(4.71969f, -4.58846f, -5.2794f));
  points.push_back(Vector3(9.96948f, -7.30522f, 5.11094f));
  points.push_back(Vector3(-1.93274f, 2.64016f, 6.43788f));
  points.push_back(Vector3(8.97031f, -9.8236f, -1.58483f));
  points.push_back(Vector3(-6.69179f, -5.74999f, -8.77255f));
  points.push_back(Vector3(0.873134f, 0.234077f, -2.82571f));
  points.push_back(Vector3(-8.45088f, -7.9577f, -7.92169f));
  points.push_back(Vector3(0.307322f, 1.96936f, -6.13391f));
  points.push_back(Vector3(9.74181f, -9.2346f, 1.69408f));
  points.push_back(Vector3(-2.94717f, 6.10767f, 4.86862f));
  points.push_back(Vector3(6.85904f, 7.93451f, 7.97174f));
  points.push_back(Vector3(-5.14634f, -7.13797f, -6.04419f));
  points.push_back(Vector3(2.06885f, 3.30241f, -6.38234f));
  points.push_back(Vector3(-1.57384f, -6.67287f, 0.867641f));
  points.push_back(Vector3(-9.15708f, 5.14817f, -6.52516f));
  points.push_back(Vector3(-7.03726f, -3.48613f, 3.55693f));
  points.push_back(Vector3(8.12128f, -8.98984f, 1.60558f));
  points.push_back(Vector3(-5.00778f, 0.442213f, -6.10828f));
  points.push_back(Vector3(-5.74938f, -9.27f, -0.409864f));
  points.push_back(Vector3(-4.78683f, 0.435499f, 2.39357f));
  points.push_back(Vector3(-8.04376f, 2.02124f, -9.64232f));
  points.push_back(Vector3(-4.62752f, 3.41105f, 0.586261f));
  points.push_back(Vector3(3.57891f, 5.87939f, -9.36094f));
  points.push_back(Vector3(-5.12131f, -3.91705f, -7.74285f));
  points.push_back(Vector3(9.71557f, -6.62709f, 2.38258f));
  points.push_back(Vector3(1.1771f, 0.0131226f, 9.78881f));
  points.push_back(Vector3(-4.21857f, -0.664998f, -8.39351f));
  points.push_back(Vector3(-6.59536f, -0.0674456f, -8.70907f));
  points.push_back(Vector3(-5.73473f, -5.42528f, -5.19394f));
  points.push_back(Vector3(-5.94104f, -3.82427f, 1.05136f));
  points.push_back(Vector3(-9.80224f, 9.08261f, -9.48241f));
  TestSphereRitter(points, file);
}

void SphereRitterTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.84313f, -5.74816f, -6.50807f));
  points.push_back(Vector3(-4.47798f, -2.7958f, -2.22144f));
  points.push_back(Vector3(2.38502f, 9.99939f, -8.40388f));
  points.push_back(Vector3(-2.73232f, 7.52434f, 8.95321f));
  points.push_back(Vector3(-3.96527f, 6.8102f, -9.94873f));
  points.push_back(Vector3(8.73409f, -7.8576f, -0.483719f));
  points.push_back(Vector3(6.69423f, 0.967742f, 9.62401f));
  points.push_back(Vector3(2.53578f, -2.77261f, -7.33634f));
  points.push_back(Vector3(-1.56651f, 3.04056f, -0.989715f));
  points.push_back(Vector3(5.11704f, 4.58418f, 3.31889f));
  points.push_back(Vector3(-5.78784f, -5.28367f, 0.202948f));
  points.push_back(Vector3(-6.9097f, 5.36546f, 8.80245f));
  points.push_back(Vector3(8.47896f, -2.82571f, 4.85519f));
  points.push_back(Vector3(-0.323191f, -5.88855f, 6.63686f));
  points.push_back(Vector3(-6.01184f, 9.91516f, -0.101627f));
  points.push_back(Vector3(-8.46736f, -2.74697f, -7.9223f));
  points.push_back(Vector3(0.0747705f, 3.38603f, 7.39799f));
  points.push_back(Vector3(-4.02264f, -8.95993f, 5.50768f));
  points.push_back(Vector3(6.95853f, 6.59291f, -9.90967f));
  points.push_back(Vector3(-0.70101f, 1.85461f, 5.78539f));
  points.push_back(Vector3(2.78481f, 4.65804f, -7.32048f));
  points.push_back(Vector3(0.923185f, 9.56786f, 2.60353f));
  points.push_back(Vector3(-0.68453f, -6.45375f, -7.72454f));
  points.push_back(Vector3(-1.74047f, 4.48592f, -8.80734f));
  points.push_back(Vector3(-8.84762f, -7.68914f, -8.11396f));
  points.push_back(Vector3(-4.47981f, -7.00491f, -0.936003f));
  points.push_back(Vector3(4.50728f, 9.8352f, 1.0593f));
  points.push_back(Vector3(-0.768151f, -1.22471f, -0.837123f));
  points.push_back(Vector3(7.34611f, 5.57054f, 0.142522f));
  TestSphereRitter(points, file);
}

void SphereRitterTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.43013f, 5.50951f, 4.02081f));
  points.push_back(Vector3(8.66268f, 0.726036f, -2.97159f));
  points.push_back(Vector3(9.26511f, 3.25236f, -3.19071f));
  points.push_back(Vector3(-3.24259f, 3.5197f, 8.01385f));
  points.push_back(Vector3(-3.60454f, 7.05496f, -6.52882f));
  points.push_back(Vector3(-8.39412f, -2.65908f, -7.16605f));
  points.push_back(Vector3(-1.08188f, 1.01596f, 9.56114f));
  points.push_back(Vector3(1.74535f, -5.2678f, 8.15607f));
  points.push_back(Vector3(-2.28675f, 8.60958f, 0.055238f));
  points.push_back(Vector3(-5.20798f, 8.93124f, 4.73739f));
  points.push_back(Vector3(-7.16117f, 1.41575f, 8.70785f));
  points.push_back(Vector3(2.75613f, -2.13294f, -4.32234f));
  points.push_back(Vector3(-1.9187f, -2.42958f, 6.37501f));
  points.push_back(Vector3(6.02832f, -4.77218f, -5.92334f));
  points.push_back(Vector3(-4.60006f, -3.82977f, -5.40635f));
  points.push_back(Vector3(2.26661f, 4.9797f, 5.9624f));
  points.push_back(Vector3(6.69362f, -9.66918f, 1.32908f));
  points.push_back(Vector3(-5.61327f, -8.30439f, -3.59539f));
  points.push_back(Vector3(4.11725f, -9.78027f, 9.92737f));
  points.push_back(Vector3(9.56359f, 5.97278f, 6.01489f));
  points.push_back(Vector3(2.40516f, 0.703452f, 2.33863f));
  points.push_back(Vector3(1.03732f, 4.54817f, -6.06738f));
  points.push_back(Vector3(-0.193793f, 8.76583f, 6.16504f));
  points.push_back(Vector3(-0.906094f, 1.04831f, -5.41124f));
  points.push_back(Vector3(0.861537f, -3.95978f, -5.51927f));
  points.push_back(Vector3(-0.673543f, -2.01148f, -8.69747f));
  points.push_back(Vector3(6.77297f, -5.80981f, 0.248725f));
  points.push_back(Vector3(-2.24342f, 6.33778f, 2.81777f));
  points.push_back(Vector3(0.0985754f, -2.07251f, -4.07819f));
  points.push_back(Vector3(-3.99884f, -1.94128f, 1.55065f));
  points.push_back(Vector3(2.42286f, -5.26109f, -3.9317f));
  points.push_back(Vector3(-1.98462f, 2.05664f, 7.22892f));
  points.push_back(Vector3(8.99899f, -5.68224f, 9.00327f));
  points.push_back(Vector3(-4.56954f, 7.46086f, -0.857875f));
  points.push_back(Vector3(5.65416f, -3.16935f, -4.97665f));
  points.push_back(Vector3(3.70769f, 2.584f, -2.98135f));
  points.push_back(Vector3(-7.02933f, -0.0112921f, 6.37562f));
  points.push_back(Vector3(-1.28269f, -4.27229f, 6.95181f));
  points.push_back(Vector3(3.42143f, 2.87515f, -9.93408f));
  points.push_back(Vector3(5.41917f, -2.17322f, 7.45842f));
  points.push_back(Vector3(1.23569f, -7.80694f, -8.46065f));
  points.push_back(Vector3(-0.435499f, -0.874966f, -5.93799f));
  points.push_back(Vector3(4.83016f, 6.48f, 7.59209f));
  points.push_back(Vector3(-7.68609f, 5.24888f, 1.30345f));
  points.push_back(Vector3(-9.92004f, -1.61473f, 3.18522f));
  points.push_back(Vector3(8.77316f, -5.34776f, -8.73775f));
  points.push_back(Vector3(2.15491f, 6.90786f, -8.44172f));
  points.push_back(Vector3(-2.72622f, 1.9364f, -9.90478f));
  points.push_back(Vector3(6.4745f, 2.88186f, 2.68227f));
  points.push_back(Vector3(9.25169f, 2.52113f, 1.11179f));
  points.push_back(Vector3(8.13837f, -5.32701f, -5.79272f));
  points.push_back(Vector3(2.23853f, -3.91705f, -2.78604f));
  points.push_back(Vector3(6.40431f, 9.35606f, -5.87817f));
  points.push_back(Vector3(-6.59658f, 1.48473f, 9.45067f));
  points.push_back(Vector3(8.95749f, -9.86267f, 7.75872f));
  points.push_back(Vector3(0.616169f, 7.25944f, -5.43809f));
  points.push_back(Vector3(-7.68853f, 6.0155f, -3.73455f));
  points.push_back(Vector3(4.78011f, -8.60714f, -4.76608f));
  points.push_back(Vector3(2.22999f, 7.00125f, 1.45909f));
  points.push_back(Vector3(3.63811f, -5.15244f, -5.23606f));
  points.push_back(Vector3(4.01227f, -9.63927f, -4.2143f));
  points.push_back(Vector3(-4.14594f, 7.3516f, 5.55834f));
  points.push_back(Vector3(-0.564898f, 2.53639f, -1.64037f));
  points.push_back(Vector3(7.53349f, 4.01776f, -9.25901f));
  points.push_back(Vector3(7.45964f, 0.379956f, 1.16855f));
  points.push_back(Vector3(3.81512f, -0.144352f, -7.24601f));
  points.push_back(Vector3(2.12012f, 6.46901f, -3.81024f));
  points.push_back(Vector3(8.40083f, 8.61263f, 4.54634f));
  points.push_back(Vector3(-4.26008f, 9.07041f, 6.65639f));
  points.push_back(Vector3(-3.76446f, 5.27573f, 0.435499f));
  points.push_back(Vector3(-8.7286f, -9.64415f, 6.71194f));
  points.push_back(Vector3(1.64891f, 0.326242f, -2.66579f));
  points.push_back(Vector3(6.66799f, 9.14914f, 5.60656f));
  points.push_back(Vector3(8.5052f, 1.07089f, -4.64217f));
  points.push_back(Vector3(-0.769371f, 9.35545f, -2.07678f));
  TestSphereRitter(points, file);
}

void SphereRitterTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.90439f, 4.15693f, 9.00571f));
  points.push_back(Vector3(-6.86331f, 2.39296f, 6.61794f));
  points.push_back(Vector3(-2.86111f, 8.47896f, -1.7246f));
  points.push_back(Vector3(9.84924f, 5.00534f, 1.61534f));
  points.push_back(Vector3(0.420851f, 3.47453f, 9.75707f));
  points.push_back(Vector3(-6.98843f, 1.66112f, -3.68206f));
  points.push_back(Vector3(-7.01895f, 3.73577f, -8.69747f));
  points.push_back(Vector3(-7.07327f, -6.37684f, 9.18943f));
  points.push_back(Vector3(-3.56792f, 2.17017f, 4.69771f));
  points.push_back(Vector3(1.47435f, -2.68349f, 3.52763f));
  points.push_back(Vector3(-3.08939f, -8.98557f, 5.03464f));
  points.push_back(Vector3(6.70522f, 2.3307f, 8.06696f));
  points.push_back(Vector3(-0.544145f, -9.81262f, 1.21006f));
  points.push_back(Vector3(-5.27207f, -4.30097f, -5.62792f));
  points.push_back(Vector3(5.10971f, 8.09015f, -2.8074f));
  points.push_back(Vector3(-5.11643f, -0.575884f, 5.13779f));
  points.push_back(Vector3(3.45622f, -8.27693f, -1.5421f));
  points.push_back(Vector3(-9.95972f, -9.25779f, -0.144352f));
  points.push_back(Vector3(-3.8432f, -5.55345f, -6.35121f));
  points.push_back(Vector3(-3.13456f, 5.63402f, -8.59004f));
  points.push_back(Vector3(5.833f, -6.68569f, -9.39512f));
  points.push_back(Vector3(-6.42018f, 7.95404f, 1.41331f));
  points.push_back(Vector3(1.86987f, -3.73455f, 0.966522f));
  points.push_back(Vector3(7.64336f, 1.94861f, -3.98419f));
  points.push_back(Vector3(-7.10868f, 4.94369f, -8.98557f));
  points.push_back(Vector3(5.32701f, -2.8135f, -9.90051f));
  points.push_back(Vector3(-2.7604f, -5.88977f, -9.15525f));
  points.push_back(Vector3(1.94739f, 4.67391f, -1.45604f));
  points.push_back(Vector3(-9.65514f, 6.02222f, 1.7954f));
  points.push_back(Vector3(7.9284f, -6.42445f, 1.2833f));
  points.push_back(Vector3(3.84808f, -9.82299f, 5.86596f));
  points.push_back(Vector3(-5.47227f, -7.91192f, 6.5685f));
  points.push_back(Vector3(-1.76305f, -1.86926f, 3.0784f));
  points.push_back(Vector3(-3.62713f, -3.98907f, -3.2786f));
  points.push_back(Vector3(-8.8641f, 1.99499f, 5.8916f));
  points.push_back(Vector3(-1.55675f, -2.9191f, -4.75631f));
  points.push_back(Vector3(-0.353099f, -3.60576f, 6.39088f));
  points.push_back(Vector3(-4.63607f, 2.5248f, -4.09894f));
  points.push_back(Vector3(9.89441f, -0.553911f, 4.51033f));
  points.push_back(Vector3(-6.77114f, -9.86694f, -4.15021f));
  points.push_back(Vector3(2.56508f, 1.3657f, 6.51357f));
  points.push_back(Vector3(1.25584f, -8.02911f, 8.50276f));
  points.push_back(Vector3(2.36183f, -8.02362f, -5.10666f));
  points.push_back(Vector3(4.83078f, 7.03421f, -9.84924f));
  points.push_back(Vector3(4.30219f, 3.28837f, -3.82061f));
  points.push_back(Vector3(-4.5555f, -8.63643f, 9.87609f));
  points.push_back(Vector3(3.29447f, 2.88614f, -5.55834f));
  points.push_back(Vector3(9.16257f, 5.69201f, -2.5425f));
  points.push_back(Vector3(-2.1836f, 0.621662f, -1.98828f));
  points.push_back(Vector3(6.07166f, -1.73864f, -8.59737f));
  points.push_back(Vector3(-3.60881f, -2.44728f, 7.59392f));
  TestSphereRitter(points, file);
}

void SphereRitterTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.77355f, 3.86334f, -4.88815f));
  points.push_back(Vector3(9.5703f, -2.45277f, 1.66234f));
  points.push_back(Vector3(8.87448f, 7.46574f, 4.32173f));
  points.push_back(Vector3(5.93127f, 4.36506f, 9.8938f));
  points.push_back(Vector3(8.97275f, 0.277413f, 4.12763f));
  points.push_back(Vector3(-0.884731f, 5.03708f, -6.87857f));
  points.push_back(Vector3(5.80248f, 3.50871f, -6.5154f));
  points.push_back(Vector3(-6.75588f, -2.02124f, 2.51198f));
  points.push_back(Vector3(-9.61364f, -0.718711f, -5.79028f));
  points.push_back(Vector3(-2.92459f, -1.94372f, -1.20884f));
  points.push_back(Vector3(-1.68065f, 6.24683f, -2.19825f));
  points.push_back(Vector3(2.51015f, 4.12091f, 0.509964f));
  points.push_back(Vector3(-7.60613f, 9.43724f, 2.67373f));
  points.push_back(Vector3(-7.48527f, -3.60515f, 8.3581f));
  points.push_back(Vector3(0.207831f, -4.19111f, 2.45766f));
  points.push_back(Vector3(-8.5577f, -0.165105f, -7.80084f));
  points.push_back(Vector3(1.72765f, 5.53453f, -9.82726f));
  points.push_back(Vector3(-1.61901f, 6.47511f, -3.23527f));
  points.push_back(Vector3(-5.98865f, -7.55852f, -6.9274f));
  points.push_back(Vector3(-5.89282f, 0.13947f, -9.12595f));
  points.push_back(Vector3(-6.54286f, 7.50969f, -8.53572f));
  points.push_back(Vector3(-3.27433f, -6.8157f, 6.2096f));
  points.push_back(Vector3(-7.45903f, 9.50255f, 4.21674f));
  points.push_back(Vector3(-7.116f, 9.64782f, -2.54677f));
  points.push_back(Vector3(-9.44029f, -0.763268f, 6.51418f));
  points.push_back(Vector3(2.87332f, -5.12253f, -1.34739f));
  points.push_back(Vector3(8.85067f, 1.02329f, 4.20698f));
  points.push_back(Vector3(1.55675f, 0.784631f, 3.32072f));
  points.push_back(Vector3(2.89773f, 9.08628f, 6.33778f));
  points.push_back(Vector3(5.40757f, -9.09055f, -9.29136f));
  points.push_back(Vector3(-0.935392f, -9.27305f, -4.42122f));
  points.push_back(Vector3(1.41636f, -7.01224f, -9.76745f));
  points.push_back(Vector3(-5.00717f, 4.10688f, -2.06214f));
  points.push_back(Vector3(8.85617f, 1.08615f, -7.20878f));
  points.push_back(Vector3(-7.56157f, -0.644246f, 0.590533f));
  points.push_back(Vector3(2.24525f, -2.74697f, -5.82263f));
  points.push_back(Vector3(1.96081f, -0.273751f, 2.72195f));
  points.push_back(Vector3(-4.05744f, 6.40614f, 4.39741f));
  points.push_back(Vector3(-6.98721f, -2.8019f, -8.86166f));
  points.push_back(Vector3(-9.94324f, 0.409863f, -9.95361f));
  points.push_back(Vector3(2.98379f, -3.17606f, -7.23319f));
  points.push_back(Vector3(-3.71319f, -9.53856f, 7.3925f));
  points.push_back(Vector3(7.25089f, -1.31443f, -7.52861f));
  points.push_back(Vector3(-6.52699f, -6.50136f, 4.00678f));
  points.push_back(Vector3(-8.30073f, 8.07001f, -0.687582f));
  points.push_back(Vector3(-1.5775f, -9.11435f, 9.18394f));
  points.push_back(Vector3(2.02002f, 5.93799f, 0.0595117f));
  points.push_back(Vector3(-9.39451f, -1.3773f, -9.55992f));
  points.push_back(Vector3(-7.12027f, 5.38865f, -2.14087f));
  points.push_back(Vector3(-7.98883f, 0.204779f, -2.34962f));
  points.push_back(Vector3(7.30888f, -1.87841f, -0.992767f));
  points.push_back(Vector3(8.31843f, -1.34678f, 0.332958f));
  points.push_back(Vector3(4.77096f, -9.5587f, 4.23872f));
  points.push_back(Vector3(-2.87088f, 5.48692f, -0.295724f));
  points.push_back(Vector3(8.03705f, -6.48732f, 5.37828f));
  points.push_back(Vector3(-4.24604f, 4.16181f, -2.02185f));
  TestSphereRitter(points, file);
}

void SphereRitterTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.04965f, 5.25437f, -6.94449f));
  points.push_back(Vector3(0.98056f, 2.82022f, -8.34101f));
  points.push_back(Vector3(-1.53844f, -1.0947f, 2.59011f));
  points.push_back(Vector3(3.78033f, -8.4051f, -0.347606f));
  points.push_back(Vector3(-5.99109f, 6.24195f, 0.970184f));
  points.push_back(Vector3(0.501419f, 6.76748f, -4.62996f));
  points.push_back(Vector3(-8.58821f, -4.42183f, 2.67739f));
  points.push_back(Vector3(3.16385f, 4.62813f, -3.35063f));
  points.push_back(Vector3(5.78661f, 9.43907f, -4.71908f));
  points.push_back(Vector3(7.38945f, -7.92657f, -4.74288f));
  points.push_back(Vector3(3.2017f, 9.25352f, 4.32844f));
  points.push_back(Vector3(0.773034f, -7.64519f, 7.07694f));
  points.push_back(Vector3(7.86065f, 0.612507f, -2.81106f));
  points.push_back(Vector3(-7.47734f, -7.34977f, -0.0753808f));
  points.push_back(Vector3(6.99515f, -6.48915f, -2.4308f));
  points.push_back(Vector3(4.21552f, 8.89706f, 7.74224f));
  points.push_back(Vector3(2.2953f, 6.10034f, 0.0985754f));
  points.push_back(Vector3(8.45576f, 2.55104f, 2.09021f));
  points.push_back(Vector3(-9.9823f, 0.533159f, -5.71337f));
  points.push_back(Vector3(2.55043f, 2.9252f, -4.91806f));
  points.push_back(Vector3(2.4308f, 7.93878f, 1.71239f));
  points.push_back(Vector3(0.620441f, -0.161443f, -5.53636f));
  points.push_back(Vector3(3.42753f, -1.01352f, -7.93939f));
  points.push_back(Vector3(2.28187f, 7.18253f, 4.10077f));
  points.push_back(Vector3(-6.44398f, 9.35545f, 5.07797f));
  TestSphereRitter(points, file);
}

void SphereRitterTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.93857f, -7.36869f, 3.64727f));
  points.push_back(Vector3(3.45683f, -4.63118f, 5.8739f));
  points.push_back(Vector3(4.8442f, -1.1478f, 6.95181f));
  points.push_back(Vector3(-6.35914f, 6.53981f, 1.15818f));
  points.push_back(Vector3(-9.29136f, -8.20978f, -0.508744f));
  points.push_back(Vector3(-6.88833f, 3.39824f, 5.1384f));
  points.push_back(Vector3(-6.01489f, -0.355541f, -7.14469f));
  points.push_back(Vector3(-7.70257f, -2.17017f, -3.59416f));
  points.push_back(Vector3(4.91195f, 7.42485f, 3.60027f));
  points.push_back(Vector3(-9.47752f, -6.37013f, -7.98761f));
  points.push_back(Vector3(5.77319f, -8.30012f, 0.48677f));
  points.push_back(Vector3(-0.15595f, -1.63671f, 1.80883f));
  points.push_back(Vector3(-1.87475f, 8.23237f, -6.30177f));
  points.push_back(Vector3(1.06418f, 4.30464f, -1.32115f));
  points.push_back(Vector3(-7.05435f, -1.05869f, 1.66356f));
  points.push_back(Vector3(0.070498f, 9.02768f, 7.93634f));
  points.push_back(Vector3(8.93918f, -2.46315f, -7.46147f));
  points.push_back(Vector3(8.30195f, -0.754113f, 0.101627f));
  points.push_back(Vector3(1.30283f, 1.89123f, -8.94223f));
  points.push_back(Vector3(-9.75646f, 4.18134f, 3.22428f));
  points.push_back(Vector3(7.50664f, 8.50093f, -0.232856f));
  points.push_back(Vector3(5.90503f, 9.62584f, -3.7669f));
  points.push_back(Vector3(0.588702f, -7.48405f, -0.833461f));
  points.push_back(Vector3(9.68261f, 3.44157f, 6.11805f));
  points.push_back(Vector3(-3.39213f, 8.43684f, -3.51848f));
  points.push_back(Vector3(-8.53084f, 9.98657f, -8.5577f));
  points.push_back(Vector3(3.70159f, 4.10504f, 0.17365f));
  TestSphereRitter(points, file);
}

void SphereRitterTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.59737f, -5.5565f, 8.13349f));
  points.push_back(Vector3(-5.05234f, -8.09809f, -4.29304f));
  points.push_back(Vector3(9.69115f, -6.66799f, -9.73266f));
  points.push_back(Vector3(-8.034f, -8.89767f, 5.75549f));
  points.push_back(Vector3(1.34556f, -1.26804f, -4.31013f));
  points.push_back(Vector3(2.69082f, 2.3954f, 8.18171f));
  points.push_back(Vector3(5.05783f, -5.08347f, 4.29304f));
  points.push_back(Vector3(8.04498f, 4.92904f, 8.52046f));
  points.push_back(Vector3(9.02646f, 1.6306f, 7.57622f));
  points.push_back(Vector3(-9.90844f, -3.40373f, -0.395825f));
  points.push_back(Vector3(-1.01779f, -2.25501f, -5.53026f));
  points.push_back(Vector3(4.15021f, 8.85434f, 3.57952f));
  points.push_back(Vector3(-7.3162f, -3.85846f, -8.38923f));
  points.push_back(Vector3(3.21635f, -2.25257f, 5.94592f));
  points.push_back(Vector3(-3.52641f, -1.97546f, 8.4521f));
  points.push_back(Vector3(-1.51585f, -5.21653f, 5.67125f));
  points.push_back(Vector3(6.72719f, -6.82485f, 2.66884f));
  points.push_back(Vector3(4.61776f, 6.65395f, 9.3878f));
  points.push_back(Vector3(7.19047f, 8.18232f, -5.22996f));
  points.push_back(Vector3(-1.97729f, -5.91174f, -6.19862f));
  points.push_back(Vector3(4.79171f, 9.86022f, 5.2562f));
  points.push_back(Vector3(3.11136f, -0.132145f, 1.27903f));
  points.push_back(Vector3(4.06415f, -1.52867f, 5.50645f));
  points.push_back(Vector3(7.99371f, -3.33964f, -8.43684f));
  points.push_back(Vector3(6.28407f, 7.41813f, 7.37541f));
  points.push_back(Vector3(9.68017f, 9.4702f, 0.290231f));
  points.push_back(Vector3(-3.65093f, -4.6617f, 8.69503f));
  points.push_back(Vector3(-7.82342f, -2.48512f, 4.83871f));
  points.push_back(Vector3(-9.22666f, 5.49059f, 5.05417f));
  points.push_back(Vector3(1.38707f, -1.17038f, 2.63283f));
  points.push_back(Vector3(6.45558f, 8.42341f, -5.17991f));
  points.push_back(Vector3(-9.09787f, -9.69726f, -1.23936f));
  points.push_back(Vector3(-5.38804f, 9.78149f, 6.91214f));
  points.push_back(Vector3(5.17502f, -1.28636f, -8.77133f));
  points.push_back(Vector3(9.94202f, 8.06513f, -8.20002f));
  points.push_back(Vector3(-6.386f, -2.88797f, 5.80432f));
  points.push_back(Vector3(-5.84948f, -0.103458f, -1.85461f));
  points.push_back(Vector3(3.41655f, -1.87353f, 0.698569f));
  points.push_back(Vector3(8.05902f, 5.19394f, -7.18925f));
  points.push_back(Vector3(-7.11844f, -6.06494f, 6.89077f));
  points.push_back(Vector3(9.94018f, -5.74572f, -6.70522f));
  points.push_back(Vector3(-6.89749f, -5.75304f, 5.04135f));
  points.push_back(Vector3(-7.80633f, -4.52132f, -8.65474f));
  points.push_back(Vector3(-3.26823f, 1.17283f, -2.63283f));
  points.push_back(Vector3(-3.72417f, -1.06052f, -2.81716f));
  points.push_back(Vector3(0.539262f, -4.15571f, 4.89059f));
  TestSphereRitter(points, file);
}

void SphereRitterTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.73104f, -5.0969f, -2.39662f));
  points.push_back(Vector3(-9.08689f, -9.89746f, -4.87106f));
  points.push_back(Vector3(2.41188f, -6.96158f, -4.9498f));
  points.push_back(Vector3(3.38298f, -2.8489f, 3.95428f));
  points.push_back(Vector3(-3.99152f, 2.09449f, 8.17377f));
  points.push_back(Vector3(-6.32557f, 5.71215f, -5.2916f));
  points.push_back(Vector3(-9.80712f, -7.09037f, -2.52724f));
  points.push_back(Vector3(-5.02609f, -9.08078f, 1.66845f));
  points.push_back(Vector3(-8.14203f, 9.39818f, -3.80657f));
  points.push_back(Vector3(-3.65764f, 7.94916f, 5.98682f));
  points.push_back(Vector3(-3.7376f, -6.61183f, 4.78072f));
  points.push_back(Vector3(-7.59575f, 3.80779f, -2.26051f));
  points.push_back(Vector3(8.61751f, -3.33171f, 4.35896f));
  points.push_back(Vector3(-5.93127f, -8.33857f, -6.4922f));
  points.push_back(Vector3(9.71191f, 9.02707f, -2.16102f));
  points.push_back(Vector3(9.51537f, -5.01572f, 2.58217f));
  points.push_back(Vector3(1.27781f, -9.7943f, 9.92798f));
  TestSphereRitter(points, file);
}

void SphereRitterTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.5876f, 7.2454f, 2.64992f));
  points.push_back(Vector3(8.38313f, 3.37993f, 7.69097f));
  points.push_back(Vector3(-0.810877f, 4.39924f, 0.657064f));
  points.push_back(Vector3(-6.79617f, -9.34873f, 5.88977f));
  points.push_back(Vector3(8.90866f, 9.22788f, 3.41472f));
  points.push_back(Vector3(-4.39802f, 4.23749f, 4.41084f));
  points.push_back(Vector3(-9.35545f, 6.70888f, -0.869473f));
  points.push_back(Vector3(-4.71114f, -9.24924f, 9.92737f));
  points.push_back(Vector3(-6.4098f, -4.62081f, -7.39555f));
  points.push_back(Vector3(0.371411f, 7.09281f, 1.81494f));
  points.push_back(Vector3(8.1402f, 2.24891f, -3.01614f));
  points.push_back(Vector3(2.63833f, 2.96182f, -0.462356f));
  points.push_back(Vector3(-1.04221f, -2.10669f, 5.58519f));
  points.push_back(Vector3(-9.23582f, -4.78378f, -2.37953f));
  points.push_back(Vector3(5.93371f, -3.99091f, 5.96057f));
  points.push_back(Vector3(9.8999f, 3.72417f, -5.74023f));
  points.push_back(Vector3(9.82055f, 2.05359f, -2.64931f));
  points.push_back(Vector3(-0.753502f, 8.85861f, 8.13898f));
  points.push_back(Vector3(-6.08814f, -8.12555f, 3.76751f));
  points.push_back(Vector3(5.85315f, -9.01425f, -2.61452f));
  points.push_back(Vector3(-5.11277f, -4.01837f, -9.20347f));
  points.push_back(Vector3(5.36912f, -5.33006f, 7.50847f));
  points.push_back(Vector3(-2.61818f, 0.75106f, 7.21244f));
  points.push_back(Vector3(1.51158f, -2.60231f, 3.88653f));
  points.push_back(Vector3(0.484939f, -3.02103f, -4.82101f));
  points.push_back(Vector3(7.87347f, 6.88223f, -8.3404f));
  points.push_back(Vector3(2.6841f, -8.77743f, -2.28431f));
  points.push_back(Vector3(1.08066f, -5.05539f, -5.99597f));
  points.push_back(Vector3(7.28141f, 8.85006f, -8.34712f));
  points.push_back(Vector3(-4.40962f, -7.88324f, -8.62239f));
  points.push_back(Vector3(-4.49934f, 2.13294f, 1.08493f));
  points.push_back(Vector3(-9.86511f, 7.90765f, 1.8833f));
  points.push_back(Vector3(1.89734f, -5.42528f, -7.24967f));
  points.push_back(Vector3(-3.50017f, -0.28901f, 8.84335f));
  points.push_back(Vector3(8.7347f, 6.44398f, -6.57949f));
  points.push_back(Vector3(5.40574f, 1.00925f, -1.38524f));
  points.push_back(Vector3(5.27146f, -1.2888f, -7.31742f));
  points.push_back(Vector3(-5.67309f, 9.41282f, 2.02307f));
  points.push_back(Vector3(6.83035f, 5.77075f, -4.97177f));
  points.push_back(Vector3(0.532548f, -0.356761f, -0.932951f));
  points.push_back(Vector3(9.66002f, 3.89386f, -4.66659f));
  points.push_back(Vector3(-7.94183f, -1.65319f, -8.70785f));
  points.push_back(Vector3(3.42265f, 4.94308f, 4.21735f));
  points.push_back(Vector3(-7.81732f, 9.035f, -8.11335f));
  points.push_back(Vector3(6.14551f, -4.18928f, -3.17423f));
  points.push_back(Vector3(-8.25617f, -0.893277f, -7.34916f));
  points.push_back(Vector3(-0.827967f, 5.99353f, -7.14835f));
  points.push_back(Vector3(0.884122f, 9.23337f, 2.03528f));
  points.push_back(Vector3(-5.47533f, 1.03916f, 9.53551f));
  points.push_back(Vector3(9.96094f, -9.14487f, -8.07794f));
  points.push_back(Vector3(-3.32438f, -9.07712f, -4.45845f));
  points.push_back(Vector3(-9.03745f, -7.20634f, 5.23545f));
  points.push_back(Vector3(-2.54128f, -7.90643f, 4.41755f));
  points.push_back(Vector3(-1.67882f, -4.69344f, 4.30219f));
  points.push_back(Vector3(-0.675985f, -7.36015f, -5.66637f));
  points.push_back(Vector3(3.96344f, 2.02979f, 3.71685f));
  points.push_back(Vector3(-9.24863f, 4.09162f, -1.46153f));
  points.push_back(Vector3(8.39229f, -0.527055f, -6.36891f));
  points.push_back(Vector3(-9.02768f, 3.43364f, -9.20774f));
  points.push_back(Vector3(9.91089f, -7.28751f, 2.61513f));
  points.push_back(Vector3(-8.35933f, -2.62001f, 4.67086f));
  points.push_back(Vector3(-9.59899f, -2.96731f, -4.22529f));
  points.push_back(Vector3(3.31584f, 0.358592f, 7.33207f));
  points.push_back(Vector3(-9.19309f, 1.09165f, -3.58196f));
  points.push_back(Vector3(-3.03201f, -9.47264f, 9.8352f));
  points.push_back(Vector3(5.11216f, -9.78149f, 0.472121f));
  points.push_back(Vector3(7.82647f, -3.08267f, 9.6881f));
  points.push_back(Vector3(0.727867f, 1.1063f, 4.8204f));
  points.push_back(Vector3(-6.89261f, -1.96692f, -0.336619f));
  points.push_back(Vector3(-1.7655f, 4.07941f, -3.95611f));
  points.push_back(Vector3(-0.857875f, 0.398266f, -7.60918f));
  points.push_back(Vector3(-3.11014f, 8.67122f, 7.36992f));
  points.push_back(Vector3(9.52879f, -2.85745f, -4.54085f));
  points.push_back(Vector3(-3.24992f, -4.99741f, -8.67672f));
  points.push_back(Vector3(6.77175f, 4.25764f, -0.872524f));
  points.push_back(Vector3(6.07837f, -0.0918609f, -3.72051f));
  points.push_back(Vector3(-2.13355f, -1.86438f, 0.822474f));
  points.push_back(Vector3(-3.4727f, 5.374f, -0.813929f));
  points.push_back(Vector3(1.99683f, -5.08896f, -0.145573f));
  points.push_back(Vector3(0.793176f, 0.194403f, -3.2255f));
  points.push_back(Vector3(8.37458f, 2.47291f, 4.72579f));
  points.push_back(Vector3(0.600909f, 6.23585f, -3.93658f));
  points.push_back(Vector3(-3.15409f, 2.32276f, -3.45012f));
  points.push_back(Vector3(9.50438f, -9.09116f, -5.6444f));
  points.push_back(Vector3(5.63219f, 3.15897f, -2.09387f));
  points.push_back(Vector3(-2.51137f, -0.162663f, 1.79907f));
  points.push_back(Vector3(2.00903f, -6.25416f, -0.529496f));
  points.push_back(Vector3(-4.62264f, -9.25047f, -1.64769f));
  points.push_back(Vector3(-4.6086f, -8.28242f, 9.4586f));
  points.push_back(Vector3(-1.07578f, 3.9378f, -5.75976f));
  points.push_back(Vector3(0.577105f, 0.0161743f, 5.79211f));
  points.push_back(Vector3(7.85394f, -1.50792f, 1.13804f));
  points.push_back(Vector3(4.42854f, 6.13514f, 0.489823f));
  points.push_back(Vector3(6.37745f, 9.74242f, 2.92398f));
  points.push_back(Vector3(4.47493f, -6.25294f, 2.50587f));
  TestSphereRitter(points, file);
}

void SphereRitterTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.75951f, 9.78515f, -8.5638f));
  points.push_back(Vector3(-7.62078f, 1.61595f, -1.56102f));
  points.push_back(Vector3(2.172f, -5.04135f, -5.13352f));
  points.push_back(Vector3(-0.336009f, 3.73272f, -9.91821f));
  points.push_back(Vector3(3.55998f, -9.02951f, -1.05747f));
  points.push_back(Vector3(-6.80837f, -0.384228f, -8.1579f));
  points.push_back(Vector3(-6.37623f, 4.01715f, 2.25745f));
  points.push_back(Vector3(-0.614337f, 2.12867f, -7.51885f));
  points.push_back(Vector3(-7.22282f, 6.53005f, 5.2501f));
  points.push_back(Vector3(7.46757f, 0.75106f, 5.8855f));
  points.push_back(Vector3(-8.85922f, -3.44768f, -2.67861f));
  points.push_back(Vector3(-7.60674f, -0.128483f, -5.14878f));
  points.push_back(Vector3(1.1771f, 7.88934f, 2.23731f));
  points.push_back(Vector3(-8.9172f, -7.14835f, 0.702231f));
  points.push_back(Vector3(-6.54897f, 8.98801f, 2.96915f));
  points.push_back(Vector3(3.42448f, -7.64885f, -7.77642f));
  points.push_back(Vector3(-9.94263f, 1.96387f, 6.72842f));
  points.push_back(Vector3(-9.13816f, -6.43849f, -0.984222f));
  points.push_back(Vector3(-4.57808f, -2.38868f, -2.3307f));
  points.push_back(Vector3(5.06943f, 7.79717f, -8.10785f));
  points.push_back(Vector3(-8.40815f, 0.0796533f, -8.77926f));
  points.push_back(Vector3(-0.113834f, 1.90771f, -1.06967f));
  points.push_back(Vector3(7.1868f, 8.6932f, 2.60476f));
  points.push_back(Vector3(3.80963f, 6.6802f, 2.70608f));
  points.push_back(Vector3(-1.55858f, -3.37809f, 5.43931f));
  points.push_back(Vector3(0.343944f, 7.25456f, -5.42955f));
  points.push_back(Vector3(9.71923f, -0.739464f, -3.16385f));
  points.push_back(Vector3(2.02185f, -6.81387f, 8.15241f));
  points.push_back(Vector3(2.00842f, 1.7893f, -4.42366f));
  points.push_back(Vector3(-6.70827f, -2.48817f, -1.04831f));
  points.push_back(Vector3(3.72845f, -2.07984f, -7.65069f));
  points.push_back(Vector3(0.342723f, 5.63402f, 2.60537f));
  points.push_back(Vector3(1.19236f, 2.81838f, -1.88757f));
  points.push_back(Vector3(4.71053f, -1.5183f, -6.54897f));
  points.push_back(Vector3(-1.25217f, 5.08652f, -9.14121f));
  points.push_back(Vector3(-5.24766f, 2.70913f, 0.112003f));
  points.push_back(Vector3(-6.47084f, -1.93274f, -4.92538f));
  points.push_back(Vector3(7.21793f, 6.20594f, -1.74841f));
  points.push_back(Vector3(-0.706503f, 2.00476f, -5.75732f));
  points.push_back(Vector3(-7.23197f, -7.59941f, -4.90646f));
  points.push_back(Vector3(-9.40855f, -1.68065f, 7.62078f));
  points.push_back(Vector3(6.82608f, 4.20209f, 8.14447f));
  points.push_back(Vector3(-5.02976f, 7.93634f, -9.50865f));
  TestSphereRitter(points, file);
}

void SphereRitterTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.69686f, 6.42872f, 2.96182f));
  points.push_back(Vector3(0.593585f, -2.87332f, -4.12763f));
  points.push_back(Vector3(-5.29344f, 2.66579f, 6.3744f));
  points.push_back(Vector3(-0.409253f, -4.38398f, 4.41084f));
  points.push_back(Vector3(5.7213f, -6.22181f, -1.56163f));
  points.push_back(Vector3(-6.42628f, 3.97504f, 5.88855f));
  points.push_back(Vector3(9.18882f, -2.99905f, -4.08063f));
  points.push_back(Vector3(-4.39619f, 6.71316f, -5.54796f));
  points.push_back(Vector3(3.3787f, 4.1435f, -2.70913f));
  points.push_back(Vector3(7.29423f, -8.36482f, -2.49855f));
  points.push_back(Vector3(7.40043f, -7.29789f, 9.3292f));
  points.push_back(Vector3(-0.735191f, 7.7752f, 3.99762f));
  points.push_back(Vector3(9.94812f, -1.37608f, -8.12983f));
  points.push_back(Vector3(7.7337f, 3.05643f, 9.20164f));
  points.push_back(Vector3(1.87536f, -1.8247f, -6.63259f));
  points.push_back(Vector3(4.32844f, 5.38133f, -0.216376f));
  points.push_back(Vector3(-4.20515f, 4.39253f, -4.15326f));
  points.push_back(Vector3(-1.67333f, 4.60494f, 1.87719f));
  points.push_back(Vector3(-0.0283819f, -8.74508f, 4.79598f));
  points.push_back(Vector3(-0.274972f, 5.10971f, -2.03955f));
  points.push_back(Vector3(3.83099f, 8.64437f, 9.85046f));
  points.push_back(Vector3(0.536211f, -8.15668f, -1.01535f));
  points.push_back(Vector3(-9.11618f, -4.32295f, 8.44844f));
  points.push_back(Vector3(-7.18558f, 9.28648f, 8.33369f));
  points.push_back(Vector3(4.27351f, -8.19086f, 9.33653f));
  points.push_back(Vector3(7.74712f, -4.64827f, 6.5923f));
  points.push_back(Vector3(-6.9982f, -3.502f, -1.51646f));
  points.push_back(Vector3(3.14127f, 3.21573f, -6.8157f));
  points.push_back(Vector3(5.69994f, -8.10358f, -0.939054f));
  points.push_back(Vector3(-0.644856f, 5.95508f, -4.81857f));
  points.push_back(Vector3(-4.00189f, -8.67f, -7.82769f));
  points.push_back(Vector3(-1.42674f, -5.74084f, 7.52922f));
  points.push_back(Vector3(1.76305f, 1.8424f, 8.52473f));
  points.push_back(Vector3(-8.07245f, 8.74569f, -4.90951f));
  points.push_back(Vector3(2.01819f, 4.27656f, -4.40413f));
  points.push_back(Vector3(-4.29853f, -1.85888f, -2.29957f));
  points.push_back(Vector3(2.94839f, -8.25373f, -7.22953f));
  points.push_back(Vector3(-5.69201f, 5.26902f, 7.45293f));
  points.push_back(Vector3(0.342723f, -0.56856f, -9.36522f));
  points.push_back(Vector3(-3.62896f, -3.1431f, 5.12436f));
  points.push_back(Vector3(-7.26005f, -2.13843f, -7.08853f));
  points.push_back(Vector3(3.54228f, -6.75222f, 2.52846f));
  points.push_back(Vector3(-0.0894195f, 0.835291f, 2.62734f));
  points.push_back(Vector3(2.21839f, 2.55654f, -7.83319f));
  points.push_back(Vector3(-2.41676f, -9.77905f, 0.753502f));
  points.push_back(Vector3(8.49605f, 3.22123f, 7.28874f));
  points.push_back(Vector3(-5.94348f, -4.26435f, -1.94433f));
  points.push_back(Vector3(-0.906705f, -8.21833f, -3.07047f));
  points.push_back(Vector3(-9.99695f, 0.768762f, 1.90161f));
  points.push_back(Vector3(-1.95105f, 1.46764f, -5.86718f));
  points.push_back(Vector3(1.86071f, 8.05902f, -8.33979f));
  points.push_back(Vector3(-6.76077f, 4.42793f, -6.14246f));
  points.push_back(Vector3(-5.73107f, 3.87432f, 1.71178f));
  points.push_back(Vector3(0.732139f, -1.39927f, -7.38334f));
  points.push_back(Vector3(0.465407f, -1.06052f, -7.67083f));
  points.push_back(Vector3(-8.89706f, -8.19391f, 7.05191f));
  points.push_back(Vector3(-6.03076f, -8.89401f, 8.4753f));
  points.push_back(Vector3(-7.3217f, -6.80105f, -3.94024f));
  points.push_back(Vector3(-7.24479f, 1.20273f, 7.16422f));
  TestSphereRitter(points, file);
}

void SphereRitterTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.47407f, 1.18259f, 4.14533f));
  points.push_back(Vector3(5.54674f, 2.22449f, 4.30158f));
  points.push_back(Vector3(7.47185f, 2.55898f, -4.90402f));
  points.push_back(Vector3(-0.687582f, 5.50584f, 7.42241f));
  points.push_back(Vector3(-0.879238f, 4.87533f, -4.51888f));
  points.push_back(Vector3(-1.20518f, -2.76406f, 5.42528f));
  points.push_back(Vector3(-5.66454f, 5.13291f, -3.72356f));
  points.push_back(Vector3(-6.12903f, -6.88284f, -6.03504f));
  points.push_back(Vector3(1.62877f, 0.115055f, -6.07044f));
  points.push_back(Vector3(-6.4391f, -5.25193f, -8.98862f));
  points.push_back(Vector3(-7.4572f, -2.40272f, 1.64525f));
  points.push_back(Vector3(-2.59316f, -0.207831f, -6.53859f));
  points.push_back(Vector3(-9.52269f, -6.13697f, 4.88937f));
  points.push_back(Vector3(4.81552f, 6.36403f, 6.70949f));
  points.push_back(Vector3(-6.22364f, 9.03684f, 1.55675f));
  points.push_back(Vector3(-3.96222f, 5.64318f, 0.171819f));
  points.push_back(Vector3(9.08322f, 7.26676f, 0.727867f));
  points.push_back(Vector3(-1.98096f, -2.00537f, -2.09937f));
  points.push_back(Vector3(2.54128f, -6.02832f, -1.66173f));
  points.push_back(Vector3(-4.88815f, 7.622f, -5.75549f));
  points.push_back(Vector3(-6.01794f, 9.76867f, 5.95996f));
  points.push_back(Vector3(-8.89584f, -8.58272f, 2.52358f));
  points.push_back(Vector3(5.20676f, 2.73598f, 2.65908f));
  points.push_back(Vector3(7.61528f, 1.80395f, -7.63237f));
  points.push_back(Vector3(-9.6234f, 9.97314f, -7.85638f));
  points.push_back(Vector3(7.27653f, -6.62404f, 1.27537f));
  points.push_back(Vector3(6.90054f, 8.30073f, 0.555742f));
  points.push_back(Vector3(-9.41954f, 1.64586f, -9.51048f));
  points.push_back(Vector3(-8.5345f, -4.85397f, 6.88528f));
  points.push_back(Vector3(5.50706f, -8.40632f, 7.07266f));
  points.push_back(Vector3(-9.37864f, 6.41896f, 0.50325f));
  points.push_back(Vector3(1.39073f, -3.40312f, 5.48692f));
  points.push_back(Vector3(0.945158f, 1.22471f, -8.6877f));
  points.push_back(Vector3(5.60167f, 7.07511f, 5.64135f));
  points.push_back(Vector3(2.96793f, -4.03668f, -0.106509f));
  points.push_back(Vector3(-4.17463f, 5.43687f, -7.34733f));
  points.push_back(Vector3(8.04254f, 5.20249f, -9.58983f));
  points.push_back(Vector3(-5.29038f, 2.07007f, 9.12351f));
  points.push_back(Vector3(0.826136f, -3.6607f, 0.308542f));
  points.push_back(Vector3(-0.245674f, 8.71273f, 5.88122f));
  points.push_back(Vector3(-5.5327f, 1.78076f, -9.4293f));
  points.push_back(Vector3(-7.84234f, -6.91458f, -5.44359f));
  points.push_back(Vector3(3.4257f, 8.83663f, -9.04416f));
  points.push_back(Vector3(-3.67168f, -5.34288f, -2.58522f));
  points.push_back(Vector3(2.65542f, -3.85113f, 8.75607f));
  points.push_back(Vector3(-0.526444f, 8.29829f, 9.11801f));
  points.push_back(Vector3(4.10932f, -1.1948f, 9.64049f));
  points.push_back(Vector3(0.557573f, 2.95572f, 4.05316f));
  points.push_back(Vector3(4.66597f, -5.22629f, 7.62871f));
  points.push_back(Vector3(-7.51335f, -1.23386f, -6.00452f));
  points.push_back(Vector3(6.21204f, 5.52721f, 0.655233f));
  points.push_back(Vector3(3.14921f, 0.736412f, -3.3372f));
  points.push_back(Vector3(1.12827f, 6.06738f, 1.03977f));
  points.push_back(Vector3(2.36854f, 2.80252f, 8.77682f));
  points.push_back(Vector3(-2.58644f, -3.34269f, -1.04343f));
  points.push_back(Vector3(-4.52803f, -4.83016f, -0.103458f));
  points.push_back(Vector3(9.59899f, -7.17887f, 4.24055f));
  points.push_back(Vector3(2.49489f, -7.11661f, 8.20612f));
  points.push_back(Vector3(6.94327f, -5.8916f, 0.00762939f));
  points.push_back(Vector3(-2.46376f, -2.96182f, -1.0239f));
  points.push_back(Vector3(-8.29402f, 8.26533f, -6.80349f));
  points.push_back(Vector3(8.00653f, 4.47798f, -6.78701f));
  points.push_back(Vector3(0.00946045f, 8.25922f, 4.6556f));
  points.push_back(Vector3(0.849941f, -0.341502f, 0.423292f));
  points.push_back(Vector3(-1.53783f, -2.36366f, 7.37968f));
  points.push_back(Vector3(4.93576f, -3.84075f, 8.06818f));
  points.push_back(Vector3(-6.16443f, 7.35343f, -1.57018f));
  points.push_back(Vector3(8.03644f, 7.53471f, 3.61003f));
  points.push_back(Vector3(3.10831f, -2.86477f, 0.40498f));
  points.push_back(Vector3(-4.2204f, 0.385449f, 5.54186f));
  points.push_back(Vector3(9.66613f, -1.02451f, -5.37767f));
  points.push_back(Vector3(1.0831f, -0.893887f, -9.99756f));
  points.push_back(Vector3(0.882291f, -7.86615f, -7.09281f));
  points.push_back(Vector3(-1.60375f, -5.68224f, -8.04437f));
  points.push_back(Vector3(5.12803f, -9.53307f, 0.537431f));
  points.push_back(Vector3(0.100406f, -2.02246f, 6.9805f));
  points.push_back(Vector3(5.06882f, 6.51967f, -7.99615f));
  points.push_back(Vector3(5.0914f, -9.94995f, -6.01489f));
  points.push_back(Vector3(1.32054f, 5.16282f, -3.72417f));
  points.push_back(Vector3(4.46211f, -3.00089f, -6.90603f));
  points.push_back(Vector3(-1.28513f, 5.0444f, -8.25373f));
  points.push_back(Vector3(-1.21982f, -6.36769f, -0.904874f));
  points.push_back(Vector3(9.00266f, 1.32542f, 5.43931f));
  TestSphereRitter(points, file);
}

void SphereRitterTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.65535f, 4.19477f, -8.06635f));
  points.push_back(Vector3(9.0228f, 9.33287f, 6.2035f));
  points.push_back(Vector3(-5.40025f, 5.14573f, -1.99072f));
  points.push_back(Vector3(0.788293f, -6.27979f, 4.97543f));
  points.push_back(Vector3(9.48363f, 5.85925f, 2.8843f));
  points.push_back(Vector3(-4.19721f, 7.71172f, 2.05115f));
  points.push_back(Vector3(-7.34672f, -1.47435f, 2.2367f));
  points.push_back(Vector3(-5.97095f, -0.836512f, 4.36079f));
  points.push_back(Vector3(-9.45006f, 5.88855f, -9.58983f));
  points.push_back(Vector3(-1.53111f, -0.281075f, -1.21555f));
  points.push_back(Vector3(5.86474f, 3.62041f, 7.88812f));
  points.push_back(Vector3(8.9172f, -4.97177f, 7.022f));
  points.push_back(Vector3(-3.70281f, -6.01856f, -9.43907f));
  points.push_back(Vector3(3.43608f, -7.57378f, 6.23402f));
  points.push_back(Vector3(1.6068f, -2.28797f, -5.0969f));
  points.push_back(Vector3(0.273751f, 2.93008f, -0.143742f));
  points.push_back(Vector3(0.786462f, -6.31886f, 5.51256f));
  points.push_back(Vector3(-4.23139f, 5.86535f, 8.63155f));
  points.push_back(Vector3(4.20331f, 6.76443f, -1.41514f));
  points.push_back(Vector3(-7.39006f, 8.081f, 1.01352f));
  points.push_back(Vector3(-1.76611f, 6.35487f, 9.12107f));
  points.push_back(Vector3(-9.98291f, -1.98645f, -2.85195f));
  points.push_back(Vector3(-5.18052f, -6.74917f, 3.61248f));
  points.push_back(Vector3(5.59008f, 0.383618f, 4.0904f));
  points.push_back(Vector3(7.91803f, 3.96527f, 6.11255f));
  points.push_back(Vector3(0.159001f, -7.51885f, -5.65539f));
  points.push_back(Vector3(-4.39253f, -8.78903f, -3.62774f));
  points.push_back(Vector3(4.53658f, 0.421461f, 7.67937f));
  points.push_back(Vector3(3.03079f, -9.99451f, 6.77786f));
  points.push_back(Vector3(3.00882f, 5.76281f, -0.0888091f));
  points.push_back(Vector3(9.99023f, -7.09098f, 2.48512f));
  points.push_back(Vector3(-4.64888f, -7.80267f, 4.64644f));
  points.push_back(Vector3(0.359813f, -2.02307f, 9.04721f));
  points.push_back(Vector3(-0.508744f, 0.932341f, -2.9313f));
  points.push_back(Vector3(0.945768f, 1.46764f, 6.40187f));
  points.push_back(Vector3(-6.77541f, -5.4912f, -4.49629f));
  points.push_back(Vector3(-2.34657f, 6.76992f, 9.82849f));
  points.push_back(Vector3(8.1341f, 4.66964f, 9.01608f));
  points.push_back(Vector3(-0.81637f, -6.14856f, -0.0509661f));
  points.push_back(Vector3(4.50545f, -7.17032f, 0.265816f));
  points.push_back(Vector3(-4.43587f, 2.34657f, -2.52724f));
  points.push_back(Vector3(-4.84298f, -5.5327f, 4.78378f));
  points.push_back(Vector3(3.06986f, 8.58577f, -3.31095f));
  points.push_back(Vector3(-8.88607f, 3.57524f, -6.84927f));
  points.push_back(Vector3(-8.96603f, 5.08286f, 7.25394f));
  points.push_back(Vector3(-4.44624f, -7.07755f, 3.88165f));
  points.push_back(Vector3(2.19153f, 2.56142f, -1.02206f));
  points.push_back(Vector3(-3.13211f, -7.37358f, -5.48509f));
  points.push_back(Vector3(-0.99765f, -7.85577f, -8.27509f));
  points.push_back(Vector3(9.50682f, -4.03912f, -0.893277f));
  points.push_back(Vector3(6.42628f, -8.66573f, -5.4033f));
  points.push_back(Vector3(5.09751f, -0.0143439f, 4.38398f));
  points.push_back(Vector3(9.3054f, -5.71459f, 7.83563f));
  points.push_back(Vector3(8.53877f, -8.11151f, 6.89688f));
  points.push_back(Vector3(-7.60674f, -2.73843f, 9.82238f));
  points.push_back(Vector3(6.92251f, -7.56279f, -8.0401f));
  points.push_back(Vector3(-4.3498f, -3.57219f, -7.31925f));
  points.push_back(Vector3(-4.23078f, -1.8601f, 4.58724f));
  points.push_back(Vector3(9.16562f, -0.127262f, -4.17829f));
  points.push_back(Vector3(9.57213f, -3.9024f, 5.48753f));
  points.push_back(Vector3(3.53191f, 1.41331f, 9.76012f));
  points.push_back(Vector3(-5.36363f, -7.40837f, 3.96954f));
  points.push_back(Vector3(-4.88754f, -7.16117f, -4.7557f));
  points.push_back(Vector3(4.11542f, -9.28159f, -3.16446f));
  points.push_back(Vector3(-6.02832f, 6.28712f, -2.38502f));
  points.push_back(Vector3(-9.51964f, 6.32435f, 0.50264f));
  points.push_back(Vector3(7.75079f, 1.92053f, 5.87695f));
  points.push_back(Vector3(1.98279f, -2.3777f, 5.1915f));
  points.push_back(Vector3(9.94934f, -9.50194f, 2.73049f));
  points.push_back(Vector3(-0.65218f, -5.2971f, 7.57195f));
  points.push_back(Vector3(-1.52135f, 8.26472f, 8.25373f));
  points.push_back(Vector3(-0.347606f, 3.1486f, -5.77929f));
  points.push_back(Vector3(2.40577f, 5.39293f, 8.56136f));
  points.push_back(Vector3(8.39961f, -6.34327f, -4.84359f));
  TestSphereRitter(points, file);
}

void SphereRitterTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.45433f, -6.48793f, -0.339671f));
  points.push_back(Vector3(-0.497146f, 4.83016f, 5.1323f));
  points.push_back(Vector3(-7.17582f, 1.59886f, -3.21818f));
  points.push_back(Vector3(5.53392f, 9.92126f, -5.80493f));
  points.push_back(Vector3(-9.02829f, -3.87066f, -8.66939f));
  points.push_back(Vector3(-7.71172f, 3.98175f, -7.54021f));
  points.push_back(Vector3(4.58357f, 0.839564f, 8.28059f));
  points.push_back(Vector3(5.05478f, 4.27045f, 7.53166f));
  points.push_back(Vector3(-3.54167f, -0.759606f, -4.87777f));
  points.push_back(Vector3(6.02954f, -6.06861f, 9.52025f));
  points.push_back(Vector3(-8.61995f, -9.61852f, -1.74596f));
  points.push_back(Vector3(-5.85925f, -4.4261f, -8.82748f));
  points.push_back(Vector3(-6.84988f, 5.49242f, 9.84313f));
  points.push_back(Vector3(-5.93921f, -4.59456f, 1.94128f));
  points.push_back(Vector3(2.92398f, -8.6755f, 2.72744f));
  points.push_back(Vector3(-3.25846f, 5.92273f, 7.0397f));
  points.push_back(Vector3(0.710166f, 9.64049f, 0.00213623f));
  points.push_back(Vector3(6.51357f, -2.96854f, -3.20231f));
  points.push_back(Vector3(9.1113f, -9.39146f, 0.990936f));
  points.push_back(Vector3(4.66109f, 0.871303f, 5.56688f));
  points.push_back(Vector3(-3.46904f, 3.40129f, 8.764f));
  points.push_back(Vector3(8.96603f, 6.86148f, 5.5504f));
  points.push_back(Vector3(-1.2833f, 2.44301f, -1.19663f));
  points.push_back(Vector3(5.4564f, -6.75588f, -8.21161f));
  points.push_back(Vector3(4.99374f, 5.18418f, -3.76141f));
  points.push_back(Vector3(5.05844f, -7.94488f, 2.48695f));
  points.push_back(Vector3(-8.43806f, 9.31211f, -7.11844f));
  points.push_back(Vector3(6.47084f, -1.45787f, -3.38664f));
  points.push_back(Vector3(-8.86471f, -3.41472f, -2.48573f));
  points.push_back(Vector3(-0.0894195f, -3.89203f, -9.44761f));
  points.push_back(Vector3(8.52657f, 9.42442f, -3.94879f));
  points.push_back(Vector3(-3.12967f, -3.67229f, -9.82788f));
  points.push_back(Vector3(8.11029f, -2.49001f, 4.77828f));
  points.push_back(Vector3(-8.7524f, -7.01468f, -3.56426f));
  points.push_back(Vector3(-6.20472f, 7.48039f, -7.55669f));
  points.push_back(Vector3(-0.434889f, 1.96081f, 4.11054f));
  points.push_back(Vector3(-2.19092f, 1.80639f, 7.92169f));
  points.push_back(Vector3(5.08835f, 7.55058f, 6.78457f));
  points.push_back(Vector3(-6.29688f, -4.36262f, 2.10059f));
  points.push_back(Vector3(-5.10971f, 3.0607f, -0.300607f));
  points.push_back(Vector3(-5.97766f, 5.28001f, -2.00537f));
  points.push_back(Vector3(2.88125f, 1.65807f, 7.43278f));
  points.push_back(Vector3(6.55751f, 1.54515f, 4.36934f));
  points.push_back(Vector3(7.91681f, 5.07126f, 8.64681f));
  points.push_back(Vector3(7.51701f, 7.96686f, 7.88812f));
  points.push_back(Vector3(-3.03751f, -4.71847f, -2.48878f));
  points.push_back(Vector3(7.80877f, -9.67772f, -0.534379f));
  points.push_back(Vector3(1.26865f, 0.299387f, 3.47392f));
  points.push_back(Vector3(2.39845f, 2.53822f, -5.38743f));
  points.push_back(Vector3(1.20762f, 2.0603f, 0.312204f));
  points.push_back(Vector3(4.95346f, -2.63771f, -2.42897f));
  points.push_back(Vector3(5.3325f, 4.8088f, 4.34004f));
  points.push_back(Vector3(3.14066f, -9.2822f, -2.43629f));
  points.push_back(Vector3(-6.24805f, 8.63216f, 2.78909f));
  points.push_back(Vector3(6.53432f, -1.17588f, -5.15488f));
  points.push_back(Vector3(7.70989f, -2.46193f, -0.814539f));
  points.push_back(Vector3(1.65624f, -0.426954f, -2.8721f));
  points.push_back(Vector3(-1.33946f, -2.87149f, 5.56566f));
  points.push_back(Vector3(-0.381787f, -3.29875f, 5.72985f));
  points.push_back(Vector3(-6.18397f, 6.02039f, 4.07086f));
  points.push_back(Vector3(3.77483f, -0.426344f, 8.02179f));
  points.push_back(Vector3(3.41349f, 0.215156f, -4.67696f));
  points.push_back(Vector3(0.125431f, 5.33006f, -9.08567f));
  points.push_back(Vector3(0.481277f, -6.91824f, 0.588092f));
  points.push_back(Vector3(4.51582f, -3.72539f, -8.26899f));
  points.push_back(Vector3(-0.909147f, 8.04132f, 1.45115f));
  points.push_back(Vector3(-9.61242f, -3.77667f, -9.6588f));
  points.push_back(Vector3(-5.57176f, 8.88852f, -2.78542f));
  points.push_back(Vector3(-8.47652f, -2.48512f, 2.42592f));
  points.push_back(Vector3(0.126041f, -3.52336f, 8.82687f));
  points.push_back(Vector3(8.30317f, 8.09931f, 0.625935f));
  points.push_back(Vector3(-2.47658f, 2.7195f, -8.51131f));
  points.push_back(Vector3(6.04236f, 7.9931f, -9.27305f));
  points.push_back(Vector3(-9.49706f, -3.98053f, 8.60225f));
  points.push_back(Vector3(2.4723f, -1.02817f, 6.26453f));
  points.push_back(Vector3(-9.35789f, -1.15024f, -0.119938f));
  points.push_back(Vector3(0.8005f, -6.89749f, -0.296335f));
  points.push_back(Vector3(-6.72292f, -9.26267f, -1.12094f));
  TestSphereRitter(points, file);
}

void SphereRitterTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.79553f, -8.30073f, -4.82589f));
  points.push_back(Vector3(8.01202f, 8.29707f, -4.70138f));
  points.push_back(Vector3(7.97784f, 7.11905f, -6.53493f));
  points.push_back(Vector3(7.20878f, 5.35813f, 9.59838f));
  points.push_back(Vector3(0.334789f, 9.18821f, -4.86801f));
  points.push_back(Vector3(2.70974f, 4.98459f, 9.4171f));
  points.push_back(Vector3(-6.48915f, 5.56749f, -3.15714f));
  points.push_back(Vector3(9.30723f, -2.82937f, -4.58296f));
  points.push_back(Vector3(-7.65618f, 5.03464f, -8.85311f));
  points.push_back(Vector3(6.46046f, -9.035f, 1.92846f));
  points.push_back(Vector3(-1.82836f, 4.33699f, 4.27045f));
  points.push_back(Vector3(9.8352f, 5.08347f, 9.40184f));
  points.push_back(Vector3(-1.40782f, -2.41737f, -8.07001f));
  points.push_back(Vector3(-8.09137f, 1.96326f, -5.3264f));
  points.push_back(Vector3(-0.603961f, -7.75811f, -4.0611f));
  points.push_back(Vector3(-1.67211f, 0.166326f, 9.30418f));
  points.push_back(Vector3(2.92215f, -5.99841f, 9.87243f));
  points.push_back(Vector3(-9.80285f, 9.62401f, 7.51274f));
  points.push_back(Vector3(3.22367f, 4.31806f, -2.72134f));
  points.push_back(Vector3(-3.11808f, -9.03561f, -5.01083f));
  points.push_back(Vector3(-0.754113f, 2.04199f, 3.27494f));
  points.push_back(Vector3(-8.24091f, -3.38115f, 0.504471f));
  points.push_back(Vector3(-4.75814f, -3.51299f, -7.53655f));
  points.push_back(Vector3(7.1453f, 0.0253308f, 6.34327f));
  points.push_back(Vector3(-4.68246f, 6.20289f, 5.34532f));
  points.push_back(Vector3(6.84805f, -7.42851f, -6.36097f));
  points.push_back(Vector3(-1.08615f, -1.92175f, 4.36079f));
  points.push_back(Vector3(-2.03101f, -1.56102f, 7.1569f));
  points.push_back(Vector3(-1.41881f, -2.30445f, -8.1811f));
  points.push_back(Vector3(-9.84008f, -2.32276f, 3.46965f));
  points.push_back(Vector3(3.16446f, -3.43364f, -2.20313f));
  points.push_back(Vector3(3.10404f, -9.74181f, -0.638142f));
  points.push_back(Vector3(-9.03378f, -2.43934f, 9.09299f));
  points.push_back(Vector3(3.94147f, -9.10703f, 6.04114f));
  points.push_back(Vector3(-8.905f, -1.07761f, -6.97501f));
  points.push_back(Vector3(8.30439f, 8.67977f, 1.99499f));
  points.push_back(Vector3(9.49644f, 2.33375f, -5.26048f));
  points.push_back(Vector3(3.93902f, 9.57762f, 5.48326f));
  points.push_back(Vector3(-8.28486f, 3.93048f, 6.25965f));
  points.push_back(Vector3(-2.77566f, -4.14716f, -0.741905f));
  points.push_back(Vector3(-0.400097f, -8.38252f, -7.71783f));
  points.push_back(Vector3(2.584f, -2.55776f, -2.36061f));
  points.push_back(Vector3(6.54836f, 9.8645f, 4.15326f));
  points.push_back(Vector3(-4.51949f, 8.20307f, -7.93756f));
  points.push_back(Vector3(-2.0127f, 6.95181f, -7.90216f));
  points.push_back(Vector3(-6.2273f, 7.70928f, 8.11152f));
  points.push_back(Vector3(-7.00308f, -8.11396f, 9.10092f));
  points.push_back(Vector3(2.24769f, -0.867641f, -2.58095f));
  points.push_back(Vector3(-8.02423f, 1.33457f, 0.332958f));
  points.push_back(Vector3(-6.37135f, -1.67272f, -5.90747f));
  TestSphereRitter(points, file);
}

void SphereRitterTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.19553f, 6.35182f, -2.89468f));
  points.push_back(Vector3(8.82382f, 3.24747f, -3.29875f));
  points.push_back(Vector3(-0.215766f, 9.33226f, 6.90542f));
  points.push_back(Vector3(-2.28919f, -1.81494f, -2.30995f));
  points.push_back(Vector3(7.59697f, 9.00998f, -7.81671f));
  points.push_back(Vector3(7.99127f, -8.12616f, -5.57787f));
  points.push_back(Vector3(9.84375f, 7.42119f, -5.94836f));
  points.push_back(Vector3(-6.59963f, -4.36811f, 4.10932f));
  points.push_back(Vector3(-4.85885f, -3.29264f, 5.86474f));
  points.push_back(Vector3(-6.69607f, 6.83035f, 6.98782f));
  points.push_back(Vector3(9.49278f, 7.49138f, 3.62163f));
  points.push_back(Vector3(-5.15854f, 2.37953f, 6.08692f));
  points.push_back(Vector3(-3.73455f, -8.96298f, -6.83218f));
  TestSphereRitter(points, file);
}

void SphereRitterTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.29035f, 4.73128f, 7.26066f));
  points.push_back(Vector3(-3.24198f, 1.41209f, 4.79659f));
  points.push_back(Vector3(2.24342f, -1.81433f, 2.26051f));
  points.push_back(Vector3(-5.91601f, -2.80007f, 9.89563f));
  points.push_back(Vector3(2.27027f, -6.01001f, 1.62328f));
  points.push_back(Vector3(-7.63054f, 3.12967f, 0.0442517f));
  points.push_back(Vector3(-2.86538f, 6.09851f, -2.71645f));
  points.push_back(Vector3(3.16202f, -3.76507f, 7.8515f));
  points.push_back(Vector3(1.62023f, 1.09104f, -9.54589f));
  points.push_back(Vector3(-1.29063f, -4.12397f, -4.31928f));
  points.push_back(Vector3(1.05319f, 4.47493f, 3.90545f));
  points.push_back(Vector3(1.5952f, -4.62264f, -0.907315f));
  points.push_back(Vector3(5.91235f, 2.0951f, 2.01147f));
  points.push_back(Vector3(-8.29585f, -0.204779f, -2.48024f));
  points.push_back(Vector3(7.83868f, 7.95892f, -0.2884f));
  points.push_back(Vector3(2.08533f, -9.85717f, 6.20472f));
  points.push_back(Vector3(9.99878f, -5.37278f, 6.87979f));
  points.push_back(Vector3(-0.805383f, 2.56508f, -1.08066f));
  points.push_back(Vector3(-1.77709f, 1.91809f, -4.10932f));
  points.push_back(Vector3(-0.819422f, 2.5425f, -3.80718f));
  points.push_back(Vector3(3.74981f, 6.40614f, -0.0930816f));
  points.push_back(Vector3(-7.35771f, -8.21833f, 1.14475f));
  points.push_back(Vector3(6.25782f, 0.677205f, -9.95361f));
  points.push_back(Vector3(8.34407f, -8.6285f, 7.11051f));
  points.push_back(Vector3(-0.323802f, 9.89257f, 3.73577f));
  points.push_back(Vector3(-5.96728f, 1.75329f, -9.52025f));
  points.push_back(Vector3(-4.54085f, -3.43303f, 6.54408f));
  TestSphereRitter(points, file);
}

void SphereRitterTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.89319f, -9.75585f, 0.906705f));
  points.push_back(Vector3(8.34529f, -0.183416f, -2.13904f));
  points.push_back(Vector3(-5.10727f, -5.33433f, -0.350658f));
  points.push_back(Vector3(-8.92026f, 9.78149f, 8.28059f));
  points.push_back(Vector3(-1.7777f, -5.66698f, -8.57417f));
  points.push_back(Vector3(4.17585f, 2.93252f, -1.6306f));
  points.push_back(Vector3(-1.24058f, 2.00964f, -4.56343f));
  points.push_back(Vector3(6.3152f, 8.11945f, 5.62975f));
  points.push_back(Vector3(-4.3907f, -5.70299f, -6.13269f));
  points.push_back(Vector3(-6.28224f, -7.89727f, -7.73247f));
  points.push_back(Vector3(8.42219f, -7.8045f, 7.63237f));
  points.push_back(Vector3(0.647908f, -5.17441f, 9.0112f));
  points.push_back(Vector3(-2.8898f, 8.41609f, -5.01511f));
  points.push_back(Vector3(1.93579f, 6.76626f, 8.26411f));
  points.push_back(Vector3(6.68935f, -0.00946045f, -7.64397f));
  points.push_back(Vector3(-5.54491f, -7.44804f, 9.13449f));
  points.push_back(Vector3(4.60921f, -1.00558f, -6.01306f));
  points.push_back(Vector3(2.83181f, 0.015564f, -8.24824f));
  points.push_back(Vector3(-6.6509f, 8.12922f, 5.10361f));
  points.push_back(Vector3(0.150456f, -2.63466f, 7.86492f));
  points.push_back(Vector3(-7.69524f, -5.95019f, 4.97726f));
  points.push_back(Vector3(9.63622f, -4.44441f, -8.54366f));
  points.push_back(Vector3(-7.2747f, 2.14942f, 3.74615f));
  points.push_back(Vector3(-9.33042f, 0.798669f, 9.57823f));
  points.push_back(Vector3(0.147405f, 4.50667f, -7.13309f));
  points.push_back(Vector3(4.55428f, 5.93616f, 2.21534f));
  points.push_back(Vector3(-7.16666f, -9.58556f, -4.04645f));
  points.push_back(Vector3(-5.28611f, -8.37458f, -6.66494f));
  points.push_back(Vector3(4.8088f, 8.77377f, 7.14591f));
  points.push_back(Vector3(-5.03159f, -8.53999f, 5.76769f));
  points.push_back(Vector3(-9.48363f, 6.2157f, 0.401928f));
  points.push_back(Vector3(-9.37925f, 1.73559f, -7.24479f));
  points.push_back(Vector3(7.57439f, 6.58132f, 7.87286f));
  points.push_back(Vector3(-7.04093f, 9.12412f, 1.42491f));
  points.push_back(Vector3(6.92984f, -1.19053f, 1.14536f));
  points.push_back(Vector3(0.74984f, -9.51231f, 0.4532f));
  points.push_back(Vector3(8.51802f, -6.05396f, 8.27265f));
  TestSphereRitter(points, file);
}

void SphereRitterTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.52452f, 3.90118f, -1.95959f));
  points.push_back(Vector3(5.45518f, 4.48714f, 1.24851f));
  points.push_back(Vector3(6.33534f, 4.13251f, 4.05622f));
  points.push_back(Vector3(-3.12906f, -6.92007f, -6.74612f));
  points.push_back(Vector3(-1.9657f, 2.66213f, -2.74514f));
  points.push_back(Vector3(1.90649f, -8.91293f, -1.14536f));
  points.push_back(Vector3(5.47533f, -1.70141f, 4.41938f));
  points.push_back(Vector3(-5.65416f, 3.51909f, -2.31666f));
  points.push_back(Vector3(4.50545f, 2.99966f, -0.0772119f));
  points.push_back(Vector3(-5.92944f, 0.459304f, -5.95202f));
  points.push_back(Vector3(7.27348f, -1.1478f, 0.655233f));
  points.push_back(Vector3(4.72152f, -3.18155f, -0.534989f));
  points.push_back(Vector3(9.23521f, -6.76992f, -7.97601f));
  points.push_back(Vector3(1.22288f, -6.87796f, 0.219429f));
  points.push_back(Vector3(-3.29569f, 5.43077f, 0.168157f));
  points.push_back(Vector3(-3.16752f, 3.34086f, 6.91824f));
  points.push_back(Vector3(-0.942106f, 7.4926f, -2.77261f));
  points.push_back(Vector3(4.10932f, 1.59764f, 5.54064f));
  points.push_back(Vector3(2.05786f, 4.75997f, -1.99133f));
  points.push_back(Vector3(9.8529f, 2.58644f, 0.477614f));
  points.push_back(Vector3(5.85864f, 0.549638f, 9.97986f));
  points.push_back(Vector3(-6.83645f, 3.24442f, 1.94433f));
  points.push_back(Vector3(5.4912f, 6.48732f, 6.8627f));
  points.push_back(Vector3(-9.16929f, 8.08283f, 5.52782f));
  TestSphereRitter(points, file);
}

void SphereRitterTest51(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-8.39595f, -2.96548f, 9.37925f));
  points.push_back(Vector3(-1.65746f, 7.74956f, -2.22755f));
  points.push_back(Vector3(1.5598f, -7.12149f, 8.13349f));
  points.push_back(Vector3(-7.03482f, 1.98523f, 8.47041f));
  points.push_back(Vector3(6.10828f, -1.26133f, 3.73699f));
  points.push_back(Vector3(-8.49422f, -4.39619f, 4.01349f));
  points.push_back(Vector3(-2.08655f, 5.10727f, 8.00775f));
  points.push_back(Vector3(2.37526f, -9.47447f, 0.939054f));
  points.push_back(Vector3(9.23398f, 3.3549f, 1.15207f));
  points.push_back(Vector3(5.74389f, -7.38273f, -3.74798f));
  points.push_back(Vector3(3.97687f, 0.730308f, -6.74429f));
  points.push_back(Vector3(9.6173f, 1.83386f, 1.93579f));
  points.push_back(Vector3(-0.588092f, -0.473342f, -6.11377f));
  points.push_back(Vector3(-0.307932f, 6.62282f, 9.7705f));
  points.push_back(Vector3(7.56401f, 5.31907f, -9.87793f));
  points.push_back(Vector3(-2.92032f, -4.62325f, 3.16385f));
  points.push_back(Vector3(1.21006f, 4.51766f, 7.24845f));
  points.push_back(Vector3(-3.87432f, 3.19315f, -3.84136f));
  points.push_back(Vector3(-1.02878f, 2.77505f, -7.48222f));
  points.push_back(Vector3(-3.81573f, 7.42546f, -5.79943f));
  points.push_back(Vector3(0.731529f, 5.51683f, -6.82363f));
  points.push_back(Vector3(8.71273f, 2.87637f, -0.887173f));
  points.push_back(Vector3(1.08737f, -7.48466f, 7.96503f));
  points.push_back(Vector3(-2.94534f, -0.444655f, 4.48714f));
  points.push_back(Vector3(-2.85501f, -7.45293f, 9.01425f));
  points.push_back(Vector3(-6.81631f, 5.18845f, 9.96033f));
  points.push_back(Vector3(0.310984f, 4.19599f, -9.07956f));
  points.push_back(Vector3(-3.97137f, -6.40065f, 5.9801f));
  points.push_back(Vector3(-7.77642f, -2.65053f, -1.64037f));
  points.push_back(Vector3(-7.70806f, -8.764f, 7.28446f));
  points.push_back(Vector3(5.17808f, 9.35118f, 5.36851f));
  points.push_back(Vector3(-4.59273f, 9.9115f, 5.84765f));
  points.push_back(Vector3(9.76745f, 2.61696f, 0.854824f));
  points.push_back(Vector3(-6.06555f, 6.85049f, -4.97055f));
  points.push_back(Vector3(9.54894f, -0.681478f, 8.53328f));
  points.push_back(Vector3(-0.202338f, -4.16608f, 8.57479f));
  points.push_back(Vector3(6.7803f, -4.04096f, 7.80267f));
  points.push_back(Vector3(-8.18598f, -1.901f, 3.45561f));
  points.push_back(Vector3(7.24479f, 8.83419f, -2.39418f));
  points.push_back(Vector3(3.43303f, 1.23386f, -3.14127f));
  points.push_back(Vector3(8.28364f, -7.0336f, -9.53001f));
  points.push_back(Vector3(8.01508f, 1.06479f, 7.31681f));
  points.push_back(Vector3(-4.99374f, -5.34165f, -8.64437f));
  points.push_back(Vector3(-7.1514f, 7.97845f, -9.3292f));
  points.push_back(Vector3(7.63604f, 3.7492f, -2.02918f));
  points.push_back(Vector3(6.86819f, -2.25562f, 5.64745f));
  points.push_back(Vector3(-9.35545f, 8.62423f, 8.49605f));
  points.push_back(Vector3(-0.718101f, -9.76134f, -6.1449f));
  points.push_back(Vector3(4.23566f, 7.32902f, -8.41304f));
  points.push_back(Vector3(-0.521562f, -0.0314343f, -2.48085f));
  points.push_back(Vector3(3.26273f, -5.5327f, -5.32029f));
  points.push_back(Vector3(5.17136f, -7.51335f, 7.6397f));
  points.push_back(Vector3(0.480666f, 8.25983f, -7.58782f));
  points.push_back(Vector3(8.29524f, -6.80654f, -1.84668f));
  points.push_back(Vector3(2.42225f, 7.82769f, 0.0143433f));
  points.push_back(Vector3(-0.863979f, 2.05542f, 0.440382f));
  points.push_back(Vector3(9.86267f, -0.152898f, 9.19492f));
  points.push_back(Vector3(-8.90927f, 2.05908f, 9.75646f));
  points.push_back(Vector3(3.13334f, 8.7817f, -1.05564f));
  points.push_back(Vector3(3.80718f, 3.15531f, -6.67226f));
  points.push_back(Vector3(-1.93335f, 8.34407f, 3.24137f));
  points.push_back(Vector3(-6.26392f, 9.57213f, -8.53938f));
  points.push_back(Vector3(-6.4629f, 6.29627f, -5.59679f));
  points.push_back(Vector3(-9.12107f, 9.20469f, -9.46837f));
  points.push_back(Vector3(8.9581f, 2.50038f, -2.72195f));
  points.push_back(Vector3(-8.86959f, -8.19758f, 2.73904f));
  points.push_back(Vector3(2.15796f, 3.9671f, -1.90466f));
  points.push_back(Vector3(7.77337f, 0.35432f, -5.63951f));
  points.push_back(Vector3(-2.68105f, 2.83731f, 5.12314f));
  points.push_back(Vector3(5.67919f, -0.0363171f, 4.67147f));
  points.push_back(Vector3(-8.39717f, 6.42323f, 8.78109f));
  points.push_back(Vector3(-9.73266f, 3.08756f, -6.09119f));
  points.push_back(Vector3(0.229194f, -4.97665f, 9.05576f));
  points.push_back(Vector3(2.68044f, -8.25678f, -6.55873f));
  points.push_back(Vector3(-4.41755f, -4.23139f, 0.449538f));
  points.push_back(Vector3(5.48448f, -1.23814f, 6.93411f));
  points.push_back(Vector3(8.94345f, 1.72948f, 1.75939f));
  points.push_back(Vector3(-7.99677f, 3.77606f, -9.57396f));
  points.push_back(Vector3(6.12171f, -2.55959f, 2.24891f));
  points.push_back(Vector3(-0.396435f, -2.02918f, 4.26069f));
  points.push_back(Vector3(-6.84133f, -2.67006f, -0.292062f));
  points.push_back(Vector3(-4.57137f, 7.11539f, 3.27494f));
  points.push_back(Vector3(7.84661f, 4.85397f, 2.04749f));
  points.push_back(Vector3(-5.3264f, 7.60186f, -3.88592f));
  points.push_back(Vector3(-0.462356f, -0.162663f, 4.87167f));
  points.push_back(Vector3(9.02951f, -2.40455f, 2.32582f));
  TestSphereRitter(points, file);
}

void SphereRitterTest52(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<Vector3> points;
  points.push_back(Vector3(-9.41465f, 4.71847f, -2.27149f));
  points.push_back(Vector3(-6.75588f, -9.25962f, -6.93411f));
  points.push_back(Vector3(-9.1821f, 5.86596f, -1.66112f));
  points.push_back(Vector3(-9.51598f, 8.11579f, -5.26719f));
  points.push_back(Vector3(1.22166f, -1.88879f, 2.23121f));
  points.push_back(Vector3(3.93475f, -8.65047f, -2.62307f));
  points.push_back(Vector3(6.93106f, 8.00653f, 7.62993f));
  points.push_back(Vector3(-5.96973f, 7.55669f, -6.84133f));
  points.push_back(Vector3(-5.28306f, -5.76464f, -4.76363f));
  points.push_back(Vector3(7.14835f, -4.08429f, -1.63915f));
  points.push_back(Vector3(8.01324f, 8.39839f, 8.31111f));
  points.push_back(Vector3(8.09503f, -7.99493f, -4.99435f));
  points.push_back(Vector3(-1.75512f, -7.50114f, 5.73901f));
  points.push_back(Vector3(4.25459f, 5.28245f, 8.63948f));
  points.push_back(Vector3(0.763268f, 3.26457f, -7.2393f));
  points.push_back(Vector3(2.40455f, 1.68371f, 1.39439f));
  points.push_back(Vector3(-6.91702f, 5.15366f, -1.26926f));
  points.push_back(Vector3(5.02976f, 5.85131f, -8.19819f));
  points.push_back(Vector3(-4.00189f, 5.3795f, -3.01981f));
  points.push_back(Vector3(-4.0202f, 8.46309f, -0.0967437f));
  points.push_back(Vector3(-3.08512f, 7.88751f, -9.07407f));
  points.push_back(Vector3(-2.00415f, 6.07105f, -9.53246f));
  points.push_back(Vector3(-8.01141f, -8.43989f, 5.62731f));
  points.push_back(Vector3(-0.472121f, -3.80657f, 1.08005f));
  points.push_back(Vector3(1.79785f, 3.42753f, 7.45476f));
  points.push_back(Vector3(-2.16895f, 1.70263f, 7.96625f));
  points.push_back(Vector3(-3.09915f, -7.15201f, 8.96603f));
  points.push_back(Vector3(2.08533f, 8.26167f, -6.72475f));
  points.push_back(Vector3(-2.83486f, -9.97742f, 3.91217f));
  points.push_back(Vector3(-3.27677f, 9.2584f, 5.53331f));
  points.push_back(Vector3(4.70931f, 4.26191f, 1.11057f));
  points.push_back(Vector3(-6.198f, 6.74551f, 2.66518f));
  points.push_back(Vector3(-2.75002f, -3.31462f, 8.13349f));
  points.push_back(Vector3(-3.45378f, -3.02774f, 6.89138f));
  points.push_back(Vector3(8.96542f, 0.995818f, -1.07456f));
  points.push_back(Vector3(6.73086f, -2.45094f, -4.64827f));
  points.push_back(Vector3(2.29041f, -2.93374f, 6.07166f));
  points.push_back(Vector3(8.73165f, -7.30644f, -7.91803f));
  points.push_back(Vector3(8.79025f, -2.05725f, -9.02951f));
  points.push_back(Vector3(5.49486f, 5.89221f, 8.40571f));
  points.push_back(Vector3(-7.09769f, 3.83709f, 6.14856f));
  points.push_back(Vector3(-1.2949f, -1.9425f, -7.56279f));
  points.push_back(Vector3(-8.52962f, -0.597858f, -4.1435f));
  points.push_back(Vector3(9.72228f, -3.48247f, 6.72658f));
  points.push_back(Vector3(6.99454f, -4.54695f, 6.45985f));
  points.push_back(Vector3(1.78015f, 8.33125f, 8.5522f));
  points.push_back(Vector3(6.03259f, -4.12946f, -0.608234f));
  points.push_back(Vector3(0.101627f, 3.00943f, -7.39677f));
  points.push_back(Vector3(-1.41209f, -2.81777f, -0.793176f));
  points.push_back(Vector3(7.35771f, 7.2924f, 3.14493f));
  points.push_back(Vector3(-7.84112f, -5.2324f, 9.89196f));
  points.push_back(Vector3(9.95911f, 0.578326f, 4.84115f));
  points.push_back(Vector3(7.93695f, 8.25251f, -3.27067f));
  points.push_back(Vector3(-1.67028f, -3.47514f, -9.32554f));
  points.push_back(Vector3(-1.15329f, 1.98828f, 1.02878f));
  points.push_back(Vector3(-5.79638f, 6.7101f, -5.66393f));
  points.push_back(Vector3(2.30445f, -2.97769f, -6.48549f));
  points.push_back(Vector3(2.55592f, 8.50093f, -4.4084f));
  points.push_back(Vector3(-2.84829f, 1.14841f, 1.59459f));
  points.push_back(Vector3(-2.45521f, 4.02936f, -7.03116f));
  points.push_back(Vector3(-3.22001f, -3.97565f, 8.5577f));
  points.push_back(Vector3(3.27494f, 2.82754f, 1.09104f));
  points.push_back(Vector3(1.42186f, 7.40043f, 1.55431f));
  points.push_back(Vector3(-6.17115f, 9.77233f, 2.69509f));
  points.push_back(Vector3(-5.48692f, -1.23142f, 2.4485f));
  points.push_back(Vector3(5.34471f, -4.96262f, 0.256661f));
  points.push_back(Vector3(3.37321f, 2.69204f, -6.24012f));
  points.push_back(Vector3(-1.60375f, 6.23158f, 1.70263f));
  points.push_back(Vector3(-4.26557f, -9.81811f, -6.43727f));
  points.push_back(Vector3(4.17707f, -2.71401f, -8.54305f));
  points.push_back(Vector3(-6.60146f, 9.35423f, 0.707114f));
  points.push_back(Vector3(5.31053f, -3.4434f, -5.48204f));
  points.push_back(Vector3(-5.06211f, -7.44865f, 7.72698f));
  points.push_back(Vector3(-5.76159f, 0.340892f, 9.87671f));
  points.push_back(Vector3(-6.52577f, 8.8702f, 5.70605f));
  points.push_back(Vector3(-8.64803f, 0.277413f, 2.93252f));
  points.push_back(Vector3(-4.39192f, -1.14841f, 5.73229f));
  points.push_back(Vector3(4.00189f, -3.76202f, -2.94595f));
  points.push_back(Vector3(-3.95734f, 2.72866f, -2.16285f));
  points.push_back(Vector3(-1.35594f, -7.06839f, 4.62264f));
  points.push_back(Vector3(1.85766f, -4.67025f, 3.08267f));
  points.push_back(Vector3(-5.78906f, -9.02585f, 0.30427f));
  points.push_back(Vector3(-6.74856f, 2.2129f, -9.92431f));
  points.push_back(Vector3(-2.21778f, 4.16608f, -3.66192f));
  points.push_back(Vector3(-3.64849f, 5.22691f, 6.33778f));
  points.push_back(Vector3(6.35182f, 6.61183f, 3.89874f));
  points.push_back(Vector3(-9.08689f, 4.01593f, -0.0473034f));
  points.push_back(Vector3(3.35185f, -1.30711f, 8.54305f));
  TestSphereRitter(points, file);
}

void BoundingSphereSpatialPartitionStructure1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mBoundingSphere = Sphere(Vector3(0, 0, 0), 1);
  data[1].mBoundingSphere = Sphere(Vector3(3, 0, 0), 1);
  data[2].mBoundingSphere = Sphere(Vector3(1.5f, 1, 0), 1.5f);
  data[3].mBoundingSphere = Sphere(Vector3(-1, 0, 0), 0.5f);
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());

  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);

  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[1].mBoundingSphere = Sphere(Vector3(1, 2, 3), 1.5);
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

// Medium BoundingSphereSpatialPartition insertion/update/removal test
void BoundingSphereSpatialPartitionStructure2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mBoundingSphere = Sphere(Vector3(-0.5f, -1, -1), 3.5f);
  data[1].mBoundingSphere = Sphere(Vector3(2.25f, -1.75f, 2.25f), 3.75f);
  data[2].mBoundingSphere = Sphere(Vector3(-4.75f, -4.75f, 2.5f), 3.5f);
  data[3].mBoundingSphere = Sphere(Vector3(0, 2.75f, -4.5f), 3.25f);
  data[4].mBoundingSphere = Sphere(Vector3(-0.25f, 4.5f, 0.5f), 0.25f);
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());

  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);

  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 0
  data[0].mBoundingSphere = Sphere(Vector3(-1, -1.5f, -9.75f), 2);
  spatialPartition.UpdateData(keys[0], data[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 4
  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 2
  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 3
  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 1
  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 0
  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

}

// Large BoundingSphereSpatialPartition insertion/update/removal test
void BoundingSphereSpatialPartitionStructure3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(15);
  data[0].mBoundingSphere = Sphere(Vector3(3.75f, 5, 5.25f), 2.25f);
  data[1].mBoundingSphere = Sphere(Vector3(5.25f, 5.5f, 1.75f), 2);
  data[2].mBoundingSphere = Sphere(Vector3(1.25f, 0, -8.5f), 1);
  data[3].mBoundingSphere = Sphere(Vector3(-2.25f, 3.5f, -5.75f), 2.75f);
  data[4].mBoundingSphere = Sphere(Vector3(6.25f, 3.75f, -1.25f), 0.25f);
  data[5].mBoundingSphere = Sphere(Vector3(-2.75f, -6.75f, -3.5f), 1.25f);
  data[6].mBoundingSphere = Sphere(Vector3(4.5f, 5.5f, -1.25f), 1.5f);
  data[7].mBoundingSphere = Sphere(Vector3(-5, 7.5f, 2.75f), 0.5f);
  data[8].mBoundingSphere = Sphere(Vector3(-5.5f, -6.25f, -3.25f), 2.5f);
  data[9].mBoundingSphere = Sphere(Vector3(8.75f, -1, -2.5f), 1.5f);
  data[10].mBoundingSphere = Sphere(Vector3(-4.75f, -4.75f, 4.25f), 2.25f);
  data[11].mBoundingSphere = Sphere(Vector3(-0.5f, -3, -7.5f), 0.75f);
  data[12].mBoundingSphere = Sphere(Vector3(0, -4.5f, 3.25f), 1);
  data[13].mBoundingSphere = Sphere(Vector3(1.75f, 4.75f, 7.5f), 1.25f);
  data[14].mBoundingSphere = Sphere(Vector3(-5, 7.25f, -2.75f), 3);
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());

  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);

  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 2
  data[2].mBoundingSphere = Sphere(Vector3(-4.75f, -1.5f, 5.75f), 1.5f);
  spatialPartition.UpdateData(keys[2], data[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 3
  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 0
  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 13
  data[13].mBoundingSphere = Sphere(Vector3(6.75f, -0.25f, 5.75f), 1.75f);
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 8
  data[8].mBoundingSphere = Sphere(Vector3(-3.25f, 8.5f, 0.5f), 0.5f);
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 7
  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 1
  data[1].mBoundingSphere = Sphere(Vector3(1, 8.25f, -1.75f), 0.5f);
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 12
  data[12].mBoundingSphere = Sphere(Vector3(-2.5f, -2.5f, 2.25f), 0.25f);
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 9
  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 4
  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 10
  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 11
  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 6
  data[6].mBoundingSphere = Sphere(Vector3(-1.75f, 1.5f, -9.25f), 1.75f);
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 14
  data[14].mBoundingSphere = Sphere(Vector3(-5.75f, -2, 5.25f), 1.25f);
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 5
  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 2
  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 13
  data[13].mBoundingSphere = Sphere(Vector3(-7.25f, 1.5f, -4), 0.75f);
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 8
  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 1
  data[1].mBoundingSphere = Sphere(Vector3(5.25f, 5, 0.25f), 0.5f);
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 12
  data[12].mBoundingSphere = Sphere(Vector3(-6.25f, -2.75f, -4), 2.5f);
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 6
  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Update object number 14
  data[14].mBoundingSphere = Sphere(Vector3(-1, 2, -6.75f), 1.75f);
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 13
  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 1
  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 12
  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

  // Remove object number 14
  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true); 

}


void BoundingSphereSpatialPartitionRayCastTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mBoundingSphere = Sphere(Vector3(0, 0, 0), 1); 
  data[1].mBoundingSphere = Sphere(Vector3(3, 0, 0), 1); 
  data[2].mBoundingSphere = Sphere(Vector3(1.5f, 1, 0), 1.5f); 
  data[3].mBoundingSphere = Sphere(Vector3(-1, 0, 0), 0.5f); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  
  Ray ray;
  ray.mStart = Vector3(Vector3(0, 0, 0));
  ray.mDirection = Vector3(Vector3(1, 0, 0));

  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void BoundingSphereSpatialPartitionRayCastTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mBoundingSphere = Sphere(Vector3(-3.25f, 0.75f, -1.75f), 3.5f); 
  data[1].mBoundingSphere = Sphere(Vector3(-4, -8.25f, 1), 3.5f); 
  data[2].mBoundingSphere = Sphere(Vector3(-5.25f, 1.75f, -3.25f), 2.25f); 
  data[3].mBoundingSphere = Sphere(Vector3(-3.5f, -8.5f, -0.75f), 3.5f); 
  data[4].mBoundingSphere = Sphere(Vector3(3, 7.75f, 1.25f), 0.5f); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  
  Ray ray;
  ray.mStart = Vector3(Vector3(-4.25f, -0.5f, -1.25f));
  ray.mDirection = Vector3(Vector3(0.242536f, 0.970143f, 0));

  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void BoundingSphereSpatialPartitionRayCastTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(8);
  data[0].mBoundingSphere = Sphere(Vector3(4.75f, -7, 4.25f), 1.5f); 
  data[1].mBoundingSphere = Sphere(Vector3(-4.25f, -7.25f, 1.5f), 2.25f); 
  data[2].mBoundingSphere = Sphere(Vector3(8, 2.5f, 0), 1.25f); 
  data[3].mBoundingSphere = Sphere(Vector3(-2, 2, 4), 3); 
  data[4].mBoundingSphere = Sphere(Vector3(-2, -0.5f, 1), 3.5f); 
  data[5].mBoundingSphere = Sphere(Vector3(1.5f, 0.5f, -4.75f), 2.75f); 
  data[6].mBoundingSphere = Sphere(Vector3(-1.5f, -3.25f, -4.25f), 3.5f); 
  data[7].mBoundingSphere = Sphere(Vector3(-9, -1, 4.25f), 1.25f); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  
  Ray ray;
  ray.mStart = Vector3(Vector3(-0.5f, -2.25f, 3.5f));
  ray.mDirection = Vector3(Vector3(-0.235702f, 0.942809f, -0.235702f));

  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void BoundingSphereSpatialPartitionFrustumCastTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mBoundingSphere = Sphere(Vector3(-3.5f, 5.75f, -2.25f), 2.25f); 
  data[1].mBoundingSphere = Sphere(Vector3(-3.5f, -1.25f, 2), 2.5f); 
  data[2].mBoundingSphere = Sphere(Vector3(5.25f, -6.5f, 1.5f), 2); 
  data[3].mBoundingSphere = Sphere(Vector3(-4.75f, -7.25f, 0.25f), 2.5f); 
  data[4].mBoundingSphere = Sphere(Vector3(0.75f, -7.75f, 1.5f), 0.75f); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  
  Frustum frustum;
  frustum.Set(Vector3(-2.5724f, -3.36233f, 0.876387f), Vector3(6.1442f, -3.36233f, 0.876387f), Vector3(6.1442f, 5.06641f, 0.876387f), Vector3(-2.5724f, 5.06641f, 0.876387f), Vector3(0.289302f, -0.50063f, -4.34991f), Vector3(3.2825f, -0.50063f, -4.34991f), Vector3(3.2825f, 2.20471f, -4.34991f), Vector3(0.289302f, 2.20471f, -4.34991f));

  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void BoundingSphereSpatialPartitionSelfQuery1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mBoundingSphere = Sphere(Vector3(-0.75f, 1.5f, 3.75f), 2.75f);
  data[1].mBoundingSphere = Sphere(Vector3(0, 1.25f, 0.75f), 1.5f);
  data[2].mBoundingSphere = Sphere(Vector3(-5.75f, -5, 0.5f), 3);
  data[3].mBoundingSphere = Sphere(Vector3(6, -0.75f, 6.75f), 3);
  data[4].mBoundingSphere = Sphere(Vector3(-9.5f, -1.5f, 2), 3.75f);
  
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;
  
  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());
  
  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);
  
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void BoundingSphereSpatialPartitionSelfQuery2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mBoundingSphere = Sphere(Vector3(2, 1.5f, 0.25f), 0.25f);
  data[1].mBoundingSphere = Sphere(Vector3(-2, 7, -6), 0.75f);
  data[2].mBoundingSphere = Sphere(Vector3(6, 3.5f, -6.25f), 0.25f);
  data[3].mBoundingSphere = Sphere(Vector3(-5.75f, -0.75f, 2), 1.5f);
  data[4].mBoundingSphere = Sphere(Vector3(2, -0.5f, -6.5f), 1);
  
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;
  
  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());
  
  BoundingSphereSpatialPartition spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);
  
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionStructureFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(12.1133f, 63.9507f, -45.8527f), 3.8729f); 
  data[1].mBoundingSphere = Sphere(Vector3(-53.2922f, -50.5657f, -17.4317f), 2.88854f); 
  data[2].mBoundingSphere = Sphere(Vector3(-10.0796f, 45.8398f, 46.3538f), 0.213105f); 
  data[3].mBoundingSphere = Sphere(Vector3(-44.1893f, -12.8978f, 9.08879f), 2.05819f); 
  data[4].mBoundingSphere = Sphere(Vector3(-42.5728f, -62.8023f, -32.6821f), 0.282803f); 
  data[5].mBoundingSphere = Sphere(Vector3(-4.16479f, -77.0127f, -48.3367f), 3.75751f); 
  data[6].mBoundingSphere = Sphere(Vector3(12.4219f, -20.9396f, 16.3388f), 0.503262f); 
  data[7].mBoundingSphere = Sphere(Vector3(0.710801f, -21.1439f, -76.9608f), 1.30207f); 
  data[8].mBoundingSphere = Sphere(Vector3(42.4061f, 42.7265f, -31.1818f), 1.96901f); 
  data[9].mBoundingSphere = Sphere(Vector3(-8.79841f, 22.8637f, 94.4291f), 3.58343f); 
  data[10].mBoundingSphere = Sphere(Vector3(-55.0731f, -12.7229f, 77.9166f), 3.87081f); 
  data[11].mBoundingSphere = Sphere(Vector3(33.083f, 4.21261f, 48.4401f), 0.229225f); 
  data[12].mBoundingSphere = Sphere(Vector3(-35.1704f, 74.2576f, -10.2726f), 1.19549f); 
  data[13].mBoundingSphere = Sphere(Vector3(2.30722f, -65.6331f, 73.9283f), 3.24179f); 
  data[14].mBoundingSphere = Sphere(Vector3(37.8938f, 43.4157f, 61.7349f), 1.90047f); 
  data[15].mBoundingSphere = Sphere(Vector3(-41.9322f, -29.8107f, 74.283f), 1.70808f); 
  data[16].mBoundingSphere = Sphere(Vector3(27.3172f, -50.492f, -52.7709f), 2.76225f); 
  data[17].mBoundingSphere = Sphere(Vector3(-70.1172f, 23.7024f, 66.5752f), 0.587341f); 
  data[18].mBoundingSphere = Sphere(Vector3(25.1088f, -36.6376f, -45.403f), 3.70091f); 
  data[19].mBoundingSphere = Sphere(Vector3(2.93311f, 66.9721f, -65.7093f), 2.97331f); 
  data[20].mBoundingSphere = Sphere(Vector3(-8.5462f, 84.4843f, -7.69969f), 3.57938f); 
  data[21].mBoundingSphere = Sphere(Vector3(15.442f, -40.6033f, -53.9992f), 0.429273f); 
  data[22].mBoundingSphere = Sphere(Vector3(-69.9541f, 15.6529f, 29.1443f), 0.277816f); 
  data[23].mBoundingSphere = Sphere(Vector3(-22.5421f, 43.4104f, 61.0101f), 1.80178f); 
  data[24].mBoundingSphere = Sphere(Vector3(-42.672f, 32.3134f, 12.6087f), 2.64268f); 
  data[25].mBoundingSphere = Sphere(Vector3(-52.3366f, 21.3157f, 68.7492f), 1.70576f); 
  data[26].mBoundingSphere = Sphere(Vector3(-77.1917f, -2.93215f, 7.86012f), 1.11072f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[23].mBoundingSphere = Sphere(Vector3(30.5435f, 87.7266f, 11.1788f), 1.35495f); 
  spatialPartition.UpdateData(keys[23], data[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[10].mBoundingSphere = Sphere(Vector3(-22.548f, -73.1778f, 58.104f), 3.38895f); 
  spatialPartition.UpdateData(keys[10], data[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[4].mBoundingSphere = Sphere(Vector3(18.1153f, 85.8719f, 26.7117f), 0.476705f); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[20].mBoundingSphere = Sphere(Vector3(-55.4841f, 44.6007f, 69.4695f), 1.23063f); 
  spatialPartition.UpdateData(keys[20], data[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[13].mBoundingSphere = Sphere(Vector3(20.0387f, 16.0543f, -34.0961f), 0.262856f); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(18); 
  data[0].mBoundingSphere = Sphere(Vector3(-47.4416f, 10.3674f, 77.844f), 3.24805f); 
  data[1].mBoundingSphere = Sphere(Vector3(-49.3584f, 63.5766f, 15.9383f), 2.3603f); 
  data[2].mBoundingSphere = Sphere(Vector3(46.6234f, -61.2208f, 13.0952f), 2.14042f); 
  data[3].mBoundingSphere = Sphere(Vector3(47.861f, 4.58898f, 83.4184f), 1.59779f); 
  data[4].mBoundingSphere = Sphere(Vector3(14.6026f, -9.6707f, -70.4486f), 3.6594f); 
  data[5].mBoundingSphere = Sphere(Vector3(48.4942f, 21.1472f, 29.314f), 0.938615f); 
  data[6].mBoundingSphere = Sphere(Vector3(58.2919f, -4.67668f, -69.0398f), 2.92611f); 
  data[7].mBoundingSphere = Sphere(Vector3(-22.7006f, 59.741f, 76.3843f), 2.12f); 
  data[8].mBoundingSphere = Sphere(Vector3(38.5521f, -73.9811f, 8.09066f), 2.71609f); 
  data[9].mBoundingSphere = Sphere(Vector3(58.2988f, 12.5235f, 11.4229f), 1.77499f); 
  data[10].mBoundingSphere = Sphere(Vector3(-15.7622f, 52.9167f, -64.9862f), 0.886312f); 
  data[11].mBoundingSphere = Sphere(Vector3(-43.4172f, 55.2648f, 33.0697f), 2.84772f); 
  data[12].mBoundingSphere = Sphere(Vector3(-12.9475f, 38.2593f, 37.9882f), 2.97784f); 
  data[13].mBoundingSphere = Sphere(Vector3(-47.1268f, -80.7238f, 5.13397f), 1.78276f); 
  data[14].mBoundingSphere = Sphere(Vector3(22.5951f, -64.301f, 12.1594f), 2.99303f); 
  data[15].mBoundingSphere = Sphere(Vector3(8.98706f, 40.5562f, -24.4105f), 0.890835f); 
  data[16].mBoundingSphere = Sphere(Vector3(53.6477f, -13.8821f, -15.4216f), 0.747612f); 
  data[17].mBoundingSphere = Sphere(Vector3(15.8094f, -31.8427f, -47.7427f), 0.287674f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[14].mBoundingSphere = Sphere(Vector3(-47.5593f, -34.5162f, 61.7309f), 1.74716f); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[12].mBoundingSphere = Sphere(Vector3(53.6717f, 57.1061f, -50.9294f), 3.78348f); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[1].mBoundingSphere = Sphere(Vector3(48.1265f, 45.4456f, 17.7055f), 0.358415f); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(6.84295f, -32.6234f, 55.6119f), 3.30093f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(16); 
  data[0].mBoundingSphere = Sphere(Vector3(40.5899f, 54.4156f, -67.3579f), 1.57344f); 
  data[1].mBoundingSphere = Sphere(Vector3(-44.8686f, 69.0884f, 1.74266f), 1.47393f); 
  data[2].mBoundingSphere = Sphere(Vector3(1.55937f, 47.4698f, -78.5002f), 1.31505f); 
  data[3].mBoundingSphere = Sphere(Vector3(-58.4435f, 56.463f, -22.6317f), 3.30047f); 
  data[4].mBoundingSphere = Sphere(Vector3(-38.9505f, -57.0808f, 21.7441f), 2.44333f); 
  data[5].mBoundingSphere = Sphere(Vector3(-32.0847f, -51.9622f, -18.1122f), 2.86813f); 
  data[6].mBoundingSphere = Sphere(Vector3(-9.81452f, -40.1009f, 89.0731f), 2.47128f); 
  data[7].mBoundingSphere = Sphere(Vector3(-15.7815f, -51.8171f, 54.5488f), 2.58713f); 
  data[8].mBoundingSphere = Sphere(Vector3(-18.4397f, 39.7975f, 6.54929f), 2.40112f); 
  data[9].mBoundingSphere = Sphere(Vector3(-56.6899f, -39.0232f, 37.1882f), 0.555797f); 
  data[10].mBoundingSphere = Sphere(Vector3(-52.4788f, 43.5097f, -70.7809f), 1.69161f); 
  data[11].mBoundingSphere = Sphere(Vector3(16.3911f, -97.4255f, -3.47249f), 0.96668f); 
  data[12].mBoundingSphere = Sphere(Vector3(32.0751f, -19.1715f, 92.4445f), 1.30102f); 
  data[13].mBoundingSphere = Sphere(Vector3(-61.4836f, 44.8021f, -21.026f), 2.18622f); 
  data[14].mBoundingSphere = Sphere(Vector3(-33.799f, -46.9345f, 45.4835f), 3.40565f); 
  data[15].mBoundingSphere = Sphere(Vector3(-41.0205f, 61.2938f, 56.036f), 0.510453f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[0].mBoundingSphere = Sphere(Vector3(42.3077f, 64.1843f, -19.7331f), 1.12544f); 
  spatialPartition.UpdateData(keys[0], data[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mBoundingSphere = Sphere(Vector3(35.4672f, -2.57869f, -42.7753f), 2.62042f); 
  data[1].mBoundingSphere = Sphere(Vector3(29.6398f, -14.4915f, -54.6612f), 3.93193f); 
  data[2].mBoundingSphere = Sphere(Vector3(9.62282f, 26.6952f, 94.9523f), 3.36727f); 
  data[3].mBoundingSphere = Sphere(Vector3(54.3364f, -4.10775f, -64.4801f), 3.90027f); 
  data[4].mBoundingSphere = Sphere(Vector3(4.00953f, -45.0185f, 32.2976f), 1.89247f); 
  data[5].mBoundingSphere = Sphere(Vector3(25.3089f, -84.759f, -29.5652f), 0.232704f); 
  data[6].mBoundingSphere = Sphere(Vector3(-6.09073f, -15.6927f, 0.924554f), 2.89898f); 
  data[7].mBoundingSphere = Sphere(Vector3(41.2291f, -28.8786f, 61.9414f), 1.9871f); 
  data[8].mBoundingSphere = Sphere(Vector3(6.80437f, -53.2129f, -27.0873f), 0.550694f); 
  data[9].mBoundingSphere = Sphere(Vector3(54.723f, 46.8689f, -55.8431f), 2.4947f); 
  data[10].mBoundingSphere = Sphere(Vector3(25.0815f, -15.4852f, -34.2932f), 1.46651f); 
  data[11].mBoundingSphere = Sphere(Vector3(72.5256f, 63.5528f, -11.2861f), 2.59061f); 
  data[12].mBoundingSphere = Sphere(Vector3(32.2118f, 18.7123f, -31.5096f), 2.58702f); 
  data[13].mBoundingSphere = Sphere(Vector3(-3.34265f, -32.9179f, -33.3609f), 3.91986f); 
  data[14].mBoundingSphere = Sphere(Vector3(40.8796f, -39.9439f, 28.2643f), 3.71854f); 
  data[15].mBoundingSphere = Sphere(Vector3(18.4927f, -44.2772f, 47.5364f), 2.71667f); 
  data[16].mBoundingSphere = Sphere(Vector3(-31.0584f, -90.8003f, -5.91844f), 1.55036f); 
  data[17].mBoundingSphere = Sphere(Vector3(-2.95501f, -12.7605f, 44.3828f), 3.06145f); 
  data[18].mBoundingSphere = Sphere(Vector3(-21.0998f, -12.2274f, 62.615f), 3.14843f); 
  data[19].mBoundingSphere = Sphere(Vector3(-79.089f, 27.0059f, 53.7081f), 0.209742f); 
  data[20].mBoundingSphere = Sphere(Vector3(-36.5085f, -47.4558f, -75.4186f), 1.92239f); 
  data[21].mBoundingSphere = Sphere(Vector3(-62.1315f, 29.899f, -17.9684f), 2.39903f); 
  data[22].mBoundingSphere = Sphere(Vector3(53.1265f, 59.1754f, 56.2277f), 3.46955f); 
  data[23].mBoundingSphere = Sphere(Vector3(14.7125f, -19.5355f, -48.0246f), 3.72817f); 
  data[24].mBoundingSphere = Sphere(Vector3(64.4392f, 66.8982f, 32.1211f), 2.49204f); 
  data[25].mBoundingSphere = Sphere(Vector3(-4.76271f, -15.2489f, 0.717901f), 3.11341f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[19].mBoundingSphere = Sphere(Vector3(-24.3897f, -27.8628f, 68.1669f), 2.53066f); 
  spatialPartition.UpdateData(keys[19], data[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[23].mBoundingSphere = Sphere(Vector3(33.5355f, -63.5788f, 66.3506f), 3.71402f); 
  spatialPartition.UpdateData(keys[23], data[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(23); 
  data[0].mBoundingSphere = Sphere(Vector3(52.9097f, 4.41851f, 23.7314f), 1.57147f); 
  data[1].mBoundingSphere = Sphere(Vector3(-5.52325f, 87.7682f, 5.97457f), 2.3066f); 
  data[2].mBoundingSphere = Sphere(Vector3(-19.3775f, 44.8283f, 82.276f), 0.994977f); 
  data[3].mBoundingSphere = Sphere(Vector3(-44.9472f, 79.1364f, -19.423f), 1.60394f); 
  data[4].mBoundingSphere = Sphere(Vector3(24.9504f, 20.4572f, 60.6192f), 2.69522f); 
  data[5].mBoundingSphere = Sphere(Vector3(-3.07111f, -15.4429f, 23.5205f), 0.201044f); 
  data[6].mBoundingSphere = Sphere(Vector3(23.1017f, 34.916f, 36.7408f), 1.40076f); 
  data[7].mBoundingSphere = Sphere(Vector3(-73.4927f, 42.1013f, 39.2355f), 3.66392f); 
  data[8].mBoundingSphere = Sphere(Vector3(8.36574f, -6.70762f, -86.3066f), 3.63237f); 
  data[9].mBoundingSphere = Sphere(Vector3(-97.4947f, -9.13099f, 14.8067f), 1.80723f); 
  data[10].mBoundingSphere = Sphere(Vector3(-31.4174f, -7.54416f, 26.6189f), 3.61104f); 
  data[11].mBoundingSphere = Sphere(Vector3(-55.9013f, 63.3942f, -12.9833f), 0.891879f); 
  data[12].mBoundingSphere = Sphere(Vector3(-28.6099f, 45.4333f, 46.248f), 3.3421f); 
  data[13].mBoundingSphere = Sphere(Vector3(38.3921f, -32.8819f, 52.1904f), 1.19491f); 
  data[14].mBoundingSphere = Sphere(Vector3(80.1034f, -35.1142f, 29.9738f), 0.87344f); 
  data[15].mBoundingSphere = Sphere(Vector3(80.9883f, -3.22412f, -23.9154f), 3.5563f); 
  data[16].mBoundingSphere = Sphere(Vector3(47.9814f, -80.9883f, 7.73382f), 3.73002f); 
  data[17].mBoundingSphere = Sphere(Vector3(34.616f, 41.4879f, -74.8263f), 3.09937f); 
  data[18].mBoundingSphere = Sphere(Vector3(45.6941f, 13.4559f, -10.74f), 2.92055f); 
  data[19].mBoundingSphere = Sphere(Vector3(44.3444f, -0.514472f, -11.4046f), 0.527848f); 
  data[20].mBoundingSphere = Sphere(Vector3(21.909f, -70.8864f, 15.467f), 2.52323f); 
  data[21].mBoundingSphere = Sphere(Vector3(-1.33976f, 15.3593f, 7.16351f), 0.596039f); 
  data[22].mBoundingSphere = Sphere(Vector3(-31.2521f, 18.3275f, -6.32471f), 2.24305f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[9].mBoundingSphere = Sphere(Vector3(24.4967f, -73.055f, 58.8415f), 3.19841f); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[22].mBoundingSphere = Sphere(Vector3(40.3076f, 12.8932f, -77.9522f), 3.54099f); 
  spatialPartition.UpdateData(keys[22], data[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[18].mBoundingSphere = Sphere(Vector3(-74.7388f, -35.1223f, -10.5103f), 3.23587f); 
  spatialPartition.UpdateData(keys[18], data[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(20); 
  data[0].mBoundingSphere = Sphere(Vector3(-5.62074f, 26.7074f, -50.6438f), 3.98342f); 
  data[1].mBoundingSphere = Sphere(Vector3(-55.8845f, 61.7573f, 6.0682f), 3.20526f); 
  data[2].mBoundingSphere = Sphere(Vector3(25.8397f, -60.0785f, -10.5389f), 3.11596f); 
  data[3].mBoundingSphere = Sphere(Vector3(-15.7535f, 74.9967f, 28.3843f), 3.35115f); 
  data[4].mBoundingSphere = Sphere(Vector3(-69.6073f, 24.321f, 30.609f), 3.49541f); 
  data[5].mBoundingSphere = Sphere(Vector3(27.8454f, 57.4899f, -6.07652f), 1.49632f); 
  data[6].mBoundingSphere = Sphere(Vector3(-6.38781f, -17.0665f, -86.7406f), 3.90177f); 
  data[7].mBoundingSphere = Sphere(Vector3(-2.26219f, -5.26943f, 31.495f), 1.96333f); 
  data[8].mBoundingSphere = Sphere(Vector3(59.9334f, -2.5471f, -35.0003f), 2.55153f); 
  data[9].mBoundingSphere = Sphere(Vector3(34.1257f, -76.5772f, -31.301f), 3.32343f); 
  data[10].mBoundingSphere = Sphere(Vector3(48.584f, 8.32966f, 66.8236f), 1.09042f); 
  data[11].mBoundingSphere = Sphere(Vector3(-49.1283f, -32.272f, 60.3811f), 2.62656f); 
  data[12].mBoundingSphere = Sphere(Vector3(-34.5657f, -51.8421f, -72.918f), 1.46199f); 
  data[13].mBoundingSphere = Sphere(Vector3(-11.502f, 39.0521f, 85.5702f), 3.90746f); 
  data[14].mBoundingSphere = Sphere(Vector3(-51.6842f, 2.51929f, 47.7166f), 3.24956f); 
  data[15].mBoundingSphere = Sphere(Vector3(-48.9007f, 67.2775f, 21.6967f), 0.565191f); 
  data[16].mBoundingSphere = Sphere(Vector3(33.8892f, 20.1237f, -73.3396f), 1.49481f); 
  data[17].mBoundingSphere = Sphere(Vector3(41.6169f, -81.6884f, -19.2848f), 3.82025f); 
  data[18].mBoundingSphere = Sphere(Vector3(-2.35584f, -52.0779f, -32.0856f), 3.99107f); 
  data[19].mBoundingSphere = Sphere(Vector3(-54.139f, -22.9262f, -37.8801f), 3.24283f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[0].mBoundingSphere = Sphere(Vector3(-78.0365f, 32.6584f, -10.6609f), 1.41444f); 
  spatialPartition.UpdateData(keys[0], data[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[5].mBoundingSphere = Sphere(Vector3(44.2705f, 76.8261f, 40.7136f), 2.43614f); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[9].mBoundingSphere = Sphere(Vector3(-18.9982f, -77.1119f, 45.7754f), 1.56323f); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[1].mBoundingSphere = Sphere(Vector3(39.7964f, 21.1499f, 61.3727f), 3.24202f); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(22); 
  data[0].mBoundingSphere = Sphere(Vector3(-1.99086f, 4.22225f, -57.3524f), 2.9732f); 
  data[1].mBoundingSphere = Sphere(Vector3(12.8368f, -10.2279f, -31.2492f), 0.307273f); 
  data[2].mBoundingSphere = Sphere(Vector3(29.2385f, 66.0527f, -10.5308f), 0.547795f); 
  data[3].mBoundingSphere = Sphere(Vector3(-75.8088f, -51.395f, 37.0752f), 3.37678f); 
  data[4].mBoundingSphere = Sphere(Vector3(-30.3851f, 29.0416f, 32.7488f), 2.14239f); 
  data[5].mBoundingSphere = Sphere(Vector3(-16.9199f, 82.3531f, 25.9413f), 1.6799f); 
  data[6].mBoundingSphere = Sphere(Vector3(-29.2954f, 82.1828f, -15.181f), 0.662142f); 
  data[7].mBoundingSphere = Sphere(Vector3(-57.0865f, 11.0146f, -72.2851f), 0.350877f); 
  data[8].mBoundingSphere = Sphere(Vector3(64.728f, 10.3833f, 1.32671f), 0.915885f); 
  data[9].mBoundingSphere = Sphere(Vector3(48.0701f, -9.02177f, 20.3491f), 1.59106f); 
  data[10].mBoundingSphere = Sphere(Vector3(-19.6983f, 41.9851f, 15.5935f), 3.83683f); 
  data[11].mBoundingSphere = Sphere(Vector3(33.8776f, 2.2216f, -53.0229f), 3.37678f); 
  data[12].mBoundingSphere = Sphere(Vector3(41.9664f, -52.3251f, 24.4776f), 0.678146f); 
  data[13].mBoundingSphere = Sphere(Vector3(-84.4281f, 18.5179f, -41.798f), 0.543852f); 
  data[14].mBoundingSphere = Sphere(Vector3(-49.4007f, 0.0663061f, -44.1647f), 3.78545f); 
  data[15].mBoundingSphere = Sphere(Vector3(13.9726f, 92.2119f, 3.23819f), 3.44589f); 
  data[16].mBoundingSphere = Sphere(Vector3(-10.8746f, -67.853f, 43.7484f), 3.74115f); 
  data[17].mBoundingSphere = Sphere(Vector3(-30.9735f, -70.0994f, 34.7854f), 1.98768f); 
  data[18].mBoundingSphere = Sphere(Vector3(19.0127f, 39.7579f, 67.5165f), 1.08694f); 
  data[19].mBoundingSphere = Sphere(Vector3(-45.7727f, 66.2296f, 18.0673f), 0.434028f); 
  data[20].mBoundingSphere = Sphere(Vector3(-55.092f, 72.9339f, -35.3278f), 2.41956f); 
  data[21].mBoundingSphere = Sphere(Vector3(-84.5817f, -42.6984f, 14.4884f), 1.39345f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[8].mBoundingSphere = Sphere(Vector3(-54.7632f, 18.9109f, 58.2238f), 1.13113f); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(57.8544f, 24.7419f, -75.3518f), 2.24931f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[13].mBoundingSphere = Sphere(Vector3(-6.24151f, 37.2949f, 65.476f), 1.91462f); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[18].mBoundingSphere = Sphere(Vector3(39.3303f, -44.0797f, -52.3295f), 0.906259f); 
  spatialPartition.UpdateData(keys[18], data[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[3].mBoundingSphere = Sphere(Vector3(66.5259f, -39.0564f, 28.4021f), 3.61092f); 
  spatialPartition.UpdateData(keys[3], data[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[20].mBoundingSphere = Sphere(Vector3(-18.8241f, -60.4734f, 28.8502f), 1.28131f); 
  spatialPartition.UpdateData(keys[20], data[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(16); 
  data[0].mBoundingSphere = Sphere(Vector3(19.993f, 1.34959f, 96.6747f), 0.206262f); 
  data[1].mBoundingSphere = Sphere(Vector3(21.6287f, 5.26167f, 61.2954f), 3.17302f); 
  data[2].mBoundingSphere = Sphere(Vector3(0.35969f, -30.624f, -18.1434f), 0.943834f); 
  data[3].mBoundingSphere = Sphere(Vector3(-79.2035f, 17.8182f, 25.7907f), 1.5013f); 
  data[4].mBoundingSphere = Sphere(Vector3(-33.3125f, 19.373f, -19.9209f), 2.28758f); 
  data[5].mBoundingSphere = Sphere(Vector3(72.9668f, 52.5829f, -25.744f), 0.255434f); 
  data[6].mBoundingSphere = Sphere(Vector3(10.3448f, 86.9924f, -14.6013f), 2.19179f); 
  data[7].mBoundingSphere = Sphere(Vector3(3.3507f, -27.1124f, 28.2804f), 0.28547f); 
  data[8].mBoundingSphere = Sphere(Vector3(36.0093f, -30.7455f, -6.9685f), 1.40748f); 
  data[9].mBoundingSphere = Sphere(Vector3(-28.8561f, 76.8711f, 53.598f), 2.87068f); 
  data[10].mBoundingSphere = Sphere(Vector3(69.9019f, -49.3428f, -36.3055f), 3.14089f); 
  data[11].mBoundingSphere = Sphere(Vector3(71.8455f, -11.9635f, 54.292f), 1.06514f); 
  data[12].mBoundingSphere = Sphere(Vector3(43.4889f, 34.433f, 47.8272f), 3.68734f); 
  data[13].mBoundingSphere = Sphere(Vector3(94.3713f, 6.84321f, 2.45243f), 3.81955f); 
  data[14].mBoundingSphere = Sphere(Vector3(-4.98648f, 77.8102f, 27.7845f), 0.638948f); 
  data[15].mBoundingSphere = Sphere(Vector3(-19.2258f, 41.4732f, 37.1726f), 1.67677f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[5].mBoundingSphere = Sphere(Vector3(-11.3368f, -34.5214f, -5.12339f), 2.30776f); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[13].mBoundingSphere = Sphere(Vector3(33.261f, -51.2905f, 72.9659f), 2.44194f); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(12.9502f, 92.6491f, -23.3494f), 3.30116f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[8].mBoundingSphere = Sphere(Vector3(-2.88226f, 75.2125f, 57.5241f), 1.50884f); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[7].mBoundingSphere = Sphere(Vector3(-27.8756f, 1.9005f, 33.3044f), 3.38234f); 
  spatialPartition.UpdateData(keys[7], data[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[1].mBoundingSphere = Sphere(Vector3(-15.2217f, -19.0874f, -80.9395f), 0.671651f); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mBoundingSphere = Sphere(Vector3(-17.4183f, 55.2102f, -74.7578f), 3.85631f); 
  data[1].mBoundingSphere = Sphere(Vector3(6.36882f, 1.24214f, -62.7131f), 0.904984f); 
  data[2].mBoundingSphere = Sphere(Vector3(54.2855f, 25.0231f, -13.6269f), 3.84646f); 
  data[3].mBoundingSphere = Sphere(Vector3(-28.1621f, -77.2659f, -43.7043f), 0.493057f); 
  data[4].mBoundingSphere = Sphere(Vector3(38.7744f, -32.8628f, 27.378f), 0.999268f); 
  data[5].mBoundingSphere = Sphere(Vector3(6.68936f, -28.5499f, -5.90913f), 2.72409f); 
  data[6].mBoundingSphere = Sphere(Vector3(20.5843f, 85.7046f, -21.9996f), 3.67099f); 
  data[7].mBoundingSphere = Sphere(Vector3(77.8534f, 28.8804f, -17.6241f), 2.39799f); 
  data[8].mBoundingSphere = Sphere(Vector3(15.8016f, -52.9563f, 45.9048f), 3.99003f); 
  data[9].mBoundingSphere = Sphere(Vector3(23.6607f, -11.4f, -52.5214f), 0.390307f); 
  data[10].mBoundingSphere = Sphere(Vector3(-61.0909f, -45.5829f, -56.4127f), 1.68175f); 
  data[11].mBoundingSphere = Sphere(Vector3(12.6085f, 80.599f, -13.9241f), 3.01379f); 
  data[12].mBoundingSphere = Sphere(Vector3(-38.9778f, 64.2503f, 56.124f), 1.76757f); 
  data[13].mBoundingSphere = Sphere(Vector3(-69.5347f, -6.56617f, -26.3885f), 2.85433f); 
  data[14].mBoundingSphere = Sphere(Vector3(-71.2663f, -59.7116f, 23.119f), 1.33883f); 
  data[15].mBoundingSphere = Sphere(Vector3(44.7575f, 26.2534f, -79.1478f), 1.88377f); 
  data[16].mBoundingSphere = Sphere(Vector3(-7.99601f, -49.2271f, -47.0398f), 3.4067f); 
  data[17].mBoundingSphere = Sphere(Vector3(-64.1037f, -14.861f, -34.4654f), 0.845723f); 
  data[18].mBoundingSphere = Sphere(Vector3(62.3449f, 32.1887f, -42.7563f), 0.825544f); 
  data[19].mBoundingSphere = Sphere(Vector3(87.9021f, 6.61135f, 28.4651f), 0.517991f); 
  data[20].mBoundingSphere = Sphere(Vector3(4.68502f, 14.0433f, -39.7336f), 2.49273f); 
  data[21].mBoundingSphere = Sphere(Vector3(45.4152f, 11.4365f, -26.0782f), 3.44879f); 
  data[22].mBoundingSphere = Sphere(Vector3(-29.2903f, 15.8739f, -35.0965f), 3.02295f); 
  data[23].mBoundingSphere = Sphere(Vector3(34.1647f, 1.06492f, 1.19933f), 0.968303f); 
  data[24].mBoundingSphere = Sphere(Vector3(40.6791f, 56.0001f, 65.9367f), 2.29663f); 
  data[25].mBoundingSphere = Sphere(Vector3(-1.52252f, 97.1166f, 19.5731f), 2.4765f); 
  data[26].mBoundingSphere = Sphere(Vector3(15.678f, -73.481f, -2.06219f), 2.68513f); 
  data[27].mBoundingSphere = Sphere(Vector3(22.5606f, -63.1173f, 53.5007f), 0.603229f); 
  data[28].mBoundingSphere = Sphere(Vector3(1.18472f, 30.2336f, 49.8095f), 2.94954f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[17].mBoundingSphere = Sphere(Vector3(-54.2611f, -1.31125f, 15.6592f), 0.791913f); 
  spatialPartition.UpdateData(keys[17], data[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[23].mBoundingSphere = Sphere(Vector3(-47.4396f, -14.0837f, -50.6654f), 3.81862f); 
  spatialPartition.UpdateData(keys[23], data[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[28]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[10].mBoundingSphere = Sphere(Vector3(13.9987f, 63.4027f, 12.3924f), 2.22113f); 
  spatialPartition.UpdateData(keys[10], data[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(-60.4203f, -26.3962f, 54.4252f), 3.0255f); 
  data[1].mBoundingSphere = Sphere(Vector3(-29.0418f, 46.0212f, -30.793f), 1.30694f); 
  data[2].mBoundingSphere = Sphere(Vector3(57.8032f, 24.9432f, 60.0135f), 2.61833f); 
  data[3].mBoundingSphere = Sphere(Vector3(-4.2521f, -14.7707f, 11.6466f), 2.49563f); 
  data[4].mBoundingSphere = Sphere(Vector3(14.983f, -14.1973f, 50.7554f), 1.87322f); 
  data[5].mBoundingSphere = Sphere(Vector3(32.6288f, -51.6065f, 57.6372f), 2.07466f); 
  data[6].mBoundingSphere = Sphere(Vector3(-19.1797f, 7.24697f, -72.0874f), 3.7589f); 
  data[7].mBoundingSphere = Sphere(Vector3(47.3627f, 1.73998f, 57.132f), 3.292f); 
  data[8].mBoundingSphere = Sphere(Vector3(-88.3799f, 17.1756f, -37.6688f), 0.91113f); 
  data[9].mBoundingSphere = Sphere(Vector3(3.62924f, -70.758f, -49.9286f), 2.7517f); 
  data[10].mBoundingSphere = Sphere(Vector3(1.75391f, -38.2039f, 10.766f), 0.704239f); 
  data[11].mBoundingSphere = Sphere(Vector3(23.9811f, -66.0111f, -37.0135f), 3.37237f); 
  data[12].mBoundingSphere = Sphere(Vector3(19.4955f, 16.3632f, -90.5689f), 3.29722f); 
  data[13].mBoundingSphere = Sphere(Vector3(73.5245f, -0.317218f, 23.5343f), 3.12094f); 
  data[14].mBoundingSphere = Sphere(Vector3(-12.2621f, 9.54718f, 90.9609f), 1.05447f); 
  data[15].mBoundingSphere = Sphere(Vector3(-75.2379f, 25.9167f, 48.7187f), 1.32109f); 
  data[16].mBoundingSphere = Sphere(Vector3(-7.60142f, -26.9884f, 3.26411f), 3.1424f); 
  data[17].mBoundingSphere = Sphere(Vector3(0.102372f, -19.5917f, -55.0301f), 3.53705f); 
  data[18].mBoundingSphere = Sphere(Vector3(-26.1592f, -55.177f, 34.0136f), 1.69996f); 
  data[19].mBoundingSphere = Sphere(Vector3(-36.7017f, 39.2684f, 43.5538f), 2.23168f); 
  data[20].mBoundingSphere = Sphere(Vector3(53.7048f, -34.553f, 47.9591f), 1.18528f); 
  data[21].mBoundingSphere = Sphere(Vector3(42.84f, 34.8238f, 75.4436f), 1.60092f); 
  data[22].mBoundingSphere = Sphere(Vector3(-22.729f, -0.16126f, -83.8247f), 0.696237f); 
  data[23].mBoundingSphere = Sphere(Vector3(-5.37618f, -47.5816f, 13.4699f), 2.98143f); 
  data[24].mBoundingSphere = Sphere(Vector3(0.0294333f, -68.2313f, -62.3562f), 1.46709f); 
  data[25].mBoundingSphere = Sphere(Vector3(-24.5756f, -78.4728f, 27.779f), 0.235023f); 
  data[26].mBoundingSphere = Sphere(Vector3(70.6269f, -28.8518f, 7.58334f), 1.53923f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[26].mBoundingSphere = Sphere(Vector3(-72.1325f, -25.7011f, 47.2132f), 0.459194f); 
  spatialPartition.UpdateData(keys[26], data[26]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[12].mBoundingSphere = Sphere(Vector3(56.9109f, -50.7447f, 57.8028f), 2.2398f); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[14].mBoundingSphere = Sphere(Vector3(35.8476f, -38.1854f, 69.1113f), 2.50213f); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[9].mBoundingSphere = Sphere(Vector3(-28.2185f, -29.0881f, 37.9882f), 3.21395f); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[15].mBoundingSphere = Sphere(Vector3(-69.9196f, 50.4582f, -19.7362f), 0.281063f); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[16].mBoundingSphere = Sphere(Vector3(-28.4558f, -40.1281f, -31.1084f), 1.43381f); 
  spatialPartition.UpdateData(keys[16], data[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(-44.5833f, -18.5177f, 60.7602f), 2.28294f); 
  data[1].mBoundingSphere = Sphere(Vector3(53.7581f, -24.8174f, 60.4416f), 1.80051f); 
  data[2].mBoundingSphere = Sphere(Vector3(-78.7823f, 7.88167f, -42.4286f), 1.15014f); 
  data[3].mBoundingSphere = Sphere(Vector3(56.5499f, -29.3618f, -30.898f), 3.19343f); 
  data[4].mBoundingSphere = Sphere(Vector3(-6.25063f, 50.6172f, 17.9477f), 2.57577f); 
  data[5].mBoundingSphere = Sphere(Vector3(54.0546f, 1.1559f, 80.5934f), 3.54157f); 
  data[6].mBoundingSphere = Sphere(Vector3(-26.2569f, 29.3765f, 18.8462f), 1.83425f); 
  data[7].mBoundingSphere = Sphere(Vector3(42.5229f, -72.2796f, -31.6466f), 2.52636f); 
  data[8].mBoundingSphere = Sphere(Vector3(-4.30725f, 10.2824f, -25.4669f), 3.12187f); 
  data[9].mBoundingSphere = Sphere(Vector3(60.2874f, 27.0641f, -6.26585f), 2.37572f); 
  data[10].mBoundingSphere = Sphere(Vector3(77.092f, -33.9528f, 9.10976f), 2.17428f); 
  data[11].mBoundingSphere = Sphere(Vector3(52.2917f, -83.0936f, -7.57601f), 2.11919f); 
  data[12].mBoundingSphere = Sphere(Vector3(21.646f, 30.6556f, 33.9175f), 3.55989f); 
  data[13].mBoundingSphere = Sphere(Vector3(44.3862f, -29.0535f, 20.0708f), 3.71808f); 
  data[14].mBoundingSphere = Sphere(Vector3(-75.7425f, 34.0632f, 29.0324f), 0.447249f); 
  data[15].mBoundingSphere = Sphere(Vector3(-62.9998f, 45.6116f, 29.5253f), 3.47164f); 
  data[16].mBoundingSphere = Sphere(Vector3(54.2738f, -75.8102f, 31.6035f), 0.448524f); 
  data[17].mBoundingSphere = Sphere(Vector3(25.8727f, 72.5591f, -0.063474f), 2.89701f); 
  data[18].mBoundingSphere = Sphere(Vector3(5.36868f, -36.5172f, -78.1543f), 2.53738f); 
  data[19].mBoundingSphere = Sphere(Vector3(13.8688f, -6.88647f, 42.7343f), 1.08474f); 
  data[20].mBoundingSphere = Sphere(Vector3(26.9602f, 73.7267f, 8.88469f), 1.04148f); 
  data[21].mBoundingSphere = Sphere(Vector3(-46.4414f, -38.5493f, -55.1224f), 2.72978f); 
  data[22].mBoundingSphere = Sphere(Vector3(-53.2814f, -81.052f, -12.2784f), 1.51128f); 
  data[23].mBoundingSphere = Sphere(Vector3(11.4536f, -8.29068f, 49.8394f), 3.8359f); 
  data[24].mBoundingSphere = Sphere(Vector3(35.3767f, -62.1889f, 16.0845f), 2.86222f); 
  data[25].mBoundingSphere = Sphere(Vector3(55.3288f, -11.6969f, 60.0223f), 0.697049f); 
  data[26].mBoundingSphere = Sphere(Vector3(-8.58013f, -28.3868f, 49.5688f), 2.79321f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[4].mBoundingSphere = Sphere(Vector3(35.4964f, 84.2613f, 34.3204f), 2.96496f); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[23].mBoundingSphere = Sphere(Vector3(-14.897f, 2.74119f, 40.7194f), 2.83531f); 
  spatialPartition.UpdateData(keys[23], data[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[12].mBoundingSphere = Sphere(Vector3(8.34954f, -49.9204f, 57.3148f), 0.665273f); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(20); 
  data[0].mBoundingSphere = Sphere(Vector3(-20.4865f, -6.86404f, -96.9868f), 2.19782f); 
  data[1].mBoundingSphere = Sphere(Vector3(59.5144f, -73.8847f, 3.20123f), 2.20524f); 
  data[2].mBoundingSphere = Sphere(Vector3(55.1999f, -80.5451f, 1.36795f), 3.45366f); 
  data[3].mBoundingSphere = Sphere(Vector3(-63.8799f, 15.7934f, -50.6688f), 1.78589f); 
  data[4].mBoundingSphere = Sphere(Vector3(8.02338f, -63.8157f, 23.7562f), 2.06306f); 
  data[5].mBoundingSphere = Sphere(Vector3(62.4354f, -41.4173f, -4.24839f), 3.97263f); 
  data[6].mBoundingSphere = Sphere(Vector3(17.7457f, -66.1285f, -52.7965f), 1.97493f); 
  data[7].mBoundingSphere = Sphere(Vector3(-69.9069f, 63.6073f, 28.6219f), 1.88238f); 
  data[8].mBoundingSphere = Sphere(Vector3(0.293635f, -40.0338f, -13.5675f), 1.46454f); 
  data[9].mBoundingSphere = Sphere(Vector3(4.75789f, -53.6538f, 46.1113f), 2.17358f); 
  data[10].mBoundingSphere = Sphere(Vector3(-33.465f, -43.8117f, 23.8085f), 3.18287f); 
  data[11].mBoundingSphere = Sphere(Vector3(15.841f, 64.9128f, 35.8301f), 3.03327f); 
  data[12].mBoundingSphere = Sphere(Vector3(32.1731f, 8.84735f, -71.3685f), 1.88064f); 
  data[13].mBoundingSphere = Sphere(Vector3(-55.3109f, 43.6123f, 20.7684f), 3.03849f); 
  data[14].mBoundingSphere = Sphere(Vector3(36.8774f, 30.414f, -16.5821f), 3.00799f); 
  data[15].mBoundingSphere = Sphere(Vector3(-23.2006f, 40.6845f, -10.2095f), 2.89295f); 
  data[16].mBoundingSphere = Sphere(Vector3(-13.843f, 73.1973f, 56.7163f), 3.78812f); 
  data[17].mBoundingSphere = Sphere(Vector3(29.246f, -27.894f, 9.24273f), 3.83927f); 
  data[18].mBoundingSphere = Sphere(Vector3(-78.1979f, -16.7119f, -56.889f), 1.36655f); 
  data[19].mBoundingSphere = Sphere(Vector3(9.80959f, -25.3969f, 94.7624f), 2.74509f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[9].mBoundingSphere = Sphere(Vector3(54.1233f, 15.3422f, 49.4654f), 0.735203f); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[7].mBoundingSphere = Sphere(Vector3(-35.217f, -22.5866f, -70.2544f), 3.31995f); 
  spatialPartition.UpdateData(keys[7], data[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[4].mBoundingSphere = Sphere(Vector3(1.52183f, 41.6971f, -13.4204f), 1.95741f); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(-6.55626f, -48.0813f, -3.86713f), 2.86871f); 
  data[1].mBoundingSphere = Sphere(Vector3(-69.7673f, -34.9182f, -55.9375f), 0.635933f); 
  data[2].mBoundingSphere = Sphere(Vector3(10.2787f, -18.0931f, 47.4186f), 2.85676f); 
  data[3].mBoundingSphere = Sphere(Vector3(57.9294f, -61.5888f, -23.5879f), 3.37132f); 
  data[4].mBoundingSphere = Sphere(Vector3(-15.4068f, -22.8407f, 69.8406f), 1.7823f); 
  data[5].mBoundingSphere = Sphere(Vector3(-8.04801f, 84.2345f, 37.6929f), 3.84576f); 
  data[6].mBoundingSphere = Sphere(Vector3(-51.1644f, 4.0604f, -61.4633f), 1.76943f); 
  data[7].mBoundingSphere = Sphere(Vector3(15.9218f, 32.6486f, 54.9526f), 2.9398f); 
  data[8].mBoundingSphere = Sphere(Vector3(-56.3276f, -7.83752f, -27.0513f), 2.69568f); 
  data[9].mBoundingSphere = Sphere(Vector3(53.2282f, -61.1976f, 22.0029f), 3.86652f); 
  data[10].mBoundingSphere = Sphere(Vector3(-0.966981f, 41.4123f, -48.092f), 1.49481f); 
  data[11].mBoundingSphere = Sphere(Vector3(-29.3571f, 29.881f, 23.6994f), 1.26159f); 
  data[12].mBoundingSphere = Sphere(Vector3(66.5777f, -1.6025f, -47.84f), 1.43334f); 
  data[13].mBoundingSphere = Sphere(Vector3(-62.8849f, -69.3596f, -16.356f), 3.04835f); 
  data[14].mBoundingSphere = Sphere(Vector3(-46.5244f, -7.99949f, 46.5747f), 0.963433f); 
  data[15].mBoundingSphere = Sphere(Vector3(-36.7741f, -51.1692f, 17.2955f), 1.4773f); 
  data[16].mBoundingSphere = Sphere(Vector3(53.6558f, -66.7948f, -2.60285f), 3.95048f); 
  data[17].mBoundingSphere = Sphere(Vector3(33.0138f, -44.2766f, -3.36549f), 3.54111f); 
  data[18].mBoundingSphere = Sphere(Vector3(56.2989f, 57.9336f, 37.389f), 1.87426f); 
  data[19].mBoundingSphere = Sphere(Vector3(-50.0439f, -10.4745f, 74.0406f), 2.96427f); 
  data[20].mBoundingSphere = Sphere(Vector3(-27.6253f, -26.8226f, 82.636f), 0.207886f); 
  data[21].mBoundingSphere = Sphere(Vector3(-79.8805f, 29.2062f, -29.9426f), 3.96869f); 
  data[22].mBoundingSphere = Sphere(Vector3(54.5685f, -28.7595f, 35.9839f), 1.56926f); 
  data[23].mBoundingSphere = Sphere(Vector3(-92.3746f, 1.11599f, 19.078f), 1.81245f); 
  data[24].mBoundingSphere = Sphere(Vector3(43.0773f, 76.4745f, 30.6325f), 2.64813f); 
  data[25].mBoundingSphere = Sphere(Vector3(-13.3172f, -9.64942f, 11.5525f), 3.70532f); 
  data[26].mBoundingSphere = Sphere(Vector3(-62.035f, 17.5656f, 50.0426f), 1.10898f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(-1.92377f, -14.1896f, 14.7275f), 1.45445f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[2].mBoundingSphere = Sphere(Vector3(-6.07024f, -70.2122f, -62.5705f), 2.23922f); 
  spatialPartition.UpdateData(keys[2], data[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[25]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[26]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(23); 
  data[0].mBoundingSphere = Sphere(Vector3(-70.2642f, 4.87167f, 45.728f), 2.32701f); 
  data[1].mBoundingSphere = Sphere(Vector3(37.4993f, 44.5428f, 11.7449f), 2.95221f); 
  data[2].mBoundingSphere = Sphere(Vector3(34.3282f, 82.5481f, 33.445f), 1.73429f); 
  data[3].mBoundingSphere = Sphere(Vector3(-42.8193f, 57.2782f, -11.6195f), 0.30275f); 
  data[4].mBoundingSphere = Sphere(Vector3(56.3647f, 10.0423f, 17.0594f), 1.85559f); 
  data[5].mBoundingSphere = Sphere(Vector3(-33.1495f, 24.6258f, 49.3283f), 0.868221f); 
  data[6].mBoundingSphere = Sphere(Vector3(38.6551f, -20.4294f, -20.318f), 2.70751f); 
  data[7].mBoundingSphere = Sphere(Vector3(-50.5044f, 62.3435f, -31.3797f), 2.96496f); 
  data[8].mBoundingSphere = Sphere(Vector3(30.816f, -5.69794f, -72.5247f), 2.63167f); 
  data[9].mBoundingSphere = Sphere(Vector3(48.0349f, 30.5158f, -19.9518f), 0.331974f); 
  data[10].mBoundingSphere = Sphere(Vector3(-34.9065f, -61.9829f, 37.6909f), 3.5345f); 
  data[11].mBoundingSphere = Sphere(Vector3(69.1375f, -1.17339f, -42.6271f), 3.75936f); 
  data[12].mBoundingSphere = Sphere(Vector3(4.38108f, 59.5079f, 54.4677f), 3.30197f); 
  data[13].mBoundingSphere = Sphere(Vector3(40.8542f, -21.0336f, -85.6624f), 0.489114f); 
  data[14].mBoundingSphere = Sphere(Vector3(29.4559f, 18.3032f, -15.2568f), 1.98965f); 
  data[15].mBoundingSphere = Sphere(Vector3(31.6426f, 62.7336f, -47.3545f), 3.39417f); 
  data[16].mBoundingSphere = Sphere(Vector3(-7.44401f, 5.8765f, -32.2904f), 2.78741f); 
  data[17].mBoundingSphere = Sphere(Vector3(-41.4582f, -65.5296f, 62.9853f), 3.72886f); 
  data[18].mBoundingSphere = Sphere(Vector3(6.36503f, -18.7788f, 42.8454f), 3.95964f); 
  data[19].mBoundingSphere = Sphere(Vector3(52.6522f, 84.5268f, 4.21301f), 2.25801f); 
  data[20].mBoundingSphere = Sphere(Vector3(-8.52103f, 75.6752f, 64.8014f), 0.758977f); 
  data[21].mBoundingSphere = Sphere(Vector3(-6.51115f, -53.6572f, -57.8105f), 0.958214f); 
  data[22].mBoundingSphere = Sphere(Vector3(51.0326f, 2.59053f, -2.38073f), 1.11791f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[15].mBoundingSphere = Sphere(Vector3(50.1862f, -16.1274f, -78.7443f), 2.05877f); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[19].mBoundingSphere = Sphere(Vector3(50.1296f, 41.9447f, 10.1314f), 1.80062f); 
  spatialPartition.UpdateData(keys[19], data[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[14].mBoundingSphere = Sphere(Vector3(-38.9184f, -22.9338f, -34.3103f), 1.24141f); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[4].mBoundingSphere = Sphere(Vector3(-41.7977f, 42.9372f, -57.9996f), 0.859639f); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[12].mBoundingSphere = Sphere(Vector3(14.5389f, -5.82576f, -15.9773f), 2.79832f); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(21); 
  data[0].mBoundingSphere = Sphere(Vector3(80.276f, -32.7756f, -7.30023f), 0.957518f); 
  data[1].mBoundingSphere = Sphere(Vector3(-53.6031f, -41.0132f, 63.9825f), 3.46433f); 
  data[2].mBoundingSphere = Sphere(Vector3(24.5205f, 6.79356f, -73.7427f), 3.45911f); 
  data[3].mBoundingSphere = Sphere(Vector3(-60.1524f, -58.2256f, -27.1759f), 1.93376f); 
  data[4].mBoundingSphere = Sphere(Vector3(28.2248f, -2.35706f, -26.1395f), 0.307157f); 
  data[5].mBoundingSphere = Sphere(Vector3(11.9052f, 24.1181f, 28.2442f), 1.44622f); 
  data[6].mBoundingSphere = Sphere(Vector3(0.587453f, -66.2393f, 4.34973f), 3.06424f); 
  data[7].mBoundingSphere = Sphere(Vector3(86.1438f, 36.7425f, 12.6173f), 0.716068f); 
  data[8].mBoundingSphere = Sphere(Vector3(-4.80868f, 5.88729f, -34.2589f), 0.509293f); 
  data[9].mBoundingSphere = Sphere(Vector3(21.7903f, -37.7977f, 69.5803f), 1.74786f); 
  data[10].mBoundingSphere = Sphere(Vector3(-27.7419f, 51.4065f, 1.14294f), 2.82928f); 
  data[11].mBoundingSphere = Sphere(Vector3(24.9338f, -67.5817f, -23.0776f), 2.01354f); 
  data[12].mBoundingSphere = Sphere(Vector3(-5.39172f, 94.3316f, -8.8595f), 0.781939f); 
  data[13].mBoundingSphere = Sphere(Vector3(15.2175f, 86.8818f, -1.93592f), 0.405036f); 
  data[14].mBoundingSphere = Sphere(Vector3(-53.2695f, -11.8016f, 64.6989f), 3.23773f); 
  data[15].mBoundingSphere = Sphere(Vector3(-66.6288f, -3.08128f, -67.1702f), 1.76166f); 
  data[16].mBoundingSphere = Sphere(Vector3(40.7331f, 62.945f, -65.2277f), 1.09286f); 
  data[17].mBoundingSphere = Sphere(Vector3(-25.5357f, -45.1811f, 37.7691f), 0.97213f); 
  data[18].mBoundingSphere = Sphere(Vector3(-24.6295f, -93.534f, -23.7798f), 1.9602f); 
  data[19].mBoundingSphere = Sphere(Vector3(29.2858f, -70.8824f, 45.604f), 3.65232f); 
  data[20].mBoundingSphere = Sphere(Vector3(-67.8505f, 36.3686f, -51.2125f), 2.6341f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[2].mBoundingSphere = Sphere(Vector3(51.1139f, -14.8073f, 69.3028f), 2.04428f); 
  spatialPartition.UpdateData(keys[2], data[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[8].mBoundingSphere = Sphere(Vector3(35.0135f, -16.45f, -36.5005f), 0.697513f); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[17].mBoundingSphere = Sphere(Vector3(4.04547f, -1.72824f, 32.3958f), 0.41756f); 
  spatialPartition.UpdateData(keys[17], data[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[15].mBoundingSphere = Sphere(Vector3(27.3998f, 43.8184f, 45.086f), 3.11062f); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(23); 
  data[0].mBoundingSphere = Sphere(Vector3(-64.3305f, -24.1098f, 41.1287f), 3.96764f); 
  data[1].mBoundingSphere = Sphere(Vector3(-11.7477f, 46.6418f, 55.9049f), 0.707254f); 
  data[2].mBoundingSphere = Sphere(Vector3(11.6174f, 35.2504f, -8.37812f), 2.06596f); 
  data[3].mBoundingSphere = Sphere(Vector3(73.8826f, 20.9739f, -12.5127f), 3.44149f); 
  data[4].mBoundingSphere = Sphere(Vector3(-6.23466f, -81.3806f, 22.2526f), 1.19329f); 
  data[5].mBoundingSphere = Sphere(Vector3(-32.3698f, -1.94513f, -70.9677f), 2.73558f); 
  data[6].mBoundingSphere = Sphere(Vector3(66.6009f, 30.4525f, -57.9965f), 2.31704f); 
  data[7].mBoundingSphere = Sphere(Vector3(-15.7209f, -6.26965f, 92.2447f), 0.985467f); 
  data[8].mBoundingSphere = Sphere(Vector3(-23.5303f, -16.974f, -11.9681f), 2.92727f); 
  data[9].mBoundingSphere = Sphere(Vector3(55.2136f, 74.7355f, 2.01102f), 2.07617f); 
  data[10].mBoundingSphere = Sphere(Vector3(-16.9238f, -4.22553f, -80.0105f), 1.38719f); 
  data[11].mBoundingSphere = Sphere(Vector3(-30.8837f, 21.7959f, -33.1234f), 2.66727f); 
  data[12].mBoundingSphere = Sphere(Vector3(25.5947f, -74.5237f, -7.03774f), 1.75644f); 
  data[13].mBoundingSphere = Sphere(Vector3(-26.7832f, 38.6581f, -22.8843f), 2.95557f); 
  data[14].mBoundingSphere = Sphere(Vector3(-64.758f, -21.2287f, -9.91797f), 3.89076f); 
  data[15].mBoundingSphere = Sphere(Vector3(-1.59029f, 85.7095f, -9.20665f), 2.7118f); 
  data[16].mBoundingSphere = Sphere(Vector3(-22.9972f, 33.3845f, 90.0516f), 1.61588f); 
  data[17].mBoundingSphere = Sphere(Vector3(-43.4322f, -36.6177f, -34.7628f), 1.59188f); 
  data[18].mBoundingSphere = Sphere(Vector3(31.467f, -11.976f, -74.1155f), 0.377319f); 
  data[19].mBoundingSphere = Sphere(Vector3(-0.549738f, -3.94676f, -90.101f), 3.72411f); 
  data[20].mBoundingSphere = Sphere(Vector3(-33.3718f, -85.641f, 7.77808f), 1.16395f); 
  data[21].mBoundingSphere = Sphere(Vector3(48.5031f, -16.8169f, 32.6922f), 3.77026f); 
  data[22].mBoundingSphere = Sphere(Vector3(-17.4551f, -9.4122f, -22.3941f), 3.90746f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(-66.9303f, 28.4792f, -16.9273f), 0.920988f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[21].mBoundingSphere = Sphere(Vector3(-56.4211f, 68.0849f, -19.7684f), 3.69627f); 
  spatialPartition.UpdateData(keys[21], data[21]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[17].mBoundingSphere = Sphere(Vector3(25.3919f, -64.4854f, 8.54351f), 0.837721f); 
  spatialPartition.UpdateData(keys[17], data[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[12].mBoundingSphere = Sphere(Vector3(-39.5945f, -55.1382f, 13.6794f), 1.37872f); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mBoundingSphere = Sphere(Vector3(-58.1993f, -28.8777f, -9.68055f), 3.91279f); 
  data[1].mBoundingSphere = Sphere(Vector3(-12.9947f, 19.1536f, -10.577f), 3.47373f); 
  data[2].mBoundingSphere = Sphere(Vector3(-33.0333f, -50.8004f, -72.6075f), 1.73382f); 
  data[3].mBoundingSphere = Sphere(Vector3(11.1478f, 57.2897f, -25.6117f), 1.18586f); 
  data[4].mBoundingSphere = Sphere(Vector3(-40.7525f, -42.6582f, -50.3156f), 1.40087f); 
  data[5].mBoundingSphere = Sphere(Vector3(15.8742f, 58.6153f, 14.0961f), 1.47196f); 
  data[6].mBoundingSphere = Sphere(Vector3(-37.3762f, -2.23159f, -76.7177f), 2.28781f); 
  data[7].mBoundingSphere = Sphere(Vector3(59.4915f, 14.5814f, -66.0327f), 2.57925f); 
  data[8].mBoundingSphere = Sphere(Vector3(16.2774f, 77.6018f, 15.9836f), 3.58285f); 
  data[9].mBoundingSphere = Sphere(Vector3(62.4171f, 65.2481f, 12.9761f), 1.06375f); 
  data[10].mBoundingSphere = Sphere(Vector3(47.5283f, 19.7677f, 80.1134f), 2.3451f); 
  data[11].mBoundingSphere = Sphere(Vector3(-33.0021f, 22.9795f, 37.9624f), 1.48263f); 
  data[12].mBoundingSphere = Sphere(Vector3(32.7879f, -30.0813f, -17.4903f), 3.4343f); 
  data[13].mBoundingSphere = Sphere(Vector3(-8.73345f, -89.594f, 16.9607f), 3.76214f); 
  data[14].mBoundingSphere = Sphere(Vector3(18.4456f, -76.0927f, -20.8265f), 3.31647f); 
  data[15].mBoundingSphere = Sphere(Vector3(36.4916f, 2.12611f, 32.4915f), 2.44066f); 
  data[16].mBoundingSphere = Sphere(Vector3(-63.3439f, -21.0075f, -63.1054f), 0.94453f); 
  data[17].mBoundingSphere = Sphere(Vector3(37.4085f, -79.9918f, 42.0339f), 1.58701f); 
  data[18].mBoundingSphere = Sphere(Vector3(-34.6548f, -25.9198f, 5.77877f), 1.51371f); 
  data[19].mBoundingSphere = Sphere(Vector3(9.7057f, -50.7079f, -7.5332f), 3.59283f); 
  data[20].mBoundingSphere = Sphere(Vector3(-23.3655f, -37.9713f, 29.0801f), 1.69184f); 
  data[21].mBoundingSphere = Sphere(Vector3(-64.1162f, 12.8306f, -46.093f), 2.24108f); 
  data[22].mBoundingSphere = Sphere(Vector3(65.839f, -1.95111f, 52.1472f), 1.92506f); 
  data[23].mBoundingSphere = Sphere(Vector3(98.6308f, -4.43804f, 1.29273f), 2.84865f); 
  data[24].mBoundingSphere = Sphere(Vector3(-7.50945f, 30.7337f, -15.651f), 0.218091f); 
  data[25].mBoundingSphere = Sphere(Vector3(26.4897f, 58.2662f, -43.5227f), 1.08439f); 
  data[26].mBoundingSphere = Sphere(Vector3(41.5483f, 60.2282f, -61.8992f), 1.58979f); 
  data[27].mBoundingSphere = Sphere(Vector3(-44.0344f, 30.6864f, -63.3524f), 0.262392f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[16].mBoundingSphere = Sphere(Vector3(55.2857f, 44.4673f, 0.279323f), 2.60859f); 
  spatialPartition.UpdateData(keys[16], data[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[20].mBoundingSphere = Sphere(Vector3(29.9002f, -15.3577f, -17.7387f), 3.23204f); 
  spatialPartition.UpdateData(keys[20], data[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[22].mBoundingSphere = Sphere(Vector3(38.4892f, 8.90342f, 32.9268f), 0.80351f); 
  spatialPartition.UpdateData(keys[22], data[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[27]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(17); 
  data[0].mBoundingSphere = Sphere(Vector3(-2.71325f, -58.7315f, 51.8331f), 0.915769f); 
  data[1].mBoundingSphere = Sphere(Vector3(82.477f, 22.0184f, 5.30665f), 2.84273f); 
  data[2].mBoundingSphere = Sphere(Vector3(31.8258f, -26.6294f, 13.4853f), 1.65543f); 
  data[3].mBoundingSphere = Sphere(Vector3(-27.5031f, 50.247f, 66.4275f), 1.67653f); 
  data[4].mBoundingSphere = Sphere(Vector3(12.6809f, -68.9699f, 52.9502f), 1.47857f); 
  data[5].mBoundingSphere = Sphere(Vector3(7.50023f, -39.1447f, -25.5079f), 0.941282f); 
  data[6].mBoundingSphere = Sphere(Vector3(-4.63373f, -64.5449f, -64.229f), 0.953111f); 
  data[7].mBoundingSphere = Sphere(Vector3(32.8333f, -19.1576f, -21.7115f), 3.95141f); 
  data[8].mBoundingSphere = Sphere(Vector3(-70.7473f, 15.6595f, -1.59478f), 2.59572f); 
  data[9].mBoundingSphere = Sphere(Vector3(17.2671f, -68.918f, -3.68453f), 2.76631f); 
  data[10].mBoundingSphere = Sphere(Vector3(69.7687f, -0.140476f, -48.739f), 3.58343f); 
  data[11].mBoundingSphere = Sphere(Vector3(1.01605f, -10.3108f, -27.9271f), 3.43975f); 
  data[12].mBoundingSphere = Sphere(Vector3(-23.1679f, 46.675f, -66.9988f), 0.34044f); 
  data[13].mBoundingSphere = Sphere(Vector3(88.5441f, -37.666f, 4.77191f), 2.34441f); 
  data[14].mBoundingSphere = Sphere(Vector3(-50.5248f, -28.4186f, -30.3923f), 1.20001f); 
  data[15].mBoundingSphere = Sphere(Vector3(9.86957f, 13.5594f, -77.4613f), 0.970623f); 
  data[16].mBoundingSphere = Sphere(Vector3(59.0736f, -27.2988f, -55.2767f), 1.54085f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[4].mBoundingSphere = Sphere(Vector3(7.63147f, -94.4966f, 2.3935f), 0.450148f); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(18.9945f, 5.96756f, -17.7323f), 3.35381f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[1].mBoundingSphere = Sphere(Vector3(2.17459f, -25.6396f, -36.2148f), 0.448177f); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[7].mBoundingSphere = Sphere(Vector3(-72.9868f, 64.5406f, 19.8233f), 0.61297f); 
  spatialPartition.UpdateData(keys[7], data[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[10].mBoundingSphere = Sphere(Vector3(-55.6273f, -43.6374f, -18.4959f), 2.95754f); 
  spatialPartition.UpdateData(keys[10], data[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[15].mBoundingSphere = Sphere(Vector3(-17.1721f, -29.82f, 79.7028f), 1.10863f); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mBoundingSphere = Sphere(Vector3(-41.2654f, -56.6359f, 65.3345f), 2.1069f); 
  data[1].mBoundingSphere = Sphere(Vector3(15.1482f, 44.9468f, 65.7853f), 0.818238f); 
  data[2].mBoundingSphere = Sphere(Vector3(13.3587f, 23.5989f, 16.7493f), 3.3123f); 
  data[3].mBoundingSphere = Sphere(Vector3(31.843f, -11.7084f, 86.3406f), 2.80609f); 
  data[4].mBoundingSphere = Sphere(Vector3(15.7399f, -79.9886f, 16.9468f), 3.76528f); 
  data[5].mBoundingSphere = Sphere(Vector3(30.3466f, -69.6721f, 26.9308f), 1.34903f); 
  data[6].mBoundingSphere = Sphere(Vector3(-78.8927f, 7.64832f, 21.5886f), 1.80317f); 
  data[7].mBoundingSphere = Sphere(Vector3(15.9724f, 1.09666f, 70.0356f), 0.316782f); 
  data[8].mBoundingSphere = Sphere(Vector3(14.8348f, 11.7087f, -36.3413f), 1.96356f); 
  data[9].mBoundingSphere = Sphere(Vector3(-62.7308f, -22.2158f, 60.5966f), 2.24595f); 
  data[10].mBoundingSphere = Sphere(Vector3(32.8014f, -65.3422f, -63.2248f), 1.46048f); 
  data[11].mBoundingSphere = Sphere(Vector3(-18.0204f, 17.7741f, -68.9328f), 3.60396f); 
  data[12].mBoundingSphere = Sphere(Vector3(17.1295f, -42.495f, -43.9953f), 1.83727f); 
  data[13].mBoundingSphere = Sphere(Vector3(-44.8903f, -6.23734f, 56.1469f), 3.82929f); 
  data[14].mBoundingSphere = Sphere(Vector3(84.3228f, 2.6931f, -7.07692f), 3.9942f); 
  data[15].mBoundingSphere = Sphere(Vector3(-92.2092f, 9.49289f, -7.91299f), 1.54653f); 
  data[16].mBoundingSphere = Sphere(Vector3(60.1534f, 1.39017f, 65.335f), 1.78984f); 
  data[17].mBoundingSphere = Sphere(Vector3(-9.36962f, -14.0177f, -5.0884f), 0.756194f); 
  data[18].mBoundingSphere = Sphere(Vector3(53.4573f, 30.6441f, -61.8162f), 0.819745f); 
  data[19].mBoundingSphere = Sphere(Vector3(66.0012f, -36.0564f, -43.5822f), 0.398889f); 
  data[20].mBoundingSphere = Sphere(Vector3(-32.0337f, 49.0572f, -22.1539f), 1.56787f); 
  data[21].mBoundingSphere = Sphere(Vector3(-11.9183f, 16.5368f, 41.6335f), 1.49608f); 
  data[22].mBoundingSphere = Sphere(Vector3(53.5844f, -8.94388f, 11.9741f), 3.93424f); 
  data[23].mBoundingSphere = Sphere(Vector3(59.7397f, 15.6787f, -67.2233f), 1.46442f); 
  data[24].mBoundingSphere = Sphere(Vector3(-25.7447f, -39.1957f, 72.4984f), 2.73685f); 
  data[25].mBoundingSphere = Sphere(Vector3(73.591f, 36.929f, 33.6464f), 2.20571f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(32.9105f, -39.3211f, -72.4737f), 0.882949f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[25]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[21].mBoundingSphere = Sphere(Vector3(-43.3057f, 8.1233f, -33.7492f), 0.801886f); 
  spatialPartition.UpdateData(keys[21], data[21]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mBoundingSphere = Sphere(Vector3(-9.17275f, -90.1601f, 33.1252f), 3.26417f); 
  data[1].mBoundingSphere = Sphere(Vector3(-17.0369f, -40.4097f, -12.1535f), 0.895358f); 
  data[2].mBoundingSphere = Sphere(Vector3(-56.0453f, -4.81565f, -22.7622f), 2.89086f); 
  data[3].mBoundingSphere = Sphere(Vector3(-69.2687f, 4.9495f, -49.6887f), 3.79833f); 
  data[4].mBoundingSphere = Sphere(Vector3(60.343f, 75.6231f, 15.3881f), 2.14204f); 
  data[5].mBoundingSphere = Sphere(Vector3(-15.2856f, 37.2496f, -45.5184f), 2.26694f); 
  data[6].mBoundingSphere = Sphere(Vector3(-66.1759f, 45.759f, -24.152f), 2.26717f); 
  data[7].mBoundingSphere = Sphere(Vector3(-79.149f, -21.0568f, -50.1667f), 2.31843f); 
  data[8].mBoundingSphere = Sphere(Vector3(-2.39581f, -46.6226f, -37.7012f), 1.22993f); 
  data[9].mBoundingSphere = Sphere(Vector3(12.8475f, -63.5513f, -67.1075f), 0.338585f); 
  data[10].mBoundingSphere = Sphere(Vector3(14.9692f, -9.42935f, -37.9541f), 1.5608f); 
  data[11].mBoundingSphere = Sphere(Vector3(56.1675f, 24.1608f, -1.5138f), 3.5222f); 
  data[12].mBoundingSphere = Sphere(Vector3(-4.12348f, -67.8061f, 43.0539f), 1.58445f); 
  data[13].mBoundingSphere = Sphere(Vector3(23.7749f, 51.1195f, -36.8192f), 3.2869f); 
  data[14].mBoundingSphere = Sphere(Vector3(46.8321f, 10.0321f, -87.4753f), 0.421967f); 
  data[15].mBoundingSphere = Sphere(Vector3(38.5346f, 44.4932f, -7.13847f), 0.463832f); 
  data[16].mBoundingSphere = Sphere(Vector3(-32.8993f, -19.175f, -30.7769f), 1.56833f); 
  data[17].mBoundingSphere = Sphere(Vector3(-4.33462f, -50.0252f, 41.5413f), 2.46247f); 
  data[18].mBoundingSphere = Sphere(Vector3(-35.7329f, 74.5565f, -53.6604f), 3.64455f); 
  data[19].mBoundingSphere = Sphere(Vector3(-14.4486f, 56.5858f, 35.8237f), 3.80992f); 
  data[20].mBoundingSphere = Sphere(Vector3(-15.1591f, -8.3173f, -69.5492f), 1.12498f); 
  data[21].mBoundingSphere = Sphere(Vector3(-20.665f, 7.6636f, -30.3488f), 3.26591f); 
  data[22].mBoundingSphere = Sphere(Vector3(-75.132f, 2.26253f, 23.4509f), 2.64036f); 
  data[23].mBoundingSphere = Sphere(Vector3(54.8475f, 64.5944f, -12.824f), 0.513352f); 
  data[24].mBoundingSphere = Sphere(Vector3(-66.0237f, -8.9029f, 23.6869f), 2.25395f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[23].mBoundingSphere = Sphere(Vector3(71.7969f, 58.7981f, 14.4078f), 1.65438f); 
  spatialPartition.UpdateData(keys[23], data[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[7].mBoundingSphere = Sphere(Vector3(58.767f, -21.7745f, 50.3368f), 3.35926f); 
  spatialPartition.UpdateData(keys[7], data[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[21].mBoundingSphere = Sphere(Vector3(-4.08342f, -14.703f, 23.4409f), 0.328495f); 
  spatialPartition.UpdateData(keys[21], data[21]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(19); 
  data[0].mBoundingSphere = Sphere(Vector3(60.3116f, -46.5968f, -31.8815f), 1.0641f); 
  data[1].mBoundingSphere = Sphere(Vector3(-79.9344f, 40.6606f, -22.4079f), 1.79854f); 
  data[2].mBoundingSphere = Sphere(Vector3(84.9251f, -18.5501f, 33.598f), 3.57114f); 
  data[3].mBoundingSphere = Sphere(Vector3(-51.5841f, -12.1568f, 79.5262f), 2.74056f); 
  data[4].mBoundingSphere = Sphere(Vector3(26.2984f, -18.5561f, 20.0142f), 2.40796f); 
  data[5].mBoundingSphere = Sphere(Vector3(-2.13329f, 8.81883f, -42.5644f), 2.81293f); 
  data[6].mBoundingSphere = Sphere(Vector3(16.6163f, -0.913774f, -31.5129f), 0.555217f); 
  data[7].mBoundingSphere = Sphere(Vector3(-12.219f, 46.0801f, -17.813f), 3.40936f); 
  data[8].mBoundingSphere = Sphere(Vector3(-35.8408f, -14.3973f, 7.00463f), 1.0002f); 
  data[9].mBoundingSphere = Sphere(Vector3(58.204f, -2.60779f, 40.1622f), 0.711893f); 
  data[10].mBoundingSphere = Sphere(Vector3(24.6332f, -37.2152f, -20.3726f), 0.694034f); 
  data[11].mBoundingSphere = Sphere(Vector3(-56.852f, -39.2473f, -36.0498f), 1.73742f); 
  data[12].mBoundingSphere = Sphere(Vector3(14.5309f, -3.44066f, -65.8221f), 2.08661f); 
  data[13].mBoundingSphere = Sphere(Vector3(29.8996f, 25.8177f, 68.5765f), 0.958214f); 
  data[14].mBoundingSphere = Sphere(Vector3(40.4964f, 13.6073f, -45.8902f), 0.434144f); 
  data[15].mBoundingSphere = Sphere(Vector3(-61.2808f, -16.2025f, -64.1331f), 1.09831f); 
  data[16].mBoundingSphere = Sphere(Vector3(-64.3215f, 6.48477f, 58.5448f), 3.88403f); 
  data[17].mBoundingSphere = Sphere(Vector3(22.3502f, 35.9881f, -88.5263f), 2.10458f); 
  data[18].mBoundingSphere = Sphere(Vector3(0.728714f, 16.5662f, 44.0368f), 2.73175f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[4].mBoundingSphere = Sphere(Vector3(14.3293f, -62.0671f, 7.64589f), 2.04613f); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[16].mBoundingSphere = Sphere(Vector3(-16.8649f, 44.6668f, -76.2413f), 1.41711f); 
  spatialPartition.UpdateData(keys[16], data[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[13].mBoundingSphere = Sphere(Vector3(-45.9931f, 20.0093f, -18.1526f), 1.66007f); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[6].mBoundingSphere = Sphere(Vector3(20.9991f, 18.4867f, -69.4408f), 3.84031f); 
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(17); 
  data[0].mBoundingSphere = Sphere(Vector3(68.9257f, 12.7718f, 31.6478f), 0.954619f); 
  data[1].mBoundingSphere = Sphere(Vector3(-32.75f, 78.7742f, 19.6506f), 1.49539f); 
  data[2].mBoundingSphere = Sphere(Vector3(-5.93595f, -48.9171f, 40.5044f), 0.743321f); 
  data[3].mBoundingSphere = Sphere(Vector3(39.9447f, -38.5837f, -21.0449f), 2.20455f); 
  data[4].mBoundingSphere = Sphere(Vector3(-17.8272f, -46.2997f, 13.6008f), 3.36054f); 
  data[5].mBoundingSphere = Sphere(Vector3(2.86024f, -4.1701f, 41.9046f), 1.17183f); 
  data[6].mBoundingSphere = Sphere(Vector3(58.3545f, 6.9648f, 25.426f), 2.78498f); 
  data[7].mBoundingSphere = Sphere(Vector3(28.7313f, 83.2921f, 30.0558f), 1.66018f); 
  data[8].mBoundingSphere = Sphere(Vector3(-32.2435f, -54.8671f, -59.4445f), 2.8396f); 
  data[9].mBoundingSphere = Sphere(Vector3(29.8795f, -43.6346f, 76.129f), 0.574352f); 
  data[10].mBoundingSphere = Sphere(Vector3(-62.6452f, -71.4276f, -30.7734f), 3.43024f); 
  data[11].mBoundingSphere = Sphere(Vector3(-41.2878f, 27.4172f, -20.2531f), 2.48821f); 
  data[12].mBoundingSphere = Sphere(Vector3(-5.44496f, 12.14f, -78.5659f), 0.561711f); 
  data[13].mBoundingSphere = Sphere(Vector3(73.2126f, 48.4452f, -10.8826f), 1.84713f); 
  data[14].mBoundingSphere = Sphere(Vector3(24.1784f, -9.64526f, -25.4724f), 3.92705f); 
  data[15].mBoundingSphere = Sphere(Vector3(-3.70748f, -35.4289f, 2.80807f), 1.74507f); 
  data[16].mBoundingSphere = Sphere(Vector3(36.61f, 53.9726f, 2.34814f), 3.6718f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[8].mBoundingSphere = Sphere(Vector3(-55.5583f, 15.8123f, 38.3605f), 2.96392f); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[12].mBoundingSphere = Sphere(Vector3(-33.9047f, 65.5681f, 51.5168f), 1.90418f); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[6].mBoundingSphere = Sphere(Vector3(22.9416f, -15.2883f, 32.585f), 3.00034f); 
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(4.40387f, 1.27302f, -97.7121f), 2.51778f); 
  data[1].mBoundingSphere = Sphere(Vector3(-24.1609f, 20.8422f, 30.5824f), 2.65915f); 
  data[2].mBoundingSphere = Sphere(Vector3(-27.1171f, 55.3223f, -63.5783f), 3.45274f); 
  data[3].mBoundingSphere = Sphere(Vector3(40.5447f, -57.8432f, -44.6121f), 2.62564f); 
  data[4].mBoundingSphere = Sphere(Vector3(-2.40887f, -78.1233f, -55.8536f), 0.222382f); 
  data[5].mBoundingSphere = Sphere(Vector3(8.56518f, 82.2303f, -54.9119f), 3.13138f); 
  data[6].mBoundingSphere = Sphere(Vector3(-33.9876f, -72.8042f, -5.71545f), 2.1853f); 
  data[7].mBoundingSphere = Sphere(Vector3(19.5856f, 38.5351f, -36.7711f), 0.334873f); 
  data[8].mBoundingSphere = Sphere(Vector3(10.5587f, 96.0063f, 25.6847f), 1.28397f); 
  data[9].mBoundingSphere = Sphere(Vector3(-85.342f, 50.3343f, -9.12487f), 2.88251f); 
  data[10].mBoundingSphere = Sphere(Vector3(49.8776f, 69.8391f, -49.7245f), 3.47094f); 
  data[11].mBoundingSphere = Sphere(Vector3(-43.1877f, -8.79764f, 43.3206f), 0.869729f); 
  data[12].mBoundingSphere = Sphere(Vector3(-50.9024f, 55.6053f, -59.606f), 3.60698f); 
  data[13].mBoundingSphere = Sphere(Vector3(-14.015f, 58.7741f, -63.2254f), 3.67865f); 
  data[14].mBoundingSphere = Sphere(Vector3(19.2961f, 28.5063f, -20.8756f), 2.63538f); 
  data[15].mBoundingSphere = Sphere(Vector3(-49.3246f, -35.1374f, 34.6173f), 2.28897f); 
  data[16].mBoundingSphere = Sphere(Vector3(46.9649f, -28.7478f, 56.6641f), 2.43173f); 
  data[17].mBoundingSphere = Sphere(Vector3(57.2312f, 66.4663f, 11.7288f), 0.465108f); 
  data[18].mBoundingSphere = Sphere(Vector3(-17.2424f, -8.90435f, -53.889f), 1.12753f); 
  data[19].mBoundingSphere = Sphere(Vector3(-41.5512f, -13.3745f, 46.0418f), 0.818122f); 
  data[20].mBoundingSphere = Sphere(Vector3(-29.3169f, -42.0562f, -28.4061f), 0.30936f); 
  data[21].mBoundingSphere = Sphere(Vector3(-33.1134f, 87.144f, -31.165f), 2.74172f); 
  data[22].mBoundingSphere = Sphere(Vector3(44.0032f, -39.1599f, 22.3917f), 1.29395f); 
  data[23].mBoundingSphere = Sphere(Vector3(13.3786f, 28.0453f, -12.3506f), 0.824732f); 
  data[24].mBoundingSphere = Sphere(Vector3(-88.9177f, -14.9554f, -40.8606f), 1.87635f); 
  data[25].mBoundingSphere = Sphere(Vector3(-7.05971f, 13.756f, -86.7831f), 2.73731f); 
  data[26].mBoundingSphere = Sphere(Vector3(-8.02045f, 87.4891f, -29.2074f), 1.15478f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[15].mBoundingSphere = Sphere(Vector3(-15.6669f, -77.6893f, -46.9053f), 2.56788f); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[8].mBoundingSphere = Sphere(Vector3(-36.1563f, -26.7842f, 11.0056f), 3.9826f); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[19].mBoundingSphere = Sphere(Vector3(-62.4529f, -40.8023f, 7.96562f), 2.61439f); 
  spatialPartition.UpdateData(keys[19], data[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[6].mBoundingSphere = Sphere(Vector3(-56.9239f, 36.6473f, 48.6327f), 0.59975f); 
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[13].mBoundingSphere = Sphere(Vector3(23.9848f, 67.5098f, 24.1408f), 0.435768f); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(19); 
  data[0].mBoundingSphere = Sphere(Vector3(40.883f, 8.3159f, -12.0049f), 0.336729f); 
  data[1].mBoundingSphere = Sphere(Vector3(-41.5171f, 54.2024f, 65.4826f), 1.7605f); 
  data[2].mBoundingSphere = Sphere(Vector3(-10.9036f, -83.3015f, -28.9735f), 2.81397f); 
  data[3].mBoundingSphere = Sphere(Vector3(57.1794f, 67.4455f, 30.6029f), 0.294748f); 
  data[4].mBoundingSphere = Sphere(Vector3(-72.6988f, 65.2361f, 1.55328f), 3.8548f); 
  data[5].mBoundingSphere = Sphere(Vector3(37.8496f, -9.37706f, -14.6281f), 3.76516f); 
  data[6].mBoundingSphere = Sphere(Vector3(72.6728f, -14.1592f, -44.1823f), 2.8142f); 
  data[7].mBoundingSphere = Sphere(Vector3(-34.2903f, 53.4936f, 70.4291f), 2.73175f); 
  data[8].mBoundingSphere = Sphere(Vector3(18.012f, 38.0206f, -10.1699f), 0.832502f); 
  data[9].mBoundingSphere = Sphere(Vector3(-37.8158f, 4.26724f, 32.5941f), 1.27423f); 
  data[10].mBoundingSphere = Sphere(Vector3(34.2258f, 5.02008f, -92.3016f), 1.62226f); 
  data[11].mBoundingSphere = Sphere(Vector3(-15.5235f, -39.6798f, -88.4604f), 0.916001f); 
  data[12].mBoundingSphere = Sphere(Vector3(46.97f, 14.559f, -0.505775f), 1.3299f); 
  data[13].mBoundingSphere = Sphere(Vector3(-56.8815f, 38.1506f, 36.966f), 0.223194f); 
  data[14].mBoundingSphere = Sphere(Vector3(34.1006f, 82.4462f, -34.2228f), 1.67224f); 
  data[15].mBoundingSphere = Sphere(Vector3(54.0645f, -63.5487f, 4.30645f), 2.47905f); 
  data[16].mBoundingSphere = Sphere(Vector3(8.88711f, 97.045f, -3.63385f), 0.748772f); 
  data[17].mBoundingSphere = Sphere(Vector3(-32.532f, 33.2782f, 24.518f), 1.0329f); 
  data[18].mBoundingSphere = Sphere(Vector3(86.473f, -33.5004f, 33.5963f), 3.32772f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[14].mBoundingSphere = Sphere(Vector3(7.27136f, -75.8754f, -47.9627f), 3.55108f); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[8].mBoundingSphere = Sphere(Vector3(47.295f, 21.9328f, -38.3641f), 3.45587f); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[11].mBoundingSphere = Sphere(Vector3(-37.2189f, -1.6926f, 6.63984f), 3.63133f); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[5].mBoundingSphere = Sphere(Vector3(38.0731f, -40.8376f, -26.1063f), 3.51142f); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[6].mBoundingSphere = Sphere(Vector3(-25.6027f, -33.4123f, 8.38995f), 2.90536f); 
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionStructureFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(24); 
  data[0].mBoundingSphere = Sphere(Vector3(-52.7098f, 28.5138f, -32.5596f), 3.57938f); 
  data[1].mBoundingSphere = Sphere(Vector3(-57.9605f, -41.3061f, 10.3312f), 1.83507f); 
  data[2].mBoundingSphere = Sphere(Vector3(26.9475f, 62.6873f, 9.91317f), 1.49272f); 
  data[3].mBoundingSphere = Sphere(Vector3(-49.5544f, 44.0235f, -2.14745f), 3.62692f); 
  data[4].mBoundingSphere = Sphere(Vector3(-44.8781f, 41.118f, -13.4557f), 2.99778f); 
  data[5].mBoundingSphere = Sphere(Vector3(24.5392f, -57.8476f, 16.9016f), 2.21452f); 
  data[6].mBoundingSphere = Sphere(Vector3(36.121f, 1.54897f, 5.36876f), 1.85942f); 
  data[7].mBoundingSphere = Sphere(Vector3(57.9314f, 9.6923f, 0.148785f), 0.415937f); 
  data[8].mBoundingSphere = Sphere(Vector3(30.1503f, 12.5264f, 12.0953f), 1.74438f); 
  data[9].mBoundingSphere = Sphere(Vector3(22.8202f, 53.2549f, -26.8773f), 3.15052f); 
  data[10].mBoundingSphere = Sphere(Vector3(99.3294f, -0.0285605f, -4.71912f), 3.58842f); 
  data[11].mBoundingSphere = Sphere(Vector3(3.6322f, 33.6193f, 91.0986f), 1.24362f); 
  data[12].mBoundingSphere = Sphere(Vector3(19.2551f, -90.4162f, 25.8907f), 1.03905f); 
  data[13].mBoundingSphere = Sphere(Vector3(-8.41794f, 79.9976f, 0.331412f), 1.30752f); 
  data[14].mBoundingSphere = Sphere(Vector3(-64.4393f, -37.2769f, -8.56453f), 2.97471f); 
  data[15].mBoundingSphere = Sphere(Vector3(-58.3615f, -17.4114f, -52.3072f), 2.89689f); 
  data[16].mBoundingSphere = Sphere(Vector3(73.372f, 48.3486f, 45.0418f), 3.29641f); 
  data[17].mBoundingSphere = Sphere(Vector3(21.0705f, 19.7976f, -60.8152f), 2.73882f); 
  data[18].mBoundingSphere = Sphere(Vector3(-40.5631f, 8.48196f, 46.1612f), 0.596155f); 
  data[19].mBoundingSphere = Sphere(Vector3(77.3413f, 42.9861f, 8.58696f), 3.23877f); 
  data[20].mBoundingSphere = Sphere(Vector3(27.6864f, -22.1031f, 77.5308f), 3.10053f); 
  data[21].mBoundingSphere = Sphere(Vector3(-25.0561f, 9.3577f, -24.1456f), 2.46096f); 
  data[22].mBoundingSphere = Sphere(Vector3(1.54856f, -9.10823f, -28.7225f), 3.68781f); 
  data[23].mBoundingSphere = Sphere(Vector3(-52.4905f, -70.7511f, -40.99f), 3.58367f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[15].mBoundingSphere = Sphere(Vector3(43.2708f, -5.06355f, -33.8523f), 1.01829f); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[13].mBoundingSphere = Sphere(Vector3(-17.9609f, 81.8324f, -4.97157f), 0.499319f); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);

  data[17].mBoundingSphere = Sphere(Vector3(-30.7169f, 82.4041f, -22.6041f), 2.80817f); 
  spatialPartition.UpdateData(keys[17], data[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintSphereData, file, true);
}

void SphereSpatialPartitionRayCastFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mBoundingSphere = Sphere(Vector3(-5.26247f, 7.17047f, -14.3424f), 2.39451f); 
  data[1].mBoundingSphere = Sphere(Vector3(-6.08262f, -38.0933f, -17.3042f), 3.91523f); 
  data[2].mBoundingSphere = Sphere(Vector3(39.3644f, 12.1685f, -27.0028f), 1.90349f); 
  data[3].mBoundingSphere = Sphere(Vector3(34.1295f, -33.515f, 1.52629f), 1.35994f); 
  data[4].mBoundingSphere = Sphere(Vector3(0.203711f, -43.8077f, -22.5967f), 3.47384f); 
  data[5].mBoundingSphere = Sphere(Vector3(-9.70535f, -47.7021f, 8.45147f), 2.01285f); 
  data[6].mBoundingSphere = Sphere(Vector3(-25.6452f, 30.7422f, -23.8555f), 0.591516f); 
  data[7].mBoundingSphere = Sphere(Vector3(-12.1264f, 1.4839f, 13.1183f), 3.43406f); 
  data[8].mBoundingSphere = Sphere(Vector3(23.9692f, 17.3361f, -14.3942f), 2.55408f); 
  data[9].mBoundingSphere = Sphere(Vector3(33.873f, -16.1128f, -6.40945f), 1.36759f); 
  data[10].mBoundingSphere = Sphere(Vector3(29.3499f, -5.89969f, -1.99338f), 2.65927f); 
  data[11].mBoundingSphere = Sphere(Vector3(-31.8262f, 28.6584f, 13.5665f), 0.919016f); 
  data[12].mBoundingSphere = Sphere(Vector3(-19.0155f, 35.5948f, -16.9283f), 1.97064f); 
  data[13].mBoundingSphere = Sphere(Vector3(30.4739f, -36.8383f, 12.8174f), 3.50759f); 
  data[14].mBoundingSphere = Sphere(Vector3(-18.6776f, -14.9198f, 20.5344f), 1.2573f); 
  data[15].mBoundingSphere = Sphere(Vector3(-5.05542f, 7.46378f, 21.6573f), 3.79299f); 
  data[16].mBoundingSphere = Sphere(Vector3(24.2839f, 27.2689f, -16.778f), 2.90849f); 
  data[17].mBoundingSphere = Sphere(Vector3(13.838f, 32.1401f, -11.3889f), 0.378826f); 
  data[18].mBoundingSphere = Sphere(Vector3(33.8874f, 22.5827f, 4.87271f), 3.82349f); 
  data[19].mBoundingSphere = Sphere(Vector3(-15.2475f, -12.8352f, 10.0995f), 1.28931f); 
  data[20].mBoundingSphere = Sphere(Vector3(-14.1748f, -11.5292f, -38.307f), 2.07675f); 
  data[21].mBoundingSphere = Sphere(Vector3(-9.41328f, 8.92825f, -19.7369f), 1.42152f); 
  data[22].mBoundingSphere = Sphere(Vector3(44.0725f, -14.7525f, -15.9679f), 2.15746f); 
  data[23].mBoundingSphere = Sphere(Vector3(-35.115f, -9.24835f, 32.8248f), 0.372912f); 
  data[24].mBoundingSphere = Sphere(Vector3(27.9334f, -33.82f, -10.2109f), 3.58471f); 
  data[25].mBoundingSphere = Sphere(Vector3(-14.5892f, 23.1187f, 32.9472f), 3.64386f); 
  data[26].mBoundingSphere = Sphere(Vector3(21.2655f, 3.21916f, -4.99672f), 3.29652f); 
  data[27].mBoundingSphere = Sphere(Vector3(5.89452f, 41.9947f, -22.1512f), 3.41435f); 
  data[28].mBoundingSphere = Sphere(Vector3(-17.9104f, -21.2869f, -17.6823f), 0.415821f); 
  data[29].mBoundingSphere = Sphere(Vector3(32.6515f, 2.70782f, -3.43741f), 3.49715f); 
  data[30].mBoundingSphere = Sphere(Vector3(12.1299f, 12.2873f, 4.10034f), 3.38965f); 
  data[31].mBoundingSphere = Sphere(Vector3(25.8288f, 26.5686f, -25.357f), 1.06467f); 
  data[32].mBoundingSphere = Sphere(Vector3(41.0294f, -9.57393f, 24.9998f), 1.50362f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(22.4641f, 2.81232f, -8.0048f));
  ray.mDirection = Vector3(Vector3(0.919881f, 0.00185213f, 0.392193f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mBoundingSphere = Sphere(Vector3(-10.1801f, -35.2866f, 29.0709f), 1.63641f); 
  data[1].mBoundingSphere = Sphere(Vector3(21.93f, -35.7074f, 26.5285f), 0.923887f); 
  data[2].mBoundingSphere = Sphere(Vector3(11.3041f, 21.8558f, 19.5054f), 2.7118f); 
  data[3].mBoundingSphere = Sphere(Vector3(-40.8889f, -15.7011f, -13.2518f), 2.04903f); 
  data[4].mBoundingSphere = Sphere(Vector3(-21.1707f, -19.7584f, -12.8416f), 1.45434f); 
  data[5].mBoundingSphere = Sphere(Vector3(8.00269f, -1.72874f, 40.7492f), 3.46909f); 
  data[6].mBoundingSphere = Sphere(Vector3(-31.4033f, 4.51078f, 32.1382f), 2.11827f); 
  data[7].mBoundingSphere = Sphere(Vector3(-25.3091f, 37.197f, -8.01228f), 3.80076f); 
  data[8].mBoundingSphere = Sphere(Vector3(2.33352f, 0.378003f, 34.1988f), 3.62391f); 
  data[9].mBoundingSphere = Sphere(Vector3(0.283248f, -11.7914f, 38.8078f), 0.905679f); 
  data[10].mBoundingSphere = Sphere(Vector3(8.17779f, -42.7253f, 19.1516f), 2.65961f); 
  data[11].mBoundingSphere = Sphere(Vector3(7.53043f, -0.825635f, -38.1977f), 3.7814f); 
  data[12].mBoundingSphere = Sphere(Vector3(15.1922f, 1.30391f, 1.89779f), 2.95975f); 
  data[13].mBoundingSphere = Sphere(Vector3(-17.1438f, -19.4944f, 16.1637f), 1.2246f); 
  data[14].mBoundingSphere = Sphere(Vector3(-17.6917f, 12.7984f, 3.78404f), 0.35656f); 
  data[15].mBoundingSphere = Sphere(Vector3(-6.50395f, 10.8237f, -31.6261f), 1.27238f); 
  data[16].mBoundingSphere = Sphere(Vector3(9.6944f, -42.9674f, 8.6256f), 0.607868f); 
  data[17].mBoundingSphere = Sphere(Vector3(14.7625f, 6.09681f, -4.14932f), 0.816034f); 
  data[18].mBoundingSphere = Sphere(Vector3(14.5681f, 7.94766f, -8.56275f), 1.59883f); 
  data[19].mBoundingSphere = Sphere(Vector3(-15.9901f, -11.7317f, 35.2988f), 3.1918f); 
  data[20].mBoundingSphere = Sphere(Vector3(-6.41748f, 35.3806f, 26.9102f), 0.720011f); 
  data[21].mBoundingSphere = Sphere(Vector3(14.1637f, 15.0526f, 44.764f), 3.94503f); 
  data[22].mBoundingSphere = Sphere(Vector3(-16.1835f, -13.0551f, -18.1517f), 0.819166f); 
  data[23].mBoundingSphere = Sphere(Vector3(30.6344f, -15.1528f, 31.2342f), 1.77012f); 
  data[24].mBoundingSphere = Sphere(Vector3(7.15115f, -26.1274f, -40.9805f), 3.32888f); 
  data[25].mBoundingSphere = Sphere(Vector3(6.04659f, 18.5985f, 24.8543f), 0.425794f); 
  data[26].mBoundingSphere = Sphere(Vector3(-30.251f, 9.76921f, -1.54535f), 3.76876f); 
  data[27].mBoundingSphere = Sphere(Vector3(42.5949f, 12.3483f, 16.4067f), 1.62458f); 
  data[28].mBoundingSphere = Sphere(Vector3(-28.2991f, -24.261f, 15.7523f), 1.02269f); 
  data[29].mBoundingSphere = Sphere(Vector3(-7.95766f, -1.38554f, 16.6857f), 0.297647f); 
  data[30].mBoundingSphere = Sphere(Vector3(-17.0471f, 18.3165f, -12.7009f), 1.39183f); 
  data[31].mBoundingSphere = Sphere(Vector3(17.435f, -14.3848f, -9.30279f), 1.23492f); 
  data[32].mBoundingSphere = Sphere(Vector3(-23.9267f, -27.5677f, -11.7977f), 2.22101f); 
  data[33].mBoundingSphere = Sphere(Vector3(36.1567f, -32.0035f, -12.1229f), 3.89957f); 
  data[34].mBoundingSphere = Sphere(Vector3(33.5726f, 11.3668f, 13.6703f), 0.701572f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-20.4009f, -4.31494f, -2.00424f));
  ray.mDirection = Vector3(Vector3(0.572562f, 0.0637757f, 0.817377f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mBoundingSphere = Sphere(Vector3(-0.445919f, 11.6948f, -32.7032f), 0.786926f); 
  data[1].mBoundingSphere = Sphere(Vector3(17.4601f, -25.9223f, 31.7268f), 2.11687f); 
  data[2].mBoundingSphere = Sphere(Vector3(21.8433f, -19.5444f, 19.2736f), 0.597082f); 
  data[3].mBoundingSphere = Sphere(Vector3(-3.07212f, -12.2667f, 17.3293f), 1.5826f); 
  data[4].mBoundingSphere = Sphere(Vector3(-14.4227f, 13.0122f, 44.6717f), 0.219019f); 
  data[5].mBoundingSphere = Sphere(Vector3(14.148f, -11.127f, -36.2296f), 2.26798f); 
  data[6].mBoundingSphere = Sphere(Vector3(27.0225f, 15.9972f, -16.4007f), 2.78127f); 
  data[7].mBoundingSphere = Sphere(Vector3(24.7825f, 5.90323f, -4.90007f), 1.12278f); 
  data[8].mBoundingSphere = Sphere(Vector3(6.37818f, 34.1742f, 26.1954f), 3.52673f); 
  data[9].mBoundingSphere = Sphere(Vector3(12.9906f, 30.8371f, 9.96358f), 0.48923f); 
  data[10].mBoundingSphere = Sphere(Vector3(36.7964f, 20.5623f, 15.5815f), 3.18716f); 
  data[11].mBoundingSphere = Sphere(Vector3(2.55531f, -32.5751f, 13.6201f), 0.813947f); 
  data[12].mBoundingSphere = Sphere(Vector3(5.6033f, -16.1635f, 10.6139f), 2.83276f); 
  data[13].mBoundingSphere = Sphere(Vector3(17.5271f, 26.7685f, 2.20915f), 1.43404f); 
  data[14].mBoundingSphere = Sphere(Vector3(5.09605f, -27.9276f, 4.15299f), 2.99616f); 
  data[15].mBoundingSphere = Sphere(Vector3(28.4436f, -7.29622f, -33.2398f), 1.83959f); 
  data[16].mBoundingSphere = Sphere(Vector3(-12.6304f, -32.3396f, -24.4699f), 3.27124f); 
  data[17].mBoundingSphere = Sphere(Vector3(8.18156f, -25.3306f, -2.73827f), 3.40971f); 
  data[18].mBoundingSphere = Sphere(Vector3(18.358f, 34.7559f, 1.5258f), 2.3879f); 
  data[19].mBoundingSphere = Sphere(Vector3(19.8032f, -40.5286f, 18.7886f), 0.224818f); 
  data[20].mBoundingSphere = Sphere(Vector3(35.8335f, 24.0187f, -19.4342f), 2.56127f); 
  data[21].mBoundingSphere = Sphere(Vector3(10.082f, -43.3666f, 4.1775f), 0.752947f); 
  data[22].mBoundingSphere = Sphere(Vector3(-36.7731f, 6.92711f, 12.9917f), 2.91533f); 
  data[23].mBoundingSphere = Sphere(Vector3(9.75581f, -42.5174f, 12.0565f), 0.611811f); 
  data[24].mBoundingSphere = Sphere(Vector3(39.5616f, -0.32241f, 5.46168f), 0.982684f); 
  data[25].mBoundingSphere = Sphere(Vector3(-15.7475f, -11.1681f, -38.4105f), 1.40505f); 
  data[26].mBoundingSphere = Sphere(Vector3(7.00256f, 33.6411f, -8.76295f), 2.27888f); 
  data[27].mBoundingSphere = Sphere(Vector3(-12.9163f, -19.372f, -41.1175f), 3.43441f); 
  data[28].mBoundingSphere = Sphere(Vector3(-10.8186f, 40.8622f, 10.7427f), 3.75113f); 
  data[29].mBoundingSphere = Sphere(Vector3(-17.1716f, -22.8053f, 34.0739f), 2.68617f); 
  data[30].mBoundingSphere = Sphere(Vector3(-24.6142f, 13.151f, 31.2181f), 0.502451f); 
  data[31].mBoundingSphere = Sphere(Vector3(-2.51661f, -24.5028f, 5.3598f), 3.49309f); 
  data[32].mBoundingSphere = Sphere(Vector3(31.6046f, -28.5634f, 19.4012f), 0.592096f); 
  data[33].mBoundingSphere = Sphere(Vector3(-35.3285f, 24.8315f, -22.895f), 1.27087f); 
  data[34].mBoundingSphere = Sphere(Vector3(-9.40968f, -20.0759f, 20.517f), 2.30799f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(7.72551f, -20.2318f, 10.0446f));
  ray.mDirection = Vector3(Vector3(-0.0458432f, 0.94515f, -0.323405f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mBoundingSphere = Sphere(Vector3(11.9981f, 22.1768f, -0.890872f), 3.11747f); 
  data[1].mBoundingSphere = Sphere(Vector3(35.7882f, -19.918f, 19.5841f), 3.97483f); 
  data[2].mBoundingSphere = Sphere(Vector3(9.78065f, -21.8236f, -18.2948f), 1.15826f); 
  data[3].mBoundingSphere = Sphere(Vector3(-25.5519f, 12.8131f, 33.1924f), 0.509177f); 
  data[4].mBoundingSphere = Sphere(Vector3(-8.82467f, 16.534f, -29.2629f), 2.67608f); 
  data[5].mBoundingSphere = Sphere(Vector3(-15.718f, -32.3172f, -13.0192f), 2.99303f); 
  data[6].mBoundingSphere = Sphere(Vector3(-34.9788f, 12.4706f, -12.2713f), 2.49795f); 
  data[7].mBoundingSphere = Sphere(Vector3(-39.5902f, -0.622552f, -11.8971f), 1.71028f); 
  data[8].mBoundingSphere = Sphere(Vector3(9.03578f, 37.9738f, 11.5741f), 3.88496f); 
  data[9].mBoundingSphere = Sphere(Vector3(-16.8597f, -35.5266f, -2.33593f), 3.16478f); 
  data[10].mBoundingSphere = Sphere(Vector3(-9.86906f, 26.2143f, -12.7206f), 3.34628f); 
  data[11].mBoundingSphere = Sphere(Vector3(21.8108f, -36.6863f, -4.80659f), 0.88121f); 
  data[12].mBoundingSphere = Sphere(Vector3(-17.2552f, 0.30774f, -19.5683f), 0.629438f); 
  data[13].mBoundingSphere = Sphere(Vector3(-13.3859f, 3.15193f, 36.7721f), 1.78381f); 
  data[14].mBoundingSphere = Sphere(Vector3(3.46566f, 20.7451f, 43.4322f), 3.48532f); 
  data[15].mBoundingSphere = Sphere(Vector3(20.551f, 9.14062f, 41.2793f), 1.74728f); 
  data[16].mBoundingSphere = Sphere(Vector3(-13.2176f, 34.6643f, 12.7408f), 0.607172f); 
  data[17].mBoundingSphere = Sphere(Vector3(-22.4218f, 10.5735f, -6.83967f), 1.52508f); 
  data[18].mBoundingSphere = Sphere(Vector3(3.48835f, -16.4111f, 39.8452f), 1.54676f); 
  data[19].mBoundingSphere = Sphere(Vector3(7.44452f, 0.562794f, 10.9396f), 2.25163f); 
  data[20].mBoundingSphere = Sphere(Vector3(21.3714f, -28.5937f, 15.1835f), 3.00289f); 
  data[21].mBoundingSphere = Sphere(Vector3(-19.93f, 4.24331f, -43.1626f), 3.31925f); 
  data[22].mBoundingSphere = Sphere(Vector3(9.97127f, -2.04815f, 9.5611f), 3.08708f); 
  data[23].mBoundingSphere = Sphere(Vector3(-6.98224f, 30.1248f, 28.3642f), 2.63097f); 
  data[24].mBoundingSphere = Sphere(Vector3(16.481f, 4.42354f, -23.0685f), 2.76306f); 
  data[25].mBoundingSphere = Sphere(Vector3(-33.1468f, -16.0842f, -11.1547f), 3.0277f); 
  data[26].mBoundingSphere = Sphere(Vector3(4.10972f, -25.1641f, -32.7969f), 2.06202f); 
  data[27].mBoundingSphere = Sphere(Vector3(18.9841f, -9.25912f, 9.87093f), 2.09043f); 
  data[28].mBoundingSphere = Sphere(Vector3(27.6627f, 25.5949f, -16.5381f), 2.47174f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(8.33722f, -1.76422f, 10.1208f));
  ray.mDirection = Vector3(Vector3(-0.823764f, 0.331458f, -0.459944f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(static_cast<int>(Application::mStatistics.mRaySphereTests))); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(41); 
  data[0].mBoundingSphere = Sphere(Vector3(20.3601f, -36.2425f, -22.6041f), 1.65195f); 
  data[1].mBoundingSphere = Sphere(Vector3(14.2928f, 25.4994f, 20.3206f), 1.45399f); 
  data[2].mBoundingSphere = Sphere(Vector3(18.8948f, 27.6272f, -24.4629f), 2.04115f); 
  data[3].mBoundingSphere = Sphere(Vector3(25.526f, -16.8766f, -3.35298f), 1.36388f); 
  data[4].mBoundingSphere = Sphere(Vector3(10.9711f, -1.49332f, 9.31393f), 0.765587f); 
  data[5].mBoundingSphere = Sphere(Vector3(28.2082f, -1.14464f, -21.1615f), 3.88078f); 
  data[6].mBoundingSphere = Sphere(Vector3(20.3447f, -27.2091f, 9.79705f), 1.49852f); 
  data[7].mBoundingSphere = Sphere(Vector3(-23.2029f, -33.231f, 28.5613f), 1.4105f); 
  data[8].mBoundingSphere = Sphere(Vector3(30.0396f, -1.20164f, 26.9304f), 0.760832f); 
  data[9].mBoundingSphere = Sphere(Vector3(12.6453f, -35.4633f, -20.7932f), 1.7177f); 
  data[10].mBoundingSphere = Sphere(Vector3(-5.93476f, 6.17424f, -6.08599f), 2.18588f); 
  data[11].mBoundingSphere = Sphere(Vector3(-11.1553f, -39.6061f, 24.4512f), 1.57715f); 
  data[12].mBoundingSphere = Sphere(Vector3(28.7663f, -17.2688f, 24.4445f), 2.75935f); 
  data[13].mBoundingSphere = Sphere(Vector3(-1.93112f, -12.2142f, 20.6977f), 0.801422f); 
  data[14].mBoundingSphere = Sphere(Vector3(7.14964f, -11.6916f, -26.3955f), 0.434028f); 
  data[15].mBoundingSphere = Sphere(Vector3(1.76929f, 14.3389f, 40.0526f), 3.44369f); 
  data[16].mBoundingSphere = Sphere(Vector3(19.5069f, 15.4388f, 15.0696f), 0.319913f); 
  data[17].mBoundingSphere = Sphere(Vector3(-28.7879f, -25.3388f, -19.8653f), 2.37421f); 
  data[18].mBoundingSphere = Sphere(Vector3(31.3668f, 3.0257f, -4.38196f), 0.448524f); 
  data[19].mBoundingSphere = Sphere(Vector3(-36.6308f, 15.0995f, 17.8878f), 0.433564f); 
  data[20].mBoundingSphere = Sphere(Vector3(37.4023f, -29.5439f, -3.455f), 2.71366f); 
  data[21].mBoundingSphere = Sphere(Vector3(-14.8956f, -41.4849f, 4.50145f), 3.55931f); 
  data[22].mBoundingSphere = Sphere(Vector3(30.7362f, -12.234f, 29.7071f), 3.38478f); 
  data[23].mBoundingSphere = Sphere(Vector3(2.10428f, 9.23576f, -46.6981f), 1.08856f); 
  data[24].mBoundingSphere = Sphere(Vector3(11.3553f, 23.1158f, -2.49295f), 1.19839f); 
  data[25].mBoundingSphere = Sphere(Vector3(22.1085f, 42.248f, -11.844f), 1.63571f); 
  data[26].mBoundingSphere = Sphere(Vector3(-32.7835f, -7.82569f, -36.394f), 0.22215f); 
  data[27].mBoundingSphere = Sphere(Vector3(-0.597686f, -19.6282f, 17.4293f), 2.94397f); 
  data[28].mBoundingSphere = Sphere(Vector3(22.0829f, 9.5644f, -18.2753f), 3.69465f); 
  data[29].mBoundingSphere = Sphere(Vector3(24.9533f, -27.264f, 31.9242f), 2.11351f); 
  data[30].mBoundingSphere = Sphere(Vector3(-0.137885f, 9.49202f, 3.92025f), 2.64025f); 
  data[31].mBoundingSphere = Sphere(Vector3(8.66421f, 10.3438f, -13.56f), 0.603113f); 
  data[32].mBoundingSphere = Sphere(Vector3(-36.7728f, 1.33328f, -23.3065f), 1.05946f); 
  data[33].mBoundingSphere = Sphere(Vector3(21.7233f, -37.0548f, 4.45847f), 0.208234f); 
  data[34].mBoundingSphere = Sphere(Vector3(-22.4963f, -41.8396f, -1.08905f), 1.693f); 
  data[35].mBoundingSphere = Sphere(Vector3(19.0155f, 6.65431f, -32.3526f), 1.41282f); 
  data[36].mBoundingSphere = Sphere(Vector3(27.1309f, -33.9609f, -22.6029f), 1.76224f); 
  data[37].mBoundingSphere = Sphere(Vector3(14.814f, -1.03425f, 24.0583f), 0.316782f); 
  data[38].mBoundingSphere = Sphere(Vector3(17.8273f, 24.3252f, -17.2362f), 2.48392f); 
  data[39].mBoundingSphere = Sphere(Vector3(-2.4091f, 5.10165f, -18.4658f), 3.13846f); 
  data[40].mBoundingSphere = Sphere(Vector3(-15.4959f, -15.8574f, -8.13514f), 1.95973f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-13.154f, 18.482f, -3.77084f));
  ray.mDirection = Vector3(Vector3(0.483217f, -0.831913f, -0.272805f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mBoundingSphere = Sphere(Vector3(-4.04294f, 4.31075f, -5.04536f), 2.61114f); 
  data[1].mBoundingSphere = Sphere(Vector3(26.0207f, -18.6986f, 35.3284f), 3.08233f); 
  data[2].mBoundingSphere = Sphere(Vector3(27.7437f, 8.79238f, -32.141f), 1.18726f); 
  data[3].mBoundingSphere = Sphere(Vector3(6.82082f, 15.6433f, -20.3578f), 3.75403f); 
  data[4].mBoundingSphere = Sphere(Vector3(11.4893f, 29.2439f, 32.6694f), 3.01785f); 
  data[5].mBoundingSphere = Sphere(Vector3(-11.051f, 27.9943f, -9.96895f), 3.51617f); 
  data[6].mBoundingSphere = Sphere(Vector3(-42.9188f, 18.2428f, -4.92167f), 2.17126f); 
  data[7].mBoundingSphere = Sphere(Vector3(17.738f, -0.420144f, -42.3802f), 3.38362f); 
  data[8].mBoundingSphere = Sphere(Vector3(15.9274f, 9.87154f, 42.7535f), 0.852565f); 
  data[9].mBoundingSphere = Sphere(Vector3(32.4431f, 5.14055f, -22.5964f), 0.343107f); 
  data[10].mBoundingSphere = Sphere(Vector3(26.6715f, -9.94645f, -25.1162f), 3.71854f); 
  data[11].mBoundingSphere = Sphere(Vector3(-14.9828f, -16.6791f, 43.394f), 1.85617f); 
  data[12].mBoundingSphere = Sphere(Vector3(0.472648f, 25.3425f, 38.3922f), 3.1671f); 
  data[13].mBoundingSphere = Sphere(Vector3(46.0045f, 10.28f, 0.565416f), 0.935948f); 
  data[14].mBoundingSphere = Sphere(Vector3(4.40658f, -24.4947f, -42.403f), 1.20245f); 
  data[15].mBoundingSphere = Sphere(Vector3(13.5241f, 5.62181f, -11.6958f), 3.00034f); 
  data[16].mBoundingSphere = Sphere(Vector3(29.4147f, 10.6238f, -36.7531f), 3.27519f); 
  data[17].mBoundingSphere = Sphere(Vector3(29.0774f, -11.5608f, 12.698f), 1.89839f); 
  data[18].mBoundingSphere = Sphere(Vector3(-16.9298f, -29.4252f, -5.85705f), 3.09636f); 
  data[19].mBoundingSphere = Sphere(Vector3(10.0955f, 19.4276f, -21.3838f), 3.91337f); 
  data[20].mBoundingSphere = Sphere(Vector3(-0.395677f, -3.37427f, -45.2419f), 0.227833f); 
  data[21].mBoundingSphere = Sphere(Vector3(-25.6738f, -13.2336f, 25.0589f), 0.939775f); 
  data[22].mBoundingSphere = Sphere(Vector3(-25.2577f, 3.67254f, -35.1527f), 1.04763f); 
  data[23].mBoundingSphere = Sphere(Vector3(-37.5702f, 25.345f, -12.9193f), 3.03594f); 
  data[24].mBoundingSphere = Sphere(Vector3(34.5957f, -11.3668f, -29.856f), 3.08871f); 
  data[25].mBoundingSphere = Sphere(Vector3(-27.8226f, -33.864f, 19.4458f), 1.15072f); 
  data[26].mBoundingSphere = Sphere(Vector3(22.4677f, -33.3982f, 22.263f), 0.75399f); 
  data[27].mBoundingSphere = Sphere(Vector3(-18.0211f, 18.9946f, 7.10832f), 0.626887f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-17.7085f, 13.3992f, 7.09386f));
  ray.mDirection = Vector3(Vector3(0.72858f, -0.418212f, -0.542467f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mBoundingSphere = Sphere(Vector3(-30.4733f, 14.5422f, 5.01615f), 1.57773f); 
  data[1].mBoundingSphere = Sphere(Vector3(28.1814f, -20.2923f, 17.2794f), 0.484011f); 
  data[2].mBoundingSphere = Sphere(Vector3(28.4303f, 11.4934f, -12.9735f), 2.23656f); 
  data[3].mBoundingSphere = Sphere(Vector3(-15.28f, 15.3748f, -4.30079f), 2.95673f); 
  data[4].mBoundingSphere = Sphere(Vector3(-2.69278f, 8.35616f, -31.7945f), 1.58724f); 
  data[5].mBoundingSphere = Sphere(Vector3(-22.1941f, 16.2312f, 40.4877f), 2.91858f); 
  data[6].mBoundingSphere = Sphere(Vector3(-4.03824f, -15.9812f, -45.5171f), 2.45435f); 
  data[7].mBoundingSphere = Sphere(Vector3(13.8423f, 19.9102f, 30.2881f), 1.20906f); 
  data[8].mBoundingSphere = Sphere(Vector3(-1.64259f, 18.4628f, 24.5256f), 1.63247f); 
  data[9].mBoundingSphere = Sphere(Vector3(4.3396f, 0.885307f, 7.92235f), 2.35299f); 
  data[10].mBoundingSphere = Sphere(Vector3(22.1418f, 20.5815f, -27.7513f), 3.73582f); 
  data[11].mBoundingSphere = Sphere(Vector3(0.64549f, 46.589f, -7.04833f), 3.94723f); 
  data[12].mBoundingSphere = Sphere(Vector3(35.4128f, 7.71385f, -27.5278f), 1.32167f); 
  data[13].mBoundingSphere = Sphere(Vector3(45.0568f, 15.0531f, -7.1592f), 1.72246f); 
  data[14].mBoundingSphere = Sphere(Vector3(-22.9066f, 16.2849f, -19.5003f), 1.091f); 
  data[15].mBoundingSphere = Sphere(Vector3(-13.1048f, -7.18364f, 8.72948f), 2.68003f); 
  data[16].mBoundingSphere = Sphere(Vector3(39.9909f, -25.7513f, -13.1818f), 1.16105f); 
  data[17].mBoundingSphere = Sphere(Vector3(-7.42298f, 13.611f, -15.8786f), 0.703543f); 
  data[18].mBoundingSphere = Sphere(Vector3(4.93122f, -9.97545f, 46.4401f), 3.01599f); 
  data[19].mBoundingSphere = Sphere(Vector3(-0.614998f, 32.4737f, 3.62548f), 2.04578f); 
  data[20].mBoundingSphere = Sphere(Vector3(14.4221f, -10.3596f, 38.2538f), 1.12985f); 
  data[21].mBoundingSphere = Sphere(Vector3(-39.4083f, 4.3169f, 8.09951f), 1.89004f); 
  data[22].mBoundingSphere = Sphere(Vector3(28.2613f, -14.5982f, -26.3753f), 3.467f); 
  data[23].mBoundingSphere = Sphere(Vector3(-7.44694f, -15.1927f, -25.4316f), 3.18728f); 
  data[24].mBoundingSphere = Sphere(Vector3(14.8773f, -17.9985f, -15.1848f), 3.05542f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(5.34996f, 3.67138f, 8.00019f));
  ray.mDirection = Vector3(Vector3(-0.628002f, -0.767062f, 0.13126f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mBoundingSphere = Sphere(Vector3(47.9253f, 2.80149f, 10.8588f), 3.93471f); 
  data[1].mBoundingSphere = Sphere(Vector3(17.8331f, -24.6588f, -1.27812f), 2.54225f); 
  data[2].mBoundingSphere = Sphere(Vector3(16.8543f, 28.8001f, -13.9899f), 1.32596f); 
  data[3].mBoundingSphere = Sphere(Vector3(13.2348f, -33.1441f, -5.84849f), 1.99406f); 
  data[4].mBoundingSphere = Sphere(Vector3(34.8327f, -5.46442f, 0.797554f), 1.11999f); 
  data[5].mBoundingSphere = Sphere(Vector3(-19.3152f, -5.52929f, -33.7463f), 1.77511f); 
  data[6].mBoundingSphere = Sphere(Vector3(-12.4757f, 26.8987f, -25.1574f), 3.85318f); 
  data[7].mBoundingSphere = Sphere(Vector3(-21.3334f, -26.2828f, 30.7674f), 1.22912f); 
  data[8].mBoundingSphere = Sphere(Vector3(-36.8011f, 11.8533f, 0.447226f), 0.599402f); 
  data[9].mBoundingSphere = Sphere(Vector3(-2.05118f, 6.31537f, 15.4519f), 2.74311f); 
  data[10].mBoundingSphere = Sphere(Vector3(-23.4946f, 24.1814f, -31.1734f), 2.85108f); 
  data[11].mBoundingSphere = Sphere(Vector3(22.4659f, 2.46751f, -10.4162f), 3.86571f); 
  data[12].mBoundingSphere = Sphere(Vector3(25.4364f, -37.8425f, 8.59398f), 0.673855f); 
  data[13].mBoundingSphere = Sphere(Vector3(31.0716f, -20.4918f, -30.2862f), 1.29963f); 
  data[14].mBoundingSphere = Sphere(Vector3(44.8019f, 1.82657f, 0.0752611f), 1.13727f); 
  data[15].mBoundingSphere = Sphere(Vector3(-30.5346f, -36.9622f, 1.64996f), 2.97042f); 
  data[16].mBoundingSphere = Sphere(Vector3(-31.6731f, -37.5126f, -8.63974f), 2.62053f); 
  data[17].mBoundingSphere = Sphere(Vector3(32.5815f, 2.14289f, 29.1287f), 2.56835f); 
  data[18].mBoundingSphere = Sphere(Vector3(35.7596f, 8.03395f, -7.91048f), 3.79253f); 
  data[19].mBoundingSphere = Sphere(Vector3(29.952f, 30.9521f, 9.88423f), 3.96753f); 
  data[20].mBoundingSphere = Sphere(Vector3(-31.7565f, -2.39768f, 36.565f), 1.6676f); 
  data[21].mBoundingSphere = Sphere(Vector3(4.84412f, 43.8909f, 15.5199f), 0.517643f); 
  data[22].mBoundingSphere = Sphere(Vector3(-43.9959f, -9.32312f, 11.0089f), 1.53621f); 
  data[23].mBoundingSphere = Sphere(Vector3(24.669f, 29.3826f, 12.6941f), 0.413385f); 
  data[24].mBoundingSphere = Sphere(Vector3(-37.3423f, 8.04792f, 26.5546f), 0.769646f); 
  data[25].mBoundingSphere = Sphere(Vector3(-11.6747f, 36.1574f, 21.1536f), 3.13602f); 
  data[26].mBoundingSphere = Sphere(Vector3(31.0433f, -7.96942f, -30.3469f), 3.50168f); 
  data[27].mBoundingSphere = Sphere(Vector3(-44.93f, 0.344634f, -14.4804f), 2.97227f); 
  data[28].mBoundingSphere = Sphere(Vector3(35.1313f, -11.5203f, -3.14696f), 1.58248f); 
  data[29].mBoundingSphere = Sphere(Vector3(8.71601f, 15.1524f, -36.5551f), 0.606592f); 
  data[30].mBoundingSphere = Sphere(Vector3(-1.74832f, 7.94377f, 6.74009f), 1.61066f); 
  data[31].mBoundingSphere = Sphere(Vector3(-18.9534f, 38.1109f, 9.65558f), 3.79021f); 
  data[32].mBoundingSphere = Sphere(Vector3(20.3391f, -2.20235f, -6.17401f), 3.03779f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(8.82474f, 0.871751f, -18.8326f));
  ray.mDirection = Vector3(Vector3(0.816267f, 0.0798447f, 0.57213f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mBoundingSphere = Sphere(Vector3(2.84311f, -30.786f, 20.7743f), 1.06015f); 
  data[1].mBoundingSphere = Sphere(Vector3(-18.7902f, -3.24567f, 11.3477f), 3.20688f); 
  data[2].mBoundingSphere = Sphere(Vector3(-3.89365f, 15.726f, -5.43459f), 1.78926f); 
  data[3].mBoundingSphere = Sphere(Vector3(-9.63183f, -4.88088f, 12.7162f), 1.47243f); 
  data[4].mBoundingSphere = Sphere(Vector3(38.3662f, 4.57914f, -6.39571f), 3.46723f); 
  data[5].mBoundingSphere = Sphere(Vector3(-1.79677f, 28.0372f, -14.8336f), 3.37248f); 
  data[6].mBoundingSphere = Sphere(Vector3(8.69318f, 11.4787f, -13.9894f), 0.722098f); 
  data[7].mBoundingSphere = Sphere(Vector3(3.71092f, -20.2927f, -34.0867f), 2.78011f); 
  data[8].mBoundingSphere = Sphere(Vector3(-38.8566f, 21.6989f, 3.39314f), 3.22694f); 
  data[9].mBoundingSphere = Sphere(Vector3(14.7575f, -41.3119f, -14.7755f), 1.8818f); 
  data[10].mBoundingSphere = Sphere(Vector3(37.3901f, -30.5488f, 1.29865f), 0.504422f); 
  data[11].mBoundingSphere = Sphere(Vector3(-24.3969f, -9.17018f, 7.59125f), 2.81977f); 
  data[12].mBoundingSphere = Sphere(Vector3(-8.3286f, 22.3169f, -22.1757f), 3.2085f); 
  data[13].mBoundingSphere = Sphere(Vector3(-16.5798f, 0.699844f, 14.9883f), 3.60999f); 
  data[14].mBoundingSphere = Sphere(Vector3(13.9254f, 10.4383f, -44.2027f), 1.27806f); 
  data[15].mBoundingSphere = Sphere(Vector3(-7.85282f, -7.62173f, 39.3164f), 2.13555f); 
  data[16].mBoundingSphere = Sphere(Vector3(45.4399f, 9.98933f, 12.9063f), 2.36957f); 
  data[17].mBoundingSphere = Sphere(Vector3(-38.6143f, 12.5519f, -13.2583f), 0.61413f); 
  data[18].mBoundingSphere = Sphere(Vector3(-11.7338f, 5.11014f, 33.1919f), 2.36064f); 
  data[19].mBoundingSphere = Sphere(Vector3(-45.6435f, -12.0774f, 4.21989f), 2.94165f); 
  data[20].mBoundingSphere = Sphere(Vector3(37.5451f, 23.2797f, -23.1846f), 1.93179f); 
  data[21].mBoundingSphere = Sphere(Vector3(-3.72062f, -16.4391f, 14.4124f), 1.10921f); 
  data[22].mBoundingSphere = Sphere(Vector3(-1.16929f, -16.2439f, 14.0901f), 0.821021f); 
  data[23].mBoundingSphere = Sphere(Vector3(11.8471f, -14.7714f, -1.94668f), 3.89725f); 
  data[24].mBoundingSphere = Sphere(Vector3(4.45317f, 19.8793f, 41.0476f), 3.95524f); 
  data[25].mBoundingSphere = Sphere(Vector3(-22.2532f, 24.3279f, -22.5982f), 1.44227f); 
  data[26].mBoundingSphere = Sphere(Vector3(2.41719f, 5.82199f, -18.1708f), 1.45457f); 
  data[27].mBoundingSphere = Sphere(Vector3(1.70572f, 8.96752f, -11.0835f), 2.48114f); 
  data[28].mBoundingSphere = Sphere(Vector3(-29.3258f, -8.57781f, 19.2047f), 2.31379f); 
  data[29].mBoundingSphere = Sphere(Vector3(-10.6131f, -16.004f, -30.9659f), 3.66786f); 
  data[30].mBoundingSphere = Sphere(Vector3(-38.1858f, 27.2355f, -0.310624f), 3.95732f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-13.9257f, -0.472823f, 16.6313f));
  ray.mDirection = Vector3(Vector3(-0.867252f, -0.0830694f, -0.49089f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(34); 
  data[0].mBoundingSphere = Sphere(Vector3(-18.2116f, 3.31502f, -32.605f), 1.77279f); 
  data[1].mBoundingSphere = Sphere(Vector3(7.51186f, -10.8624f, -18.0162f), 0.820789f); 
  data[2].mBoundingSphere = Sphere(Vector3(24.9753f, -16.6921f, -19.528f), 2.35264f); 
  data[3].mBoundingSphere = Sphere(Vector3(-28.4768f, -25.1134f, 17.8909f), 0.346702f); 
  data[4].mBoundingSphere = Sphere(Vector3(15.3715f, -14.4429f, -8.1174f), 2.995f); 
  data[5].mBoundingSphere = Sphere(Vector3(-24.8898f, 13.2308f, 15.8695f), 2.3516f); 
  data[6].mBoundingSphere = Sphere(Vector3(4.4304f, -12.051f, 42.0872f), 2.97819f); 
  data[7].mBoundingSphere = Sphere(Vector3(-7.99845f, 39.1589f, -8.31879f), 3.30395f); 
  data[8].mBoundingSphere = Sphere(Vector3(-22.1707f, -16.3647f, 31.4144f), 3.92068f); 
  data[9].mBoundingSphere = Sphere(Vector3(-38.3242f, 10.8756f, 14.3184f), 1.12927f); 
  data[10].mBoundingSphere = Sphere(Vector3(-12.3246f, -41.9637f, -9.95031f), 1.95869f); 
  data[11].mBoundingSphere = Sphere(Vector3(31.6856f, 6.99113f, 27.1802f), 3.7749f); 
  data[12].mBoundingSphere = Sphere(Vector3(5.05471f, 28.8916f, 34.2073f), 1.29824f); 
  data[13].mBoundingSphere = Sphere(Vector3(-30.1318f, -38.689f, -2.00259f), 2.14981f); 
  data[14].mBoundingSphere = Sphere(Vector3(10.8489f, 31.8466f, 7.23505f), 3.54261f); 
  data[15].mBoundingSphere = Sphere(Vector3(-8.4126f, 41.7993f, -12.823f), 2.43962f); 
  data[16].mBoundingSphere = Sphere(Vector3(40.1142f, 1.18106f, -22.5789f), 3.19459f); 
  data[17].mBoundingSphere = Sphere(Vector3(-2.8246f, 28.5794f, 5.54228f), 3.79624f); 
  data[18].mBoundingSphere = Sphere(Vector3(17.2891f, -18.861f, -32.0477f), 0.733811f); 
  data[19].mBoundingSphere = Sphere(Vector3(-17.0513f, 33.5569f, 21.2886f), 1.2028f); 
  data[20].mBoundingSphere = Sphere(Vector3(-7.8137f, -0.394388f, 6.70954f), 2.86964f); 
  data[21].mBoundingSphere = Sphere(Vector3(15.0777f, -45.2241f, -13.4589f), 2.11374f); 
  data[22].mBoundingSphere = Sphere(Vector3(20.3289f, -13.0188f, 23.3182f), 2.36749f); 
  data[23].mBoundingSphere = Sphere(Vector3(37.7145f, -3.92289f, -24.4771f), 3.66914f); 
  data[24].mBoundingSphere = Sphere(Vector3(7.37785f, 34.2252f, -4.18496f), 2.05193f); 
  data[25].mBoundingSphere = Sphere(Vector3(-20.5254f, -8.68722f, -17.2332f), 2.68478f); 
  data[26].mBoundingSphere = Sphere(Vector3(0.462132f, -47.0231f, -14.4475f), 3.18415f); 
  data[27].mBoundingSphere = Sphere(Vector3(6.68453f, 34.3876f, 11.8794f), 0.417212f); 
  data[28].mBoundingSphere = Sphere(Vector3(4.65827f, 7.52968f, -11.8173f), 3.81352f); 
  data[29].mBoundingSphere = Sphere(Vector3(-17.6992f, 37.1849f, 15.1531f), 2.10354f); 
  data[30].mBoundingSphere = Sphere(Vector3(-12.86f, -30.3398f, 36.9646f), 3.30557f); 
  data[31].mBoundingSphere = Sphere(Vector3(3.42609f, -45.1182f, 13.3916f), 3.67586f); 
  data[32].mBoundingSphere = Sphere(Vector3(-13.8433f, -4.62937f, -22.9573f), 1.64418f); 
  data[33].mBoundingSphere = Sphere(Vector3(38.7098f, -21.379f, 18.6393f), 3.65163f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(4.66264f, -20.0209f, -0.0194476f));
  ray.mDirection = Vector3(Vector3(0.781112f, 0.29437f, -0.550645f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(41); 
  data[0].mBoundingSphere = Sphere(Vector3(-5.12207f, 14.7343f, 5.2238f), 3.32331f); 
  data[1].mBoundingSphere = Sphere(Vector3(-17.9455f, 3.67355f, -32.9498f), 3.29838f); 
  data[2].mBoundingSphere = Sphere(Vector3(17.785f, -17.9469f, -0.415677f), 2.7394f); 
  data[3].mBoundingSphere = Sphere(Vector3(-31.3257f, -36.6996f, 2.64424f), 3.81016f); 
  data[4].mBoundingSphere = Sphere(Vector3(-13.7984f, 38.0614f, -22.6103f), 0.565539f); 
  data[5].mBoundingSphere = Sphere(Vector3(0.503292f, -45.8439f, 1.17709f), 3.43882f); 
  data[6].mBoundingSphere = Sphere(Vector3(30.244f, 13.0784f, -37.2715f), 2.995f); 
  data[7].mBoundingSphere = Sphere(Vector3(15.5669f, -3.94864f, -6.20726f), 3.69604f); 
  data[8].mBoundingSphere = Sphere(Vector3(0.156203f, -23.7837f, 43.102f), 1.91903f); 
  data[9].mBoundingSphere = Sphere(Vector3(-20.2832f, 13.8935f, 6.42341f), 0.973058f); 
  data[10].mBoundingSphere = Sphere(Vector3(-4.58642f, -24.0996f, 7.93396f), 1.95799f); 
  data[11].mBoundingSphere = Sphere(Vector3(22.5256f, 33.9037f, 0.709426f), 2.40842f); 
  data[12].mBoundingSphere = Sphere(Vector3(-10.9581f, 17.0804f, 37.0865f), 2.36702f); 
  data[13].mBoundingSphere = Sphere(Vector3(5.94642f, -23.8502f, 4.22675f), 0.430897f); 
  data[14].mBoundingSphere = Sphere(Vector3(12.9881f, -13.2937f, 20.2174f), 2.21556f); 
  data[15].mBoundingSphere = Sphere(Vector3(-9.85698f, -4.10741f, -5.11222f), 0.977697f); 
  data[16].mBoundingSphere = Sphere(Vector3(18.8481f, -3.31339f, 11.752f), 2.24409f); 
  data[17].mBoundingSphere = Sphere(Vector3(0.45611f, 22.3839f, -13.438f), 2.78718f); 
  data[18].mBoundingSphere = Sphere(Vector3(10.9526f, -24.3011f, -27.8133f), 3.39348f); 
  data[19].mBoundingSphere = Sphere(Vector3(-26.6952f, -9.93488f, 23.9877f), 3.76678f); 
  data[20].mBoundingSphere = Sphere(Vector3(13.2199f, -14.8851f, 15.1088f), 2.01099f); 
  data[21].mBoundingSphere = Sphere(Vector3(5.6591f, 33.2083f, 19.8729f), 0.236183f); 
  data[22].mBoundingSphere = Sphere(Vector3(1.67906f, 13.8925f, -31.6147f), 0.730912f); 
  data[23].mBoundingSphere = Sphere(Vector3(-6.34461f, -13.9448f, 21.9789f), 1.8455f); 
  data[24].mBoundingSphere = Sphere(Vector3(-5.92323f, -14.4107f, 45.9409f), 0.829603f); 
  data[25].mBoundingSphere = Sphere(Vector3(-23.6614f, 19.1699f, -23.2847f), 1.36771f); 
  data[26].mBoundingSphere = Sphere(Vector3(-2.04953f, 6.14148f, 26.3861f), 2.65625f); 
  data[27].mBoundingSphere = Sphere(Vector3(4.51746f, -30.3231f, 4.47333f), 1.96692f); 
  data[28].mBoundingSphere = Sphere(Vector3(-13.3263f, -27.7915f, -11.0801f), 2.73824f); 
  data[29].mBoundingSphere = Sphere(Vector3(5.87463f, -31.5097f, -24.563f), 3.14124f); 
  data[30].mBoundingSphere = Sphere(Vector3(17.802f, -24.1253f, 26.1398f), 2.76132f); 
  data[31].mBoundingSphere = Sphere(Vector3(29.0863f, 16.4593f, 14.0113f), 2.34081f); 
  data[32].mBoundingSphere = Sphere(Vector3(6.69676f, -42.0689f, 18.3193f), 3.82581f); 
  data[33].mBoundingSphere = Sphere(Vector3(29.6727f, -23.932f, 4.05626f), 0.860915f); 
  data[34].mBoundingSphere = Sphere(Vector3(-29.4939f, 6.99859f, 0.79494f), 1.17635f); 
  data[35].mBoundingSphere = Sphere(Vector3(-19.8051f, 14.5541f, 7.44805f), 2.60975f); 
  data[36].mBoundingSphere = Sphere(Vector3(-3.88986f, -43.6274f, -20.3582f), 2.20791f); 
  data[37].mBoundingSphere = Sphere(Vector3(12.8664f, -20.6643f, 26.2281f), 1.95127f); 
  data[38].mBoundingSphere = Sphere(Vector3(-31.4125f, -23.8916f, 15.7738f), 3.14576f); 
  data[39].mBoundingSphere = Sphere(Vector3(3.09256f, 6.88275f, 10.8226f), 2.42362f); 
  data[40].mBoundingSphere = Sphere(Vector3(-1.50983f, 13.7892f, -4.70796f), 0.401093f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-19.1786f, 14.8321f, 3.92847f));
  ray.mDirection = Vector3(Vector3(-0.177408f, -0.201655f, 0.963256f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mBoundingSphere = Sphere(Vector3(-11.8452f, 29.0876f, -38.3061f), 2.78869f); 
  data[1].mBoundingSphere = Sphere(Vector3(-36.1471f, -16.0483f, 20.0103f), 1.33384f); 
  data[2].mBoundingSphere = Sphere(Vector3(-10.6562f, 20.4635f, 38.5231f), 1.05308f); 
  data[3].mBoundingSphere = Sphere(Vector3(-9.62846f, -34.9546f, -2.96531f), 2.62262f); 
  data[4].mBoundingSphere = Sphere(Vector3(-4.71314f, 12.171f, 15.6328f), 1.46976f); 
  data[5].mBoundingSphere = Sphere(Vector3(-10.3813f, 19.5587f, 4.74158f), 0.261464f); 
  data[6].mBoundingSphere = Sphere(Vector3(13.3568f, 44.1748f, 18.4066f), 3.91337f); 
  data[7].mBoundingSphere = Sphere(Vector3(-34.4858f, 10.3821f, -17.67f), 1.03847f); 
  data[8].mBoundingSphere = Sphere(Vector3(10.9582f, -13.9732f, 35.9786f), 2.25697f); 
  data[9].mBoundingSphere = Sphere(Vector3(-29.2587f, -31.5223f, 12.2604f), 2.48102f); 
  data[10].mBoundingSphere = Sphere(Vector3(8.92966f, -41.6175f, 9.25643f), 2.19875f); 
  data[11].mBoundingSphere = Sphere(Vector3(-0.547147f, 9.54059f, -14.0551f), 0.471139f); 
  data[12].mBoundingSphere = Sphere(Vector3(14.1973f, 6.6038f, 30.6359f), 3.4648f); 
  data[13].mBoundingSphere = Sphere(Vector3(18.8156f, 5.67044f, -20.0148f), 3.03571f); 
  data[14].mBoundingSphere = Sphere(Vector3(-24.8784f, 23.2188f, -18.1422f), 3.38617f); 
  data[15].mBoundingSphere = Sphere(Vector3(-17.2681f, -11.5434f, 27.7047f), 0.910434f); 
  data[16].mBoundingSphere = Sphere(Vector3(18.2585f, 16.0057f, -22.1532f), 2.85108f); 
  data[17].mBoundingSphere = Sphere(Vector3(21.6399f, 15.6957f, -30.2832f), 1.28073f); 
  data[18].mBoundingSphere = Sphere(Vector3(-21.3785f, -37.7412f, -4.66654f), 3.23297f); 
  data[19].mBoundingSphere = Sphere(Vector3(1.21769f, 3.36593f, 14.2897f), 3.0959f); 
  data[20].mBoundingSphere = Sphere(Vector3(19.2406f, -34.5439f, 13.8945f), 1.89189f); 
  data[21].mBoundingSphere = Sphere(Vector3(37.2174f, -28.4364f, -10.4127f), 1.35878f); 
  data[22].mBoundingSphere = Sphere(Vector3(-14.3855f, 38.6599f, -20.782f), 1.45271f); 
  data[23].mBoundingSphere = Sphere(Vector3(-4.76498f, -12.4321f, 40.6727f), 1.39148f); 
  data[24].mBoundingSphere = Sphere(Vector3(-23.7931f, 30.107f, -6.39579f), 2.45678f); 
  data[25].mBoundingSphere = Sphere(Vector3(-0.439291f, -17.2537f, 6.76583f), 1.27423f); 
  data[26].mBoundingSphere = Sphere(Vector3(-24.5919f, -11.2871f, -9.34737f), 2.1897f); 
  data[27].mBoundingSphere = Sphere(Vector3(0.220056f, 1.95938f, 26.2561f), 3.26104f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(1.08862f, 3.95058f, -6.23842f));
  ray.mDirection = Vector3(Vector3(0.0348851f, -0.00183792f, 0.99939f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(34); 
  data[0].mBoundingSphere = Sphere(Vector3(26.1902f, -18.2923f, -37.7076f), 1.72629f); 
  data[1].mBoundingSphere = Sphere(Vector3(21.1661f, -8.31683f, 6.20795f), 1.91474f); 
  data[2].mBoundingSphere = Sphere(Vector3(20.5359f, -17.1161f, -37.7682f), 3.67296f); 
  data[3].mBoundingSphere = Sphere(Vector3(20.9547f, -14.0573f, 33.1707f), 3.109f); 
  data[4].mBoundingSphere = Sphere(Vector3(-15.1033f, -38.737f, -15.8889f), 3.45007f); 
  data[5].mBoundingSphere = Sphere(Vector3(3.77994f, 30.8284f, 8.74762f), 1.3713f); 
  data[6].mBoundingSphere = Sphere(Vector3(-2.07442f, 5.68461f, 25.2835f), 2.57554f); 
  data[7].mBoundingSphere = Sphere(Vector3(21.4396f, 40.8171f, 3.77082f), 1.18946f); 
  data[8].mBoundingSphere = Sphere(Vector3(-9.75218f, 18.2465f, 38.895f), 3.84959f); 
  data[9].mBoundingSphere = Sphere(Vector3(0.845472f, -35.4795f, 7.07134f), 2.64118f); 
  data[10].mBoundingSphere = Sphere(Vector3(-10.1478f, -4.12394f, 10.597f), 0.563799f); 
  data[11].mBoundingSphere = Sphere(Vector3(-33.2134f, 15.36f, 1.42496f), 1.29001f); 
  data[12].mBoundingSphere = Sphere(Vector3(0.399539f, -10, 16.1174f), 2.2848f); 
  data[13].mBoundingSphere = Sphere(Vector3(31.3201f, -22.3614f, 31.1885f), 1.85733f); 
  data[14].mBoundingSphere = Sphere(Vector3(-15.8478f, 2.59056f, -5.7787f), 2.21197f); 
  data[15].mBoundingSphere = Sphere(Vector3(45.8341f, 7.37049f, -1.92135f), 3.86733f); 
  data[16].mBoundingSphere = Sphere(Vector3(32.9417f, 16.8641f, -33.326f), 3.91221f); 
  data[17].mBoundingSphere = Sphere(Vector3(6.9149f, -6.58764f, -3.87487f), 1.0685f); 
  data[18].mBoundingSphere = Sphere(Vector3(6.60302f, -11.6282f, -17.8587f), 1.02756f); 
  data[19].mBoundingSphere = Sphere(Vector3(17.7153f, -8.56892f, 20.0287f), 2.23041f); 
  data[20].mBoundingSphere = Sphere(Vector3(15.9038f, 28.0574f, 8.64419f), 1.15965f); 
  data[21].mBoundingSphere = Sphere(Vector3(-6.83593f, 12.1385f, 37.0934f), 1.1135f); 
  data[22].mBoundingSphere = Sphere(Vector3(7.3791f, 0.902254f, 16.1006f), 2.72757f); 
  data[23].mBoundingSphere = Sphere(Vector3(-1.31385f, 1.50618f, -16.2191f), 2.01644f); 
  data[24].mBoundingSphere = Sphere(Vector3(-9.15292f, 17.8431f, -0.793173f), 0.801654f); 
  data[25].mBoundingSphere = Sphere(Vector3(23.3585f, -4.57896f, 6.95096f), 2.09623f); 
  data[26].mBoundingSphere = Sphere(Vector3(-9.62779f, -11.6909f, 32.0451f), 2.91266f); 
  data[27].mBoundingSphere = Sphere(Vector3(-40.7228f, 17.8932f, -17.8221f), 2.66518f); 
  data[28].mBoundingSphere = Sphere(Vector3(17.8896f, -18.5437f, -18.9852f), 1.63421f); 
  data[29].mBoundingSphere = Sphere(Vector3(15.7155f, 40.2048f, -20.0957f), 3.86118f); 
  data[30].mBoundingSphere = Sphere(Vector3(26.7216f, 34.6998f, 3.31756f), 2.74265f); 
  data[31].mBoundingSphere = Sphere(Vector3(-2.43109f, -44.6367f, -14.3909f), 1.69416f); 
  data[32].mBoundingSphere = Sphere(Vector3(8.12057f, -31.0931f, 16.8175f), 2.54967f); 
  data[33].mBoundingSphere = Sphere(Vector3(-6.55948f, -25.2842f, -33.9333f), 2.6675f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-5.29611f, -7.9267f, 15.9779f));
  ray.mDirection = Vector3(Vector3(0.88704f, -0.309458f, 0.342631f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mBoundingSphere = Sphere(Vector3(-12.8376f, -17.8773f, 2.33504f), 2.76886f); 
  data[1].mBoundingSphere = Sphere(Vector3(43.3232f, 11.2016f, 1.28559f), 0.851405f); 
  data[2].mBoundingSphere = Sphere(Vector3(13.7126f, 14.2906f, -41.5836f), 1.06653f); 
  data[3].mBoundingSphere = Sphere(Vector3(19.6266f, 4.27519f, 33.3593f), 2.76155f); 
  data[4].mBoundingSphere = Sphere(Vector3(-14.2988f, 27.7829f, -2.71748f), 3.52568f); 
  data[5].mBoundingSphere = Sphere(Vector3(-29.8406f, -10.6839f, 10.9539f), 3.05739f); 
  data[6].mBoundingSphere = Sphere(Vector3(21.9822f, 32.2741f, 18.4839f), 2.91243f); 
  data[7].mBoundingSphere = Sphere(Vector3(-29.3177f, -22.8265f, 4.74524f), 3.20131f); 
  data[8].mBoundingSphere = Sphere(Vector3(-5.27046f, 41.5656f, -9.92234f), 3.00579f); 
  data[9].mBoundingSphere = Sphere(Vector3(15.8641f, -35.9317f, 30.2345f), 1.7816f); 
  data[10].mBoundingSphere = Sphere(Vector3(-33.8339f, -31.2267f, -2.05211f), 0.918204f); 
  data[11].mBoundingSphere = Sphere(Vector3(14.9596f, 5.70988f, -47.3027f), 2.72178f); 
  data[12].mBoundingSphere = Sphere(Vector3(8.53503f, 7.68997f, 14.0148f), 2.19249f); 
  data[13].mBoundingSphere = Sphere(Vector3(9.14175f, 19.4994f, -33.0178f), 3.06377f); 
  data[14].mBoundingSphere = Sphere(Vector3(-36.5469f, 11.8566f, -3.30596f), 0.272018f); 
  data[15].mBoundingSphere = Sphere(Vector3(29.7792f, -30.1773f, -7.10134f), 2.12662f); 
  data[16].mBoundingSphere = Sphere(Vector3(44.62f, -0.0128297f, 16.9916f), 3.94735f); 
  data[17].mBoundingSphere = Sphere(Vector3(-7.20088f, -11.4546f, -20.7486f), 1.64f); 
  data[18].mBoundingSphere = Sphere(Vector3(3.70895f, -2.92045f, 47.2979f), 2.57704f); 
  data[19].mBoundingSphere = Sphere(Vector3(40.1728f, 1.35244f, -13.0168f), 0.653096f); 
  data[20].mBoundingSphere = Sphere(Vector3(16.0688f, 34.8293f, 24.0051f), 2.08417f); 
  data[21].mBoundingSphere = Sphere(Vector3(-11.171f, -3.98994f, 30.6404f), 0.417908f); 
  data[22].mBoundingSphere = Sphere(Vector3(23.9239f, 26.8854f, 26.6988f), 2.52683f); 
  data[23].mBoundingSphere = Sphere(Vector3(-19.6981f, -0.026439f, -37.8295f), 0.248012f); 
  data[24].mBoundingSphere = Sphere(Vector3(-12.2077f, -23.5199f, 3.81149f), 0.844563f); 
  data[25].mBoundingSphere = Sphere(Vector3(24.2599f, 10.9075f, 5.15728f), 3.35184f); 
  data[26].mBoundingSphere = Sphere(Vector3(16.0121f, -8.80727f, -45.6382f), 3.57996f); 
  data[27].mBoundingSphere = Sphere(Vector3(27.2739f, 28.0766f, 4.19659f), 2.24166f); 
  data[28].mBoundingSphere = Sphere(Vector3(7.11509f, -7.76798f, -19.6378f), 3.97333f); 
  data[29].mBoundingSphere = Sphere(Vector3(30.208f, -12.6319f, 10.2062f), 0.717112f); 
  data[30].mBoundingSphere = Sphere(Vector3(11.9044f, 29.5816f, -0.487603f), 3.23541f); 
  data[31].mBoundingSphere = Sphere(Vector3(39.8499f, 0.248358f, 20.1011f), 0.223542f); 
  data[32].mBoundingSphere = Sphere(Vector3(16.6383f, 24.8561f, 30.9692f), 3.79241f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-2.10337f, 5.95835f, -4.86698f));
  ray.mDirection = Vector3(Vector3(0.525232f, 0.00438123f, 0.850948f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(41); 
  data[0].mBoundingSphere = Sphere(Vector3(19.956f, 20.2228f, 17.3104f), 2.81212f); 
  data[1].mBoundingSphere = Sphere(Vector3(0.189103f, 32.6018f, -29.6915f), 0.683944f); 
  data[2].mBoundingSphere = Sphere(Vector3(20.5035f, 34.4422f, -2.85377f), 1.19619f); 
  data[3].mBoundingSphere = Sphere(Vector3(-30.9705f, 19.232f, 12.4675f), 1.61867f); 
  data[4].mBoundingSphere = Sphere(Vector3(-11.7052f, -5.86962f, 36.301f), 2.1853f); 
  data[5].mBoundingSphere = Sphere(Vector3(12.6894f, 15.1139f, -31.1865f), 0.581195f); 
  data[6].mBoundingSphere = Sphere(Vector3(-17.6209f, -4.12774f, -6.19707f), 3.36077f); 
  data[7].mBoundingSphere = Sphere(Vector3(17.1666f, -13.4532f, -38.6389f), 2.63642f); 
  data[8].mBoundingSphere = Sphere(Vector3(-25.2726f, -5.17849f, 3.67162f), 1.28734f); 
  data[9].mBoundingSphere = Sphere(Vector3(-1.99471f, -24.9077f, -29.192f), 0.825776f); 
  data[10].mBoundingSphere = Sphere(Vector3(14.9148f, -22.9417f, -3.02893f), 0.969231f); 
  data[11].mBoundingSphere = Sphere(Vector3(5.6729f, 6.88721f, 29.6046f), 1.06073f); 
  data[12].mBoundingSphere = Sphere(Vector3(-18.1433f, -36.57f, -2.40495f), 2.98016f); 
  data[13].mBoundingSphere = Sphere(Vector3(-18.9579f, 8.93563f, -22.836f), 1.89108f); 
  data[14].mBoundingSphere = Sphere(Vector3(13.391f, -6.02691f, 40.3696f), 2.46861f); 
  data[15].mBoundingSphere = Sphere(Vector3(11.6745f, -15.4769f, -1.09212f), 1.34729f); 
  data[16].mBoundingSphere = Sphere(Vector3(-27.9531f, -12.8039f, -14.5663f), 3.83892f); 
  data[17].mBoundingSphere = Sphere(Vector3(-1.3533f, 11.7749f, -21.2323f), 0.458498f); 
  data[18].mBoundingSphere = Sphere(Vector3(-15.1687f, -12.4735f, -23.7474f), 1.70634f); 
  data[19].mBoundingSphere = Sphere(Vector3(-15.8751f, 3.74127f, -40.7905f), 3.44636f); 
  data[20].mBoundingSphere = Sphere(Vector3(2.87338f, -16.2947f, 39.8045f), 0.327683f); 
  data[21].mBoundingSphere = Sphere(Vector3(32.8164f, 22.2343f, 8.38472f), 3.19169f); 
  data[22].mBoundingSphere = Sphere(Vector3(-1.44154f, -42.5165f, 21.3343f), 3.1068f); 
  data[23].mBoundingSphere = Sphere(Vector3(-1.01545f, 25.9777f, -18.4399f), 1.37837f); 
  data[24].mBoundingSphere = Sphere(Vector3(36.03f, 2.32806f, -7.51488f), 2.57449f); 
  data[25].mBoundingSphere = Sphere(Vector3(-10.4328f, -18.4756f, 15.9929f), 2.5114f); 
  data[26].mBoundingSphere = Sphere(Vector3(-20.7747f, 13.6296f, 35.7023f), 1.24489f); 
  data[27].mBoundingSphere = Sphere(Vector3(-36.8989f, 1.04745f, -26.1288f), 0.808844f); 
  data[28].mBoundingSphere = Sphere(Vector3(2.06214f, 17.1727f, -27.7684f), 2.54538f); 
  data[29].mBoundingSphere = Sphere(Vector3(10.822f, -44.3831f, 18.9397f), 1.77105f); 
  data[30].mBoundingSphere = Sphere(Vector3(23.6926f, -1.44879f, 9.05867f), 2.22217f); 
  data[31].mBoundingSphere = Sphere(Vector3(-46.8816f, -14.5464f, 4.93079f), 2.42292f); 
  data[32].mBoundingSphere = Sphere(Vector3(11.1025f, -43.5332f, 17.2969f), 3.02063f); 
  data[33].mBoundingSphere = Sphere(Vector3(-17.9656f, 9.87508f, 32.2347f), 2.43173f); 
  data[34].mBoundingSphere = Sphere(Vector3(-16.0846f, 28.4591f, 3.29185f), 3.24109f); 
  data[35].mBoundingSphere = Sphere(Vector3(43.5026f, 2.45927f, 15.1529f), 0.500595f); 
  data[36].mBoundingSphere = Sphere(Vector3(-41.4301f, -12.3861f, 21.8823f), 0.968883f); 
  data[37].mBoundingSphere = Sphere(Vector3(17.8525f, -37.5907f, -10.7074f), 2.63642f); 
  data[38].mBoundingSphere = Sphere(Vector3(19.7201f, -29.5334f, -27.9399f), 2.58029f); 
  data[39].mBoundingSphere = Sphere(Vector3(14.3293f, -26.9531f, 2.42885f), 0.492129f); 
  data[40].mBoundingSphere = Sphere(Vector3(22.1981f, -0.708956f, -18.0631f), 1.32259f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-8.97955f, -20.7954f, -10.4117f));
  ray.mDirection = Vector3(Vector3(-0.0464727f, 0.0167394f, 0.998779f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(14.6327f, 33.7717f, 10.0722f), 0.466732f); 
  data[1].mBoundingSphere = Sphere(Vector3(5.1061f, 41.3816f, -7.9338f), 1.09077f); 
  data[2].mBoundingSphere = Sphere(Vector3(-15.6634f, 43.8079f, -17.9538f), 2.60349f); 
  data[3].mBoundingSphere = Sphere(Vector3(19.7003f, -29.1998f, 28.4508f), 0.83401f); 
  data[4].mBoundingSphere = Sphere(Vector3(3.74086f, 11.3801f, -3.7105f), 0.476241f); 
  data[5].mBoundingSphere = Sphere(Vector3(41.7832f, 11.1289f, 14.2577f), 1.39125f); 
  data[6].mBoundingSphere = Sphere(Vector3(40.1171f, 15.1801f, 1.79626f), 0.682437f); 
  data[7].mBoundingSphere = Sphere(Vector3(-11.4523f, -6.56641f, 38.4704f), 3.06134f); 
  data[8].mBoundingSphere = Sphere(Vector3(-9.90749f, 18.5285f, -34.2086f), 3.52046f); 
  data[9].mBoundingSphere = Sphere(Vector3(18.1399f, -21.8429f, -30.5156f), 3.23634f); 
  data[10].mBoundingSphere = Sphere(Vector3(31.0197f, -1.83714f, 7.92441f), 3.71715f); 
  data[11].mBoundingSphere = Sphere(Vector3(-5.14342f, -35.8198f, 4.73246f), 2.11618f); 
  data[12].mBoundingSphere = Sphere(Vector3(42.2879f, -13.939f, 17.4009f), 1.13008f); 
  data[13].mBoundingSphere = Sphere(Vector3(-10.4113f, 2.91787f, -15.1938f), 3.19841f); 
  data[14].mBoundingSphere = Sphere(Vector3(-26.0001f, -9.03421f, 23.1463f), 2.42535f); 
  data[15].mBoundingSphere = Sphere(Vector3(11.4688f, -23.7367f, 40.5161f), 0.942674f); 
  data[16].mBoundingSphere = Sphere(Vector3(2.50205f, 40.5658f, -3.19382f), 2.32945f); 
  data[17].mBoundingSphere = Sphere(Vector3(-9.59043f, 26.8716f, -19.4691f), 1.21138f); 
  data[18].mBoundingSphere = Sphere(Vector3(-14.3146f, 38.6051f, -21.0449f), 1.45642f); 
  data[19].mBoundingSphere = Sphere(Vector3(-38.1004f, 18.7231f, -12.4878f), 2.2841f); 
  data[20].mBoundingSphere = Sphere(Vector3(6.81862f, -20.2703f, -14.4115f), 3.71089f); 
  data[21].mBoundingSphere = Sphere(Vector3(7.98158f, 30.323f, 18.3781f), 1.04693f); 
  data[22].mBoundingSphere = Sphere(Vector3(-27.1965f, -8.39281f, -19.2283f), 0.864858f); 
  data[23].mBoundingSphere = Sphere(Vector3(-0.509654f, -46.8324f, -17.3057f), 1.55848f); 
  data[24].mBoundingSphere = Sphere(Vector3(-4.00669f, 18.6998f, -3.03087f), 1.31053f); 
  data[25].mBoundingSphere = Sphere(Vector3(37.5191f, -17.7591f, 10.2968f), 3.18856f); 
  data[26].mBoundingSphere = Sphere(Vector3(21.6372f, -9.31718f, -27.1447f), 0.386712f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-16.3149f, -5.60588f, -5.24464f));
  ray.mDirection = Vector3(Vector3(0.215882f, 0.620435f, -0.75396f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mBoundingSphere = Sphere(Vector3(26.147f, -17.899f, -33.1627f), 2.39555f); 
  data[1].mBoundingSphere = Sphere(Vector3(-34.7778f, 6.28231f, -14.0873f), 2.70844f); 
  data[2].mBoundingSphere = Sphere(Vector3(24.1942f, 11.783f, -33.2463f), 3.01831f); 
  data[3].mBoundingSphere = Sphere(Vector3(21.4394f, 7.13988f, 16.0967f), 3.59573f); 
  data[4].mBoundingSphere = Sphere(Vector3(14.1469f, -9.83644f, -3.5449f), 1.01492f); 
  data[5].mBoundingSphere = Sphere(Vector3(7.55481f, -33.9694f, 27.6034f), 1.79424f); 
  data[6].mBoundingSphere = Sphere(Vector3(-43.1983f, 7.47879f, -22.0425f), 0.536314f); 
  data[7].mBoundingSphere = Sphere(Vector3(-26.1294f, -8.88865f, 37.4398f), 1.06525f); 
  data[8].mBoundingSphere = Sphere(Vector3(10.2841f, 8.99783f, 36.5388f), 1.35077f); 
  data[9].mBoundingSphere = Sphere(Vector3(-1.66717f, -47.0401f, -3.55392f), 1.7155f); 
  data[10].mBoundingSphere = Sphere(Vector3(31.1227f, 17.6433f, 23.5549f), 2.71888f); 
  data[11].mBoundingSphere = Sphere(Vector3(-1.87174f, -21.845f, 38.4748f), 2.80933f); 
  data[12].mBoundingSphere = Sphere(Vector3(18.4349f, -31.5565f, -16.0463f), 3.94248f); 
  data[13].mBoundingSphere = Sphere(Vector3(21.8098f, -2.06162f, -27.2481f), 1.05459f); 
  data[14].mBoundingSphere = Sphere(Vector3(21.4139f, -7.89233f, 18.4395f), 1.58886f); 
  data[15].mBoundingSphere = Sphere(Vector3(0.135119f, -17.5066f, 1.9553f), 1.5652f); 
  data[16].mBoundingSphere = Sphere(Vector3(-18.8586f, -21.2136f, 16.631f), 2.39022f); 
  data[17].mBoundingSphere = Sphere(Vector3(-18.8834f, 6.37932f, -18.5308f), 2.49772f); 
  data[18].mBoundingSphere = Sphere(Vector3(42.5025f, -17.0686f, -17.8752f), 2.41747f); 
  data[19].mBoundingSphere = Sphere(Vector3(-24.8918f, -21.4811f, 26.2983f), 2.9623f); 
  data[20].mBoundingSphere = Sphere(Vector3(-38.0316f, 3.05489f, -25.2249f), 0.929685f); 
  data[21].mBoundingSphere = Sphere(Vector3(-38.3439f, -1.64062f, -28.1262f), 3.8105f); 
  data[22].mBoundingSphere = Sphere(Vector3(15.0708f, -9.22106f, 2.04913f), 3.29861f); 
  data[23].mBoundingSphere = Sphere(Vector3(-19.7508f, 34.1313f, 23.1767f), 1.29615f); 
  data[24].mBoundingSphere = Sphere(Vector3(31.0562f, -25.5831f, 25.3388f), 1.60718f); 
  data[25].mBoundingSphere = Sphere(Vector3(36.1222f, 23.1819f, 13.8781f), 0.212873f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(10.659f, -4.89102f, 14.9044f));
  ray.mDirection = Vector3(Vector3(0.178378f, -0.215463f, -0.960082f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(37); 
  data[0].mBoundingSphere = Sphere(Vector3(17.0357f, -15.9574f, -20.3406f), 1.47799f); 
  data[1].mBoundingSphere = Sphere(Vector3(17.0732f, 0.0180041f, -31.838f), 2.14007f); 
  data[2].mBoundingSphere = Sphere(Vector3(-8.77707f, -26.9708f, -31.1408f), 1.48843f); 
  data[3].mBoundingSphere = Sphere(Vector3(-6.08264f, 30.1031f, -15.8921f), 2.2398f); 
  data[4].mBoundingSphere = Sphere(Vector3(16.5173f, 10.9615f, 4.13769f), 1.75331f); 
  data[5].mBoundingSphere = Sphere(Vector3(-10.478f, -32.0514f, 9.66597f), 1.94431f); 
  data[6].mBoundingSphere = Sphere(Vector3(-2.9798f, 6.8109f, -40.441f), 1.03742f); 
  data[7].mBoundingSphere = Sphere(Vector3(-2.75913f, 15.7795f, -13.2312f), 3.85202f); 
  data[8].mBoundingSphere = Sphere(Vector3(12.5501f, -20.0448f, -38.8808f), 1.81303f); 
  data[9].mBoundingSphere = Sphere(Vector3(13.6839f, 6.38094f, 32.173f), 1.18378f); 
  data[10].mBoundingSphere = Sphere(Vector3(30.941f, 35.9896f, 5.85674f), 1.30949f); 
  data[11].mBoundingSphere = Sphere(Vector3(1.31443f, -25.4363f, 22.8496f), 1.37084f); 
  data[12].mBoundingSphere = Sphere(Vector3(9.97377f, 6.74923f, -12.2633f), 2.0372f); 
  data[13].mBoundingSphere = Sphere(Vector3(10.4774f, -10.1049f, -0.979977f), 0.715372f); 
  data[14].mBoundingSphere = Sphere(Vector3(30.6883f, 8.77545f, -7.80052f), 3.78696f); 
  data[15].mBoundingSphere = Sphere(Vector3(-22.6234f, -36.2959f, -18.6732f), 1.32758f); 
  data[16].mBoundingSphere = Sphere(Vector3(-18.1508f, 9.23283f, 19.2917f), 3.70393f); 
  data[17].mBoundingSphere = Sphere(Vector3(-26.1111f, -2.09737f, 16.6926f), 1.93782f); 
  data[18].mBoundingSphere = Sphere(Vector3(18.9685f, 8.70393f, -6.87418f), 0.637208f); 
  data[19].mBoundingSphere = Sphere(Vector3(33.3001f, 26.1381f, 7.93803f), 0.582354f); 
  data[20].mBoundingSphere = Sphere(Vector3(12.8643f, 10.3999f, -7.18704f), 3.76412f); 
  data[21].mBoundingSphere = Sphere(Vector3(21.4822f, 16.8818f, -20.3984f), 1.56636f); 
  data[22].mBoundingSphere = Sphere(Vector3(-27.3767f, -2.69115f, -17.5184f), 3.68885f); 
  data[23].mBoundingSphere = Sphere(Vector3(-32.1928f, -3.63273f, -15.2256f), 3.89969f); 
  data[24].mBoundingSphere = Sphere(Vector3(-1.95609f, 31.9635f, -9.28299f), 1.01701f); 
  data[25].mBoundingSphere = Sphere(Vector3(-13.06f, -20.1689f, -0.891573f), 1.60858f); 
  data[26].mBoundingSphere = Sphere(Vector3(35.3888f, 23.1932f, -12.2072f), 1.20535f); 
  data[27].mBoundingSphere = Sphere(Vector3(0.589246f, -19.7876f, -19.2167f), 3.52325f); 
  data[28].mBoundingSphere = Sphere(Vector3(30.5928f, -8.20492f, -21.8027f), 2.28503f); 
  data[29].mBoundingSphere = Sphere(Vector3(12.4238f, 21.192f, -8.83645f), 0.338353f); 
  data[30].mBoundingSphere = Sphere(Vector3(16.4884f, 3.20597f, -12.0663f), 2.06144f); 
  data[31].mBoundingSphere = Sphere(Vector3(5.07477f, 41.1927f, 26.0911f), 0.551042f); 
  data[32].mBoundingSphere = Sphere(Vector3(-4.70105f, 4.97789f, -18.4854f), 1.98826f); 
  data[33].mBoundingSphere = Sphere(Vector3(-1.42747f, -26.3959f, -36.2333f), 2.27169f); 
  data[34].mBoundingSphere = Sphere(Vector3(2.59595f, 10.8727f, -30.7985f), 3.91685f); 
  data[35].mBoundingSphere = Sphere(Vector3(26.3957f, 6.92757f, -26.6627f), 3.51084f); 
  data[36].mBoundingSphere = Sphere(Vector3(32.5434f, -32.2901f, 5.88354f), 3.52f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(16.3052f, -1.79403f, 18.5787f));
  ray.mDirection = Vector3(Vector3(-0.233583f, 0.261032f, -0.936644f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(48); 
  data[0].mBoundingSphere = Sphere(Vector3(-12.0599f, 11.5355f, -33.4302f), 2.87764f); 
  data[1].mBoundingSphere = Sphere(Vector3(-21.8583f, 22.0361f, 14.0555f), 3.30905f); 
  data[2].mBoundingSphere = Sphere(Vector3(29.3528f, -26.2635f, 23.0818f), 3.88287f); 
  data[3].mBoundingSphere = Sphere(Vector3(10.4152f, 30.728f, -16.6932f), 0.965404f); 
  data[4].mBoundingSphere = Sphere(Vector3(35.9339f, -26.3271f, 14.2366f), 0.214612f); 
  data[5].mBoundingSphere = Sphere(Vector3(-13.4449f, -10.3157f, -31.7394f), 2.89886f); 
  data[6].mBoundingSphere = Sphere(Vector3(-6.88357f, -2.21423f, -3.67969f), 1.5732f); 
  data[7].mBoundingSphere = Sphere(Vector3(-17.4554f, -39.0988f, 19.4974f), 0.435536f); 
  data[8].mBoundingSphere = Sphere(Vector3(-17.1304f, 10.7197f, 21.2658f), 1.5775f); 
  data[9].mBoundingSphere = Sphere(Vector3(-6.16758f, -9.60532f, -41.9043f), 2.85085f); 
  data[10].mBoundingSphere = Sphere(Vector3(-37.4822f, 24.7351f, -6.73668f), 1.76827f); 
  data[11].mBoundingSphere = Sphere(Vector3(-10.4358f, 14.8912f, 13.8597f), 1.75748f); 
  data[12].mBoundingSphere = Sphere(Vector3(18.7864f, -35.3042f, -9.11417f), 3.78511f); 
  data[13].mBoundingSphere = Sphere(Vector3(-4.2931f, -47.1283f, -13.7411f), 2.47209f); 
  data[14].mBoundingSphere = Sphere(Vector3(-21.8545f, 3.64561f, 26.2329f), 3.91824f); 
  data[15].mBoundingSphere = Sphere(Vector3(-16.9656f, -44.5451f, 6.61032f), 0.753526f); 
  data[16].mBoundingSphere = Sphere(Vector3(41.8351f, -21.0839f, 14.7476f), 1.43694f); 
  data[17].mBoundingSphere = Sphere(Vector3(13.3107f, -43.7197f, 10.3432f), 3.71924f); 
  data[18].mBoundingSphere = Sphere(Vector3(45.2047f, -17.9327f, 8.74784f), 1.69115f); 
  data[19].mBoundingSphere = Sphere(Vector3(-11.3664f, 35.2718f, -14.2563f), 3.73362f); 
  data[20].mBoundingSphere = Sphere(Vector3(16.654f, -3.53414f, 23.0996f), 2.58899f); 
  data[21].mBoundingSphere = Sphere(Vector3(6.29407f, 19.3977f, -19.0616f), 0.273641f); 
  data[22].mBoundingSphere = Sphere(Vector3(6.65332f, -22.6296f, 29.922f), 2.4489f); 
  data[23].mBoundingSphere = Sphere(Vector3(-27.8721f, 7.7538f, 22.4478f), 1.3691f); 
  data[24].mBoundingSphere = Sphere(Vector3(-16.2496f, 32.2389f, -10.1063f), 2.62761f); 
  data[25].mBoundingSphere = Sphere(Vector3(-19.8945f, 43.3508f, -10.6509f), 0.816382f); 
  data[26].mBoundingSphere = Sphere(Vector3(-39.8037f, 11.8251f, 14.7089f), 1.1215f); 
  data[27].mBoundingSphere = Sphere(Vector3(8.58094f, -43.7848f, -15.9712f), 3.36924f); 
  data[28].mBoundingSphere = Sphere(Vector3(33.7665f, 27.3087f, -14.1096f), 3.7451f); 
  data[29].mBoundingSphere = Sphere(Vector3(16.5251f, 29.5484f, 27.1182f), 2.90292f); 
  data[30].mBoundingSphere = Sphere(Vector3(-26.2631f, 25.3926f, -4.83467f), 3.48231f); 
  data[31].mBoundingSphere = Sphere(Vector3(15.1315f, -22.6333f, 32.2038f), 0.954619f); 
  data[32].mBoundingSphere = Sphere(Vector3(11.5313f, -45.3602f, -13.2939f), 0.214612f); 
  data[33].mBoundingSphere = Sphere(Vector3(18.8402f, 4.23653f, -16.7535f), 1.03812f); 
  data[34].mBoundingSphere = Sphere(Vector3(-13.0836f, -42.6652f, -0.589766f), 3.82315f); 
  data[35].mBoundingSphere = Sphere(Vector3(9.68142f, -31.1336f, -18.7774f), 1.53354f); 
  data[36].mBoundingSphere = Sphere(Vector3(19.3303f, -37.4093f, -21.006f), 1.66552f); 
  data[37].mBoundingSphere = Sphere(Vector3(15.5652f, 10.4765f, 3.37259f), 2.35218f); 
  data[38].mBoundingSphere = Sphere(Vector3(12.1441f, -13.9677f, -41.9008f), 0.628974f); 
  data[39].mBoundingSphere = Sphere(Vector3(34.312f, 0.154615f, -31.4622f), 2.57623f); 
  data[40].mBoundingSphere = Sphere(Vector3(-7.14923f, 31.1987f, -6.08624f), 3.15748f); 
  data[41].mBoundingSphere = Sphere(Vector3(38.2876f, 9.89964f, 14.6126f), 0.474154f); 
  data[42].mBoundingSphere = Sphere(Vector3(6.23061f, 32.9282f, -14.0812f), 0.853493f); 
  data[43].mBoundingSphere = Sphere(Vector3(-25.4365f, -30.5158f, -8.28423f), 3.48996f); 
  data[44].mBoundingSphere = Sphere(Vector3(-37.0724f, 5.99436f, 12.8186f), 0.856392f); 
  data[45].mBoundingSphere = Sphere(Vector3(-4.95127f, -19.2982f, 6.65046f), 2.60812f); 
  data[46].mBoundingSphere = Sphere(Vector3(-17.6652f, -37.0951f, 13.7825f), 1.39357f); 
  data[47].mBoundingSphere = Sphere(Vector3(-11.685f, 14.7625f, 29.3749f), 2.29396f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(4.0834f, 12.6011f, 18.3077f));
  ray.mDirection = Vector3(Vector3(-0.387889f, 0.53087f, -0.753471f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(36); 
  data[0].mBoundingSphere = Sphere(Vector3(-7.50931f, 34.8837f, 12.3809f), 2.79716f); 
  data[1].mBoundingSphere = Sphere(Vector3(32.9756f, 22.9658f, 9.51919f), 0.594995f); 
  data[2].mBoundingSphere = Sphere(Vector3(-15.9094f, -7.23568f, -19.0849f), 1.72246f); 
  data[3].mBoundingSphere = Sphere(Vector3(-16.0275f, -40.0599f, -17.9016f), 1.15571f); 
  data[4].mBoundingSphere = Sphere(Vector3(16.7888f, 15.4326f, 1.5787f), 2.94711f); 
  data[5].mBoundingSphere = Sphere(Vector3(26.9904f, 2.88069f, 4.62083f), 1.61843f); 
  data[6].mBoundingSphere = Sphere(Vector3(-24.6015f, -26.0703f, 26.8388f), 0.761412f); 
  data[7].mBoundingSphere = Sphere(Vector3(-35.402f, -0.4888f, -7.53761f), 1.51093f); 
  data[8].mBoundingSphere = Sphere(Vector3(0.00231166f, -48.2819f, -8.85199f), 1.86383f); 
  data[9].mBoundingSphere = Sphere(Vector3(-3.2424f, 16.2879f, -5.77554f), 0.30762f); 
  data[10].mBoundingSphere = Sphere(Vector3(-8.85141f, 8.14421f, 34.3416f), 2.93203f); 
  data[11].mBoundingSphere = Sphere(Vector3(-30.1187f, -13.3235f, 0.7329f), 0.888284f); 
  data[12].mBoundingSphere = Sphere(Vector3(32.1602f, 25.3632f, -7.69337f), 0.968651f); 
  data[13].mBoundingSphere = Sphere(Vector3(12.2183f, 13.7096f, -38.017f), 3.97159f); 
  data[14].mBoundingSphere = Sphere(Vector3(-2.60529f, -30.8937f, 19.9625f), 2.22287f); 
  data[15].mBoundingSphere = Sphere(Vector3(36.38f, -14.5451f, -25.324f), 1.12312f); 
  data[16].mBoundingSphere = Sphere(Vector3(38.5343f, -2.07465f, 15.5741f), 3.08592f); 
  data[17].mBoundingSphere = Sphere(Vector3(13.261f, -17.5312f, -14.3057f), 0.540141f); 
  data[18].mBoundingSphere = Sphere(Vector3(-9.86038f, 32.5099f, -5.3324f), 3.76261f); 
  data[19].mBoundingSphere = Sphere(Vector3(-37.5668f, -5.98562f, 0.57241f), 2.00937f); 
  data[20].mBoundingSphere = Sphere(Vector3(-5.99311f, -24.4871f, 3.7229f), 1.51719f); 
  data[21].mBoundingSphere = Sphere(Vector3(-21.3854f, 21.7337f, 24.9624f), 0.840504f); 
  data[22].mBoundingSphere = Sphere(Vector3(41.5507f, -5.51771f, -10.1684f), 1.49481f); 
  data[23].mBoundingSphere = Sphere(Vector3(-9.81634f, -28.7164f, 24.8735f), 1.77523f); 
  data[24].mBoundingSphere = Sphere(Vector3(-12.4865f, -38.9523f, 15.9363f), 3.31728f); 
  data[25].mBoundingSphere = Sphere(Vector3(34.352f, -20.3184f, -13.5859f), 1.07071f); 
  data[26].mBoundingSphere = Sphere(Vector3(-23.2236f, -27.7635f, -3.05411f), 1.45399f); 
  data[27].mBoundingSphere = Sphere(Vector3(35.0422f, -17.4084f, -21.3328f), 1.35785f); 
  data[28].mBoundingSphere = Sphere(Vector3(10.1903f, 12.821f, -36.1795f), 3.16849f); 
  data[29].mBoundingSphere = Sphere(Vector3(-5.05263f, -3.86898f, -28.7189f), 2.27564f); 
  data[30].mBoundingSphere = Sphere(Vector3(-7.12306f, -20.9689f, 39.8121f), 3.23923f); 
  data[31].mBoundingSphere = Sphere(Vector3(-36.9905f, 25.8832f, 19.9662f), 1.07848f); 
  data[32].mBoundingSphere = Sphere(Vector3(-24.7489f, -18.3264f, -19.2388f), 2.85584f); 
  data[33].mBoundingSphere = Sphere(Vector3(19.936f, 14.6945f, 28.2391f), 3.22138f); 
  data[34].mBoundingSphere = Sphere(Vector3(3.96315f, -41.3547f, 19.1602f), 0.490042f); 
  data[35].mBoundingSphere = Sphere(Vector3(-15.1731f, -24.7534f, 36.1844f), 1.11698f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-5.56579f, 0.429891f, -12.7737f));
  ray.mDirection = Vector3(Vector3(0.464324f, 0.376554f, -0.80163f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(27.3775f, -6.47144f, 30.949f), 1.49191f); 
  data[1].mBoundingSphere = Sphere(Vector3(19.3811f, -5.23385f, 24.1717f), 0.549419f); 
  data[2].mBoundingSphere = Sphere(Vector3(-3.60997f, -11.1877f, 27.8571f), 0.537474f); 
  data[3].mBoundingSphere = Sphere(Vector3(11.7712f, -23.5276f, 5.35781f), 2.14401f); 
  data[4].mBoundingSphere = Sphere(Vector3(1.91556f, -44.9208f, -7.60539f), 3.08662f); 
  data[5].mBoundingSphere = Sphere(Vector3(-8.69511f, 23.7145f, 20.9099f), 3.32842f); 
  data[6].mBoundingSphere = Sphere(Vector3(-13.4534f, 2.63861f, 3.1308f), 3.89018f); 
  data[7].mBoundingSphere = Sphere(Vector3(2.36403f, -47.5152f, -6.68417f), 3.64861f); 
  data[8].mBoundingSphere = Sphere(Vector3(14.9579f, -37.3969f, -6.99534f), 0.419764f); 
  data[9].mBoundingSphere = Sphere(Vector3(-6.66109f, -3.55399f, -28.4529f), 0.717112f); 
  data[10].mBoundingSphere = Sphere(Vector3(25.2302f, -29.3356f, -10.6229f), 2.67956f); 
  data[11].mBoundingSphere = Sphere(Vector3(-25.5732f, 0.941943f, -36.2428f), 2.16349f); 
  data[12].mBoundingSphere = Sphere(Vector3(7.2061f, -4.23618f, 33.7319f), 1.69242f); 
  data[13].mBoundingSphere = Sphere(Vector3(-48.308f, 7.95374f, -3.54589f), 3.85736f); 
  data[14].mBoundingSphere = Sphere(Vector3(14.115f, 33.0272f, -29.8538f), 3.06215f); 
  data[15].mBoundingSphere = Sphere(Vector3(-1.51508f, 22.8853f, 11.6142f), 2.80794f); 
  data[16].mBoundingSphere = Sphere(Vector3(18.8403f, 33.7947f, 31.4379f), 0.209626f); 
  data[17].mBoundingSphere = Sphere(Vector3(-19.5057f, -3.91505f, 21.4319f), 2.93876f); 
  data[18].mBoundingSphere = Sphere(Vector3(-30.9224f, -25.5976f, -10.1509f), 1.2856f); 
  data[19].mBoundingSphere = Sphere(Vector3(-3.1295f, -7.29355f, -40.6233f), 2.33629f); 
  data[20].mBoundingSphere = Sphere(Vector3(33.1837f, 24.3513f, 27.8377f), 3.35602f); 
  data[21].mBoundingSphere = Sphere(Vector3(-0.263955f, 36.4633f, -18.2371f), 3.77432f); 
  data[22].mBoundingSphere = Sphere(Vector3(-6.7154f, 19.3776f, -9.57918f), 1.26461f); 
  data[23].mBoundingSphere = Sphere(Vector3(24.9321f, -13.1156f, 18.6458f), 2.91336f); 
  data[24].mBoundingSphere = Sphere(Vector3(11.3567f, -37.7165f, 5.55401f), 0.769878f); 
  data[25].mBoundingSphere = Sphere(Vector3(6.68607f, 36.5223f, 17.8092f), 3.57485f); 
  data[26].mBoundingSphere = Sphere(Vector3(-25.2928f, -8.38279f, 29.3301f), 1.75586f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-11.5603f, -0.419143f, 18.8636f));
  ray.mDirection = Vector3(Vector3(-0.256082f, 0.0259169f, -0.966308f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(52); 
  data[0].mBoundingSphere = Sphere(Vector3(25.3793f, 15.7971f, -7.42543f), 1.88783f); 
  data[1].mBoundingSphere = Sphere(Vector3(-41.1598f, 13.327f, -5.64164f), 0.409906f); 
  data[2].mBoundingSphere = Sphere(Vector3(-5.0196f, -28.936f, -38.1908f), 1.51093f); 
  data[3].mBoundingSphere = Sphere(Vector3(8.72034f, 0.817559f, -47.1991f), 2.26903f); 
  data[4].mBoundingSphere = Sphere(Vector3(38.1156f, -9.58274f, 23.6163f), 3.54029f); 
  data[5].mBoundingSphere = Sphere(Vector3(21.6104f, 30.9692f, -32.635f), 3.59584f); 
  data[6].mBoundingSphere = Sphere(Vector3(-3.81176f, -38.1195f, -2.34149f), 1.35367f); 
  data[7].mBoundingSphere = Sphere(Vector3(4.87983f, -14.2085f, -13.9602f), 1.72721f); 
  data[8].mBoundingSphere = Sphere(Vector3(3.64391f, -14.4032f, 41.8512f), 2.30683f); 
  data[9].mBoundingSphere = Sphere(Vector3(-44.3041f, 10.9446f, 4.91539f), 3.29467f); 
  data[10].mBoundingSphere = Sphere(Vector3(-9.35147f, -34.9415f, -20.0653f), 0.91055f); 
  data[11].mBoundingSphere = Sphere(Vector3(4.06545f, -34.1319f, 21.8097f), 2.38871f); 
  data[12].mBoundingSphere = Sphere(Vector3(27.0491f, 9.24495f, 31.76f), 3.7829f); 
  data[13].mBoundingSphere = Sphere(Vector3(7.60685f, 32.2984f, -34.1877f), 2.89944f); 
  data[14].mBoundingSphere = Sphere(Vector3(-18.2483f, -19.942f, 22.0059f), 3.77676f); 
  data[15].mBoundingSphere = Sphere(Vector3(41.7084f, -9.79142f, 14.4913f), 1.43056f); 
  data[16].mBoundingSphere = Sphere(Vector3(18.6595f, -13.2844f, -34.0063f), 2.08208f); 
  data[17].mBoundingSphere = Sphere(Vector3(12.3635f, -23.8592f, -11.2305f), 2.99001f); 
  data[18].mBoundingSphere = Sphere(Vector3(-16.8857f, 2.31981f, 14.9919f), 2.30324f); 
  data[19].mBoundingSphere = Sphere(Vector3(-8.39653f, 15.6883f, -46.5491f), 1.05424f); 
  data[20].mBoundingSphere = Sphere(Vector3(6.49656f, -27.1636f, -29.1225f), 3.11132f); 
  data[21].mBoundingSphere = Sphere(Vector3(30.9219f, -16.2137f, -10.6366f), 1.80839f); 
  data[22].mBoundingSphere = Sphere(Vector3(24.3303f, -43.3877f, -4.99374f), 0.8717f); 
  data[23].mBoundingSphere = Sphere(Vector3(-10.5887f, 21.3018f, 3.69175f), 1.3611f); 
  data[24].mBoundingSphere = Sphere(Vector3(26.7064f, 15.6514f, 5.36434f), 2.24606f); 
  data[25].mBoundingSphere = Sphere(Vector3(33.234f, -9.11852f, 10.1994f), 3.3341f); 
  data[26].mBoundingSphere = Sphere(Vector3(-43.5108f, -3.28516f, -7.7987f), 1.16998f); 
  data[27].mBoundingSphere = Sphere(Vector3(-17.5147f, -5.86461f, 3.80361f), 0.826936f); 
  data[28].mBoundingSphere = Sphere(Vector3(-6.31922f, -28.8971f, -33.1995f), 3.80505f); 
  data[29].mBoundingSphere = Sphere(Vector3(6.00979f, -9.88717f, 5.38733f), 2.55791f); 
  data[30].mBoundingSphere = Sphere(Vector3(10.6776f, 34.571f, -32.5025f), 2.66646f); 
  data[31].mBoundingSphere = Sphere(Vector3(-21.6985f, 19.7965f, -27.3442f), 0.577251f); 
  data[32].mBoundingSphere = Sphere(Vector3(7.2122f, -39.7404f, 22.2362f), 2.52532f); 
  data[33].mBoundingSphere = Sphere(Vector3(-42.7009f, -13.8531f, -8.26788f), 1.15849f); 
  data[34].mBoundingSphere = Sphere(Vector3(1.25297f, 0.59572f, 25.0854f), 3.08557f); 
  data[35].mBoundingSphere = Sphere(Vector3(-27.8944f, -14.993f, -27.1058f), 1.06212f); 
  data[36].mBoundingSphere = Sphere(Vector3(18.7165f, 33.4368f, 29.8368f), 3.03721f); 
  data[37].mBoundingSphere = Sphere(Vector3(1.91854f, -7.52871f, -29.6227f), 0.376275f); 
  data[38].mBoundingSphere = Sphere(Vector3(-14.4158f, 10.9425f, -16.5511f), 2.68258f); 
  data[39].mBoundingSphere = Sphere(Vector3(29.7763f, -16.2816f, 4.42331f), 2.53124f); 
  data[40].mBoundingSphere = Sphere(Vector3(10.0835f, 35.6047f, -23.9107f), 1.33964f); 
  data[41].mBoundingSphere = Sphere(Vector3(-40.2999f, -22.0108f, -12.5818f), 1.48901f); 
  data[42].mBoundingSphere = Sphere(Vector3(2.15686f, 6.16065f, 43.1847f), 0.918552f); 
  data[43].mBoundingSphere = Sphere(Vector3(17.8529f, -32.1975f, -10.3513f), 1.70877f); 
  data[44].mBoundingSphere = Sphere(Vector3(-2.17129f, 38.1168f, 4.59687f), 0.474734f); 
  data[45].mBoundingSphere = Sphere(Vector3(8.79887f, -6.56921f, 39.2433f), 2.48636f); 
  data[46].mBoundingSphere = Sphere(Vector3(6.23813f, -8.91418f, -23.6093f), 0.735435f); 
  data[47].mBoundingSphere = Sphere(Vector3(34.189f, -0.108176f, 34.5438f), 3.66879f); 
  data[48].mBoundingSphere = Sphere(Vector3(-3.70372f, 3.93996f, -5.94355f), 3.4031f); 
  data[49].mBoundingSphere = Sphere(Vector3(-7.51005f, 21.5503f, -18.859f), 2.34487f); 
  data[50].mBoundingSphere = Sphere(Vector3(-4.7496f, -19.9013f, -14.8476f), 1.43114f); 
  data[51].mBoundingSphere = Sphere(Vector3(38.3837f, 2.42102f, 15.4216f), 3.30963f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-5.09552f, 5.28894f, -6.94109f));
  ray.mDirection = Vector3(Vector3(-0.443367f, -0.149497f, 0.883785f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(42); 
  data[0].mBoundingSphere = Sphere(Vector3(2.96575f, -26.2257f, 17.2972f), 0.965752f); 
  data[1].mBoundingSphere = Sphere(Vector3(0.434973f, 19.3435f, 43.2727f), 3.06482f); 
  data[2].mBoundingSphere = Sphere(Vector3(12.0085f, -11.6484f, -1.72098f), 3.07641f); 
  data[3].mBoundingSphere = Sphere(Vector3(-42.3219f, 10.3947f, 5.62287f), 0.550462f); 
  data[4].mBoundingSphere = Sphere(Vector3(7.11862f, 32.6878f, 31.554f), 0.536314f); 
  data[5].mBoundingSphere = Sphere(Vector3(45.6706f, -9.93909f, -12.9595f), 2.3567f); 
  data[6].mBoundingSphere = Sphere(Vector3(-5.41936f, 29.2701f, -4.96518f), 0.47311f); 
  data[7].mBoundingSphere = Sphere(Vector3(-13.7111f, -36.356f, 26.6826f), 3.26011f); 
  data[8].mBoundingSphere = Sphere(Vector3(-10.369f, -27.7843f, -2.04758f), 1.40679f); 
  data[9].mBoundingSphere = Sphere(Vector3(-12.6343f, -9.8798f, -21.6819f), 1.96553f); 
  data[10].mBoundingSphere = Sphere(Vector3(17.5592f, 5.20376f, 16.3946f), 2.76526f); 
  data[11].mBoundingSphere = Sphere(Vector3(4.28191f, 2.86297f, 38.8675f), 0.408979f); 
  data[12].mBoundingSphere = Sphere(Vector3(18.1197f, -26.9349f, -16.7699f), 1.57413f); 
  data[13].mBoundingSphere = Sphere(Vector3(-43.9969f, 2.67768f, 21.6263f), 1.63444f); 
  data[14].mBoundingSphere = Sphere(Vector3(-24.2699f, -16.8164f, -6.01329f), 2.92159f); 
  data[15].mBoundingSphere = Sphere(Vector3(-6.8961f, -33.1775f, 35.4652f), 2.44635f); 
  data[16].mBoundingSphere = Sphere(Vector3(32.6117f, 3.84792f, -12.791f), 2.39775f); 
  data[17].mBoundingSphere = Sphere(Vector3(39.7129f, 14.5243f, 26.3873f), 3.63736f); 
  data[18].mBoundingSphere = Sphere(Vector3(-4.75134f, 5.23046f, 5.76932f), 1.3103f); 
  data[19].mBoundingSphere = Sphere(Vector3(42.9585f, 10.8353f, 5.87165f), 3.58981f); 
  data[20].mBoundingSphere = Sphere(Vector3(32.5305f, -24.8949f, -26.731f), 1.47382f); 
  data[21].mBoundingSphere = Sphere(Vector3(-34.7492f, 5.63239f, 34.7252f), 1.85408f); 
  data[22].mBoundingSphere = Sphere(Vector3(-23.4098f, 9.6812f, 33.5189f), 2.37468f); 
  data[23].mBoundingSphere = Sphere(Vector3(-8.017f, -4.70563f, 17.893f), 1.30009f); 
  data[24].mBoundingSphere = Sphere(Vector3(26.4462f, 22.1627f, -15.5892f), 0.710849f); 
  data[25].mBoundingSphere = Sphere(Vector3(-3.86288f, 20.4471f, 16.6252f), 1.24536f); 
  data[26].mBoundingSphere = Sphere(Vector3(18.1168f, 26.1975f, 14.7186f), 3.70729f); 
  data[27].mBoundingSphere = Sphere(Vector3(-40.7469f, -13.6436f, 9.35589f), 0.556725f); 
  data[28].mBoundingSphere = Sphere(Vector3(20.2308f, -19.4069f, 7.30749f), 3.00463f); 
  data[29].mBoundingSphere = Sphere(Vector3(12.9219f, -0.678305f, 10.6655f), 3.12802f); 
  data[30].mBoundingSphere = Sphere(Vector3(1.14148f, -37.1358f, 14.293f), 0.325248f); 
  data[31].mBoundingSphere = Sphere(Vector3(-14.7928f, 6.10766f, -28.2384f), 2.93215f); 
  data[32].mBoundingSphere = Sphere(Vector3(-20.5786f, -18.7964f, -15.2633f), 1.31262f); 
  data[33].mBoundingSphere = Sphere(Vector3(-2.64239f, -49.3885f, 0.215848f), 1.8491f); 
  data[34].mBoundingSphere = Sphere(Vector3(41.3045f, -0.542574f, 21.7664f), 0.345659f); 
  data[35].mBoundingSphere = Sphere(Vector3(-10.8102f, -18.1473f, 21.45f), 3.26742f); 
  data[36].mBoundingSphere = Sphere(Vector3(-39.4729f, -21.51f, -20.9124f), 1.9297f); 
  data[37].mBoundingSphere = Sphere(Vector3(20.323f, -15.3928f, -37.6813f), 0.461977f); 
  data[38].mBoundingSphere = Sphere(Vector3(5.18172f, -25.3811f, -4.50406f), 2.65996f); 
  data[39].mBoundingSphere = Sphere(Vector3(7.76096f, 24.9747f, 9.27562f), 1.54676f); 
  data[40].mBoundingSphere = Sphere(Vector3(-8.68252f, 1.76349f, 42.2352f), 2.16268f); 
  data[41].mBoundingSphere = Sphere(Vector3(-13.0119f, -30.8959f, 0.215881f), 3.60118f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(9.37048f, -11.734f, 0.482818f));
  ray.mDirection = Vector3(Vector3(0.714052f, 0.560919f, -0.418928f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(40); 
  data[0].mBoundingSphere = Sphere(Vector3(-25.8241f, 22.2253f, 0.554288f), 2.29651f); 
  data[1].mBoundingSphere = Sphere(Vector3(-18.8687f, 10.4706f, 38.0424f), 1.01156f); 
  data[2].mBoundingSphere = Sphere(Vector3(27.913f, 17.7402f, -7.82147f), 2.54469f); 
  data[3].mBoundingSphere = Sphere(Vector3(36.9277f, 1.79645f, -6.1456f), 2.37873f); 
  data[4].mBoundingSphere = Sphere(Vector3(-13.9984f, -19.7484f, -10.1899f), 1.17914f); 
  data[5].mBoundingSphere = Sphere(Vector3(-28.2045f, -15.5241f, -27.9131f), 2.90686f); 
  data[6].mBoundingSphere = Sphere(Vector3(-4.53062f, -16.6219f, 11.3071f), 0.707718f); 
  data[7].mBoundingSphere = Sphere(Vector3(-36.6301f, 14.4459f, -13.9826f), 2.47279f); 
  data[8].mBoundingSphere = Sphere(Vector3(35.6147f, -13.719f, -14.872f), 1.66053f); 
  data[9].mBoundingSphere = Sphere(Vector3(-13.7751f, 30.6497f, 6.8304f), 2.27854f); 
  data[10].mBoundingSphere = Sphere(Vector3(17.4933f, -0.451264f, 17.7144f), 3.6144f); 
  data[11].mBoundingSphere = Sphere(Vector3(35.9089f, 22.2938f, 14.926f), 0.413038f); 
  data[12].mBoundingSphere = Sphere(Vector3(37.2683f, 1.69843f, -31.0041f), 2.89225f); 
  data[13].mBoundingSphere = Sphere(Vector3(20.866f, 21.3405f, 19.1167f), 3.08952f); 
  data[14].mBoundingSphere = Sphere(Vector3(4.46543f, -44.6998f, 7.04261f), 2.13705f); 
  data[15].mBoundingSphere = Sphere(Vector3(-7.01432f, 46.6837f, -13.3022f), 2.15561f); 
  data[16].mBoundingSphere = Sphere(Vector3(-48.6129f, -0.316935f, 5.88366f), 1.36144f); 
  data[17].mBoundingSphere = Sphere(Vector3(-17.8188f, 7.4974f, 31.3355f), 2.48195f); 
  data[18].mBoundingSphere = Sphere(Vector3(-9.70798f, 36.3853f, 19.9029f), 0.871468f); 
  data[19].mBoundingSphere = Sphere(Vector3(10.1645f, -24.9537f, 30.2471f), 0.422663f); 
  data[20].mBoundingSphere = Sphere(Vector3(30.6455f, 22.3084f, 26.1128f), 0.866134f); 
  data[21].mBoundingSphere = Sphere(Vector3(-4.08401f, -34.2039f, 8.84697f), 0.752483f); 
  data[22].mBoundingSphere = Sphere(Vector3(-43.4573f, 18.916f, 8.58317f), 1.51128f); 
  data[23].mBoundingSphere = Sphere(Vector3(10.4034f, 12.0401f, 10.0607f), 3.67865f); 
  data[24].mBoundingSphere = Sphere(Vector3(-18.2271f, -19.9572f, -2.12893f), 3.4895f); 
  data[25].mBoundingSphere = Sphere(Vector3(-17.0607f, -34.1244f, -10.868f), 3.72712f); 
  data[26].mBoundingSphere = Sphere(Vector3(4.57706f, -36.1247f, 0.316728f), 1.42337f); 
  data[27].mBoundingSphere = Sphere(Vector3(-34.3034f, 16.8082f, -10.1352f), 0.504538f); 
  data[28].mBoundingSphere = Sphere(Vector3(7.59062f, 23.5164f, -30.9952f), 0.575396f); 
  data[29].mBoundingSphere = Sphere(Vector3(1.08645f, 33.0734f, -24.4793f), 2.33559f); 
  data[30].mBoundingSphere = Sphere(Vector3(-10.6608f, -41.9188f, 5.28073f), 3.6224f); 
  data[31].mBoundingSphere = Sphere(Vector3(-3.95062f, -35.0854f, -7.9676f), 3.77351f); 
  data[32].mBoundingSphere = Sphere(Vector3(-5.05301f, 13.2749f, 9.5777f), 1.70356f); 
  data[33].mBoundingSphere = Sphere(Vector3(25.3301f, -39.3574f, -10.1661f), 1.58759f); 
  data[34].mBoundingSphere = Sphere(Vector3(-14.6698f, -15.8351f, -17.5391f), 3.94607f); 
  data[35].mBoundingSphere = Sphere(Vector3(40.2988f, 26.5218f, 2.54621f), 3.62495f); 
  data[36].mBoundingSphere = Sphere(Vector3(45.981f, -4.70254f, 17.983f), 2.86361f); 
  data[37].mBoundingSphere = Sphere(Vector3(-3.04862f, 1.53094f, -40.4347f), 3.94677f); 
  data[38].mBoundingSphere = Sphere(Vector3(30.2223f, -0.831824f, 19.8396f), 1.03046f); 
  data[39].mBoundingSphere = Sphere(Vector3(-5.41516f, -9.85734f, -13.7688f), 0.240242f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-12.0718f, -13.9088f, -12.8717f));
  ray.mDirection = Vector3(Vector3(-0.807905f, -0.0232441f, -0.588855f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionRayCastFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(32); 
  data[0].mBoundingSphere = Sphere(Vector3(-44.7175f, -15.9427f, 1.58157f), 2.89828f); 
  data[1].mBoundingSphere = Sphere(Vector3(-36.3783f, -21.8241f, 26.0934f), 1.22008f); 
  data[2].mBoundingSphere = Sphere(Vector3(29.8113f, -21.8677f, -13.6038f), 2.714f); 
  data[3].mBoundingSphere = Sphere(Vector3(6.54999f, -28.979f, 6.91797f), 0.344847f); 
  data[4].mBoundingSphere = Sphere(Vector3(-19.3006f, 6.32703f, 20.3555f), 2.33571f); 
  data[5].mBoundingSphere = Sphere(Vector3(-2.81776f, 8.14589f, -15.8964f), 0.81789f); 
  data[6].mBoundingSphere = Sphere(Vector3(3.69901f, 2.16497f, -13.5008f), 3.21882f); 
  data[7].mBoundingSphere = Sphere(Vector3(3.76015f, 18.2041f, 24.9841f), 3.95489f); 
  data[8].mBoundingSphere = Sphere(Vector3(-11.2343f, 26.7755f, -4.24047f), 3.90003f); 
  data[9].mBoundingSphere = Sphere(Vector3(7.37821f, 35.2762f, -18.4707f), 0.797247f); 
  data[10].mBoundingSphere = Sphere(Vector3(-43.2271f, -4.21579f, 6.77877f), 1.08184f); 
  data[11].mBoundingSphere = Sphere(Vector3(29.662f, -21.4882f, -25.5061f), 2.88541f); 
  data[12].mBoundingSphere = Sphere(Vector3(-12.7814f, 37.1112f, 26.4495f), 0.690554f); 
  data[13].mBoundingSphere = Sphere(Vector3(9.99608f, 31.5892f, 35.5779f), 3.20294f); 
  data[14].mBoundingSphere = Sphere(Vector3(34.5344f, 1.6402f, 10.3858f), 0.910318f); 
  data[15].mBoundingSphere = Sphere(Vector3(13.012f, -5.01801f, -6.94163f), 0.298575f); 
  data[16].mBoundingSphere = Sphere(Vector3(-6.3981f, 21.33f, -15.858f), 0.391119f); 
  data[17].mBoundingSphere = Sphere(Vector3(-19.8569f, -0.597979f, 19.786f), 1.98246f); 
  data[18].mBoundingSphere = Sphere(Vector3(28.7891f, -18.7261f, 15.1227f), 1.23306f); 
  data[19].mBoundingSphere = Sphere(Vector3(-23.2269f, -27.595f, -6.00707f), 0.996368f); 
  data[20].mBoundingSphere = Sphere(Vector3(-27.9731f, -3.1294f, 38.9542f), 2.94432f); 
  data[21].mBoundingSphere = Sphere(Vector3(-15.1118f, 14.2672f, 29.6266f), 2.46107f); 
  data[22].mBoundingSphere = Sphere(Vector3(-3.30882f, -19.7247f, -17.0965f), 2.57519f); 
  data[23].mBoundingSphere = Sphere(Vector3(27.2386f, 20.8368f, 14.3164f), 0.496188f); 
  data[24].mBoundingSphere = Sphere(Vector3(5.70227f, 9.16606f, -19.83f), 1.02652f); 
  data[25].mBoundingSphere = Sphere(Vector3(-9.94894f, -33.4709f, 0.201409f), 2.81699f); 
  data[26].mBoundingSphere = Sphere(Vector3(-1.81994f, -39.7253f, 4.97678f), 0.333366f); 
  data[27].mBoundingSphere = Sphere(Vector3(7.40595f, -6.23547f, -44.4686f), 0.705399f); 
  data[28].mBoundingSphere = Sphere(Vector3(7.75589f, 0.860892f, -45.7752f), 3.32354f); 
  data[29].mBoundingSphere = Sphere(Vector3(-6.21222f, -5.05871f, -29.7937f), 2.15596f); 
  data[30].mBoundingSphere = Sphere(Vector3(-23.5985f, -38.8826f, 5.21852f), 0.45096f); 
  data[31].mBoundingSphere = Sphere(Vector3(-31.184f, -25.3439f, -21.6522f), 2.79692f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(4.86922f, 9.04562f, -18.3219f));
  ray.mDirection = Vector3(Vector3(0.0620408f, -0.257009f, -0.964415f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRaySphereTests", static_cast<int>(Application::mStatistics.mRaySphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mBoundingSphere = Sphere(Vector3(-40.6788f, 20.1842f, 10.0033f), 1.48889f); 
  data[1].mBoundingSphere = Sphere(Vector3(-37.9822f, -3.8293f, -27.189f), 3.05217f); 
  data[2].mBoundingSphere = Sphere(Vector3(-21.2522f, -10.6417f, -29.8238f), 1.00681f); 
  data[3].mBoundingSphere = Sphere(Vector3(1.40734f, -8.34536f, -37.4009f), 3.23112f); 
  data[4].mBoundingSphere = Sphere(Vector3(-13.8617f, 12.937f, 28.4589f), 2.11351f); 
  data[5].mBoundingSphere = Sphere(Vector3(-15.0252f, 9.1951f, -26.6559f), 1.3553f); 
  data[6].mBoundingSphere = Sphere(Vector3(29.1962f, 22.7544f, 23.9902f), 3.89006f); 
  data[7].mBoundingSphere = Sphere(Vector3(-18.4959f, -14.6588f, 27.1834f), 0.463601f); 
  data[8].mBoundingSphere = Sphere(Vector3(15.778f, -0.346477f, -17.1926f), 0.318754f); 
  data[9].mBoundingSphere = Sphere(Vector3(3.97458f, 4.91111f, -18.2996f), 2.92739f); 
  data[10].mBoundingSphere = Sphere(Vector3(-26.0683f, 14.4135f, -34.9593f), 1.61449f); 
  data[11].mBoundingSphere = Sphere(Vector3(18.4331f, -28.1758f, 34.4903f), 1.61843f); 
  data[12].mBoundingSphere = Sphere(Vector3(-30.2984f, -25.7141f, 9.01221f), 2.82951f); 
  data[13].mBoundingSphere = Sphere(Vector3(-9.36997f, -11.4224f, -38.5302f), 1.95208f); 
  data[14].mBoundingSphere = Sphere(Vector3(35.3804f, 14.4691f, -4.73601f), 2.33385f); 
  data[15].mBoundingSphere = Sphere(Vector3(-3.48489f, 28.8571f, -10.7596f), 2.60488f); 
  data[16].mBoundingSphere = Sphere(Vector3(-12.8934f, -0.926247f, 6.58035f), 1.77151f); 
  data[17].mBoundingSphere = Sphere(Vector3(-38.4573f, -19.6831f, 1.18045f), 3.30801f); 
  data[18].mBoundingSphere = Sphere(Vector3(-13.6149f, -8.27118f, -5.14756f), 2.69023f); 
  data[19].mBoundingSphere = Sphere(Vector3(-42.4391f, 9.67575f, -9.50297f), 1.71469f); 
  data[20].mBoundingSphere = Sphere(Vector3(18.1474f, 6.08225f, -0.120913f), 3.6471f); 
  data[21].mBoundingSphere = Sphere(Vector3(-23.2216f, 5.21007f, 40.5587f), 2.14366f); 
  data[22].mBoundingSphere = Sphere(Vector3(-15.6401f, -35.9171f, -6.77132f), 1.58932f); 
  data[23].mBoundingSphere = Sphere(Vector3(41.1087f, -22.3452f, 2.77363f), 1.99684f); 
  data[24].mBoundingSphere = Sphere(Vector3(6.60036f, -28.6144f, -21.4833f), 1.30937f); 
  data[25].mBoundingSphere = Sphere(Vector3(13.0042f, -34.592f, -5.84045f), 1.45236f); 
  data[26].mBoundingSphere = Sphere(Vector3(-11.6885f, 32.7699f, 25.6274f), 2.57577f); 
  data[27].mBoundingSphere = Sphere(Vector3(14.0957f, 26.5999f, -23.4702f), 2.23284f); 
  data[28].mBoundingSphere = Sphere(Vector3(-6.9253f, -4.49236f, -23.9677f), 2.93818f); 
  data[29].mBoundingSphere = Sphere(Vector3(11.9668f, 40.0205f, -6.69683f), 3.04788f); 
  data[30].mBoundingSphere = Sphere(Vector3(3.96702f, -20.242f, 20.064f), 1.00785f); 
  data[31].mBoundingSphere = Sphere(Vector3(41.8707f, -23.2612f, -4.71055f), 3.1119f); 
  data[32].mBoundingSphere = Sphere(Vector3(11.8044f, -44.2936f, 15.3309f), 0.213684f); 
  data[33].mBoundingSphere = Sphere(Vector3(-38.2953f, 3.79411f, -18.1491f), 2.33803f); 
  data[34].mBoundingSphere = Sphere(Vector3(40.1409f, -10.9722f, 25.1429f), 1.63235f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-60.8324f, 57.6851f, 88.24f), Vector3(-8.42747f, 101.142f, -23.2433f), Vector3(-8.42747f, -23.8633f, -71.9713f), Vector3(-60.8324f, -67.3203f, 39.512f), Vector3(28.8177f, 9.91559f, 38.4021f), Vector3(30.9882f, 11.7155f, 33.7846f), Vector3(30.9882f, 3.3761f, 30.5339f), Vector3(28.8177f, 1.57619f, 35.1513f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(60); 
  data[0].mBoundingSphere = Sphere(Vector3(-15.6538f, -27.9841f, 23.3984f), 3.29119f); 
  data[1].mBoundingSphere = Sphere(Vector3(-19.4662f, 32.3109f, 8.57204f), 3.15887f); 
  data[2].mBoundingSphere = Sphere(Vector3(35.1317f, -8.43962f, -26.5631f), 0.78797f); 
  data[3].mBoundingSphere = Sphere(Vector3(21.3594f, -11.1214f, 28.5557f), 1.75563f); 
  data[4].mBoundingSphere = Sphere(Vector3(-8.55568f, 25.2498f, -39.3301f), 2.43115f); 
  data[5].mBoundingSphere = Sphere(Vector3(-3.04367f, 14.8576f, 37.6966f), 3.03547f); 
  data[6].mBoundingSphere = Sphere(Vector3(-4.14442f, -48.5883f, 5.0587f), 3.34082f); 
  data[7].mBoundingSphere = Sphere(Vector3(8.97864f, 13.4579f, 14.8417f), 2.02885f); 
  data[8].mBoundingSphere = Sphere(Vector3(-21.2318f, -12.671f, -0.254307f), 3.62739f); 
  data[9].mBoundingSphere = Sphere(Vector3(-2.45501f, -10.0309f, 1.14117f), 0.842012f); 
  data[10].mBoundingSphere = Sphere(Vector3(4.42291f, -31.8206f, 26.5015f), 0.805597f); 
  data[11].mBoundingSphere = Sphere(Vector3(-39.0947f, 18.5278f, -22.6518f), 2.25882f); 
  data[12].mBoundingSphere = Sphere(Vector3(-6.47844f, 37.0084f, -18.5416f), 3.58541f); 
  data[13].mBoundingSphere = Sphere(Vector3(13.8514f, 13.9775f, 25.2237f), 1.91798f); 
  data[14].mBoundingSphere = Sphere(Vector3(-2.52973f, -16.8146f, 40.8507f), 2.43521f); 
  data[15].mBoundingSphere = Sphere(Vector3(30.5349f, 21.0493f, -7.68311f), 1.8658f); 
  data[16].mBoundingSphere = Sphere(Vector3(40.8205f, 26.1087f, -12.2759f), 3.3232f); 
  data[17].mBoundingSphere = Sphere(Vector3(-22.2681f, 19.4942f, 13.302f), 3.6834f); 
  data[18].mBoundingSphere = Sphere(Vector3(-32.4333f, -4.90019f, -26.7377f), 3.0313f); 
  data[19].mBoundingSphere = Sphere(Vector3(13.5288f, 30.0631f, -21.0413f), 3.98863f); 
  data[20].mBoundingSphere = Sphere(Vector3(-11.0624f, 24.5133f, 1.96768f), 1.04554f); 
  data[21].mBoundingSphere = Sphere(Vector3(-7.10388f, 11.2907f, -28.1315f), 3.71402f); 
  data[22].mBoundingSphere = Sphere(Vector3(18.4302f, -2.06361f, -35.4557f), 2.4206f); 
  data[23].mBoundingSphere = Sphere(Vector3(46.6187f, 4.42477f, 16.6223f), 0.758977f); 
  data[24].mBoundingSphere = Sphere(Vector3(-39.1843f, 11.0547f, -5.40094f), 0.219599f); 
  data[25].mBoundingSphere = Sphere(Vector3(-27.9297f, 9.7407f, -3.09601f), 0.216816f); 
  data[26].mBoundingSphere = Sphere(Vector3(-21.0148f, 8.90387f, 6.95139f), 2.1294f); 
  data[27].mBoundingSphere = Sphere(Vector3(21.2608f, -32.3625f, 10.8209f), 0.377319f); 
  data[28].mBoundingSphere = Sphere(Vector3(20.1409f, -34.1003f, 30.4807f), 2.44971f); 
  data[29].mBoundingSphere = Sphere(Vector3(-9.09743f, 6.36313f, -42.8785f), 3.90444f); 
  data[30].mBoundingSphere = Sphere(Vector3(-18.9318f, 21.9739f, 8.12131f), 2.05866f); 
  data[31].mBoundingSphere = Sphere(Vector3(-39.0716f, 9.15265f, -28.477f), 0.987554f); 
  data[32].mBoundingSphere = Sphere(Vector3(2.56771f, 30.553f, 3.99628f), 0.351921f); 
  data[33].mBoundingSphere = Sphere(Vector3(3.1821f, -33.1377f, 5.19161f), 0.470327f); 
  data[34].mBoundingSphere = Sphere(Vector3(37.1787f, 1.90872f, -3.50679f), 0.958562f); 
  data[35].mBoundingSphere = Sphere(Vector3(9.30403f, -12.8652f, 9.09628f), 2.55176f); 
  data[36].mBoundingSphere = Sphere(Vector3(-20.1653f, -3.13771f, 36.3448f), 0.829719f); 
  data[37].mBoundingSphere = Sphere(Vector3(-25.7997f, -13.5815f, 21.1715f), 0.463948f); 
  data[38].mBoundingSphere = Sphere(Vector3(42.5682f, 2.36552f, -8.84052f), 0.704355f); 
  data[39].mBoundingSphere = Sphere(Vector3(0.272471f, -1.9331f, -23.7649f), 3.5251f); 
  data[40].mBoundingSphere = Sphere(Vector3(-4.2666f, 39.741f, 5.92018f), 2.74856f); 
  data[41].mBoundingSphere = Sphere(Vector3(37.8026f, -1.61384f, -17.3396f), 1.7656f); 
  data[42].mBoundingSphere = Sphere(Vector3(40.722f, 5.23294f, 25.3794f), 2.97076f); 
  data[43].mBoundingSphere = Sphere(Vector3(-32.4324f, 16.2556f, -21.0673f), 1.33152f); 
  data[44].mBoundingSphere = Sphere(Vector3(-22.0799f, 25.928f, 13.7806f), 1.21103f); 
  data[45].mBoundingSphere = Sphere(Vector3(28.0032f, -19.1618f, 23.4458f), 1.46129f); 
  data[46].mBoundingSphere = Sphere(Vector3(-9.70615f, 15.2059f, 40.9728f), 2.1069f); 
  data[47].mBoundingSphere = Sphere(Vector3(42.012f, 14.9464f, -7.27627f), 0.438203f); 
  data[48].mBoundingSphere = Sphere(Vector3(-19.0485f, -31.6313f, -32.2024f), 3.23193f); 
  data[49].mBoundingSphere = Sphere(Vector3(-30.4575f, -12.7946f, -25.0837f), 3.91801f); 
  data[50].mBoundingSphere = Sphere(Vector3(-46.1231f, -10.1349f, 13.5398f), 0.811743f); 
  data[51].mBoundingSphere = Sphere(Vector3(-21.5501f, -10.9464f, -16.5091f), 1.43068f); 
  data[52].mBoundingSphere = Sphere(Vector3(-35.0701f, -27.2082f, 22.2449f), 1.61298f); 
  data[53].mBoundingSphere = Sphere(Vector3(4.29066f, 16.0258f, 12.1988f), 2.25847f); 
  data[54].mBoundingSphere = Sphere(Vector3(23.5282f, 9.37022f, 22.0149f), 3.0088f); 
  data[55].mBoundingSphere = Sphere(Vector3(10.8631f, 36.1275f, -21.8438f), 1.48773f); 
  data[56].mBoundingSphere = Sphere(Vector3(1.8599f, -3.67335f, 12.9155f), 2.01818f); 
  data[57].mBoundingSphere = Sphere(Vector3(-12.2326f, -30.6942f, -13.8372f), 1.63652f); 
  data[58].mBoundingSphere = Sphere(Vector3(16.5213f, -13.0604f, -45.1976f), 2.92124f); 
  data[59].mBoundingSphere = Sphere(Vector3(23.8369f, 13.1947f, -27.5803f), 2.45667f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(3.971f, -46.6626f, -18.4929f), Vector3(-29.4307f, -29.8088f, 43.7408f), Vector3(-29.4307f, 36.5549f, 25.7686f), Vector3(3.971f, 19.7011f, -36.4651f), Vector3(46.3189f, -0.983369f, 28.8659f), Vector3(42.0911f, 1.14984f, 36.7429f), Vector3(42.0911f, 6.29569f, 35.3493f), Vector3(46.3189f, 4.16249f, 27.4723f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(42); 
  data[0].mBoundingSphere = Sphere(Vector3(4.78074f, 7.61621f, -47.838f), 2.89631f); 
  data[1].mBoundingSphere = Sphere(Vector3(11.4842f, -38.6495f, -24.4988f), 3.85968f); 
  data[2].mBoundingSphere = Sphere(Vector3(-7.74841f, -25.3192f, -2.88155f), 3.44647f); 
  data[3].mBoundingSphere = Sphere(Vector3(-10.6156f, -19.6711f, -5.48511f), 1.49794f); 
  data[4].mBoundingSphere = Sphere(Vector3(4.42308f, 18.4315f, -28.4159f), 2.13589f); 
  data[5].mBoundingSphere = Sphere(Vector3(14.5115f, 0.792441f, 25.8144f), 1.50385f); 
  data[6].mBoundingSphere = Sphere(Vector3(15.2171f, -12.7772f, 22.3633f), 3.7197f); 
  data[7].mBoundingSphere = Sphere(Vector3(-30.4255f, 36.1755f, -7.17741f), 3.88101f); 
  data[8].mBoundingSphere = Sphere(Vector3(4.38625f, 42.9903f, 19.5655f), 0.469747f); 
  data[9].mBoundingSphere = Sphere(Vector3(-12.4824f, 16.4985f, -41.1058f), 2.59908f); 
  data[10].mBoundingSphere = Sphere(Vector3(25.9008f, -0.921691f, -3.51607f), 3.24329f); 
  data[11].mBoundingSphere = Sphere(Vector3(16.2198f, -7.27389f, -8.10737f), 1.70031f); 
  data[12].mBoundingSphere = Sphere(Vector3(-0.706853f, -5.17641f, -26.6512f), 0.824732f); 
  data[13].mBoundingSphere = Sphere(Vector3(-0.749808f, 36.2845f, 6.45623f), 1.47904f); 
  data[14].mBoundingSphere = Sphere(Vector3(17.2292f, -0.368424f, 10.8282f), 3.34477f); 
  data[15].mBoundingSphere = Sphere(Vector3(16.3615f, 31.8433f, -28.3568f), 0.337657f); 
  data[16].mBoundingSphere = Sphere(Vector3(14.9804f, 31.05f, 26.0559f), 2.57229f); 
  data[17].mBoundingSphere = Sphere(Vector3(-22.1999f, 17.6291f, -34.8078f), 0.916581f); 
  data[18].mBoundingSphere = Sphere(Vector3(32.9582f, -19.7852f, 0.205304f), 1.08346f); 
  data[19].mBoundingSphere = Sphere(Vector3(10.4244f, 29.9039f, -22.0181f), 1.50049f); 
  data[20].mBoundingSphere = Sphere(Vector3(16.1263f, 29.2157f, -26.4618f), 3.90398f); 
  data[21].mBoundingSphere = Sphere(Vector3(18.4744f, -28.5846f, -6.45209f), 3.0981f); 
  data[22].mBoundingSphere = Sphere(Vector3(17.9388f, -8.33993f, 23.6399f), 3.02678f); 
  data[23].mBoundingSphere = Sphere(Vector3(-5.87924f, -2.20857f, 24.0165f), 1.79772f); 
  data[24].mBoundingSphere = Sphere(Vector3(-29.7863f, -3.0838f, -22.0295f), 0.405963f); 
  data[25].mBoundingSphere = Sphere(Vector3(11.6038f, 23.2153f, -40.0158f), 0.598358f); 
  data[26].mBoundingSphere = Sphere(Vector3(-46.9544f, -6.63431f, -11.1799f), 1.02397f); 
  data[27].mBoundingSphere = Sphere(Vector3(-23.0882f, 19.9788f, -17.1022f), 2.71424f); 
  data[28].mBoundingSphere = Sphere(Vector3(-29.816f, 25.8304f, -6.06223f), 2.63491f); 
  data[29].mBoundingSphere = Sphere(Vector3(-16.2978f, -13.65f, -16.1494f), 1.55616f); 
  data[30].mBoundingSphere = Sphere(Vector3(-37.9962f, -20.9421f, -14.0036f), 1.18215f); 
  data[31].mBoundingSphere = Sphere(Vector3(-38.1199f, -2.81198f, 4.36175f), 1.58619f); 
  data[32].mBoundingSphere = Sphere(Vector3(-38.2284f, 19.2524f, 9.07336f), 0.491897f); 
  data[33].mBoundingSphere = Sphere(Vector3(14.8277f, 13.6666f, 27.3663f), 1.6487f); 
  data[34].mBoundingSphere = Sphere(Vector3(-44.462f, -10.4064f, 8.50677f), 3.44068f); 
  data[35].mBoundingSphere = Sphere(Vector3(-29.9677f, -17.1522f, 26.6725f), 1.57576f); 
  data[36].mBoundingSphere = Sphere(Vector3(12.968f, -11.5229f, -7.80971f), 3.59828f); 
  data[37].mBoundingSphere = Sphere(Vector3(-18.8337f, -6.98446f, -44.183f), 1.66714f); 
  data[38].mBoundingSphere = Sphere(Vector3(-10.2561f, 0.639983f, -19.722f), 3.94619f); 
  data[39].mBoundingSphere = Sphere(Vector3(4.35893f, 12.4657f, 24.2217f), 2.64651f); 
  data[40].mBoundingSphere = Sphere(Vector3(-2.07949f, -35.0832f, -1.88075f), 1.86997f); 
  data[41].mBoundingSphere = Sphere(Vector3(4.32847f, -15.8144f, 12.5669f), 2.29118f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-81.8643f, -12.6721f, 8.72603f), Vector3(14.2567f, -70.5252f, 68.9857f), Vector3(79.6266f, 38.0846f, 68.9857f), Vector3(-16.4944f, 95.9377f, 8.72603f), Vector3(27.4253f, -9.91242f, -38.8541f), Vector3(34.8995f, -14.411f, -34.1684f), Vector3(39.7053f, -6.42634f, -34.1684f), Vector3(32.2311f, -1.92775f, -38.8541f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(62); 
  data[0].mBoundingSphere = Sphere(Vector3(15.5644f, -35.8843f, 6.41482f), 1.44912f); 
  data[1].mBoundingSphere = Sphere(Vector3(7.70645f, 22.1754f, -36.3568f), 1.57506f); 
  data[2].mBoundingSphere = Sphere(Vector3(38.3959f, -14.5456f, 6.01683f), 1.81929f); 
  data[3].mBoundingSphere = Sphere(Vector3(-31.3744f, -13.7426f, 23.0739f), 1.91543f); 
  data[4].mBoundingSphere = Sphere(Vector3(5.6594f, -20.6771f, -43.6626f), 1.82289f); 
  data[5].mBoundingSphere = Sphere(Vector3(-31.9667f, 16.1758f, 25.331f), 1.84191f); 
  data[6].mBoundingSphere = Sphere(Vector3(12.5089f, -9.30928f, 39.9566f), 2.46977f); 
  data[7].mBoundingSphere = Sphere(Vector3(-7.28696f, -14.4572f, -38.895f), 2.2151f); 
  data[8].mBoundingSphere = Sphere(Vector3(4.70119f, 7.02898f, -30.195f), 1.09065f); 
  data[9].mBoundingSphere = Sphere(Vector3(0.590102f, 4.93813f, 38.6456f), 0.40202f); 
  data[10].mBoundingSphere = Sphere(Vector3(14.968f, 23.5436f, 34.344f), 2.75228f); 
  data[11].mBoundingSphere = Sphere(Vector3(38.9627f, -22.9247f, 19.7732f), 2.87915f); 
  data[12].mBoundingSphere = Sphere(Vector3(-10.2469f, -15.2794f, -29.2582f), 3.6471f); 
  data[13].mBoundingSphere = Sphere(Vector3(-34.6587f, 4.09957f, -5.44052f), 2.20884f); 
  data[14].mBoundingSphere = Sphere(Vector3(17.1234f, 17.506f, -5.87071f), 3.4583f); 
  data[15].mBoundingSphere = Sphere(Vector3(0.594756f, -19.8448f, 42.1327f), 1.99209f); 
  data[16].mBoundingSphere = Sphere(Vector3(-3.40674f, 0.0653338f, -49.2946f), 1.09923f); 
  data[17].mBoundingSphere = Sphere(Vector3(37.0653f, -25.2273f, 0.359881f), 2.39625f); 
  data[18].mBoundingSphere = Sphere(Vector3(16.1617f, -3.53343f, -10.0747f), 0.428809f); 
  data[19].mBoundingSphere = Sphere(Vector3(13.5079f, -21.9093f, -14.3463f), 3.25675f); 
  data[20].mBoundingSphere = Sphere(Vector3(25.8299f, -5.9333f, 29.8669f), 3.14112f); 
  data[21].mBoundingSphere = Sphere(Vector3(-9.11234f, -43.5882f, 0.977362f), 1.9544f); 
  data[22].mBoundingSphere = Sphere(Vector3(2.7223f, 31.5938f, -38.3079f), 0.229688f); 
  data[23].mBoundingSphere = Sphere(Vector3(-35.3735f, -28.3568f, -2.17059f), 1.26878f); 
  data[24].mBoundingSphere = Sphere(Vector3(31.0415f, 9.56307f, -23.27f), 3.82871f); 
  data[25].mBoundingSphere = Sphere(Vector3(1.40532f, 43.0317f, 16.4825f), 0.604157f); 
  data[26].mBoundingSphere = Sphere(Vector3(-34.9359f, -3.41394f, 11.8229f), 1.6262f); 
  data[27].mBoundingSphere = Sphere(Vector3(1.63833f, -0.767034f, 28.9007f), 0.992889f); 
  data[28].mBoundingSphere = Sphere(Vector3(16.6848f, 7.55171f, -29.3052f), 1.05528f); 
  data[29].mBoundingSphere = Sphere(Vector3(-32.8166f, -10.7857f, -12.4765f), 1.07233f); 
  data[30].mBoundingSphere = Sphere(Vector3(36.4201f, 5.29914f, 11.1249f), 3.65824f); 
  data[31].mBoundingSphere = Sphere(Vector3(-2.6509f, -14.6951f, 15.8543f), 3.03698f); 
  data[32].mBoundingSphere = Sphere(Vector3(-11.7802f, 36.58f, -24.9743f), 0.82624f); 
  data[33].mBoundingSphere = Sphere(Vector3(2.23788f, 0.865001f, -41.7391f), 1.96634f); 
  data[34].mBoundingSphere = Sphere(Vector3(0.559568f, 22.229f, -33.1534f), 2.31507f); 
  data[35].mBoundingSphere = Sphere(Vector3(-20.5046f, -9.76093f, 27.4017f), 0.326872f); 
  data[36].mBoundingSphere = Sphere(Vector3(-29.9772f, -21.4156f, -24.6333f), 2.53402f); 
  data[37].mBoundingSphere = Sphere(Vector3(17.955f, 38.0131f, 5.46106f), 2.04973f); 
  data[38].mBoundingSphere = Sphere(Vector3(-41.5247f, -17.7254f, -12.7903f), 2.41121f); 
  data[39].mBoundingSphere = Sphere(Vector3(-5.70256f, 6.40971f, 5.69009f), 1.38232f); 
  data[40].mBoundingSphere = Sphere(Vector3(42.8391f, 6.9563f, 4.35154f), 3.23332f); 
  data[41].mBoundingSphere = Sphere(Vector3(36.6943f, -15.6926f, -11.8889f), 0.716648f); 
  data[42].mBoundingSphere = Sphere(Vector3(18.7095f, -18.6853f, -7.74354f), 2.86024f); 
  data[43].mBoundingSphere = Sphere(Vector3(34.1051f, -19.1615f, -19.3918f), 0.391119f); 
  data[44].mBoundingSphere = Sphere(Vector3(-34.3557f, 6.38308f, 21.028f), 1.91543f); 
  data[45].mBoundingSphere = Sphere(Vector3(-16.051f, -9.43782f, -7.94601f), 2.26764f); 
  data[46].mBoundingSphere = Sphere(Vector3(7.2563f, 36.2878f, 5.16683f), 0.950444f); 
  data[47].mBoundingSphere = Sphere(Vector3(-24.0119f, -15.2253f, -13.9425f), 0.484359f); 
  data[48].mBoundingSphere = Sphere(Vector3(-4.78642f, 25.3356f, 3.88248f), 3.78453f); 
  data[49].mBoundingSphere = Sphere(Vector3(0.443631f, 18.0357f, -41.6636f), 1.18331f); 
  data[50].mBoundingSphere = Sphere(Vector3(-20.7209f, 14.6236f, -15.9381f), 0.419764f); 
  data[51].mBoundingSphere = Sphere(Vector3(16.1589f, -9.62886f, 37.8357f), 2.81188f); 
  data[52].mBoundingSphere = Sphere(Vector3(12.4707f, -11.9813f, -43.7956f), 1.73127f); 
  data[53].mBoundingSphere = Sphere(Vector3(-5.36344f, -0.884126f, -11.9894f), 1.60069f); 
  data[54].mBoundingSphere = Sphere(Vector3(-29.1493f, -29.5219f, -2.96908f), 0.530979f); 
  data[55].mBoundingSphere = Sphere(Vector3(2.34429f, 15.157f, 14.82f), 3.21639f); 
  data[56].mBoundingSphere = Sphere(Vector3(-22.2446f, 16.8045f, 6.05902f), 3.40867f); 
  data[57].mBoundingSphere = Sphere(Vector3(23.1213f, 28.5134f, 21.4326f), 2.58771f); 
  data[58].mBoundingSphere = Sphere(Vector3(30.1029f, 24.1933f, -10.8559f), 0.346007f); 
  data[59].mBoundingSphere = Sphere(Vector3(-2.83419f, 14.6325f, -25.0481f), 0.309128f); 
  data[60].mBoundingSphere = Sphere(Vector3(-11.175f, 36.7682f, -15.6328f), 1.30694f); 
  data[61].mBoundingSphere = Sphere(Vector3(-27.8328f, -8.321f, 11.9001f), 2.4177f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(20.1041f, 13.5037f, -64.6238f), Vector3(-25.0262f, -56.4566f, -32.1508f), Vector3(-25.0262f, -19.0143f, 48.5157f), Vector3(20.1041f, 50.9461f, 16.0428f), Vector3(28.0609f, -15.9407f, -6.81324f), Vector3(23.3735f, -23.2071f, -3.44044f), Vector3(23.3735f, -19.4804f, 4.58852f), Vector3(28.0609f, -12.2139f, 1.21572f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(61); 
  data[0].mBoundingSphere = Sphere(Vector3(33.1719f, 22.7641f, -26.7511f), 0.408747f); 
  data[1].mBoundingSphere = Sphere(Vector3(6.19569f, -14.4511f, 41.4299f), 0.403296f); 
  data[2].mBoundingSphere = Sphere(Vector3(-0.607668f, -4.37494f, -29.2225f), 0.274453f); 
  data[3].mBoundingSphere = Sphere(Vector3(-0.495756f, 38.7298f, 26.0966f), 3.93958f); 
  data[4].mBoundingSphere = Sphere(Vector3(10.9858f, 3.36829f, -19.7298f), 1.98142f); 
  data[5].mBoundingSphere = Sphere(Vector3(13.4608f, 23.972f, -27.4293f), 0.830183f); 
  data[6].mBoundingSphere = Sphere(Vector3(-27.1682f, -19.0802f, -32.1884f), 1.95753f); 
  data[7].mBoundingSphere = Sphere(Vector3(15.6734f, -5.95824f, -32.9052f), 1.97968f); 
  data[8].mBoundingSphere = Sphere(Vector3(-27.3395f, -15.9345f, -30.324f), 2.8614f); 
  data[9].mBoundingSphere = Sphere(Vector3(39.2371f, 13.3602f, -0.418726f), 0.5137f); 
  data[10].mBoundingSphere = Sphere(Vector3(17.5203f, -11.2154f, -25.3972f), 1.7823f); 
  data[11].mBoundingSphere = Sphere(Vector3(34.1279f, -16.8971f, 1.48293f), 3.91615f); 
  data[12].mBoundingSphere = Sphere(Vector3(22.9612f, -39.8817f, -17.2567f), 1.71399f); 
  data[13].mBoundingSphere = Sphere(Vector3(-41.9156f, 24.5486f, -8.50472f), 2.04022f); 
  data[14].mBoundingSphere = Sphere(Vector3(-0.789622f, 17.8341f, 2.78396f), 0.704007f); 
  data[15].mBoundingSphere = Sphere(Vector3(1.0679f, -0.911086f, 22.0828f), 3.99641f); 
  data[16].mBoundingSphere = Sphere(Vector3(-31.5335f, -35.8294f, 1.69805f), 0.477981f); 
  data[17].mBoundingSphere = Sphere(Vector3(5.2518f, -1.58932f, -10.4601f), 3.31276f); 
  data[18].mBoundingSphere = Sphere(Vector3(-18.1676f, -1.03927f, -18.961f), 0.338585f); 
  data[19].mBoundingSphere = Sphere(Vector3(12.2321f, -29.3823f, 37.7985f), 0.572497f); 
  data[20].mBoundingSphere = Sphere(Vector3(-12.6425f, -22.4089f, -5.61882f), 2.94003f); 
  data[21].mBoundingSphere = Sphere(Vector3(2.38511f, 17.5293f, 39.4863f), 3.72585f); 
  data[22].mBoundingSphere = Sphere(Vector3(18.1944f, -7.9134f, 33.1462f), 1.53725f); 
  data[23].mBoundingSphere = Sphere(Vector3(21.1571f, 20.4677f, 16.8763f), 2.27587f); 
  data[24].mBoundingSphere = Sphere(Vector3(-13.104f, -6.53336f, 32.6169f), 1.90987f); 
  data[25].mBoundingSphere = Sphere(Vector3(11.0902f, -34.884f, 10.9909f), 2.74114f); 
  data[26].mBoundingSphere = Sphere(Vector3(-18.7537f, 34.9915f, -25.013f), 2.96183f); 
  data[27].mBoundingSphere = Sphere(Vector3(1.14538f, -4.14105f, 22.1041f), 3.79207f); 
  data[28].mBoundingSphere = Sphere(Vector3(17.1897f, 9.4937f, -16.1858f), 0.595111f); 
  data[29].mBoundingSphere = Sphere(Vector3(-22.5134f, 34.6952f, -20.3168f), 2.26218f); 
  data[30].mBoundingSphere = Sphere(Vector3(11.4315f, 12.9163f, -19.7816f), 3.45969f); 
  data[31].mBoundingSphere = Sphere(Vector3(-24.2038f, 2.63256f, -2.86768f), 3.4358f); 
  data[32].mBoundingSphere = Sphere(Vector3(18.5524f, 19.8003f, 8.05495f), 3.57682f); 
  data[33].mBoundingSphere = Sphere(Vector3(7.467f, -43.2165f, -22.0543f), 1.40887f); 
  data[34].mBoundingSphere = Sphere(Vector3(-19.0746f, -9.59254f, -33.964f), 0.638832f); 
  data[35].mBoundingSphere = Sphere(Vector3(-1.41894f, -2.13435f, 28.4672f), 1.7264f); 
  data[36].mBoundingSphere = Sphere(Vector3(26.8018f, 0.521711f, -29.6425f), 3.98365f); 
  data[37].mBoundingSphere = Sphere(Vector3(-13.8902f, 10.6404f, 27.084f), 2.69348f); 
  data[38].mBoundingSphere = Sphere(Vector3(3.80517f, -34.7215f, 7.5777f), 0.842824f); 
  data[39].mBoundingSphere = Sphere(Vector3(-6.05901f, -24.3117f, -14.7211f), 3.76238f); 
  data[40].mBoundingSphere = Sphere(Vector3(-33.3126f, 12.7698f, -17.4318f), 1.06734f); 
  data[41].mBoundingSphere = Sphere(Vector3(3.92399f, -0.446986f, -31.043f), 2.99442f); 
  data[42].mBoundingSphere = Sphere(Vector3(17.054f, -20.5354f, 31.4417f), 2.52567f); 
  data[43].mBoundingSphere = Sphere(Vector3(14.9712f, 10.7933f, -26.6841f), 2.59641f); 
  data[44].mBoundingSphere = Sphere(Vector3(2.35629f, -13.9075f, -10.3161f), 2.5782f); 
  data[45].mBoundingSphere = Sphere(Vector3(40.1936f, -16.3207f, -4.79108f), 3.53252f); 
  data[46].mBoundingSphere = Sphere(Vector3(23.1355f, 31.5809f, -11.8309f), 2.42698f); 
  data[47].mBoundingSphere = Sphere(Vector3(-44.6923f, 12.5994f, -13.0525f), 0.610535f); 
  data[48].mBoundingSphere = Sphere(Vector3(25.4823f, 8.55146f, -34.2991f), 0.538981f); 
  data[49].mBoundingSphere = Sphere(Vector3(-21.7378f, 13.1546f, 37.2134f), 0.844795f); 
  data[50].mBoundingSphere = Sphere(Vector3(10.206f, 36.3291f, -8.49574f), 1.7467f); 
  data[51].mBoundingSphere = Sphere(Vector3(-2.57359f, -15.0585f, -0.63041f), 0.491318f); 
  data[52].mBoundingSphere = Sphere(Vector3(14.8172f, 19.7375f, 35.4311f), 0.220875f); 
  data[53].mBoundingSphere = Sphere(Vector3(-25.3568f, 13.3297f, -2.65244f), 0.560668f); 
  data[54].mBoundingSphere = Sphere(Vector3(20.9966f, 0.477183f, 24.6014f), 0.920756f); 
  data[55].mBoundingSphere = Sphere(Vector3(-1.42882f, 9.83001f, -28.8941f), 3.80447f); 
  data[56].mBoundingSphere = Sphere(Vector3(14.4309f, -9.28858f, 20.973f), 3.51803f); 
  data[57].mBoundingSphere = Sphere(Vector3(24.3905f, -3.38659f, -29.4202f), 0.776141f); 
  data[58].mBoundingSphere = Sphere(Vector3(30.3935f, 7.51131f, 36.2725f), 1.99545f); 
  data[59].mBoundingSphere = Sphere(Vector3(44.3866f, 10.1242f, -7.66617f), 1.96948f); 
  data[60].mBoundingSphere = Sphere(Vector3(-3.70332f, 32.4415f, -16.1943f), 3.72225f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-51.7127f, -97.8463f, -6.48696f), Vector3(63.0913f, -38.0422f, 56.1214f), Vector3(-3.36095f, 89.5238f, 56.1214f), Vector3(-118.165f, 29.7197f, -6.48696f), Vector3(0.00122023f, 5.05344f, -44.8805f), Vector3(7.23173f, 8.81999f, -40.9374f), Vector3(3.02737f, 16.8909f, -40.9374f), Vector3(-4.20313f, 13.1244f, -44.8805f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(39); 
  data[0].mBoundingSphere = Sphere(Vector3(-28.8785f, 15.8591f, 9.84905f), 3.5055f); 
  data[1].mBoundingSphere = Sphere(Vector3(-1.9133f, 28.0832f, -35.985f), 3.95697f); 
  data[2].mBoundingSphere = Sphere(Vector3(-22.0716f, -31.1758f, 10.047f), 2.81838f); 
  data[3].mBoundingSphere = Sphere(Vector3(-17.459f, -37.2308f, 3.25673f), 3.81584f); 
  data[4].mBoundingSphere = Sphere(Vector3(-32.1251f, 33.6273f, 0.120642f), 0.949632f); 
  data[5].mBoundingSphere = Sphere(Vector3(40.6194f, -6.66784f, 27.2836f), 0.433448f); 
  data[6].mBoundingSphere = Sphere(Vector3(-37.1478f, 12.3198f, -4.30784f), 2.2964f); 
  data[7].mBoundingSphere = Sphere(Vector3(-14.2929f, -6.57008f, -23.9886f), 1.61484f); 
  data[8].mBoundingSphere = Sphere(Vector3(-19.179f, -4.30694f, 30.6459f), 2.84528f); 
  data[9].mBoundingSphere = Sphere(Vector3(-19.8594f, -2.83706f, -40.1309f), 0.824152f); 
  data[10].mBoundingSphere = Sphere(Vector3(29.4918f, 9.12903f, 11.5631f), 1.70611f); 
  data[11].mBoundingSphere = Sphere(Vector3(-13.1178f, -7.08642f, 25.6289f), 3.20653f); 
  data[12].mBoundingSphere = Sphere(Vector3(-7.80238f, 34.1694f, -9.36791f), 3.62055f); 
  data[13].mBoundingSphere = Sphere(Vector3(-20.6286f, 32.7031f, 4.03263f), 0.783563f); 
  data[14].mBoundingSphere = Sphere(Vector3(-17.3449f, -30.2674f, 32.0298f), 1.04566f); 
  data[15].mBoundingSphere = Sphere(Vector3(-23.892f, -10.7173f, 34.5893f), 3.22509f); 
  data[16].mBoundingSphere = Sphere(Vector3(14.2188f, 9.38906f, 37.5227f), 0.412458f); 
  data[17].mBoundingSphere = Sphere(Vector3(-28.1f, -23.2884f, -15.4117f), 2.59421f); 
  data[18].mBoundingSphere = Sphere(Vector3(-31.8158f, 13.9214f, -33.7114f), 1.35994f); 
  data[19].mBoundingSphere = Sphere(Vector3(38.2242f, -11.3678f, -20.8624f), 1.26959f); 
  data[20].mBoundingSphere = Sphere(Vector3(27.4742f, 36.4518f, 3.42677f), 1.14249f); 
  data[21].mBoundingSphere = Sphere(Vector3(21.0785f, -16.0095f, 32.2962f), 0.44087f); 
  data[22].mBoundingSphere = Sphere(Vector3(39.5434f, 2.3724f, 13.0482f), 1.41502f); 
  data[23].mBoundingSphere = Sphere(Vector3(-11.2695f, 15.624f, 32.8579f), 2.37989f); 
  data[24].mBoundingSphere = Sphere(Vector3(27.9179f, -2.639f, 30.0953f), 2.1904f); 
  data[25].mBoundingSphere = Sphere(Vector3(3.46176f, -42.2592f, 2.93275f), 3.68665f); 
  data[26].mBoundingSphere = Sphere(Vector3(2.41991f, 9.7537f, -3.3999f), 1.97377f); 
  data[27].mBoundingSphere = Sphere(Vector3(13.2156f, 27.1654f, 0.417683f), 2.31054f); 
  data[28].mBoundingSphere = Sphere(Vector3(32.414f, 27.9563f, 18.5057f), 0.270394f); 
  data[29].mBoundingSphere = Sphere(Vector3(-21.8848f, 15.8061f, -13.3044f), 3.14681f); 
  data[30].mBoundingSphere = Sphere(Vector3(-27.521f, -33.8263f, -23.6455f), 2.47024f); 
  data[31].mBoundingSphere = Sphere(Vector3(-13.402f, 38.4337f, -2.66276f), 3.20688f); 
  data[32].mBoundingSphere = Sphere(Vector3(-12.273f, -13.859f, 30.6948f), 0.505582f); 
  data[33].mBoundingSphere = Sphere(Vector3(-0.216824f, 18.0191f, -31.4427f), 1.96484f); 
  data[34].mBoundingSphere = Sphere(Vector3(-13.8124f, 35.1867f, 14.5108f), 1.48518f); 
  data[35].mBoundingSphere = Sphere(Vector3(-20.5719f, -29.9498f, -19.4904f), 2.67887f); 
  data[36].mBoundingSphere = Sphere(Vector3(27.3843f, -28.2227f, 17.9479f), 1.70298f); 
  data[37].mBoundingSphere = Sphere(Vector3(-8.81495f, -10.3071f, 16.1881f), 0.88666f); 
  data[38].mBoundingSphere = Sphere(Vector3(-1.83045f, 32.0249f, -27.9251f), 1.02084f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-41.04f, -26.8453f, 4.17312f), Vector3(27.5862f, -49, 40.4492f), Vector3(53.812f, 32.2364f, 40.4492f), Vector3(-14.8142f, 54.3912f, 4.17312f), Vector3(22.0407f, -7.25372f, -18.5833f), Vector3(26.0163f, -8.53717f, -16.4818f), Vector3(28.8787f, 0.329589f, -16.4818f), Vector3(24.9032f, 1.61304f, -18.5833f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(73); 
  data[0].mBoundingSphere = Sphere(Vector3(-3.92215f, 39.1475f, 16.1138f), 2.82684f); 
  data[1].mBoundingSphere = Sphere(Vector3(-16.7537f, 42.1089f, -8.37571f), 1.40725f); 
  data[2].mBoundingSphere = Sphere(Vector3(33.2309f, 16.9318f, 14.8695f), 0.751671f); 
  data[3].mBoundingSphere = Sphere(Vector3(47.5673f, 3.09187f, 13.2624f), 3.71564f); 
  data[4].mBoundingSphere = Sphere(Vector3(-18.8438f, -7.54862f, -6.11629f), 2.44426f); 
  data[5].mBoundingSphere = Sphere(Vector3(44.7877f, -5.65067f, 3.37787f), 3.75066f); 
  data[6].mBoundingSphere = Sphere(Vector3(23.0237f, -18.5693f, 23.0867f), 0.409558f); 
  data[7].mBoundingSphere = Sphere(Vector3(1.51578f, 1.5086f, -37.2042f), 3.91314f); 
  data[8].mBoundingSphere = Sphere(Vector3(6.35588f, -8.50718f, 41.2636f), 1.52937f); 
  data[9].mBoundingSphere = Sphere(Vector3(-14.9627f, -7.57865f, -12.1715f), 2.61091f); 
  data[10].mBoundingSphere = Sphere(Vector3(7.24741f, -1.53216f, 1.87913f), 2.79263f); 
  data[11].mBoundingSphere = Sphere(Vector3(22.7724f, 23.4246f, -31.1376f), 0.627583f); 
  data[12].mBoundingSphere = Sphere(Vector3(-1.43131f, 25.914f, -8.92444f), 1.70066f); 
  data[13].mBoundingSphere = Sphere(Vector3(-9.41134f, 22.6741f, -9.95283f), 0.365374f); 
  data[14].mBoundingSphere = Sphere(Vector3(-21.7706f, -22.0489f, 26.4139f), 3.34813f); 
  data[15].mBoundingSphere = Sphere(Vector3(23.6686f, 14.0792f, -7.03044f), 1.76247f); 
  data[16].mBoundingSphere = Sphere(Vector3(19.3729f, -28.0139f, -26.1346f), 1.37791f); 
  data[17].mBoundingSphere = Sphere(Vector3(22.9808f, 28.152f, -31.1861f), 2.48659f); 
  data[18].mBoundingSphere = Sphere(Vector3(12.9909f, 45.4712f, -6.17268f), 1.21196f); 
  data[19].mBoundingSphere = Sphere(Vector3(13.3787f, -16.7533f, 8.31081f), 2.40286f); 
  data[20].mBoundingSphere = Sphere(Vector3(0.086938f, -5.19589f, 18.6208f), 1.95683f); 
  data[21].mBoundingSphere = Sphere(Vector3(21.9951f, -26.4336f, -31.5996f), 0.916697f); 
  data[22].mBoundingSphere = Sphere(Vector3(17.4995f, 35.3659f, -5.24926f), 1.58376f); 
  data[23].mBoundingSphere = Sphere(Vector3(3.08019f, 14.6706f, -1.64348f), 2.39172f); 
  data[24].mBoundingSphere = Sphere(Vector3(-16.8565f, 40.2403f, -10.254f), 2.5847f); 
  data[25].mBoundingSphere = Sphere(Vector3(0.00734873f, 30.6903f, -12.1147f), 3.04301f); 
  data[26].mBoundingSphere = Sphere(Vector3(-5.66484f, 44.2003f, -11.0896f), 2.90188f); 
  data[27].mBoundingSphere = Sphere(Vector3(17.2235f, -2.17973f, -44.3335f), 1.10445f); 
  data[28].mBoundingSphere = Sphere(Vector3(2.47f, 5.72169f, 31.685f), 2.53448f); 
  data[29].mBoundingSphere = Sphere(Vector3(4.03929f, 10.6148f, 38.7309f), 3.6725f); 
  data[30].mBoundingSphere = Sphere(Vector3(-14.324f, -10.6025f, -45.5087f), 3.0182f); 
  data[31].mBoundingSphere = Sphere(Vector3(33.9343f, 34.9062f, -7.63827f), 3.62368f); 
  data[32].mBoundingSphere = Sphere(Vector3(-28.3757f, 4.76142f, 37.6031f), 3.58088f); 
  data[33].mBoundingSphere = Sphere(Vector3(13.5457f, -43.443f, -7.67426f), 2.60731f); 
  data[34].mBoundingSphere = Sphere(Vector3(1.53961f, -5.22547f, 32.8297f), 1.5732f); 
  data[35].mBoundingSphere = Sphere(Vector3(21.8924f, -10.1168f, 20.1953f), 0.314347f); 
  data[36].mBoundingSphere = Sphere(Vector3(17.7262f, -22.7648f, -9.19854f), 0.974218f); 
  data[37].mBoundingSphere = Sphere(Vector3(-37.9094f, -7.84363f, -4.56219f), 3.2194f); 
  data[38].mBoundingSphere = Sphere(Vector3(24.6514f, 21.1461f, -13.804f), 2.37143f); 
  data[39].mBoundingSphere = Sphere(Vector3(29.4633f, 16.1619f, -14.4358f), 3.39927f); 
  data[40].mBoundingSphere = Sphere(Vector3(-12.932f, -44.2512f, -14.0351f), 1.01643f); 
  data[41].mBoundingSphere = Sphere(Vector3(-6.78979f, 10.9305f, -29.6414f), 3.42571f); 
  data[42].mBoundingSphere = Sphere(Vector3(-25.6497f, -12.6475f, -39.0042f), 0.555565f); 
  data[43].mBoundingSphere = Sphere(Vector3(15.3923f, -44.7337f, -7.03995f), 3.73338f); 
  data[44].mBoundingSphere = Sphere(Vector3(5.74339f, 28.5794f, -11.3152f), 0.635353f); 
  data[45].mBoundingSphere = Sphere(Vector3(0.276578f, -3.47454f, -31.562f), 1.8629f); 
  data[46].mBoundingSphere = Sphere(Vector3(20.192f, 11.1771f, -16.9325f), 3.98272f); 
  data[47].mBoundingSphere = Sphere(Vector3(18.4548f, -0.571696f, 19.4475f), 3.57903f); 
  data[48].mBoundingSphere = Sphere(Vector3(-39.4461f, -10.179f, 14.0495f), 1.16023f); 
  data[49].mBoundingSphere = Sphere(Vector3(14.862f, 8.20438f, 1.25564f), 0.262856f); 
  data[50].mBoundingSphere = Sphere(Vector3(-38.5554f, -12.4673f, 28.9609f), 1.39751f); 
  data[51].mBoundingSphere = Sphere(Vector3(-6.70747f, 10.1145f, -44.1898f), 3.78093f); 
  data[52].mBoundingSphere = Sphere(Vector3(31.1585f, -27.0721f, -4.1872f), 3.09613f); 
  data[53].mBoundingSphere = Sphere(Vector3(-24.9313f, 13.7101f, 38.4883f), 2.02711f); 
  data[54].mBoundingSphere = Sphere(Vector3(14.0121f, 2.67152f, 43.7215f), 2.80956f); 
  data[55].mBoundingSphere = Sphere(Vector3(-35.1408f, 11.572f, 12.5115f), 2.34151f); 
  data[56].mBoundingSphere = Sphere(Vector3(-4.34487f, -7.85545f, -44.9691f), 0.899533f); 
  data[57].mBoundingSphere = Sphere(Vector3(12.4492f, -10.4328f, -43.5248f), 3.20247f); 
  data[58].mBoundingSphere = Sphere(Vector3(22.5672f, -41.1076f, -4.03497f), 3.86617f); 
  data[59].mBoundingSphere = Sphere(Vector3(34.351f, -9.53132f, 3.13775f), 0.595227f); 
  data[60].mBoundingSphere = Sphere(Vector3(-24.3377f, -24.7151f, 4.68837f), 0.553014f); 
  data[61].mBoundingSphere = Sphere(Vector3(-10.597f, -31.2938f, -31.1002f), 2.45504f); 
  data[62].mBoundingSphere = Sphere(Vector3(-15.4424f, -19.4558f, -25.7138f), 3.42873f); 
  data[63].mBoundingSphere = Sphere(Vector3(-8.24086f, -16.2488f, -46.4793f), 1.78439f); 
  data[64].mBoundingSphere = Sphere(Vector3(30.6313f, -12.3764f, 34.7789f), 2.94061f); 
  data[65].mBoundingSphere = Sphere(Vector3(-9.83369f, 42.28f, -5.70403f), 3.44659f); 
  data[66].mBoundingSphere = Sphere(Vector3(-17.0527f, -32.6095f, 22.3273f), 2.14714f); 
  data[67].mBoundingSphere = Sphere(Vector3(-9.86039f, -33.7888f, 18.6526f), 2.03338f); 
  data[68].mBoundingSphere = Sphere(Vector3(-22.8285f, 26.6307f, 23.7079f), 2.42199f); 
  data[69].mBoundingSphere = Sphere(Vector3(-41.1961f, 14.332f, 6.05982f), 2.00821f); 
  data[70].mBoundingSphere = Sphere(Vector3(1.21418f, -23.0671f, 0.157208f), 1.03522f); 
  data[71].mBoundingSphere = Sphere(Vector3(-25.9178f, -10.6138f, -35.8627f), 0.324088f); 
  data[72].mBoundingSphere = Sphere(Vector3(-7.75384f, 10.6334f, 36.7324f), 0.608447f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-154.709f, -108.944f, 144.303f), Vector3(58.3593f, -197.275f, -61.5633f), Vector3(175.547f, 85.4028f, -61.5633f), Vector3(-37.5212f, 173.734f, 144.303f), Vector3(-53.1238f, 10.9998f, -26.8408f), Vector3(-46.4003f, 8.21245f, -33.3371f), Vector3(-43.8729f, 14.309f, -33.3371f), Vector3(-50.5964f, 17.0963f, -26.8408f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(65); 
  data[0].mBoundingSphere = Sphere(Vector3(25.8589f, -3.51974f, -10.2858f), 2.76909f); 
  data[1].mBoundingSphere = Sphere(Vector3(3.63407f, 17.783f, 25.568f), 2.7539f); 
  data[2].mBoundingSphere = Sphere(Vector3(-9.9054f, -13.5512f, -44.4831f), 3.50527f); 
  data[3].mBoundingSphere = Sphere(Vector3(-2.62289f, 15.8216f, -11.9546f), 0.785882f); 
  data[4].mBoundingSphere = Sphere(Vector3(4.0289f, -4.58397f, 13.33f), 2.33617f); 
  data[5].mBoundingSphere = Sphere(Vector3(20.0797f, 1.41735f, -13.9511f), 2.91776f); 
  data[6].mBoundingSphere = Sphere(Vector3(21.8876f, -39.3494f, -6.09451f), 1.61101f); 
  data[7].mBoundingSphere = Sphere(Vector3(-34.2531f, 15.4202f, 28.4767f), 2.58389f); 
  data[8].mBoundingSphere = Sphere(Vector3(16.0809f, -27.7587f, -11.4833f), 3.25048f); 
  data[9].mBoundingSphere = Sphere(Vector3(-18.177f, 3.80454f, -38.3488f), 2.23876f); 
  data[10].mBoundingSphere = Sphere(Vector3(25.0617f, -8.96481f, -29.7255f), 0.686843f); 
  data[11].mBoundingSphere = Sphere(Vector3(3.08987f, 22.1057f, -17.3217f), 2.6842f); 
  data[12].mBoundingSphere = Sphere(Vector3(10.246f, -12.9573f, 4.58921f), 3.57938f); 
  data[13].mBoundingSphere = Sphere(Vector3(-11.5306f, 7.38276f, -13.571f), 2.37131f); 
  data[14].mBoundingSphere = Sphere(Vector3(11.283f, -19.6673f, -7.31534f), 0.303214f); 
  data[15].mBoundingSphere = Sphere(Vector3(9.76839f, -26.2594f, 10.1852f), 2.87833f); 
  data[16].mBoundingSphere = Sphere(Vector3(5.80698f, -18.8012f, -3.29721f), 2.6152f); 
  data[17].mBoundingSphere = Sphere(Vector3(31.3601f, -7.98669f, -32.5329f), 1.56277f); 
  data[18].mBoundingSphere = Sphere(Vector3(-25.2657f, -6.21064f, -14.8625f), 1.49284f); 
  data[19].mBoundingSphere = Sphere(Vector3(7.41427f, -3.75804f, 47.5239f), 1.78879f); 
  data[20].mBoundingSphere = Sphere(Vector3(39.1345f, 9.75915f, 21.1213f), 1.34985f); 
  data[21].mBoundingSphere = Sphere(Vector3(20.9437f, 36.0889f, 18.6603f), 2.72943f); 
  data[22].mBoundingSphere = Sphere(Vector3(20.8252f, -8.75531f, -40.7327f), 3.4256f); 
  data[23].mBoundingSphere = Sphere(Vector3(-38.9262f, -20.2728f, 0.413898f), 2.94397f); 
  data[24].mBoundingSphere = Sphere(Vector3(28.9646f, 22.3072f, -21.5044f), 2.94989f); 
  data[25].mBoundingSphere = Sphere(Vector3(-23.438f, -22.696f, 9.21341f), 2.53042f); 
  data[26].mBoundingSphere = Sphere(Vector3(-22.3558f, 27.8356f, -29.8526f), 3.88322f); 
  data[27].mBoundingSphere = Sphere(Vector3(-14.2841f, -12.4132f, -9.99169f), 3.85283f); 
  data[28].mBoundingSphere = Sphere(Vector3(-10.4605f, 45.132f, -12.048f), 1.83124f); 
  data[29].mBoundingSphere = Sphere(Vector3(11.4499f, 8.06917f, -39.9222f), 3.24353f); 
  data[30].mBoundingSphere = Sphere(Vector3(9.76255f, 39.6743f, -16.9327f), 3.53484f); 
  data[31].mBoundingSphere = Sphere(Vector3(-19.4134f, 18.7196f, -23.0644f), 0.280367f); 
  data[32].mBoundingSphere = Sphere(Vector3(-3.26016f, 27.4831f, -5.81626f), 3.13475f); 
  data[33].mBoundingSphere = Sphere(Vector3(18.5714f, 29.2116f, -9.39987f), 1.8397f); 
  data[34].mBoundingSphere = Sphere(Vector3(-19.2407f, -16.6365f, 4.61936f), 2.61149f); 
  data[35].mBoundingSphere = Sphere(Vector3(-5.88631f, 12.4962f, -19.7852f), 1.92274f); 
  data[36].mBoundingSphere = Sphere(Vector3(0.663139f, -25.9479f, 4.85715f), 0.400513f); 
  data[37].mBoundingSphere = Sphere(Vector3(11.4997f, -35.006f, 13.4409f), 1.85861f); 
  data[38].mBoundingSphere = Sphere(Vector3(38.5947f, 2.07789f, 18.6163f), 1.38151f); 
  data[39].mBoundingSphere = Sphere(Vector3(-13.2592f, -13.8155f, 44.5601f), 2.48137f); 
  data[40].mBoundingSphere = Sphere(Vector3(12.7392f, 7.72744f, -4.54741f), 2.64373f); 
  data[41].mBoundingSphere = Sphere(Vector3(38.6497f, 15.3323f, 8.39523f), 2.48276f); 
  data[42].mBoundingSphere = Sphere(Vector3(3.40643f, -11.1579f, -19.9536f), 0.625147f); 
  data[43].mBoundingSphere = Sphere(Vector3(13.3975f, -18.0113f, 43.2076f), 2.9391f); 
  data[44].mBoundingSphere = Sphere(Vector3(23.5565f, -22.3728f, -12.289f), 0.62851f); 
  data[45].mBoundingSphere = Sphere(Vector3(-26.433f, -37.7954f, 8.51804f), 2.81351f); 
  data[46].mBoundingSphere = Sphere(Vector3(29.63f, -7.67326f, 26.4895f), 2.11003f); 
  data[47].mBoundingSphere = Sphere(Vector3(-39.7999f, -6.58425f, 3.36403f), 0.634077f); 
  data[48].mBoundingSphere = Sphere(Vector3(-25.0852f, -28.7574f, 0.60917f), 1.54444f); 
  data[49].mBoundingSphere = Sphere(Vector3(-5.99527f, -3.90869f, 29.9029f), 2.34522f); 
  data[50].mBoundingSphere = Sphere(Vector3(-11.0006f, -23.9584f, -33.134f), 1.46547f); 
  data[51].mBoundingSphere = Sphere(Vector3(13.3067f, -42.6189f, 21.2006f), 1.13576f); 
  data[52].mBoundingSphere = Sphere(Vector3(-22.313f, 21.1145f, -5.51795f), 2.88007f); 
  data[53].mBoundingSphere = Sphere(Vector3(11.8069f, -27.6413f, -29.4473f), 1.44088f); 
  data[54].mBoundingSphere = Sphere(Vector3(18.9605f, -2.6104f, -28.5025f), 3.8584f); 
  data[55].mBoundingSphere = Sphere(Vector3(9.95972f, 3.31473f, -47.69f), 3.51942f); 
  data[56].mBoundingSphere = Sphere(Vector3(-5.07508f, 32.4626f, -31.1373f), 0.498624f); 
  data[57].mBoundingSphere = Sphere(Vector3(16.5435f, -45.5109f, 9.62671f), 1.80654f); 
  data[58].mBoundingSphere = Sphere(Vector3(-11.3523f, 23.957f, -33.2189f), 1.66726f); 
  data[59].mBoundingSphere = Sphere(Vector3(-27.5237f, -5.1629f, 11.9021f), 0.840852f); 
  data[60].mBoundingSphere = Sphere(Vector3(-1.14747f, 43.4319f, 9.58203f), 0.51312f); 
  data[61].mBoundingSphere = Sphere(Vector3(3.30503f, 21.1273f, -25.164f), 3.16699f); 
  data[62].mBoundingSphere = Sphere(Vector3(-1.99687f, 36.1534f, 33.9235f), 1.55813f); 
  data[63].mBoundingSphere = Sphere(Vector3(23.7518f, -5.4224f, 26.1982f), 3.41226f); 
  data[64].mBoundingSphere = Sphere(Vector3(-20.1399f, -25.3243f, 0.11356f), 0.761296f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(15.0269f, 23.301f, 36.9641f), Vector3(-9.82889f, 40.0549f, -18.385f), Vector3(-9.82889f, -18.4855f, -36.1049f), Vector3(15.0269f, -35.2394f, 19.2442f), Vector3(20.2781f, 6.03296f, -2.37893f), Vector3(17.4771f, 7.92098f, -8.61628f), Vector3(17.4771f, 2.83663f, -10.1553f), Vector3(20.2781f, 0.948611f, -3.91794f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mBoundingSphere = Sphere(Vector3(29.4754f, 13.3817f, 5.49623f), 1.27922f); 
  data[1].mBoundingSphere = Sphere(Vector3(10.474f, 20.1089f, 36.9728f), 0.609955f); 
  data[2].mBoundingSphere = Sphere(Vector3(15.8423f, -20.1374f, -0.604614f), 3.15342f); 
  data[3].mBoundingSphere = Sphere(Vector3(-38.7646f, -1.81503f, 9.3937f), 1.36968f); 
  data[4].mBoundingSphere = Sphere(Vector3(24.0893f, 35.2514f, -7.65282f), 0.816498f); 
  data[5].mBoundingSphere = Sphere(Vector3(-4.15899f, 16.3403f, -6.69293f), 3.55943f); 
  data[6].mBoundingSphere = Sphere(Vector3(-16.1462f, -14.4664f, -33.3428f), 3.93935f); 
  data[7].mBoundingSphere = Sphere(Vector3(3.83936f, -0.17202f, 24.4955f), 1.09297f); 
  data[8].mBoundingSphere = Sphere(Vector3(-8.80673f, 11.3796f, 9.54814f), 0.316434f); 
  data[9].mBoundingSphere = Sphere(Vector3(19.6333f, -15.0249f, -43.0881f), 3.86814f); 
  data[10].mBoundingSphere = Sphere(Vector3(-4.19468f, 15.5301f, 19.4598f), 1.30415f); 
  data[11].mBoundingSphere = Sphere(Vector3(0.0746189f, 2.14611f, 36.6838f), 1.73997f); 
  data[12].mBoundingSphere = Sphere(Vector3(28.0613f, 30.0525f, 6.30038f), 0.331394f); 
  data[13].mBoundingSphere = Sphere(Vector3(15.9203f, 3.57353f, -33.7536f), 1.17044f); 
  data[14].mBoundingSphere = Sphere(Vector3(27.644f, -8.69083f, 5.77951f), 0.286978f); 
  data[15].mBoundingSphere = Sphere(Vector3(-27.5665f, 13.6583f, -33.7591f), 1.55303f); 
  data[16].mBoundingSphere = Sphere(Vector3(-30.8352f, 12.7451f, -23.7863f), 3.38095f); 
  data[17].mBoundingSphere = Sphere(Vector3(5.81259f, -36.7871f, 27.2941f), 2.41109f); 
  data[18].mBoundingSphere = Sphere(Vector3(-4.2721f, -22.3779f, -14.7299f), 1.48136f); 
  data[19].mBoundingSphere = Sphere(Vector3(-6.76204f, 24.8459f, -7.75485f), 3.47291f); 
  data[20].mBoundingSphere = Sphere(Vector3(19.7046f, -26.5967f, -24.1275f), 1.12614f); 
  data[21].mBoundingSphere = Sphere(Vector3(-6.56963f, 11.3179f, -15.6835f), 1.42024f); 
  data[22].mBoundingSphere = Sphere(Vector3(33.5572f, -23.1043f, 20.8066f), 3.40983f); 
  data[23].mBoundingSphere = Sphere(Vector3(-8.08495f, 21.8686f, 12.678f), 2.3131f); 
  data[24].mBoundingSphere = Sphere(Vector3(-10.909f, -28.2998f, 28.4299f), 3.345f); 
  data[25].mBoundingSphere = Sphere(Vector3(12.1261f, 41.806f, 22.8409f), 0.288485f); 
  data[26].mBoundingSphere = Sphere(Vector3(23.699f, 21.4267f, 7.11746f), 1.09935f); 
  data[27].mBoundingSphere = Sphere(Vector3(1.44614f, -10.2742f, -18.5048f), 2.33525f); 
  data[28].mBoundingSphere = Sphere(Vector3(13.9737f, 18.2614f, 15.1396f), 2.30718f); 
  data[29].mBoundingSphere = Sphere(Vector3(-6.13156f, 28.9966f, 39.059f), 0.342412f); 
  data[30].mBoundingSphere = Sphere(Vector3(-33.5723f, 23.1859f, 6.29577f), 3.65638f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-28.3433f, -19.4934f, 48.7853f), Vector3(17.6628f, -31.0036f, 3.13146f), Vector3(33.7777f, 33.4074f, 3.13146f), Vector3(-12.2283f, 44.9176f, 48.7853f), Vector3(-36.562f, 13.1687f, -10.5423f), Vector3(-32.0566f, 12.0415f, -15.0132f), Vector3(-30.354f, 18.8467f, -15.0132f), Vector3(-34.8594f, 19.9739f, -10.5423f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(47); 
  data[0].mBoundingSphere = Sphere(Vector3(-12.3023f, -16.5423f, -15.4208f), 3.49089f); 
  data[1].mBoundingSphere = Sphere(Vector3(-40.1968f, -7.88377f, 8.96432f), 2.90536f); 
  data[2].mBoundingSphere = Sphere(Vector3(-6.37864f, 25.3661f, 9.80817f), 1.14516f); 
  data[3].mBoundingSphere = Sphere(Vector3(3.25252f, -7.95863f, -8.08619f), 3.57346f); 
  data[4].mBoundingSphere = Sphere(Vector3(5.06868f, -24.6106f, 7.02706f), 3.1903f); 
  data[5].mBoundingSphere = Sphere(Vector3(38.8201f, -7.42455f, 29.4803f), 2.50549f); 
  data[6].mBoundingSphere = Sphere(Vector3(7.6935f, 20.1828f, 0.240617f), 1.12939f); 
  data[7].mBoundingSphere = Sphere(Vector3(19.4587f, -24.0249f, 8.5828f), 3.35149f); 
  data[8].mBoundingSphere = Sphere(Vector3(-36.9788f, -0.155995f, -17.6112f), 3.07432f); 
  data[9].mBoundingSphere = Sphere(Vector3(10.4865f, 29.3557f, -9.74735f), 0.371636f); 
  data[10].mBoundingSphere = Sphere(Vector3(-1.59075f, 7.1884f, 25.3522f), 3.24179f); 
  data[11].mBoundingSphere = Sphere(Vector3(-4.54751f, 13.1546f, 47.0875f), 0.419764f); 
  data[12].mBoundingSphere = Sphere(Vector3(-24.9832f, -2.335f, 9.36456f), 3.22451f); 
  data[13].mBoundingSphere = Sphere(Vector3(10.8985f, -2.07573f, -5.79427f), 1.70147f); 
  data[14].mBoundingSphere = Sphere(Vector3(-27.6865f, 3.01673f, -18.7107f), 1.43717f); 
  data[15].mBoundingSphere = Sphere(Vector3(-30.7868f, 12.0324f, 13.5839f), 1.50826f); 
  data[16].mBoundingSphere = Sphere(Vector3(25.7773f, -35.074f, 4.8912f), 3.93784f); 
  data[17].mBoundingSphere = Sphere(Vector3(-19.2529f, -3.356f, -33.5005f), 2.52602f); 
  data[18].mBoundingSphere = Sphere(Vector3(20.3508f, -37.6329f, 12.6236f), 3.42351f); 
  data[19].mBoundingSphere = Sphere(Vector3(21.6077f, -29.8421f, 32.9167f), 2.51825f); 
  data[20].mBoundingSphere = Sphere(Vector3(-12.7331f, 2.07891f, -33.579f), 2.4017f); 
  data[21].mBoundingSphere = Sphere(Vector3(7.12935f, 20.5784f, 29.0791f), 1.58538f); 
  data[22].mBoundingSphere = Sphere(Vector3(13.8456f, 12.6392f, -31.5916f), 0.736131f); 
  data[23].mBoundingSphere = Sphere(Vector3(37.1521f, 3.90761f, -32.1994f), 0.397729f); 
  data[24].mBoundingSphere = Sphere(Vector3(35.2097f, -20.8713f, 16.9123f), 0.768371f); 
  data[25].mBoundingSphere = Sphere(Vector3(38.8387f, 6.98899f, 14.7629f), 0.804089f); 
  data[26].mBoundingSphere = Sphere(Vector3(-31.1679f, -4.10548f, 36.0133f), 3.41354f); 
  data[27].mBoundingSphere = Sphere(Vector3(43.7519f, -10.9017f, -19.8367f), 1.2159f); 
  data[28].mBoundingSphere = Sphere(Vector3(33.6466f, -27.0945f, 20.7029f), 2.8309f); 
  data[29].mBoundingSphere = Sphere(Vector3(-29.3756f, 16.7758f, -26.9752f), 1.9653f); 
  data[30].mBoundingSphere = Sphere(Vector3(6.46863f, 26.3315f, -36.8368f), 2.47116f); 
  data[31].mBoundingSphere = Sphere(Vector3(23.3893f, -27.9126f, 20.2313f), 1.63884f); 
  data[32].mBoundingSphere = Sphere(Vector3(12.0515f, -4.77817f, -39.8693f), 3.09682f); 
  data[33].mBoundingSphere = Sphere(Vector3(-3.87321f, -31.4146f, -18.8213f), 3.38617f); 
  data[34].mBoundingSphere = Sphere(Vector3(37.4438f, -2.21041f, 4.4531f), 0.777996f); 
  data[35].mBoundingSphere = Sphere(Vector3(-6.3056f, -12.8455f, -17.5219f), 0.80351f); 
  data[36].mBoundingSphere = Sphere(Vector3(38.7995f, 7.7837f, -9.3008f), 2.46444f); 
  data[37].mBoundingSphere = Sphere(Vector3(32.659f, -15.4662f, 24.8663f), 0.392743f); 
  data[38].mBoundingSphere = Sphere(Vector3(4.74006f, -3.00236f, -41.603f), 2.13044f); 
  data[39].mBoundingSphere = Sphere(Vector3(2.42233f, -12.8965f, -9.22655f), 0.794116f); 
  data[40].mBoundingSphere = Sphere(Vector3(6.68542f, 22.8358f, 2.25316f), 1.09715f); 
  data[41].mBoundingSphere = Sphere(Vector3(2.11155f, 6.17513f, -47.4179f), 0.995441f); 
  data[42].mBoundingSphere = Sphere(Vector3(32.8254f, -23.982f, -27.7842f), 0.327567f); 
  data[43].mBoundingSphere = Sphere(Vector3(-31.75f, -14.9502f, -31.1426f), 3.31578f); 
  data[44].mBoundingSphere = Sphere(Vector3(18.5696f, 44.1041f, -13.9083f), 2.69858f); 
  data[45].mBoundingSphere = Sphere(Vector3(-39.2025f, -16.3357f, -0.636465f), 3.56337f); 
  data[46].mBoundingSphere = Sphere(Vector3(-25.9288f, 21.3801f, -33.0997f), 2.44635f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-27.5608f, -91.2463f, 64.32f), Vector3(113.762f, -41.9738f, 39.0131f), Vector3(64.9196f, 98.1169f, 39.0131f), Vector3(-76.4035f, 48.8444f, 64.32f), Vector3(4.78402f, -4.54378f, -13.4279f), Vector3(13.4883f, -1.50901f, -14.9866f), Vector3(11.5396f, 4.08039f, -14.9866f), Vector3(2.83527f, 1.04562f, -13.4279f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mBoundingSphere = Sphere(Vector3(-23.9707f, -6.03385f, 18.5931f), 1.87334f); 
  data[1].mBoundingSphere = Sphere(Vector3(27.0667f, 2.37536f, 34.0735f), 1.11918f); 
  data[2].mBoundingSphere = Sphere(Vector3(20.4189f, 18.1084f, 9.37872f), 0.89443f); 
  data[3].mBoundingSphere = Sphere(Vector3(2.98251f, -16.3626f, 20.7825f), 0.875295f); 
  data[4].mBoundingSphere = Sphere(Vector3(21.4854f, 6.27778f, -12.6328f), 1.56868f); 
  data[5].mBoundingSphere = Sphere(Vector3(-18.5475f, 14.7867f, -30.8783f), 1.03325f); 
  data[6].mBoundingSphere = Sphere(Vector3(14.2432f, -10.0173f, 33.3335f), 1.04995f); 
  data[7].mBoundingSphere = Sphere(Vector3(-12.3916f, 43.2634f, -19.9925f), 3.38373f); 
  data[8].mBoundingSphere = Sphere(Vector3(-13.3764f, 15.1459f, -26.3048f), 1.47996f); 
  data[9].mBoundingSphere = Sphere(Vector3(13.7725f, -17.6523f, -9.69378f), 1.44251f); 
  data[10].mBoundingSphere = Sphere(Vector3(-7.23091f, 35.7154f, 22.2987f), 1.63154f); 
  data[11].mBoundingSphere = Sphere(Vector3(1.86296f, -26.1251f, -17.9375f), 2.73117f); 
  data[12].mBoundingSphere = Sphere(Vector3(-22.6335f, 26.3521f, -9.95035f), 0.426026f); 
  data[13].mBoundingSphere = Sphere(Vector3(38.0248f, 3.42523f, 21.9342f), 2.1323f); 
  data[14].mBoundingSphere = Sphere(Vector3(-39.9347f, 0.650941f, 14.4343f), 0.927018f); 
  data[15].mBoundingSphere = Sphere(Vector3(-36.0917f, -14.9745f, 13.1497f), 1.31958f); 
  data[16].mBoundingSphere = Sphere(Vector3(-7.64981f, -20.7283f, 15.0266f), 3.40786f); 
  data[17].mBoundingSphere = Sphere(Vector3(-32.6484f, -11.5834f, -10.5971f), 2.06411f); 
  data[18].mBoundingSphere = Sphere(Vector3(-25.6928f, -33.8648f, -21.6537f), 2.69441f); 
  data[19].mBoundingSphere = Sphere(Vector3(4.051f, 18.7225f, 3.33551f), 3.8141f); 
  data[20].mBoundingSphere = Sphere(Vector3(37.7429f, -18.1582f, -2.37235f), 1.18378f); 
  data[21].mBoundingSphere = Sphere(Vector3(-24.7048f, 30.8573f, -18.2789f), 1.75064f); 
  data[22].mBoundingSphere = Sphere(Vector3(10.6506f, -29.1776f, 29.952f), 3.33653f); 
  data[23].mBoundingSphere = Sphere(Vector3(-45.2212f, 0.182098f, 3.56477f), 0.518455f); 
  data[24].mBoundingSphere = Sphere(Vector3(-2.9091f, -6.14828f, 0.437025f), 3.04498f); 
  data[25].mBoundingSphere = Sphere(Vector3(-23.5291f, -24.488f, 8.80869f), 1.83089f); 
  data[26].mBoundingSphere = Sphere(Vector3(4.45845f, 6.46296f, 20.7208f), 1.87612f); 
  data[27].mBoundingSphere = Sphere(Vector3(-32.3965f, -16.448f, -19.0329f), 3.95048f); 
  data[28].mBoundingSphere = Sphere(Vector3(-4.01054f, -25.3035f, 9.35865f), 1.18957f); 
  data[29].mBoundingSphere = Sphere(Vector3(20.6198f, 2.36485f, 40.408f), 1.64476f); 
  data[30].mBoundingSphere = Sphere(Vector3(20.0099f, 14.385f, -26.4909f), 2.33072f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-53.4979f, 46.0129f, -44.6323f), Vector3(17.6525f, -52.0933f, -54.5665f), Vector3(17.6525f, -64.3078f, 66.058f), Vector3(-53.4979f, 33.7984f, 75.9922f), Vector3(33.9103f, 34.1569f, 10.6491f), Vector3(39.2984f, 26.7276f, 9.8968f), Vector3(39.2984f, 25.8358f, 18.704f), Vector3(33.9103f, 33.2651f, 19.4563f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(60); 
  data[0].mBoundingSphere = Sphere(Vector3(20.4394f, -15.9612f, 24.847f), 3.63203f); 
  data[1].mBoundingSphere = Sphere(Vector3(-11.3044f, 24.7954f, 7.59115f), 3.7175f); 
  data[2].mBoundingSphere = Sphere(Vector3(-10.0107f, 17.6411f, 24.604f), 3.58564f); 
  data[3].mBoundingSphere = Sphere(Vector3(-6.93196f, -28.608f, -22.0152f), 1.14527f); 
  data[4].mBoundingSphere = Sphere(Vector3(-21.1725f, 3.86238f, -30.5767f), 3.26196f); 
  data[5].mBoundingSphere = Sphere(Vector3(-10.555f, 46.1833f, 4.73823f), 0.483548f); 
  data[6].mBoundingSphere = Sphere(Vector3(-19.6163f, 5.87687f, -2.60883f), 0.286978f); 
  data[7].mBoundingSphere = Sphere(Vector3(17.8512f, 4.94211f, -0.988961f), 1.16499f); 
  data[8].mBoundingSphere = Sphere(Vector3(-10.167f, 8.94193f, -16.0309f), 0.772314f); 
  data[9].mBoundingSphere = Sphere(Vector3(-20.0517f, -9.61618f, -27.6822f), 0.249287f); 
  data[10].mBoundingSphere = Sphere(Vector3(14.1974f, 11.0343f, 42.4792f), 2.94247f); 
  data[11].mBoundingSphere = Sphere(Vector3(-8.47763f, 8.7054f, -10.7249f), 3.83892f); 
  data[12].mBoundingSphere = Sphere(Vector3(15.6817f, 8.74739f, -1.60823f), 2.13682f); 
  data[13].mBoundingSphere = Sphere(Vector3(-11.2738f, 22.9221f, 16.9583f), 2.86743f); 
  data[14].mBoundingSphere = Sphere(Vector3(-29.7317f, 14.5329f, -5.36016f), 1.98791f); 
  data[15].mBoundingSphere = Sphere(Vector3(0.0243014f, 12.3633f, 25.2604f), 0.666085f); 
  data[16].mBoundingSphere = Sphere(Vector3(40.0469f, 13.6274f, -20.4315f), 1.37269f); 
  data[17].mBoundingSphere = Sphere(Vector3(-22.4576f, -28.5181f, 15.0378f), 1.10237f); 
  data[18].mBoundingSphere = Sphere(Vector3(3.80347f, -1.98596f, 23.7499f), 2.74056f); 
  data[19].mBoundingSphere = Sphere(Vector3(-37.1171f, 2.63785f, 18.0784f), 2.89573f); 
  data[20].mBoundingSphere = Sphere(Vector3(-33.3069f, 7.17152f, 3.7014f), 3.81549f); 
  data[21].mBoundingSphere = Sphere(Vector3(4.57001f, 22.4067f, 17.5022f), 3.9063f); 
  data[22].mBoundingSphere = Sphere(Vector3(33.4696f, 18.737f, -30.2413f), 3.07328f); 
  data[23].mBoundingSphere = Sphere(Vector3(6.94261f, -22.4018f, 13.6705f), 0.200928f); 
  data[24].mBoundingSphere = Sphere(Vector3(6.22789f, -24.7165f, -28.9281f), 3.79392f); 
  data[25].mBoundingSphere = Sphere(Vector3(3.04999f, -1.34887f, 23.4652f), 2.69278f); 
  data[26].mBoundingSphere = Sphere(Vector3(5.9074f, 0.0390818f, 33.7048f), 1.48936f); 
  data[27].mBoundingSphere = Sphere(Vector3(-23.7013f, 16.6319f, 23.8081f), 2.95534f); 
  data[28].mBoundingSphere = Sphere(Vector3(-26.3619f, 16.3912f, 35.2962f), 0.305301f); 
  data[29].mBoundingSphere = Sphere(Vector3(-26.9722f, -1.65193f, -20.0314f), 2.21313f); 
  data[30].mBoundingSphere = Sphere(Vector3(12.6447f, 1.53624f, 31.2632f), 1.39879f); 
  data[31].mBoundingSphere = Sphere(Vector3(-21.4427f, -39.6067f, -19.2482f), 0.906259f); 
  data[32].mBoundingSphere = Sphere(Vector3(-3.39055f, 24.9008f, 22.883f), 1.29035f); 
  data[33].mBoundingSphere = Sphere(Vector3(33.7027f, 31.5695f, 4.15874f), 2.03036f); 
  data[34].mBoundingSphere = Sphere(Vector3(-6.5969f, 11.4102f, -3.25795f), 3.99177f); 
  data[35].mBoundingSphere = Sphere(Vector3(-1.28084f, -18.2329f, -6.10644f), 1.65125f); 
  data[36].mBoundingSphere = Sphere(Vector3(-40.8113f, 4.54979f, -16.4032f), 1.30786f); 
  data[37].mBoundingSphere = Sphere(Vector3(26.9449f, 3.0013f, -2.80574f), 0.794812f); 
  data[38].mBoundingSphere = Sphere(Vector3(-22.4397f, -23.1845f, -9.4734f), 1.11443f); 
  data[39].mBoundingSphere = Sphere(Vector3(13.1474f, 14.7692f, 34.1518f), 0.712009f); 
  data[40].mBoundingSphere = Sphere(Vector3(7.82635f, -2.26073f, -31.6865f), 1.01017f); 
  data[41].mBoundingSphere = Sphere(Vector3(12.0659f, 19.621f, 14.8408f), 0.518802f); 
  data[42].mBoundingSphere = Sphere(Vector3(-3.29505f, -9.54939f, 33.855f), 2.34406f); 
  data[43].mBoundingSphere = Sphere(Vector3(-20.9421f, 14.6179f, -42.2674f), 1.0104f); 
  data[44].mBoundingSphere = Sphere(Vector3(30.6123f, 4.99498f, -26.923f), 3.81804f); 
  data[45].mBoundingSphere = Sphere(Vector3(10.4807f, -15.9533f, 30.9966f), 3.25188f); 
  data[46].mBoundingSphere = Sphere(Vector3(30.3448f, 15.1329f, -6.46091f), 1.26368f); 
  data[47].mBoundingSphere = Sphere(Vector3(-3.21001f, -30.9041f, 35.0015f), 1.94245f); 
  data[48].mBoundingSphere = Sphere(Vector3(38.1676f, 20.35f, 15.2652f), 1.19526f); 
  data[49].mBoundingSphere = Sphere(Vector3(-2.90592f, 8.47963f, -46.4335f), 3.95558f); 
  data[50].mBoundingSphere = Sphere(Vector3(-8.30012f, -11.4285f, -47.8226f), 2.53692f); 
  data[51].mBoundingSphere = Sphere(Vector3(-34.8659f, -19.6814f, 17.1391f), 3.65383f); 
  data[52].mBoundingSphere = Sphere(Vector3(14.7505f, 26.7959f, -29.016f), 3.52464f); 
  data[53].mBoundingSphere = Sphere(Vector3(-15.2443f, 16.9833f, 36.2758f), 2.37862f); 
  data[54].mBoundingSphere = Sphere(Vector3(-11.707f, 32.062f, 2.64647f), 3.38373f); 
  data[55].mBoundingSphere = Sphere(Vector3(-12.5548f, 41.8844f, -22.894f), 3.71506f); 
  data[56].mBoundingSphere = Sphere(Vector3(-19.6143f, -30.2401f, -27.1857f), 2.58157f); 
  data[57].mBoundingSphere = Sphere(Vector3(36.8458f, 15.499f, 0.167129f), 3.53171f); 
  data[58].mBoundingSphere = Sphere(Vector3(-17.7997f, -3.54069f, -18.7115f), 1.65438f); 
  data[59].mBoundingSphere = Sphere(Vector3(-1.47597f, -34.8477f, 27.5121f), 1.39589f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-68.3335f, 19.3717f, 23.1596f), Vector3(-0.0397873f, -45.59f, 30.6361f), Vector3(65.2209f, 23.0181f, 30.6361f), Vector3(-3.07288f, 87.9798f, 23.1596f), Vector3(-5.06734f, 18.6688f, -15.859f), Vector3(0.972922f, 12.9232f, -15.1977f), Vector3(6.83143f, 19.0822f, -15.1977f), Vector3(0.791169f, 24.8278f, -15.859f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(67); 
  data[0].mBoundingSphere = Sphere(Vector3(-6.42902f, -10.4568f, 48.0395f), 1.83889f); 
  data[1].mBoundingSphere = Sphere(Vector3(33.2641f, -15.6514f, 11.596f), 3.98782f); 
  data[2].mBoundingSphere = Sphere(Vector3(15.0647f, -11.1575f, 19.4271f), 3.0451f); 
  data[3].mBoundingSphere = Sphere(Vector3(31.1765f, -27.7771f, 3.16555f), 3.85249f); 
  data[4].mBoundingSphere = Sphere(Vector3(-36.3641f, 22.7653f, 15.3258f), 0.719199f); 
  data[5].mBoundingSphere = Sphere(Vector3(23.6641f, -28.1307f, 8.9105f), 0.881674f); 
  data[6].mBoundingSphere = Sphere(Vector3(-6.96377f, -12.1736f, 28.2195f), 3.79438f); 
  data[7].mBoundingSphere = Sphere(Vector3(-19.324f, 38.1381f, 10.3293f), 1.44807f); 
  data[8].mBoundingSphere = Sphere(Vector3(8.47921f, 0.52666f, 36.1285f), 2.20733f); 
  data[9].mBoundingSphere = Sphere(Vector3(22.4414f, 12.813f, 1.23089f), 2.76225f); 
  data[10].mBoundingSphere = Sphere(Vector3(-0.718547f, 34.452f, 8.83402f), 1.88621f); 
  data[11].mBoundingSphere = Sphere(Vector3(-7.59336f, -22.9293f, -2.5731f), 3.3973f); 
  data[12].mBoundingSphere = Sphere(Vector3(20.826f, -13.7291f, -21.2327f), 3.21152f); 
  data[13].mBoundingSphere = Sphere(Vector3(-36.0031f, -6.60351f, 8.16399f), 2.88784f); 
  data[14].mBoundingSphere = Sphere(Vector3(44.1953f, 6.66426f, -1.31412f), 2.7532f); 
  data[15].mBoundingSphere = Sphere(Vector3(-6.7496f, 6.556f, -0.839832f), 1.80457f); 
  data[16].mBoundingSphere = Sphere(Vector3(-11.4386f, -20.9646f, -32.1061f), 3.08198f); 
  data[17].mBoundingSphere = Sphere(Vector3(3.98262f, -17.4336f, 23.2887f), 2.30301f); 
  data[18].mBoundingSphere = Sphere(Vector3(35.8753f, 6.29958f, 4.92779f), 0.256825f); 
  data[19].mBoundingSphere = Sphere(Vector3(37.2849f, -14.3049f, 14.2016f), 1.54792f); 
  data[20].mBoundingSphere = Sphere(Vector3(13.9211f, 11.6936f, 20.0377f), 2.18588f); 
  data[21].mBoundingSphere = Sphere(Vector3(7.27131f, 31.6062f, 26.8311f), 0.812555f); 
  data[22].mBoundingSphere = Sphere(Vector3(9.6938f, 9.13975f, 36.4241f), 1.86336f); 
  data[23].mBoundingSphere = Sphere(Vector3(-13.6653f, 32.8162f, -13.0064f), 3.6529f); 
  data[24].mBoundingSphere = Sphere(Vector3(-28.7231f, 18.2203f, -1.2122f), 0.481808f); 
  data[25].mBoundingSphere = Sphere(Vector3(11.4359f, 5.81854f, 11.9192f), 3.15678f); 
  data[26].mBoundingSphere = Sphere(Vector3(18.7978f, 11.7556f, -0.771143f), 0.618073f); 
  data[27].mBoundingSphere = Sphere(Vector3(25.3357f, -25.6941f, 2.066f), 3.86826f); 
  data[28].mBoundingSphere = Sphere(Vector3(1.58802f, 30.9612f, -4.4923f), 3.69569f); 
  data[29].mBoundingSphere = Sphere(Vector3(-11.7635f, -17.3389f, -31.3068f), 2.38662f); 
  data[30].mBoundingSphere = Sphere(Vector3(6.03563f, -34.8128f, -25.1126f), 0.320609f); 
  data[31].mBoundingSphere = Sphere(Vector3(-29.2244f, -20.1831f, 31.255f), 2.64083f); 
  data[32].mBoundingSphere = Sphere(Vector3(30.3237f, -21.8702f, -13.5723f), 2.89469f); 
  data[33].mBoundingSphere = Sphere(Vector3(-9.28309f, 7.50918f, 27.1636f), 1.23851f); 
  data[34].mBoundingSphere = Sphere(Vector3(9.34654f, 37.2744f, -29.3739f), 2.70009f); 
  data[35].mBoundingSphere = Sphere(Vector3(15.42f, 25.6448f, 37.5033f), 3.69059f); 
  data[36].mBoundingSphere = Sphere(Vector3(-17.0394f, 8.76231f, 2.24489f), 3.46317f); 
  data[37].mBoundingSphere = Sphere(Vector3(34.0729f, -0.617494f, 22.3946f), 3.1097f); 
  data[38].mBoundingSphere = Sphere(Vector3(-1.59237f, -8.84659f, -18.8453f), 2.18959f); 
  data[39].mBoundingSphere = Sphere(Vector3(15.2213f, 10.8894f, -27.1961f), 1.18517f); 
  data[40].mBoundingSphere = Sphere(Vector3(-4.72104f, -25.1747f, 27.2535f), 0.877035f); 
  data[41].mBoundingSphere = Sphere(Vector3(-31.0812f, -18.1473f, -8.66902f), 3.03234f); 
  data[42].mBoundingSphere = Sphere(Vector3(44.1297f, 21.9334f, 6.45394f), 3.50168f); 
  data[43].mBoundingSphere = Sphere(Vector3(24.1663f, -21.0213f, -32.0996f), 1.11582f); 
  data[44].mBoundingSphere = Sphere(Vector3(-0.761887f, -1.93652f, 45.6463f), 2.0793f); 
  data[45].mBoundingSphere = Sphere(Vector3(9.33271f, -35.456f, 25.9272f), 1.44981f); 
  data[46].mBoundingSphere = Sphere(Vector3(-23.5689f, -14.2195f, -8.21719f), 1.20303f); 
  data[47].mBoundingSphere = Sphere(Vector3(34.3051f, 24.3835f, 25.5034f), 0.871468f); 
  data[48].mBoundingSphere = Sphere(Vector3(-11.2736f, 14.6366f, -6.14722f), 1.2544f); 
  data[49].mBoundingSphere = Sphere(Vector3(27.1975f, -18.1773f, 22.2442f), 2.79078f); 
  data[50].mBoundingSphere = Sphere(Vector3(-1.87812f, 23.5086f, -1.12045f), 1.09007f); 
  data[51].mBoundingSphere = Sphere(Vector3(-36.9877f, -4.97313f, -25.5676f), 3.01425f); 
  data[52].mBoundingSphere = Sphere(Vector3(28.4091f, -4.05012f, 1.31416f), 1.20384f); 
  data[53].mBoundingSphere = Sphere(Vector3(48.1485f, -1.84281f, 5.59734f), 2.64442f); 
  data[54].mBoundingSphere = Sphere(Vector3(8.00037f, -5.01814f, -22.5975f), 3.56326f); 
  data[55].mBoundingSphere = Sphere(Vector3(-1.82469f, 10.0818f, -27.2662f), 2.454f); 
  data[56].mBoundingSphere = Sphere(Vector3(3.24557f, 8.71454f, -19.3903f), 0.520774f); 
  data[57].mBoundingSphere = Sphere(Vector3(37.4485f, 4.3531f, -11.9477f), 1.5354f); 
  data[58].mBoundingSphere = Sphere(Vector3(-4.24656f, 31.7335f, 31.3076f), 2.202f); 
  data[59].mBoundingSphere = Sphere(Vector3(44.3608f, -12.4646f, 5.50211f), 0.224122f); 
  data[60].mBoundingSphere = Sphere(Vector3(16.3232f, 17.3277f, 40.2806f), 3.56546f); 
  data[61].mBoundingSphere = Sphere(Vector3(11.8306f, -2.29324f, -26.8868f), 3.49912f); 
  data[62].mBoundingSphere = Sphere(Vector3(9.07189f, 11.0396f, -40.3678f), 1.34045f); 
  data[63].mBoundingSphere = Sphere(Vector3(-5.05246f, -38.8315f, 10.2548f), 2.05576f); 
  data[64].mBoundingSphere = Sphere(Vector3(-41.5336f, -9.01793f, -18.2106f), 3.83927f); 
  data[65].mBoundingSphere = Sphere(Vector3(32.3188f, -2.56795f, 26.1781f), 3.25628f); 
  data[66].mBoundingSphere = Sphere(Vector3(-46.8296f, 0.727412f, 13.162f), 3.24225f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-31.1368f, 92.9894f, 52.3529f), Vector3(-50.0227f, 64.3677f, -111.845f), Vector3(-50.0227f, -103.795f, -82.5318f), Vector3(-31.1368f, -75.1729f, 81.6657f), Vector3(57.8392f, -2.30056f, -23.8096f), Vector3(57.1469f, -3.34974f, -29.8285f), Vector3(57.1469f, -12.3212f, -28.2647f), Vector3(57.8392f, -11.2721f, -22.2457f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mBoundingSphere = Sphere(Vector3(9.34149f, -33.7233f, -34.3768f), 0.683596f); 
  data[1].mBoundingSphere = Sphere(Vector3(-11.9133f, 41.5334f, 10.5424f), 1.90024f); 
  data[2].mBoundingSphere = Sphere(Vector3(29.9367f, -9.16617f, -29.0495f), 0.50477f); 
  data[3].mBoundingSphere = Sphere(Vector3(19.977f, -8.54786f, -34.347f), 3.18693f); 
  data[4].mBoundingSphere = Sphere(Vector3(-18.9499f, 43.7471f, -2.8905f), 3.03733f); 
  data[5].mBoundingSphere = Sphere(Vector3(34.9647f, 6.20881f, 8.6527f), 1.36133f); 
  data[6].mBoundingSphere = Sphere(Vector3(-1.06442f, 13.7897f, 18.5577f), 1.86046f); 
  data[7].mBoundingSphere = Sphere(Vector3(20.4275f, 38.2467f, -23.5932f), 1.05145f); 
  data[8].mBoundingSphere = Sphere(Vector3(-16.9448f, 31.2842f, 22.3589f), 0.69067f); 
  data[9].mBoundingSphere = Sphere(Vector3(15.4623f, -34.0799f, -17.9441f), 1.47347f); 
  data[10].mBoundingSphere = Sphere(Vector3(-23.0391f, 22.1221f, 25.2547f), 3.77722f); 
  data[11].mBoundingSphere = Sphere(Vector3(-45.2437f, -1.10202f, -2.05316f), 1.31517f); 
  data[12].mBoundingSphere = Sphere(Vector3(18.6804f, -13.24f, 15.2052f), 1.07314f); 
  data[13].mBoundingSphere = Sphere(Vector3(2.76898f, 14.4068f, -46.2441f), 2.19434f); 
  data[14].mBoundingSphere = Sphere(Vector3(-10.2142f, 3.54912f, 2.18817f), 1.38243f); 
  data[15].mBoundingSphere = Sphere(Vector3(1.52174f, -1.18646f, 31.8423f), 3.093f); 
  data[16].mBoundingSphere = Sphere(Vector3(14.3298f, 24.3043f, -19.2769f), 3.27959f); 
  data[17].mBoundingSphere = Sphere(Vector3(43.471f, 2.84234f, 0.466684f), 0.965172f); 
  data[18].mBoundingSphere = Sphere(Vector3(-11.7817f, 0.669442f, -39.5485f), 2.60673f); 
  data[19].mBoundingSphere = Sphere(Vector3(-24.436f, 38.3305f, -7.25027f), 1.32143f); 
  data[20].mBoundingSphere = Sphere(Vector3(11.5194f, -4.56719f, 17.7856f), 2.34452f); 
  data[21].mBoundingSphere = Sphere(Vector3(10.0739f, -24.5692f, 18.2005f), 0.786462f); 
  data[22].mBoundingSphere = Sphere(Vector3(7.17617f, 1.64696f, 46.9278f), 2.05819f); 
  data[23].mBoundingSphere = Sphere(Vector3(-18.6059f, 0.528164f, 12.9926f), 2.43359f); 
  data[24].mBoundingSphere = Sphere(Vector3(-14.1772f, 16.8238f, 29.7611f), 1.11825f); 
  data[25].mBoundingSphere = Sphere(Vector3(-2.23762f, -10.7138f, -1.81725f), 3.09265f); 
  data[26].mBoundingSphere = Sphere(Vector3(-4.25026f, -40.9955f, 21.1304f), 0.49758f); 
  data[27].mBoundingSphere = Sphere(Vector3(-6.80727f, 22.0025f, 10.5565f), 2.03395f); 
  data[28].mBoundingSphere = Sphere(Vector3(-45.4042f, -1.31503f, 8.84222f), 3.14124f); 
  data[29].mBoundingSphere = Sphere(Vector3(36.0019f, -21.285f, 9.36111f), 0.626887f); 
  data[30].mBoundingSphere = Sphere(Vector3(18.293f, -1.38997f, 17.8967f), 1.5136f); 
  data[31].mBoundingSphere = Sphere(Vector3(-34.9499f, -23.0928f, -0.0396299f), 0.57354f); 
  data[32].mBoundingSphere = Sphere(Vector3(32.4931f, 1.79313f, 36.0971f), 1.71886f); 
  data[33].mBoundingSphere = Sphere(Vector3(-17.9312f, 41.746f, -18.284f), 0.716764f); 
  data[34].mBoundingSphere = Sphere(Vector3(-21.7991f, -11.9713f, 1.18461f), 0.597546f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-39.1491f, 36.6877f, -79.2335f), Vector3(-4.21863f, -86.9219f, -64.1839f), Vector3(-4.21863f, -71.5281f, 62.2528f), Vector3(-39.1491f, 52.0815f, 47.2031f), Vector3(12.872f, -3.70568f, -13.572f), Vector3(15.2305f, -12.0518f, -12.5559f), Vector3(15.2305f, -11.2332f, -5.83177f), Vector3(12.872f, -2.88701f, -6.84793f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(58); 
  data[0].mBoundingSphere = Sphere(Vector3(25.0304f, 14.779f, -34.1992f), 3.21117f); 
  data[1].mBoundingSphere = Sphere(Vector3(-9.07809f, -17.1749f, -19.1794f), 3.37944f); 
  data[2].mBoundingSphere = Sphere(Vector3(-10.56f, 17.8439f, 9.29266f), 2.97216f); 
  data[3].mBoundingSphere = Sphere(Vector3(7.65509f, 9.08233f, -24.8757f), 0.99718f); 
  data[4].mBoundingSphere = Sphere(Vector3(21.669f, 32.4254f, -26.9861f), 0.333598f); 
  data[5].mBoundingSphere = Sphere(Vector3(5.87664f, -23.2472f, -29.7711f), 0.743089f); 
  data[6].mBoundingSphere = Sphere(Vector3(-7.77493f, 23.0327f, -17.3469f), 0.853609f); 
  data[7].mBoundingSphere = Sphere(Vector3(-31.1392f, -22.0569f, 0.453051f), 1.16012f); 
  data[8].mBoundingSphere = Sphere(Vector3(5.22866f, -29.8185f, 32.9857f), 1.98606f); 
  data[9].mBoundingSphere = Sphere(Vector3(-25.1086f, 0.770589f, -1.89823f), 3.64884f); 
  data[10].mBoundingSphere = Sphere(Vector3(-24.4312f, -37.7616f, -9.93687f), 2.7735f); 
  data[11].mBoundingSphere = Sphere(Vector3(-28.7664f, 0.452349f, -19.424f), 3.39835f); 
  data[12].mBoundingSphere = Sphere(Vector3(-46.4376f, -8.84912f, 1.83796f), 2.61543f); 
  data[13].mBoundingSphere = Sphere(Vector3(-28.1305f, 6.69217f, -34.9311f), 3.59155f); 
  data[14].mBoundingSphere = Sphere(Vector3(-19.7233f, -6.92113f, -18.2206f), 0.5108f); 
  data[15].mBoundingSphere = Sphere(Vector3(-31.6909f, 6.47465f, 22.2544f), 3.48567f); 
  data[16].mBoundingSphere = Sphere(Vector3(13.6961f, 21.5979f, -17.1272f), 2.37653f); 
  data[17].mBoundingSphere = Sphere(Vector3(13.3796f, -32.0521f, -0.217301f), 2.16767f); 
  data[18].mBoundingSphere = Sphere(Vector3(-29.3961f, 33.8432f, -10.0394f), 2.23969f); 
  data[19].mBoundingSphere = Sphere(Vector3(-24.004f, -33.1314f, 9.87085f), 2.78463f); 
  data[20].mBoundingSphere = Sphere(Vector3(-9.32347f, -32.6936f, -18.1095f), 1.90627f); 
  data[21].mBoundingSphere = Sphere(Vector3(2.33735f, 31.3376f, -26.2425f), 2.04949f); 
  data[22].mBoundingSphere = Sphere(Vector3(10.9384f, -23.5546f, -3.03469f), 0.703659f); 
  data[23].mBoundingSphere = Sphere(Vector3(17.18f, -36.4628f, 8.11221f), 3.02144f); 
  data[24].mBoundingSphere = Sphere(Vector3(-27.0575f, -25.7423f, 27.9629f), 0.898953f); 
  data[25].mBoundingSphere = Sphere(Vector3(31.9259f, 33.2844f, 0.150608f), 1.76084f); 
  data[26].mBoundingSphere = Sphere(Vector3(9.82228f, 10.2796f, 17.8335f), 2.19167f); 
  data[27].mBoundingSphere = Sphere(Vector3(-38.0548f, -3.10082f, 15.1614f), 1.06073f); 
  data[28].mBoundingSphere = Sphere(Vector3(30.5559f, -21.9842f, -2.78654f), 0.481576f); 
  data[29].mBoundingSphere = Sphere(Vector3(-8.95922f, -25.2252f, -20.6615f), 1.85142f); 
  data[30].mBoundingSphere = Sphere(Vector3(22.7889f, 17.5301f, -37.7168f), 2.3124f); 
  data[31].mBoundingSphere = Sphere(Vector3(17.7139f, 24.0639f, 2.21011f), 2.6813f); 
  data[32].mBoundingSphere = Sphere(Vector3(1.22002f, 4.27968f, -36.7211f), 1.78856f); 
  data[33].mBoundingSphere = Sphere(Vector3(4.94433f, 3.57896f, -35.8042f), 3.47222f); 
  data[34].mBoundingSphere = Sphere(Vector3(-4.35935f, 18.2817f, 20.0441f), 3.5694f); 
  data[35].mBoundingSphere = Sphere(Vector3(17.3292f, 26.633f, -31.527f), 2.15004f); 
  data[36].mBoundingSphere = Sphere(Vector3(28.2768f, -2.11587f, 39.4764f), 0.736247f); 
  data[37].mBoundingSphere = Sphere(Vector3(-30.8238f, -15.0008f, -12.316f), 3.15029f); 
  data[38].mBoundingSphere = Sphere(Vector3(8.33754f, 4.02303f, -36.6075f), 2.53611f); 
  data[39].mBoundingSphere = Sphere(Vector3(38.3156f, 29.8262f, -3.45104f), 3.59805f); 
  data[40].mBoundingSphere = Sphere(Vector3(-6.81437f, -6.05848f, -34.5962f), 1.799f); 
  data[41].mBoundingSphere = Sphere(Vector3(-35.9786f, -9.66786f, 31.8874f), 2.76399f); 
  data[42].mBoundingSphere = Sphere(Vector3(11.1544f, -20.2449f, -40.7137f), 3.63121f); 
  data[43].mBoundingSphere = Sphere(Vector3(3.7238f, -8.64431f, -38.5298f), 0.594647f); 
  data[44].mBoundingSphere = Sphere(Vector3(-29.2509f, 0.465576f, -26.0636f), 1.38325f); 
  data[45].mBoundingSphere = Sphere(Vector3(0.454554f, -7.14665f, -16.5237f), 0.517875f); 
  data[46].mBoundingSphere = Sphere(Vector3(27.411f, 6.92499f, 24.1125f), 2.49285f); 
  data[47].mBoundingSphere = Sphere(Vector3(-17.9086f, 0.467129f, 1.05538f), 1.32317f); 
  data[48].mBoundingSphere = Sphere(Vector3(7.93078f, 29.6673f, 23.3282f), 1.60289f); 
  data[49].mBoundingSphere = Sphere(Vector3(-10.9192f, 5.51484f, -25.6748f), 0.917508f); 
  data[50].mBoundingSphere = Sphere(Vector3(-8.89577f, 6.11849f, -31.6187f), 3.55456f); 
  data[51].mBoundingSphere = Sphere(Vector3(20.1186f, -22.092f, 4.36362f), 0.555449f); 
  data[52].mBoundingSphere = Sphere(Vector3(-16.0633f, -14.0786f, 4.44162f), 1.25452f); 
  data[53].mBoundingSphere = Sphere(Vector3(-6.58425f, 16.9835f, -34.084f), 1.94489f); 
  data[54].mBoundingSphere = Sphere(Vector3(18.601f, -2.02857f, 41.0444f), 3.41934f); 
  data[55].mBoundingSphere = Sphere(Vector3(-7.34702f, 34.098f, -32.2937f), 1.0271f); 
  data[56].mBoundingSphere = Sphere(Vector3(29.259f, -0.389955f, -37.2561f), 1.99186f); 
  data[57].mBoundingSphere = Sphere(Vector3(14.4026f, -13.848f, -40.3554f), 1.23283f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(7.88315f, 48.4427f, 60.4418f), Vector3(-74.7533f, 12.5294f, 6.4649f), Vector3(-32.9196f, -83.7299f, 6.4649f), Vector3(49.7169f, -47.8166f, 60.4418f), Vector3(28.0024f, 2.45997f, -37.433f), Vector3(24.3433f, 0.869762f, -39.8231f), Vector3(26.1664f, -3.32509f, -39.8231f), Vector3(29.8255f, -1.73488f, -37.433f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(53); 
  data[0].mBoundingSphere = Sphere(Vector3(33.4333f, -24.5345f, 19.1969f), 3.70984f); 
  data[1].mBoundingSphere = Sphere(Vector3(26.6813f, 22.9497f, 20.3247f), 0.323856f); 
  data[2].mBoundingSphere = Sphere(Vector3(-40.3674f, -18.2474f, 3.71059f), 0.536198f); 
  data[3].mBoundingSphere = Sphere(Vector3(25.2698f, -7.14746f, -31.5156f), 0.786114f); 
  data[4].mBoundingSphere = Sphere(Vector3(31.41f, -1.87235f, -38.1455f), 2.7953f); 
  data[5].mBoundingSphere = Sphere(Vector3(5.36512f, -30.805f, 14.1372f), 0.250331f); 
  data[6].mBoundingSphere = Sphere(Vector3(10.0657f, -20.6806f, -10.3803f), 3.93703f); 
  data[7].mBoundingSphere = Sphere(Vector3(12.2145f, -28.2499f, -13.7171f), 0.946269f); 
  data[8].mBoundingSphere = Sphere(Vector3(21.0841f, -26.1648f, 22.2759f), 3.87093f); 
  data[9].mBoundingSphere = Sphere(Vector3(-7.25504f, 31.2202f, -22.5906f), 2.01053f); 
  data[10].mBoundingSphere = Sphere(Vector3(-1.47778f, -31.594f, -26.3573f), 0.905332f); 
  data[11].mBoundingSphere = Sphere(Vector3(-20.7352f, -31.9413f, -23.7258f), 0.214844f); 
  data[12].mBoundingSphere = Sphere(Vector3(6.65767f, -19.9818f, 13.7375f), 1.26333f); 
  data[13].mBoundingSphere = Sphere(Vector3(22.0649f, 5.89055f, -23.0986f), 0.668056f); 
  data[14].mBoundingSphere = Sphere(Vector3(21.3142f, -0.414891f, 17.7194f), 0.571221f); 
  data[15].mBoundingSphere = Sphere(Vector3(-6.12542f, -23.5183f, 26.6394f), 3.29316f); 
  data[16].mBoundingSphere = Sphere(Vector3(34.8165f, 2.75968f, 3.59925f), 1.92448f); 
  data[17].mBoundingSphere = Sphere(Vector3(-1.80976f, -44.7582f, -21.9061f), 3.07989f); 
  data[18].mBoundingSphere = Sphere(Vector3(-38.4476f, 10.3709f, 25.0415f), 2.12685f); 
  data[19].mBoundingSphere = Sphere(Vector3(14.0161f, -40.8614f, 8.51516f), 0.312491f); 
  data[20].mBoundingSphere = Sphere(Vector3(4.13806f, 13.2982f, -28.4056f), 3.10552f); 
  data[21].mBoundingSphere = Sphere(Vector3(41.1103f, 4.39568f, -6.53148f), 1.7714f); 
  data[22].mBoundingSphere = Sphere(Vector3(20.6965f, -38.202f, 11.604f), 2.90118f); 
  data[23].mBoundingSphere = Sphere(Vector3(39.7988f, 2.11975f, -7.76091f), 1.51093f); 
  data[24].mBoundingSphere = Sphere(Vector3(2.37871f, 13.2956f, 12.0058f), 0.432289f); 
  data[25].mBoundingSphere = Sphere(Vector3(32.3924f, 7.21223f, 1.45879f), 2.01134f); 
  data[26].mBoundingSphere = Sphere(Vector3(-4.45655f, 39.3748f, -29.8446f), 2.63549f); 
  data[27].mBoundingSphere = Sphere(Vector3(-0.486904f, 8.46175f, 47.8624f), 3.50678f); 
  data[28].mBoundingSphere = Sphere(Vector3(-12.4716f, -32.7458f, -26.517f), 3.5803f); 
  data[29].mBoundingSphere = Sphere(Vector3(15.3561f, 19.6432f, 19.5323f), 3.28678f); 
  data[30].mBoundingSphere = Sphere(Vector3(31.6953f, 2.59793f, 4.78662f), 3.95837f); 
  data[31].mBoundingSphere = Sphere(Vector3(24.2686f, -4.16082f, 32.3819f), 1.17438f); 
  data[32].mBoundingSphere = Sphere(Vector3(-25.727f, -25.8767f, -13.8736f), 2.54944f); 
  data[33].mBoundingSphere = Sphere(Vector3(-15.0605f, 0.809391f, 21.5442f), 3.57822f); 
  data[34].mBoundingSphere = Sphere(Vector3(35.0613f, 1.64501f, -16.8154f), 1.46013f); 
  data[35].mBoundingSphere = Sphere(Vector3(-15.857f, -33.5133f, 9.32568f), 3.31925f); 
  data[36].mBoundingSphere = Sphere(Vector3(41.8999f, 14.231f, -11.7885f), 2.33954f); 
  data[37].mBoundingSphere = Sphere(Vector3(-8.00371f, -10.4161f, 44.5462f), 3.63435f); 
  data[38].mBoundingSphere = Sphere(Vector3(-31.9611f, -4.99196f, -5.65958f), 1.16847f); 
  data[39].mBoundingSphere = Sphere(Vector3(-23.0577f, -29.8177f, 22.0709f), 2.75228f); 
  data[40].mBoundingSphere = Sphere(Vector3(-23.7175f, -30.465f, 10.5132f), 0.624799f); 
  data[41].mBoundingSphere = Sphere(Vector3(5.44041f, 35.6272f, 30.4151f), 3.64722f); 
  data[42].mBoundingSphere = Sphere(Vector3(11.1353f, -40.1392f, 21.7734f), 2.02305f); 
  data[43].mBoundingSphere = Sphere(Vector3(-26.9111f, 0.500615f, -38.1882f), 3.36541f); 
  data[44].mBoundingSphere = Sphere(Vector3(-26.0202f, 38.9445f, 12.9099f), 3.42931f); 
  data[45].mBoundingSphere = Sphere(Vector3(-8.61148f, -1.31965f, -31.6926f), 3.72759f); 
  data[46].mBoundingSphere = Sphere(Vector3(-19.1179f, -18.2096f, 21.0369f), 3.95083f); 
  data[47].mBoundingSphere = Sphere(Vector3(21.8098f, -30.5383f, 25.0042f), 1.60649f); 
  data[48].mBoundingSphere = Sphere(Vector3(18.1501f, -2.06044f, -9.19512f), 2.80121f); 
  data[49].mBoundingSphere = Sphere(Vector3(8.20107f, 8.95705f, -26.3426f), 0.579223f); 
  data[50].mBoundingSphere = Sphere(Vector3(30.5471f, 4.34298f, -28.6807f), 0.740422f); 
  data[51].mBoundingSphere = Sphere(Vector3(20.6834f, -10.4887f, -4.12703f), 2.15433f); 
  data[52].mBoundingSphere = Sphere(Vector3(14.8246f, -9.40187f, 37.1022f), 3.88902f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-23.0999f, -31.2134f, 14.5288f), Vector3(24.7837f, -60.7892f, -11.589f), Vector3(55.0672f, -11.76f, -11.589f), Vector3(7.18358f, 17.8159f, 14.5288f), Vector3(4.02978f, -16.751f, -15.209f), Vector3(10.9166f, -21.0047f, -18.9654f), Vector3(13.2842f, -17.1715f, -18.9654f), Vector3(6.39737f, -12.9178f, -15.209f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(74); 
  data[0].mBoundingSphere = Sphere(Vector3(11.643f, 23.1602f, -38.2147f), 0.659474f); 
  data[1].mBoundingSphere = Sphere(Vector3(-5.78809f, 42.2061f, 5.76347f), 1.66227f); 
  data[2].mBoundingSphere = Sphere(Vector3(-35.3717f, -22.9355f, -13.351f), 3.52359f); 
  data[3].mBoundingSphere = Sphere(Vector3(-21.5005f, -0.453573f, 34.3409f), 1.94756f); 
  data[4].mBoundingSphere = Sphere(Vector3(-4.92123f, -16.4753f, -18.4799f), 3.52615f); 
  data[5].mBoundingSphere = Sphere(Vector3(-15.4081f, -5.6973f, 29.2991f), 3.3595f); 
  data[6].mBoundingSphere = Sphere(Vector3(12.7321f, -2.38449f, 4.04313f), 2.11131f); 
  data[7].mBoundingSphere = Sphere(Vector3(32.0556f, 24.1632f, 7.75989f), 2.28816f); 
  data[8].mBoundingSphere = Sphere(Vector3(-6.66371f, -10.7644f, 1.01124f), 1.44007f); 
  data[9].mBoundingSphere = Sphere(Vector3(22.5482f, 15.8711f, 25.3423f), 1.8745f); 
  data[10].mBoundingSphere = Sphere(Vector3(-36.8469f, 10.0756f, 15.7366f), 1.37687f); 
  data[11].mBoundingSphere = Sphere(Vector3(4.93282f, 12.1767f, -2.25916f), 3.53403f); 
  data[12].mBoundingSphere = Sphere(Vector3(-14.4503f, -26.2685f, -29.5335f), 0.750743f); 
  data[13].mBoundingSphere = Sphere(Vector3(0.347583f, -24.087f, 27.0623f), 2.75657f); 
  data[14].mBoundingSphere = Sphere(Vector3(30.4609f, 19.7637f, -30.3915f), 2.8933f); 
  data[15].mBoundingSphere = Sphere(Vector3(-21.4536f, -7.37617f, 8.36754f), 0.580267f); 
  data[16].mBoundingSphere = Sphere(Vector3(-45.1473f, 9.32312f, -6.11255f), 2.64895f); 
  data[17].mBoundingSphere = Sphere(Vector3(42.163f, 7.45368f, -6.54192f), 0.340672f); 
  data[18].mBoundingSphere = Sphere(Vector3(-23.1302f, -15.4108f, -15.196f), 0.462905f); 
  data[19].mBoundingSphere = Sphere(Vector3(6.87504f, 20.7936f, -12.4984f), 1.09529f); 
  data[20].mBoundingSphere = Sphere(Vector3(24.901f, 6.07798f, -30.0708f), 1.74217f); 
  data[21].mBoundingSphere = Sphere(Vector3(11.5793f, 41.7706f, -20.9357f), 2.9224f); 
  data[22].mBoundingSphere = Sphere(Vector3(-17.9899f, 1.04988f, -18.3568f), 1.49249f); 
  data[23].mBoundingSphere = Sphere(Vector3(-36.7011f, 6.89897f, -20.7294f), 1.29546f); 
  data[24].mBoundingSphere = Sphere(Vector3(-3.24249f, -32.3638f, 25.1731f), 3.63005f); 
  data[25].mBoundingSphere = Sphere(Vector3(-19.8597f, 3.42648f, -10.327f), 0.932585f); 
  data[26].mBoundingSphere = Sphere(Vector3(2.16291f, 43.8116f, 22.3978f), 2.76805f); 
  data[27].mBoundingSphere = Sphere(Vector3(18.509f, 34.5111f, 11.3225f), 3.52023f); 
  data[28].mBoundingSphere = Sphere(Vector3(-15.068f, 10.6169f, 3.6465f), 0.67861f); 
  data[29].mBoundingSphere = Sphere(Vector3(28.4473f, 6.14517f, 33.0918f), 3.51281f); 
  data[30].mBoundingSphere = Sphere(Vector3(-20.197f, 23.3608f, -20.2193f), 0.429621f); 
  data[31].mBoundingSphere = Sphere(Vector3(-8.95083f, -25.3401f, 22.1262f), 3.40693f); 
  data[32].mBoundingSphere = Sphere(Vector3(-30.9977f, -1.65397f, 15.4404f), 0.996136f); 
  data[33].mBoundingSphere = Sphere(Vector3(0.165357f, 30.5255f, 31.429f), 2.89016f); 
  data[34].mBoundingSphere = Sphere(Vector3(-30.8579f, -10.4444f, 23.5025f), 3.74718f); 
  data[35].mBoundingSphere = Sphere(Vector3(12.6162f, 17.2215f, -8.72217f), 3.66601f); 
  data[36].mBoundingSphere = Sphere(Vector3(-1.62157f, 20.2484f, -18.3319f), 0.407355f); 
  data[37].mBoundingSphere = Sphere(Vector3(0.493479f, -27.8943f, 36.1003f), 3.35242f); 
  data[38].mBoundingSphere = Sphere(Vector3(12.4299f, -30.4982f, -33.403f), 3.2942f); 
  data[39].mBoundingSphere = Sphere(Vector3(-41.4408f, 6.35866f, -12.644f), 1.25614f); 
  data[40].mBoundingSphere = Sphere(Vector3(-1.58639f, -42.68f, -20.5606f), 3.82906f); 
  data[41].mBoundingSphere = Sphere(Vector3(-26.9481f, -14.7452f, -23.8312f), 0.822413f); 
  data[42].mBoundingSphere = Sphere(Vector3(-26.3583f, -14.0503f, -23.9368f), 3.72411f); 
  data[43].mBoundingSphere = Sphere(Vector3(-40.1682f, -22.6847f, 3.25033f), 1.15884f); 
  data[44].mBoundingSphere = Sphere(Vector3(35.1825f, 5.77535f, -4.45752f), 1.04438f); 
  data[45].mBoundingSphere = Sphere(Vector3(-5.19849f, 20.3265f, -24.8887f), 0.43426f); 
  data[46].mBoundingSphere = Sphere(Vector3(16.4618f, -24.8182f, 8.95458f), 3.34419f); 
  data[47].mBoundingSphere = Sphere(Vector3(-27.8315f, 6.71696f, -5.64357f), 3.55386f); 
  data[48].mBoundingSphere = Sphere(Vector3(-11.926f, 36.2921f, 9.23694f), 3.2789f); 
  data[49].mBoundingSphere = Sphere(Vector3(28.8451f, 19.2705f, 28.4186f), 3.70718f); 
  data[50].mBoundingSphere = Sphere(Vector3(-8.86699f, 44.498f, 16.92f), 0.291269f); 
  data[51].mBoundingSphere = Sphere(Vector3(-12.9691f, 25.5595f, -35.8064f), 2.39358f); 
  data[52].mBoundingSphere = Sphere(Vector3(22.9941f, 0.659352f, -41.9507f), 3.77444f); 
  data[53].mBoundingSphere = Sphere(Vector3(29.105f, -4.19489f, 28.4527f), 3.32076f); 
  data[54].mBoundingSphere = Sphere(Vector3(-17.9724f, 18.512f, 0.24647f), 2.50665f); 
  data[55].mBoundingSphere = Sphere(Vector3(-2.95916f, -13.0514f, -41.1287f), 2.979f); 
  data[56].mBoundingSphere = Sphere(Vector3(19.0388f, -7.03358f, 24.8331f), 2.96485f); 
  data[57].mBoundingSphere = Sphere(Vector3(17.3113f, -33.1737f, 28.0928f), 0.690554f); 
  data[58].mBoundingSphere = Sphere(Vector3(-18.9568f, -3.54087f, -40.3722f), 3.51037f); 
  data[59].mBoundingSphere = Sphere(Vector3(-21.5485f, -26.0438f, -9.56654f), 1.53389f); 
  data[60].mBoundingSphere = Sphere(Vector3(-31.9613f, 18.1633f, 17.3041f), 3.25083f); 
  data[61].mBoundingSphere = Sphere(Vector3(-9.09951f, -3.17353f, 15.241f), 0.742509f); 
  data[62].mBoundingSphere = Sphere(Vector3(8.04352f, 42.7335f, 3.14938f), 3.77711f); 
  data[63].mBoundingSphere = Sphere(Vector3(-12.1873f, -35.5409f, 30.372f), 2.77988f); 
  data[64].mBoundingSphere = Sphere(Vector3(-24.3888f, 35.5651f, -20.5735f), 3.77479f); 
  data[65].mBoundingSphere = Sphere(Vector3(5.5624f, 29.0309f, 16.7316f), 2.88808f); 
  data[66].mBoundingSphere = Sphere(Vector3(19.8015f, 13.639f, -25.6523f), 1.71086f); 
  data[67].mBoundingSphere = Sphere(Vector3(12.0824f, 34.1743f, -33.9372f), 3.31206f); 
  data[68].mBoundingSphere = Sphere(Vector3(-22.1275f, 18.1604f, -38.1166f), 2.79588f); 
  data[69].mBoundingSphere = Sphere(Vector3(17.3084f, 28.5618f, 15.5364f), 3.39916f); 
  data[70].mBoundingSphere = Sphere(Vector3(31.8757f, 27.5027f, 6.74961f), 3.62008f); 
  data[71].mBoundingSphere = Sphere(Vector3(-1.02432f, 4.95665f, 6.69557f), 1.57309f); 
  data[72].mBoundingSphere = Sphere(Vector3(13.5573f, 8.00132f, -40.4401f), 3.86524f); 
  data[73].mBoundingSphere = Sphere(Vector3(28.5336f, 27.3192f, 28.015f), 1.58341f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(0.526157f, -51.9333f, -4.26679f), Vector3(-9.70993f, -12.5393f, 66.3503f), Vector3(-9.70993f, 58.7894f, 26.5594f), Vector3(0.526157f, 19.3954f, -44.0578f), Vector3(24.1162f, 2.36422f, 13.566f), Vector3(23.6081f, 4.3198f, 17.0715f), Vector3(23.6081f, 8.00139f, 15.0177f), Vector3(24.1162f, 6.04581f, 11.5122f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mBoundingSphere = Sphere(Vector3(-33.5487f, -28.5612f, -9.92538f), 3.45981f); 
  data[1].mBoundingSphere = Sphere(Vector3(39.9808f, 1.53788f, -2.54862f), 0.900577f); 
  data[2].mBoundingSphere = Sphere(Vector3(-31.1105f, 13.3223f, -32.7016f), 1.13878f); 
  data[3].mBoundingSphere = Sphere(Vector3(21.9083f, -18.0403f, 6.13646f), 0.754222f); 
  data[4].mBoundingSphere = Sphere(Vector3(34.8169f, 23.4245f, -2.81222f), 1.28583f); 
  data[5].mBoundingSphere = Sphere(Vector3(-30.0585f, 21.1101f, -20.0722f), 1.37988f); 
  data[6].mBoundingSphere = Sphere(Vector3(18.2525f, -3.90262f, -38.0956f), 1.08775f); 
  data[7].mBoundingSphere = Sphere(Vector3(-27.0597f, -13.1177f, 33.3684f), 2.28178f); 
  data[8].mBoundingSphere = Sphere(Vector3(13.9022f, 5.26356f, 33.7492f), 2.45829f); 
  data[9].mBoundingSphere = Sphere(Vector3(-7.00325f, -37.4635f, -14.4565f), 3.89737f); 
  data[10].mBoundingSphere = Sphere(Vector3(-25.0343f, -21.5957f, 14.3773f), 1.2457f); 
  data[11].mBoundingSphere = Sphere(Vector3(22.4701f, 27.1417f, -8.49416f), 3.6943f); 
  data[12].mBoundingSphere = Sphere(Vector3(4.30282f, -5.34177f, -47.9646f), 1.61681f); 
  data[13].mBoundingSphere = Sphere(Vector3(27.9867f, -22.9646f, 23.5342f), 2.24085f); 
  data[14].mBoundingSphere = Sphere(Vector3(20.5674f, -25.6139f, -24.9542f), 2.41596f); 
  data[15].mBoundingSphere = Sphere(Vector3(-3.81666f, -28.7305f, 29.3089f), 0.252766f); 
  data[16].mBoundingSphere = Sphere(Vector3(-29.1689f, -1.65738f, -22.1269f), 1.09761f); 
  data[17].mBoundingSphere = Sphere(Vector3(4.90086f, 7.45367f, 19.6368f), 1.81373f); 
  data[18].mBoundingSphere = Sphere(Vector3(-15.8751f, -22.4142f, -26.1566f), 3.83613f); 
  data[19].mBoundingSphere = Sphere(Vector3(-35.5434f, -25.9626f, -19.6632f), 0.971435f); 
  data[20].mBoundingSphere = Sphere(Vector3(-10.5447f, 27.6385f, 12.7751f), 3.80842f); 
  data[21].mBoundingSphere = Sphere(Vector3(-11.5087f, -39.8059f, 5.84526f), 1.10932f); 
  data[22].mBoundingSphere = Sphere(Vector3(33.3623f, 9.47095f, 28.3881f), 3.65278f); 
  data[23].mBoundingSphere = Sphere(Vector3(3.52504f, -31.7715f, -35.7788f), 2.49204f); 
  data[24].mBoundingSphere = Sphere(Vector3(-41.6566f, 10.6813f, -12.1589f), 2.30301f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-18.8898f, 32.0175f, 166.385f), Vector3(-54.2723f, 157.09f, -49.7706f), Vector3(-54.2723f, -59.7419f, -175.234f), Vector3(-18.8898f, -184.814f, 40.9221f), Vector3(44.7528f, -7.26211f, -8.82495f), Vector3(43.528f, -2.93272f, -16.3072f), Vector3(43.528f, -9.0067f, -19.8217f), Vector3(44.7528f, -13.3361f, -12.3395f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mBoundingSphere = Sphere(Vector3(-19.2456f, 18.9172f, -11.2174f), 0.973406f); 
  data[1].mBoundingSphere = Sphere(Vector3(28.3935f, -18.4378f, -20.1243f), 1.4461f); 
  data[2].mBoundingSphere = Sphere(Vector3(-25.4125f, -36.2918f, -7.02994f), 2.26079f); 
  data[3].mBoundingSphere = Sphere(Vector3(34.6822f, -4.04503f, 33.5797f), 0.398077f); 
  data[4].mBoundingSphere = Sphere(Vector3(-31.0728f, 35.8985f, 14.111f), 1.85374f); 
  data[5].mBoundingSphere = Sphere(Vector3(-33.7709f, 17.7943f, -23.5507f), 2.45574f); 
  data[6].mBoundingSphere = Sphere(Vector3(24.7651f, 21.8443f, -27.3506f), 1.52264f); 
  data[7].mBoundingSphere = Sphere(Vector3(-36.0081f, -13.0013f, 21.2964f), 0.507553f); 
  data[8].mBoundingSphere = Sphere(Vector3(-5.61944f, -0.927432f, 12.1002f), 1.47231f); 
  data[9].mBoundingSphere = Sphere(Vector3(12.3576f, 38.3102f, -25.635f), 3.13266f); 
  data[10].mBoundingSphere = Sphere(Vector3(20.5407f, 16.2441f, -12.6528f), 1.72257f); 
  data[11].mBoundingSphere = Sphere(Vector3(29.4233f, -31.318f, -10.416f), 2.01783f); 
  data[12].mBoundingSphere = Sphere(Vector3(2.79745f, -38.5016f, 21.2071f), 3.96382f); 
  data[13].mBoundingSphere = Sphere(Vector3(-16.3008f, -9.30082f, -28.7524f), 0.651588f); 
  data[14].mBoundingSphere = Sphere(Vector3(24.1648f, -13.2132f, -37.5382f), 2.4867f); 
  data[15].mBoundingSphere = Sphere(Vector3(-7.65199f, 21.7294f, 42.7876f), 1.69149f); 
  data[16].mBoundingSphere = Sphere(Vector3(-10.7368f, 41.7149f, -8.52136f), 2.18878f); 
  data[17].mBoundingSphere = Sphere(Vector3(16.9842f, 1.54633f, 31.8525f), 2.40726f); 
  data[18].mBoundingSphere = Sphere(Vector3(3.53727f, 36.6885f, -27.5886f), 2.85421f); 
  data[19].mBoundingSphere = Sphere(Vector3(-29.9214f, 21.628f, -27.0396f), 1.9987f); 
  data[20].mBoundingSphere = Sphere(Vector3(3.79463f, -20.6386f, 20.8032f), 2.9093f); 
  data[21].mBoundingSphere = Sphere(Vector3(10.7377f, 19.4619f, 27.2494f), 1.78903f); 
  data[22].mBoundingSphere = Sphere(Vector3(0.188052f, 2.33972f, 39.3988f), 3.65336f); 
  data[23].mBoundingSphere = Sphere(Vector3(24.7968f, -0.887165f, -18.6967f), 2.09229f); 
  data[24].mBoundingSphere = Sphere(Vector3(-15.7276f, 3.24469f, -16.3358f), 2.56348f); 
  data[25].mBoundingSphere = Sphere(Vector3(39.3744f, 20.2431f, 4.23895f), 3.3566f); 
  data[26].mBoundingSphere = Sphere(Vector3(0.125365f, -37.9f, -11.5434f), 2.27575f); 
  data[27].mBoundingSphere = Sphere(Vector3(-7.63653f, 12.648f, 11.6149f), 3.98214f); 
  data[28].mBoundingSphere = Sphere(Vector3(-7.77482f, 28.1409f, -38.8011f), 1.04032f); 
  data[29].mBoundingSphere = Sphere(Vector3(2.01575f, 25.6337f, 12.7112f), 2.41411f); 
  data[30].mBoundingSphere = Sphere(Vector3(-21.9732f, 27.6404f, -25.4983f), 2.65208f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-155.165f, -30.546f, -15.6689f), Vector3(4.68864f, -138.094f, 101.341f), Vector3(133.64f, 53.5716f, 101.341f), Vector3(-26.2141f, 161.12f, -15.6689f), Vector3(17.8502f, -13.572f, -23.2478f), Vector3(20.749f, -15.5223f, -21.1259f), Vector3(26.1533f, -7.48967f, -21.1259f), Vector3(23.2545f, -5.53936f, -23.2478f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(49); 
  data[0].mBoundingSphere = Sphere(Vector3(-10.8146f, -36.7701f, -19.084f), 0.288833f); 
  data[1].mBoundingSphere = Sphere(Vector3(27.2191f, -4.81725f, 31.4067f), 2.47383f); 
  data[2].mBoundingSphere = Sphere(Vector3(-7.30962f, 5.91977f, -13.121f), 3.38025f); 
  data[3].mBoundingSphere = Sphere(Vector3(37.7927f, -17.6055f, 12.876f), 3.98086f); 
  data[4].mBoundingSphere = Sphere(Vector3(15.7641f, -32.2938f, -27.2771f), 1.06201f); 
  data[5].mBoundingSphere = Sphere(Vector3(-0.853749f, -5.94568f, -44.3846f), 2.9543f); 
  data[6].mBoundingSphere = Sphere(Vector3(-6.54778f, 11.0641f, -1.658f), 2.25395f); 
  data[7].mBoundingSphere = Sphere(Vector3(-38.5625f, -15.3448f, 9.42048f), 2.5535f); 
  data[8].mBoundingSphere = Sphere(Vector3(9.84678f, -0.47524f, 21.8727f), 0.499667f); 
  data[9].mBoundingSphere = Sphere(Vector3(0.207276f, -18.5563f, 46.2572f), 1.19792f); 
  data[10].mBoundingSphere = Sphere(Vector3(-18.7567f, 11.5183f, 36.2256f), 2.4823f); 
  data[11].mBoundingSphere = Sphere(Vector3(14.1544f, 17.7316f, 38.7517f), 3.26892f); 
  data[12].mBoundingSphere = Sphere(Vector3(-1.40447f, -9.29888f, 11.8315f), 1.91323f); 
  data[13].mBoundingSphere = Sphere(Vector3(-18.1532f, -15.4725f, 33.8494f), 1.78717f); 
  data[14].mBoundingSphere = Sphere(Vector3(-18.5453f, -29.0781f, 18.1153f), 1.39809f); 
  data[15].mBoundingSphere = Sphere(Vector3(-8.05915f, -46.0383f, 10.0968f), 1.46535f); 
  data[16].mBoundingSphere = Sphere(Vector3(-2.0297f, -34.1324f, -17.1389f), 2.4758f); 
  data[17].mBoundingSphere = Sphere(Vector3(-38.4431f, 3.11763f, -3.39244f), 2.33501f); 
  data[18].mBoundingSphere = Sphere(Vector3(-31.3095f, -33.5635f, 0.999006f), 1.83391f); 
  data[19].mBoundingSphere = Sphere(Vector3(-4.4124f, 40.1555f, 13.0821f), 0.949284f); 
  data[20].mBoundingSphere = Sphere(Vector3(-37.2985f, -21.1868f, -5.39248f), 1.29789f); 
  data[21].mBoundingSphere = Sphere(Vector3(5.37891f, -20.644f, 17.0497f), 1.56033f); 
  data[22].mBoundingSphere = Sphere(Vector3(45.9535f, 6.17413f, 3.78931f), 1.19677f); 
  data[23].mBoundingSphere = Sphere(Vector3(-10.9653f, 37.981f, -23.0355f), 3.07096f); 
  data[24].mBoundingSphere = Sphere(Vector3(-0.590137f, 29.948f, 36.4408f), 3.70207f); 
  data[25].mBoundingSphere = Sphere(Vector3(5.73585f, -18.2478f, 24.6546f), 3.56291f); 
  data[26].mBoundingSphere = Sphere(Vector3(-8.67648f, 29.5004f, 29.2487f), 3.34338f); 
  data[27].mBoundingSphere = Sphere(Vector3(-15.4597f, 0.462589f, 13.9649f), 3.39846f); 
  data[28].mBoundingSphere = Sphere(Vector3(23.5478f, -39.7466f, -5.75569f), 2.69789f); 
  data[29].mBoundingSphere = Sphere(Vector3(-1.20208f, 8.28124f, -0.651396f), 3.09833f); 
  data[30].mBoundingSphere = Sphere(Vector3(-14.455f, 36.6995f, -15.801f), 3.85515f); 
  data[31].mBoundingSphere = Sphere(Vector3(-25.7266f, 31.6828f, -22.1336f), 3.59352f); 
  data[32].mBoundingSphere = Sphere(Vector3(28.5383f, 25.2407f, -12.7257f), 2.25708f); 
  data[33].mBoundingSphere = Sphere(Vector3(16.292f, 9.97682f, -12.2597f), 1.25313f); 
  data[34].mBoundingSphere = Sphere(Vector3(21.2003f, -8.73653f, 20.6933f), 1.91288f); 
  data[35].mBoundingSphere = Sphere(Vector3(6.44874f, -20.6681f, 28.9825f), 0.486215f); 
  data[36].mBoundingSphere = Sphere(Vector3(10.9404f, 43.3139f, -9.01133f), 2.58864f); 
  data[37].mBoundingSphere = Sphere(Vector3(2.73739f, -3.68304f, -24.6374f), 2.11966f); 
  data[38].mBoundingSphere = Sphere(Vector3(-13.1925f, -37.5316f, -7.09752f), 0.772198f); 
  data[39].mBoundingSphere = Sphere(Vector3(1.80397f, -10.7856f, 7.19534f), 1.74994f); 
  data[40].mBoundingSphere = Sphere(Vector3(9.61273f, 25.8713f, -36.0026f), 0.613318f); 
  data[41].mBoundingSphere = Sphere(Vector3(-25.5885f, -6.04078f, 21.5283f), 1.87113f); 
  data[42].mBoundingSphere = Sphere(Vector3(-28.7182f, 30.9161f, -18.9962f), 0.638136f); 
  data[43].mBoundingSphere = Sphere(Vector3(29.2624f, -13.9127f, 12.7263f), 3.57659f); 
  data[44].mBoundingSphere = Sphere(Vector3(-8.58442f, -39.3278f, 22.114f), 1.83936f); 
  data[45].mBoundingSphere = Sphere(Vector3(4.06884f, 2.97266f, -21.8512f), 0.636628f); 
  data[46].mBoundingSphere = Sphere(Vector3(-11.9843f, -13.2794f, -23.7029f), 2.94073f); 
  data[47].mBoundingSphere = Sphere(Vector3(16.5945f, 43.5582f, -17.6179f), 2.55107f); 
  data[48].mBoundingSphere = Sphere(Vector3(-30.7951f, -35.031f, 3.5992f), 1.83692f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-124.911f, -35.9222f, 6.64405f), Vector3(6.80827f, -111.195f, 86.7315f), Vector3(90.4189f, 35.1146f, 86.7315f), Vector3(-41.3004f, 110.387f, 6.64405f), Vector3(5.10756f, -16.4689f, -14.5491f), Vector3(11.8268f, -20.3087f, -10.4637f), Vector3(14.662f, -15.3474f, -10.4637f), Vector3(7.94277f, -11.5076f, -14.5491f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mBoundingSphere = Sphere(Vector3(-6.33161f, 36.1289f, -32.1465f), 0.728361f); 
  data[1].mBoundingSphere = Sphere(Vector3(6.09256f, -2.44672f, 37.7741f), 3.52023f); 
  data[2].mBoundingSphere = Sphere(Vector3(41.5385f, 14.1794f, -13.703f), 1.54572f); 
  data[3].mBoundingSphere = Sphere(Vector3(-20.0507f, 28.1378f, 28.635f), 2.21638f); 
  data[4].mBoundingSphere = Sphere(Vector3(-16.6411f, 8.99386f, -34.8964f), 1.60498f); 
  data[5].mBoundingSphere = Sphere(Vector3(-10.8851f, -33.4702f, 16.5408f), 0.409095f); 
  data[6].mBoundingSphere = Sphere(Vector3(22.4402f, -15.3047f, -3.05055f), 3.05878f); 
  data[7].mBoundingSphere = Sphere(Vector3(-1.20873f, 24.4361f, -34.5647f), 0.563103f); 
  data[8].mBoundingSphere = Sphere(Vector3(2.56949f, 35.6478f, -17.0687f), 0.389496f); 
  data[9].mBoundingSphere = Sphere(Vector3(16.4559f, 25.6229f, -2.49247f), 1.3691f); 
  data[10].mBoundingSphere = Sphere(Vector3(-18.2599f, 26.9032f, 12.2532f), 3.52151f); 
  data[11].mBoundingSphere = Sphere(Vector3(7.83131f, 23.3995f, -25.661f), 3.99385f); 
  data[12].mBoundingSphere = Sphere(Vector3(30.1933f, -13.9598f, 28.8782f), 0.60694f); 
  data[13].mBoundingSphere = Sphere(Vector3(1.69167f, 24.5868f, -25.0569f), 3.60361f); 
  data[14].mBoundingSphere = Sphere(Vector3(-2.04789f, -8.3397f, 36.0897f), 1.08891f); 
  data[15].mBoundingSphere = Sphere(Vector3(0.757586f, 34.275f, -32.5658f), 0.363866f); 
  data[16].mBoundingSphere = Sphere(Vector3(-1.69071f, 41.0343f, -0.85373f), 3.23692f); 
  data[17].mBoundingSphere = Sphere(Vector3(35.1003f, -11.3836f, 11.4038f), 3.31392f); 
  data[18].mBoundingSphere = Sphere(Vector3(-15.1772f, 3.57372f, 29.0584f), 3.9586f); 
  data[19].mBoundingSphere = Sphere(Vector3(22.3091f, 23.054f, -38.2842f), 0.514744f); 
  data[20].mBoundingSphere = Sphere(Vector3(-21.741f, -31.7692f, 0.692098f), 1.4875f); 
  data[21].mBoundingSphere = Sphere(Vector3(-34.4477f, -3.33288f, -3.53867f), 1.87473f); 
  data[22].mBoundingSphere = Sphere(Vector3(-14.1042f, 42.0222f, -13.2478f), 2.34406f); 
  data[23].mBoundingSphere = Sphere(Vector3(-26.7355f, 40.165f, -9.04432f), 2.64802f); 
  data[24].mBoundingSphere = Sphere(Vector3(-24.1483f, 25.3942f, -27.801f), 2.17208f); 
  data[25].mBoundingSphere = Sphere(Vector3(-6.01117f, 0.153336f, -3.53698f), 0.608679f); 
  data[26].mBoundingSphere = Sphere(Vector3(-6.99372f, 22.4525f, 39.7049f), 1.25104f); 
  data[27].mBoundingSphere = Sphere(Vector3(-7.49798f, 25.2429f, -2.80032f), 2.3008f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-89.6871f, -93.2409f, -7.99578f), Vector3(38.0086f, -78.5461f, 123.468f), Vector3(38.0086f, 107.546f, 102.667f), Vector3(-89.6871f, 92.8516f, -28.7968f), Vector3(41.8153f, -4.95542f, -20.8857f), Vector3(45.6487f, -4.51429f, -16.9392f), Vector3(45.6487f, 4.34015f, -17.9289f), Vector3(41.8153f, 3.89901f, -21.8754f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(-36.0248f, 31.0525f, -2.3004f), 2.67666f); 
  data[1].mBoundingSphere = Sphere(Vector3(-26.0466f, -34.2901f, -3.21944f), 2.462f); 
  data[2].mBoundingSphere = Sphere(Vector3(-4.79903f, 37.5018f, -11.1634f), 3.84205f); 
  data[3].mBoundingSphere = Sphere(Vector3(-20.1822f, -18.4273f, -10.4788f), 2.69765f); 
  data[4].mBoundingSphere = Sphere(Vector3(1.49577f, 14.4679f, -37.972f), 3.17116f); 
  data[5].mBoundingSphere = Sphere(Vector3(24.083f, 20.451f, 26.5515f), 3.71123f); 
  data[6].mBoundingSphere = Sphere(Vector3(-3.86272f, -22.3689f, 23.2491f), 1.0576f); 
  data[7].mBoundingSphere = Sphere(Vector3(-14.6385f, -7.75988f, 15.5957f), 3.25791f); 
  data[8].mBoundingSphere = Sphere(Vector3(-12.2812f, -11.3742f, -24.5623f), 2.90953f); 
  data[9].mBoundingSphere = Sphere(Vector3(7.8813f, 32.649f, -7.78519f), 0.522629f); 
  data[10].mBoundingSphere = Sphere(Vector3(45.7411f, 6.78078f, 10.3985f), 2.59734f); 
  data[11].mBoundingSphere = Sphere(Vector3(-13.6467f, -5.5245f, -3.5139f), 0.706095f); 
  data[12].mBoundingSphere = Sphere(Vector3(-8.30541f, -14.7148f, 13.6303f), 0.579571f); 
  data[13].mBoundingSphere = Sphere(Vector3(-3.09442f, -46.6054f, 10.9192f), 1.08636f); 
  data[14].mBoundingSphere = Sphere(Vector3(25.7241f, -8.1849f, 4.00693f), 3.50121f); 
  data[15].mBoundingSphere = Sphere(Vector3(-7.40905f, 39.3418f, 24.3562f), 0.35598f); 
  data[16].mBoundingSphere = Sphere(Vector3(-28.3925f, -19.2887f, -9.86038f), 1.46756f); 
  data[17].mBoundingSphere = Sphere(Vector3(-38.5022f, -29.7292f, 8.23448f), 0.833662f); 
  data[18].mBoundingSphere = Sphere(Vector3(27.6734f, -12.0488f, -0.527887f), 2.7946f); 
  data[19].mBoundingSphere = Sphere(Vector3(15.8598f, 15.9368f, -14.808f), 3.55073f); 
  data[20].mBoundingSphere = Sphere(Vector3(16.84f, -32.8365f, -1.89793f), 0.832038f); 
  data[21].mBoundingSphere = Sphere(Vector3(27.9533f, -6.57359f, 7.7141f), 3.81294f); 
  data[22].mBoundingSphere = Sphere(Vector3(13.2782f, 27.8075f, 0.845764f), 2.06364f); 
  data[23].mBoundingSphere = Sphere(Vector3(-7.75093f, 25.0186f, -35.3317f), 2.4373f); 
  data[24].mBoundingSphere = Sphere(Vector3(21.147f, 20.8221f, -35.8236f), 1.4802f); 
  data[25].mBoundingSphere = Sphere(Vector3(-6.89037f, 10.9188f, 31.7479f), 2.26392f); 
  data[26].mBoundingSphere = Sphere(Vector3(8.39692f, -33.3247f, 29.3338f), 0.22992f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-45.5346f, 31.0618f, -23.2166f), Vector3(-2.21943f, -5.52127f, -77.5021f), Vector3(-2.21943f, -70.4003f, -33.7799f), Vector3(-45.5346f, -33.8172f, 20.5055f), Vector3(17.5228f, -0.590452f, -4.58303f), Vector3(20.3766f, -3.00069f, -8.15956f), Vector3(20.3766f, -7.07489f, -5.41395f), Vector3(17.5228f, -4.66465f, -1.83741f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(44); 
  data[0].mBoundingSphere = Sphere(Vector3(3.36778f, -33.7776f, 15.7664f), 1.70356f); 
  data[1].mBoundingSphere = Sphere(Vector3(41.2831f, -23.3822f, 6.03848f), 2.8701f); 
  data[2].mBoundingSphere = Sphere(Vector3(13.529f, 34.6701f, -33.0425f), 3.69755f); 
  data[3].mBoundingSphere = Sphere(Vector3(24.2884f, -4.82415f, 23.9847f), 1.48901f); 
  data[4].mBoundingSphere = Sphere(Vector3(-25.8508f, 6.19698f, 11.9096f), 1.2602f); 
  data[5].mBoundingSphere = Sphere(Vector3(26.8455f, -8.81463f, -16.5799f), 3.96904f); 
  data[6].mBoundingSphere = Sphere(Vector3(3.52868f, -17.0341f, 10.8328f), 1.19375f); 
  data[7].mBoundingSphere = Sphere(Vector3(-8.35001f, -4.05573f, 26.7739f), 2.03187f); 
  data[8].mBoundingSphere = Sphere(Vector3(-10.5844f, 3.57795f, 30.0514f), 2.08289f); 
  data[9].mBoundingSphere = Sphere(Vector3(4.69092f, -11.893f, -39.9097f), 3.88994f); 
  data[10].mBoundingSphere = Sphere(Vector3(-15.7261f, 3.44293f, -9.61286f), 2.50363f); 
  data[11].mBoundingSphere = Sphere(Vector3(-38.1413f, -19.2085f, 11.5069f), 3.74127f); 
  data[12].mBoundingSphere = Sphere(Vector3(-8.82217f, 35.1975f, 5.49142f), 0.206494f); 
  data[13].mBoundingSphere = Sphere(Vector3(-45.6237f, -4.33474f, -18.2019f), 3.04127f); 
  data[14].mBoundingSphere = Sphere(Vector3(-10.521f, -23.4093f, -6.02696f), 1.27586f); 
  data[15].mBoundingSphere = Sphere(Vector3(44.8224f, -15.7914f, 8.81651f), 3.74869f); 
  data[16].mBoundingSphere = Sphere(Vector3(-8.99739f, -35.2368f, 3.05288f), 1.26252f); 
  data[17].mBoundingSphere = Sphere(Vector3(35.085f, -26.8285f, -22.6399f), 3.94665f); 
  data[18].mBoundingSphere = Sphere(Vector3(7.76294f, -11.1067f, 46.1365f), 2.8933f); 
  data[19].mBoundingSphere = Sphere(Vector3(-8.98736f, 40.5391f, -6.98678f), 3.43859f); 
  data[20].mBoundingSphere = Sphere(Vector3(3.25237f, 16.7829f, 40.5633f), 1.95394f); 
  data[21].mBoundingSphere = Sphere(Vector3(3.34875f, 19.5828f, 25.9537f), 0.319681f); 
  data[22].mBoundingSphere = Sphere(Vector3(19.6268f, -4.06283f, 41.3295f), 2.14784f); 
  data[23].mBoundingSphere = Sphere(Vector3(-44.4494f, 17.7664f, 8.25268f), 1.32282f); 
  data[24].mBoundingSphere = Sphere(Vector3(-3.42158f, -32.1021f, -37.7808f), 1.93005f); 
  data[25].mBoundingSphere = Sphere(Vector3(10.1705f, -18.3674f, -18.1616f), 0.339628f); 
  data[26].mBoundingSphere = Sphere(Vector3(-7.10909f, 47.3144f, -1.91289f), 2.71528f); 
  data[27].mBoundingSphere = Sphere(Vector3(31.7649f, 36.7336f, 11.2951f), 1.1164f); 
  data[28].mBoundingSphere = Sphere(Vector3(14.2184f, 44.1367f, 12.4819f), 0.461861f); 
  data[29].mBoundingSphere = Sphere(Vector3(-1.15902f, -24.7791f, -20.0974f), 0.525529f); 
  data[30].mBoundingSphere = Sphere(Vector3(14.6339f, -22.7188f, -28.2378f), 2.43034f); 
  data[31].mBoundingSphere = Sphere(Vector3(-25.5023f, -19.5436f, -25.885f), 0.811628f); 
  data[32].mBoundingSphere = Sphere(Vector3(5.2489f, 39.4251f, 3.89916f), 0.71688f); 
  data[33].mBoundingSphere = Sphere(Vector3(-5.44952f, 17.602f, -10.7283f), 1.41201f); 
  data[34].mBoundingSphere = Sphere(Vector3(1.26632f, 5.76688f, -43.628f), 1.36191f); 
  data[35].mBoundingSphere = Sphere(Vector3(12.3415f, 35.8897f, -18.6463f), 0.753758f); 
  data[36].mBoundingSphere = Sphere(Vector3(-6.11782f, -23.5076f, 0.319533f), 2.63387f); 
  data[37].mBoundingSphere = Sphere(Vector3(2.28832f, -15.0046f, -41.7722f), 0.423707f); 
  data[38].mBoundingSphere = Sphere(Vector3(15.2503f, 3.81856f, 11.2218f), 2.28758f); 
  data[39].mBoundingSphere = Sphere(Vector3(44.0049f, 6.46305f, -21.2744f), 0.315738f); 
  data[40].mBoundingSphere = Sphere(Vector3(-31.3974f, 12.1879f, 29.7022f), 3.39811f); 
  data[41].mBoundingSphere = Sphere(Vector3(33.9863f, 2.99576f, 12.8727f), 3.75078f); 
  data[42].mBoundingSphere = Sphere(Vector3(-4.96203f, -9.57353f, 29.4636f), 3.56917f); 
  data[43].mBoundingSphere = Sphere(Vector3(-30.53f, -13.4146f, -36.0664f), 2.17347f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-23.142f, -34.5889f, 27.9484f), Vector3(2.764f, 13.1633f, 60.7937f), Vector3(2.764f, 51.1616f, 5.54983f), Vector3(-23.142f, 3.40939f, -27.2955f), Vector3(22.5348f, -7.74071f, 10.3391f), Vector3(24.182f, -4.70452f, 12.4275f), Vector3(24.182f, -0.395867f, 6.16334f), Vector3(22.5348f, -3.43205f, 4.07497f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(32); 
  data[0].mBoundingSphere = Sphere(Vector3(7.92863f, -4.10898f, -46.5013f), 3.28319f); 
  data[1].mBoundingSphere = Sphere(Vector3(-22.9152f, -22.6454f, 10.4193f), 2.6479f); 
  data[2].mBoundingSphere = Sphere(Vector3(10.0787f, -37.8768f, 5.8821f), 1.06062f); 
  data[3].mBoundingSphere = Sphere(Vector3(-36.5957f, 12.5196f, 28.6212f), 2.78823f); 
  data[4].mBoundingSphere = Sphere(Vector3(17.1005f, 5.16071f, -7.17738f), 3.51548f); 
  data[5].mBoundingSphere = Sphere(Vector3(19.2087f, -23.366f, -0.667582f), 1.6429f); 
  data[6].mBoundingSphere = Sphere(Vector3(-23.7178f, -11.1514f, -42.0665f), 3.35857f); 
  data[7].mBoundingSphere = Sphere(Vector3(35.7736f, 31.9962f, -2.63609f), 2.12627f); 
  data[8].mBoundingSphere = Sphere(Vector3(-11.9246f, 32.065f, -7.31225f), 2.53413f); 
  data[9].mBoundingSphere = Sphere(Vector3(1.32302f, 33.114f, 7.51788f), 0.74796f); 
  data[10].mBoundingSphere = Sphere(Vector3(20.597f, 11.9281f, 29.575f), 2.91243f); 
  data[11].mBoundingSphere = Sphere(Vector3(8.5343f, 14.9092f, 15.9982f), 1.34358f); 
  data[12].mBoundingSphere = Sphere(Vector3(37.9907f, 12.4338f, 26.2853f), 1.258f); 
  data[13].mBoundingSphere = Sphere(Vector3(-11.2638f, 2.49997f, -6.38358f), 2.68014f); 
  data[14].mBoundingSphere = Sphere(Vector3(-0.923585f, -36.8299f, 0.981904f), 3.67285f); 
  data[15].mBoundingSphere = Sphere(Vector3(-0.578564f, -38.8039f, -14.8133f), 1.73603f); 
  data[16].mBoundingSphere = Sphere(Vector3(8.16622f, -31.6398f, 5.4608f), 2.51372f); 
  data[17].mBoundingSphere = Sphere(Vector3(-13.3109f, -21.0574f, -19.9224f), 1.25742f); 
  data[18].mBoundingSphere = Sphere(Vector3(11.0307f, 33.7529f, -9.69583f), 3.18589f); 
  data[19].mBoundingSphere = Sphere(Vector3(-27.0862f, -2.3483f, -24.4914f), 2.58783f); 
  data[20].mBoundingSphere = Sphere(Vector3(-17.4234f, 31.8029f, -29.2336f), 2.01018f); 
  data[21].mBoundingSphere = Sphere(Vector3(-37.1942f, -8.83331f, -16.7575f), 2.77431f); 
  data[22].mBoundingSphere = Sphere(Vector3(-22.7128f, 8.90697f, -3.71985f), 2.56695f); 
  data[23].mBoundingSphere = Sphere(Vector3(1.09641f, -25.0348f, -39.8523f), 3.54714f); 
  data[24].mBoundingSphere = Sphere(Vector3(21.1643f, -17.8541f, 22.4693f), 2.11351f); 
  data[25].mBoundingSphere = Sphere(Vector3(-28.4305f, -0.223518f, -15.6011f), 1.77615f); 
  data[26].mBoundingSphere = Sphere(Vector3(28.6459f, -10.7516f, 26.3514f), 1.2769f); 
  data[27].mBoundingSphere = Sphere(Vector3(-28.2559f, -5.0762f, 26.7528f), 3.20062f); 
  data[28].mBoundingSphere = Sphere(Vector3(5.03328f, 34.4654f, -28.3876f), 2.82255f); 
  data[29].mBoundingSphere = Sphere(Vector3(5.01899f, -25.846f, -29.3443f), 2.43486f); 
  data[30].mBoundingSphere = Sphere(Vector3(10.7066f, -48.492f, 1.58455f), 3.29629f); 
  data[31].mBoundingSphere = Sphere(Vector3(1.33465f, -15.6294f, 31.6283f), 1.44865f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(1.09873f, -74.1972f, 65.5745f), Vector3(-24.5682f, 25.209f, 72.6188f), Vector3(-24.5682f, 32.6688f, -32.649f), Vector3(1.09873f, -66.7375f, -39.6933f), Vector3(23.6097f, -14.2301f, 20.4756f), Vector3(22.4978f, -9.92381f, 20.7808f), Vector3(22.4978f, -9.42321f, 13.7166f), Vector3(23.6097f, -13.7295f, 13.4114f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionFrustumCastFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mBoundingSphere = Sphere(Vector3(12.3431f, -10.2916f, -43.0183f), 0.291037f); 
  data[1].mBoundingSphere = Sphere(Vector3(17.8116f, 15.2079f, -18.8203f), 3.5607f); 
  data[2].mBoundingSphere = Sphere(Vector3(-31.1692f, 5.95817f, 3.16211f), 2.96288f); 
  data[3].mBoundingSphere = Sphere(Vector3(6.72989f, -29.3558f, -5.7346f), 2.20374f); 
  data[4].mBoundingSphere = Sphere(Vector3(-20.1867f, 5.20089f, -28.8682f), 1.54688f); 
  data[5].mBoundingSphere = Sphere(Vector3(21.1417f, -14.5741f, 3.31796f), 2.80597f); 
  data[6].mBoundingSphere = Sphere(Vector3(-37.3987f, 11.9904f, -3.93294f), 1.3691f); 
  data[7].mBoundingSphere = Sphere(Vector3(-22.4043f, 19.1701f, -3.72702f), 3.37828f); 
  data[8].mBoundingSphere = Sphere(Vector3(-4.9198f, 31.1561f, -24.3962f), 1.67885f); 
  data[9].mBoundingSphere = Sphere(Vector3(6.80664f, 23.2168f, 8.34387f), 1.85965f); 
  data[10].mBoundingSphere = Sphere(Vector3(7.14534f, 10.0212f, 47.4881f), 1.60197f); 
  data[11].mBoundingSphere = Sphere(Vector3(22.4189f, 12.1861f, 22.9823f), 3.25686f); 
  data[12].mBoundingSphere = Sphere(Vector3(34.8263f, -10.3282f, -2.38083f), 0.867757f); 
  data[13].mBoundingSphere = Sphere(Vector3(-12.7393f, 26.2438f, -19.2194f), 2.55965f); 
  data[14].mBoundingSphere = Sphere(Vector3(-29.7562f, -23.6576f, 31.0871f), 2.91127f); 
  data[15].mBoundingSphere = Sphere(Vector3(6.02837f, -25.2918f, -27.9942f), 0.430317f); 
  data[16].mBoundingSphere = Sphere(Vector3(-24.0545f, -18.2444f, 37.4615f), 1.35066f); 
  data[17].mBoundingSphere = Sphere(Vector3(21.6969f, -6.15485f, 23.4251f), 2.26868f); 
  data[18].mBoundingSphere = Sphere(Vector3(-4.85046f, -6.72464f, -34.7238f), 3.03977f); 
  data[19].mBoundingSphere = Sphere(Vector3(24.7909f, 30.789f, -17.1302f), 0.651704f); 
  data[20].mBoundingSphere = Sphere(Vector3(-29.0907f, 11.5887f, -6.51359f), 0.295096f); 
  data[21].mBoundingSphere = Sphere(Vector3(13.129f, -25.9425f, -25.098f), 1.94872f); 
  data[22].mBoundingSphere = Sphere(Vector3(-21.687f, -5.49882f, -19.9887f), 2.49876f); 
  data[23].mBoundingSphere = Sphere(Vector3(25.244f, 19.5035f, 33.2336f), 2.11107f); 
  data[24].mBoundingSphere = Sphere(Vector3(-10.7599f, 6.2051f, -42.8064f), 1.67491f); 
  data[25].mBoundingSphere = Sphere(Vector3(-2.01844f, 8.3231f, 10.5346f), 2.83682f); 
  data[26].mBoundingSphere = Sphere(Vector3(-29.5889f, -16.2492f, 1.20509f), 1.24292f); 
  data[27].mBoundingSphere = Sphere(Vector3(-6.31784f, -4.05883f, -27.7706f), 3.51072f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-52.6887f, 58.183f, 18.2189f), Vector3(-52.0938f, -1.19486f, -71.0995f), Vector3(-52.0938f, -93.619f, -9.65697f), Vector3(-52.6887f, -34.2411f, 79.6614f), Vector3(24.0041f, -12.0222f, 4.38909f), Vector3(24.0353f, -15.144f, -0.306859f), Vector3(24.0353f, -22.9445f, 4.87876f), Vector3(24.0041f, -19.8226f, 9.57471f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumSphereTests", static_cast<int>(Application::mStatistics.mFrustumSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(51); 
  data[0].mBoundingSphere = Sphere(Vector3(0, -0, 0), 2.50758f); 
  data[1].mBoundingSphere = Sphere(Vector3(2.69623f, 19.4254f, 37.3266f), 0.59859f); 
  data[2].mBoundingSphere = Sphere(Vector3(-17.5457f, -15.1591f, -39.0424f), 0.817658f); 
  data[3].mBoundingSphere = Sphere(Vector3(27.8845f, -22.305f, -6.40712f), 1.47973f); 
  data[4].mBoundingSphere = Sphere(Vector3(-11.8679f, -4.96141f, -13.527f), 2.52068f); 
  data[5].mBoundingSphere = Sphere(Vector3(-27.6664f, -19.8129f, -33.8513f), 2.26845f); 
  data[6].mBoundingSphere = Sphere(Vector3(2.3407f, -29.4768f, 10.9676f), 3.87208f); 
  data[7].mBoundingSphere = Sphere(Vector3(-14.3804f, 37.4119f, 24.0843f), 2.05274f); 
  data[8].mBoundingSphere = Sphere(Vector3(-16.4982f, 1.58027f, -19.3367f), 2.55072f); 
  data[9].mBoundingSphere = Sphere(Vector3(-1.73636f, -36.522f, -2.07099f), 3.13277f); 
  data[10].mBoundingSphere = Sphere(Vector3(32.6371f, 11.2948f, 22.9751f), 3.73072f); 
  data[11].mBoundingSphere = Sphere(Vector3(1.10206f, 40.3932f, 1.63285f), 1.17995f); 
  data[12].mBoundingSphere = Sphere(Vector3(3.67683f, -33.3144f, -0.490011f), 1.92633f); 
  data[13].mBoundingSphere = Sphere(Vector3(2.95695f, -19.4389f, -20.6844f), 0.569829f); 
  data[14].mBoundingSphere = Sphere(Vector3(11.7022f, -3.32931f, 44.9942f), 3.2143f); 
  data[15].mBoundingSphere = Sphere(Vector3(36.8277f, 1.91903f, -28.8203f), 1.69126f); 
  data[16].mBoundingSphere = Sphere(Vector3(17.7058f, 24.0818f, 19.0838f), 0.428694f); 
  data[17].mBoundingSphere = Sphere(Vector3(-41.0813f, -26.8734f, -6.01176f), 2.02966f); 
  data[18].mBoundingSphere = Sphere(Vector3(-15.7179f, 14.3788f, 12.1571f), 3.93332f); 
  data[19].mBoundingSphere = Sphere(Vector3(-48.1695f, 1.24722f, -9.77584f), 3.42548f); 
  data[20].mBoundingSphere = Sphere(Vector3(-27.3352f, -25.5117f, -9.25006f), 2.56672f); 
  data[21].mBoundingSphere = Sphere(Vector3(-30.1807f, -0.723544f, 32.8346f), 2.70832f); 
  data[22].mBoundingSphere = Sphere(Vector3(-11.9258f, 31.9933f, 0.532535f), 2.9638f); 
  data[23].mBoundingSphere = Sphere(Vector3(7.67931f, -14.2594f, -42.675f), 3.70509f); 
  data[24].mBoundingSphere = Sphere(Vector3(35.033f, 10.3165f, 26.9304f), 2.85189f); 
  data[25].mBoundingSphere = Sphere(Vector3(16.1442f, -22.5596f, -17.0795f), 1.22355f); 
  data[26].mBoundingSphere = Sphere(Vector3(-25.5479f, 19.8757f, 24.4525f), 0.921683f); 
  data[27].mBoundingSphere = Sphere(Vector3(31.4708f, -5.47642f, -9.33483f), 1.61159f); 
  data[28].mBoundingSphere = Sphere(Vector3(13.491f, -45.2442f, 15.6148f), 1.47359f); 
  data[29].mBoundingSphere = Sphere(Vector3(-10.1644f, -10.2274f, 11.3755f), 3.88994f); 
  data[30].mBoundingSphere = Sphere(Vector3(-32.2222f, -32.2732f, -9.70015f), 2.43858f); 
  data[31].mBoundingSphere = Sphere(Vector3(-18.3609f, 20.9918f, -25.3397f), 2.26102f); 
  data[32].mBoundingSphere = Sphere(Vector3(-11.3878f, 12.6137f, 21.0738f), 0.974102f); 
  data[33].mBoundingSphere = Sphere(Vector3(30.3302f, 26.7945f, -6.83283f), 2.393f); 
  data[34].mBoundingSphere = Sphere(Vector3(38.615f, -22.3331f, -7.93274f), 1.43578f); 
  data[35].mBoundingSphere = Sphere(Vector3(-26.5758f, -6.18249f, 28.4558f), 2.07779f); 
  data[36].mBoundingSphere = Sphere(Vector3(18.4839f, 0.643563f, 16.9532f), 2.92322f); 
  data[37].mBoundingSphere = Sphere(Vector3(-21.0953f, -0.98367f, 3.85015f), 0.864974f); 
  data[38].mBoundingSphere = Sphere(Vector3(0.295875f, -23.2894f, -2.33097f), 0.600446f); 
  data[39].mBoundingSphere = Sphere(Vector3(-2.41517f, -46.4778f, 12.2318f), 0.670492f); 
  data[40].mBoundingSphere = Sphere(Vector3(-15.4214f, 25.4093f, -6.80134f), 0.793304f); 
  data[41].mBoundingSphere = Sphere(Vector3(-31.217f, -4.9739f, 12.294f), 1.15977f); 
  data[42].mBoundingSphere = Sphere(Vector3(11.6021f, -32.8158f, 16.3092f), 0.459542f); 
  data[43].mBoundingSphere = Sphere(Vector3(-39.8255f, -21.6229f, 2.11414f), 1.30682f); 
  data[44].mBoundingSphere = Sphere(Vector3(28.0209f, -2.24267f, -39.5012f), 3.23506f); 
  data[45].mBoundingSphere = Sphere(Vector3(8.59973f, 2.75262f, 48.3893f), 3.3276f); 
  data[46].mBoundingSphere = Sphere(Vector3(-28.9072f, -11.3874f, -34.2429f), 3.65534f); 
  data[47].mBoundingSphere = Sphere(Vector3(-28.4112f, -26.091f, 9.53762f), 3.78835f); 
  data[48].mBoundingSphere = Sphere(Vector3(8.01055f, 31.6377f, -14.8529f), 0.351109f); 
  data[49].mBoundingSphere = Sphere(Vector3(-0.805364f, -21.7799f, 27.6817f), 2.2899f); 
  data[50].mBoundingSphere = Sphere(Vector3(25.1418f, -8.98801f, -0.649624f), 0.220179f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(44); 
  data[0].mBoundingSphere = Sphere(Vector3(-20.7831f, -27.668f, -20.6765f), 1.14156f); 
  data[1].mBoundingSphere = Sphere(Vector3(-8.76888f, -7.3873f, 0.43807f), 3.82848f); 
  data[2].mBoundingSphere = Sphere(Vector3(0.417869f, -18.9054f, 21.103f), 2.2899f); 
  data[3].mBoundingSphere = Sphere(Vector3(-29.6923f, 24.1032f, 3.07364f), 2.08591f); 
  data[4].mBoundingSphere = Sphere(Vector3(-8.46492f, 39.9178f, 16.5213f), 1.17635f); 
  data[5].mBoundingSphere = Sphere(Vector3(1.57019f, 39.4903f, -23.3753f), 0.463601f); 
  data[6].mBoundingSphere = Sphere(Vector3(-20.3992f, -16.263f, -28.7692f), 0.895706f); 
  data[7].mBoundingSphere = Sphere(Vector3(15.7331f, 41.1786f, 11.7435f), 2.44519f); 
  data[8].mBoundingSphere = Sphere(Vector3(35.3538f, -18.5894f, -24.5802f), 3.18658f); 
  data[9].mBoundingSphere = Sphere(Vector3(5.10703f, -42.6675f, 21.2762f), 2.38743f); 
  data[10].mBoundingSphere = Sphere(Vector3(25.9566f, 35.8337f, 17.7928f), 2.03419f); 
  data[11].mBoundingSphere = Sphere(Vector3(-34.9517f, 5.87177f, -4.04872f), 3.11062f); 
  data[12].mBoundingSphere = Sphere(Vector3(41.0155f, -13.3715f, -4.87181f), 2.44751f); 
  data[13].mBoundingSphere = Sphere(Vector3(-10.7542f, 14.3741f, -30.1695f), 1.66297f); 
  data[14].mBoundingSphere = Sphere(Vector3(11.9479f, -12.9642f, -33.2487f), 3.61162f); 
  data[15].mBoundingSphere = Sphere(Vector3(-7.37611f, -26.4997f, -9.47454f), 0.260768f); 
  data[16].mBoundingSphere = Sphere(Vector3(-38.0721f, -30.8694f, 6.69876f), 3.41505f); 
  data[17].mBoundingSphere = Sphere(Vector3(31.7552f, -15.9352f, -33.4664f), 0.603113f); 
  data[18].mBoundingSphere = Sphere(Vector3(-7.54443f, 4.04762f, -21.5565f), 1.55221f); 
  data[19].mBoundingSphere = Sphere(Vector3(-14.3896f, 5.43076f, 42.9536f), 0.561711f); 
  data[20].mBoundingSphere = Sphere(Vector3(-20.527f, -21.7442f, 13.796f), 1.20094f); 
  data[21].mBoundingSphere = Sphere(Vector3(16.4986f, -37.8987f, 19.9986f), 0.845723f); 
  data[22].mBoundingSphere = Sphere(Vector3(-3.18849f, 28.2197f, 22.2938f), 1.67433f); 
  data[23].mBoundingSphere = Sphere(Vector3(41.315f, -17.4256f, -5.81908f), 2.64152f); 
  data[24].mBoundingSphere = Sphere(Vector3(12.0309f, -15.408f, 33.6222f), 1.04832f); 
  data[25].mBoundingSphere = Sphere(Vector3(-9.02641f, 10.4972f, -0.510316f), 3.82778f); 
  data[26].mBoundingSphere = Sphere(Vector3(-6.97217f, 27.4371f, -39.1691f), 1.16406f); 
  data[27].mBoundingSphere = Sphere(Vector3(6.39157f, -41.3249f, -10.2947f), 1.36515f); 
  data[28].mBoundingSphere = Sphere(Vector3(22.8323f, -25.988f, -8.03217f), 3.60558f); 
  data[29].mBoundingSphere = Sphere(Vector3(-32.7575f, -11.6221f, -15.4106f), 3.66113f); 
  data[30].mBoundingSphere = Sphere(Vector3(-18.2342f, 10.3577f, -11.4617f), 0.451192f); 
  data[31].mBoundingSphere = Sphere(Vector3(-25.1954f, 0.415519f, -26.8746f), 3.50979f); 
  data[32].mBoundingSphere = Sphere(Vector3(-29.4415f, -20.5758f, -4.54009f), 1.60208f); 
  data[33].mBoundingSphere = Sphere(Vector3(-10.1875f, -12.6152f, -8.97727f), 2.80701f); 
  data[34].mBoundingSphere = Sphere(Vector3(-2.15904f, 46.7348f, -4.97522f), 3.68363f); 
  data[35].mBoundingSphere = Sphere(Vector3(1.69086f, 11.2609f, 10.3807f), 1.63989f); 
  data[36].mBoundingSphere = Sphere(Vector3(23.2434f, -3.11835f, 16.2766f), 3.12326f); 
  data[37].mBoundingSphere = Sphere(Vector3(4.61506f, -34.2623f, 29.5668f), 1.15165f); 
  data[38].mBoundingSphere = Sphere(Vector3(-10.6721f, -43.7867f, 19.4454f), 0.202667f); 
  data[39].mBoundingSphere = Sphere(Vector3(-6.98337f, 29.3109f, 3.24353f), 1.18169f); 
  data[40].mBoundingSphere = Sphere(Vector3(-30.5262f, -9.33709f, 6.16672f), 1.68535f); 
  data[41].mBoundingSphere = Sphere(Vector3(-16.0548f, -12.5447f, 44.6584f), 0.280483f); 
  data[42].mBoundingSphere = Sphere(Vector3(-20.9322f, -17.1525f, -41.5921f), 3.01831f); 
  data[43].mBoundingSphere = Sphere(Vector3(25.3246f, -41.9894f, -8.93664f), 1.89908f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(59); 
  data[0].mBoundingSphere = Sphere(Vector3(-17.6507f, -26.7832f, 26.507f), 1.04647f); 
  data[1].mBoundingSphere = Sphere(Vector3(0.533831f, 8.36776f, 37.392f), 3.29316f); 
  data[2].mBoundingSphere = Sphere(Vector3(5.32214f, -25.373f, 39.0891f), 2.91336f); 
  data[3].mBoundingSphere = Sphere(Vector3(5.34033f, -49.4296f, 1.76269f), 1.42755f); 
  data[4].mBoundingSphere = Sphere(Vector3(10.0903f, 17.1288f, -39.7279f), 3.80239f); 
  data[5].mBoundingSphere = Sphere(Vector3(-33.8963f, 10.0206f, 32.3029f), 1.68558f); 
  data[6].mBoundingSphere = Sphere(Vector3(30.227f, 26.2932f, 27.2853f), 0.49584f); 
  data[7].mBoundingSphere = Sphere(Vector3(-29.7167f, -3.45723f, 22.7302f), 0.238386f); 
  data[8].mBoundingSphere = Sphere(Vector3(-27.577f, -27.9617f, 11.0878f), 1.41247f); 
  data[9].mBoundingSphere = Sphere(Vector3(-13.4385f, 13.5063f, -29.868f), 2.88228f); 
  data[10].mBoundingSphere = Sphere(Vector3(13.5729f, 32.7268f, -27.2636f), 2.27784f); 
  data[11].mBoundingSphere = Sphere(Vector3(-18.5055f, 0.223568f, 0.684441f), 3.79636f); 
  data[12].mBoundingSphere = Sphere(Vector3(-9.37229f, -27.469f, 19.1307f), 2.17092f); 
  data[13].mBoundingSphere = Sphere(Vector3(-30.4494f, 18.4663f, -31.6645f), 0.877035f); 
  data[14].mBoundingSphere = Sphere(Vector3(-9.17861f, -1.25381f, 5.21631f), 2.94513f); 
  data[15].mBoundingSphere = Sphere(Vector3(9.061f, -24.3578f, 32.9414f), 2.0466f); 
  data[16].mBoundingSphere = Sphere(Vector3(-9.23612f, -15.5659f, -1.10182f), 3.42444f); 
  data[17].mBoundingSphere = Sphere(Vector3(1.52551f, -15.5418f, 4.33637f), 3.53577f); 
  data[18].mBoundingSphere = Sphere(Vector3(25.2972f, 12.731f, 31.7855f), 3.57427f); 
  data[19].mBoundingSphere = Sphere(Vector3(25.7406f, 31.4343f, -19.2148f), 0.94337f); 
  data[20].mBoundingSphere = Sphere(Vector3(9.25773f, 3.95075f, 2.80253f), 3.44763f); 
  data[21].mBoundingSphere = Sphere(Vector3(35.6751f, 8.0509f, -27.0974f), 2.66994f); 
  data[22].mBoundingSphere = Sphere(Vector3(0.753456f, -0.830873f, 16.0216f), 0.706442f); 
  data[23].mBoundingSphere = Sphere(Vector3(-36.3239f, 20.707f, -16.0317f), 3.92532f); 
  data[24].mBoundingSphere = Sphere(Vector3(-12.4158f, 31.5578f, -22.4027f), 1.91764f); 
  data[25].mBoundingSphere = Sphere(Vector3(15.9083f, -19.3892f, 39.3462f), 1.11303f); 
  data[26].mBoundingSphere = Sphere(Vector3(-13.5672f, -11.5637f, 5.39971f), 2.02375f); 
  data[27].mBoundingSphere = Sphere(Vector3(-1.40605f, 1.02539f, -16.0893f), 0.989178f); 
  data[28].mBoundingSphere = Sphere(Vector3(-16.15f, 36.1749f, -11.3396f), 0.389496f); 
  data[29].mBoundingSphere = Sphere(Vector3(-35.4394f, -7.38218f, -17.0486f), 1.79517f); 
  data[30].mBoundingSphere = Sphere(Vector3(-18.5787f, 31.1476f, 4.6454f), 0.222962f); 
  data[31].mBoundingSphere = Sphere(Vector3(-15.3191f, 32.0736f, 34.439f), 3.3668f); 
  data[32].mBoundingSphere = Sphere(Vector3(21.4905f, 22.4049f, -2.43198f), 2.77118f); 
  data[33].mBoundingSphere = Sphere(Vector3(-21.5304f, -24.0142f, -8.96693f), 0.617609f); 
  data[34].mBoundingSphere = Sphere(Vector3(23.6989f, 0.356755f, 24.5661f), 3.89145f); 
  data[35].mBoundingSphere = Sphere(Vector3(-5.29328f, -0.344571f, -32.191f), 2.19805f); 
  data[36].mBoundingSphere = Sphere(Vector3(-39.1954f, -9.73841f, -12.7962f), 0.597778f); 
  data[37].mBoundingSphere = Sphere(Vector3(-11.5091f, -15.5753f, 30.9489f), 2.12824f); 
  data[38].mBoundingSphere = Sphere(Vector3(13.0576f, 16.3448f, -24.8407f), 3.78163f); 
  data[39].mBoundingSphere = Sphere(Vector3(44.7878f, 5.11053f, -10.2112f), 3.20723f); 
  data[40].mBoundingSphere = Sphere(Vector3(-5.15287f, -26.7133f, -5.76533f), 3.9078f); 
  data[41].mBoundingSphere = Sphere(Vector3(-29.6504f, -20.8575f, 3.45521f), 0.996368f); 
  data[42].mBoundingSphere = Sphere(Vector3(-39.1394f, 5.95178f, 1.90755f), 3.68607f); 
  data[43].mBoundingSphere = Sphere(Vector3(0.925775f, 14.2112f, 10.2351f), 0.577599f); 
  data[44].mBoundingSphere = Sphere(Vector3(22.1123f, -32.7342f, 11.4295f), 3.71692f); 
  data[45].mBoundingSphere = Sphere(Vector3(-22.2594f, -34.1601f, 26.3492f), 1.48785f); 
  data[46].mBoundingSphere = Sphere(Vector3(-5.87854f, -1.35093f, 42.979f), 1.91277f); 
  data[47].mBoundingSphere = Sphere(Vector3(-12.5492f, -3.34634f, -46.5792f), 0.226441f); 
  data[48].mBoundingSphere = Sphere(Vector3(-3.28295f, -14.2014f, 19.966f), 2.89399f); 
  data[49].mBoundingSphere = Sphere(Vector3(6.43692f, -24.2093f, -18.5606f), 2.7401f); 
  data[50].mBoundingSphere = Sphere(Vector3(-10.0413f, 33.7579f, -15.3686f), 3.16003f); 
  data[51].mBoundingSphere = Sphere(Vector3(4.13254f, -3.61426f, -28.5382f), 1.27992f); 
  data[52].mBoundingSphere = Sphere(Vector3(-15.1669f, 14.2479f, 36.3197f), 1.43543f); 
  data[53].mBoundingSphere = Sphere(Vector3(-30.9573f, 34.3691f, 13.9466f), 3.03675f); 
  data[54].mBoundingSphere = Sphere(Vector3(6.97011f, -24.6473f, -20.5992f), 3.79114f); 
  data[55].mBoundingSphere = Sphere(Vector3(-12.3698f, 3.60271f, 31.7099f), 0.348094f); 
  data[56].mBoundingSphere = Sphere(Vector3(6.06173f, -43.1257f, 1.12876f), 0.74019f); 
  data[57].mBoundingSphere = Sphere(Vector3(6.49584f, -27.772f, -18.9256f), 0.283151f); 
  data[58].mBoundingSphere = Sphere(Vector3(1.98334f, -46.9332f, 0.594994f), 3.70961f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(68); 
  data[0].mBoundingSphere = Sphere(Vector3(17.3076f, 28.7092f, 31.4705f), 2.44646f); 
  data[1].mBoundingSphere = Sphere(Vector3(12.9982f, -4.87007f, -44.7332f), 2.15166f); 
  data[2].mBoundingSphere = Sphere(Vector3(4.6693f, 14.0051f, 39.7934f), 1.48785f); 
  data[3].mBoundingSphere = Sphere(Vector3(18.1767f, -34.6536f, 22.1064f), 2.51419f); 
  data[4].mBoundingSphere = Sphere(Vector3(-14.9257f, -20.1343f, 24.9194f), 1.75609f); 
  data[5].mBoundingSphere = Sphere(Vector3(13.5456f, -17.4234f, -2.33049f), 2.70867f); 
  data[6].mBoundingSphere = Sphere(Vector3(-22.1853f, -7.1645f, -16.4603f), 0.647646f); 
  data[7].mBoundingSphere = Sphere(Vector3(41.4061f, 10.216f, 11.0077f), 0.668984f); 
  data[8].mBoundingSphere = Sphere(Vector3(14.9609f, 3.08501f, 28.763f), 2.57484f); 
  data[9].mBoundingSphere = Sphere(Vector3(-29.5057f, 18.205f, 10.3378f), 2.1243f); 
  data[10].mBoundingSphere = Sphere(Vector3(-6.8904f, -29.7027f, 9.62012f), 3.06853f); 
  data[11].mBoundingSphere = Sphere(Vector3(23.794f, -1.12094f, -36.8079f), 2.08869f); 
  data[12].mBoundingSphere = Sphere(Vector3(7.31243f, 35.9232f, 30.3584f), 1.85513f); 
  data[13].mBoundingSphere = Sphere(Vector3(21.2048f, -25.5236f, 14.7705f), 2.92658f); 
  data[14].mBoundingSphere = Sphere(Vector3(18.1722f, -37.9627f, 7.30442f), 3.10912f); 
  data[15].mBoundingSphere = Sphere(Vector3(-12.2924f, 13.6682f, -33.0667f), 0.591864f); 
  data[16].mBoundingSphere = Sphere(Vector3(6.34402f, 32.7701f, -31.504f), 2.88031f); 
  data[17].mBoundingSphere = Sphere(Vector3(-40.5959f, 9.12457f, 20.1314f), 3.64351f); 
  data[18].mBoundingSphere = Sphere(Vector3(-19.5155f, 9.81422f, 20.5449f), 1.69625f); 
  data[19].mBoundingSphere = Sphere(Vector3(10.6946f, 19.003f, -33.4723f), 1.43427f); 
  data[20].mBoundingSphere = Sphere(Vector3(9.03134f, -37.8263f, 18.5953f), 0.539561f); 
  data[21].mBoundingSphere = Sphere(Vector3(20.0958f, 22.0839f, 38.273f), 3.07722f); 
  data[22].mBoundingSphere = Sphere(Vector3(18.8426f, 20.3199f, 36.3682f), 3.00034f); 
  data[23].mBoundingSphere = Sphere(Vector3(-28.9205f, 34.574f, -16.9597f), 2.78382f); 
  data[24].mBoundingSphere = Sphere(Vector3(43.1961f, 9.09739f, -2.77396f), 0.803162f); 
  data[25].mBoundingSphere = Sphere(Vector3(31.0723f, 7.58431f, 12.73f), 0.779852f); 
  data[26].mBoundingSphere = Sphere(Vector3(0.824103f, 3.51118f, -36.8285f), 3.51003f); 
  data[27].mBoundingSphere = Sphere(Vector3(-35.0704f, 2.92535f, 24.7567f), 3.79531f); 
  data[28].mBoundingSphere = Sphere(Vector3(30.2838f, 10.859f, 16.3689f), 2.19689f); 
  data[29].mBoundingSphere = Sphere(Vector3(0.140957f, 22.1077f, 31.9421f), 1.32816f); 
  data[30].mBoundingSphere = Sphere(Vector3(0.323742f, 31.1196f, -12.8976f), 3.90711f); 
  data[31].mBoundingSphere = Sphere(Vector3(-10.5057f, 37.0004f, -7.84322f), 1.28583f); 
  data[32].mBoundingSphere = Sphere(Vector3(-40.6022f, -16.6546f, -23.5016f), 3.87765f); 
  data[33].mBoundingSphere = Sphere(Vector3(-12.2494f, 16.8663f, 25.6963f), 3.39846f); 
  data[34].mBoundingSphere = Sphere(Vector3(-0.48497f, -5.31256f, 32.7596f), 1.94211f); 
  data[35].mBoundingSphere = Sphere(Vector3(-17.838f, 21.35f, -27.9542f), 1.60962f); 
  data[36].mBoundingSphere = Sphere(Vector3(-32.8381f, -0.0440757f, -3.82492f), 2.30799f); 
  data[37].mBoundingSphere = Sphere(Vector3(43.0358f, 19.7475f, 4.86204f), 0.241169f); 
  data[38].mBoundingSphere = Sphere(Vector3(-6.3804f, -37.4192f, -8.98698f), 0.508713f); 
  data[39].mBoundingSphere = Sphere(Vector3(-6.37141f, -33.4789f, -26.8946f), 2.13288f); 
  data[40].mBoundingSphere = Sphere(Vector3(-10.9863f, -22.1335f, -8.57728f), 3.13161f); 
  data[41].mBoundingSphere = Sphere(Vector3(9.521f, -6.95598f, 1.86648f), 2.95719f); 
  data[42].mBoundingSphere = Sphere(Vector3(-24.3943f, 10.8809f, -39.5724f), 0.519034f); 
  data[43].mBoundingSphere = Sphere(Vector3(-6.60698f, 0.618787f, -40.0837f), 3.10912f); 
  data[44].mBoundingSphere = Sphere(Vector3(-13.2403f, 11.7262f, 18.1682f), 2.64767f); 
  data[45].mBoundingSphere = Sphere(Vector3(-5.87949f, -48.686f, 1.71768f), 2.97134f); 
  data[46].mBoundingSphere = Sphere(Vector3(7.08943f, -6.11684f, -12.5477f), 1.37861f); 
  data[47].mBoundingSphere = Sphere(Vector3(-47.4174f, 7.75107f, -0.0659851f), 0.55371f); 
  data[48].mBoundingSphere = Sphere(Vector3(17.6891f, 3.23218f, 21.6092f), 3.4866f); 
  data[49].mBoundingSphere = Sphere(Vector3(-26.5361f, 5.48513f, 39.1233f), 2.90837f); 
  data[50].mBoundingSphere = Sphere(Vector3(-17.8468f, -3.20267f, 39.6808f), 1.46106f); 
  data[51].mBoundingSphere = Sphere(Vector3(11.1283f, -18.6528f, -24.0353f), 1.91926f); 
  data[52].mBoundingSphere = Sphere(Vector3(0.0276968f, -27.513f, -6.94253f), 3.68433f); 
  data[53].mBoundingSphere = Sphere(Vector3(-22.8558f, -34.6959f, -6.73473f), 1.86754f); 
  data[54].mBoundingSphere = Sphere(Vector3(14.3059f, -19.2944f, 21.2383f), 2.38349f); 
  data[55].mBoundingSphere = Sphere(Vector3(-7.94229f, -15.068f, -40.8843f), 3.77073f); 
  data[56].mBoundingSphere = Sphere(Vector3(40.8413f, 13.5926f, 1.25242f), 0.957982f); 
  data[57].mBoundingSphere = Sphere(Vector3(10.0414f, 8.91546f, 27.6109f), 3.49785f); 
  data[58].mBoundingSphere = Sphere(Vector3(41.4087f, 6.2603f, 16.4933f), 1.88157f); 
  data[59].mBoundingSphere = Sphere(Vector3(-11.2763f, -20.3046f, -19.6707f), 3.65951f); 
  data[60].mBoundingSphere = Sphere(Vector3(-22.003f, 12.0284f, 19.7292f), 0.780316f); 
  data[61].mBoundingSphere = Sphere(Vector3(-35.7465f, -11.6576f, 24.6887f), 1.27261f); 
  data[62].mBoundingSphere = Sphere(Vector3(-47.9682f, -3.46447f, -13.3906f), 1.88424f); 
  data[63].mBoundingSphere = Sphere(Vector3(-9.55193f, -41.1043f, -6.39164f), 0.599866f); 
  data[64].mBoundingSphere = Sphere(Vector3(34.808f, -9.43565f, -22.1149f), 1.65508f); 
  data[65].mBoundingSphere = Sphere(Vector3(14.9876f, 35.6731f, 13.3388f), 1.76514f); 
  data[66].mBoundingSphere = Sphere(Vector3(-10.4696f, -35.572f, 22.5235f), 1.38846f); 
  data[67].mBoundingSphere = Sphere(Vector3(-43.6877f, -6.7892f, 21.7591f), 3.14275f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mBoundingSphere = Sphere(Vector3(6.56411f, -31.8098f, 22.8017f), 0.211017f); 
  data[1].mBoundingSphere = Sphere(Vector3(42.1225f, 22.6353f, 5.6503f), 3.60118f); 
  data[2].mBoundingSphere = Sphere(Vector3(-0.489213f, -12.0418f, -12.4454f), 0.895242f); 
  data[3].mBoundingSphere = Sphere(Vector3(13.67f, 33.8939f, 27.838f), 2.00427f); 
  data[4].mBoundingSphere = Sphere(Vector3(3.8792f, -28.9251f, 11.1965f), 3.7197f); 
  data[5].mBoundingSphere = Sphere(Vector3(39.284f, 13.9249f, 20.6536f), 0.498276f); 
  data[6].mBoundingSphere = Sphere(Vector3(-40.1796f, -15.2874f, -0.384426f), 1.1026f); 
  data[7].mBoundingSphere = Sphere(Vector3(26.8027f, -19.6292f, 30.1056f), 2.76561f); 
  data[8].mBoundingSphere = Sphere(Vector3(-2.11505f, -20.2491f, 29.7563f), 1.84863f); 
  data[9].mBoundingSphere = Sphere(Vector3(-35.1095f, 26.5021f, 21.9704f), 0.262856f); 
  data[10].mBoundingSphere = Sphere(Vector3(-24.0719f, -41.5434f, 4.68326f), 3.01947f); 
  data[11].mBoundingSphere = Sphere(Vector3(-45.0416f, -8.73535f, -9.14478f), 0.380682f); 
  data[12].mBoundingSphere = Sphere(Vector3(-6.1161f, 5.09858f, -43.3975f), 2.17973f); 
  data[13].mBoundingSphere = Sphere(Vector3(-39.23f, -22.1647f, -16.1092f), 3.77629f); 
  data[14].mBoundingSphere = Sphere(Vector3(-12.8916f, -17.7219f, -16.4346f), 0.619001f); 
  data[15].mBoundingSphere = Sphere(Vector3(0.399399f, 14.2382f, -32.5659f), 3.08244f); 
  data[16].mBoundingSphere = Sphere(Vector3(34.2354f, 20.6413f, -19.9085f), 1.34706f); 
  data[17].mBoundingSphere = Sphere(Vector3(29.8447f, -33.23f, -10.2675f), 2.99141f); 
  data[18].mBoundingSphere = Sphere(Vector3(-37.074f, -6.71913f, -27.7179f), 1.86522f); 
  data[19].mBoundingSphere = Sphere(Vector3(13.6352f, -39.0903f, -5.95995f), 0.912638f); 
  data[20].mBoundingSphere = Sphere(Vector3(-13.968f, 7.26434f, 31.303f), 0.712705f); 
  data[21].mBoundingSphere = Sphere(Vector3(9.78712f, -5.61292f, 19.1222f), 0.407587f); 
  data[22].mBoundingSphere = Sphere(Vector3(-33.5027f, 28.8225f, -20.246f), 2.58725f); 
  data[23].mBoundingSphere = Sphere(Vector3(13.8422f, 31.3357f, -0.228962f), 0.39715f); 
  data[24].mBoundingSphere = Sphere(Vector3(25.0497f, -24.2799f, -16.5968f), 3.04151f); 
  data[25].mBoundingSphere = Sphere(Vector3(-23.9491f, 10.6878f, -10.1029f), 1.94605f); 
  data[26].mBoundingSphere = Sphere(Vector3(42.9249f, -0.234588f, -18.7414f), 2.15422f); 
  data[27].mBoundingSphere = Sphere(Vector3(9.87624f, 40.5721f, 5.4138f), 2.22078f); 
  data[28].mBoundingSphere = Sphere(Vector3(-32.715f, 16.3972f, 18.6601f), 1.57587f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(-11.6476f, 30.164f, 2.11529f), 1.31134f); 
  data[1].mBoundingSphere = Sphere(Vector3(1.4927f, -11.0498f, 47.9365f), 1.94802f); 
  data[2].mBoundingSphere = Sphere(Vector3(25.4996f, -34.724f, -9.22234f), 3.99153f); 
  data[3].mBoundingSphere = Sphere(Vector3(6.80252f, -33.0934f, 27.9508f), 1.88841f); 
  data[4].mBoundingSphere = Sphere(Vector3(0.972098f, 3.61971f, -20.7456f), 2.81107f); 
  data[5].mBoundingSphere = Sphere(Vector3(11.4824f, 17.7289f, -24.9036f), 2.65869f); 
  data[6].mBoundingSphere = Sphere(Vector3(30.392f, -12.3611f, -37.5504f), 1.10631f); 
  data[7].mBoundingSphere = Sphere(Vector3(-31.151f, -17.608f, 15.8679f), 2.61798f); 
  data[8].mBoundingSphere = Sphere(Vector3(29.174f, 4.46213f, 37.9857f), 1.64441f); 
  data[9].mBoundingSphere = Sphere(Vector3(-10.485f, -15.0351f, -8.56476f), 1.52322f); 
  data[10].mBoundingSphere = Sphere(Vector3(-13.0218f, 10.4717f, -42.1945f), 0.700644f); 
  data[11].mBoundingSphere = Sphere(Vector3(40.967f, -6.75717f, -2.65756f), 3.39371f); 
  data[12].mBoundingSphere = Sphere(Vector3(-42.3965f, -16.3453f, 9.55804f), 3.51095f); 
  data[13].mBoundingSphere = Sphere(Vector3(-13.0101f, 16.8644f, -8.93964f), 2.75112f); 
  data[14].mBoundingSphere = Sphere(Vector3(-29.2843f, 21.8942f, -6.53029f), 0.298575f); 
  data[15].mBoundingSphere = Sphere(Vector3(17.7643f, 22.8951f, 3.69183f), 3.63411f); 
  data[16].mBoundingSphere = Sphere(Vector3(8.76321f, 2.10162f, -15.5691f), 1.67793f); 
  data[17].mBoundingSphere = Sphere(Vector3(-17.438f, -46.1846f, 3.99803f), 2.98851f); 
  data[18].mBoundingSphere = Sphere(Vector3(25.8046f, -24.9636f, 9.18731f), 0.429969f); 
  data[19].mBoundingSphere = Sphere(Vector3(26.3324f, -13.6531f, 3.31092f), 3.032f); 
  data[20].mBoundingSphere = Sphere(Vector3(-2.60305f, -28.6972f, 32.3709f), 0.256362f); 
  data[21].mBoundingSphere = Sphere(Vector3(16.4224f, 5.53564f, -41.376f), 3.19389f); 
  data[22].mBoundingSphere = Sphere(Vector3(29.7834f, 26.3419f, -22.5543f), 1.30555f); 
  data[23].mBoundingSphere = Sphere(Vector3(-32.8436f, 21.6378f, -21.9402f), 2.03013f); 
  data[24].mBoundingSphere = Sphere(Vector3(-14.0243f, -10.4141f, -4.23183f), 3.27565f); 
  data[25].mBoundingSphere = Sphere(Vector3(-28.985f, 21.9402f, 12.0541f), 1.5129f); 
  data[26].mBoundingSphere = Sphere(Vector3(-27.2691f, 25.4012f, -25.1168f), 3.29177f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(71); 
  data[0].mBoundingSphere = Sphere(Vector3(7.07211f, 20.6303f, 2.25967f), 3.44902f); 
  data[1].mBoundingSphere = Sphere(Vector3(-16.6163f, 6.16938f, 31.6956f), 0.895822f); 
  data[2].mBoundingSphere = Sphere(Vector3(25.2249f, -22.5179f, -14.4509f), 3.65464f); 
  data[3].mBoundingSphere = Sphere(Vector3(-0.049398f, 11.845f, 14.8693f), 0.204871f); 
  data[4].mBoundingSphere = Sphere(Vector3(4.0198f, 27.9756f, 26.36f), 2.39752f); 
  data[5].mBoundingSphere = Sphere(Vector3(9.23842f, -24.0691f, 18.1852f), 1.94756f); 
  data[6].mBoundingSphere = Sphere(Vector3(2.92828f, -9.10191f, -16.3535f), 1.30241f); 
  data[7].mBoundingSphere = Sphere(Vector3(11.343f, -12.5326f, -26.8689f), 0.253346f); 
  data[8].mBoundingSphere = Sphere(Vector3(33.5358f, 9.15295f, 27.2367f), 3.95906f); 
  data[9].mBoundingSphere = Sphere(Vector3(20.1096f, 12.3783f, -29.2951f), 2.14969f); 
  data[10].mBoundingSphere = Sphere(Vector3(-21.8484f, -23.2777f, 12.4282f), 0.381262f); 
  data[11].mBoundingSphere = Sphere(Vector3(-35.8744f, 7.42975f, -12.3499f), 3.70787f); 
  data[12].mBoundingSphere = Sphere(Vector3(26.7089f, -7.32271f, 6.19328f), 3.24063f); 
  data[13].mBoundingSphere = Sphere(Vector3(23.9285f, -19.8273f, -34.5268f), 2.76399f); 
  data[14].mBoundingSphere = Sphere(Vector3(0.475822f, -20.805f, 40.0302f), 2.87486f); 
  data[15].mBoundingSphere = Sphere(Vector3(6.48702f, -19.3705f, -15.9672f), 2.61578f); 
  data[16].mBoundingSphere = Sphere(Vector3(16.1028f, -34.1427f, -25.3734f), 0.671072f); 
  data[17].mBoundingSphere = Sphere(Vector3(19.4361f, 1.47308f, 7.0029f), 2.89272f); 
  data[18].mBoundingSphere = Sphere(Vector3(31.7935f, -24.9555f, 3.31563f), 3.02562f); 
  data[19].mBoundingSphere = Sphere(Vector3(-27.2002f, -25.7492f, 29.4073f), 0.4026f); 
  data[20].mBoundingSphere = Sphere(Vector3(-19.7234f, -22.813f, 4.22381f), 2.4758f); 
  data[21].mBoundingSphere = Sphere(Vector3(29.2287f, 12.9332f, -14.8663f), 3.75298f); 
  data[22].mBoundingSphere = Sphere(Vector3(-0.639446f, -24.742f, -21.8881f), 3.01019f); 
  data[23].mBoundingSphere = Sphere(Vector3(-11.8524f, 33.8019f, -33.6213f), 0.946385f); 
  data[24].mBoundingSphere = Sphere(Vector3(6.60285f, 18.6758f, 6.28734f), 2.1512f); 
  data[25].mBoundingSphere = Sphere(Vector3(36.16f, 27.7055f, -9.43116f), 1.55233f); 
  data[26].mBoundingSphere = Sphere(Vector3(-31.0731f, -22.3519f, 12.7169f), 2.01412f); 
  data[27].mBoundingSphere = Sphere(Vector3(-13.2735f, 1.59068f, -15.3547f), 1.45283f); 
  data[28].mBoundingSphere = Sphere(Vector3(-4.80408f, -13.5015f, 4.88854f), 3.74057f); 
  data[29].mBoundingSphere = Sphere(Vector3(1.75526f, 31.614f, -7.4166f), 3.16374f); 
  data[30].mBoundingSphere = Sphere(Vector3(20.3747f, -4.54877f, -28.3069f), 0.506974f); 
  data[31].mBoundingSphere = Sphere(Vector3(-24.8006f, -39.1338f, 12.3269f), 0.530515f); 
  data[32].mBoundingSphere = Sphere(Vector3(-19.2308f, 21.4659f, -34.413f), 3.3087f); 
  data[33].mBoundingSphere = Sphere(Vector3(-8.6805f, 41.2461f, 23.4549f), 3.80007f); 
  data[34].mBoundingSphere = Sphere(Vector3(2.37447f, 7.28931f, 18.9657f), 0.71746f); 
  data[35].mBoundingSphere = Sphere(Vector3(21.8019f, -29.8204f, 23.133f), 2.46571f); 
  data[36].mBoundingSphere = Sphere(Vector3(7.50587f, 5.21891f, 32.446f), 0.816382f); 
  data[37].mBoundingSphere = Sphere(Vector3(-20.716f, 28.767f, -27.9311f), 2.68942f); 
  data[38].mBoundingSphere = Sphere(Vector3(27.2466f, 29.4053f, -24.3933f), 3.30789f); 
  data[39].mBoundingSphere = Sphere(Vector3(10.2257f, 25.1037f, 40.3386f), 2.49726f); 
  data[40].mBoundingSphere = Sphere(Vector3(15.3389f, 5.95934f, 11.808f), 0.596735f); 
  data[41].mBoundingSphere = Sphere(Vector3(-29.305f, 0.112375f, 22.2511f), 3.55061f); 
  data[42].mBoundingSphere = Sphere(Vector3(-12.4952f, -27.2412f, 2.81203f), 0.249751f); 
  data[43].mBoundingSphere = Sphere(Vector3(-8.12928f, -3.35642f, 9.9797f), 2.49204f); 
  data[44].mBoundingSphere = Sphere(Vector3(-8.14137f, 14.1127f, 11.0735f), 0.428578f); 
  data[45].mBoundingSphere = Sphere(Vector3(-18.378f, 23.1362f, -32.3231f), 3.66705f); 
  data[46].mBoundingSphere = Sphere(Vector3(-42.7577f, 1.30402f, -23.0842f), 1.89862f); 
  data[47].mBoundingSphere = Sphere(Vector3(0.914803f, 11.6048f, 45.3251f), 3.81943f); 
  data[48].mBoundingSphere = Sphere(Vector3(-6.79202f, -1.43112f, -44.7028f), 3.41261f); 
  data[49].mBoundingSphere = Sphere(Vector3(-2.3243f, -24.2782f, -8.67504f), 1.96275f); 
  data[50].mBoundingSphere = Sphere(Vector3(-29.8734f, -12.0735f, 10.2498f), 3.26904f); 
  data[51].mBoundingSphere = Sphere(Vector3(-28.3028f, 3.37528f, -16.7008f), 0.226441f); 
  data[52].mBoundingSphere = Sphere(Vector3(-37.4911f, 17.6535f, -2.5506f), 3.24584f); 
  data[53].mBoundingSphere = Sphere(Vector3(-2.91531f, 42.194f, -19.4237f), 3.27704f); 
  data[54].mBoundingSphere = Sphere(Vector3(2.76402f, -4.37164f, 22.654f), 3.50921f); 
  data[55].mBoundingSphere = Sphere(Vector3(9.63226f, 23.2126f, -2.03532f), 1.11338f); 
  data[56].mBoundingSphere = Sphere(Vector3(-33.1407f, -17.8948f, -22.1396f), 3.69697f); 
  data[57].mBoundingSphere = Sphere(Vector3(39.2251f, 0.567911f, 22.1363f), 3.63922f); 
  data[58].mBoundingSphere = Sphere(Vector3(34.8614f, -30.0672f, 17.6171f), 2.57681f); 
  data[59].mBoundingSphere = Sphere(Vector3(-34.8438f, -0.547915f, -12.3317f), 1.60765f); 
  data[60].mBoundingSphere = Sphere(Vector3(33.6829f, -12.6864f, 8.16733f), 3.8809f); 
  data[61].mBoundingSphere = Sphere(Vector3(35.8114f, 22.8373f, -7.57759f), 2.14378f); 
  data[62].mBoundingSphere = Sphere(Vector3(-24.4285f, 3.29403f, -5.37175f), 1.71318f); 
  data[63].mBoundingSphere = Sphere(Vector3(-25.3976f, 8.33113f, 6.64463f), 2.03964f); 
  data[64].mBoundingSphere = Sphere(Vector3(-13.1434f, -1.47292f, -12.5124f), 2.53425f); 
  data[65].mBoundingSphere = Sphere(Vector3(-14.761f, 21.4195f, -32.2586f), 1.42592f); 
  data[66].mBoundingSphere = Sphere(Vector3(1.19018f, 15.3613f, -12.4987f), 3.91882f); 
  data[67].mBoundingSphere = Sphere(Vector3(-17.3786f, -6.07962f, -11.1708f), 0.817542f); 
  data[68].mBoundingSphere = Sphere(Vector3(18.5242f, -4.93007f, 31.2177f), 2.29002f); 
  data[69].mBoundingSphere = Sphere(Vector3(19.9967f, -13.6382f, 30.2143f), 0.535502f); 
  data[70].mBoundingSphere = Sphere(Vector3(-0.718205f, 16.8044f, 4.00665f), 3.02794f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mBoundingSphere = Sphere(Vector3(32.5867f, -35.6729f, 10.4661f), 2.23018f); 
  data[1].mBoundingSphere = Sphere(Vector3(-20.4304f, 38.0323f, 10.5941f), 0.779736f); 
  data[2].mBoundingSphere = Sphere(Vector3(-9.71203f, 29.0653f, 16.7936f), 3.55792f); 
  data[3].mBoundingSphere = Sphere(Vector3(-9.36902f, 23.1339f, 17.3394f), 2.98746f); 
  data[4].mBoundingSphere = Sphere(Vector3(9.32263f, 16.9664f, 11.691f), 1.3125f); 
  data[5].mBoundingSphere = Sphere(Vector3(10.3892f, -16.9965f, 37.0668f), 3.15817f); 
  data[6].mBoundingSphere = Sphere(Vector3(-18.0643f, -27.9442f, 1.12276f), 3.79357f); 
  data[7].mBoundingSphere = Sphere(Vector3(4.1303f, 4.77268f, -17.3514f), 1.71585f); 
  data[8].mBoundingSphere = Sphere(Vector3(22.8544f, -18.7606f, -3.9428f), 1.47533f); 
  data[9].mBoundingSphere = Sphere(Vector3(13.9555f, -3.13533f, -37.514f), 1.1578f); 
  data[10].mBoundingSphere = Sphere(Vector3(8.56053f, 9.68735f, -18.8076f), 2.64709f); 
  data[11].mBoundingSphere = Sphere(Vector3(-26.8467f, 26.9512f, -10.2527f), 1.08149f); 
  data[12].mBoundingSphere = Sphere(Vector3(15.3485f, 18.062f, -2.87459f), 3.02782f); 
  data[13].mBoundingSphere = Sphere(Vector3(-3.09557f, -2.52175f, 7.46413f), 3.18716f); 
  data[14].mBoundingSphere = Sphere(Vector3(-29.5848f, -16.5585f, 13.1039f), 3.35567f); 
  data[15].mBoundingSphere = Sphere(Vector3(27.4648f, 25.8156f, 25.9582f), 3.45494f); 
  data[16].mBoundingSphere = Sphere(Vector3(-13.7337f, -22.502f, 11.707f), 2.75355f); 
  data[17].mBoundingSphere = Sphere(Vector3(-7.35956f, -33.9036f, -10.167f), 1.89746f); 
  data[18].mBoundingSphere = Sphere(Vector3(8.42333f, -23.666f, -18.6567f), 2.06828f); 
  data[19].mBoundingSphere = Sphere(Vector3(20.9134f, -39.1384f, -3.27676f), 3.28736f); 
  data[20].mBoundingSphere = Sphere(Vector3(-24.2038f, 2.48238f, -5.26567f), 1.98583f); 
  data[21].mBoundingSphere = Sphere(Vector3(25.5046f, -16.9117f, 18.4987f), 1.25869f); 
  data[22].mBoundingSphere = Sphere(Vector3(16.3601f, 42.1636f, -12.4012f), 3.52116f); 
  data[23].mBoundingSphere = Sphere(Vector3(15.6969f, -6.14699f, 30.7519f), 2.78591f); 
  data[24].mBoundingSphere = Sphere(Vector3(-17.1346f, -1.26727f, -33.9885f), 3.28261f); 
  data[25].mBoundingSphere = Sphere(Vector3(36.3027f, 26.6187f, 0.834055f), 1.83425f); 
  data[26].mBoundingSphere = Sphere(Vector3(-18.6758f, 32.561f, 32.0139f), 2.51245f); 
  data[27].mBoundingSphere = Sphere(Vector3(-33.3288f, -2.22734f, -24.7808f), 0.988714f); 
  data[28].mBoundingSphere = Sphere(Vector3(11.6603f, 23.0624f, 12.3632f), 3.30128f); 
  data[29].mBoundingSphere = Sphere(Vector3(5.17821f, 6.32113f, -47.4155f), 1.25545f); 
  data[30].mBoundingSphere = Sphere(Vector3(-24.057f, -33.815f, -20.9078f), 3.37782f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(33.5559f, 6.33107f, -4.72272f), 0.327567f); 
  data[1].mBoundingSphere = Sphere(Vector3(29.5759f, 10.001f, -26.3738f), 3.23668f); 
  data[2].mBoundingSphere = Sphere(Vector3(45.171f, -17.2411f, 5.05461f), 2.49656f); 
  data[3].mBoundingSphere = Sphere(Vector3(19.7827f, -32.4339f, -13.4826f), 0.693454f); 
  data[4].mBoundingSphere = Sphere(Vector3(24.3399f, -21.1395f, 24.9785f), 1.73765f); 
  data[5].mBoundingSphere = Sphere(Vector3(19.9922f, -39.8255f, -21.2446f), 2.88495f); 
  data[6].mBoundingSphere = Sphere(Vector3(5.38026f, 1.14066f, -0.424739f), 0.21438f); 
  data[7].mBoundingSphere = Sphere(Vector3(46.6406f, -0.737906f, 12.9305f), 1.03162f); 
  data[8].mBoundingSphere = Sphere(Vector3(-29.3422f, -10.3977f, -11.5072f), 0.563567f); 
  data[9].mBoundingSphere = Sphere(Vector3(24.8466f, -33.1901f, -12.97f), 3.817f); 
  data[10].mBoundingSphere = Sphere(Vector3(-21.411f, 11.5984f, 20.1325f), 0.566582f); 
  data[11].mBoundingSphere = Sphere(Vector3(-1.46508f, 36.0628f, -33.7018f), 0.944645f); 
  data[12].mBoundingSphere = Sphere(Vector3(-16.1674f, 28.4651f, 10.5733f), 1.59605f); 
  data[13].mBoundingSphere = Sphere(Vector3(-33.4041f, 13.359f, 2.4906f), 1.27342f); 
  data[14].mBoundingSphere = Sphere(Vector3(2.5042f, -28.4877f, -25.9408f), 0.436116f); 
  data[15].mBoundingSphere = Sphere(Vector3(32.8749f, 25.6417f, -14.2402f), 1.12475f); 
  data[16].mBoundingSphere = Sphere(Vector3(13.0783f, 16.9965f, -13.1106f), 2.47893f); 
  data[17].mBoundingSphere = Sphere(Vector3(-9.51962f, -17.7377f, -34.5655f), 0.566814f); 
  data[18].mBoundingSphere = Sphere(Vector3(12.7891f, 20.2965f, 30.3463f), 3.0959f); 
  data[19].mBoundingSphere = Sphere(Vector3(-7.66647f, -8.31696f, 6.09385f), 3.07316f); 
  data[20].mBoundingSphere = Sphere(Vector3(19.136f, 35.7955f, -3.97417f), 1.78346f); 
  data[21].mBoundingSphere = Sphere(Vector3(42.0223f, 11.6686f, -10.7863f), 2.35589f); 
  data[22].mBoundingSphere = Sphere(Vector3(6.50318f, -43.9411f, -11.3785f), 3.48451f); 
  data[23].mBoundingSphere = Sphere(Vector3(18.9171f, 23.5864f, -25.2533f), 1.97493f); 
  data[24].mBoundingSphere = Sphere(Vector3(-33.1409f, 7.3422f, 14.7047f), 3.284f); 
  data[25].mBoundingSphere = Sphere(Vector3(29.9816f, 16.8372f, -23.0283f), 3.12094f); 
  data[26].mBoundingSphere = Sphere(Vector3(-16.0512f, 4.68497f, -23.6918f), 3.88055f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(38); 
  data[0].mBoundingSphere = Sphere(Vector3(-17.9757f, -27.4478f, 4.7874f), 2.31541f); 
  data[1].mBoundingSphere = Sphere(Vector3(-15.1949f, -11.9622f, -9.92967f), 3.20966f); 
  data[2].mBoundingSphere = Sphere(Vector3(30.5511f, -6.17163f, 23.3368f), 3.02655f); 
  data[3].mBoundingSphere = Sphere(Vector3(32.5524f, 27.0997f, 22.062f), 2.25917f); 
  data[4].mBoundingSphere = Sphere(Vector3(31.0759f, -3.73318f, 31.5856f), 3.58599f); 
  data[5].mBoundingSphere = Sphere(Vector3(24.7767f, 8.84679f, 35.9126f), 3.47315f); 
  data[6].mBoundingSphere = Sphere(Vector3(-17.6691f, 18.019f, 36.1227f), 1.68686f); 
  data[7].mBoundingSphere = Sphere(Vector3(-2.74072f, 4.33756f, 24.7537f), 2.23203f); 
  data[8].mBoundingSphere = Sphere(Vector3(-28.4171f, 16.5261f, 26.323f), 3.93865f); 
  data[9].mBoundingSphere = Sphere(Vector3(-24.9958f, 20.2352f, 3.71386f), 3.61208f); 
  data[10].mBoundingSphere = Sphere(Vector3(-27.3364f, 30.2092f, 10.45f), 2.36876f); 
  data[11].mBoundingSphere = Sphere(Vector3(17.0693f, 15.6181f, 28.6284f), 3.87417f); 
  data[12].mBoundingSphere = Sphere(Vector3(30.898f, -1.74671f, 20.7914f), 2.7009f); 
  data[13].mBoundingSphere = Sphere(Vector3(17.6463f, -1.16739f, 38.0546f), 0.929337f); 
  data[14].mBoundingSphere = Sphere(Vector3(4.64944f, 15.7803f, 17.8892f), 1.80596f); 
  data[15].mBoundingSphere = Sphere(Vector3(0.0756847f, 5.78292f, -40.318f), 3.37886f); 
  data[16].mBoundingSphere = Sphere(Vector3(12.1283f, -17.9395f, -43.9507f), 1.73104f); 
  data[17].mBoundingSphere = Sphere(Vector3(2.42845f, 30.882f, 23.8983f), 3.92821f); 
  data[18].mBoundingSphere = Sphere(Vector3(-18.0082f, -7.73611f, -27.6648f), 3.30035f); 
  data[19].mBoundingSphere = Sphere(Vector3(19.7873f, -23.2314f, -1.77153f), 0.777184f); 
  data[20].mBoundingSphere = Sphere(Vector3(7.30257f, -9.74698f, -47.3246f), 2.09345f); 
  data[21].mBoundingSphere = Sphere(Vector3(8.0202f, 14.1492f, 32.2282f), 2.38558f); 
  data[22].mBoundingSphere = Sphere(Vector3(7.40405f, 14.6511f, -3.44452f), 1.99719f); 
  data[23].mBoundingSphere = Sphere(Vector3(23.9325f, 7.58962f, -33.1926f), 0.411762f); 
  data[24].mBoundingSphere = Sphere(Vector3(-0.237271f, 40.2389f, -20.8646f), 0.733116f); 
  data[25].mBoundingSphere = Sphere(Vector3(43.2083f, 23.9067f, -4.61192f), 2.02874f); 
  data[26].mBoundingSphere = Sphere(Vector3(-36.048f, 14.5288f, 13.163f), 0.559972f); 
  data[27].mBoundingSphere = Sphere(Vector3(-45.1376f, 6.97907f, 1.53556f), 3.81294f); 
  data[28].mBoundingSphere = Sphere(Vector3(-3.59277f, 42.4089f, 24.5475f), 2.35102f); 
  data[29].mBoundingSphere = Sphere(Vector3(19.6854f, 34.4053f, 25.8503f), 2.2093f); 
  data[30].mBoundingSphere = Sphere(Vector3(23.6665f, 8.62988f, 8.39798f), 2.04602f); 
  data[31].mBoundingSphere = Sphere(Vector3(38.4356f, 3.71501f, -19.9481f), 2.57217f); 
  data[32].mBoundingSphere = Sphere(Vector3(-5.60989f, -34.3704f, 35.0282f), 0.87889f); 
  data[33].mBoundingSphere = Sphere(Vector3(9.31767f, 11.0377f, 13.8233f), 3.97866f); 
  data[34].mBoundingSphere = Sphere(Vector3(-21.3867f, -29.3528f, -28.2832f), 2.78034f); 
  data[35].mBoundingSphere = Sphere(Vector3(-8.68398f, 9.11102f, 35.8513f), 1.61901f); 
  data[36].mBoundingSphere = Sphere(Vector3(-12.3958f, -41.732f, 6.24524f), 2.75459f); 
  data[37].mBoundingSphere = Sphere(Vector3(22.4476f, -32.4866f, -25.2314f), 2.9623f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(38); 
  data[0].mBoundingSphere = Sphere(Vector3(-2.69486f, -22.9054f, -42.3021f), 1.76386f); 
  data[1].mBoundingSphere = Sphere(Vector3(-17.0537f, -9.90016f, -21.67f), 2.47557f); 
  data[2].mBoundingSphere = Sphere(Vector3(-27.7331f, -14.4299f, -29.487f), 1.08219f); 
  data[3].mBoundingSphere = Sphere(Vector3(-33.0039f, 2.53005f, -16.9044f), 3.84437f); 
  data[4].mBoundingSphere = Sphere(Vector3(4.74531f, 25.3714f, -0.166212f), 2.81304f); 
  data[5].mBoundingSphere = Sphere(Vector3(-32.4979f, 30.5758f, 7.81233f), 2.00763f); 
  data[6].mBoundingSphere = Sphere(Vector3(-25.6231f, -35.885f, -21.1315f), 0.286166f); 
  data[7].mBoundingSphere = Sphere(Vector3(0.777061f, -0.697698f, 47.2519f), 0.378942f); 
  data[8].mBoundingSphere = Sphere(Vector3(-2.87856f, 14.1067f, -1.30726f), 1.45144f); 
  data[9].mBoundingSphere = Sphere(Vector3(-25.7312f, -24.4054f, -13.8649f), 2.86801f); 
  data[10].mBoundingSphere = Sphere(Vector3(29.8457f, -28.8288f, -11.1443f), 3.12384f); 
  data[11].mBoundingSphere = Sphere(Vector3(11.3181f, 34.8587f, -24.2113f), 2.61357f); 
  data[12].mBoundingSphere = Sphere(Vector3(-13.2935f, -10.4365f, -18.3584f), 1.80573f); 
  data[13].mBoundingSphere = Sphere(Vector3(-11.2327f, 20.6626f, 34.1048f), 3.49263f); 
  data[14].mBoundingSphere = Sphere(Vector3(11.6966f, 48.5356f, 2.43003f), 3.08233f); 
  data[15].mBoundingSphere = Sphere(Vector3(37.1252f, 13.2559f, -13.8551f), 2.20397f); 
  data[16].mBoundingSphere = Sphere(Vector3(-8.25364f, 17.0447f, 3.04695f), 2.99291f); 
  data[17].mBoundingSphere = Sphere(Vector3(14.31f, 14.0308f, -29.1156f), 3.12651f); 
  data[18].mBoundingSphere = Sphere(Vector3(23.9646f, -27.606f, 23.4745f), 2.17695f); 
  data[19].mBoundingSphere = Sphere(Vector3(-21.8959f, 3.7778f, 32.9414f), 1.35286f); 
  data[20].mBoundingSphere = Sphere(Vector3(-13.0895f, -12.8563f, 15.7499f), 3.3951f); 
  data[21].mBoundingSphere = Sphere(Vector3(17.4827f, -26.6448f, 27.7524f), 2.29837f); 
  data[22].mBoundingSphere = Sphere(Vector3(19.8179f, -11.2799f, -19.7126f), 3.82558f); 
  data[23].mBoundingSphere = Sphere(Vector3(-21.3429f, -1.79265f, 23.9636f), 1.88424f); 
  data[24].mBoundingSphere = Sphere(Vector3(-4.4627f, 3.47187f, -47.6791f), 1.1346f); 
  data[25].mBoundingSphere = Sphere(Vector3(16.5251f, 30.4885f, -0.159812f), 3.29629f); 
  data[26].mBoundingSphere = Sphere(Vector3(17.5156f, -4.32158f, 45.0334f), 1.92367f); 
  data[27].mBoundingSphere = Sphere(Vector3(23.6427f, -18.7564f, 3.88442f), 1.91868f); 
  data[28].mBoundingSphere = Sphere(Vector3(23.9828f, -41.5823f, -2.49823f), 2.14923f); 
  data[29].mBoundingSphere = Sphere(Vector3(-1.09527f, 12.0747f, 34.3525f), 3.0422f); 
  data[30].mBoundingSphere = Sphere(Vector3(-6.88793f, -26.7188f, 9.69783f), 0.319681f); 
  data[31].mBoundingSphere = Sphere(Vector3(-20.3007f, -17.3702f, 39.6477f), 3.94816f); 
  data[32].mBoundingSphere = Sphere(Vector3(10.8344f, 5.6439f, 25.8005f), 1.66656f); 
  data[33].mBoundingSphere = Sphere(Vector3(-11.8617f, 12.0225f, 10.267f), 1.16684f); 
  data[34].mBoundingSphere = Sphere(Vector3(-20.9154f, -22.0368f, -20.9467f), 1.88992f); 
  data[35].mBoundingSphere = Sphere(Vector3(-33.9827f, -18.6452f, 3.86003f), 3.91117f); 
  data[36].mBoundingSphere = Sphere(Vector3(5.59918f, -10.8054f, 3.2833f), 3.68004f); 
  data[37].mBoundingSphere = Sphere(Vector3(-30.1118f, 5.47522f, -33.227f), 0.485055f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mBoundingSphere = Sphere(Vector3(8.47304f, 16.4827f, -7.18822f), 3.67749f); 
  data[1].mBoundingSphere = Sphere(Vector3(33.423f, -19.8209f, 22.9979f), 1.36167f); 
  data[2].mBoundingSphere = Sphere(Vector3(38.4018f, -0.519169f, 6.61628f), 1.12568f); 
  data[3].mBoundingSphere = Sphere(Vector3(-16.2933f, 12.0505f, 29.7919f), 1.89665f); 
  data[4].mBoundingSphere = Sphere(Vector3(-24.9788f, -34.269f, -21.3539f), 1.73951f); 
  data[5].mBoundingSphere = Sphere(Vector3(-18.506f, 28.081f, -28.4f), 2.11328f); 
  data[6].mBoundingSphere = Sphere(Vector3(-1.89127f, 44.4512f, -6.07503f), 1.18088f); 
  data[7].mBoundingSphere = Sphere(Vector3(-4.53208f, 35.3618f, 4.3882f), 0.990338f); 
  data[8].mBoundingSphere = Sphere(Vector3(-5.17701f, -21.4012f, -44.5497f), 2.0459f); 
  data[9].mBoundingSphere = Sphere(Vector3(-10.4791f, 26.9379f, 16.8919f), 2.48879f); 
  data[10].mBoundingSphere = Sphere(Vector3(-17.2947f, 15.5733f, -19.4949f), 2.00485f); 
  data[11].mBoundingSphere = Sphere(Vector3(-13.5901f, 3.39871f, -45.1457f), 1.0794f); 
  data[12].mBoundingSphere = Sphere(Vector3(24.6821f, -9.26388f, 7.24867f), 3.31717f); 
  data[13].mBoundingSphere = Sphere(Vector3(30.8787f, -0.630685f, 18.997f), 3.84553f); 
  data[14].mBoundingSphere = Sphere(Vector3(-12.5239f, 23.9715f, -8.38303f), 1.92112f); 
  data[15].mBoundingSphere = Sphere(Vector3(45.8036f, -7.22149f, 13.5152f), 1.75887f); 
  data[16].mBoundingSphere = Sphere(Vector3(2.72986f, -6.29378f, -1.84665f), 1.6988f); 
  data[17].mBoundingSphere = Sphere(Vector3(33.7216f, -28.7474f, -10.0318f), 0.333946f); 
  data[18].mBoundingSphere = Sphere(Vector3(2.22692f, 38.8961f, -7.08542f), 0.715604f); 
  data[19].mBoundingSphere = Sphere(Vector3(-29.3347f, 14.1997f, -22.193f), 3.25419f); 
  data[20].mBoundingSphere = Sphere(Vector3(-11.3308f, 23.9948f, -32.8673f), 2.63573f); 
  data[21].mBoundingSphere = Sphere(Vector3(-26.5464f, 22.679f, 33.0596f), 2.17382f); 
  data[22].mBoundingSphere = Sphere(Vector3(18.7601f, 15.5994f, 26.8808f), 3.9114f); 
  data[23].mBoundingSphere = Sphere(Vector3(-5.39927f, -10.1677f, -11.1848f), 3.09659f); 
  data[24].mBoundingSphere = Sphere(Vector3(-22.6619f, -14.7314f, -10.3516f), 2.34765f); 
  data[25].mBoundingSphere = Sphere(Vector3(-39.3322f, 12.5937f, -1.37837f), 1.92703f); 
  data[26].mBoundingSphere = Sphere(Vector3(-34.8025f, 35.0049f, -7.14647f), 0.442958f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(32); 
  data[0].mBoundingSphere = Sphere(Vector3(-1.07721f, 21.6094f, -43.5651f), 1.00043f); 
  data[1].mBoundingSphere = Sphere(Vector3(10.0629f, -3.29986f, 11.6083f), 3.78186f); 
  data[2].mBoundingSphere = Sphere(Vector3(-41.1747f, -11.5992f, -12.3091f), 0.537706f); 
  data[3].mBoundingSphere = Sphere(Vector3(-1.74384f, 16.0399f, -9.53822f), 1.50154f); 
  data[4].mBoundingSphere = Sphere(Vector3(5.57637f, 8.07686f, 34.6258f), 3.46248f); 
  data[5].mBoundingSphere = Sphere(Vector3(-0.305638f, 26.2359f, -41.1932f), 1.17392f); 
  data[6].mBoundingSphere = Sphere(Vector3(19.3881f, -7.25146f, -5.75779f), 1.29198f); 
  data[7].mBoundingSphere = Sphere(Vector3(-5.71062f, 30.7116f, -17.6444f), 1.91393f); 
  data[8].mBoundingSphere = Sphere(Vector3(-2.14794f, -47.0822f, 2.08624f), 1.48472f); 
  data[9].mBoundingSphere = Sphere(Vector3(-11.584f, 19.1114f, 7.75574f), 3.9412f); 
  data[10].mBoundingSphere = Sphere(Vector3(-29.8046f, 18.4131f, -13.4697f), 0.260884f); 
  data[11].mBoundingSphere = Sphere(Vector3(17.7407f, 20.4285f, 24.042f), 3.48787f); 
  data[12].mBoundingSphere = Sphere(Vector3(35.4968f, 2.82045f, -4.31188f), 3.39869f); 
  data[13].mBoundingSphere = Sphere(Vector3(-18.0333f, 23.9498f, -26.2791f), 1.48321f); 
  data[14].mBoundingSphere = Sphere(Vector3(9.90886f, 8.31362f, 22.2464f), 3.80517f); 
  data[15].mBoundingSphere = Sphere(Vector3(29.3231f, -2.29032f, 29.9614f), 3.21175f); 
  data[16].mBoundingSphere = Sphere(Vector3(10.2215f, 33.8289f, -6.11066f), 3.26289f); 
  data[17].mBoundingSphere = Sphere(Vector3(20.8549f, -26.3112f, -11.3949f), 1.3466f); 
  data[18].mBoundingSphere = Sphere(Vector3(-2.09742f, 20.9752f, 25.6601f), 2.35055f); 
  data[19].mBoundingSphere = Sphere(Vector3(6.57839f, 21.9847f, -1.60417f), 3.15852f); 
  data[20].mBoundingSphere = Sphere(Vector3(-20.9838f, 9.6019f, -20.4514f), 0.339164f); 
  data[21].mBoundingSphere = Sphere(Vector3(-35.9541f, 14.6032f, -24.3407f), 3.09903f); 
  data[22].mBoundingSphere = Sphere(Vector3(2.7053f, -10.2938f, 30.3162f), 1.26345f); 
  data[23].mBoundingSphere = Sphere(Vector3(22.0371f, 0.0781755f, -18.4657f), 3.35973f); 
  data[24].mBoundingSphere = Sphere(Vector3(-19.7983f, 44.5531f, 8.71989f), 3.36715f); 
  data[25].mBoundingSphere = Sphere(Vector3(-19.8441f, 28.5604f, -9.6357f), 2.60488f); 
  data[26].mBoundingSphere = Sphere(Vector3(24.5556f, -14.4416f, -20.7287f), 2.32504f); 
  data[27].mBoundingSphere = Sphere(Vector3(15.5318f, 6.6952f, -46.9675f), 2.28086f); 
  data[28].mBoundingSphere = Sphere(Vector3(-26.4565f, -20.1622f, 33.0214f), 0.638484f); 
  data[29].mBoundingSphere = Sphere(Vector3(28.8337f, 24.5424f, 31.3913f), 3.89969f); 
  data[30].mBoundingSphere = Sphere(Vector3(5.13683f, -13.4913f, 33.6079f), 0.406775f); 
  data[31].mBoundingSphere = Sphere(Vector3(9.10554f, 18.966f, 8.30312f), 1.0968f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mBoundingSphere = Sphere(Vector3(-32.0252f, -7.89826f, 3.02146f), 1.61623f); 
  data[1].mBoundingSphere = Sphere(Vector3(24.2241f, -15.5199f, -33.106f), 0.51138f); 
  data[2].mBoundingSphere = Sphere(Vector3(-24.9047f, -31.0215f, 19.7809f), 2.35369f); 
  data[3].mBoundingSphere = Sphere(Vector3(-2.04725f, 14.1994f, -32.0211f), 3.36773f); 
  data[4].mBoundingSphere = Sphere(Vector3(27.1269f, 12.8464f, 9.8799f), 2.59432f); 
  data[5].mBoundingSphere = Sphere(Vector3(-1.3804f, -7.49573f, 6.27008f), 0.299966f); 
  data[6].mBoundingSphere = Sphere(Vector3(17.0737f, 1.03419f, 36.3536f), 3.96254f); 
  data[7].mBoundingSphere = Sphere(Vector3(-10.6794f, -17.2513f, -31.2592f), 2.05355f); 
  data[8].mBoundingSphere = Sphere(Vector3(15.1652f, -0.0363484f, 45.1696f), 0.737059f); 
  data[9].mBoundingSphere = Sphere(Vector3(-38.4574f, -17.5887f, -16.1101f), 3.90108f); 
  data[10].mBoundingSphere = Sphere(Vector3(-28.1886f, 23.4257f, 14.986f), 1.24176f); 
  data[11].mBoundingSphere = Sphere(Vector3(44.2945f, -0.310023f, 3.95629f), 0.620508f); 
  data[12].mBoundingSphere = Sphere(Vector3(26.4933f, -14.4008f, -30.5624f), 2.20953f); 
  data[13].mBoundingSphere = Sphere(Vector3(-2.45771f, -45.9367f, 4.55259f), 3.84738f); 
  data[14].mBoundingSphere = Sphere(Vector3(-42.2244f, 18.9991f, 5.59517f), 3.52232f); 
  data[15].mBoundingSphere = Sphere(Vector3(26.7944f, 15.4555f, 29.6569f), 1.00136f); 
  data[16].mBoundingSphere = Sphere(Vector3(-5.19424f, -14.7321f, -30.0673f), 2.52613f); 
  data[17].mBoundingSphere = Sphere(Vector3(8.52597f, 29.3312f, -24.3455f), 3.99003f); 
  data[18].mBoundingSphere = Sphere(Vector3(-2.45721f, 2.64324f, 44.3401f), 2.12105f); 
  data[19].mBoundingSphere = Sphere(Vector3(-12.6903f, -39.5101f, 21.947f), 1.48449f); 
  data[20].mBoundingSphere = Sphere(Vector3(12.1897f, 6.73222f, 38.0491f), 2.22589f); 
  data[21].mBoundingSphere = Sphere(Vector3(3.58655f, 19.9805f, 17.7156f), 0.973406f); 
  data[22].mBoundingSphere = Sphere(Vector3(-15.1249f, -17.0531f, -6.16197f), 1.21787f); 
  data[23].mBoundingSphere = Sphere(Vector3(0.69876f, 45.4056f, -7.4417f), 0.989178f); 
  data[24].mBoundingSphere = Sphere(Vector3(-20.575f, 29.9174f, 10.3159f), 3.54783f); 
  data[25].mBoundingSphere = Sphere(Vector3(26.1376f, -3.51174f, -39.1907f), 3.71077f); 
  data[26].mBoundingSphere = Sphere(Vector3(-18.7301f, 0.668311f, -25.7961f), 3.12953f); 
  data[27].mBoundingSphere = Sphere(Vector3(-21.7204f, 27.1084f, 15.3042f), 3.34059f); 
  data[28].mBoundingSphere = Sphere(Vector3(26.666f, -17.4691f, -4.52305f), 3.30511f); 
  data[29].mBoundingSphere = Sphere(Vector3(10.0205f, 40.9254f, 20.9759f), 3.45784f); 
  data[30].mBoundingSphere = Sphere(Vector3(-14.2256f, -25.5227f, 19.7274f), 1.18447f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(30); 
  data[0].mBoundingSphere = Sphere(Vector3(-13.3388f, -4.87409f, 3.41875f), 1.96762f); 
  data[1].mBoundingSphere = Sphere(Vector3(15.4671f, -26.4297f, -22.9504f), 3.17255f); 
  data[2].mBoundingSphere = Sphere(Vector3(-18.5127f, -19.1565f, -22.0217f), 1.46361f); 
  data[3].mBoundingSphere = Sphere(Vector3(-7.33224f, 33.1938f, 18.2531f), 3.78847f); 
  data[4].mBoundingSphere = Sphere(Vector3(-11.8262f, -28.1712f, 2.6383f), 2.96717f); 
  data[5].mBoundingSphere = Sphere(Vector3(-32.1539f, -18.2564f, 17.2192f), 2.8766f); 
  data[6].mBoundingSphere = Sphere(Vector3(16.0932f, 2.96291f, 6.20451f), 0.563335f); 
  data[7].mBoundingSphere = Sphere(Vector3(12.9298f, -16.2934f, -29.3138f), 2.35751f); 
  data[8].mBoundingSphere = Sphere(Vector3(-33.1781f, -11.2936f, -22.0111f), 0.953807f); 
  data[9].mBoundingSphere = Sphere(Vector3(-9.50535f, 1.53695f, 31.4213f), 1.93109f); 
  data[10].mBoundingSphere = Sphere(Vector3(12.2553f, -32.8483f, 35.2155f), 3.87661f); 
  data[11].mBoundingSphere = Sphere(Vector3(21.9626f, 30.7398f, -24.89f), 1.58527f); 
  data[12].mBoundingSphere = Sphere(Vector3(14.1712f, 26.3261f, -37.032f), 3.90282f); 
  data[13].mBoundingSphere = Sphere(Vector3(16.0449f, 41.2809f, 19.5128f), 2.67968f); 
  data[14].mBoundingSphere = Sphere(Vector3(10.3943f, 14.9814f, -29.252f), 2.20756f); 
  data[15].mBoundingSphere = Sphere(Vector3(15.4621f, -21.5889f, -23.5806f), 3.80413f); 
  data[16].mBoundingSphere = Sphere(Vector3(13.4636f, -30.9107f, 0.694687f), 1.71643f); 
  data[17].mBoundingSphere = Sphere(Vector3(17.0854f, -36.4432f, 27.1183f), 2.81223f); 
  data[18].mBoundingSphere = Sphere(Vector3(-20.7267f, 40.5202f, 17.9004f), 0.826008f); 
  data[19].mBoundingSphere = Sphere(Vector3(20.5219f, -27.7889f, -23.9875f), 1.23028f); 
  data[20].mBoundingSphere = Sphere(Vector3(-11.2113f, -12.2944f, 13.1391f), 0.900229f); 
  data[21].mBoundingSphere = Sphere(Vector3(-43.7437f, 4.94466f, -6.09111f), 3.44044f); 
  data[22].mBoundingSphere = Sphere(Vector3(-3.67691f, 31.9924f, -37.3588f), 0.941746f); 
  data[23].mBoundingSphere = Sphere(Vector3(5.07078f, -3.4243f, -13.7341f), 3.03942f); 
  data[24].mBoundingSphere = Sphere(Vector3(-6.84552f, 48.5332f, 5.25789f), 2.23748f); 
  data[25].mBoundingSphere = Sphere(Vector3(25.6422f, -7.04084f, -9.93f), 1.97214f); 
  data[26].mBoundingSphere = Sphere(Vector3(0.874714f, 19.1332f, -34.0604f), 3.98005f); 
  data[27].mBoundingSphere = Sphere(Vector3(1.84159f, 14.25f, 13.419f), 1.6407f); 
  data[28].mBoundingSphere = Sphere(Vector3(-48.8965f, -3.7578f, 7.48978f), 1.2638f); 
  data[29].mBoundingSphere = Sphere(Vector3(-12.2043f, 31.8601f, 14.8715f), 3.98713f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mBoundingSphere = Sphere(Vector3(29.8964f, 11.2144f, -37.3389f), 1.83854f); 
  data[1].mBoundingSphere = Sphere(Vector3(-27.575f, -2.91901f, 32.894f), 0.6829f); 
  data[2].mBoundingSphere = Sphere(Vector3(-1.99579f, -8.58837f, 35.1625f), 2.07107f); 
  data[3].mBoundingSphere = Sphere(Vector3(3.25759f, 6.09078f, -5.54067f), 0.393207f); 
  data[4].mBoundingSphere = Sphere(Vector3(25.4223f, -6.9595f, -3.49156f), 1.70274f); 
  data[5].mBoundingSphere = Sphere(Vector3(20.3554f, -6.61883f, 16.9052f), 3.99965f); 
  data[6].mBoundingSphere = Sphere(Vector3(-8.53333f, 24.9475f, -37.8432f), 1.95521f); 
  data[7].mBoundingSphere = Sphere(Vector3(0.378164f, 31.6791f, -26.1048f), 2.89596f); 
  data[8].mBoundingSphere = Sphere(Vector3(23.7558f, -15.4457f, -17.9272f), 0.964244f); 
  data[9].mBoundingSphere = Sphere(Vector3(-2.33489f, 41.1275f, 15.7053f), 2.67655f); 
  data[10].mBoundingSphere = Sphere(Vector3(7.00901f, -4.31992f, -24.0522f), 1.39577f); 
  data[11].mBoundingSphere = Sphere(Vector3(15.4224f, -17.2249f, 7.31402f), 1.53865f); 
  data[12].mBoundingSphere = Sphere(Vector3(20.1471f, -17.5523f, -7.39789f), 1.43497f); 
  data[13].mBoundingSphere = Sphere(Vector3(-12.287f, -20.2274f, -33.6542f), 1.45921f); 
  data[14].mBoundingSphere = Sphere(Vector3(-40.9018f, 23.006f, -14.9381f), 3.35149f); 
  data[15].mBoundingSphere = Sphere(Vector3(11.7338f, 8.29286f, 21.0935f), 2.74787f); 
  data[16].mBoundingSphere = Sphere(Vector3(20.8628f, -42.8846f, -12.9115f), 1.63896f); 
  data[17].mBoundingSphere = Sphere(Vector3(-1.39036f, -30.6052f, 32.8149f), 0.97387f); 
  data[18].mBoundingSphere = Sphere(Vector3(12.3039f, -2.75187f, 12.3498f), 3.93053f); 
  data[19].mBoundingSphere = Sphere(Vector3(-7.14846f, -17.6315f, -33.7057f), 1.46872f); 
  data[20].mBoundingSphere = Sphere(Vector3(6.61042f, 48.1681f, 7.96442f), 1.08752f); 
  data[21].mBoundingSphere = Sphere(Vector3(23.5617f, 10.1566f, -24.2819f), 2.00299f); 
  data[22].mBoundingSphere = Sphere(Vector3(-15.3686f, -9.45392f, 7.71554f), 0.465688f); 
  data[23].mBoundingSphere = Sphere(Vector3(-12.2776f, -27.374f, -39.8639f), 0.63918f); 
  data[24].mBoundingSphere = Sphere(Vector3(9.1695f, -19.0898f, -10.8209f), 0.75399f); 
  data[25].mBoundingSphere = Sphere(Vector3(13.3672f, -35.5576f, 6.25651f), 2.1592f); 
  data[26].mBoundingSphere = Sphere(Vector3(14.8537f, -23.2751f, 2.64532f), 3.38976f); 
  data[27].mBoundingSphere = Sphere(Vector3(12.3202f, 3.89144f, -29.4906f), 0.266103f); 
  data[28].mBoundingSphere = Sphere(Vector3(22.2154f, 29.7229f, 24.1264f), 3.54864f); 
  data[29].mBoundingSphere = Sphere(Vector3(-28.6089f, 25.3472f, -12.3832f), 3.96185f); 
  data[30].mBoundingSphere = Sphere(Vector3(-35.5686f, 19.8092f, 15.8646f), 2.50955f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(60); 
  data[0].mBoundingSphere = Sphere(Vector3(-41.1529f, 19.2429f, -9.80222f), 2.44495f); 
  data[1].mBoundingSphere = Sphere(Vector3(-23.9417f, -22.2075f, 30.3804f), 1.60115f); 
  data[2].mBoundingSphere = Sphere(Vector3(10.426f, 7.08149f, 39.5625f), 0.658431f); 
  data[3].mBoundingSphere = Sphere(Vector3(3.42137f, -32.0128f, -7.26321f), 2.49123f); 
  data[4].mBoundingSphere = Sphere(Vector3(19.1752f, 13.497f, -36.0653f), 2.26578f); 
  data[5].mBoundingSphere = Sphere(Vector3(30.435f, 11.3898f, -26.4764f), 1.74322f); 
  data[6].mBoundingSphere = Sphere(Vector3(10.594f, -10.5885f, -20.7107f), 3.07537f); 
  data[7].mBoundingSphere = Sphere(Vector3(-10.0891f, 0.0270844f, -32.005f), 2.1621f); 
  data[8].mBoundingSphere = Sphere(Vector3(20.7558f, 35.0032f, 1.102f), 3.54076f); 
  data[9].mBoundingSphere = Sphere(Vector3(8.36185f, -1.14633f, 19.1069f), 0.873788f); 
  data[10].mBoundingSphere = Sphere(Vector3(33.3354f, -15.3893f, 9.99034f), 2.00612f); 
  data[11].mBoundingSphere = Sphere(Vector3(3.12937f, 42.7297f, -19.6203f), 0.858596f); 
  data[12].mBoundingSphere = Sphere(Vector3(-22.6299f, 13.2303f, 26.9513f), 1.72779f); 
  data[13].mBoundingSphere = Sphere(Vector3(26.8171f, -0.0899915f, -18.6036f), 2.06016f); 
  data[14].mBoundingSphere = Sphere(Vector3(-8.66462f, 20.4747f, 7.64175f), 1.26971f); 
  data[15].mBoundingSphere = Sphere(Vector3(5.66874f, 15.576f, -33.1924f), 2.46826f); 
  data[16].mBoundingSphere = Sphere(Vector3(19.3774f, 9.70061f, -23.169f), 2.47522f); 
  data[17].mBoundingSphere = Sphere(Vector3(-11.4268f, 20.047f, -23.0217f), 1.57842f); 
  data[18].mBoundingSphere = Sphere(Vector3(-7.97579f, 15.6295f, -14.4933f), 1.81245f); 
  data[19].mBoundingSphere = Sphere(Vector3(44.4182f, -3.60647f, -13.026f), 2.42466f); 
  data[20].mBoundingSphere = Sphere(Vector3(-37.0931f, 18.3874f, -13.9263f), 3.09636f); 
  data[21].mBoundingSphere = Sphere(Vector3(-30.4404f, 19.3014f, -26.6932f), 0.788781f); 
  data[22].mBoundingSphere = Sphere(Vector3(18.3355f, 23.538f, -19.9119f), 2.18356f); 
  data[23].mBoundingSphere = Sphere(Vector3(-5.14658f, 43.1733f, 7.87945f), 3.38339f); 
  data[24].mBoundingSphere = Sphere(Vector3(-39.631f, 12.4051f, 6.47879f), 0.64463f); 
  data[25].mBoundingSphere = Sphere(Vector3(-31.9169f, -2.15143f, -22.0828f), 2.81049f); 
  data[26].mBoundingSphere = Sphere(Vector3(-9.19504f, 2.75091f, 38.439f), 2.8098f); 
  data[27].mBoundingSphere = Sphere(Vector3(-18.9315f, 25.9203f, 4.17957f), 0.495724f); 
  data[28].mBoundingSphere = Sphere(Vector3(-34.0203f, 13.3865f, -31.4145f), 3.7894f); 
  data[29].mBoundingSphere = Sphere(Vector3(-22.0356f, 19.7507f, 1.86496f), 1.59651f); 
  data[30].mBoundingSphere = Sphere(Vector3(11.8927f, 4.75746f, 30.2735f), 2.67075f); 
  data[31].mBoundingSphere = Sphere(Vector3(20.4235f, -39.7113f, 17.4144f), 0.885385f); 
  data[32].mBoundingSphere = Sphere(Vector3(-27.4804f, -27.0529f, 31.3074f), 2.88924f); 
  data[33].mBoundingSphere = Sphere(Vector3(10.1037f, -10.2624f, -16.0627f), 1.40922f); 
  data[34].mBoundingSphere = Sphere(Vector3(-16.6097f, -2.66934f, -32.624f), 3.57393f); 
  data[35].mBoundingSphere = Sphere(Vector3(5.06253f, 14.5226f, -39.8465f), 1.32201f); 
  data[36].mBoundingSphere = Sphere(Vector3(10.5052f, -34.0822f, 12.2148f), 0.868685f); 
  data[37].mBoundingSphere = Sphere(Vector3(-28.2374f, -8.51282f, 16.4306f), 2.09449f); 
  data[38].mBoundingSphere = Sphere(Vector3(9.83842f, -43.6447f, 21.7439f), 3.0291f); 
  data[39].mBoundingSphere = Sphere(Vector3(-49.7985f, 0.40107f, 3.54558f), 3.8969f); 
  data[40].mBoundingSphere = Sphere(Vector3(40.2607f, -13.9937f, -1.69628f), 2.29245f); 
  data[41].mBoundingSphere = Sphere(Vector3(-1.44599f, 3.43158f, -46.6267f), 3.35764f); 
  data[42].mBoundingSphere = Sphere(Vector3(3.05159f, -16.2984f, -29.7215f), 2.32446f); 
  data[43].mBoundingSphere = Sphere(Vector3(15.5778f, -7.88833f, -35.9931f), 1.29743f); 
  data[44].mBoundingSphere = Sphere(Vector3(20.8394f, -21.961f, 18.7655f), 2.4402f); 
  data[45].mBoundingSphere = Sphere(Vector3(44.6746f, 16.4946f, 3.39933f), 1.4374f); 
  data[46].mBoundingSphere = Sphere(Vector3(4.55899f, 2.28667f, 36.1121f), 3.20027f); 
  data[47].mBoundingSphere = Sphere(Vector3(-33.0835f, 9.96686f, 0.849113f), 3.8831f); 
  data[48].mBoundingSphere = Sphere(Vector3(9.95931f, -25.7993f, 28.5831f), 2.9819f); 
  data[49].mBoundingSphere = Sphere(Vector3(-0.494598f, 27.8069f, -33.954f), 0.98222f); 
  data[50].mBoundingSphere = Sphere(Vector3(-38.8678f, 3.21959f, -13.6594f), 0.724882f); 
  data[51].mBoundingSphere = Sphere(Vector3(-21.1753f, 4.35585f, -42.6516f), 2.15943f); 
  data[52].mBoundingSphere = Sphere(Vector3(22.3078f, 9.42398f, -33.7364f), 1.05296f); 
  data[53].mBoundingSphere = Sphere(Vector3(-10.4989f, -7.77432f, 8.03798f), 0.323972f); 
  data[54].mBoundingSphere = Sphere(Vector3(-3.04767f, -12.8684f, 25.6791f), 2.76944f); 
  data[55].mBoundingSphere = Sphere(Vector3(-19.6363f, 17.0247f, -28.6332f), 1.45376f); 
  data[56].mBoundingSphere = Sphere(Vector3(-15.468f, -16.1727f, 29.114f), 1.12881f); 
  data[57].mBoundingSphere = Sphere(Vector3(-18.1673f, -23.8127f, 38.4473f), 3.35381f); 
  data[58].mBoundingSphere = Sphere(Vector3(20.7084f, -6.81274f, 21.664f), 3.88264f); 
  data[59].mBoundingSphere = Sphere(Vector3(-10.3919f, 45.6307f, 13.7762f), 3.38965f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(36); 
  data[0].mBoundingSphere = Sphere(Vector3(-3.61077f, -10.2347f, -32.0815f), 1.84376f); 
  data[1].mBoundingSphere = Sphere(Vector3(-28.4344f, 27.1667f, -28.0624f), 1.63513f); 
  data[2].mBoundingSphere = Sphere(Vector3(0.063083f, -10.8752f, -8.09251f), 0.436f); 
  data[3].mBoundingSphere = Sphere(Vector3(-8.31752f, 11.5127f, 3.56205f), 0.926554f); 
  data[4].mBoundingSphere = Sphere(Vector3(-5.92786f, -5.69411f, 34.5035f), 3.74684f); 
  data[5].mBoundingSphere = Sphere(Vector3(-14.6609f, -36.2806f, -0.285426f), 1.17021f); 
  data[6].mBoundingSphere = Sphere(Vector3(-1.62855f, -3.14205f, -1.52472f), 2.34232f); 
  data[7].mBoundingSphere = Sphere(Vector3(0.0388654f, -47.6952f, 2.84197f), 0.90278f); 
  data[8].mBoundingSphere = Sphere(Vector3(-14.0727f, -6.25757f, 14.854f), 2.91139f); 
  data[9].mBoundingSphere = Sphere(Vector3(8.53212f, 30.4157f, 31.4734f), 3.09404f); 
  data[10].mBoundingSphere = Sphere(Vector3(-12.2299f, 20.5755f, -40.7358f), 0.730332f); 
  data[11].mBoundingSphere = Sphere(Vector3(-6.6274f, -18.3689f, 16.0259f), 3.9993f); 
  data[12].mBoundingSphere = Sphere(Vector3(-27.0228f, -19.2893f, 20.5167f), 3.90305f); 
  data[13].mBoundingSphere = Sphere(Vector3(-38.556f, -23.1406f, -8.73604f), 3.38675f); 
  data[14].mBoundingSphere = Sphere(Vector3(5.81357f, -4.78529f, -3.40797f), 2.97494f); 
  data[15].mBoundingSphere = Sphere(Vector3(28.9391f, -27.6862f, -21.5835f), 1.93979f); 
  data[16].mBoundingSphere = Sphere(Vector3(26.3982f, 23.693f, -3.95284f), 3.72411f); 
  data[17].mBoundingSphere = Sphere(Vector3(8.77642f, 43.9336f, -12.5568f), 1.03441f); 
  data[18].mBoundingSphere = Sphere(Vector3(16.9992f, 19.75f, 32.072f), 1.52519f); 
  data[19].mBoundingSphere = Sphere(Vector3(34.5295f, 9.94552f, -16.0784f), 1.09146f); 
  data[20].mBoundingSphere = Sphere(Vector3(-44.4658f, 7.54902f, -21.4797f), 3.68966f); 
  data[21].mBoundingSphere = Sphere(Vector3(18.8079f, -23.7661f, -28.0413f), 2.30405f); 
  data[22].mBoundingSphere = Sphere(Vector3(4.99586f, 6.04869f, -46.8473f), 0.980132f); 
  data[23].mBoundingSphere = Sphere(Vector3(-19.6523f, -16.3764f, -29.7668f), 3.93993f); 
  data[24].mBoundingSphere = Sphere(Vector3(-35.3922f, -26.5985f, -1.48033f), 2.30115f); 
  data[25].mBoundingSphere = Sphere(Vector3(0.613626f, 8.51316f, 0.944776f), 2.21846f); 
  data[26].mBoundingSphere = Sphere(Vector3(10.8238f, -13.4215f, -14.7676f), 3.36332f); 
  data[27].mBoundingSphere = Sphere(Vector3(-9.37472f, 34.187f, 33.3909f), 1.96379f); 
  data[28].mBoundingSphere = Sphere(Vector3(-18.1408f, -22.8196f, -25.5658f), 3.32297f); 
  data[29].mBoundingSphere = Sphere(Vector3(-9.02269f, -39.8656f, 27.0274f), 0.231312f); 
  data[30].mBoundingSphere = Sphere(Vector3(4.92242f, -48.7077f, -8.93159f), 0.997644f); 
  data[31].mBoundingSphere = Sphere(Vector3(-5.14623f, -42.2736f, 7.99633f), 2.71064f); 
  data[32].mBoundingSphere = Sphere(Vector3(-39.4815f, -16.6211f, 2.88922f), 1.05563f); 
  data[33].mBoundingSphere = Sphere(Vector3(28.7264f, -30.2028f, 11.9482f), 0.702152f); 
  data[34].mBoundingSphere = Sphere(Vector3(-34.1682f, 3.8092f, -10.7237f), 3.581f); 
  data[35].mBoundingSphere = Sphere(Vector3(33.278f, 28.9135f, 13.8559f), 3.67076f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(57); 
  data[0].mBoundingSphere = Sphere(Vector3(19.8823f, 1.7487f, 26.2706f), 2.74149f); 
  data[1].mBoundingSphere = Sphere(Vector3(-25.1273f, 35.305f, -2.93602f), 1.64105f); 
  data[2].mBoundingSphere = Sphere(Vector3(21.9559f, -22.4811f, -36.4688f), 2.37352f); 
  data[3].mBoundingSphere = Sphere(Vector3(-1.50612f, 13.149f, -30.9305f), 0.475546f); 
  data[4].mBoundingSphere = Sphere(Vector3(21.5196f, 21.0836f, 16.0745f), 1.09181f); 
  data[5].mBoundingSphere = Sphere(Vector3(2.8773f, -2.75166f, -4.81797f), 3.03316f); 
  data[6].mBoundingSphere = Sphere(Vector3(-13.7874f, 2.76456f, 43.7163f), 1.39705f); 
  data[7].mBoundingSphere = Sphere(Vector3(33.9154f, 18.4773f, -2.84479f), 0.318986f); 
  data[8].mBoundingSphere = Sphere(Vector3(31.6422f, -16.63f, 1.18429f), 2.7234f); 
  data[9].mBoundingSphere = Sphere(Vector3(-25.3433f, 11.0661f, 26.7577f), 2.84215f); 
  data[10].mBoundingSphere = Sphere(Vector3(11.6663f, -35.6746f, -2.05001f), 1.5717f); 
  data[11].mBoundingSphere = Sphere(Vector3(7.08929f, 40.7973f, -14.7348f), 2.03523f); 
  data[12].mBoundingSphere = Sphere(Vector3(7.88795f, -15.2294f, -46.4051f), 1.6189f); 
  data[13].mBoundingSphere = Sphere(Vector3(-29.3916f, -25.4037f, -25.3551f), 2.45214f); 
  data[14].mBoundingSphere = Sphere(Vector3(-21.3765f, 20.9795f, 13.2819f), 3.40055f); 
  data[15].mBoundingSphere = Sphere(Vector3(39.0005f, -7.94858f, 12.6362f), 3.13799f); 
  data[16].mBoundingSphere = Sphere(Vector3(18.6514f, -28.7858f, -25.3494f), 3.97031f); 
  data[17].mBoundingSphere = Sphere(Vector3(-2.29595f, 24.6802f, 12.7788f), 0.375463f); 
  data[18].mBoundingSphere = Sphere(Vector3(5.84152f, 5.03622f, 16.7427f), 2.73279f); 
  data[19].mBoundingSphere = Sphere(Vector3(-22.58f, -31.1408f, -7.66681f), 0.406195f); 
  data[20].mBoundingSphere = Sphere(Vector3(9.51192f, 28.6766f, -20.8665f), 2.05321f); 
  data[21].mBoundingSphere = Sphere(Vector3(22.3099f, 19.58f, 33.285f), 0.32664f); 
  data[22].mBoundingSphere = Sphere(Vector3(24.6661f, -6.3474f, 23.3308f), 2.63955f); 
  data[23].mBoundingSphere = Sphere(Vector3(-12.542f, 45.059f, 17.0236f), 1.75922f); 
  data[24].mBoundingSphere = Sphere(Vector3(39.384f, -6.5271f, 11.534f), 1.30253f); 
  data[25].mBoundingSphere = Sphere(Vector3(-19.4642f, 15.8376f, -8.2885f), 0.298343f); 
  data[26].mBoundingSphere = Sphere(Vector3(-30.0663f, -4.57797f, -5.48038f), 2.48821f); 
  data[27].mBoundingSphere = Sphere(Vector3(3.23729f, 35.7276f, 23.2279f), 3.93169f); 
  data[28].mBoundingSphere = Sphere(Vector3(-7.51446f, -31.012f, 3.20336f), 1.78705f); 
  data[29].mBoundingSphere = Sphere(Vector3(-23.4538f, 7.64869f, -29.8481f), 0.635701f); 
  data[30].mBoundingSphere = Sphere(Vector3(-4.62078f, -13.2268f, -5.92851f), 2.21754f); 
  data[31].mBoundingSphere = Sphere(Vector3(1.2756f, -37.7292f, -9.97775f), 0.547911f); 
  data[32].mBoundingSphere = Sphere(Vector3(13.7984f, 5.03152f, 5.04929f), 2.4845f); 
  data[33].mBoundingSphere = Sphere(Vector3(-31.2848f, 23.5398f, 6.24955f), 0.711661f); 
  data[34].mBoundingSphere = Sphere(Vector3(-5.33689f, 32.0405f, 31.2376f), 2.81304f); 
  data[35].mBoundingSphere = Sphere(Vector3(-15.2467f, -42.335f, 11.0058f), 1.59825f); 
  data[36].mBoundingSphere = Sphere(Vector3(-26.7275f, 29.6504f, -4.05194f), 1.75655f); 
  data[37].mBoundingSphere = Sphere(Vector3(-7.85502f, -38.3441f, 13.982f), 3.20212f); 
  data[38].mBoundingSphere = Sphere(Vector3(-29.3629f, -20.0633f, -28.8076f), 2.98143f); 
  data[39].mBoundingSphere = Sphere(Vector3(32.312f, 19.4648f, -20.5152f), 3.51281f); 
  data[40].mBoundingSphere = Sphere(Vector3(-11.2712f, 14.7796f, -9.70742f), 1.62412f); 
  data[41].mBoundingSphere = Sphere(Vector3(5.47351f, -4.116f, 24.2433f), 3.16026f); 
  data[42].mBoundingSphere = Sphere(Vector3(25.1719f, 32.1993f, -6.77039f), 3.42618f); 
  data[43].mBoundingSphere = Sphere(Vector3(5.17514f, -31.8412f, 21.5353f), 1.03673f); 
  data[44].mBoundingSphere = Sphere(Vector3(-12.2214f, -34.0986f, -0.936637f), 0.641615f); 
  data[45].mBoundingSphere = Sphere(Vector3(19.1437f, 6.07096f, 28.1773f), 3.69245f); 
  data[46].mBoundingSphere = Sphere(Vector3(8.89584f, 25.6296f, 12.9024f), 3.52197f); 
  data[47].mBoundingSphere = Sphere(Vector3(44.6183f, 14.1402f, 0.00142839f), 2.90617f); 
  data[48].mBoundingSphere = Sphere(Vector3(-10.2183f, -17.5416f, 4.46259f), 1.97307f); 
  data[49].mBoundingSphere = Sphere(Vector3(-37.6493f, -8.95666f, 22.5359f), 1.23225f); 
  data[50].mBoundingSphere = Sphere(Vector3(-17.5907f, 7.68096f, -2.27521f), 1.79019f); 
  data[51].mBoundingSphere = Sphere(Vector3(-1.62784f, 42.2667f, 22.9849f), 1.92158f); 
  data[52].mBoundingSphere = Sphere(Vector3(2.48161f, -34.7069f, -7.74472f), 2.46594f); 
  data[53].mBoundingSphere = Sphere(Vector3(-25.571f, -19.6273f, 33.6415f), 1.21671f); 
  data[54].mBoundingSphere = Sphere(Vector3(-28.0855f, -13.7148f, -14.7791f), 3.97379f); 
  data[55].mBoundingSphere = Sphere(Vector3(-15.1235f, 6.29517f, 23.0449f), 2.02433f); 
  data[56].mBoundingSphere = Sphere(Vector3(-4.51902f, 9.27329f, 42.562f), 0.726737f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(30); 
  data[0].mBoundingSphere = Sphere(Vector3(-7.7449f, 28.5437f, 26.2115f), 1.66726f); 
  data[1].mBoundingSphere = Sphere(Vector3(-21.0008f, -16.6375f, -28.6768f), 2.66553f); 
  data[2].mBoundingSphere = Sphere(Vector3(0.0707304f, -24.1882f, 29.5773f), 0.380334f); 
  data[3].mBoundingSphere = Sphere(Vector3(-26.2257f, 30.7845f, -16.1204f), 1.01933f); 
  data[4].mBoundingSphere = Sphere(Vector3(11.5629f, -17.8008f, 0.328477f), 1.90616f); 
  data[5].mBoundingSphere = Sphere(Vector3(29.8249f, 36.0679f, 2.83469f), 3.84228f); 
  data[6].mBoundingSphere = Sphere(Vector3(-5.96455f, 17.2537f, 37.4045f), 3.49588f); 
  data[7].mBoundingSphere = Sphere(Vector3(18.2718f, -9.85485f, -38.8216f), 2.03859f); 
  data[8].mBoundingSphere = Sphere(Vector3(23.3575f, 30.561f, 6.56526f), 3.54563f); 
  data[9].mBoundingSphere = Sphere(Vector3(-27.0977f, 13.8166f, -31.1473f), 1.38139f); 
  data[10].mBoundingSphere = Sphere(Vector3(-18.6039f, -16.2928f, -14.3563f), 2.08695f); 
  data[11].mBoundingSphere = Sphere(Vector3(-19.6668f, -20.1178f, -28.6389f), 1.38939f); 
  data[12].mBoundingSphere = Sphere(Vector3(-3.4275f, 36.3151f, -24.922f), 3.65128f); 
  data[13].mBoundingSphere = Sphere(Vector3(-17.2753f, 11.8673f, -7.3968f), 3.17916f); 
  data[14].mBoundingSphere = Sphere(Vector3(12.4588f, 24.5131f, 20.5983f), 3.50979f); 
  data[15].mBoundingSphere = Sphere(Vector3(-8.11979f, 22.3708f, -10.2364f), 0.692642f); 
  data[16].mBoundingSphere = Sphere(Vector3(-7.03555f, 43.9529f, 5.44003f), 2.88842f); 
  data[17].mBoundingSphere = Sphere(Vector3(-5.82432f, 2.12318f, 35.1505f), 0.899881f); 
  data[18].mBoundingSphere = Sphere(Vector3(12.1511f, -33.6281f, -31.0871f), 3.75762f); 
  data[19].mBoundingSphere = Sphere(Vector3(-37.5647f, -4.73572f, -11.1088f), 0.775329f); 
  data[20].mBoundingSphere = Sphere(Vector3(14.9533f, -11.3573f, 22.3435f), 2.19701f); 
  data[21].mBoundingSphere = Sphere(Vector3(37.1882f, -9.33441f, 12.7055f), 0.851289f); 
  data[22].mBoundingSphere = Sphere(Vector3(-3.7067f, 19.4771f, 4.79664f), 3.13567f); 
  data[23].mBoundingSphere = Sphere(Vector3(7.88938f, 21.7946f, -7.32269f), 2.14482f); 
  data[24].mBoundingSphere = Sphere(Vector3(4.78409f, 4.2872f, 2.78642f), 3.2666f); 
  data[25].mBoundingSphere = Sphere(Vector3(13.5942f, 3.76353f, 4.18638f), 1.65972f); 
  data[26].mBoundingSphere = Sphere(Vector3(26.5884f, -7.77435f, -21.2998f), 2.93655f); 
  data[27].mBoundingSphere = Sphere(Vector3(2.6811f, 43.7417f, 13.4271f), 1.66459f); 
  data[28].mBoundingSphere = Sphere(Vector3(11.6406f, -27.3535f, 37.8183f), 1.41108f); 
  data[29].mBoundingSphere = Sphere(Vector3(22.5908f, 12.2067f, -23.5958f), 2.34464f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(59); 
  data[0].mBoundingSphere = Sphere(Vector3(-44.2257f, -18.8985f, 0.371356f), 0.530168f); 
  data[1].mBoundingSphere = Sphere(Vector3(-20.105f, -19.2087f, -34.0145f), 3.40368f); 
  data[2].mBoundingSphere = Sphere(Vector3(33.5015f, 26.7981f, -21.6323f), 1.93051f); 
  data[3].mBoundingSphere = Sphere(Vector3(37.1811f, 24.6543f, 17.4954f), 3.10471f); 
  data[4].mBoundingSphere = Sphere(Vector3(-33.4637f, -16.8126f, -11.0629f), 0.561132f); 
  data[5].mBoundingSphere = Sphere(Vector3(-26.2793f, -14.8676f, -11.2214f), 2.27483f); 
  data[6].mBoundingSphere = Sphere(Vector3(0.541932f, -47.6977f, -3.64563f), 3.67134f); 
  data[7].mBoundingSphere = Sphere(Vector3(0.0604228f, -16.369f, -29.5409f), 0.319102f); 
  data[8].mBoundingSphere = Sphere(Vector3(-13.8409f, 2.38257f, -13.534f), 3.27113f); 
  data[9].mBoundingSphere = Sphere(Vector3(-10.9757f, 6.91227f, -21.507f), 3.02411f); 
  data[10].mBoundingSphere = Sphere(Vector3(32.1578f, 24.2595f, -11.4417f), 0.965984f); 
  data[11].mBoundingSphere = Sphere(Vector3(43.304f, 21.203f, 0.704918f), 1.07871f); 
  data[12].mBoundingSphere = Sphere(Vector3(23.3957f, 34.0398f, 1.41896f), 0.965172f); 
  data[13].mBoundingSphere = Sphere(Vector3(-12.2015f, -11.8016f, 31.3446f), 0.370708f); 
  data[14].mBoundingSphere = Sphere(Vector3(40.0572f, 6.05598f, -19.257f), 1.66517f); 
  data[15].mBoundingSphere = Sphere(Vector3(27.8722f, 26.8194f, -6.07132f), 1.45747f); 
  data[16].mBoundingSphere = Sphere(Vector3(9.86394f, -12.3229f, 39.6848f), 1.0082f); 
  data[17].mBoundingSphere = Sphere(Vector3(-2.14451f, -22.2205f, -18.2204f), 0.411994f); 
  data[18].mBoundingSphere = Sphere(Vector3(-48.608f, -11.3669f, -1.90419f), 0.415937f); 
  data[19].mBoundingSphere = Sphere(Vector3(2.62616f, -15.5911f, 30.1213f), 3.44462f); 
  data[20].mBoundingSphere = Sphere(Vector3(2.10771f, -7.64314f, -27.9231f), 3.49854f); 
  data[21].mBoundingSphere = Sphere(Vector3(22.9664f, -18.5887f, 28.9913f), 2.09797f); 
  data[22].mBoundingSphere = Sphere(Vector3(-38.663f, 13.709f, -20.5934f), 1.937f); 
  data[23].mBoundingSphere = Sphere(Vector3(-12.4048f, 16.3243f, -39.5534f), 0.219251f); 
  data[24].mBoundingSphere = Sphere(Vector3(2.79508f, -27.4471f, -9.38345f), 1.74345f); 
  data[25].mBoundingSphere = Sphere(Vector3(-27.8103f, 1.81569f, 37.2438f), 1.50606f); 
  data[26].mBoundingSphere = Sphere(Vector3(33.7707f, -7.66879f, -27.0469f), 2.4932f); 
  data[27].mBoundingSphere = Sphere(Vector3(3.84519f, 38.0483f, 19.7257f), 3.22659f); 
  data[28].mBoundingSphere = Sphere(Vector3(6.07018f, -17.2217f, 17.1913f), 1.7344f); 
  data[29].mBoundingSphere = Sphere(Vector3(8.87051f, -26.4708f, -9.62802f), 3.7168f); 
  data[30].mBoundingSphere = Sphere(Vector3(-8.63085f, 13.7997f, -26.1114f), 1.77117f); 
  data[31].mBoundingSphere = Sphere(Vector3(-12.7852f, 0.866738f, -33.9953f), 0.850941f); 
  data[32].mBoundingSphere = Sphere(Vector3(5.81385f, -32.2817f, 29.3245f), 3.62205f); 
  data[33].mBoundingSphere = Sphere(Vector3(-33.1072f, -24.792f, -15.0633f), 0.488186f); 
  data[34].mBoundingSphere = Sphere(Vector3(-31.7804f, 5.30138f, -21.9786f), 3.35405f); 
  data[35].mBoundingSphere = Sphere(Vector3(-23.3278f, -0.666687f, -12.5268f), 0.692758f); 
  data[36].mBoundingSphere = Sphere(Vector3(23.6168f, 4.86989f, -27.6342f), 2.19063f); 
  data[37].mBoundingSphere = Sphere(Vector3(9.4023f, -32.982f, -11.9221f), 1.13124f); 
  data[38].mBoundingSphere = Sphere(Vector3(-39.191f, -14.4571f, -1.29969f), 3.79601f); 
  data[39].mBoundingSphere = Sphere(Vector3(-18.4073f, -14.2412f, -37.4031f), 3.44253f); 
  data[40].mBoundingSphere = Sphere(Vector3(23.181f, 0.673612f, -21.4602f), 2.53344f); 
  data[41].mBoundingSphere = Sphere(Vector3(28.3315f, 5.59896f, -18.1896f), 2.58447f); 
  data[42].mBoundingSphere = Sphere(Vector3(-16.2757f, -36.1024f, -5.09968f), 0.698325f); 
  data[43].mBoundingSphere = Sphere(Vector3(-20.4317f, -42.9683f, 9.5756f), 3.35532f); 
  data[44].mBoundingSphere = Sphere(Vector3(14.9867f, -19.4689f, -23.8253f), 1.52786f); 
  data[45].mBoundingSphere = Sphere(Vector3(-27.4379f, 41.0322f, -6.0911f), 1.28919f); 
  data[46].mBoundingSphere = Sphere(Vector3(-41.5373f, 4.79207f, -3.58223f), 0.783447f); 
  data[47].mBoundingSphere = Sphere(Vector3(14.3284f, -38.36f, 5.55531f), 3.9274f); 
  data[48].mBoundingSphere = Sphere(Vector3(24.0988f, 14.6181f, -22.286f), 3.51037f); 
  data[49].mBoundingSphere = Sphere(Vector3(1.74104f, -18.0581f, -46.3965f), 1.26449f); 
  data[50].mBoundingSphere = Sphere(Vector3(-1.0687f, -18.6349f, 21.7402f), 1.37037f); 
  data[51].mBoundingSphere = Sphere(Vector3(33.5001f, -31.1873f, -5.1745f), 3.16965f); 
  data[52].mBoundingSphere = Sphere(Vector3(16.9698f, 34.6458f, -9.86149f), 1.7511f); 
  data[53].mBoundingSphere = Sphere(Vector3(25.964f, -7.47303f, 34.6037f), 3.18218f); 
  data[54].mBoundingSphere = Sphere(Vector3(20.2351f, 3.93848f, -26.4188f), 2.24943f); 
  data[55].mBoundingSphere = Sphere(Vector3(4.74067f, 45.2602f, 4.08423f), 1.68326f); 
  data[56].mBoundingSphere = Sphere(Vector3(6.59493f, 19.457f, 41.168f), 1.72965f); 
  data[57].mBoundingSphere = Sphere(Vector3(16.4454f, -31.8562f, 28.0043f), 1.04171f); 
  data[58].mBoundingSphere = Sphere(Vector3(-12.2858f, 22.5174f, 13.8536f), 2.35763f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(65); 
  data[0].mBoundingSphere = Sphere(Vector3(-18.4349f, 33.5727f, 24.8786f), 1.71933f); 
  data[1].mBoundingSphere = Sphere(Vector3(-39.7946f, 25.372f, 13.1243f), 2.50398f); 
  data[2].mBoundingSphere = Sphere(Vector3(-41.0019f, -10.7315f, -10.4456f), 2.85166f); 
  data[3].mBoundingSphere = Sphere(Vector3(-1.86378f, 5.95127f, -25.7888f), 2.86569f); 
  data[4].mBoundingSphere = Sphere(Vector3(31.2624f, -0.170852f, 26.6069f), 2.53576f); 
  data[5].mBoundingSphere = Sphere(Vector3(36.8401f, -12.0963f, -12.6256f), 1.28873f); 
  data[6].mBoundingSphere = Sphere(Vector3(45.4843f, 2.09034f, 4.17084f), 1.55593f); 
  data[7].mBoundingSphere = Sphere(Vector3(-20.3812f, -17.9254f, 1.30782f), 3.17325f); 
  data[8].mBoundingSphere = Sphere(Vector3(38.562f, -10.2789f, -0.113268f), 3.93482f); 
  data[9].mBoundingSphere = Sphere(Vector3(-19.8379f, 12.2137f, -6.43713f), 0.717808f); 
  data[10].mBoundingSphere = Sphere(Vector3(14.6547f, -8.17452f, 4.16812f), 2.69789f); 
  data[11].mBoundingSphere = Sphere(Vector3(-28.2269f, 5.32844f, 36.8507f), 1.67259f); 
  data[12].mBoundingSphere = Sphere(Vector3(-12.7871f, 21.6165f, 42.6023f), 2.21382f); 
  data[13].mBoundingSphere = Sphere(Vector3(9.67123f, 28.787f, -18.2628f), 3.46596f); 
  data[14].mBoundingSphere = Sphere(Vector3(25.0346f, -28.4947f, -21.5895f), 1.10909f); 
  data[15].mBoundingSphere = Sphere(Vector3(-16.6637f, -35.7487f, -30.1101f), 3.90722f); 
  data[16].mBoundingSphere = Sphere(Vector3(-23.9182f, 40.1521f, 5.83437f), 0.357604f); 
  data[17].mBoundingSphere = Sphere(Vector3(14.2521f, 39.897f, -13.0219f), 2.86372f); 
  data[18].mBoundingSphere = Sphere(Vector3(5.16853f, -10.0307f, 26.0043f), 2.34592f); 
  data[19].mBoundingSphere = Sphere(Vector3(6.4315f, -7.74141f, 13.7176f), 2.51361f); 
  data[20].mBoundingSphere = Sphere(Vector3(18.1274f, -18.8263f, -13.896f), 0.91287f); 
  data[21].mBoundingSphere = Sphere(Vector3(-16.0557f, 11.433f, -44.1709f), 2.3727f); 
  data[22].mBoundingSphere = Sphere(Vector3(7.0992f, -21.389f, -13.066f), 0.36433f); 
  data[23].mBoundingSphere = Sphere(Vector3(8.33443f, 2.59389f, -40.1484f), 0.496884f); 
  data[24].mBoundingSphere = Sphere(Vector3(19.2956f, 45.8776f, -2.94423f), 2.66356f); 
  data[25].mBoundingSphere = Sphere(Vector3(-37.0756f, -12.446f, 22.0323f), 0.860103f); 
  data[26].mBoundingSphere = Sphere(Vector3(-35.0878f, 15.9663f, 22.7747f), 0.794928f); 
  data[27].mBoundingSphere = Sphere(Vector3(-17.383f, 31.1595f, -31.4249f), 0.305649f); 
  data[28].mBoundingSphere = Sphere(Vector3(-36.4197f, 5.40252f, -17.1491f), 1.10538f); 
  data[29].mBoundingSphere = Sphere(Vector3(-1.26206f, -24.0206f, -31.1764f), 0.674319f); 
  data[30].mBoundingSphere = Sphere(Vector3(-23.0665f, -23.9238f, -10.6068f), 2.41782f); 
  data[31].mBoundingSphere = Sphere(Vector3(-34.8232f, 6.57362f, 26.6947f), 0.50651f); 
  data[32].mBoundingSphere = Sphere(Vector3(-30.2063f, -3.62578f, 8.77306f), 0.321537f); 
  data[33].mBoundingSphere = Sphere(Vector3(26.0674f, 9.24008f, -36.2353f), 0.519382f); 
  data[34].mBoundingSphere = Sphere(Vector3(11.0561f, 28.0781f, -15.3407f), 1.43752f); 
  data[35].mBoundingSphere = Sphere(Vector3(-39.2569f, -6.47123f, -25.6589f), 3.85968f); 
  data[36].mBoundingSphere = Sphere(Vector3(-8.70907f, 29.0142f, -8.22205f), 2.05738f); 
  data[37].mBoundingSphere = Sphere(Vector3(15.563f, 34.7614f, -30.6542f), 3.5302f); 
  data[38].mBoundingSphere = Sphere(Vector3(-31.1195f, 12.6674f, -17.6037f), 3.3588f); 
  data[39].mBoundingSphere = Sphere(Vector3(-35.8211f, 4.09781f, 27.507f), 2.5934f); 
  data[40].mBoundingSphere = Sphere(Vector3(-34.4765f, -32.8007f, -11.0775f), 2.39509f); 
  data[41].mBoundingSphere = Sphere(Vector3(43.8885f, 16.7612f, -10.6296f), 2.54759f); 
  data[42].mBoundingSphere = Sphere(Vector3(20.2666f, -34.6767f, 16.305f), 0.66017f); 
  data[43].mBoundingSphere = Sphere(Vector3(4.93938f, -5.22725f, -22.7977f), 1.67967f); 
  data[44].mBoundingSphere = Sphere(Vector3(2.37636f, -36.1577f, 24.4062f), 3.6289f); 
  data[45].mBoundingSphere = Sphere(Vector3(-41.4418f, 7.67499f, -3.49641f), 3.39301f); 
  data[46].mBoundingSphere = Sphere(Vector3(-32.4978f, -25.8372f, 11.5823f), 3.67076f); 
  data[47].mBoundingSphere = Sphere(Vector3(-31.3694f, 23.2941f, 13.5287f), 0.30878f); 
  data[48].mBoundingSphere = Sphere(Vector3(15.5628f, 6.41333f, 31.7042f), 3.34059f); 
  data[49].mBoundingSphere = Sphere(Vector3(-4.20288f, -12.6831f, 43.6015f), 2.54422f); 
  data[50].mBoundingSphere = Sphere(Vector3(-14.9035f, 14.2445f, -32.8202f), 3.04324f); 
  data[51].mBoundingSphere = Sphere(Vector3(-17.623f, -8.38917f, 26.0112f), 3.76435f); 
  data[52].mBoundingSphere = Sphere(Vector3(-21.5045f, -11.6383f, 10.6544f), 2.4736f); 
  data[53].mBoundingSphere = Sphere(Vector3(24.7155f, 26.9419f, -0.355955f), 2.80945f); 
  data[54].mBoundingSphere = Sphere(Vector3(2.00475f, -8.06051f, 27.5796f), 0.630714f); 
  data[55].mBoundingSphere = Sphere(Vector3(-16.5862f, 37.2285f, -5.52665f), 1.09796f); 
  data[56].mBoundingSphere = Sphere(Vector3(-16.8986f, -45.4936f, -6.21309f), 2.00867f); 
  data[57].mBoundingSphere = Sphere(Vector3(30.1149f, -23.5075f, 11.6012f), 2.40297f); 
  data[58].mBoundingSphere = Sphere(Vector3(28.7951f, 26.9311f, -6.08623f), 2.36053f); 
  data[59].mBoundingSphere = Sphere(Vector3(-29.34f, -25.6158f, 9.06921f), 3.60524f); 
  data[60].mBoundingSphere = Sphere(Vector3(-0.772138f, -18.3971f, -12.6788f), 3.4387f); 
  data[61].mBoundingSphere = Sphere(Vector3(-39.4749f, 6.54604f, -19.9817f), 3.01993f); 
  data[62].mBoundingSphere = Sphere(Vector3(27.4428f, -14.8897f, 16.9717f), 3.50968f); 
  data[63].mBoundingSphere = Sphere(Vector3(32.1708f, 28.3985f, 5.90515f), 3.49959f); 
  data[64].mBoundingSphere = Sphere(Vector3(-5.77437f, -7.60494f, 13.7457f), 1.64371f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(73); 
  data[0].mBoundingSphere = Sphere(Vector3(41.1016f, 17.6241f, 15.1909f), 2.55687f); 
  data[1].mBoundingSphere = Sphere(Vector3(15.3035f, 1.48509f, -20.5123f), 1.43126f); 
  data[2].mBoundingSphere = Sphere(Vector3(-21.1155f, 15.5109f, -18.9078f), 1.42302f); 
  data[3].mBoundingSphere = Sphere(Vector3(8.5752f, 1.83521f, -12.0096f), 0.922147f); 
  data[4].mBoundingSphere = Sphere(Vector3(4.32166f, -1.51978f, 21.5841f), 1.18238f); 
  data[5].mBoundingSphere = Sphere(Vector3(13.3112f, -34.5412f, 25.3036f), 3.74626f); 
  data[6].mBoundingSphere = Sphere(Vector3(-30.6611f, -24.2524f, -29.8774f), 1.37617f); 
  data[7].mBoundingSphere = Sphere(Vector3(-44.9003f, 13.4424f, 4.55133f), 1.56022f); 
  data[8].mBoundingSphere = Sphere(Vector3(12.938f, -3.93161f, 4.91971f), 1.96043f); 
  data[9].mBoundingSphere = Sphere(Vector3(8.31379f, 0.740862f, -31.731f), 2.47406f); 
  data[10].mBoundingSphere = Sphere(Vector3(-21.7394f, -4.34605f, -1.25436f), 3.67586f); 
  data[11].mBoundingSphere = Sphere(Vector3(13.135f, -23.8611f, 41.5664f), 0.308896f); 
  data[12].mBoundingSphere = Sphere(Vector3(-5.26007f, 22.9142f, -40.8704f), 3.19517f); 
  data[13].mBoundingSphere = Sphere(Vector3(42.7534f, 6.7658f, 8.75082f), 3.23135f); 
  data[14].mBoundingSphere = Sphere(Vector3(17.7258f, 14.6705f, -22.7439f), 2.04312f); 
  data[15].mBoundingSphere = Sphere(Vector3(25.6022f, 23.7249f, -34.7045f), 3.96138f); 
  data[16].mBoundingSphere = Sphere(Vector3(-15.4083f, -2.19215f, -24.6636f), 2.69348f); 
  data[17].mBoundingSphere = Sphere(Vector3(2.22029f, -20.7746f, -20.6626f), 1.75621f); 
  data[18].mBoundingSphere = Sphere(Vector3(-21.5117f, 4.38638f, 42.9693f), 3.23958f); 
  data[19].mBoundingSphere = Sphere(Vector3(-17.1057f, -16.5071f, -42.9255f), 3.96938f); 
  data[20].mBoundingSphere = Sphere(Vector3(1.96123f, 42.5416f, -22.0854f), 2.04022f); 
  data[21].mBoundingSphere = Sphere(Vector3(-8.92541f, 17.7248f, -12.719f), 3.38339f); 
  data[22].mBoundingSphere = Sphere(Vector3(-41.0097f, -12.0039f, -17.6724f), 0.94163f); 
  data[23].mBoundingSphere = Sphere(Vector3(-29.7538f, -25.3798f, -0.0131282f), 3.51907f); 
  data[24].mBoundingSphere = Sphere(Vector3(-2.22386f, -3.185f, -36.9073f), 3.27275f); 
  data[25].mBoundingSphere = Sphere(Vector3(-31.3488f, -13.3533f, -1.19975f), 0.718619f); 
  data[26].mBoundingSphere = Sphere(Vector3(-8.24154f, -45.8372f, 3.75447f), 2.81652f); 
  data[27].mBoundingSphere = Sphere(Vector3(15.2654f, -4.13183f, -2.23291f), 2.80516f); 
  data[28].mBoundingSphere = Sphere(Vector3(10.1429f, 41.9476f, 4.35116f), 1.75621f); 
  data[29].mBoundingSphere = Sphere(Vector3(1.04298f, -13.6312f, 7.49055f), 0.714792f); 
  data[30].mBoundingSphere = Sphere(Vector3(-20.5065f, 7.96475f, 33.5394f), 0.596155f); 
  data[31].mBoundingSphere = Sphere(Vector3(-0.338213f, -24.2428f, -36.3124f), 3.30673f); 
  data[32].mBoundingSphere = Sphere(Vector3(-1.21761f, 32.7585f, -21.3372f), 1.85849f); 
  data[33].mBoundingSphere = Sphere(Vector3(3.63352f, 25.0823f, -23.3475f), 2.3458f); 
  data[34].mBoundingSphere = Sphere(Vector3(17.7563f, -29.2755f, 29.3255f), 0.953111f); 
  data[35].mBoundingSphere = Sphere(Vector3(19.9842f, -11.5324f, -36.4441f), 0.661678f); 
  data[36].mBoundingSphere = Sphere(Vector3(-25.7082f, -24.6282f, 1.0881f), 2.22519f); 
  data[37].mBoundingSphere = Sphere(Vector3(39.7132f, -19.9477f, -13.962f), 0.882138f); 
  data[38].mBoundingSphere = Sphere(Vector3(-12.7282f, 16.0109f, 22.8763f), 3.17499f); 
  data[39].mBoundingSphere = Sphere(Vector3(12.9529f, -13.7817f, 1.34772f), 0.341832f); 
  data[40].mBoundingSphere = Sphere(Vector3(-1.35077f, 8.95496f, -7.2914f), 3.85469f); 
  data[41].mBoundingSphere = Sphere(Vector3(23.301f, 18.9707f, -0.964253f), 3.56604f); 
  data[42].mBoundingSphere = Sphere(Vector3(-8.24571f, 6.93576f, -32.6351f), 3.4394f); 
  data[43].mBoundingSphere = Sphere(Vector3(24.544f, 20.0611f, -21.6375f), 2.92832f); 
  data[44].mBoundingSphere = Sphere(Vector3(7.41381f, -3.54582f, -1.17791f), 2.97308f); 
  data[45].mBoundingSphere = Sphere(Vector3(18.1589f, -8.00172f, -2.51814f), 3.48567f); 
  data[46].mBoundingSphere = Sphere(Vector3(26.4382f, -23.601f, 17.9432f), 2.8897f); 
  data[47].mBoundingSphere = Sphere(Vector3(22.7912f, -29.2001f, 11.2381f), 0.78565f); 
  data[48].mBoundingSphere = Sphere(Vector3(-5.08196f, -10.9078f, 34.7074f), 2.90918f); 
  data[49].mBoundingSphere = Sphere(Vector3(15.3721f, 10.1761f, 25.6424f), 2.85108f); 
  data[50].mBoundingSphere = Sphere(Vector3(5.20313f, -0.521044f, -17.363f), 0.508945f); 
  data[51].mBoundingSphere = Sphere(Vector3(-13.8265f, -27.9362f, -20.9413f), 1.33779f); 
  data[52].mBoundingSphere = Sphere(Vector3(-20.6549f, -20.3961f, -20.4819f), 3.16965f); 
  data[53].mBoundingSphere = Sphere(Vector3(-40.7861f, 17.3732f, 3.0636f), 2.06886f); 
  data[54].mBoundingSphere = Sphere(Vector3(-24.9833f, -32.0911f, 16.3635f), 1.19595f); 
  data[55].mBoundingSphere = Sphere(Vector3(36.6582f, 30.8886f, -13.9257f), 0.942674f); 
  data[56].mBoundingSphere = Sphere(Vector3(-43.4273f, 5.07762f, -15.4531f), 3.71355f); 
  data[57].mBoundingSphere = Sphere(Vector3(-15.8144f, -10.7969f, -45.7783f), 0.216932f); 
  data[58].mBoundingSphere = Sphere(Vector3(-1.70521f, 24.6799f, -7.74253f), 1.03939f); 
  data[59].mBoundingSphere = Sphere(Vector3(-12.1158f, 43.5278f, 10.4164f), 0.567046f); 
  data[60].mBoundingSphere = Sphere(Vector3(7.007f, 48.8314f, -5.99513f), 1.62806f); 
  data[61].mBoundingSphere = Sphere(Vector3(-34.669f, -29.6644f, -16.2662f), 3.1199f); 
  data[62].mBoundingSphere = Sphere(Vector3(-12.2742f, 44.7942f, 11.5428f), 2.99326f); 
  data[63].mBoundingSphere = Sphere(Vector3(-11.2908f, -5.90782f, 8.85974f), 2.46177f); 
  data[64].mBoundingSphere = Sphere(Vector3(-5.96669f, -25.0011f, 9.53928f), 3.00312f); 
  data[65].mBoundingSphere = Sphere(Vector3(-26.6709f, -10.97f, 9.08043f), 1.02316f); 
  data[66].mBoundingSphere = Sphere(Vector3(26.3893f, -22.2618f, -24.955f), 2.1519f); 
  data[67].mBoundingSphere = Sphere(Vector3(6.25855f, 10.2036f, -19.5707f), 3.03652f); 
  data[68].mBoundingSphere = Sphere(Vector3(-7.21883f, 27.4572f, -9.21999f), 0.605896f); 
  data[69].mBoundingSphere = Sphere(Vector3(11.1947f, 22.4506f, 9.42411f), 3.26973f); 
  data[70].mBoundingSphere = Sphere(Vector3(-5.83587f, -0.451876f, -12.1654f), 1.42465f); 
  data[71].mBoundingSphere = Sphere(Vector3(-4.23301f, -23.2105f, -19.6645f), 0.935136f); 
  data[72].mBoundingSphere = Sphere(Vector3(-12.9882f, 4.95313f, -27.1704f), 0.956822f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(42); 
  data[0].mBoundingSphere = Sphere(Vector3(7.75305f, -34.5791f, -34.5822f), 1.87345f); 
  data[1].mBoundingSphere = Sphere(Vector3(-8.93181f, -8.91854f, -15.915f), 1.8825f); 
  data[2].mBoundingSphere = Sphere(Vector3(-17.1835f, -32.2846f, -16.7128f), 3.30963f); 
  data[3].mBoundingSphere = Sphere(Vector3(-19.0653f, -44.2458f, -2.44242f), 2.35809f); 
  data[4].mBoundingSphere = Sphere(Vector3(-8.57288f, 30.937f, -35.3369f), 0.774517f); 
  data[5].mBoundingSphere = Sphere(Vector3(25.5716f, 7.48236f, -30.1153f), 3.84854f); 
  data[6].mBoundingSphere = Sphere(Vector3(15.2617f, -0.332209f, -9.46507f), 0.934208f); 
  data[7].mBoundingSphere = Sphere(Vector3(-28.1244f, 3.76497f, -15.3148f), 0.813483f); 
  data[8].mBoundingSphere = Sphere(Vector3(2.93887f, 15.7297f, -44.2118f), 2.3153f); 
  data[9].mBoundingSphere = Sphere(Vector3(-18.7531f, 3.87261f, -35.5983f), 2.91545f); 
  data[10].mBoundingSphere = Sphere(Vector3(-2.40429f, 33.1343f, -27.198f), 1.73174f); 
  data[11].mBoundingSphere = Sphere(Vector3(-27.1045f, -5.81976f, 33.66f), 0.833778f); 
  data[12].mBoundingSphere = Sphere(Vector3(17.9763f, 32.1144f, 12.4936f), 3.08813f); 
  data[13].mBoundingSphere = Sphere(Vector3(-4.7487f, -13.5263f, -13.8263f), 2.85491f); 
  data[14].mBoundingSphere = Sphere(Vector3(1.91827f, -21.1705f, -5.39387f), 1.9566f); 
  data[15].mBoundingSphere = Sphere(Vector3(26.4382f, 9.3088f, 19.6833f), 1.6371f); 
  data[16].mBoundingSphere = Sphere(Vector3(6.61579f, -27.6154f, -0.968585f), 2.92925f); 
  data[17].mBoundingSphere = Sphere(Vector3(-38.0946f, 2.1279f, -13.8813f), 0.350298f); 
  data[18].mBoundingSphere = Sphere(Vector3(21.3865f, 35.5214f, -20.8468f), 1.34486f); 
  data[19].mBoundingSphere = Sphere(Vector3(-42.797f, -2.95903f, 13.8194f), 3.19111f); 
  data[20].mBoundingSphere = Sphere(Vector3(-32.9946f, 23.9073f, 21.3982f), 1.59257f); 
  data[21].mBoundingSphere = Sphere(Vector3(12.4749f, 32.5945f, -13.5838f), 0.672231f); 
  data[22].mBoundingSphere = Sphere(Vector3(27.3436f, 21.0672f, 12.7954f), 3.58796f); 
  data[23].mBoundingSphere = Sphere(Vector3(-3.84731f, 31.7049f, 2.53137f), 2.88796f); 
  data[24].mBoundingSphere = Sphere(Vector3(-15.8039f, -16.7153f, 2.10291f), 3.02597f); 
  data[25].mBoundingSphere = Sphere(Vector3(-21.8261f, 32.8989f, 27.2239f), 2.95337f); 
  data[26].mBoundingSphere = Sphere(Vector3(14.5508f, -22.038f, 23.2703f), 1.60823f); 
  data[27].mBoundingSphere = Sphere(Vector3(1.46927f, 5.76106f, 45.6854f), 3.61962f); 
  data[28].mBoundingSphere = Sphere(Vector3(23.6237f, 3.22935f, -35.8387f), 0.645326f); 
  data[29].mBoundingSphere = Sphere(Vector3(-15.6417f, 33.1239f, -4.17552f), 0.663417f); 
  data[30].mBoundingSphere = Sphere(Vector3(28.4334f, -29.7002f, 7.68873f), 0.834126f); 
  data[31].mBoundingSphere = Sphere(Vector3(-16.3133f, 20.1137f, -20.3462f), 2.85247f); 
  data[32].mBoundingSphere = Sphere(Vector3(-12.6282f, -23.5732f, 8.9804f), 0.610071f); 
  data[33].mBoundingSphere = Sphere(Vector3(27.9367f, -8.53625f, -22.1731f), 1.08451f); 
  data[34].mBoundingSphere = Sphere(Vector3(-25.0325f, -7.15556f, 0.883258f), 0.674667f); 
  data[35].mBoundingSphere = Sphere(Vector3(-15.4948f, -0.552873f, -37.1326f), 0.702615f); 
  data[36].mBoundingSphere = Sphere(Vector3(15.0615f, -21.0636f, 26.7496f), 3.29792f); 
  data[37].mBoundingSphere = Sphere(Vector3(23.9785f, 33.4662f, -23.2809f), 2.98538f); 
  data[38].mBoundingSphere = Sphere(Vector3(-1.42185f, -4.52494f, -45.5776f), 2.59502f); 
  data[39].mBoundingSphere = Sphere(Vector3(20.4988f, -33.9437f, -15.2458f), 0.410834f); 
  data[40].mBoundingSphere = Sphere(Vector3(-28.1368f, -26.5949f, 14.1658f), 3.77977f); 
  data[41].mBoundingSphere = Sphere(Vector3(-41.5291f, -7.94679f, -24.3355f), 0.89733f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void SphereSpatialPartitionSelfQueryFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mBoundingSphere = Sphere(Vector3(15.3516f, 37.9164f, 21.7173f), 3.29073f); 
  data[1].mBoundingSphere = Sphere(Vector3(-16.6094f, -0.391814f, -8.71492f), 2.51813f); 
  data[2].mBoundingSphere = Sphere(Vector3(18.5878f, 19.5356f, 41.6782f), 1.65612f); 
  data[3].mBoundingSphere = Sphere(Vector3(22.7061f, 18.6315f, -28.0229f), 0.898257f); 
  data[4].mBoundingSphere = Sphere(Vector3(35.9152f, 0.23072f, -17.5943f), 1.76734f); 
  data[5].mBoundingSphere = Sphere(Vector3(-39.9507f, 26.5183f, 14.1351f), 3.38281f); 
  data[6].mBoundingSphere = Sphere(Vector3(9.7112f, -9.94728f, -36.1083f), 2.20895f); 
  data[7].mBoundingSphere = Sphere(Vector3(18.2049f, 6.59881f, -5.98027f), 3.95268f); 
  data[8].mBoundingSphere = Sphere(Vector3(-10.9452f, -3.98993f, -35.0112f), 1.17891f); 
  data[9].mBoundingSphere = Sphere(Vector3(-15.522f, 23.8506f, -16.7847f), 1.31494f); 
  data[10].mBoundingSphere = Sphere(Vector3(26.5303f, 37.5882f, 4.28267f), 1.60846f); 
  data[11].mBoundingSphere = Sphere(Vector3(5.51348f, -19.8891f, 11.3759f), 3.31195f); 
  data[12].mBoundingSphere = Sphere(Vector3(-3.12375f, -33.2332f, 21.496f), 3.45911f); 
  data[13].mBoundingSphere = Sphere(Vector3(41.8168f, -24.8529f, -2.5853f), 3.30429f); 
  data[14].mBoundingSphere = Sphere(Vector3(17.851f, 23.468f, 25.2992f), 2.19272f); 
  data[15].mBoundingSphere = Sphere(Vector3(30.753f, 10.8744f, 11.9084f), 0.729752f); 
  data[16].mBoundingSphere = Sphere(Vector3(34.111f, -17.6945f, -1.46347f), 1.01887f); 
  data[17].mBoundingSphere = Sphere(Vector3(-21.3853f, 17.8971f, -25.4553f), 1.23689f); 
  data[18].mBoundingSphere = Sphere(Vector3(-28.9385f, 39.3832f, -3.27221f), 2.71493f); 
  data[19].mBoundingSphere = Sphere(Vector3(3.09651f, 0.811412f, -32.2733f), 3.99362f); 
  data[20].mBoundingSphere = Sphere(Vector3(-17.4403f, -19.3401f, -4.59785f), 3.82245f); 
  data[21].mBoundingSphere = Sphere(Vector3(-13.2077f, 32.0981f, 17.5703f), 2.77048f); 
  data[22].mBoundingSphere = Sphere(Vector3(8.8168f, 26.8391f, -4.67077f), 2.61845f); 
  data[23].mBoundingSphere = Sphere(Vector3(38.3229f, 12.7462f, -26.0876f), 0.799799f); 
  data[24].mBoundingSphere = Sphere(Vector3(20.5243f, 20.9427f, -17.8918f), 1.86139f); 
  data[25].mBoundingSphere = Sphere(Vector3(22.3172f, 26.6435f, -17.9302f), 1.40911f); 
  data[26].mBoundingSphere = Sphere(Vector3(-8.09812f, -20.8528f, -28.1493f), 2.5121f); 
  data[27].mBoundingSphere = Sphere(Vector3(23.9629f, 18.2946f, 6.43805f), 1.02919f); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  BoundingSphereSpatialPartition spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mSphereSphereTests", static_cast<int>(Application::mStatistics.mSphereSphereTests)); 
  } 
}

void DebugDrawingPoints1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  gDebugDrawer->DrawPoint(Vector3(0, 0, 0));
  gDebugDrawer->DrawPoint(Vector3(1, 0, 0));
  gDebugDrawer->DrawPoint(Vector3(0, 1, 0));
  gDebugDrawer->DrawPoint(Vector3(0, 0, 1));
}

void DebugDrawingLines1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  gDebugDrawer->DrawLine(LineSegment(Vector3(0, 0, 0), Vector3(1, 0, 0)));
  gDebugDrawer->DrawLine(LineSegment(Vector3(0, 0, 0), Vector3(0, 1, 0)));
  gDebugDrawer->DrawLine(LineSegment(Vector3(0, 0, 0), Vector3(0, 0, 1)));
}

void DebugDrawingRays1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  gDebugDrawer->DrawRay(Ray(Vector3(0, 0, 0), Vector3(1, 0, 0)), 5.0f);
  gDebugDrawer->DrawRay(Ray(Vector3(0, 0, 0), Vector3(0, 1, 0)), 5.0f);
  gDebugDrawer->DrawRay(Ray(Vector3(0, 0, 0), Vector3(0, 0, 1)), 5.0f);
}

void DebugDrawingSpheres1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  gDebugDrawer->DrawSphere(Sphere(Vector3(0.577812f, 0.776177f, -0.602959f), 0.464431f));
  gDebugDrawer->DrawSphere(Sphere(Vector3(0.207284f, -6.23721f, -1.81981f), 4.92792f));
  gDebugDrawer->DrawSphere(Sphere(Vector3(-1.47537f, -3.06628f, 5.96103f), 4.30209f));
  gDebugDrawer->DrawSphere(Sphere(Vector3(0.429898f, 0.439505f, -3.27661f), 2.48084f));
  gDebugDrawer->DrawSphere(Sphere(Vector3(-4.36159f, -2.82456f, 3.94174f), 2.29526f));
}

void DebugDrawingAabbs1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  gDebugDrawer->DrawAabb(Aabb(Vector3(-0.403611f, -1.15883f, 0.148383f), Vector3(9.11458f, -1.08467f, 1.5852f)));
  gDebugDrawer->DrawAabb(Aabb(Vector3(-4.35474f, 3.87758f, -5.91895f), Vector3(-1.90989f, 5.3357f, 5.60585f)));
  gDebugDrawer->DrawAabb(Aabb(Vector3(0.964619f, -8.42168f, -2.07626f), Vector3(2.97204f, -1.10177f, 0.241842f)));
  gDebugDrawer->DrawAabb(Aabb(Vector3(-4.96121f, -0.145595f, -5.8373f), Vector3(-3.95404f, 0.285578f, -3.56056f)));
  gDebugDrawer->DrawAabb(Aabb(Vector3(2.04614f, 0.412525f, -0.525727f), Vector3(6.36345f, 5.07618f, 1.69215f)));
}

void DebugDrawingTriangles1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  gDebugDrawer->DrawTriangle(Triangle(Vector3(0, 0, 0), Vector3(1, 1, 0), Vector3(1, 1, 1)));
}

void DebugDrawingPlanes1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  Plane plane(Vector3(0.066559f, -0.520518f, -0.851253f), Vector3(0.577812f, 0.776177f, -0.602959f));
  gDebugDrawer->DrawPlane(plane, 3, 4);
}

void DebugDrawingQuads1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  Vector3 p0 = Vector3(-2.18192f, -1.94761f, 0.846782f);
  Vector3 p1 = Vector3(-2.18192f, 3.30789f, -2.36682f);
  Vector3 p2 = Vector3(3.33754f, 3.49997f, -2.0527f);
  Vector3 p3 = Vector3(3.33754f, -1.75554f, 1.1609f);
  gDebugDrawer->DrawQuad(p0, p1, p2, p3);
}

void DebugDrawingFrustum1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  Frustum frustum;
  frustum.Set(Vector3(-2.0169f, -9.57391f, -1.3047f), Vector3(2.42922f, -9.57391f, -1.3047f), Vector3(2.42922f, -2.83268f, -1.3047f), Vector3(-2.0169f, -2.83268f, -1.3047f), Vector3(-0.94259f, -8.4996f, -2.31514f), Vector3(1.3549f, -8.4996f, -2.31514f), Vector3(1.3549f, -3.90699f, -2.31514f), Vector3(-0.94259f, -3.90699f, -2.31514f));
  gDebugDrawer->DrawFrustum(frustum);
}

void RegisterAabbSurfaceAreaTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(AabbSurfaceAreaTest1, list);
  DeclareSimpleUnitTest(AabbSurfaceAreaTest2, list);
}

void RegisterAabbVolumeTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(AabbVolumeTest1, list);
  DeclareSimpleUnitTest(AabbVolumeTest2, list);
}

void RegisterAabbContainsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(AabbContainsTest1, list);
  DeclareSimpleUnitTest(AabbContainsTest2, list);
  DeclareSimpleUnitTest(AabbContainsTest3, list);
  DeclareSimpleUnitTest(AabbContainsTest4, list);
  DeclareSimpleUnitTest(AabbContainsTest5, list);
}

void RegisterAabbTransformTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(AabbTransformTest1, list);
  DeclareSimpleUnitTest(AabbTransformTest2, list);
  DeclareSimpleUnitTest(AabbTransformTest3, list);
  DeclareSimpleUnitTest(AabbTransformTest4, list);
  DeclareSimpleUnitTest(AabbTransformTest5, list);
  DeclareSimpleUnitTest(AabbTransformTest6, list);
  DeclareSimpleUnitTest(AabbTransformTest7, list);
}

void RegisterSphereCentroidTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereCentroidTest1, list);
  DeclareSimpleUnitTest(SphereCentroidTest2, list);
  DeclareSimpleUnitTest(SphereCentroidTest3, list);
  DeclareSimpleUnitTest(SphereCentroidTest4, list);
  DeclareSimpleUnitTest(SphereCentroidTest5, list);
  DeclareSimpleUnitTest(SphereCentroidTest6, list);
  DeclareSimpleUnitTest(SphereCentroidTest7, list);
  DeclareSimpleUnitTest(SphereCentroidTest8, list);
  DeclareSimpleUnitTest(SphereCentroidTest9, list);
  DeclareSimpleUnitTest(SphereCentroidTest10, list);
  DeclareSimpleUnitTest(SphereCentroidTest11, list);
  DeclareSimpleUnitTest(SphereCentroidTest12, list);
  DeclareSimpleUnitTest(SphereCentroidTest13, list);
  DeclareSimpleUnitTest(SphereCentroidTest14, list);
  DeclareSimpleUnitTest(SphereCentroidTest15, list);
  DeclareSimpleUnitTest(SphereCentroidTest16, list);
  DeclareSimpleUnitTest(SphereCentroidTest17, list);
  DeclareSimpleUnitTest(SphereCentroidTest18, list);
  DeclareSimpleUnitTest(SphereCentroidTest19, list);
  DeclareSimpleUnitTest(SphereCentroidTest20, list);
  DeclareSimpleUnitTest(SphereCentroidTest21, list);
  DeclareSimpleUnitTest(SphereCentroidTest22, list);
  DeclareSimpleUnitTest(SphereCentroidTest23, list);
  DeclareSimpleUnitTest(SphereCentroidTest24, list);
  DeclareSimpleUnitTest(SphereCentroidTest25, list);
  DeclareSimpleUnitTest(SphereCentroidTest26, list);
  DeclareSimpleUnitTest(SphereCentroidTest27, list);
  DeclareSimpleUnitTest(SphereCentroidTest28, list);
  DeclareSimpleUnitTest(SphereCentroidTest29, list);
  DeclareSimpleUnitTest(SphereCentroidTest30, list);
  DeclareSimpleUnitTest(SphereCentroidTest31, list);
  DeclareSimpleUnitTest(SphereCentroidTest32, list);
  DeclareSimpleUnitTest(SphereCentroidTest33, list);
  DeclareSimpleUnitTest(SphereCentroidTest34, list);
  DeclareSimpleUnitTest(SphereCentroidTest35, list);
  DeclareSimpleUnitTest(SphereCentroidTest36, list);
  DeclareSimpleUnitTest(SphereCentroidTest37, list);
  DeclareSimpleUnitTest(SphereCentroidTest38, list);
  DeclareSimpleUnitTest(SphereCentroidTest39, list);
  DeclareSimpleUnitTest(SphereCentroidTest40, list);
  DeclareSimpleUnitTest(SphereCentroidTest41, list);
  DeclareSimpleUnitTest(SphereCentroidTest42, list);
  DeclareSimpleUnitTest(SphereCentroidTest43, list);
  DeclareSimpleUnitTest(SphereCentroidTest44, list);
  DeclareSimpleUnitTest(SphereCentroidTest45, list);
  DeclareSimpleUnitTest(SphereCentroidTest46, list);
  DeclareSimpleUnitTest(SphereCentroidTest47, list);
  DeclareSimpleUnitTest(SphereCentroidTest48, list);
  DeclareSimpleUnitTest(SphereCentroidTest49, list);
  DeclareSimpleUnitTest(SphereCentroidTest50, list);
  DeclareSimpleUnitTest(SphereCentroidTest51, list);
  DeclareSimpleUnitTest(SphereCentroidTest52, list);
}

void RegisterSphereRitterTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereRitterTest1, list);
  DeclareSimpleUnitTest(SphereRitterTest2, list);
  DeclareSimpleUnitTest(SphereRitterTest3, list);
  DeclareSimpleUnitTest(SphereRitterTest4, list);
  DeclareSimpleUnitTest(SphereRitterTest5, list);
  DeclareSimpleUnitTest(SphereRitterTest6, list);
  DeclareSimpleUnitTest(SphereRitterTest7, list);
  DeclareSimpleUnitTest(SphereRitterTest8, list);
  DeclareSimpleUnitTest(SphereRitterTest9, list);
  DeclareSimpleUnitTest(SphereRitterTest10, list);
  DeclareSimpleUnitTest(SphereRitterTest11, list);
  DeclareSimpleUnitTest(SphereRitterTest12, list);
  DeclareSimpleUnitTest(SphereRitterTest13, list);
  DeclareSimpleUnitTest(SphereRitterTest14, list);
  DeclareSimpleUnitTest(SphereRitterTest15, list);
  DeclareSimpleUnitTest(SphereRitterTest16, list);
  DeclareSimpleUnitTest(SphereRitterTest17, list);
  DeclareSimpleUnitTest(SphereRitterTest18, list);
  DeclareSimpleUnitTest(SphereRitterTest19, list);
  DeclareSimpleUnitTest(SphereRitterTest20, list);
  DeclareSimpleUnitTest(SphereRitterTest21, list);
  DeclareSimpleUnitTest(SphereRitterTest22, list);
  DeclareSimpleUnitTest(SphereRitterTest23, list);
  DeclareSimpleUnitTest(SphereRitterTest24, list);
  DeclareSimpleUnitTest(SphereRitterTest25, list);
  DeclareSimpleUnitTest(SphereRitterTest26, list);
  DeclareSimpleUnitTest(SphereRitterTest27, list);
  DeclareSimpleUnitTest(SphereRitterTest28, list);
  DeclareSimpleUnitTest(SphereRitterTest29, list);
  DeclareSimpleUnitTest(SphereRitterTest30, list);
  DeclareSimpleUnitTest(SphereRitterTest31, list);
  DeclareSimpleUnitTest(SphereRitterTest32, list);
  DeclareSimpleUnitTest(SphereRitterTest33, list);
  DeclareSimpleUnitTest(SphereRitterTest34, list);
  DeclareSimpleUnitTest(SphereRitterTest35, list);
  DeclareSimpleUnitTest(SphereRitterTest36, list);
  DeclareSimpleUnitTest(SphereRitterTest37, list);
  DeclareSimpleUnitTest(SphereRitterTest38, list);
  DeclareSimpleUnitTest(SphereRitterTest39, list);
  DeclareSimpleUnitTest(SphereRitterTest40, list);
  DeclareSimpleUnitTest(SphereRitterTest41, list);
  DeclareSimpleUnitTest(SphereRitterTest42, list);
  DeclareSimpleUnitTest(SphereRitterTest43, list);
  DeclareSimpleUnitTest(SphereRitterTest44, list);
  DeclareSimpleUnitTest(SphereRitterTest45, list);
  DeclareSimpleUnitTest(SphereRitterTest46, list);
  DeclareSimpleUnitTest(SphereRitterTest47, list);
  DeclareSimpleUnitTest(SphereRitterTest48, list);
  DeclareSimpleUnitTest(SphereRitterTest49, list);
  DeclareSimpleUnitTest(SphereRitterTest50, list);
  DeclareSimpleUnitTest(SphereRitterTest51, list);
  DeclareSimpleUnitTest(SphereRitterTest52, list);
}

void RegisterSpherePCATests(AssignmentUnitTestList& list)
{
}

void RegisterSphereSpatialPartitionStructureTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionStructure1, list);
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionStructure2, list);
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionStructure3, list);
}

void RegisterSphereSpatialPartitionRayCastTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionRayCastTest1, list);
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionRayCastTest2, list);
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionRayCastTest3, list);
}

void RegisterSphereSpatialPartitionFrustumCastTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionFrustumCastTest1, list);
}

void RegisterSphereSpatialPartitionSelfQueryTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionSelfQuery1, list);
  DeclareSimpleUnitTest(BoundingSphereSpatialPartitionSelfQuery2, list);
}

void RegisterSphereSpatialPartitionStructureFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest1, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest2, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest3, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest4, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest5, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest6, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest7, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest8, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest9, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest10, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest11, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest12, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest13, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest14, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest15, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest16, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest17, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest18, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest19, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest20, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest21, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest22, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest23, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest24, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionStructureFuzzTest25, list);
}

void RegisterSphereSpatialPartitionRayCastFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest1, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest2, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest3, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest4, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest5, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest6, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest7, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest8, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest9, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest10, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest11, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest12, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest13, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest14, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest15, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest16, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest17, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest18, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest19, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest20, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest21, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest22, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest23, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest24, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionRayCastFuzzTest25, list);
}

void RegisterSphereSpatialPartitionFrustumCastFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest1, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest2, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest3, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest4, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest5, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest6, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest7, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest8, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest9, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest10, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest11, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest12, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest13, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest14, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest15, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest16, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest17, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest18, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest19, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest20, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest21, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest22, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest23, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest24, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionFrustumCastFuzzTest25, list);
}

void RegisterSphereSpatialPartitionSelfQueryFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest1, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest2, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest3, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest4, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest5, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest6, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest7, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest8, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest9, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest10, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest11, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest12, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest13, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest14, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest15, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest16, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest17, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest18, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest19, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest20, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest21, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest22, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest23, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest24, list);
  DeclareSimpleUnitTest(SphereSpatialPartitionSelfQueryFuzzTest25, list);
}

void RegisterDebugDrawingPointsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingPoints1, list);
}

void RegisterDebugDrawingLinesTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingLines1, list);
}

void RegisterDebugDrawingRaysTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingRays1, list);
}

void RegisterDebugDrawingSpheresTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingSpheres1, list);
}

void RegisterDebugDrawingAabbsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingAabbs1, list);
}

void RegisterDebugDrawingTrianglesTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingTriangles1, list);
}

void RegisterDebugDrawingPlanesTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingPlanes1, list);
}

void RegisterDebugDrawingQuadsTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingQuads1, list);
}

void RegisterDebugDrawingFrustumTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DebugDrawingFrustum1, list);
}

void InitializeAssignment2Tests()
{
  mTestFns.push_back(AssignmentUnitTestList());
  AssignmentUnitTestList& list = mTestFns[1];
  
  // Add Aabb Shape Tests
  RegisterAabbSurfaceAreaTests(list);
  RegisterAabbVolumeTests(list);
  RegisterAabbContainsTests(list);
  RegisterAabbTransformTests(list);
  // Add Sphere Shape Tests
  RegisterSphereCentroidTests(list);
  RegisterSphereRitterTests(list);
  RegisterSpherePCATests(list);
  // Add Sphere Spatial Partition Tests
  RegisterSphereSpatialPartitionStructureTests(list);
  RegisterSphereSpatialPartitionRayCastTests(list);
  RegisterSphereSpatialPartitionFrustumCastTests(list);
  RegisterSphereSpatialPartitionSelfQueryTests(list);
  RegisterSphereSpatialPartitionStructureFuzzTests(list);
  RegisterSphereSpatialPartitionRayCastFuzzTests(list);
  RegisterSphereSpatialPartitionFrustumCastFuzzTests(list);
  RegisterSphereSpatialPartitionSelfQueryFuzzTests(list);
  // Add Debug Drawing Tests
  RegisterDebugDrawingPointsTests(list);
  RegisterDebugDrawingLinesTests(list);
  RegisterDebugDrawingRaysTests(list);
  RegisterDebugDrawingSpheresTests(list);
  RegisterDebugDrawingAabbsTests(list);
  RegisterDebugDrawingTrianglesTests(list);
  RegisterDebugDrawingPlanesTests(list);
  RegisterDebugDrawingQuadsTests(list);
  RegisterDebugDrawingFrustumTests(list);
}
