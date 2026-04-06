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

void PrintAabbTreeFrustumCastTest(SpatialPartition& spatialPartition, const Frustum& frustum, size_t& testCount, FILE* outFile)
{
  if(outFile != NULL)
    fprintf(outFile, "  Test Frustum Cast\n");
  CastResults castResults;

  testCount = 0;
  spatialPartition.CastFrustum(frustum, castResults);
  if(outFile != NULL)
    fprintf(outFile, "    CastResults: %d\n", static_cast<int>(castResults.mResults.size()));
  // Do the cast again to check that the current number of node/plane tests is performed
  testCount = 0;
  spatialPartition.CastFrustum(frustum, castResults);
  if(outFile != NULL)
    fprintf(outFile, "    Plane Tests: %d\n", static_cast<int>(testCount));

  Matrix4 transform = Matrix4::cIdentity;
  spatialPartition.DebugDraw(-1, transform);
  frustum.DebugDraw();
}

void DynamicAabbTreeStructure1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mAabb = Aabb(Vector3(-4, 0, -5), Vector3(-3, 0.1f, 5));
  data[1].mAabb = Aabb(Vector3(1, 0, -1), Vector3(2, 1, 1));
  data[2].mAabb = Aabb(Vector3(-2, 0, -1), Vector3(-1, 0.1f, 1));
  data[3].mAabb = Aabb(Vector3(-2, 0.2f, -1), Vector3(-1, 0.3f, 1));
  
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;
  
  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());
  
  DynamicAabbTree spatialPartition;
  spatialPartition.InsertData(keys[0], data[0]);
  
  // Expected Tree:
  // [Data0]
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
  
  // Expected Tree:
  //     [Root]
  //     /    \
  // [Data0] [Data1]
  spatialPartition.InsertData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
  
  // Expected Tree:
  //     [Root]
  //     /    \
  // [Data0] [Node]
  //         /    \
  //    [Data1] [Data2] 
  spatialPartition.InsertData(keys[2], data[2]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // Expected Tree: 
  //             [Root] 
  //           /        \ 
  //     [Node]          [Node] 
  //     /    \          /    \ 
  //[Data0] [Data1] [Data2] [Data3] 
  spatialPartition.InsertData(keys[3], data[3]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructure2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(4); 
  data[0].mAabb = Aabb(Vector3(-4, 0, -5), Vector3(-3, 0.1f, 5)); 
  data[1].mAabb = Aabb(Vector3(1, 0, -1), Vector3(2, 1, 1)); 
  data[2].mAabb = Aabb(Vector3(-2, 0, -1), Vector3(-1, 0.1f, 1)); 
  data[3].mAabb = Aabb(Vector3(-2, 0.2f, -1), Vector3(-1, 0.3f, 1)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
   
  // Expected Tree: 
  //             [Root] 
  //           /        \ 
  //     [Node]          [Node] 
  //     /    \          /    \ 
  //[Data0] [Data1] [Data2] [Data3] 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // Expected Tree: 
  //             [Root] 
  //           /        \ 
  //     [Node]          [Data3] 
  //     /    \           
  //[Data0] [Data1] 
  spatialPartition.RemoveData(keys[2]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // Expected Tree: 
  //     [Root] 
  //     /    \           
  //[Data0] [Data1] 
  spatialPartition.RemoveData(keys[3]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // Expected Tree: 
  // [Data1] 
  spatialPartition.RemoveData(keys[0]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // [Empty]  
  // Expected Tree:  
  spatialPartition.RemoveData(keys[1]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructure3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(5); 
  data[0].mAabb = Aabb(Vector3(-4, 0, -5), Vector3(-3, 0.1f, 5)); 
  data[1].mAabb = Aabb(Vector3(1, 0, -1), Vector3(2, 1, 1)); 
  data[2].mAabb = Aabb(Vector3(-2, 0, -1), Vector3(-1, 0.1f, 1)); 
  data[3].mAabb = Aabb(Vector3(-2, 0.2f, -1), Vector3(-1, 0.3f, 1)); 
  data[4].mAabb = Aabb(Vector3(0.5f, 0, -1), Vector3(0.9f, 1, 1)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
   
  // Expected Tree:  
  //             [Root]  
  //           /        \  
  //     [Node]          [Node]  
  //     /    \          /    \  
  //[Data0] [Node]  [Data2] [Data3] 
  //        /    \ 
  //    [Data1] [Data4] 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // Expected Tree:  
  //             [Root]  
  //           /        \  
  //     [Node]          [Node]  
  //     /    \          /    \  
  // [Data0] [Data3] [Data1] [Data4] 
  spatialPartition.RemoveData(keys[2]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // Expected Tree:  
  //             [Root]  
  //           /        \  
  //     [Node]          [Node]  
  //     /    \          /    \  
  // [Data0] [Data3] [Data1] [Data4] 
  data[3].mAabb.mMin += Vector3(0.001f); 
  data[3].mAabb.mMax -= Vector3(0.001f); 
  // Nothing should change, we only slightly shrunk the aabb 
  spatialPartition.UpdateData(keys[3], data[3]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
   
  // Update Data 4: 
  // Expected Tree:                  | Remove [Data4]:             | Re-insert [Data4]|              |          Balance Tree: 
  //             [Root]              |             [Root]          |                 [Root]          |             [Root]  
  //           /        \            |           /        \        |               /        \        |           /        \  
  //     [Node]          [Node]      |     [Node]          [Data1] |         [Node]          [Data1] |     [Node]          [Node]  
  //     /    \          /    \      |     /    \                  |         /    \                  |     /    \          /    \  
  // [Data0] [Data3] [Data1] [Data4] | [Data0] [Data3]             |     [Node]  [Data3]             | [Data0] [Data4] [Data3] [Data1] 
  //                                 |                             |     /    \                      | 
  //                                 |                             |[Data0]  [Data4]                 | 
  data[4].mAabb.mMin = data[0].mAabb.mMin + Vector3(0.1f); 
  data[4].mAabb.mMax = data[0].mAabb.mMax + Vector3(0.1f); 
  spatialPartition.UpdateData(keys[4], data[4]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructure4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(3); 
  data[0].mAabb = Aabb(Vector3(-4, -2, -5), Vector3(2, 4, 5)); 
  data[1].mAabb = Aabb(Vector3(-2, -2, -3), Vector3(4, 2, 3)); 
  data[2].mAabb = Aabb(Vector3(-1, -1, -1), Vector3(1, 1, 1)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
   
  // Test the edge case when both sides have the same surface area expansion
  // Expected Tree:  
  //             [Root]  
  //           /        \  
  //     [Data0]         [Node]  
  //                      /    \  
  //                [Data1] [Data2] 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

// Test for correct heuristic of smallest 'increase' in surface area
void DynamicAabbTreeStructure5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(3);
  data[0].mAabb = Aabb(Vector3(-2, -2, -2), Vector3(2, 2, 2));
  data[1].mAabb = Aabb(Vector3(3, -1, -1), Vector3(4, 1, 1));
  data[2].mAabb = Aabb(Vector3(1, -1, -1), Vector3(2, 1, 1));
  
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;
  
  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());
  
  DynamicAabbTree spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);
  
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeRayCastTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-6.25f, -0.5f, -3.75f), Vector3(-0.25f, 2, 0.25f)); 
  data[1].mAabb = Aabb(Vector3(-1.25f, -4.25f, 1.75f), Vector3(0.75f, -3.75f, 6.25f)); 
  data[2].mAabb = Aabb(Vector3(-6.75f, -10.25f, -1.25f), Vector3(-0.25f, -6.75f, -0.25f)); 
  data[3].mAabb = Aabb(Vector3(-0.25f, 5, -9.75f), Vector3(1.25f, 5.5f, -4.25f)); 
  data[4].mAabb = Aabb(Vector3(-6, -6.5f, 2), Vector3(-5, -4, 7.5f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition;
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  
  Ray ray;
  ray.mStart = Vector3(Vector3(-3, -1, 1.25f));
  ray.mDirection = Vector3(Vector3(-0.267261f, 0.801784f, -0.534522f));

  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(8);
  data[0].mAabb = Aabb(Vector3(2.25f, -8.5f, 3.5f), Vector3(7.25f, -5.5f, 5)); 
  data[1].mAabb = Aabb(Vector3(4, -0.25f, 0.5f), Vector3(5.5f, 5.75f, 1)); 
  data[2].mAabb = Aabb(Vector3(-2, 1.25f, 0.5f), Vector3(-2, 2.75f, 7.5f)); 
  data[3].mAabb = Aabb(Vector3(-2.25f, -3, 2.5f), Vector3(-1.75f, -0.5f, 2.5f)); 
  data[4].mAabb = Aabb(Vector3(-1.75f, -4.25f, -8), Vector3(-1.25f, -2.25f, -0.5f)); 
  data[5].mAabb = Aabb(Vector3(-0.5f, -1, -3.5f), Vector3(3.5f, 4.5f, 1)); 
  data[6].mAabb = Aabb(Vector3(3, -3.25f, 5.25f), Vector3(7.5f, -1.75f, 9.25f)); 
  data[7].mAabb = Aabb(Vector3(-7.5f, -4.5f, 2.5f), Vector3(-1.5f, -2, 6.5f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition;
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  
  Ray ray;
  ray.mStart = Vector3(Vector3(-1.5f, 3, -2));
  ray.mDirection = Vector3(Vector3(-0.534522f, -0.801784f, 0.267261f));

  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

// Simple results test
void DynamicAabbTreeFrustumCastTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-3.75f, 4, -5.25f), Vector3(-3.25f, 7.5f, 0.75f)); 
  data[1].mAabb = Aabb(Vector3(-0.25f, 1.75f, -0.25f), Vector3(6.25f, 2.75f, 1.75f)); 
  data[2].mAabb = Aabb(Vector3(-7.5f, -9.25f, 0.25f), Vector3(-2, -5.25f, 0.25f)); 
  data[3].mAabb = Aabb(Vector3(4.75f, -3.75f, -6.5f), Vector3(4.75f, 1.75f, -2)); 
  data[4].mAabb = Aabb(Vector3(1.25f, -1.25f, -4.25f), Vector3(2.25f, 3.25f, 0.75f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition;
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  
  Frustum frustum;
  frustum.Set(Vector3(-5.83007f, -13.0648f, -0.797914f), Vector3(11.8711f, -13.0648f, -0.797914f), Vector3(11.8711f, 4.03264f, -0.797914f), Vector3(-5.83007f, 4.03264f, -0.797914f), Vector3(1.08651f, -6.14827f, -6.19642f), Vector3(4.95449f, -6.14827f, -6.19642f), Vector3(4.95449f, -2.88394f, -6.19642f), Vector3(1.08651f, -2.88394f, -6.19642f));

  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-3.75f, 4, -5.25f), Vector3(-3.25f, 7.5f, 0.75f)); 
  data[1].mAabb = Aabb(Vector3(-0.25f, 1.75f, -0.25f), Vector3(6.25f, 2.75f, 1.75f)); 
  data[2].mAabb = Aabb(Vector3(-7.5f, -9.25f, 0.25f), Vector3(-2, -5.25f, 0.25f)); 
  data[3].mAabb = Aabb(Vector3(4.75f, -3.75f, -6.5f), Vector3(4.75f, 1.75f, -2)); 
  data[4].mAabb = Aabb(Vector3(1.25f, -1.25f, -4.25f), Vector3(2.25f, 3.25f, 0.75f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum fully contains root node
  frustum.Set(Vector3(10, -30, -30), Vector3(10, 30, -30), Vector3(10, 30, 30), Vector3(10, -30, 30), Vector3(-10, -10, -10), Vector3(-10, 10, -10), Vector3(-10, 10, 10), Vector3(-10, -10, 10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-3.75f, 4, -5.25f), Vector3(-3.25f, 7.5f, 0.75f)); 
  data[1].mAabb = Aabb(Vector3(-0.25f, 1.75f, -0.25f), Vector3(6.25f, 2.75f, 1.75f)); 
  data[2].mAabb = Aabb(Vector3(-7.5f, -9.25f, 0.25f), Vector3(-2, -5.25f, 0.25f)); 
  data[3].mAabb = Aabb(Vector3(4.75f, -3.75f, -6.5f), Vector3(4.75f, 1.75f, -2)); 
  data[4].mAabb = Aabb(Vector3(1.25f, -1.25f, -4.25f), Vector3(2.25f, 3.25f, 0.75f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum outside on the +y axis
  frustum.Set(Vector3(10, 20, -30), Vector3(10, 80, -30), Vector3(10, 80, 30), Vector3(10, 20, 30), Vector3(-10, 40, -10), Vector3(-10, 60, -10), Vector3(-10, 60, 10), Vector3(-10, 40, 10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-3.75f, 4, -5.25f), Vector3(-3.25f, 7.5f, 0.75f)); 
  data[1].mAabb = Aabb(Vector3(-0.25f, 1.75f, -0.25f), Vector3(6.25f, 2.75f, 1.75f)); 
  data[2].mAabb = Aabb(Vector3(-7.5f, -9.25f, 0.25f), Vector3(-2, -5.25f, 0.25f)); 
  data[3].mAabb = Aabb(Vector3(4.75f, -3.75f, -6.5f), Vector3(4.75f, 1.75f, -2)); 
  data[4].mAabb = Aabb(Vector3(1.25f, -1.25f, -4.25f), Vector3(2.25f, 3.25f, 0.75f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum outside on the -y axis
  frustum.Set(Vector3(10, -80, -30), Vector3(10, -20, -30), Vector3(10, -20, 30), Vector3(10, -80, 30), Vector3(-10, -60, -10), Vector3(-10, -40, -10), Vector3(-10, -40, 10), Vector3(-10, -60, 10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-3.75f, 4, -5.25f), Vector3(-3.25f, 7.5f, 0.75f)); 
  data[1].mAabb = Aabb(Vector3(-0.25f, 1.75f, -0.25f), Vector3(6.25f, 2.75f, 1.75f)); 
  data[2].mAabb = Aabb(Vector3(-7.5f, -9.25f, 0.25f), Vector3(-2, -5.25f, 0.25f)); 
  data[3].mAabb = Aabb(Vector3(4.75f, -3.75f, -6.5f), Vector3(4.75f, 1.75f, -2)); 
  data[4].mAabb = Aabb(Vector3(1.25f, -1.25f, -4.25f), Vector3(2.25f, 3.25f, 0.75f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum outside on the +x axis
  frustum.Set(Vector3(60, -30, -30), Vector3(60, 30, -30), Vector3(60, 30, 30), Vector3(60, -30, 30), Vector3(40, -10, -10), Vector3(40, 10, -10), Vector3(40, 10, 10), Vector3(40, -10, 10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-3.75f, 4, -5.25f), Vector3(-3.25f, 7.5f, 0.75f)); 
  data[1].mAabb = Aabb(Vector3(-0.25f, 1.75f, -0.25f), Vector3(6.25f, 2.75f, 1.75f)); 
  data[2].mAabb = Aabb(Vector3(-7.5f, -9.25f, 0.25f), Vector3(-2, -5.25f, 0.25f)); 
  data[3].mAabb = Aabb(Vector3(4.75f, -3.75f, -6.5f), Vector3(4.75f, 1.75f, -2)); 
  data[4].mAabb = Aabb(Vector3(1.25f, -1.25f, -4.25f), Vector3(2.25f, 3.25f, 0.75f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum outside on the -x axis
  frustum.Set(Vector3(-40, -30, -30), Vector3(-40, 30, -30), Vector3(-40, 30, 30), Vector3(-40, -30, 30), Vector3(-60, -10, -10), Vector3(-60, 10, -10), Vector3(-60, 10, 10), Vector3(-60, -10, 10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mAabb = Aabb(Vector3(-10.5f, 1.5f, -0.5f), Vector3(-9.5f, 2.5f, 0.5f)); 
  data[1].mAabb = Aabb(Vector3(9.5f, 1.5f, -0.5f), Vector3(10.5f, 2.5f, 0.5f)); 
  data[2].mAabb = Aabb(Vector3(-10.5f, -2.5f, -0.5f), Vector3(-9.5f, -1.5f, 0.5f)); 
  data[3].mAabb = Aabb(Vector3(9.5f, -2.5f, -0.5f), Vector3(10.5f, -1.5f, 0.5f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum contains left sub-tree
  frustum.Set(Vector3(-1.32615f, -11.3262f, 10), Vector3(21.3262f, -11.3262f, 10), Vector3(21.3262f, 11.3262f, 10), Vector3(-1.32615f, 11.3262f, 10), Vector3(8, -2, -10), Vector3(12, -2, -10), Vector3(12, 2, -10), Vector3(8, 2, -10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mAabb = Aabb(Vector3(-10.5f, 1.5f, -0.5f), Vector3(-9.5f, 2.5f, 0.5f)); 
  data[1].mAabb = Aabb(Vector3(9.5f, 1.5f, -0.5f), Vector3(10.5f, 2.5f, 0.5f)); 
  data[2].mAabb = Aabb(Vector3(-10.5f, -2.5f, -0.5f), Vector3(-9.5f, -1.5f, 0.5f)); 
  data[3].mAabb = Aabb(Vector3(9.5f, -2.5f, -0.5f), Vector3(10.5f, -1.5f, 0.5f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum contains right sub-tree
  frustum.Set(Vector3(-21.3262f, -11.3262f, 10), Vector3(1.32615f, -11.3262f, 10), Vector3(1.32615f, 11.3262f, 10), Vector3(-21.3262f, 11.3262f, 10), Vector3(-12, -2, -10), Vector3(-8, -2, -10), Vector3(-8, 2, -10), Vector3(-12, 2, -10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mAabb = Aabb(Vector3(-10.5f, 1.5f, -0.5f), Vector3(-9.5f, 2.5f, 0.5f)); 
  data[1].mAabb = Aabb(Vector3(9.5f, 1.5f, -0.5f), Vector3(10.5f, 2.5f, 0.5f)); 
  data[2].mAabb = Aabb(Vector3(-10.5f, -2.5f, -0.5f), Vector3(-9.5f, -1.5f, 0.5f)); 
  data[3].mAabb = Aabb(Vector3(9.5f, -2.5f, -0.5f), Vector3(10.5f, -1.5f, 0.5f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum contains bottom-left aabb
  frustum.Set(Vector3(-1.32615f, -18.3262f, 10), Vector3(21.3262f, -18.3262f, 10), Vector3(21.3262f, 4.32615f, 10), Vector3(-1.32615f, 4.32615f, 10), Vector3(8, -9, -10), Vector3(12, -9, -10), Vector3(12, -5, -10), Vector3(8, -5, -10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mAabb = Aabb(Vector3(-10.5f, 1.5f, -0.5f), Vector3(-9.5f, 2.5f, 0.5f)); 
  data[1].mAabb = Aabb(Vector3(9.5f, 1.5f, -0.5f), Vector3(10.5f, 2.5f, 0.5f)); 
  data[2].mAabb = Aabb(Vector3(-10.5f, -2.5f, -0.5f), Vector3(-9.5f, -1.5f, 0.5f)); 
  data[3].mAabb = Aabb(Vector3(9.5f, -2.5f, -0.5f), Vector3(10.5f, -1.5f, 0.5f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum contains top-right aabb
  frustum.Set(Vector3(-21.3262f, -4.32615f, 10), Vector3(1.32615f, -4.32615f, 10), Vector3(1.32615f, 18.3262f, 10), Vector3(-21.3262f, 18.3262f, 10), Vector3(-12, 5, -10), Vector3(-8, 5, -10), Vector3(-8, 9, -10), Vector3(-12, 9, -10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

// Check that the correct number of tests are performed
void DynamicAabbTreeFrustumCastTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data;
  data.resize(4);
  data[0].mAabb = Aabb(Vector3(-10.5f, 1.5f, -0.5f), Vector3(-9.5f, 2.5f, 0.5f)); 
  data[1].mAabb = Aabb(Vector3(9.5f, 1.5f, -0.5f), Vector3(10.5f, 2.5f, 0.5f)); 
  data[2].mAabb = Aabb(Vector3(-10.5f, -2.5f, -0.5f), Vector3(-9.5f, -1.5f, 0.5f)); 
  data[3].mAabb = Aabb(Vector3(9.5f, -2.5f, -0.5f), Vector3(10.5f, -1.5f, 0.5f)); 

  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;

  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 

  CastResults castResults; 
  Frustum frustum;

  // Frustum hits top aabb but is rejected after 1 test on both children
  frustum.Set(Vector3(-11.3262f, -11.3262f, 10), Vector3(11.3262f, -11.3262f, 10), Vector3(11.3262f, 11.3262f, 10), Vector3(-11.3262f, 11.3262f, 10), Vector3(-2, -2, -10), Vector3(2, -2, -10), Vector3(2, 2, -10), Vector3(-2, 2, -10));
  PrintAabbTreeFrustumCastTest(spatialPartition, frustum, Application::mStatistics.mPlaneAabbTests, file);
}

void DynamicAabbTreeSelfQuery1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Application::mStatistics.Clear();
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(1.75f, 0.75f, -0.25f), Vector3(2.25f, 2.25f, 0.75f));
  data[1].mAabb = Aabb(Vector3(-6, 5.75f, -6), Vector3(-1.5f, 6.75f, -5.5f));
  data[2].mAabb = Aabb(Vector3(-8.25f, -1.25f, 1), Vector3(-3.25f, -0.25f, 3));
  data[3].mAabb = Aabb(Vector3(2.5f, 4, -6.5f), Vector3(6.5f, 5, -2.5f));
  data[4].mAabb = Aabb(Vector3(-1, 5.25f, 1), Vector3(4, 8.75f, 1.5f));
  
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;
  
  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());
  
  DynamicAabbTree spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);
  
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void DynamicAabbTreeSelfQuery2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  Application::mStatistics.Clear();
  std::vector<SpatialPartitionData> data;
  data.resize(5);
  data[0].mAabb = Aabb(Vector3(-0.75f, 1, 0.25f), Vector3(-0.75f, 2, 7.25f));
  data[1].mAabb = Aabb(Vector3(8, -4, -5.75f), Vector3(8.5f, 0, -0.75f));
  data[2].mAabb = Aabb(Vector3(5.25f, -1.25f, 3.25f), Vector3(6.75f, -0.25f, 10.25f));
  data[3].mAabb = Aabb(Vector3(-2.5f, -3.75f, 0.5f), Vector3(1, 0.75f, 4.5f));
  data[4].mAabb = Aabb(Vector3(1.5f, -4, -4), Vector3(6, 0, -2.5f));
  
  for(size_t i = 0; i < data.size(); ++i)
    data[i].mClientData = (void*)i;
  
  std::vector<SpatialPartitionKey> keys;
  keys.resize(data.size());
  
  DynamicAabbTree spatialPartition;
  for(size_t i = 0; i < data.size(); ++i)
    spatialPartition.InsertData(keys[i], data[i]);
  
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void DynamicAabbTreeStructureFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(8.24045f, 62.115f, -47.514f), Vector3(15.9862f, 65.7863f, -44.1914f)); 
  data[1].mAabb = Aabb(Vector3(6.94307f, -46.9453f, 20.3042f), Vector3(13.8133f, -40.5835f, 20.7304f)); 
  data[2].mAabb = Aabb(Vector3(-46.2474f, -15.2402f, 7.74045f), Vector3(-42.1311f, -10.5553f, 10.4371f)); 
  data[3].mAabb = Aabb(Vector3(-1.49339f, 14.5689f, -55.154f), Vector3(0.688606f, 16.8036f, -47.639f)); 
  data[4].mAabb = Aabb(Vector3(11.9187f, -23.0718f, 16.0709f), Vector3(12.9252f, -18.8074f, 16.6068f)); 
  data[5].mAabb = Aabb(Vector3(20.5042f, -55.8954f, -28.6125f), Vector3(22.9564f, -50.7408f, -24.6745f)); 
  data[6].mAabb = Aabb(Vector3(-12.3818f, 19.2744f, 90.7911f), Vector3(-5.21498f, 26.4531f, 98.067f)); 
  data[7].mAabb = Aabb(Vector3(-7.12306f, -13.6493f, 30.583f), Vector3(0.206861f, -9.29608f, 31.0415f)); 
  data[8].mAabb = Aabb(Vector3(-36.3659f, 70.3832f, -13.7931f), Vector3(-33.9749f, 78.132f, -6.75214f)); 
  data[9].mAabb = Aabb(Vector3(37.1108f, 27.7927f, 36.2366f), Vector3(44.0887f, 33.0248f, 40.0375f)); 
  data[10].mAabb = Aabb(Vector3(-43.6403f, -31.8124f, 73.4688f), Vector3(-40.2241f, -27.809f, 75.0971f)); 
  data[11].mAabb = Aabb(Vector3(-67.842f, 1.63719f, 23.473f), Vector3(-61.1008f, 9.2429f, 24.6477f)); 
  data[12].mAabb = Aabb(Vector3(21.4079f, -39.9807f, -46.173f), Vector3(28.8097f, -33.2945f, -44.633f)); 
  data[13].mAabb = Aabb(Vector3(56.7571f, 51.9024f, 38.0519f), Vector3(60.614f, 58.1243f, 45.2107f)); 
  data[14].mAabb = Aabb(Vector3(15.0127f, -42.5641f, -56.8148f), Vector3(15.8713f, -38.6425f, -51.1836f)); 
  data[15].mAabb = Aabb(Vector3(24.1455f, -7.16797f, -96.5728f), Vector3(31.3103f, -0.488642f, -92.9693f)); 
  data[16].mAabb = Aabb(Vector3(-45.3146f, 29.4351f, 9.04092f), Vector3(-40.0293f, 35.1918f, 16.1765f)); 
  data[17].mAabb = Aabb(Vector3(91.5668f, -19.2772f, -21.4254f), Vector3(96.1515f, -18.8312f, -19.204f)); 
  data[18].mAabb = Aabb(Vector3(30.5041f, -11.5053f, -37.8216f), Vector3(35.8785f, -3.71639f, -30.9901f)); 
  data[19].mAabb = Aabb(Vector3(8.69988f, -68.0967f, 59.2551f), Vector3(16.0196f, -61.9968f, 66.5741f)); 
  data[20].mAabb = Aabb(Vector3(-29.6441f, 56.1396f, -1.5831f), Vector3(-24.9901f, 61.8343f, 1.1268f)); 
  data[21].mAabb = Aabb(Vector3(-1.4562f, 7.3045f, -86.5713f), Vector3(5.69351f, 13.8015f, -84.6328f)); 
  data[22].mAabb = Aabb(Vector3(-53.7418f, -34.2637f, -74.1035f), Vector3(-52.5175f, -27.9963f, -68.7971f)); 
  data[23].mAabb = Aabb(Vector3(-32.1644f, 34.8103f, -76.9026f), Vector3(-25.9865f, 37.7939f, -76.0283f)); 
  data[24].mAabb = Aabb(Vector3(-34.2149f, -0.504411f, 7.5752f), Vector3(-27.361f, 6.67661f, 10.0365f)); 
  data[25].mAabb = Aabb(Vector3(-45.6392f, 54.8391f, 59.0832f), Vector3(-44.6482f, 56.0036f, 64.1003f)); 
  data[26].mAabb = Aabb(Vector3(13.7112f, 18.5719f, -1.46693f), Vector3(14.6068f, 23.6625f, 4.80279f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[12].mAabb = Aabb(Vector3(90.5011f, -15.8833f, 10.8627f), Vector3(95.3386f, -12.796f, 15.1435f)); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[6].mAabb = Aabb(Vector3(-71.0922f, -10.1138f, -17.8716f), Vector3(-67.5998f, -2.79505f, -15.79f)); 
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[24]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[21]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[15].mAabb = Aabb(Vector3(57.5693f, 51.5211f, -14.7613f), Vector3(58.8734f, 57.9713f, -12.9887f)); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[18].mAabb = Aabb(Vector3(-39.8735f, 52.7382f, -61.7444f), Vector3(-33.3788f, 58.6939f, -55.1094f)); 
  spatialPartition.UpdateData(keys[18], data[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(17); 
  data[0].mAabb = Aabb(Vector3(24.2093f, -48.5264f, -12.1674f), Vector3(29.0767f, -45.8177f, -6.18137f)); 
  data[1].mAabb = Aabb(Vector3(8.09623f, 39.633f, -26.0011f), Vector3(9.8779f, 41.4794f, -22.8199f)); 
  data[2].mAabb = Aabb(Vector3(-13.7559f, -54.0117f, -55.2214f), Vector3(-12.6037f, -51.1542f, -54.6461f)); 
  data[3].mAabb = Aabb(Vector3(-41.5073f, 28.0387f, -30.2351f), Vector3(-36.3094f, 31.8327f, -29.4227f)); 
  data[4].mAabb = Aabb(Vector3(22.9301f, 23.9374f, 45.3939f), Vector3(29.8824f, 25.0968f, 48.8883f)); 
  data[5].mAabb = Aabb(Vector3(-42.0353f, 12.2358f, -38.1757f), Vector3(-35.4302f, 14.3651f, -32.9883f)); 
  data[6].mAabb = Aabb(Vector3(64.7467f, -66.3985f, 29.1604f), Vector3(69.5097f, -61.7988f, 34.1363f)); 
  data[7].mAabb = Aabb(Vector3(40.3213f, -91.2848f, -5.24621f), Vector3(45.5032f, -86.1694f, -4.52938f)); 
  data[8].mAabb = Aabb(Vector3(-36.878f, 65.2636f, -42.9239f), Vector3(-34.4065f, 72.723f, -40.3738f)); 
  data[9].mAabb = Aabb(Vector3(-79.2414f, 6.64234f, -52.3394f), Vector3(-78.8208f, 13.7155f, -45.2922f)); 
  data[10].mAabb = Aabb(Vector3(30.2165f, -40.7649f, 12.2528f), Vector3(34.8683f, -36.4846f, 19.0495f)); 
  data[11].mAabb = Aabb(Vector3(-55.3939f, -26.8483f, 35.3602f), Vector3(-49.3337f, -24.2182f, 40.3222f)); 
  data[12].mAabb = Aabb(Vector3(-20.5007f, 40.2268f, 53.2456f), Vector3(-17.2112f, 45.5674f, 54.8213f)); 
  data[13].mAabb = Aabb(Vector3(45.2278f, -60.0963f, -44.3205f), Vector3(46.8587f, -54.3601f, -36.7289f)); 
  data[14].mAabb = Aabb(Vector3(63.8209f, 42.8864f, -57.7067f), Vector3(67.6736f, 49.7828f, -55.7643f)); 
  data[15].mAabb = Aabb(Vector3(39.9094f, 19.226f, -72.7232f), Vector3(46.5342f, 24.0282f, -68.685f)); 
  data[16].mAabb = Aabb(Vector3(-48.0825f, -30.268f, -76.5983f), Vector3(-40.4701f, -28.805f, -69.4356f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[14].mAabb = Aabb(Vector3(40.6939f, -51.3168f, -2.81437f), Vector3(47.9322f, -46.9443f, 0.613156f)); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[8].mAabb = Aabb(Vector3(-41.531f, 60.6624f, 52.844f), Vector3(-40.5101f, 61.9252f, 59.2281f)); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[10].mAabb = Aabb(Vector3(41.1822f, 63.1537f, -21.875f), Vector3(43.4331f, 65.215f, -17.5912f)); 
  spatialPartition.UpdateData(keys[10], data[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(15); 
  data[0].mAabb = Aabb(Vector3(30.559f, 36.4406f, -2.95011f), Vector3(32.937f, 37.3873f, 0.699847f)); 
  data[1].mAabb = Aabb(Vector3(-34.6134f, 0.836478f, 91.6042f), Vector3(-28.9319f, 7.57101f, 96.577f)); 
  data[2].mAabb = Aabb(Vector3(-49.2353f, 4.84733f, -2.38213f), Vector3(-47.5325f, 11.2567f, 0.0253148f)); 
  data[3].mAabb = Aabb(Vector3(31.1707f, -50.3859f, 47.3293f), Vector3(33.8217f, -49.9205f, 47.7657f)); 
  data[4].mAabb = Aabb(Vector3(13.5674f, 57.9842f, -51.6496f), Vector3(17.8317f, 65.1332f, -48.1887f)); 
  data[5].mAabb = Aabb(Vector3(8.65814f, -63.8997f, -47.0865f), Vector3(11.112f, -62.7983f, -40.9295f)); 
  data[6].mAabb = Aabb(Vector3(44.2342f, 33.5735f, 11.2785f), Vector3(45.337f, 34.8917f, 14.8094f)); 
  data[7].mAabb = Aabb(Vector3(33.5497f, -16.3875f, 56.9315f), Vector3(38.6201f, -11.2063f, 58.2142f)); 
  data[8].mAabb = Aabb(Vector3(37.7682f, 39.1671f, 8.31844f), Vector3(38.9566f, 40.6691f, 10.496f)); 
  data[9].mAabb = Aabb(Vector3(13.7921f, 81.9249f, -49.1852f), Vector3(17.0561f, 89.362f, -46.4433f)); 
  data[10].mAabb = Aabb(Vector3(42.4605f, 72.9139f, -36.411f), Vector3(49.6156f, 76.88f, -34.5096f)); 
  data[11].mAabb = Aabb(Vector3(-42.9902f, 2.36151f, -58.146f), Vector3(-40.9655f, 8.48441f, -55.44f)); 
  data[12].mAabb = Aabb(Vector3(-12.7917f, 50.8679f, -86.1257f), Vector3(-4.94061f, 57.1223f, -78.5237f)); 
  data[13].mAabb = Aabb(Vector3(-7.32825f, -7.25127f, 9.18098f), Vector3(-5.82143f, -3.40649f, 12.3304f)); 
  data[14].mAabb = Aabb(Vector3(28.9432f, 14.4068f, 59.15f), Vector3(36.3645f, 20.8006f, 64.365f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(15); 
  data[0].mAabb = Aabb(Vector3(-5.48084f, 48.1453f, 7.98256f), Vector3(0.70956f, 51.808f, 15.3048f)); 
  data[1].mAabb = Aabb(Vector3(-68.4142f, 26.1662f, -71.5946f), Vector3(-61.6367f, 29.0533f, -64.1666f)); 
  data[2].mAabb = Aabb(Vector3(66.7966f, 28.6055f, 19.9717f), Vector3(71.7763f, 33.379f, 20.7055f)); 
  data[3].mAabb = Aabb(Vector3(8.28054f, 33.5332f, -30.7463f), Vector3(12.5813f, 36.6762f, -25.1416f)); 
  data[4].mAabb = Aabb(Vector3(61.761f, 19.4797f, 38.8883f), Vector3(68.8177f, 26.9474f, 45.4818f)); 
  data[5].mAabb = Aabb(Vector3(33.8571f, -30.2346f, 34.8886f), Vector3(40.5819f, -27.0268f, 37.7498f)); 
  data[6].mAabb = Aabb(Vector3(52.6532f, 75.9682f, 20.3933f), Vector3(53.2255f, 83.3259f, 22.4559f)); 
  data[7].mAabb = Aabb(Vector3(-5.0591f, 2.90973f, -7.54408f), Vector3(0.333889f, 5.71125f, -0.962863f)); 
  data[8].mAabb = Aabb(Vector3(-44.4733f, 25.7259f, 72.753f), Vector3(-39.0738f, 26.1549f, 76.1355f)); 
  data[9].mAabb = Aabb(Vector3(28.5206f, 12.7997f, 90.6403f), Vector3(29.0335f, 16.4141f, 91.5979f)); 
  data[10].mAabb = Aabb(Vector3(-31.0141f, 19.5009f, -89.9918f), Vector3(-25.8618f, 23.124f, -83.0176f)); 
  data[11].mAabb = Aabb(Vector3(-28.4285f, 44.8082f, -17.9983f), Vector3(-21.6488f, 51.4924f, -14.6833f)); 
  data[12].mAabb = Aabb(Vector3(3.66804f, -95.0792f, -23.2818f), Vector3(10.0744f, -89.6473f, -19.5815f)); 
  data[13].mAabb = Aabb(Vector3(32.5523f, -46.2267f, 8.22566f), Vector3(36.7042f, -39.1141f, 15.0295f)); 
  data[14].mAabb = Aabb(Vector3(-72.5721f, 32.6495f, -29.4976f), Vector3(-66.1969f, 33.7688f, -24.2387f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[9].mAabb = Aabb(Vector3(39.8503f, 7.59881f, 61.4618f), Vector3(40.7232f, 11.8226f, 62.4347f)); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[4].mAabb = Aabb(Vector3(-22.3169f, 82.3097f, -39.0847f), Vector3(-19.1408f, 83.2274f, -34.7327f)); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(-0.564432f, -86.6929f, 7.97906f), Vector3(5.13912f, -85.8325f, 12.5535f)); 
  data[1].mAabb = Aabb(Vector3(-89.5084f, 31.0096f, -16.9959f), Vector3(-87.4978f, 35.7939f, -13.2757f)); 
  data[2].mAabb = Aabb(Vector3(27.422f, -17.5208f, 22.1996f), Vector3(31.6618f, -12.0008f, 27.0575f)); 
  data[3].mAabb = Aabb(Vector3(-9.60416f, 24.2943f, -52.8819f), Vector3(-1.63733f, 29.1205f, -48.4058f)); 
  data[4].mAabb = Aabb(Vector3(17.8162f, -76.437f, 52.3541f), Vector3(21.4116f, -73.6457f, 58.5861f)); 
  data[5].mAabb = Aabb(Vector3(-19.1047f, 72.8631f, 25.5558f), Vector3(-12.4024f, 77.1303f, 31.2128f)); 
  data[6].mAabb = Aabb(Vector3(4.01688f, -69.1483f, 70.6127f), Vector3(7.85702f, -63.5939f, 73.6053f)); 
  data[7].mAabb = Aabb(Vector3(-10.2896f, -17.3912f, -90.7099f), Vector3(-2.48604f, -16.7418f, -82.7714f)); 
  data[8].mAabb = Aabb(Vector3(27.8122f, -51.0328f, -6.65364f), Vector3(30.0972f, -46.8842f, -1.55058f)); 
  data[9].mAabb = Aabb(Vector3(30.8023f, -78.9529f, -34.93f), Vector3(37.4492f, -74.2014f, -27.672f)); 
  data[10].mAabb = Aabb(Vector3(52.3057f, 39.1028f, -45.547f), Vector3(59.2285f, 40.2058f, -40.2938f)); 
  data[11].mAabb = Aabb(Vector3(-36.0277f, -55.2756f, -76.7338f), Vector3(-33.1037f, -48.4086f, -69.1023f)); 
  data[12].mAabb = Aabb(Vector3(13.2186f, -27.2139f, 84.8979f), Vector3(19.9947f, -19.2729f, 91.397f)); 
  data[13].mAabb = Aabb(Vector3(-49.4658f, 64.8842f, 21.2703f), Vector3(-48.3355f, 69.6707f, 22.123f)); 
  data[14].mAabb = Aabb(Vector3(-36.4849f, 69.8232f, -30.4639f), Vector3(-33.0671f, 72.6932f, -22.8235f)); 
  data[15].mAabb = Aabb(Vector3(-6.34691f, -53.5779f, -33.1565f), Vector3(1.63523f, -50.5779f, -31.0147f)); 
  data[16].mAabb = Aabb(Vector3(-16.1941f, -31.1626f, 20.5783f), Vector3(-9.66969f, -27.1907f, 27.4789f)); 
  data[17].mAabb = Aabb(Vector3(44.4346f, -21.3835f, 9.58275f), Vector3(51.9551f, -18.5546f, 16.1802f)); 
  data[18].mAabb = Aabb(Vector3(-17.9965f, 17.2053f, 19.3897f), Vector3(-11.4721f, 23.1095f, 24.062f)); 
  data[19].mAabb = Aabb(Vector3(41.8344f, 73.3269f, 40.1515f), Vector3(46.7067f, 80.3253f, 41.2757f)); 
  data[20].mAabb = Aabb(Vector3(-0.924f, 39.0892f, 23.568f), Vector3(1.08381f, 42.2157f, 28.6579f)); 
  data[21].mAabb = Aabb(Vector3(29.6305f, -59.9491f, -26.1817f), Vector3(35.0523f, -54.7786f, -23.0891f)); 
  data[22].mAabb = Aabb(Vector3(52.2017f, 35.2144f, -18.9257f), Vector3(55.9567f, 42.4773f, -14.1348f)); 
  data[23].mAabb = Aabb(Vector3(83.3663f, 26.9041f, 20.3945f), Vector3(88.5288f, 28.8442f, 22.3889f)); 
  data[24].mAabb = Aabb(Vector3(-10.8062f, 25.6065f, -4.82779f), Vector3(-8.95816f, 26.019f, 1.80515f)); 
  data[25].mAabb = Aabb(Vector3(-36.2437f, -13.3242f, -83.7919f), Vector3(-32.8574f, -12.7097f, -80.4356f)); 
  data[26].mAabb = Aabb(Vector3(-62.6779f, -40.9316f, 27.1209f), Vector3(-54.9465f, -35.3058f, 28.2415f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[16].mAabb = Aabb(Vector3(54.4937f, -35.1651f, 25.1799f), Vector3(61.0079f, -33.8408f, 31.6468f)); 
  spatialPartition.UpdateData(keys[16], data[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[26]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(-77.1628f, -43.9612f, -50.9398f), Vector3(-72.8836f, -37.2076f, -47.7733f)); 
  data[1].mAabb = Aabb(Vector3(-62.6064f, -60.9336f, -28.8104f), Vector3(-55.4802f, -58.3879f, -21.0716f)); 
  data[2].mAabb = Aabb(Vector3(-24.3715f, -39.1157f, -22.0576f), Vector3(-16.3731f, -31.5448f, -15.481f)); 
  data[3].mAabb = Aabb(Vector3(-45.8496f, 53.6824f, 35.2422f), Vector3(-41.341f, 59.9231f, 37.35f)); 
  data[4].mAabb = Aabb(Vector3(24.8845f, 90.3325f, 16.5911f), Vector3(26.6812f, 94.3079f, 20.9745f)); 
  data[5].mAabb = Aabb(Vector3(-11.7816f, -92.7925f, 31.3877f), Vector3(-7.11247f, -87.7604f, 38.2192f)); 
  data[6].mAabb = Aabb(Vector3(4.45665f, -19.2742f, 31.2828f), Vector3(11.3392f, -14.4351f, 38.3752f)); 
  data[7].mAabb = Aabb(Vector3(15.2949f, -40.3727f, -74.3548f), Vector3(18.9124f, -39.966f, -67.2547f)); 
  data[8].mAabb = Aabb(Vector3(-75.6053f, 33.5775f, 52.1088f), Vector3(-68.0133f, 40.73f, 54.8324f)); 
  data[9].mAabb = Aabb(Vector3(16.4982f, 3.26238f, -2.38536f), Vector3(23.3919f, 10.8602f, -0.123113f)); 
  data[10].mAabb = Aabb(Vector3(6.97456f, 75.449f, 0.638736f), Vector3(14.5647f, 76.7322f, 5.32755f)); 
  data[11].mAabb = Aabb(Vector3(45.9897f, -67.5809f, 9.92464f), Vector3(51.0401f, -61.0006f, 13.6099f)); 
  data[12].mAabb = Aabb(Vector3(75.2039f, -28.102f, 50.4261f), Vector3(80.0609f, -21.0381f, 57.6032f)); 
  data[13].mAabb = Aabb(Vector3(-32.846f, -58.5864f, -65.5784f), Vector3(-28.7104f, -56.9061f, -62.3972f)); 
  data[14].mAabb = Aabb(Vector3(-41.1469f, -38.696f, 19.8064f), Vector3(-34.3202f, -34.074f, 25.3193f)); 
  data[15].mAabb = Aabb(Vector3(-27.266f, 37.1189f, 57.8591f), Vector3(-20.8344f, 39.5262f, 60.8211f)); 
  data[16].mAabb = Aabb(Vector3(12.5892f, -73.6936f, -56.6802f), Vector3(18.2315f, -66.0262f, -49.5331f)); 
  data[17].mAabb = Aabb(Vector3(-56.9519f, -57.6723f, -60.4531f), Vector3(-49.2379f, -49.7514f, -56.1716f)); 
  data[18].mAabb = Aabb(Vector3(-12.6324f, -1.18934f, -5.6319f), Vector3(-8.14378f, 5.1567f, -4.88909f)); 
  data[19].mAabb = Aabb(Vector3(-20.7145f, -53.8845f, -34.9254f), Vector3(-15.6173f, -48.534f, -27.1931f)); 
  data[20].mAabb = Aabb(Vector3(1.21454f, -40.2864f, -61.8506f), Vector3(8.5774f, -35.7113f, -55.2281f)); 
  data[21].mAabb = Aabb(Vector3(-72.4423f, -8.18314f, 11.1929f), Vector3(-66.7181f, -4.60787f, 17.1497f)); 
  data[22].mAabb = Aabb(Vector3(-25.7138f, 29.4634f, -71.4415f), Vector3(-23.2651f, 30.0344f, -70.2084f)); 
  data[23].mAabb = Aabb(Vector3(26.5247f, -69.9716f, -20.2092f), Vector3(34.0896f, -63.6945f, -13.6749f)); 
  data[24].mAabb = Aabb(Vector3(22.1031f, -69.7328f, 50.8252f), Vector3(25.5596f, -63.451f, 56.9228f)); 
  data[25].mAabb = Aabb(Vector3(-14.6518f, -95.0882f, -7.37838f), Vector3(-11.2658f, -88.4068f, -2.36832f)); 
  data[26].mAabb = Aabb(Vector3(67.4116f, 1.85475f, 65.2729f), Vector3(71.6991f, 9.49385f, 69.9828f)); 
  data[27].mAabb = Aabb(Vector3(-56.8079f, -52.8405f, 42.1726f), Vector3(-54.8534f, -46.2431f, 48.7977f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[5].mAabb = Aabb(Vector3(51.1395f, 74.3956f, -18.5121f), Vector3(52.6502f, 76.181f, -13.8813f)); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[24].mAabb = Aabb(Vector3(-22.8408f, 50.8327f, 2.91181f), Vector3(-17.2523f, 58.1913f, 9.86691f)); 
  spatialPartition.UpdateData(keys[24], data[24]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[18].mAabb = Aabb(Vector3(-32.4542f, -90.3279f, -25.2733f), Vector3(-30.7342f, -86.7206f, -24.6033f)); 
  spatialPartition.UpdateData(keys[18], data[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(-32.7617f, -29.9568f, -5.78515f), Vector3(-30.8536f, -23.0956f, 0.72231f)); 
  data[1].mAabb = Aabb(Vector3(-74.6574f, -46.2886f, -30.0655f), Vector3(-72.0955f, -45.6233f, -26.6245f)); 
  data[2].mAabb = Aabb(Vector3(-39.1225f, -24.144f, -13.8745f), Vector3(-32.0113f, -16.2264f, -7.10986f)); 
  data[3].mAabb = Aabb(Vector3(-53.3008f, 31.2685f, 3.83817f), Vector3(-46.5263f, 33.2979f, 9.49905f)); 
  data[4].mAabb = Aabb(Vector3(-84.0642f, -11.8908f, 16.7727f), Vector3(-82.5784f, -10.5475f, 19.0686f)); 
  data[5].mAabb = Aabb(Vector3(-21.2746f, 54.0579f, -74.9678f), Vector3(-13.562f, 56.3626f, -74.5477f)); 
  data[6].mAabb = Aabb(Vector3(-9.39165f, -64.8284f, -54.7668f), Vector3(-6.03626f, -60.106f, -47.0739f)); 
  data[7].mAabb = Aabb(Vector3(-28.6552f, -78.1971f, -46.7053f), Vector3(-27.6691f, -76.3347f, -40.7033f)); 
  data[8].mAabb = Aabb(Vector3(-60.341f, -11.2406f, -44.9823f), Vector3(-56.8917f, -8.66217f, -39.5341f)); 
  data[9].mAabb = Aabb(Vector3(16.9133f, 83.1802f, -23.7052f), Vector3(24.2553f, 88.229f, -20.2941f)); 
  data[10].mAabb = Aabb(Vector3(52.5938f, -60.2647f, 8.91046f), Vector3(59.2218f, -57.6139f, 16.8905f)); 
  data[11].mAabb = Aabb(Vector3(23.2704f, -14.8403f, -53.4911f), Vector3(24.051f, -7.95964f, -51.5517f)); 
  data[12].mAabb = Aabb(Vector3(39.8418f, 15.6234f, -13.2998f), Vector3(43.4024f, 21.5357f, -7.2722f)); 
  data[13].mAabb = Aabb(Vector3(-40.7454f, 62.4687f, 54.6955f), Vector3(-37.2102f, 66.0319f, 57.5524f)); 
  data[14].mAabb = Aabb(Vector3(-18.9938f, 14.9308f, 8.44906f), Vector3(-13.8768f, 16.1743f, 11.1267f)); 
  data[15].mAabb = Aabb(Vector3(42.8737f, 24.829f, -79.9442f), Vector3(46.6412f, 27.6778f, -78.3515f)); 
  data[16].mAabb = Aabb(Vector3(36.012f, -23.9997f, 40.9447f), Vector3(38.4489f, -23.3241f, 42.6362f)); 
  data[17].mAabb = Aabb(Vector3(61.5193f, 28.9683f, -45.4402f), Vector3(63.1704f, 35.4092f, -40.0725f)); 
  data[18].mAabb = Aabb(Vector3(-39.6375f, -23.2747f, -69.1014f), Vector3(-38.9984f, -17.5641f, -64.1159f)); 
  data[19].mAabb = Aabb(Vector3(41.9665f, 10.8059f, -26.8002f), Vector3(48.864f, 12.0671f, -25.3562f)); 
  data[20].mAabb = Aabb(Vector3(-84.5361f, -23.2731f, 46.2928f), Vector3(-80.2028f, -19.0354f, 48.2295f)); 
  data[21].mAabb = Aabb(Vector3(38.3824f, 52.1039f, 63.4614f), Vector3(42.9757f, 59.8963f, 68.412f)); 
  data[22].mAabb = Aabb(Vector3(77.2949f, -41.9341f, 15.3385f), Vector3(81.3907f, -39.3798f, 20.7088f)); 
  data[23].mAabb = Aabb(Vector3(21.9573f, -64.0695f, 49.7769f), Vector3(23.1638f, -62.1651f, 57.2246f)); 
  data[24].mAabb = Aabb(Vector3(38.964f, 69.593f, 39.0646f), Vector3(40.6385f, 71.5978f, 41.958f)); 
  data[25].mAabb = Aabb(Vector3(24.0637f, 28.0354f, -31.7445f), Vector3(24.493f, 29.6192f, -30.9491f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[5].mAabb = Aabb(Vector3(-5.62802f, 79.4506f, 46.5787f), Vector3(-4.55702f, 84.4008f, 50.8658f)); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(17); 
  data[0].mAabb = Aabb(Vector3(0.746055f, 45.0971f, -84.6615f), Vector3(3.0039f, 47.4257f, -77.8805f)); 
  data[1].mAabb = Aabb(Vector3(-37.6969f, 49.4008f, 20.7175f), Vector3(-33.0041f, 54.6375f, 21.172f)); 
  data[2].mAabb = Aabb(Vector3(58.7446f, 41.818f, -56.3845f), Vector3(60.3948f, 49.5381f, -53.1019f)); 
  data[3].mAabb = Aabb(Vector3(-42.832f, 60.2393f, 12.189f), Vector3(-39.85f, 64.3886f, 15.7884f)); 
  data[4].mAabb = Aabb(Vector3(-13.2185f, 1.26649f, 21.524f), Vector3(-9.70982f, 8.39114f, 25.7684f)); 
  data[5].mAabb = Aabb(Vector3(12.8648f, -45.6462f, 77.5377f), Vector3(20.6327f, -43.824f, 82.8866f)); 
  data[6].mAabb = Aabb(Vector3(24.2711f, 43.3515f, -30.0179f), Vector3(25.1477f, 48.5812f, -27.6624f)); 
  data[7].mAabb = Aabb(Vector3(4.04016f, -54.0912f, -3.32822f), Vector3(6.76166f, -47.3464f, 3.39982f)); 
  data[8].mAabb = Aabb(Vector3(-12.225f, 21.045f, 3.79659f), Vector3(-8.32825f, 26.4034f, 7.99137f)); 
  data[9].mAabb = Aabb(Vector3(-78.6893f, 2.31708f, 49.0722f), Vector3(-71.4895f, 4.42602f, 55.6458f)); 
  data[10].mAabb = Aabb(Vector3(11.1126f, -41.5316f, 80.6761f), Vector3(11.6836f, -36.8922f, 82.644f)); 
  data[11].mAabb = Aabb(Vector3(-73.3094f, -16.8145f, -56.7198f), Vector3(-71.0031f, -9.74039f, -53.7243f)); 
  data[12].mAabb = Aabb(Vector3(53.411f, -46.0083f, -61.5623f), Vector3(56.3273f, -39.416f, -54.5531f)); 
  data[13].mAabb = Aabb(Vector3(-26.9548f, 75.9511f, -1.78142f), Vector3(-23.4464f, 78.3217f, 4.8281f)); 
  data[14].mAabb = Aabb(Vector3(61.3277f, -69.5951f, 20.7992f), Vector3(66.7068f, -69.0627f, 21.2078f)); 
  data[15].mAabb = Aabb(Vector3(20.901f, 22.0786f, -41.4464f), Vector3(23.065f, 28.0415f, -35.0445f)); 
  data[16].mAabb = Aabb(Vector3(20.9849f, -43.6387f, -28.2546f), Vector3(26.3545f, -38.2226f, -26.3217f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[1].mAabb = Aabb(Vector3(88.4939f, 20.6242f, -39.4657f), Vector3(89.1072f, 24.6067f, -33.5327f)); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[2].mAabb = Aabb(Vector3(54.6711f, -54.5253f, 53.9724f), Vector3(59.1507f, -46.9641f, 61.6333f)); 
  spatialPartition.UpdateData(keys[2], data[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[7].mAabb = Aabb(Vector3(-31.4325f, -31.1454f, 37.6143f), Vector3(-25.0046f, -27.0308f, 38.3622f)); 
  spatialPartition.UpdateData(keys[7], data[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[3].mAabb = Aabb(Vector3(-10.4529f, 25.6859f, -95.3534f), Vector3(-8.8907f, 26.6667f, -89.5409f)); 
  spatialPartition.UpdateData(keys[3], data[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(-72.4487f, -41.429f, 56.1954f), Vector3(-65.8929f, -39.4152f, 62.2798f)); 
  data[1].mAabb = Aabb(Vector3(69.1787f, -15.5738f, -67.5071f), Vector3(76.354f, -14.6976f, -62.9412f)); 
  data[2].mAabb = Aabb(Vector3(51.9576f, -27.7717f, 59.239f), Vector3(55.5586f, -21.8631f, 61.6441f)); 
  data[3].mAabb = Aabb(Vector3(51.5772f, -69.8417f, -52.9199f), Vector3(54.1192f, -66.221f, -46.5331f)); 
  data[4].mAabb = Aabb(Vector3(-8.8264f, 46.9438f, 14.2699f), Vector3(-3.67486f, 54.2907f, 21.6255f)); 
  data[5].mAabb = Aabb(Vector3(-47.8002f, -29.3961f, 58.5217f), Vector3(-41.9604f, -22.4138f, 62.1903f)); 
  data[6].mAabb = Aabb(Vector3(39.9966f, -72.5613f, -32.006f), Vector3(45.0493f, -71.998f, -31.2871f)); 
  data[7].mAabb = Aabb(Vector3(18.838f, -78.2559f, 47.8236f), Vector3(22.6793f, -73.5456f, 52.575f)); 
  data[8].mAabb = Aabb(Vector3(74.9177f, -37.7811f, 7.15594f), Vector3(79.2663f, -30.1246f, 11.0636f)); 
  data[9].mAabb = Aabb(Vector3(-51.3036f, -53.3524f, -2.85526f), Vector3(-44.5556f, -47.9961f, 4.26452f)); 
  data[10].mAabb = Aabb(Vector3(40.6681f, -31.8411f, 17.3438f), Vector3(48.1043f, -26.2658f, 22.7978f)); 
  data[11].mAabb = Aabb(Vector3(40.0596f, 18.4734f, -88.4628f), Vector3(45.6082f, 25.7154f, -81.5195f)); 
  data[12].mAabb = Aabb(Vector3(53.8253f, -77.7473f, 29.5051f), Vector3(54.7223f, -73.873f, 33.702f)); 
  data[13].mAabb = Aabb(Vector3(48.6405f, 62.5218f, 34.6279f), Vector3(49.4044f, 64.9984f, 39.7027f)); 
  data[14].mAabb = Aabb(Vector3(12.7841f, -8.9602f, 40.4206f), Vector3(14.9535f, -4.81274f, 45.048f)); 
  data[15].mAabb = Aabb(Vector3(69.645f, 20.3956f, -52.0594f), Vector3(71.2824f, 21.6336f, -46.5998f)); 
  data[16].mAabb = Aabb(Vector3(-54.7926f, -81.7804f, -16.2063f), Vector3(-51.7701f, -80.3237f, -8.35053f)); 
  data[17].mAabb = Aabb(Vector3(21.0832f, -19.9212f, 64.4706f), Vector3(26.1167f, -16.9956f, 70.195f)); 
  data[18].mAabb = Aabb(Vector3(54.6318f, -12.6293f, 56.2918f), Vector3(56.0259f, -10.7646f, 63.7528f)); 
  data[19].mAabb = Aabb(Vector3(44.5242f, 25.842f, 18.2997f), Vector3(49.6305f, 30.0435f, 24.6056f)); 
  data[20].mAabb = Aabb(Vector3(-35.9368f, 26.2284f, 5.46131f), Vector3(-28.5783f, 29.8148f, 8.74943f)); 
  data[21].mAabb = Aabb(Vector3(70.6594f, -51.5016f, 26.9729f), Vector3(75.6641f, -50.5718f, 34.4522f)); 
  data[22].mAabb = Aabb(Vector3(9.41898f, 77.0768f, 30.5128f), Vector3(13.4001f, 84.1469f, 34.9653f)); 
  data[23].mAabb = Aabb(Vector3(-17.7323f, -0.073591f, 37.9919f), Vector3(-12.0617f, 5.55597f, 43.4468f)); 
  data[24].mAabb = Aabb(Vector3(10.7996f, 7.37329f, -17.3424f), Vector3(18.0287f, 8.89634f, -13.1268f)); 
  data[25].mAabb = Aabb(Vector3(-56.3026f, -63.3678f, -5.3156f), Vector3(-52.5049f, -56.3194f, -2.81514f)); 
  data[26].mAabb = Aabb(Vector3(-48.1865f, 5.34039f, 9.39982f), Vector3(-47.6956f, 6.13144f, 13.7955f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[8].mAabb = Aabb(Vector3(-65.6658f, 14.3686f, -53.4271f), Vector3(-62.094f, 17.2183f, -47.9105f)); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[15].mAabb = Aabb(Vector3(-79.056f, 2.51997f, -15.8214f), Vector3(-73.744f, 4.39952f, -13.2043f)); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[23].mAabb = Aabb(Vector3(-86.2469f, -44.6985f, -12.4507f), Vector3(-83.2666f, -42.3896f, -9.52166f)); 
  spatialPartition.UpdateData(keys[23], data[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[22].mAabb = Aabb(Vector3(4.96589f, 88.1114f, -13.8962f), Vector3(10.7764f, 94.1779f, -9.77985f)); 
  spatialPartition.UpdateData(keys[22], data[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[3].mAabb = Aabb(Vector3(-58.3494f, 42.9201f, 19.2911f), Vector3(-52.2724f, 44.3045f, 22.2457f)); 
  spatialPartition.UpdateData(keys[3], data[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[1].mAabb = Aabb(Vector3(-40.5984f, -19.9257f, 77.9385f), Vector3(-35.5512f, -16.6471f, 85.617f)); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(23); 
  data[0].mAabb = Aabb(Vector3(3.65703f, -23.9678f, -56.8419f), Vector3(11.3414f, -16.1155f, -54.096f)); 
  data[1].mAabb = Aabb(Vector3(-25.1767f, -85.9062f, -9.70008f), Vector3(-18.6168f, -83.0339f, -3.00451f)); 
  data[2].mAabb = Aabb(Vector3(9.32641f, 31.8688f, -81.9932f), Vector3(12.9212f, 38.578f, -77.459f)); 
  data[3].mAabb = Aabb(Vector3(-23.6985f, 9.53439f, 44.4478f), Vector3(-23.2099f, 14.0894f, 45.3829f)); 
  data[4].mAabb = Aabb(Vector3(-32.9329f, -15.1098f, 26.4023f), Vector3(-31.3602f, -9.67741f, 31.308f)); 
  data[5].mAabb = Aabb(Vector3(4.01602f, 71.0355f, 19.6555f), Vector3(5.69239f, 72.8879f, 20.6954f)); 
  data[6].mAabb = Aabb(Vector3(56.8409f, -14.9604f, 32.3673f), Vector3(62.3963f, -13.6837f, 36.2654f)); 
  data[7].mAabb = Aabb(Vector3(-42.8569f, 36.5674f, 24.0961f), Vector3(-40.8711f, 37.5287f, 25.368f)); 
  data[8].mAabb = Aabb(Vector3(7.4219f, -20.8634f, 45.8291f), Vector3(13.1354f, -15.3229f, 49.008f)); 
  data[9].mAabb = Aabb(Vector3(43.3977f, -25.5511f, 46.2596f), Vector3(51.1326f, -23.9689f, 49.8242f)); 
  data[10].mAabb = Aabb(Vector3(-11.8938f, 82.0834f, 37.0513f), Vector3(-4.20225f, 86.3855f, 38.3345f)); 
  data[11].mAabb = Aabb(Vector3(18.2086f, -98.3596f, -20.265f), Vector3(25.5786f, -92.8084f, -14.3854f)); 
  data[12].mAabb = Aabb(Vector3(-59.0233f, -10.2929f, -29.6488f), Vector3(-53.6319f, -5.38213f, -24.4538f)); 
  data[13].mAabb = Aabb(Vector3(0.283288f, -29.3352f, 64.8854f), Vector3(1.60409f, -23.207f, 67.875f)); 
  data[14].mAabb = Aabb(Vector3(-30.6187f, 27.5859f, 22.7079f), Vector3(-28.0955f, 32.1762f, 24.6909f)); 
  data[15].mAabb = Aabb(Vector3(-48.482f, 56.8635f, -30.8257f), Vector3(-44.936f, 58.2726f, -24.729f)); 
  data[16].mAabb = Aabb(Vector3(-47.4878f, -9.25969f, 43.9718f), Vector3(-45.5609f, -6.73929f, 49.1776f)); 
  data[17].mAabb = Aabb(Vector3(32.4342f, 35.0528f, -21.3959f), Vector3(36.5188f, 38.1714f, -13.495f)); 
  data[18].mAabb = Aabb(Vector3(29.4727f, -47.1569f, -6.26354f), Vector3(36.5549f, -41.3963f, -0.467443f)); 
  data[19].mAabb = Aabb(Vector3(7.79592f, 83.8954f, -13.1346f), Vector3(15.1228f, 84.545f, -7.20604f)); 
  data[20].mAabb = Aabb(Vector3(-27.8332f, -29.8084f, 81.167f), Vector3(-27.4174f, -23.8368f, 84.1051f)); 
  data[21].mAabb = Aabb(Vector3(8.60353f, -15.172f, 94.9187f), Vector3(14.7183f, -11.5587f, 98.0572f)); 
  data[22].mAabb = Aabb(Vector3(-94.187f, -2.13704f, 16.352f), Vector3(-90.5621f, 4.36903f, 21.8041f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[13].mAabb = Aabb(Vector3(-37.5822f, 89.8022f, 8.08767f), Vector3(-29.916f, 92.0202f, 8.91142f)); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[11].mAabb = Aabb(Vector3(-60.6474f, 4.08362f, -23.073f), Vector3(-53.3944f, 10.7824f, -18.4735f)); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[9].mAabb = Aabb(Vector3(12.7069f, -36.2869f, 71.7639f), Vector3(15.8264f, -29.0509f, 76.1503f)); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(22); 
  data[0].mAabb = Aabb(Vector3(41.0558f, 44.929f, -61.5709f), Vector3(46.8104f, 46.7816f, -56.6885f)); 
  data[1].mAabb = Aabb(Vector3(52.9955f, 32.7517f, -10.4735f), Vector3(59.2648f, 40.668f, -5.81947f)); 
  data[2].mAabb = Aabb(Vector3(34.5471f, 41.038f, 8.97914f), Vector3(40.4515f, 48.0476f, 14.5106f)); 
  data[3].mAabb = Aabb(Vector3(61.4523f, -62.5916f, -17.289f), Vector3(65.0429f, -55.7151f, -16.6835f)); 
  data[4].mAabb = Aabb(Vector3(54.5091f, 8.83062f, 13.5025f), Vector3(58.2203f, 11.2541f, 20.6163f)); 
  data[5].mAabb = Aabb(Vector3(-57.2927f, -49.3641f, -65.2615f), Vector3(-54.6941f, -45.7522f, -59.8465f)); 
  data[6].mAabb = Aabb(Vector3(-53.4693f, 60.2695f, -31.7356f), Vector3(-47.5394f, 64.4174f, -31.0238f)); 
  data[7].mAabb = Aabb(Vector3(-15.787f, -75.749f, 21.4382f), Vector3(-12.8442f, -70.8644f, 22.1021f)); 
  data[8].mAabb = Aabb(Vector3(-38.441f, -64.2197f, 36.5879f), Vector3(-31.372f, -59.7462f, 38.7938f)); 
  data[9].mAabb = Aabb(Vector3(34.6426f, 3.58861f, 65.8907f), Vector3(41.4045f, 9.59972f, 72.4946f)); 
  data[10].mAabb = Aabb(Vector3(40.3651f, -21.4402f, -86.9973f), Vector3(41.3433f, -20.6269f, -84.3276f)); 
  data[11].mAabb = Aabb(Vector3(63.9898f, 49.8651f, -8.26102f), Vector3(66.066f, 55.4001f, -1.47268f)); 
  data[12].mAabb = Aabb(Vector3(-10.2314f, 1.88786f, -35.5884f), Vector3(-4.65659f, 9.86513f, -28.9925f)); 
  data[13].mAabb = Aabb(Vector3(-25.894f, -18.5173f, 42.603f), Vector3(-18.2454f, -15.822f, 50.5223f)); 
  data[14].mAabb = Aabb(Vector3(50.3942f, 80.5276f, 0.881696f), Vector3(54.9102f, 88.526f, 7.54433f)); 
  data[15].mAabb = Aabb(Vector3(64.1341f, -2.82443f, -65.5892f), Vector3(65.5584f, -0.670497f, -63.6728f)); 
  data[16].mAabb = Aabb(Vector3(49.9147f, 2.25206f, -5.36101f), Vector3(52.1505f, 2.929f, 0.59954f)); 
  data[17].mAabb = Aabb(Vector3(2.40723f, -69.2285f, -58.4284f), Vector3(9.27396f, -68.1863f, -54.6045f)); 
  data[18].mAabb = Aabb(Vector3(50.4315f, -59.1755f, -10.0786f), Vector3(55.3877f, -56.5763f, -5.2965f)); 
  data[19].mAabb = Aabb(Vector3(-82.8961f, 24.4163f, -13.8524f), Vector3(-75.6005f, 30.0099f, -12.8869f)); 
  data[20].mAabb = Aabb(Vector3(-40.1598f, -25.8313f, -35.3578f), Vector3(-37.677f, -20.0363f, -33.2628f)); 
  data[21].mAabb = Aabb(Vector3(-30.2062f, -10.1562f, -53.8119f), Vector3(-29.6619f, -3.33631f, -51.6511f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[15].mAabb = Aabb(Vector3(-42.6574f, 42.5122f, -58.249f), Vector3(-40.9381f, 43.3622f, -57.7502f)); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[9].mAabb = Aabb(Vector3(-14.5254f, -18.6871f, -96.1611f), Vector3(-10.7864f, -13.0904f, -92.5381f)); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[7].mAabb = Aabb(Vector3(-26.4011f, 35.0317f, 33.5574f), Vector3(-25.2111f, 41.9604f, 37.5652f)); 
  spatialPartition.UpdateData(keys[7], data[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[12].mAabb = Aabb(Vector3(-27.731f, -12.2577f, -75.6719f), Vector3(-20.7791f, -6.71229f, -72.7795f)); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(16); 
  data[0].mAabb = Aabb(Vector3(-5.06216f, 67.9646f, -42.6485f), Vector3(1.75124f, 72.6719f, -37.9608f)); 
  data[1].mAabb = Aabb(Vector3(-24.3099f, -2.0671f, -49.2638f), Vector3(-17.3814f, -1.04852f, -44.6538f)); 
  data[2].mAabb = Aabb(Vector3(75.7234f, -52.6243f, -35.8311f), Vector3(77.6391f, -48.35f, -29.1325f)); 
  data[3].mAabb = Aabb(Vector3(49.0111f, -73.7346f, -15.3381f), Vector3(51.7387f, -69.7075f, -8.44257f)); 
  data[4].mAabb = Aabb(Vector3(-40.3711f, -55.9639f, 36.8605f), Vector3(-34.752f, -51.8472f, 42.7508f)); 
  data[5].mAabb = Aabb(Vector3(-27.6004f, 21.6597f, 4.60428f), Vector3(-26.9367f, 28.1352f, 11.4508f)); 
  data[6].mAabb = Aabb(Vector3(3.68335f, -5.61985f, -54.4903f), Vector3(11.5423f, -3.91402f, -49.0851f)); 
  data[7].mAabb = Aabb(Vector3(-15.7659f, 51.3708f, -33.0258f), Vector3(-14.0881f, 53.3151f, -25.116f)); 
  data[8].mAabb = Aabb(Vector3(54.468f, -16.5734f, -43.5622f), Vector3(60.2671f, -10.4313f, -40.7883f)); 
  data[9].mAabb = Aabb(Vector3(-16.983f, -80.3412f, 55.5801f), Vector3(-9.57818f, -75.073f, 56.9466f)); 
  data[10].mAabb = Aabb(Vector3(-73.0868f, 53.0761f, -44.6752f), Vector3(-68.8125f, 58.5461f, -37.4612f)); 
  data[11].mAabb = Aabb(Vector3(68.8694f, -30.3925f, -3.02768f), Vector3(72.8986f, -29.1775f, -1.48422f)); 
  data[12].mAabb = Aabb(Vector3(-39.4671f, -40.0781f, -10.1837f), Vector3(-33.1916f, -38.2017f, -4.89606f)); 
  data[13].mAabb = Aabb(Vector3(65.7345f, -17.6088f, 21.4345f), Vector3(69.7233f, -16.7785f, 22.6681f)); 
  data[14].mAabb = Aabb(Vector3(3.62791f, -2.46878f, 31.4048f), Vector3(4.46303f, -0.987707f, 33.3868f)); 
  data[15].mAabb = Aabb(Vector3(-2.09083f, 7.19436f, 29.561f), Vector3(0.693521f, 10.046f, 36.2592f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[9].mAabb = Aabb(Vector3(13.4487f, -12.8947f, -43.2487f), Vector3(17.2554f, -10.4534f, -38.488f)); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[3].mAabb = Aabb(Vector3(9.55147f, 33.26f, -10.1726f), Vector3(13.6834f, 37.2409f, -6.58364f)); 
  spatialPartition.UpdateData(keys[3], data[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[8].mAabb = Aabb(Vector3(-41.273f, 72.9154f, 8.47612f), Vector3(-33.3354f, 73.7744f, 10.4471f)); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(23); 
  data[0].mAabb = Aabb(Vector3(4.90752f, 37.5091f, -56.8427f), Vector3(11.4089f, 41.7913f, -51.5124f)); 
  data[1].mAabb = Aabb(Vector3(-78.7976f, -7.05316f, 7.08689f), Vector3(-78.1017f, -4.27878f, 8.45176f)); 
  data[2].mAabb = Aabb(Vector3(17.4021f, 24.6327f, 43.2512f), Vector3(21.5649f, 28.4947f, 45.9514f)); 
  data[3].mAabb = Aabb(Vector3(6.72256f, -53.9645f, -54.4556f), Vector3(13.5554f, -48.0534f, -51.5734f)); 
  data[4].mAabb = Aabb(Vector3(-35.0771f, 4.02747f, -70.8231f), Vector3(-29.8064f, 7.82169f, -64.7006f)); 
  data[5].mAabb = Aabb(Vector3(-34.9932f, 7.35846f, 79.3644f), Vector3(-28.1636f, 10.5902f, 82.0096f)); 
  data[6].mAabb = Aabb(Vector3(23.8193f, -29.4635f, -50.1027f), Vector3(28.319f, -28.7233f, -46.3426f)); 
  data[7].mAabb = Aabb(Vector3(-32.897f, -3.82176f, 13.6379f), Vector3(-30.7644f, 3.62645f, 20.0027f)); 
  data[8].mAabb = Aabb(Vector3(0.420965f, -67.1917f, -52.3129f), Vector3(2.53431f, -60.9505f, -48.5166f)); 
  data[9].mAabb = Aabb(Vector3(-22.7592f, -26.3572f, -92.9022f), Vector3(-21.761f, -18.5423f, -92.4976f)); 
  data[10].mAabb = Aabb(Vector3(-7.16811f, -0.895887f, 67.1053f), Vector3(-0.536793f, 2.66964f, 70.444f)); 
  data[11].mAabb = Aabb(Vector3(54.3399f, -52.6693f, -61.6595f), Vector3(58.3475f, -51.4346f, -59.7396f)); 
  data[12].mAabb = Aabb(Vector3(-0.967814f, 15.1188f, 53.5086f), Vector3(2.40311f, 22.0558f, 60.9011f)); 
  data[13].mAabb = Aabb(Vector3(-65.2554f, 61.8361f, 7.88875f), Vector3(-63.3685f, 63.4824f, 14.8323f)); 
  data[14].mAabb = Aabb(Vector3(24.5542f, -67.6342f, 6.73651f), Vector3(26.2297f, -61.3367f, 10.3505f)); 
  data[15].mAabb = Aabb(Vector3(60.4844f, 42.4102f, -28.0717f), Vector3(62.0311f, 45.1677f, -25.638f)); 
  data[16].mAabb = Aabb(Vector3(-41.5088f, -71.9011f, -59.0882f), Vector3(-37.6675f, -69.3239f, -51.5312f)); 
  data[17].mAabb = Aabb(Vector3(-12.6406f, 46.804f, 48.142f), Vector3(-11.39f, 48.9921f, 55.0403f)); 
  data[18].mAabb = Aabb(Vector3(72.167f, -40.1758f, -39.2818f), Vector3(73.1241f, -32.3502f, -38.7566f)); 
  data[19].mAabb = Aabb(Vector3(-34.3887f, 36.0868f, 44.9682f), Vector3(-27.5604f, 37.3694f, 46.5709f)); 
  data[20].mAabb = Aabb(Vector3(16.7179f, -62.8768f, -37.6048f), Vector3(22.5854f, -60.5051f, -33.6619f)); 
  data[21].mAabb = Aabb(Vector3(13.9892f, -36.8419f, -44.4771f), Vector3(16.2306f, -31.7827f, -38.697f)); 
  data[22].mAabb = Aabb(Vector3(-64.1836f, -23.3595f, 13.9818f), Vector3(-63.7115f, -18.7839f, 19.9346f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[14].mAabb = Aabb(Vector3(-14.8775f, -83.0643f, 18.9891f), Vector3(-7.91499f, -75.6673f, 23.6659f)); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[5].mAabb = Aabb(Vector3(-72.1041f, -59.6068f, -34.8431f), Vector3(-69.2942f, -56.3047f, -27.9745f)); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[11].mAabb = Aabb(Vector3(-40.9882f, -36.147f, 39.8331f), Vector3(-34.2636f, -31.8766f, 44.7144f)); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[20].mAabb = Aabb(Vector3(4.83025f, 26.2652f, 50.0667f), Vector3(7.65937f, 29.4392f, 51.0992f)); 
  spatialPartition.UpdateData(keys[20], data[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(-29.7339f, -35.8909f, -18.0201f), Vector3(-26.0826f, -33.3621f, -10.8344f)); 
  data[1].mAabb = Aabb(Vector3(-25.0573f, -40.1169f, 28.0748f), Vector3(-21.6736f, -35.8257f, 30.0854f)); 
  data[2].mAabb = Aabb(Vector3(79.0365f, 52.3188f, 5.42146f), Vector3(85.5952f, 56.4829f, 9.27158f)); 
  data[3].mAabb = Aabb(Vector3(95.7821f, -4.80516f, 0.035197f), Vector3(101.479f, -4.07093f, 2.55026f)); 
  data[4].mAabb = Aabb(Vector3(11.3557f, -5.6674f, -92.5756f), Vector3(13.4189f, -0.0835266f, -90.4068f)); 
  data[5].mAabb = Aabb(Vector3(39.9585f, 57.853f, -62.5495f), Vector3(43.1381f, 62.6035f, -61.2489f)); 
  data[6].mAabb = Aabb(Vector3(18.903f, 7.034f, -94.8904f), Vector3(25.8641f, 13.2177f, -92.0831f)); 
  data[7].mAabb = Aabb(Vector3(78.0315f, -1.61239f, -24.7138f), Vector3(83.0509f, 3.60479f, -23.3098f)); 
  data[8].mAabb = Aabb(Vector3(40.6144f, -49.9984f, 55.706f), Vector3(41.8716f, -42.9853f, 56.5233f)); 
  data[9].mAabb = Aabb(Vector3(-22.7063f, 58.025f, -11.2879f), Vector3(-15.0211f, 63.5045f, -8.15467f)); 
  data[10].mAabb = Aabb(Vector3(37.6857f, 7.74887f, 29.7848f), Vector3(39.2927f, 10.058f, 36.0689f)); 
  data[11].mAabb = Aabb(Vector3(50.7058f, -7.42743f, 32.6162f), Vector3(57.4187f, -5.18327f, 34.4478f)); 
  data[12].mAabb = Aabb(Vector3(79.6343f, 21.5028f, 2.61944f), Vector3(85.3197f, 22.5341f, 7.99387f)); 
  data[13].mAabb = Aabb(Vector3(28.7423f, 59.8609f, -18.7588f), Vector3(35.8201f, 66.5666f, -15.4058f)); 
  data[14].mAabb = Aabb(Vector3(11.2023f, -69.5725f, 51.8744f), Vector3(14.1594f, -68.3672f, 54.026f)); 
  data[15].mAabb = Aabb(Vector3(-3.34327f, 50.6845f, -40.8565f), Vector3(-1.82022f, 52.8978f, -38.9503f)); 
  data[16].mAabb = Aabb(Vector3(28.8819f, -20.8043f, -23.7697f), Vector3(36.7847f, -17.5109f, -19.6533f)); 
  data[17].mAabb = Aabb(Vector3(58.8542f, -74.7778f, 23.0189f), Vector3(62.8574f, -72.1809f, 28.5515f)); 
  data[18].mAabb = Aabb(Vector3(66.1852f, -0.440906f, -49.0576f), Vector3(73.3521f, 0.159955f, -48.4203f)); 
  data[19].mAabb = Aabb(Vector3(34.2512f, 26.0489f, 44.9872f), Vector3(35.4517f, 32.7062f, 45.6681f)); 
  data[20].mAabb = Aabb(Vector3(86.1997f, -38.9315f, 3.55416f), Vector3(90.8885f, -36.4004f, 5.98967f)); 
  data[21].mAabb = Aabb(Vector3(37.9444f, -3.18574f, -21.5114f), Vector3(38.4305f, 2.1532f, -19.5701f)); 
  data[22].mAabb = Aabb(Vector3(57.5328f, -30.2507f, -55.9175f), Vector3(60.6145f, -24.347f, -54.6359f)); 
  data[23].mAabb = Aabb(Vector3(25.7374f, 45.6696f, 1.77494f), Vector3(32.6194f, 49.9655f, 4.17242f)); 
  data[24].mAabb = Aabb(Vector3(-39.0666f, 1.9762f, -19.3584f), Vector3(-32.5494f, 2.52023f, -17.6857f)); 
  data[25].mAabb = Aabb(Vector3(61.0084f, -2.51546f, 53.4831f), Vector3(61.9943f, 3.04615f, 54.5494f)); 
  data[26].mAabb = Aabb(Vector3(-39.0166f, -19.8724f, -24.611f), Vector3(-34.0332f, -11.9079f, -18.1643f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[20].mAabb = Aabb(Vector3(-90.1439f, 22.1528f, 23.05f), Vector3(-82.5222f, 24.5287f, 26.7881f)); 
  spatialPartition.UpdateData(keys[20], data[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[6].mAabb = Aabb(Vector3(-7.8764f, -40.5908f, 54.2924f), Vector3(-7.40589f, -37.9064f, 57.8835f)); 
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[5].mAabb = Aabb(Vector3(-59.3045f, 52.1165f, -50.2777f), Vector3(-52.5131f, 53.6549f, -46.0639f)); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[10].mAabb = Aabb(Vector3(-4.28495f, 15.9519f, -54.2897f), Vector3(1.192f, 22.5765f, -47.8223f)); 
  spatialPartition.UpdateData(keys[10], data[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[15].mAabb = Aabb(Vector3(11.9746f, -82.1802f, 14.2122f), Vector3(19.5052f, -77.797f, 19.6815f)); 
  spatialPartition.UpdateData(keys[15], data[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[26]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[25].mAabb = Aabb(Vector3(-98.1663f, -19.5188f, -0.331204f), Vector3(-97.244f, -18.6902f, 4.67699f)); 
  spatialPartition.UpdateData(keys[25], data[25]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(-64.9768f, -25.8475f, 59.7394f), Vector3(-60.4849f, -18.5841f, 61.4538f)); 
  data[1].mAabb = Aabb(Vector3(51.032f, -42.8563f, -26.7229f), Vector3(51.6649f, -35.798f, -19.515f)); 
  data[2].mAabb = Aabb(Vector3(15.2922f, -44.1226f, -47.5738f), Vector3(18.9668f, -40.8674f, -40.4169f)); 
  data[3].mAabb = Aabb(Vector3(7.14488f, 5.09742f, 21.4983f), Vector3(11.0272f, 9.33604f, 29.4867f)); 
  data[4].mAabb = Aabb(Vector3(-93.7557f, 6.63021f, -11.4106f), Vector3(-90.6627f, 12.3556f, -4.41537f)); 
  data[5].mAabb = Aabb(Vector3(-80.0017f, -3.48744f, -13.7445f), Vector3(-76.8996f, -1.90013f, -12.2321f)); 
  data[6].mAabb = Aabb(Vector3(52.6375f, 27.9485f, -62.9636f), Vector3(54.277f, 33.3398f, -60.6689f)); 
  data[7].mAabb = Aabb(Vector3(-2.3157f, -36.7562f, -68.6399f), Vector3(0.540325f, -29.9565f, -65.5041f)); 
  data[8].mAabb = Aabb(Vector3(-13.4144f, 15.6826f, 39.1246f), Vector3(-10.4222f, 17.391f, 44.1425f)); 
  data[9].mAabb = Aabb(Vector3(-2.07498f, 18.0284f, 73.7899f), Vector3(-0.673231f, 22.5389f, 76.7187f)); 
  data[10].mAabb = Aabb(Vector3(-28.4815f, -42.0698f, 69.6797f), Vector3(-23.0078f, -36.3217f, 75.3171f)); 
  data[11].mAabb = Aabb(Vector3(-78.3292f, 26.1039f, 3.83402f), Vector3(-77.527f, 30.9147f, 5.41344f)); 
  data[12].mAabb = Aabb(Vector3(-53.2599f, -34.5253f, 24.9321f), Vector3(-50.1169f, -32.7594f, 28.8131f)); 
  data[13].mAabb = Aabb(Vector3(-39.4138f, -15.5517f, -14.7819f), Vector3(-38.8088f, -11.2924f, -9.97504f)); 
  data[14].mAabb = Aabb(Vector3(42.6752f, -26.7646f, 13.8347f), Vector3(43.4152f, -22.74f, 18.3533f)); 
  data[15].mAabb = Aabb(Vector3(22.7389f, -80.7146f, -48.1093f), Vector3(25.1417f, -75.2621f, -46.4099f)); 
  data[16].mAabb = Aabb(Vector3(-12.3464f, -20.1055f, 51.9033f), Vector3(-9.05643f, -12.8781f, 57.4078f)); 
  data[17].mAabb = Aabb(Vector3(-41.9937f, -28.0846f, 36.8546f), Vector3(-38.8086f, -26.2672f, 38.6453f)); 
  data[18].mAabb = Aabb(Vector3(-58.9361f, -7.38168f, -23.7566f), Vector3(-53.1544f, -2.24962f, -21.7678f)); 
  data[19].mAabb = Aabb(Vector3(-43.9427f, 13.7596f, 86.9041f), Vector3(-39.1458f, 19.045f, 91.1881f)); 
  data[20].mAabb = Aabb(Vector3(-17.5526f, 34.7971f, -47.0721f), Vector3(-13.0187f, 39.7021f, -43.9646f)); 
  data[21].mAabb = Aabb(Vector3(-73.7402f, 62.8841f, 6.18809f), Vector3(-71.525f, 63.5986f, 10.825f)); 
  data[22].mAabb = Aabb(Vector3(-3.62574f, -49.9101f, -38.4348f), Vector3(-1.16587f, -43.3351f, -36.9676f)); 
  data[23].mAabb = Aabb(Vector3(-22.3863f, -12.7013f, -62.3362f), Vector3(-21.6305f, -9.18127f, -59.2146f)); 
  data[24].mAabb = Aabb(Vector3(52.6453f, 21.984f, -4.63092f), Vector3(59.6897f, 26.3376f, 1.60332f)); 
  data[25].mAabb = Aabb(Vector3(19.3179f, -59.0506f, -20.157f), Vector3(21.4401f, -53.4772f, -13.5832f)); 
  data[26].mAabb = Aabb(Vector3(46.4101f, 9.04f, -89.3465f), Vector3(47.2541f, 11.0242f, -85.604f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[12].mAabb = Aabb(Vector3(-27.7386f, 55.3547f, -35.6011f), Vector3(-25.5432f, 60.2797f, -27.9238f)); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[14].mAabb = Aabb(Vector3(-72.3684f, 22.7481f, 52.4209f), Vector3(-69.1709f, 23.2603f, 53.4279f)); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[0].mAabb = Aabb(Vector3(4.90126f, 7.50165f, 83.1218f), Vector3(10.0855f, 11.133f, 88.3703f)); 
  spatialPartition.UpdateData(keys[0], data[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[2].mAabb = Aabb(Vector3(13.6331f, -24.6199f, -0.575122f), Vector3(14.8289f, -22.7766f, 5.06952f)); 
  spatialPartition.UpdateData(keys[2], data[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(50.158f, -2.37577f, 46.5271f), Vector3(57.538f, 1.82852f, 50.8745f)); 
  data[1].mAabb = Aabb(Vector3(-48.9592f, 79.2286f, -12.3984f), Vector3(-41.8065f, 82.5225f, -8.68603f)); 
  data[2].mAabb = Aabb(Vector3(64.6849f, 1.81779f, 46.0644f), Vector3(71.7696f, 7.13053f, 53.7075f)); 
  data[3].mAabb = Aabb(Vector3(-4.41191f, -15.2683f, 19.6853f), Vector3(-3.75492f, -14.1377f, 27.1964f)); 
  data[4].mAabb = Aabb(Vector3(84.4523f, 34.7075f, 19.5691f), Vector3(87.1858f, 38.1118f, 21.6973f)); 
  data[5].mAabb = Aabb(Vector3(-81.733f, 37.3848f, -25.1929f), Vector3(-78.1359f, 43.9363f, -19.6229f)); 
  data[6].mAabb = Aabb(Vector3(-37.8919f, 34.817f, 57.279f), Vector3(-30.5298f, 35.4969f, 62.7601f)); 
  data[7].mAabb = Aabb(Vector3(23.8904f, -19.0693f, 19.7724f), Vector3(28.7063f, -18.0429f, 20.2559f)); 
  data[8].mAabb = Aabb(Vector3(-82.59f, -26.0845f, 34.0992f), Vector3(-81.7503f, -21.9509f, 35.2097f)); 
  data[9].mAabb = Aabb(Vector3(-15.6284f, 45.6503f, -20.2521f), Vector3(-8.80963f, 46.51f, -15.374f)); 
  data[10].mAabb = Aabb(Vector3(16.3699f, -27.4879f, -23.4723f), Vector3(22.7266f, -23.3421f, -22.0485f)); 
  data[11].mAabb = Aabb(Vector3(23.9392f, -39.2132f, -21.5936f), Vector3(25.3272f, -35.2172f, -19.1516f)); 
  data[12].mAabb = Aabb(Vector3(15.6427f, -44.0314f, -10.8295f), Vector3(16.1369f, -40.1126f, -6.6563f)); 
  data[13].mAabb = Aabb(Vector3(28.9414f, 24.6811f, 67.8672f), Vector3(30.8578f, 26.9543f, 69.2859f)); 
  data[14].mAabb = Aabb(Vector3(3.41262f, 38.864f, -73.0029f), Vector3(4.90993f, 39.5767f, -70.8062f)); 
  data[15].mAabb = Aabb(Vector3(-68.2055f, 2.69282f, 58.1587f), Vector3(-60.4375f, 10.2767f, 58.9309f)); 
  data[16].mAabb = Aabb(Vector3(-72.9981f, -56.0773f, -2.52167f), Vector3(-65.2419f, -50.0304f, 2.94183f)); 
  data[17].mAabb = Aabb(Vector3(66.4877f, -17.6164f, 42.6735f), Vector3(74.2736f, -11.4093f, 47.3154f)); 
  data[18].mAabb = Aabb(Vector3(37.6151f, -90.8259f, -16.3038f), Vector3(39.5292f, -84.9973f, -8.56544f)); 
  data[19].mAabb = Aabb(Vector3(0.906332f, -37.7425f, 64.9593f), Vector3(5.5592f, -35.168f, 69.0516f)); 
  data[20].mAabb = Aabb(Vector3(32.281f, 39.939f, -65.8394f), Vector3(36.4927f, 40.711f, -64.6645f)); 
  data[21].mAabb = Aabb(Vector3(-15.6584f, -5.83291f, -24.1368f), Vector3(-8.03764f, 0.0995753f, -23.1574f)); 
  data[22].mAabb = Aabb(Vector3(-82.555f, -33.8005f, -34.2454f), Vector3(-79.6765f, -28.5311f, -32.6919f)); 
  data[23].mAabb = Aabb(Vector3(22.6404f, -26.2836f, 59.7991f), Vector3(28.3605f, -19.1213f, 60.3378f)); 
  data[24].mAabb = Aabb(Vector3(2.62097f, 85.749f, -25.9737f), Vector3(6.20992f, 86.4243f, -20.9005f)); 
  data[25].mAabb = Aabb(Vector3(53.2948f, -18.5431f, -80.2975f), Vector3(59.0584f, -14.1215f, -78.3882f)); 
  data[26].mAabb = Aabb(Vector3(-34.2453f, 77.588f, 16.3441f), Vector3(-31.2546f, 79.9604f, 22.9571f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[6].mAabb = Aabb(Vector3(36.4496f, 37.5567f, -59.285f), Vector3(38.305f, 44.2777f, -58.3135f)); 
  spatialPartition.UpdateData(keys[6], data[6]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[20].mAabb = Aabb(Vector3(27.0711f, 80.5825f, 29.2527f), Vector3(30.3914f, 86.0017f, 30.8588f)); 
  spatialPartition.UpdateData(keys[20], data[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[21].mAabb = Aabb(Vector3(-79.0695f, -55.0027f, -30.6535f), Vector3(-71.0999f, -51.9736f, -29.2243f)); 
  spatialPartition.UpdateData(keys[21], data[21]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[17].mAabb = Aabb(Vector3(94.453f, 0.246002f, 19.2142f), Vector3(94.9064f, 6.85598f, 20.3376f)); 
  spatialPartition.UpdateData(keys[17], data[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[3].mAabb = Aabb(Vector3(-20.8968f, -30.22f, -68.7411f), Vector3(-17.156f, -22.3659f, -67.9943f)); 
  spatialPartition.UpdateData(keys[3], data[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(16); 
  data[0].mAabb = Aabb(Vector3(8.39273f, -62.2468f, -15.1733f), Vector3(12.7295f, -56.8677f, -7.82974f)); 
  data[1].mAabb = Aabb(Vector3(64.4992f, -41.0614f, 57.883f), Vector3(66.2542f, -33.4599f, 63.4815f)); 
  data[2].mAabb = Aabb(Vector3(14.9605f, -19.5122f, -60.0548f), Vector3(17.8504f, -13.294f, -56.3186f)); 
  data[3].mAabb = Aabb(Vector3(-58.5222f, 13.7932f, 36.0856f), Vector3(-52.5944f, 17.8314f, 40.6354f)); 
  data[4].mAabb = Aabb(Vector3(-17.0627f, 57.4634f, 28.4542f), Vector3(-10.3854f, 61.2717f, 34.4586f)); 
  data[5].mAabb = Aabb(Vector3(-83.9169f, -44.4653f, -3.95312f), Vector3(-76.8159f, -40.9763f, 2.04755f)); 
  data[6].mAabb = Aabb(Vector3(-37.9752f, -2.64102f, 63.0785f), Vector3(-33.4348f, 2.39455f, 64.1346f)); 
  data[7].mAabb = Aabb(Vector3(33.2654f, 47.1256f, 69.8231f), Vector3(38.9393f, 48.5967f, 76.4744f)); 
  data[8].mAabb = Aabb(Vector3(-11.4962f, -95.7191f, 12.2206f), Vector3(-8.45673f, -90.4678f, 19.3584f)); 
  data[9].mAabb = Aabb(Vector3(-54.9577f, -2.93865f, -33.2561f), Vector3(-47.1274f, -0.841081f, -27.2817f)); 
  data[10].mAabb = Aabb(Vector3(88.466f, -22.4396f, 3.25628f), Vector3(90.2377f, -18.069f, 5.04537f)); 
  data[11].mAabb = Aabb(Vector3(-55.186f, -14.1978f, 16.6497f), Vector3(-47.1992f, -9.02123f, 22.6172f)); 
  data[12].mAabb = Aabb(Vector3(11.6427f, -7.42279f, 59.9515f), Vector3(18.9981f, -1.65777f, 67.7673f)); 
  data[13].mAabb = Aabb(Vector3(-39.5603f, 42.6043f, 18.7236f), Vector3(-37.3664f, 49.468f, 19.3667f)); 
  data[14].mAabb = Aabb(Vector3(-16.5602f, -36.5286f, 40.7212f), Vector3(-9.56355f, -29.3147f, 46.2047f)); 
  data[15].mAabb = Aabb(Vector3(-36.9593f, 20.3441f, 27.0361f), Vector3(-36.0634f, 22.5736f, 32.4163f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[9].mAabb = Aabb(Vector3(-42.3693f, -14.3396f, 44.8626f), Vector3(-40.7331f, -12.4094f, 47.2209f)); 
  spatialPartition.UpdateData(keys[9], data[9]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[5].mAabb = Aabb(Vector3(19.8048f, -84.4393f, -23.1133f), Vector3(20.4889f, -81.6429f, -17.5517f)); 
  spatialPartition.UpdateData(keys[5], data[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[7].mAabb = Aabb(Vector3(-25.4099f, 81.9707f, 30.3106f), Vector3(-22.4087f, 88.1813f, 32.6202f)); 
  spatialPartition.UpdateData(keys[7], data[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(95.6686f, -20.6147f, 3.89996f), Vector3(99.5268f, -14.2786f, 6.16453f)); 
  data[1].mAabb = Aabb(Vector3(24.8858f, 51.1181f, 14.1763f), Vector3(29.2524f, 52.2884f, 15.48f)); 
  data[2].mAabb = Aabb(Vector3(-69.3681f, -48.7462f, -54.4649f), Vector3(-64.2653f, -47.575f, -46.4997f)); 
  data[3].mAabb = Aabb(Vector3(-47.32f, -22.9182f, -54.2327f), Vector3(-43.7107f, -18.3147f, -53.1327f)); 
  data[4].mAabb = Aabb(Vector3(13.9623f, 58.7803f, -61.5189f), Vector3(17.4944f, 63.5823f, -55.1019f)); 
  data[5].mAabb = Aabb(Vector3(-56.8305f, 21.9196f, -78.3298f), Vector3(-52.7006f, 24.2902f, -74.6458f)); 
  data[6].mAabb = Aabb(Vector3(-74.8617f, -30.6673f, 31.7282f), Vector3(-73.334f, -30.2207f, 36.4045f)); 
  data[7].mAabb = Aabb(Vector3(48.4171f, 21.6195f, 60.4142f), Vector3(52.5887f, 25.7639f, 61.436f)); 
  data[8].mAabb = Aabb(Vector3(-72.7143f, -19.3541f, 1.10592f), Vector3(-65.8798f, -12.5238f, 7.99655f)); 
  data[9].mAabb = Aabb(Vector3(34.2732f, -60.9117f, 26.6915f), Vector3(36.4157f, -55.2838f, 33.3172f)); 
  data[10].mAabb = Aabb(Vector3(-86.5451f, -15.2094f, 20.5294f), Vector3(-79.0598f, -10.949f, 27.6448f)); 
  data[11].mAabb = Aabb(Vector3(21.0722f, -49.3926f, -87.0843f), Vector3(24.9785f, -41.8623f, -81.8131f)); 
  data[12].mAabb = Aabb(Vector3(20.2555f, 54.1621f, -7.22104f), Vector3(27.142f, 61.1835f, -0.431537f)); 
  data[13].mAabb = Aabb(Vector3(32.4887f, -32.919f, -73.8535f), Vector3(38.0535f, -31.254f, -72.4974f)); 
  data[14].mAabb = Aabb(Vector3(1.23184f, -24.6602f, 88.2554f), Vector3(8.9106f, -24.0185f, 92.6315f)); 
  data[15].mAabb = Aabb(Vector3(-31.7751f, -10.8397f, 63.7166f), Vector3(-29.9261f, -9.00767f, 65.0204f)); 
  data[16].mAabb = Aabb(Vector3(21.0847f, -78.279f, 3.92495f), Vector3(25.0677f, -72.2741f, 11.2103f)); 
  data[17].mAabb = Aabb(Vector3(2.44083f, -12.6691f, 11.2022f), Vector3(8.06645f, -9.32458f, 16.0342f)); 
  data[18].mAabb = Aabb(Vector3(59.1151f, 43.4753f, -25.2397f), Vector3(66.5633f, 47.5337f, -19.2502f)); 
  data[19].mAabb = Aabb(Vector3(0.414195f, 35.7608f, -41.0395f), Vector3(7.45048f, 37.8266f, -33.3468f)); 
  data[20].mAabb = Aabb(Vector3(-42.0746f, 74.9589f, -13.3183f), Vector3(-34.9671f, 80.8529f, -7.27423f)); 
  data[21].mAabb = Aabb(Vector3(7.84822f, 70.9873f, 52.1877f), Vector3(10.0244f, 73.4028f, 59.2899f)); 
  data[22].mAabb = Aabb(Vector3(-17.063f, -57.435f, 41.452f), Vector3(-14.602f, -55.4873f, 46.1772f)); 
  data[23].mAabb = Aabb(Vector3(-20.6481f, 82.5071f, 40.1155f), Vector3(-16.0148f, 83.319f, 44.9822f)); 
  data[24].mAabb = Aabb(Vector3(7.7314f, 3.93912f, 15.3861f), Vector3(9.92685f, 10.6356f, 15.7872f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[11].mAabb = Aabb(Vector3(0.784893f, -66.645f, -26.4416f), Vector3(8.18509f, -59.4863f, -23.2144f)); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[16].mAabb = Aabb(Vector3(59.573f, -10.0715f, -30.2032f), Vector3(66.6944f, -2.81762f, -27.9623f)); 
  spatialPartition.UpdateData(keys[16], data[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[24].mAabb = Aabb(Vector3(77.7507f, 12.5959f, -34.0392f), Vector3(78.5218f, 17.3541f, -29.7874f)); 
  spatialPartition.UpdateData(keys[24], data[24]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(57.5155f, 9.33191f, -2.61126f), Vector3(58.3474f, 10.0527f, 2.90883f)); 
  data[1].mAabb = Aabb(Vector3(4.06452f, -83.725f, -18.6026f), Vector3(6.66541f, -78.1147f, -12.3016f)); 
  data[2].mAabb = Aabb(Vector3(95.741f, -3.71521f, -8.60037f), Vector3(102.918f, 3.65809f, -0.837874f)); 
  data[3].mAabb = Aabb(Vector3(-62.9029f, 52.0366f, -41.6762f), Vector3(-57.678f, 54.5904f, -39.5982f)); 
  data[4].mAabb = Aabb(Vector3(-9.72546f, 78.1986f, -1.55143f), Vector3(-7.11042f, 81.7966f, 2.21426f)); 
  data[5].mAabb = Aabb(Vector3(37.7011f, 3.86768f, 17.2225f), Vector3(39.4253f, 4.61838f, 23.0163f)); 
  data[6].mAabb = Aabb(Vector3(70.0756f, 46.9884f, 44.6577f), Vector3(76.6684f, 49.7089f, 45.4258f)); 
  data[7].mAabb = Aabb(Vector3(-9.20298f, -83.9815f, 28.0743f), Vector3(-2.17528f, -76.2308f, 29.2666f)); 
  data[8].mAabb = Aabb(Vector3(74.1025f, 40.4325f, 4.75883f), Vector3(80.5801f, 45.5398f, 12.4151f)); 
  data[9].mAabb = Aabb(Vector3(-60.4138f, -21.8225f, 36.1039f), Vector3(-58.7601f, -14.2548f, 41.0259f)); 
  data[10].mAabb = Aabb(Vector3(-2.13925f, -12.7942f, -30.021f), Vector3(5.23637f, -5.42228f, -27.424f)); 
  data[11].mAabb = Aabb(Vector3(30.5526f, -5.93275f, 40.3622f), Vector3(34.6776f, -5.04498f, 42.3244f)); 
  data[12].mAabb = Aabb(Vector3(-22.5206f, -55.9895f, -54.7646f), Vector3(-18.4615f, -53.9529f, -49.4604f)); 
  data[13].mAabb = Aabb(Vector3(82.481f, -15.2157f, -11.9407f), Vector3(90.385f, -11.9028f, -4.99693f)); 
  data[14].mAabb = Aabb(Vector3(61.3646f, 37.6824f, -31.8181f), Vector3(65.3395f, 44.0438f, -30.8195f)); 
  data[15].mAabb = Aabb(Vector3(-19.5733f, -48.2098f, -78.5275f), Vector3(-14.6621f, -43.957f, -70.6101f)); 
  data[16].mAabb = Aabb(Vector3(-54.1906f, 55.1775f, -47.0798f), Vector3(-48.1008f, 58.4315f, -40.5482f)); 
  data[17].mAabb = Aabb(Vector3(34.5776f, 69.0004f, 34.7856f), Vector3(36.8695f, 75.8808f, 36.128f)); 
  data[18].mAabb = Aabb(Vector3(-58.5108f, 46.6271f, -7.07517f), Vector3(-54.7704f, 54.3998f, -4.15166f)); 
  data[19].mAabb = Aabb(Vector3(-33.6803f, -55.8654f, -17.9623f), Vector3(-29.9925f, -48.5041f, -13.9144f)); 
  data[20].mAabb = Aabb(Vector3(-20.0033f, -14.5585f, 79.4019f), Vector3(-12.556f, -9.22423f, 81.393f)); 
  data[21].mAabb = Aabb(Vector3(-37.9238f, -2.72794f, 84.2095f), Vector3(-31.8403f, 0.640438f, 91.946f)); 
  data[22].mAabb = Aabb(Vector3(-41.7959f, 75.9146f, -37.9501f), Vector3(-37.9409f, 81.4551f, -36.2316f)); 
  data[23].mAabb = Aabb(Vector3(-15.5882f, 15.3682f, -57.134f), Vector3(-9.30483f, 19.4661f, -52.1455f)); 
  data[24].mAabb = Aabb(Vector3(48.5019f, -16.6508f, 73.537f), Vector3(54.8667f, -13.3158f, 80.3734f)); 
  data[25].mAabb = Aabb(Vector3(-12.9716f, -13.2273f, 32.8844f), Vector3(-5.68413f, -10.632f, 39.6187f)); 
  data[26].mAabb = Aabb(Vector3(-74.8398f, -22.0059f, 31.0759f), Vector3(-67.6228f, -14.2033f, 37.1491f)); 
  data[27].mAabb = Aabb(Vector3(27.871f, 16.3893f, 17.7398f), Vector3(31.2944f, 20.5952f, 23.0895f)); 
  data[28].mAabb = Aabb(Vector3(-35.2869f, -3.49175f, -47.6016f), Vector3(-33.4978f, 2.83225f, -41.0607f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[28].mAabb = Aabb(Vector3(-25.9029f, -41.0498f, -55.9f), Vector3(-24.3894f, -35.969f, -55.2212f)); 
  spatialPartition.UpdateData(keys[28], data[28]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[22].mAabb = Aabb(Vector3(92.0694f, -5.44946f, 15.3057f), Vector3(95.4742f, 1.43908f, 19.7607f)); 
  spatialPartition.UpdateData(keys[22], data[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[19]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[20]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[25]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[16].mAabb = Aabb(Vector3(-91.1178f, -21.7108f, 1.62358f), Vector3(-86.5867f, -20.2766f, 8.8758f)); 
  spatialPartition.UpdateData(keys[16], data[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(-12.9938f, -6.04718f, 61.2576f), Vector3(-6.72846f, -0.105418f, 66.2229f)); 
  data[1].mAabb = Aabb(Vector3(-96.6874f, 4.9621f, 12.5323f), Vector3(-91.859f, 9.41897f, 13.6197f)); 
  data[2].mAabb = Aabb(Vector3(4.66005f, 59.0487f, 69.4076f), Vector3(6.53797f, 62.4614f, 74.5708f)); 
  data[3].mAabb = Aabb(Vector3(-42.4695f, 15.9287f, 34.3198f), Vector3(-40.5128f, 21.1611f, 41.2058f)); 
  data[4].mAabb = Aabb(Vector3(35.6927f, 54.3932f, -30.2001f), Vector3(42.2141f, 59.5658f, -28.0612f)); 
  data[5].mAabb = Aabb(Vector3(-6.55905f, -28.7918f, 21.1325f), Vector3(-4.39213f, -22.3667f, 21.8651f)); 
  data[6].mAabb = Aabb(Vector3(2.11f, -75.3414f, -55.2192f), Vector3(9.59023f, -68.5053f, -51.5769f)); 
  data[7].mAabb = Aabb(Vector3(-91.9851f, 38.4919f, -15.9117f), Vector3(-85.8697f, 39.8185f, -10.3427f)); 
  data[8].mAabb = Aabb(Vector3(-24.7576f, -9.20611f, 34.3081f), Vector3(-23.1895f, -3.07834f, 36.9102f)); 
  data[9].mAabb = Aabb(Vector3(22.0001f, -55.8073f, 64.1997f), Vector3(29.4923f, -48.0566f, 69.7057f)); 
  data[10].mAabb = Aabb(Vector3(-10.5092f, -35.3794f, -71.9334f), Vector3(-2.60519f, -29.9755f, -65.323f)); 
  data[11].mAabb = Aabb(Vector3(-15.5397f, -17.3471f, -6.12147f), Vector3(-9.43903f, -16.1063f, -5.27545f)); 
  data[12].mAabb = Aabb(Vector3(26.3275f, -12.3554f, -7.8961f), Vector3(32.7513f, -10.9894f, -0.861438f)); 
  data[13].mAabb = Aabb(Vector3(40.2205f, 13.3575f, -21.7254f), Vector3(42.8262f, 15.4362f, -17.2101f)); 
  data[14].mAabb = Aabb(Vector3(45.6338f, 12.4551f, 6.79988f), Vector3(47.9763f, 19.7801f, 9.27366f)); 
  data[15].mAabb = Aabb(Vector3(-52.3864f, 58.6566f, 47.0484f), Vector3(-50.1232f, 64.8036f, 52.3694f)); 
  data[16].mAabb = Aabb(Vector3(2.54675f, -33.292f, 90.2724f), Vector3(5.1256f, -28.1122f, 95.3423f)); 
  data[17].mAabb = Aabb(Vector3(5.598f, -66.3024f, 46.1595f), Vector3(11.1385f, -65.5886f, 49.4049f)); 
  data[18].mAabb = Aabb(Vector3(-33.99f, 19.2242f, 3.16196f), Vector3(-28.4391f, 20.8996f, 10.7489f)); 
  data[19].mAabb = Aabb(Vector3(-46.4171f, 13.5367f, -64.0707f), Vector3(-41.074f, 19.5406f, -58.6824f)); 
  data[20].mAabb = Aabb(Vector3(-49.4332f, 20.8803f, 50.0152f), Vector3(-46.2718f, 23.3798f, 56.9861f)); 
  data[21].mAabb = Aabb(Vector3(65.397f, -38.341f, 44.6206f), Vector3(72.1582f, -34.7293f, 48.7931f)); 
  data[22].mAabb = Aabb(Vector3(65.2925f, -3.85847f, 9.31846f), Vector3(70.4751f, -1.07714f, 10.8216f)); 
  data[23].mAabb = Aabb(Vector3(-27.9269f, 21.7563f, 2.17893f), Vector3(-22.4831f, 25.9828f, 3.40093f)); 
  data[24].mAabb = Aabb(Vector3(-0.141321f, 23.0818f, 60.8732f), Vector3(1.10828f, 28.237f, 64.9311f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[22].mAabb = Aabb(Vector3(19.1785f, 0.18405f, 32.9371f), Vector3(22.9569f, 5.14841f, 39.806f)); 
  spatialPartition.UpdateData(keys[22], data[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[8].mAabb = Aabb(Vector3(-89.9165f, -12.4838f, 43.8668f), Vector3(-83.3801f, -9.80242f, 46.151f)); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[4].mAabb = Aabb(Vector3(-67.1843f, -2.71233f, -20.5284f), Vector3(-65.7951f, 1.18232f, -19.3841f)); 
  spatialPartition.UpdateData(keys[4], data[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(24); 
  data[0].mAabb = Aabb(Vector3(9.97861f, 8.06335f, -7.11324f), Vector3(13.3667f, 11.8307f, -1.66412f)); 
  data[1].mAabb = Aabb(Vector3(45.3171f, -60.911f, -53.2691f), Vector3(50.5939f, -53.5484f, -49.1973f)); 
  data[2].mAabb = Aabb(Vector3(-5.48414f, -16.7674f, 93.0301f), Vector3(1.25688f, -9.90343f, 97.566f)); 
  data[3].mAabb = Aabb(Vector3(40.7826f, 10.6758f, 81.5587f), Vector3(48.0698f, 18.3666f, 88.3519f)); 
  data[4].mAabb = Aabb(Vector3(23.1471f, -25.271f, 61.2228f), Vector3(25.3214f, -20.7443f, 64.4603f)); 
  data[5].mAabb = Aabb(Vector3(67.2105f, -58.7313f, -45.3994f), Vector3(71.3208f, -51.6894f, -43.8984f)); 
  data[6].mAabb = Aabb(Vector3(45.0004f, 27.012f, -22.6399f), Vector3(52.7481f, 28.8725f, -20.4481f)); 
  data[7].mAabb = Aabb(Vector3(69.1655f, 46.0353f, -47.2887f), Vector3(71.8453f, 48.9927f, -42.1465f)); 
  data[8].mAabb = Aabb(Vector3(50.5799f, 53.2983f, -21.8571f), Vector3(55.9923f, 56.8745f, -19.2891f)); 
  data[9].mAabb = Aabb(Vector3(-30.1687f, 11.5621f, -82.4121f), Vector3(-29.1877f, 15.2963f, -77.3443f)); 
  data[10].mAabb = Aabb(Vector3(-93.5737f, 11.4927f, 17.7352f), Vector3(-90.0457f, 19.3909f, 22.1241f)); 
  data[11].mAabb = Aabb(Vector3(-55.9032f, 74.6097f, 6.37611f), Vector3(-47.993f, 80.5053f, 13.0489f)); 
  data[12].mAabb = Aabb(Vector3(-14.8567f, -18.1919f, 81.9389f), Vector3(-14.1503f, -10.5177f, 87.9938f)); 
  data[13].mAabb = Aabb(Vector3(3.20124f, 65.4485f, 50.6539f), Vector3(4.12543f, 71.7396f, 58.1989f)); 
  data[14].mAabb = Aabb(Vector3(-14.7024f, -12.0816f, -32.4461f), Vector3(-13.845f, -6.98599f, -31.4136f)); 
  data[15].mAabb = Aabb(Vector3(45.7996f, -36.2048f, 20.4361f), Vector3(49.4438f, -32.2651f, 22.8201f)); 
  data[16].mAabb = Aabb(Vector3(86.8741f, 28.4515f, 4.43245f), Vector3(91.9632f, 33.0937f, 6.30342f)); 
  data[17].mAabb = Aabb(Vector3(0.941022f, -42.6145f, -87.0966f), Vector3(6.71161f, -38.8849f, -81.9399f)); 
  data[18].mAabb = Aabb(Vector3(-0.949703f, 74.8488f, -49.635f), Vector3(0.181606f, 80.7117f, -45.7266f)); 
  data[19].mAabb = Aabb(Vector3(-51.7077f, -6.22153f, 16.6373f), Vector3(-45.4294f, 1.26263f, 19.3811f)); 
  data[20].mAabb = Aabb(Vector3(-13.5678f, 25.8366f, -72.7701f), Vector3(-8.42555f, 32.043f, -70.3407f)); 
  data[21].mAabb = Aabb(Vector3(-16.6506f, -55.7505f, -50.7001f), Vector3(-15.5988f, -51.2408f, -49.3501f)); 
  data[22].mAabb = Aabb(Vector3(61.5639f, -15.171f, -3.37207f), Vector3(63.3242f, -10.0577f, 2.66826f)); 
  data[23].mAabb = Aabb(Vector3(28.3297f, -17.0594f, 7.06479f), Vector3(34.2508f, -9.12364f, 10.9979f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[15]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[1].mAabb = Aabb(Vector3(82.8362f, 25.029f, 8.44085f), Vector3(86.7512f, 27.0741f, 13.2744f)); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[18].mAabb = Aabb(Vector3(37.7496f, 47.0319f, 24.5942f), Vector3(41.9346f, 51.1583f, 25.909f)); 
  spatialPartition.UpdateData(keys[18], data[18]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[10].mAabb = Aabb(Vector3(15.669f, 49.0132f, -29.6719f), Vector3(23.5234f, 53.0373f, -26.5916f)); 
  spatialPartition.UpdateData(keys[10], data[10]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(17); 
  data[0].mAabb = Aabb(Vector3(-16.3596f, 65.6675f, 29.5101f), Vector3(-13.7731f, 73.403f, 35.0666f)); 
  data[1].mAabb = Aabb(Vector3(-57.6462f, 26.4261f, -71.34f), Vector3(-56.8876f, 33.419f, -68.7524f)); 
  data[2].mAabb = Aabb(Vector3(-23.1992f, 73.9538f, -5.98973f), Vector3(-21.5265f, 79.7471f, 1.43969f)); 
  data[3].mAabb = Aabb(Vector3(-8.01743f, -27.0309f, -89.9873f), Vector3(-4.25429f, -22.0088f, -88.2027f)); 
  data[4].mAabb = Aabb(Vector3(-70.4166f, 4.25715f, 60.3289f), Vector3(-66.3572f, 6.54445f, 63.7958f)); 
  data[5].mAabb = Aabb(Vector3(58.7815f, 43.3063f, 4.19394f), Vector3(66.6166f, 46.6231f, 6.56474f)); 
  data[6].mAabb = Aabb(Vector3(50.9668f, 55.2321f, 36.9152f), Vector3(54.0752f, 62.2222f, 43.0947f)); 
  data[7].mAabb = Aabb(Vector3(-30.7249f, 32.2069f, -90.2017f), Vector3(-29.661f, 33.1014f, -82.9704f)); 
  data[8].mAabb = Aabb(Vector3(-27.7381f, -40.4618f, 67.8564f), Vector3(-20.8705f, -33.1741f, 75.274f)); 
  data[9].mAabb = Aabb(Vector3(-26.5276f, -28.6106f, -2.29892f), Vector3(-23.788f, -20.6382f, 3.36265f)); 
  data[10].mAabb = Aabb(Vector3(-71.9336f, -2.15317f, -1.29654f), Vector3(-69.871f, 0.902321f, 0.703621f)); 
  data[11].mAabb = Aabb(Vector3(-51.2992f, 5.62972f, -70.1871f), Vector3(-46.3937f, 9.37941f, -62.32f)); 
  data[12].mAabb = Aabb(Vector3(-13.5722f, -36.7249f, -9.98734f), Vector3(-12.4033f, -31.6344f, -3.32147f)); 
  data[13].mAabb = Aabb(Vector3(54.738f, -83.0839f, -1.59572f), Vector3(61.1153f, -78.6755f, 5.62241f)); 
  data[14].mAabb = Aabb(Vector3(55.3465f, 19.8552f, 8.35346f), Vector3(61.424f, 22.4206f, 13.1123f)); 
  data[15].mAabb = Aabb(Vector3(-52.9412f, -42.0034f, -69.9028f), Vector3(-47.9072f, -36.5162f, -62.129f)); 
  data[16].mAabb = Aabb(Vector3(-67.5851f, -30.1847f, -42.7148f), Vector3(-60.72f, -22.6173f, -41.0475f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[2].mAabb = Aabb(Vector3(43.6115f, -33.033f, 78.0292f), Vector3(49.7406f, -31.531f, 80.2681f)); 
  spatialPartition.UpdateData(keys[2], data[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[0].mAabb = Aabb(Vector3(-27.9266f, -14.8232f, -65.9139f), Vector3(-25.6634f, -12.917f, -59.0984f)); 
  spatialPartition.UpdateData(keys[0], data[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[14].mAabb = Aabb(Vector3(-100.888f, 2.58227f, -3.08125f), Vector3(-96.429f, 5.63288f, 0.507474f)); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(18); 
  data[0].mAabb = Aabb(Vector3(-50.2213f, 79.2093f, -0.139205f), Vector3(-49.264f, 84.7096f, 5.47227f)); 
  data[1].mAabb = Aabb(Vector3(-21.6353f, 38.1551f, 62.3936f), Vector3(-21.0581f, 43.9459f, 70.2127f)); 
  data[2].mAabb = Aabb(Vector3(-56.7803f, 20.3802f, 22.7799f), Vector3(-49.0651f, 22.3976f, 23.5473f)); 
  data[3].mAabb = Aabb(Vector3(47.1048f, -71.8133f, 8.04385f), Vector3(53.5972f, -65.2075f, 14.7802f)); 
  data[4].mAabb = Aabb(Vector3(27.018f, -72.0728f, -38.4505f), Vector3(29.5003f, -71.0584f, -31.9085f)); 
  data[5].mAabb = Aabb(Vector3(59.1326f, -54.5099f, 52.1682f), Vector3(60.5889f, -53.858f, 59.4835f)); 
  data[6].mAabb = Aabb(Vector3(27.0779f, 67.1614f, -67.9938f), Vector3(31.0797f, 70.2299f, -61.401f)); 
  data[7].mAabb = Aabb(Vector3(12.4423f, 32.9837f, -47.9716f), Vector3(12.9562f, 33.7994f, -42.1366f)); 
  data[8].mAabb = Aabb(Vector3(-52.6318f, 26.4745f, -54.3418f), Vector3(-49.2498f, 33.8785f, -48.5445f)); 
  data[9].mAabb = Aabb(Vector3(-13.1191f, 64.7117f, 65.3604f), Vector3(-11.9605f, 71.62f, 72.3449f)); 
  data[10].mAabb = Aabb(Vector3(25.2392f, -22.5655f, -47.0858f), Vector3(31.46f, -20.1907f, -42.4358f)); 
  data[11].mAabb = Aabb(Vector3(-41.7526f, 10.4027f, 0.515295f), Vector3(-38.4367f, 12.1706f, 4.71959f)); 
  data[12].mAabb = Aabb(Vector3(-35.5133f, -14.7169f, -23.8415f), Vector3(-33.3886f, -8.16327f, -19.5878f)); 
  data[13].mAabb = Aabb(Vector3(-2.0068f, 59.2872f, 1.54578f), Vector3(4.94808f, 66.1609f, 3.44806f)); 
  data[14].mAabb = Aabb(Vector3(83.172f, 2.63181f, -28.687f), Vector3(88.0164f, 7.01284f, -27.8737f)); 
  data[15].mAabb = Aabb(Vector3(59.1343f, 21.131f, -32.3624f), Vector3(62.3797f, 26.4544f, -30.0041f)); 
  data[16].mAabb = Aabb(Vector3(-9.91123f, 49.6892f, 80.0369f), Vector3(-3.88157f, 53.8339f, 80.789f)); 
  data[17].mAabb = Aabb(Vector3(-11.3525f, -14.3654f, -43.0585f), Vector3(-9.58386f, -6.91204f, -37.5336f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[16].mAabb = Aabb(Vector3(-55.8659f, -47.6269f, -72.6351f), Vector3(-50.526f, -44.3295f, -66.4869f)); 
  spatialPartition.UpdateData(keys[16], data[16]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[0].mAabb = Aabb(Vector3(-84.8865f, 29.749f, 37.9513f), Vector3(-79.6378f, 36.634f, 38.6792f)); 
  spatialPartition.UpdateData(keys[0], data[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[2].mAabb = Aabb(Vector3(-29.7829f, 94.4061f, -11.9294f), Vector3(-23.7962f, 96.8064f, -10.3106f)); 
  spatialPartition.UpdateData(keys[2], data[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[8].mAabb = Aabb(Vector3(-23.2693f, -77.8139f, -60.7708f), Vector3(-20.8683f, -74.9574f, -54.7365f)); 
  spatialPartition.UpdateData(keys[8], data[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[14].mAabb = Aabb(Vector3(-28.3909f, 48.6629f, -50.7264f), Vector3(-21.7085f, 49.8856f, -48.5328f)); 
  spatialPartition.UpdateData(keys[14], data[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[13].mAabb = Aabb(Vector3(-37.243f, -18.53f, -3.83783f), Vector3(-34.4616f, -14.9652f, 1.99445f)); 
  spatialPartition.UpdateData(keys[13], data[13]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[3].mAabb = Aabb(Vector3(-18.7362f, -40.225f, -59.2401f), Vector3(-16.7808f, -35.9314f, -56.6461f)); 
  spatialPartition.UpdateData(keys[3], data[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(-26.5018f, 35.4158f, -1.72492f), Vector3(-21.9074f, 36.4404f, 1.42613f)); 
  data[1].mAabb = Aabb(Vector3(-97.3458f, -26.5719f, -1.81157f), Vector3(-91.1543f, -21.8005f, 5.95208f)); 
  data[2].mAabb = Aabb(Vector3(-23.8831f, 51.7618f, 48.4016f), Vector3(-22.2893f, 55.2798f, 55.3897f)); 
  data[3].mAabb = Aabb(Vector3(-7.60496f, 49.1439f, 21.3393f), Vector3(-5.39049f, 54.1384f, 22.6921f)); 
  data[4].mAabb = Aabb(Vector3(-45.3583f, 59.1235f, -8.46758f), Vector3(-39.378f, 61.4456f, -1.85227f)); 
  data[5].mAabb = Aabb(Vector3(13.3414f, 57.4186f, -29.5226f), Vector3(16.713f, 60.4265f, -21.6919f)); 
  data[6].mAabb = Aabb(Vector3(8.49529f, -28.8027f, -83.0179f), Vector3(15.3115f, -25.4847f, -82.437f)); 
  data[7].mAabb = Aabb(Vector3(90.2785f, -25.5521f, -9.51789f), Vector3(98.1392f, -22.7029f, -1.72548f)); 
  data[8].mAabb = Aabb(Vector3(-43.8873f, 39.6374f, 34.6019f), Vector3(-37.329f, 43.729f, 42.4681f)); 
  data[9].mAabb = Aabb(Vector3(24.3661f, -4.91566f, 69.8274f), Vector3(30.1385f, 0.469208f, 75.2468f)); 
  data[10].mAabb = Aabb(Vector3(54.9432f, -39.9699f, -48.3024f), Vector3(57.1092f, -35.9184f, -45.9233f)); 
  data[11].mAabb = Aabb(Vector3(83.9197f, -18.7018f, -19.7965f), Vector3(88.394f, -11.4878f, -15.5813f)); 
  data[12].mAabb = Aabb(Vector3(-22.7559f, 21.8478f, -36.3445f), Vector3(-15.5586f, 27.4451f, -32.8435f)); 
  data[13].mAabb = Aabb(Vector3(0.0560186f, -20.528f, -17.7879f), Vector3(2.28557f, -17.6746f, -14.4865f)); 
  data[14].mAabb = Aabb(Vector3(-43.263f, -4.1674f, 73.1563f), Vector3(-36.9156f, 0.983214f, 80.2033f)); 
  data[15].mAabb = Aabb(Vector3(-0.88227f, 72.0094f, -2.05376f), Vector3(0.140954f, 78.127f, 2.47989f)); 
  data[16].mAabb = Aabb(Vector3(-28.6617f, 48.2027f, 72.4061f), Vector3(-24.9638f, 55.2603f, 79.6096f)); 
  data[17].mAabb = Aabb(Vector3(47.4275f, 29.4874f, -77.3075f), Vector3(48.7782f, 33.4851f, -75.6045f)); 
  data[18].mAabb = Aabb(Vector3(12.4553f, -6.76721f, -86.3091f), Vector3(15.3925f, -0.787409f, -79.7019f)); 
  data[19].mAabb = Aabb(Vector3(-28.0658f, -1.26445f, 38.3803f), Vector3(-26.5326f, 2.10207f, 41.262f)); 
  data[20].mAabb = Aabb(Vector3(78.0664f, -22.4397f, -43.7978f), Vector3(82.8795f, -21.2553f, -37.0726f)); 
  data[21].mAabb = Aabb(Vector3(64.0491f, 16.3657f, -9.98162f), Vector3(69.1744f, 24.1741f, -6.0044f)); 
  data[22].mAabb = Aabb(Vector3(45.9385f, -53.3123f, -71.2984f), Vector3(50.4204f, -49.7655f, -66.3986f)); 
  data[23].mAabb = Aabb(Vector3(3.35285f, -16.3398f, -65.354f), Vector3(9.12391f, -15.0457f, -62.7939f)); 
  data[24].mAabb = Aabb(Vector3(-12.1977f, -80.3717f, 11.6232f), Vector3(-6.58015f, -72.898f, 17.0441f)); 
  data[25].mAabb = Aabb(Vector3(-59.4704f, -65.4685f, 24.189f), Vector3(-53.1905f, -61.4568f, 24.6108f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[3]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[23]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[11].mAabb = Aabb(Vector3(55.8604f, 65.3615f, -46.3429f), Vector3(57.9784f, 72.7909f, -41.5015f)); 
  spatialPartition.UpdateData(keys[11], data[11]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[2]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[22]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[8]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[14]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[25].mAabb = Aabb(Vector3(-12.6524f, -64.4687f, 63.6388f), Vector3(-11.883f, -61.9444f, 68.1794f)); 
  spatialPartition.UpdateData(keys[25], data[25]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeStructureFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(18); 
  data[0].mAabb = Aabb(Vector3(-72.5516f, 45.944f, 29.3277f), Vector3(-65.1403f, 46.7898f, 36.656f)); 
  data[1].mAabb = Aabb(Vector3(-35.7468f, 51.7426f, -56.7357f), Vector3(-33.2988f, 55.9244f, -53.7074f)); 
  data[2].mAabb = Aabb(Vector3(62.9245f, -28.1661f, 33.6097f), Vector3(63.3967f, -24.6001f, 41.5728f)); 
  data[3].mAabb = Aabb(Vector3(40.3652f, 45.4427f, -71.9316f), Vector3(41.1231f, 48.5005f, -70.0004f)); 
  data[4].mAabb = Aabb(Vector3(15.797f, -33.5932f, 82.4532f), Vector3(23.098f, -25.984f, 86.7419f)); 
  data[5].mAabb = Aabb(Vector3(-25.0802f, 65.6274f, -40.2565f), Vector3(-23.103f, 71.4499f, -39.3024f)); 
  data[6].mAabb = Aabb(Vector3(-55.2277f, -75.2337f, 13.806f), Vector3(-48.6006f, -73.651f, 15.2943f)); 
  data[7].mAabb = Aabb(Vector3(-85.7162f, 29.3733f, 8.38983f), Vector3(-78.1251f, 33.8293f, 9.04357f)); 
  data[8].mAabb = Aabb(Vector3(10.5464f, -18.7974f, -29.5108f), Vector3(16.882f, -11.9313f, -27.7817f)); 
  data[9].mAabb = Aabb(Vector3(13.4469f, 46.9247f, -74.3223f), Vector3(21.2962f, 48.8247f, -67.7958f)); 
  data[10].mAabb = Aabb(Vector3(-33.3522f, 53.3134f, -9.36654f), Vector3(-26.3066f, 56.8314f, -8.08354f)); 
  data[11].mAabb = Aabb(Vector3(60.4308f, 54.4801f, -8.67803f), Vector3(63.4694f, 54.9764f, -2.93992f)); 
  data[12].mAabb = Aabb(Vector3(-46.1053f, -9.6106f, -46.6356f), Vector3(-40.1443f, -8.23483f, -46.15f)); 
  data[13].mAabb = Aabb(Vector3(70.576f, -49.3529f, -50.2285f), Vector3(71.3564f, -47.8076f, -48.525f)); 
  data[14].mAabb = Aabb(Vector3(-31.6433f, 23.2981f, -60.1784f), Vector3(-26.986f, 23.8493f, -56.6809f)); 
  data[15].mAabb = Aabb(Vector3(74.2092f, -67.7494f, 3.55967f), Vector3(74.7252f, -62.4564f, 5.33021f)); 
  data[16].mAabb = Aabb(Vector3(-69.5185f, -68.063f, 19.3933f), Vector3(-66.0589f, -60.7264f, 23.2239f)); 
  data[17].mAabb = Aabb(Vector3(83.7511f, 24.2756f, -11.0356f), Vector3(86.6992f, 28.7371f, -9.55611f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[12].mAabb = Aabb(Vector3(1.93068f, 33.345f, 22.3137f), Vector3(2.38356f, 35.7399f, 28.5512f)); 
  spatialPartition.UpdateData(keys[12], data[12]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[5]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[0]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[1].mAabb = Aabb(Vector3(-30.6429f, 13.9617f, 48.836f), Vector3(-26.6634f, 15.6047f, 54.2413f)); 
  spatialPartition.UpdateData(keys[1], data[1]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[7]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  data[17].mAabb = Aabb(Vector3(74.3021f, -38.1849f, 32.291f), Vector3(80.8582f, -36.1702f, 39.8157f)); 
  spatialPartition.UpdateData(keys[17], data[17]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);

  spatialPartition.RemoveData(keys[4]);
  PrintSpatialPartitionStructure(spatialPartition, PrintAabbData, file, false);
}

void DynamicAabbTreeRayCastFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(5.78385f, 11.2187f, -20.5214f), Vector3(7.58454f, 16.1407f, -15.0127f)); 
  data[1].mAabb = Aabb(Vector3(-16.2056f, -39.0694f, 16.5112f), Vector3(-11.9646f, -37.715f, 19.0813f)); 
  data[2].mAabb = Aabb(Vector3(-11.7538f, 25.6054f, -6.08261f), Vector3(-5.71533f, 29.3252f, 0.131674f)); 
  data[3].mAabb = Aabb(Vector3(2.10622f, -9.29249f, -21.7373f), Vector3(7.61588f, -5.79446f, -19.2978f)); 
  data[4].mAabb = Aabb(Vector3(-0.422651f, -8.83493f, -30.2156f), Vector3(6.83769f, -4.13591f, -28.9544f)); 
  data[5].mAabb = Aabb(Vector3(10.7869f, -19.9303f, 11.5446f), Vector3(13.7382f, -17.2454f, 14.0256f)); 
  data[6].mAabb = Aabb(Vector3(15.6756f, 17.8036f, -9.15831f), Vector3(20.9677f, 19.9037f, -1.38283f)); 
  data[7].mAabb = Aabb(Vector3(-38.8856f, 4.2038f, -26.1935f), Vector3(-34.4459f, 5.06954f, -18.6681f)); 
  data[8].mAabb = Aabb(Vector3(6.11565f, 20.1767f, -29.8111f), Vector3(13.4716f, 24.0728f, -21.8902f)); 
  data[9].mAabb = Aabb(Vector3(-13.1203f, 39.7542f, -19.963f), Vector3(-9.20619f, 40.9962f, -16.4497f)); 
  data[10].mAabb = Aabb(Vector3(-43.5262f, 12.5725f, 0.338448f), Vector3(-36.4865f, 19.1064f, 1.26124f)); 
  data[11].mAabb = Aabb(Vector3(33.4155f, -31.8208f, -8.99941f), Vector3(37.2341f, -27.2243f, -3.45149f)); 
  data[12].mAabb = Aabb(Vector3(1.69863f, 33.9889f, -25.3569f), Vector3(6.00799f, 40.1371f, -18.8789f)); 
  data[13].mAabb = Aabb(Vector3(27.3745f, -8.99399f, 6.43565f), Vector3(33.6401f, -8.33955f, 13.3001f)); 
  data[14].mAabb = Aabb(Vector3(6.46523f, 26.9755f, 19.1266f), Vector3(9.14219f, 29.2843f, 26.0885f)); 
  data[15].mAabb = Aabb(Vector3(-8.51152f, -23.033f, 14.8638f), Vector3(-6.3759f, -19.3986f, 15.294f)); 
  data[16].mAabb = Aabb(Vector3(-10.6674f, -23.5108f, 10.0133f), Vector3(-3.63205f, -16.1463f, 12.0628f)); 
  data[17].mAabb = Aabb(Vector3(16.6675f, -14.1139f, 30.1814f), Vector3(22.9946f, -11.5659f, 35.6713f)); 
  data[18].mAabb = Aabb(Vector3(-18.0872f, -6.40865f, 12.2195f), Vector3(-14.2129f, -4.16612f, 15.7806f)); 
  data[19].mAabb = Aabb(Vector3(-5.9958f, 0.394307f, -29.2936f), Vector3(-4.82994f, 5.01354f, -24.5317f)); 
  data[20].mAabb = Aabb(Vector3(-35.4381f, 31.831f, -3.96975f), Vector3(-34.8479f, 35.6058f, 3.37223f)); 
  data[21].mAabb = Aabb(Vector3(29.1808f, 34.5308f, -10.1397f), Vector3(32.5737f, 38.9086f, -4.33295f)); 
  data[22].mAabb = Aabb(Vector3(-26.0525f, -31.1201f, -23.3776f), Vector3(-19.2511f, -29.1191f, -17.7844f)); 
  data[23].mAabb = Aabb(Vector3(-9.86672f, 19.5673f, -21.1468f), Vector3(-7.13316f, 23.1604f, -18.0727f)); 
  data[24].mAabb = Aabb(Vector3(-32.1155f, -40.1872f, -3.81265f), Vector3(-28.1383f, -39.2628f, -3.34794f)); 
  data[25].mAabb = Aabb(Vector3(-18.3438f, 6.88972f, 17.6069f), Vector3(-15.6734f, 12.7016f, 18.5499f)); 
  data[26].mAabb = Aabb(Vector3(-1.26006f, -48.1578f, -16.1727f), Vector3(5.44525f, -43.3878f, -9.85082f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-10.7378f, -14.7255f, -6.10565f));
  ray.mDirection = Vector3(Vector3(0.107446f, -0.314813f, 0.943052f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(42.1216f, -7.28003f, 14.3232f), Vector3(42.8486f, 0.298998f, 22.2402f)); 
  data[1].mAabb = Aabb(Vector3(-5.00704f, 0.970592f, 44.4351f), Vector3(-0.491954f, 7.66616f, 45.1357f)); 
  data[2].mAabb = Aabb(Vector3(2.99788f, 42.5164f, 19.3399f), Vector3(7.2794f, 44.0502f, 26.7717f)); 
  data[3].mAabb = Aabb(Vector3(19.7632f, 11.5091f, -5.93565f), Vector3(25.6781f, 14.8186f, -5.48856f)); 
  data[4].mAabb = Aabb(Vector3(32.1666f, -1.26836f, -6.81882f), Vector3(35.5704f, 4.58967f, 0.841158f)); 
  data[5].mAabb = Aabb(Vector3(24.8151f, 16.0722f, 27.9524f), Vector3(28.1595f, 17.3399f, 31.3509f)); 
  data[6].mAabb = Aabb(Vector3(-10.8522f, -23.3809f, -20.9883f), Vector3(-4.38558f, -22.1923f, -13.3669f)); 
  data[7].mAabb = Aabb(Vector3(-24.1366f, 18.6398f, 20.7355f), Vector3(-16.405f, 25.1055f, 21.9665f)); 
  data[8].mAabb = Aabb(Vector3(28.8748f, 18.1119f, 26.0535f), Vector3(30.3867f, 19.7757f, 29.5613f)); 
  data[9].mAabb = Aabb(Vector3(24.0502f, 0.84784f, 7.08107f), Vector3(29.3873f, 1.38213f, 10.6918f)); 
  data[10].mAabb = Aabb(Vector3(23.1024f, 26.6849f, 3.04002f), Vector3(25.5177f, 33.368f, 10.0891f)); 
  data[11].mAabb = Aabb(Vector3(-48.4823f, -10.1524f, -15.8362f), Vector3(-43.1023f, -7.9377f, -14.5144f)); 
  data[12].mAabb = Aabb(Vector3(-19.9825f, -14.0444f, -3.52291f), Vector3(-12.7945f, -10.5985f, -0.781935f)); 
  data[13].mAabb = Aabb(Vector3(-42.9812f, -2.40404f, -17.6451f), Vector3(-36.1725f, 4.08904f, -10.3367f)); 
  data[14].mAabb = Aabb(Vector3(17.2326f, -36.8659f, -21.6145f), Vector3(22.2139f, -31.0756f, -20.2201f)); 
  data[15].mAabb = Aabb(Vector3(18.9165f, -24.18f, 29.8541f), Vector3(26.3984f, -22.4686f, 30.448f)); 
  data[16].mAabb = Aabb(Vector3(-15.0165f, -23.2226f, -20.92f), Vector3(-10.057f, -18.5948f, -19.2559f)); 
  data[17].mAabb = Aabb(Vector3(-5.75247f, 8.28146f, -42.6563f), Vector3(-5.33507f, 10.9468f, -38.0157f)); 
  data[18].mAabb = Aabb(Vector3(-15.4252f, -18.4096f, -18.9492f), Vector3(-12.2313f, -12.9055f, -17.1207f)); 
  data[19].mAabb = Aabb(Vector3(35.5846f, 7.97923f, 10.667f), Vector3(39.9645f, 11.4775f, 14.1775f)); 
  data[20].mAabb = Aabb(Vector3(-40.8454f, 3.98986f, -0.683858f), Vector3(-35.6915f, 9.77529f, 1.99171f)); 
  data[21].mAabb = Aabb(Vector3(36.4665f, 2.99764f, -16.3086f), Vector3(41.525f, 4.20873f, -12.9166f)); 
  data[22].mAabb = Aabb(Vector3(20.49f, 22.1852f, -12.9125f), Vector3(25.2329f, 25.5749f, -5.95391f)); 
  data[23].mAabb = Aabb(Vector3(-17.2765f, -37.0956f, 0.449353f), Vector3(-11.1684f, -35.9019f, 4.10348f)); 
  data[24].mAabb = Aabb(Vector3(-3.16575f, 30.0864f, -15.791f), Vector3(3.89002f, 33.6598f, -13.9024f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-14.5705f, -5.18838f, 14.7989f));
  ray.mDirection = Vector3(Vector3(-0.00356662f, -0.327739f, -0.944762f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(43); 
  data[0].mAabb = Aabb(Vector3(-2.39938f, -21.0268f, -4.15299f), Vector3(0.0637314f, -16.7821f, -0.464766f)); 
  data[1].mAabb = Aabb(Vector3(-33.3021f, -12.6946f, 11.829f), Vector3(-31.4369f, -10.2603f, 17.71f)); 
  data[2].mAabb = Aabb(Vector3(-0.520029f, -33.0547f, 11.7705f), Vector3(0.924168f, -32.2963f, 18.9321f)); 
  data[3].mAabb = Aabb(Vector3(27.1452f, -5.70038f, 13.8839f), Vector3(33.7846f, -0.660875f, 14.947f)); 
  data[4].mAabb = Aabb(Vector3(-31.3014f, -19.4735f, -35.1162f), Vector3(-28.9718f, -16.6071f, -29.3495f)); 
  data[5].mAabb = Aabb(Vector3(-9.83178f, -12.6417f, -34.2891f), Vector3(-3.03346f, -5.88958f, -30.8164f)); 
  data[6].mAabb = Aabb(Vector3(-7.17645f, -14.4189f, -33.0522f), Vector3(-5.10276f, -11.5199f, -32.3177f)); 
  data[7].mAabb = Aabb(Vector3(-5.69045f, 0.987979f, -23.0011f), Vector3(1.34723f, 8.69063f, -18.7309f)); 
  data[8].mAabb = Aabb(Vector3(-30.3634f, 0.529116f, -9.03967f), Vector3(-25.457f, 1.39833f, -5.72279f)); 
  data[9].mAabb = Aabb(Vector3(-15.5761f, -20.774f, -15.3066f), Vector3(-13.2612f, -17.2052f, -12.7386f)); 
  data[10].mAabb = Aabb(Vector3(14.7926f, -31.963f, 23.8925f), Vector3(17.7978f, -31.2566f, 27.2331f)); 
  data[11].mAabb = Aabb(Vector3(-5.50962f, -5.80601f, 36.8517f), Vector3(-2.30013f, 1.54247f, 40.6195f)); 
  data[12].mAabb = Aabb(Vector3(-23.8197f, 31.5841f, -16.1421f), Vector3(-18.6621f, 38.5028f, -11.274f)); 
  data[13].mAabb = Aabb(Vector3(-1.82482f, 38.7542f, -28.557f), Vector3(0.453202f, 43.6951f, -24.8948f)); 
  data[14].mAabb = Aabb(Vector3(-24.6069f, -39.5794f, -12.4902f), Vector3(-17.8304f, -32.381f, -7.26092f)); 
  data[15].mAabb = Aabb(Vector3(31.599f, 27.5879f, -1.04777f), Vector3(36.2684f, 34.3424f, 2.6154f)); 
  data[16].mAabb = Aabb(Vector3(39.0234f, -17.3715f, -10.7093f), Vector3(42.6525f, -11.0874f, -8.02069f)); 
  data[17].mAabb = Aabb(Vector3(-14.2837f, 16.9669f, 16.5804f), Vector3(-7.4248f, 21.118f, 24.1008f)); 
  data[18].mAabb = Aabb(Vector3(25.2867f, 16.0078f, 22.3947f), Vector3(28.7307f, 21.3975f, 30.2008f)); 
  data[19].mAabb = Aabb(Vector3(33.3172f, 4.12036f, 32.7842f), Vector3(34.4852f, 4.6586f, 39.6765f)); 
  data[20].mAabb = Aabb(Vector3(-11.4229f, -34.9167f, 12.4651f), Vector3(-10.4655f, -27.2017f, 19.4228f)); 
  data[21].mAabb = Aabb(Vector3(-42.6572f, 0.388123f, -9.94099f), Vector3(-38.7271f, 2.31267f, -5.0091f)); 
  data[22].mAabb = Aabb(Vector3(19.3517f, -46.486f, -2.15903f), Vector3(26.7153f, -41.34f, 2.91875f)); 
  data[23].mAabb = Aabb(Vector3(16.9493f, 0.487441f, 18.9857f), Vector3(19.5124f, 8.03539f, 20.6366f)); 
  data[24].mAabb = Aabb(Vector3(-14.723f, -40.6867f, -20.9221f), Vector3(-13.7967f, -36.8202f, -16.0384f)); 
  data[25].mAabb = Aabb(Vector3(-20.7981f, 34.7736f, -20.1135f), Vector3(-18.984f, 41.7649f, -14.6924f)); 
  data[26].mAabb = Aabb(Vector3(-35.9525f, 26.9684f, -19.1743f), Vector3(-32.2288f, 32.951f, -15.1268f)); 
  data[27].mAabb = Aabb(Vector3(9.4783f, 9.44691f, 30.3937f), Vector3(13.1176f, 11.0456f, 32.3739f)); 
  data[28].mAabb = Aabb(Vector3(-15.6637f, -12.5402f, 38.4096f), Vector3(-8.78795f, -7.66141f, 42.7855f)); 
  data[29].mAabb = Aabb(Vector3(-0.486794f, 38.882f, 13.5682f), Vector3(1.68918f, 42.2826f, 19.6714f)); 
  data[30].mAabb = Aabb(Vector3(0.615631f, -39.7312f, 15.6453f), Vector3(3.04511f, -37.1125f, 16.1063f)); 
  data[31].mAabb = Aabb(Vector3(-23.6507f, -16.2953f, -3.74113f), Vector3(-20.5721f, -9.73005f, 1.9564f)); 
  data[32].mAabb = Aabb(Vector3(27.4588f, 28.8651f, 7.46528f), Vector3(31.3168f, 29.565f, 10.1302f)); 
  data[33].mAabb = Aabb(Vector3(26.8476f, -4.31347f, -17.533f), Vector3(31.1175f, 1.87229f, -10.4399f)); 
  data[34].mAabb = Aabb(Vector3(-10.0998f, -14.0492f, 13.0445f), Vector3(-8.36565f, -11.172f, 14.1455f)); 
  data[35].mAabb = Aabb(Vector3(30.0967f, -18.658f, -38.1144f), Vector3(30.6776f, -12.44f, -31.635f)); 
  data[36].mAabb = Aabb(Vector3(44.5058f, -1.34082f, -6.92459f), Vector3(47.0536f, 2.8245f, -5.45256f)); 
  data[37].mAabb = Aabb(Vector3(-4.12346f, -17.6185f, -5.24666f), Vector3(-1.5896f, -9.84092f, -0.184185f)); 
  data[38].mAabb = Aabb(Vector3(-26.3301f, 21.7245f, -37.5496f), Vector3(-20.3076f, 26.4611f, -36.1174f)); 
  data[39].mAabb = Aabb(Vector3(11.0929f, 40.7936f, -7.15214f), Vector3(13.6275f, 44.0071f, -0.257342f)); 
  data[40].mAabb = Aabb(Vector3(3.89033f, -18.8223f, -5.32316f), Vector3(4.38868f, -11.1996f, 2.60703f)); 
  data[41].mAabb = Aabb(Vector3(-5.83623f, -13.8938f, -35.6167f), Vector3(2.09975f, -8.72577f, -32.6684f)); 
  data[42].mAabb = Aabb(Vector3(-27.5155f, 30.4538f, 12.2675f), Vector3(-20.9723f, 31.3872f, 14.4657f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(16.8737f, -10.8015f, 7.97549f));
  ray.mDirection = Vector3(Vector3(-0.783636f, -0.469302f, -0.407025f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(38); 
  data[0].mAabb = Aabb(Vector3(16.1499f, 4.04015f, -46.9506f), Vector3(20.446f, 10.8851f, -41.5122f)); 
  data[1].mAabb = Aabb(Vector3(18.5495f, 6.10416f, 3.89461f), Vector3(22.2092f, 10.6923f, 8.70264f)); 
  data[2].mAabb = Aabb(Vector3(-11.7596f, 20.7883f, 6.71924f), Vector3(-7.55208f, 21.3991f, 10.3836f)); 
  data[3].mAabb = Aabb(Vector3(-7.66801f, -14.8962f, -42.8979f), Vector3(-5.70427f, -10.5801f, -39.202f)); 
  data[4].mAabb = Aabb(Vector3(-23.4159f, -20.418f, 23.7462f), Vector3(-19.5099f, -17.8199f, 25.0129f)); 
  data[5].mAabb = Aabb(Vector3(26.1552f, -7.65052f, 22.8372f), Vector3(32.2524f, -1.94882f, 30.7043f)); 
  data[6].mAabb = Aabb(Vector3(6.13804f, -1.40147f, 44.3024f), Vector3(14.1188f, 5.27972f, 49.4015f)); 
  data[7].mAabb = Aabb(Vector3(-46.5733f, 20.0248f, -13.998f), Vector3(-39.4883f, 24.6496f, -8.99115f)); 
  data[8].mAabb = Aabb(Vector3(-12.3907f, 37.074f, 9.55301f), Vector3(-4.63906f, 41.1602f, 14.8722f)); 
  data[9].mAabb = Aabb(Vector3(-25.8419f, 14.1626f, 30.0543f), Vector3(-25.326f, 19.1131f, 37.8852f)); 
  data[10].mAabb = Aabb(Vector3(13.6236f, 8.72427f, 29.6699f), Vector3(16.1758f, 14.7215f, 35.0926f)); 
  data[11].mAabb = Aabb(Vector3(-17.7025f, -28.8146f, 8.22694f), Vector3(-11.5947f, -28.1374f, 16.1142f)); 
  data[12].mAabb = Aabb(Vector3(-40.8979f, 2.16547f, 12.7008f), Vector3(-40.2377f, 7.7788f, 18.6171f)); 
  data[13].mAabb = Aabb(Vector3(-38.4876f, 8.54264f, 26.2455f), Vector3(-35.333f, 13.7895f, 26.9274f)); 
  data[14].mAabb = Aabb(Vector3(-23.5033f, -3.73314f, 29.3476f), Vector3(-17.2143f, 1.09043f, 36.2964f)); 
  data[15].mAabb = Aabb(Vector3(0.60876f, 33.455f, 7.84697f), Vector3(3.57611f, 37.7259f, 12.7884f)); 
  data[16].mAabb = Aabb(Vector3(30.2667f, 22.5853f, -6.45021f), Vector3(33.1202f, 29.0418f, 1.3582f)); 
  data[17].mAabb = Aabb(Vector3(-14.0265f, -41.5885f, -18.576f), Vector3(-9.60651f, -41.0426f, -15.9514f)); 
  data[18].mAabb = Aabb(Vector3(-1.68345f, 32.1221f, 2.30907f), Vector3(5.6416f, 38.1933f, 10.1115f)); 
  data[19].mAabb = Aabb(Vector3(-10.5135f, -30.4648f, -15.6363f), Vector3(-10.0015f, -25.6533f, -11.9402f)); 
  data[20].mAabb = Aabb(Vector3(42.4136f, -22.1774f, -2.31812f), Vector3(43.0959f, -16.3275f, -0.691385f)); 
  data[21].mAabb = Aabb(Vector3(11.6397f, -44.1768f, 21.9084f), Vector3(14.1044f, -39.86f, 24.5826f)); 
  data[22].mAabb = Aabb(Vector3(0.796785f, -47.0191f, 6.4323f), Vector3(7.6735f, -46.0638f, 10.5781f)); 
  data[23].mAabb = Aabb(Vector3(-37.2663f, 7.5785f, 5.53449f), Vector3(-31.2105f, 13.4384f, 6.09917f)); 
  data[24].mAabb = Aabb(Vector3(17.77f, 6.39971f, -13.8729f), Vector3(21.4891f, 12.9162f, -7.38797f)); 
  data[25].mAabb = Aabb(Vector3(16.4735f, 4.1401f, -10.3264f), Vector3(19.9875f, 7.10026f, -6.7305f)); 
  data[26].mAabb = Aabb(Vector3(-32.5112f, -18.0978f, -36.8685f), Vector3(-26.0047f, -10.895f, -32.1115f)); 
  data[27].mAabb = Aabb(Vector3(42.1785f, -2.58936f, 10.6852f), Vector3(45.8572f, -0.763162f, 15.8471f)); 
  data[28].mAabb = Aabb(Vector3(-28.3343f, -27.1172f, -16.8368f), Vector3(-20.5391f, -23.1657f, -13.5064f)); 
  data[29].mAabb = Aabb(Vector3(-0.0677249f, 34.2938f, -18.2359f), Vector3(4.07371f, 35.6761f, -11.4417f)); 
  data[30].mAabb = Aabb(Vector3(9.63488f, -15.2725f, -35.131f), Vector3(17.3721f, -13.4626f, -30.3777f)); 
  data[31].mAabb = Aabb(Vector3(8.85341f, 35.5199f, 15.5703f), Vector3(16.133f, 36.7361f, 16.5362f)); 
  data[32].mAabb = Aabb(Vector3(-36.6922f, 16.962f, -21.0738f), Vector3(-35.9023f, 24.0168f, -18.828f)); 
  data[33].mAabb = Aabb(Vector3(-32.2809f, -6.38263f, 25.6994f), Vector3(-31.1851f, -5.27057f, 27.6499f)); 
  data[34].mAabb = Aabb(Vector3(-35.0473f, 30.4248f, 9.32919f), Vector3(-33.8327f, 37.176f, 11.4001f)); 
  data[35].mAabb = Aabb(Vector3(10.0479f, -36.0949f, 0.885326f), Vector3(14.4248f, -30.8691f, 2.82077f)); 
  data[36].mAabb = Aabb(Vector3(25.0429f, -34.7551f, -16.9043f), Vector3(28.6138f, -28.2409f, -16.403f)); 
  data[37].mAabb = Aabb(Vector3(-17.3186f, 2.79795f, 38.1846f), Vector3(-11.4397f, 4.97114f, 45.2f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(18.8255f, 4.13852f, -7.22364f));
  ray.mDirection = Vector3(Vector3(0.235373f, 0.687906f, -0.686575f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(-30.006f, -37.297f, -21.0807f), Vector3(-26.1654f, -32.9462f, -20.3268f)); 
  data[1].mAabb = Aabb(Vector3(-27.2001f, -8.13497f, 33.9994f), Vector3(-19.217f, -3.05811f, 36.7221f)); 
  data[2].mAabb = Aabb(Vector3(10.1779f, -10.17f, 44.4559f), Vector3(17.7695f, -6.86335f, 49.8855f)); 
  data[3].mAabb = Aabb(Vector3(15.811f, -33.9642f, -12.149f), Vector3(22.6704f, -30.9322f, -6.53614f)); 
  data[4].mAabb = Aabb(Vector3(-27.1593f, -42.8653f, -6.25898f), Vector3(-24.762f, -35.2339f, 1.07349f)); 
  data[5].mAabb = Aabb(Vector3(-11.3665f, 24.8187f, 15.1819f), Vector3(-7.64164f, 31.2787f, 16.786f)); 
  data[6].mAabb = Aabb(Vector3(-27.0581f, 15.2814f, -27.7926f), Vector3(-25.6842f, 20.1224f, -22.6675f)); 
  data[7].mAabb = Aabb(Vector3(6.93327f, 6.41736f, -36.6208f), Vector3(13.4758f, 11.3632f, -36.106f)); 
  data[8].mAabb = Aabb(Vector3(-26.1252f, -21.0409f, -1.23506f), Vector3(-19.0611f, -14.8842f, 2.36456f)); 
  data[9].mAabb = Aabb(Vector3(-28.914f, 24.1043f, 0.637018f), Vector3(-24.7542f, 24.935f, 1.90935f)); 
  data[10].mAabb = Aabb(Vector3(-17.5538f, -28.7543f, 16.2552f), Vector3(-12.0345f, -26.2501f, 20.3878f)); 
  data[11].mAabb = Aabb(Vector3(-8.37911f, -20.39f, -25.6469f), Vector3(-5.85175f, -14.384f, -21.274f)); 
  data[12].mAabb = Aabb(Vector3(-15.0626f, 18.9034f, 8.29462f), Vector3(-8.4677f, 21.1897f, 8.91033f)); 
  data[13].mAabb = Aabb(Vector3(15.2743f, 23.8405f, -15.2953f), Vector3(18.2558f, 31.156f, -10.7689f)); 
  data[14].mAabb = Aabb(Vector3(-40.7047f, 20.3043f, -3.46096f), Vector3(-33.9941f, 25.6465f, 3.76876f)); 
  data[15].mAabb = Aabb(Vector3(-23.3746f, 3.09181f, 18.6607f), Vector3(-17.4208f, 10.7685f, 25.6065f)); 
  data[16].mAabb = Aabb(Vector3(-14.046f, -7.60041f, 45.9996f), Vector3(-9.09514f, -0.694934f, 49.8152f)); 
  data[17].mAabb = Aabb(Vector3(-34.485f, 25.7771f, 26.4391f), Vector3(-28.8141f, 28.8063f, 26.985f)); 
  data[18].mAabb = Aabb(Vector3(15.4723f, 6.1002f, -7.89694f), Vector3(17.428f, 13.676f, -0.280342f)); 
  data[19].mAabb = Aabb(Vector3(2.08755f, -22.084f, -43.3707f), Vector3(8.09541f, -19.853f, -35.8559f)); 
  data[20].mAabb = Aabb(Vector3(-10.5163f, -1.99062f, 34.9196f), Vector3(-4.44284f, -0.478238f, 40.0617f)); 
  data[21].mAabb = Aabb(Vector3(22.1654f, 18.3104f, 22.749f), Vector3(30.019f, 21.5143f, 28.463f)); 
  data[22].mAabb = Aabb(Vector3(18.5644f, -19.9735f, 26.641f), Vector3(25.1974f, -17.5074f, 32.7655f)); 
  data[23].mAabb = Aabb(Vector3(-12.1925f, 8.96225f, 8.11281f), Vector3(-6.43626f, 11.0153f, 11.0936f)); 
  data[24].mAabb = Aabb(Vector3(-21.6098f, 25.6729f, 8.55265f), Vector3(-14.4083f, 29.5323f, 15.081f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-6.662f, 0.415712f, 9.07621f));
  ray.mDirection = Vector3(Vector3(-0.320427f, 0.943274f, 0.0869472f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(11.597f, -3.10589f, -18.0922f), Vector3(19.3463f, 3.43521f, -11.061f)); 
  data[1].mAabb = Aabb(Vector3(-33.6293f, -18.4976f, 18.4157f), Vector3(-30.8639f, -11.7798f, 19.0321f)); 
  data[2].mAabb = Aabb(Vector3(-12.6282f, 19.2362f, 18.6435f), Vector3(-5.67585f, 20.8311f, 19.1646f)); 
  data[3].mAabb = Aabb(Vector3(-42.7455f, 11.3646f, -7.58276f), Vector3(-36.8645f, 18.1761f, -5.18204f)); 
  data[4].mAabb = Aabb(Vector3(-9.12011f, 35.4724f, 20.6125f), Vector3(-6.08295f, 36.1526f, 28.2367f)); 
  data[5].mAabb = Aabb(Vector3(-31.7083f, 13.8705f, 12.3342f), Vector3(-24.1049f, 19.934f, 19.1439f)); 
  data[6].mAabb = Aabb(Vector3(6.96519f, -27.0706f, 40.2169f), Vector3(10.116f, -25.3323f, 43.1861f)); 
  data[7].mAabb = Aabb(Vector3(18.2692f, -39.5312f, -16.8575f), Vector3(24.0795f, -35.0468f, -10.5758f)); 
  data[8].mAabb = Aabb(Vector3(15.6718f, -4.36877f, 34.939f), Vector3(18.4974f, 0.469878f, 42.2865f)); 
  data[9].mAabb = Aabb(Vector3(29.7571f, 1.27434f, -28.8985f), Vector3(36.792f, 6.90158f, -28.3976f)); 
  data[10].mAabb = Aabb(Vector3(-6.05135f, -18.5904f, -0.383873f), Vector3(0.166419f, -14.8764f, 1.57407f)); 
  data[11].mAabb = Aabb(Vector3(-14.9631f, -4.08364f, -3.78281f), Vector3(-7.11151f, -1.00264f, -2.01993f)); 
  data[12].mAabb = Aabb(Vector3(-2.89188f, 33.6134f, -25.3004f), Vector3(-2.01293f, 40.9072f, -21.8432f)); 
  data[13].mAabb = Aabb(Vector3(4.92659f, 17.488f, -36.4147f), Vector3(10.1762f, 19.0862f, -31.5665f)); 
  data[14].mAabb = Aabb(Vector3(12.3726f, -30.1668f, 22.9779f), Vector3(18.8986f, -23.7178f, 29.9502f)); 
  data[15].mAabb = Aabb(Vector3(22.9527f, 16.117f, 3.53706f), Vector3(28.315f, 21.2785f, 6.40329f)); 
  data[16].mAabb = Aabb(Vector3(44.8731f, 2.11993f, 9.31092f), Vector3(46.7382f, 4.68208f, 11.3531f)); 
  data[17].mAabb = Aabb(Vector3(-6.44676f, -22.3001f, -36.2131f), Vector3(-2.73998f, -15.3923f, -31.0774f)); 
  data[18].mAabb = Aabb(Vector3(15.7726f, -46.3069f, -8.38617f), Vector3(16.7156f, -40.7725f, -4.13549f)); 
  data[19].mAabb = Aabb(Vector3(33.2979f, 3.20953f, 19.2614f), Vector3(38.403f, 7.63045f, 21.0182f)); 
  data[20].mAabb = Aabb(Vector3(21.9393f, 34.0593f, -5.55771f), Vector3(29.015f, 37.8983f, -2.05063f)); 
  data[21].mAabb = Aabb(Vector3(39.1339f, -15.182f, -15.0239f), Vector3(42.3765f, -8.27857f, -9.27787f)); 
  data[22].mAabb = Aabb(Vector3(18.969f, -32.1087f, 24.5856f), Vector3(20.7966f, -27.868f, 31.9325f)); 
  data[23].mAabb = Aabb(Vector3(17.2208f, 22.4034f, -18.8974f), Vector3(20.451f, 26.9452f, -15.5086f)); 
  data[24].mAabb = Aabb(Vector3(-48.848f, -15.2283f, 1.90122f), Vector3(-47.7513f, -8.2423f, 6.10667f)); 
  data[25].mAabb = Aabb(Vector3(41.8932f, 12.8519f, 7.16229f), Vector3(49.0228f, 16.2295f, 11.641f)); 
  data[26].mAabb = Aabb(Vector3(26.9627f, -22.3792f, 35.5422f), Vector3(28.3362f, -15.8979f, 37.7771f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-7.90521f, 20.9006f, 6.71414f));
  ray.mDirection = Vector3(Vector3(0.429839f, -0.811145f, 0.396588f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(-6.98379f, 20.724f, 31.9836f), Vector3(-5.89121f, 27.6803f, 33.9701f)); 
  data[1].mAabb = Aabb(Vector3(-28.3478f, 12.2778f, 27.1538f), Vector3(-24.8354f, 18.6102f, 34.997f)); 
  data[2].mAabb = Aabb(Vector3(12.2524f, -15.3709f, -31.0962f), Vector3(20.1376f, -13.2432f, -24.8283f)); 
  data[3].mAabb = Aabb(Vector3(10.963f, 29.3843f, -25.6844f), Vector3(18.3047f, 35.0907f, -21.9616f)); 
  data[4].mAabb = Aabb(Vector3(2.69764f, -49.2306f, -6.84813f), Vector3(8.92561f, -44.6719f, 0.326859f)); 
  data[5].mAabb = Aabb(Vector3(1.93117f, 37.6863f, 22.3348f), Vector3(8.76891f, 39.4037f, 28.1469f)); 
  data[6].mAabb = Aabb(Vector3(14.8864f, -20.7772f, 20.4407f), Vector3(20.5069f, -16.5926f, 26.6445f)); 
  data[7].mAabb = Aabb(Vector3(22.1707f, 12.7288f, 23.283f), Vector3(28.4163f, 17.4547f, 24.4348f)); 
  data[8].mAabb = Aabb(Vector3(23.322f, 16.8658f, -16.515f), Vector3(24.7894f, 17.3725f, -10.6134f)); 
  data[9].mAabb = Aabb(Vector3(-42.9297f, -3.74392f, -15.912f), Vector3(-37.9869f, -2.94313f, -9.18583f)); 
  data[10].mAabb = Aabb(Vector3(-31.3302f, -4.52966f, 7.82697f), Vector3(-26.7483f, -0.191536f, 15.3028f)); 
  data[11].mAabb = Aabb(Vector3(-4.8975f, -3.76745f, -21.0221f), Vector3(0.289992f, -0.238106f, -20.5339f)); 
  data[12].mAabb = Aabb(Vector3(15.0869f, -4.46657f, 6.71818f), Vector3(21.3049f, 0.483643f, 12.5238f)); 
  data[13].mAabb = Aabb(Vector3(2.24074f, 9.19811f, 12.599f), Vector3(9.29674f, 16.6517f, 15.697f)); 
  data[14].mAabb = Aabb(Vector3(-36.0911f, 11.5339f, 3.54523f), Vector3(-30.3875f, 19.365f, 4.16418f)); 
  data[15].mAabb = Aabb(Vector3(-34.6269f, -34.6817f, -9.428f), Vector3(-29.0941f, -31.2301f, -5.20005f)); 
  data[16].mAabb = Aabb(Vector3(-4.36666f, 30.2325f, -31.9238f), Vector3(-0.839172f, 31.4983f, -31.1274f)); 
  data[17].mAabb = Aabb(Vector3(-13.9877f, 37.2497f, 1.53283f), Vector3(-13.074f, 44.6251f, 2.63793f)); 
  data[18].mAabb = Aabb(Vector3(33.6588f, 16.182f, 25.2989f), Vector3(37.011f, 18.74f, 27.8622f)); 
  data[19].mAabb = Aabb(Vector3(-30.6534f, -21.0186f, -26.7952f), Vector3(-23.5635f, -17.1668f, -24.048f)); 
  data[20].mAabb = Aabb(Vector3(7.04001f, 29.8874f, 20.6983f), Vector3(8.84534f, 33.983f, 22.8357f)); 
  data[21].mAabb = Aabb(Vector3(-14.054f, -23.7073f, -5.08123f), Vector3(-13.587f, -16.3363f, -3.52176f)); 
  data[22].mAabb = Aabb(Vector3(-22.3082f, 42.3959f, 9.49609f), Vector3(-20.3857f, 42.8769f, 10.2846f)); 
  data[23].mAabb = Aabb(Vector3(15.7897f, -3.94623f, 17.0669f), Vector3(22.6818f, 1.81577f, 18.4162f)); 
  data[24].mAabb = Aabb(Vector3(-17.8699f, 5.15689f, 11.2437f), Vector3(-11.5768f, 5.69281f, 13.3208f)); 
  data[25].mAabb = Aabb(Vector3(40.7902f, -14.13f, -21.4045f), Vector3(44.1298f, -10.7533f, -16.7758f)); 
  data[26].mAabb = Aabb(Vector3(-4.53582f, -32.2045f, 13.1605f), Vector3(-1.55363f, -24.413f, 17.9936f)); 
  data[27].mAabb = Aabb(Vector3(-10.888f, 32.5706f, -24.1841f), Vector3(-4.35759f, 37.662f, -21.5254f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-6.66787f, -2.18159f, -6.08864f));
  ray.mDirection = Vector3(Vector3(0.784557f, -0.0038363f, 0.620045f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mAabb = Aabb(Vector3(-21.1906f, -11.7775f, 29.3357f), Vector3(-19.1746f, -5.73738f, 30.65f)); 
  data[1].mAabb = Aabb(Vector3(-37.5277f, -36.6264f, 2.91149f), Vector3(-32.4428f, -30.7141f, 9.6669f)); 
  data[2].mAabb = Aabb(Vector3(11.9877f, 2.80733f, 5.08631f), Vector3(19.3197f, 7.59217f, 10.072f)); 
  data[3].mAabb = Aabb(Vector3(10.398f, 3.99616f, 21.801f), Vector3(12.9486f, 7.45731f, 26.5726f)); 
  data[4].mAabb = Aabb(Vector3(-15.6338f, -7.96993f, 16.6037f), Vector3(-8.05242f, -2.35637f, 23.4422f)); 
  data[5].mAabb = Aabb(Vector3(19.9096f, -14.4321f, 4.96082f), Vector3(24.9741f, -8.28549f, 8.68268f)); 
  data[6].mAabb = Aabb(Vector3(-4.36186f, 32.7126f, -34.9212f), Vector3(-2.19748f, 37.9437f, -29.4786f)); 
  data[7].mAabb = Aabb(Vector3(20.5391f, -31.8681f, 7.03704f), Vector3(21.0034f, -24.9531f, 14.2248f)); 
  data[8].mAabb = Aabb(Vector3(4.25228f, -38.6081f, 3.07615f), Vector3(8.70197f, -32.8498f, 3.83218f)); 
  data[9].mAabb = Aabb(Vector3(-31.6423f, -39.0491f, -12.17f), Vector3(-27.9494f, -36.804f, -7.75695f)); 
  data[10].mAabb = Aabb(Vector3(-44.2706f, -12.1708f, -18.7553f), Vector3(-41.2798f, -7.43749f, -10.7873f)); 
  data[11].mAabb = Aabb(Vector3(8.36135f, 17.8774f, 21.3787f), Vector3(12.3699f, 19.2421f, 23.8841f)); 
  data[12].mAabb = Aabb(Vector3(-15.9756f, -2.65475f, 12.5523f), Vector3(-11.3408f, 2.78045f, 15.3325f)); 
  data[13].mAabb = Aabb(Vector3(10.2306f, -16.2817f, 0.412244f), Vector3(15.6015f, -9.53674f, 0.813404f)); 
  data[14].mAabb = Aabb(Vector3(4.40359f, -42.052f, -13.426f), Vector3(8.84098f, -38.4628f, -7.48661f)); 
  data[15].mAabb = Aabb(Vector3(-27.2055f, -25.6313f, -29.7627f), Vector3(-20.3923f, -18.1973f, -21.8492f)); 
  data[16].mAabb = Aabb(Vector3(20.1038f, 10.69f, -3.65628f), Vector3(24.7988f, 12.2105f, -1.82915f)); 
  data[17].mAabb = Aabb(Vector3(46.3218f, -4.42504f, -10.3021f), Vector3(47.7337f, 3.36992f, -6.13049f)); 
  data[18].mAabb = Aabb(Vector3(-39.9915f, 2.22978f, -8.71978f), Vector3(-35.9837f, 8.04467f, -3.39174f)); 
  data[19].mAabb = Aabb(Vector3(38.2336f, -22.0226f, -16.5687f), Vector3(46.1056f, -18.0168f, -9.40023f)); 
  data[20].mAabb = Aabb(Vector3(-38.7037f, -2.3015f, 4.25639f), Vector3(-36.0184f, 4.04987f, 6.43769f)); 
  data[21].mAabb = Aabb(Vector3(-6.46542f, -15.0028f, 33.1924f), Vector3(-5.93484f, -8.11289f, 37.5148f)); 
  data[22].mAabb = Aabb(Vector3(26.1535f, -33.8045f, 7.57891f), Vector3(32.3622f, -27.0375f, 8.61397f)); 
  data[23].mAabb = Aabb(Vector3(39.922f, 7.70771f, -22.9144f), Vector3(42.5271f, 8.33988f, -15.9412f)); 
  data[24].mAabb = Aabb(Vector3(13.1412f, 10.199f, 26.1807f), Vector3(17.7998f, 15.6196f, 32.1898f)); 
  data[25].mAabb = Aabb(Vector3(20.4188f, -8.88007f, 16.6504f), Vector3(23.7011f, -4.52711f, 18.1676f)); 
  data[26].mAabb = Aabb(Vector3(9.10767f, 24.8902f, 0.844923f), Vector3(10.1341f, 29.4016f, 4.42785f)); 
  data[27].mAabb = Aabb(Vector3(2.38536f, 26.1507f, 2.4001f), Vector3(3.86064f, 30.6417f, 9.30256f)); 
  data[28].mAabb = Aabb(Vector3(-20.7292f, 0.591145f, 41.0975f), Vector3(-15.2121f, 7.81368f, 41.7199f)); 
  data[29].mAabb = Aabb(Vector3(-20.6371f, -5.97841f, -17.0169f), Vector3(-16.5852f, -5.00992f, -13.4303f)); 
  data[30].mAabb = Aabb(Vector3(7.94894f, 6.91311f, 4.22986f), Vector3(15.9044f, 11.4577f, 6.38264f)); 
  data[31].mAabb = Aabb(Vector3(-18.6061f, -5.19141f, 20.2521f), Vector3(-10.8559f, -1.69825f, 24.7324f)); 
  data[32].mAabb = Aabb(Vector3(-3.41608f, 10.5903f, -47.9675f), Vector3(1.59838f, 13.4385f, -44.273f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-3.88417f, 8.6913f, -2.66489f));
  ray.mDirection = Vector3(Vector3(-0.378673f, -0.400973f, 0.834162f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(32); 
  data[0].mAabb = Aabb(Vector3(1.98398f, 14.1359f, -35.7516f), Vector3(3.21618f, 17.6284f, -30.5729f)); 
  data[1].mAabb = Aabb(Vector3(41.9898f, 2.01204f, 3.09096f), Vector3(46.0407f, 4.69387f, 5.77743f)); 
  data[2].mAabb = Aabb(Vector3(-18.4606f, 13.2063f, 31.901f), Vector3(-11.9258f, 14.0398f, 37.1198f)); 
  data[3].mAabb = Aabb(Vector3(-22.982f, 13.786f, 18.542f), Vector3(-15.9314f, 17.384f, 21.1392f)); 
  data[4].mAabb = Aabb(Vector3(-25.7838f, -33.8458f, -26.4808f), Vector3(-19.4686f, -27.5912f, -19.7632f)); 
  data[5].mAabb = Aabb(Vector3(-8.07089f, -10.1495f, -0.934171f), Vector3(-2.68092f, -3.92102f, 1.16711f)); 
  data[6].mAabb = Aabb(Vector3(-19.4885f, 39.4489f, -11.7141f), Vector3(-17.9759f, 44.6853f, -10.7816f)); 
  data[7].mAabb = Aabb(Vector3(3.42872f, 20.8702f, 29.9549f), Vector3(4.52199f, 22.844f, 37.9329f)); 
  data[8].mAabb = Aabb(Vector3(22.4352f, -3.26358f, 20.0118f), Vector3(25.6366f, 3.80286f, 23.4887f)); 
  data[9].mAabb = Aabb(Vector3(-3.9593f, -3.38105f, 9.98255f), Vector3(3.50051f, 4.46285f, 17.4593f)); 
  data[10].mAabb = Aabb(Vector3(32.0616f, 6.49561f, 9.79396f), Vector3(35.6759f, 8.10982f, 14.229f)); 
  data[11].mAabb = Aabb(Vector3(-6.27371f, -19.8516f, -42.366f), Vector3(-3.96136f, -19.2916f, -40.7084f)); 
  data[12].mAabb = Aabb(Vector3(-23.5699f, -26.8346f, 14.3613f), Vector3(-18.439f, -26.2175f, 21.3259f)); 
  data[13].mAabb = Aabb(Vector3(-7.69039f, 36.4842f, -33.4425f), Vector3(-4.47208f, 38.0323f, -29.3752f)); 
  data[14].mAabb = Aabb(Vector3(-6.36826f, 25.4439f, 34.3522f), Vector3(-5.7725f, 30.0234f, 42.1934f)); 
  data[15].mAabb = Aabb(Vector3(-8.07481f, -14.623f, -49.4602f), Vector3(-0.133029f, -10.7972f, -44.94f)); 
  data[16].mAabb = Aabb(Vector3(19.5579f, -13.4674f, 26.3326f), Vector3(26.8814f, -7.26242f, 31.5466f)); 
  data[17].mAabb = Aabb(Vector3(-33.3976f, -16.2554f, 6.99754f), Vector3(-30.2046f, -10.6493f, 14.9414f)); 
  data[18].mAabb = Aabb(Vector3(-5.66591f, -18.5981f, 44.892f), Vector3(-3.11025f, -17.9268f, 45.9245f)); 
  data[19].mAabb = Aabb(Vector3(-26.4311f, 33.6167f, -27.1553f), Vector3(-21.0936f, 34.8387f, -23.5047f)); 
  data[20].mAabb = Aabb(Vector3(-25.0142f, -48.0481f, -10.7069f), Vector3(-17.8765f, -40.7864f, -3.30185f)); 
  data[21].mAabb = Aabb(Vector3(-40.4443f, 7.7878f, -19.7888f), Vector3(-38.7199f, 11.9779f, -14.3874f)); 
  data[22].mAabb = Aabb(Vector3(17.0019f, 16.3525f, 11.1642f), Vector3(22.567f, 20.6123f, 12.7183f)); 
  data[23].mAabb = Aabb(Vector3(23.3923f, -22.2995f, -35.6742f), Vector3(30.4914f, -16.0778f, -33.2556f)); 
  data[24].mAabb = Aabb(Vector3(4.56635f, 20.5134f, -33.2904f), Vector3(6.94411f, 22.5447f, -27.4576f)); 
  data[25].mAabb = Aabb(Vector3(-15.1415f, -14.135f, -10.919f), Vector3(-10.1788f, -6.33885f, -3.60138f)); 
  data[26].mAabb = Aabb(Vector3(-24.7569f, 30.5323f, -14.782f), Vector3(-22.9061f, 34.7261f, -12.9242f)); 
  data[27].mAabb = Aabb(Vector3(27.9772f, 34.2559f, 15.6705f), Vector3(31.5156f, 35.5648f, 18.7818f)); 
  data[28].mAabb = Aabb(Vector3(36.8369f, 16.9931f, 0.535999f), Vector3(42.59f, 22.7883f, 7.76596f)); 
  data[29].mAabb = Aabb(Vector3(-4.33443f, -11.888f, -14.5384f), Vector3(2.2774f, -7.48196f, -8.8787f)); 
  data[30].mAabb = Aabb(Vector3(4.08176f, -23.0424f, 40.1589f), Vector3(10.2125f, -16.6361f, 41.2568f)); 
  data[31].mAabb = Aabb(Vector3(-7.20541f, 2.09075f, 42.8572f), Vector3(0.0957477f, 3.80841f, 48.5735f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-14.3298f, -1.30328f, 16.7148f));
  ray.mDirection = Vector3(Vector3(0.896447f, 0.396867f, -0.19718f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(39); 
  data[0].mAabb = Aabb(Vector3(6.7055f, -43.9475f, 11.0375f), Vector3(8.30463f, -39.1233f, 13.5787f)); 
  data[1].mAabb = Aabb(Vector3(-20.6192f, 22.1733f, -8.92559f), Vector3(-20.1635f, 27.9137f, -4.03197f)); 
  data[2].mAabb = Aabb(Vector3(-13.4328f, -31.7405f, 5.69958f), Vector3(-10.9321f, -29.9806f, 6.80282f)); 
  data[3].mAabb = Aabb(Vector3(-23.1033f, 10.0839f, 34.074f), Vector3(-21.0712f, 13.1164f, 37.9837f)); 
  data[4].mAabb = Aabb(Vector3(11.2555f, 10.5929f, -37.6294f), Vector3(13.1594f, 11.7205f, -34.0875f)); 
  data[5].mAabb = Aabb(Vector3(-10.7512f, 16.7887f, -18.6672f), Vector3(-5.31738f, 22.1771f, -13.8601f)); 
  data[6].mAabb = Aabb(Vector3(27.4838f, 15.4655f, -37.2201f), Vector3(28.3222f, 21.4617f, -36.0373f)); 
  data[7].mAabb = Aabb(Vector3(10.5513f, 28.1387f, 14.6763f), Vector3(14.9184f, 33.4739f, 21.1339f)); 
  data[8].mAabb = Aabb(Vector3(-2.96362f, -5.10674f, -2.41443f), Vector3(-2.35232f, 2.3911f, -1.14488f)); 
  data[9].mAabb = Aabb(Vector3(-25.476f, -33.5592f, 23.2586f), Vector3(-20.1498f, -31.0968f, 25.5403f)); 
  data[10].mAabb = Aabb(Vector3(12.2826f, 39.9105f, 7.66042f), Vector3(20.0875f, 42.9956f, 9.30339f)); 
  data[11].mAabb = Aabb(Vector3(1.11449f, -7.31298f, -42.2669f), Vector3(3.13297f, -0.323081f, -35.6727f)); 
  data[12].mAabb = Aabb(Vector3(0.233882f, -24.0078f, -27.2024f), Vector3(6.09516f, -22.9792f, -20.5018f)); 
  data[13].mAabb = Aabb(Vector3(10.5922f, -43.6697f, -14.9677f), Vector3(13.3028f, -37.4877f, -12.5615f)); 
  data[14].mAabb = Aabb(Vector3(35.4461f, -25.222f, 4.53509f), Vector3(37.7121f, -21.9067f, 6.28521f)); 
  data[15].mAabb = Aabb(Vector3(7.92941f, -23.7152f, -19.0767f), Vector3(8.93222f, -22.0414f, -15.7705f)); 
  data[16].mAabb = Aabb(Vector3(-2.43522f, 36.7371f, -26.0221f), Vector3(3.30893f, 37.649f, -21.755f)); 
  data[17].mAabb = Aabb(Vector3(-2.58012f, -0.424693f, -41.2305f), Vector3(2.16831f, 1.13918f, -33.4059f)); 
  data[18].mAabb = Aabb(Vector3(-3.59923f, -14.5119f, 34.0029f), Vector3(0.878987f, -10.5539f, 36.7747f)); 
  data[19].mAabb = Aabb(Vector3(-26.729f, 37.7109f, -2.98374f), Vector3(-20.7016f, 44.6539f, -2.08507f)); 
  data[20].mAabb = Aabb(Vector3(-51.2534f, -10.3986f, 2.18823f), Vector3(-46.2874f, -6.35346f, 2.82295f)); 
  data[21].mAabb = Aabb(Vector3(-4.57237f, -36.827f, -10.3247f), Vector3(2.31478f, -30.9082f, -9.07439f)); 
  data[22].mAabb = Aabb(Vector3(9.63061f, -25.0727f, -7.88338f), Vector3(14.556f, -19.9211f, -1.95508f)); 
  data[23].mAabb = Aabb(Vector3(-12.7816f, -21.9598f, -28.8231f), Vector3(-6.21499f, -14.7604f, -23.538f)); 
  data[24].mAabb = Aabb(Vector3(-13.4648f, 36.287f, -29.2925f), Vector3(-8.00364f, 41.4761f, -27.0632f)); 
  data[25].mAabb = Aabb(Vector3(-0.922707f, 13.1136f, -40.7517f), Vector3(3.18742f, 17.8096f, -39.6209f)); 
  data[26].mAabb = Aabb(Vector3(-4.48076f, 7.34768f, -42.0298f), Vector3(-0.172793f, 11.279f, -34.5354f)); 
  data[27].mAabb = Aabb(Vector3(-40.0805f, -10.7441f, 30.3164f), Vector3(-34.3911f, -6.7312f, 32.5411f)); 
  data[28].mAabb = Aabb(Vector3(-5.92032f, -9.28784f, 6.03876f), Vector3(1.03664f, -6.41187f, 11.1506f)); 
  data[29].mAabb = Aabb(Vector3(-8.76151f, -25.4924f, 4.42024f), Vector3(-1.82102f, -23.2134f, 4.88101f)); 
  data[30].mAabb = Aabb(Vector3(-24.527f, -2.90761f, -44.3613f), Vector3(-19.4434f, 1.50752f, -38.1113f)); 
  data[31].mAabb = Aabb(Vector3(-39.8712f, -32.8008f, 1.99135f), Vector3(-39.4362f, -26.2087f, 3.1048f)); 
  data[32].mAabb = Aabb(Vector3(14.6553f, -41.5159f, -23.6248f), Vector3(19.3427f, -35.2492f, -18.8177f)); 
  data[33].mAabb = Aabb(Vector3(21.0959f, -28.1604f, -22.3822f), Vector3(22.4869f, -22.8296f, -20.8357f)); 
  data[34].mAabb = Aabb(Vector3(-13.9802f, -19.4011f, -31.825f), Vector3(-6.22556f, -13.2666f, -31.3422f)); 
  data[35].mAabb = Aabb(Vector3(7.69149f, 19.7731f, 15.1092f), Vector3(12.6322f, 20.1764f, 22.6314f)); 
  data[36].mAabb = Aabb(Vector3(0.987258f, 4.81774f, 27.4399f), Vector3(8.04558f, 9.77677f, 30.7187f)); 
  data[37].mAabb = Aabb(Vector3(-8.54807f, 3.71161f, -40.7621f), Vector3(-3.68159f, 10.801f, -38.4419f)); 
  data[38].mAabb = Aabb(Vector3(-31.0757f, 34.729f, 13.2252f), Vector3(-26.7679f, 35.6474f, 18.1655f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(4.5444f, 2.17988f, -22.104f));
  ray.mDirection = Vector3(Vector3(-0.645662f, 0.667604f, 0.370708f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(5.18108f, -29.22f, 13.0937f), Vector3(11.8384f, -23.0275f, 14.3089f)); 
  data[1].mAabb = Aabb(Vector3(-13.7313f, 6.45392f, -0.558265f), Vector3(-8.11746f, 12.8366f, 5.9318f)); 
  data[2].mAabb = Aabb(Vector3(3.19923f, -28.5206f, -39.8218f), Vector3(10.4085f, -27.6619f, -33.3763f)); 
  data[3].mAabb = Aabb(Vector3(-30.989f, 19.5927f, 25.2567f), Vector3(-28.7333f, 23.8209f, 32.3076f)); 
  data[4].mAabb = Aabb(Vector3(-46.0708f, 5.31772f, -1.72394f), Vector3(-42.678f, 9.99448f, 1.24295f)); 
  data[5].mAabb = Aabb(Vector3(-1.74667f, -50.314f, 6.62266f), Vector3(0.344408f, -46.2183f, 8.75109f)); 
  data[6].mAabb = Aabb(Vector3(28.6783f, 26.956f, -4.19385f), Vector3(35.6244f, 28.6973f, 2.9765f)); 
  data[7].mAabb = Aabb(Vector3(38.5613f, 9.4449f, 8.47566f), Vector3(45.9088f, 16.6846f, 14.4232f)); 
  data[8].mAabb = Aabb(Vector3(-15.0456f, 1.76942f, -30.5325f), Vector3(-9.21359f, 2.512f, -27.487f)); 
  data[9].mAabb = Aabb(Vector3(21.3905f, -10.2777f, -10.856f), Vector3(26.1885f, -3.80272f, -8.79827f)); 
  data[10].mAabb = Aabb(Vector3(28.0382f, 4.50112f, -30.9394f), Vector3(32.0322f, 6.65367f, -29.2672f)); 
  data[11].mAabb = Aabb(Vector3(17.3408f, -12.3933f, 20.3123f), Vector3(25.1196f, -5.0188f, 28.3093f)); 
  data[12].mAabb = Aabb(Vector3(-31.2556f, 24.4762f, -26.6131f), Vector3(-27.6653f, 25.6463f, -25.5804f)); 
  data[13].mAabb = Aabb(Vector3(18.8218f, 43.0407f, -2.82254f), Vector3(23.2907f, 46.92f, 0.284437f)); 
  data[14].mAabb = Aabb(Vector3(18.7284f, -37.3422f, -19.4723f), Vector3(25.1503f, -35.8544f, -14.9106f)); 
  data[15].mAabb = Aabb(Vector3(5.52305f, -30.6917f, -32.3001f), Vector3(10.5677f, -23.805f, -30.8793f)); 
  data[16].mAabb = Aabb(Vector3(-14.7896f, -5.05088f, -11.6251f), Vector3(-13.7244f, -1.50762f, -9.37952f)); 
  data[17].mAabb = Aabb(Vector3(11.3011f, 6.47047f, 13.6843f), Vector3(15.4203f, 14.2483f, 14.2126f)); 
  data[18].mAabb = Aabb(Vector3(-13.2807f, 26.1302f, 4.27403f), Vector3(-8.45622f, 29.2442f, 8.55764f)); 
  data[19].mAabb = Aabb(Vector3(16.7877f, 35.2287f, -22.1348f), Vector3(22.2716f, 41.0772f, -17.4001f)); 
  data[20].mAabb = Aabb(Vector3(-31.3791f, 1.39173f, -13.1516f), Vector3(-25.9669f, 3.31419f, -12.4282f)); 
  data[21].mAabb = Aabb(Vector3(-9.06262f, 10.1954f, 22.5102f), Vector3(-7.5043f, 14.6003f, 27.9355f)); 
  data[22].mAabb = Aabb(Vector3(9.05908f, -39.0343f, 10.5157f), Vector3(9.7337f, -33.7202f, 15.9982f)); 
  data[23].mAabb = Aabb(Vector3(31.3784f, -12.2973f, 9.44351f), Vector3(32.7465f, -6.61281f, 12.3916f)); 
  data[24].mAabb = Aabb(Vector3(-10.0883f, -39.2736f, -14.7743f), Vector3(-6.01696f, -38.2098f, -9.80532f)); 
  data[25].mAabb = Aabb(Vector3(34.5178f, 1.50732f, 29.0064f), Vector3(41.5729f, 2.07733f, 35.1338f)); 
  data[26].mAabb = Aabb(Vector3(-36.9385f, 26.6598f, -11.3504f), Vector3(-35.5145f, 27.4442f, -3.63456f)); 
  data[27].mAabb = Aabb(Vector3(20.1353f, -8.4398f, 11.6276f), Vector3(22.4762f, -2.38625f, 14.1452f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(16.4026f, -11.5172f, 1.10941f));
  ray.mDirection = Vector3(Vector3(0.349957f, 0.222699f, 0.909909f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(-21.7246f, -17.6249f, -42.9201f), Vector3(-15.9933f, -10.5183f, -37.4814f)); 
  data[1].mAabb = Aabb(Vector3(1.57502f, -8.06779f, 17.9136f), Vector3(9.30087f, -1.96808f, 20.0459f)); 
  data[2].mAabb = Aabb(Vector3(-16.3835f, -36.6474f, 7.97094f), Vector3(-9.77167f, -34.0987f, 11.8577f)); 
  data[3].mAabb = Aabb(Vector3(-7.40137f, -51.6918f, 1.68927f), Vector3(0.341177f, -46.3888f, 9.4462f)); 
  data[4].mAabb = Aabb(Vector3(37.6088f, 9.50088f, -4.23247f), Vector3(42.8376f, 13.1364f, 0.993516f)); 
  data[5].mAabb = Aabb(Vector3(-6.2246f, -34.4039f, 16.8352f), Vector3(-2.2207f, -31.4911f, 19.6613f)); 
  data[6].mAabb = Aabb(Vector3(-8.59325f, -2.66066f, 26.7162f), Vector3(-6.24889f, 0.711425f, 29.4834f)); 
  data[7].mAabb = Aabb(Vector3(-40.3536f, -12.7741f, 0.946546f), Vector3(-36.3309f, -11.3747f, 8.45298f)); 
  data[8].mAabb = Aabb(Vector3(-33.713f, -19.6492f, 31.9424f), Vector3(-31.6806f, -16.4752f, 32.6933f)); 
  data[9].mAabb = Aabb(Vector3(-22.9781f, -39.9212f, 9.61142f), Vector3(-15.1421f, -38.1894f, 13.8612f)); 
  data[10].mAabb = Aabb(Vector3(-10.6242f, -7.25869f, -22.5212f), Vector3(-7.35091f, -4.32334f, -17.3297f)); 
  data[11].mAabb = Aabb(Vector3(20.3665f, 6.07841f, -44.7566f), Vector3(27.7502f, 6.56957f, -38.4479f)); 
  data[12].mAabb = Aabb(Vector3(-17.5401f, -6.71136f, 39.4999f), Vector3(-12.1441f, -3.96157f, 42.063f)); 
  data[13].mAabb = Aabb(Vector3(-0.111959f, -6.84626f, 23.894f), Vector3(4.2577f, 0.693804f, 27.6678f)); 
  data[14].mAabb = Aabb(Vector3(15.5866f, -9.43172f, 25.9036f), Vector3(18.9705f, -4.78582f, 32.7486f)); 
  data[15].mAabb = Aabb(Vector3(-11.0013f, -8.4002f, 34.3239f), Vector3(-9.28783f, -2.28541f, 40.6734f)); 
  data[16].mAabb = Aabb(Vector3(15.2223f, -35.9992f, 26.4737f), Vector3(21.2754f, -29.4978f, 27.3026f)); 
  data[17].mAabb = Aabb(Vector3(-26.0031f, -29.1306f, -17.2396f), Vector3(-22.4937f, -27.73f, -14.1616f)); 
  data[18].mAabb = Aabb(Vector3(-2.06708f, -0.780764f, 12.5418f), Vector3(5.45071f, 6.11427f, 16.7544f)); 
  data[19].mAabb = Aabb(Vector3(-44.56f, -17.8771f, -16.2417f), Vector3(-43.158f, -14.4238f, -11.2166f)); 
  data[20].mAabb = Aabb(Vector3(18.1667f, 32.0713f, 24.9569f), Vector3(23.9224f, 37.5332f, 28.1395f)); 
  data[21].mAabb = Aabb(Vector3(-5.12008f, 9.12383f, 34.2484f), Vector3(-0.0272295f, 16.0518f, 40.5891f)); 
  data[22].mAabb = Aabb(Vector3(15.6022f, -25.1539f, 35.2972f), Vector3(21.887f, -22.8395f, 39.7017f)); 
  data[23].mAabb = Aabb(Vector3(-25.1919f, -2.30048f, -29.8194f), Vector3(-21.5011f, 1.18108f, -22.9137f)); 
  data[24].mAabb = Aabb(Vector3(-11.6399f, -26.2336f, -29.2665f), Vector3(-5.67613f, -22.3434f, -27.9374f)); 
  data[25].mAabb = Aabb(Vector3(-36.0273f, 3.11643f, 19.4016f), Vector3(-29.2102f, 8.82324f, 27.3791f)); 
  data[26].mAabb = Aabb(Vector3(5.14863f, 11.9998f, 23.6273f), Vector3(6.47778f, 12.7869f, 25.3596f)); 
  data[27].mAabb = Aabb(Vector3(-12.0003f, 33.5726f, -18.85f), Vector3(-5.4235f, 38.035f, -16.401f)); 
  data[28].mAabb = Aabb(Vector3(-43.4289f, -23.6663f, 9.67385f), Vector3(-42.1452f, -22.7106f, 12.868f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(2.17514f, 0.482986f, 6.99895f));
  ray.mDirection = Vector3(Vector3(-0.458433f, -0.136194f, 0.878231f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(-10.3773f, -18.6776f, 44.4074f), Vector3(-5.66479f, -11.6302f, 48.2246f)); 
  data[1].mAabb = Aabb(Vector3(-36.6845f, 11.6208f, -30.6909f), Vector3(-29.5362f, 16.9924f, -28.8357f)); 
  data[2].mAabb = Aabb(Vector3(-14.3358f, -38.027f, -0.775828f), Vector3(-13.6348f, -32.7085f, 1.70213f)); 
  data[3].mAabb = Aabb(Vector3(28.4563f, -29.4923f, -24.0691f), Vector3(34.1873f, -25.7417f, -16.0735f)); 
  data[4].mAabb = Aabb(Vector3(-31.1086f, -31.9404f, 23.4436f), Vector3(-29.7915f, -29.3609f, 24.3882f)); 
  data[5].mAabb = Aabb(Vector3(17.757f, 7.2246f, -17.6803f), Vector3(22.2563f, 14.555f, -10.535f)); 
  data[6].mAabb = Aabb(Vector3(3.22726f, -23.7275f, -40.3797f), Vector3(10.3382f, -16.1188f, -34.1911f)); 
  data[7].mAabb = Aabb(Vector3(-10.0403f, 35.3578f, 12.6602f), Vector3(-5.27144f, 36.8472f, 15.6575f)); 
  data[8].mAabb = Aabb(Vector3(1.06376f, 27.1852f, -22.9945f), Vector3(2.45229f, 28.6018f, -18.003f)); 
  data[9].mAabb = Aabb(Vector3(-29.2563f, 0.697529f, -19.492f), Vector3(-22.5165f, 5.86971f, -14.009f)); 
  data[10].mAabb = Aabb(Vector3(23.6402f, 17.4366f, -0.451262f), Vector3(31.2173f, 21.035f, 6.69241f)); 
  data[11].mAabb = Aabb(Vector3(46.7831f, 4.8622f, -13.6726f), Vector3(47.9269f, 12.4621f, -6.78751f)); 
  data[12].mAabb = Aabb(Vector3(-27.1927f, -41.1824f, 5.62946f), Vector3(-22.3155f, -39.7595f, 12.0525f)); 
  data[13].mAabb = Aabb(Vector3(-16.1064f, 6.37327f, -22.9029f), Vector3(-11.7977f, 9.10428f, -16.1923f)); 
  data[14].mAabb = Aabb(Vector3(14.5597f, 9.28332f, -20.5934f), Vector3(15.7763f, 16.7088f, -16.1281f)); 
  data[15].mAabb = Aabb(Vector3(-20.2794f, -12.1563f, 10.9578f), Vector3(-15.9297f, -10.6281f, 15.2971f)); 
  data[16].mAabb = Aabb(Vector3(-0.809508f, -25.5634f, -4.23795f), Vector3(3.10486f, -19.6972f, -0.995521f)); 
  data[17].mAabb = Aabb(Vector3(0.681103f, -11.286f, -41.3656f), Vector3(2.29114f, -6.5898f, -37.5085f)); 
  data[18].mAabb = Aabb(Vector3(23.7543f, -1.53653f, -4.5244f), Vector3(28.422f, -0.921516f, 0.332808f)); 
  data[19].mAabb = Aabb(Vector3(-28.9341f, 25.8442f, -15.8188f), Vector3(-25.4824f, 31.799f, -12.5278f)); 
  data[20].mAabb = Aabb(Vector3(-12.6582f, -22.1621f, -43.1083f), Vector3(-12.1798f, -21.3358f, -35.1403f)); 
  data[21].mAabb = Aabb(Vector3(-3.4243f, -18.1763f, -2.66882f), Vector3(1.09126f, -17.2704f, 2.38368f)); 
  data[22].mAabb = Aabb(Vector3(20.6938f, 17.2891f, -36.7395f), Vector3(28.2209f, 22.2551f, -35.1819f)); 
  data[23].mAabb = Aabb(Vector3(-18.6398f, -0.72177f, 42.5187f), Vector3(-16.076f, 2.44064f, 47.1285f)); 
  data[24].mAabb = Aabb(Vector3(-33.7816f, 0.588655f, 11.1944f), Vector3(-27.774f, 2.54196f, 17.3525f)); 
  data[25].mAabb = Aabb(Vector3(21.551f, 30.272f, -4.71246f), Vector3(26.565f, 35.0659f, -2.92105f)); 
  data[26].mAabb = Aabb(Vector3(8.35177f, -26.575f, -7.94915f), Vector3(9.29126f, -22.1937f, -4.3087f)); 
  data[27].mAabb = Aabb(Vector3(-21.072f, -47.2702f, -0.837428f), Vector3(-13.1954f, -42.9112f, 6.10655f)); 
  data[28].mAabb = Aabb(Vector3(18.5412f, 14.9004f, -26.8364f), Vector3(26.3051f, 18.2748f, -23.5474f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(0.443841f, -12.5573f, -3.45909f));
  ray.mDirection = Vector3(Vector3(0.46151f, 0.74312f, -0.484542f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(11.7336f, -47.9805f, -6.75077f), Vector3(18.8917f, -40.8025f, -5.69021f)); 
  data[1].mAabb = Aabb(Vector3(-11.4594f, 43.9298f, -0.966048f), Vector3(-7.68332f, 51.7528f, 2.60713f)); 
  data[2].mAabb = Aabb(Vector3(-16.9879f, -41.3026f, 3.04783f), Vector3(-15.4456f, -39.5834f, 7.76587f)); 
  data[3].mAabb = Aabb(Vector3(-27.7919f, 28.1194f, 25.9253f), Vector3(-22.7208f, 32.48f, 32.2496f)); 
  data[4].mAabb = Aabb(Vector3(13.9132f, 3.11043f, -21.3755f), Vector3(16.2093f, 3.96109f, -16.8804f)); 
  data[5].mAabb = Aabb(Vector3(-43.4539f, -2.16411f, 9.67556f), Vector3(-38.3977f, -1.58969f, 12.7698f)); 
  data[6].mAabb = Aabb(Vector3(16.7005f, -14.0557f, -35.7811f), Vector3(19.2789f, -7.38398f, -28.2129f)); 
  data[7].mAabb = Aabb(Vector3(-14.3346f, -13.5729f, 24.7814f), Vector3(-11.2341f, -10.0983f, 31.6829f)); 
  data[8].mAabb = Aabb(Vector3(2.28046f, 10.6202f, -41.0706f), Vector3(6.59446f, 15.3992f, -37.8414f)); 
  data[9].mAabb = Aabb(Vector3(-27.6259f, 29.2131f, 27.7267f), Vector3(-25.7528f, 29.7069f, 32.2567f)); 
  data[10].mAabb = Aabb(Vector3(-29.8061f, -4.47065f, -21.4019f), Vector3(-26.5345f, -0.18541f, -19.9167f)); 
  data[11].mAabb = Aabb(Vector3(1.93929f, 15.3453f, -38.5737f), Vector3(8.20066f, 18.9245f, -32.8704f)); 
  data[12].mAabb = Aabb(Vector3(-17.8283f, -34.7369f, 2.47532f), Vector3(-16.7749f, -30.0362f, 4.73316f)); 
  data[13].mAabb = Aabb(Vector3(38.9607f, -28.9417f, -10.5565f), Vector3(41.4955f, -28.3845f, -4.91235f)); 
  data[14].mAabb = Aabb(Vector3(7.10582f, -22.0699f, -32.6044f), Vector3(8.11304f, -18.6264f, -30.3009f)); 
  data[15].mAabb = Aabb(Vector3(31.9879f, 19.5528f, 3.57982f), Vector3(37.8487f, 24.9826f, 4.01253f)); 
  data[16].mAabb = Aabb(Vector3(22.0914f, -11.0725f, 27.1723f), Vector3(24.169f, -9.13223f, 33.5376f)); 
  data[17].mAabb = Aabb(Vector3(1.92772f, -23.4799f, 26.7823f), Vector3(4.0188f, -15.6124f, 27.5592f)); 
  data[18].mAabb = Aabb(Vector3(13.8302f, 20.5349f, -41.4527f), Vector3(14.6349f, 23.0117f, -36.1882f)); 
  data[19].mAabb = Aabb(Vector3(-5.7508f, 25.7063f, 7.67263f), Vector3(-4.62181f, 31.6674f, 8.88002f)); 
  data[20].mAabb = Aabb(Vector3(-0.60252f, 27.7697f, -24.8545f), Vector3(7.2829f, 34.7366f, -23.4034f)); 
  data[21].mAabb = Aabb(Vector3(-35.1848f, -30.6424f, 2.36847f), Vector3(-30.1988f, -24.5469f, 4.67688f)); 
  data[22].mAabb = Aabb(Vector3(-4.93904f, -9.7077f, -11.647f), Vector3(0.968622f, -7.51271f, -4.92664f)); 
  data[23].mAabb = Aabb(Vector3(-8.09525f, -25.8642f, -29.5855f), Vector3(-6.64781f, -19.0585f, -24.5872f)); 
  data[24].mAabb = Aabb(Vector3(-38.9868f, 25.3634f, -17.5132f), Vector3(-32.313f, 28.8656f, -16.1048f)); 
  data[25].mAabb = Aabb(Vector3(-14.0123f, 0.870009f, -34.5826f), Vector3(-7.15558f, 3.34913f, -28.5108f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-12.5342f, -2.09581f, -21.4915f));
  ray.mDirection = Vector3(Vector3(0.671035f, 0.608101f, -0.424177f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mAabb = Aabb(Vector3(-17.556f, -33.1501f, 25.5128f), Vector3(-10.7865f, -25.9282f, 27.9251f)); 
  data[1].mAabb = Aabb(Vector3(-1.15356f, -10.7508f, 16.1339f), Vector3(3.62038f, -7.07188f, 16.8176f)); 
  data[2].mAabb = Aabb(Vector3(-31.025f, 13.4362f, -35.3637f), Vector3(-28.3476f, 17.0523f, -27.8223f)); 
  data[3].mAabb = Aabb(Vector3(14.7926f, -1.10748f, -30.7676f), Vector3(19.0126f, 5.99305f, -29.017f)); 
  data[4].mAabb = Aabb(Vector3(10.8501f, -30.4017f, -16.9756f), Vector3(11.4572f, -26.1862f, -15.1934f)); 
  data[5].mAabb = Aabb(Vector3(6.20007f, -1.79718f, 33.1925f), Vector3(12.7546f, 0.597976f, 36.2121f)); 
  data[6].mAabb = Aabb(Vector3(18.4832f, 12.2119f, -20.9186f), Vector3(22.674f, 15.8236f, -17.0256f)); 
  data[7].mAabb = Aabb(Vector3(30.141f, 3.00371f, -6.81797f), Vector3(34.1662f, 4.1218f, -5.68921f)); 
  data[8].mAabb = Aabb(Vector3(-8.29028f, -6.07816f, -1.34975f), Vector3(-5.38833f, -5.62133f, 5.15006f)); 
  data[9].mAabb = Aabb(Vector3(22.4864f, -0.293315f, -47.5479f), Vector3(23.7582f, 1.16689f, -40.5691f)); 
  data[10].mAabb = Aabb(Vector3(-9.91004f, 27.8118f, 28.8571f), Vector3(-9.21431f, 29.6049f, 32.8118f)); 
  data[11].mAabb = Aabb(Vector3(19.5596f, 1.92788f, -6.68519f), Vector3(23.0082f, 9.65512f, -1.33789f)); 
  data[12].mAabb = Aabb(Vector3(3.23973f, -14.9766f, 37.1411f), Vector3(4.79734f, -10.3207f, 41.9178f)); 
  data[13].mAabb = Aabb(Vector3(23.1458f, -28.9705f, -33.6836f), Vector3(24.6591f, -23.8479f, -30.6924f)); 
  data[14].mAabb = Aabb(Vector3(13.0504f, 30.7352f, -14.6904f), Vector3(16.0731f, 37.7002f, -12.8628f)); 
  data[15].mAabb = Aabb(Vector3(-43.6891f, 20.7981f, 16.328f), Vector3(-36.0451f, 25.347f, 19.8207f)); 
  data[16].mAabb = Aabb(Vector3(22.816f, -7.28456f, 2.46424f), Vector3(29.7458f, -2.56977f, 8.85805f)); 
  data[17].mAabb = Aabb(Vector3(-23.5158f, 15.8156f, -36.1629f), Vector3(-15.9707f, 18.1548f, -34.4501f)); 
  data[18].mAabb = Aabb(Vector3(-7.2267f, -0.186035f, 36.0435f), Vector3(0.0438461f, 4.48956f, 41.3732f)); 
  data[19].mAabb = Aabb(Vector3(32.8911f, 23.2844f, -19.2656f), Vector3(37.0968f, 30.917f, -15.9464f)); 
  data[20].mAabb = Aabb(Vector3(3.97429f, -22.6563f, -36.9255f), Vector3(6.37107f, -20.6368f, -36.2931f)); 
  data[21].mAabb = Aabb(Vector3(-27.2979f, -13.3651f, 5.67889f), Vector3(-22.6253f, -8.26297f, 8.32361f)); 
  data[22].mAabb = Aabb(Vector3(-27.5518f, -36.777f, -20.942f), Vector3(-20.9003f, -31.4942f, -15.1392f)); 
  data[23].mAabb = Aabb(Vector3(22.8611f, -6.10482f, -29.1536f), Vector3(25.6629f, -0.348383f, -21.2105f)); 
  data[24].mAabb = Aabb(Vector3(1.63942f, -44.1144f, 17.3234f), Vector3(3.91884f, -39.2367f, 23.6834f)); 
  data[25].mAabb = Aabb(Vector3(-19.1708f, -35.1537f, 25.1218f), Vector3(-12.023f, -27.2393f, 26.826f)); 
  data[26].mAabb = Aabb(Vector3(-11.8751f, -21.0916f, 6.97669f), Vector3(-7.58733f, -15.5254f, 9.04782f)); 
  data[27].mAabb = Aabb(Vector3(-8.39659f, -18.7572f, -21.8103f), Vector3(-1.03953f, -12.8773f, -20.7221f)); 
  data[28].mAabb = Aabb(Vector3(-14.2024f, -24.103f, -22.8525f), Vector3(-9.89049f, -20.1956f, -21.393f)); 
  data[29].mAabb = Aabb(Vector3(39.9799f, -11.6505f, -13.3317f), Vector3(44.2058f, -6.43384f, -5.98626f)); 
  data[30].mAabb = Aabb(Vector3(26.9788f, 17.15f, 10.6735f), Vector3(31.1344f, 17.8423f, 13.2226f)); 
  data[31].mAabb = Aabb(Vector3(-2.19694f, 27.275f, 17.624f), Vector3(2.18803f, 34.6195f, 19.305f)); 
  data[32].mAabb = Aabb(Vector3(-11.4044f, -10.5432f, 45.066f), Vector3(-7.54296f, -5.16596f, 50.2818f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-0.849798f, -4.62445f, -15.8818f));
  ray.mDirection = Vector3(Vector3(-0.34751f, -0.479844f, 0.805597f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(34); 
  data[0].mAabb = Aabb(Vector3(40.8272f, -1.93486f, 18.0374f), Vector3(43.5536f, 2.10475f, 20.0856f)); 
  data[1].mAabb = Aabb(Vector3(34.1255f, 18.8798f, 9.46082f), Vector3(35.9828f, 26.5509f, 13.9822f)); 
  data[2].mAabb = Aabb(Vector3(-43.3754f, 20.362f, 10.0386f), Vector3(-39.1208f, 24.3263f, 14.386f)); 
  data[3].mAabb = Aabb(Vector3(-13.9823f, 37.3627f, 16.4797f), Vector3(-7.75109f, 44.5347f, 18.4001f)); 
  data[4].mAabb = Aabb(Vector3(-24.7065f, -5.22322f, 15.5552f), Vector3(-22.8081f, 0.88345f, 22.2426f)); 
  data[5].mAabb = Aabb(Vector3(-15.7991f, -30.9563f, 30.3535f), Vector3(-9.72255f, -25.1377f, 36.8435f)); 
  data[6].mAabb = Aabb(Vector3(0.64209f, 14.79f, 18.8615f), Vector3(7.03799f, 22.177f, 21.9664f)); 
  data[7].mAabb = Aabb(Vector3(-39.8963f, 2.13677f, 4.74751f), Vector3(-36.3038f, 3.85164f, 10.7595f)); 
  data[8].mAabb = Aabb(Vector3(-0.245939f, -28.0652f, -7.07265f), Vector3(5.75868f, -23.0519f, -1.24176f)); 
  data[9].mAabb = Aabb(Vector3(-1.35484f, -0.0752017f, 17.4595f), Vector3(2.94455f, 2.69848f, 23.3871f)); 
  data[10].mAabb = Aabb(Vector3(23.9972f, -42.946f, -0.57103f), Vector3(31.879f, -38.5524f, 4.8361f)); 
  data[11].mAabb = Aabb(Vector3(3.21223f, 19.9817f, -7.3528f), Vector3(8.37884f, 24.3648f, -3.16591f)); 
  data[12].mAabb = Aabb(Vector3(26.4007f, 7.76146f, 3.47516f), Vector3(29.9377f, 11.7841f, 10.9477f)); 
  data[13].mAabb = Aabb(Vector3(-31.2926f, -24.1523f, 5.75911f), Vector3(-24.8471f, -21.7963f, 10.4071f)); 
  data[14].mAabb = Aabb(Vector3(-24.3996f, 7.84606f, -26.5296f), Vector3(-22.8255f, 14.3577f, -20.2008f)); 
  data[15].mAabb = Aabb(Vector3(30.686f, -36.5379f, -6.79433f), Vector3(33.0065f, -35.5106f, 0.24335f)); 
  data[16].mAabb = Aabb(Vector3(0.33053f, 46.8873f, -1.15714f), Vector3(1.46161f, 49.2038f, 1.24219f)); 
  data[17].mAabb = Aabb(Vector3(8.50339f, 4.70115f, -13.3985f), Vector3(8.99477f, 7.66062f, -6.91125f)); 
  data[18].mAabb = Aabb(Vector3(-0.11088f, -11.3905f, -30.1108f), Vector3(3.51936f, -5.6549f, -22.8162f)); 
  data[19].mAabb = Aabb(Vector3(1.3374f, 5.42421f, -29.4405f), Vector3(4.33584f, 6.34051f, -27.9856f)); 
  data[20].mAabb = Aabb(Vector3(-13.5442f, 32.7025f, -15.181f), Vector3(-8.88852f, 38.8316f, -8.38752f)); 
  data[21].mAabb = Aabb(Vector3(-32.5079f, 31.8712f, -9.78637f), Vector3(-26.5077f, 36.046f, -4.7413f)); 
  data[22].mAabb = Aabb(Vector3(46.3382f, -16.1141f, -9.2312f), Vector3(49.103f, -10.9799f, -1.37477f)); 
  data[23].mAabb = Aabb(Vector3(-33.591f, 19.5353f, -26.4005f), Vector3(-29.5669f, 25.3653f, -21.0572f)); 
  data[24].mAabb = Aabb(Vector3(24.9481f, -37.6765f, -8.62565f), Vector3(29.999f, -34.8485f, -5.44329f)); 
  data[25].mAabb = Aabb(Vector3(-32.7799f, -19.3165f, 2.29273f), Vector3(-29.5855f, -12.5579f, 8.8649f)); 
  data[26].mAabb = Aabb(Vector3(8.04084f, -6.77397f, -49.2965f), Vector3(13.5064f, -1.9657f, -47.0772f)); 
  data[27].mAabb = Aabb(Vector3(-27.726f, -38.4998f, -25.1383f), Vector3(-21.5734f, -35.5941f, -19.6533f)); 
  data[28].mAabb = Aabb(Vector3(33.317f, -2.15688f, -13.5644f), Vector3(40.2347f, 2.19214f, -5.82115f)); 
  data[29].mAabb = Aabb(Vector3(0.667876f, 25.5162f, 37.1237f), Vector3(4.25335f, 31.4696f, 41.5333f)); 
  data[30].mAabb = Aabb(Vector3(-8.98154f, 0.431828f, -5.87935f), Vector3(-6.62465f, 4.43016f, -2.56061f)); 
  data[31].mAabb = Aabb(Vector3(-33.5335f, 21.322f, 12.115f), Vector3(-30.0633f, 25.3569f, 18.6674f)); 
  data[32].mAabb = Aabb(Vector3(24.7286f, 27.0126f, -21.7535f), Vector3(25.7861f, 34.9903f, -16.2021f)); 
  data[33].mAabb = Aabb(Vector3(2.962f, 20.9375f, -33.048f), Vector3(6.71099f, 21.8967f, -29.2698f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(4.50654f, -19.6576f, 1.26855f));
  ray.mDirection = Vector3(Vector3(0.611277f, 0.753241f, 0.242836f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(57); 
  data[0].mAabb = Aabb(Vector3(1.24673f, -36.2046f, -6.57517f), Vector3(5.1836f, -35.2192f, 0.70558f)); 
  data[1].mAabb = Aabb(Vector3(0.534644f, 5.74894f, 2.25588f), Vector3(3.06131f, 10.3907f, 5.80679f)); 
  data[2].mAabb = Aabb(Vector3(-1.63989f, -17.6219f, -38.4741f), Vector3(5.26559f, -11.2158f, -32.081f)); 
  data[3].mAabb = Aabb(Vector3(-5.33184f, -33.9074f, -10.0147f), Vector3(-3.56989f, -26.9249f, -5.50722f)); 
  data[4].mAabb = Aabb(Vector3(-5.07444f, 22.7656f, 33.9933f), Vector3(-0.560272f, 27.6237f, 39.5331f)); 
  data[5].mAabb = Aabb(Vector3(0.77098f, 21.3658f, 20.4386f), Vector3(3.32687f, 21.8305f, 26.3127f)); 
  data[6].mAabb = Aabb(Vector3(-19.0444f, 22.9986f, 17.4196f), Vector3(-16.6631f, 29.0078f, 23.7733f)); 
  data[7].mAabb = Aabb(Vector3(-36.8196f, 7.65617f, -32.678f), Vector3(-29.119f, 14.9583f, -27.6323f)); 
  data[8].mAabb = Aabb(Vector3(15.8282f, -41.8211f, 14.4471f), Vector3(16.5546f, -39.0506f, 22.041f)); 
  data[9].mAabb = Aabb(Vector3(-9.8248f, -13.8631f, -44.2793f), Vector3(-5.74228f, -6.86923f, -40.6405f)); 
  data[10].mAabb = Aabb(Vector3(18.6109f, 24.8926f, -19.8728f), Vector3(20.4568f, 29.8103f, -17.7369f)); 
  data[11].mAabb = Aabb(Vector3(-21.5873f, -4.59628f, -37.8889f), Vector3(-17.7537f, 1.13233f, -34.351f)); 
  data[12].mAabb = Aabb(Vector3(-15.1071f, -47.452f, 8.85144f), Vector3(-9.79208f, -44.3137f, 16.7301f)); 
  data[13].mAabb = Aabb(Vector3(-37.7395f, 11.3828f, -29.2913f), Vector3(-32.1045f, 14.7018f, -22.949f)); 
  data[14].mAabb = Aabb(Vector3(1.44477f, -45.6124f, 10.9954f), Vector3(6.36924f, -41.7431f, 18.8783f)); 
  data[15].mAabb = Aabb(Vector3(-25.1156f, 30.3332f, -0.979759f), Vector3(-19.0258f, 37.8642f, 6.44387f)); 
  data[16].mAabb = Aabb(Vector3(7.39709f, -49.8777f, 5.64657f), Vector3(12.5651f, -44.391f, 11.2982f)); 
  data[17].mAabb = Aabb(Vector3(-28.2682f, 28.785f, -7.19269f), Vector3(-23.3905f, 35.0585f, -4.78524f)); 
  data[18].mAabb = Aabb(Vector3(-44.5873f, -2.42318f, -4.70915f), Vector3(-41.6974f, 0.58638f, -2.24303f)); 
  data[19].mAabb = Aabb(Vector3(-32.4674f, -5.96197f, 1.34998f), Vector3(-28.0657f, -4.05227f, 6.2443f)); 
  data[20].mAabb = Aabb(Vector3(20.5068f, 17.5179f, 35.1659f), Vector3(25.6156f, 23.4012f, 39.0583f)); 
  data[21].mAabb = Aabb(Vector3(5.84197f, -36.588f, -15.4416f), Vector3(6.68033f, -33.6374f, -13.4178f)); 
  data[22].mAabb = Aabb(Vector3(-1.19181f, 20.5002f, -25.8329f), Vector3(4.63491f, 24.4992f, -19.07f)); 
  data[23].mAabb = Aabb(Vector3(-9.49445f, 16.2276f, -25.9957f), Vector3(-4.18404f, 17.6219f, -19.6872f)); 
  data[24].mAabb = Aabb(Vector3(37.374f, -25.6535f, -17.2246f), Vector3(37.8106f, -22.3329f, -11.8683f)); 
  data[25].mAabb = Aabb(Vector3(2.75504f, -40.3288f, -0.836782f), Vector3(3.63933f, -34.5527f, 0.297774f)); 
  data[26].mAabb = Aabb(Vector3(-13.0595f, 18.5188f, 1.73922f), Vector3(-6.91431f, 23.9898f, 7.81992f)); 
  data[27].mAabb = Aabb(Vector3(15.5981f, 31.3143f, -23.142f), Vector3(18.914f, 37.5014f, -16.3871f)); 
  data[28].mAabb = Aabb(Vector3(-18.1866f, -2.84959f, 29.4663f), Vector3(-10.772f, 4.19992f, 35.2401f)); 
  data[29].mAabb = Aabb(Vector3(12.6656f, 44.9429f, 2.28456f), Vector3(14.7135f, 47.0706f, 5.66175f)); 
  data[30].mAabb = Aabb(Vector3(-12.956f, -15.1587f, -32.4965f), Vector3(-5.31063f, -8.12006f, -25.9603f)); 
  data[31].mAabb = Aabb(Vector3(-11.3915f, -7.1183f, -30.9883f), Vector3(-8.81307f, 0.340119f, -24.4217f)); 
  data[32].mAabb = Aabb(Vector3(-33.8417f, 9.67503f, -32.7644f), Vector3(-26.5341f, 16.5364f, -25.7606f)); 
  data[33].mAabb = Aabb(Vector3(28.8539f, 20.0484f, 0.607699f), Vector3(36.3594f, 23.754f, 1.7789f)); 
  data[34].mAabb = Aabb(Vector3(18.1447f, -41.4936f, 18.1132f), Vector3(24.276f, -34.8389f, 23.0936f)); 
  data[35].mAabb = Aabb(Vector3(-26.3045f, -40.4463f, 11.527f), Vector3(-20.9345f, -33.6534f, 17.7123f)); 
  data[36].mAabb = Aabb(Vector3(8.60681f, -34.1085f, 23.3677f), Vector3(10.1809f, -28.9876f, 27.9018f)); 
  data[37].mAabb = Aabb(Vector3(34.7402f, -17.8619f, -33.9786f), Vector3(36.941f, -12.2096f, -26.8194f)); 
  data[38].mAabb = Aabb(Vector3(-15.4285f, -6.06501f, 42.4098f), Vector3(-10.2538f, 0.452654f, 44.0859f)); 
  data[39].mAabb = Aabb(Vector3(-40.851f, 2.12769f, 13.7497f), Vector3(-35.5738f, 8.31205f, 19.5249f)); 
  data[40].mAabb = Aabb(Vector3(-41.5987f, -1.21805f, 8.93167f), Vector3(-38.1949f, 2.93197f, 11.4646f)); 
  data[41].mAabb = Aabb(Vector3(-3.52795f, 17.7386f, 21.1745f), Vector3(-1.90353f, 25.1627f, 27.9679f)); 
  data[42].mAabb = Aabb(Vector3(28.9384f, -33.5644f, -20.5965f), Vector3(35.168f, -28.8348f, -18.0733f)); 
  data[43].mAabb = Aabb(Vector3(9.66859f, -27.5994f, 6.71474f), Vector3(17.6192f, -21.0186f, 8.70191f)); 
  data[44].mAabb = Aabb(Vector3(8.1484f, -24.8544f, -3.88975f), Vector3(11.7541f, -22.6956f, -0.637809f)); 
  data[45].mAabb = Aabb(Vector3(-17.2817f, 27.8958f, 28.6927f), Vector3(-10.743f, 32.1054f, 35.5696f)); 
  data[46].mAabb = Aabb(Vector3(41.661f, 0.71763f, 13.9574f), Vector3(45.157f, 3.82299f, 17.194f)); 
  data[47].mAabb = Aabb(Vector3(-26.9893f, -22.8296f, -37.0409f), Vector3(-23.581f, -15.4496f, -32.3674f)); 
  data[48].mAabb = Aabb(Vector3(9.02702f, -2.65844f, -30.6595f), Vector3(11.0757f, 2.67193f, -29.4841f)); 
  data[49].mAabb = Aabb(Vector3(6.20903f, -40.226f, -6.88175f), Vector3(6.664f, -38.124f, -1.63095f)); 
  data[50].mAabb = Aabb(Vector3(6.59966f, -41.8679f, 18.433f), Vector3(12.9332f, -41.1391f, 19.5044f)); 
  data[51].mAabb = Aabb(Vector3(18.0837f, 7.33423f, 36.9118f), Vector3(21.9702f, 14.2093f, 42.0619f)); 
  data[52].mAabb = Aabb(Vector3(40.0841f, 8.41028f, -15.8337f), Vector3(45.9342f, 11.2619f, -11.1094f)); 
  data[53].mAabb = Aabb(Vector3(-14.3294f, -36.1261f, 20.3673f), Vector3(-9.26922f, -32.3077f, 23.6028f)); 
  data[54].mAabb = Aabb(Vector3(-40.2142f, -25.6264f, -11.089f), Vector3(-34.5877f, -21.9646f, -7.38729f)); 
  data[55].mAabb = Aabb(Vector3(-37.8815f, 21.9863f, -13.0138f), Vector3(-35.0401f, 25.6356f, -10.6015f)); 
  data[56].mAabb = Aabb(Vector3(-24.0774f, -0.864299f, 16.937f), Vector3(-22.163f, 2.8339f, 18.8428f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(10.1021f, 2.61199f, -1.84554f));
  ray.mDirection = Vector3(Vector3(-0.826505f, 0.179287f, -0.533616f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mAabb = Aabb(Vector3(-10.6383f, 1.84164f, 41.545f), Vector3(-6.23801f, 9.72775f, 48.0752f)); 
  data[1].mAabb = Aabb(Vector3(24.1541f, -35.2515f, -19.7452f), Vector3(30.2202f, -28.3641f, -14.0289f)); 
  data[2].mAabb = Aabb(Vector3(21.2044f, -28.9724f, -10.2096f), Vector3(22.651f, -22.7971f, -2.41256f)); 
  data[3].mAabb = Aabb(Vector3(38.8292f, -7.61544f, -24.5762f), Vector3(41.7692f, -0.611854f, -21.1853f)); 
  data[4].mAabb = Aabb(Vector3(-34.3622f, -20.3101f, -27.6655f), Vector3(-33.1298f, -19.8461f, -19.9689f)); 
  data[5].mAabb = Aabb(Vector3(-35.3379f, -37, -15.0343f), Vector3(-31.1575f, -32.7653f, -8.16713f)); 
  data[6].mAabb = Aabb(Vector3(0.60281f, -16.2447f, 15.1297f), Vector3(7.28191f, -8.88994f, 20.8599f)); 
  data[7].mAabb = Aabb(Vector3(31.6265f, 11.386f, -10.1099f), Vector3(39.3528f, 15.893f, -7.17498f)); 
  data[8].mAabb = Aabb(Vector3(-1.73216f, -15.0586f, -26.9996f), Vector3(0.538212f, -14.33f, -21.3565f)); 
  data[9].mAabb = Aabb(Vector3(26.1876f, 33.5791f, -1.65571f), Vector3(33.9853f, 35.9792f, 2.27605f)); 
  data[10].mAabb = Aabb(Vector3(-26.5493f, 16.7139f, 2.50319f), Vector3(-21.2503f, 20.7829f, 6.01723f)); 
  data[11].mAabb = Aabb(Vector3(-0.553908f, -31.3384f, -31.8523f), Vector3(6.28732f, -29.9742f, -30.5071f)); 
  data[12].mAabb = Aabb(Vector3(-25.4555f, 12.1081f, -28.1563f), Vector3(-20.4927f, 14.5191f, -20.4091f)); 
  data[13].mAabb = Aabb(Vector3(40.6613f, -4.42231f, -15.2207f), Vector3(45.1701f, -0.396848f, -13.6784f)); 
  data[14].mAabb = Aabb(Vector3(-30.2346f, 34.8881f, 10.1796f), Vector3(-27.402f, 41.0808f, 16.3515f)); 
  data[15].mAabb = Aabb(Vector3(43.022f, 1.34607f, -1.67613f), Vector3(47.3135f, 4.09099f, 5.87043f)); 
  data[16].mAabb = Aabb(Vector3(-16.4732f, -1.10957f, 22.9894f), Vector3(-9.27297f, 1.55394f, 27.459f)); 
  data[17].mAabb = Aabb(Vector3(2.56957f, -40.0319f, 23.3109f), Vector3(8.77806f, -35.6404f, 27.8765f)); 
  data[18].mAabb = Aabb(Vector3(-14.0605f, 3.52346f, 10.3991f), Vector3(-9.243f, 9.0366f, 15.9354f)); 
  data[19].mAabb = Aabb(Vector3(-33.8047f, 10.3991f, -34.0707f), Vector3(-30.4289f, 15.8913f, -30.8788f)); 
  data[20].mAabb = Aabb(Vector3(-26.7589f, -31.2897f, 28.9421f), Vector3(-21.0154f, -24.5067f, 29.7353f)); 
  data[21].mAabb = Aabb(Vector3(-20.1952f, 22.5707f, -35.1104f), Vector3(-16.1363f, 27.4949f, -27.1308f)); 
  data[22].mAabb = Aabb(Vector3(-27.8167f, 34.9892f, 0.522528f), Vector3(-25.4111f, 38.7159f, 3.18465f)); 
  data[23].mAabb = Aabb(Vector3(4.01917f, -18.9308f, 18.1972f), Vector3(9.84333f, -18.2254f, 21.5129f)); 
  data[24].mAabb = Aabb(Vector3(0.0817554f, -26.2902f, 39.8366f), Vector3(7.07954f, -22.8126f, 46.6479f)); 
  data[25].mAabb = Aabb(Vector3(-10.2126f, 21.0336f, -43.3575f), Vector3(-6.00809f, 25.8882f, -36.6577f)); 
  data[26].mAabb = Aabb(Vector3(30.6259f, -4.59138f, -41.3976f), Vector3(31.3395f, -1.20607f, -36.8066f)); 
  data[27].mAabb = Aabb(Vector3(-21.1422f, -0.96985f, 8.49644f), Vector3(-20.6559f, 6.70404f, 16.2316f)); 
  data[28].mAabb = Aabb(Vector3(5.75286f, -45.0608f, 19.4455f), Vector3(10.2946f, -43.1708f, 22.1122f)); 
  data[29].mAabb = Aabb(Vector3(-9.95049f, -0.431554f, 21.6873f), Vector3(-2.51433f, 0.993624f, 24.2439f)); 
  data[30].mAabb = Aabb(Vector3(-45.2851f, 0.00585222f, 0.792567f), Vector3(-38.9602f, 7.39562f, 3.85757f)); 
  data[31].mAabb = Aabb(Vector3(28.5682f, 7.05253f, -35.0864f), Vector3(30.9404f, 9.41011f, -32.2868f)); 
  data[32].mAabb = Aabb(Vector3(-1.9402f, 35.5323f, 30.9564f), Vector3(5.06199f, 36.6989f, 37.9987f)); 
  data[33].mAabb = Aabb(Vector3(-2.89408f, -37.3482f, -30.3941f), Vector3(-1.69017f, -31.1001f, -29.5724f)); 
  data[34].mAabb = Aabb(Vector3(-6.99352f, -9.7767f, -44.5632f), Vector3(-2.65215f, -5.54087f, -40.9715f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(10.1538f, 12.8054f, 1.44652f));
  ray.mDirection = Vector3(Vector3(-0.894426f, -0.239423f, 0.377728f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(42.3786f, -0.467583f, -24.2724f), Vector3(45.5638f, 3.58015f, -20.5763f)); 
  data[1].mAabb = Aabb(Vector3(-1.64109f, 12.132f, -12.6115f), Vector3(6.29513f, 17.6187f, -5.21318f)); 
  data[2].mAabb = Aabb(Vector3(-5.4499f, 38.3958f, -14.6591f), Vector3(-5.0383f, 44.0662f, -12.5553f)); 
  data[3].mAabb = Aabb(Vector3(-13.102f, -0.94607f, -3.74209f), Vector3(-11.1077f, 6.7153f, -3.01622f)); 
  data[4].mAabb = Aabb(Vector3(-44.7102f, -6.75097f, 22.7373f), Vector3(-38.4184f, -0.295459f, 29.4818f)); 
  data[5].mAabb = Aabb(Vector3(35.2249f, 23.2069f, 12.4361f), Vector3(39.6162f, 26.6467f, 16.6086f)); 
  data[6].mAabb = Aabb(Vector3(10.0636f, 13.991f, 26.8772f), Vector3(12.8876f, 19.901f, 30.8595f)); 
  data[7].mAabb = Aabb(Vector3(-21.5304f, -30.7821f, 10.8832f), Vector3(-19.1939f, -24.1316f, 11.3582f)); 
  data[8].mAabb = Aabb(Vector3(-25.4187f, 20.6455f, -29.0087f), Vector3(-23.2323f, 26.5738f, -23.1544f)); 
  data[9].mAabb = Aabb(Vector3(-21.1463f, -1.82795f, -32.0365f), Vector3(-19.9656f, -1.07795f, -25.4938f)); 
  data[10].mAabb = Aabb(Vector3(13.6927f, -35.5379f, 11.5709f), Vector3(20.6935f, -31.8938f, 13.7337f)); 
  data[11].mAabb = Aabb(Vector3(2.48083f, 27.3625f, 31.1937f), Vector3(3.55438f, 31.4543f, 37.1009f)); 
  data[12].mAabb = Aabb(Vector3(-12.3456f, -43.6072f, 15.92f), Vector3(-5.45428f, -39.0033f, 19.5284f)); 
  data[13].mAabb = Aabb(Vector3(-6.791f, -2.08751f, 4.26552f), Vector3(-0.789869f, 5.58105f, 9.60446f)); 
  data[14].mAabb = Aabb(Vector3(9.79498f, -21.4326f, 19.8957f), Vector3(10.4457f, -16.7104f, 23.6648f)); 
  data[15].mAabb = Aabb(Vector3(22.4016f, -4.33212f, 29.7499f), Vector3(25.6076f, -2.63442f, 37.0937f)); 
  data[16].mAabb = Aabb(Vector3(-12.4633f, 13.7146f, -47.5335f), Vector3(-5.7411f, 21, -44.3896f)); 
  data[17].mAabb = Aabb(Vector3(-12.9696f, -44.9054f, -9.04494f), Vector3(-6.15901f, -37.697f, -3.39079f)); 
  data[18].mAabb = Aabb(Vector3(19.8347f, -13.1021f, 30.3373f), Vector3(25.5457f, -12.4402f, 37.0048f)); 
  data[19].mAabb = Aabb(Vector3(-23.9565f, 14.1754f, 31.9545f), Vector3(-19.1088f, 16.3662f, 32.5785f)); 
  data[20].mAabb = Aabb(Vector3(-7.07904f, 27.8839f, 21.0712f), Vector3(-1.43718f, 28.8698f, 22.1861f)); 
  data[21].mAabb = Aabb(Vector3(21.2605f, 21.4505f, -6.58774f), Vector3(23.5798f, 21.9285f, -1.74329f)); 
  data[22].mAabb = Aabb(Vector3(4.68373f, 31.7591f, -20.8904f), Vector3(11.1541f, 33.6306f, -14.6861f)); 
  data[23].mAabb = Aabb(Vector3(9.90331f, -20.2779f, -15.8185f), Vector3(16.9932f, -14.566f, -10.3341f)); 
  data[24].mAabb = Aabb(Vector3(-12.4952f, -10.9346f, -21.7283f), Vector3(-11.1345f, -7.10928f, -21.1198f)); 
  data[25].mAabb = Aabb(Vector3(-47.636f, 12.1789f, 12.7476f), Vector3(-40.8077f, 19.5443f, 18.4289f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(5.8359f, 11.0692f, -10.4854f));
  ray.mDirection = Vector3(Vector3(0.0751983f, 0.975419f, -0.207129f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(-9.75794f, 27.623f, 21.7351f), Vector3(-3.05519f, 33.4337f, 28.6958f)); 
  data[1].mAabb = Aabb(Vector3(-11.7162f, -8.56936f, -50.7462f), Vector3(-5.66381f, -4.73385f, -43.9289f)); 
  data[2].mAabb = Aabb(Vector3(-3.3671f, 16.3757f, -36.6983f), Vector3(-2.33344f, 17.3686f, -29.6247f)); 
  data[3].mAabb = Aabb(Vector3(-10.7172f, 12.1773f, 0.93743f), Vector3(-10.3093f, 13.1563f, 7.86888f)); 
  data[4].mAabb = Aabb(Vector3(-22.3886f, -27.5903f, -25.2193f), Vector3(-17.8206f, -23.6112f, -18.2486f)); 
  data[5].mAabb = Aabb(Vector3(-35.3211f, -25.1994f, 24.3336f), Vector3(-33.6347f, -22.3257f, 30.1078f)); 
  data[6].mAabb = Aabb(Vector3(-32.5467f, -24.3732f, 16.2831f), Vector3(-26.6494f, -20.6859f, 21.102f)); 
  data[7].mAabb = Aabb(Vector3(-15.3233f, -43.3957f, 14.4218f), Vector3(-13.2742f, -39.845f, 21.5249f)); 
  data[8].mAabb = Aabb(Vector3(-3.86896f, -3.74644f, 32.3867f), Vector3(-0.278852f, 0.0672623f, 35.1077f)); 
  data[9].mAabb = Aabb(Vector3(-22.282f, -14.0886f, 4.32971f), Vector3(-14.5228f, -9.41692f, 5.9082f)); 
  data[10].mAabb = Aabb(Vector3(-2.40717f, 29.5953f, -29.3072f), Vector3(2.03255f, 33.9366f, -25.0635f)); 
  data[11].mAabb = Aabb(Vector3(19.8856f, 27.4275f, 4.17932f), Vector3(22.2007f, 31.0032f, 9.65719f)); 
  data[12].mAabb = Aabb(Vector3(-7.2352f, 23.5221f, 8.5964f), Vector3(-5.93411f, 29.7987f, 11.6491f)); 
  data[13].mAabb = Aabb(Vector3(4.3657f, 13.7234f, 11.2864f), Vector3(6.94827f, 20.0606f, 12.8559f)); 
  data[14].mAabb = Aabb(Vector3(-10.7389f, 5.61363f, 14.7125f), Vector3(-4.4585f, 7.945f, 20.834f)); 
  data[15].mAabb = Aabb(Vector3(-21.086f, -42.9771f, 13.496f), Vector3(-16.7036f, -35.3304f, 15.8144f)); 
  data[16].mAabb = Aabb(Vector3(0.832486f, 3.87636f, -25.7409f), Vector3(6.87259f, 7.53466f, -23.77f)); 
  data[17].mAabb = Aabb(Vector3(25.7994f, 21.6677f, -30.2136f), Vector3(31.2012f, 26.3989f, -26.8346f)); 
  data[18].mAabb = Aabb(Vector3(34.6456f, 4.4975f, -18.9603f), Vector3(40.8261f, 9.05364f, -17.9531f)); 
  data[19].mAabb = Aabb(Vector3(26.8593f, -16.458f, -10.9006f), Vector3(27.5474f, -14.1371f, -8.95985f)); 
  data[20].mAabb = Aabb(Vector3(-12.6441f, -29.2853f, 27.9052f), Vector3(-11.5833f, -25.6314f, 33.1215f)); 
  data[21].mAabb = Aabb(Vector3(-7.01679f, -25.0381f, 34.4435f), Vector3(-5.2277f, -17.5574f, 41.9339f)); 
  data[22].mAabb = Aabb(Vector3(-33.0906f, 5.51659f, -36.4042f), Vector3(-27.4648f, 11.8058f, -32.5225f)); 
  data[23].mAabb = Aabb(Vector3(29.2768f, 24.0871f, -21.0874f), Vector3(34.4787f, 25.7983f, -14.3401f)); 
  data[24].mAabb = Aabb(Vector3(-3.77737f, -2.83035f, 17.6874f), Vector3(-2.98655f, 2.16022f, 22.2714f)); 
  data[25].mAabb = Aabb(Vector3(-8.27692f, 32.8931f, 21.7199f), Vector3(-1.48579f, 36.0358f, 29.6649f)); 
  data[26].mAabb = Aabb(Vector3(0.232391f, 4.1979f, -50.9286f), Vector3(5.19397f, 11.378f, -44.7809f)); 
  data[27].mAabb = Aabb(Vector3(-23.4616f, -19.9193f, -3.44682f), Vector3(-20.4168f, -18.8963f, 1.16198f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-5.86404f, 3.32949f, 9.55027f));
  ray.mDirection = Vector3(Vector3(0.0623949f, 0.863324f, 0.500778f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mAabb = Aabb(Vector3(-33.6278f, 23.3002f, 21.8913f), Vector3(-29.1306f, 28.1461f, 25.5642f)); 
  data[1].mAabb = Aabb(Vector3(4.24705f, -40.8049f, 13.6053f), Vector3(5.83899f, -33.8577f, 21.4868f)); 
  data[2].mAabb = Aabb(Vector3(-28.0167f, -29.3895f, -6.32378f), Vector3(-20.9593f, -28.729f, -1.3673f)); 
  data[3].mAabb = Aabb(Vector3(28.3523f, 27.0865f, 18.7168f), Vector3(32.0679f, 29.0758f, 26.1166f)); 
  data[4].mAabb = Aabb(Vector3(4.10739f, -2.37086f, -23.5542f), Vector3(5.79651f, 2.11918f, -19.4599f)); 
  data[5].mAabb = Aabb(Vector3(-24.0231f, 16.4596f, 18.0735f), Vector3(-16.1984f, 18.7926f, 23.9991f)); 
  data[6].mAabb = Aabb(Vector3(4.57065f, 4.09807f, 3.94361f), Vector3(10.2696f, 11.601f, 4.86107f)); 
  data[7].mAabb = Aabb(Vector3(-8.52314f, 8.84779f, -23.1958f), Vector3(-5.93849f, 12.9595f, -19.9262f)); 
  data[8].mAabb = Aabb(Vector3(-22.1978f, 19.4922f, 23.4651f), Vector3(-15.4649f, 24.1377f, 27.3981f)); 
  data[9].mAabb = Aabb(Vector3(-7.85067f, -15.0594f, -9.05637f), Vector3(-4.59595f, -13.6593f, -1.20412f)); 
  data[10].mAabb = Aabb(Vector3(-30.2344f, -0.111037f, 14.0442f), Vector3(-26.1526f, 2.6645f, 19.0176f)); 
  data[11].mAabb = Aabb(Vector3(8.72862f, 31.9224f, 30.1275f), Vector3(9.63819f, 36.5207f, 33.5299f)); 
  data[12].mAabb = Aabb(Vector3(1.43263f, 14.9574f, -38.1948f), Vector3(7.0826f, 17.1677f, -34.9929f)); 
  data[13].mAabb = Aabb(Vector3(-20.107f, -31.5267f, -13.465f), Vector3(-17.0524f, -26.4916f, -12.4685f)); 
  data[14].mAabb = Aabb(Vector3(-10.7581f, 7.98885f, 24.4345f), Vector3(-4.76321f, 12.6412f, 27.6201f)); 
  data[15].mAabb = Aabb(Vector3(-12.6014f, -37.7496f, 8.73324f), Vector3(-9.65468f, -36.2931f, 12.9786f)); 
  data[16].mAabb = Aabb(Vector3(-7.23334f, -27.3496f, 25.4283f), Vector3(-2.1776f, -21.8949f, 32.1782f)); 
  data[17].mAabb = Aabb(Vector3(36.9842f, -3.45302f, 21.2205f), Vector3(44.518f, -0.793915f, 21.6613f)); 
  data[18].mAabb = Aabb(Vector3(-23.6738f, -28.653f, 10.2992f), Vector3(-21.8917f, -20.6646f, 17.2218f)); 
  data[19].mAabb = Aabb(Vector3(3.46181f, -30.3096f, -13.0895f), Vector3(10.3847f, -22.3769f, -10.0083f)); 
  data[20].mAabb = Aabb(Vector3(32.3858f, -3.67629f, 23.8383f), Vector3(39.4576f, -0.799862f, 27.5272f)); 
  data[21].mAabb = Aabb(Vector3(12.678f, 21.6732f, -28.4062f), Vector3(20.3709f, 22.846f, -20.6273f)); 
  data[22].mAabb = Aabb(Vector3(32.1677f, 8.1949f, 10.4816f), Vector3(34.8017f, 11.2474f, 14.7409f)); 
  data[23].mAabb = Aabb(Vector3(39.5989f, 12.9593f, -15.8701f), Vector3(44.961f, 18.8895f, -14.9014f)); 
  data[24].mAabb = Aabb(Vector3(-17.0453f, -8.47334f, 41.915f), Vector3(-12.8544f, -7.06208f, 46.6857f)); 
  data[25].mAabb = Aabb(Vector3(-22.753f, -14.9914f, -35.0153f), Vector3(-19.6291f, -13.3403f, -29.9849f)); 
  data[26].mAabb = Aabb(Vector3(-6.12725f, -28.6963f, 5.11263f), Vector3(-2.50235f, -25.8853f, 12.4426f)); 
  data[27].mAabb = Aabb(Vector3(-15.9311f, -2.75272f, 37.8956f), Vector3(-10.5685f, 4.02913f, 40.7256f)); 
  data[28].mAabb = Aabb(Vector3(16.2713f, 35.9949f, -7.57406f), Vector3(17.9623f, 40.7846f, -6.66193f)); 
  data[29].mAabb = Aabb(Vector3(5.40636f, 3.09786f, 3.10606f), Vector3(7.26898f, 10.4874f, 9.61074f)); 
  data[30].mAabb = Aabb(Vector3(23.155f, -7.21844f, 28.5197f), Vector3(27.8571f, -4.45821f, 34.1486f)); 
  data[31].mAabb = Aabb(Vector3(-24.1854f, 16.5145f, 34.2814f), Vector3(-23.7629f, 23.0841f, 41.3718f)); 
  data[32].mAabb = Aabb(Vector3(-46.4965f, 7.47769f, 18.1944f), Vector3(-39.6968f, 7.92176f, 20.668f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(16.3607f, 17.8483f, -0.194343f));
  ray.mDirection = Vector3(Vector3(-0.689713f, -0.613443f, 0.384686f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(34.4564f, 3.07812f, 7.2565f), Vector3(35.4724f, 9.53316f, 8.98436f)); 
  data[1].mAabb = Aabb(Vector3(-16.8731f, 4.42183f, -6.74743f), Vector3(-9.64147f, 5.47475f, -4.12103f)); 
  data[2].mAabb = Aabb(Vector3(-1.01142f, -16.0438f, 13.3694f), Vector3(0.452493f, -8.49776f, 20.6555f)); 
  data[3].mAabb = Aabb(Vector3(-23.5124f, -26.7926f, -39.0513f), Vector3(-20.8226f, -26.3541f, -32.1324f)); 
  data[4].mAabb = Aabb(Vector3(10.0606f, -30.2191f, 14.0795f), Vector3(12.1902f, -26.321f, 15.6345f)); 
  data[5].mAabb = Aabb(Vector3(-3.75068f, -13.9793f, -36.0416f), Vector3(-0.784718f, -9.2218f, -31.0573f)); 
  data[6].mAabb = Aabb(Vector3(29.0115f, 8.16653f, 1.74221f), Vector3(31.3976f, 14.639f, 5.95184f)); 
  data[7].mAabb = Aabb(Vector3(-17.9522f, -20.5453f, -16.1364f), Vector3(-15.0401f, -13.6862f, -9.87409f)); 
  data[8].mAabb = Aabb(Vector3(34.1509f, -4.84562f, 9.86425f), Vector3(40.1945f, -4.21577f, 14.0611f)); 
  data[9].mAabb = Aabb(Vector3(-22.5089f, -2.80122f, -47.1568f), Vector3(-22.0013f, 3.14263f, -40.0192f)); 
  data[10].mAabb = Aabb(Vector3(-13.2261f, -6.13822f, 26.1388f), Vector3(-7.58402f, 1.3601f, 32.6238f)); 
  data[11].mAabb = Aabb(Vector3(8.74745f, 25.38f, 10.3172f), Vector3(11.3135f, 30.0723f, 12.7149f)); 
  data[12].mAabb = Aabb(Vector3(44.6168f, 18.8436f, 2.42936f), Vector3(47.0043f, 19.2628f, 6.80134f)); 
  data[13].mAabb = Aabb(Vector3(-17.1193f, 31.479f, -6.53569f), Vector3(-12.4305f, 39.1388f, -6.02853f)); 
  data[14].mAabb = Aabb(Vector3(-14.9389f, -21.6045f, -11.9852f), Vector3(-14.2518f, -17.6152f, -7.57032f)); 
  data[15].mAabb = Aabb(Vector3(-3.00615f, 34.6756f, 27.3958f), Vector3(4.91847f, 35.4154f, 31.8543f)); 
  data[16].mAabb = Aabb(Vector3(4.60533f, -34.7681f, -29.7715f), Vector3(8.25668f, -26.9632f, -22.9294f)); 
  data[17].mAabb = Aabb(Vector3(-33.3371f, 25.8214f, -23.8944f), Vector3(-31.7345f, 27.542f, -16.5337f)); 
  data[18].mAabb = Aabb(Vector3(-14.0593f, -21.8368f, -1.71436f), Vector3(-12.3368f, -14.611f, 5.99804f)); 
  data[19].mAabb = Aabb(Vector3(29.8648f, 25.0956f, -16.1069f), Vector3(35.0641f, 27.2305f, -13.5723f)); 
  data[20].mAabb = Aabb(Vector3(8.85222f, -8.43642f, -39.8003f), Vector3(12.4454f, -7.56628f, -38.1186f)); 
  data[21].mAabb = Aabb(Vector3(-46.7432f, -5.54978f, 13.3324f), Vector3(-39.2446f, -1.12491f, 17.4015f)); 
  data[22].mAabb = Aabb(Vector3(4.64614f, 34.272f, -1.22666f), Vector3(7.42353f, 40.6241f, 4.81692f)); 
  data[23].mAabb = Aabb(Vector3(-5.69616f, -16.0666f, -4.56383f), Vector3(-3.38311f, -11.0359f, -2.95797f)); 
  data[24].mAabb = Aabb(Vector3(-28.9755f, -37.0137f, -5.22558f), Vector3(-23.9791f, -29.4414f, -3.92681f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(13.295f, -4.58963f, 3.00536f));
  ray.mDirection = Vector3(Vector3(-0.825577f, -0.425545f, -0.370586f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(34); 
  data[0].mAabb = Aabb(Vector3(27.8401f, 32.5856f, 8.05947f), Vector3(35.6428f, 34.7521f, 11.5642f)); 
  data[1].mAabb = Aabb(Vector3(4.88174f, -1.31114f, -19.5427f), Vector3(11.1113f, -0.616348f, -13.3934f)); 
  data[2].mAabb = Aabb(Vector3(33.2465f, 20.0135f, 18.9646f), Vector3(36.006f, 22.4681f, 20.2756f)); 
  data[3].mAabb = Aabb(Vector3(-20.5967f, 33.1278f, -11.2583f), Vector3(-13.9631f, 39.0519f, -3.68091f)); 
  data[4].mAabb = Aabb(Vector3(5.63741f, -17.2178f, -3.89957f), Vector3(6.77521f, -10.5784f, -1.71873f)); 
  data[5].mAabb = Aabb(Vector3(-37.1375f, 4.85708f, 12.4834f), Vector3(-30.5155f, 8.07516f, 14.1798f)); 
  data[6].mAabb = Aabb(Vector3(-38.4464f, -19.0009f, -14.9454f), Vector3(-35.6811f, -16.8284f, -11.7869f)); 
  data[7].mAabb = Aabb(Vector3(-19.8767f, -29.0924f, 12.9644f), Vector3(-16.702f, -22.3231f, 15.6966f)); 
  data[8].mAabb = Aabb(Vector3(-16.3783f, 9.63998f, -5.24387f), Vector3(-14.5323f, 10.6734f, -1.71801f)); 
  data[9].mAabb = Aabb(Vector3(27.2091f, 3.65277f, 1.8124f), Vector3(29.9886f, 11.1831f, 4.58886f)); 
  data[10].mAabb = Aabb(Vector3(38.5755f, -6.24758f, 18.3025f), Vector3(40.6325f, -5.15315f, 20.2616f)); 
  data[11].mAabb = Aabb(Vector3(-17.7895f, -1.83126f, 15.4858f), Vector3(-10.081f, 4.00543f, 21.4565f)); 
  data[12].mAabb = Aabb(Vector3(-9.74436f, -26.8203f, 24.7425f), Vector3(-2.07441f, -22.6177f, 30.6727f)); 
  data[13].mAabb = Aabb(Vector3(-4.0619f, -21.8835f, 27.3632f), Vector3(2.16468f, -20.0562f, 34.2545f)); 
  data[14].mAabb = Aabb(Vector3(1.76352f, 23.8228f, -40.2664f), Vector3(3.62637f, 28.1164f, -38.4476f)); 
  data[15].mAabb = Aabb(Vector3(-4.54196f, 38.9472f, 6.06402f), Vector3(-1.94293f, 42.2249f, 13.9671f)); 
  data[16].mAabb = Aabb(Vector3(-18.9316f, 28.2394f, -1.98654f), Vector3(-15.7448f, 35.2863f, 3.20814f)); 
  data[17].mAabb = Aabb(Vector3(3.0444f, 13.6727f, -18.4212f), Vector3(7.99856f, 21.5296f, -11.5848f)); 
  data[18].mAabb = Aabb(Vector3(-9.03881f, 6.38184f, -3.92308f), Vector3(-2.89549f, 8.37898f, -0.990289f)); 
  data[19].mAabb = Aabb(Vector3(-12.1877f, -20.0374f, -24.8896f), Vector3(-5.31844f, -15.8164f, -20.7324f)); 
  data[20].mAabb = Aabb(Vector3(-24.7172f, 15.3266f, -20.2341f), Vector3(-21.4752f, 17.1408f, -14.8077f)); 
  data[21].mAabb = Aabb(Vector3(15.4242f, -11.9084f, 19.865f), Vector3(17.7637f, -9.75148f, 26.4518f)); 
  data[22].mAabb = Aabb(Vector3(8.21655f, 1.12119f, -49.0401f), Vector3(9.11452f, 2.38332f, -43.6837f)); 
  data[23].mAabb = Aabb(Vector3(-13.41f, 30.7245f, 17.0955f), Vector3(-11.8165f, 37.7111f, 25.0336f)); 
  data[24].mAabb = Aabb(Vector3(10.8716f, 0.42148f, -43.1049f), Vector3(17.002f, 1.37419f, -42.6211f)); 
  data[25].mAabb = Aabb(Vector3(28.1429f, 34.514f, 8.08694f), Vector3(31.9599f, 36.1597f, 15.8151f)); 
  data[26].mAabb = Aabb(Vector3(2.81009f, -14.8454f, 9.45523f), Vector3(5.63572f, -13.2428f, 15.5802f)); 
  data[27].mAabb = Aabb(Vector3(19.1016f, -40.0434f, -0.378225f), Vector3(19.7477f, -36.3937f, 2.3544f)); 
  data[28].mAabb = Aabb(Vector3(5.8555f, 9.12942f, 18.9019f), Vector3(6.78525f, 16.6405f, 20.8737f)); 
  data[29].mAabb = Aabb(Vector3(-32.9819f, -0.0127476f, -14.8276f), Vector3(-28.4842f, 1.94636f, -10.3877f)); 
  data[30].mAabb = Aabb(Vector3(17.7806f, -20.5449f, -29.3793f), Vector3(20.3098f, -14.7797f, -24.4553f)); 
  data[31].mAabb = Aabb(Vector3(17.8089f, -19.9406f, 19.9244f), Vector3(18.6018f, -19.3852f, 22.2954f)); 
  data[32].mAabb = Aabb(Vector3(-10.5046f, -3.26924f, -14.2037f), Vector3(-4.3202f, 2.22139f, -6.81602f)); 
  data[33].mAabb = Aabb(Vector3(30.8068f, 14.7212f, -10.5603f), Vector3(31.3474f, 18.8631f, -8.33194f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-1.92852f, 9.95669f, 21.7409f));
  ray.mDirection = Vector3(Vector3(-0.385524f, 0.673351f, -0.630848f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(-21.2796f, -2.27429f, 42.7336f), Vector3(-14.3869f, 5.44228f, 46.5628f)); 
  data[1].mAabb = Aabb(Vector3(-2.59038f, -4.65577f, 27.0092f), Vector3(2.88796f, -1.25678f, 29.2376f)); 
  data[2].mAabb = Aabb(Vector3(17.1098f, 27.8554f, 14.0588f), Vector3(18.3207f, 32.2633f, 16.0662f)); 
  data[3].mAabb = Aabb(Vector3(-27.592f, 14.7115f, -11.4274f), Vector3(-22.5926f, 22.4647f, -7.27741f)); 
  data[4].mAabb = Aabb(Vector3(-8.5673f, -29.6597f, 16.9298f), Vector3(-7.79063f, -28.5036f, 24.885f)); 
  data[5].mAabb = Aabb(Vector3(-5.3218f, 28.6173f, -15.6051f), Vector3(-0.337033f, 35.127f, -10.2905f)); 
  data[6].mAabb = Aabb(Vector3(29.1476f, -19.6562f, -20.6372f), Vector3(30.0618f, -14.6483f, -18.1662f)); 
  data[7].mAabb = Aabb(Vector3(23.6375f, -24.6386f, -25.7908f), Vector3(29.8425f, -23.9005f, -22.0189f)); 
  data[8].mAabb = Aabb(Vector3(-14.6678f, 28.6147f, -19.6226f), Vector3(-14.2571f, 35.1215f, -15.8086f)); 
  data[9].mAabb = Aabb(Vector3(15.7501f, 30.1499f, -22.5884f), Vector3(20.5623f, 30.822f, -15.6372f)); 
  data[10].mAabb = Aabb(Vector3(29.8795f, 32.0848f, -6.29682f), Vector3(34.5936f, 39.2324f, -2.91407f)); 
  data[11].mAabb = Aabb(Vector3(-8.30295f, -1.83803f, 35.2748f), Vector3(-0.793036f, 0.212924f, 41.9604f)); 
  data[12].mAabb = Aabb(Vector3(-5.78365f, -5.42937f, -45.0744f), Vector3(-0.270053f, -1.74602f, -44.6247f)); 
  data[13].mAabb = Aabb(Vector3(19.2469f, 20.3684f, -17.8463f), Vector3(25.6083f, 27.9105f, -16.4502f)); 
  data[14].mAabb = Aabb(Vector3(33.1674f, -6.13471f, 20.9727f), Vector3(36.1371f, -2.04268f, 23.6821f)); 
  data[15].mAabb = Aabb(Vector3(-34.2151f, -10.7782f, 5.7492f), Vector3(-30.0122f, -8.02168f, 8.24525f)); 
  data[16].mAabb = Aabb(Vector3(43.9913f, 14.8923f, 2.01708f), Vector3(47.8391f, 16.7104f, 9.62162f)); 
  data[17].mAabb = Aabb(Vector3(-5.5737f, -31.9072f, 23.8769f), Vector3(-2.55555f, -26.5374f, 27.7536f)); 
  data[18].mAabb = Aabb(Vector3(-26.4637f, -4.90941f, 12.8176f), Vector3(-25.4367f, 0.99083f, 18.8076f)); 
  data[19].mAabb = Aabb(Vector3(-27.5333f, -29.0082f, -32.2118f), Vector3(-26.8896f, -23.3489f, -28.6683f)); 
  data[20].mAabb = Aabb(Vector3(-15.0923f, -8.54723f, 2.21332f), Vector3(-10.6519f, -0.547698f, 10.1157f)); 
  data[21].mAabb = Aabb(Vector3(-2.01784f, 19.3789f, 1.53947f), Vector3(4.94399f, 19.9545f, 4.59658f)); 
  data[22].mAabb = Aabb(Vector3(-28.7721f, -5.84753f, 15.0108f), Vector3(-21.4596f, 0.890253f, 17.8021f)); 
  data[23].mAabb = Aabb(Vector3(-30.4059f, -11.5071f, -22.5801f), Vector3(-27.0603f, -11.0542f, -17.1329f)); 
  data[24].mAabb = Aabb(Vector3(33.8853f, -22.1018f, 10.3303f), Vector3(37.2393f, -21.3407f, 16.311f)); 
  data[25].mAabb = Aabb(Vector3(36.7819f, 28.2811f, -6.19887f), Vector3(37.7038f, 33.5588f, -0.538917f)); 
  data[26].mAabb = Aabb(Vector3(35.3626f, -0.482295f, 25.6795f), Vector3(42.5417f, 2.05852f, 30.5972f)); 
  data[27].mAabb = Aabb(Vector3(-28.7755f, -7.88235f, 18.0696f), Vector3(-21.3678f, -3.45888f, 19.8705f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(0.734514f, -10.8889f, -11.5217f));
  ray.mDirection = Vector3(Vector3(-0.665881f, 0.273436f, 0.694144f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(-14.4188f, -16.4936f, 36.5659f), Vector3(-10.0274f, -13.7424f, 37.0169f)); 
  data[1].mAabb = Aabb(Vector3(-16.844f, 17.9272f, 35.5845f), Vector3(-12.9602f, 22.732f, 37.808f)); 
  data[2].mAabb = Aabb(Vector3(-24.7744f, -8.24775f, 3.01426f), Vector3(-21.3137f, -1.75908f, 9.47325f)); 
  data[3].mAabb = Aabb(Vector3(15.4676f, -0.518881f, -39.2601f), Vector3(22.7838f, 4.92815f, -36.8848f)); 
  data[4].mAabb = Aabb(Vector3(30.8187f, -7.93081f, 15.1174f), Vector3(38.5854f, -2.8071f, 17.0042f)); 
  data[5].mAabb = Aabb(Vector3(-36.7082f, -28.1633f, -5.67254f), Vector3(-29.905f, -21.9188f, -2.52868f)); 
  data[6].mAabb = Aabb(Vector3(-31.6834f, -28.6156f, -9.66202f), Vector3(-26.7908f, -26.8244f, -2.18667f)); 
  data[7].mAabb = Aabb(Vector3(5.95474f, 9.67577f, 0.993669f), Vector3(9.28346f, 11.5402f, 2.05702f)); 
  data[8].mAabb = Aabb(Vector3(1.41982f, 24.3742f, -40.8023f), Vector3(6.00217f, 28.3558f, -40.0092f)); 
  data[9].mAabb = Aabb(Vector3(25.9503f, -16.0582f, -9.11889f), Vector3(27.9542f, -14.8012f, -2.4762f)); 
  data[10].mAabb = Aabb(Vector3(-7.53082f, 12.7594f, -24.4623f), Vector3(-3.49144f, 17.8457f, -19.6075f)); 
  data[11].mAabb = Aabb(Vector3(15.4188f, 28.1573f, -15.8586f), Vector3(19.5999f, 32.9106f, -13.9399f)); 
  data[12].mAabb = Aabb(Vector3(-43.2748f, -10.7814f, -0.471106f), Vector3(-39.3221f, -7.00315f, 1.95397f)); 
  data[13].mAabb = Aabb(Vector3(0.461301f, 15.5879f, -28.8212f), Vector3(1.71206f, 22.1195f, -24.5149f)); 
  data[14].mAabb = Aabb(Vector3(-42.039f, -17.7939f, -18.0527f), Vector3(-41.138f, -14.751f, -15.4736f)); 
  data[15].mAabb = Aabb(Vector3(-8.67296f, -38.1939f, 16.7532f), Vector3(-4.59855f, -34.0274f, 19.5598f)); 
  data[16].mAabb = Aabb(Vector3(-3.29471f, 9.50628f, 3.4216f), Vector3(3.91275f, 12.3769f, 8.0965f)); 
  data[17].mAabb = Aabb(Vector3(28.863f, 8.72077f, -14.0584f), Vector3(30.2845f, 9.48863f, -6.42297f)); 
  data[18].mAabb = Aabb(Vector3(44.1473f, -6.41885f, 9.53889f), Vector3(45.245f, -1.99654f, 13.4428f)); 
  data[19].mAabb = Aabb(Vector3(-32.5492f, 14.3306f, 7.92145f), Vector3(-29.5786f, 18.5576f, 10.0571f)); 
  data[20].mAabb = Aabb(Vector3(19.3271f, 32.4578f, -10.5498f), Vector3(21.9632f, 34.1414f, -6.56792f)); 
  data[21].mAabb = Aabb(Vector3(-27.8703f, 32.2271f, -1.34698f), Vector3(-20.8315f, 33.7074f, 6.29166f)); 
  data[22].mAabb = Aabb(Vector3(-22.3455f, 26.0221f, -3.28064f), Vector3(-18.857f, 27.2387f, 3.06075f)); 
  data[23].mAabb = Aabb(Vector3(-18.8407f, -29.4661f, -18.5976f), Vector3(-15.8413f, -25.9628f, -14.0702f)); 
  data[24].mAabb = Aabb(Vector3(-20.9f, 15.5404f, 11.5468f), Vector3(-14.1146f, 15.9648f, 13.7362f)); 
  data[25].mAabb = Aabb(Vector3(-19.3609f, 28.0907f, 10.8935f), Vector3(-18.8073f, 30.2388f, 14.6731f)); 
  data[26].mAabb = Aabb(Vector3(5.0999f, -37.7344f, 14.589f), Vector3(9.04233f, -34.7956f, 19.7816f)); 
  data[27].mAabb = Aabb(Vector3(9.64559f, 3.09329f, -0.260674f), Vector3(15.4619f, 6.02933f, 4.37595f)); 
  data[28].mAabb = Aabb(Vector3(0.39855f, -15.8883f, -4.76729f), Vector3(8.3009f, -14.3422f, 1.98486f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-0.760911f, -17.7239f, -3.35734f));
  ray.mDirection = Vector3(Vector3(0.385464f, 0.917472f, -0.0983001f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(64); 
  data[0].mAabb = Aabb(Vector3(30.0269f, -8.0517f, -30.3728f), Vector3(32.878f, -0.314021f, -26.3974f)); 
  data[1].mAabb = Aabb(Vector3(-35.3667f, -4.65543f, -6.38964f), Vector3(-29.9159f, 2.97764f, -4.61516f)); 
  data[2].mAabb = Aabb(Vector3(-4.22929f, -11.7463f, 9.72123f), Vector3(3.15491f, -5.80962f, 17.2977f)); 
  data[3].mAabb = Aabb(Vector3(20.3365f, -20.6729f, 15.6597f), Vector3(23.6274f, -18.7553f, 18.8968f)); 
  data[4].mAabb = Aabb(Vector3(-0.953966f, -27.3318f, 17.7236f), Vector3(3.85546f, -21.699f, 25.2245f)); 
  data[5].mAabb = Aabb(Vector3(30.0118f, 16.8756f, -24.8032f), Vector3(33.588f, 22.9148f, -17.2879f)); 
  data[6].mAabb = Aabb(Vector3(11.5131f, 34.8557f, -3.79802f), Vector3(16.7713f, 42.3085f, 0.00919247f)); 
  data[7].mAabb = Aabb(Vector3(3.66545f, 24.3975f, -40.4134f), Vector3(5.18271f, 31.7411f, -39.7864f)); 
  data[8].mAabb = Aabb(Vector3(-13.632f, 24.6028f, 17.3566f), Vector3(-7.81413f, 31.6418f, 24.4149f)); 
  data[9].mAabb = Aabb(Vector3(-37.7366f, -11.6122f, -16.6565f), Vector3(-37.085f, -10.1019f, -13.3913f)); 
  data[10].mAabb = Aabb(Vector3(-0.736518f, 6.93926f, -19.0101f), Vector3(6.29351f, 12.3434f, -16.5106f)); 
  data[11].mAabb = Aabb(Vector3(-16.5438f, 26.418f, -24.8792f), Vector3(-13.233f, 31.4116f, -18.9595f)); 
  data[12].mAabb = Aabb(Vector3(7.34373f, -7.68543f, 17.8167f), Vector3(13.1166f, -1.14828f, 23.5115f)); 
  data[13].mAabb = Aabb(Vector3(4.49788f, -13.8457f, 20.1359f), Vector3(10.8288f, -7.70216f, 24.4974f)); 
  data[14].mAabb = Aabb(Vector3(11.4093f, 5.28929f, -23.5442f), Vector3(19.3967f, 6.36076f, -15.5708f)); 
  data[15].mAabb = Aabb(Vector3(21.5365f, -47.4724f, 6.90662f), Vector3(23.3001f, -39.7489f, 11.0548f)); 
  data[16].mAabb = Aabb(Vector3(-6.53667f, 46.0876f, -3.78404f), Vector3(-4.42147f, 53.1128f, -2.32825f)); 
  data[17].mAabb = Aabb(Vector3(23.6705f, 19.6709f, 15.6659f), Vector3(25.0231f, 25.4002f, 19.0746f)); 
  data[18].mAabb = Aabb(Vector3(15.4216f, 28.5797f, -9.20519f), Vector3(20.5258f, 35.4093f, -5.58516f)); 
  data[19].mAabb = Aabb(Vector3(25.9544f, 15.2674f, -39.7864f), Vector3(26.8928f, 19.2954f, -34.8058f)); 
  data[20].mAabb = Aabb(Vector3(-2.12814f, -8.71499f, 17.1802f), Vector3(5.72249f, -5.05832f, 22.3485f)); 
  data[21].mAabb = Aabb(Vector3(-35.3123f, 12.2703f, 18.7333f), Vector3(-32.9629f, 13.1986f, 25.0828f)); 
  data[22].mAabb = Aabb(Vector3(28.2816f, -14.2844f, 15.1861f), Vector3(36.2167f, -8.42519f, 19.5462f)); 
  data[23].mAabb = Aabb(Vector3(21.323f, -28.5172f, -16.1237f), Vector3(22.6742f, -26.9255f, -12.8595f)); 
  data[24].mAabb = Aabb(Vector3(27.9682f, 1.66609f, 12.955f), Vector3(30.0871f, 4.59772f, 18.9281f)); 
  data[25].mAabb = Aabb(Vector3(-21.6731f, 32.8745f, 8.65023f), Vector3(-15.6688f, 40.4748f, 9.73445f)); 
  data[26].mAabb = Aabb(Vector3(-2.6158f, -2.30995f, -0.688659f), Vector3(2.24511f, 5.50287f, 2.96245f)); 
  data[27].mAabb = Aabb(Vector3(-32.6741f, 1.20078f, -0.368599f), Vector3(-31.0698f, 8.39247f, 0.952901f)); 
  data[28].mAabb = Aabb(Vector3(-8.15045f, 34.2639f, -33.2819f), Vector3(-4.83426f, 35.1971f, -27.0737f)); 
  data[29].mAabb = Aabb(Vector3(-0.0259545f, -23.8414f, 13.1161f), Vector3(2.53295f, -22.7651f, 16.1593f)); 
  data[30].mAabb = Aabb(Vector3(-16.3775f, -9.72068f, -19.365f), Vector3(-10.0929f, -6.89319f, -14.3202f)); 
  data[31].mAabb = Aabb(Vector3(11.2755f, -26.9695f, -17.5294f), Vector3(12.244f, -23.2229f, -15.7352f)); 
  data[32].mAabb = Aabb(Vector3(21.7079f, 31.1947f, -19.6766f), Vector3(28.0714f, 35.5136f, -15.6345f)); 
  data[33].mAabb = Aabb(Vector3(9.07212f, -6.68696f, -47.5397f), Vector3(14.5437f, -4.697f, -44.6352f)); 
  data[34].mAabb = Aabb(Vector3(-20.957f, -34.769f, -22.9917f), Vector3(-15.7816f, -30.8788f, -21.4957f)); 
  data[35].mAabb = Aabb(Vector3(5.49147f, -35.9407f, 22.3182f), Vector3(8.40826f, -35.4185f, 23.7583f)); 
  data[36].mAabb = Aabb(Vector3(-45.5641f, 5.69204f, -9.33172f), Vector3(-41.037f, 8.54946f, -1.87678f)); 
  data[37].mAabb = Aabb(Vector3(-16.8853f, -6.58726f, -41.7391f), Vector3(-10.8598f, 0.663344f, -35.2729f)); 
  data[38].mAabb = Aabb(Vector3(31.9904f, -18.3244f, 23.8528f), Vector3(38.956f, -15.5739f, 25.0723f)); 
  data[39].mAabb = Aabb(Vector3(-37.7267f, 19.4067f, 11.9144f), Vector3(-33.3206f, 23.8858f, 12.9916f)); 
  data[40].mAabb = Aabb(Vector3(18.3229f, 4.08218f, 38.4671f), Vector3(22.9139f, 11.603f, 42.0398f)); 
  data[41].mAabb = Aabb(Vector3(-3.24529f, -9.85027f, -37.8502f), Vector3(1.63975f, -8.07927f, -30.7847f)); 
  data[42].mAabb = Aabb(Vector3(-33.1171f, 9.67808f, 26.6375f), Vector3(-25.9229f, 16.2635f, 27.9354f)); 
  data[43].mAabb = Aabb(Vector3(-9.43947f, -25.4703f, -0.567419f), Vector3(-8.12841f, -21.4938f, 0.712796f)); 
  data[44].mAabb = Aabb(Vector3(19.4227f, -9.52864f, -27.0457f), Vector3(24.3968f, -7.4088f, -22.0957f)); 
  data[45].mAabb = Aabb(Vector3(-7.05782f, -43.2915f, 2.12372f), Vector3(-0.452483f, -37.947f, 6.67383f)); 
  data[46].mAabb = Aabb(Vector3(-46.1457f, -0.482922f, -2.57316f), Vector3(-45.6422f, 2.92999f, 5.26726f)); 
  data[47].mAabb = Aabb(Vector3(16.4323f, -13.1153f, 27.4591f), Vector3(18.8391f, -10.2922f, 34.0833f)); 
  data[48].mAabb = Aabb(Vector3(-15.3038f, 21.4249f, -11.224f), Vector3(-10.1008f, 27.5209f, -5.62205f)); 
  data[49].mAabb = Aabb(Vector3(42.5337f, -10.4532f, 0.349668f), Vector3(50.0228f, -7.78276f, 2.74691f)); 
  data[50].mAabb = Aabb(Vector3(33.3557f, -25.3164f, -3.63225f), Vector3(39.1404f, -24.6589f, 0.257299f)); 
  data[51].mAabb = Aabb(Vector3(-15.9926f, -23.3856f, -6.22508f), Vector3(-9.69228f, -17.5069f, -5.7439f)); 
  data[52].mAabb = Aabb(Vector3(13.1227f, 9.75099f, -45.322f), Vector3(14.254f, 15.0104f, -38.9342f)); 
  data[53].mAabb = Aabb(Vector3(18.3355f, -13.1654f, -28.6876f), Vector3(25.5302f, -5.83341f, -22.9022f)); 
  data[54].mAabb = Aabb(Vector3(4.71901f, -29.3937f, 5.91213f), Vector3(5.79001f, -21.7314f, 7.79145f)); 
  data[55].mAabb = Aabb(Vector3(-36.4232f, -5.196f, -4.65813f), Vector3(-34.3077f, -3.39044f, -0.291946f)); 
  data[56].mAabb = Aabb(Vector3(16.334f, 10.8181f, -35.4526f), Vector3(20.2438f, 12.0956f, -32.1088f)); 
  data[57].mAabb = Aabb(Vector3(-17.5417f, -18.5252f, -22.816f), Vector3(-11.4478f, -17.2038f, -17.4028f)); 
  data[58].mAabb = Aabb(Vector3(14.6306f, 3.70119f, 41.0464f), Vector3(18.4163f, 11.4551f, 46.89f)); 
  data[59].mAabb = Aabb(Vector3(-2.30477f, 13.5121f, -42.3368f), Vector3(-0.780555f, 17.2829f, -40.9072f)); 
  data[60].mAabb = Aabb(Vector3(-10.7478f, -18.5968f, -20.0728f), Vector3(-4.04084f, -13.4819f, -12.7074f)); 
  data[61].mAabb = Aabb(Vector3(-18.9754f, -38.0588f, -28.7173f), Vector3(-15.3958f, -37.1411f, -20.8463f)); 
  data[62].mAabb = Aabb(Vector3(-28.4862f, -24.8155f, 1.85641f), Vector3(-20.9505f, -21.3588f, 6.82935f)); 
  data[63].mAabb = Aabb(Vector3(9.07937f, -43.2145f, 7.27612f), Vector3(12.4092f, -39.5179f, 12.5821f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-1.201f, -7.61519f, 23.244f));
  ray.mDirection = Vector3(Vector3(-0.2994f, -0.464712f, -0.833308f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(48); 
  data[0].mAabb = Aabb(Vector3(-50.7323f, 4.30563f, 1.16645f), Vector3(-45.446f, 7.38872f, 3.72744f)); 
  data[1].mAabb = Aabb(Vector3(-7.77731f, -17.1123f, 27.5942f), Vector3(-2.60884f, -14.0898f, 34.7184f)); 
  data[2].mAabb = Aabb(Vector3(17.1258f, -20.9307f, 27.8113f), Vector3(22.2815f, -17.6781f, 34.7309f)); 
  data[3].mAabb = Aabb(Vector3(4.5464f, -18.3768f, -1.16307f), Vector3(7.04964f, -16.36f, 5.15072f)); 
  data[4].mAabb = Aabb(Vector3(-10.7145f, -40.8147f, -16.3468f), Vector3(-3.10513f, -35.8777f, -13.4277f)); 
  data[5].mAabb = Aabb(Vector3(17.4794f, -10.3937f, -4.24907f), Vector3(23.7895f, -5.05959f, -0.041069f)); 
  data[6].mAabb = Aabb(Vector3(40.4301f, -17.4851f, -15.6103f), Vector3(42.0742f, -10.8549f, -11.9865f)); 
  data[7].mAabb = Aabb(Vector3(-21.8952f, -20.3377f, 24.8727f), Vector3(-16.1283f, -18.6475f, 27.3748f)); 
  data[8].mAabb = Aabb(Vector3(0.946359f, 36.3077f, -17.1212f), Vector3(5.23577f, 43.8826f, -16.2107f)); 
  data[9].mAabb = Aabb(Vector3(-3.76464f, -2.83262f, -49.1122f), Vector3(-0.609181f, 3.03237f, -46.9235f)); 
  data[10].mAabb = Aabb(Vector3(31.3844f, -9.84739f, -21.2691f), Vector3(31.959f, -9.13635f, -20.3941f)); 
  data[11].mAabb = Aabb(Vector3(-15.1545f, -21.8583f, 29.3209f), Vector3(-10.0783f, -17.5552f, 35.5162f)); 
  data[12].mAabb = Aabb(Vector3(25.8314f, -26.2064f, 15.0879f), Vector3(27.2023f, -19.0045f, 17.9917f)); 
  data[13].mAabb = Aabb(Vector3(7.01235f, 24.3574f, -17.0516f), Vector3(10.1801f, 24.9615f, -13.4817f)); 
  data[14].mAabb = Aabb(Vector3(-21.767f, -10.7355f, 31.7742f), Vector3(-16.6484f, -6.63835f, 34.7028f)); 
  data[15].mAabb = Aabb(Vector3(-6.96028f, -13.7254f, -48.3349f), Vector3(-1.11269f, -6.96858f, -45.9033f)); 
  data[16].mAabb = Aabb(Vector3(-18.9862f, 31.3069f, -12.7299f), Vector3(-16.1311f, 34.6108f, -6.35699f)); 
  data[17].mAabb = Aabb(Vector3(-33.0779f, 28.4564f, 2.1453f), Vector3(-29.0984f, 33.0352f, 10.0367f)); 
  data[18].mAabb = Aabb(Vector3(22.5501f, -28.5262f, 2.50658f), Vector3(25.1039f, -24.3085f, 4.60972f)); 
  data[19].mAabb = Aabb(Vector3(7.97052f, -41.5143f, 3.13244f), Vector3(15.9582f, -37.0808f, 7.69415f)); 
  data[20].mAabb = Aabb(Vector3(-27.6369f, -2.47674f, 31.9071f), Vector3(-20.6491f, -0.294735f, 36.4066f)); 
  data[21].mAabb = Aabb(Vector3(-30.1089f, 1.66776f, -3.11106f), Vector3(-28.7872f, 6.942f, 3.95584f)); 
  data[22].mAabb = Aabb(Vector3(24.1444f, -22.7579f, -2.46668f), Vector3(28.8738f, -19.6184f, -1.51095f)); 
  data[23].mAabb = Aabb(Vector3(-14.2058f, -7.57875f, -6.43289f), Vector3(-13.2109f, -1.68918f, -3.87167f)); 
  data[24].mAabb = Aabb(Vector3(-16.1206f, 37.7737f, -2.06939f), Vector3(-8.82082f, 44.0068f, 1.05429f)); 
  data[25].mAabb = Aabb(Vector3(13.7797f, -20.8436f, 23.7593f), Vector3(15.4781f, -20.2398f, 24.4344f)); 
  data[26].mAabb = Aabb(Vector3(-13.5875f, 19.6556f, -7.52837f), Vector3(-11.5078f, 23.1794f, -6.31797f)); 
  data[27].mAabb = Aabb(Vector3(-30.3094f, 30.9307f, -17.6576f), Vector3(-28.4982f, 33.8319f, -10.3616f)); 
  data[28].mAabb = Aabb(Vector3(5.0859f, 38.6973f, -19.7085f), Vector3(11.0808f, 42.8821f, -15.2618f)); 
  data[29].mAabb = Aabb(Vector3(-25.9773f, -35.766f, -17.3729f), Vector3(-24.1634f, -28.3348f, -16.5751f)); 
  data[30].mAabb = Aabb(Vector3(-8.97558f, 8.25826f, 29.9249f), Vector3(-2.98743f, 11.3933f, 34.9579f)); 
  data[31].mAabb = Aabb(Vector3(40.0624f, -12.7895f, -7.77577f), Vector3(46.285f, -10.5284f, -7.04038f)); 
  data[32].mAabb = Aabb(Vector3(-12.664f, 16.3961f, 0.695417f), Vector3(-10.9744f, 24.2732f, 3.77642f)); 
  data[33].mAabb = Aabb(Vector3(13.3684f, -21.8078f, 28.066f), Vector3(16.5185f, -19.1699f, 31.6747f)); 
  data[34].mAabb = Aabb(Vector3(-32.1633f, 9.97758f, -7.11783f), Vector3(-27.4622f, 15.4745f, -1.2591f)); 
  data[35].mAabb = Aabb(Vector3(36.3872f, -20.7183f, -25.7713f), Vector3(42.1998f, -15.397f, -20.8542f)); 
  data[36].mAabb = Aabb(Vector3(1.3748f, 25.8495f, -35.4618f), Vector3(6.07637f, 32.2445f, -33.4081f)); 
  data[37].mAabb = Aabb(Vector3(-25.357f, 9.51397f, 15.2772f), Vector3(-24.4657f, 11.4606f, 18.9893f)); 
  data[38].mAabb = Aabb(Vector3(-33.5551f, -26.462f, -20.5619f), Vector3(-25.9624f, -25.3601f, -13.7933f)); 
  data[39].mAabb = Aabb(Vector3(-27.5506f, -2.28252f, -15.4651f), Vector3(-25.4889f, 1.40779f, -11.8047f)); 
  data[40].mAabb = Aabb(Vector3(10.1452f, 12.9977f, 27.5618f), Vector3(14.0347f, 17.7501f, 30.7184f)); 
  data[41].mAabb = Aabb(Vector3(-12.0663f, -14.9568f, 37.2241f), Vector3(-11.6635f, -12.0683f, 42.8576f)); 
  data[42].mAabb = Aabb(Vector3(-29.0485f, 0.856139f, -38.8919f), Vector3(-26.0704f, 3.62449f, -33.2762f)); 
  data[43].mAabb = Aabb(Vector3(-6.95974f, 6.75193f, 11.7467f), Vector3(-4.55878f, 13.0305f, 17.5778f)); 
  data[44].mAabb = Aabb(Vector3(4.64957f, 29.3242f, 16.2044f), Vector3(11.2911f, 32.0482f, 18.2521f)); 
  data[45].mAabb = Aabb(Vector3(-20.6013f, 28.4613f, -6.83271f), Vector3(-20.1449f, 33.1868f, -4.77573f)); 
  data[46].mAabb = Aabb(Vector3(-26.3388f, -23.2029f, 3.30213f), Vector3(-22.0608f, -17.2548f, 11.0039f)); 
  data[47].mAabb = Aabb(Vector3(-44.5272f, -3.23024f, 9.76282f), Vector3(-38.7156f, 3.26215f, 16.7175f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-20.2427f, -2.87202f, -8.23097f));
  ray.mDirection = Vector3(Vector3(0.953102f, -0.11765f, 0.278848f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(-30.8352f, -6.22083f, -19.6599f), Vector3(-24.193f, -5.27484f, -14.757f)); 
  data[1].mAabb = Aabb(Vector3(24.1334f, 7.26809f, -25.1156f), Vector3(29.1989f, 9.39767f, -18.06f)); 
  data[2].mAabb = Aabb(Vector3(8.31238f, -26.7671f, 8.86822f), Vector3(13.8594f, -20.9292f, 16.5273f)); 
  data[3].mAabb = Aabb(Vector3(11.4121f, 28.0792f, 24.243f), Vector3(16.1578f, 34.6911f, 24.9044f)); 
  data[4].mAabb = Aabb(Vector3(14.1291f, -48.5565f, -6.45878f), Vector3(14.9343f, -45.9271f, -4.63699f)); 
  data[5].mAabb = Aabb(Vector3(33.8493f, 28.0608f, -1.24781f), Vector3(35.8124f, 30.4543f, 5.87105f)); 
  data[6].mAabb = Aabb(Vector3(-31.7888f, -7.73305f, -2.84975f), Vector3(-26.8168f, -0.478736f, 0.150542f)); 
  data[7].mAabb = Aabb(Vector3(-22.9033f, -14.9694f, 0.736096f), Vector3(-21.8485f, -7.43932f, 5.20573f)); 
  data[8].mAabb = Aabb(Vector3(-4.02267f, -34.5099f, 3.03169f), Vector3(-0.975069f, -32.7572f, 9.97265f)); 
  data[9].mAabb = Aabb(Vector3(-1.60664f, 41.0443f, 15.9477f), Vector3(3.56276f, 47.4552f, 16.9839f)); 
  data[10].mAabb = Aabb(Vector3(-7.37022f, 16.0517f, 36.8931f), Vector3(-4.54482f, 22.5387f, 44.4289f)); 
  data[11].mAabb = Aabb(Vector3(14.5132f, -17.2371f, -8.26429f), Vector3(18.8493f, -13.6316f, -6.12728f)); 
  data[12].mAabb = Aabb(Vector3(-35.7992f, -9.3725f, -8.13103f), Vector3(-33.1392f, -4.49187f, -6.88073f)); 
  data[13].mAabb = Aabb(Vector3(-12.2681f, -41.5603f, 22.8829f), Vector3(-8.87858f, -37.819f, 23.9822f)); 
  data[14].mAabb = Aabb(Vector3(-13.2199f, 33.8359f, 6.3034f), Vector3(-8.54571f, 39.2867f, 11.5825f)); 
  data[15].mAabb = Aabb(Vector3(11.1027f, 2.3428f, 45.0964f), Vector3(17.9625f, 8.82127f, 46.0849f)); 
  data[16].mAabb = Aabb(Vector3(30.6043f, -24.4512f, 2.9596f), Vector3(38.2174f, -19.1591f, 9.50441f)); 
  data[17].mAabb = Aabb(Vector3(-20.7835f, -36.2919f, -6.95917f), Vector3(-17.4369f, -33.2499f, -5.70238f)); 
  data[18].mAabb = Aabb(Vector3(-13.8325f, -48.4396f, -3.78814f), Vector3(-12.8693f, -45.466f, 1.86601f)); 
  data[19].mAabb = Aabb(Vector3(-21.3916f, -32.9748f, 5.62809f), Vector3(-18.1084f, -30.3971f, 6.96049f)); 
  data[20].mAabb = Aabb(Vector3(-30.2279f, 15.6597f, 24.9603f), Vector3(-29.2849f, 23.0754f, 29.4817f)); 
  data[21].mAabb = Aabb(Vector3(-12.6562f, -13.0026f, -15.1092f), Vector3(-7.24792f, -6.61455f, -14.1546f)); 
  data[22].mAabb = Aabb(Vector3(16.0671f, 7.49514f, -7.94402f), Vector3(21.6482f, 8.19968f, -4.97319f)); 
  data[23].mAabb = Aabb(Vector3(8.47767f, -29.4907f, 9.94693f), Vector3(15.1306f, -25.574f, 13.621f)); 
  data[24].mAabb = Aabb(Vector3(-3.05049f, -28.5812f, -18.4128f), Vector3(-2.35709f, -21.9701f, -13.3786f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(15.77f, -15.883f, 4.0046f));
  ray.mDirection = Vector3(Vector3(-0.485672f, -0.731148f, 0.47911f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(24.4825f, 20.037f, -13.2879f), Vector3(27.7699f, 22.7049f, -9.76946f)); 
  data[1].mAabb = Aabb(Vector3(-14.8874f, -5.27559f, -31.7419f), Vector3(-8.69146f, 1.32372f, -30.1683f)); 
  data[2].mAabb = Aabb(Vector3(6.56735f, 12.598f, 28.0629f), Vector3(10.3523f, 13.57f, 30.8526f)); 
  data[3].mAabb = Aabb(Vector3(28.2455f, 21.1974f, 0.928914f), Vector3(28.7529f, 25.8662f, 8.14635f)); 
  data[4].mAabb = Aabb(Vector3(-39.8974f, -11.4215f, -21.4618f), Vector3(-39.0843f, -4.69922f, -13.6177f)); 
  data[5].mAabb = Aabb(Vector3(-19.4385f, 16.8298f, 29.3137f), Vector3(-18.6885f, 20.0618f, 33.5732f)); 
  data[6].mAabb = Aabb(Vector3(5.69816f, -32.4638f, -24.0826f), Vector3(8.07197f, -26.4592f, -20.5313f)); 
  data[7].mAabb = Aabb(Vector3(8.42294f, 24.5438f, 29.5738f), Vector3(9.93185f, 26.714f, 37.5156f)); 
  data[8].mAabb = Aabb(Vector3(16.5598f, 17.859f, -4.82954f), Vector3(17.3942f, 21.2239f, -0.101759f)); 
  data[9].mAabb = Aabb(Vector3(-2.36209f, -5.91384f, -25.4617f), Vector3(-0.759018f, -2.46405f, -19.7667f)); 
  data[10].mAabb = Aabb(Vector3(-12.4021f, 29.457f, 34.3552f), Vector3(-10.5128f, 31.8369f, 37.6217f)); 
  data[11].mAabb = Aabb(Vector3(-12.5223f, -39.4122f, -13.7938f), Vector3(-8.06079f, -37.971f, -9.55616f)); 
  data[12].mAabb = Aabb(Vector3(14.7343f, -28.6525f, 24.5212f), Vector3(20.0545f, -27.03f, 30.4145f)); 
  data[13].mAabb = Aabb(Vector3(-9.24119f, 34.7215f, -32.1976f), Vector3(-7.23129f, 42.23f, -27.9457f)); 
  data[14].mAabb = Aabb(Vector3(-7.10042f, -14.3399f, 41.1515f), Vector3(-5.17819f, -11.4059f, 45.755f)); 
  data[15].mAabb = Aabb(Vector3(-18.3306f, 41.6304f, 13.0082f), Vector3(-17.6026f, 42.9192f, 18.0067f)); 
  data[16].mAabb = Aabb(Vector3(-34.9477f, 20.5191f, -1.60757f), Vector3(-33.3882f, 22.8293f, 5.23922f)); 
  data[17].mAabb = Aabb(Vector3(33.9674f, -14.9235f, -21.2593f), Vector3(41.9078f, -13.6542f, -20.6253f)); 
  data[18].mAabb = Aabb(Vector3(22.7535f, -10.2455f, 0.533379f), Vector3(27.4597f, -4.3218f, 6.05231f)); 
  data[19].mAabb = Aabb(Vector3(2.61934f, -24.3507f, -25.5281f), Vector3(9.22189f, -21.6737f, -23.3325f)); 
  data[20].mAabb = Aabb(Vector3(8.63274f, 5.52628f, 13.8678f), Vector3(9.4776f, 9.10897f, 14.4928f)); 
  data[21].mAabb = Aabb(Vector3(-20.9214f, -19.9715f, -6.8058f), Vector3(-17.0733f, -14.2811f, -5.39685f)); 
  data[22].mAabb = Aabb(Vector3(-4.26148f, -14.5675f, 11.6636f), Vector3(0.614516f, -11.6152f, 17.5416f)); 
  data[23].mAabb = Aabb(Vector3(-47.7664f, -4.92438f, 15.1941f), Vector3(-41.0117f, -1.33357f, 16.683f)); 
  data[24].mAabb = Aabb(Vector3(18.8109f, -9.31725f, 5.43365f), Vector3(20.9164f, -5.68052f, 13.1231f)); 
  data[25].mAabb = Aabb(Vector3(6.22581f, 21.3733f, -4.39085f), Vector3(9.21913f, 23.2352f, -3.69954f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(15.501f, -6.84826f, 9.03749f));
  ray.mDirection = Vector3(Vector3(0.91695f, 0.0604844f, -0.394391f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(55); 
  data[0].mAabb = Aabb(Vector3(2.82942f, 45.4548f, -5.96646f), Vector3(7.87171f, 46.1828f, -4.28035f)); 
  data[1].mAabb = Aabb(Vector3(0.648309f, -0.588358f, 23.4012f), Vector3(5.82119f, 3.48071f, 25.2628f)); 
  data[2].mAabb = Aabb(Vector3(-27.4165f, -33.0908f, -10.8321f), Vector3(-21.0724f, -31.2811f, -10.1479f)); 
  data[3].mAabb = Aabb(Vector3(39.4769f, -2.29504f, -27.1608f), Vector3(41.047f, 4.34138f, -21.8297f)); 
  data[4].mAabb = Aabb(Vector3(10.8604f, 15.9619f, 11.9296f), Vector3(15.7784f, 17.6201f, 19.0976f)); 
  data[5].mAabb = Aabb(Vector3(-43.574f, 15.6616f, -9.90574f), Vector3(-37.9183f, 22.4629f, -6.24906f)); 
  data[6].mAabb = Aabb(Vector3(-14.3497f, -1.35652f, -9.66296f), Vector3(-11.1309f, 4.22689f, -8.27258f)); 
  data[7].mAabb = Aabb(Vector3(10.2948f, 37.7879f, 6.18212f), Vector3(18.2104f, 45.0035f, 8.13914f)); 
  data[8].mAabb = Aabb(Vector3(18.4053f, 21.2078f, -29.6102f), Vector3(18.8937f, 25.5728f, -25.9215f)); 
  data[9].mAabb = Aabb(Vector3(-1.28306f, -2.14762f, 6.40652f), Vector3(4.12292f, 3.92889f, 9.74196f)); 
  data[10].mAabb = Aabb(Vector3(-25.167f, -40.6027f, -17.0466f), Vector3(-22.161f, -37.2436f, -11.8252f)); 
  data[11].mAabb = Aabb(Vector3(-40.7194f, -4.71061f, 23.4204f), Vector3(-36.575f, -0.499596f, 29.0996f)); 
  data[12].mAabb = Aabb(Vector3(-21.2579f, -23.1771f, 8.40243f), Vector3(-16.9517f, -18.2905f, 9.24915f)); 
  data[13].mAabb = Aabb(Vector3(-18.0194f, 39.7933f, 17.1768f), Vector3(-14.7951f, 45.9693f, 18.3837f)); 
  data[14].mAabb = Aabb(Vector3(34.8221f, -22.8706f, -20.125f), Vector3(40.2554f, -15.5462f, -17.8212f)); 
  data[15].mAabb = Aabb(Vector3(-37.0306f, -19.1242f, -13.2414f), Vector3(-34.8094f, -14.4343f, -10.0173f)); 
  data[16].mAabb = Aabb(Vector3(-8.10744f, 26.3184f, -13.8541f), Vector3(-3.41607f, 30.8246f, -8.9976f)); 
  data[17].mAabb = Aabb(Vector3(-1.07113f, -27.913f, 6.95814f), Vector3(0.180791f, -25.6051f, 13.7057f)); 
  data[18].mAabb = Aabb(Vector3(15.07f, -44.9963f, 13.9845f), Vector3(19.2209f, -41.7472f, 19.9513f)); 
  data[19].mAabb = Aabb(Vector3(27.4754f, -13.6443f, 0.83169f), Vector3(32.3231f, -7.19161f, 7.09724f)); 
  data[20].mAabb = Aabb(Vector3(-7.84313f, -2.72776f, -19.7579f), Vector3(0.135998f, 1.88799f, -16.201f)); 
  data[21].mAabb = Aabb(Vector3(-10.0587f, -43.3326f, 11.1949f), Vector3(-3.45428f, -37.2218f, 18.8894f)); 
  data[22].mAabb = Aabb(Vector3(-36.2632f, 5.51302f, 5.26645f), Vector3(-29.6115f, 10.2385f, 9.29029f)); 
  data[23].mAabb = Aabb(Vector3(1.03316f, 33.9161f, 9.85203f), Vector3(4.80581f, 37.0281f, 15.6444f)); 
  data[24].mAabb = Aabb(Vector3(-25.8679f, -24.6164f, -13.8966f), Vector3(-20.1553f, -18.2871f, -8.32737f)); 
  data[25].mAabb = Aabb(Vector3(18.7577f, 10.6315f, -18.8664f), Vector3(22.31f, 18.134f, -17.9723f)); 
  data[26].mAabb = Aabb(Vector3(26.6522f, 5.36003f, -16.8574f), Vector3(28.5932f, 11.116f, -14.6517f)); 
  data[27].mAabb = Aabb(Vector3(0.884448f, -43.7547f, -8.141f), Vector3(2.73292f, -42.2363f, -4.71023f)); 
  data[28].mAabb = Aabb(Vector3(-37.5718f, 6.35846f, 17.3597f), Vector3(-33.9817f, 13.6063f, 21.7643f)); 
  data[29].mAabb = Aabb(Vector3(6.46138f, -36.8779f, -31.9496f), Vector3(9.11816f, -34.7935f, -30.6551f)); 
  data[30].mAabb = Aabb(Vector3(4.30517f, -33.49f, -4.67388f), Vector3(9.4639f, -26.9967f, 3.06982f)); 
  data[31].mAabb = Aabb(Vector3(9.34962f, -16.29f, 1.16417f), Vector3(14.9892f, -10.3538f, 6.87747f)); 
  data[32].mAabb = Aabb(Vector3(18.8902f, -35.5128f, 17.5456f), Vector3(24.6499f, -32.5397f, 21.8677f)); 
  data[33].mAabb = Aabb(Vector3(26.6111f, -24.6801f, -5.75423f), Vector3(33.8617f, -24.189f, 0.538918f)); 
  data[34].mAabb = Aabb(Vector3(-30.0338f, -35.1266f, 5.66897f), Vector3(-25.6867f, -29.3973f, 11.9702f)); 
  data[35].mAabb = Aabb(Vector3(27.2039f, -17.8832f, 12.1435f), Vector3(30.9835f, -10.9121f, 16.243f)); 
  data[36].mAabb = Aabb(Vector3(-37.059f, -1.77341f, -25.4405f), Vector3(-29.3948f, -0.801907f, -23.9044f)); 
  data[37].mAabb = Aabb(Vector3(-24.4887f, 0.326991f, 5.51444f), Vector3(-20.8181f, 4.82074f, 12.1439f)); 
  data[38].mAabb = Aabb(Vector3(-39.9085f, -22.1919f, -19.873f), Vector3(-32.9348f, -16.4582f, -19.2148f)); 
  data[39].mAabb = Aabb(Vector3(22.0785f, -34.0598f, 17.2352f), Vector3(29.1593f, -28.4136f, 18.7817f)); 
  data[40].mAabb = Aabb(Vector3(-5.94333f, -17.088f, 35.7005f), Vector3(-0.906603f, -15.8992f, 40.0753f)); 
  data[41].mAabb = Aabb(Vector3(5.30381f, -3.80762f, -35.9802f), Vector3(10.9464f, -0.758397f, -30.1472f)); 
  data[42].mAabb = Aabb(Vector3(-46.8406f, -11.351f, 8.74291f), Vector3(-43.0418f, -4.70759f, 13.4862f)); 
  data[43].mAabb = Aabb(Vector3(-18.9437f, 17.4042f, -42.2254f), Vector3(-12.9539f, 21.6329f, -40.1963f)); 
  data[44].mAabb = Aabb(Vector3(-41.6761f, -1.52708f, -28.3099f), Vector3(-37.8295f, 2.6858f, -27.1225f)); 
  data[45].mAabb = Aabb(Vector3(32.7327f, 13.6622f, -21.2509f), Vector3(33.5442f, 17.1661f, -18.6343f)); 
  data[46].mAabb = Aabb(Vector3(17.4316f, -17.7531f, -16.1095f), Vector3(19.5303f, -12.1516f, -15.4941f)); 
  data[47].mAabb = Aabb(Vector3(-10.4769f, -33.7212f, 21.8308f), Vector3(-5.48052f, -31.9147f, 22.8415f)); 
  data[48].mAabb = Aabb(Vector3(-30.7898f, 2.15201f, -10.5315f), Vector3(-29.9962f, 3.541f, -5.32914f)); 
  data[49].mAabb = Aabb(Vector3(-24.9592f, -12.926f, 34.6851f), Vector3(-20.3325f, -6.18197f, 39.3088f)); 
  data[50].mAabb = Aabb(Vector3(-10.6777f, -44.8293f, 3.10107f), Vector3(-4.04358f, -44.1829f, 5.90027f)); 
  data[51].mAabb = Aabb(Vector3(-1.49346f, 36.7532f, -5.83891f), Vector3(3.29788f, 38.9681f, 1.42329f)); 
  data[52].mAabb = Aabb(Vector3(-16.7463f, 5.45478f, 16.6305f), Vector3(-13.9921f, 13.1384f, 18.1649f)); 
  data[53].mAabb = Aabb(Vector3(-41.1414f, 17.0526f, -7.70205f), Vector3(-39.9688f, 19.3498f, -2.58137f)); 
  data[54].mAabb = Aabb(Vector3(-27.5053f, -7.7276f, -14.7626f), Vector3(-25.9349f, -0.776668f, -12.7977f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-6.50958f, -1.10386f, -17.845f));
  ray.mDirection = Vector3(Vector3(0.596264f, -0.493683f, 0.633045f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(41); 
  data[0].mAabb = Aabb(Vector3(-20.7389f, 35.9397f, -8.48852f), Vector3(-14.3724f, 36.6046f, -3.4998f)); 
  data[1].mAabb = Aabb(Vector3(-0.45128f, -2.2795f, -33.1115f), Vector3(1.33758f, 0.587418f, -27.991f)); 
  data[2].mAabb = Aabb(Vector3(-7.93406f, 27.5829f, -30.1344f), Vector3(-6.58102f, 28.7669f, -24.7778f)); 
  data[3].mAabb = Aabb(Vector3(-26.0023f, -4.15863f, -39.2985f), Vector3(-21.3295f, -3.45734f, -38.3085f)); 
  data[4].mAabb = Aabb(Vector3(44.3337f, -12.1352f, -3.31108f), Vector3(47.5766f, -10.4108f, -1.65466f)); 
  data[5].mAabb = Aabb(Vector3(36.5642f, 5.28683f, 3.13875f), Vector3(40.447f, 12.3721f, 7.65454f)); 
  data[6].mAabb = Aabb(Vector3(-20.7786f, -39.0382f, 16.1405f), Vector3(-19.7238f, -35.5341f, 21.7492f)); 
  data[7].mAabb = Aabb(Vector3(17.7689f, -25.057f, -15.4558f), Vector3(19.8245f, -19.131f, -14.3333f)); 
  data[8].mAabb = Aabb(Vector3(26.8265f, 11.5733f, -23.0673f), Vector3(30.5338f, 15.0139f, -18.2221f)); 
  data[9].mAabb = Aabb(Vector3(-18.6357f, 30.8273f, 33.2332f), Vector3(-14.4877f, 33.3204f, 35.2176f)); 
  data[10].mAabb = Aabb(Vector3(16.1062f, -20.3548f, 9.04677f), Vector3(22.6761f, -14.2122f, 11.0288f)); 
  data[11].mAabb = Aabb(Vector3(23.0696f, -23.9393f, 24.5315f), Vector3(24.7001f, -18.6546f, 30.8049f)); 
  data[12].mAabb = Aabb(Vector3(-2.14939f, 2.50256f, -27.9466f), Vector3(5.57924f, 3.27158f, -20.8505f)); 
  data[13].mAabb = Aabb(Vector3(37.3504f, -7.8736f, -3.94215f), Vector3(42.6764f, -1.5067f, 2.05064f)); 
  data[14].mAabb = Aabb(Vector3(2.37229f, 20.7897f, 29.0711f), Vector3(2.94531f, 22.1029f, 36.1899f)); 
  data[15].mAabb = Aabb(Vector3(37.1705f, -30.507f, -19.8798f), Vector3(39.3708f, -23.062f, -14.2686f)); 
  data[16].mAabb = Aabb(Vector3(14.9633f, 3.84308f, 20.301f), Vector3(17.2856f, 5.72078f, 28.1651f)); 
  data[17].mAabb = Aabb(Vector3(-40.4069f, -30.118f, 8.30044f), Vector3(-36.3819f, -24.1855f, 10.4087f)); 
  data[18].mAabb = Aabb(Vector3(-1.13907f, 27.0567f, -10.1784f), Vector3(5.95984f, 33.3587f, -5.18408f)); 
  data[19].mAabb = Aabb(Vector3(36.0678f, -15.1938f, 11.9734f), Vector3(38.4845f, -11.7164f, 16.0167f)); 
  data[20].mAabb = Aabb(Vector3(-20.9324f, 0.0820076f, 30.5544f), Vector3(-14.1679f, 6.58576f, 32.9999f)); 
  data[21].mAabb = Aabb(Vector3(-21.8228f, 7.9915f, 1.67767f), Vector3(-16.0503f, 12.4025f, 6.54044f)); 
  data[22].mAabb = Aabb(Vector3(-26.3305f, 15.1936f, 2.7164f), Vector3(-20.2002f, 15.6924f, 5.73547f)); 
  data[23].mAabb = Aabb(Vector3(-26.4293f, -39.1321f, 21.2879f), Vector3(-23.4218f, -33.3546f, 25.5427f)); 
  data[24].mAabb = Aabb(Vector3(42.4955f, -10.2665f, 15.067f), Vector3(43.4496f, -9.80553f, 21.1212f)); 
  data[25].mAabb = Aabb(Vector3(13.0797f, -43.1674f, 18.9108f), Vector3(19.8945f, -35.9537f, 26.8783f)); 
  data[26].mAabb = Aabb(Vector3(37.0693f, 19.4842f, 8.41717f), Vector3(42.0065f, 27.2669f, 13.7093f)); 
  data[27].mAabb = Aabb(Vector3(-18.8676f, 31.8476f, 21.5525f), Vector3(-12.7164f, 36.1816f, 22.6511f)); 
  data[28].mAabb = Aabb(Vector3(-0.476756f, -37.6272f, 17.8683f), Vector3(2.45348f, -33.3264f, 24.5164f)); 
  data[29].mAabb = Aabb(Vector3(17.2168f, -31.1036f, -24.0036f), Vector3(20.6865f, -29.6158f, -16.3274f)); 
  data[30].mAabb = Aabb(Vector3(-27.9241f, 3.70723f, 24.826f), Vector3(-20.157f, 8.75903f, 30.6256f)); 
  data[31].mAabb = Aabb(Vector3(34.2141f, 2.11634f, -26.7683f), Vector3(38.6195f, 9.66754f, -23.2348f)); 
  data[32].mAabb = Aabb(Vector3(14.4134f, -31.9823f, 10.363f), Vector3(21.1459f, -30.6673f, 16.4685f)); 
  data[33].mAabb = Aabb(Vector3(18.2581f, -4.4188f, -25.7343f), Vector3(24.4874f, 0.593111f, -25.2346f)); 
  data[34].mAabb = Aabb(Vector3(-18.4638f, -36.6662f, -20.5921f), Vector3(-12.9822f, -30.1578f, -19.0238f)); 
  data[35].mAabb = Aabb(Vector3(37.3773f, -17.9976f, -2.14758f), Vector3(42.1605f, -17.5802f, -1.63462f)); 
  data[36].mAabb = Aabb(Vector3(10.3473f, 9.97736f, -1.29758f), Vector3(11.365f, 13.2757f, 4.05459f)); 
  data[37].mAabb = Aabb(Vector3(34.8601f, -0.987498f, -7.07437f), Vector3(40.2686f, 1.05232f, -6.20422f)); 
  data[38].mAabb = Aabb(Vector3(22.4448f, 17.9405f, -28.3254f), Vector3(30.3928f, 22.4308f, -25.3578f)); 
  data[39].mAabb = Aabb(Vector3(-10.6658f, -18.7531f, 27.9371f), Vector3(-6.2876f, -16.3262f, 35.5632f)); 
  data[40].mAabb = Aabb(Vector3(17.8029f, -37.6831f, -24.9528f), Vector3(25.426f, -34.4296f, -23.115f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-6.44627f, 13.0371f, -1.75658f));
  ray.mDirection = Vector3(Vector3(0.98686f, -0.0419404f, 0.156041f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(36); 
  data[0].mAabb = Aabb(Vector3(32.1409f, 3.10478f, 30.2522f), Vector3(34.3312f, 6.08581f, 32.8819f)); 
  data[1].mAabb = Aabb(Vector3(-27.674f, -5.71049f, -0.894466f), Vector3(-22.6621f, 1.75466f, 4.91509f)); 
  data[2].mAabb = Aabb(Vector3(-24.485f, 17.4407f, 19.4501f), Vector3(-21.1774f, 22.072f, 23.8381f)); 
  data[3].mAabb = Aabb(Vector3(14.4376f, -25.3281f, 6.16702f), Vector3(21.4943f, -18.5346f, 13.265f)); 
  data[4].mAabb = Aabb(Vector3(-1.12536f, -11.3351f, -8.4507f), Vector3(0.640074f, -4.60191f, -1.7389f)); 
  data[5].mAabb = Aabb(Vector3(14.3008f, 31.5123f, 4.12425f), Vector3(20.9813f, 36.5435f, 10.6795f)); 
  data[6].mAabb = Aabb(Vector3(24.2608f, -13.2347f, -18.6108f), Vector3(26.3025f, -8.33183f, -11.613f)); 
  data[7].mAabb = Aabb(Vector3(-35.8184f, -26.8518f, -5.09868f), Vector3(-34.3784f, -23.5817f, -4.47973f)); 
  data[8].mAabb = Aabb(Vector3(-15.7886f, -26.0085f, -4.94911f), Vector3(-12.779f, -18.6517f, 2.94304f)); 
  data[9].mAabb = Aabb(Vector3(17.3731f, -34.9402f, -2.00012f), Vector3(22.3113f, -30.0666f, 0.486654f)); 
  data[10].mAabb = Aabb(Vector3(17.3871f, -31.1629f, -31.6636f), Vector3(24.9053f, -25.9453f, -28.8458f)); 
  data[11].mAabb = Aabb(Vector3(3.06126f, 11.7236f, 7.56696f), Vector3(6.12463f, 13.1302f, 11.1381f)); 
  data[12].mAabb = Aabb(Vector3(1.20737f, 38.505f, -11.5394f), Vector3(6.23876f, 39.3513f, -3.71078f)); 
  data[13].mAabb = Aabb(Vector3(32.3054f, 7.59506f, 10.8154f), Vector3(39.4503f, 9.81742f, 12.408f)); 
  data[14].mAabb = Aabb(Vector3(6.48086f, 8.65942f, -39.744f), Vector3(7.67015f, 10.4719f, -33.1929f)); 
  data[15].mAabb = Aabb(Vector3(19.3075f, 2.31673f, -18.9653f), Vector3(25.4912f, 4.33312f, -16.9993f)); 
  data[16].mAabb = Aabb(Vector3(-1.96921f, 12.0498f, -25.1717f), Vector3(2.78223f, 17.9605f, -19.5893f)); 
  data[17].mAabb = Aabb(Vector3(17.9072f, -14.866f, 7.85433f), Vector3(22.0319f, -10.0044f, 9.60028f)); 
  data[18].mAabb = Aabb(Vector3(20.3927f, 31.526f, 2.00688f), Vector3(28.1793f, 37.9674f, 8.30745f)); 
  data[19].mAabb = Aabb(Vector3(-16.6443f, 18.515f, 29.8882f), Vector3(-11.5125f, 25.1517f, 37.0812f)); 
  data[20].mAabb = Aabb(Vector3(-10.3821f, -20.3214f, 23.5351f), Vector3(-4.01033f, -15.1576f, 30.3865f)); 
  data[21].mAabb = Aabb(Vector3(-29.6926f, 21.727f, -20.0446f), Vector3(-26.0674f, 22.8845f, -17.0195f)); 
  data[22].mAabb = Aabb(Vector3(-5.34125f, -17.6468f, -7.26119f), Vector3(1.12168f, -16.3089f, 0.217171f)); 
  data[23].mAabb = Aabb(Vector3(-3.01011f, 8.95189f, 14.2382f), Vector3(-1.36204f, 12.0828f, 14.6505f)); 
  data[24].mAabb = Aabb(Vector3(-11.7139f, 18.8217f, 9.33926f), Vector3(-10.3175f, 23.8851f, 13.0908f)); 
  data[25].mAabb = Aabb(Vector3(25.2825f, 6.41046f, -14.288f), Vector3(29.2064f, 7.64475f, -8.50417f)); 
  data[26].mAabb = Aabb(Vector3(19.6868f, 23.7018f, 16.9643f), Vector3(22.1777f, 29.0328f, 18.6276f)); 
  data[27].mAabb = Aabb(Vector3(-27.1724f, -1.42402f, -44.3219f), Vector3(-21.6094f, 0.217099f, -37.2202f)); 
  data[28].mAabb = Aabb(Vector3(11.1588f, -17.7042f, -37.5079f), Vector3(13.9613f, -15.905f, -36.3569f)); 
  data[29].mAabb = Aabb(Vector3(-18.9389f, -2.02461f, -46.7172f), Vector3(-12.5516f, 4.87646f, -44.6454f)); 
  data[30].mAabb = Aabb(Vector3(-37.0764f, -11.0005f, -34.6596f), Vector3(-35.9696f, -3.09332f, -29.8509f)); 
  data[31].mAabb = Aabb(Vector3(-46.9391f, 12.9509f, -14.0582f), Vector3(-40.9087f, 19.8385f, -8.70792f)); 
  data[32].mAabb = Aabb(Vector3(-14.8617f, -34.374f, 9.50039f), Vector3(-10.7026f, -29.2396f, 15.4528f)); 
  data[33].mAabb = Aabb(Vector3(-40.6165f, -2.52073f, -19.3696f), Vector3(-34.4387f, -0.993495f, -13.3641f)); 
  data[34].mAabb = Aabb(Vector3(20.5061f, -1.57656f, 7.81201f), Vector3(25.6117f, 5.24982f, 13.0116f)); 
  data[35].mAabb = Aabb(Vector3(2.79308f, 16.6523f, -9.45627f), Vector3(7.30353f, 19.4737f, -7.15065f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-1.11691f, 12.8045f, -2.78f));
  ray.mDirection = Vector3(Vector3(-0.515249f, 0.493792f, 0.700491f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(-0.71402f, -20.1794f, 4.55023f), Vector3(4.62933f, -17.132f, 11.8175f)); 
  data[1].mAabb = Aabb(Vector3(-2.3699f, 22.1625f, -18.6877f), Vector3(1.01796f, 23.9812f, -17.3033f)); 
  data[2].mAabb = Aabb(Vector3(1.37574f, -16.4512f, 31.9564f), Vector3(5.72267f, -15.0666f, 39.2733f)); 
  data[3].mAabb = Aabb(Vector3(-43.1886f, -5.42173f, -10.4218f), Vector3(-39.5688f, 1.9126f, -2.66885f)); 
  data[4].mAabb = Aabb(Vector3(-18.8861f, 31.5477f, -13.0763f), Vector3(-15.7244f, 37.9457f, -7.732f)); 
  data[5].mAabb = Aabb(Vector3(-4.72546f, -3.12603f, 32.9239f), Vector3(-1.53568f, 4.43653f, 35.5631f)); 
  data[6].mAabb = Aabb(Vector3(25.7377f, -0.439808f, 16.4279f), Vector3(29.7599f, 3.19298f, 19.7976f)); 
  data[7].mAabb = Aabb(Vector3(-20.2962f, -15.9812f, 29.2335f), Vector3(-18.2886f, -10.1236f, 30.1646f)); 
  data[8].mAabb = Aabb(Vector3(37.8426f, 11.5562f, -11.0642f), Vector3(39.243f, 12.1109f, -8.5139f)); 
  data[9].mAabb = Aabb(Vector3(-1.94766f, -38.1673f, 21.252f), Vector3(1.54179f, -35.4532f, 24.3669f)); 
  data[10].mAabb = Aabb(Vector3(-6.91138f, -24.7024f, 40.2905f), Vector3(-2.66209f, -20.2306f, 44.1534f)); 
  data[11].mAabb = Aabb(Vector3(-17.8767f, 8.18901f, -3.88509f), Vector3(-12.6866f, 15.91f, 3.98131f)); 
  data[12].mAabb = Aabb(Vector3(0.885337f, 33.9529f, -10.9483f), Vector3(4.401f, 38.9966f, -8.32401f)); 
  data[13].mAabb = Aabb(Vector3(-13.1914f, -23.9449f, -1.24279f), Vector3(-5.92846f, -18.3149f, 1.45088f)); 
  data[14].mAabb = Aabb(Vector3(-22.6411f, -37.2493f, -6.70057f), Vector3(-22.0529f, -35.2584f, 0.316694f)); 
  data[15].mAabb = Aabb(Vector3(-6.14141f, 23.3058f, -9.22332f), Vector3(-1.83228f, 25.9628f, -6.53732f)); 
  data[16].mAabb = Aabb(Vector3(18.9526f, -39.3597f, -2.17748f), Vector3(22.2919f, -38.5445f, -1.60028f)); 
  data[17].mAabb = Aabb(Vector3(-0.324551f, 18.4282f, -22.8986f), Vector3(5.6571f, 20.1537f, -16.8671f)); 
  data[18].mAabb = Aabb(Vector3(16.3035f, 0.602117f, -11.2856f), Vector3(22.9318f, 2.54568f, -10.8745f)); 
  data[19].mAabb = Aabb(Vector3(6.21214f, -34.2897f, -29.6334f), Vector3(12.5129f, -31.9249f, -24.8615f)); 
  data[20].mAabb = Aabb(Vector3(-16.7023f, 18.312f, 8.88792f), Vector3(-15.9794f, 23.2223f, 10.7893f)); 
  data[21].mAabb = Aabb(Vector3(-35.3615f, 0.609308f, -17.4433f), Vector3(-27.5125f, 3.78633f, -14.3917f)); 
  data[22].mAabb = Aabb(Vector3(-10.7384f, -16.837f, -47.6786f), Vector3(-9.03144f, -9.45052f, -40.4136f)); 
  data[23].mAabb = Aabb(Vector3(-4.94688f, 35.5033f, -27.3659f), Vector3(0.657635f, 39.2882f, -20.8394f)); 
  data[24].mAabb = Aabb(Vector3(-31.3942f, -19.7745f, -23.6203f), Vector3(-28.4612f, -14.3013f, -18.8937f)); 
  data[25].mAabb = Aabb(Vector3(3.03036f, 0.675735f, -27.5559f), Vector3(9.20939f, 7.64801f, -23.9642f)); 
  data[26].mAabb = Aabb(Vector3(36.0197f, 15.4887f, -15.8951f), Vector3(37.2018f, 20.9566f, -11.4243f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(13.6738f, -18.302f, -3.45742f));
  ray.mDirection = Vector3(Vector3(-0.305053f, 0.89295f, -0.331034f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(7.41722f, 5.48406f, 32.6917f), Vector3(10.6485f, 8.11741f, 39.3499f)); 
  data[1].mAabb = Aabb(Vector3(-11.1658f, 46.2562f, 4.79507f), Vector3(-8.36683f, 47.3559f, 7.58522f)); 
  data[2].mAabb = Aabb(Vector3(2.39599f, -36.6269f, 11.9614f), Vector3(7.41207f, -34.8415f, 15.5849f)); 
  data[3].mAabb = Aabb(Vector3(-1.67469f, 8.15009f, 23.6799f), Vector3(2.88748f, 9.88051f, 27.7031f)); 
  data[4].mAabb = Aabb(Vector3(24.9597f, -22.3279f, 18.5218f), Vector3(32.0976f, -17.7191f, 18.9893f)); 
  data[5].mAabb = Aabb(Vector3(30.0993f, -8.54811f, 5.87446f), Vector3(35.628f, -3.80061f, 11.8197f)); 
  data[6].mAabb = Aabb(Vector3(4.35902f, -7.11747f, -21.7875f), Vector3(7.71255f, -3.97894f, -15.6231f)); 
  data[7].mAabb = Aabb(Vector3(-21.3103f, -22.8948f, -13.6977f), Vector3(-19.3491f, -19.764f, -9.1863f)); 
  data[8].mAabb = Aabb(Vector3(-16.3305f, -18.9769f, 15.9068f), Vector3(-10.1169f, -12.2614f, 23.4225f)); 
  data[9].mAabb = Aabb(Vector3(10.4453f, 4.91938f, -46.5155f), Vector3(14.6345f, 12.4903f, -40.497f)); 
  data[10].mAabb = Aabb(Vector3(7.01266f, -10.5319f, 40.5528f), Vector3(13.3937f, -7.05449f, 44.4824f)); 
  data[11].mAabb = Aabb(Vector3(27.142f, 9.57838f, 33.3742f), Vector3(34.5661f, 13.4475f, 39.1251f)); 
  data[12].mAabb = Aabb(Vector3(-31.4964f, -29.9838f, -21.6115f), Vector3(-29.1128f, -23.0345f, -14.3101f)); 
  data[13].mAabb = Aabb(Vector3(40.6602f, 7.85015f, 17.835f), Vector3(41.7263f, 15.5208f, 19.2697f)); 
  data[14].mAabb = Aabb(Vector3(34.8413f, -28.3916f, -12.0716f), Vector3(42.3073f, -24.2472f, -9.11515f)); 
  data[15].mAabb = Aabb(Vector3(18.2382f, -44.7985f, 9.73295f), Vector3(19.3016f, -40.3604f, 11.0976f)); 
  data[16].mAabb = Aabb(Vector3(-8.98347f, -20.7539f, 32.9906f), Vector3(-4.18866f, -15.3637f, 33.6158f)); 
  data[17].mAabb = Aabb(Vector3(13.6982f, -15.4295f, -42.9228f), Vector3(16.1819f, -8.2471f, -36.8591f)); 
  data[18].mAabb = Aabb(Vector3(-19.5442f, -9.17946f, 8.33943f), Vector3(-18.6541f, -4.48856f, 12.5446f)); 
  data[19].mAabb = Aabb(Vector3(-33.2643f, -26.9556f, 20.7826f), Vector3(-29.1978f, -22.2954f, 27.7206f)); 
  data[20].mAabb = Aabb(Vector3(19.208f, 11.3361f, 23.2995f), Vector3(23.3447f, 16.5755f, 29.7731f)); 
  data[21].mAabb = Aabb(Vector3(-39.3016f, -0.473524f, -21.4341f), Vector3(-37.3894f, 1.67948f, -17.8426f)); 
  data[22].mAabb = Aabb(Vector3(18.5253f, 6.98836f, 41.3655f), Vector3(20.7692f, 9.47861f, 46.0845f)); 
  data[23].mAabb = Aabb(Vector3(-21.6001f, 8.55431f, -23.9202f), Vector3(-16.3238f, 12.0927f, -16.6573f)); 
  data[24].mAabb = Aabb(Vector3(3.42598f, -16.4419f, 38.366f), Vector3(4.53826f, -13.0332f, 45.014f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(6.54515f, -6.01411f, -18.2316f));
  ray.mDirection = Vector3(Vector3(0.837635f, -0.522492f, 0.159276f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mAabb = Aabb(Vector3(20.7247f, -3.45095f, -1.82715f), Vector3(23.9244f, 4.07171f, 5.10245f)); 
  data[1].mAabb = Aabb(Vector3(31.1014f, 7.41669f, -5.04814f), Vector3(34.3649f, 10.1277f, -2.63536f)); 
  data[2].mAabb = Aabb(Vector3(-39.651f, 4.71643f, 8.92341f), Vector3(-37.2342f, 6.47792f, 14.2276f)); 
  data[3].mAabb = Aabb(Vector3(23.1177f, 4.03466f, -31.1525f), Vector3(27.3496f, 5.77806f, -28.6947f)); 
  data[4].mAabb = Aabb(Vector3(12.9643f, -35.3919f, -6.56239f), Vector3(18.7521f, -34.9429f, -5.92465f)); 
  data[5].mAabb = Aabb(Vector3(37.2027f, -21.5542f, -7.09886f), Vector3(43.95f, -13.5941f, -2.48705f)); 
  data[6].mAabb = Aabb(Vector3(30.2325f, -22.2683f, 31.6451f), Vector3(32.12f, -14.8705f, 32.3267f)); 
  data[7].mAabb = Aabb(Vector3(-14.3711f, -47.2366f, 4.11248f), Vector3(-13.6146f, -41.7063f, 4.6477f)); 
  data[8].mAabb = Aabb(Vector3(-38.0296f, 12.1549f, -20.3576f), Vector3(-32.4576f, 17.742f, -19.8992f)); 
  data[9].mAabb = Aabb(Vector3(-4.40816f, 31.6501f, -18.3722f), Vector3(-3.48165f, 37.3483f, -11.0087f)); 
  data[10].mAabb = Aabb(Vector3(-5.81074f, 3.34381f, 32.5208f), Vector3(-3.34995f, 4.50666f, 37.6353f)); 
  data[11].mAabb = Aabb(Vector3(32.3864f, -1.80728f, -29.3132f), Vector3(33.764f, 5.02536f, -28.9118f)); 
  data[12].mAabb = Aabb(Vector3(18.0273f, -34.6898f, -32.6049f), Vector3(21.6643f, -31.8978f, -29.102f)); 
  data[13].mAabb = Aabb(Vector3(10.5139f, 38.3502f, 6.69635f), Vector3(16.3508f, 40.1652f, 14.5106f)); 
  data[14].mAabb = Aabb(Vector3(9.94771f, 13.9256f, 16.6707f), Vector3(15.0689f, 19.4116f, 21.0369f)); 
  data[15].mAabb = Aabb(Vector3(26.7702f, 21.0747f, 24.4484f), Vector3(32.4469f, 26.0519f, 26.7122f)); 
  data[16].mAabb = Aabb(Vector3(-17.4486f, -10.2974f, -19.635f), Vector3(-9.76036f, -4.09267f, -12.0611f)); 
  data[17].mAabb = Aabb(Vector3(4.00004f, -19.5327f, -30.0211f), Vector3(6.49771f, -13.3349f, -26.5544f)); 
  data[18].mAabb = Aabb(Vector3(-38.1431f, -15.6193f, 16.7324f), Vector3(-32.559f, -9.267f, 23.816f)); 
  data[19].mAabb = Aabb(Vector3(2.88624f, 43.3014f, -0.52386f), Vector3(3.80857f, 50.0784f, 3.29935f)); 
  data[20].mAabb = Aabb(Vector3(-43.82f, -4.84527f, -14.6061f), Vector3(-42.9102f, 0.834391f, -10.1207f)); 
  data[21].mAabb = Aabb(Vector3(10.8919f, -49.2151f, -2.59283f), Vector3(18.3499f, -42.5204f, -0.703075f)); 
  data[22].mAabb = Aabb(Vector3(-20.1095f, -11.6768f, -11.9591f), Vector3(-14.6791f, -8.17269f, -11.3685f)); 
  data[23].mAabb = Aabb(Vector3(-0.440155f, -31.1845f, -2.64952f), Vector3(6.25263f, -24.5147f, 4.00522f)); 
  data[24].mAabb = Aabb(Vector3(12.8529f, 5.68602f, 25.2502f), Vector3(15.8268f, 9.84624f, 29.9833f)); 
  data[25].mAabb = Aabb(Vector3(16.1205f, -9.18949f, -39.2751f), Vector3(20.0114f, -8.15954f, -38.3008f)); 
  data[26].mAabb = Aabb(Vector3(43.449f, -12.3649f, 17.1757f), Vector3(44.8354f, -7.77211f, 23.6112f)); 
  data[27].mAabb = Aabb(Vector3(21.0302f, -35.5183f, 16.6468f), Vector3(27.4736f, -33.5107f, 17.3885f)); 
  data[28].mAabb = Aabb(Vector3(-15.9648f, 38.6519f, -6.25601f), Vector3(-13.1687f, 45.2579f, -3.94064f)); 
  data[29].mAabb = Aabb(Vector3(27.9238f, -21.3175f, 15.8929f), Vector3(30.7642f, -18.249f, 16.4344f)); 
  data[30].mAabb = Aabb(Vector3(8.97831f, -4.84282f, 44.2042f), Vector3(14.9382f, -3.56261f, 51.8878f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(21.5388f, 3.29856f, 4.55234f));
  ray.mDirection = Vector3(Vector3(-0.859087f, -0.185493f, -0.477035f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(34); 
  data[0].mAabb = Aabb(Vector3(-4.40014f, -34.5368f, 20.1117f), Vector3(-1.63504f, -31.8564f, 23.0679f)); 
  data[1].mAabb = Aabb(Vector3(-15.8506f, -42.9899f, 16.8078f), Vector3(-11.6973f, -36.7478f, 18.9356f)); 
  data[2].mAabb = Aabb(Vector3(-27.1358f, 8.33064f, -7.6056f), Vector3(-21.6097f, 15.2765f, -1.3113f)); 
  data[3].mAabb = Aabb(Vector3(-27.5545f, -8.3313f, 35.8636f), Vector3(-22.7863f, -6.99264f, 43.0989f)); 
  data[4].mAabb = Aabb(Vector3(4.31656f, -7.70051f, -35.2473f), Vector3(8.29818f, -4.24817f, -33.1757f)); 
  data[5].mAabb = Aabb(Vector3(-24.0845f, 29.9442f, -24.131f), Vector3(-21.6464f, 33.2777f, -22.6347f)); 
  data[6].mAabb = Aabb(Vector3(15.2396f, -32.8626f, -4.83141f), Vector3(21.9004f, -25.4325f, -4.08257f)); 
  data[7].mAabb = Aabb(Vector3(-46.9087f, -3.56516f, 1.29019f), Vector3(-40.1681f, 4.03265f, 7.58102f)); 
  data[8].mAabb = Aabb(Vector3(-26.4426f, -3.20046f, -17.9531f), Vector3(-26.0231f, 1.45936f, -13.4232f)); 
  data[9].mAabb = Aabb(Vector3(-8.32432f, 15.1288f, -25.1608f), Vector3(-6.48397f, 23.0246f, -17.4231f)); 
  data[10].mAabb = Aabb(Vector3(-47.8598f, 4.55635f, -20.15f), Vector3(-40.7073f, 5.35181f, -13.5012f)); 
  data[11].mAabb = Aabb(Vector3(14.9018f, 30.9637f, 10.7283f), Vector3(21.4067f, 36.3634f, 11.6358f)); 
  data[12].mAabb = Aabb(Vector3(-6.51341f, 2.80909f, 27.4289f), Vector3(-1.01698f, 7.30447f, 34.2485f)); 
  data[13].mAabb = Aabb(Vector3(-5.5767f, 6.33931f, -19.1767f), Vector3(1.48881f, 9.98323f, -11.3715f)); 
  data[14].mAabb = Aabb(Vector3(-13.8648f, 2.53424f, -34.6587f), Vector3(-5.86871f, 10.3151f, -30.0409f)); 
  data[15].mAabb = Aabb(Vector3(8.18759f, -19.7496f, 8.12364f), Vector3(10.6871f, -18.2122f, 10.5074f)); 
  data[16].mAabb = Aabb(Vector3(-18.0056f, -32.567f, 9.71199f), Vector3(-15.2178f, -32.1392f, 14.4588f)); 
  data[17].mAabb = Aabb(Vector3(8.01671f, -30.4249f, -33.8716f), Vector3(9.80302f, -22.4821f, -27.4121f)); 
  data[18].mAabb = Aabb(Vector3(4.17904f, -29.0795f, 23.4936f), Vector3(8.85046f, -23.2655f, 27.0531f)); 
  data[19].mAabb = Aabb(Vector3(-7.94165f, -7.52779f, 25.4149f), Vector3(-0.438234f, -1.19359f, 26.3931f)); 
  data[20].mAabb = Aabb(Vector3(4.66225f, -10.6191f, 29.3253f), Vector3(5.28399f, -2.80072f, 35.3193f)); 
  data[21].mAabb = Aabb(Vector3(-32.7999f, -12.471f, 18.6706f), Vector3(-32.174f, -5.41641f, 26.3825f)); 
  data[22].mAabb = Aabb(Vector3(11.6809f, -28.5626f, -7.53103f), Vector3(15.8971f, -21.5586f, -3.24393f)); 
  data[23].mAabb = Aabb(Vector3(26.684f, -32.4547f, -7.35183f), Vector3(32.776f, -30.7857f, -6.32258f)); 
  data[24].mAabb = Aabb(Vector3(-36.1245f, -33.4117f, 9.40744f), Vector3(-32.0995f, -26.1293f, 15.0347f)); 
  data[25].mAabb = Aabb(Vector3(-34.5961f, 21.4191f, 20.3991f), Vector3(-31.305f, 23.9634f, 25.2199f)); 
  data[26].mAabb = Aabb(Vector3(-9.56503f, 13.9552f, 20.8906f), Vector3(-4.32049f, 18.0331f, 22.9993f)); 
  data[27].mAabb = Aabb(Vector3(-1.74009f, -5.52923f, -0.863864f), Vector3(2.50757f, -0.575073f, 2.96213f)); 
  data[28].mAabb = Aabb(Vector3(16.9004f, -14.4533f, 28.7938f), Vector3(19.7177f, -11.248f, 29.3474f)); 
  data[29].mAabb = Aabb(Vector3(24.6758f, 20.7465f, -24.4802f), Vector3(27.5288f, 22.3758f, -22.8442f)); 
  data[30].mAabb = Aabb(Vector3(4.9176f, -41.4198f, -25.3818f), Vector3(6.63595f, -36.7495f, -19.8232f)); 
  data[31].mAabb = Aabb(Vector3(26.3493f, -13.0628f, -19.1824f), Vector3(30.2643f, -12.0007f, -12.5543f)); 
  data[32].mAabb = Aabb(Vector3(-15.4578f, 16.2609f, 0.190524f), Vector3(-9.20042f, 19.6675f, 5.12148f)); 
  data[33].mAabb = Aabb(Vector3(-13.4236f, -0.746982f, -38.3736f), Vector3(-10.8412f, 3.93371f, -33.8785f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(7.40894f, -17.7488f, -5.14847f));
  ray.mDirection = Vector3(Vector3(-0.399723f, 0.855809f, -0.328349f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(-23.5971f, -33.0239f, 24.0119f), Vector3(-20.0195f, -27.1777f, 28.2602f)); 
  data[1].mAabb = Aabb(Vector3(14.5592f, -30.7079f, 23.9017f), Vector3(19.6228f, -23.8172f, 31.0639f)); 
  data[2].mAabb = Aabb(Vector3(-3.18288f, 22.5247f, 11.5787f), Vector3(-0.0877248f, 26.3094f, 13.3446f)); 
  data[3].mAabb = Aabb(Vector3(18.9708f, 21.3111f, -29.3679f), Vector3(20.1404f, 28.2279f, -25.3914f)); 
  data[4].mAabb = Aabb(Vector3(-5.40191f, 38.8902f, -15.6846f), Vector3(-4.57792f, 43.3515f, -13.6438f)); 
  data[5].mAabb = Aabb(Vector3(32.3184f, -27.4881f, 22.082f), Vector3(38.6115f, -20.2131f, 24.5131f)); 
  data[6].mAabb = Aabb(Vector3(13.9554f, 3.53286f, -5.89783f), Vector3(15.3829f, 6.93f, -4.52182f)); 
  data[7].mAabb = Aabb(Vector3(8.46344f, -16.4995f, -32.2071f), Vector3(15.0303f, -10.1175f, -28.3808f)); 
  data[8].mAabb = Aabb(Vector3(-4.49842f, -40.9096f, 10.2393f), Vector3(-0.456021f, -39.7945f, 10.8424f)); 
  data[9].mAabb = Aabb(Vector3(-21.7804f, 0.462646f, -7.86854f), Vector3(-18.1058f, 5.212f, -6.52918f)); 
  data[10].mAabb = Aabb(Vector3(17.0192f, -13.5815f, -37.526f), Vector3(24.5402f, -8.11847f, -32.7592f)); 
  data[11].mAabb = Aabb(Vector3(16.0972f, 25.0047f, 13.2345f), Vector3(16.7233f, 29.912f, 15.3355f)); 
  data[12].mAabb = Aabb(Vector3(-35.1912f, 34.4866f, -0.491915f), Vector3(-34.0488f, 36.0162f, 5.52569f)); 
  data[13].mAabb = Aabb(Vector3(6.3451f, 25.3893f, 27.8807f), Vector3(9.62696f, 27.4715f, 28.8944f)); 
  data[14].mAabb = Aabb(Vector3(19.2263f, 38.8607f, -10.3668f), Vector3(25.33f, 43.6846f, -4.83238f)); 
  data[15].mAabb = Aabb(Vector3(39.1137f, -8.33644f, 14.7831f), Vector3(45.1517f, -1.14777f, 16.8073f)); 
  data[16].mAabb = Aabb(Vector3(-1.65704f, 30.2418f, 28.1471f), Vector3(-0.354327f, 35.8127f, 33.0266f)); 
  data[17].mAabb = Aabb(Vector3(19.3023f, -38.2727f, 22.4171f), Vector3(21.5891f, -35.1752f, 26.3843f)); 
  data[18].mAabb = Aabb(Vector3(16.3586f, -28.9463f, -7.53886f), Vector3(20.2881f, -25.5517f, -0.135184f)); 
  data[19].mAabb = Aabb(Vector3(13.4459f, 19.2766f, 21.6048f), Vector3(16.711f, 24.2243f, 22.6909f)); 
  data[20].mAabb = Aabb(Vector3(-8.68155f, -31.3958f, 26.5385f), Vector3(-7.11002f, -30.3506f, 32.2525f)); 
  data[21].mAabb = Aabb(Vector3(12.0718f, 6.37796f, 2.68926f), Vector3(19.1707f, 10.2336f, 9.2387f)); 
  data[22].mAabb = Aabb(Vector3(-1.61533f, -32.3256f, -25.0038f), Vector3(4.73858f, -27.8903f, -21.5881f)); 
  data[23].mAabb = Aabb(Vector3(-24.2265f, 36.2286f, 5.11937f), Vector3(-23.766f, 39.0213f, 9.79032f)); 
  data[24].mAabb = Aabb(Vector3(9.56917f, -10.8317f, -3.97016f), Vector3(13.3877f, -6.78563f, -2.43087f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(14.3584f, -13.816f, 7.30092f));
  ray.mDirection = Vector3(Vector3(0.26113f, -0.811663f, -0.522507f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(34); 
  data[0].mAabb = Aabb(Vector3(10.2868f, -23.4738f, -42.0641f), Vector3(12.6165f, -21.5251f, -39.1046f)); 
  data[1].mAabb = Aabb(Vector3(0.275641f, 0.893734f, -50.9128f), Vector3(7.73244f, 3.38584f, -45.6184f)); 
  data[2].mAabb = Aabb(Vector3(-1.17068f, -5.73654f, 3.10772f), Vector3(-0.526678f, 1.67364f, 7.93083f)); 
  data[3].mAabb = Aabb(Vector3(12.7792f, 27.449f, 1.30171f), Vector3(18.8696f, 30.2275f, 6.82899f)); 
  data[4].mAabb = Aabb(Vector3(-0.683641f, -24.9448f, -38.5435f), Vector3(2.66177f, -17.2598f, -32.3856f)); 
  data[5].mAabb = Aabb(Vector3(-22.9773f, -6.32313f, -15.9036f), Vector3(-19.4902f, -4.02098f, -11.2978f)); 
  data[6].mAabb = Aabb(Vector3(31.993f, 10.4552f, 20.6391f), Vector3(36.0918f, 15.5467f, 21.3139f)); 
  data[7].mAabb = Aabb(Vector3(-25.5993f, 6.97816f, 4.96384f), Vector3(-23.7346f, 10.9737f, 11.7401f)); 
  data[8].mAabb = Aabb(Vector3(-47.769f, 4.20342f, -8.73079f), Vector3(-42.2345f, 6.86275f, -5.21026f)); 
  data[9].mAabb = Aabb(Vector3(2.14651f, -35.4395f, 8.51523f), Vector3(8.30142f, -30.3566f, 11.8221f)); 
  data[10].mAabb = Aabb(Vector3(18.5888f, -28.7367f, 29.032f), Vector3(23.7642f, -21.3372f, 35.352f)); 
  data[11].mAabb = Aabb(Vector3(-24.2804f, 21.3421f, 19.8884f), Vector3(-20.2501f, 24.1923f, 23.5161f)); 
  data[12].mAabb = Aabb(Vector3(38.1657f, -8.85655f, 22.1355f), Vector3(44.2459f, -3.8291f, 24.7925f)); 
  data[13].mAabb = Aabb(Vector3(3.1624f, 3.28514f, -7.77435f), Vector3(7.41517f, 11.1362f, -1.61178f)); 
  data[14].mAabb = Aabb(Vector3(-2.15339f, -7.48852f, 30.8709f), Vector3(4.20795f, -1.8339f, 36.7192f)); 
  data[15].mAabb = Aabb(Vector3(-7.82042f, -5.12514f, -0.500125f), Vector3(-2.0162f, 1.23713f, 6.91144f)); 
  data[16].mAabb = Aabb(Vector3(7.69604f, -32.8472f, -32.5294f), Vector3(9.24067f, -27.485f, -25.0542f)); 
  data[17].mAabb = Aabb(Vector3(-2.92636f, -4.46099f, -34.0226f), Vector3(-0.862181f, 3.24073f, -26.772f)); 
  data[18].mAabb = Aabb(Vector3(17.4303f, -28.24f, -1.38728f), Vector3(24.5621f, -27.0757f, 2.35174f)); 
  data[19].mAabb = Aabb(Vector3(-38.1248f, -13.232f, 24.3673f), Vector3(-31.6275f, -9.01337f, 26.9733f)); 
  data[20].mAabb = Aabb(Vector3(-3.65975f, 32.038f, -22.8936f), Vector3(2.46524f, 35.119f, -15.1944f)); 
  data[21].mAabb = Aabb(Vector3(-29.6273f, -27.3596f, -29.3483f), Vector3(-23.119f, -24.9862f, -25.8779f)); 
  data[22].mAabb = Aabb(Vector3(-44.4907f, 11.6514f, -23.3257f), Vector3(-40.8085f, 17.6762f, -16.0702f)); 
  data[23].mAabb = Aabb(Vector3(-30.3652f, 26.0071f, 8.02982f), Vector3(-23.63f, 31.9199f, 10.6504f)); 
  data[24].mAabb = Aabb(Vector3(-3.64217f, 25.0795f, -36.0925f), Vector3(-1.18184f, 27.1012f, -28.7503f)); 
  data[25].mAabb = Aabb(Vector3(-35.6865f, -38.6136f, -11.1622f), Vector3(-28.0516f, -35.4526f, -9.76206f)); 
  data[26].mAabb = Aabb(Vector3(-1.55006f, -1.91041f, 41.1646f), Vector3(3.26145f, 1.16177f, 43.3547f)); 
  data[27].mAabb = Aabb(Vector3(-5.16752f, -12.3237f, 3.85317f), Vector3(-0.538774f, -5.69103f, 11.5827f)); 
  data[28].mAabb = Aabb(Vector3(3.94217f, -39.2573f, 1.09308f), Vector3(11.3347f, -36.0879f, 2.01819f)); 
  data[29].mAabb = Aabb(Vector3(27.432f, -14.2456f, -39.4183f), Vector3(33.1022f, -7.2641f, -37.1132f)); 
  data[30].mAabb = Aabb(Vector3(33.5921f, -16.5915f, -33.0386f), Vector3(36.7467f, -14.8068f, -27.4782f)); 
  data[31].mAabb = Aabb(Vector3(-15.1535f, -28.8609f, -25.1398f), Vector3(-14.6598f, -25.3598f, -23.1867f)); 
  data[32].mAabb = Aabb(Vector3(13.9223f, -16.3145f, -13.0188f), Vector3(15.7928f, -14.1815f, -9.17057f)); 
  data[33].mAabb = Aabb(Vector3(23.2972f, 17.9155f, 33.8741f), Vector3(24.3095f, 21.501f, 37.1691f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-6.47478f, -10.2168f, 9.7354f));
  ray.mDirection = Vector3(Vector3(0.443227f, 0.884202f, -0.147435f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(44); 
  data[0].mAabb = Aabb(Vector3(21.5772f, 15.3274f, 22.4645f), Vector3(25.1418f, 22.9482f, 23.2989f)); 
  data[1].mAabb = Aabb(Vector3(27.272f, -22.4173f, 3.22478f), Vector3(29.4598f, -21.6575f, 10.1678f)); 
  data[2].mAabb = Aabb(Vector3(5.3483f, -9.24705f, 32.9704f), Vector3(8.05054f, -8.13847f, 38.7637f)); 
  data[3].mAabb = Aabb(Vector3(3.28434f, 29.4708f, -29.393f), Vector3(9.26089f, 37.4615f, -28.6384f)); 
  data[4].mAabb = Aabb(Vector3(28.4709f, -24.9517f, 9.4032f), Vector3(35.1256f, -24.1247f, 16.6278f)); 
  data[5].mAabb = Aabb(Vector3(21.6252f, 37.2193f, 18.5358f), Vector3(26.7694f, 38.4527f, 23.9079f)); 
  data[6].mAabb = Aabb(Vector3(6.97828f, -18.8605f, 10.8094f), Vector3(9.09812f, -14.4122f, 16.6224f)); 
  data[7].mAabb = Aabb(Vector3(-40.4345f, 25.8088f, 7.53564f), Vector3(-34.4295f, 31.8366f, 12.063f)); 
  data[8].mAabb = Aabb(Vector3(-1.70179f, 5.32483f, -1.84394f), Vector3(6.25484f, 12.8329f, 4.96876f)); 
  data[9].mAabb = Aabb(Vector3(20.895f, -2.06307f, -12.2681f), Vector3(22.2886f, 1.56207f, -8.06726f)); 
  data[10].mAabb = Aabb(Vector3(-37.0035f, -8.42239f, -0.370603f), Vector3(-29.0063f, -5.80388f, 5.88915f)); 
  data[11].mAabb = Aabb(Vector3(-47.9981f, -10.2335f, 18.8361f), Vector3(-42.6854f, -5.53793f, 19.3885f)); 
  data[12].mAabb = Aabb(Vector3(12.6186f, 42.0493f, -15.673f), Vector3(14.8284f, 45.6809f, -14.5457f)); 
  data[13].mAabb = Aabb(Vector3(0.104855f, -0.933148f, 11.7806f), Vector3(5.46537f, 3.95815f, 17.6607f)); 
  data[14].mAabb = Aabb(Vector3(-44.2583f, 2.5752f, -21.6754f), Vector3(-36.4362f, 9.69568f, -16.5366f)); 
  data[15].mAabb = Aabb(Vector3(1.55017f, 24.7047f, -42.7081f), Vector3(2.40408f, 27.2369f, -40.4964f)); 
  data[16].mAabb = Aabb(Vector3(-24.1047f, 3.52081f, -33.0535f), Vector3(-18.6881f, 8.45456f, -30.5085f)); 
  data[17].mAabb = Aabb(Vector3(-29.3038f, 16.7647f, 2.61293f), Vector3(-27.3486f, 20.8398f, 10.1637f)); 
  data[18].mAabb = Aabb(Vector3(15.5308f, -21.2341f, -39.7818f), Vector3(19.1494f, -15.4391f, -35.7962f)); 
  data[19].mAabb = Aabb(Vector3(-40.834f, 0.250003f, 29.6434f), Vector3(-35.1179f, 7.87843f, 32.4218f)); 
  data[20].mAabb = Aabb(Vector3(24.1098f, -28.0469f, -10.5005f), Vector3(28.6872f, -23.1472f, -2.95902f)); 
  data[21].mAabb = Aabb(Vector3(-24.537f, 27.6187f, -18.252f), Vector3(-17.7832f, 31.7949f, -15.2181f)); 
  data[22].mAabb = Aabb(Vector3(-33.7609f, -27.7714f, -21.5782f), Vector3(-27.8984f, -23.8909f, -15.2859f)); 
  data[23].mAabb = Aabb(Vector3(8.98046f, 13.5358f, -1.84243f), Vector3(14.2501f, 15.5086f, 4.53839f)); 
  data[24].mAabb = Aabb(Vector3(-4.01893f, -7.10922f, 44.1494f), Vector3(-0.636401f, -2.2112f, 49.2428f)); 
  data[25].mAabb = Aabb(Vector3(8.93858f, -44.1804f, 10.7045f), Vector3(10.6611f, -36.8088f, 15.474f)); 
  data[26].mAabb = Aabb(Vector3(-12.362f, 21.6619f, 1.12797f), Vector3(-10.6191f, 26.8837f, 6.61095f)); 
  data[27].mAabb = Aabb(Vector3(-4.30738f, -40.5663f, -30.6996f), Vector3(-2.63634f, -35.0179f, -27.7829f)); 
  data[28].mAabb = Aabb(Vector3(-7.72503f, -3.48143f, 8.15679f), Vector3(-3.47458f, 3.78216f, 13.1875f)); 
  data[29].mAabb = Aabb(Vector3(-22.0366f, 12.2127f, -21.65f), Vector3(-17.1409f, 15.7982f, -19.5455f)); 
  data[30].mAabb = Aabb(Vector3(-40.6425f, -9.59291f, -1.18976f), Vector3(-36.463f, -7.56701f, -0.746624f)); 
  data[31].mAabb = Aabb(Vector3(-3.41119f, 38.3851f, -3.49463f), Vector3(-1.09745f, 42.961f, -2.57647f)); 
  data[32].mAabb = Aabb(Vector3(-23.2304f, -33.1432f, 11.3661f), Vector3(-20.5145f, -31.4147f, 14.1034f)); 
  data[33].mAabb = Aabb(Vector3(20.1429f, 5.3285f, -3.18994f), Vector3(20.5441f, 10.9386f, 4.74349f)); 
  data[34].mAabb = Aabb(Vector3(27.861f, 0.691081f, 16.8695f), Vector3(34.6445f, 4.26333f, 17.4629f)); 
  data[35].mAabb = Aabb(Vector3(18.6511f, 29.6776f, 18.2057f), Vector3(24.0812f, 31.9883f, 22.5698f)); 
  data[36].mAabb = Aabb(Vector3(-38.0107f, -8.24228f, -8.44955f), Vector3(-36.2394f, -2.23419f, -6.64678f)); 
  data[37].mAabb = Aabb(Vector3(-21.6991f, 23.7915f, -16.125f), Vector3(-19.2608f, 25.3667f, -12.4391f)); 
  data[38].mAabb = Aabb(Vector3(2.97338f, 6.44027f, -40.7678f), Vector3(7.34699f, 11.5092f, -35.4497f)); 
  data[39].mAabb = Aabb(Vector3(-23.7512f, 15.2898f, 17.5713f), Vector3(-18.1093f, 17.6779f, 19.4163f)); 
  data[40].mAabb = Aabb(Vector3(36.5596f, 1.52517f, -3.57786f), Vector3(37.1122f, 2.45539f, -0.153354f)); 
  data[41].mAabb = Aabb(Vector3(-7.27645f, 42.0798f, 2.6463f), Vector3(-4.87388f, 49.2495f, 5.71362f)); 
  data[42].mAabb = Aabb(Vector3(-29.0556f, -29.205f, -3.29526f), Vector3(-26.0161f, -26.1446f, 0.0445853f)); 
  data[43].mAabb = Aabb(Vector3(-4.46087f, -7.83109f, 35.6082f), Vector3(-0.197439f, -7.11379f, 41.6864f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-7.42144f, 18.2044f, 7.68134f));
  ray.mDirection = Vector3(Vector3(0.913269f, -0.238752f, -0.330058f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(69); 
  data[0].mAabb = Aabb(Vector3(-0.633301f, -13.6376f, -5.99008f), Vector3(7.31776f, -8.5158f, -2.75437f)); 
  data[1].mAabb = Aabb(Vector3(-20.2512f, -13.8741f, 23.5095f), Vector3(-19.3857f, -7.36198f, 29.9465f)); 
  data[2].mAabb = Aabb(Vector3(39.1975f, -25.1166f, 4.95897f), Vector3(39.9999f, -19.3595f, 10.8745f)); 
  data[3].mAabb = Aabb(Vector3(2.60753f, 31.2203f, -7.03217f), Vector3(10.1706f, 33.9145f, -4.96707f)); 
  data[4].mAabb = Aabb(Vector3(-4.2341f, -35.1427f, -31.5406f), Vector3(1.17652f, -30.2266f, -30.3603f)); 
  data[5].mAabb = Aabb(Vector3(19.9964f, -26.0872f, -27.9952f), Vector3(25.5163f, -20.1225f, -20.5289f)); 
  data[6].mAabb = Aabb(Vector3(32.0172f, -5.12944f, -37.3834f), Vector3(39.3887f, 0.39181f, -31.2519f)); 
  data[7].mAabb = Aabb(Vector3(30.4217f, 16.6654f, -19.1555f), Vector3(33.4802f, 18.4673f, -18.4906f)); 
  data[8].mAabb = Aabb(Vector3(0.519228f, 27.9961f, 23.4545f), Vector3(5.4372f, 33.4141f, 31.2525f)); 
  data[9].mAabb = Aabb(Vector3(35.983f, 7.47723f, 20.1071f), Vector3(38.079f, 13.8395f, 20.5841f)); 
  data[10].mAabb = Aabb(Vector3(-8.34156f, -1.79599f, -3.12874f), Vector3(-3.36305f, -1.1455f, -1.36817f)); 
  data[11].mAabb = Aabb(Vector3(30.7281f, 11.8916f, -20.5399f), Vector3(37.4942f, 13.5501f, -15.1226f)); 
  data[12].mAabb = Aabb(Vector3(10.2417f, 6.68129f, -29.9262f), Vector3(13.6977f, 12.3554f, -27.1632f)); 
  data[13].mAabb = Aabb(Vector3(-43.7329f, 23.3423f, -1.57003f), Vector3(-38.5127f, 27.7521f, 1.23612f)); 
  data[14].mAabb = Aabb(Vector3(-20.3426f, -8.03979f, -11.695f), Vector3(-15.7524f, -6.12313f, -7.41739f)); 
  data[15].mAabb = Aabb(Vector3(-15.1073f, -34.1273f, 15.9736f), Vector3(-10.938f, -27.0586f, 20.1081f)); 
  data[16].mAabb = Aabb(Vector3(37.9544f, -21.199f, 14.4262f), Vector3(39.2216f, -17.6959f, 19.0094f)); 
  data[17].mAabb = Aabb(Vector3(-7.05591f, -28.6086f, 12.508f), Vector3(-5.13137f, -24.0652f, 18.2556f)); 
  data[18].mAabb = Aabb(Vector3(41.5836f, -21.5155f, 5.74576f), Vector3(43.1194f, -14.3632f, 13.6702f)); 
  data[19].mAabb = Aabb(Vector3(22.4881f, 14.8545f, 17.7225f), Vector3(24.7952f, 19.5792f, 23.4384f)); 
  data[20].mAabb = Aabb(Vector3(-11.7332f, -43.8674f, 22.3535f), Vector3(-3.73527f, -36.4062f, 26.6592f)); 
  data[21].mAabb = Aabb(Vector3(-30.7912f, -29.3441f, -6.19023f), Vector3(-30.0103f, -28.6131f, 0.158124f)); 
  data[22].mAabb = Aabb(Vector3(-22.0584f, -1.15092f, 38.6051f), Vector3(-17.1302f, 3.03853f, 42.2977f)); 
  data[23].mAabb = Aabb(Vector3(-13.0996f, 12.3481f, 42.6781f), Vector3(-5.91977f, 12.9315f, 46.6954f)); 
  data[24].mAabb = Aabb(Vector3(-15.8071f, 18.988f, 15.4681f), Vector3(-11.6128f, 26.9421f, 22.043f)); 
  data[25].mAabb = Aabb(Vector3(4.16225f, -8.10606f, 41.4404f), Vector3(10.1448f, -7.08145f, 43.5938f)); 
  data[26].mAabb = Aabb(Vector3(-15.7599f, -18.7518f, 16.2958f), Vector3(-8.52668f, -17.4025f, 18.1528f)); 
  data[27].mAabb = Aabb(Vector3(-18.9242f, 19.3207f, 38.9911f), Vector3(-16.8172f, 20.5132f, 42.4637f)); 
  data[28].mAabb = Aabb(Vector3(-17.6609f, 6.03237f, -43.6503f), Vector3(-15.1129f, 12.3357f, -40.4088f)); 
  data[29].mAabb = Aabb(Vector3(25.9096f, 3.13765f, 16.0241f), Vector3(33.4877f, 6.7999f, 18.1258f)); 
  data[30].mAabb = Aabb(Vector3(-3.45217f, -29.4913f, -5.88149f), Vector3(4.53901f, -27.6092f, -3.62271f)); 
  data[31].mAabb = Aabb(Vector3(-35.4367f, -5.46117f, -37.226f), Vector3(-32.3998f, 1.90679f, -29.5642f)); 
  data[32].mAabb = Aabb(Vector3(14.3942f, 0.360886f, -38.8862f), Vector3(18.0847f, 3.31989f, -37.7668f)); 
  data[33].mAabb = Aabb(Vector3(20.0254f, -23.1936f, -34.3513f), Vector3(23.0788f, -22.4753f, -26.3805f)); 
  data[34].mAabb = Aabb(Vector3(33.9943f, -20.158f, 23.6543f), Vector3(34.4811f, -14.2039f, 27.1911f)); 
  data[35].mAabb = Aabb(Vector3(2.03732f, -12.3446f, -40.192f), Vector3(2.83904f, -7.32619f, -38.9591f)); 
  data[36].mAabb = Aabb(Vector3(7.33436f, -26.0826f, -32.249f), Vector3(13.474f, -20.4187f, -27.155f)); 
  data[37].mAabb = Aabb(Vector3(-38.5454f, 22.021f, 4.22578f), Vector3(-33.8805f, 23.7273f, 8.59777f)); 
  data[38].mAabb = Aabb(Vector3(-18.8134f, -33.614f, 26.0874f), Vector3(-13.8442f, -31.4262f, 31.7991f)); 
  data[39].mAabb = Aabb(Vector3(-14.4804f, -32.9651f, 18.3468f), Vector3(-11.3528f, -26.6409f, 19.5197f)); 
  data[40].mAabb = Aabb(Vector3(31.7024f, -4.42283f, -29.3449f), Vector3(33.6959f, -0.0547853f, -27.1655f)); 
  data[41].mAabb = Aabb(Vector3(-38.6212f, -4.84674f, -3.75885f), Vector3(-36.4986f, -3.71752f, -0.862238f)); 
  data[42].mAabb = Aabb(Vector3(38.2326f, 6.99334f, 9.55084f), Vector3(42.2553f, 8.94503f, 15.6689f)); 
  data[43].mAabb = Aabb(Vector3(-21.693f, -15.719f, 7.24304f), Vector3(-20.1017f, -11.2563f, 9.86921f)); 
  data[44].mAabb = Aabb(Vector3(-23.7795f, -46.9374f, -6.36455f), Vector3(-19.8025f, -42.8449f, 0.962585f)); 
  data[45].mAabb = Aabb(Vector3(10.1865f, 27.1583f, 6.61208f), Vector3(12.4089f, 31.1413f, 10.2226f)); 
  data[46].mAabb = Aabb(Vector3(-3.75303f, 4.36287f, -34.5732f), Vector3(-3.26675f, 6.76104f, -30.6853f)); 
  data[47].mAabb = Aabb(Vector3(31.4047f, -29.0121f, -7.13488f), Vector3(39.3335f, -27.5274f, -5.46895f)); 
  data[48].mAabb = Aabb(Vector3(19.8242f, 16.1114f, 6.43622f), Vector3(24.8607f, 17.8546f, 10.2184f)); 
  data[49].mAabb = Aabb(Vector3(-17.6741f, 5.47119f, 0.344917f), Vector3(-16.6456f, 8.27688f, 3.44447f)); 
  data[50].mAabb = Aabb(Vector3(-15.2656f, -30.9582f, 14.849f), Vector3(-7.34054f, -23.5053f, 20.2322f)); 
  data[51].mAabb = Aabb(Vector3(-25.4718f, -37.6803f, 2.47297f), Vector3(-17.7802f, -34.7382f, 6.28784f)); 
  data[52].mAabb = Aabb(Vector3(-11.2466f, 26.0613f, 13.7417f), Vector3(-3.961f, 31.7368f, 19.7349f)); 
  data[53].mAabb = Aabb(Vector3(4.50433f, -21.8038f, 31.6373f), Vector3(7.11241f, -16.7228f, 35.1021f)); 
  data[54].mAabb = Aabb(Vector3(25.0335f, -14.4058f, -2.67463f), Vector3(29.0829f, -6.45198f, 5.1034f)); 
  data[55].mAabb = Aabb(Vector3(25.6575f, -1.83755f, 33.9874f), Vector3(31.7137f, 0.225462f, 37.1013f)); 
  data[56].mAabb = Aabb(Vector3(-22.8289f, 6.13585f, -1.07927f), Vector3(-16.5292f, 12.9437f, 0.789607f)); 
  data[57].mAabb = Aabb(Vector3(-41.1281f, 8.83787f, -24.3959f), Vector3(-40.4163f, 13.4265f, -17.3269f)); 
  data[58].mAabb = Aabb(Vector3(-35.7247f, 16.1726f, -3.09703f), Vector3(-29.0384f, 20.7137f, 3.29492f)); 
  data[59].mAabb = Aabb(Vector3(21.1076f, -13.0684f, -11.3719f), Vector3(27.2894f, -6.38903f, -3.62684f)); 
  data[60].mAabb = Aabb(Vector3(21.5221f, -40.5716f, 8.9144f), Vector3(28.2837f, -32.8854f, 11.3003f)); 
  data[61].mAabb = Aabb(Vector3(31.2885f, 7.22949f, 12.1811f), Vector3(38.2967f, 10.336f, 16.8103f)); 
  data[62].mAabb = Aabb(Vector3(9.81021f, 22.3642f, -10.073f), Vector3(16.1094f, 27.2592f, -7.09965f)); 
  data[63].mAabb = Aabb(Vector3(23.6874f, 14.0377f, 14.3865f), Vector3(27.4042f, 16.54f, 20.1854f)); 
  data[64].mAabb = Aabb(Vector3(-28.4925f, -27.278f, -21.4766f), Vector3(-27.4347f, -24.1808f, -20.5299f)); 
  data[65].mAabb = Aabb(Vector3(39.1975f, -11.5719f, 14.8354f), Vector3(43.4628f, -5.10986f, 17.1004f)); 
  data[66].mAabb = Aabb(Vector3(31.9285f, -18.8879f, -29.7184f), Vector3(37.7863f, -15.9623f, -22.3871f)); 
  data[67].mAabb = Aabb(Vector3(1.01291f, -45.6108f, 8.04429f), Vector3(6.07052f, -40.8596f, 16.0413f)); 
  data[68].mAabb = Aabb(Vector3(9.63929f, -31.6225f, 9.51201f), Vector3(11.6619f, -28.1885f, 11.5998f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-18.4121f, 0.957651f, -13.3985f));
  ray.mDirection = Vector3(Vector3(0.135835f, -0.69452f, 0.706534f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mAabb = Aabb(Vector3(-6.76494f, 39.2078f, 8.13579f), Vector3(-0.550884f, 43.3815f, 12.6973f)); 
  data[1].mAabb = Aabb(Vector3(-19.7567f, 30.7767f, 27.3481f), Vector3(-19.1136f, 31.5849f, 30.0895f)); 
  data[2].mAabb = Aabb(Vector3(-10.6393f, -30.0655f, -34.5597f), Vector3(-8.86178f, -23.6297f, -34.1168f)); 
  data[3].mAabb = Aabb(Vector3(2.68597f, 2.69179f, 35.6426f), Vector3(10.3464f, 9.57685f, 36.6642f)); 
  data[4].mAabb = Aabb(Vector3(6.09661f, 6.30334f, 28.0389f), Vector3(7.99611f, 11.3718f, 29.0983f)); 
  data[5].mAabb = Aabb(Vector3(-15.0978f, 18.2596f, -4.66631f), Vector3(-10.0259f, 21.0831f, -3.20008f)); 
  data[6].mAabb = Aabb(Vector3(7.82814f, 13.6008f, 28.8263f), Vector3(8.47144f, 14.8995f, 31.9098f)); 
  data[7].mAabb = Aabb(Vector3(35.7258f, 12.0025f, -3.13103f), Vector3(40.51f, 19.8422f, 2.69986f)); 
  data[8].mAabb = Aabb(Vector3(-33.7357f, 10.4268f, -16.0914f), Vector3(-27.597f, 13.3273f, -11.5856f)); 
  data[9].mAabb = Aabb(Vector3(-11.3807f, 30.521f, 27.4752f), Vector3(-3.47121f, 34.5209f, 28.9741f)); 
  data[10].mAabb = Aabb(Vector3(-12.5912f, -13.549f, -7.93358f), Vector3(-8.0849f, -7.47993f, -3.07266f)); 
  data[11].mAabb = Aabb(Vector3(-5.55466f, 36.3196f, -30.0174f), Vector3(-1.52757f, 38.4121f, -23.8896f)); 
  data[12].mAabb = Aabb(Vector3(8.57315f, -13.3993f, 16.9746f), Vector3(16.5073f, -7.35012f, 19.7697f)); 
  data[13].mAabb = Aabb(Vector3(-7.6058f, 24.3247f, -0.55044f), Vector3(-1.58008f, 25.0768f, 6.3657f)); 
  data[14].mAabb = Aabb(Vector3(6.8733f, -42.5693f, -22.5261f), Vector3(10.1839f, -40.4926f, -19.4358f)); 
  data[15].mAabb = Aabb(Vector3(1.77209f, -18.3927f, 26.3281f), Vector3(9.00715f, -14.4839f, 32.4411f)); 
  data[16].mAabb = Aabb(Vector3(-5.8213f, -15.643f, 27.9109f), Vector3(-4.18389f, -11.6446f, 31.3676f)); 
  data[17].mAabb = Aabb(Vector3(-3.68548f, -4.23062f, 0.592328f), Vector3(-1.51855f, 2.78827f, 1.16373f)); 
  data[18].mAabb = Aabb(Vector3(-20.8426f, -19.2371f, 37.6942f), Vector3(-16.2178f, -13.1065f, 41.2985f)); 
  data[19].mAabb = Aabb(Vector3(-22.6962f, 24.4458f, 26.748f), Vector3(-19.5233f, 27.589f, 27.3145f)); 
  data[20].mAabb = Aabb(Vector3(-7.11379f, 1.1256f, 4.12235f), Vector3(-2.0258f, 5.24616f, 11.0304f)); 
  data[21].mAabb = Aabb(Vector3(-10.9855f, 1.7853f, 25.2737f), Vector3(-6.6546f, 4.71809f, 32.1604f)); 
  data[22].mAabb = Aabb(Vector3(-8.49279f, 14.8095f, 17.5264f), Vector3(-3.73718f, 19.9838f, 21.5057f)); 
  data[23].mAabb = Aabb(Vector3(8.97488f, -24.5097f, -3.90823f), Vector3(16.2528f, -18.9228f, 1.90317f)); 
  data[24].mAabb = Aabb(Vector3(19.5064f, -15.4497f, 26.3417f), Vector3(26.0076f, -14.4418f, 33.596f)); 
  data[25].mAabb = Aabb(Vector3(-24.5919f, -30.1709f, -36.2533f), Vector3(-17.4976f, -24.129f, -28.8642f)); 
  data[26].mAabb = Aabb(Vector3(-9.81589f, 27.5125f, -33.2362f), Vector3(-7.20387f, 33.4072f, -28.9236f)); 
  data[27].mAabb = Aabb(Vector3(32.9287f, 6.68474f, 24.3534f), Vector3(40.2888f, 12.5462f, 28.58f)); 
  data[28].mAabb = Aabb(Vector3(-13.0289f, -7.49221f, -26.6787f), Vector3(-5.72097f, 0.0645509f, -21.2377f)); 
  data[29].mAabb = Aabb(Vector3(-27.9475f, 21.0921f, -32.781f), Vector3(-27.4596f, 27.7709f, -27.3796f)); 
  data[30].mAabb = Aabb(Vector3(-20.5492f, 2.81666f, 21.5463f), Vector3(-17.4295f, 4.49604f, 29.0952f)); 
  data[31].mAabb = Aabb(Vector3(-22.0168f, -10.3122f, -18.2207f), Vector3(-16.7634f, -3.44667f, -10.7078f)); 
  data[32].mAabb = Aabb(Vector3(-24.5997f, 11.1975f, 1.71126f), Vector3(-23.2555f, 14.5953f, 6.067f)); 
  data[33].mAabb = Aabb(Vector3(13.7916f, -17.8898f, 2.49418f), Vector3(20.217f, -14.8091f, 6.77501f)); 
  data[34].mAabb = Aabb(Vector3(-33.3349f, 4.85964f, -0.0928172f), Vector3(-30.7505f, 6.18462f, 1.66311f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(0.384226f, 7.51933f, 15.52f));
  ray.mDirection = Vector3(Vector3(-0.360584f, -0.505405f, -0.783929f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(36); 
  data[0].mAabb = Aabb(Vector3(-15.8996f, 22.5766f, 16.2296f), Vector3(-9.28641f, 27.8724f, 21.8759f)); 
  data[1].mAabb = Aabb(Vector3(8.49706f, -32.2018f, -28.872f), Vector3(13.1107f, -28.952f, -26.5972f)); 
  data[2].mAabb = Aabb(Vector3(-5.73405f, 30.3346f, -38.9807f), Vector3(-1.16306f, 37.9783f, -33.0944f)); 
  data[3].mAabb = Aabb(Vector3(-11.8829f, -31.099f, -13.5906f), Vector3(-5.89797f, -24.7005f, -9.90565f)); 
  data[4].mAabb = Aabb(Vector3(5.00717f, -33.0318f, -3.74816f), Vector3(7.35269f, -29.5266f, 1.38203f)); 
  data[5].mAabb = Aabb(Vector3(26.8931f, 14.3285f, -31.5715f), Vector3(31.569f, 21.0115f, -28.6341f)); 
  data[6].mAabb = Aabb(Vector3(42.0726f, -10.5843f, -11.3946f), Vector3(46.3692f, -8.69847f, -6.28461f)); 
  data[7].mAabb = Aabb(Vector3(-34.6286f, 20.4169f, -11.0206f), Vector3(-33.4469f, 25.3173f, -8.82582f)); 
  data[8].mAabb = Aabb(Vector3(-27.0989f, -22.9854f, 17.5701f), Vector3(-22.9999f, -17.0812f, 19.8103f)); 
  data[9].mAabb = Aabb(Vector3(19.0657f, -18.1232f, 24.9437f), Vector3(20.5783f, -12.3362f, 31.2927f)); 
  data[10].mAabb = Aabb(Vector3(-1.6212f, -14.7277f, -27.8327f), Vector3(3.204f, -9.93363f, -20.9711f)); 
  data[11].mAabb = Aabb(Vector3(15.0451f, -29.3032f, -8.6517f), Vector3(21.6027f, -25.9251f, -7.26039f)); 
  data[12].mAabb = Aabb(Vector3(8.81508f, 38.1855f, 12.9588f), Vector3(12.4713f, 41.8108f, 15.4882f)); 
  data[13].mAabb = Aabb(Vector3(4.29533f, -10.2385f, -45.6085f), Vector3(5.87567f, -6.27267f, -43.2203f)); 
  data[14].mAabb = Aabb(Vector3(-13.939f, -18.9858f, -11.5263f), Vector3(-8.35812f, -14.9327f, -4.91123f)); 
  data[15].mAabb = Aabb(Vector3(25.6129f, 22.6578f, 15.2338f), Vector3(30.1271f, 24.529f, 22.6526f)); 
  data[16].mAabb = Aabb(Vector3(22.1336f, -3.34587f, -10.2155f), Vector3(27.1812f, 1.86551f, -8.14252f)); 
  data[17].mAabb = Aabb(Vector3(-42.7924f, -16.8607f, -15.6469f), Vector3(-35.2795f, -15.0238f, -14.0053f)); 
  data[18].mAabb = Aabb(Vector3(-32.7385f, 1.70443f, -25.0862f), Vector3(-25.2057f, 8.16713f, -24.6395f)); 
  data[19].mAabb = Aabb(Vector3(-31.0403f, -9.80705f, 17.735f), Vector3(-23.8168f, -6.54745f, 25.1864f)); 
  data[20].mAabb = Aabb(Vector3(20.7356f, 28.3662f, 8.43103f), Vector3(27.9194f, 33.4804f, 14.3964f)); 
  data[21].mAabb = Aabb(Vector3(43.1336f, 14.1231f, -0.698368f), Vector3(46.9768f, 16.1935f, 4.98826f)); 
  data[22].mAabb = Aabb(Vector3(-36.8609f, 22.0494f, -28.0915f), Vector3(-29.1577f, 23.2649f, -24.0867f)); 
  data[23].mAabb = Aabb(Vector3(-14.7317f, 24.4828f, -8.43466f), Vector3(-9.90955f, 25.9597f, -3.3817f)); 
  data[24].mAabb = Aabb(Vector3(21.7213f, -3.54574f, -22.6783f), Vector3(24.3579f, -0.997271f, -20.5139f)); 
  data[25].mAabb = Aabb(Vector3(-36.0471f, -11.8285f, -25.0245f), Vector3(-31.573f, -8.04892f, -23.9007f)); 
  data[26].mAabb = Aabb(Vector3(5.4754f, 46.6597f, 3.492f), Vector3(7.69892f, 50.5831f, 7.5228f)); 
  data[27].mAabb = Aabb(Vector3(-5.82038f, -9.63971f, -4.20084f), Vector3(-0.0616255f, -4.76488f, -0.918508f)); 
  data[28].mAabb = Aabb(Vector3(24.6798f, 0.809724f, -42.34f), Vector3(32.455f, 3.10213f, -37.6531f)); 
  data[29].mAabb = Aabb(Vector3(20.4206f, 10.6396f, -26.6875f), Vector3(22.7183f, 12.2974f, -18.9262f)); 
  data[30].mAabb = Aabb(Vector3(15.868f, 16.2334f, 22.7442f), Vector3(21.4122f, 22.8276f, 24.7731f)); 
  data[31].mAabb = Aabb(Vector3(6.47051f, -17.8757f, -15.6625f), Vector3(8.55718f, -14.339f, -11.7091f)); 
  data[32].mAabb = Aabb(Vector3(-49.6925f, -8.38222f, -10.5312f), Vector3(-47.0471f, -3.34272f, -3.97623f)); 
  data[33].mAabb = Aabb(Vector3(-26.3394f, -35.2808f, -3.99467f), Vector3(-22.9734f, -30.2109f, 2.15027f)); 
  data[34].mAabb = Aabb(Vector3(21.5009f, 27.4335f, -4.66676f), Vector3(23.6922f, 28.19f, 1.02033f)); 
  data[35].mAabb = Aabb(Vector3(-8.07367f, 17.8766f, -35.2271f), Vector3(-3.96192f, 20.0187f, -27.3857f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(5.03522f, 2.3621f, 2.21572f));
  ray.mDirection = Vector3(Vector3(-0.422826f, -0.818086f, -0.389813f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(70); 
  data[0].mAabb = Aabb(Vector3(-26.357f, -20.1498f, 31.9302f), Vector3(-24.287f, -19.5313f, 38.0454f)); 
  data[1].mAabb = Aabb(Vector3(-42.1132f, 15.3063f, -21.0097f), Vector3(-36.8146f, 21.8771f, -13.1398f)); 
  data[2].mAabb = Aabb(Vector3(3.56148f, 17.9015f, -18.8089f), Vector3(5.97983f, 19.0953f, -14.4257f)); 
  data[3].mAabb = Aabb(Vector3(14.8315f, 37.6039f, -2.48128f), Vector3(16.975f, 40.1389f, 1.69425f)); 
  data[4].mAabb = Aabb(Vector3(9.47116f, -13.892f, -5.73099f), Vector3(14.59f, -11.7777f, -5.18417f)); 
  data[5].mAabb = Aabb(Vector3(-0.606211f, 27.7055f, 12.296f), Vector3(5.02196f, 28.4485f, 19.5979f)); 
  data[6].mAabb = Aabb(Vector3(2.11959f, -15.7097f, -35.2816f), Vector3(7.91059f, -12.4026f, -27.4103f)); 
  data[7].mAabb = Aabb(Vector3(-36.1908f, 14.3331f, -33.3838f), Vector3(-32.1845f, 16.0865f, -31.5748f)); 
  data[8].mAabb = Aabb(Vector3(-7.87308f, -3.41452f, 6.65827f), Vector3(-5.93601f, -1.93067f, 10.6179f)); 
  data[9].mAabb = Aabb(Vector3(22.0117f, 34.0977f, -16.3676f), Vector3(27.2556f, 38.1468f, -11.4615f)); 
  data[10].mAabb = Aabb(Vector3(35.2015f, 21.4092f, 16.0239f), Vector3(36.6712f, 23.0218f, 21.6219f)); 
  data[11].mAabb = Aabb(Vector3(10.6055f, 0.640946f, 3.44784f), Vector3(18.3662f, 7.90268f, 9.87667f)); 
  data[12].mAabb = Aabb(Vector3(-31.6893f, -29.1679f, -21.6062f), Vector3(-25.7306f, -23.7457f, -14.3229f)); 
  data[13].mAabb = Aabb(Vector3(39.2774f, 2.87325f, -6.29042f), Vector3(42.5263f, 5.63557f, 1.2232f)); 
  data[14].mAabb = Aabb(Vector3(16.3863f, -17.8879f, 24.8813f), Vector3(17.0487f, -15.5294f, 28.4902f)); 
  data[15].mAabb = Aabb(Vector3(-14.1335f, 13.5917f, 34.6726f), Vector3(-6.59946f, 15.3272f, 42.3048f)); 
  data[16].mAabb = Aabb(Vector3(5.24474f, -46.3122f, -0.38745f), Vector3(7.65196f, -38.9099f, 7.47408f)); 
  data[17].mAabb = Aabb(Vector3(-2.21632f, 33.9596f, -18.5912f), Vector3(3.01802f, 36.6027f, -12.3472f)); 
  data[18].mAabb = Aabb(Vector3(3.14796f, 10.5876f, 24.5026f), Vector3(10.5189f, 14.547f, 30.3003f)); 
  data[19].mAabb = Aabb(Vector3(-16.489f, -37.1196f, -2.72998f), Vector3(-14.7004f, -35.7239f, 2.80495f)); 
  data[20].mAabb = Aabb(Vector3(0.487407f, -42.8235f, -22.9698f), Vector3(4.32987f, -39.383f, -15.2106f)); 
  data[21].mAabb = Aabb(Vector3(15.9357f, -22.7462f, 28.0936f), Vector3(23.7464f, -15.805f, 32.5945f)); 
  data[22].mAabb = Aabb(Vector3(-9.02272f, -28.5491f, -39.9454f), Vector3(-3.01138f, -24.8121f, -38.6555f)); 
  data[23].mAabb = Aabb(Vector3(-0.217185f, 33.0088f, 12.8363f), Vector3(4.71354f, 38.9923f, 14.0766f)); 
  data[24].mAabb = Aabb(Vector3(12.1815f, 28.5679f, 5.74585f), Vector3(20.0368f, 33.5784f, 11.1662f)); 
  data[25].mAabb = Aabb(Vector3(10.8789f, 22.805f, -26.422f), Vector3(17.7718f, 29.5506f, -25.1042f)); 
  data[26].mAabb = Aabb(Vector3(23.3559f, 32.7171f, 2.67896f), Vector3(26.5408f, 34.5243f, 8.65435f)); 
  data[27].mAabb = Aabb(Vector3(-43.2779f, -0.481359f, 21.9023f), Vector3(-36.6118f, 6.20516f, 25.363f)); 
  data[28].mAabb = Aabb(Vector3(13.7419f, -5.25019f, 14.0215f), Vector3(15.0539f, -1.74334f, 17.4973f)); 
  data[29].mAabb = Aabb(Vector3(24.6084f, -25.2363f, -9.41174f), Vector3(31.5399f, -18.7488f, -3.35888f)); 
  data[30].mAabb = Aabb(Vector3(9.1141f, -27.4693f, -39.3132f), Vector3(11.1122f, -25.3478f, -33.1233f)); 
  data[31].mAabb = Aabb(Vector3(-5.93768f, 34.7883f, -22.8436f), Vector3(-5.22734f, 41.2056f, -17.1574f)); 
  data[32].mAabb = Aabb(Vector3(-41.9653f, -8.8082f, 6.68866f), Vector3(-38.4135f, -0.949451f, 8.98941f)); 
  data[33].mAabb = Aabb(Vector3(9.47427f, 32.124f, -9.89964f), Vector3(10.1698f, 33.1154f, -9.06591f)); 
  data[34].mAabb = Aabb(Vector3(22.6033f, 31.9477f, -6.19482f), Vector3(30.3368f, 35.0015f, -4.0446f)); 
  data[35].mAabb = Aabb(Vector3(-48.4106f, -3.71217f, -6.8903f), Vector3(-41.5652f, 3.12557f, -4.58398f)); 
  data[36].mAabb = Aabb(Vector3(-28.5892f, 12.5108f, 11.2316f), Vector3(-24.4589f, 15.8057f, 13.3549f)); 
  data[37].mAabb = Aabb(Vector3(14.6202f, 21.2962f, 33.7326f), Vector3(17.8267f, 24.1731f, 36.487f)); 
  data[38].mAabb = Aabb(Vector3(-1.00137f, -21.3651f, 38.4017f), Vector3(3.06677f, -13.424f, 45.1372f)); 
  data[39].mAabb = Aabb(Vector3(-45.0478f, 4.89056f, 18.9444f), Vector3(-42.3207f, 12.628f, 24.6245f)); 
  data[40].mAabb = Aabb(Vector3(5.32317f, -46.4448f, 1.92851f), Vector3(13.2383f, -40.3873f, 8.01059f)); 
  data[41].mAabb = Aabb(Vector3(19.824f, -43.6579f, 0.986718f), Vector3(22.844f, -37.348f, 6.40545f)); 
  data[42].mAabb = Aabb(Vector3(13.087f, 8.41927f, -3.86234f), Vector3(19.3335f, 10.7903f, 3.35741f)); 
  data[43].mAabb = Aabb(Vector3(13.6971f, -37.3886f, -0.99783f), Vector3(14.741f, -33.8296f, 2.35779f)); 
  data[44].mAabb = Aabb(Vector3(0.647748f, 32.234f, -2.54843f), Vector3(5.77887f, 33.4993f, -1.1156f)); 
  data[45].mAabb = Aabb(Vector3(16.9376f, 17.4376f, -13.6573f), Vector3(22.5968f, 18.5411f, -8.62038f)); 
  data[46].mAabb = Aabb(Vector3(-17.6437f, -31.7134f, -27.9349f), Vector3(-16.4447f, -27.0905f, -23.9788f)); 
  data[47].mAabb = Aabb(Vector3(10.9852f, 3.34705f, 23.4904f), Vector3(17.0098f, 10.8224f, 30.9318f)); 
  data[48].mAabb = Aabb(Vector3(-22.1464f, -7.22735f, -43.641f), Vector3(-18.9891f, -0.0975871f, -41.1325f)); 
  data[49].mAabb = Aabb(Vector3(18.3303f, 9.45204f, -2.24673f), Vector3(20.3272f, 16.2239f, 2.42376f)); 
  data[50].mAabb = Aabb(Vector3(40.1429f, 20.715f, 7.0747f), Vector3(42.7561f, 27.5699f, 11.4829f)); 
  data[51].mAabb = Aabb(Vector3(31.8349f, -19.8689f, 23.5988f), Vector3(33.1295f, -15.6535f, 29.7245f)); 
  data[52].mAabb = Aabb(Vector3(1.45347f, -37.9767f, 28.6932f), Vector3(7.52326f, -30.8606f, 32.118f)); 
  data[53].mAabb = Aabb(Vector3(35.8626f, 2.77905f, 19.4656f), Vector3(41.8521f, 7.70792f, 23.6933f)); 
  data[54].mAabb = Aabb(Vector3(-41.9085f, -1.33182f, 19.9048f), Vector3(-41.3555f, 3.312f, 22.8388f)); 
  data[55].mAabb = Aabb(Vector3(36.5481f, -18.0672f, 6.45046f), Vector3(43.6347f, -11.6249f, 13.7985f)); 
  data[56].mAabb = Aabb(Vector3(-3.56409f, 12.3104f, 41.7388f), Vector3(-0.825893f, 13.1701f, 49.4781f)); 
  data[57].mAabb = Aabb(Vector3(21.823f, -2.67392f, -40.5539f), Vector3(23.9816f, 2.96817f, -33.9453f)); 
  data[58].mAabb = Aabb(Vector3(5.79501f, -20.1268f, -6.56757f), Vector3(9.02585f, -19.2518f, 0.550125f)); 
  data[59].mAabb = Aabb(Vector3(20.7663f, 20.6964f, 25.3755f), Vector3(22.3611f, 22.5739f, 32.7934f)); 
  data[60].mAabb = Aabb(Vector3(26.7418f, 5.9411f, -29.8502f), Vector3(28.2136f, 12.5715f, -23.9915f)); 
  data[61].mAabb = Aabb(Vector3(5.79961f, 37.6173f, -27.3332f), Vector3(6.22466f, 42.15f, -25.333f)); 
  data[62].mAabb = Aabb(Vector3(-38.9877f, 6.11401f, 17.2824f), Vector3(-35.2619f, 12.8942f, 21.5357f)); 
  data[63].mAabb = Aabb(Vector3(17.1315f, -14.4198f, -25.6123f), Vector3(20.0179f, -10.4396f, -19.4259f)); 
  data[64].mAabb = Aabb(Vector3(35.1794f, -33.8312f, -6.87172f), Vector3(39.9927f, -26.0871f, -0.274033f)); 
  data[65].mAabb = Aabb(Vector3(6.16179f, -9.50047f, 24.0544f), Vector3(6.85218f, -4.07246f, 30.3772f)); 
  data[66].mAabb = Aabb(Vector3(21.062f, -29.0912f, -11.9504f), Vector3(21.9173f, -26.9646f, -7.17348f)); 
  data[67].mAabb = Aabb(Vector3(16.2378f, -17.8838f, 20.0212f), Vector3(23.5944f, -16.9126f, 27.9875f)); 
  data[68].mAabb = Aabb(Vector3(29.6004f, 28.3688f, 1.10324f), Vector3(30.0856f, 30.8981f, 2.38832f)); 
  data[69].mAabb = Aabb(Vector3(-29.7108f, -22.6338f, 8.51472f), Vector3(-25.7398f, -20.4713f, 10.3384f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-16.843f, -14.2668f, -3.75203f));
  ray.mDirection = Vector3(Vector3(0.777098f, 0.541208f, 0.321268f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(64); 
  data[0].mAabb = Aabb(Vector3(-10.3802f, 32.5554f, 11.0881f), Vector3(-4.284f, 35.5657f, 13.86f)); 
  data[1].mAabb = Aabb(Vector3(-44.5101f, -3.63181f, -26.0526f), Vector3(-42.7414f, 0.33428f, -21.4568f)); 
  data[2].mAabb = Aabb(Vector3(11.3932f, 3.59261f, -44.6437f), Vector3(14.9149f, 8.44865f, -43.5672f)); 
  data[3].mAabb = Aabb(Vector3(-40.8544f, -4.82195f, -15.0605f), Vector3(-38.8138f, -3.93603f, -7.62787f)); 
  data[4].mAabb = Aabb(Vector3(-36.1421f, -6.57157f, -14.2014f), Vector3(-33.9335f, -2.52222f, -10.0205f)); 
  data[5].mAabb = Aabb(Vector3(32.6873f, 17.0125f, 0.115604f), Vector3(33.5266f, 19.4408f, 1.8481f)); 
  data[6].mAabb = Aabb(Vector3(38.5986f, -20.2546f, -26.9064f), Vector3(39.9122f, -15.5343f, -20.6119f)); 
  data[7].mAabb = Aabb(Vector3(-10.1952f, 4.32791f, 40.3026f), Vector3(-6.49055f, 7.07329f, 47.222f)); 
  data[8].mAabb = Aabb(Vector3(-0.0454417f, 32.5398f, 17.8641f), Vector3(2.51161f, 36.8563f, 21.3132f)); 
  data[9].mAabb = Aabb(Vector3(-12.3672f, -1.13803f, -24.0109f), Vector3(-8.87267f, 1.13396f, -19.1433f)); 
  data[10].mAabb = Aabb(Vector3(2.01075f, 1.73373f, 31.4023f), Vector3(5.58347f, 6.73822f, 39.1439f)); 
  data[11].mAabb = Aabb(Vector3(11.428f, 43.086f, 5.58787f), Vector3(16.7855f, 50.9396f, 8.75446f)); 
  data[12].mAabb = Aabb(Vector3(20.237f, -21.2328f, -41.7371f), Vector3(23.8591f, -18.0553f, -36.0797f)); 
  data[13].mAabb = Aabb(Vector3(25.5585f, -35.2357f, -9.09825f), Vector3(32.0368f, -33.8741f, -6.17659f)); 
  data[14].mAabb = Aabb(Vector3(-27.8424f, 16.8899f, 11.7375f), Vector3(-21.6077f, 20.2736f, 14.4808f)); 
  data[15].mAabb = Aabb(Vector3(-0.179541f, 16.9665f, -27.1062f), Vector3(0.476057f, 17.9769f, -19.4443f)); 
  data[16].mAabb = Aabb(Vector3(11.4528f, 28.0853f, -26.0226f), Vector3(18.5928f, 36.0661f, -21.7358f)); 
  data[17].mAabb = Aabb(Vector3(20.9434f, -38.1306f, 14.6805f), Vector3(26.5208f, -36.7493f, 15.5875f)); 
  data[18].mAabb = Aabb(Vector3(-20.6617f, -29.4064f, -37.108f), Vector3(-15.4111f, -23.3702f, -31.2409f)); 
  data[19].mAabb = Aabb(Vector3(34.1048f, -4.93672f, -26.3167f), Vector3(34.5577f, 2.63836f, -20.1477f)); 
  data[20].mAabb = Aabb(Vector3(-29.3603f, -33.3211f, 17.4009f), Vector3(-27.0997f, -29.3374f, 20.4404f)); 
  data[21].mAabb = Aabb(Vector3(12.0342f, 32.402f, -31.1985f), Vector3(19.5021f, 35.1005f, -26.2629f)); 
  data[22].mAabb = Aabb(Vector3(-17.3152f, 35.4211f, 10.4148f), Vector3(-11.494f, 42.9656f, 13.2319f)); 
  data[23].mAabb = Aabb(Vector3(12.7929f, -42.72f, 13.4124f), Vector3(19.9143f, -39.418f, 14.6244f)); 
  data[24].mAabb = Aabb(Vector3(-21.3008f, -13.4561f, -13.3961f), Vector3(-19.4456f, -8.4247f, -12.7874f)); 
  data[25].mAabb = Aabb(Vector3(6.76748f, -33.7541f, 17.9371f), Vector3(11.2775f, -29.7762f, 18.5127f)); 
  data[26].mAabb = Aabb(Vector3(34.4067f, 25.8917f, -7.04479f), Vector3(36.584f, 31.8379f, -2.96412f)); 
  data[27].mAabb = Aabb(Vector3(-37.5271f, 24.9899f, 19.412f), Vector3(-32.5066f, 28.7226f, 21.719f)); 
  data[28].mAabb = Aabb(Vector3(18.6188f, -19.4269f, -40.5833f), Vector3(22.3931f, -16.846f, -33.7839f)); 
  data[29].mAabb = Aabb(Vector3(-51.1723f, -1.00182f, -6.58377f), Vector3(-47.4794f, 5.54507f, -6.02234f)); 
  data[30].mAabb = Aabb(Vector3(-15.3805f, -35.8678f, -22.9127f), Vector3(-11.465f, -28.5302f, -22.1901f)); 
  data[31].mAabb = Aabb(Vector3(4.96517f, -17.6531f, 42.1081f), Vector3(10.0835f, -11.3201f, 44.0526f)); 
  data[32].mAabb = Aabb(Vector3(-10.929f, 40.3739f, 1.48581f), Vector3(-4.67461f, 43.4528f, 7.88356f)); 
  data[33].mAabb = Aabb(Vector3(-11.7626f, 14.7712f, -45.4921f), Vector3(-8.43134f, 15.4783f, -43.2426f)); 
  data[34].mAabb = Aabb(Vector3(-19.9207f, -36.7605f, 13.7603f), Vector3(-17.4425f, -31.9378f, 18.4916f)); 
  data[35].mAabb = Aabb(Vector3(-2.48162f, 2.17731f, -23.421f), Vector3(4.48044f, 7.95996f, -17.6968f)); 
  data[36].mAabb = Aabb(Vector3(-11.4056f, 34.7424f, 14.0821f), Vector3(-9.92844f, 36.4306f, 21.6748f)); 
  data[37].mAabb = Aabb(Vector3(-19.211f, -27.066f, -20.4752f), Vector3(-18.102f, -24.6645f, -13.6481f)); 
  data[38].mAabb = Aabb(Vector3(-21.0036f, 1.18779f, -10.2275f), Vector3(-14.0156f, 9.00433f, -3.51435f)); 
  data[39].mAabb = Aabb(Vector3(16.2589f, 0.228322f, -6.3599f), Vector3(23.456f, 6.64347f, -3.72422f)); 
  data[40].mAabb = Aabb(Vector3(13.1853f, -21.2506f, -16.4188f), Vector3(13.6625f, -14.8864f, -8.67389f)); 
  data[41].mAabb = Aabb(Vector3(-22.9183f, -9.87579f, -12.897f), Vector3(-19.842f, -6.44989f, -11.4273f)); 
  data[42].mAabb = Aabb(Vector3(-37.4928f, -25.4385f, -21.9465f), Vector3(-32.4626f, -17.7424f, -19.0079f)); 
  data[43].mAabb = Aabb(Vector3(46.1254f, -13.4685f, 1.55901f), Vector3(47.5636f, -12.0635f, 6.2513f)); 
  data[44].mAabb = Aabb(Vector3(-47.126f, 13.9743f, -16.528f), Vector3(-41.3267f, 19.8548f, -10.8198f)); 
  data[45].mAabb = Aabb(Vector3(-7.5323f, 27.0867f, -22.5172f), Vector3(-4.33881f, 29.4812f, -19.2678f)); 
  data[46].mAabb = Aabb(Vector3(5.62183f, -46.1048f, -14.5144f), Vector3(13.6074f, -44.7028f, -13.9769f)); 
  data[47].mAabb = Aabb(Vector3(-25.672f, -3.21011f, -37.0194f), Vector3(-23.3812f, -0.900543f, -33.1055f)); 
  data[48].mAabb = Aabb(Vector3(30.3081f, -8.64522f, 21.3615f), Vector3(38.2135f, -5.23788f, 28.8612f)); 
  data[49].mAabb = Aabb(Vector3(7.86791f, -2.94635f, 37.2563f), Vector3(9.69481f, -0.492975f, 42.4484f)); 
  data[50].mAabb = Aabb(Vector3(-23.2608f, -9.56407f, -24.1004f), Vector3(-17.1479f, -4.97359f, -23.2565f)); 
  data[51].mAabb = Aabb(Vector3(1.59635f, 4.24577f, -44.3595f), Vector3(6.71519f, 9.20062f, -38.0998f)); 
  data[52].mAabb = Aabb(Vector3(4.53344f, 16.165f, -33.6772f), Vector3(9.86775f, 17.8019f, -31.1579f)); 
  data[53].mAabb = Aabb(Vector3(11.3226f, 19.3243f, -43.6156f), Vector3(13.7168f, 25.4688f, -36.7299f)); 
  data[54].mAabb = Aabb(Vector3(-20.4226f, 1.86923f, -18.1493f), Vector3(-19.8403f, 3.00286f, -11.6579f)); 
  data[55].mAabb = Aabb(Vector3(11.7121f, 13.5565f, -44.0486f), Vector3(19.6794f, 19.2336f, -38.3381f)); 
  data[56].mAabb = Aabb(Vector3(4.18695f, -34.0706f, 18.4972f), Vector3(7.19536f, -26.2103f, 20.018f)); 
  data[57].mAabb = Aabb(Vector3(27.0924f, -13.2914f, -0.942442f), Vector3(28.4522f, -5.87147f, -0.505099f)); 
  data[58].mAabb = Aabb(Vector3(37.7666f, 23.3656f, 3.03207f), Vector3(43.8245f, 27.1382f, 9.69122f)); 
  data[59].mAabb = Aabb(Vector3(16.0363f, -43.9897f, -17.8874f), Vector3(18.0675f, -36.879f, -12.9849f)); 
  data[60].mAabb = Aabb(Vector3(8.0487f, 1.07023f, 29.3484f), Vector3(11.9401f, 6.86378f, 36.4805f)); 
  data[61].mAabb = Aabb(Vector3(-25.7294f, -23.0535f, 30.5255f), Vector3(-23.7494f, -19.4474f, 33.5081f)); 
  data[62].mAabb = Aabb(Vector3(-12.4769f, 10.6186f, -25.8955f), Vector3(-11.4465f, 16.1369f, -24.6294f)); 
  data[63].mAabb = Aabb(Vector3(15.4215f, -29.5877f, 32.6534f), Vector3(16.4988f, -23.6603f, 38.9623f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-20.9633f, 3.15491f, -8.89923f));
  ray.mDirection = Vector3(Vector3(-0.843859f, -0.118571f, -0.523301f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(30); 
  data[0].mAabb = Aabb(Vector3(-43.7894f, 5.88923f, -10.4628f), Vector3(-36.1487f, 13.5007f, -9.60699f)); 
  data[1].mAabb = Aabb(Vector3(-17.2579f, -36.2288f, 28.2987f), Vector3(-15.1749f, -33.981f, 30.5361f)); 
  data[2].mAabb = Aabb(Vector3(2.82539f, 17.9903f, -3.37417f), Vector3(7.12014f, 23.8781f, 2.47435f)); 
  data[3].mAabb = Aabb(Vector3(-1.83366f, -14.8627f, -5.62328f), Vector3(1.11073f, -8.15349f, -3.89867f)); 
  data[4].mAabb = Aabb(Vector3(33.6953f, -9.27891f, -29.8259f), Vector3(35.6528f, -5.43738f, -28.7387f)); 
  data[5].mAabb = Aabb(Vector3(-25.267f, 15.7737f, 5.30875f), Vector3(-24.7251f, 17.2444f, 5.97943f)); 
  data[6].mAabb = Aabb(Vector3(-2.39202f, -3.36939f, 24.8369f), Vector3(2.19358f, -0.248728f, 30.5094f)); 
  data[7].mAabb = Aabb(Vector3(-14.8027f, -5.89894f, 2.40418f), Vector3(-8.0526f, -3.67635f, 9.47548f)); 
  data[8].mAabb = Aabb(Vector3(2.78671f, 41.2209f, -17.4363f), Vector3(5.84475f, 48.125f, -14.6158f)); 
  data[9].mAabb = Aabb(Vector3(18.2697f, 8.62567f, -27.2603f), Vector3(25.8191f, 15.7793f, -25.0995f)); 
  data[10].mAabb = Aabb(Vector3(19.9699f, 3.18469f, -25.5849f), Vector3(27.6345f, 4.47047f, -23.2243f)); 
  data[11].mAabb = Aabb(Vector3(30.0917f, -24.4649f, -5.53003f), Vector3(35.4135f, -22.3412f, 2.0156f)); 
  data[12].mAabb = Aabb(Vector3(11.182f, 33.5957f, -25.3627f), Vector3(13.7117f, 37.2466f, -24.3046f)); 
  data[13].mAabb = Aabb(Vector3(31.293f, 27.789f, -25.6218f), Vector3(32.3028f, 29.6207f, -19.2742f)); 
  data[14].mAabb = Aabb(Vector3(-35.3738f, -24.9098f, 3.68067f), Vector3(-28.3855f, -21.9072f, 4.7656f)); 
  data[15].mAabb = Aabb(Vector3(-36.1456f, 0.508579f, 5.00559f), Vector3(-31.8872f, 7.55182f, 7.18179f)); 
  data[16].mAabb = Aabb(Vector3(-26.4957f, 0.717512f, 24.5925f), Vector3(-19.3757f, 1.72125f, 32.1354f)); 
  data[17].mAabb = Aabb(Vector3(-13.1432f, 17.1888f, 24.7967f), Vector3(-9.29563f, 22.1249f, 26.7307f)); 
  data[18].mAabb = Aabb(Vector3(-6.03059f, -31.1421f, -32.4562f), Vector3(-2.76937f, -29.6455f, -26.7024f)); 
  data[19].mAabb = Aabb(Vector3(-24.2705f, 15.5027f, 20.7695f), Vector3(-22.5368f, 22.1379f, 21.8407f)); 
  data[20].mAabb = Aabb(Vector3(-6.58065f, 18.7704f, 7.68712f), Vector3(-1.48014f, 24.0321f, 14.1222f)); 
  data[21].mAabb = Aabb(Vector3(-39.32f, 20.7891f, -4.83932f), Vector3(-34.3139f, 26.5761f, -4.33541f)); 
  data[22].mAabb = Aabb(Vector3(-4.92636f, 21.8922f, -19.6392f), Vector3(2.44949f, 24.0721f, -12.1771f)); 
  data[23].mAabb = Aabb(Vector3(13.4085f, 11.5824f, 25.7316f), Vector3(16.4828f, 15.8308f, 30.3365f)); 
  data[24].mAabb = Aabb(Vector3(11.588f, 25.8169f, 27.6037f), Vector3(17.3693f, 28.1758f, 33.5438f)); 
  data[25].mAabb = Aabb(Vector3(-24.8335f, 0.319021f, -20.1038f), Vector3(-18.0331f, 4.26029f, -13.4213f)); 
  data[26].mAabb = Aabb(Vector3(-4.58131f, -7.6381f, -16.7992f), Vector3(-2.08109f, -0.451985f, -16.3062f)); 
  data[27].mAabb = Aabb(Vector3(-8.55163f, -21.1581f, 12.6621f), Vector3(-6.4675f, -16.6122f, 15.2196f)); 
  data[28].mAabb = Aabb(Vector3(-10.0016f, 20.2865f, -33.8377f), Vector3(-2.22725f, 27.7595f, -30.3548f)); 
  data[29].mAabb = Aabb(Vector3(23.4439f, -30.8841f, 28.2143f), Vector3(24.4621f, -25.9979f, 28.8669f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-11.1176f, -11.4338f, -17.3326f));
  ray.mDirection = Vector3(Vector3(0.300345f, 0.953585f, 0.0216377f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(47); 
  data[0].mAabb = Aabb(Vector3(20.7429f, -24.8812f, -15.7169f), Vector3(25.886f, -18.8131f, -11.7285f)); 
  data[1].mAabb = Aabb(Vector3(5.18115f, 38.487f, -16.409f), Vector3(7.11822f, 39.1609f, -8.86039f)); 
  data[2].mAabb = Aabb(Vector3(11.6859f, 32.2604f, 27.9253f), Vector3(16.843f, 36.1202f, 34.0394f)); 
  data[3].mAabb = Aabb(Vector3(24.3237f, 13.5571f, -35.2135f), Vector3(30.4271f, 19.1679f, -34.4127f)); 
  data[4].mAabb = Aabb(Vector3(-35.4075f, 19.7939f, -11.0878f), Vector3(-30.8096f, 25.5446f, -5.8978f)); 
  data[5].mAabb = Aabb(Vector3(21.6487f, -19.8396f, -4.84485f), Vector3(26.6847f, -12.8405f, -3.91115f)); 
  data[6].mAabb = Aabb(Vector3(-31.2925f, 0.124745f, 8.15669f), Vector3(-27.9756f, 4.39584f, 13.7276f)); 
  data[7].mAabb = Aabb(Vector3(-21.2296f, -20.459f, -33.8491f), Vector3(-16.7915f, -19.4778f, -27.4836f)); 
  data[8].mAabb = Aabb(Vector3(-3.60212f, 21.8909f, -28.3203f), Vector3(0.124376f, 22.6075f, -23.9434f)); 
  data[9].mAabb = Aabb(Vector3(-0.257295f, -24.45f, 9.8303f), Vector3(5.46226f, -23.298f, 12.4706f)); 
  data[10].mAabb = Aabb(Vector3(-41.5019f, 3.91184f, -2.53176f), Vector3(-40.3857f, 10.5564f, -1.28147f)); 
  data[11].mAabb = Aabb(Vector3(12.299f, -18.8401f, -3.04664f), Vector3(14.2599f, -10.9259f, 4.7685f)); 
  data[12].mAabb = Aabb(Vector3(-21.6626f, -35.4191f, -25.348f), Vector3(-21.0161f, -32.0683f, -20.37f)); 
  data[13].mAabb = Aabb(Vector3(9.80723f, 6.82976f, -3.53185f), Vector3(11.1932f, 8.54393f, -2.93633f)); 
  data[14].mAabb = Aabb(Vector3(-32.3166f, -23.946f, -11.5492f), Vector3(-25.6918f, -16.6606f, -8.05492f)); 
  data[15].mAabb = Aabb(Vector3(30.8053f, -8.37106f, 22.9164f), Vector3(33.8543f, -4.74245f, 26.5529f)); 
  data[16].mAabb = Aabb(Vector3(-22.6863f, 0.34306f, 0.2302f), Vector3(-20.8033f, 5.91232f, 4.60868f)); 
  data[17].mAabb = Aabb(Vector3(-36.5189f, -13.0356f, 12.905f), Vector3(-31.2641f, -5.6811f, 18.3174f)); 
  data[18].mAabb = Aabb(Vector3(-42.0967f, -28.597f, 5.62895f), Vector3(-34.828f, -27.8189f, 11.5856f)); 
  data[19].mAabb = Aabb(Vector3(-2.65578f, 17.9102f, 34.0498f), Vector3(-0.835836f, 18.7835f, 34.7283f)); 
  data[20].mAabb = Aabb(Vector3(-0.20493f, -23.3151f, 37.1615f), Vector3(3.25066f, -18.1633f, 43.2823f)); 
  data[21].mAabb = Aabb(Vector3(-4.27037f, -34.5739f, -4.31938f), Vector3(-3.17826f, -31.1239f, 3.40925f)); 
  data[22].mAabb = Aabb(Vector3(8.69127f, 2.42754f, 41.7848f), Vector3(11.3868f, 5.4218f, 49.5656f)); 
  data[23].mAabb = Aabb(Vector3(-43.9425f, 9.49226f, 14.9311f), Vector3(-36.3089f, 11.315f, 15.3965f)); 
  data[24].mAabb = Aabb(Vector3(-30.1092f, 34.3167f, -25.0439f), Vector3(-22.9915f, 36.9424f, -18.5877f)); 
  data[25].mAabb = Aabb(Vector3(30.7161f, 14.3179f, -31.1087f), Vector3(32.2041f, 21.9756f, -30.0129f)); 
  data[26].mAabb = Aabb(Vector3(16.6906f, 5.88565f, -28.5469f), Vector3(21.079f, 12.5358f, -26.0207f)); 
  data[27].mAabb = Aabb(Vector3(42.0221f, -12.4955f, -16.4396f), Vector3(48.9635f, -6.37535f, -10.978f)); 
  data[28].mAabb = Aabb(Vector3(-20.1667f, -21.312f, 17.3711f), Vector3(-16.8197f, -14.6136f, 18.2174f)); 
  data[29].mAabb = Aabb(Vector3(-15.7384f, -36.3491f, -0.946533f), Vector3(-11.2551f, -33.9171f, 1.07288f)); 
  data[30].mAabb = Aabb(Vector3(12.5601f, 3.27053f, 21.2917f), Vector3(14.5885f, 9.48436f, 23.5371f)); 
  data[31].mAabb = Aabb(Vector3(-23.6002f, 16.4994f, -2.78969f), Vector3(-17.7085f, 21.1738f, 4.495f)); 
  data[32].mAabb = Aabb(Vector3(-19.3839f, 20.3475f, -42.151f), Vector3(-14.7004f, 22.9862f, -39.3356f)); 
  data[33].mAabb = Aabb(Vector3(-21.3746f, 23.0656f, 28.3239f), Vector3(-20.8296f, 28.3632f, 34.6701f)); 
  data[34].mAabb = Aabb(Vector3(-2.0859f, -37.0434f, -32.6713f), Vector3(0.925523f, -36.0788f, -27.1621f)); 
  data[35].mAabb = Aabb(Vector3(15.7594f, 4.84685f, 33.0636f), Vector3(17.4376f, 9.65233f, 35.5341f)); 
  data[36].mAabb = Aabb(Vector3(-5.24628f, -19.5485f, 17.5085f), Vector3(2.57188f, -14.3362f, 24.2597f)); 
  data[37].mAabb = Aabb(Vector3(-3.9723f, -37.5389f, -32.0902f), Vector3(3.12197f, -34.2703f, -28.4859f)); 
  data[38].mAabb = Aabb(Vector3(10.8847f, 7.73713f, -2.31269f), Vector3(11.5939f, 14.3367f, -1.72853f)); 
  data[39].mAabb = Aabb(Vector3(22.6487f, -31.13f, -3.71935f), Vector3(26.8917f, -27.991f, 1.6683f)); 
  data[40].mAabb = Aabb(Vector3(-37.6556f, 1.96222f, -19.7833f), Vector3(-31.6225f, 8.12455f, -18.7276f)); 
  data[41].mAabb = Aabb(Vector3(-22.4121f, -33.7279f, 9.23393f), Vector3(-21.0305f, -31.5091f, 12.9266f)); 
  data[42].mAabb = Aabb(Vector3(-8.36576f, -51.4354f, 5.61111f), Vector3(-7.68024f, -44.4443f, 9.37101f)); 
  data[43].mAabb = Aabb(Vector3(32.2659f, -16.8958f, -7.60439f), Vector3(34.193f, -11.843f, -2.50504f)); 
  data[44].mAabb = Aabb(Vector3(27.4021f, 7.92918f, 2.88755f), Vector3(33.7918f, 14.1725f, 4.13645f)); 
  data[45].mAabb = Aabb(Vector3(-37.6008f, 18.7181f, 15.7845f), Vector3(-35.086f, 19.9204f, 23.3034f)); 
  data[46].mAabb = Aabb(Vector3(1.18509f, -6.39159f, -8.78758f), Vector3(7.24514f, -0.69523f, -5.1636f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(5.84429f, -4.02051f, -10.6675f));
  ray.mDirection = Vector3(Vector3(0.308268f, 0.781558f, 0.542344f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(67); 
  data[0].mAabb = Aabb(Vector3(17.0159f, -6.37258f, -10.8176f), Vector3(17.6602f, -1.96464f, -5.10038f)); 
  data[1].mAabb = Aabb(Vector3(-5.97136f, 43.2443f, 7.19172f), Vector3(-2.7048f, 50.6048f, 13.1196f)); 
  data[2].mAabb = Aabb(Vector3(-9.29759f, -35.4767f, 32.6488f), Vector3(-1.74384f, -29.8831f, 34.6044f)); 
  data[3].mAabb = Aabb(Vector3(28.9871f, -33.0143f, -2.31427f), Vector3(31.1f, -25.6523f, 0.317232f)); 
  data[4].mAabb = Aabb(Vector3(40.4006f, -20.1058f, 7.81375f), Vector3(43.9197f, -18.8402f, 8.62011f)); 
  data[5].mAabb = Aabb(Vector3(-36.6996f, -21.8014f, -15.8141f), Vector3(-32.7817f, -20.61f, -15.3369f)); 
  data[6].mAabb = Aabb(Vector3(-20.0785f, -46.2573f, 8.28771f), Vector3(-18.1066f, -39.4954f, 14.2538f)); 
  data[7].mAabb = Aabb(Vector3(-38.7932f, 3.96228f, 16.0766f), Vector3(-33.7085f, 4.75449f, 18.0501f)); 
  data[8].mAabb = Aabb(Vector3(-31.6813f, 4.92282f, -23.8433f), Vector3(-27.7792f, 6.57345f, -18.1362f)); 
  data[9].mAabb = Aabb(Vector3(-26.1545f, 2.15551f, -10.6363f), Vector3(-20.1926f, 8.96427f, -6.71496f)); 
  data[10].mAabb = Aabb(Vector3(15.0504f, 18.5319f, -35.4207f), Vector3(16.4418f, 20.3198f, -32.6563f)); 
  data[11].mAabb = Aabb(Vector3(-3.97552f, -10.8396f, -18.9353f), Vector3(-0.490013f, -3.93434f, -13.6251f)); 
  data[12].mAabb = Aabb(Vector3(-39.7872f, 0.361947f, -0.359546f), Vector3(-34.4033f, 5.81222f, 3.62301f)); 
  data[13].mAabb = Aabb(Vector3(31.1858f, 10.8512f, -8.43317f), Vector3(38.656f, 12.7955f, -0.80404f)); 
  data[14].mAabb = Aabb(Vector3(-3.27435f, -27.4165f, -7.30442f), Vector3(2.98586f, -23.1535f, -1.72426f)); 
  data[15].mAabb = Aabb(Vector3(17.5785f, -0.709031f, -41.0488f), Vector3(23.7866f, 1.13341f, -33.6945f)); 
  data[16].mAabb = Aabb(Vector3(21.703f, -11.2205f, -22.79f), Vector3(29.2799f, -8.58693f, -16.688f)); 
  data[17].mAabb = Aabb(Vector3(-8.95236f, 19.9946f, -0.137752f), Vector3(-8.11539f, 25.1046f, 6.84009f)); 
  data[18].mAabb = Aabb(Vector3(-8.89265f, 27.1978f, 7.13438f), Vector3(-5.35983f, 28.0012f, 14.0489f)); 
  data[19].mAabb = Aabb(Vector3(0.573606f, -4.76855f, -16.5939f), Vector3(4.32028f, -2.07836f, -10.2562f)); 
  data[20].mAabb = Aabb(Vector3(-16.8259f, -29.5182f, -7.88726f), Vector3(-14.3011f, -22.6897f, -5.97732f)); 
  data[21].mAabb = Aabb(Vector3(8.4574f, -24.5261f, -1.84947f), Vector3(14.9329f, -19.6134f, -1.23747f)); 
  data[22].mAabb = Aabb(Vector3(-12.8276f, 20.6718f, -32.9798f), Vector3(-7.47153f, 22.8759f, -25.4947f)); 
  data[23].mAabb = Aabb(Vector3(28.7397f, -22.9785f, -34.7853f), Vector3(31.1091f, -20.0116f, -27.2413f)); 
  data[24].mAabb = Aabb(Vector3(12.7875f, -35.4099f, 22.8869f), Vector3(15.2471f, -32.9635f, 28.134f)); 
  data[25].mAabb = Aabb(Vector3(-26.0818f, 41.8772f, 2.24454f), Vector3(-25.0198f, 42.3941f, 8.09538f)); 
  data[26].mAabb = Aabb(Vector3(-20.215f, 8.98696f, -7.68357f), Vector3(-15.1157f, 9.52543f, -7.1611f)); 
  data[27].mAabb = Aabb(Vector3(6.03501f, -37.1072f, -36.1748f), Vector3(12.0712f, -33.0446f, -31.1833f)); 
  data[28].mAabb = Aabb(Vector3(8.32411f, -6.70815f, -45.4118f), Vector3(16.0335f, -3.74729f, -44.6922f)); 
  data[29].mAabb = Aabb(Vector3(5.35238f, -32.2302f, 31.0696f), Vector3(11.6852f, -30.969f, 37.7144f)); 
  data[30].mAabb = Aabb(Vector3(33.1284f, 23.733f, 11.5466f), Vector3(35.8668f, 30.7444f, 14.1612f)); 
  data[31].mAabb = Aabb(Vector3(-49.6086f, -15.788f, -0.124272f), Vector3(-42.7391f, -9.95963f, 1.94153f)); 
  data[32].mAabb = Aabb(Vector3(-6.94853f, -12.6967f, -19.5238f), Vector3(-0.99216f, -5.16339f, -14.103f)); 
  data[33].mAabb = Aabb(Vector3(16.2787f, 15.2968f, -22.4106f), Vector3(20.3067f, 15.9162f, -21.1262f)); 
  data[34].mAabb = Aabb(Vector3(-13.9698f, 36.5655f, 9.06435f), Vector3(-13.1808f, 39.9251f, 10.9666f)); 
  data[35].mAabb = Aabb(Vector3(-32.1846f, -29.1862f, -15.6778f), Vector3(-25.2084f, -27.2707f, -12.9754f)); 
  data[36].mAabb = Aabb(Vector3(25.2861f, 22.5974f, 26.7515f), Vector3(28.1305f, 30.5867f, 29.1515f)); 
  data[37].mAabb = Aabb(Vector3(-25.4267f, -5.60951f, -25.1517f), Vector3(-21.6392f, -5.09262f, -19.6928f)); 
  data[38].mAabb = Aabb(Vector3(-44.6735f, -7.52731f, 13.9687f), Vector3(-43.4851f, -3.12332f, 19.993f)); 
  data[39].mAabb = Aabb(Vector3(-19.4516f, -14.3134f, -38.0663f), Vector3(-16.2354f, -11.6492f, -34.5462f)); 
  data[40].mAabb = Aabb(Vector3(-16.4667f, -36.9477f, 11.6112f), Vector3(-8.52884f, -36.403f, 15.8958f)); 
  data[41].mAabb = Aabb(Vector3(-25.8882f, -17.3798f, 22.5011f), Vector3(-20.2915f, -14.9862f, 25.5631f)); 
  data[42].mAabb = Aabb(Vector3(-29.1754f, -17.5106f, 24.1894f), Vector3(-26.0466f, -13.7595f, 31.6023f)); 
  data[43].mAabb = Aabb(Vector3(-35.2754f, 12.4938f, -7.10969f), Vector3(-31.9518f, 14.4367f, -5.00678f)); 
  data[44].mAabb = Aabb(Vector3(4.97467f, -37.4992f, 0.0449784f), Vector3(11.4019f, -35.2146f, 3.89347f)); 
  data[45].mAabb = Aabb(Vector3(-1.85349f, -17.2438f, 15.7257f), Vector3(-0.672773f, -14.2256f, 20.0778f)); 
  data[46].mAabb = Aabb(Vector3(38.4061f, 6.00494f, 3.32924f), Vector3(40.2458f, 13.8493f, 6.06882f)); 
  data[47].mAabb = Aabb(Vector3(-43.6164f, 20.2413f, -20.618f), Vector3(-38.6398f, 22.1777f, -14.3836f)); 
  data[48].mAabb = Aabb(Vector3(33.338f, -24.6467f, 3.86234f), Vector3(41.1277f, -20.6602f, 5.3963f)); 
  data[49].mAabb = Aabb(Vector3(-49.9327f, -10.0686f, -17.0862f), Vector3(-43.1812f, -7.41622f, -10.8288f)); 
  data[50].mAabb = Aabb(Vector3(16.8067f, 28.1028f, 25.8965f), Vector3(20.3282f, 34.4681f, 31.6065f)); 
  data[51].mAabb = Aabb(Vector3(-14.2118f, -34.1439f, -32.9298f), Vector3(-7.99353f, -29.457f, -26.8514f)); 
  data[52].mAabb = Aabb(Vector3(-38.3676f, -20.09f, -18.4384f), Vector3(-34.1733f, -16.3904f, -14.9174f)); 
  data[53].mAabb = Aabb(Vector3(-37.3211f, -9.85241f, -13.2192f), Vector3(-31.8207f, -7.08731f, -6.38286f)); 
  data[54].mAabb = Aabb(Vector3(-16.2427f, -10.3345f, -24.0774f), Vector3(-13.0151f, -9.74774f, -16.4689f)); 
  data[55].mAabb = Aabb(Vector3(-6.63654f, 2.57056f, -18.1469f), Vector3(-5.0105f, 3.35118f, -17.6335f)); 
  data[56].mAabb = Aabb(Vector3(-11.4149f, -21.1321f, 30.4181f), Vector3(-5.55265f, -14.0624f, 35.9663f)); 
  data[57].mAabb = Aabb(Vector3(2.0282f, -15.1836f, 15.029f), Vector3(7.07444f, -14.288f, 20.5126f)); 
  data[58].mAabb = Aabb(Vector3(-7.2901f, -7.44295f, 4.89227f), Vector3(-5.5924f, -1.37038f, 12.0872f)); 
  data[59].mAabb = Aabb(Vector3(-19.4286f, -13.4712f, -33.207f), Vector3(-17.1951f, -12.682f, -31.5916f)); 
  data[60].mAabb = Aabb(Vector3(-34.8605f, -35.913f, -7.50124f), Vector3(-33.4885f, -33.8964f, -7.0189f)); 
  data[61].mAabb = Aabb(Vector3(4.51564f, -6.60328f, 1.72136f), Vector3(9.14508f, -1.13398f, 5.81038f)); 
  data[62].mAabb = Aabb(Vector3(39.56f, -20.8786f, 9.46554f), Vector3(47.5006f, -13.9485f, 13.0141f)); 
  data[63].mAabb = Aabb(Vector3(-29.9056f, 8.8639f, -30.0089f), Vector3(-26.7065f, 11.921f, -27.9192f)); 
  data[64].mAabb = Aabb(Vector3(30.1628f, -1.41332f, -37.9777f), Vector3(32.6835f, 5.14749f, -35.9379f)); 
  data[65].mAabb = Aabb(Vector3(18.0787f, -43.8721f, 13.8478f), Vector3(19.3355f, -41.7124f, 15.3919f)); 
  data[66].mAabb = Aabb(Vector3(-39.5867f, 15.3389f, 9.84753f), Vector3(-34.8206f, 21.35f, 14.9125f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-1.98501f, -6.41595f, -17.1503f));
  ray.mDirection = Vector3(Vector3(0.594798f, 0.623928f, 0.506882f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(51); 
  data[0].mAabb = Aabb(Vector3(12.6381f, 1.70129f, 6.86413f), Vector3(19.885f, 9.30328f, 11.4098f)); 
  data[1].mAabb = Aabb(Vector3(-21.1027f, -31.2922f, -36.3252f), Vector3(-19.8225f, -28.8805f, -31.3193f)); 
  data[2].mAabb = Aabb(Vector3(-9.75972f, -17.1114f, -16.8773f), Vector3(-3.40093f, -9.26282f, -12.3033f)); 
  data[3].mAabb = Aabb(Vector3(-10.2384f, -11.114f, 25.2363f), Vector3(-3.70724f, -5.69482f, 31.7241f)); 
  data[4].mAabb = Aabb(Vector3(13.0675f, -7.05158f, -50.1791f), Vector3(20.6423f, -4.32568f, -42.5133f)); 
  data[5].mAabb = Aabb(Vector3(-32.8701f, 8.86869f, -22.4135f), Vector3(-27.6358f, 11.9474f, -19.1425f)); 
  data[6].mAabb = Aabb(Vector3(14.3777f, 30.2111f, -28.5498f), Vector3(21.0246f, 31.2526f, -25.5388f)); 
  data[7].mAabb = Aabb(Vector3(21.5312f, 13.6084f, 6.9186f), Vector3(29.1942f, 18.685f, 7.50137f)); 
  data[8].mAabb = Aabb(Vector3(-19.4215f, 19.948f, -1.37842f), Vector3(-13.1578f, 24.3367f, 4.87368f)); 
  data[9].mAabb = Aabb(Vector3(-28.8402f, 37.6634f, -0.271754f), Vector3(-27.9502f, 38.8835f, 4.64831f)); 
  data[10].mAabb = Aabb(Vector3(31.7537f, 14.5648f, -28.1661f), Vector3(35.5544f, 15.713f, -27.6416f)); 
  data[11].mAabb = Aabb(Vector3(-9.46513f, 6.22569f, 46.0052f), Vector3(-6.36047f, 12.7675f, 47.7964f)); 
  data[12].mAabb = Aabb(Vector3(-9.77218f, -8.92264f, -43.5644f), Vector3(-8.54253f, -8.51522f, -42.6049f)); 
  data[13].mAabb = Aabb(Vector3(-20.0072f, -12.4425f, 16.2725f), Vector3(-17.7862f, -7.32341f, 17.9726f)); 
  data[14].mAabb = Aabb(Vector3(5.4438f, -15.5323f, 1.57176f), Vector3(7.24913f, -8.77986f, 9.31894f)); 
  data[15].mAabb = Aabb(Vector3(-21.5886f, -31.1912f, 19.9345f), Vector3(-17.8962f, -24.1071f, 24.9097f)); 
  data[16].mAabb = Aabb(Vector3(27.7313f, 12.3195f, 31.6374f), Vector3(35.2684f, 20.2226f, 35.9231f)); 
  data[17].mAabb = Aabb(Vector3(10.3026f, 12.9255f, -23.613f), Vector3(16.1474f, 16.7988f, -17.7508f)); 
  data[18].mAabb = Aabb(Vector3(32.4231f, -20.1392f, 18.2448f), Vector3(37.282f, -14.7896f, 19.3669f)); 
  data[19].mAabb = Aabb(Vector3(-43.1255f, -1.69578f, -3.09922f), Vector3(-37.0868f, 4.09591f, -2.60922f)); 
  data[20].mAabb = Aabb(Vector3(28.1492f, -20.4864f, -19.3504f), Vector3(33.863f, -12.5312f, -18.3539f)); 
  data[21].mAabb = Aabb(Vector3(-17.4274f, 2.60269f, 30.301f), Vector3(-9.45172f, 10.3448f, 35.4261f)); 
  data[22].mAabb = Aabb(Vector3(19.7281f, -22.5082f, -0.234758f), Vector3(21.9878f, -19.0262f, 7.07313f)); 
  data[23].mAabb = Aabb(Vector3(12.1689f, -12.3418f, -12.3849f), Vector3(16.4284f, -5.94894f, -8.66309f)); 
  data[24].mAabb = Aabb(Vector3(-19.3595f, 41.8162f, -9.27572f), Vector3(-18.1449f, 44.6288f, -4.7706f)); 
  data[25].mAabb = Aabb(Vector3(22.8148f, -7.3585f, 26.9992f), Vector3(26.1405f, -1.51879f, 32.9882f)); 
  data[26].mAabb = Aabb(Vector3(-9.53874f, 10.9751f, 34.3136f), Vector3(-5.89065f, 15.5943f, 36.6408f)); 
  data[27].mAabb = Aabb(Vector3(-26.2146f, -46.6287f, 3.57105f), Vector3(-19.7633f, -40.8032f, 9.34767f)); 
  data[28].mAabb = Aabb(Vector3(-14.052f, -5.94801f, 39.8718f), Vector3(-9.01503f, -0.297803f, 43.8666f)); 
  data[29].mAabb = Aabb(Vector3(2.12568f, 22.8833f, -31.8795f), Vector3(6.08272f, 23.666f, -26.0783f)); 
  data[30].mAabb = Aabb(Vector3(-14.1596f, 14.5094f, -33.6331f), Vector3(-11.1663f, 17.7019f, -28.1629f)); 
  data[31].mAabb = Aabb(Vector3(-7.00227f, -45.7648f, -10.3235f), Vector3(-4.28565f, -39.2067f, -5.40299f)); 
  data[32].mAabb = Aabb(Vector3(-17.5423f, 27.5911f, -17.4993f), Vector3(-13.6604f, 28.4669f, -10.0001f)); 
  data[33].mAabb = Aabb(Vector3(-33.0829f, -14.0693f, -11.173f), Vector3(-31.0007f, -7.78542f, -3.60439f)); 
  data[34].mAabb = Aabb(Vector3(1.11862f, -17.0508f, -35.8353f), Vector3(1.65175f, -14.6742f, -32.6908f)); 
  data[35].mAabb = Aabb(Vector3(-4.96363f, -10.9533f, -19.6376f), Vector3(1.37799f, -3.89428f, -14.8706f)); 
  data[36].mAabb = Aabb(Vector3(-11.1751f, 42.4187f, -17.1163f), Vector3(-7.70379f, 46.6081f, -10.9897f)); 
  data[37].mAabb = Aabb(Vector3(-23.4278f, 30.4891f, 15.6662f), Vector3(-17.4107f, 38.1871f, 17.1879f)); 
  data[38].mAabb = Aabb(Vector3(-18.0368f, -19.3636f, -38.1448f), Vector3(-11.3728f, -16.9401f, -31.4662f)); 
  data[39].mAabb = Aabb(Vector3(-39.5879f, -29.3821f, -1.4118f), Vector3(-38.6662f, -28.5644f, 0.606449f)); 
  data[40].mAabb = Aabb(Vector3(-27.2154f, 10.8476f, -30.304f), Vector3(-26.6055f, 12.8378f, -26.4666f)); 
  data[41].mAabb = Aabb(Vector3(15.6573f, 25.6829f, -6.83402f), Vector3(20.5389f, 32.7999f, -2.20365f)); 
  data[42].mAabb = Aabb(Vector3(6.18186f, -4.93307f, -3.30266f), Vector3(11.1594f, 0.210814f, -2.43112f)); 
  data[43].mAabb = Aabb(Vector3(-27.2611f, -2.793f, 5.91702f), Vector3(-21.022f, 3.92274f, 7.66877f)); 
  data[44].mAabb = Aabb(Vector3(4.51427f, -6.08036f, 7.2496f), Vector3(10.1151f, -0.345953f, 13.62f)); 
  data[45].mAabb = Aabb(Vector3(11.0778f, 29.3285f, -8.69089f), Vector3(17.5286f, 30.3439f, -2.62504f)); 
  data[46].mAabb = Aabb(Vector3(-14.5985f, 29.9946f, -15.3677f), Vector3(-10.9453f, 34.1358f, -11.6721f)); 
  data[47].mAabb = Aabb(Vector3(-14.7577f, -2.36194f, 9.68246f), Vector3(-10.2693f, -0.817547f, 16.7443f)); 
  data[48].mAabb = Aabb(Vector3(19.8024f, -25.1323f, 8.55588f), Vector3(20.8099f, -24.289f, 15.8751f)); 
  data[49].mAabb = Aabb(Vector3(-42.5884f, -16.9923f, -20.7036f), Vector3(-38.8299f, -12.5463f, -19.6876f)); 
  data[50].mAabb = Aabb(Vector3(5.88897f, -34.335f, 31.2323f), Vector3(13.8832f, -27.9405f, 32.7804f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(3.60748f, -15.4099f, 6.42362f));
  ray.mDirection = Vector3(Vector3(0.373409f, 0.925333f, 0.0657674f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(31); 
  data[0].mAabb = Aabb(Vector3(-44.5305f, 8.69931f, -1.75764f), Vector3(-39.2069f, 12.8405f, 2.14026f)); 
  data[1].mAabb = Aabb(Vector3(24.8004f, -36.7211f, 18.0076f), Vector3(27.3277f, -32.3855f, 22.624f)); 
  data[2].mAabb = Aabb(Vector3(-7.59991f, -19.4231f, 25.1796f), Vector3(0.382929f, -12.3228f, 30.426f)); 
  data[3].mAabb = Aabb(Vector3(11.7037f, -31.5372f, -9.96591f), Vector3(18.6655f, -28.2718f, -3.28658f)); 
  data[4].mAabb = Aabb(Vector3(8.57235f, -18.0692f, 41.5446f), Vector3(9.15349f, -17.2923f, 45.461f)); 
  data[5].mAabb = Aabb(Vector3(15.8685f, -41.9883f, 8.00982f), Vector3(21.3846f, -34.513f, 8.77931f)); 
  data[6].mAabb = Aabb(Vector3(14.6516f, 6.72239f, -6.14144f), Vector3(15.3694f, 13.5676f, -2.05451f)); 
  data[7].mAabb = Aabb(Vector3(-36.9502f, -1.00525f, 6.69621f), Vector3(-34.5552f, 3.2684f, 12.8915f)); 
  data[8].mAabb = Aabb(Vector3(11.731f, -8.14411f, -18.6336f), Vector3(18.2062f, -3.50006f, -12.2632f)); 
  data[9].mAabb = Aabb(Vector3(-7.60724f, -22.9253f, -6.34968f), Vector3(0.0181766f, -16.377f, -2.76235f)); 
  data[10].mAabb = Aabb(Vector3(-39.8291f, -14.4343f, -2.63944f), Vector3(-36.6509f, -9.47292f, 3.28261f)); 
  data[11].mAabb = Aabb(Vector3(-21.9467f, 7.73797f, 15.665f), Vector3(-20.1625f, 9.57253f, 18.8054f)); 
  data[12].mAabb = Aabb(Vector3(-40.1261f, -18.6156f, 4.7771f), Vector3(-32.733f, -14.1341f, 11.3152f)); 
  data[13].mAabb = Aabb(Vector3(-9.32457f, -28.33f, -12.5324f), Vector3(-6.87653f, -21.7147f, -10.1632f)); 
  data[14].mAabb = Aabb(Vector3(-7.88731f, 15.9743f, -17.3378f), Vector3(-7.20574f, 20.947f, -9.58758f)); 
  data[15].mAabb = Aabb(Vector3(-30.2269f, -32.6305f, -22.6728f), Vector3(-27.0454f, -31.2243f, -17.5756f)); 
  data[16].mAabb = Aabb(Vector3(-11.4f, 2.61283f, -45.7694f), Vector3(-6.77383f, 6.47872f, -44.0343f)); 
  data[17].mAabb = Aabb(Vector3(-37.48f, -15.0918f, -29.5299f), Vector3(-37.0064f, -10.2947f, -25.943f)); 
  data[18].mAabb = Aabb(Vector3(-15.8624f, -1.1525f, -25.2649f), Vector3(-10.6309f, 2.34206f, -19.819f)); 
  data[19].mAabb = Aabb(Vector3(37.7485f, -19.0487f, -12.2853f), Vector3(38.7847f, -17.8439f, -7.95902f)); 
  data[20].mAabb = Aabb(Vector3(-12.84f, -7.04042f, -11.2504f), Vector3(-11.8177f, -0.584449f, -6.07009f)); 
  data[21].mAabb = Aabb(Vector3(-20.9362f, 5.122f, -35.0076f), Vector3(-16.4587f, 8.42868f, -28.0233f)); 
  data[22].mAabb = Aabb(Vector3(17.1318f, -6.54186f, 17.8997f), Vector3(21.6766f, -0.188406f, 24.0996f)); 
  data[23].mAabb = Aabb(Vector3(1.09348f, -6.90866f, -18.7289f), Vector3(6.8534f, -4.9978f, -16.8855f)); 
  data[24].mAabb = Aabb(Vector3(-7.04167f, -42.2249f, 13.1367f), Vector3(-1.45501f, -40.0365f, 16.9451f)); 
  data[25].mAabb = Aabb(Vector3(1.58884f, -13.525f, 46.9642f), Vector3(6.44883f, -12.8933f, 48.5941f)); 
  data[26].mAabb = Aabb(Vector3(-17.0833f, 2.21429f, -45.127f), Vector3(-10.0559f, 9.87218f, -37.4197f)); 
  data[27].mAabb = Aabb(Vector3(-40.5284f, -3.13973f, -31.561f), Vector3(-33.2896f, -1.01201f, -25.715f)); 
  data[28].mAabb = Aabb(Vector3(26.6673f, 34.6523f, 3.65802f), Vector3(31.4999f, 42.2761f, 8.09262f)); 
  data[29].mAabb = Aabb(Vector3(-6.87982f, 30.4583f, 22.8396f), Vector3(-4.34434f, 36.9268f, 27.5593f)); 
  data[30].mAabb = Aabb(Vector3(-44.4088f, -6.48042f, 1.06728f), Vector3(-36.5074f, 1.33774f, 5.97412f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-11.6497f, -8.94538f, 4.60617f));
  ray.mDirection = Vector3(Vector3(-0.0405561f, 0.270982f, -0.96173f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeRayCastFuzzTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(32); 
  data[0].mAabb = Aabb(Vector3(14.441f, -42.1771f, 13.9863f), Vector3(15.2578f, -38.702f, 17.5446f)); 
  data[1].mAabb = Aabb(Vector3(20.2581f, 3.11519f, -16.6284f), Vector3(26.1627f, 5.98466f, -8.71259f)); 
  data[2].mAabb = Aabb(Vector3(16.0549f, -2.43701f, 6.99464f), Vector3(22.361f, 5.44493f, 8.31846f)); 
  data[3].mAabb = Aabb(Vector3(11.2492f, -30.0133f, -32.3168f), Vector3(15.8495f, -24.7787f, -28.6481f)); 
  data[4].mAabb = Aabb(Vector3(7.41272f, -1.65364f, -37.0763f), Vector3(11.666f, 5.35945f, -29.4154f)); 
  data[5].mAabb = Aabb(Vector3(-29.6147f, 8.924f, 24.4293f), Vector3(-23.6386f, 16.2263f, 30.2514f)); 
  data[6].mAabb = Aabb(Vector3(20.0948f, 2.26499f, -38.6752f), Vector3(21.1167f, 9.3915f, -31.1277f)); 
  data[7].mAabb = Aabb(Vector3(1.55885f, 20.4463f, 37.0787f), Vector3(2.74792f, 22.1695f, 42.1335f)); 
  data[8].mAabb = Aabb(Vector3(-11.474f, -0.918088f, -32.6192f), Vector3(-4.40456f, 2.29164f, -31.0758f)); 
  data[9].mAabb = Aabb(Vector3(-31.6562f, -7.57104f, -14.8548f), Vector3(-27.5685f, -4.65913f, -8.00264f)); 
  data[10].mAabb = Aabb(Vector3(-39.2872f, -8.70581f, -24.2169f), Vector3(-33.4551f, -5.73498f, -19.1449f)); 
  data[11].mAabb = Aabb(Vector3(-4.69233f, -16.6153f, -32.5336f), Vector3(-0.472265f, -11.9515f, -30.6295f)); 
  data[12].mAabb = Aabb(Vector3(16.6095f, -21.9439f, -37.8486f), Vector3(23.788f, -19.6107f, -34.6694f)); 
  data[13].mAabb = Aabb(Vector3(-16.8636f, -15.1715f, -1.45289f), Vector3(-9.24441f, -13.9147f, 2.99401f)); 
  data[14].mAabb = Aabb(Vector3(7.74027f, 8.96107f, -45.9944f), Vector3(10.5262f, 13.311f, -42.9486f)); 
  data[15].mAabb = Aabb(Vector3(-35.7622f, -32.148f, -7.79653f), Vector3(-28.346f, -25.8918f, -1.49109f)); 
  data[16].mAabb = Aabb(Vector3(7.72905f, -31.273f, 25.6995f), Vector3(9.54064f, -28.8257f, 29.2024f)); 
  data[17].mAabb = Aabb(Vector3(-35.2569f, 6.07674f, -3.2701f), Vector3(-31.5018f, 13.1033f, -1.06908f)); 
  data[18].mAabb = Aabb(Vector3(22.7275f, -39.9878f, -0.803077f), Vector3(24.9387f, -35.2746f, 0.218524f)); 
  data[19].mAabb = Aabb(Vector3(-5.19809f, -10.654f, -46.72f), Vector3(1.06259f, -8.6831f, -44.2335f)); 
  data[20].mAabb = Aabb(Vector3(-0.174083f, 11.8988f, 32.9058f), Vector3(7.50259f, 16.8367f, 36.3398f)); 
  data[21].mAabb = Aabb(Vector3(-29.0276f, -31.9288f, -22.8963f), Vector3(-27.2956f, -26.6221f, -16.4252f)); 
  data[22].mAabb = Aabb(Vector3(20.5306f, 33.3173f, -23.5974f), Vector3(26.8778f, 40.8439f, -19.9452f)); 
  data[23].mAabb = Aabb(Vector3(2.67947f, -1.40719f, 3.67738f), Vector3(5.39655f, 5.38068f, 8.17438f)); 
  data[24].mAabb = Aabb(Vector3(-31.9608f, -3.43235f, 24.8703f), Vector3(-25.8966f, 0.956798f, 31.1155f)); 
  data[25].mAabb = Aabb(Vector3(-0.0316651f, -16.4748f, -19.4953f), Vector3(7.6021f, -9.68135f, -14.7622f)); 
  data[26].mAabb = Aabb(Vector3(24.3215f, 12.905f, 31.1527f), Vector3(28.6473f, 16.23f, 37.0879f)); 
  data[27].mAabb = Aabb(Vector3(-19.0735f, 6.46498f, -31.4368f), Vector3(-15.7737f, 14.2502f, -30.595f)); 
  data[28].mAabb = Aabb(Vector3(5.70132f, 18.4957f, 19.5646f), Vector3(8.60836f, 24.5073f, 22.8469f)); 
  data[29].mAabb = Aabb(Vector3(-12.0061f, 9.05095f, -12.6266f), Vector3(-4.27149f, 15.6076f, -6.9356f)); 
  data[30].mAabb = Aabb(Vector3(-20.4612f, -7.25671f, -16.3609f), Vector3(-14.354f, -2.54377f, -14.1525f)); 
  data[31].mAabb = Aabb(Vector3(0.652334f, 2.67661f, -44.0808f), Vector3(3.54546f, 7.75462f, -37.6174f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Ray ray;
  ray.mStart = Vector3(Vector3(-17.8044f, 3.18449f, -5.21445f));
  ray.mDirection = Vector3(Vector3(0.654809f, -0.597756f, -0.462508f));

  Application::mStatistics.Clear();
  PrintRayCastResults(spatialPartition, ray, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mRayAabbTests", static_cast<int>(Application::mStatistics.mRayAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mAabb = Aabb(Vector3(-42.1677f, 16.8547f, 9.0055f), Vector3(-39.1899f, 23.5138f, 11.001f)); 
  data[1].mAabb = Aabb(Vector3(9.61212f, -4.25645f, 5.30613f), Vector3(10.8404f, -3.29492f, 7.31974f)); 
  data[2].mAabb = Aabb(Vector3(-1.82378f, -9.76108f, -41.0821f), Vector3(4.63845f, -6.92964f, -33.7197f)); 
  data[3].mAabb = Aabb(Vector3(2.77457f, -51.4767f, -1.01452f), Vector3(3.80429f, -44.141f, 1.69607f)); 
  data[4].mAabb = Aabb(Vector3(25.3061f, 21.1362f, 20.4555f), Vector3(33.0863f, 24.3726f, 27.525f)); 
  data[5].mAabb = Aabb(Vector3(-10.373f, -9.27229f, -20.7413f), Vector3(-8.97241f, -5.09884f, -20.1038f)); 
  data[6].mAabb = Aabb(Vector3(1.04719f, 1.76581f, -18.9534f), Vector3(6.90197f, 8.05641f, -17.6458f)); 
  data[7].mAabb = Aabb(Vector3(-40.7551f, 27.1149f, -14.043f), Vector3(-33.836f, 30.1156f, -10.8062f)); 
  data[8].mAabb = Aabb(Vector3(-33.1279f, -28.0502f, 8.68627f), Vector3(-27.4689f, -23.3779f, 9.33815f)); 
  data[9].mAabb = Aabb(Vector3(23.1234f, -9.23255f, -4.35827f), Vector3(26.8561f, -4.56299f, 0.309435f)); 
  data[10].mAabb = Aabb(Vector3(-6.08976f, 28.5644f, -13.7216f), Vector3(-0.88001f, 29.1499f, -7.79767f)); 
  data[11].mAabb = Aabb(Vector3(4.60154f, 8.3081f, -5.25431f), Vector3(8.90534f, 9.2803f, 1.3617f)); 
  data[12].mAabb = Aabb(Vector3(-16.3052f, -11.1597f, -6.8423f), Vector3(-10.9247f, -5.38264f, -3.45282f)); 
  data[13].mAabb = Aabb(Vector3(-47.4864f, -18.9931f, -13.6648f), Vector3(-43.3104f, -14.4019f, -6.37063f)); 
  data[14].mAabb = Aabb(Vector3(-25.3652f, 3.09992f, 38.7823f), Vector3(-21.0779f, 7.32022f, 42.3351f)); 
  data[15].mAabb = Aabb(Vector3(-14.4758f, 22.7301f, -9.6493f), Vector3(-10.051f, 26.3279f, -5.65561f)); 
  data[16].mAabb = Aabb(Vector3(5.29098f, -30.4065f, -23.2867f), Vector3(7.90973f, -26.8222f, -19.6799f)); 
  data[17].mAabb = Aabb(Vector3(16.135f, 21.8654f, -14.0759f), Vector3(22.5886f, 28.3799f, -8.92435f)); 
  data[18].mAabb = Aabb(Vector3(11.8629f, 25.9047f, -23.7738f), Vector3(16.3285f, 27.2951f, -23.1667f)); 
  data[19].mAabb = Aabb(Vector3(14.1279f, 9.59282f, 6.89459f), Vector3(17.7264f, 15.3414f, 12.9904f)); 
  data[20].mAabb = Aabb(Vector3(2.95917f, -23.8312f, 18.15f), Vector3(4.97487f, -16.6527f, 21.978f)); 
  data[21].mAabb = Aabb(Vector3(-28.7206f, 16.7571f, 19.7172f), Vector3(-23.3154f, 19.3722f, 20.1446f)); 
  data[22].mAabb = Aabb(Vector3(-40.6333f, 0.100276f, -21.2316f), Vector3(-35.9573f, 7.48795f, -15.0665f)); 
  data[23].mAabb = Aabb(Vector3(-35.9732f, -19.2706f, -11.4882f), Vector3(-30.9358f, -13.9127f, -7.4778f)); 
  data[24].mAabb = Aabb(Vector3(-5.82017f, 20.2214f, 38.8791f), Vector3(-3.63377f, 26.8921f, 46.3255f)); 
  data[25].mAabb = Aabb(Vector3(-10.4467f, 29.9785f, -1.93543f), Vector3(-8.76359f, 36.5609f, 1.98404f)); 
  data[26].mAabb = Aabb(Vector3(-8.56228f, 29.9116f, 26.5763f), Vector3(-2.67898f, 31.8608f, 30.4911f)); 
  data[27].mAabb = Aabb(Vector3(-21.3574f, 16.0924f, -7.91824f), Vector3(-17.7381f, 19.6036f, -0.996302f)); 
  data[28].mAabb = Aabb(Vector3(12.7857f, 5.29881f, 10.1643f), Vector3(17.2647f, 13.0242f, 16.5087f)); 
  data[29].mAabb = Aabb(Vector3(18.8812f, 3.3726f, 39.6333f), Vector3(21.0783f, 10.0542f, 40.6767f)); 
  data[30].mAabb = Aabb(Vector3(43.8062f, -7.33572f, 14.2162f), Vector3(45.1254f, -3.1748f, 15.2671f)); 
  data[31].mAabb = Aabb(Vector3(7.56896f, 2.23976f, -49.8382f), Vector3(9.57862f, 3.92378f, -45.0464f)); 
  data[32].mAabb = Aabb(Vector3(-26.1802f, -36.2696f, -25.0588f), Vector3(-18.6995f, -33.8323f, -17.5941f)); 
  data[33].mAabb = Aabb(Vector3(4.04706f, -43.0534f, 6.00848f), Vector3(10.3567f, -35.8826f, 8.39923f)); 
  data[34].mAabb = Aabb(Vector3(41.6941f, -17.6749f, 11.0525f), Vector3(47.3615f, -9.96665f, 13.1719f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-73.9413f, -28.0282f, 33.4646f), Vector3(23.7861f, -70.578f, 52.5694f), Vector3(67.568f, 29.9794f, 52.5694f), Vector3(-30.1594f, 72.5291f, 33.4646f), Vector3(-0.466604f, -5.07931f, -5.76028f), Vector3(6.34117f, -8.04337f, -4.42942f), Vector3(9.90654f, 0.145493f, -4.42942f), Vector3(3.09876f, 3.10955f, -5.76028f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(25); 
  data[0].mAabb = Aabb(Vector3(19.9385f, 16.1432f, 36.9828f), Vector3(21.7376f, 16.7246f, 43.0454f)); 
  data[1].mAabb = Aabb(Vector3(9.54012f, 29.267f, -23.28f), Vector3(17.5174f, 30.8592f, -18.8027f)); 
  data[2].mAabb = Aabb(Vector3(-2.4759f, -42.224f, -29.66f), Vector3(-1.70943f, -35.4448f, -22.232f)); 
  data[3].mAabb = Aabb(Vector3(16.0096f, -5.99357f, -38.1912f), Vector3(20.8508f, 1.86634f, -32.7201f)); 
  data[4].mAabb = Aabb(Vector3(24.9111f, 5.65231f, -28.5078f), Vector3(28.6114f, 13.3197f, -28.0686f)); 
  data[5].mAabb = Aabb(Vector3(-28.1465f, 9.12785f, -5.74959f), Vector3(-27.7128f, 10.3536f, -0.442422f)); 
  data[6].mAabb = Aabb(Vector3(45.914f, 4.60178f, 0.379515f), Vector3(51.1367f, 7.60486f, 1.13415f)); 
  data[7].mAabb = Aabb(Vector3(17.6912f, -36.9419f, 30.2201f), Vector3(22.5906f, -31.2587f, 30.7414f)); 
  data[8].mAabb = Aabb(Vector3(-5.57818f, -18.3195f, 43.8364f), Vector3(-0.353585f, -11.3593f, 47.9537f)); 
  data[9].mAabb = Aabb(Vector3(-40.0591f, 8.054f, -30.8226f), Vector3(-38.084f, 10.2513f, -26.1315f)); 
  data[10].mAabb = Aabb(Vector3(3.70292f, -17.8182f, -42.0166f), Vector3(8.48845f, -15.4024f, -41.076f)); 
  data[11].mAabb = Aabb(Vector3(36.2201f, 1.52247f, -6.55131f), Vector3(38.1373f, 2.29497f, -0.462271f)); 
  data[12].mAabb = Aabb(Vector3(26.2915f, 16.0696f, 7.54566f), Vector3(33.8048f, 16.6563f, 9.2051f)); 
  data[13].mAabb = Aabb(Vector3(-26.2637f, -16.2909f, 19.4573f), Vector3(-25.3358f, -10.8722f, 22.8857f)); 
  data[14].mAabb = Aabb(Vector3(-21.2424f, -18.2977f, -32.8464f), Vector3(-20.8296f, -15.8283f, -25.7962f)); 
  data[15].mAabb = Aabb(Vector3(-7.01517f, 37.3491f, 4.61173f), Vector3(-1.51804f, 42.1328f, 7.22862f)); 
  data[16].mAabb = Aabb(Vector3(-34.506f, 20.7546f, -9.92357f), Vector3(-28.308f, 25.1092f, -3.98205f)); 
  data[17].mAabb = Aabb(Vector3(-33.7639f, 14.5482f, -23.88f), Vector3(-31.1008f, 17.9629f, -18.2546f)); 
  data[18].mAabb = Aabb(Vector3(35.8018f, 14.0568f, -23.7278f), Vector3(42.162f, 17.5309f, -20.8052f)); 
  data[19].mAabb = Aabb(Vector3(-11.813f, 12.2021f, 39.1788f), Vector3(-7.59925f, 18.2097f, 42.7668f)); 
  data[20].mAabb = Aabb(Vector3(-19.4034f, 6.41218f, -39.1574f), Vector3(-17.701f, 8.05654f, -32.6935f)); 
  data[21].mAabb = Aabb(Vector3(-34.3755f, -16.5988f, -27.7073f), Vector3(-26.5395f, -8.99035f, -22.46f)); 
  data[22].mAabb = Aabb(Vector3(-4.67789f, -11.8691f, -12.459f), Vector3(-2.62114f, -10.9006f, -9.59765f)); 
  data[23].mAabb = Aabb(Vector3(-36.6831f, -27.6736f, 19.0194f), Vector3(-33.4571f, -26.7427f, 25.4705f)); 
  data[24].mAabb = Aabb(Vector3(10.9841f, -44.3699f, 0.708763f), Vector3(17.6771f, -39.7115f, 6.72637f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-36.2776f, 35.2886f, -68.9134f), Vector3(-36.6607f, -77.2606f, -30.1711f), Vector3(-36.6607f, -38.7343f, 81.7505f), Vector3(-36.2776f, 73.8149f, 43.0083f), Vector3(38.3401f, 0.287793f, 2.4445f), Vector3(38.318f, -6.20886f, 4.68081f), Vector3(38.318f, -4.18876f, 10.5493f), Vector3(38.3401f, 2.30789f, 8.31304f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(39); 
  data[0].mAabb = Aabb(Vector3(-7.9285f, -4.80817f, 33.5905f), Vector3(-4.53762f, 1.03432f, 37.5331f)); 
  data[1].mAabb = Aabb(Vector3(17.9431f, 11.8543f, 2.91377f), Vector3(25.0794f, 16.9353f, 5.52672f)); 
  data[2].mAabb = Aabb(Vector3(-22.728f, -6.56122f, -14.0607f), Vector3(-17.8174f, -5.13813f, -10.9511f)); 
  data[3].mAabb = Aabb(Vector3(-23.3642f, -31.0845f, 9.80346f), Vector3(-21.1708f, -25.4914f, 13.2024f)); 
  data[4].mAabb = Aabb(Vector3(42.3746f, -0.258738f, -17.9478f), Vector3(49.8314f, 7.17602f, -13.1558f)); 
  data[5].mAabb = Aabb(Vector3(-10.0693f, 4.54971f, 11.2689f), Vector3(-5.96129f, 11.5248f, 13.3551f)); 
  data[6].mAabb = Aabb(Vector3(-40.7806f, 18.7138f, -2.38263f), Vector3(-40.3152f, 24.1358f, 3.40999f)); 
  data[7].mAabb = Aabb(Vector3(7.62455f, -39.4239f, -26.3932f), Vector3(15.3439f, -37.8751f, -22.6043f)); 
  data[8].mAabb = Aabb(Vector3(-18.5995f, -12.7709f, 19.3177f), Vector3(-15.3051f, -11.0695f, 22.3136f)); 
  data[9].mAabb = Aabb(Vector3(2.28719f, 17.4411f, -32.1716f), Vector3(6.55898f, 19.4218f, -24.6603f)); 
  data[10].mAabb = Aabb(Vector3(-11.9782f, -38.6196f, -16.243f), Vector3(-4.93752f, -35.2644f, -8.80357f)); 
  data[11].mAabb = Aabb(Vector3(-34.3065f, 32.7305f, -10.0611f), Vector3(-26.5445f, 39.6204f, -4.29374f)); 
  data[12].mAabb = Aabb(Vector3(-4.68964f, 19.3211f, -41.2952f), Vector3(-3.88397f, 26.2045f, -36.097f)); 
  data[13].mAabb = Aabb(Vector3(22.6575f, -1.34841f, -4.82765f), Vector3(29.1441f, -0.494969f, -2.2045f)); 
  data[14].mAabb = Aabb(Vector3(-19.9995f, -32.3206f, -8.78155f), Vector3(-19.5285f, -30.1847f, -7.13208f)); 
  data[15].mAabb = Aabb(Vector3(-2.22885f, 35.8282f, 3.34537f), Vector3(0.729229f, 36.7408f, 9.56708f)); 
  data[16].mAabb = Aabb(Vector3(-3.18818f, 27.0651f, 25.6025f), Vector3(-1.3476f, 32.591f, 26.2778f)); 
  data[17].mAabb = Aabb(Vector3(12.4081f, 28.0999f, 25.4291f), Vector3(17.5527f, 34.0002f, 26.6827f)); 
  data[18].mAabb = Aabb(Vector3(34.0291f, -12.4077f, -31.0752f), Vector3(38.2493f, -8.86168f, -28.9083f)); 
  data[19].mAabb = Aabb(Vector3(8.92394f, 27.3556f, -22.9376f), Vector3(11.9249f, 32.4524f, -21.0987f)); 
  data[20].mAabb = Aabb(Vector3(5.06159f, -13.4217f, 38.4368f), Vector3(8.55382f, -10.4277f, 44.633f)); 
  data[21].mAabb = Aabb(Vector3(14.912f, -9.00497f, 19.7017f), Vector3(20.9655f, -7.67488f, 27.5781f)); 
  data[22].mAabb = Aabb(Vector3(15.1491f, -10.3497f, -3.47074f), Vector3(17.0973f, -9.82488f, -2.65881f)); 
  data[23].mAabb = Aabb(Vector3(11.0055f, 19.4995f, -41.6798f), Vector3(12.2022f, 26.9311f, -38.3518f)); 
  data[24].mAabb = Aabb(Vector3(5.22333f, -13.2539f, -10.6893f), Vector3(7.5663f, -6.11672f, -5.2608f)); 
  data[25].mAabb = Aabb(Vector3(-32.4509f, 25.052f, -7.01291f), Vector3(-27.181f, 26.6089f, -5.11156f)); 
  data[26].mAabb = Aabb(Vector3(-2.67579f, 22.4878f, -8.05947f), Vector3(0.356968f, 23.4971f, -5.69516f)); 
  data[27].mAabb = Aabb(Vector3(-39.706f, -5.55834f, 1.86774f), Vector3(-36.5337f, -0.0656149f, 6.85576f)); 
  data[28].mAabb = Aabb(Vector3(6.56536f, -26.4178f, -42.8927f), Vector3(13.8245f, -21.3171f, -39.5953f)); 
  data[29].mAabb = Aabb(Vector3(-47.9026f, -13.1314f, 5.24527f), Vector3(-41.0213f, -7.68134f, 11.7683f)); 
  data[30].mAabb = Aabb(Vector3(-21.0196f, -8.76493f, -9.60975f), Vector3(-18.3795f, -5.44364f, -2.41319f)); 
  data[31].mAabb = Aabb(Vector3(-20.5008f, -7.5188f, -44.598f), Vector3(-17.1665f, -6.45011f, -43.768f)); 
  data[32].mAabb = Aabb(Vector3(-9.57225f, -13.093f, 45.7763f), Vector3(-2.0359f, -7.39983f, 51.0693f)); 
  data[33].mAabb = Aabb(Vector3(-3.94946f, -35.5512f, -5.13657f), Vector3(-0.209517f, -34.6152f, 1.37507f)); 
  data[34].mAabb = Aabb(Vector3(1.00581f, 29.5516f, 2.11195f), Vector3(4.55046f, 36.2596f, 4.5674f)); 
  data[35].mAabb = Aabb(Vector3(28.5082f, 0.967918f, 1.52129f), Vector3(36.3846f, 8.10579f, 3.16797f)); 
  data[36].mAabb = Aabb(Vector3(28.666f, 15.3054f, -2.78913f), Vector3(31.4611f, 18.2037f, 2.53914f)); 
  data[37].mAabb = Aabb(Vector3(10.4275f, -17.4852f, 6.59602f), Vector3(15.1722f, -12.7342f, 10.358f)); 
  data[38].mAabb = Aabb(Vector3(-7.29339f, 34.7725f, 1.08169f), Vector3(-6.39403f, 38.6033f, 8.47888f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-21.6662f, 77.2502f, 45.7339f), Vector3(-7.19457f, 24.3769f, -65.9153f), Vector3(-7.19457f, -89.85f, -11.8214f), Vector3(-21.6662f, -36.9767f, 99.8279f), Vector3(36.0696f, 2.3361f, 23.75f), Vector3(36.9757f, -0.974291f, 16.7596f), Vector3(36.9757f, -9.82656f, 20.9517f), Vector3(36.0696f, -6.51617f, 27.9421f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(47); 
  data[0].mAabb = Aabb(Vector3(-9.50206f, -15.5945f, -39.1654f), Vector3(-5.07186f, -13.3199f, -38.6247f)); 
  data[1].mAabb = Aabb(Vector3(32.3077f, -9.13584f, -23.4835f), Vector3(40.2766f, -3.1797f, -22.6795f)); 
  data[2].mAabb = Aabb(Vector3(12.2157f, 19.6917f, 31.4826f), Vector3(17.7203f, 27.3955f, 37.2054f)); 
  data[3].mAabb = Aabb(Vector3(15.9509f, -30.5809f, 11.6512f), Vector3(16.9339f, -28.9955f, 18.9454f)); 
  data[4].mAabb = Aabb(Vector3(-36.8676f, 3.41411f, -7.09757f), Vector3(-32.4499f, 4.78502f, -3.78347f)); 
  data[5].mAabb = Aabb(Vector3(-14.5375f, 26.7657f, 28.5979f), Vector3(-6.9f, 29.1019f, 32.5821f)); 
  data[6].mAabb = Aabb(Vector3(-4.50597f, -2.87493f, -51.4099f), Vector3(-2.30751f, 3.0056f, -47.1794f)); 
  data[7].mAabb = Aabb(Vector3(-35.2384f, -15.0217f, 5.33861f), Vector3(-33.0149f, -11.0821f, 6.19623f)); 
  data[8].mAabb = Aabb(Vector3(10.2511f, -24.0447f, -17.8674f), Vector3(16.7646f, -19.7739f, -10.8251f)); 
  data[9].mAabb = Aabb(Vector3(6.51458f, 30.1778f, 19.2574f), Vector3(10.798f, 32.2285f, 23.1662f)); 
  data[10].mAabb = Aabb(Vector3(2.49261f, 28.5512f, -40.317f), Vector3(2.95198f, 34.6363f, -36.2987f)); 
  data[11].mAabb = Aabb(Vector3(20.343f, -0.89387f, -14.231f), Vector3(22.33f, 3.66761f, -6.57362f)); 
  data[12].mAabb = Aabb(Vector3(0.801166f, 41.2868f, 13.776f), Vector3(2.00948f, 44.7765f, 19.189f)); 
  data[13].mAabb = Aabb(Vector3(-8.13955f, -13.1756f, -4.10187f), Vector3(-0.146973f, -9.50527f, -2.11609f)); 
  data[14].mAabb = Aabb(Vector3(15.6296f, 5.84561f, -30.7598f), Vector3(17.7401f, 9.25782f, -27.8507f)); 
  data[15].mAabb = Aabb(Vector3(12.2732f, -6.55389f, -13.657f), Vector3(17.5727f, -2.17913f, -6.3405f)); 
  data[16].mAabb = Aabb(Vector3(-5.68788f, -17.8218f, 14.7896f), Vector3(0.386081f, -11.5683f, 16.919f)); 
  data[17].mAabb = Aabb(Vector3(29.4788f, 15.1334f, -33.0782f), Vector3(29.885f, 19.7795f, -29.1455f)); 
  data[18].mAabb = Aabb(Vector3(-1.7555f, 20.6587f, -36.7163f), Vector3(2.87463f, 23.7993f, -29.5905f)); 
  data[19].mAabb = Aabb(Vector3(1.41349f, 6.43531f, -21.8294f), Vector3(3.50132f, 7.58564f, -16.7613f)); 
  data[20].mAabb = Aabb(Vector3(15.9052f, 34.6717f, 3.87893f), Vector3(20.0047f, 41.3545f, 7.0432f)); 
  data[21].mAabb = Aabb(Vector3(-22.872f, -4.57239f, 1.89713f), Vector3(-16.5717f, 2.40707f, 4.66177f)); 
  data[22].mAabb = Aabb(Vector3(39.6058f, 4.5611f, 2.79399f), Vector3(46.0724f, 9.3515f, 5.90909f)); 
  data[23].mAabb = Aabb(Vector3(-14.4634f, -24.1819f, -30.4579f), Vector3(-11.3314f, -20.9311f, -24.7374f)); 
  data[24].mAabb = Aabb(Vector3(33.714f, -21.4236f, -22.4715f), Vector3(34.4962f, -16.8994f, -16.3122f)); 
  data[25].mAabb = Aabb(Vector3(-46.4044f, -20.4812f, -7.07685f), Vector3(-43.6959f, -19.4382f, -2.54158f)); 
  data[26].mAabb = Aabb(Vector3(6.30585f, 35.1225f, 3.90338f), Vector3(8.20674f, 37.4532f, 6.43028f)); 
  data[27].mAabb = Aabb(Vector3(-9.79299f, -12.3316f, -22.8337f), Vector3(-5.02717f, -6.00575f, -15.2646f)); 
  data[28].mAabb = Aabb(Vector3(-0.739682f, 17.0186f, -42.7526f), Vector3(1.62694f, 19.0527f, -40.5746f)); 
  data[29].mAabb = Aabb(Vector3(14.1501f, 11.8811f, -45.5372f), Vector3(21.7528f, 15.4311f, -39.9135f)); 
  data[30].mAabb = Aabb(Vector3(10.7394f, -12.2506f, -44.1651f), Vector3(14.202f, -11.7119f, -43.426f)); 
  data[31].mAabb = Aabb(Vector3(10.7426f, 5.86327f, -4.42371f), Vector3(14.6713f, 7.22096f, -3.36175f)); 
  data[32].mAabb = Aabb(Vector3(-0.872099f, 14.2511f, 12.3164f), Vector3(5.56068f, 16.0629f, 17.3235f)); 
  data[33].mAabb = Aabb(Vector3(23.2394f, 20.397f, 30.6258f), Vector3(29.3553f, 25.6729f, 35.8012f)); 
  data[34].mAabb = Aabb(Vector3(29.7569f, 23.2406f, -11.3231f), Vector3(30.4489f, 25.1461f, -10.3887f)); 
  data[35].mAabb = Aabb(Vector3(12.4776f, 3.42732f, -44.6976f), Vector3(15.2457f, 9.88422f, -42.0837f)); 
  data[36].mAabb = Aabb(Vector3(-30.2505f, -11.1114f, 11.0966f), Vector3(-25.4151f, -5.53058f, 12.7036f)); 
  data[37].mAabb = Aabb(Vector3(1.66492f, -27.5212f, 10.5098f), Vector3(4.78675f, -24.1941f, 17.5998f)); 
  data[38].mAabb = Aabb(Vector3(29.027f, -2.28527f, 30.7323f), Vector3(36.2836f, -0.189318f, 35.6598f)); 
  data[39].mAabb = Aabb(Vector3(-18.5148f, 3.14571f, 6.47084f), Vector3(-15.7248f, 3.9523f, 8.44038f)); 
  data[40].mAabb = Aabb(Vector3(-10.698f, -4.07739f, -7.89175f), Vector3(-4.10407f, 2.24591f, -1.77627f)); 
  data[41].mAabb = Aabb(Vector3(-28.8556f, -14.0682f, -42.2431f), Vector3(-24.2958f, -10.1054f, -38.2819f)); 
  data[42].mAabb = Aabb(Vector3(-15.7353f, -22.2203f, 8.32323f), Vector3(-9.32359f, -20.6654f, 9.46382f)); 
  data[43].mAabb = Aabb(Vector3(-37.6096f, -14.7179f, -10.2634f), Vector3(-33.849f, -10.7585f, -4.74469f)); 
  data[44].mAabb = Aabb(Vector3(2.19681f, 12.0712f, -24.6f), Vector3(6.92714f, 16.2328f, -20.003f)); 
  data[45].mAabb = Aabb(Vector3(-16.6154f, -14.5861f, -11.3288f), Vector3(-13.1042f, -11.0215f, -7.56335f)); 
  data[46].mAabb = Aabb(Vector3(-43.1882f, 4.06178f, -7.34766f), Vector3(-35.5699f, 6.92754f, -4.41579f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(99.9491f, 62.2028f, 23.3748f), Vector3(-54.8129f, 102.551f, -9.80993f), Vector3(-95.075f, -51.8825f, -9.80993f), Vector3(59.6871f, -92.2303f, 23.3748f), Vector3(16.9518f, 3.85409f, -41.474f), Vector3(8.85824f, 5.96415f, -43.2094f), Vector3(7.64837f, 1.32347f, -43.2094f), Vector3(15.7419f, -0.786595f, -41.474f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(59); 
  data[0].mAabb = Aabb(Vector3(-3.71747f, 0.905769f, 35.9082f), Vector3(0.617632f, 2.3328f, 36.8642f)); 
  data[1].mAabb = Aabb(Vector3(1.93904f, -2.34111f, -11.1892f), Vector3(8.56456f, -0.837532f, -9.7309f)); 
  data[2].mAabb = Aabb(Vector3(-7.37312f, -0.540035f, -10.248f), Vector3(-0.26632f, 2.23712f, -9.10296f)); 
  data[3].mAabb = Aabb(Vector3(-15.5825f, -25.0714f, -9.45275f), Vector3(-9.70246f, -19.7464f, -1.78489f)); 
  data[4].mAabb = Aabb(Vector3(18.8613f, -7.47183f, 36.9604f), Vector3(26.3218f, -3.76806f, 40.0349f)); 
  data[5].mAabb = Aabb(Vector3(18.8813f, 18.8783f, 13.0429f), Vector3(23.433f, 22.057f, 20.7096f)); 
  data[6].mAabb = Aabb(Vector3(17.041f, -8.20018f, -4.83809f), Vector3(22.3337f, -5.52786f, 0.644191f)); 
  data[7].mAabb = Aabb(Vector3(-21.7155f, 34.4444f, -28.9781f), Vector3(-15.7918f, 35.5386f, -21.0479f)); 
  data[8].mAabb = Aabb(Vector3(-11.1035f, -13.5349f, 28.978f), Vector3(-9.32044f, -8.72455f, 30.1682f)); 
  data[9].mAabb = Aabb(Vector3(-24.7756f, 33.9456f, -20.9848f), Vector3(-20.2513f, 35.4448f, -19.6489f)); 
  data[10].mAabb = Aabb(Vector3(-24.0304f, -24.1843f, 27.3142f), Vector3(-20.2749f, -16.3153f, 34.1858f)); 
  data[11].mAabb = Aabb(Vector3(14.9756f, 16.0041f, 6.80855f), Vector3(22.1292f, 23.5966f, 9.30135f)); 
  data[12].mAabb = Aabb(Vector3(29.7141f, 2.66588f, -12.498f), Vector3(30.6969f, 3.6295f, -11.2203f)); 
  data[13].mAabb = Aabb(Vector3(-3.14534f, -4.275f, 27.7765f), Vector3(0.307459f, 0.00629735f, 29.158f)); 
  data[14].mAabb = Aabb(Vector3(-3.102f, -8.77667f, 36.7311f), Vector3(4.28985f, -1.56736f, 42.118f)); 
  data[15].mAabb = Aabb(Vector3(2.96234f, -35.6675f, 6.4402f), Vector3(4.64799f, -33.7754f, 8.71521f)); 
  data[16].mAabb = Aabb(Vector3(13.151f, -3.70259f, 24.9818f), Vector3(15.6828f, 3.85464f, 27.1165f)); 
  data[17].mAabb = Aabb(Vector3(0.929568f, -2.77991f, -34.5914f), Vector3(6.91841f, 1.88594f, -27.4946f)); 
  data[18].mAabb = Aabb(Vector3(4.5062f, -36.7835f, 5.37455f), Vector3(5.58092f, -31.8279f, 10.5674f)); 
  data[19].mAabb = Aabb(Vector3(-0.22191f, -16.6349f, -12.2076f), Vector3(4.9345f, -11.1802f, -8.42472f)); 
  data[20].mAabb = Aabb(Vector3(22.315f, 4.62961f, 26.2156f), Vector3(25.4157f, 9.96484f, 31.0695f)); 
  data[21].mAabb = Aabb(Vector3(-45.3028f, 9.88386f, -13.657f), Vector3(-44.0817f, 15.3149f, -12.448f)); 
  data[22].mAabb = Aabb(Vector3(-1.02017f, 19.5541f, -34.5332f), Vector3(6.31811f, 26.8959f, -32.8437f)); 
  data[23].mAabb = Aabb(Vector3(8.45926f, 36.0204f, -10.5174f), Vector3(11.9526f, 36.6377f, -6.47408f)); 
  data[24].mAabb = Aabb(Vector3(6.30535f, 10.0304f, -25.8949f), Vector3(13.6234f, 15.3515f, -25.4531f)); 
  data[25].mAabb = Aabb(Vector3(-25.9175f, 12.1008f, -6.19749f), Vector3(-24.7961f, 14.5586f, 0.892604f)); 
  data[26].mAabb = Aabb(Vector3(-4.56478f, -34.036f, -28.4944f), Vector3(-3.95835f, -27.7614f, -20.8855f)); 
  data[27].mAabb = Aabb(Vector3(10.9129f, -11.2053f, 20.33f), Vector3(17.9489f, -7.37187f, 21.616f)); 
  data[28].mAabb = Aabb(Vector3(-23.9883f, 16.9422f, -29.2359f), Vector3(-16.9116f, 21.4353f, -25.245f)); 
  data[29].mAabb = Aabb(Vector3(42.4171f, 8.45233f, -8.92196f), Vector3(46.356f, 11.7961f, -6.41037f)); 
  data[30].mAabb = Aabb(Vector3(2.41356f, 20.4555f, 37.0678f), Vector3(7.19446f, 26.2409f, 41.1253f)); 
  data[31].mAabb = Aabb(Vector3(-29.4248f, 3.38072f, -20.1807f), Vector3(-22.6202f, 10.2412f, -16.0028f)); 
  data[32].mAabb = Aabb(Vector3(10.7704f, 14.7174f, -11.7755f), Vector3(18.163f, 21.7284f, -5.57765f)); 
  data[33].mAabb = Aabb(Vector3(-22.2215f, -1.14752f, 11.5733f), Vector3(-18.0752f, 4.01909f, 13.1281f)); 
  data[34].mAabb = Aabb(Vector3(41.6009f, 6.94358f, 2.17507f), Vector3(43.3706f, 14.5753f, 8.69065f)); 
  data[35].mAabb = Aabb(Vector3(-12.5122f, -28.2992f, 27.4991f), Vector3(-4.78962f, -21.9998f, 31.5023f)); 
  data[36].mAabb = Aabb(Vector3(14.7636f, -8.94307f, -1.37823f), Vector3(22.3763f, -4.35028f, 0.457017f)); 
  data[37].mAabb = Aabb(Vector3(5.96153f, -11.5887f, -19.0677f), Vector3(9.29581f, -4.19015f, -18.4005f)); 
  data[38].mAabb = Aabb(Vector3(-37.8087f, -13.7413f, 18.4297f), Vector3(-37.2371f, -12.093f, 21.0081f)); 
  data[39].mAabb = Aabb(Vector3(33.7365f, -30.2142f, 3.69222f), Vector3(35.7214f, -22.7296f, 4.69132f)); 
  data[40].mAabb = Aabb(Vector3(27.7599f, -36.2066f, -12.8998f), Vector3(34.1314f, -28.9655f, -8.92773f)); 
  data[41].mAabb = Aabb(Vector3(-20.5731f, -28.1302f, 27.0174f), Vector3(-13.715f, -21.3602f, 28.6879f)); 
  data[42].mAabb = Aabb(Vector3(-23.6439f, 13.9255f, 6.95737f), Vector3(-22.7339f, 20.3757f, 11.6126f)); 
  data[43].mAabb = Aabb(Vector3(-47.521f, -18.011f, 8.52914f), Vector3(-43.3406f, -15.2892f, 9.76621f)); 
  data[44].mAabb = Aabb(Vector3(-18.1639f, -21.6608f, 31.9211f), Vector3(-10.6628f, -18.9409f, 37.8638f)); 
  data[45].mAabb = Aabb(Vector3(3.19236f, -33.7253f, -5.62641f), Vector3(9.42381f, -29.2409f, -0.307645f)); 
  data[46].mAabb = Aabb(Vector3(-25.1684f, 19.7979f, 2.77631f), Vector3(-21.7541f, 20.6796f, 7.58759f)); 
  data[47].mAabb = Aabb(Vector3(16.555f, -43.0803f, -2.59907f), Vector3(20.3147f, -35.6022f, 4.25677f)); 
  data[48].mAabb = Aabb(Vector3(-29.4965f, 28.6317f, 2.1889f), Vector3(-25.4104f, 33.0125f, 7.25694f)); 
  data[49].mAabb = Aabb(Vector3(-31.2768f, 16.204f, -22.136f), Vector3(-30.8042f, 19.1862f, -16.3302f)); 
  data[50].mAabb = Aabb(Vector3(29.3597f, -0.913932f, -24.9551f), Vector3(34.9121f, 3.70716f, -18.3892f)); 
  data[51].mAabb = Aabb(Vector3(-43.9448f, -6.27794f, 6.42794f), Vector3(-41.8873f, -3.75777f, 13.6714f)); 
  data[52].mAabb = Aabb(Vector3(0.281599f, -16.3413f, 41.5173f), Vector3(1.75548f, -11.4008f, 46.0437f)); 
  data[53].mAabb = Aabb(Vector3(-9.51093f, 26.0831f, 22.8334f), Vector3(-6.31071f, 33.5372f, 24.2567f)); 
  data[54].mAabb = Aabb(Vector3(-22.2296f, -10.4329f, -7.36395f), Vector3(-16.1151f, -6.5032f, -2.92795f)); 
  data[55].mAabb = Aabb(Vector3(15.8341f, -30.5515f, 26.4225f), Vector3(20.496f, -28.1478f, 27.9945f)); 
  data[56].mAabb = Aabb(Vector3(9.30111f, 38.3036f, 12.1905f), Vector3(12.5329f, 41.7096f, 13.1631f)); 
  data[57].mAabb = Aabb(Vector3(-16.4708f, -30.8013f, -37.8754f), Vector3(-11.3938f, -29.0964f, -31.7063f)); 
  data[58].mAabb = Aabb(Vector3(23.9811f, -10.111f, 12.2657f), Vector3(27.0278f, -6.3564f, 14.6978f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-31.1274f, -29.4109f, 24.3172f), Vector3(49.0287f, -31.3402f, 36.3449f), Vector3(50.9906f, 50.1709f, 36.3449f), Vector3(-29.1655f, 52.1002f, 24.3172f), Vector3(9.4516f, 6.09263f, 5.41697f), Vector3(17.4963f, 5.899f, 6.6241f), Vector3(17.7031f, 14.4919f, 6.6241f), Vector3(9.65843f, 14.6855f, 5.41697f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mAabb = Aabb(Vector3(9.84348f, -29.3175f, 20.6483f), Vector3(14.1788f, -23.7102f, 25.4183f)); 
  data[1].mAabb = Aabb(Vector3(3.92432f, 3.07753f, 24.1609f), Vector3(8.20283f, 10.5088f, 25.1403f)); 
  data[2].mAabb = Aabb(Vector3(11.2247f, 6.87306f, -33.1559f), Vector3(17.265f, 11.3566f, -29.1077f)); 
  data[3].mAabb = Aabb(Vector3(-18.8f, 44.7503f, -9.33814f), Vector3(-15.421f, 45.5694f, -5.79163f)); 
  data[4].mAabb = Aabb(Vector3(-7.0444f, -3.29029f, -0.36972f), Vector3(-1.94134f, 4.5898f, 2.70641f)); 
  data[5].mAabb = Aabb(Vector3(-5.39262f, -20.0531f, -30.4276f), Vector3(-4.42552f, -14.8313f, -30.0005f)); 
  data[6].mAabb = Aabb(Vector3(14.5951f, 38.0192f, -5.41543f), Vector3(20.5842f, 39.5974f, -0.248349f)); 
  data[7].mAabb = Aabb(Vector3(6.27989f, -16.9989f, -16.9198f), Vector3(12.4466f, -13.5976f, -15.3895f)); 
  data[8].mAabb = Aabb(Vector3(-27.7389f, -10.4808f, 20.5274f), Vector3(-23.2349f, -3.81561f, 21.885f)); 
  data[9].mAabb = Aabb(Vector3(23.505f, -27.2642f, -32.3131f), Vector3(28.3541f, -23.7393f, -27.1022f)); 
  data[10].mAabb = Aabb(Vector3(6.6253f, -26.5444f, -11.5587f), Vector3(13.7022f, -24.8191f, -6.2868f)); 
  data[11].mAabb = Aabb(Vector3(23.3826f, -12.5216f, 30.4812f), Vector3(29.146f, -10.0977f, 31.6204f)); 
  data[12].mAabb = Aabb(Vector3(36.5577f, 16.3074f, -13.7175f), Vector3(37.397f, 24.1676f, -11.3973f)); 
  data[13].mAabb = Aabb(Vector3(-17.3923f, -26.4433f, 22.23f), Vector3(-14.2529f, -24.6099f, 26.4649f)); 
  data[14].mAabb = Aabb(Vector3(22.9701f, -28.5795f, 10.467f), Vector3(23.5786f, -24.7937f, 16.3166f)); 
  data[15].mAabb = Aabb(Vector3(24.8341f, -27.1911f, 17.033f), Vector3(31.4139f, -22.0217f, 19.617f)); 
  data[16].mAabb = Aabb(Vector3(-2.80289f, 27.865f, 13.906f), Vector3(3.48469f, 31.1474f, 20.1602f)); 
  data[17].mAabb = Aabb(Vector3(-31.9589f, -7.68223f, 31.2139f), Vector3(-27.9112f, -5.47333f, 33.6612f)); 
  data[18].mAabb = Aabb(Vector3(32.6584f, 26.5507f, 15.6258f), Vector3(37.0561f, 34.3972f, 21.286f)); 
  data[19].mAabb = Aabb(Vector3(-24.1294f, -8.19165f, 38.2708f), Vector3(-22.9587f, -2.15526f, 45.9463f)); 
  data[20].mAabb = Aabb(Vector3(-32.9096f, 17.5389f, 5.64876f), Vector3(-26.0559f, 24.7568f, 13.4477f)); 
  data[21].mAabb = Aabb(Vector3(32.8368f, -22.282f, 26.2108f), Vector3(35.5024f, -17.0674f, 28.8516f)); 
  data[22].mAabb = Aabb(Vector3(28.1756f, -38.1491f, -22.2319f), Vector3(30.468f, -31.5094f, -18.5555f)); 
  data[23].mAabb = Aabb(Vector3(5.92621f, -11.5237f, -10.642f), Vector3(9.02646f, -9.57528f, -6.64086f)); 
  data[24].mAabb = Aabb(Vector3(-5.26728f, 11.6137f, -38.1284f), Vector3(-2.19509f, 14.3273f, -33.0708f)); 
  data[25].mAabb = Aabb(Vector3(10.7521f, -41.7039f, -12.6143f), Vector3(15.559f, -34.9054f, -5.40959f)); 
  data[26].mAabb = Aabb(Vector3(-7.4811f, -28.2188f, -24.2799f), Vector3(-5.02959f, -27.5046f, -17.5075f)); 
  data[27].mAabb = Aabb(Vector3(-23.2546f, -16.4092f, 34.8295f), Vector3(-22.3004f, -15.298f, 41.8818f)); 
  data[28].mAabb = Aabb(Vector3(-32.4046f, 13.3434f, -17.777f), Vector3(-30.1456f, 16.1683f, -11.9169f)); 
  data[29].mAabb = Aabb(Vector3(-22.3943f, -2.26533f, -12.7568f), Vector3(-19.9982f, 1.46047f, -10.9308f)); 
  data[30].mAabb = Aabb(Vector3(-29.8355f, -2.64771f, 2.65712f), Vector3(-28.263f, 4.30809f, 6.81966f)); 
  data[31].mAabb = Aabb(Vector3(23.6345f, 38.2689f, 13.1307f), Vector3(24.34f, 40.5894f, 13.8306f)); 
  data[32].mAabb = Aabb(Vector3(-43.8501f, -7.36599f, 6.13556f), Vector3(-35.938f, -0.95641f, 6.91386f)); 
  data[33].mAabb = Aabb(Vector3(-31.8668f, -6.98721f, 15.9952f), Vector3(-25.0585f, 0.574656f, 20.7372f)); 
  data[34].mAabb = Aabb(Vector3(-6.37516f, 33.5599f, -12.4173f), Vector3(0.691509f, 40.8156f, -5.02547f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-19.1816f, -82.534f, 99.7438f), Vector3(-53.6264f, 110.099f, 92.2313f), Vector3(-53.6264f, 102.558f, -101.139f), Vector3(-19.1816f, -90.0752f, -93.6268f), Vector3(60.8264f, 22.594f, 2.50506f), Vector3(59.1209f, 32.1318f, 2.1331f), Vector3(59.1209f, 31.8441f, -5.24263f), Vector3(60.8264f, 22.3063f, -4.87066f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(42); 
  data[0].mAabb = Aabb(Vector3(-7.21987f, 18.6339f, -20.6889f), Vector3(-6.58004f, 21.9902f, -14.284f)); 
  data[1].mAabb = Aabb(Vector3(18.701f, -42.7009f, -6.30156f), Vector3(26.4334f, -39.5143f, -1.76838f)); 
  data[2].mAabb = Aabb(Vector3(10.6233f, -20.2861f, -31.0532f), Vector3(15.3323f, -18.9268f, -29.9471f)); 
  data[3].mAabb = Aabb(Vector3(-13.052f, -32.883f, -31.8336f), Vector3(-8.14191f, -29.7046f, -30.3667f)); 
  data[4].mAabb = Aabb(Vector3(-19.0224f, -0.3212f, 16.5103f), Vector3(-18.3603f, 1.41107f, 20.0791f)); 
  data[5].mAabb = Aabb(Vector3(27.6907f, -15.1276f, 32.9264f), Vector3(33.572f, -9.62514f, 36.6313f)); 
  data[6].mAabb = Aabb(Vector3(17.9879f, 23.9599f, 30.5614f), Vector3(24.1591f, 25.6771f, 34.8556f)); 
  data[7].mAabb = Aabb(Vector3(-11.8938f, -36.2957f, 15.4886f), Vector3(-7.82701f, -31.2819f, 21.8165f)); 
  data[8].mAabb = Aabb(Vector3(16.7584f, 39.0405f, 6.05849f), Vector3(21.4813f, 46.6356f, 10.0749f)); 
  data[9].mAabb = Aabb(Vector3(0.178962f, -26.1312f, -0.445325f), Vector3(2.2494f, -20.003f, 0.759741f)); 
  data[10].mAabb = Aabb(Vector3(3.05556f, -4.52043f, -19.0916f), Vector3(10.8329f, 2.34167f, -17.8747f)); 
  data[11].mAabb = Aabb(Vector3(-29.7205f, 6.27543f, -6.08438f), Vector3(-22.2876f, 10.4869f, -0.805274f)); 
  data[12].mAabb = Aabb(Vector3(17.4431f, -11.2869f, 45.0043f), Vector3(18.6004f, -4.77757f, 46.7463f)); 
  data[13].mAabb = Aabb(Vector3(14.115f, 26.792f, -4.39563f), Vector3(16.3893f, 34.0906f, 1.46054f)); 
  data[14].mAabb = Aabb(Vector3(-30.5603f, -9.61579f, 31.0728f), Vector3(-29.0238f, -2.60524f, 31.9594f)); 
  data[15].mAabb = Aabb(Vector3(-14.128f, -23.957f, 14.5947f), Vector3(-13.5364f, -16.3019f, 17.7669f)); 
  data[16].mAabb = Aabb(Vector3(-7.91031f, -28.8633f, -34.6178f), Vector3(-3.62507f, -23.0278f, -28.5141f)); 
  data[17].mAabb = Aabb(Vector3(21.1952f, -28.9913f, -16.5449f), Vector3(27.9633f, -22.4956f, -9.05652f)); 
  data[18].mAabb = Aabb(Vector3(18.0398f, -37.9594f, -18.2622f), Vector3(20.9749f, -31.4584f, -13.1589f)); 
  data[19].mAabb = Aabb(Vector3(4.26176f, 0.808508f, 15.3169f), Vector3(8.52403f, 2.17756f, 19.1014f)); 
  data[20].mAabb = Aabb(Vector3(-9.70917f, -20.8592f, -16.4636f), Vector3(-3.77716f, -15.4908f, -15.7572f)); 
  data[21].mAabb = Aabb(Vector3(-32.0047f, 25.6002f, -15.9834f), Vector3(-31.1691f, 27.1252f, -8.67222f)); 
  data[22].mAabb = Aabb(Vector3(13.2201f, -21.4862f, -33.5184f), Vector3(16.1501f, -20.8798f, -31.5076f)); 
  data[23].mAabb = Aabb(Vector3(10.8629f, 36.9646f, -18.2542f), Vector3(11.7458f, 40.5366f, -15.5919f)); 
  data[24].mAabb = Aabb(Vector3(-23.9327f, -30.0929f, 22.452f), Vector3(-20.0343f, -26.9674f, 24.4876f)); 
  data[25].mAabb = Aabb(Vector3(2.90077f, -14.2056f, -30.9747f), Vector3(10.1284f, -6.26243f, -27.3421f)); 
  data[26].mAabb = Aabb(Vector3(1.39054f, 30.6039f, 15.9646f), Vector3(5.88614f, 33.3036f, 22.1707f)); 
  data[27].mAabb = Aabb(Vector3(17.9368f, 35.7547f, 12.9364f), Vector3(24.4807f, 36.6315f, 16.655f)); 
  data[28].mAabb = Aabb(Vector3(43.7875f, -1.63183f, 14.1837f), Vector3(44.7683f, 4.25612f, 19.223f)); 
  data[29].mAabb = Aabb(Vector3(3.66727f, 27.6232f, 5.88377f), Vector3(6.43632f, 32.8559f, 7.21431f)); 
  data[30].mAabb = Aabb(Vector3(-17.8944f, -34.1957f, 25.528f), Vector3(-10.976f, -26.4293f, 26.5238f)); 
  data[31].mAabb = Aabb(Vector3(-23.375f, 3.30367f, -28.0381f), Vector3(-16.3058f, 6.54726f, -21.6626f)); 
  data[32].mAabb = Aabb(Vector3(-38.7572f, -9.7408f, 5.15517f), Vector3(-33.8146f, -3.25375f, 10.8151f)); 
  data[33].mAabb = Aabb(Vector3(-23.5543f, 20.4262f, 10.7629f), Vector3(-20.4374f, 22.1564f, 17.8349f)); 
  data[34].mAabb = Aabb(Vector3(5.37884f, -9.54469f, -10.4262f), Vector3(11.6217f, -3.2752f, -7.22043f)); 
  data[35].mAabb = Aabb(Vector3(30.1553f, -16.6165f, 10.8354f), Vector3(31.6057f, -11.7375f, 12.0977f)); 
  data[36].mAabb = Aabb(Vector3(-26.2312f, -16.471f, -34.068f), Vector3(-19.5987f, -12.6255f, -32.5484f)); 
  data[37].mAabb = Aabb(Vector3(-36.5966f, -14.6492f, -21.9918f), Vector3(-32.5282f, -9.14693f, -19.3079f)); 
  data[38].mAabb = Aabb(Vector3(5.40635f, 34.423f, 6.31752f), Vector3(9.67141f, 37.186f, 13.6539f)); 
  data[39].mAabb = Aabb(Vector3(26.3225f, 17.9f, -39.0925f), Vector3(26.9523f, 20.9908f, -34.233f)); 
  data[40].mAabb = Aabb(Vector3(31.8834f, 22.0324f, 8.97236f), Vector3(36.5402f, 26.9979f, 10.1231f)); 
  data[41].mAabb = Aabb(Vector3(-17.9657f, -15.8665f, -9.83072f), Vector3(-10.5471f, -8.29447f, -6.75714f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-19.604f, -65.4718f, 50.5429f), Vector3(-68.0546f, 26.448f, 56.9129f), Vector3(-68.0546f, 33.782f, -48.9187f), Vector3(-19.604f, -58.1378f, -55.2886f), Vector3(5.11976f, 6.96161f, 5.73483f), Vector3(2.93912f, 11.0987f, 6.02153f), Vector3(2.93912f, 11.5597f, -0.630996f), Vector3(5.11976f, 7.42263f, -0.917693f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(-30.3175f, 0.229832f, -21.4237f), Vector3(-29.2187f, 4.92027f, -16.4081f)); 
  data[1].mAabb = Aabb(Vector3(5.29608f, -18.865f, -16.5354f), Vector3(13.0372f, -13.035f, -13.8693f)); 
  data[2].mAabb = Aabb(Vector3(19.3078f, -17.06f, -18.559f), Vector3(26.3912f, -11.2998f, -13.6292f)); 
  data[3].mAabb = Aabb(Vector3(9.98304f, -22.3939f, -11.8887f), Vector3(12.8435f, -21.3486f, -7.85421f)); 
  data[4].mAabb = Aabb(Vector3(-27.1668f, -5.34938f, 39.589f), Vector3(-22.5781f, 1.68946f, 45.6287f)); 
  data[5].mAabb = Aabb(Vector3(-23.6659f, -13.1314f, 13.4281f), Vector3(-15.9486f, -8.19047f, 16.1498f)); 
  data[6].mAabb = Aabb(Vector3(-31.5861f, -24.8132f, 7.51121f), Vector3(-24.9508f, -21.4787f, 9.6241f)); 
  data[7].mAabb = Aabb(Vector3(-10.2126f, -15.5872f, -44.7527f), Vector3(-4.45806f, -10.5688f, -38.6207f)); 
  data[8].mAabb = Aabb(Vector3(-17.4071f, -16.5661f, -13.1345f), Vector3(-11.4948f, -10.2321f, -5.30776f)); 
  data[9].mAabb = Aabb(Vector3(23.6462f, -35.0334f, 22.4707f), Vector3(26.8302f, -28.0507f, 26.3993f)); 
  data[10].mAabb = Aabb(Vector3(41.2806f, -0.297922f, -22.0815f), Vector3(42.7679f, 1.22467f, -21.223f)); 
  data[11].mAabb = Aabb(Vector3(-16.0582f, 5.47159f, -47.5667f), Vector3(-14.8907f, 8.7189f, -42.9936f)); 
  data[12].mAabb = Aabb(Vector3(25.5213f, 15.8597f, -24.9743f), Vector3(31.2226f, 18.4119f, -20.1382f)); 
  data[13].mAabb = Aabb(Vector3(17.7194f, 27.814f, -18.496f), Vector3(25.1588f, 33.3332f, -17.2761f)); 
  data[14].mAabb = Aabb(Vector3(12.6888f, -22.2725f, -3.15162f), Vector3(18.9957f, -18.0023f, 1.94239f)); 
  data[15].mAabb = Aabb(Vector3(3.39901f, 4.70056f, -4.56953f), Vector3(6.92858f, 10.0755f, -2.93654f)); 
  data[16].mAabb = Aabb(Vector3(-7.71841f, 14.228f, -7.20002f), Vector3(-0.599557f, 18.4527f, -6.18584f)); 
  data[17].mAabb = Aabb(Vector3(-8.38441f, -6.31463f, 22.5323f), Vector3(-0.430333f, -2.16879f, 24.7182f)); 
  data[18].mAabb = Aabb(Vector3(-9.12317f, 7.45278f, 9.09613f), Vector3(-8.4903f, 15.3064f, 10.0001f)); 
  data[19].mAabb = Aabb(Vector3(-12.8411f, -12.9349f, 32.8659f), Vector3(-5.71223f, -6.51582f, 35.4742f)); 
  data[20].mAabb = Aabb(Vector3(-1.66535f, -0.241898f, 34.296f), Vector3(1.81459f, 4.53413f, 39.0716f)); 
  data[21].mAabb = Aabb(Vector3(13.4122f, -9.6467f, -41.0789f), Vector3(14.191f, -5.17962f, -38.738f)); 
  data[22].mAabb = Aabb(Vector3(27.357f, -11.7874f, 5.08385f), Vector3(27.931f, -5.59424f, 6.47517f)); 
  data[23].mAabb = Aabb(Vector3(43.3665f, 10.7754f, -17.4148f), Vector3(45.3606f, 18.3013f, -10.6529f)); 
  data[24].mAabb = Aabb(Vector3(3.4015f, -37.5941f, 26.2257f), Vector3(8.22368f, -35.9801f, 28.3625f)); 
  data[25].mAabb = Aabb(Vector3(-17.5064f, -26.8728f, -13.3006f), Vector3(-14.4022f, -20.4514f, -6.35474f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-15.8763f, 39.5379f, -45.588f), Vector3(-33.8427f, -23.008f, -39.0753f), Vector3(-33.8427f, -15.9583f, 28.6278f), Vector3(-15.8763f, 46.5876f, 22.1151f), Vector3(19.1039f, 1.91696f, -11.7731f), Vector3(17.4758f, -3.75107f, -11.1829f), Vector3(17.4758f, -2.86086f, -2.63359f), Vector3(19.1039f, 2.80717f, -3.22379f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(64); 
  data[0].mAabb = Aabb(Vector3(-0.905045f, 31.3153f, -35.5126f), Vector3(5.62329f, 35.9415f, -30.7447f)); 
  data[1].mAabb = Aabb(Vector3(-21.2223f, 29.8355f, -32.0153f), Vector3(-13.5999f, 35.8011f, -26.2568f)); 
  data[2].mAabb = Aabb(Vector3(7.40594f, 6.99876f, -6.75668f), Vector3(12.4951f, 9.19746f, -5.77613f)); 
  data[3].mAabb = Aabb(Vector3(15.6297f, 3.60303f, -11.7588f), Vector3(17.2984f, 9.8927f, -6.44888f)); 
  data[4].mAabb = Aabb(Vector3(-18.4709f, 11.0197f, 34.053f), Vector3(-12.1189f, 11.7045f, 38.7309f)); 
  data[5].mAabb = Aabb(Vector3(-21.464f, 11.2511f, 31.6697f), Vector3(-19.8994f, 15.1546f, 35.536f)); 
  data[6].mAabb = Aabb(Vector3(8.94615f, 18.7246f, -0.458482f), Vector3(16.1701f, 22.2236f, 3.76042f)); 
  data[7].mAabb = Aabb(Vector3(-10.6829f, -12.0027f, -3.86425f), Vector3(-8.75909f, -10.4761f, 3.11753f)); 
  data[8].mAabb = Aabb(Vector3(-43.1022f, -8.74642f, 6.1972f), Vector3(-37.2914f, -7.02111f, 11.7314f)); 
  data[9].mAabb = Aabb(Vector3(-40.6095f, -4.68237f, -26.7943f), Vector3(-39.0129f, -1.9131f, -19.6474f)); 
  data[10].mAabb = Aabb(Vector3(1.87839f, -28.4549f, 3.79107f), Vector3(8.25898f, -20.7664f, 10.263f)); 
  data[11].mAabb = Aabb(Vector3(-35.3282f, -21.257f, 7.16669f), Vector3(-31.0859f, -15.5975f, 9.42546f)); 
  data[12].mAabb = Aabb(Vector3(16.1072f, -26.3138f, 7.29975f), Vector3(22.8102f, -21.736f, 9.86584f)); 
  data[13].mAabb = Aabb(Vector3(-0.858234f, -6.54018f, 1.87353f), Vector3(2.20769f, -0.855184f, 2.6168f)); 
  data[14].mAabb = Aabb(Vector3(-4.83254f, 3.38961f, 21.4301f), Vector3(1.65103f, 10.9872f, 29.2743f)); 
  data[15].mAabb = Aabb(Vector3(-15.1066f, -18.1f, -44.3225f), Vector3(-9.57789f, -17.5873f, -37.8735f)); 
  data[16].mAabb = Aabb(Vector3(9.19704f, -3.42396f, -6.83471f), Vector3(12.6f, -0.727512f, -4.75384f)); 
  data[17].mAabb = Aabb(Vector3(28.0137f, -38.6965f, -18.8501f), Vector3(33.6581f, -31.1472f, -15.8336f)); 
  data[18].mAabb = Aabb(Vector3(21.8394f, -37.0475f, 4.43235f), Vector3(29.7151f, -33.1004f, 5.35004f)); 
  data[19].mAabb = Aabb(Vector3(1.01459f, 12.7911f, -0.0409539f), Vector3(6.28998f, 15.6907f, 6.80607f)); 
  data[20].mAabb = Aabb(Vector3(19.0895f, -31.4571f, 32.5903f), Vector3(24.126f, -28.227f, 33.2431f)); 
  data[21].mAabb = Aabb(Vector3(32.8058f, -35.5293f, 6.28525f), Vector3(40.0473f, -29.8327f, 9.45601f)); 
  data[22].mAabb = Aabb(Vector3(13.1094f, 8.79239f, -32.4512f), Vector3(14.5817f, 16.486f, -30.7321f)); 
  data[23].mAabb = Aabb(Vector3(3.22114f, 14.9881f, -36.7145f), Vector3(8.87228f, 18.5408f, -35.1777f)); 
  data[24].mAabb = Aabb(Vector3(38.0347f, 3.46794f, 11.2315f), Vector3(39.6428f, 10.51f, 18.2942f)); 
  data[25].mAabb = Aabb(Vector3(-10.9042f, 0.745424f, 8.29716f), Vector3(-8.23448f, 4.65005f, 10.729f)); 
  data[26].mAabb = Aabb(Vector3(30.8157f, -29.7568f, 19.7875f), Vector3(36.4775f, -24.4322f, 21.6183f)); 
  data[27].mAabb = Aabb(Vector3(-5.29924f, 45.326f, -5.91549f), Vector3(-4.15958f, 51.1346f, -0.973166f)); 
  data[28].mAabb = Aabb(Vector3(21.7504f, -30.3527f, 19.9382f), Vector3(25.0281f, -25.4725f, 20.5245f)); 
  data[29].mAabb = Aabb(Vector3(25.004f, -20.1837f, 16.5646f), Vector3(27.2619f, -18.0321f, 23.337f)); 
  data[30].mAabb = Aabb(Vector3(36.6658f, -2.76238f, 3.82471f), Vector3(38.2218f, -1.65844f, 5.08149f)); 
  data[31].mAabb = Aabb(Vector3(18.3067f, 2.5619f, -21.6341f), Vector3(21.6373f, 7.00138f, -16.7053f)); 
  data[32].mAabb = Aabb(Vector3(32.2662f, -17.9153f, 24.6492f), Vector3(33.0517f, -13.0172f, 25.0833f)); 
  data[33].mAabb = Aabb(Vector3(-37.3547f, -8.91369f, -0.198908f), Vector3(-35.3404f, -6.38912f, 1.38932f)); 
  data[34].mAabb = Aabb(Vector3(5.58827f, 19.3021f, 2.03542f), Vector3(7.78256f, 26.3695f, 2.47091f)); 
  data[35].mAabb = Aabb(Vector3(-35.6825f, 8.32435f, -26.1066f), Vector3(-33.6267f, 11.7612f, -25.4515f)); 
  data[36].mAabb = Aabb(Vector3(-35.0658f, -18.9125f, -32.7124f), Vector3(-28.4342f, -10.9878f, -29.5729f)); 
  data[37].mAabb = Aabb(Vector3(29.6717f, 26.8542f, 10.333f), Vector3(33.8148f, 27.7318f, 17.4598f)); 
  data[38].mAabb = Aabb(Vector3(-28.3751f, 19.5969f, -35.4026f), Vector3(-23.4824f, 23.1634f, -30.7968f)); 
  data[39].mAabb = Aabb(Vector3(-13.5048f, 37.7009f, 6.90356f), Vector3(-6.22337f, 42.0446f, 14.079f)); 
  data[40].mAabb = Aabb(Vector3(-33.1321f, -8.23265f, -11.2287f), Vector3(-30.9329f, -1.74839f, -10.5304f)); 
  data[41].mAabb = Aabb(Vector3(-30.0118f, 21.6557f, 11.4064f), Vector3(-25.7059f, 23.8941f, 13.2676f)); 
  data[42].mAabb = Aabb(Vector3(-18.166f, -42.2624f, 8.80683f), Vector3(-16.6193f, -35.0956f, 11.3249f)); 
  data[43].mAabb = Aabb(Vector3(-1.75765f, -20.8243f, -22.4684f), Vector3(2.7862f, -17.6869f, -18.4791f)); 
  data[44].mAabb = Aabb(Vector3(-4.60262f, -18.0342f, 16.7162f), Vector3(-0.968673f, -10.4661f, 20.1748f)); 
  data[45].mAabb = Aabb(Vector3(0.431617f, -15.3816f, 26.3294f), Vector3(6.86903f, -8.61418f, 28.9484f)); 
  data[46].mAabb = Aabb(Vector3(7.68029f, -15.0537f, 15.5924f), Vector3(8.96329f, -12.3635f, 18.6938f)); 
  data[47].mAabb = Aabb(Vector3(-5.47126f, 31.6633f, 7.35414f), Vector3(0.870368f, 34.9264f, 9.69919f)); 
  data[48].mAabb = Aabb(Vector3(11.6582f, 21.2574f, -17.8319f), Vector3(14.9269f, 24.411f, -10.8735f)); 
  data[49].mAabb = Aabb(Vector3(-2.04897f, 16.0322f, 4.61161f), Vector3(2.25969f, 20.2968f, 9.66852f)); 
  data[50].mAabb = Aabb(Vector3(-3.93443f, -6.8577f, 43.0856f), Vector3(0.726321f, -1.44569f, 43.9613f)); 
  data[51].mAabb = Aabb(Vector3(-5.59466f, 20.0641f, -24.7602f), Vector3(-3.7223f, 26.8798f, -21.4695f)); 
  data[52].mAabb = Aabb(Vector3(12.4925f, -2.18785f, -32.2815f), Vector3(19.4935f, 0.287322f, -30.7665f)); 
  data[53].mAabb = Aabb(Vector3(14.6275f, 6.67938f, -40.7875f), Vector3(20.4697f, 14.3076f, -35.8988f)); 
  data[54].mAabb = Aabb(Vector3(-0.560624f, -39.7663f, -9.01389f), Vector3(4.86135f, -37.1613f, -2.09729f)); 
  data[55].mAabb = Aabb(Vector3(-21.7209f, 25.7981f, 7.54831f), Vector3(-18.9976f, 32.4712f, 13.6234f)); 
  data[56].mAabb = Aabb(Vector3(-1.99052f, -3.25706f, 39.7636f), Vector3(-1.57127f, 1.1866f, 41.529f)); 
  data[57].mAabb = Aabb(Vector3(29.7414f, 28.804f, -14.5625f), Vector3(31.1156f, 32.4658f, -13.501f)); 
  data[58].mAabb = Aabb(Vector3(41.5679f, -18.2412f, 14.7366f), Vector3(46.1632f, -15.8046f, 15.7048f)); 
  data[59].mAabb = Aabb(Vector3(16.3161f, 31.1552f, -36.6696f), Vector3(18.426f, 33.5343f, -30.5701f)); 
  data[60].mAabb = Aabb(Vector3(34.1431f, -34.227f, -0.693685f), Vector3(37.4229f, -32.8092f, 4.26024f)); 
  data[61].mAabb = Aabb(Vector3(21.5898f, -35.0815f, -5.20943f), Vector3(26.6607f, -31.4489f, -1.52747f)); 
  data[62].mAabb = Aabb(Vector3(-33.9307f, 29.5582f, 11.4411f), Vector3(-27.3846f, 37.3706f, 13.7512f)); 
  data[63].mAabb = Aabb(Vector3(16.8074f, -16.8494f, 22.2371f), Vector3(24.0714f, -15.073f, 27.4569f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(87.7912f, 17.2901f, -3.92113f), Vector3(8.52278f, 57.2681f, 69.2781f), Vector3(-43.3232f, -45.5322f, 69.2781f), Vector3(35.9451f, -85.5102f, -3.92113f), Vector3(4.17042f, -2.05808f, 2.43951f), Vector3(0.585906f, -0.250278f, 5.74959f), Vector3(-1.78857f, -4.95839f, 5.74959f), Vector3(1.79595f, -6.76619f, 2.43951f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(-6.06128f, 4.76968f, 18.0221f), Vector3(0.900327f, 9.34786f, 24.3938f)); 
  data[1].mAabb = Aabb(Vector3(6.1067f, -4.45082f, -18.4811f), Vector3(13.7546f, -3.87686f, -17.6931f)); 
  data[2].mAabb = Aabb(Vector3(0.481201f, -39.4591f, -0.213717f), Vector3(1.44296f, -38.1622f, 6.91372f)); 
  data[3].mAabb = Aabb(Vector3(-20.1052f, -16.5347f, -10.7301f), Vector3(-19.1643f, -16.0361f, -4.36248f)); 
  data[4].mAabb = Aabb(Vector3(8.20067f, 46.6256f, 6.85767f), Vector3(8.85952f, 48.311f, 13.8916f)); 
  data[5].mAabb = Aabb(Vector3(17.6139f, -7.89949f, -45.7391f), Vector3(22.4293f, -3.62585f, -43.5866f)); 
  data[6].mAabb = Aabb(Vector3(9.09018f, 31.7174f, 25.9405f), Vector3(11.7811f, 35.3099f, 33.3906f)); 
  data[7].mAabb = Aabb(Vector3(-31.288f, 8.68721f, -28.3997f), Vector3(-25.1904f, 10.0194f, -21.6963f)); 
  data[8].mAabb = Aabb(Vector3(9.70125f, -33.27f, 2.67885f), Vector3(13.7891f, -27.6157f, 4.17198f)); 
  data[9].mAabb = Aabb(Vector3(-21.3486f, -1.76184f, -26.3508f), Vector3(-17.7307f, 3.71929f, -21.646f)); 
  data[10].mAabb = Aabb(Vector3(0.964293f, 6.85222f, 39.8997f), Vector3(3.8115f, 11.4626f, 47.6432f)); 
  data[11].mAabb = Aabb(Vector3(-15.7714f, 32.5641f, -34.1297f), Vector3(-9.91472f, 40.3767f, -26.6444f)); 
  data[12].mAabb = Aabb(Vector3(-1.90052f, 25.3065f, 3.34135f), Vector3(-0.284222f, 31.4201f, 6.00486f)); 
  data[13].mAabb = Aabb(Vector3(-3.50345f, -5.99397f, -0.864051f), Vector3(-0.904885f, 1.59387f, 0.345422f)); 
  data[14].mAabb = Aabb(Vector3(26.1355f, 37.1227f, -8.72009f), Vector3(28.9715f, 45.0657f, -4.51209f)); 
  data[15].mAabb = Aabb(Vector3(-17.8145f, 28.4283f, -8.83938f), Vector3(-10.5546f, 34.3389f, -2.12502f)); 
  data[16].mAabb = Aabb(Vector3(-28.1212f, -5.67617f, 39.1251f), Vector3(-25.4071f, -3.73909f, 39.5991f)); 
  data[17].mAabb = Aabb(Vector3(-38.519f, 7.19627f, -19.3256f), Vector3(-34.1728f, 9.99384f, -11.7816f)); 
  data[18].mAabb = Aabb(Vector3(-11.3177f, -26.656f, -25.2399f), Vector3(-8.5306f, -19.8976f, -18.9762f)); 
  data[19].mAabb = Aabb(Vector3(22.9089f, 5.34112f, 19.3407f), Vector3(28.0194f, 9.40184f, 19.8929f)); 
  data[20].mAabb = Aabb(Vector3(-27.4028f, -1.13052f, 23.0436f), Vector3(-26.5677f, 1.86536f, 25.2588f)); 
  data[21].mAabb = Aabb(Vector3(9.28116f, -32.0961f, 13.0929f), Vector3(17.1469f, -29.4804f, 14.7241f)); 
  data[22].mAabb = Aabb(Vector3(-22.5506f, -33.4056f, 0.66657f), Vector3(-19.8386f, -30.2761f, 2.03631f)); 
  data[23].mAabb = Aabb(Vector3(-30.9673f, 12.0474f, -1.78378f), Vector3(-25.7471f, 13.4714f, 0.745438f)); 
  data[24].mAabb = Aabb(Vector3(-3.68577f, -15.2792f, -3.83836f), Vector3(-2.03492f, -9.02108f, 1.59475f)); 
  data[25].mAabb = Aabb(Vector3(-21.6878f, 0.391718f, -10.3055f), Vector3(-19.7896f, 5.07984f, -2.58294f)); 
  data[26].mAabb = Aabb(Vector3(-5.12361f, -12.8427f, 14.5724f), Vector3(-0.5111f, -11.1336f, 18.968f)); 
  data[27].mAabb = Aabb(Vector3(-44.2746f, 18.2893f, -12.6746f), Vector3(-41.2715f, 24.7931f, -9.77386f)); 
  data[28].mAabb = Aabb(Vector3(3.52861f, -39.5082f, 4.36625f), Vector3(10.1618f, -32.4663f, 6.54106f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-36.4726f, -74.7351f, 94.7499f), Vector3(-57.5419f, 76.5284f, 82.0425f), Vector3(-57.5419f, 63.6058f, -71.7818f), Vector3(-36.4726f, -87.6577f, -59.0744f), Vector3(24.5626f, 0.315684f, 15.9724f), Vector3(23.3507f, 9.0166f, 15.2415f), Vector3(23.3507f, 8.18533f, 5.34648f), Vector3(24.5626f, -0.515582f, 6.07743f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(46); 
  data[0].mAabb = Aabb(Vector3(10.8902f, 21.3938f, 2.45571f), Vector3(16.5857f, 22.4154f, 9.76337f)); 
  data[1].mAabb = Aabb(Vector3(11.2259f, 24.2029f, -32.7242f), Vector3(18.2752f, 29.389f, -25.3078f)); 
  data[2].mAabb = Aabb(Vector3(18.0951f, -45.7992f, 3.60615f), Vector3(22.5888f, -39.2757f, 10.3736f)); 
  data[3].mAabb = Aabb(Vector3(-16.2698f, 38.8869f, -23.8499f), Vector3(-8.83971f, 44.8818f, -21.9381f)); 
  data[4].mAabb = Aabb(Vector3(24.0011f, -0.538118f, 3.32545f), Vector3(28.217f, 4.14351f, 10.3889f)); 
  data[5].mAabb = Aabb(Vector3(-19.4541f, -6.4058f, -21.9882f), Vector3(-16.1453f, -0.675573f, -15.4349f)); 
  data[6].mAabb = Aabb(Vector3(26.4601f, 5.69571f, -13.9772f), Vector3(29.7403f, 8.55985f, -9.15665f)); 
  data[7].mAabb = Aabb(Vector3(-3.63728f, 37.0202f, 8.90493f), Vector3(2.2887f, 43.1206f, 13.834f)); 
  data[8].mAabb = Aabb(Vector3(-13.5284f, -0.591776f, 37.0782f), Vector3(-11.8952f, 3.08601f, 40.9608f)); 
  data[9].mAabb = Aabb(Vector3(-43.684f, -2.60364f, -7.77891f), Vector3(-42.0876f, 4.33083f, -4.34999f)); 
  data[10].mAabb = Aabb(Vector3(41.7876f, 6.66416f, 4.86644f), Vector3(45.1073f, 14.3691f, 11.0153f)); 
  data[11].mAabb = Aabb(Vector3(-18.1173f, -21.3906f, 34.3568f), Vector3(-14.427f, -16.2954f, 37.5027f)); 
  data[12].mAabb = Aabb(Vector3(-23.9633f, 4.08299f, -16.867f), Vector3(-22.2918f, 11.6718f, -11.2933f)); 
  data[13].mAabb = Aabb(Vector3(13.5667f, -32.4768f, 25.5062f), Vector3(17.0749f, -24.5777f, 29.7812f)); 
  data[14].mAabb = Aabb(Vector3(25.2f, 1.39624f, -30.7893f), Vector3(30.0275f, 6.92074f, -27.6522f)); 
  data[15].mAabb = Aabb(Vector3(33.9267f, -14.6522f, 20.6922f), Vector3(35.1981f, -10.8547f, 22.6054f)); 
  data[16].mAabb = Aabb(Vector3(-20.8002f, -41.1383f, -22.4598f), Vector3(-17.3052f, -34.7153f, -18.8528f)); 
  data[17].mAabb = Aabb(Vector3(0.841881f, 11.6405f, -42.1214f), Vector3(6.67741f, 15.7288f, -37.7404f)); 
  data[18].mAabb = Aabb(Vector3(3.35715f, -3.78079f, -45.3499f), Vector3(10.4248f, -0.171654f, -41.0545f)); 
  data[19].mAabb = Aabb(Vector3(-1.8885f, 13.7557f, -16.5191f), Vector3(0.0506549f, 20.9697f, -13.9474f)); 
  data[20].mAabb = Aabb(Vector3(34.4587f, 16.1684f, -10.9499f), Vector3(38.8689f, 16.6821f, -5.92041f)); 
  data[21].mAabb = Aabb(Vector3(25.7368f, -37.847f, -7.6182f), Vector3(27.3411f, -30.8327f, -2.57313f)); 
  data[22].mAabb = Aabb(Vector3(-19.4499f, 35.0761f, 5.80794f), Vector3(-13.6234f, 36.7012f, 9.755f)); 
  data[23].mAabb = Aabb(Vector3(42.5545f, -19.7654f, 8.62f), Vector3(46.252f, -16.8712f, 15.1973f)); 
  data[24].mAabb = Aabb(Vector3(41.0692f, -5.50726f, -18.2394f), Vector3(48.3855f, -4.54364f, -17.5128f)); 
  data[25].mAabb = Aabb(Vector3(-8.72556f, 43.8908f, 4.63066f), Vector3(-7.66175f, 47.9587f, 9.50666f)); 
  data[26].mAabb = Aabb(Vector3(21.2367f, 0.281131f, -6.99342f), Vector3(24.4782f, 8.01765f, -4.72444f)); 
  data[27].mAabb = Aabb(Vector3(-30.7241f, 15.3853f, 17.4404f), Vector3(-27.0737f, 16.4273f, 19.0193f)); 
  data[28].mAabb = Aabb(Vector3(-11.029f, -38.7263f, 9.19028f), Vector3(-8.52133f, -38.0851f, 16.0287f)); 
  data[29].mAabb = Aabb(Vector3(15.2388f, 20.1576f, -39.6758f), Vector3(19.4647f, 23.061f, -36.2316f)); 
  data[30].mAabb = Aabb(Vector3(-38.5748f, 8.27786f, -29.5908f), Vector3(-31.3975f, 10.7549f, -22.3105f)); 
  data[31].mAabb = Aabb(Vector3(10.8513f, 18.4761f, 9.53545f), Vector3(17.9662f, 25.6464f, 14.9806f)); 
  data[32].mAabb = Aabb(Vector3(17.4105f, 4.62248f, -45.2825f), Vector3(24.836f, 11.5488f, -40.7597f)); 
  data[33].mAabb = Aabb(Vector3(-5.00174f, 45.3098f, -1.30715f), Vector3(-4.04833f, 46.08f, 0.777431f)); 
  data[34].mAabb = Aabb(Vector3(-33.517f, -20.3706f, -18.8893f), Vector3(-28.5659f, -18.0003f, -15.3085f)); 
  data[35].mAabb = Aabb(Vector3(-18.9676f, -37.1922f, -27.1442f), Vector3(-15.4827f, -33.882f, -26.105f)); 
  data[36].mAabb = Aabb(Vector3(8.64184f, 1.26154f, 40.6806f), Vector3(13.3998f, 8.26489f, 46.7499f)); 
  data[37].mAabb = Aabb(Vector3(-4.54863f, -29.5962f, -6.92386f), Vector3(1.65197f, -21.6001f, -5.07725f)); 
  data[38].mAabb = Aabb(Vector3(-9.81109f, 32.8463f, 16.8241f), Vector3(-7.19977f, 35.7459f, 18.6654f)); 
  data[39].mAabb = Aabb(Vector3(-2.07793f, -22.9382f, -31.8138f), Vector3(5.56836f, -16.7689f, -26.8664f)); 
  data[40].mAabb = Aabb(Vector3(3.79057f, -12.3265f, -26.8258f), Vector3(10.6844f, -9.81769f, -22.6619f)); 
  data[41].mAabb = Aabb(Vector3(18.1987f, 39.13f, -11.734f), Vector3(19.3989f, 43.1497f, -5.53754f)); 
  data[42].mAabb = Aabb(Vector3(-5.77374f, -29.4664f, 13.9139f), Vector3(-5.21208f, -23.4379f, 15.7551f)); 
  data[43].mAabb = Aabb(Vector3(-0.09231f, -42.4001f, -3.89257f), Vector3(7.24735f, -37.7617f, 0.261156f)); 
  data[44].mAabb = Aabb(Vector3(-28.7451f, -10.2511f, -33.9678f), Vector3(-25.2227f, -3.12463f, -32.0653f)); 
  data[45].mAabb = Aabb(Vector3(10.4812f, 6.89319f, 6.00299f), Vector3(11.4859f, 7.66685f, 11.6718f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-78.0467f, 97.8913f, 66.1002f), Vector3(1.27099f, 82.5163f, -114.162f), Vector3(1.27099f, -114.883f, -97.3251f), Vector3(-78.0467f, -99.5083f, 82.9369f), Vector3(41.4937f, -1.32045f, 22.8606f), Vector3(44.3781f, -1.87957f, 16.3052f), Vector3(44.3781f, -9.6115f, 16.9647f), Vector3(41.4937f, -9.05238f, 23.5201f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(-45.3729f, -11.4175f, -21.5042f), Vector3(-37.6943f, -6.61832f, -14.917f)); 
  data[1].mAabb = Aabb(Vector3(-30.0819f, 10.8547f, 20.7039f), Vector3(-24.8539f, 18.8359f, 27.1884f)); 
  data[2].mAabb = Aabb(Vector3(-9.70102f, -2.44166f, -7.93176f), Vector3(-6.1951f, 3.97349f, -7.11079f)); 
  data[3].mAabb = Aabb(Vector3(-23.9346f, 24.4171f, -14.2185f), Vector3(-17.9084f, 32.3227f, -6.64178f)); 
  data[4].mAabb = Aabb(Vector3(-19.7148f, -19.8763f, -14.2073f), Vector3(-13.9658f, -14.7756f, -7.7694f)); 
  data[5].mAabb = Aabb(Vector3(-21.7957f, -31.7872f, 7.18572f), Vector3(-17.955f, -30.7777f, 11.6676f)); 
  data[6].mAabb = Aabb(Vector3(-8.25708f, 18.6118f, -6.04019f), Vector3(-1.23239f, 22.5818f, 0.554245f)); 
  data[7].mAabb = Aabb(Vector3(30.3689f, 28.6054f, -10.4985f), Vector3(34.7814f, 31.3281f, -9.34465f)); 
  data[8].mAabb = Aabb(Vector3(34.075f, -17.0536f, 22.5762f), Vector3(41.7883f, -14.6698f, 28.0828f)); 
  data[9].mAabb = Aabb(Vector3(-6.35264f, 28.9901f, 3.31087f), Vector3(0.347797f, 30.3714f, 8.05744f)); 
  data[10].mAabb = Aabb(Vector3(13.3444f, -29.6044f, -15.5117f), Vector3(17.8491f, -23.0475f, -8.43719f)); 
  data[11].mAabb = Aabb(Vector3(42.1323f, -7.38548f, 23.0684f), Vector3(42.5617f, -4.75514f, 24.7309f)); 
  data[12].mAabb = Aabb(Vector3(-9.38313f, -44.9906f, -11.9553f), Vector3(-2.04022f, -44.4127f, -6.08495f)); 
  data[13].mAabb = Aabb(Vector3(2.9549f, 3.47004f, -40.6517f), Vector3(10.5504f, 6.23491f, -38.2779f)); 
  data[14].mAabb = Aabb(Vector3(-4.88371f, 47.7101f, -13.2642f), Vector3(-2.05784f, 49.7663f, -7.80854f)); 
  data[15].mAabb = Aabb(Vector3(41.9528f, 0.519215f, 12.7218f), Vector3(46.2317f, 2.44956f, 17.396f)); 
  data[16].mAabb = Aabb(Vector3(-1.42216f, 0.135445f, 10.1251f), Vector3(4.9081f, 3.73692f, 16.9116f)); 
  data[17].mAabb = Aabb(Vector3(-17.2655f, 7.39145f, -29.0782f), Vector3(-13.5221f, 12.0805f, -26.6496f)); 
  data[18].mAabb = Aabb(Vector3(-27.6043f, -39.1879f, -19.5684f), Vector3(-20.6875f, -31.2338f, -15.0955f)); 
  data[19].mAabb = Aabb(Vector3(-9.15485f, 19.92f, -33.7612f), Vector3(-1.18917f, 24.7871f, -30.2789f)); 
  data[20].mAabb = Aabb(Vector3(-1.99298f, -35.1241f, 33.8984f), Vector3(-1.50995f, -31.5465f, 35.9494f)); 
  data[21].mAabb = Aabb(Vector3(3.96749f, 36.0247f, 26.2279f), Vector3(6.14254f, 37.0199f, 27.6167f)); 
  data[22].mAabb = Aabb(Vector3(32.38f, -6.36541f, 26.3247f), Vector3(38.8051f, -1.43932f, 26.7597f)); 
  data[23].mAabb = Aabb(Vector3(32.5229f, 27.4631f, 11.204f), Vector3(36.077f, 28.7168f, 12.6276f)); 
  data[24].mAabb = Aabb(Vector3(24.3387f, -41.1325f, -1.69611f), Vector3(29.208f, -36.9361f, -0.589846f)); 
  data[25].mAabb = Aabb(Vector3(-10.8624f, 7.41843f, 15.537f), Vector3(-6.59577f, 10.8562f, 23.0791f)); 
  data[26].mAabb = Aabb(Vector3(-18.0024f, -22.0994f, 20.8621f), Vector3(-16.664f, -17.7186f, 21.8695f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-38.2387f, 5.9228f, 8.84647f), Vector3(-4.30484f, 18.9122f, -38.0927f), Vector3(-4.30484f, -41.6861f, -54.8619f), Vector3(-38.2387f, -54.6754f, -7.92271f), Vector3(10.0368f, -20.3393f, 2.24415f), Vector3(13.0186f, -19.1979f, -1.88037f), Vector3(13.0186f, -27.6139f, -4.20931f), Vector3(10.0368f, -28.7553f, -0.0847944f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(30); 
  data[0].mAabb = Aabb(Vector3(-31.1341f, -29.843f, 18.734f), Vector3(-26.2889f, -23.4207f, 20.371f)); 
  data[1].mAabb = Aabb(Vector3(-10.9317f, 14.2473f, -20.7185f), Vector3(-9.8185f, 20.0014f, -13.9721f)); 
  data[2].mAabb = Aabb(Vector3(-34.6804f, -11.1454f, -33.3679f), Vector3(-29.4275f, -9.15102f, -26.4823f)); 
  data[3].mAabb = Aabb(Vector3(-29.6816f, -6.85945f, 8.06726f), Vector3(-25.8853f, -5.61889f, 10.6668f)); 
  data[4].mAabb = Aabb(Vector3(-8.62706f, -21.5757f, -16.8907f), Vector3(-2.13328f, -19.8685f, -13.1115f)); 
  data[5].mAabb = Aabb(Vector3(-2.53182f, -27.1967f, -33.2806f), Vector3(3.32482f, -20.1971f, -30.7707f)); 
  data[6].mAabb = Aabb(Vector3(21.2617f, -9.73857f, -32.1015f), Vector3(29.2246f, -2.44088f, -25.7601f)); 
  data[7].mAabb = Aabb(Vector3(10.3314f, 22.9555f, -28.9787f), Vector3(13.2747f, 25.0291f, -20.9977f)); 
  data[8].mAabb = Aabb(Vector3(33.4155f, 1.52184f, 29.7346f), Vector3(34.0433f, 6.7527f, 35.8039f)); 
  data[9].mAabb = Aabb(Vector3(-8.02517f, 4.89732f, 22.0926f), Vector3(-6.32885f, 6.60036f, 22.9008f)); 
  data[10].mAabb = Aabb(Vector3(-8.39857f, 17.731f, -1.77335f), Vector3(-0.642344f, 24.7024f, 0.399602f)); 
  data[11].mAabb = Aabb(Vector3(24.2534f, 10.57f, 8.29898f), Vector3(27.2011f, 14.7462f, 11.0773f)); 
  data[12].mAabb = Aabb(Vector3(22.4302f, -23.4776f, 19.6596f), Vector3(29.3953f, -18.9982f, 24.5922f)); 
  data[13].mAabb = Aabb(Vector3(10.9862f, 26.332f, -30.7646f), Vector3(14.8611f, 28.7455f, -28.8006f)); 
  data[14].mAabb = Aabb(Vector3(-19.3347f, -36.4763f, 3.00855f), Vector3(-13.3247f, -32.3773f, 4.49543f)); 
  data[15].mAabb = Aabb(Vector3(-26.2039f, -28.7446f, -15.1888f), Vector3(-21.578f, -23.7948f, -12.3562f)); 
  data[16].mAabb = Aabb(Vector3(-11.5148f, 32.7413f, 25.1089f), Vector3(-10.1949f, 34.5392f, 31.4737f)); 
  data[17].mAabb = Aabb(Vector3(31.956f, -24.0074f, 7.60669f), Vector3(33.0773f, -16.8361f, 12.7842f)); 
  data[18].mAabb = Aabb(Vector3(15.7738f, 35.6682f, 2.57088f), Vector3(16.2721f, 37.7897f, 6.24054f)); 
  data[19].mAabb = Aabb(Vector3(-35.2535f, -17.9599f, -8.62314f), Vector3(-32.5026f, -16.0817f, -6.73594f)); 
  data[20].mAabb = Aabb(Vector3(-23.9383f, -18.3816f, 25.7477f), Vector3(-18.9451f, -13.7568f, 27.7831f)); 
  data[21].mAabb = Aabb(Vector3(20.472f, 31.5895f, 9.22854f), Vector3(23.9491f, 32.0171f, 14.9926f)); 
  data[22].mAabb = Aabb(Vector3(-38.7964f, -5.0959f, -9.55563f), Vector3(-33.8385f, 1.84853f, -7.89433f)); 
  data[23].mAabb = Aabb(Vector3(-32.4442f, 24.7115f, 24.6661f), Vector3(-26.5915f, 26.235f, 30.0683f)); 
  data[24].mAabb = Aabb(Vector3(-20.2452f, 11.4439f, 31.8448f), Vector3(-16.1355f, 12.9164f, 35.1833f)); 
  data[25].mAabb = Aabb(Vector3(-4.7895f, 17.3859f, -31.9708f), Vector3(-1.11612f, 17.9019f, -27.227f)); 
  data[26].mAabb = Aabb(Vector3(19.636f, -8.6114f, 34.3572f), Vector3(24.6361f, -1.41531f, 37.5417f)); 
  data[27].mAabb = Aabb(Vector3(3.371f, -7.22338f, -10.1453f), Vector3(10.9408f, -0.552404f, -9.42779f)); 
  data[28].mAabb = Aabb(Vector3(-32.3004f, -17.0226f, 26.0532f), Vector3(-29.3913f, -9.76014f, 30.2468f)); 
  data[29].mAabb = Aabb(Vector3(-10.9813f, -7.67795f, -8.48532f), Vector3(-6.92428f, -6.00576f, -0.504569f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-71.8733f, 83.8443f, 64.3284f), Vector3(-10.9549f, 95.2291f, -69.5853f), Vector3(-10.9549f, -50.6922f, -81.9909f), Vector3(-71.8733f, -62.0769f, 51.9228f), Vector3(48.0619f, 15.3944f, 35.1958f), Vector3(50.6278f, 15.8739f, 29.5554f), Vector3(50.6278f, 10.7881f, 29.1231f), Vector3(48.0619f, 10.3086f, 34.7634f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(53); 
  data[0].mAabb = Aabb(Vector3(-29.5129f, -7.95798f, -12.6039f), Vector3(-22.2416f, -0.848862f, -10.5298f)); 
  data[1].mAabb = Aabb(Vector3(-33.8277f, -24.6815f, -11.6303f), Vector3(-32.7964f, -19.7078f, -10.3599f)); 
  data[2].mAabb = Aabb(Vector3(-33.5807f, -13.7296f, -5.48548f), Vector3(-27.0334f, -9.83978f, 0.495013f)); 
  data[3].mAabb = Aabb(Vector3(-31.6827f, 39.5049f, -4.88087f), Vector3(-24.7524f, 41.1232f, 1.47583f)); 
  data[4].mAabb = Aabb(Vector3(-18.8895f, -15.3257f, 18.1109f), Vector3(-14.7056f, -11.342f, 24.0626f)); 
  data[5].mAabb = Aabb(Vector3(1.3462f, -18.368f, -7.83231f), Vector3(3.59337f, -17.4721f, -1.2606f)); 
  data[6].mAabb = Aabb(Vector3(11.6778f, -11.3942f, -22.1781f), Vector3(14.7982f, -10.1699f, -21.0512f)); 
  data[7].mAabb = Aabb(Vector3(-5.65105f, 30.852f, 9.01677f), Vector3(0.145285f, 34.2862f, 16.4365f)); 
  data[8].mAabb = Aabb(Vector3(26.3575f, 20.0789f, 18.0662f), Vector3(27.0052f, 25.8204f, 22.5833f)); 
  data[9].mAabb = Aabb(Vector3(10.3075f, 1.79911f, -17.5476f), Vector3(11.5881f, 5.6657f, -15.9753f)); 
  data[10].mAabb = Aabb(Vector3(28.6147f, -3.30082f, -41.0283f), Vector3(34.2053f, -0.443874f, -35.2628f)); 
  data[11].mAabb = Aabb(Vector3(-6.47212f, -6.81243f, -35.684f), Vector3(-3.83528f, -3.96453f, -27.8099f)); 
  data[12].mAabb = Aabb(Vector3(11.2683f, -30.442f, -16.867f), Vector3(13.1608f, -26.0579f, -10.5673f)); 
  data[13].mAabb = Aabb(Vector3(11.7775f, -4.60946f, 31.0791f), Vector3(13.7884f, 1.76672f, 35.1001f)); 
  data[14].mAabb = Aabb(Vector3(-2.38311f, -34.5398f, -27.4526f), Vector3(-0.572448f, -28.6482f, -25.262f)); 
  data[15].mAabb = Aabb(Vector3(-5.49113f, -3.76365f, -28.1031f), Vector3(0.784856f, -1.07463f, -25.5765f)); 
  data[16].mAabb = Aabb(Vector3(21.3969f, 5.04297f, -26.4131f), Vector3(22.733f, 6.73813f, -19.7841f)); 
  data[17].mAabb = Aabb(Vector3(13.1353f, -17.6133f, -35.1585f), Vector3(20.1426f, -15.6215f, -28.5721f)); 
  data[18].mAabb = Aabb(Vector3(32.892f, -1.20947f, 2.33395f), Vector3(36.7409f, 6.72883f, 4.86455f)); 
  data[19].mAabb = Aabb(Vector3(-16.4039f, 19.3758f, 13.9772f), Vector3(-10.1811f, 27.0571f, 18.2309f)); 
  data[20].mAabb = Aabb(Vector3(13.7036f, -42.0239f, 8.12114f), Vector3(14.3286f, -39.6988f, 8.90917f)); 
  data[21].mAabb = Aabb(Vector3(30.4447f, 17.3717f, 21.7457f), Vector3(34.0518f, 21.7005f, 25.2885f)); 
  data[22].mAabb = Aabb(Vector3(17.7953f, -40.4371f, 9.86721f), Vector3(23.5976f, -35.967f, 13.3409f)); 
  data[23].mAabb = Aabb(Vector3(-39.6465f, -14.0336f, -12.8052f), Vector3(-32.9219f, -8.14774f, -11.9406f)); 
  data[24].mAabb = Aabb(Vector3(30.3811f, 3.30094f, 0.501851f), Vector3(34.4037f, 11.1235f, 2.41573f)); 
  data[25].mAabb = Aabb(Vector3(-42.2979f, 18.386f, 9.39785f), Vector3(-34.3561f, 24.5555f, 16.4114f)); 
  data[26].mAabb = Aabb(Vector3(-16.0519f, -33.9117f, -29.7887f), Vector3(-8.89133f, -31.5799f, -23.2453f)); 
  data[27].mAabb = Aabb(Vector3(1.02116f, -35.7033f, 23.0173f), Vector3(5.78675f, -31.4043f, 30.934f)); 
  data[28].mAabb = Aabb(Vector3(23.0942f, -6.16938f, 30.9571f), Vector3(25.443f, -2.15226f, 33.8067f)); 
  data[29].mAabb = Aabb(Vector3(-18.5007f, -23.2323f, 2.34278f), Vector3(-11.1877f, -15.2972f, 9.49921f)); 
  data[30].mAabb = Aabb(Vector3(33.6012f, -0.253605f, -19.3788f), Vector3(36.5214f, 3.54363f, -14.2519f)); 
  data[31].mAabb = Aabb(Vector3(-4.08977f, 19.1977f, 15.7654f), Vector3(-0.867976f, 23.7937f, 20.4444f)); 
  data[32].mAabb = Aabb(Vector3(-11.6381f, -11.6927f, 42.7737f), Vector3(-4.36936f, -9.1394f, 46.3188f)); 
  data[33].mAabb = Aabb(Vector3(3.28131f, 10.2333f, -9.88747f), Vector3(9.40143f, 11.7371f, -4.38292f)); 
  data[34].mAabb = Aabb(Vector3(-24.3423f, -33.8534f, 7.18777f), Vector3(-23.0927f, -27.0767f, 13.8386f)); 
  data[35].mAabb = Aabb(Vector3(-15.6472f, 21.4848f, 34.5693f), Vector3(-9.6869f, 24.1121f, 38.6154f)); 
  data[36].mAabb = Aabb(Vector3(-30.2765f, -3.18557f, -40.7931f), Vector3(-23.5457f, 4.1868f, -35.5834f)); 
  data[37].mAabb = Aabb(Vector3(6.90269f, -33.2795f, 29.3281f), Vector3(7.43861f, -32.6956f, 36.7833f)); 
  data[38].mAabb = Aabb(Vector3(-23.0688f, -21.1972f, 17.8833f), Vector3(-15.1671f, -15.2221f, 24.1904f)); 
  data[39].mAabb = Aabb(Vector3(-32.1382f, -16.1963f, -11.9423f), Vector3(-29.6468f, -12.133f, -6.33991f)); 
  data[40].mAabb = Aabb(Vector3(7.62184f, 6.4845f, -27.149f), Vector3(8.78029f, 11.4296f, -25.5362f)); 
  data[41].mAabb = Aabb(Vector3(-24.043f, -18.9959f, -30.9709f), Vector3(-20.5088f, -15.3635f, -26.6623f)); 
  data[42].mAabb = Aabb(Vector3(10.9356f, -10.7021f, 35.3369f), Vector3(18.7136f, -8.10166f, 38.8676f)); 
  data[43].mAabb = Aabb(Vector3(-14.805f, 8.26611f, 8.00364f), Vector3(-11.8347f, 11.2555f, 14.6403f)); 
  data[44].mAabb = Aabb(Vector3(3.27954f, 15.2849f, -16.496f), Vector3(9.66616f, 16.3401f, -10.9594f)); 
  data[45].mAabb = Aabb(Vector3(18.586f, 8.06751f, 3.87555f), Vector3(24.8509f, 11.392f, 9.53295f)); 
  data[46].mAabb = Aabb(Vector3(-16.5471f, 10.6813f, -28.9707f), Vector3(-12.1025f, 18.1019f, -28.5452f)); 
  data[47].mAabb = Aabb(Vector3(-18.194f, -22.799f, -30.0147f), Vector3(-16.2451f, -15.7467f, -27.3103f)); 
  data[48].mAabb = Aabb(Vector3(-11.3241f, 17.0048f, -45.3858f), Vector3(-10.7722f, 22.337f, -41.4098f)); 
  data[49].mAabb = Aabb(Vector3(30.2672f, 19.9432f, 3.45718f), Vector3(35.2485f, 24.5195f, 3.98173f)); 
  data[50].mAabb = Aabb(Vector3(12.0391f, -29.9864f, -30.0099f), Vector3(15.6327f, -23.215f, -25.0681f)); 
  data[51].mAabb = Aabb(Vector3(9.95287f, 33.7466f, 4.60843f), Vector3(17.63f, 36.5004f, 5.15246f)); 
  data[52].mAabb = Aabb(Vector3(-27.2536f, 23.2917f, 13.2355f), Vector3(-22.322f, 24.825f, 14.9272f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-77.7404f, 56.4963f, 5.61071f), Vector3(-12.2864f, -4.60312f, -84.7009f), Vector3(-12.2864f, -110.519f, -13.0446f), Vector3(-77.7404f, -49.4198f, 77.2671f), Vector3(29.5611f, 3.95773f, 34.6952f), Vector3(33.4467f, 0.330643f, 29.334f), Vector3(33.4467f, -6.50461f, 33.9583f), Vector3(29.5611f, -2.87752f, 39.3195f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(-5.6302f, 6.72067f, -17.6436f), Vector3(-1.92969f, 14.4743f, -12.3457f)); 
  data[1].mAabb = Aabb(Vector3(41.8223f, 6.28742f, -7.73045f), Vector3(42.5036f, 8.61995f, -5.35339f)); 
  data[2].mAabb = Aabb(Vector3(-9.13481f, -11.1681f, -20.6517f), Vector3(-6.81829f, -5.45429f, -18.4611f)); 
  data[3].mAabb = Aabb(Vector3(23.1589f, 2.48713f, -31.3444f), Vector3(26.6432f, 9.66884f, -28.7971f)); 
  data[4].mAabb = Aabb(Vector3(-27.0751f, -34.279f, 17.7872f), Vector3(-25.5868f, -26.3495f, 20.7722f)); 
  data[5].mAabb = Aabb(Vector3(-37.9965f, 4.58391f, -23.9923f), Vector3(-35.4056f, 9.21404f, -17.4665f)); 
  data[6].mAabb = Aabb(Vector3(-16.4787f, -13.9656f, 23.8832f), Vector3(-14.0116f, -6.17222f, 25.7484f)); 
  data[7].mAabb = Aabb(Vector3(-0.605137f, 41.5522f, 19.7701f), Vector3(4.93096f, 46.071f, 25.0255f)); 
  data[8].mAabb = Aabb(Vector3(-29.9534f, -13.1493f, 32.0522f), Vector3(-25.016f, -5.89177f, 33.4094f)); 
  data[9].mAabb = Aabb(Vector3(24.9345f, 4.41634f, 32.0325f), Vector3(31.9601f, 7.87401f, 34.151f)); 
  data[10].mAabb = Aabb(Vector3(8.54443f, -20.2927f, -45.265f), Vector3(15.1597f, -18.4034f, -38.4511f)); 
  data[11].mAabb = Aabb(Vector3(-31.9938f, -4.41077f, 11.9775f), Vector3(-30.0015f, 1.10283f, 18.9034f)); 
  data[12].mAabb = Aabb(Vector3(37.8049f, 4.41035f, 15.138f), Vector3(44.2282f, 5.20512f, 22.6323f)); 
  data[13].mAabb = Aabb(Vector3(8.95017f, 16.3988f, -9.54922f), Vector3(16.2822f, 18.0443f, -7.89512f)); 
  data[14].mAabb = Aabb(Vector3(-4.87498f, 19.5527f, -44.0502f), Vector3(2.33178f, 21.8741f, -37.3453f)); 
  data[15].mAabb = Aabb(Vector3(9.13565f, -33.2511f, -34.9544f), Vector3(15.7241f, -27.7454f, -31.8516f)); 
  data[16].mAabb = Aabb(Vector3(-27.8462f, 34.486f, -25.855f), Vector3(-25.2945f, 36.7411f, -18.1969f)); 
  data[17].mAabb = Aabb(Vector3(-27.7706f, -16.6501f, -24.7431f), Vector3(-26.1257f, -12.8403f, -22.9194f)); 
  data[18].mAabb = Aabb(Vector3(-5.07249f, 10.2036f, 17.098f), Vector3(-0.605412f, 11.2254f, 19.4157f)); 
  data[19].mAabb = Aabb(Vector3(34.1382f, 4.52675f, -5.10481f), Vector3(36.2269f, 7.02395f, -3.81022f)); 
  data[20].mAabb = Aabb(Vector3(-3.95832f, -23.698f, -43.8644f), Vector3(1.33586f, -20.6896f, -37.176f)); 
  data[21].mAabb = Aabb(Vector3(-31.3853f, 4.6714f, -8.19011f), Vector3(-24.2776f, 8.76251f, -3.09703f)); 
  data[22].mAabb = Aabb(Vector3(3.19045f, 33.3686f, 25.0989f), Vector3(9.79857f, 38.281f, 32.5133f)); 
  data[23].mAabb = Aabb(Vector3(-9.15826f, 41.365f, 16.3033f), Vector3(-8.57572f, 47.631f, 17.5367f)); 
  data[24].mAabb = Aabb(Vector3(-33.4224f, 30.4078f, 3.47068f), Vector3(-32.5544f, 34.6425f, 11.0196f)); 
  data[25].mAabb = Aabb(Vector3(25.7842f, -4.91711f, 26.3345f), Vector3(32.4258f, -3.47268f, 30.5708f)); 
  data[26].mAabb = Aabb(Vector3(27.7849f, 36.2676f, 7.24726f), Vector3(28.3714f, 38.2979f, 13.2053f)); 
  data[27].mAabb = Aabb(Vector3(16.074f, -10.3478f, 21.5923f), Vector3(22.0037f, -3.71931f, 28.0738f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(0.647493f, 64.1223f, -4.98806f), Vector3(-52.6825f, 7.97603f, -33.6299f), Vector3(-52.6825f, -28.9322f, 38.7209f), Vector3(0.647493f, 27.214f, 67.3627f), Vector3(33.9245f, -22.8764f, -6.4959f), Vector3(29.9307f, -27.0811f, -8.64083f), Vector3(29.9307f, -29.2804f, -4.32947f), Vector3(33.9245f, -25.0758f, -2.18455f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(58); 
  data[0].mAabb = Aabb(Vector3(-8.37952f, 19.4971f, 36.9167f), Vector3(-7.62303f, 26.9089f, 37.7226f)); 
  data[1].mAabb = Aabb(Vector3(21.1346f, 1.78156f, 4.88851f), Vector3(27.0096f, 9.33578f, 12.1449f)); 
  data[2].mAabb = Aabb(Vector3(12.5397f, 32.2431f, -31.6689f), Vector3(19.5718f, 34.8069f, -24.8418f)); 
  data[3].mAabb = Aabb(Vector3(-1.87875f, 42.5554f, -19.7966f), Vector3(3.99227f, 48.3316f, -16.7541f)); 
  data[4].mAabb = Aabb(Vector3(16.227f, -15.9108f, 2.08245f), Vector3(24.0707f, -14.3079f, 7.77139f)); 
  data[5].mAabb = Aabb(Vector3(-29.302f, -21.5283f, 32.5744f), Vector3(-22.1332f, -15.9366f, 36.0128f)); 
  data[6].mAabb = Aabb(Vector3(-25.5763f, 32.941f, 12.0428f), Vector3(-20.4633f, 37.7426f, 17.1039f)); 
  data[7].mAabb = Aabb(Vector3(-8.50479f, 2.35907f, -50.5026f), Vector3(-2.15829f, 5.50525f, -45.1337f)); 
  data[8].mAabb = Aabb(Vector3(-19.2303f, 0.330411f, 0.188756f), Vector3(-11.9245f, 6.72863f, 5.31246f)); 
  data[9].mAabb = Aabb(Vector3(-5.29526f, 13.9634f, 29.634f), Vector3(-3.672f, 20.5474f, 32.2297f)); 
  data[10].mAabb = Aabb(Vector3(-47.8507f, -6.15697f, 4.49278f), Vector3(-47.1775f, 1.56145f, 8.3487f)); 
  data[11].mAabb = Aabb(Vector3(-37.0085f, -30.7202f, -11.9046f), Vector3(-30.0889f, -26.4022f, -7.94616f)); 
  data[12].mAabb = Aabb(Vector3(-16.3142f, 22.9595f, -26.2927f), Vector3(-14.7547f, 30.4702f, -24.0152f)); 
  data[13].mAabb = Aabb(Vector3(21.1541f, -20.5018f, 4.16351f), Vector3(22.6625f, -15.5788f, 8.10942f)); 
  data[14].mAabb = Aabb(Vector3(31.2967f, 16.0501f, -19.393f), Vector3(33.6744f, 23.3095f, -16.6333f)); 
  data[15].mAabb = Aabb(Vector3(17.1647f, -6.85795f, -41.607f), Vector3(19.3402f, -0.947276f, -34.5842f)); 
  data[16].mAabb = Aabb(Vector3(12.9931f, -14.3509f, -0.46963f), Vector3(20.6707f, -9.71306f, 4.44695f)); 
  data[17].mAabb = Aabb(Vector3(-10.9006f, -39.0881f, -17.3142f), Vector3(-3.10588f, -35.8389f, -11.5988f)); 
  data[18].mAabb = Aabb(Vector3(14.1845f, -17.3322f, -14.5723f), Vector3(17.4939f, -12.0686f, -7.18368f)); 
  data[19].mAabb = Aabb(Vector3(2.68601f, -7.98921f, -51.1002f), Vector3(5.91963f, -2.69433f, -44.8291f)); 
  data[20].mAabb = Aabb(Vector3(32.8003f, 12.2058f, 0.293515f), Vector3(34.7017f, 15.3242f, 5.12544f)); 
  data[21].mAabb = Aabb(Vector3(-4.06943f, -30.427f, 28.3561f), Vector3(-3.56389f, -27.034f, 30.2618f)); 
  data[22].mAabb = Aabb(Vector3(-11.5748f, -6.97276f, -7.30751f), Vector3(-3.91504f, -1.57629f, -3.68005f)); 
  data[23].mAabb = Aabb(Vector3(-19.7112f, -26.0201f, -27.4816f), Vector3(-12.0389f, -18.8082f, -24.8316f)); 
  data[24].mAabb = Aabb(Vector3(-0.784545f, -21.8511f, -17.6121f), Vector3(4.92203f, -15.3103f, -9.9953f)); 
  data[25].mAabb = Aabb(Vector3(-12.6181f, -42.7426f, 2.54178f), Vector3(-10.3994f, -36.8693f, 9.14874f)); 
  data[26].mAabb = Aabb(Vector3(-18.1869f, 14.4962f, 30.866f), Vector3(-16.8206f, 16.9299f, 35.85f)); 
  data[27].mAabb = Aabb(Vector3(-43.9596f, 10.2096f, -15.3757f), Vector3(-39.3536f, 11.153f, -8.94221f)); 
  data[28].mAabb = Aabb(Vector3(-7.46742f, -29.2868f, 32.8557f), Vector3(-5.65212f, -26.5864f, 39.2481f)); 
  data[29].mAabb = Aabb(Vector3(-27.7107f, -26.7247f, -18.4475f), Vector3(-25.7931f, -23.9833f, -11.3871f)); 
  data[30].mAabb = Aabb(Vector3(-2.76496f, -30.8323f, -3.60752f), Vector3(0.738407f, -27.9401f, 2.28089f)); 
  data[31].mAabb = Aabb(Vector3(22.3739f, 3.6713f, -28.0668f), Vector3(29.6862f, 10.5053f, -24.0073f)); 
  data[32].mAabb = Aabb(Vector3(0.364123f, 2.94299f, 14.6937f), Vector3(7.32712f, 6.65047f, 20.5798f)); 
  data[33].mAabb = Aabb(Vector3(11.1525f, 8.28196f, 23.6459f), Vector3(16.3456f, 10.0581f, 28.7202f)); 
  data[34].mAabb = Aabb(Vector3(1.02842f, 32.4706f, 11.8683f), Vector3(1.84753f, 33.4857f, 12.4137f)); 
  data[35].mAabb = Aabb(Vector3(4.4552f, -14.5731f, -49.4936f), Vector3(11.4637f, -12.4143f, -43.771f)); 
  data[36].mAabb = Aabb(Vector3(4.88621f, -37.4352f, -30.6379f), Vector3(9.89557f, -33.99f, -24.9822f)); 
  data[37].mAabb = Aabb(Vector3(31.9184f, -7.76048f, -10.6255f), Vector3(37.5134f, -1.73081f, -8.23781f)); 
  data[38].mAabb = Aabb(Vector3(-41.2035f, -23.4563f, -20.8287f), Vector3(-40.1764f, -22.1531f, -14.2927f)); 
  data[39].mAabb = Aabb(Vector3(14.5028f, 9.75447f, -6.91222f), Vector3(21.88f, 17.3003f, -0.402667f)); 
  data[40].mAabb = Aabb(Vector3(24.861f, -21.7894f, 11.3451f), Vector3(31.2657f, -17.4118f, 14.6128f)); 
  data[41].mAabb = Aabb(Vector3(39.6204f, 22.8625f, -1.57592f), Vector3(45.9539f, 24.7854f, 4.40782f)); 
  data[42].mAabb = Aabb(Vector3(-14.1892f, -38.0058f, 22.6777f), Vector3(-6.94622f, -34.0084f, 24.6463f)); 
  data[43].mAabb = Aabb(Vector3(8.28029f, 37.7987f, -23.702f), Vector3(11.3242f, 44.9433f, -18.2118f)); 
  data[44].mAabb = Aabb(Vector3(-40.3857f, -3.44548f, -1.71984f), Vector3(-34.3827f, 3.86125f, 0.503448f)); 
  data[45].mAabb = Aabb(Vector3(28.6267f, -1.11733f, -4.20931f), Vector3(29.7575f, 0.372795f, 3.5446f)); 
  data[46].mAabb = Aabb(Vector3(34.8519f, 8.13927f, 15.3074f), Vector3(39.6263f, 14.8525f, 19.4885f)); 
  data[47].mAabb = Aabb(Vector3(28.9748f, 5.51137f, -21.0317f), Vector3(29.7784f, 12.0599f, -14.2747f)); 
  data[48].mAabb = Aabb(Vector3(-25.5769f, -17.3891f, 40.6125f), Vector3(-19.1508f, -15.3084f, 42.4473f)); 
  data[49].mAabb = Aabb(Vector3(31.5484f, 18.5994f, 17.8469f), Vector3(36.9718f, 20.5748f, 24.7594f)); 
  data[50].mAabb = Aabb(Vector3(27.7102f, -24.1254f, 21.8322f), Vector3(33.571f, -17.104f, 23.0243f)); 
  data[51].mAabb = Aabb(Vector3(-52.5312f, -4.72017f, 0.152235f), Vector3(-44.9348f, -4.25731f, 2.56409f)); 
  data[52].mAabb = Aabb(Vector3(-11.1034f, -39.1965f, -22.6103f), Vector3(-10.5258f, -34.3437f, -15.5577f)); 
  data[53].mAabb = Aabb(Vector3(-37.8775f, -11.6018f, 4.27998f), Vector3(-36.7659f, -4.42522f, 11.0405f)); 
  data[54].mAabb = Aabb(Vector3(33.8118f, -20.5971f, 11.9248f), Vector3(41.7735f, -14.6139f, 13.8273f)); 
  data[55].mAabb = Aabb(Vector3(5.3751f, 27.099f, -21.6821f), Vector3(5.80943f, 29.2265f, -15.7735f)); 
  data[56].mAabb = Aabb(Vector3(-8.80173f, 8.52073f, -4.17856f), Vector3(-4.29383f, 13.6076f, 0.862568f)); 
  data[57].mAabb = Aabb(Vector3(-18.4644f, -14.2497f, 4.17954f), Vector3(-10.8f, -10.108f, 5.17888f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-21.8489f, -24.5096f, -12.0373f), Vector3(-32.6508f, -22.0273f, 43.9401f), Vector3(-32.6508f, 35.7227f, 41.3791f), Vector3(-21.8489f, 33.2403f, -14.5982f), Vector3(9.21279f, 1.57735f, 18.0101f), Vector3(7.77347f, 1.90812f, 25.469f), Vector3(7.77347f, 10.2462f, 25.0992f), Vector3(9.21279f, 9.91548f, 17.6404f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(37); 
  data[0].mAabb = Aabb(Vector3(-12.9044f, 7.70975f, -46.3491f), Vector3(-10.7857f, 11.5362f, -41.632f)); 
  data[1].mAabb = Aabb(Vector3(24.588f, -18.1085f, -38.3554f), Vector3(28.5826f, -12.1414f, -36.7421f)); 
  data[2].mAabb = Aabb(Vector3(16.7128f, 12.4129f, 23.597f), Vector3(18.8032f, 15.3436f, 27.3988f)); 
  data[3].mAabb = Aabb(Vector3(20.6607f, 14.2659f, -20.8426f), Vector3(24.5896f, 18.1329f, -12.9404f)); 
  data[4].mAabb = Aabb(Vector3(33.7261f, 0.522872f, 20.0954f), Vector3(35.1181f, 4.19068f, 25.1504f)); 
  data[5].mAabb = Aabb(Vector3(-14.6051f, -36.5343f, 22.7003f), Vector3(-9.29194f, -32.8082f, 23.7252f)); 
  data[6].mAabb = Aabb(Vector3(-10.5988f, 21.0158f, -25.5434f), Vector3(-7.99049f, 24.1364f, -19.022f)); 
  data[7].mAabb = Aabb(Vector3(0.0320148f, -41.7195f, -1.48827f), Vector3(6.25558f, -39.4327f, 4.9517f)); 
  data[8].mAabb = Aabb(Vector3(-28.2997f, 18.1961f, 29.8171f), Vector3(-22.1759f, 25.6002f, 32.0645f)); 
  data[9].mAabb = Aabb(Vector3(-35.5978f, 25.7583f, -22.6446f), Vector3(-30.5506f, 32.5773f, -16.1986f)); 
  data[10].mAabb = Aabb(Vector3(-16.3748f, 17.5333f, -43.4083f), Vector3(-8.41097f, 24.3303f, -36.8732f)); 
  data[11].mAabb = Aabb(Vector3(19.4966f, 27.9936f, -15.6602f), Vector3(19.9325f, 31.8987f, -9.38588f)); 
  data[12].mAabb = Aabb(Vector3(14.1507f, -44.7853f, 7.14852f), Vector3(20.7045f, -37.075f, 13.6333f)); 
  data[13].mAabb = Aabb(Vector3(-19.6116f, 12.3796f, 20.3942f), Vector3(-15.2712f, 15.3757f, 25.4701f)); 
  data[14].mAabb = Aabb(Vector3(-5.41348f, -24.8003f, -34.2197f), Vector3(-0.548852f, -22.294f, -32.0282f)); 
  data[15].mAabb = Aabb(Vector3(42.8595f, -17.8466f, -7.19116f), Vector3(46.1381f, -10.6023f, -4.52533f)); 
  data[16].mAabb = Aabb(Vector3(11.8236f, -13.1266f, 10.1858f), Vector3(17.6243f, -7.94927f, 11.5428f)); 
  data[17].mAabb = Aabb(Vector3(7.51607f, -1.45019f, -3.96608f), Vector3(11.9284f, 2.0824f, -2.07493f)); 
  data[18].mAabb = Aabb(Vector3(-2.34779f, 5.22676f, -28.6839f), Vector3(0.152663f, 8.72665f, -22.6078f)); 
  data[19].mAabb = Aabb(Vector3(-18.1627f, -16.9596f, 8.72997f), Vector3(-15.3769f, -10.3522f, 9.41038f)); 
  data[20].mAabb = Aabb(Vector3(5.05004f, -30.252f, 20.7799f), Vector3(12.0555f, -28.9758f, 23.7447f)); 
  data[21].mAabb = Aabb(Vector3(-36.4796f, 25.0676f, -7.23992f), Vector3(-30.1893f, 31.0973f, -5.19987f)); 
  data[22].mAabb = Aabb(Vector3(-24.0589f, -2.37399f, -31.9599f), Vector3(-19.0957f, -1.10073f, -29.9678f)); 
  data[23].mAabb = Aabb(Vector3(-1.11936f, 14.1305f, -19.908f), Vector3(6.80364f, 16.9869f, -14.2483f)); 
  data[24].mAabb = Aabb(Vector3(31.1384f, 6.01545f, 24.1973f), Vector3(32.5661f, 9.68929f, 30.1713f)); 
  data[25].mAabb = Aabb(Vector3(2.49486f, 32.3786f, 7.40248f), Vector3(6.38696f, 39.9451f, 9.18972f)); 
  data[26].mAabb = Aabb(Vector3(32.9543f, -23.7924f, -20.6725f), Vector3(40.3712f, -16.3384f, -18.9772f)); 
  data[27].mAabb = Aabb(Vector3(26.3413f, -11.5333f, -36.713f), Vector3(34.2851f, -7.79524f, -29.6726f)); 
  data[28].mAabb = Aabb(Vector3(39.9927f, 11.2323f, -17.0154f), Vector3(43.0842f, 17.1265f, -10.3906f)); 
  data[29].mAabb = Aabb(Vector3(46.8176f, -3.6052f, 1.2944f), Vector3(47.6769f, 3.79546f, 4.50436f)); 
  data[30].mAabb = Aabb(Vector3(-11.2942f, -34.291f, 14.6529f), Vector3(-10.476f, -32.6494f, 18.4288f)); 
  data[31].mAabb = Aabb(Vector3(24.3665f, 16.8896f, 18.1005f), Vector3(25.4649f, 23.0494f, 19.2267f)); 
  data[32].mAabb = Aabb(Vector3(2.17999f, 34.5807f, -19.0137f), Vector3(2.95898f, 36.715f, -15.1237f)); 
  data[33].mAabb = Aabb(Vector3(17.8087f, -37.9039f, -20.2607f), Vector3(23.3487f, -31.0826f, -13.2177f)); 
  data[34].mAabb = Aabb(Vector3(3.83746f, 20.601f, -29.0066f), Vector3(11.8252f, 26.1981f, -22.3155f)); 
  data[35].mAabb = Aabb(Vector3(12.6624f, -22.1367f, -33.2845f), Vector3(14.1532f, -16.1198f, -26.0773f)); 
  data[36].mAabb = Aabb(Vector3(-3.1368f, -11.7537f, 35.2982f), Vector3(-0.958975f, -4.9257f, 36.8811f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(66.9477f, 71.0588f, 11.0056f), Vector3(-33.9126f, 43.042f, 70.3163f), Vector3(-2.02836f, -71.7409f, 70.3163f), Vector3(98.8319f, -43.724f, 11.0056f), Vector3(5.35575f, -4.39281f, -15.3427f), Vector3(-1.25498f, -6.22913f, -11.4552f), Vector3(0.538383f, -12.6852f, -11.4552f), Vector3(7.14911f, -10.8489f, -15.3427f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(-22.3929f, 0.618175f, 42.7983f), Vector3(-14.4163f, 1.90558f, 47.9682f)); 
  data[1].mAabb = Aabb(Vector3(20.8914f, -0.261986f, -3.37101f), Vector3(22.4654f, 2.71302f, -0.411077f)); 
  data[2].mAabb = Aabb(Vector3(5.55954f, -4.98241f, -40.3613f), Vector3(11.9798f, -1.87056f, -33.8696f)); 
  data[3].mAabb = Aabb(Vector3(12.1773f, -12.8306f, 33.9645f), Vector3(18.9877f, -7.5346f, 39.8063f)); 
  data[4].mAabb = Aabb(Vector3(18.898f, 1.15139f, 17.3155f), Vector3(19.3187f, 3.42477f, 25.2846f)); 
  data[5].mAabb = Aabb(Vector3(-20.8861f, 7.0284f, 12.8438f), Vector3(-14.4208f, 9.53048f, 14.3728f)); 
  data[6].mAabb = Aabb(Vector3(26.037f, 7.85651f, 22.1122f), Vector3(32.2302f, 12.2331f, 23.1494f)); 
  data[7].mAabb = Aabb(Vector3(18.4012f, 8.56719f, 24.5008f), Vector3(21.9612f, 14.8903f, 27.1589f)); 
  data[8].mAabb = Aabb(Vector3(-48.1832f, 10.0921f, -1.93442f), Vector3(-43.9439f, 12.8539f, 5.39921f)); 
  data[9].mAabb = Aabb(Vector3(-15.1945f, 29.9957f, -23.7988f), Vector3(-14.5862f, 35.1278f, -16.8347f)); 
  data[10].mAabb = Aabb(Vector3(11.6716f, 18.2141f, 29.4388f), Vector3(16.9669f, 22.1307f, 30.953f)); 
  data[11].mAabb = Aabb(Vector3(23.3529f, -36.51f, -1.79534f), Vector3(29.6068f, -28.8259f, 0.129204f)); 
  data[12].mAabb = Aabb(Vector3(9.89907f, 18.1038f, -29.1588f), Vector3(14.3868f, 18.7553f, -23.1834f)); 
  data[13].mAabb = Aabb(Vector3(-22.4209f, 37.4348f, 4.49965f), Vector3(-17.3693f, 44.8572f, 6.98549f)); 
  data[14].mAabb = Aabb(Vector3(-6.57506f, -42.4785f, -26.8341f), Vector3(-5.45882f, -35.6739f, -25.8445f)); 
  data[15].mAabb = Aabb(Vector3(-33.043f, -22.5704f, -28.4115f), Vector3(-31.7394f, -16.7514f, -25.5198f)); 
  data[16].mAabb = Aabb(Vector3(-31.8444f, -19.2937f, 13.2835f), Vector3(-24.9712f, -14.26f, 17.6615f)); 
  data[17].mAabb = Aabb(Vector3(10.0697f, -10.0922f, -41.0148f), Vector3(10.935f, -8.68005f, -39.9927f)); 
  data[18].mAabb = Aabb(Vector3(-31.8246f, -23.0409f, -32.0875f), Vector3(-24.7143f, -17.9758f, -29.8677f)); 
  data[19].mAabb = Aabb(Vector3(18.427f, 6.6086f, -24.0973f), Vector3(22.2544f, 13.611f, -17.4409f)); 
  data[20].mAabb = Aabb(Vector3(-38.1581f, -11.2228f, 24.9559f), Vector3(-34.9887f, -8.07203f, 26.0777f)); 
  data[21].mAabb = Aabb(Vector3(11.0687f, 5.9136f, 30.9138f), Vector3(12.264f, 7.58092f, 32.9863f)); 
  data[22].mAabb = Aabb(Vector3(15.2376f, 34.6229f, -7.73331f), Vector3(16.824f, 36.7328f, -2.58038f)); 
  data[23].mAabb = Aabb(Vector3(44.9997f, -8.39074f, -10.2396f), Vector3(47.8729f, -6.72666f, -8.24128f)); 
  data[24].mAabb = Aabb(Vector3(-41.8031f, 10.8577f, -5.93136f), Vector3(-39.622f, 15.162f, -0.370216f)); 
  data[25].mAabb = Aabb(Vector3(-33.2031f, -23.7361f, 10.9812f), Vector3(-26.7397f, -18.8615f, 17.5021f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-18.4458f, 25.1694f, 54.4369f), Vector3(14.4977f, 57.6067f, -1.37034f), Vector3(14.4977f, -5.49804f, -38.0492f), Vector3(-18.4458f, -37.9353f, 17.758f), Vector3(34.9773f, 1.84947f, 30.8366f), Vector3(38.8083f, 5.62165f, 24.3467f), Vector3(38.8083f, -2.11419f, 19.8503f), Vector3(34.9773f, -5.88637f, 26.3402f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mAabb = Aabb(Vector3(22.1418f, -13.7991f, -6.54155f), Vector3(24.8269f, -9.14248f, -5.42995f)); 
  data[1].mAabb = Aabb(Vector3(-15.0663f, -20.6222f, -20.1097f), Vector3(-12.4141f, -18.3042f, -16.5219f)); 
  data[2].mAabb = Aabb(Vector3(27.7255f, -26.3701f, -22.6863f), Vector3(34.7773f, -21.6904f, -19.1096f)); 
  data[3].mAabb = Aabb(Vector3(-7.94337f, -12.7415f, 41.774f), Vector3(-2.2934f, -5.34643f, 44.6651f)); 
  data[4].mAabb = Aabb(Vector3(24.1591f, -42.8803f, -6.81119f), Vector3(26.0618f, -37.1266f, 0.904214f)); 
  data[5].mAabb = Aabb(Vector3(-2.47006f, -33.3439f, -14.7529f), Vector3(1.34619f, -25.4058f, -13.8297f)); 
  data[6].mAabb = Aabb(Vector3(2.08338f, -44.9511f, -20.3586f), Vector3(3.8669f, -37.1608f, -19.4117f)); 
  data[7].mAabb = Aabb(Vector3(-37.6833f, 4.67683f, -20.9696f), Vector3(-30.0776f, 8.84262f, -16.0948f)); 
  data[8].mAabb = Aabb(Vector3(-13.4882f, 1.28237f, -9.34355f), Vector3(-12.6874f, 3.53418f, -1.60425f)); 
  data[9].mAabb = Aabb(Vector3(26.2465f, 24.7524f, 14.558f), Vector3(27.2108f, 32.2349f, 17.9632f)); 
  data[10].mAabb = Aabb(Vector3(-37.7633f, -1.7629f, 23.7792f), Vector3(-30.073f, 1.03444f, 24.2938f)); 
  data[11].mAabb = Aabb(Vector3(23.2775f, -22.4001f, -35.7101f), Vector3(25.0666f, -19.8484f, -28.4449f)); 
  data[12].mAabb = Aabb(Vector3(-39.8473f, 14.5823f, -5.5235f), Vector3(-36.492f, 19.1002f, -3.52589f)); 
  data[13].mAabb = Aabb(Vector3(-0.389983f, -13.4355f, 27.6807f), Vector3(3.02037f, -5.5422f, 34.8406f)); 
  data[14].mAabb = Aabb(Vector3(9.64457f, 43.9176f, -18.3693f), Vector3(14.5836f, 47.4871f, -12.0054f)); 
  data[15].mAabb = Aabb(Vector3(-45.7399f, 8.82101f, 16.0256f), Vector3(-39.8714f, 12.7289f, 18.5486f)); 
  data[16].mAabb = Aabb(Vector3(-45.4614f, -12.2944f, 19.5345f), Vector3(-39.1691f, -4.67521f, 23.4875f)); 
  data[17].mAabb = Aabb(Vector3(15.2486f, 9.91641f, 29.4336f), Vector3(22.7887f, 12.5621f, 37.2956f)); 
  data[18].mAabb = Aabb(Vector3(19.1793f, -6.6386f, -14.616f), Vector3(20.88f, -4.92466f, -11.7043f)); 
  data[19].mAabb = Aabb(Vector3(7.64263f, -4.0757f, 8.60945f), Vector3(13.923f, 1.35486f, 16.5454f)); 
  data[20].mAabb = Aabb(Vector3(-5.83576f, -43.3968f, 8.73768f), Vector3(1.29701f, -38.2091f, 14.4607f)); 
  data[21].mAabb = Aabb(Vector3(-8.2409f, -17.0473f, -11.8414f), Vector3(-5.99743f, -15.9963f, -7.81111f)); 
  data[22].mAabb = Aabb(Vector3(18.6373f, 11.8413f, -8.82335f), Vector3(23.3078f, 13.6572f, -7.63244f)); 
  data[23].mAabb = Aabb(Vector3(22.9546f, 7.48803f, 0.0821986f), Vector3(28.8945f, 8.92179f, 1.07156f)); 
  data[24].mAabb = Aabb(Vector3(7.8053f, -23.1337f, 15.8004f), Vector3(13.3936f, -22.6994f, 21.6389f)); 
  data[25].mAabb = Aabb(Vector3(3.22398f, -33.1242f, 6.42967f), Vector3(8.92336f, -31.6167f, 7.70131f)); 
  data[26].mAabb = Aabb(Vector3(17.7316f, 24.7447f, -24.4009f), Vector3(23.4692f, 25.3732f, -21.9178f)); 
  data[27].mAabb = Aabb(Vector3(-4.53034f, 6.48768f, -21.0514f), Vector3(-0.0335698f, 11.0628f, -13.3647f)); 
  data[28].mAabb = Aabb(Vector3(-35.1933f, -9.60299f, -2.25003f), Vector3(-29.2875f, -2.88956f, 5.30209f)); 
  data[29].mAabb = Aabb(Vector3(18.3448f, 37.4362f, -11.7208f), Vector3(22.6511f, 44.9377f, -9.44296f)); 
  data[30].mAabb = Aabb(Vector3(-32.1936f, 23.8146f, -32.7264f), Vector3(-24.5877f, 25.2244f, -31.8257f)); 
  data[31].mAabb = Aabb(Vector3(15.2282f, 28.5141f, 19.8846f), Vector3(17.5726f, 31.9474f, 21.7025f)); 
  data[32].mAabb = Aabb(Vector3(-51.1439f, -5.47544f, 3.38088f), Vector3(-46.1803f, 1.75986f, 6.27586f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(54.1615f, 64.6982f, -27.2865f), Vector3(-49.6161f, 58.8415f, 74.2165f), Vector3(-41.4795f, -85.3347f, 74.2165f), Vector3(62.2981f, -79.478f, -27.2865f), Vector3(-26.644f, -8.68002f, -15.7933f), Vector3(-32.2621f, -8.99708f, -10.2983f), Vector3(-31.8684f, -15.9743f, -10.2983f), Vector3(-26.2502f, -15.6573f, -15.7933f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(73); 
  data[0].mAabb = Aabb(Vector3(-9.37304f, 15.3583f, 23.4269f), Vector3(-1.77174f, 20.9348f, 24.2606f)); 
  data[1].mAabb = Aabb(Vector3(-24.5775f, 22.2306f, 1.60303f), Vector3(-22.4936f, 26.3468f, 4.73529f)); 
  data[2].mAabb = Aabb(Vector3(-10.713f, -8.63057f, 30.7476f), Vector3(-9.78142f, -1.91343f, 37.8999f)); 
  data[3].mAabb = Aabb(Vector3(36.3434f, -0.0581332f, 5.78088f), Vector3(39.3465f, 3.34759f, 12.3115f)); 
  data[4].mAabb = Aabb(Vector3(27.274f, 24.5159f, -17.6791f), Vector3(33.3257f, 26.0118f, -13.9526f)); 
  data[5].mAabb = Aabb(Vector3(10.0983f, 42.7943f, 5.29913f), Vector3(11.2848f, 49.584f, 10.7703f)); 
  data[6].mAabb = Aabb(Vector3(-5.70689f, 20.5624f, 24.9275f), Vector3(-1.12431f, 23.0784f, 25.4669f)); 
  data[7].mAabb = Aabb(Vector3(4.97273f, 7.39619f, 28.4806f), Vector3(8.41626f, 11.6974f, 30.7502f)); 
  data[8].mAabb = Aabb(Vector3(19.7562f, -44.9439f, 5.81877f), Vector3(22.0382f, -41.4719f, 8.42963f)); 
  data[9].mAabb = Aabb(Vector3(28.1873f, 23.0392f, -18.3297f), Vector3(30.5607f, 24.8659f, -16.7115f)); 
  data[10].mAabb = Aabb(Vector3(18.7755f, -26.7239f, -8.28349f), Vector3(24.4935f, -20.3521f, -4.90444f)); 
  data[11].mAabb = Aabb(Vector3(1.15271f, 3.70918f, -30.041f), Vector3(7.69705f, 5.52425f, -23.3348f)); 
  data[12].mAabb = Aabb(Vector3(11.394f, -39.4672f, 5.72958f), Vector3(12.0761f, -33.9186f, 7.04343f)); 
  data[13].mAabb = Aabb(Vector3(9.36531f, 11.9109f, 31.9344f), Vector3(16.1084f, 12.894f, 34.2873f)); 
  data[14].mAabb = Aabb(Vector3(-19.9953f, 40.1772f, -14.3941f), Vector3(-16.6429f, 44.4042f, -11.9203f)); 
  data[15].mAabb = Aabb(Vector3(-0.262514f, -4.49037f, -33.6938f), Vector3(4.14078f, 2.18943f, -29.9281f)); 
  data[16].mAabb = Aabb(Vector3(-18.048f, 24.4165f, -4.04751f), Vector3(-17.433f, 30.8178f, 1.86851f)); 
  data[17].mAabb = Aabb(Vector3(7.24633f, 22.8894f, 11.6434f), Vector3(11.372f, 24.261f, 14.1754f)); 
  data[18].mAabb = Aabb(Vector3(10.0316f, -2.15902f, 38.7813f), Vector3(12.5944f, 4.43356f, 41.8567f)); 
  data[19].mAabb = Aabb(Vector3(17.9098f, -38.7639f, -24.4587f), Vector3(23.1165f, -37.4514f, -21.9944f)); 
  data[20].mAabb = Aabb(Vector3(-16.5946f, -0.021976f, -5.46437f), Vector3(-15.8147f, 3.82397f, -2.23748f)); 
  data[21].mAabb = Aabb(Vector3(-37.4537f, -21.2053f, -25.321f), Vector3(-36.9712f, -19.1476f, -18.0173f)); 
  data[22].mAabb = Aabb(Vector3(-9.86947f, -21.349f, 29.8313f), Vector3(-7.64433f, -19.62f, 31.966f)); 
  data[23].mAabb = Aabb(Vector3(-39.5738f, -13.7251f, 4.19386f), Vector3(-37.3686f, -11.2367f, 5.09207f)); 
  data[24].mAabb = Aabb(Vector3(28.9647f, -17.2899f, 12.9076f), Vector3(29.6917f, -11.7935f, 17.7214f)); 
  data[25].mAabb = Aabb(Vector3(-30.5555f, -0.0175116f, 16.7804f), Vector3(-26.0953f, 6.38813f, 19.0512f)); 
  data[26].mAabb = Aabb(Vector3(-18.2643f, 23.2145f, 34.6086f), Vector3(-15.56f, 24.1946f, 38.9124f)); 
  data[27].mAabb = Aabb(Vector3(-1.64267f, 6.75861f, -48.328f), Vector3(-1.00238f, 10.1179f, -47.7459f)); 
  data[28].mAabb = Aabb(Vector3(14.2509f, 14.0219f, -21.1087f), Vector3(21.3723f, 16.3939f, -16.5319f)); 
  data[29].mAabb = Aabb(Vector3(-7.12365f, -45.0524f, 20.2737f), Vector3(-3.63443f, -42.4798f, 24.6812f)); 
  data[30].mAabb = Aabb(Vector3(-21.7336f, 4.47323f, -31.2117f), Vector3(-18.6398f, 5.92856f, -26.5248f)); 
  data[31].mAabb = Aabb(Vector3(32.3601f, -5.55202f, 12.3642f), Vector3(36.1814f, 2.0727f, 15.1024f)); 
  data[32].mAabb = Aabb(Vector3(-25.7826f, 17.0426f, -4.66459f), Vector3(-19.026f, 21.2976f, -2.78945f)); 
  data[33].mAabb = Aabb(Vector3(-32.5251f, -36.3017f, -12.0394f), Vector3(-27.0862f, -30.5467f, -8.32007f)); 
  data[34].mAabb = Aabb(Vector3(5.54337f, 8.59007f, 44.1165f), Vector3(8.7473f, 11.4524f, 50.8597f)); 
  data[35].mAabb = Aabb(Vector3(22.9535f, -23.6442f, 24.5099f), Vector3(26.905f, -19.7929f, 26.2454f)); 
  data[36].mAabb = Aabb(Vector3(-15.2989f, 22.4437f, -22.5222f), Vector3(-10.1796f, 30.0439f, -15.9166f)); 
  data[37].mAabb = Aabb(Vector3(19.534f, -8.4142f, 9.70218f), Vector3(20.9496f, -5.83117f, 10.5628f)); 
  data[38].mAabb = Aabb(Vector3(-25.4051f, -19.4896f, 33.9928f), Vector3(-22.7038f, -16.9991f, 40.9303f)); 
  data[39].mAabb = Aabb(Vector3(24.9161f, -29.6859f, 0.37689f), Vector3(25.42f, -28.1417f, 6.45642f)); 
  data[40].mAabb = Aabb(Vector3(24.1392f, 29.5943f, -18.8433f), Vector3(25.4426f, 31.9836f, -15.4171f)); 
  data[41].mAabb = Aabb(Vector3(13.7877f, -5.85643f, -48.4711f), Vector3(15.5046f, -2.98974f, -44.5736f)); 
  data[42].mAabb = Aabb(Vector3(-24.1858f, -8.67068f, -23.4594f), Vector3(-19.1882f, -2.32697f, -16.518f)); 
  data[43].mAabb = Aabb(Vector3(10.6518f, 37.1606f, -1.42845f), Vector3(11.2023f, 44.5278f, 1.92137f)); 
  data[44].mAabb = Aabb(Vector3(-4.85526f, 6.95145f, 8.36678f), Vector3(0.818374f, 9.69475f, 12.7024f)); 
  data[45].mAabb = Aabb(Vector3(-7.28962f, -18.4382f, -13.2263f), Vector3(-6.75788f, -17.3475f, -6.20489f)); 
  data[46].mAabb = Aabb(Vector3(0.0737529f, -3.85685f, 12.4878f), Vector3(3.16264f, -0.634597f, 20.2746f)); 
  data[47].mAabb = Aabb(Vector3(-20.5015f, 24.9752f, 17.9607f), Vector3(-19.466f, 27.2873f, 23.8517f)); 
  data[48].mAabb = Aabb(Vector3(20.4771f, 13.5202f, 14.1322f), Vector3(21.9565f, 15.9963f, 21.3301f)); 
  data[49].mAabb = Aabb(Vector3(-7.17025f, 17.4464f, 32.3549f), Vector3(-0.977533f, 24.706f, 36.7682f)); 
  data[50].mAabb = Aabb(Vector3(-9.82194f, 16.268f, 12.454f), Vector3(-9.27698f, 16.8245f, 13.6182f)); 
  data[51].mAabb = Aabb(Vector3(8.2423f, -45.804f, 9.87195f), Vector3(14.4468f, -42.3839f, 12.3216f)); 
  data[52].mAabb = Aabb(Vector3(-7.31718f, -9.00978f, -13.8262f), Vector3(-4.89675f, -1.40455f, -9.2063f)); 
  data[53].mAabb = Aabb(Vector3(-15.9978f, 7.59893f, -1.75042f), Vector3(-8.22142f, 10.2921f, -0.491544f)); 
  data[54].mAabb = Aabb(Vector3(-31.745f, -18.2172f, -22.249f), Vector3(-24.897f, -10.2269f, -18.7709f)); 
  data[55].mAabb = Aabb(Vector3(18.0333f, 10.1264f, 19.8562f), Vector3(22.2126f, 14.9778f, 25.6217f)); 
  data[56].mAabb = Aabb(Vector3(-29.4653f, -4.04784f, 1.80855f), Vector3(-28.5068f, -2.64029f, 6.98607f)); 
  data[57].mAabb = Aabb(Vector3(23.6538f, -32.9412f, -0.304403f), Vector3(30.9107f, -29.7686f, 6.4067f)); 
  data[58].mAabb = Aabb(Vector3(3.21204f, -6.3774f, 21.5803f), Vector3(10.2929f, 0.333473f, 25.0256f)); 
  data[59].mAabb = Aabb(Vector3(13.7753f, 3.58824f, 3.9859f), Vector3(15.2608f, 7.09439f, 10.0947f)); 
  data[60].mAabb = Aabb(Vector3(5.90326f, 23.1197f, 4.5795f), Vector3(10.8864f, 24.7483f, 10.7718f)); 
  data[61].mAabb = Aabb(Vector3(-19.513f, 23.2986f, 13.77f), Vector3(-16.3077f, 24.7328f, 14.9748f)); 
  data[62].mAabb = Aabb(Vector3(-10.3559f, -16.3657f, 21.5122f), Vector3(-5.58773f, -13.0298f, 27.1605f)); 
  data[63].mAabb = Aabb(Vector3(42.1566f, 1.35986f, -3.18808f), Vector3(43.6852f, 4.15372f, 2.38211f)); 
  data[64].mAabb = Aabb(Vector3(7.83097f, -8.29664f, 24.1264f), Vector3(11.0565f, -3.65654f, 25.7838f)); 
  data[65].mAabb = Aabb(Vector3(0.657922f, 18.2407f, 18.9938f), Vector3(2.70122f, 20.0571f, 26.3156f)); 
  data[66].mAabb = Aabb(Vector3(7.89803f, 22.4666f, -39.2418f), Vector3(8.96115f, 23.6098f, -36.4055f)); 
  data[67].mAabb = Aabb(Vector3(15.7993f, 37.8054f, 21.5054f), Vector3(18.8203f, 39.9602f, 25.4051f)); 
  data[68].mAabb = Aabb(Vector3(15.3715f, 11.8358f, 5.57735f), Vector3(19.1933f, 18.3825f, 8.09334f)); 
  data[69].mAabb = Aabb(Vector3(-28.6413f, -10.3816f, -20.8734f), Vector3(-24.9983f, -7.72784f, -15.071f)); 
  data[70].mAabb = Aabb(Vector3(-36.719f, 16.721f, -28.9001f), Vector3(-35.23f, 19.3754f, -22.7236f)); 
  data[71].mAabb = Aabb(Vector3(-1.54392f, 6.80344f, -22.2152f), Vector3(1.61362f, 9.53329f, -17.9191f)); 
  data[72].mAabb = Aabb(Vector3(12.9765f, -13.0565f, -16.2987f), Vector3(14.6041f, -10.6115f, -15.4132f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(132.876f, 34.8941f, -1.99508f), Vector3(-37.5285f, 119.65f, 36.0986f), Vector3(-123.491f, -53.1804f, 36.0986f), Vector3(46.9132f, -137.936f, -1.99508f), Vector3(-0.410046f, -1.6835f, -47.5372f), Vector3(-9.08327f, 2.6304f, -45.5983f), Vector3(-13.0083f, -5.26099f, -45.5983f), Vector3(-4.33508f, -9.5749f, -47.5372f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(57); 
  data[0].mAabb = Aabb(Vector3(25.3937f, -5.48405f, 25.6587f), Vector3(29.4097f, 0.0596955f, 32.0391f)); 
  data[1].mAabb = Aabb(Vector3(-14.0165f, 18.4667f, -5.60935f), Vector3(-6.09303f, 21.9495f, -3.77781f)); 
  data[2].mAabb = Aabb(Vector3(25.8577f, -42.7517f, -10.1722f), Vector3(26.9848f, -38.0202f, -3.3345f)); 
  data[3].mAabb = Aabb(Vector3(23.4301f, 21.4715f, -30.4255f), Vector3(30.6166f, 24.6209f, -23.1833f)); 
  data[4].mAabb = Aabb(Vector3(-5.27449f, 23.5608f, 23.1284f), Vector3(-1.06209f, 26.0316f, 23.5868f)); 
  data[5].mAabb = Aabb(Vector3(-11.7592f, 26.4516f, -19.7792f), Vector3(-7.17796f, 31.4754f, -12.8197f)); 
  data[6].mAabb = Aabb(Vector3(-27.0376f, 2.37856f, 15.9214f), Vector3(-21.7762f, 9.31325f, 22.4158f)); 
  data[7].mAabb = Aabb(Vector3(40.9967f, -4.46757f, 3.72352f), Vector3(42.5207f, 0.356469f, 6.25065f)); 
  data[8].mAabb = Aabb(Vector3(-9.94995f, 6.21166f, 13.7336f), Vector3(-7.73131f, 13.7545f, 18.0625f)); 
  data[9].mAabb = Aabb(Vector3(-36.5889f, 8.81487f, -34.94f), Vector3(-29.5737f, 11.464f, -27.5934f)); 
  data[10].mAabb = Aabb(Vector3(-20.4217f, 13.5273f, 32.0187f), Vector3(-19.6293f, 20.1086f, 33.0862f)); 
  data[11].mAabb = Aabb(Vector3(-13.7087f, -11.0714f, -25.8666f), Vector3(-7.47123f, -6.03441f, -20.5181f)); 
  data[12].mAabb = Aabb(Vector3(12.7619f, -24.0255f, 10.1075f), Vector3(20.018f, -22.1723f, 12.9445f)); 
  data[13].mAabb = Aabb(Vector3(-17.8205f, -20.2396f, 38.404f), Vector3(-14.672f, -17.606f, 44.038f)); 
  data[14].mAabb = Aabb(Vector3(-36.9549f, 2.77188f, 10.4643f), Vector3(-32.557f, 7.44423f, 12.1188f)); 
  data[15].mAabb = Aabb(Vector3(0.736681f, 32.4112f, -18.986f), Vector3(2.66958f, 34.5011f, -16.1466f)); 
  data[16].mAabb = Aabb(Vector3(27.4629f, -42.2635f, -2.96507f), Vector3(32.8538f, -35.7518f, -0.633699f)); 
  data[17].mAabb = Aabb(Vector3(-8.62778f, -25.1135f, -36.8701f), Vector3(-7.45264f, -20.5221f, -30.4768f)); 
  data[18].mAabb = Aabb(Vector3(31.4973f, 22.7398f, 5.8097f), Vector3(34.432f, 24.1596f, 9.48145f)); 
  data[19].mAabb = Aabb(Vector3(-4.20989f, 43.8042f, -12.2836f), Vector3(-1.7941f, 45.8426f, -10.6592f)); 
  data[20].mAabb = Aabb(Vector3(-24.8047f, -34.0163f, 28.3561f), Vector3(-17.5631f, -26.0961f, 32.3883f)); 
  data[21].mAabb = Aabb(Vector3(44.1617f, 16.2779f, -14.6568f), Vector3(44.7157f, 21.1681f, -8.77189f)); 
  data[22].mAabb = Aabb(Vector3(-34.915f, -9.63032f, 4.68565f), Vector3(-29.493f, -2.13201f, 10.9846f)); 
  data[23].mAabb = Aabb(Vector3(27.2635f, 3.73964f, -14.6625f), Vector3(30.6235f, 9.21891f, -8.0112f)); 
  data[24].mAabb = Aabb(Vector3(15.6902f, -30.3772f, 0.717701f), Vector3(21.1272f, -28.5775f, 1.86988f)); 
  data[25].mAabb = Aabb(Vector3(35.5359f, -3.55321f, -20.8636f), Vector3(37.4913f, 1.26294f, -15.2988f)); 
  data[26].mAabb = Aabb(Vector3(14.922f, -8.16075f, -47.3629f), Vector3(19.6975f, -1.31651f, -44.8882f)); 
  data[27].mAabb = Aabb(Vector3(-26.9668f, -0.785498f, 28.3741f), Vector3(-19.0366f, 5.9052f, 29.3479f)); 
  data[28].mAabb = Aabb(Vector3(17.0874f, 29.1807f, 14.4866f), Vector3(18.5114f, 33.1481f, 21.8548f)); 
  data[29].mAabb = Aabb(Vector3(-36.1096f, 17.6005f, 20.8345f), Vector3(-33.1552f, 18.2872f, 24.5777f)); 
  data[30].mAabb = Aabb(Vector3(19.9667f, 16.845f, 7.42889f), Vector3(21.2548f, 24.656f, 15.2719f)); 
  data[31].mAabb = Aabb(Vector3(-3.77211f, -1.52588f, -8.14147f), Vector3(3.97763f, 3.59527f, -5.65516f)); 
  data[32].mAabb = Aabb(Vector3(26.7736f, -3.42064f, -41.5208f), Vector3(29.5997f, 1.64183f, -38.8817f)); 
  data[33].mAabb = Aabb(Vector3(-22.1501f, 7.38846f, -33.2151f), Vector3(-19.8716f, 8.90502f, -29.8226f)); 
  data[34].mAabb = Aabb(Vector3(-17.4734f, -13.286f, -19.9595f), Vector3(-13.2088f, -5.41819f, -12.913f)); 
  data[35].mAabb = Aabb(Vector3(-18.2943f, 4.7473f, -24.24f), Vector3(-15.4717f, 11.1759f, -16.6366f)); 
  data[36].mAabb = Aabb(Vector3(-18.5308f, 16.142f, -38.9554f), Vector3(-11.9811f, 22.164f, -37.3769f)); 
  data[37].mAabb = Aabb(Vector3(-13.9936f, 5.33242f, 13.8895f), Vector3(-8.99191f, 11.1617f, 19.594f)); 
  data[38].mAabb = Aabb(Vector3(-27.0887f, -17.79f, -23.5468f), Vector3(-22.8855f, -16.7376f, -16.1731f)); 
  data[39].mAabb = Aabb(Vector3(12.4674f, 15.213f, 15.4539f), Vector3(19.6718f, 20.6823f, 19.7494f)); 
  data[40].mAabb = Aabb(Vector3(-22.4125f, -36.4101f, -8.03209f), Vector3(-16.7697f, -31.3005f, -6.85857f)); 
  data[41].mAabb = Aabb(Vector3(-7.50563f, -9.40453f, -15.1865f), Vector3(-1.61653f, -3.38136f, -10.5879f)); 
  data[42].mAabb = Aabb(Vector3(-33.847f, -30.7335f, -24.0398f), Vector3(-27.1181f, -25.3607f, -21.8866f)); 
  data[43].mAabb = Aabb(Vector3(-14.2617f, 10.2086f, -11.2532f), Vector3(-11.8422f, 14.6456f, -9.98246f)); 
  data[44].mAabb = Aabb(Vector3(41.0005f, 8.16295f, 5.97172f), Vector3(45.7035f, 13.0531f, 11.4273f)); 
  data[45].mAabb = Aabb(Vector3(11.2663f, -13.5783f, -14.5376f), Vector3(15.3605f, -8.49352f, -9.55794f)); 
  data[46].mAabb = Aabb(Vector3(-17.3783f, 1.04019f, 17.41f), Vector3(-10.7017f, 4.79777f, 20.9094f)); 
  data[47].mAabb = Aabb(Vector3(12.8028f, -29.4446f, -22.49f), Vector3(18.6385f, -24.0927f, -17.5825f)); 
  data[48].mAabb = Aabb(Vector3(-1.50218f, 17.8279f, 3.51322f), Vector3(4.33311f, 18.985f, 9.03377f)); 
  data[49].mAabb = Aabb(Vector3(14.009f, 16.1122f, 15.6465f), Vector3(17.6112f, 23.7652f, 19.5915f)); 
  data[50].mAabb = Aabb(Vector3(35.214f, 14.1146f, 26.1362f), Vector3(36.3365f, 17.8738f, 33.744f)); 
  data[51].mAabb = Aabb(Vector3(14.9592f, 8.26166f, 22.3794f), Vector3(18.1395f, 15.5617f, 27.1167f)); 
  data[52].mAabb = Aabb(Vector3(-2.59166f, 21.8873f, 35.1743f), Vector3(0.567511f, 28.8967f, 41.2737f)); 
  data[53].mAabb = Aabb(Vector3(-24.9643f, 21.4174f, 8.65928f), Vector3(-17.9377f, 22.0498f, 12.0031f)); 
  data[54].mAabb = Aabb(Vector3(18.3027f, -34.1885f, -3.86566f), Vector3(21.672f, -30.9843f, 2.73388f)); 
  data[55].mAabb = Aabb(Vector3(-24.7953f, -21.7504f, 19.1417f), Vector3(-21.1816f, -20.9291f, 24.6196f)); 
  data[56].mAabb = Aabb(Vector3(-21.1036f, -4.02689f, 25.7596f), Vector3(-20.0361f, 1.64976f, 33.238f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(66.548f, 25.4498f, -14.3898f), Vector3(14.7928f, 58.4755f, 24.6521f), Vector3(-25.0294f, -3.93072f, 24.6521f), Vector3(26.7258f, -36.9564f, -14.3898f), Vector3(8.4418f, 24.3046f, -30.4008f), Vector3(2.52866f, 28.0779f, -25.9401f), Vector3(-2.62739f, 19.9977f, -25.9401f), Vector3(3.28576f, 16.2245f, -30.4008f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(68); 
  data[0].mAabb = Aabb(Vector3(26.8409f, -34.0355f, -18.3982f), Vector3(33.7805f, -31.3889f, -10.8085f)); 
  data[1].mAabb = Aabb(Vector3(-16.1451f, -25.7978f, -31.4644f), Vector3(-12.4156f, -21.629f, -30.3982f)); 
  data[2].mAabb = Aabb(Vector3(23.3123f, -21.9481f, -8.49811f), Vector3(27.5171f, -18.8228f, -4.59488f)); 
  data[3].mAabb = Aabb(Vector3(-1.78459f, -30.2238f, -40.0428f), Vector3(0.155032f, -28.4704f, -38.9104f)); 
  data[4].mAabb = Aabb(Vector3(-3.09877f, -0.794026f, 25.0867f), Vector3(3.92314f, 0.946127f, 28.5569f)); 
  data[5].mAabb = Aabb(Vector3(20.5913f, 22.3751f, 15.7168f), Vector3(22.9241f, 25.0785f, 17.2144f)); 
  data[6].mAabb = Aabb(Vector3(-2.43479f, -38.2607f, -35.0499f), Vector3(1.13074f, -35.0535f, -30.4286f)); 
  data[7].mAabb = Aabb(Vector3(-3.95546f, -15.186f, -18.2694f), Vector3(2.8396f, -10.237f, -16.8632f)); 
  data[8].mAabb = Aabb(Vector3(5.5244f, -2.04626f, -30.732f), Vector3(6.97741f, -0.571682f, -29.0176f)); 
  data[9].mAabb = Aabb(Vector3(-3.32516f, 24.6654f, 5.39161f), Vector3(-0.83561f, 29.238f, 12.285f)); 
  data[10].mAabb = Aabb(Vector3(8.82594f, 31.341f, 7.73978f), Vector3(13.5166f, 33.6728f, 11.495f)); 
  data[11].mAabb = Aabb(Vector3(-12.3872f, -8.92109f, -29.4872f), Vector3(-5.26416f, -5.45832f, -26.418f)); 
  data[12].mAabb = Aabb(Vector3(-16.6697f, -14.1691f, -32.8697f), Vector3(-9.64613f, -9.75681f, -29.8311f)); 
  data[13].mAabb = Aabb(Vector3(32.6183f, -7.949f, 10.3953f), Vector3(35.3349f, -6.10401f, 12.5794f)); 
  data[14].mAabb = Aabb(Vector3(21.5847f, -9.29245f, 38.5038f), Vector3(26.8332f, -5.39037f, 40.3771f)); 
  data[15].mAabb = Aabb(Vector3(-34.0127f, 3.63514f, -38.3869f), Vector3(-27.4582f, 8.53131f, -34.5572f)); 
  data[16].mAabb = Aabb(Vector3(-14.2423f, 0.826976f, -36.383f), Vector3(-12.3862f, 1.69271f, -32.3279f)); 
  data[17].mAabb = Aabb(Vector3(9.64222f, 11.2509f, 17.4523f), Vector3(17.0004f, 11.71f, 21.2977f)); 
  data[18].mAabb = Aabb(Vector3(27.7623f, 4.4296f, 14.704f), Vector3(28.3247f, 8.53323f, 17.8483f)); 
  data[19].mAabb = Aabb(Vector3(11.509f, 0.475209f, -31.3187f), Vector3(19.2733f, 5.94613f, -29.0311f)); 
  data[20].mAabb = Aabb(Vector3(-6.77275f, 25.2628f, -40.5492f), Vector3(-4.68213f, 30.1135f, -37.1303f)); 
  data[21].mAabb = Aabb(Vector3(-15.2379f, -27.3357f, -40.0363f), Vector3(-7.68205f, -26.0246f, -37.4078f)); 
  data[22].mAabb = Aabb(Vector3(-0.93015f, 3.21394f, -1.6594f), Vector3(2.84041f, 10.6464f, 5.09832f)); 
  data[23].mAabb = Aabb(Vector3(23.3326f, 8.23725f, 35.2387f), Vector3(24.8487f, 15.9292f, 40.9808f)); 
  data[24].mAabb = Aabb(Vector3(0.99579f, 4.37164f, 21.9151f), Vector3(3.70986f, 7.95874f, 23.2371f)); 
  data[25].mAabb = Aabb(Vector3(-31.9166f, -18.9405f, -27.255f), Vector3(-30.7485f, -17.6476f, -21.2103f)); 
  data[26].mAabb = Aabb(Vector3(16.1676f, 24.3698f, -35.069f), Vector3(22.2986f, 25.74f, -28.1747f)); 
  data[27].mAabb = Aabb(Vector3(-22.2777f, -7.21106f, -41.5678f), Vector3(-20.0206f, -2.44964f, -40.6188f)); 
  data[28].mAabb = Aabb(Vector3(-1.06031f, -40.4282f, -17.0159f), Vector3(1.88571f, -37.4777f, -9.47699f)); 
  data[29].mAabb = Aabb(Vector3(36.6646f, 7.05762f, 16.1701f), Vector3(42.7077f, 13.7411f, 21.9673f)); 
  data[30].mAabb = Aabb(Vector3(34.284f, -17.2241f, -3.62972f), Vector3(40.9659f, -13.0124f, -2.3836f)); 
  data[31].mAabb = Aabb(Vector3(-2.9845f, 30.7507f, -16.4238f), Vector3(-1.91999f, 38.4951f, -14.5934f)); 
  data[32].mAabb = Aabb(Vector3(-16.5919f, 16.1882f, -16.0714f), Vector3(-14.4226f, 17.8618f, -11.2492f)); 
  data[33].mAabb = Aabb(Vector3(-18.0291f, -27.2007f, -16.8243f), Vector3(-12.1929f, -21.5146f, -9.85922f)); 
  data[34].mAabb = Aabb(Vector3(-25.1403f, -22.2842f, 22.0602f), Vector3(-20.2931f, -18.6429f, 22.5476f)); 
  data[35].mAabb = Aabb(Vector3(-12.0668f, 13.8715f, -47.9104f), Vector3(-8.92644f, 15.3514f, -40.9803f)); 
  data[36].mAabb = Aabb(Vector3(-39.5051f, 11.7429f, 28.6695f), Vector3(-35.0909f, 12.4419f, 33.1168f)); 
  data[37].mAabb = Aabb(Vector3(19.7255f, 11.3148f, 9.36844f), Vector3(22.9069f, 17.8334f, 14.3029f)); 
  data[38].mAabb = Aabb(Vector3(22.6616f, -4.12749f, -24.8614f), Vector3(24.2579f, 2.33172f, -24.1668f)); 
  data[39].mAabb = Aabb(Vector3(14.9487f, 17.7873f, 7.01151f), Vector3(20.405f, 21.8357f, 10.4339f)); 
  data[40].mAabb = Aabb(Vector3(10.0692f, 19.9963f, 37.9751f), Vector3(11.0605f, 22.0751f, 41.8839f)); 
  data[41].mAabb = Aabb(Vector3(-12.2224f, 16.0979f, 5.80726f), Vector3(-6.81018f, 20.3488f, 7.09002f)); 
  data[42].mAabb = Aabb(Vector3(-3.46268f, 22.9756f, -40.1086f), Vector3(-1.91133f, 24.8334f, -37.793f)); 
  data[43].mAabb = Aabb(Vector3(1.84525f, -18.333f, 40.704f), Vector3(7.5945f, -16.5497f, 43.1753f)); 
  data[44].mAabb = Aabb(Vector3(21.8895f, -28.1146f, -35.9487f), Vector3(29.1624f, -23.0869f, -29.5841f)); 
  data[45].mAabb = Aabb(Vector3(-27.9426f, -20.0542f, 15.4862f), Vector3(-24.3116f, -14.4578f, 20.8975f)); 
  data[46].mAabb = Aabb(Vector3(6.89323f, 31.1476f, 11.0604f), Vector3(7.7374f, 36.5965f, 18.3525f)); 
  data[47].mAabb = Aabb(Vector3(-34.9388f, -17.0989f, 19.3437f), Vector3(-30.0714f, -14.0626f, 26.4793f)); 
  data[48].mAabb = Aabb(Vector3(18.483f, 1.71445f, 8.19853f), Vector3(20.1102f, 8.43809f, 11.2297f)); 
  data[49].mAabb = Aabb(Vector3(24.374f, -15.7255f, 15.1431f), Vector3(30.3041f, -10.477f, 21.8702f)); 
  data[50].mAabb = Aabb(Vector3(42.4879f, 5.40243f, 15.4841f), Vector3(48.8126f, 7.04262f, 17.4122f)); 
  data[51].mAabb = Aabb(Vector3(-10.8618f, -15.4277f, 2.55785f), Vector3(-8.39729f, -9.93684f, 6.58656f)); 
  data[52].mAabb = Aabb(Vector3(39.4781f, -11.6169f, 5.46178f), Vector3(41.5791f, -3.75681f, 7.59252f)); 
  data[53].mAabb = Aabb(Vector3(-18.9607f, 27.6017f, 27.2815f), Vector3(-15.7995f, 32.5735f, 28.2581f)); 
  data[54].mAabb = Aabb(Vector3(-8.70699f, -42.4757f, -7.63059f), Vector3(-6.18798f, -37.7148f, -6.11913f)); 
  data[55].mAabb = Aabb(Vector3(-37.0886f, -30.2023f, 18.5653f), Vector3(-35.8424f, -23.976f, 20.0916f)); 
  data[56].mAabb = Aabb(Vector3(23.9611f, -0.161706f, 12.9458f), Vector3(31.2131f, 7.58548f, 18.5304f)); 
  data[57].mAabb = Aabb(Vector3(-33.0875f, -16.4202f, 28.3495f), Vector3(-29.0305f, -9.38062f, 30.6783f)); 
  data[58].mAabb = Aabb(Vector3(-8.36601f, 35.2806f, -7.56614f), Vector3(-5.22888f, 38.6207f, -2.88266f)); 
  data[59].mAabb = Aabb(Vector3(-20.9143f, -4.56231f, 3.86552f), Vector3(-16.3503f, -3.22991f, 4.38868f)); 
  data[60].mAabb = Aabb(Vector3(-20.7975f, -2.41288f, 3.37474f), Vector3(-14.4663f, 1.75013f, 5.23318f)); 
  data[61].mAabb = Aabb(Vector3(-12.4113f, -5.8613f, -24.3813f), Vector3(-9.51124f, -1.03773f, -21.1749f)); 
  data[62].mAabb = Aabb(Vector3(-10.348f, 28.2505f, 13.4667f), Vector3(-5.61794f, 36.1814f, 15.4258f)); 
  data[63].mAabb = Aabb(Vector3(2.41054f, -34.431f, 8.83407f), Vector3(9.77271f, -28.9576f, 12.9762f)); 
  data[64].mAabb = Aabb(Vector3(6.67244f, 19.3122f, -3.68525f), Vector3(12.2895f, 24.8685f, 1.6748f)); 
  data[65].mAabb = Aabb(Vector3(-36.4278f, 13.3973f, 14.2795f), Vector3(-35.0836f, 14.3305f, 17.7508f)); 
  data[66].mAabb = Aabb(Vector3(-10.1501f, 29.3064f, -21.2554f), Vector3(-8.0514f, 34.1957f, -13.2568f)); 
  data[67].mAabb = Aabb(Vector3(-2.93595f, 2.19166f, 30.7751f), Vector3(0.335011f, 9.99289f, 36.3657f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(21.831f, 116.741f, 28.2818f), Vector3(-73.9807f, 26.3531f, 72.9961f), Vector3(21.7801f, -75.1542f, 72.9961f), Vector3(117.592f, 15.2333f, 28.2818f), Vector3(-0.47205f, 5.73045f, -40.321f), Vector3(-6.12961f, 0.393177f, -37.6807f), Vector3(-0.186094f, -5.90701f, -37.6807f), Vector3(5.47146f, -0.569736f, -40.321f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(44); 
  data[0].mAabb = Aabb(Vector3(-21.7927f, 8.10133f, 20.2445f), Vector3(-19.5564f, 12.7526f, 26.3437f)); 
  data[1].mAabb = Aabb(Vector3(-22.3234f, 8.87549f, -28.2356f), Vector3(-21.4549f, 16.7303f, -27.2159f)); 
  data[2].mAabb = Aabb(Vector3(37.3736f, 21.9091f, -15.5512f), Vector3(41.4722f, 28.0107f, -11.475f)); 
  data[3].mAabb = Aabb(Vector3(4.139f, 15.5391f, -1.60039f), Vector3(10.4032f, 18.8152f, 2.83862f)); 
  data[4].mAabb = Aabb(Vector3(-19.8434f, -1.31521f, -47.7495f), Vector3(-19.0561f, -0.805268f, -43.0996f)); 
  data[5].mAabb = Aabb(Vector3(27.4317f, -27.4493f, 27.4446f), Vector3(34.0584f, -20.6445f, 35.045f)); 
  data[6].mAabb = Aabb(Vector3(-36.3469f, -16.2018f, -11.3513f), Vector3(-29.7984f, -10.5163f, -6.78841f)); 
  data[7].mAabb = Aabb(Vector3(-29.1113f, 11.4904f, -3.30121f), Vector3(-26.8993f, 17.2988f, 0.891253f)); 
  data[8].mAabb = Aabb(Vector3(17.9479f, -36.6783f, 14.8189f), Vector3(24.5616f, -31.5073f, 18.4811f)); 
  data[9].mAabb = Aabb(Vector3(33.1027f, 32.3127f, 2.72215f), Vector3(34.8426f, 36.1985f, 7.06607f)); 
  data[10].mAabb = Aabb(Vector3(-10.2072f, 16.8352f, -48.6999f), Vector3(-8.75978f, 17.597f, -42.6999f)); 
  data[11].mAabb = Aabb(Vector3(24.5698f, 2.3933f, 7.61133f), Vector3(25.9627f, 5.51419f, 10.9259f)); 
  data[12].mAabb = Aabb(Vector3(0.463895f, -18.987f, 18.6973f), Vector3(5.69151f, -14.2764f, 22.9623f)); 
  data[13].mAabb = Aabb(Vector3(-4.79794f, 44.2892f, -8.77284f), Vector3(-0.415982f, 45.7202f, -5.07673f)); 
  data[14].mAabb = Aabb(Vector3(13.7796f, -7.01382f, -16.4202f), Vector3(14.9538f, -5.22867f, -14.962f)); 
  data[15].mAabb = Aabb(Vector3(11.6766f, 36.2646f, 22.3447f), Vector3(13.6716f, 42.9411f, 24.883f)); 
  data[16].mAabb = Aabb(Vector3(-29.672f, 12.1497f, 8.98396f), Vector3(-25.1267f, 18.4932f, 14.906f)); 
  data[17].mAabb = Aabb(Vector3(2.50845f, 28.061f, 20.2354f), Vector3(2.94556f, 32.6487f, 23.2466f)); 
  data[18].mAabb = Aabb(Vector3(-29.3928f, 16.1341f, 31.9834f), Vector3(-21.4932f, 17.5164f, 32.4217f)); 
  data[19].mAabb = Aabb(Vector3(-13.9168f, -22.2232f, -39.8211f), Vector3(-7.141f, -18.4431f, -32.2034f)); 
  data[20].mAabb = Aabb(Vector3(25.8582f, -29.4476f, -18.3891f), Vector3(30.9097f, -23.1174f, -14.4813f)); 
  data[21].mAabb = Aabb(Vector3(-2.34786f, -18.9661f, -11.1313f), Vector3(2.75636f, -15.2544f, -4.1653f)); 
  data[22].mAabb = Aabb(Vector3(-27.4551f, -30.9771f, -20.4728f), Vector3(-22.3307f, -23.6393f, -14.5517f)); 
  data[23].mAabb = Aabb(Vector3(12.3381f, 43.8557f, 3.41203f), Vector3(15.7754f, 45.106f, 8.13007f)); 
  data[24].mAabb = Aabb(Vector3(-1.49637f, -25.701f, -45.0125f), Vector3(3.61643f, -21.2646f, -38.2768f)); 
  data[25].mAabb = Aabb(Vector3(-21.4757f, 22.5511f, 0.0769087f), Vector3(-13.829f, 25.8647f, 1.60066f)); 
  data[26].mAabb = Aabb(Vector3(18.5089f, -27.8211f, 5.07981f), Vector3(25.6804f, -27.2973f, 8.85014f)); 
  data[27].mAabb = Aabb(Vector3(1.06284f, 16.6923f, -29.6309f), Vector3(4.01767f, 18.3313f, -28.2765f)); 
  data[28].mAabb = Aabb(Vector3(-13.7554f, -22.3256f, 13.9879f), Vector3(-10.3144f, -18.1869f, 19.12f)); 
  data[29].mAabb = Aabb(Vector3(-15.2014f, -17.6701f, -14.6843f), Vector3(-10.7065f, -11.3159f, -13.6355f)); 
  data[30].mAabb = Aabb(Vector3(-11.7394f, 18.629f, 23.6992f), Vector3(-4.71774f, 26.4156f, 26.1823f)); 
  data[31].mAabb = Aabb(Vector3(-25.2751f, -36.5306f, 1.80562f), Vector3(-18.6229f, -33.329f, 5.48712f)); 
  data[32].mAabb = Aabb(Vector3(16.3689f, -12.0684f, 36.7387f), Vector3(22.6778f, -5.88494f, 43.3044f)); 
  data[33].mAabb = Aabb(Vector3(-34.6737f, -28.351f, -29.6128f), Vector3(-27.976f, -21.2625f, -28.2426f)); 
  data[34].mAabb = Aabb(Vector3(32.9337f, -18.8175f, -5.47077f), Vector3(37.8382f, -17.8425f, -3.77816f)); 
  data[35].mAabb = Aabb(Vector3(4.79731f, -28.752f, 6.3856f), Vector3(5.62524f, -27.9352f, 12.5168f)); 
  data[36].mAabb = Aabb(Vector3(-9.5042f, 4.49626f, -6.15696f), Vector3(-7.99112f, 7.59141f, -0.0352204f)); 
  data[37].mAabb = Aabb(Vector3(8.93535f, 10.4333f, 31.8283f), Vector3(15.4588f, 18.0958f, 37.8242f)); 
  data[38].mAabb = Aabb(Vector3(-2.27903f, 17.7188f, -39.9492f), Vector3(1.35863f, 19.8463f, -36.5302f)); 
  data[39].mAabb = Aabb(Vector3(-1.47127f, 29.7471f, -28.2709f), Vector3(0.262624f, 32.806f, -27.2681f)); 
  data[40].mAabb = Aabb(Vector3(-37.2103f, 29.6752f, 6.48345f), Vector3(-31.7218f, 34.708f, 14.2139f)); 
  data[41].mAabb = Aabb(Vector3(-40.4874f, -15.5798f, 2.36202f), Vector3(-38.4751f, -10.3221f, 6.23812f)); 
  data[42].mAabb = Aabb(Vector3(-39.8075f, 15.3613f, -10.8736f), Vector3(-38.6929f, 22.5632f, -5.60305f)); 
  data[43].mAabb = Aabb(Vector3(-10.7276f, 20.0004f, 25.0862f), Vector3(-4.22849f, 24.6164f, 25.4883f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-19.0768f, 15.6346f, 32.0913f), Vector3(-46.3087f, 25.1319f, 4.11904f), Vector3(-46.3087f, -13.8702f, -9.12325f), Vector3(-19.0768f, -23.3675f, 18.849f), Vector3(-8.25623f, 10.6495f, -3.22307f), Vector3(-13.6985f, 12.5476f, -8.81326f), Vector3(-13.6985f, 3.9867f, -11.7199f), Vector3(-8.25623f, 2.08867f, -6.12973f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(20.6419f, 3.87612f, -35.9532f), Vector3(27.0624f, 6.62058f, -33.9941f)); 
  data[1].mAabb = Aabb(Vector3(-29.4178f, -32.2346f, -20.2391f), Vector3(-25.4698f, -30.227f, -14.9781f)); 
  data[2].mAabb = Aabb(Vector3(-25.8975f, 6.21473f, 14.9814f), Vector3(-23.3731f, 8.43315f, 17.9784f)); 
  data[3].mAabb = Aabb(Vector3(-9.25435f, 17.2008f, -24.0114f), Vector3(-4.97375f, 23.9569f, -17.4733f)); 
  data[4].mAabb = Aabb(Vector3(17.6641f, 38.7504f, -7.0092f), Vector3(19.1902f, 39.3916f, -5.41215f)); 
  data[5].mAabb = Aabb(Vector3(-7.98524f, -12.9355f, -4.37581f), Vector3(-7.44468f, -11.6093f, 2.59669f)); 
  data[6].mAabb = Aabb(Vector3(-1.69471f, 39.6253f, 8.88028f), Vector3(4.12435f, 47.5857f, 12.8197f)); 
  data[7].mAabb = Aabb(Vector3(21.5648f, -43.1307f, -15.0522f), Vector3(26.0746f, -41.5188f, -8.47961f)); 
  data[8].mAabb = Aabb(Vector3(-11.4162f, 12.1931f, 9.44958f), Vector3(-10.0393f, 15.3555f, 17.1817f)); 
  data[9].mAabb = Aabb(Vector3(-2.11162f, 14.7644f, 28.814f), Vector3(1.93588f, 16.043f, 33.2182f)); 
  data[10].mAabb = Aabb(Vector3(-15.6422f, 23.3956f, -28.6791f), Vector3(-12.0864f, 24.5221f, -26.5746f)); 
  data[11].mAabb = Aabb(Vector3(-19.264f, -32.4222f, 17.0545f), Vector3(-15.2612f, -24.7354f, 24.7221f)); 
  data[12].mAabb = Aabb(Vector3(-6.17927f, -0.524079f, -11.0568f), Vector3(-0.529528f, 0.274395f, -6.72517f)); 
  data[13].mAabb = Aabb(Vector3(-9.24913f, 30.5446f, -27.0091f), Vector3(-6.71458f, 36.1185f, -20.3031f)); 
  data[14].mAabb = Aabb(Vector3(28.8526f, 22.2384f, -31.8275f), Vector3(31.7237f, 25.7427f, -24.0819f)); 
  data[15].mAabb = Aabb(Vector3(-46.7506f, 3.2491f, 0.416993f), Vector3(-45.3365f, 4.44999f, 5.4296f)); 
  data[16].mAabb = Aabb(Vector3(-7.12972f, 14.7414f, -35.4385f), Vector3(-4.91617f, 19.953f, -27.6394f)); 
  data[17].mAabb = Aabb(Vector3(1.27803f, -8.63322f, -4.53597f), Vector3(1.94244f, -1.55077f, 2.55783f)); 
  data[18].mAabb = Aabb(Vector3(15.658f, -10.4386f, -42.3727f), Vector3(16.5182f, -3.28538f, -38.9254f)); 
  data[19].mAabb = Aabb(Vector3(-14.6409f, -19.47f, -8.07137f), Vector3(-6.93659f, -13.4638f, -0.416269f)); 
  data[20].mAabb = Aabb(Vector3(-45.194f, -18.9699f, 0.124368f), Vector3(-41.9928f, -17.1453f, 4.12803f)); 
  data[21].mAabb = Aabb(Vector3(-8.17438f, -12.5303f, 3.65547f), Vector3(-4.42052f, -7.26809f, 10.2253f)); 
  data[22].mAabb = Aabb(Vector3(33.8767f, -25.971f, -16.6696f), Vector3(39.7992f, -24.402f, -9.63373f)); 
  data[23].mAabb = Aabb(Vector3(-6.36538f, -21.8979f, -40.3858f), Vector3(-2.29283f, -14.912f, -38.4912f)); 
  data[24].mAabb = Aabb(Vector3(-7.70789f, 17.0615f, -11.8285f), Vector3(-1.67985f, 23.0223f, -3.9628f)); 
  data[25].mAabb = Aabb(Vector3(24.7268f, -17.9731f, -33.1064f), Vector3(27.5079f, -12.4377f, -32.5131f)); 
  data[26].mAabb = Aabb(Vector3(14.84f, 27.7551f, 0.956803f), Vector3(21.6977f, 33.3547f, 7.19011f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-11.3761f, 99.8605f, 7.09462f), Vector3(-19.7f, 40.3509f, -90.5589f), Vector3(-19.7f, -60.3148f, -29.2137f), Vector3(-11.3761f, -0.805267f, 68.4398f), Vector3(20.6312f, 22.6483f, -13.4466f), Vector3(20.3285f, 20.4839f, -16.9983f), Vector3(20.3285f, 14.1688f, -13.15f), Vector3(20.6312f, 16.3332f, -9.59823f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(42); 
  data[0].mAabb = Aabb(Vector3(22.2304f, 26.0736f, -6.03951f), Vector3(22.8992f, 32.507f, -3.58034f)); 
  data[1].mAabb = Aabb(Vector3(-31.3906f, -7.46562f, 23.215f), Vector3(-29.7372f, -4.07359f, 23.6219f)); 
  data[2].mAabb = Aabb(Vector3(-8.42064f, -29.6867f, -17.5826f), Vector3(-1.18118f, -22.0622f, -9.6939f)); 
  data[3].mAabb = Aabb(Vector3(-5.98049f, -36.0486f, -17.7584f), Vector3(-0.904795f, -35.5811f, -11.4981f)); 
  data[4].mAabb = Aabb(Vector3(11.9016f, -25.9995f, -29.2371f), Vector3(18.7361f, -22.7056f, -26.1739f)); 
  data[5].mAabb = Aabb(Vector3(-2.37867f, -0.211627f, -12.7703f), Vector3(3.51624f, 1.74957f, -5.40463f)); 
  data[6].mAabb = Aabb(Vector3(-41.0092f, -27.943f, 14.2657f), Vector3(-35.622f, -27.4259f, 16.5984f)); 
  data[7].mAabb = Aabb(Vector3(-34.2747f, -32.3083f, 9.33959f), Vector3(-33.0054f, -31.2605f, 14.9937f)); 
  data[8].mAabb = Aabb(Vector3(5.46455f, -42.4231f, 12.0458f), Vector3(8.65805f, -40.9325f, 13.304f)); 
  data[9].mAabb = Aabb(Vector3(38.4365f, 2.42581f, -0.449609f), Vector3(39.4558f, 7.174f, 3.16973f)); 
  data[10].mAabb = Aabb(Vector3(-47.6811f, 3.77087f, -14.3772f), Vector3(-42.2006f, 5.04714f, -12.9537f)); 
  data[11].mAabb = Aabb(Vector3(-10.9506f, -45.1962f, 18.5337f), Vector3(-6.67907f, -40.9469f, 23.5319f)); 
  data[12].mAabb = Aabb(Vector3(-14.1413f, 7.41693f, 30.1817f), Vector3(-8.70281f, 14.7719f, 38.0105f)); 
  data[13].mAabb = Aabb(Vector3(-48.1637f, 6.06679f, 12.54f), Vector3(-43.1926f, 9.08261f, 14.234f)); 
  data[14].mAabb = Aabb(Vector3(-5.80215f, -27.965f, 15.7019f), Vector3(2.07956f, -22.2695f, 20.0326f)); 
  data[15].mAabb = Aabb(Vector3(-12.5778f, 22.5343f, 11.6134f), Vector3(-7.44803f, 29.0325f, 19.1141f)); 
  data[16].mAabb = Aabb(Vector3(-8.87405f, 17.3241f, -34.4828f), Vector3(-2.97589f, 24.3803f, -26.8918f)); 
  data[17].mAabb = Aabb(Vector3(-32.2202f, 3.81201f, -31.4473f), Vector3(-28.9515f, 6.46462f, -26.058f)); 
  data[18].mAabb = Aabb(Vector3(20.3511f, 12.5453f, -15.8156f), Vector3(21.802f, 19.5728f, -8.41884f)); 
  data[19].mAabb = Aabb(Vector3(28.719f, 18.673f, 22.0317f), Vector3(34.0396f, 22.9587f, 27.7592f)); 
  data[20].mAabb = Aabb(Vector3(-28.488f, 30.0017f, 15.4466f), Vector3(-23.3912f, 31.1325f, 23.2596f)); 
  data[21].mAabb = Aabb(Vector3(-24.9445f, -19.1138f, 10.0469f), Vector3(-22.2928f, -11.477f, 16.2306f)); 
  data[22].mAabb = Aabb(Vector3(-34.1879f, 5.7685f, 28.8239f), Vector3(-27.2096f, 8.25434f, 36.2571f)); 
  data[23].mAabb = Aabb(Vector3(-27.7316f, -0.0967157f, 8.22451f), Vector3(-25.2622f, 6.50932f, 13.3104f)); 
  data[24].mAabb = Aabb(Vector3(4.43096f, 11.4498f, 29.3437f), Vector3(10.6916f, 17.2074f, 33.3295f)); 
  data[25].mAabb = Aabb(Vector3(-19.1936f, -16.7627f, -17.6802f), Vector3(-17.1194f, -16.1129f, -14.2849f)); 
  data[26].mAabb = Aabb(Vector3(-2.26836f, 10.9642f, -49.7008f), Vector3(5.29768f, 15.4605f, -43.7356f)); 
  data[27].mAabb = Aabb(Vector3(-37.8362f, -5.51379f, -29.6214f), Vector3(-30.4149f, 1.03148f, -26.9649f)); 
  data[28].mAabb = Aabb(Vector3(-7.82101f, 39.1282f, -24.9363f), Vector3(-4.64468f, 44.712f, -17.5936f)); 
  data[29].mAabb = Aabb(Vector3(9.23488f, -47.2941f, 1.72158f), Vector3(11.8866f, -42.8514f, 4.21021f)); 
  data[30].mAabb = Aabb(Vector3(-19.1009f, 21.0439f, 22.0149f), Vector3(-17.5391f, 24.5414f, 28.8417f)); 
  data[31].mAabb = Aabb(Vector3(-15.0451f, 9.88349f, 34.3372f), Vector3(-13.5854f, 13.1039f, 39.314f)); 
  data[32].mAabb = Aabb(Vector3(18.027f, -2.40645f, 5.04136f), Vector3(25.0262f, 0.0967931f, 9.97139f)); 
  data[33].mAabb = Aabb(Vector3(-5.80369f, -6.67572f, -15.1366f), Vector3(-2.449f, -3.12898f, -8.40506f)); 
  data[34].mAabb = Aabb(Vector3(2.68123f, -2.77581f, 21.138f), Vector3(5.92343f, 1.11815f, 28.2452f)); 
  data[35].mAabb = Aabb(Vector3(-10.7819f, -23.4612f, 40.7117f), Vector3(-9.90205f, -17.7917f, 41.8841f)); 
  data[36].mAabb = Aabb(Vector3(30.8904f, -30.9021f, 17.0426f), Vector3(36.4998f, -30.1208f, 21.2705f)); 
  data[37].mAabb = Aabb(Vector3(-47.1645f, -2.27508f, 17.7296f), Vector3(-41.9084f, 5.26521f, 19.9334f)); 
  data[38].mAabb = Aabb(Vector3(-20.5325f, -21.2783f, -33.7058f), Vector3(-14.0882f, -17.3215f, -29.4611f)); 
  data[39].mAabb = Aabb(Vector3(34.7562f, 8.19224f, 19.9592f), Vector3(35.6619f, 11.6815f, 25.4009f)); 
  data[40].mAabb = Aabb(Vector3(5.34829f, -26.3984f, 30.7296f), Vector3(12.8698f, -21.8662f, 33.4492f)); 
  data[41].mAabb = Aabb(Vector3(-34.788f, -17.0858f, 16.5359f), Vector3(-33.2633f, -12.0247f, 21.0914f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(35.3276f, 92.8983f, -6.76069f), Vector3(-42.7824f, 61.8341f, 64.862f), Vector3(-1.98682f, -40.745f, 64.862f), Vector3(76.1231f, -9.68074f, -6.76069f), Vector3(-29.3672f, 10.656f, -32.0155f), Vector3(-33.1933f, 9.13431f, -28.5071f), Vector3(-31.2097f, 4.14645f, -28.5071f), Vector3(-27.3835f, 5.66811f, -32.0155f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(42); 
  data[0].mAabb = Aabb(Vector3(-9.69159f, -13.9288f, -25.4305f), Vector3(-8.32394f, -10.7871f, -20.6844f)); 
  data[1].mAabb = Aabb(Vector3(-28.0962f, 5.25031f, 23.5652f), Vector3(-26.3563f, 11.6223f, 29.7368f)); 
  data[2].mAabb = Aabb(Vector3(44.9234f, -13.3968f, 14.4636f), Vector3(45.4505f, -7.5817f, 21.8218f)); 
  data[3].mAabb = Aabb(Vector3(-48.1395f, -2.67799f, -2.94039f), Vector3(-47.3243f, 1.81761f, 4.56929f)); 
  data[4].mAabb = Aabb(Vector3(10.7848f, -31.7642f, -15.3799f), Vector3(11.4239f, -28.7888f, -13.916f)); 
  data[5].mAabb = Aabb(Vector3(-31.0429f, 26.5075f, -2.58949f), Vector3(-29.6297f, 27.8529f, 3.28315f)); 
  data[6].mAabb = Aabb(Vector3(13.727f, -29.5676f, -20.831f), Vector3(19.1706f, -22.9087f, -14.2442f)); 
  data[7].mAabb = Aabb(Vector3(13.3858f, -24.8703f, -21.1532f), Vector3(14.6291f, -22.728f, -19.201f)); 
  data[8].mAabb = Aabb(Vector3(-17.4764f, 41.5514f, 6.66654f), Vector3(-13.3913f, 42.3367f, 14.5731f)); 
  data[9].mAabb = Aabb(Vector3(-29.0823f, -18.8384f, -25.1409f), Vector3(-23.4165f, -14.6617f, -19.5744f)); 
  data[10].mAabb = Aabb(Vector3(-20.8569f, 33.1171f, -2.36892f), Vector3(-19.572f, 38.6077f, 3.14538f)); 
  data[11].mAabb = Aabb(Vector3(-39.0884f, -9.67758f, -8.95287f), Vector3(-38.3954f, -5.02611f, -5.37505f)); 
  data[12].mAabb = Aabb(Vector3(27.9313f, 1.49368f, 33.3099f), Vector3(34.4323f, 7.59246f, 39.4581f)); 
  data[13].mAabb = Aabb(Vector3(2.91745f, 5.30989f, 19.047f), Vector3(4.62884f, 7.08368f, 20.3198f)); 
  data[14].mAabb = Aabb(Vector3(-26.974f, -16.7197f, -15.5282f), Vector3(-19.4882f, -11.8245f, -11.8092f)); 
  data[15].mAabb = Aabb(Vector3(2.99659f, -6.017f, 42.8349f), Vector3(10.6264f, -0.0641117f, 45.851f)); 
  data[16].mAabb = Aabb(Vector3(10.3529f, 28.1294f, 7.64585f), Vector3(13.4105f, 29.7955f, 13.9919f)); 
  data[17].mAabb = Aabb(Vector3(-17.012f, 37.8341f, -21.6119f), Vector3(-11.4381f, 43.6033f, -14.6208f)); 
  data[18].mAabb = Aabb(Vector3(-3.17886f, -18.4515f, 34.5806f), Vector3(3.31608f, -16.8864f, 38.7861f)); 
  data[19].mAabb = Aabb(Vector3(18.0675f, 12.5529f, -30.3418f), Vector3(25.9026f, 16.9319f, -28.902f)); 
  data[20].mAabb = Aabb(Vector3(-13.2944f, 3.5049f, -7.8364f), Vector3(-10.1278f, 4.17325f, -3.52148f)); 
  data[21].mAabb = Aabb(Vector3(1.61335f, 0.126459f, -39.2163f), Vector3(9.39996f, 2.73917f, -32.4011f)); 
  data[22].mAabb = Aabb(Vector3(16.6429f, 8.88798f, 9.59323f), Vector3(18.4571f, 11.0723f, 13.3487f)); 
  data[23].mAabb = Aabb(Vector3(23.3108f, 3.63708f, 25.1635f), Vector3(26.9932f, 10.0717f, 32.0188f)); 
  data[24].mAabb = Aabb(Vector3(15.9965f, 3.57516f, -5.37002f), Vector3(19.4892f, 9.65051f, -0.715764f)); 
  data[25].mAabb = Aabb(Vector3(34.8582f, 16.0386f, -0.849472f), Vector3(42.3039f, 19.4577f, 2.14826f)); 
  data[26].mAabb = Aabb(Vector3(9.76814f, 44.824f, 12.0175f), Vector3(14.5558f, 47.0839f, 15.2479f)); 
  data[27].mAabb = Aabb(Vector3(8.47323f, 27.7094f, -11.4204f), Vector3(13.4385f, 28.8263f, -6.6801f)); 
  data[28].mAabb = Aabb(Vector3(19.5603f, 37.1967f, -22.1837f), Vector3(23.2787f, 38.5489f, -19.1586f)); 
  data[29].mAabb = Aabb(Vector3(-7.83349f, -23.6782f, 29.3365f), Vector3(-2.42334f, -20.1166f, 37.3011f)); 
  data[30].mAabb = Aabb(Vector3(-30.608f, -0.469808f, 32.8314f), Vector3(-27.2677f, 3.43551f, 40.2537f)); 
  data[31].mAabb = Aabb(Vector3(25.5781f, -32.2923f, 10.1028f), Vector3(27.2503f, -28.705f, 11.936f)); 
  data[32].mAabb = Aabb(Vector3(-9.0328f, 29.567f, 1.0686f), Vector3(-4.774f, 32.4858f, 4.64456f)); 
  data[33].mAabb = Aabb(Vector3(33.5572f, 2.65794f, 8.01043f), Vector3(35.8118f, 6.67575f, 11.087f)); 
  data[34].mAabb = Aabb(Vector3(-26.1982f, -5.85746f, -18.2188f), Vector3(-22.3304f, -1.68402f, -11.3634f)); 
  data[35].mAabb = Aabb(Vector3(19.1785f, -7.73349f, -34.0967f), Vector3(21.4925f, -6.74459f, -27.8235f)); 
  data[36].mAabb = Aabb(Vector3(14.7356f, -42.32f, 24.6328f), Vector3(15.8602f, -35.1631f, 27.7527f)); 
  data[37].mAabb = Aabb(Vector3(3.49396f, -2.28056f, 0.589044f), Vector3(4.46477f, 0.98507f, 1.53619f)); 
  data[38].mAabb = Aabb(Vector3(-0.136106f, -49.3497f, 16.4212f), Vector3(1.73834f, -42.2104f, 18.7595f)); 
  data[39].mAabb = Aabb(Vector3(-18.2799f, -31.9409f, 1.3912f), Vector3(-17.2794f, -26.3213f, 6.27879f)); 
  data[40].mAabb = Aabb(Vector3(-5.88251f, -49.5649f, 1.41199f), Vector3(-5.14574f, -42.7975f, 4.27334f)); 
  data[41].mAabb = Aabb(Vector3(5.79601f, -19.3704f, -10.4172f), Vector3(12.614f, -14.0431f, -4.31354f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-0.885708f, -73.3374f, 4.01311f), Vector3(-75.2669f, 6.09273f, 63.6566f), Vector3(-75.2669f, 80.6114f, -35.5835f), Vector3(-0.885708f, 1.18134f, -95.227f), Vector3(25.9667f, 34.0737f, 13.26f), Vector3(20.0386f, 40.4043f, 18.0136f), Vector3(20.0386f, 46.3491f, 10.0965f), Vector3(25.9667f, 40.0186f, 5.34295f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(-22.9208f, -32.019f, 26.2384f), Vector3(-15.452f, -30.5287f, 26.7175f)); 
  data[1].mAabb = Aabb(Vector3(-24.6966f, -15.8152f, 7.0279f), Vector3(-23.6379f, -14.2894f, 12.994f)); 
  data[2].mAabb = Aabb(Vector3(-14.2794f, -32.3525f, 11.266f), Vector3(-6.70272f, -29.3759f, 18.5855f)); 
  data[3].mAabb = Aabb(Vector3(-4.67426f, 11.9461f, 32.7556f), Vector3(0.628965f, 18.1954f, 37.954f)); 
  data[4].mAabb = Aabb(Vector3(-26.6935f, -17.017f, -23.6886f), Vector3(-25.4139f, -14.1482f, -19.9823f)); 
  data[5].mAabb = Aabb(Vector3(21.0879f, -16.3149f, 28.8066f), Vector3(22.0357f, -10.4696f, 31.4369f)); 
  data[6].mAabb = Aabb(Vector3(23.7699f, -13.6004f, -21.0313f), Vector3(31.0281f, -10.3208f, -18.3922f)); 
  data[7].mAabb = Aabb(Vector3(0.270985f, 10.9369f, -16.0191f), Vector3(3.01034f, 14.1506f, -12.0765f)); 
  data[8].mAabb = Aabb(Vector3(-9.81925f, 15.628f, -45.8759f), Vector3(-2.71037f, 19.7358f, -38.9196f)); 
  data[9].mAabb = Aabb(Vector3(25.4406f, 27.4188f, 2.36559f), Vector3(30.8533f, 29.8689f, 7.71613f)); 
  data[10].mAabb = Aabb(Vector3(-27.5525f, -5.26508f, 19.7341f), Vector3(-25.9141f, -4.85975f, 24.6161f)); 
  data[11].mAabb = Aabb(Vector3(21.7845f, 25.2332f, 15.5304f), Vector3(28.4963f, 32.4638f, 16.8294f)); 
  data[12].mAabb = Aabb(Vector3(36.8201f, -2.18763f, -7.47464f), Vector3(44.0241f, 5.42155f, -3.88801f)); 
  data[13].mAabb = Aabb(Vector3(13.648f, -35.4303f, -30.8243f), Vector3(14.8626f, -27.441f, -27.8425f)); 
  data[14].mAabb = Aabb(Vector3(-28.322f, -7.11921f, -23.9577f), Vector3(-26.9604f, -3.87747f, -22.2326f)); 
  data[15].mAabb = Aabb(Vector3(8.18137f, -3.41785f, -29.0369f), Vector3(9.9032f, -3.01599f, -28.0988f)); 
  data[16].mAabb = Aabb(Vector3(9.37216f, -42.552f, -27.1826f), Vector3(15.0583f, -38.2468f, -23.868f)); 
  data[17].mAabb = Aabb(Vector3(8.44178f, -22.4968f, 15.1717f), Vector3(12.8147f, -17.0098f, 21.5741f)); 
  data[18].mAabb = Aabb(Vector3(4.12334f, 5.07229f, -4.66004f), Vector3(7.54785f, 6.84143f, -0.345348f)); 
  data[19].mAabb = Aabb(Vector3(-18.2426f, -14.5527f, -1.68501f), Vector3(-10.79f, -9.91517f, 4.17116f)); 
  data[20].mAabb = Aabb(Vector3(-35.123f, -12.266f, 26.1656f), Vector3(-33.6782f, -10.4929f, 27.2387f)); 
  data[21].mAabb = Aabb(Vector3(32.2693f, -28.8308f, -18.1749f), Vector3(39.9177f, -25.7351f, -16.7184f)); 
  data[22].mAabb = Aabb(Vector3(4.43292f, -14.622f, 39.6518f), Vector3(6.9786f, -13.2704f, 44.8502f)); 
  data[23].mAabb = Aabb(Vector3(-33.4619f, -9.50738f, 28.6772f), Vector3(-25.7372f, -3.52457f, 30.8717f)); 
  data[24].mAabb = Aabb(Vector3(-28.44f, 16.9699f, 10.4972f), Vector3(-20.8563f, 20.8681f, 18.118f)); 
  data[25].mAabb = Aabb(Vector3(37.9028f, -26.085f, 0.241451f), Vector3(42.8767f, -19.688f, 1.89277f)); 
  data[26].mAabb = Aabb(Vector3(10.4142f, 27.1208f, 8.13355f), Vector3(13.9011f, 28.0958f, 13.7418f)); 
  data[27].mAabb = Aabb(Vector3(-12.4818f, -16.1421f, -19.832f), Vector3(-10.4765f, -10.8488f, -15.02f)); 
  data[28].mAabb = Aabb(Vector3(12.6817f, -34.2558f, 32.0479f), Vector3(16.0962f, -31.3673f, 37.1443f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(18.2139f, -41.9342f, 29.6414f), Vector3(59.0815f, -20.5455f, -7.05808f), Vector3(31.1203f, 32.8804f, -7.05808f), Vector3(-9.74736f, 11.4918f, 29.6414f), Vector3(0.336961f, -22.4278f, -19.9698f), Vector3(5.74544f, -19.5972f, -24.8266f), Vector3(1.49955f, -11.4845f, -24.8266f), Vector3(-3.90893f, -14.3151f, -19.9698f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(59); 
  data[0].mAabb = Aabb(Vector3(36.4873f, 15.2722f, -14.7232f), Vector3(41.1882f, 19.9383f, -9.45984f)); 
  data[1].mAabb = Aabb(Vector3(33.67f, -19.5066f, -10.3135f), Vector3(40.7768f, -18.1023f, -3.87472f)); 
  data[2].mAabb = Aabb(Vector3(2.61314f, 0.680546f, 28.4691f), Vector3(7.77883f, 3.30648f, 33.8784f)); 
  data[3].mAabb = Aabb(Vector3(-4.31919f, -14.8979f, -7.13712f), Vector3(0.00733423f, -13.8371f, -3.25755f)); 
  data[4].mAabb = Aabb(Vector3(-3.47443f, 22.0883f, -26.5317f), Vector3(-3.00879f, 29.2783f, -24.3305f)); 
  data[5].mAabb = Aabb(Vector3(-4.60313f, -11.0516f, 31.5352f), Vector3(0.437071f, -9.98152f, 32.7795f)); 
  data[6].mAabb = Aabb(Vector3(-50.4088f, 6.31267f, -2.26712f), Vector3(-46.2477f, 8.19593f, 4.37649f)); 
  data[7].mAabb = Aabb(Vector3(-7.944f, 25.6861f, 33.3248f), Vector3(-3.25657f, 32.4995f, 36.8175f)); 
  data[8].mAabb = Aabb(Vector3(14.0537f, 16.9108f, -6.00634f), Vector3(19.3994f, 19.0404f, -0.649772f)); 
  data[9].mAabb = Aabb(Vector3(-27.7513f, 14.4446f, 4.05279f), Vector3(-20.2207f, 19.068f, 8.91834f)); 
  data[10].mAabb = Aabb(Vector3(15.7768f, 7.6917f, 18.068f), Vector3(20.5329f, 10.66f, 20.5386f)); 
  data[11].mAabb = Aabb(Vector3(23.0098f, -2.5187f, -38.3499f), Vector3(26.7047f, 0.59686f, -37.6115f)); 
  data[12].mAabb = Aabb(Vector3(-9.29181f, -1.95237f, -20.6269f), Vector3(-3.22271f, -1.53683f, -12.8308f)); 
  data[13].mAabb = Aabb(Vector3(43.341f, 3.67751f, -14.591f), Vector3(46.268f, 6.8434f, -13.8496f)); 
  data[14].mAabb = Aabb(Vector3(-8.89905f, -20.564f, 35.3649f), Vector3(-5.90085f, -17.9394f, 37.7561f)); 
  data[15].mAabb = Aabb(Vector3(28.574f, -10.5854f, -0.519341f), Vector3(36.5555f, -5.98824f, 7.22854f)); 
  data[16].mAabb = Aabb(Vector3(-5.05541f, 0.306896f, 27.7229f), Vector3(1.54622f, 4.3203f, 35.3537f)); 
  data[17].mAabb = Aabb(Vector3(-35.2161f, -14.4707f, -6.87482f), Vector3(-30.8645f, -12.6349f, 0.279292f)); 
  data[18].mAabb = Aabb(Vector3(-2.40126f, 34.2916f, -26.5973f), Vector3(-0.988376f, 40.0084f, -18.9016f)); 
  data[19].mAabb = Aabb(Vector3(-19.1018f, 38.9746f, -9.63226f), Vector3(-17.8448f, 39.5776f, -2.5751f)); 
  data[20].mAabb = Aabb(Vector3(-24.816f, 11.9245f, 2.15905f), Vector3(-20.8043f, 14.2561f, 5.75125f)); 
  data[21].mAabb = Aabb(Vector3(-9.44655f, -27.9962f, -26.5814f), Vector3(-6.44858f, -20.0943f, -20.0387f)); 
  data[22].mAabb = Aabb(Vector3(5.50439f, -6.79893f, -42.9331f), Vector3(11.7451f, -3.68708f, -41.1192f)); 
  data[23].mAabb = Aabb(Vector3(-18.514f, -5.93184f, 41.8785f), Vector3(-12.1257f, -1.36967f, 42.9773f)); 
  data[24].mAabb = Aabb(Vector3(4.66861f, -39.8604f, 29.8665f), Vector3(11.7534f, -32.2803f, 32.8257f)); 
  data[25].mAabb = Aabb(Vector3(-18.4823f, -16.043f, -33.9107f), Vector3(-16.2971f, -11.4345f, -32.7439f)); 
  data[26].mAabb = Aabb(Vector3(-5.3275f, 34.1103f, -24.9516f), Vector3(1.29895f, 37.8245f, -18.7339f)); 
  data[27].mAabb = Aabb(Vector3(-0.0975356f, -13.6805f, 2.50801f), Vector3(7.72433f, -8.17107f, 8.14731f)); 
  data[28].mAabb = Aabb(Vector3(12.079f, -2.49457f, 12.7364f), Vector3(18.5645f, 4.98263f, 16.5712f)); 
  data[29].mAabb = Aabb(Vector3(35.8623f, -17.3302f, -30.6681f), Vector3(38.8206f, -15.3567f, -25.546f)); 
  data[30].mAabb = Aabb(Vector3(-7.03986f, -33.0324f, -29.1487f), Vector3(-2.88079f, -26.3865f, -24.8766f)); 
  data[31].mAabb = Aabb(Vector3(-41.553f, -18.0132f, -16.9887f), Vector3(-33.5894f, -13.8583f, -14.6202f)); 
  data[32].mAabb = Aabb(Vector3(-27.3856f, -17.5889f, 6.19831f), Vector3(-24.5119f, -15.0812f, 11.1699f)); 
  data[33].mAabb = Aabb(Vector3(18.0457f, -16.6253f, 2.02029f), Vector3(20.0853f, -11.6955f, 5.12286f)); 
  data[34].mAabb = Aabb(Vector3(39.8756f, 14.2448f, -5.99006f), Vector3(41.4358f, 19.0264f, 0.64729f)); 
  data[35].mAabb = Aabb(Vector3(6.17816f, 13.3993f, -42.6125f), Vector3(10.8814f, 17.704f, -39.9738f)); 
  data[36].mAabb = Aabb(Vector3(-50.2114f, -3.40615f, 4.42423f), Vector3(-48.4439f, 2.72511f, 9.57368f)); 
  data[37].mAabb = Aabb(Vector3(-50.0961f, -0.523396f, -6.94106f), Vector3(-47.9646f, 4.70027f, -1.82524f)); 
  data[38].mAabb = Aabb(Vector3(20.4313f, -5.89181f, -46.934f), Vector3(24.1548f, -1.4623f, -39.5145f)); 
  data[39].mAabb = Aabb(Vector3(-18.7718f, -28.1223f, 17.6491f), Vector3(-14.6793f, -22.8915f, 19.6026f)); 
  data[40].mAabb = Aabb(Vector3(34.7085f, -26.9261f, -3.45372f), Vector3(40.2669f, -18.9871f, 3.84814f)); 
  data[41].mAabb = Aabb(Vector3(16.67f, -34.9885f, -17.5951f), Vector3(20.8362f, -30.3303f, -10.4482f)); 
  data[42].mAabb = Aabb(Vector3(19.9514f, -30.6851f, -4.42341f), Vector3(20.6541f, -22.7185f, 1.44969f)); 
  data[43].mAabb = Aabb(Vector3(-22.9911f, -27.1103f, 11.4224f), Vector3(-15.6201f, -24.5182f, 19.0304f)); 
  data[44].mAabb = Aabb(Vector3(42.7953f, -9.81375f, -2.29469f), Vector3(45.4681f, -3.65141f, 2.29741f)); 
  data[45].mAabb = Aabb(Vector3(-25.4005f, -21.3363f, -12.2873f), Vector3(-21.0914f, -20.0463f, -10.1155f)); 
  data[46].mAabb = Aabb(Vector3(-8.74077f, -14.9477f, 39.5037f), Vector3(-7.68809f, -9.92954f, 41.9559f)); 
  data[47].mAabb = Aabb(Vector3(-25.6642f, 21.4031f, 24.9533f), Vector3(-22.9223f, 26.907f, 27.6284f)); 
  data[48].mAabb = Aabb(Vector3(-0.43553f, -36.1039f, -26.9173f), Vector3(6.65457f, -32.973f, -25.2771f)); 
  data[49].mAabb = Aabb(Vector3(-12.3138f, -49.4364f, -8.03616f), Vector3(-9.0048f, -42.4625f, -4.514f)); 
  data[50].mAabb = Aabb(Vector3(19.2554f, -15.5628f, -31.6351f), Vector3(20.7717f, -12.9687f, -24.423f)); 
  data[51].mAabb = Aabb(Vector3(22.8989f, -42.0336f, 2.73152f), Vector3(30.3993f, -40.6755f, 8.68024f)); 
  data[52].mAabb = Aabb(Vector3(-18.6783f, 29.5065f, 22.748f), Vector3(-14.8757f, 31.1861f, 25.2473f)); 
  data[53].mAabb = Aabb(Vector3(22.8856f, -24.3235f, -30.6411f), Vector3(28.286f, -19.2662f, -26.0418f)); 
  data[54].mAabb = Aabb(Vector3(2.9896f, -31.7409f, -29.2283f), Vector3(10.0549f, -28.7712f, -21.8682f)); 
  data[55].mAabb = Aabb(Vector3(35.1833f, 19.5558f, 13.7086f), Vector3(36.0915f, 24.7516f, 21.1371f)); 
  data[56].mAabb = Aabb(Vector3(29.8064f, -28.1072f, 6.35321f), Vector3(35.239f, -25.6008f, 9.73551f)); 
  data[57].mAabb = Aabb(Vector3(-20.4667f, -21.4893f, -43.7771f), Vector3(-13.7118f, -13.6614f, -39.6394f)); 
  data[58].mAabb = Aabb(Vector3(-15.9901f, 4.46521f, 29.3412f), Vector3(-13.5026f, 9.78305f, 37.3f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-8.17737f, -26.4006f, -25.8128f), Vector3(2.55109f, -49.7842f, 36.3858f), Vector3(2.55109f, 14.8036f, 60.6675f), Vector3(-8.17737f, 38.1872f, -1.53108f), Vector3(20.4673f, -7.0343f, 9.64859f), Vector3(21.439f, -9.15216f, 15.282f), Vector3(21.439f, -1.86212f, 18.0226f), Vector3(20.4673f, 0.255745f, 12.3893f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(48); 
  data[0].mAabb = Aabb(Vector3(33.1935f, 21.4963f, -7.4802f), Vector3(38.2704f, 25.3791f, -6.57434f)); 
  data[1].mAabb = Aabb(Vector3(16.5512f, -33.1609f, -26.0704f), Vector3(22.7068f, -28.4326f, -25.3768f)); 
  data[2].mAabb = Aabb(Vector3(-3.79948f, 19.6085f, -16.9161f), Vector3(-0.681138f, 22.2772f, -10.8278f)); 
  data[3].mAabb = Aabb(Vector3(-27.2013f, -6.05423f, 38.3933f), Vector3(-25.6835f, 1.35687f, 39.8957f)); 
  data[4].mAabb = Aabb(Vector3(-10.16f, 6.49094f, 19.8907f), Vector3(-3.54009f, 13.2753f, 22.483f)); 
  data[5].mAabb = Aabb(Vector3(2.54805f, -0.932078f, 45.2763f), Vector3(10.4485f, 5.0454f, 51.2429f)); 
  data[6].mAabb = Aabb(Vector3(-11.0548f, -2.81932f, 27.0231f), Vector3(-7.39946f, -1.04693f, 32.9161f)); 
  data[7].mAabb = Aabb(Vector3(-0.222857f, -29.9926f, -21.5947f), Vector3(7.73563f, -27.6185f, -18.889f)); 
  data[8].mAabb = Aabb(Vector3(-16.0292f, 34.5347f, -23.2923f), Vector3(-10.3027f, 38.1863f, -20.7301f)); 
  data[9].mAabb = Aabb(Vector3(-13.8862f, 18.5952f, -24.3012f), Vector3(-11.5977f, 23.7041f, -16.7983f)); 
  data[10].mAabb = Aabb(Vector3(-22.2858f, 21.7877f, 20.2469f), Vector3(-15.744f, 28.6187f, 23.2177f)); 
  data[11].mAabb = Aabb(Vector3(27.5222f, 29.5293f, -1.91901f), Vector3(30.7974f, 30.6912f, 4.10207f)); 
  data[12].mAabb = Aabb(Vector3(-14.4143f, -44.8648f, 7.32039f), Vector3(-9.71849f, -38.5447f, 15.2313f)); 
  data[13].mAabb = Aabb(Vector3(-13.1198f, 0.612594f, -23.1825f), Vector3(-10.1752f, 6.48338f, -18.5841f)); 
  data[14].mAabb = Aabb(Vector3(31.3947f, -26.4444f, 23.3926f), Vector3(34.0093f, -18.9015f, 28.1958f)); 
  data[15].mAabb = Aabb(Vector3(-23.9531f, -33.757f, -30.2799f), Vector3(-20.0617f, -26.5551f, -27.5987f)); 
  data[16].mAabb = Aabb(Vector3(-14.9611f, -5.59465f, -10.851f), Vector3(-10.6328f, -3.92477f, -4.30199f)); 
  data[17].mAabb = Aabb(Vector3(-17.9832f, -39.3451f, -1.07482f), Vector3(-17.0449f, -34.7222f, -0.413193f)); 
  data[18].mAabb = Aabb(Vector3(11.439f, 10.6843f, -15.8445f), Vector3(11.8733f, 12.8673f, -10.3531f)); 
  data[19].mAabb = Aabb(Vector3(-8.74938f, 15.33f, -43.1438f), Vector3(-6.15754f, 19.9142f, -42.0649f)); 
  data[20].mAabb = Aabb(Vector3(-5.28834f, 21.4647f, 36.1894f), Vector3(-4.47827f, 28.0429f, 43.7374f)); 
  data[21].mAabb = Aabb(Vector3(-38.4919f, -1.99495f, 5.28491f), Vector3(-35.7359f, 3.63207f, 8.23209f)); 
  data[22].mAabb = Aabb(Vector3(-11.4608f, -18.5632f, -30.6925f), Vector3(-6.8165f, -16.8331f, -26.9168f)); 
  data[23].mAabb = Aabb(Vector3(35.3226f, -4.01471f, 3.59693f), Vector3(42.2919f, -1.68821f, 11.5726f)); 
  data[24].mAabb = Aabb(Vector3(-26.6739f, 1.20319f, 8.10839f), Vector3(-20.3307f, 7.22265f, 14.5433f)); 
  data[25].mAabb = Aabb(Vector3(13.0634f, -15.0453f, 8.91018f), Vector3(13.8839f, -11.9601f, 9.34752f)); 
  data[26].mAabb = Aabb(Vector3(-21.9095f, 0.823147f, -25.6575f), Vector3(-19.0322f, 1.56619f, -18.8506f)); 
  data[27].mAabb = Aabb(Vector3(40.7197f, 7.15582f, 6.44423f), Vector3(43.5381f, 14.5437f, 9.50204f)); 
  data[28].mAabb = Aabb(Vector3(-6.23375f, 23.7769f, 34.9579f), Vector3(-3.90308f, 26.0948f, 39.5963f)); 
  data[29].mAabb = Aabb(Vector3(11.0913f, 36.8698f, -5.11498f), Vector3(16.8526f, 42.7995f, -1.63922f)); 
  data[30].mAabb = Aabb(Vector3(45.9169f, -1.67933f, 0.768085f), Vector3(52.7316f, 4.50806f, 5.23354f)); 
  data[31].mAabb = Aabb(Vector3(-24.5364f, -11.1601f, 0.644164f), Vector3(-22.4527f, -8.72764f, 6.29089f)); 
  data[32].mAabb = Aabb(Vector3(-14.6128f, 4.29998f, 42.9631f), Vector3(-7.10403f, 5.04905f, 50.0477f)); 
  data[33].mAabb = Aabb(Vector3(8.85913f, -39.9837f, 28.8111f), Vector3(12.494f, -33.4231f, 29.5604f)); 
  data[34].mAabb = Aabb(Vector3(21.2624f, -7.77098f, -34.852f), Vector3(24.2546f, -5.15989f, -29.4191f)); 
  data[35].mAabb = Aabb(Vector3(2.26073f, 29.8503f, 2.50531f), Vector3(2.81265f, 34.1952f, 8.05973f)); 
  data[36].mAabb = Aabb(Vector3(24.497f, -33.6403f, 14.5366f), Vector3(27.125f, -26.1491f, 20.0764f)); 
  data[37].mAabb = Aabb(Vector3(13.598f, 12.6946f, -5.38662f), Vector3(18.8931f, 20.2309f, -4.56403f)); 
  data[38].mAabb = Aabb(Vector3(9.06885f, -18.5529f, -5.36222f), Vector3(12.4291f, -12.8805f, 0.448724f)); 
  data[39].mAabb = Aabb(Vector3(3.07147f, -18.745f, -40.4796f), Vector3(7.97066f, -12.8529f, -39.187f)); 
  data[40].mAabb = Aabb(Vector3(42.9737f, -10.0652f, -10.9256f), Vector3(43.4632f, -8.13901f, -10.0168f)); 
  data[41].mAabb = Aabb(Vector3(-22.0444f, -11.0488f, 41.6391f), Vector3(-16.5306f, -3.63657f, 47.8585f)); 
  data[42].mAabb = Aabb(Vector3(26.9111f, -1.63715f, -19.7125f), Vector3(32.5008f, 4.77869f, -15.4193f)); 
  data[43].mAabb = Aabb(Vector3(17.0291f, 37.254f, 15.3998f), Vector3(24.5147f, 42.9931f, 16.6296f)); 
  data[44].mAabb = Aabb(Vector3(-26.7873f, -2.91733f, -14.046f), Vector3(-23.8629f, 1.97513f, -10.3668f)); 
  data[45].mAabb = Aabb(Vector3(6.10475f, -33.8197f, 21.6547f), Vector3(9.95556f, -27.2904f, 25.31f)); 
  data[46].mAabb = Aabb(Vector3(-16.2142f, 27.4517f, 17.5353f), Vector3(-8.68322f, 31.3503f, 20.6193f)); 
  data[47].mAabb = Aabb(Vector3(14.698f, 26.8922f, 5.62137f), Vector3(22.3981f, 32.1052f, 8.52193f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(42.7925f, 82.5297f, 59.9714f), Vector3(-68.2374f, 37.7803f, 9.68188f), Vector3(-19.9128f, -82.1203f, 9.68188f), Vector3(91.1172f, -37.371f, 59.9714f), Vector3(36.8145f, 13.8818f, -24.368f), Vector3(30.8542f, 11.4796f, -27.0676f), Vector3(33.2467f, 5.54351f, -27.0676f), Vector3(39.207f, 7.94575f, -24.368f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(46); 
  data[0].mAabb = Aabb(Vector3(-0.261767f, -19.3444f, -12.5468f), Vector3(7.65126f, -11.9236f, -7.97956f)); 
  data[1].mAabb = Aabb(Vector3(24.0339f, 15.0873f, -3.75833f), Vector3(26.9496f, 18.9448f, -1.54247f)); 
  data[2].mAabb = Aabb(Vector3(27.0793f, -17.6078f, 14.3158f), Vector3(34.1339f, -13.863f, 16.5261f)); 
  data[3].mAabb = Aabb(Vector3(11.0682f, 13.5171f, 12.6623f), Vector3(15.2889f, 18.0478f, 17.4265f)); 
  data[4].mAabb = Aabb(Vector3(-33.2168f, 12.3235f, 10.3537f), Vector3(-28.5066f, 17.3644f, 13.9384f)); 
  data[5].mAabb = Aabb(Vector3(16.8519f, -22.7096f, -6.40551f), Vector3(24.7644f, -21.4677f, -1.31521f)); 
  data[6].mAabb = Aabb(Vector3(-28.3002f, 29.4504f, 1.27666f), Vector3(-26.0997f, 32.1459f, 7.95784f)); 
  data[7].mAabb = Aabb(Vector3(-2.07957f, 44.8989f, 2.26065f), Vector3(-1.28295f, 47.9649f, 5.1561f)); 
  data[8].mAabb = Aabb(Vector3(25.0769f, -37.1722f, -22.246f), Vector3(29.0129f, -33.3464f, -20.9398f)); 
  data[9].mAabb = Aabb(Vector3(-19.953f, -11.8477f, 43.3248f), Vector3(-13.6415f, -5.94425f, 48.9571f)); 
  data[10].mAabb = Aabb(Vector3(15.5814f, 1.4467f, 5.18555f), Vector3(17.7174f, 5.41302f, 9.66075f)); 
  data[11].mAabb = Aabb(Vector3(3.27414f, -13.4375f, 19.2212f), Vector3(9.41258f, -10.9424f, 24.343f)); 
  data[12].mAabb = Aabb(Vector3(-1.06261f, 30.8218f, 11.5657f), Vector3(0.700972f, 37.1419f, 13.2715f)); 
  data[13].mAabb = Aabb(Vector3(-26.1134f, 18.5559f, -20.6174f), Vector3(-18.7536f, 22.7581f, -18.1615f)); 
  data[14].mAabb = Aabb(Vector3(-5.45855f, -25.0008f, -14.0149f), Vector3(-0.777617f, -21.2631f, -6.28164f)); 
  data[15].mAabb = Aabb(Vector3(-8.90939f, -0.785103f, 12.2898f), Vector3(-7.63984f, 7.03816f, 20.0901f)); 
  data[16].mAabb = Aabb(Vector3(-32.6284f, 26.9529f, -7.93994f), Vector3(-27.5845f, 29.9386f, 0.0136735f)); 
  data[17].mAabb = Aabb(Vector3(7.9516f, 19.9944f, 10.2309f), Vector3(12.5043f, 24.0992f, 16.5384f)); 
  data[18].mAabb = Aabb(Vector3(29.4422f, 2.76266f, -35.7054f), Vector3(35.5874f, 10.2313f, -33.9945f)); 
  data[19].mAabb = Aabb(Vector3(3.30199f, -0.120505f, 41.0724f), Vector3(8.24641f, 6.69358f, 47.8292f)); 
  data[20].mAabb = Aabb(Vector3(-0.987459f, -27.8887f, 21.4808f), Vector3(3.33071f, -20.3266f, 29.0223f)); 
  data[21].mAabb = Aabb(Vector3(-45.0906f, 14.6858f, 14.7128f), Vector3(-43.0062f, 16.1145f, 19.2414f)); 
  data[22].mAabb = Aabb(Vector3(-33.0969f, -8.27317f, 1.37579f), Vector3(-29.7791f, -7.14418f, 2.05691f)); 
  data[23].mAabb = Aabb(Vector3(-26.6615f, -4.65109f, -34.2374f), Vector3(-22.9879f, 0.461714f, -28.3096f)); 
  data[24].mAabb = Aabb(Vector3(7.20647f, -2.98615f, 43.0435f), Vector3(11.9906f, -0.815287f, 50.8039f)); 
  data[25].mAabb = Aabb(Vector3(6.86961f, 12.9321f, 20.775f), Vector3(7.32063f, 20.7707f, 27.0466f)); 
  data[26].mAabb = Aabb(Vector3(-2.41088f, 18.0283f, 21.2387f), Vector3(5.39707f, 25.0115f, 23.4233f)); 
  data[27].mAabb = Aabb(Vector3(18.2726f, -40.3853f, 19.604f), Vector3(18.7732f, -38.8706f, 27.5518f)); 
  data[28].mAabb = Aabb(Vector3(31.8285f, 19.6407f, 7.32415f), Vector3(33.0364f, 24.6169f, 14.851f)); 
  data[29].mAabb = Aabb(Vector3(27.948f, 4.85392f, 7.2853f), Vector3(29.9488f, 8.6442f, 8.54812f)); 
  data[30].mAabb = Aabb(Vector3(12.7485f, -0.751211f, 6.46355f), Vector3(17.0797f, 2.77048f, 13.2957f)); 
  data[31].mAabb = Aabb(Vector3(15.7539f, 5.79342f, -15.0109f), Vector3(23.4834f, 7.9796f, -13.1227f)); 
  data[32].mAabb = Aabb(Vector3(-7.59152f, 33.8882f, 22.9026f), Vector3(-4.40104f, 35.8794f, 27.2612f)); 
  data[33].mAabb = Aabb(Vector3(-40.9553f, 3.20407f, 10.3253f), Vector3(-34.9184f, 9.6322f, 17.2426f)); 
  data[34].mAabb = Aabb(Vector3(-1.31777f, 15.4522f, 36.5611f), Vector3(3.63314f, 21.397f, 41.4169f)); 
  data[35].mAabb = Aabb(Vector3(27.1242f, 0.119955f, 30.3455f), Vector3(33.5235f, 5.55423f, 31.6932f)); 
  data[36].mAabb = Aabb(Vector3(-29.6851f, 20.3947f, 15.1147f), Vector3(-25.306f, 21.8311f, 21.2174f)); 
  data[37].mAabb = Aabb(Vector3(15.0769f, 22.4968f, -15.2516f), Vector3(16.1435f, 24.1243f, -9.05035f)); 
  data[38].mAabb = Aabb(Vector3(-27.4605f, -13.7079f, -1.40533f), Vector3(-25.7948f, -10.513f, 4.00575f)); 
  data[39].mAabb = Aabb(Vector3(20.8814f, 28.8173f, 6.29068f), Vector3(22.5944f, 31.4451f, 9.98075f)); 
  data[40].mAabb = Aabb(Vector3(20.31f, -22.0801f, 17.8564f), Vector3(22.3438f, -16.6331f, 19.667f)); 
  data[41].mAabb = Aabb(Vector3(-26.2063f, 15.4224f, 20.1595f), Vector3(-18.7991f, 21.6154f, 24.4076f)); 
  data[42].mAabb = Aabb(Vector3(-1.43074f, -30.8714f, -35.9383f), Vector3(5.91866f, -28.5317f, -33.7111f)); 
  data[43].mAabb = Aabb(Vector3(-19.86f, -10.9529f, -6.97396f), Vector3(-12.6475f, -8.08572f, 0.260172f)); 
  data[44].mAabb = Aabb(Vector3(-28.4738f, -39.0599f, 6.16534f), Vector3(-22.334f, -33.5286f, 11.9311f)); 
  data[45].mAabb = Aabb(Vector3(-10.6446f, -46.2357f, 4.66895f), Vector3(-7.71245f, -44.4344f, 10.7909f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-4.25546f, 22.8165f, 23.7969f), Vector3(17.2209f, 29.2977f, -22.7101f), Vector3(17.2209f, -25.5336f, -30.3515f), Vector3(-4.25546f, -32.0149f, 16.1556f), Vector3(35.4756f, 0.703388f, 13.1084f), Vector3(37.5059f, 1.3161f, 8.71183f), Vector3(37.5059f, -7.20932f, 7.52371f), Vector3(35.4756f, -7.82203f, 11.9203f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(56); 
  data[0].mAabb = Aabb(Vector3(-0.795379f, -27.2196f, 23.7318f), Vector3(4.64446f, -19.5473f, 25.5098f)); 
  data[1].mAabb = Aabb(Vector3(-41.3652f, 14.7246f, 17.1446f), Vector3(-39.9554f, 20.3022f, 23.8434f)); 
  data[2].mAabb = Aabb(Vector3(-40.3479f, -17.6646f, -20.9252f), Vector3(-39.5677f, -16.0868f, -17.9681f)); 
  data[3].mAabb = Aabb(Vector3(-12.5986f, -28.1421f, 34.8847f), Vector3(-6.43211f, -21.1986f, 38.3023f)); 
  data[4].mAabb = Aabb(Vector3(-6.1321f, 29.1412f, 34.3643f), Vector3(-4.99963f, 33.9506f, 42.1789f)); 
  data[5].mAabb = Aabb(Vector3(25.504f, -12.4808f, -0.505699f), Vector3(32.8873f, -11.4903f, 3.43882f)); 
  data[6].mAabb = Aabb(Vector3(8.65019f, -29.5893f, -11.5958f), Vector3(12.0837f, -21.5926f, -6.45959f)); 
  data[7].mAabb = Aabb(Vector3(28.8596f, 33.5769f, 12.373f), Vector3(33.9868f, 35.2215f, 16.2941f)); 
  data[8].mAabb = Aabb(Vector3(26.3233f, -38.2352f, 11.6342f), Vector3(31.8123f, -34.7363f, 15.1482f)); 
  data[9].mAabb = Aabb(Vector3(-12.4409f, 0.738718f, -42.6627f), Vector3(-7.35552f, 4.87088f, -35.2553f)); 
  data[10].mAabb = Aabb(Vector3(-14.5516f, 12.0709f, 8.76361f), Vector3(-8.51386f, 19.1311f, 16.0019f)); 
  data[11].mAabb = Aabb(Vector3(-32.8177f, -17.9778f, -23.0818f), Vector3(-25.135f, -16.5027f, -20.0344f)); 
  data[12].mAabb = Aabb(Vector3(3.20258f, -39.8617f, -15.5781f), Vector3(4.11725f, -32.292f, -12.9236f)); 
  data[13].mAabb = Aabb(Vector3(5.76274f, -34.1884f, -14.5188f), Vector3(7.01489f, -30.1599f, -7.40849f)); 
  data[14].mAabb = Aabb(Vector3(10.1692f, -27.0238f, -30.3266f), Vector3(17.5406f, -19.8708f, -22.3774f)); 
  data[15].mAabb = Aabb(Vector3(-25.8093f, -36.2558f, 1.19753f), Vector3(-24.9865f, -30.4836f, 9.0579f)); 
  data[16].mAabb = Aabb(Vector3(14.2236f, -44.8619f, -21.967f), Vector3(14.64f, -38.8659f, -14.442f)); 
  data[17].mAabb = Aabb(Vector3(33.2829f, -29.8079f, -23.3195f), Vector3(36.3634f, -25.3355f, -16.5934f)); 
  data[18].mAabb = Aabb(Vector3(10.2812f, -46.2921f, -3.86696f), Vector3(16.3243f, -43.3732f, 2.45959f)); 
  data[19].mAabb = Aabb(Vector3(24.3321f, -8.33962f, -20.6014f), Vector3(29.8042f, -4.1177f, -14.6737f)); 
  data[20].mAabb = Aabb(Vector3(-20.1789f, 10.0127f, -19.2033f), Vector3(-12.9807f, 12.9258f, -16.719f)); 
  data[21].mAabb = Aabb(Vector3(21.354f, -34.9341f, 16.7726f), Vector3(26.9645f, -33.8592f, 17.9956f)); 
  data[22].mAabb = Aabb(Vector3(27.1132f, 11.8809f, 13.4774f), Vector3(33.276f, 15.7313f, 20.7102f)); 
  data[23].mAabb = Aabb(Vector3(9.90574f, -6.38997f, 34.1412f), Vector3(17.2512f, -4.50392f, 38.3826f)); 
  data[24].mAabb = Aabb(Vector3(-32.7973f, 27.1168f, 17.0617f), Vector3(-27.193f, 33.8973f, 18.7314f)); 
  data[25].mAabb = Aabb(Vector3(-7.96201f, 29.0116f, 30.1709f), Vector3(-5.10251f, 30.0791f, 31.2301f)); 
  data[26].mAabb = Aabb(Vector3(16.8f, -35.3039f, -10.2765f), Vector3(17.9601f, -28.4771f, -5.35848f)); 
  data[27].mAabb = Aabb(Vector3(24.5078f, 6.79432f, -25.9387f), Vector3(27.2652f, 10.1722f, -25.2511f)); 
  data[28].mAabb = Aabb(Vector3(-2.80842f, 5.31982f, -8.96554f), Vector3(3.40981f, 6.81458f, -2.51305f)); 
  data[29].mAabb = Aabb(Vector3(-36.3719f, -16.3486f, -22.7786f), Vector3(-28.8673f, -14.096f, -18.6499f)); 
  data[30].mAabb = Aabb(Vector3(-4.32339f, 17.533f, -6.98752f), Vector3(-0.16665f, 21.3372f, -1.43472f)); 
  data[31].mAabb = Aabb(Vector3(10.5552f, -43.5413f, 6.44612f), Vector3(17.936f, -41.7517f, 8.51331f)); 
  data[32].mAabb = Aabb(Vector3(1.38243f, 37.3907f, 10.4311f), Vector3(2.0872f, 43.2473f, 12.2789f)); 
  data[33].mAabb = Aabb(Vector3(-20.6268f, -9.9624f, -17.2736f), Vector3(-17.5127f, -2.85699f, -10.617f)); 
  data[34].mAabb = Aabb(Vector3(14.121f, -16.1074f, 28.0875f), Vector3(17.4066f, -14.2526f, 33.1984f)); 
  data[35].mAabb = Aabb(Vector3(-9.97487f, 23.3001f, -23.9968f), Vector3(-4.8231f, 26.0679f, -18.9311f)); 
  data[36].mAabb = Aabb(Vector3(33.8184f, -19.15f, -5.27131f), Vector3(35.0255f, -18.1685f, 1.80023f)); 
  data[37].mAabb = Aabb(Vector3(25.1885f, -10.1764f, 22.738f), Vector3(28.9215f, -6.55032f, 30.1822f)); 
  data[38].mAabb = Aabb(Vector3(-30.8005f, 29.2383f, -10.9171f), Vector3(-29.2394f, 31.5926f, -4.71464f)); 
  data[39].mAabb = Aabb(Vector3(-5.97757f, -2.37879f, -35.8278f), Vector3(-1.4474f, 5.36306f, -34.8542f)); 
  data[40].mAabb = Aabb(Vector3(-21.1112f, 19.1234f, 34.8093f), Vector3(-16.4194f, 25.9894f, 40.2577f)); 
  data[41].mAabb = Aabb(Vector3(-6.07814f, 3.71625f, 19.1954f), Vector3(-5.66492f, 7.85745f, 24.6599f)); 
  data[42].mAabb = Aabb(Vector3(0.760995f, 32.5528f, 30.521f), Vector3(5.26124f, 40.2766f, 35.977f)); 
  data[43].mAabb = Aabb(Vector3(-0.467346f, -40.8543f, -22.7424f), Vector3(4.0394f, -38.9569f, -18.5843f)); 
  data[44].mAabb = Aabb(Vector3(42.5053f, 10.1207f, 14.4061f), Vector3(46.3487f, 14.2218f, 18.8755f)); 
  data[45].mAabb = Aabb(Vector3(16.376f, -9.44096f, -12.1574f), Vector3(23.874f, -4.18667f, -6.15051f)); 
  data[46].mAabb = Aabb(Vector3(14.7312f, -15.4908f, 18.036f), Vector3(15.8231f, -12.9759f, 22.6884f)); 
  data[47].mAabb = Aabb(Vector3(15.8977f, -30.4949f, 0.627857f), Vector3(23.0579f, -22.7881f, 6.10875f)); 
  data[48].mAabb = Aabb(Vector3(-6.01764f, 3.06048f, -36.0614f), Vector3(0.492832f, 7.79685f, -30.4608f)); 
  data[49].mAabb = Aabb(Vector3(-14.5418f, -9.37132f, 2.54704f), Vector3(-11.0883f, -3.62277f, 7.52137f)); 
  data[50].mAabb = Aabb(Vector3(-16.326f, 0.837473f, -12.0959f), Vector3(-9.80759f, 8.49095f, -4.48953f)); 
  data[51].mAabb = Aabb(Vector3(-16.7008f, 0.279909f, -28.6297f), Vector3(-15.9357f, 6.68184f, -25.9592f)); 
  data[52].mAabb = Aabb(Vector3(-49.1228f, -17.6986f, -3.49686f), Vector3(-46.2222f, -11.4047f, 1.94205f)); 
  data[53].mAabb = Aabb(Vector3(17.8819f, -36.7746f, -13.0997f), Vector3(19.7347f, -30.3364f, -8.57353f)); 
  data[54].mAabb = Aabb(Vector3(6.47273f, 43.9936f, -3.65259f), Vector3(13.497f, 46.6174f, -0.852239f)); 
  data[55].mAabb = Aabb(Vector3(0.984566f, 9.45842f, -10.3645f), Vector3(5.63581f, 10.4624f, -9.12275f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(0.677267f, 27.2063f, 22.6753f), Vector3(-10.7389f, 4.38277f, -27.6049f), Vector3(-10.7389f, -48.5209f, -3.59063f), Vector3(0.677267f, -25.6974f, 46.6896f), Vector3(32.247f, -7.6471f, 4.26938f), Vector3(30.6014f, -10.9371f, -2.97841f), Vector3(30.6014f, -19.8981f, 1.08924f), Vector3(32.247f, -16.6082f, 8.33703f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(59); 
  data[0].mAabb = Aabb(Vector3(8.26107f, -23.4078f, 39.2374f), Vector3(9.89383f, -18.8702f, 43.65f)); 
  data[1].mAabb = Aabb(Vector3(-11.327f, 46.4675f, 11.7284f), Vector3(-8.55703f, 47.4766f, 12.226f)); 
  data[2].mAabb = Aabb(Vector3(-7.09373f, 13.6769f, 14.6575f), Vector3(-2.2247f, 14.9761f, 20.8187f)); 
  data[3].mAabb = Aabb(Vector3(47.1723f, 4.63069f, -2.22958f), Vector3(48.1333f, 11.7846f, -1.25831f)); 
  data[4].mAabb = Aabb(Vector3(6.01099f, 7.14224f, -26.3256f), Vector3(11.0633f, 8.58296f, -19.5317f)); 
  data[5].mAabb = Aabb(Vector3(-2.76606f, -7.12838f, 4.56099f), Vector3(2.87185f, -2.67893f, 5.72013f)); 
  data[6].mAabb = Aabb(Vector3(-15.988f, -16.4285f, -45.7407f), Vector3(-14.1031f, -15.2517f, -40.636f)); 
  data[7].mAabb = Aabb(Vector3(-14.5532f, -12.1744f, -25.9432f), Vector3(-8.1974f, -9.69579f, -22.5818f)); 
  data[8].mAabb = Aabb(Vector3(-5.42719f, -44.1622f, 9.00759f), Vector3(-0.923932f, -42.6727f, 16.9779f)); 
  data[9].mAabb = Aabb(Vector3(-17.0934f, -3.32948f, -34.5452f), Vector3(-13.5861f, -0.0193183f, -31.5135f)); 
  data[10].mAabb = Aabb(Vector3(16.3914f, -14.9183f, 24.2879f), Vector3(23.6779f, -7.36222f, 25.0043f)); 
  data[11].mAabb = Aabb(Vector3(-23.6979f, 7.40772f, 32.2581f), Vector3(-17.8285f, 11.5171f, 35.0499f)); 
  data[12].mAabb = Aabb(Vector3(-35.4367f, -11.7251f, -9.77554f), Vector3(-31.3426f, -7.64118f, -4.8142f)); 
  data[13].mAabb = Aabb(Vector3(16.9714f, 16.6299f, -10.2558f), Vector3(24.5305f, 17.3449f, -9.50298f)); 
  data[14].mAabb = Aabb(Vector3(-48.6181f, -3.46836f, -4.95217f), Vector3(-44.9714f, -0.248423f, -2.61013f)); 
  data[15].mAabb = Aabb(Vector3(19.4007f, 22.2019f, 31.1791f), Vector3(27.2174f, 29.3451f, 38.233f)); 
  data[16].mAabb = Aabb(Vector3(-29.8069f, -18.7081f, -32.8422f), Vector3(-24.8444f, -11.6189f, -24.899f)); 
  data[17].mAabb = Aabb(Vector3(13.9043f, 40.2584f, 16.6128f), Vector3(17.9293f, 43.6389f, 18.5436f)); 
  data[18].mAabb = Aabb(Vector3(-40.2394f, -7.52553f, 5.26181f), Vector3(-38.2782f, -3.09278f, 8.69652f)); 
  data[19].mAabb = Aabb(Vector3(-5.20338f, -28.2921f, -22.3219f), Vector3(0.973334f, -27.0618f, -19.0929f)); 
  data[20].mAabb = Aabb(Vector3(-29.1909f, 40.8133f, -2.83683f), Vector3(-22.6405f, 43.8175f, 3.30022f)); 
  data[21].mAabb = Aabb(Vector3(-1.26652f, -50.1617f, 0.369333f), Vector3(1.93765f, -46.3879f, 5.04029f)); 
  data[22].mAabb = Aabb(Vector3(-4.39559f, -8.20751f, 42.4898f), Vector3(2.90534f, -3.82253f, 49.3088f)); 
  data[23].mAabb = Aabb(Vector3(18.9389f, -34.2844f, 5.56356f), Vector3(24.3228f, -31.677f, 11.5547f)); 
  data[24].mAabb = Aabb(Vector3(10.3882f, 16.7909f, 32.2317f), Vector3(15.42f, 23.2756f, 39.0801f)); 
  data[25].mAabb = Aabb(Vector3(-9.28577f, 5.53489f, 18.7708f), Vector3(-7.25963f, 6.7562f, 21.4234f)); 
  data[26].mAabb = Aabb(Vector3(-39.9129f, -0.333883f, -5.03573f), Vector3(-35.5015f, 0.854483f, -3.53262f)); 
  data[27].mAabb = Aabb(Vector3(35.0337f, -2.50621f, 6.06554f), Vector3(39.2997f, 0.973729f, 10.322f)); 
  data[28].mAabb = Aabb(Vector3(2.33346f, -42.5881f, 10.1389f), Vector3(6.42201f, -35.7773f, 13.3951f)); 
  data[29].mAabb = Aabb(Vector3(12.1671f, -38.4886f, 1.59311f), Vector3(16.7316f, -37.3228f, 4.1483f)); 
  data[30].mAabb = Aabb(Vector3(30.705f, -30.6634f, -17.6144f), Vector3(31.9724f, -26.6412f, -12.5679f)); 
  data[31].mAabb = Aabb(Vector3(4.65858f, 23.6489f, 19.4132f), Vector3(12.1926f, 29.6036f, 23.2f)); 
  data[32].mAabb = Aabb(Vector3(21.797f, -17.0084f, -37.4313f), Vector3(29.4349f, -12.0995f, -29.6795f)); 
  data[33].mAabb = Aabb(Vector3(-4.09599f, 16.9765f, 28.3992f), Vector3(-2.39481f, 24.7513f, 29.0346f)); 
  data[34].mAabb = Aabb(Vector3(7.16919f, -13.5488f, 39.7351f), Vector3(8.54171f, -11.3211f, 44.0053f)); 
  data[35].mAabb = Aabb(Vector3(-32.471f, -7.22556f, 6.07337f), Vector3(-28.4513f, -3.50231f, 12.2132f)); 
  data[36].mAabb = Aabb(Vector3(7.41217f, 12.9753f, -33.0043f), Vector3(9.51949f, 18.6968f, -26.5761f)); 
  data[37].mAabb = Aabb(Vector3(-30.6837f, 21.8004f, 21.8011f), Vector3(-29.542f, 24.097f, 23.8539f)); 
  data[38].mAabb = Aabb(Vector3(-36.2927f, -19.5695f, -6.38575f), Vector3(-32.6817f, -15.8161f, 0.737746f)); 
  data[39].mAabb = Aabb(Vector3(18.0863f, -23.3008f, 23.5849f), Vector3(20.9535f, -20.6057f, 31.157f)); 
  data[40].mAabb = Aabb(Vector3(6.92017f, -27.7702f, -25.1209f), Vector3(11.1831f, -23.8431f, -23.2024f)); 
  data[41].mAabb = Aabb(Vector3(31.6065f, -31.815f, 10.1083f), Vector3(32.9839f, -25.8438f, 17.0563f)); 
  data[42].mAabb = Aabb(Vector3(34.4872f, -2.49034f, -26.3765f), Vector3(41.3363f, 3.66086f, -25.8807f)); 
  data[43].mAabb = Aabb(Vector3(6.20882f, -35.734f, -6.20406f), Vector3(12.2784f, -31.1553f, -3.51573f)); 
  data[44].mAabb = Aabb(Vector3(6.78478f, -20.9166f, -8.82997f), Vector3(14.1699f, -19.9045f, -1.04359f)); 
  data[45].mAabb = Aabb(Vector3(-18.917f, -2.87859f, -7.8504f), Vector3(-18.1164f, 4.2071f, -6.5841f)); 
  data[46].mAabb = Aabb(Vector3(-13.3573f, 16.5437f, -37.4363f), Vector3(-9.2699f, 23.0646f, -33.4524f)); 
  data[47].mAabb = Aabb(Vector3(-40.1105f, -22.5952f, -7.25267f), Vector3(-39.1515f, -15.9787f, -6.80697f)); 
  data[48].mAabb = Aabb(Vector3(8.73573f, 10.6923f, 40.4837f), Vector3(9.33056f, 11.1813f, 42.5467f)); 
  data[49].mAabb = Aabb(Vector3(35.1481f, 21.5721f, 14.0435f), Vector3(39.1167f, 24.5545f, 19.5821f)); 
  data[50].mAabb = Aabb(Vector3(5.53535f, -30.829f, -40.8094f), Vector3(9.69766f, -26.0232f, -34.0067f)); 
  data[51].mAabb = Aabb(Vector3(-3.86112f, 15.9407f, -27.1194f), Vector3(-0.815143f, 18.2067f, -20.2155f)); 
  data[52].mAabb = Aabb(Vector3(3.58428f, -10.3996f, 14.7278f), Vector3(4.11532f, -6.33148f, 21.8664f)); 
  data[53].mAabb = Aabb(Vector3(12.566f, -33.5456f, -8.46383f), Vector3(14.783f, -28.5277f, -1.66018f)); 
  data[54].mAabb = Aabb(Vector3(-18.3881f, 3.9693f, 31.0609f), Vector3(-13.3915f, 4.40711f, 31.7112f)); 
  data[55].mAabb = Aabb(Vector3(6.2729f, -2.17106f, 11.9184f), Vector3(11.0812f, 3.46292f, 17.0442f)); 
  data[56].mAabb = Aabb(Vector3(-49.6886f, -1.32812f, -15.7621f), Vector3(-45.512f, -0.150651f, -8.56327f)); 
  data[57].mAabb = Aabb(Vector3(-34.6075f, 13.6161f, -20.5955f), Vector3(-29.9825f, 20.0855f, -18.9082f)); 
  data[58].mAabb = Aabb(Vector3(2.65218f, 30.3604f, 28.6003f), Vector3(4.65443f, 35.4946f, 30.7239f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(23.037f, 143.492f, -19.5386f), Vector3(-91.5976f, 24.7801f, -115.934f), Vector3(-91.5976f, -96.698f, 33.6674f), Vector3(23.037f, 22.0144f, 130.063f), Vector3(36.0743f, -13.048f, -26.7709f), Vector3(33.0803f, -16.1485f, -29.2886f), Vector3(33.0803f, -20.2997f, -24.1764f), Vector3(36.0743f, -17.1992f, -21.6587f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(50); 
  data[0].mAabb = Aabb(Vector3(-1.18842f, 8.79933f, 44.5392f), Vector3(0.635468f, 14.1276f, 49.1325f)); 
  data[1].mAabb = Aabb(Vector3(2.21837f, -46.2009f, -2.39452f), Vector3(5.79595f, -44.9698f, 1.04529f)); 
  data[2].mAabb = Aabb(Vector3(-7.6648f, 16.0204f, 29.6015f), Vector3(-5.69665f, 22.1087f, 36.0415f)); 
  data[3].mAabb = Aabb(Vector3(26.8184f, -0.723394f, 20.0419f), Vector3(27.9263f, 3.73394f, 26.101f)); 
  data[4].mAabb = Aabb(Vector3(-17.3646f, 34.3458f, -21.7847f), Vector3(-13.366f, 35.5298f, -20.9519f)); 
  data[5].mAabb = Aabb(Vector3(-9.42309f, -32.9612f, -1.60328f), Vector3(-1.83734f, -26.084f, 3.82264f)); 
  data[6].mAabb = Aabb(Vector3(-5.73333f, 41.7446f, 11.8023f), Vector3(-0.579473f, 47.1432f, 15.5316f)); 
  data[7].mAabb = Aabb(Vector3(-34.5083f, 5.96257f, 26.5579f), Vector3(-34.0619f, 8.2798f, 28.8165f)); 
  data[8].mAabb = Aabb(Vector3(-10.3297f, 17.4795f, -11.4917f), Vector3(-6.73636f, 24.0635f, -9.51564f)); 
  data[9].mAabb = Aabb(Vector3(-18.7899f, -11.0457f, -17.755f), Vector3(-13.0177f, -5.11874f, -16.2227f)); 
  data[10].mAabb = Aabb(Vector3(-7.46882f, 1.65571f, 28.7448f), Vector3(-4.16446f, 4.85245f, 31.8991f)); 
  data[11].mAabb = Aabb(Vector3(-8.71482f, -13.7817f, -48.2812f), Vector3(-3.62985f, -7.74109f, -40.5473f)); 
  data[12].mAabb = Aabb(Vector3(7.46285f, -46.1055f, -22.4939f), Vector3(13.5391f, -41.486f, -15.4305f)); 
  data[13].mAabb = Aabb(Vector3(-18.3967f, -14.8478f, 21.3391f), Vector3(-17.8262f, -11.6857f, 23.3045f)); 
  data[14].mAabb = Aabb(Vector3(3.12572f, -11.626f, -11.8988f), Vector3(10.5364f, -7.14381f, -6.89175f)); 
  data[15].mAabb = Aabb(Vector3(-6.34024f, -4.81107f, -38.3715f), Vector3(0.351384f, -1.03285f, -37.0157f)); 
  data[16].mAabb = Aabb(Vector3(9.2156f, -20.9983f, -42.3603f), Vector3(11.0427f, -19.2034f, -39.5148f)); 
  data[17].mAabb = Aabb(Vector3(21.5115f, 15.1975f, 18.0174f), Vector3(27.4609f, 22.9094f, 19.0594f)); 
  data[18].mAabb = Aabb(Vector3(-29.8388f, 5.95776f, 35.0621f), Vector3(-23.0101f, 10.0632f, 41.8365f)); 
  data[19].mAabb = Aabb(Vector3(-15.4968f, -36.9779f, -25.5942f), Vector3(-13.9297f, -35.8421f, -20.0084f)); 
  data[20].mAabb = Aabb(Vector3(25.5663f, 1.89486f, -26.6172f), Vector3(30.151f, 5.78696f, -20.8255f)); 
  data[21].mAabb = Aabb(Vector3(11.1456f, 39.5846f, -5.47289f), Vector3(18.3897f, 41.1361f, -2.18825f)); 
  data[22].mAabb = Aabb(Vector3(1.04984f, 24.8359f, -28.9299f), Vector3(1.65163f, 31.6437f, -25.1088f)); 
  data[23].mAabb = Aabb(Vector3(5.05171f, 6.10424f, 40.4798f), Vector3(13.0049f, 13.353f, 47.1239f)); 
  data[24].mAabb = Aabb(Vector3(31.7903f, 24.313f, 6.76925f), Vector3(39.0293f, 28.4972f, 13.0155f)); 
  data[25].mAabb = Aabb(Vector3(14.0811f, 23.5006f, 27.8657f), Vector3(19.6211f, 27.4583f, 32.7329f)); 
  data[26].mAabb = Aabb(Vector3(-5.84892f, 18.6054f, -12.775f), Vector3(1.08114f, 25.0156f, -6.55078f)); 
  data[27].mAabb = Aabb(Vector3(-12.7325f, 21.9798f, -41.7508f), Vector3(-12.0451f, 24.7885f, -36.9189f)); 
  data[28].mAabb = Aabb(Vector3(17.6413f, 22.1144f, -22.8968f), Vector3(24.6205f, 28.9139f, -18.1585f)); 
  data[29].mAabb = Aabb(Vector3(-14.8599f, -0.15212f, -16.2324f), Vector3(-12.654f, 3.75482f, -9.82192f)); 
  data[30].mAabb = Aabb(Vector3(13.13f, 19.3063f, -20.4963f), Vector3(18.3198f, 27.0588f, -17.143f)); 
  data[31].mAabb = Aabb(Vector3(-31.8841f, -21.0155f, -29.4036f), Vector3(-25.469f, -16.9749f, -24.4028f)); 
  data[32].mAabb = Aabb(Vector3(-35.8541f, 18.8042f, 4.60846f), Vector3(-34.8663f, 20.44f, 8.24542f)); 
  data[33].mAabb = Aabb(Vector3(-15.6945f, -0.224614f, -50.2474f), Vector3(-8.51789f, 3.40446f, -46.3205f)); 
  data[34].mAabb = Aabb(Vector3(-7.57566f, 36.2622f, 5.56181f), Vector3(-7.07013f, 44.0439f, 10.9676f)); 
  data[35].mAabb = Aabb(Vector3(-24.7567f, -25.5766f, 26.7773f), Vector3(-22.6137f, -23.3473f, 34.7507f)); 
  data[36].mAabb = Aabb(Vector3(32.5806f, -23.9982f, 7.7081f), Vector3(38.4667f, -19.173f, 15.071f)); 
  data[37].mAabb = Aabb(Vector3(-38.6885f, -20.4444f, 21.3166f), Vector3(-36.1043f, -14.3363f, 24.8327f)); 
  data[38].mAabb = Aabb(Vector3(-39.1887f, -13.813f, -17.3747f), Vector3(-33.519f, -9.62097f, -15.0869f)); 
  data[39].mAabb = Aabb(Vector3(-36.132f, -24.9111f, 24.6151f), Vector3(-31.3813f, -18.1154f, 27.528f)); 
  data[40].mAabb = Aabb(Vector3(-15.8833f, 9.91073f, 17.4434f), Vector3(-13.7541f, 15.0572f, 18.185f)); 
  data[41].mAabb = Aabb(Vector3(24.0121f, 30.463f, 17.0707f), Vector3(30.1559f, 37.7964f, 23.6819f)); 
  data[42].mAabb = Aabb(Vector3(11.9947f, -21.5445f, 38.272f), Vector3(13.2065f, -20.3309f, 45.3204f)); 
  data[43].mAabb = Aabb(Vector3(12.1959f, -41.6858f, -10.1962f), Vector3(14.0421f, -39.7621f, -3.95155f)); 
  data[44].mAabb = Aabb(Vector3(22.4041f, 23.8035f, -33.0121f), Vector3(23.3563f, 30.1938f, -28.6963f)); 
  data[45].mAabb = Aabb(Vector3(20.8528f, 14.23f, 22.816f), Vector3(27.1365f, 15.1322f, 26.5877f)); 
  data[46].mAabb = Aabb(Vector3(40.9996f, -0.787591f, -4.95199f), Vector3(43.8202f, 4.16424f, -3.49086f)); 
  data[47].mAabb = Aabb(Vector3(27.2015f, 7.71451f, -7.89327f), Vector3(32.3755f, 14.4511f, -5.16807f)); 
  data[48].mAabb = Aabb(Vector3(-13.1556f, 14.3289f, -6.87239f), Vector3(-7.29081f, 20.4026f, -5.47319f)); 
  data[49].mAabb = Aabb(Vector3(24.6986f, -16.6401f, -30.2811f), Vector3(29.328f, -14.0694f, -22.8496f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(19.1252f, 34.6841f, 31.4298f), Vector3(-48.3534f, 34.6453f, -26.4962f), Vector3(-48.3017f, -55.2168f, -26.4962f), Vector3(19.1769f, -55.178f, 31.4298f), Vector3(12.9761f, -6.29695f, -24.2906f), Vector3(7.68266f, -6.29999f, -28.8347f), Vector3(7.68721f, -14.207f, -28.8347f), Vector3(12.9806f, -14.204f, -24.2906f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(0.72745f, 42.452f, -0.658959f), Vector3(6.99555f, 48.6066f, 5.45977f)); 
  data[1].mAabb = Aabb(Vector3(22.64f, -15.8574f, 33.7395f), Vector3(23.8694f, -8.54074f, 39.7515f)); 
  data[2].mAabb = Aabb(Vector3(-22.5795f, -10.9861f, -39.7057f), Vector3(-17.4298f, -7.28447f, -34.1724f)); 
  data[3].mAabb = Aabb(Vector3(-20.3863f, 11.131f, -14.6219f), Vector3(-16.0498f, 13.1652f, -7.3295f)); 
  data[4].mAabb = Aabb(Vector3(-46.5783f, 4.06197f, -13.2048f), Vector3(-44.3005f, 11.0083f, -8.37315f)); 
  data[5].mAabb = Aabb(Vector3(-38.7768f, -17.2952f, 14.2074f), Vector3(-34.8585f, -12.8267f, 16.9984f)); 
  data[6].mAabb = Aabb(Vector3(-11.2354f, -0.959505f, -9.02291f), Vector3(-9.19557f, 0.234428f, -5.72829f)); 
  data[7].mAabb = Aabb(Vector3(-40.9286f, 11.8573f, 12.1479f), Vector3(-38.3231f, 18.5051f, 17.4035f)); 
  data[8].mAabb = Aabb(Vector3(-10.9387f, 7.03529f, 1.58318f), Vector3(-7.85587f, 9.12753f, 8.19432f)); 
  data[9].mAabb = Aabb(Vector3(21.4723f, -21.8094f, -38.6109f), Vector3(25.0996f, -17.9546f, -36.0796f)); 
  data[10].mAabb = Aabb(Vector3(-20.7535f, -39.0386f, 8.24174f), Vector3(-15.6827f, -31.227f, 10.4293f)); 
  data[11].mAabb = Aabb(Vector3(-28.1789f, 25.2496f, 24.7384f), Vector3(-22.2423f, 29.7454f, 27.7396f)); 
  data[12].mAabb = Aabb(Vector3(-12.464f, -0.507473f, -40.3687f), Vector3(-5.38061f, 6.18716f, -36.7215f)); 
  data[13].mAabb = Aabb(Vector3(39.1161f, 5.66164f, 23.7372f), Vector3(41.3779f, 6.46034f, 24.6817f)); 
  data[14].mAabb = Aabb(Vector3(-19.4359f, -5.27737f, 12.8012f), Vector3(-17.2638f, -1.2359f, 20.3245f)); 
  data[15].mAabb = Aabb(Vector3(-12.4567f, -14.3692f, 17.0287f), Vector3(-10.7627f, -11.6693f, 20.1524f)); 
  data[16].mAabb = Aabb(Vector3(25.2156f, 20.4997f, -31.2653f), Vector3(31.9988f, 25.5912f, -25.8964f)); 
  data[17].mAabb = Aabb(Vector3(-6.03117f, -33.6647f, 25.7022f), Vector3(-2.60736f, -28.1808f, 29.0175f)); 
  data[18].mAabb = Aabb(Vector3(-12.1075f, 5.62751f, 39.1541f), Vector3(-4.14717f, 7.0884f, 39.8426f)); 
  data[19].mAabb = Aabb(Vector3(-17.9796f, -6.30397f, -44.0753f), Vector3(-13.9398f, -5.1657f, -43.579f)); 
  data[20].mAabb = Aabb(Vector3(11.9852f, 8.77134f, -21.6723f), Vector3(15.8277f, 13.276f, -14.9654f)); 
  data[21].mAabb = Aabb(Vector3(23.632f, 27.9772f, 10.7091f), Vector3(26.6332f, 30.7625f, 11.3086f)); 
  data[22].mAabb = Aabb(Vector3(-35.5791f, 31.0536f, -4.50423f), Vector3(-30.4143f, 36.7128f, 0.282463f)); 
  data[23].mAabb = Aabb(Vector3(17.5187f, 17.8799f, -44.7465f), Vector3(22.6162f, 20.0339f, -38.3994f)); 
  data[24].mAabb = Aabb(Vector3(42.1343f, -12.3499f, -13.0995f), Vector3(48.2131f, -6.31629f, -6.39075f)); 
  data[25].mAabb = Aabb(Vector3(-19.8132f, -21.7877f, 3.69697f), Vector3(-12.5062f, -17.5342f, 7.17575f)); 
  data[26].mAabb = Aabb(Vector3(-32.622f, 33.03f, -18.0735f), Vector3(-27.8186f, 35.9373f, -15.048f)); 
  data[27].mAabb = Aabb(Vector3(15.0185f, -11.7967f, 15.4351f), Vector3(21.885f, -5.78371f, 16.521f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-22.384f, 64.2386f, -20.3133f), Vector3(-25.9984f, -17.7488f, -53.7105f), Vector3(-25.9984f, -52.7459f, 32.2044f), Vector3(-22.384f, 29.2414f, 65.6017f), Vector3(44.0888f, 6.85639f, 1.74206f), Vector3(43.9071f, 2.73295f, 0.0623887f), Vector3(43.9071f, -0.520198f, 8.0486f), Vector3(44.0888f, 3.60324f, 9.72827f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(65); 
  data[0].mAabb = Aabb(Vector3(-40.8464f, 22.0326f, 3.62193f), Vector3(-34.9656f, 26.4464f, 7.98301f)); 
  data[1].mAabb = Aabb(Vector3(33.5739f, -2.59726f, -0.138534f), Vector3(38.3831f, 5.39276f, 7.72045f)); 
  data[2].mAabb = Aabb(Vector3(-26.1107f, -15.598f, 17.6179f), Vector3(-23.9802f, -12.299f, 22.0875f)); 
  data[3].mAabb = Aabb(Vector3(4.51774f, -37.0332f, 15.4119f), Vector3(8.18045f, -33.0889f, 23.3509f)); 
  data[4].mAabb = Aabb(Vector3(-40.4009f, 13.5832f, 15.506f), Vector3(-33.4166f, 16.2175f, 22.8366f)); 
  data[5].mAabb = Aabb(Vector3(0.730722f, 27.7182f, -8.50896f), Vector3(5.54131f, 33.8718f, -8.07996f)); 
  data[6].mAabb = Aabb(Vector3(17.7408f, -26.5257f, 27.3396f), Vector3(22.1933f, -22.2606f, 32.5178f)); 
  data[7].mAabb = Aabb(Vector3(-2.26896f, -12.9717f, -1.01339f), Vector3(-0.46804f, -10.8426f, -0.489994f)); 
  data[8].mAabb = Aabb(Vector3(2.90759f, 18.0627f, 33.3925f), Vector3(9.31833f, 25.0751f, 34.1949f)); 
  data[9].mAabb = Aabb(Vector3(-8.25281f, 16.8612f, -37.1058f), Vector3(-6.49294f, 20.4088f, -30.0713f)); 
  data[10].mAabb = Aabb(Vector3(33.5511f, 4.61853f, -34.1529f), Vector3(38.7727f, 11.8779f, -29.3097f)); 
  data[11].mAabb = Aabb(Vector3(-16.555f, 4.85862f, 5.77683f), Vector3(-8.96665f, 8.564f, 13.2482f)); 
  data[12].mAabb = Aabb(Vector3(6.88401f, 13.8316f, -20.9211f), Vector3(8.2454f, 16.3503f, -19.5877f)); 
  data[13].mAabb = Aabb(Vector3(-44.1478f, 2.77677f, 16.3853f), Vector3(-39.401f, 7.59338f, 17.9858f)); 
  data[14].mAabb = Aabb(Vector3(8.57957f, 16.1094f, -23.3607f), Vector3(16.545f, 19.5597f, -21.4573f)); 
  data[15].mAabb = Aabb(Vector3(-26.9625f, 18.2151f, -12.2258f), Vector3(-26.1221f, 19.127f, -6.02704f)); 
  data[16].mAabb = Aabb(Vector3(-20.8738f, 7.27266f, 35.2735f), Vector3(-14.4709f, 10.0542f, 36.5335f)); 
  data[17].mAabb = Aabb(Vector3(30.4718f, -21.4662f, 29.2078f), Vector3(33.2842f, -17.964f, 36.7182f)); 
  data[18].mAabb = Aabb(Vector3(-24.2698f, 8.24562f, 35.6193f), Vector3(-18.6741f, 13.6507f, 43.1721f)); 
  data[19].mAabb = Aabb(Vector3(2.53915f, -21.7958f, -12.3379f), Vector3(5.30332f, -18.4253f, -5.81424f)); 
  data[20].mAabb = Aabb(Vector3(11.4848f, -41.9742f, 7.80289f), Vector3(15.8772f, -34.6684f, 13.6405f)); 
  data[21].mAabb = Aabb(Vector3(-0.550796f, -41.9033f, 11.7299f), Vector3(1.06133f, -40.6514f, 13.7087f)); 
  data[22].mAabb = Aabb(Vector3(35.841f, -12.3256f, -20.4152f), Vector3(37.9504f, -7.54098f, -14.9603f)); 
  data[23].mAabb = Aabb(Vector3(-16.7359f, 19.7144f, -13.4124f), Vector3(-11.943f, 20.4828f, -10.3961f)); 
  data[24].mAabb = Aabb(Vector3(14.7218f, 39.5174f, 7.72654f), Vector3(15.6762f, 44.7608f, 14.4557f)); 
  data[25].mAabb = Aabb(Vector3(-10.8899f, 20.9164f, -6.46436f), Vector3(-6.85938f, 26.0927f, 1.11769f)); 
  data[26].mAabb = Aabb(Vector3(-13.4858f, -11.188f, 30.8525f), Vector3(-12.4454f, -4.20015f, 38.2597f)); 
  data[27].mAabb = Aabb(Vector3(20.2153f, -42.3129f, 8.71699f), Vector3(21.1931f, -39.821f, 9.21812f)); 
  data[28].mAabb = Aabb(Vector3(19.0659f, -12.0225f, 11.4265f), Vector3(26.7737f, -9.10736f, 17.2708f)); 
  data[29].mAabb = Aabb(Vector3(-15.0698f, 14.0851f, 29.3376f), Vector3(-11.925f, 17.4426f, 35.4689f)); 
  data[30].mAabb = Aabb(Vector3(18.7119f, -40.9976f, 8.5953f), Vector3(24.9735f, -35.0489f, 15.9491f)); 
  data[31].mAabb = Aabb(Vector3(1.29976f, 13.6544f, 0.548266f), Vector3(3.62556f, 15.8628f, 6.85695f)); 
  data[32].mAabb = Aabb(Vector3(-11.6029f, 13.6202f, -39.0319f), Vector3(-4.16071f, 20.2993f, -33.1029f)); 
  data[33].mAabb = Aabb(Vector3(-2.65522f, -2.03271f, -49.591f), Vector3(0.0839051f, 0.0739128f, -47.8903f)); 
  data[34].mAabb = Aabb(Vector3(43.7194f, 0.575437f, 12.6486f), Vector3(48.4551f, 6.05517f, 13.6004f)); 
  data[35].mAabb = Aabb(Vector3(-8.46f, 6.87514f, -45.0737f), Vector3(-5.77051f, 11.075f, -40.5975f)); 
  data[36].mAabb = Aabb(Vector3(-11.3821f, 26.351f, -19.5783f), Vector3(-3.73812f, 33.1027f, -13.6746f)); 
  data[37].mAabb = Aabb(Vector3(17.4198f, 40.959f, -18.6714f), Vector3(21.989f, 44.3309f, -13.8668f)); 
  data[38].mAabb = Aabb(Vector3(-23.2553f, 7.62823f, -21.7205f), Vector3(-16.0379f, 11.197f, -19.4373f)); 
  data[39].mAabb = Aabb(Vector3(-22.7496f, 4.67262f, -0.764931f), Vector3(-20.1439f, 9.71723f, 2.33579f)); 
  data[40].mAabb = Aabb(Vector3(7.34975f, 18.2793f, 20.0301f), Vector3(14.9234f, 25.8577f, 20.9831f)); 
  data[41].mAabb = Aabb(Vector3(-24.8557f, 13.526f, -8.11698f), Vector3(-16.8805f, 19.4216f, -1.99408f)); 
  data[42].mAabb = Aabb(Vector3(-4.70922f, -29.9381f, -31.5841f), Vector3(-2.20459f, -29.5144f, -27.0641f)); 
  data[43].mAabb = Aabb(Vector3(21.5973f, -25.5389f, -12.7984f), Vector3(29.0819f, -18.8874f, -9.95907f)); 
  data[44].mAabb = Aabb(Vector3(-29.6742f, -28.999f, 8.77492f), Vector3(-29.1821f, -23.0463f, 11.3401f)); 
  data[45].mAabb = Aabb(Vector3(-27.6697f, -3.02186f, -14.7565f), Vector3(-21.0231f, 4.30133f, -14.1793f)); 
  data[46].mAabb = Aabb(Vector3(-26.5827f, 20.9126f, 29.0975f), Vector3(-18.9424f, 28.3805f, 35.2352f)); 
  data[47].mAabb = Aabb(Vector3(16.4126f, 8.52141f, -15.1093f), Vector3(24.1615f, 15.9629f, -14.5481f)); 
  data[48].mAabb = Aabb(Vector3(-34.4002f, -17.8751f, -14.1737f), Vector3(-30.963f, -17.3865f, -11.6542f)); 
  data[49].mAabb = Aabb(Vector3(-18.1568f, -20.5493f, 16.3544f), Vector3(-12.82f, -13.2895f, 17.3143f)); 
  data[50].mAabb = Aabb(Vector3(7.47228f, 15.7374f, 21.5634f), Vector3(8.73533f, 19.822f, 25.4437f)); 
  data[51].mAabb = Aabb(Vector3(-29.6128f, -0.781534f, -30.8441f), Vector3(-27.8907f, 1.2998f, -25.4945f)); 
  data[52].mAabb = Aabb(Vector3(-30.2952f, -4.94483f, -12.3292f), Vector3(-29.7454f, -2.90525f, -9.90161f)); 
  data[53].mAabb = Aabb(Vector3(6.63529f, -8.2711f, 12.2215f), Vector3(9.96099f, -4.59193f, 15.3294f)); 
  data[54].mAabb = Aabb(Vector3(12.8797f, 14.2541f, -37.4281f), Vector3(18.9908f, 15.9685f, -31.7547f)); 
  data[55].mAabb = Aabb(Vector3(-8.4887f, -43.6974f, -0.79464f), Vector3(-8.01888f, -38.7315f, 5.25729f)); 
  data[56].mAabb = Aabb(Vector3(-23.2834f, -35.1402f, -21.8112f), Vector3(-18.1319f, -28.0721f, -19.3906f)); 
  data[57].mAabb = Aabb(Vector3(-32.69f, -4.5646f, 35.6379f), Vector3(-27.2552f, 0.916293f, 37.068f)); 
  data[58].mAabb = Aabb(Vector3(29.4388f, 15.8183f, 24.3817f), Vector3(30.9092f, 18.3162f, 27.0707f)); 
  data[59].mAabb = Aabb(Vector3(-6.19094f, -41.6371f, -29.0923f), Vector3(0.795017f, -35.3854f, -24.7091f)); 
  data[60].mAabb = Aabb(Vector3(-32.2993f, 30.3989f, -22.072f), Vector3(-27.3296f, 32.766f, -19.8828f)); 
  data[61].mAabb = Aabb(Vector3(14.9262f, 18.8397f, 24.1007f), Vector3(22.3171f, 26.8379f, 25.6855f)); 
  data[62].mAabb = Aabb(Vector3(2.76318f, -8.84163f, 36.8664f), Vector3(10.3427f, -4.36829f, 43.8252f)); 
  data[63].mAabb = Aabb(Vector3(-5.88105f, 1.96593f, -22.2062f), Vector3(1.85245f, 6.80365f, -15.0298f)); 
  data[64].mAabb = Aabb(Vector3(-5.80199f, -35.2746f, 31.3271f), Vector3(-2.48673f, -34.144f, 38.0997f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-59.3582f, 50.8744f, -14.7028f), Vector3(-1.91346f, -22.0872f, -54.9891f), Vector3(-1.91346f, -71.0544f, 33.6943f), Vector3(-59.3582f, 1.9072f, 73.9806f), Vector3(37.2334f, 37.6656f, 30.8066f), Vector3(42.2133f, 31.3405f, 27.3141f), Vector3(42.2133f, 27.0601f, 35.0663f), Vector3(37.2334f, 33.3852f, 38.5587f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(36); 
  data[0].mAabb = Aabb(Vector3(-21.1772f, 18.5665f, 14.8895f), Vector3(-13.3333f, 25.5001f, 19.191f)); 
  data[1].mAabb = Aabb(Vector3(-8.07815f, -15.5366f, -1.68151f), Vector3(-2.69375f, -11.3274f, 2.292f)); 
  data[2].mAabb = Aabb(Vector3(-12.0521f, -32.2041f, -36.7296f), Vector3(-7.51031f, -24.4898f, -35.5522f)); 
  data[3].mAabb = Aabb(Vector3(-16.7353f, 17.9069f, 34.0255f), Vector3(-9.38588f, 21.3189f, 40.281f)); 
  data[4].mAabb = Aabb(Vector3(42.1997f, 18.9329f, -0.488195f), Vector3(47.3684f, 23.7363f, 1.69033f)); 
  data[5].mAabb = Aabb(Vector3(20.5249f, -34.2714f, -15.706f), Vector3(26.1077f, -32.0134f, -9.20454f)); 
  data[6].mAabb = Aabb(Vector3(-4.51209f, -6.8955f, 16.6168f), Vector3(2.56085f, -6.05714f, 20.562f)); 
  data[7].mAabb = Aabb(Vector3(-24.4269f, -18.4055f, -38.6335f), Vector3(-21.9397f, -17.9241f, -34.9545f)); 
  data[8].mAabb = Aabb(Vector3(20.111f, -40.6301f, -5.41837f), Vector3(22.7786f, -37.2835f, -2.42086f)); 
  data[9].mAabb = Aabb(Vector3(-33.6022f, 13.9944f, 23.4022f), Vector3(-26.5877f, 21.6767f, 31.2335f)); 
  data[10].mAabb = Aabb(Vector3(-7.31932f, -25.1362f, -9.96924f), Vector3(-0.524487f, -22.6327f, -5.46412f)); 
  data[11].mAabb = Aabb(Vector3(26.0108f, -16.3747f, -2.8635f), Vector3(30.1833f, -11.7299f, 4.10436f)); 
  data[12].mAabb = Aabb(Vector3(1.8759f, 3.61377f, -17.3854f), Vector3(2.49717f, 5.30568f, -15.5233f)); 
  data[13].mAabb = Aabb(Vector3(10.1922f, -26.1481f, -14.7188f), Vector3(16.9209f, -18.4578f, -9.85629f)); 
  data[14].mAabb = Aabb(Vector3(-7.25947f, -40.8675f, -16.5299f), Vector3(-4.52568f, -39.9939f, -9.90367f)); 
  data[15].mAabb = Aabb(Vector3(24.6679f, 39.2162f, -10.6622f), Vector3(31.1276f, 40.3846f, -4.639f)); 
  data[16].mAabb = Aabb(Vector3(-12.5059f, -34.9904f, 6.72117f), Vector3(-9.41678f, -31.2085f, 13.7053f)); 
  data[17].mAabb = Aabb(Vector3(-11.6526f, 10.5605f, 45.2248f), Vector3(-8.33294f, 11.5679f, 47.9205f)); 
  data[18].mAabb = Aabb(Vector3(5.84176f, -0.80139f, -18.7563f), Vector3(6.72907f, 1.35255f, -16.0483f)); 
  data[19].mAabb = Aabb(Vector3(14.188f, -15.9505f, 32.8676f), Vector3(20.0724f, -9.12022f, 40.2314f)); 
  data[20].mAabb = Aabb(Vector3(-18.8902f, 38.0031f, 15.1918f), Vector3(-15.2611f, 38.4147f, 17.557f)); 
  data[21].mAabb = Aabb(Vector3(-27.0376f, -31.7774f, 9.31856f), Vector3(-20.4031f, -25.0257f, 12.5234f)); 
  data[22].mAabb = Aabb(Vector3(-8.98585f, -19.7518f, 24.3383f), Vector3(-8.56869f, -16.1487f, 25.8776f)); 
  data[23].mAabb = Aabb(Vector3(26.1815f, -15.9051f, 18.6447f), Vector3(34.0257f, -10.7355f, 26.6173f)); 
  data[24].mAabb = Aabb(Vector3(12.1071f, 46.2944f, -4.0762f), Vector3(14.4758f, 47.9137f, -1.44424f)); 
  data[25].mAabb = Aabb(Vector3(21.939f, -26.1277f, 3.36035f), Vector3(24.7691f, -24.0682f, 5.51638f)); 
  data[26].mAabb = Aabb(Vector3(37.2447f, -20.9214f, -7.19329f), Vector3(40.8413f, -19.224f, -3.38933f)); 
  data[27].mAabb = Aabb(Vector3(10.7045f, 29.7105f, -8.88996f), Vector3(14.2366f, 30.2f, -8.39811f)); 
  data[28].mAabb = Aabb(Vector3(-24.5221f, 35.4839f, -6.21861f), Vector3(-23.2967f, 40.9075f, -3.80212f)); 
  data[29].mAabb = Aabb(Vector3(-13.0377f, -23.0278f, 3.78819f), Vector3(-9.05652f, -21.7642f, 6.26684f)); 
  data[30].mAabb = Aabb(Vector3(4.35428f, -10.1287f, 14.5072f), Vector3(8.25867f, -5.08273f, 15.8478f)); 
  data[31].mAabb = Aabb(Vector3(-16.8263f, 12.6423f, 21.488f), Vector3(-15.8063f, 13.2511f, 23.6561f)); 
  data[32].mAabb = Aabb(Vector3(-22.1467f, -26.6022f, -6.27792f), Vector3(-14.4705f, -24.8884f, -3.5875f)); 
  data[33].mAabb = Aabb(Vector3(-23.0264f, -6.06311f, -8.7892f), Vector3(-15.587f, -3.04473f, -3.10373f)); 
  data[34].mAabb = Aabb(Vector3(28.7328f, -2.11261f, -36.0793f), Vector3(34.2033f, 4.34012f, -28.3296f)); 
  data[35].mAabb = Aabb(Vector3(-14.4936f, 15.3806f, -2.15812f), Vector3(-10.4032f, 19.7428f, -1.01034f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(6.91085f, 21.7598f, 51.1422f), Vector3(-41.0284f, 16.5177f, 19.8325f), Vector3(-34.8847f, -39.666f, 19.8325f), Vector3(13.0545f, -34.4239f, 51.1422f), Vector3(12.3011f, -3.0063f, 0.748248f), Vector3(6.39251f, -3.65241f, -3.11076f), Vector3(7.05645f, -9.72419f, -3.11076f), Vector3(12.9651f, -9.07809f, 0.748248f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(37); 
  data[0].mAabb = Aabb(Vector3(25.0297f, -9.60798f, 24.161f), Vector3(32.2295f, -4.13567f, 29.4935f)); 
  data[1].mAabb = Aabb(Vector3(0.712916f, 38.1458f, 18.667f), Vector3(3.37782f, 45.3656f, 20.12f)); 
  data[2].mAabb = Aabb(Vector3(-15.2685f, -21.4994f, 20.5745f), Vector3(-12.8391f, -20.3604f, 21.0183f)); 
  data[3].mAabb = Aabb(Vector3(-1.09562f, -46.6394f, -20.5933f), Vector3(0.448311f, -42.7828f, -16.9739f)); 
  data[4].mAabb = Aabb(Vector3(-25.2556f, 23.2879f, 23.0225f), Vector3(-18.7366f, 31.1661f, 27.4968f)); 
  data[5].mAabb = Aabb(Vector3(-18.5554f, -36.14f, -27.4426f), Vector3(-14.7306f, -29.4289f, -19.9877f)); 
  data[6].mAabb = Aabb(Vector3(2.26604f, 26.276f, -41.5332f), Vector3(2.86922f, 28.4135f, -36.018f)); 
  data[7].mAabb = Aabb(Vector3(-13.7881f, 39.6728f, -19.2736f), Vector3(-10.2228f, 45.8908f, -13.2488f)); 
  data[8].mAabb = Aabb(Vector3(-17.9451f, -27.7962f, 5.34577f), Vector3(-16.6793f, -20.0518f, 9.30096f)); 
  data[9].mAabb = Aabb(Vector3(0.841635f, 28.3332f, 18.7795f), Vector3(2.09262f, 33.5102f, 23.5417f)); 
  data[10].mAabb = Aabb(Vector3(14.9225f, -16.6554f, -39.1199f), Vector3(19.8138f, -9.36168f, -35.3572f)); 
  data[11].mAabb = Aabb(Vector3(-25.9014f, -25.4999f, -1.50531f), Vector3(-25.2577f, -24.8441f, 4.89778f)); 
  data[12].mAabb = Aabb(Vector3(18.9432f, -34.4996f, 27.6686f), Vector3(21.4548f, -27.6398f, 35.0732f)); 
  data[13].mAabb = Aabb(Vector3(0.782874f, 4.7176f, 16.82f), Vector3(4.58753f, 6.49046f, 19.1126f)); 
  data[14].mAabb = Aabb(Vector3(37.337f, -18.8449f, -7.27781f), Vector3(38.5956f, -13.9561f, -5.46459f)); 
  data[15].mAabb = Aabb(Vector3(-3.28595f, 17.2257f, -19.2032f), Vector3(-0.183841f, 20.544f, -14.2349f)); 
  data[16].mAabb = Aabb(Vector3(-23.6854f, -15.0038f, 28.4533f), Vector3(-16.1512f, -7.46332f, 31.0711f)); 
  data[17].mAabb = Aabb(Vector3(11.3584f, -21.9983f, 30.5215f), Vector3(14.5806f, -15.8195f, 31.66f)); 
  data[18].mAabb = Aabb(Vector3(-14.2807f, 24.8521f, 19.3586f), Vector3(-7.47054f, 25.7874f, 26.6065f)); 
  data[19].mAabb = Aabb(Vector3(-0.720932f, 4.43608f, 8.56269f), Vector3(2.86686f, 8.96185f, 12.9289f)); 
  data[20].mAabb = Aabb(Vector3(0.156295f, -28.327f, -5.54366f), Vector3(1.72736f, -20.9507f, 2.17824f)); 
  data[21].mAabb = Aabb(Vector3(-5.40316f, -24.5259f, 17.9729f), Vector3(-0.799237f, -16.5931f, 20.9414f)); 
  data[22].mAabb = Aabb(Vector3(4.19533f, -15.7656f, 11.4354f), Vector3(9.71079f, -8.96613f, 16.3761f)); 
  data[23].mAabb = Aabb(Vector3(26.9769f, 9.21645f, 21.5806f), Vector3(33.1156f, 12.3963f, 26.8312f)); 
  data[24].mAabb = Aabb(Vector3(4.7359f, 17.6333f, -42.0954f), Vector3(7.0559f, 20.1591f, -41.6089f)); 
  data[25].mAabb = Aabb(Vector3(-33.2674f, -13.7469f, -2.56851f), Vector3(-32.4141f, -6.23304f, 5.40992f)); 
  data[26].mAabb = Aabb(Vector3(-37.6041f, 28.6381f, 7.76009f), Vector3(-34.9158f, 33.6062f, 14.3102f)); 
  data[27].mAabb = Aabb(Vector3(18.1591f, 2.29478f, 12.5058f), Vector3(24.9547f, 6.41465f, 19.1157f)); 
  data[28].mAabb = Aabb(Vector3(-39.2835f, -15.1612f, -7.21346f), Vector3(-36.4927f, -8.00061f, -2.82177f)); 
  data[29].mAabb = Aabb(Vector3(-20.0605f, -4.06277f, -38.438f), Vector3(-14.9951f, -1.28863f, -36.0447f)); 
  data[30].mAabb = Aabb(Vector3(-16.7379f, 10.0482f, 28.1876f), Vector3(-16.1273f, 17.0198f, 34.1426f)); 
  data[31].mAabb = Aabb(Vector3(-13.7278f, 2.17773f, -23.0617f), Vector3(-11.9596f, 3.40993f, -19.8172f)); 
  data[32].mAabb = Aabb(Vector3(15.813f, 31.4952f, 16.172f), Vector3(18.32f, 36.9573f, 20.8107f)); 
  data[33].mAabb = Aabb(Vector3(-5.41159f, -36.9468f, -4.95809f), Vector3(-1.34414f, -30.2234f, -1.03816f)); 
  data[34].mAabb = Aabb(Vector3(-4.0645f, 11.2161f, -25.3329f), Vector3(-2.3545f, 17.0698f, -17.4871f)); 
  data[35].mAabb = Aabb(Vector3(16.8931f, 20.4475f, 35.9872f), Vector3(24.8416f, 27.5543f, 39.1793f)); 
  data[36].mAabb = Aabb(Vector3(-14.7294f, 24.0881f, 35.0356f), Vector3(-9.27214f, 27.4539f, 40.7519f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-28.4714f, 34.3846f, 55.7754f), Vector3(-33.1354f, 39.8636f, -33.4615f), Vector3(-33.1354f, -51.0424f, -39.043f), Vector3(-28.4714f, -56.5214f, 50.1939f), Vector3(-6.32905f, -4.34962f, 10.6805f), Vector3(-6.67688f, -3.94101f, 4.02541f), Vector3(-6.67688f, -12.1528f, 3.52122f), Vector3(-6.32905f, -12.5615f, 10.1763f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(54); 
  data[0].mAabb = Aabb(Vector3(-49.3205f, 3.03323f, -16.3368f), Vector3(-46.2796f, 7.60607f, -8.6912f)); 
  data[1].mAabb = Aabb(Vector3(-19.1084f, -39.4049f, 6.31861f), Vector3(-12.8809f, -31.808f, 11.5761f)); 
  data[2].mAabb = Aabb(Vector3(3.73489f, -34.9666f, -2.02559f), Vector3(4.60689f, -29.6782f, -1.37463f)); 
  data[3].mAabb = Aabb(Vector3(17.5149f, -26.2736f, 36.8719f), Vector3(19.7718f, -18.4849f, 42.7348f)); 
  data[4].mAabb = Aabb(Vector3(-11.1616f, 43.2255f, -22.4696f), Vector3(-6.08501f, 44.4646f, -21.2502f)); 
  data[5].mAabb = Aabb(Vector3(-23.1774f, 32.9382f, -21.0253f), Vector3(-21.1826f, 37.2376f, -14.2082f)); 
  data[6].mAabb = Aabb(Vector3(-29.2316f, -0.695972f, -3.03791f), Vector3(-27.1188f, 6.67199f, 4.33701f)); 
  data[7].mAabb = Aabb(Vector3(-29.3181f, 5.61225f, -3.71416f), Vector3(-26.113f, 12.6532f, -0.739157f)); 
  data[8].mAabb = Aabb(Vector3(-34.2422f, -12.862f, 27.9599f), Vector3(-30.2427f, -8.59256f, 28.7382f)); 
  data[9].mAabb = Aabb(Vector3(0.0697272f, 22.9287f, 21.8517f), Vector3(5.82895f, 25.4872f, 25.8669f)); 
  data[10].mAabb = Aabb(Vector3(19.0175f, 24.4285f, -4.24284f), Vector3(21.9176f, 29.516f, 3.0502f)); 
  data[11].mAabb = Aabb(Vector3(28.397f, -23.0046f, -25.6307f), Vector3(32.123f, -15.7858f, -18.705f)); 
  data[12].mAabb = Aabb(Vector3(8.35856f, -39.745f, 5.4999f), Vector3(11.8631f, -36.7219f, 12.0064f)); 
  data[13].mAabb = Aabb(Vector3(36.9459f, -19.1162f, -19.3792f), Vector3(39.3554f, -11.5766f, -16.32f)); 
  data[14].mAabb = Aabb(Vector3(-30.5862f, 11.956f, -30.0832f), Vector3(-24.9189f, 19.2219f, -28.4952f)); 
  data[15].mAabb = Aabb(Vector3(-27.5694f, 3.64288f, -40.7841f), Vector3(-24.4649f, 8.62f, -34.5563f)); 
  data[16].mAabb = Aabb(Vector3(3.89555f, 15.7281f, -36.3868f), Vector3(7.46478f, 22.2907f, -34.1559f)); 
  data[17].mAabb = Aabb(Vector3(-48.1858f, -13.2625f, 8.76112f), Vector3(-47.2584f, -6.98209f, 9.64704f)); 
  data[18].mAabb = Aabb(Vector3(-4.80382f, 45.3036f, 2.316f), Vector3(-1.74369f, 52.7405f, 6.36327f)); 
  data[19].mAabb = Aabb(Vector3(14.5418f, -8.88429f, -36.4453f), Vector3(21.4315f, -6.95905f, -34.3659f)); 
  data[20].mAabb = Aabb(Vector3(-10.7076f, 28.1563f, 32.9295f), Vector3(-8.85799f, 29.6227f, 38.9097f)); 
  data[21].mAabb = Aabb(Vector3(41.445f, -13.8419f, -10.4548f), Vector3(44.215f, -12.0988f, -5.84534f)); 
  data[22].mAabb = Aabb(Vector3(4.6643f, -8.00365f, 20.6387f), Vector3(7.26982f, -6.96419f, 25.3309f)); 
  data[23].mAabb = Aabb(Vector3(16.9031f, -9.01732f, -12.5577f), Vector3(24.0573f, -5.96323f, -7.07309f)); 
  data[24].mAabb = Aabb(Vector3(27.8119f, 32.7081f, 6.92959f), Vector3(31.4361f, 38.3017f, 14.4214f)); 
  data[25].mAabb = Aabb(Vector3(37.726f, 2.42353f, -34.4681f), Vector3(39.3272f, 8.4409f, -28.1527f)); 
  data[26].mAabb = Aabb(Vector3(5.55285f, 9.89832f, -14.212f), Vector3(12.2858f, 17.2549f, -11.4474f)); 
  data[27].mAabb = Aabb(Vector3(-2.10774f, -22.1592f, -28.8151f), Vector3(-0.407485f, -17.446f, -23.8937f)); 
  data[28].mAabb = Aabb(Vector3(2.34234f, 3.45531f, -4.3161f), Vector3(4.63265f, 8.45284f, 1.15273f)); 
  data[29].mAabb = Aabb(Vector3(19.1711f, 9.24625f, -19.6842f), Vector3(25.9485f, 17.115f, -15.2783f)); 
  data[30].mAabb = Aabb(Vector3(29.6474f, 6.18717f, 15.5785f), Vector3(30.9912f, 11.1905f, 18.2358f)); 
  data[31].mAabb = Aabb(Vector3(-25.0196f, -27.5476f, -36.3755f), Vector3(-18.3996f, -25.0114f, -33.0104f)); 
  data[32].mAabb = Aabb(Vector3(-8.54281f, -20.4038f, 8.0243f), Vector3(-2.13369f, -18.4136f, 15.8142f)); 
  data[33].mAabb = Aabb(Vector3(-5.33118f, 17.6555f, 33.8758f), Vector3(1.64596f, 23.4984f, 36.9714f)); 
  data[34].mAabb = Aabb(Vector3(19.018f, 21.8859f, -20.8877f), Vector3(25.1507f, 23.2526f, -17.6895f)); 
  data[35].mAabb = Aabb(Vector3(18.3854f, 13.9101f, 12.5094f), Vector3(19.7855f, 18.0285f, 15.566f)); 
  data[36].mAabb = Aabb(Vector3(28.0612f, 11.8808f, 9.02773f), Vector3(34.9618f, 19.0372f, 11.2925f)); 
  data[37].mAabb = Aabb(Vector3(-27.1278f, 32.0145f, -30.8798f), Vector3(-23.942f, 32.5312f, -25.6228f)); 
  data[38].mAabb = Aabb(Vector3(-1.24835f, -2.65835f, 29.351f), Vector3(3.00813f, 5.33701f, 32.4325f)); 
  data[39].mAabb = Aabb(Vector3(-36.801f, 7.4291f, -16.6913f), Vector3(-32.6181f, 8.27187f, -10.2681f)); 
  data[40].mAabb = Aabb(Vector3(-8.39238f, 23.21f, -5.4422f), Vector3(-2.40284f, 28.927f, -1.09805f)); 
  data[41].mAabb = Aabb(Vector3(-11.9562f, 12.8722f, 40.9751f), Vector3(-7.32774f, 19.571f, 47.1901f)); 
  data[42].mAabb = Aabb(Vector3(9.50076f, 2.77873f, -0.450044f), Vector3(11.6572f, 8.31529f, 6.80311f)); 
  data[43].mAabb = Aabb(Vector3(6.5418f, 16.3412f, 37.7947f), Vector3(10.1134f, 21.5985f, 40.5436f)); 
  data[44].mAabb = Aabb(Vector3(-38.7723f, 16.7897f, -29.5999f), Vector3(-37.2717f, 21.4342f, -22.624f)); 
  data[45].mAabb = Aabb(Vector3(-39.9629f, 0.932087f, 9.81144f), Vector3(-37.1115f, 2.91369f, 13.5045f)); 
  data[46].mAabb = Aabb(Vector3(-10.0372f, -16.5167f, 11.8084f), Vector3(-3.30986f, -11.8888f, 15.6485f)); 
  data[47].mAabb = Aabb(Vector3(1.63608f, -5.48181f, -45.2486f), Vector3(7.02513f, -3.48234f, -43.8534f)); 
  data[48].mAabb = Aabb(Vector3(-21.3927f, -24.8435f, -35.3125f), Vector3(-18.8897f, -23.9184f, -27.8991f)); 
  data[49].mAabb = Aabb(Vector3(5.00597f, 24.4076f, -34.8396f), Vector3(12.5623f, 26.0515f, -33.7317f)); 
  data[50].mAabb = Aabb(Vector3(31.2016f, -23.3038f, -18.79f), Vector3(34.6384f, -18.1698f, -15.953f)); 
  data[51].mAabb = Aabb(Vector3(7.68814f, -20.3599f, -43.8262f), Vector3(12.074f, -18.0837f, -38.6554f)); 
  data[52].mAabb = Aabb(Vector3(-12.2753f, -42.909f, 22.3488f), Vector3(-4.36811f, -35.1335f, 26.9228f)); 
  data[53].mAabb = Aabb(Vector3(-12.0285f, 36.7885f, -10.2666f), Vector3(-4.64617f, 38.2893f, -6.05138f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-31.8927f, 10.0449f, 68.9043f), Vector3(-30.4232f, 47.0024f, 4.11026f), Vector3(-30.4232f, -17.867f, -32.8902f), Vector3(-31.8927f, -54.8245f, 31.9038f), Vector3(7.34823f, -2.70142f, 24.3667f), Vector3(7.51232f, 1.42539f, 17.1315f), Vector3(7.51232f, -5.87398f, 12.9681f), Vector3(7.34823f, -10.0008f, 20.2032f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mAabb = Aabb(Vector3(-8.07805f, 44.4235f, 17.1248f), Vector3(-1.36324f, 45.6313f, 19.1278f)); 
  data[1].mAabb = Aabb(Vector3(-11.3589f, 35.4065f, -13.2102f), Vector3(-6.6344f, 43.0459f, -11.7041f)); 
  data[2].mAabb = Aabb(Vector3(-12.3519f, -5.35152f, 33.772f), Vector3(-8.9149f, -2.76339f, 36.8486f)); 
  data[3].mAabb = Aabb(Vector3(-3.07875f, 17.3616f, -5.51661f), Vector3(1.155f, 22.6171f, -1.51411f)); 
  data[4].mAabb = Aabb(Vector3(24.2058f, 25.2254f, -22.859f), Vector3(27.6355f, 32.2076f, -17.5352f)); 
  data[5].mAabb = Aabb(Vector3(-12.944f, 9.83658f, -46.2524f), Vector3(-8.92617f, 10.4261f, -42.5485f)); 
  data[6].mAabb = Aabb(Vector3(29.6662f, 13.326f, 8.14272f), Vector3(31.3732f, 17.7453f, 11.3777f)); 
  data[7].mAabb = Aabb(Vector3(-4.50734f, -37.7549f, -19.7749f), Vector3(1.29711f, -33.818f, -16.6839f)); 
  data[8].mAabb = Aabb(Vector3(44.1281f, -10.2681f, 2.22273f), Vector3(48.7517f, -6.33518f, 5.44753f)); 
  data[9].mAabb = Aabb(Vector3(-33.6496f, 31.2216f, 8.43191f), Vector3(-30.9093f, 36.3044f, 15.7625f)); 
  data[10].mAabb = Aabb(Vector3(35.3408f, -7.17908f, -24.3654f), Vector3(39.9971f, -1.51982f, -16.784f)); 
  data[11].mAabb = Aabb(Vector3(-30.1292f, 22.3698f, 11.153f), Vector3(-24.6476f, 26.6513f, 17.5468f)); 
  data[12].mAabb = Aabb(Vector3(-48.6435f, -0.702525f, -4.84053f), Vector3(-46.2377f, 5.38211f, 0.536683f)); 
  data[13].mAabb = Aabb(Vector3(-25.472f, 17.4357f, -29.1247f), Vector3(-24.0891f, 18.172f, -28.0602f)); 
  data[14].mAabb = Aabb(Vector3(12.9632f, -26.1894f, 19.6813f), Vector3(17.2923f, -18.4888f, 23.0374f)); 
  data[15].mAabb = Aabb(Vector3(25.9909f, -11.0916f, -22.1346f), Vector3(31.1921f, -8.14697f, -20.7813f)); 
  data[16].mAabb = Aabb(Vector3(29.5245f, 20.7478f, 14.4361f), Vector3(34.0373f, 21.2763f, 18.8382f)); 
  data[17].mAabb = Aabb(Vector3(39.3348f, 14.159f, 16.2079f), Vector3(45.6535f, 21.6292f, 21.8381f)); 
  data[18].mAabb = Aabb(Vector3(-3.13245f, 37.9021f, -6.77211f), Vector3(-2.52811f, 38.6838f, -2.15844f)); 
  data[19].mAabb = Aabb(Vector3(19.482f, -41.1651f, -7.72758f), Vector3(20.7898f, -38.2594f, -4.73935f)); 
  data[20].mAabb = Aabb(Vector3(29.6433f, 13.6431f, 16.1344f), Vector3(36.2538f, 21.3619f, 23.9182f)); 
  data[21].mAabb = Aabb(Vector3(34.0043f, 2.19521f, -39.4347f), Vector3(34.4465f, 8.58345f, -32.2863f)); 
  data[22].mAabb = Aabb(Vector3(-26.0965f, -28.4488f, 3.06223f), Vector3(-19.8231f, -26.4902f, 9.77171f)); 
  data[23].mAabb = Aabb(Vector3(-7.55619f, -17.4848f, -48.904f), Vector3(-0.586474f, -10.4578f, -44.0828f)); 
  data[24].mAabb = Aabb(Vector3(21.7778f, -3.91777f, 32.842f), Vector3(29.1233f, 1.99894f, 38.9756f)); 
  data[25].mAabb = Aabb(Vector3(3.65439f, -44.3739f, 16.7127f), Vector3(5.81436f, -38.3621f, 18.3494f)); 
  data[26].mAabb = Aabb(Vector3(18.994f, 12.67f, 4.76207f), Vector3(22.6579f, 15.4924f, 11.1011f)); 
  data[27].mAabb = Aabb(Vector3(-34.0778f, 23.2048f, -1.17303f), Vector3(-30.7377f, 28.1643f, 1.78133f)); 
  data[28].mAabb = Aabb(Vector3(12.4845f, 10.3161f, 13.8444f), Vector3(17.8316f, 11.302f, 20.8482f)); 
  data[29].mAabb = Aabb(Vector3(-27.6626f, 3.81142f, 20.8964f), Vector3(-20.135f, 6.8741f, 27.1608f)); 
  data[30].mAabb = Aabb(Vector3(-22.1423f, 1.11168f, -9.09277f), Vector3(-17.3232f, 2.88245f, -5.56435f)); 
  data[31].mAabb = Aabb(Vector3(-3.32219f, -43.0246f, 5.66263f), Vector3(-1.44797f, -36.4211f, 7.7595f)); 
  data[32].mAabb = Aabb(Vector3(36.5518f, -15.7408f, 5.82246f), Vector3(37.5303f, -9.80157f, 10.8756f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(38.939f, 87.8027f, 35.2721f), Vector3(-73.3332f, 18.7091f, 50.0776f), Vector3(-5.39565f, -91.6845f, 50.0776f), Vector3(106.877f, -22.5909f, 35.2721f), Vector3(13.5148f, -1.4074f, -12.4932f), Vector3(7.28823f, -5.2393f, -11.6721f), Vector3(9.55379f, -8.92067f, -11.6721f), Vector3(15.7804f, -5.08877f, -12.4932f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(47); 
  data[0].mAabb = Aabb(Vector3(-11.5463f, -16.0925f, -6.38737f), Vector3(-9.61548f, -11.1005f, -5.87372f)); 
  data[1].mAabb = Aabb(Vector3(32.6805f, -8.06635f, -13.3268f), Vector3(37.7103f, -5.22842f, -7.0942f)); 
  data[2].mAabb = Aabb(Vector3(-14.2383f, -21.9914f, 19.7978f), Vector3(-10.4128f, -16.2224f, 27.6779f)); 
  data[3].mAabb = Aabb(Vector3(-24.488f, -18.6512f, -8.91488f), Vector3(-18.6238f, -17.1762f, -7.49318f)); 
  data[4].mAabb = Aabb(Vector3(-27.4853f, 36.0573f, 9.92892f), Vector3(-26.9373f, 37.9485f, 14.3881f)); 
  data[5].mAabb = Aabb(Vector3(4.29437f, -19.4581f, -39.9017f), Vector3(8.5534f, -18.4467f, -37.8915f)); 
  data[6].mAabb = Aabb(Vector3(-39.4405f, -2.62206f, -18.1194f), Vector3(-34.3602f, -0.266333f, -12.4836f)); 
  data[7].mAabb = Aabb(Vector3(-4.2312f, 38.9147f, 1.38759f), Vector3(-0.299667f, 41.4377f, 3.67466f)); 
  data[8].mAabb = Aabb(Vector3(27.413f, -33.8293f, 24.3504f), Vector3(31.8766f, -26.7155f, 27.28f)); 
  data[9].mAabb = Aabb(Vector3(7.5759f, 1.01803f, 5.10687f), Vector3(10.8986f, 6.89113f, 10.8083f)); 
  data[10].mAabb = Aabb(Vector3(8.01731f, -24.8045f, 11.4819f), Vector3(9.20823f, -21.2578f, 19.2949f)); 
  data[11].mAabb = Aabb(Vector3(-31.7534f, -31.4738f, -2.43324f), Vector3(-24.712f, -26.6423f, 1.92134f)); 
  data[12].mAabb = Aabb(Vector3(-2.79566f, -35.1966f, -3.80937f), Vector3(2.29766f, -30.1387f, 3.38464f)); 
  data[13].mAabb = Aabb(Vector3(-14.0741f, 2.8534f, 8.73857f), Vector3(-10.714f, 3.29027f, 11.3028f)); 
  data[14].mAabb = Aabb(Vector3(6.87807f, 10.4282f, -17.2442f), Vector3(14.6626f, 15.6113f, -12.1181f)); 
  data[15].mAabb = Aabb(Vector3(-25.3934f, -28.7915f, 15.9862f), Vector3(-22.8682f, -22.0772f, 18.5472f)); 
  data[16].mAabb = Aabb(Vector3(16.757f, -26.9051f, 32.1633f), Vector3(22.686f, -21.2623f, 34.4712f)); 
  data[17].mAabb = Aabb(Vector3(11.555f, 41.1535f, -7.28304f), Vector3(18.0017f, 45.0134f, -3.90121f)); 
  data[18].mAabb = Aabb(Vector3(0.799529f, -25.4839f, 42.4724f), Vector3(7.39142f, -18.5434f, 46.8086f)); 
  data[19].mAabb = Aabb(Vector3(-21.4156f, -6.95143f, 27.7217f), Vector3(-16.8921f, -3.94326f, 34.7436f)); 
  data[20].mAabb = Aabb(Vector3(-28.098f, -9.09544f, -31.8856f), Vector3(-23.1635f, -3.17224f, -30.4135f)); 
  data[21].mAabb = Aabb(Vector3(19.252f, -25.1345f, 7.90441f), Vector3(22.2286f, -22.2439f, 14.8117f)); 
  data[22].mAabb = Aabb(Vector3(29.9039f, 25.6005f, -0.665433f), Vector3(34.224f, 28.5184f, 2.61991f)); 
  data[23].mAabb = Aabb(Vector3(-6.14003f, -12.1732f, -2.89646f), Vector3(1.50301f, -9.67277f, 3.05527f)); 
  data[24].mAabb = Aabb(Vector3(-9.4466f, 28.5731f, 24.8729f), Vector3(-8.75691f, 30.6259f, 27.412f)); 
  data[25].mAabb = Aabb(Vector3(42.6381f, -14.7302f, 3.54748f), Vector3(48.2349f, -12.656f, 4.80636f)); 
  data[26].mAabb = Aabb(Vector3(-10.0058f, -23.472f, 40.0389f), Vector3(-8.73367f, -16.0938f, 44.9161f)); 
  data[27].mAabb = Aabb(Vector3(10.0792f, -12.7126f, -21.1229f), Vector3(12.4435f, -8.20697f, -15.5887f)); 
  data[28].mAabb = Aabb(Vector3(4.81179f, 18.1977f, -40.1168f), Vector3(11.9003f, 24.1965f, -33.8438f)); 
  data[29].mAabb = Aabb(Vector3(35.0165f, -3.35063f, 24.5507f), Vector3(35.5037f, -0.649542f, 28.8167f)); 
  data[30].mAabb = Aabb(Vector3(12.3276f, -22.2902f, 15.1155f), Vector3(18.8448f, -20.6572f, 16.7265f)); 
  data[31].mAabb = Aabb(Vector3(30.0652f, 25.7655f, -15.7577f), Vector3(36.6671f, 28.7568f, -13.415f)); 
  data[32].mAabb = Aabb(Vector3(6.75673f, -35.8837f, 28.4144f), Vector3(7.33695f, -29.6787f, 33.5687f)); 
  data[33].mAabb = Aabb(Vector3(-14.3696f, 33.1765f, 6.25514f), Vector3(-8.37335f, 34.8865f, 12.2189f)); 
  data[34].mAabb = Aabb(Vector3(17.2792f, 10.2482f, 13.3614f), Vector3(18.0897f, 17.5357f, 20.0266f)); 
  data[35].mAabb = Aabb(Vector3(-40.6723f, 30.7223f, -2.25945f), Vector3(-33.6641f, 33.7774f, 0.0512743f)); 
  data[36].mAabb = Aabb(Vector3(0.325766f, -25.459f, -5.95015f), Vector3(5.49447f, -24.0519f, -2.40945f)); 
  data[37].mAabb = Aabb(Vector3(-16.9406f, 10.8507f, -1.24388f), Vector3(-13.9187f, 13.7902f, 1.99484f)); 
  data[38].mAabb = Aabb(Vector3(33.5305f, -10.371f, -19.8883f), Vector3(40.3954f, -5.74688f, -17.5606f)); 
  data[39].mAabb = Aabb(Vector3(2.59514f, -18.3884f, 0.475044f), Vector3(10.2298f, -14.237f, 3.12672f)); 
  data[40].mAabb = Aabb(Vector3(3.78554f, 13.1647f, -39.9755f), Vector3(7.34063f, 14.361f, -39.2024f)); 
  data[41].mAabb = Aabb(Vector3(38.8221f, -9.16246f, -24.8345f), Vector3(41.9676f, -6.43354f, -19.8107f)); 
  data[42].mAabb = Aabb(Vector3(-24.6111f, -7.76428f, 37.7066f), Vector3(-23.6421f, -0.0189464f, 41.0887f)); 
  data[43].mAabb = Aabb(Vector3(-33.4802f, 17.8397f, -24.8108f), Vector3(-29.9422f, 21.1074f, -19.6813f)); 
  data[44].mAabb = Aabb(Vector3(-10.9303f, 33.5128f, 0.420059f), Vector3(-9.69599f, 34.4455f, 6.05473f)); 
  data[45].mAabb = Aabb(Vector3(-20.0943f, -9.961f, -24.1319f), Vector3(-16.3808f, -6.89043f, -23.6878f)); 
  data[46].mAabb = Aabb(Vector3(-34.5158f, -34.9589f, -27.7924f), Vector3(-26.5554f, -27.2977f, -20.3551f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-11.2913f, -51.8817f, -60.4829f), Vector3(-24.4539f, -37.8417f, 47.1027f), Vector3(-24.4539f, 68.3418f, 33.2457f), Vector3(-11.2913f, 54.3018f, -74.3399f), Vector3(61.4754f, 5.53236f, -8.12106f), Vector3(60.3936f, 6.68623f, 0.720856f), Vector3(60.3936f, 13.4021f, -0.155573f), Vector3(61.4754f, 12.2483f, -8.99749f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(52); 
  data[0].mAabb = Aabb(Vector3(12.015f, -9.21467f, 19.3586f), Vector3(19.0272f, -3.5568f, 21.7352f)); 
  data[1].mAabb = Aabb(Vector3(-10.4018f, -2.35412f, 44.8471f), Vector3(-7.02489f, -1.84465f, 45.8357f)); 
  data[2].mAabb = Aabb(Vector3(0.388557f, 40.4883f, 3.86917f), Vector3(1.56695f, 44.303f, 10.6557f)); 
  data[3].mAabb = Aabb(Vector3(-19.8041f, -45.856f, -10.4793f), Vector3(-17.4313f, -39.3416f, -5.74041f)); 
  data[4].mAabb = Aabb(Vector3(-12.8559f, -30.7493f, 17.682f), Vector3(-10.2875f, -26.9358f, 25.2515f)); 
  data[5].mAabb = Aabb(Vector3(-43.9318f, 23.7002f, -12.3742f), Vector3(-37.7715f, 27.9521f, -10.4659f)); 
  data[6].mAabb = Aabb(Vector3(-32.154f, 14.7833f, -3.83834f), Vector3(-29.6997f, 20.7292f, -2.11373f)); 
  data[7].mAabb = Aabb(Vector3(32.8642f, 6.52985f, 24.9653f), Vector3(35.1629f, 10.8784f, 31.461f)); 
  data[8].mAabb = Aabb(Vector3(-4.33308f, -34.7711f, -14.4397f), Vector3(-3.04104f, -28.9592f, -6.69178f)); 
  data[9].mAabb = Aabb(Vector3(-27.6545f, 11.0876f, 5.46829f), Vector3(-22.5563f, 14.3565f, 11.4019f)); 
  data[10].mAabb = Aabb(Vector3(-9.0381f, 32.7605f, -34.1677f), Vector3(-1.72696f, 37.963f, -28.7353f)); 
  data[11].mAabb = Aabb(Vector3(-39.5835f, 11.9926f, -19.1367f), Vector3(-37.9475f, 19.4976f, -13.769f)); 
  data[12].mAabb = Aabb(Vector3(-19.6218f, 26.6045f, 28.0858f), Vector3(-17.9939f, 28.4068f, 35.7005f)); 
  data[13].mAabb = Aabb(Vector3(-12.7776f, -38.9223f, -4.55157f), Vector3(-6.56517f, -32.8551f, 2.08415f)); 
  data[14].mAabb = Aabb(Vector3(4.87774f, 19.0048f, -12.9454f), Vector3(7.53383f, 26.8248f, -5.81498f)); 
  data[15].mAabb = Aabb(Vector3(-8.80068f, -24.8754f, 42.0411f), Vector3(-8.10194f, -20.7899f, 43.5544f)); 
  data[16].mAabb = Aabb(Vector3(-12.5964f, -45.4525f, -18.4708f), Vector3(-11.0356f, -38.3898f, -16.4314f)); 
  data[17].mAabb = Aabb(Vector3(-15.1619f, -35.4454f, -23.4f), Vector3(-9.5152f, -29.31f, -21.6128f)); 
  data[18].mAabb = Aabb(Vector3(32.2465f, 10.1085f, 18.0767f), Vector3(40.1823f, 17.0761f, 20.0628f)); 
  data[19].mAabb = Aabb(Vector3(5.24666f, -9.70486f, 30.166f), Vector3(11.6627f, -8.40447f, 33.4817f)); 
  data[20].mAabb = Aabb(Vector3(12.8501f, 20.1392f, 14.0367f), Vector3(20.7f, 24.0382f, 19.9908f)); 
  data[21].mAabb = Aabb(Vector3(31.3346f, -22.8945f, -30.2012f), Vector3(33.3225f, -17.1102f, -24.7556f)); 
  data[22].mAabb = Aabb(Vector3(12.308f, 31.2706f, 16.7569f), Vector3(17.3498f, 34.2976f, 23.0389f)); 
  data[23].mAabb = Aabb(Vector3(-14.4441f, 8.08302f, 34.4873f), Vector3(-7.31963f, 10.3661f, 40.227f)); 
  data[24].mAabb = Aabb(Vector3(34.8907f, 8.48551f, 3.70405f), Vector3(35.5004f, 10.7844f, 5.30179f)); 
  data[25].mAabb = Aabb(Vector3(-30.0837f, 12.347f, -19.3823f), Vector3(-28.1487f, 19.082f, -11.5968f)); 
  data[26].mAabb = Aabb(Vector3(16.4275f, 30.191f, 14.6222f), Vector3(22.3985f, 36.008f, 19.7605f)); 
  data[27].mAabb = Aabb(Vector3(0.926846f, -33.2041f, 25.6814f), Vector3(2.44294f, -30.3511f, 33.3216f)); 
  data[28].mAabb = Aabb(Vector3(-6.41728f, 4.14183f, 15.6177f), Vector3(-1.17204f, 10.6741f, 16.3308f)); 
  data[29].mAabb = Aabb(Vector3(23.0677f, -44.5207f, 0.212081f), Vector3(27.1894f, -39.0614f, 4.49314f)); 
  data[30].mAabb = Aabb(Vector3(22.7564f, 27.9176f, -6.9391f), Vector3(30.1877f, 31.4885f, 0.641785f)); 
  data[31].mAabb = Aabb(Vector3(12.9828f, -7.44123f, -40.7919f), Vector3(18.7156f, -6.81068f, -33.5724f)); 
  data[32].mAabb = Aabb(Vector3(-30.4989f, -26.2761f, 8.26712f), Vector3(-24.7666f, -24.5738f, 9.55777f)); 
  data[33].mAabb = Aabb(Vector3(18.4035f, -0.0562799f, 36.5689f), Vector3(25.0877f, 0.811081f, 39.6095f)); 
  data[34].mAabb = Aabb(Vector3(-7.26388f, -16.903f, -26.3423f), Vector3(-0.063607f, -11.8399f, -22.5791f)); 
  data[35].mAabb = Aabb(Vector3(-4.43351f, 16.9144f, 34.9076f), Vector3(-3.82244f, 18.2155f, 40.944f)); 
  data[36].mAabb = Aabb(Vector3(42.2052f, -3.23956f, -2.2627f), Vector3(45.4576f, 2.60919f, 3.80129f)); 
  data[37].mAabb = Aabb(Vector3(-6.00332f, 18.8182f, -5.81191f), Vector3(-5.42055f, 25.5664f, -3.12451f)); 
  data[38].mAabb = Aabb(Vector3(-33.5808f, -19.4972f, 5.46087f), Vector3(-27.3475f, -16.6115f, 12.1975f)); 
  data[39].mAabb = Aabb(Vector3(3.66114f, 24.2615f, -16.7821f), Vector3(5.1392f, 31.0905f, -10.4196f)); 
  data[40].mAabb = Aabb(Vector3(-33.231f, 23.7084f, 10.4165f), Vector3(-25.8565f, 28.9059f, 14.3728f)); 
  data[41].mAabb = Aabb(Vector3(-29.9733f, -1.79436f, 13.0703f), Vector3(-28.1005f, -1.23734f, 14.6634f)); 
  data[42].mAabb = Aabb(Vector3(-5.21057f, -16.665f, -18.5052f), Vector3(-1.08931f, -14.3905f, -14.5361f)); 
  data[43].mAabb = Aabb(Vector3(-25.6473f, 13.3032f, 11.8457f), Vector3(-18.3505f, 14.1237f, 13.7364f)); 
  data[44].mAabb = Aabb(Vector3(-31.0098f, -8.35242f, -30.9429f), Vector3(-25.6222f, -7.7889f, -25.2686f)); 
  data[45].mAabb = Aabb(Vector3(-40.6608f, 9.46748f, 17.4844f), Vector3(-39.337f, 15.6236f, 17.9461f)); 
  data[46].mAabb = Aabb(Vector3(18.1722f, 30.7817f, 13.0173f), Vector3(21.2699f, 32.2681f, 14.035f)); 
  data[47].mAabb = Aabb(Vector3(-19.1247f, -20.5999f, 2.48781f), Vector3(-15.1642f, -13.5594f, 9.88105f)); 
  data[48].mAabb = Aabb(Vector3(-3.95374f, -46.5099f, -15.5855f), Vector3(-2.96948f, -41.2078f, -11.3308f)); 
  data[49].mAabb = Aabb(Vector3(25.4872f, 9.36316f, -3.26688f), Vector3(31.6771f, 15.8655f, 2.5937f)); 
  data[50].mAabb = Aabb(Vector3(-44.307f, -5.53875f, -6.46328f), Vector3(-38.9094f, 1.78631f, -1.11575f)); 
  data[51].mAabb = Aabb(Vector3(-2.23197f, 15.2275f, 12.682f), Vector3(4.30774f, 15.9007f, 18.4111f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(6.19322f, 48.4178f, 44.5239f), Vector3(-45.0512f, 37.3943f, 6.63668f), Vector3(-31.7662f, -24.3629f, 6.63668f), Vector3(19.4782f, -13.3394f, 44.5239f), Vector3(21.8981f, 23.8144f, -16.2576f), Vector3(14.0036f, 22.1162f, -22.0944f), Vector3(15.7823f, 13.8473f, -22.0944f), Vector3(23.6769f, 15.5456f, -16.2576f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(57); 
  data[0].mAabb = Aabb(Vector3(-6.42543f, -41.0094f, -8.89244f), Vector3(-2.48485f, -37.0456f, -6.67472f)); 
  data[1].mAabb = Aabb(Vector3(-2.65661f, 35.2383f, -2.74389f), Vector3(4.0538f, 38.914f, 3.99574f)); 
  data[2].mAabb = Aabb(Vector3(-3.70961f, 4.29989f, -33.0701f), Vector3(-0.284644f, 5.56271f, -27.8935f)); 
  data[3].mAabb = Aabb(Vector3(11.2381f, -30.6382f, -16.5361f), Vector3(17.233f, -24.9565f, -13.2108f)); 
  data[4].mAabb = Aabb(Vector3(9.69805f, 23.6158f, 30.2746f), Vector3(14.3646f, 28.1223f, 37.5178f)); 
  data[5].mAabb = Aabb(Vector3(-12.7235f, -35.1144f, 31.2438f), Vector3(-5.455f, -33.387f, 36.9023f)); 
  data[6].mAabb = Aabb(Vector3(18.1957f, 21.9047f, 22.3848f), Vector3(24.7326f, 22.696f, 23.3723f)); 
  data[7].mAabb = Aabb(Vector3(45.354f, 4.40037f, 8.07649f), Vector3(47.3164f, 12.0738f, 15.171f)); 
  data[8].mAabb = Aabb(Vector3(-38.4812f, 4.4683f, -15.6944f), Vector3(-34.6058f, 7.37813f, -10.1887f)); 
  data[9].mAabb = Aabb(Vector3(-26.8704f, 13.3029f, 24.797f), Vector3(-24.5504f, 14.7689f, 27.9595f)); 
  data[10].mAabb = Aabb(Vector3(30.2132f, -36.0833f, -10.9627f), Vector3(36.5639f, -32.8704f, -4.68812f)); 
  data[11].mAabb = Aabb(Vector3(5.17341f, -20.215f, -40.4179f), Vector3(12.299f, -14.2115f, -39.9949f)); 
  data[12].mAabb = Aabb(Vector3(44.4514f, -11.4938f, 1.72885f), Vector3(51.735f, -9.90297f, 8.79274f)); 
  data[13].mAabb = Aabb(Vector3(8.1361f, 23.8605f, -10.6202f), Vector3(15.6597f, 29.9802f, -4.45852f)); 
  data[14].mAabb = Aabb(Vector3(21.9971f, 23.507f, -12.0408f), Vector3(25.8423f, 24.4358f, -7.42461f)); 
  data[15].mAabb = Aabb(Vector3(-41.0446f, -8.51559f, -23.2969f), Vector3(-33.9958f, -3.16064f, -21.766f)); 
  data[16].mAabb = Aabb(Vector3(27.4981f, -3.44271f, 0.298388f), Vector3(34.1408f, 0.562572f, 4.9044f)); 
  data[17].mAabb = Aabb(Vector3(-17.6125f, 10.3929f, -43.6087f), Vector3(-14.2439f, 17.9626f, -40.2517f)); 
  data[18].mAabb = Aabb(Vector3(24.1109f, 3.01393f, 17.7662f), Vector3(27.4113f, 7.0039f, 21.3584f)); 
  data[19].mAabb = Aabb(Vector3(44.4055f, 14.5733f, 2.63117f), Vector3(49.0663f, 15.642f, 7.04792f)); 
  data[20].mAabb = Aabb(Vector3(8.94904f, -25.6726f, 29.235f), Vector3(13.0058f, -23.1647f, 31.5121f)); 
  data[21].mAabb = Aabb(Vector3(-20.4509f, -11.9386f, -12.4446f), Vector3(-15.1214f, -8.04277f, -8.63948f)); 
  data[22].mAabb = Aabb(Vector3(-13.3342f, 38.9276f, -25.2039f), Vector3(-8.55006f, 40.7584f, -20.0215f)); 
  data[23].mAabb = Aabb(Vector3(16.8427f, 8.55694f, 32.7359f), Vector3(18.4237f, 13.3193f, 33.5344f)); 
  data[24].mAabb = Aabb(Vector3(-18.8069f, -38.9661f, 13.1288f), Vector3(-15.7633f, -33.8076f, 16.7843f)); 
  data[25].mAabb = Aabb(Vector3(-0.119916f, 3.88368f, -45.0433f), Vector3(1.84522f, 6.55461f, -40.811f)); 
  data[26].mAabb = Aabb(Vector3(-7.25372f, 20.9464f, 34.4565f), Vector3(-5.86101f, 22.8978f, 38.1761f)); 
  data[27].mAabb = Aabb(Vector3(-35.9681f, 32.6334f, -1.03894f), Vector3(-28.9707f, 35.1336f, 5.76262f)); 
  data[28].mAabb = Aabb(Vector3(25.7381f, -4.41688f, -21.767f), Vector3(30.0885f, -1.73644f, -17.2883f)); 
  data[29].mAabb = Aabb(Vector3(-39.5882f, 1.4977f, -31.6714f), Vector3(-35.4152f, 3.90492f, -24.9731f)); 
  data[30].mAabb = Aabb(Vector3(-30.0558f, 36.4664f, 1.06103f), Vector3(-26.0159f, 37.6167f, 4.85154f)); 
  data[31].mAabb = Aabb(Vector3(12.2403f, 5.25276f, 21.4202f), Vector3(17.9307f, 10.3686f, 24.6183f)); 
  data[32].mAabb = Aabb(Vector3(-31.8141f, -23.6398f, -36.2708f), Vector3(-25.2974f, -18.8717f, -30.95f)); 
  data[33].mAabb = Aabb(Vector3(-3.03403f, 34.5974f, 23.5284f), Vector3(4.95345f, 39.948f, 26.5384f)); 
  data[34].mAabb = Aabb(Vector3(10.5273f, 4.17261f, 21.7197f), Vector3(13.1224f, 7.48022f, 26.6082f)); 
  data[35].mAabb = Aabb(Vector3(-19.4927f, 21.4829f, 21.8108f), Vector3(-12.6805f, 22.162f, 23.8075f)); 
  data[36].mAabb = Aabb(Vector3(-12.2655f, 22.3297f, -8.24029f), Vector3(-8.67629f, 23.6749f, -2.02113f)); 
  data[37].mAabb = Aabb(Vector3(12.0916f, 21.4943f, -14.3512f), Vector3(19.8258f, 22.3203f, -12.4647f)); 
  data[38].mAabb = Aabb(Vector3(18.1007f, -30.689f, -20.5579f), Vector3(25.8746f, -22.9715f, -14.5442f)); 
  data[39].mAabb = Aabb(Vector3(40.6066f, -11.3471f, 19.5351f), Vector3(46.8188f, -4.62739f, 23.3154f)); 
  data[40].mAabb = Aabb(Vector3(-32.5716f, 30.5795f, 2.84018f), Vector3(-29.7857f, 31.3518f, 8.89002f)); 
  data[41].mAabb = Aabb(Vector3(9.51318f, -18.3098f, -30.2777f), Vector3(15.8434f, -15.5751f, -22.605f)); 
  data[42].mAabb = Aabb(Vector3(-43.7738f, -11.031f, 18.57f), Vector3(-37.1993f, -5.05997f, 25.995f)); 
  data[43].mAabb = Aabb(Vector3(25.9697f, 24.445f, 21.1689f), Vector3(32.6832f, 26.2399f, 26.5651f)); 
  data[44].mAabb = Aabb(Vector3(11.2193f, 20.2951f, -35.086f), Vector3(12.5292f, 20.8994f, -28.4308f)); 
  data[45].mAabb = Aabb(Vector3(-24.587f, 29.1564f, -5.95288f), Vector3(-17.0507f, 33.5525f, -2.15008f)); 
  data[46].mAabb = Aabb(Vector3(3.89968f, 1.91032f, -30.2718f), Vector3(9.97017f, 4.49915f, -28.3299f)); 
  data[47].mAabb = Aabb(Vector3(-3.67058f, 17.3142f, 0.266662f), Vector3(1.46797f, 19.7258f, 5.06101f)); 
  data[48].mAabb = Aabb(Vector3(2.02004f, -1.17199f, -45.4781f), Vector3(7.93327f, 3.7272f, -38.9579f)); 
  data[49].mAabb = Aabb(Vector3(-40.451f, 20.7296f, 22.6384f), Vector3(-35.0938f, 23.4757f, 24.4535f)); 
  data[50].mAabb = Aabb(Vector3(-29.3209f, -0.579158f, -10.3417f), Vector3(-23.3743f, 5.62655f, -8.94508f)); 
  data[51].mAabb = Aabb(Vector3(2.08602f, 30.5555f, -0.0327904f), Vector3(8.97457f, 34.6512f, 2.98466f)); 
  data[52].mAabb = Aabb(Vector3(1.28162f, 29.2637f, -28.1673f), Vector3(5.29293f, 33.7384f, -21.3685f)); 
  data[53].mAabb = Aabb(Vector3(-1.66469f, 35.296f, 31.0312f), Vector3(-0.669761f, 35.9769f, 34.9472f)); 
  data[54].mAabb = Aabb(Vector3(6.84734f, -24.4247f, -36.3523f), Vector3(13.0368f, -17.9249f, -29.3176f)); 
  data[55].mAabb = Aabb(Vector3(-16.9364f, 34.7379f, 16.8309f), Vector3(-9.66978f, 41.7507f, 20.17f)); 
  data[56].mAabb = Aabb(Vector3(30.5458f, -17.1565f, -5.63245f), Vector3(38.4927f, -13.4393f, 1.25794f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-22.4373f, -60.2127f, 29.419f), Vector3(-42.616f, 28.3916f, 32.902f), Vector3(-42.616f, 32.0927f, -61.2494f), Vector3(-22.4373f, -56.5116f, -64.7325f), Vector3(36.9613f, -1.14526f, -11.1104f), Vector3(35.7846f, 4.02175f, -10.9073f), Vector3(35.7846f, 4.35907f, -19.4883f), Vector3(36.9613f, -0.807941f, -19.6915f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(61); 
  data[0].mAabb = Aabb(Vector3(34.4719f, -2.40127f, -36.3429f), Vector3(35.3258f, 5.55536f, -31.8046f)); 
  data[1].mAabb = Aabb(Vector3(-28.646f, -34.5355f, 26.3094f), Vector3(-20.9578f, -29.332f, 29.0476f)); 
  data[2].mAabb = Aabb(Vector3(-21.8388f, -32.2684f, -22.3057f), Vector3(-14.9401f, -30.3988f, -18.0889f)); 
  data[3].mAabb = Aabb(Vector3(-35.7506f, -6.77444f, 20.1041f), Vector3(-28.4718f, -1.76022f, 23.0157f)); 
  data[4].mAabb = Aabb(Vector3(-5.65047f, 7.55383f, -39.3616f), Vector3(0.199668f, 15.4026f, -36.1836f)); 
  data[5].mAabb = Aabb(Vector3(-1.09362f, -25.2537f, 15.2746f), Vector3(-0.172915f, -20.6827f, 18.5583f)); 
  data[6].mAabb = Aabb(Vector3(-17.8213f, -22.1952f, 15.1527f), Vector3(-13.5549f, -16.0389f, 18.9743f)); 
  data[7].mAabb = Aabb(Vector3(-3.67474f, 16.9238f, 4.73689f), Vector3(-2.55874f, 20.9022f, 5.64739f)); 
  data[8].mAabb = Aabb(Vector3(2.68023f, 32.011f, -16.142f), Vector3(8.33507f, 38.2274f, -10.704f)); 
  data[9].mAabb = Aabb(Vector3(2.56239f, -5.30488f, -7.83342f), Vector3(5.96787f, -1.84651f, 0.157764f)); 
  data[10].mAabb = Aabb(Vector3(-22.0129f, -12.9176f, -41.5743f), Vector3(-15.7566f, -7.80527f, -39.2144f)); 
  data[11].mAabb = Aabb(Vector3(-24.9581f, 17.7372f, -25.7767f), Vector3(-17.3561f, 24.8396f, -20.3122f)); 
  data[12].mAabb = Aabb(Vector3(1.02616f, -36.6687f, 32.5567f), Vector3(3.26893f, -33.6259f, 33.8024f)); 
  data[13].mAabb = Aabb(Vector3(0.517281f, -27.2787f, -28.6256f), Vector3(0.960654f, -21.7206f, -23.2286f)); 
  data[14].mAabb = Aabb(Vector3(-20.9074f, 42.549f, 7.87204f), Vector3(-20.4142f, 42.9938f, 10.0334f)); 
  data[15].mAabb = Aabb(Vector3(17.1734f, 19.8994f, 33.199f), Vector3(24.9064f, 20.9656f, 34.7209f)); 
  data[16].mAabb = Aabb(Vector3(-24.2811f, -37.0828f, -5.56879f), Vector3(-18.171f, -32.9854f, 0.350243f)); 
  data[17].mAabb = Aabb(Vector3(3.60965f, 8.2838f, 32.2469f), Vector3(4.62939f, 13.5437f, 33.7417f)); 
  data[18].mAabb = Aabb(Vector3(28.2382f, 9.08694f, 22.164f), Vector3(29.8729f, 14.0044f, 29.702f)); 
  data[19].mAabb = Aabb(Vector3(17.5835f, 26.6453f, 11.7965f), Vector3(20.0775f, 31.3814f, 15.6206f)); 
  data[20].mAabb = Aabb(Vector3(-0.879928f, 31.8456f, 4.67359f), Vector3(4.25213f, 39.8082f, 12.5616f)); 
  data[21].mAabb = Aabb(Vector3(40.302f, -17.177f, 0.515978f), Vector3(46.2507f, -9.96091f, 7.16005f)); 
  data[22].mAabb = Aabb(Vector3(29.7532f, -38.698f, 7.40598f), Vector3(31.4537f, -37.3475f, 10.7857f)); 
  data[23].mAabb = Aabb(Vector3(-33.1997f, -0.67753f, 32.5505f), Vector3(-28.9216f, 3.02531f, 35.0377f)); 
  data[24].mAabb = Aabb(Vector3(-15.8684f, -26.1303f, 19.6344f), Vector3(-13.0975f, -20.7204f, 20.5983f)); 
  data[25].mAabb = Aabb(Vector3(29.2912f, 5.28146f, -15.6451f), Vector3(36.661f, 10.3349f, -15.0194f)); 
  data[26].mAabb = Aabb(Vector3(4.55721f, -37.882f, 25.8509f), Vector3(9.51623f, -32.7086f, 28.9507f)); 
  data[27].mAabb = Aabb(Vector3(14.5603f, 38.9913f, 20.4782f), Vector3(21.0174f, 42.5626f, 21.2133f)); 
  data[28].mAabb = Aabb(Vector3(15.2538f, -16.8699f, -11.8921f), Vector3(19.2867f, -10.5428f, -9.53173f)); 
  data[29].mAabb = Aabb(Vector3(-6.70683f, -12.7067f, 0.872149f), Vector3(-2.31466f, -9.86714f, 5.92465f)); 
  data[30].mAabb = Aabb(Vector3(-18.6429f, -29.7035f, -39.1898f), Vector3(-15.6855f, -27.1775f, -32.8598f)); 
  data[31].mAabb = Aabb(Vector3(-34.5077f, -31.5055f, -10.1148f), Vector3(-27.4061f, -27.1055f, -2.13176f)); 
  data[32].mAabb = Aabb(Vector3(-5.23332f, 14.3177f, 27.6531f), Vector3(-2.85579f, 17.3115f, 30.9167f)); 
  data[33].mAabb = Aabb(Vector3(40.5195f, -3.94207f, -11.087f), Vector3(45.01f, -0.646286f, -6.36226f)); 
  data[34].mAabb = Aabb(Vector3(-21.4261f, -2.38675f, 28.0523f), Vector3(-17.7198f, 4.72399f, 31.218f)); 
  data[35].mAabb = Aabb(Vector3(-29.6264f, 8.4452f, -25.4637f), Vector3(-26.3745f, 11.1338f, -21.9947f)); 
  data[36].mAabb = Aabb(Vector3(2.83292f, -27.9007f, -28.921f), Vector3(8.40751f, -21.3904f, -24.9686f)); 
  data[37].mAabb = Aabb(Vector3(4.458f, 3.8053f, -4.19091f), Vector3(7.50838f, 4.5662f, 2.54895f)); 
  data[38].mAabb = Aabb(Vector3(-14.2421f, 4.49912f, -17.1564f), Vector3(-13.2311f, 12.2922f, -14.9953f)); 
  data[39].mAabb = Aabb(Vector3(31.8739f, 21.6218f, -25.0268f), Vector3(33.2112f, 27.3409f, -17.7888f)); 
  data[40].mAabb = Aabb(Vector3(-7.98321f, -39.1367f, 30.1762f), Vector3(-6.27228f, -34.7661f, 34.4445f)); 
  data[41].mAabb = Aabb(Vector3(39.103f, 23.8746f, -15.3999f), Vector3(40.9661f, 25.5809f, -13.6518f)); 
  data[42].mAabb = Aabb(Vector3(-37.4118f, 29.382f, 5.18643f), Vector3(-31.8558f, 33.9356f, 11.2625f)); 
  data[43].mAabb = Aabb(Vector3(34.6549f, -3.85674f, -17.0819f), Vector3(40.4025f, 1.18579f, -9.40797f)); 
  data[44].mAabb = Aabb(Vector3(-6.76045f, 22.4005f, 11.024f), Vector3(-4.60953f, 28.7929f, 13.6603f)); 
  data[45].mAabb = Aabb(Vector3(-25.4434f, 4.55401f, -12.3718f), Vector3(-19.6717f, 8.95267f, -10.209f)); 
  data[46].mAabb = Aabb(Vector3(-12.4776f, 5.42242f, -48.5157f), Vector3(-11.5416f, 9.0123f, -44.7469f)); 
  data[47].mAabb = Aabb(Vector3(-3.62075f, 10.9586f, 12.2935f), Vector3(-2.8682f, 18.8791f, 16.2765f)); 
  data[48].mAabb = Aabb(Vector3(-18.509f, -15.277f, 1.19016f), Vector3(-16.6677f, -11.0857f, 4.82597f)); 
  data[49].mAabb = Aabb(Vector3(5.29077f, 19.4265f, 38.8801f), Vector3(10.356f, 25.5417f, 44.1963f)); 
  data[50].mAabb = Aabb(Vector3(-43.6968f, 5.93177f, 10.2977f), Vector3(-36.6563f, 7.90015f, 11.3406f)); 
  data[51].mAabb = Aabb(Vector3(-14.0485f, 30.7305f, 2.4956f), Vector3(-12.2761f, 35.8424f, 6.52176f)); 
  data[52].mAabb = Aabb(Vector3(-31.1713f, 14.5266f, 9.85452f), Vector3(-29.8f, 18.3354f, 13.5736f)); 
  data[53].mAabb = Aabb(Vector3(-43.7424f, -21.3122f, 10.0748f), Vector3(-39.3171f, -13.8046f, 13.2432f)); 
  data[54].mAabb = Aabb(Vector3(34.9012f, -7.23515f, -5.56066f), Vector3(42.4858f, 0.189409f, 1.28312f)); 
  data[55].mAabb = Aabb(Vector3(-17.5514f, -19.7501f, -19.1048f), Vector3(-16.3533f, -15.5363f, -11.2029f)); 
  data[56].mAabb = Aabb(Vector3(-17.3185f, 4.05544f, -6.08665f), Vector3(-10.1393f, 6.34344f, -5.40322f)); 
  data[57].mAabb = Aabb(Vector3(-26.185f, 13.08f, 0.98187f), Vector3(-18.9648f, 19.5951f, 1.54655f)); 
  data[58].mAabb = Aabb(Vector3(13.702f, -3.65269f, -22.4449f), Vector3(21.4955f, -1.53749f, -17.2446f)); 
  data[59].mAabb = Aabb(Vector3(-15.8015f, -31.0165f, -13.3063f), Vector3(-9.43896f, -24.0667f, -5.62708f)); 
  data[60].mAabb = Aabb(Vector3(-4.37023f, 26.1805f, 4.65648f), Vector3(-2.06044f, 33.1811f, 8.74712f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(58.7231f, 48.6255f, 15.942f), Vector3(-9.61109f, 60.4559f, 45.5642f), Vector3(-22.3818f, -13.3092f, 45.5642f), Vector3(45.9524f, -25.1397f, 15.942f), Vector3(-4.95672f, 24.6996f, -33.7299f), Vector3(-10.879f, 25.725f, -31.1627f), Vector3(-11.9002f, 19.8267f, -31.1627f), Vector3(-5.97787f, 18.8014f, -33.7299f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(57); 
  data[0].mAabb = Aabb(Vector3(-16.0164f, -4.71068f, 33.6419f), Vector3(-11.1151f, -2.25174f, 40.0501f)); 
  data[1].mAabb = Aabb(Vector3(-19.2356f, -25.9601f, -22.8074f), Vector3(-15.2433f, -24.2362f, -16.2846f)); 
  data[2].mAabb = Aabb(Vector3(-15.8314f, 0.853156f, -19.2587f), Vector3(-8.82297f, 4.10023f, -16.549f)); 
  data[3].mAabb = Aabb(Vector3(12.3817f, -33.2123f, 16.7862f), Vector3(14.1485f, -31.2105f, 18.0829f)); 
  data[4].mAabb = Aabb(Vector3(-20.1973f, -21.1891f, 9.67545f), Vector3(-19.1931f, -13.5757f, 13.9085f)); 
  data[5].mAabb = Aabb(Vector3(-38.443f, -13.5836f, 9.03607f), Vector3(-31.8852f, -11.429f, 16.0995f)); 
  data[6].mAabb = Aabb(Vector3(-0.132518f, 34.4031f, -28.5174f), Vector3(5.12455f, 39.8137f, -26.9739f)); 
  data[7].mAabb = Aabb(Vector3(-17.0878f, 13.4612f, 30.8681f), Vector3(-9.74772f, 13.9057f, 31.3409f)); 
  data[8].mAabb = Aabb(Vector3(-8.84351f, -42.7316f, 25.923f), Vector3(-8.42147f, -35.4288f, 28.0846f)); 
  data[9].mAabb = Aabb(Vector3(-23.48f, 6.14239f, -2.3724f), Vector3(-22.8532f, 7.45832f, 2.86611f)); 
  data[10].mAabb = Aabb(Vector3(8.00524f, -4.02391f, 2.1622f), Vector3(10.2146f, 0.0880677f, 9.32698f)); 
  data[11].mAabb = Aabb(Vector3(21.6812f, -16.3078f, -2.76533f), Vector3(28.2817f, -8.45322f, 4.64345f)); 
  data[12].mAabb = Aabb(Vector3(-10.7815f, -6.98528f, -51.7766f), Vector3(-6.35135f, -3.25739f, -45.8629f)); 
  data[13].mAabb = Aabb(Vector3(-5.02422f, -23.8864f, 9.34145f), Vector3(-2.12088f, -21.0267f, 12.9408f)); 
  data[14].mAabb = Aabb(Vector3(-24.3579f, -23.6836f, 19.3739f), Vector3(-18.5198f, -17.0474f, 22.2886f)); 
  data[15].mAabb = Aabb(Vector3(-7.04344f, 17.2069f, -30.1589f), Vector3(-0.336968f, 19.1036f, -29.6678f)); 
  data[16].mAabb = Aabb(Vector3(-28.4519f, 0.866008f, 39.8841f), Vector3(-24.7786f, 6.19753f, 43.3156f)); 
  data[17].mAabb = Aabb(Vector3(-1.26574f, -23.0803f, -2.05438f), Vector3(6.19383f, -15.9953f, -1.54606f)); 
  data[18].mAabb = Aabb(Vector3(38.4548f, -22.1602f, 6.50609f), Vector3(46.2837f, -16.6837f, 8.76394f)); 
  data[19].mAabb = Aabb(Vector3(11.0474f, 7.67279f, -24.7644f), Vector3(14.7943f, 13.4654f, -19.9747f)); 
  data[20].mAabb = Aabb(Vector3(-18.6971f, -38.2186f, 4.65808f), Vector3(-14.5554f, -36.1495f, 5.72538f)); 
  data[21].mAabb = Aabb(Vector3(-28.6438f, 4.87255f, 34.2021f), Vector3(-22.3948f, 7.46601f, 41.7679f)); 
  data[22].mAabb = Aabb(Vector3(-45.2044f, 4.04864f, -15.5173f), Vector3(-40.8716f, 11.3299f, -7.97194f)); 
  data[23].mAabb = Aabb(Vector3(-35.1356f, 14.4363f, -34.0012f), Vector3(-33.3041f, 15.0889f, -28.5465f)); 
  data[24].mAabb = Aabb(Vector3(4.75477f, 12.4614f, 42.5341f), Vector3(9.4524f, 14.8916f, 46.2583f)); 
  data[25].mAabb = Aabb(Vector3(-11.8059f, -28.4825f, 21.988f), Vector3(-5.03424f, -23.982f, 25.3742f)); 
  data[26].mAabb = Aabb(Vector3(10.7418f, -14.6522f, 0.941468f), Vector3(17.0097f, -12.7043f, 4.86975f)); 
  data[27].mAabb = Aabb(Vector3(-5.70472f, 23.6922f, 17.1506f), Vector3(0.824772f, 29.652f, 19.8217f)); 
  data[28].mAabb = Aabb(Vector3(-28.5403f, 20.7471f, -25.9206f), Vector3(-26.388f, 23.7546f, -18.5767f)); 
  data[29].mAabb = Aabb(Vector3(17.6626f, 11.086f, 0.152338f), Vector3(18.9498f, 13.4751f, 6.27779f)); 
  data[30].mAabb = Aabb(Vector3(17.9386f, 38.4884f, 15.2297f), Vector3(19.1089f, 39.5585f, 18.8535f)); 
  data[31].mAabb = Aabb(Vector3(-24.0646f, 38.6089f, 16.348f), Vector3(-16.207f, 40.0099f, 19.3079f)); 
  data[32].mAabb = Aabb(Vector3(20.8729f, 25.1657f, -29.9977f), Vector3(21.9653f, 29.7838f, -26.2522f)); 
  data[33].mAabb = Aabb(Vector3(2.08947f, -22.6965f, 25.5852f), Vector3(3.66982f, -22.2749f, 31.9414f)); 
  data[34].mAabb = Aabb(Vector3(-8.18249f, 31.1115f, 19.87f), Vector3(-4.78605f, 32.4422f, 23.5967f)); 
  data[35].mAabb = Aabb(Vector3(1.88973f, -2.74051f, -51.9433f), Vector3(7.91012f, -1.08849f, -44.3149f)); 
  data[36].mAabb = Aabb(Vector3(-29.8008f, -34.0283f, 20.2484f), Vector3(-24.1441f, -27.4555f, 25.6652f)); 
  data[37].mAabb = Aabb(Vector3(-11.2743f, 23.1183f, -17.3278f), Vector3(-8.80117f, 28.3132f, -13.8316f)); 
  data[38].mAabb = Aabb(Vector3(-42.7609f, -11.4771f, 13.4721f), Vector3(-40.0874f, -9.42799f, 15.4479f)); 
  data[39].mAabb = Aabb(Vector3(32.5378f, 3.93593f, 32.7767f), Vector3(36.4368f, 6.8439f, 38.749f)); 
  data[40].mAabb = Aabb(Vector3(-26.0045f, -23.0696f, 30.1096f), Vector3(-25.1429f, -21.1722f, 36.7604f)); 
  data[41].mAabb = Aabb(Vector3(7.5009f, -30.612f, 24.2596f), Vector3(10.7913f, -23.6636f, 25.6426f)); 
  data[42].mAabb = Aabb(Vector3(-23.0616f, 2.83854f, 4.42938f), Vector3(-22.0594f, 3.71333f, 6.49518f)); 
  data[43].mAabb = Aabb(Vector3(-0.633688f, 1.8054f, 0.226731f), Vector3(3.65712f, 8.17324f, 3.85511f)); 
  data[44].mAabb = Aabb(Vector3(-15.6137f, -19.2225f, -45.7907f), Vector3(-8.53428f, -15.3198f, -43.1553f)); 
  data[45].mAabb = Aabb(Vector3(26.9269f, 2.40761f, -40.5249f), Vector3(30.6003f, 9.73104f, -39.5233f)); 
  data[46].mAabb = Aabb(Vector3(-26.2651f, -30.3374f, 28.6359f), Vector3(-25.1579f, -24.703f, 29.5494f)); 
  data[47].mAabb = Aabb(Vector3(11.4672f, 3.10139f, -17.4302f), Vector3(15.7638f, 7.7677f, -14.9557f)); 
  data[48].mAabb = Aabb(Vector3(-32.1496f, -13.6151f, 9.47378f), Vector3(-29.6862f, -8.31045f, 15.1676f)); 
  data[49].mAabb = Aabb(Vector3(4.4518f, -25.637f, -41.1696f), Vector3(8.59486f, -23.1676f, -35.608f)); 
  data[50].mAabb = Aabb(Vector3(-18.5011f, -1.67145f, -9.46005f), Vector3(-11.2607f, -0.14561f, -5.99008f)); 
  data[51].mAabb = Aabb(Vector3(-47.2378f, -6.49434f, 10.9224f), Vector3(-45.1764f, 0.763218f, 11.4043f)); 
  data[52].mAabb = Aabb(Vector3(23.1286f, -28.4231f, 12.4048f), Vector3(25.2542f, -21.7289f, 17.8767f)); 
  data[53].mAabb = Aabb(Vector3(-10.5953f, 22.9905f, -23.6963f), Vector3(-6.77952f, 28.0931f, -16.1746f)); 
  data[54].mAabb = Aabb(Vector3(-18.7392f, -38.5037f, 3.64717f), Vector3(-10.8092f, -35.6395f, 7.93334f)); 
  data[55].mAabb = Aabb(Vector3(-11.3724f, 39.8881f, -2.20307f), Vector3(-10.7089f, 45.8349f, 1.71408f)); 
  data[56].mAabb = Aabb(Vector3(-35.6481f, -18.9195f, -35.4357f), Vector3(-30.3854f, -10.9603f, -33.1575f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-7.82998f, -47.5816f, 29.2474f), Vector3(-9.283f, 14.1376f, 22.617f), Vector3(-9.283f, 7.20481f, -41.9175f), Vector3(-7.82998f, -54.5144f, -35.2871f), Vector3(30.7173f, -21.0468f, -2.64916f), Vector3(30.6159f, -16.741f, -3.11173f), Vector3(30.6159f, -17.5043f, -10.2171f), Vector3(30.7173f, -21.8101f, -9.75449f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(-10.9697f, 14.154f, -36.442f), Vector3(-9.91375f, 15.7063f, -34.2679f)); 
  data[1].mAabb = Aabb(Vector3(-6.47128f, -49.043f, -7.14875f), Vector3(-3.90171f, -41.2111f, -6.49663f)); 
  data[2].mAabb = Aabb(Vector3(-50.5744f, -4.70876f, -2.57904f), Vector3(-45.3146f, -2.56804f, -1.0539f)); 
  data[3].mAabb = Aabb(Vector3(-0.258215f, 15.2748f, 35.2257f), Vector3(3.84495f, 18.7169f, 37.4728f)); 
  data[4].mAabb = Aabb(Vector3(-13.0221f, 8.54901f, -23.9689f), Vector3(-10.5239f, 11.2872f, -19.1216f)); 
  data[5].mAabb = Aabb(Vector3(-27.9835f, 5.06864f, -20.0626f), Vector3(-25.6345f, 9.84744f, -13.1485f)); 
  data[6].mAabb = Aabb(Vector3(-17.5837f, -0.36625f, -18.6016f), Vector3(-15.2715f, 7.15758f, -14.8016f)); 
  data[7].mAabb = Aabb(Vector3(-8.36117f, 15.3846f, 16.4929f), Vector3(-3.12822f, 20.9601f, 24.0072f)); 
  data[8].mAabb = Aabb(Vector3(-32.7827f, -6.60678f, -19.3845f), Vector3(-26.2975f, -4.87474f, -13.0612f)); 
  data[9].mAabb = Aabb(Vector3(45.2687f, 7.52919f, 7.94637f), Vector3(48.1368f, 12.1426f, 13.2872f)); 
  data[10].mAabb = Aabb(Vector3(9.32966f, 26.2562f, 30.5959f), Vector3(16.7911f, 27.3059f, 35.8947f)); 
  data[11].mAabb = Aabb(Vector3(-2.19622f, 16.3312f, -19.7547f), Vector3(-0.263089f, 16.7699f, -13.7412f)); 
  data[12].mAabb = Aabb(Vector3(11.6083f, 1.33166f, -18.3549f), Vector3(13.2314f, 4.56505f, -13.4121f)); 
  data[13].mAabb = Aabb(Vector3(1.39734f, -11.3085f, -39.7627f), Vector3(2.5602f, -8.1716f, -37.5561f)); 
  data[14].mAabb = Aabb(Vector3(-4.03149f, -32.0474f, -36.188f), Vector3(3.65748f, -24.0796f, -34.1009f)); 
  data[15].mAabb = Aabb(Vector3(-11.3226f, -34.0238f, -23.3075f), Vector3(-4.80423f, -30.9504f, -21.4335f)); 
  data[16].mAabb = Aabb(Vector3(-11.4445f, 22.9049f, -25.1686f), Vector3(-10.7511f, 24.0744f, -19.4563f)); 
  data[17].mAabb = Aabb(Vector3(-6.66178f, -22.2188f, -26.7038f), Vector3(-0.665743f, -14.7012f, -21.8714f)); 
  data[18].mAabb = Aabb(Vector3(34.3968f, -14.5342f, 25.399f), Vector3(37.0141f, -13.1782f, 32.4945f)); 
  data[19].mAabb = Aabb(Vector3(-7.34934f, -19.8784f, 10.4029f), Vector3(-4.70972f, -18.4216f, 11.967f)); 
  data[20].mAabb = Aabb(Vector3(1.92691f, -6.92532f, -32.3696f), Vector3(6.4485f, -3.43981f, -25.633f)); 
  data[21].mAabb = Aabb(Vector3(-1.04298f, -30.5451f, 3.57896f), Vector3(1.87311f, -26.2274f, 7.08604f)); 
  data[22].mAabb = Aabb(Vector3(-15.4956f, 8.18265f, 41.7506f), Vector3(-8.64041f, 9.66836f, 44.5744f)); 
  data[23].mAabb = Aabb(Vector3(0.767647f, -43.9404f, 4.22311f), Vector3(6.58717f, -41.0352f, 8.10941f)); 
  data[24].mAabb = Aabb(Vector3(-3.33966f, -12.8529f, -4.65207f), Vector3(1.45701f, -11.9711f, -2.85068f)); 
  data[25].mAabb = Aabb(Vector3(-33.6871f, -2.02371f, -15.2107f), Vector3(-29.8223f, -1.3528f, -7.65277f)); 
  data[26].mAabb = Aabb(Vector3(-44.538f, -4.07407f, -5.61164f), Vector3(-36.8469f, 1.06077f, -4.20942f)); 
  data[27].mAabb = Aabb(Vector3(-22.0933f, -6.53766f, 26.6824f), Vector3(-18.76f, -4.91185f, 32.8044f)); 
  data[28].mAabb = Aabb(Vector3(18.215f, -40.0205f, -14.0004f), Vector3(21.2589f, -38.9154f, -7.29253f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-34.0024f, -58.4708f, -32.0353f), Vector3(22.0242f, -61.1318f, 64.7054f), Vector3(22.0242f, 48.9054f, 67.7321f), Vector3(-34.0024f, 51.5664f, -29.0086f), Vector3(25.5827f, -7.25537f, -5.09983f), Vector3(29.5666f, -7.44458f, 1.77911f), Vector3(29.5666f, -1.24154f, 1.94973f), Vector3(25.5827f, -1.05232f, -4.92921f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(74); 
  data[0].mAabb = Aabb(Vector3(20.6453f, 4.15073f, 38.8438f), Vector3(25.3142f, 5.59308f, 43.0606f)); 
  data[1].mAabb = Aabb(Vector3(-25.2262f, -28.6435f, -15.9425f), Vector3(-24.4486f, -20.9004f, -8.75663f)); 
  data[2].mAabb = Aabb(Vector3(-14.3051f, 9.4298f, -29.4656f), Vector3(-13.1464f, 17.3059f, -28.5338f)); 
  data[3].mAabb = Aabb(Vector3(16.2799f, 40.2115f, 9.26404f), Vector3(23.6272f, 45.5516f, 12.6371f)); 
  data[4].mAabb = Aabb(Vector3(-6.64019f, -17.2703f, 5.69875f), Vector3(-5.46852f, -12.093f, 12.4184f)); 
  data[5].mAabb = Aabb(Vector3(-45.3858f, 1.08812f, -4.07646f), Vector3(-38.6194f, 3.08735f, -3.64886f)); 
  data[6].mAabb = Aabb(Vector3(18.3935f, -30.9109f, -17.7321f), Vector3(22.1532f, -27.5532f, -13.484f)); 
  data[7].mAabb = Aabb(Vector3(-2.74192f, -27.6466f, 37.1315f), Vector3(2.46296f, -22.9522f, 40.7112f)); 
  data[8].mAabb = Aabb(Vector3(-43.0524f, -18.7653f, -15.2572f), Vector3(-37.177f, -12.0953f, -8.43636f)); 
  data[9].mAabb = Aabb(Vector3(7.20044f, -5.66183f, 35.7661f), Vector3(8.66852f, -2.29763f, 39.9133f)); 
  data[10].mAabb = Aabb(Vector3(5.82595f, -8.53765f, -16.3496f), Vector3(10.3624f, -6.16639f, -8.81164f)); 
  data[11].mAabb = Aabb(Vector3(-29.9109f, -3.71227f, -35.1942f), Vector3(-22.5158f, 0.0571332f, -33.5751f)); 
  data[12].mAabb = Aabb(Vector3(24.0041f, -19.9279f, -28.9376f), Vector3(27.341f, -15.0906f, -23.334f)); 
  data[13].mAabb = Aabb(Vector3(8.6046f, 17.1271f, -35.7596f), Vector3(9.84075f, 20.1324f, -27.9971f)); 
  data[14].mAabb = Aabb(Vector3(-8.51537f, 34.4291f, -32.2567f), Vector3(-1.59598f, 41.0206f, -24.3703f)); 
  data[15].mAabb = Aabb(Vector3(-28.714f, 30.55f, 18.1377f), Vector3(-23.9872f, 34.8891f, 19.3031f)); 
  data[16].mAabb = Aabb(Vector3(6.83226f, -29.8468f, 23.9493f), Vector3(12.8568f, -26.6341f, 30.3981f)); 
  data[17].mAabb = Aabb(Vector3(4.73663f, 26.9062f, 30.2306f), Vector3(11.8986f, 31.5331f, 32.2621f)); 
  data[18].mAabb = Aabb(Vector3(9.02342f, -9.55151f, -47.4178f), Vector3(14.148f, -5.06936f, -42.4599f)); 
  data[19].mAabb = Aabb(Vector3(16.995f, -18.0586f, -0.558006f), Vector3(20.1871f, -12.6786f, 7.07553f)); 
  data[20].mAabb = Aabb(Vector3(37.1086f, 4.34185f, -18.0687f), Vector3(41.1656f, 5.47269f, -14.5818f)); 
  data[21].mAabb = Aabb(Vector3(-10.6598f, -37.0523f, 17.4123f), Vector3(-6.20985f, -33.5134f, 21.255f)); 
  data[22].mAabb = Aabb(Vector3(-1.30136f, -27.3687f, -17.06f), Vector3(-0.431675f, -22.613f, -10.0356f)); 
  data[23].mAabb = Aabb(Vector3(11.4618f, -2.13961f, 41.8547f), Vector3(14.3781f, 3.2775f, 47.7991f)); 
  data[24].mAabb = Aabb(Vector3(37.086f, 0.514499f, 0.779257f), Vector3(40.3776f, 1.57298f, 5.97927f)); 
  data[25].mAabb = Aabb(Vector3(-11.3194f, 35.0558f, -11.2246f), Vector3(-6.77277f, 39.7355f, -9.07181f)); 
  data[26].mAabb = Aabb(Vector3(15.9207f, -35.9076f, 23.4507f), Vector3(17.9577f, -33.5282f, 27.1913f)); 
  data[27].mAabb = Aabb(Vector3(-37.9984f, -37.2498f, -8.40033f), Vector3(-33.3376f, -31.4312f, -4.56227f)); 
  data[28].mAabb = Aabb(Vector3(26.0416f, 1.52249f, 17.2571f), Vector3(33.4052f, 3.20095f, 19.229f)); 
  data[29].mAabb = Aabb(Vector3(-14.3717f, -36.8105f, -25.4254f), Vector3(-13.5556f, -33.226f, -17.9489f)); 
  data[30].mAabb = Aabb(Vector3(-34.9797f, 0.885901f, -28.8219f), Vector3(-31.6925f, 7.2208f, -24.3509f)); 
  data[31].mAabb = Aabb(Vector3(-12.5391f, -3.83514f, -43.9249f), Vector3(-6.66578f, 3.60681f, -39.9867f)); 
  data[32].mAabb = Aabb(Vector3(-17.6446f, 16.925f, -10.131f), Vector3(-13.1692f, 24.0473f, -3.33939f)); 
  data[33].mAabb = Aabb(Vector3(-1.91155f, 29.1489f, 31.5136f), Vector3(3.05444f, 35.4551f, 38.8384f)); 
  data[34].mAabb = Aabb(Vector3(-15.7921f, -25.2836f, -28.4617f), Vector3(-13.968f, -17.7593f, -23.0958f)); 
  data[35].mAabb = Aabb(Vector3(3.76701f, 39.5193f, -10.1419f), Vector3(5.25759f, 40.1473f, -6.77558f)); 
  data[36].mAabb = Aabb(Vector3(-0.939481f, -45.5432f, 1.17689f), Vector3(3.59649f, -38.7906f, 3.07987f)); 
  data[37].mAabb = Aabb(Vector3(-36.6238f, 27.084f, 4.8129f), Vector3(-31.999f, 31.7696f, 7.20806f)); 
  data[38].mAabb = Aabb(Vector3(-8.68184f, 19.725f, -23.5137f), Vector3(-5.97287f, 20.5539f, -16.7244f)); 
  data[39].mAabb = Aabb(Vector3(-16.1218f, 36.545f, -2.03355f), Vector3(-11.4321f, 38.293f, 3.67186f)); 
  data[40].mAabb = Aabb(Vector3(21.2415f, -26.1422f, -20.5787f), Vector3(26.291f, -24.6461f, -16.3702f)); 
  data[41].mAabb = Aabb(Vector3(-4.14653f, 11.3189f, 44.6413f), Vector3(-2.47851f, 17.0913f, 46.5515f)); 
  data[42].mAabb = Aabb(Vector3(15.559f, -1.51194f, -35.018f), Vector3(21.6824f, -0.994808f, -29.9245f)); 
  data[43].mAabb = Aabb(Vector3(-19.8068f, 24.1216f, 18.1653f), Vector3(-15.8175f, 31.2191f, 25.4792f)); 
  data[44].mAabb = Aabb(Vector3(10.8925f, -12.9296f, -9.77515f), Vector3(18.2465f, -12.2021f, -3.55668f)); 
  data[45].mAabb = Aabb(Vector3(-7.08678f, -8.93235f, -14.3204f), Vector3(-4.99107f, -7.53779f, -13.415f)); 
  data[46].mAabb = Aabb(Vector3(-35.8177f, -8.04765f, -30.0788f), Vector3(-28.8333f, -4.64518f, -25.7309f)); 
  data[47].mAabb = Aabb(Vector3(-3.17003f, 1.04377f, -21.8689f), Vector3(-2.06865f, 1.98536f, -18.2783f)); 
  data[48].mAabb = Aabb(Vector3(-11.9679f, 38.2504f, 5.22113f), Vector3(-4.75647f, 43.151f, 10.5754f)); 
  data[49].mAabb = Aabb(Vector3(-7.40572f, -8.39311f, 40.7757f), Vector3(-4.34837f, -1.31414f, 48.6467f)); 
  data[50].mAabb = Aabb(Vector3(4.53391f, -16.1599f, 42.2813f), Vector3(11.4422f, -10.0104f, 44.6952f)); 
  data[51].mAabb = Aabb(Vector3(-30.6786f, -8.18716f, 35.1063f), Vector3(-22.7454f, -5.67534f, 42.6944f)); 
  data[52].mAabb = Aabb(Vector3(-4.94087f, -7.88214f, -16.788f), Vector3(0.404804f, -3.55075f, -9.18902f)); 
  data[53].mAabb = Aabb(Vector3(-2.00475f, -30.3669f, -35.3408f), Vector3(5.42514f, -27.2451f, -34.203f)); 
  data[54].mAabb = Aabb(Vector3(15.5292f, -29.041f, -4.23849f), Vector3(16.0535f, -28.3065f, 0.333427f)); 
  data[55].mAabb = Aabb(Vector3(0.551555f, 21.8359f, -19.1171f), Vector3(3.28859f, 29.3855f, -14.1411f)); 
  data[56].mAabb = Aabb(Vector3(31.1506f, -27.3546f, 0.397467f), Vector3(38.6835f, -25.6224f, 5.89715f)); 
  data[57].mAabb = Aabb(Vector3(-14.1959f, 33.8175f, -23.5776f), Vector3(-9.05249f, 39.6268f, -19.5588f)); 
  data[58].mAabb = Aabb(Vector3(-33.3672f, -1.52628f, -24.1382f), Vector3(-28.8275f, -0.275055f, -20.2403f)); 
  data[59].mAabb = Aabb(Vector3(-7.85855f, 45.0383f, -1.50421f), Vector3(-2.84316f, 51.8408f, 3.45296f)); 
  data[60].mAabb = Aabb(Vector3(2.43152f, -3.95836f, -34.6962f), Vector3(9.07026f, 1.33489f, -28.1776f)); 
  data[61].mAabb = Aabb(Vector3(-4.87877f, 4.22048f, -9.79619f), Vector3(-0.0206411f, 7.00738f, -4.69962f)); 
  data[62].mAabb = Aabb(Vector3(22.4316f, 21.0267f, 31.7918f), Vector3(30.3839f, 24.2963f, 36.1826f)); 
  data[63].mAabb = Aabb(Vector3(-19.3488f, 1.94397f, -3.09715f), Vector3(-16.1806f, 7.4135f, 1.63272f)); 
  data[64].mAabb = Aabb(Vector3(-49.8751f, 3.5894f, -7.04492f), Vector3(-42.4654f, 11.1053f, -1.71084f)); 
  data[65].mAabb = Aabb(Vector3(16.8784f, 40.5798f, -0.754079f), Vector3(18.8693f, 43.9608f, 4.10939f)); 
  data[66].mAabb = Aabb(Vector3(-32.8119f, 3.00377f, -7.98456f), Vector3(-29.7968f, 7.94679f, -4.73726f)); 
  data[67].mAabb = Aabb(Vector3(-10.6556f, -24.5422f, -45.9693f), Vector3(-9.75787f, -18.9456f, -41.6669f)); 
  data[68].mAabb = Aabb(Vector3(38.4202f, -29.5524f, -9.64819f), Vector3(41.2303f, -26.5575f, -3.15441f)); 
  data[69].mAabb = Aabb(Vector3(27.5143f, 19.9272f, -30.2833f), Vector3(32.2063f, 26.1948f, -23.973f)); 
  data[70].mAabb = Aabb(Vector3(-23.357f, 11.7309f, -37.0175f), Vector3(-17.6052f, 17.9976f, -36.3275f)); 
  data[71].mAabb = Aabb(Vector3(3.78136f, -6.1247f, -42.9878f), Vector3(11.3404f, -1.50361f, -39.7305f)); 
  data[72].mAabb = Aabb(Vector3(33.7184f, 24.0571f, -26.6639f), Vector3(34.4308f, 31.2101f, -20.4496f)); 
  data[73].mAabb = Aabb(Vector3(-6.62942f, -46.5281f, -17.6725f), Vector3(-2.85862f, -40.4335f, -15.0073f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-11.1103f, -133.649f, -24.5751f), Vector3(-54.0817f, -41.5552f, 122.559f), Vector3(-54.0817f, 109.289f, 28.1429f), Vector3(-11.1103f, 17.195f, -118.991f), Vector3(46.1327f, -4.99779f, 17.2095f), Vector3(44.9179f, -2.39427f, 21.3691f), Vector3(44.9179f, 1.1593f, 19.1448f), Vector3(46.1327f, -1.44422f, 14.9853f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(40); 
  data[0].mAabb = Aabb(Vector3(-9.41656f, 29.3368f, -3.49587f), Vector3(-5.49106f, 31.4727f, 4.02517f)); 
  data[1].mAabb = Aabb(Vector3(21.5106f, -5.4028f, 13.7586f), Vector3(25.5498f, -4.82931f, 14.809f)); 
  data[2].mAabb = Aabb(Vector3(2.35732f, -10.8124f, -50.5851f), Vector3(4.27004f, -9.69288f, -46.5355f)); 
  data[3].mAabb = Aabb(Vector3(9.64013f, -10.7645f, 11.7277f), Vector3(10.1744f, -5.64755f, 13.7219f)); 
  data[4].mAabb = Aabb(Vector3(-11.6991f, 17.8375f, 12.1834f), Vector3(-6.78232f, 24.8731f, 19.3563f)); 
  data[5].mAabb = Aabb(Vector3(25.8332f, -24.6148f, -24.5434f), Vector3(29.2737f, -23.0059f, -23.1583f)); 
  data[6].mAabb = Aabb(Vector3(-4.3446f, 15.3716f, 5.13654f), Vector3(2.53072f, 15.9421f, 6.26368f)); 
  data[7].mAabb = Aabb(Vector3(-35.5791f, 1.31207f, -34.1286f), Vector3(-31.0181f, 8.76284f, -31.3434f)); 
  data[8].mAabb = Aabb(Vector3(-26.7062f, -35.7791f, -2.60946f), Vector3(-24.395f, -33.7379f, -1.90191f)); 
  data[9].mAabb = Aabb(Vector3(5.18603f, -13.5057f, -31.5945f), Vector3(7.3279f, -10.7805f, -30.7142f)); 
  data[10].mAabb = Aabb(Vector3(9.08237f, -30.2011f, -15.3739f), Vector3(12.3584f, -25.7875f, -11.1167f)); 
  data[11].mAabb = Aabb(Vector3(-26.3453f, 19.3477f, 0.284641f), Vector3(-21.9713f, 21.6902f, 3.78453f)); 
  data[12].mAabb = Aabb(Vector3(2.1589f, -44.3712f, -11.1695f), Vector3(4.91008f, -38.2237f, -6.78732f)); 
  data[13].mAabb = Aabb(Vector3(-27.069f, 6.90126f, 37.5758f), Vector3(-22.6603f, 7.73777f, 40.0461f)); 
  data[14].mAabb = Aabb(Vector3(-18.2217f, 6.22971f, -7.53992f), Vector3(-14.2512f, 12.1091f, -0.780341f)); 
  data[15].mAabb = Aabb(Vector3(30.0262f, -7.63232f, -30.8258f), Vector3(32.68f, -4.29827f, -25.6608f)); 
  data[16].mAabb = Aabb(Vector3(23.4189f, 12.7945f, -7.53486f), Vector3(26.7464f, 19.9456f, -3.9408f)); 
  data[17].mAabb = Aabb(Vector3(-23.4467f, 5.55469f, -15.3831f), Vector3(-19.5396f, 8.35458f, -8.57152f)); 
  data[18].mAabb = Aabb(Vector3(-3.2526f, -1.29251f, -46.8005f), Vector3(-2.52463f, 0.310565f, -42.7078f)); 
  data[19].mAabb = Aabb(Vector3(-8.22218f, 7.37858f, -41.7546f), Vector3(-3.92882f, 10.4577f, -33.9334f)); 
  data[20].mAabb = Aabb(Vector3(25.1288f, -9.78144f, -6.26261f), Vector3(26.1794f, -8.14033f, 1.45211f)); 
  data[21].mAabb = Aabb(Vector3(-12.5114f, -33.2995f, -34.8503f), Vector3(-10.5465f, -32.2584f, -33.048f)); 
  data[22].mAabb = Aabb(Vector3(-7.42524f, 39.3848f, -20.2992f), Vector3(-2.84544f, 46.7732f, -12.9375f)); 
  data[23].mAabb = Aabb(Vector3(-25.3078f, 10.2864f, 22.9153f), Vector3(-24.3764f, 14.8309f, 23.4336f)); 
  data[24].mAabb = Aabb(Vector3(32.2583f, 12.727f, 14.9067f), Vector3(35.5288f, 14.4655f, 21.9782f)); 
  data[25].mAabb = Aabb(Vector3(26.2675f, 18.4998f, -16.3484f), Vector3(30.2452f, 25.6311f, -10.0928f)); 
  data[26].mAabb = Aabb(Vector3(-23.4849f, 37.3625f, -11.1347f), Vector3(-19.9727f, 40.0823f, -6.55975f)); 
  data[27].mAabb = Aabb(Vector3(-15.9062f, -6.69595f, -14.0715f), Vector3(-13.8715f, -2.93304f, -8.61053f)); 
  data[28].mAabb = Aabb(Vector3(0.621365f, -24.2379f, 29.9737f), Vector3(1.66848f, -23.2541f, 36.855f)); 
  data[29].mAabb = Aabb(Vector3(27.5018f, 10.1376f, -4.43513f), Vector3(29.068f, 16.652f, -3.91916f)); 
  data[30].mAabb = Aabb(Vector3(-7.50186f, 1.44145f, 29.6557f), Vector3(-2.78822f, 3.0705f, 36.1089f)); 
  data[31].mAabb = Aabb(Vector3(-3.54697f, -18.6494f, 34.3121f), Vector3(2.58428f, -10.9157f, 35.8153f)); 
  data[32].mAabb = Aabb(Vector3(-3.93485f, -43.5349f, 8.44261f), Vector3(0.335539f, -37.7578f, 9.77594f)); 
  data[33].mAabb = Aabb(Vector3(-31.8785f, -8.06422f, -15.1783f), Vector3(-30.5923f, -6.48133f, -12.7449f)); 
  data[34].mAabb = Aabb(Vector3(-38.3389f, 12.6855f, -0.726118f), Vector3(-36.5705f, 19.3926f, 1.26082f)); 
  data[35].mAabb = Aabb(Vector3(-21.4634f, -18.7588f, -6.69413f), Vector3(-18.0711f, -14.8853f, -6.1f)); 
  data[36].mAabb = Aabb(Vector3(-15.7599f, -3.70525f, 16.6493f), Vector3(-14.029f, -0.538659f, 18.4701f)); 
  data[37].mAabb = Aabb(Vector3(-6.23039f, -21.3161f, -8.43511f), Vector3(-5.1464f, -15.0114f, -0.954422f)); 
  data[38].mAabb = Aabb(Vector3(19.9445f, 6.07089f, -35.2849f), Vector3(23.5481f, 11.9401f, -29.1411f)); 
  data[39].mAabb = Aabb(Vector3(18.7603f, -32.3266f, 22.622f), Vector3(20.7273f, -29.706f, 26.1954f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(9.65319f, 51.3159f, 168.363f), Vector3(-73.5191f, 150.053f, -21.9577f), Vector3(-73.5191f, -56.917f, -129.332f), Vector3(9.65319f, -155.654f, 60.9883f), Vector3(58.0621f, 15.8448f, -6.60168f), Vector3(55.9437f, 18.3595f, -11.449f), Vector3(55.9437f, 10.3289f, -15.6153f), Vector3(58.0621f, 7.8141f, -10.7679f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(65); 
  data[0].mAabb = Aabb(Vector3(-39.3131f, 24.0188f, -6.60614f), Vector3(-33.091f, 27.6015f, -2.33713f)); 
  data[1].mAabb = Aabb(Vector3(-23.7638f, -14.7114f, -18.7234f), Vector3(-16.978f, -12.6483f, -17.5403f)); 
  data[2].mAabb = Aabb(Vector3(14.5687f, 5.02396f, -26.67f), Vector3(21.6061f, 11.5249f, -25.1147f)); 
  data[3].mAabb = Aabb(Vector3(11.4018f, 5.55213f, 20.0665f), Vector3(13.7452f, 10.6067f, 27.5316f)); 
  data[4].mAabb = Aabb(Vector3(24.4002f, -36.5769f, 18.6132f), Vector3(30.1378f, -33.8246f, 26.3759f)); 
  data[5].mAabb = Aabb(Vector3(9.89005f, 17.749f, 37.366f), Vector3(14.156f, 19.1299f, 42.6092f)); 
  data[6].mAabb = Aabb(Vector3(-3.32913f, 3.83929f, 43.8936f), Vector3(0.644847f, 7.26658f, 49.9884f)); 
  data[7].mAabb = Aabb(Vector3(19.7297f, -19.7814f, 30.6152f), Vector3(26.563f, -13.0826f, 32.0703f)); 
  data[8].mAabb = Aabb(Vector3(14.6422f, 25.35f, -14.6811f), Vector3(17.6059f, 28.9869f, -11.1587f)); 
  data[9].mAabb = Aabb(Vector3(-35.76f, -10.2255f, -20.3215f), Vector3(-30.6957f, -8.72957f, -18.1147f)); 
  data[10].mAabb = Aabb(Vector3(-19.3573f, -18.5996f, -17.3472f), Vector3(-18.016f, -11.5846f, -15.4078f)); 
  data[11].mAabb = Aabb(Vector3(15.7721f, 2.60046f, -22.3787f), Vector3(18.5587f, 3.52975f, -14.8742f)); 
  data[12].mAabb = Aabb(Vector3(3.14484f, -6.77944f, 10.0059f), Vector3(8.69648f, -1.79537f, 16.2072f)); 
  data[13].mAabb = Aabb(Vector3(-32.1991f, -18.7374f, -16.6573f), Vector3(-25.6664f, -11.1456f, -9.05048f)); 
  data[14].mAabb = Aabb(Vector3(-38.49f, -19.281f, 25.7363f), Vector3(-34.3912f, -15.619f, 29.0022f)); 
  data[15].mAabb = Aabb(Vector3(36.5128f, -21.605f, -6.14097f), Vector3(40.581f, -18.8374f, -5.43504f)); 
  data[16].mAabb = Aabb(Vector3(-12.0746f, -36.8636f, 33.7358f), Vector3(-6.91122f, -31.8041f, 36.1816f)); 
  data[17].mAabb = Aabb(Vector3(-46.6334f, 12.6744f, -14.8056f), Vector3(-40.4576f, 19.0167f, -12.5527f)); 
  data[18].mAabb = Aabb(Vector3(33.1092f, -14.4432f, 30.9351f), Vector3(38.4078f, -9.40436f, 33.5181f)); 
  data[19].mAabb = Aabb(Vector3(11.1968f, -13.555f, -31.7272f), Vector3(17.2921f, -9.83079f, -26.89f)); 
  data[20].mAabb = Aabb(Vector3(17.2554f, 22.4985f, -27.0344f), Vector3(21.1778f, 28.3917f, -20.4722f)); 
  data[21].mAabb = Aabb(Vector3(34.3992f, -13.1001f, 8.37636f), Vector3(38.0401f, -7.55422f, 11.7111f)); 
  data[22].mAabb = Aabb(Vector3(15.0024f, 20.847f, 25.1972f), Vector3(22.3143f, 28.0559f, 27.369f)); 
  data[23].mAabb = Aabb(Vector3(-31.391f, 23.8763f, -33.3146f), Vector3(-29.7867f, 24.3809f, -27.0773f)); 
  data[24].mAabb = Aabb(Vector3(-11.1329f, -13.6584f, -31.0344f), Vector3(-3.51604f, -10.3845f, -24.1055f)); 
  data[25].mAabb = Aabb(Vector3(9.60085f, -12.6811f, 23.5877f), Vector3(17.0574f, -6.375f, 27.0602f)); 
  data[26].mAabb = Aabb(Vector3(-4.2079f, -25.3245f, -6.74533f), Vector3(-2.0027f, -21.4268f, -2.07089f)); 
  data[27].mAabb = Aabb(Vector3(-47.8771f, 2.50837f, 8.17939f), Vector3(-46.7145f, 4.37817f, 9.81216f)); 
  data[28].mAabb = Aabb(Vector3(-6.25818f, -10.5928f, -35.5094f), Vector3(-5.40937f, -10.0972f, -34.749f)); 
  data[29].mAabb = Aabb(Vector3(-31.979f, -14.3165f, 15.2119f), Vector3(-31.0583f, -7.74967f, 20.9308f)); 
  data[30].mAabb = Aabb(Vector3(30.6273f, 15.8686f, 27.3847f), Vector3(33.0951f, 16.5838f, 28.2166f)); 
  data[31].mAabb = Aabb(Vector3(-32.2284f, 23.9943f, -26.8147f), Vector3(-26.9933f, 30.0363f, -21.6993f)); 
  data[32].mAabb = Aabb(Vector3(2.41877f, -53.4907f, 2.92735f), Vector3(5.35736f, -45.6394f, 6.1503f)); 
  data[33].mAabb = Aabb(Vector3(-9.00334f, 25.6768f, -41.0577f), Vector3(-5.06161f, 31.2702f, -38.5477f)); 
  data[34].mAabb = Aabb(Vector3(9.16066f, 9.56161f, 24.0749f), Vector3(9.64787f, 17.0864f, 31.6507f)); 
  data[35].mAabb = Aabb(Vector3(29.3912f, 27.6861f, -15.0497f), Vector3(33.1859f, 29.5487f, -8.38293f)); 
  data[36].mAabb = Aabb(Vector3(16.1711f, 6.7375f, 11.7067f), Vector3(23.1443f, 11.8164f, 12.2037f)); 
  data[37].mAabb = Aabb(Vector3(-11.8085f, 17.9348f, 36.6006f), Vector3(-8.98282f, 19.7406f, 41.5687f)); 
  data[38].mAabb = Aabb(Vector3(-21.2137f, 10.5065f, -41.7947f), Vector3(-20.0367f, 11.8695f, -37.3304f)); 
  data[39].mAabb = Aabb(Vector3(-28.7688f, 24.2425f, -30.4451f), Vector3(-25.2782f, 25.5438f, -26.6082f)); 
  data[40].mAabb = Aabb(Vector3(-8.25222f, -18.3873f, 33.9152f), Vector3(-1.04175f, -14.8299f, 40.3074f)); 
  data[41].mAabb = Aabb(Vector3(-20.9461f, -20.6634f, -22.5433f), Vector3(-14.5298f, -13.9008f, -21.5913f)); 
  data[42].mAabb = Aabb(Vector3(11.4543f, -19.476f, -17.9662f), Vector3(17.6064f, -15.1643f, -13.8923f)); 
  data[43].mAabb = Aabb(Vector3(-23.7371f, 6.37488f, -14.0248f), Vector3(-17.3783f, 10.8877f, -6.25512f)); 
  data[44].mAabb = Aabb(Vector3(-15.3173f, 14.9494f, 23.6225f), Vector3(-14.4877f, 18.0005f, 25.7328f)); 
  data[45].mAabb = Aabb(Vector3(20.186f, -42.6141f, -16.7195f), Vector3(20.7697f, -35.7228f, -15.2596f)); 
  data[46].mAabb = Aabb(Vector3(1.29708f, 1.25705f, 44.0844f), Vector3(3.95271f, 3.9739f, 49.621f)); 
  data[47].mAabb = Aabb(Vector3(15.3499f, 33.2822f, 4.89359f), Vector3(16.2377f, 37.127f, 5.67258f)); 
  data[48].mAabb = Aabb(Vector3(30.859f, 7.28574f, -16.0818f), Vector3(32.8914f, 9.89057f, -13.7208f)); 
  data[49].mAabb = Aabb(Vector3(-29.4929f, 9.61381f, 37.4492f), Vector3(-23.2287f, 11.7879f, 38.0285f)); 
  data[50].mAabb = Aabb(Vector3(12.0743f, -34.3677f, -18.7318f), Vector3(14.9108f, -30.2081f, -12.7022f)); 
  data[51].mAabb = Aabb(Vector3(-10.9477f, 12.7678f, 15.0191f), Vector3(-7.5144f, 16.7635f, 20.0964f)); 
  data[52].mAabb = Aabb(Vector3(23.7985f, -37.3718f, -0.797157f), Vector3(31.4947f, -32.5807f, 3.82764f)); 
  data[53].mAabb = Aabb(Vector3(22.4006f, 2.39124f, -20.2999f), Vector3(27.8481f, 10.1702f, -19.3101f)); 
  data[54].mAabb = Aabb(Vector3(32.7596f, -35.2347f, -11.4829f), Vector3(36.9344f, -31.059f, -9.66782f)); 
  data[55].mAabb = Aabb(Vector3(-2.53201f, -26.9417f, -25.0674f), Vector3(3.04861f, -21.818f, -20.8176f)); 
  data[56].mAabb = Aabb(Vector3(14.7629f, 1.17447f, -7.9473f), Vector3(20.5358f, 6.87687f, -3.98052f)); 
  data[57].mAabb = Aabb(Vector3(-11.9742f, -20.2877f, 34.3475f), Vector3(-10.8561f, -16.8458f, 40.562f)); 
  data[58].mAabb = Aabb(Vector3(13.4976f, 21.9488f, 26.3218f), Vector3(21.3563f, 29.076f, 27.9497f)); 
  data[59].mAabb = Aabb(Vector3(22.481f, -44.1378f, -3.25047f), Vector3(24.7405f, -42.0359f, 3.03155f)); 
  data[60].mAabb = Aabb(Vector3(23.4988f, -1.47014f, 10.1323f), Vector3(27.1423f, -0.370838f, 15.134f)); 
  data[61].mAabb = Aabb(Vector3(-12.0873f, 29.9757f, -9.87549f), Vector3(-6.14412f, 35.3775f, -7.45436f)); 
  data[62].mAabb = Aabb(Vector3(33.6455f, -0.476709f, 30.3539f), Vector3(37.5566f, 1.94465f, 37.1701f)); 
  data[63].mAabb = Aabb(Vector3(31.2635f, 20.242f, -10.2248f), Vector3(33.0723f, 27.7717f, -5.28826f)); 
  data[64].mAabb = Aabb(Vector3(-4.79918f, -2.07562f, -22.4895f), Vector3(-4.38202f, -0.971215f, -21.8819f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(-23.9041f, -83.5241f, -126.692f), Vector3(-50.174f, -119.304f, 92.4832f), Vector3(-50.174f, 102.306f, 128.661f), Vector3(-23.9041f, 138.086f, -90.5141f), Vector3(61.6681f, 4.30855f, 8.58639f), Vector3(60.8853f, 3.2424f, 15.1172f), Vector3(60.8853f, 10.7255f, 16.3388f), Vector3(61.6681f, 11.7917f, 9.80801f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(53); 
  data[0].mAabb = Aabb(Vector3(-22.4144f, -2.5267f, -35.9091f), Vector3(-18.9665f, 0.262987f, -28.6223f)); 
  data[1].mAabb = Aabb(Vector3(-32.1114f, 34.8609f, -12.6258f), Vector3(-25.8644f, 36.8459f, -4.7779f)); 
  data[2].mAabb = Aabb(Vector3(0.357713f, 2.99862f, -24.3824f), Vector3(5.47585f, 4.11972f, -17.2622f)); 
  data[3].mAabb = Aabb(Vector3(-11.4133f, -8.59646f, -10.5493f), Vector3(-4.78824f, -4.79945f, -4.87911f)); 
  data[4].mAabb = Aabb(Vector3(-8.70769f, 28.4767f, -35.8678f), Vector3(-3.20105f, 34.4268f, -35.224f)); 
  data[5].mAabb = Aabb(Vector3(2.05586f, 17.5598f, 17.5697f), Vector3(5.65942f, 20.3797f, 19.2315f)); 
  data[6].mAabb = Aabb(Vector3(-42.3548f, -17.0713f, -22.3513f), Vector3(-36.2293f, -9.94738f, -21.392f)); 
  data[7].mAabb = Aabb(Vector3(-24.565f, -21.9635f, -22.4767f), Vector3(-21.9541f, -20.6745f, -16.7251f)); 
  data[8].mAabb = Aabb(Vector3(-23.936f, 18.4469f, -42.311f), Vector3(-16.6527f, 18.9852f, -38.9916f)); 
  data[9].mAabb = Aabb(Vector3(-36.2669f, 1.12572f, 32.1387f), Vector3(-31.467f, 4.05364f, 36.3668f)); 
  data[10].mAabb = Aabb(Vector3(-21.0103f, -9.96395f, 16.1864f), Vector3(-15.8005f, -6.55684f, 19.1581f)); 
  data[11].mAabb = Aabb(Vector3(35.7851f, -5.03668f, 8.43895f), Vector3(40.174f, -0.266449f, 10.7596f)); 
  data[12].mAabb = Aabb(Vector3(-21.1787f, -28.4233f, -3.23756f), Vector3(-19.2734f, -27.4555f, 2.65016f)); 
  data[13].mAabb = Aabb(Vector3(-5.57581f, 32.8422f, -13.3651f), Vector3(-2.69312f, 39.5946f, -7.49362f)); 
  data[14].mAabb = Aabb(Vector3(-18.5115f, -19.2136f, 28.6238f), Vector3(-11.9394f, -17.1759f, 30.3704f)); 
  data[15].mAabb = Aabb(Vector3(42.4174f, 15.8171f, 3.67108f), Vector3(48.1474f, 21.4026f, 8.24045f)); 
  data[16].mAabb = Aabb(Vector3(0.811722f, 6.19481f, -0.504999f), Vector3(4.5025f, 10.8256f, 1.03035f)); 
  data[17].mAabb = Aabb(Vector3(-0.637065f, -31.6461f, 4.41773f), Vector3(0.548982f, -27.6243f, 12.2227f)); 
  data[18].mAabb = Aabb(Vector3(32.1484f, -21.8354f, 25.67f), Vector3(35.5564f, -18.3859f, 28.392f)); 
  data[19].mAabb = Aabb(Vector3(-5.6513f, -15.4948f, -4.3227f), Vector3(-4.71158f, -13.4709f, 1.28391f)); 
  data[20].mAabb = Aabb(Vector3(-22.7372f, -36.2462f, -3.08527f), Vector3(-17.8443f, -28.5834f, 4.08671f)); 
  data[21].mAabb = Aabb(Vector3(2.67496f, 35.2583f, -19.7377f), Vector3(8.14078f, 36.2127f, -15.2115f)); 
  data[22].mAabb = Aabb(Vector3(2.24106f, -27.9884f, -31.3761f), Vector3(5.76901f, -24.8058f, -23.4097f)); 
  data[23].mAabb = Aabb(Vector3(10.2863f, 6.454f, 30.1448f), Vector3(14.3546f, 8.94912f, 33.4744f)); 
  data[24].mAabb = Aabb(Vector3(29.3806f, 2.03329f, 17.088f), Vector3(31.1765f, 4.68845f, 18.9722f)); 
  data[25].mAabb = Aabb(Vector3(-6.16491f, -41.3323f, -12.9075f), Vector3(-4.38579f, -38.3237f, -7.74533f)); 
  data[26].mAabb = Aabb(Vector3(28.5477f, -1.86098f, -28.0719f), Vector3(31.2741f, 1.51111f, -23.5589f)); 
  data[27].mAabb = Aabb(Vector3(2.77772f, 30.6692f, -30.7853f), Vector3(3.44329f, 33.7256f, -23.7412f)); 
  data[28].mAabb = Aabb(Vector3(10.0361f, 8.59928f, -8.89319f), Vector3(16.5969f, 12.5211f, -5.74586f)); 
  data[29].mAabb = Aabb(Vector3(-12.1118f, 37.3482f, 15.5117f), Vector3(-8.39083f, 39.7324f, 18.0766f)); 
  data[30].mAabb = Aabb(Vector3(-23.2737f, -12.4548f, -45.6793f), Vector3(-18.5698f, -6.26691f, -38.8367f)); 
  data[31].mAabb = Aabb(Vector3(10.1144f, -4.67604f, 41.2495f), Vector3(17.0052f, 3.01247f, 48.284f)); 
  data[32].mAabb = Aabb(Vector3(-8.2444f, -21.9463f, 29.3318f), Vector3(-1.42474f, -20.4357f, 36.5123f)); 
  data[33].mAabb = Aabb(Vector3(10.9273f, -20.9203f, -34.4915f), Vector3(12.7314f, -20.2459f, -30.3354f)); 
  data[34].mAabb = Aabb(Vector3(41.7967f, 17.6174f, -1.62703f), Vector3(45.7329f, 22.0397f, -0.112321f)); 
  data[35].mAabb = Aabb(Vector3(0.393797f, -28.1463f, -7.86981f), Vector3(8.36318f, -20.424f, -1.98627f)); 
  data[36].mAabb = Aabb(Vector3(-14.3233f, 33.1075f, -33.8354f), Vector3(-12.1016f, 36.6765f, -27.5803f)); 
  data[37].mAabb = Aabb(Vector3(-7.7152f, -12.8706f, -42.9682f), Vector3(-2.72115f, -7.00302f, -40.0454f)); 
  data[38].mAabb = Aabb(Vector3(-4.464f, -41.8413f, -19.2269f), Vector3(3.03084f, -34.6097f, -12.4741f)); 
  data[39].mAabb = Aabb(Vector3(22.2875f, -17.1774f, -20.0785f), Vector3(26.2884f, -13.1617f, -14.9341f)); 
  data[40].mAabb = Aabb(Vector3(-14.2922f, 6.40431f, 3.87663f), Vector3(-10.3634f, 13.8625f, 10.8301f)); 
  data[41].mAabb = Aabb(Vector3(-17.3755f, -25.5685f, -41.6189f), Vector3(-10.1857f, -21.8708f, -39.0556f)); 
  data[42].mAabb = Aabb(Vector3(17.6905f, 37.9226f, -22.0935f), Vector3(22.3081f, 40.1445f, -18.319f)); 
  data[43].mAabb = Aabb(Vector3(-43.4553f, -12.168f, 19.5224f), Vector3(-39.9374f, -8.68741f, 20.0808f)); 
  data[44].mAabb = Aabb(Vector3(-24.6301f, -0.785291f, -2.81102f), Vector3(-19.9703f, 5.22999f, 0.964181f)); 
  data[45].mAabb = Aabb(Vector3(15.4112f, -5.40735f, 19.9952f), Vector3(23.2693f, -3.25945f, 25.5768f)); 
  data[46].mAabb = Aabb(Vector3(-19.3742f, -6.86846f, -37.5509f), Vector3(-13.7453f, -0.977032f, -31.6743f)); 
  data[47].mAabb = Aabb(Vector3(4.77996f, 36.853f, -21.3247f), Vector3(7.57266f, 42.0115f, -20.2961f)); 
  data[48].mAabb = Aabb(Vector3(-4.14422f, -27.4916f, -13.6536f), Vector3(-1.25735f, -21.3213f, -6.75649f)); 
  data[49].mAabb = Aabb(Vector3(-23.9973f, 12.7415f, 16.6991f), Vector3(-20.7815f, 17.4414f, 19.0314f)); 
  data[50].mAabb = Aabb(Vector3(-24.0998f, -10.1211f, -42.4546f), Vector3(-18.29f, -5.30399f, -35.5881f)); 
  data[51].mAabb = Aabb(Vector3(25.4417f, 32.3471f, 2.66739f), Vector3(29.5363f, 38.5943f, 5.6039f)); 
  data[52].mAabb = Aabb(Vector3(21.9653f, 23.5996f, -6.04857f), Vector3(27.1456f, 30.3075f, -3.57038f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(81.852f, 77.4977f, 52.3495f), Vector3(-63.1414f, 78.1371f, 21.859f), Vector3(-63.7884f, -68.5569f, 21.859f), Vector3(81.2051f, -69.1964f, 52.3495f), Vector3(29.8886f, 7.56866f, -42.9368f), Vector3(22.2057f, 7.60255f, -44.5524f), Vector3(22.1776f, 1.22216f, -44.5524f), Vector3(29.8605f, 1.18828f, -42.9368f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void DynamicAabbTreeFrustumCastFuzzTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(41); 
  data[0].mAabb = Aabb(Vector3(28.9928f, -17.9434f, 29.1508f), Vector3(34.1135f, -17.4483f, 34.2089f)); 
  data[1].mAabb = Aabb(Vector3(-32.7598f, 1.40172f, 28.9171f), Vector3(-31.7282f, 4.16427f, 30.2961f)); 
  data[2].mAabb = Aabb(Vector3(25.0094f, 23.691f, -8.30698f), Vector3(31.1517f, 30.4318f, -7.18332f)); 
  data[3].mAabb = Aabb(Vector3(-1.27917f, -21.5195f, -35.868f), Vector3(5.68939f, -15.9349f, -34.9209f)); 
  data[4].mAabb = Aabb(Vector3(-45.7214f, 15.2638f, -18.423f), Vector3(-40.8468f, 16.0117f, -10.5589f)); 
  data[5].mAabb = Aabb(Vector3(-18.3754f, -15.1896f, -30.1509f), Vector3(-13.52f, -8.19275f, -26.3926f)); 
  data[6].mAabb = Aabb(Vector3(1.08971f, 8.93214f, 12.8797f), Vector3(7.28428f, 13.4475f, 18.0718f)); 
  data[7].mAabb = Aabb(Vector3(32.27f, 11.9703f, -10.5209f), Vector3(34.9108f, 15.2786f, -8.29827f)); 
  data[8].mAabb = Aabb(Vector3(2.80735f, 40.9039f, -17.6776f), Vector3(9.20533f, 48.7986f, -10.1464f)); 
  data[9].mAabb = Aabb(Vector3(-31.4432f, -41.9582f, 2.76584f), Vector3(-25.3411f, -36.4814f, 6.7695f)); 
  data[10].mAabb = Aabb(Vector3(-18.2796f, 35.2084f, -18.9437f), Vector3(-14.898f, 40.7048f, -17.3918f)); 
  data[11].mAabb = Aabb(Vector3(22.2479f, 18.6454f, -2.6362f), Vector3(25.1864f, 20.9205f, -1.67421f)); 
  data[12].mAabb = Aabb(Vector3(44.439f, 16.5938f, 3.45767f), Vector3(45.1029f, 23.0679f, 6.72584f)); 
  data[13].mAabb = Aabb(Vector3(7.58103f, 33.0787f, 0.413357f), Vector3(8.49432f, 37.6019f, 6.93079f)); 
  data[14].mAabb = Aabb(Vector3(13.9689f, -27.3289f, -30.4764f), Vector3(18.9785f, -24.6271f, -26.1232f)); 
  data[15].mAabb = Aabb(Vector3(-0.086452f, 0.011435f, 18.8275f), Vector3(7.61388f, 5.80197f, 20.4137f)); 
  data[16].mAabb = Aabb(Vector3(-32.2331f, 11.1699f, -36.8557f), Vector3(-26.215f, 17.4273f, -29.9157f)); 
  data[17].mAabb = Aabb(Vector3(-12.0769f, 40.8621f, 12.7676f), Vector3(-8.57259f, 43.6493f, 18.2276f)); 
  data[18].mAabb = Aabb(Vector3(20.6844f, -26.1544f, -36.1358f), Vector3(26.5253f, -21.2299f, -28.1683f)); 
  data[19].mAabb = Aabb(Vector3(29.9766f, -35.8439f, -10.5601f), Vector3(36.892f, -33.7897f, -5.17477f)); 
  data[20].mAabb = Aabb(Vector3(-6.72791f, 20.6234f, 24.4722f), Vector3(-2.94135f, 28.0468f, 27.6079f)); 
  data[21].mAabb = Aabb(Vector3(11.7934f, 14.0033f, -6.57891f), Vector3(15.6511f, 18.168f, 0.230078f)); 
  data[22].mAabb = Aabb(Vector3(10.982f, -8.89398f, -11.962f), Vector3(14.9616f, -4.60804f, -11.2296f)); 
  data[23].mAabb = Aabb(Vector3(29.5297f, 7.42704f, -13.7861f), Vector3(36.827f, 13.4764f, -7.33568f)); 
  data[24].mAabb = Aabb(Vector3(-32.2532f, -8.99176f, -27.4464f), Vector3(-28.2191f, -8.10631f, -26.0548f)); 
  data[25].mAabb = Aabb(Vector3(4.02439f, -29.3865f, -31.2727f), Vector3(6.39542f, -24.3268f, -25.5077f)); 
  data[26].mAabb = Aabb(Vector3(19.3871f, 26.5258f, 11.3853f), Vector3(21.8905f, 28.2617f, 13.8347f)); 
  data[27].mAabb = Aabb(Vector3(15.7824f, -23.3192f, 31.7953f), Vector3(22.3831f, -15.9394f, 35.5529f)); 
  data[28].mAabb = Aabb(Vector3(19.2034f, 30.3805f, 3.6174f), Vector3(25.0754f, 34.1495f, 9.37663f)); 
  data[29].mAabb = Aabb(Vector3(18.3665f, 13.5642f, 29.6665f), Vector3(25.8168f, 15.1631f, 32.779f)); 
  data[30].mAabb = Aabb(Vector3(-2.17962f, 27.8304f, 31.7971f), Vector3(2.26659f, 34.7366f, 35.8232f)); 
  data[31].mAabb = Aabb(Vector3(-26.5523f, -20.3123f, -31.312f), Vector3(-19.6582f, -13.8514f, -24.7115f)); 
  data[32].mAabb = Aabb(Vector3(-0.0573878f, -15.1329f, -32.3941f), Vector3(4.25731f, -14.2967f, -31.8672f)); 
  data[33].mAabb = Aabb(Vector3(-44.6571f, -9.95978f, 2.49951f), Vector3(-41.0013f, -8.36366f, 9.82433f)); 
  data[34].mAabb = Aabb(Vector3(31.4882f, -4.24101f, -10.4492f), Vector3(32.6193f, 0.46891f, -3.57502f)); 
  data[35].mAabb = Aabb(Vector3(-18.9446f, -6.68636f, 40.1358f), Vector3(-12.9653f, -4.42875f, 43.1075f)); 
  data[36].mAabb = Aabb(Vector3(34.1912f, 26.112f, 3.14382f), Vector3(39.8808f, 30.7767f, 8.43382f)); 
  data[37].mAabb = Aabb(Vector3(-33.9623f, -3.88993f, -30.0156f), Vector3(-29.5641f, -0.624767f, -22.7919f)); 
  data[38].mAabb = Aabb(Vector3(-18.1317f, 28.992f, 31.7285f), Vector3(-11.0787f, 31.6864f, 34.1802f)); 
  data[39].mAabb = Aabb(Vector3(1.24816f, -24.7693f, 2.32039f), Vector3(6.20974f, -17.1945f, 7.21934f)); 
  data[40].mAabb = Aabb(Vector3(11.3386f, -23.1163f, 20.6762f), Vector3(17.3318f, -16.5439f, 22.4465f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Frustum frustum;
  frustum.Set(Vector3(35.9026f, -47.8884f, -164.017f), Vector3(-100.458f, -139.725f, 26.888f), Vector3(-100.458f, 87.7902f, 136.337f), Vector3(35.9026f, 179.627f, -54.5674f), Vector3(43.3449f, -2.80854f, 24.3787f), Vector3(39.3675f, -5.48725f, 29.947f), Vector3(39.3675f, 1.61672f, 33.3645f), Vector3(43.3449f, 4.29543f, 27.7962f));

  Application::mStatistics.Clear();
  PrintFrustumCastResults(spatialPartition, frustum, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mFrustumAabbTests", static_cast<int>(Application::mStatistics.mFrustumAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest1(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(51); 
  data[0].mAabb = Aabb(Vector3(-2.50758f, -2.47893f, -3.78198f), Vector3(2.50758f, 2.47893f, 3.78198f)); 
  data[1].mAabb = Aabb(Vector3(-0.301213f, 26.9339f, -36.3619f), Vector3(0.631554f, 28.1958f, -34.7266f)); 
  data[2].mAabb = Aabb(Vector3(26.4048f, -22.7028f, -7.13026f), Vector3(29.3642f, -21.9073f, -5.68398f)); 
  data[3].mAabb = Aabb(Vector3(-15.2883f, 12.2861f, 2.13709f), Vector3(-13.7683f, 13.4378f, 6.67398f)); 
  data[4].mAabb = Aabb(Vector3(-1.53138f, -32.7849f, 7.889f), Vector3(6.21279f, -26.1686f, 14.0462f)); 
  data[5].mAabb = Aabb(Vector3(-32.0719f, -38.3121f, -3.70923f), Vector3(-30.7571f, -30.4276f, 1.39221f)); 
  data[6].mAabb = Aabb(Vector3(-4.86913f, -38.8917f, -5.22337f), Vector3(1.39642f, -34.1523f, 1.08138f)); 
  data[7].mAabb = Aabb(Vector3(18.5335f, 0.799971f, 34.0444f), Vector3(22.887f, 6.86698f, 36.4043f)); 
  data[8].mAabb = Aabb(Vector3(1.75049f, -34.221f, -1.21292f), Vector3(5.60316f, -32.4078f, 0.2329f)); 
  data[9].mAabb = Aabb(Vector3(-11.0306f, 15.9638f, -29.3692f), Vector3(-3.16232f, 19.8286f, -22.9406f)); 
  data[10].mAabb = Aabb(Vector3(35.1365f, 0.363217f, -31.9428f), Vector3(38.519f, 3.47483f, -25.6979f)); 
  data[11].mAabb = Aabb(Vector3(-22.0401f, 3.59088f, -40.1107f), Vector3(-18.302f, 4.69157f, -36.0514f)); 
  data[12].mAabb = Aabb(Vector3(-19.6512f, 10.5655f, 10.4349f), Vector3(-11.7846f, 18.1921f, 13.8793f)); 
  data[13].mAabb = Aabb(Vector3(32.8426f, -10.1661f, 32.2664f), Vector3(36.13f, -8.85777f, 37.3999f)); 
  data[14].mAabb = Aabb(Vector3(-32.8891f, -2.13404f, 30.705f), Vector3(-27.4724f, 0.686955f, 34.9643f)); 
  data[15].mAabb = Aabb(Vector3(-3.1337f, 39.9141f, 17.4567f), Vector3(-2.48645f, 42.8116f, 24.8668f)); 
  data[16].mAabb = Aabb(Vector3(32.1811f, 9.06543f, 25.8265f), Vector3(37.8849f, 11.5675f, 28.0343f)); 
  data[17].mAabb = Aabb(Vector3(27.2713f, 3.03492f, -17.1581f), Vector3(33.7618f, 10.2352f, -15.3147f)); 
  data[18].mAabb = Aabb(Vector3(29.8592f, -9.41472f, -12.0314f), Vector3(33.0824f, -1.53812f, -6.63822f)); 
  data[19].mAabb = Aabb(Vector3(-33.2269f, -10.3172f, -14.8775f), Vector3(-26.6733f, -8.96348f, -7.0976f)); 
  data[20].mAabb = Aabb(Vector3(-34.6607f, -34.0997f, -10.5224f), Vector3(-29.7836f, -30.4467f, -8.87785f)); 
  data[21].mAabb = Aabb(Vector3(-29.3278f, -43.3592f, 3.06245f), Vector3(-22.1698f, -36.3709f, 5.01065f)); 
  data[22].mAabb = Aabb(Vector3(27.9372f, 23.767f, -8.60017f), Vector3(32.7232f, 29.8219f, -5.06549f)); 
  data[23].mAabb = Aabb(Vector3(31.4475f, -2.49368f, -15.132f), Vector3(38.3903f, -1.8172f, -10.9764f)); 
  data[24].mAabb = Aabb(Vector3(15.5607f, 0.142852f, 14.5124f), Vector3(21.4071f, 1.14427f, 19.394f)); 
  data[25].mAabb = Aabb(Vector3(-7.83327f, -5.61041f, -6.9384f), Vector3(-4.01168f, -3.29504f, -5.73751f)); 
  data[26].mAabb = Aabb(Vector3(-3.08567f, -47.5396f, 10.5556f), Vector3(-1.74468f, -45.416f, 13.908f)); 
  data[27].mAabb = Aabb(Vector3(10.2303f, -31.9265f, -33.564f), Vector3(15.8077f, -31.3353f, -31.2444f)); 
  data[28].mAabb = Aabb(Vector3(11.1425f, -35.8542f, 14.1206f), Vector3(12.0616f, -29.7775f, 18.4977f)); 
  data[29].mAabb = Aabb(Vector3(-17.1332f, 8.22728f, -12.1976f), Vector3(-16.0293f, 12.3307f, -5.7275f)); 
  data[30].mAabb = Aabb(Vector3(5.27213f, -0.452401f, 47.6964f), Vector3(11.9273f, 5.95765f, 49.0822f)); 
  data[31].mAabb = Aabb(Vector3(8.66877f, -0.514738f, 12.2106f), Vector3(13.7809f, 0.7838f, 19.7873f)); 
  data[32].mAabb = Aabb(Vector3(7.65944f, 30.1082f, -18.4458f), Vector3(8.36166f, 33.1672f, -11.2601f)); 
  data[33].mAabb = Aabb(Vector3(-19.9466f, -27.3205f, 2.9031f), Vector3(-15.8391f, -23.5358f, 3.34346f)); 
  data[34].mAabb = Aabb(Vector3(25.552f, 3.29898f, -32.182f), Vector3(27.8029f, 4.81971f, -29.8989f)); 
  data[35].mAabb = Aabb(Vector3(-12.5974f, -8.27895f, -3.07694f), Vector3(-4.9404f, -6.49566f, 3.95308f)); 
  data[36].mAabb = Aabb(Vector3(27.803f, -12.9732f, 1.07696f), Vector3(32.3074f, -5.79799f, 5.24877f)); 
  data[37].mAabb = Aabb(Vector3(-9.64127f, 36.7748f, 15.3885f), Vector3(-7.28856f, 43.0607f, 17.654f)); 
  data[38].mAabb = Aabb(Vector3(21.8809f, -5.11757f, -39.9109f), Vector3(23.2659f, -3.90346f, -38.1195f)); 
  data[39].mAabb = Aabb(Vector3(13.288f, 37.8424f, 10.6393f), Vector3(18.1783f, 44.5148f, 12.8477f)); 
  data[40].mAabb = Aabb(Vector3(-25.6522f, 19.5684f, 19.0766f), Vector3(-19.7661f, 22.0125f, 23.8515f)); 
  data[41].mAabb = Aabb(Vector3(23.9224f, 34.2538f, 15.9084f), Vector3(27.9908f, 37.4137f, 19.6772f)); 
  data[42].mAabb = Aabb(Vector3(21.7822f, 32.7412f, 23.9109f), Vector3(25.5558f, 36.56f, 28.8059f)); 
  data[43].mAabb = Aabb(Vector3(-12.4172f, 12.5538f, -30.5909f), Vector3(-9.09125f, 16.1944f, -29.7481f)); 
  data[44].mAabb = Aabb(Vector3(-8.58084f, -21.5101f, 28.2587f), Vector3(-5.61836f, -19.5385f, 28.7802f)); 
  data[45].mAabb = Aabb(Vector3(-41.4872f, -34.6043f, 5.90152f), Vector3(-34.6571f, -27.1345f, 7.49601f)); 
  data[46].mAabb = Aabb(Vector3(-19.0943f, -17.3009f, -31.1915f), Vector3(-18.4259f, -9.8965f, -28.0871f)); 
  data[47].mAabb = Aabb(Vector3(-14.9513f, 4.14504f, 40.0576f), Vector3(-13.8279f, 6.71647f, 45.8496f)); 
  data[48].mAabb = Aabb(Vector3(-6.34929f, 20.6273f, -12.8183f), Vector3(-0.494276f, 23.424f, -11.1268f)); 
  data[49].mAabb = Aabb(Vector3(-4.86283f, 25.2095f, 20.4383f), Vector3(-1.51416f, 31.2299f, 24.1493f)); 
  data[50].mAabb = Aabb(Vector3(31.9946f, -8.22262f, 9.76155f), Vector3(39.4797f, -5.12074f, 11.8582f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest2(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(55); 
  data[0].mAabb = Aabb(Vector3(-16.7992f, 8.13461f, 42.1253f), Vector3(-15.679f, 14.5356f, 44.4535f)); 
  data[1].mAabb = Aabb(Vector3(5.02642f, -42.8865f, -11.965f), Vector3(7.75673f, -39.7632f, -8.62443f)); 
  data[2].mAabb = Aabb(Vector3(18.9163f, -8.46484f, 24.647f), Vector3(21.5761f, -7.65244f, 31.9692f)); 
  data[3].mAabb = Aabb(Vector3(-18.6854f, 8.63766f, -12.1757f), Vector3(-17.7831f, 12.0777f, -10.7477f)); 
  data[4].mAabb = Aabb(Vector3(22.5016f, -23.5486f, 35.4653f), Vector3(26.2251f, -22.4107f, 38.6694f)); 
  data[5].mAabb = Aabb(Vector3(-12.9946f, -15.9812f, -10.8763f), Vector3(-7.38054f, -9.24921f, -7.07819f)); 
  data[6].mAabb = Aabb(Vector3(-28.1376f, -7.56459f, 36.3472f), Vector3(-21.3776f, -1.64487f, 39.627f)); 
  data[7].mAabb = Aabb(Vector3(20.1201f, -6.18015f, 12.9417f), Vector3(26.3666f, -0.0565526f, 19.6115f)); 
  data[8].mAabb = Aabb(Vector3(-29.3063f, 8.31736f, -16.3589f), Vector3(-23.6009f, 10.3282f, -15.9535f)); 
  data[9].mAabb = Aabb(Vector3(-8.16506f, 28.0661f, 0.783155f), Vector3(-5.80168f, 30.5556f, 5.70391f)); 
  data[10].mAabb = Aabb(Vector3(-20.4696f, 5.24913f, -4.22516f), Vector3(-12.8124f, 6.45142f, -3.66419f)); 
  data[11].mAabb = Aabb(Vector3(-23.9505f, -21.1168f, -43.3514f), Vector3(-17.9139f, -13.1882f, -39.8327f)); 
  data[12].mAabb = Aabb(Vector3(23.515f, 20.3285f, -6.98747f), Vector3(28.6197f, 25.1092f, -0.366823f)); 
  data[13].mAabb = Aabb(Vector3(17.4141f, -9.92787f, -18.258f), Vector3(25.322f, -3.90493f, -11.6717f)); 
  data[14].mAabb = Aabb(Vector3(2.40878f, -29.316f, 36.9218f), Vector3(8.23549f, -21.4301f, 41.2565f)); 
  data[15].mAabb = Aabb(Vector3(8.65784f, 25.9946f, -15.1993f), Vector3(9.45956f, 31.4507f, -7.59449f)); 
  data[16].mAabb = Aabb(Vector3(-35.5819f, 6.35854f, 29.1334f), Vector3(-32.2107f, 13.6827f, 35.4725f)); 
  data[17].mAabb = Aabb(Vector3(16.9973f, -10.694f, -36.1173f), Vector3(23.4962f, -10.1539f, -35.6405f)); 
  data[18].mAabb = Aabb(Vector3(-28.9895f, -29.5135f, 10.5896f), Vector3(-26.1645f, -26.41f, 11.5859f)); 
  data[19].mAabb = Aabb(Vector3(8.62469f, 39.6999f, 17.4092f), Vector3(10.5073f, 45.3244f, 21.9649f)); 
  data[20].mAabb = Aabb(Vector3(-22.3019f, -1.25338f, -2.46121f), Vector3(-14.7091f, 1.70052f, 3.83009f)); 
  data[21].mAabb = Aabb(Vector3(-20.1797f, 18.0789f, 0.210073f), Vector3(-18.5047f, 25.4195f, 1.96414f)); 
  data[22].mAabb = Aabb(Vector3(-12.1237f, -3.69946f, 1.62464f), Vector3(-6.23348f, 1.19184f, 8.80797f)); 
  data[23].mAabb = Aabb(Vector3(-34.187f, -10.7773f, -4.37218f), Vector3(-30.2179f, -9.12506f, 2.4767f)); 
  data[24].mAabb = Aabb(Vector3(-2.01026f, -18.0873f, 0.817768f), Vector3(5.06129f, -12.9963f, 7.85498f)); 
  data[25].mAabb = Aabb(Vector3(2.61003f, 23.2893f, 31.3032f), Vector3(5.1854f, 28.5593f, 33.1899f)); 
  data[26].mAabb = Aabb(Vector3(5.8101f, 0.883966f, 1.83365f), Vector3(12.7054f, 7.01754f, 3.77142f)); 
  data[27].mAabb = Aabb(Vector3(-41.8875f, -9.60237f, 11.4717f), Vector3(-33.8968f, -6.41143f, 12.8846f)); 
  data[28].mAabb = Aabb(Vector3(-40.2492f, 18.4659f, -17.0844f), Vector3(-32.3986f, 22.9481f, -14.979f)); 
  data[29].mAabb = Aabb(Vector3(-21.5014f, 41.2704f, -5.58592f), Vector3(-14.097f, 44.4015f, -3.35985f)); 
  data[30].mAabb = Aabb(Vector3(-15.5909f, -11.8925f, 5.18869f), Vector3(-11.5434f, -11.2348f, 5.61072f)); 
  data[31].mAabb = Aabb(Vector3(34.8541f, 11.0595f, -28.6099f), Vector3(38.0084f, 17.6674f, -27.8309f)); 
  data[32].mAabb = Aabb(Vector3(-37.2345f, -9.06823f, -19.39f), Vector3(-33.6442f, -5.69614f, -14.7072f)); 
  data[33].mAabb = Aabb(Vector3(-10.5908f, -1.94345f, -49.892f), Vector3(-3.74658f, 4.69552f, -43.1584f)); 
  data[34].mAabb = Aabb(Vector3(18.7193f, 21.0645f, -4.02304f), Vector3(24.2616f, 23.7454f, -0.840916f)); 
  data[35].mAabb = Aabb(Vector3(3.17898f, -16.6541f, -23.8376f), Vector3(10.1137f, -12.4359f, -16.0547f)); 
  data[36].mAabb = Aabb(Vector3(-7.49134f, -2.85621f, -33.7172f), Vector3(-3.09523f, 2.16707f, -30.6649f)); 
  data[37].mAabb = Aabb(Vector3(4.24276f, -7.40439f, -15.5067f), Vector3(11.6641f, -5.87414f, -11.2502f)); 
  data[38].mAabb = Aabb(Vector3(9.27599f, 13.143f, -26.5209f), Vector3(16.8392f, 19.5466f, -23.1604f)); 
  data[39].mAabb = Aabb(Vector3(-17.0557f, -29.8338f, 19.4942f), Vector3(-13.6435f, -27.7643f, 27.3098f)); 
  data[40].mAabb = Aabb(Vector3(-30.6467f, -22.9503f, 1.26377f), Vector3(-28.654f, -18.7646f, 5.64666f)); 
  data[41].mAabb = Aabb(Vector3(-29.5187f, -10.3244f, 40.8242f), Vector3(-23.101f, -4.3031f, 41.9794f)); 
  data[42].mAabb = Aabb(Vector3(18.3954f, -36.4122f, 8.29819f), Vector3(25.8292f, -29.0563f, 14.5607f)); 
  data[43].mAabb = Aabb(Vector3(10.5395f, 20.7391f, -10.6232f), Vector3(18.5027f, 21.4123f, -6.79764f)); 
  data[44].mAabb = Aabb(Vector3(-12.7757f, -4.00558f, -50.2138f), Vector3(-12.3228f, -2.6871f, -42.9446f)); 
  data[45].mAabb = Aabb(Vector3(-2.25887f, -28.4404f, 9.75473f), Vector3(-0.321107f, -25.8261f, 15.2349f)); 
  data[46].mAabb = Aabb(Vector3(-13.2013f, 32.8117f, -15.6028f), Vector3(-6.88127f, 34.704f, -15.1344f)); 
  data[47].mAabb = Aabb(Vector3(24.3236f, 13.477f, -17.1439f), Vector3(31.8207f, 20.5648f, -14.273f)); 
  data[48].mAabb = Aabb(Vector3(-33.994f, 33.0895f, 13.0373f), Vector3(-27.9205f, 35.6487f, 14.8559f)); 
  data[49].mAabb = Aabb(Vector3(2.65838f, -17.507f, 29.2395f), Vector3(10.3789f, -9.8498f, 29.9357f)); 
  data[50].mAabb = Aabb(Vector3(5.32154f, -44.5449f, 0.0792797f), Vector3(6.80192f, -41.7066f, 2.17825f)); 
  data[51].mAabb = Aabb(Vector3(-6.74213f, 7.29229f, -35.2315f), Vector3(-2.494f, 9.64338f, -27.8122f)); 
  data[52].mAabb = Aabb(Vector3(-11.6855f, 44.4472f, -5.78841f), Vector3(-4.88463f, 49.891f, -0.895482f)); 
  data[53].mAabb = Aabb(Vector3(10.8466f, -7.39447f, -48.6145f), Vector3(15.1499f, -2.34568f, -40.8518f)); 
  data[54].mAabb = Aabb(Vector3(3.20252f, 40.0957f, -16.8093f), Vector3(9.27162f, 42.98f, -11.7809f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest3(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(60); 
  data[0].mAabb = Aabb(Vector3(-24.1076f, -15.1458f, -7.50096f), Vector3(-20.3066f, -12.0465f, -2.08362f)); 
  data[1].mAabb = Aabb(Vector3(-22.833f, -9.96212f, -19.0351f), Vector3(-21.5377f, -4.36688f, -13.8854f)); 
  data[2].mAabb = Aabb(Vector3(0.631305f, -28.6168f, -33.2125f), Vector3(8.18737f, -24.1708f, -28.0628f)); 
  data[3].mAabb = Aabb(Vector3(-31.63f, 17.0114f, 7.66613f), Vector3(-27.3814f, 19.3986f, 13.0095f)); 
  data[4].mAabb = Aabb(Vector3(11.284f, 20.786f, 13.1499f), Vector3(12.2938f, 24.9291f, 17.3272f)); 
  data[5].mAabb = Aabb(Vector3(5.45731f, 34.2665f, 27.4857f), Vector3(9.16756f, 37.5799f, 33.231f)); 
  data[6].mAabb = Aabb(Vector3(20.4289f, 21.0309f, 12.3696f), Vector3(25.2787f, 23.8709f, 18.5878f)); 
  data[7].mAabb = Aabb(Vector3(-12.8842f, 10.5288f, -33.8625f), Vector3(-11.7005f, 16.8076f, -32.2708f)); 
  data[8].mAabb = Aabb(Vector3(-13.1027f, 35.6984f, 14.7574f), Vector3(-7.24773f, 43.431f, 22.0444f)); 
  data[9].mAabb = Aabb(Vector3(-21.2117f, 7.67613f, 20.0369f), Vector3(-17.8192f, 11.9523f, 21.0529f)); 
  data[10].mAabb = Aabb(Vector3(23.0219f, 30.3879f, -15.8573f), Vector3(28.8611f, 32.9714f, -14.7782f)); 
  data[11].mAabb = Aabb(Vector3(17.0185f, 18.978f, 34.6618f), Vector3(23.173f, 25.1897f, 41.8843f)); 
  data[12].mAabb = Aabb(Vector3(-31.0684f, 19.9345f, 17.5381f), Vector3(-28.2065f, 26.8771f, 23.1057f)); 
  data[13].mAabb = Aabb(Vector3(42.393f, 7.6572f, -5.57656f), Vector3(43.9993f, 10.5376f, 0.0286493f)); 
  data[14].mAabb = Aabb(Vector3(24.0206f, -19.2932f, -31.7671f), Vector3(24.4387f, -13.472f, -24.7471f)); 
  data[15].mAabb = Aabb(Vector3(-38.8657f, 1.29555f, 21.7951f), Vector3(-31.2751f, 4.55515f, 27.7183f)); 
  data[16].mAabb = Aabb(Vector3(36.5921f, -10.9933f, 0.767018f), Vector3(43.9166f, -4.90103f, 3.42334f)); 
  data[17].mAabb = Aabb(Vector3(-3.58337f, 29.0808f, -14.618f), Vector3(4.23085f, 33.1585f, -11.1773f)); 
  data[18].mAabb = Aabb(Vector3(-42.8968f, 2.98605f, -23.7177f), Vector3(-40.4907f, 3.85689f, -15.9624f)); 
  data[19].mAabb = Aabb(Vector3(-15.6478f, 15.5547f, 21.721f), Vector3(-8.85091f, 18.1778f, 29.6716f)); 
  data[20].mAabb = Aabb(Vector3(29.9655f, -5.2548f, -4.17559f), Vector3(31.4721f, 1.68709f, -0.956351f)); 
  data[21].mAabb = Aabb(Vector3(-35.1461f, -3.52256f, -6.11899f), Vector3(-30.5301f, 3.43441f, -1.53084f)); 
  data[22].mAabb = Aabb(Vector3(6.80666f, -2.39641f, -41.0308f), Vector3(10.1312f, -0.300688f, -40.0134f)); 
  data[23].mAabb = Aabb(Vector3(-8.50428f, -34.2229f, -28.3715f), Vector3(-4.23853f, -32.7349f, -25.4176f)); 
  data[24].mAabb = Aabb(Vector3(-25.8713f, -3.73087f, 12.2759f), Vector3(-21.0772f, -0.29407f, 18.1903f)); 
  data[25].mAabb = Aabb(Vector3(-24.9134f, 8.64199f, -39.7979f), Vector3(-23.8753f, 13.1197f, -39.3469f)); 
  data[26].mAabb = Aabb(Vector3(-32.2395f, -34.3641f, 23.7758f), Vector3(-25.3166f, -27.2408f, 29.0711f)); 
  data[27].mAabb = Aabb(Vector3(-8.85083f, -49.0027f, 1.14042f), Vector3(-2.90815f, -48.3693f, 2.29493f)); 
  data[28].mAabb = Aabb(Vector3(-27.6988f, 18.0122f, -14.3845f), Vector3(-23.504f, 25.8162f, -13.2771f)); 
  data[29].mAabb = Aabb(Vector3(14.2025f, -0.244559f, 17.9472f), Vector3(21.1757f, 6.70892f, 25.2711f)); 
  data[30].mAabb = Aabb(Vector3(19.1237f, 38.1133f, 19.5793f), Vector3(26.78f, 38.7281f, 22.5014f)); 
  data[31].mAabb = Aabb(Vector3(9.20902f, -19.5473f, -25.6704f), Vector3(13.0475f, -17.7582f, -22.4001f)); 
  data[32].mAabb = Aabb(Vector3(12.4933f, 9.11336f, 24.4029f), Vector3(16.0852f, 10.7088f, 28.138f)); 
  data[33].mAabb = Aabb(Vector3(11.9225f, -22.1356f, 20.8921f), Vector3(16.6894f, -16.4531f, 21.5844f)); 
  data[34].mAabb = Aabb(Vector3(6.37434f, 7.86109f, 23.5307f), Vector3(10.6849f, 12.4497f, 25.4467f)); 
  data[35].mAabb = Aabb(Vector3(6.54352f, 5.94342f, 24.8147f), Vector3(13.5392f, 11.8875f, 30.4071f)); 
  data[36].mAabb = Aabb(Vector3(-6.99589f, -40.4121f, -8.29334f), Vector3(-5.2518f, -38.7253f, -0.974315f)); 
  data[37].mAabb = Aabb(Vector3(-22.7833f, 9.06183f, 16.5864f), Vector3(-21.2227f, 14.995f, 22.8721f)); 
  data[38].mAabb = Aabb(Vector3(-18.1843f, 0.240179f, -9.91485f), Vector3(-15.0035f, 0.727389f, -6.14637f)); 
  data[39].mAabb = Aabb(Vector3(-10.1518f, -43.6059f, -7.49841f), Vector3(-8.95207f, -38.6026f, -5.28487f)); 
  data[40].mAabb = Aabb(Vector3(33.0221f, 8.42706f, -10.7894f), Vector3(38.4606f, 14.046f, -7.25914f)); 
  data[41].mAabb = Aabb(Vector3(-11.8581f, -39.4094f, 19.5845f), Vector3(-9.08117f, -31.7346f, 25.4625f)); 
  data[42].mAabb = Aabb(Vector3(1.66683f, 9.59869f, 4.78217f), Vector3(4.86264f, 13.7986f, 11.1655f)); 
  data[43].mAabb = Aabb(Vector3(-5.08494f, -0.204044f, -36.2143f), Vector3(-0.439031f, 4.59239f, -29.012f)); 
  data[44].mAabb = Aabb(Vector3(-1.38446f, -15.1896f, -15.6967f), Vector3(0.406029f, -8.8941f, -9.19407f)); 
  data[45].mAabb = Aabb(Vector3(-4.35118f, -45.3077f, -5.94481f), Vector3(1.20996f, -42.8464f, 1.49459f)); 
  data[46].mAabb = Aabb(Vector3(38.7857f, 11.3094f, 18.5706f), Vector3(39.7823f, 16.5405f, 22.7366f)); 
  data[47].mAabb = Aabb(Vector3(-0.386935f, 14.4689f, -12.9177f), Vector3(6.36476f, 17.9043f, -7.38645f)); 
  data[48].mAabb = Aabb(Vector3(-3.96368f, -24.0631f, 26.8074f), Vector3(-0.266415f, -16.4351f, 32.7053f)); 
  data[49].mAabb = Aabb(Vector3(-12.1519f, 6.51251f, -49.6466f), Vector3(-7.58296f, 8.17728f, -43.6076f)); 
  data[50].mAabb = Aabb(Vector3(-45.4222f, -11.5465f, -9.37597f), Vector3(-44.6609f, -5.92416f, -8.91358f)); 
  data[51].mAabb = Aabb(Vector3(-35.7609f, 33.1267f, -1.75854f), Vector3(-32.8401f, 34.1488f, 5.79405f)); 
  data[52].mAabb = Aabb(Vector3(-13.5106f, -19.2871f, -16.7938f), Vector3(-12.2726f, -16.1567f, -16.0753f)); 
  data[53].mAabb = Aabb(Vector3(7.78797f, 35.3104f, 22.0959f), Vector3(10.294f, 40.1666f, 24.7901f)); 
  data[54].mAabb = Aabb(Vector3(26.8533f, -36.5412f, -11.2416f), Vector3(32.8362f, -29.9189f, -9.29343f)); 
  data[55].mAabb = Aabb(Vector3(11.2015f, 6.39833f, -2.80066f), Vector3(14.86f, 9.10428f, -0.97538f)); 
  data[56].mAabb = Aabb(Vector3(-14.6807f, 6.73162f, 27.5666f), Vector3(-13.2553f, 7.79706f, 35.0394f)); 
  data[57].mAabb = Aabb(Vector3(-16.1351f, 4.70555f, -35.8644f), Vector3(-13.5178f, 11.8462f, -30.6899f)); 
  data[58].mAabb = Aabb(Vector3(13.445f, 29.383f, -1.5127f), Vector3(14.2393f, 33.2885f, 1.05478f)); 
  data[59].mAabb = Aabb(Vector3(-15.21f, -31.9402f, 15.9602f), Vector3(-12.376f, -24.4479f, 19.8523f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest4(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(53); 
  data[0].mAabb = Aabb(Vector3(30.1325f, 19.8332f, -1.08904f), Vector3(34.8211f, 25.6444f, 3.35253f)); 
  data[1].mAabb = Aabb(Vector3(-34.2909f, 12.7534f, 17.074f), Vector3(-31.1392f, 20.0411f, 20.2462f)); 
  data[2].mAabb = Aabb(Vector3(-12.9589f, 26.34f, -1.8353f), Vector3(-10.3362f, 33.9881f, 6.06589f)); 
  data[3].mAabb = Aabb(Vector3(11.2737f, 28.0102f, -6.58087f), Vector3(14.6783f, 31.0764f, 1.4022f)); 
  data[4].mAabb = Aabb(Vector3(4.91411f, -33.5782f, 27.7206f), Vector3(8.69093f, -32.6086f, 28.1811f)); 
  data[5].mAabb = Aabb(Vector3(6.72911f, -43.368f, 14.0211f), Vector3(8.03113f, -37.9632f, 19.3385f)); 
  data[6].mAabb = Aabb(Vector3(29.2857f, -14.3844f, -40.4207f), Vector3(31.4983f, -10.3378f, -34.6802f)); 
  data[7].mAabb = Aabb(Vector3(-19.7352f, 11.0762f, 4.27473f), Vector3(-12.5344f, 15.4598f, 7.56355f)); 
  data[8].mAabb = Aabb(Vector3(-12.0083f, -18.0766f, -8.89825f), Vector3(-8.96183f, -11.9935f, -8.23128f)); 
  data[9].mAabb = Aabb(Vector3(26.9241f, 12.9917f, -38.8451f), Vector3(30.8814f, 16.9939f, -32.0577f)); 
  data[10].mAabb = Aabb(Vector3(-45.9075f, -16.92f, 8.19335f), Vector3(-38.8856f, -15.7706f, 10.9227f)); 
  data[11].mAabb = Aabb(Vector3(35.8588f, -27.5141f, 15.9616f), Vector3(39.3907f, -20.2907f, 16.5588f)); 
  data[12].mAabb = Aabb(Vector3(14.1302f, 22.5182f, 3.23623f), Vector3(21.3984f, 23.2721f, 4.14743f)); 
  data[13].mAabb = Aabb(Vector3(-41.2775f, 6.00958f, -12.0192f), Vector3(-36.7707f, 7.8729f, -6.04218f)); 
  data[14].mAabb = Aabb(Vector3(25.3746f, -25.9718f, 6.87654f), Vector3(26.2345f, -23.9554f, 11.4981f)); 
  data[15].mAabb = Aabb(Vector3(15.5233f, 25.525f, 18.1673f), Vector3(22.5617f, 27.7155f, 18.68f)); 
  data[16].mAabb = Aabb(Vector3(13.2285f, 2.43175f, -42.5386f), Vector3(19.6163f, 8.63954f, -40.2135f)); 
  data[17].mAabb = Aabb(Vector3(3.4828f, 34.7006f, -19.8117f), Vector3(5.83156f, 41.9959f, -15.7514f)); 
  data[18].mAabb = Aabb(Vector3(-17.2999f, -12.3218f, -6.92984f), Vector3(-10.7486f, -8.50628f, -1.53383f)); 
  data[19].mAabb = Aabb(Vector3(5.81491f, 41.7975f, -18.1913f), Vector3(7.89115f, 48.8904f, -11.6078f)); 
  data[20].mAabb = Aabb(Vector3(-30.4795f, -19.3565f, 27.9933f), Vector3(-25.8878f, -13.656f, 34.8914f)); 
  data[21].mAabb = Aabb(Vector3(-17.5121f, 4.45783f, 30.3423f), Vector3(-15.7204f, 7.88094f, 33.049f)); 
  data[22].mAabb = Aabb(Vector3(-23.2456f, -10.1253f, 29.532f), Vector3(-16.0734f, -4.02029f, 29.9418f)); 
  data[23].mAabb = Aabb(Vector3(1.62228f, 26.821f, 23.1648f), Vector3(6.41733f, 29.1303f, 29.5551f)); 
  data[24].mAabb = Aabb(Vector3(-32.1828f, -12.6207f, -4.01258f), Vector3(-31.2633f, -9.94424f, -1.40775f)); 
  data[25].mAabb = Aabb(Vector3(11.0896f, -15.3509f, -30.1407f), Vector3(11.5963f, -9.71433f, -23.5971f)); 
  data[26].mAabb = Aabb(Vector3(6.909f, -6.1996f, 37.7484f), Vector3(8.15049f, -1.33219f, 42.0477f)); 
  data[27].mAabb = Aabb(Vector3(-22.2296f, -25.2344f, 10.9351f), Vector3(-21.4671f, -21.321f, 13.9212f)); 
  data[28].mAabb = Aabb(Vector3(-13.3063f, -25.9744f, 38.6083f), Vector3(-8.27702f, -22.0981f, 45.0895f)); 
  data[29].mAabb = Aabb(Vector3(21.1645f, -22.8191f, -38.3126f), Vector3(26.6925f, -16.8356f, -30.741f)); 
  data[30].mAabb = Aabb(Vector3(-19.4289f, -23.5471f, 10.2917f), Vector3(-17.5691f, -20.8562f, 15.5233f)); 
  data[31].mAabb = Aabb(Vector3(15.4318f, -34.6128f, -28.1158f), Vector3(16.7739f, -33.6726f, -22.631f)); 
  data[32].mAabb = Aabb(Vector3(33.2929f, 5.91026f, 13.6636f), Vector3(37.8036f, 9.30532f, 19.7148f)); 
  data[33].mAabb = Aabb(Vector3(-27.6028f, -26.8076f, 27.0438f), Vector3(-26.7976f, -24.6908f, 31.7709f)); 
  data[34].mAabb = Aabb(Vector3(13.5889f, -22.7314f, 1.33949f), Vector3(16.1863f, -18.0275f, 8.84545f)); 
  data[35].mAabb = Aabb(Vector3(-3.64964f, -28.5464f, -22.6878f), Vector3(2.37075f, -20.9377f, -21.0884f)); 
  data[36].mAabb = Aabb(Vector3(-35.9157f, -18.9507f, -30.3764f), Vector3(-30.566f, -13.2617f, -26.074f)); 
  data[37].mAabb = Aabb(Vector3(34.6077f, 25.5107f, -12.1302f), Vector3(37.7124f, 29.9003f, -6.73212f)); 
  data[38].mAabb = Aabb(Vector3(-21.7436f, -13.4454f, -2.49914f), Vector3(-20.4095f, -5.58966f, 0.406519f)); 
  data[39].mAabb = Aabb(Vector3(-8.54466f, -14.747f, 3.22187f), Vector3(-1.06351f, -12.256f, 6.55522f)); 
  data[40].mAabb = Aabb(Vector3(21.102f, -32.5611f, 24.8263f), Vector3(22.2438f, -28.6268f, 25.8402f)); 
  data[41].mAabb = Aabb(Vector3(-25.3311f, -42.0833f, 11.6835f), Vector3(-24.2701f, -36.1844f, 12.9702f)); 
  data[42].mAabb = Aabb(Vector3(-32.4816f, 18.8396f, 26.5216f), Vector3(-26.4339f, 25.1905f, 34.1217f)); 
  data[43].mAabb = Aabb(Vector3(1.65701f, 4.57217f, 15.8573f), Vector3(3.09193f, 10.0064f, 22.0741f)); 
  data[44].mAabb = Aabb(Vector3(8.0825f, -34.9085f, 5.96901f), Vector3(15.9401f, -29.9736f, 7.60178f)); 
  data[45].mAabb = Aabb(Vector3(-23.4054f, 25.4256f, -29.0435f), Vector3(-18.0266f, 32.1084f, -26.8188f)); 
  data[46].mAabb = Aabb(Vector3(-32.5185f, 13.4355f, 24.7674f), Vector3(-25.1645f, 19.0676f, 29.762f)); 
  data[47].mAabb = Aabb(Vector3(14.7422f, 4.24488f, 8.55899f), Vector3(15.9356f, 7.6738f, 15.0569f)); 
  data[48].mAabb = Aabb(Vector3(-8.6839f, -32.5328f, 37.9166f), Vector3(-4.12891f, -30.753f, 38.4161f)); 
  data[49].mAabb = Aabb(Vector3(-10.6213f, -3.78882f, 6.81168f), Vector3(-5.63724f, -2.92402f, 13.1477f)); 
  data[50].mAabb = Aabb(Vector3(9.84703f, 16.2778f, -45.0574f), Vector3(11.2423f, 23.1898f, -37.7233f)); 
  data[51].mAabb = Aabb(Vector3(-44.6563f, -2.01129f, -27.0245f), Vector3(-40.8591f, 4.61933f, -19.1439f)); 
  data[52].mAabb = Aabb(Vector3(0.905584f, 18.8621f, 37.4681f), Vector3(1.35058f, 19.5132f, 44.2933f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest5(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(68); 
  data[0].mAabb = Aabb(Vector3(8.5138f, -29.0914f, -5.49732f), Vector3(13.8657f, -28.2268f, 1.04076f)); 
  data[1].mAabb = Aabb(Vector3(-28.5292f, 0.99968f, -18.6841f), Vector3(-28.0763f, 5.75089f, -14.7176f)); 
  data[2].mAabb = Aabb(Vector3(-14.8418f, 33.4101f, 26.1557f), Vector3(-12.2277f, 39.5936f, 32.7098f)); 
  data[3].mAabb = Aabb(Vector3(-0.745198f, -5.05895f, 20.7074f), Vector3(6.27323f, -3.68433f, 24.6006f)); 
  data[4].mAabb = Aabb(Vector3(17.6287f, 32.2114f, -26.611f), Vector3(19.9031f, 33.2102f, -19.2171f)); 
  data[5].mAabb = Aabb(Vector3(35.5859f, -3.27298f, 19.3573f), Vector3(42.8643f, 4.4088f, 24.9154f)); 
  data[6].mAabb = Aabb(Vector3(27.5858f, -20.1954f, 7.53418f), Vector3(30.5181f, -19.7764f, 10.7495f)); 
  data[7].mAabb = Aabb(Vector3(29.802f, -15.3278f, 6.40103f), Vector3(37.5638f, -10.045f, 9.93362f)); 
  data[8].mAabb = Aabb(Vector3(-30.6966f, -34.2063f, -0.746721f), Vector3(-27.3057f, -26.3685f, 2.67964f)); 
  data[9].mAabb = Aabb(Vector3(-27.4372f, 7.94766f, 5.8504f), Vector3(-23.358f, 8.71459f, 7.43886f)); 
  data[10].mAabb = Aabb(Vector3(10.8935f, 1.94367f, 1.5557f), Vector3(12.1355f, 8.77353f, 4.40755f)); 
  data[11].mAabb = Aabb(Vector3(-2.72864f, 14.8577f, -13.6132f), Vector3(5.109f, 15.8649f, -11.3843f)); 
  data[12].mAabb = Aabb(Vector3(7.1769f, -10.5834f, -15.0116f), Vector3(14.6151f, -6.698f, -10.4315f)); 
  data[13].mAabb = Aabb(Vector3(19.4612f, -13.9953f, 27.674f), Vector3(20.5322f, -13.281f, 32.7546f)); 
  data[14].mAabb = Aabb(Vector3(-40.1678f, -14.0513f, 19.7511f), Vector3(-36.7048f, -6.30526f, 24.7556f)); 
  data[15].mAabb = Aabb(Vector3(7.90261f, 30.8285f, 1.67473f), Vector3(13.0082f, 37.5247f, 3.2342f)); 
  data[16].mAabb = Aabb(Vector3(-13.27f, 28.012f, 13.6096f), Vector3(-6.15411f, 30.1186f, 19.9776f)); 
  data[17].mAabb = Aabb(Vector3(-37.5404f, -25.3909f, 20.4688f), Vector3(-31.3759f, -19.8986f, 23.0938f)); 
  data[18].mAabb = Aabb(Vector3(7.23104f, -18.3184f, 34.9028f), Vector3(13.5474f, -15.6746f, 39.2309f)); 
  data[19].mAabb = Aabb(Vector3(-11.9758f, -6.43637f, 22.4118f), Vector3(-11.3468f, -1.19925f, 25.8435f)); 
  data[20].mAabb = Aabb(Vector3(21.379f, -20.9279f, -4.26746f), Vector3(24.3297f, -16.5932f, -3.61813f)); 
  data[21].mAabb = Aabb(Vector3(-28.3409f, -21.5341f, -21.854f), Vector3(-27.2724f, -16.3095f, -16.5598f)); 
  data[22].mAabb = Aabb(Vector3(-27.9282f, 26.3374f, -12.1239f), Vector3(-25.7652f, 27.565f, -8.38144f)); 
  data[23].mAabb = Aabb(Vector3(-41.3185f, -1.75843f, 17.8311f), Vector3(-33.7678f, -0.53156f, 24.2055f)); 
  data[24].mAabb = Aabb(Vector3(-32.9405f, -19.6727f, 9.92623f), Vector3(-26.2291f, -13.4442f, 16.2815f)); 
  data[25].mAabb = Aabb(Vector3(-13.5257f, -28.7451f, 27.6519f), Vector3(-7.7834f, -27.1079f, 33.159f)); 
  data[26].mAabb = Aabb(Vector3(-9.25702f, -35.0349f, -11.1342f), Vector3(-5.4621f, -32.7722f, -9.19987f)); 
  data[27].mAabb = Aabb(Vector3(7.46426f, 30.8363f, -3.84886f), Vector3(11.3844f, 33.7299f, 2.72587f)); 
  data[28].mAabb = Aabb(Vector3(-26.1896f, 0.892356f, -8.34858f), Vector3(-22.2179f, 4.0724f, -2.18277f)); 
  data[29].mAabb = Aabb(Vector3(-16.691f, 26.7652f, -19.9259f), Vector3(-13.4959f, 32.4175f, -12.8836f)); 
  data[30].mAabb = Aabb(Vector3(12.911f, -8.02624f, 30.3475f), Vector3(18.4828f, -4.26774f, 31.1562f)); 
  data[31].mAabb = Aabb(Vector3(-1.02876f, 6.34498f, 5.24083f), Vector3(3.24163f, 11.3103f, 8.90934f)); 
  data[32].mAabb = Aabb(Vector3(-21.1883f, 30.1738f, 31.046f), Vector3(-16.1634f, 34.9482f, 32.9819f)); 
  data[33].mAabb = Aabb(Vector3(-6.3011f, -11.0154f, -9.72948f), Vector3(-0.46116f, -5.48158f, -3.12693f)); 
  data[34].mAabb = Aabb(Vector3(3.92277f, 3.07076f, -48.6606f), Vector3(6.43366f, 9.5715f, -46.1704f)); 
  data[35].mAabb = Aabb(Vector3(-3.97252f, 18.2754f, 16.0899f), Vector3(2.77152f, 21.1662f, 19.7693f)); 
  data[36].mAabb = Aabb(Vector3(13.0919f, 1.86785f, -40.9757f), Vector3(14.8397f, 6.46227f, -34.5023f)); 
  data[37].mAabb = Aabb(Vector3(42.6744f, -19.4327f, 3.59007f), Vector3(47.6675f, -15.0495f, 6.51915f)); 
  data[38].mAabb = Aabb(Vector3(19.2973f, 4.52731f, -27.5f), Vector3(25.8247f, 7.86229f, -24.0247f)); 
  data[39].mAabb = Aabb(Vector3(17.1072f, -40.0306f, -23.1983f), Vector3(22.8771f, -39.6204f, -19.2909f)); 
  data[40].mAabb = Aabb(Vector3(-6.76642f, 0.647208f, -41.2706f), Vector3(-1.55133f, 4.82807f, -39.2074f)); 
  data[41].mAabb = Aabb(Vector3(-29.9058f, -13.0899f, -13.04f), Vector3(-28.7787f, -7.70553f, -9.97452f)); 
  data[42].mAabb = Aabb(Vector3(-3.0896f, -18.8191f, 31.3449f), Vector3(3.53175f, -11.4196f, 32.4781f)); 
  data[43].mAabb = Aabb(Vector3(-2.40973f, 34.6251f, -36.3858f), Vector3(-0.520437f, 37.5004f, -31.0178f)); 
  data[44].mAabb = Aabb(Vector3(29.8725f, -35.7953f, -13.7477f), Vector3(34.335f, -28.2554f, -11.2008f)); 
  data[45].mAabb = Aabb(Vector3(2.06808f, -31.2874f, -27.4267f), Vector3(2.94031f, -25.688f, -24.4549f)); 
  data[46].mAabb = Aabb(Vector3(-26.4452f, -28.6504f, -24.1903f), Vector3(-24.2273f, -23.3436f, -19.2324f)); 
  data[47].mAabb = Aabb(Vector3(-10.0864f, -19.6974f, -38.156f), Vector3(-8.9528f, -15.7779f, -30.975f)); 
  data[48].mAabb = Aabb(Vector3(-39.8738f, -4.64573f, 19.7508f), Vector3(-33.8715f, -3.24653f, 25.8971f)); 
  data[49].mAabb = Aabb(Vector3(17.3525f, 32.8389f, -5.61801f), Vector3(20.9194f, 38.7521f, -2.33034f)); 
  data[50].mAabb = Aabb(Vector3(-7.02078f, 38.8224f, 2.00958f), Vector3(-3.76374f, 41.3001f, 8.9786f)); 
  data[51].mAabb = Aabb(Vector3(16.9422f, 21.8475f, -28.1086f), Vector3(20.892f, 25.3254f, -22.3981f)); 
  data[52].mAabb = Aabb(Vector3(34.2268f, 13.3912f, 27.6723f), Vector3(36.3123f, 18.2101f, 33.9142f)); 
  data[53].mAabb = Aabb(Vector3(-19.9318f, 1.32037f, -27.2159f), Vector3(-12.1707f, 8.04956f, -20.1677f)); 
  data[54].mAabb = Aabb(Vector3(-20.2911f, -27.9601f, 3.55526f), Vector3(-15.6603f, -26.9355f, 6.01953f)); 
  data[55].mAabb = Aabb(Vector3(15.6865f, -5.31387f, 10.7997f), Vector3(22.164f, -1.35498f, 16.8528f)); 
  data[56].mAabb = Aabb(Vector3(30.2932f, 24.2268f, 18.6124f), Vector3(34.8115f, 29.9726f, 25.5116f)); 
  data[57].mAabb = Aabb(Vector3(3.05541f, 21.1707f, 27.1656f), Vector3(10.3209f, 25.7856f, 34.1119f)); 
  data[58].mAabb = Aabb(Vector3(-19.356f, 17.3279f, 32.1622f), Vector3(-15.9823f, 18.7101f, 40.0831f)); 
  data[59].mAabb = Aabb(Vector3(35.2084f, 23.3819f, -0.660703f), Vector3(41.7836f, 30.7448f, 7.2166f)); 
  data[60].mAabb = Aabb(Vector3(-28.6079f, 17.7732f, 1.1418f), Vector3(-21.3837f, 22.6972f, 6.28592f)); 
  data[61].mAabb = Aabb(Vector3(34.4263f, -30.5378f, 2.87007f), Vector3(41.5818f, -25.4415f, 10.6184f)); 
  data[62].mAabb = Aabb(Vector3(28.1971f, -4.19306f, 16.9684f), Vector3(33.5989f, 0.699635f, 24.6145f)); 
  data[63].mAabb = Aabb(Vector3(-26.8001f, -23.3785f, -26.0848f), Vector3(-19.803f, -17.625f, -22.4729f)); 
  data[64].mAabb = Aabb(Vector3(-3.30318f, 2.00733f, -40.7136f), Vector3(3.45455f, 9.55852f, -39.9224f)); 
  data[65].mAabb = Aabb(Vector3(30.8312f, -7.50313f, -10.7355f), Vector3(37.3523f, -1.49805f, -2.87906f)); 
  data[66].mAabb = Aabb(Vector3(-21.3086f, -8.80438f, -29.6547f), Vector3(-14.7079f, -6.66784f, -25.6749f)); 
  data[67].mAabb = Aabb(Vector3(-23.6471f, 8.83903f, -26.9095f), Vector3(-23.1272f, 11.9168f, -22.7226f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest6(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mAabb = Aabb(Vector3(-43.71f, -12.9205f, 4.57206f), Vector3(-40.2903f, -7.38653f, 8.56644f)); 
  data[1].mAabb = Aabb(Vector3(23.5208f, 4.55855f, -34.418f), Vector3(24.3443f, 10.6207f, -31.9672f)); 
  data[2].mAabb = Aabb(Vector3(-33.1303f, 2.346f, -34.7239f), Vector3(-29.2837f, 7.15728f, -30.6664f)); 
  data[3].mAabb = Aabb(Vector3(-36.608f, 11.4273f, 10.9991f), Vector3(-35.4881f, 17.6302f, 15.3268f)); 
  data[4].mAabb = Aabb(Vector3(-23.4072f, 3.67646f, 42.3565f), Vector3(-17.3087f, 9.87869f, 47.0585f)); 
  data[5].mAabb = Aabb(Vector3(17.4761f, 33.6361f, 23.1494f), Vector3(21.8948f, 35.1745f, 28.5512f)); 
  data[6].mAabb = Aabb(Vector3(21.4777f, 32.0877f, -3.7401f), Vector3(23.9336f, 36.4042f, 1.40424f)); 
  data[7].mAabb = Aabb(Vector3(-6.48878f, -34.8133f, 31.6145f), Vector3(-4.731f, -33.9274f, 38.4418f)); 
  data[8].mAabb = Aabb(Vector3(-2.15006f, 5.54891f, 39.7893f), Vector3(-0.28489f, 7.08727f, 45.3499f)); 
  data[9].mAabb = Aabb(Vector3(-10.303f, 6.325f, 33.4815f), Vector3(-7.06497f, 11.897f, 38.2211f)); 
  data[10].mAabb = Aabb(Vector3(-13.3442f, 23.2462f, 7.17912f), Vector3(-11.1903f, 26.2775f, 13.1037f)); 
  data[11].mAabb = Aabb(Vector3(-29.0272f, 21.1673f, -26.6276f), Vector3(-28.1636f, 23.3257f, -23.0998f)); 
  data[12].mAabb = Aabb(Vector3(-19.5292f, -12.5129f, -22.4663f), Vector3(-14.5781f, -7.28739f, -20.8737f)); 
  data[13].mAabb = Aabb(Vector3(13.9327f, -13.4802f, -15.2076f), Vector3(16.4044f, -5.57272f, -7.51885f)); 
  data[14].mAabb = Aabb(Vector3(1.93227f, 22.3455f, -2.59389f), Vector3(7.55836f, 28.3972f, 2.26146f)); 
  data[15].mAabb = Aabb(Vector3(-45.2228f, 18.1727f, -2.61543f), Vector3(-42.665f, 19.7227f, -2.0431f)); 
  data[16].mAabb = Aabb(Vector3(0.398119f, -0.989546f, 45.3237f), Vector3(1.156f, -0.405849f, 49.1801f)); 
  data[17].mAabb = Aabb(Vector3(39.0176f, -16.1689f, -18.5926f), Vector3(41.834f, -14.8509f, -12.8565f)); 
  data[18].mAabb = Aabb(Vector3(26.7219f, -31.6052f, -12.1971f), Vector3(32.9696f, -26.0524f, -10.0916f)); 
  data[19].mAabb = Aabb(Vector3(-31.0585f, -30.6937f, 10.1946f), Vector3(-29.6542f, -29.4887f, 13.806f)); 
  data[20].mAabb = Aabb(Vector3(-14.7253f, 16.6662f, 31.9124f), Vector3(-7.74008f, 24.659f, 36.2972f)); 
  data[21].mAabb = Aabb(Vector3(31.5963f, 16.8673f, 20.8833f), Vector3(34.5363f, 21.4822f, 25.2913f)); 
  data[22].mAabb = Aabb(Vector3(-11.2466f, 15.5022f, 2.51203f), Vector3(-5.26073f, 18.5872f, 3.58188f)); 
  data[23].mAabb = Aabb(Vector3(16.8901f, 28.2713f, 20.8999f), Vector3(23.1434f, 31.436f, 25.2538f)); 
  data[24].mAabb = Aabb(Vector3(-23.2488f, 3.14801f, 29.6038f), Vector3(-20.5431f, 4.40758f, 36.279f)); 
  data[25].mAabb = Aabb(Vector3(11.1403f, 9.551f, 14.6771f), Vector3(17.8358f, 12.5534f, 19.2739f)); 
  data[26].mAabb = Aabb(Vector3(15.9923f, -12.4891f, -23.2293f), Vector3(23.6434f, -10.0706f, -16.196f)); 
  data[27].mAabb = Aabb(Vector3(-9.07264f, 4.1647f, -2.48111f), Vector3(-8.6462f, 11.365f, -0.2119f)); 
  data[28].mAabb = Aabb(Vector3(13.2289f, 26.8177f, -4.02355f), Vector3(19.8214f, 34.1594f, 3.70392f)); 
  data[29].mAabb = Aabb(Vector3(-8.27041f, -39.7074f, -5.50466f), Vector3(-3.5853f, -36.3186f, -1.6673f)); 
  data[30].mAabb = Aabb(Vector3(21.8335f, -43.252f, -6.38991f), Vector3(26.132f, -39.9126f, 1.39346f)); 
  data[31].mAabb = Aabb(Vector3(-14.9646f, -38.7649f, 22.3511f), Vector3(-9.50457f, -36.7701f, 22.9905f)); 
  data[32].mAabb = Aabb(Vector3(-24.2488f, -18.2773f, 35.8305f), Vector3(-16.3525f, -16.463f, 43.465f)); 
  data[33].mAabb = Aabb(Vector3(-40.5886f, -18.8533f, -10.6596f), Vector3(-34.4147f, -11.8115f, -8.32596f)); 
  data[34].mAabb = Aabb(Vector3(-22.8053f, -24.0337f, -23.235f), Vector3(-19.0255f, -20.04f, -18.6585f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest7(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(26); 
  data[0].mAabb = Aabb(Vector3(1.91914f, -13.8079f, 2.5808f), Vector3(9.27922f, -7.80284f, 3.9858f)); 
  data[1].mAabb = Aabb(Vector3(11.7349f, -23.4477f, -43.502f), Vector3(12.8544f, -22.5701f, -40.6762f)); 
  data[2].mAabb = Aabb(Vector3(-0.997606f, 22.5759f, 35.0433f), Vector3(5.13782f, 26.1285f, 37.7667f)); 
  data[3].mAabb = Aabb(Vector3(37.2761f, -2.1412f, 2.94529f), Vector3(39.5274f, 1.10286f, 10.2873f)); 
  data[4].mAabb = Aabb(Vector3(-30.9472f, 36.9036f, -6.90366f), Vector3(-28.4562f, 38.4417f, -3.42465f)); 
  data[5].mAabb = Aabb(Vector3(-20.6193f, 25.1285f, -30.2429f), Vector3(-16.3927f, 31.0336f, -26.557f)); 
  data[6].mAabb = Aabb(Vector3(25.1066f, -32.0647f, -23.032f), Vector3(29.7708f, -25.8105f, -21.0513f)); 
  data[7].mAabb = Aabb(Vector3(-7.22291f, -22.7419f, -47.6084f), Vector3(-3.13111f, -20.0605f, -41.491f)); 
  data[8].mAabb = Aabb(Vector3(-8.33838f, -48.5469f, 7.52874f), Vector3(-6.57851f, -41.4337f, 11.5384f)); 
  data[9].mAabb = Aabb(Vector3(-14.6695f, 2.57734f, -47.7494f), Vector3(-12.5107f, 4.22008f, -42.542f)); 
  data[10].mAabb = Aabb(Vector3(18.4116f, -21.9335f, 20.5693f), Vector3(24.6024f, -17.7582f, 28.2603f)); 
  data[11].mAabb = Aabb(Vector3(-14.445f, 20.3463f, -11.0147f), Vector3(-10.6028f, 27.5967f, -5.75136f)); 
  data[12].mAabb = Aabb(Vector3(-39.9813f, -2.08929f, -8.70363f), Vector3(-36.769f, 0.70573f, -5.30603f)); 
  data[13].mAabb = Aabb(Vector3(33.3876f, -30.8351f, -11.7918f), Vector3(34.0555f, -26.6598f, -8.27173f)); 
  data[14].mAabb = Aabb(Vector3(29.9007f, -5.59498f, -36.2199f), Vector3(31.9619f, 1.85973f, -29.7115f)); 
  data[15].mAabb = Aabb(Vector3(-13.9665f, 20.4153f, -36.2737f), Vector3(-8.69508f, 27.5743f, -29.461f)); 
  data[16].mAabb = Aabb(Vector3(32.0524f, -34.7811f, -2.04467f), Vector3(39.0662f, -29.7421f, 5.77813f)); 
  data[17].mAabb = Aabb(Vector3(-8.49586f, -11.1048f, -12.6052f), Vector3(-2.30268f, -9.23058f, -9.7643f)); 
  data[18].mAabb = Aabb(Vector3(18.512f, 4.99851f, 1.01229f), Vector3(22.5852f, 12.6237f, 4.86635f)); 
  data[19].mAabb = Aabb(Vector3(-35.2454f, 34.6293f, -9.41735f), Vector3(-34.3595f, 35.3805f, -4.87558f)); 
  data[20].mAabb = Aabb(Vector3(-2.07764f, 21.2915f, -47.0687f), Vector3(-0.0767858f, 21.9274f, -40.0614f)); 
  data[21].mAabb = Aabb(Vector3(3.33373f, 16.7645f, 33.3679f), Vector3(6.48293f, 17.4966f, 34.4433f)); 
  data[22].mAabb = Aabb(Vector3(-3.24537f, 14.4228f, -13.4662f), Vector3(-0.2423f, 17.6571f, -5.61023f)); 
  data[23].mAabb = Aabb(Vector3(-28.023f, 9.48086f, 29.9457f), Vector3(-27.0281f, 15.595f, 32.2935f)); 
  data[24].mAabb = Aabb(Vector3(18.0961f, -8.85528f, -6.92336f), Vector3(20.6801f, -5.64764f, -4.59223f)); 
  data[25].mAabb = Aabb(Vector3(-26.7249f, 37.536f, -5.99078f), Vector3(-22.3569f, 39.7809f, -3.02134f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest8(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(30); 
  data[0].mAabb = Aabb(Vector3(8.878f, -9.07409f, 45.3967f), Vector3(11.7143f, -1.74347f, 45.9184f)); 
  data[1].mAabb = Aabb(Vector3(14.2529f, 18.8256f, 22.1704f), Vector3(21.2286f, 22.0314f, 25.9136f)); 
  data[2].mAabb = Aabb(Vector3(28.3338f, -2.17212f, 25.8687f), Vector3(30.029f, 4.70854f, 28.8352f)); 
  data[3].mAabb = Aabb(Vector3(6.10369f, 5.86368f, 18.7906f), Vector3(13.714f, 10.7636f, 25.7023f)); 
  data[4].mAabb = Aabb(Vector3(19.928f, -26.2694f, 19.7644f), Vector3(23.4805f, -20.5243f, 26.2902f)); 
  data[5].mAabb = Aabb(Vector3(19.5083f, -27.6245f, -14.963f), Vector3(22.2015f, -24.9979f, -7.82673f)); 
  data[6].mAabb = Aabb(Vector3(-39.0867f, -28.9217f, 2.87403f), Vector3(-35.1517f, -23.1734f, 9.19107f)); 
  data[7].mAabb = Aabb(Vector3(-21.323f, 6.47968f, -21.5418f), Vector3(-20.6447f, 12.7241f, -19.3609f)); 
  data[8].mAabb = Aabb(Vector3(0.119425f, -42.753f, 24.4772f), Vector3(7.90488f, -40.1422f, 27.0041f)); 
  data[9].mAabb = Aabb(Vector3(18.6774f, -3.81513f, -20.9002f), Vector3(25.3968f, 3.97148f, -16.0312f)); 
  data[10].mAabb = Aabb(Vector3(23.1831f, -27.8873f, 28.5629f), Vector3(26.3685f, -21.0528f, 33.7727f)); 
  data[11].mAabb = Aabb(Vector3(22.2306f, -18.4234f, -21.0411f), Vector3(26.8807f, -10.4598f, -20.4163f)); 
  data[12].mAabb = Aabb(Vector3(-21.8295f, 36.2089f, 3.29557f), Vector3(-14.9523f, 37.3965f, 4.57254f)); 
  data[13].mAabb = Aabb(Vector3(24.9341f, 22.8548f, 27.5455f), Vector3(32.7334f, 26.2301f, 35.237f)); 
  data[14].mAabb = Aabb(Vector3(-15.5964f, -11.1938f, -31.1854f), Vector3(-10.0014f, -5.63521f, -28.9918f)); 
  data[15].mAabb = Aabb(Vector3(9.84017f, -46.6552f, 3.74175f), Vector3(14.3868f, -45.9627f, 6.97421f)); 
  data[16].mAabb = Aabb(Vector3(23.7127f, -18.3858f, -36.0519f), Vector3(24.7355f, -12.654f, -30.16f)); 
  data[17].mAabb = Aabb(Vector3(14.2559f, -24.481f, 0.117932f), Vector3(14.988f, -18.2078f, 6.85339f)); 
  data[18].mAabb = Aabb(Vector3(24.5326f, 12.6172f, 6.57282f), Vector3(29.7213f, 13.0757f, 13.187f)); 
  data[19].mAabb = Aabb(Vector3(5.78981f, -0.995328f, -32.6024f), Vector3(13.4282f, 3.27785f, -24.6773f)); 
  data[20].mAabb = Aabb(Vector3(-12.733f, -20.7397f, -35.1604f), Vector3(-8.62589f, -13.7628f, -27.358f)); 
  data[21].mAabb = Aabb(Vector3(0.0717379f, 27.1426f, -32.3614f), Vector3(2.91894f, 28.0615f, -24.5592f)); 
  data[22].mAabb = Aabb(Vector3(-29.4304f, 20.5519f, 12.7169f), Vector3(-26.9468f, 26.2995f, 17.255f)); 
  data[23].mAabb = Aabb(Vector3(15.9238f, 16.6706f, -33.0885f), Vector3(17.4188f, 20.2683f, -28.6694f)); 
  data[24].mAabb = Aabb(Vector3(-6.30509f, -49.2207f, 2.22465f), Vector3(1.38967f, -42.6527f, 6.88053f)); 
  data[25].mAabb = Aabb(Vector3(18.8269f, 21.1307f, 35.5689f), Vector3(25.6567f, 25.9635f, 37.5716f)); 
  data[26].mAabb = Aabb(Vector3(-7.72038f, -16.7438f, -30.9831f), Vector3(-2.66811f, -12.7204f, -29.1516f)); 
  data[27].mAabb = Aabb(Vector3(-2.70512f, 0.864467f, 42.1455f), Vector3(5.28235f, 7.87037f, 46.3876f)); 
  data[28].mAabb = Aabb(Vector3(-14.1748f, -41.7321f, 18.0628f), Vector3(-11.2058f, -37.288f, 25.8313f)); 
  data[29].mAabb = Aabb(Vector3(-26.421f, -37.5747f, 1.7898f), Vector3(-19.7224f, -31.6895f, 3.73661f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest9(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mAabb = Aabb(Vector3(-5.97936f, 19.2607f, -12.8838f), Vector3(-2.39388f, 25.3421f, -10.9054f)); 
  data[1].mAabb = Aabb(Vector3(-24.1228f, 26.513f, 9.7922f), Vector3(-17.0272f, 33.3218f, 10.8395f)); 
  data[2].mAabb = Aabb(Vector3(0.65465f, -24.6874f, 30.0307f), Vector3(1.78039f, -16.7305f, 36.2898f)); 
  data[3].mAabb = Aabb(Vector3(-25.061f, 25.8936f, 13.4711f), Vector3(-18.3798f, 28.3231f, 17.1373f)); 
  data[4].mAabb = Aabb(Vector3(-17.4028f, 21.8462f, 20.0576f), Vector3(-11.5093f, 27.6557f, 26.9733f)); 
  data[5].mAabb = Aabb(Vector3(-15.41f, -28.9515f, 19.1732f), Vector3(-13.0411f, -22.094f, 20.2817f)); 
  data[6].mAabb = Aabb(Vector3(-15.3064f, -6.77781f, 2.45822f), Vector3(-11.3712f, -2.97037f, 4.37928f)); 
  data[7].mAabb = Aabb(Vector3(12.9618f, -25.7564f, 18.1248f), Vector3(14.7406f, -24.3858f, 21.0521f)); 
  data[8].mAabb = Aabb(Vector3(-11.1207f, 32.1171f, 15.9896f), Vector3(-3.54377f, 34.2706f, 20.5166f)); 
  data[9].mAabb = Aabb(Vector3(21.6103f, 6.2537f, 10.1671f), Vector3(27.4145f, 7.27832f, 15.9203f)); 
  data[10].mAabb = Aabb(Vector3(15.5299f, 1.3512f, 5.65404f), Vector3(16.6566f, 4.57461f, 6.75497f)); 
  data[11].mAabb = Aabb(Vector3(30.9691f, 13.9785f, 3.85245f), Vector3(33.1481f, 14.7753f, 5.76006f)); 
  data[12].mAabb = Aabb(Vector3(-11.4364f, -2.39335f, 27.9748f), Vector3(-7.57426f, 5.46725f, 34.8678f)); 
  data[13].mAabb = Aabb(Vector3(-0.364495f, 9.26032f, 29.9485f), Vector3(1.74491f, 14.6099f, 33.119f)); 
  data[14].mAabb = Aabb(Vector3(10.2684f, 22.6799f, -39.898f), Vector3(18.0741f, 29.9723f, -34.166f)); 
  data[15].mAabb = Aabb(Vector3(19.2449f, -39.8685f, 11.2788f), Vector3(20.2201f, -34.5022f, 15.694f)); 
  data[16].mAabb = Aabb(Vector3(11.658f, -22.9548f, -25.7198f), Vector3(19.2663f, -20.2231f, -21.4415f)); 
  data[17].mAabb = Aabb(Vector3(-31.8015f, 15.6398f, -9.68303f), Vector3(-25.4782f, 18.4701f, -4.05856f)); 
  data[18].mAabb = Aabb(Vector3(-21.5527f, 38.0588f, 16.8841f), Vector3(-19.9006f, 42.9816f, 18.9167f)); 
  data[19].mAabb = Aabb(Vector3(-31.1126f, -15.1587f, -17.0486f), Vector3(-24.5576f, -13.753f, -15.2482f)); 
  data[20].mAabb = Aabb(Vector3(-47.1841f, 1.09704f, -6.75198f), Vector3(-40.3032f, 8.79227f, -5.43025f)); 
  data[21].mAabb = Aabb(Vector3(-36.136f, -7.91261f, -30.955f), Vector3(-35.4071f, -4.43105f, -24.8762f)); 
  data[22].mAabb = Aabb(Vector3(-9.083f, 47.6379f, 3.82257f), Vector3(-4.60803f, 49.4284f, 6.6932f)); 
  data[23].mAabb = Aabb(Vector3(37.6028f, -8.47625f, -6.57239f), Vector3(38.4906f, -2.43151f, 1.38772f)); 
  data[24].mAabb = Aabb(Vector3(0.200895f, 10.3384f, 11.0321f), Vector3(3.48229f, 18.1616f, 15.8058f)); 
  data[25].mAabb = Aabb(Vector3(5.95274f, -8.67676f, -9.05396f), Vector3(11.6711f, -2.13427f, -1.0797f)); 
  data[26].mAabb = Aabb(Vector3(-23.6199f, 5.35256f, 0.0475942f), Vector3(-22.3079f, 9.98664f, 3.72468f)); 
  data[27].mAabb = Aabb(Vector3(-28.2579f, -4.56725f, 28.951f), Vector3(-26.8921f, -1.27077f, 36.8369f)); 
  data[28].mAabb = Aabb(Vector3(-30.7807f, -3.79582f, -0.848413f), Vector3(-28.9585f, 1.71012f, -0.0620002f)); 
  data[29].mAabb = Aabb(Vector3(23.7196f, -7.77634f, -6.76918f), Vector3(27.1251f, -6.14265f, -0.213943f)); 
  data[30].mAabb = Aabb(Vector3(-0.700713f, -2.53377f, 36.3522f), Vector3(0.381424f, 3.96488f, 40.2627f)); 
  data[31].mAabb = Aabb(Vector3(-2.5178f, 30.3331f, -27.189f), Vector3(3.27413f, 33.0252f, -25.0207f)); 
  data[32].mAabb = Aabb(Vector3(8.95076f, 24.2407f, -24.8487f), Vector3(14.5045f, 30.4093f, -19.4956f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest10(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(58); 
  data[0].mAabb = Aabb(Vector3(-28.7271f, -24.2945f, -15.3296f), Vector3(-23.381f, -21.1112f, -12.2523f)); 
  data[1].mAabb = Aabb(Vector3(18.7121f, -19.8695f, -7.94371f), Vector3(21.5821f, -15.2352f, -6.85207f)); 
  data[2].mAabb = Aabb(Vector3(-26.2467f, 3.3975f, -12.5653f), Vector3(-23.1994f, 10.7778f, -5.8623f)); 
  data[3].mAabb = Aabb(Vector3(8.98595f, 4.42646f, 19.49f), Vector3(14.4817f, 12.1593f, 22.697f)); 
  data[4].mAabb = Aabb(Vector3(1.95203f, 31.6778f, -9.29026f), Vector3(8.92963f, 33.9229f, -7.34252f)); 
  data[5].mAabb = Aabb(Vector3(8.37333f, -4.7145f, 11.9044f), Vector3(16.2344f, -0.789232f, 12.7952f)); 
  data[6].mAabb = Aabb(Vector3(-28.4613f, 4.30207f, -10.6161f), Vector3(-23.647f, 10.2371f, -8.44108f)); 
  data[7].mAabb = Aabb(Vector3(21.5587f, 9.72688f, -27.129f), Vector3(25.5647f, 10.5864f, -21.4349f)); 
  data[8].mAabb = Aabb(Vector3(-11.9054f, -0.434438f, -19.7525f), Vector3(-10.7416f, 1.35558f, -18.4741f)); 
  data[9].mAabb = Aabb(Vector3(8.41551f, -21.0245f, -13.2297f), Vector3(9.92349f, -17.1551f, -8.41214f)); 
  data[10].mAabb = Aabb(Vector3(-18.393f, -31.1176f, -2.3385f), Vector3(-13.8306f, -28.1305f, 4.44103f)); 
  data[11].mAabb = Aabb(Vector3(12.054f, 1.05543f, -32.6263f), Vector3(12.5863f, 6.72745f, -26.3549f)); 
  data[12].mAabb = Aabb(Vector3(26.2832f, -0.24919f, 29.0246f), Vector3(29.312f, 6.87384f, 36.9483f)); 
  data[13].mAabb = Aabb(Vector3(-38.0782f, 19.181f, 15.4344f), Vector3(-33.0591f, 20.4373f, 16.2948f)); 
  data[14].mAabb = Aabb(Vector3(-43.5978f, 16.3456f, -13.1964f), Vector3(-38.7079f, 22.1403f, -6.40805f)); 
  data[15].mAabb = Aabb(Vector3(17.3899f, 8.00392f, -7.11585f), Vector3(25.2106f, 12.9256f, -5.79899f)); 
  data[16].mAabb = Aabb(Vector3(0.930146f, -34.6329f, -7.7703f), Vector3(5.9126f, -29.3927f, -6.75612f)); 
  data[17].mAabb = Aabb(Vector3(34.6378f, 19.9836f, 1.93147f), Vector3(36.4376f, 24.6167f, 5.41791f)); 
  data[18].mAabb = Aabb(Vector3(7.51867f, -11.9373f, -20.9986f), Vector3(13.6694f, -9.23965f, -20.4228f)); 
  data[19].mAabb = Aabb(Vector3(46.2668f, 9.61457f, -1.90673f), Vector3(50.5696f, 15.0672f, 5.17479f)); 
  data[20].mAabb = Aabb(Vector3(7.48806f, -3.02117f, 16.508f), Vector3(9.23564f, 0.728518f, 21.7057f)); 
  data[21].mAabb = Aabb(Vector3(-20.945f, 29.2085f, -2.72495f), Vector3(-18.3272f, 35.2201f, -1.00776f)); 
  data[22].mAabb = Aabb(Vector3(-24.3577f, 11.9734f, 25.9343f), Vector3(-20.9021f, 14.4873f, 27.9683f)); 
  data[23].mAabb = Aabb(Vector3(-34.2172f, -27.3933f, -2.10146f), Vector3(-28.782f, -20.809f, 0.437961f)); 
  data[24].mAabb = Aabb(Vector3(3.20048f, 14.4055f, -33.9048f), Vector3(8.13701f, 16.7466f, -32.48f)); 
  data[25].mAabb = Aabb(Vector3(6.13074f, -43.5086f, 6.62716f), Vector3(7.64684f, -36.782f, 9.78401f)); 
  data[26].mAabb = Aabb(Vector3(-9.78825f, 12.3869f, -16.0602f), Vector3(-6.16334f, 18.8721f, -12.9264f)); 
  data[27].mAabb = Aabb(Vector3(17.8784f, 29.8524f, 3.62595f), Vector3(20.8668f, 37.2958f, 9.81867f)); 
  data[28].mAabb = Aabb(Vector3(-31.2292f, 17.6983f, -27.7385f), Vector3(-29.6516f, 20.9046f, -25.6479f)); 
  data[29].mAabb = Aabb(Vector3(13.3942f, 37.0201f, -1.48387f), Vector3(18.2718f, 43.2636f, 5.2829f)); 
  data[30].mAabb = Aabb(Vector3(-40.2756f, 10.4196f, 5.45819f), Vector3(-38.9863f, 14.3906f, 7.4994f)); 
  data[31].mAabb = Aabb(Vector3(6.27771f, -3.98394f, 1.3118f), Vector3(14.1645f, 3.66444f, 6.9314f)); 
  data[32].mAabb = Aabb(Vector3(-19.4272f, 22.3156f, 3.31784f), Vector3(-18.4358f, 29.525f, 5.04129f)); 
  data[33].mAabb = Aabb(Vector3(-7.99396f, -25.2172f, 41.959f), Vector3(-3.55495f, -18.1012f, 45.152f)); 
  data[34].mAabb = Aabb(Vector3(9.22193f, 1.20998f, 27.4832f), Vector3(14.5634f, 8.30494f, 33.0638f)); 
  data[35].mAabb = Aabb(Vector3(-29.5481f, 2.58287f, -24.8788f), Vector3(-22.953f, 3.92339f, -19.1004f)); 
  data[36].mAabb = Aabb(Vector3(8.69452f, -11.9658f, -16.474f), Vector3(11.513f, -8.55896f, -15.6514f)); 
  data[37].mAabb = Aabb(Vector3(6.29545f, 3.63907f, 10.0738f), Vector3(6.95035f, 9.33335f, 12.7179f)); 
  data[38].mAabb = Aabb(Vector3(9.63654f, -35.452f, 9.19011f), Vector3(11.3739f, -32.7124f, 15.2395f)); 
  data[39].mAabb = Aabb(Vector3(-20.8405f, 0.441804f, -3.08126f), Vector3(-14.9795f, 3.00998f, 2.97693f)); 
  data[40].mAabb = Aabb(Vector3(-53.6954f, -2.15858f, 1.52114f), Vector3(-45.9016f, 2.96072f, 5.57003f)); 
  data[41].mAabb = Aabb(Vector3(-16.1198f, 31.2455f, 0.514035f), Vector3(-15.7077f, 37.8279f, 7.22931f)); 
  data[42].mAabb = Aabb(Vector3(0.727133f, -18.4447f, -30.1121f), Vector3(5.37605f, -14.1521f, -29.331f)); 
  data[43].mAabb = Aabb(Vector3(18.8755f, -27.6628f, -18.3244f), Vector3(25.0775f, -24.4445f, -13.444f)); 
  data[44].mAabb = Aabb(Vector3(43.2372f, 14.8199f, -0.582003f), Vector3(46.112f, 18.1692f, 7.38065f)); 
  data[45].mAabb = Aabb(Vector3(14.2299f, -33.4786f, 20.1799f), Vector3(18.5233f, -25.8325f, 27.9461f)); 
  data[46].mAabb = Aabb(Vector3(6.97741f, -28.5695f, 27.953f), Vector3(12.9412f, -23.029f, 29.2132f)); 
  data[47].mAabb = Aabb(Vector3(32.3269f, 10.558f, -27.484f), Vector3(35.2708f, 18.4581f, -26.0342f)); 
  data[48].mAabb = Aabb(Vector3(-23.3347f, 1.97863f, -43.2081f), Vector3(-19.0158f, 6.73308f, -42.0952f)); 
  data[49].mAabb = Aabb(Vector3(-36.9944f, -6.80302f, -23.084f), Vector3(-30.8031f, -5.63205f, -22.4361f)); 
  data[50].mAabb = Aabb(Vector3(-5.81711f, -14.8262f, 24.986f), Vector3(-0.278232f, -10.9106f, 26.3722f)); 
  data[51].mAabb = Aabb(Vector3(32.2767f, -28.1163f, -17.4647f), Vector3(39.4895f, -26.7394f, -15.2071f)); 
  data[52].mAabb = Aabb(Vector3(-21.5211f, -24.8953f, 35.008f), Vector3(-14.8135f, -22.7302f, 41.8865f)); 
  data[53].mAabb = Aabb(Vector3(-14.5854f, 2.54486f, 32.5435f), Vector3(-9.3123f, 8.9157f, 39.3228f)); 
  data[54].mAabb = Aabb(Vector3(8.50307f, -22.8689f, 36.2925f), Vector3(9.10347f, -20.9792f, 39.9801f)); 
  data[55].mAabb = Aabb(Vector3(-30.0695f, 26.891f, -29.0282f), Vector3(-26.7992f, 27.4424f, -27.0967f)); 
  data[56].mAabb = Aabb(Vector3(-17.5956f, -5.82074f, -28.5461f), Vector3(-12.4712f, 1.03602f, -26.693f)); 
  data[57].mAabb = Aabb(Vector3(-9.6747f, -7.71577f, 32.4173f), Vector3(-2.18102f, -3.67245f, 36.5896f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest11(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(41); 
  data[0].mAabb = Aabb(Vector3(-3.97087f, -6.65799f, -3.73774f), Vector3(0.713774f, 0.373888f, 0.688291f)); 
  data[1].mAabb = Aabb(Vector3(-24.9751f, -12.018f, -22.7622f), Vector3(-18.1371f, -11.1119f, -16.9394f)); 
  data[2].mAabb = Aabb(Vector3(5.43808f, 27.0093f, 31.0116f), Vector3(11.6262f, 33.822f, 31.9353f)); 
  data[3].mAabb = Aabb(Vector3(-25.7996f, -24.5864f, -37.9453f), Vector3(-19.189f, -22.7052f, -29.9467f)); 
  data[4].mAabb = Aabb(Vector3(-30.9258f, -22.4114f, 18.779f), Vector3(-23.1197f, -16.1672f, 22.2543f)); 
  data[5].mAabb = Aabb(Vector3(-17.5504f, -2.14432f, 11.9735f), Vector3(-14.9173f, 1.22266f, 17.9234f)); 
  data[6].mAabb = Aabb(Vector3(26.9993f, -29.2683f, -23.4731f), Vector3(30.8789f, -26.104f, -19.694f)); 
  data[7].mAabb = Aabb(Vector3(-9.17352f, 17.8089f, 35.3394f), Vector3(-5.99904f, 23.6704f, 37.4082f)); 
  data[8].mAabb = Aabb(Vector3(15.4741f, 17.6954f, 30.748f), Vector3(18.5244f, 21.8046f, 33.396f)); 
  data[9].mAabb = Aabb(Vector3(-35.8833f, -3.32102f, -25.3638f), Vector3(-33.3173f, 4.47557f, -17.9845f)); 
  data[10].mAabb = Aabb(Vector3(16.5038f, -27.224f, -28.2674f), Vector3(21.1119f, -20.3081f, -27.8152f)); 
  data[11].mAabb = Aabb(Vector3(33.9693f, -4.18178f, -29.3382f), Vector3(35.2873f, -2.94146f, -21.4584f)); 
  data[12].mAabb = Aabb(Vector3(-37.6934f, -26.8178f, -3.78936f), Vector3(-33.0911f, -26.3793f, 0.828709f)); 
  data[13].mAabb = Aabb(Vector3(-38.3933f, -26.5856f, -0.545408f), Vector3(-36.6652f, -23.4647f, 6.18124f)); 
  data[14].mAabb = Aabb(Vector3(-11.3385f, 32.2148f, 32.5436f), Vector3(-7.41092f, 36.1591f, 34.2381f)); 
  data[15].mAabb = Aabb(Vector3(-21.8161f, 6.2024f, 16.6023f), Vector3(-15.5202f, 8.23318f, 17.065f)); 
  data[16].mAabb = Aabb(Vector3(3.92477f, -51.3808f, -11.3822f), Vector3(5.92006f, -46.0346f, -6.48096f)); 
  data[17].mAabb = Aabb(Vector3(17.2259f, 21.1194f, 8.80031f), Vector3(21.6816f, 22.0014f, 10.9116f)); 
  data[18].mAabb = Aabb(Vector3(28.0243f, -31.8227f, 10.414f), Vector3(29.4286f, -28.5829f, 13.4824f)); 
  data[19].mAabb = Aabb(Vector3(-10.3034f, 27.6533f, 35.072f), Vector3(-4.96402f, 32.7185f, 42.4135f)); 
  data[20].mAabb = Aabb(Vector3(0.555726f, -19.6126f, 31.3689f), Vector3(7.78151f, -15.3065f, 36.8519f)); 
  data[21].mAabb = Aabb(Vector3(-26.7684f, 31.7136f, -3.59666f), Vector3(-23.4863f, 38.8965f, -2.27539f)); 
  data[22].mAabb = Aabb(Vector3(11.6362f, -36.6616f, 4.70627f), Vector3(12.3426f, -30.4236f, 5.65736f)); 
  data[23].mAabb = Aabb(Vector3(20.4278f, 20.8762f, 15.4392f), Vector3(22.6114f, 21.291f, 16.7099f)); 
  data[24].mAabb = Aabb(Vector3(-8.78721f, 27.2466f, 16.3671f), Vector3(-0.969746f, 35.0072f, 19.1612f)); 
  data[25].mAabb = Aabb(Vector3(33.5964f, 16.8865f, -5.0077f), Vector3(34.2344f, 20.0681f, -0.681874f)); 
  data[26].mAabb = Aabb(Vector3(30.6769f, -13.4662f, 9.47256f), Vector3(37.5193f, -5.96415f, 15.1569f)); 
  data[27].mAabb = Aabb(Vector3(10.0946f, -38.4558f, -3.51305f), Vector3(13.238f, -32.8935f, -0.58698f)); 
  data[28].mAabb = Aabb(Vector3(-44.2967f, 7.31796f, -3.14842f), Vector3(-43.661f, 10.196f, 0.0893756f)); 
  data[29].mAabb = Aabb(Vector3(-31.8438f, -26.673f, -28.2253f), Vector3(-26.9395f, -24.1345f, -22.4849f)); 
  data[30].mAabb = Aabb(Vector3(27.6154f, 15.3237f, 29.6156f), Vector3(32.9652f, 19.2805f, 35.8916f)); 
  data[31].mAabb = Aabb(Vector3(14.6811f, -29.6451f, -28.3201f), Vector3(22.6217f, -27.9265f, -22.3788f)); 
  data[32].mAabb = Aabb(Vector3(-22.0738f, -8.47361f, -44.2344f), Vector3(-14.4224f, -3.41299f, -38.7688f)); 
  data[33].mAabb = Aabb(Vector3(-22.9862f, -32.8457f, -8.68707f), Vector3(-22.1738f, -29.4358f, -6.64656f)); 
  data[34].mAabb = Aabb(Vector3(7.60946f, 40.4072f, -1.41952f), Vector3(14.6455f, 45.4785f, -0.766239f)); 
  data[35].mAabb = Aabb(Vector3(22.0265f, -10.296f, 20.581f), Vector3(27.3056f, -2.39877f, 26.0806f)); 
  data[36].mAabb = Aabb(Vector3(38.0695f, 18.4216f, -9.60393f), Vector3(43.3243f, 22.423f, -6.99887f)); 
  data[37].mAabb = Aabb(Vector3(-19.7625f, 14.7404f, -10.0516f), Vector3(-19.1658f, 16.9347f, -6.52545f)); 
  data[38].mAabb = Aabb(Vector3(6.8904f, 6.96115f, -0.982882f), Vector3(13.1557f, 12.9518f, 6.8805f)); 
  data[39].mAabb = Aabb(Vector3(-9.30151f, -32.9772f, 2.56789f), Vector3(-5.72741f, -29.0468f, 3.83883f)); 
  data[40].mAabb = Aabb(Vector3(-32.1892f, -6.46091f, -40.0962f), Vector3(-29.47f, -4.56745f, -35.6611f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest12(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(72); 
  data[0].mAabb = Aabb(Vector3(-20.6916f, -5.73656f, -28.3981f), Vector3(-15.2561f, -1.11361f, -23.4291f)); 
  data[1].mAabb = Aabb(Vector3(-31.9965f, 20.5576f, 2.83253f), Vector3(-30.5732f, 26.5221f, 9.66657f)); 
  data[2].mAabb = Aabb(Vector3(-14.4914f, 39.9449f, 15.647f), Vector3(-9.38855f, 41.8268f, 18.8435f)); 
  data[3].mAabb = Aabb(Vector3(-28.4841f, 27.2677f, -6.79111f), Vector3(-24.971f, 32.033f, -1.31277f)); 
  data[4].mAabb = Aabb(Vector3(-3.50134f, 23.8058f, 14.4698f), Vector3(-1.69322f, 24.9308f, 20.4327f)); 
  data[5].mAabb = Aabb(Vector3(28.7992f, 18.9845f, -21.7356f), Vector3(35.8248f, 19.9451f, -19.2947f)); 
  data[6].mAabb = Aabb(Vector3(-35.8012f, -34.7673f, -15.0403f), Vector3(-27.9443f, -31.3472f, -8.7198f)); 
  data[7].mAabb = Aabb(Vector3(21.7457f, 30.2254f, -9.92531f), Vector3(28.598f, 34.1732f, -3.61547f)); 
  data[8].mAabb = Aabb(Vector3(17.6767f, -19.3287f, -18.8516f), Vector3(21.7785f, -17.445f, -17.5684f)); 
  data[9].mAabb = Aabb(Vector3(15.4513f, 5.04676f, 25.2612f), Vector3(22.8362f, 7.09516f, 31.0933f)); 
  data[10].mAabb = Aabb(Vector3(-14.6693f, 24.2923f, 30.2692f), Vector3(-10.4692f, 28.8635f, 36.0815f)); 
  data[11].mAabb = Aabb(Vector3(-12.1914f, -20.472f, 1.40648f), Vector3(-8.24523f, -14.6112f, 7.51871f)); 
  data[12].mAabb = Aabb(Vector3(-15.7539f, -9.02198f, -9.41121f), Vector3(-12.0012f, -1.51995f, -5.83084f)); 
  data[13].mAabb = Aabb(Vector3(-3.54942f, 40.6897f, 21.2977f), Vector3(0.293739f, 43.8438f, 24.6721f)); 
  data[14].mAabb = Aabb(Vector3(-14.8671f, 28.6307f, 4.94041f), Vector3(-7.92335f, 29.8226f, 7.37384f)); 
  data[15].mAabb = Aabb(Vector3(-32.0592f, -14.6019f, -18.4277f), Vector3(-24.1117f, -12.8277f, -11.1305f)); 
  data[16].mAabb = Aabb(Vector3(19.079f, 39.8066f, -2.6755f), Vector3(26.9721f, 46.4551f, -1.22202f)); 
  data[17].mAabb = Aabb(Vector3(35.7905f, -4.74626f, 20.6963f), Vector3(42.5109f, 1.67422f, 24.0308f)); 
  data[18].mAabb = Aabb(Vector3(-23.6664f, -18.5332f, -32.2476f), Vector3(-18.3353f, -14.7418f, -25.106f)); 
  data[19].mAabb = Aabb(Vector3(5.77652f, 7.8442f, -29.5455f), Vector3(8.56945f, 14.7977f, -27.5068f)); 
  data[20].mAabb = Aabb(Vector3(9.65679f, -21.1345f, -1.88639f), Vector3(13.4691f, -14.4672f, 2.54335f)); 
  data[21].mAabb = Aabb(Vector3(11.4171f, 4.72789f, 36.0167f), Vector3(19.0321f, 11.2305f, 43.0085f)); 
  data[22].mAabb = Aabb(Vector3(16.2332f, -11.8611f, -41.2413f), Vector3(20.3104f, -7.84862f, -36.4019f)); 
  data[23].mAabb = Aabb(Vector3(14.2722f, 19.9796f, 31.5074f), Vector3(15.7535f, 27.4093f, 34.2702f)); 
  data[24].mAabb = Aabb(Vector3(-20.6909f, -18.4598f, -15.1009f), Vector3(-16.517f, -14.1258f, -13.6117f)); 
  data[25].mAabb = Aabb(Vector3(7.58212f, 18.5403f, -13.0462f), Vector3(9.6384f, 24.7541f, -5.74361f)); 
  data[26].mAabb = Aabb(Vector3(-20.4545f, 10.4344f, -10.6359f), Vector3(-14.0961f, 13.3001f, -4.15768f)); 
  data[27].mAabb = Aabb(Vector3(-20.221f, -25.8126f, 31.8541f), Vector3(-17.5224f, -19.2915f, 33.2394f)); 
  data[28].mAabb = Aabb(Vector3(-9.92398f, 42.3705f, 1.46891f), Vector3(-4.14713f, 45.5353f, 9.41115f)); 
  data[29].mAabb = Aabb(Vector3(-23.384f, 29.2782f, -34.7428f), Vector3(-21.6772f, 31.9976f, -27.2276f)); 
  data[30].mAabb = Aabb(Vector3(-38.34f, -5.69149f, -14.6633f), Vector3(-36.7894f, -3.77994f, -7.5542f)); 
  data[31].mAabb = Aabb(Vector3(33.4559f, 4.29003f, 1.02437f), Vector3(38.8512f, 8.19257f, 2.72695f)); 
  data[32].mAabb = Aabb(Vector3(-6.84238f, 18.8405f, 3.26901f), Vector3(-0.571031f, 20.1137f, 6.32427f)); 
  data[33].mAabb = Aabb(Vector3(-47.1205f, -3.30589f, -2.22192f), Vector3(-41.4084f, 1.77792f, 4.31128f)); 
  data[34].mAabb = Aabb(Vector3(11.9344f, 2.26641f, 3.24452f), Vector3(15.2539f, 5.26066f, 5.12824f)); 
  data[35].mAabb = Aabb(Vector3(-7.05168f, 31.2279f, 15.2642f), Vector3(-1.73848f, 37.2538f, 18.5934f)); 
  data[36].mAabb = Aabb(Vector3(10.2295f, -28.8427f, 37.0039f), Vector3(13.0517f, -25.8642f, 38.6327f)); 
  data[37].mAabb = Aabb(Vector3(2.47148f, 37.2923f, 3.25066f), Vector3(9.67522f, 44.4566f, 7.48f)); 
  data[38].mAabb = Aabb(Vector3(4.2776f, 9.49871f, -19.9516f), Vector3(5.53555f, 10.8211f, -13.1442f)); 
  data[39].mAabb = Aabb(Vector3(31.5709f, 23.2528f, -24.426f), Vector3(35.432f, 30.3433f, -18.8386f)); 
  data[40].mAabb = Aabb(Vector3(33.703f, -5.75769f, 21.6543f), Vector3(36.8264f, -4.79454f, 22.7765f)); 
  data[41].mAabb = Aabb(Vector3(-28.5542f, -18.3961f, -13.1766f), Vector3(-24.0045f, -11.3392f, -9.26622f)); 
  data[42].mAabb = Aabb(Vector3(5.9065f, -17.7294f, 25.7931f), Vector3(6.78267f, -15.425f, 26.4313f)); 
  data[43].mAabb = Aabb(Vector3(-17.1121f, 1.70095f, -14.007f), Vector3(-10.5698f, 3.0642f, -13.061f)); 
  data[44].mAabb = Aabb(Vector3(34.5302f, 19.6227f, 22.6048f), Vector3(37.6919f, 24.6044f, 24.5368f)); 
  data[45].mAabb = Aabb(Vector3(42.2252f, 18.8569f, -1.46031f), Vector3(44.3827f, 23.549f, 2.87015f)); 
  data[46].mAabb = Aabb(Vector3(18.8121f, -27.1578f, -26.2951f), Vector3(26.3535f, -25.8279f, -25.5537f)); 
  data[47].mAabb = Aabb(Vector3(38.392f, 4.03131f, -21.0624f), Vector3(41.7224f, 8.08066f, -17.4516f)); 
  data[48].mAabb = Aabb(Vector3(24.8515f, 26.5229f, -15.4413f), Vector3(32.5825f, 29.6394f, -13.4249f)); 
  data[49].mAabb = Aabb(Vector3(-2.5565f, -26.2104f, -20.248f), Vector3(-1.73251f, -18.2306f, -16.1929f)); 
  data[50].mAabb = Aabb(Vector3(-0.732902f, -8.30367f, -18.1536f), Vector3(6.83175f, -5.80183f, -11.2643f)); 
  data[51].mAabb = Aabb(Vector3(-1.39083f, -9.99937f, -31.3538f), Vector3(5.60626f, -5.2869f, -24.4924f)); 
  data[52].mAabb = Aabb(Vector3(-6.66252f, 32.387f, -1.97608f), Vector3(-4.1674f, 39.9748f, 1.89793f)); 
  data[53].mAabb = Aabb(Vector3(-12.624f, 15.372f, -41.0416f), Vector3(-12.1855f, 17.2766f, -38.0652f)); 
  data[54].mAabb = Aabb(Vector3(-14.3654f, 25.7471f, -7.54157f), Vector3(-7.12289f, 33.6683f, -4.52946f)); 
  data[55].mAabb = Aabb(Vector3(31.2775f, -10.2908f, -30.0179f), Vector3(36.2639f, -5.04675f, -24.0759f)); 
  data[56].mAabb = Aabb(Vector3(-28.8898f, -27.2277f, 25.0086f), Vector3(-22.085f, -24.5178f, 28.4774f)); 
  data[57].mAabb = Aabb(Vector3(5.15371f, -27.5564f, -10.1156f), Vector3(12.5873f, -25.3853f, -9.14041f)); 
  data[58].mAabb = Aabb(Vector3(34.2492f, -34.9756f, -9.01249f), Vector3(34.8935f, -27.0575f, -7.3106f)); 
  data[59].mAabb = Aabb(Vector3(2.19179f, -35.0746f, 27.8747f), Vector3(9.4359f, -29.4889f, 30.7743f)); 
  data[60].mAabb = Aabb(Vector3(11.1952f, -5.86583f, -23.1915f), Vector3(13.2538f, 1.93424f, -16.4835f)); 
  data[61].mAabb = Aabb(Vector3(-24.0206f, -2.36642f, -13.1952f), Vector3(-22.6351f, 1.03304f, -11.8584f)); 
  data[62].mAabb = Aabb(Vector3(28.5384f, -28.4756f, 0.801743f), Vector3(31.4907f, -25.8397f, 3.06423f)); 
  data[63].mAabb = Aabb(Vector3(-42.987f, -17.256f, -1.78648f), Vector3(-35.395f, -11.6582f, -0.812891f)); 
  data[64].mAabb = Aabb(Vector3(-0.562456f, -18.7968f, 14.1235f), Vector3(1.05662f, -14.5617f, 19.1904f)); 
  data[65].mAabb = Aabb(Vector3(25.747f, 3.46376f, -20.047f), Vector3(30.916f, 7.73416f, -16.3323f)); 
  data[66].mAabb = Aabb(Vector3(-18.9284f, 8.91646f, -24.2816f), Vector3(-13.9787f, 10.6796f, -17.5709f)); 
  data[67].mAabb = Aabb(Vector3(13.4588f, -23.4088f, -25.6926f), Vector3(16.5145f, -15.529f, -21.958f)); 
  data[68].mAabb = Aabb(Vector3(33.9571f, 19.2515f, -20.9461f), Vector3(37.8327f, 27.1126f, -19.3792f)); 
  data[69].mAabb = Aabb(Vector3(10.401f, -39.9703f, 4.63374f), Vector3(18.2558f, -36.7496f, 6.47688f)); 
  data[70].mAabb = Aabb(Vector3(-28.3131f, 0.715837f, 29.808f), Vector3(-27.6521f, 3.1321f, 32.3369f)); 
  data[71].mAabb = Aabb(Vector3(-2.43907f, -21.8059f, 19.8537f), Vector3(0.301674f, -15.464f, 23.6268f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest13(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(32); 
  data[0].mAabb = Aabb(Vector3(15.2187f, 31.8734f, -13.4591f), Vector3(18.7209f, 37.4181f, -6.2639f)); 
  data[1].mAabb = Aabb(Vector3(9.36388f, -32.2232f, 19.6607f), Vector3(10.568f, -27.7907f, 24.1596f)); 
  data[2].mAabb = Aabb(Vector3(3.05741f, 42.0993f, 0.284165f), Vector3(6.42393f, 48.421f, 7.8843f)); 
  data[3].mAabb = Aabb(Vector3(-4.13501f, 42.0551f, -9.68701f), Vector3(2.40423f, 44.9315f, -7.60358f)); 
  data[4].mAabb = Aabb(Vector3(-14.6435f, 20.8781f, 13.0286f), Vector3(-9.92822f, 24.1568f, 14.6787f)); 
  data[5].mAabb = Aabb(Vector3(-20.1542f, 29.7993f, 22.2695f), Vector3(-16.7155f, 37.3461f, 27.4876f)); 
  data[6].mAabb = Aabb(Vector3(22.3509f, 40.4515f, 7.44918f), Vector3(25.6416f, 41.1611f, 13.1525f)); 
  data[7].mAabb = Aabb(Vector3(-4.72947f, 3.64803f, -29.1202f), Vector3(1.00192f, 8.25451f, -22.4573f)); 
  data[8].mAabb = Aabb(Vector3(35.725f, 8.27795f, 7.80762f), Vector3(38.7485f, 12.0942f, 10.3851f)); 
  data[9].mAabb = Aabb(Vector3(43.9284f, 1.28034f, 1.97939f), Vector3(47.0402f, 2.90035f, 6.36228f)); 
  data[10].mAabb = Aabb(Vector3(17.9299f, -0.872068f, 9.79222f), Vector3(22.1191f, 3.01284f, 17.6619f)); 
  data[11].mAabb = Aabb(Vector3(-20.5557f, 11.8566f, -8.99516f), Vector3(-19.1201f, 12.5709f, -3.87911f)); 
  data[12].mAabb = Aabb(Vector3(-17.9145f, 28.614f, 10.1011f), Vector3(-10.7175f, 36.3884f, 13.4463f)); 
  data[13].mAabb = Aabb(Vector3(-15.0009f, 20.0637f, 41.4815f), Vector3(-10.5732f, 23.1693f, 43.7231f)); 
  data[14].mAabb = Aabb(Vector3(14.5264f, 24.9611f, 30.7836f), Vector3(16.8467f, 28.133f, 33.0018f)); 
  data[15].mAabb = Aabb(Vector3(-20.571f, -39.1249f, -32.4455f), Vector3(-12.7565f, -32.3725f, -27.7748f)); 
  data[16].mAabb = Aabb(Vector3(4.64656f, 14.8877f, -46.0501f), Vector3(7.73011f, 20.5727f, -40.3226f)); 
  data[17].mAabb = Aabb(Vector3(2.82261f, -10.3805f, 22.3724f), Vector3(7.51444f, -9.68101f, 29.6362f)); 
  data[18].mAabb = Aabb(Vector3(-10.3788f, -35.1073f, 6.83246f), Vector3(-7.9628f, -31.8802f, 8.6582f)); 
  data[19].mAabb = Aabb(Vector3(-18.4284f, 10.6956f, -45.3179f), Vector3(-13.683f, 12.1704f, -43.0239f)); 
  data[20].mAabb = Aabb(Vector3(12.5868f, 2.32365f, -31.1077f), Vector3(13.0736f, 6.88861f, -30.1139f)); 
  data[21].mAabb = Aabb(Vector3(16.632f, 42.928f, -5.97681f), Vector3(21.9591f, 48.8271f, 0.0883446f)); 
  data[22].mAabb = Aabb(Vector3(-0.259312f, 10.0757f, -12.9381f), Vector3(5.87357f, 17.5592f, -11.3483f)); 
  data[23].mAabb = Aabb(Vector3(-17.6886f, 28.9226f, -32.7227f), Vector3(-17.0773f, 33.3964f, -30.1271f)); 
  data[24].mAabb = Aabb(Vector3(41.5593f, -4.18319f, -26.6424f), Vector3(42.7507f, -1.94669f, -25.2938f)); 
  data[25].mAabb = Aabb(Vector3(-25.4844f, -26.7788f, -13.85f), Vector3(-20.6487f, -21.0687f, -7.3636f)); 
  data[26].mAabb = Aabb(Vector3(-1.95836f, -27.223f, -41.8363f), Vector3(3.29453f, -26.6785f, -41.1932f)); 
  data[27].mAabb = Aabb(Vector3(25.548f, 7.86077f, -37.4743f), Vector3(26.5868f, 10.6194f, -34.9963f)); 
  data[28].mAabb = Aabb(Vector3(-25.1824f, 33.3745f, -19.2673f), Vector3(-23.0419f, 33.9721f, -11.5479f)); 
  data[29].mAabb = Aabb(Vector3(-10.7664f, 25.2616f, -9.13075f), Vector3(-6.65169f, 32.7667f, -7.31336f)); 
  data[30].mAabb = Aabb(Vector3(25.4996f, -15.011f, 29.7865f), Vector3(27.9361f, -7.47856f, 36.5041f)); 
  data[31].mAabb = Aabb(Vector3(-38.4145f, 0.352018f, 25.8377f), Vector3(-33.2277f, 7.8436f, 29.1762f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest14(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(53); 
  data[0].mAabb = Aabb(Vector3(41.3409f, 14.085f, -13.4443f), Vector3(46.436f, 19.4374f, -7.81494f)); 
  data[1].mAabb = Aabb(Vector3(-17.7964f, -15.9604f, -27.9735f), Vector3(-17.2204f, -12.7447f, -24.6141f)); 
  data[2].mAabb = Aabb(Vector3(-1.25254f, -38.6572f, 22.4633f), Vector3(6.00525f, -33.6583f, 26.3492f)); 
  data[3].mAabb = Aabb(Vector3(20.4049f, 27.4343f, 30.022f), Vector3(25.626f, 28.6468f, 37.3635f)); 
  data[4].mAabb = Aabb(Vector3(-31.6782f, 21.688f, 9.75055f), Vector3(-31.0606f, 24.9001f, 17.3069f)); 
  data[5].mAabb = Aabb(Vector3(-5.62239f, 30.2529f, 24.4008f), Vector3(2.21084f, 32.1658f, 29.4893f)); 
  data[6].mAabb = Aabb(Vector3(-17.9467f, 12.9991f, -36.4399f), Vector3(-11.8602f, 15.49f, -29.2004f)); 
  data[7].mAabb = Aabb(Vector3(-8.62954f, -8.63432f, 15.638f), Vector3(-2.9116f, -7.63429f, 20.5852f)); 
  data[8].mAabb = Aabb(Vector3(21.9061f, 26.0155f, -4.27524f), Vector3(27.525f, 27.8684f, 3.56333f)); 
  data[9].mAabb = Aabb(Vector3(17.7988f, 3.99202f, -26.6556f), Vector3(21.4882f, 10.599f, -24.4597f)); 
  data[10].mAabb = Aabb(Vector3(-18.9073f, -47.6284f, -8.86517f), Vector3(-14.8899f, -43.3587f, -3.56101f)); 
  data[11].mAabb = Aabb(Vector3(34.3469f, -0.754113f, 3.48701f), Vector3(37.9672f, 4.35544f, 8.20807f)); 
  data[12].mAabb = Aabb(Vector3(-32.9453f, -26.1554f, 8.04675f), Vector3(-25.7348f, -25.0761f, 10.0917f)); 
  data[13].mAabb = Aabb(Vector3(3.47893f, 17.7395f, 18.9736f), Vector3(5.98124f, 25.5407f, 25.0135f)); 
  data[14].mAabb = Aabb(Vector3(23.9332f, -17.5604f, 14.6127f), Vector3(30.9525f, -12.2189f, 19.3307f)); 
  data[15].mAabb = Aabb(Vector3(-31.6236f, -7.47194f, 29.6794f), Vector3(-24.3027f, -5.95746f, 32.9669f)); 
  data[16].mAabb = Aabb(Vector3(-28.3464f, 36.4892f, 4.52337f), Vector3(-22.9242f, 41.1792f, 9.6371f)); 
  data[17].mAabb = Aabb(Vector3(13.8722f, 0.259683f, -21.5004f), Vector3(16.7348f, 2.7105f, -19.5241f)); 
  data[18].mAabb = Aabb(Vector3(-45.0433f, -9.64699f, -18.1172f), Vector3(-43.9122f, -5.19197f, -16.2729f)); 
  data[19].mAabb = Aabb(Vector3(3.13927f, -4.46039f, 18.4119f), Vector3(5.50404f, 1.42084f, 24.7563f)); 
  data[20].mAabb = Aabb(Vector3(-17.0825f, 4.31057f, 27.8981f), Vector3(-15.19f, 5.52004f, 30.6504f)); 
  data[21].mAabb = Aabb(Vector3(-46.4605f, 13.1518f, 1.80172f), Vector3(-43.3401f, 13.7329f, 7.30094f)); 
  data[22].mAabb = Aabb(Vector3(7.52054f, -39.6526f, -5.29498f), Vector3(8.04556f, -35.3451f, -0.346858f)); 
  data[23].mAabb = Aabb(Vector3(-25.4153f, -8.27704f, -4.94356f), Vector3(-18.0636f, -0.415048f, 2.43484f)); 
  data[24].mAabb = Aabb(Vector3(-6.54283f, 6.60457f, -35.7699f), Vector3(-5.63673f, 12.9775f, -29.3795f)); 
  data[25].mAabb = Aabb(Vector3(39.522f, 5.53633f, 7.98651f), Vector3(45.9847f, 7.99527f, 9.51513f)); 
  data[26].mAabb = Aabb(Vector3(-41.3893f, 9.52677f, -5.23094f), Vector3(-39.8686f, 14.6308f, 2.69182f)); 
  data[27].mAabb = Aabb(Vector3(-18.1018f, -3.16347f, -25.4275f), Vector3(-12.7148f, -1.22083f, -23.8996f)); 
  data[28].mAabb = Aabb(Vector3(-29.2667f, 14.9251f, -9.06494f), Vector3(-21.6828f, 22.6818f, -2.58577f)); 
  data[29].mAabb = Aabb(Vector3(-21.0751f, -20.0634f, -44.1508f), Vector3(-13.1364f, -12.9508f, -41.7002f)); 
  data[30].mAabb = Aabb(Vector3(-36.8749f, -31.0188f, -4.80809f), Vector3(-34.7254f, -24.3546f, 1.95869f)); 
  data[31].mAabb = Aabb(Vector3(-41.9514f, -14.0222f, -19.7718f), Vector3(-40.0681f, -9.98559f, -15.5731f)); 
  data[32].mAabb = Aabb(Vector3(13.2768f, -9.47351f, 14.7495f), Vector3(13.6977f, -7.91079f, 21.295f)); 
  data[33].mAabb = Aabb(Vector3(-32.0674f, -16.6541f, -3.45242f), Vector3(-30.6302f, -10.0525f, 1.05293f)); 
  data[34].mAabb = Aabb(Vector3(-29.2754f, -7.63976f, 8.60678f), Vector3(-25.6066f, -3.75949f, 14.2171f)); 
  data[35].mAabb = Aabb(Vector3(8.38673f, 41.6324f, 1.33818f), Vector3(11.8991f, 42.2627f, 7.36413f)); 
  data[36].mAabb = Aabb(Vector3(18.1f, -1.54337f, -23.9263f), Vector3(25.4774f, 6.00852f, -22.734f)); 
  data[37].mAabb = Aabb(Vector3(-3.64494f, -26.2619f, -37.3759f), Vector3(2.96852f, -22.2237f, -35.2489f)); 
  data[38].mAabb = Aabb(Vector3(20.2875f, -42.9105f, -8.13527f), Vector3(21.9129f, -36.9845f, -3.44367f)); 
  data[39].mAabb = Aabb(Vector3(16.8032f, -31.9152f, 28.8309f), Vector3(18.7094f, -26.6359f, 29.8202f)); 
  data[40].mAabb = Aabb(Vector3(13.5719f, -19.3174f, -30.4966f), Vector3(17.888f, -17.9934f, -26.0462f)); 
  data[41].mAabb = Aabb(Vector3(38.8311f, -21.0262f, -17.4784f), Vector3(40.5954f, -18.8693f, -10.4456f)); 
  data[42].mAabb = Aabb(Vector3(-39.1175f, -14.7636f, 26.5279f), Vector3(-34.6474f, -11.5511f, 27.2115f)); 
  data[43].mAabb = Aabb(Vector3(-5.20546f, 7.92902f, -9.33046f), Vector3(2.50392f, 9.9809f, -5.25234f)); 
  data[44].mAabb = Aabb(Vector3(11.9037f, -27.614f, 29.2577f), Vector3(12.4952f, -20.4599f, 36.1365f)); 
  data[45].mAabb = Aabb(Vector3(21.6157f, 19.8437f, -23.468f), Vector3(27.4724f, 20.2785f, -19.8071f)); 
  data[46].mAabb = Aabb(Vector3(-32.6796f, -14.9625f, 13.8427f), Vector3(-28.958f, -11.2646f, 20.814f)); 
  data[47].mAabb = Aabb(Vector3(23.5485f, -25.5642f, 15.2916f), Vector3(29.3279f, -21.6378f, 20.5948f)); 
  data[48].mAabb = Aabb(Vector3(16.4673f, -16.3643f, -27.411f), Vector3(24.2576f, -14.5916f, -21.5927f)); 
  data[49].mAabb = Aabb(Vector3(12.5211f, 9.79482f, 25.3617f), Vector3(18.2232f, 10.5573f, 25.9231f)); 
  data[50].mAabb = Aabb(Vector3(17.9953f, -10.779f, -34.2142f), Vector3(20.0762f, -9.03486f, -31.5386f)); 
  data[51].mAabb = Aabb(Vector3(-23.8245f, -23.2638f, -22.7129f), Vector3(-17.4852f, -17.5284f, -18.251f)); 
  data[52].mAabb = Aabb(Vector3(19.6729f, 42.6772f, -1.99748f), Vector3(25.2914f, 44.1771f, 0.394426f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest15(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(38); 
  data[0].mAabb = Aabb(Vector3(-11.4269f, 28.2195f, -29.5633f), Vector3(-8.49317f, 36.0787f, -22.1362f)); 
  data[1].mAabb = Aabb(Vector3(-16.0314f, -11.5264f, -47.3108f), Vector3(-15.5975f, -10.0674f, -44.2458f)); 
  data[2].mAabb = Aabb(Vector3(-13.6657f, 32.9853f, -26.0487f), Vector3(-8.612f, 39.4137f, -24.9146f)); 
  data[3].mAabb = Aabb(Vector3(5.37894f, 45.1759f, -7.45712f), Vector3(8.63506f, 52.4868f, -4.53314f)); 
  data[4].mAabb = Aabb(Vector3(-17.1372f, 11.0199f, 9.96888f), Vector3(-12.0207f, 17.4434f, 15.9554f)); 
  data[5].mAabb = Aabb(Vector3(-13.7526f, -6.7343f, 6.09865f), Vector3(-8.82903f, -5.08135f, 11.6208f)); 
  data[6].mAabb = Aabb(Vector3(-7.41175f, -26.2642f, 13.1639f), Vector3(-2.07049f, -25.3922f, 15.2103f)); 
  data[7].mAabb = Aabb(Vector3(24.2374f, -22.8289f, -25.4342f), Vector3(28.5412f, -21.6948f, -24.4759f)); 
  data[8].mAabb = Aabb(Vector3(-10.3025f, -40.0967f, 20.878f), Vector3(-7.27627f, -33.6857f, 22.0898f)); 
  data[9].mAabb = Aabb(Vector3(7.92497f, 22.1758f, 9.03624f), Vector3(14.4644f, 22.7254f, 9.81198f)); 
  data[10].mAabb = Aabb(Vector3(-2.11526f, -11.7581f, -5.03753f), Vector3(-0.348206f, -9.67627f, -3.16725f)); 
  data[11].mAabb = Aabb(Vector3(-13.945f, 4.4051f, -29.8948f), Vector3(-12.0314f, 5.50116f, -24.4461f)); 
  data[12].mAabb = Aabb(Vector3(5.8796f, -35.0339f, -35.1935f), Vector3(9.6265f, -34.1243f, -33.9708f)); 
  data[13].mAabb = Aabb(Vector3(23.3193f, 2.27721f, -6.1706f), Vector3(25.9399f, 3.98558f, 0.448658f)); 
  data[14].mAabb = Aabb(Vector3(-21.4234f, -47.7539f, -3.13611f), Vector3(-16.7072f, -40.7377f, -1.74874f)); 
  data[15].mAabb = Aabb(Vector3(32.2175f, 1.90363f, -36.1349f), Vector3(33.5714f, 6.44794f, -28.4378f)); 
  data[16].mAabb = Aabb(Vector3(14.3275f, -1.55136f, -10.6627f), Vector3(16.1959f, 0.886939f, -8.2675f)); 
  data[17].mAabb = Aabb(Vector3(-18.2276f, 28.9002f, -35.9303f), Vector3(-17.6008f, 34.7768f, -31.2997f)); 
  data[18].mAabb = Aabb(Vector3(-21.6686f, 1.26658f, -36.4947f), Vector3(-15.8377f, 6.47865f, -34.7019f)); 
  data[19].mAabb = Aabb(Vector3(19.6337f, 37.9895f, -9.68366f), Vector3(26.7669f, 38.6453f, -8.0161f)); 
  data[20].mAabb = Aabb(Vector3(14.8882f, 31.6742f, 11.7125f), Vector3(21.0644f, 32.5546f, 13.2746f)); 
  data[21].mAabb = Aabb(Vector3(-24.6368f, -14.6893f, 9.58834f), Vector3(-21.3714f, -12.28f, 13.5015f)); 
  data[22].mAabb = Aabb(Vector3(24.8011f, 8.41147f, 17.6481f), Vector3(28.0753f, 10.2061f, 21.7185f)); 
  data[23].mAabb = Aabb(Vector3(27.5062f, 2.58596f, 14.0512f), Vector3(30.407f, 10.5185f, 14.7518f)); 
  data[24].mAabb = Aabb(Vector3(20.0417f, 32.5383f, -23.5293f), Vector3(22.7314f, 38.5046f, -18.1642f)); 
  data[25].mAabb = Aabb(Vector3(-4.67953f, 5.31958f, 4.79064f), Vector3(1.28728f, 12.5611f, 7.97578f)); 
  data[26].mAabb = Aabb(Vector3(11.8027f, 30.8779f, -16.3442f), Vector3(13.1471f, 34.3112f, -10.8234f)); 
  data[27].mAabb = Aabb(Vector3(-0.0936913f, -29.324f, 30.2201f), Vector3(4.40656f, -23.078f, 35.9961f)); 
  data[28].mAabb = Aabb(Vector3(-18.8299f, -20.2683f, -1.07568f), Vector3(-12.778f, -13.1624f, 5.28149f)); 
  data[29].mAabb = Aabb(Vector3(21.2053f, -35.8245f, 19.608f), Vector3(27.9176f, -32.8186f, 22.8244f)); 
  data[30].mAabb = Aabb(Vector3(-2.15035f, 3.13636f, 45.1673f), Vector3(5.08888f, 8.38577f, 46.2035f)); 
  data[31].mAabb = Aabb(Vector3(19.1501f, -18.4142f, -31.4788f), Vector3(22.9197f, -11.7806f, -30.152f)); 
  data[32].mAabb = Aabb(Vector3(27.5992f, -30.9861f, 6.76252f), Vector3(29.2675f, -28.4142f, 8.61494f)); 
  data[33].mAabb = Aabb(Vector3(-21.3823f, -40.2875f, 18.2097f), Vector3(-15.9726f, -38.5824f, 19.4298f)); 
  data[34].mAabb = Aabb(Vector3(26.8522f, -9.27366f, -24.3376f), Vector3(29.0212f, -7.79884f, -20.0087f)); 
  data[35].mAabb = Aabb(Vector3(11.2487f, 1.33635f, -13.9758f), Vector3(11.9421f, 1.77949f, -12.5706f)); 
  data[36].mAabb = Aabb(Vector3(11.7636f, -24.4799f, 25.5848f), Vector3(18.3594f, -17.6473f, 27.9143f)); 
  data[37].mAabb = Aabb(Vector3(-4.96045f, 37.0814f, 17.5987f), Vector3(-4.54004f, 39.0132f, 22.7887f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest16(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(12.0302f, 5.25573f, -34.6972f), Vector3(17.5359f, 6.57167f, -27.1376f)); 
  data[1].mAabb = Aabb(Vector3(-42.4265f, -10.3099f, -26.0898f), Vector3(-40.6318f, -5.58371f, -22.5811f)); 
  data[2].mAabb = Aabb(Vector3(12.0608f, 37.5157f, 20.4999f), Vector3(18.6423f, 38.3172f, 22.9347f)); 
  data[3].mAabb = Aabb(Vector3(-11.207f, -1.57418f, 0.0539637f), Vector3(-3.81658f, 3.60589f, 3.36621f)); 
  data[4].mAabb = Aabb(Vector3(21.8078f, 16.4868f, -29.287f), Vector3(23.6043f, 20.7762f, -26.7588f)); 
  data[5].mAabb = Aabb(Vector3(-41.7356f, -3.51066f, -10.3358f), Vector3(-36.4612f, 3.78053f, -3.57017f)); 
  data[6].mAabb = Aabb(Vector3(7.50225f, -10.4003f, -37.6476f), Vector3(11.9202f, -9.49424f, -34.5689f)); 
  data[7].mAabb = Aabb(Vector3(7.14018f, -5.6806f, 38.896f), Vector3(7.73455f, -4.85777f, 41.2538f)); 
  data[8].mAabb = Aabb(Vector3(-16.837f, 20.6515f, -19.0608f), Vector3(-14.2071f, 27.0497f, -14.5086f)); 
  data[9].mAabb = Aabb(Vector3(-36.159f, -26.9241f, -14.5295f), Vector3(-30.1247f, -24.297f, -7.9056f)); 
  data[10].mAabb = Aabb(Vector3(-6.58286f, -36.9473f, 19.4969f), Vector3(0.335364f, -29.519f, 23.4952f)); 
  data[11].mAabb = Aabb(Vector3(24.9616f, -8.20976f, 21.4399f), Vector3(31.6361f, -2.89633f, 25.8253f)); 
  data[12].mAabb = Aabb(Vector3(30.0232f, 8.94565f, 9.88069f), Vector3(31.4827f, 12.8032f, 13.9361f)); 
  data[13].mAabb = Aabb(Vector3(27.1773f, -16.6205f, -22.6082f), Vector3(28.8154f, -9.46335f, -20.1344f)); 
  data[14].mAabb = Aabb(Vector3(-31.6534f, 38.1462f, -3.48148f), Vector3(-26.2236f, 40.6202f, -3.06293f)); 
  data[15].mAabb = Aabb(Vector3(-3.76451f, -3.37893f, 36.7769f), Vector3(-0.22519f, -1.96651f, 44.4218f)); 
  data[16].mAabb = Aabb(Vector3(-15.9782f, 31.1845f, 15.7803f), Vector3(-10.4372f, 33.0118f, 19.3604f)); 
  data[17].mAabb = Aabb(Vector3(-33.8253f, 25.0978f, 11.3112f), Vector3(-31.6872f, 29.6861f, 12.9108f)); 
  data[18].mAabb = Aabb(Vector3(18.6629f, 18.9244f, -19.1207f), Vector3(22.3857f, 22.961f, -16.6629f)); 
  data[19].mAabb = Aabb(Vector3(27.192f, -29.8469f, -18.1724f), Vector3(28.417f, -27.9949f, -13.1482f)); 
  data[20].mAabb = Aabb(Vector3(22.9337f, 17.2789f, 5.57621f), Vector3(24.9921f, 19.3104f, 7.2999f)); 
  data[21].mAabb = Aabb(Vector3(1.29194f, -25.3793f, 29.301f), Vector3(7.23068f, -22.2572f, 36.4582f)); 
  data[22].mAabb = Aabb(Vector3(36.9925f, 2.82484f, 3.99866f), Vector3(38.6821f, 3.714f, 6.4395f)); 
  data[23].mAabb = Aabb(Vector3(-9.76697f, -17.0002f, 7.25424f), Vector3(-8.63358f, -12.5839f, 12.0565f)); 
  data[24].mAabb = Aabb(Vector3(-12.255f, -12.0009f, 40.3847f), Vector3(-7.25373f, -5.47143f, 45.0139f)); 
  data[25].mAabb = Aabb(Vector3(41.0096f, -10.7363f, -4.39274f), Vector3(44.22f, -8.32604f, -1.09534f)); 
  data[26].mAabb = Aabb(Vector3(-20.2076f, 22.5608f, 7.1826f), Vector3(-13.6095f, 24.9924f, 14.9465f)); 
  data[27].mAabb = Aabb(Vector3(-33.382f, 31.8213f, -17.1628f), Vector3(-27.7617f, 32.9979f, -14.2138f)); 
  data[28].mAabb = Aabb(Vector3(-23.8205f, -27.1259f, -26.2107f), Vector3(-15.8307f, -19.8906f, -21.9862f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest17(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(71); 
  data[0].mAabb = Aabb(Vector3(-25.5756f, 19.1238f, 28.6433f), Vector3(-19.728f, 25.2022f, 33.2294f)); 
  data[1].mAabb = Aabb(Vector3(4.661f, 29.571f, -9.58502f), Vector3(7.16841f, 36.3299f, -3.13346f)); 
  data[2].mAabb = Aabb(Vector3(28.9318f, -11.4647f, -25.0528f), Vector3(34.7101f, -6.19394f, -23.976f)); 
  data[3].mAabb = Aabb(Vector3(-24.0327f, 30.0996f, -18.2097f), Vector3(-17.2228f, 34.3684f, -15.9025f)); 
  data[4].mAabb = Aabb(Vector3(7.13305f, -12.5663f, -0.753874f), Vector3(10.8667f, -10.2505f, 3.95813f)); 
  data[5].mAabb = Aabb(Vector3(-0.841058f, -14.6845f, 7.2795f), Vector3(4.16111f, -14.2273f, 12.5503f)); 
  data[6].mAabb = Aabb(Vector3(-16.4202f, -36.7253f, -2.87308f), Vector3(-12.2721f, -35.9985f, 1.03781f)); 
  data[7].mAabb = Aabb(Vector3(-1.49427f, -24.2525f, -19.7601f), Vector3(4.10468f, -17.6776f, -15.5509f)); 
  data[8].mAabb = Aabb(Vector3(26.5703f, -12.1371f, 34.1447f), Vector3(30.5295f, -7.66032f, 40.3399f)); 
  data[9].mAabb = Aabb(Vector3(-17.2544f, 11.0731f, 37.0917f), Vector3(-16.313f, 17.7603f, 44.9442f)); 
  data[10].mAabb = Aabb(Vector3(16.4931f, 2.93963f, -40.6584f), Vector3(22.1069f, 7.57162f, -39.3158f)); 
  data[11].mAabb = Aabb(Vector3(-21.1319f, -10.0744f, 10.821f), Vector3(-17.5257f, -6.77004f, 16.5781f)); 
  data[12].mAabb = Aabb(Vector3(23.4522f, 25.5209f, -28.7944f), Vector3(28.7826f, 27.738f, -22.9563f)); 
  data[13].mAabb = Aabb(Vector3(-1.92023f, 8.02618f, -38.5109f), Vector3(0.92605f, 14.5258f, -30.9113f)); 
  data[14].mAabb = Aabb(Vector3(-29.7025f, -25.8586f, 29.0596f), Vector3(-26.7748f, -21.4803f, 34.0982f)); 
  data[15].mAabb = Aabb(Vector3(10.0406f, 2.8993f, 24.9952f), Vector3(13.3275f, 7.1792f, 26.9098f)); 
  data[16].mAabb = Aabb(Vector3(33.1815f, -33.9289f, -6.54522f), Vector3(40.5163f, -30.8261f, -5.68343f)); 
  data[17].mAabb = Aabb(Vector3(2.0424f, -3.11092f, 7.40924f), Vector3(9.9211f, -1.73886f, 13.3965f)); 
  data[18].mAabb = Aabb(Vector3(9.47908f, 23.9286f, 0.326179f), Vector3(9.97186f, 26.6893f, 4.5372f)); 
  data[19].mAabb = Aabb(Vector3(8.01295f, 31.9833f, -22.9659f), Vector3(8.93922f, 36.5691f, -20.5534f)); 
  data[20].mAabb = Aabb(Vector3(23.886f, 21.5821f, -29.1039f), Vector3(24.8512f, 28.5126f, -23.9553f)); 
  data[21].mAabb = Aabb(Vector3(-23.1098f, -47.1589f, -0.314866f), Vector3(-17.1038f, -43.2425f, 2.12389f)); 
  data[22].mAabb = Aabb(Vector3(-6.03435f, -45.6494f, 25.2286f), Vector3(1.52798f, -37.6796f, 28.4611f)); 
  data[23].mAabb = Aabb(Vector3(22.2516f, -4.74265f, -6.72719f), Vector3(23.2296f, -2.39227f, -5.52793f)); 
  data[24].mAabb = Aabb(Vector3(-15.2768f, -43.539f, 17.6893f), Vector3(-12.9781f, -39.7425f, 19.5167f)); 
  data[25].mAabb = Aabb(Vector3(22.0379f, -28.2019f, -3.47245f), Vector3(22.5534f, -21.4665f, 0.68128f)); 
  data[26].mAabb = Aabb(Vector3(-40.1486f, 6.24433f, -0.0557498f), Vector3(-34.356f, 7.98285f, 3.14401f)); 
  data[27].mAabb = Aabb(Vector3(19.5685f, -36.1447f, -5.7907f), Vector3(24.322f, -33.8667f, -1.94708f)); 
  data[28].mAabb = Aabb(Vector3(24.6896f, 4.9701f, -33.505f), Vector3(31.9108f, 8.48923f, -26.4894f)); 
  data[29].mAabb = Aabb(Vector3(-31.0753f, -11.5015f, 29.0516f), Vector3(-28.1975f, -6.07048f, 31.7342f)); 
  data[30].mAabb = Aabb(Vector3(32.7143f, 8.75984f, 0.589261f), Vector3(37.8582f, 15.9972f, 4.03696f)); 
  data[31].mAabb = Aabb(Vector3(-15.7011f, 5.11991f, 5.86958f), Vector3(-11.8401f, 10.053f, 12.355f)); 
  data[32].mAabb = Aabb(Vector3(18.7079f, -8.56591f, 18.4966f), Vector3(22.8132f, -3.35894f, 20.0036f)); 
  data[33].mAabb = Aabb(Vector3(7.98004f, 8.06108f, -28.1444f), Vector3(9.941f, 15.4242f, -27.2893f)); 
  data[34].mAabb = Aabb(Vector3(31.6741f, -8.26242f, -39.147f), Vector3(37.4066f, -2.34408f, -31.3335f)); 
  data[35].mAabb = Aabb(Vector3(4.89235f, 21.6593f, -31.3631f), Vector3(6.08675f, 26.2316f, -29.0255f)); 
  data[36].mAabb = Aabb(Vector3(-36.3004f, -10.7872f, 31.2361f), Vector3(-29.8637f, -5.00666f, 35.247f)); 
  data[37].mAabb = Aabb(Vector3(-8.02434f, 43.773f, 15.4533f), Vector3(-1.62913f, 47.1289f, 21.909f)); 
  data[38].mAabb = Aabb(Vector3(34.558f, -5.69252f, 27.747f), Vector3(36.9905f, 1.62024f, 32.9974f)); 
  data[39].mAabb = Aabb(Vector3(-9.84417f, -0.28008f, -32.1688f), Vector3(-3.57073f, 2.84708f, -31.6311f)); 
  data[40].mAabb = Aabb(Vector3(7.13034f, 17.4467f, -7.61086f), Vector3(13.7201f, 19.2884f, -2.86986f)); 
  data[41].mAabb = Aabb(Vector3(-23.0801f, -20.8431f, 23.2764f), Vector3(-22.5945f, -19.6866f, 31.127f)); 
  data[42].mAabb = Aabb(Vector3(-10.5329f, -46.5566f, -22.189f), Vector3(-8.70693f, -40.7827f, -21.3963f)); 
  data[43].mAabb = Aabb(Vector3(7.06689f, 19.8086f, -18.1891f), Vector3(11.9974f, 22.1462f, -14.0393f)); 
  data[44].mAabb = Aabb(Vector3(-14.8336f, -35.2075f, 10.9153f), Vector3(-12.078f, -27.5123f, 13.4702f)); 
  data[45].mAabb = Aabb(Vector3(20.4903f, 6.73319f, -46.6449f), Vector3(24.1683f, 13.3766f, -39.0812f)); 
  data[46].mAabb = Aabb(Vector3(-21.6023f, 25.3083f, -22.6082f), Vector3(-15.6958f, 31.8826f, -16.9392f)); 
  data[47].mAabb = Aabb(Vector3(1.21334f, 7.78683f, -17.4346f), Vector3(3.36542f, 12.221f, -16.9195f)); 
  data[48].mAabb = Aabb(Vector3(-16.5048f, -22.642f, 3.75751f), Vector3(-8.68737f, -16.7248f, 5.00038f)); 
  data[49].mAabb = Aabb(Vector3(9.91738f, -20.9524f, -45.6405f), Vector3(15.489f, -14.9703f, -38.2364f)); 
  data[50].mAabb = Aabb(Vector3(-23.1516f, -0.637859f, 23.4871f), Vector3(-22.6229f, 6.71201f, 25.2528f)); 
  data[51].mAabb = Aabb(Vector3(-16.3348f, -7.51829f, -3.00863f), Vector3(-15.068f, -1.3476f, -1.05463f)); 
  data[52].mAabb = Aabb(Vector3(5.41955f, -27.0224f, 24.4912f), Vector3(7.59714f, -21.7451f, 27.4586f)); 
  data[53].mAabb = Aabb(Vector3(8.56183f, -17.4261f, 11.5583f), Vector3(13.0451f, -15.4907f, 17.5f)); 
  data[54].mAabb = Aabb(Vector3(21.8935f, 18.7756f, 10.0946f), Vector3(23.7053f, 22.0714f, 12.1377f)); 
  data[55].mAabb = Aabb(Vector3(-24.0311f, -25.9427f, -26.9866f), Vector3(-20.9473f, -24.5353f, -19.3683f)); 
  data[56].mAabb = Aabb(Vector3(-20.9182f, 6.27708f, 13.8554f), Vector3(-15.5839f, 7.08483f, 20.3067f)); 
  data[57].mAabb = Aabb(Vector3(39.9601f, 4.97933f, -4.50756f), Vector3(45.1225f, 10.4997f, -3.7724f)); 
  data[58].mAabb = Aabb(Vector3(-7.04987f, -29.3313f, -15.9479f), Vector3(-6.16048f, -28.4194f, -14.2506f)); 
  data[59].mAabb = Aabb(Vector3(39.1422f, -22.6343f, 1.89912f), Vector3(41.3251f, -15.1895f, 2.3905f)); 
  data[60].mAabb = Aabb(Vector3(-6.83666f, 9.21292f, 31.5449f), Vector3(0.971061f, 12.4375f, 33.8623f)); 
  data[61].mAabb = Aabb(Vector3(16.6127f, 1.60802f, 18.1823f), Vector3(22.6669f, 7.42268f, 21.0888f)); 
  data[62].mAabb = Aabb(Vector3(-36.6718f, 18.016f, 5.40836f), Vector3(-32.059f, 20.3139f, 7.8833f)); 
  data[63].mAabb = Aabb(Vector3(-44.1193f, 5.12673f, -2.70548f), Vector3(-40.9657f, 6.2302f, -0.36414f)); 
  data[64].mAabb = Aabb(Vector3(8.9495f, 8.49248f, 24.2549f), Vector3(15.566f, 9.38953f, 27.8332f)); 
  data[65].mAabb = Aabb(Vector3(10.1417f, -25.5466f, 2.65799f), Vector3(16.6556f, -24.0299f, 5.92756f)); 
  data[66].mAabb = Aabb(Vector3(-35.475f, 20.4349f, 17.3484f), Vector3(-30.4747f, 26.0329f, 21.0867f)); 
  data[67].mAabb = Aabb(Vector3(-8.89887f, -18.2133f, 2.56879f), Vector3(-5.31595f, -10.4522f, 6.79025f)); 
  data[68].mAabb = Aabb(Vector3(-31.3844f, -31.137f, -27.6999f), Vector3(-23.8786f, -24.7682f, -22.8264f)); 
  data[69].mAabb = Aabb(Vector3(-18.8159f, -3.19056f, 6.10012f), Vector3(-15.2448f, -0.137851f, 10.1049f)); 
  data[70].mAabb = Aabb(Vector3(-25.0517f, 38.7087f, -3.5858f), Vector3(-18.4104f, 45.106f, 2.29195f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest18(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(52); 
  data[0].mAabb = Aabb(Vector3(-31.0373f, -31.6902f, 2.05105f), Vector3(-30.5158f, -27.937f, 6.9347f)); 
  data[1].mAabb = Aabb(Vector3(10.8337f, 10.6219f, -19.4968f), Vector3(18.0942f, 12.9577f, -17.6031f)); 
  data[2].mAabb = Aabb(Vector3(15.9019f, 16.9041f, -40.7024f), Vector3(17.7896f, 21.7418f, -35.8125f)); 
  data[3].mAabb = Aabb(Vector3(-35.2845f, 5.75027f, 11.289f), Vector3(-32.0077f, 13.3319f, 17.4877f)); 
  data[4].mAabb = Aabb(Vector3(8.90939f, -27.8492f, -10.1529f), Vector3(13.8807f, -24.3646f, -3.00366f)); 
  data[5].mAabb = Aabb(Vector3(21.7153f, -17.054f, 29.5786f), Vector3(23.8681f, -11.4282f, 35.9404f)); 
  data[6].mAabb = Aabb(Vector3(-7.96585f, 14.333f, -47.8239f), Vector3(-1.30206f, 15.7658f, -45.2121f)); 
  data[7].mAabb = Aabb(Vector3(-33.8394f, 2.88025f, -5.7623f), Vector3(-29.013f, 4.65195f, -1.83587f)); 
  data[8].mAabb = Aabb(Vector3(10.2337f, 4.34391f, 1.90979f), Vector3(13.2231f, 10.9843f, 7.81467f)); 
  data[9].mAabb = Aabb(Vector3(27.228f, -36.3495f, -16.3516f), Vector3(32.8372f, -34.8037f, -10.8273f)); 
  data[10].mAabb = Aabb(Vector3(-1.5472f, 25.4867f, 3.40387f), Vector3(1.82837f, 26.4176f, 10.4448f)); 
  data[11].mAabb = Aabb(Vector3(24.7476f, 26.2195f, 1.19041f), Vector3(30.7353f, 28.7682f, 1.96615f)); 
  data[12].mAabb = Aabb(Vector3(43.9224f, -3.7526f, -2.93813f), Vector3(51.5093f, 0.686416f, 2.20088f)); 
  data[13].mAabb = Aabb(Vector3(27.7555f, 5.19371f, -13.403f), Vector3(33.9278f, 12.6498f, -7.46867f)); 
  data[14].mAabb = Aabb(Vector3(14.134f, -42.3322f, -0.890292f), Vector3(17.1744f, -39.4235f, 4.27794f)); 
  data[15].mAabb = Aabb(Vector3(9.96584f, -42.2712f, 17.8917f), Vector3(13.5736f, -41.3348f, 20.8748f)); 
  data[16].mAabb = Aabb(Vector3(24.5566f, -19.7219f, -21.6211f), Vector3(30.574f, -18.0024f, -14.6913f)); 
  data[17].mAabb = Aabb(Vector3(9.35028f, 33.1484f, 6.71485f), Vector3(10.409f, 37.3944f, 9.61958f)); 
  data[18].mAabb = Aabb(Vector3(-37.4232f, -25.3263f, -24.1352f), Vector3(-30.5481f, -22.2351f, -20.0478f)); 
  data[19].mAabb = Aabb(Vector3(-26.0064f, 26.1759f, 29.6764f), Vector3(-18.124f, 32.0374f, 31.703f)); 
  data[20].mAabb = Aabb(Vector3(-31.2905f, -2.40557f, -41.1277f), Vector3(-28.91f, 0.731326f, -33.8685f)); 
  data[21].mAabb = Aabb(Vector3(-18.7988f, -17.1801f, 39.9355f), Vector3(-12.1011f, -13.6621f, 46.8973f)); 
  data[22].mAabb = Aabb(Vector3(31.7475f, 15.0394f, 15.1593f), Vector3(37.4211f, 17.8629f, 17.7439f)); 
  data[23].mAabb = Aabb(Vector3(25.6952f, 9.84695f, -38.3586f), Vector3(31.5657f, 11.457f, -34.9042f)); 
  data[24].mAabb = Aabb(Vector3(-18.249f, 13.0572f, -32.3577f), Vector3(-14.0772f, 20.963f, -28.1804f)); 
  data[25].mAabb = Aabb(Vector3(14.8982f, 13.1327f, 19.3747f), Vector3(20.5173f, 14.8529f, 26.0187f)); 
  data[26].mAabb = Aabb(Vector3(-12.9032f, -18.8983f, 17.4438f), Vector3(-6.17009f, -14.6084f, 22.9214f)); 
  data[27].mAabb = Aabb(Vector3(-28.2109f, -22.4847f, -25.2818f), Vector3(-22.4521f, -17.447f, -18.0223f)); 
  data[28].mAabb = Aabb(Vector3(-5.94514f, -9.89766f, 24.6932f), Vector3(0.0392931f, -4.40053f, 27.8095f)); 
  data[29].mAabb = Aabb(Vector3(23.5805f, 18.829f, 25.2544f), Vector3(28.0747f, 22.4579f, 29.7101f)); 
  data[30].mAabb = Aabb(Vector3(39.0447f, 22.599f, 6.62582f), Vector3(43.7762f, 29.2832f, 11.3295f)); 
  data[31].mAabb = Aabb(Vector3(-22.7193f, 34.932f, 26.5859f), Vector3(-20.4446f, 37.0564f, 27.1932f)); 
  data[32].mAabb = Aabb(Vector3(38.8412f, 22.132f, 0.481247f), Vector3(39.6026f, 24.8157f, 4.63521f)); 
  data[33].mAabb = Aabb(Vector3(38.805f, -6.04243f, -27.9068f), Vector3(44.5285f, -4.86125f, -23.5386f)); 
  data[34].mAabb = Aabb(Vector3(-12.0077f, -34.0276f, 13.5825f), Vector3(-9.05539f, -30.6964f, 16.7424f)); 
  data[35].mAabb = Aabb(Vector3(-21.6817f, -5.84944f, 4.05293f), Vector3(-14.0296f, 1.23602f, 4.96575f)); 
  data[36].mAabb = Aabb(Vector3(-34.8833f, -29.9796f, 9.23405f), Vector3(-31.486f, -24.0448f, 10.8624f)); 
  data[37].mAabb = Aabb(Vector3(-6.62826f, 1.97504f, 13.0241f), Vector3(-0.0521414f, 8.43333f, 17.0948f)); 
  data[38].mAabb = Aabb(Vector3(34.2865f, 11.7743f, 30.4133f), Vector3(35.5495f, 14.3302f, 35.7047f)); 
  data[39].mAabb = Aabb(Vector3(8.72854f, -27.8873f, 12.0355f), Vector3(12.4713f, -27.0615f, 16.9008f)); 
  data[40].mAabb = Aabb(Vector3(4.70027f, -20.8621f, 13.6331f), Vector3(7.78892f, -16.1013f, 17.335f)); 
  data[41].mAabb = Aabb(Vector3(-43.8026f, -21.9084f, -7.29984f), Vector3(-40.7232f, -19.8908f, -0.127632f)); 
  data[42].mAabb = Aabb(Vector3(-27.3256f, -12.0665f, 26.9419f), Vector3(-23.2851f, -8.16256f, 31.7732f)); 
  data[43].mAabb = Aabb(Vector3(2.82141f, -30.4408f, 2.63874f), Vector3(8.10492f, -23.2888f, 5.64506f)); 
  data[44].mAabb = Aabb(Vector3(28.7422f, -15.8665f, 9.74656f), Vector3(32.7788f, -14.2562f, 13.2219f)); 
  data[45].mAabb = Aabb(Vector3(-47.5387f, 14.0231f, 10.4753f), Vector3(-41.2812f, 20.2005f, 12.6287f)); 
  data[46].mAabb = Aabb(Vector3(0.226173f, 10.1914f, -37.9435f), Vector3(2.67421f, 15.9043f, -30.255f)); 
  data[47].mAabb = Aabb(Vector3(-11.4345f, -11.1549f, -7.02173f), Vector3(-5.12651f, -8.42896f, -0.365359f)); 
  data[48].mAabb = Aabb(Vector3(-9.95883f, 21.8857f, 2.68675f), Vector3(-7.46093f, 26.5997f, 3.13778f)); 
  data[49].mAabb = Aabb(Vector3(17.9015f, -30.8526f, -37.9618f), Vector3(20.2208f, -28.406f, -31.0935f)); 
  data[50].mAabb = Aabb(Vector3(-16.9768f, -15.2826f, 0.224812f), Vector3(-9.91522f, -9.25219f, 6.61282f)); 
  data[51].mAabb = Aabb(Vector3(12.6366f, -1.33923f, 3.66575f), Vector3(19.9485f, 1.43608f, 7.25563f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest19(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(52); 
  data[0].mAabb = Aabb(Vector3(-13.6571f, 21.071f, 17.7559f), Vector3(-9.70868f, 23.7783f, 23.0777f)); 
  data[1].mAabb = Aabb(Vector3(-10.0891f, 12.6565f, -22.592f), Vector3(-6.29444f, 16.9916f, -20.9701f)); 
  data[2].mAabb = Aabb(Vector3(21.998f, 3.54374f, -26.0477f), Vector3(26.8575f, 8.28196f, -19.5716f)); 
  data[3].mAabb = Aabb(Vector3(18.3535f, -17.3757f, -24.865f), Vector3(20.4175f, -15.4205f, -23.9797f)); 
  data[4].mAabb = Aabb(Vector3(-5.84667f, -23.0917f, 40.5459f), Vector3(-2.57965f, -15.9065f, 42.7982f)); 
  data[5].mAabb = Aabb(Vector3(-2.2851f, 28.1201f, 15.6434f), Vector3(-0.647932f, 29.1714f, 21.0883f)); 
  data[6].mAabb = Aabb(Vector3(2.52682f, 1.75909f, -27.3482f), Vector3(4.90273f, 5.26199f, -20.4694f)); 
  data[7].mAabb = Aabb(Vector3(-23.624f, -3.03102f, -2.13147f), Vector3(-20.0305f, -1.7849f, 1.91835f)); 
  data[8].mAabb = Aabb(Vector3(3.93246f, 18.3981f, -19.145f), Vector3(7.33354f, 19.4107f, -11.8499f)); 
  data[9].mAabb = Aabb(Vector3(17.6569f, 41.3501f, -7.70851f), Vector3(20.5345f, 42.7403f, -0.409205f)); 
  data[10].mAabb = Aabb(Vector3(25.4683f, 17.7181f, -2.41735f), Vector3(30.5118f, 20.2418f, -0.282668f)); 
  data[11].mAabb = Aabb(Vector3(19.2856f, -34.0752f, -24.8493f), Vector3(22.5635f, -31.0039f, -19.0834f)); 
  data[12].mAabb = Aabb(Vector3(31.1278f, -2.32065f, -31.7059f), Vector3(35.8308f, 3.22589f, -28.5674f)); 
  data[13].mAabb = Aabb(Vector3(-6.67624f, 48.442f, -12.3465f), Vector3(-0.199162f, 49.1618f, -6.01973f)); 
  data[14].mAabb = Aabb(Vector3(-3.54711f, 16.7056f, 0.520968f), Vector3(0.492734f, 20.3621f, 7.83071f)); 
  data[15].mAabb = Aabb(Vector3(-16.1798f, 3.88729f, -20.652f), Vector3(-13.7016f, 9.46166f, -15.7303f)); 
  data[16].mAabb = Aabb(Vector3(-1.75915f, 38.1026f, 18.2999f), Vector3(0.738988f, 39.2773f, 23.4257f)); 
  data[17].mAabb = Aabb(Vector3(-44.4345f, -7.26913f, 28.0438f), Vector3(-36.7063f, -4.70373f, 28.9902f)); 
  data[18].mAabb = Aabb(Vector3(-21.359f, 19.0154f, 23.6396f), Vector3(-17.1551f, 21.9396f, 26.3572f)); 
  data[19].mAabb = Aabb(Vector3(4.35669f, 19.9655f, 34.2055f), Vector3(7.79952f, 21.3515f, 36.9017f)); 
  data[20].mAabb = Aabb(Vector3(16.7467f, -37.9487f, 20.3779f), Vector3(23.3952f, -32.5226f, 22.1074f)); 
  data[21].mAabb = Aabb(Vector3(-11.8202f, 16.2604f, -21.4561f), Vector3(-8.96555f, 21.0659f, -13.9865f)); 
  data[22].mAabb = Aabb(Vector3(-7.14784f, -1.42472f, -21.5728f), Vector3(-2.97393f, 3.07529f, -15.6009f)); 
  data[23].mAabb = Aabb(Vector3(-18.7064f, 7.56406f, 20.76f), Vector3(-17.6215f, 9.85763f, 25.3078f)); 
  data[24].mAabb = Aabb(Vector3(-13.398f, 35.659f, -21.6757f), Vector3(-8.33156f, 43.2856f, -15.4962f)); 
  data[25].mAabb = Aabb(Vector3(34.7471f, -8.23898f, 18.7201f), Vector3(40.7125f, -6.22305f, 26.1816f)); 
  data[26].mAabb = Aabb(Vector3(29.8524f, -14.4086f, -37.1018f), Vector3(33.9301f, -9.16868f, -32.7147f)); 
  data[27].mAabb = Aabb(Vector3(35.6131f, -30.2241f, 4.40098f), Vector3(37.9219f, -26.1626f, 8.55447f)); 
  data[28].mAabb = Aabb(Vector3(20.2347f, -16.5205f, 26.8303f), Vector3(28.2145f, -13.2126f, 33.2677f)); 
  data[29].mAabb = Aabb(Vector3(-6.63671f, 4.1842f, 32.5459f), Vector3(-4.28284f, 8.15122f, 38.8035f)); 
  data[30].mAabb = Aabb(Vector3(-28.8854f, -15.9729f, 1.7589f), Vector3(-26.1145f, -13.0072f, 7.889f)); 
  data[31].mAabb = Aabb(Vector3(12.4372f, 33.2992f, 2.27149f), Vector3(16.9231f, 38.7005f, 4.7694f)); 
  data[32].mAabb = Aabb(Vector3(6.8379f, -12.0639f, 7.40251f), Vector3(12.582f, -5.29876f, 10.9286f)); 
  data[33].mAabb = Aabb(Vector3(17.0938f, -11.67f, -35.1821f), Vector3(22.4455f, -9.80249f, -30.1547f)); 
  data[34].mAabb = Aabb(Vector3(1.3647f, -9.78108f, -41.607f), Vector3(8.79158f, -4.08263f, -36.6849f)); 
  data[35].mAabb = Aabb(Vector3(36.7593f, 19.4596f, -1.87663f), Vector3(38.2408f, 24.5525f, -0.563253f)); 
  data[36].mAabb = Aabb(Vector3(-24.8724f, -22.1454f, 2.08023f), Vector3(-17.463f, -18.3804f, 3.48384f)); 
  data[37].mAabb = Aabb(Vector3(-12.8781f, -0.841851f, 40.1921f), Vector3(-8.95025f, 3.11867f, 48.1796f)); 
  data[38].mAabb = Aabb(Vector3(-2.5811f, 4.06052f, 41.1539f), Vector3(-0.307256f, 7.89974f, 43.845f)); 
  data[39].mAabb = Aabb(Vector3(9.18527f, 2.20979f, -27.9509f), Vector3(11.2179f, 9.8635f, -26.6839f)); 
  data[40].mAabb = Aabb(Vector3(9.50657f, 25.0024f, 5.80185f), Vector3(9.93811f, 29.0052f, 7.39356f)); 
  data[41].mAabb = Aabb(Vector3(-14.8912f, 3.38362f, 34.907f), Vector3(-8.64908f, 7.05143f, 38.8729f)); 
  data[42].mAabb = Aabb(Vector3(14.5077f, -6.94564f, -27.1194f), Vector3(20.1948f, -2.84109f, -20.5482f)); 
  data[43].mAabb = Aabb(Vector3(29.88f, 11.2352f, -20.6936f), Vector3(37.1524f, 18.3947f, -15.7921f)); 
  data[44].mAabb = Aabb(Vector3(3.87334f, -7.63566f, 13.3395f), Vector3(10.2113f, -4.32481f, 15.8906f)); 
  data[45].mAabb = Aabb(Vector3(11.1133f, -10.6231f, 40.7884f), Vector3(15.4593f, -8.73609f, 46.7051f)); 
  data[46].mAabb = Aabb(Vector3(35.9361f, 21.6325f, 18.4037f), Vector3(38.7605f, 25.24f, 26.2596f)); 
  data[47].mAabb = Aabb(Vector3(39.7626f, 13.6645f, 23.9682f), Vector3(40.3706f, 16.0791f, 25.4915f)); 
  data[48].mAabb = Aabb(Vector3(1.93066f, -7.57634f, 39.2482f), Vector3(6.2711f, 0.0400305f, 43.0565f)); 
  data[49].mAabb = Aabb(Vector3(17.9015f, 10.6466f, -6.78177f), Vector3(22.3361f, 18.0815f, -2.79782f)); 
  data[50].mAabb = Aabb(Vector3(12.5809f, -28.2388f, -1.74385f), Vector3(17.1992f, -25.348f, 1.95527f)); 
  data[51].mAabb = Aabb(Vector3(8.49209f, 9.33199f, -12.5264f), Vector3(11.6397f, 14.8057f, -10.0844f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest20(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(30); 
  data[0].mAabb = Aabb(Vector3(34.8656f, 12.997f, -4.08115f), Vector3(42.0983f, 16.9383f, 1.93066f)); 
  data[1].mAabb = Aabb(Vector3(-22.122f, -3.26779f, -29.2628f), Vector3(-15.2068f, 1.90718f, -26.2595f)); 
  data[2].mAabb = Aabb(Vector3(37.1204f, 15.4574f, 6.51484f), Vector3(37.8876f, 18.1977f, 13.8401f)); 
  data[3].mAabb = Aabb(Vector3(20.6679f, 43.059f, 4.56778f), Vector3(25.6279f, 43.9421f, 7.85823f)); 
  data[4].mAabb = Aabb(Vector3(18.6587f, 16.533f, -28.4038f), Vector3(26.5212f, 18.5116f, -23.5572f)); 
  data[5].mAabb = Aabb(Vector3(14.65f, -24.8839f, -25.9386f), Vector3(19.2762f, -18.1046f, -24.6955f)); 
  data[6].mAabb = Aabb(Vector3(-1.79568f, 21.8927f, -1.40191f), Vector3(-0.955687f, 27.9209f, 3.44231f)); 
  data[7].mAabb = Aabb(Vector3(-13.2881f, -45.8448f, 19.2807f), Vector3(-6.57884f, -42.2484f, 21.9865f)); 
  data[8].mAabb = Aabb(Vector3(-17.5947f, -4.76668f, -31.7801f), Vector3(-12.9133f, 1.69092f, -27.7022f)); 
  data[9].mAabb = Aabb(Vector3(-28.7683f, 28.1934f, -22.6252f), Vector3(-24.8541f, 32.4288f, -15.2981f)); 
  data[10].mAabb = Aabb(Vector3(-27.6215f, 23.9726f, 10.8375f), Vector3(-21.2593f, 30.045f, 18.77f)); 
  data[11].mAabb = Aabb(Vector3(-35.7368f, -21.8153f, -6.15697f), Vector3(-33.7933f, -20.8642f, 0.298302f)); 
  data[12].mAabb = Aabb(Vector3(-17.5267f, 8.51485f, -21.3699f), Vector3(-16.5805f, 13.6008f, -18.8179f)); 
  data[13].mAabb = Aabb(Vector3(-14.4832f, 3.74167f, -41.1794f), Vector3(-13.96f, 6.06817f, -40.7163f)); 
  data[14].mAabb = Aabb(Vector3(24.424f, -16.5589f, -20.1916f), Vector3(32.3698f, -16.0703f, -18.8063f)); 
  data[15].mAabb = Aabb(Vector3(21.3266f, 5.79234f, 27.5594f), Vector3(26.7024f, 13.5919f, 33.3768f)); 
  data[16].mAabb = Aabb(Vector3(5.71792f, -11.5106f, 35.3196f), Vector3(7.37945f, -3.63543f, 36.8742f)); 
  data[17].mAabb = Aabb(Vector3(-3.8723f, -24.3159f, -31.5993f), Vector3(-2.31538f, -17.4105f, -28.3982f)); 
  data[18].mAabb = Aabb(Vector3(-13.8552f, 15.3022f, 24.1071f), Vector3(-6.83535f, 20.0919f, 24.9702f)); 
  data[19].mAabb = Aabb(Vector3(-16.4731f, -16.2642f, 14.8471f), Vector3(-11.7068f, -13.2674f, 17.0537f)); 
  data[20].mAabb = Aabb(Vector3(21.9677f, -41.8919f, -15.8873f), Vector3(22.5755f, -39.4311f, -13.8378f)); 
  data[21].mAabb = Aabb(Vector3(6.76081f, -16.8487f, -45.8634f), Vector3(13.8041f, -12.6463f, -39.1557f)); 
  data[22].mAabb = Aabb(Vector3(-27.4822f, 15.5848f, -8.29314f), Vector3(-22.2471f, 16.1615f, -5.18779f)); 
  data[23].mAabb = Aabb(Vector3(15.2706f, -17.429f, 38.901f), Vector3(17.1341f, -13.2412f, 42.1249f)); 
  data[24].mAabb = Aabb(Vector3(25.1885f, 35.8574f, 7.23699f), Vector3(27.7058f, 37.4304f, 13.904f)); 
  data[25].mAabb = Aabb(Vector3(-36.0505f, 22.6464f, 11.5178f), Vector3(-32.2024f, 24.5294f, 18.7114f)); 
  data[26].mAabb = Aabb(Vector3(17.0491f, -19.9887f, 15.5054f), Vector3(23.2613f, -13.7005f, 17.1989f)); 
  data[27].mAabb = Aabb(Vector3(-46.7658f, -1.31851f, -2.48642f), Vector3(-41.8729f, 5.41765f, -1.82711f)); 
  data[28].mAabb = Aabb(Vector3(-4.96272f, -16.6296f, 3.06579f), Vector3(-0.561739f, -8.77314f, 6.70971f)); 
  data[29].mAabb = Aabb(Vector3(6.36581f, -1.16545f, 30.0764f), Vector3(8.70599f, 1.49064f, 31.5668f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest21(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(45); 
  data[0].mAabb = Aabb(Vector3(26.9962f, 4.07132f, 28.9198f), Vector3(34.5632f, 11.1171f, 32.5872f)); 
  data[1].mAabb = Aabb(Vector3(-39.0303f, 26.3177f, -17.2935f), Vector3(-32.5873f, 28.5006f, -12.8765f)); 
  data[2].mAabb = Aabb(Vector3(3.02063f, -32.6106f, -1.38043f), Vector3(4.13339f, -29.0622f, 5.04562f)); 
  data[3].mAabb = Aabb(Vector3(-21.3055f, -35.8956f, -32.9038f), Vector3(-18.7281f, -28.9678f, -29.0094f)); 
  data[4].mAabb = Aabb(Vector3(-17.8945f, 17.4564f, -40.7741f), Vector3(-12.0091f, 18.756f, -36.3339f)); 
  data[5].mAabb = Aabb(Vector3(21.911f, 15.4184f, 16.3692f), Vector3(24.0573f, 21.0925f, 20.3105f)); 
  data[6].mAabb = Aabb(Vector3(0.941139f, -9.1796f, 39.1145f), Vector3(6.28101f, -5.339f, 46.6838f)); 
  data[7].mAabb = Aabb(Vector3(-34.0685f, 12.5147f, 17.8939f), Vector3(-27.411f, 19.011f, 25.0347f)); 
  data[8].mAabb = Aabb(Vector3(-16.6204f, 41.9587f, -20.0587f), Vector3(-15.3763f, 43.8283f, -13.7526f)); 
  data[9].mAabb = Aabb(Vector3(22.254f, 17.5643f, 23.2157f), Vector3(29.4891f, 20.4066f, 30.5023f)); 
  data[10].mAabb = Aabb(Vector3(-14.6492f, -2.16899f, 41.3105f), Vector3(-10.9346f, 3.81962f, 43.1054f)); 
  data[11].mAabb = Aabb(Vector3(22.7647f, -20.4472f, 26.0705f), Vector3(25.7476f, -15.555f, 26.8323f)); 
  data[12].mAabb = Aabb(Vector3(-6.26473f, 15.1466f, 24.0172f), Vector3(0.57302f, 16.2327f, 28.9465f)); 
  data[13].mAabb = Aabb(Vector3(5.16736f, 19.2167f, 2.52788f), Vector3(12.8359f, 26.3762f, 9.31158f)); 
  data[14].mAabb = Aabb(Vector3(10.9855f, -0.694908f, 31.654f), Vector3(16.6756f, 3.82969f, 36.8941f)); 
  data[15].mAabb = Aabb(Vector3(-25.4619f, -18.4007f, 26.5478f), Vector3(-21.8279f, -17.9981f, 34.465f)); 
  data[16].mAabb = Aabb(Vector3(4.81062f, -5.39017f, 32.9605f), Vector3(12.7839f, -1.69916f, 38.2543f)); 
  data[17].mAabb = Aabb(Vector3(33.8374f, -11.7275f, -0.0595496f), Vector3(37.0908f, -8.81979f, 7.2212f)); 
  data[18].mAabb = Aabb(Vector3(36.6584f, 15.8423f, -4.3249f), Vector3(41.5281f, 16.5183f, 2.97928f)); 
  data[19].mAabb = Aabb(Vector3(21.8392f, -15.2001f, 34.0027f), Vector3(23.8829f, -8.7963f, 41.45f)); 
  data[20].mAabb = Aabb(Vector3(-8.23044f, 31.0584f, -10.0027f), Vector3(-1.89438f, 31.5018f, -7.92435f)); 
  data[21].mAabb = Aabb(Vector3(-21.5045f, 4.04665f, -37.4353f), Vector3(-14.9602f, 5.72186f, -36.3771f)); 
  data[22].mAabb = Aabb(Vector3(10.6156f, -42.7541f, -5.23044f), Vector3(17.2634f, -39.7051f, 1.37212f)); 
  data[23].mAabb = Aabb(Vector3(-29.583f, 27.4218f, -29.3776f), Vector3(-24.9069f, 29.354f, -23.2146f)); 
  data[24].mAabb = Aabb(Vector3(6.24331f, -33.5459f, -14.2403f), Vector3(6.84533f, -30.0669f, -11.4553f)); 
  data[25].mAabb = Aabb(Vector3(6.16033f, 3.39717f, -33.6559f), Vector3(7.0553f, 4.39906f, -32.9632f)); 
  data[26].mAabb = Aabb(Vector3(6.73934f, 21.2076f, -34.4006f), Vector3(14.1379f, 27.6683f, -28.3333f)); 
  data[27].mAabb = Aabb(Vector3(39.5397f, -0.572448f, -11.9848f), Vector3(46.1903f, 4.23234f, -10.0994f)); 
  data[28].mAabb = Aabb(Vector3(2.08475f, -29.6284f, 9.55969f), Vector3(4.28762f, -23.0117f, 11.9739f)); 
  data[29].mAabb = Aabb(Vector3(-43.5274f, -4.21357f, 23.3774f), Vector3(-41.4667f, 0.560599f, 24.0483f)); 
  data[30].mAabb = Aabb(Vector3(-6.35983f, 14.8839f, -4.65819f), Vector3(-5.14548f, 17.0049f, 0.540427f)); 
  data[31].mAabb = Aabb(Vector3(14.9536f, -5.76529f, -19.1017f), Vector3(17.4737f, -4.47718f, -14.6689f)); 
  data[32].mAabb = Aabb(Vector3(6.63769f, -31.0456f, 19.5422f), Vector3(9.27754f, -23.628f, 20.4329f)); 
  data[33].mAabb = Aabb(Vector3(3.42486f, -35.8714f, 8.16639f), Vector3(6.37389f, -29.3017f, 12.7198f)); 
  data[34].mAabb = Aabb(Vector3(33.6719f, -33.0146f, -5.9449f), Vector3(40.2684f, -29.0462f, -1.27835f)); 
  data[35].mAabb = Aabb(Vector3(7.1516f, -0.502193f, 17.0585f), Vector3(8.94974f, 5.41869f, 17.6197f)); 
  data[36].mAabb = Aabb(Vector3(-1.63613f, 6.76355f, 9.73379f), Vector3(4.71152f, 14.1549f, 12.177f)); 
  data[37].mAabb = Aabb(Vector3(-40.0587f, -8.41447f, -3.0322f), Vector3(-33.3362f, -4.06035f, 0.656721f)); 
  data[38].mAabb = Aabb(Vector3(-0.135072f, -5.41144f, 36.3323f), Vector3(6.55632f, 2.13813f, 36.7952f)); 
  data[39].mAabb = Aabb(Vector3(-21.6539f, -22.1278f, 36.2482f), Vector3(-18.0202f, -21.4462f, 43.5023f)); 
  data[40].mAabb = Aabb(Vector3(16.5809f, -13.5949f, -19.8583f), Vector3(24.0892f, -8.03931f, -15.3314f)); 
  data[41].mAabb = Aabb(Vector3(-17.3866f, 24.2283f, -22.6283f), Vector3(-13.5384f, 25.1539f, -16.3493f)); 
  data[42].mAabb = Aabb(Vector3(-18.1853f, 10.4014f, -42.157f), Vector3(-17.5025f, 13.4553f, -36.5551f)); 
  data[43].mAabb = Aabb(Vector3(-30.9834f, 8.02504f, 1.71961f), Vector3(-24.3449f, 12.3367f, 2.27849f)); 
  data[44].mAabb = Aabb(Vector3(26.3395f, 6.63209f, 26.4567f), Vector3(28.3549f, 7.1745f, 33.1599f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest22(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(68); 
  data[0].mAabb = Aabb(Vector3(15.8536f, 19.6215f, 29.8871f), Vector3(18.6447f, 25.4624f, 37.2748f)); 
  data[1].mAabb = Aabb(Vector3(31.8839f, 5.72023f, 28.9605f), Vector3(32.4569f, 10.364f, 30.1749f)); 
  data[2].mAabb = Aabb(Vector3(1.89348f, 5.9557f, -24.7995f), Vector3(3.07257f, 12.5643f, -23.9725f)); 
  data[3].mAabb = Aabb(Vector3(-30.2493f, -25.609f, -0.998543f), Vector3(-28.785f, -24.9622f, 0.782664f)); 
  data[4].mAabb = Aabb(Vector3(11.5067f, 3.56099f, 33.8074f), Vector3(16.0302f, 8.68679f, 39.9579f)); 
  data[5].mAabb = Aabb(Vector3(23.5247f, -11.3569f, -34.1792f), Vector3(26.0662f, -3.46382f, -29.4732f)); 
  data[6].mAabb = Aabb(Vector3(15.8657f, 17.2406f, -29.9776f), Vector3(21.0771f, 17.9403f, -28.9481f)); 
  data[7].mAabb = Aabb(Vector3(-29.047f, -4.67295f, 39.9641f), Vector3(-21.2351f, 1.14796f, 42.2528f)); 
  data[8].mAabb = Aabb(Vector3(11.1164f, 43.5273f, 2.51846f), Vector3(14.4347f, 51.3965f, 10.2696f)); 
  data[9].mAabb = Aabb(Vector3(-3.53495f, -17.3968f, 23.5018f), Vector3(-1.03496f, -13.9588f, 24.7607f)); 
  data[10].mAabb = Aabb(Vector3(-11.6215f, -36.3229f, 11.9621f), Vector3(-8.3867f, -35.3465f, 19.7102f)); 
  data[11].mAabb = Aabb(Vector3(11.6996f, -47.5215f, -17.6922f), Vector3(14.8798f, -41.7419f, -16.8461f)); 
  data[12].mAabb = Aabb(Vector3(-15.947f, -9.04352f, -29.6325f), Vector3(-11.5201f, -3.5123f, -28.6682f)); 
  data[13].mAabb = Aabb(Vector3(27.6775f, -25.8735f, 15.1096f), Vector3(33.6487f, -20.9091f, 18.573f)); 
  data[14].mAabb = Aabb(Vector3(-26.6129f, 0.68349f, 9.30597f), Vector3(-22.4622f, 2.40718f, 14.8653f)); 
  data[15].mAabb = Aabb(Vector3(-27.4333f, 28.5337f, -21.0494f), Vector3(-22.9163f, 33.643f, -13.6346f)); 
  data[16].mAabb = Aabb(Vector3(22.4161f, -8.53847f, -5.67807f), Vector3(25.7392f, -7.6834f, -3.20892f)); 
  data[17].mAabb = Aabb(Vector3(-2.34181f, -29.679f, -22.5967f), Vector3(0.115736f, -26.7147f, -19.3592f)); 
  data[18].mAabb = Aabb(Vector3(-4.17507f, 3.56353f, 7.68241f), Vector3(-1.20377f, 9.20818f, 15.62f)); 
  data[19].mAabb = Aabb(Vector3(-4.89907f, 10.9136f, 16.8549f), Vector3(-2.79268f, 12.846f, 23.5518f)); 
  data[20].mAabb = Aabb(Vector3(-1.1022f, -21.3005f, 40.124f), Vector3(4.16461f, -18.9262f, 45.0482f)); 
  data[21].mAabb = Aabb(Vector3(0.340107f, -10.6739f, 21.5343f), Vector3(0.78464f, -6.54063f, 27.1743f)); 
  data[22].mAabb = Aabb(Vector3(1.60294f, 14.4177f, 16.6801f), Vector3(8.16189f, 21.8594f, 24.3878f)); 
  data[23].mAabb = Aabb(Vector3(13.1458f, -6.36289f, -30.0776f), Vector3(17.6933f, -1.38415f, -24.3529f)); 
  data[24].mAabb = Aabb(Vector3(34.267f, -9.38111f, 11.0814f), Vector3(38.2865f, -4.71155f, 16.8761f)); 
  data[25].mAabb = Aabb(Vector3(-0.0664351f, 34.5934f, -3.35439f), Vector3(6.28563f, 37.9087f, 3.2811f)); 
  data[26].mAabb = Aabb(Vector3(-38.7208f, 30.13f, 5.87562f), Vector3(-35.4067f, 31.2805f, 6.52032f)); 
  data[27].mAabb = Aabb(Vector3(31.2328f, 13.4772f, -13.5699f), Vector3(36.1137f, 19.4818f, -10.9718f)); 
  data[28].mAabb = Aabb(Vector3(19.3803f, 20.0414f, -17.9924f), Vector3(22.687f, 27.6184f, -15.0635f)); 
  data[29].mAabb = Aabb(Vector3(13.2658f, -3.10248f, 44.7575f), Vector3(13.8998f, -1.1058f, 50.7649f)); 
  data[30].mAabb = Aabb(Vector3(-27.6913f, 21.3224f, 24.1016f), Vector3(-21.6748f, 24.9457f, 30.3286f)); 
  data[31].mAabb = Aabb(Vector3(33.5344f, 4.53689f, 12.0292f), Vector3(38.1974f, 10.901f, 17.6054f)); 
  data[32].mAabb = Aabb(Vector3(-27.2269f, -23.6772f, 11.8715f), Vector3(-22.1935f, -17.7305f, 14.3566f)); 
  data[33].mAabb = Aabb(Vector3(15.4885f, 16.445f, 9.81774f), Vector3(17.7217f, 21.3991f, 12.0415f)); 
  data[34].mAabb = Aabb(Vector3(-0.75924f, 28.9268f, -39.5455f), Vector3(-0.147479f, 31.8867f, -37.6087f)); 
  data[35].mAabb = Aabb(Vector3(-12.3936f, 20.0923f, 0.600992f), Vector3(-7.59716f, 26.6213f, 4.31427f)); 
  data[36].mAabb = Aabb(Vector3(20.9606f, -33.1068f, -8.23454f), Vector3(23.2075f, -29.9572f, -7.02669f)); 
  data[37].mAabb = Aabb(Vector3(5.26249f, 13.7056f, -40.2807f), Vector3(6.24188f, 16.3531f, -33.0311f)); 
  data[38].mAabb = Aabb(Vector3(29.3558f, -2.74572f, -31.9162f), Vector3(35.5091f, 3.94614f, -25.2037f)); 
  data[39].mAabb = Aabb(Vector3(35.5737f, -6.2788f, 16.4102f), Vector3(36.6765f, -2.11974f, 19.4612f)); 
  data[40].mAabb = Aabb(Vector3(14.262f, -37.2939f, -25.5913f), Vector3(21.5144f, -34.5578f, -21.7368f)); 
  data[41].mAabb = Aabb(Vector3(13.0193f, -1.3593f, -17.3694f), Vector3(14.1629f, 6.40691f, -12.0581f)); 
  data[42].mAabb = Aabb(Vector3(16.3316f, 1.76571f, -21.8142f), Vector3(19.0197f, 9.08056f, -19.8653f)); 
  data[43].mAabb = Aabb(Vector3(-19.5603f, 20.6095f, 15.5045f), Vector3(-12.1081f, 21.3317f, 22.2599f)); 
  data[44].mAabb = Aabb(Vector3(-10.4086f, -8.62528f, 13.0994f), Vector3(-9.53354f, -7.58211f, 20.4711f)); 
  data[45].mAabb = Aabb(Vector3(13.6654f, 20.3444f, -36.5234f), Vector3(15.0036f, 21.8689f, -28.9117f)); 
  data[46].mAabb = Aabb(Vector3(23.8628f, -38.4955f, -20.6271f), Vector3(24.5615f, -36.5238f, -18.9758f)); 
  data[47].mAabb = Aabb(Vector3(1.40092f, -36.1411f, -29.0889f), Vector3(9.03817f, -34.6756f, -21.483f)); 
  data[48].mAabb = Aabb(Vector3(10.3223f, 1.22148f, 41.9818f), Vector3(10.7982f, 4.77425f, 47.0032f)); 
  data[49].mAabb = Aabb(Vector3(15.6154f, 2.98778f, 41.6033f), Vector3(17.5842f, 5.4945f, 46.9961f)); 
  data[50].mAabb = Aabb(Vector3(-32.646f, -35.3145f, 8.1402f), Vector3(-31.3556f, -29.8294f, 13.8746f)); 
  data[51].mAabb = Aabb(Vector3(-5.14212f, -0.956564f, 31.2489f), Vector3(-1.94028f, 0.177992f, 31.6848f)); 
  data[52].mAabb = Aabb(Vector3(-22.6825f, -9.23554f, 17.2008f), Vector3(-19.1677f, -5.03681f, 20.0605f)); 
  data[53].mAabb = Aabb(Vector3(-21.2914f, -15.0141f, -27.3347f), Vector3(-19.0551f, -7.75093f, -23.5838f)); 
  data[54].mAabb = Aabb(Vector3(-18.0578f, 41.4532f, 15.2477f), Vector3(-11.3265f, 44.9935f, 18.9227f)); 
  data[55].mAabb = Aabb(Vector3(3.37939f, -13.8279f, -32.4219f), Vector3(8.5961f, -12.7836f, -25.5009f)); 
  data[56].mAabb = Aabb(Vector3(31.9422f, 14.4441f, 20.9112f), Vector3(36.2073f, 18.0697f, 23.8718f)); 
  data[57].mAabb = Aabb(Vector3(-14.4835f, -43.1178f, 0.316063f), Vector3(-12.5961f, -35.2644f, 7.31617f)); 
  data[58].mAabb = Aabb(Vector3(-28.5474f, 11.183f, -9.8139f), Vector3(-23.2066f, 16.8009f, -4.09991f)); 
  data[59].mAabb = Aabb(Vector3(24.4852f, 0.0643864f, -31.1281f), Vector3(24.8905f, 4.36424f, -24.8113f)); 
  data[60].mAabb = Aabb(Vector3(22.7821f, 19.3287f, -37.4432f), Vector3(30.459f, 24.9251f, -30.8492f)); 
  data[61].mAabb = Aabb(Vector3(22.1717f, 4.31385f, -2.76737f), Vector3(29.8894f, 7.05784f, -1.5811f)); 
  data[62].mAabb = Aabb(Vector3(-16.9136f, 30.0835f, -19.6568f), Vector3(-10.1848f, 37.7606f, -16.8497f)); 
  data[63].mAabb = Aabb(Vector3(18.4096f, -37.3217f, -14.8637f), Vector3(24.0318f, -32.9567f, -7.1133f)); 
  data[64].mAabb = Aabb(Vector3(5.43948f, -6.07047f, 34.3527f), Vector3(7.83185f, 0.678669f, 41.3045f)); 
  data[65].mAabb = Aabb(Vector3(8.21705f, -27.748f, 21.2359f), Vector3(15.0381f, -26.6948f, 23.3875f)); 
  data[66].mAabb = Aabb(Vector3(-20.0836f, 36.9757f, -10.8075f), Vector3(-14.2988f, 38.3633f, -3.48038f)); 
  data[67].mAabb = Aabb(Vector3(12.8215f, -48.9269f, 2.8112f), Vector3(18.2992f, -43.9417f, 8.46906f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest23(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(54); 
  data[0].mAabb = Aabb(Vector3(3.43515f, -27.3444f, 25.0632f), Vector3(5.26019f, -23.0966f, 26.9592f)); 
  data[1].mAabb = Aabb(Vector3(24.8261f, 17.7646f, 6.67505f), Vector3(26.7743f, 18.7632f, 14.2432f)); 
  data[2].mAabb = Aabb(Vector3(-36.2072f, 26.1236f, 1.25446f), Vector3(-28.8926f, 32.4288f, 8.42109f)); 
  data[3].mAabb = Aabb(Vector3(10.2676f, -16.4412f, -25.8875f), Vector3(11.041f, -13.1308f, -24.7782f)); 
  data[4].mAabb = Aabb(Vector3(-17.4262f, -23.6016f, 17.5359f), Vector3(-13.9959f, -19.0585f, 22.5404f)); 
  data[5].mAabb = Aabb(Vector3(-48.7334f, -7.099f, -5.10987f), Vector3(-45.1307f, -6.65934f, -2.81283f)); 
  data[6].mAabb = Aabb(Vector3(-7.35578f, -11.0351f, -14.5851f), Vector3(-5.8754f, -4.48399f, -10.6849f)); 
  data[7].mAabb = Aabb(Vector3(18.634f, -45.7991f, -19.027f), Vector3(21.7031f, -38.2192f, -13.7567f)); 
  data[8].mAabb = Aabb(Vector3(3.30119f, -11.6326f, -15.9436f), Vector3(10.6865f, -5.17778f, -11.1272f)); 
  data[9].mAabb = Aabb(Vector3(8.44593f, -13.918f, 22.337f), Vector3(15.8754f, -12.9644f, 28.5249f)); 
  data[10].mAabb = Aabb(Vector3(4.75162f, 4.38916f, 2.55452f), Vector3(5.7085f, 8.71452f, 5.37088f)); 
  data[11].mAabb = Aabb(Vector3(-11.8025f, -7.11923f, 0.223166f), Vector3(-10.3592f, 0.4983f, 6.6912f)); 
  data[12].mAabb = Aabb(Vector3(-0.971197f, 29.0055f, 12.0105f), Vector3(3.46573f, 30.0246f, 13.6027f)); 
  data[13].mAabb = Aabb(Vector3(39.0716f, 4.42108f, -26.0647f), Vector3(41.1666f, 6.37508f, -22.9072f)); 
  data[14].mAabb = Aabb(Vector3(5.52161f, 20.1395f, -44.6782f), Vector3(10.9211f, 24.4539f, -42.9756f)); 
  data[15].mAabb = Aabb(Vector3(-30.1862f, -33.864f, -6.25897f), Vector3(-23.6265f, -30.0438f, -1.11532f)); 
  data[16].mAabb = Aabb(Vector3(2.95419f, -10.9418f, 22.5388f), Vector3(7.49108f, -8.53409f, 25.6764f)); 
  data[17].mAabb = Aabb(Vector3(24.2279f, -10.3871f, 32.6796f), Vector3(30.2613f, -8.48182f, 36.401f)); 
  data[18].mAabb = Aabb(Vector3(-41.7828f, 6.44654f, 4.67973f), Vector3(-35.1454f, 7.87682f, 10.4972f)); 
  data[19].mAabb = Aabb(Vector3(32.8642f, 7.78372f, -18.4883f), Vector3(40.1011f, 13.8551f, -13.8584f)); 
  data[20].mAabb = Aabb(Vector3(-2.26512f, 13.0314f, 29.7217f), Vector3(2.60322f, 20.0793f, 32.764f)); 
  data[21].mAabb = Aabb(Vector3(22.731f, -4.91562f, 7.57722f), Vector3(29.9385f, -0.685351f, 12.045f)); 
  data[22].mAabb = Aabb(Vector3(26.4404f, -0.0377407f, -1.18892f), Vector3(31.1172f, 1.16918f, 3.51521f)); 
  data[23].mAabb = Aabb(Vector3(-40.053f, 17.0774f, -13.745f), Vector3(-32.4657f, 23.0011f, -10.3423f)); 
  data[24].mAabb = Aabb(Vector3(-2.05427f, 14.9471f, -13.1157f), Vector3(-1.43184f, 21.8384f, -11.4458f)); 
  data[25].mAabb = Aabb(Vector3(-9.75949f, 32.9414f, 14.114f), Vector3(-3.33343f, 34.31f, 19.5996f)); 
  data[26].mAabb = Aabb(Vector3(-5.98203f, 46.6668f, -10.0211f), Vector3(-0.330892f, 49.8122f, -3.06549f)); 
  data[27].mAabb = Aabb(Vector3(-32.1954f, -28.5431f, 5.30341f), Vector3(-31.6897f, -25.6126f, 12.5867f)); 
  data[28].mAabb = Aabb(Vector3(-26.9808f, 12.5689f, -13.2746f), Vector3(-26.4553f, 16.2054f, -11.779f)); 
  data[29].mAabb = Aabb(Vector3(-18.9932f, 17.3939f, 30.7511f), Vector3(-14.3882f, 22.7785f, 32.8918f)); 
  data[30].mAabb = Aabb(Vector3(-15.6324f, 12.5958f, 2.0871f), Vector3(-13.9089f, 20.4044f, 5.81476f)); 
  data[31].mAabb = Aabb(Vector3(-16.4476f, 43.1284f, 10.7269f), Vector3(-9.73815f, 44.6591f, 11.5154f)); 
  data[32].mAabb = Aabb(Vector3(17.2851f, -39.4407f, 18.8904f), Vector3(18.8003f, -38.343f, 26.0879f)); 
  data[33].mAabb = Aabb(Vector3(4.9982f, -22.0389f, 24.1368f), Vector3(7.38268f, -18.5149f, 28.5394f)); 
  data[34].mAabb = Aabb(Vector3(-37.6946f, 14.0464f, 24.7043f), Vector3(-36.3575f, 16.1136f, 30.1084f)); 
  data[35].mAabb = Aabb(Vector3(-3.05826f, -4.01003f, -45.958f), Vector3(1.4675f, -2.72982f, -39.1237f)); 
  data[36].mAabb = Aabb(Vector3(28.6069f, 1.18853f, 14.2346f), Vector3(34.7737f, 7.31468f, 21.9273f)); 
  data[37].mAabb = Aabb(Vector3(10.4373f, -0.679533f, -36.1031f), Vector3(14.9275f, 5.2439f, -28.3828f)); 
  data[38].mAabb = Aabb(Vector3(29.6627f, 8.07294f, 33.3075f), Vector3(33.0812f, 14.1678f, 40.3166f)); 
  data[39].mAabb = Aabb(Vector3(-13.3104f, -5.08334f, -5.57081f), Vector3(-12.078f, -1.84856f, 1.21452f)); 
  data[40].mAabb = Aabb(Vector3(21.5308f, 9.6443f, 8.46772f), Vector3(28.4004f, 17.3231f, 9.39654f)); 
  data[41].mAabb = Aabb(Vector3(-10.9404f, 31.7773f, 1.82411f), Vector3(-4.89169f, 34.8437f, 8.47399f)); 
  data[42].mAabb = Aabb(Vector3(12.0005f, -28.0499f, 35.7922f), Vector3(18.7276f, -20.7366f, 39.765f)); 
  data[43].mAabb = Aabb(Vector3(27.1799f, -22.2741f, -20.393f), Vector3(29.9264f, -20.1909f, -18.3358f)); 
  data[44].mAabb = Aabb(Vector3(-16.8507f, -33.592f, 18.8789f), Vector3(-11.6969f, -31.1513f, 19.5011f)); 
  data[45].mAabb = Aabb(Vector3(-21.3866f, -33.0826f, 10.27f), Vector3(-14.935f, -25.2898f, 15.9003f)); 
  data[46].mAabb = Aabb(Vector3(12.5543f, 29.0273f, 31.6463f), Vector3(13.7128f, 30.7146f, 36.6914f)); 
  data[47].mAabb = Aabb(Vector3(-8.41456f, 1.16253f, 11.7648f), Vector3(-5.15218f, 3.19284f, 16.2342f)); 
  data[48].mAabb = Aabb(Vector3(-11.1415f, -0.147535f, 9.01853f), Vector3(-5.77288f, 6.04657f, 11.1653f)); 
  data[49].mAabb = Aabb(Vector3(-5.46862f, 13.0981f, -37.8518f), Vector3(-0.299688f, 20.1539f, -33.7415f)); 
  data[50].mAabb = Aabb(Vector3(23.8588f, 11.1045f, -39.3638f), Vector3(25.6541f, 16.6763f, -35.0053f)); 
  data[51].mAabb = Aabb(Vector3(-17.6979f, -32.1186f, 29.848f), Vector3(-15.2556f, -28.8891f, 30.7926f)); 
  data[52].mAabb = Aabb(Vector3(-15.5975f, 39.2457f, 14.6066f), Vector3(-10.092f, 44.5844f, 20.9798f)); 
  data[53].mAabb = Aabb(Vector3(-10.7883f, -10.6534f, -13.2828f), Vector3(-8.47384f, -4.82433f, -11.6576f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest24(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(50); 
  data[0].mAabb = Aabb(Vector3(-42.6785f, 7.69574f, -4.06954f), Vector3(-38.8344f, 12.6232f, 2.05127f)); 
  data[1].mAabb = Aabb(Vector3(40.6134f, 3.33532f, 2.41273f), Vector3(47.1749f, 8.73387f, 5.88525f)); 
  data[2].mAabb = Aabb(Vector3(-5.62089f, -3.16774f, 39.4587f), Vector3(-2.96271f, 2.00862f, 40.1992f)); 
  data[3].mAabb = Aabb(Vector3(28.1782f, -20.0452f, -3.20601f), Vector3(35.8009f, -16.9245f, -1.50993f)); 
  data[4].mAabb = Aabb(Vector3(12.5306f, 45.9422f, 5.57761f), Vector3(14.9249f, 47.1767f, 7.56872f)); 
  data[5].mAabb = Aabb(Vector3(-2.51602f, 26.5457f, 35.8502f), Vector3(-1.75999f, 33.0163f, 38.4386f)); 
  data[6].mAabb = Aabb(Vector3(5.907f, 31.6066f, 1.48749f), Vector3(8.09016f, 38.4613f, 8.87076f)); 
  data[7].mAabb = Aabb(Vector3(-7.36033f, -25.2672f, -22.8515f), Vector3(-4.52031f, -22.9154f, -17.8222f)); 
  data[8].mAabb = Aabb(Vector3(8.12211f, 42.4973f, -2.46604f), Vector3(13.0992f, 44.3184f, -1.02509f)); 
  data[9].mAabb = Aabb(Vector3(-28.0211f, -11.056f, -20.0056f), Vector3(-22.9471f, -5.93411f, -14.132f)); 
  data[10].mAabb = Aabb(Vector3(19.0333f, -42.6273f, 12.0529f), Vector3(26.6777f, -40.111f, 13.5068f)); 
  data[11].mAabb = Aabb(Vector3(-8.3786f, 29.5851f, -12.1517f), Vector3(-4.1481f, 33.2787f, -6.46064f)); 
  data[12].mAabb = Aabb(Vector3(-27.3783f, 5.62397f, 15.1843f), Vector3(-21.2779f, 9.40149f, 18.1417f)); 
  data[13].mAabb = Aabb(Vector3(-39.1405f, 14.0057f, -1.73391f), Vector3(-33.07f, 21.3423f, 5.48468f)); 
  data[14].mAabb = Aabb(Vector3(1.56827f, -9.42781f, 48.0971f), Vector3(4.45792f, -4.82365f, 48.5544f)); 
  data[15].mAabb = Aabb(Vector3(44.1179f, 10.7153f, 8.69284f), Vector3(49.5582f, 12.5498f, 10.2273f)); 
  data[16].mAabb = Aabb(Vector3(0.221786f, 17.6731f, -34.1386f), Vector3(1.04763f, 24.9557f, -29.827f)); 
  data[17].mAabb = Aabb(Vector3(8.13254f, -17.731f, -23.3287f), Vector3(12.6956f, -13.8672f, -21.756f)); 
  data[18].mAabb = Aabb(Vector3(-0.769997f, -10.6751f, 32.398f), Vector3(-0.259826f, -5.79025f, 33.6745f)); 
  data[19].mAabb = Aabb(Vector3(-37.504f, -7.74894f, -15.9591f), Vector3(-31.8703f, -5.29186f, -11.9749f)); 
  data[20].mAabb = Aabb(Vector3(-38.2601f, -20.8591f, -15.1039f), Vector3(-37.7268f, -14.5478f, -7.46733f)); 
  data[21].mAabb = Aabb(Vector3(25.1445f, 24.6093f, 21.9224f), Vector3(26.3776f, 30.4207f, 29.2813f)); 
  data[22].mAabb = Aabb(Vector3(-7.12318f, -13.4432f, 32.8604f), Vector3(-4.14492f, -6.01192f, 35.6853f)); 
  data[23].mAabb = Aabb(Vector3(41.2884f, -13.8393f, 6.43863f), Vector3(46.3602f, -8.73347f, 12.2644f)); 
  data[24].mAabb = Aabb(Vector3(-20.4183f, -1.17592f, 7.2597f), Vector3(-12.5837f, 1.91645f, 13.712f)); 
  data[25].mAabb = Aabb(Vector3(32.9061f, 20.2758f, -15.63f), Vector3(35.0832f, 25.4668f, -13.1947f)); 
  data[26].mAabb = Aabb(Vector3(-16.1523f, -14.8389f, 0.324913f), Vector3(-9.61607f, -13.0837f, 3.37414f)); 
  data[27].mAabb = Aabb(Vector3(7.42553f, -15.401f, 1.5164f), Vector3(12.8626f, -13.1917f, 8.46107f)); 
  data[28].mAabb = Aabb(Vector3(-2.00224f, -23.6626f, -42.3877f), Vector3(1.83466f, -22.9451f, -34.5434f)); 
  data[29].mAabb = Aabb(Vector3(0.736747f, 22.1681f, 34.5396f), Vector3(7.1454f, 26.9831f, 36.151f)); 
  data[30].mAabb = Aabb(Vector3(-6.27957f, 28.6789f, -14.4029f), Vector3(-4.04469f, 33.9708f, -13.7232f)); 
  data[31].mAabb = Aabb(Vector3(7.5041f, -33.0367f, 20.3713f), Vector3(14.8152f, -31.1875f, 27.1866f)); 
  data[32].mAabb = Aabb(Vector3(-22.5277f, 16.8789f, 23.9508f), Vector3(-21.5695f, 21.176f, 30.1389f)); 
  data[33].mAabb = Aabb(Vector3(14.7314f, 2.38687f, 15.4474f), Vector3(20.44f, 5.92689f, 17.5594f)); 
  data[34].mAabb = Aabb(Vector3(-35.1161f, -21.1079f, -24.5879f), Vector3(-30.5908f, -20.6789f, -22.8558f)); 
  data[35].mAabb = Aabb(Vector3(-33.8261f, 12.8555f, -17.0116f), Vector3(-29.0255f, 18.7372f, -15.2712f)); 
  data[36].mAabb = Aabb(Vector3(-12.7585f, -13.7923f, -21.3772f), Vector3(-11.0675f, -5.9231f, -14.1496f)); 
  data[37].mAabb = Aabb(Vector3(-32.4312f, -9.73474f, -29.4389f), Vector3(-28.0024f, -4.47095f, -27.2493f)); 
  data[38].mAabb = Aabb(Vector3(2.7486f, -15.9177f, 12.1691f), Vector3(3.74283f, -14.2235f, 20.128f)); 
  data[39].mAabb = Aabb(Vector3(-17.0413f, 19.6664f, 3.87041f), Vector3(-9.32662f, 23.8733f, 5.91023f)); 
  data[40].mAabb = Aabb(Vector3(-12.4052f, -28.1093f, 20.7063f), Vector3(-4.69028f, -23.0526f, 24.4061f)); 
  data[41].mAabb = Aabb(Vector3(7.90641f, -30.5685f, 18.9459f), Vector3(13.9481f, -24.4992f, 20.6366f)); 
  data[42].mAabb = Aabb(Vector3(-6.52163f, 16.7139f, 11.4416f), Vector3(-0.865626f, 18.9984f, 13.8933f)); 
  data[43].mAabb = Aabb(Vector3(-6.91701f, 1.74463f, -3.42236f), Vector3(0.773576f, 9.73396f, -2.63316f)); 
  data[44].mAabb = Aabb(Vector3(31.9325f, 14.8833f, 12.1571f), Vector3(35.2951f, 15.56f, 13.4705f)); 
  data[45].mAabb = Aabb(Vector3(-3.94889f, -25.508f, -22.47f), Vector3(0.494761f, -20.3873f, -15.1013f)); 
  data[46].mAabb = Aabb(Vector3(8.9943f, -17.2086f, -32.9662f), Vector3(11.9842f, -12.0587f, -32.1737f)); 
  data[47].mAabb = Aabb(Vector3(-19.5179f, -26.8214f, 12.1822f), Vector3(-17.2478f, -18.9923f, 12.6924f)); 
  data[48].mAabb = Aabb(Vector3(-33.9255f, 9.69235f, 25.366f), Vector3(-29.2926f, 10.1891f, 29.2397f)); 
  data[49].mAabb = Aabb(Vector3(9.9049f, -19.3854f, 23.0492f), Vector3(12.7094f, -18.0776f, 27.9959f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest25(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(46); 
  data[0].mAabb = Aabb(Vector3(-12.907f, -34.0479f, -10.1282f), Vector3(-12.353f, -27.2071f, -9.30767f)); 
  data[1].mAabb = Aabb(Vector3(-21.4431f, -18.0547f, -41.7765f), Vector3(-14.835f, -14.2739f, -40.5137f)); 
  data[2].mAabb = Aabb(Vector3(-11.7776f, -34.423f, 26.0697f), Vector3(-8.61801f, -32.6367f, 29.298f)); 
  data[3].mAabb = Aabb(Vector3(-0.628168f, 31.0418f, 29.513f), Vector3(3.70671f, 34.9107f, 31.158f)); 
  data[4].mAabb = Aabb(Vector3(5.23973f, 9.39792f, -6.07075f), Vector3(9.1933f, 13.5758f, -2.91599f)); 
  data[5].mAabb = Aabb(Vector3(-2.14414f, 34.2687f, 9.30831f), Vector3(0.617482f, 40.3007f, 16.5705f)); 
  data[6].mAabb = Aabb(Vector3(-30.8581f, -17.8825f, -7.54199f), Vector3(-26.9068f, -10.8211f, -0.958217f)); 
  data[7].mAabb = Aabb(Vector3(13.7491f, -9.10349f, 20.5157f), Vector3(19.7403f, -2.19222f, 25.6649f)); 
  data[8].mAabb = Aabb(Vector3(-27.8068f, -15.249f, 5.86678f), Vector3(-20.5544f, -7.89794f, 12.3956f)); 
  data[9].mAabb = Aabb(Vector3(-6.15743f, 34.5004f, -18.5921f), Vector3(0.905299f, 37.8811f, -14.0902f)); 
  data[10].mAabb = Aabb(Vector3(-10.7765f, 4.77225f, 41.2311f), Vector3(-10.1721f, 6.3635f, 43.6307f)); 
  data[11].mAabb = Aabb(Vector3(29.5377f, 15.3318f, 13.106f), Vector3(31.9748f, 20.4806f, 18.6286f)); 
  data[12].mAabb = Aabb(Vector3(20.9707f, -36.1775f, 4.70682f), Vector3(23.479f, -35.0898f, 11.5622f)); 
  data[13].mAabb = Aabb(Vector3(13.458f, 2.79324f, 23.7564f), Vector3(17.1613f, 5.28048f, 27.8401f)); 
  data[14].mAabb = Aabb(Vector3(-28.507f, -9.08115f, -21.226f), Vector3(-24.5275f, -1.80712f, -16.4697f)); 
  data[15].mAabb = Aabb(Vector3(-29.596f, 5.91054f, 25.5958f), Vector3(-22.2739f, 12.5597f, 31.9174f)); 
  data[16].mAabb = Aabb(Vector3(-33.0126f, -1.36828f, -43.9177f), Vector3(-26.135f, 4.60387f, -36.489f)); 
  data[17].mAabb = Aabb(Vector3(-28.023f, -35.5342f, 0.905349f), Vector3(-27.0053f, -32.3938f, 1.5795f)); 
  data[18].mAabb = Aabb(Vector3(-38.3188f, 9.64848f, 11.4469f), Vector3(-33.6026f, 14.6639f, 18.7002f)); 
  data[19].mAabb = Aabb(Vector3(-18.9556f, 2.13316f, 43.3877f), Vector3(-13.3198f, 5.93851f, 49.2618f)); 
  data[20].mAabb = Aabb(Vector3(-35.521f, -4.06745f, -30.3149f), Vector3(-30.0554f, 2.62116f, -24.1254f)); 
  data[21].mAabb = Aabb(Vector3(20.7652f, -2.51009f, 33.173f), Vector3(23.8339f, 4.97987f, 38.7805f)); 
  data[22].mAabb = Aabb(Vector3(-19.1817f, 13.4021f, 39.1307f), Vector3(-17.1208f, 21.2553f, 45.9809f)); 
  data[23].mAabb = Aabb(Vector3(-35.6336f, -0.753111f, 32.0813f), Vector3(-28.1592f, 1.29135f, 34.4382f)); 
  data[24].mAabb = Aabb(Vector3(-4.30039f, 19.1668f, 32.6918f), Vector3(1.02859f, 21.9854f, 35.6299f)); 
  data[25].mAabb = Aabb(Vector3(-27.6035f, 31.1993f, -2.65375f), Vector3(-26.4154f, 38.6287f, -1.51896f)); 
  data[26].mAabb = Aabb(Vector3(-7.51003f, -27.3754f, -11.6965f), Vector3(-0.466092f, -21.5513f, -9.6414f)); 
  data[27].mAabb = Aabb(Vector3(-19.3712f, 44.6967f, -7.28172f), Vector3(-11.6184f, 48.5816f, -3.65937f)); 
  data[28].mAabb = Aabb(Vector3(-15.9641f, -25.7583f, -30.8668f), Vector3(-12.9369f, -18.3718f, -27.1672f)); 
  data[29].mAabb = Aabb(Vector3(-16.1898f, -24.8643f, -38.3178f), Vector3(-15.5242f, -21.5351f, -32.4896f)); 
  data[30].mAabb = Aabb(Vector3(-4.20743f, -15.2055f, -4.27692f), Vector3(0.59643f, -12.2185f, -0.813909f)); 
  data[31].mAabb = Aabb(Vector3(-28.6608f, -8.26252f, -22.8417f), Vector3(-24.7912f, -6.71789f, -19.0669f)); 
  data[32].mAabb = Aabb(Vector3(-11.5803f, 35.1596f, -0.0100648f), Vector3(-9.05643f, 35.9114f, 7.02715f)); 
  data[33].mAabb = Aabb(Vector3(-5.42918f, 19.2882f, -21.0664f), Vector3(1.1583f, 20.2073f, -13.1357f)); 
  data[34].mAabb = Aabb(Vector3(-3.47328f, 14.1083f, -13.3364f), Vector3(1.79376f, 16.2995f, -9.89034f)); 
  data[35].mAabb = Aabb(Vector3(-37.33f, -4.43437f, -20.0114f), Vector3(-34.4086f, -3.53546f, -17.5154f)); 
  data[36].mAabb = Aabb(Vector3(13.8659f, -32.3484f, 23.4072f), Vector3(21.3046f, -25.2892f, 26.1229f)); 
  data[37].mAabb = Aabb(Vector3(-14.142f, -24.0925f, 25.09f), Vector3(-7.31608f, -22.5708f, 27.3481f)); 
  data[38].mAabb = Aabb(Vector3(-16.7962f, -45.2309f, 11.2653f), Vector3(-9.42455f, -39.233f, 15.0565f)); 
  data[39].mAabb = Aabb(Vector3(17.2629f, -7.88146f, -18.1837f), Vector3(20.5292f, -7.09389f, -11.8593f)); 
  data[40].mAabb = Aabb(Vector3(-22.1151f, 1.87673f, 19.0834f), Vector3(-19.2674f, 6.30554f, 24.6005f)); 
  data[41].mAabb = Aabb(Vector3(-30.9846f, 23.7563f, -26.1445f), Vector3(-25.6063f, 24.5903f, -24.6929f)); 
  data[42].mAabb = Aabb(Vector3(16.012f, -1.78612f, -29.9323f), Vector3(22.0579f, 1.56347f, -27.4418f)); 
  data[43].mAabb = Aabb(Vector3(30.0609f, -29.7401f, -20.5292f), Vector3(33.6985f, -26.0662f, -16.2505f)); 
  data[44].mAabb = Aabb(Vector3(-18.8166f, -0.112072f, -12.7115f), Vector3(-16.1795f, 3.19345f, -10.1083f)); 
  data[45].mAabb = Aabb(Vector3(20.009f, -26.3102f, 31.6562f), Vector3(27.2649f, -22.4485f, 38.0342f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest26(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(59); 
  data[0].mAabb = Aabb(Vector3(12.8457f, -33.1238f, -4.81694f), Vector3(13.3257f, -27.7258f, 0.153915f)); 
  data[1].mAabb = Aabb(Vector3(25.9984f, 4.75766f, -24.785f), Vector3(27.4349f, 6.48506f, -19.4108f)); 
  data[2].mAabb = Aabb(Vector3(34.9239f, -2.27698f, -15.7082f), Vector3(39.355f, -0.665784f, -13.6751f)); 
  data[3].mAabb = Aabb(Vector3(1.1166f, -9.20224f, -31.8995f), Vector3(5.19819f, -5.18258f, -28.4138f)); 
  data[4].mAabb = Aabb(Vector3(3.3685f, 24.8634f, 4.73788f), Vector3(8.89648f, 31.404f, 8.81251f)); 
  data[5].mAabb = Aabb(Vector3(16.939f, -31.9685f, 11.5557f), Vector3(20.999f, -26.0699f, 16.1204f)); 
  data[6].mAabb = Aabb(Vector3(-4.51229f, -7.36364f, 31.7046f), Vector3(-2.89693f, -6.20357f, 35.7486f)); 
  data[7].mAabb = Aabb(Vector3(-24.9178f, -30.2426f, -4.99932f), Vector3(-24.0317f, -29.546f, -1.4157f)); 
  data[8].mAabb = Aabb(Vector3(27.3129f, -5.56594f, 8.90321f), Vector3(30.1875f, 1.11919f, 16.1304f)); 
  data[9].mAabb = Aabb(Vector3(5.71302f, -40.1237f, -16.3231f), Vector3(7.58492f, -39.1954f, -8.47035f)); 
  data[10].mAabb = Aabb(Vector3(18.5692f, 26.2157f, -35.0954f), Vector3(21.1135f, 26.6656f, -33.6363f)); 
  data[11].mAabb = Aabb(Vector3(10.77f, -43.346f, 7.04107f), Vector3(17.8165f, -40.5693f, 12.8536f)); 
  data[12].mAabb = Aabb(Vector3(-36.3506f, 4.31897f, 7.27418f), Vector3(-29.7295f, 8.78605f, 9.37824f)); 
  data[13].mAabb = Aabb(Vector3(-23.8453f, -18.3146f, 11.5149f), Vector3(-23.1612f, -11.5541f, 14.3939f)); 
  data[14].mAabb = Aabb(Vector3(-28.9046f, 18.6682f, 3.54239f), Vector3(-21.5626f, 19.29f, 5.05641f)); 
  data[15].mAabb = Aabb(Vector3(12.3396f, -0.0888942f, -7.6856f), Vector3(15.6887f, 1.16163f, -6.14005f)); 
  data[16].mAabb = Aabb(Vector3(-17.5699f, 37.6498f, 18.1354f), Vector3(-15.7013f, 44.3224f, 24.2173f)); 
  data[17].mAabb = Aabb(Vector3(-34.0123f, 21.2554f, 14.7082f), Vector3(-27.4404f, 22.9494f, 19.0728f)); 
  data[18].mAabb = Aabb(Vector3(36.5172f, 13.2203f, -12.6557f), Vector3(38.3005f, 18.2345f, -6.85543f)); 
  data[19].mAabb = Aabb(Vector3(10.4949f, -14.5574f, 15.5445f), Vector3(14.1604f, -7.94972f, 16.1558f)); 
  data[20].mAabb = Aabb(Vector3(-50.0522f, 4.23884f, -9.82281f), Vector3(-44.4446f, 9.4087f, -3.58579f)); 
  data[21].mAabb = Aabb(Vector3(16.2649f, -31.1473f, 7.20675f), Vector3(23.4665f, -29.9411f, 14.8563f)); 
  data[22].mAabb = Aabb(Vector3(-3.65148f, -4.32222f, 25.5253f), Vector3(-2.0927f, 3.64948f, 31.368f)); 
  data[23].mAabb = Aabb(Vector3(-2.65341f, -10.3394f, -37.1721f), Vector3(-2.00222f, -3.80987f, -30.9643f)); 
  data[24].mAabb = Aabb(Vector3(-21.1313f, 19.9986f, 5.23309f), Vector3(-17.9753f, 27.9924f, 8.13248f)); 
  data[25].mAabb = Aabb(Vector3(-10.7903f, 13.8409f, 8.27178f), Vector3(-7.28279f, 14.7623f, 16.0526f)); 
  data[26].mAabb = Aabb(Vector3(22.1654f, 3.78217f, 9.28086f), Vector3(24.6522f, 11.1835f, 15.5074f)); 
  data[27].mAabb = Aabb(Vector3(5.8266f, 32.5058f, -26.4f), Vector3(13.2273f, 33.0436f, -24.2338f)); 
  data[28].mAabb = Aabb(Vector3(32.3291f, 4.09362f, -9.49272f), Vector3(35.678f, 7.07001f, -4.73966f)); 
  data[29].mAabb = Aabb(Vector3(-12.5194f, -3.33043f, 0.849799f), Vector3(-12.0016f, 0.801262f, 1.40218f)); 
  data[30].mAabb = Aabb(Vector3(4.30058f, 8.7736f, -50.9595f), Vector3(8.17946f, 11.8562f, -44.0872f)); 
  data[31].mAabb = Aabb(Vector3(-28.7734f, -19.1828f, 1.17018f), Vector3(-23.7286f, -13.9946f, 6.93636f)); 
  data[32].mAabb = Aabb(Vector3(-34.0707f, -23.8971f, 29.0424f), Vector3(-26.7921f, -17.9642f, 34.9299f)); 
  data[33].mAabb = Aabb(Vector3(-6.98919f, -28.1844f, -15.1517f), Vector3(-4.05756f, -23.5174f, -11.6567f)); 
  data[34].mAabb = Aabb(Vector3(42.1819f, 5.28397f, -21.8024f), Vector3(48.4363f, 8.80728f, -16.5382f)); 
  data[35].mAabb = Aabb(Vector3(-23.3998f, -15.546f, 4.03485f), Vector3(-21.893f, -8.53058f, 7.73304f)); 
  data[36].mAabb = Aabb(Vector3(-4.6439f, -20.6333f, 30.7744f), Vector3(-0.0627046f, -17.2417f, 36.3743f)); 
  data[37].mAabb = Aabb(Vector3(-16.3085f, -20.061f, 11.3997f), Vector3(-8.46387f, -18.9455f, 17.013f)); 
  data[38].mAabb = Aabb(Vector3(39.8094f, 1.04716f, -0.868682f), Vector3(46.0119f, 4.41553f, 2.55652f)); 
  data[39].mAabb = Aabb(Vector3(-14.1151f, -18.8691f, -4.10522f), Vector3(-10.3149f, -11.2405f, 2.67361f)); 
  data[40].mAabb = Aabb(Vector3(-3.88558f, 37.3754f, 5.38631f), Vector3(0.992733f, 40.0535f, 7.93246f)); 
  data[41].mAabb = Aabb(Vector3(4.43175f, 25.8964f, -16.3665f), Vector3(6.65828f, 30.2914f, -10.89f)); 
  data[42].mAabb = Aabb(Vector3(-13.4785f, 5.36819f, -20.8162f), Vector3(-9.88046f, 12.7255f, -16.3716f)); 
  data[43].mAabb = Aabb(Vector3(-21.5017f, 30.5472f, 30.1826f), Vector3(-18.8192f, 31.9052f, 33.1971f)); 
  data[44].mAabb = Aabb(Vector3(13.1455f, -13.49f, 21.2557f), Vector3(18.7099f, -6.54208f, 22.4988f)); 
  data[45].mAabb = Aabb(Vector3(-13.0365f, 23.7708f, -11.4368f), Vector3(-9.88871f, 24.4414f, -10.5803f)); 
  data[46].mAabb = Aabb(Vector3(10.284f, 20.0923f, 28.9782f), Vector3(16.2494f, 22.1242f, 35.1325f)); 
  data[47].mAabb = Aabb(Vector3(4.81162f, -37.1042f, -17.1317f), Vector3(12.4674f, -30.0558f, -10.5164f)); 
  data[48].mAabb = Aabb(Vector3(1.92965f, 10.654f, -10.8798f), Vector3(4.27586f, 17.023f, -10.3608f)); 
  data[49].mAabb = Aabb(Vector3(29.2527f, -8.31629f, 22.6499f), Vector3(33.2081f, -1.88769f, 28.5677f)); 
  data[50].mAabb = Aabb(Vector3(21.6604f, -14.4979f, -14.3172f), Vector3(28.8521f, -10.1027f, -7.16235f)); 
  data[51].mAabb = Aabb(Vector3(-0.361657f, 30.9705f, 19.8889f), Vector3(2.75692f, 32.9758f, 25.2485f)); 
  data[52].mAabb = Aabb(Vector3(-28.6048f, 5.15395f, 30.4715f), Vector3(-24.0723f, 5.80839f, 32.3879f)); 
  data[53].mAabb = Aabb(Vector3(-29.0813f, 13.4144f, -21.5113f), Vector3(-27.8816f, 16.3075f, -16.7555f)); 
  data[54].mAabb = Aabb(Vector3(48.0588f, 2.35223f, -2.74197f), Vector3(49.4499f, 7.46898f, -0.349138f)); 
  data[55].mAabb = Aabb(Vector3(0.794802f, -2.96412f, -29.1122f), Vector3(1.2623f, -1.67346f, -25.2052f)); 
  data[56].mAabb = Aabb(Vector3(19.8774f, -26.9017f, -4.39154f), Vector3(21.7498f, -19.5546f, 3.25104f)); 
  data[57].mAabb = Aabb(Vector3(-18.0412f, 8.14193f, -28.9774f), Vector3(-14.5295f, 8.74163f, -28.0913f)); 
  data[58].mAabb = Aabb(Vector3(-24.8998f, -26.4798f, 14.306f), Vector3(-23.38f, -22.5864f, 20.5931f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest27(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(55); 
  data[0].mAabb = Aabb(Vector3(-39.1228f, -29.8099f, -8.66555f), Vector3(-35.6586f, -25.2549f, -2.44083f)); 
  data[1].mAabb = Aabb(Vector3(8.7546f, -30.5062f, 2.53118f), Vector3(9.2258f, -30.0909f, 3.22366f)); 
  data[2].mAabb = Aabb(Vector3(-1.7148f, 15.4244f, 17.4488f), Vector3(1.36922f, 16.3309f, 24.4473f)); 
  data[3].mAabb = Aabb(Vector3(-36.4183f, 5.95699f, -26.8904f), Vector3(-31.8132f, 8.03763f, -24.6592f)); 
  data[4].mAabb = Aabb(Vector3(-31.4922f, 27.981f, -16.5602f), Vector3(-30.095f, 29.0794f, -15.0793f)); 
  data[5].mAabb = Aabb(Vector3(-39.7024f, 10.0575f, -2.62116f), Vector3(-35.3068f, 15.1865f, 4.62411f)); 
  data[6].mAabb = Aabb(Vector3(22.8016f, 13.6849f, 34.2318f), Vector3(23.5743f, 16.5567f, 35.129f)); 
  data[7].mAabb = Aabb(Vector3(-18.3539f, 5.14044f, 10.4714f), Vector3(-16.0645f, 9.15524f, 18.4649f)); 
  data[8].mAabb = Aabb(Vector3(-10.233f, -27.3362f, -22.8714f), Vector3(-7.03673f, -20.017f, -15.4425f)); 
  data[9].mAabb = Aabb(Vector3(-31.9231f, -4.49761f, -43.6638f), Vector3(-26.2957f, 2.33086f, -36.0005f)); 
  data[10].mAabb = Aabb(Vector3(3.45018f, 42.0029f, 3.1643f), Vector3(8.41385f, 49.2484f, 5.74292f)); 
  data[11].mAabb = Aabb(Vector3(-21.382f, -8.34772f, -43.4785f), Vector3(-13.7468f, -4.36656f, -36.5176f)); 
  data[12].mAabb = Aabb(Vector3(3.37676f, 28.3903f, 27.0216f), Vector3(4.56698f, 35.2853f, 28.0634f)); 
  data[13].mAabb = Aabb(Vector3(21.4351f, 8.81255f, 0.616655f), Vector3(28.986f, 10.6766f, 1.60741f)); 
  data[14].mAabb = Aabb(Vector3(35.0301f, -17.2849f, 1.67811f), Vector3(37.7736f, -12.8806f, 5.27286f)); 
  data[15].mAabb = Aabb(Vector3(-39.3097f, -13.9941f, 0.753323f), Vector3(-33.2947f, -12.4571f, 4.98591f)); 
  data[16].mAabb = Aabb(Vector3(-11.0863f, -23.6539f, -23.7576f), Vector3(-7.47605f, -22.3069f, -18.2435f)); 
  data[17].mAabb = Aabb(Vector3(1.69643f, -7.64777f, 47.1683f), Vector3(8.51886f, -3.73062f, 50.7981f)); 
  data[18].mAabb = Aabb(Vector3(-16.3758f, -40.4014f, -12.8241f), Vector3(-8.65992f, -39.2728f, -12.3856f)); 
  data[19].mAabb = Aabb(Vector3(-8.83755f, -2.44681f, 26.4833f), Vector3(-2.07983f, 2.67712f, 28.4867f)); 
  data[20].mAabb = Aabb(Vector3(9.55112f, -6.68003f, -41.6376f), Vector3(17.1545f, 0.717849f, -34.6505f)); 
  data[21].mAabb = Aabb(Vector3(5.33139f, 21.7197f, -36.069f), Vector3(9.9548f, 24.3652f, -29.8663f)); 
  data[22].mAabb = Aabb(Vector3(-37.6447f, 4.71346f, -33.5797f), Vector3(-36.457f, 11.7481f, -28.7357f)); 
  data[23].mAabb = Aabb(Vector3(12.357f, 20.1542f, -4.71412f), Vector3(15.9297f, 26.5436f, -4.11163f)); 
  data[24].mAabb = Aabb(Vector3(-12.3252f, 1.38945f, -9.2402f), Vector3(-4.92798f, 3.46731f, -4.01468f)); 
  data[25].mAabb = Aabb(Vector3(4.44885f, -28.1148f, 17.1954f), Vector3(11.9103f, -21.2365f, 20.7624f)); 
  data[26].mAabb = Aabb(Vector3(45.4757f, -3.02521f, -3.73263f), Vector3(52.7868f, 3.73182f, 1.90598f)); 
  data[27].mAabb = Aabb(Vector3(-12.3105f, -8.68464f, 33.6159f), Vector3(-11.7825f, -7.62918f, 37.8981f)); 
  data[28].mAabb = Aabb(Vector3(-40.4214f, -2.11475f, -6.89401f), Vector3(-33.2074f, 3.47028f, -6.24885f)); 
  data[29].mAabb = Aabb(Vector3(23.2057f, -5.44619f, -44.5881f), Vector3(25.5269f, -0.284915f, -37.8083f)); 
  data[30].mAabb = Aabb(Vector3(-28.3192f, 15.9732f, -27.5365f), Vector3(-22.5495f, 17.4559f, -23.7912f)); 
  data[31].mAabb = Aabb(Vector3(-6.32626f, -47.0174f, -17.3019f), Vector3(-1.54954f, -43.8761f, -15.9449f)); 
  data[32].mAabb = Aabb(Vector3(26.7065f, -18.385f, 17.4272f), Vector3(27.6409f, -12.8941f, 18.7157f)); 
  data[33].mAabb = Aabb(Vector3(14.5486f, -33.1955f, -31.6042f), Vector3(19.5164f, -29.9424f, -24.9935f)); 
  data[34].mAabb = Aabb(Vector3(-3.40022f, -23.6184f, -8.87565f), Vector3(-2.18077f, -17.785f, -2.54307f)); 
  data[35].mAabb = Aabb(Vector3(5.01335f, 23.322f, 29.6894f), Vector3(6.24579f, 30.2532f, 31.9828f)); 
  data[36].mAabb = Aabb(Vector3(-5.90511f, -31.3505f, 19.2066f), Vector3(-3.79269f, -27.8513f, 23.358f)); 
  data[37].mAabb = Aabb(Vector3(-33.2446f, -10.8102f, 4.41366f), Vector3(-26.6486f, -9.04015f, 7.02104f)); 
  data[38].mAabb = Aabb(Vector3(-10.6846f, -27.9763f, 20.8502f), Vector3(-4.40726f, -20.2789f, 23.7677f)); 
  data[39].mAabb = Aabb(Vector3(-4.15244f, 9.63251f, -8.11094f), Vector3(3.84478f, 12.7286f, -1.56265f)); 
  data[40].mAabb = Aabb(Vector3(38.012f, -24.5312f, -19.9711f), Vector3(40.7495f, -22.1769f, -15.9115f)); 
  data[41].mAabb = Aabb(Vector3(26.6121f, -13.4102f, -10.3799f), Vector3(27.8454f, -8.2294f, -3.87103f)); 
  data[42].mAabb = Aabb(Vector3(-11.8383f, -11.3701f, -44.3471f), Vector3(-5.3784f, -6.35677f, -37.3303f)); 
  data[43].mAabb = Aabb(Vector3(-5.76219f, -5.50915f, 15.1731f), Vector3(1.57469f, -2.14472f, 19.7313f)); 
  data[44].mAabb = Aabb(Vector3(-36.6778f, 22.3188f, 13.1412f), Vector3(-31.5274f, 23.6166f, 20.4368f)); 
  data[45].mAabb = Aabb(Vector3(38.8382f, -10.6805f, -2.37518f), Vector3(41.5298f, -9.32843f, 2.43331f)); 
  data[46].mAabb = Aabb(Vector3(-13.9338f, 9.61881f, 7.73033f), Vector3(-8.06046f, 15.122f, 11.4779f)); 
  data[47].mAabb = Aabb(Vector3(14.5808f, 4.38426f, 23.1129f), Vector3(17.5895f, 9.72529f, 30.3916f)); 
  data[48].mAabb = Aabb(Vector3(28.2257f, 16.7032f, 26.6726f), Vector3(33.7328f, 24.3806f, 31.0476f)); 
  data[49].mAabb = Aabb(Vector3(-25.1783f, 2.89919f, -3.08375f), Vector3(-19.7108f, 5.74408f, -1.01331f)); 
  data[50].mAabb = Aabb(Vector3(-25.4604f, 4.41351f, -1.60293f), Vector3(-23.8095f, 8.9678f, -0.622611f)); 
  data[51].mAabb = Aabb(Vector3(35.7857f, 18.211f, -22.8403f), Vector3(42.1234f, 22.0892f, -18.3684f)); 
  data[52].mAabb = Aabb(Vector3(32.6215f, 6.36113f, 17.9843f), Vector3(37.4128f, 10.2644f, 18.7572f)); 
  data[53].mAabb = Aabb(Vector3(27.1364f, 6.36585f, 14.8288f), Vector3(30.7314f, 11.9244f, 15.4206f)); 
  data[54].mAabb = Aabb(Vector3(-28.5929f, 28.1701f, -27.0095f), Vector3(-27.4491f, 30.2229f, -25.3696f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest28(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(56); 
  data[0].mAabb = Aabb(Vector3(23.9853f, -22.5211f, -3.22521f), Vector3(27.7338f, -18.0104f, 3.9572f)); 
  data[1].mAabb = Aabb(Vector3(-12.601f, 21.267f, -23.6181f), Vector3(-5.79942f, 24.9807f, -17.4672f)); 
  data[2].mAabb = Aabb(Vector3(-21.6222f, -24.3639f, 24.4098f), Vector3(-14.0047f, -23.5225f, 25.838f)); 
  data[3].mAabb = Aabb(Vector3(24.3226f, 17.2929f, 12.1987f), Vector3(27.8791f, 21.0607f, 19.2505f)); 
  data[4].mAabb = Aabb(Vector3(-6.43253f, -4.75609f, -49.1138f), Vector3(-0.478476f, 2.67983f, -41.2418f)); 
  data[5].mAabb = Aabb(Vector3(22.2178f, 11.642f, 11.5254f), Vector3(24.0526f, 16.0623f, 16.6644f)); 
  data[6].mAabb = Aabb(Vector3(-27.8395f, 1.82545f, 35.3178f), Vector3(-26.1362f, 4.38064f, 40.3546f)); 
  data[7].mAabb = Aabb(Vector3(-7.07004f, -0.790256f, 11.6759f), Vector3(-3.00259f, 0.152022f, 14.7813f)); 
  data[8].mAabb = Aabb(Vector3(-4.36474f, 0.497714f, -44.4219f), Vector3(-3.85596f, 7.51173f, -40.9742f)); 
  data[9].mAabb = Aabb(Vector3(-8.54253f, 23.3859f, -20.8895f), Vector3(-6.45562f, 29.3599f, -18.4649f)); 
  data[10].mAabb = Aabb(Vector3(14.6288f, 24.3972f, 27.911f), Vector3(17.2607f, 29.2651f, 29.3265f)); 
  data[11].mAabb = Aabb(Vector3(0.483692f, 43.0853f, 13.8521f), Vector3(2.25353f, 44.3263f, 16.6949f)); 
  data[12].mAabb = Aabb(Vector3(-4.27638f, -16.3808f, -14.0371f), Vector3(-0.778585f, -10.9505f, -10.7995f)); 
  data[13].mAabb = Aabb(Vector3(-38.6008f, -7.15288f, -20.2272f), Vector3(-34.6632f, -1.84084f, -16.86f)); 
  data[14].mAabb = Aabb(Vector3(-20.4865f, -0.281029f, 22.6044f), Vector3(-18.811f, 6.69704f, 28.1331f)); 
  data[15].mAabb = Aabb(Vector3(-16.1193f, 36.9689f, -23.0506f), Vector3(-13.0983f, 41.413f, -19.4816f)); 
  data[16].mAabb = Aabb(Vector3(27.6269f, -22.1985f, -13.4131f), Vector3(29.3358f, -15.8879f, -7.81622f)); 
  data[17].mAabb = Aabb(Vector3(10.2886f, 30.5614f, 31.3272f), Vector3(10.6976f, 37.1554f, 39.1359f)); 
  data[18].mAabb = Aabb(Vector3(-14.198f, -23.2753f, -46.3546f), Vector3(-11.1887f, -18.7532f, -40.3811f)); 
  data[19].mAabb = Aabb(Vector3(0.907643f, 27.7463f, -14.7285f), Vector3(3.6971f, 31.3914f, -13.9309f)); 
  data[20].mAabb = Aabb(Vector3(-37.6356f, -0.680251f, 3.20797f), Vector3(-29.8138f, 0.473324f, 3.81834f)); 
  data[21].mAabb = Aabb(Vector3(34.6451f, 7.70631f, -16.5355f), Vector3(37.0382f, 10.8281f, -13.0233f)); 
  data[22].mAabb = Aabb(Vector3(3.31347f, -33.7711f, -15.0895f), Vector3(4.79083f, -30.6882f, -14.6301f)); 
  data[23].mAabb = Aabb(Vector3(25.2749f, 12.5001f, -21.5222f), Vector3(25.9332f, 18.1473f, -18.4572f)); 
  data[24].mAabb = Aabb(Vector3(-12.2003f, -20.9272f, -7.06908f), Vector3(-5.85289f, -18.6111f, 0.87479f)); 
  data[25].mAabb = Aabb(Vector3(-47.0927f, -7.78443f, -19.0111f), Vector3(-43.2662f, -1.97348f, -16.6579f)); 
  data[26].mAabb = Aabb(Vector3(18.4195f, 5.17211f, -11.088f), Vector3(21.5251f, 7.41951f, -5.45405f)); 
  data[27].mAabb = Aabb(Vector3(-37.5891f, -2.37079f, -18.2438f), Vector3(-31.2129f, 4.33707f, -12.3503f)); 
  data[28].mAabb = Aabb(Vector3(-5.6936f, -43.6616f, -3.78054f), Vector3(-2.57619f, -42.7194f, 1.66394f)); 
  data[29].mAabb = Aabb(Vector3(-27.8069f, -11.0227f, -18.5898f), Vector3(-22.9882f, -6.81282f, -13.3341f)); 
  data[30].mAabb = Aabb(Vector3(-36.2041f, -8.44835f, -33.1777f), Vector3(-31.4388f, -5.03405f, -29.9195f)); 
  data[31].mAabb = Aabb(Vector3(22.1607f, -16.8228f, 8.41648f), Vector3(26.7178f, -9.58816f, 9.32351f)); 
  data[32].mAabb = Aabb(Vector3(-10.4284f, -17.0966f, 15.8199f), Vector3(-6.90604f, -15.2469f, 22.7147f)); 
  data[33].mAabb = Aabb(Vector3(-10.4492f, 39.6983f, -21.5991f), Vector3(-8.92155f, 44.1693f, -19.3157f)); 
  data[34].mAabb = Aabb(Vector3(-5.66041f, -20.9856f, -14.4883f), Vector3(-4.27164f, -17.6335f, -9.35343f)); 
  data[35].mAabb = Aabb(Vector3(20.8499f, -7.30277f, -16.5386f), Vector3(28.5843f, -2.67356f, -11.6446f)); 
  data[36].mAabb = Aabb(Vector3(30.4718f, -7.43245f, 13.6269f), Vector3(33.5732f, -1.27244f, 21.3386f)); 
  data[37].mAabb = Aabb(Vector3(12.4172f, -6.68549f, -20.0324f), Vector3(16.0046f, 0.784994f, -12.1542f)); 
  data[38].mAabb = Aabb(Vector3(-15.5911f, 16.9348f, -33.1381f), Vector3(-10.0592f, 20.8415f, -31.9544f)); 
  data[39].mAabb = Aabb(Vector3(13.5828f, -22.9592f, -6.07682f), Vector3(21.3442f, -20.6745f, -3.65523f)); 
  data[40].mAabb = Aabb(Vector3(-42.9375f, 2.25834f, -10.6027f), Vector3(-37.2564f, 4.74071f, -6.80312f)); 
  data[41].mAabb = Aabb(Vector3(37.1123f, -2.61054f, -18.6915f), Vector3(40.195f, 0.467916f, -16.6431f)); 
  data[42].mAabb = Aabb(Vector3(-35.5921f, 21.2475f, 17.7998f), Vector3(-32.3036f, 28.707f, 21.2688f)); 
  data[43].mAabb = Aabb(Vector3(23.3639f, 0.314162f, 34.3818f), Vector3(31.3257f, 4.64393f, 41.5816f)); 
  data[44].mAabb = Aabb(Vector3(-22.0667f, 21.9185f, 32.4252f), Vector3(-21.4025f, 26.5655f, 39.9771f)); 
  data[45].mAabb = Aabb(Vector3(14.8689f, 5.13371f, -31.7369f), Vector3(16.3463f, 8.51415f, -26.2272f)); 
  data[46].mAabb = Aabb(Vector3(25.2452f, 16.6174f, 2.45741f), Vector3(31.4015f, 22.352f, 6.23331f)); 
  data[47].mAabb = Aabb(Vector3(3.61533f, -33.3373f, 14.5989f), Vector3(10.4484f, -29.6141f, 21.1948f)); 
  data[48].mAabb = Aabb(Vector3(-3.70332f, 37.5955f, -25.7457f), Vector3(2.16654f, 44.3827f, -17.9445f)); 
  data[49].mAabb = Aabb(Vector3(-29.3909f, 9.24734f, -42.3712f), Vector3(-26.1005f, 10.0938f, -34.7994f)); 
  data[50].mAabb = Aabb(Vector3(22.2385f, 13.6407f, -27.582f), Vector3(23.4854f, 21.1982f, -23.2462f)); 
  data[51].mAabb = Aabb(Vector3(-7.7561f, 42.7289f, 4.44001f), Vector3(-6.0584f, 44.43f, 6.53781f)); 
  data[52].mAabb = Aabb(Vector3(-39.9048f, -0.938292f, 10.1093f), Vector3(-38.7842f, 6.83394f, 13.0319f)); 
  data[53].mAabb = Aabb(Vector3(-18.1102f, 11.2479f, 0.777541f), Vector3(-11.1876f, 14.6995f, 5.95714f)); 
  data[54].mAabb = Aabb(Vector3(10.6301f, -23.1927f, -11.8815f), Vector3(15.6847f, -16.4308f, -4.17673f)); 
  data[55].mAabb = Aabb(Vector3(-6.85427f, 7.73608f, 2.4275f), Vector3(-2.15062f, 13.107f, 3.75666f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest29(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(28); 
  data[0].mAabb = Aabb(Vector3(-3.96097f, 47.2677f, -11.2278f), Vector3(-3.27661f, 49.8948f, -10.1964f)); 
  data[1].mAabb = Aabb(Vector3(-0.263345f, -10.1079f, 23.8564f), Vector3(0.837348f, -4.759f, 24.5148f)); 
  data[2].mAabb = Aabb(Vector3(3.04902f, -0.888619f, 35.1354f), Vector3(6.83768f, 1.2841f, 39.2627f)); 
  data[3].mAabb = Aabb(Vector3(10.3168f, -11.6693f, -8.42706f), Vector3(17.5292f, -7.98685f, -5.34699f)); 
  data[4].mAabb = Aabb(Vector3(34.106f, -11.3846f, -27.287f), Vector3(41.756f, -9.13303f, -19.3902f)); 
  data[5].mAabb = Aabb(Vector3(-28.6017f, -24.3886f, 21.5633f), Vector3(-21.7354f, -20.2957f, 28.3078f)); 
  data[6].mAabb = Aabb(Vector3(-21.8409f, -3.61924f, 21.3601f), Vector3(-14.6202f, -2.22607f, 23.3533f)); 
  data[7].mAabb = Aabb(Vector3(20.4174f, -10.0608f, 9.29064f), Vector3(24.7003f, -5.65012f, 14.98f)); 
  data[8].mAabb = Aabb(Vector3(12.3967f, -31.5826f, 22.7435f), Vector3(12.9249f, -23.86f, 27.5602f)); 
  data[9].mAabb = Aabb(Vector3(-17.3333f, -16.1736f, 8.82977f), Vector3(-12.4758f, -14.0862f, 9.6751f)); 
  data[10].mAabb = Aabb(Vector3(28.8404f, 12.924f, 17.4171f), Vector3(29.5275f, 18.7013f, 19.9324f)); 
  data[11].mAabb = Aabb(Vector3(13.5783f, -13.7663f, -21.0611f), Vector3(16.532f, -10.4996f, -18.7821f)); 
  data[12].mAabb = Aabb(Vector3(16.6037f, -30.5331f, 2.28793f), Vector3(17.6812f, -27.5144f, 8.38788f)); 
  data[13].mAabb = Aabb(Vector3(-11.4474f, -34.1728f, 10.7416f), Vector3(-5.10023f, -28.6692f, 14.8215f)); 
  data[14].mAabb = Aabb(Vector3(-0.440103f, 25.3219f, -3.4651f), Vector3(4.16498f, 31.0243f, -2.1828f)); 
  data[15].mAabb = Aabb(Vector3(37.7386f, 27.4624f, -1.07292f), Vector3(42.6682f, 30.7637f, 3.73094f)); 
  data[16].mAabb = Aabb(Vector3(-25.8952f, -14.5994f, 8.57483f), Vector3(-22.1409f, -9.02921f, 11.2256f)); 
  data[17].mAabb = Aabb(Vector3(10.6511f, -40.5429f, -23.5996f), Vector3(18.5388f, -39.519f, -17.8165f)); 
  data[18].mAabb = Aabb(Vector3(-32.1527f, 30.3312f, -1.96992f), Vector3(-25.0058f, 32.1565f, 0.0232757f)); 
  data[19].mAabb = Aabb(Vector3(43.7492f, -4.09816f, 11.0947f), Vector3(47.5188f, 2.25158f, 18.1119f)); 
  data[20].mAabb = Aabb(Vector3(10.4797f, 14.0404f, 39.0408f), Vector3(17.397f, 17.0943f, 46.225f)); 
  data[21].mAabb = Aabb(Vector3(-2.7498f, 15.5571f, -23.0946f), Vector3(2.89972f, 19.5823f, -21.7367f)); 
  data[22].mAabb = Aabb(Vector3(10.6176f, 14.9422f, 11.1087f), Vector3(16.037f, 18.0965f, 18.1536f)); 
  data[23].mAabb = Aabb(Vector3(-20.7325f, -3.39262f, -29.768f), Vector3(-15.329f, 1.29504f, -27.8702f)); 
  data[24].mAabb = Aabb(Vector3(-1.33908f, 10.3983f, -0.530308f), Vector3(1.83771f, 15.5371f, 4.85874f)); 
  data[25].mAabb = Aabb(Vector3(-3.9562f, 6.42979f, -30.1798f), Vector3(2.95531f, 7.74016f, -25.7016f)); 
  data[26].mAabb = Aabb(Vector3(4.8337f, 27.1194f, -32.6191f), Vector3(10.6219f, 30.3871f, -27.3479f)); 
  data[27].mAabb = Aabb(Vector3(-36.0755f, -32.6467f, -2.58451f), Vector3(-28.2814f, -28.6055f, 1.24218f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest30(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(41); 
  data[0].mAabb = Aabb(Vector3(37.5098f, 27.6326f, 5.50556f), Vector3(40.552f, 31.1861f, 12.7782f)); 
  data[1].mAabb = Aabb(Vector3(-8.01335f, -7.24666f, -20.6024f), Vector3(-4.77509f, -2.57315f, -17.6054f)); 
  data[2].mAabb = Aabb(Vector3(18.7957f, -28.0744f, 16.2262f), Vector3(21.2288f, -24.6072f, 23.485f)); 
  data[3].mAabb = Aabb(Vector3(-6.89395f, -14.3471f, -41.5823f), Vector3(0.21864f, -7.63116f, -37.4494f)); 
  data[4].mAabb = Aabb(Vector3(1.83805f, -22.3006f, 10.4592f), Vector3(3.71134f, -17.1802f, 11.8141f)); 
  data[5].mAabb = Aabb(Vector3(-30.9322f, -16.2244f, -33.4058f), Vector3(-26.2761f, -10.3256f, -26.9195f)); 
  data[6].mAabb = Aabb(Vector3(42.8285f, -3.69206f, 18.9626f), Vector3(46.0863f, 2.91629f, 21.8945f)); 
  data[7].mAabb = Aabb(Vector3(17.4437f, 14.0566f, 33.0456f), Vector3(19.9131f, 17.152f, 35.4635f)); 
  data[8].mAabb = Aabb(Vector3(10.9398f, 27.6972f, -24.725f), Vector3(13.9179f, 31.532f, -20.2829f)); 
  data[9].mAabb = Aabb(Vector3(-32.7814f, -9.21077f, 27.8831f), Vector3(-24.8577f, -4.92252f, 35.3761f)); 
  data[10].mAabb = Aabb(Vector3(25.7829f, -39.8071f, 5.1488f), Vector3(31.6084f, -34.9564f, 7.00609f)); 
  data[11].mAabb = Aabb(Vector3(-24.2425f, -45.3523f, -8.48349f), Vector3(-18.3989f, -43.1826f, -7.75135f)); 
  data[12].mAabb = Aabb(Vector3(2.93309f, -36.6863f, 27.5365f), Vector3(10.3154f, -34.7546f, 29.0211f)); 
  data[13].mAabb = Aabb(Vector3(21.18f, -24.0075f, 5.36451f), Vector3(26.3895f, -17.9192f, 6.38704f)); 
  data[14].mAabb = Aabb(Vector3(-9.02562f, -18.5522f, 41.338f), Vector3(-8.31435f, -10.9932f, 43.6121f)); 
  data[15].mAabb = Aabb(Vector3(12.4692f, 0.768355f, 11.0257f), Vector3(13.2577f, 1.1716f, 12.1273f)); 
  data[16].mAabb = Aabb(Vector3(8.03354f, -42.0827f, 6.84217f), Vector3(14.1908f, -36.2189f, 10.5766f)); 
  data[17].mAabb = Aabb(Vector3(-32.6399f, 14.7002f, -35.4184f), Vector3(-30.2304f, 18.612f, -34.2877f)); 
  data[18].mAabb = Aabb(Vector3(21.0052f, -23.494f, -10.8482f), Vector3(26.2779f, -21.794f, -8.68338f)); 
  data[19].mAabb = Aabb(Vector3(-48.359f, -1.37695f, -14.6033f), Vector3(-41.1657f, 2.63019f, -6.93846f)); 
  data[20].mAabb = Aabb(Vector3(14.8583f, -27.5415f, -5.2943f), Vector3(20.5396f, -23.8343f, -2.77505f)); 
  data[21].mAabb = Aabb(Vector3(-11.4728f, -10.7686f, -46.8711f), Vector3(-4.82342f, -4.13357f, -42.4353f)); 
  data[22].mAabb = Aabb(Vector3(0.271727f, -36.2324f, 25.2382f), Vector3(4.16058f, -34.3078f, 26.2482f)); 
  data[23].mAabb = Aabb(Vector3(24.7358f, 11.605f, -13.7564f), Vector3(27.493f, 17.7128f, -9.79561f)); 
  data[24].mAabb = Aabb(Vector3(-10.6382f, -8.57705f, 45.7884f), Vector3(-6.18136f, -4.25632f, 49.5599f)); 
  data[25].mAabb = Aabb(Vector3(26.7229f, -13.5932f, -25.1341f), Vector3(30.0579f, -11.7529f, -21.1504f)); 
  data[26].mAabb = Aabb(Vector3(3.16209f, 7.14043f, -6.66964f), Vector3(10.1126f, 9.94125f, -0.382056f)); 
  data[27].mAabb = Aabb(Vector3(6.50932f, 11.4327f, 1.58209f), Vector3(7.7276f, 14.3015f, 4.95232f)); 
  data[28].mAabb = Aabb(Vector3(-36.9325f, -25.1202f, -0.189533f), Vector3(-31.86f, -23.6672f, 6.31329f)); 
  data[29].mAabb = Aabb(Vector3(-9.339f, 23.2153f, 13.4496f), Vector3(-2.98416f, 26.03f, 21.3111f)); 
  data[30].mAabb = Aabb(Vector3(-7.68098f, 15.9931f, -25.3928f), Vector3(-7.08383f, 23.9908f, -18.6685f)); 
  data[31].mAabb = Aabb(Vector3(-24.0248f, -33.2453f, -21.8894f), Vector3(-17.8956f, -30.2851f, -16.3863f)); 
  data[32].mAabb = Aabb(Vector3(16.8897f, 22.1132f, -14.2693f), Vector3(17.4064f, 29.3139f, -13.0383f)); 
  data[33].mAabb = Aabb(Vector3(-8.40406f, -10.1519f, -30.6278f), Vector3(-1.77251f, -4.86029f, -28.8663f)); 
  data[34].mAabb = Aabb(Vector3(13.1291f, -8.8497f, -41.7808f), Vector3(20.738f, -1.63366f, -38.2816f)); 
  data[35].mAabb = Aabb(Vector3(24.0575f, 11.4663f, -12.869f), Vector3(31.4399f, 19.0456f, -9.9309f)); 
  data[36].mAabb = Aabb(Vector3(-39.1096f, 8.21986f, -8.48653f), Vector3(-35.8653f, 11.0353f, -4.92796f)); 
  data[37].mAabb = Aabb(Vector3(1.65491f, -27.6161f, -29.8915f), Vector3(3.07754f, -21.6667f, -22.395f)); 
  data[38].mAabb = Aabb(Vector3(-5.59888f, 3.69242f, 15.643f), Vector3(-4.55664f, 10.9657f, 16.2362f)); 
  data[39].mAabb = Aabb(Vector3(32.9228f, -32.0523f, -17.7023f), Vector3(38.2112f, -25.4321f, -9.79047f)); 
  data[40].mAabb = Aabb(Vector3(-6.51242f, -39.6803f, 3.64462f), Vector3(-3.52489f, -35.2169f, 4.94084f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest31(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(62); 
  data[0].mAabb = Aabb(Vector3(-16.8083f, 30.1975f, 11.257f), Vector3(-14.9473f, 34.1812f, 16.5746f)); 
  data[1].mAabb = Aabb(Vector3(10.225f, -14.4297f, 1.23372f), Vector3(17.7173f, -9.82921f, 6.96604f)); 
  data[2].mAabb = Aabb(Vector3(-5.79866f, 20.3944f, -27.134f), Vector3(1.95873f, 27.9036f, -25.5712f)); 
  data[3].mAabb = Aabb(Vector3(-16.7799f, -6.16668f, 2.15939f), Vector3(-11.6381f, -1.79772f, 6.96766f)); 
  data[4].mAabb = Aabb(Vector3(27.7282f, 4.31862f, 26.4224f), Vector3(35.1773f, 9.24494f, 30.5057f)); 
  data[5].mAabb = Aabb(Vector3(30.5493f, 1.98948f, -14.9944f), Vector3(31.5864f, 3.12751f, -12.2409f)); 
  data[6].mAabb = Aabb(Vector3(17.8431f, -8.18089f, -17.6966f), Vector3(18.9366f, -0.916147f, -15.406f)); 
  data[7].mAabb = Aabb(Vector3(23.2269f, 21.4885f, 25.0244f), Vector3(28.5322f, 24.859f, 30.8775f)); 
  data[8].mAabb = Aabb(Vector3(10.1971f, 14.3367f, -29.6434f), Vector3(16.0124f, 21.5846f, -26.1201f)); 
  data[9].mAabb = Aabb(Vector3(-0.306425f, 14.9432f, -36.7426f), Vector3(5.72046f, 22.5313f, -35.0943f)); 
  data[10].mAabb = Aabb(Vector3(20.1657f, -29.5137f, -34.3796f), Vector3(20.9308f, -26.9471f, -26.5044f)); 
  data[11].mAabb = Aabb(Vector3(21.788f, -45.5925f, -4.77797f), Vector3(23.0749f, -41.721f, -2.48881f)); 
  data[12].mAabb = Aabb(Vector3(-23.2839f, -9.17599f, -1.54011f), Vector3(-15.8934f, -8.05605f, 5.18839f)); 
  data[13].mAabb = Aabb(Vector3(5.84972f, -17.6253f, -46.7786f), Vector3(11.6187f, -13.0963f, -43.6025f)); 
  data[14].mAabb = Aabb(Vector3(24.1592f, 17.8371f, 28.0056f), Vector3(27.689f, 25.8167f, 33.7108f)); 
  data[15].mAabb = Aabb(Vector3(8.33392f, 29.8451f, -19.1167f), Vector3(14.3364f, 36.2463f, -15.2176f)); 
  data[16].mAabb = Aabb(Vector3(-22.6447f, 27.3061f, 10.7632f), Vector3(-18.1001f, 32.5521f, 11.3111f)); 
  data[17].mAabb = Aabb(Vector3(-12.0298f, -44.8145f, -12.2696f), Vector3(-10.2486f, -38.5559f, -5.57964f)); 
  data[18].mAabb = Aabb(Vector3(10.9227f, -41.2437f, -6.5297f), Vector3(13.4215f, -40.6851f, -1.08291f)); 
  data[19].mAabb = Aabb(Vector3(-46.5369f, 2.0631f, 5.84348f), Vector3(-45.0772f, 4.62432f, 8.44111f)); 
  data[20].mAabb = Aabb(Vector3(8.77255f, -10.0072f, -44.0158f), Vector3(14.1748f, -6.58987f, -42.4997f)); 
  data[21].mAabb = Aabb(Vector3(24.6743f, 34.2516f, 19.6785f), Vector3(26.9183f, 38.4891f, 24.6802f)); 
  data[22].mAabb = Aabb(Vector3(16.963f, 6.33235f, 15.7858f), Vector3(21.2889f, 11.4157f, 19.5315f)); 
  data[23].mAabb = Aabb(Vector3(24.1136f, 15.3249f, -1.93534f), Vector3(29.7003f, 17.8711f, 1.93727f)); 
  data[24].mAabb = Aabb(Vector3(-16.0274f, 37.7969f, 5.35865f), Vector3(-12.0219f, 41.4181f, 13.0866f)); 
  data[25].mAabb = Aabb(Vector3(0.523749f, -8.20004f, 43.6424f), Vector3(8.12504f, -5.51403f, 50.4389f)); 
  data[26].mAabb = Aabb(Vector3(-32.1819f, -19.2689f, 18.4316f), Vector3(-26.637f, -13.8297f, 25.3104f)); 
  data[27].mAabb = Aabb(Vector3(30.2293f, 8.60428f, 33.1869f), Vector3(37.2354f, 11.4174f, 35.3587f)); 
  data[28].mAabb = Aabb(Vector3(-8.03316f, -46.2958f, -0.714904f), Vector3(-7.20476f, -39.3966f, 0.350998f)); 
  data[29].mAabb = Aabb(Vector3(27.3392f, -16.3294f, -20.1514f), Vector3(28.2508f, -14.4002f, -17.1562f)); 
  data[30].mAabb = Aabb(Vector3(34.6192f, -8.73952f, -13.2649f), Vector3(38.6801f, -7.38903f, -8.31077f)); 
  data[31].mAabb = Aabb(Vector3(-3.96851f, -18.186f, -2.53251f), Vector3(-0.605237f, -10.2824f, 4.11087f)); 
  data[32].mAabb = Aabb(Vector3(27.0733f, -9.30929f, -6.12059f), Vector3(32.3148f, -1.75415f, 1.25804f)); 
  data[33].mAabb = Aabb(Vector3(3.60421f, 32.8202f, 29.4966f), Vector3(7.35552f, 38.3113f, 35.7665f)); 
  data[34].mAabb = Aabb(Vector3(-47.4094f, 9.73456f, 12.3934f), Vector3(-39.621f, 14.9988f, 16.4583f)); 
  data[35].mAabb = Aabb(Vector3(-45.2821f, -25.7417f, 9.88135f), Vector3(-38.8797f, -18.1309f, 13.1055f)); 
  data[36].mAabb = Aabb(Vector3(0.886348f, 9.54834f, -39.4288f), Vector3(5.61111f, 11.4599f, -37.6254f)); 
  data[37].mAabb = Aabb(Vector3(-29.2253f, -25.5896f, -16.0347f), Vector3(-21.5614f, -20.2481f, -11.9271f)); 
  data[38].mAabb = Aabb(Vector3(-9.26084f, -10.3773f, -14.838f), Vector3(-6.82162f, -8.19557f, -12.4487f)); 
  data[39].mAabb = Aabb(Vector3(21.6872f, 7.0392f, -42.6407f), Vector3(27.8796f, 7.74328f, -37.5615f)); 
  data[40].mAabb = Aabb(Vector3(-11.4682f, -35.35f, -26.2757f), Vector3(-4.23525f, -29.7304f, -25.0289f)); 
  data[41].mAabb = Aabb(Vector3(-23.8397f, 5.63211f, 39.1981f), Vector3(-22.2767f, 11.7221f, 47.0219f)); 
  data[42].mAabb = Aabb(Vector3(-17.9263f, 34.4067f, 20.7522f), Vector3(-10.2622f, 36.049f, 22.195f)); 
  data[43].mAabb = Aabb(Vector3(-15.5706f, -12.876f, -30.4876f), Vector3(-14.6654f, -8.48015f, -29.0967f)); 
  data[44].mAabb = Aabb(Vector3(18.6482f, -23.0732f, 29.8754f), Vector3(23.3565f, -19.5583f, 31.8842f)); 
  data[45].mAabb = Aabb(Vector3(18.9839f, 20.8547f, 21.7137f), Vector3(22.8685f, 25.387f, 29.2406f)); 
  data[46].mAabb = Aabb(Vector3(-33.4262f, -3.08041f, 3.03122f), Vector3(-32.2872f, 3.97513f, 9.91048f)); 
  data[47].mAabb = Aabb(Vector3(17.2623f, -21.3261f, 38.5559f), Vector3(18.2022f, -17.3628f, 40.6537f)); 
  data[48].mAabb = Aabb(Vector3(8.0102f, 22.1069f, 9.48769f), Vector3(10.1565f, 28.9337f, 15.2478f)); 
  data[49].mAabb = Aabb(Vector3(29.3757f, -18.5422f, 9.38656f), Vector3(30.0624f, -17.9975f, 17.2193f)); 
  data[50].mAabb = Aabb(Vector3(-32.6681f, -11.0214f, -25.0604f), Vector3(-25.6619f, -9.83746f, -21.4179f)); 
  data[51].mAabb = Aabb(Vector3(36.2097f, -16.1877f, 10.7669f), Vector3(43.612f, -14.0688f, 14.5505f)); 
  data[52].mAabb = Aabb(Vector3(-33.6466f, -3.57028f, 32.7022f), Vector3(-27.8382f, 1.36462f, 37.3708f)); 
  data[53].mAabb = Aabb(Vector3(-21.5032f, -25.2667f, -23.477f), Vector3(-19.0006f, -23.936f, -15.9492f)); 
  data[54].mAabb = Aabb(Vector3(-39.6054f, 19.2243f, -8.48386f), Vector3(-32.9433f, 20.2016f, -4.21532f)); 
  data[55].mAabb = Aabb(Vector3(0.556487f, -40.3003f, -30.8175f), Vector3(3.48394f, -39.3705f, -24.9015f)); 
  data[56].mAabb = Aabb(Vector3(-2.3264f, 26.4599f, -25.8052f), Vector3(-1.34075f, 28.1379f, -24.1293f)); 
  data[57].mAabb = Aabb(Vector3(2.44877f, -37.5167f, -27.9962f), Vector3(9.58943f, -33.702f, -21.2123f)); 
  data[58].mAabb = Aabb(Vector3(5.65706f, -4.03192f, -27.3914f), Vector3(11.9544f, 2.92109f, -24.2548f)); 
  data[59].mAabb = Aabb(Vector3(-22.4028f, -27.8262f, -11.596f), Vector3(-21.9421f, -20.3369f, -10.1595f)); 
  data[60].mAabb = Aabb(Vector3(15.7373f, -5.86646f, 19.1164f), Vector3(18.6525f, 1.14524f, 24.2406f)); 
  data[61].mAabb = Aabb(Vector3(-9.77104f, 0.969118f, 43.518f), Vector3(-4.53323f, 1.75669f, 47.4398f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest32(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(51); 
  data[0].mAabb = Aabb(Vector3(-12.8871f, 16.1063f, -12.5843f), Vector3(-9.01614f, 22.0309f, -5.06395f)); 
  data[1].mAabb = Aabb(Vector3(5.4477f, -35.3803f, 11.8671f), Vector3(7.947f, -32.5528f, 15.1559f)); 
  data[2].mAabb = Aabb(Vector3(-18.9295f, 23.5364f, 12.7665f), Vector3(-13.5972f, 24.9048f, 19.5532f)); 
  data[3].mAabb = Aabb(Vector3(17.8604f, -25.3785f, 11.2874f), Vector3(22.3861f, -18.0567f, 15.375f)); 
  data[4].mAabb = Aabb(Vector3(-2.78284f, -14.8416f, 24.049f), Vector3(3.10882f, -12.9121f, 25.804f)); 
  data[5].mAabb = Aabb(Vector3(-21.3668f, -16.756f, 17.6734f), Vector3(-18.8072f, -15.8177f, 18.5433f)); 
  data[6].mAabb = Aabb(Vector3(-7.97805f, -35.3385f, 22.1884f), Vector3(-6.45545f, -33.5712f, 28.8951f)); 
  data[7].mAabb = Aabb(Vector3(7.37569f, -38.2747f, 9.18545f), Vector3(12.2828f, -33.2797f, 16.7661f)); 
  data[8].mAabb = Aabb(Vector3(-39.7511f, 0.044421f, 30.2437f), Vector3(-36.4335f, 4.6509f, 32.3065f)); 
  data[9].mAabb = Aabb(Vector3(-34.6199f, -15.6347f, 18.996f), Vector3(-30.6065f, -9.59859f, 20.9356f)); 
  data[10].mAabb = Aabb(Vector3(13.5438f, -14.9068f, 10.9929f), Vector3(16.6415f, -14.351f, 13.9674f)); 
  data[11].mAabb = Aabb(Vector3(11.6441f, -23.6932f, -37.1267f), Vector3(14.7627f, -15.7725f, -33.2892f)); 
  data[12].mAabb = Aabb(Vector3(16.4565f, -23.9006f, -24.4048f), Vector3(20.273f, -16.9478f, -20.158f)); 
  data[13].mAabb = Aabb(Vector3(10.9249f, 20.9826f, 36.3379f), Vector3(15.9111f, 28.6655f, 38.7312f)); 
  data[14].mAabb = Aabb(Vector3(12.2862f, -18.2588f, 24.0448f), Vector3(17.5528f, -10.5745f, 25.8574f)); 
  data[15].mAabb = Aabb(Vector3(-45.2337f, 1.46317f, 24.9741f), Vector3(-38.1151f, 6.60102f, 27.1719f)); 
  data[16].mAabb = Aabb(Vector3(-39.1798f, -31.2079f, -18.5222f), Vector3(-32.5721f, -29.5456f, -12.5524f)); 
  data[17].mAabb = Aabb(Vector3(-3.18231f, 10.7455f, -44.6376f), Vector3(2.8898f, 18.7304f, -37.5918f)); 
  data[18].mAabb = Aabb(Vector3(35.8406f, -10.8886f, -25.2665f), Vector3(43.2622f, -8.38703f, -21.8758f)); 
  data[19].mAabb = Aabb(Vector3(-45.2997f, -17.5611f, 4.61238f), Vector3(-43.2153f, -11.9828f, 11.089f)); 
  data[20].mAabb = Aabb(Vector3(26.7212f, -8.34143f, -13.857f), Vector3(27.134f, -5.8702f, -10.3377f)); 
  data[21].mAabb = Aabb(Vector3(-33.3704f, 31.3088f, 2.09402f), Vector3(-26.7467f, 33.6337f, 5.92489f)); 
  data[22].mAabb = Aabb(Vector3(4.10638f, 34.1242f, -32.0248f), Vector3(9.97067f, 35.3035f, -27.2578f)); 
  data[23].mAabb = Aabb(Vector3(0.306653f, -11.9634f, 33.8821f), Vector3(6.38433f, -4.09145f, 40.622f)); 
  data[24].mAabb = Aabb(Vector3(7.7687f, -15.0812f, -39.0106f), Vector3(11.9271f, -9.42797f, -38.0899f)); 
  data[25].mAabb = Aabb(Vector3(-27.1027f, 0.410602f, -14.5611f), Vector3(-21.3131f, 7.0563f, -11.0067f)); 
  data[26].mAabb = Aabb(Vector3(-29.4323f, 31.7962f, -18.4872f), Vector3(-22.9494f, 35.2755f, -10.9116f)); 
  data[27].mAabb = Aabb(Vector3(-15.3847f, -22.387f, -40.3128f), Vector3(-14.1279f, -20.8807f, -34.08f)); 
  data[28].mAabb = Aabb(Vector3(-21.7678f, 40.6247f, -11.5001f), Vector3(-15.7418f, 46.2881f, -10.498f)); 
  data[29].mAabb = Aabb(Vector3(-15.0553f, -22.5788f, 25.6657f), Vector3(-14.351f, -20.4977f, 26.4605f)); 
  data[30].mAabb = Aabb(Vector3(-27.3249f, -20.6693f, -9.08855f), Vector3(-23.1462f, -19.3944f, -1.42231f)); 
  data[31].mAabb = Aabb(Vector3(29.2506f, -31.386f, -11.5053f), Vector3(35.8935f, -24.4516f, -4.04919f)); 
  data[32].mAabb = Aabb(Vector3(8.64576f, -16.4523f, 1.3732f), Vector3(14.7007f, -12.3162f, 2.58058f)); 
  data[33].mAabb = Aabb(Vector3(27.2335f, -2.98595f, 21.6856f), Vector3(31.8616f, -0.843843f, 29.6413f)); 
  data[34].mAabb = Aabb(Vector3(-0.989341f, -44.7116f, -3.85218f), Vector3(4.05272f, -41.4903f, 0.841501f)); 
  data[35].mAabb = Aabb(Vector3(-12.203f, -23.0742f, -18.0998f), Vector3(-8.57158f, -20.4429f, -15.1092f)); 
  data[36].mAabb = Aabb(Vector3(19.0628f, 18.9807f, -18.5532f), Vector3(21.7565f, 23.6083f, -16.792f)); 
  data[37].mAabb = Aabb(Vector3(-11.8174f, -7.96123f, -7.94299f), Vector3(-10.3832f, -1.11537f, -5.07723f)); 
  data[38].mAabb = Aabb(Vector3(-8.98593f, 16.9941f, 25.5359f), Vector3(-1.30671f, 24.1705f, 26.7917f)); 
  data[39].mAabb = Aabb(Vector3(-36.8047f, -34.1933f, 0.631892f), Vector3(-33.7457f, -29.0905f, 2.10833f)); 
  data[40].mAabb = Aabb(Vector3(-33.4326f, -19.4971f, -6.69592f), Vector3(-26.532f, -18.7833f, -3.31293f)); 
  data[41].mAabb = Aabb(Vector3(-10.2639f, -5.67813f, -18.3799f), Vector3(-6.61392f, -2.27705f, -15.6472f)); 
  data[42].mAabb = Aabb(Vector3(-16.1888f, -29.0529f, 32.3977f), Vector3(-11.6995f, -25.0836f, 39.1819f)); 
  data[43].mAabb = Aabb(Vector3(9.84179f, -12.648f, 2.27169f), Vector3(12.8572f, -7.20421f, 7.41047f)); 
  data[44].mAabb = Aabb(Vector3(-34.9745f, 28.8653f, 22.9033f), Vector3(-27.1777f, 31.8992f, 25.053f)); 
  data[45].mAabb = Aabb(Vector3(-13.2341f, -9.68971f, 14.5547f), Vector3(-11.9903f, -3.84954f, 20.5971f)); 
  data[46].mAabb = Aabb(Vector3(1.41553f, -40.4218f, 26.0553f), Vector3(4.30564f, -35.4988f, 26.6362f)); 
  data[47].mAabb = Aabb(Vector3(-40.076f, -24.4701f, 0.876693f), Vector3(-35.4679f, -18.8255f, 8.13959f)); 
  data[48].mAabb = Aabb(Vector3(-2.00528f, 19.2053f, -3.60543f), Vector3(1.86107f, 25.6915f, 0.706941f)); 
  data[49].mAabb = Aabb(Vector3(18.095f, -20.7743f, 8.22939f), Vector3(24.4614f, -16.3731f, 15.8414f)); 
  data[50].mAabb = Aabb(Vector3(-16.5493f, 31.9419f, 28.1402f), Vector3(-10.8538f, 33.8646f, 34.1506f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest33(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(45); 
  data[0].mAabb = Aabb(Vector3(9.22394f, -38.0951f, -25.1229f), Vector3(11.2461f, -30.6509f, -23.1895f)); 
  data[1].mAabb = Aabb(Vector3(10.3241f, -4.68207f, -28.7822f), Vector3(18.1815f, -0.730125f, -21.4868f)); 
  data[2].mAabb = Aabb(Vector3(-30.3461f, 24.535f, -4.19175f), Vector3(-27.1554f, 31.7905f, 0.751971f)); 
  data[3].mAabb = Aabb(Vector3(24.8248f, 4.74827f, -6.22739f), Vector3(27.0603f, 5.57782f, 1.21131f)); 
  data[4].mAabb = Aabb(Vector3(2.78858f, -3.34219f, -37.3824f), Vector3(9.44402f, -1.23835f, -36.229f)); 
  data[5].mAabb = Aabb(Vector3(-24.7814f, -15.3262f, 32.2413f), Vector3(-20.7592f, -14.29f, 34.1591f)); 
  data[6].mAabb = Aabb(Vector3(-0.740915f, 37.1667f, 26.926f), Vector3(5.67145f, 40.825f, 28.1093f)); 
  data[7].mAabb = Aabb(Vector3(-34.3175f, -39.7517f, -2.91517f), Vector3(-26.4079f, -36.9673f, 4.51889f)); 
  data[8].mAabb = Aabb(Vector3(9.54012f, 7.09242f, 27.5194f), Vector3(14.3312f, 12.153f, 33.7369f)); 
  data[9].mAabb = Aabb(Vector3(-31.1237f, -31.5989f, -22.3124f), Vector3(-24.7637f, -29.8293f, -14.8672f)); 
  data[10].mAabb = Aabb(Vector3(3.71223f, 32.0225f, 8.24461f), Vector3(5.80772f, 39.2738f, 12.3243f)); 
  data[11].mAabb = Aabb(Vector3(-17.305f, -2.97192f, -16.3926f), Vector3(-14.8924f, -1.76639f, -11.6767f)); 
  data[12].mAabb = Aabb(Vector3(-10.4073f, -4.65563f, -45.6821f), Vector3(-9.48131f, -3.40464f, -39.4469f)); 
  data[13].mAabb = Aabb(Vector3(-49.0879f, 7.08585f, 6.84115f), Vector3(-48.1368f, 9.95764f, 8.46487f)); 
  data[14].mAabb = Aabb(Vector3(-38.1891f, 14.4655f, 12.7665f), Vector3(-35.0931f, 17.533f, 15.6935f)); 
  data[15].mAabb = Aabb(Vector3(-13.9974f, 19.8426f, -42.5065f), Vector3(-10.1542f, 21.5816f, -34.7756f)); 
  data[16].mAabb = Aabb(Vector3(0.94781f, -26.7114f, -11.6614f), Vector3(4.59915f, -20.6314f, -5.39773f)); 
  data[17].mAabb = Aabb(Vector3(-7.97995f, -18.8157f, -19.2826f), Vector3(-0.209335f, -15.8811f, -16.1447f)); 
  data[18].mAabb = Aabb(Vector3(-26.7174f, 19.1028f, -20.8731f), Vector3(-21.2084f, 19.8781f, -15.6063f)); 
  data[19].mAabb = Aabb(Vector3(4.96912f, -40.4558f, 14.9592f), Vector3(12.3241f, -32.8446f, 17.69f)); 
  data[20].mAabb = Aabb(Vector3(7.04614f, -7.01722f, 41.8047f), Vector3(13.0737f, -4.3275f, 44.4998f)); 
  data[21].mAabb = Aabb(Vector3(-38.2248f, -32.2379f, -10.9532f), Vector3(-35.826f, -30.5597f, -5.51985f)); 
  data[22].mAabb = Aabb(Vector3(7.87808f, 13.9538f, -24.725f), Vector3(9.6964f, 16.7757f, -21.4726f)); 
  data[23].mAabb = Aabb(Vector3(27.6102f, 30.7114f, 3.27166f), Vector3(31.8577f, 38.6747f, 7.24725f)); 
  data[24].mAabb = Aabb(Vector3(-14.161f, -7.71248f, -1.51671f), Vector3(-13.0473f, -5.93127f, -0.119599f)); 
  data[25].mAabb = Aabb(Vector3(-16.742f, -22.8748f, -16.6517f), Vector3(-14.6532f, -15.1144f, -10.1556f)); 
  data[26].mAabb = Aabb(Vector3(-35.8921f, -19.5443f, 4.52807f), Vector3(-29.4907f, -15.6146f, 10.0908f)); 
  data[27].mAabb = Aabb(Vector3(1.29284f, -14.1069f, 44.1424f), Vector3(7.09706f, -8.97365f, 46.426f)); 
  data[28].mAabb = Aabb(Vector3(-32.7017f, 8.09312f, 23.0626f), Vector3(-30.4599f, 11.6774f, 25.6642f)); 
  data[29].mAabb = Aabb(Vector3(9.67161f, 19.634f, -28.039f), Vector3(14.2491f, 21.5664f, -22.6614f)); 
  data[30].mAabb = Aabb(Vector3(-42.2356f, -10.9661f, 5.30089f), Vector3(-40.0585f, -4.22575f, 10.9678f)); 
  data[31].mAabb = Aabb(Vector3(30.189f, 28.8624f, 3.11537f), Vector3(33.6713f, 33.7519f, 5.04942f)); 
  data[32].mAabb = Aabb(Vector3(-10.2562f, 10.4042f, -18.8139f), Vector3(-3.01119f, 16.7815f, -13.0709f)); 
  data[33].mAabb = Aabb(Vector3(-7.59943f, 34.9375f, 27.4654f), Vector3(0.0521953f, 42.4873f, 30.9379f)); 
  data[34].mAabb = Aabb(Vector3(-11.3442f, 1.06889f, 49.0482f), Vector3(-5.33312f, 1.48025f, 49.4853f)); 
  data[35].mAabb = Aabb(Vector3(6.47114f, 22.3841f, -37.8624f), Vector3(8.72202f, 29.996f, -36.3446f)); 
  data[36].mAabb = Aabb(Vector3(22.391f, 5.84637f, -21.8992f), Vector3(27.8234f, 6.60425f, -18.0514f)); 
  data[37].mAabb = Aabb(Vector3(-23.8221f, -32.8673f, 13.1498f), Vector3(-20.7615f, -30.4406f, 16.0937f)); 
  data[38].mAabb = Aabb(Vector3(17.5336f, 1.68453f, 14.6515f), Vector3(19.5971f, 2.64606f, 20.5664f)); 
  data[39].mAabb = Aabb(Vector3(30.4268f, -12.6405f, -5.6136f), Vector3(35.9086f, -12.2136f, -3.27388f)); 
  data[40].mAabb = Aabb(Vector3(-11.2357f, -23.4642f, -28.896f), Vector3(-6.71829f, -20.6003f, -23.4219f)); 
  data[41].mAabb = Aabb(Vector3(-4.53634f, -31.1301f, 23.3034f), Vector3(1.04545f, -24.6839f, 25.6044f)); 
  data[42].mAabb = Aabb(Vector3(-22.2266f, -19.421f, -33.8296f), Vector3(-20.7326f, -18.0562f, -32.008f)); 
  data[43].mAabb = Aabb(Vector3(28.9144f, 31.1172f, 6.96758f), Vector3(33.5004f, 34.6062f, 13.3373f)); 
  data[44].mAabb = Aabb(Vector3(22.4849f, -26.9529f, -34.465f), Vector3(23.9776f, -22.0101f, -33.8646f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest34(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(36); 
  data[0].mAabb = Aabb(Vector3(14.047f, -29.719f, -9.37468f), Vector3(18.3717f, -28.2226f, -8.09307f)); 
  data[1].mAabb = Aabb(Vector3(-2.14542f, 9.8784f, -50.454f), Vector3(0.281505f, 13.3961f, -43.8712f)); 
  data[2].mAabb = Aabb(Vector3(18.0968f, -24.273f, -5.80905f), Vector3(21.841f, -18.1714f, -0.287336f)); 
  data[3].mAabb = Aabb(Vector3(15.4777f, -39.8852f, -6.60476f), Vector3(22.7422f, -34.887f, -0.97844f)); 
  data[4].mAabb = Aabb(Vector3(26.4938f, 36.1919f, -11.465f), Vector3(32.439f, 37.2875f, -7.78531f)); 
  data[5].mAabb = Aabb(Vector3(-13.6477f, 40.9231f, -6.51137f), Vector3(-10.6885f, 46.2491f, 1.14628f)); 
  data[6].mAabb = Aabb(Vector3(-43.9461f, -2.92408f, -1.60408f), Vector3(-40.9277f, -1.19668f, 0.444783f)); 
  data[7].mAabb = Aabb(Vector3(-16.3801f, -40.3645f, -23.6613f), Vector3(-15.0286f, -36.1335f, -20.9266f)); 
  data[8].mAabb = Aabb(Vector3(1.79997f, -2.73954f, 14.8779f), Vector3(2.26747f, -1.86151f, 20.0872f)); 
  data[9].mAabb = Aabb(Vector3(-10.6125f, -4.01282f, -29.7001f), Vector3(-3.84112f, -0.642587f, -23.153f)); 
  data[10].mAabb = Aabb(Vector3(-8.66157f, 15.6421f, 24.9771f), Vector3(-3.29456f, 23.0815f, 31.3198f)); 
  data[11].mAabb = Aabb(Vector3(-20.0627f, 3.32378f, -16.0509f), Vector3(-16.9274f, 5.61108f, -12.8706f)); 
  data[12].mAabb = Aabb(Vector3(-12.4487f, -5.39418f, 17.929f), Vector3(-5.09934f, 2.00487f, 22.1553f)); 
  data[13].mAabb = Aabb(Vector3(15.2345f, -41.6473f, -12.7573f), Vector3(19.2889f, -38.7062f, -6.17002f)); 
  data[14].mAabb = Aabb(Vector3(-15.1375f, -25.4918f, -32.5897f), Vector3(-8.80102f, -20.3924f, -24.8797f)); 
  data[15].mAabb = Aabb(Vector3(34.0085f, -0.36569f, -9.96589f), Vector3(35.3743f, 6.99137f, -4.08861f)); 
  data[16].mAabb = Aabb(Vector3(-9.67735f, -26.9989f, 3.11533f), Vector3(-1.75134f, -25.0718f, 10.6085f)); 
  data[17].mAabb = Aabb(Vector3(22.393f, -23.0399f, -0.0875862f), Vector3(23.2952f, -18.6675f, 4.04712f)); 
  data[18].mAabb = Aabb(Vector3(-3.86007f, -39.2141f, 4.36043f), Vector3(-1.51432f, -37.7128f, 9.57505f)); 
  data[19].mAabb = Aabb(Vector3(-24.1298f, 8.18621f, 17.2443f), Vector3(-20.4211f, 14.2182f, 24.3254f)); 
  data[20].mAabb = Aabb(Vector3(-14.7075f, 19.6958f, -5.39496f), Vector3(-7.21914f, 24.7135f, -1.72877f)); 
  data[21].mAabb = Aabb(Vector3(-5.2903f, -30.9169f, -24.514f), Vector3(2.23399f, -22.918f, -20.4209f)); 
  data[22].mAabb = Aabb(Vector3(9.91595f, 7.60969f, -21.7724f), Vector3(11.8546f, 9.82532f, -19.2307f)); 
  data[23].mAabb = Aabb(Vector3(2.30364f, 21.4351f, -28.0265f), Vector3(3.03717f, 24.3765f, -24.9381f)); 
  data[24].mAabb = Aabb(Vector3(20.4348f, -5.23407f, -30.6192f), Vector3(23.7433f, -0.00436521f, -28.2818f)); 
  data[25].mAabb = Aabb(Vector3(5.61565f, -7.92806f, -10.2739f), Vector3(6.42619f, -3.06459f, -8.08357f)); 
  data[26].mAabb = Aabb(Vector3(17.1707f, -21.7576f, 17.92f), Vector3(20.9563f, -20.1737f, 25.5317f)); 
  data[27].mAabb = Aabb(Vector3(2.28911f, -12.319f, -10.2061f), Vector3(5.3439f, -11.7865f, -4.91675f)); 
  data[28].mAabb = Aabb(Vector3(-22.7798f, -11.9181f, -1.06782f), Vector3(-19.7624f, -9.11961f, 0.753512f)); 
  data[29].mAabb = Aabb(Vector3(-4.33725f, -18.3009f, -30.9092f), Vector3(1.30902f, -11.9243f, -28.5048f)); 
  data[30].mAabb = Aabb(Vector3(-9.98427f, -18.5775f, 33.2985f), Vector3(-7.75032f, -14.5126f, 39.8421f)); 
  data[31].mAabb = Aabb(Vector3(17.3552f, -46.2443f, 1.44656f), Vector3(24.5742f, -43.6393f, 8.21542f)); 
  data[32].mAabb = Aabb(Vector3(8.0867f, -8.31114f, -30.2825f), Vector3(10.2165f, -7.25591f, -22.9866f)); 
  data[33].mAabb = Aabb(Vector3(-1.44434f, 5.33087f, -26.3157f), Vector3(4.73006f, 11.8534f, -25.1755f)); 
  data[34].mAabb = Aabb(Vector3(4.24715f, -2.56985f, 41.3202f), Vector3(9.45783f, -1.6408f, 44.4901f)); 
  data[35].mAabb = Aabb(Vector3(33.7741f, -18.2153f, 7.345f), Vector3(38.118f, -14.6231f, 14.2036f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest35(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(63); 
  data[0].mAabb = Aabb(Vector3(-11.9675f, 44.8421f, -0.866984f), Vector3(-11.4785f, 48.6996f, 3.97352f)); 
  data[1].mAabb = Aabb(Vector3(6.34844f, 37.8997f, 23.4482f), Vector3(12.9046f, 43.0132f, 28.4284f)); 
  data[2].mAabb = Aabb(Vector3(-38.996f, -25.7742f, -9.30408f), Vector3(-36.4953f, -24.0229f, -3.35675f)); 
  data[3].mAabb = Aabb(Vector3(-17.0357f, -26.5728f, -41.3316f), Vector3(-14.5981f, -21.4685f, -39.0853f)); 
  data[4].mAabb = Aabb(Vector3(-7.88235f, 16.1153f, -40.3699f), Vector3(-2.56149f, 24.1099f, -36.2983f)); 
  data[5].mAabb = Aabb(Vector3(10.1873f, 43.2933f, -13.609f), Vector3(12.0272f, 50.3045f, -12.3963f)); 
  data[6].mAabb = Aabb(Vector3(-5.72025f, -24.4908f, 16.4395f), Vector3(0.817135f, -22.8903f, 22.6775f)); 
  data[7].mAabb = Aabb(Vector3(17.3949f, 8.33366f, 38.7933f), Vector3(24.18f, 12.6992f, 42.2953f)); 
  data[8].mAabb = Aabb(Vector3(-3.78361f, -7.66069f, 34.7489f), Vector3(2.50652f, -1.18501f, 35.5014f)); 
  data[9].mAabb = Aabb(Vector3(42.3124f, 10.6439f, 6.98825f), Vector3(46.7498f, 18.1837f, 12.3803f)); 
  data[10].mAabb = Aabb(Vector3(-31.4415f, 0.306559f, 4.16203f), Vector3(-24.2055f, 3.47663f, 4.75872f)); 
  data[11].mAabb = Aabb(Vector3(-32.3147f, -6.96788f, 22.3985f), Vector3(-27.5336f, -4.99672f, 23.0021f)); 
  data[12].mAabb = Aabb(Vector3(23.8063f, -24.9937f, -16.6912f), Vector3(24.6205f, -21.6576f, -15.1823f)); 
  data[13].mAabb = Aabb(Vector3(-38.7382f, -9.32441f, 10.5178f), Vector3(-37.9119f, -7.33353f, 18.3788f)); 
  data[14].mAabb = Aabb(Vector3(-17.7393f, -12.0191f, 40.8598f), Vector3(-10.7162f, -10.3736f, 44.5346f)); 
  data[15].mAabb = Aabb(Vector3(2.55812f, -49.4217f, 2.71744f), Vector3(6.61675f, -48.8969f, 3.15432f)); 
  data[16].mAabb = Aabb(Vector3(-13.1087f, -19.2708f, 23.1967f), Vector3(-9.98916f, -16.7318f, 29.7323f)); 
  data[17].mAabb = Aabb(Vector3(7.07523f, 27.1404f, -7.5458f), Vector3(14.7199f, 33.4685f, -1.24476f)); 
  data[18].mAabb = Aabb(Vector3(-8.56353f, -8.44601f, -10.6668f), Vector3(-2.50534f, -4.68403f, -6.64246f)); 
  data[19].mAabb = Aabb(Vector3(24.9065f, 18.2172f, -17.0811f), Vector3(32.2111f, 25.2338f, -11.8053f)); 
  data[20].mAabb = Aabb(Vector3(-2.47686f, 47.1328f, 3.36981f), Vector3(4.80088f, 51.9888f, 7.11857f)); 
  data[21].mAabb = Aabb(Vector3(38.0287f, 12.7765f, -8.06216f), Vector3(38.7947f, 16.7317f, -2.24217f)); 
  data[22].mAabb = Aabb(Vector3(-25.7956f, -30.2397f, -13.7251f), Vector3(-23.9471f, -24.0778f, -7.49877f)); 
  data[23].mAabb = Aabb(Vector3(9.36719f, -31.9084f, 2.44222f), Vector3(13.8788f, -29.0389f, 6.28863f)); 
  data[24].mAabb = Aabb(Vector3(12.1356f, -29.4921f, -21.588f), Vector3(12.7323f, -23.7901f, -15.8993f)); 
  data[25].mAabb = Aabb(Vector3(-6.45944f, -5.55951f, 1.53215f), Vector3(-5.33138f, 1.87873f, 2.52314f)); 
  data[26].mAabb = Aabb(Vector3(34.789f, -18.6696f, 8.15222f), Vector3(36.002f, -13.5675f, 9.91904f)); 
  data[27].mAabb = Aabb(Vector3(-44.6261f, -12.2582f, -6.87463f), Vector3(-36.6461f, -6.12368f, -1.73307f)); 
  data[28].mAabb = Aabb(Vector3(-31.79f, 11.9324f, -39.4291f), Vector3(-27.2295f, 13.4485f, -31.8598f)); 
  data[29].mAabb = Aabb(Vector3(-36.5846f, -6.67452f, 23.345f), Vector3(-30.8776f, 0.499771f, 23.9872f)); 
  data[30].mAabb = Aabb(Vector3(7.9112f, 6.63288f, -41.9202f), Vector3(14.5077f, 13.7462f, -36.4565f)); 
  data[31].mAabb = Aabb(Vector3(1.32756f, -3.55518f, 48.4521f), Vector3(2.19654f, 1.88048f, 49.4362f)); 
  data[32].mAabb = Aabb(Vector3(35.8711f, 3.20744f, -12.6198f), Vector3(42.3872f, 7.90344f, -9.3396f)); 
  data[33].mAabb = Aabb(Vector3(-26.0578f, 17.4556f, -23.3769f), Vector3(-19.2128f, 22.0686f, -19.1817f)); 
  data[34].mAabb = Aabb(Vector3(20.4599f, 8.10616f, 25.3125f), Vector3(27.5933f, 10.7117f, 30.7548f)); 
  data[35].mAabb = Aabb(Vector3(-8.602f, 10.1443f, 15.7872f), Vector3(-4.10361f, 11.4099f, 17.5287f)); 
  data[36].mAabb = Aabb(Vector3(-38.4377f, -3.78181f, -21.7131f), Vector3(-35.9247f, 2.49835f, -15.5725f)); 
  data[37].mAabb = Aabb(Vector3(-40.0529f, 13.5889f, 3.01479f), Vector3(-38.9263f, 14.4152f, 8.56921f)); 
  data[38].mAabb = Aabb(Vector3(15.3535f, 13.1371f, -36.4023f), Vector3(20.2154f, 14.3653f, -34.9389f)); 
  data[39].mAabb = Aabb(Vector3(-23.5717f, -25.4069f, 32.0736f), Vector3(-17.5718f, -22.7761f, 32.8637f)); 
  data[40].mAabb = Aabb(Vector3(32.949f, 4.76805f, -17.5914f), Vector3(39.8436f, 12.7483f, -11.5947f)); 
  data[41].mAabb = Aabb(Vector3(-0.495022f, 11.113f, 24.2376f), Vector3(2.51153f, 12.3422f, 25.9833f)); 
  data[42].mAabb = Aabb(Vector3(-6.63477f, -1.21412f, -0.678229f), Vector3(1.0535f, 6.16289f, 1.72064f)); 
  data[43].mAabb = Aabb(Vector3(3.25649f, 42.8606f, 8.84186f), Vector3(7.97499f, 44.7072f, 13.3706f)); 
  data[44].mAabb = Aabb(Vector3(-32.2853f, 20.7214f, 5.951f), Vector3(-26.4514f, 22.6856f, 13.0276f)); 
  data[45].mAabb = Aabb(Vector3(10.7978f, 4.02984f, 4.37087f), Vector3(13.1815f, 10.079f, 10.6777f)); 
  data[46].mAabb = Aabb(Vector3(-1.8756f, 17.9864f, -42.1175f), Vector3(2.42518f, 22.2369f, -36.4592f)); 
  data[47].mAabb = Aabb(Vector3(11.7399f, 7.60878f, 0.681898f), Vector3(14.3049f, 11.092f, 7.02399f)); 
  data[48].mAabb = Aabb(Vector3(-21.3805f, 1.68324f, -17.3365f), Vector3(-14.5557f, 6.47365f, -15.1455f)); 
  data[49].mAabb = Aabb(Vector3(-1.1168f, -11.1797f, -33.3811f), Vector3(1.81368f, -9.43232f, -26.7565f)); 
  data[50].mAabb = Aabb(Vector3(16.1172f, -42.8132f, 0.308859f), Vector3(20.7508f, -35.5524f, 4.47441f)); 
  data[51].mAabb = Aabb(Vector3(34.5055f, 10.2736f, -30.4094f), Vector3(41.2221f, 11.3339f, -27.0927f)); 
  data[52].mAabb = Aabb(Vector3(21.2884f, -35.0162f, 1.95843f), Vector3(23.0091f, -33.9287f, 6.84277f)); 
  data[53].mAabb = Aabb(Vector3(2.10833f, 27.9794f, 15.0568f), Vector3(6.47544f, 34.5678f, 17.0816f)); 
  data[54].mAabb = Aabb(Vector3(-18.0719f, -43.4343f, -15.0178f), Vector3(-16.5906f, -39.0461f, -9.33214f)); 
  data[55].mAabb = Aabb(Vector3(36.5472f, -0.415598f, 4.77971f), Vector3(43.2161f, 7.47725f, 7.28295f)); 
  data[56].mAabb = Aabb(Vector3(-30.3678f, -24.3312f, -6.45064f), Vector3(-27.4447f, -22.1411f, -3.71477f)); 
  data[57].mAabb = Aabb(Vector3(-33.3949f, -24.2426f, -23.1574f), Vector3(-28.7052f, -21.0816f, -22.1455f)); 
  data[58].mAabb = Aabb(Vector3(-5.00965f, -9.6125f, -14.4093f), Vector3(0.25275f, -5.7211f, -10.7081f)); 
  data[59].mAabb = Aabb(Vector3(-23.9876f, -14.0726f, -15.7637f), Vector3(-22.3505f, -8.78699f, -14.2321f)); 
  data[60].mAabb = Aabb(Vector3(16.03f, 37.7011f, 1.449f), Vector3(16.7508f, 44.6217f, 8.06199f)); 
  data[61].mAabb = Aabb(Vector3(11.4262f, 12.4124f, 16.3606f), Vector3(18.5724f, 13.4607f, 16.8177f)); 
  data[62].mAabb = Aabb(Vector3(29.2208f, -20.8596f, 7.11179f), Vector3(36.7541f, -13.8952f, 13.3507f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest36(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(29); 
  data[0].mAabb = Aabb(Vector3(12.5274f, 38.3396f, -26.6242f), Vector3(16.9018f, 42.9595f, -22.9844f)); 
  data[1].mAabb = Aabb(Vector3(-25.934f, -24.2803f, -37.146f), Vector3(-20.0632f, -19.5484f, -33.2256f)); 
  data[2].mAabb = Aabb(Vector3(-20.9596f, -31.9211f, -34.2358f), Vector3(-13.4214f, -25.5691f, -30.3486f)); 
  data[3].mAabb = Aabb(Vector3(23.8645f, -3.01564f, -15.6373f), Vector3(24.91f, 0.736596f, -9.62925f)); 
  data[4].mAabb = Aabb(Vector3(10.3881f, -32.4275f, 33.0278f), Vector3(14.2491f, -31.1401f, 38.0963f)); 
  data[5].mAabb = Aabb(Vector3(25.6877f, 15.9385f, -34.5309f), Vector3(29.9929f, 22.9036f, -30.0666f)); 
  data[6].mAabb = Aabb(Vector3(-16.1259f, -46.5278f, -7.43233f), Vector3(-10.4984f, -40.891f, -6.66865f)); 
  data[7].mAabb = Aabb(Vector3(15.5122f, -23.2673f, 6.49981f), Vector3(17.0283f, -20.9912f, 11.7548f)); 
  data[8].mAabb = Aabb(Vector3(-1.55041f, 34.0215f, 5.44704f), Vector3(5.39217f, 38.965f, 10.811f)); 
  data[9].mAabb = Aabb(Vector3(33.6389f, -7.66189f, 11.2819f), Vector3(41.2926f, -1.67514f, 18.8811f)); 
  data[10].mAabb = Aabb(Vector3(-10.2121f, 12.3387f, 38.9676f), Vector3(-9.27978f, 16.4746f, 40.7321f)); 
  data[11].mAabb = Aabb(Vector3(-0.371097f, -2.66789f, 11.7746f), Vector3(5.79773f, 1.67418f, 15.3376f)); 
  data[12].mAabb = Aabb(Vector3(-21.4602f, -4.45021f, -38.5247f), Vector3(-17.5558f, -0.955656f, -33.6112f)); 
  data[13].mAabb = Aabb(Vector3(-7.90424f, 0.312402f, -11.3047f), Vector3(-7.25745f, 6.91937f, -3.41258f)); 
  data[14].mAabb = Aabb(Vector3(1.19025f, -2.88455f, -2.11889f), Vector3(6.67346f, 3.81241f, 4.21067f)); 
  data[15].mAabb = Aabb(Vector3(-13.0909f, 25.8801f, 12.2185f), Vector3(-12.2189f, 32.0004f, 18.9092f)); 
  data[16].mAabb = Aabb(Vector3(-3.75005f, 33.2892f, -3.16275f), Vector3(-0.618482f, 36.9894f, -1.46528f)); 
  data[17].mAabb = Aabb(Vector3(43.7118f, 5.2278f, -6.77486f), Vector3(48.5222f, 12.0277f, -0.610437f)); 
  data[18].mAabb = Aabb(Vector3(5.17234f, -17.2993f, 36.3304f), Vector3(7.24533f, -13.069f, 39.7911f)); 
  data[19].mAabb = Aabb(Vector3(9.26953f, -33.29f, -26.9738f), Vector3(13.0909f, -30.5388f, -25.3362f)); 
  data[20].mAabb = Aabb(Vector3(-27.0054f, -16.2194f, -38.8813f), Vector3(-24.7828f, -12.3892f, -33.9663f)); 
  data[21].mAabb = Aabb(Vector3(-18.2653f, 10.4784f, 39.7008f), Vector3(-14.0492f, 18.1298f, 42.1558f)); 
  data[22].mAabb = Aabb(Vector3(-16.6718f, 1.31067f, 9.58905f), Vector3(-11.5256f, 5.15221f, 12.5128f)); 
  data[23].mAabb = Aabb(Vector3(-43.6644f, 6.68084f, 7.48619f), Vector3(-42.3986f, 13.045f, 8.87171f)); 
  data[24].mAabb = Aabb(Vector3(12.7324f, 27.1494f, -18.3826f), Vector3(14.5848f, 32.4209f, -10.6212f)); 
  data[25].mAabb = Aabb(Vector3(16.3552f, -34.7434f, -17.8042f), Vector3(21.5394f, -33.2911f, -16.469f)); 
  data[26].mAabb = Aabb(Vector3(-49.9417f, -0.492568f, 11.2737f), Vector3(-45.1039f, 6.22225f, 14.0733f)); 
  data[27].mAabb = Aabb(Vector3(-11.5556f, -46.4989f, -1.01539f), Vector3(-9.57099f, -42.25f, 2.17091f)); 
  data[28].mAabb = Aabb(Vector3(-13.4518f, -2.64117f, 24.7127f), Vector3(-11.8633f, -0.353178f, 27.6657f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest37(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(33); 
  data[0].mAabb = Aabb(Vector3(-10.8464f, 33.3598f, -29.2602f), Vector3(-4.07455f, 38.9124f, -25.5393f)); 
  data[1].mAabb = Aabb(Vector3(-7.97295f, -33.4501f, 34.6701f), Vector3(-0.586898f, -32.9348f, 36.1825f)); 
  data[2].mAabb = Aabb(Vector3(-19.499f, -28.8675f, 5.86984f), Vector3(-14.6984f, -21.1653f, 12.6983f)); 
  data[3].mAabb = Aabb(Vector3(2.93764f, -8.8364f, -45.1296f), Vector3(5.94396f, -2.06082f, -38.6298f)); 
  data[4].mAabb = Aabb(Vector3(5.77876f, -13.7222f, 39.8838f), Vector3(12.3208f, -8.67716f, 42.631f)); 
  data[5].mAabb = Aabb(Vector3(-35.8866f, -3.60958f, -29.3417f), Vector3(-31.6383f, -3.172f, -27.7732f)); 
  data[6].mAabb = Aabb(Vector3(41.0074f, 0.319793f, -0.658611f), Vector3(43.6951f, 4.0776f, 3.33902f)); 
  data[7].mAabb = Aabb(Vector3(-12.5463f, 28.4461f, -22.0314f), Vector3(-8.08385f, 35.4889f, -18.5135f)); 
  data[8].mAabb = Aabb(Vector3(-4.68676f, -12.1895f, 16.153f), Vector3(2.31056f, -8.38115f, 20.8875f)); 
  data[9].mAabb = Aabb(Vector3(-10.0126f, 18.8939f, 9.68552f), Vector3(-3.18974f, 24.2767f, 10.5144f)); 
  data[10].mAabb = Aabb(Vector3(10.909f, -29.2815f, -36.7332f), Vector3(13.4538f, -25.8136f, -32.4716f)); 
  data[11].mAabb = Aabb(Vector3(-21.5494f, -37.6841f, -19.7232f), Vector3(-19.0846f, -34.1466f, -13.9185f)); 
  data[12].mAabb = Aabb(Vector3(34.8604f, 23.1503f, 16.1123f), Vector3(39.0376f, 27.9379f, 20.9665f)); 
  data[13].mAabb = Aabb(Vector3(-34.5077f, -4.82375f, -24.2791f), Vector3(-31.5413f, 0.257048f, -21.2798f)); 
  data[14].mAabb = Aabb(Vector3(10.5426f, -47.5278f, 12.213f), Vector3(14.7378f, -42.9262f, 19.6093f)); 
  data[15].mAabb = Aabb(Vector3(-24.54f, 0.505216f, 2.31907f), Vector3(-22.0553f, 4.42724f, 4.06409f)); 
  data[16].mAabb = Aabb(Vector3(29.2628f, 3.78079f, -6.78777f), Vector3(34.5363f, 8.80198f, -5.03857f)); 
  data[17].mAabb = Aabb(Vector3(14.2018f, 18.5717f, 26.4311f), Vector3(21.2615f, 24.2145f, 28.0228f)); 
  data[18].mAabb = Aabb(Vector3(-4.48532f, -26.0998f, -2.85685f), Vector3(1.94258f, -20.919f, 2.91002f)); 
  data[19].mAabb = Aabb(Vector3(-16.057f, 2.50522f, 10.8255f), Vector3(-9.65811f, 3.49783f, 13.2953f)); 
  data[20].mAabb = Aabb(Vector3(-20.3215f, -44.8683f, -6.99068f), Vector3(-18.6769f, -44.2699f, -1.53113f)); 
  data[21].mAabb = Aabb(Vector3(-43.1584f, -15.6045f, -3.85612f), Vector3(-40.3017f, -12.1988f, 1.33415f)); 
  data[22].mAabb = Aabb(Vector3(7.26969f, -29.7553f, -35.8653f), Vector3(8.65543f, -24.6411f, -33.6335f)); 
  data[23].mAabb = Aabb(Vector3(30.0455f, -19.041f, 6.37599f), Vector3(33.3612f, -11.2398f, 11.2701f)); 
  data[24].mAabb = Aabb(Vector3(-43.4703f, -26.7438f, 7.56655f), Vector3(-36.0912f, -18.976f, 12.0649f)); 
  data[25].mAabb = Aabb(Vector3(-43.5363f, 7.23705f, 22.8368f), Vector3(-38.4471f, 12.5658f, 29.4999f)); 
  data[26].mAabb = Aabb(Vector3(5.9863f, -28.7308f, 5.15329f), Vector3(10.0934f, -27.5519f, 7.54543f)); 
  data[27].mAabb = Aabb(Vector3(-11.8131f, 17.5693f, 30.9649f), Vector3(-8.66163f, 20.0941f, 36.0705f)); 
  data[28].mAabb = Aabb(Vector3(24.3846f, 10.7318f, 35.8799f), Vector3(28.1948f, 12.1249f, 41.9599f)); 
  data[29].mAabb = Aabb(Vector3(31.2464f, 3.94395f, 15.1569f), Vector3(35.4614f, 8.6817f, 17.8847f)); 
  data[30].mAabb = Aabb(Vector3(0.649646f, -15.6811f, 15.1949f), Vector3(3.861f, -9.92719f, 19.1886f)); 
  data[31].mAabb = Aabb(Vector3(26.5068f, -25.0124f, 14.253f), Vector3(33.0644f, -21.5253f, 15.5583f)); 
  data[32].mAabb = Aabb(Vector3(16.8526f, -32.1329f, -36.221f), Vector3(19.6319f, -25.7685f, -31.1263f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest38(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(35); 
  data[0].mAabb = Aabb(Vector3(21.9135f, -13.5946f, 29.6499f), Vector3(24.9792f, -10.7643f, 33.925f)); 
  data[1].mAabb = Aabb(Vector3(0.507396f, 8.09217f, 17.3073f), Vector3(6.78616f, 13.3045f, 24.383f)); 
  data[2].mAabb = Aabb(Vector3(-41.9022f, -12.5537f, -23.2409f), Vector3(-40.4065f, -10.2589f, -22.8347f)); 
  data[3].mAabb = Aabb(Vector3(-10.9114f, -0.06236f, -29.6648f), Vector3(-7.47413f, 7.20332f, -21.8794f)); 
  data[4].mAabb = Aabb(Vector3(15.3608f, 28.474f, 26.0155f), Vector3(21.5461f, 36.2604f, 33.8446f)); 
  data[5].mAabb = Aabb(Vector3(5.25497f, 21.9949f, 15.8269f), Vector3(6.5426f, 25.5565f, 17.7266f)); 
  data[6].mAabb = Aabb(Vector3(-1.73646f, -6.95644f, 39.8156f), Vector3(3.78317f, -4.12756f, 45.5414f)); 
  data[7].mAabb = Aabb(Vector3(28.1641f, 27.7621f, 11.5158f), Vector3(31.8461f, 29.0796f, 17.9632f)); 
  data[8].mAabb = Aabb(Vector3(-16.7783f, -22.6249f, -40.9018f), Vector3(-13.9427f, -16.4329f, -33.6468f)); 
  data[9].mAabb = Aabb(Vector3(27.5437f, 12.9994f, -17.4572f), Vector3(35.4542f, 17.4579f, -14.1907f)); 
  data[10].mAabb = Aabb(Vector3(-12.4224f, 10.415f, 28.1603f), Vector3(-5.37293f, 14.4669f, 31.871f)); 
  data[11].mAabb = Aabb(Vector3(-3.31299f, -7.93629f, -35.9121f), Vector3(4.19645f, -7.49454f, -27.9306f)); 
  data[12].mAabb = Aabb(Vector3(24.5449f, -28.1112f, -4.66596f), Vector3(31.5172f, -21.0149f, 0.39814f)); 
  data[13].mAabb = Aabb(Vector3(0.484477f, -25.3311f, 9.06946f), Vector3(4.7725f, -22.3004f, 10.6053f)); 
  data[14].mAabb = Aabb(Vector3(24.9362f, -1.52427f, -18.003f), Vector3(32.8624f, 2.80458f, -15.2057f)); 
  data[15].mAabb = Aabb(Vector3(-5.19236f, -32.0012f, -4.83249f), Vector3(-1.50854f, -31.3292f, -4.32742f)); 
  data[16].mAabb = Aabb(Vector3(-20.2367f, -8.72757f, 0.920676f), Vector3(-12.3671f, -1.58784f, 4.8638f)); 
  data[17].mAabb = Aabb(Vector3(-24.0936f, -8.86472f, 3.90432f), Vector3(-22.3296f, -2.91554f, 10.4964f)); 
  data[18].mAabb = Aabb(Vector3(44.3341f, 1.70298f, -11.501f), Vector3(51.183f, 2.70509f, -3.65898f)); 
  data[19].mAabb = Aabb(Vector3(40.5972f, 2.5438f, -24.0971f), Vector3(46.9764f, 5.10781f, -18.9739f)); 
  data[20].mAabb = Aabb(Vector3(28.9765f, -8.1747f, -13.3649f), Vector3(31.962f, -2.21299f, -7.23462f)); 
  data[21].mAabb = Aabb(Vector3(15.975f, 11.3457f, 0.153287f), Vector3(17.9617f, 11.8667f, 3.91063f)); 
  data[22].mAabb = Aabb(Vector3(1.3807f, -31.9516f, -36.9566f), Vector3(1.79369f, -29.4778f, -30.2849f)); 
  data[23].mAabb = Aabb(Vector3(9.42778f, -6.63315f, -51.137f), Vector3(16.4831f, -3.06925f, -43.5201f)); 
  data[24].mAabb = Aabb(Vector3(10.8419f, -19.0488f, -30.7141f), Vector3(16.0177f, -11.6892f, -28.3713f)); 
  data[25].mAabb = Aabb(Vector3(41.264f, -14.5777f, -11.7113f), Vector3(45.6239f, -13.8409f, -6.0548f)); 
  data[26].mAabb = Aabb(Vector3(-33.9196f, -8.58312f, 5.58448f), Vector3(-29.8186f, -5.34556f, 12.5721f)); 
  data[27].mAabb = Aabb(Vector3(15.6175f, 18.7896f, 34.6722f), Vector3(22.7027f, 25.3191f, 35.9543f)); 
  data[28].mAabb = Aabb(Vector3(22.0174f, 14.0552f, -13.0649f), Vector3(23.4755f, 20.2505f, -9.6126f)); 
  data[29].mAabb = Aabb(Vector3(32.2494f, -30.0995f, -17.5039f), Vector3(35.2937f, -27.0348f, -15.0737f)); 
  data[30].mAabb = Aabb(Vector3(-21.9779f, 21.2045f, 24.1697f), Vector3(-19.9592f, 26.6673f, 26.1838f)); 
  data[31].mAabb = Aabb(Vector3(-22.972f, -4.56491f, -16.9895f), Vector3(-21.0412f, 1.8354f, -12.8682f)); 
  data[32].mAabb = Aabb(Vector3(-8.94272f, -13.7559f, -41.9158f), Vector3(-1.21711f, -7.10279f, -41.0029f)); 
  data[33].mAabb = Aabb(Vector3(-44.4726f, -18.4086f, 14.5857f), Vector3(-40.5622f, -14.4128f, 22.4865f)); 
  data[34].mAabb = Aabb(Vector3(28.4528f, 3.41274f, 28.2137f), Vector3(36.2403f, 6.93953f, 29.9167f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest39(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(50); 
  data[0].mAabb = Aabb(Vector3(-11.6386f, -4.11775f, -50.7503f), Vector3(-6.85975f, 0.746878f, -45.1681f)); 
  data[1].mAabb = Aabb(Vector3(15.7197f, -18.0092f, 16.3382f), Vector3(23.6557f, -10.1604f, 23.3939f)); 
  data[2].mAabb = Aabb(Vector3(9.60437f, -10.7877f, -43.2358f), Vector3(13.6876f, -9.27768f, -39.3653f)); 
  data[3].mAabb = Aabb(Vector3(-25.1055f, 6.61368f, -39.1372f), Vector3(-17.2224f, 12.0526f, -34.6784f)); 
  data[4].mAabb = Aabb(Vector3(-41.2643f, -6.19297f, -11.1434f), Vector3(-36.8313f, -0.0951171f, -4.05752f)); 
  data[5].mAabb = Aabb(Vector3(-12.0154f, -35.4979f, -3.67376f), Vector3(-5.39773f, -30.1195f, 1.89342f)); 
  data[6].mAabb = Aabb(Vector3(-29.2758f, -13.5152f, -42.3519f), Vector3(-24.8233f, -8.65147f, -34.7617f)); 
  data[7].mAabb = Aabb(Vector3(-8.86714f, 9.65657f, 2.0153f), Vector3(-7.10518f, 16.3336f, 5.5356f)); 
  data[8].mAabb = Aabb(Vector3(16.6946f, 35.9898f, -30.2784f), Vector3(19.3278f, 36.8806f, -26.0143f)); 
  data[9].mAabb = Aabb(Vector3(-18.1653f, -20.7827f, 31.9854f), Vector3(-11.9491f, -15.6993f, 34.7117f)); 
  data[10].mAabb = Aabb(Vector3(-28.9432f, -37.1537f, 15.5702f), Vector3(-20.9873f, -29.2643f, 20.5898f)); 
  data[11].mAabb = Aabb(Vector3(23.9571f, -40.5046f, -15.1981f), Vector3(28.5365f, -35.434f, -10.4425f)); 
  data[12].mAabb = Aabb(Vector3(-18.7251f, -25.1663f, -7.75622f), Vector3(-15.6351f, -21.3147f, -2.47851f)); 
  data[13].mAabb = Aabb(Vector3(-12.7985f, -20.4585f, 21.0628f), Vector3(-5.42152f, -16.0548f, 21.5468f)); 
  data[14].mAabb = Aabb(Vector3(-9.85525f, 39.1723f, 1.87088f), Vector3(-6.47829f, 46.9227f, 9.6002f)); 
  data[15].mAabb = Aabb(Vector3(-6.65213f, 31.7017f, -20.9791f), Vector3(-0.44619f, 34.0588f, -15.6819f)); 
  data[16].mAabb = Aabb(Vector3(19.2094f, 15.8229f, -26.026f), Vector3(20.6234f, 21.26f, -25.5393f)); 
  data[17].mAabb = Aabb(Vector3(21.9564f, -15.5485f, -13.3698f), Vector3(23.466f, -12.7122f, -10.1995f)); 
  data[18].mAabb = Aabb(Vector3(47.7698f, 4.13677f, -1.75109f), Vector3(49.0199f, 6.04345f, 2.79369f)); 
  data[19].mAabb = Aabb(Vector3(-37.236f, -20.9473f, 2.55434f), Vector3(-32.3389f, -18.2742f, 6.18713f)); 
  data[20].mAabb = Aabb(Vector3(25.6919f, -42.0165f, -4.5877f), Vector3(26.0966f, -39.9284f, -3.33462f)); 
  data[21].mAabb = Aabb(Vector3(-14.2208f, -42.4861f, -19.9041f), Vector3(-8.9051f, -37.7316f, -18.4699f)); 
  data[22].mAabb = Aabb(Vector3(31.9754f, -29.2891f, -11.9696f), Vector3(34.1003f, -21.9638f, -9.24366f)); 
  data[23].mAabb = Aabb(Vector3(14.742f, 27.9272f, 4.03748f), Vector3(20.9359f, 28.574f, 8.95753f)); 
  data[24].mAabb = Aabb(Vector3(-1.01071f, 47.4284f, -5.2993f), Vector3(6.27932f, 51.939f, 2.19113f)); 
  data[25].mAabb = Aabb(Vector3(31.8989f, -12.0954f, 3.92124f), Vector3(33.1619f, -4.11583f, 10.454f)); 
  data[26].mAabb = Aabb(Vector3(-32.0808f, -27.3152f, 12.7687f), Vector3(-24.5013f, -22.6804f, 18.2605f)); 
  data[27].mAabb = Aabb(Vector3(-20.6885f, -14.43f, -19.3469f), Vector3(-16.9146f, -13.5826f, -18.6976f)); 
  data[28].mAabb = Aabb(Vector3(17.0887f, 43.7916f, -5.33112f), Vector3(22.4029f, 45.1015f, -2.56439f)); 
  data[29].mAabb = Aabb(Vector3(-22.3326f, -3.93832f, 23.949f), Vector3(-17.9177f, -3.03802f, 29.718f)); 
  data[30].mAabb = Aabb(Vector3(13.0448f, -24.4326f, 24.0339f), Vector3(18.6294f, -19.5959f, 25.9311f)); 
  data[31].mAabb = Aabb(Vector3(-16.1577f, -47.7586f, 6.74877f), Vector3(-13.1043f, -39.982f, 13.0085f)); 
  data[32].mAabb = Aabb(Vector3(-14.1339f, 19.2588f, -20.6728f), Vector3(-8.05554f, 26.4308f, -13.3162f)); 
  data[33].mAabb = Aabb(Vector3(-7.76163f, -37.0995f, 0.0629029f), Vector3(-2.958f, -30.9552f, 6.34585f)); 
  data[34].mAabb = Aabb(Vector3(-11.5641f, -18.025f, 38.6169f), Vector3(-5.34542f, -15.0641f, 45.6782f)); 
  data[35].mAabb = Aabb(Vector3(-24.28f, -4.5825f, -15.4715f), Vector3(-20.3409f, -2.96644f, -10.7704f)); 
  data[36].mAabb = Aabb(Vector3(-39.3329f, -31.2363f, -14.0934f), Vector3(-31.5043f, -25.6389f, -7.25192f)); 
  data[37].mAabb = Aabb(Vector3(16.9204f, -1.67847f, -31.5816f), Vector3(22.6808f, 1.91767f, -23.9931f)); 
  data[38].mAabb = Aabb(Vector3(28.4775f, 34.4892f, -9.8965f), Vector3(34.6081f, 35.5628f, -9.15578f)); 
  data[39].mAabb = Aabb(Vector3(-16.3176f, -25.8215f, 33.1409f), Vector3(-12.7878f, -19.6988f, 34.2903f)); 
  data[40].mAabb = Aabb(Vector3(19.9362f, -17.9539f, -31.8039f), Vector3(24.0317f, -11.3708f, -24.7425f)); 
  data[41].mAabb = Aabb(Vector3(-15.6289f, -44.9725f, -17.5828f), Vector3(-9.43994f, -38.6024f, -15.9185f)); 
  data[42].mAabb = Aabb(Vector3(23.0773f, -15.8046f, 24.5213f), Vector3(26.432f, -13.9682f, 31.9813f)); 
  data[43].mAabb = Aabb(Vector3(-48.525f, -5.40101f, 2.48646f), Vector3(-44.9583f, 0.0713015f, 8.95704f)); 
  data[44].mAabb = Aabb(Vector3(-3.36838f, -36.7364f, 8.5596f), Vector3(1.31487f, -30.2793f, 11.177f)); 
  data[45].mAabb = Aabb(Vector3(14.5025f, -1.78604f, 34.1917f), Vector3(17.2963f, -0.857212f, 34.7837f)); 
  data[46].mAabb = Aabb(Vector3(5.09437f, 8.05628f, -29.6707f), Vector3(6.76865f, 11.07f, -23.0862f)); 
  data[47].mAabb = Aabb(Vector3(1.76915f, -24.4894f, -39.0081f), Vector3(9.20553f, -18.0318f, -36.3743f)); 
  data[48].mAabb = Aabb(Vector3(10.5591f, 23.4596f, -35.4072f), Vector3(11.1671f, 25.3185f, -32.0847f)); 
  data[49].mAabb = Aabb(Vector3(3.12392f, -10.1296f, 35.5632f), Vector3(7.78096f, -7.97473f, 36.1086f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest40(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(40); 
  data[0].mAabb = Aabb(Vector3(-51.3945f, -5.82605f, 3.59481f), Vector3(-44.6449f, -4.45213f, 4.98427f)); 
  data[1].mAabb = Aabb(Vector3(11.4338f, -30.259f, 26.2893f), Vector3(15.5105f, -27.1847f, 30.007f)); 
  data[2].mAabb = Aabb(Vector3(24.1877f, 9.66127f, -20.3739f), Vector3(28.7912f, 12.2424f, -15.4177f)); 
  data[3].mAabb = Aabb(Vector3(14.3412f, -27.9614f, 30.0121f), Vector3(18.0703f, -24.4288f, 35.2228f)); 
  data[4].mAabb = Aabb(Vector3(-23.7285f, 5.63157f, 12.5866f), Vector3(-20.2921f, 6.47829f, 14.0345f)); 
  data[5].mAabb = Aabb(Vector3(-19.2317f, -20.6276f, -11.883f), Vector3(-17.0128f, -18.7376f, -7.52609f)); 
  data[6].mAabb = Aabb(Vector3(28.6757f, 10.3469f, 12.4633f), Vector3(29.3276f, 16.04f, 18.9629f)); 
  data[7].mAabb = Aabb(Vector3(-25.6901f, -13.725f, 40.632f), Vector3(-18.622f, -5.97894f, 41.9013f)); 
  data[8].mAabb = Aabb(Vector3(-26.7662f, 30.8841f, -26.8216f), Vector3(-24.6684f, 38.6663f, -22.3109f)); 
  data[9].mAabb = Aabb(Vector3(32.4871f, -26.7324f, -9.88965f), Vector3(38.4131f, -21.6231f, -8.24668f)); 
  data[10].mAabb = Aabb(Vector3(-22.4153f, -45.9213f, -6.57171f), Vector3(-20.0891f, -42.9384f, -1.87942f)); 
  data[11].mAabb = Aabb(Vector3(16.6252f, -41.8076f, 7.51365f), Vector3(19.8718f, -36.9819f, 10.4353f)); 
  data[12].mAabb = Aabb(Vector3(-31.3775f, -23.3117f, 27.6828f), Vector3(-30.9418f, -22.4111f, 34.6178f)); 
  data[13].mAabb = Aabb(Vector3(0.647399f, -19.933f, 18.9379f), Vector3(3.77061f, -12.2139f, 24.0605f)); 
  data[14].mAabb = Aabb(Vector3(-38.613f, -3.91656f, 16.7217f), Vector3(-32.0093f, -3.39502f, 22.2379f)); 
  data[15].mAabb = Aabb(Vector3(-28.856f, 1.2367f, -16.0559f), Vector3(-27.1578f, 8.64896f, -14.6279f)); 
  data[16].mAabb = Aabb(Vector3(-0.412205f, 13.9423f, -13.034f), Vector3(5.41521f, 16.9896f, -10.1785f)); 
  data[17].mAabb = Aabb(Vector3(16.8218f, 17.529f, -13.6465f), Vector3(19.9826f, 23.5132f, -7.38765f)); 
  data[18].mAabb = Aabb(Vector3(-11.697f, 21.7347f, 11.398f), Vector3(-10.5166f, 28.3054f, 13.1303f)); 
  data[19].mAabb = Aabb(Vector3(-0.189519f, 22.381f, 28.3521f), Vector3(0.952923f, 26.6147f, 35.8894f)); 
  data[20].mAabb = Aabb(Vector3(14.7418f, -18.2423f, -0.0539089f), Vector3(19.0435f, -11.1803f, 1.32187f)); 
  data[21].mAabb = Aabb(Vector3(-17.294f, -9.17838f, -21.9742f), Vector3(-9.36755f, -8.67401f, -14.7144f)); 
  data[22].mAabb = Aabb(Vector3(-42.9894f, -7.06816f, -5.58868f), Vector3(-38.7645f, -3.54114f, -1.7439f)); 
  data[23].mAabb = Aabb(Vector3(-5.67396f, -14.5275f, 3.70479f), Vector3(-0.797041f, -9.08816f, 10.3904f)); 
  data[24].mAabb = Aabb(Vector3(24.216f, 6.86742f, -32.5583f), Vector3(26.8825f, 8.26523f, -29.654f)); 
  data[25].mAabb = Aabb(Vector3(-29.1653f, -6.8343f, 12.8078f), Vector3(-26.2548f, 0.989192f, 14.262f)); 
  data[26].mAabb = Aabb(Vector3(22.3618f, -3.67089f, 17.767f), Vector3(24.8402f, 0.748175f, 25.1816f)); 
  data[27].mAabb = Aabb(Vector3(-34.0492f, 29.3961f, 9.64744f), Vector3(-31.2361f, 34.6768f, 12.1973f)); 
  data[28].mAabb = Aabb(Vector3(-1.40046f, 27.7517f, -19.5526f), Vector3(5.58388f, 32.0456f, -16.508f)); 
  data[29].mAabb = Aabb(Vector3(1.92957f, 17.7892f, -34.0249f), Vector3(8.14942f, 22.3736f, -30.0074f)); 
  data[30].mAabb = Aabb(Vector3(21.9746f, -33.973f, 29.0963f), Vector3(22.4808f, -30.022f, 32.2193f)); 
  data[31].mAabb = Aabb(Vector3(33.217f, -6.10657f, 20.0902f), Vector3(37.0855f, 0.439167f, 24.6397f)); 
  data[32].mAabb = Aabb(Vector3(-34.7953f, 0.846781f, -32.0573f), Vector3(-28.9962f, 8.54108f, -30.8219f)); 
  data[33].mAabb = Aabb(Vector3(0.710473f, 13.4624f, 40.4188f), Vector3(1.45166f, 15.303f, 44.2685f)); 
  data[34].mAabb = Aabb(Vector3(-36.5463f, 10.9228f, -5.45093f), Vector3(-30.2458f, 17.422f, 1.33671f)); 
  data[35].mAabb = Aabb(Vector3(22.8039f, -37.9666f, -3.31395f), Vector3(27.5078f, -30.0248f, 1.58153f)); 
  data[36].mAabb = Aabb(Vector3(-0.770177f, -9.74444f, 32.7148f), Vector3(2.94333f, -4.02627f, 39.2536f)); 
  data[37].mAabb = Aabb(Vector3(17.4743f, -33.9407f, 27.505f), Vector3(25.3312f, -27.629f, 31.305f)); 
  data[38].mAabb = Aabb(Vector3(-29.6704f, 13.7854f, 11.027f), Vector3(-23.2397f, 15.6418f, 13.9514f)); 
  data[39].mAabb = Aabb(Vector3(-16.6044f, 16.4701f, -17.8612f), Vector3(-15.3921f, 24.0971f, -11.8657f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest41(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(44); 
  data[0].mAabb = Aabb(Vector3(-21.2179f, -19.3762f, 19.8409f), Vector3(-18.6397f, -16.1834f, 21.1773f)); 
  data[1].mAabb = Aabb(Vector3(4.76948f, 7.49611f, 7.06729f), Vector3(9.61532f, 14.1548f, 10.2415f)); 
  data[2].mAabb = Aabb(Vector3(6.58777f, -27.9433f, 24.2517f), Vector3(13.651f, -26.8157f, 28.4159f)); 
  data[3].mAabb = Aabb(Vector3(-24.5889f, 3.03477f, 0.263008f), Vector3(-17.5863f, 7.82262f, 6.22054f)); 
  data[4].mAabb = Aabb(Vector3(0.373428f, 25.9046f, -8.31662f), Vector3(1.20321f, 28.8047f, -2.69865f)); 
  data[5].mAabb = Aabb(Vector3(-2.8975f, 5.71572f, 26.8693f), Vector3(-0.280841f, 12.9893f, 29.642f)); 
  data[6].mAabb = Aabb(Vector3(-28.6919f, 18.2744f, -28.1428f), Vector3(-25.0819f, 25.2237f, -27.2669f)); 
  data[7].mAabb = Aabb(Vector3(34.3443f, 2.82779f, -21.8258f), Vector3(37.0621f, 9.45702f, -17.9523f)); 
  data[8].mAabb = Aabb(Vector3(10.5304f, -12.8048f, 28.3091f), Vector3(17.9904f, -8.01556f, 30.4612f)); 
  data[9].mAabb = Aabb(Vector3(-7.71311f, -1.77981f, -20.7255f), Vector3(-3.8398f, 3.54568f, -14.3978f)); 
  data[10].mAabb = Aabb(Vector3(-42.1504f, 10.5374f, -24.5598f), Vector3(-39.6868f, 16.0818f, -22.4212f)); 
  data[11].mAabb = Aabb(Vector3(-3.83612f, -27.5422f, 4.03731f), Vector3(-1.39388f, -21.2604f, 10.4279f)); 
  data[12].mAabb = Aabb(Vector3(21.6325f, -17.8699f, -34.2411f), Vector3(24.1467f, -10.8079f, -27.7237f)); 
  data[13].mAabb = Aabb(Vector3(25.3854f, -24.9646f, -2.57769f), Vector3(27.5177f, -24.3241f, 2.25331f)); 
  data[14].mAabb = Aabb(Vector3(-30.9053f, -9.8545f, 24.2053f), Vector3(-29.3282f, -9.08827f, 27.4549f)); 
  data[15].mAabb = Aabb(Vector3(12.1862f, 43.1168f, -12.5486f), Vector3(20.1642f, 47.0075f, -8.53193f)); 
  data[16].mAabb = Aabb(Vector3(-30.2663f, -1.74089f, -14.6862f), Vector3(-22.8091f, 0.672121f, -8.52502f)); 
  data[17].mAabb = Aabb(Vector3(-27.6665f, 8.6812f, 32.241f), Vector3(-24.0223f, 13.8768f, 39.2149f)); 
  data[18].mAabb = Aabb(Vector3(-4.50965f, 26.0834f, 23.8751f), Vector3(2.96408f, 31.9938f, 26.4061f)); 
  data[19].mAabb = Aabb(Vector3(4.11936f, 8.0918f, -40.9449f), Vector3(8.09867f, 10.686f, -38.9104f)); 
  data[20].mAabb = Aabb(Vector3(-11.1273f, 32.1247f, -0.180831f), Vector3(-5.6657f, 36.4895f, 2.81922f)); 
  data[21].mAabb = Aabb(Vector3(-26.9067f, -9.93799f, -37.0094f), Vector3(-21.9888f, -5.8005f, -32.9257f)); 
  data[22].mAabb = Aabb(Vector3(-7.30171f, -12.4535f, 21.2756f), Vector3(0.00803566f, -6.53259f, 21.9831f)); 
  data[23].mAabb = Aabb(Vector3(-8.54794f, -20.6014f, -28.1393f), Vector3(-2.29236f, -20.1847f, -21.3238f)); 
  data[24].mAabb = Aabb(Vector3(24.0176f, -25.249f, -12.2318f), Vector3(27.264f, -19.1003f, -4.65858f)); 
  data[25].mAabb = Aabb(Vector3(31.443f, -31.0273f, -4.12283f), Vector3(36.8432f, -29.6724f, -3.50735f)); 
  data[26].mAabb = Aabb(Vector3(1.8647f, 32.403f, 8.23344f), Vector3(8.35407f, 36.5525f, 13.2658f)); 
  data[27].mAabb = Aabb(Vector3(-24.3073f, -24.008f, -16.7328f), Vector3(-23.4864f, -18.0919f, -15.0511f)); 
  data[28].mAabb = Aabb(Vector3(-16.3588f, -26.0044f, -7.40049f), Vector3(-9.89189f, -24.6664f, 0.140962f)); 
  data[29].mAabb = Aabb(Vector3(-0.0218821f, 35.8807f, -3.25955f), Vector3(2.07732f, 41.0791f, 0.530028f)); 
  data[30].mAabb = Aabb(Vector3(-5.59585f, 26.4527f, 30.935f), Vector3(-2.96737f, 28.1159f, 38.6978f)); 
  data[31].mAabb = Aabb(Vector3(10.5008f, -42.8256f, 3.8454f), Vector3(12.199f, -41.2489f, 7.36894f)); 
  data[32].mAabb = Aabb(Vector3(-20.5413f, -26.525f, -17.9406f), Vector3(-13.0884f, -21.0926f, -16.1998f)); 
  data[33].mAabb = Aabb(Vector3(-19.7489f, -19.6772f, -23.739f), Vector3(-15.5852f, -12.1278f, -20.9583f)); 
  data[34].mAabb = Aabb(Vector3(1.55122f, 25.5612f, -0.957706f), Vector3(3.81973f, 33.4623f, 6.43067f)); 
  data[35].mAabb = Aabb(Vector3(18.7856f, -21.6721f, 10.1475f), Vector3(25.8537f, -14.714f, 12.708f)); 
  data[36].mAabb = Aabb(Vector3(-10.9725f, 2.6545f, -30.0693f), Vector3(-10.4741f, 5.04269f, -29.0855f)); 
  data[37].mAabb = Aabb(Vector3(-12.5859f, 23.3242f, 32.9604f), Vector3(-5.97804f, 25.166f, 38.8175f)); 
  data[38].mAabb = Aabb(Vector3(-26.5728f, -11.3152f, -11.2966f), Vector3(-22.2317f, -4.63191f, -7.08978f)); 
  data[39].mAabb = Aabb(Vector3(40.6855f, 1.16768f, 20.2424f), Vector3(46.4572f, 7.15397f, 25.8998f)); 
  data[40].mAabb = Aabb(Vector3(-11.8459f, 2.92353f, 39.7508f), Vector3(-9.94499f, 9.02695f, 42.9609f)); 
  data[41].mAabb = Aabb(Vector3(-0.602744f, 33.3013f, -36.4045f), Vector3(6.18537f, 35.3333f, -30.1343f)); 
  data[42].mAabb = Aabb(Vector3(24.7524f, -26.4638f, -4.86819f), Vector3(28.3335f, -19.6956f, -4.21723f)); 
  data[43].mAabb = Aabb(Vector3(-40.2837f, -16.7873f, 6.4874f), Vector3(-36.5203f, -12.3844f, 7.27381f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest42(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(45); 
  data[0].mAabb = Aabb(Vector3(-25.7304f, -22.7625f, 19.1844f), Vector3(-21.9415f, -17.5393f, 23.6359f)); 
  data[1].mAabb = Aabb(Vector3(-23.2814f, -41.8449f, -19.9514f), Vector3(-17.1882f, -35.834f, -13.8957f)); 
  data[2].mAabb = Aabb(Vector3(-7.89705f, -42.9759f, 23.405f), Vector3(-1.53293f, -38.3854f, 30.608f)); 
  data[3].mAabb = Aabb(Vector3(33.8156f, 7.48406f, -24.3203f), Vector3(36.0595f, 13.7642f, -23.7888f)); 
  data[4].mAabb = Aabb(Vector3(-9.72276f, 14.7561f, -25.6051f), Vector3(-6.75309f, 18.7428f, -19.8109f)); 
  data[5].mAabb = Aabb(Vector3(7.07726f, -39.9098f, -0.624745f), Vector3(13.8872f, -38.6948f, 5.04309f)); 
  data[6].mAabb = Aabb(Vector3(-20.8749f, -43.1288f, 10.3335f), Vector3(-18.4473f, -40.6408f, 16.5049f)); 
  data[7].mAabb = Aabb(Vector3(29.9766f, -9.94913f, -18.0078f), Vector3(31.7882f, -2.79943f, -12.5374f)); 
  data[8].mAabb = Aabb(Vector3(9.89249f, -27.2646f, -41.3929f), Vector3(12.7775f, -20.9461f, -34.7245f)); 
  data[9].mAabb = Aabb(Vector3(1.74774f, 32.7438f, -19.394f), Vector3(6.40223f, 40.4564f, -13.2729f)); 
  data[10].mAabb = Aabb(Vector3(-27.3614f, 38.3578f, 6.61459f), Vector3(-25.8238f, 40.8824f, 14.2516f)); 
  data[11].mAabb = Aabb(Vector3(1.16389f, -42.3753f, -20.8429f), Vector3(1.83642f, -35.3339f, -16.258f)); 
  data[12].mAabb = Aabb(Vector3(24.7494f, 15.2513f, -15.1742f), Vector3(26.4504f, 16.672f, -10.4135f)); 
  data[13].mAabb = Aabb(Vector3(24.8376f, 16.8576f, 11.9979f), Vector3(30.7374f, 20.781f, 16.4353f)); 
  data[14].mAabb = Aabb(Vector3(37.8702f, 3.90376f, 23.2176f), Vector3(44.4015f, 8.9704f, 29.8895f)); 
  data[15].mAabb = Aabb(Vector3(-1.26628f, -13.0497f, -7.71473f), Vector3(-0.791829f, -8.62322f, -0.294117f)); 
  data[16].mAabb = Aabb(Vector3(-31.7421f, 23.7419f, 25.786f), Vector3(-24.0228f, 29.7829f, 33.0628f)); 
  data[17].mAabb = Aabb(Vector3(-39.0435f, -10.2664f, -18.4147f), Vector3(-37.7468f, -5.88306f, -14.1726f)); 
  data[18].mAabb = Aabb(Vector3(-42.9113f, 14.2363f, -24.1304f), Vector3(-38.849f, 17.2132f, -17.5237f)); 
  data[19].mAabb = Aabb(Vector3(-0.410666f, -35.7281f, -1.3728f), Vector3(6.33361f, -29.6389f, 5.75928f)); 
  data[20].mAabb = Aabb(Vector3(18.091f, 1.51417f, -2.74448f), Vector3(23.1196f, 9.42835f, 0.26137f)); 
  data[21].mAabb = Aabb(Vector3(27.9034f, 0.37292f, -29.4322f), Vector3(33.9674f, 0.950122f, -26.168f)); 
  data[22].mAabb = Aabb(Vector3(27.9328f, 31.2625f, 4.89155f), Vector3(28.9623f, 32.7993f, 9.75966f)); 
  data[23].mAabb = Aabb(Vector3(-29.676f, -10.2081f, -13.5994f), Vector3(-23.025f, -7.57315f, -12.8387f)); 
  data[24].mAabb = Aabb(Vector3(-16.6887f, -0.615051f, -31.3213f), Vector3(-9.18206f, 0.744024f, -23.7861f)); 
  data[25].mAabb = Aabb(Vector3(-29.2332f, -30.6436f, -19.8249f), Vector3(-23.4078f, -29.1226f, -19.0362f)); 
  data[26].mAabb = Aabb(Vector3(14.5071f, -1.61128f, -16.8884f), Vector3(19.4591f, -0.295813f, -10.5192f)); 
  data[27].mAabb = Aabb(Vector3(13.4865f, 6.47855f, -31.5838f), Vector3(18.2078f, 12.2269f, -24.9226f)); 
  data[28].mAabb = Aabb(Vector3(-8.49659f, 11.4153f, 4.91233f), Vector3(-4.9262f, 12.4408f, 11.7253f)); 
  data[29].mAabb = Aabb(Vector3(-8.16315f, -37.6542f, 27.3561f), Vector3(-2.06251f, -30.7597f, 29.6142f)); 
  data[30].mAabb = Aabb(Vector3(18.1435f, -16.2642f, -24.3466f), Vector3(19.6397f, -11.4735f, -20.699f)); 
  data[31].mAabb = Aabb(Vector3(1.99842f, 6.83427f, -39.1409f), Vector3(4.50119f, 9.63508f, -36.0643f)); 
  data[32].mAabb = Aabb(Vector3(-44.4175f, 0.654401f, 5.22095f), Vector3(-39.9379f, 4.04133f, 9.48485f)); 
  data[33].mAabb = Aabb(Vector3(-11.834f, 9.68447f, 16.1467f), Vector3(-8.89006f, 13.6095f, 21.9706f)); 
  data[34].mAabb = Aabb(Vector3(-36.776f, -5.77968f, -35.0806f), Vector3(-31.6236f, -4.66183f, -27.725f)); 
  data[35].mAabb = Aabb(Vector3(0.541518f, 14.5045f, 18.6727f), Vector3(6.31976f, 18.1129f, 19.3394f)); 
  data[36].mAabb = Aabb(Vector3(-24.4777f, -28.3272f, -37.7892f), Vector3(-18.2908f, -25.9404f, -33.5373f)); 
  data[37].mAabb = Aabb(Vector3(5.27965f, 13.2414f, 38.7238f), Vector3(9.21233f, 16.4608f, 39.9715f)); 
  data[38].mAabb = Aabb(Vector3(5.74082f, 21.6659f, -20.4545f), Vector3(6.40152f, 26.7189f, -14.9569f)); 
  data[39].mAabb = Aabb(Vector3(8.69701f, -19.4038f, 23.693f), Vector3(14.8264f, -14.8641f, 29.4764f)); 
  data[40].mAabb = Aabb(Vector3(-19.5291f, -23.788f, -26.1271f), Vector3(-17.9898f, -17.7537f, -18.2317f)); 
  data[41].mAabb = Aabb(Vector3(8.53067f, 20.8947f, 37.8071f), Vector3(9.23706f, 28.0426f, 41.7606f)); 
  data[42].mAabb = Aabb(Vector3(19.5689f, -1.25885f, 32.7664f), Vector3(22.7578f, 4.36075f, 36.1874f)); 
  data[43].mAabb = Aabb(Vector3(21.8799f, -0.0321815f, -40.4685f), Vector3(27.1312f, 6.74155f, -36.7707f)); 
  data[44].mAabb = Aabb(Vector3(7.31802f, 4.24191f, 4.32931f), Vector3(12.1495f, 4.99098f, 10.359f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest43(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(46); 
  data[0].mAabb = Aabb(Vector3(-7.49891f, -26.121f, 24.3142f), Vector3(-4.86996f, -18.7486f, 28.7455f)); 
  data[1].mAabb = Aabb(Vector3(-20.3185f, 18.2098f, -24.3066f), Vector3(-12.5641f, 21.7816f, -22.3097f)); 
  data[2].mAabb = Aabb(Vector3(22.0443f, 20.482f, 18.8738f), Vector3(29.4213f, 27.449f, 19.3872f)); 
  data[3].mAabb = Aabb(Vector3(-43.1699f, 2.97381f, 7.04f), Vector3(-35.8467f, 6.37257f, 12.4585f)); 
  data[4].mAabb = Aabb(Vector3(38.6482f, -4.22934f, -15.5237f), Vector3(41.4247f, 0.518624f, -12.0226f)); 
  data[5].mAabb = Aabb(Vector3(-20.1871f, -26.5816f, 9.70523f), Vector3(-12.6913f, -20.6802f, 13.4419f)); 
  data[6].mAabb = Aabb(Vector3(-11.0015f, -2.50171f, -22.9562f), Vector3(-6.39196f, 0.872702f, -19.355f)); 
  data[7].mAabb = Aabb(Vector3(11.2054f, -0.243933f, 35.6825f), Vector3(13.9447f, 0.994069f, 38.8816f)); 
  data[8].mAabb = Aabb(Vector3(-27.0085f, -18.2287f, 4.72147f), Vector3(-20.3927f, -13.2743f, 7.43253f)); 
  data[9].mAabb = Aabb(Vector3(12.1269f, 38.6317f, 15.8695f), Vector3(17.58f, 41.6503f, 20.2253f)); 
  data[10].mAabb = Aabb(Vector3(1.06542f, 35.0264f, -32.8934f), Vector3(2.14871f, 40.347f, -25.0585f)); 
  data[11].mAabb = Aabb(Vector3(16.4286f, -13.9734f, 4.38309f), Vector3(19.5815f, -9.17558f, 6.81605f)); 
  data[12].mAabb = Aabb(Vector3(-39.6428f, 32.3973f, 4.76386f), Vector3(-34.3085f, 33.427f, 6.30571f)); 
  data[13].mAabb = Aabb(Vector3(26.9976f, -11.3846f, 30.5402f), Vector3(28.2302f, -4.74078f, 36.0162f)); 
  data[14].mAabb = Aabb(Vector3(-41.0074f, -5.00714f, -24.7047f), Vector3(-33.5954f, -1.86444f, -18.6323f)); 
  data[15].mAabb = Aabb(Vector3(-28.4491f, -28.0778f, 0.621008f), Vector3(-22.7374f, -26.7304f, 6.99116f)); 
  data[16].mAabb = Aabb(Vector3(1.12731f, 34.0426f, -19.4266f), Vector3(7.61761f, 36.4953f, -15.1237f)); 
  data[17].mAabb = Aabb(Vector3(-13.929f, -15.8822f, 24.6284f), Vector3(-12.1504f, -12.2473f, 25.2852f)); 
  data[18].mAabb = Aabb(Vector3(10.1988f, 3.15734f, 20.1524f), Vector3(10.7088f, 4.97936f, 21.5424f)); 
  data[19].mAabb = Aabb(Vector3(4.41958f, 10.8831f, -25.342f), Vector3(7.37163f, 15.2252f, -23.8881f)); 
  data[20].mAabb = Aabb(Vector3(-31.4487f, -18.6738f, -24.6235f), Vector3(-26.0866f, -11.5858f, -19.1268f)); 
  data[21].mAabb = Aabb(Vector3(-13.0121f, -11.5048f, -36.2964f), Vector3(-6.75138f, -10.1846f, -29.7008f)); 
  data[22].mAabb = Aabb(Vector3(-7.94515f, 33.1163f, 8.27709f), Vector3(-4.22399f, 34.121f, 14.4974f)); 
  data[23].mAabb = Aabb(Vector3(6.0463f, 33.97f, 29.5798f), Vector3(10.3093f, 38.1986f, 35.6792f)); 
  data[24].mAabb = Aabb(Vector3(8.9196f, 18.4116f, 36.605f), Vector3(15.572f, 22.8919f, 39.3372f)); 
  data[25].mAabb = Aabb(Vector3(7.75121f, 30.4581f, -1.04331f), Vector3(11.066f, 35.8395f, 6.83701f)); 
  data[26].mAabb = Aabb(Vector3(-15.233f, -5.17069f, -46.5865f), Vector3(-10.739f, 1.20086f, -45.9462f)); 
  data[27].mAabb = Aabb(Vector3(14.4734f, 40.5348f, -8.03013f), Vector3(15.7729f, 44.3413f, -5.41254f)); 
  data[28].mAabb = Aabb(Vector3(29.5125f, -36.2619f, -9.88908f), Vector3(34.8498f, -33.3604f, -3.09889f)); 
  data[29].mAabb = Aabb(Vector3(-20.9215f, -45.3768f, -6.02549f), Vector3(-17.49f, -39.0908f, -0.735019f)); 
  data[30].mAabb = Aabb(Vector3(-19.8219f, -1.8827f, -31.007f), Vector3(-13.8134f, 2.07619f, -29.1947f)); 
  data[31].mAabb = Aabb(Vector3(-10.2412f, -40.2895f, 7.28874f), Vector3(-3.69382f, -38.9569f, 10.383f)); 
  data[32].mAabb = Aabb(Vector3(-19.5535f, -32.4309f, -16.7602f), Vector3(-16.8835f, -28.2857f, -13.5245f)); 
  data[33].mAabb = Aabb(Vector3(6.56205f, -17.6187f, -9.82551f), Vector3(14.257f, -12.9477f, -9.37634f)); 
  data[34].mAabb = Aabb(Vector3(1.40383f, -24.9569f, -37.0549f), Vector3(8.95688f, -21.1283f, -35.6838f)); 
  data[35].mAabb = Aabb(Vector3(10.4359f, -6.44956f, -16.6444f), Vector3(15.872f, -1.46781f, -10.6291f)); 
  data[36].mAabb = Aabb(Vector3(-11.2643f, -42.1921f, -27.4915f), Vector3(-7.13331f, -40.1212f, -25.1077f)); 
  data[37].mAabb = Aabb(Vector3(-10.2898f, 0.243349f, 30.3545f), Vector3(-6.21795f, 7.51807f, 30.7786f)); 
  data[38].mAabb = Aabb(Vector3(-18.0529f, -0.145212f, -32.9359f), Vector3(-13.2024f, 0.942492f, -26.5716f)); 
  data[39].mAabb = Aabb(Vector3(15.9029f, 30.5869f, -15.4841f), Vector3(20.3076f, 33.2384f, -12.2409f)); 
  data[40].mAabb = Aabb(Vector3(-26.9575f, -17.5436f, -11.2261f), Vector3(-20.0279f, -12.6417f, -8.41416f)); 
  data[41].mAabb = Aabb(Vector3(-3.99408f, -17.7154f, 39.9565f), Vector3(2.45726f, -16.6959f, 44.2362f)); 
  data[42].mAabb = Aabb(Vector3(31.1199f, 8.18264f, 19.4825f), Vector3(33.8553f, 14.4431f, 20.8237f)); 
  data[43].mAabb = Aabb(Vector3(-13.8501f, 2.81889f, -43.5365f), Vector3(-6.13564f, 3.94138f, -36.5923f)); 
  data[44].mAabb = Aabb(Vector3(-5.7466f, -4.19675f, 36.9466f), Vector3(1.70996f, 1.07122f, 42.1724f)); 
  data[45].mAabb = Aabb(Vector3(-39.6206f, 21.6278f, -13.5269f), Vector3(-32.3881f, 25.6741f, -11.549f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest44(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(36); 
  data[0].mAabb = Aabb(Vector3(-38.2071f, -8.70341f, -24.2029f), Vector3(-31.8156f, -3.37792f, -20.4022f)); 
  data[1].mAabb = Aabb(Vector3(19.3204f, 30.1723f, 29.7805f), Vector3(24.5849f, 33.0568f, 33.4845f)); 
  data[2].mAabb = Aabb(Vector3(40.3353f, 3.69071f, 19.7294f), Vector3(41.0065f, 9.99847f, 21.0686f)); 
  data[3].mAabb = Aabb(Vector3(38.9676f, -18.1139f, -13.746f), Vector3(39.7981f, -16.8288f, -13.2223f)); 
  data[4].mAabb = Aabb(Vector3(-8.60553f, 24.9259f, 37.8986f), Vector3(-5.47118f, 31.8817f, 42.3217f)); 
  data[5].mAabb = Aabb(Vector3(-7.04049f, 11.6789f, -5.50781f), Vector3(-0.683789f, 18.865f, 0.882986f)); 
  data[6].mAabb = Aabb(Vector3(-5.39842f, 3.11378f, 34.3079f), Vector3(-3.89763f, 9.14391f, 39.9115f)); 
  data[7].mAabb = Aabb(Vector3(-16.788f, -39.3029f, -21.3018f), Vector3(-13.2872f, -35.7016f, -18.0061f)); 
  data[8].mAabb = Aabb(Vector3(-1.63087f, 37.0941f, 27.0866f), Vector3(0.626974f, 41.3332f, 33.9369f)); 
  data[9].mAabb = Aabb(Vector3(-49.9398f, -7.38322f, -9.62356f), Vector3(-46.3826f, -0.37245f, -2.30709f)); 
  data[10].mAabb = Aabb(Vector3(-20.3148f, -16.2011f, -43.1101f), Vector3(-19.2294f, -11.5935f, -40.8212f)); 
  data[11].mAabb = Aabb(Vector3(-12.3681f, -2.10111f, -28.9786f), Vector3(-6.41383f, 2.60904f, -24.1638f)); 
  data[12].mAabb = Aabb(Vector3(-29.6218f, 26.0382f, 3.18799f), Vector3(-27.8139f, 30.1151f, 5.64832f)); 
  data[13].mAabb = Aabb(Vector3(-15.9283f, -27.4091f, -19.1101f), Vector3(-9.81698f, -24.9112f, -13.0777f)); 
  data[14].mAabb = Aabb(Vector3(-20.2912f, -31.7434f, -13.7109f), Vector3(-13.6105f, -29.0539f, -7.90433f)); 
  data[15].mAabb = Aabb(Vector3(16.0978f, -46.7085f, -10.9162f), Vector3(23.8582f, -40.9548f, -8.81657f)); 
  data[16].mAabb = Aabb(Vector3(-41.7007f, -5.21506f, 13.7844f), Vector3(-38.2734f, 2.11579f, 15.4872f)); 
  data[17].mAabb = Aabb(Vector3(-1.62787f, -33.9416f, -37.1105f), Vector3(4.62307f, -29.9628f, -34.2436f)); 
  data[18].mAabb = Aabb(Vector3(-30.1263f, -21.3991f, -7.2865f), Vector3(-29.0878f, -18.2172f, -5.342f)); 
  data[19].mAabb = Aabb(Vector3(-22.2889f, -43.0183f, 4.20892f), Vector3(-17.9438f, -40.7621f, 6.5818f)); 
  data[20].mAabb = Aabb(Vector3(7.17154f, 4.15416f, -47.3309f), Vector3(10.1159f, 10.808f, -44.7308f)); 
  data[21].mAabb = Aabb(Vector3(-1.65647f, -38.6808f, 29.6549f), Vector3(2.56452f, -36.1855f, 33.4185f)); 
  data[22].mAabb = Aabb(Vector3(-22.2283f, 34.581f, 19.0485f), Vector3(-14.6546f, 40.3374f, 21.3956f)); 
  data[23].mAabb = Aabb(Vector3(32.3765f, 33.929f, 1.4972f), Vector3(39.1681f, 34.6495f, 5.16873f)); 
  data[24].mAabb = Aabb(Vector3(7.30519f, 14.6152f, 12.5906f), Vector3(8.21778f, 15.0899f, 14.5898f)); 
  data[25].mAabb = Aabb(Vector3(-7.48098f, -15.2181f, 40.91f), Vector3(0.174585f, -13.1465f, 47.9806f)); 
  data[26].mAabb = Aabb(Vector3(-3.06605f, -5.79188f, 25.5521f), Vector3(-2.3905f, -3.4851f, 30.4919f)); 
  data[27].mAabb = Aabb(Vector3(10.6829f, -42.7523f, -13.5212f), Vector3(16.2742f, -36.9801f, -10.5067f)); 
  data[28].mAabb = Aabb(Vector3(13.322f, 18.2651f, -10.6081f), Vector3(16.0214f, 26.1956f, -6.63533f)); 
  data[29].mAabb = Aabb(Vector3(7.7014f, -12.9311f, -42.3059f), Vector3(13.0063f, -5.5972f, -39.6264f)); 
  data[30].mAabb = Aabb(Vector3(-9.48591f, 9.26415f, -13.058f), Vector3(-6.31143f, 13.0222f, -8.35226f)); 
  data[31].mAabb = Aabb(Vector3(-14.3103f, -33.6362f, 32.82f), Vector3(-6.76303f, -32.9811f, 34.5342f)); 
  data[32].mAabb = Aabb(Vector3(22.5059f, 32.8963f, -29.0184f), Vector3(23.0314f, 34.0439f, -26.4462f)); 
  data[33].mAabb = Aabb(Vector3(16.8781f, -1.36575f, 11.8089f), Vector3(23.1898f, 3.11479f, 18.1248f)); 
  data[34].mAabb = Aabb(Vector3(3.84713f, 17.0722f, -26.0919f), Vector3(10.0011f, 24.0957f, -24.4783f)); 
  data[35].mAabb = Aabb(Vector3(16.1525f, -5.85021f, 28.322f), Vector3(17.5763f, -1.17229f, 36.1645f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest45(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(43); 
  data[0].mAabb = Aabb(Vector3(9.47026f, 34.0947f, 15.7223f), Vector3(16.7972f, 38.0805f, 21.4066f)); 
  data[1].mAabb = Aabb(Vector3(6.27248f, -5.24534f, -29.8696f), Vector3(13.6481f, 1.96745f, -24.5188f)); 
  data[2].mAabb = Aabb(Vector3(-24.7645f, -11.7247f, 39.8205f), Vector3(-19.0716f, -8.01979f, 46.0272f)); 
  data[3].mAabb = Aabb(Vector3(0.465026f, -38.7571f, 0.348077f), Vector3(5.90811f, -33.2686f, 2.48972f)); 
  data[4].mAabb = Aabb(Vector3(40.0695f, 0.733969f, -17.3585f), Vector3(45.1675f, 4.77358f, -12.9854f)); 
  data[5].mAabb = Aabb(Vector3(-13.2751f, -5.42182f, 34.9548f), Vector3(-8.15861f, -1.05332f, 39.7927f)); 
  data[6].mAabb = Aabb(Vector3(28.1486f, -23.6727f, -25.6071f), Vector3(29.4095f, -22.0977f, -23.4072f)); 
  data[7].mAabb = Aabb(Vector3(14.257f, -27.7114f, 20.9211f), Vector3(18.7533f, -21.5623f, 26.2823f)); 
  data[8].mAabb = Aabb(Vector3(-35.164f, -31.2502f, 17.6974f), Vector3(-32.4856f, -25.5638f, 25.267f)); 
  data[9].mAabb = Aabb(Vector3(17.8755f, -17.9662f, -28.6173f), Vector3(21.871f, -14.3304f, -27.2557f)); 
  data[10].mAabb = Aabb(Vector3(-26.323f, -14.5025f, 24.6456f), Vector3(-19.2232f, -8.94114f, 27.6545f)); 
  data[11].mAabb = Aabb(Vector3(-19.7453f, -39.4077f, -0.120295f), Vector3(-15.3112f, -35.4986f, 7.45595f)); 
  data[12].mAabb = Aabb(Vector3(28.8567f, -5.70291f, 21.5988f), Vector3(34.6024f, 0.293125f, 26.1561f)); 
  data[13].mAabb = Aabb(Vector3(5.00863f, -4.0907f, 1.59507f), Vector3(11.1682f, -0.676398f, 1.99971f)); 
  data[14].mAabb = Aabb(Vector3(9.36728f, -16.6959f, 38.2277f), Vector3(13.9494f, -12.2736f, 38.7566f)); 
  data[15].mAabb = Aabb(Vector3(12.5429f, -12.6452f, -38.372f), Vector3(12.9452f, -11.1015f, -32.9544f)); 
  data[16].mAabb = Aabb(Vector3(-5.15086f, 40.2987f, -18.0557f), Vector3(-2.58871f, 46.1312f, -13.4643f)); 
  data[17].mAabb = Aabb(Vector3(-36.0252f, -33.8082f, -9.29546f), Vector3(-34.9426f, -26.9387f, -7.59195f)); 
  data[18].mAabb = Aabb(Vector3(-7.1611f, 20.8873f, -44.6827f), Vector3(0.18738f, 22.1503f, -39.3185f)); 
  data[19].mAabb = Aabb(Vector3(-14.5258f, 2.19509f, -26.3379f), Vector3(-12.6497f, 3.78727f, -21.5614f)); 
  data[20].mAabb = Aabb(Vector3(-24.7084f, -10.5337f, -12.3027f), Vector3(-19.0806f, -4.03525f, -10.9437f)); 
  data[21].mAabb = Aabb(Vector3(29.2981f, -17.9862f, -9.89646f), Vector3(34.8549f, -17.2204f, -7.94872f)); 
  data[22].mAabb = Aabb(Vector3(28.3454f, 10.9906f, -10.0708f), Vector3(34.8784f, 14.0482f, -7.80599f)); 
  data[23].mAabb = Aabb(Vector3(4.28868f, 9.27205f, 31.7055f), Vector3(7.11223f, 13.9365f, 34.7856f)); 
  data[24].mAabb = Aabb(Vector3(24.9899f, -25.044f, 30.2246f), Vector3(31.8703f, -17.714f, 33.2209f)); 
  data[25].mAabb = Aabb(Vector3(-20.7882f, 0.845687f, 7.82395f), Vector3(-18.7588f, 4.49471f, 10.0577f)); 
  data[26].mAabb = Aabb(Vector3(-18.9651f, -30.2744f, 3.59434f), Vector3(-14.6244f, -26.7578f, 5.11136f)); 
  data[27].mAabb = Aabb(Vector3(29.9009f, -19.166f, 4.52988f), Vector3(32.12f, -18.066f, 11.2486f)); 
  data[28].mAabb = Aabb(Vector3(-16.0647f, 9.14913f, -34.4009f), Vector3(-10.0157f, 13.3926f, -28.3511f)); 
  data[29].mAabb = Aabb(Vector3(17.9463f, -9.10054f, -32.0024f), Vector3(21.2402f, -8.12208f, -26.9378f)); 
  data[30].mAabb = Aabb(Vector3(-38.7185f, 16.7894f, -29.8691f), Vector3(-31.9443f, 21.6993f, -25.9865f)); 
  data[31].mAabb = Aabb(Vector3(-5.13182f, 7.09761f, 2.96352f), Vector3(1.37773f, 8.67262f, 10.585f)); 
  data[32].mAabb = Aabb(Vector3(-17.8543f, 18.3166f, -27.6123f), Vector3(-12.1009f, 22.7466f, -23.02f)); 
  data[33].mAabb = Aabb(Vector3(-35.8851f, -23.8699f, 6.70004f), Vector3(-27.9574f, -21.0582f, 7.60265f)); 
  data[34].mAabb = Aabb(Vector3(-13.4441f, 26.2668f, 37.6674f), Vector3(-7.53479f, 29.1029f, 38.3538f)); 
  data[35].mAabb = Aabb(Vector3(-13.915f, 24.9839f, -0.355328f), Vector3(-7.4896f, 32.8441f, 2.22862f)); 
  data[36].mAabb = Aabb(Vector3(-9.7682f, -31.4113f, -16.4496f), Vector3(-6.01086f, -27.4111f, -12.3632f)); 
  data[37].mAabb = Aabb(Vector3(3.1006f, 14.1458f, 38.8148f), Vector3(9.18176f, 17.7805f, 43.8856f)); 
  data[38].mAabb = Aabb(Vector3(16.1482f, -22.8842f, 30.0834f), Vector3(21.0706f, -20.5421f, 31.5403f)); 
  data[39].mAabb = Aabb(Vector3(13.6664f, 34.1644f, -9.12477f), Vector3(17.8132f, 41.4927f, -6.98034f)); 
  data[40].mAabb = Aabb(Vector3(-36.6452f, 12.9178f, -28.9417f), Vector3(-31.4758f, 17.4983f, -26.9429f)); 
  data[41].mAabb = Aabb(Vector3(-6.35469f, -22.0106f, 12.4797f), Vector3(-1.13659f, -18.0636f, 13.0833f)); 
  data[42].mAabb = Aabb(Vector3(11.8899f, 42.2194f, -11.4607f), Vector3(14.157f, 46.8142f, -5.94596f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest46(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(47); 
  data[0].mAabb = Aabb(Vector3(-10.2897f, 25.618f, 35.3957f), Vector3(-4.08001f, 28.5728f, 37.8593f)); 
  data[1].mAabb = Aabb(Vector3(-30.8085f, -2.78552f, -32.0342f), Vector3(-27.2449f, 0.0227175f, -31.6029f)); 
  data[2].mAabb = Aabb(Vector3(-1.66923f, -28.5968f, 38.605f), Vector3(5.78084f, -24.4705f, 44.7722f)); 
  data[3].mAabb = Aabb(Vector3(-7.64047f, -43.7161f, -11.0197f), Vector3(-6.52888f, -41.3508f, -7.54836f)); 
  data[4].mAabb = Aabb(Vector3(39.6999f, -3.02687f, 1.02428f), Vector3(46.9324f, 4.26455f, 3.73975f)); 
  data[5].mAabb = Aabb(Vector3(18.4907f, 6.72371f, 23.2201f), Vector3(22.788f, 10.3704f, 26.8327f)); 
  data[6].mAabb = Aabb(Vector3(31.035f, -5.40695f, 23.3602f), Vector3(35.4583f, -3.02108f, 29.0626f)); 
  data[7].mAabb = Aabb(Vector3(8.89434f, 13.9481f, 18.1583f), Vector3(16.2138f, 16.967f, 20.8267f)); 
  data[8].mAabb = Aabb(Vector3(11.7776f, 8.83674f, 19.8947f), Vector3(17.5155f, 16.7034f, 27.2376f)); 
  data[9].mAabb = Aabb(Vector3(-23.7156f, 5.15522f, -10.3769f), Vector3(-16.0533f, 5.74402f, -6.73786f)); 
  data[10].mAabb = Aabb(Vector3(-25.1907f, 27.3944f, 11.2093f), Vector3(-17.2869f, 32.8737f, 17.2281f)); 
  data[11].mAabb = Aabb(Vector3(-23.3176f, -29.367f, -18.3986f), Vector3(-16.648f, -28.0355f, -12.0176f)); 
  data[12].mAabb = Aabb(Vector3(-17.4691f, -23.6756f, -0.842524f), Vector3(-9.66946f, -19.7898f, 3.6067f)); 
  data[13].mAabb = Aabb(Vector3(20.247f, 25.9061f, -0.523367f), Vector3(22.1277f, 28.8375f, 4.6059f)); 
  data[14].mAabb = Aabb(Vector3(2.5827f, -0.656262f, 45.0398f), Vector3(5.87292f, 0.355365f, 52.7355f)); 
  data[15].mAabb = Aabb(Vector3(-21.2967f, 34.1493f, 19.3952f), Vector3(-17.155f, 34.6523f, 23.1867f)); 
  data[16].mAabb = Aabb(Vector3(18.3193f, 2.80325f, 12.522f), Vector3(22.7859f, 9.18152f, 13.3395f)); 
  data[17].mAabb = Aabb(Vector3(2.25549f, 0.307081f, 38.4737f), Vector3(4.55531f, 4.45571f, 46.3925f)); 
  data[18].mAabb = Aabb(Vector3(30.176f, -22.6121f, 27.2459f), Vector3(32.9775f, -15.3327f, 31.0364f)); 
  data[19].mAabb = Aabb(Vector3(21.8091f, -39.4744f, -1.34083f), Vector3(29.1122f, -38.7532f, 0.329507f)); 
  data[20].mAabb = Aabb(Vector3(13.8558f, 5.00974f, 39.3152f), Vector3(15.3955f, 7.07739f, 40.4122f)); 
  data[21].mAabb = Aabb(Vector3(-3.6522f, -3.87498f, -2.74346f), Vector3(3.6522f, 3.87498f, 2.74346f)); 
  data[22].mAabb = Aabb(Vector3(-16.1365f, -9.18914f, 18.3311f), Vector3(-14.5091f, -1.97565f, 21.5357f)); 
  data[23].mAabb = Aabb(Vector3(-26.9383f, 6.12537f, 39.2171f), Vector3(-23.3414f, 12.6196f, 41.5932f)); 
  data[24].mAabb = Aabb(Vector3(35.3453f, 12.2964f, 0.817364f), Vector3(36.2442f, 14.0887f, 7.76087f)); 
  data[25].mAabb = Aabb(Vector3(20.3859f, -0.532796f, -27.8113f), Vector3(24.967f, 0.354511f, -24.4767f)); 
  data[26].mAabb = Aabb(Vector3(-47.4831f, -13.3374f, -4.22344f), Vector3(-40.2517f, -8.46098f, 1.37203f)); 
  data[27].mAabb = Aabb(Vector3(27.7917f, 20.5161f, 11.034f), Vector3(30.9747f, 25.0491f, 12.8417f)); 
  data[28].mAabb = Aabb(Vector3(13.208f, 35.5282f, 0.656939f), Vector3(14.9865f, 41.8037f, 6.36444f)); 
  data[29].mAabb = Aabb(Vector3(-32.8841f, -4.99445f, 31.4453f), Vector3(-27.9502f, -0.0799556f, 35.5545f)); 
  data[30].mAabb = Aabb(Vector3(-30.8793f, -2.13035f, 12.7606f), Vector3(-25.8792f, 4.77698f, 16.6021f)); 
  data[31].mAabb = Aabb(Vector3(-0.457169f, -33.4071f, 1.17524f), Vector3(2.46774f, -30.0221f, 8.48081f)); 
  data[32].mAabb = Aabb(Vector3(7.64146f, -13.2106f, 7.56379f), Vector3(14.4356f, -9.96542f, 9.438f)); 
  data[33].mAabb = Aabb(Vector3(14.3577f, -13.1817f, 20.842f), Vector3(19.2047f, -7.88794f, 25.9242f)); 
  data[34].mAabb = Aabb(Vector3(17.5375f, 0.159845f, 2.33417f), Vector3(22.1855f, 5.07643f, 8.79153f)); 
  data[35].mAabb = Aabb(Vector3(-21.892f, -32.665f, -17.7007f), Vector3(-21.1614f, -27.048f, -10.5973f)); 
  data[36].mAabb = Aabb(Vector3(1.33434f, 24.7038f, -0.309515f), Vector3(3.65458f, 28.2094f, 7.34327f)); 
  data[37].mAabb = Aabb(Vector3(-5.64745f, 30.2343f, -18.5067f), Vector3(1.35103f, 34.2876f, -10.5315f)); 
  data[38].mAabb = Aabb(Vector3(4.76949f, -34.0927f, 14.9508f), Vector3(12.5357f, -29.6612f, 18.9062f)); 
  data[39].mAabb = Aabb(Vector3(12.6603f, -18.251f, -4.55073f), Vector3(19.7003f, -14.5045f, 0.779171f)); 
  data[40].mAabb = Aabb(Vector3(18.9106f, -33.9027f, -0.646748f), Vector3(23.3266f, -31.9464f, 7.26697f)); 
  data[41].mAabb = Aabb(Vector3(-14.6941f, -13.5593f, -19.3285f), Vector3(-7.16891f, -8.76236f, -16.1916f)); 
  data[42].mAabb = Aabb(Vector3(-0.346645f, -40.8514f, -15.324f), Vector3(2.57849f, -39.2595f, -14.5618f)); 
  data[43].mAabb = Aabb(Vector3(19.8993f, -10.8062f, -12.1634f), Vector3(27.7648f, -3.27956f, -11.203f)); 
  data[44].mAabb = Aabb(Vector3(-29.3989f, 9.79619f, -15.0987f), Vector3(-26.2975f, 17.7598f, -9.21883f)); 
  data[45].mAabb = Aabb(Vector3(-1.09274f, -48.6398f, -14.7412f), Vector3(2.14436f, -42.3808f, -10.6717f)); 
  data[46].mAabb = Aabb(Vector3(-41.7559f, 3.1904f, -10.8054f), Vector3(-38.2294f, 9.58537f, -4.52365f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest47(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(68); 
  data[0].mAabb = Aabb(Vector3(11.0836f, -8.19619f, -22.1538f), Vector3(13.0016f, -0.602087f, -18.7718f)); 
  data[1].mAabb = Aabb(Vector3(-32.6493f, -8.80537f, -17.4682f), Vector3(-27.485f, -5.31777f, -11.5466f)); 
  data[2].mAabb = Aabb(Vector3(33.1858f, 5.92865f, -6.54442f), Vector3(36.2907f, 6.57358f, -5.11808f)); 
  data[3].mAabb = Aabb(Vector3(-3.82525f, 13.3801f, 12.287f), Vector3(4.08336f, 20.9125f, 19.9938f)); 
  data[4].mAabb = Aabb(Vector3(-22.6162f, -8.00707f, 13.3413f), Vector3(-16.6905f, -4.28823f, 17.0576f)); 
  data[5].mAabb = Aabb(Vector3(-24.6568f, 26.997f, 8.55735f), Vector3(-20.497f, 29.7802f, 16.4347f)); 
  data[6].mAabb = Aabb(Vector3(8.1869f, -16.5206f, -19.8979f), Vector3(16.019f, -12.5926f, -11.9872f)); 
  data[7].mAabb = Aabb(Vector3(20.7125f, 39.0416f, -17.0185f), Vector3(22.7055f, 45.7796f, -9.64264f)); 
  data[8].mAabb = Aabb(Vector3(21.7133f, 21.766f, 15.3912f), Vector3(23.0204f, 22.9785f, 17.6143f)); 
  data[9].mAabb = Aabb(Vector3(18.9546f, 19.7202f, -32.3822f), Vector3(21.7325f, 20.9766f, -24.6675f)); 
  data[10].mAabb = Aabb(Vector3(20.618f, -38.3967f, -7.15178f), Vector3(26.8907f, -33.2289f, -2.76843f)); 
  data[11].mAabb = Aabb(Vector3(-32.7363f, 7.66083f, 8.56707f), Vector3(-25.9808f, 9.94257f, 14.6341f)); 
  data[12].mAabb = Aabb(Vector3(-19.6299f, 7.08538f, -43.6073f), Vector3(-16.6911f, 10.8325f, -41.0252f)); 
  data[13].mAabb = Aabb(Vector3(8.64531f, 12.4633f, 26.5346f), Vector3(14.9832f, 19.2516f, 28.0071f)); 
  data[14].mAabb = Aabb(Vector3(-6.87352f, 28.8503f, -35.7483f), Vector3(-1.66493f, 33.146f, -29.5289f)); 
  data[15].mAabb = Aabb(Vector3(3.29886f, -28.7532f, 40.5334f), Vector3(7.24732f, -25.8355f, 42.01f)); 
  data[16].mAabb = Aabb(Vector3(-13.5488f, 36.1246f, 22.3718f), Vector3(-11.7973f, 44.1066f, 30.0327f)); 
  data[17].mAabb = Aabb(Vector3(-39.5376f, -0.678786f, -12.5516f), Vector3(-35.7318f, 4.39111f, -5.44016f)); 
  data[18].mAabb = Aabb(Vector3(-27.1583f, 8.82337f, -6.29516f), Vector3(-22.3008f, 11.7084f, -4.27459f)); 
  data[19].mAabb = Aabb(Vector3(10.4764f, -23.335f, 13.8522f), Vector3(17.3501f, -19.8284f, 17.3823f)); 
  data[20].mAabb = Aabb(Vector3(17.363f, -0.10362f, 3.24548f), Vector3(19.9448f, 3.90468f, 6.2249f)); 
  data[21].mAabb = Aabb(Vector3(-5.35096f, -25.7361f, 12.2655f), Vector3(-1.36747f, -22.7017f, 14.8796f)); 
  data[22].mAabb = Aabb(Vector3(-20.1985f, -6.24416f, -11.8357f), Vector3(-19.0987f, -1.17403f, -7.78194f)); 
  data[23].mAabb = Aabb(Vector3(17.8368f, 17.9054f, -26.9393f), Vector3(21.7804f, 20.23f, -21.6829f)); 
  data[24].mAabb = Aabb(Vector3(-2.42649f, -45.1803f, 6.63472f), Vector3(3.54751f, -38.0338f, 12.3591f)); 
  data[25].mAabb = Aabb(Vector3(9.04659f, -29.1956f, 32.0477f), Vector3(9.4946f, -24.3555f, 37.6868f)); 
  data[26].mAabb = Aabb(Vector3(-13.1874f, -17.834f, -41.6239f), Vector3(-5.53117f, -13.6167f, -36.9831f)); 
  data[27].mAabb = Aabb(Vector3(15.6303f, -30.2994f, 15.363f), Vector3(21.4271f, -26.1788f, 19.3803f)); 
  data[28].mAabb = Aabb(Vector3(6.35569f, 9.34132f, -22.3869f), Vector3(8.68637f, 16.43f, -17.8748f)); 
  data[29].mAabb = Aabb(Vector3(22.1486f, 3.37668f, -26.9999f), Vector3(24.1889f, 7.37965f, -21.0797f)); 
  data[30].mAabb = Aabb(Vector3(27.5474f, 6.62972f, -31.7349f), Vector3(34.56f, 7.57408f, -26.0381f)); 
  data[31].mAabb = Aabb(Vector3(19.3335f, 1.34812f, 4.05807f), Vector3(25.2947f, 2.67194f, 8.74016f)); 
  data[32].mAabb = Aabb(Vector3(31.4385f, 28.512f, 6.81095f), Vector3(32.6537f, 30.1086f, 9.51504f)); 
  data[33].mAabb = Aabb(Vector3(-0.723399f, 44.9507f, 4.0143f), Vector3(6.58101f, 51.8311f, 5.23305f)); 
  data[34].mAabb = Aabb(Vector3(5.77551f, 18.5199f, 19.0935f), Vector3(10.6728f, 25.4128f, 26.5137f)); 
  data[35].mAabb = Aabb(Vector3(-28.4472f, -25.7957f, -16.9327f), Vector3(-21.3779f, -23.9936f, -12.1773f)); 
  data[36].mAabb = Aabb(Vector3(-35.5187f, -2.41998f, -39.4344f), Vector3(-30.9154f, -1.67439f, -33.7643f)); 
  data[37].mAabb = Aabb(Vector3(27.5193f, -8.37368f, 15.3493f), Vector3(33.9954f, -4.47091f, 22.9756f)); 
  data[38].mAabb = Aabb(Vector3(-27.8564f, 7.30136f, -14.4593f), Vector3(-21.9478f, 11.1468f, -12.7287f)); 
  data[39].mAabb = Aabb(Vector3(31.2582f, -14.2785f, 4.3112f), Vector3(37.4785f, -9.9086f, 11.3637f)); 
  data[40].mAabb = Aabb(Vector3(-3.72765f, 22.7459f, -13.6624f), Vector3(3.95622f, 24.4654f, -8.45958f)); 
  data[41].mAabb = Aabb(Vector3(27.1376f, -28.6256f, 6.57859f), Vector3(30.5607f, -26.0683f, 12.7926f)); 
  data[42].mAabb = Aabb(Vector3(-29.1625f, 25.3044f, -9.44259f), Vector3(-27.6735f, 27.9035f, -8.58312f)); 
  data[43].mAabb = Aabb(Vector3(33.7148f, -10.6189f, 12.3627f), Vector3(41.6118f, -4.97404f, 19.0893f)); 
  data[44].mAabb = Aabb(Vector3(-9.32302f, 9.99989f, 7.6769f), Vector3(-4.91184f, 12.5574f, 14.3581f)); 
  data[45].mAabb = Aabb(Vector3(-1.17403f, 34.6435f, -15.9973f), Vector3(3.15899f, 35.1636f, -15.161f)); 
  data[46].mAabb = Aabb(Vector3(-45.7527f, -11.6724f, -13.1377f), Vector3(-45.0978f, -7.68888f, -10.8504f)); 
  data[47].mAabb = Aabb(Vector3(45.0342f, -17.5001f, -2.1681f), Vector3(48.6612f, -9.64274f, 3.67763f)); 
  data[48].mAabb = Aabb(Vector3(14.4154f, 10.8129f, 26.3679f), Vector3(19.8216f, 12.4607f, 34.1527f)); 
  data[49].mAabb = Aabb(Vector3(-28.0266f, -13.5317f, -39.9297f), Vector3(-24.4739f, -8.19184f, -35.2483f)); 
  data[50].mAabb = Aabb(Vector3(24.2324f, 32.6013f, 19.8441f), Vector3(28.9029f, 39.0333f, 23.8779f)); 
  data[51].mAabb = Aabb(Vector3(18.4816f, 19.7443f, 36.9687f), Vector3(20.4801f, 24.5509f, 39.2873f)); 
  data[52].mAabb = Aabb(Vector3(-19.8202f, -0.867999f, -26.3121f), Vector3(-12.4117f, 4.9063f, -21.5539f)); 
  data[53].mAabb = Aabb(Vector3(-10.7465f, 24.7368f, 33.2072f), Vector3(-6.16898f, 26.6057f, 34.3798f)); 
  data[54].mAabb = Aabb(Vector3(22.0484f, 10.6543f, 37.4793f), Vector3(23.7234f, 17.5565f, 39.3764f)); 
  data[55].mAabb = Aabb(Vector3(-35.1798f, -3.9308f, -17.455f), Vector3(-32.5557f, 2.83968f, -11.0835f)); 
  data[56].mAabb = Aabb(Vector3(-26.6396f, -12.7245f, 9.40689f), Vector3(-21.1624f, -11.6143f, 12.8321f)); 
  data[57].mAabb = Aabb(Vector3(0.493583f, -29.525f, -33.8682f), Vector3(8.28391f, -25.2766f, -33.0475f)); 
  data[58].mAabb = Aabb(Vector3(17.5297f, -25.2875f, 15.2704f), Vector3(21.5902f, -22.5417f, 21.9129f)); 
  data[59].mAabb = Aabb(Vector3(-2.20369f, -18.0771f, -14.1634f), Vector3(3.05013f, -12.5162f, -13.1887f)); 
  data[60].mAabb = Aabb(Vector3(8.22709f, 15.2252f, -38.633f), Vector3(10.5696f, 16.3913f, -34.0808f)); 
  data[61].mAabb = Aabb(Vector3(18.4643f, 23.6803f, -2.80812f), Vector3(21.4142f, 26.1197f, 1.21688f)); 
  data[62].mAabb = Aabb(Vector3(-0.615524f, -29.0032f, -11.1208f), Vector3(6.38156f, -26.8738f, -5.98042f)); 
  data[63].mAabb = Aabb(Vector3(-17.4551f, 23.1709f, 27.6189f), Vector3(-16.6145f, 24.5047f, 33.4382f)); 
  data[64].mAabb = Aabb(Vector3(-35.8697f, 7.24909f, -21.9036f), Vector3(-28.5608f, 11.731f, -20.3411f)); 
  data[65].mAabb = Aabb(Vector3(-34.1731f, -4.61909f, -19.6319f), Vector3(-29.7412f, -3.71253f, -13.0073f)); 
  data[66].mAabb = Aabb(Vector3(9.7685f, 27.4234f, 8.27498f), Vector3(14.6779f, 32.4275f, 10.5331f)); 
  data[67].mAabb = Aabb(Vector3(-13.5251f, -30.1796f, -11.1223f), Vector3(-8.17104f, -22.6339f, -9.47883f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest48(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(27); 
  data[0].mAabb = Aabb(Vector3(-11.9299f, 26.1454f, -23.4476f), Vector3(-6.1294f, 30.1917f, -21.4226f)); 
  data[1].mAabb = Aabb(Vector3(13.7957f, 12.4436f, 38.0818f), Vector3(16.3964f, 13.1454f, 43.0666f)); 
  data[2].mAabb = Aabb(Vector3(-40.9839f, 4.30377f, 7.94546f), Vector3(-34.2876f, 6.59664f, 10.6927f)); 
  data[3].mAabb = Aabb(Vector3(25.9821f, -14.7743f, 34.1157f), Vector3(29.6912f, -11.8002f, 41.4229f)); 
  data[4].mAabb = Aabb(Vector3(29.0443f, 7.96998f, -32.4755f), Vector3(29.59f, 14.0115f, -31.6137f)); 
  data[5].mAabb = Aabb(Vector3(-41.1594f, -20.9259f, -11.9888f), Vector3(-34.4311f, -19.1632f, -10.7649f)); 
  data[6].mAabb = Aabb(Vector3(-8.96364f, -12.6717f, -39.0519f), Vector3(-2.58467f, -5.91931f, -37.3157f)); 
  data[7].mAabb = Aabb(Vector3(12.3539f, -34.4367f, -18.3871f), Vector3(15.5829f, -27.3897f, -11.8669f)); 
  data[8].mAabb = Aabb(Vector3(17.3858f, 6.84863f, 39.6462f), Vector3(23.5626f, 9.67798f, 45.7638f)); 
  data[9].mAabb = Aabb(Vector3(40.8143f, 18.5083f, -3.86082f), Vector3(46.7906f, 19.6542f, -0.866334f)); 
  data[10].mAabb = Aabb(Vector3(-7.43177f, -10.7979f, -32.4962f), Vector3(-5.63897f, -4.29109f, -28.7735f)); 
  data[11].mAabb = Aabb(Vector3(-8.4575f, 20.5588f, 17.1666f), Vector3(-3.71557f, 26.2897f, 19.2429f)); 
  data[12].mAabb = Aabb(Vector3(29.3953f, 13.8151f, -13.1304f), Vector3(35.081f, 19.9138f, -11.1955f)); 
  data[13].mAabb = Aabb(Vector3(1.85059f, 13.9642f, 43.2656f), Vector3(8.71106f, 14.9698f, 50.3945f)); 
  data[14].mAabb = Aabb(Vector3(-43.8272f, -10.946f, 13.6257f), Vector3(-37.7507f, -5.96542f, 20.3424f)); 
  data[15].mAabb = Aabb(Vector3(24.7079f, 34.0554f, 14.3874f), Vector3(32.1487f, 36.7796f, 16.2382f)); 
  data[16].mAabb = Aabb(Vector3(40.8817f, -18.1115f, 5.45866f), Vector3(45.0233f, -14.0559f, 11.312f)); 
  data[17].mAabb = Aabb(Vector3(5.78052f, -20.6388f, -6.45741f), Vector3(9.7255f, -14.5366f, -2.86568f)); 
  data[18].mAabb = Aabb(Vector3(-1.42362f, -23.0885f, -17.9575f), Vector3(2.83449f, -19.6116f, -17.1155f)); 
  data[19].mAabb = Aabb(Vector3(13.8567f, 4.36331f, -39.9572f), Vector3(20.305f, 7.84928f, -38.1681f)); 
  data[20].mAabb = Aabb(Vector3(-0.0634001f, -24.5218f, -38.8188f), Vector3(1.64196f, -20.1394f, -38.086f)); 
  data[21].mAabb = Aabb(Vector3(22.3308f, -35.5504f, -9.51577f), Vector3(28.6237f, -30.3035f, -8.77157f)); 
  data[22].mAabb = Aabb(Vector3(3.00354f, 26.4551f, 12.4415f), Vector3(3.85002f, 29.2643f, 13.882f)); 
  data[23].mAabb = Aabb(Vector3(29.057f, -2.8135f, -20.1665f), Vector3(35.5011f, 1.94072f, -18.0204f)); 
  data[24].mAabb = Aabb(Vector3(19.8186f, 40.2719f, 0.254717f), Vector3(21.6114f, 42.4172f, 0.888514f)); 
  data[25].mAabb = Aabb(Vector3(-6.92036f, 18.1095f, 27.6464f), Vector3(0.948364f, 25.0076f, 29.8035f)); 
  data[26].mAabb = Aabb(Vector3(-0.0765039f, -13.9204f, -22.1415f), Vector3(3.69243f, -8.03481f, -18.8198f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest49(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(65); 
  data[0].mAabb = Aabb(Vector3(-28.7661f, 2.3077f, -21.5301f), Vector3(-23.0916f, 5.54758f, -14.7053f)); 
  data[1].mAabb = Aabb(Vector3(-11.6195f, -22.4192f, -30.877f), Vector3(-4.1785f, -18.2149f, -25.7352f)); 
  data[2].mAabb = Aabb(Vector3(-15.6502f, 6.11185f, -43.2056f), Vector3(-12.2747f, 11.9495f, -37.0514f)); 
  data[3].mAabb = Aabb(Vector3(0.791049f, 19.2104f, 6.09795f), Vector3(1.35596f, 24.1105f, 9.14347f)); 
  data[4].mAabb = Aabb(Vector3(-2.33319f, -12.4417f, -22.6172f), Vector3(3.03219f, -10.2989f, -16.7889f)); 
  data[5].mAabb = Aabb(Vector3(0.703323f, -18.7834f, 35.9837f), Vector3(3.34433f, -11.9341f, 39.3307f)); 
  data[6].mAabb = Aabb(Vector3(4.46596f, 4.38423f, 33.6934f), Vector3(5.49893f, 6.56228f, 40.7019f)); 
  data[7].mAabb = Aabb(Vector3(-15.3704f, 22.8979f, 20.7118f), Vector3(-11.8988f, 25.8511f, 28.1308f)); 
  data[8].mAabb = Aabb(Vector3(30.917f, 0.830217f, 17.2498f), Vector3(38.2525f, 5.64126f, 21.7275f)); 
  data[9].mAabb = Aabb(Vector3(-3.17295f, 32.6722f, -31.0922f), Vector3(1.4769f, 35.4835f, -23.1687f)); 
  data[10].mAabb = Aabb(Vector3(-3.61115f, -12.9158f, 26.7302f), Vector3(3.10366f, -8.46657f, 30.3402f)); 
  data[11].mAabb = Aabb(Vector3(13.6164f, -3.72103f, 20.515f), Vector3(20.6817f, 0.936246f, 24.1687f)); 
  data[12].mAabb = Aabb(Vector3(20.5944f, -8.70257f, -10.4098f), Vector3(25.8619f, -6.91719f, -8.02745f)); 
  data[13].mAabb = Aabb(Vector3(16.3459f, 15.5344f, -22.3198f), Vector3(18.0331f, 18.5901f, -16.4245f)); 
  data[14].mAabb = Aabb(Vector3(23.7969f, 3.09808f, 16.8809f), Vector3(28.5699f, 10.0569f, 20.6234f)); 
  data[15].mAabb = Aabb(Vector3(-5.0437f, -6.27042f, -9.7688f), Vector3(-2.69934f, 1.14788f, -5.04148f)); 
  data[16].mAabb = Aabb(Vector3(38.467f, 20.6917f, -5.04739f), Vector3(43.6658f, 25.1845f, 0.796726f)); 
  data[17].mAabb = Aabb(Vector3(-46.3628f, 6.80485f, 1.4292f), Vector3(-42.957f, 11.4081f, 5.70052f)); 
  data[18].mAabb = Aabb(Vector3(-16.8817f, -31.2586f, -30.3602f), Vector3(-10.7912f, -26.0851f, -25.2968f)); 
  data[19].mAabb = Aabb(Vector3(-16.5159f, 7.79442f, 13.8408f), Vector3(-9.03196f, 15.658f, 17.9428f)); 
  data[20].mAabb = Aabb(Vector3(6.50098f, 8.18004f, -45.9802f), Vector3(8.36151f, 10.4094f, -41.0727f)); 
  data[21].mAabb = Aabb(Vector3(32.1894f, -2.01809f, 26.7303f), Vector3(38.2548f, 1.7817f, 32.1639f)); 
  data[22].mAabb = Aabb(Vector3(-29.8031f, -33.8252f, 8.35097f), Vector3(-22.0098f, -31.2484f, 13.281f)); 
  data[23].mAabb = Aabb(Vector3(-39.7882f, -10.0757f, 1.86033f), Vector3(-34.8023f, -2.08781f, 4.74975f)); 
  data[24].mAabb = Aabb(Vector3(-22.0038f, -9.95819f, -37.0785f), Vector3(-16.637f, -6.26533f, -36.3165f)); 
  data[25].mAabb = Aabb(Vector3(-1.79425f, -8.47867f, -14.7114f), Vector3(-1.33371f, -5.54147f, -12.2351f)); 
  data[26].mAabb = Aabb(Vector3(1.1904f, -15.676f, 15.692f), Vector3(4.50775f, -12.3995f, 23.0692f)); 
  data[27].mAabb = Aabb(Vector3(-29.1291f, 32.189f, 19.1385f), Vector3(-23.9472f, 39.2371f, 22.9028f)); 
  data[28].mAabb = Aabb(Vector3(15.7465f, 10.3267f, 16.678f), Vector3(19.9988f, 13.7921f, 24.1726f)); 
  data[29].mAabb = Aabb(Vector3(-21.7719f, -34.0318f, 27.1987f), Vector3(-14.3738f, -31.4079f, 29.2944f)); 
  data[30].mAabb = Aabb(Vector3(-12.038f, -2.66692f, -14.3198f), Vector3(-7.21693f, 2.89584f, -9.8771f)); 
  data[31].mAabb = Aabb(Vector3(-12.0798f, -26.4628f, 39.6435f), Vector3(-11.5951f, -24.4543f, 41.672f)); 
  data[32].mAabb = Aabb(Vector3(-26.7893f, -10.7124f, -41.3273f), Vector3(-25.125f, -8.06583f, -35.939f)); 
  data[33].mAabb = Aabb(Vector3(19.0113f, -21.2246f, 20.4998f), Vector3(22.9331f, -18.5205f, 27.1285f)); 
  data[34].mAabb = Aabb(Vector3(-42.8096f, -17.6501f, -9.07485f), Vector3(-40.8611f, -13.6349f, -1.22979f)); 
  data[35].mAabb = Aabb(Vector3(-14.0385f, 34.151f, -23.1386f), Vector3(-6.04983f, 38.2878f, -18.913f)); 
  data[36].mAabb = Aabb(Vector3(-4.60375f, -13.9092f, -15.2366f), Vector3(-1.50489f, -6.27288f, -9.8592f)); 
  data[37].mAabb = Aabb(Vector3(-27.392f, 32.1005f, -6.91147f), Vector3(-21.1021f, 36.2301f, -6.07078f)); 
  data[38].mAabb = Aabb(Vector3(22.607f, -0.948212f, 1.14369f), Vector3(23.9524f, 6.81335f, 2.95203f)); 
  data[39].mAabb = Aabb(Vector3(-43.5113f, -6.37562f, 1.99267f), Vector3(-40.6889f, -3.02766f, 7.89686f)); 
  data[40].mAabb = Aabb(Vector3(2.87623f, 25.4685f, -5.09054f), Vector3(4.16804f, 32.5356f, -0.844034f)); 
  data[41].mAabb = Aabb(Vector3(3.57661f, 33.5214f, -7.43657f), Vector3(9.27831f, 37.6997f, -3.16896f)); 
  data[42].mAabb = Aabb(Vector3(-17.3783f, 43.351f, -6.06227f), Vector3(-14.8563f, 49.8362f, 0.978885f)); 
  data[43].mAabb = Aabb(Vector3(23.581f, -9.10096f, -40.7082f), Vector3(24.2315f, -7.19961f, -36.1755f)); 
  data[44].mAabb = Aabb(Vector3(21.5449f, -21.9015f, -33.7942f), Vector3(25.4012f, -19.0142f, -28.411f)); 
  data[45].mAabb = Aabb(Vector3(34.9024f, 12.1261f, 15.9007f), Vector3(42.4768f, 19.9526f, 22.6023f)); 
  data[46].mAabb = Aabb(Vector3(-23.9087f, 25.0966f, -15.9374f), Vector3(-22.6412f, 30.8795f, -10.8772f)); 
  data[47].mAabb = Aabb(Vector3(-2.21472f, 8.6688f, 17.4144f), Vector3(-1.69086f, 12.1135f, 19.3775f)); 
  data[48].mAabb = Aabb(Vector3(-10.3069f, -41.6934f, 15.1109f), Vector3(-5.43374f, -37.2994f, 21.3899f)); 
  data[49].mAabb = Aabb(Vector3(-4.94198f, 43.8337f, -11.4293f), Vector3(2.22675f, 45.9014f, -4.19728f)); 
  data[50].mAabb = Aabb(Vector3(42.2616f, -3.28194f, -6.22193f), Vector3(47.6755f, -0.609389f, -2.15216f)); 
  data[51].mAabb = Aabb(Vector3(-4.168f, 19.1989f, -37.7158f), Vector3(-3.28533f, 22.2089f, -33.6711f)); 
  data[52].mAabb = Aabb(Vector3(-40.2936f, 11.2504f, 5.74675f), Vector3(-33.464f, 11.6662f, 11.7284f)); 
  data[53].mAabb = Aabb(Vector3(-30.8567f, -8.67541f, 16.3734f), Vector3(-22.8903f, -1.49184f, 21.2078f)); 
  data[54].mAabb = Aabb(Vector3(-8.93822f, 34.0417f, -16.5036f), Vector3(-6.78846f, 41.6931f, -13.3173f)); 
  data[55].mAabb = Aabb(Vector3(10.7432f, -15.0195f, 33.994f), Vector3(15.85f, -14.2609f, 36.1825f)); 
  data[56].mAabb = Aabb(Vector3(42.831f, -6.49663f, -1.74126f), Vector3(44.538f, -5.65154f, 0.354694f)); 
  data[57].mAabb = Aabb(Vector3(-33.1799f, -31.264f, 7.31008f), Vector3(-26.3835f, -23.9002f, 8.61929f)); 
  data[58].mAabb = Aabb(Vector3(1.02807f, 2.09417f, 27.6638f), Vector3(7.11386f, 8.99755f, 34.6799f)); 
  data[59].mAabb = Aabb(Vector3(-22.1273f, 31.546f, 17.3364f), Vector3(-20.7096f, 33.0227f, 22.6183f)); 
  data[60].mAabb = Aabb(Vector3(19.7944f, -4.38315f, 31.9847f), Vector3(22.2169f, -1.26132f, 39.7208f)); 
  data[61].mAabb = Aabb(Vector3(23.7789f, 3.44173f, -16.5629f), Vector3(30.2901f, 7.31898f, -15.1923f)); 
  data[62].mAabb = Aabb(Vector3(30.978f, 7.05893f, -26.8897f), Vector3(36.6447f, 8.51542f, -19.9088f)); 
  data[63].mAabb = Aabb(Vector3(-8.2987f, -39.6242f, -21.5925f), Vector3(-7.56749f, -32.2038f, -13.6416f)); 
  data[64].mAabb = Aabb(Vector3(-15.8673f, -29.8375f, 30.7618f), Vector3(-12.8518f, -24.5721f, 35.5877f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void AabbTreeSelfQueryFuzzTest50(const std::string& testName, int debuggingIndex, FILE* file = NULL)
{
  PrintTestHeader(file, testName);
  
  std::vector<SpatialPartitionData> data; 
  data.resize(64); 
  data[0].mAabb = Aabb(Vector3(-35.7951f, -15.2081f, -13.0109f), Vector3(-33.5085f, -8.05654f, -11.0108f)); 
  data[1].mAabb = Aabb(Vector3(-16.2322f, -25.0191f, 8.15692f), Vector3(-10.8369f, -24.3001f, 12.99f)); 
  data[2].mAabb = Aabb(Vector3(40.2259f, 8.12043f, -13.5301f), Vector3(41.8021f, 13.4751f, -8.2037f)); 
  data[3].mAabb = Aabb(Vector3(25.9993f, 7.09807f, -0.941658f), Vector3(26.9511f, 13.9544f, 3.778f)); 
  data[4].mAabb = Aabb(Vector3(-15.7226f, 18.5132f, 35.5389f), Vector3(-12.2564f, 21.0872f, 43.0757f)); 
  data[5].mAabb = Aabb(Vector3(-44.12f, 9.99829f, 8.89473f), Vector3(-37.8268f, 17.8958f, 16.1033f)); 
  data[6].mAabb = Aabb(Vector3(-0.262443f, -9.23563f, 23.5669f), Vector3(4.99393f, -5.25169f, 29.2598f)); 
  data[7].mAabb = Aabb(Vector3(13.1312f, 23.9023f, 25.7502f), Vector3(20.4954f, 25.4476f, 30.1238f)); 
  data[8].mAabb = Aabb(Vector3(-35.2045f, 3.48061f, 3.15596f), Vector3(-29.6039f, 4.0214f, 4.72378f)); 
  data[9].mAabb = Aabb(Vector3(22.4572f, 22.5143f, 11.3103f), Vector3(25.8606f, 29.712f, 15.7997f)); 
  data[10].mAabb = Aabb(Vector3(-21.5471f, -18.7255f, -9.22656f), Vector3(-18.8254f, -11.134f, -2.7518f)); 
  data[11].mAabb = Aabb(Vector3(24.3328f, -13.5381f, 33.5385f), Vector3(30.861f, -11.9037f, 37.1541f)); 
  data[12].mAabb = Aabb(Vector3(-17.4109f, -10.1506f, 40.6037f), Vector3(-10.4607f, -2.84295f, 48.4126f)); 
  data[13].mAabb = Aabb(Vector3(-12.3177f, -31.4593f, -1.50584f), Vector3(-9.03977f, -24.6174f, 0.874007f)); 
  data[14].mAabb = Aabb(Vector3(-33.9896f, -0.988262f, -16.3531f), Vector3(-28.3909f, 2.72315f, -15.7274f)); 
  data[15].mAabb = Aabb(Vector3(10.5263f, 5.62257f, -34.0845f), Vector3(10.9518f, 6.65136f, -28.1103f)); 
  data[16].mAabb = Aabb(Vector3(-24.8678f, 37.0768f, -10.5929f), Vector3(-20.4816f, 44.7288f, -9.23472f)); 
  data[17].mAabb = Aabb(Vector3(-19.8101f, -44.2538f, -4.08699f), Vector3(-12.2429f, -40.1483f, -1.71387f)); 
  data[18].mAabb = Aabb(Vector3(-35.2654f, 33.4931f, 7.25174f), Vector3(-27.8413f, 34.3913f, 9.84242f)); 
  data[19].mAabb = Aabb(Vector3(16.6155f, -34.6059f, -7.80016f), Vector3(19.4221f, -33.5266f, -6.60809f)); 
  data[20].mAabb = Aabb(Vector3(-8.2994f, 8.18206f, 44.0242f), Vector3(-1.52265f, 8.88938f, 50.425f)); 
  data[21].mAabb = Aabb(Vector3(14.6541f, 42.4921f, -17.5993f), Vector3(22.3004f, 43.9305f, -11.0086f)); 
  data[22].mAabb = Aabb(Vector3(-1.38717f, -15.2467f, -9.20575f), Vector3(1.82094f, -14.8126f, -5.22389f)); 
  data[23].mAabb = Aabb(Vector3(-28.0147f, 23.057f, -18.2597f), Vector3(-22.223f, 27.4343f, -11.6181f)); 
  data[24].mAabb = Aabb(Vector3(6.99381f, -15.3977f, 18.3671f), Vector3(14.522f, -9.05401f, 19.4249f)); 
  data[25].mAabb = Aabb(Vector3(-4.40016f, -41.0175f, 7.11835f), Vector3(2.39259f, -40.283f, 11.6893f)); 
  data[26].mAabb = Aabb(Vector3(-2.42842f, -50.8258f, 12.745f), Vector3(3.72185f, -43.8396f, 15.1503f)); 
  data[27].mAabb = Aabb(Vector3(-43.1505f, -4.34069f, -12.9572f), Vector3(-38.7282f, -2.44166f, -5.8699f)); 
  data[28].mAabb = Aabb(Vector3(18.2398f, -12.4275f, 20.1314f), Vector3(25.7133f, -8.32545f, 21.9214f)); 
  data[29].mAabb = Aabb(Vector3(-34.2028f, -34.1476f, -15.5351f), Vector3(-32.6897f, -27.9952f, -12.7266f)); 
  data[30].mAabb = Aabb(Vector3(27.979f, 22.922f, -20.0632f), Vector3(28.6803f, 25.1364f, -15.4249f)); 
  data[31].mAabb = Aabb(Vector3(-16.3653f, 24.6574f, -7.88009f), Vector3(-13.2027f, 31.3604f, -2.27905f)); 
  data[32].mAabb = Aabb(Vector3(-27.9019f, 26.6203f, 1.77429f), Vector3(-23.3912f, 28.8967f, 2.32157f)); 
  data[33].mAabb = Aabb(Vector3(17.7072f, -2.55244f, 25.9657f), Vector3(23.3251f, 2.53276f, 31.8092f)); 
  data[34].mAabb = Aabb(Vector3(-21.8424f, 12.9816f, 12.8923f), Vector3(-15.0629f, 15.912f, 17.047f)); 
  data[35].mAabb = Aabb(Vector3(38.57f, 6.88143f, 13.3408f), Vector3(42.9225f, 11.8544f, 16.5665f)); 
  data[36].mAabb = Aabb(Vector3(20.0512f, 16.1578f, -1.73814f), Vector3(23.244f, 17.705f, 3.73394f)); 
  data[37].mAabb = Aabb(Vector3(-17.8516f, 5.40625f, -43.8479f), Vector3(-14.8014f, 8.68788f, -41.748f)); 
  data[38].mAabb = Aabb(Vector3(-7.06739f, 16.0776f, 15.5159f), Vector3(-2.73507f, 22.2815f, 16.9183f)); 
  data[39].mAabb = Aabb(Vector3(7.50821f, -5.62647f, 8.54168f), Vector3(8.91483f, 1.18808f, 14.7312f)); 
  data[40].mAabb = Aabb(Vector3(-39.8716f, -6.94162f, 17.8243f), Vector3(-33.6049f, -1.77803f, 20.5736f)); 
  data[41].mAabb = Aabb(Vector3(-18.2613f, 18.1921f, 12.1001f), Vector3(-14.0006f, 25.9086f, 19.7759f)); 
  data[42].mAabb = Aabb(Vector3(24.3472f, 7.31865f, -28.7694f), Vector3(31.3998f, 11.1398f, -22.739f)); 
  data[43].mAabb = Aabb(Vector3(-15.1681f, -12.0023f, -35.6712f), Vector3(-7.99956f, -5.20138f, -28.2019f)); 
  data[44].mAabb = Aabb(Vector3(36.2568f, -19.5535f, 9.69373f), Vector3(39.6182f, -12.2774f, 16.5039f)); 
  data[45].mAabb = Aabb(Vector3(-8.4437f, -27.7215f, 15.8005f), Vector3(-7.50931f, -24.9123f, 16.6813f)); 
  data[46].mAabb = Aabb(Vector3(20.8036f, 4.77484f, -42.0482f), Vector3(24.8446f, 10.0683f, -36.1851f)); 
  data[47].mAabb = Aabb(Vector3(6.06407f, -10.4525f, -3.58867f), Vector3(9.4735f, -4.18577f, -0.0834479f)); 
  data[48].mAabb = Aabb(Vector3(-2.8512f, 24.8421f, -32.3684f), Vector3(-0.422883f, 29.8624f, -31.0881f)); 
  data[49].mAabb = Aabb(Vector3(0.713785f, -27.5227f, -1.77418f), Vector3(2.18234f, -23.9265f, 4.15297f)); 
  data[50].mAabb = Aabb(Vector3(-24.6854f, 4.64723f, -6.40291f), Vector3(-19.6636f, 5.66095f, -4.97356f)); 
  data[51].mAabb = Aabb(Vector3(-46.1741f, 11.932f, 13.132f), Vector3(-44.6909f, 18.0867f, 13.5974f)); 
  data[52].mAabb = Aabb(Vector3(-11.814f, -33.3995f, 15.6313f), Vector3(-6.68425f, -29.4506f, 21.9165f)); 
  data[53].mAabb = Aabb(Vector3(5.03413f, -1.93799f, -40.16f), Vector3(7.83193f, 1.51226f, -33.5934f)); 
  data[54].mAabb = Aabb(Vector3(17.0221f, 10.1059f, -15.4163f), Vector3(23.7272f, 12.4709f, -8.17801f)); 
  data[55].mAabb = Aabb(Vector3(9.49758f, 19.9219f, -26.8909f), Vector3(10.8226f, 20.7317f, -24.4322f)); 
  data[56].mAabb = Aabb(Vector3(-8.98668f, -16.967f, 35.5956f), Vector3(-6.80908f, -15.3523f, 41.6944f)); 
  data[57].mAabb = Aabb(Vector3(8.76893f, 8.20325f, 33.2351f), Vector3(13.74f, 14.5541f, 34.5163f)); 
  data[58].mAabb = Aabb(Vector3(2.18659f, 31.0876f, 7.91959f), Vector3(4.08794f, 37.8657f, 12.5242f)); 
  data[59].mAabb = Aabb(Vector3(-6.5934f, -41.9791f, -25.3181f), Vector3(-5.78959f, -38.8527f, -20.8184f)); 
  data[60].mAabb = Aabb(Vector3(3.70479f, -45.9915f, 11.2126f), Vector3(7.52777f, -43.4878f, 14.3593f)); 
  data[61].mAabb = Aabb(Vector3(-5.67514f, 31.4918f, 32.3956f), Vector3(2.00316f, 38.3964f, 34.817f)); 
  data[62].mAabb = Aabb(Vector3(9.17301f, -11.4112f, -38.0979f), Vector3(10.1603f, -9.67176f, -32.4709f)); 
  data[63].mAabb = Aabb(Vector3(-5.4997f, 24.6384f, -40.2318f), Vector3(-3.27456f, 26.8798f, -34.8098f)); 
   
  for(size_t i = 0; i < data.size(); ++i) 
    data[i].mClientData = (void*)i; 
   
  std::vector<SpatialPartitionKey> keys; 
  keys.resize(data.size()); 
   
  DynamicAabbTree spatialPartition; 
  for(size_t i = 0; i < data.size(); ++i) 
    spatialPartition.InsertData(keys[i], data[i]); 
  Application::mStatistics.Clear();
  PrintSpatialPartitionSelfQuery(spatialPartition, file);
  if(file != NULL) 
  { 
    fprintf(file, "  Statistics (%s): %d", "mAabbAabbTests", static_cast<int>(Application::mStatistics.mAabbAabbTests)); 
  } 
}

void RegisterAabbTreeStructureTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DynamicAabbTreeStructure1, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructure2, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructure3, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructure4, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructure5, list);
}

void RegisterAabbTreeRayCastTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastTest1, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastTest2, list);
}

void RegisterAabbTreeFrustumCastTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest1, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest2, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest3, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest4, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest5, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest6, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest7, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest8, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest9, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest10, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastTest11, list);
}

void RegisterAabbTreeSelfQueryTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DynamicAabbTreeSelfQuery1, list);
  DeclareSimpleUnitTest(DynamicAabbTreeSelfQuery2, list);
}

void RegisterDynamicAabbTreeStructureFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest1, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest2, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest3, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest4, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest5, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest6, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest7, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest8, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest9, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest10, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest11, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest12, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest13, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest14, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest15, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest16, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest17, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest18, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest19, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest20, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest21, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest22, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest23, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest24, list);
  DeclareSimpleUnitTest(DynamicAabbTreeStructureFuzzTest25, list);
}

void RegisterDynamicAabbTreeRayCastFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest1, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest2, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest3, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest4, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest5, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest6, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest7, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest8, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest9, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest10, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest11, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest12, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest13, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest14, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest15, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest16, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest17, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest18, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest19, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest20, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest21, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest22, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest23, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest24, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest25, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest26, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest27, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest28, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest29, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest30, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest31, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest32, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest33, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest34, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest35, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest36, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest37, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest38, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest39, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest40, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest41, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest42, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest43, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest44, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest45, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest46, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest47, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest48, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest49, list);
  DeclareSimpleUnitTest(DynamicAabbTreeRayCastFuzzTest50, list);
}

void RegisterDynamicAabbTreeFrustumCastFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest1, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest2, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest3, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest4, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest5, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest6, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest7, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest8, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest9, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest10, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest11, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest12, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest13, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest14, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest15, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest16, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest17, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest18, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest19, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest20, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest21, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest22, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest23, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest24, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest25, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest26, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest27, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest28, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest29, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest30, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest31, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest32, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest33, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest34, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest35, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest36, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest37, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest38, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest39, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest40, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest41, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest42, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest43, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest44, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest45, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest46, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest47, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest48, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest49, list);
  DeclareSimpleUnitTest(DynamicAabbTreeFrustumCastFuzzTest50, list);
}

void RegisterAabbTreeSelfQueryFuzzTests(AssignmentUnitTestList& list)
{
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest1, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest2, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest3, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest4, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest5, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest6, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest7, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest8, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest9, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest10, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest11, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest12, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest13, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest14, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest15, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest16, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest17, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest18, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest19, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest20, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest21, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest22, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest23, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest24, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest25, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest26, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest27, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest28, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest29, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest30, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest31, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest32, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest33, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest34, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest35, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest36, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest37, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest38, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest39, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest40, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest41, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest42, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest43, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest44, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest45, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest46, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest47, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest48, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest49, list);
  DeclareSimpleUnitTest(AabbTreeSelfQueryFuzzTest50, list);
}

void InitializeAssignment3Tests()
{
  mTestFns.push_back(AssignmentUnitTestList());
  AssignmentUnitTestList& list = mTestFns[2];
  
  // Add Aabb Tree Spatial Partition Tests
  RegisterAabbTreeStructureTests(list);
  RegisterAabbTreeRayCastTests(list);
  RegisterAabbTreeFrustumCastTests(list);
  RegisterAabbTreeSelfQueryTests(list);
  RegisterDynamicAabbTreeStructureFuzzTests(list);
  RegisterDynamicAabbTreeRayCastFuzzTests(list);
  RegisterDynamicAabbTreeFrustumCastFuzzTests(list);
  RegisterAabbTreeSelfQueryFuzzTests(list);
}
