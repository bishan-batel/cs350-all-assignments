/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: SimpleNSquared.hpp
Purpose: NSquared Partition Structure & Sphere Bounding
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_2
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#pragma once

#include <unordered_map>

#include "SpatialPartition.hpp"

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
// A very bad, brute force spatial partition that is used for assignment 1 (before you get to implement something better).
// Do not implement your spatial partitions this way, make sure that update and remove are O(1) (no searches).
class NSquaredSpatialPartition : public SpatialPartition
{
public:
  NSquaredSpatialPartition();

  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const override;
  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  std::vector<void*> mData;
};

/******Student:Assignment2******/
// Implement the n-squared sphere spatial partition
//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
class BoundingSphereSpatialPartition : public SpatialPartition
{
public:
  BoundingSphereSpatialPartition();

  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  // Add your implementation here
private:
  
  std::unordered_map<void*, SpatialPartitionData> objects;
};
