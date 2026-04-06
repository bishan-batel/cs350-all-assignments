/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: SimpleNSquared.cpp
Purpose: NSquared Partition Structure & Sphere Bounding
Language: MSVC, C++
Platform: Windows
Project: kishan.patel_CS350_2
Author: Kishan S Patel kishan.patel 006624
End Header -------------------------------------------------------*/
#include "Precompiled.hpp"

#include <unordered_set>

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
    mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
    // Doing this lazily (and bad, but it's n-squared...).
    // Just store as the key what the client data is so we can look it up later.
    key.mVoidKey = data.mClientData;
    mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
    // Nothing to do here, update doesn't do anything
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
    // Find the key data and remove it
    for (size_t i = 0; i < mData.size(); ++i)
    {
        if (mData[i] == key.mVoidKey)
        {
            mData[i] = mData.back();
            mData.pop_back();
            break;
        }
    }
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
    // Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        CastResult result;
        result.mClientData = mData[i];
        results.AddResult(result);
    }
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        CastResult result;
        result.mClientData = mData[i];
        results.AddResult(result);
    }
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        for (size_t j = i + 1; j < mData.size(); ++j)
        {
            results.AddResult(QueryResult(mData[i], mData[j]));
        }
    }
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
    data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
    for (size_t i = 0; i < mData.size(); ++i)
    {
        SpatialPartitionQueryData data;
        data.mClientData = mData[i];
        results.push_back(data);
    }
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
    mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
    key.mVoidKey = data.mClientData;
    objects.insert_or_assign(key.mVoidKey, data);
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
    objects[key.mVoidKey] = data;
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
    objects.erase(key.mVoidKey);
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Matrix4& transform, const Vector4& color,
                                               int bitMask)
{
    for (const auto& pair : objects)
    {
        const auto& data = std::get<1>(pair);
        gDebugDrawer->DrawSphere(data.mBoundingSphere).SetTransform(transform).Color(color).SetMaskBit(bitMask);
    }
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
    float t;

    for (const auto& pair : objects)
    {
        const auto& data = std::get<1>(pair);

        if (!RaySphere(
            ray.mStart, ray.mDirection, data.mBoundingSphere.GetCenter(), data.mBoundingSphere.GetRadius(),
            t
        ))
        {
            continue;
        }

        results.AddResult({data.mClientData, t});
    }
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
    size_t last_axis;

    for (const auto& pair : objects)
    {
        const auto& data = std::get<1>(pair);

        IntersectionType::Type type = FrustumSphere(
            frustum.GetPlanes(),
            data.mBoundingSphere.GetCenter(),
            data.mBoundingSphere.GetRadius(),
            last_axis
        );

        switch (type)
        {
        case IntersectionType::Inside:
        case IntersectionType::Overlaps:
            results.AddResult(CastResult{data.mClientData});
        case IntersectionType::Coplanar:
        case IntersectionType::Outside:
        case IntersectionType::NotImplemented:
        default:
            break;
        }
    }
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
    // algo pulled from our GAM project which was pulled from some stack overflow comment probaly
    struct hash final
    {
        size_t operator()(const QueryResult& key) const
        {
            const size_t hash1 = std::hash<void*>()(key.mClientData0);
            const size_t hash2 = std::hash<void*>()(key.mClientData1);
            return hash1 ^ (0x9e3779b9 + (hash2 << 6) + (hash2 >> 2));
        }
    };

    // cache of pairs we've looked at
    std::unordered_set<QueryResult, hash> searched{};

    for (const std::pair<void* const, SpatialPartitionData>& pair1 : objects)
    {
        const SpatialPartitionData& data1 = std::get<1>(pair1);
        const Sphere& sphere1 = data1.mBoundingSphere;

        for (const std::pair<void* const, SpatialPartitionData>& pair2 : objects)
        {
            const SpatialPartitionData& data2 = std::get<1>(pair2);
            const Sphere& sphere2 = data2.mBoundingSphere;

            if (data1.mClientData == data2.mClientData)
            {
                continue;
            }

            QueryResult result{data1.mClientData, data2.mClientData};

            if (searched.find(result) != searched.end())
            {
                continue;
            }

            searched.emplace(data1.mClientData, data2.mClientData);

            // put if spheres overlap
            if (SphereSphere(sphere1.GetCenter(), sphere1.GetRadius(), sphere2.GetCenter(), sphere2.GetRadius()))
            {
                results.AddResult(result);
            }
        }
    }
}

void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
    for (const auto& pair : objects)
    {
        const SpatialPartitionData& obj = std::get<1>(pair);
        SpatialPartitionQueryData data;
        data.mClientData = obj.mClientData;
        data.mBoundingSphere = obj.mBoundingSphere;
        results.push_back(data);
    }
}
