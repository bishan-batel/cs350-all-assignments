///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include "Model.hpp"

#include "Application.hpp"
#include "Main/Support.hpp"
#include "Geometry.hpp"
#include "DebugDraw.hpp"
#include "BspTree.hpp"
#include "SimplePropertyBinding.hpp"

//-----------------------------------------------------------------------------Model
Model::Model()
{
  mOverlap = 0;
  mMidPhase = NULL;
  mMidphaseDrawLevel = 0;
}

void Model::TransformUpdate(TransformUpdateFlags::Enum flags)
{
  bool recompute = (flags & TransformUpdateFlags::Scale) != 0;
  UpdateBoundingVolumes(recompute);
}

void Model::DebugDraw()
{
  if(mMidPhase)
    mMidPhase->DebugDraw(mMidphaseDrawLevel, mOwner->has(Transform)->GetTransform());

  //mBoundingSphere.DebugDraw();
}

void Model::DisplayProperties(TwBar* bar)
{
  std::string name = mOwner->mName;
  std::string groupName = "group=" + name + ".Model";

  TwAddVarRW(bar, (name + ".MidphaseDrawLevel").c_str(), TW_TYPE_INT32, &mMidphaseDrawLevel, (groupName + " label=MidphaseDrawLevel").c_str());
  BindSimpleProperty(bar, name, Model, MeshType, mOwner->mApplication->mMeshTypesEnum, int);

  DeclareObjectComponentGroup(bar, name, Model);
}

void Model::UpdateBoundingVolumes(bool recompute)
{
  if(recompute)
  {
    ComputeBoundingSphere();
    ComputeAabb();
  }

  UpdateAabb();
  UpdateBoundingSphere();

  mOwner->mApplication->UpdateGameObject(mOwner);
}

void Model::ComputeAabb()
{
  mLocalAabb = Aabb();

  for(size_t i = 0; i < mMesh->mVertices.size(); ++i)
  {
    Vector3 point = mMesh->mVertices[i];
    mLocalAabb.mMin = Math::Min(mLocalAabb.mMin, point);
    mLocalAabb.mMax = Math::Max(mLocalAabb.mMax, point);
  }
}

void Model::ComputeBoundingSphere()
{
  int boundingSphereType = mOwner->mApplication->mBoundSphereType;
  if(boundingSphereType == BoundingSphereType::Centroid)
    mLocalSphere.ComputeCentroid(mMesh->mVertices);
  else if(boundingSphereType == BoundingSphereType::RitterSphere)
    mLocalSphere.ComputeRitter(mMesh->mVertices);
  else if(boundingSphereType == BoundingSphereType::PCA)
    mLocalSphere.ComputePCA(mMesh->mVertices);
}

void Model::UpdateAabb()
{
  mAabb = mLocalAabb;

  Transform* transform = mOwner->has(Transform);

  Math::Matrix3 rotation = Math::ToMatrix3(transform->mRotation);
  Math::Vector3 scale = transform->mScale;
  Math::Vector3 translation = transform->mTranslation;

  mAabb.Transform(scale, rotation, translation);
}

void Model::UpdateBoundingSphere()
{
  Vector3 scale = mOwner->has(Transform)->mScale;
  float maxScale = Math::Max(Math::Max(scale.x, scale.y), scale.z);
  mBoundingSphere.mCenter = mOwner->has(Transform)->mTranslation + mLocalSphere.mCenter * maxScale;

  mBoundingSphere.mRadius = maxScale * mLocalSphere.mRadius;
}

void Model::CheckTriangle(const Ray& ray, const Triangle& tri, float& minT)
{
  float t;
  if(RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], t, 0) == false)
    return;

  if(t < minT)
    minT = t;
}

bool Model::CastRayMidphase(const Ray& localRay, CastResult& castInfo)
{
  CastResults results;
  mMidPhase->CastRay(localRay, results);

  float minT = Math::PositiveMax();
  for(size_t i = 0; i < results.mResults.size(); ++i)
  {
    CastResult& result = results.mResults[i];
    size_t triangleIndex = (size_t)result.mClientData;

    Triangle tri = mMesh->TriangleAt(triangleIndex);
    CheckTriangle(localRay, tri, minT);
  }

  if(minT >= Math::PositiveMax())
    return false;

  castInfo.mTime = minT;
  return true;
}

bool Model::CastRay(const Ray& worldRay, CastResult& castInfo)
{
  Math::Matrix4 toWorldMat = mOwner->has(Transform)->GetTransform();
  Math::Matrix4 toLocalMat = toWorldMat.Inverted();

  Ray localRay = worldRay.Transform(toLocalMat);

  if(mMidPhase != NULL)
    return CastRayMidphase(worldRay, castInfo);

  float minT = Math::PositiveMax();
  for(size_t i = 0; i < mMesh->TriangleCount(); ++i)
  {
    Triangle tri = mMesh->TriangleAt(i);
    CheckTriangle(localRay, tri, minT);
  }

  if(minT >= Math::PositiveMax())
    return false;

  castInfo.mTime = minT;
  return true;
}

void Model::SetMidPhase(SpatialPartition* midPhase)
{
  if(mMidPhase != NULL)
    delete mMidPhase;

  mMidPhase = midPhase;

  SpatialPartitionKey dummyKey;
  for(size_t i = 0; i < mMesh->TriangleCount(); ++i)
  {
    Triangle tri = mMesh->TriangleAt(i);
    Aabb aabb;
    aabb.Expand(tri.mPoints[0]);
    aabb.Expand(tri.mPoints[1]);
    aabb.Expand(tri.mPoints[2]);

    SpatialPartitionData data;
    //fix this later!
    //data.mBoundingSphere = Sphere(aabb;
    data.mAabb = aabb;
    data.mClientData = (void*)i;

    mMidPhase->InsertData(dummyKey, data);
  }
}

int Model::GetMeshType()
{
  return mMesh->mType;
}

void Model::SetMeshType(const int& meshIndex)
{
  mMesh = mOwner->mApplication->mMeshes[meshIndex];
  UpdateBoundingVolumes(true);
}

void Model::Draw()
{
  glPushMatrix();

  Transform* transform = mOwner->has(Transform);
  Vector3 axis;
  float radians;
  Math::ToAxisAngle(transform->mRotation, &axis, &radians);

  glTranslatef(transform->mTranslation.x, transform->mTranslation.y, transform->mTranslation.z);
  glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
  glScalef(transform->mScale.x, transform->mScale.y, transform->mScale.z);

  if(mOverlap == 0)
  {
    float color[3] = {1, 1, 1};
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  color);
  }
  else if(mOverlap == 1)
  {
    float color[3] = {1, 0, 0};
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  color);
  }
  else
  {
    float color[3] = {0, 0, 1};
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  color);
  }

  glDisable(GL_CULL_FACE);
  glBegin(GL_TRIANGLES);
  
  for(size_t i = 0; i < mMesh->TriangleCount(); ++i)
  {
    Triangle tri = mMesh->TriangleAt(i);
    DrawTriangle(tri);
  }
  glEnd();

  glPopMatrix();
}

void Model::DrawTriangle(Triangle& tri)
{
  Math::Vector3 normal = Math::Cross(tri.mPoints[1] - tri.mPoints[0], tri.mPoints[2] - tri.mPoints[0]);
  normal.AttemptNormalize();

  glNormal3f(normal.x, normal.y, normal.z);
  for(int i = 0; i < 3; ++i)
    glVertex3f(tri.mPoints[i].x, tri.mPoints[i].y, tri.mPoints[i].z);
}

std::vector<Triangle> Model::GetWorldTriangles()
{
  std::vector<Triangle> results;
  Math::Matrix4 worldMat = mOwner->has(Transform)->GetTransform();
  for(size_t i = 0; i < mMesh->mIndices.size(); i += 3)
  {
    Vector3 p0 = Math::TransformPoint(worldMat, mMesh->mVertices[mMesh->mIndices[i + 0]]);
    Vector3 p1 = Math::TransformPoint(worldMat, mMesh->mVertices[mMesh->mIndices[i + 1]]);
    Vector3 p2 = Math::TransformPoint(worldMat, mMesh->mVertices[mMesh->mIndices[i + 2]]);
    results.push_back(Triangle(p0, p1, p2));
  }
  
  return results;
}

void Model::SetWorldTriangles(const std::vector<Triangle>& tris)
{
  if(mMesh->mDynamic == true)
    delete mMesh;

  mMesh = new Mesh();
  mMesh->mDynamic = true;
  mMesh->mType = -1;
  
  Math::Matrix4 localMat = mOwner->has(Transform)->GetTransform().Inverted();

  mMesh->mIndices.clear();
  mMesh->mVertices.clear();
  mMesh->mIndices.reserve(tris.size() * 3);
  mMesh->mVertices.reserve(tris.size() * 3);

  for(size_t i = 0; i < tris.size(); ++i)
  {
    mMesh->mIndices.push_back(i * 3 + 0);
    mMesh->mIndices.push_back(i * 3 + 1);
    mMesh->mIndices.push_back(i * 3 + 2);

    const Triangle& tri = tris[i];
    Vector3 p0 = Math::TransformPoint(localMat, tri.mPoints[0]);
    Vector3 p1 = Math::TransformPoint(localMat, tri.mPoints[1]);
    Vector3 p2 = Math::TransformPoint(localMat, tri.mPoints[2]);

    mMesh->mVertices.push_back(p0);
    mMesh->mVertices.push_back(p1);
    mMesh->mVertices.push_back(p2);
  }
}
