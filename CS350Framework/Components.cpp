///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include "Components.hpp"

#include "Model.hpp"
#include "Application.hpp"
#include "SimplePropertyBinding.hpp"

//-----------------------------------------------------------------------------GameObject
GameObject::GameObject(Application* application)
{
  mApplication = application;

  mTransform = nullptr;
  mModel = nullptr;
}

GameObject::~GameObject()
{
  for(size_t i = 0; i < mGameplayComponents.size(); ++i)
    delete mGameplayComponents[i];
  mGameplayComponents.clear();

  delete mModel;
  delete mTransform;
  mModel = NULL;
  mTransform = NULL;
}

void GameObject::Update(float dt)
{
  if(mTransform != NULL)
    mTransform->Update(dt);
  if(mModel != NULL)
    mModel->Update(dt);

  for(size_t i = 0; i < mGameplayComponents.size(); ++i)
    mGameplayComponents[i]->Update(dt);
}

void GameObject::TransformUpdate(TransformUpdateFlags::Enum flags)
{
  if(mTransform != NULL)
    mTransform->TransformUpdate(flags);
  if(mModel != NULL)
    mModel->TransformUpdate(flags);

  for(size_t i = 0; i < mGameplayComponents.size(); ++i)
    mGameplayComponents[i]->TransformUpdate(flags);
}

void GameObject::DebugDraw()
{
  if(mTransform != NULL)
    mTransform->DebugDraw();
  if(mModel != NULL)
    mModel->DebugDraw();

  for(size_t i = 0; i < mGameplayComponents.size(); ++i)
    mGameplayComponents[i]->DebugDraw();
}

void GameObject::DisplayComponentProperties(TwBar* bar)
{
  if(mTransform != NULL)
    mTransform->DisplayProperties(bar);
  if(mModel != NULL)
    mModel->DisplayProperties(bar);

  for(size_t i = 0; i < mGameplayComponents.size(); ++i)
    mGameplayComponents[i]->DisplayProperties(bar);

  BindSimpleObjectMethod(bar, mName, GameObject, Union);
  BindSimpleObjectMethod(bar, mName, GameObject, Intersection);
  BindSimpleObjectMethod(bar, mName, GameObject, Subtract);
  BindSimpleObjectMethod(bar, mName, GameObject, Destroy);
}

void GameObject::ProcessKeyboardInput(unsigned int key, int x, int y)
{
  for(size_t i = 0; i < mGameplayComponents.size(); ++i)
    mGameplayComponents[i]->ProcessKeyboardInput(key, x, y);
}

void GameObject::Add(Transform* transform)
{
  transform->mOwner = this;
  mTransform = transform;
}

void GameObject::Add(Model* model)
{
  model->mOwner = this;
  mModel = model;
}

void GameObject::Add(Component* component)
{
  component->mOwner = this;
  mGameplayComponents.push_back(component);
}

void GameObject::Union()
{
  mApplication->Union(this);
}

void GameObject::Intersection()
{
  mApplication->Intersection(this);
}

void GameObject::Subtract()
{
  mApplication->Subtract(this);
}

void GameObject::Destroy()
{
  mApplication->DestroyGameObject(this);
}

//-----------------------------------------------------------------------------Transform
Transform::Transform()
{
  mScale = Vector3(1, 1, 1);
  mRotation = Math::Quaternion::cIdentity;
  mTranslation = Math::Vector3::cZero;
}

void Transform::DisplayProperties(TwBar* bar)
{
  std::string name = mOwner->mName;
  
  std::string groupName = "group=" + name + ".Transform";
  BindSimpleVector3Property(bar, name, Transform, Scale);
  BindSimpleVector3Property(bar, name, Transform, Translation);
  BindSimpleQuaternionProperty(bar, name, Transform, Rotation);
  BindSimpleVector3Property(bar, name, Transform, RotationEuler);

  DeclareObjectComponentGroup(bar, name, Transform);
}

Vector3 Transform::GetScale()
{
  return mScale;
}

void Transform::SetScale(const Vector3& scale)
{
  mScale = scale;
  mOwner->TransformUpdate(TransformUpdateFlags::Scale);
}

Quaternion Transform::GetRotation()
{
  return mRotation;
}

void Transform::SetRotation(const Quaternion& rotation)
{
  mRotation = rotation;
  mOwner->TransformUpdate(TransformUpdateFlags::Rotation);
}

Vector3 Transform::GetRotationEuler()
{
  Math::EulerAngles angles = Math::ToEulerAngles(mRotation);
  Vector3 result;
  result.x = Math::RadToDeg(angles.Angles.x);
  result.y = Math::RadToDeg(angles.Angles.y);
  result.z = Math::RadToDeg(angles.Angles.z);
  return result;
}

void Transform::SetRotationEuler(const Vector3& rotationEuler)
{
  Vector3 angles = rotationEuler;
  angles.x = Math::DegToRad(angles.x);
  angles.y = Math::DegToRad(angles.y);
  angles.z = Math::DegToRad(angles.z);

  mRotation = Math::ToQuaternion(Math::EulerAngles(angles, Math::EulerOrders::XYZs));
  mOwner->TransformUpdate(TransformUpdateFlags::Rotation);
}

Vector3 Transform::GetTranslation()
{
  return mTranslation;
}

void Transform::SetTranslation(const Vector3& translation)
{
  mTranslation = translation;
  mOwner->TransformUpdate(TransformUpdateFlags::Translation);
}

Math::Matrix4 Transform::GetTransform() const
{
  return Math::BuildTransform(mTranslation, Math::ToMatrix3(mRotation), mScale);
}

//-----------------------------------------------------------------------------SimpleMover
SimpleMover::SimpleMover()
{
  mTotalTime = 0;
}

void SimpleMover::Update(float dt)
{
  mTotalTime += dt * 4;

  Transform* transform = mOwner->has(Transform);
  if(transform != NULL)
  {
    transform->mTranslation.y = Math::Cos(mTotalTime);
    transform->mTranslation.z = Math::Sin(mTotalTime);
    mOwner->TransformUpdate(TransformUpdateFlags::Translation);
  }
};
