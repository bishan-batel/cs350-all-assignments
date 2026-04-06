///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Math/Math.hpp"
#include <vector>

class Model;
class Application;
class Component;
class Transform;

// Flags used to denote what kind of a transform update is happening
namespace TransformUpdateFlags
{
  enum Enum {Scale = 1 << 1, Rotation = 1 << 2, Translation = 1 << 3};
}

//-----------------------------------------------------------------------------GameObject
// A simple component based game object. This gameobject always contains a transform
// and model and can contain other misc. components such as the simple mover.
// Feel free to add your own components if you want to.
class GameObject
{
public:
  //GameObject();
  GameObject(Application* application);
  ~GameObject();

  // Simple helpers to wrap calling functions on all components
  void Update(float dt);
  void TransformUpdate(TransformUpdateFlags::Enum flags);
  void DebugDraw();

  void DisplayComponentProperties(TwBar* bar);
  void ProcessKeyboardInput(unsigned int key, int x, int y);

  // Various add functions
  void Add(Transform* transform);
  void Add(Model* model);
  void Add(Component* component);

  void Union();
  void Intersection();
  void Subtract();
  void Destroy();

  // Function to look up a component by name and cast it to the correct type.
  // Some types (Transform and Model) avoid a lookup.
  template <typename ComponentType>
  ComponentType* QueryComponentType(const std::string& name);

  // The name of the object (must be unique due to display object properties in anttweakbar
  std::string mName;
  // The application that owns this object. Used to get access to the broadphase.
  Application* mApplication;

  Transform* mTransform;
  Model* mModel;
  std::vector<Component*> mGameplayComponents;  
};

//-----------------------------------------------------------------------------Component
class Component
{
public:
  virtual const char* GetName() = 0;
  // Some time has passed, run update logic if desired.
  virtual void Update(float dt) {};
  // This object has been moved (by anttweakbar or the simple mover for example).
  // The flags specify what was changed in case special logic is needed.
  virtual void TransformUpdate(TransformUpdateFlags::Enum flags) {};
  // Add any debug drawing if desired.
  virtual void DebugDraw() {}

  virtual void DisplayProperties(TwBar* bar) {};
  virtual void ProcessKeyboardInput(unsigned int key, int x, int y) {};

  GameObject* mOwner = nullptr;
};

// Simple macro to add some required data per component (at the moment only to make the has macro work)
#define DeclareComponent(ComponentType) \
  const char* GetName() override { return #ComponentType; }

//-----------------------------------------------------------------------------Transform
class Transform : public Component
{
public:
  Transform();

  DeclareComponent(Transform);
  void DisplayProperties(TwBar* bar) override;

  Vector3 GetScale();
  void SetScale(const Vector3& scale);

  Quaternion GetRotation();
  void SetRotation(const Quaternion& rotation);

  Vector3 GetRotationEuler();
  void SetRotationEuler(const Vector3& rotationEuler);

  Vector3 GetTranslation();
  void SetTranslation(const Vector3& translation);

  // Combines Scale, Rotation, and Translation together into a Matrix4.
  Matrix4 GetTransform() const;

  Vector3 mScale;
  Quaternion mRotation;
  Vector3 mTranslation;
};

//-----------------------------------------------------------------------------SimpleMover
// A simple component to move the object around over time (based upon cos and sin)
class SimpleMover : public Component
{
public:
  SimpleMover();

  DeclareComponent(SimpleMover);
  void Update(float dt) override;

  float mTotalTime;
};

// Generic Implementation of looking up a component by name
template <typename ComponentType>
inline ComponentType* GameObject::QueryComponentType(const std::string& name)
{
  for(size_t i = 0; i < mGameplayComponents.size(); ++i)
  {
    if(mGameplayComponents->GetName() == name)
      return mGameplayComponents;
  }
  return NULL;
}

// Transform is statically composited
template <>
inline Transform* GameObject::QueryComponentType(const std::string& name)
{
  return mTransform;
}

// Model is statically composited
template <>
inline Model* GameObject::QueryComponentType(const std::string& name)
{
  return mModel;
}

// Simple helper macro to call the templated function with the component
// type (and auto make the component's string name)
#define has(ComponentType) QueryComponentType<ComponentType>(#ComponentType)
