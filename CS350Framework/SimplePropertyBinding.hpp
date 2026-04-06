///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

// Simple wrapper around a get callback global function from ant-tweakbar to a member function call
template <typename ClassType, typename ValueType, ValueType (ClassType::*getter)()>
void TW_CALL GetCallback(void* value, void* clientData)
{
  ClassType* instance = static_cast<ClassType*>(clientData);
  *(ValueType*)value = (instance->*getter)();
}

// Simple wrapper around a set callback global function from ant-tweakbar to a member function call
template <typename ClassType, typename ValueType, void (ClassType::*setter)(const ValueType&)>
void TW_CALL SetCallback(const void* value, void* clientData)
{
  ClassType* instance = static_cast<ClassType*>(clientData);
  ValueType& data = *(ValueType*)value;
  (instance->*setter)(data);
}

// Simple wrapper around a method global function from ant-tweakbar to a member function call
template <typename ClassType, void (ClassType::*method)()>
void TW_CALL MethodCallback(void* clientData)
{
  ClassType* instance = static_cast<ClassType*>(clientData);
  (instance->*method)();
}

#define BindFullProperty(bar, propertyName, anttweakbarType, classType, valueType, getter, setter, groupName, labelName) \
  TwAddVarCB(bar, propertyName.c_str(), anttweakbarType, &SetCallback<classType, valueType, &classType::setter>,  &GetCallback<classType, valueType, &classType::getter>, this, (groupName + " " + labelName).c_str());

#define BindPropertyInGroup(bar, classType, propertyName, valueType, anttweakbarType, groupName) \
  TwAddVarCB(bar, #propertyName, anttweakbarType, &SetCallback<classType, valueType, &classType::Set##propertyName>,  &GetCallback<classType, valueType, &classType::Get##propertyName>, this, groupName);

#define BindProperty(bar, objName, propertyName, anttweakbarType, classType, valueType, getter, setter) \
  BindFullProperty(bar, (objName + "." + propertyName), anttweakbarType, classType, valueType, getter, setter, "group=" + objName + "." + #classType, "label=" + propertyName)

#define BindSimpleProperty(bar, objName, classType, propertyName, anttweakbarType, valueType) \
  BindProperty(bar, objName, #propertyName, anttweakbarType, classType, valueType, Get##propertyName, Set##propertyName)

#define BindSimpleVector3Property(bar, objName, classType, propertyName) \
  BindSimpleProperty(bar, objName, classType, propertyName, TW_TYPE_DIR3F, Vector3)

#define BindSimpleQuaternionProperty(bar, objName, classType, propertyName) \
  BindSimpleProperty(bar, objName, classType, propertyName, TW_TYPE_QUAT4F, Quaternion)

// Add a button to ant-tweakbar that calls a method on an object
#define BindMethod(bar, classType, displayName, methodName, extraDef) \
  TwAddButton(bar, displayName, &MethodCallback<classType, &classType::methodName>, this, extraDef);

#define BindSimpleMethod(bar, classType, methodName, extraDef) \
  BindMethod(bar, classType, #methodName, methodName, extraDef);

// Simple method binding for an object (takes the object name and auto groups it)
#define BindSimpleObjectMethod(bar, objName, classType, methodName) \
  BindMethod(bar, classType, (objName + "." + #methodName).c_str(), methodName, ("group=" + objName + " label=" + #methodName).c_str())

#define DeclareObjectGroup(bar, objName, groupName, extraDef) \
  TwDefine((TwGetBarName(bar) + std::string("/") + objName + "." groupName " group=" + objName + " label=" groupName " " extraDef).c_str());

#define DeclareObjectComponentGroup(bar, objName, componentType) \
  DeclareObjectGroup(bar, objName, #componentType, "")
