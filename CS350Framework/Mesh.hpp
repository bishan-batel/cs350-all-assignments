///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <vector>
#include "Shapes.hpp"
#include <string>

class Mesh
{
public:
  Mesh() { mDynamic = false; mType = 0; }
  Mesh(const std::string& name, int type)
  {
    mName = name;
    mDynamic = false;
    mType = type;
  }

  size_t TriangleCount() const
  {
    return mIndices.size() / 3;
  }

  Triangle TriangleAt(size_t triangleIndex) const
  {
    Triangle tri;
    tri.mPoints[0] = mVertices[mIndices[triangleIndex * 3 + 0]];
    tri.mPoints[1] = mVertices[mIndices[triangleIndex * 3 + 1]];
    tri.mPoints[2] = mVertices[mIndices[triangleIndex * 3 + 2]];
    return tri;
  }

  typedef std::vector<Vector3> Vertices;
  Vertices mVertices;
  typedef std::vector<size_t> Indices;
  Indices mIndices;

  std::string mName;
  bool mDynamic;
  int mType;
};
