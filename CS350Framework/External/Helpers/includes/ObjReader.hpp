///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Evan Gray
/// Copyright 2026, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <string>
#include <vector>
#include "Math/Math.hpp"

namespace Helpers
{
  void LoadObjFile(std::string fileData, std::vector<Vector3>& outVertices, std::vector<size_t>& outIndices);
}//namespace Helpers
