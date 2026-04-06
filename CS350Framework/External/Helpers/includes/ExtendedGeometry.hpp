///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis & Evan Gray
/// Copyright 2026, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Math/Math.hpp"
#include <vector>

namespace Helpers
{

bool RayCylinder(const Vector3& rayStart, const Vector3& rayDir, const Vector3& pointA, const Vector3& pointB, float radius, float& t);

}//namespace Helpers
