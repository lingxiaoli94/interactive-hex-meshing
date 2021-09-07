#pragma once

#include "common.h"

namespace geomlib {
bool RayTriangleIntersection(const Vector3f& ray_origin,
                             const Vector3f& ray_dir, const Vector3f& v0,
                             const Vector3f& v1, const Vector3f& v2,
                             float* result_t = nullptr);
}  // namespace geomlib
