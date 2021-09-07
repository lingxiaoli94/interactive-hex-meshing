#include "ray_triangle_intersection.h"

namespace geomlib {
namespace {
const float kEps = 1e-8f;
}

// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool RayTriangleIntersection(const Vector3f& ray_origin,
                             const Vector3f& ray_dir, const Vector3f& v0,
                             const Vector3f& v1, const Vector3f& v2,
                             float* result_t) {
  Vector3f e1 = v1 - v0;
  Vector3f e2 = v2 - v0;
  Vector3f h = ray_dir.cross(e2);
  float a = e1.dot(h);
  if (std::abs(a) < kEps) {
    return false;
  }

  float f = 1.0f / a;
  Vector3f s = ray_origin - v0;
  float u = f * s.dot(h);

  if (u < 0 || u > 1) {
    return false;
  }

  Vector3f q = s.cross(e1);
  float v = f * ray_dir.dot(q);
  if (v < 0 || u + v > 1.0) {
    return false;
  }

  float t = f * e2.dot(q);
  if (t > kEps) {
    if (result_t != nullptr) {
      *result_t = t;
    }
    return true;
  } else {
    return false;
  }
}
}  // namespace geomlib
