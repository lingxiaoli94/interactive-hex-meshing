#include "QuadrilateralMesh.h"

#include <geomlib/ray_triangle_intersection.h>

#include <limits>

namespace hex {
QuadrilateralMesh::QuadrilateralMesh(std::vector<Vector3f> vertices,
                                     std::vector<Vector4i> quads)
    : vertices_{std::move(vertices)}, quads_{std::move(quads)} {}

bool QuadrilateralMesh::IntersectWithRay(const Ray& ray, HitRecord& record) {
  bool success = false;
  float min_t = std::numeric_limits<float>::max();
#pragma omp parallel for
  for (size_t i = 0; i < quads_.size(); i++) {
    auto& quad = quads_[i];
    auto& v0 = vertices_[quad[0]];
    auto& v1 = vertices_[quad[1]];
    auto& v2 = vertices_[quad[2]];
    auto& v3 = vertices_[quad[3]];

    float t1, t2;
    bool b1 = geomlib::RayTriangleIntersection(
        ray.GetOrigin(), ray.GetDirection(), v0, v1, v2, &t1);
    bool b2 = geomlib::RayTriangleIntersection(
        ray.GetOrigin(), ray.GetDirection(), v0, v2, v3, &t2);

    float t = std::numeric_limits<float>::max();
    if (b1) {
      t = std::min(t, t1);
    }
    if (b2) {
      t = std::min(t, t2);
    }

#pragma omp critical
    if (t < min_t) {
      min_t = t;
      record.t = min_t;
      record.quad_id = i;
    }
  }

  success = min_t < std::numeric_limits<float>::max();
  return success;
}
}  // namespace hex
