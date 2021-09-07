#pragma once

#include "common.h"

#include <vkoo/core/VertexObject.h>

#include "Ray.h"

namespace hex {
class QuadrilateralMesh {
 public:
  QuadrilateralMesh(std::vector<Vector3f> vertices,
                    std::vector<Vector4i> quads);

  struct HitRecord {
    float t;
    size_t quad_id;
  };

  bool IntersectWithRay(const Ray& ray, HitRecord& record);

  const std::vector<Vector3f>& GetVertices() const { return vertices_; }
  const std::vector<Vector4i>& GetQuads() const { return quads_; }

 private:
  std::vector<Vector3f> vertices_;
  std::vector<Vector4i> quads_;
};
}  // namespace hex
