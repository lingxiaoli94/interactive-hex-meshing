#pragma once

#include "HittableBase.h"

namespace vkoo {
namespace st {
class Plane : public HittableBase {
 public:
  Plane(const glm::vec3& normal, float d);
  bool Intersect(const Ray& ray, HitRecord& record) const override;

 private:
  glm::vec3 normal_;
  float d_;
};
}  // namespace st
}  // namespace vkoo
