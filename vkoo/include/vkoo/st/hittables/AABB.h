#pragma once

#include "HittableBase.h"

namespace vkoo {
namespace st {
class AABB : public HittableBase {
 public:
  AABB(const glm::vec3& mn, const glm::vec3& mx);
  static AABB FromCuboid(const glm::vec3& center, const glm::vec3& halflengths);

  bool Intersect(const Ray& ray, HitRecord& record) const override;

 private:
  glm::vec3 mn_;
  glm::vec3 mx_;
};
}  // namespace st
}  // namespace vkoo
