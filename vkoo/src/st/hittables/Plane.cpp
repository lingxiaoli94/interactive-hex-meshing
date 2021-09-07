#include "vkoo/st/hittables/Plane.h"

namespace vkoo {
namespace st {
Plane::Plane(const glm::vec3& normal, float d) : normal_{normal}, d_{d} {}

bool Plane::Intersect(const Ray& ray, HitRecord& record) const {
  float t = (d_ - glm::dot(ray.GetOrigin(), normal_)) /
            glm::dot(ray.GetDirection(), normal_);
  if (t < record.time) {
    record.time = t;
    record.normal = normal_;
    return true;
  }
  return false;
}
}  // namespace st
}  // namespace vkoo
