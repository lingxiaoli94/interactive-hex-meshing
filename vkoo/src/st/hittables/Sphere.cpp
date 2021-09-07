#include "vkoo/st/hittables/Sphere.h"

namespace vkoo {
namespace st {
bool Sphere::Intersect(const Ray& ray, HitRecord& record) const {
  float a = glm::length2(ray.GetDirection());
  float b = 2 * glm::dot(ray.GetDirection(), ray.GetOrigin());
  float c = glm::length2(ray.GetOrigin()) - radius_ * radius_;

  float d = b * b - 4 * a * c;

  if (d < 0) {
    return false;
  }
  d = sqrt(d);

  float t_plus = (-b + d) / (2 * a);
  float t_minus = (-b - d) / (2 * a);

  float t;
  if (t_minus < 0) {
    if (t_plus < 0)
      return false;
    else {
      t = t_plus;
    }
  } else {
    t = t_minus;
  }

  if (t < record.time) {
    record.time = t;
    record.normal = glm::normalize(ray.At(t));
    return true;
  }

  return false;
}
}  // namespace st
}  // namespace vkoo
