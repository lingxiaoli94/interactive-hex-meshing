#include "vkoo/st/hittables/AABB.h"

namespace vkoo {
namespace st {
AABB::AABB(const glm::vec3& mn, const glm::vec3& mx) : mn_{mn}, mx_{mx} {}

bool AABB::Intersect(const Ray& ray, HitRecord& record) const {
  auto ray_dir = ray.GetDirection();
  auto ray_origin = ray.GetOrigin();

  float divx = 1 / (ray_dir[0] + 1e-8f);
  float divy = 1 / (ray_dir[1] + 1e-8f);
  float divz = 1 / (ray_dir[2] + 1e-8f);

  float tx0 = (mn_[0] - ray_origin[0]) * divx;
  float tx1 = (mx_[0] - ray_origin[0]) * divx;
  float ty0 = (mn_[1] - ray_origin[1]) * divy;
  float ty1 = (mx_[1] - ray_origin[1]) * divy;
  float tz0 = (mn_[2] - ray_origin[2]) * divz;
  float tz1 = (mx_[2] - ray_origin[2]) * divz;

  if (ray_dir[0] < 0) {
    std::swap(tx0, tx1);
  }
  if (ray_dir[1] < 0) {
    std::swap(ty0, ty1);
  }
  if (ray_dir[2] < 0) {
    std::swap(tz0, tz1);
  }

  float t_lower = std::max(std::max(tx0, ty0), tz0);
  float t_upper = std::min(std::min(tx1, ty1), tz1);
  if (t_lower <= t_upper) {
    // Normal doesn't make sense in the interior of the AABB.
    if (record.time > t_lower) {
      record.time = t_lower;
      return true;
    }
  }
  return false;
}

AABB AABB::FromCuboid(const glm::vec3& center, const glm::vec3& halflengths) {
  return AABB(center - halflengths, center + halflengths);
}
}  // namespace st
}  // namespace vkoo
