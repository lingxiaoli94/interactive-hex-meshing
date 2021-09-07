#include "vkoo/st/hittables/Cylinder.h"

namespace vkoo {
namespace st {
Cylinder::Cylinder(float radius, float height)
    : radius_{radius}, height_{height} {}

bool Cylinder::Intersect(const Ray& ray, HitRecord& record) const {
  auto d = ray.GetDirection();
  auto o = ray.GetOrigin();

  float a = d.x * d.x + d.z * d.z;
  float b = 2 * (o.x * d.x + o.z * d.z);
  float c = o.x * o.x + o.z * o.z - radius_ * radius_;

  float D = b * b - 4 * a * c;

  if (D < 0) {
    return false;
  }
  D = sqrt(D);
  float t_plus = (-b + D) / (2 * a);
  float t_minus = (-b - D) / (2 * a);

  float t;
  if (t_minus < 0) {
    if (t_plus < 0) {
      return false;
    } else {
      if (IsYOnCylinder(ray.At(t_plus).y)) {
        t = t_plus;
      } else {
        return false;
      }
    }
  } else {
    if (IsYOnCylinder(ray.At(t_minus).y)) {
      t = t_minus;
    } else if (IsYOnCylinder(ray.At(t_plus).y)) {
      t = t_plus;
    } else {
      return false;
    }
  }

  if (t < record.time) {
    record.time = t;
    auto intersection = ray.At(t);
    record.normal =
        glm::normalize(glm::vec3{intersection.x, 0.0f, intersection.z});
    return true;
  }
  return false;
}

bool Cylinder::IsYOnCylinder(float y) const { return 0 <= y && y <= height_; }
}  // namespace st
}  // namespace vkoo
