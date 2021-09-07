#pragma once

#include "HittableBase.h"

namespace vkoo {
namespace st {
class Sphere : public HittableBase {
 public:
  // A sphere is always centered at origin in its local coordinate.
  Sphere(float radius) : radius_(radius) {}
  bool Intersect(const Ray& ray, HitRecord& record) const override;

 private:
  float radius_;
};
}  // namespace st
}  // namespace vkoo
