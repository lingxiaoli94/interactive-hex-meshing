#pragma once

#include "HittableBase.h"

namespace vkoo {
namespace st {
class Cylinder : public HittableBase {
 public:
  // Cylinder has its base on xz plane.
  Cylinder(float radius, float height);
  bool Intersect(const Ray& ray, HitRecord& record) const override;
  bool IsYOnCylinder(float y) const;

 private:
  float radius_;
  float height_;
};
}  // namespace st
}  // namespace vkoo
