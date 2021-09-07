#pragma once

#include "Ray.h"

namespace vkoo {
namespace st {
class HittableBase {
 public:
  virtual ~HittableBase() = default;
  virtual bool Intersect(const Ray& ray, HitRecord& record) const = 0;
};
}  // namespace st
}  // namespace vkoo