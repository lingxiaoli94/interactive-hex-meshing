#pragma once

#include "common.h"

#include <vkoo/st/hittables/Ray.h>

namespace hex {
class Ray {
 public:
  Ray(const Vector3f& origin, const Vector3f& direction);
  Ray(const vkoo::st::Ray& ray);
  vkoo::st::Ray ToGlm() const;

  const Vector3f& GetOrigin() const { return origin_; }
  const Vector3f& GetDirection() const { return direction_; }
  Vector3f At(float t) const;

 private:
  Vector3f origin_;
  Vector3f direction_;
};
}  // namespace hex
