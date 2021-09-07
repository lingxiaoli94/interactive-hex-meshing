#include "Ray.h"

namespace hex {
Ray::Ray(const Vector3f& origin, const Vector3f& direction)
    : origin_{origin}, direction_{direction} {}

Ray::Ray(const vkoo::st::Ray& ray)
    : origin_{ToEigen(ray.GetOrigin())},
      direction_{ToEigen(ray.GetDirection())} {}

vkoo::st::Ray Ray::ToGlm() const {
  return {::hex::ToGlm(origin_), ::hex::ToGlm(direction_)};
}

Vector3f Ray::At(float t) const { return origin_ + t * direction_; }
}  // namespace hex
