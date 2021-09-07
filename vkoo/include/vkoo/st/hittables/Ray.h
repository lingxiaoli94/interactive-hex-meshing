#pragma once

#include "vkoo/common.h"

namespace vkoo {
namespace st {

struct HitRecord {
  HitRecord() { time = std::numeric_limits<float>::max(); }

  float time;
  glm::vec3 normal;
};

class Ray {
 public:
  Ray(const glm::vec3& origin, const glm::vec3& direction)
      : origin_(origin), direction_(direction) {}

  const glm::vec3& GetOrigin() const { return origin_; }

  const glm::vec3& GetDirection() const { return direction_; }

  void SetDirection(const glm::vec3& direction) { direction_ = direction; }

  glm::vec3 At(float t) const { return origin_ + t * direction_; }

  void ApplyTransform(const glm::mat4& transform) {
    glm::vec3 ref = At(1.0f);
    glm::vec4 new_origin = transform * glm::vec4(origin_, 1.0f);
    origin_ = glm::vec3(new_origin / new_origin.w);
    glm::vec4 new_ref = transform * glm::vec4(ref, 1.0f);
    ref = glm::vec3(new_ref / new_ref.w);
    direction_ = ref - origin_;
    // Note: do not normalize direction_ here since we want
    // ref to always be At(1.0) before/after transform.
  }

 private:
  glm::vec3 origin_;
  glm::vec3 direction_;
};
}  // namespace st
}  // namespace vkoo