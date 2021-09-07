#pragma once

#include "ComponentBase.h"
#include "vkoo/common.h"

namespace vkoo {
namespace st {
class Camera : public ComponentBase {
 public:
  Camera(float fov, float aspect_ratio, float z_near, float z_far);
  std::type_index GetType() const override { return typeid(Camera); }
  glm::mat4 GetProjectionMatrix() const;
  glm::mat4 GetViewMatrix() const;
  glm::vec3 GetFrontDirection() const;
  void SetAspectRatio(float aspect_ratio) { aspect_ratio_ = aspect_ratio; }
  void SetPrescribedViewMatrix(const glm::mat4& view_mat);

 private:
  float fov_;
  float aspect_ratio_;
  float z_near_;
  float z_far_;

  std::unique_ptr<glm::mat4> prescribed_view_mat_;
};
}  // namespace st
}  // namespace vkoo
