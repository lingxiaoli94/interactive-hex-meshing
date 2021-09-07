#include "vkoo/st/components/Camera.h"

#include <vkoo/st/hittables/Ray.h>

#include "vkoo/st/Node.h"
#include "vkoo/st/Transform.h"

namespace vkoo {
namespace st {
Camera::Camera(float fov, float aspect_ratio, float z_near, float z_far)
    : fov_(fov), aspect_ratio_(aspect_ratio), z_near_(z_near), z_far_(z_far) {}

glm::mat4 Camera::GetProjectionMatrix() const {
  return glm::perspective(fov_ * kPi / 180.f, aspect_ratio_, z_near_, z_far_);
}

glm::vec3 Camera::GetFrontDirection() const {
  Ray ray{{0, 0, 0}, {0, 0, 1}};
  glm::mat4 proj_view_mat = GetProjectionMatrix() * GetViewMatrix();
  ray.ApplyTransform(glm::inverse(proj_view_mat));
  ray.SetDirection(glm::normalize(ray.GetDirection()));
  return ray.GetDirection();
}

glm::mat4 Camera::GetViewMatrix() const {
  if (prescribed_view_mat_) {
    return *prescribed_view_mat_;
  }
  return glm::inverse(GetNode()->GetTransform().GetLocalToWorldMatrix());
}

void Camera::SetPrescribedViewMatrix(const glm::mat4& view_mat) {
  prescribed_view_mat_ = std::make_unique<glm::mat4>(view_mat);
}
}  // namespace st
}  // namespace vkoo
