#include "vkoo/st/Scene.h"

namespace vkoo {
namespace st {
Camera* Scene::GetActiveCameraPtr() const { return active_camera_ptr_; }

void Scene::SetActiveCameraPtr(Camera* active_camera_ptr) {
  active_camera_ptr_ = active_camera_ptr;
}

Scene::Scene(std::unique_ptr<Node> root) : root_(std::move(root)) {}
}  // namespace st
}  // namespace vkoo
