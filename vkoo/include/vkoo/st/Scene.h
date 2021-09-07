#pragma once

#include "Node.h"
#include "vkoo/st/components/Camera.h"

namespace vkoo {
namespace st {
class Scene {
 public:
  Scene(std::unique_ptr<Node> root);
  Node& GetRoot() { return *root_; }
  Camera* GetActiveCameraPtr() const;
  void SetActiveCameraPtr(Camera* active_camera_ptr);

 private:
  std::unique_ptr<Node> root_;
  Camera* active_camera_ptr_;
};
}  // namespace st
}  // namespace vkoo
