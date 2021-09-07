#pragma once

#include "vkoo/st/Node.h"
#include "vkoo/st/components/Camera.h"
#include "vkoo/st/components/Script.h"

namespace vkoo {
namespace st {
class ArcBallCameraScript : public Script {
 public:
  ArcBallCameraScript(Camera& camera, const glm::vec3& position,
                      const glm::vec3& anchor, const glm::vec3& up_direction,
                      bool reversed);
  void OnWindowResize(uint32_t width, uint32_t height) override;
  bool HandleInputEvent(const InputEvent& event) override;
  bool& GetReversedRef() { return reversed_; }
  const glm::vec3& GetPosition() const { return position_; }
  const glm::vec3& GetAnchor() const { return anchor_; }
  const glm::vec3& GetUpDirection() const { return up_direction_; }
  void UpdateViewMatrix();

 private:
  void PerformRotation(glm::vec2 new_mouse_position);
  void PerformZoom(glm::vec2 new_mouse_position);
  void PerformTranslation(glm::vec2 new_mouse_position);
  glm::vec3 GetRightDirection();

  Camera& camera_;
  glm::vec2 last_position_;
  std::map<MouseButton, bool> is_mouse_pressed_;

  glm::vec3 position_;
  glm::vec3 anchor_;
  glm::vec3 up_direction_;
  bool reversed_;
};
}  // namespace st
}  // namespace vkoo
