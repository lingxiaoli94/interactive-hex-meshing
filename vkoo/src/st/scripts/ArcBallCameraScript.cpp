#include "vkoo/st/scripts/ArcBallCameraScript.h"

#include <glm/gtx/rotate_vector.hpp>

#include "vkoo/logging.h"

namespace vkoo {
namespace {
const float kRotateSensitivity = 1e-3f;
const float kZoomSensitivity = 1e-3f;
const float kTranslateSensitivity = 5e-3f;
}  // namespace

namespace st {
ArcBallCameraScript::ArcBallCameraScript(Camera& camera,
                                         const glm::vec3& position,
                                         const glm::vec3& anchor,
                                         const glm::vec3& up_direction,
                                         bool reversed)
    : Script(*camera.GetNode()),
      camera_(camera),
      position_{position},
      anchor_{anchor},
      up_direction_{up_direction},
      reversed_{reversed} {
  const MouseButton buttons[] = {MouseButton::Left, MouseButton::Right,
                                 MouseButton::Middle};
  for (auto button : buttons) {
    is_mouse_pressed_[button] = false;
  }
}

void ArcBallCameraScript::OnWindowResize(uint32_t width, uint32_t height) {
  float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);
  camera_.SetAspectRatio(aspect_ratio);
}

bool ArcBallCameraScript::HandleInputEvent(const InputEvent& event) {
  bool handled = false;
  if (event.GetSource() == EventSource::Mouse) {
    auto& mouse_event = static_cast<const MouseButtonInputEvent&>(event);
    auto action = mouse_event.GetAction();
    auto button = mouse_event.GetButton();
    if (action == MouseAction::Move) {
      glm::vec2 new_mouse_position{static_cast<float>(mouse_event.GetXPos()),
                                   static_cast<float>(mouse_event.GetYPos())};
      if (is_mouse_pressed_[MouseButton::Left]) {
        PerformRotation(new_mouse_position);
        handled = true;
      } else if (is_mouse_pressed_[MouseButton::Right]) {
        PerformZoom(new_mouse_position);
        handled = true;
      } else if (is_mouse_pressed_[MouseButton::Middle]) {
        PerformTranslation(new_mouse_position);
        handled = true;
      }
    } else if (action == MouseAction::Down) {
      is_mouse_pressed_[button] = true;
      handled = true;
    } else if (action == MouseAction::Up) {
      is_mouse_pressed_[button] = false;
      handled = true;
    }
    last_position_.x = static_cast<float>(mouse_event.GetXPos());
    last_position_.y = static_cast<float>(mouse_event.GetYPos());
  }
  return handled;
}

void ArcBallCameraScript::PerformRotation(const glm::vec2 new_mouse_position) {
  glm::vec2 diff = new_mouse_position - last_position_;
  diff *= kRotateSensitivity;
  if (reversed_) {
    diff *= -1;
  }

  glm::vec3 new_position = position_;
  glm::vec3 right_direction = GetRightDirection();

  new_position =
      glm::rotate(new_position - anchor_, diff.x, up_direction_) + anchor_;
  new_position =
      glm::rotate(new_position - anchor_, diff.y, right_direction) + anchor_;

  position_ = new_position;
  glm::vec3 new_up_direction =
      glm::rotate(up_direction_, diff.y, right_direction);
  up_direction_ = new_up_direction;

  UpdateViewMatrix();
}

void ArcBallCameraScript::PerformZoom(const glm::vec2 new_mouse_position) {
  float diff_y = new_mouse_position.y - last_position_.y;
  position_ =
      anchor_ + std::exp(diff_y * kZoomSensitivity) * (position_ - anchor_);
  UpdateViewMatrix();
}

void ArcBallCameraScript::PerformTranslation(glm::vec2 new_mouse_position) {
  glm::vec2 diff = new_mouse_position - last_position_;
  diff *= kTranslateSensitivity;
  if (reversed_) {
    diff *= -1;
  }

  glm::vec3 delta = diff.x * GetRightDirection() - diff.y * up_direction_;
  position_ += delta;
  anchor_ += delta;

  UpdateViewMatrix();
}

void ArcBallCameraScript::UpdateViewMatrix() {
  glm::mat4 new_view_mat = glm::lookAt(position_, anchor_, up_direction_);
  camera_.SetPrescribedViewMatrix(new_view_mat);
}

glm::vec3 ArcBallCameraScript::GetRightDirection() {
  return glm::normalize(glm::cross(anchor_ - position_, up_direction_));
}

}  // namespace st
}  // namespace vkoo
