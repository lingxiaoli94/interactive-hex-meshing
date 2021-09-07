#pragma once

#include "ComponentBase.h"
#include "vkoo/core/InputEvent.h"

namespace vkoo {
namespace st {
class Script : public ComponentBase {
 public:
  Script(Node& node);

  std::type_index GetType() const final;

  virtual ~Script() = default;
  virtual void Update([[maybe_unused]] float delta_time) {}
  virtual bool HandleInputEvent(
      [[maybe_unused]] const InputEvent& input_event) {
    return false;
  }
  virtual void OnWindowResize([[maybe_unused]] uint32_t width,
                              [[maybe_unused]] uint32_t height) {}
};
}  // namespace st
}  // namespace vkoo
