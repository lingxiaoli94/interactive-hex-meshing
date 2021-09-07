#pragma once

#include <vkoo/core/InputEvent.h>

#include "common.h"

namespace hex {
class GlobalController;

class PipelineStage {
 public:
  PipelineStage(GlobalController& global_controller,
                unsigned long default_visibility = 0);
  virtual ~PipelineStage() = default;
  virtual std::string GetName() const = 0;
  virtual bool HandleInputEvent(
      [[maybe_unused]] const vkoo::InputEvent& event) {
    return false;
  };
  virtual void Update([[maybe_unused]] float delta_time) {}
  virtual bool CanSwitchTo() { return true; }
  virtual void SwitchTo();
  virtual void SwitchFrom();
  virtual void DrawStageWindow(){};

 protected:
  GlobalController& global_controller_;
  unsigned long last_visibility_{0};
};
}  // namespace hex
