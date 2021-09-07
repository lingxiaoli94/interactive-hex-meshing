#include "PipelineStage.h"

#include "controllers/GlobalController.h"

namespace hex {
PipelineStage::PipelineStage(GlobalController& global_controller,
                             unsigned long default_visibility)
    : global_controller_{global_controller},
      last_visibility_{default_visibility} {}

void PipelineStage::SwitchTo() {
  global_controller_.GetGlobalView().SetVisibility(last_visibility_);
}

void PipelineStage::SwitchFrom() {
  last_visibility_ = global_controller_.GetGlobalView().GetVisibility();
}
}  // namespace hex
