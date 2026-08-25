#include "PipelineStage.h"

#include "controllers/GlobalController.h"

namespace hex {
PipelineStage::PipelineStage(GlobalController& global_controller,
                             unsigned long default_visibility)
    : global_controller_{global_controller},
      last_visibility_{default_visibility} {}

// Visibility is state, not rendering: these two work with no view present
// because the mask lives in GlobalController (Step A2). Before that, a stage
// switch could not happen at all without a GlobalView.
void PipelineStage::SwitchTo() {
  global_controller_.SetVisibility(last_visibility_);
}

void PipelineStage::SwitchFrom() {
  last_visibility_ = global_controller_.GetVisibility();
}
}  // namespace hex
