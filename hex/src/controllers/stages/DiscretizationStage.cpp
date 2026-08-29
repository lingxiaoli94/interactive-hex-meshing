#include "DiscretizationStage.h"

#include <models/PolycubeGraph.h>
#include <yaml-cpp/yaml.h>

#include "controllers/GlobalController.h"
#include "logging.h"
#include "utility/StopWatch.h"
#include "vkoo/core/InputEvent.h"

namespace hex {
DiscretizationStage::DiscretizationStage(GlobalController& global_controller)
    : PipelineStage(global_controller, VisibilityDecomposition) {}


bool DiscretizationStage::CanSwitchTo() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycube()) {
    LOGW("Polycube must exist before discretization stage!");
    return false;
  }
  return true;
}

void DiscretizationStage::DiscretizePolycube() {
  auto& global_state = global_controller_.GetGlobalState();
  auto& polycube = global_state.GetPolycube();
  current_graph_ =
      std::make_unique<PolycubeGraph>(polycube, hex_size_, round_to_nearest_);
  current_quad_complex_ = current_graph_->GenerateQuadComplex();
  current_surface_mesh_ = current_quad_complex_->ExtractQuadMesh();

#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->UpdatePolycubeComplexView(*current_quad_complex_,
                                           color_by_patch_, true);
  }
#endif
  global_controller_.SetVisibility(VisibilityPolycubeComplex);
}

void DiscretizationStage::FinalizePolycube() {
  assert(current_graph_);
  auto& global_state = global_controller_.GetGlobalState();
  global_state.SetPolycubeComplex(current_graph_->GenerateHexComplex(padding_));

#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->UpdatePolycubeComplexView(
        *current_graph_->GenerateQuadComplex(), color_by_patch_, false);
  }
#endif
  global_controller_.SetVisibility(VisibilityPolycubeComplex);

  // Invalidate target complex and hex mesh.
  global_state.SetTargetComplex(nullptr);
  global_controller_.ResetHexahedralizationStage();
}

void DiscretizationStage::SwitchTo() { PipelineStage::SwitchTo(); }
void DiscretizationStage::SwitchFrom() { PipelineStage::SwitchFrom(); }

void DiscretizationStage::RunFromScript(const YAML::Node& cfg) {
  if (!global_controller_.GetGlobalState().HasPolycube()) {
    throw std::runtime_error(
        "[script] discretization requires a polycube; load an HDF5 that has "
        "one (post-decomposition stage)");
  }
  if (cfg["hex_size"]) hex_size_ = cfg["hex_size"].as<float>();
  if (cfg["round_to_nearest"])
    round_to_nearest_ = cfg["round_to_nearest"].as<bool>();
  if (cfg["padding"]) padding_ = cfg["padding"].as<bool>();

  LOGI("[script] discretization: hex_size={}, round_to_nearest={}, padding={}",
       hex_size_, round_to_nearest_, padding_);
  DiscretizePolycube();
  FinalizePolycube();
}



}  // namespace hex
