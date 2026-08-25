#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "DeformationStage.h"
#include "controllers/GlobalController.h"
#include "logging.h"
#include "models/GlobalState.h"

namespace hex {
DeformationStage::DeformationStage(GlobalController& global_controller)
    : PipelineStage(global_controller) {}


void DeformationStage::InitDeformedMesh() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasTargetVolumeMesh()) {
    LOGW("Target volume mesh must exist before initializing deformed mesh!");
    return;
  }

  auto deformed_volume_mesh = std::make_unique<TetrahedralMesh>(
      global_state.GetTargetVolumeMesh().GetVertices(),
      global_state.GetTargetVolumeMesh().GetTets());
  global_state.SetDeformedVolumeMesh(std::move(deformed_volume_mesh));

  UpdateDeformationViews();
}

void DeformationStage::PrepareVolumetricDeformation() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasTargetVolumeMesh()) {
    LOGW(
        "Cannot perform volumetric cubic deformation if target has no volume "
        "mesh!");
    return;
  }

  if (!global_state.HasDeformedVolumeMesh()) {
    InitDeformedMesh();
  }

  cubic_volumetric_deformer_ = std::make_unique<CubicVolumetricDeformer>(
      global_state.GetTargetVolumeMesh(), global_state.GetDeformedVolumeMesh(),
      distortion_options_, deformer_options_);
}

void DeformationStage::Reoptimize(size_t num_steps) {
  if (cubic_volumetric_deformer_) {
    auto& global_state = global_controller_.GetGlobalState();
    cubic_volumetric_deformer_->Optimize(num_steps);

    // Recreate the whole tetrahedral mesh.
    auto deformed_volume_mesh = std::make_unique<TetrahedralMesh>(
        cubic_volumetric_deformer_->GetOptimizedPositions(),
        global_state.GetTargetVolumeMesh().GetTets());
    global_state.SetDeformedVolumeMesh(std::move(deformed_volume_mesh));
  }

  UpdateDeformationViews();
}

void DeformationStage::RunFromScript(const YAML::Node& cfg) {
  if (!global_controller_.GetGlobalState().HasTargetVolumeMesh()) {
    throw std::runtime_error(
        "[script] deformation requires a target volume mesh; load an HDF5 or "
        ".mesh that has one");
  }

  if (cfg["cubeness_weight"])
    deformer_options_.cubeness_weight = cfg["cubeness_weight"].as<float>();
  if (cfg["smoothness_weight"])
    deformer_options_.smoothness_weight = cfg["smoothness_weight"].as<float>();
  if (cfg["norm_eps"])
    deformer_options_.norm_eps = cfg["norm_eps"].as<float>();
  if (cfg["learning_rate"])
    deformer_options_.learning_rate = cfg["learning_rate"].as<float>();

  if (cfg["conformal_weight"])
    distortion_options_.conformal_weight = cfg["conformal_weight"].as<float>();
  if (cfg["authalic_weight"])
    distortion_options_.authalic_weight = cfg["authalic_weight"].as<float>();

  int steps = cfg["steps"] ? cfg["steps"].as<int>() : reopt_steps_;
  reopt_steps_ = steps;

  LOGI(
      "[script] deformation: cubeness={}, smoothness={}, conformal={}, "
      "authalic={}, lr={}, steps={}",
      deformer_options_.cubeness_weight, deformer_options_.smoothness_weight,
      distortion_options_.conformal_weight,
      distortion_options_.authalic_weight, deformer_options_.learning_rate,
      steps);

  PrepareVolumetricDeformation();
  Reoptimize(static_cast<size_t>(steps));
}

void DeformationStage::UpdateDeformationViews() {
#if HEX_ENABLE_VULKAN
  auto& global_state = global_controller_.GetGlobalState();
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->UpdateDeformedMeshView(global_state);
  }
#endif
  global_controller_.SetVisibility(VisibilityDeformedMesh);
}

}  // namespace hex
