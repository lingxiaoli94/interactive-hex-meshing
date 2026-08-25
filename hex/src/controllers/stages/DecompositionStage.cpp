#include <yaml-cpp/yaml.h>

#include <cassert>
#include <stdexcept>

#include "DecompositionStage.h"
#include "controllers/GlobalController.h"
#include "logging.h"
#include "models/GlobalState.h"
#include "vkoo/core/InputEvent.h"

namespace hex {
namespace {
const float kDuplicateOffset = 0.01f;
}  // namespace
DecompositionStage::DecompositionStage(GlobalController& global_controller)
    : PipelineStage(global_controller, VisibilityDeformedMesh) {
  // Its constructor allocates a cylinder vertex object against the device, and
  // its later methods assume the nodes and materials that constructor builds.
  // With no device, do not build a controller that cannot honour its own API.
#if HEX_ENABLE_VULKAN
  if (global_controller.HasGraphicsDevice()) {
    cuboid_editing_controller_ =
        std::make_unique<CuboidEditingController>(global_controller, false);
  }
#else
  (void)global_controller;
#endif
  polycube_optimizer_ =
      std::make_unique<PolycubeOptimizer>(PolycubeOptimizer::Options());
}



bool DecompositionStage::CanSwitchTo() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasDeformedVolumeMesh()) {
    LOGW(
        "Deformed mesh is required for cuboid "
        "decomposition stage!");
    return false;
  }
  return true;
}

void DecompositionStage::SwitchTo() { PipelineStage::SwitchTo(); }


void DecompositionStage::Reoptimize(size_t num_steps) {
  UnfocusCuboid();

  auto& deformed_mesh =
      global_controller_.GetGlobalState().GetDeformedVolumeMesh();
  auto& global_state = global_controller_.GetGlobalState();
  auto& polycube = global_state.GetPolycube();
  auto& polycube_info = global_state.GetPolycubeInfo();
  opt_thread_ = std::thread([&, num_steps]() {
    polycube_optimizer_->Optimize(deformed_mesh, polycube, num_steps,
                                  polycube_info.locked);
  });
}

void DecompositionStage::SuggestNewCuboid(
    PolycubeOptimizer::SuggestStrategy strategy) {
  UnfocusCuboid();

  auto& deformed_mesh =
      global_controller_.GetGlobalState().GetDeformedVolumeMesh();
  auto& polycube = global_controller_.GetGlobalState().GetPolycube();
  Cuboid new_cuboid;
  bool success = polycube_optimizer_->SuggestNewCuboid(deformed_mesh, polycube,
                                                       strategy, new_cuboid);
  if (success) {
    AddNewCuboid(new_cuboid);
  } else {
    LOGI("Failed to suggest a new cuboid!");
  }
}

void DecompositionStage::RunFromScript(const YAML::Node& cfg) {
  auto& gs = global_controller_.GetGlobalState();
  if (!gs.HasDeformedVolumeMesh()) {
    throw std::runtime_error(
        "[script] decomposition requires a deformed volume mesh; run "
        "deformation first or load an HDF5 that has one");
  }

  if (cfg["grid_size"])
    anchors_options_.grid_size = cfg["grid_size"].as<int>();
  if (cfg["inside_only"])
    anchors_options_.inside_only = cfg["inside_only"].as<bool>();
  if (cfg["bbox_padding"])
    anchors_options_.bbox_padding = cfg["bbox_padding"].as<float>();
  if (cfg["surface_samples"])
    anchors_options_.surface_samples = cfg["surface_samples"].as<int>();
  if (cfg["perturbation"])
    anchors_options_.perturbation = cfg["perturbation"].as<float>();

  if (cfg["positive_l2_weight"])
    options_.positive_l2_weight = cfg["positive_l2_weight"].as<float>();
  if (cfg["negative_l2_weight"])
    options_.negative_l2_weight = cfg["negative_l2_weight"].as<float>();
  if (cfg["learning_rate"])
    options_.learning_rate = cfg["learning_rate"].as<float>();
  options_.snapshot_freq = -1;

  int num_cuboids = cfg["num_cuboids"] ? cfg["num_cuboids"].as<int>() : 8;
  std::string strat_str = cfg["suggest_strategy"]
                              ? cfg["suggest_strategy"].as<std::string>()
                              : "largest";
  PolycubeOptimizer::SuggestStrategy strategy =
      (strat_str == "simple") ? PolycubeOptimizer::SuggestStrategy::Simple
                              : PolycubeOptimizer::SuggestStrategy::Largest;
  int reopt_steps =
      cfg["reopt_steps"] ? cfg["reopt_steps"].as<int>() : reoptimize_steps_;
  reoptimize_steps_ = reopt_steps;

  LOGI(
      "[script] decomposition: num_cuboids={}, strategy={}, grid_size={}, "
      "samples={}, reopt_steps={}",
      num_cuboids, strat_str, anchors_options_.grid_size,
      anchors_options_.surface_samples, reopt_steps);

  CreateSdfAndAnchors();
  ResetPolycube();
  for (int i = 0; i < num_cuboids; ++i) {
    SuggestNewCuboid(strategy);
  }
  Reoptimize(static_cast<size_t>(reopt_steps));
  if (opt_thread_.joinable()) opt_thread_.join();

  // The GUI pulls the optimized polycube back into GlobalState every frame via
  // Update(); the script runner has no frame loop, so do it once here. Without
  // this the un-optimized (raw cuboid-suggestion) polycube is what gets saved --
  // the gradient refinement from Reoptimize() would be silently discarded.
  gs.SetPolycube(
      std::make_unique<Polycube>(polycube_optimizer_->GetOptimizedPolycube()));

  LOGI("[script] decomposition done");
}

void DecompositionStage::AddNewCuboid(const Cuboid& cuboid) {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycube()) {
    LOGW("You must initialize the polycube first!");
    return;
  }

  assert(global_state.CheckPolycubeConsistency());
  auto& polycube = global_state.GetPolycube();
  polycube.AddCuboid(cuboid);

  auto& polycube_info = global_state.GetPolycubeInfo();
  polycube_info.Push();

#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->GetPolycubeView().AddCuboid(cuboid);
  }
#endif

  FocusCuboid(polycube.GetCuboidCount() - 1);
}

void DecompositionStage::DuplicateCuboid(int i) {
  auto& global_state = global_controller_.GetGlobalState();
  auto& polycube = global_state.GetPolycube();

  Cuboid cuboid = polycube.GetCuboid(i);
  // Add in slight offset.
  cuboid.center += kDuplicateOffset * Vector3f::Ones();
  AddNewCuboid(cuboid);
}

void DecompositionStage::DeleteCuboid(int i) {
  if (i == focused_cuboid_index_) {
    UnfocusCuboid();
  }
  auto& global_state = global_controller_.GetGlobalState();

  auto& polycube = global_state.GetPolycube();
  polycube.DeleteCuboid(i);

  auto& polycube_info = global_state.GetPolycubeInfo();
  polycube_info.Delete(i);

#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->GetPolycubeView().DeleteCuboid(i);
  }
#endif
}

void DecompositionStage::ResetPolycube() {
  UnfocusCuboid();
  global_controller_.GetGlobalState().SetPolycube(std::make_unique<Polycube>());
  global_controller_.GetGlobalState().SetPolycubeInfo(
      std::make_unique<PolycubeInfo>());
#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->UpdatePolycubeView(global_controller_.GetGlobalState());
  }
#endif
  global_controller_.SetVisibility(VisibilityDeformedMesh |
                                   VisibilityDecomposition);
}


void DecompositionStage::FocusCuboid(int id) {
  if (focused_cuboid_index_ != -1) {
    UnfocusCuboid();
  }

  // Bookkeeping first, and unconditionally: UnfocusCuboid reads this index, so
  // focus/unfocus must stay paired whether or not anything is drawn.
  // AddNewCuboid reaches here on the script path, once per suggested cuboid.
  focused_cuboid_index_ = id;

#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    auto& cuboid_node = global_view->GetPolycubeView().GetCuboidNode(id);
    cuboid_node.UpdateMode(CuboidNode::Mode::Focused);
    cuboid_editing_controller_->Focus(
        cuboid_node,
        global_controller_.GetGlobalState().GetPolycube().GetCuboid(id));
  }
#endif

  global_controller_.AddVisibility(VisibilityDecomposition);
}

void DecompositionStage::JoinOptimizerThread() {
  if (opt_thread_.joinable()) {
    opt_thread_.join();
  }
}

void DecompositionStage::UnfocusCuboid() {
  // Load-bearing, and deliberately outside every view guard below: Reoptimize()
  // and ResetPolycube() both reach here on the script path, and skipping the
  // join along with the view work would race the optimizer thread.
  JoinOptimizerThread();

#if HEX_ENABLE_VULKAN
  if (subtract_editing_controller_) {
    subtract_editing_controller_.reset();  // destructor unfocuses automatically

    // Note: deletion of tmp cuboid must happen before calling the destructor of
    // subtract_editing_controller_!
#if HEX_ENABLE_VULKAN
    if (auto* global_view = global_controller_.GetGlobalView()) {
      global_view->GetPolycubeView().DeleteTmpCuboid();
    }
#endif
  }
#endif

  if (focused_cuboid_index_ == -1) {
    return;
  }
#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    cuboid_editing_controller_->Unfocus();
    auto& cuboid_node =
        global_view->GetPolycubeView().GetCuboidNode(focused_cuboid_index_);
    bool locked =
        global_controller_.GetGlobalState().GetPolycubeInfo().locked.at(
            focused_cuboid_index_);
    cuboid_node.UpdateMode(locked ? CuboidNode::Mode::Locked
                                  : CuboidNode::Mode::Free);
  }
#endif
  focused_cuboid_index_ = -1;
}

DecompositionStage::~DecompositionStage() {
#if HEX_ENABLE_VULKAN
  CuboidNode::Cleanup();  // TODO: fix this
#endif
  if (opt_thread_.joinable()) {
    polycube_optimizer_->Stop();
    opt_thread_.join();
  }
}

void DecompositionStage::SwitchFrom() {
  PipelineStage::SwitchFrom();
  if (opt_thread_.joinable()) {
    polycube_optimizer_->Stop();
    opt_thread_.join();
  }
  UnfocusCuboid();
}

void DecompositionStage::CreateSdfAndAnchors() {
  LOGI("Creating anchors and sdf for the deformed mesh...");
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasDeformedVolumeMesh()) {
    LOGW("Cannot create anchors and sdfs without a deformed mesh!");
    return;
  }
  // Note: order matters here; otherwise distance field will be built on a
  // different set of anchors due to randomness!
  global_state.GetDeformedVolumeMesh().CreateAnchors(
      anchors_options_.grid_size, anchors_options_.inside_only,
      anchors_options_.bbox_padding, anchors_options_.surface_samples,
      anchors_options_.perturbation);
  global_state.GetDeformedVolumeMesh().CreateDistanceField();

#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->UpdateAnchorsView(global_state);
  }
#endif
  global_controller_.SetVisibility(VisibilityDeformedMesh |
                                   VisibilityAnchors);
}






}  // namespace hex
// namespace hex
