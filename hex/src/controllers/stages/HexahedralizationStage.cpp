#include "HexahedralizationStage.h"

#include <geomlib/autograd/GeneralizedProjection.h>
#include <geomlib/hausdorff_distance.h>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <stdexcept>

#include "DiscretizationStage.h"
#include "controllers/GlobalController.h"
#include "logging.h"
#include "optim/torch_utils.h"
#include "utility/StopWatch.h"
#include "vkoo/core/InputEvent.h"

namespace hex {
namespace {
const int kHausdorffDistanceBatchSize = 5000;
const int kHausdorffDistanceRepeatTimes = 10;
}  // namespace

HexahedralizationStage::HexahedralizationStage(
    GlobalController& global_controller)
    : PipelineStage{global_controller,
                    global_controller.GetGlobalState().HasResultMesh()
                        ? VisibilityResultFilteredMesh
                        : VisibilityPolycubeComplex} {
  // Step A4: both views allocate against the device, and the constructor used
  // to build them before checking anything. Skip construction rather than
  // early-returning from their users later.
#if HEX_ENABLE_VULKAN
  if (global_controller_.HasGraphicsDevice()) {
    landmarks_view_ = std::make_unique<LandmarksEditingView>(
        global_controller_.GetDevice(),
        global_controller_.GetScene().GetRoot());
    landmarks_view_->GetWrapperNode()->SetVisible(false);

    QuadSurfaceView::Options view_options;
    pickable_surface_view_ = std::make_unique<QuadSurfaceView>(
        global_controller_.GetDevice(), global_controller_.GetScene().GetRoot(),
        view_options);
  }
#endif

  auto& global_state = global_controller_.GetGlobalState();
  if (global_state.HasTargetComplex()) {
    assert(global_state.HasResultMesh());
    UpdateCurrentSurface();
    ComputeHausdorffDistance();
  }
}

HexahedralizationStage::~HexahedralizationStage() {
  StopAndJoinOptThread();
#if HEX_ENABLE_VULKAN
  if (pickable_surface_view_) {
    pickable_surface_view_->GetWrapperNode()->RemoveFromParent();
  }
  if (landmarks_view_) {
    landmarks_view_->GetWrapperNode()->RemoveFromParent();
  }
#endif
}

void HexahedralizationStage::UpdateCurrentSurface() {
  auto& global_state = global_controller_.GetGlobalState();
  assert(global_state.HasTargetComplex());
  auto& target_complex = global_state.GetTargetComplex();
  // current_surface_mesh_ is load-bearing (FindClosestVertexOnQuad reads it);
  // only the two view updates below need a device.
  current_surface_mesh_ = target_complex.GetQuadComplex().ExtractQuadMesh();

#if HEX_ENABLE_VULKAN
  if (pickable_surface_view_) {
    pickable_surface_view_->Update(*current_surface_mesh_);
    pickable_surface_view_->GetWrapperNode()->SetVisible(is_editing_);
  }
  if (landmarks_view_) {
    landmarks_view_->UpdatePositions(
        global_state.GetResultMesh().GetVertices());
  }
#else
  (void)global_state;
#endif
}


bool HexahedralizationStage::CanSwitchTo() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycubeComplex() ||
      !global_state.HasDeformedVolumeMesh() ||
      !global_state.HasTargetVolumeMesh()) {
    LOGW(
        "Hexahedralization depends on having a target mesh, a deformed mesh, "
        "and a polycube complex!");
    return false;
  }
  return true;
}


void HexahedralizationStage::ClearLandmarks() {
  // landmarks_set_ is optimizer input (OptimizeHexDeformation turns it into
  // fixed surface indices), so it is cleared in every mode; only removing the
  // markers from the scene needs a view.
#if HEX_ENABLE_VULKAN
  if (landmarks_view_) {
    for (auto& landmark : landmarks_set_) {
      landmarks_view_->RemoveLandmark(landmark);
    }
  }
#endif
  landmarks_set_.clear();
}


void HexahedralizationStage::SwitchTo() { PipelineStage::SwitchTo(); }

void HexahedralizationStage::SwitchFrom() {
  PipelineStage::SwitchFrom();
  StopAndJoinOptThread();
#if HEX_ENABLE_VULKAN
  // is_editing_ can only ever be set from the GUI, and LeaveEditMode lives in
  // the GUI translation unit.
  if (is_editing_) {
    LeaveEditMode();
  }
#endif
}
void HexahedralizationStage::StopAndJoinOptThread() {
  if (opt_thread_.joinable()) {
    hex_complex_deformer_->ForceStop();
    opt_thread_.join();
  }
}

void HexahedralizationStage::InitTargetComplex(bool use_pullback, bool indirect,
                                               bool transport) {
  // Copy polycube complex + target surface complex to
  // initialize(DeformationStage).
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycubeComplex() ||
      !global_state.HasTargetVolumeMesh()) {
    LOGW("Hex deformation requires an initialized target complex!");
    return;
  }

  auto target_complex =
      std::make_unique<HexComplex>(global_state.GetPolycubeComplex());
  global_state.SetTargetComplex(std::move(target_complex));

  if (use_pullback) {
    auto volume_vertices = PullPolycubeVolumeBack(indirect);

    if (transport) {
      auto deformer_options = hex_deformer_options_;
      deformer_options.mode = HexComplexDeformer::Mode::Projection;
      deformer_options.snapshot_freq = -1;
      deformer_options.projection_weight = 1.0f;
      deformer_options.learning_rate = 1e-3f;
      deformer_options.smooth_transport = true;

      auto distortion_options = DistortionOptions();
      // distortion_options.amips = true;
      hex_complex_deformer_ = std::make_unique<HexComplexDeformer>(
          global_state.GetPolycubeComplex(), global_state.GetTargetComplex(),
          global_state.GetTargetVolumeMesh().GetSurfaceMesh(),
          distortion_options, deformer_options);
      hex_complex_deformer_->SetPulledbackPositions(volume_vertices);
      if (!global_controller_.IsExpertMode()) {
        OptimizeHexDeformation(200);  // this updates target_complex
        hex_complex_deformer_.reset();
      } else {
        in_transition_ = true;
      }
    } else {
      global_state.GetTargetComplex().UpdateVertices(volume_vertices);
    }
  }
  UpdateOnTargetComplexChange();
  global_controller_.SetVisibility(VisibilityResultFilteredMesh);

  // Invalidate optimizer and landmarks.
  if (!transport) {
    hex_complex_deformer_.reset();
  }
  ClearLandmarks();
}

void HexahedralizationStage::PrepareHexDeformation() {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycubeComplex() ||
      !global_state.HasTargetVolumeMesh() || !global_state.HasTargetComplex()) {
    LOGW("Hex deformation requires an initialized target complex!");
    return;
  }

  auto& target_complex = global_state.GetTargetComplex();
  hex_complex_deformer_ = std::make_unique<HexComplexDeformer>(
      global_state.GetPolycubeComplex(), target_complex,
      global_state.GetTargetVolumeMesh().GetSurfaceMesh(), distortion_options_,
      hex_deformer_options_);
  deformer_dirty_ = false;
}

void HexahedralizationStage::OptimizeHexDeformation(int num_steps) {
  if (opt_thread_.joinable()) {
    opt_thread_.join();
  }

  if (!hex_complex_deformer_) {
    LOGW("You need to prepare hex complex deformer first!");
    return;
  }

  if (deformer_dirty_) {
    PrepareHexDeformation();
  }

  auto& global_state = global_controller_.GetGlobalState();
  auto& target_complex = global_state.GetTargetComplex();

  // Translate landmarks from hex vtx ids to surface ids.
  std::vector<int> fixed_surface_indices;
  for (auto i : landmarks_set_) {
    fixed_surface_indices.push_back(
        target_complex.GetToSurfaceIndexDict().at(i));
  }

  if (hex_deformer_options_.snapshot_freq == -1) {
    hex_complex_deformer_->Optimize(num_steps, fixed_surface_indices);
    FetchSnapshot();
  } else {
    opt_thread_ = std::thread([&, num_steps, fixed_surface_indices]() {
      hex_complex_deformer_->Optimize(num_steps, fixed_surface_indices);
    });
  }
}
void HexahedralizationStage::RunFromScript(const YAML::Node& cfg) {
  auto& global_state = global_controller_.GetGlobalState();
  if (!global_state.HasPolycubeComplex() ||
      !global_state.HasDeformedVolumeMesh() ||
      !global_state.HasTargetVolumeMesh()) {
    throw std::runtime_error(
        "[script] hexahedralization requires polycube_complex + deformed_mesh "
        "+ target_mesh; load an HDF5 with all three (post-discretization)");
  }

  bool inversion_free =
      cfg["inversion_free"] ? cfg["inversion_free"].as<bool>() : true;
  if (cfg["projection_weight"])
    hex_deformer_options_.projection_weight =
        cfg["projection_weight"].as<float>();
  if (cfg["hausdorff_weight"])
    hex_deformer_options_.hausdorff_weight =
        cfg["hausdorff_weight"].as<float>();
  if (cfg["fairness_weight"])
    hex_deformer_options_.fairness_weight = cfg["fairness_weight"].as<float>();
  if (cfg["smoothness_weight"])
    hex_deformer_options_.smoothness_weight =
        cfg["smoothness_weight"].as<float>();
  if (cfg["learning_rate"])
    hex_deformer_options_.learning_rate = cfg["learning_rate"].as<float>();
  // Force blocking optimization for scripted runs (no async snapshot thread).
  hex_deformer_options_.snapshot_freq = -1;

  int steps = cfg["steps"] ? cfg["steps"].as<int>() : 100;

  LOGI(
      "[script] hexahedralization: inversion_free={}, projection={}, "
      "hausdorff={}, fairness={}, smoothness={}, lr={}, steps={}",
      inversion_free, hex_deformer_options_.projection_weight,
      hex_deformer_options_.hausdorff_weight,
      hex_deformer_options_.fairness_weight,
      hex_deformer_options_.smoothness_weight,
      hex_deformer_options_.learning_rate, steps);

  // Init pullback + (if inversion_free) the inversion-free 200-step init.
  // In non-expert mode this auto-resets hex_complex_deformer_ at the end.
  InitTargetComplex(/*use_pullback=*/true, /*indirect=*/inversion_free,
                    /*transport=*/inversion_free);

  // Run additional user-requested optimization steps with the configured
  // weights. Must rebuild deformer because Init resets it.
  if (steps > 0) {
    PrepareHexDeformation();
    OptimizeHexDeformation(steps);
  }
}

void HexahedralizationStage::FetchSnapshot() {
  auto& global_state = global_controller_.GetGlobalState();
  auto& target_complex = global_state.GetTargetComplex();
  target_complex.UpdateVertices(hex_complex_deformer_->FetchSnapshot());
  UpdateOnTargetComplexChange();
  global_controller_.AddVisibility(VisibilityResultFilteredMesh);
}


void HexahedralizationStage::UpdateOnTargetComplexChange() {
  auto& global_state = global_controller_.GetGlobalState();
  auto& target_complex = global_state.GetTargetComplex();
  global_state.SetResultMesh(target_complex.ExtractHexMesh());

  UpdateCurrentSurface();

  UpdateFilteredMeshView();
  ComputeHausdorffDistance();
}

void HexahedralizationStage::ComputeHausdorffDistance() {
  StopWatch timer;
  timer.Tic();
  auto& global_state = global_controller_.GetGlobalState();
  auto& input_mesh = global_state.GetTargetVolumeMesh().GetSurfaceMesh();
  auto& quad_complex = global_state.GetTargetComplex().GetQuadComplex();
  auto quad_faces = quad_complex.GetTriangularFaces();

  auto vertices0 = MatrixXfToTensor(input_mesh.GetVertices()).to(hex::ComputeDevice());
  auto faces0 =
      MatrixXiToTensor(input_mesh.GetFaces()).to(hex::ComputeDevice()).to(torch::kInt64);
  auto vertices1 =
      MatrixXfToTensor(ArrayVector3fToMatrixXf(quad_complex.GetVertices()))
          .to(hex::ComputeDevice());
  auto faces1 =
      MatrixXiToTensor(ArrayVector3iToMatrixXi(quad_complex.GetRing()))
          .to(hex::ComputeDevice())
          .to(torch::kInt64);
  hausdorff_distance_ = geomlib::ComputeHausdorffDistance(
      vertices0, faces0, vertices1, faces1, kHausdorffDistanceBatchSize,
      kHausdorffDistanceRepeatTimes);
  // Scale by diagonal of the bbox of input mesh.
  auto bbox_max = std::get<0>(vertices0.max(0));
  auto bbox_min = std::get<0>(vertices0.min(0));
  float diagonal_length =
      (bbox_max - bbox_min).square().sum().sqrt().item<float>();
  std::get<0>(hausdorff_distance_) /= diagonal_length;
  std::get<1>(hausdorff_distance_) /= diagonal_length;
}

void HexahedralizationStage::UpdateFilteredMeshView() {
#if HEX_ENABLE_VULKAN
  if (auto* global_view = global_controller_.GetGlobalView()) {
    global_view->UpdateFilteredHexView(global_controller_.GetGlobalState(),
                                       filtering_setting_);
  }
#endif
}

std::vector<Vector3f> HexahedralizationStage::PullPolycubeVolumeBack(
    bool indirect) {
  auto& global_state = global_controller_.GetGlobalState();
  assert(global_state.HasPolycubeComplex() &&
         global_state.HasDeformedVolumeMesh() &&
         global_state.HasTargetVolumeMesh() && global_state.HasTargetComplex());

  auto& target_volume_mesh = global_state.GetTargetVolumeMesh();
  auto& deformed_volume_mesh = global_state.GetDeformedVolumeMesh();
  auto& target_complex = global_state.GetTargetComplex();

  if (indirect) {
    // First optimize using projection for a few steps.
    auto deformer_options = HexComplexDeformer::Options();
    deformer_options.mode = HexComplexDeformer::Mode::Projection;
    deformer_options.snapshot_freq = -1;
    deformer_options.projection_weight = 1.0f;
    deformer_options.learning_rate = 1e-3f;

    auto distortion_options = DistortionOptions();
    distortion_options.regularizer_eps = 1e-4f;  // use a stricter regularizer
    // distortion_options.amips = true;
    hex_complex_deformer_ = std::make_unique<HexComplexDeformer>(
        global_state.GetPolycubeComplex(), target_complex,
        global_state.GetDeformedVolumeMesh().GetSurfaceMesh(),
        distortion_options, deformer_options);
    OptimizeHexDeformation(100);  // this updates target_complex
    hex_complex_deformer_.reset();
  }

  auto result = geomlib::GeneralizedTetrahedronProjection<3>::apply(
      MatrixXfToTensor(ArrayVector3fToMatrixXf(target_complex.GetVertices()))
          .to(hex::ComputeDevice()),
      MatrixXfToTensor(deformed_volume_mesh.GetVertices()).to(hex::ComputeDevice()),
      MatrixXiToTensor(deformed_volume_mesh.GetTets()).to(hex::ComputeDevice()));

  auto tet_ids = TensorToVectorXi(result[1].detach().cpu());
  auto bary_coords = TensorToMatrixXf(result[2].detach().cpu());
  Eigen::MatrixXf pulled_points =
      target_volume_mesh.RetrievePointsFromBarycentricCoordinates(tet_ids,
                                                                  bary_coords);

  return MatrixXfToArrayVector3f(pulled_points);
}




// Note: this returns vertex index in result hex mesh.




}  // namespace hex
