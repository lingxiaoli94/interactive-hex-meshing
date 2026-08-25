#include "GlobalController.h"

#include <cassert>
#include <exception>
#include <filesystem>
#include <stdexcept>

#include "logging.h"

// The non-visual half of GlobalController. Everything that draws, polls input
// or touches the application object lives in GlobalControllerGui.cpp, which the
// HEX_ENABLE_VULKAN=OFF build does not compile. No imgui, no GLFW, no
// HexMeshingApp below this line.
#include <vkoo/st/components/Script.h>

#include "Settings.h"
#include "serialization/Serializer.h"
#include "stages/DecompositionStage.h"
#include "stages/DeformationStage.h"
#include "stages/DiscretizationStage.h"
#include "stages/HexahedralizationStage.h"
#include "utility/PathManager.h"
#include "vkoo/core/InputEvent.h"

namespace fs = std::filesystem;

namespace hex {
namespace {
const int kHexahedralizationStageId = 3;

void ValidateStateForStage(const GlobalState& state, int stage_index) {
  if (state.HasPolycube() != state.HasPolycubeInfo()) {
    throw std::runtime_error(
        "stage restart input must contain both polycube and polycube_info, or "
        "neither");
  }
  if (state.HasPolycube() && !state.CheckPolycubeConsistency()) {
    throw std::runtime_error(
        "stage restart input has inconsistent polycube and polycube_info");
  }

  switch (stage_index) {
    case 0:
      if (!state.HasTargetVolumeMesh()) {
        throw std::runtime_error(
            "deformation requires target_volume_mesh in the input HDF5");
      }
      return;
    case 1:
      if (!state.HasDeformedVolumeMesh()) {
        throw std::runtime_error(
            "decomposition requires deformed_volume_mesh in the input HDF5");
      }
      return;
    case 2:
      if (!state.HasPolycube()) {
        throw std::runtime_error(
            "discretization requires polycube and polycube_info in the input "
            "HDF5");
      }
      return;
    case 3:
      if (!state.HasTargetVolumeMesh() ||
          !state.HasDeformedVolumeMesh() || !state.HasPolycubeComplex()) {
        throw std::runtime_error(
            "hexahedralization requires target_volume_mesh, "
            "deformed_volume_mesh, and polycube_complex in the input HDF5");
      }
      return;
    default:
      throw std::invalid_argument("stage index must be between 0 and 3");
  }
}
}  // namespace

GlobalController::GlobalController(Settings& settings, vkoo::Device* device,
                                   vkoo::st::Scene& scene,
                                   uint32_t window_width,
                                   uint32_t window_height)
    : settings_{settings},
      device_{device},
      scene_{scene},
      shortcut_controller_{*this},
      window_width_{window_width},
      window_height_{window_height} {
  global_state_ = std::make_unique<GlobalState>();
  InitArcBallCamera();
  RebuildViewsAndStages();
}

GlobalController::~GlobalController() {
  // When this is called, scene has not been destructed yet.

  // First destruct all stages, since they might hold scene nodes attached to
  // views.
  deformation_stage_.reset();
  decomposition_stage_.reset();
  discretization_stage_.reset();
  hexahedralization_stage_.reset();

  // Next destruct global view and state.
#if HEX_ENABLE_VULKAN
  global_view_.reset();
#endif
  global_state_.reset();
}

void GlobalController::RebuildStages() {
  deformation_stage_ = std::make_unique<DeformationStage>(*this);
  decomposition_stage_ = std::make_unique<DecompositionStage>(*this);
  discretization_stage_ = std::make_unique<DiscretizationStage>(*this);
  hexahedralization_stage_ = std::make_unique<HexahedralizationStage>(*this);

  stages_.clear();

  stages_.push_back(deformation_stage_.get());
  stages_.push_back(decomposition_stage_.get());
  stages_.push_back(discretization_stage_.get());
  stages_.push_back(hexahedralization_stage_.get());

  current_stage_idx_ = 0;
}

DeformationStage& GlobalController::GetDeformationStage() {
  return *deformation_stage_;
}

DecompositionStage& GlobalController::GetDecompositionStage() {
  return *decomposition_stage_;
}

DiscretizationStage& GlobalController::GetDiscretizationStage() {
  return *discretization_stage_;
}

HexahedralizationStage& GlobalController::GetHexahedralizationStage() {
  return *hexahedralization_stage_;
}

void GlobalController::ResetHexahedralizationStage() {
  hexahedralization_stage_ = std::make_unique<HexahedralizationStage>(*this);
  stages_[kHexahedralizationStageId] = hexahedralization_stage_.get();
}

void GlobalController::RebuildViewsAndStages() {
  // This will rebuild all views and controllers given global_state_;
  RebuildStages();
  // Step A4: do not construct what needs a device. With no device there is no
  // view at all, rather than a view that is present but cannot allocate.
#if HEX_ENABLE_VULKAN
  if (device_ != nullptr) {
    global_view_ = std::make_unique<GlobalView>(*device_, scene_);
    global_view_->UpdateAllViews(*global_state_);
    global_view_->ApplyVisibility(visibility_mask_);
  }
#else
  assert(device_ == nullptr);  // no renderer in this build, so never a device
#endif
  model_name_.clear();
  project_name_.clear();
}

vkoo::Device& GlobalController::GetDevice() {
  // Vulkan-only method: reaching it without a device means a call site was
  // misclassified, not that no-Vulkan mode is unsupported.
  assert(device_ != nullptr);
  return *device_;
}

void GlobalController::SetVisibility(unsigned long flags) {
  visibility_mask_ = flags;
#if HEX_ENABLE_VULKAN
  if (global_view_ != nullptr) {
    global_view_->ApplyVisibility(visibility_mask_);
  }
#endif
}

void GlobalController::AddVisibility(unsigned long flags) {
  SetVisibility(visibility_mask_ | flags);
}

void GlobalController::RemoveVisibility(unsigned long flags) {
  SetVisibility(visibility_mask_ & ~flags);
}

void GlobalController::SwitchStage(int new_stage_idx) {
  if (stages_[new_stage_idx]->CanSwitchTo()) {
    stages_[current_stage_idx_]->SwitchFrom();
    stages_[new_stage_idx]->SwitchTo();
    current_stage_idx_ = new_stage_idx;
  }
}

void GlobalController::SwitchStage(PipelineStage& stage) {
  auto it = std::find(stages_.begin(), stages_.end(), &stage);
  if (it != stages_.end()) {
    SwitchStage(std::distance(stages_.begin(), it));
  }
}

void GlobalController::LoadTargetMesh(const std::string& file_path) {
  auto extension = fs::path(file_path).extension();
  if (extension != ".mesh" && extension != ".vtk") {
    LOGW("Cannot load a non-volumetric mesh {}!", file_path);
    return;
  }
  auto tet_mesh = std::make_unique<TetrahedralMesh>(file_path);

  {
    // TODO HACK: normalize the model.
    Eigen::MatrixXf vertices = tet_mesh->GetVertices();
    Vector3f bbox_min = vertices.colwise().minCoeff();
    Vector3f bbox_max = vertices.colwise().maxCoeff();
    Vector3f side_length = bbox_max - bbox_min;
    float max_length = side_length.maxCoeff();
    Vector3f center = (bbox_min + bbox_max) / 2;

    for (int i = 0; i < vertices.rows(); i++) {
      for (int j = 0; j < vertices.cols(); j++) {
        vertices(i, j) = (vertices(i, j) - center(j)) / (max_length / 2);
      }
    }
    tet_mesh->FixTetsOrientation();
    tet_mesh = std::make_unique<TetrahedralMesh>(vertices, tet_mesh->GetTets());

    global_state_->SetFloat("input_scale", max_length);
    global_state_->SetMatrixXf("input_center", center);
    LOGI("calculated input_center: ({}, {}, {})", center.x(), center.y(),
         center.z());
  }

  global_state_->SetTargetVolumeMesh(std::move(tet_mesh));
#if HEX_ENABLE_VULKAN
  if (global_view_ != nullptr) {
    global_view_->UpdateInputMeshView(*global_state_);
  }
#endif
  SetVisibility(VisibilityInputMesh);
}

void GlobalController::LoadResultMesh(const std::string& file_path) {
  auto extension = fs::path(file_path).extension();
  if (extension != ".mesh") {
    LOGW("Cannot load a non-volumetric mesh {}!", file_path);
    return;
  }
  auto hex_mesh = std::make_unique<HexahedralMesh>(file_path);

  global_state_->SetResultMesh(std::move(hex_mesh));
#if HEX_ENABLE_VULKAN
  if (global_view_ != nullptr) {
    global_view_->UpdateFilteredHexView(*global_state_, FilteringSetting{});
  }
#endif
}

void GlobalController::OpenProject(const std::string& file_path) {
  global_state_ = GlobalState::LoadFromFile(file_path);
  RebuildViewsAndStages();
  project_name_ = fs::path(file_path).filename().string();
  last_visited_path_ = fs::path(file_path).parent_path();
}

void GlobalController::OpenProjectForStage(const std::string& file_path,
                                           int stage_index) {
  auto restart_state = Serializer::LoadStateForStage(file_path, stage_index);
  ValidateStateForStage(*restart_state, stage_index);

  // Assign only after loading and validation succeed. This leaves the current
  // project intact on failure and, critically, rebuilds controllers/views only
  // from the already-filtered state.
  global_state_ = std::move(restart_state);
  RebuildViewsAndStages();
  project_name_ = fs::path(file_path).filename().string();
  last_visited_path_ = fs::path(file_path).parent_path();
}

void GlobalController::SaveProject(const std::string& file_path) {
  global_state_->SaveToFile(file_path);
  last_visited_path_ = fs::path(file_path).parent_path();
}

void GlobalController::ExportTargetComplex(const std::string& selected_file) {
  if (!global_state_->HasResultMesh()) {
    LOGW("Cannot save result mesh if it does not exist!");
    return;
  }

  std::function<Vector3f(Vector3f)> vertex_map = [](Vector3f v) { return v; };

  if (not settings_.export_unit_scale &&
      global_state_->HasFloat("input_scale") &&
      global_state_->HasMatrixXf("input_center")) {
    float input_scale = global_state_->GetFloat("input_scale");
    auto input_center_mat = global_state_->GetMatrixXf("input_center");
    Vector3f input_center = Eigen::Map<Vector3f>(input_center_mat.data(), 3);
    vertex_map = [=](Vector3f v) -> Vector3f {  // don't omit return type here!
      return v * (input_scale / 2) + input_center;
    };
  }
  Serializer::SaveHexMesh(global_state_->GetResultMesh(), selected_file,
                          vertex_map);
}

void GlobalController::UpdateCameraSettings() {
  auto& spec = settings_.camera;
  spec.position = arc_ball_camera_script_->GetPosition();
  spec.anchor = arc_ball_camera_script_->GetAnchor();
  spec.up_direction = arc_ball_camera_script_->GetUpDirection();
}

bool GlobalController::IsExpertMode() const {
  return settings_.expert_mode;
}

void GlobalController::InitArcBallCamera() {
  auto& camera_spec = settings_.camera;
  arc_ball_camera_script_ = std::make_unique<vkoo::st::ArcBallCameraScript>(
      *scene_.GetActiveCameraPtr(), camera_spec.position, camera_spec.anchor,
      camera_spec.up_direction, true);
  arc_ball_camera_script_->UpdateViewMatrix();
}

void GlobalController::OnWindowResize(uint32_t width, uint32_t height) {
  window_width_ = width;
  window_height_ = height;
  arc_ball_camera_script_->OnWindowResize(width, height);
}

void GlobalController::Update(float delta_time) {
  if (current_stage_idx_ != -1) {
    stages_[current_stage_idx_]->Update(delta_time);
  }
}

GlobalState& GlobalController::GetGlobalState() { return *global_state_; }

Ray GlobalController::ShootRayAtMousePosition(float x_pos, float y_pos) {
  // Make (0, 0) the bottom left.
  y_pos = window_height_ - y_pos;

  // Rescale to [-1, 1].
  x_pos = x_pos / window_width_ * 2.0f - 1.0f;
  y_pos = y_pos / window_height_ * 2.0f - 1.0f;

  vkoo::st::Ray ray{{x_pos, y_pos, 0}, {0, 0, 1}};
  auto& camera = *scene_.GetActiveCameraPtr();
  glm::mat4 proj_view_mat =
      camera.GetProjectionMatrix() * camera.GetViewMatrix();
  ray.ApplyTransform(glm::inverse(proj_view_mat));
  ray.SetDirection(glm::normalize(ray.GetDirection()));
  return Ray(ray);
}

glm::mat4 GlobalController::GetWorldToScreenMatrix() const {
  auto& camera = *scene_.GetActiveCameraPtr();
  return camera.GetProjectionMatrix() * camera.GetViewMatrix();
}


}  // namespace hex
