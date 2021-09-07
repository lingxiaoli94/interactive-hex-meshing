#include "GlobalController.h"

#include <exception>
#include <filesystem>

#include "logging.h"

// clang-format off
#include <imgui.h>
#include "utility/imfilebrowser.h"
// clang-format on

#include <vkoo/st/components/Script.h>

#include "HexMeshingApp.h"
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
ImGui::FileBrowser gFileBrowser(ImGuiFileBrowserFlags_EnterNewFilename |
                                ImGuiFileBrowserFlags_CreateNewDir);
[[maybe_unused]] const int kDeformationStageId = 0;
[[maybe_unused]] const int kDecompositionStageId = 1;
[[maybe_unused]] const int kDiscretizationStageId = 2;
const int kHexahedralizationStageId = 3;
}  // namespace

GlobalController::GlobalController(HexMeshingApp& app, uint32_t window_width,
                                   uint32_t window_height)
    : app_{app},
      device_{app.GetDevice()},
      scene_{app.GetScene()},
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
  global_view_.reset();
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
  global_view_ = std::make_unique<GlobalView>(device_, scene_);
  global_view_->UpdateAllViews(*global_state_);
  model_name_.clear();
  project_name_.clear();
}

void GlobalController::DrawMainMenuBar() {
  static std::string file_op;
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("New")) {
        global_state_ = std::make_unique<GlobalState>();
        RebuildViewsAndStages();
      }
      if (ImGui::MenuItem("Open")) {
        if (!last_visited_path_.empty()) {
          gFileBrowser.SetPwd(last_visited_path_);
        } else {
          gFileBrowser.SetPwd(PathManager::GetWorkspacePath());
        }
        gFileBrowser.Open();
        file_op = "open";
      }
      if (ImGui::MenuItem("Save")) {
        if (!last_visited_path_.empty()) {
          gFileBrowser.SetPwd(last_visited_path_);
        } else {
          gFileBrowser.SetPwd(PathManager::GetWorkspacePath());
        }
        gFileBrowser.Open();
        if (!project_name_.empty()) {
          gFileBrowser.SetInputBuf(project_name_);
        } else if (!model_name_.empty()) {
          gFileBrowser.SetInputBuf(model_name_);
        }
        file_op = "save";
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Import")) {
        gFileBrowser.SetPwd(PathManager::GetImportPath());
        gFileBrowser.Open();
        file_op = "import";
      }
      if (ImGui::MenuItem("Export", nullptr, false,
                          global_state_->HasResultMesh())) {
        gFileBrowser.SetPwd(PathManager::GetWorkspacePath());
        gFileBrowser.Open();
        if (!model_name_.empty()) {
          gFileBrowser.SetInputBuf(model_name_ + ".mesh");
        }
        file_op = "export";
      }

      if (IsExpertMode()) {
        if (ImGui::MenuItem("Import Hex Mesh")) {
          if (!last_visited_path_.empty()) {
            gFileBrowser.SetPwd(last_visited_path_);
          } else {
            gFileBrowser.SetPwd(PathManager::GetWorkspacePath());
          }
          gFileBrowser.Open();
          file_op = "import_hex";
        }
      }

      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Options")) {
      DrawRenderingOptions();
      ImGui::Separator();
      ImGui::Checkbox("Expert Mode", &app_.GetSettings().expert_mode);
      if (ImGui::MenuItem("Load Settings")) {
        if (!last_visited_path_.empty()) {
          gFileBrowser.SetPwd(last_visited_path_);
        } else {
          gFileBrowser.SetPwd(PathManager::GetWorkspacePath());
        }
        gFileBrowser.Open();
        file_op = "load_settings";
      }
      if (ImGui::MenuItem("Save Settings")) {
        if (!last_visited_path_.empty()) {
          gFileBrowser.SetPwd(last_visited_path_);
        } else {
          gFileBrowser.SetPwd(PathManager::GetWorkspacePath());
        }
        gFileBrowser.Open();
        file_op = "save_settings";
      }
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  gFileBrowser.Display();
  if (!file_op.empty() && gFileBrowser.HasSelected()) {
    auto selected_file = gFileBrowser.GetSelected().string();
#ifdef NDEBUG
    try {
#endif
      if (file_op == "open") {
        global_state_ = GlobalState::LoadFromFile(selected_file);
        RebuildViewsAndStages();
        project_name_ = fs::path(selected_file).filename().string();
        last_visited_path_ = fs::path(selected_file).parent_path();
      } else if (file_op == "save") {
        global_state_->SaveToFile(selected_file);
        last_visited_path_ = fs::path(selected_file).parent_path();
      } else if (file_op == "import") {
        global_state_ = std::make_unique<GlobalState>();
        RebuildViewsAndStages();
        LoadTargetMesh(selected_file);
        model_name_ = fs::path(selected_file).stem().string();
      } else if (file_op == "import_hex") {
        LoadResultMesh(selected_file);
      } else if (file_op == "export") {
        ExportTargetComplex(selected_file);
      } else if (file_op == "load_settings") {
        app_.ReloadSettings(selected_file);
      } else if (file_op == "save_settings") {
        app_.SaveSettings(selected_file);
      } else {
        LOGW("Unknown file op: {}", file_op);
      }
#ifdef NDEBUG
    } catch (...) {
      LOGW("Unknown exception thrown during I/O!");
    }
#endif
    gFileBrowser.ClearSelected();
    file_op = "";
  }
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

void GlobalController::DrawNavigationWindow() {
  ImGui::SetNextWindowPos(ImVec2(0, 100), ImGuiCond_Once);
  ImGui::SetNextWindowSize(ImVec2(200, 500), ImGuiCond_Once);

  ImGui::Begin("Navigation");

  ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
  if (ImGui::TreeNode("Stages")) {
    int tmp_stage_idx = current_stage_idx_;
    for (int i = 0; i < static_cast<int>(stages_.size()); i++) {
      if (ImGui::RadioButton(stages_[i]->GetName().c_str(), &tmp_stage_idx,
                             i)) {
        SwitchStage(tmp_stage_idx);
        break;
      }
    }
    ImGui::TreePop();
    ImGui::Spacing();
  }

  ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
  if (ImGui::TreeNode("Visibility")) {
    global_view_->DrawNodeVisibilityGui(IsKeyPressed(GLFW_KEY_LEFT_SHIFT));
    ImGui::TreePop();
    ImGui::Spacing();
  }

  ImGui::End();
}

void GlobalController::DrawGui() {
  if (app_.GetSettings().show_demo) {
    ImGui::ShowDemoWindow();
  }
  // return;

  DrawMainMenuBar();
  DrawNavigationWindow();
  DrawStageWindow();
  if (app_.GetSettings().show_advanced) {
    DrawAdvancedWindow();
  }
}

void GlobalController::DrawAdvancedWindow() {
  ImGui::Begin("Advanced");
  ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);

  ImGui::SetNextItemWidth(120);
  ImGui::InputInt("FPS limit", &app_.GetSettings().fps_limit);
  ImGui::End();
}

void GlobalController::DrawStageWindow() {
  if (current_stage_idx_ != -1) {
    ImGui::SetNextWindowPos(ImVec2(window_width_, 100), ImGuiCond_Once,
                            ImVec2(1.0f, 0.0f));

    ImGui::SetNextWindowSize(ImVec2(400, 600), ImGuiCond_Once);
    stages_[current_stage_idx_]->DrawStageWindow();
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
  global_view_->UpdateInputMeshView(*global_state_);
  global_view_->SetVisibility(GlobalView::VisibilityInputMesh);
}

void GlobalController::LoadResultMesh(const std::string& file_path) {
  auto extension = fs::path(file_path).extension();
  if (extension != ".mesh") {
    LOGW("Cannot load a non-volumetric mesh {}!", file_path);
    return;
  }
  auto hex_mesh = std::make_unique<HexahedralMesh>(file_path);

  global_state_->SetResultMesh(std::move(hex_mesh));
  global_view_->UpdateFilteredHexView(*global_state_, FilteringSetting{});
}

void GlobalController::ExportTargetComplex(const std::string& selected_file) {
  if (!global_state_->HasResultMesh()) {
    LOGW("Cannot save result mesh if it does not exist!");
    return;
  }

  std::function<Vector3f(Vector3f)> vertex_map = [](Vector3f v) { return v; };

  if (not app_.GetSettings().export_unit_scale &&
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

bool GlobalController::HandleInputEvent(const vkoo::InputEvent& event) {
  // Handle a couple of backdoor shortcuts.
  if (event.GetSource() == vkoo::EventSource::Keyboard) {
    auto& key_event = static_cast<const vkoo::KeyInputEvent&>(event);
    if (key_event.GetAction() == vkoo::KeyAction::Down) {
      auto code = key_event.GetCode();
      if (code == GLFW_KEY_F12) {
        app_.GetSettings().show_advanced ^= 1;
      } else if (code == GLFW_KEY_F11) {
        app_.GetSettings().show_demo ^= 1;
      } else if (code == GLFW_KEY_F10) {
        app_.SaveScreenshot("screenshot.png");
      } else if (code == GLFW_KEY_F9) {
        app_.GetSettings().show_gui ^= 1;
      }
    }
  }
  if (current_stage_idx_ == -1 ||
      !stages_[current_stage_idx_]->HandleInputEvent(event)) {
    bool handled = arc_ball_camera_script_->HandleInputEvent(event);
    if (handled) {
      UpdateCameraSettings();
    }
    return handled;
  }
  return false;
}

void GlobalController::UpdateCameraSettings() {
  auto& spec = app_.GetSettings().camera;
  spec.position = arc_ball_camera_script_->GetPosition();
  spec.anchor = arc_ball_camera_script_->GetAnchor();
  spec.up_direction = arc_ball_camera_script_->GetUpDirection();
}

bool GlobalController::IsExpertMode() const {
  return app_.GetSettings().expert_mode;
}

void GlobalController::InitArcBallCamera() {
  auto& camera_spec = app_.GetSettings().camera;
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

GlobalView& GlobalController::GetGlobalView() { return *global_view_; }

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

bool GlobalController::IsKeyPressed(int keycode) {
  return app_.IsKeyPressed(keycode);
}

void GlobalController::GetMousePosition(float& x_pos, float& y_pos) {
  app_.GetMousePosition(x_pos, y_pos);
}

void GlobalController::DrawRenderingOptions() {
  {
    // People have different convention than mine, so invert this.
    ImGui::Checkbox("Reverse Camera",
                    &arc_ball_camera_script_->GetReversedRef());
  }
  auto& settings = app_.GetSettings();

  bool need_update = false;
  if (ImGui::Checkbox("MSAA", &settings.msaa_on)) {
    need_update = true;
  }
  if (ImGui::Checkbox("SSAO", &settings.ssao_on)) {
    need_update = true;
  }
  if (ImGui::Checkbox("Export unit scale", &settings.export_unit_scale)) {
    need_update = true;
  }

  if (need_update) {
    app_.UpdatePipelines();
  }
}

}  // namespace hex
