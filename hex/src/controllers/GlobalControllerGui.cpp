#include "GlobalController.h"

// The GUI half of GlobalController: menus, windows, input polling and the few
// calls that genuinely need the application object (screenshots, settings I/O,
// pipeline rebuilds). Split out of GlobalController.cpp so a
// HEX_ENABLE_VULKAN=OFF build can own a GlobalController with no imgui, no GLFW
// and no HexMeshingApp in the binary. hex/CMakeLists.txt excludes this file
// when the option is OFF; the guard below makes that explicit rather than
// relying on the build system alone.
#if HEX_ENABLE_VULKAN

#include <filesystem>

// clang-format off
#include <imgui.h>
#include "utility/imfilebrowser.h"
// clang-format on

#include "HexMeshingApp.h"
#include "Settings.h"
#include "logging.h"
#include "utility/PathManager.h"
#include "views/GlobalView.h"
#include "vkoo/core/InputEvent.h"

namespace fs = std::filesystem;

namespace hex {
namespace {
ImGui::FileBrowser gFileBrowser(ImGuiFileBrowserFlags_EnterNewFilename |
                                ImGuiFileBrowserFlags_CreateNewDir);
}  // namespace

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
      ImGui::Checkbox("Expert Mode", &settings_.expert_mode);
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
        gui_app_->ReloadSettings(selected_file);
      } else if (file_op == "save_settings") {
        gui_app_->SaveSettings(selected_file);
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
    // GUI-only: a GUI implies a device, hence a view.
    assert(global_view_ != nullptr);
    if (global_view_->DrawNodeVisibilityGui(IsKeyPressed(GLFW_KEY_LEFT_SHIFT),
                                            visibility_mask_)) {
      global_view_->ApplyVisibility(visibility_mask_);
    }
    ImGui::TreePop();
    ImGui::Spacing();
  }

  ImGui::End();
}

void GlobalController::DrawGui() {
  if (settings_.show_demo) {
    ImGui::ShowDemoWindow();
  }
  // return;

  DrawMainMenuBar();
  DrawNavigationWindow();
  DrawStageWindow();
  if (settings_.show_advanced) {
    DrawAdvancedWindow();
  }
}

void GlobalController::DrawAdvancedWindow() {
  ImGui::Begin("Advanced");
  ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);

  ImGui::SetNextItemWidth(120);
  ImGui::InputInt("FPS limit", &settings_.fps_limit);
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

bool GlobalController::HandleInputEvent(const vkoo::InputEvent& event) {
  // Handle a couple of backdoor shortcuts.
  if (event.GetSource() == vkoo::EventSource::Keyboard) {
    auto& key_event = static_cast<const vkoo::KeyInputEvent&>(event);
    if (key_event.GetAction() == vkoo::KeyAction::Down) {
      auto code = key_event.GetCode();
      if (code == GLFW_KEY_F12) {
        settings_.show_advanced ^= 1;
      } else if (code == GLFW_KEY_F11) {
        settings_.show_demo ^= 1;
      } else if (code == GLFW_KEY_F10) {
        gui_app_->SaveScreenshot("screenshot.png");
      } else if (code == GLFW_KEY_F9) {
        settings_.show_gui ^= 1;
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

bool GlobalController::IsKeyPressed(int keycode) {
  return gui_app_->IsKeyPressed(keycode);
}

void GlobalController::GetMousePosition(float& x_pos, float& y_pos) {
  gui_app_->GetMousePosition(x_pos, y_pos);
}

void GlobalController::DrawRenderingOptions() {
  {
    // People have different convention than mine, so invert this.
    ImGui::Checkbox("Reverse Camera",
                    &arc_ball_camera_script_->GetReversedRef());
  }
  auto& settings = settings_;

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
    gui_app_->UpdatePipelines();
  }
}

}  // namespace hex

#endif  // HEX_ENABLE_VULKAN
