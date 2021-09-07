#pragma once

#include <vkoo/st/hittables/Ray.h>
#include <vkoo/st/scripts/ArcBallCameraScript.h>

#include "ShortcutController.h"
#include "models/GlobalState.h"
#include "models/Ray.h"
#include "stages/DecompositionStage.h"
#include "stages/DeformationStage.h"
#include "stages/DiscretizationStage.h"
#include "stages/HexahedralizationStage.h"
#include "views/GlobalView.h"

namespace hex {
class HexMeshingApp;

class GlobalController {
 public:
  GlobalController(HexMeshingApp& app, uint32_t window_width,
                   uint32_t window_height);
  ~GlobalController();
  bool HandleInputEvent(const vkoo::InputEvent& event);
  void OnWindowResize(uint32_t width, uint32_t height);
  void Update(float delta_time);
  void DrawGui();
  GlobalState& GetGlobalState();
  GlobalView& GetGlobalView();
  vkoo::Device& GetDevice() { return device_; }
  vkoo::st::Scene& GetScene() { return scene_; }
  bool IsExpertMode() const;

  DeformationStage& GetDeformationStage();
  DecompositionStage& GetDecompositionStage();
  DiscretizationStage& GetDiscretizationStage();
  HexahedralizationStage& GetHexahedralizationStage();
  void ResetHexahedralizationStage();

  void SwitchStage(PipelineStage& stage);
  void SwitchStage(int new_stage_idx);

  // Utility functions that might be refactored to another place.
  Ray ShootRayAtMousePosition(float x_pos, float y_pos);
  glm::mat4 GetWorldToScreenMatrix() const;
  bool IsKeyPressed(int keycode);
  void GetMousePosition(float& x_pos, float& y_pos);

  void LoadTargetMesh(const std::string& file_path);
  void LoadResultMesh(const std::string& file_path);
  void ExportTargetComplex(const std::string& selected_file);

  void UpdateCameraSettings();
  void InitArcBallCamera();

 private:
  void RebuildStages();
  // Gui related.
  void DrawMainMenuBar();
  void DrawNavigationWindow();
  void DrawStageWindow();
  void DrawRenderingOptions();
  void DrawAdvancedWindow();

  void RebuildViewsAndStages();

  HexMeshingApp& app_;
  vkoo::Device& device_;
  vkoo::st::Scene& scene_;

  std::unique_ptr<DeformationStage> deformation_stage_;
  std::unique_ptr<DecompositionStage> decomposition_stage_;
  std::unique_ptr<DiscretizationStage> discretization_stage_;
  std::unique_ptr<HexahedralizationStage> hexahedralization_stage_;

  std::vector<PipelineStage*> stages_;
  int current_stage_idx_{0};
  ShortcutController shortcut_controller_;

  std::unique_ptr<GlobalState> global_state_;
  std::unique_ptr<GlobalView> global_view_;

  uint32_t window_width_;
  uint32_t window_height_;

  std::unique_ptr<vkoo::st::ArcBallCameraScript> arc_ball_camera_script_;
  std::string model_name_;
  std::string project_name_;
  std::string last_visited_path_;
};
}  // namespace hex
