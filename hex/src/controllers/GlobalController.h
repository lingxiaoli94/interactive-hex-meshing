#pragma once

#include <vkoo/st/Scene.h>
#include <vkoo/st/hittables/Ray.h>
#include <vkoo/st/scripts/ArcBallCameraScript.h>

#include "ShortcutController.h"
#include "models/GlobalState.h"
#include "models/Ray.h"
#include "models/Visibility.h"
#include "stages/DecompositionStage.h"
#include "stages/DeformationStage.h"
#include "stages/DiscretizationStage.h"
#include "stages/HexahedralizationStage.h"
#if HEX_ENABLE_VULKAN
#include "views/GlobalView.h"
#endif

namespace vkoo {
// Only ever a pointer or a reference here. Including Device.h would drag the
// whole renderer back into this header, which is the opposite of Step B2.
class Device;
}  // namespace vkoo

namespace hex {
class GlobalView;
class HexMeshingApp;
class Settings;

class GlobalController {
 public:
  // Constructor data, not an application reference: `Settings`, the scene and
  // the (nullable) device are everything the non-visual half needs. This is
  // what lets a HEX_ENABLE_VULKAN=OFF build own a GlobalController without
  // there being a vkoo::Application anywhere in the binary.
  GlobalController(Settings& settings, vkoo::Device* device,
                   vkoo::st::Scene& scene, uint32_t window_width,
                   uint32_t window_height);
  ~GlobalController();
  void OnWindowResize(uint32_t width, uint32_t height);
  void Update(float delta_time);
  GlobalState& GetGlobalState();

  // Nullability lives here, at the orchestration boundary, and nowhere below
  // it: view constructors still take `Device&`. In --no-vulkan mode there is
  // no device, so no view was ever constructed and this returns nullptr.
  // Every caller outside views/ must check for nullptr or route through a
  // controller method whose visual tail is a no-op without a graphics device.
  bool HasGraphicsDevice() const { return device_ != nullptr; }
#if HEX_ENABLE_VULKAN
  GlobalView* GetGlobalView() { return global_view_.get(); }
#else
  // This build has no renderer, so there is never a view. Returning nullptr
  // rather than deleting the accessor keeps every `if (auto* v = ...)` call
  // site from Phase A compiling and correct in both variants.
  GlobalView* GetGlobalView() { return nullptr; }
#endif
  vkoo::Device& GetDevice();
  vkoo::st::Scene& GetScene() { return scene_; }
  bool IsExpertMode() const;

  // Which objects the user has chosen to see. This is non-visual state that
  // used to live inside GlobalView; it moved here so PipelineStage::SwitchTo /
  // SwitchFrom and the ~10 stage call sites keep working with no view present.
  // GlobalView is now a consumer of the mask, not its owner.
  void SetVisibility(unsigned long flags);
  void AddVisibility(unsigned long flags);
  void RemoveVisibility(unsigned long flags);
  unsigned long GetVisibility() const { return visibility_mask_; }

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

#if HEX_ENABLE_VULKAN
  // Defined in GlobalControllerGui.cpp -- these poll the GLFW window or drive
  // ImGui, and the OFF build compiles neither that file nor any caller of them.
  void DrawGui();
  bool HandleInputEvent(const vkoo::InputEvent& event);
  bool IsKeyPressed(int keycode);
  void GetMousePosition(float& x_pos, float& y_pos);
  // Back-reference used only by the GUI translation unit, for the handful of
  // calls that genuinely need the application (screenshots, settings I/O,
  // pipeline rebuilds). Set by HexMeshingApp right after construction.
  void SetGuiHost(HexMeshingApp& app) { gui_app_ = &app; }
#endif

  void LoadTargetMesh(const std::string& file_path);
  void LoadResultMesh(const std::string& file_path);
  void ExportTargetComplex(const std::string& selected_file);

  // Programmatic project I/O for the CLI script runner. Mirrors File->Open
  // and File->Save in the menu bar but without the file browser dialog.
  void OpenProject(const std::string& file_path);
  void OpenProjectForStage(const std::string& file_path, int stage_index);
  void SaveProject(const std::string& file_path);

  void UpdateCameraSettings();
  void InitArcBallCamera();

 private:
  void RebuildStages();
#if HEX_ENABLE_VULKAN
  // Gui related. All in GlobalControllerGui.cpp.
  void DrawMainMenuBar();
  void DrawNavigationWindow();
  void DrawStageWindow();
  void DrawRenderingOptions();
  void DrawAdvancedWindow();
#endif

  void RebuildViewsAndStages();

  Settings& settings_;
  // Null in --no-vulkan mode, and in the OFF build always. Pointer rather than
  // reference for exactly that reason; the views it is handed to still take a
  // reference.
  vkoo::Device* device_;
  vkoo::st::Scene& scene_;
#if HEX_ENABLE_VULKAN
  HexMeshingApp* gui_app_{nullptr};
#endif

  std::unique_ptr<DeformationStage> deformation_stage_;
  std::unique_ptr<DecompositionStage> decomposition_stage_;
  std::unique_ptr<DiscretizationStage> discretization_stage_;
  std::unique_ptr<HexahedralizationStage> hexahedralization_stage_;

  std::vector<PipelineStage*> stages_;
  int current_stage_idx_{0};
  ShortcutController shortcut_controller_;

  std::unique_ptr<GlobalState> global_state_;
#if HEX_ENABLE_VULKAN
  // Absent, not empty, when there is no device to allocate against.
  std::unique_ptr<GlobalView> global_view_;
#endif
  // Survives with no renderer -- that is the whole point of Step A2.
  unsigned long visibility_mask_{VisibilityNone};

  uint32_t window_width_;
  uint32_t window_height_;

  std::unique_ptr<vkoo::st::ArcBallCameraScript> arc_ball_camera_script_;
  std::string model_name_;
  std::string project_name_;
  std::string last_visited_path_;
};
}  // namespace hex
