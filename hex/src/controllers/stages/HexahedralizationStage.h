#pragma once

#include <vkoo/st/Node.h>

#include "PipelineStage.h"
#include "optim/DistortionEnergy.h"
#include "optim/HexComplexDeformer.h"
#include "views/FilteredHexMesh.h"
#include "views/LandmarksEditingView.h"
#include "views/QuadSurfaceView.h"
#include "views/TriSurfaceView.h"

namespace hex {
class HexahedralizationStage : public PipelineStage {
 public:
  HexahedralizationStage(GlobalController& global_controller);
  ~HexahedralizationStage();
  std::string GetName() const override { return "Hexahedralization"; }
  bool CanSwitchTo() override;
  void SwitchTo() override;
  void SwitchFrom() override;
  void DrawStageWindow() override;
  bool HandleInputEvent(const vkoo::InputEvent& event) override;
  void Update(float delta_time) override;

  void InitTargetComplex(bool use_pullback, bool indirect, bool transport);
  void PrepareHexDeformation();
  void OptimizeHexDeformation(int steps);
  void Reset();

 private:
  void DrawHexahedralizationWindow();
  void DrawFilteringWindow();
  void DrawQualityParametersChildWindow();
  void DrawOptimizationControlWindow();

  void UpdateOnTargetComplexChange();
  void UpdateCurrentSurface();
  void UpdateFilteredMeshView();
  std::vector<Vector3f> PullPolycubeVolumeBack(bool indirect);
  void PutLandmark();
  size_t FindClosestVertexOnQuad(const Vector3f& p, size_t quad_id);
  void ComputeHausdorffDistance();

  void EnterEditMode();
  void LeaveEditMode();
  void ShowGuideSurface();
  void HideGuideSurface();
  void ClearLandmarks();

  void FetchSnapshot();
  void StopAndJoinOptThread();

  std::thread opt_thread_;
  std::unique_ptr<HexComplexDeformer> hex_complex_deformer_;

  DistortionOptions distortion_options_;
  HexComplexDeformer::Options hex_deformer_options_;

  FilteringSetting filtering_setting_;

  // Variables for supporting landmarks.
  bool is_editing_{false};
  std::unique_ptr<QuadrilateralMesh> current_surface_mesh_;
  bool is_being_dragged_{false};
  size_t dragged_vtx_id_;
  Vector3f dragged_vtx_position_;
  bool deformer_dirty_{false};

  bool in_transition_{false};

  std::tuple<float, float> hausdorff_distance_{0.0f, 0.0f};

  std::unique_ptr<QuadSurfaceView> pickable_surface_view_;
  std::unique_ptr<LandmarksEditingView> landmarks_view_;

  std::unordered_set<size_t> landmarks_set_;  // indices are in result hex mesh
  unsigned long saved_visibility_flag_;
};
}  // namespace hex
