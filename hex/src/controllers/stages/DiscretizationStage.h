#pragma once

#include "PipelineStage.h"
#include "models/PolycubeGraph.h"
#include "models/QuadrilateralMesh.h"
#include "views/HexCollectionView.h"

namespace hex {
class DiscretizationStage : public PipelineStage {
 public:
  DiscretizationStage(GlobalController& global_controller);
  std::string GetName() const override { return "Discretization"; }
  bool CanSwitchTo() override;
  void SwitchTo() override;
  void SwitchFrom() override;
  bool HandleInputEvent(const vkoo::InputEvent& event) override;
  void DrawStageWindow() override;

  void DiscretizePolycube();
  void FinalizePolycube();

 private:
  enum class Status { Idle, Digging, Extrusion };

  void ApplyPendingChanges();
  void HandleQuadClicked(size_t quad_id);

  float hex_size_{0.05f};
  bool round_to_nearest_{false};
  bool padding_{true};
  bool color_by_patch_{true};

  int mode_{0};  // 0: dig, 1: extrude
  Status status_{Status::Idle};
  std::unique_ptr<PolycubeGraph> current_graph_;
  std::unique_ptr<QuadComplex> current_quad_complex_;
  std::unique_ptr<QuadrilateralMesh> current_surface_mesh_;

  std::unordered_set<Vector3i, Vector3iHasher> pending_hex_coords_;
  std::unique_ptr<HexCollectionView> pending_view_;
};
}  // namespace hex
