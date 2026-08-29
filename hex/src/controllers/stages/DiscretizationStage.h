#pragma once

#include "PipelineStage.h"
#include "models/PolycubeGraph.h"
#include "models/QuadrilateralMesh.h"
#if HEX_ENABLE_VULKAN
#include "views/HexCollectionView.h"
#endif

namespace YAML {
class Node;
}

namespace hex {
class DiscretizationStage : public PipelineStage {
 public:
  DiscretizationStage(GlobalController& global_controller);
  std::string GetName() const override { return "Discretization"; }
  bool CanSwitchTo() override;
  void SwitchTo() override;
  void SwitchFrom() override;
#if HEX_ENABLE_VULKAN
  // Defined in the *Gui.cpp translation unit. Declared only in the ON
  // build: a virtual override must be defined even when never called,
  // and PipelineStage already supplies a do-nothing base.
  void DrawStageWindow() override;
  bool HandleInputEvent(const vkoo::InputEvent& event) override;
#endif

  void DiscretizePolycube();
  void FinalizePolycube();

  // Drives the stage from a YAML script config. Reads hex_size, padding,
  // round_to_nearest from the node (using current values as defaults), then
  // runs DiscretizePolycube() + FinalizePolycube() in sequence.
  void RunFromScript(const YAML::Node& cfg);

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
#if HEX_ENABLE_VULKAN
#if HEX_ENABLE_VULKAN
  std::unique_ptr<HexCollectionView> pending_view_;
#endif
#endif
};
}  // namespace hex
