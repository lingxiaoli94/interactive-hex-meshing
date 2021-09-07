#pragma once

#include "optim/PolycubeOptimizer.h"

#include "PipelineStage.h"
#include "controllers/CuboidEditingController.h"
#include "models/TetrahedralMesh.h"
#include "views/CuboidNode.h"

namespace hex {
class DecompositionStage : public PipelineStage {
 public:
  DecompositionStage(GlobalController& global_controller);
  ~DecompositionStage();
  std::string GetName() const override { return "Decomposition"; }
  void Update(float delta_time) override;
  void DrawStageWindow() override;
  bool CanSwitchTo() override;
  void SwitchTo() override;
  void SwitchFrom() override;
  bool HandleInputEvent(const vkoo::InputEvent& event) override;

  void EstimateScale(const TetrahedralMesh& mesh);
  void ResetPolycube();
  void AddNewCuboid(const Cuboid& cuboid);
  void DuplicateCuboid(int i);
  void DeleteCuboid(int i);
  void Reoptimize(size_t num_steps);
  void SuggestNewCuboid(PolycubeOptimizer::SuggestStrategy strategy);

  void FocusCuboid(int i);
  void UnfocusCuboid();

 private:
  void CreateSdfAndAnchors();

  void StartSubtractCuboid();
  void PerformSubtraction();

  void DrawAnchorsSdfChildWindow();
  void DrawPolycubeControlChildWindow();
  void DrawPolycubeInfoWindow();
  void DrawPolycubeOptimizerTreeNode();
  void DrawPolycubeManipulationTreeNode();

  std::unique_ptr<PolycubeOptimizer>
      polycube_optimizer_;
  std::thread opt_thread_;

  int focused_cuboid_index_{-1};
  CuboidEditingController cuboid_editing_controller_;
  std::unique_ptr<CuboidEditingController> subtract_editing_controller_;
  std::unique_ptr<Cuboid> subtract_cuboid_;

  int reoptimize_steps_{1000};
  PolycubeOptimizer::Options options_;

  struct {
    int grid_size{24};
    bool inside_only{false};
    float bbox_padding{0.2f};
    int surface_samples{20000};
    float perturbation{0.10f};
  } anchors_options_;
};
}  // namespace hex
