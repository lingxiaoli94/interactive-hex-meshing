#pragma once

#include "optim/PolycubeOptimizer.h"

#include "PipelineStage.h"
#include "models/TetrahedralMesh.h"
#if HEX_ENABLE_VULKAN
#include "controllers/CuboidEditingController.h"
#include "views/CuboidNode.h"
#endif

namespace YAML {
class Node;
}

namespace hex {
class DecompositionStage : public PipelineStage {
 public:
  DecompositionStage(GlobalController& global_controller);
  ~DecompositionStage();
  std::string GetName() const override { return "Decomposition"; }
  bool CanSwitchTo() override;
  void SwitchTo() override;
  void SwitchFrom() override;
#if HEX_ENABLE_VULKAN
  // Defined in DecompositionStageGui.cpp. Declared only in the ON build: a
  // virtual override must be defined even when never called, and PipelineStage
  // already supplies a do-nothing base for all three.
  void Update(float delta_time) override;
  void DrawStageWindow() override;
  bool HandleInputEvent(const vkoo::InputEvent& event) override;
#endif

  void EstimateScale(const TetrahedralMesh& mesh);
  void CreateSdfAndAnchors();
  void ResetPolycube();
  void AddNewCuboid(const Cuboid& cuboid);
  void DuplicateCuboid(int i);
  void DeleteCuboid(int i);
  void Reoptimize(size_t num_steps);
  void SuggestNewCuboid(PolycubeOptimizer::SuggestStrategy strategy);

  void FocusCuboid(int i);
  // Joins the optimizer thread, then drops the cuboid highlight. The join is
  // the load-bearing half and runs in every display mode; see
  // JoinOptimizerThread below.
  void UnfocusCuboid();

  // Drives the stage from a YAML script config. Greedy auto-add N cuboids
  // then run Reoptimize() blocking on the optimizer thread.
  void RunFromScript(const YAML::Node& cfg);

 private:
  // Split out of UnfocusCuboid(). Reoptimize() and ResetPolycube() both reach
  // UnfocusCuboid on the script path, and skipping the join to skip the view
  // work would race the optimizer thread.
  void JoinOptimizerThread();

#if HEX_ENABLE_VULKAN
  void StartSubtractCuboid();
#endif
#if HEX_ENABLE_VULKAN
  void PerformSubtraction();
#endif

#if HEX_ENABLE_VULKAN
  void DrawAnchorsSdfChildWindow();
#endif
#if HEX_ENABLE_VULKAN
  void DrawPolycubeControlChildWindow();
#endif
#if HEX_ENABLE_VULKAN
  void DrawPolycubeInfoWindow();
#endif
#if HEX_ENABLE_VULKAN
  void DrawPolycubeOptimizerTreeNode();
#endif
#if HEX_ENABLE_VULKAN
  void DrawPolycubeManipulationTreeNode();
#endif

  std::unique_ptr<PolycubeOptimizer>
      polycube_optimizer_;
  std::thread opt_thread_;

  // Kept in both variants: it is the index of the cuboid an edit applies to,
  // and UnfocusCuboid pairs against it whether or not anything is drawn.
  int focused_cuboid_index_{-1};
#if HEX_ENABLE_VULKAN
  // unique_ptr, not a plain member: its constructor allocates Vulkan
  // primitives, so with no device it is absent rather than half-built.
  std::unique_ptr<CuboidEditingController> cuboid_editing_controller_;
  std::unique_ptr<CuboidEditingController> subtract_editing_controller_;
#endif
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
