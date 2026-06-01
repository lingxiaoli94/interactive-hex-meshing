#pragma once

#include "PipelineStage.h"
#include "optim/CubicVolumetricDeformer.h"

namespace YAML {
class Node;
}

namespace hex {
class DeformationStage : public PipelineStage {
 public:
  DeformationStage(GlobalController& global_controller);
  std::string GetName() const override { return "Deformation"; }
  void DrawStageWindow() override;
  void PrepareVolumetricDeformation();
  void Reoptimize(size_t num_steps);

  // Drives the stage from a YAML script config. Reads cubeness/smoothness/
  // distortion weights, learning rate, and step count from the node (using
  // current values as defaults), then runs PrepareVolumetricDeformation() +
  // Reoptimize() in sequence.
  void RunFromScript(const YAML::Node& cfg);

 private:
  void UpdateDeformationViews();
  void InitDeformedMesh();

  std::unique_ptr<CubicVolumetricDeformer> cubic_volumetric_deformer_;

  int reopt_steps_{100};
  DistortionOptions distortion_options_;
  CubicVolumetricDeformer::Options deformer_options_;
};
}  // namespace hex
