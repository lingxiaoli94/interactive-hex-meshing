#pragma once

#include "PipelineStage.h"
#include "optim/CubicVolumetricDeformer.h"

namespace hex {
class DeformationStage : public PipelineStage {
 public:
  DeformationStage(GlobalController& global_controller);
  std::string GetName() const override { return "Deformation"; }
  void DrawStageWindow() override;
  void PrepareVolumetricDeformation();
  void Reoptimize(size_t num_steps);

 private:
  void UpdateDeformationViews();
  void InitDeformedMesh();

  std::unique_ptr<CubicVolumetricDeformer> cubic_volumetric_deformer_;

  int reopt_steps_{100};
  DistortionOptions distortion_options_;
  CubicVolumetricDeformer::Options deformer_options_;
};
}  // namespace hex
