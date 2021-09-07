#pragma once

#include "common.h"

#include <torch/torch.h>

#include "models/TetrahedralMesh.h"
#include "optim/DistortionEnergy.h"

namespace hex {
class CubicVolumetricDeformer {
 public:
  struct Options {
    float cubeness_weight{1.0f};
    float smoothness_weight{1.0f};
    float norm_eps{1e-6f};
    float learning_rate{1e-3};
    Vector2f adam_betas{0.9f, 0.9f};
  };
  CubicVolumetricDeformer(const TetrahedralMesh& target_mesh,
                          const TetrahedralMesh& current_deformed_mesh,
                          const DistortionOptions& distortion_options,
                          const Options& options);

  void Optimize(size_t num_steps);
  Eigen::MatrixXf GetOptimizedPositions() const;

 private:
  void BuildTetJacobians();
  std::vector<torch::Tensor> ComputeLoss();
  torch::Tensor Regularize(torch::Tensor D);

  void PrintLosses(const std::vector<torch::Tensor>& losses);
  torch::Tensor RegularizedL1Norm(torch::Tensor x);
  torch::Tensor RegularizedL2Norm(torch::Tensor x);
  torch::Tensor AlternativeL1Norm(torch::Tensor x);

  DistortionOptions distortion_options_;
  Options options_;

  float total_surface_area_;
  float total_volume_;

  // Constant tensors stored on GPU.
  torch::Tensor original_vertex_positions_;  // |V|x3
  torch::Tensor tets_;                       // |T|x4
  torch::Tensor original_tet_volumes_;       // |T|
  torch::Tensor surface_triangles_;          // |S|x3
  torch::Tensor surface_triangles_areas_;    // |S|
  torch::Tensor surface_adjacent_pairs_;     // |P|x2

  torch::Tensor tet_jacobians_;  // |T|x4x3

  torch::Tensor X_;  // |V|x3, deformed positions

  std::unique_ptr<torch::optim::Optimizer> optimizer_;
  size_t global_step_;
};
}  // namespace hex
