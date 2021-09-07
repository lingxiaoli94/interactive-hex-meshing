#pragma once

#include "common.h"

#include <geomlib/TriangularMeshSampler.h>
#include <geomlib/generalized_projection_info.h>
#include <torch/torch.h>

#include "models/HexComplex.h"
#include "models/TriangularMesh.h"
#include "optim/DistortionEnergy.h"

namespace hex {
class HexComplexDeformer {
 public:
  enum class Mode { FixedBoundary, Projection, LiftedProjection };
  enum class CustomQuality { None, ScaledJacobian };
  enum class Status { Idle, Running };

  struct Options {
    Mode mode{Mode::Projection};
    float projection_weight{1.0f};
    float hausdorff_weight{1.0f};
    float fairness_weight{0.0f};
    float smoothness_weight{1.0f};
    float custom_weight{1.0f};
    CustomQuality custom_quality{CustomQuality::None};
    bool custom_exp_scaling{false};

    bool smooth_transport{false};
    bool use_fixed_samples{false};

    float learning_rate{1e-4};
    Vector2f adam_betas{0.9f, 0.9f};
    int snapshot_freq{-1};
  };

  HexComplexDeformer(const HexComplex& polycube_complex,
                     const HexComplex& target_complex,
                     const TriangularMesh& original_surface,
                     const DistortionOptions& distortion_options,
                     const Options& options);
  ~HexComplexDeformer();
  void SetPulledbackPositions(const std::vector<Vector3f>& positions);
  void Optimize(size_t num_steps,
                const std::vector<int>& fixed_surface_indices);
  std::vector<Vector3f> GetOptimizedPositions() const;
  std::vector<Vector3f> FetchSnapshot();
  bool HasFreshSnapshot() const;
  Status GetStatus() const { return status_; }
  void ForceStop();

 private:
  torch::Tensor BuildHexJacobian(bool include_principal_axes);
  std::vector<torch::Tensor> ComputeLoss();
  void PrepareCustomLoss();
  torch::Tensor Regularize(torch::Tensor D);
  torch::Tensor GetMaskedX();
  std::vector<torch::Tensor> GetLiftedX() const;
  void PrintLosses(const std::vector<torch::Tensor>& losses);

  void ResetOptimizer();

  DistortionOptions distortion_options_;
  Options options_;
  float hex_size_;
  std::function<torch::Tensor(void)> custom_loss_fn_;

  // Constant tensors stored on GPU.
  torch::Tensor original_vertices_;  // for projection
  torch::Tensor original_faces_;
  torch::Tensor original_lifted_vertices_;
  torch::Tensor original_face_normals_;

  std::unique_ptr<geomlib::TriangularProjectionInfo> original_proj_info_;
  std::unique_ptr<geomlib::TriangularMeshSampler> original_sampler_;
  torch::Tensor fixed_original_samples_;

  torch::Tensor surface_indices_;  // surface vertex indices in the hex complex
  torch::Tensor rims_;             // Rx3
  torch::Tensor bi_edges_;         // Ex2
  torch::Tensor rings_;            // optional, 4Qx3
  torch::Tensor surface_degrees_;  // S

  torch::Tensor hex_jacobian_;        // 8x8x3
  torch::Tensor hex_corner_indices_;  // Hx8

  torch::Tensor pulled_back_positions_;  // optional, Nx3

  // Below are variables.
  torch::Tensor X_;                 // Nx3, mapped positions
  torch::Tensor lifted_surface_X_;  // SxD
  std::vector<torch::Tensor> opt_variables_;

  std::unique_ptr<torch::optim::Optimizer> optimizer_;
  size_t global_step_;

  // Snapshot.
  std::vector<Vector3f> snapshot_;
  bool snapshot_fresh_{false};
  mutable std::mutex snapshot_mutex_;
  std::atomic<Status> status_{Status::Idle};
  std::atomic<bool> force_stop_{false};
};
}  // namespace hex
