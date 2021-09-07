#pragma once

#include "common.h"

#include <torch/torch.h>

#include "models/Polycube.h"
#include "models/TetrahedralMesh.h"
#include "optim/DirectModel.h"

namespace hex {
class PolycubeOptimizer {
 public:
  enum class Status { Idle, Running, Completed };
  enum class SuggestStrategy { Simple, Largest };

  struct Options {
    float positive_l2_weight{1.0f};
    float negative_l2_weight{1.0f};
    float learning_rate{5e-3};
    Vector2f adam_betas{0.9f, 0.9f};
    int snapshot_freq{50};
  };

  PolycubeOptimizer(Options options);
  void Optimize(const TetrahedralMesh& mesh, const Polycube& polycube,
                size_t num_steps, const std::vector<int>& locked);
  bool SuggestNewCuboid(const TetrahedralMesh& mesh, const Polycube& polycube,
                        SuggestStrategy strategy, Cuboid& new_cuboid);
  bool SuggestSubtractCuboid(const TetrahedralMesh& mesh,
                             const Polycube& polycube, Cuboid& subtract_cuboid);
  float ComputeCurrentLoss();
  Polycube GetSnapshot();
  Status GetStatus();
  void SetIdle();
  bool IsIdle();
  void Stop();

 private:
  void Prepare(const TetrahedralMesh& mesh, const Polycube& polycube,
               const std::vector<int>& locked);
  void OptimizeSteps(size_t num_steps);

  void PrepareBBox(const TetrahedralMesh& mesh);
  void PrepareGPUMeshData(const TetrahedralMesh& target_mesh);
  std::tuple<torch::Tensor, Vector3f> CreateAmbientGrid(int discrete_num,
                                                        float padding = 0.0f);

  Polycube GetOptimizedPolycube();

  torch::Tensor TransformToCuboidParams(const torch::Tensor& weight);
  torch::Tensor TransformToWeight(const torch::Tensor& cuboid_params);
  torch::Tensor TransformToCuboidParams(const Polycube& polycube);
  Polycube TransformToPolycube(const torch::Tensor& cuboid_params);
  torch::Tensor ComputeLoss(const torch::Tensor& cuboid_params,
                            const torch::Tensor& polycube_sdf);
  torch::Tensor CombineCuboidParams(bool preserve_order);

  Options options_;
  size_t global_step_;

  // Indices of cuboids that are being optimized/locked, etc.
  torch::Tensor var_indices_;
  torch::Tensor fixed_indices_;

  // Variable to optimize. Cuboid parameters are scaled version of weights_.
  torch::Tensor weights_;
  torch::Tensor fixed_params_;

  // GPU copies of mesh data.
  torch::Tensor mesh_sdf_;
  torch::Tensor anchors_;

  // Derived small tensors, used for computing transformations.
  torch::Tensor bbox_[2];
  torch::Tensor max_b_;

  std::unique_ptr<torch::optim::Optimizer> optimizer_;

  torch::Device device_;

  // Snapshot.
  Polycube snapshot_;
  std::mutex snapshot_mutex_;
  std::atomic<Status> status_{Status::Idle};
  std::atomic<bool> stopped_{false};
};

}  // namespace hex
