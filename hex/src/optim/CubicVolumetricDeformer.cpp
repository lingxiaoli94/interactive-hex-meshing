#include "CubicVolumetricDeformer.h"

#include <torch/types.h>

#include "debug.h"
#include "logging.h"
#include "optim/DistortionEnergy.h"
#include "torch_utils.h"

using namespace torch::indexing;

namespace hex {
CubicVolumetricDeformer::CubicVolumetricDeformer(
    const TetrahedralMesh& target_mesh,
    const TetrahedralMesh& current_deformed_mesh,
    const DistortionOptions& distortion_options, const Options& options)
    : distortion_options_{distortion_options}, options_{options} {
  original_vertex_positions_ =
      MatrixXfToTensor(target_mesh.GetVertices()).cuda();
  tets_ = MatrixXiToTensor(target_mesh.GetTets()).to(torch::kInt64).cuda();
  original_tet_volumes_ = VectorXfToTensor(target_mesh.GetTetVolumes()).cuda();

  total_volume_ = original_tet_volumes_.sum().item<float>();
  LOGI("Total tet volume: {}", total_volume_);

  surface_triangles_ = MatrixXiToTensor(target_mesh.GetSurfaceTriangles())
                           .to(torch::kInt64)
                           .cuda();
  surface_triangles_areas_ =
      VectorXfToTensor(target_mesh.GetSurfaceMesh().GetFaceAreas()).cuda();
  total_surface_area_ = surface_triangles_areas_.sum().item<float>();
  LOGI("Total surface area: {}", total_surface_area_);

  surface_adjacent_pairs_ =
      MatrixXiToTensor(target_mesh.GetSurfaceMesh().GetAdjacentFacePairs())
          .to(torch::kInt64)
          .cuda();
  LOGI("Surface has {} adjacent triangle pairs.",
       surface_adjacent_pairs_.size(0));

  X_ = MatrixXfToTensor(current_deformed_mesh.GetVertices()).cuda();
  X_.requires_grad_(true);

  BuildTetJacobians();

  optimizer_ = std::make_unique<torch::optim::Adam>(
      std::vector<torch::Tensor>{X_},
      torch::optim::AdamOptions(options_.learning_rate)
          .betas({options.adam_betas(0), options.adam_betas(1)}));
  global_step_ = 0;
}

void CubicVolumetricDeformer::BuildTetJacobians() {
  Eigen::MatrixXf D_mat(4, 3);
  D_mat << -1, -1, -1, 1, 0, 0, 0, 1, 0, 0, 0, 1;
  auto D = MatrixXfToTensor(D_mat).cuda();  // 4x3
  std::vector<torch::Tensor> corners;
  for (int k = 0; k < 4; k++) {
    corners.push_back(
        original_vertex_positions_.index({tets_.index({Slice(), k}), Slice()}));
  }
  auto S = torch::stack({corners[1] - corners[0], corners[2] - corners[0],
                         corners[3] - corners[0]},
                        -1);  // |T|x3x3
  // Inversion can fail here if input mesh has degenerated tets.
  auto Sinv = torch::linalg_pinv(S.to(torch::kFloat64)).to(torch::kFloat32);

  tet_jacobians_ = torch::matmul(D.unsqueeze(0).repeat({tets_.size(0), 1, 1}),
                                 Sinv);  // |T|x4x3
}

std::vector<torch::Tensor> CubicVolumetricDeformer::ComputeLoss() {
  auto tet_corners =
      X_.index({tets_.reshape({-1}), Slice()}).reshape({-1, 4, 3});  // |T|x4x3
  auto J =
      torch::matmul(tet_corners.transpose(1, 2), tet_jacobians_);  // |T|x3x3

  auto distortion_loss = ComputeDistortionLoss(
      J, original_tet_volumes_ / total_volume_, distortion_options_);  // |T|

  // Next compute cubeness loss.
  torch::Tensor normals;
  {
    std::vector<torch::Tensor> triangle_corners;  // each element: |S|x3
    for (int k = 0; k < 3; k++) {
      triangle_corners.push_back(
          X_.index({surface_triangles_.index({Slice(), k}), Slice()}));
      assert(!triangle_corners.back().isnan().any().item<bool>());
    }
    normals = torch::cross(triangle_corners[1] - triangle_corners[0],
                           triangle_corners[2] - triangle_corners[0]);  // |S|x3
  }
  auto normals_unit = normals / RegularizedL2Norm(normals).unsqueeze(-1);
  // Note: area is already incorporated in the unnormalized normals.

  auto current_total_area = (normals.norm(2, -1) / 2).sum();

  auto l1_loss =
      options_.cubeness_weight *
      (surface_triangles_areas_ * AlternativeL1Norm(normals_unit)).sum() /
      total_surface_area_;

  torch::Tensor smoothness_loss;
  /*{
    // Next compute smoothness loss based on normals.
    auto n_fi = normals.index(
        {surface_adjacent_pairs_.index({Slice(), 0}), Slice()});  // |P|x3
    auto n_fj = normals.index(
        {surface_adjacent_pairs_.index({Slice(), 1}), Slice()});  // |P|x3
    auto n_fi_norm =
        RegularizedL2Norm(n_fi).unsqueeze(1).repeat({1, 3});  // |P|x3
    auto n_fj_norm =
        RegularizedL2Norm(n_fj).unsqueeze(1).repeat({1, 3});  // |P|x3
    auto normal_diff = torch::where(
        n_fi_norm <= n_fj_norm, n_fi - n_fi_norm / n_fj_norm * n_fj,
        n_fj - n_fj_norm / n_fi_norm * n_fi);  // |P|x3
    smoothness_loss = smoothness_weight_ *
                      RegularizedL2Norm(normal_diff).sum() /
                      (3 * total_surface_area_);
  }*/
  {
    // Next compute smoothness loss based on normals.
    auto n_fi = normals_unit.index(
        {surface_adjacent_pairs_.index({Slice(), 0}), Slice()});  // |P|x3
    auto n_fj = normals_unit.index(
        {surface_adjacent_pairs_.index({Slice(), 1}), Slice()});  // |P|x3
    auto neighbor_area_sum =
        surface_triangles_areas_.index(
            {surface_adjacent_pairs_.index({Slice(), 0})}) +
        surface_triangles_areas_.index(
            {surface_adjacent_pairs_.index({Slice(), 1})});
    smoothness_loss =
        options_.smoothness_weight *
        (neighbor_area_sum * (n_fi - n_fj).square().sum(-1)).sum() /
        (3 * total_surface_area_);
  }
  assert(IsTensorFinite(distortion_loss));
  assert(IsTensorFinite(l1_loss));
  assert(IsTensorFinite(smoothness_loss));

  return {distortion_loss, l1_loss, smoothness_loss};
}

torch::Tensor CubicVolumetricDeformer::RegularizedL2Norm(torch::Tensor x) {
  // Assuming x is Bx3.
  return torch::sqrt(x.square().sum(-1) +
                     options_.norm_eps * options_.norm_eps);
}

torch::Tensor CubicVolumetricDeformer::RegularizedL1Norm(torch::Tensor x) {
  // Assuming x is Bx3.
  return torch::sqrt(x.square() + options_.norm_eps * options_.norm_eps)
      .sum(-1);
}

torch::Tensor CubicVolumetricDeformer::AlternativeL1Norm(torch::Tensor v) {
  auto nx = v.index({Slice(), 0}).square();
  auto ny = v.index({Slice(), 1}).square();
  auto nz = v.index({Slice(), 2}).square();

  return nx * ny + ny * nz + nz * nx;
}

void CubicVolumetricDeformer::Optimize(size_t num_steps) {
  auto losses_before = ComputeLoss();
  LOGI("Before cubic volume deform: ");
  PrintLosses(losses_before);
  size_t target_step = global_step_ + num_steps;
  while (global_step_ < target_step) {
    global_step_++;

    auto closure = [&]() {
      auto losses = ComputeLoss();
      auto loss = losses[0] + losses[1] + losses[2];

      optimizer_->zero_grad();
      loss.backward();

      auto X_grad = X_.grad();
      if (X_grad.isnan().any().item<bool>()) {
        LOGW("X_grad has NaN! Set to 0.");
        X_.mutable_grad() = X_.grad().nan_to_num(0.0f);
      }
      return loss;
    };
    optimizer_->step(closure);
  }
  auto loss_after = ComputeLoss();
  LOGI("After cubic volume deform: ");
  PrintLosses(loss_after);
}

Eigen::MatrixXf CubicVolumetricDeformer::GetOptimizedPositions() const {
  return TensorToMatrixXf(X_.cpu().detach());
}

void CubicVolumetricDeformer::PrintLosses(
    const std::vector<torch::Tensor>& losses) {
  LOGI("distortion_loss: {} | l1_loss: {} | smoothness_loss: {}",
       losses[0].item<float>(), losses[1].item<float>(),
       losses[2].item<float>());
}
}  // namespace hex
