#include "HexComplexDeformer.h"

#include <ATen/Functions.h>
#include <c10/cuda/CUDACachingAllocator.h>
#include <geomlib/autograd/GeneralizedProjection.h>

#include "geomlib/TriangularMeshSampler.h"
#include "geomlib/generalized_projection_info.h"
#include "logging.h"
#include "models/HexConvention.h"
#include "optim/DistortionEnergy.h"
#include "torch_utils.h"
#include "utility/StopWatch.h"
using namespace torch::indexing;

namespace hex {
namespace {
const float kNormalAlignExpCoeff = 1.0f;
const float kCustomExpCoeff = 10.0f;
}  // namespace

HexComplexDeformer::HexComplexDeformer(
    const HexComplex& polycube_complex, const HexComplex& target_complex,
    const TriangularMesh& original_surface,
    const DistortionOptions& distortion_options, const Options& options)
    : distortion_options_{distortion_options}, options_{options} {
  // The assumption is that polycube_complex and target_complex share the same
  // topology, but we use target_complex's current vertex positions as
  // initialization.

  {
    // Estimate hex element size.
    auto& vertices = polycube_complex.GetVertices();
    auto& quad = polycube_complex.GetQuads().at(0);
    hex_size_ = (vertices[quad[0]] - vertices[quad[1]]).norm();
  }
  hex_jacobian_ = BuildHexJacobian(false) / hex_size_;

  original_vertices_ = MatrixXfToTensor(original_surface.GetVertices()).cuda();
  original_faces_ = MatrixXiToTensor(original_surface.GetFaces()).cuda();
  original_face_normals_ =
      MatrixXfToTensor(original_surface.GetFaceNormals()).cuda();
  original_face_normals_ /= original_face_normals_.norm(2, -1, true) + 1e-8f;
  original_proj_info_ = std::make_unique<geomlib::TriangularProjectionInfo>(
      original_vertices_, original_faces_);
  original_sampler_ = std::make_unique<geomlib::TriangularMeshSampler>(
      original_vertices_, original_faces_.to(torch::kInt64));

  auto& hexes = polycube_complex.GetHexes();
  Eigen::MatrixXi hex_mat(hexes.size(), 8);
  for (size_t i = 0; i < hexes.size(); i++) {
    for (size_t j = 0; j < 8; j++) {
      hex_mat(i, j) = static_cast<int>(hexes[i][j]);
    }
  }
  hex_corner_indices_ = MatrixXiToTensor(hex_mat).to(torch::kInt64).cuda();

  {
    auto& surface_indices_uint = polycube_complex.GetSurfaceIndices();
    // Convert to std;:vector<int>.
    std::vector<int> surface_indices;
    for (auto x : surface_indices_uint) {
      surface_indices.push_back(static_cast<int>(x));
    }
    surface_indices_ =
        torch::from_blob(surface_indices.data(), {(int)surface_indices.size()},
                         torch::kInt32)
            .to(torch::kInt64)
            .cuda();

    rims_ = MatrixXiToTensor(ArrayVector3iToMatrixXi(
                                 polycube_complex.GetQuadComplex().GetRims()))
                .to(torch::kInt64)
                .cuda();

    bi_edges_ =
        MatrixXiToTensor(ArrayVector2iToMatrixXi(
                             polycube_complex.GetQuadComplex().GetBiEdges()))
            .to(torch::kInt64)
            .cuda();

    rings_ = MatrixXiToTensor(ArrayVector3iToMatrixXi(
                                  polycube_complex.GetQuadComplex().GetRing()))
                 .to(torch::kInt64)
                 .cuda();

    auto degrees = polycube_complex.GetQuadComplex().GetDegrees();
    surface_degrees_ =
        torch::from_blob(degrees.data(), {(int)degrees.size()}, torch::kInt32)
            .to(torch::kInt64)
            .cuda();
  }

  // Initialize positions to be the same as the ones in the complex (in
  // particular the surface positions should come from quad deformation).
  X_ = MatrixXfToTensor(ArrayVector3fToMatrixXf(target_complex.GetVertices()))
           .cuda();
  opt_variables_ = std::vector<torch::Tensor>{X_};
  if (options_.mode == Mode::LiftedProjection) {
    auto surface_X = X_.index({surface_indices_, Slice()});
    auto proj_result = geomlib::GeneralizedTriangleProjection<3>::apply(
        surface_X, *original_proj_info_);
    lifted_surface_X_ = surface_X.clone().detach();
    lifted_surface_X_.requires_grad_(true);
    opt_variables_.push_back(lifted_surface_X_);
  }
  X_.requires_grad_(true);
  global_step_ = 0;
  ResetOptimizer();

  PrepareCustomLoss();
}

void HexComplexDeformer::SetPulledbackPositions(
    const std::vector<Vector3f>& positions) {
  pulled_back_positions_ =
      MatrixXfToTensor(ArrayVector3fToMatrixXf(positions)).cuda();
}

void HexComplexDeformer::ResetOptimizer() {
  // TODO: make this user adjustable.
  optimizer_ = std::make_unique<torch::optim::Adam>(
      opt_variables_,
      torch::optim::AdamOptions(options_.learning_rate)
          .betas({options_.adam_betas(0), options_.adam_betas(1)}));
}

HexComplexDeformer::~HexComplexDeformer() {
  c10::cuda::CUDACachingAllocator::emptyCache();
}

torch::Tensor HexComplexDeformer::BuildHexJacobian(
    bool include_principal_axes) {
  auto& hex_jacobians = HexConvention::GetCornerJacobians();
  std::vector<torch::Tensor> hex_jacobian_tensors;
  for (int i = 0; i < 8; i++) {
    hex_jacobian_tensors.push_back(MatrixXfToTensor(hex_jacobians[i]));
  }

  if (include_principal_axes) {
    hex_jacobian_tensors.push_back(
        MatrixXfToTensor(HexConvention::GetPrincipleAxes()));
  }
  auto hex_jacobian =
      torch::stack(hex_jacobian_tensors, 0).cuda();  // 8x8x3 or 9x8x3

  return hex_jacobian;
}

void HexComplexDeformer::ForceStop() { force_stop_ = true; }

void HexComplexDeformer::Optimize(
    size_t num_steps, const std::vector<int>& fixed_surface_indices) {
  assert(status_ == Status::Idle);
  status_ = Status::Running;
  force_stop_ = false;
  auto losses_before = ComputeLoss();
  LOGI("Before hex deform: ");
  PrintLosses(losses_before);
  size_t target_step = global_step_ + num_steps;

  StopWatch watch;
  watch.Tic();

  torch::Tensor fixed_indices;
  if (!fixed_surface_indices.empty()) {
    ResetOptimizer();  // clear out previously-stored gradients

    std::vector<int> fixed_indices_copy = fixed_surface_indices;
    fixed_indices =
        torch::from_blob(fixed_indices_copy.data(),
                         {(int)fixed_surface_indices.size()}, torch::kInt32)
            .to(torch::kInt64)
            .cuda();
  }
  while (global_step_ < target_step && !force_stop_) {
    global_step_++;

    auto losses = ComputeLoss();
    auto loss = losses[0] + losses[1] + losses[2] + losses[3] + losses[4];

    optimizer_->zero_grad();
    loss.backward();

    if (!fixed_surface_indices.empty()) {
      // Zero-out gradients for fixed indices.
      if (options_.mode == Mode::LiftedProjection) {
        lifted_surface_X_.mutable_grad().scatter_(
            0, fixed_indices.unsqueeze(1).repeat({1, 3}),
            torch::zeros({fixed_indices.size(0), 3},
                         lifted_surface_X_.options()));
      } else {
        auto indices = surface_indices_.index({fixed_indices});
        X_.mutable_grad().scatter_(
            0, indices.unsqueeze(1).repeat({1, 3}),
            torch::zeros({indices.size(0), 3}, X_.options()));
      }
    }

    optimizer_->step();
    if (options_.snapshot_freq != -1 &&
        global_step_ % options_.snapshot_freq == 0) {
      std::lock_guard<std::mutex> lock_guard{snapshot_mutex_};
      snapshot_ = GetOptimizedPositions();
      snapshot_fresh_ = true;
    }
  }
  auto loss_after = ComputeLoss();
  LOGI("Took {} sec(s). After hex deform: ", watch.Toc().count());
  PrintLosses(loss_after);
  {
    std::lock_guard<std::mutex> lock_guard{snapshot_mutex_};
    snapshot_ = GetOptimizedPositions();
    snapshot_fresh_ = true;
  }
  status_ = Status::Idle;
}

std::vector<Vector3f> HexComplexDeformer::FetchSnapshot() {
  std::lock_guard<std::mutex> lock_guard{snapshot_mutex_};
  snapshot_fresh_ = false;
  return snapshot_;
}

bool HexComplexDeformer::HasFreshSnapshot() const {
  std::lock_guard<std::mutex> lock_guard{snapshot_mutex_};
  return snapshot_fresh_;
}

std::vector<torch::Tensor> HexComplexDeformer::ComputeLoss() {
  torch::Tensor X;
  torch::Tensor proj_dist;
  if (options_.mode == Mode::LiftedProjection) {
    auto tmp = GetLiftedX();
    X = tmp[0];
    proj_dist = tmp[1];
  } else if (options_.mode == Mode::FixedBoundary) {
    X = GetMaskedX();
  } else {
    assert(options_.mode == Mode::Projection);
    X = X_;  // Nx3
  }
  auto hex_corners = X.index({hex_corner_indices_.reshape({-1}), Slice()})
                         .reshape({-1, 8, 3});  // Hx8x3
  auto J = torch::matmul(hex_corners.transpose(1, 2).unsqueeze(1),
                         hex_jacobian_.unsqueeze(0));  // Hx8x3x3

  // Equal weight since all hexes in polycube have the same volume.
  auto distortion_weight =
      torch::ones({J.size(0) * J.size(1)}, torch::kFloat32).cuda() /
      (J.size(0) * J.size(1));
  auto distortion_loss = ComputeDistortionLoss(
      J.reshape({-1, 3, 3}), distortion_weight, distortion_options_);

  auto proj_loss = torch::zeros_like(distortion_loss);

  if (options_.smooth_transport && pulled_back_positions_.dim() > 0) {
    assert(options_.mode == Mode::Projection);
    proj_loss = (X - pulled_back_positions_).square().sum();
  } else {
    if (options_.mode == Mode::Projection ||
        options_.mode == Mode::LiftedProjection) {
      auto X_surface = X.index({surface_indices_, Slice()});
      if (options_.mode == Mode::Projection) {
        // Add in projection loss.
        auto proj_result = geomlib::GeneralizedTriangleProjection<3>::apply(
            X_surface, *original_proj_info_);
        auto proj_face_idx = proj_result[1];
        auto weight =
            options_.projection_weight * torch::ones_like(distortion_loss);
        {
          // Calculate projection weight depending on normal alignment.
          auto proj_face_normals =
              original_face_normals_.index({proj_face_idx, Slice()});

          auto ring_u = X_surface.index({rings_.index({Slice(), 0}), Slice()});
          auto ring_v = X_surface.index({rings_.index({Slice(), 1}), Slice()});
          auto ring_w = X_surface.index({rings_.index({Slice(), 2}), Slice()});
          auto ring_normals = torch::cross(ring_v - ring_u, ring_w - ring_u);
          // No normalization here.
          auto indices = rings_.index({Slice(), 0}).unsqueeze(1).repeat({1, 3});
          auto vertex_normals = torch::scatter_add(torch::zeros_like(X_surface),
                                                   0, indices, ring_normals);
          vertex_normals /= vertex_normals.norm(2, 1, true) + 1e-8f;

          weight = (vertex_normals *
                    original_face_normals_.index({proj_face_idx, Slice()}))
                       .sum(-1);
          weight = options_.projection_weight *
                   torch::exp(kNormalAlignExpCoeff * (weight - 1));
          weight = weight.detach();  // must detach here; otherwise scatter_add
          // cannot backprop
        }

        auto Y_surface = original_proj_info_
                             ->GetPointsAtBarycentricCoordinates(
                                 proj_face_idx, proj_result[3], proj_result[4])
                             .detach();

        proj_loss = (weight * (X_surface - Y_surface).square().sum(-1)).sum();
      } else {
        proj_loss = options_.projection_weight * proj_dist.sum();
      }

      // Add input mesh to current surface distance.
      // This part is a bit slow so make it optional.
      if (options_.hausdorff_weight > 0) {
        torch::Tensor mesh_samples;
        if (options_.use_fixed_samples) {
          if (!fixed_original_samples_.numel()) {
            fixed_original_samples_ =
                original_sampler_->Sample(surface_indices_.size(0));
          }
          mesh_samples = fixed_original_samples_;
        } else {
          mesh_samples = original_sampler_->Sample(surface_indices_.size(0));
        }
        geomlib::TriangularProjectionInfo X_proj_info{X_surface.detach(),
                                                      rings_};
        auto proj_result = geomlib::GeneralizedTriangleProjection<3>::apply(
            mesh_samples, X_proj_info);
        auto face_ids = proj_result[1];
        auto v0 = X_surface.index({rings_.index({face_ids, 0}), Slice()});
        auto v1 = X_surface.index({rings_.index({face_ids, 1}), Slice()});
        auto v2 = X_surface.index({rings_.index({face_ids, 2}), Slice()});

        auto projected_samples = v0 * proj_result[2].detach().unsqueeze(1) +
                                 v1 * proj_result[3].detach().unsqueeze(1) +
                                 v2 * proj_result[4].detach().unsqueeze(1);
        proj_loss += options_.hausdorff_weight *
                     (mesh_samples - projected_samples).square().sum(-1).sum();
      }
    }
  }

  auto fairness_loss = torch::zeros_like(distortion_loss);
  auto smoothness_loss = torch::zeros_like(distortion_loss);
  if (options_.mode != Mode::FixedBoundary) {
    // Fairness and smoothness losses.
    auto surface_X = X.index({surface_indices_, Slice()});
    auto L = surface_X.index({rims_.index({Slice(), 0}), Slice()}) -
             2 * surface_X.index({rims_.index({Slice(), 1}), Slice()}) +
             surface_X.index({rims_.index({Slice(), 2}), Slice()});
    fairness_loss = options_.fairness_weight * L.square().sum(-1).sum();

    {
      auto src =
          surface_X.index({bi_edges_.index({Slice(), 1}), Slice()});  // Ex3
      auto index =
          bi_edges_.index({Slice(), 0}).unsqueeze(1).repeat({1, 3});  // Ex3
      auto avg = torch::scatter_add(torch::zeros_like(surface_X), 0, index,
                                    src);  // Sx3
      avg = avg /
            (surface_degrees_.to(torch::kFloat32).unsqueeze(1).repeat({1, 3}));
      smoothness_loss =
          10 * options_.smoothness_weight *
          (surface_X - avg).square().sum();  // FIXME: artificial scaling
    }
  }

  auto custom_loss = options_.custom_weight * custom_loss_fn_();

  return {distortion_loss, proj_loss, fairness_loss, smoothness_loss,
          custom_loss};
}

void HexComplexDeformer::PrepareCustomLoss() {
  if (options_.custom_quality == CustomQuality::None) {
    custom_loss_fn_ = []() {
      return torch::zeros({1}, torch::kFloat32).cuda();
    };
  } else if (options_.custom_quality == CustomQuality::ScaledJacobian) {
    auto aug_hex_jacobians =
        BuildHexJacobian(true);  // 9x8x3, lives in a closure
    custom_loss_fn_ = [aug_hex_jacobians, this]() {
      torch::Tensor X;
      if (options_.mode == Mode::LiftedProjection) {
        auto tmp = GetLiftedX();
        X = tmp[0];
      } else if (options_.mode == Mode::FixedBoundary) {
        X = GetMaskedX();
      } else {
        assert(options_.mode == Mode::Projection ||
               options_.mode == Mode::Riemannian);
        X = X_;
      }

      auto hex_corners = X.index({hex_corner_indices_.reshape({-1}), Slice()})
                             .reshape({-1, 8, 3});  // Hx8x3

      auto hex_corners_ex =
          hex_corners.transpose(1, 2).unsqueeze(1);  // Hx1x3x8
      auto J = torch::matmul(hex_corners_ex,
                             aug_hex_jacobians.unsqueeze(0));  // Hx9x3x3
      // Normalization is done here.
      J = J / (torch::norm(J, 2, 2, true) + 1e-8f);
      auto detJ =
          torch::linalg_det(J.reshape({-1, 3, 3})).reshape({-1, 9});  // Hx9

      auto scaled_jacobian = std::get<0>(detJ.min(1));  // H

      // We want to maximize scaled jacobian, so negative here.
      auto loss = -scaled_jacobian;

      if (options_.custom_exp_scaling) {
        loss = torch::logsumexp(kCustomExpCoeff * loss, -1) / kCustomExpCoeff;
      } else {
        loss = loss.mean();
      }
      return loss;
    };
  } else {
    throw std::runtime_error("Unknown custom quality!");
  }
}

torch::Tensor HexComplexDeformer::GetMaskedX() {
  // For now fix surface vertices.
  auto mask = torch::ones_like(X_);
  mask.index_put_({surface_indices_, Slice()}, 0.0f);
  return mask * X_ + (1 - mask) * X_.detach();
}

std::vector<torch::Tensor> HexComplexDeformer::GetLiftedX() const {
  torch::Tensor surface_X;
  torch::Tensor proj_dist;
  auto proj_result = geomlib::GeneralizedTriangleProjection<3>::apply(
      lifted_surface_X_, *original_proj_info_);
  surface_X = original_proj_info_->GetPointsAtBarycentricCoordinates(
      proj_result[1], proj_result[3], proj_result[4]);

  proj_dist = (surface_X.detach() - lifted_surface_X_).square().sum(-1);
  auto X = X_.scatter(0, surface_indices_.unsqueeze(1).expand_as(surface_X),
                      surface_X);

  return {X, proj_dist};
}

std::vector<Vector3f> HexComplexDeformer::GetOptimizedPositions() const {
  auto X = options_.mode == Mode::LiftedProjection ? GetLiftedX()[0] : X_;
  return MatrixXfToArrayVector3f(TensorToMatrixXf(X.cpu().detach()));
}

void HexComplexDeformer::PrintLosses(const std::vector<torch::Tensor>& losses) {
  LOGI(
      "distortion_loss: {}  | proj_loss : {} | fairness_loss: {} | "
      "smoothness_loss: {} |"
      "custom_loss: {}",
      losses[0].item<float>(), losses[1].item<float>(), losses[2].item<float>(),
      losses[3].item<float>(), losses[4].item<float>());
}
}  // namespace hex
