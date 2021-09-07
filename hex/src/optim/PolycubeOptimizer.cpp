#include "PolycubeOptimizer.h"

#include <ATen/Functions.h>
#include <c10/cuda/CUDACachingAllocator.h>

#include "LargestCuboidSolver.h"
#include "logging.h"
#include "torch_utils.h"

using namespace torch::indexing;

namespace hex {
namespace {
const float kSideLengthLower = 0.0f;
const float kSideLengthUpper = 1.0f;  // 0.6f;
const float kSimpleCubeHalflength = 0.1f;
const int kSuggestDiscreteNum = 32;
const float kSubtractPadding = 0.2f;
const float kSubtractExtra = 0.005f;
}  // namespace

PolycubeOptimizer::PolycubeOptimizer(Options options)
    : options_{options}, device_{torch::kCUDA}, status_{Status::Idle} {}

void PolycubeOptimizer::Optimize(const TetrahedralMesh& mesh,
                                 const Polycube& polycube, size_t num_steps,
                                 const std::vector<int>& locked) {
  LOGI("Optimize cuboid decomposition for {} steps...", num_steps);
  stopped_ = false;
  Prepare(mesh, polycube, locked);
  LOGI("Loss before optimization: {}", ComputeCurrentLoss());
  status_ = Status::Running;
  OptimizeSteps(num_steps);
  status_ = Status::Completed;
  LOGI("Loss after optimization: {}", ComputeCurrentLoss());
  c10::cuda::CUDACachingAllocator::emptyCache();
}

void PolycubeOptimizer::Prepare(const TetrahedralMesh& mesh,
                                const Polycube& polycube,
                                const std::vector<int>& locked) {
  PrepareGPUMeshData(mesh);
  PrepareBBox(mesh);

  // Initialize model.
  std::vector<int> fixed_indices;
  std::vector<int> var_indices;
  for (int i = 0; i < static_cast<int>(locked.size()); i++) {
    if (locked[i]) {
      fixed_indices.push_back(i);
    } else {
      var_indices.push_back(i);
    }
  }
  var_indices_ = torch::from_blob(var_indices.data(), {(int)var_indices.size()},
                                  torch::kInt32)
                     .to(torch::kInt64)
                     .to(device_);
  fixed_indices_ = torch::from_blob(fixed_indices.data(),
                                    {(int)fixed_indices.size()}, torch::kInt32)
                       .to(torch::kInt64)
                       .to(device_);
  auto cuboid_params = TransformToCuboidParams(polycube).to(device_);

  weights_ = TransformToWeight(cuboid_params.index({var_indices_, Slice()}));
  weights_.requires_grad_(true);
  fixed_params_ = cuboid_params.index({fixed_indices_, Slice()});

  // Initialize optimizer.
  optimizer_ = std::make_unique<torch::optim::Adam>(
      std::vector<torch::Tensor>{weights_},
      torch::optim::AdamOptions(options_.learning_rate)
          .betas({options_.adam_betas(0), options_.adam_betas(1)}));

  global_step_ = 0;
  snapshot_ = polycube;
}

torch::Tensor PolycubeOptimizer::CombineCuboidParams(bool preserve_order) {
  auto var_params = TransformToCuboidParams(weights_);
  if (preserve_order) {
    auto result = torch::zeros(
        {var_params.size(0) + fixed_params_.size(0), var_params.size(1)},
        fixed_params_.options());
    result.scatter_(0,
                    var_indices_.unsqueeze(1).repeat({1, var_params.size(1)}),
                    var_params);
    result.scatter_(
        0, fixed_indices_.unsqueeze(1).repeat({1, fixed_params_.size(1)}),
        fixed_params_);
    return result;
  } else {
    return torch::cat({var_params, fixed_params_}, 0);
  }
}

float PolycubeOptimizer::ComputeCurrentLoss() {
  auto cuboid_params = CombineCuboidParams(false);
  auto polycube_sdf = ComputeUnionOfCuboidsSDF(anchors_,
                                               cuboid_params);  // N
  auto loss = ComputeLoss(cuboid_params, polycube_sdf);
  return loss.item<float>();
}

void PolycubeOptimizer::PrepareGPUMeshData(const TetrahedralMesh& target_mesh) {
  mesh_sdf_ = VectorXfToTensor(target_mesh.GetDistanceField()).to(device_);
  anchors_ = MatrixXfToTensor(target_mesh.GetAnchors()).to(device_);
}

void PolycubeOptimizer::PrepareBBox(const TetrahedralMesh& mesh) {
  auto vertices = MatrixXfToTensor(mesh.GetVertices()).to(device_);
  bbox_[0] = std::get<0>(vertices.min(0));
  bbox_[1] = std::get<0>(vertices.max(0));
  max_b_ = (bbox_[1] - bbox_[0]) / 2;
}

void PolycubeOptimizer::OptimizeSteps(size_t num_steps) {
  size_t target_step = global_step_ + num_steps;
  while (!stopped_ && global_step_ < target_step) {
    global_step_++;

    auto closure = [&]() {
      auto cuboid_params = CombineCuboidParams(false);
      auto polycube_sdf = ComputeUnionOfCuboidsSDF(anchors_,
                                                   cuboid_params);  // N
      auto loss = ComputeLoss(cuboid_params, polycube_sdf);

      optimizer_->zero_grad();
      loss.backward();
      return loss;
    };
    optimizer_->step(closure);

    if (global_step_ % options_.snapshot_freq == 0) {
      std::lock_guard<std::mutex> lock_guard(snapshot_mutex_);
      snapshot_ = GetOptimizedPolycube();
    }
  }
}

torch::Tensor PolycubeOptimizer::ComputeLoss(
    const torch::Tensor& cuboid_params, const torch::Tensor& polycube_sdf) {
  auto positive_l2_loss =
      ((torch::where(mesh_sdf_ > 0, polycube_sdf,
                     torch::zeros_like(polycube_sdf)) -
        torch::maximum(mesh_sdf_, torch::zeros_like(mesh_sdf_)))
           .square())
          .mean(-1);

  auto negative_l2_loss =
      (torch::where(torch::logical_and(mesh_sdf_<0, polycube_sdf> 0),
                    polycube_sdf, torch::zeros_like(polycube_sdf)))
          .square()
          .mean(-1);
  auto l2_loss = options_.positive_l2_weight * positive_l2_loss +
                 options_.negative_l2_weight * negative_l2_loss;

  return l2_loss;
}

Polycube PolycubeOptimizer::GetOptimizedPolycube() {
  return TransformToPolycube(CombineCuboidParams(true));
}

torch::Tensor PolycubeOptimizer::TransformToCuboidParams(
    const torch::Tensor& weight) {
  // weight - Cx6
  auto pos_weight = torch::sigmoid(weight);  // Cx6
  auto bT = ParseCuboidParams(pos_weight);
  auto b = std::get<0>(bT);
  auto t = std::get<1>(bT);

  auto max_b = max_b_.index({None, Slice()});
  b = (kSideLengthLower + b * (kSideLengthUpper - kSideLengthLower)) * max_b;
  t = t * max_b * 2 + bbox_[0];

  return torch::cat({b, t}, -1);  // Cx6
}

torch::Tensor PolycubeOptimizer::TransformToWeight(
    const torch::Tensor& cuboid_params) {
  // cuboid_params - Cx6
  auto bT = ParseCuboidParams(cuboid_params);
  auto b = std::get<0>(bT);
  auto t = std::get<1>(bT);

  auto max_b = max_b_.index({None, Slice()});
  b = (b / max_b - kSideLengthLower) / (kSideLengthUpper - kSideLengthLower);
  t = (t - bbox_[0]) / (max_b * 2);

  auto weight = torch::cat({b, t}, -1);  // Cx6
  weight = torch::clamp(weight, 0.0f, 1.0f);

  // Inverse sigmoid.
  weight = torch::log(weight / (1 - weight));
  return weight;
}

torch::Tensor PolycubeOptimizer::TransformToCuboidParams(
    const Polycube& polycube) {
  return MatrixXfToTensor(polycube.ToMatrix());
}

Polycube PolycubeOptimizer::TransformToPolycube(
    const torch::Tensor& cuboid_params_gpu) {
  return Polycube(TensorToMatrixXf(cuboid_params_gpu.cpu()));
}

Polycube PolycubeOptimizer::GetSnapshot() {
  std::lock_guard<std::mutex> lock_guard(snapshot_mutex_);
  return snapshot_;
}
void PolycubeOptimizer::SetIdle() { status_ = Status::Idle; }

bool PolycubeOptimizer::IsIdle() { return status_ == Status::Idle; }

PolycubeOptimizer::Status PolycubeOptimizer::GetStatus() { return status_; }

void PolycubeOptimizer::Stop() { stopped_ = true; }

std::tuple<torch::Tensor, Vector3f> PolycubeOptimizer::CreateAmbientGrid(
    int discrete_num, float padding) {
  std::vector<torch::Tensor> steps;

  for (int i = 0; i < 3; i++) {
    float lb = bbox_[0][i].item<float>();
    float rb = bbox_[1][i].item<float>();
    float h = rb - lb;
    steps.push_back(
        torch::linspace(lb - padding * h, rb + padding * h, discrete_num));
  }
  auto delta = TensorToVectorXf(
      ((bbox_[1] - bbox_[0]) * (1.0f + 2 * padding) / (discrete_num - 1))
          .detach()
          .cpu());
  auto grids = torch::meshgrid(steps);

  return {torch::stack(grids, -1).to(torch::kFloat32).cuda(),
          delta};  // DxDxDx3
}

bool PolycubeOptimizer::SuggestNewCuboid(const TetrahedralMesh& mesh,
                                         const Polycube& polycube,
                                         SuggestStrategy strategy,
                                         Cuboid& new_cuboid) {
  if (polycube.GetCuboidCount() == 0) {
    strategy = SuggestStrategy::Largest;
  }

  PrepareGPUMeshData(mesh);
  PrepareBBox(mesh);

  auto cuboid_params = TransformToCuboidParams(polycube).to(device_);
  if (strategy == SuggestStrategy::Simple) {
    // Anchors are better to use than uniform grid, since it has points close to
    // the surface.
    auto polycube_sdf = ComputeUnionOfCuboidsSDF(anchors_,
                                                 cuboid_params);  // N

    // Strategy: find an uncovered point with mesh sdf < 0 but largest polycube
    // sdf.
    auto tmp_sdf = torch::where(mesh_sdf_ < 0, polycube_sdf,
                                torch::zeros_like(polycube_sdf));
    auto tmp_max = tmp_sdf.max(0);
    float max_value = std::get<0>(tmp_max).item<float>();
    int anchor_id = std::get<1>(tmp_max).item<int>();

    if (max_value < 1e-8f) {
      return false;
    }
    new_cuboid.center =
        TensorToVectorXf(anchors_.index({anchor_id, Slice()}).detach().cpu());
    new_cuboid.halflengths = kSimpleCubeHalflength * Eigen::Vector3f::Ones();
    return true;
  } else {
    assert(strategy == SuggestStrategy::Largest);
    int discrete_num = kSuggestDiscreteNum;
    auto ambient_grid = CreateAmbientGrid(discrete_num);
    auto grid_points = std::get<0>(ambient_grid);
    Vector3f delta = std::get<1>(ambient_grid);
    auto grid_points_flattened = grid_points.reshape({-1, 3});

    // Find the largest cuboid contained in the target mesh but disjoint from
    // the current polycube.
    auto mesh_sdf = mesh.ComputeDistanceFieldGPU(grid_points_flattened);  // N
    torch::Tensor polycube_sdf;
    if (polycube.GetCuboidCount() == 0) {
      // Just set polycube_sdf to > 0 zero if there's no polycube.
      polycube_sdf = torch::ones_like(mesh_sdf);
    } else {
      polycube_sdf = ComputeUnionOfCuboidsSDF(grid_points_flattened,
                                              cuboid_params);  // N
    }
    auto mask = torch::logical_and(polycube_sdf >= 0, mesh_sdf < 0);  // N
    mask = mask.reshape({discrete_num, discrete_num, discrete_num})
               .to(torch::kInt32);  // DxDxD

    // Next find the largest cuboid of 1's in mask.
    auto mask_array_3d = TensorToArray3DInt(mask.detach().cpu());

    Vector3i cl, cr;
    int max_volume;
    LargestCuboidSolver(mask_array_3d).Solve(cl, cr, max_volume);

    Vector3f bbox_low = TensorToVectorXf(bbox_[0].detach().cpu());
    if (max_volume > 0) {
      LOGI("Find largest cuboid ({},{},{})-({},{},{}) with integral volume {}",
           cl.x(), cl.y(), cl.z(), cr.x(), cr.y(), cr.z(), max_volume);
      // Vector3f delta = (bbox_high - bbox_low) / (discrete_num - 1);

      Vector3f pl = cl.cast<float>().cwiseProduct(delta);
      Vector3f pr = cr.cast<float>().cwiseProduct(delta);
      new_cuboid.SetBound(bbox_low + pl - delta / 2, bbox_low + pr + delta / 2);
      return true;

    } else {
      return false;
    }
  }
};

bool PolycubeOptimizer::SuggestSubtractCuboid(const TetrahedralMesh& mesh,
                                              const Polycube& polycube,
                                              Cuboid& subtract_cuboid) {
  if (polycube.GetCuboidCount() == 0) {
    return false;
  }

  PrepareGPUMeshData(mesh);
  PrepareBBox(mesh);

  auto cuboid_params = TransformToCuboidParams(polycube).to(device_);
  int discrete_num = kSuggestDiscreteNum;
  auto ambient_grid = CreateAmbientGrid(
      discrete_num,
      kSubtractPadding);  // create grid slightly bigger than the mesh
  auto grid_points = std::get<0>(ambient_grid);
  Vector3f delta = std::get<1>(ambient_grid);
  auto grid_points_flattened = grid_points.reshape({-1, 3});

  // Find the largest cuboid contained in the polycube but not in the mesh.
  auto mesh_sdf = mesh.ComputeDistanceFieldGPU(grid_points_flattened);  // N
  torch::Tensor polycube_sdf;
  polycube_sdf = ComputeUnionOfCuboidsSDF(grid_points_flattened,
                                          cuboid_params);       // N
  auto mask = torch::logical_and(polycube_sdf<0, mesh_sdf> 0);  // N
  mask = mask.reshape({discrete_num, discrete_num, discrete_num})
             .to(torch::kInt32);  // DxDxD

  // Next find the largest cuboid of 1's in mask
  auto mask_array_3d = TensorToArray3DInt(mask.detach().cpu());

  Vector3i cl, cr;
  int max_volume;
  LargestCuboidSolver(mask_array_3d).Solve(cl, cr, max_volume);

  Vector3f bbox_low = TensorToVectorXf(
      (bbox_[0] - (bbox_[1] - bbox_[0]) * kSubtractPadding).detach().cpu());
  if (max_volume > 0) {
    LOGI("Find largest cuboid ({},{},{})-({},{},{}) with integral volume {}",
         cl.x(), cl.y(), cl.z(), cr.x(), cr.y(), cr.z(), max_volume);
    Vector3f pl = cl.cast<float>().cwiseProduct(delta).array() - kSubtractExtra;
    Vector3f pr = cr.cast<float>().cwiseProduct(delta).array() + kSubtractExtra;
    subtract_cuboid.SetBound(bbox_low + pl - delta / 2,
                             bbox_low + pr + delta / 2);
    return true;

  } else {
    return false;
  }
}
}  // namespace hex
