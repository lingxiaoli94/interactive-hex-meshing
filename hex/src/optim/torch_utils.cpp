#include "torch_utils.h"

#include <c10/core/ScalarType.h>

using namespace torch::indexing;
namespace hex {
torch::Tensor VectorXfToTensor(const Eigen::VectorXf& vec) {
  auto res = torch::zeros({vec.rows()}, torch::dtype(torch::kFloat32));
  Eigen::Map<Eigen::VectorXf> mp{res.data_ptr<float>(), vec.rows()};
  mp = vec;
  return res;
}

torch::Tensor VectorXiToTensor(const Eigen::VectorXi& vec) {
  auto res = torch::zeros({vec.rows()}, torch::dtype(torch::kInt32));
  Eigen::Map<Eigen::VectorXi> mp{res.data_ptr<int>(), vec.rows()};
  mp = vec;
  return res;
}

torch::Tensor MatrixXfToTensor(const Eigen::MatrixXf& mat) {
  auto res =
      torch::zeros({mat.cols(), mat.rows()}, torch::dtype(torch::kFloat32));
  Eigen::Map<Eigen::MatrixXf> mp{res.data_ptr<float>(), mat.rows(), mat.cols()};
  mp = mat;
  return res.transpose(0, 1);
}

Eigen::MatrixXf TensorToMatrixXf(const torch::Tensor& tensor) {
  assert(tensor.dtype() == torch::kFloat32);
  assert(tensor.dim() == 2);
  assert(!tensor.is_cuda());

  auto tensor_cont = tensor.contiguous();

  // Adjust for column-major Eigen conventions.
  return Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>{
      tensor_cont.data_ptr<float>(), tensor.size(0), tensor.size(1)};
}

Eigen::VectorXf TensorToVectorXf(const torch::Tensor& tensor) {
  assert(tensor.dtype() == torch::kFloat32);
  assert(tensor.dim() == 1);
  assert(!tensor.is_cuda());

  auto tensor_cont = tensor.contiguous();

  return Eigen::Map<Eigen::VectorXf>{tensor_cont.data_ptr<float>(),
                                     tensor.size(0)};
}

Eigen::VectorXi TensorToVectorXi(const torch::Tensor& tensor) {
  assert(tensor.dtype() == torch::kInt32);
  assert(tensor.dim() == 1);
  assert(!tensor.is_cuda());

  auto tensor_cont = tensor.contiguous();

  return Eigen::Map<Eigen::VectorXi>{tensor_cont.data_ptr<int>(),
                                     tensor.size(0)};
}

torch::Tensor MatrixXiToTensor(const Eigen::MatrixXi& mat) {
  auto res =
      torch::zeros({mat.cols(), mat.rows()}, torch::dtype(torch::kInt32));
  Eigen::Map<Eigen::MatrixXi> mp{res.data_ptr<int>(), mat.rows(), mat.cols()};
  mp = mat;
  return res.transpose(0, 1);
}

std::tuple<torch::Tensor, torch::Tensor> ParseCuboidParams(
    torch::Tensor cuboid_params) {
  // Input: cuboid_params - Cx6
  // Output: (b, T), b - Cx3, T - Cx3
  return std::make_tuple(cuboid_params.index({Slice(), Slice(0, 3)}),
                         cuboid_params.index({Slice(), Slice(3, 6)}));
}

torch::Tensor ComputeCuboidsSDF(torch::Tensor anchors,
                                torch::Tensor cuboid_params) {
  // Inputs: anchors - Nx3, cuboid_params - Cx6
  // Output: sdf - NxC
  auto cuboid_params_bT = ParseCuboidParams(cuboid_params);
  auto b = std::get<0>(cuboid_params_bT);
  auto T = std::get<1>(cuboid_params_bT);  // Cx3
  auto p = anchors.unsqueeze(1);           // Nx1x3

  p = p - T;             // NxCx3
  auto d = p.abs() - b;  // NxCx3

  auto length = torch::sqrt(
      torch::sum(torch::maximum(d, torch::zeros_like(d)).square(), -1) +
      1e-6f);                           // NxC
  auto max_d = std::get<0>(d.max(-1));  // NxC

  auto sdf = length + torch::minimum(max_d, torch::zeros_like(max_d));  // NxC
  return sdf;
}

torch::Tensor ComputeUnionOfCuboidsSDF(torch::Tensor anchors,
                                       torch::Tensor cuboid_params) {
  // Inputs: anchors - Nx3, cuboid_params - Cx6
  // Output: sdf - N
  auto sdf = ComputeCuboidsSDF(anchors, cuboid_params);  // NxC
  sdf = std::get<0>(sdf.min(-1));                        // N
  return sdf;
}

torch::Tensor SolveOrthogonalProcrust(torch::Tensor M) {
  // Input: M - Bx3x3
  // Output: argmax_R Tr(RM) - Bx3x3

  // TODO: cuSolver SVD instead.
  auto svd_result = torch::svd(M);
  // auto svd_result = geomlib::BatchSVDTorch(M);
  auto U = std::get<0>(svd_result);  // Bx3x3
  auto V = std::get<2>(svd_result);  // Bx3x3
  auto det = torch::det(torch::matmul(U, V.transpose(1, 2)));
  auto V_alt = torch::cat({V.index({Slice(), Slice(), Slice(None, 2)}),
                           -1 * V.index({Slice(), Slice(), 2}).unsqueeze(2)},
                          2);
  V = torch::where((det < 0).unsqueeze(1).unsqueeze(2).repeat({1, 3, 3}), V_alt,
                   V);                           // Bx3x3
  auto R = torch::matmul(U, V.transpose(1, 2));  // Bx3x3
  return R;
}

torch::Tensor SolveShrinkage(torch::Tensor b, torch::Tensor k) {
  // Inputs: b - BxD, k - B
  return torch::maximum(torch::zeros_like(b), b - k.unsqueeze(1) / 2) +
         torch::minimum(torch::zeros_like(b), b + k.unsqueeze(1) / 2);
}

Array3D<int> TensorToArray3DInt(torch::Tensor tensor) {
  assert(!tensor.is_cuda());
  assert(tensor.dim() == 3);
  assert(tensor.dtype() == torch::kInt32);

  Array3D<int> result(tensor.size(0), tensor.size(1), tensor.size(2));
  auto tensor_cont = tensor.contiguous();
  std::memcpy(result.GetDataPtr(), tensor_cont.data_ptr<int>(),
              tensor.size(0) * tensor.size(1) * tensor.size(2) * sizeof(int));
  return result;
}
}  // namespace hex
