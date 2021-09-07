#pragma once

#include "common.h"

#include <torch/torch.h>

#include "models/MultiDimArray.h"

namespace hex {
torch::Tensor VectorXfToTensor(const Eigen::VectorXf& vec);
torch::Tensor VectorXiToTensor(const Eigen::VectorXi& vec);

torch::Tensor MatrixXfToTensor(const Eigen::MatrixXf& mat);
torch::Tensor MatrixXiToTensor(const Eigen::MatrixXi& mat);
Eigen::MatrixXf TensorToMatrixXf(const torch::Tensor& tensor);
Eigen::VectorXf TensorToVectorXf(const torch::Tensor& tensor);
Eigen::VectorXi TensorToVectorXi(const torch::Tensor& tensor);

Array3D<int> TensorToArray3DInt(torch::Tensor tensor);

std::tuple<torch::Tensor, torch::Tensor> ParseCuboidParams(
    torch::Tensor cuboid_params);

torch::Tensor ComputeCuboidsSDF(torch::Tensor anchors,
                                torch::Tensor cuboid_params);

torch::Tensor ComputeUnionOfCuboidsSDF(torch::Tensor anchors,
                                       torch::Tensor cuboid_params);

torch::Tensor SolveOrthogonalProcrust(torch::Tensor M);
torch::Tensor SolveShrinkage(torch::Tensor b, torch::Tensor k);
}  // namespace hex
