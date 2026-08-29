#pragma once

// CUDA entry points behind the public dispatchers in generalized_projection.cpp.
// The whole file is compiled out of a CPU-only build, so nothing outside a CUDA
// build can even name these.
#if HEX_ENABLE_CUDA

#include <torch/torch.h>

#include "generalized_projection_info.h"

namespace geomlib {
// Result tensors are allocated by the caller so that the CPU and CUDA paths
// cannot drift on shape, dtype or device.
template <int dim>
void ComputeGeneralizedTriangleProjectionCuda(
    torch::Tensor points, const TriangularProjectionInfo& info,
    torch::Tensor result_dists, torch::Tensor result_idxs,
    torch::Tensor result_w1, torch::Tensor result_w2);

// Takes the pre-computed per-tet quantities: they come from device-agnostic
// torch ops that the dispatcher runs for both devices.
template <int dim>
void ComputeGeneralizedTetrahedronProjectionCuda(
    torch::Tensor points, size_t num_tets, torch::Tensor v0,
    torch::Tensor e_mat, torch::Tensor e_dot_mat, torch::Tensor e_dot_inv_mat,
    torch::Tensor is_degenerate, torch::Tensor result_dists,
    torch::Tensor result_idxs, torch::Tensor result_weights);
}  // namespace geomlib

#endif  // HEX_ENABLE_CUDA
