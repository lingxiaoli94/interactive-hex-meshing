#pragma once

#include <torch/torch.h>

#include "generalized_projection_info.h"

namespace geomlib {
template <int dim>
std::vector<torch::Tensor> ComputeGeneralizedTriangleProjection(
    torch::Tensor points, const TriangularProjectionInfo& info);

template <int dim>
std::vector<torch::Tensor> ComputeGeneralizedTetrahedronProjection(
    torch::Tensor points, torch::Tensor vertices, torch::Tensor tets);
}  // namespace geomlib
