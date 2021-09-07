#pragma once

#include <torch/torch.h>

namespace geomlib {
// Symmetric Hausdorff distance using Monte Carlo sampling.
std::tuple<float, float> ComputeHausdorffDistance(
    torch::Tensor vertices0, torch::Tensor faces0, torch::Tensor vertices1,
    torch::Tensor faces1, int batch_size, int repeat_times);
}
