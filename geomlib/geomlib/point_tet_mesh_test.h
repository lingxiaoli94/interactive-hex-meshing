#pragma once

#include <torch/torch.h>

namespace geomlib {
// Returned tensor contains 1 or -1 (type same as tets).
torch::Tensor PointTetMeshTest(torch::Tensor points, torch::Tensor vertices,
                               torch::Tensor tets);
}
