#include "common.h"

#include <torch/torch.h>

namespace hex {
bool IsTensorFinite(torch::Tensor x);
}
