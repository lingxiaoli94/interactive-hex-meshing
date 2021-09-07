#include "debug.h"

namespace hex {
bool IsTensorFinite(torch::Tensor x) { return x.isfinite().all().item<bool>(); }
}  // namespace hex
