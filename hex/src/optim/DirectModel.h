#pragma once

#include <torch/torch.h>

namespace hex {
struct DirectModelImpl : torch::nn::Module {
  DirectModelImpl(torch::Tensor init_weight) {
    weight = register_parameter("weight", init_weight);
  }
  // torch::Tensor forward(torch::Tensor input) { return weight; }

  torch::Tensor weight;
};
TORCH_MODULE(DirectModel);
}  // namespace hex