#pragma once

#include "common.h"

#include <torch/torch.h>

namespace hex {
struct DistortionOptions {
  float conformal_weight{1.0f};
  float authalic_weight{1.0f};
  float regularizer_eps{1e-3f};
  bool amips{false};
};

torch::Tensor ComputeDistortionLoss(torch::Tensor J, torch::Tensor weight,
                                    const DistortionOptions& options);
class ConformalVolumetricEnergy
    : public torch::autograd::Function<ConformalVolumetricEnergy> {
 public:
  // J - Bx3x3
  static torch::Tensor forward(torch::autograd::AutogradContext* ctx,
                               torch::Tensor J, float eps);

  static std::vector<torch::Tensor> backward(
      torch::autograd::AutogradContext* ctx,
      std::vector<torch::Tensor> grad_outputs);
};

class AuthalicVolumetricEnergy
    : public torch::autograd::Function<AuthalicVolumetricEnergy> {
 public:
  // J - Bx3x3
  static torch::Tensor forward(torch::autograd::AutogradContext* ctx,
                               torch::Tensor J, float eps);

  static std::vector<torch::Tensor> backward(
      torch::autograd::AutogradContext* ctx,
      std::vector<torch::Tensor> grad_outputs);
};
}  // namespace hex
