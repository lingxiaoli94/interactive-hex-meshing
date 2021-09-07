#include "DistortionEnergy.h"

#include "debug.h"
#include "logging.h"

using namespace torch::indexing;

namespace hex {

namespace {
std::pair<torch::Tensor, torch::Tensor> ExtractAB(torch::Tensor J) {
  std::vector<torch::Tensor> a_list, b_list;
  for (int k = 0; k < 3; k++) {
    a_list.push_back(J.index({Slice(), Slice(), k}));  // Bx3
  }

  for (int k = 0; k < 3; k++) {
    b_list.push_back(
        torch::cross(a_list[(k + 1) % 3], a_list[(k + 2) % 3], 1));  // Bx3
  }
  // Stack in such a way that first returned = J.
  return {torch::stack(a_list, 2), torch::stack(b_list, 2)};
}

torch::Tensor Regularize(torch::Tensor D, float eps) {
  return (D + torch::sqrt(eps * eps + D.square())) / 2;
}

torch::Tensor RegularizeDerivaive(torch::Tensor D, float eps) {
  return (1 + D / torch::sqrt(eps * eps + D.square())) / 2;
}
}  // namespace

torch::Tensor ComputeDistortionLoss(torch::Tensor J, torch::Tensor weight,
                                    const DistortionOptions& options) {
  // J - Nx3x3, weight - N
  auto F = ConformalVolumetricEnergy::apply(J, options.regularizer_eps);  // N
  auto G = AuthalicVolumetricEnergy::apply(J, options.regularizer_eps);   // N

  auto loss = options.conformal_weight * (F - 3) +
              options.authalic_weight * (G - 2);  // N
  if (options.amips) {
    auto old_loss = loss;
    // Use log-sum-exp trick for stability.
    loss = torch::logsumexp(loss + torch::log(weight), -1);
  } else {
    loss = (loss * weight).sum();
  }

  return loss;
}

torch::Tensor ConformalVolumetricEnergy::forward(
    torch::autograd::AutogradContext* ctx, torch::Tensor J, float eps) {
  auto extract_result = ExtractAB(J);
  auto A = extract_result.first;
  auto B = extract_result.second;

  auto trace = A.square().sum(-1).sum(-1);     // B
  assert(IsTensorFinite(trace));
  auto det = (A.index({Slice(), Slice(), 0}) * B.index({Slice(), Slice(), 0}))
                 .sum(-1);  // B
  assert(IsTensorFinite(det));

  auto reg_det = Regularize(det, eps);
  assert(IsTensorFinite(reg_det));
  auto d_reg_det = RegularizeDerivaive(det, eps);  // B
  assert(IsTensorFinite(d_reg_det));
  auto result = trace / (1e-8f + torch::pow(reg_det, 2.0f / 3.0f));

  ctx->save_for_backward({A, B, trace, det, reg_det, d_reg_det, result});
  return result;
}

std::vector<torch::Tensor> ConformalVolumetricEnergy::backward(
    torch::autograd::AutogradContext* ctx,
    std::vector<torch::Tensor> grad_outputs) {
  auto saved = ctx->get_saved_variables();
  auto A = saved[0];          // Bx3x3
  auto B = saved[1];          // Bx3x3
  auto trace = saved[2];      // B
  auto det = saved[3];        // B
  auto reg_det = saved[4];    // B
  auto d_reg_det = saved[5];  // B
  auto f = saved[6];          // B

  auto dJ = 2 * A /
            (1e-8f + torch::pow(reg_det, 2.0f / 3.0f))
                .unsqueeze(1)
                .unsqueeze(2);  // Bx3x3
  dJ = dJ - (2.0f / 3.0f * f * d_reg_det / (1e-8f + reg_det))
                    .unsqueeze(1)
                    .unsqueeze(2) *
                B;

  return {grad_outputs.at(0).unsqueeze(1).unsqueeze(2) * dJ,
          torch::Tensor()};  // Bx3x3
}

torch::Tensor AuthalicVolumetricEnergy::forward(
    torch::autograd::AutogradContext* ctx, torch::Tensor J, float eps) {
  auto extract_result = ExtractAB(J);
  auto A = extract_result.first;
  auto B = extract_result.second;

  auto det = (A.index({Slice(), Slice(), 0}) * B.index({Slice(), Slice(), 0}))
                 .sum(-1);  // B

  auto reg_det = Regularize(det, eps);
  auto d_reg_det = RegularizeDerivaive(det, eps);  // B
  auto result = (det.square() + 1) / (1e-8f + reg_det);

  ctx->save_for_backward({A, B, det, reg_det, d_reg_det, result});
  return result;
}

std::vector<torch::Tensor> AuthalicVolumetricEnergy::backward(
    torch::autograd::AutogradContext* ctx,
    std::vector<torch::Tensor> grad_outputs) {
  auto saved = ctx->get_saved_variables();
  auto A = saved[0];          // Bx3x3
  auto B = saved[1];          // Bx3x3
  auto det = saved[2];        // B
  auto reg_det = saved[3];    // B
  auto d_reg_det = saved[4];  // B
  auto g = saved[5];          // B

  auto dJ = ((2 * det - g * d_reg_det) / (1e-8f + reg_det))
                .unsqueeze(1)
                .unsqueeze(2) *
            B;

  return {grad_outputs.at(0).unsqueeze(1).unsqueeze(2) * dJ,
          torch::Tensor()};  // Bx3x3
}
}  // namespace hex
