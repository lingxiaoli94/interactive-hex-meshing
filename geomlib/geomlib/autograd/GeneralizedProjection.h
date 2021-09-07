#pragma once

#include "geomlib/common.h"
#include "geomlib/generalized_projection.h"

namespace geomlib {
template <int dim>
class GeneralizedTriangleProjection
    : public torch::autograd::Function<GeneralizedTriangleProjection<dim>> {
 public:
  static std::vector<torch::Tensor> forward(
      torch::autograd::AutogradContext* ctx, torch::Tensor points,
      const TriangularProjectionInfo& info) {
    auto proj_result =
        ComputeGeneralizedTriangleProjection<dim>(points.contiguous(), info);
    auto result_dists = proj_result[0];
    auto result_idxs = proj_result[1];  // always int64_t
    auto result_w1 = proj_result[2];
    auto result_w2 = proj_result[3];
    auto grad_w1 = info.grad_w1;
    auto grad_w2 = info.grad_w2;

    std::vector<torch::Tensor> result{result_dists, result_idxs,
                                      1 - result_w1 - result_w2, result_w1,
                                      result_w2};
    ctx->mark_non_differentiable({result_dists, result_idxs});
    ctx->save_for_backward({result_idxs, grad_w1, grad_w2});
    return result;
  }

  static std::vector<torch::Tensor> backward(
      torch::autograd::AutogradContext* ctx,
      std::vector<torch::Tensor> grad_outputs) {
    assert(grad_outputs.size() == 5);
    auto saved = ctx->get_saved_variables();
    auto result_idxs = saved[0];
    auto grad_w1 = saved[1];
    auto grad_w2 = saved[2];

    using namespace torch::indexing;
    auto grad_w1_result = grad_w1.index({result_idxs, Slice()});
    auto grad_w2_result = grad_w2.index({result_idxs, Slice()});
    auto grad_w0_result = -grad_w1_result - grad_w2_result;

    grad_w0_result = grad_outputs.at(2).unsqueeze(1) * grad_w0_result;
    grad_w1_result = grad_outputs.at(3).unsqueeze(1) * grad_w1_result;
    grad_w2_result = grad_outputs.at(4).unsqueeze(1) * grad_w2_result;

    auto grad_p = grad_w0_result + grad_w1_result + grad_w2_result;

    // Return as many tensors as there were in the inputs, and empty tensors for
    // non-input.
    return {grad_p, torch::Tensor()};
  }
};

template <int dim>
class GeneralizedTetrahedronProjection
    : public torch::autograd::Function<GeneralizedTetrahedronProjection<dim>> {
 public:
  static std::vector<torch::Tensor> forward(
      torch::autograd::AutogradContext* ctx, torch::Tensor points,
      torch::Tensor vertices, torch::Tensor tets) {
    auto proj_result = ComputeGeneralizedTetrahedronProjection<dim>(
        points.contiguous(), vertices.contiguous(),
        tets.contiguous().to(torch::kInt64));
    auto result_dists = proj_result[0];
    auto result_idxs = proj_result[1].to(tets.dtype());
    torch::Tensor result_weights = proj_result[2];  // Nx3
    auto base_weight = (1 - result_weights.sum(-1)).unsqueeze(1);
    result_weights = torch::cat({base_weight, result_weights}, 1);

    std::vector<torch::Tensor> result{result_dists, result_idxs,
                                      result_weights};
    ctx->mark_non_differentiable({result_dists, result_idxs, result_weights});
    return result;
  }

  // No gradient for tetrahedron projection.
  static std::vector<torch::Tensor> backward(
      [[maybe_unused]] torch::autograd::AutogradContext* ctx,
      [[maybe_unused]] std::vector<torch::Tensor> grad_outputs) {
    return {torch::Tensor(), torch::Tensor(), torch::Tensor()};
  }
};
}  // namespace geomlib
