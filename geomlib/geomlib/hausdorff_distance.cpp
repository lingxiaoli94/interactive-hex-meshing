#include "hausdorff_distance.h"

#include "TriangularMeshSampler.h"
#include "generalized_projection.h"

namespace geomlib {
std::tuple<float, float> ComputeHausdorffDistance(
    torch::Tensor vertices0, torch::Tensor faces0, torch::Tensor vertices1,
    torch::Tensor faces1, int batch_size, int repeat_times) {
  assert(vertices0.is_cuda() && vertices1.is_cuda() && faces0.is_cuda() &&
         faces1.is_cuda());
  TriangularMeshSampler sampler0{vertices0, faces0};
  TriangularMeshSampler sampler1{vertices1, faces1};

  TriangularProjectionInfo info0{vertices0, faces0};
  TriangularProjectionInfo info1{vertices1, faces1};

  float result_max = 0;
  float result_sum = 0;
  for (int i = 0; i < repeat_times; i++) {
    auto points0 = sampler0.Sample(batch_size);
    auto points1 = sampler1.Sample(batch_size);
    auto dist0 = ComputeGeneralizedTriangleProjection<3>(points0, info1)[0];
    auto dist1 = ComputeGeneralizedTriangleProjection<3>(points1, info0)[0];
    result_max = std::max(result_max, std::sqrt(dist0.max().item<float>()));
    result_max = std::max(result_max, std::sqrt(dist1.max().item<float>()));
    result_sum += dist0.sqrt().sum().item<float>();
    result_sum += dist1.sqrt().sum().item<float>();
  }
  result_sum /= repeat_times * batch_size * 2;
  return {result_max, result_sum};
}
}  // namespace geomlib
