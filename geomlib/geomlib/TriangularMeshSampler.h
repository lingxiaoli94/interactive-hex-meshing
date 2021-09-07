#pragma once
#include <torch/torch.h>

namespace geomlib {
class TriangularMeshSampler {
 public:
  TriangularMeshSampler(torch::Tensor vertices, torch::Tensor faces);
  torch::Tensor Sample(int batch_size);
  torch::Tensor GetVertices();
  torch::Tensor GetFaces();

 private:
  torch::Tensor vertices_;
  torch::Tensor faces_;
  torch::Tensor areas_;
};
}  // namespace geomlib
