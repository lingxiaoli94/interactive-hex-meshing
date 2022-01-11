#include "TriangularMeshSampler.h"

#include <ATen/Functions.h>

using namespace torch::indexing;

namespace geomlib {
TriangularMeshSampler::TriangularMeshSampler(torch::Tensor vertices,
                                             torch::Tensor faces)
    : vertices_{vertices}, faces_{faces} {
  auto v0 = vertices.index({faces.index({Slice(), 0}), Slice()});
  auto v1 = vertices.index({faces.index({Slice(), 1}), Slice()});
  auto v2 = vertices.index({faces.index({Slice(), 2}), Slice()});
  areas_ = (v1 - v0).cross(v2 - v0).norm(2, -1) / 2;
}

torch::Tensor TriangularMeshSampler::Sample(int batch_size) {
  // Sample with replacement.
  auto chosen_faces = torch::multinomial(areas_, batch_size, true);
  auto v0 = vertices_.index({faces_.index({chosen_faces, 0}), Slice()});
  auto v1 = vertices_.index({faces_.index({chosen_faces, 1}), Slice()});
  auto v2 = vertices_.index({faces_.index({chosen_faces, 2}), Slice()});

  auto r1 = torch::rand({batch_size}, v0.options());
  auto r2 = torch::rand({batch_size}, v0.options());

  return (1 - torch::sqrt(r1 + 1e-10f)).unsqueeze(1) * v0 +
         (torch::sqrt(r1 + 1e-10f) * (1 - r2)).unsqueeze(1) * v1 +
         (r2 * torch::sqrt(r1 + 1e-10f)).unsqueeze(1) * v2;
}

torch::Tensor TriangularMeshSampler::GetVertices() { return vertices_; }

torch::Tensor TriangularMeshSampler::GetFaces() { return faces_; }
}  // namespace geomlib
