#pragma once

#include <torch/torch.h>

namespace geomlib {
struct TriangularProjectionInfo {
  TriangularProjectionInfo(torch::Tensor vertices, torch::Tensor faces);
  TriangularProjectionInfo() = delete;

  torch::Tensor GetPointsAtBarycentricCoordinates(torch::Tensor face_ids,
                                                  torch::Tensor w1,
                                                  torch::Tensor w2);

  int dim;
  int num_faces;
  torch::Tensor v0;           // |F|xD
  torch::Tensor e1;           // |F|xD
  torch::Tensor e2;           // |F|xD
  torch::Tensor e1_dot_e2;    // |F|
  torch::Tensor e1_norm_sqr;  // |F|
  torch::Tensor e2_norm_sqr;  // |F|
  torch::Tensor grad_w1;      // |F|xD
  torch::Tensor grad_w2;      // |F|xD
};
}  // namespace geomlib
