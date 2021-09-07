#include "generalized_projection_info.h"

namespace geomlib {
TriangularProjectionInfo::TriangularProjectionInfo(torch::Tensor vertices,
                                                   torch::Tensor faces) {
  using namespace torch::indexing;
  dim = vertices.size(1);
  num_faces = faces.size(0);
  if (faces.dtype() == torch::kInt32) {
    faces = faces.to(torch::kInt64);
  }
  v0 = vertices.index({faces.index({Slice(), 0})}).contiguous();  // |F|x3
  e1 =
      (vertices.index({faces.index({Slice(), 1})}) - v0).contiguous();  // |F|x3
  e2 =
      (vertices.index({faces.index({Slice(), 2})}) - v0).contiguous();  // |F|x3
  e1_dot_e2 = (e1 * e2).sum(-1).contiguous();                           // |F|
  e1_norm_sqr = (e1 * e1).sum(-1).contiguous();                         // |F|
  e2_norm_sqr = (e2 * e2).sum(-1).contiguous();                         // |F|
  grad_w1 =
      ((e1_dot_e2 / e2_norm_sqr).unsqueeze(1) * (-e2) + e1) /
      (e1_norm_sqr - e1_dot_e2.square() / e2_norm_sqr).unsqueeze(1);  // |F|x3
  grad_w1 = grad_w1.contiguous();
  grad_w2 =
      ((e1_dot_e2 / e1_norm_sqr).unsqueeze(1) * (-e1) + e2) /
      (e2_norm_sqr - e1_dot_e2.square() / e1_norm_sqr).unsqueeze(1);  // |F|x3
  grad_w2 = grad_w2.contiguous();
}

torch::Tensor TriangularProjectionInfo::GetPointsAtBarycentricCoordinates(
    torch::Tensor face_ids, torch::Tensor w1, torch::Tensor w2) {
  // Note: this function backpropagates.
  assert(w1.size(0) == face_ids.size(0) && w2.size(0) == face_ids.size(0));
  assert(face_ids.max().item<int>() < v0.size(0));

  using namespace torch::indexing;
  return v0.index({face_ids, Slice()}) +
         w1.unsqueeze(1) * e1.index({face_ids, Slice()}) +
         w2.unsqueeze(1) * e2.index({face_ids, Slice()});
}
}  // namespace geomlib
