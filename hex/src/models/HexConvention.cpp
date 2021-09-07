#include "HexConvention.h"
/*
 * Indices of hex corners denote the following:
 *
 *       2------3
 *      /|     /|
 *     6------7 |
 *     | |    | |
 *     | 0----|-1
 *     |/     |/
 *     4------5
 *
 *       y
 *       |
 *       |
 *       *----x
 *      /
 *     /
 *    z
 *
 */

namespace hex {
HexConvention& HexConvention::GetInstance() {
  static HexConvention instance;
  return instance;
}

void HexConvention::BuildCorners() {
  corners_ = std::make_unique<std::vector<Vector3i>>();
  corner_to_id_ =
      std::make_unique<std::unordered_map<Vector3i, size_t, Vector3iHasher>>();
  for (int dz = 0; dz <= 1; dz++)
    for (int dy = 0; dy <= 1; dy++)
      for (int dx = 0; dx <= 1; dx++) {
        auto corner = Vector3i{dx, dy, dz};
        corner_to_id_->emplace(corner, corners_->size());
        corners_->push_back(corner);
      }
}

const std::vector<Vector3i>& HexConvention::GetCorners() {
  auto& instance = GetInstance();
  if (!instance.corners_) {
    instance.BuildCorners();
  }
  return *instance.corners_;
}

const std::unordered_map<Vector3i, size_t, Vector3iHasher>&
HexConvention::GetCornerToIdMap() {
  auto& instance = GetInstance();
  if (!instance.corner_to_id_) {
    instance.BuildCorners();
  }
  return *instance.corner_to_id_;
}

const std::vector<Eigen::MatrixXf>& HexConvention::GetCornerJacobians() {
  auto& instance = GetInstance();
  if (!instance.corner_jacobians_) {
    auto& corners = instance.GetCorners();
    auto& corner_to_id = instance.GetCornerToIdMap();

    instance.corner_jacobians_ =
        std::make_unique<std::vector<Eigen::MatrixXf>>();
    for (int i = 0; i < 8; i++) {
      // Build a Jacobian matrix from the 3 vertices closest to corner i.
      auto c = corners[i];
      auto nx = Vector3i{(c.x() + 1) % 2, c.y(), c.z()};
      size_t ix = corner_to_id.at(nx);
      auto ny = Vector3i{c.x(), (c.y() + 1) % 2, c.z()};
      size_t iy = corner_to_id.at(ny);
      auto nz = Vector3i{c.x(), c.y(), (c.z() + 1) % 2};
      size_t iz = corner_to_id.at(nz);

      Eigen::MatrixXf D = Eigen::MatrixXf::Zero(8, 3);
      D(ix, 0) = 1;
      D(i, 0) = -1;
      D(iy, 1) = 1;
      D(i, 1) = -1;
      D(iz, 2) = 1;
      D(i, 2) = -1;

      Eigen::MatrixXd S = Eigen::MatrixXd::Zero(3, 3);
      S.col(0) = (corners[ix] - corners[i]).cast<double>();
      S.col(1) = (corners[iy] - corners[i]).cast<double>();
      S.col(2) = (corners[iz] - corners[i]).cast<double>();

      Eigen::MatrixXf J = D * S.inverse().cast<float>();

      instance.corner_jacobians_->push_back(J);
    }
  }
  return *instance.corner_jacobians_;
}

const Eigen::MatrixXf& HexConvention::GetPrincipleAxes() {
  auto& instance = GetInstance();
  if (!instance.principal_axes_) {
    auto& corners = instance.GetCorners();
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(8, 3);
    for (int i = 0; i < 8; i++) {
      auto c = corners[i];
      H(i, 0) = c.x() == 0 ? -1 : 1;
      H(i, 1) = c.y() == 0 ? -1 : 1;
      H(i, 2) = c.z() == 0 ? -1 : 1;
    }
    instance.principal_axes_ = std::make_unique<Eigen::MatrixXf>(H);
  }
  return *instance.principal_axes_;
}

const std::vector<Vector3i>& HexConvention::GetFaceOffsets() {
  auto& instance = GetInstance();
  if (!instance.face_offsets_) {
    instance.face_offsets_ = std::make_unique<std::vector<Vector3i>>(
        std::vector<Vector3i>{{-1, 0, 0},
                              {1, 0, 0},
                              {0, -1, 0},
                              {0, 1, 0},
                              {0, 0, -1},
                              {0, 0, 1}});
  }
  return *instance.face_offsets_;
}

const std::unordered_map<Vector3i, size_t, Vector3iHasher>&
HexConvention::GetFaceOffsetToId() {
  auto& instance = GetInstance();
  if (!instance.face_offset_to_id_) {
    instance.face_offset_to_id_ = std::make_unique<
        std::unordered_map<Vector3i, size_t, Vector3iHasher>>();
    auto& face_offsets = GetFaceOffsets();
    for (size_t i = 0; i < 6; i++) {
      (*instance.face_offset_to_id_)[face_offsets[i]] = i;
    }
  }
  return *instance.face_offset_to_id_;
}

const std::vector<std::vector<size_t>>& HexConvention::GetFaceCornerIds() {
  auto& instance = GetInstance();
  if (!instance.face_corner_ids_) {
    instance.face_corner_ids_ =
        std::make_unique<std::vector<std::vector<size_t>>>(
            std::vector<std::vector<size_t>>{{0, 4, 6, 2},
                                             {1, 3, 7, 5},
                                             {0, 1, 5, 4},
                                             {2, 6, 7, 3},
                                             {0, 2, 3, 1},
                                             {4, 5, 7, 6}});
  }
  return *instance.face_corner_ids_;
}

const std::vector<std::vector<size_t>>&
HexConvention::GetOppositeFaceCornerIds() {
  auto& instance = GetInstance();
  if (!instance.opposite_face_corner_ids_) {
    instance.opposite_face_corner_ids_ =
        std::make_unique<std::vector<std::vector<size_t>>>(
            std::vector<std::vector<size_t>>{{1, 5, 7, 3},
                                             {0, 2, 6, 4},
                                             {2, 3, 7, 6},
                                             {0, 4, 5, 1},
                                             {4, 6, 7, 5},
                                             {0, 1, 3, 2}});
  }
  return *instance.opposite_face_corner_ids_;
}

void HexConvention::Initialize() {
  // Initialize ahead to prevent race conditions.
  GetCorners();
  GetCornerJacobians();
  GetCornerToIdMap();
  GetPrincipleAxes();
  GetFaceOffsets();
  GetFaceOffsetToId();
  GetFaceCornerIds();
  GetOppositeFaceCornerIds();
}
}  // namespace hex
