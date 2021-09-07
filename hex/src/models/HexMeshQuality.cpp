#include "HexMeshQuality.h"

#include <stdexcept>

#include "HexConvention.h"
#include "HexahedralMesh.h"
#include "TriangularMesh.h"

namespace hex {

namespace {
float EvaluateScaledJacobian(const Eigen::MatrixXf& positions) {
  // positions is 8x3.
  auto& corner_jacobians = HexConvention::GetCornerJacobians();
  float dets[9];
  for (int i = 0; i < 8; i++) {
    Eigen::MatrixXf J = positions.transpose() * corner_jacobians[i];
    J.colwise().normalize();
    dets[i] = J.determinant();
  }

  // Add in the Jacobian coming from principal axes.
  auto& H = HexConvention::GetPrincipleAxes();
  Eigen::MatrixXf J = positions.transpose() * H;
  J.colwise().normalize();
  dets[8] = J.determinant();

  return *std::min_element(dets, dets + 9);
}

float EvaluateJacobian(const Eigen::MatrixXf& positions) {
  // positions is 8x3.
  auto& corner_jacobians = HexConvention::GetCornerJacobians();
  float dets[9];
  for (int i = 0; i < 8; i++) {
    Eigen::MatrixXf J = positions.transpose() * corner_jacobians[i];
    dets[i] = J.determinant();
  }

  // Add in the Jacobian coming from principal axes.
  auto& H = HexConvention::GetPrincipleAxes();
  Eigen::MatrixXf J = positions.transpose() * H;
  dets[8] = J.determinant() / 64.0f;

  return *std::min_element(dets, dets + 9);
}

}  // namespace

HexMeshQuality::HexMeshQuality(const HexahedralMesh& mesh) : mesh_{mesh} {}

const Eigen::VectorXf& HexMeshQuality::GetQuality(HexQualityType type) const {
  switch (type) {
    case HexQualityType::ScaledJacobian:
      return GetScaledJacobians();
    case HexQualityType::Jacobian:
      return GetJacobians();
    default:
      throw std::runtime_error("Unknown hex quality type!");
  }
}

const Eigen::VectorXf& HexMeshQuality::GetScaledJacobians() const {
  if (!scaled_jacobians_) {
    auto& hexes = mesh_.GetHexes();
    auto vertices = mesh_.GetVertices();
    scaled_jacobians_ = std::make_unique<Eigen::VectorXf>(hexes.size());
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(hexes.size()); i++) {
      auto& hex = hexes[i];
      Eigen::MatrixXf positions = Eigen::MatrixXf::Zero(8, 3);
      for (int k = 0; k < 8; k++) {
        positions.row(k) = vertices[hex[k]];
      }
      (*scaled_jacobians_)(i) = EvaluateScaledJacobian(positions);
    }
  }
  return *scaled_jacobians_;
}

const Eigen::VectorXf& HexMeshQuality::GetJacobians() const {
  if (!jacobians_) {
    auto& hexes = mesh_.GetHexes();
    auto vertices = mesh_.GetVertices();
    jacobians_ = std::make_unique<Eigen::VectorXf>(hexes.size());
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(hexes.size()); i++) {
      auto& hex = hexes[i];
      Eigen::MatrixXf positions = Eigen::MatrixXf::Zero(8, 3);
      for (int k = 0; k < 8; k++) {
        positions.row(k) = vertices[hex[k]];
      }
      (*jacobians_)(i) = EvaluateJacobian(positions);
    }
  }
  return *jacobians_;
}
}  // namespace hex
