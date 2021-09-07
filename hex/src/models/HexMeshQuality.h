#pragma once

#include "common.h"

namespace hex {
enum class HexQualityType { ScaledJacobian, Jacobian };

class TriangularMesh;
class HexahedralMesh;
class HexMeshQuality {
 public:
  HexMeshQuality(const HexahedralMesh& mesh);

  const Eigen::VectorXf& GetQuality(HexQualityType type) const;
  const Eigen::VectorXf& GetScaledJacobians() const;
  const Eigen::VectorXf& GetJacobians() const;

 private:
  const HexahedralMesh& mesh_;

  mutable std::unique_ptr<Eigen::VectorXf> scaled_jacobians_;
  mutable std::unique_ptr<Eigen::VectorXf> jacobians_;
};
}  // namespace hex
