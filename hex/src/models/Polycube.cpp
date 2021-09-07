#include "Polycube.h"

#include "logging.h"

namespace hex {
Cuboid& Polycube::GetCuboid(size_t index) { return cuboids_.at(index); }

const Cuboid& Polycube::GetCuboid(size_t index) const {
  return cuboids_.at(index);
}

void Polycube::AddCuboid(const Cuboid& cuboid) {
  cuboids_.push_back(cuboid);
}

void Polycube::DeleteCuboid(int i) { cuboids_.erase(cuboids_.begin() + i); }

size_t Polycube::GetCuboidCount() const { return cuboids_.size(); }

Polycube::Polycube(const Eigen::MatrixXf& matrix) {
  for (int i = 0; i < matrix.rows(); i++) {
    Eigen::VectorXf p = matrix.row(i);
    AddCuboid(Cuboid({p[3], p[4], p[5]}, {p[0], p[1], p[2]}));
  }
}

Eigen::MatrixXf Polycube::ToMatrix() const {
  Eigen::MatrixXf res{GetCuboidCount(), 6};
  for (int i = 0; i < res.rows(); i++) {
    auto& cuboid = cuboids_[i];
    // For legacy reasons, halflengths is placed before center.
    res.block<1, 3>(i, 0) = cuboid.halflengths;
    res.block<1, 3>(i, 3) = cuboid.center;
  }
  return res;
}

}  // namespace hex
