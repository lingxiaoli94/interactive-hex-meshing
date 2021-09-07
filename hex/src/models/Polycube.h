#pragma once

#include "models/Cuboid.h"

namespace hex {
class Polycube {
 public:
  Polycube() = default;
  Polycube(const Eigen::MatrixXf& params);
  size_t GetCuboidCount() const;
  Cuboid& GetCuboid(size_t index);
  const Cuboid& GetCuboid(size_t index) const;
  void AddCuboid(const Cuboid& cuboid);
  void DeleteCuboid(int i);

  Eigen::MatrixXf ToMatrix() const;

 private:
  std::vector<Cuboid> cuboids_;
};
}  // namespace hex
