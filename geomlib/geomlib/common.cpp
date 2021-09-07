#include "common.h"

namespace geomlib {
std::vector<std::string> Split(const std::string& s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> result;
  while (std::getline(ss, item, delim)) {
    result.emplace_back(std::move(item));
  }
  return result;
}

std::size_t Vector2iHasher::operator()(const Vector2i& c) const {
  return hash_combine(c.x(), c.y());
}

std::size_t Vector3iHasher::operator()(const Vector3i& c) const {
  return hash_combine(0, hash_combine(c.x(), hash_combine(c.y(), c.z())));
}

Eigen::MatrixXf ArrayVector3fToMatrixXf(const std::vector<Vector3f>& array) {
  Eigen::MatrixXf result{array.size(), array[0].size()};
  for (int i = 0; i < static_cast<int>(array.size()); i++) {
    result.row(i) = array[i];
  }
  return result;
}

Eigen::MatrixXi ArrayVector2iToMatrixXi(const std::vector<Vector2i>& array) {
  Eigen::MatrixXi result{array.size(), array[0].size()};
  for (int i = 0; i < static_cast<int>(array.size()); i++) {
    result.row(i) = array[i];
  }
  return result;
}

Eigen::MatrixXi ArrayVector3iToMatrixXi(const std::vector<Vector3i>& array) {
  Eigen::MatrixXi result{array.size(), array[0].size()};
  for (int i = 0; i < static_cast<int>(array.size()); i++) {
    result.row(i) = array[i];
  }
  return result;
}
}  // namespace geomlib