#include "common.h"

namespace hex {
glm::vec3 ToGlm(const Vector3f& vec) {
  return glm::vec3{vec.x(), vec.y(), vec.z()};
}

std::vector<glm::vec3> ToGlm(const std::vector<Vector3f>& vec) {
  std::vector<glm::vec3> result;
  for (auto& v : vec) {
    result.push_back(ToGlm(v));
  }
  return result;
}

Eigen::Vector3f ToEigen(const glm::vec3& vec) {
  return Eigen::Vector3f{vec.x, vec.y, vec.z};
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

Eigen::MatrixXi ArrayVector4iToMatrixXi(const std::vector<Vector4i>& array) {
  Eigen::MatrixXi result{array.size(), array[0].size()};
  for (int i = 0; i < static_cast<int>(array.size()); i++) {
    result.row(i) = array[i];
  }
  return result;
}

std::vector<Vector3f> MatrixXfToArrayVector3f(const Eigen::MatrixXf& matrix) {
  std::vector<Vector3f> result(matrix.rows());
  for (int i = 0; i < matrix.rows(); i++) {
    result[i] = matrix.row(i);
  }
  return result;
}

std::vector<Vector4i> MatrixXiToArrayVector4i(const Eigen::MatrixXi& matrix) {
  std::vector<Vector4i> result(matrix.rows());
  for (int i = 0; i < matrix.rows(); i++) {
    result[i] = matrix.row(i);
  }
  return result;
}

std::size_t Vector3iHasher::operator()(const Vector3i& c) const {
  using std::hash;
  return hash<int>()(c.x()) ^ (hash<int>()(c.y()) << 4) ^
         (hash<int>()(c.z()) << 8);
}

std::size_t Vector4iHasher::operator()(const Vector4i& c) const {
  using std::hash;
  return hash<int>()(c.x()) ^ (hash<int>()(c.y()) << 4) ^
         (hash<int>()(c.z()) << 8) ^ (hash<int>()(c.w()) << 12);
}

}  // namespace hex
