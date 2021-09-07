#pragma once

// clang-format off
// On MSVC, the order of the include matters to avoid C2131.
// The reason is not clear (something to do with glm and torch).
#include <vkoo/common.h>
#include <geomlib/common.h>
// clang-format on

#include <Eigen/Dense>
#include <atomic>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <list>
#include <unordered_map>
#include <unordered_set>

namespace hex {
using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;
using Vector2i = Eigen::Vector2i;
using Vector3i = Eigen::Vector3i;
using Vector4i = Eigen::Matrix<int, 4, 1, Eigen::DontAlign>;

glm::vec3 ToGlm(const Vector3f& vec);
std::vector<glm::vec3> ToGlm(const std::vector<Vector3f>& vec);
Eigen::Vector3f ToEigen(const glm::vec3& vec);
Eigen::MatrixXf ArrayVector3fToMatrixXf(const std::vector<Vector3f>& array);
Eigen::MatrixXi ArrayVector2iToMatrixXi(const std::vector<Vector2i>& array);
Eigen::MatrixXi ArrayVector3iToMatrixXi(const std::vector<Vector3i>& array);
Eigen::MatrixXi ArrayVector4iToMatrixXi(const std::vector<Vector4i>& array);
std::vector<Vector3f> MatrixXfToArrayVector3f(const Eigen::MatrixXf& matrix);
std::vector<Vector4i> MatrixXiToArrayVector4i(const Eigen::MatrixXi& matrix);

struct Vector3iHasher {
  std::size_t operator()(const Vector3i& c) const;
};

struct Vector4iHasher {
  std::size_t operator()(const Vector4i& c) const;
};

}  // namespace hex
