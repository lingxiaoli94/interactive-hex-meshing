#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace geomlib {
std::vector<std::string> Split(const std::string& s, char delim);

// TODO: merge below with common.h in hex
template <class T>
inline std::size_t hash_combine(std::size_t seed, const T& v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

using Vector3f = Eigen::Vector3f;
using Vector2i = Eigen::Vector2i;
using Vector3i = Eigen::Vector3i;
using Vector4i = Eigen::Matrix<int, 4, 1, Eigen::DontAlign>;

struct Vector2iHasher {
  std::size_t operator()(const Vector2i& c) const;
};

struct Vector3iHasher {
  std::size_t operator()(const Vector3i& c) const;
};

Eigen::MatrixXf ArrayVector3fToMatrixXf(const std::vector<Vector3f>& array);
Eigen::MatrixXi ArrayVector2iToMatrixXi(const std::vector<Vector2i>& array);
Eigen::MatrixXi ArrayVector3iToMatrixXi(const std::vector<Vector3i>& array); 
}  // namespace geomlib
