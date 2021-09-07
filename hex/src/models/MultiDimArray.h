#pragma once

#include "common.h"

namespace hex {
template <typename T>
class Array2D {
 public:
  Array2D(int n0, int n1) {
    dims_[0] = n0;
    dims_[1] = n1;
    data_.resize(n0 * n1);
  }

  T& operator()(int i, int j) { return data_[GetIndex(i, j)]; }
  T operator()(int i, int j) const { return data_[GetIndex(i, j)]; }
  int GetDim(int d) const {
    assert(0 <= d && d <= 1);
    return dims_[d];
  }
  T* GetDataPtr() { return data_.data(); }

 private:
  int GetIndex(int i, int j) { return i * dims_[1] + j; }

  int dims_[2];
  std::vector<T> data_;
};

template <typename T>
class Array3D {
 public:
  Array3D(int n0, int n1, int n2) {
    dims_[0] = n0;
    dims_[1] = n1;
    dims_[2] = n2;
    data_.resize(n0 * n1 * n2);
  }

  T& operator()(int i, int j, int k) { return data_[GetIndex(i, j, k)]; }
  T operator()(int i, int j, int k) const { return data_[GetIndex(i, j, k)]; }
  int GetDim(int d) const {
    assert(0 <= d && d <= 2);
    return dims_[d];
  }
  T* GetDataPtr() { return data_.data(); }

 private:
  int GetIndex(int i, int j, int k) {
    return i * (dims_[1] * dims_[2]) + j * dims_[2] + k;
  }

  int dims_[3];
  std::vector<T> data_;
};
}  // namespace hex
