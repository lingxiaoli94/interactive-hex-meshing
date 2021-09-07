#pragma once

#include <float.h>
#include <math.h>
#include <thrust/tuple.h>

#include <cstdio>

namespace geomlib {
const auto kEpsilon = 1e-12;

template <typename T>
struct MakeVec3;

template <>
struct MakeVec3<float> {
  typedef float3 type;
  static __device__ float3 Create(float x, float y, float z) {
    return make_float3(x, y, z);
  }
  static __device__ float3 FromRaw(const float* arr) {
    return make_float3(arr[0], arr[1], arr[2]);
  }
};

template <>
struct MakeVec3<double> {
  typedef double3 type;
  static __device__ double3 Create(double x, double y, double z) {
    return make_double3(x, y, z);
  }
  static __device__ double3 FromRaw(const double* arr) {
    return make_double3(arr[0], arr[1], arr[2]);
  }
};

template <typename scalar_t, typename idx_t>
__device__ void WarpReduceMin(volatile scalar_t* min_vals,
                              volatile idx_t* min_idxs, size_t tid) {
  // s = 32
  if (min_vals[tid] > min_vals[tid + 32]) {
    min_idxs[tid] = min_idxs[tid + 32];
    min_vals[tid] = min_vals[tid + 32];
  }
  // s = 16
  if (min_vals[tid] > min_vals[tid + 16]) {
    min_idxs[tid] = min_idxs[tid + 16];
    min_vals[tid] = min_vals[tid + 16];
  }
  // s = 8
  if (min_vals[tid] > min_vals[tid + 8]) {
    min_idxs[tid] = min_idxs[tid + 8];
    min_vals[tid] = min_vals[tid + 8];
  }
  // s = 4
  if (min_vals[tid] > min_vals[tid + 4]) {
    min_idxs[tid] = min_idxs[tid + 4];
    min_vals[tid] = min_vals[tid + 4];
  }
  // s = 2
  if (min_vals[tid] > min_vals[tid + 2]) {
    min_idxs[tid] = min_idxs[tid + 2];
    min_vals[tid] = min_vals[tid + 2];
  }
  // s = 1
  if (min_vals[tid] > min_vals[tid + 1]) {
    min_idxs[tid] = min_idxs[tid + 1];
    min_vals[tid] = min_vals[tid + 1];
  }
}

template <typename scalar_t, typename idx_t>
__device__ inline void ReduceMin(scalar_t* min_vals, idx_t* min_tids,
                                 idx_t tid) {
  // Reduction over shared memory, only carrying along min_tids.
  for (idx_t s = blockDim.x / 2; s > 32; s >>= 1) {
    if (tid < s) {
      if (min_vals[tid] > min_vals[tid + s]) {
        min_vals[tid] = min_vals[tid + s];
        min_tids[tid] = min_tids[tid + s];
      }
    }
    __syncthreads();
  }

  // Unroll the last 6 iterations of the loop since they will happen
  // to be in a single warp - no synchronization is needed.
  if (tid < 32) {
    WarpReduceMin(min_vals, min_tids, tid);
  }
  __syncthreads();
}

template <typename scalar_t>
__device__ void WarpReduceSum(volatile scalar_t* vals, size_t tid) {
  vals[tid] += vals[tid + 32];
  vals[tid] += vals[tid + 16];
  vals[tid] += vals[tid + 8];
  vals[tid] += vals[tid + 4];
  vals[tid] += vals[tid + 2];
  vals[tid] += vals[tid + 1];
}

template <typename scalar_t>
__device__ inline int sign(scalar_t x) {
  int t = x < 0 ? -1 : 0;
  return x > 0 ? 1 : t;
}

template <typename scalar_t>
__device__ inline scalar_t clamp01(scalar_t x) {
  return x < 0 ? 0 : (x > 1 ? 1 : x);
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type operator-(
    const typename MakeVec3<scalar_t>::type& a,
    const typename MakeVec3<scalar_t>::type& b) {
  return MakeVec3<scalar_t>::Create(a.x - b.x, a.y - b.y, a.z - b.z);
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type operator+(
    const typename MakeVec3<scalar_t>::type& a,
    const typename MakeVec3<scalar_t>::type& b) {
  return MakeVec3<scalar_t>::Create(a.x + b.x, a.y + b.y, a.z + b.z);
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type operator*(
    const typename MakeVec3<scalar_t>::type& a,
    const typename MakeVec3<scalar_t>::type& b) {
  return MakeVec3<scalar_t>::Create(a.x * b.x, a.y * b.y, a.z * b.z);
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type operator*(
    scalar_t a, const typename MakeVec3<scalar_t>::type& b) {
  return MakeVec3<scalar_t>::Create(a * b.x, a * b.y, a * b.z);
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type operator/(
    const typename MakeVec3<scalar_t>::type& a,
    const typename MakeVec3<scalar_t>::type& b) {
  return MakeVec3<scalar_t>::Create(a.x / b.x, a.y / b.y, a.z / b.z);
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type operator/(
    const typename MakeVec3<scalar_t>::type& a, scalar_t b) {
  return MakeVec3<scalar_t>::Create(a.x / b, a.y / b, a.z / b);
}

template <typename scalar_t>
__device__ inline scalar_t dot(const typename MakeVec3<scalar_t>::type& a,
                               const typename MakeVec3<scalar_t>::type& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename scalar_t>
__device__ inline scalar_t distance_sqr(
    const typename MakeVec3<scalar_t>::type& a,
    const typename MakeVec3<scalar_t>::type& b) {
  return dot(a - b, a - b);
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type cross(
    const typename MakeVec3<scalar_t>::type& a,
    const typename MakeVec3<scalar_t>::type& b) {
  return MakeVec3<scalar_t>::Create(
      a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

template <typename scalar_t>
__device__ inline scalar_t norm(const typename MakeVec3<scalar_t>::type& a) {
  return sqrt(dot(a, a));
}

template <typename scalar_t>
__device__ inline typename MakeVec3<scalar_t>::type normalize(
    const typename MakeVec3<scalar_t>::type& a) {
  return a / (norm(a) + static_cast<scalar_t>(kEpsilon));
}

template <typename scalar_t>
__device__ inline bool IsOnSameSideOfPlane(
    const typename MakeVec3<scalar_t>::type& v0,
    const typename MakeVec3<scalar_t>::type& v1,
    const typename MakeVec3<scalar_t>::type& v2,
    const typename MakeVec3<scalar_t>::type& p,
    const typename MakeVec3<scalar_t>::type& q) {
  auto n =
      cross<scalar_t>(operator-<scalar_t>(v1, v0), operator-<scalar_t>(v2, v0));
  return sign(dot<scalar_t>(n, operator-<scalar_t>(p, v0))) ==
         sign(dot<scalar_t>(n, operator-<scalar_t>(q, v0)));
}

template <typename scalar_t>
__device__ inline bool IsPointInTetrahedron(
    const typename MakeVec3<scalar_t>::type& p,
    const typename MakeVec3<scalar_t>::type& v0,
    const typename MakeVec3<scalar_t>::type& v1,
    const typename MakeVec3<scalar_t>::type& v2,
    const typename MakeVec3<scalar_t>::type& v3) {
  return IsOnSameSideOfPlane<scalar_t>(v0, v1, v2, p, v3) &&
         IsOnSameSideOfPlane<scalar_t>(v1, v2, v3, p, v0) &&
         IsOnSameSideOfPlane<scalar_t>(v2, v3, v0, p, v1) &&
         IsOnSameSideOfPlane<scalar_t>(v3, v0, v1, p, v2);
}
}  // namespace geomlib
