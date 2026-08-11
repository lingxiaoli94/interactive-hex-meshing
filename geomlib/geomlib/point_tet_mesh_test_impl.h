#pragma once

// Per-element winding test shared by the CUDA kernel (point_tet_mesh_test_cuda.cu)
// and the CPU reference loop (point_tet_mesh_test.cpp), so both count exactly the
// same containments.
//
// The `__host__ __device__` decorations expand to nothing outside nvcc; see
// cuda_compat.h.

#include "utils.cuh"

namespace geomlib {
namespace detail {
// True if point `p` (3 scalars) lies inside tet `j` of the (vertices, tets) mesh.
template <typename scalar_t>
__host__ __device__ inline bool IsPointInTetAt(const scalar_t* p,
                                               const scalar_t* vertices,
                                               const int* tets, size_t j) {
  auto v0 = &vertices[3 * tets[4 * j]];
  auto v1 = &vertices[3 * tets[4 * j + 1]];
  auto v2 = &vertices[3 * tets[4 * j + 2]];
  auto v3 = &vertices[3 * tets[4 * j + 3]];
  return IsPointInTetrahedron<scalar_t>(
      MakeVec3<scalar_t>::FromRaw(p), MakeVec3<scalar_t>::FromRaw(v0),
      MakeVec3<scalar_t>::FromRaw(v1), MakeVec3<scalar_t>::FromRaw(v2),
      MakeVec3<scalar_t>::FromRaw(v3));
}
}  // namespace detail
}  // namespace geomlib
