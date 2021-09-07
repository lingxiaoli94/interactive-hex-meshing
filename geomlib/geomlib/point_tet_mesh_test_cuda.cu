#include "point_tet_mesh_test.h"

#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>

#include "common.cuh"
#include "utils.cuh"

namespace geomlib {
namespace {
template <typename scalar_t>
__global__ void PointTetMeshTestKernel(const scalar_t* __restrict__ points,
                                       const scalar_t* __restrict__ vertices,
                                       const int* __restrict__ tets,
                                       size_t num_tets,
                                       int* __restrict__ result_signs) {
  extern __shared__ char shared_buf[];
  int* windings = (int*)shared_buf;

  // Each block tests a single point.
  const size_t i = blockIdx.x;
  // Tets are divided evenly among the threads.
  const size_t tid = threadIdx.x;

  int winding = 0;
  for (size_t j = tid; j < num_tets; j += blockDim.x) {
    auto p = &points[i * 3];
    auto v0 = &vertices[3 * tets[4 * j]];
    auto v1 = &vertices[3 * tets[4 * j + 1]];
    auto v2 = &vertices[3 * tets[4 * j + 2]];
    auto v3 = &vertices[3 * tets[4 * j + 3]];
    if (IsPointInTetrahedron<scalar_t>(
            MakeVec3<scalar_t>::FromRaw(p), MakeVec3<scalar_t>::FromRaw(v0),
            MakeVec3<scalar_t>::FromRaw(v1), MakeVec3<scalar_t>::FromRaw(v2),
            MakeVec3<scalar_t>::FromRaw(v3))) {
      winding += 1;
    }
  }
  windings[tid] = winding;
  __syncthreads();

  // Reduction.
  for (size_t s = blockDim.x / 2; s > 32; s >>= 1) {
    if (tid < s) {
      windings[tid] += windings[tid + s];
    }
    __syncthreads();
  }

  if (tid < 32) {
    WarpReduceSum<int>(windings, tid);
  }

  if (tid == 0) {
    int sgn = windings[0] > 0 ? -1 : 1;
    result_signs[i] = sgn;
  }
}
}  // namespace

torch::Tensor PointTetMeshTest(torch::Tensor points, torch::Tensor vertices,
                               torch::Tensor tets) {
  CHECK_INPUT(points);
  CHECK_INPUT(vertices);
  CHECK_INPUT(tets);

  TORCH_CHECK(points.size(1) == 3);
  TORCH_CHECK(vertices.size(1) == 3);
  TORCH_CHECK(tets.size(1) == 4);
  TORCH_CHECK(points.dtype() == torch::kFloat32 ||
              points.dtype() == torch::kFloat64);
  TORCH_CHECK(tets.dtype() == torch::kInt32);
  TORCH_CHECK(points.dtype() == vertices.dtype());

  at::cuda::CUDAGuard device_guard{points.device()};
  cudaStream_t stream = at::cuda::getCurrentCUDAStream();

  size_t num_points = points.size(0);
  size_t num_tets = tets.size(0);
  torch::Tensor result_signs =
      torch::zeros({static_cast<int>(num_points)}, tets.options()).contiguous();

  size_t num_threads = 128;
  dim3 num_blocks(num_points);
  size_t shared_size = num_threads * sizeof(int);
  AT_DISPATCH_FLOATING_TYPES(points.scalar_type(), "PointTetMeshTest", [&] {
    PointTetMeshTestKernel<scalar_t>
        <<<num_blocks, num_threads, shared_size, stream>>>(
            points.contiguous().data_ptr<scalar_t>(),
            vertices.contiguous().data_ptr<scalar_t>(),
            tets.contiguous().data_ptr<int>(), num_tets,
            result_signs.data_ptr<int>());
  });

  AT_CUDA_CHECK(cudaGetLastError());

  return result_signs;
}
}  // namespace geomlib
