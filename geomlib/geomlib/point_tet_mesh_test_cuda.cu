#include "point_tet_mesh_test_cuda.h"

#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>

#include "common.cuh"
#include "point_tet_mesh_test_impl.h"
#include "utils.cuh"

// Only ever compiled in a CUDA build: geomlib/geomlib/CMakeLists.txt adds this
// file to the target exclusively when HEX_ENABLE_CUDA is ON.

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
    if (detail::IsPointInTetAt<scalar_t>(&points[i * 3], vertices, tets, j)) {
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

void PointTetMeshTestCuda(torch::Tensor points, torch::Tensor vertices,
                          torch::Tensor tets, torch::Tensor result_signs) {
  at::cuda::CUDAGuard device_guard{points.device()};
  cudaStream_t stream = at::cuda::getCurrentCUDAStream();

  size_t num_points = points.size(0);
  size_t num_tets = tets.size(0);

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
}
}  // namespace geomlib
