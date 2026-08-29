// clang-format off
#include "generalized_projection_cuda.h"
// clang-format on

#include <ATen/Functions.h>
#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>

#include "common.cuh"
#include "generalized_projection_impl.h"
#include "utils.cuh"
#include "vec_utils.cuh"

// Only ever compiled in a CUDA build: geomlib/geomlib/CMakeLists.txt adds this
// file to the target exclusively when HEX_ENABLE_CUDA is ON.

namespace geomlib {
    namespace {
        using detail::GeneralizedTetrahedronProjection;
        using detail::GeneralizedTriangleProjection;

        template <int dim, typename scalar_t>
            __global__ void GeneralizedTriangleProjectionKernel(
                    const scalar_t* __restrict__ points,  // P x D
                    const size_t num_faces,
                    // Pre-computed values:
                    const scalar_t* __restrict__ v0,           // FxD, vertex 0 of faces
                    const scalar_t* __restrict__ e1,           // FxD, vertex 1 - vertex 0
                    const scalar_t* __restrict__ e2,           // FxD
                    const scalar_t* __restrict__ e1_dot_e2,    // F
                    const scalar_t* __restrict__ e1_norm_sqr,  // F
                    const scalar_t* __restrict__ e2_norm_sqr,  // F
                    const scalar_t* __restrict__ grad_w1,      // FxD, dw1/dp on the face
                    const scalar_t* __restrict__ grad_w2,      // FxD
                    // Results:
                    scalar_t* __restrict__ result_dists, int* __restrict__ result_idxs,
                    scalar_t* __restrict__ result_w1, scalar_t* __restrict__ result_w2) {
                extern __shared__ char shared_buf[];
                scalar_t* min_dists = (scalar_t*)shared_buf;         // scalar_t[num_threads]
                size_t* min_tids = (size_t*)&min_dists[blockDim.x];  // size_t[num_threads]

                // Each block computing the projection of a single point.
                const size_t i = blockIdx.x;
                // Faces are divided evenly among the threads.
                const size_t tid = threadIdx.x;

                const scalar_t* p = &points[i * dim];  // point to project

                scalar_t min_dist = FLT_MAX;
                size_t min_idx = 0;
                scalar_t min_w1;
                scalar_t min_w2;

                for (int j = tid; j < num_faces; j += blockDim.x) {
                    scalar_t dist, w1, w2;
                    GeneralizedTriangleProjection<dim, scalar_t>(
                            p, &v0[j * dim], &e1[j * dim], &e2[j * dim], e1_dot_e2[j],
                            e1_norm_sqr[j], e2_norm_sqr[j], &grad_w1[j * dim], &grad_w2[j * dim],
                            &dist, &w1, &w2);

                    if (dist < min_dist) {
                        min_dist = dist;
                        min_idx = j;
                        min_w1 = w1;
                        min_w2 = w2;
                    }
                }
                min_dists[tid] = min_dist;
                min_tids[tid] = tid;
                __syncthreads();

                ReduceMin(min_dists, min_tids, tid);

                __syncthreads();

                // Finally thread with min_dist writes the result to the output.
                if (tid == min_tids[0]) {
                    result_dists[i] = min_dist;  // squared minimum distance
                    result_idxs[i] = min_idx;
                    assert(min_idx < num_faces);
                    result_w1[i] = min_w1;
                    result_w2[i] = min_w2;
                }
            }

        template <int dim, typename scalar_t>
            __global__ void GeneralizedTetrahedronProjectionKernel(
                    const scalar_t* __restrict__ points,  // P x D
                    const size_t num_tets,
                    // Pre-computed values:
                    const scalar_t* __restrict__ v0,             // TxD, vertex 0 of faces
                    const scalar_t* __restrict__ e_mat,          // Tx3xD, vertex i - vertex 0
                    const scalar_t* __restrict__ e_dot_mat,      // Tx3x3
                    const scalar_t* __restrict__ e_dot_inv_mat,  // Tx3x3
                    const scalar_t* __restrict__ is_degenerate,  // T
                    // Results:
                    scalar_t* __restrict__ result_dists, int* __restrict__ result_idxs,
                    scalar_t* __restrict__ result_weights  // Px3
                    ) {
                extern __shared__ char shared_buf[];
                scalar_t* min_dists = (scalar_t*)shared_buf;         // scalar_t[num_threads]
                size_t* min_tids = (size_t*)&min_dists[blockDim.x];  // size_t[num_threads]

                // Each block computing the projection of a single point.
                const size_t i = blockIdx.x;
                // Faces are divided evenly among the threads.
                const size_t tid = threadIdx.x;

                const scalar_t* p = &points[i * dim];  // point to project

                scalar_t min_dist = FLT_MAX;
                size_t min_idx = 0;
                scalar_t min_weights[3];

                for (int j = tid; j < num_tets; j += blockDim.x) {
                    scalar_t dist;
                    scalar_t weights[3];
                    GeneralizedTetrahedronProjection<dim, scalar_t>(
                            p, &v0[j * dim], &e_mat[j * 3 * dim], &e_dot_mat[j * 3 * 3],
                            &e_dot_inv_mat[j * 3 * 3], is_degenerate[j],
                            &dist, weights);

                    if (dist < min_dist) {
                        min_dist = dist;
                        min_idx = j;
                        copy_vec<3>(min_weights, weights);
                    }
                }
                min_dists[tid] = min_dist;
                min_tids[tid] = tid;
                __syncthreads();

                ReduceMin(min_dists, min_tids, tid);

                // Finally thread with min_dist writes the result to the output.
                if (tid == min_tids[0]) {
                    result_dists[i] = min_dist;  // squared minimum distance
                    result_idxs[i] = min_idx;
                    assert(min_idx < num_tets);
                    copy_vec<3>(&result_weights[i * 3], min_weights);
                }
            }
    }  // namespace

    template <int dim>
        void ComputeGeneralizedTriangleProjectionCuda(
                torch::Tensor points, const TriangularProjectionInfo& info,
                torch::Tensor result_dists, torch::Tensor result_idxs,
                torch::Tensor result_w1, torch::Tensor result_w2) {
            int num_points = points.size(0);
            int num_faces = info.num_faces;

            at::cuda::CUDAGuard device_guard{points.device()};
            cudaStream_t stream = at::cuda::getCurrentCUDAStream();

            size_t num_threads = 128;
            dim3 num_blocks(num_points);
            size_t shared_size =
                num_threads *
                ((points.dtype() == torch::kFloat32 ? sizeof(float) : sizeof(double)) +
                 sizeof(size_t));

            AT_DISPATCH_FLOATING_TYPES(
                    points.scalar_type(), "ComputeGeneralizedTriangleProjection", [&] {
                    GeneralizedTriangleProjectionKernel<dim, scalar_t>
                    <<<num_blocks, num_threads, shared_size, stream>>>(
                            points.contiguous().data_ptr<scalar_t>(),
                            static_cast<size_t>(num_faces),
                            info.v0.contiguous().data_ptr<scalar_t>(),
                            info.e1.contiguous().data_ptr<scalar_t>(),
                            info.e2.contiguous().data_ptr<scalar_t>(),
                            info.e1_dot_e2.contiguous().data_ptr<scalar_t>(),
                            info.e1_norm_sqr.contiguous().data_ptr<scalar_t>(),
                            info.e2_norm_sqr.contiguous().data_ptr<scalar_t>(),
                            info.grad_w1.contiguous().data_ptr<scalar_t>(),
                            info.grad_w2.contiguous().data_ptr<scalar_t>(),
                            result_dists.data_ptr<scalar_t>(), result_idxs.data_ptr<int>(),
                            result_w1.data_ptr<scalar_t>(), result_w2.data_ptr<scalar_t>());
                    });

            AT_CUDA_CHECK(cudaGetLastError());
        }

    template <int dim>
        void ComputeGeneralizedTetrahedronProjectionCuda(
                torch::Tensor points, size_t num_tets, torch::Tensor v0,
                torch::Tensor e_mat, torch::Tensor e_dot_mat,
                torch::Tensor e_dot_inv_mat, torch::Tensor is_degenerate,
                torch::Tensor result_dists, torch::Tensor result_idxs,
                torch::Tensor result_weights) {
            size_t num_points = points.size(0);

            at::cuda::CUDAGuard device_guard{points.device()};
            cudaStream_t stream = at::cuda::getCurrentCUDAStream();

            size_t num_threads = 128;
            dim3 num_blocks(num_points);
            size_t shared_size =
                num_threads *
                ((points.dtype() == torch::kFloat32 ? sizeof(float) : sizeof(double)) +
                 sizeof(size_t));

            AT_DISPATCH_FLOATING_TYPES(
                    points.scalar_type(), "ComputeGeneralizedTetrahedronProjection", [&] {
                    GeneralizedTetrahedronProjectionKernel<dim, scalar_t>
                    <<<num_blocks, num_threads, shared_size, stream>>>(
                            points.contiguous().data_ptr<scalar_t>(), num_tets,
                            v0.contiguous().data_ptr<scalar_t>(),
                            e_mat.contiguous().data_ptr<scalar_t>(),
                            e_dot_mat.contiguous().data_ptr<scalar_t>(),
                            e_dot_inv_mat.contiguous().data_ptr<scalar_t>(),
                            is_degenerate.contiguous().data_ptr<scalar_t>(),
                            result_dists.data_ptr<scalar_t>(), result_idxs.data_ptr<int>(),
                            result_weights.data_ptr<scalar_t>());
                    });

            AT_CUDA_CHECK(cudaGetLastError());
        }

    // Explicit instantiations.
    template void ComputeGeneralizedTriangleProjectionCuda<3>(
            torch::Tensor points, const TriangularProjectionInfo& info,
            torch::Tensor result_dists, torch::Tensor result_idxs,
            torch::Tensor result_w1, torch::Tensor result_w2);
    template void ComputeGeneralizedTriangleProjectionCuda<8>(
            torch::Tensor points, const TriangularProjectionInfo& info,
            torch::Tensor result_dists, torch::Tensor result_idxs,
            torch::Tensor result_w1, torch::Tensor result_w2);
    template void ComputeGeneralizedTetrahedronProjectionCuda<3>(
            torch::Tensor points, size_t num_tets, torch::Tensor v0,
            torch::Tensor e_mat, torch::Tensor e_dot_mat,
            torch::Tensor e_dot_inv_mat, torch::Tensor is_degenerate,
            torch::Tensor result_dists, torch::Tensor result_idxs,
            torch::Tensor result_weights);

}  // namespace geomlib
