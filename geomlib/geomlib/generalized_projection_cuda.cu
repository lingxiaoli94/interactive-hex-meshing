// clang-format off
#include "generalized_projection.h"
// clang-format on

#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>

#include "common.cuh"
#include "utils.cuh"
#include "vec_utils.cuh"

namespace geomlib {
    namespace {
        template <int dim, typename scalar_t>
            __device__ inline void ComputeBarycentricGradient(
                    const scalar_t* e1, const scalar_t* e2, const scalar_t e1_dot_e2,
                    const scalar_t e1_norm_sqr, const scalar_t e2_norm_sqr, scalar_t* grad_w1,
                    scalar_t* grad_w2) {
                {
                    zero_out_vec<dim>(grad_w1);
                    add_vec<dim>(grad_w1, e1);
                    scalar_t tmp[dim];
                    scalar_times_vec<dim>(-e1_dot_e2 / e2_norm_sqr, e2, tmp);
                    add_vec<dim>(grad_w1, tmp);
                    scalar_t denom = e1_norm_sqr - e1_dot_e2 * e1_dot_e2 / e2_norm_sqr;
                    scalar_times_vec<dim>(1 / denom, grad_w1, grad_w1);
                }
                {
                    zero_out_vec<dim>(grad_w2);
                    add_vec<dim>(grad_w2, e2);
                    scalar_t tmp[dim];
                    scalar_times_vec<dim>(-e1_dot_e2 / e1_norm_sqr, e1, tmp);
                    add_vec<dim>(grad_w2, tmp);
                    scalar_t denom = e2_norm_sqr - e1_dot_e2 * e1_dot_e2 / e1_norm_sqr;
                    scalar_times_vec<dim>(1 / denom, grad_w2, grad_w2);
                }
            }

        template <int dim, typename scalar_t>
            __device__ void GeneralizedTriangleProjection(
                    const scalar_t* p, const scalar_t* v0, const scalar_t* e1,
                    const scalar_t* e2, const scalar_t e1_dot_e2, const scalar_t e1_norm_sqr,
                    const scalar_t e2_norm_sqr, const scalar_t* grad_w1,
                    const scalar_t* grad_w2,
                    // Below are results:
                    scalar_t* result_dist, scalar_t* result_w1, scalar_t* result_w2) {
                scalar_t w1, w2;
                {
                    scalar_t
                        p_minus_v0[dim];  // this array is unavoidable since we need to compute
                    // this for every pair of query point and face

                    minus_vec<dim>(p, v0, p_minus_v0);

                    w1 = dot_vec<dim>(grad_w1, p_minus_v0);
                    w2 = dot_vec<dim>(grad_w2, p_minus_v0);


                    if (0 <= 1 - w1 - w2 && 0 <= w1 && 0 <= w2) {
                        scalar_t p_proj[dim];
                        {
                            {
                                scalar_t w1_e1[dim];
                                scalar_times_vec<dim>(w1, e1, w1_e1);
                                scalar_times_vec<dim>(w2, e2, p_proj);
                                add_vec<dim>(p_proj, w1_e1);
                            }
                        }
                        *result_dist = distance_sqr_vec<dim>(p_minus_v0, p_proj);
                        *result_w1 = w1;
                        *result_w2 = w2;
                    } else {
                        // Project to three edges.
                        scalar_t w1_tmp[3];
                        scalar_t w2_tmp[3];

                        w1_tmp[0] = clamp01(dot_vec<dim>(p_minus_v0, e1) / e1_norm_sqr);
                        w2_tmp[0] = 0;

                        w2_tmp[1] = clamp01(dot_vec<dim>(p_minus_v0, e2) / e2_norm_sqr);
                        w1_tmp[1] = 0;

                        {
                            scalar_t numer = dot_vec<dim>(p_minus_v0, e2) -
                                dot_vec<dim>(p_minus_v0, e1) - e1_dot_e2 + e1_norm_sqr;
                            scalar_t denom = e1_norm_sqr + e2_norm_sqr - 2 * e1_dot_e2;
                            w2_tmp[2] = clamp01(numer / denom);
                            w1_tmp[2] = 1 - w2_tmp[2];
                        }

                        scalar_t best_dist = FLT_MAX;
                        int best_k = -1;

                        for (int k = 0; k < 3; k++) {
                            scalar_t p_proj[dim];
                            scalar_t w1_e1[dim];
                            scalar_times_vec<dim>(w1_tmp[k], e1, w1_e1);
                            scalar_times_vec<dim>(w2_tmp[k], e2, p_proj);
                            add_vec<dim>(p_proj, w1_e1);
                            scalar_t dist = distance_sqr_vec<dim>(p_minus_v0, p_proj);
                            if (dist < best_dist) {
                                best_dist = dist;
                                best_k = k;
                            }
                        }

                        *result_dist = best_dist;
                        *result_w1 = w1_tmp[best_k];
                        *result_w2 = w2_tmp[best_k];
                    }
                }
            }

        template <int dim, typename scalar_t>
            __device__ void GeneralizedTetrahedronProjection(
                    const scalar_t* p,              // D
                    const scalar_t* v0,             // D
                    const scalar_t* e_mat,          // 3xD,
                    const scalar_t* e_dot_mat,      // 3x3
                    const scalar_t* e_dot_inv_mat,  // 3x3
                    // Below are results:
                    scalar_t* result_dist,    // scalar
                    scalar_t* result_weights  // 3
                    ) {
                scalar_t p_minus_v0[dim];
                minus_vec<dim>(p, v0, p_minus_v0);

                scalar_t weights[3];
                {
                    scalar_t b[3];
                    for (int i = 0; i < 3; i++) {
                        b[i] = dot_vec<dim>(&e_mat[i * dim], p_minus_v0);
                    }
                    mat_vec_mult<3>(e_dot_inv_mat, b, weights);
                }

                // TODO: if adding break or has_negative_weight as a terminating condition,
                // then the result would be wrong. Is this a CUDA bug?
                bool has_negative_weight = false;
                {
                    scalar_t grad_wj[dim];
                    scalar_t grad_wk[dim];
                    scalar_t
                        tmp_weights[3];  // if using result_dist directly it would be buggy??
                    for (int i = 0; i < 3; i++) {
                        if (weights[i] < 0) {
                            int j = (i + 1) % 3;
                            int k = (i + 2) % 3;
                            ComputeBarycentricGradient<dim>(
                                    &e_mat[j * dim], &e_mat[k * dim], e_dot_mat[j * 3 + k],
                                    e_dot_mat[j * 3 + j], e_dot_mat[k * 3 + k], grad_wj, grad_wk);

                            GeneralizedTriangleProjection<dim>(
                                    p, v0, &e_mat[j * dim], &e_mat[k * dim], e_dot_mat[j * 3 + k],
                                    e_dot_mat[j * 3 + j], e_dot_mat[k * 3 + k], grad_wj, grad_wk,
                                    result_dist, &tmp_weights[j], &tmp_weights[k]);
                            tmp_weights[i] = 0;
                            has_negative_weight = true;
                            break;
                        }
                    }

                    if (has_negative_weight) {
                        copy_vec<3>(result_weights, tmp_weights);
                    }
                }

                if (!has_negative_weight) {
                    scalar_t weight_op = 1 - weights[0] - weights[1] - weights[2];
                    if (weight_op >= 0) {
                        for (int i = 0; i < 3; i++) {
                            result_weights[i] = weights[i];
                        }

                        // Compute distance.
                        scalar_t p_proj[dim];
                        zero_out_vec<dim>(p_proj);
                        for (int i = 0; i < 3; i++) {
                            scalar_t tmp[dim];
                            scalar_times_vec<dim>(result_weights[i], &e_mat[i * dim], tmp);
                            add_vec<dim>(p_proj, tmp);
                        }
                        *result_dist = distance_sqr_vec<dim>(p, p_proj);
                    } else {
                        // Project to the side opposite of v0.
                        scalar_t v3[dim];
                        plus_vec<dim>(v0, &e_mat[2 * dim], v3);
                        scalar_t e31[dim];
                        scalar_t e32[dim];
                        minus_vec<dim>(&e_mat[0 * dim], &e_mat[2 * dim], e31);
                        minus_vec<dim>(&e_mat[1 * dim], &e_mat[2 * dim], e32);
                        scalar_t e31_norm_sqr = dot_vec<dim>(e31, e31);
                        scalar_t e32_norm_sqr = dot_vec<dim>(e32, e32);
                        scalar_t e31_dot_e32 = dot_vec<dim>(e31, e32);

                        scalar_t grad_w31[dim];
                        scalar_t grad_w32[dim];
                        ComputeBarycentricGradient<dim>(e31, e32, e31_dot_e32, e31_norm_sqr,
                                e32_norm_sqr, grad_w31, grad_w32);

                        GeneralizedTriangleProjection<dim>(
                                p, v3, e31, e32, e31_dot_e32, e31_norm_sqr, e32_norm_sqr, grad_w31,
                                grad_w32, result_dist, &result_weights[0], &result_weights[1]);

                        result_weights[2] = 1 - result_weights[0] - result_weights[1];
                    }
                }
            }

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
                            &e_dot_inv_mat[j * 3 * 3], &dist, weights);

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
        std::vector<torch::Tensor> ComputeGeneralizedTriangleProjection(
                torch::Tensor points, const TriangularProjectionInfo& info) {
            CHECK_INPUT(points);

            TORCH_CHECK(points.size(1) == dim);
            TORCH_CHECK(info.dim == dim);
            TORCH_CHECK(points.dtype() == torch::kFloat32 ||
                    points.dtype() == torch::kFloat64);
            TORCH_CHECK(points.dtype() == info.v0.dtype());

            at::cuda::CUDAGuard device_guard{points.device()};
            cudaStream_t stream = at::cuda::getCurrentCUDAStream();

            int num_points = points.size(0);
            int num_faces = info.num_faces;

            torch::Tensor result_dists =
                torch::zeros({num_points}, points.options()).contiguous();
            torch::Tensor result_idxs =
                torch::zeros({num_points}, points.options().dtype(torch::kInt32))
                .contiguous();
            torch::Tensor result_w1 =
                torch::zeros({num_points}, points.options()).contiguous();
            torch::Tensor result_w2 =
                torch::zeros({num_points}, points.options()).contiguous();

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

            return {result_dists, result_idxs.to(torch::kInt64), result_w1, result_w2};
        }

    template <int dim>
        std::vector<torch::Tensor> ComputeGeneralizedTetrahedronProjection(
                torch::Tensor points, torch::Tensor vertices, torch::Tensor tets) {
            CHECK_INPUT(points);
            CHECK_INPUT(vertices);
            CHECK_INPUT(tets);

            TORCH_CHECK(points.size(1) == dim);
            TORCH_CHECK(vertices.size(1) == dim);
            TORCH_CHECK(points.dtype() == torch::kFloat32 ||
                    points.dtype() == torch::kFloat64);
            TORCH_CHECK(tets.dtype() ==
                    torch::kInt64);  // torch requires 64-bit int for indexing
            TORCH_CHECK(points.dtype() == vertices.dtype());

            at::cuda::CUDAGuard device_guard{points.device()};
            cudaStream_t stream = at::cuda::getCurrentCUDAStream();

            // Pre-compute reusable values.
            using namespace torch::indexing;
            std::vector<torch::Tensor> v_list;
            for (int i = 0; i < 4; i++) {
                v_list.push_back(vertices.index({tets.index({Slice(), i}), Slice()}));
            }
            std::vector<torch::Tensor> e_list;
            for (int i = 1; i < 4; i++) {
                e_list.push_back(v_list[i] - v_list[0]);
            }
            std::vector<torch::Tensor> e_dot_list;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    e_dot_list.push_back((e_list[i] * e_list[j]).sum(-1));
                }
            }
            auto e_mat = torch::stack(e_list, 1);                              // Tx3xD
            auto e_dot_mat = torch::stack(e_dot_list, 1).reshape({-1, 3, 3});  // Tx3x3

            auto e_dot_inv_mat = torch::linalg_inv(e_dot_mat);  // Tx3x3

            size_t num_points = points.size(0);
            size_t num_tets = tets.size(0);

            torch::Tensor result_dists =
                torch::zeros({static_cast<int>(num_points)}, points.options())
                .contiguous();
            torch::Tensor result_idxs = torch::zeros({static_cast<int>(num_points)},
                    tets.options().dtype(torch::kInt32))
                .contiguous();
            torch::Tensor result_weights =
                torch::zeros({static_cast<int>(num_points), 3},
                        points.options())
                .contiguous();  // w1, w2, w3 of barycentric coordinates

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
                            v_list[0].contiguous().data_ptr<scalar_t>(),
                            e_mat.contiguous().data_ptr<scalar_t>(),
                            e_dot_mat.contiguous().data_ptr<scalar_t>(),
                            e_dot_inv_mat.contiguous().data_ptr<scalar_t>(),
                            result_dists.data_ptr<scalar_t>(), result_idxs.data_ptr<int>(),
                            result_weights.data_ptr<scalar_t>());
                    });

            AT_CUDA_CHECK(cudaGetLastError());

            return {result_dists, result_idxs.to(torch::kInt64), result_weights};
        }

    // Explicit instantiations.
    template std::vector<torch::Tensor> ComputeGeneralizedTriangleProjection<3>(
            torch::Tensor points, const TriangularProjectionInfo& info);
    template std::vector<torch::Tensor> ComputeGeneralizedTriangleProjection<8>(
            torch::Tensor points, const TriangularProjectionInfo& info);
    template std::vector<torch::Tensor> ComputeGeneralizedTetrahedronProjection<3>(
            torch::Tensor points, torch::Tensor vertices, torch::Tensor tets);

}  // namespace geomlib
