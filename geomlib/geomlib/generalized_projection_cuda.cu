// clang-format off
#include "generalized_projection.h"
// clang-format on

#include <ATen/Functions.h>
#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>

#include "common.cuh"
#include "utils.cuh"
#include "vec_utils.cuh"

namespace geomlib {
    namespace {
        const float kEps = 1e-8;
        template <int dim, typename scalar_t>
            __device__ inline void ComputeBarycentricGradient(
                    const scalar_t* e1, const scalar_t* e2, const scalar_t e1_dot_e2,
                    const scalar_t e1_norm_sqr, const scalar_t e2_norm_sqr, scalar_t* grad_w1,
                    scalar_t* grad_w2) {
                {
                    zero_out_vec<dim>(grad_w1);
                    add_vec<dim>(grad_w1, e1);
                    scalar_t tmp[dim];
                    scalar_times_vec<dim>(-e1_dot_e2 / (kEps + e2_norm_sqr), e2, tmp);
                    add_vec<dim>(grad_w1, tmp);
                    scalar_t denom = e1_norm_sqr - e1_dot_e2 * e1_dot_e2 / (kEps + e2_norm_sqr);
                    scalar_times_vec<dim>(1 / (denom + kEps), grad_w1, grad_w1);
                }
                {
                    zero_out_vec<dim>(grad_w2);
                    add_vec<dim>(grad_w2, e2);
                    scalar_t tmp[dim];
                    scalar_times_vec<dim>(-e1_dot_e2 / (kEps + e1_norm_sqr), e1, tmp);
                    add_vec<dim>(grad_w2, tmp);
                    scalar_t denom = e2_norm_sqr - e1_dot_e2 * e1_dot_e2 / (kEps + e1_norm_sqr);
                    scalar_times_vec<dim>(1 / (denom + kEps), grad_w2, grad_w2);
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
                scalar_t
                    p_minus_v0[dim];  // this array is unavoidable since we need to compute
                // this for every pair of query point and face

                bool degenerate = false;
                minus_vec<dim>(p, v0, p_minus_v0);
                if (grad_w1 == nullptr || grad_w2 == nullptr) {
                    scalar_t b1 = dot_vec<dim>(e1, p_minus_v0);
                    scalar_t b2 = dot_vec<dim>(e2, p_minus_v0);

                    scalar_t det = e1_norm_sqr * e2_norm_sqr - e1_dot_e2 * e1_dot_e2;
                    if (det < -kEps || det > kEps) {
                        // Cramer's rule.
                        w1 = (b1 * e2_norm_sqr - b2 * e1_dot_e2) / det;
                        w2 = (b2 * e1_norm_sqr - b1 * e1_dot_e2) / det;
                    } else {
                        degenerate = true;
                    }
                } else {
                    w1 = dot_vec<dim>(grad_w1, p_minus_v0);
                    w2 = dot_vec<dim>(grad_w2, p_minus_v0);
                }


                if (!degenerate && 0 <= 1 - w1 - w2 && 0 <= w1 && 0 <= w2) {
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

                    w1_tmp[0] = clamp01(dot_vec<dim>(p_minus_v0, e1) / (kEps + e1_norm_sqr));
                    w2_tmp[0] = 0;

                    w2_tmp[1] = clamp01(dot_vec<dim>(p_minus_v0, e2) / (kEps + e2_norm_sqr));
                    w1_tmp[1] = 0;

                    {
                        scalar_t numer = dot_vec<dim>(p_minus_v0, e2) -
                            dot_vec<dim>(p_minus_v0, e1) - e1_dot_e2 + e1_norm_sqr;
                        scalar_t denom = e1_norm_sqr + e2_norm_sqr - 2 * e1_dot_e2;
                        w2_tmp[2] = clamp01(numer / (kEps + denom));
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


        template <int dim, typename scalar_t>
            __device__ void GeneralizedTetrahedronProjection(
                    const scalar_t* p,              // D
                    const scalar_t* v0,             // D
                    const scalar_t* e_mat,          // 3xD,
                    const scalar_t* e_dot_mat,      // 3x3
                    const scalar_t* e_dot_inv_mat,  // 3x3
                    const scalar_t is_degenerate,   // float, 1.0 if degenerate
                    // Below are results:
                    scalar_t* result_dist,    // scalar
                    scalar_t* result_weights  // 3
                    ) {
                bool recurse = true;
                {
                    scalar_t p_minus_v0[dim];
                    minus_vec<dim>(p, v0, p_minus_v0);
                    scalar_t b[3];
                    for (int i = 0; i < 3; i++) {
                        b[i] = dot_vec<dim>(&e_mat[i * dim], p_minus_v0);
                    }
                    if (is_degenerate < 0.5) {
                        scalar_t weights[3];
                        mat_vec_mult<3>(e_dot_inv_mat, b, weights);
                        scalar_t weight_op = 1 - weights[0] - weights[1] - weights[2];
                        if (weights[0] >= 0 && weights[1] >= 0 && weights[2] >= 0
                                && weight_op >= 0) {
                            recurse = false;
                            copy_vec<3>(result_weights, weights);

                            scalar_t p_proj[dim];
                            zero_out_vec<dim>(p_proj);
                            for (int i = 0; i < 3; i++) {
                                scalar_t tmp[dim];
                                scalar_times_vec<dim>(result_weights[i], &e_mat[i * dim], tmp);
                                add_vec<dim>(p_proj, tmp);
                            }
                            *result_dist = distance_sqr_vec<dim>(p, p_proj);
                        }
                    }
                }
                if (recurse) {
                    // Calculate weights by projecting onto each of 4 faces.
                    *result_dist = FLT_MAX;
                    for (int i = 0; i < 4; i++) {
                        scalar_t vc[dim];
                        scalar_t ecj[dim];
                        scalar_t eck[dim];
                        scalar_t ecj_dot_eck;
                        scalar_t ecj_norm_sqr;
                        scalar_t eck_norm_sqr;
                        if (i < 3) {
                            copy_vec<dim>(vc, v0);
                            int j = i;
                            int k = (i + 1) % 3;
                            copy_vec<dim>(ecj, &e_mat[j * dim]);
                            copy_vec<dim>(eck, &e_mat[k * dim]);
                            ecj_dot_eck = e_dot_mat[j * 3 + k];
                            ecj_norm_sqr = e_dot_mat[j * 3 + j];
                            eck_norm_sqr = e_dot_mat[k * 3 + k];
                        } else {
                            plus_vec<dim>(v0, &e_mat[2 * dim], vc);
                            minus_vec<dim>(&e_mat[0 * dim], &e_mat[2 * dim], ecj);
                            minus_vec<dim>(&e_mat[1 * dim], &e_mat[2 * dim], eck);
                            ecj_dot_eck = dot_vec<dim>(ecj, eck);
                            ecj_norm_sqr = dot_vec<dim>(ecj, ecj);
                            eck_norm_sqr = dot_vec<dim>(eck, eck);
                        }
                        scalar_t cur_dist;
                        scalar_t cur_weights[2];
                        GeneralizedTriangleProjection<dim>(
                                p, vc, ecj, eck, ecj_dot_eck, ecj_norm_sqr, eck_norm_sqr,
                                (const scalar_t*)nullptr, (const scalar_t*)nullptr,
                                &cur_dist, &cur_weights[0], &cur_weights[1]);
                        if (cur_dist < *result_dist) {
                            *result_dist = cur_dist;
                            zero_out_vec<3>(result_weights);
                            if (i < 3) {
                                result_weights[i] = cur_weights[0];
                                result_weights[(i + 1) % 3] = cur_weights[1];
                            } else {
                                result_weights[0] = cur_weights[0];
                                result_weights[1] = cur_weights[1];
                                result_weights[2] = 1 - cur_weights[0] - cur_weights[1];
                            }
                        }
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

            auto e_dot_mat_det = torch::linalg_det(e_dot_mat); // T
            auto is_degenerate = (e_dot_mat_det.abs() < kEps).to(points.dtype()); // T
            auto e_dot_inv_mat = torch::linalg_pinv(e_dot_mat);  // Tx3x3

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
                            is_degenerate.contiguous().data_ptr<scalar_t>(),
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
