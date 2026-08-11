#pragma once

// Per-element projection math shared by the CUDA kernels
// (generalized_projection_cuda.cu) and the CPU reference loops
// (generalized_projection.cpp). Moved here verbatim from the .cu -- original
// formatting preserved so the split is diffable -- so that a CPU-only build,
// where plain g++ never sees a .cu at all, still runs the identical arithmetic.
//
// The `__host__ __device__` decorations expand to nothing outside nvcc; see
// cuda_compat.h.

#include "utils.cuh"
#include "vec_utils.cuh"

namespace geomlib {
    namespace detail {
        const float kEps = 1e-8;
        template <int dim, typename scalar_t>
            __host__ __device__ inline void ComputeBarycentricGradient(
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
            __host__ __device__ void GeneralizedTriangleProjection(
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
            __host__ __device__ void GeneralizedTetrahedronProjection(
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
    }  // namespace detail
}  // namespace geomlib
