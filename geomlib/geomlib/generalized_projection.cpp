// clang-format off
#include "generalized_projection.h"
// clang-format on

#include <ATen/Functions.h>

#include "common.cuh"
#include "generalized_projection_cuda.h"
#include "generalized_projection_impl.h"

// Compiled in every configuration, by the plain C++ compiler. Holds the public
// dispatchers, the CPU reference loops and the explicit instantiations; the
// CUDA kernels live in generalized_projection_cuda.cu, which only exists in a
// CUDA build. Bodies moved verbatim from the former .cu -- original formatting
// preserved so the split is diffable.

namespace geomlib {
    namespace {
        using detail::GeneralizedTetrahedronProjection;
        using detail::GeneralizedTriangleProjection;

        // ---- CPU reference loops (--device cpu): the SAME per-element
        // projection math as the CUDA kernels (shared via
        // generalized_projection_impl.h), run as a serial host loop with a plain
        // min-reduction. Results match the CUDA path by construction. ----
        template <int dim, typename scalar_t>
            void GeneralizedTriangleProjectionCpu(
                    const scalar_t* points, size_t num_points, size_t num_faces,
                    const scalar_t* v0, const scalar_t* e1, const scalar_t* e2,
                    const scalar_t* e1_dot_e2, const scalar_t* e1_norm_sqr,
                    const scalar_t* e2_norm_sqr, const scalar_t* grad_w1,
                    const scalar_t* grad_w2, scalar_t* result_dists,
                    int* result_idxs, scalar_t* result_w1, scalar_t* result_w2) {
                for (size_t i = 0; i < num_points; i++) {
                    const scalar_t* p = &points[i * dim];
                    scalar_t min_dist = FLT_MAX;
                    size_t min_idx = 0;
                    scalar_t min_w1 = 0, min_w2 = 0;
                    for (size_t j = 0; j < num_faces; j++) {
                        scalar_t dist, w1, w2;
                        GeneralizedTriangleProjection<dim, scalar_t>(
                                p, &v0[j * dim], &e1[j * dim], &e2[j * dim],
                                e1_dot_e2[j], e1_norm_sqr[j], e2_norm_sqr[j],
                                &grad_w1[j * dim], &grad_w2[j * dim], &dist, &w1, &w2);
                        if (dist < min_dist) {
                            min_dist = dist; min_idx = j; min_w1 = w1; min_w2 = w2;
                        }
                    }
                    result_dists[i] = min_dist;
                    result_idxs[i] = static_cast<int>(min_idx);
                    result_w1[i] = min_w1;
                    result_w2[i] = min_w2;
                }
            }

        template <int dim, typename scalar_t>
            void GeneralizedTetrahedronProjectionCpu(
                    const scalar_t* points, size_t num_points, size_t num_tets,
                    const scalar_t* v0, const scalar_t* e_mat,
                    const scalar_t* e_dot_mat, const scalar_t* e_dot_inv_mat,
                    const scalar_t* is_degenerate, scalar_t* result_dists,
                    int* result_idxs, scalar_t* result_weights) {
                for (size_t i = 0; i < num_points; i++) {
                    const scalar_t* p = &points[i * dim];
                    scalar_t min_dist = FLT_MAX;
                    size_t min_idx = 0;
                    scalar_t min_weights[3] = {0, 0, 0};
                    for (size_t j = 0; j < num_tets; j++) {
                        scalar_t dist;
                        scalar_t weights[3];
                        GeneralizedTetrahedronProjection<dim, scalar_t>(
                                p, &v0[j * dim], &e_mat[j * 3 * dim],
                                &e_dot_mat[j * 3 * 3], &e_dot_inv_mat[j * 3 * 3],
                                is_degenerate[j], &dist, weights);
                        if (dist < min_dist) {
                            min_dist = dist; min_idx = j;
                            copy_vec<3>(min_weights, weights);
                        }
                    }
                    result_dists[i] = min_dist;
                    result_idxs[i] = static_cast<int>(min_idx);
                    copy_vec<3>(&result_weights[i * 3], min_weights);
                }
            }
    }  // namespace

    template <int dim>
        std::vector<torch::Tensor> ComputeGeneralizedTriangleProjection(
                torch::Tensor points, const TriangularProjectionInfo& info) {
            CHECK_CONTIGUOUS(points);

            // Every cached projection tensor is dereferenced as a raw pointer
            // on whatever device `points` selected.
            CHECK_SAME_DEVICE(points, info.v0);
            CHECK_SAME_DEVICE(points, info.e1);
            CHECK_SAME_DEVICE(points, info.e2);
            CHECK_SAME_DEVICE(points, info.e1_dot_e2);
            CHECK_SAME_DEVICE(points, info.e1_norm_sqr);
            CHECK_SAME_DEVICE(points, info.e2_norm_sqr);
            CHECK_SAME_DEVICE(points, info.grad_w1);
            CHECK_SAME_DEVICE(points, info.grad_w2);

            TORCH_CHECK(points.size(1) == dim);
            TORCH_CHECK(info.dim == dim);
            TORCH_CHECK(points.dtype() == torch::kFloat32 ||
                    points.dtype() == torch::kFloat64);
            TORCH_CHECK(points.dtype() == info.v0.dtype());

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

            if (points.is_cuda()) {
#if HEX_ENABLE_CUDA
                ComputeGeneralizedTriangleProjectionCuda<dim>(
                        points, info, result_dists, result_idxs, result_w1, result_w2);
#else
                TORCH_CHECK(false,
                        "geomlib was built without CUDA support; use CPU tensors");
#endif
            } else {
                // CPU path (--device cpu): same projection math, host loop.
                AT_DISPATCH_FLOATING_TYPES(
                        points.scalar_type(),
                        "ComputeGeneralizedTriangleProjectionCpu", [&] {
                        GeneralizedTriangleProjectionCpu<dim, scalar_t>(
                                points.contiguous().data_ptr<scalar_t>(),
                                static_cast<size_t>(num_points),
                                static_cast<size_t>(num_faces),
                                info.v0.contiguous().data_ptr<scalar_t>(),
                                info.e1.contiguous().data_ptr<scalar_t>(),
                                info.e2.contiguous().data_ptr<scalar_t>(),
                                info.e1_dot_e2.contiguous().data_ptr<scalar_t>(),
                                info.e1_norm_sqr.contiguous().data_ptr<scalar_t>(),
                                info.e2_norm_sqr.contiguous().data_ptr<scalar_t>(),
                                info.grad_w1.contiguous().data_ptr<scalar_t>(),
                                info.grad_w2.contiguous().data_ptr<scalar_t>(),
                                result_dists.data_ptr<scalar_t>(),
                                result_idxs.data_ptr<int>(),
                                result_w1.data_ptr<scalar_t>(),
                                result_w2.data_ptr<scalar_t>());
                        });
            }

            return {result_dists, result_idxs.to(torch::kInt64), result_w1, result_w2};
        }

    template <int dim>
        std::vector<torch::Tensor> ComputeGeneralizedTetrahedronProjection(
                torch::Tensor points, torch::Tensor vertices, torch::Tensor tets) {
            CHECK_CONTIGUOUS(points);
            CHECK_CONTIGUOUS(vertices);
            CHECK_CONTIGUOUS(tets);

            // The pre-computation below indexes `vertices` with `tets`, then the
            // loop/kernel takes raw pointers into the results.
            CHECK_SAME_DEVICE(points, vertices);
            CHECK_SAME_DEVICE(points, tets);

            TORCH_CHECK(points.size(1) == dim);
            TORCH_CHECK(vertices.size(1) == dim);
            TORCH_CHECK(points.dtype() == torch::kFloat32 ||
                    points.dtype() == torch::kFloat64);
            TORCH_CHECK(tets.dtype() ==
                    torch::kInt64);  // torch requires 64-bit int for indexing
            TORCH_CHECK(points.dtype() == vertices.dtype());

            // Pre-compute reusable values (device-agnostic torch ops; run on
            // whatever device `points` is on, CPU or CUDA).
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
            auto is_degenerate =
                (e_dot_mat_det.abs() < detail::kEps).to(points.dtype()); // T
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

            if (points.is_cuda()) {
#if HEX_ENABLE_CUDA
                ComputeGeneralizedTetrahedronProjectionCuda<dim>(
                        points, num_tets, v_list[0], e_mat, e_dot_mat, e_dot_inv_mat,
                        is_degenerate, result_dists, result_idxs, result_weights);
#else
                TORCH_CHECK(false,
                        "geomlib was built without CUDA support; use CPU tensors");
#endif
            } else {
                // CPU path (--device cpu): same projection math, host loop.
                AT_DISPATCH_FLOATING_TYPES(
                        points.scalar_type(),
                        "ComputeGeneralizedTetrahedronProjectionCpu", [&] {
                        GeneralizedTetrahedronProjectionCpu<dim, scalar_t>(
                                points.contiguous().data_ptr<scalar_t>(), num_points,
                                num_tets, v_list[0].contiguous().data_ptr<scalar_t>(),
                                e_mat.contiguous().data_ptr<scalar_t>(),
                                e_dot_mat.contiguous().data_ptr<scalar_t>(),
                                e_dot_inv_mat.contiguous().data_ptr<scalar_t>(),
                                is_degenerate.contiguous().data_ptr<scalar_t>(),
                                result_dists.data_ptr<scalar_t>(),
                                result_idxs.data_ptr<int>(),
                                result_weights.data_ptr<scalar_t>());
                        });
            }

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
