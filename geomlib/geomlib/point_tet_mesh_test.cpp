#include "point_tet_mesh_test.h"

#include "common.cuh"
#include "point_tet_mesh_test_cuda.h"
#include "point_tet_mesh_test_impl.h"

// Compiled in every configuration, by the plain C++ compiler. Holds the public
// dispatcher and the CPU reference loop; the CUDA kernel lives in
// point_tet_mesh_test_cuda.cu, which only exists in a CUDA build.

namespace geomlib {
namespace {
// CPU reference: identical per-point winding test (shares IsPointInTetAt with
// the kernel), just a serial host loop instead of one block per point. Used when
// the inputs are CPU tensors (--device cpu).
template <typename scalar_t>
void PointTetMeshTestCpuKernel(const scalar_t* __restrict__ points,
                               const scalar_t* __restrict__ vertices,
                               const int* __restrict__ tets, size_t num_points,
                               size_t num_tets, int* __restrict__ result_signs) {
  for (size_t i = 0; i < num_points; i++) {
    int winding = 0;
    auto p = &points[i * 3];
    for (size_t j = 0; j < num_tets; j++) {
      if (detail::IsPointInTetAt<scalar_t>(p, vertices, tets, j)) {
        winding += 1;
      }
    }
    result_signs[i] = winding > 0 ? -1 : 1;
  }
}

void PointTetMeshTestCpu(torch::Tensor points, torch::Tensor vertices,
                         torch::Tensor tets, torch::Tensor result_signs) {
  size_t num_points = points.size(0);
  size_t num_tets = tets.size(0);
  AT_DISPATCH_FLOATING_TYPES(points.scalar_type(), "PointTetMeshTestCpu", [&] {
    PointTetMeshTestCpuKernel<scalar_t>(
        points.contiguous().data_ptr<scalar_t>(),
        vertices.contiguous().data_ptr<scalar_t>(),
        tets.contiguous().data_ptr<int>(), num_points, num_tets,
        result_signs.data_ptr<int>());
  });
}
}  // namespace

torch::Tensor PointTetMeshTest(torch::Tensor points, torch::Tensor vertices,
                               torch::Tensor tets) {
  CHECK_CONTIGUOUS(points);
  CHECK_CONTIGUOUS(vertices);
  CHECK_CONTIGUOUS(tets);

  // All three are dereferenced as raw pointers on the device `points` selected.
  CHECK_SAME_DEVICE(points, vertices);
  CHECK_SAME_DEVICE(points, tets);

  TORCH_CHECK(points.size(1) == 3);
  TORCH_CHECK(vertices.size(1) == 3);
  TORCH_CHECK(tets.size(1) == 4);
  TORCH_CHECK(points.dtype() == torch::kFloat32 ||
              points.dtype() == torch::kFloat64);
  TORCH_CHECK(tets.dtype() == torch::kInt32);
  TORCH_CHECK(points.dtype() == vertices.dtype());

  size_t num_points = points.size(0);
  torch::Tensor result_signs =
      torch::zeros({static_cast<int64_t>(num_points)}, tets.options())
          .contiguous();

  if (points.is_cuda()) {
#if HEX_ENABLE_CUDA
    PointTetMeshTestCuda(points, vertices, tets, result_signs);
#else
    TORCH_CHECK(false,
                "geomlib was built without CUDA support; use CPU tensors");
#endif
  } else {
    // CPU path (--device cpu): no CUDA; same winding test on a host loop.
    PointTetMeshTestCpu(points, vertices, tets, result_signs);
  }

  return result_signs;
}
}  // namespace geomlib
