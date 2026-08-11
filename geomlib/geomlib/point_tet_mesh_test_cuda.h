#pragma once

// CUDA entry point behind the public dispatcher in point_tet_mesh_test.cpp.
// The whole file is compiled out of a CPU-only build.
#if HEX_ENABLE_CUDA

#include <torch/torch.h>

namespace geomlib {
// `result_signs` is allocated by the caller so that the CPU and CUDA paths
// cannot drift on shape, dtype or device.
void PointTetMeshTestCuda(torch::Tensor points, torch::Tensor vertices,
                          torch::Tensor tets, torch::Tensor result_signs);
}  // namespace geomlib

#endif  // HEX_ENABLE_CUDA
