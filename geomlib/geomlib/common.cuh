#pragma once

#include <torch/torch.h>

#if HEX_ENABLE_CUDA
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <driver_types.h>
#endif

namespace geomlib {
// Stays available in every build: the CPU dispatchers use it too.
#define CHECK_CONTIGUOUS(x) \
  TORCH_CHECK(x.is_contiguous(), #x " must be contiguous")

// The dispatchers pick a device from one tensor and then take raw pointers from
// all of them, so every related tensor must live on that same device. Mixing
// them hands a device pointer to a host loop (or vice versa) and segfaults
// rather than raising -- dispatching on `points.is_cuda()` alone is not enough.
#define CHECK_SAME_DEVICE(ref, x)                                   \
  TORCH_CHECK((x).device() == (ref).device(),                       \
              #x " must be on the same device as " #ref " (got ",   \
              (x).device(), " vs ", (ref).device(), ")")

#if HEX_ENABLE_CUDA
#define CHECK_CUDA(code) __CheckCudaErrors(code, __FILE__, __LINE__)

inline void __CheckCudaErrors(cudaError_t code, const char* file,
                              const int line) {
  if (code != cudaSuccess) {
    fprintf(stderr, "CheckCudaErrors(): %s %s %d\n", cudaGetErrorString(code),
            file, line);
    exit(code);
  }
}

#define CHECK_ON_CUDA(x) TORCH_CHECK(x.is_cuda(), #x " must be a CUDA tensor")
#define CHECK_INPUT(x) \
  CHECK_ON_CUDA(x);    \
  CHECK_CONTIGUOUS(x)
#endif  // HEX_ENABLE_CUDA
}  // namespace geomlib
