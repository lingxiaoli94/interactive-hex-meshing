#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <driver_types.h>
#include <torch/torch.h>

namespace geomlib {
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
#define CHECK_CONTIGUOUS(x) \
  TORCH_CHECK(x.is_contiguous(), #x " must be contiguous")
#define CHECK_INPUT(x) \
  CHECK_ON_CUDA(x);    \
  CHECK_CONTIGUOUS(x)
}  // namespace geomlib
