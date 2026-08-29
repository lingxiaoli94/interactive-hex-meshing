#pragma once

// Lets the shared per-element math (`__host__ __device__` templates operating on
// float3/double3) be compiled by a plain C++ compiler when geomlib is built
// without CUDA.
//
// Keyed on the build option, not on `__CUDACC__`: in a CUDA-enabled build the
// .cpp translation units are still compiled by the host compiler, and they must
// use CUDA's own float3/double3 so both halves of the library agree on the type.
// Redefining those inside a CUDA translation unit would be a hard error, and
// defining a look-alike in one TU while the .cu uses CUDA's would be an ODR
// violation, so this header never introduces its own types in a CUDA build.
//
// `vector_types.h` pulls in `crt/host_defines.h`, which expands `__host__` and
// `__device__` to nothing outside nvcc, so the decorations cost nothing there.
#if HEX_ENABLE_CUDA

#include <vector_functions.h>
#include <vector_types.h>

#else

struct float3 {
  float x, y, z;
};
struct double3 {
  double x, y, z;
};

inline float3 make_float3(float x, float y, float z) { return {x, y, z}; }
inline double3 make_double3(double x, double y, double z) { return {x, y, z}; }

#endif  // HEX_ENABLE_CUDA

// `__global__` is deliberately NOT defined away. Kernels are excluded from
// CPU-only compilation by splitting them into .cu translation units that plain
// g++ never sees; an empty `__global__` would instead let a kernel that leaked
// into a host TU compile as ordinary -- and silently wrong -- host code.
#ifndef __host__
#define __host__
#endif
#ifndef __device__
#define __device__
#endif
