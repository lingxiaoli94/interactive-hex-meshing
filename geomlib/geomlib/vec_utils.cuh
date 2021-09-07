#pragma once

template <int dim, typename scalar_t>
__device__ inline void zero_out_vec(scalar_t* x) {
  for (int i = 0; i < dim; i++) {
    x[i] = 0;
  }
}

template <int dim, typename scalar_t>
__device__ inline void copy_vec(scalar_t* dest, const scalar_t* src) {
  for (int i = 0; i < dim; i++) {
    dest[i] = src[i];
  }
}

template <int dim, typename scalar_t>
__device__ inline scalar_t dot_vec(const scalar_t* a, const scalar_t* b) {
  scalar_t result = 0;
  for (int i = 0; i < dim; i++) {
    result += a[i] * b[i];
  }
  return result;
}

template <int dim, typename scalar_t>
__device__ inline void plus_vec(const scalar_t* a, const scalar_t* b,
                                scalar_t* result) {
  for (int i = 0; i < dim; i++) {
    result[i] = a[i] + b[i];
  }
}

template <int dim, typename scalar_t>
__device__ inline void add_vec(scalar_t* a, const scalar_t* b) {
  for (int i = 0; i < dim; i++) {
    a[i] += b[i];
  }
}

template <int dim, typename scalar_t>
__device__ inline void scalar_times_vec(scalar_t a, const scalar_t* b,
                                        scalar_t* result) {
  for (int i = 0; i < dim; i++) {
    result[i] = a * b[i];
  }
}

template <int dim, typename scalar_t>
__device__ inline void minus_vec(const scalar_t* a, const scalar_t* b,
                                 scalar_t* result) {
  for (int i = 0; i < dim; i++) {
    result[i] = a[i] - b[i];
  }
}

template <int dim, typename scalar_t>
__device__ inline void mat_vec_mult(const scalar_t* A, const scalar_t* b,
                                    scalar_t* result) {
  for (int i = 0; i < dim; i++) {
    result[i] = 0;
    for (int j = 0; j < dim; j++) {
      result[i] += A[i * dim + j] * b[j];
    }
  }
}
template <int dim, typename scalar_t>
__device__ inline scalar_t distance_sqr_vec(const scalar_t* a,
                                            const scalar_t* b) {
  scalar_t d[dim];
  minus_vec<dim>(a, b, d);
  return dot_vec<dim>(d, d);
}
