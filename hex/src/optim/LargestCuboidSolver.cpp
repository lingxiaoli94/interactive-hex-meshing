#include "LargestCuboidSolver.h"

#include <stack>

namespace hex {
namespace {
void FindLargestRectangleIn1DHistogram(const std::vector<int>& histogram,
                                       Vector3i& result, int& max_area) {
  // result: (left_idx, right_idx, height).

  std::stack<int> s;

  int n = static_cast<int>(histogram.size());
  max_area = 0;

  for (int i = 0; i <= n;) {
    if (i < n && (s.empty() || histogram[s.top()] <= histogram[i])) {
      s.push(i++);
    } else {
      if (i < n || !s.empty()) {
        int tp = s.top();
        s.pop();

        int area_with_top = histogram[tp] * (s.empty() ? i : i - s.top() - 1);
        if (max_area < area_with_top) {
          max_area = area_with_top;
          result = {s.empty() ? 0 : s.top() + 1, i - 1, histogram[tp]};
        }
      } else {
        // i == n && s.empty()
        break;
      }
    }
  }

  if (max_area > 0) {
    assert(max_area == ((result(1) - result(0) + 1) * result(2)));
  }
}
}  // namespace

LargestCuboidSolver::LargestCuboidSolver(const Array3D<int>& occupancy)
    : occupancy_{occupancy} {}

LargestRectangleSolver::LargestRectangleSolver(const Array2D<int>& occupancy)
    : occupancy_{occupancy} {}

void LargestRectangleSolver::Solve(Vector2i& result_cl, Vector2i& result_cr,
                                   int& max_area) {
  int d0 = occupancy_.GetDim(0);
  int d1 = occupancy_.GetDim(1);

  max_area = 0;
  std::vector<int> histogram(d1, 0);
  for (int i = 0; i < d0; i++) {
    // Update histogram.
    for (int j = 0; j < d1; j++) {
      histogram[j] = occupancy_(i, j) ? histogram[j] + 1 : 0;
    }

    Vector3i res_1d;
    int area;
    FindLargestRectangleIn1DHistogram(histogram, res_1d, area);
    int l = res_1d(0);
    int r = res_1d(1);
    int h = res_1d(2);
    if (max_area < area) {
      max_area = area;
      result_cl = {i - h + 1, l};
      result_cr = {i, r};
    }
  }
  if (max_area > 0) {
    assert(max_area == (result_cr.x() - result_cl.x() + 1) *
                           (result_cr.y() - result_cl.y() + 1));
  }
}

void LargestCuboidSolver::Solve(Vector3i& result_cl, Vector3i& result_cr,
                                int& max_volume) {
  int d0 = occupancy_.GetDim(0);
  int d1 = occupancy_.GetDim(1);
  int d2 = occupancy_.GetDim(2);
  max_volume = 0;
  Array3D<int> histogram(d0, d1, d2);

  // We will use omp to speed things up, so pre-compute histogram.
  // histogram(i, j, k) denote the number of consecutive 1's along i-1, i-2,
  // etc..
  for (int j = 0; j < d1; j++) {
    for (int k = 0; k < d2; k++) {
      histogram(0, j, k) = occupancy_(0, j, k) ? 1 : 0;
    }
  }

  for (int i = 1; i < d0; i++) {
    for (int j = 0; j < d1; j++) {
      for (int k = 0; k < d2; k++) {
        histogram(i, j, k) =
            occupancy_(i, j, k) ? histogram(i - 1, j, k) + 1 : 0;
      }
    }
  }

#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < d0; i++) {
    // Here is the slow part: we enumerate a threshold of height and generate
    // a binary 2D matrix to call the subroutine.
    Vector3i local_cl, local_cr;
    int local_max_volume = 0;
    for (int t = 1; t <= i + 1; t++) {
      Array2D<int> mask(d1, d2);
      for (int j = 0; j < d1; j++) {
        for (int k = 0; k < d2; k++) {
          mask(j, k) = histogram(i, j, k) >= t ? 1 : 0;
        }
      }

      Vector2i cl, cr;
      int max_area;
      LargestRectangleSolver(mask).Solve(cl, cr, max_area);
      int volume = max_area * t;
      if (volume > local_max_volume) {
        local_max_volume = volume;
        local_cl = {i - t + 1, cl.x(), cl.y()};
        local_cr = {i, cr.x(), cr.y()};
      }
    }

#pragma omp critical
    {
      if (local_max_volume > max_volume) {
        max_volume = local_max_volume;
        result_cl = local_cl;
        result_cr = local_cr;
      }
    }
  }

  if (max_volume > 0) {
    assert(max_volume == (result_cr.x() - result_cl.x() + 1) *
                             (result_cr.y() - result_cl.y() + 1) *
                             (result_cr.z() - result_cl.z() + 1));
  }
}

void LargestCuboidSolver::SolveBruteForce(Vector3i& cl, Vector3i& cr,
                                          int& max_volume) {
  int d0 = occupancy_.GetDim(0);
  int d1 = occupancy_.GetDim(1);
  int d2 = occupancy_.GetDim(2);
  Array3D<int> prefix_sum(d0, d1, d2);
  for (int i = 0; i < d0; i++)
    for (int j = 0; j < d1; j++)
      for (int k = 0; k < d2; k++) {
        int v0 = i == 0 ? 0 : prefix_sum(i - 1, j, k);
        int v1 = j == 0 ? 0 : prefix_sum(i, j - 1, k);
        int v2 = k == 0 ? 0 : prefix_sum(i, j, k - 1);
        int v01 = (i == 0 || j == 0) ? 0 : prefix_sum(i - 1, j - 1, k);
        int v02 = (i == 0 || k == 0) ? 0 : prefix_sum(i - 1, j, k - 1);
        int v12 = (j == 0 || k == 0) ? 0 : prefix_sum(i, j - 1, k - 1);
        int v012 =
            (i == 0 || j == 0 || k == 0) ? 0 : prefix_sum(i - 1, j - 1, k - 1);

        prefix_sum(i, j, k) =
            occupancy_(i, j, k) + v0 + v1 + v2 - v01 - v02 - v12 + v012;
      }

  max_volume = 0;
  for (int il = 0; il < d0; il++)
    for (int jl = 0; jl < d1; jl++)
      for (int kl = 0; kl < d2; kl++) {
        for (int ir = il; ir < d0; ir++)
          for (int jr = jl; jr < d1; jr++)
            for (int kr = kl; kr < d2; kr++) {
              int v0 = il == 0 ? 0 : prefix_sum(il - 1, jr, kr);
              int v1 = jl == 0 ? 0 : prefix_sum(ir, jl - 1, kr);
              int v2 = kl == 0 ? 0 : prefix_sum(ir, jr, kl - 1);
              int v01 =
                  (il == 0 || jl == 0) ? 0 : prefix_sum(il - 1, jl - 1, kr);
              int v02 =
                  (il == 0 || kl == 0) ? 0 : prefix_sum(il - 1, jr, kl - 1);
              int v12 =
                  (jl == 0 || kl == 0) ? 0 : prefix_sum(ir, jl - 1, kl - 1);
              int v012 = (il == 0 || jl == 0 || kl == 0)
                             ? 0
                             : prefix_sum(il - 1, jl - 1, kl - 1);

              int count = prefix_sum(ir, jr, kr) - v0 - v1 - v2 + v01 + v02 +
                          v12 - v012;

              if (count == (ir - il + 1) * (jr - jl + 1) * (kr - kl + 1)) {
                if (max_volume < count) {
                  max_volume = count;
                  cl = {il, jl, kl};
                  cr = {ir, jr, kr};
                }
              }
            }
      }
}
}  // namespace hex
