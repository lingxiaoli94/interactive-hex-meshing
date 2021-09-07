#pragma once

#include "models/MultiDimArray.h"

namespace hex {
class LargestCuboidSolver {
 public:
  LargestCuboidSolver(const Array3D<int>& occupancy);

  // Return the two corners of the largest cuboid and the largest volume.
  void Solve(Vector3i& cl, Vector3i& cr, int& volume);
  void SolveBruteForce(Vector3i& cl, Vector3i& cr, int& volume);

 private:
  Array3D<int> occupancy_;
};

class LargestRectangleSolver {
 public:
  LargestRectangleSolver(const Array2D<int>& occupancy);

  // Return the two corners of the largest cuboid.
  void Solve(Vector2i& cl, Vector2i& cr, int& area);

 private:
  Array2D<int> occupancy_;
};
}  // namespace hex
