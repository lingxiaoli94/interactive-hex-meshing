#pragma once

#include "common.h"

namespace hex {
struct Cuboid {
  Cuboid() = default;
  Cuboid(const Vector3f& center, const Vector3f& halflengths);
  static Cuboid FromBounds(const Vector3f& lower, const Vector3f& upper);
  std::pair<Vector3f, Vector3f> GetBound();
  void SetBound(const Vector3f& lower, const Vector3f& upper);

  // GetFace returns 4 points on a face. ax_id can be 0, 1, 2 and side can be 1
  // or -1.
  std::vector<Vector3f> GetFace(int ax_id, int side);

  Vector3f center{0.0f, 0.0f, 0.0f};
  Vector3f halflengths{1.0f, 1.0f, 1.0f};
};

}  // namespace hex
