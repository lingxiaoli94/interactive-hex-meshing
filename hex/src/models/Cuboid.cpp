#include "Cuboid.h"

namespace hex {
Cuboid::Cuboid(const Eigen::Vector3f& center,
               const Eigen::Vector3f& halflengths)
    : center{center}, halflengths{halflengths} {}

Cuboid Cuboid::FromBounds(const Vector3f& lower, const Vector3f& upper) {
  Cuboid cuboid;
  cuboid.SetBound(lower, upper);
  return cuboid;
}

std::pair<Vector3f, Vector3f> Cuboid::GetBound() {
  return {center - halflengths, center + halflengths};
}

void Cuboid::SetBound(const Vector3f& lower, const Vector3f& upper) {
  center = (lower + upper) / 2;
  halflengths = (upper - lower) / 2;
}

std::vector<Vector3f> Cuboid::GetFace(int ax_id, int side) {
  std::vector<Vector3i> pattern;
  if (ax_id == 0) {
    pattern.push_back({side, -1, -1});
    pattern.push_back({side, 1, -1});
    pattern.push_back({side, 1, 1});
    pattern.push_back({side, -1, 1});
  } else if (ax_id == 1) {
    pattern.push_back({-1, side, -1});
    pattern.push_back({-1, side, 1});
    pattern.push_back({1, side, 1});
    pattern.push_back({1, side, -1});
  } else {
    pattern.push_back({-1, -1, side});
    pattern.push_back({1, -1, side});
    pattern.push_back({1, 1, side});
    pattern.push_back({-1, 1, side});
  }
  assert(!pattern.empty());

  if (side == -1) {
    std::reverse(pattern.begin(), pattern.end());
  }

  std::vector<Vector3f> result;
  for (auto& p : pattern) {
    result.push_back(center + halflengths.cwiseProduct(p.cast<float>()));
  }
  return result;
}
}  // namespace hex
