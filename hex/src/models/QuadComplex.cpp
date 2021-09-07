#include "QuadComplex.h"

namespace hex {
std::vector<Vector3i> QuadComplex::GetRims() const {
  // This is a bit hacky, and only makes sense for polycube hex complex!
  std::vector<std::set<size_t>> neighbors(vertices_.size());
  for (auto& quad : quads_) {
    for (int k = 0; k < 4; k++) {
      neighbors[quad[k]].insert(quad[(k + 1) % 4]);
    }
  }

  std::unordered_set<Vector3i, Vector3iHasher> result_set;
  for (size_t u = 0; u < vertices_.size(); u++) {
    for (size_t v : neighbors[u]) {
      for (size_t w : neighbors[v]) {
        Vector3f e_uv = (vertices_[v] - vertices_[u]).normalized();
        Vector3f e_vw = (vertices_[w] - vertices_[v]).normalized();
        if (std::abs(e_uv.dot(e_vw) - 1) < 1e-6f) {
          Vector3i rim{static_cast<int>(u), static_cast<int>(v),
                       static_cast<int>(w)};
          if (!result_set.count({rim[0], rim[1], rim[2]}) &&
              !result_set.count({rim[2], rim[1], rim[0]})) {
            result_set.emplace(rim);
          }
        }
      }
    }
  }

  return {result_set.begin(), result_set.end()};
}
QuadComplex::QuadComplex(std::vector<Vector3f> vertices,
                         std::vector<Vector4i> quads,
                         std::vector<Patch> patches)
    : vertices_{std::move(vertices)},
      quads_{std::move(quads)},
      patches_{std::move(patches)} {}

std::vector<Vector2i> QuadComplex::GetBiEdges() const {
  std::vector<Vector2i> result;
  for (auto& quad : quads_) {
    for (int k = 0; k < 4; k++) {
      int u = quad[k];
      int v = quad[(k + 1) % 4];
      result.push_back({u, v});
      result.push_back({v, u});
    }
  }
  return result;
}

std::vector<Vector3i> QuadComplex::GetRing() const {
  std::vector<Vector3i> result;
  for (auto& quad : quads_) {
    for (int k = 0; k < 4; k++) {
      int u = quad[k];
      int v = quad[(k + 1) % 4];
      int w = quad[(k + 3) % 4];
      result.push_back({u, v, w});
    }
  }
  return result;
}

std::vector<Vector3i> QuadComplex::GetTriangularFaces() const {
  std::vector<Vector3i> result;
  for (auto& quad : quads_) {
    for (int k = 0; k < 4; k += 2) {
      int u = quad[k];
      int v = quad[(k + 1) % 4];
      int w = quad[(k + 3) % 4];
      result.push_back({u, v, w});
    }
  }
  return result;
}

std::vector<int> QuadComplex::GetDegrees() const {
  std::vector<int> degrees(vertices_.size(), 0);
  for (auto& quad : quads_) {
    for (int k = 0; k < 4; k++) {
      int u = quad[k];
      int v = quad[(k + 1) % 4];
      degrees[u]++;
      degrees[v]++;
    }
  }
  return degrees;
}

void QuadComplex::UpdateVertices(std::vector<Vector3f> vertices) {
  vertices_ = std::move(vertices);
}

std::unique_ptr<QuadrilateralMesh> QuadComplex::ExtractQuadMesh() const {
  return std::make_unique<QuadrilateralMesh>(vertices_, quads_);
}
}  // namespace hex
