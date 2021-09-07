#include "HexComplex.h"

#include <utility>

namespace hex {
HexComplex::HexComplex(std::vector<Vector3f> vertices,
                       std::vector<Vector4i> quads,
                       std::vector<QuadComplex::Patch> patches,
                       std::vector<Hex> hexes)
    : vertices_{std::move(vertices)},
      quads_{std::move(quads)},
      patches_{std::move(patches)},
      hexes_{std::move(hexes)} {
  // Build the mapping to surface indices.
  for (auto& quad : quads_) {
    for (size_t k = 0; k < 4; k++) {
      int v = quad[k];
      if (!to_surface_index_.count(v)) {
        to_surface_index_.emplace(v, surface_indices_.size());
        surface_indices_.push_back(v);
      }
    }
  }
}

const QuadComplex& HexComplex::GetQuadComplex() const {
  if (quad_complex_ == nullptr) {
    BuildQuadComplex();
  }
  return *quad_complex_;
}

void HexComplex::BuildQuadComplex() const {
  std::vector<Vector3f> surface_vertices;
  for (auto i : surface_indices_) {
    surface_vertices.push_back(vertices_[i]);
  }

  std::vector<Vector4i> surface_quads;
  for (auto& quad : quads_) {
    Vector4i surface_quad;
    for (size_t k = 0; k < 4; k++) {
      surface_quad[k] = static_cast<int>(to_surface_index_.at(quad[k]));
    }
    surface_quads.push_back(surface_quad);
  }

  quad_complex_ =
      std::make_unique<QuadComplex>(surface_vertices, surface_quads, patches_);
}

void HexComplex::UpdateVertices(std::vector<Vector3f> vertices) {
  assert(vertices.size() == vertices_.size());
  vertices_ = std::move(vertices);
  BuildQuadComplex();
}

void HexComplex::UpdateSingleVertex(size_t id, const Vector3f& new_position) {
  vertices_.at(id) = new_position;
  BuildQuadComplex();
}

void HexComplex::UpdateSurfaceVertices(
    const std::vector<Vector3f>& surface_vertices) {
  assert(surface_vertices.size() == surface_indices_.size());
  for (size_t i = 0; i < surface_vertices.size(); i++) {
    vertices_[surface_indices_[i]] = surface_vertices[i];
  }
  BuildQuadComplex();
}

std::unique_ptr<HexahedralMesh> HexComplex::ExtractHexMesh() const {
  return std::make_unique<HexahedralMesh>(GetVertices(), GetHexes());
}

HexComplex::HexComplex(const HexComplex& other)
    : vertices_{other.vertices_},
      quads_{other.quads_},
      patches_{other.patches_},
      hexes_{other.hexes_},
      surface_indices_{other.surface_indices_},
      to_surface_index_{other.to_surface_index_} {
  if (other.quad_complex_ != nullptr) {
    quad_complex_ = std::make_unique<QuadComplex>(*other.quad_complex_);
  }
}

}  // namespace hex
