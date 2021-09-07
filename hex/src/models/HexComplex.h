#pragma once

#include "common.h"

#include <unordered_map>

#include "HexahedralMesh.h"
#include "QuadComplex.h"

namespace hex {
// HexComplex is a decorated version of HexahedralMesh resulted from a polycube
// with surface quads divided into patches.
class HexComplex {
 public:
  using Hex = std::vector<size_t>;

  HexComplex(std::vector<Vector3f> vertices, std::vector<Vector4i> quads,
             std::vector<QuadComplex::Patch> patches, std::vector<Hex> hexes);
  HexComplex(const HexComplex& other);
  HexComplex& operator=(const HexComplex& other) = delete;
  ~HexComplex() = default;

  const QuadComplex& GetQuadComplex() const;
  void BuildQuadComplex() const;
  const std::vector<Vector3f>& GetVertices() const { return vertices_; }
  const std::vector<Vector4i>& GetQuads() const { return quads_; }
  const std::vector<QuadComplex::Patch>& GetPatches() const { return patches_; }
  const std::vector<Hex>& GetHexes() const { return hexes_; }
  const std::vector<size_t>& GetSurfaceIndices() const {
    return surface_indices_;
  }
  const std::unordered_map<size_t, size_t>& GetToSurfaceIndexDict() const {
    return to_surface_index_;
  }
  void UpdateVertices(std::vector<Vector3f> vertices);
  void UpdateSingleVertex(size_t id, const Vector3f& new_position);
  void UpdateSurfaceVertices(const std::vector<Vector3f>& surface_vertices);
  std::unique_ptr<HexahedralMesh> ExtractHexMesh() const;

 private:
  std::vector<Vector3f> vertices_;
  std::vector<Vector4i> quads_;              // surface quads
  std::vector<QuadComplex::Patch> patches_;  // surface patches
  std::vector<Hex> hexes_;
  mutable std::unique_ptr<QuadComplex> quad_complex_;

  std::vector<size_t> surface_indices_;  // indices of surface vertices
  std::unordered_map<size_t, size_t> to_surface_index_;
};
}  // namespace hex
