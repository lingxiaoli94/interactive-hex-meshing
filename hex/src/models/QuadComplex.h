#pragma once

#include "common.h"

#include "QuadrilateralMesh.h"

namespace hex {
class QuadComplex {
 public:
  using Patch = std::vector<size_t>;

  QuadComplex(std::vector<Vector3f> vertices, std::vector<Vector4i> quads,
              std::vector<Patch> patches);
  QuadComplex(const QuadComplex& other) = default;
  QuadComplex& operator=(const QuadComplex& other) = delete;
  ~QuadComplex() = default;

  const std::vector<Patch>& GetPatches() const { return patches_; }
  const std::vector<Vector3f>& GetVertices() const { return vertices_; }
  const std::vector<Vector4i>& GetQuads() const { return quads_; }
  // Rims are three neighboring vertices on a straight line on the surface.
  std::vector<Vector3i> GetRims() const;
  std::vector<Vector2i> GetBiEdges() const;
  // Ring is used to estimate per-vertex normals on a quad mesh.
  std::vector<Vector3i> GetRing() const;
  std::vector<Vector3i> GetTriangularFaces() const;
  std::vector<int> GetDegrees() const;
  void UpdateVertices(std::vector<Vector3f> vertices);

  std::unique_ptr<QuadrilateralMesh> ExtractQuadMesh() const;

 private:
  std::vector<Vector3f> vertices_;
  std::vector<Vector4i> quads_;
  std::vector<Patch> patches_;
};
}  // namespace hex
