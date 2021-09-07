#pragma once

#include "HexMeshQuality.h"
#include "common.h"

namespace hex {
class HexahedralMesh {
 public:
  using Hex = std::vector<size_t>;

  HexahedralMesh(const std::string& file_name);
  HexahedralMesh(std::vector<Vector3f> vertices, std::vector<Hex> hexes);
  const std::vector<Vector3f>& GetVertices() const;
  const std::vector<Hex>& GetHexes() const;
  const HexMeshQuality& GetMeshQuality() const;

 private:
  std::vector<Vector3f> vertices_;
  std::vector<Hex> hexes_;
  mutable std::unique_ptr<HexMeshQuality> mesh_quality_;
};
}  // namespace hex
