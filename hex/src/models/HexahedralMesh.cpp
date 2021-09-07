#include "HexahedralMesh.h"

#include "HexMeshQuality.h"
#include "serialization/Serializer.h"

namespace hex {
HexahedralMesh::HexahedralMesh(const std::string& file_name) {
  auto mesh = Serializer::LoadHexMesh(file_name);
  vertices_ = mesh->GetVertices();
  hexes_ = mesh->GetHexes();
}

HexahedralMesh::HexahedralMesh(std::vector<Vector3f> vertices,
                               std::vector<Hex> hexes)
    : vertices_{std::move(vertices)}, hexes_{std::move(hexes)} {}

const std::vector<Vector3f>& HexahedralMesh::GetVertices() const {
  return vertices_;
}

const std::vector<HexahedralMesh::Hex>& HexahedralMesh::GetHexes() const {
  return hexes_;
}

const HexMeshQuality& HexahedralMesh::GetMeshQuality() const {
  if (!mesh_quality_) {
    mesh_quality_ = std::make_unique<HexMeshQuality>(*this);
  }
  return *mesh_quality_;
}
}  // namespace hex
