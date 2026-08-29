#pragma once

// No vkoo includes: FilteredHexMesh works on HexahedralMesh/QuadrilateralMesh
// only, and the Device/Material/Node includes that used to be here were unused.
// Keeping this header Vulkan-free is what lets HexahedralizationStage.h hold a
// FilteringSetting in the HEX_ENABLE_VULKAN=OFF build. The .cpp is still
// renderer-side and is excluded there.
#include "models/HexahedralMesh.h"
#include "models/QuadrilateralMesh.h"

namespace hex {
struct FilteringSetting {
  bool operator!=(const FilteringSetting& other);
  float slice_dist{0.0f};
  Vector3f slice_normal{0.0f, 1.0f, 0.0f};
  HexQualityType quality{HexQualityType::ScaledJacobian};
  float quality_cutoff{1.0f};  // show hexes <= this cutoff
};

class FilteredHexMesh {
 public:
  FilteredHexMesh(const HexahedralMesh& hex_mesh);

  void Filter(const FilteringSetting& setting);

  struct SurfaceExtractionResult {
    std::unique_ptr<QuadrilateralMesh> quad_mesh;
    // Correspondence will be needed in editing boundary vertices.
    std::vector<size_t> vtx_id_quad_to_hex;
    // Two patches: interior (=0) and boundary (=1) quads.
    std::vector<std::vector<size_t>> patches;
  };
  SurfaceExtractionResult ExtractFilteredSurfaceMesh(
      bool need_patches = false) const;

 private:
  void ClearFilters();
  void FilterByPlane(const Vector3f& normal, float d);
  void FilterByQuality(HexQualityType quality, float d);
  void FilterByScaledJacobian(float d);
  void FilterByJacobian(float d);

  const std::vector<Vector3f>& vertices_;
  const std::vector<HexahedralMesh::Hex>& hexes_;
  const HexMeshQuality& mesh_quality_;
  std::vector<bool> hex_visibilities_;
  std::unordered_set<Vector4i, Vector4iHasher> boundary_quads_set_;
};
}  // namespace hex
