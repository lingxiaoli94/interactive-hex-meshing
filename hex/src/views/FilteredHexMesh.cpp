#include "views/FilteredHexMesh.h"

#include <vkoo/core/VertexObject.h>
#include <vkoo/st/components/Mesh.h>

#include "models/HexConvention.h"
#include "models/PolycubeGraph.h"

namespace hex {
namespace {
const float kEps = 1e-6f;
}

bool FilteringSetting::operator!=(const FilteringSetting& other) {
  return std::abs(slice_dist - other.slice_dist) > kEps ||
         std::abs(quality_cutoff - other.quality_cutoff) > kEps ||
         (slice_normal - other.slice_normal).norm() > kEps ||
         quality != other.quality;
}

FilteredHexMesh::FilteredHexMesh(const HexahedralMesh& hex_mesh)
    : vertices_{hex_mesh.GetVertices()},
      hexes_{hex_mesh.GetHexes()},
      mesh_quality_{hex_mesh.GetMeshQuality()} {
  ClearFilters();
  auto result = ExtractFilteredSurfaceMesh();
  for (auto& quad : result.quad_mesh->GetQuads()) {
    Vector4i quad_on_hex;
    for (int k = 0; k < 4; k++) {
      quad_on_hex(k) = result.vtx_id_quad_to_hex[quad[k]];
    }
    boundary_quads_set_.insert(quad_on_hex);
  }
}

void FilteredHexMesh::Filter(const FilteringSetting& setting) {
  ClearFilters();
  FilterByPlane(setting.slice_normal, setting.slice_dist);
  FilterByQuality(setting.quality, setting.quality_cutoff);
}

void FilteredHexMesh::FilterByQuality(HexQualityType quality, float d) {
  if (d >= 1.0f) {
    return;
  }
  switch (quality) {
    case HexQualityType::ScaledJacobian:
      FilterByScaledJacobian(d);
      break;
    case HexQualityType::Jacobian:
      FilterByJacobian(d);
      break;
  }
}

void FilteredHexMesh::ClearFilters() {
  hex_visibilities_.assign(hexes_.size(), true);
}

void FilteredHexMesh::FilterByPlane(const Vector3f& normal, float d) {
  if (d == 0.0f) {
    return;
  }
  float min_d = std::numeric_limits<float>::max();
  float max_d = std::numeric_limits<float>::min();

  for (auto& v : vertices_) {
    float dot = normal.dot(v);
    min_d = std::min(min_d, dot);
    max_d = std::max(max_d, dot);
  }

  float cutoff = min_d + (max_d - min_d) * std::clamp(d, 0.0f, 1.0f);

  // #pragma omp parallel for
  for (size_t i = 0; i < hexes_.size(); i++) {
    Vector3f center{0.0f, 0.0f, 0.0f};
    for (size_t k = 0; k < 8; k++) {
      center += vertices_[hexes_[i][k]];
    }
    center /= 8;
    hex_visibilities_[i] =
        hex_visibilities_[i] & (normal.dot(center) >= cutoff);
  }
}

void FilteredHexMesh::FilterByScaledJacobian(float d) {
  auto& scaled_jacobians = mesh_quality_.GetScaledJacobians();
  float cutoff = -1 + 2 * std::clamp(d, 0.0f, 1.0f);

  /* #pragma omp parallel for */
  for (int i = 0; i < (int)hexes_.size(); i++) {
    hex_visibilities_[i] =
        hex_visibilities_[i] & (scaled_jacobians(i) <= cutoff);
  }
}

void FilteredHexMesh::FilterByJacobian(float d) {
  auto& jacobians = mesh_quality_.GetJacobians();
  float q_min = jacobians.minCoeff();
  float q_max = jacobians.maxCoeff();
  float cutoff = q_min + d * (q_max - q_min);

  /* #pragma omp parallel for */
  for (int i = 0; i < (int)hexes_.size(); i++) {
    hex_visibilities_[i] = hex_visibilities_[i] & (jacobians(i) <= cutoff);
  }
}

FilteredHexMesh::SurfaceExtractionResult
FilteredHexMesh::ExtractFilteredSurfaceMesh(bool need_patches) const {
  auto& kFaceCornerIds = HexConvention::GetFaceCornerIds();
  // First collect all faces of visible hexes.
  std::vector<Vector4i> all_faces;
#pragma omp parallel for
  for (int i = 0; i < (int)hexes_.size(); i++) {
    if (hex_visibilities_[i]) {
      auto& hex = hexes_[i];
      for (size_t f = 0; f < 6; f++) {
        std::vector<int> corner_ids;  // must use int over size_t here!
        for (size_t k = 0; k < 4; k++) {
          corner_ids.push_back(hex[kFaceCornerIds[f][k]]);
        }
        Vector4i corners_id_vec = Eigen::Map<Vector4i>(corner_ids.data());
#pragma omp critical
        all_faces.push_back(corners_id_vec);
      }
    }
  }

  // Quads in the interior must be met by exactly two hexes.
  std::unordered_set<Vector4i, Vector4iHasher> face_loop_set(all_faces.begin(),
                                                             all_faces.end());

  std::unordered_map<size_t, size_t> vtx_id_hex_to_quad;
  std::vector<size_t> vtx_id_quad_to_hex;
  std::vector<Vector3f> surface_vertices;
  std::vector<Vector4i> surface_quads;
  std::vector<std::vector<size_t>> patches(2);
// Loop over all faces once again.
#pragma omp parallel for
  for (int i = 0; i < (int)hexes_.size(); i++)
    if (hex_visibilities_[i]) {
      auto& hex = hexes_[i];
      for (size_t f = 0; f < 6; f++) {
        Vector4i vec;
        for (size_t k = 0; k < 4; k++) {
          vec(k) = hex[kFaceCornerIds[f][k]];
        }

        bool on_surface = true;
        bool on_boundary = false;
        for (size_t p = 0; p < 4; p++) {
          // Opposing face's corner ids will be reversed.
          Vector4i cycled_vec;
          for (size_t k = 0; k < 4; k++) {
            cycled_vec(k) = vec[(4 + p - k) % 4];
          }
          if (face_loop_set.count(cycled_vec)) {
            on_surface = false;
          }
          if (boundary_quads_set_.count(vec)) {
            on_boundary = true;
          }
        }

        if (on_surface) {
#pragma omp critical
          {
            Vector4i quad;
            for (size_t k = 0; k < 4; k++) {
              size_t v = static_cast<size_t>(vec[k]);
              if (vtx_id_hex_to_quad.count(v) == 0) {
                vtx_id_hex_to_quad[v] = vtx_id_quad_to_hex.size();
                vtx_id_quad_to_hex.push_back(v);
                surface_vertices.push_back(vertices_[v]);
              }
              quad[k] = vtx_id_hex_to_quad[v];
            }
            if (need_patches) {
              patches[on_boundary ? 1 : 0].push_back(surface_quads.size());
            }
            surface_quads.push_back(quad);
          }
        }
      }
    }
  return {std::make_unique<QuadrilateralMesh>(surface_vertices, surface_quads),
          vtx_id_quad_to_hex, patches};
}

}  // namespace hex
