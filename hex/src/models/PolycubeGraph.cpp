#include "PolycubeGraph.h"

#include <queue>

#include "logging.h"
#include "models/HexConvention.h"

namespace hex {
namespace {
const float kPaddingFraction = 0.2f;
}  // namespace

PolycubeGraph::PolycubeGraph(const Polycube& initial_polycube, float hex_size,
                             bool round_to_nearest)
    : hex_size_{hex_size}, round_to_nearest_{round_to_nearest} {
  std::vector<AABB> aabbs;
  for (size_t i = 0; i < initial_polycube.GetCuboidCount(); i++) {
    const auto& cuboid = initial_polycube.GetCuboid(i);
    aabbs.emplace_back(cuboid);
  }
  Integralize(aabbs);
  GenerateGrid(aabbs);
  GenerateGraph();
}

PolycubeGraph::PolycubeGraph(const std::vector<Vector3i>& hexes, float hex_size)
    : hex_size_{hex_size} {
  GenerateGrid(hexes);
  GenerateGraph();
}

void PolycubeGraph::Integralize(std::vector<AABB>& aabbs) {
  for (auto& aabb : aabbs) {
    if (round_to_nearest_) {
      aabb.cmin_i = (aabb.cmin_f / hex_size_).array().round().cast<int>();
      aabb.cmax_i = (aabb.cmax_f / hex_size_).array().round().cast<int>();
    } else {
      aabb.cmin_i = (aabb.cmin_f / hex_size_).array().floor().cast<int>();
      aabb.cmax_i = (aabb.cmax_f / hex_size_).array().ceil().cast<int>();
    }
    aabb.is_integral = true;
  }
}

void PolycubeGraph::GenerateGraph() {
  patches_.clear();
  for (auto& kv : coord_to_hex_) {
    auto& hex = kv.second;
    for (size_t d = 0; d < 6; d++) {
      auto& face = hex.faces[d];
      if (face.patch_id == -1 && IsSurfaceFace(face)) {
        patches_.push_back(FloodfillPatch(face, patches_.size()));
      }
    }
  }
}

void PolycubeGraph::GenerateGrid(const std::vector<AABB>& aabbs) {
  coord_to_point_.clear();
  id_to_point_.clear();

  for (auto& aabb : aabbs) {
    assert(aabb.is_integral);
    for (int z = aabb.cmin_i[2]; z <= aabb.cmax_i[2]; z++)
      for (int y = aabb.cmin_i[1]; y <= aabb.cmax_i[1]; y++)
        for (int x = aabb.cmin_i[0]; x <= aabb.cmax_i[0]; x++) {
          Vector3i coord{x, y, z};
          if (coord_to_point_.count(coord) == 0) {
            auto itr =
                coord_to_point_.try_emplace(coord, id_to_point_.size(), coord);
            id_to_point_.push_back(&itr.first->second);
          }
        }
  }

  coord_to_hex_.clear();
  id_to_hex_.clear();
  for (auto& aabb : aabbs) {
    assert(aabb.is_integral);
    for (int z = aabb.cmin_i[2]; z < aabb.cmax_i[2]; z++)
      for (int y = aabb.cmin_i[1]; y < aabb.cmax_i[1]; y++)
        for (int x = aabb.cmin_i[0]; x < aabb.cmax_i[0]; x++) {
          Vector3i coord{x, y, z};
          if (coord_to_hex_.count(coord) == 0) {
            auto itr =
                coord_to_hex_.try_emplace(coord, id_to_hex_.size(), coord);
            id_to_hex_.push_back(&itr.first->second);
          }
        }
  }
}

void PolycubeGraph::GenerateGrid(const std::vector<Vector3i>& hexes) {
  coord_to_point_.clear();
  id_to_point_.clear();

  for (auto& hex : hexes) {
    for (int dz = 0; dz <= 1; dz++)
      for (int dy = 0; dy <= 1; dy++)
        for (int dx = 0; dx <= 1; dx++) {
          Vector3i coord = hex + Vector3i{dx, dy, dz};
          if (coord_to_point_.count(coord) == 0) {
            auto itr =
                coord_to_point_.try_emplace(coord, id_to_point_.size(), coord);
            id_to_point_.push_back(&itr.first->second);
          }
        }
  }

  coord_to_hex_.clear();
  id_to_hex_.clear();
  for (auto& hex : hexes) {
    auto itr = coord_to_hex_.try_emplace(hex, id_to_hex_.size(), hex);
    id_to_hex_.push_back(&itr.first->second);
  }
}

PolycubeGraph::Patch PolycubeGraph::FloodfillPatch(
    PolycubeGraph::GridHexFace& start_face, size_t patch_id) {
  Patch patch;

  std::queue<GridHexFace*> queue;
  queue.push(&start_face);
  start_face.patch_id = static_cast<int>(patch_id);

  while (!queue.empty()) {
    GridHexFace* face = queue.front();
    queue.pop();
    patch.faces.push_back(face);

    auto& kFaceOffsets = HexConvention::GetFaceOffsets();
    auto& kFaceOffsetToId = HexConvention::GetFaceOffsetToId();
    for (size_t d = 0; d < 6; d++) {
      auto& offset = kFaceOffsets[d];
      if (offset == face->offset || offset == -face->offset) {
        // Expand only along non-normal directions.
        continue;
      }
      auto next_hex_coord = face->hex.coord + offset;
      auto next_hex_it = coord_to_hex_.find(next_hex_coord);
      if (next_hex_it != coord_to_hex_.end()) {
        auto& next_hex = next_hex_it->second;
        auto& next_face = next_hex.faces[kFaceOffsetToId.at(face->offset)];
        if (IsSurfaceFace(next_face) && next_face.patch_id == -1) {
          next_face.patch_id = static_cast<int>(patch_id);
          queue.push(&next_face);
        }
      }
    }
  }
  return patch;
}

bool PolycubeGraph::IsSurfaceFace(const GridHexFace& face) {
  return coord_to_hex_.count(face.hex.coord + face.offset) == 0;
}

size_t PolycubeGraph::GetHexIdFromQuadId(size_t quad_id) const {
  return quad_id_to_face_.at(quad_id)->hex.id;
}

std::vector<size_t> PolycubeGraph::GetHexesOnSamePatch(size_t quad_id) const {
  int patch_id = quad_id_to_face_.at(quad_id)->patch_id;
  assert(patch_id != -1);
  std::vector<size_t> result;
  for (auto face : patches_[patch_id].faces) {
    result.push_back(face->hex.id);
  }
  return result;
}

Vector3i PolycubeGraph::GetExtrusionCoord(size_t quad_id) const {
  auto face = quad_id_to_face_.at(quad_id);
  return face->hex.coord + face->offset;
}

std::vector<Vector3i> PolycubeGraph::GetExtrusionPatch(size_t quad_id) const {
  int patch_id = quad_id_to_face_.at(quad_id)->patch_id;
  assert(patch_id != -1);
  std::vector<Vector3i> result;
  for (auto face : patches_[patch_id].faces) {
    result.push_back(face->hex.coord + face->offset);
  }
  return result;
}

Vector3i PolycubeGraph::GetHexCoord(size_t hex_id) const {
  return id_to_hex_.at(hex_id)->coord;
}

std::vector<Vector3i> PolycubeGraph::GetAllHexCoords() const {
  std::vector<Vector3i> result;
  for (auto& hex : id_to_hex_) {
    result.push_back(hex->coord);
  }
  return result;
}

std::unique_ptr<QuadComplex> PolycubeGraph::GenerateQuadComplex() {
  // A faster version of GenerateHexComplex().GetQuadComplex().
  quad_id_to_face_.clear();

  std::vector<Vector4i> complex_quads;
  std::vector<QuadComplex::Patch> complex_patches;
  ExtractQuadComplex(complex_quads, complex_patches, &quad_id_to_face_);

  std::vector<size_t> surface_indices;  // indices of surface vertices
  std::unordered_map<size_t, size_t> to_surface_index;
  for (auto& quad : complex_quads) {
    for (size_t k = 0; k < 4; k++) {
      int v = quad[k];
      if (!to_surface_index.count(v)) {
        to_surface_index.emplace(v, surface_indices.size());
        surface_indices.push_back(v);
      }
    }
  }

  std::vector<Vector3f> surface_vertices;
  for (auto i : surface_indices) {
    surface_vertices.push_back(id_to_point_[i]->coord.cast<float>() *
                               hex_size_);
  }

  std::vector<Vector4i> surface_quads;
  for (auto& quad : complex_quads) {
    Vector4i surface_quad;
    for (size_t k = 0; k < 4; k++) {
      surface_quad[k] = static_cast<int>(to_surface_index.at(quad[k]));
    }
    surface_quads.push_back(surface_quad);
  }

  return std::make_unique<QuadComplex>(surface_vertices, surface_quads,
                                       complex_patches);
}

void PolycubeGraph::ExtractQuadComplex(
    std::vector<Vector4i>& complex_quads,
    std::vector<QuadComplex::Patch>& complex_patches,
    std::vector<const GridHexFace*>* quad_id_to_face) {
  auto& kFaceCornerIds = HexConvention::GetFaceCornerIds();
  for (auto& p : patches_) {
    QuadComplex::Patch complex_patch;
    auto& faces = p.faces;
    for (auto& face : faces) {
      auto& corners = kFaceCornerIds[face->face_id];
      auto hex_corners = GetHexCorners(face->hex);
      complex_patch.push_back(complex_quads.size());
      complex_quads.emplace_back(
          hex_corners[corners[0]]->id, hex_corners[corners[1]]->id,
          hex_corners[corners[2]]->id, hex_corners[corners[3]]->id);

      for (int k = 0; k < 4; k++) {
        assert(complex_quads.back()(k) < id_to_point_.size());
      }
      if (quad_id_to_face) {
        quad_id_to_face->push_back(face);
      }
    }
    complex_patches.push_back(complex_patch);
  }
}

std::unique_ptr<HexComplex> PolycubeGraph::GenerateHexComplex(bool padding) {
  std::vector<Vector3f> complex_vertices;
  for (auto& p : id_to_point_) {
    complex_vertices.push_back(p->coord.cast<float>() * hex_size_);
  }

  std::vector<Vector4i> complex_quads;
  std::vector<QuadComplex::Patch> complex_patches;
  ExtractQuadComplex(complex_quads, complex_patches, &quad_id_to_face_);

  std::vector<HexComplex::Hex> complex_hexes;
  for (auto& kv : coord_to_hex_) {
    GridHex& h = kv.second;
    auto corners = GetHexCorners(h);
    HexComplex::Hex hex;
    for (size_t k = 0; k < 8; k++) {
      hex.push_back(corners[k]->id);
    }
    complex_hexes.push_back(hex);
  }

  if (padding) {
    // Change vertices and hexes, but keep quads and patches the same.

    auto& kFaceCornerIds = HexConvention::GetFaceCornerIds();
    auto& kOppositeFaceCornerIds = HexConvention::GetOppositeFaceCornerIds();
    std::unordered_map<size_t, size_t> new_layer_dict;
    std::vector<size_t> new_layer_to_old_id;
    std::vector<Vector3f> vertex_normals;
    std::vector<bool> is_hex_on_border(complex_hexes.size(), false);
    size_t new_layer_count = 0;
    for (auto& f : quad_id_to_face_) {
      is_hex_on_border[f->hex.id] = true;
      auto& corners = kFaceCornerIds[f->face_id];
      std::vector<GridPoint*> hex_corners = GetHexCorners(f->hex);

      Vector3f normal;
      {
        Vector3f p0 = hex_corners[corners[0]]->coord.cast<float>();
        Vector3f p1 = hex_corners[corners[1]]->coord.cast<float>();
        Vector3f p2 = hex_corners[corners[2]]->coord.cast<float>();
        normal = (p1 - p0).cross(p2 - p0);
      }
      for (int k = 0; k < 4; k++) {
        auto id = hex_corners[corners[k]]->id;
        if (!new_layer_dict.count(id)) {
          new_layer_to_old_id.push_back(id);
          new_layer_dict.emplace(id, new_layer_count++);
          vertex_normals.emplace_back(normal);
        } else {
          vertex_normals[new_layer_dict[id]] += normal;
        }
      }
    }

    std::vector<Vector3f> new_layer_vertices;
    for (int i = 0; i < new_layer_count; i++) {
      vertex_normals[i].normalize();
      new_layer_vertices.emplace_back(complex_vertices[new_layer_to_old_id[i]] -
                                      kPaddingFraction * vertex_normals[i] *
                                          hex_size_);
    }
    std::vector<HexComplex::Hex> new_layer_hexes;
    for (auto& f : quad_id_to_face_) {
      // Each border face gives rise to two new hexes (and one deletion).
      auto& corners = kFaceCornerIds[f->face_id];
      auto& op_corners = kOppositeFaceCornerIds[f->face_id];
      std::vector<GridPoint*> hex_corners =
          GetHexCorners(f->hex);  // old corners

      size_t top_ids[4];
      size_t mid_ids[4];
      size_t bot_ids[4];
      for (int k = 0; k < 4; k++) {
        top_ids[k] = hex_corners[corners[k]]->id;
        mid_ids[k] = new_layer_dict[hex_corners[corners[k]]->id] +
                     complex_vertices.size();
        bot_ids[k] = hex_corners[op_corners[k]]->id;
      }
      new_layer_hexes.push_back({mid_ids[0], mid_ids[3], top_ids[0], top_ids[3],
                                 mid_ids[1], mid_ids[2], top_ids[1],
                                 top_ids[2]});
    }

    for (int i = 0; i < (int)complex_hexes.size(); i++) {
      for (int j = 0; j < 8; j++) {
        if (new_layer_dict.count(complex_hexes[i][j])) {
          complex_hexes[i][j] =
              new_layer_dict[complex_hexes[i][j]] + complex_vertices.size();
        }
      }
    }
    complex_hexes.insert(complex_hexes.end(), new_layer_hexes.begin(),
                         new_layer_hexes.end());

    complex_vertices.insert(complex_vertices.end(), new_layer_vertices.begin(),
                            new_layer_vertices.end());
  }


  LOGI(
      "Generate a hex complex with {} vertices, {} quads, {} patches, and {} "
      "hexes.",
      complex_vertices.size(), complex_quads.size(), complex_patches.size(),
      complex_hexes.size());
  return std::make_unique<HexComplex>(complex_vertices, complex_quads,
                                      complex_patches, complex_hexes);
}

std::vector<PolycubeGraph::GridPoint*> PolycubeGraph::GetHexCorners(
    const GridHex& hex) {
  std::vector<GridPoint*> result;
  for (int dz = 0; dz <= 1; dz++)
    for (int dy = 0; dy <= 1; dy++)
      for (int dx = 0; dx <= 1; dx++) {
        Vector3i coord = hex.coord + Vector3i{dx, dy, dz};
        auto it = coord_to_point_.find(coord);
        assert(it != coord_to_point_.end());
        result.push_back(&it->second);
      }
  return result;
}

PolycubeGraph::AABB::AABB(const Cuboid& cuboid)
    : cmin_f{cuboid.center - cuboid.halflengths},
      cmax_f{cuboid.center + cuboid.halflengths} {}

PolycubeGraph::GridHex::GridHex(size_t hex_id, const Vector3i& coord)
    : id{hex_id}, coord{coord} {
  auto& kFaceOffsets = HexConvention::GetFaceOffsets();
  for (size_t i = 0; i < 6; i++) {
    faces.emplace_back(*this, kFaceOffsets[i], i);
  }
}

PolycubeGraph::GridHexFace::GridHexFace(const PolycubeGraph::GridHex& hex,
                                        const Vector3i& offset, size_t face_id)
    : hex{hex}, offset{offset}, face_id{face_id} {}

bool PolycubeGraph::GridHexFace::operator==(
    const PolycubeGraph::GridHexFace& other) const {
  return hex.id == other.hex.id && offset == other.offset;
}

PolycubeGraph::GridPoint::GridPoint(size_t id, const Vector3i& coord)
    : id{id}, coord{coord} {}
}  // namespace hex
