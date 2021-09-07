#pragma once

#include "HexComplex.h"
#include "Polycube.h"

namespace hex {
class PolycubeGraph {
 public:
  PolycubeGraph(const Polycube& initial_polycube, float hex_size_,
                bool round_to_nearest);
  PolycubeGraph(const std::vector<Vector3i>& hexes, float hex_size);

  PolycubeGraph(const PolycubeGraph&) = delete;
  PolycubeGraph& operator=(const PolycubeGraph&) = delete;

  float GetHexSize() const { return hex_size_; }
  std::unique_ptr<QuadComplex> GenerateQuadComplex();
  std::unique_ptr<HexComplex> GenerateHexComplex(bool padding);
  static const std::vector<std::vector<size_t>>& GetFaceCornerIds();

  // Support for changing the discretized polycube.
  size_t GetHexIdFromQuadId(size_t quad_id) const;
  std::vector<size_t> GetHexesOnSamePatch(size_t quad_id) const;
  std::vector<Vector3i> GetExtrusionPatch(size_t quad_id) const;
  Vector3i GetExtrusionCoord(size_t quad_id) const;
  Vector3i GetHexCoord(size_t hex_id) const;
  std::vector<Vector3i> GetAllHexCoords() const;

 private:
  struct AABB {
    AABB(const Cuboid& cuboid);

    Vector3f cmin_f;
    Vector3f cmax_f;
    Vector3i cmin_i;
    Vector3i cmax_i;
    bool is_integral{false};
  };

  struct GridPoint {
    GridPoint(size_t id, const Vector3i& coord);
    GridPoint(const GridPoint&) = delete;
    GridPoint& operator=(const GridPoint&) = delete;
    size_t id;
    Vector3i coord;
  };

  struct GridHex;

  struct GridHexFace {
    GridHexFace(const GridHex& hex, const Vector3i& offset, size_t face_id);
    const GridHex& hex;
    Vector3i offset;
    int patch_id{-1};
    size_t face_id;
    bool operator==(const GridHexFace& other) const;
  };

  struct GridHex {
    GridHex(size_t id, const Vector3i& coord);
    GridHex(const GridHex& other) = delete;
    GridHex& operator=(const GridHex& other) = delete;
    size_t id;
    Vector3i coord;
    std::vector<GridHexFace> faces;
  };

  struct Patch {
    std::vector<GridHexFace*> faces;
  };

  void Integralize(std::vector<AABB>& aabbs);
  void GenerateGrid(const std::vector<AABB>& aabbs);
  void GenerateGrid(const std::vector<Vector3i>& hexes);
  void GenerateGraph();
  Patch FloodfillPatch(GridHexFace& start_face, size_t patch_id);
  bool IsSurfaceFace(const GridHexFace& face);
  std::vector<GridPoint*> GetHexCorners(const GridHex& hex);
  void ExtractQuadComplex(
      std::vector<Vector4i>& complex_quads,
      std::vector<QuadComplex::Patch>& complex_patches,
      std::vector<const GridHexFace*>* quad_id_to_face =
          nullptr);  // vertex indices correspond to GridPoint

  float hex_size_;
  bool round_to_nearest_{true};

  std::unordered_map<Vector3i, GridPoint, Vector3iHasher> coord_to_point_;
  std::unordered_map<Vector3i, GridHex, Vector3iHasher> coord_to_hex_;

  // WARNING: don't store pointers to vector elements which may get invalidated
  // during resizing!
  std::vector<GridPoint*> id_to_point_;
  std::vector<GridHex*> id_to_hex_;

  std::vector<Patch> patches_;

  // Cache to go back from generated hex mesh to this graph.
  std::vector<const GridHexFace*> quad_id_to_face_;
};
}  // namespace hex
