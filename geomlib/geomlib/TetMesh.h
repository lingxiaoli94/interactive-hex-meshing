#pragma once

#include "TriMesh.h"

namespace geomlib {
class TetMesh {
 public:
  TetMesh(const Eigen::MatrixXf& vertices, const Eigen::MatrixXi& tets);
  TetMesh(const TetMesh&) = delete;
  TetMesh& operator=(const TetMesh&) = delete;
  ~TetMesh() = default;

  const Eigen::MatrixXf& GetVertices() const { return vertices_; }
  size_t GetNumVertices() const { return vertices_.rows(); }
  const Eigen::MatrixXi& GetTets() const { return tets_; }
  size_t GetNumTets() const { return tets_.rows(); }

  const TriMesh& GetSurfaceMesh() const;
  std::shared_ptr<TriMesh> GetSurfaceMeshSharedPtr() const;

  const Eigen::VectorXf& GetTetVolumes() const;
  const Eigen::MatrixXi& GetSurfaceTriangles() const;


  Eigen::MatrixXf RetrievePointsFromBarycentricCoordinates(
      const Eigen::VectorXi& tet_ids,
      const Eigen::MatrixXf& barycentric_coords) const;

  void FixTetsOrientation();
  void InvalidateCache();

 private:
  void GenerateSurfaceMesh() const;
  void CalculateTetVolumes() const;
  void GenerateSurfaceTriangles() const;

  Eigen::MatrixXf vertices_;
  Eigen::MatrixXi tets_;

  // Surface mesh is made shared_ptr so it can be passed to other owners.
  mutable std::shared_ptr<TriMesh> surface_mesh_;

  mutable std::unique_ptr<Eigen::VectorXf> tet_volumes_;
  mutable std::unique_ptr<Eigen::MatrixXi> surface_triangles_;

  // Maps that go between surface and volume indices.
  mutable std::unordered_map<int, int> surface_to_volume_vid_;
  mutable std::unordered_map<int, int> volume_to_surface_vid_;
};
}  // namespace geomlib
