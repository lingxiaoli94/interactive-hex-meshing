#pragma once

#include "common.h"

#include <geomlib/TetMesh.h>
#include <torch/torch.h>

#include "TriangularMesh.h"

namespace hex {
class TetrahedralMesh {
 public:
  TetrahedralMesh(std::shared_ptr<geomlib::TetMesh> tet_mesh);
  TetrahedralMesh(const std::string& file_path);
  TetrahedralMesh(const Eigen::MatrixXf& vertices, const Eigen::MatrixXi& tets);

  TetrahedralMesh(const TetrahedralMesh&) = delete;
  TetrahedralMesh& operator=(const TetrahedralMesh&) = delete;

  // Wrapper functions of geomlib::TetMesh.
  const Eigen::MatrixXf& GetVertices() const;
  const Eigen::MatrixXi& GetTets() const;
  const Eigen::VectorXf& GetTetVolumes() const;
  const Eigen::MatrixXi& GetSurfaceTriangles() const;
  Eigen::MatrixXf RetrievePointsFromBarycentricCoordinates(
      const Eigen::VectorXi& tet_ids,
      const Eigen::MatrixXf& barycentric_coords) const;

  const TriangularMesh& GetSurfaceMesh() const;

  // Lazy initialization getters and setters.
  const Eigen::MatrixXf& GetAnchors() const;
  const Eigen::VectorXf& GetDistanceField() const;
  bool HasAnchors() const;
  bool HasDistanceField() const;
  void CreateAnchors(int grid_size, bool inside_only, float bbox_padding,
                     int surface_samples, float perturbation) const;
  void CreateDistanceField() const;
  void SetAnchors(const Eigen::MatrixXf& anchors);
  void SetDistanceField(const Eigen::VectorXf& DistanceField);

  void FixTetsOrientation();
  void InvalidateCache();
  torch::Tensor ComputeDistanceFieldGPU(torch::Tensor points) const;

  std::vector<std::shared_ptr<vkoo::VertexObject>> CreateAnchorVBOs(
      vkoo::Device& device) const;

 private:
  std::shared_ptr<geomlib::TetMesh> tet_mesh_;

  mutable std::unique_ptr<Eigen::MatrixXf> anchors_;
  // Distance field is computed with respect to anchors.
  mutable std::unique_ptr<Eigen::VectorXf> distance_field_;  // squared, signed
  mutable std::shared_ptr<TriangularMesh> surface_mesh_;
};
}  // namespace hex
