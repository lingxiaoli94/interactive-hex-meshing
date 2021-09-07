#pragma once

#include "common.h"

#include <geomlib/TriMesh.h>
#include <vkoo/core/VertexObject.h>

#include "Ray.h"

namespace hex {
class TriangularMesh {
 public:
  TriangularMesh(std::shared_ptr<geomlib::TriMesh> tri_mesh);
  TriangularMesh(const std::string& mesh_obj_file);
  TriangularMesh(const Eigen::MatrixXf& vertices, const Eigen::MatrixXi& faces);

  TriangularMesh(const TriangularMesh&) = delete;
  TriangularMesh& operator=(const TriangularMesh&) = delete;

  // Wrapper functions of geomlib::TriMesh.
  const Eigen::MatrixXf& GetVertices() const;
  const Eigen::MatrixXf& GetVertexNormals() const;
  const Eigen::MatrixXf& GetFaceNormals() const;
  const Eigen::MatrixXi& GetFaces() const;
  const Eigen::VectorXf& GetFaceAreas() const;
  const Eigen::MatrixXi& GetAdjacentFacePairs() const;

  std::shared_ptr<geomlib::TriMesh> GetMeshSharedPtr() const;

  struct HitRecord {
    float t;
  };
  bool IntersectWithRay(const Ray& ray, HitRecord& record) const;

 private:
  std::shared_ptr<geomlib::TriMesh> tri_mesh_;

  mutable std::unique_ptr<Eigen::MatrixXf> embedding_;
};
}  // namespace hex
