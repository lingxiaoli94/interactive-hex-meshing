#include "TriangularMesh.h"

#include <geomlib/io/ObjParser.h>
#include <geomlib/ray_triangle_intersection.h>

#include <filesystem>

#include "logging.h"
#include "serialization/Serializer.h"

namespace hex {
TriangularMesh::TriangularMesh(std::shared_ptr<geomlib::TriMesh> tri_mesh) {
  tri_mesh_ = std::move(tri_mesh);
}

TriangularMesh::TriangularMesh(const std::string& mesh_obj_file)
    : TriangularMesh(geomlib::ObjParser::ParseToMesh(mesh_obj_file)) {}

TriangularMesh::TriangularMesh(const Eigen::MatrixXf& vertices,
                               const Eigen::MatrixXi& faces)
    : TriangularMesh(std::make_shared<geomlib::TriMesh>(vertices, faces)) {}

const Eigen::MatrixXf& TriangularMesh::GetVertices() const {
  return tri_mesh_->GetVertices();
}

const Eigen::MatrixXf& TriangularMesh::GetVertexNormals() const {
  return tri_mesh_->GetVertexNormals();
}

const Eigen::MatrixXf& TriangularMesh::GetFaceNormals() const {
  return tri_mesh_->GetFaceNormals();
}

const Eigen::MatrixXi& TriangularMesh::GetFaces() const {
  return tri_mesh_->GetFaces();
}

const Eigen::VectorXf& TriangularMesh::GetFaceAreas() const {
  return tri_mesh_->GetFaceAreas();
}

const Eigen::MatrixXi& TriangularMesh::GetAdjacentFacePairs() const {
  return tri_mesh_->GetAdjacentFacePairs();
}

std::shared_ptr<geomlib::TriMesh> TriangularMesh::GetMeshSharedPtr() const {
  return tri_mesh_;
}

bool TriangularMesh::IntersectWithRay(const Ray& ray, HitRecord& record) const {
  bool success = false;
  float min_t = std::numeric_limits<float>::max();
  auto& vertices = GetVertices();
  auto& faces = GetFaces();
#pragma omp parallel for
  for (size_t i = 0; i < faces.rows(); i++) {
    Vector3f v0 = vertices.row(faces(i, 0));
    Vector3f v1 = vertices.row(faces(i, 1));
    Vector3f v2 = vertices.row(faces(i, 2));

    float t;
    bool b = geomlib::RayTriangleIntersection(
        ray.GetOrigin(), ray.GetDirection(), v0, v1, v2, &t);

    if (b) {
#pragma omp critical
      if (t < min_t) {
        min_t = t;
        record.t = min_t;
      }
    }
  }

  success = min_t < std::numeric_limits<float>::max();
  return success;
}

}  // namespace hex
