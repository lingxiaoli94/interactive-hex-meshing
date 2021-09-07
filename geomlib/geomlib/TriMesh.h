#pragma once

#include "common.h"

namespace geomlib {
class TriMesh {
 public:
  TriMesh(const Eigen::MatrixXf& vertices, const Eigen::MatrixXi& faces);
  TriMesh(const TriMesh&) = delete;
  TriMesh& operator=(const TriMesh&) = delete;
  ~TriMesh() = default;

  const Eigen::MatrixXf& GetVertices() const { return vertices_; }
  size_t GetNumVertices() const { return vertices_.rows(); }
  const Eigen::MatrixXi& GetFaces() const { return faces_; }
  size_t GetNumFaces() const { return faces_.rows(); }

  const Eigen::MatrixXf& GetVertexNormals() const;
  const Eigen::MatrixXf& GetFaceNormals() const;
  const Eigen::VectorXf& GetFaceAreas() const;
  const Eigen::MatrixXf& GetCotangentWeights() const;
  const Eigen::VectorXf& GetVertexAreas() const;
  const Eigen::MatrixXi& GetAdjacentFacePairs() const;

 private:
  // Lazy initializations.
  void CalculateVertexNormals() const;
  void CalculateFaceNormals() const;
  void CalculateFaceAreas() const;
  void CalculateCotangentWeights() const;
  void CalculateVertexAreas() const;
  void BuildAdjacentFacePairs() const;

  Eigen::MatrixXf vertices_;
  Eigen::MatrixXi faces_;

  mutable std::unique_ptr<Eigen::MatrixXf> vertex_normals_;
  mutable std::unique_ptr<Eigen::MatrixXf> face_normals_;
  mutable std::unique_ptr<Eigen::VectorXf> face_areas_;
  mutable std::unique_ptr<Eigen::MatrixXf> cotangent_weights_;  // |F| x 3
  mutable std::unique_ptr<Eigen::VectorXf> vertex_areas_;
  mutable std::unique_ptr<Eigen::MatrixXi> adjacent_face_pairs_;
};
}  // namespace geomlib
