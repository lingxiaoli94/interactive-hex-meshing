#include "TriMesh.h"
#include <Eigen/Core>

namespace geomlib {
TriMesh::TriMesh(const Eigen::MatrixXf& vertices, const Eigen::MatrixXi& faces)
    : vertices_{vertices}, faces_{faces} {}

const Eigen::MatrixXf& TriMesh::GetVertexNormals() const {
  if (!vertex_normals_) {
    CalculateVertexNormals();
  }
  return *vertex_normals_;
}

const Eigen::MatrixXf& TriMesh::GetFaceNormals() const {
  if (!face_normals_) {
    CalculateFaceNormals();
  }
  return *face_normals_;
}

const Eigen::VectorXf& TriMesh::GetFaceAreas() const {
  if (!face_areas_) {
    CalculateFaceAreas();
  }
  return *face_areas_;
}

const Eigen::MatrixXf& TriMesh::GetCotangentWeights() const {
  if (!cotangent_weights_) {
    CalculateCotangentWeights();
  }
  return *cotangent_weights_;
}

const Eigen::VectorXf& TriMesh::GetVertexAreas() const {
  if (!vertex_areas_) {
    CalculateVertexAreas();
  }
  return *vertex_areas_;
}

const Eigen::MatrixXi& TriMesh::GetAdjacentFacePairs() const {
  if (!adjacent_face_pairs_) {
    BuildAdjacentFacePairs();
  }
  return *adjacent_face_pairs_;
}

void TriMesh::CalculateVertexNormals() const {
  CalculateFaceNormals();
  vertex_normals_ = std::make_unique<Eigen::MatrixXf>(
      Eigen::MatrixXf::Zero(GetNumVertices(), 3));
  for (int i = 0; i < static_cast<int>(GetNumFaces()); i++) {
    int v1 = faces_(i, 0);
    int v2 = faces_(i, 1);
    int v3 = faces_(i, 2);
    Eigen::Vector3f n = face_normals_->row(i);
    vertex_normals_->row(v1) += n;
    vertex_normals_->row(v2) += n;
    vertex_normals_->row(v3) += n;
  }

  for (int i = 0; i < static_cast<int>(GetNumVertices()); i++) {
    vertex_normals_->row(i).normalize();
  }
}

void TriMesh::CalculateFaceNormals() const {
  face_normals_ = std::make_unique<Eigen::MatrixXf>(GetNumFaces(), 3);
  for (int i = 0; i < GetNumFaces(); i++) {
    int v1 = faces_(i, 0);
    int v2 = faces_(i, 1);
    int v3 = faces_(i, 2);
    Eigen::Vector3f p1 = vertices_.row(v1);
    Eigen::Vector3f p2 = vertices_.row(v2);
    Eigen::Vector3f p3 = vertices_.row(v3);
    face_normals_->row(i) = (p2 - p1).cross(p3 - p1);
  }
}

void TriMesh::CalculateFaceAreas() const {
  auto& face_normals = GetFaceNormals();
  face_areas_ = std::make_unique<Eigen::VectorXf>(GetNumFaces());
  for (int k = 0; k < GetNumFaces(); k++) {
    (*face_areas_)(k) = face_normals.row(k).norm() / 2;
  }
}

void TriMesh::CalculateCotangentWeights() const {
  Eigen::MatrixXf p1(faces_.rows(), 3);
  Eigen::MatrixXf p2(faces_.rows(), 3);
  Eigen::MatrixXf p3(faces_.rows(), 3);
  for (int i = 0; i < faces_.rows(); i++) {
    p1.row(i) = vertices_.row(faces_(i, 0));
    p2.row(i) = vertices_.row(faces_(i, 1));
    p3.row(i) = vertices_.row(faces_(i, 2));
  }

  Eigen::VectorXf l1 = (p2 - p3).rowwise().norm();
  Eigen::VectorXf l2 = (p1 - p3).rowwise().norm();
  Eigen::VectorXf l3 = (p1 - p2).rowwise().norm();
  Eigen::VectorXf s = (l1 + l2 + l3) / 2;
  Eigen::VectorXf r = (s - l1)
                          .cwiseProduct(s - l2)
                          .cwiseProduct(s - l3)
                          .cwiseQuotient(s)
                          .cwiseSqrt();

  auto fn = [&](const Eigen::VectorXf& l) -> Eigen::VectorXf {
    return ((s - l).cwiseAbs2() - r.cwiseAbs2())
        .cwiseQuotient(2 * (s - l).cwiseProduct(r));
  };

  cotangent_weights_ = std::make_unique<Eigen::MatrixXf>(GetNumFaces(), 3);
  cotangent_weights_->col(0) = fn(l1);
  cotangent_weights_->col(1) = fn(l2);
  cotangent_weights_->col(2) = fn(l3);
}

void TriMesh::CalculateVertexAreas() const {
  vertex_areas_ = std::make_unique<Eigen::VectorXf>(
      Eigen::VectorXf::Zero(GetNumVertices()));
  for (int k = 0; k < GetNumFaces(); k++) {
    Eigen::Vector3f n = face_normals_->row(k);
    float a = n.norm() / 6;
    for (int i = 0; i < 3; i++) {
      (*vertex_areas_)(faces_(k, i)) += a;
    }
  }
}

void TriMesh::BuildAdjacentFacePairs() const {
  std::vector<Vector2i> face_pairs;
  std::unordered_map<Vector2i, int, Vector2iHasher> edge_neighbor_;

  for (int i = 0; i < GetNumFaces(); i++) {
    for (int k = 0; k < 3; k++) {
      int u = faces_(i, k);
      int v = faces_(i, (k + 1) % 3);

      if (edge_neighbor_.count({v, u})) {
        face_pairs.emplace_back(i, edge_neighbor_[{v, u}]);
      } else {
        edge_neighbor_.emplace(Vector2i{u, v}, i);
      }
    }
  }

  adjacent_face_pairs_ =
      std::make_unique<Eigen::MatrixXi>(ArrayVector2iToMatrixXi(face_pairs));
}
}  // namespace geomlib
