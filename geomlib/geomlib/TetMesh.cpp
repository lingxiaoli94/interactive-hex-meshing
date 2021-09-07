#include "TetMesh.h"

#include "logging.h"

namespace geomlib {
namespace {
// We assume p3 lies in the positive half space of CCW of (p0, p1, p2).
const std::vector<Vector3i> kFacesOfATet = {
    // Faces with orientation pointing outwards.
    {0, 2, 1},
    {0, 3, 2},
    {0, 1, 3},
    {1, 2, 3}};
}  // namespace

geomlib::TetMesh::TetMesh(const Eigen::MatrixXf& vertices,
                          const Eigen::MatrixXi& tets)
    : vertices_{vertices}, tets_{tets} {}

const TriMesh& TetMesh::GetSurfaceMesh() const {
  if (!surface_mesh_) {
    GenerateSurfaceMesh();
  }
  return *surface_mesh_;
}

std::shared_ptr<TriMesh> TetMesh::GetSurfaceMeshSharedPtr() const {
  if (!surface_mesh_) {
    GenerateSurfaceMesh();
  }
  return surface_mesh_;
}

const Eigen::VectorXf& TetMesh::GetTetVolumes() const {
  if (!tet_volumes_) {
    CalculateTetVolumes();
  }
  return *tet_volumes_;
}

const Eigen::MatrixXi& TetMesh::GetSurfaceTriangles() const {
  if (!surface_triangles_) {
    GenerateSurfaceTriangles();
  }
  return *surface_triangles_;
}

void TetMesh::FixTetsOrientation() {
  for (int i = 0; i < static_cast<int>(GetNumTets()); i++) {
    Vector3f p[4];
    for (int k = 0; k < 4; k++) {
      p[k] = vertices_.row(tets_(i, k));
    }
    Eigen::Matrix3f M;
    for (int k = 0; k < 3; k++) {
      M.row(k) = p[k + 1] - p[0];
    }
    if (M.determinant() < 0) {
      int tmp = tets_(i, 1);
      tets_(i, 1) = tets_(i, 2);
      tets_(i, 2) = tmp;
    }
  }
  InvalidateCache();
}

void TetMesh::InvalidateCache() {
  surface_mesh_.reset();
  tet_volumes_.reset();
  surface_triangles_.reset();
  surface_to_volume_vid_.clear();
  volume_to_surface_vid_.clear();
}

void TetMesh::GenerateSurfaceMesh() const {
  surface_to_volume_vid_.clear();
  volume_to_surface_vid_.clear();

  size_t num_tets = tets_.rows();
  std::unordered_set<Vector3i, Vector3iHasher> face_loop_set;
  for (int i = 0; i < static_cast<int>(num_tets); i++) {
    for (auto& f : kFacesOfATet) {
      face_loop_set.insert(
          Vector3i{tets_(i, f(0)), tets_(i, f(1)), tets_(i, f(2))});
    }
  }

  // Identify faces on the surface.
  std::vector<Vector3i> faces_on_surface;
  for (int i = 0; i < static_cast<int>(num_tets); i++) {
    for (auto& f : kFacesOfATet) {
      Vector3i face{tets_(i, f(0)), tets_(i, f(1)), tets_(i, f(2))};
      bool on_surface = true;
      for (int p = 0; p < 3; p++) {
        std::vector<int> cycled_corner_ids_;
        for (int k = 0; k < 3; k++) {
          cycled_corner_ids_.push_back(face((3 + p - k) % 3));
        }
        Vector3i cycled_vec = Eigen::Map<Vector3i>(cycled_corner_ids_.data());
        if (face_loop_set.count(cycled_vec)) {
          on_surface = false;
        }
      }

      if (on_surface) {
        faces_on_surface.push_back(face);
      }
    }
  }

  std::vector<Vector3f> vertices_of_surface;
  std::vector<Vector3i> faces_of_surface;
  int surface_vid_counter = 0;
  for (auto& face : faces_on_surface) {
    std::vector<int> face_of_surface;
    for (int k = 0; k < 3; k++) {
      int v = face(k);
      if (!volume_to_surface_vid_.count(v)) {
        volume_to_surface_vid_[v] = surface_vid_counter;
        surface_to_volume_vid_[surface_vid_counter] = v;
        vertices_of_surface.emplace_back(vertices_.row(v));
        surface_vid_counter++;
      }
      face_of_surface.push_back(volume_to_surface_vid_[v]);
    }
    faces_of_surface.emplace_back(Eigen::Map<Vector3i>(face_of_surface.data()));
  }

  LOGI(
      "Surface mesh with {} vertices and {} surfaces extracted from the tet "
      "mesh.",
      vertices_of_surface.size(), faces_of_surface.size());
  surface_mesh_ =
      std::make_shared<TriMesh>(ArrayVector3fToMatrixXf(vertices_of_surface),
                                ArrayVector3iToMatrixXi(faces_of_surface));
}

void TetMesh::CalculateTetVolumes() const {
  tet_volumes_ = std::make_unique<Eigen::VectorXf>(GetNumTets());

  for (int i = 0; i < static_cast<int>(GetNumTets()); i++) {
    Vector3f p[4];
    for (int k = 0; k < 4; k++) {
      p[k] = vertices_.row(tets_(i, k));
    }

    Eigen::Matrix3f M;
    for (int k = 0; k < 3; k++) {
      M.row(k) = p[k + 1] - p[0];
    }
    (*tet_volumes_)(i) = M.determinant() / 6;

    assert((*tet_volumes_)(i) > 0);
  }
}

void TetMesh::GenerateSurfaceTriangles() const {
  auto& faces = GetSurfaceMesh().GetFaces();
  surface_triangles_ = std::make_unique<Eigen::MatrixXi>(faces.rows(), 3);
  for (int i = 0; i < faces.rows(); i++) {
    Vector3i f;
    for (int k = 0; k < 3; k++) {
      f(k) = surface_to_volume_vid_.at(faces(i, k));
    }
    surface_triangles_->row(i) = f;
  }
}

Eigen::MatrixXf TetMesh::RetrievePointsFromBarycentricCoordinates(
    const Eigen::VectorXi& tet_ids,
    const Eigen::MatrixXf& barycentric_coords) const {
  assert(tet_ids.size() == barycentric_coords.rows());
  auto& vertices = GetVertices();
  auto& tets = GetTets();
  Eigen::MatrixXf points{tet_ids.size(), 3};
  for (int i = 0; i < tet_ids.size(); i++) {
    Vector3f v0 = vertices.row(tets(tet_ids[i], 0));
    Vector3f v1 = vertices.row(tets(tet_ids[i], 1));
    Vector3f v2 = vertices.row(tets(tet_ids[i], 2));
    Vector3f v3 = vertices.row(tets(tet_ids[i], 3));
    points.row(i) =
        barycentric_coords(i, 0) * v0 + barycentric_coords(i, 1) * v1 +
        barycentric_coords(i, 2) * v2 + barycentric_coords(i, 3) * v3;
  }
  return points;
}

}  // namespace geomlib
