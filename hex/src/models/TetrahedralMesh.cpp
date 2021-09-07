#include "TetrahedralMesh.h"

#include <geomlib/TriangularMeshSampler.h>
#include <geomlib/autograd/GeneralizedProjection.h>
#include <geomlib/io/TetMeshParser.h>
#include <geomlib/point_tet_mesh_test.h>
#include <vkoo/core/VertexObject.h>

#include <random>

#include "logging.h"
#include "optim/torch_utils.h"

namespace hex {
TetrahedralMesh::TetrahedralMesh(std::shared_ptr<geomlib::TetMesh> tet_mesh)
    : tet_mesh_{std::move(tet_mesh)} {}

TetrahedralMesh::TetrahedralMesh(const Eigen::MatrixXf& vertices,
                                 const Eigen::MatrixXi& tets)
    : TetrahedralMesh{std::make_shared<geomlib::TetMesh>(vertices, tets)} {}

TetrahedralMesh::TetrahedralMesh(const std::string& file_path)
    : TetrahedralMesh(geomlib::TetMeshParser::Parse(file_path)) {}

const Eigen::MatrixXf& TetrahedralMesh::GetVertices() const {
  return tet_mesh_->GetVertices();
}

const Eigen::MatrixXi& TetrahedralMesh::GetTets() const {
  return tet_mesh_->GetTets();
}

const Eigen::VectorXf& TetrahedralMesh::GetTetVolumes() const {
  return tet_mesh_->GetTetVolumes();
}

const Eigen::MatrixXi& TetrahedralMesh::GetSurfaceTriangles() const {
  return tet_mesh_->GetSurfaceTriangles();
}

const Eigen::MatrixXf& TetrahedralMesh::GetAnchors() const {
  if (!anchors_) {
    LOGI("Creating anchors with default parameters...");
    CreateAnchors(32, true, 0.0f, 4000, 0.01f);  // default set of parameters
  }
  return *anchors_;
}

const Eigen::VectorXf& TetrahedralMesh::GetDistanceField() const {
  if (!distance_field_) {
    CreateDistanceField();
  }
  return *distance_field_;
}

bool TetrahedralMesh::HasAnchors() const { return anchors_ != nullptr; }

bool TetrahedralMesh::HasDistanceField() const {
  return distance_field_ != nullptr;
}

void TetrahedralMesh::CreateAnchors(int grid_size, bool inside_only,
                                    float bbox_padding, int surface_samples,
                                    float perturbation) const {
  Eigen::Vector3f bbox_min = GetVertices().colwise().minCoeff();
  Eigen::Vector3f bbox_max = GetVertices().colwise().maxCoeff();
  Eigen::Vector3f side_length = bbox_max - bbox_min;

  bbox_max += bbox_padding * side_length;
  bbox_min -= bbox_padding * side_length;

  Eigen::Vector3f bbox_h = (bbox_max - bbox_min) / grid_size;

  int num_voxels = grid_size * grid_size * grid_size;

  Eigen::MatrixXf uniform_anchors = Eigen::MatrixXf::Zero(num_voxels, 3);
  int count = 0;

  for (int k = 0; k < grid_size; k++)
    for (int j = 0; j < grid_size; j++)
      for (int i = 0; i < grid_size; i++) {
        uniform_anchors.row(count) =
            bbox_min + Eigen::Vector3f(static_cast<float>(i),
                                       static_cast<float>(j),
                                       static_cast<float>(k))
                           .cwiseProduct(bbox_h);
        count++;
      }

  if (inside_only) {
    using namespace torch::indexing;
    auto uniform_anchors_gpu = MatrixXfToTensor(uniform_anchors).cuda();
    auto uniform_sdf_gpu = ComputeDistanceFieldGPU(uniform_anchors_gpu);
    uniform_anchors = TensorToMatrixXf(
        uniform_anchors_gpu.index({uniform_sdf_gpu < 0, Slice()})
            .detach()
            .cpu());
  }

  auto& surface_vertices = GetSurfaceMesh().GetVertices();

  // Eigen::MatrixXf surface_anchors{surface_vertices.rows() * surface_multiple,
  //                                3};
  Eigen::MatrixXf surface_anchors;
  {
    auto vertices = MatrixXfToTensor(GetSurfaceMesh().GetVertices()).cuda();
    auto faces =
        MatrixXiToTensor(GetSurfaceMesh().GetFaces()).cuda().to(torch::kInt64);
    geomlib::TriangularMeshSampler sampler{vertices, faces};
    auto samples = sampler.Sample(surface_samples);
    surface_anchors = TensorToMatrixXf(samples.detach().cpu());
  }
  std::default_random_engine gen(42);
  std::normal_distribution<float> dis(0.0f, 1.0f);
  for (int i = 0; i < surface_anchors.rows(); i++) {
    Eigen::Vector3f delta{dis(gen), dis(gen), dis(gen)};
    Eigen::Vector3f r = surface_vertices.row(i % surface_vertices.rows());
    surface_anchors.row(i) = r + perturbation * delta;
  }

  anchors_ = std::make_unique<Eigen::MatrixXf>(
      uniform_anchors.rows() + surface_anchors.rows(), 3);
  *anchors_ << uniform_anchors, surface_anchors;

  LOGI("{} anchors have been created.", anchors_->rows());
}

void TetrahedralMesh::CreateDistanceField() const {
  auto anchors_gpu = MatrixXfToTensor(GetAnchors()).cuda();
  auto result_gpu = ComputeDistanceFieldGPU(anchors_gpu);

  distance_field_ = std::make_unique<Eigen::VectorXf>(
      TensorToVectorXf(result_gpu.detach().cpu()));
}

torch::Tensor TetrahedralMesh::ComputeDistanceFieldGPU(
    torch::Tensor points) const {
  points = points.contiguous();
  assert(points.is_cuda());
  auto& surface_mesh = GetSurfaceMesh();

  auto surface_vertices_gpu =
      MatrixXfToTensor(surface_mesh.GetVertices()).cuda().contiguous();
  auto surface_faces_gpu =
      MatrixXiToTensor(surface_mesh.GetFaces()).cuda().contiguous();

  auto proj_info = geomlib::TriangularProjectionInfo(surface_vertices_gpu,
                                                     surface_faces_gpu);
  auto proj_result =
      geomlib::GeneralizedTriangleProjection<3>::apply(points, proj_info);

  auto udf =
      proj_result[0].to(torch::kFloat32).sqrt();  // GeneralizedTriangleProjection
                                                  // returns squared distance.
  // At this point udf is unsigned.
  auto vertices_gpu = MatrixXfToTensor(GetVertices()).cuda().contiguous();
  auto tets_gpu = MatrixXiToTensor(GetTets()).cuda().contiguous();
  auto test_result = geomlib::PointTetMeshTest(points, vertices_gpu, tets_gpu)
                         .to(torch::kFloat32);
  return udf * test_result;
}

void TetrahedralMesh::SetAnchors(const Eigen::MatrixXf& anchors) {
  anchors_ = std::make_unique<Eigen::MatrixXf>(anchors);
}

void TetrahedralMesh::SetDistanceField(const Eigen::VectorXf& distance_field) {
  distance_field_ = std::make_unique<Eigen::VectorXf>(distance_field);
}

void TetrahedralMesh::FixTetsOrientation() {
  tet_mesh_->FixTetsOrientation();
  InvalidateCache();
}

void TetrahedralMesh::InvalidateCache() { surface_mesh_.reset(); }

Eigen::MatrixXf TetrahedralMesh::RetrievePointsFromBarycentricCoordinates(
    const Eigen::VectorXi& tet_ids,
    const Eigen::MatrixXf& barycentric_coords) const {
  return tet_mesh_->RetrievePointsFromBarycentricCoordinates(
      tet_ids, barycentric_coords);
}

const TriangularMesh& TetrahedralMesh::GetSurfaceMesh() const {
  if (!surface_mesh_) {
    surface_mesh_ =
        std::make_shared<TriangularMesh>(tet_mesh_->GetSurfaceMeshSharedPtr());
  }
  return *surface_mesh_;
}

std::vector<std::shared_ptr<vkoo::VertexObject>>
TetrahedralMesh::CreateAnchorVBOs(vkoo::Device& device) const {
  // Return two vbos, one for anchors inside the mesh and one for outside
  // anchors.
  auto& anchors = GetAnchors();
  auto& sdf = GetDistanceField();

  int num_anchors = anchors.rows();
  std::vector<std::vector<glm::vec3>> positions(2);
  for (int i = 0; i < num_anchors; i++) {
    positions[sdf[i] <= 0 ? 0 : 1].push_back(ToGlm(anchors.row(i)));
  }

  std::vector<std::shared_ptr<vkoo::VertexObject>> vbos;
  for (int i = 0; i < 2; i++) {
    auto vbo = std::make_shared<vkoo::VertexObject>(device);
    vbo->Update("position", positions[i]);
    vbo->Update("normal", positions[i]);
    std::vector<uint32_t> indices(positions[i].size());
    for (int k = 0; k < positions[i].size(); k++) {
      indices[k] = k;
    }
    vbo->UpdateIndices(indices);
    vbos.push_back(vbo);
  }
  return vbos;
}

}  // namespace hex

