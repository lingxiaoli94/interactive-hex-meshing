#include "TriSurfaceView.h"

#include <vkoo/core/VertexObject.h>
#include <vkoo/st/components/Mesh.h>

#include "logging.h"

namespace hex {
namespace {
const float kWireframeOffset = 1e-4f;
}
TriSurfaceView::TriSurfaceView(vkoo::Device& device,
                                 vkoo::st::Node& parent_node,
                                 const Options& options)
    : device_{device}, options_{options} {
  opaque_material_.colors["diffuse_color"] = options.opaque_color;
  wireframe_material_.colors["diffuse_color"] = options.wireframe_color;
  transparent_material_.colors["diffuse_color"] = options.transparent_color;

  auto wrapper_node = std::make_unique<vkoo::st::Node>();
  wrapper_node_ = wrapper_node.get();
  parent_node.AddChild(std::move(wrapper_node));
}

vkoo::st::Node* TriSurfaceView::GetWrapperNode() const { return wrapper_node_; }

bool TriSurfaceView::IsEmpty() const {
  return wrapper_node_->GetChildrenCount() == 0;
}

void TriSurfaceView::Update(const TriangularMesh& tri_mesh) {
  wrapper_node_->RemoveAllChildren();

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> indices;

  Eigen::MatrixXf face_normals =
      tri_mesh.GetFaceNormals().rowwise().normalized();
  auto& vertices = tri_mesh.GetVertices();
  auto& faces = tri_mesh.GetFaces();
  for (int i = 0; i < faces.rows(); i++) {
    for (int k = 0; k < 3; k++) {
      Vector3f p = vertices.row(faces(i, k));
      positions.push_back(ToGlm(p));
      normals.push_back(ToGlm(Vector3f{face_normals.row(i)}));
      indices.push_back(static_cast<int>(positions.size()) - 1);
    }
  }

  {
    auto vertex_object = std::make_shared<vkoo::VertexObject>(device_);
    vertex_object->Update("position", positions);
    vertex_object->Update("normal", normals);
    vertex_object->UpdateIndices(indices);

    auto node = std::make_unique<vkoo::st::Node>();
    auto& mesh =
        node->CreateComponent<vkoo::st::Mesh>(vertex_object, indices.size());
    if (options_.transparent) {
      mesh.SetTransparent(true);
      mesh.SetMaterial(transparent_material_);
    } else {
      mesh.SetMaterial(opaque_material_);
    }

    surface_mesh_ = &mesh;
    wrapper_node_->AddChild(std::move(node));
  }
  if (options_.wireframe) {
    // For wireframe, offset a little bit.
    for (size_t i = 0; i < positions.size(); i++) {
      positions[i] += kWireframeOffset * normals[i];
    }
    auto vertex_object = std::make_shared<vkoo::VertexObject>(device_);
    vertex_object->Update("position", positions);
    vertex_object->Update("normal", normals);
    vertex_object->UpdateIndices(indices);
    auto node = std::make_unique<vkoo::st::Node>();
    auto& mesh =
        node->CreateComponent<vkoo::st::Mesh>(vertex_object, indices.size());
    mesh.SetMaterial(wireframe_material_);
    if (options_.transparent) {
      mesh.SetTransparent(true);
    }
    mesh.SetPolygonMode(vkoo::st::PolygonMode::Line);
    mesh.SetLineWidth(1.5f);

    wireframe_mesh_ = &mesh;
    wrapper_node_->AddChild(std::move(node));
  }
  LOGI("Finish updating tri surface view...");
}

void TriSurfaceView::SetTransparency(bool transparent) {
  surface_mesh_->SetTransparent(transparent);
  surface_mesh_->SetMaterial(transparent ? transparent_material_
                                         : opaque_material_);
  if (options_.wireframe) {
    assert(wireframe_mesh_);
    wireframe_mesh_->SetTransparent(transparent);
  }
}

void TriSurfaceView::SetWireframeVisible(bool visible) {
  assert(wireframe_mesh_);
  wireframe_mesh_->GetNode()->SetVisible(visible);
}
}  // namespace hex
