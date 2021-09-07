#include "CuboidNode.h"

#include <vkoo/core/PrimitiveFactory.h>
#include <vkoo/st/components/Mesh.h>
#include <vkoo/st/components/Tracing.h>
#include <vkoo/st/hittables/AABB.h>

#include "models/HexConvention.h"

namespace hex {
namespace {
// TODO: refactor this class to use dictionary.
std::shared_ptr<vkoo::VertexObject> cuboid_vertex_object_ = nullptr;
std::shared_ptr<vkoo::VertexObject> wireframe_vertex_object_ = nullptr;
std::unique_ptr<vkoo::st::Material> default_material_ = nullptr;
std::unique_ptr<vkoo::st::Material> focused_material_ = nullptr;
std::unique_ptr<vkoo::st::Material> locked_material_ = nullptr;
std::unique_ptr<vkoo::st::Material> subtract_material_ = nullptr;
std::unique_ptr<vkoo::st::Material> focused_wf_material_ = nullptr;
std::unique_ptr<vkoo::st::Material> normal_wf_material_ = nullptr;
}  // namespace

std::shared_ptr<vkoo::VertexObject> CuboidNode::CreateWireframeVbo(
    vkoo::Device& device) {
  auto& corners = HexConvention::GetCorners();
  auto& face_corners = HexConvention::GetFaceCornerIds();

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> indices;

  for (auto& f : face_corners) {
    Vector3f v[4];
    for (int k = 0; k < 4; k++) {
      v[k] = 2.0f * corners[f[k]].cast<float>().array() - 1.0f;
    }

    Vector3f n = (v[1] - v[0]).cross(v[2] - v[0]).normalized();
    for (size_t k = 0; k < 4; k++) {
      positions.push_back(ToGlm(v[k]));
      normals.push_back(ToGlm(n));
    }
    uint32_t j = static_cast<uint32_t>(positions.size() - 4);
    indices.insert(indices.end(),
                   {j, j + 1, j + 1, j + 2, j + 2, j + 3, j + 3, j});
  }
  auto vertex_object = std::make_shared<vkoo::VertexObject>(device);
  vertex_object->Update("position", positions);
  vertex_object->Update("normal", normals);
  vertex_object->UpdateIndices(indices);
  return vertex_object;
}

CuboidNode::CuboidNode(vkoo::Device& device) {
  if (cuboid_vertex_object_ == nullptr) {
    cuboid_vertex_object_ = vkoo::PrimitiveFactory::CreateCube(device);
  }
  if (wireframe_vertex_object_ == nullptr) {
    wireframe_vertex_object_ = CreateWireframeVbo(device);
  }
  if (default_material_ == nullptr) {
    default_material_ = std::make_unique<vkoo::st::Material>();
    default_material_->colors["diffuse_color"] =
        glm::vec4{0.7f, 0.9f, 0.1f, 1.0f};
  }
  if (focused_material_ == nullptr) {
    focused_material_ = std::make_unique<vkoo::st::Material>();
    focused_material_->colors["diffuse_color"] =
        glm::vec4{1.0f, 0.4f, 0.7f, 1.0f};
  }
  if (locked_material_ == nullptr) {
    locked_material_ = std::make_unique<vkoo::st::Material>();
    locked_material_->colors["diffuse_color"] =
        glm::vec4{0.7f, 0.6f, 0.7f, 1.0f};
  }
  if (subtract_material_ == nullptr) {
    subtract_material_ = std::make_unique<vkoo::st::Material>();
    subtract_material_->colors["diffuse_color"] =
        glm::vec4{0.1f, 0.9f, 0.2f, 1.0f};
  }
  if (focused_wf_material_ == nullptr) {
    focused_wf_material_ = std::make_unique<vkoo::st::Material>();
    focused_wf_material_->colors["diffuse_color"] =
        glm::vec4{0.8f, 0.2f, 0.1f, 0.8f};
  }
  if (normal_wf_material_ == nullptr) {
    normal_wf_material_ = std::make_unique<vkoo::st::Material>();
    normal_wf_material_->colors["diffuse_color"] =
        glm::vec4{0.4f, 0.4f, 0.4f, 1.0f};
  }

  // A cuboid node puts the actual mesh on a child node.
  auto mesh_child = std::make_unique<vkoo::st::Node>();
  mesh_child_node_ = mesh_child.get();
  AddChild(std::move(mesh_child));

  auto wireframe_child = std::make_unique<vkoo::st::Node>();
  wireframe_child_node_ = wireframe_child.get();
  AddChild(std::move(wireframe_child));

  mesh_child_node_->CreateComponent<vkoo::st::Mesh>(
      cuboid_vertex_object_, cuboid_vertex_object_->GetIndexCount());
  wireframe_child_node_->CreateComponent<vkoo::st::Mesh>(
      wireframe_vertex_object_, wireframe_vertex_object_->GetIndexCount());
  UpdateMode(mode_);

  // Tracing component is attached at the cuboid node so that GlobalScript can
  // locate it.
  CreateComponent<vkoo::st::Tracing>(std::make_shared<vkoo::st::AABB>(
      vkoo::st::AABB::FromCuboid(glm::vec3{0.0f}, glm::vec3{1.0f})));
}

void CuboidNode::UpdateView(const Cuboid& cuboid) {
  GetTransform().SetPosition(ToGlm(cuboid.center));
  mesh_child_node_->GetTransform().SetScale(ToGlm(cuboid.halflengths));
  wireframe_child_node_->GetTransform().SetScale(ToGlm(cuboid.halflengths));
  CreateComponent<vkoo::st::Tracing>(std::make_shared<vkoo::st::AABB>(
      vkoo::st::AABB::FromCuboid(glm::vec3{0.0f}, ToGlm(cuboid.halflengths))));
}

void CuboidNode::UpdateMode(Mode mode) {
  mode_ = mode;
  auto mesh = mesh_child_node_->GetComponentPtr<vkoo::st::Mesh>();
  auto wf_mesh = wireframe_child_node_->GetComponentPtr<vkoo::st::Mesh>();
  if (mode_ == Mode::Free || mode_ == Mode::Locked) {
    if (mode_ == Mode::Free) {
      mesh->SetMaterial(*default_material_);
    } else if (mode_ == Mode::Locked) {
      mesh->SetMaterial(*locked_material_);
    }
    wf_mesh->SetMaterial(*normal_wf_material_);
    wf_mesh->SetTransparent(false);
  } else if (mode_ == Mode::Focused || mode_ == Mode::Subtract) {
    if (mode_ == Mode::Focused) {
      mesh->SetMaterial(*focused_material_);
      wf_mesh->SetMaterial(*focused_wf_material_);
    } else {
      mesh->SetMaterial(*subtract_material_);
      wf_mesh->SetMaterial(*subtract_material_);
    }
    wf_mesh->SetTransparent(true);
    wf_mesh->SetAllowDepthTesting(false);
  } else {
    throw std::runtime_error("Unknown CuboidNode mode!");
  }
  wf_mesh->SetPolygonMode(vkoo::st::PolygonMode::Line);
  wf_mesh->SetPrimitiveTopology(vkoo::st::PrimitiveTopology::LineList);
  wf_mesh->SetLineWidth(2.0f);
}

void CuboidNode::Cleanup() {
  cuboid_vertex_object_ = nullptr;
  wireframe_vertex_object_ = nullptr;
  focused_material_ = nullptr;
  default_material_ = nullptr;
  locked_material_ = nullptr;
  subtract_material_ = nullptr;
  focused_wf_material_ = nullptr;
  normal_wf_material_ = nullptr;
}
}  // namespace hex
