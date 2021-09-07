#include "QuadSurfaceView.h"

#include <vkoo/core/VertexObject.h>
#include <vkoo/st/components/Mesh.h>

#include "logging.h"

namespace hex {
namespace {
const float kWireframeOffset = 1e-4f;
}
QuadSurfaceView::QuadSurfaceView(vkoo::Device& device,
                                 vkoo::st::Node& parent_node,
                                 const Options& options)
    : device_{device}, options_{options} {
  rand_eng_ = std::default_random_engine(options_.seed);
  base_material_.colors["diffuse_color"] = options.base_color;
  wireframe_material_.colors["diffuse_color"] = options.wireframe_color;

  auto wrapper_node = std::make_unique<vkoo::st::Node>();
  wrapper_node_ = wrapper_node.get();
  parent_node.AddChild(std::move(wrapper_node));
}

vkoo::st::Node* QuadSurfaceView::GetWrapperNode() const {
  return wrapper_node_;
}

std::unique_ptr<vkoo::st::Node> QuadSurfaceView::CreateNodeForAllPatches(
    const std::vector<Vector3f>& vertices, const std::vector<Vector4i>& quads,
    const std::vector<std::vector<size_t>>& patches,
    const std::vector<glm::vec4>& patch_colors) {
  auto node = std::make_unique<vkoo::st::Node>();
  if (patches.empty()) {
    return node;
  }
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec4> colors;
  std::vector<uint32_t> indices;
  std::vector<glm::vec3> wireframe_positions;
  std::vector<uint32_t> wireframe_indices;

  for (size_t i = 0; i < patches.size(); i++) {
    auto& patch = patches[i];
    for (auto& quad_id : patch) {
      auto& quad = quads[quad_id];
      Vector3f v[4];
      for (size_t k = 0; k < 4; k++) {
        v[k] = vertices[quad(k)];
      }
      Vector3f n = (v[1] - v[0]).cross(v[2] - v[0]).normalized();
      for (size_t k = 0; k < 4; k++) {
        positions.push_back(ToGlm(v[k]));
        normals.push_back(ToGlm(n));
        colors.push_back(patch_colors.at(i));
        wireframe_positions.push_back(positions.back() +
                                      kWireframeOffset * normals.back());
      }

      uint32_t j = static_cast<uint32_t>(positions.size() - 4);
      indices.insert(indices.end(), {j, j + 1, j + 2, j, j + 2, j + 3});
      wireframe_indices.insert(
          wireframe_indices.end(),
          {j, j + 1, j + 1, j + 2, j + 2, j + 3, j + 3, j});
    }
  }

  if (indices.size() > 0) {
    auto vertex_object = std::make_shared<vkoo::VertexObject>(device_);
    vertex_object->Update("position", positions);
    vertex_object->Update("normal", normals);
    vertex_object->Update("color", colors);
    vertex_object->UpdateIndices(indices);

    auto patches_node = std::make_unique<vkoo::st::Node>();
    auto& mesh = patches_node->CreateComponent<vkoo::st::Mesh>(vertex_object,
                                                               indices.size());
    mesh.UpdateShaderVariant();  // FIXME: we are using per-vertex color
                                 // here, but there should be a more
                                 // elegant way.

    node->AddChild(std::move(patches_node));
  }
  if (wireframe_indices.size() > 0) {
    auto vertex_object = std::make_shared<vkoo::VertexObject>(device_);
    vertex_object->Update("position", wireframe_positions);
    vertex_object->Update("normal", normals);
    vertex_object->UpdateIndices(wireframe_indices);

    auto wireframe_node = std::make_unique<vkoo::st::Node>();
    auto& wireframe_mesh = wireframe_node->CreateComponent<vkoo::st::Mesh>(
        vertex_object, wireframe_indices.size());
    wireframe_mesh.SetMaterial(wireframe_material_);
    wireframe_mesh.SetPolygonMode(vkoo::st::PolygonMode::Line);
    wireframe_mesh.SetPrimitiveTopology(vkoo::st::PrimitiveTopology::LineList);
    wireframe_mesh.SetLineWidth(2.0f);

    node->AddChild(std::move(wireframe_node));
  }
  return node;
}

std::unique_ptr<vkoo::st::Node> QuadSurfaceView::CreateNodeForPatch(
    const std::vector<Vector3f>& vertices, const std::vector<Vector4i>& quads,
    const std::vector<size_t>& patch, const vkoo::st::Material& material) {
  auto node = std::make_unique<vkoo::st::Node>();
  if (patch.empty()) {
    return node;
  }

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> indices;
  std::vector<glm::vec3> wireframe_positions;
  std::vector<uint32_t> wireframe_indices;

  for (auto& quad_id : patch) {
    auto& quad = quads[quad_id];
    Vector3f v[4];
    for (size_t k = 0; k < 4; k++) {
      v[k] = vertices[quad(k)];
    }
    Vector3f n = (v[1] - v[0]).cross(v[2] - v[0]).normalized();
    for (size_t k = 0; k < 4; k++) {
      positions.push_back(ToGlm(v[k]));
      normals.push_back(ToGlm(n));
      wireframe_positions.push_back(positions.back() +
                                    kWireframeOffset * normals.back());
    }

    uint32_t j = static_cast<uint32_t>(positions.size() - 4);
    indices.insert(indices.end(), {j, j + 1, j + 2, j, j + 2, j + 3});
    wireframe_indices.insert(wireframe_indices.end(),
                             {j, j + 1, j + 1, j + 2, j + 2, j + 3, j + 3, j});
  }

  {
    auto vertex_object = std::make_shared<vkoo::VertexObject>(device_);
    vertex_object->Update("position", positions);
    vertex_object->Update("normal", normals);
    vertex_object->UpdateIndices(indices);

    auto patch_node = std::make_unique<vkoo::st::Node>();
    auto& mesh = patch_node->CreateComponent<vkoo::st::Mesh>(vertex_object,
                                                             indices.size());
    mesh.SetMaterial(material);

    node->AddChild(std::move(patch_node));
  }
  {
    auto vertex_object = std::make_shared<vkoo::VertexObject>(device_);
    vertex_object->Update("position", wireframe_positions);
    vertex_object->Update("normal", normals);
    vertex_object->UpdateIndices(wireframe_indices);

    auto wireframe_node = std::make_unique<vkoo::st::Node>();
    auto& wireframe_mesh = wireframe_node->CreateComponent<vkoo::st::Mesh>(
        vertex_object, wireframe_indices.size());
    wireframe_mesh.SetMaterial(wireframe_material_);
    wireframe_mesh.SetPolygonMode(vkoo::st::PolygonMode::Line);
    wireframe_mesh.SetPrimitiveTopology(vkoo::st::PrimitiveTopology::LineList);
    wireframe_mesh.SetLineWidth(2.0f);

    node->AddChild(std::move(wireframe_node));
  }
  return node;
}

void QuadSurfaceView::Update(const QuadComplex& quad_complex,
                             const std::vector<glm::vec4>& patch_colors) {
  wrapper_node_->RemoveAllChildren();

  auto& complex_vertices = quad_complex.GetVertices();
  auto& complex_quads = quad_complex.GetQuads();
  auto& complex_patches = quad_complex.GetPatches();

  auto node = CreateNodeForAllPatches(complex_vertices, complex_quads,
                                      complex_patches, patch_colors);
  wrapper_node_->AddChild(std::move(node));
}

void QuadSurfaceView::Update(const QuadrilateralMesh& quad_mesh) {
  wrapper_node_->RemoveAllChildren();

  auto& vertices = quad_mesh.GetVertices();
  auto& quads = quad_mesh.GetQuads();
  // Create a dummy patch that is the entire quad mesh.
  std::vector<size_t> patch;
  for (size_t i = 0; i < quads.size(); i++) {
    patch.push_back(i);
  }
  auto node = CreateNodeForPatch(vertices, quads, patch, base_material_);
  wrapper_node_->AddChild(std::move(node));
}

const vkoo::st::Material& QuadSurfaceView::GetPatchMaterial(size_t patch_id) {
  std::uniform_real_distribution<float> dis(0.0f, 1.0f);

  auto it = patch_materials_.find(patch_id);
  if (it == patch_materials_.end()) {
    glm::vec4 new_color((float)dis(rand_eng_), (float)dis(rand_eng_),
                        (float)dis(rand_eng_), 1.0f);
    vkoo::st::Material new_material;
    new_material.colors["diffuse_color"] = new_color;
    auto emplace_it = patch_materials_.emplace(patch_id, new_material);
    assert(emplace_it.second);
    it = emplace_it.first;
  }

  return it->second;
}
}  // namespace hex
