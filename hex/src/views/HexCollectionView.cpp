#include "HexCollectionView.h"

#include <vkoo/core/VertexObject.h>
#include <vkoo/st/Node.h>
#include <vkoo/st/components/Mesh.h>

#include "logging.h"
#include "models/HexConvention.h"

namespace hex {
namespace {
const float kWireframeOffset = 1e-4f;
}

HexCollectionView::HexCollectionView(vkoo::Device& device,
                                     vkoo::st::Node& parent_node,
                                     float hex_size, const glm::vec4& color)
    : device_{device}, hex_size_{hex_size} {
  wireframe_material_.colors["diffuse_color"] = {0.1f, 0.1f, 0.1f, 1.0f};
  hex_material_.colors["diffuse_color"] = color;

  auto& kCorners = HexConvention::GetCorners();  // {0, 1}
  auto& kFaceCornerIds = HexConvention::GetFaceCornerIds();

  // Build VertexObjects for a single hex.
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> indices;
  std::vector<glm::vec3> wireframe_positions;
  std::vector<uint32_t> wireframe_indices;

  for (auto& face : kFaceCornerIds) {
    Vector3f v[4];
    for (size_t k = 0; k < 4; k++) {
      v[k] = kCorners[face[k]].cast<float>();
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

  hex_vbo_ = std::make_shared<vkoo::VertexObject>(device_);
  hex_vbo_->Update("position", positions);
  hex_vbo_->Update("normal", normals);
  hex_vbo_->UpdateIndices(indices);

  wireframe_vbo_ = std::make_shared<vkoo::VertexObject>(device_);
  wireframe_vbo_->Update("position", wireframe_positions);
  wireframe_vbo_->Update("normal", normals);
  wireframe_vbo_->UpdateIndices(wireframe_indices);

  auto node = std::make_unique<vkoo::st::Node>();
  wrapper_node_ = node.get();
  parent_node.AddChild(std::move(node));
};

HexCollectionView::~HexCollectionView() {
  assert(wrapper_node_);
  wrapper_node_->RemoveFromParent();
}

void HexCollectionView::AddHex(const Vector3i& hex, bool wireframe,
                               bool transparent) {
  auto scale = glm::vec3{hex_size_, hex_size_, hex_size_};
  auto position =
      glm::vec3{(float)hex.x() * hex_size_, (float)hex.y() * hex_size_,
                (float)hex.z() * hex_size_};
  auto hex_node = std::make_unique<vkoo::st::Node>();
  hex_node->GetTransform().SetScale(scale);
  hex_node->GetTransform().SetPosition(position);

  auto& hex_mesh = hex_node->CreateComponent<vkoo::st::Mesh>(
      hex_vbo_, hex_vbo_->GetIndexCount());
  hex_mesh.SetMaterial(hex_material_);
  if (transparent) {
    hex_mesh.SetTransparent(true);
    hex_mesh.SetAllowDepthTesting(false);
  }
  wrapper_node_->AddChild(std::move(hex_node));

  // FIXME: wireframe now appears weird due to alpha blending overwriting black
  // color.
  if (wireframe) {
    auto wireframe_node = std::make_unique<vkoo::st::Node>();
    wireframe_node->GetTransform().SetScale(scale);
    wireframe_node->GetTransform().SetPosition(position);
    auto& wireframe_mesh = wireframe_node->CreateComponent<vkoo::st::Mesh>(
        wireframe_vbo_, wireframe_vbo_->GetIndexCount());
    wireframe_mesh.SetMaterial(wireframe_material_);
    wireframe_mesh.SetPolygonMode(vkoo::st::PolygonMode::Line);
    wireframe_mesh.SetPrimitiveTopology(vkoo::st::PrimitiveTopology::LineList);
    wrapper_node_->AddChild(std::move(wireframe_node));
  }
}
}  // namespace hex
