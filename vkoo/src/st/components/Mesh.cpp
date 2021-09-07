#include "vkoo/st/components/Mesh.h"

#include "vkoo/st/Material.h"

namespace vkoo {
namespace st {
Mesh::Mesh(std::shared_ptr<VertexObject> vertex_object, uint32_t index_count,
           uint32_t index_offset)
    : vertex_object_{std::move(vertex_object)},
      index_offset_{index_offset},
      index_count_{index_count} {
  if (vertex_object_->FindResource("tex_coord") == nullptr) {
    vertex_object_->Update(
        "tex_coord",
        std::vector<glm::vec2>(vertex_object_->GetResourceCount("position"),
                               glm::vec2(0.0f, 0.0f)));
  }
  SetAttributes(vertex_object_->GetVertexAttributes());
}

void Mesh::SetAttribute(const std::string& name,
                        const VertexAttribute& attribute) {
  vertex_attributes_[name] = attribute;
}

void Mesh::SetAttributes(
    const std::unordered_map<std::string, VertexAttribute>& attributes) {
  for (const auto& kv : attributes) {
    vertex_attributes_[kv.first] = kv.second;
  }
}

std::optional<VertexAttribute> Mesh::GetAttribute(
    const std::string& name) const {
  auto it = vertex_attributes_.find(name);
  if (it == vertex_attributes_.end()) {
    return {};
  }
  return it->second;
}

void Mesh::SetMaterial(const Material& material) {
  material_ = &material;
  UpdateShaderVariant();
}

void Mesh::UpdateShaderVariant() {
  shader_variant_.Clear();
  if (material_ != nullptr) {
    for (auto& pv : material_->textures) {
      std::string texture_name = pv.first;
      std::transform(texture_name.begin(), texture_name.end(),
                     texture_name.begin(), toupper);

      shader_variant_.AddDef("HAS_" + texture_name);
    }
  }
  if (topology_ == PrimitiveTopology::PointList) {
    shader_variant_.AddDef("DRAW_POINTS_ONLY");
  }
  if (GetAttribute("color")) {
    shader_variant_.AddDef("PER_VERTEX_COLOR");
  }
}

std::type_index Mesh::GetType() const { return typeid(Mesh); }

PrimitiveTopology Mesh::GetPrimitiveTopology() const { return topology_; }

void Mesh::SetPrimitiveTopology(PrimitiveTopology topology) {
  topology_ = topology;
  UpdateShaderVariant();
}

PolygonMode Mesh::GetPolygonMode() const { return polygon_mode_; }

void Mesh::SetPolygonMode(PolygonMode polygon_mode) {
  polygon_mode_ = polygon_mode;
}

void Mesh::SetTransparent(bool transparent) { transparent_ = transparent; }

void Mesh::SetAllowDepthTesting(bool allow) { allow_depth_testing_ = allow; }

float Mesh::GetLineWidth() const { return line_width_; }

void Mesh::SetLineWidth(float line_width) { line_width_ = line_width; }
}  // namespace st
}  // namespace vkoo
