#pragma once

#include "ComponentBase.h"
#include "vkoo/core/ShaderModule.h"
#include "vkoo/core/VertexObject.h"

namespace vkoo {
namespace st {
class Material;

enum class PrimitiveTopology { TriangleList, LineList, PointList };

enum class PolygonMode { Line, Fill };

class Mesh : public ComponentBase {
 public:
  Mesh(std::shared_ptr<VertexObject> vertex_object, uint32_t index_count,
       uint32_t index_offset = 0);
  std::type_index GetType() const override;

  VertexObject& GetVertexObject() const { return *vertex_object_; }
  uint32_t GetIndexOffset() const { return index_offset_; }
  uint32_t GetIndexCount() const { return index_count_; }
  void SetAttribute(const std::string& name, const VertexAttribute& attribute);
  void SetAttributes(
      const std::unordered_map<std::string, VertexAttribute>& attributes);
  std::optional<VertexAttribute> GetAttribute(const std::string& name) const;
  void SetMaterial(const Material& material);
  const Material* GetMaterial() const { return material_; }
  float GetLineWidth() const;

  PrimitiveTopology GetPrimitiveTopology() const;
  void SetPrimitiveTopology(PrimitiveTopology topology);
  void SetLineWidth(float line_width);
  PolygonMode GetPolygonMode() const;
  void SetPolygonMode(PolygonMode polygon_mode);
  void SetTransparent(bool transparent);
  bool IsTransparent() const { return transparent_; }
  void SetAllowDepthTesting(bool allow);
  bool AllowDepthTesting() const { return allow_depth_testing_; }
  const ShaderVariant& GetShaderVariant() const { return shader_variant_; }
  void UpdateShaderVariant();

 private:
  std::shared_ptr<VertexObject> vertex_object_;
  uint32_t index_offset_;
  uint32_t index_count_;
  std::unordered_map<std::string, VertexAttribute> vertex_attributes_;
  const Material* material_{nullptr};
  bool transparent_{false};
  bool allow_depth_testing_{true};

  PrimitiveTopology topology_{PrimitiveTopology::TriangleList};
  PolygonMode polygon_mode_{PolygonMode::Fill};
  float line_width_{1.0f};

  ShaderVariant shader_variant_;
};
}  // namespace st
}  // namespace vkoo
