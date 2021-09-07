#pragma once

#include "common.h"

#include <vkoo/core/Device.h>
#include <vkoo/core/VertexObject.h>
#include <vkoo/st/Material.h>

namespace hex {
// View for a collection of axis-aligned hexes of same size.
class HexCollectionView {
 public:
  HexCollectionView(vkoo::Device& device, vkoo::st::Node& parent_node,
                    float hex_size, const glm::vec4& color);
  ~HexCollectionView();
  void AddHex(const Vector3i& hex, bool wireframe, bool transparent);

 private:
  vkoo::Device& device_;
  vkoo::st::Node* wrapper_node_;

  float hex_size_;
  vkoo::st::Material hex_material_;
  std::shared_ptr<vkoo::VertexObject> hex_vbo_;
  vkoo::st::Material wireframe_material_;
  std::shared_ptr<vkoo::VertexObject> wireframe_vbo_;
};
}  // namespace hex
