#pragma once

#include <vkoo/core/VertexObject.h>
#include <vkoo/st/Material.h>
#include <vkoo/st/Node.h>

#include "models/Cuboid.h"

namespace hex {
class CuboidNode : public vkoo::st::Node {
 public:
  enum class Mode { Free, Locked, Focused, Subtract };
  CuboidNode(vkoo::Device& device);
  void UpdateView(const Cuboid& cuboid);
  void UpdateMode(Mode mode);

  static void Cleanup();

 private:
  std::shared_ptr<vkoo::VertexObject> CreateWireframeVbo(vkoo::Device& device);

  vkoo::st::Node* mesh_child_node_;
  vkoo::st::Node* wireframe_child_node_;
  Mode mode_{Mode::Free};
};

}  // namespace hex
