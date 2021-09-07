#pragma once

#include <vkoo/st/Scene.h>

#include "CuboidNode.h"
#include "models/Cuboid.h"
#include "models/Polycube.h"
#include "models/PolycubeInfo.h"

namespace hex {
// This view class manages all cuboids scene nodes.
class PolycubeView {
 public:
  PolycubeView(vkoo::Device& device, vkoo::st::Node* wrapper_node);
  ~PolycubeView();
  void AddCuboid(const Cuboid& cuboid);
  void DeleteCuboid(int i);
  void Update(const Polycube& polycube, const PolycubeInfo& polycube_info);
  CuboidNode& GetCuboidNode(int i);
  const std::vector<CuboidNode*>& GetCuboidNodes() { return cuboid_nodes_; }

  // Temporary cuboid is for cuboid subtraction.
  void CreateTmpCuboid(const Cuboid& cuboid);
  void DeleteTmpCuboid();
  CuboidNode& GetTmpCuboidNode();

  void Cleanup();

 private:
  vkoo::Device& device_;

  std::vector<CuboidNode*> cuboid_nodes_;
  CuboidNode* tmp_cuboid_node_{nullptr};
  vkoo::st::Node* wrapper_node_{nullptr};
};
}  // namespace hex
