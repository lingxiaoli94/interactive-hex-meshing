#include "PolycubeView.h"

namespace hex {
// FIXME: passing wrapper_node is bad practice. See other views.
PolycubeView::PolycubeView(vkoo::Device& device, vkoo::st::Node* wrapper_node)
    : device_{device}, wrapper_node_{wrapper_node} {}

PolycubeView::~PolycubeView() { Cleanup(); }

void PolycubeView::AddCuboid(const Cuboid& cuboid) {
  auto cuboid_node = std::make_unique<CuboidNode>(device_);
  cuboid_node->UpdateView(cuboid);

  cuboid_nodes_.push_back(cuboid_node.get());
  wrapper_node_->AddChild(std::move(cuboid_node));
}

void PolycubeView::DeleteCuboid(int i) {
  wrapper_node_->RemoveChild(cuboid_nodes_.at(i));
  cuboid_nodes_.erase(cuboid_nodes_.begin() + i);
}

CuboidNode& PolycubeView::GetCuboidNode(int i) { return *cuboid_nodes_.at(i); }

void PolycubeView::Update(const Polycube& polycube,
                          const PolycubeInfo& polycube_info) {
  Cleanup();
  for (size_t i = 0; i < polycube.GetCuboidCount(); i++) {
    auto& cuboid = polycube.GetCuboid(i);
    AddCuboid(cuboid);
    cuboid_nodes_.back()->UpdateMode(polycube_info.locked[i]
                                         ? CuboidNode::Mode::Locked
                                         : CuboidNode::Mode::Free);
  }
}

void PolycubeView::CreateTmpCuboid(const Cuboid& cuboid) {
  if (tmp_cuboid_node_ != nullptr) {
    DeleteTmpCuboid();
  }
  auto cuboid_node = std::make_unique<CuboidNode>(device_);
  cuboid_node->UpdateView(cuboid);

  tmp_cuboid_node_ = cuboid_node.get();
  wrapper_node_->AddChild(std::move(cuboid_node));
}

void PolycubeView::DeleteTmpCuboid() {
  assert(tmp_cuboid_node_ != nullptr);
  tmp_cuboid_node_->RemoveFromParent();
  tmp_cuboid_node_ = nullptr;
}

CuboidNode& PolycubeView::GetTmpCuboidNode() {
  assert(tmp_cuboid_node_);
  return *tmp_cuboid_node_;
}

void PolycubeView::Cleanup() {
  for (auto& node : cuboid_nodes_) {
    wrapper_node_->RemoveChild(node);
  }
  cuboid_nodes_.clear();
}
}  // namespace hex
