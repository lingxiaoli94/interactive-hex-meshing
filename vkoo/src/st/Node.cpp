#include "vkoo/st/Node.h"

namespace vkoo {
namespace st {

void Node::GatherComponentsRecursive(std::type_index index,
                                     std::vector<ComponentBase*>& result,
                                     bool visible_only) const {
  if (!visible_) {
    return;
  }
  ComponentBase* component = GetComponent(index);
  if (component != nullptr) {
    result.push_back(component);
  }
  size_t child_count = GetChildrenCount();
  for (size_t i = 0; i < child_count; i++) {
    Node& child = GetChild(i);
    child.GatherComponentsRecursive(index, result);
  }
}

ComponentBase* Node::GetComponent(std::type_index index) const {
  auto itr = component_dict_.find(index);
  if (itr != component_dict_.end()) {
    return itr->second.get();
  } else {
    return nullptr;
  }
}

Node::Node() : transform_(*this) {}

void Node::AddChild(std::unique_ptr<Node> child) {
  child->parent_ = this;
  children_.emplace_back(std::move(child));
}

std::unique_ptr<Node> Node::RemoveChild(std::function<bool(Node*)> predicate) {
  auto it = std::find_if(children_.begin(), children_.end(),
                         [&](const std::unique_ptr<Node>& child) {
                           if (predicate(child.get())) {
                             return true;
                           }
                           return false;
                         });
  if (it == children_.end()) {
    return nullptr;
  }
  auto ret = std::move(*it);
  children_.erase(it);
  return ret;
}

std::unique_ptr<Node> Node::RemoveChild(Node* child) {
  return RemoveChild([&](Node* node) { return node == child; });
}

void Node::RemoveAllChildren() { children_.clear(); }

void Node::RemoveFromParent() {
  assert(parent_);
  parent_->RemoveChild(this);
}

}  // namespace st
}  // namespace vkoo
