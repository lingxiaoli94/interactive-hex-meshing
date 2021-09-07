#pragma once

#include "vkoo/common.h"

namespace vkoo {
namespace st {
class Node;
class ComponentBase {
 public:
  virtual ~ComponentBase() = default;

  // Disallow copy constructor & assignment by default.
  ComponentBase() {}
  ComponentBase(const ComponentBase&) = delete;
  ComponentBase& operator=(const ComponentBase&) = delete;

  void SetNode(Node* node) { node_ = node; }
  Node* GetNode() const { return node_; }
  virtual std::type_index GetType() const = 0;

 protected:
  Node* node_;
};
}  // namespace st
}  // namespace vkoo
