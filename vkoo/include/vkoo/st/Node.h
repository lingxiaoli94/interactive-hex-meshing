#pragma once

#include <typeindex>

#include "Transform.h"
#include "vkoo/common.h"
#include "vkoo/st/components/ComponentBase.h"

namespace vkoo {
namespace st {
class Node {
 public:
  Node();
  virtual ~Node() = default;

  // Disallow copy constructor & assignment by default.
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  size_t GetChildrenCount() const { return children_.size(); }

  Node& GetChild(size_t index) const { return *children_.at(index); }

  Node* GetParentPtr() const { return parent_; }

  void AddChild(std::unique_ptr<Node> child);

  std::unique_ptr<Node> RemoveChild(std::function<bool(Node*)> predicate);
  std::unique_ptr<Node> RemoveChild(Node* child);
  void RemoveAllChildren();
  void RemoveFromParent();

  template <class T>
  void SetComponent(std::unique_ptr<T> component) {
    component->SetNode(this);
    component_dict_[component->GetType()] = std::move(component);
  }

  template <class T>
  bool RemoveComponent() {
    auto itr = component_dict_.find(typeid(T));
    if (itr != component_dict_.end()) {
      component_dict_.erase(itr);
      return true;
    }
    return false;
  }

  template <class T, typename... Args>
  T& CreateComponent(Args&&... args) {
    auto component = std::make_unique<T>(std::forward<Args>(args)...);
    std::type_index index = component->GetType();
    SetComponent<T>(std::move(component));
    return static_cast<T&>(*GetComponent(index));
  }

  ComponentBase* GetComponent(std::type_index index) const;

  template <class T>
  T* GetComponentPtr() const {
    return static_cast<T*>(GetComponent(typeid(T)));
  }

  template <class T>
  std::vector<T*> GetComponentsRecursive(bool visible_only = false) const {
    std::vector<T*> result;
    std::vector<ComponentBase*> result_raw;
    GatherComponentsRecursive(typeid(T), result_raw, visible_only);
    for (ComponentBase* c : result_raw) {
      result.push_back(static_cast<T*>(c));
    }
    return result;
  }

  Transform& GetTransform() { return transform_; }

  const Transform& GetTransform() const { return transform_; }

  const std::string& GetTag() const { return tag_; }
  void SetTag(const std::string& tag) { tag_ = tag; }

  bool IsVisible() const { return visible_; }
  void SetVisible(bool visible) { visible_ = visible; }

 private:
  void GatherComponentsRecursive(std::type_index index,
                                 std::vector<ComponentBase*>& result,
                                 bool visible_only = false) const;

  Transform transform_;
  std::unordered_map<std::type_index, std::unique_ptr<ComponentBase>>
      component_dict_;
  std::vector<std::unique_ptr<Node>> children_;
  Node* parent_{nullptr};
  std::string tag_;
  bool visible_{true};
};
}  // namespace st
}  // namespace vkoo
