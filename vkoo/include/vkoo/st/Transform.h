#pragma once

#include "vkoo/common.h"

namespace vkoo {
namespace st {
class Node;

class Transform {
 public:
  Transform(Node& node);
  void SetPosition(const glm::vec3& position);
  void SetRotation(const glm::quat& rotation);
  void SetRotation(const glm::vec3& axis, float angle);
  void SetScale(const glm::vec3& scale);
  void SetMatrix4x4(const glm::mat4& T);
  glm::vec3 GetPosition() const { return position_; }
  glm::quat GetRotation() const { return rotation_; }
  glm::vec3 GetScale() const { return scale_; }
  glm::vec3 GetWorldPosition() const;
  glm::mat4 GetWorldToLocalMatrix() const;
  glm::mat4 GetLocalToWorldMatrix() const;
  glm::mat4 GetLocalToParentMatrix() const;
  glm::mat4 GetLocalToAncestorMatrix(Node* ancestor) const;
  glm::vec3 GetForwardDirection() const;
  glm::vec3 GetUpDirection() const;
  glm::vec3 GetRightDirection() const;
  static glm::vec3 GetWorldUp();
  static glm::vec3 GetWorldRight();
  static glm::vec3 GetWorldForward();

 private:
  void UpdateLocalTransformMatrix();

  glm::vec3 position_;
  glm::quat rotation_;
  glm::vec3 scale_;

  glm::mat4 local_transform_mat_;

  Node& node_;
};
}  // namespace st
}  // namespace vkoo
