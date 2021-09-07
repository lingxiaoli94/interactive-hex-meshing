#pragma once

#include "VertexObject.h"

namespace vkoo {
class PrimitiveFactory {
 public:
  // Create a radius-r sphere with prescribed vertical/horizontal numbers
  // of slices/stacks.
  static std::unique_ptr<VertexObject> CreateSphere(Device& device, float r,
                                                    size_t slices,
                                                    size_t stacks);

  // Create a cylinder with prescribed radius, height, and number of sides.
  static std::unique_ptr<VertexObject> CreateCylinder(Device& device, float r,
                                                      float h,
                                                      size_t num_sides);

  // Create a standard cube with corners at -1 or 1.
  static std::unique_ptr<VertexObject> CreateCube(Device& device);

  // Create a flat quad with corners at -1 or 1 in both x and y
  // coordinates.
  static std::unique_ptr<VertexObject> CreateQuad(Device& device);

  // Create a line segment between p and q.
  static std::unique_ptr<VertexObject> CreateLineSegment(Device& device,
                                                         const glm::vec3& p,
                                                         const glm::vec3& q);
};
}  // namespace vkoo
