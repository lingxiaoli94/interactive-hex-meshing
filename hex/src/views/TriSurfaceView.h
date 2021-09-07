#pragma once

#include <vkoo/core/Device.h>
#include <vkoo/st/Material.h>
#include <vkoo/st/Scene.h>

#include "models/TriangularMesh.h"
#include "vkoo/st/components/Mesh.h"

namespace hex {
class TriSurfaceView {
 public:
  struct Options {
    glm::vec4 opaque_color{1.0f, 1.0f, 1.0f, 1.0f};
    bool wireframe{true};
    glm::vec4 wireframe_color{0.1f, 0.1f, 0.1f, 1.0f};
    bool transparent{false};
    glm::vec4 transparent_color{0.0f, 1.0f, 0.1f, 0.5f};
  };

  TriSurfaceView(vkoo::Device& device, vkoo::st::Node& parent_node,
                 const Options& options);
  void Update(const TriangularMesh& tri_mesh);
  void SetTransparency(bool transparent);
  void SetWireframeVisible(bool visible);
  vkoo::st::Node* GetWrapperNode() const;
  bool IsEmpty() const;

 private:
  vkoo::Device& device_;

  vkoo::st::Node* wrapper_node_{nullptr};
  vkoo::st::Mesh* surface_mesh_{nullptr};
  vkoo::st::Mesh* wireframe_mesh_{nullptr};

  vkoo::st::Material opaque_material_;
  vkoo::st::Material transparent_material_;
  vkoo::st::Material wireframe_material_;

  Options options_;
};
}  // namespace hex
