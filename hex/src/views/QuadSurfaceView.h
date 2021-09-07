#pragma once

#include <vkoo/core/Device.h>
#include <vkoo/st/Material.h>
#include <vkoo/st/Scene.h>

#include "models/QuadComplex.h"
#include "models/QuadrilateralMesh.h"

namespace hex {
class QuadSurfaceView {
 public:
  struct Options {
    glm::vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
    bool wireframe{true};
    glm::vec4 wireframe_color{0.1f, 0.1f, 0.1f, 1.0f};
    bool random{true};  // random only for multi-patch complexes
    int seed{42};
  };

  QuadSurfaceView(vkoo::Device& device, vkoo::st::Node& parent_node,
                  const Options& options);
  void Update(const QuadComplex& quad_complex,
              const std::vector<glm::vec4>& patch_colors = {});
  void Update(const QuadrilateralMesh& quad_mesh);
  vkoo::st::Node* GetWrapperNode() const;

 private:
  std::unique_ptr<vkoo::st::Node> CreateNodeForPatch(
      const std::vector<Vector3f>& vertices, const std::vector<Vector4i>& quads,
      const std::vector<size_t>& patch, const vkoo::st::Material& material);
  std::unique_ptr<vkoo::st::Node> CreateNodeForAllPatches(
      const std::vector<Vector3f>& vertices, const std::vector<Vector4i>& quads,
      const std::vector<std::vector<size_t>>& patches,
      const std::vector<glm::vec4>& patch_colors);
  const vkoo::st::Material& GetPatchMaterial(size_t patch_id);
  vkoo::Device& device_;

  vkoo::st::Node* wrapper_node_{nullptr};

  std::unordered_map<size_t, vkoo::st::Material> patch_materials_;
  vkoo::st::Material base_material_;
  vkoo::st::Material wireframe_material_;

  Options options_;
  std::default_random_engine rand_eng_;
};
}  // namespace hex
