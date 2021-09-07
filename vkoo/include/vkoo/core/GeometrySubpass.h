#pragma once

#include "Subpass.h"
#include "vkoo/st/Scene.h"
#include "vkoo/st/components/Mesh.h"

namespace vkoo {
enum class TransparencyMode { SolidOnly, TransparentOnly, SolidAndTransparent };

class GeometrySubpass : public Subpass {
 public:
  GeometrySubpass(RenderContext& render_context,
                  const st::ShaderProgram& shader_program, st::Scene& scene,
                  TransparencyMode blending_mode);
  virtual ~GeometrySubpass() = default;
  virtual void Prepare() override;
  virtual void Draw(CommandBuffer& command_buffer) override;

 protected:
  // No alignas since this is used for push constant.
  struct MaterialUniform {
    glm::vec4 diffuse_color;
    uint32_t has_texture;
  };

  struct alignas(16) GlobalUniform {
    glm::mat4 model_mat;
    glm::mat4 normal_mat;
    glm::mat4 view_proj_mat;
    glm::vec3 camera_position;
  };

  void UpdateUniform(CommandBuffer& command_buffer, st::Node& node);
  void DrawSubmesh(CommandBuffer& command_buffer, st::Mesh& mesh);
  void DrawSubmeshCommand(CommandBuffer& command_buffer, st::Mesh& mesh);
  void PreparePipelineState(CommandBuffer& command_buffer, st::Mesh& mesh);
  PipelineLayout& GetPipelineLayout(
      CommandBuffer& command_buffer,
      const std::vector<ShaderModule*>& shader_modules);

  TransparencyMode transparency_mode_;
};
}  // namespace vkoo
