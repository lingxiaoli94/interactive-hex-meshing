#include "vkoo/core/Subpass.h"

namespace vkoo {

Subpass::Subpass(RenderContext& render_context,
                 const st::ShaderProgram& shader_program, st::Scene& scene)
    : render_context_{render_context},
      shader_program_{shader_program},
      scene_{scene} {}

glm::mat4 VulkanStyleProjection(const glm::mat4& proj) {
  // Flip Y in clipspace. X = -1, Y = -1 is top-left in Vulkan.
  glm::mat4 mat = proj;
  mat[1][1] *= -1;

  return mat;
}
}  // namespace vkoo
