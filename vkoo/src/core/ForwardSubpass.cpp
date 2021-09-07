#include "vkoo/core/ForwardSubpass.h"

#include "vkoo/st/components/Light.h"

namespace vkoo {
ForwardSubpass::ForwardSubpass(RenderContext& render_context,
                               const st::ShaderProgram& shader_program,
                               st::Scene& scene,
                               TransparencyMode transparency_mode)
    : GeometrySubpass{render_context, shader_program, scene,
                      transparency_mode} {}

void ForwardSubpass::Prepare() { GeometrySubpass::Prepare(); }

void ForwardSubpass::Draw(CommandBuffer& command_buffer) {
  ForwardLights light_ubo;
  AggregateLights(light_ubo,
                  scene_.GetRoot().GetComponentsRecursive<st::Light>());
  auto& render_frame = render_context_.GetActiveFrame();
  auto& light_buffer = render_frame.AllocateBuffer(
      VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(ForwardLights));
  light_buffer.ConvertAndUpdate(light_ubo);

  // The lights' uniform parameter is always at binding = 4.
  command_buffer.BindBuffer(light_buffer, 0, light_buffer.GetSize(), 0, 4, 0);

  GeometrySubpass::Draw(command_buffer);
}
}  // namespace vkoo