#pragma once

#include "GeometrySubpass.h"
#include "vkoo/core/Buffer.h"

#define MAX_FORWARD_LIGHT_COUNT 32

namespace vkoo {
class ForwardSubpass : public GeometrySubpass {
 public:
  ForwardSubpass(RenderContext& render_context,
                 const st::ShaderProgram& shader_program, st::Scene& scene,
                 TransparencyMode transparency_mode);

  void Prepare() override;
  void Draw(CommandBuffer& command_buffer) override;

 private:
  struct alignas(16) ForwardLights {
    Light directional_lights[MAX_FORWARD_LIGHT_COUNT];
    Light point_lights[MAX_FORWARD_LIGHT_COUNT];
    uint32_t directional_light_count;
    uint32_t point_light_count;
  };
};
}  // namespace vkoo
