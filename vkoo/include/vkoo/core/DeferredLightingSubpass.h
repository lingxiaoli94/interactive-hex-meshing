#pragma once

#include "Subpass.h"
#include "vkoo/core/ShaderModule.h"
#include "vkoo/st/Scene.h"
#include "vkoo/st/ShaderProgram.h"

#define MAX_DEFERRED_LIGHT_COUNT 32

namespace vkoo {
class DeferredLightingSubpass : public Subpass {
 public:
  DeferredLightingSubpass(RenderContext& render_context,
                          const st::ShaderProgram& shader_program,
                          st::Scene& scene, const core::Sampler& sampler,
                          uint32_t depth_attachment_id,
                          uint32_t albedo_attachment_id,
                          uint32_t normal_attachment_id);
  virtual void Prepare() override;
  virtual void Draw(CommandBuffer& command_buffer) override;
  void SetSSAOAttachment(uint32_t ssao_attachment_id);

 private:
  struct alignas(16) CameraUniform {
    glm::mat4 clip_to_world_mat;
    glm::vec2 inv_resolution;
  };

  struct alignas(16) DeferredLights {
    Light directional_lights[MAX_DEFERRED_LIGHT_COUNT];
    Light point_lights[MAX_DEFERRED_LIGHT_COUNT];
    uint32_t directional_light_count;
    uint32_t point_light_count;
  };

  struct alignas(16) SSAOFlags {
    glm::mat4 __placeholder;
    uint32_t use_ssao;
    uint32_t ssao_only;
  };

  const core::Sampler& sampler_;
  uint32_t depth_attachment_id_;
  uint32_t albedo_attachment_id_;
  uint32_t normal_attachment_id_;
  uint32_t ssao_attachment_id_;
  bool ssao_enabled_{false};
  ShaderVariant shader_variant_;
};
}  // namespace vkoo
