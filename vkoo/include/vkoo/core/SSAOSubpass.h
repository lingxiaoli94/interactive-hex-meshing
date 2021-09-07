#pragma once

#include "Subpass.h"
#include "vkoo/st/Image.h"
#include "vkoo/st/Scene.h"
#include "vkoo/st/ShaderProgram.h"

namespace vkoo {
class SSAOSubpass : public Subpass {
 public:
  SSAOSubpass(RenderContext& render_context,
              const st::ShaderProgram& shader_program, st::Scene& scene,
              const core::Sampler& texture_sampler,
              uint32_t depth_attachment_id, uint32_t normal_attachment_id);
  void Prepare() override;
  void Draw(CommandBuffer& command_buffer) override;

 private:
  struct alignas(16) CameraUniform {
    glm::mat4 world_to_clip_mat;
    glm::mat4 clip_to_world_mat;
  };

  const size_t kSSAOKernelSize = 64;
  [[maybe_unused]] const float kSSAORadius = 0.2f;
  const size_t kSSAONoiseDim = 4;

  std::unique_ptr<core::Buffer> ssao_kernel_buffer_;
  std::unique_ptr<st::Image> ssao_noise_image_;
  std::unique_ptr<core::Sampler> noise_sampler_;

  const core::Sampler& texture_sampler_;
  uint32_t depth_attachment_id_;
  uint32_t normal_attachment_id_;
};
}  // namespace vkoo
