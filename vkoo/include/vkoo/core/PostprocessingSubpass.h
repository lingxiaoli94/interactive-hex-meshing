#pragma once

#include "Subpass.h"
#include "vkoo/st/Image.h"
#include "vkoo/st/Scene.h"
#include "vkoo/st/ShaderProgram.h"

namespace vkoo {
class PostprocessingSubpass : public Subpass {
 public:
  PostprocessingSubpass(RenderContext& render_context,
                        const st::ShaderProgram& shader_program,
                        st::Scene& scene, uint32_t input_detachment_id);
  void Prepare() override;
  void Draw(CommandBuffer& command_buffer) override;

 private:
  std::unique_ptr<core::Sampler> input_sampler_;
  uint32_t input_attachment_id_;
};
}  // namespace vkoo
