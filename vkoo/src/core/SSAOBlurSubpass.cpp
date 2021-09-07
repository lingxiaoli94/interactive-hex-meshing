#include "vkoo/core/SSAOBlurSubpass.h"

namespace vkoo {
SSAOBlurSubpass::SSAOBlurSubpass(RenderContext& render_context,
                                 const st::ShaderProgram& shader_program,
                                 st::Scene& scene, uint32_t ssao_detachment_id)
    : Subpass{render_context, shader_program, scene},
      ssao_detachment_id_{ssao_detachment_id} {
  // Create sampler.
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  sampler_info.anisotropyEnable = VK_TRUE;
  sampler_info.maxAnisotropy = 1.0f;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

  blur_sampler_ = std::make_unique<vkoo::core::Sampler>(
      render_context.GetDevice(), sampler_info);
}

void SSAOBlurSubpass::Prepare() {}

void SSAOBlurSubpass::Draw(CommandBuffer& command_buffer) {
  auto& render_frame = render_context_.GetActiveFrame();

  // Bind pipeline layout.
  auto& resource_cache = command_buffer.GetDevice().GetResourceCache();
  auto& vert_shader_module = resource_cache.RequestShaderModule(
      VK_SHADER_STAGE_VERTEX_BIT, shader_program_.vertex_shader_source);
  auto& frag_shader_module = resource_cache.RequestShaderModule(
      VK_SHADER_STAGE_FRAGMENT_BIT, shader_program_.fragment_shader_source);
  std::vector<ShaderModule*> shader_modules{&vert_shader_module,
                                            &frag_shader_module};

  auto& pipeline_layout = resource_cache.RequestPipelineLayout(shader_modules);
  command_buffer.BindPipelineLayout(pipeline_layout);

  command_buffer.SetVertexInputState({});  // no vertex input

  // Set cull mode to front as the full screen triangle is clock-wise.
  RasterizationState rasterization_state{};
  rasterization_state.cull_mode = VK_CULL_MODE_FRONT_BIT;
  command_buffer.SetRasterizationState(rasterization_state);

  auto& target_views = render_frame.GetRenderTarget().GetViews();

  // Bind ssao at binding 0.
  command_buffer.BindImage(target_views.at(ssao_detachment_id_), *blur_sampler_,
                           0, 0, 0);

  command_buffer.Draw(3, 1, 0, 0);
}
}  // namespace vkoo
