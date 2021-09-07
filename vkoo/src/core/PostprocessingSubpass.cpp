#include "vkoo/core/PostprocessingSubpass.h"

namespace vkoo {
PostprocessingSubpass::PostprocessingSubpass(
    RenderContext& render_context, const st::ShaderProgram& shader_program,
    st::Scene& scene, uint32_t input_attachment_id)
    : Subpass{render_context, shader_program, scene},
      input_attachment_id_{input_attachment_id} {
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

  input_sampler_ = std::make_unique<vkoo::core::Sampler>(
      render_context.GetDevice(), sampler_info);
}

void PostprocessingSubpass::Prepare() {}

void PostprocessingSubpass::Draw(CommandBuffer& command_buffer) {
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

  // Enable alpha blending.
  ColorBlendAttachmentState color_blend_attachment{};
  color_blend_attachment.blend_enable = VK_TRUE;
  color_blend_attachment.src_color_blend_factor = VK_BLEND_FACTOR_SRC_ALPHA;
  color_blend_attachment.dst_color_blend_factor =
      VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  color_blend_attachment.src_alpha_blend_factor = VK_BLEND_FACTOR_ONE;
  color_blend_attachment.dst_alpha_blend_factor = VK_BLEND_FACTOR_ONE;

  ColorBlendState color_blend_state{};
  color_blend_state.attachments.resize(GetOutputAttachments().size());
  for (auto& it : color_blend_state.attachments) {
    it = color_blend_attachment;
  }
  command_buffer.SetColorBlendState(color_blend_state);

  auto& target_views = render_frame.GetRenderTarget().GetViews();

  // Bind input texture at binding 0.
  command_buffer.BindImage(target_views.at(input_attachment_id_),
                           *input_sampler_, 0, 0, 0);

  command_buffer.Draw(3, 1, 0, 0);
}
}  // namespace vkoo
