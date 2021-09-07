#include "vkoo/core/DeferredLightingSubpass.h"

namespace vkoo {
DeferredLightingSubpass::DeferredLightingSubpass(
    RenderContext& render_context, const st::ShaderProgram& shader_program,
    st::Scene& scene, const core::Sampler& sampler,
    uint32_t depth_attachment_id, uint32_t albedo_attachment_id,
    uint32_t normal_attachment_id)
    : Subpass{render_context, shader_program, scene},
      sampler_{sampler},
      depth_attachment_id_{depth_attachment_id},
      albedo_attachment_id_{albedo_attachment_id},
      normal_attachment_id_{normal_attachment_id} {}

void DeferredLightingSubpass::Prepare() {}

void DeferredLightingSubpass::SetSSAOAttachment(uint32_t ssao_attachment_id) {
  ssao_attachment_id_ = ssao_attachment_id;
  ssao_enabled_ = true;

  shader_variant_.Clear();
  shader_variant_.AddDef("SSAO_ENABLED");
}

void DeferredLightingSubpass::Draw(CommandBuffer& command_buffer) {
  DeferredLights light_ubo;
  AggregateLights(light_ubo,
                  scene_.GetRoot().GetComponentsRecursive<st::Light>());
  auto& render_frame = render_context_.GetActiveFrame();
  auto& light_buffer = render_frame.AllocateBuffer(
      VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(DeferredLights));
  light_buffer.ConvertAndUpdate(light_ubo);

  // The lights' uniform parameter is always at binding = 5.
  command_buffer.BindBuffer(light_buffer, 0, light_buffer.GetSize(), 0, 5, 0);

  auto& resource_cache = command_buffer.GetDevice().GetResourceCache();
  auto& vert_shader_module = resource_cache.RequestShaderModule(
      VK_SHADER_STAGE_VERTEX_BIT, shader_program_.vertex_shader_source,
      shader_variant_);
  auto& frag_shader_module = resource_cache.RequestShaderModule(
      VK_SHADER_STAGE_FRAGMENT_BIT, shader_program_.fragment_shader_source,
      shader_variant_);
  std::vector<ShaderModule*> shader_modules{&vert_shader_module,
                                            &frag_shader_module};

  auto& pipeline_layout = resource_cache.RequestPipelineLayout(shader_modules);
  command_buffer.BindPipelineLayout(pipeline_layout);

  command_buffer.SetVertexInputState({});  // no vertex input

  auto& render_target = render_context_.GetActiveFrame().GetRenderTarget();
  auto& target_views = render_target.GetViews();

  // Bind depth, albedo, and normal as input attachments.
  auto& depth_view = target_views.at(depth_attachment_id_);
  command_buffer.BindImage(depth_view, sampler_, 0, 0, 0);

  auto& albedo_view = target_views.at(albedo_attachment_id_);
  command_buffer.BindImage(albedo_view, sampler_, 0, 1, 0);

  auto& normal_view = target_views.at(normal_attachment_id_);
  command_buffer.BindImage(normal_view, sampler_, 0, 2, 0);

  if (ssao_enabled_) {
    auto& ssao_view = target_views.at(ssao_attachment_id_);
    command_buffer.BindImage(ssao_view, sampler_, 0, 3, 0);
  }

  // Set cull mode to front as full screen triangle is clock-wise.
  RasterizationState rasterization_state{};
  rasterization_state.cull_mode = VK_CULL_MODE_FRONT_BIT;
  command_buffer.SetRasterizationState(rasterization_state);

  CameraUniform camera_uniform;
  camera_uniform.inv_resolution.x =
      1.0f / static_cast<float>(render_target.GetExtent().width);
  camera_uniform.inv_resolution.y =
      1.0f / static_cast<float>(render_target.GetExtent().height);

  const st::Camera& camera = *scene_.GetActiveCameraPtr();
  glm::mat4 view_proj_mat =
      VulkanStyleProjection(camera.GetProjectionMatrix()) *
      camera.GetViewMatrix();
  camera_uniform.clip_to_world_mat = glm::inverse(view_proj_mat);

  core::Buffer& buffer = render_frame.AllocateBuffer(
      VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(CameraUniform));
  buffer.ConvertAndUpdate(camera_uniform);

  // Bind camera uniform at binding 4.
  command_buffer.BindBuffer(buffer, 0, buffer.GetSize(), 0, 4, 0);

  command_buffer.Draw(3, 1, 0, 0);
}
}  // namespace vkoo
