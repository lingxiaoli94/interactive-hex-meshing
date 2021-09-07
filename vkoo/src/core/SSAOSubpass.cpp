#include "vkoo/core/SSAOSubpass.h"

namespace vkoo {
SSAOSubpass::SSAOSubpass(RenderContext& render_context,
                         const st::ShaderProgram& shader_program,
                         st::Scene& scene, const core::Sampler& texture_sampler,
                         uint32_t depth_attachment_id,
                         uint32_t normal_attachment_id)
    : Subpass{render_context, shader_program, scene},
      texture_sampler_{texture_sampler},
      depth_attachment_id_{depth_attachment_id},
      normal_attachment_id_{normal_attachment_id} {
  std::default_random_engine rnd_engine((unsigned)time(nullptr));
  std::uniform_real_distribution<float> rnd_dist(0.0f, 1.0f);

  // Prepare SSAO kernel.
  std::vector<glm::vec4> ssao_kernel(kSSAOKernelSize);
  for (size_t i = 0; i < kSSAOKernelSize; i++) {
    glm::vec3 sample(rnd_dist(rnd_engine) * 2.0 - 1.0,
                     rnd_dist(rnd_engine) * 2.0 - 1.0,
                     rnd_dist(rnd_engine));  // hemisphere
    sample = glm::normalize(sample);
    sample *= rnd_dist(rnd_engine);
    ssao_kernel[i] = glm::vec4(sample, 1.0f);
  }
  ssao_kernel_buffer_ = std::make_unique<core::Buffer>(
      render_context.GetDevice(), ssao_kernel.size() * sizeof(glm::vec4),
      VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  ssao_kernel_buffer_->Update(
      reinterpret_cast<const uint8_t*>(ssao_kernel.data()),
      ssao_kernel.size() * sizeof(glm::vec4), 0);

  // Prepare SSAO noise texture.
  std::vector<glm::vec4> ssao_noise(kSSAONoiseDim * kSSAONoiseDim);
  for (size_t i = 0; i < ssao_noise.size(); i++) {
    ssao_noise[i] = glm::vec4(rnd_dist(rnd_engine) * 2.0 - 1.0,
                              rnd_dist(rnd_engine) * 2.0 - 1.0,
                              rnd_dist(rnd_engine) * 2.0 - 1.0, 0.0f);
  }

  st::Mipmap mipmap{};
  mipmap.extent.width = kSSAONoiseDim;
  mipmap.extent.height = kSSAONoiseDim;

  std::vector<uint8_t> image_data{
      reinterpret_cast<uint8_t*>(ssao_noise.data()),
      reinterpret_cast<uint8_t*>(ssao_noise.data()) +
          ssao_noise.size() * sizeof(glm::vec4)};
  ssao_noise_image_ = std::make_unique<st::Image>(
      std::move(image_data), std::vector<st::Mipmap>{mipmap},
      VK_FORMAT_R32G32B32A32_SFLOAT);
  ssao_noise_image_->CreateVkImageAndView(render_context.GetDevice(),
                                          VK_IMAGE_VIEW_TYPE_2D);
  ssao_noise_image_->UploadDataToGPU(render_context.GetDevice());

  // Create sampler.
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.anisotropyEnable = VK_TRUE;
  sampler_info.maxAnisotropy = 1.0f;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

  noise_sampler_ = std::make_unique<vkoo::core::Sampler>(
      render_context.GetDevice(), sampler_info);
}

void SSAOSubpass::Prepare() {}

void SSAOSubpass::Draw(CommandBuffer& command_buffer) {
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

  // Depth at binding 0.
  command_buffer.BindImage(target_views.at(depth_attachment_id_),
                           texture_sampler_, 0, 0, 0);
  // Normal at binding 1.
  command_buffer.BindImage(target_views.at(normal_attachment_id_),
                           texture_sampler_, 0, 1, 0);

  // Bind ssao noise image at binding 2.
  command_buffer.BindImage(ssao_noise_image_->GetVkImageView(), *noise_sampler_,
                           0, 2, 0);

  // Bind ssao kernel at binding 3.
  command_buffer.BindBuffer(*ssao_kernel_buffer_, 0,
                            ssao_kernel_buffer_->GetSize(), 0, 3, 0);

  CameraUniform camera_uniform;
  const st::Camera& camera = *scene_.GetActiveCameraPtr();
  glm::mat4 view_proj_mat =
      VulkanStyleProjection(camera.GetProjectionMatrix()) *
      camera.GetViewMatrix();
  camera_uniform.world_to_clip_mat = view_proj_mat;
  camera_uniform.clip_to_world_mat = glm::inverse(view_proj_mat);

  core::Buffer& buffer = render_frame.AllocateBuffer(
      VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(CameraUniform));
  buffer.ConvertAndUpdate(camera_uniform);

  // Bind camera uniform at binding 4.
  command_buffer.BindBuffer(buffer, 0, buffer.GetSize(), 0, 4, 0);

  command_buffer.Draw(3, 1, 0, 0);
}
}  // namespace vkoo
