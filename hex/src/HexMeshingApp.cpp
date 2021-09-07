#include "HexMeshingApp.h"

// FIXME: put stb implementation somewhere else.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include <vkoo/ObjParser.h>
#include <vkoo/core/DeferredLightingSubpass.h>
#include <vkoo/core/ForwardSubpass.h>
#include <vkoo/core/Gui.h>
#include <vkoo/core/InputEvent.h>
#include <vkoo/core/PostprocessingSubpass.h>
#include <vkoo/core/SSAOBlurSubpass.h>
#include <vkoo/core/SSAOSubpass.h>
#include <vkoo/core/VertexObject.h>
#include <vkoo/st/components/Light.h>
#include <vkoo/st/scripts/ArcBallCameraScript.h>
#include <vkoo/utils.h>

#include <chrono>

#include "logging.h"
#include "models/HexConvention.h"
#include "utility/PathManager.h"

namespace hex {
HexMeshingApp::HexMeshingApp(bool enable_validation_layer)
    : Application(enable_validation_layer),
      settings_{PathManager::GetWorkspacePath() / "config.yml", 60} {
  torch::manual_seed(42);
  starter_width_ = settings_.window_width;
  starter_height_ = settings_.window_height;
}

HexMeshingApp::~HexMeshingApp() {
  // device_->WaitIdle();
}

void HexMeshingApp::Prepare() {
  Application::Prepare();

  // Initialize HexConvention early to prevent race conditions caused by lazy
  // intialization.
  HexConvention::Initialize();

  CreateSampler();  // this also creates sampler
  SetupScene();
  SetupRenderPipelines();  // this should happen after samplers are created
  gui_ = std::make_unique<vkoo::Gui>(*this, 20.0f);
}

void HexMeshingApp::MainLoop() {
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint =
      std::chrono::time_point<Clock, std::chrono::duration<double>>;
  TimePoint last_tick_time = Clock::now();
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    TimePoint current_tick_time = Clock::now();

    if (settings_.fps_limit > 0) {
      if ((current_tick_time - last_tick_time).count() <
          1.0 / settings_.fps_limit) {
        std::this_thread::sleep_until(last_tick_time +
                                      std::chrono::milliseconds((int)std::round(
                                          1000.0 / settings_.fps_limit)));
        current_tick_time = Clock::now();
      }
    }
    double delta_time = (current_tick_time - last_tick_time).count();
    last_tick_time = current_tick_time;

    Update(delta_time);
    settings_.TrySave();
  }

  settings_.Save();
}

Settings& HexMeshingApp::GetSettings() { return settings_; }

void HexMeshingApp::ReloadSettings(const std::string& filename) {
  settings_.Load(filename);
  ChangeWindowSize(settings_.window_width, settings_.window_height);
  UpdateCameraAndLighting();
}

void HexMeshingApp::SaveSettings(const std::string& filename) {
  settings_.Save(filename);
}

void HexMeshingApp::SetupScene() {
  auto root = std::make_unique<vkoo::st::Node>();
  auto camera_node = std::make_unique<vkoo::st::Node>();

  // Create camera. The view matrix will be governed by arc ball camera script.
  auto& camera = camera_node->CreateComponent<vkoo::st::Camera>(
      45.0f, GetWindowAspectRatio(), 0.1f, 100.f);

  root->AddChild(std::move(camera_node));
  // Scene must be initialized before controller due to internal dependency.
  scene_ = std::make_unique<vkoo::st::Scene>(std::move(root));
  scene_->SetActiveCameraPtr(&camera);

  RecreateLights();

  uint32_t window_width;
  uint32_t window_height;
  GetWindowSize(window_width, window_height);
  global_controller_ =
      std::make_unique<GlobalController>(*this, window_width, window_height);
}

void HexMeshingApp::RecreateLights() {
  // Remove existing lights.
  auto lights = scene_->GetRoot().GetComponentsRecursive<vkoo::st::Light>();
  for (auto light : lights) {
    light->GetNode()->RemoveFromParent();
  }
  // Add in lights.
  for (auto& spec : settings_.lights) {
    auto light_node = std::make_unique<vkoo::st::Node>();
    auto& light_comp = light_node->CreateComponent<vkoo::st::Light>();
    light_comp.SetType(spec.type);
    light_comp.SetProperties(spec.properties);
    scene_->GetRoot().AddChild(std::move(light_node));
  }
}

void HexMeshingApp::UpdateCameraAndLighting() {
  global_controller_->InitArcBallCamera();
  RecreateLights();
}

void HexMeshingApp::CreateSampler() {
  VkSamplerCreateInfo sampler_info{};
  sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  sampler_info.magFilter = VK_FILTER_LINEAR;
  sampler_info.minFilter = VK_FILTER_LINEAR;
  sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  sampler_info.anisotropyEnable = VK_TRUE;
  sampler_info.maxAnisotropy =
      device_->GetGPU().GetProperties().limits.maxSamplerAnisotropy;
  sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  sampler_info.unnormalizedCoordinates = VK_FALSE;
  sampler_info.compareEnable = VK_FALSE;
  sampler_info.compareOp = VK_COMPARE_OP_ALWAYS;
  sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

  sampler_ = std::make_unique<vkoo::core::Sampler>(*device_, sampler_info);
}

bool HexMeshingApp::IsMSAAEnabled() const {
  return settings_.msaa_on && sample_count_ != VK_SAMPLE_COUNT_1_BIT;
}

void HexMeshingApp::SetupGBufferPipeline() {
  auto gbuffer_shader_program = vkoo::st::ShaderProgram{
      {vkoo::GetShaderPath() + "/deferred_geometry.vert", "main"},
      {vkoo::GetShaderPath() + "/deferred_geometry.frag", "main"}};
  auto gbuffer_subpass = std::make_unique<vkoo::GeometrySubpass>(
      *render_context_, gbuffer_shader_program, *scene_,
      vkoo::TransparencyMode::SolidOnly);

  std::vector<VkClearValue> clear_value;
  if (IsMSAAEnabled()) {
    pipeline_infos.gbuffer_generation.attachment_ids = {
        attachment_dict.depth_ms,  attachment_dict.depth_resolved,
        attachment_dict.albedo_ms, attachment_dict.albedo_resolved,
        attachment_dict.normal_ms, attachment_dict.normal_resolved};
    gbuffer_subpass->SetOutputAttachments({2, 4});
    gbuffer_subpass->SetColorResolveAttachments({3, 5});
    gbuffer_subpass->SetDepthStencilResolveAttachment(1);
    gbuffer_subpass->SetDepthStencilResolveMode(depth_resolve_mode_);
    gbuffer_subpass->SetSampleCount(sample_count_);
    clear_value = std::vector<VkClearValue>(6);
    clear_value[0].depthStencil = {1.0f, ~0U};
    clear_value[1].depthStencil = {1.0f, ~0U};
    clear_value[2].color = background_clear_color_;
    clear_value[3].color = background_clear_color_;
    clear_value[4].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
    clear_value[5].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
  } else {
    pipeline_infos.gbuffer_generation.attachment_ids = {
        attachment_dict.depth_resolved, attachment_dict.albedo_resolved,
        attachment_dict.normal_resolved};
    gbuffer_subpass->SetOutputAttachments({1, 2});
    clear_value = std::vector<VkClearValue>(3);
    clear_value[0].depthStencil = {1.0f, ~0U};
    clear_value[1].color = background_clear_color_;
    clear_value[2].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
  }

  gbuffer_subpass->SetDisableDepthStencilAttachment(false);

  auto gbuffer_pipeline = std::make_unique<vkoo::RenderPipeline>();
  gbuffer_pipeline->AddSubpass(std::move(gbuffer_subpass));

  gbuffer_pipeline->SetClearValue(clear_value);

  pipeline_infos.gbuffer_generation.pipeline = std::move(gbuffer_pipeline);

  std::vector<AttachmentInfo> attachment_infos;

  if (IsMSAAEnabled()) {
    attachment_infos = std::vector<AttachmentInfo>{6};
    for (size_t i = 0; i < attachment_infos.size(); i++) {
      auto& info = attachment_infos[i];
      if (i == 0 || i == 2 || i == 4) {
        info.load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
        info.store_op = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        info.final_layout =
            i == 0 ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
                   : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
      } else {
        info.load_op = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        info.store_op = VK_ATTACHMENT_STORE_OP_STORE;
        info.final_layout =
            i == 1 ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
                   : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
      info.initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    }
  } else {
    attachment_infos = std::vector<AttachmentInfo>{3};
    for (size_t i = 0; i < attachment_infos.size(); i++) {
      attachment_infos[i].load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
      attachment_infos[i].store_op = VK_ATTACHMENT_STORE_OP_STORE;
      attachment_infos[i].initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
      attachment_infos[i].final_layout =
          i == 0 ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
                 : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    }
  }
  pipeline_infos.gbuffer_generation.attachment_infos = attachment_infos;

  std::vector<VkSubpassDependency2KHR> dependencies{2};
  dependencies[0].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass = 0;
  dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
  dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  dependencies[1].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[1].srcSubpass = 0;
  dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
  dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  pipeline_infos.gbuffer_generation.dependencies = dependencies;
}

void HexMeshingApp::SetupSSAOPipeline() {
  pipeline_infos.ssao.attachment_ids = {attachment_dict.ssao};
  auto ssao_shader_program = vkoo::st::ShaderProgram{
      {vkoo::GetShaderPath() + "/offscreen.vert", "main"},
      {vkoo::GetShaderPath() + "/deferred_ssao.frag", "main"}};

  auto ssao_subpass = std::make_unique<vkoo::SSAOSubpass>(
      *render_context_, ssao_shader_program, *scene_, *sampler_,
      attachment_dict.depth_resolved, attachment_dict.normal_resolved);
  ssao_subpass->SetDisableDepthStencilAttachment(true);
  ssao_subpass->SetOutputAttachments({0});

  auto ssao_pipeline = std::make_unique<vkoo::RenderPipeline>();
  ssao_pipeline->AddSubpass(std::move(ssao_subpass));

  std::vector<VkClearValue> clear_value{1};
  clear_value[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};

  ssao_pipeline->SetClearValue(clear_value);

  pipeline_infos.ssao.pipeline = std::move(ssao_pipeline);
  std::vector<AttachmentInfo> attachment_infos{1};
  attachment_infos[0].load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachment_infos[0].store_op = VK_ATTACHMENT_STORE_OP_STORE;
  attachment_infos[0].initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachment_infos[0].final_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  pipeline_infos.ssao.attachment_infos = attachment_infos;

  std::vector<VkSubpassDependency2KHR> dependencies{2};
  dependencies[0].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass = 0;
  dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[0].srcAccessMask =
      VK_ACCESS_SHADER_READ_BIT;  // no need to include write here; this is
                                  // taken care of by GBuffer's dependencies.
  dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  dependencies[1].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[1].srcSubpass = 0;
  dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
  dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  pipeline_infos.ssao.dependencies = dependencies;
}

void HexMeshingApp::SetupSSAOBlurPipeline() {
  pipeline_infos.ssao_blur.attachment_ids = {attachment_dict.ssao_blur};
  auto shader_program = vkoo::st::ShaderProgram{
      {vkoo::GetShaderPath() + "/offscreen.vert", "main"},
      {vkoo::GetShaderPath() + "/deferred_ssao_blur.frag", "main"}};

  auto subpass = std::make_unique<vkoo::SSAOBlurSubpass>(
      *render_context_, shader_program, *scene_, attachment_dict.ssao);
  subpass->SetDisableDepthStencilAttachment(true);
  subpass->SetOutputAttachments({0});

  auto pipeline = std::make_unique<vkoo::RenderPipeline>();
  pipeline->AddSubpass(std::move(subpass));

  std::vector<VkClearValue> clear_value{1};
  clear_value[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};

  pipeline->SetClearValue(clear_value);

  pipeline_infos.ssao_blur.pipeline = std::move(pipeline);
  std::vector<AttachmentInfo> attachment_infos{1};
  attachment_infos[0].load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachment_infos[0].store_op = VK_ATTACHMENT_STORE_OP_STORE;
  attachment_infos[0].initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachment_infos[0].final_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  pipeline_infos.ssao_blur.attachment_infos = attachment_infos;

  std::vector<VkSubpassDependency2KHR> dependencies{2};
  dependencies[0].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass = 0;
  dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
  dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  dependencies[1].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[1].srcSubpass = 0;
  dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
  dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  pipeline_infos.ssao_blur.dependencies = dependencies;
}

void HexMeshingApp::SetupLightingPipeline() {
  pipeline_infos.lighting.attachment_ids = {attachment_dict.swapchain};
  auto lighting_shader_program = vkoo::st::ShaderProgram{
      {vkoo::GetShaderPath() + "/offscreen.vert", "main"},
      {vkoo::GetShaderPath() + "/deferred_lighting.frag", "main"}};

  auto lighting_subpass = std::make_unique<vkoo::DeferredLightingSubpass>(
      *render_context_, lighting_shader_program, *scene_, *sampler_,
      attachment_dict.depth_resolved, attachment_dict.albedo_resolved,
      attachment_dict.normal_resolved);

  if (settings_.ssao_on) {
    lighting_subpass->SetSSAOAttachment(attachment_dict.ssao_blur);
  }
  // No depth attachment for lighting pass.
  lighting_subpass->SetDisableDepthStencilAttachment(true);

  lighting_subpass->SetOutputAttachments({0});

  auto lighting_pipeline = std::make_unique<vkoo::RenderPipeline>();
  lighting_pipeline->AddSubpass(std::move(lighting_subpass));

  std::vector<VkClearValue> clear_value{1};
  clear_value[0].color = {
      {0.0f, 0.0f, 0.0f, 0.0f}};  // this value does not matter

  lighting_pipeline->SetClearValue(clear_value);

  pipeline_infos.lighting.pipeline = std::move(lighting_pipeline);
  std::vector<AttachmentInfo> attachment_infos{1};
  attachment_infos[0].load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachment_infos[0].store_op = VK_ATTACHMENT_STORE_OP_STORE;
  attachment_infos[0].initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  attachment_infos[0].final_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  pipeline_infos.lighting.attachment_infos = attachment_infos;

  std::vector<VkSubpassDependency2KHR> dependencies{2};
  dependencies[0].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass = 0;
  dependencies[0].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  dependencies[1].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[1].srcSubpass = 0;
  dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  pipeline_infos.lighting.dependencies = dependencies;
}

void HexMeshingApp::SetupTransparentPipeline() {
  auto transparent_shader_program = vkoo::st::ShaderProgram{
      {vkoo::GetShaderPath() + "/deferred_geometry.vert", "main"},
      {vkoo::GetShaderPath() + "/deferred_transparent.frag", "main"}};
  auto transparent_subpass = std::make_unique<vkoo::GeometrySubpass>(
      *render_context_, transparent_shader_program, *scene_,
      vkoo::TransparencyMode::TransparentOnly);

  std::vector<VkClearValue> clear_value;
  if (IsMSAAEnabled()) {
    pipeline_infos.transparent.attachment_ids = {
        attachment_dict.transparent_resolved, attachment_dict.transparent_ms,
        attachment_dict.depth_ms};
    transparent_subpass->SetOutputAttachments({1});
    transparent_subpass->SetColorResolveAttachments({0});
    transparent_subpass->SetSampleCount(sample_count_);
    clear_value = std::vector<VkClearValue>(3);
    clear_value[0].color = {{0.0f, 0.0f, 0.0f, 0.0f}};
    clear_value[1].color = {{0.0f, 0.0f, 0.0f, 0.0f}};
    clear_value[2].depthStencil = {1.0f, ~0U};
  } else {
    pipeline_infos.transparent.attachment_ids = {
        attachment_dict.transparent_resolved, attachment_dict.depth_resolved};
    transparent_subpass->SetOutputAttachments({0});
    clear_value = std::vector<VkClearValue>(2);
    clear_value[0].color = {{0.0f, 0.0f, 0.0f, 0.0f}};
    clear_value[1].depthStencil = {1.0f, ~0U};
  }
  transparent_subpass->SetDisableDepthStencilAttachment(false);

  auto transparent_pipeline = std::make_unique<vkoo::RenderPipeline>();
  transparent_pipeline->AddSubpass(std::move(transparent_subpass));
  transparent_pipeline->SetClearValue(clear_value);

  pipeline_infos.transparent.pipeline = std::move(transparent_pipeline);

  std::vector<AttachmentInfo> attachment_infos;

  if (IsMSAAEnabled()) {
    attachment_infos = std::vector<AttachmentInfo>{3};
    attachment_infos[0].load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachment_infos[0].store_op = VK_ATTACHMENT_STORE_OP_STORE;
    attachment_infos[0].initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachment_infos[0].final_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    attachment_infos[1].load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachment_infos[1].store_op = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment_infos[1].initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachment_infos[1].final_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    attachment_infos[2].load_op = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachment_infos[2].store_op = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment_infos[2].initial_layout =
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
    attachment_infos[2].final_layout =
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
  } else {
    attachment_infos = std::vector<AttachmentInfo>{2};
    attachment_infos[0].load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachment_infos[0].store_op = VK_ATTACHMENT_STORE_OP_STORE;
    attachment_infos[0].initial_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachment_infos[0].final_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    attachment_infos[1].load_op = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachment_infos[1].store_op = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment_infos[1].initial_layout =
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
    attachment_infos[1].final_layout =
        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
  }
  pipeline_infos.transparent.attachment_infos = attachment_infos;

  std::vector<VkSubpassDependency2KHR> dependencies{2};
  dependencies[0].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass = 0;
  dependencies[0].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

  dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  dependencies[1].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[1].srcSubpass = 0;
  dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
  dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
  dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
  pipeline_infos.transparent.dependencies = dependencies;
}

void HexMeshingApp::SetupPostprocessingPipeline() {
  // For now the purpose of this is to write transparent part of the scene to
  // the swapchain, plus GUI.
  pipeline_infos.postprocessing.attachment_ids = {attachment_dict.swapchain};
  auto shader_program = vkoo::st::ShaderProgram{
      {vkoo::GetShaderPath() + "/offscreen.vert", "main"},
      {vkoo::GetShaderPath() + "/postprocessing.frag", "main"}};

  auto subpass = std::make_unique<vkoo::PostprocessingSubpass>(
      *render_context_, shader_program, *scene_,
      attachment_dict.transparent_resolved);
  subpass->SetDisableDepthStencilAttachment(true);
  subpass->SetOutputAttachments({0});

  auto pipeline = std::make_unique<vkoo::RenderPipeline>();
  pipeline->AddSubpass(std::move(subpass));

  std::vector<VkClearValue> clear_value{1};
  clear_value[0].color = {{0.0f, 0.0f, 0.0f, 0.0f}};

  pipeline->SetClearValue(clear_value);

  pipeline_infos.postprocessing.pipeline = std::move(pipeline);
  std::vector<AttachmentInfo> attachment_infos{1};
  attachment_infos[0].load_op = VK_ATTACHMENT_LOAD_OP_LOAD;
  attachment_infos[0].store_op = VK_ATTACHMENT_STORE_OP_STORE;
  attachment_infos[0].initial_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  attachment_infos[0].final_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  pipeline_infos.postprocessing.attachment_infos = attachment_infos;

  std::vector<VkSubpassDependency2KHR> dependencies{2};
  dependencies[0].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[0].dstSubpass = 0;
  dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
  dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  dependencies[1].sType = VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR;
  dependencies[1].srcSubpass = 0;
  dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
  dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                  VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
  dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

  pipeline_infos.postprocessing.dependencies = dependencies;
}

void HexMeshingApp::SetupRenderPipelines() {
  SetupGBufferPipeline();
  if (settings_.ssao_on) {
    SetupSSAOPipeline();
    SetupSSAOBlurPipeline();
  }
  SetupLightingPipeline();
  SetupTransparentPipeline();
  SetupPostprocessingPipeline();
}

void HexMeshingApp::DrawGui() { global_controller_->DrawGui(); }

void HexMeshingApp::PrepareRenderContext() {
  // Create a render target for each swapchain image.
  // Note this will be called whenever swapchain is being recreated.
  render_context_->Prepare([this](vkoo::core::Image&& swapchain_image) {
    return CreateRenderTarget(std::move(swapchain_image));
  });
}

std::unique_ptr<vkoo::RenderTarget> HexMeshingApp::CreateRenderTarget(
    vkoo::core::Image&& swapchain_image) {
  if (supported_sample_count_list_.empty()) {
    PrepareSupportedSampleCountList();
  }
  if (supported_depth_resolve_mode_list_.empty()) {
    PrepareDepthResolveModeList();
  }

  auto& device = swapchain_image.GetDevice();
  auto& extent = swapchain_image.GetExtent();

  std::vector<vkoo::core::Image> images;
  attachment_dict.swapchain = images.size();
  images.push_back(std::move(swapchain_image));

  auto depth_format = vkoo::GetSuitableDepthFormat(device.GetGPU().GetHandle());

  if (IsMSAAEnabled()) {
    vkoo::core::Image depth_ms_image{
        device, extent, depth_format,
        VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
            VK_IMAGE_USAGE_SAMPLED_BIT,
        // VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, sample_count_};
    attachment_dict.depth_ms = images.size();
    images.push_back(std::move(depth_ms_image));

    vkoo::core::Image albedo_ms_image{
        device,
        extent,
        VK_FORMAT_R32G32B32A32_SFLOAT,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
            VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        sample_count_};
    attachment_dict.albedo_ms = images.size();
    images.push_back(std::move(albedo_ms_image));

    vkoo::core::Image normal_ms_image{
        device,
        extent,
        VK_FORMAT_R32G32B32A32_SFLOAT,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
            VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        sample_count_};
    attachment_dict.normal_ms = images.size();
    images.push_back(std::move(normal_ms_image));

    vkoo::core::Image transparent_ms_image{
        device,
        extent,
        VK_FORMAT_R32G32B32A32_SFLOAT,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
            VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        sample_count_};
    attachment_dict.transparent_ms = images.size();
    images.push_back(std::move(transparent_ms_image));
  }
  vkoo::core::Image depth_resolved_image{
      device, extent, depth_format,
      VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT};

  vkoo::core::Image albedo_resolved_image{
      device, extent, VK_FORMAT_R32G32B32A32_SFLOAT,
      VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT};

  vkoo::core::Image normal_resolved_image{
      device, extent, VK_FORMAT_R32G32B32A32_SFLOAT,
      VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT};

  vkoo::core::Image transparent_resolved_image{
      device, extent, VK_FORMAT_R32G32B32A32_SFLOAT,
      VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT};

  attachment_dict.depth_resolved = images.size();
  images.push_back(std::move(depth_resolved_image));
  attachment_dict.albedo_resolved = images.size();
  images.push_back(std::move(albedo_resolved_image));
  attachment_dict.normal_resolved = images.size();
  images.push_back(std::move(normal_resolved_image));
  attachment_dict.transparent_resolved = images.size();
  images.push_back(std::move(transparent_resolved_image));

  if (settings_.ssao_on) {
    vkoo::core::Image ssao_image{
        device, extent, VK_FORMAT_R32_SFLOAT,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT};

    vkoo::core::Image ssao_blur_image{
        device, extent, VK_FORMAT_R32_SFLOAT,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT};

    attachment_dict.ssao = images.size();
    images.push_back(std::move(ssao_image));
    attachment_dict.ssao_blur = images.size();
    images.push_back(std::move(ssao_blur_image));
  }

  return std::make_unique<vkoo::RenderTarget>(std::move(images));
}

void HexMeshingApp::DrawRenderPasses(vkoo::CommandBuffer& command_buffer,
                                     vkoo::RenderTarget& render_target) {
  auto& extent = render_target.GetExtent();
  SetViewPortAndScissor(command_buffer, extent);
  // Draw G-buffer pipeline first.
  DrawPipeline(command_buffer, render_target,
               pipeline_infos.gbuffer_generation);
  command_buffer.EndRenderPass();

  if (settings_.ssao_on) {
    command_buffer.Reset();
    // Draw SSAO.
    SetViewPortAndScissor(command_buffer, extent);
    DrawPipeline(command_buffer, render_target, pipeline_infos.ssao);
    command_buffer.EndRenderPass();

    command_buffer.Reset();
    // Draw SSAO blur.
    SetViewPortAndScissor(command_buffer, extent);
    DrawPipeline(command_buffer, render_target, pipeline_infos.ssao_blur);
    command_buffer.EndRenderPass();
  }

  command_buffer.Reset();
  // Draw lighting pipeline.
  SetViewPortAndScissor(command_buffer, extent);
  DrawPipeline(command_buffer, render_target, pipeline_infos.lighting);

  command_buffer.EndRenderPass();

  command_buffer.Reset();
  // Draw transparent pipeline.
  SetViewPortAndScissor(command_buffer, extent);
  DrawPipeline(command_buffer, render_target, pipeline_infos.transparent);
  command_buffer.EndRenderPass();

  command_buffer.Reset();
  // Draw postprocessing pipeline.
  SetViewPortAndScissor(command_buffer, extent);
  DrawPipeline(command_buffer, render_target, pipeline_infos.postprocessing);

  // Draw GUI in the end in which we require only 1 color attachment that
  // corresponds to the swapchain in the render pass.
  if (gui_ && settings_.show_gui) {
    gui_->Draw(command_buffer);
  }
  command_buffer.EndRenderPass();
}

void HexMeshingApp::DrawPipeline(vkoo::CommandBuffer& command_buffer,
                                 vkoo::RenderTarget& render_target,
                                 const PipelineInfo& pipeline_info) {
  auto& attachment_ids = pipeline_info.attachment_ids;
  auto& attachment_infos = pipeline_info.attachment_infos;
  auto& dependencies = pipeline_info.dependencies;

  auto& extent = render_target.GetExtent();
  auto& views = render_target.GetViews();

  std::vector<VkImageView> attachments;
  std::vector<VkAttachmentDescription2KHR> attachment_descriptions;

  for (size_t i = 0; i < attachment_ids.size(); i++) {
    auto& view = views.at(attachment_ids[i]);
    auto& image = view.GetImage();
    attachments.push_back(view.GetHandle());

    VkAttachmentDescription2KHR description{};
    description.sType = VK_STRUCTURE_TYPE_ATTACHMENT_DESCRIPTION_2_KHR;
    description.samples = image.GetSampleCount();
    description.format = image.GetFormat();
    description.initialLayout = attachment_infos[i].initial_layout;
    description.finalLayout = attachment_infos[i].final_layout;
    description.loadOp = attachment_infos[i].load_op;
    description.storeOp = attachment_infos[i].store_op;
    description.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    description.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment_descriptions.push_back(description);
  }

  auto forced_dependencies = dependencies;
  pipeline_info.pipeline->Draw(command_buffer, extent, attachments,
                               attachment_descriptions, dependencies);
}

void HexMeshingApp::UpdateScene(float delta_time) {
  global_controller_->Update(delta_time);
}

void HexMeshingApp::HandleInputEvent(const vkoo::InputEvent& event) {
  bool captured_by_gui = false;
  if (gui_) {
    captured_by_gui = gui_->HandleInputEvent(event);
  }
  if (!captured_by_gui) {
    // Camera handling will be called within global_script_.
    global_controller_->HandleInputEvent(event);
  }
}

void HexMeshingApp::FramebufferSizeCallback(int width, int height) {
  if (gui_) {
    gui_->Resize(width, height);
  }
  global_controller_->OnWindowResize(width, height);
  settings_.window_width = width;
  settings_.window_height = height;
}

void HexMeshingApp::PrepareSupportedSampleCountList() {
  VkPhysicalDeviceProperties gpu_properties;
  vkGetPhysicalDeviceProperties(device_->GetGPU().GetHandle(), &gpu_properties);

  VkSampleCountFlags supported_by_depth_and_color =
      gpu_properties.limits.framebufferColorSampleCounts &
      gpu_properties.limits.framebufferDepthSampleCounts;

  std::vector<VkSampleCountFlagBits> counts = {
      VK_SAMPLE_COUNT_4_BIT,  VK_SAMPLE_COUNT_2_BIT,  VK_SAMPLE_COUNT_8_BIT,
      VK_SAMPLE_COUNT_16_BIT, VK_SAMPLE_COUNT_32_BIT, VK_SAMPLE_COUNT_64_BIT,
      VK_SAMPLE_COUNT_1_BIT};

  for (auto& count : counts) {
    if (supported_by_depth_and_color & count) {
      supported_sample_count_list_.push_back(count);

      if (sample_count_ == VK_SAMPLE_COUNT_1_BIT) {
        sample_count_ = count;
        LOGI("Preferred MSAA sample count: {}", sample_count_);
      }
    }
  }
}

void HexMeshingApp::PrepareDepthResolveModeList() {
  VkPhysicalDeviceProperties2KHR gpu_properties{};
  gpu_properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2_KHR;
  VkPhysicalDeviceDepthStencilResolvePropertiesKHR depth_resolve_properties{};
  depth_resolve_properties.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DEPTH_STENCIL_RESOLVE_PROPERTIES_KHR;
  gpu_properties.pNext = static_cast<void*>(&depth_resolve_properties);

  auto get_properties_fn =
      (PFN_vkGetPhysicalDeviceProperties2KHR)vkGetInstanceProcAddr(
          instance_->GetHandle(), "vkGetPhysicalDeviceProperties2KHR");
  if (!get_properties_fn) {
    throw std::runtime_error(
        "Physical device properties 2 extension not availabe/enabled!");
  }
  get_properties_fn(device_->GetGPU().GetHandle(), &gpu_properties);

  if (depth_resolve_properties.supportedDepthResolveModes == 0) {
    throw std::runtime_error("No depth stencil resolve modes supported!");
  } else {
    std::vector<VkResolveModeFlagBits> modes = {
        VK_RESOLVE_MODE_SAMPLE_ZERO_BIT, VK_RESOLVE_MODE_MIN_BIT,
        VK_RESOLVE_MODE_MAX_BIT, VK_RESOLVE_MODE_AVERAGE_BIT};

    for (auto& mode : modes) {
      if (depth_resolve_properties.supportedDepthResolveModes & mode) {
        supported_depth_resolve_mode_list_.push_back(mode);

        if (depth_resolve_mode_ == VK_RESOLVE_MODE_NONE) {
          depth_resolve_mode_ = mode;
        }
      }
    }
  }
}

void HexMeshingApp::UpdatePipelines() {
  device_->WaitIdle();
  render_context_->UpdateSwapchain();
  device_->WaitIdle();
  // Pipeline setup must be after render target creations!
  SetupRenderPipelines();
}

void HexMeshingApp::SaveScreenshot(const std::string& filename) {
  LOGI("Saving screenshot ...");
  device_->WaitIdle();

  VkImage src_image_vk = GetRenderContext().GetLastActiveSwapchainImage();

  VkExtent2D extent{};
  GetWindowSize(extent.width, extent.height);
  VkImageSubresourceRange subresource_range{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0,
                                            1};
  vkoo::core::Image dst_image{*device_,
                              extent,
                              VK_FORMAT_R8G8B8A8_UNORM,
                              VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                  VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                              VK_SAMPLE_COUNT_1_BIT,
                              VK_IMAGE_TILING_LINEAR};

  vkoo::CommandBuffer& cmd = device_->RequestCommandBuffer();
  cmd.Begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
  {
    vkoo::ImageMemoryBarrier barrier;
    barrier.src_access_mask = 0;
    barrier.dst_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.new_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    cmd.InsertImageMemoryBarrier(dst_image.GetHandle(), barrier,
                                 subresource_range);
  }
  {
    vkoo::ImageMemoryBarrier barrier;
    barrier.src_access_mask = VK_ACCESS_MEMORY_READ_BIT;
    barrier.dst_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    barrier.new_layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    cmd.InsertImageMemoryBarrier(src_image_vk, barrier, subresource_range);
  }
  {
    VkImageCopy copy_region{};
    copy_region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_region.srcSubresource.layerCount = 1;
    copy_region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    copy_region.dstSubresource.layerCount = 1;
    copy_region.extent.width = extent.width;
    copy_region.extent.height = extent.height;
    copy_region.extent.depth = 1;

    vkCmdCopyImage(cmd.GetHandle(), src_image_vk,
                   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, dst_image.GetHandle(),
                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy_region);
  }
  {
    vkoo::ImageMemoryBarrier barrier;
    barrier.src_access_mask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dst_access_mask = VK_ACCESS_MEMORY_READ_BIT;
    barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_GENERAL;
    cmd.InsertImageMemoryBarrier(dst_image.GetHandle(), barrier,
                                 subresource_range);
  }
  {
    // Transition swapchain image back.
    vkoo::ImageMemoryBarrier barrier;
    barrier.src_access_mask = VK_ACCESS_TRANSFER_READ_BIT;
    barrier.dst_access_mask = VK_ACCESS_MEMORY_READ_BIT;
    barrier.src_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.dst_stage_mask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    barrier.old_layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.new_layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    cmd.InsertImageMemoryBarrier(src_image_vk, barrier, subresource_range);
  }
  cmd.End();

  auto& queue = device_->GetQueueByFlags(VK_QUEUE_GRAPHICS_BIT, 0);

  queue.Submit(cmd, device_->RequestFence());

  device_->GetFencePool().Wait();
  device_->GetFencePool().Reset();
  device_->GetCommandPool().ResetPool();
  device_->WaitIdle();

  VkImageSubresource sub_resource{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
  VkSubresourceLayout sub_resource_layout;
  vkGetImageSubresourceLayout(device_->GetHandle(), dst_image.GetHandle(),
                              &sub_resource, &sub_resource_layout);

  const char* data;
  vkMapMemory(device_->GetHandle(), dst_image.GetMemory(), 0, VK_WHOLE_SIZE, 0,
              (void**)&data);
  data += sub_resource_layout.offset;

  bool color_swizzle = false;
  // Check if source is BGR
  // Note: Not complete, only contains most common and basic BGR surface formats
  // for demonstration purposes.
  {
    std::vector<VkFormat> formatsBGR = {VK_FORMAT_B8G8R8A8_SRGB,
                                        VK_FORMAT_B8G8R8A8_UNORM,
                                        VK_FORMAT_B8G8R8A8_SNORM};
    auto itr = std::find(formatsBGR.begin(), formatsBGR.end(),
                         GetRenderContext().GetSwapchain().GetFormat());
    color_swizzle = (itr != formatsBGR.end());
  }

  std::vector<uint8_t> pixels;
  for (uint32_t y = 0; y < extent.height; y++) {
    unsigned int* row = (unsigned int*)data;
    for (uint32_t x = 0; x < extent.width; x++) {
      if (color_swizzle) {
        pixels.push_back(*((uint8_t*)row + 2));
        pixels.push_back(*((uint8_t*)row + 1));
        pixels.push_back(*((uint8_t*)row + 0));
        pixels.push_back(*((uint8_t*)row + 3));
      } else {
        pixels.push_back(*((uint8_t*)row + 0));
        pixels.push_back(*((uint8_t*)row + 1));
        pixels.push_back(*((uint8_t*)row + 2));
        pixels.push_back(*((uint8_t*)row + 3));
      }
      row++;
    }
    data += sub_resource_layout.rowPitch;
  }
  stbi_write_png(filename.c_str(), extent.width, extent.height, 4,
                 pixels.data(), extent.width * 4);

  LOGI("Done saving screenshot.");
}

}  // namespace hex
