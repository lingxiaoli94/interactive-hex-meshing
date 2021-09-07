#pragma once

#include <vkoo/core/Application.h>
#include <vkoo/core/Sampler.h>
#include <vkoo/core/VertexObject.h>
#include <vkoo/st/Material.h>
#include <vkoo/st/ShaderProgram.h>
#include <vkoo/st/Texture.h>
#include <vkoo/st/components/Mesh.h>
#include <vkoo/st/components/Script.h>

#include "Settings.h"
#include "controllers/GlobalController.h"

namespace hex {
class HexMeshingApp : public vkoo::Application {
 public:
  HexMeshingApp(bool enable_validation_layer);
  ~HexMeshingApp();
  void Prepare() override;
  void DrawGui() override;
  Settings& GetSettings();
  void UpdatePipelines();
  void MainLoop() override;

  void ReloadSettings(const std::string& filename);
  void SaveSettings(const std::string& filename);
  void SaveScreenshot(const std::string& filename);
  void UpdateCameraAndLighting();

 private:
  struct AttachmentInfo {
    VkAttachmentLoadOp load_op;
    VkAttachmentStoreOp store_op;
    VkImageLayout initial_layout;
    VkImageLayout final_layout;
  };

  struct PipelineInfo {
    std::unique_ptr<vkoo::RenderPipeline> pipeline;
    std::vector<uint32_t> attachment_ids;  // index in render target
    std::vector<AttachmentInfo>
        attachment_infos;  // same size as attachment_ids
    std::vector<VkSubpassDependency2KHR> dependencies;
  };

  void UpdateScene(float delta_time) override;
  void HandleInputEvent(const vkoo::InputEvent& event) override;
  void FramebufferSizeCallback(int width, int height) override;

  void PrepareRenderContext() override;
  void DrawRenderPasses(vkoo::CommandBuffer& command_buffer,
                        vkoo::RenderTarget& render_target) override;

  std::unique_ptr<vkoo::RenderTarget> CreateRenderTarget(
      vkoo::core::Image&& swapchain_image);

  void DrawPipeline(vkoo::CommandBuffer& command_buffer,
                    vkoo::RenderTarget& render_target,
                    const PipelineInfo& pipeline_info);

  void SetupScene();
  void RecreateLights();
  void SetupRenderPipelines();
  void SetupGBufferPipeline();
  void SetupLightingPipeline();
  void SetupSSAOPipeline();
  void SetupSSAOBlurPipeline();
  void SetupTransparentPipeline();
  void SetupPostprocessingPipeline();

  void PrepareSupportedSampleCountList();
  void PrepareDepthResolveModeList();

  bool IsMSAAEnabled() const;

  float GetDpiFactor() const override { return 1.0f; }

  struct {
    PipelineInfo gbuffer_generation{};
    PipelineInfo ssao{};
    PipelineInfo ssao_blur{};
    PipelineInfo lighting{};
    PipelineInfo transparent{};
    PipelineInfo postprocessing{};
  } pipeline_infos;

  // Below are application specific implementations.
  void CreateSampler();

  std::unique_ptr<vkoo::core::Sampler> sampler_;

  std::unique_ptr<GlobalController> global_controller_;

  Settings settings_;

  VkSampleCountFlagBits sample_count_{VK_SAMPLE_COUNT_1_BIT};
  std::vector<VkSampleCountFlagBits> supported_sample_count_list_;
  VkResolveModeFlagBits depth_resolve_mode_{
      VkResolveModeFlagBits::VK_RESOLVE_MODE_NONE};
  std::vector<VkResolveModeFlagBits> supported_depth_resolve_mode_list_;

  // Keep track of indices of attachments.
  struct {
    uint32_t swapchain;
    uint32_t depth_ms;
    uint32_t depth_resolved;
    uint32_t albedo_ms;
    uint32_t albedo_resolved;
    uint32_t normal_ms;
    uint32_t normal_resolved;
    uint32_t ssao;
    uint32_t ssao_blur;
    uint32_t transparent_ms;
    uint32_t transparent_resolved;
  } attachment_dict;

  VkClearColorValue background_clear_color_{{0.0f, 0.0f, 0.0f, 0.0f}};
};
}  // namespace hex
