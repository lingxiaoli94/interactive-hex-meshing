#pragma once

#include "DescriptorSetLayout.h"
#include "Framebuffer.h"
#include "GraphicsPipeline.h"
#include "RenderPass.h"
#include "vkoo/common.h"

namespace vkoo {
class Device;

struct ResourceCacheState {
  std::unordered_map<size_t, RenderPass> render_passes;
  std::unordered_map<size_t, DescriptorSetLayout> descriptor_set_layouts;
  std::unordered_map<size_t, ShaderModule> shader_modules;
  std::unordered_map<size_t, Framebuffer> framebuffers;
  std::unordered_map<size_t, PipelineLayout> pipeline_layouts;
  std::unordered_map<size_t, GraphicsPipeline> graphics_pipelines;
};

class ResourceCache {
 public:
  ResourceCache(Device& device);

  DescriptorSetLayout& RequestDescriptorSetLayout(
      const std::vector<ShaderModule*>& shader_modules, uint32_t set_index,
      const std::vector<ShaderResource>& resource_set);

  RenderPass& RequestRenderPass(
      const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
      const std::vector<SubpassInfo>& subpasses,
      const std::vector<VkSubpassDependency2KHR>& dependencies);
  Framebuffer& RequestFramebuffer(const VkExtent2D& extent,
                                  const std::vector<VkImageView>& attachments,
                                  const RenderPass& render_pass);
  GraphicsPipeline& RequestGraphicsPipeline(PipelineState& pipeline_state);
  ShaderModule& RequestShaderModule(VkShaderStageFlagBits stage,
                                    const ShaderSource& shader_source,
                                    const ShaderVariant& shader_variant = {});
  PipelineLayout& RequestPipelineLayout(
      const std::vector<ShaderModule*>& shader_modules);

  void ClearFramebuffers();
  void Clear();

 private:
  Device& device_;

  ResourceCacheState state_;

  std::mutex shader_module_mutex_;
  std::mutex descriptor_set_layout_mutex_;
  std::mutex render_pass_mutex_;
  std::mutex framebuffer_mutex_;
  std::mutex pipeline_layout_mutex_;
  std::mutex graphics_pipeline_mutex_;
};
}  // namespace vkoo
