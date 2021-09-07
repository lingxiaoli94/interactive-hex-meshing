#include "vkoo/core/ResourceCache.h"

#include "vkoo/core/resource_caching.h"

namespace vkoo {
namespace {
template <class T, class... A>
T& RequestResourceSync(Device& device, std::mutex& resource_mutex,
                       std::unordered_map<std::size_t, T>& resource_pool,
                       A&... args) {
  std::lock_guard<std::mutex> guard(resource_mutex);
  return RequestResource(device, resource_pool, args...);
}
}  // namespace

ResourceCache::ResourceCache(Device& device) : device_{device} {}

DescriptorSetLayout& ResourceCache::RequestDescriptorSetLayout(
    const std::vector<ShaderModule*>& shader_modules, uint32_t set_index,
    const std::vector<ShaderResource>& resource_set) {
  return RequestResourceSync(device_, descriptor_set_layout_mutex_,
                             state_.descriptor_set_layouts, shader_modules,
                             set_index, resource_set);
}

RenderPass& ResourceCache::RequestRenderPass(
    const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
    const std::vector<SubpassInfo>& subpasses,
    const std::vector<VkSubpassDependency2KHR>& dependencies) {
  return RequestResourceSync(device_, render_pass_mutex_, state_.render_passes,
                             attachment_descriptions, subpasses, dependencies);
}

Framebuffer& ResourceCache::RequestFramebuffer(
    const VkExtent2D& extent, const std::vector<VkImageView>& attachments,
    const RenderPass& render_pass) {
  return RequestResourceSync(device_, framebuffer_mutex_, state_.framebuffers,
                             extent, attachments, render_pass);
}

GraphicsPipeline& ResourceCache::RequestGraphicsPipeline(
    PipelineState& pipeline_state) {
  return RequestResourceSync(device_, graphics_pipeline_mutex_,
                             state_.graphics_pipelines, pipeline_state);
}

ShaderModule& ResourceCache::RequestShaderModule(
    VkShaderStageFlagBits stage, const ShaderSource& shader_source,
    const ShaderVariant& shader_variant) {
  return RequestResourceSync(device_, shader_module_mutex_,
                             state_.shader_modules, stage, shader_source,
                             shader_variant);
}

PipelineLayout& ResourceCache::RequestPipelineLayout(
    const std::vector<ShaderModule*>& shader_modules) {
  return RequestResourceSync(device_, pipeline_layout_mutex_,
                             state_.pipeline_layouts, shader_modules);
}

void ResourceCache::ClearFramebuffers() { state_.framebuffers.clear(); }

void ResourceCache::Clear() {
  state_.pipeline_layouts.clear();
  state_.shader_modules.clear();
  state_.graphics_pipelines.clear();
  state_.descriptor_set_layouts.clear();
  state_.render_passes.clear();
  ClearFramebuffers();
}

}  // namespace vkoo
