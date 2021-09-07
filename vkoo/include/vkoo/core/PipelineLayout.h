#pragma once

#include "DescriptorSetLayout.h"
#include "ShaderModule.h"
#include "vkoo/common.h"

namespace vkoo {
class Device;
class PipelineLayout {
 public:
  PipelineLayout(Device& device,
                 const std::vector<ShaderModule*>& shader_modules);
  PipelineLayout(PipelineLayout&& other);
  ~PipelineLayout();

  std::vector<ShaderResource> GetResources(
      ShaderResourceType type = ShaderResourceType::All,
      VkShaderStageFlagBits stage = VK_SHADER_STAGE_ALL) const;

  const std::vector<ShaderModule*>& GetShaderModules() const;
  VkPipelineLayout GetHandle() const { return handle_; }

  DescriptorSetLayout& GetDescriptorSetLayout(uint32_t set_index) const;
  bool HasDescriptorSetLayout(uint32_t set_index) const;
  VkShaderStageFlags GetPushConstantRangeStage(uint32_t size,
                                               uint32_t offset = 0) const;

 private:
  Device& device_;
  VkPipelineLayout handle_;
  std::vector<ShaderModule*> shader_modules_;

  std::unordered_map<std::string, ShaderResource> name_to_resource_dict_;
  std::unordered_map<uint32_t, std::vector<ShaderResource>> shader_sets_;
  std::vector<DescriptorSetLayout*> descriptor_set_layouts_;
};
}  // namespace vkoo
