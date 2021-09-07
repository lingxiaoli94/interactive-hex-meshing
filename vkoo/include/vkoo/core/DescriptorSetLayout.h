#pragma once

#include "vkoo/common.h"
#include <optional>

namespace vkoo {
class Device;
struct ShaderResource;
class ShaderModule;

class DescriptorSetLayout {
 public:
  DescriptorSetLayout(Device& device,
                      const std::vector<ShaderModule*>& shader_modules,
                      uint32_t set_index,
                      const std::vector<ShaderResource>& resource_set);
  DescriptorSetLayout(DescriptorSetLayout&& other);
  ~DescriptorSetLayout();

  VkDescriptorSetLayout GetHandle() const { return handle_; }

  std::optional<VkDescriptorSetLayoutBinding> GetLayoutBinding(
      uint32_t binding_index) const;

  std::optional<VkDescriptorSetLayoutBinding> GetLayoutBinding(
      const std::string& name) const;

  const std::vector<VkDescriptorSetLayoutBinding> GetBindings() const {
    return bindings_;
  }

  uint32_t GetSetIndex() const { return set_index_; }

 private:
  Device& device_;
  VkDescriptorSetLayout handle_;
  uint32_t set_index_;
  std::vector<ShaderModule*> shader_modules_;
  std::vector<VkDescriptorSetLayoutBinding> bindings_;
  std::vector<VkDescriptorBindingFlagsEXT> binding_flags_;

  std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding>
      binding_id_to_layout_dict_;
  std::unordered_map<std::string, uint32_t> name_to_binding_id_dict_;
};
}  // namespace vkoo