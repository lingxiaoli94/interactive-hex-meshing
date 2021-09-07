#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;
class DescriptorSetLayout;
class DescriptorPool;

class DescriptorSet {
 public:
  DescriptorSet(Device& device, DescriptorSetLayout& descriptor_set_layout,
                DescriptorPool& descriptor_pool,
                const BindingMap<VkDescriptorBufferInfo>& buffer_infos = {},
                const BindingMap<VkDescriptorImageInfo>& image_infos = {});
  DescriptorSet(DescriptorSet&& other);
  ~DescriptorSet() = default;

  void Update();
  VkDescriptorSet GetHandle() const { return handle_; }

 private:
  void Prepare();

  Device& device_;
  DescriptorSetLayout& descriptor_set_layout_;
  DescriptorPool& descriptor_pool_;
  BindingMap<VkDescriptorBufferInfo> buffer_infos_;
  BindingMap<VkDescriptorImageInfo> image_infos_;

  VkDescriptorSet handle_;
  std::vector<VkWriteDescriptorSet> write_descriptor_sets_;

  // Map binding number to a hash of the binding description, to prevent
  // duplicate calls to vkUpdateDescriptorSets.
  std::unordered_map<uint32_t, size_t> updated_bindings_;
};
}  // namespace vkoo
