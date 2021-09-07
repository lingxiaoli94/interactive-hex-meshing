#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;
class DescriptorSetLayout;

class DescriptorPool {
 public:
  static const uint32_t MAX_SETS_PER_POOL = 16;

  DescriptorPool(Device& device,
                 const DescriptorSetLayout& descriptor_set_layout,
                 uint32_t pool_size = MAX_SETS_PER_POOL);

  ~DescriptorPool();

  void Reset();
  VkDescriptorSet Allocate();
  void Free(VkDescriptorSet descriptor_set);

  const DescriptorSetLayout& GetDescriptorSetLayout() const {
    return descriptor_set_layout_;
  }

 private:
  uint32_t FindAvailablePool(uint32_t search_index);

  Device& device_;
  const DescriptorSetLayout& descriptor_set_layout_;
  std::vector<VkDescriptorPoolSize> pool_sizes_;
  uint32_t pool_max_sets_;
  std::vector<VkDescriptorPool> pools_;
  std::vector<uint32_t> pool_sets_count_;
  uint32_t pool_index_;
  std::unordered_map<VkDescriptorSet, uint32_t> set_pool_mapping_;
};
}  // namespace vkoo
