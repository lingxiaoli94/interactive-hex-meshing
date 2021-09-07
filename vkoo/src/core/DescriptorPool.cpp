#include "vkoo/core/DescriptorPool.h"

#include "vkoo/core/DescriptorSetLayout.h"
#include "vkoo/core/Device.h"

namespace vkoo {
DescriptorPool::DescriptorPool(Device& device,
                               const DescriptorSetLayout& descriptor_set_layout,
                               uint32_t pool_size)
    : device_{device},
      descriptor_set_layout_{descriptor_set_layout},
      pool_index_{0} {
  const std::vector<VkDescriptorSetLayoutBinding>& bindings =
      descriptor_set_layout.GetBindings();

  std::map<VkDescriptorType, std::uint32_t> descriptor_type_counts;

  for (auto& binding : bindings) {
    descriptor_type_counts[binding.descriptorType] += binding.descriptorCount;
  }

  pool_sizes_.resize(descriptor_type_counts.size());

  auto pool_size_it = pool_sizes_.begin();

  for (auto& it : descriptor_type_counts) {
    pool_size_it->type = it.first;
    pool_size_it->descriptorCount = it.second * pool_size;
    ++pool_size_it;
  }

  pool_max_sets_ = pool_size;
}

DescriptorPool::~DescriptorPool() {
  for (auto pool : pools_) {
    vkDestroyDescriptorPool(device_.GetHandle(), pool, nullptr);
  }
}

uint32_t DescriptorPool::FindAvailablePool(uint32_t search_index) {
  if (pools_.size() <= search_index) {
    VkDescriptorPoolCreateInfo create_info{
        VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};

    create_info.poolSizeCount = static_cast<uint32_t>(pool_sizes_.size());
    create_info.pPoolSizes = pool_sizes_.data();
    create_info.maxSets = pool_max_sets_;
    create_info.flags = 0;

    VkDescriptorPool handle = VK_NULL_HANDLE;

    VK_CHECK(vkCreateDescriptorPool(device_.GetHandle(), &create_info, nullptr,
                                    &handle));

    pools_.push_back(handle);
    pool_sets_count_.push_back(0);

    return search_index;
  } else if (pool_sets_count_[search_index] < pool_max_sets_) {
    return search_index;
  }

  return FindAvailablePool(++search_index);
}
void DescriptorPool::Reset() {
  for (auto pool : pools_) {
    vkResetDescriptorPool(device_.GetHandle(), pool, 0);
  }

  std::fill(pool_sets_count_.begin(), pool_sets_count_.end(), 0);
  set_pool_mapping_.clear();

  pool_index_ = 0;
}

VkDescriptorSet DescriptorPool::Allocate() {
  pool_index_ = FindAvailablePool(pool_index_);

  // Increment allocated set count for the current pool.
  ++pool_sets_count_[pool_index_];

  VkDescriptorSetLayout set_layout = descriptor_set_layout_.GetHandle();

  VkDescriptorSetAllocateInfo alloc_info{
      VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
  alloc_info.descriptorPool = pools_[pool_index_];
  alloc_info.descriptorSetCount = 1;
  alloc_info.pSetLayouts = &set_layout;

  VkDescriptorSet handle = VK_NULL_HANDLE;

  VK_CHECK(vkAllocateDescriptorSets(device_.GetHandle(), &alloc_info, &handle));

  set_pool_mapping_.emplace(handle, pool_index_);

  return handle;
}

void DescriptorPool::Free(VkDescriptorSet descriptor_set) {
  // Get the pool index of the descriptor set
  auto it = set_pool_mapping_.find(descriptor_set);

  if (it == set_pool_mapping_.end()) {
    throw std::runtime_error(
        "Cannot free descriptor: no set-pool mapping found!");
  }

  auto desc_pool_index = it->second;

  vkFreeDescriptorSets(device_.GetHandle(), pools_[desc_pool_index], 1,
                       &descriptor_set);

  set_pool_mapping_.erase(it);
  --pool_sets_count_[desc_pool_index];
  pool_index_ = desc_pool_index;
}

}  // namespace vkoo