#include "vkoo/core/DescriptorPool.h"
#include "vkoo/core/DescriptorSet.h"
#include "vkoo/core/DescriptorSetLayout.h"
#include "vkoo/core/Device.h"
#include "vkoo/core/resource_caching.h"
#include "vkoo/logging.h"

namespace vkoo {
DescriptorSet::DescriptorSet(
    Device& device, DescriptorSetLayout& descriptor_set_layout,
    DescriptorPool& descriptor_pool,
    const BindingMap<VkDescriptorBufferInfo>& buffer_infos,
    const BindingMap<VkDescriptorImageInfo>& image_infos)
    : device_{device},
      descriptor_set_layout_{descriptor_set_layout},
      descriptor_pool_(descriptor_pool),
      buffer_infos_{buffer_infos},
      image_infos_{image_infos},
      handle_{descriptor_pool.Allocate()} {
  Prepare();
}

DescriptorSet::DescriptorSet(DescriptorSet&& other)
    : device_{other.device_},
      descriptor_set_layout_{other.descriptor_set_layout_},
      descriptor_pool_{other.descriptor_pool_},
      buffer_infos_{std::move(other.buffer_infos_)},
      image_infos_{std::move(other.image_infos_)},
      handle_{other.handle_},
      write_descriptor_sets_{std::move(other.write_descriptor_sets_)},
      updated_bindings_{std::move(other.updated_bindings_)} {
  other.handle_ = VK_NULL_HANDLE;
}

void DescriptorSet::Prepare() {
  if (!write_descriptor_sets_.empty()) {
    throw std::runtime_error(
        "Trying to prepare a descriptor set that has already been prepared, "
        "skipping.");
    return;
  }

  for (auto& binding_it : buffer_infos_) {
    auto binding_index = binding_it.first;
    auto& buffer_bindings = binding_it.second;

    if (auto binding_info =
            descriptor_set_layout_.GetLayoutBinding(binding_index)) {
      for (auto& element_it : buffer_bindings) {
        auto& buffer_info = element_it.second;

#ifndef NDEBUG
        size_t uniform_buffer_range_limit =
            device_.GetGPU().GetProperties().limits.maxUniformBufferRange;
        size_t buffer_range_limit = buffer_info.range;
        assert(binding_info->descriptorType ==
               VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        assert(buffer_range_limit <= uniform_buffer_range_limit);
#endif

        VkWriteDescriptorSet write_descriptor_set{
            VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};

        write_descriptor_set.dstBinding = binding_index;
        write_descriptor_set.descriptorType = binding_info->descriptorType;
        write_descriptor_set.pBufferInfo = &buffer_info;
        write_descriptor_set.dstSet = handle_;
        write_descriptor_set.dstArrayElement = element_it.first;
        write_descriptor_set.descriptorCount = 1;

        write_descriptor_sets_.push_back(write_descriptor_set);
      }
    } else {
      throw std::runtime_error(
          fmt::format("Shader layout set does not use buffer binding at {}",
                      binding_index));
    }
  }

  for (auto& binding_it : image_infos_) {
    auto binding_index = binding_it.first;
    auto& binding_resources = binding_it.second;

    if (auto binding_info =
            descriptor_set_layout_.GetLayoutBinding(binding_index)) {
      for (auto& element_it : binding_resources) {
        auto& image_info = element_it.second;

        VkWriteDescriptorSet write_descriptor_set{
            VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};

        write_descriptor_set.dstBinding = binding_index;
        write_descriptor_set.descriptorType = binding_info->descriptorType;
        write_descriptor_set.pImageInfo = &image_info;
        write_descriptor_set.dstSet = handle_;
        write_descriptor_set.dstArrayElement = element_it.first;
        write_descriptor_set.descriptorCount = 1;

        write_descriptor_sets_.push_back(write_descriptor_set);
      }
    } else {
      throw std::runtime_error(fmt::format(
          "Shader layout set does not use image binding at {}", binding_index));
    }
  }
}

void DescriptorSet::Update() {
  std::vector<VkWriteDescriptorSet> write_operations;
  std::vector<size_t> write_operation_hashes;

  for (size_t i = 0; i < write_descriptor_sets_.size(); i++) {
    const auto& write_operation = write_descriptor_sets_[i];
    size_t write_operation_hash = 0;
    hash_param(write_operation_hash, write_operation);
    auto it = updated_bindings_.find(write_operation.dstBinding);
    if (it == updated_bindings_.end() || it->second != write_operation_hash) {
      write_operations.push_back(write_operation);
      write_operation_hashes.push_back(write_operation_hash);
    }
  }

  if (!write_operations.empty()) {
    vkUpdateDescriptorSets(device_.GetHandle(),
                           static_cast<uint32_t>(write_operations.size()),
                           write_operations.data(), 0, nullptr);
  }

  for (size_t i = 0; i < write_operations.size(); i++) {
    updated_bindings_[write_operations[i].dstBinding] =
        write_operation_hashes[i];
  }
}

}  // namespace vkoo
