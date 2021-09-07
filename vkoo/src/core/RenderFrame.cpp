#include "vkoo/core/RenderFrame.h"

#include "vkoo/core/resource_caching.h"
#include "vkoo/logging.h"

namespace vkoo {

RenderFrame::RenderFrame(Device& device,
                         std::unique_ptr<RenderTarget>&& render_target)
    : device_{device},
      render_target_{std::move(render_target)},
      fence_pool_{device},
      semaphore_pool_{device} {
  for (auto& usage_kv : kSupportedUsageMap) {
    buffer_pool_dict_.emplace(
        usage_kv.first, BufferPool(device, usage_kv.first,
                                   VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                       VK_MEMORY_PROPERTY_HOST_COHERENT_BIT));
  }
}

DescriptorSet& RenderFrame::RequestDescriptorSet(
    DescriptorSetLayout& descriptor_set_layout,
    const BindingMap<VkDescriptorBufferInfo>& buffer_infos,
    const BindingMap<VkDescriptorImageInfo>& image_infos) {
  DescriptorPool& descriptor_pool =
      RequestResource(device_, descriptor_pool_, descriptor_set_layout);

  return RequestResource(device_, descriptor_sets_, descriptor_set_layout,
                         descriptor_pool, buffer_infos, image_infos);
}

core::Buffer& RenderFrame::AllocateBuffer(VkBufferUsageFlags usage,
                                          VkDeviceSize size) {
  auto buffer_pool_it = buffer_pool_dict_.find(usage);
  if (buffer_pool_it == buffer_pool_dict_.end()) {
    throw std::runtime_error("No buffer pool for usage " +
                             std::to_string(usage) + "!");
  }
  auto& buffer_pool = buffer_pool_it->second;
  return buffer_pool.RequestBuffer(size);
}

void RenderFrame::Reset() {
  fence_pool_.Wait();
  fence_pool_.Reset();

  semaphore_pool_.Reset();

  for (auto& queue_cmd_pool_kv : command_pool_dict_) {
    queue_cmd_pool_kv.second->ResetPool();
  }

  for (auto& usage_buffer_pool_kv : buffer_pool_dict_) {
    usage_buffer_pool_kv.second.Reset();
  }

  // TODO: for now clear descriptors frame to frame.
  ClearDescriptors();
}

void RenderFrame::ClearDescriptors() {
  descriptor_sets_.clear();
  for (auto& kv : descriptor_pool_) {
    kv.second.Reset();
  }
  descriptor_pool_.clear();
}

CommandBuffer& RenderFrame::RequestCommandBuffer(const Queue& queue) {
  uint32_t queue_family_index = queue.GetFamilyIndex();
  auto it = command_pool_dict_.find(queue_family_index);
  if (it == command_pool_dict_.end()) {
    auto emplace_it = command_pool_dict_.emplace(
        queue_family_index,
        std::make_unique<CommandPool>(device_, queue_family_index, this));
    if (!emplace_it.second) {
      throw std::runtime_error(fmt::format(
          "Cannot emplace command pool for family {}!", queue_family_index));
    }
    it = emplace_it.first;
  }

  CommandPool& command_pool = *it->second;
  return command_pool.RequestCommandBuffer();
}

VkSemaphore RenderFrame::RequestSemaphore() {
  return semaphore_pool_.RequestSemaphore();
}

VkFence RenderFrame::RequestFence() { return fence_pool_.RequestFence(); }

void RenderFrame::UpdateRenderTarget(
    std::unique_ptr<RenderTarget>&& render_target) {
  render_target_ = std::move(render_target);
}

}  // namespace vkoo