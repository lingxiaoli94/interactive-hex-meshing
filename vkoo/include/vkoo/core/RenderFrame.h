#pragma once

#include "BufferPool.h"
#include "CommandPool.h"
#include "DescriptorPool.h"
#include "DescriptorSet.h"
#include "Device.h"
#include "FencePool.h"
#include "Queue.h"
#include "RenderTarget.h"
#include "SemaphorePool.h"

namespace vkoo {
class RenderFrame {
 public:
  RenderFrame(Device& device, std::unique_ptr<RenderTarget>&& render_target);

  const std::unordered_map<VkBufferUsageFlags, uint32_t> kSupportedUsageMap = {
      {VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, 1},
      {VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, 1},
      {VK_BUFFER_USAGE_INDEX_BUFFER_BIT, 1}};

  DescriptorSet& RequestDescriptorSet(
      DescriptorSetLayout& descriptor_set_layout,
      const BindingMap<VkDescriptorBufferInfo>& buffer_infos,
      const BindingMap<VkDescriptorImageInfo>& image_infos);
  CommandBuffer& RequestCommandBuffer(const Queue& queue);
  core::Buffer& AllocateBuffer(VkBufferUsageFlags usage, VkDeviceSize size);
  VkSemaphore RequestSemaphore();
  VkFence RequestFence();
  void UpdateRenderTarget(std::unique_ptr<RenderTarget>&& render_target);

  void Reset();
  void ClearDescriptors();
  RenderTarget& GetRenderTarget() { return *render_target_; }

 private:
  Device& device_;
  std::unique_ptr<RenderTarget> render_target_;

  // One command pool per queue family.
  std::map<uint32_t, std::unique_ptr<CommandPool>> command_pool_dict_;
  FencePool fence_pool_;
  SemaphorePool semaphore_pool_;
  std::map<VkBufferUsageFlags, BufferPool> buffer_pool_dict_;
  std::unordered_map<size_t, DescriptorPool> descriptor_pool_;
  std::unordered_map<size_t, DescriptorSet> descriptor_sets_;
};
}  // namespace vkoo
