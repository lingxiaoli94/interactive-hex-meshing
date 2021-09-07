#pragma once

#include "CommandPool.h"
#include "FencePool.h"
#include "PhysicalDevice.h"
#include "Queue.h"
#include "ResourceCache.h"

namespace vkoo {

class Device {
 public:
  Device(const PhysicalDevice& gpu, VkSurfaceKHR surface,
         const std::vector<const char*>& required_extensions);

  ~Device();

  VkDevice GetHandle() const { return handle_; }
  const PhysicalDevice& GetGPU() const { return gpu_; }
  const Queue& GetSuitableGraphicsQueue() const;
  ResourceCache& GetResourceCache() { return resource_cache_; }
  const Queue& GetQueueByFlags(VkQueueFlags required_queue_flags,
                               uint32_t queue_index);
  CommandBuffer& RequestCommandBuffer();

  const std::unordered_map<VkBufferUsageFlags, uint32_t> kSupportedUsageMap = {
      {VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, 1}};

  VkFence RequestFence();
  CommandPool& GetCommandPool() { return *command_pool_; }
  FencePool& GetFencePool() { return *fence_pool_; }

  void WaitIdle();

 private:
  const PhysicalDevice& gpu_;
  VkDevice handle_;
  std::vector<std::vector<Queue>> queue_families_;

  ResourceCache resource_cache_;

  std::unique_ptr<CommandPool> command_pool_;
  std::unique_ptr<FencePool> fence_pool_;
};
}  // namespace vkoo
