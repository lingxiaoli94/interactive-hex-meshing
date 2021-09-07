#pragma once

#include "Buffer.h"

namespace vkoo {
class Device;

class BufferPool {
 public:
  BufferPool(Device& device, VkBufferUsageFlags usage,
             VkMemoryPropertyFlags memory_properties);

  core::Buffer& RequestBuffer(VkDeviceSize size);
  void Reset();

 private:
  Device& device_;
  std::vector<std::unique_ptr<core::Buffer>> buffers_;
  VkBufferUsageFlags usage_;
  VkMemoryPropertyFlags memory_properties_;
};
}  // namespace vkoo
