#include "vkoo/core/BufferPool.h"

#include "vkoo/core/Device.h"

namespace vkoo {

BufferPool::BufferPool(Device& device, VkBufferUsageFlags usage,
                       VkMemoryPropertyFlags memory_properties)
    : device_{device}, usage_{usage}, memory_properties_{memory_properties} {}

core::Buffer& BufferPool::RequestBuffer(VkDeviceSize size) {
  buffers_.emplace_back(std::make_unique<core::Buffer>(device_, size, usage_,
                                                       memory_properties_));
  return *buffers_.back();
}

void BufferPool::Reset() { buffers_.clear(); }

}  // namespace vkoo
