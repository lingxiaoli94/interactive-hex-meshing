#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;

namespace core {
class Buffer {
 public:
  Buffer(Device& device, VkDeviceSize size, VkBufferUsageFlags usage,
         VkMemoryPropertyFlags properties);
  ~Buffer();
  Buffer(Buffer&& other);

  VkBuffer GetHandle() const { return handle_; }
  VkDeviceSize GetSize() const { return size_; }
  uint8_t* Map();
  void Unmap();
  void Update(const std::vector<uint8_t>& data, size_t offset = 0);
  void Update(const uint8_t* data, size_t size, size_t offset = 0);

  template <class T>
  void ConvertAndUpdate(const T& value, uint32_t offset = 0) {
    Update(to_bytes(value), offset);
  }

 private:
  Device& device_;
  VkDeviceSize size_;
  VkBuffer handle_;
  VkDeviceMemory memory_;
  uint8_t* mapped_data_;
  bool mapped_;
};
}  // namespace core
}  // namespace vkoo