#include "vkoo/core/Buffer.h"

#include "vkoo/core/Device.h"
#include "vkoo/utils.h"

namespace vkoo {
namespace core {
Buffer::Buffer(Device& device, VkDeviceSize size, VkBufferUsageFlags usage,
               VkMemoryPropertyFlags properties)
    : device_(device), size_(size), mapped_data_{nullptr}, mapped_(false) {
  VkBufferCreateInfo buffer_info{};
  buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_info.size = size;
  buffer_info.usage = usage;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  VK_CHECK(
      vkCreateBuffer(device_.GetHandle(), &buffer_info, nullptr, &handle_));

  VkMemoryRequirements mem_requirements;
  vkGetBufferMemoryRequirements(device_.GetHandle(), handle_,
                                &mem_requirements);

  VkMemoryAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc_info.allocationSize = mem_requirements.size;
  alloc_info.memoryTypeIndex =
      FindMemoryType(device_, mem_requirements.memoryTypeBits, properties);

  VK_CHECK(
      vkAllocateMemory(device_.GetHandle(), &alloc_info, nullptr, &memory_));

  VK_CHECK(vkBindBufferMemory(device_.GetHandle(), handle_, memory_, 0));
}

Buffer::~Buffer() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyBuffer(device_.GetHandle(), handle_, nullptr);
  }
  if (memory_ != VK_NULL_HANDLE) {
    vkFreeMemory(device_.GetHandle(), memory_, nullptr);
  }
}

uint8_t* Buffer::Map() {
  if (!mapped_ && !mapped_data_) {
    vkMapMemory(device_.GetHandle(), memory_, 0, size_, 0,
                reinterpret_cast<void**>(&mapped_data_));
    mapped_ = true;
  }
  return mapped_data_;
}

void Buffer::Unmap() {
  if (mapped_) {
    vkUnmapMemory(device_.GetHandle(), memory_);
    mapped_data_ = nullptr;
    mapped_ = false;
  }
}

void Buffer::Update(const std::vector<uint8_t>& data, size_t offset) {
  Update(data.data(), data.size(), offset);
}

void Buffer::Update(const uint8_t* data, size_t size, size_t offset) {
  Map();
  std::copy(data, data + size, mapped_data_ + offset);
  Unmap();
}

Buffer::Buffer(Buffer&& other)
    : device_{other.device_},
      size_{other.size_},
      handle_{other.handle_},
      memory_{other.memory_},
      mapped_data_{other.mapped_data_},
      mapped_{other.mapped_} {
  other.handle_ = VK_NULL_HANDLE;
  other.memory_ = VK_NULL_HANDLE;
  other.mapped_data_ = nullptr;
  other.mapped_ = false;
}

}  // namespace core
}  // namespace vkoo
