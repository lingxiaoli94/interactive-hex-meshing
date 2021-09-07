#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;

namespace core {
class Image {
 public:
  Image(Device& device, VkImage handle, VkExtent2D extent, VkFormat format,
        VkImageUsageFlags usage);
  Image(Device& device, const VkExtent2D& extent, VkFormat format,
        VkImageUsageFlags usage, VkMemoryPropertyFlags properties,
        VkSampleCountFlagBits sample_count = VK_SAMPLE_COUNT_1_BIT,
        VkImageTiling tiling = VK_IMAGE_TILING_OPTIMAL,
        VkImageCreateFlags flags = 0);
  Image(Image&& other) noexcept;
  ~Image();

  Device& GetDevice() { return device_; }
  const VkExtent2D& GetExtent() { return extent_; }
  VkImage GetHandle() const { return handle_; }
  VkFormat GetFormat() const { return format_; }
  VkImageUsageFlags GetUsage() const { return usage_; }
  VkSampleCountFlagBits GetSampleCount() const { return sample_count_; };
  VkDeviceMemory GetMemory() const { return memory_; }

 private:
  Device& device_;
  VkImage handle_{VK_NULL_HANDLE};
  VkExtent2D extent_;
  VkFormat format_{VK_FORMAT_UNDEFINED};
  VkImageUsageFlags usage_;
  VkImageTiling tiling_{VK_IMAGE_TILING_OPTIMAL};
  VkSampleCountFlagBits sample_count_{VK_SAMPLE_COUNT_1_BIT};
  VkImageSubresource subresource_{};

  VkDeviceMemory memory_{VK_NULL_HANDLE};
};
}  // namespace core
}  // namespace vkoo
