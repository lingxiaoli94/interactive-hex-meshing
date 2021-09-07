#include "vkoo/core/Image.h"

#include "vkoo/core/Device.h"
#include "vkoo/utils.h"

namespace vkoo {
namespace core {
Image::Image(Device& device, VkImage handle, VkExtent2D extent, VkFormat format,
             VkImageUsageFlags usage)
    : device_{device},
      handle_{handle},
      extent_{extent},
      format_{format},
      usage_{usage} {
  subresource_.mipLevel = 1;
  subresource_.arrayLayer = 1;
}

Image::Image(Image&& other) noexcept
    : device_{other.device_},
      handle_{other.handle_},
      extent_{other.extent_},
      format_{other.format_},
      usage_{other.usage_},
      tiling_{other.tiling_},
      sample_count_{other.sample_count_},
      subresource_{other.subresource_},
      memory_{other.memory_} {
  other.handle_ = VK_NULL_HANDLE;
  other.memory_ = VK_NULL_HANDLE;
}

Image::Image(Device& device, const VkExtent2D& extent, VkFormat format,
             VkImageUsageFlags usage, VkMemoryPropertyFlags properties,
             VkSampleCountFlagBits sample_count, VkImageTiling tiling,
             VkImageCreateFlags flags)
    : device_{device},
      extent_{extent},
      format_{format},
      usage_{usage},
      tiling_{tiling},
      sample_count_{sample_count} {
  subresource_.mipLevel = 1;
  subresource_.arrayLayer = 1;

  VkImageCreateInfo image_info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
  image_info.flags = flags;
  image_info.imageType = VK_IMAGE_TYPE_2D;
  image_info.extent.width = extent.width;
  image_info.extent.height = extent.height;
  image_info.extent.depth = 1;
  image_info.format = format;
  image_info.tiling = tiling;
  image_info.usage = usage;
  image_info.samples = sample_count;
  image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  image_info.mipLevels = subresource_.mipLevel;
  image_info.arrayLayers = subresource_.arrayLayer;

  VK_CHECK(vkCreateImage(device.GetHandle(), &image_info, nullptr, &handle_));

  VkMemoryRequirements mem_requirements;
  vkGetImageMemoryRequirements(device_.GetHandle(), handle_, &mem_requirements);

  VkMemoryAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc_info.allocationSize = mem_requirements.size;
  alloc_info.memoryTypeIndex =
      FindMemoryType(device_, mem_requirements.memoryTypeBits, properties);

  VK_CHECK(
      vkAllocateMemory(device_.GetHandle(), &alloc_info, nullptr, &memory_));

  VK_CHECK(vkBindImageMemory(device_.GetHandle(), handle_, memory_, 0));
}

Image::~Image() {
  // Don't destroy swapchain images here.
  if (handle_ != VK_NULL_HANDLE && memory_ != VK_NULL_HANDLE) {
    vkDestroyImage(device_.GetHandle(), handle_, nullptr);
    vkFreeMemory(device_.GetHandle(), memory_, nullptr);
  }
}
}  // namespace core
}  // namespace vkoo
