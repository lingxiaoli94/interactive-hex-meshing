#include "vkoo/core/ImageView.h"

#include "vkoo/core/Device.h"

namespace vkoo {
namespace core {
ImageView::ImageView(Image& image, VkImageViewType view_type)
    : device_{image.GetDevice()}, image_{image}, format_{image.GetFormat()} {
  // TODO: mipmap.
  VkImageViewCreateInfo view_create_info{};
  view_create_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  view_create_info.image = image_.GetHandle();
  view_create_info.viewType = view_type;
  view_create_info.format = format_;

  subresource_range_.baseMipLevel = 0;
  subresource_range_.levelCount = 1;
  subresource_range_.baseArrayLayer = 0;
  subresource_range_.layerCount = 1;

  if (IsDepthStencilFormat(format_)) {
    subresource_range_.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  } else {
    subresource_range_.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  }

  view_create_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
  view_create_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
  view_create_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
  view_create_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
  view_create_info.subresourceRange = subresource_range_;

  VK_CHECK(vkCreateImageView(image_.GetDevice().GetHandle(), &view_create_info,
                             nullptr, &handle_));
}

ImageView::ImageView(ImageView&& other)
    : device_{other.device_},
      image_{other.image_},
      handle_{other.handle_},
      format_{other.format_},
      subresource_range_{other.subresource_range_} {
  other.handle_ = VK_NULL_HANDLE;
}

ImageView::~ImageView() {
  if (handle_ != nullptr) {
    vkDestroyImageView(device_.GetHandle(), handle_, nullptr);
  }
}

VkImageSubresourceLayers ImageView::GetSubresourceLayers() const {
  VkImageSubresourceLayers subresource{};
  subresource.aspectMask = subresource_range_.aspectMask;
  subresource.baseArrayLayer = subresource_range_.baseArrayLayer;
  subresource.layerCount = subresource_range_.layerCount;
  subresource.mipLevel = subresource_range_.baseMipLevel;
  return subresource;
}
}  // namespace core

}  // namespace vkoo
