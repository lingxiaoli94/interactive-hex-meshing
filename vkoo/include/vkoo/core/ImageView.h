#pragma once

#include "Image.h"

namespace vkoo {
namespace core {
class ImageView {
 public:
  ImageView(Image& image, VkImageViewType view_type);
  ImageView(ImageView&& other);
  ~ImageView();

  VkImageView GetHandle() const { return handle_; }
  VkImageSubresourceRange GetSubresourceRange() const {
    return subresource_range_;
  }
  VkFormat GetFormat() const { return format_; }
  const Image& GetImage() const { return image_; }
  VkImageSubresourceLayers GetSubresourceLayers() const;

 private:
  Device& device_;
  Image& image_;
  VkImageView handle_;
  VkFormat format_;
  VkImageSubresourceRange subresource_range_;
};
}  // namespace core
}  // namespace vkoo
