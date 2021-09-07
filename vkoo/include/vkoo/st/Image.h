#pragma once

#include "vkoo/common.h"
#include "vkoo/core/Device.h"
#include "vkoo/core/Image.h"
#include "vkoo/core/ImageView.h"

namespace vkoo {
namespace st {
struct Mipmap {
  /// Mipmap level.
  uint32_t level = 0;

  /// Byte offset used for uploading.
  uint32_t offset = 0;

  /// Width and height of the mipmap.
  VkExtent2D extent = {0, 0};
};

class Image {
 public:
  Image(std::vector<uint8_t>&& data, std::vector<Mipmap>&& mipmaps = {{}},
        VkFormat format = VK_FORMAT_R8G8B8A8_SRGB);
  static std::unique_ptr<Image> Load(const std::string& image_path);
  virtual ~Image() = default;

  void CreateVkImageAndView(
      Device& device, VkImageViewType image_view_type = VK_IMAGE_VIEW_TYPE_2D,
      VkImageCreateFlags flags = 0);

  void UploadDataToGPU(Device& device);

  const core::Image& GetVkImage() const { return *vk_image_; }
  const core::ImageView& GetVkImageView() const { return *vk_image_view_; }
  const VkExtent2D& GetExtent() const { return mipmaps_.at(0).extent; }

 private:
  std::vector<uint8_t> data_;
  std::vector<Mipmap> mipmaps_;

  VkFormat format_{VK_FORMAT_UNDEFINED};
  std::unique_ptr<core::Image> vk_image_;
  std::unique_ptr<core::ImageView> vk_image_view_;
};
}  // namespace st
}  // namespace vkoo
