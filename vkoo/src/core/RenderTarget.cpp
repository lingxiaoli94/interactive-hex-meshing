#include "vkoo/core/RenderTarget.h"

#include "vkoo/core/Device.h"

namespace vkoo {
RenderTarget::RenderTarget(std::vector<core::Image>&& images)
    : device_(images.back().GetDevice()), images_(std::move(images)) {
  // Check if the extent is unique.
  for (auto& image : images_) {
    if (image.GetExtent().height != images_.back().GetExtent().height ||
        image.GetExtent().width != images_.back().GetExtent().width) {
      throw std::runtime_error("Extent size is not unique!");
    }
  }

  extent_ = images_.back().GetExtent();
  for (auto& image : images_) {
    views_.emplace_back(image, VK_IMAGE_VIEW_TYPE_2D);
  }
}

const RenderTarget::CreateFunc RenderTarget::kDefaultCreateFunc =
    [](core::Image&& image) -> std::unique_ptr<RenderTarget> {
  VkFormat depth_format =
      GetSuitableDepthFormat(image.GetDevice().GetGPU().GetHandle());

  std::vector<core::Image> images;
  images.push_back(std::move(image));

  core::Image depth_image{image.GetDevice(), image.GetExtent(), depth_format,
                          VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
                              VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT};
  images.push_back(std::move(depth_image));

  return std::make_unique<RenderTarget>(std::move(images));
};
}  // namespace vkoo
