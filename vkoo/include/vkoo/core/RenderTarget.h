#pragma once

#include "Image.h"
#include "ImageView.h"
#include "vkoo/common.h"

namespace vkoo {
class RenderTarget {
 public:
  using CreateFunc =
      std::function<std::unique_ptr<RenderTarget>(core::Image&&)>;
  static const CreateFunc kDefaultCreateFunc;

  RenderTarget(std::vector<core::Image>&& images);

  const VkExtent2D& GetExtent() const { return extent_; }
  const std::vector<core::ImageView>& GetViews() const { return views_; }

 private:
  Device& device_;
  VkExtent2D extent_;
  std::vector<core::Image> images_;
  std::vector<core::ImageView> views_;
};
}  // namespace vkoo
