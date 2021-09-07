#pragma once

#include "Buffer.h"
#include "ImageView.h"
#include "Sampler.h"
#include "vkoo/common.h"

namespace vkoo {
struct ResourceInfo {
  const core::Buffer* buffer{nullptr};
  VkDeviceSize offset{0};
  VkDeviceSize range{0};
  const core::ImageView* image_view{nullptr};
  const core::Sampler* sampler{nullptr};
};

class ResourceSet {
 public:
  void Reset();
  void BindBuffer(const core::Buffer& buffer, VkDeviceSize offset,
                  VkDeviceSize range, uint32_t binding, uint32_t array_element);

  void BindInput(const core::ImageView& image_view, uint32_t binding,
                 uint32_t array_element);
  void BindImage(const core::ImageView& image_view,
                 const core::Sampler& sampler, uint32_t binding,
                 uint32_t array_element);
  void BindImage(const core::ImageView& image_view, uint32_t binding,
                 uint32_t array_element);

  const BindingMap<ResourceInfo>& GetResourceBindings() const {
    return resource_bindings_;
  }

 private:
  BindingMap<ResourceInfo> resource_bindings_;
};

class ResourceBindingState {
 public:
  void Reset();
  void BindBuffer(const core::Buffer& buffer, VkDeviceSize offset,
                  VkDeviceSize range, uint32_t set, uint32_t binding,
                  uint32_t array_element);

  void BindInput(const core::ImageView& image_view, uint32_t set,
                 uint32_t binding, uint32_t array_element);
  void BindImage(const core::ImageView& image_view,
                 const core::Sampler& sampler, uint32_t set, uint32_t binding,
                 uint32_t array_element);

  void BindImage(const core::ImageView& image_view, uint32_t set,
                 uint32_t binding, uint32_t array_element);

  const std::unordered_map<uint32_t, ResourceSet>& GetResourceSets() {
    return resource_sets_;
  }

 private:
  std::unordered_map<uint32_t, ResourceSet> resource_sets_;
};
}  // namespace vkoo
