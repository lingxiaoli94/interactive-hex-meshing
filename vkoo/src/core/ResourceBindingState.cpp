#include "vkoo/core/ResourceBindingState.h"

namespace vkoo {
void ResourceBindingState::Reset() { resource_sets_.clear(); }

void ResourceBindingState::BindBuffer(const core::Buffer& buffer,
                                      VkDeviceSize offset, VkDeviceSize range,
                                      uint32_t set, uint32_t binding,
                                      uint32_t array_element) {
  resource_sets_[set].BindBuffer(buffer, offset, range, binding, array_element);
}

void ResourceBindingState::BindInput(const core::ImageView& image_view,
                                     uint32_t set, uint32_t binding,
                                     uint32_t array_element) {
  resource_sets_[set].BindInput(image_view, binding, array_element);
}

void ResourceBindingState::BindImage(const core::ImageView& image_view,
                                     uint32_t set, uint32_t binding,
                                     uint32_t array_element) {
  resource_sets_[set].BindImage(image_view, binding, array_element);
}

void ResourceBindingState::BindImage(const core::ImageView& image_view,
                                     const core::Sampler& sampler, uint32_t set,
                                     uint32_t binding, uint32_t array_element) {
  resource_sets_[set].BindImage(image_view, sampler, binding, array_element);
}

void ResourceSet::Reset() { resource_bindings_.clear(); }

void ResourceSet::BindBuffer(const core::Buffer& buffer, VkDeviceSize offset,
                             VkDeviceSize range, uint32_t binding,
                             uint32_t array_element) {
  resource_bindings_[binding][array_element].buffer = &buffer;
  resource_bindings_[binding][array_element].offset = offset;
  resource_bindings_[binding][array_element].range = range;
}

void ResourceSet::BindInput(const core::ImageView& image_view,
                            const uint32_t binding,
                            const uint32_t array_element) {
  resource_bindings_[binding][array_element].image_view = &image_view;
}

void ResourceSet::BindImage(const core::ImageView& image_view,
                            const core::Sampler& sampler, uint32_t binding,
                            uint32_t array_element) {
  resource_bindings_[binding][array_element].image_view = &image_view;
  resource_bindings_[binding][array_element].sampler = &sampler;
}

void ResourceSet::BindImage(const core::ImageView& image_view, uint32_t binding,
                            uint32_t array_element) {
  resource_bindings_[binding][array_element].image_view = &image_view;
  resource_bindings_[binding][array_element].sampler = nullptr;
}

}  // namespace vkoo