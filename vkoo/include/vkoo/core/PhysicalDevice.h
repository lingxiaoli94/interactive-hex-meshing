#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Instance;

class PhysicalDevice {
 public:
  PhysicalDevice(Instance& instance, VkPhysicalDevice physical_device);

  VkPhysicalDevice GetHandle() const { return handle_; }
  VkPhysicalDeviceProperties GetProperties() const { return properties_; }
  const std::vector<VkQueueFamilyProperties>& GetQueueFamilyProperties() const {
    return queue_family_properties_;
  }
  VkBool32 IsPresentSupported(VkSurfaceKHR surface,
                              uint32_t queue_family_index) const;

 private:
  [[maybe_unused]] Instance& instance_;
  VkPhysicalDevice handle_;
  VkPhysicalDeviceFeatures features_;
  VkPhysicalDeviceProperties properties_;
  VkPhysicalDeviceMemoryProperties memory_properties_;
  std::vector<VkQueueFamilyProperties> queue_family_properties_;
};
}  // namespace vkoo
