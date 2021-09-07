#include "vkoo/core/PhysicalDevice.h"

#include "vkoo/core/Instance.h"

namespace vkoo {
PhysicalDevice::PhysicalDevice(Instance& instance,
                               VkPhysicalDevice physical_device)
    : instance_(instance), handle_(physical_device) {
  vkGetPhysicalDeviceFeatures(physical_device, &features_);
  vkGetPhysicalDeviceProperties(physical_device, &properties_);
  vkGetPhysicalDeviceMemoryProperties(physical_device, &memory_properties_);

  uint32_t queue_family_properties_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(
      physical_device, &queue_family_properties_count, nullptr);
  queue_family_properties_.resize(queue_family_properties_count);
  vkGetPhysicalDeviceQueueFamilyProperties(physical_device,
                                           &queue_family_properties_count,
                                           queue_family_properties_.data());
}
VkBool32 PhysicalDevice::IsPresentSupported(VkSurfaceKHR surface,
                                            uint32_t queue_family_index) const {
  VkBool32 present_supported{VK_FALSE};

  if (surface != VK_NULL_HANDLE) {
    VK_CHECK(vkGetPhysicalDeviceSurfaceSupportKHR(handle_, queue_family_index,
                                                  surface, &present_supported));
  }

  return present_supported;
}
}  // namespace vkoo
