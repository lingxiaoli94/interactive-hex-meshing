#pragma once

#include "Device.h"

namespace vkoo {
struct SwapchainProperties {
  VkSwapchainKHR old_swapchain{VK_NULL_HANDLE};
  uint32_t image_count;
  VkExtent2D extent;
  VkSurfaceFormatKHR surface_format;
  VkImageUsageFlags image_usage;
  VkPresentModeKHR present_mode;
  VkSurfaceTransformFlagBitsKHR pre_transform;
};

class Swapchain {
 public:
  Swapchain(Device& device, VkSurfaceKHR surface, const VkExtent2D& extent);
  Swapchain(VkSwapchainKHR old_swapchain, Device& device, VkSurfaceKHR surface,
            const VkExtent2D& extent);

  // Special constructor from existing swapchain: to make sure the destructor of
  // old_swapchain is called after the swapchain is created, Create() will be
  // called automatically at the end of this constructor.
  Swapchain(Swapchain& old_swapchain, const VkExtent2D& new_extent);

  ~Swapchain();
  void Create();
  VkResult AcquireNextImage(uint32_t& image_index,
                            VkSemaphore image_acquired_semaphore,
                            VkFence fence);

  VkSwapchainKHR GetHandle() const { return handle_; }
  const std::vector<VkImage>& GetImages() const { return images_; }
  VkFormat GetFormat() const;
  VkImageUsageFlags GetUsage() const;
  VkSurfaceKHR GetSurface() const { return surface_; }
  const VkExtent2D& GetExtent() const { return properties_.extent; }

 private:
  Device& device_;
  VkSurfaceKHR surface_;
  std::vector<VkSurfaceFormatKHR> surface_formats_;
  std::vector<VkPresentModeKHR> present_modes_;

  SwapchainProperties properties_;
  VkSwapchainKHR handle_;
  std::vector<VkImage> images_;
};
}  // namespace vkoo
