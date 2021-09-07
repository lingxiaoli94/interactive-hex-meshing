#include "vkoo/core/Swapchain.h"

#include "vkoo/logging.h"

namespace vkoo {

namespace {
VkSurfaceFormatKHR ChooseSurfaceFormat(
    const std::vector<VkSurfaceFormatKHR>& formats) {
  for (const auto& x : formats) {
    if (x.format == VK_FORMAT_B8G8R8A8_SRGB &&
        x.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
      return x;
    }
  }
  return formats[0];
}

VkPresentModeKHR ChoosePresentMode(
    const std::vector<VkPresentModeKHR>& present_modes) {
  for (const auto& mode : present_modes) {
    if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
      return mode;
    }
  }
  return VK_PRESENT_MODE_FIFO_KHR;
}

uint32_t ChooseImageCount(uint32_t min_count, uint32_t max_count) {
  uint32_t image_count = min_count + 1;
  if (max_count > 0 && image_count > max_count) {
    image_count = max_count;
  }
  return image_count;
}

VkExtent2D ChooseExtent(VkExtent2D request_extent,
                        const VkExtent2D& min_image_extent,
                        const VkExtent2D& max_image_extent,
                        const VkExtent2D& current_extent) {
  if (request_extent.width < 1 || request_extent.height < 1) {
    return current_extent;
  }

  request_extent.width = std::max(request_extent.width, min_image_extent.width);
  request_extent.width = std::min(request_extent.width, max_image_extent.width);

  request_extent.height =
      std::max(request_extent.height, min_image_extent.height);
  request_extent.height =
      std::min(request_extent.height, max_image_extent.height);

  return request_extent;
}

}  // namespace

Swapchain::Swapchain(Swapchain& old_swapchain, const VkExtent2D& new_extent)
    : Swapchain{old_swapchain.GetHandle(), old_swapchain.device_,
                old_swapchain.surface_, new_extent} {
  Create();
}

Swapchain::Swapchain(Device& device, VkSurfaceKHR surface,
                     const VkExtent2D& extent)
    : Swapchain{VK_NULL_HANDLE, device, surface, extent} {}

Swapchain::Swapchain(VkSwapchainKHR old_swapchain, Device& device,
                     VkSurfaceKHR surface, const VkExtent2D& extent)
    : device_{device}, surface_{surface} {
  VkPhysicalDevice gpu_handle = device_.GetGPU().GetHandle();
  VkSurfaceCapabilitiesKHR surface_capabilities{};
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(gpu_handle, surface,
                                            &surface_capabilities);

  uint32_t surface_format_count;
  VK_CHECK(vkGetPhysicalDeviceSurfaceFormatsKHR(
      gpu_handle, surface, &surface_format_count, nullptr));
  surface_formats_.resize(surface_format_count);
  VK_CHECK(vkGetPhysicalDeviceSurfaceFormatsKHR(
      gpu_handle, surface, &surface_format_count, surface_formats_.data()));

  uint32_t present_mode_count;
  VK_CHECK(vkGetPhysicalDeviceSurfacePresentModesKHR(
      gpu_handle, surface, &present_mode_count, nullptr));
  present_modes_.resize(present_mode_count);
  VK_CHECK(vkGetPhysicalDeviceSurfacePresentModesKHR(
      gpu_handle, surface, &present_mode_count, present_modes_.data()));

  properties_.image_count = ChooseImageCount(
      surface_capabilities.minImageCount, surface_capabilities.maxImageCount);
  properties_.surface_format = ChooseSurfaceFormat(surface_formats_);
  properties_.image_usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
  if (surface_capabilities.supportedUsageFlags &
      VK_IMAGE_USAGE_TRANSFER_SRC_BIT) {
    // Screenshot support.
    properties_.image_usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  }
  properties_.present_mode = ChoosePresentMode(present_modes_);
  properties_.extent = ChooseExtent(extent, surface_capabilities.minImageExtent,
                                    surface_capabilities.maxImageExtent,
                                    surface_capabilities.currentExtent);
  properties_.pre_transform = surface_capabilities.currentTransform;
  properties_.old_swapchain = old_swapchain;
}

void Swapchain::Create() {
  VkSwapchainCreateInfoKHR create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  create_info.minImageCount = properties_.image_count;
  create_info.imageExtent = properties_.extent;
  create_info.presentMode = properties_.present_mode;
  create_info.imageFormat = properties_.surface_format.format;
  create_info.imageColorSpace = properties_.surface_format.colorSpace;
  create_info.imageArrayLayers = 1;
  create_info.imageUsage = properties_.image_usage;
  create_info.preTransform = properties_.pre_transform;
  create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  create_info.oldSwapchain = properties_.old_swapchain;
  create_info.surface = surface_;
  create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  create_info.clipped = VK_TRUE;

  VK_CHECK(vkCreateSwapchainKHR(device_.GetHandle(), &create_info, nullptr,
                                &handle_));

  uint32_t image_available_count;
  VK_CHECK(vkGetSwapchainImagesKHR(device_.GetHandle(), handle_,
                                   &image_available_count, nullptr));

  images_.resize(image_available_count);

  VK_CHECK(vkGetSwapchainImagesKHR(device_.GetHandle(), handle_,
                                   &image_available_count, images_.data()));
}

VkFormat Swapchain::GetFormat() const {
  return properties_.surface_format.format;
}

VkImageUsageFlags Swapchain::GetUsage() const {
  return VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
}

VkResult Swapchain::AcquireNextImage(uint32_t& image_index,
                                     VkSemaphore image_acquired_semaphore,
                                     VkFence fence) {
  return vkAcquireNextImageKHR(device_.GetHandle(), handle_,
                               std::numeric_limits<uint64_t>::max(),
                               image_acquired_semaphore, fence, &image_index);
}

Swapchain::~Swapchain() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroySwapchainKHR(device_.GetHandle(), handle_, nullptr);
    handle_ = VK_NULL_HANDLE;
  }
}

}  // namespace vkoo
