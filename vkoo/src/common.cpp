#include "vkoo/common.h"

namespace vkoo {
std::string GetErrorString(VkResult error_code) {
  switch (error_code) {
#define STR(r) \
  case VK_##r: \
    return #r
    STR(NOT_READY);
    STR(TIMEOUT);
    STR(EVENT_SET);
    STR(EVENT_RESET);
    STR(INCOMPLETE);
    STR(ERROR_OUT_OF_HOST_MEMORY);
    STR(ERROR_OUT_OF_DEVICE_MEMORY);
    STR(ERROR_INITIALIZATION_FAILED);
    STR(ERROR_DEVICE_LOST);
    STR(ERROR_MEMORY_MAP_FAILED);
    STR(ERROR_LAYER_NOT_PRESENT);
    STR(ERROR_EXTENSION_NOT_PRESENT);
    STR(ERROR_FEATURE_NOT_PRESENT);
    STR(ERROR_INCOMPATIBLE_DRIVER);
    STR(ERROR_TOO_MANY_OBJECTS);
    STR(ERROR_FORMAT_NOT_SUPPORTED);
    STR(ERROR_SURFACE_LOST_KHR);
    STR(ERROR_NATIVE_WINDOW_IN_USE_KHR);
    STR(SUBOPTIMAL_KHR);
    STR(ERROR_OUT_OF_DATE_KHR);
    STR(ERROR_INCOMPATIBLE_DISPLAY_KHR);
    STR(ERROR_VALIDATION_FAILED_EXT);
    STR(ERROR_INVALID_SHADER_NV);
#undef STR
    default:
      return "UNKNOWN_ERROR";
  }
}

bool IsDepthOnlyFormat(VkFormat format) {
  return format == VK_FORMAT_D16_UNORM || format == VK_FORMAT_D32_SFLOAT;
}

bool IsDepthStencilFormat(VkFormat format) {
  return format == VK_FORMAT_D16_UNORM_S8_UINT ||
         format == VK_FORMAT_D24_UNORM_S8_UINT ||
         format == VK_FORMAT_D32_SFLOAT_S8_UINT || IsDepthOnlyFormat(format);
}

VkFormat GetSuitableDepthFormat(
    VkPhysicalDevice physical_device, bool depth_only,
    const std::vector<VkFormat>& depth_format_priority_list) {
  VkFormat depth_format{VK_FORMAT_UNDEFINED};

  for (auto& format : depth_format_priority_list) {
    if (depth_only && !IsDepthOnlyFormat(format)) {
      continue;
    }

    VkFormatProperties properties;
    vkGetPhysicalDeviceFormatProperties(physical_device, format, &properties);

    // Format must support depth stencil attachment for optimal tiling.
    if (properties.optimalTilingFeatures &
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) {
      depth_format = format;
      break;
    }
  }

  if (depth_format != VK_FORMAT_UNDEFINED) {
    return depth_format;
  }

  throw std::runtime_error("No suitable depth format could be determined");
}

}  // namespace vkoo