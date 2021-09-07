#pragma once

#ifndef GLM_FORCE_RADIANS
#define GLM_FORCE_RADIANS
#endif
#ifndef GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/hash.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/norm.hpp>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <typeindex>
#include <vector>

namespace vkoo {
std::string GetErrorString(VkResult error_code);

#define VK_CHECK(f)                                                            \
  {                                                                            \
    VkResult res = (f);                                                        \
    if (res != VK_SUCCESS) {                                                   \
      std::cout << "Fatal : VkResult is \"" << GetErrorString(res) << "\" in " \
                << __FILE__ << " at line " << __LINE__ << "\n";                \
      assert(res == VK_SUCCESS);                                               \
    }                                                                          \
  }

bool IsDepthOnlyFormat(VkFormat format);
bool IsDepthStencilFormat(VkFormat format);
VkFormat GetSuitableDepthFormat(
    VkPhysicalDevice physical_device, bool depth_only = false,
    const std::vector<VkFormat>& depth_format_priority_list = {
        VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT,
        VK_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D16_UNORM_S8_UINT,
        VK_FORMAT_D16_UNORM});

const float kPi = std::atan(1.0f) * 4;

template <class T>
using BindingMap = std::map<uint32_t, std::map<uint32_t, T>>;

struct LoadStoreInfo {
  VkAttachmentLoadOp load_op = VK_ATTACHMENT_LOAD_OP_CLEAR;
  VkAttachmentStoreOp store_op = VK_ATTACHMENT_STORE_OP_STORE;
};

template <typename T>
inline std::vector<uint8_t> to_bytes(const T& value) {
  return std::vector<uint8_t>{
      reinterpret_cast<const uint8_t*>(&value),
      reinterpret_cast<const uint8_t*>(&value) + sizeof(T)};
}

struct ImageMemoryBarrier {
  VkPipelineStageFlags src_stage_mask{VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT};
  VkPipelineStageFlags dst_stage_mask{VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT};
  VkAccessFlags src_access_mask{0};
  VkAccessFlags dst_access_mask{0};
  VkImageLayout old_layout{VK_IMAGE_LAYOUT_UNDEFINED};
  VkImageLayout new_layout{VK_IMAGE_LAYOUT_UNDEFINED};
};

template <class T>
inline void hash_combine(size_t& seed, const T& v) {
  std::hash<T> hasher;
  glm::detail::hash_combine(seed, hasher(v));
}

}  // namespace vkoo
