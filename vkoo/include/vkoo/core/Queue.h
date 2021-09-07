#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;
class CommandBuffer;

class Queue {
 public:
  Queue(Device& device, uint32_t family_index, uint32_t index,
        VkQueueFamilyProperties properties, VkBool32 present_supported);

  VkBool32 IsPresentSupported() const { return present_supported_; }
  uint32_t GetFamilyIndex() const { return family_index_; }
  VkResult Submit(const std::vector<VkSubmitInfo>& submit_infos,
                  VkFence fence) const;
  VkResult Submit(const CommandBuffer& command_buffer, VkFence fence) const;
  VkResult Present(const VkPresentInfoKHR& present_info) const;
  const VkQueueFamilyProperties& GetProperties() const { return properties_; }

 private:
  [[maybe_unused]] Device& device_;
  VkQueue handle_;
  uint32_t family_index_;
  [[maybe_unused]] uint32_t index_;
  VkBool32 present_supported_;
  VkQueueFamilyProperties properties_;
};
}  // namespace vkoo
