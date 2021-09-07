#include "vkoo/core/Queue.h"

#include "vkoo/core/CommandBuffer.h"
#include "vkoo/core/Device.h"

namespace vkoo {
Queue::Queue(Device& device, uint32_t family_index, uint32_t index,
             VkQueueFamilyProperties properties, VkBool32 present_supported)
    : device_(device),
      family_index_(family_index),
      index_(index),
      present_supported_(present_supported),
      properties_(properties) {
  vkGetDeviceQueue(device.GetHandle(), family_index, index, &handle_);
}

VkResult Queue::Submit(const std::vector<VkSubmitInfo>& submit_infos,
                       VkFence fence) const {
  return vkQueueSubmit(handle_, static_cast<uint32_t>(submit_infos.size()),
                       submit_infos.data(), fence);
}

VkResult Queue::Submit(const CommandBuffer& command_buffer,
                       VkFence fence) const {
  VkSubmitInfo submit_info{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &command_buffer.GetHandle();

  return Submit({submit_info}, fence);
}

VkResult Queue::Present(const VkPresentInfoKHR& present_info) const {
  if (!present_supported_) {
    throw std::runtime_error(
        "Present is not supported on this queue! Cannot call Present() on it.");
  }

  return vkQueuePresentKHR(handle_, &present_info);
}
}  // namespace vkoo
