#pragma once

#include "CommandBuffer.h"
#include "vkoo/common.h"

namespace vkoo {
class Device;
class RenderFrame;

class CommandPool {
 public:
  CommandPool(Device& device, uint32_t queue_family_index,
              RenderFrame* render_frame = nullptr);
  ~CommandPool();
  VkCommandPool GetHandle() const { return handle_; }
  Device& GetDevice() { return device_; }
  void ResetPool();
  CommandBuffer& RequestCommandBuffer();
  RenderFrame* GetRenderFrame() const { return render_frame_; }

 private:
  void ResetCommandBuffers();

  Device& device_;
  RenderFrame* render_frame_{nullptr};
  VkCommandPool handle_;
  uint32_t active_command_buffer_count_;
  std::vector<std::unique_ptr<CommandBuffer>> command_buffers_;
};
}  // namespace vkoo
