#include "vkoo/core/CommandPool.h"

#include "vkoo/core/Device.h"

namespace vkoo {
CommandPool::CommandPool(Device& device, uint32_t queue_family_index,
                         RenderFrame* render_frame)
    : device_{device},
      render_frame_{render_frame},
      active_command_buffer_count_{0} {
  VkCommandPoolCreateInfo create_info{
      VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};

  create_info.queueFamilyIndex = queue_family_index;
  create_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;

  VK_CHECK(vkCreateCommandPool(device_.GetHandle(), &create_info, nullptr,
                               &handle_));
}

CommandPool::~CommandPool() {
  command_buffers_.clear();
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyCommandPool(device_.GetHandle(), handle_, nullptr);
  }
}

void CommandPool::ResetPool() {
  VK_CHECK(vkResetCommandPool(device_.GetHandle(), handle_, 0));
  ResetCommandBuffers();
}

void CommandPool::ResetCommandBuffers() {
  for (auto& cmd_buf : command_buffers_) {
    cmd_buf->Reset();
  }
  active_command_buffer_count_ = 0;
}

CommandBuffer& CommandPool::RequestCommandBuffer() {
  if (active_command_buffer_count_ < command_buffers_.size()) {
    return *command_buffers_.at(active_command_buffer_count_);
  }
  command_buffers_.emplace_back(std::make_unique<CommandBuffer>(*this));
  active_command_buffer_count_++;
  return *command_buffers_.back();
}
}  // namespace vkoo
