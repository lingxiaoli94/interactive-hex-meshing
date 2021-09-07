#pragma once

#include "Device.h"
#include "Queue.h"
#include "RenderFrame.h"
#include "Swapchain.h"

namespace vkoo {
class RenderContext {
 public:
  RenderContext(Device& device, VkSurfaceKHR surface, uint32_t window_width,
                uint32_t window_height);

  void Prepare(
      RenderTarget::CreateFunc create_func = RenderTarget::kDefaultCreateFunc);
  CommandBuffer& Begin();
  VkSemaphore BeginFrame();
  RenderFrame& GetActiveFrame();
  void Submit(CommandBuffer& command_buffer);
  void Submit(const std::vector<CommandBuffer*>& command_buffers);
  VkSemaphore Submit(const Queue& queue,
                     const std::vector<CommandBuffer*>& command_buffers,
                     VkSemaphore wait_semaphore,
                     VkPipelineStageFlags wait_pipeline_stage);
  void EndFrame(VkSemaphore render_semaphore);
  Device& GetDevice() { return device_; }
  const VkExtent2D& GetSurfaceExtent() const { return surface_extent_; }
  void UpdateSwapchain();
  VkImage GetLastActiveSwapchainImage() const;
  Swapchain& GetSwapchain() { return *swapchain_; }

 private:
  void HandleWindowResize();
  void Recreate();
  void RecreateSwapchain(const VkExtent2D& extent);

  Device& device_;
  std::unique_ptr<Swapchain> swapchain_;
  const Queue& queue_;
  VkExtent2D surface_extent_;
  std::vector<std::unique_ptr<RenderFrame>> frames_;

  bool prepared_;
  bool is_frame_active_;
  uint32_t active_frame_index_;

  VkSemaphore acquired_semaphore_;  // TODO: is it right to put this here?

  RenderTarget::CreateFunc create_render_target_func_{
      RenderTarget::kDefaultCreateFunc};
};
}  // namespace vkoo
