#include "vkoo/core/RenderContext.h"

#include "vkoo/core/RenderTarget.h"
#include "vkoo/logging.h"
#include <GLFW/glfw3.h>

namespace vkoo {
RenderContext::RenderContext(Device& device, VkSurfaceKHR surface,
                             uint32_t window_width, uint32_t window_height)
    : device_{device},
      queue_{device.GetSuitableGraphicsQueue()},
      surface_extent_{window_width, window_height},
      prepared_{false},
      is_frame_active_{false},
      active_frame_index_{0} {
  swapchain_ = std::make_unique<Swapchain>(device, surface, surface_extent_);
}

void RenderContext::Prepare(RenderTarget::CreateFunc create_func) {
  device_.WaitIdle();
  swapchain_->Create();

  for (VkImage image_handle : swapchain_->GetImages()) {
    core::Image image(device_, image_handle, surface_extent_,
                      swapchain_->GetFormat(), swapchain_->GetUsage());
    std::unique_ptr<RenderTarget> render_target = create_func(std::move(image));
    frames_.emplace_back(
        std::make_unique<RenderFrame>(device_, std::move(render_target)));
  }

  create_render_target_func_ = create_func;
  prepared_ = true;
}

CommandBuffer& RenderContext::Begin() {
  assert(prepared_);
  acquired_semaphore_ = BeginFrame();

  const Queue& queue = device_.GetSuitableGraphicsQueue();
  return GetActiveFrame().RequestCommandBuffer(queue);
}

VkSemaphore RenderContext::BeginFrame() {
  HandleWindowResize();

  assert(!is_frame_active_);
  RenderFrame& prev_frame = *frames_.at(active_frame_index_);
  VkSemaphore acquired_semaphore = prev_frame.RequestSemaphore();
  VkResult acquire_result = swapchain_->AcquireNextImage(
      active_frame_index_, acquired_semaphore, VK_NULL_HANDLE);
  if (acquire_result == VK_SUBOPTIMAL_KHR ||
      acquire_result == VK_ERROR_OUT_OF_DATE_KHR) {
    HandleWindowResize();
    acquire_result = swapchain_->AcquireNextImage(
        active_frame_index_, acquired_semaphore, VK_NULL_HANDLE);
  }

  if (acquire_result != VK_SUCCESS) {
    throw std::runtime_error("Cannot acquire next image from the swapchain.");
  }

  is_frame_active_ = true;

  // Release resources on the frame that was just acquired.
  GetActiveFrame().Reset();
  return acquired_semaphore;
}

RenderFrame& RenderContext::GetActiveFrame() {
  assert(is_frame_active_);
  return *frames_.at(active_frame_index_);
}

void RenderContext::Submit(CommandBuffer& command_buffer) {
  Submit({&command_buffer});
}

void RenderContext::Submit(const std::vector<CommandBuffer*>& command_buffers) {
  assert(is_frame_active_);
  VkSemaphore render_semaphore =
      Submit(queue_, command_buffers, acquired_semaphore_,
             VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
  EndFrame(render_semaphore);
  acquired_semaphore_ = VK_NULL_HANDLE;
}

VkSemaphore RenderContext::Submit(
    const Queue& queue, const std::vector<CommandBuffer*>& command_buffers,
    VkSemaphore wait_semaphore, VkPipelineStageFlags wait_pipeline_stage) {
  std::vector<VkCommandBuffer> cmd_buf_handles(command_buffers.size(),
                                               VK_NULL_HANDLE);
  std::transform(
      command_buffers.begin(), command_buffers.end(), cmd_buf_handles.begin(),
      [](const CommandBuffer* cmd_buf) { return cmd_buf->GetHandle(); });

  RenderFrame& frame = GetActiveFrame();

  VkSemaphore signal_semaphore = frame.RequestSemaphore();

  VkSubmitInfo submit_info{VK_STRUCTURE_TYPE_SUBMIT_INFO};

  submit_info.commandBufferCount =
      static_cast<uint32_t>(cmd_buf_handles.size());
  submit_info.pCommandBuffers = cmd_buf_handles.data();
  submit_info.waitSemaphoreCount = 1;
  submit_info.pWaitSemaphores = &wait_semaphore;
  submit_info.pWaitDstStageMask = &wait_pipeline_stage;
  submit_info.signalSemaphoreCount = 1;
  submit_info.pSignalSemaphores = &signal_semaphore;

  VkFence fence = frame.RequestFence();

  queue.Submit({submit_info}, fence);

  return signal_semaphore;
}

void RenderContext::HandleWindowResize() {
  VkSurfaceCapabilitiesKHR surface_properties{};
  VK_CHECK(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(
      device_.GetGPU().GetHandle(), swapchain_->GetSurface(),
      &surface_properties));

  if (surface_properties.currentExtent.width != surface_extent_.width ||
      surface_properties.currentExtent.height != surface_extent_.height) {
    device_.WaitIdle();
    // TODO: make this not dependent on glfw.
    while (surface_properties.currentExtent.width < 1 &&
           surface_properties.currentExtent.height < 1) {
      glfwWaitEvents();
      VK_CHECK(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(
          device_.GetGPU().GetHandle(), swapchain_->GetSurface(),
          &surface_properties));
    }

    RecreateSwapchain(surface_properties.currentExtent);
    surface_extent_ = surface_properties.currentExtent;
  }
}

void RenderContext::EndFrame(VkSemaphore render_semaphore) {
  assert(is_frame_active_ && "Frame is not active, please call begin_frame");

  VkSwapchainKHR vk_swapchain = swapchain_->GetHandle();

  VkPresentInfoKHR present_info{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};

  present_info.waitSemaphoreCount = 1;
  present_info.pWaitSemaphores = &render_semaphore;
  present_info.swapchainCount = 1;
  present_info.pSwapchains = &vk_swapchain;
  present_info.pImageIndices = &active_frame_index_;

  VkResult result = queue_.Present(present_info);

  if (result == VK_SUBOPTIMAL_KHR || result == VK_ERROR_OUT_OF_DATE_KHR) {
    HandleWindowResize();
  }

  is_frame_active_ = false;
}

void RenderContext::RecreateSwapchain(const VkExtent2D& new_extent) {
  device_.GetResourceCache().ClearFramebuffers();
  swapchain_ = std::make_unique<Swapchain>(*swapchain_, new_extent);
  Recreate();
}

void RenderContext::UpdateSwapchain() { RecreateSwapchain(surface_extent_); }

void RenderContext::Recreate() {
  VkExtent2D extent = swapchain_->GetExtent();
  auto frame_it = frames_.begin();
  for (auto& image_handle : swapchain_->GetImages()) {
    core::Image new_image{device_, image_handle, extent,
                          swapchain_->GetFormat(), swapchain_->GetUsage()};
    std::unique_ptr<RenderTarget> render_target =
        create_render_target_func_(std::move(new_image));
    if (frame_it != frames_.end()) {
      (*frame_it)->UpdateRenderTarget(std::move(render_target));
    } else {
      frames_.emplace_back(
          std::make_unique<RenderFrame>(device_, std::move(render_target)));
    }
    frame_it++;
  }
}

VkImage RenderContext::GetLastActiveSwapchainImage() const {
  return swapchain_->GetImages()[active_frame_index_];
}

}  // namespace vkoo
