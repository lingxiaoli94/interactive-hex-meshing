#include "vkoo/core/Framebuffer.h"

#include "vkoo/core/Device.h"

namespace vkoo {
Framebuffer::Framebuffer(Device& device, const VkExtent2D& extent,
                         const std::vector<VkImageView>& attachments,
                         const RenderPass& render_pass)
    : device_{device}, extent_{extent} {
  VkFramebufferCreateInfo create_info{
      VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};

  create_info.renderPass = render_pass.GetHandle();
  create_info.attachmentCount = static_cast<uint32_t>(attachments.size());
  create_info.pAttachments = attachments.data();
  create_info.width = extent_.width;
  create_info.height = extent_.height;
  create_info.layers = 1;

  VK_CHECK(
      vkCreateFramebuffer(device.GetHandle(), &create_info, nullptr, &handle_));
}

Framebuffer::~Framebuffer() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyFramebuffer(device_.GetHandle(), handle_, nullptr);
  }
}

Framebuffer::Framebuffer(Framebuffer&& other)
    : device_{other.device_}, handle_{other.handle_}, extent_{other.extent_} {
  other.handle_ = VK_NULL_HANDLE;
}

}  // namespace vkoo