#pragma once

#include "RenderPass.h"
#include "RenderTarget.h"

namespace vkoo {
class Device;

class Framebuffer {
 public:
  Framebuffer(Device& device, const VkExtent2D& extent,
              const std::vector<VkImageView>& attachments,
              const RenderPass& render_pass);

  Framebuffer(Framebuffer&& other);

  ~Framebuffer();

  VkFramebuffer GetHandle() const { return handle_; }
  const VkExtent2D& GetExtent() const { return extent_; }

 private:
  Device& device_;
  VkFramebuffer handle_{VK_NULL_HANDLE};
  VkExtent2D extent_{};
};
}  // namespace vkoo
