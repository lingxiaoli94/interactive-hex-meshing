#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;
struct Attachment;

struct SubpassInfo {
  std::vector<uint32_t> input_attachments;
  std::vector<uint32_t> output_attachments;
  std::vector<uint32_t> color_resolve_attachments;
  bool disable_depth_stencil_attachment;
  uint32_t depth_stencil_resolve_attachment;
  VkResolveModeFlagBits depth_stencil_resolve_mode;
};

class RenderPass {
 public:
  RenderPass(
      Device& device,
      const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
      const std::vector<SubpassInfo>& subpasses,
      const std::vector<VkSubpassDependency2KHR>& dependencies);
  RenderPass(RenderPass&& other);
  ~RenderPass();

  VkRenderPass GetHandle() const { return handle_; }
  uint32_t GetColorOutputCount(uint32_t subpass_index) const {
    return color_output_count_.at(subpass_index);
  }

 private:
  Device& device_;
  VkRenderPass handle_;
  size_t subpass_count_;
  std::vector<uint32_t> color_output_count_;
};
}  // namespace vkoo
