#pragma once

#include "CommandBuffer.h"
#include "RenderTarget.h"
#include "Subpass.h"

namespace vkoo {
class RenderPipeline {
 public:
  RenderPipeline(std::vector<std::unique_ptr<Subpass>>&& subpasses = {});
  virtual ~RenderPipeline() = default;
  RenderPipeline(RenderPipeline&&) = default;

  void Prepare();

  void AddSubpass(std::unique_ptr<Subpass>&& subpass);

  std::vector<std::unique_ptr<Subpass>>& GetSubpasses();

  void Draw(
      CommandBuffer& command_buffer, const VkExtent2D& extent,
      const std::vector<VkImageView>& attachments,
      const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
      const std::vector<VkSubpassDependency2KHR>& dependencies);

  const std::vector<VkClearValue>& GetClearValue() const;
  void SetClearValue(const std::vector<VkClearValue>& clear_value);

 private:
  std::vector<std::unique_ptr<Subpass>> subpasses_;
  std::vector<VkClearValue> clear_value_;
  size_t active_subpass_index_{0};
};
}  // namespace vkoo
