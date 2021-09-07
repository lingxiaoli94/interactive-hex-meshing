#include "vkoo/core/RenderPipeline.h"

namespace vkoo {
RenderPipeline::RenderPipeline(
    std::vector<std::unique_ptr<Subpass>>&& subpasses)
    : subpasses_{std::move(subpasses)} {
  Prepare();
}

void RenderPipeline::Prepare() {
  for (auto& subpass : subpasses_) {
    subpass->Prepare();
  }
}

void RenderPipeline::AddSubpass(std::unique_ptr<Subpass>&& subpass) {
  subpass->Prepare();
  subpasses_.emplace_back(std::move(subpass));
}

std::vector<std::unique_ptr<Subpass>>& RenderPipeline::GetSubpasses() {
  return subpasses_;
}

const std::vector<VkClearValue>& RenderPipeline::GetClearValue() const {
  return clear_value_;
}

void RenderPipeline::SetClearValue(
    const std::vector<VkClearValue>& clear_value) {
  clear_value_ = clear_value;
}

void RenderPipeline::Draw(
    CommandBuffer& command_buffer, const VkExtent2D& extent,
    const std::vector<VkImageView>& attachments,
    const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
    const std::vector<VkSubpassDependency2KHR>& dependencies) {
  assert(!subpasses_.empty() &&
         "Render pipeline should contain at least one sub-pass");

  for (size_t i = 0; i < subpasses_.size(); ++i) {
    active_subpass_index_ = i;
    auto& subpass = subpasses_[i];

    if (i == 0) {
      command_buffer.BeginRenderPass(
          extent, attachments, attachment_descriptions, clear_value_,
          subpasses_, dependencies, VK_SUBPASS_CONTENTS_INLINE);
    } else {
      command_buffer.NextSubpass();
    }

    subpass->Draw(command_buffer);
  }
  active_subpass_index_ = 0;
}

}  // namespace vkoo
