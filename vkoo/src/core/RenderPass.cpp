#include "vkoo/core/RenderPass.h"

#include <stdexcept>

#include "vkoo/core/Device.h"
#include "vkoo/core/RenderTarget.h"

namespace vkoo {
namespace {
VkAttachmentReference2KHR GetAttachmentReference(const uint32_t attachment,
                                                 const VkImageLayout layout) {
  VkAttachmentReference2KHR reference{};

  reference.sType = VK_STRUCTURE_TYPE_ATTACHMENT_REFERENCE_2_KHR;
  reference.attachment = attachment;
  reference.layout = layout;

  return reference;
}
}  // namespace

RenderPass::RenderPass(Device& device,
                       const std::vector<VkAttachmentDescription2KHR>&
                           attachment_descriptions_const,
                       const std::vector<SubpassInfo>& subpasses,
                       const std::vector<VkSubpassDependency2KHR>& dependencies)
    : device_(device), subpass_count_(subpasses.size()) {
  // TODO: might need to modify depth resolve attachment's initial layout.
  auto attachment_descriptions = attachment_descriptions_const;
#ifndef NDEBUG
  assert(subpasses.size() >= 1);
  for (auto& attachment : attachment_descriptions) {
    assert(attachment.sType == VK_STRUCTURE_TYPE_ATTACHMENT_DESCRIPTION_2_KHR);
  }
  for (auto& dep : dependencies) {
    assert(dep.sType == VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2_KHR);
  }
#endif

  std::vector<std::vector<VkAttachmentReference2KHR>> color_attachments{
      subpass_count_};
  std::vector<std::vector<VkAttachmentReference2KHR>> input_attachments{
      subpass_count_};
  std::vector<std::vector<VkAttachmentReference2KHR>> depth_stencil_attachments{
      subpass_count_};
  std::vector<std::vector<VkAttachmentReference2KHR>> color_resolve_attachments{
      subpass_count_};
  std::vector<std::vector<VkAttachmentReference2KHR>> depth_resolve_attachments{
      subpass_count_};

  for (size_t i = 0; i < subpasses.size(); i++) {
    const SubpassInfo& subpass = subpasses[i];
    for (auto o_attachment : subpass.output_attachments) {
      auto initial_layout =
          attachment_descriptions[o_attachment].initialLayout ==
                  VK_IMAGE_LAYOUT_UNDEFINED
              ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
              : attachment_descriptions[o_attachment].initialLayout;
      auto& description = attachment_descriptions[o_attachment];
      if (!IsDepthStencilFormat(description.format)) {
        color_attachments[i].push_back(
            GetAttachmentReference(o_attachment, initial_layout));
      }
    }

    for (auto i_attachment : subpass.input_attachments) {
      auto default_layout =
          IsDepthStencilFormat(attachment_descriptions[i_attachment].format)
              ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
              : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      auto initial_layout =
          attachment_descriptions[i_attachment].initialLayout ==
                  VK_IMAGE_LAYOUT_UNDEFINED
              ? default_layout
              : attachment_descriptions[i_attachment].initialLayout;
      input_attachments[i].push_back(
          GetAttachmentReference(i_attachment, initial_layout));
    }

    for (auto r_attachment : subpass.color_resolve_attachments) {
      auto initial_layout =
          attachment_descriptions[r_attachment].initialLayout ==
                  VK_IMAGE_LAYOUT_UNDEFINED
              ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
              : attachment_descriptions[r_attachment].initialLayout;
      color_resolve_attachments[i].push_back(
          GetAttachmentReference(r_attachment, initial_layout));
    }

    if (!subpass.disable_depth_stencil_attachment) {
      // Assume depth stencil attachment appears before any depth resolve
      // attachment.
      auto it = find_if(attachment_descriptions.begin(),
                        attachment_descriptions.end(),
                        [](const VkAttachmentDescription2KHR& attachment) {
                          return IsDepthStencilFormat(attachment.format);
                        });
      if (it != attachment_descriptions.end()) {
        auto i_depth_stencil = static_cast<uint32_t>(
            std::distance(attachment_descriptions.begin(), it));
        auto initial_layout =
            it->initialLayout == VK_IMAGE_LAYOUT_UNDEFINED
                ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
                : it->initialLayout;
        depth_stencil_attachments[i].push_back(
            GetAttachmentReference(i_depth_stencil, initial_layout));

        if (subpass.depth_stencil_resolve_mode != VK_RESOLVE_MODE_NONE) {
          auto i_depth_stencil_resolve =
              subpass.depth_stencil_resolve_attachment;
          initial_layout =
              attachment_descriptions[i_depth_stencil_resolve].initialLayout ==
                      VK_IMAGE_LAYOUT_UNDEFINED
                  ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
                  : attachment_descriptions[i_depth_stencil_resolve]
                        .initialLayout;
          depth_resolve_attachments[i].push_back(
              GetAttachmentReference(i_depth_stencil_resolve, initial_layout));
        }
      }
    }
  }

  std::vector<VkSubpassDescription2KHR> subpass_descriptions;
  VkSubpassDescriptionDepthStencilResolveKHR depth_resolve{};
  for (size_t i = 0; i < subpasses.size(); ++i) {
    VkSubpassDescription2KHR subpass_description{};
    subpass_description.sType = VK_STRUCTURE_TYPE_SUBPASS_DESCRIPTION_2_KHR;
    subpass_description.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;

    subpass_description.pColorAttachments =
        color_attachments[i].empty() ? nullptr : color_attachments[i].data();
    subpass_description.colorAttachmentCount =
        static_cast<uint32_t>(color_attachments[i].size());

    subpass_description.pInputAttachments =
        input_attachments[i].empty() ? nullptr : input_attachments[i].data();
    subpass_description.inputAttachmentCount =
        static_cast<uint32_t>(input_attachments[i].size());

    subpass_description.pResolveAttachments =
        color_resolve_attachments[i].empty()
            ? nullptr
            : color_resolve_attachments[i].data();

    if (!depth_stencil_attachments[i].empty()) {
      subpass_description.pDepthStencilAttachment =
          depth_stencil_attachments[i].data();
      if (!depth_resolve_attachments[i].empty()) {
        auto& subpass = subpasses[i];
        depth_resolve.sType =
            VK_STRUCTURE_TYPE_SUBPASS_DESCRIPTION_DEPTH_STENCIL_RESOLVE_KHR;
        depth_resolve.depthResolveMode = subpass.depth_stencil_resolve_mode;
        depth_resolve.pDepthStencilResolveAttachment =
            &depth_resolve_attachments[i][0];
        subpass_description.pNext = &depth_resolve;
      }
    } else {
      subpass_description.pDepthStencilAttachment = nullptr;
    }

    subpass_descriptions.push_back(subpass_description);
  }

  for (size_t i = 0; i < subpass_count_; i++) {
    color_output_count_.push_back(
        static_cast<uint32_t>(color_attachments[i].size()));
  }

  VkRenderPassCreateInfo2KHR create_info{
      VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
  create_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO_2_KHR;
  create_info.attachmentCount =
      static_cast<uint32_t>(attachment_descriptions.size());
  create_info.pAttachments = attachment_descriptions.data();
  create_info.subpassCount = static_cast<uint32_t>(subpass_descriptions.size());
  create_info.pSubpasses = subpass_descriptions.data();
  create_info.dependencyCount = static_cast<uint32_t>(dependencies.size());
  create_info.pDependencies = dependencies.data();

  auto create_fn = (PFN_vkCreateRenderPass2KHR)vkGetDeviceProcAddr(
      device_.GetHandle(), "vkCreateRenderPass2KHR");
  if (!create_fn) {
    throw std::runtime_error("Depth resolve extension not availabe/enabled!");
  }
  VK_CHECK(create_fn(device_.GetHandle(), &create_info, nullptr, &handle_));
}

RenderPass::RenderPass(RenderPass&& other)
    : device_{other.device_},
      handle_{other.handle_},
      subpass_count_{other.subpass_count_},
      color_output_count_{other.color_output_count_} {
  other.handle_ = VK_NULL_HANDLE;
}

RenderPass::~RenderPass() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyRenderPass(device_.GetHandle(), handle_, nullptr);
  }
}
}  // namespace vkoo
