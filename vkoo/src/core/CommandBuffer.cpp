#include "vkoo/core/CommandBuffer.h"

#include "vkoo/core/CommandPool.h"
#include "vkoo/core/Device.h"
#include "vkoo/core/Subpass.h"
#include "vkoo/logging.h"

namespace vkoo {
CommandBuffer::CommandBuffer(CommandPool& command_pool)
    : command_pool_{command_pool},
      handle_{VK_NULL_HANDLE},
      max_push_constants_size_{command_pool.GetDevice()
                                   .GetGPU()
                                   .GetProperties()
                                   .limits.maxPushConstantsSize} {
  VkCommandBufferAllocateInfo allocate_info{
      VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};

  allocate_info.commandPool = command_pool_.GetHandle();
  allocate_info.commandBufferCount = 1;
  allocate_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;

  VK_CHECK(vkAllocateCommandBuffers(command_pool_.GetDevice().GetHandle(),
                                    &allocate_info, &handle_));
}

CommandBuffer::~CommandBuffer() {
  if (handle_ != VK_NULL_HANDLE) {
    vkFreeCommandBuffers(command_pool_.GetDevice().GetHandle(),
                         command_pool_.GetHandle(), 1, &handle_);
  }
}

void CommandBuffer::Reset() {
  pipeline_state_.Reset();
  resource_binding_state_.Reset();
  stored_push_constants_.clear();
}

void CommandBuffer::Begin(VkCommandBufferUsageFlags flags) {
  // Note: command buffer might already have some resources when requested from
  // the pool.
  pipeline_state_.Reset();
  resource_binding_state_.Reset();
  stored_push_constants_.clear();

  VkCommandBufferBeginInfo begin_info{
      VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  begin_info.flags = flags;
  VK_CHECK(vkBeginCommandBuffer(GetHandle(), &begin_info));
}

void CommandBuffer::End() { VK_CHECK(vkEndCommandBuffer(GetHandle())); }

void CommandBuffer::SetViewport(uint32_t first_viewport,
                                const std::vector<VkViewport>& viewports) {
  vkCmdSetViewport(GetHandle(), first_viewport,
                   static_cast<uint32_t>(viewports.size()), viewports.data());
}

void CommandBuffer::SetScissor(uint32_t first_scissor,
                               const std::vector<VkRect2D>& scissors) {
  vkCmdSetScissor(GetHandle(), first_scissor,
                  static_cast<uint32_t>(scissors.size()), scissors.data());
}

void CommandBuffer::BeginRenderPass(
    const VkExtent2D& extent, const std::vector<VkImageView>& attachments,
    const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
    const std::vector<VkClearValue>& clear_values,
    const std::vector<std::unique_ptr<Subpass>>& subpasses,
    const std::vector<VkSubpassDependency2KHR>& dependencies,
    VkSubpassContents contents) {
  assert(attachments.size() == attachment_descriptions.size() &&
         attachments.size() == clear_values.size());
  RenderPass& render_pass =
      GetRenderPass(attachment_descriptions, subpasses, dependencies);
  Framebuffer& framebuffer = GetDevice().GetResourceCache().RequestFramebuffer(
      extent, attachments, render_pass);
  BeginRenderPass(extent, render_pass, framebuffer, clear_values, contents);
}

void CommandBuffer::BeginRenderPass(
    const VkExtent2D& extent, const RenderPass& render_pass,
    const Framebuffer& framebuffer,
    const std::vector<VkClearValue>& clear_values, VkSubpassContents contents) {
  current_render_pass_.render_pass = &render_pass;
  current_render_pass_.framebuffer = &framebuffer;

  VkRenderPassBeginInfo begin_info{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
  begin_info.renderPass = current_render_pass_.render_pass->GetHandle();
  begin_info.framebuffer = current_render_pass_.framebuffer->GetHandle();
  begin_info.renderArea.extent = extent;
  begin_info.clearValueCount = static_cast<uint32_t>(clear_values.size());
  begin_info.pClearValues = clear_values.data();

  vkCmdBeginRenderPass(GetHandle(), &begin_info, contents);

  // Update blend state attachments for first subpass.
  auto blend_state = pipeline_state_.GetColorBlendState();
  blend_state.attachments.resize(
      current_render_pass_.render_pass->GetColorOutputCount(
          pipeline_state_.GetSubpassIndex()));
  pipeline_state_.SetColorBlendState(blend_state);
}

void CommandBuffer::EndRenderPass() { vkCmdEndRenderPass(GetHandle()); }

RenderPass& CommandBuffer::GetRenderPass(
    const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
    const std::vector<std::unique_ptr<Subpass>>& subpasses,
    const std::vector<VkSubpassDependency2KHR>& dependencies) {
  assert(subpasses.size() > 0 &&
         "Cannot create a render pass without any subpass");

  std::vector<SubpassInfo> subpass_infos;
  for (auto& subpass : subpasses) {
    SubpassInfo info{};
    info.input_attachments = subpass->GetInputAttachments();
    info.output_attachments = subpass->GetOutputAttachments();
    info.color_resolve_attachments = subpass->GetColorResolveAttachments();
    info.depth_stencil_resolve_attachment =
        subpass->GetDepthStencilResolveAttachment();
    info.depth_stencil_resolve_mode = subpass->GetDepthStencilResolveMode();
    info.disable_depth_stencil_attachment =
        subpass->IsDisableDepthStencilAttachment();
    subpass_infos.push_back(info);
  }

  return GetDevice().GetResourceCache().RequestRenderPass(
      attachment_descriptions, subpass_infos, dependencies);
}

Device& CommandBuffer::GetDevice() { return command_pool_.GetDevice(); }

void CommandBuffer::SetRasterizationState(const RasterizationState& state) {
  pipeline_state_.SetRasterizationState(state);
}

void CommandBuffer::SetMultisampleState(const MultisampleState& state) {
  pipeline_state_.SetMultisampleState(state);
}

void CommandBuffer::SetInputAssemblyState(const InputAssemblyState& state) {
  pipeline_state_.SetInputAssemblyState(state);
}

void CommandBuffer::SetVertexInputState(const VertexInputState& state) {
  pipeline_state_.SetVertexInputState(state);
}

void CommandBuffer::SetColorBlendState(const ColorBlendState& state) {
  pipeline_state_.SetColorBlendState(state);
}

void CommandBuffer::SetDepthStencilState(const DepthStencilState& state) {
  pipeline_state_.SetDepthStencilState(state);
}

void CommandBuffer::BindPipelineLayout(PipelineLayout& pipeline_layout) {
  pipeline_state_.SetPipelineLayout(&pipeline_layout);
}

void CommandBuffer::BindVertexBuffers(
    uint32_t first_binding, const std::vector<core::Buffer*>& buffers,
    const std::vector<VkDeviceSize>& offsets) {
  std::vector<VkBuffer> buffer_handles(buffers.size(), VK_NULL_HANDLE);
  std::transform(
      buffers.begin(), buffers.end(), buffer_handles.begin(),
      [](const core::Buffer* buffer) { return buffer->GetHandle(); });
  vkCmdBindVertexBuffers(handle_, first_binding,
                         static_cast<uint32_t>(buffer_handles.size()),
                         buffer_handles.data(), offsets.data());
}

void CommandBuffer::BindIndexBuffer(const core::Buffer& buffer,
                                    VkDeviceSize offset,
                                    VkIndexType index_type) {
  vkCmdBindIndexBuffer(handle_, buffer.GetHandle(), offset, index_type);
}

void CommandBuffer::Draw(uint32_t vertex_count, uint32_t instance_count,
                         uint32_t first_vertex, uint32_t first_instance) {
  Flush(VK_PIPELINE_BIND_POINT_GRAPHICS);

  vkCmdDraw(handle_, vertex_count, instance_count, first_vertex,
            first_instance);
}

void CommandBuffer::DrawIndexed(uint32_t index_count, uint32_t instance_count,
                                uint32_t first_index, int32_t vertex_offset,
                                uint32_t first_instance) {
  Flush(VK_PIPELINE_BIND_POINT_GRAPHICS);

  vkCmdDrawIndexed(handle_, index_count, instance_count, first_index,
                   vertex_offset, first_instance);
}

void CommandBuffer::Flush(VkPipelineBindPoint pipeline_bind_point) {
  // This is where pipeline and descriptor sets are created in a lazy manner.
  FlushPipelineState(pipeline_bind_point);
  FlushPushConstants();
  FlushDescriptorState(pipeline_bind_point);
}

void CommandBuffer::FlushPipelineState(
    VkPipelineBindPoint pipeline_bind_point) {
  // TODO: lazy flush.
  assert(pipeline_bind_point == VK_PIPELINE_BIND_POINT_GRAPHICS);
  pipeline_state_.SetRenderPass(current_render_pass_.render_pass);
  GraphicsPipeline& pipeline =
      GetDevice().GetResourceCache().RequestGraphicsPipeline(pipeline_state_);
  vkCmdBindPipeline(handle_, pipeline_bind_point, pipeline.GetHandle());
}

void CommandBuffer::FlushDescriptorState(
    VkPipelineBindPoint pipeline_bind_point) {
  assert(command_pool_.GetRenderFrame() &&
         "The command pool must be associated to a render frame");

  const PipelineLayout& pipeline_layout = *pipeline_state_.GetPipelineLayout();

  // TODO: for now we just recreate the descriptor set every time.
  for (auto& resource_set_kv : resource_binding_state_.GetResourceSets()) {
    uint32_t descriptor_set_id = resource_set_kv.first;
    const ResourceSet& resource_set = resource_set_kv.second;

    // Skip resource set if a descriptor set layout doesn't exist for it.
    if (!pipeline_layout.HasDescriptorSetLayout(descriptor_set_id)) {
      continue;
    }

    DescriptorSetLayout& descriptor_set_layout =
        pipeline_layout.GetDescriptorSetLayout(descriptor_set_id);

    BindingMap<VkDescriptorBufferInfo> buffer_infos;
    BindingMap<VkDescriptorImageInfo> image_infos;
    // Iterate over all resource bindings.
    for (auto& binding_kv : resource_set.GetResourceBindings()) {
      uint32_t binding_index = binding_kv.first;
      auto& binding_resources = binding_kv.second;

      // Check if binding exists in the pipeline layout.
      if (auto binding_info =
              descriptor_set_layout.GetLayoutBinding(binding_index)) {
        // Iterate over all binding resources.
        for (auto& element_it : binding_resources) {
          uint32_t array_element = element_it.first;
          const ResourceInfo& resource_info = element_it.second;

          const core::Buffer* buffer = resource_info.buffer;
          const core::Sampler* sampler = resource_info.sampler;
          const core::ImageView* image_view = resource_info.image_view;
          if (buffer != nullptr) {
            assert(binding_info->descriptorType ==
                   VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
            VkDescriptorBufferInfo buffer_info{};

            buffer_info.buffer = resource_info.buffer->GetHandle();
            buffer_info.offset = resource_info.offset;
            buffer_info.range = resource_info.range;

            buffer_infos[binding_index][array_element] = buffer_info;
          } else if (image_view != nullptr) {
            VkDescriptorImageInfo image_info{};
            image_info.sampler =
                sampler ? sampler->GetHandle() : VK_NULL_HANDLE;
            image_info.imageView = image_view->GetHandle();
            switch (binding_info->descriptorType) {
              case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
                // image_info.imageLayout =
                //    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                if (IsDepthStencilFormat(image_view->GetFormat())) {
                  image_info.imageLayout =
                      VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
                } else {
                  image_info.imageLayout =
                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                }
                break;
              case VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT:
                if (IsDepthStencilFormat(image_view->GetFormat())) {
                  image_info.imageLayout =
                      VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
                } else {
                  image_info.imageLayout =
                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                }
                break;
              case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
                image_info.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
                break;

              default:
                continue;
            }
            image_infos[binding_index][array_element] = image_info;
          } else {
            throw std::runtime_error(
                fmt::format("No resource is bound to {}!", binding_index));
          }
        }
      }
    }

    DescriptorSet& descriptor_set =
        command_pool_.GetRenderFrame()->RequestDescriptorSet(
            descriptor_set_layout, buffer_infos, image_infos);
    descriptor_set.Update();

    VkDescriptorSet descriptor_set_handle = descriptor_set.GetHandle();

    vkCmdBindDescriptorSets(handle_, pipeline_bind_point,
                            pipeline_layout.GetHandle(), descriptor_set_id, 1,
                            &descriptor_set_handle, 0, nullptr);
  }
}

void CommandBuffer::BindBuffer(const core::Buffer& buffer, VkDeviceSize offset,
                               VkDeviceSize range, uint32_t set,
                               uint32_t binding, uint32_t array_element) {
  resource_binding_state_.BindBuffer(buffer, offset, range, set, binding,
                                     array_element);
}

void CommandBuffer::InsertImageMemoryBarrier(
    const core::ImageView& image_view,
    const ImageMemoryBarrier& memory_barrier) {
  auto subresource_range = image_view.GetSubresourceRange();
  auto format = image_view.GetFormat();
  if (IsDepthOnlyFormat(format)) {
    subresource_range.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
  } else if (IsDepthStencilFormat(format)) {
    subresource_range.aspectMask =
        VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
  }
  InsertImageMemoryBarrier(image_view.GetImage().GetHandle(), memory_barrier,
                           subresource_range);
}

void CommandBuffer::InsertImageMemoryBarrier(
    VkImage image, const ImageMemoryBarrier& memory_barrier,
    VkImageSubresourceRange subresource_range) {
  VkImageMemoryBarrier image_memory_barrier{
      VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
  image_memory_barrier.oldLayout = memory_barrier.old_layout;
  image_memory_barrier.newLayout = memory_barrier.new_layout;
  image_memory_barrier.image = image;
  image_memory_barrier.subresourceRange = subresource_range;
  image_memory_barrier.srcAccessMask = memory_barrier.src_access_mask;
  image_memory_barrier.dstAccessMask = memory_barrier.dst_access_mask;

  VkPipelineStageFlags src_stage_mask = memory_barrier.src_stage_mask;
  VkPipelineStageFlags dst_stage_mask = memory_barrier.dst_stage_mask;

  vkCmdPipelineBarrier(handle_, src_stage_mask, dst_stage_mask, 0, 0, nullptr,
                       0, nullptr, 1, &image_memory_barrier);
}

void CommandBuffer::BindImage(const core::ImageView& image_view, uint32_t set,
                              uint32_t binding, uint32_t array_element) {
  resource_binding_state_.BindImage(image_view, set, binding, array_element);
}

void CommandBuffer::BindImage(const core::ImageView& image_view,
                              const core::Sampler& sampler, uint32_t set,
                              uint32_t binding, uint32_t array_element) {
  resource_binding_state_.BindImage(image_view, sampler, set, binding,
                                    array_element);
}

void CommandBuffer::CopyBufferToImage(
    const core::Buffer& buffer, const core::Image& image,
    const std::vector<VkBufferImageCopy>& regions) {
  vkCmdCopyBufferToImage(handle_, buffer.GetHandle(), image.GetHandle(),
                         VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                         static_cast<uint32_t>(regions.size()), regions.data());
}

void CommandBuffer::FlushPushConstants() {
  if (stored_push_constants_.empty()) {
    return;
  }

  const PipelineLayout& pipeline_layout = *pipeline_state_.GetPipelineLayout();

  VkShaderStageFlags shader_stage = pipeline_layout.GetPushConstantRangeStage(
      static_cast<uint32_t>(stored_push_constants_.size()));

  if (shader_stage) {
    vkCmdPushConstants(handle_, pipeline_layout.GetHandle(), shader_stage, 0,
                       static_cast<uint32_t>(stored_push_constants_.size()),
                       stored_push_constants_.data());
  } else {
    throw std::runtime_error(
        fmt::format("Push constant range [{}, {}] not found", 0,
                    stored_push_constants_.size()));
  }

  stored_push_constants_.clear();
}

void CommandBuffer::BindInput(const core::ImageView& image_view, uint32_t set,
                              uint32_t binding, uint32_t array_element) {
  resource_binding_state_.BindInput(image_view, set, binding, array_element);
}

void CommandBuffer::NextSubpass() {
  pipeline_state_.SetSubpassIndex(pipeline_state_.GetSubpassIndex() + 1);

  auto blend_state = pipeline_state_.GetColorBlendState();
  blend_state.attachments.resize(
      current_render_pass_.render_pass->GetColorOutputCount(
          pipeline_state_.GetSubpassIndex()));
  pipeline_state_.SetColorBlendState(blend_state);

  resource_binding_state_.Reset();
  stored_push_constants_.clear();
  vkCmdNextSubpass(handle_, VK_SUBPASS_CONTENTS_INLINE);
}

}  // namespace vkoo
