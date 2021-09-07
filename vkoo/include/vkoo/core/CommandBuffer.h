#pragma once

#include "Framebuffer.h"
#include "PipelineState.h"
#include "ResourceBindingState.h"
#include "vkoo/common.h"

namespace vkoo {
class CommandPool;
class Subpass;

class CommandBuffer {
 public:
  struct RenderPassBinding {
    const RenderPass* render_pass;
    const Framebuffer* framebuffer;
  };

  CommandBuffer(CommandPool& command_pool);
  ~CommandBuffer();
  const VkCommandBuffer& GetHandle() const { return handle_; }
  void Flush(VkPipelineBindPoint pipeline_bind_point);
  void Begin(VkCommandBufferUsageFlags flags);
  void End();
  void Reset();

  void SetViewport(uint32_t first_viewport,
                   const std::vector<VkViewport>& viewports);
  void SetScissor(uint32_t first_scissor,
                  const std::vector<VkRect2D>& scissors);

  void BeginRenderPass(
      const VkExtent2D& extent,
      const std::vector<VkImageView>& attachment_views,
      const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
      const std::vector<VkClearValue>& clear_values,
      const std::vector<std::unique_ptr<Subpass>>& subpasses,
      const std::vector<VkSubpassDependency2KHR>& dependencies,
      VkSubpassContents contents);
  void BeginRenderPass(const VkExtent2D& extent, const RenderPass& render_pass,
                       const Framebuffer& framebuffer,
                       const std::vector<VkClearValue>& clear_values,
                       VkSubpassContents contents);

  void EndRenderPass();
  Device& GetDevice();

  void SetVertexInputState(const VertexInputState& state);
  void SetInputAssemblyState(const InputAssemblyState& state);
  void SetRasterizationState(const RasterizationState& state);
  void SetMultisampleState(const MultisampleState& state);
  void SetColorBlendState(const ColorBlendState& state);
  void SetDepthStencilState(const DepthStencilState& state);

  void BindPipelineLayout(PipelineLayout& pipeline_layout);
  void BindVertexBuffers(uint32_t first_binding,
                         const std::vector<core::Buffer*>& buffers,
                         const std::vector<VkDeviceSize>& offsets);
  void BindIndexBuffer(const core::Buffer& buffer, VkDeviceSize offset,
                       VkIndexType index_type);
  void DrawIndexed(uint32_t index_count, uint32_t instance_count,
                   uint32_t first_index, int32_t vertex_offset,
                   uint32_t first_instance);
  void Draw(uint32_t vertex_count, uint32_t instance_count,
            uint32_t first_vertex, uint32_t first_instance);

  void BindBuffer(const core::Buffer& buffer, VkDeviceSize offset,
                  VkDeviceSize range, uint32_t set, uint32_t binding,
                  uint32_t array_element);
  void BindImage(const core::ImageView& image_view, uint32_t set,
                 uint32_t binding, uint32_t array_element);
  void BindImage(const core::ImageView& image_view,
                 const core::Sampler& sampler, uint32_t set, uint32_t binding,
                 uint32_t array_element);
  void BindInput(const core::ImageView& image_view, uint32_t set,
                 uint32_t binding, uint32_t array_element);

  void InsertImageMemoryBarrier(const core::ImageView& image_view,
                                const ImageMemoryBarrier& memory_barrier);
  void InsertImageMemoryBarrier(VkImage image,
                                const ImageMemoryBarrier& memory_barrier,
                                VkImageSubresourceRange subresource_range);

  void CopyBufferToImage(const core::Buffer& buffer, const core::Image& image,
                         const std::vector<VkBufferImageCopy>& regions);

  template <typename T>
  void PushConstants(const T& value) {
    auto data = to_bytes(value);

    uint32_t size =
        static_cast<uint32_t>(stored_push_constants_.size() + data.size());

    if (size > max_push_constants_size_) {
      throw std::runtime_error("Cannot overflow push constant limit");
    }

    stored_push_constants_.insert(stored_push_constants_.end(), data.begin(),
                                  data.end());
  }

  void NextSubpass();

 private:
  void FlushPipelineState(VkPipelineBindPoint pipeline_bind_point);
  void FlushDescriptorState(VkPipelineBindPoint pipeline_bind_point);
  void FlushPushConstants();

  RenderPass& GetRenderPass(
      const std::vector<VkAttachmentDescription2KHR>& attachment_descriptions,
      const std::vector<std::unique_ptr<Subpass>>& subpasses,
      const std::vector<VkSubpassDependency2KHR>& dependencies);

  CommandPool& command_pool_;
  VkCommandBuffer handle_;
  PipelineState pipeline_state_;
  ResourceBindingState resource_binding_state_;
  //  std::unordered_map<uint32_t, DescriptorSetLayout*>
  //      descriptor_set_layout_binding_state_;
  RenderPassBinding current_render_pass_;

  std::vector<uint8_t> stored_push_constants_;
  uint32_t max_push_constants_size_;
};
}  // namespace vkoo
