#pragma once

#include "PipelineLayout.h"
#include "RenderPass.h"
#include "vkoo/common.h"

namespace vkoo {
struct VertexInputState {
  std::vector<VkVertexInputBindingDescription> bindings;
  std::vector<VkVertexInputAttributeDescription> attributes;
};

struct InputAssemblyState {
  VkPrimitiveTopology topology{VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST};
  VkBool32 primitive_restart_enable{VK_FALSE};
};

struct RasterizationState {
  VkBool32 depth_clamp_enable{VK_FALSE};
  VkBool32 rasterizer_discard_enable{VK_FALSE};
  VkPolygonMode polygon_mode{VK_POLYGON_MODE_FILL};
  VkCullModeFlags cull_mode{VK_CULL_MODE_BACK_BIT};
  VkFrontFace front_face{VK_FRONT_FACE_COUNTER_CLOCKWISE};
  VkBool32 depth_bias_enable{VK_FALSE};
  float line_width{1.0f};
};

struct MultisampleState {
  VkSampleCountFlagBits rasterization_samples{VK_SAMPLE_COUNT_1_BIT};
  VkBool32 sample_shading_enable{VK_FALSE};
  float min_sample_shading{0.0f};
};

struct ViewportState {
  uint32_t viewport_count{1};
  uint32_t scissor_count{1};
};

struct StencilOpState {
  VkStencilOp fail_op{VK_STENCIL_OP_REPLACE};
  VkStencilOp pass_op{VK_STENCIL_OP_REPLACE};
  VkStencilOp depth_fail_op{VK_STENCIL_OP_REPLACE};
  VkCompareOp compare_op{VK_COMPARE_OP_NEVER};
};

struct DepthStencilState {
  VkBool32 depth_test_enable{VK_TRUE};
  VkBool32 depth_write_enable{VK_TRUE};

  VkCompareOp depth_compare_op{VK_COMPARE_OP_LESS};
  VkBool32 depth_bounds_test_enable{VK_FALSE};
  VkBool32 stencil_test_enable{VK_FALSE};
  StencilOpState front{};
  StencilOpState back{};
};

struct ColorBlendAttachmentState {
  VkBool32 blend_enable{VK_FALSE};
  VkBlendFactor src_color_blend_factor{VK_BLEND_FACTOR_ONE};
  VkBlendFactor dst_color_blend_factor{VK_BLEND_FACTOR_ZERO};
  VkBlendOp color_blend_op{VK_BLEND_OP_ADD};
  VkBlendFactor src_alpha_blend_factor{VK_BLEND_FACTOR_ONE};
  VkBlendFactor dst_alpha_blend_factor{VK_BLEND_FACTOR_ZERO};
  VkBlendOp alpha_blend_op{VK_BLEND_OP_ADD};
  VkColorComponentFlags color_write_mask{
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
      VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT};
};

struct ColorBlendState {
  VkBool32 logic_op_enable{VK_FALSE};
  VkLogicOp logic_op{VK_LOGIC_OP_CLEAR};
  std::vector<ColorBlendAttachmentState> attachments;
};

class PipelineState {
 public:
  PipelineState() = default;
  void Reset();

  PipelineLayout* GetPipelineLayout() const;
  void SetPipelineLayout(PipelineLayout* pipeline_layout);
  const RenderPass* GetRenderPass() const;
  void SetRenderPass(const RenderPass* render_pass);
  const VertexInputState& GetVertexInputState() const;
  void SetVertexInputState(const VertexInputState& vertex_input_state);
  const InputAssemblyState& GetInputAssemblyState() const;
  void SetInputAssemblyState(const InputAssemblyState& input_assembly_state);
  const RasterizationState& GetRasterizationState() const;
  void SetRasterizationState(const RasterizationState& rasterization_state);
  const MultisampleState& GetMultisampleState() const;
  void SetMultisampleState(const MultisampleState& multisample_state);
  const ViewportState& GetViewportState() const;
  void SetViewportState(const ViewportState& viewport_state);
  const ColorBlendState& GetColorBlendState() const;
  void SetColorBlendState(const ColorBlendState& color_blend_state);
  uint32_t GetSubpassIndex() const;
  void SetSubpassIndex(uint32_t subpass_index);

  const DepthStencilState& GetDepthStencilState() const;
  void SetDepthStencilState(const DepthStencilState& depth_stencil_state);

 private:
  PipelineLayout* pipeline_layout_{nullptr};
  const RenderPass* render_pass_{nullptr};

  VertexInputState vertex_input_state_{};
  InputAssemblyState input_assembly_state_{};
  RasterizationState rasterization_state_{};
  MultisampleState multisample_state_{};
  ViewportState viewport_state_{};
  DepthStencilState depth_stencil_state_{};
  ColorBlendState color_blend_state_{};
  uint32_t subpass_index{0};
};
}  // namespace vkoo
