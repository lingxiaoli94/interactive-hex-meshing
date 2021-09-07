#include "vkoo/core/PipelineState.h"

namespace vkoo {
PipelineLayout* PipelineState::GetPipelineLayout() const {
  return pipeline_layout_;
}

void PipelineState::SetPipelineLayout(PipelineLayout* pipeline_layout) {
  pipeline_layout_ = pipeline_layout;
}

const RenderPass* PipelineState::GetRenderPass() const { return render_pass_; }

void PipelineState::SetRenderPass(const RenderPass* render_pass) {
  render_pass_ = render_pass;
}

const VertexInputState& PipelineState::GetVertexInputState() const {
  return vertex_input_state_;
}

void PipelineState::SetVertexInputState(
    const VertexInputState& vertex_input_state) {
  vertex_input_state_ = vertex_input_state;
}

const InputAssemblyState& PipelineState::GetInputAssemblyState() const {
  return input_assembly_state_;
}

void PipelineState::SetInputAssemblyState(
    const InputAssemblyState& input_assembly_state) {
  input_assembly_state_ = input_assembly_state;
}

const RasterizationState& PipelineState::GetRasterizationState() const {
  return rasterization_state_;
}

void PipelineState::SetRasterizationState(
    const RasterizationState& rasterization_state) {
  rasterization_state_ = rasterization_state;
}

const MultisampleState& PipelineState::GetMultisampleState() const {
  return multisample_state_;
}

void PipelineState::SetMultisampleState(
    const MultisampleState& multisample_state) {
  multisample_state_ = multisample_state;
}

const ViewportState& PipelineState::GetViewportState() const {
  return viewport_state_;
}

void PipelineState::SetViewportState(const ViewportState& viewport_state) {
  viewport_state_ = viewport_state;
}

const ColorBlendState& PipelineState::GetColorBlendState() const {
  return color_blend_state_;
}

void PipelineState::SetColorBlendState(
    const ColorBlendState& color_blend_state) {
  color_blend_state_ = color_blend_state;
}

uint32_t PipelineState::GetSubpassIndex() const { return subpass_index; }

void PipelineState::SetSubpassIndex(uint32_t subpass_index) {
  PipelineState::subpass_index = subpass_index;
}

void PipelineState::Reset() {
  pipeline_layout_ = nullptr;
  render_pass_ = nullptr;
  vertex_input_state_ = {};
  input_assembly_state_ = {};
  rasterization_state_ = {};
  multisample_state_ = {};
  viewport_state_ = {};
  depth_stencil_state_ = {};
  color_blend_state_ = {};
  subpass_index = {0};
}

const DepthStencilState& PipelineState::GetDepthStencilState() const {
  return depth_stencil_state_;
}
void PipelineState::SetDepthStencilState(
    const DepthStencilState& depth_stencil_state) {
  depth_stencil_state_ = depth_stencil_state;
}
}  // namespace vkoo
