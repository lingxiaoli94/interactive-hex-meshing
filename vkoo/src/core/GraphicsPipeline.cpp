#include "vkoo/core/GraphicsPipeline.h"

#include "vkoo/core/Device.h"
#include "vkoo/logging.h"

namespace vkoo {
GraphicsPipeline::GraphicsPipeline(Device& device,
                                   PipelineState& pipeline_state)
    : device_(device) {
  std::vector<VkShaderModule> shader_modules;

  std::vector<VkPipelineShaderStageCreateInfo> stage_create_infos;

  for (const ShaderModule* shader_module :
       pipeline_state.GetPipelineLayout()->GetShaderModules()) {
    VkPipelineShaderStageCreateInfo stage_create_info{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};

    stage_create_info.stage = shader_module->GetStage();
    stage_create_info.pName = shader_module->GetEntryPoint().c_str();

    VkShaderModuleCreateInfo vk_create_info{
        VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};

    vk_create_info.codeSize =
        shader_module->GetBinary().size() * sizeof(uint32_t);
    vk_create_info.pCode = shader_module->GetBinary().data();

    VK_CHECK(vkCreateShaderModule(device_.GetHandle(), &vk_create_info, nullptr,
                                  &stage_create_info.module));

    stage_create_infos.push_back(stage_create_info);
    shader_modules.push_back(stage_create_info.module);
  }

  VkGraphicsPipelineCreateInfo create_info{
      VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};

  create_info.stageCount = static_cast<uint32_t>(stage_create_infos.size());
  create_info.pStages = stage_create_infos.data();

  VkPipelineVertexInputStateCreateInfo vertex_input_state{
      VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};

  vertex_input_state.pVertexAttributeDescriptions =
      pipeline_state.GetVertexInputState().attributes.data();
  vertex_input_state.vertexAttributeDescriptionCount = static_cast<uint32_t>(
      pipeline_state.GetVertexInputState().attributes.size());

  vertex_input_state.pVertexBindingDescriptions =
      pipeline_state.GetVertexInputState().bindings.data();
  vertex_input_state.vertexBindingDescriptionCount = static_cast<uint32_t>(
      pipeline_state.GetVertexInputState().bindings.size());

  VkPipelineInputAssemblyStateCreateInfo input_assembly_state{
      VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};

  input_assembly_state.topology =
      pipeline_state.GetInputAssemblyState().topology;
  input_assembly_state.primitiveRestartEnable =
      pipeline_state.GetInputAssemblyState().primitive_restart_enable;

  VkPipelineViewportStateCreateInfo viewport_state{
      VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};

  viewport_state.viewportCount =
      pipeline_state.GetViewportState().viewport_count;
  viewport_state.scissorCount = pipeline_state.GetViewportState().scissor_count;

  VkPipelineRasterizationStateCreateInfo rasterization_state{
      VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};

  rasterization_state.depthClampEnable =
      pipeline_state.GetRasterizationState().depth_clamp_enable;
  rasterization_state.rasterizerDiscardEnable =
      pipeline_state.GetRasterizationState().rasterizer_discard_enable;
  rasterization_state.polygonMode =
      pipeline_state.GetRasterizationState().polygon_mode;
  rasterization_state.cullMode =
      pipeline_state.GetRasterizationState().cull_mode;
  rasterization_state.frontFace =
      pipeline_state.GetRasterizationState().front_face;
  rasterization_state.depthBiasEnable =
      pipeline_state.GetRasterizationState().depth_bias_enable;
  // rasterization_state.depthBiasClamp = 1.0f;
  // rasterization_state.depthBiasSlopeFactor = 1.0f;
  rasterization_state.lineWidth =
      pipeline_state.GetRasterizationState().line_width;

  VkPipelineMultisampleStateCreateInfo multisample_state{
      VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};

  multisample_state.rasterizationSamples =
      pipeline_state.GetMultisampleState().rasterization_samples;
  multisample_state.sampleShadingEnable =
      pipeline_state.GetMultisampleState().sample_shading_enable;
  multisample_state.minSampleShading =
      pipeline_state.GetMultisampleState().min_sample_shading;

  VkPipelineDepthStencilStateCreateInfo depth_stencil_state{
      VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};

  depth_stencil_state.depthTestEnable =
      pipeline_state.GetDepthStencilState().depth_test_enable;
  depth_stencil_state.depthWriteEnable =
      pipeline_state.GetDepthStencilState().depth_write_enable;
  depth_stencil_state.depthCompareOp =
      pipeline_state.GetDepthStencilState().depth_compare_op;
  depth_stencil_state.depthBoundsTestEnable =
      pipeline_state.GetDepthStencilState().depth_bounds_test_enable;

  VkPipelineColorBlendStateCreateInfo color_blend_state{
      VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};

  color_blend_state.logicOpEnable =
      pipeline_state.GetColorBlendState().logic_op_enable;
  color_blend_state.logicOp = pipeline_state.GetColorBlendState().logic_op;
  color_blend_state.attachmentCount = static_cast<uint32_t>(
      pipeline_state.GetColorBlendState().attachments.size());
  color_blend_state.pAttachments =
      reinterpret_cast<const VkPipelineColorBlendAttachmentState*>(
          pipeline_state.GetColorBlendState().attachments.data());
  color_blend_state.blendConstants[0] = 1.0f;
  color_blend_state.blendConstants[1] = 1.0f;
  color_blend_state.blendConstants[2] = 1.0f;
  color_blend_state.blendConstants[3] = 1.0f;

  std::array<VkDynamicState, 2> dynamic_states{
      VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR,
      // VK_DYNAMIC_STATE_LINE_WIDTH
      //      VK_DYNAMIC_STATE_DEPTH_BIAS,
      //      VK_DYNAMIC_STATE_BLEND_CONSTANTS,
      //      VK_DYNAMIC_STATE_DEPTH_BOUNDS,
      //      VK_DYNAMIC_STATE_STENCIL_COMPARE_MASK,
      //      VK_DYNAMIC_STATE_STENCIL_WRITE_MASK,
      //      VK_DYNAMIC_STATE_STENCIL_REFERENCE,
  };

  VkPipelineDynamicStateCreateInfo dynamic_state{
      VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};

  dynamic_state.pDynamicStates = dynamic_states.data();
  dynamic_state.dynamicStateCount =
      static_cast<uint32_t>(dynamic_states.size());

  create_info.pVertexInputState = &vertex_input_state;
  create_info.pInputAssemblyState = &input_assembly_state;
  create_info.pViewportState = &viewport_state;
  create_info.pRasterizationState = &rasterization_state;
  create_info.pMultisampleState = &multisample_state;
  create_info.pDepthStencilState = &depth_stencil_state;
  create_info.pColorBlendState = &color_blend_state;
  create_info.pDynamicState = &dynamic_state;

  create_info.layout = pipeline_state.GetPipelineLayout()->GetHandle();
  create_info.renderPass = pipeline_state.GetRenderPass()->GetHandle();
  create_info.subpass = pipeline_state.GetSubpassIndex();

  VK_CHECK(vkCreateGraphicsPipelines(device_.GetHandle(), VK_NULL_HANDLE, 1,
                                     &create_info, nullptr, &handle_));

  for (auto shader_module : shader_modules) {
    vkDestroyShaderModule(device.GetHandle(), shader_module, nullptr);
  }

  state_ = pipeline_state;
}

GraphicsPipeline::GraphicsPipeline(GraphicsPipeline&& other)
    : device_{other.device_}, handle_{other.handle_}, state_{other.state_} {
  other.handle_ = VK_NULL_HANDLE;
}

GraphicsPipeline::~GraphicsPipeline() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyPipeline(device_.GetHandle(), handle_, nullptr);
  }
}
}  // namespace vkoo
