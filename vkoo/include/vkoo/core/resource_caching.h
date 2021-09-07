#pragma once

#include <type_traits>

#include "DescriptorPool.h"
#include "DescriptorSetLayout.h"
#include "Device.h"
#include "ShaderModule.h"
#include "vkoo/common.h"

namespace std {
template <>
struct hash<vkoo::ShaderModule> {
  std::size_t operator()(const vkoo::ShaderModule& shader_module) const {
    std::size_t result = 0;
    vkoo::hash_combine(result, shader_module.GetHashID());
    return result;
  }
};

template <>
struct hash<vkoo::ShaderSource> {
  std::size_t operator()(const vkoo::ShaderSource& shader_source) const {
    std::size_t result = 0;
    vkoo::hash_combine(result, shader_source.glsl_file);
    vkoo::hash_combine(result, shader_source.entry_point);
    return result;
  }
};

template <>
struct hash<vkoo::ShaderVariant> {
  std::size_t operator()(const vkoo::ShaderVariant& shader_variant) const {
    std::size_t result = 0;
    vkoo::hash_combine(result, shader_variant.GetHashId());
    return result;
  }
};

template <>
struct hash<vkoo::ShaderResource> {
  std::size_t operator()(const vkoo::ShaderResource& shader_resource) const {
    std::size_t result = 0;

    if (shader_resource.type == vkoo::ShaderResourceType::Input ||
        shader_resource.type == vkoo::ShaderResourceType::Output ||
        shader_resource.type == vkoo::ShaderResourceType::PushConstant ||
        shader_resource.type ==
            vkoo::ShaderResourceType::SpecializationConstant) {
      return result;
    }

    vkoo::hash_combine(result, shader_resource.set);
    vkoo::hash_combine(result, shader_resource.binding);
    vkoo::hash_combine(
        result,
        static_cast<std::underlying_type<vkoo::ShaderResourceType>::type>(
            shader_resource.type));
    vkoo::hash_combine(result, shader_resource.mode);

    return result;
  }
};

template <>
struct hash<VkDescriptorBufferInfo> {
  std::size_t operator()(
      const VkDescriptorBufferInfo& descriptor_buffer_info) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, descriptor_buffer_info.buffer);
    vkoo::hash_combine(result, descriptor_buffer_info.range);
    vkoo::hash_combine(result, descriptor_buffer_info.offset);

    return result;
  }
};

template <>
struct hash<VkDescriptorImageInfo> {
  std::size_t operator()(
      const VkDescriptorImageInfo& descriptor_image_info) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, descriptor_image_info.imageView);
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkImageLayout>::type>(
                           descriptor_image_info.imageLayout));
    vkoo::hash_combine(result, descriptor_image_info.sampler);

    return result;
  }
};

template <>
struct hash<VkWriteDescriptorSet> {
  std::size_t operator()(
      const VkWriteDescriptorSet& write_descriptor_set) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, write_descriptor_set.dstSet);
    vkoo::hash_combine(result, write_descriptor_set.dstBinding);
    vkoo::hash_combine(result, write_descriptor_set.dstArrayElement);
    vkoo::hash_combine(result, write_descriptor_set.descriptorCount);
    vkoo::hash_combine(result, write_descriptor_set.descriptorType);

    switch (write_descriptor_set.descriptorType) {
      case VK_DESCRIPTOR_TYPE_SAMPLER:
      case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
      case VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE:
      case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
      case VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT:
        for (uint32_t i = 0; i < write_descriptor_set.descriptorCount; i++) {
          vkoo::hash_combine(result, write_descriptor_set.pImageInfo[i]);
        }
        break;

      case VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER:
      case VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER:
        for (uint32_t i = 0; i < write_descriptor_set.descriptorCount; i++) {
          vkoo::hash_combine(result, write_descriptor_set.pTexelBufferView[i]);
        }
        break;

      case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
      case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
      case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC:
      case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC:
        for (uint32_t i = 0; i < write_descriptor_set.descriptorCount; i++) {
          vkoo::hash_combine(result, write_descriptor_set.pBufferInfo[i]);
        }
        break;

      default:
        // Not implemented
        break;
    };

    return result;
  }
};

template <>
struct hash<vkoo::DescriptorSetLayout> {
  std::size_t operator()(
      const vkoo::DescriptorSetLayout& descriptor_set_layout) const {
    std::size_t result = 0;
    vkoo::hash_combine(result, descriptor_set_layout.GetHandle());
    return result;
  }
};

template <>
struct hash<vkoo::DescriptorPool> {
  std::size_t operator()(const vkoo::DescriptorPool& descriptor_pool) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, descriptor_pool.GetDescriptorSetLayout());

    return result;
  }
};

template <>
struct hash<VkAttachmentDescription2KHR> {
  std::size_t operator()(const VkAttachmentDescription2KHR& attachment) const {
    std::size_t result = 0;

    vkoo::hash_combine(
        result,
        static_cast<std::underlying_type<VkFormat>::type>(attachment.format));
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkSampleCountFlagBits>::type>(
                    attachment.samples));
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkAttachmentLoadOp>::type>(
                    attachment.loadOp));
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkAttachmentStoreOp>::type>(
                    attachment.storeOp));
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkAttachmentLoadOp>::type>(
                    attachment.stencilLoadOp));
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkAttachmentStoreOp>::type>(
                    attachment.stencilStoreOp));
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkImageLayout>::type>(
                           attachment.initialLayout));
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkImageLayout>::type>(
                           attachment.finalLayout));

    return result;
  }
};

template <>
struct hash<VkExtent2D> {
  size_t operator()(const VkExtent2D& extent) const {
    size_t result = 0;

    vkoo::hash_combine(result, extent.width);
    vkoo::hash_combine(result, extent.height);

    return result;
  }
};

template <>
struct hash<vkoo::StencilOpState> {
  std::size_t operator()(const vkoo::StencilOpState& stencil) const {
    std::size_t result = 0;

    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkCompareOp>::type>(
                           stencil.compare_op));
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkStencilOp>::type>(
                           stencil.depth_fail_op));
    vkoo::hash_combine(
        result,
        static_cast<std::underlying_type<VkStencilOp>::type>(stencil.fail_op));
    vkoo::hash_combine(
        result,
        static_cast<std::underlying_type<VkStencilOp>::type>(stencil.pass_op));

    return result;
  }
};

template <>
struct hash<vkoo::LoadStoreInfo> {
  std::size_t operator()(const vkoo::LoadStoreInfo& load_store_info) const {
    std::size_t result = 0;

    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkAttachmentLoadOp>::type>(
                    load_store_info.load_op));
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkAttachmentStoreOp>::type>(
                    load_store_info.store_op));

    return result;
  }
};

template <>
struct hash<vkoo::SubpassInfo> {
  std::size_t operator()(const vkoo::SubpassInfo& subpass_info) const {
    std::size_t result = 0;

    for (uint32_t output_attachment : subpass_info.output_attachments) {
      vkoo::hash_combine(result, output_attachment);
    }

    for (uint32_t input_attachment : subpass_info.input_attachments) {
      vkoo::hash_combine(result, input_attachment);
    }

    for (uint32_t color_resolve_attachments :
         subpass_info.color_resolve_attachments) {
      vkoo::hash_combine(result, color_resolve_attachments);
    }

    vkoo::hash_combine(result, subpass_info.disable_depth_stencil_attachment);
    vkoo::hash_combine(result, subpass_info.depth_stencil_resolve_attachment);
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkResolveModeFlagBits>::type>(
                    subpass_info.depth_stencil_resolve_mode));

    return result;
  }
};

template <>
struct hash<VkSubpassDependency2KHR> {
  std::size_t operator()(const VkSubpassDependency2KHR& dependency) const {
    std::size_t result = 0;
    vkoo::hash_combine(result, dependency.srcSubpass);
    vkoo::hash_combine(result, dependency.dstSubpass);
    vkoo::hash_combine(result, dependency.srcStageMask);
    vkoo::hash_combine(result, dependency.dstStageMask);
    vkoo::hash_combine(result, dependency.srcAccessMask);
    vkoo::hash_combine(result, dependency.dstAccessMask);
    vkoo::hash_combine(result, dependency.dependencyFlags);
    return result;
  };
};

template <>
struct hash<vkoo::RenderTarget> {
  std::size_t operator()(const vkoo::RenderTarget& render_target) const {
    std::size_t result = 0;

    for (auto& view : render_target.GetViews()) {
      vkoo::hash_combine(result, view.GetHandle());
    }

    return result;
  }
};

template <>
struct hash<vkoo::RenderPass> {
  std::size_t operator()(const vkoo::RenderPass& render_pass) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, render_pass.GetHandle());

    return result;
  }
};

template <>
struct hash<VkVertexInputAttributeDescription> {
  std::size_t operator()(
      const VkVertexInputAttributeDescription& vertex_attrib) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, vertex_attrib.binding);
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkFormat>::type>(
                           vertex_attrib.format));
    vkoo::hash_combine(result, vertex_attrib.location);
    vkoo::hash_combine(result, vertex_attrib.offset);

    return result;
  }
};

template <>
struct hash<VkVertexInputBindingDescription> {
  std::size_t operator()(
      const VkVertexInputBindingDescription& vertex_binding) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, vertex_binding.binding);
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkVertexInputRate>::type>(
                    vertex_binding.inputRate));
    vkoo::hash_combine(result, vertex_binding.stride);

    return result;
  }
};

template <>
struct hash<vkoo::ColorBlendAttachmentState> {
  std::size_t operator()(
      const vkoo::ColorBlendAttachmentState& color_blend_attachment) const {
    std::size_t result = 0;

    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkBlendOp>::type>(
                           color_blend_attachment.alpha_blend_op));
    vkoo::hash_combine(result, color_blend_attachment.blend_enable);
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkBlendOp>::type>(
                           color_blend_attachment.color_blend_op));
    vkoo::hash_combine(result, color_blend_attachment.color_write_mask);
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkBlendFactor>::type>(
                           color_blend_attachment.dst_alpha_blend_factor));
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkBlendFactor>::type>(
                           color_blend_attachment.dst_color_blend_factor));
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkBlendFactor>::type>(
                           color_blend_attachment.src_alpha_blend_factor));
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkBlendFactor>::type>(
                           color_blend_attachment.src_color_blend_factor));

    return result;
  }
};

template <>
struct hash<vkoo::PipelineState> {
  std::size_t operator()(const vkoo::PipelineState& pipeline_state) const {
    std::size_t result = 0;

    vkoo::hash_combine(result, pipeline_state.GetPipelineLayout()->GetHandle());

    if (auto render_pass = pipeline_state.GetRenderPass()) {
      vkoo::hash_combine(result, render_pass->GetHandle());
    }

    vkoo::hash_combine(result, pipeline_state.GetSubpassIndex());

    for (auto shader_module :
         pipeline_state.GetPipelineLayout()->GetShaderModules()) {
      vkoo::hash_combine(result, shader_module->GetHashID());
    }

    // VertexInputState
    for (auto& attribute : pipeline_state.GetVertexInputState().attributes) {
      vkoo::hash_combine(result, attribute);
    }

    for (auto& binding : pipeline_state.GetVertexInputState().bindings) {
      vkoo::hash_combine(result, binding);
    }

    // InputAssemblyState
    vkoo::hash_combine(
        result,
        pipeline_state.GetInputAssemblyState().primitive_restart_enable);
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkPrimitiveTopology>::type>(
                    pipeline_state.GetInputAssemblyState().topology));

    // ViewportState
    vkoo::hash_combine(result,
                       pipeline_state.GetViewportState().viewport_count);
    vkoo::hash_combine(result, pipeline_state.GetViewportState().scissor_count);

    // RasterizationState
    vkoo::hash_combine(result,
                       pipeline_state.GetRasterizationState().cull_mode);
    vkoo::hash_combine(
        result, pipeline_state.GetRasterizationState().depth_bias_enable);
    vkoo::hash_combine(
        result, pipeline_state.GetRasterizationState().depth_clamp_enable);
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkFrontFace>::type>(
                           pipeline_state.GetRasterizationState().front_face));
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkPolygonMode>::type>(
                    pipeline_state.GetRasterizationState().polygon_mode));
    vkoo::hash_combine(
        result,
        pipeline_state.GetRasterizationState().rasterizer_discard_enable);
    vkoo::hash_combine(result,
                       pipeline_state.GetRasterizationState().line_width);

    // MultisampleState
    vkoo::hash_combine(result,
                       pipeline_state.GetMultisampleState().min_sample_shading);
    vkoo::hash_combine(
        result,
        static_cast<std::underlying_type<VkSampleCountFlagBits>::type>(
            pipeline_state.GetMultisampleState().rasterization_samples));
    vkoo::hash_combine(
        result, pipeline_state.GetMultisampleState().sample_shading_enable);

    // DepthStencilState
    vkoo::hash_combine(result, pipeline_state.GetDepthStencilState().back);
    vkoo::hash_combine(
        result, pipeline_state.GetDepthStencilState().depth_bounds_test_enable);
    vkoo::hash_combine(
        result, static_cast<std::underlying_type<VkCompareOp>::type>(
                    pipeline_state.GetDepthStencilState().depth_compare_op));
    vkoo::hash_combine(result,
                       pipeline_state.GetDepthStencilState().depth_test_enable);
    vkoo::hash_combine(
        result, pipeline_state.GetDepthStencilState().depth_write_enable);
    vkoo::hash_combine(result, pipeline_state.GetDepthStencilState().front);
    vkoo::hash_combine(
        result, pipeline_state.GetDepthStencilState().stencil_test_enable);

    // ColorBlendState
    vkoo::hash_combine(result,
                       static_cast<std::underlying_type<VkLogicOp>::type>(
                           pipeline_state.GetColorBlendState().logic_op));
    vkoo::hash_combine(result,
                       pipeline_state.GetColorBlendState().logic_op_enable);

    for (auto& attachment : pipeline_state.GetColorBlendState().attachments) {
      vkoo::hash_combine(result, attachment);
    }

    return result;
  }
};
}  // namespace std

namespace vkoo {
template <typename T>
inline void hash_param(size_t& seed, const T& value) {
  hash_combine(seed, value);
}

template <typename T, typename... Args>
inline void hash_param(size_t& seed, const T& first_arg, const Args&... args) {
  hash_param(seed, first_arg);
  hash_param(seed, args...);
}

template <>
inline void hash_param<std::vector<ShaderModule*>>(
    size_t& seed, const std::vector<ShaderModule*>& value) {
  for (auto& shader_module : value) {
    hash_combine(seed, shader_module->GetHashID());
  }
}

template <>
inline void hash_param<std::vector<ShaderResource>>(
    size_t& seed, const std::vector<ShaderResource>& value) {
  for (auto& resource : value) {
    hash_combine(seed, resource);
  }
}

template <>
inline void hash_param<std::vector<VkImageView>>(
    size_t& seed, const std::vector<VkImageView>& value) {
  for (auto& view : value) {
    hash_combine(seed, view);
  }
}

template <>
inline void hash_param<std::vector<LoadStoreInfo>>(
    size_t& seed, const std::vector<LoadStoreInfo>& value) {
  for (auto& load_store_info : value) {
    hash_combine(seed, load_store_info);
  }
}

template <>
inline void hash_param<std::vector<VkAttachmentDescription2KHR>>(
    size_t& seed, const std::vector<VkAttachmentDescription2KHR>& value) {
  for (auto& description : value) {
    hash_combine(seed, description);
  }
}

template <>
inline void hash_param<std::vector<SubpassInfo>>(
    size_t& seed, const std::vector<SubpassInfo>& value) {
  for (auto& subpass_info : value) {
    hash_combine(seed, subpass_info);
  }
}

template <>
inline void hash_param<std::vector<VkSubpassDependency2KHR>>(
    size_t& seed, const std::vector<VkSubpassDependency2KHR>& value) {
  for (auto& subpass_info : value) {
    hash_combine(seed, subpass_info);
  }
}

template <>
inline void
hash_param<std::map<uint32_t, std::map<uint32_t, VkDescriptorBufferInfo>>>(
    size_t& seed,
    const std::map<uint32_t, std::map<uint32_t, VkDescriptorBufferInfo>>&
        value) {
  for (auto& binding_set : value) {
    hash_combine(seed, binding_set.first);

    for (auto& binding_element : binding_set.second) {
      hash_combine(seed, binding_element.first);
      hash_combine(seed, binding_element.second);
    }
  }
}

template <>
inline void
hash_param<std::map<uint32_t, std::map<uint32_t, VkDescriptorImageInfo>>>(
    size_t& seed,
    const std::map<uint32_t, std::map<uint32_t, VkDescriptorImageInfo>>&
        value) {
  for (auto& binding_set : value) {
    hash_combine(seed, binding_set.first);

    for (auto& binding_element : binding_set.second) {
      hash_combine(seed, binding_element.first);
      hash_combine(seed, binding_element.second);
    }
  }
}

template <class T, class... A>
T& RequestResource(Device& device,
                   std::unordered_map<std::size_t, T>& resource_pool,
                   A&... args) {
  std::size_t hash{0U};
  hash_param(hash, args...);

  auto res_it = resource_pool.find(hash);

  if (res_it != resource_pool.end()) {
    return res_it->second;
  }

  const char* res_type = typeid(T).name();
  size_t res_id = resource_pool.size();

  T resource(device, args...);

  auto res_ins_it = resource_pool.emplace(hash, std::move(resource));

  if (!res_ins_it.second) {
    throw std::runtime_error{std::string{"Insertion error for #"} +
                             std::to_string(res_id) + "cache object (" +
                             res_type + ")"};
  }

  res_it = res_ins_it.first;

  return res_it->second;
}
}  // namespace vkoo
