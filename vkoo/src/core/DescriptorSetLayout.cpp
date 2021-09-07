#include "vkoo/core/DescriptorSetLayout.h"

#include "vkoo/core/Device.h"
#include "vkoo/core/ShaderModule.h"

namespace vkoo {
inline VkDescriptorType FindDescriptorType(ShaderResourceType resource_type,
                                           bool dynamic) {
  switch (resource_type) {
    case ShaderResourceType::InputAttachment:
      return VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
      break;
    case ShaderResourceType::Image:
      return VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
      break;
    case ShaderResourceType::ImageSampler:
      return VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
      break;
    case ShaderResourceType::ImageStorage:
      return VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
      break;
    case ShaderResourceType::Sampler:
      return VK_DESCRIPTOR_TYPE_SAMPLER;
      break;
    case ShaderResourceType::BufferUniform:
      if (dynamic) {
        return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
      } else {
        return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
      }
      break;
    case ShaderResourceType::BufferStorage:
      if (dynamic) {
        return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC;
      } else {
        return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      }
      break;
    default:
      throw std::runtime_error(
          "No conversion possible for the shader resource type.");
      break;
  }
}

DescriptorSetLayout::DescriptorSetLayout(
    Device& device, const std::vector<ShaderModule*>& shader_modules,
    uint32_t set_index, const std::vector<ShaderResource>& resource_set)
    : device_(device), set_index_(set_index), shader_modules_{shader_modules} {
  for (auto& resource : resource_set) {
    // Skip shader resources whitout a binding point
    if (resource.type == ShaderResourceType::Input ||
        resource.type == ShaderResourceType::Output ||
        resource.type == ShaderResourceType::PushConstant ||
        resource.type == ShaderResourceType::SpecializationConstant) {
      continue;
    }

    if (resource.mode != ShaderResourceMode::Static) {
      throw std::runtime_error("Non-static resources are not supported yet.");
    }

    // Convert from ShaderResourceType to VkDescriptorType.
    auto descriptor_type = FindDescriptorType(
        resource.type, resource.mode == ShaderResourceMode::Dynamic);

    binding_flags_.push_back(0);

    // Convert ShaderResource to VkDescriptorSetLayoutBinding.
    VkDescriptorSetLayoutBinding layout_binding{};

    layout_binding.binding = resource.binding;
    layout_binding.descriptorCount = resource.array_size;
    layout_binding.descriptorType = descriptor_type;
    layout_binding.stageFlags =
        static_cast<VkShaderStageFlags>(resource.stages);

    bindings_.push_back(layout_binding);
    binding_id_to_layout_dict_.emplace(resource.binding, layout_binding);
    name_to_binding_id_dict_.emplace(resource.name, resource.binding);
  }
  VkDescriptorSetLayoutCreateInfo create_info{
      VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
  create_info.flags = 0;
  create_info.bindingCount = static_cast<uint32_t>(bindings_.size());
  create_info.pBindings = bindings_.data();

  VK_CHECK(vkCreateDescriptorSetLayout(device_.GetHandle(), &create_info,
                                       nullptr, &handle_));
}

std::optional<VkDescriptorSetLayoutBinding>
DescriptorSetLayout::GetLayoutBinding(uint32_t binding_index) const {
  auto it = binding_id_to_layout_dict_.find(binding_index);
  if (it == binding_id_to_layout_dict_.end()) {
    return {};
  }
  return it->second;
}

std::optional<VkDescriptorSetLayoutBinding>
DescriptorSetLayout::GetLayoutBinding(const std::string& name) const {
  auto it = name_to_binding_id_dict_.find(name);
  if (it == name_to_binding_id_dict_.end()) {
    return {};
  }
  return GetLayoutBinding(it->second);
}

DescriptorSetLayout::DescriptorSetLayout(DescriptorSetLayout&& other)
    : device_{other.device_},
      handle_{other.handle_},
      set_index_{other.set_index_},
      shader_modules_{other.shader_modules_},
      bindings_{other.bindings_},
      binding_flags_{other.binding_flags_},
      binding_id_to_layout_dict_{other.binding_id_to_layout_dict_},
      name_to_binding_id_dict_{other.name_to_binding_id_dict_} {
  other.handle_ = VK_NULL_HANDLE;
}

DescriptorSetLayout::~DescriptorSetLayout() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyDescriptorSetLayout(device_.GetHandle(), handle_, nullptr);
  }
}
}  // namespace vkoo
