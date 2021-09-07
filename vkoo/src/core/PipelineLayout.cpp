#include "vkoo/core/PipelineLayout.h"

#include "vkoo/core/Device.h"

#include "vkoo/logging.h"

namespace vkoo {
PipelineLayout::PipelineLayout(Device& device,
                               const std::vector<ShaderModule*>& shader_modules)
    : device_(device), shader_modules_(shader_modules) {
  for (const auto& shader_module : shader_modules) {
    for (const auto& resource : shader_module->GetResources()) {
      std::string key = resource.name;
      if (resource.type == ShaderResourceType::Input ||
          resource.type == ShaderResourceType::Output) {
        key = std::to_string(resource.stages) + "_" + key;
      }

      auto it = name_to_resource_dict_.find(key);
      if (it != name_to_resource_dict_.end()) {
        it->second.stages |= resource.stages;
      } else {
        name_to_resource_dict_.emplace(key, resource);
      }
    }
  }

  for (auto& kv : name_to_resource_dict_) {
    auto& resource = kv.second;
    auto it = shader_sets_.find(resource.set);
    if (it != shader_sets_.end()) {
      it->second.push_back(resource);
    } else {
      shader_sets_.emplace(resource.set, std::vector<ShaderResource>{resource});
    }
  }

  // Create descriptor set layouts here.
  for (auto& kv : shader_sets_) {
    descriptor_set_layouts_.emplace_back(
        &device_.GetResourceCache().RequestDescriptorSetLayout(
            shader_modules, kv.first, kv.second));
  }

  std::vector<VkDescriptorSetLayout> descriptor_set_layout_handles;
  for (size_t i = 0; i < descriptor_set_layouts_.size(); i++) {
    if (descriptor_set_layouts_[i]) {
      descriptor_set_layout_handles.push_back(
          descriptor_set_layouts_[i]->GetHandle());
    } else {
      descriptor_set_layout_handles.push_back(VK_NULL_HANDLE);
    }
  }
  // Collect all the push constant shader resources.
  std::vector<VkPushConstantRange> push_constant_ranges;
  for (auto& push_constant_resource :
       GetResources(ShaderResourceType::PushConstant)) {
    push_constant_ranges.push_back({push_constant_resource.stages,
                                    push_constant_resource.offset,
                                    push_constant_resource.size});
  }

  VkPipelineLayoutCreateInfo create_info{
      VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};

  create_info.setLayoutCount =
      static_cast<uint32_t>(descriptor_set_layout_handles.size());
  create_info.pSetLayouts = descriptor_set_layout_handles.data();
  create_info.pushConstantRangeCount =
      static_cast<uint32_t>(push_constant_ranges.size());
  create_info.pPushConstantRanges = push_constant_ranges.data();

  VK_CHECK(vkCreatePipelineLayout(device.GetHandle(), &create_info, nullptr,
                                  &handle_));
}

PipelineLayout::PipelineLayout(PipelineLayout&& other)
    : device_{other.device_},
      handle_{other.handle_},
      shader_modules_{other.shader_modules_},
      name_to_resource_dict_{other.name_to_resource_dict_},
      shader_sets_{other.shader_sets_},
      descriptor_set_layouts_{other.descriptor_set_layouts_} {
  other.handle_ = VK_NULL_HANDLE;
}

PipelineLayout::~PipelineLayout() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyPipelineLayout(device_.GetHandle(), handle_, nullptr);
  }
}

std::vector<ShaderResource> PipelineLayout::GetResources(
    ShaderResourceType type, VkShaderStageFlagBits stage) const {
  std::vector<ShaderResource> found_resources;
  for (auto& kv : name_to_resource_dict_) {
    auto& resource = kv.second;
    if (resource.type == type || type == ShaderResourceType::All) {
      if (resource.stages == stage || stage == VK_SHADER_STAGE_ALL) {
        found_resources.push_back(resource);
      }
    }
  }
  return found_resources;
}

const std::vector<ShaderModule*>& PipelineLayout::GetShaderModules() const {
  return shader_modules_;
}

DescriptorSetLayout& PipelineLayout::GetDescriptorSetLayout(
    uint32_t set_index) const {
  for (auto& descriptor_set_layout : descriptor_set_layouts_) {
    if (descriptor_set_layout->GetSetIndex() == set_index) {
      return *descriptor_set_layout;
    }
  }
  throw std::runtime_error(fmt::format(
      "Couldn't find descriptor set layout at index {}", set_index));
}

bool PipelineLayout::HasDescriptorSetLayout(uint32_t set_index) const {
  for (auto& descriptor_set_layout : descriptor_set_layouts_) {
    if (descriptor_set_layout->GetSetIndex() == set_index) {
      return true;
    }
  }
  return false;
}
VkShaderStageFlags PipelineLayout::GetPushConstantRangeStage(
    uint32_t size, uint32_t offset) const {
  VkShaderStageFlags stages = 0;

  for (auto& push_constant_resource :
       GetResources(ShaderResourceType::PushConstant)) {
    if (offset >= push_constant_resource.offset &&
        offset + size <=
            push_constant_resource.offset + push_constant_resource.size) {
      stages |= push_constant_resource.stages;
    }
  }
  return stages;
}
}  // namespace vkoo
