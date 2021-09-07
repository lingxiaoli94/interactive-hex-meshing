#include "vkoo/core/Device.h"

namespace vkoo {
Device::Device(const PhysicalDevice& gpu, VkSurfaceKHR surface,
               const std::vector<const char*>& required_extensions)
    : gpu_(gpu), resource_cache_(*this) {
  size_t queue_family_count = gpu.GetQueueFamilyProperties().size();
  std::vector<VkDeviceQueueCreateInfo> queue_create_infos;
  std::vector<std::vector<float>> queue_priorities;

  for (size_t i = 0; i < queue_family_count; i++) {
    const VkQueueFamilyProperties& family_properties =
        gpu.GetQueueFamilyProperties()[i];
    queue_priorities.push_back(
        std::vector<float>(family_properties.queueCount, 1.0f));

    VkDeviceQueueCreateInfo queue_create_info{};
    queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queue_create_info.queueFamilyIndex = i;
    queue_create_info.queueCount = family_properties.queueCount;
    queue_create_info.pQueuePriorities = queue_priorities.back().data();
    queue_create_infos.push_back(queue_create_info);
  }

  VkPhysicalDeviceFeatures device_features{};
  device_features.fillModeNonSolid = VK_TRUE;
  device_features.samplerAnisotropy = VK_TRUE;
  device_features.wideLines = VK_TRUE;

  VkDeviceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  create_info.pQueueCreateInfos = queue_create_infos.data();
  create_info.queueCreateInfoCount =
      static_cast<uint32_t>(queue_create_infos.size());
  create_info.pEnabledFeatures = &device_features;
  create_info.ppEnabledExtensionNames = required_extensions.data();
  create_info.enabledExtensionCount =
      static_cast<uint32_t>(required_extensions.size());

  VK_CHECK(vkCreateDevice(gpu_.GetHandle(), &create_info, nullptr, &handle_));

  queue_families_.resize(queue_family_count);
  for (size_t i = 0; i < queue_family_count; i++) {
    const VkQueueFamilyProperties& family_properties =
        gpu.GetQueueFamilyProperties()[i];
    VkBool32 present_supported = gpu_.IsPresentSupported(surface, i);
    for (size_t j = 0; j < family_properties.queueCount; j++) {
      queue_families_[i].emplace_back(*this, i, j, family_properties,
                                      present_supported);
    }
  }

  command_pool_ = std::make_unique<CommandPool>(
      *this, GetQueueByFlags(VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT, 0)
                 .GetFamilyIndex());
  fence_pool_ = std::make_unique<FencePool>(*this);
}

Device::~Device() {
  resource_cache_.Clear();
  command_pool_.reset();
  fence_pool_.reset();

  if (handle_ != VK_NULL_HANDLE) {
    vkDestroyDevice(handle_, nullptr);
  }
}

const Queue& Device::GetSuitableGraphicsQueue() const {
  for (const auto& queue_family : queue_families_) {
    if (!queue_family.empty() && queue_family[0].IsPresentSupported()) {
      return queue_family[0];
    }
  }
  throw std::runtime_error("No graphics queue with present supported found!");
}

const Queue& Device::GetQueueByFlags(VkQueueFlags required_queue_flags,
                                     uint32_t queue_index) {
  for (uint32_t queue_family_index = 0U;
       queue_family_index < queue_families_.size(); ++queue_family_index) {
    Queue& first_queue = queue_families_[queue_family_index][0];

    VkQueueFlags queue_flags = first_queue.GetProperties().queueFlags;
    uint32_t queue_count = first_queue.GetProperties().queueCount;

    if (((queue_flags & required_queue_flags) == required_queue_flags) &&
        queue_index < queue_count) {
      return queue_families_[queue_family_index][queue_index];
    }
  }

  throw std::runtime_error("Queue not found");
}

void Device::WaitIdle() { VK_CHECK(vkDeviceWaitIdle(handle_)); }

CommandBuffer& Device::RequestCommandBuffer() {
  return command_pool_->RequestCommandBuffer();
}

VkFence Device::RequestFence() { return fence_pool_->RequestFence(); }
}  // namespace vkoo
