#include "vkoo/core/Sampler.h"

#include "vkoo/core/Device.h"

namespace vkoo {
namespace core {
Sampler::Sampler(Device& device, const VkSamplerCreateInfo& info)
    : device_{device} {
  VK_CHECK(vkCreateSampler(device_.GetHandle(), &info, nullptr, &handle_));
}

Sampler::Sampler(Sampler&& other)
    : device_{other.device_}, handle_{other.handle_} {
  other.handle_ = VK_NULL_HANDLE;
}

Sampler::~Sampler() {
  if (handle_ != VK_NULL_HANDLE) {
    vkDestroySampler(device_.GetHandle(), handle_, nullptr);
  }
}
}  // namespace core

}  // namespace vkoo
