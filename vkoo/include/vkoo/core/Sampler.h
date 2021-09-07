#pragma once
#include "vkoo/common.h"

namespace vkoo {
class Device;

namespace core {
class Sampler {
 public:
  Sampler(Device& device, const VkSamplerCreateInfo& info);
  Sampler(Sampler&& sampler);
  ~Sampler();
  VkSampler GetHandle() const { return handle_; }

 private:
  Device& device_;
  VkSampler handle_{VK_NULL_HANDLE};
};
}  // namespace core
}  // namespace vkoo
