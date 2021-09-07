#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;

class FencePool {
 public:
  FencePool(Device& device);

  ~FencePool();

  VkFence RequestFence();

  void Wait(uint32_t timeout = std::numeric_limits<uint32_t>::max()) const;

  void Reset();

 private:
  Device& device_;

  std::vector<VkFence> fences_;

  uint32_t active_fence_count_;
};
}  // namespace vkoo
