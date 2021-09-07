#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;

class SemaphorePool {
 public:
  SemaphorePool(Device& device);

  ~SemaphorePool();

  VkSemaphore RequestSemaphore();

  void Reset();

 private:
  Device& device_;

  std::vector<VkSemaphore> semaphores_;

  uint32_t active_semaphore_count_;
};
}  // namespace vkoo
