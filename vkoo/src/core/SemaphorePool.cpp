#include "vkoo/core/SemaphorePool.h"

#include "vkoo/core/Device.h"

namespace vkoo {
SemaphorePool::SemaphorePool(Device& device)
    : device_{device}, active_semaphore_count_{0} {}

SemaphorePool::~SemaphorePool() {
  Reset();

  for (VkSemaphore semaphore : semaphores_) {
    vkDestroySemaphore(device_.GetHandle(), semaphore, nullptr);
  }

  semaphores_.clear();
}

VkSemaphore SemaphorePool::RequestSemaphore() {
  if (active_semaphore_count_ < semaphores_.size()) {
    return semaphores_.at(active_semaphore_count_++);
  }
  VkSemaphore semaphore{VK_NULL_HANDLE};
  VkSemaphoreCreateInfo create_info{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
  VK_CHECK(vkCreateSemaphore(device_.GetHandle(), &create_info, nullptr,
                             &semaphore));
  semaphores_.push_back(semaphore);
  active_semaphore_count_++;

  return semaphore;
}

void SemaphorePool::Reset() { active_semaphore_count_ = 0; }

}  // namespace vkoo
