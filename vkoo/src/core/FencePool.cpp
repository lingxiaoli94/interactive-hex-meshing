#include "vkoo/core/FencePool.h"

#include "vkoo/core/Device.h"

namespace vkoo {
FencePool::FencePool(Device& device)
    : device_{device}, active_fence_count_{0} {}

FencePool::~FencePool() {
  Wait();
  Reset();

  for (VkFence fence : fences_) {
    vkDestroyFence(device_.GetHandle(), fence, nullptr);
  }

  fences_.clear();
}

VkFence FencePool::RequestFence() {
  if (active_fence_count_ < fences_.size()) {
    return fences_.at(active_fence_count_++);
  }

  VkFence fence{VK_NULL_HANDLE};

  VkFenceCreateInfo create_info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};

  VK_CHECK(vkCreateFence(device_.GetHandle(), &create_info, nullptr, &fence));

  fences_.push_back(fence);

  active_fence_count_++;

  return fences_.back();
}

void FencePool::Wait(uint32_t timeout) const {
  if (active_fence_count_ < 1 || fences_.empty()) {
    return;
  }

  VK_CHECK(vkWaitForFences(device_.GetHandle(), active_fence_count_,
                           fences_.data(), true, timeout));
}

void FencePool::Reset() {
  if (active_fence_count_ < 1 || fences_.empty()) {
    return;
  }

  VK_CHECK(
      vkResetFences(device_.GetHandle(), active_fence_count_, fences_.data()));

  active_fence_count_ = 0;
}
}  // namespace vkoo