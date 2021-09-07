#include "vkoo/core/VertexObject.h"

namespace vkoo {
VertexObject::VertexObject(Device& device) : device_(device) {}

void VertexObject::UpdateIndices(const std::vector<uint32_t>& indices) {
  size_t data_size = indices.size() * sizeof(indices[0]);
  index_buffer_ = std::make_unique<core::Buffer>(
      device_, static_cast<VkDeviceSize>(data_size),
      VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  index_buffer_->Update(reinterpret_cast<const uint8_t*>(indices.data()),
                        data_size, 0);
  index_count_ = indices.size();
}

core::Buffer* VertexObject::FindResource(const std::string& name) {
  auto it = vertex_buffer_dict_.find(name);
  if (it == vertex_buffer_dict_.end()) {
    return nullptr;
  } else {
    return &it->second;
  }
}

int VertexObject::GetResourceCount(const std::string& name) {
  auto it = attribute_resource_count_.find(name);
  if (it == attribute_resource_count_.end()) {
    return -1;
  }
  return static_cast<int>(it->second);
}

VertexObject::~VertexObject() {
  // Need to wait all queue operations to finish before destroying the buffers.
  device_.WaitIdle();
}
}  // namespace vkoo
