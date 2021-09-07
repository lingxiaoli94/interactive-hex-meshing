#pragma once

#include "Buffer.h"
#include "Device.h"
#include "vkoo/common.h"

namespace vkoo {
struct VertexAttribute {
  VkFormat format{VK_FORMAT_UNDEFINED};
  uint32_t stride{0};
  uint32_t offset{0};
};

class VertexObject {
 public:
  VertexObject(Device& device);
  ~VertexObject();
  template <class T>
  void Update(const std::string& name, const std::vector<T>& array) {
    size_t data_size = array.size() * sizeof(array[0]);
    core::Buffer buffer{device_, static_cast<VkDeviceSize>(data_size),
                        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
    buffer.Update(reinterpret_cast<const uint8_t*>(array.data()), data_size, 0);
    if (vertex_buffer_dict_.count(name)) {
      vertex_buffer_dict_.erase(name);
      attribute_resource_count_.erase(name);
      vertex_attributes_.erase(name);
    }
    vertex_buffer_dict_.emplace(name, std::move(buffer));
    attribute_resource_count_.emplace(name, array.size());

    auto itr = kGlmToVkFormat.find(typeid(T));
    if (itr != kGlmToVkFormat.end()) {
      vertex_attributes_.insert(
          std::make_pair(name, VertexAttribute{itr->second, sizeof(T), 0U}));
    } else {
      throw std::runtime_error("Unrecognized VK_FORMAT from GLM types!");
    }
  }

  void UpdateIndices(const std::vector<uint32_t>& indices);
  core::Buffer* FindResource(const std::string& name);
  int GetResourceCount(const std::string& name);
  const core::Buffer& GetIndexBuffer() const { return *index_buffer_; }
  const std::unordered_map<std::string, VertexAttribute>& GetVertexAttributes()
      const {
    return vertex_attributes_;
  }

  size_t GetIndexCount() const { return index_count_; }

 private:
  Device& device_;
  std::unordered_map<std::string, core::Buffer> vertex_buffer_dict_;
  std::unordered_map<std::string, size_t> attribute_resource_count_;
  std::unordered_map<std::string, VertexAttribute> vertex_attributes_;
  std::unique_ptr<core::Buffer> index_buffer_;
  size_t index_count_{0U};

  const std::unordered_map<std::type_index, VkFormat> kGlmToVkFormat = {
      {typeid(glm::vec2), VK_FORMAT_R32G32_SFLOAT},
      {typeid(glm::vec3), VK_FORMAT_R32G32B32_SFLOAT},
      {typeid(glm::vec4), VK_FORMAT_R32G32B32A32_SFLOAT},
  };
};
}  // namespace vkoo
