#include "vkoo/utils.h"

#include <vkoo/core/Device.h>

namespace vkoo {
std::string GetShaderPath() { return SHADER_PATH; }
std::string GetAssetPath() { return ASSET_PATH; }

std::vector<uint8_t> ReadBinaryFile(const std::string& filename) {
  std::ifstream file(filename, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file " + filename + "!");
  }

  size_t fileSize = (size_t)file.tellg();
  std::vector<uint8_t> buffer(fileSize);

  file.seekg(0);
  file.read(reinterpret_cast<char*>(buffer.data()), fileSize);

  file.close();

  return buffer;
}

uint32_t FindMemoryType(const Device& device, uint32_t type_filter,
                        VkMemoryPropertyFlags properties) {
  VkPhysicalDeviceMemoryProperties mem_properties;
  vkGetPhysicalDeviceMemoryProperties(device.GetGPU().GetHandle(),
                                      &mem_properties);

  for (uint32_t i = 0; i < mem_properties.memoryTypeCount; i++) {
    if ((type_filter & (1 << i)) &&
        (mem_properties.memoryTypes[i].propertyFlags & properties) ==
            properties) {
      return i;
    }
  }

  throw std::runtime_error("Failed to find a suitable memory type!");
}

std::unique_ptr<std::vector<glm::vec3>> EstimateNormals(
    const std::vector<glm::vec3>& positions,
    const std::vector<uint32_t>& indices, bool inverse_normal) {
  auto normals = std::make_unique<std::vector<glm::vec3>>(positions.size(),
                                                          glm::vec3(0.0f));

  for (size_t i = 0; i < indices.size(); i += 3) {
    int v1 = indices[i];
    int v2 = indices[i + 1];
    int v3 = indices[i + 2];
    auto& p1 = positions[v1];
    auto& p2 = positions[v2];
    auto& p3 = positions[v3];
    auto n = glm::cross(p2 - p1, p3 - p1);
    // No need to normalize here, since the norm of n is
    // proportional to the area.
    (*normals)[v1] += n;
    (*normals)[v2] += n;
    (*normals)[v3] += n;
  }

  for (size_t i = 0; i < normals->size(); i++) {
    (*normals)[i] = glm::normalize((*normals)[i]);
    if (inverse_normal) {
      (*normals)[i] = -(*normals)[i];
    }
  }

  return normals;
}

std::vector<std::string> Split(const std::string& s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> result;
  while (std::getline(ss, item, delim)) {
    result.emplace_back(std::move(item));
  }
  return result;
}

}  // namespace vkoo
