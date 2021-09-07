#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;

std::string GetShaderPath();
std::string GetAssetPath();

std::vector<uint8_t> ReadBinaryFile(const std::string& filename);
uint32_t FindMemoryType(const Device& device, uint32_t type_filter,
                        VkMemoryPropertyFlags properties);

std::unique_ptr<std::vector<glm::vec3>> EstimateNormals(
    const std::vector<glm::vec3>& positions,
    const std::vector<uint32_t>& indices, bool inverse_normal = false);

std::vector<std::string> Split(const std::string& s, char delim);
}  // namespace vkoo