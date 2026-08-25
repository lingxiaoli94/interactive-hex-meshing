#pragma once

#include "vkoo/common.h"

namespace vkoo {
class Device;

std::string GetShaderPath();
std::string GetAssetPath();

std::vector<uint8_t> ReadBinaryFile(const std::string& filename);
#if HEX_ENABLE_VULKAN
// Called only by core/Buffer.cpp and core/Image.cpp, neither of which exists in
// the OFF build. The rest of this header is Vulkan-free and is needed there.
uint32_t FindMemoryType(const Device& device, uint32_t type_filter,
                        VkMemoryPropertyFlags properties);
#endif

std::unique_ptr<std::vector<glm::vec3>> EstimateNormals(
    const std::vector<glm::vec3>& positions,
    const std::vector<uint32_t>& indices, bool inverse_normal = false);

std::vector<std::string> Split(const std::string& s, char delim);
}  // namespace vkoo