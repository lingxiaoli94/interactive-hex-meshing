#pragma once

#include <spirv_glsl.hpp>

#include <string>
#include <unordered_map>

#include "ShaderModule.h"
#include "vkoo/common.h"

namespace vkoo {
class SPIRVReflection {
 public:
  static bool ReflectShaderResources(VkShaderStageFlagBits stage,
                                     const std::vector<uint32_t>& spirv,
                                     std::vector<ShaderResource>& resources);
};
}  // namespace vkoo
