#pragma once

#include <glslang/Public/ShaderLang.h>

#include "ShaderModule.h"
#include "vkoo/common.h"

namespace vkoo {
class GLSLCompiler {
 public:
  static bool CompileToSPIRV(VkShaderStageFlagBits stage,
                             const std::vector<uint8_t>& glsl_source,
                             const std::string& entry_point,
                             const ShaderVariant& shader_variant,
                             std::vector<std::uint32_t>& spirv,
                             std::string& info_log);
};
}  // namespace vkoo
