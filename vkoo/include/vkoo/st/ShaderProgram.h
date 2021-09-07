#pragma once

#include "vkoo/common.h"
#include "vkoo/core/ShaderModule.h"

namespace vkoo {
namespace st {
class ShaderProgram {
 public:
  ShaderProgram(const ShaderSource& vertex_shader_source,
                const ShaderSource& fragment_shader_source);
  ShaderSource vertex_shader_source;
  ShaderSource fragment_shader_source;
};
}  // namespace st
}  // namespace vkoo
