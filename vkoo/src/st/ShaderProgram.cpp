#include "vkoo/st/ShaderProgram.h"

namespace vkoo {
namespace st {
ShaderProgram::ShaderProgram(const ShaderSource& vertex_shader_source,
                             const ShaderSource& fragment_shader_source)
    : vertex_shader_source(vertex_shader_source),
      fragment_shader_source(fragment_shader_source) {}
}  // namespace st
}  // namespace vkoo
