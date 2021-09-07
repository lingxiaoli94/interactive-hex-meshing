#version 450

layout(location = 0) in vec4 in_pos;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;

#ifdef HAS_DIFFUSE_TEXTURE
layout(binding = 2) uniform sampler2D diffuse_texture;
#endif

layout(push_constant) uniform MaterialUniform {
    vec4 diffuse_color;
    uint has_texture;
} material_uniform;

layout(location = 0) out vec4 o_color;

void main(void)
{
    #ifdef HAS_DIFFUSE_TEXTURE
    o_color = texture(diffuse_texture, in_uv);
    #else
    o_color = material_uniform.diffuse_color;
    #endif
}
