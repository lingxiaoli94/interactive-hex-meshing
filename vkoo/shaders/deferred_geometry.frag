#version 450

layout(location = 0) in vec4 in_pos;
layout(location = 1) in vec3 in_normal;
#ifdef PER_VERTEX_COLOR
layout(location = 2) in vec4 in_color;
#else
layout(location = 2) in vec2 in_uv;
#endif

#ifdef HAS_DIFFUSE_TEXTURE
layout(binding = 2) uniform sampler2D diffuse_texture;
#endif

layout(push_constant) uniform MaterialUniform {
    vec4 diffuse_color;
    uint has_texture;
} material_uniform;

layout(location = 0) out vec4 o_albedo;
layout(location = 1) out vec4 o_normal;

void main(void)
{
    o_normal = vec4(normalize(in_normal), 1.0);

#ifdef PER_VERTEX_COLOR
    o_albedo = in_color;
#else
#ifdef HAS_DIFFUSE_TEXTURE
    o_albedo = texture(diffuse_texture, in_uv);
#else
    o_albedo = material_uniform.diffuse_color;
#endif
#endif
}
