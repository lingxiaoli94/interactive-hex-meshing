#version 450

layout(set = 0, binding = 1) uniform GlobalUniform {
    mat4 model_mat;
    mat4 normal_mat;
    mat4 view_proj_mat;
    vec3 camera_position;
} global_uniform;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
#ifdef PER_VERTEX_COLOR
layout(location = 2) in vec4 color;
#else
layout(location = 2) in vec2 tex_coord;
#endif

layout(location = 0) out vec4 o_pos;
layout(location = 1) out vec3 o_normal;
#ifdef PER_VERTEX_COLOR
layout(location = 2) out vec4 o_color;
#else
layout(location = 2) out vec2 o_tex_coord;
#endif

void main()
{
    o_pos = global_uniform.model_mat * vec4(position, 1.0);
    o_normal = vec3(global_uniform.normal_mat * vec4(normal, 0.0));
#ifdef PER_VERTEX_COLOR
    o_color = color;
#else
    o_tex_coord = tex_coord;
#endif
    gl_Position = global_uniform.view_proj_mat * o_pos;

    #ifdef DRAW_POINTS_ONLY
    gl_PointSize = 1.0f;
    #endif
}
