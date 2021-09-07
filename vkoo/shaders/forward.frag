#version 450

layout(set = 0, binding = 1) uniform GlobalUniform {
    mat4 model_mat;
    mat4 normal_mat;
    mat4 view_proj_mat;
    vec3 camera_position;
} global_uniform;

layout(location = 0) in vec4 in_pos;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec4 in_color;
layout(location = 3) in vec2 in_tex_coord;

layout(binding = 2) uniform sampler2D diffuse_tex;

layout(location = 0) out vec4 o_color;

struct Light {
    vec4 position;
    vec4 color;
    vec4 direction;
};

layout(set = 0, binding = 4) uniform LightsInfo {
    Light directional_lights[32];
    Light point_lights[32];
    uint directional_light_count;
    uint point_light_count;
} lights_info;

vec3 ApplyDirectionalLight(Light light, vec3 normal)
{
    vec3 world_to_light = -light.direction.xyz;
    world_to_light = normalize(world_to_light);
    float ndotl = clamp(dot(normal, world_to_light), 0.0, 1.0);
    return ndotl * light.color.w * light.color.rgb;
}

vec3 ApplyPointLight(Light light, vec3 pos, vec3 normal)
{
    vec3  world_to_light = light.position.xyz - pos;
    float dist = length(world_to_light) * 0.005;
    float atten = 1.0 / (dist * dist);
    world_to_light = normalize(world_to_light);
    float ndotl = clamp(dot(normal, world_to_light), 0.0, 1.0);
    return ndotl * light.color.w * atten * light.color.rgb;
}

void main() {
    vec3 normal = normalize(in_normal);
    vec3 light_contribution = vec3(0.0);
    for (uint i = 0U; i < lights_info.directional_light_count; i++) {
        light_contribution += ApplyDirectionalLight(lights_info.directional_lights[i], normal);
    }
    for (uint i = 0U; i < lights_info.point_light_count; i++) {
        light_contribution += ApplyPointLight(lights_info.point_lights[i], vec3(in_pos), normal);
    }
    light_contribution += vec3(0.2);// ambient light
    vec4 base_color = in_color;
    o_color = vec4(light_contribution * base_color.xyz, base_color.w);
    // o_color = vec4(1.0, 1.0, 1.0, 1.0);// base_color;
}