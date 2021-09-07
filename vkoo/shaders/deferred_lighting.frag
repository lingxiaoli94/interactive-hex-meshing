#version 450

layout(location = 0) in vec2 in_uv;
layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 0) uniform sampler2D depth_tex;
layout(set = 0, binding = 1) uniform sampler2D albedo_tex;
layout(set = 0, binding = 2) uniform sampler2D normal_tex;

#ifdef SSAO_ENABLED
layout(set = 0, binding = 3) uniform sampler2D ssao_tex;
#endif

layout(set = 0, binding = 4) uniform CameraUniform {
    mat4 clip_to_world_mat;
    vec2 inv_resolution;
} camera_uniform;

struct Light {
    vec4 position;
    vec4 color;
    vec4 direction;
};

layout(set = 0, binding = 5) uniform LightsInfo {
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

void main()
{
    // Retrieve position from depth. Note in_uv is in [0, 2].
    vec4 clip = vec4(in_uv * 2.0 - 1.0, texture(depth_tex, in_uv).x, 1.0);
    vec4 world_w = camera_uniform.clip_to_world_mat * clip;
    vec3 pos = world_w.xyz / world_w.w;
    vec4 albedo = texture(albedo_tex, in_uv);
    vec3 normal = texture(normal_tex, in_uv).xyz;
    // Calculate lighting.
    vec3 light_contribution = vec3(0.0);
    for (uint i = 0U; i < lights_info.directional_light_count; i++) {
        light_contribution += ApplyDirectionalLight(lights_info.directional_lights[i], normal);
    }
    for (uint i = 0U; i < lights_info.point_light_count; i++) {
        light_contribution += ApplyPointLight(lights_info.point_lights[i], pos, normal);
    }
    light_contribution += vec3(0.7);// ambient light

    vec4 base_color = albedo;
    if ((normal.x == 0) && (normal.y == 0) && (normal.z == 0)) {
        o_color = base_color;
    } else {
        o_color = vec4(light_contribution * base_color.xyz, base_color.w);
#ifdef SSAO_ENABLED
        o_color.rgb *= 1.0 - texture(ssao_tex, in_uv).x;
#endif
    }
}
