#version 450

layout(location = 0) in vec2 in_uv;
layout(location = 0) out float o_occlusion;

layout(set = 0, binding = 0) uniform sampler2D depth_tex;
layout(set = 0, binding = 1) uniform sampler2D normal_tex;
layout(set = 0, binding = 2) uniform sampler2D ssao_noise_tex;

layout(constant_id = 0) const uint kSSAOKernelSize = 64;
layout(constant_id = 1) const float kSSAORadius = 0.2;

layout(set = 0, binding = 3) uniform SSAOKernel {
    vec4 samples[kSSAOKernelSize];
} ssao_kernel;

layout(set = 0, binding = 4) uniform CameraUniform {
    mat4 world_to_clip_mat;
    mat4 clip_to_world_mat;
} camera_uniform;


void main() {
    float depth = texture(depth_tex, in_uv).x;
    vec4 clip = vec4(in_uv * 2.0 - 1.0, depth, 1.0);
    vec4 world_w = camera_uniform.clip_to_world_mat * clip;
    vec3 frag_pos = world_w.xyz / world_w.w;// world position of the fragment

    vec3 normal = texture(normal_tex, in_uv).xyz;
    normal = normalize(2.0 * normal - 1.0);

    ivec2 tex_dim = textureSize(depth_tex, 0);
    ivec2 noise_tex_dim = textureSize(ssao_noise_tex, 0);
    const vec2 noise_uv = vec2(float(tex_dim.x) / float(noise_tex_dim.x), float(tex_dim.y) / float(noise_tex_dim.y)) * in_uv;
    vec3 rand_vec = texture(ssao_noise_tex, noise_uv).xyz * 2.0 - 1.0;

    // Create TBN frame.
    vec3 tangent = normalize(rand_vec - normal * dot(rand_vec, normal));
    vec3 bitangent = cross(tangent, normal);
    mat3 TBN = mat3(tangent, bitangent, normal);

    float occlusion = 0.0;
    const float bias = 0.0025;
    for (uint i = 0; i < kSSAOKernelSize; i++) {
        vec3 sample_pos = TBN * ssao_kernel.samples[i].xyz;
        sample_pos = frag_pos + kSSAORadius * sample_pos;

        vec4 sample_clip_pos = camera_uniform.world_to_clip_mat * vec4(sample_pos, 1.0);
        sample_clip_pos.xyz /= sample_clip_pos.w;
        // Scale from clip position to uv.
        float sample_depth = texture(depth_tex, sample_clip_pos.xy * 0.5 + 0.5).x;

        float range_check = smoothstep(0.0, 1.0, kSSAORadius / abs(depth - sample_depth));
        // float range_check = 1.0f;
        occlusion += (sample_depth + bias <= sample_clip_pos.z ? 1.0: 0.0) * range_check;
    }
    occlusion = occlusion / kSSAOKernelSize;

    o_occlusion = occlusion;
}
