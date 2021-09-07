#version 450

layout(set = 0, binding = 0) uniform sampler2D ssao_tex;

layout(location = 0) in vec2 in_uv;

layout(location = 0) out float o_blurred_occlusion;

void main()
{
    const int blurRange = 2;
    int n = 0;
    vec2 texel_size = 1.0 / vec2(textureSize(ssao_tex, 0));
    float result = 0.0;
    for (int x = -blurRange; x < blurRange; x++) {
        for (int y = -blurRange; y < blurRange; y++) {
            vec2 offset = vec2(float(x), float(y)) * texel_size;
            result += texture(ssao_tex, in_uv + offset).r;
            n++;
        }
    }
    o_blurred_occlusion = result / (float(n));
}