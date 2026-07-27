#version 450
#extension GL_ARB_separate_shader_objects : enable

// One axis of a separable Gaussian blur. Run twice (horizontal, then vertical)
// to get a 2D blur for a fraction of the taps a 2D kernel would need.
//
// Uses the linear-sampling trick: each tap sits between two texels so the
// hardware's bilinear filter blends them, giving a 9-tap Gaussian from 5 reads.

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D source;

layout(push_constant) uniform BlurParams {
    /// xy = per-tap step in UV space (texel size along the blur axis), zw unused.
    vec4 direction;
} push;

const float OFFSETS[3] = float[](0.0, 1.3846153846, 3.2307692308);
const float WEIGHTS[3] = float[](0.2270270270, 0.3162162162, 0.0702702703);

void main() {
    vec3 result = texture(source, inUv).rgb * WEIGHTS[0];

    for (int i = 1; i < 3; i++) {
        vec2 offset = push.direction.xy * OFFSETS[i];
        result += texture(source, inUv + offset).rgb * WEIGHTS[i];
        result += texture(source, inUv - offset).rgb * WEIGHTS[i];
    }

    outColor = vec4(result, 1.0);
}
