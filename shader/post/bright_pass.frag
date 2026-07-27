#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : require

#include "../tonemap.glsl"

// Extracts the portion of the HDR scene above a brightness threshold, which
// then gets blurred into the bloom texture. Runs at reduced resolution; the
// hardware's linear filter does the downsampling.

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D sceneHdr;

layout(push_constant) uniform BrightPassParams {
    /// x = threshold, y = soft knee width, zw unused.
    vec4 params;
} push;

void main() {
    vec3 colour = texture(sceneHdr, inUv).rgb;

    float threshold = push.params.x;
    float knee = max(push.params.y, 0.0001);

    // Soft knee: contribution ramps in quadratically across
    // [threshold - knee, threshold + knee] so surfaces drifting across the
    // threshold fade rather than pop.
    float brightness = luminance(colour);
    float soft = clamp(brightness - threshold + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee);
    float contribution = max(soft, brightness - threshold) / max(brightness, 0.0001);

    outColor = vec4(colour * contribution, 1.0);
}
