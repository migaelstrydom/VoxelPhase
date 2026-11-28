#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : require

#include "../tonemap.glsl"

// Extracts the portion of the HDR scene above a brightness threshold, which
// then gets blurred into the bloom texture. Runs at quarter resolution: each
// pixel averages the whole 4×4 block of the scene under it, in four bilinear
// taps, each held to a ceiling and thresholded on its own.

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D sceneHdr;

layout(push_constant) uniform BrightPassParams {
    /// x = threshold, y = soft knee width, z = ceiling, w unused.
    vec4 params;
} push;

/// What of `colour` blooms: held to `ceiling`, then the part above
/// `threshold`.
vec3 aboveThreshold(vec3 colour, float threshold, float knee, float ceiling) {
    float brightness = luminance(colour);
    if (brightness > ceiling) {
        colour *= ceiling / brightness;
        brightness = ceiling;
    }

    // Soft knee: contribution ramps in quadratically across
    // [threshold - knee, threshold + knee] so surfaces drifting across the
    // threshold fade rather than pop.
    float soft = clamp(brightness - threshold + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee);
    float contribution = max(soft, brightness - threshold) / max(brightness, 0.0001);
    return colour * contribution;
}

void main() {
    float threshold = push.params.x;
    float knee = max(push.params.y, 0.0001);
    float ceiling = push.params.z;

    // A tap one scene texel off the block's centre on each axis lands where
    // four texels meet, so the linear filter averages them: four such taps
    // cover the block. One tap at its centre saw only the middle 2×2, so a
    // highlight a few pixels across bloomed or vanished as it moved.
    vec2 texel = 1.0 / vec2(textureSize(sceneHdr, 0));
    vec3 bloom = vec3(0.0);
    for (int tap = 0; tap < 4; ++tap) {
        vec2 offset = vec2((tap & 1) == 0 ? -1.0 : 1.0, (tap & 2) == 0 ? -1.0 : 1.0);
        vec3 colour = texture(sceneHdr, inUv + offset * texel).rgb;
        bloom += aboveThreshold(colour, threshold, knee, ceiling);
    }

    outColor = vec4(bloom * 0.25, 1.0);
}
