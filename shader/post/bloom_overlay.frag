#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : require

#include "../tonemap.glsl"

// Adds the blurred bloom over the finished frame, after transparent geometry.
//
// Bloom is a screen-space bleed with no depth of its own, so compositing it
// before the transparent pass let opaque-writing surfaces (water) paint over a
// halo that should have been in front of them. Running it last means glow reads
// as light in the air rather than as a texture on the scene behind.
//
// The pipeline screen-blends this output over the finished frame
// (`dst + src * (1 - dst)`), so this shader outputs the glow alone. Screen
// blending requires its source in [0, 1], which is what the ACES curve below
// guarantees — keep the tonemap, or bright glow will clip to white again.

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D bloom;

layout(push_constant) uniform BloomOverlayParams {
    /// x = bloom intensity, y = exposure, z = hue preservation strength,
    /// w unused.
    vec4 params;
} push;

void main() {
    vec3 glow = texture(bloom, inUv).rgb * push.params.x * push.params.y;

    // Curve the glow on its own so it stays inside the display range; the
    // framebuffer it blends onto already holds tonemapped colour. Uses the same
    // curve as the composite so the halo and the scene agree.
    outColor = vec4(tonemapScene(glow, push.params.z), 1.0);
}
