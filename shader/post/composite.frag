#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_GOOGLE_include_directive : require

#include "../tonemap.glsl"

// Resolves the HDR scene to the swapchain: exposure, then the filmic curve.
// Bloom is *not* added here — it is applied after the transparent pass by
// bloom_overlay.frag, so that water and other transparent surfaces cannot paint
// over the halo.
//
// The swapchain is an sRGB format, so the hardware applies the transfer
// function on write and this shader outputs linear values.

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D sceneHdr;

layout(push_constant) uniform CompositeParams {
    /// x = bloom intensity (unused here), y = exposure, zw unused.
    vec4 params;
} push;

void main() {
    vec3 scene = texture(sceneHdr, inUv).rgb * push.params.y;

    outColor = vec4(tonemapACES(scene), 1.0);
}
