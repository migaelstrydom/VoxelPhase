#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "tonemap.glsl"

// A fall's sheet: aerated water over the scene behind it. Streaks run down
// the arc at the water's own pace (they are keyed to seconds from the lip),
// the sheet whitens as it falls and breaks up, and it thins to nothing at
// its edges. Below the surface of the water it enters, nothing is drawn;
// a drop into water standing over its lip stays clear rather than white.

const vec3 WATER_TINT = vec3(0.55, 0.72, 0.80);
const vec3 AERATED = vec3(0.92, 0.96, 1.0);
const float REFRACTION = 0.006;

layout(location = 0) in vec3 fragWorldPos;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in float fragAcross;
layout(location = 3) in float fragTime;
layout(location = 4) in float fragAlong;
layout(location = 5) flat in vec4 fragBody;   // (strength, clock, cut, aeration)

layout(set = 0, binding = 0) uniform sampler2D colorSampler;
layout(set = 0, binding = 1) uniform sampler2D depthSampler;

layout(push_constant) uniform FragPushConstants {
    layout(offset = 160) vec4 cameraPos;
    layout(offset = 176) vec4 sunDir;
    layout(offset = 192) vec4 projParams;    // (near, far, time, unused)
    layout(offset = 208) vec4 screenParams;  // (width, height, hue preservation, exposure)
} fpc;

layout(location = 0) out vec4 outColor;

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1.0, 0.0)), u.x),
               mix(hash(i + vec2(0.0, 1.0)), hash(i + vec2(1.0, 1.0)), u.x), u.y);
}

void main() {
    float strength = fragBody.x;
    float clock = fragBody.y;
    if (fragWorldPos.y < fragBody.z) {
        discard;
    }
    float aeration = fragBody.w;

    // Streaks: fine across the sheet, long down it, carried with the water.
    vec2 streakUv = vec2(fragAcross * 6.0, (fragTime - clock) * 4.0);
    float streaks = 0.6 * noise(streakUv) + 0.4 * noise(streakUv * vec2(2.3, 1.7) + 3.1);

    // The sheet thins at its edges and, lower down, breaks into gaps.
    float edge = 1.0 - smoothstep(0.55, 1.0, abs(fragAcross));
    float breakUp = smoothstep(0.25 + 0.5 * fragAlong, 0.75, streaks);
    float coverage = edge * mix(0.85, breakUp, fragAlong * 0.8 * aeration) * strength;
    if (coverage < 0.02) {
        discard;
    }

    vec2 screenUV = gl_FragCoord.xy / fpc.screenParams.xy;
    vec2 offset = (vec2(streaks) - 0.5) * REFRACTION;
    vec3 behind = texture(colorSampler, clamp(screenUV + offset, vec2(0.001), vec2(0.999))).rgb;

    vec3 L = normalize(fpc.sunDir.xyz);
    float lit = 0.6 + 0.4 * abs(dot(normalize(fragNormal), L));
    float white = clamp(0.35 + 0.65 * fragAlong + 0.3 * streaks, 0.0, 1.0) * mix(0.3, 1.0, aeration);
    vec3 water = mix(WATER_TINT, AERATED, white);
    vec3 sheet = water * lit;

    // Where the sheet is thin the scene shows through, tinted only where
    // there is enough water to tint it.
    vec3 tinted = mix(behind, behind * WATER_TINT, smoothstep(0.02, 0.3, coverage));
    vec3 color = mix(tinted, sheet, coverage);

    // As in water.frag: `colorSampler` is the raw HDR scene, drawn over the
    // tonemapped one, so apply the composite's exposure and curve.
    outColor = vec4(tonemapScene(color * fpc.screenParams.w, fpc.screenParams.z), 1.0);
}
