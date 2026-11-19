#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

// A shaft of light standing up into the sky, shaded as light rather than as
// a surface: written premultiplied into the HDR scene with no coverage, so
// it only ever adds light and hides nothing behind it. Its vertex stage is
// `force_field.vert`, which passes the ordinary vertex format through.
//
// What the vertices carry (packed by `light_shaft::mesh`):
//   colour.rgb   the shaft's colour times its radiance
//   colour.a     its presence, 0..1
//   texCoord.x   metres up the shaft, less how far its streaks have risen
//   texCoord.y   how far up, 0 at its foot and 1 at its top

#include "scene.glsl"

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inTexCoord;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in vec3 inNormal;
layout(location = 4) in float inUnused;

layout(location = 0) out vec4 outColor;

/// How fast the shaft thins toward its edges: seen through its middle it is
/// a thick column of light, at its edge almost none, so it has no outline.
const float CORE_POWER = 1.8;
/// How the light dies away up the shaft, and the share of it its foot
/// takes to come on: a few metres, so it gathers out of the air rather
/// than starting at a line.
const float FADE_POWER = 1.4;
const float FOOT = 0.03;
/// Streaks round the shaft and up it, and how much of the light is in them.
const float STREAKS_AROUND = 9.0;
const float STREAK_LENGTH = 2.5;
const float STREAK_SHARE = 0.45;

float hash12(vec2 p) {
    vec3 q = fract(vec3(p.xyx) * 0.1031);
    q += dot(q, q.yzx + 33.33);
    return fract((q.x + q.y) * q.z);
}

/// Smooth value noise, about 0..1.
float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash12(i), hash12(i + vec2(1.0, 0.0)), u.x),
               mix(hash12(i + vec2(0.0, 1.0)), hash12(i + vec2(1.0, 1.0)), u.x), u.y);
}

void main() {
    vec3 n = normalize(inNormal);
    vec3 toEye = normalize(scene.camera_pos.xyz - inWorldPos);
    // The column's depth along the line of sight, as a share of its widest:
    // how squarely this side of the tube is seen.
    float core = pow(abs(dot(n, toEye)), CORE_POWER);

    float along = inTexCoord.y;
    float fade = smoothstep(0.0, FOOT, along) * pow(1.0 - along, FADE_POWER);

    // Streaks of brighter light rising through it, wrapped round it. Far
    // off they are finer than a pixel and blur into the even glow.
    float around = atan(n.z, n.x) / 6.2831853 * STREAKS_AROUND;
    float streak = noise(vec2(around, inTexCoord.x / STREAK_LENGTH));
    float blur = smoothstep(0.2, 0.6, fwidth(inTexCoord.x / STREAK_LENGTH));
    streak = mix(streak, 0.5, blur);
    float body = (1.0 - STREAK_SHARE) + STREAK_SHARE * 2.0 * streak;

    vec3 glow = inColor.rgb * inColor.a * fade * core * body;
    outColor = vec4(glow, 0.0);
}
