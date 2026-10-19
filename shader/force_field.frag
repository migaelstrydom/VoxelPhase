#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

// A force-field column, shaded as light rather than as a surface: it is
// written premultiplied into the HDR scene, so most of what it adds is glow
// and only a little of what is behind it is hidden.
//
// What the vertices carry (packed by `force_field::column`):
//   colour.rgb   the field's radiance colour, cyan at rest to white-hot
//   colour.a     its presence: 0 is no field, 1 a pod carrying its share
//   texCoord.x   the band coordinate, in bands, already scrolled: a band's
//                leading edge is where it crosses a whole number
//   texCoord.y   how far down the column, 0 at the pod and 1 at the ground
//   strain       0..1, how hard a load spike is straining the field

#include "scene.glsl"

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inTexCoord;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in vec3 inNormal;
layout(location = 4) in float inStrain;

layout(location = 0) out vec4 outColor;

/// Fresnel exponent of the rim: higher is a thinner, harder edge.
const float RIM_POWER = 2.6;
/// How sharply a band's glow trails off above its leading edge.
const float BAND_TAIL = 5.0;
/// Band coordinate change per pixel at which the bands are blurred into an
/// even glow rather than shimmering.
const float BAND_BLUR_FROM = 0.25;
const float BAND_BLUR_TO = 0.6;
/// How far the bands ripple, in bands.
const float RIPPLE = 0.12;
/// Length of the column, from the pod, that flickers under strain.
const float STRAIN_REACH = 0.4;

float hash13(vec3 p) {
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}

void main() {
    vec3 n = normalize(inNormal);
    vec3 toEye = normalize(scene.camera_pos.xyz - inWorldPos);
    float rim = pow(1.0 - abs(dot(n, toEye)), RIM_POWER);

    float along = inTexCoord.y;
    // The bands ripple around the column rather than ringing it like ribs:
    // a field, not a stack of hoops.
    float around = atan(n.z, n.x);
    float b = inTexCoord.x + RIPPLE * sin(3.0 * around + 9.0 * along);

    // A band is a sharp leading edge, moving down the column, with a glow
    // trailing above it. Far away the bands are finer than a pixel and are
    // averaged into the glow they would make.
    float x = fract(b);
    float band = pow(x, BAND_TAIL) * (1.0 - smoothstep(0.93, 1.0, x));
    float blur = smoothstep(BAND_BLUR_FROM, BAND_BLUR_TO, fwidth(b));
    band = mix(band, 1.0 / (BAND_TAIL + 1.0), blur);

    // Brighter toward the ground, where the field presses; faded in at the
    // pod and out where it meets the ground, so neither end is a hard cut.
    float ends = smoothstep(0.0, 0.05, along) * (1.0 - smoothstep(0.96, 1.0, along));
    float press = mix(0.5, 1.0, along) + 0.5 * smoothstep(0.75, 0.97, along);
    float presence = inColor.a * ends;

    // Under strain the field near the cap breaks into sparks that churn with
    // the bands.
    float cap = 1.0 - smoothstep(0.0, STRAIN_REACH, along);
    float sparkle = step(0.72, hash13(floor(inWorldPos * 28.0) + floor(b * 2.0)));
    float flicker = inStrain * cap * sparkle;

    vec3 glow = inColor.rgb * presence * press
        * (0.06 + 1.4 * rim + band * (1.0 + 0.6 * rim));
    glow += vec3(1.0) * flicker * 1.6 * presence;

    float cover = clamp(presence * press * (0.02 + 0.18 * rim + 0.06 * band), 0.0, 0.5);
    outColor = vec4(glow, cover);
}
