// Sun shadow lookup (set 0, binding 2).
//
// The map is a single orthographic depth buffer covering a slab around the
// camera, rendered by the pass in src/rendering/shadow/. Anything outside that
// slab reads as fully lit rather than fully shadowed: an under-covered frame
// should lose its shadows, not gain a black wall at the volume's edge.

#ifndef SHADOW_GLSL
#define SHADOW_GLSL

#include "scene.glsl"

// A comparison sampler, so each tap returns a filtered 0/1 occlusion rather
// than a depth to compare by hand. That makes every tap worth four with linear
// filtering, which is where most of the softness below comes from.
layout(set = 0, binding = 2) uniform sampler2DShadow shadow_map;

/// Half-width of the PCF kernel, in texels. 1 gives the 3x3 taps that, with
/// hardware comparison filtering, cover a 4x4 texel footprint.
const int SHADOW_PCF_RADIUS = 1;

/// Slope scaling applied to the normal offset.
///
/// The constant offset handles a surface facing the sun; a surface at a grazing
/// angle spans far more depth across one shadow texel and needs proportionally
/// more. This is the shader-side half of the bias — the pass itself also
/// applies a slope-scaled depth bias in raster state, which handles the
/// depth-quantisation half.
const float SHADOW_SLOPE_OFFSET = 2.0;

/// Fraction of the map's half-width at which shadows start fading out.
///
/// Without this the volume's edge is a visible line where shadows stop. The
/// fade converts it into a gradient a viewer reads as distance haze.
const float SHADOW_FADE_START = 0.85;

/// How much of the sun is blocked at `world_pos`: 0 fully lit, 1 fully shadowed.
///
/// `normal` and `light_dir` are normalized, `light_dir` pointing from the
/// surface towards the sun.
float sunShadow(vec3 world_pos, vec3 normal, vec3 light_dir) {
    float strength = scene.shadow_params.z;
    if (strength <= 0.0) {
        return 0.0;
    }

    // Offsetting along the normal rather than along the light avoids the
    // peter-panning a pure depth bias produces: the sample moves across the
    // surface instead of away from the caster, so contact stays put.
    float n_dot_l = clamp(dot(normal, light_dir), 0.0, 1.0);
    float offset = scene.shadow_params.y * (1.0 + SHADOW_SLOPE_OFFSET * (1.0 - n_dot_l));
    vec4 light_clip = scene.light_view_proj * vec4(world_pos + normal * offset, 1.0);

    vec3 ndc = light_clip.xyz / light_clip.w;
    if (ndc.z <= 0.0 || ndc.z >= 1.0) {
        return 0.0;
    }

    vec2 uv = ndc.xy * 0.5 + 0.5;
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) {
        return 0.0;
    }

    float texel = scene.shadow_params.x;
    float occlusion = 0.0;
    for (int y = -SHADOW_PCF_RADIUS; y <= SHADOW_PCF_RADIUS; ++y) {
        for (int x = -SHADOW_PCF_RADIUS; x <= SHADOW_PCF_RADIUS; ++x) {
            vec2 tap = uv + vec2(x, y) * texel;
            occlusion += 1.0 - texture(shadow_map, vec3(tap, ndc.z));
        }
    }
    float taps = float((2 * SHADOW_PCF_RADIUS + 1) * (2 * SHADOW_PCF_RADIUS + 1));
    occlusion /= taps;

    // Distance from the centre of the map in UV, as a fraction of its half
    // width — 0 at the focus point, 1 at the border.
    float edge = max(abs(uv.x - 0.5), abs(uv.y - 0.5)) * 2.0;
    float fade = 1.0 - smoothstep(SHADOW_FADE_START, 1.0, edge);

    return occlusion * fade * strength;
}

#endif // SHADOW_GLSL
