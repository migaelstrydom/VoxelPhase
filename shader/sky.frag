#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "sky_model.glsl"

layout(location = 0) in vec2 fragClipPos;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PushConstants {
    mat4 inv_view;      // Inverse view matrix (rotation part transforms view->world)
    vec4 sun_direction; // xyz = normalized sun direction, w = time
    vec4 tan_fov;       // x = tan(fov/2)*aspect, y = tan(fov/2)
} pc;

// Noise functions for clouds
float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);

    float a = hash(i);
    float b = hash(i + vec2(1.0, 0.0));
    float c = hash(i + vec2(0.0, 1.0));
    float d = hash(i + vec2(1.0, 1.0));

    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

float fbm(vec2 p, int octaves) {
    float value = 0.0;
    float amplitude = 0.5;
    float frequency = 1.0;

    for (int i = 0; i < octaves; i++) {
        value += amplitude * noise(p * frequency);
        amplitude *= 0.5;
        frequency *= 2.0;
    }

    return value;
}

// Compute world-space ray direction per-fragment
vec3 computeRayDir(vec2 clipPos) {
    // Construct view-space ray direction from clip position
    // clipPos is in NDC (-1 to 1)
    // tan_fov.x = tan(fov/2) * aspect
    // tan_fov.y = tan(fov/2)
    // Note: Negate Y to account for Vulkan's flipped Y-axis in clip space
    vec3 viewDir = normalize(vec3(
        clipPos.x * pc.tan_fov.x,
        -clipPos.y * pc.tan_fov.y,
        -1.0
    ));

    // Transform from view space to world space using inverse view matrix
    // We only need the rotation part (upper-left 3x3)
    vec3 worldDir = mat3(pc.inv_view) * viewDir;

    return normalize(worldDir);
}

// Cirrus: thin, high ice cloud, combed into strands by the wind.
//
// Drawn on a plane above the eye, so the layer converges toward the horizon.
// Large patches decide where cirrus is; inside them, noise stretched along the
// wind draws the strands, and a coarser stretched layer bends them into the
// hooks and tufts that make it read as cirrus rather than as smoke.

/// Height of the cloud plane, in the units the projection is measured in.
const float CIRRUS_HEIGHT = 0.15;

/// Noise features per unit of the plane. Sets how many patches the sky holds:
/// too low and the whole visible sky falls inside one noise cell, all clear
/// or all cloud.
const float CIRRUS_SCALE = 14.0;

/// Direction the strands are combed along, on the plane.
const vec2 CIRRUS_WIND = vec2(0.94, 0.34);

/// How far the patches drift per second, in plane units.
const float CIRRUS_DRIFT = 0.02;

/// Fraction of the sky the patches cover, roughly.
const float CIRRUS_COVERAGE = 0.45;

/// Opacity of the densest cirrus. Cirrus is thin: the sky shows through it.
const float CIRRUS_MAX_OPACITY = 0.85;

/// Brightness of sunlit cirrus against the sky's radiance scale. Ice cloud
/// is brighter than the blue behind it; this is what makes it visible.
const float CIRRUS_BRIGHTNESS = 1.6;

/// Cirrus opacity along a view ray, 0..CIRRUS_MAX_OPACITY.
float cloudDensity(vec3 rayDir, float time) {
    vec2 plane = rayDir.xz * (CIRRUS_HEIGHT / max(rayDir.y, 0.02)) * CIRRUS_SCALE;
    vec2 across = vec2(-CIRRUS_WIND.y, CIRRUS_WIND.x);
    vec2 wind_frame = vec2(dot(plane, CIRRUS_WIND), dot(plane, across));
    wind_frame.x += time * CIRRUS_DRIFT;

    // How much of the plane, across the wind, one pixel covers. Taken before
    // any early return: derivatives need every pixel of the quad.
    float pixel_across = fwidth(wind_frame.y);

    if (rayDir.y < 0.02) return 0.0;

    // Wide ramps throughout: cirrus has no surface, it thins out. A narrow
    // threshold turns every noise contour into a hard edge.
    float patches = fbm(wind_frame * vec2(0.25, 0.5), 4);
    patches = smoothstep(0.55 - CIRRUS_COVERAGE * 0.3, 0.75 - CIRRUS_COVERAGE * 0.3, patches);

    // Hooks bend the strands: a coarse field offsets where across the wind
    // each strand is drawn, so they curl instead of running ruler-straight.
    float hooks = fbm(wind_frame * vec2(0.6, 1.2) + vec2(31.7, 5.3), 3);
    vec2 strand_frame = wind_frame + vec2(0.0, hooks * 0.8);
    float strands = fbm(strand_frame * vec2(0.8, 7.0) + vec2(-12.1, 47.9), 4);
    strands = smoothstep(0.3, 0.85, strands);

    // Fibres: far finer and more stretched than the strands, so each strand
    // is a bundle of hairs rather than one smooth band.
    // Toward the horizon they shrink below a pixel and would only alias, so
    // they fade to their average there.
    float fibres = fbm(strand_frame * vec2(1.5, 28.0) + vec2(7.3, -21.4), 3);
    fibres = smoothstep(0.25, 0.75, fibres);
    fibres = mix(fibres, 0.5, smoothstep(0.15, 0.4, pixel_across * 28.0));

    float density = patches * strands * mix(0.35, 1.0, fibres);

    // Fray: where the cloud is thin, fine noise breaks it into threads; the
    // cores are left whole. The edges dissolve instead of being cut.
    float fray = fbm(strand_frame * vec2(3.0, 16.0) + vec2(-3.9, 13.1), 2);
    fray = mix(fray, 0.6, smoothstep(0.15, 0.4, pixel_across * 16.0));
    density *= mix(fray * fray * 1.6, 1.0, smoothstep(0.1, 0.5, density));

    // Far toward the horizon the plane is so foreshortened that the strands
    // only shimmer; fade them into the haze.
    density *= smoothstep(0.03, 0.22, rayDir.y);

    return density * CIRRUS_MAX_OPACITY;
}

/// Radiance of cirrus seen along a ray: white, brightened toward the sun,
/// where ice scatters light strongly forward, and warmed at a low sun.
vec3 cloudRadiance(vec3 rayDir, vec3 sunDir) {
    float sunHeight = max(0.0, sunDir.y);
    float sunDot = max(0.0, dot(rayDir, sunDir));

    vec3 colour = vec3(1.0) * CIRRUS_BRIGHTNESS;
    colour += vec3(1.0, 0.97, 0.9) * pow(sunDot, 8.0) * 1.5 * sunHeight;

    float sunsetFactor = smoothstep(0.0, 0.2, sunHeight) * (1.0 - smoothstep(0.2, 0.5, sunHeight));
    colour = mix(colour, colour * vec3(1.0, 0.75, 0.5), sunsetFactor * 0.4);

    // Scaled alongside the sky so cloud and sky brightness stay in proportion.
    return colour * SKY_RADIANCE_SCALE;
}

void main() {
    // Computed per fragment rather than interpolated, so the sun stays circular
    // at the edges of a wide field of view.
    vec3 rayDir = computeRayDir(fragClipPos);
    vec3 sunDir = normalize(pc.sun_direction.xyz);
    float time = pc.sun_direction.w;

    vec3 color = skyRadiance(rayDir, sunDir) + sunDiscRadiance(rayDir, sunDir);

    // Over the sun disc too: cirrus passing in front of the sun veils it.
    color = mix(color, cloudRadiance(rayDir, sunDir), cloudDensity(rayDir, time));

    // Written as linear radiance. The post chain owns exposure and tonemapping;
    // resolving here would both double-tonemap and clamp the sun below the
    // bloom threshold, which is what stops it reading as a light source.
    outColor = vec4(color, 1.0);
}
