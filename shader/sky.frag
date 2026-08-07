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

// Wispy cirrus clouds
float cloudDensity(vec3 rayDir, float time) {
    if (rayDir.y < 0.02) return 0.0;

    float cloudHeight = 0.15;
    float t = cloudHeight / max(0.001, rayDir.y);
    vec2 cloudUV = rayDir.xz * t;

    vec2 windOffset = vec2(time * 0.01, time * 0.005);
    cloudUV += windOffset;

    float largeScale = fbm(cloudUV * 0.5, 4);
    largeScale = smoothstep(0.4, 0.7, largeScale);

    float detail = fbm(cloudUV * 2.0 + vec2(time * 0.02, 0.0), 3);

    float streaks = fbm(cloudUV * vec2(4.0, 1.0) + vec2(0.0, time * 0.01), 3);
    streaks = smoothstep(0.45, 0.65, streaks);

    float density = largeScale * 0.3 + detail * 0.15 + streaks * 0.25;

    float horizonFade = smoothstep(0.02, 0.2, rayDir.y);
    density *= horizonFade;

    density = smoothstep(0.35, 0.65, density) * 0.5;

    return density;
}

vec3 renderClouds(vec3 rayDir, vec3 sunDir, float time) {
    float density = cloudDensity(rayDir, time);

    if (density < 0.001) return vec3(0.0);

    float sunHeight = max(0.0, sunDir.y);
    vec3 cloudColor = vec3(1.0, 1.0, 1.0);

    float sunDot = max(0.0, dot(rayDir, sunDir));
    vec3 sunTint = vec3(1.0, 0.98, 0.9) * pow(sunDot, 2.0) * 0.2;
    cloudColor += sunTint * sunHeight;

    float sunsetFactor = smoothstep(0.0, 0.2, sunHeight) * (1.0 - smoothstep(0.2, 0.5, sunHeight));
    vec3 sunsetTint = vec3(1.0, 0.75, 0.5) * sunsetFactor * 0.3;
    cloudColor += sunsetTint;

    cloudColor *= 0.95 + 0.15 * sunHeight;

    // Scaled alongside the sky so cloud and sky brightness stay in proportion.
    return cloudColor * density * SKY_RADIANCE_SCALE;
}

void main() {
    // Computed per fragment rather than interpolated, so the sun stays circular
    // at the edges of a wide field of view.
    vec3 rayDir = computeRayDir(fragClipPos);
    vec3 sunDir = normalize(pc.sun_direction.xyz);
    float time = pc.sun_direction.w;

    vec3 color = skyRadiance(rayDir, sunDir) + sunDiscRadiance(rayDir, sunDir);

    vec3 clouds = renderClouds(rayDir, sunDir, time);
    color = mix(color, color + clouds, min(1.0, length(clouds)));

    // Written as linear radiance. The post chain owns exposure and tonemapping;
    // resolving here would both double-tonemap and clamp the sun below the
    // bloom threshold, which is what stops it reading as a light source.
    outColor = vec4(color, 1.0);
}
