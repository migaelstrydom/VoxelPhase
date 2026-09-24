#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "tonemap.glsl"

// --- Tuning constants ---
const float REFRACTION_STRENGTH = 0.01;
const float SSS_INTENSITY = 1.9;
const vec3  SSS_COLOR = vec3(0.15, 0.5, 0.4);

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragWorldPos;

// Opaque color target (sampled at offset UVs for refraction)
layout(set = 0, binding = 0) uniform sampler2D colorSampler;

// Depth buffer (sampled at both current and refracted UVs)
layout(set = 0, binding = 1) uniform sampler2D depthSampler;

layout(push_constant) uniform FragPushConstants {
    layout(offset = 144) vec4 cameraPos;
    layout(offset = 160) vec4 sunDir;
    layout(offset = 176) vec4 projParams;    // (near, far, time, unused)
    layout(offset = 192) vec4 screenParams;  // (width, height, hue preservation, exposure)
} fpc;

layout(location = 0) out vec4 outColor;

// Linearize a [0,1] depth buffer value to a view-space distance.
float linearizeDepth(float d, float near, float far) {
    return near * far / (far - d * (far - near));
}

// Hash-based pseudo-random for procedural noise (no texture needed).
vec2 hash2(vec2 p) {
    p = vec2(dot(p, vec2(127.1, 311.7)),
             dot(p, vec2(269.5, 183.3)));
    return fract(sin(p) * 43758.5453);
}

// Value noise with smooth interpolation.
float valueNoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);

    float a = dot(hash2(i + vec2(0.0, 0.0)), vec2(1.0));
    float b = dot(hash2(i + vec2(1.0, 0.0)), vec2(1.0));
    float c = dot(hash2(i + vec2(0.0, 1.0)), vec2(1.0));
    float d = dot(hash2(i + vec2(1.0, 1.0)), vec2(1.0));

    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// Two-octave FBM for richer detail.
float fbm(vec2 p) {
    float v = 0.0;
    v += 0.5 * valueNoise(p);
    v += 0.25 * valueNoise(p * 2.0 + vec2(1.7, 9.2));
    return v;
}

// Sample a normal perturbation from procedural noise at a given position.
// Uses central differences on the noise heightfield.
vec3 proceduralNormal(vec2 pos, float scale, float strength) {
    float eps = 0.1;
    float h  = fbm(pos * scale);
    float hx = fbm((pos + vec2(eps, 0.0)) * scale);
    float hz = fbm((pos + vec2(0.0, eps)) * scale);
    float dx = (hx - h) / eps * strength;
    float dz = (hz - h) / eps * strength;
    return normalize(vec3(-dx, 1.0, -dz));
}

void main() {
    float time = fpc.projParams.z;

    // --- Animated normal mapping (dual-layer procedural noise) ---
    vec2 worldXZ = fragWorldPos.xz;
    vec3 noiseN1 = proceduralNormal(worldXZ + vec2(time * 0.3, time * 0.2), 0.5, 0.07);
    vec3 noiseN2 = proceduralNormal(worldXZ + vec2(-time * 0.15, time * 0.25), 1.0, 0.04);

    // Blend noise normals with the vertex normal.
    vec3 vertexN = normalize(fragNormal);
    vec3 N = normalize(vec3(
        vertexN.x + noiseN1.x + noiseN2.x,
        vertexN.y,
        vertexN.z + noiseN1.z + noiseN2.z
    ));

    vec3 V = normalize(fpc.cameraPos.xyz - fragWorldPos);
    vec3 L = normalize(fpc.sunDir.xyz);

    float near = fpc.projParams.x;
    float far = fpc.projParams.y;

    // Screen UV for texture sampling.
    vec2 screenUV = gl_FragCoord.xy / fpc.screenParams.xy;

    // --- Volumetric depth from depth buffer ---
    float terrainDepthRaw = texture(depthSampler, screenUV).r;
    float waterDepthRaw = gl_FragCoord.z;

    float terrainDist = linearizeDepth(terrainDepthRaw, near, far);
    float waterDist = linearizeDepth(waterDepthRaw, near, far);

    float opticalDepth = max(terrainDist - waterDist, 0.0);

    float depthFactor = 1.0 - exp(-opticalDepth * 0.06);

    // --- Screen-space refraction ---

    // Offset UV by the water normal's XZ, scaled by optical depth.
    // Uses smoothstep to suppress refraction in shallow water (< 0.5m),
    // avoiding double-image artifacts where the offset is too small to
    // clear an object's silhouette in the color target.
    float refractionDepth = smoothstep(0.5, 2.0, opticalDepth) * 3.0;
    vec2 refractionOffset = N.xz * REFRACTION_STRENGTH * refractionDepth;

    vec2 refractedUV = clamp(screenUV + refractionOffset, vec2(0.001), vec2(0.999));

    // Reject refraction offsets that land on pixels above the water surface
    // (e.g. a character's head poking out). If the depth at the refracted UV
    // is closer than the water fragment, fall back to the undistorted UV.
    float refractedDepthRaw = texture(depthSampler, refractedUV).r;
    if (refractedDepthRaw < waterDepthRaw) {
        refractedUV = screenUV;
        refractedDepthRaw = terrainDepthRaw;
    }

    vec3 terrainColor = texture(colorSampler, refractedUV).rgb;

    float refractedTerrainDist = linearizeDepth(refractedDepthRaw, near, far);
    float refractedOpticalDepth = max(refractedTerrainDist - waterDist, 0.0);

    // Depth-based absorption: red absorbs fastest (realistic turquoise tint).
    // Uses the optical depth at the refracted UV for correct absorption.
    vec3 refractedTerrain = terrainColor * exp(-vec3(0.04, 0.01, 0.02) * refractedOpticalDepth);

    // Deep water blend factor at the refracted point (matches the terrain we're showing).
    float refractedDepthFactor = 1.0 - exp(-refractedOpticalDepth * 0.06);

    // --- Fresnel reflectance (Schlick approximation) ---
    float F0 = 0.02;
    float NdotV = max(dot(N, V), 0.0);
    float fresnel = F0 + (1.0 - F0) * pow(1.0 - NdotV, 5.0);

    // Sky/reflection color approximated from reflected direction.
    vec3 R = reflect(-V, N);
    float skyFactor = max(R.y, 0.0);
    vec3 horizonColor = vec3(0.6, 0.7, 0.8);
    vec3 zenithColor  = vec3(0.25, 0.4, 0.7);
    vec3 reflectionColor = mix(horizonColor, zenithColor, skyFactor);

    // Deep water color (only this gets diffuse lighting).
    vec3 deepColor = vec3(0.02, 0.12, 0.22);
    float NdotL = max(dot(N, L), 0.0);
    vec3 litDeepColor = deepColor * (0.5 + 0.5 * NdotL);

    // Blend refracted terrain toward deep water as optical depth increases.
    // Uses refracted depth factor so the blend matches the terrain being shown.
    vec3 waterBody = mix(refractedTerrain, litDeepColor, refractedDepthFactor);

    // Blend water body with reflection via Fresnel.
    vec3 color = mix(waterBody, reflectionColor, fresnel);

    // --- Specular highlight (Blinn-Phong) ---
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);
    float spec = pow(NdotH, 256.0);
    vec3 sunColor = vec3(1.0, 0.95, 0.8);
    color += sunColor * spec * 0.8;

    // --- Subsurface scattering approximation ---
    float sss = pow(max(dot(V, -L), 0.0), 4.0) * (1.0 - depthFactor) * SSS_INTENSITY;
    color += SSS_COLOR * sss;

    // --- Shoreline foam ---
    float foamFactor = 1.0 - smoothstep(0.0, 0.3, opticalDepth);
    color = mix(color, vec3(0.9, 0.95, 1.0), foamFactor * 0.6);

    // The water pass draws onto the swapchain, which already holds the
    // tonemapped scene, but `colorSampler` is the raw HDR scene target. Apply
    // the same exposure and curve the composite pass used, or refracted terrain
    // resolves differently from the terrain beside it and the water reads as a
    // brightness seam rather than a surface.
    outColor = vec4(tonemapScene(color * fpc.screenParams.w, fpc.screenParams.z), 1.0);
}
