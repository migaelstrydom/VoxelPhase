#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "ripple.glsl"
#include "swell.glsl"

// --- Tuning constants ---
const float REFRACTION_STRENGTH = 0.01;
const float SSS_INTENSITY = 1.9;
const vec3  SSS_COLOR = vec3(0.15, 0.5, 0.4);
// How long the noise is carried by the flow before it starts over, s. Two
// phases half a period apart cross-fade, each faded out as it starts over,
// so the noise is never carried further than a period's flow: carried for
// ever, the small currents of still water would shear it into swirls.
const float FLOW_PERIOD = 2.0;
// Steepest a ripple may tilt the normal: past this the surface is noise.
const float MAX_RIPPLE_SLOPE = 1.0;
// How fast a falling sheet's streaks run down it, m/s.
const float SHEET_FALL_SPEED = 4.0;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragWorldPos;
// The ripple tile under this fragment, or -1 where none is awake, and the
// tile's corner.
layout(location = 2) flat in int fragLayer;
layout(location = 3) flat in vec2 fragTileOrigin;
// A river surface's flow velocity, its depth of water here and the depths it
// is drawn over; zero flow and an unbounded range for still water.
layout(location = 4) in vec2 fragFlow;
layout(location = 5) in float fragWetDepth;
layout(location = 6) flat in vec2 fragWetRange;
// The swell under this fragment: (amplitude with its shore fade, phase,
// clock, unused); zero amplitude where there is none.
layout(location = 7) in vec4 fragSwell;
// Whether this is a tile's own fine grid, which covers the whole tile and is
// masked to its body's columns.
layout(location = 8) flat in int fragMasked;

// The scene beyond the water, copied before the water is drawn (sampled at
// offset UVs for refraction)
layout(set = 0, binding = 0) uniform sampler2D colorSampler;

// Its depth, copied with it (sampled at both current and refracted UVs)
layout(set = 0, binding = 1) uniform sampler2D depthSampler;

layout(push_constant) uniform FragPushConstants {
    layout(offset = 160) vec4 cameraPos;
    layout(offset = 176) vec4 sunDir;
    layout(offset = 192) vec4 projParams;    // (near, far, time, unused)
    layout(offset = 208) vec4 screenParams;  // (width, height, unused, unused)
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

// Where the scene seen through the water at `screenUV` is looked up, for
// something `opticalDepth` metres of view ray beyond the surface. Nothing
// under 0.5 m shifts: at that depth the offset would be too small to clear
// an object's silhouette, and would only fringe it.
vec2 refractedLookup(vec2 screenUV, vec3 N, float opticalDepth) {
    float refractionDepth = smoothstep(0.5, 2.0, opticalDepth) * 3.0;
    vec2 offset = N.xz * REFRACTION_STRENGTH * refractionDepth;
    return clamp(screenUV + offset, vec2(0.001), vec2(0.999));
}

void main() {
    // How much ground a pixel covers, taken before any fragment is discarded.
    float footprint = max(length(dFdx(fragWorldPos.xz)), length(dFdy(fragWorldPos.xz)));

    // A tile's fine grid covers its whole 8 m square; only its body's
    // columns are this body's water.
    if (fragMasked != 0 && isnan(rippleFloorAt(fragLayer, fragWorldPos.xz - fragTileOrigin))) {
        discard;
    }

    // A river surface is drawn only where its water is deep enough.
    if (fragWetDepth < fragWetRange.x || fragWetDepth > fragWetRange.y) {
        discard;
    }

    float time = fpc.projParams.z;

    // --- Animated normal mapping (dual-layer procedural noise) ---
    // Running water carries its detail downstream: the noise scrolls with the
    // flow, over a drift of its own.
    vec2 worldXZ = fragWorldPos.xz;
    float speed = length(fragFlow);
    float phases[2] = float[2](fract(time / FLOW_PERIOD), fract(time / FLOW_PERIOD + 0.5));
    float firstWeight = 1.0 - abs(1.0 - 2.0 * phases[0]);
    vec3 noiseN1 = vec3(0.0);
    vec3 noiseN2 = vec3(0.0);
    float whiteNoise = 0.0;
    for (int p = 0; p < 2; p++) {
        // Each phase starts over at its own offset, so the restart is not
        // seen as the same pattern coming back.
        vec2 carried = -fragFlow * phases[p] * FLOW_PERIOD + vec2(0.5 * float(p), 0.37 * float(p));
        float weight = p == 0 ? firstWeight : 1.0 - firstWeight;
        noiseN1 += weight * proceduralNormal(worldXZ + carried + vec2(time * 0.3, time * 0.2), 0.5, 0.07 + 0.015 * min(speed, 2.0));
        noiseN2 += weight * proceduralNormal(worldXZ + carried * 1.3 + vec2(-time * 0.15, time * 0.25), 1.0, 0.04 + 0.015 * min(speed, 2.0));
        whiteNoise += weight * fbm(worldXZ * 2.0 + carried * 2.0);
    }

    // The ripples' normal is read per pixel where a tile is awake; the
    // swell's slope is added to it.
    vec3 rippleN = vec3(0.0, 1.0, 0.0);
    if (fragLayer >= 0) {
        float extent = float(RIPPLE_COLUMNS) * RIPPLE_COLUMN;
        vec2 local = clamp(worldXZ - fragTileOrigin, vec2(0.0), vec2(extent));
        vec2 slope = rippleGradientAt(fragLayer, local);
        float steepness = length(slope);
        slope *= steepness > MAX_RIPPLE_SLOPE ? MAX_RIPPLE_SLOPE / steepness : 1.0;
        rippleN = normalize(vec3(-slope.x, 1.0, -slope.y));
    }
    vec2 swellSlope = swellSlopeAt(worldXZ, fragSwell.z, fragSwell.x, fragSwell.y, footprint);
    vec3 vertexN = normalize(vec3(
        rippleN.x / rippleN.y - swellSlope.x,
        1.0,
        rippleN.z / rippleN.y - swellSlope.y
    ));

    // Blend noise normals with the vertex normal.
    vec3 N = normalize(vec3(
        vertexN.x + noiseN1.x + noiseN2.x,
        vertexN.y,
        vertexN.z + noiseN1.z + noiseN2.z
    ));

    vec3 V = normalize(fpc.cameraPos.xyz - fragWorldPos);

    // A sheet falling over an edge faces the way it falls, turned to the
    // camera from either side, and is white water streaked down its face.
    float sheetFoam = 0.0;
    if (abs(fragNormal.y) < 0.5) {
        vec3 face = normalize(vec3(fragNormal.x, 0.0, fragNormal.z));
        face = dot(face, V) < 0.0 ? -face : face;
        float along = dot(worldXZ, vec2(-face.z, face.x));
        float streaks = fbm(vec2(along * 5.0, (fragWorldPos.y + time * SHEET_FALL_SPEED) * 0.4));
        N = normalize(face + vec3(0.0, 0.3 * (streaks - 0.75), 0.0) + 0.5 * vec3(noiseN1.x, 0.0, noiseN1.z));
        sheetFoam = 0.9 * smoothstep(0.6, 1.0, streaks);
    }
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

    // Offset UV by the water normal's XZ. How far something under the water
    // appears to shift depends on its own depth below the surface, not on the
    // depth behind this pixel: a shallow object over a deep floor barely
    // moves. A first lookup is offset by the depth behind this pixel; where it
    // lands on something shallower, the offset is cut down to that depth and
    // looked up again. Otherwise the floor around a shallow object reaches
    // across onto it, and the object shows twice: in place, and refracted
    // around itself.
    vec2 firstUV = refractedLookup(screenUV, N, opticalDepth);
    float firstDist = linearizeDepth(texture(depthSampler, firstUV).r, near, far);
    float firstOpticalDepth = max(firstDist - waterDist, 0.0);
    vec2 refractedUV = refractedLookup(screenUV, N, min(opticalDepth, firstOpticalDepth));

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

    // --- Shoreline foam on still water, white water where a river runs fast ---
    float running = step(1.0e-4, speed);
    float shoreFoam = (1.0 - smoothstep(0.0, 0.3, opticalDepth)) * (1.0 - running);
    float whiteWater = smoothstep(3.0, 7.0, speed) * 0.4 * whiteNoise;
    float foamFactor = max(max(shoreFoam, whiteWater), sheetFoam);
    color = mix(color, vec3(0.9, 0.95, 1.0), foamFactor * 0.6);

    // Scene radiance, like `colorSampler`: the water is drawn into the HDR
    // scene target and resolved with everything around it.
    outColor = vec4(color, 1.0);
}
