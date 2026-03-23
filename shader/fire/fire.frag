#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// Raymarching fragment shader for volumetric fire rendering.
//
// Marches through the fire volume's 3D texture, accumulating emissive fire
// and absorptive smoke using front-to-back compositing.

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    mat4 volume_to_world;
    vec4 camera_pos;
} pc;

layout(set = 0, binding = 0) uniform sampler3D volumeTex;
layout(set = 0, binding = 1) uniform sampler2D depthSampler;

layout(location = 0) in vec3 fragWorldPos;

layout(location = 0) out vec4 outColor;

const int MAX_STEPS = 48;
const float FIRE_OPACITY = 2.5;
const float SMOKE_OPACITY = 1.5;

// Inverse of volume_to_world: maps world position to [0,1]^3 volume UVW.
// Since volume_to_world is a simple scale+translate (axis-aligned box),
// we compute the inverse analytically from the matrix columns.
vec3 worldToVolume(vec3 worldPos) {
    // Extract translation (column 3) and scale (diagonal) from volume_to_world
    vec3 origin = pc.volume_to_world[3].xyz;
    vec3 scale = vec3(
        length(pc.volume_to_world[0].xyz),
        length(pc.volume_to_world[1].xyz),
        length(pc.volume_to_world[2].xyz)
    );
    return (worldPos - origin) / scale;
}

// Blackbody-inspired color ramp for fire temperature.
vec3 blackbodyRamp(float t) {
    // Below ignition: invisible
    if (t < 0.1) return vec3(0.0);

    // Remap [0.1, 1.0] → [0, 1]
    float x = clamp((t - 0.1) / 0.9, 0.0, 1.0);

    // Deep red → orange → yellow → white-yellow
    vec3 c1 = vec3(0.5, 0.0, 0.0);    // dark red
    vec3 c2 = vec3(1.0, 0.3, 0.0);    // orange
    vec3 c3 = vec3(1.0, 0.7, 0.1);    // yellow-orange
    vec3 c4 = vec3(1.0, 0.95, 0.6);   // hot white-yellow

    if (x < 0.33) return mix(c1, c2, x / 0.33);
    if (x < 0.66) return mix(c2, c3, (x - 0.33) / 0.33);
    return mix(c3, c4, (x - 0.66) / 0.34);
}

// Ray-AABB intersection for the unit cube [0,1]^3 in volume space.
// Returns (tNear, tFar). If tNear > tFar, no intersection.
vec2 intersectBox(vec3 rayOrigin, vec3 rayDir) {
    vec3 invDir = 1.0 / rayDir;
    vec3 t0 = (vec3(0.0) - rayOrigin) * invDir;
    vec3 t1 = (vec3(1.0) - rayOrigin) * invDir;

    vec3 tmin = min(t0, t1);
    vec3 tmax = max(t0, t1);

    float tNear = max(max(tmin.x, tmin.y), tmin.z);
    float tFar = min(min(tmax.x, tmax.y), tmax.z);

    return vec2(tNear, tFar);
}

// Linearize a [0,1] depth buffer value to view-space distance.
float linearizeDepth(float d) {
    // Extract near/far from projection matrix
    float A = pc.proj[2][2];
    float B = pc.proj[3][2];
    return B / (d + A);
}

void main() {
    vec3 cameraWorld = pc.camera_pos.xyz;

    // Transform ray to volume space
    vec3 rayOriginVol = worldToVolume(cameraWorld);
    vec3 rayDirWorld = normalize(fragWorldPos - cameraWorld);

    // Scale direction to volume space (axis-aligned, so just divide by scale)
    vec3 scale = vec3(
        length(pc.volume_to_world[0].xyz),
        length(pc.volume_to_world[1].xyz),
        length(pc.volume_to_world[2].xyz)
    );
    vec3 rayDirVol = normalize(rayDirWorld / scale);

    // Intersect ray with unit cube in volume space
    vec2 tRange = intersectBox(rayOriginVol, rayDirVol);
    if (tRange.x > tRange.y) discard;

    // Clamp near to 0 (camera may be inside volume)
    tRange.x = max(tRange.x, 0.0);

    float stepSize = (tRange.y - tRange.x) / float(MAX_STEPS);

    // Read scene depth for occlusion
    vec2 screenUV = gl_FragCoord.xy / vec2(textureSize(depthSampler, 0));
    float sceneDepthRaw = texture(depthSampler, screenUV).r;
    float sceneZ = linearizeDepth(sceneDepthRaw);
    float fragZ = linearizeDepth(gl_FragCoord.z);

    // Raymarch through the volume
    vec4 accum = vec4(0.0);
    float t = tRange.x + stepSize * 0.5;

    for (int i = 0; i < MAX_STEPS; i++) {
        vec3 sampleUVW = rayOriginVol + rayDirVol * t;

        // Bounds check (should be within box, but numerical safety)
        if (any(lessThan(sampleUVW, vec3(0.0))) || any(greaterThan(sampleUVW, vec3(1.0))))
            break;

        // Check depth occlusion: convert sample position to view-space depth
        vec4 sampleWorld = pc.volume_to_world * vec4(sampleUVW, 1.0);
        vec4 sampleClip = pc.proj * pc.view * sampleWorld;
        float sampleZ = linearizeDepth(sampleClip.z / sampleClip.w);
        if (sampleZ > sceneZ) break;

        vec4 field = texture(volumeTex, sampleUVW);
        float temperature = field.r;
        float smoke = field.b;

        // Fire emission
        vec3 fireColor = blackbodyRamp(temperature);
        float fireAlpha = smoothstep(0.05, 0.3, temperature) * FIRE_OPACITY * stepSize;

        // Smoke absorption
        float smokeAlpha = smoke * SMOKE_OPACITY * stepSize;
        vec3 smokeColor = vec3(0.15, 0.12, 0.1);

        // Front-to-back compositing
        // Fire is emissive (adds light), smoke is absorptive (blocks light)
        float combinedAlpha = fireAlpha + smokeAlpha;
        vec3 combinedColor = fireColor * fireAlpha + smokeColor * smokeAlpha;

        accum.rgb += (1.0 - accum.a) * combinedColor;
        accum.a += (1.0 - accum.a) * combinedAlpha;

        if (accum.a > 0.98) break;

        t += stepSize;
    }

    // Output with premultiplied alpha
    outColor = vec4(accum.rgb, accum.a);
}
