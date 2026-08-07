#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(location = 0) in vec2 fragClipPos;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PushConstants {
    mat4 inv_view;      // Inverse view matrix (rotation part transforms view->world)
    vec4 sun_direction; // xyz = normalized sun direction, w = time
    vec4 tan_fov;       // x = tan(fov/2)*aspect, y = tan(fov/2)
} pc;

// Constants
const float PI = 3.14159265359;
const vec3 RAYLEIGH_COEFF = vec3(5.8e-6, 13.5e-6, 33.1e-6);

// Sun angular radius in radians (about 0.53 degrees, or 0.00925 radians)
const float SUN_ANGULAR_RADIUS = 0.00925;

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

// Rayleigh phase function
float rayleighPhase(float cosTheta) {
    return (3.0 / (16.0 * PI)) * (1.0 + cosTheta * cosTheta);
}

// Henyey-Greenstein phase function for Mie scattering
float miePhase(float cosTheta, float g) {
    float g2 = g * g;
    float denom = 1.0 + g2 - 2.0 * g * cosTheta;
    return (1.0 / (4.0 * PI)) * (1.0 - g2) / (denom * sqrt(denom));
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

// Simplified atmospheric scattering
vec3 atmosphericScattering(vec3 rayDir, vec3 sunDir) {
    float sunDot = dot(rayDir, sunDir);
    float sunHeight = sunDir.y;

    float horizonFactor = 1.0 - abs(rayDir.y);
    horizonFactor = pow(horizonFactor, 4.0);

    vec3 zenithColor = vec3(0.3, 0.6, 1.0);
    vec3 horizonColor = vec3(0.6, 0.8, 0.95);

    float sunInfluence = max(0.0, sunHeight);

    vec3 sunsetColor = vec3(1.0, 0.6, 0.3);
    float sunsetFactor = smoothstep(0.0, 0.2, sunHeight) * (1.0 - smoothstep(0.2, 0.5, sunHeight));
    sunsetFactor *= horizonFactor;

    float rayleigh = rayleighPhase(sunDot);
    vec3 rayleighColor = RAYLEIGH_COEFF * rayleigh * 3000.0;

    float mie = miePhase(sunDot, 0.76);
    vec3 mieColor = vec3(1.0, 0.95, 0.85) * mie * 0.03 * max(0.0, sunHeight + 0.1);

    vec3 skyColor = mix(zenithColor, horizonColor, horizonFactor);
    skyColor = mix(skyColor, sunsetColor, sunsetFactor * 0.5);
    skyColor += rayleighColor * sunInfluence;
    skyColor += mieColor;
    skyColor *= 1.0 + 0.2 * sunInfluence;

    float belowHorizon = smoothstep(0.0, -0.1, rayDir.y);
    skyColor = mix(skyColor, vec3(0.15, 0.2, 0.25), belowHorizon);

    return skyColor;
}

// Sun disk with glow - using angular distance for correct shape
vec3 renderSun(vec3 rayDir, vec3 sunDir) {
    // Calculate angular distance to sun (in radians)
    float cosAngle = clamp(dot(rayDir, sunDir), -1.0, 1.0);
    float angle = acos(cosAngle);

    // Sun disk using angular distance (sun is about 0.5 degrees radius)
    float sunRadius = SUN_ANGULAR_RADIUS * 3.0; // Make it slightly larger for visibility
    float sunSoftEdge = SUN_ANGULAR_RADIUS * 0.5;
    float sunDisk = 1.0 - smoothstep(sunRadius - sunSoftEdge, sunRadius, angle);
    vec3 sunColor = vec3(1.0, 0.95, 0.85) * sunDisk * 5.0;

    // Inner glow
    float innerGlowRadius = sunRadius * 3.0;
    float innerGlow = 1.0 - smoothstep(sunRadius, innerGlowRadius, angle);
    sunColor += vec3(1.0, 0.8, 0.5) * innerGlow * 0.8;

    // Outer glow (corona)
    float outerGlowRadius = sunRadius * 10.0;
    float outerGlow = 1.0 - smoothstep(sunRadius, outerGlowRadius, angle);
    sunColor += vec3(1.0, 0.6, 0.3) * outerGlow * 0.3;

    // Soft atmospheric glow
    float atmosphericGlow = pow(max(0.0, cosAngle), 8.0);
    sunColor += vec3(1.0, 0.7, 0.4) * atmosphericGlow * 0.15 * max(0.0, sunDir.y + 0.2);

    return sunColor;
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

    return cloudColor * density;
}

// Horizon haze
vec3 horizonHaze(vec3 rayDir, vec3 sunDir) {
    float horizonFactor = 1.0 - abs(rayDir.y);
    horizonFactor = pow(horizonFactor, 12.0);

    vec3 hazeColor = vec3(0.85, 0.9, 0.95);

    float sunDot = max(0.0, dot(normalize(vec3(rayDir.x, 0.0, rayDir.z)),
                                normalize(vec3(sunDir.x, 0.0, sunDir.z))));
    hazeColor = mix(hazeColor, vec3(1.0, 0.95, 0.85), sunDot * 0.3);

    return hazeColor * horizonFactor * 0.15;
}

void main() {
    // Compute ray direction per-fragment (no interpolation artifacts)
    vec3 rayDir = computeRayDir(fragClipPos);
    vec3 sunDir = normalize(pc.sun_direction.xyz);
    float time = pc.sun_direction.w;
    vec3 color = atmosphericScattering(rayDir, sunDir);
    color += horizonHaze(rayDir, sunDir);
    color += renderSun(rayDir, sunDir);

    vec3 clouds = renderClouds(rayDir, sunDir, time);
    color = mix(color, color + clouds, min(1.0, length(clouds)));

    // Tone mapping
    color = color / (1.0 + color);

    // Gamma correction
    color = pow(color, vec3(1.0 / 2.2));

    outColor = vec4(color, 1.0);
}
