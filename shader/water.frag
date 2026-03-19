#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragWorldPos;

// Depth buffer from the opaque geometry pass (subpass input attachment)
layout(input_attachment_index = 0, set = 0, binding = 0) uniform subpassInput depthInput;

layout(push_constant) uniform FragPushConstants {
    layout(offset = 128) vec4 cameraPos;
    layout(offset = 144) vec4 sunDir;
    layout(offset = 160) vec4 projParams;  // (near, far, unused, unused)
} fpc;

layout(location = 0) out vec4 outColor;

/// Linearize a [0,1] depth buffer value to a view-space distance.
float linearizeDepth(float d, float near, float far) {
    return near * far / (far - d * (far - near));
}

void main() {
    vec3 N = normalize(fragNormal);
    vec3 V = normalize(fpc.cameraPos.xyz - fragWorldPos);
    vec3 L = normalize(fpc.sunDir.xyz);

    float near = fpc.projParams.x;
    float far = fpc.projParams.y;

    // --- Volumetric depth from depth buffer ---
    // Read the terrain depth behind this water fragment.
    float terrainDepthRaw = subpassLoad(depthInput).r;
    float waterDepthRaw = gl_FragCoord.z;

    // Linearize both to view-space distances.
    float terrainDist = linearizeDepth(terrainDepthRaw, near, far);
    float waterDist = linearizeDepth(waterDepthRaw, near, far);

    // Optical path length through the water volume.
    float opticalDepth = max(terrainDist - waterDist, 0.0);

    // Depth factor: how opaque/colored the water appears.
    // Saturates around 5 meters of optical path.
    float depthFactor = 1.0 - exp(-opticalDepth * 0.5);

    // --- Water color based on optical depth ---
    vec3 shallowColor = vec3(0.2, 0.55, 0.5);   // light turquoise
    vec3 deepColor    = vec3(0.02, 0.12, 0.22);  // dark navy
    vec3 waterColor = mix(shallowColor, deepColor, depthFactor);

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

    // Blend water body color with reflection via Fresnel
    vec3 color = mix(waterColor, reflectionColor, fresnel);

    // --- Diffuse lighting ---
    float NdotL = max(dot(N, L), 0.0);
    color *= (0.5 + 0.5 * NdotL);

    // --- Specular highlight (Blinn-Phong) ---
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);
    float spec = pow(NdotH, 256.0);
    vec3 sunColor = vec3(1.0, 0.95, 0.8);
    color += sunColor * spec * 0.8;

    // --- Depth-based opacity ---
    // Shallow edges are transparent, deep water is nearly opaque.
    float alpha = mix(0.15, 0.9, depthFactor);

    outColor = vec4(color, alpha);
}
