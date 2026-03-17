#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec3 fragWorldPos;

layout(location = 0) out vec4 outColor;

void main() {
    // Basic directional lighting
    vec3 lightDir = normalize(vec3(0.3, 1.0, 0.5));
    float ndotl = max(dot(normalize(fragNormal), lightDir), 0.0);

    // Water color with simple lighting
    vec3 baseColor = vec3(0.1, 0.3, 0.6);
    vec3 lit = baseColor * (0.4 + 0.6 * ndotl);

    outColor = vec4(lit, 0.6);
}
