#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inTexCoord;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in vec3 inNormal;

layout(location = 0) out vec4 outColor;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

// For wireframe debugging overlay
layout(push_constant) uniform PushConstants {
    layout(offset = 64) vec4 colorOverride;
} push;

void main() {
    if (push.colorOverride.a > 0.0) {
        outColor = push.colorOverride;
        return;
    }
    // Use interpolated normal for smooth shading
    // Check if normal is non-zero before normalizing to avoid undefined behavior
    vec3 normal = length(inNormal) > 0.001 ? normalize(inNormal) : vec3(0.0, 1.0, 0.0);

    // Light direction: direction FROM surface TO light (sun coming from above and slightly offset)
    vec3 lightDir = normalize(vec3(-0.2, 1.0, -0.3));

    // Simple diffuse lighting with ambient
    float ambient = 0.3;  // Increased from 0.5 for brighter base lighting
    float diffuse = max(dot(normal, lightDir), 0.0);
    float lighting = ambient + (1.0 - ambient) * diffuse;

    // Sample texture and apply lighting
    vec4 texColor = texture(texSampler, inTexCoord);
    vec3 litColor = texColor.rgb * inColor.rgb * lighting;

    // Apply brightness boost
    float brightness = 1.0;  // Brightness multiplier
    litColor *= brightness;

    outColor = vec4(litColor, texColor.a * inColor.a);
}
