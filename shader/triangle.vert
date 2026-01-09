#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inNormal;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec2 outTexCoord;
layout(location = 2) out vec3 outWorldPos;
layout(location = 3) out vec3 outNormal;

// Per-frame scene data (updated once per frame)
layout(set = 0, binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
} ubo;

// Per-object data (updated per draw call via push constants)
layout(push_constant) uniform PushConstants {
    mat4 model;
} push;

void main() {
    vec4 worldPos = push.model * vec4(inPosition, 1.0);
    gl_Position = ubo.proj * ubo.view * worldPos;
    outColor = inColor;
    outTexCoord = inTexCoord;
    outWorldPos = worldPos.xyz;
    // Transform normal to world space
    // For proper normal transformation, we should use inverse transpose of model matrix
    // For now, using mat3(model) and normalizing in fragment shader (works for uniform scaling)
    outNormal = mat3(push.model) * inNormal;
}
