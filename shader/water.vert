#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// Vertex inputs
layout(location = 0) in vec3 inPosition;  // World-space position
layout(location = 1) in vec3 inNormal;    // Surface normal

// Push constants for matrices
layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
} pc;

// Outputs to fragment shader
layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;

void main() {
    gl_Position = pc.proj * pc.view * vec4(inPosition, 1.0);
    fragNormal = inNormal;
    fragWorldPos = inPosition;
}
