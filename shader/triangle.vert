#version 450

layout (location = 0) in vec4 inPosition;
layout (location = 1) in vec4 inColor;

// Uniform Buffer Object for transformations
layout(set = 0, binding = 0) uniform ModelMatrixUbo {
    mat4 model;
} ubo;

layout (location = 0) out vec4 fragColor;

void main() {
    // Using identity matrices for view and projection for now
    mat4 view = mat4(1.0);
    mat4 proj = mat4(1.0);

    gl_Position = proj * view * ubo.model * inPosition;
    fragColor = inColor;
}