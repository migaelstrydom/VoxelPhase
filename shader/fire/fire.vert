#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// Fire volume bounding box vertex shader.
//
// Takes 8 vertices of the volume's world-space AABB and projects them.
// The fragment shader will reconstruct a ray and raymarch through the volume.

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    mat4 volume_to_world;
    vec4 camera_pos;
} pc;

// Volume-space corners of the unit cube [0,1]^3, expanded to a box via
// volume_to_world. We draw a 36-vertex indexed box (12 triangles).
layout(location = 0) in vec3 inPosition;

layout(location = 0) out vec3 fragWorldPos;

void main() {
    vec4 worldPos = pc.volume_to_world * vec4(inPosition, 1.0);
    fragWorldPos = worldPos.xyz;
    gl_Position = pc.proj * pc.view * worldPos;
}
