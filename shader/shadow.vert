// Depth-only vertex stage for the sun shadow map.
//
// Shares the `Vertex` format and the `mat4 model` push constant with
// triangle.vert, so the shadow pass can replay the same draw calls without the
// caller knowing it exists. Only the position is consumed; the remaining
// attributes are declared so the vertex input state matches the main pipeline.

#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "scene.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inNormal;

layout(push_constant) uniform PushConstants {
    mat4 model;
} push;

void main() {
    gl_Position = scene.light_view_proj * push.model * vec4(inPosition, 1.0);
}
