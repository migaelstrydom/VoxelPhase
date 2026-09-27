#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

// The reflection probe capture: triangle.vert, seen from a probe face.

#include "probe_face.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inNormal;
layout(location = 4) in float inAo;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec2 outTexCoord;
layout(location = 2) out vec3 outWorldPos;
layout(location = 3) out vec3 outNormal;
layout(location = 4) out float outAo;

layout(push_constant) uniform PushConstants {
    mat4 model;
} push;

void main() {
    vec4 worldPos = push.model * vec4(inPosition, 1.0);
    gl_Position = probe_face.view_proj * worldPos;
    outColor = inColor;
    outTexCoord = inTexCoord;
    outWorldPos = worldPos.xyz;
    outNormal = mat3(push.model) * inNormal;
    outAo = inAo;
}
