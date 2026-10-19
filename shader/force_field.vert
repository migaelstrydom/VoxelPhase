#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

// A force-field column: the translucent field a suspension stands on, from
// its pod down to the ground. The mesh is rebuilt every frame by
// `force_field::column`, which packs everything the look needs into the
// ordinary vertex format; see `force_field.frag` for what each channel means.

#include "scene.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec3 inNormal;
layout(location = 4) in float inAo;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec2 outTexCoord;
layout(location = 2) out vec3 outWorldPos;
layout(location = 3) out vec3 outNormal;
layout(location = 4) out float outStrain;

// The geometry pipelines' push block; only the vertex stage's members.
layout(push_constant) uniform PushConstants {
    mat4 model;
    layout(offset = 96) vec4 clipPlane;
} push;

void main() {
    vec4 worldPos = push.model * vec4(inPosition, 1.0);
    gl_Position = scene.proj * scene.view * worldPos;
    // Where the water divides a blended draw that crosses its surface.
    gl_ClipDistance[0] = dot(push.clipPlane, worldPos);
    outColor = inColor;
    outTexCoord = inTexCoord;
    outWorldPos = worldPos.xyz;
    outNormal = mat3(push.model) * inNormal;
    outStrain = inAo;
}
