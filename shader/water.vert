#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "swell.glsl"

// The coarse surface: one quad per column, its plan position and the floor
// beneath it. The height is the body's level plus its swell, both pushed per
// draw, so a level change needs no new mesh.
layout(location = 0) in vec2 inXz;
layout(location = 1) in float inFloor;

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (level, swell amplitude, swell phase, clock)
    vec4 tile;   // unused by the coarse surface
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;
layout(location = 2) flat out int fragLayer;
layout(location = 3) flat out vec2 fragTileOrigin;
layout(location = 4) out vec2 fragFlow;
layout(location = 5) out float fragAlong;
layout(location = 6) flat out vec2 fragWetRange;

void main() {
    float level = pc.body.x;
    vec3 swell = swellAt(inXz, pc.body.w, pc.body.y, pc.body.z, level - inFloor);
    vec3 position = vec3(inXz.x, level + swell.x, inXz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = normalize(vec3(-swell.y, 1.0, -swell.z));
    fragWorldPos = position;
    fragLayer = -1;
    fragTileOrigin = vec2(0.0);
    fragFlow = vec2(0.0);
    fragAlong = 0.0;
    fragWetRange = vec2(-1.0e9, 1.0e9);
}
