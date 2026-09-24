#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "swell.glsl"

// The coarse surface: one quad per column, its plan position, the floor
// beneath it and the share of the swell its geometry carries. The height is
// the body's level plus its swell, both pushed per draw, so a level change
// needs no new mesh. The swell's normal is left to the fragment shader.
layout(location = 0) in vec2 inXz;
layout(location = 1) in float inFloor;
layout(location = 2) in float inSwellShare;

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
layout(location = 7) out vec4 fragSwell;

void main() {
    float level = pc.body.x;
    float depth = level - inFloor;
    vec3 swell = swellAt(inXz, pc.body.w, pc.body.y, pc.body.z, depth);
    vec3 position = vec3(inXz.x, level + swell.x * inSwellShare, inXz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = vec3(0.0, 1.0, 0.0);
    fragSwell = vec4(pc.body.y * swellShoreFade(depth), pc.body.z, pc.body.w, 0.0);
    fragWorldPos = position;
    fragLayer = -1;
    fragTileOrigin = vec2(0.0);
    fragFlow = vec2(0.0);
    fragAlong = 0.0;
    fragWetRange = vec2(-1.0e9, 1.0e9);
}
