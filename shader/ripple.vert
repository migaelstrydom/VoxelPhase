#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "swell.glsl"
#include "ripple.glsl"

// The fine surface of an awake ripple tile: a static 65 x 65 grid over the
// 8 m tile, displaced by the body's level, its swell and the tile's ripples.
// The fragment shader masks it to the body's columns.
layout(location = 0) in vec2 inLocal;

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (level, swell amplitude, swell phase, clock)
    vec4 tile;   // (origin x, origin z, layer, unused)
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;
layout(location = 2) flat out int fragLayer;
layout(location = 3) flat out vec2 fragTileOrigin;
layout(location = 4) out vec2 fragFlow;
layout(location = 5) out float fragAlong;
layout(location = 6) flat out vec2 fragWetRange;

void main() {
    int layer = int(pc.tile.z);
    vec2 origin = pc.tile.xy;
    vec2 xz = origin + inLocal;
    float level = pc.body.x;

    float columnFloor = rippleFloorAt(layer, inLocal);
    float depth = isnan(columnFloor) ? 0.0 : level - columnFloor;
    vec3 swell = swellAt(xz, pc.body.w, pc.body.y, pc.body.z, depth);
    float ripple = rippleHeightAt(layer, inLocal);
    vec2 slope = swell.yz + rippleGradientAt(layer, inLocal);

    vec3 position = vec3(xz.x, level + swell.x + ripple, xz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = normalize(vec3(-slope.x, 1.0, -slope.y));
    fragWorldPos = position;
    fragLayer = layer;
    fragTileOrigin = origin;
    fragFlow = vec2(0.0);
    fragAlong = 0.0;
    fragWetRange = vec2(-1.0e9, 1.0e9);
}
