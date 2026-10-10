#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "swell.glsl"

// A river surface: each vertex at the water's own height, carrying its depth
// (negative over a bank) and its flow. The fragment shader cuts the surface
// where the depth falls below the shore depth, so a shoreline is a contour.
//
// Water standing near sea level takes up the sea's swell, all of it at sea
// level and none `band` above or below it. Mirrors `swell_share` and the
// query's surface (src/water/waves.rs, src/water/query.rs).
layout(location = 0) in vec2 inXz;
layout(location = 1) in vec2 inWater;  // (surface, depth)
layout(location = 2) in vec2 inFlow;   // depth-averaged velocity, m/s

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (shore depth, sea swell amplitude, swell phase, clock)
    vec4 tile;   // (sea level, band, unused, unused)
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;
layout(location = 2) flat out int fragLayer;
layout(location = 3) flat out vec2 fragTileOrigin;
layout(location = 4) out vec2 fragFlow;
layout(location = 5) out float fragWetDepth;
layout(location = 6) flat out vec2 fragWetRange;
layout(location = 7) out vec4 fragSwell;

void main() {
    float surface = inWater.x;
    float depth = inWater.y;
    float band = pc.tile.y;
    float share = band > 0.0 ? clamp(1.0 - abs(surface - pc.tile.x) / band, 0.0, 1.0) : 0.0;
    float amplitude = pc.body.y * share;
    vec3 swell = swellAt(inXz, pc.body.w, amplitude, pc.body.z, depth);
    vec3 position = vec3(inXz.x, surface + swell.x, inXz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = vec3(0.0, 1.0, 0.0);
    fragSwell = vec4(amplitude * swellShoreFade(depth), pc.body.z, pc.body.w, 0.0);
    fragWorldPos = position;
    fragLayer = -1;
    fragTileOrigin = vec2(0.0);
    fragFlow = inFlow;
    fragWetDepth = depth;
    fragWetRange = vec2(pc.body.x, 1.0e9);
}
