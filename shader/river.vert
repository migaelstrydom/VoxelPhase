#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "swell.glsl"
#include "ripple.glsl"

// A river surface: each vertex at the water's own height, carrying its depth
// (negative over a bank) and its flow. Falling water is a body with a top
// face and an under face, each vertex carrying its face's normal; the surface
// faces up. The fragment shader cuts the surface
// where the depth falls below the shore depth, so a shoreline is a contour.
//
// Where its chunk's ripple tile is awake, each vertex is lifted by the tile's
// ripples, and the fragment shader takes their normal per pixel.
//
// Water standing near sea level takes up the sea's swell, all of it at sea
// level and none `band` above or below it. Mirrors `swell_share` and the
// query's surface (src/water/waves.rs, src/water/query.rs).
layout(location = 0) in vec2 inXz;
layout(location = 1) in vec2 inWater;  // (surface, depth)
layout(location = 2) in vec2 inFlow;   // depth-averaged velocity, m/s
layout(location = 3) in vec2 inFace;   // a falling sheet's facing; zero on the surface
layout(location = 4) in vec4 inSheet;  // a falling sheet's (drop below its lip, whole fall, side, along)
layout(location = 5) in vec3 inNormal; // falling water's face normal; up on the surface
layout(location = 6) in float inFray;  // 1 at an end of a fall, 0 where it joins the sheet beside it
layout(location = 7) in float inReach; // how far past its wet cell's edge a skirt lies, m; 0 elsewhere

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (ripple layer or -1, sea swell amplitude, swell phase, clock)
    vec4 tile;   // (sea level, band, ripple tile origin x, z)
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;
layout(location = 2) flat out int fragLayer;
layout(location = 3) flat out vec2 fragTileOrigin;
layout(location = 4) out vec2 fragFlow;
layout(location = 5) out float fragWetDepth;
layout(location = 6) flat out vec2 fragWetRange;
layout(location = 7) out vec4 fragSwell;
layout(location = 8) flat out int fragMasked;
layout(location = 9) out vec4 fragSheet;
layout(location = 10) out float fragFray;
layout(location = 11) out float fragReach;

// Depth at which the surface is cut, m: the shoreline's contour. Thinner
// water would lie on its floor and fight it for depth.
const float SHORE_DEPTH = 0.005;
// How far each sheet reaches past the ends of its edge, m, to overlap the
// sheets beside it, so neighbours leaning out each their own way leave no
// gap between them; the fragment shader frays the ends of a fall out.
const float SHEET_OVERLAP = 0.2;

void main() {
    float surface = inWater.x;
    float depth = inWater.y;
    float band = pc.tile.y;
    float share = band > 0.0 ? clamp(1.0 - abs(surface - pc.tile.x) / band, 0.0, 1.0) : 0.0;
    float amplitude = pc.body.y * share;
    vec3 swell = swellAt(inXz, pc.body.w, amplitude, pc.body.z, depth);
    int layer = int(pc.body.x);
    vec2 origin = pc.tile.zw;
    float extent = float(RIPPLE_COLUMNS) * RIPPLE_COLUMN;
    // A skirt or lip reaches past its chunk's tile: it takes the ripples at
    // the tile's edge, where they come to rest.
    float ripple = layer >= 0 ? rippleHeightAt(layer, clamp(inXz - origin, vec2(0.0), vec2(extent))) : 0.0;
    vec2 edge = vec2(-inFace.y, inFace.x);
    vec2 xz = inXz + edge * inSheet.z * SHEET_OVERLAP;
    vec3 position = vec3(xz.x, surface + swell.x + ripple, xz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = inSheet.y > 0.0 ? inNormal : vec3(0.0, 1.0, 0.0);
    fragSwell = vec4(amplitude * swellShoreFade(depth), pc.body.z, pc.body.w, 0.0);
    fragWorldPos = position;
    fragLayer = layer;
    fragTileOrigin = origin;
    fragMasked = 0;
    fragSheet = vec4(inSheet.xy, inSheet.w + inSheet.z * SHEET_OVERLAP, inSheet.z);
    fragFray = inFray;
    fragReach = inReach;
    fragFlow = inFlow;
    fragWetDepth = depth;
    fragWetRange = vec2(SHORE_DEPTH, 1.0e9);
}
