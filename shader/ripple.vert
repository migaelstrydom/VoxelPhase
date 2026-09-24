#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "swell.glsl"
#include "ripple.glsl"

// The fine surface of an awake ripple tile: a static 65 x 65 grid over the
// 8 m tile, displaced by the body's level, its swell and the tile's ripples.
// The fragment shader masks it to the body's columns, and adds the swell's
// slope to the ripples' for the normal.
//
// Along an edge beside coarse water (a sealed edge) the fine surface must
// meet the coarse one exactly: its ripples come to rest there through the
// apron, and its swell runs straight between column corners, as the coarse
// quads' edges do.
layout(location = 0) in vec2 inLocal;

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (level, swell amplitude, swell phase, clock)
    vec4 tile;   // (origin x, origin z, layer, sealed edges: +x, -x, +z, -z)
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;
layout(location = 2) flat out int fragLayer;
layout(location = 3) flat out vec2 fragTileOrigin;
layout(location = 4) out vec2 fragFlow;
layout(location = 5) out float fragAlong;
layout(location = 6) flat out vec2 fragWetRange;
layout(location = 7) out vec4 fragSwell;

// Steepest a ripple may tilt the normal: past this the surface is noise.
const float MAX_RIPPLE_SLOPE = 1.0;

// Water depth at a column corner; none where no wet column touches it.
float cornerDepth(int layer, ivec2 corner, float level) {
    float floor_ = rippleCornerFloor(layer, corner);
    return isnan(floor_) ? 0.0 : level - floor_;
}

// The swell at a column corner, as the coarse surface has it there.
vec3 cornerSwell(int layer, ivec2 corner, float level) {
    vec2 xz = pc.tile.xy + vec2(corner) * RIPPLE_COLUMN;
    return swellAt(xz, pc.body.w, pc.body.y, pc.body.z, cornerDepth(layer, corner, level));
}

// The swell straight between the corners either side, along a sealed edge
// at corner index `fixedIndex` of axis `axis` (0: x fixed, 1: z fixed).
vec3 edgeSwell(int layer, vec2 local, int axis, int fixedIndex, float level) {
    float along = (axis == 0 ? local.y : local.x) / RIPPLE_COLUMN;
    int a = clamp(int(floor(along)), 0, RIPPLE_COLUMNS - 1);
    float t = along - float(a);
    ivec2 c0 = axis == 0 ? ivec2(fixedIndex, a) : ivec2(a, fixedIndex);
    ivec2 c1 = axis == 0 ? ivec2(fixedIndex, a + 1) : ivec2(a + 1, fixedIndex);
    return mix(cornerSwell(layer, c0, level), cornerSwell(layer, c1, level), t);
}

void main() {
    int layer = int(pc.tile.z);
    int sealed = int(pc.tile.w);
    vec2 origin = pc.tile.xy;
    vec2 xz = origin + inLocal;
    float level = pc.body.x;
    float extent = float(RIPPLE_COLUMNS) * RIPPLE_COLUMN;
    float onEdge = 1.0e-3;

    // Depth between the corners, as the coarse surface fades its swell.
    vec2 f = inLocal / RIPPLE_COLUMN;
    ivec2 c = clamp(ivec2(floor(f)), ivec2(0), ivec2(RIPPLE_COLUMNS - 1));
    vec2 t = f - vec2(c);
    float depth = mix(
        mix(cornerDepth(layer, c, level), cornerDepth(layer, c + ivec2(1, 0), level), t.x),
        mix(cornerDepth(layer, c + ivec2(0, 1), level), cornerDepth(layer, c + ivec2(1, 1), level), t.x),
        t.y);

    vec3 swell;
    if ((sealed & 1) != 0 && inLocal.x > extent - onEdge) {
        swell = edgeSwell(layer, inLocal, 0, RIPPLE_COLUMNS, level);
    } else if ((sealed & 2) != 0 && inLocal.x < onEdge) {
        swell = edgeSwell(layer, inLocal, 0, 0, level);
    } else if ((sealed & 4) != 0 && inLocal.y > extent - onEdge) {
        swell = edgeSwell(layer, inLocal, 1, RIPPLE_COLUMNS, level);
    } else if ((sealed & 8) != 0 && inLocal.y < onEdge) {
        swell = edgeSwell(layer, inLocal, 1, 0, level);
    } else {
        swell = swellAt(xz, pc.body.w, pc.body.y, pc.body.z, depth);
    }

    float ripple = rippleHeightAt(layer, inLocal);
    vec2 rippleSlope = rippleGradientAt(layer, inLocal);
    float steepness = length(rippleSlope);
    if (steepness > MAX_RIPPLE_SLOPE) {
        rippleSlope *= MAX_RIPPLE_SLOPE / steepness;
    }

    vec3 position = vec3(xz.x, level + swell.x + ripple, xz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = normalize(vec3(-rippleSlope.x, 1.0, -rippleSlope.y));
    fragSwell = vec4(pc.body.y * swellShoreFade(depth), pc.body.z, pc.body.w, 0.0);
    fragWorldPos = position;
    fragLayer = layer;
    fragTileOrigin = origin;
    fragFlow = vec2(0.0);
    fragAlong = 0.0;
    fragWetRange = vec2(-1.0e9, 1.0e9);
}
