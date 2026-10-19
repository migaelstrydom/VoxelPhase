// Caustics: sunlight focused by the waves onto the floor under the water.
//
// The surface's slopes bend the sun's rays, and where neighbouring rays cross
// they pile up into the bright, shifting net seen on any sunlit shallow floor.
// Drawn on the floor as the water composites it, where the floor's depth is
// already known, rather than by the terrain shader, which knows nothing of the
// water above it.
//
// The net is a drifting Voronoi pattern rather than traced from the surface's
// own slopes:
// tracing needs the surface's normal at the point each ray left it, which the
// floor's pixel does not have. It is laid on the floor in the sun's direction,
// so it slides across a slope as the sun's light does.

#ifndef CAUSTICS_GLSL
#define CAUSTICS_GLSL

/// Size of the net's cells on the floor, m.
const float CAUSTIC_CELL = 0.9;

/// Width of the net's lines, as a share of a cell.
const float CAUSTIC_LINE = 0.2;

/// How far the net's lines are bent off straight, in cells.
const float CAUSTIC_BEND = 0.22;

/// Ground a pixel covers, as a share of a cell, over which the net fades to
/// its average: its lines are a few hundredths of a cell wide, and drawn into
/// pixels wider than that they break up into glitter and hatching.
const vec2 CAUSTIC_RESOLVED = vec2(0.04, 0.12);

/// The net's average brightness, which a pixel too coarse to resolve it shows.
const float CAUSTIC_AVERAGE = 0.2;

/// How fast the net shifts.
const float CAUSTIC_RATE = 0.45;

/// Brightness of the net's lines at their brightest, as a share of the floor's
/// own light.
const float CAUSTIC_STRENGTH = 0.9;

/// Depth over which the net fades in from the waterline, m: the thinnest film
/// has too little curvature in it to focus anything.
const float CAUSTIC_FADE_IN = 0.15;

/// Depth at which the net has faded to 1/e, m: deeper, the rays have spread
/// and crossed so often that the light evens out.
const float CAUSTIC_REACH = 3.0;

/// Depth past which there is no net at all, m. Also what keeps the cost to
/// the shallows: deep water skips the work.
const float CAUSTIC_MAX_DEPTH = 6.0;

/// A pseudo-random point in the unit cell `cell`, drifting on a small orbit.
vec2 causticSite(vec2 cell, float time) {
    // An arithmetic hash: no transcendental per cell, which nine cells per
    // pixel would make the net's dearest part.
    vec3 q = fract(vec3(cell.xyx) * vec3(0.1031, 0.1030, 0.0973));
    q += dot(q, q.yzx + 33.33);
    vec2 seed = fract((q.xx + q.yz) * q.zy);
    return 0.5 + 0.38 * sin(time + 6.2831853 * seed);
}

/// One layer of the net at `p` (in cells), 0 to 1: bright along the edges
/// between drifting cells, where light bent from two neighbouring wave faces
/// crosses, and dark in the cells' middles, which those faces spread thin.
float causticLayer(vec2 p, float time) {
    vec2 cell = floor(p);
    vec2 local = fract(p);
    float nearest = 8.0;
    float second = 8.0;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            vec2 offset = vec2(float(x), float(y));
            vec2 site = offset + causticSite(cell + offset, time);
            float d = length(site - local);
            second = min(max(d, nearest), second);
            nearest = min(d, nearest);
        }
    }
    return 1.0 - smoothstep(0.0, CAUSTIC_LINE, second - nearest);
}

/// The net at `p` (in cells). Bent by a drifting warp, so the cells' straight
/// edges curve as light lines do and the net shifts rather than slides. A
/// second, finer layer looked a little richer and cost as much again; the
/// water pass is the frame's dearest already.
float causticNet(vec2 p, float time) {
    p += CAUSTIC_BEND * sin(p.yx * 1.9 + vec2(time * 0.7, -time * 0.6));
    return causticLayer(p, time);
}

/// How much brighter the floor at `floor_pos`, `depth` metres under the
/// water, is lit by the net: a factor of 1 and up, for the floor's colour.
/// The sun direction `sun_dir` points towards the sun; `footprint` is the
/// ground one pixel covers, m.
float causticLight(vec3 floor_pos, float depth, vec3 sun_dir, float footprint, float time) {
    if (depth <= 0.0 || depth > CAUSTIC_MAX_DEPTH || sun_dir.y <= 0.0) {
        return 1.0;
    }
    // Where the sun's ray through this floor point left the surface.
    vec2 entry = floor_pos.xz + sun_dir.xz / sun_dir.y * depth;
    vec2 p = entry / CAUSTIC_CELL;
    float resolved = 1.0 - smoothstep(CAUSTIC_RESOLVED.x, CAUSTIC_RESOLVED.y, footprint / CAUSTIC_CELL);
    float net = resolved > 0.0
        ? mix(CAUSTIC_AVERAGE, causticNet(p, time * CAUSTIC_RATE), resolved)
        : CAUSTIC_AVERAGE;
    float fade = smoothstep(0.0, CAUSTIC_FADE_IN, depth) * exp(-depth / CAUSTIC_REACH);
    return 1.0 + CAUSTIC_STRENGTH * net * fade * sun_dir.y;
}

#endif // CAUSTICS_GLSL
