// Turf: how a lawn is painted.
//
// A lawn meshed from voxels is one colour from edge to edge, and fades into
// the ground beside it across a whole voxel, because a vertex carries one
// material's colour and the triangle blends them. Painted here instead:
//
//   pile at the vertex ──▶ cover: where the turf ends, a ragged line drawn
//                          through the blend at fragment scale
//   colour at the vertex ─▶ the bare ground beside the turf, unmixed from it
//   world position ──────▶ drift: broad patches of warmer and cooler green
//                          blades: light and dark at the scale of a tuft,
//                          faded out before a pixel is wider than one
//                          lip: turf in its own shade just inside its edge,
//                          and the ground in the turf's shade just outside
//
// What the ground *is* stays the voxel material. This changes only how grass
// is drawn (docs/VISUAL_DIRECTION.md §2).

#ifndef TURF_GLSL
#define TURF_GLSL

/// Grass's own colour: `VoxelMaterial::Grass.color()`, held to it by a test
/// in `src/terrain/voxel.rs`. Needed to tell the turf from the ground it is
/// blended with.
const vec3 TURF_ALBEDO = vec3(0.19, 0.30, 0.07);

/// How far the turf's edge wanders either side of the blend's midpoint, in
/// pile, and how many wanderings to the metre.
const float TURF_EDGE_RAGGED = 0.7;
const float TURF_EDGE_FREQUENCY = 1.6;

/// How deep into the turf its lip's shade reaches and how far out over the
/// ground, in pile, and how dark each is.
const float TURF_LIP_INSIDE = 0.5;
const float TURF_LIP_OUTSIDE = 0.12;
const float TURF_LIP_SHADE = 0.6;
const float TURF_LIP_CAST = 0.3;

/// The tint a lawn drifts between: sun-dried, yellow and pale; and lush,
/// deeper and bluer. Their mean is near white, so a lawn keeps its level.
const vec3 TURF_DRY = vec3(1.6, 1.25, 0.75);
const vec3 TURF_LUSH = vec3(0.6, 0.85, 1.0);

/// Patches of drift to the metre, at its broadest.
const float TURF_DRIFT_FREQUENCY = 0.07;

/// Tufts to the metre across the way the blades lie and along it, and how
/// far a tuft lightens or darkens the lawn.
const vec2 TURF_BLADE_FREQUENCY = vec2(26.0, 9.0);
const float TURF_BLADE_CONTRAST = 0.55;

float turfHash(vec2 p) {
    vec3 q = fract(vec3(p.xyx) * vec3(0.1031, 0.1030, 0.0973));
    q += dot(q, q.yzx + 33.33);
    return fract((q.x + q.y) * q.z);
}

/// Value noise, 0 to 1.
float turfNoise(vec2 p) {
    vec2 cell = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(
        mix(turfHash(cell), turfHash(cell + vec2(1.0, 0.0)), u.x),
        mix(turfHash(cell + vec2(0.0, 1.0)), turfHash(cell + vec2(1.0, 1.0)), u.x),
        u.y);
}

/// Where a lawn is dry and where lush, 0 to 1, in patches of several sizes.
float turfDrift(vec2 ground) {
    vec2 p = ground * TURF_DRIFT_FREQUENCY;
    float drift = turfNoise(p) * 0.5
                + turfNoise(p * 2.7 + 17.0) * 0.3
                + turfNoise(p * 7.1 + 41.0) * 0.2;
    return smoothstep(0.3, 0.7, drift);
}

/// How a lawn's tufts lighten and darken it, about 1. Two families of
/// streaks lying across one another, each faded out as a pixel grows to the
/// width of a streak: past that they would only shimmer.
float turfBlades(vec2 ground) {
    float pixel = length(fwidth(ground));
    float shade = 0.0;
    for (int family = 0; family < 2; family++) {
        // Turned, so the two families' streaks cross.
        float turn = family == 0 ? 0.6 : 2.1;
        mat2 lay = mat2(cos(turn), -sin(turn), sin(turn), cos(turn));
        vec2 frequency = TURF_BLADE_FREQUENCY * (family == 0 ? 1.0 : 0.37);
        float sharp = 1.0 - smoothstep(0.35, 1.0, pixel * frequency.x);
        shade += (turfNoise(lay * ground * frequency) - 0.5) * sharp;
    }
    return 1.0 + TURF_BLADE_CONTRAST * shade;
}

/// Ground as it is drawn: its colour before the surface field's wash, and
/// how much of it is standing turf.
struct TurfGround {
    vec3 colour;
    float pile;
};

/// Ground whose vertices blend `colour` and `pile` between turf and whatever
/// lies beside it, at `world_pos`.
TurfGround turfGround(vec3 colour, float pile, vec3 world_pos) {
    TurfGround ground;
    if (pile <= 0.0) {
        ground.colour = colour;
        ground.pile = 0.0;
        return ground;
    }
    vec2 at = world_pos.xz;

    // The edge: nowhere on a lawn, and a ragged line through the blend.
    float ragged = (turfNoise(at * TURF_EDGE_FREQUENCY) - 0.5)
                 + (turfNoise(at * TURF_EDGE_FREQUENCY * 4.3) - 0.5) * 0.4;
    // Most ragged midway through the blend, and none at either end of it,
    // so a lawn stays whole and bare ground bare.
    float edge = pile + TURF_EDGE_RAGGED * ragged * 4.0 * pile * (1.0 - pile);
    float soft = max(fwidth(edge), 0.02);
    float cover = smoothstep(0.5 - soft, 0.5 + soft, edge);

    // The ground beside the turf: what is left of the blend without it.
    vec3 bare = pile < 0.98
        ? clamp((colour - pile * TURF_ALBEDO) / (1.0 - pile), 0.0, 1.0)
        : TURF_ALBEDO;
    float shaded = smoothstep(0.5 - TURF_LIP_OUTSIDE, 0.5, edge);
    bare *= 1.0 - TURF_LIP_CAST * shaded;

    vec3 lawn = TURF_ALBEDO * mix(TURF_DRY, TURF_LUSH, turfDrift(at)) * turfBlades(at);
    float lip = 1.0 - smoothstep(0.5, 0.5 + TURF_LIP_INSIDE, edge);
    lip *= lip;
    lawn *= 1.0 - TURF_LIP_SHADE * lip;

    ground.colour = mix(bare, lawn, cover);
    ground.pile = cover;
    return ground;
}

#endif // TURF_GLSL
