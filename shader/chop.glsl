// Chop: the wind waves on every body of water, over the swell.
//
// Real water carries thousands of waves at once, so no two or three of them
// dominate and line up into a lattice. The chop is drawn from tiles of a few
// hundred random waves per octave of wavelength, from 3.3 m down to 20 cm,
// each wave at its own deep-water speed, summed each frame on the CPU by an
// FFT (src/rendering/water/chop_tiles.rs) and read here. Each band's tile
// holds its slopes at an rms of one; the longer bands carry more, in
// proportion to their length, so the longest waves carry the shape and the
// shortest only break it up. The tiles wrap, each over a span of its own, so
// they never repeat together.
//
// How long the waves run depends on the water: wind waves grow over their
// fetch, so a pond only ripples, and under a roof there is no wind at all.
// The wind map (wind.glsl) gives, per metre, the peak wavelength of the wind
// sea; a band much longer than it is not drawn. Below a floor the waves do
// not shorten further but calm, which reads better at this game's scale.
// Running water roughens of itself, by its speed.
//
// The GPU alone draws them: they are a few centimetres high, which buoyancy
// does not feel.

#ifndef CHOP_GLSL
#define CHOP_GLSL

const int CHOP_BANDS = 4;
const int CHOP_TILE_TEXELS = 128;

layout(std430, set = 1, binding = 2) readonly buffer ChopTiles {
    // Per band: its tile's texels per metre, its longest wavelength (m), two
    // unused.
    vec4 chopBands[CHOP_BANDS];
    // Each band's tile in turn, row-major (x along a row, z down the rows):
    // the slope along x and along z.
    vec2 chopTexels[];
};

// The longest band's steepness: its rms slope where the water raises it.
// Under it the shorter bands sum to the sea's under a gentle breeze.
const float CHOP_STEEPNESS = 0.12;

// Size of the gusts' patches, m: where the wind drops the chop lies flat and
// glassy, where it gusts it roughens, so a sea seen from afar is not one
// even texture.
const float CHOP_GUST_SIZE = 18.0;

// The chop's roughness in a lull and in a gust, as shares of the standard.
const vec2 CHOP_GUST_RANGE = vec2(0.35, 1.4);

// How fast the gusts' patches drift downwind, in patches per second: 3 m/s.
const vec2 CHOP_GUST_DRIFT = vec2(0.92, 0.39) * (3.0 / CHOP_GUST_SIZE);

// Shortest band drawn, by its longest wave, as a share of the longest wave
// the water raises.
const float CHOP_SHORTEST_SHARE = 0.15;

// The shortest peak wavelength the chop is drawn at, m. Water whose wind sea
// peaks shorter is drawn at this, calmer in proportion.
const float CHOP_PEAK_FLOOR = 0.5;

// Peak wavelength of the waves running water raises of itself, per m/s of
// current, m, and the most it raises.
const float CHOP_FLOW_PEAK = 0.8;
const float CHOP_FLOW_PEAK_MAX = 1.6;

// The peak wavelength of the waves over water with the wind sea's peak at
// `windPeak` and running at `speed`, m.
float chopPeak(float windPeak, float speed) {
    return max(windPeak, min(CHOP_FLOW_PEAK * speed, CHOP_FLOW_PEAK_MAX));
}

// One band's slope at xz, bilinear between its texels.
vec2 chopBandAt(int band, vec2 xz) {
    vec2 f = xz * chopBands[band].x - 0.5;
    vec2 cell = floor(f);
    vec2 t = f - cell;
    ivec2 c0 = ivec2(cell) & (CHOP_TILE_TEXELS - 1);
    ivec2 c1 = (c0 + 1) & (CHOP_TILE_TEXELS - 1);
    int base = band * CHOP_TILE_TEXELS * CHOP_TILE_TEXELS;
    vec2 a = chopTexels[base + c0.y * CHOP_TILE_TEXELS + c0.x];
    vec2 b = chopTexels[base + c0.y * CHOP_TILE_TEXELS + c1.x];
    vec2 c = chopTexels[base + c1.y * CHOP_TILE_TEXELS + c0.x];
    vec2 d = chopTexels[base + c1.y * CHOP_TILE_TEXELS + c1.x];
    return mix(mix(a, b, t.x), mix(c, d, t.x), t.y);
}

// The chop's slope at xz, `roughness` times the standard steepness, over
// water whose waves peak at `peak` (m). A band fades out as its waves grow
// past twice the peak, and as they grow too short for the pixel's footprint
// (m), rather than sparkling.
vec2 chopSlopeAt(vec2 xz, float roughness, float peak, float footprint) {
    float calm = clamp(peak / CHOP_PEAK_FLOOR, 0.0, 1.0);
    peak = max(peak, CHOP_PEAK_FLOOR);
    // A young sea's waves are as steep, for their length, as a grown one's:
    // the slopes are measured against the longest wave the water raises.
    float top = min(2.0 * peak, chopBands[0].y);
    vec2 slope = vec2(0.0);
    for (int b = 0; b < CHOP_BANDS; b++) {
        float longest = chopBands[b].y;
        float resolved = 1.0 - smoothstep(0.125 * longest, 0.25 * longest, footprint);
        if (resolved <= 0.0 || longest < CHOP_SHORTEST_SHARE * top) {
            break;
        }
        resolved *= 1.0 - smoothstep(peak, 2.0 * peak, 0.75 * longest);
        if (resolved <= 0.0) {
            continue;
        }
        slope += chopBandAt(b, xz) * (longest / top * resolved);
    }
    return slope * (CHOP_STEEPNESS * roughness * calm);
}

#endif
