// Chop: the wind waves on every body of water, over the swell.
//
// A sum of sines like the swell (swell.glsl), from 3.5 m down to 12 cm, each
// travelling at its own deep-water speed, so the pattern keeps re-forming
// rather than sliding past as one picture. A wave's steepness (amplitude ×
// wavenumber) falls in proportion to its length, so the longer waves carry
// the shape and the shortest only break it up: on the open sea the shortest
// barely show, and only on a pond, where nothing longer grows, are they the
// waves seen. One number sets how rough the water is. Their headings scatter
// around one wind; the shorter the wave, the wider the scatter.
//
// A sum of sines with neat wavelengths, headings and phases interferes into a
// regular weave. Real seas have no such order: each wave here sits at random
// within its own band of equal log width, with a heading drawn about the wind
// and a random phase (drawn once, seeded, and written out below with
// everything that follows from them, so a pixel pays one cosine a wave), and
// arrives in groups, swelling and fading across a slow field of patches, each
// wave its own way across it, so which waves dominate changes from patch to
// patch.
//
// How long the waves run depends on the water: wind waves grow over their
// fetch, so a pond only ripples, and under a roof there is no wind at all.
// The wind map (wind.glsl) gives, per metre, the peak wavelength of the wind
// sea; a wave much longer than it is not drawn. Running water roughens of
// itself, by its speed.
//
// The GPU alone draws them: they are a few centimetres high, which buoyancy
// does not feel.

#ifndef CHOP_GLSL
#define CHOP_GLSL

struct ChopWave {
    vec2 wavevector;   // along its heading, 2π / wavelength long, 1/m
    float omega;       // angular frequency, from deep water's dispersion, rad/s
    float phase;       // radians at the origin
    vec2 slope;        // along its heading, its steepness as a share of the longest's
    vec2 group;        // which way across the group field it swells
    float wavelength;  // metres
};

const int CHOP_WAVES = 22;
const ChopWave CHOP_SPECTRUM[CHOP_WAVES] = ChopWave[](
    ChopWave(vec2(1.2826, 1.3836), 4.3020, 0.455, vec2(0.6798, 0.7334), vec2(-0.9747, -0.2235), 3.330),
    ChopWave(vec2(1.1424, 1.8957), 4.6597, 0.364, vec2(0.4400, 0.7300), vec2(-0.9989, -0.0467), 2.839),
    ChopWave(vec2(2.4032, 0.4942), 4.9060, 0.570, vec2(0.7532, 0.1549), vec2(-0.8896, 0.4567), 2.561),
    ChopWave(vec2(2.8444, 1.5262), 5.6273, 0.778, vec2(0.5150, 0.2763), vec2(0.1674, 0.9859), 1.946),
    ChopWave(vec2(1.3683, 3.3833), 5.9834, 2.492, vec2(0.1938, 0.4792), vec2(0.9889, -0.1486), 1.722),
    ChopWave(vec2(3.8648, 0.4573), 6.1788, 5.394, vec2(0.4814, 0.0570), vec2(-0.2463, 0.9692), 1.614),
    ChopWave(vec2(3.0601, 3.4412), 6.7213, 5.128, vec2(0.2722, 0.3061), vec2(0.4216, 0.9068), 1.364),
    ChopWave(vec2(3.8961, 4.2156), 7.5042, 4.014, vec2(0.2231, 0.2414), vec2(-0.6955, 0.7186), 1.095),
    ChopWave(vec2(5.3070, 4.0187), 8.0811, 1.294, vec2(0.2259, 0.1711), vec2(-0.4235, -0.9059), 0.944),
    ChopWave(vec2(6.6570, 3.7044), 8.6449, 1.974, vec2(0.2164, 0.1204), vec2(-0.8589, -0.5121), 0.825),
    ChopWave(vec2(8.9015, -0.4986), 9.3521, 4.392, vec2(0.2113, -0.0118), vec2(0.0371, 0.9993), 0.705),
    ChopWave(vec2(-3.0876, 10.1276), 10.1915, 3.300, vec2(-0.0520, 0.1704), vec2(0.7077, -0.7065), 0.593),
    ChopWave(vec2(12.3992, -2.4500), 11.1350, 0.742, vec2(0.1464, -0.0289), vec2(-0.8706, 0.4921), 0.497),
    ChopWave(vec2(-14.4954, 2.9669), 12.0477, 0.955, vec2(-0.1249, 0.0256), vec2(-0.9976, 0.0693), 0.425),
    ChopWave(vec2(14.1807, -6.1322), 12.3111, 3.600, vec2(0.1121, -0.0485), vec2(0.7092, -0.7050), 0.407),
    ChopWave(vec2(8.8346, -16.5768), 13.5747, 4.369, vec2(0.0472, -0.0886), vec2(-0.8293, -0.5588), 0.334),
    ChopWave(vec2(1.5281, -22.7573), 14.9583, 5.936, vec2(0.0055, -0.0825), vec2(-0.9868, 0.1620), 0.275),
    ChopWave(vec2(15.5608, 21.9835), 16.2548, 0.381, vec2(0.0405, 0.0572), vec2(-0.3001, -0.9539), 0.233),
    ChopWave(vec2(-23.9500, 20.1741), 17.5270, 1.788, vec2(-0.0461, 0.0388), vec2(-0.7534, 0.6576), 0.201),
    ChopWave(vec2(34.8927, 11.1270), 18.9547, 0.142, vec2(0.0491, 0.0157), vec2(-0.9712, 0.2384), 0.172),
    ChopWave(vec2(30.2020, 25.5168), 19.6944, 4.827, vec2(0.0364, 0.0308), vec2(0.6876, 0.7261), 0.159),
    ChopWave(vec2(36.2557, 29.3638), 21.3936, 2.456, vec2(0.0314, 0.0254), vec2(0.6910, -0.7228), 0.135)
);

// The longest wave's steepness (amplitude × wavenumber). With their grouping
// the waves sum to an rms slope of about 0.14, the sea's under a gentle breeze.
const float CHOP_STEEPNESS = 0.085;

// How far a wave's amplitude swells and fades across the group field: at 1 it
// vanishes where the field points against it and doubles where it points along.
const float CHOP_GROUPING = 1.0;

// Size of the gusts' patches, m: where the wind drops the chop lies flat and
// glassy, where it gusts it roughens, so a sea seen from afar is not one
// even texture.
const float CHOP_GUST_SIZE = 18.0;

// The chop's roughness in a lull and in a gust, as shares of the standard.
const vec2 CHOP_GUST_RANGE = vec2(0.35, 1.4);

// Size of the group field's patches, m.
const float CHOP_GROUP_SIZE = 6.0;

// How fast the group field drifts downwind, in patches per second: the speed
// at which a group of the chop's longer waves travels, half theirs.
const vec2 CHOP_GROUP_DRIFT = vec2(0.92, 0.39) * (1.0 / CHOP_GROUP_SIZE);

// Shortest wave drawn, as a share of the longest the water raises.
const float CHOP_SHORTEST_SHARE = 0.1;

// Peak wavelength of the waves running water raises of itself, per m/s of
// current, m, and the most it raises.
const float CHOP_FLOW_PEAK = 0.8;
const float CHOP_FLOW_PEAK_MAX = 1.6;

// The longest the chop runs over water with the wind sea's peak at `windPeak`
// and running at `speed`, m.
float chopPeak(float windPeak, float speed) {
    return max(windPeak, min(CHOP_FLOW_PEAK * speed, CHOP_FLOW_PEAK_MAX));
}

// The chop's slope at xz and time t, `roughness` times the standard
// steepness. `groups` is the group field there, each component in [-1, 1].
// A wave fades out as it grows past twice `peak` (m), the longest the water
// there raises, and as it grows too short for the pixel's footprint (m),
// rather than sparkling.
vec2 chopSlopeAt(vec2 xz, float t, vec2 groups, float roughness, float peak, float footprint) {
    // A young sea's waves are as steep, for their length, as a grown one's:
    // the slopes are measured against the longest wave the water raises,
    // not the spectrum's.
    float top = clamp(2.0 * peak, CHOP_SPECTRUM[CHOP_WAVES - 1].wavelength, CHOP_SPECTRUM[0].wavelength);
    float young = CHOP_SPECTRUM[0].wavelength / top;
    vec2 slope = vec2(0.0);
    for (int i = 0; i < CHOP_WAVES; i++) {
        ChopWave w = CHOP_SPECTRUM[i];
        // Past a tenth of the longest, a wave barely tilts the surface: on
        // the sea the shortest are skipped, on a pond they are the waves.
        float resolved = 1.0 - smoothstep(0.25 * w.wavelength, 0.5 * w.wavelength, footprint);
        if (resolved <= 0.0 || w.wavelength < CHOP_SHORTEST_SHARE * top) {
            break;
        }
        resolved *= 1.0 - smoothstep(peak, 2.0 * peak, w.wavelength);
        if (resolved <= 0.0) {
            continue;
        }
        float arg = dot(w.wavevector, xz) - w.omega * t + w.phase;
        float grouped = max(1.0 + CHOP_GROUPING * dot(groups, w.group), 0.0);
        slope += w.slope * (grouped * cos(arg) * resolved);
    }
    return slope * (CHOP_STEEPNESS * roughness * young);
}

#endif
