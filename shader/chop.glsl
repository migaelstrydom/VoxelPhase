// Chop: the wind waves on every body of water, over the swell.
//
// A sum of sines like the swell (swell.glsl), from 3.5 m down to 0.4 m, each
// travelling at its own deep-water speed, so the pattern keeps re-forming
// rather than sliding past as one picture. A wave's steepness (amplitude ×
// wavenumber) falls in proportion to its length, so the longer waves carry
// the shape and the shortest only break it up: real ripples run finer still,
// but seen from a few metres up they read as a uniform texture, so they are
// left out. One number sets how rough the water is. Their headings scatter
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

const int CHOP_WAVES = 16;
const ChopWave CHOP_SPECTRUM[CHOP_WAVES] = ChopWave[](
    ChopWave(vec2(1.2752, 1.3756), 4.2897, 0.455, vec2(0.6798, 0.7334), vec2(-0.9747, -0.2235), 3.350),
    ChopWave(vec2(1.0859, 1.8676), 4.6036, 0.364, vec2(0.4364, 0.7506), vec2(-0.9989, -0.0467), 2.908),
    ChopWave(vec2(2.3221, 0.4551), 4.8180, 0.570, vec2(0.7779, 0.1525), vec2(-0.8896, 0.4567), 2.655),
    ChopWave(vec2(2.6487, 1.4423), 5.4393, 0.778, vec2(0.5462, 0.2974), vec2(0.1674, 0.9859), 2.083),
    ChopWave(vec2(1.0514, 3.1930), 5.7426, 2.492, vec2(0.1745, 0.5300), vec2(0.9889, -0.1486), 1.869),
    ChopWave(vec2(3.5440, 0.3182), 5.9081, 5.394, vec2(0.5251, 0.0471), vec2(-0.2463, 0.9692), 1.766),
    ChopWave(vec2(2.5838, 3.2208), 6.3645, 5.128, vec2(0.2843, 0.3543), vec2(0.4216, 0.9068), 1.522),
    ChopWave(vec2(3.2014, 3.8634), 7.0158, 4.014, vec2(0.2385, 0.2879), vec2(-0.6955, 0.7186), 1.252),
    ChopWave(vec2(4.4378, 3.6082), 7.4906, 1.294, vec2(0.2545, 0.2069), vec2(-0.4235, -0.9059), 1.099),
    ChopWave(vec2(5.5796, 3.2241), 7.9509, 1.974, vec2(0.2520, 0.1456), vec2(-0.8589, -0.5121), 0.975),
    ChopWave(vec2(7.3440, -0.9520), 8.5233, 4.392, vec2(0.2512, -0.0326), vec2(0.0371, 0.9993), 0.848),
    ChopWave(vec2(-4.4629, 7.3760), 9.1963, 3.300, vec2(-0.1126, 0.1862), vec2(0.7077, -0.7065), 0.729),
    ChopWave(vec2(9.6295, -2.9880), 9.9453, 0.742, vec2(0.1777, -0.0551), vec2(-0.8706, 0.4921), 0.623),
    ChopWave(vec2(-11.1705, -3.0887), 10.6628, 0.955, vec2(-0.1560, -0.0431), vec2(-0.9976, 0.0693), 0.542),
    ChopWave(vec2(10.1751, -6.4391), 10.8686, 3.600, vec2(0.1316, -0.0833), vec2(0.7092, -0.7050), 0.522),
    ChopWave(vec2(2.7513, -14.0456), 11.8493, 4.369, vec2(0.0252, -0.1286), vec2(-0.8293, -0.5588), 0.439)
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

// The chop's slope at xz and time t, `roughness` times the standard
// steepness. `groups` is the group field there, each component in [-1, 1].
// Each wave fades out as it grows too short for the pixel's footprint (m),
// rather than sparkling.
vec2 chopSlopeAt(vec2 xz, float t, vec2 groups, float roughness, float footprint) {
    vec2 slope = vec2(0.0);
    for (int i = 0; i < CHOP_WAVES; i++) {
        ChopWave w = CHOP_SPECTRUM[i];
        float resolved = 1.0 - smoothstep(0.25 * w.wavelength, 0.5 * w.wavelength, footprint);
        if (resolved <= 0.0) {
            break;
        }
        float arg = dot(w.wavevector, xz) - w.omega * t + w.phase;
        float grouped = max(1.0 + CHOP_GROUPING * dot(groups, w.group), 0.0);
        slope += w.slope * (grouped * cos(arg) * resolved);
    }
    return slope * (CHOP_STEEPNESS * roughness);
}

#endif
