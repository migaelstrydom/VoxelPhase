// Swell: the analytic long waves on every body of still water.
//
// Mirrors src/water/surface/swell.rs wave for wave, so the surface drawn is
// the surface buoyancy floats bodies on. A test in that file checks the
// spectrum below against the Rust one.

struct SwellWave {
    float heading;     // radians from +x towards +z
    float wavelength;  // metres
    float share;       // share of the body's amplitude
};

const int SWELL_WAVES = 8;
const SwellWave SWELL_SPECTRUM[SWELL_WAVES] = SwellWave[](
    SwellWave(0.0, 9.7, 0.24),
    SwellWave(0.6, 6.9, 0.19),
    SwellWave(-0.5, 5.3, 0.15),
    SwellWave(1.1, 3.8, 0.12),
    SwellWave(-0.9, 2.9, 0.10),
    SwellWave(0.3, 2.2, 0.08),
    SwellWave(-1.4, 1.6, 0.07),
    SwellWave(1.5, 1.2, 0.05)
);

const float SWELL_GRAVITY = 9.81;
const float SWELL_SHORE_FADE = 1.0;

float swellShoreFade(float depth) {
    return smoothstep(0.0, SWELL_SHORE_FADE, depth);
}

// Height above the level at xz, and its gradient in .yz.
vec3 swellAt(vec2 xz, float t, float amplitude, float phase, float depth) {
    vec3 out_h = vec3(0.0);
    if (amplitude <= 0.0) {
        return out_h;
    }
    for (int i = 0; i < SWELL_WAVES; i++) {
        SwellWave w = SWELL_SPECTRUM[i];
        float k = 6.28318530718 / w.wavelength;
        float omega = sqrt(SWELL_GRAVITY * k);
        vec2 d = vec2(cos(w.heading), sin(w.heading));
        float arg = k * dot(d, xz) - omega * t + phase;
        out_h.x += w.share * sin(arg);
        out_h.yz += d * (w.share * k * cos(arg));
    }
    return out_h * (amplitude * swellShoreFade(depth));
}

// The swell's slope at a fragment, for its normal: each wave fades out as it
// grows too short for the pixel's footprint (m), rather than sparkling.
// `amplitude` already carries the shore fade. The geometry may carry less
// swell than this, or none (the ring past the map's edge); the normal still
// reads as the sea's.
vec2 swellSlopeAt(vec2 xz, float t, float amplitude, float phase, float footprint) {
    vec2 slope = vec2(0.0);
    if (amplitude <= 0.0) {
        return slope;
    }
    for (int i = 0; i < SWELL_WAVES; i++) {
        SwellWave w = SWELL_SPECTRUM[i];
        float k = 6.28318530718 / w.wavelength;
        float omega = sqrt(SWELL_GRAVITY * k);
        vec2 d = vec2(cos(w.heading), sin(w.heading));
        float arg = k * dot(d, xz) - omega * t + phase;
        float resolved = 1.0 - smoothstep(0.25 * w.wavelength, 0.5 * w.wavelength, footprint);
        slope += d * (w.share * k * cos(arg) * resolved);
    }
    return slope * amplitude;
}
