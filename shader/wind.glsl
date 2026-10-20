// The wind map: per metre of the field, the peak wavelength of the wind sea
// there, which sets how long the chop (chop.glsl) runs. Built on the CPU by
// src/water/surface/exposure.rs and uploaded by
// src/rendering/water/renderer.rs: a header, then one float per texel,
// row-major. Read per vertex, as the map is smooth over metres, and passed to
// the fragments in fragSwell.w.

#ifndef WIND_GLSL
#define WIND_GLSL

layout(std430, set = 1, binding = 1) readonly buffer WindMap {
    // Origin x and z (m), texels per metre, width, depth, the peak past the
    // map's edges, two unused.
    vec4 windHeader[2];
    float windPeak[];
};

// One texel's peak wavelength; past the map's edges, the sea's or none.
float windTexel(ivec2 texel) {
    ivec2 size = ivec2(windHeader[0].w, windHeader[1].x);
    if (any(lessThan(texel, ivec2(0))) || any(greaterThanEqual(texel, size))) {
        return windHeader[1].y;
    }
    return windPeak[texel.y * size.x + texel.x];
}

// The wind sea's peak wavelength at xz, m, bilinear between texel centres.
float windPeakAt(vec2 xz) {
    vec2 f = (xz - windHeader[0].xy) * windHeader[0].z - 0.5;
    ivec2 c = ivec2(floor(f));
    vec2 t = f - vec2(c);
    float a = mix(windTexel(c), windTexel(c + ivec2(1, 0)), t.x);
    float b = mix(windTexel(c + ivec2(0, 1)), windTexel(c + ivec2(1, 1)), t.x);
    return mix(a, b, t.y);
}

#endif
