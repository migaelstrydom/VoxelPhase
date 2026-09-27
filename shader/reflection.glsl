// What a surface reflects: its reflection probe over the sky, or the sky alone.
//
// Only the scene pass includes this. The probe capture pass renders *into*
// the probes and reflects the sky alone (environment.glsl's `reflectedSky`).
//
// See src/rendering/reflection/ for how probes are placed, captured and
// assigned to draws.

#ifndef REFLECTION_GLSL
#define REFLECTION_GLSL

#include "material.glsl"
#include "environment.glsl"

/// Every live probe's cube, indexed by probe slot, each with a mip chain
/// filtered down from its captured faces.
///
/// Alpha is coverage: what fraction of a texel's cone the capture hit
/// something in, with the colour premultiplied by it. Where a probe saw
/// nothing but sky, alpha is 0 and the analytic sky shows through.
layout(set = 0, binding = 5) uniform samplerCubeArray reflectionProbes;

/// Sky and surroundings along a surface's mirror direction, blurred for its
/// roughness.
///
/// Roughness picks the probe's mip. Its mips are box-filtered rather than
/// prefiltered with the GGX lobe, so the mapping is a fit by eye, linear
/// across the chain: a mirror reads the full-resolution face and a fully rough
/// surface the one-texel average of it.
vec3 reflectedRadiance(SurfaceSample surface, vec3 sun_dir) {
    vec3 sky = reflectedSky(surface, sun_dir);
    uint probe = materialProbe();
    if (probe == NO_PROBE) {
        return sky;
    }

    float lod = surface.roughness * float(textureQueryLevels(reflectionProbes) - 1);
    vec4 surroundings = textureLod(
        reflectionProbes, vec4(reflectionDirection(surface), float(probe)), lod);
    return surroundings.rgb + (1.0 - surroundings.a) * sky;
}

#endif // REFLECTION_GLSL
