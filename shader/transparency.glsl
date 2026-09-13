// How much of a pixel a see-through surface claims.
//
// The whole of the transparency model that reaches the blend unit. A blended
// draw's alpha is not the authored opacity — that alone gives the flat grey
// ghost every first attempt at glass looks like. What reads as glass is that
// the surface is clear when you look straight through it and a mirror when you
// look along it, which is the Fresnel curve, and that a highlight on it is a
// solid white speck rather than a faint one, which is the second rule below.
//
// Matches src/rendering/transparency/optics.rs: the CPU owns the two authored
// numbers and the derivation of reflectance from refractive index; this owns
// the per-fragment curve. Neither duplicates the other.

#ifndef TRANSPARENCY_GLSL
#define TRANSPARENCY_GLSL

// For `luminance`, the one definition of Rec. 709 weights in the engine.
#include "tonemap.glsl"

/// Fraction of the pixel a transmissive surface covers, at this viewing angle.
///
/// Rises from `opacity` head-on to fully opaque at the silhouette, following
/// Schlick's Fresnel curve from the surface's reflectance at normal incidence.
/// Light that reflects off the interface is light that did not pass through it,
/// so the reflected fraction is exactly the coverage the surface gains.
float glassCoverage(float opacity, float reflectance, vec3 normal, vec3 view_dir) {
    // Magnitude, not the signed dot: the curve depends on the angle between
    // the surface and the view ray, and a back face — which a transmissive
    // surface shows, since both sides of it are drawn — is at the same angle
    // as the front face it belongs to. Reading the sign instead sends every
    // back face to full coverage, which turns the far half of a block of ice
    // solid and is very hard to recognise as a Fresnel bug.
    float facing = abs(dot(normal, view_dir));
    float fresnel = reflectance + (1.0 - reflectance) * pow(1.0 - facing, 5.0);
    return clamp(opacity + (1.0 - opacity) * fresnel, 0.0, 1.0);
}

/// Coverage a specular highlight adds on its own.
///
/// A highlight is light bouncing off the front of the surface, so none of it
/// is attenuated by how much passes through — but a single alpha cannot say
/// "transmit the background and add this". Letting a bright highlight raise
/// coverage is the standard approximation, and it is the difference between
/// ice that glints and ice that looks like a smudge on the lens.
///
/// Saturates, so a highlight bright enough to bloom does not also drive alpha
/// past 1 and start subtracting background.
float highlightCoverage(vec3 specular) {
    return clamp(luminance(specular), 0.0, 1.0);
}

#endif // TRANSPARENCY_GLSL
