// Lighting from the surroundings rather than from a named light.
//
// A surface lit only by discrete lights can be bright solely where a light
// happens to point at it. That is why a roughness sweep barely reads — the
// only thing distinguishing polished from matte is the width of one highlight —
// and why a metal, which has no diffuse response at all, renders as a black
// disc with a dot on it. Both need something to reflect.
//
// The something is the sky. It is an analytic function of direction
// (sky_model.glsl), so both queries below are point evaluations of it: no
// probe capture, no cubemap, no prefiltering pass, and no possibility of the
// reflection disagreeing with the sky the camera can see.

#ifndef ENVIRONMENT_GLSL
#define ENVIRONMENT_GLSL

#include "sky_model.glsl"
#include "lighting.glsl"

/// How much of the ground's radiance actually reaches a downward-facing
/// surface.
///
/// The sky model's below-horizon colour is already what the ground looks like,
/// so taking it at face value would light undersides as brightly as the ground
/// they face. This discount stands in for the occlusion that is not otherwise
/// simulated: a surface pointing down is usually pointing at something close.
/// It only affects the diffuse term — a mirror aimed at the ground still
/// reflects the ground at full strength, which is what a mirror does.
///
/// Revisit this when ambient occlusion lands; it is doing that job by proxy.
const float GROUND_ALBEDO = 0.25;

/// How much of the sky hemisphere a diffuse surface actually sees.
///
/// The same discount as `GROUND_ALBEDO` and for the same reason: nothing in the
/// scene occludes the sky yet, so taking the full hemisphere would light every
/// surface as though it stood alone on an open plain. It also keeps the fill
/// well below the sun, which is what gives a surface its modelling — a fill
/// that rivals the key renders flat however physically defensible it is.
///
/// Diffuse only. A mirror still reflects the sky at full strength, so a glossy
/// surface and the sky behind it continue to agree.
const float SKY_IRRADIANCE_FACTOR = 0.4;

/// The sky as it looks to a surface that reflects the entire hemisphere at
/// once: the zenith colour above, the nadir colour below, blended by elevation.
///
/// This is the limit case of blurring the sky infinitely, and it is the only
/// average the model needs. Both queries below are built from it — the diffuse
/// one uses it directly, the specular one interpolates towards it.
///
/// Sampling a ring of directions instead would seem more faithful and is worse:
/// with few enough taps to afford per fragment, each one crosses the horizon at
/// a different roughness and the discontinuity shows up as concentric banding
/// on rough metal. An analytic average has no taps to alias.
vec3 skyDomeAverage(vec3 dir, vec3 sun_dir) {
    vec3 above = skyRadiance(vec3(0.0, 1.0, 0.0), sun_dir);
    vec3 below = skyRadiance(vec3(0.0, -1.0, 0.0), sun_dir);
    return mix(below, above, dir.y * 0.5 + 0.5);
}

/// Irradiance arriving at a surface from the sky above it and the ground below.
///
/// Upward-facing normals see mostly sky, downward-facing ones see mostly
/// ground, and the blend tracks the normal's elevation. The gradient this
/// produces is doing the work a flat ambient constant cannot — it tells you
/// which way a surface faces even where no direct light reaches it.
vec3 environmentIrradiance(vec3 normal, vec3 sun_dir) {
    vec3 sky = skyRadiance(vec3(0.0, 1.0, 0.0), sun_dir) * SKY_IRRADIANCE_FACTOR;
    vec3 ground = skyRadiance(vec3(0.0, -1.0, 0.0), sun_dir) * GROUND_ALBEDO;

    float upward = normal.y * 0.5 + 0.5;
    return mix(ground, sky, upward);
}

/// Sky radiance over the reflection cone for a given roughness.
///
/// Stands in for a prefiltered environment map. A mirror returns the sky along
/// exactly one direction; as roughness grows the result slides towards the
/// hemisphere average, which is what an infinitely wide lobe would gather.
/// Squaring roughness keeps the low end tight, where small changes in gloss are
/// most visible.
vec3 prefilteredSkyRadiance(vec3 reflect_dir, vec3 sun_dir, float roughness) {
    vec3 sharp = skyRadiance(reflect_dir, sun_dir);
    vec3 blurred = skyDomeAverage(reflect_dir, sun_dir);
    return mix(sharp, blurred, clamp(roughness * roughness, 0.0, 1.0));
}

/// Split-sum environment BRDF, as the analytic fit Karis published for mobile.
///
/// Replaces the usual lookup texture. Returns the scale and bias to apply to
/// F0, accounting for how much of the reflected energy survives at this
/// roughness and viewing angle.
vec2 environmentBrdf(float roughness, float n_dot_v) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * n_dot_v)) * r.x + r.y;
    return vec2(-1.04, 1.04) * a004 + r.zw;
}

/// Total light a surface receives from its surroundings: a diffuse term from
/// sky and ground irradiance, and a specular term reflecting the sky.
///
/// The specular half is what makes roughness legible across its whole range and
/// what gives metals anything at all to show, since a metal's colour is
/// entirely its reflection.
vec3 shadeEnvironment(SurfaceSample surface, vec3 sun_dir) {
    float n_dot_v = max(dot(surface.normal, surface.view_dir), 0.0);
    vec3 reflect_dir = reflect(-surface.view_dir, surface.normal);

    vec3 irradiance = environmentIrradiance(surface.normal, sun_dir);
    vec3 diffuse = surface.albedo * (1.0 - surface.metallic) * irradiance;

    vec3 radiance = prefilteredSkyRadiance(reflect_dir, sun_dir, surface.roughness);
    vec3 f0 = specularF0(surface.albedo, surface.metallic);
    vec2 brdf = environmentBrdf(surface.roughness, n_dot_v);
    vec3 specular = radiance * (f0 * brdf.x + brdf.y);

    // Diffuse takes occlusion at full strength: irradiance really does arrive
    // from the whole hemisphere, and a crease really does see less of it.
    //
    // Specular takes it at half. A glossy surface in a crevice still reflects
    // whatever is in front of it, so occluding the reflection fully reads as
    // dirt rather than as shade. Half is the simple version; if it looks wrong
    // on the metals in `material_grid`, horizon-based specular occlusion is the
    // principled replacement.
    return diffuse * surface.occlusion + specular * mix(1.0, surface.occlusion, 0.5);
}

#endif // ENVIRONMENT_GLSL
