// Shared lighting model for all lit geometry.
//
// Included by every shader that shades a surface, so that terrain, props and
// special-case materials (orbs, characters) all resolve light the same way.
// Contains no uniform declarations — callers gather their own inputs and pass
// them in as plain structs.

#ifndef LIGHTING_GLSL
#define LIGHTING_GLSL

/// A shaded point on a surface, in world space.
struct SurfaceSample {
    /// Base colour before lighting.
    vec3 albedo;
    /// Surface normal, normalized.
    vec3 normal;
    /// Direction from the surface towards the camera, normalized.
    vec3 view_dir;
    /// Perceptual roughness: 0 = mirror, 1 = fully diffuse.
    float roughness;
    /// 0 = dielectric (white highlight), 1 = metal (tinted highlight, no diffuse).
    float metallic;
};

/// An infinitely distant light such as the sun.
struct DirectionalLight {
    /// Direction from the surface towards the light, normalized.
    vec3 direction;
    /// Linear light colour.
    vec3 colour;
    /// Scalar brightness multiplier.
    float intensity;
};

/// A local light with a position and a finite reach.
///
/// Declared here beside `DirectionalLight` rather than in lights.glsl so that
/// this header keeps its no-uniform-declarations property: lights.glsl owns the
/// uniform block that arrays these, and includes this file for the type. Layout
/// must match `GpuPointLight` in src/rendering/frame.rs.
struct PointLight {
    /// xyz = world position, w = range (contribution is zero at and beyond it).
    vec4 position_range;
    /// rgb = linear colour, w = intensity multiplier.
    vec4 colour_intensity;
};

/// Roughness below this would produce an aliasing-prone specular spike.
const float MIN_ROUGHNESS = 0.03;

/// Reflectance at normal incidence for a non-metallic surface.
const vec3 DIELECTRIC_F0 = vec3(0.04);

/// Blinn-Phong specular exponent derived from a perceptual roughness value.
float specularPower(float roughness) {
    float r = max(roughness, MIN_ROUGHNESS);
    float r4 = r * r * r * r;
    return max(2.0 / r4 - 2.0, 1.0);
}

/// Schlick's approximation of the Fresnel reflectance curve.
vec3 fresnelSchlick(vec3 f0, float cos_theta) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cos_theta, 0.0, 1.0), 5.0);
}

/// Reflectance at normal incidence: dielectrics are neutral, metals tint by albedo.
vec3 specularF0(vec3 albedo, float metallic) {
    return mix(DIELECTRIC_F0, albedo, metallic);
}

/// Diffuse plus specular response to light of the given radiance arriving from
/// `light_dir` (normalized, pointing from the surface towards the light).
///
/// The shared core of every light type: directional and point lights differ
/// only in how they derive `light_dir` and `radiance`.
vec3 shadeLight(SurfaceSample surface, vec3 light_dir, vec3 radiance) {
    float n_dot_l = max(dot(surface.normal, light_dir), 0.0);
    if (n_dot_l <= 0.0) {
        return vec3(0.0);
    }

    vec3 half_vector = normalize(light_dir + surface.view_dir);
    float n_dot_h = max(dot(surface.normal, half_vector), 0.0);
    float h_dot_v = max(dot(half_vector, surface.view_dir), 0.0);

    // Metals absorb the transmitted component, so they have no diffuse lobe.
    vec3 diffuse = surface.albedo * (1.0 - surface.metallic);

    float power = specularPower(surface.roughness);
    // Keeps total reflected specular energy roughly constant across roughness.
    float normalisation = (power + 8.0) / 8.0;
    vec3 fresnel = fresnelSchlick(specularF0(surface.albedo, surface.metallic), h_dot_v);
    vec3 specular = fresnel * normalisation * pow(n_dot_h, power);

    return radiance * n_dot_l * (diffuse + specular);
}

/// Diffuse plus specular response to a single directional light.
vec3 shadeDirectional(SurfaceSample surface, DirectionalLight light) {
    return shadeLight(surface, light.direction, light.colour * light.intensity);
}

/// Distance falloff for a point light, in [0, 1].
///
/// Inverse square, windowed so it decays smoothly to exactly zero at `range`.
/// The window is what lets a light drop out of the collector's set, or cross its
/// own range, without popping — an unwindowed inverse square never reaches zero.
///
/// This is the only implementation of the curve. `LightCollector` on the CPU
/// selects lights by camera distance and does not evaluate falloff, so there is
/// no second copy to keep in step.
float pointAttenuation(float dist, float range) {
    if (dist >= range || range <= 0.0) {
        return 0.0;
    }
    float ratio = dist / range;
    float window = max(1.0 - ratio * ratio * ratio * ratio, 0.0);
    return (window * window) / (1.0 + dist * dist);
}

/// Diffuse plus specular response to a single point light.
///
/// Returns black outside the light's range. The range test is a squared-distance
/// compare before any of the BRDF work, so out-of-range lights cost almost
/// nothing — and the branch is coherent across a tile, since neighbouring
/// fragments share a light's reach.
vec3 shadePoint(SurfaceSample surface, PointLight light, vec3 world_pos) {
    vec3 to_light = light.position_range.xyz - world_pos;
    float dist_sq = dot(to_light, to_light);
    float range = light.position_range.w;

    if (dist_sq >= range * range || dist_sq <= 0.0) {
        return vec3(0.0);
    }

    float dist = sqrt(dist_sq);
    vec3 light_dir = to_light / dist;
    vec3 radiance = light.colour_intensity.rgb * light.colour_intensity.w
                  * pointAttenuation(dist, range);

    return shadeLight(surface, light_dir, radiance);
}

/// Uniform ambient fill. Metals take no diffuse ambient.
vec3 shadeAmbient(SurfaceSample surface, vec3 ambient_colour) {
    return surface.albedo * (1.0 - surface.metallic) * ambient_colour;
}

/// Grazing-angle brightening, 0 facing the camera and 1 at the silhouette.
/// Higher `power` tightens the rim towards the edge.
float fresnelRim(vec3 normal, vec3 view_dir, float power) {
    float facing = clamp(dot(normal, view_dir), 0.0, 1.0);
    return pow(1.0 - facing, power);
}

#endif // LIGHTING_GLSL
