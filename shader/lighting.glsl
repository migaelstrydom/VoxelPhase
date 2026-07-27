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

/// Diffuse plus specular response to a single directional light.
vec3 shadeDirectional(SurfaceSample surface, DirectionalLight light) {
    float n_dot_l = max(dot(surface.normal, light.direction), 0.0);
    if (n_dot_l <= 0.0) {
        return vec3(0.0);
    }

    vec3 half_vector = normalize(light.direction + surface.view_dir);
    float n_dot_h = max(dot(surface.normal, half_vector), 0.0);
    float h_dot_v = max(dot(half_vector, surface.view_dir), 0.0);

    // Metals absorb the transmitted component, so they have no diffuse lobe.
    vec3 diffuse = surface.albedo * (1.0 - surface.metallic);

    float power = specularPower(surface.roughness);
    // Keeps total reflected specular energy roughly constant across roughness.
    float normalisation = (power + 8.0) / 8.0;
    vec3 fresnel = fresnelSchlick(specularF0(surface.albedo, surface.metallic), h_dot_v);
    vec3 specular = fresnel * normalisation * pow(n_dot_h, power);

    return light.colour * light.intensity * n_dot_l * (diffuse + specular);
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
