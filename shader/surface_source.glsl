// Where a fragment's shading inputs come from.
//
// These bits must match `SurfaceSource` in src/rendering/surface_source.rs.
// They replace an older arrangement in which `triplanarScale > 0` decided four
// independent things at once, which meant only terrain could have any of them.

#ifndef SURFACE_SOURCE_GLSL
#define SURFACE_SOURCE_GLSL

/// Sample the albedo by world position across the three axis planes.
const uint SOURCE_ALBEDO_TRIPLANAR = 1u << 0;

/// The location-2 vertex channel carries surface character, not UVs.
const uint SOURCE_CHARACTER_IN_TEXCOORD = 1u << 1;

/// Derive roughness per fragment from surface character.
const uint SOURCE_ROUGHNESS_FROM_CHARACTER = 1u << 2;

/// Scale the detail normal by surface character rather than by the material.
const uint SOURCE_RELIEF_FROM_CHARACTER = 1u << 3;

/// Perturb the normal with the grain atlas, projected from model space.
const uint SOURCE_GRAIN_OBJECT_SPACE = 1u << 4;

/// Perturb the normal with the grain atlas, addressed by vertex UVs.
const uint SOURCE_GRAIN_BY_UV = 1u << 5;

/// The diffuse alpha is relief height rather than coverage.
const uint SOURCE_RELIEF_IN_ALPHA = 1u << 6;

/// Either grain projection.
const uint SOURCE_GRAIN_ANY = SOURCE_GRAIN_OBJECT_SPACE | SOURCE_GRAIN_BY_UV;

bool sourceHas(uint flags, uint bits) { return (flags & bits) != 0u; }

#endif // SURFACE_SOURCE_GLSL
