// Turning a terrain material's hardness into a finish.
//
// Terrain surfaces are shaded from the number destruction spends against, so
// that what a wall looks like predicts what a grenade does to it. Hardness
// arrives per vertex (see `surface_character` in src/rendering/vertex.rs) and
// is `VoxelMaterial::hardness`: 0 is the softest thing that can exist, 1 is
// indestructible.

#ifndef SURFACE_CHARACTER_GLSL
#define SURFACE_CHARACTER_GLSL

/// Roughness of the softest material. Fully diffuse — chalk and loose grain
/// have no coherent highlight at all, and anything less than 1.0 here puts a
/// sheen on sand that reads as damp.
const float CHALK_ROUGHNESS = 1.0;

/// Roughness of an indestructible surface. Deliberately short of a polish:
/// bedrock is dense stone rather than glass, and a lobe tight enough to glare
/// would alias against the terrain's normals long before it looked slick.
const float STONE_ROUGHNESS = 0.45;

/// The finish a terrain surface of this hardness takes.
///
/// Linear in hardness, because hardness has already been through the curve that
/// decides how the material ladder is spaced (`VoxelMaterial::hardness`).
/// Putting a second curve here would make the two impossible to tune apart.
float roughnessFromHardness(float hardness) {
    return mix(CHALK_ROUGHNESS, STONE_ROUGHNESS, clamp(hardness, 0.0, 1.0));
}

#endif // SURFACE_CHARACTER_GLSL
