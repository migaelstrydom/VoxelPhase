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

/// Detail-normal strength of the softest material.
///
/// Not zero. Soft ground is not smooth — sand and loose earth have plenty of
/// relief — but its relief is rounded, and a weaker perturbation of the same
/// height field is what rounded reads as.
const float CHALK_RELIEF = 0.55;

/// Detail-normal strength of an indestructible surface. Full strength: dense
/// rock is where the tight, sharp-edged grain belongs, and it is the material
/// whose roughness is low enough for that grain to catch a highlight.
const float STONE_RELIEF = 1.0;

/// How strongly a terrain surface of this hardness shows its microstructure.
///
/// The pairing with `roughnessFromHardness` is the point: hard materials get a
/// tighter specular lobe *and* more surface for it to break up against, which
/// is what "rocky glint" is. Either alone reads as wrong — a tight lobe on a
/// smooth surface is plastic, and relief under a fully rough lobe is invisible.
float reliefFromHardness(float hardness) {
    return mix(CHALK_RELIEF, STONE_RELIEF, clamp(hardness, 0.0, 1.0));
}

#endif // SURFACE_CHARACTER_GLSL
