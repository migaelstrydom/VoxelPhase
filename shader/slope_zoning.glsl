// Slope zoning: how steep ground wears its material.
//
// Nothing settles on a steep face. Turf thins on a steep flank until its soil
// shows through, and dust and weathering collect on the ledges and tops of
// hard ground, not on its faces. Slope changes a material's finish here, never
// its substance: what a surface *is* stays the voxel material, because that is
// what destruction spends against (docs/VISUAL_DIRECTION.md §2, §4.4). Which
// material bares a cliff is the level's surface bands' call, made in
// generation.
//
// Steepness is read from the geometric normal, not the detail normal, and over
// a wide blend: gradient normals smear across rims and creases, and a narrow
// threshold would paint a band of the wrong finish along every edge, tracking
// the smear rather than the shape.

#ifndef SLOPE_ZONING_GLSL
#define SLOPE_ZONING_GLSL

/// Normal heights (`n.y`) over which ground turns from flat to steep. A slope
/// of 1.5, the island's cliff band, is n.y 0.55.
const vec2 ZONING_STEEPENING = vec2(0.9, 0.55);

/// How far turf gives way to its soil where it does: the share of the albedo
/// taken by the soil colour, and of the pile lost, in a bare patch.
const float TURF_THINNING = 0.4;

/// Steepness at which turf starts to go bare, where the patchwork is lowest
/// and where it is highest. Between them, more of a flank goes bare the
/// steeper it is, in patches rather than as an even cast.
const vec2 TURF_BARING = vec2(0.35, 1.05);

/// Width of the edge of a bare patch, in steepness.
const float TURF_PATCH_EDGE = 0.25;

/// The soil that shows through thin turf: a dark loam, darker than the dirt
/// voxel's own colour, because it is seen in the shade of the blades.
const vec3 TURF_SOIL = vec3(0.22, 0.17, 0.09);

/// How much paler hard ground weathers where it faces the sky.
const float LEDGE_WEATHERING = 0.18;

/// Colour weathering pales hard ground towards: dust and lichen, warm grey.
const vec3 LEDGE_DUST = vec3(0.80, 0.78, 0.72);

/// Hardness over which ground starts to show weathering on its tops. Soft
/// ground is loose and has no weathered skin; rock does.
const vec2 WEATHERING_HARDNESS = vec2(0.3, 0.5);

/// How steep the ground is, 0 flat to 1 at a cliff band's slope and beyond.
float zoningSteepness(vec3 geometric_normal) {
    return smoothstep(ZONING_STEEPENING.x, ZONING_STEEPENING.y, geometric_normal.y);
}

/// Where in a patchwork a point falls, 0 to 1, from the terrain field's
/// albedo wash (0.85 to 1.0): its low-frequency content is broad enough to
/// break the thinning into patches that agree with the light and dark already
/// on the surface.
float zoningPatch(float wash) {
    return smoothstep(0.88, 0.97, wash);
}

/// How much of the turf survives on ground this steep, 1 to 1 - TURF_THINNING.
/// Each point goes bare past its own steepness, set by the patchwork, so a
/// flank goes bare in patches that spread as it steepens; an even cast over
/// the whole flank reads as dirt, not as thin turf.
float turfCover(float steepness, float patch_value) {
    float threshold = mix(TURF_BARING.x, TURF_BARING.y, patch_value);
    float bare = smoothstep(threshold - TURF_PATCH_EDGE, threshold + TURF_PATCH_EDGE, steepness);
    return 1.0 - TURF_THINNING * bare;
}

/// The albedo of ground of this hardness and pile, worn by its slope.
vec3 zonedAlbedo(vec3 albedo, float hardness, float pile, float steepness, float patch_value) {
    vec3 thinned = mix(TURF_SOIL, albedo, turfCover(steepness, patch_value));
    albedo = mix(albedo, thinned, pile);

    float exposure = (1.0 - steepness) * smoothstep(WEATHERING_HARDNESS.x, WEATHERING_HARDNESS.y, hardness);
    return mix(albedo, LEDGE_DUST, LEDGE_WEATHERING * exposure * (1.0 - pile));
}

/// The pile left on ground this steep.
float zonedPile(float pile, float steepness, float patch_value) {
    return pile * turfCover(steepness, patch_value);
}

#endif // SLOPE_ZONING_GLSL
