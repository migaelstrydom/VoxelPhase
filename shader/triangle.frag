#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "scene.glsl"
#include "material.glsl"
#include "lighting.glsl"
#include "lights.glsl"
#include "environment.glsl"
#include "reflection.glsl"
#include "shadow.glsl"
#include "triplanar.glsl"
#include "surface_character.glsl"
#include "surface_source.glsl"
#include "grain.glsl"
#include "relief.glsl"
#include "transparency.glsl"
#include "aerial.glsl"
#include "pile.glsl"
#include "slope_zoning.glsl"
#include "foliage.glsl"

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inTexCoord;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in vec3 inNormal;
layout(location = 4) in float inAo;
layout(location = 5) in vec3 inModelPos;
layout(location = 6) in vec3 inModelNormal;

layout(location = 0) out vec4 outColor;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

/// The shared grain atlas: RG is stone microstructure, BA is directional
/// fibre, both as tangent-space slopes. One texture for every material in the scene — see
/// src/rendering/grain.rs.
layout(set = 0, binding = 4) uniform sampler2D grainSampler;

/// How sharply an object-space grain commits to the axis plane it most nearly
/// faces. Matches terrain's blend: low enough that a 45-degree face does not
/// show a seam, high enough that the three projections do not ghost.
const float GRAIN_BLEND_SHARPNESS = 4.0;

void main() {
    if (material.colour_override.a > 0.0) {
        outColor = material.colour_override;
        return;
    }

    // Degenerate normals occur on some generated meshes; fall back to straight up.
    vec3 normal = length(inNormal) > 0.001 ? normalize(inNormal) : vec3(0.0, 1.0, 0.0);
    // The surface's own shape, before any detail perturbs it.
    vec3 geometric_normal = normal;

    uint source = materialSource();

    // Terrain has no texture coordinates worth sampling — marching cubes emits
    // no parameterisation — so it is textured by world position instead. Meshes
    // with authored UVs take the cheaper single-sample path.
    //
    // Each of these decisions is now its own bit. They all used to hang off
    // whether the triplanar scale was positive, which is why nothing but
    // terrain could have a detail normal: asking for one meant asking for
    // world-projected albedo and terrain's roughness model with it.
    bool albedo_triplanar = sourceHas(source, SOURCE_ALBEDO_TRIPLANAR);

    // A world-textured mesh has no use for its UVs, so terrain sends per-vertex
    // surface character through that channel instead.
    float character = sourceHas(source, SOURCE_CHARACTER_IN_TEXCOORD) ? inTexCoord.x : 0.0;
    float pile = sourceHas(source, SOURCE_CHARACTER_IN_TEXCOORD) ? inTexCoord.y : 0.0;

    vec3 albedo_wash;
    float texture_alpha;

    if (albedo_triplanar) {
        // Terrain's albedo and its detail normal come out of one packed texture
        // read; the relief strength is per-material hardness rather than a
        // single authored number, which is how chalk stays softer than rock.
        float relief = sourceHas(source, SOURCE_RELIEF_FROM_CHARACTER)
            ? reliefFromHardness(character)
            : materialGrainStrength();

        // Terrain that moves carries its texture with it: projected from its
        // own frame, then the detail normal turned into the world.
        bool anchored = sourceHas(source, SOURCE_ALBEDO_MODEL_SPACE);
        vec3 projected_pos = anchored ? inModelPos + materialProjectionAnchor() : inWorldPos;
        vec3 projected_normal = anchored && length(inModelNormal) > 0.001
            ? normalize(inModelNormal)
            : normal;

        TriplanarSurface field = triplanarSurface(
            texSampler,
            projected_pos,
            projected_normal,
            materialTriplanarScale(),
            materialTriplanarSharpness(),
            relief);

        albedo_wash = vec3(field.wash);
        texture_alpha = 1.0;
        normal = anchored ? normalize(materialModelToWorld() * field.normal) : field.normal;
    } else {
        vec4 texColor = texture(texSampler, inTexCoord);
        albedo_wash = texColor.rgb;
        texture_alpha = texColor.a;
    }

    // Relief: a texture baked from a pattern that cuts into the surface keeps
    // its height in alpha. The surface is opaque; the alpha is the shape.
    float relief_occlusion = 1.0;
    if (sourceHas(source, SOURCE_RELIEF_IN_ALPHA)) {
        normal = reliefNormal(
            texSampler,
            inTexCoord,
            texture_alpha,
            normal,
            inWorldPos,
            materialReliefDepth());
        relief_occlusion = reliefOcclusion(texture_alpha);
        texture_alpha = 1.0;
    }

    // Grain: microstructure from the shared atlas, for surfaces whose albedo
    // did not already carry a detail normal of its own. Terrain's arrives with
    // its albedo in one packed read and never reaches here.
    if (sourceHas(source, SOURCE_GRAIN_ANY)) {
        if (sourceHas(source, SOURCE_GRAIN_BY_UV)) {
            normal = grainByUv(
                grainSampler,
                normal,
                inTexCoord,
                materialGrainLayer(),
                materialGrainScale(),
                materialGrainStrength());
        } else {
            normal = grainObjectSpace(
                grainSampler,
                normal,
                materialModelToWorld(),
                inModelPos,
                inModelNormal,
                materialGrainLayer(),
                materialGrainScale(),
                materialGrainStrength(),
                GRAIN_BLEND_SHARPNESS);
        }
    }

    bool foliage = sourceHas(source, SOURCE_FOLIAGE);
    if (foliage) {
        normal = leafNormal(normal, inModelPos, materialModelToWorld());
    }

    vec3 view_dir = normalize(scene.camera_pos.xyz - inWorldPos);

    // A transmissive surface is drawn from both sides — the far half of a
    // block of ice is as visible as the near half — so a fragment here may be
    // a back face, whose outward normal points away from the camera. Shade it
    // from the side it is being seen from.
    //
    // Deliberately conditional on the material rather than applied to
    // everything: opaque geometry never shows a back face (it is culled), so
    // a normal pointing away from the camera there means a mesh with its
    // winding or its normals wrong, and quietly flipping it would hide that.
    if (materialOpacity() < 1.0 && dot(normal, view_dir) < 0.0) {
        normal = -normal;
    }

    // Terrain wears its material by its slope; nothing else carries the
    // character this reads.
    vec3 albedo = albedo_wash * inColor.rgb;
    if (sourceHas(source, SOURCE_CHARACTER_IN_TEXCOORD)) {
        float steepness = zoningSteepness(geometric_normal);
        float patch_value = zoningPatch(albedo_wash.r);
        albedo = zonedAlbedo(albedo, character, pile, steepness, patch_value);
        pile = zonedPile(pile, steepness, patch_value);
    }

    SurfaceSample surface;
    surface.albedo = albedo;
    surface.normal = normal;
    surface.view_dir = view_dir;
    surface.metallic = materialMetallic();
    surface.occlusion = inAo * relief_occlusion;

    // Roughness last, because the filtering reads the shading normal that the
    // detail perturbation has already been folded into. Applied to every
    // surface, not just terrain: geometric curvature aliases a tight highlight
    // just as detail normals do.
    //
    // A terrain chunk carries many materials but a draw carries one finish, so
    // terrain derives roughness per fragment from character instead.
    surface.roughness = filteredRoughness(
        sourceHas(source, SOURCE_ROUGHNESS_FROM_CHARACTER)
            ? roughnessFromHardness(character)
            : materialRoughness(),
        surface.normal);

    DirectionalLight sun;
    sun.direction = scene.sun_direction.xyz;
    sun.colour = scene.sun_colour.rgb;
    sun.intensity = scene.sun_direction.w;

    // Only the sun is shadowed. The sky's contribution arrives from the whole
    // hemisphere, so blocking it needs occlusion the shadow map cannot express;
    // that is what `surface.occlusion` carries, baked per vertex by
    // `terrain::ao` and applied inside shadeEnvironment.
    float sun_visibility = 1.0 - sunShadow(inWorldPos, surface.normal, sun.direction);

    // The sun's two lobes are kept apart only so that a transmissive surface
    // can let its highlight hold the pixel; an opaque surface sums them again
    // immediately and the split costs it nothing.
    LightResponse sun_response = respondToDirectional(surface, sun);
    vec3 sun_specular = sun_response.specular * sun_visibility;

    // The sky supplies both hemisphere irradiance and the reflection a glossy
    // or metallic surface shows. `ambient_colour` remains on top as an author's
    // fill for lifting a scene without moving the sky.
    vec3 litColor = (totalResponse(sun_response) * sun_visibility
                     + shadeEnvironment(surface, sun.direction, reflectedRadiance(surface, sun.direction))
                     + shadeAmbient(surface, scene.ambient_colour.rgb))
                      * pileDepthShade(surface, pile)
                  + materialEmissive()
                  + pileSunResponse(surface, sun, pile) * sun_visibility
                  + pileSkyResponse(surface, sun.direction, pile);

    // Leaves take the sun past the terminator and through their edges: light
    // the shadow map, which sees the clump as solid, would put them in its
    // own shadow for. Only part of that self-shadow is let through, so a
    // clump in a wall's shadow stays mostly in it.
    if (foliage) {
        vec3 sun_radiance = sun.colour * sun.intensity;
        float leaf_visibility = mix(FOLIAGE_SELF_SHADOW, 1.0, sun_visibility);
        litColor += (surface.albedo * sun_radiance
                         * leafWrapExtra(surface.normal, sun.direction) * surface.occlusion
                     + leafTransmission(surface.albedo, surface.normal, view_dir,
                                        sun.direction, sun_radiance))
                  * leaf_visibility;
    }

    // Bounded by the live light count, not just MAX_ACTIVE_LIGHTS, to skip
    // shading unused slots. But also clamped to MAX_ACTIVE_LIGHTS: `count` comes
    // from an external UBO write, and a renderer client that never calls
    // update_lights (e.g. bench_viewer) or a stale .spv with a mismatched
    // buffer size would otherwise let a garbage/oversized count drive an
    // unbounded loop that reads past the light array — a GPU hang.
    uint active_light_count = min(light_set.count, uint(MAX_ACTIVE_LIGHTS));
    for (uint i = 0u; i < active_light_count; ++i) {
        litColor += shadePoint(surface, light_set.lights[i], inWorldPos);
    }

    // Rim light reads as a glowing silhouette; tinted by the emissive colour so
    // it stays coherent with the object's own glow. Deliberately uses the
    // normalised emissive (materialEmissive()) rather than raw material.emissive.rgb,
    // so rim brightness is proportional to emitted luminance and stays
    // consistent across hues, rather than to the authored colour's magnitude.
    float rim_strength = materialRimStrength();
    if (rim_strength > 0.0) {
        float rim = fresnelRim(surface.normal, surface.view_dir, materialRimPower());
        litColor += materialEmissive() * rim * rim_strength;
    }

    // Coverage: how much of this pixel the surface claims. An opaque material
    // claims all of it and the two rules below collapse to the authored alpha,
    // which is why there is no branch here — a transmissive material is not a
    // different shading path, only a different number.
    float coverage = glassCoverage(
        materialOpacity(), materialReflectance(), surface.normal, surface.view_dir);

    // A highlight is light that bounced off the front and never went through,
    // so it survives however clear the body of the surface is.
    coverage = max(coverage, highlightCoverage(sun_specular));

    litColor = applyAerialPerspective(litColor, inWorldPos, scene.camera_pos, sun.direction);

    outColor = vec4(litColor, texture_alpha * inColor.a * coverage);
}
