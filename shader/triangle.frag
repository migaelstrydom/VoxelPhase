#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "scene.glsl"
#include "material.glsl"
#include "lighting.glsl"
#include "lights.glsl"
#include "environment.glsl"
#include "shadow.glsl"
#include "triplanar.glsl"
#include "surface_character.glsl"
#include "surface_source.glsl"
#include "grain.glsl"

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inTexCoord;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in vec3 inNormal;
layout(location = 4) in float inAo;
layout(location = 5) in vec3 inModelPos;
layout(location = 6) in vec3 inModelNormal;

layout(location = 0) out vec4 outColor;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

/// The shared grain atlas: RG is stone microstructure, BA is wood fibre, both
/// as tangent-space slopes. One texture for every material in the scene — see
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

    vec3 albedo_wash;
    float texture_alpha;

    if (albedo_triplanar) {
        // Terrain's albedo and its detail normal come out of one packed texture
        // read; the relief strength is per-material hardness rather than a
        // single authored number, which is how chalk stays softer than rock.
        float relief = sourceHas(source, SOURCE_RELIEF_FROM_CHARACTER)
            ? reliefFromHardness(character)
            : materialGrainStrength();

        TriplanarSurface field = triplanarSurface(
            texSampler,
            inWorldPos,
            normal,
            materialTriplanarScale(),
            materialTriplanarSharpness(),
            relief);

        albedo_wash = vec3(field.wash);
        texture_alpha = 1.0;
        normal = field.normal;
    } else {
        vec4 texColor = texture(texSampler, inTexCoord);
        albedo_wash = texColor.rgb;
        texture_alpha = texColor.a;
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

    SurfaceSample surface;
    surface.albedo = albedo_wash * inColor.rgb;
    surface.normal = normal;
    surface.view_dir = normalize(scene.camera_pos.xyz - inWorldPos);
    surface.metallic = materialMetallic();
    surface.occlusion = inAo;

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

    // The sky supplies both hemisphere irradiance and the reflection a glossy
    // or metallic surface shows. `ambient_colour` remains on top as an author's
    // fill for lifting a scene without moving the sky.
    vec3 litColor = shadeDirectional(surface, sun) * sun_visibility
                  + shadeEnvironment(surface, sun.direction)
                  + shadeAmbient(surface, scene.ambient_colour.rgb)
                  + materialEmissive();

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

    outColor = vec4(litColor, texture_alpha * inColor.a);
}
