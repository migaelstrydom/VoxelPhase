#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

// The reflection probe capture: a cheap cut of triangle.frag.
//
// A probe face is 32 texels across and is seen blurred on a small object, so
// what it needs is each surface's colour under the sun and sky, not its
// detail. Kept: albedo (terrain's world-projected wash included, since the
// ground's colour is most of what a probe shows), sun with its shadow, sky
// light, emission. Dropped: grain, relief, point lights, rim, transparency.
//
// Reflections within the capture are of the sky alone. Sampling the probes
// here would read the array this pass is writing into.

#include "scene.glsl"
#include "material.glsl"
#include "lighting.glsl"
#include "environment.glsl"
#include "shadow.glsl"
#include "triplanar.glsl"
#include "surface_character.glsl"
#include "surface_source.glsl"
#include "probe_face.glsl"

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inTexCoord;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in vec3 inNormal;
layout(location = 4) in float inAo;
layout(location = 5) in vec3 inModelPos;
layout(location = 6) in vec3 inModelNormal;

/// Alpha 1: this texel hit something. See reflection.glsl.
layout(location = 0) out vec4 outColor;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

void main() {
    vec3 normal = length(inNormal) > 0.001 ? normalize(inNormal) : vec3(0.0, 1.0, 0.0);
    uint source = materialSource();
    float character = sourceHas(source, SOURCE_CHARACTER_IN_TEXCOORD) ? inTexCoord.x : 0.0;

    vec3 albedo_wash;
    if (sourceHas(source, SOURCE_ALBEDO_TRIPLANAR)) {
        // As in triangle.frag: terrain that moves carries its texture with it.
        bool anchored = sourceHas(source, SOURCE_ALBEDO_MODEL_SPACE);
        TriplanarSurface field = triplanarSurface(
            texSampler,
            anchored ? inModelPos + materialProjectionAnchor() : inWorldPos,
            anchored && length(inModelNormal) > 0.001 ? normalize(inModelNormal) : normal,
            materialTriplanarScale(),
            materialTriplanarSharpness(),
            0.0);
        albedo_wash = vec3(field.wash);
    } else {
        albedo_wash = texture(texSampler, inTexCoord).rgb;
    }

    SurfaceSample surface;
    surface.albedo = albedo_wash * inColor.rgb;
    surface.normal = normal;
    surface.view_dir = normalize(probe_face.eye.xyz - inWorldPos);
    surface.metallic = materialMetallic();
    surface.occlusion = inAo;
    surface.roughness = sourceHas(source, SOURCE_ROUGHNESS_FROM_CHARACTER)
        ? roughnessFromHardness(character)
        : materialRoughness();

    DirectionalLight sun;
    sun.direction = scene.sun_direction.xyz;
    sun.colour = scene.sun_colour.rgb;
    sun.intensity = scene.sun_direction.w;

    float sun_visibility = 1.0 - sunShadow(inWorldPos, surface.normal, sun.direction);

    vec3 lit = totalResponse(respondToDirectional(surface, sun)) * sun_visibility
             + shadeEnvironment(surface, sun.direction, reflectedSky(surface, sun.direction))
             + shadeAmbient(surface, scene.ambient_colour.rgb)
             + materialEmissive();

    outColor = vec4(lit, 1.0);
}
