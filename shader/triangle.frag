#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_GOOGLE_include_directive : require

#include "scene.glsl"
#include "material.glsl"
#include "lighting.glsl"

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inTexCoord;
layout(location = 2) in vec3 inWorldPos;
layout(location = 3) in vec3 inNormal;

layout(location = 0) out vec4 outColor;

layout(set = 1, binding = 0) uniform sampler2D texSampler;

void main() {
    if (material.colour_override.a > 0.0) {
        outColor = material.colour_override;
        return;
    }

    vec4 texColor = texture(texSampler, inTexCoord);

    SurfaceSample surface;
    surface.albedo = texColor.rgb * inColor.rgb;
    // Degenerate normals occur on some generated meshes; fall back to straight up.
    surface.normal = length(inNormal) > 0.001 ? normalize(inNormal) : vec3(0.0, 1.0, 0.0);
    surface.view_dir = normalize(scene.camera_pos.xyz - inWorldPos);
    surface.roughness = materialRoughness();
    surface.metallic = materialMetallic();

    DirectionalLight sun;
    sun.direction = scene.sun_direction.xyz;
    sun.colour = scene.sun_colour.rgb;
    sun.intensity = scene.sun_direction.w;

    vec3 litColor = shadeDirectional(surface, sun)
                  + shadeAmbient(surface, scene.ambient_colour.rgb)
                  + materialEmissive();

    // Rim light reads as a glowing silhouette; tinted by the emissive colour so
    // it stays coherent with the object's own glow.
    float rim_strength = materialRimStrength();
    if (rim_strength > 0.0) {
        float rim = fresnelRim(surface.normal, surface.view_dir, materialRimPower());
        litColor += material.emissive.rgb * rim * rim_strength;
    }

    outColor = vec4(litColor, texColor.a * inColor.a);
}
