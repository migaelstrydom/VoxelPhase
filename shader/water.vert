#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// Vertex inputs: the surface's plan position and the floor beneath it. The
// height is the body's level, pushed per draw, so a level change needs no new
// mesh.
layout(location = 0) in vec2 inXz;
layout(location = 1) in float inFloor;

layout(push_constant) uniform PushConstants {
    mat4 view;
    mat4 proj;
    vec4 body;   // (level, unused, unused, unused), per draw
} pc;

// Outputs to fragment shader
layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragWorldPos;

void main() {
    vec3 position = vec3(inXz.x, pc.body.x, inXz.y);
    gl_Position = pc.proj * pc.view * vec4(position, 1.0);
    fragNormal = vec3(0.0, 1.0, 0.0);
    fragWorldPos = position;
}
