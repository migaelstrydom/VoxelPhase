#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

// Fullscreen triangle: 3 vertices cover the entire screen
// Vertex IDs 0,1,2 generate positions that form a triangle larger than the viewport

// We pass clip position to fragment shader for per-fragment ray computation
layout(location = 0) out vec2 fragClipPos;

void main() {
    // Generate fullscreen triangle vertices from vertex ID
    vec2 uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    vec2 ndc = uv * 2.0 - 1.0;

    gl_Position = vec4(ndc, 0.0, 1.0);

    // Pass clip position to fragment shader
    fragClipPos = ndc;
}
