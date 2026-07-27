#version 450
#extension GL_ARB_separate_shader_objects : enable

// Fullscreen triangle generated from gl_VertexIndex — no vertex buffer bound.
// Draw with vertexCount = 3. The oversized triangle is clipped to the viewport,
// which is cheaper than a quad and avoids a diagonal seam.

layout(location = 0) out vec2 outUv;

void main() {
    outUv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(outUv * 2.0 - 1.0, 0.0, 1.0);
}
