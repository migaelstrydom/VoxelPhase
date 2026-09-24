// Ripple tiles on the GPU: one block per awake tile in a storage buffer,
// written by the CPU each frame. Mirrors src/water/surface/ripple_tiles.rs:
// 64 x 64 cells of 0.125 m per 8 m tile, heights at cell centres, then the
// floor under each of the tile's 16 x 16 columns (NaN where the tile's body
// holds no water).

const int RIPPLE_CELLS = 64;
const float RIPPLE_CELL = 0.125;
const int RIPPLE_COLUMNS = 16;
const float RIPPLE_COLUMN = 0.5;
const int RIPPLE_TILE_STRIDE = RIPPLE_CELLS * RIPPLE_CELLS + RIPPLE_COLUMNS * RIPPLE_COLUMNS;

layout(std430, set = 1, binding = 0) readonly buffer RippleTiles {
    float rippleData[];
};

float rippleCell(int layer, int i, int k) {
    i = clamp(i, 0, RIPPLE_CELLS - 1);
    k = clamp(k, 0, RIPPLE_CELLS - 1);
    return rippleData[layer * RIPPLE_TILE_STRIDE + k * RIPPLE_CELLS + i];
}

// Displacement at a tile-local position, bilinear between cell centres.
float rippleHeightAt(int layer, vec2 local) {
    vec2 f = local / RIPPLE_CELL - 0.5;
    ivec2 c = ivec2(floor(f));
    vec2 t = f - vec2(c);
    float a = mix(rippleCell(layer, c.x, c.y), rippleCell(layer, c.x + 1, c.y), t.x);
    float b = mix(rippleCell(layer, c.x, c.y + 1), rippleCell(layer, c.x + 1, c.y + 1), t.x);
    return mix(a, b, t.y);
}

vec2 rippleGradientAt(int layer, vec2 local) {
    float e = RIPPLE_CELL;
    return vec2(
        rippleHeightAt(layer, local + vec2(e, 0.0)) - rippleHeightAt(layer, local - vec2(e, 0.0)),
        rippleHeightAt(layer, local + vec2(0.0, e)) - rippleHeightAt(layer, local - vec2(0.0, e))
    ) / (2.0 * e);
}

// The floor under the column holding a tile-local position; NaN where the
// tile's body holds no water there.
float rippleFloorAt(int layer, vec2 local) {
    ivec2 c = clamp(ivec2(floor(local / RIPPLE_COLUMN)), ivec2(0), ivec2(RIPPLE_COLUMNS - 1));
    return rippleData[layer * RIPPLE_TILE_STRIDE + RIPPLE_CELLS * RIPPLE_CELLS
        + c.y * RIPPLE_COLUMNS + c.x];
}
