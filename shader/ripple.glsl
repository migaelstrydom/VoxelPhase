// Ripple tiles on the GPU: one block per awake tile in a storage buffer,
// written by the CPU each frame. Mirrors src/water/surface/ripple_tiles.rs:
// 64 x 64 cells of 0.125 m per 8 m tile, heights at cell centres with a
// one-cell apron (66 x 66), then the floor under each of the tile's 16 x 16
// columns (NaN where the tile's body holds no water), then the floor at each
// of its 17 x 17 column corners (NaN where no wet column touches it).

const int RIPPLE_CELLS = 64;
const int RIPPLE_PADDED = RIPPLE_CELLS + 2;
const float RIPPLE_CELL = 0.125;
const int RIPPLE_COLUMNS = 16;
const int RIPPLE_CORNERS = RIPPLE_COLUMNS + 1;
const float RIPPLE_COLUMN = 0.5;
const int RIPPLE_FLOORS_AT = RIPPLE_PADDED * RIPPLE_PADDED;
const int RIPPLE_CORNERS_AT = RIPPLE_FLOORS_AT + RIPPLE_COLUMNS * RIPPLE_COLUMNS;
const int RIPPLE_TILE_STRIDE = RIPPLE_CORNERS_AT + RIPPLE_CORNERS * RIPPLE_CORNERS;

layout(std430, set = 1, binding = 0) readonly buffer RippleTiles {
    float rippleData[];
};

// A cell's height; -1 and RIPPLE_CELLS reach into the apron.
float rippleCell(int layer, int i, int k) {
    i = clamp(i, -1, RIPPLE_CELLS);
    k = clamp(k, -1, RIPPLE_CELLS);
    return rippleData[layer * RIPPLE_TILE_STRIDE + (k + 1) * RIPPLE_PADDED + i + 1];
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
    return rippleData[layer * RIPPLE_TILE_STRIDE + RIPPLE_FLOORS_AT + c.y * RIPPLE_COLUMNS + c.x];
}

// The floor at a column corner, as the coarse surface takes it there.
float rippleCornerFloor(int layer, ivec2 corner) {
    corner = clamp(corner, ivec2(0), ivec2(RIPPLE_CORNERS - 1));
    return rippleData[layer * RIPPLE_TILE_STRIDE + RIPPLE_CORNERS_AT
        + corner.y * RIPPLE_CORNERS + corner.x];
}
