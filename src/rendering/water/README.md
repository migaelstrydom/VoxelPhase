# Water Rendering

This module draws every body of water from a static mesh. Meshes are built
from the hydrology network's basins (`src/water/`, see
`docs/WATER_HYDROLOGY_DESIGN.md` §15) when their topology changes, never per
frame; each draw pushes its body's current level.

---

## Pipeline architecture

Water renders in **subpass 1** (transparent) of the main render pass. Subpass 0
(opaque) draws terrain, models, and debug overlays, writing the depth buffer.
Subpass 1 then reads that depth buffer as a Vulkan **input attachment**, giving
the water fragment shader access to the terrain depth behind each water pixel.

```
Subpass 0 (opaque)          Subpass 1 (transparent)
+-----------------------+   +-----------------------+
| Sky                   |   | Water                 |
| Terrain        depth >--->| Particles             |
| Models         write  |   | Overlay               |
| Debug overlays        |   |           depth read   |
+-----------------------+   +-----------------------+
```

The depth buffer is created with `DEPTH_STENCIL_ATTACHMENT | INPUT_ATTACHMENT`
usage flags. In subpass 1, the depth attachment layout is
`DEPTH_STENCIL_READ_ONLY_OPTIMAL`, which allows both depth testing (water is
occluded by terrain) and fragment shader reads (volumetric depth calculation).

A subpass dependency ensures all depth writes from subpass 0 complete before
fragment shader reads in subpass 1.

## Meshes

`BasinMesher` builds one quad per region column of every basin, plus a ring of
columns just outside it whose ground stands above the water. Columns beyond a
crest, lower than the water, are left out, so a surface never hangs over the
hillside below an outlet. The depth test against the terrain cuts the
shoreline.

```
x, z = column corners (0.5 m lattice)
y    = the body's level, pushed per draw
floor (per vertex) = the lowest floor of the column's span
```

Quads are grouped into 8 m tiles, one draw per (basin, tile). The mesh key is
each basin's id and region version: a re-region, merge or split changes it and
the mesh is rebuilt, then uploaded once into each frame slot's buffers. A
level change is a push constant.

Anything implementing `WaterScene` can be drawn: `WaterWorld`, and the visual
bench's fixed `ScenePool`.

## Swell and ripples

The vertex shader adds the body's swell (`swell.glsl`, a sum of five sines
shared with the CPU so buoyancy floats bodies on the drawn surface), faded
out over the last metre of depth using the vertex's floor.

A tile whose ripples are awake is not drawn from the coarse mesh. Instead a
static 65 × 65 grid over the 8 m tile (`ripple.vert`) is displaced by the
tile's ripple heights, read from a storage buffer (set 1) the CPU fills each
frame: per awake tile, 64 × 64 heights then the floor under each of its
16 × 16 columns, NaN where the tile's body holds no water. The fragment
shader discards the grid outside the body's columns. One buffer and one
descriptor set per frame slot, both allocated once.

## Push constants

| Offset | Size | Stage    | Contents                                  |
|--------|------|----------|-------------------------------------------|
| 0      | 64   | Vertex   | View matrix (mat4)                        |
| 64     | 64   | Vertex   | Projection matrix (mat4)                  |
| 128    | 16   | Vertex   | Body: level, swell amplitude, phase, clock — per draw |
| 144    | 16   | Vertex   | Tile: origin x, origin z, ripple layer — fine draws only |
| 160    | 16   | Fragment | Camera position (vec3 + padding)          |
| 176    | 16   | Fragment | Sun direction (vec3 + padding)            |
| 192    | 16   | Fragment | Near, far planes, time                    |
| 208    | 16   | Fragment | Screen size, hue preservation, exposure   |

The near and far planes are extracted from the projection matrix at runtime:

```
near = proj[3][2] / proj[2][2]
far  = proj[3][2] / (proj[2][2] + 1)
```

These are needed by the fragment shader to linearize depth buffer values.

---

## Fragment shader effects

### 1. Volumetric depth (depth buffer sampling)

The key visual effect: water colour and opacity depend on the optical path
length through the water volume, not just the vertical water column height.

The fragment shader reads the opaque geometry depth from the input attachment:

```glsl
float terrainDepthRaw = subpassLoad(depthInput).r;
float waterDepthRaw   = gl_FragCoord.z;
```

Both are non-linear depth buffer values in [0, 1]. To compute the actual
distance through water, they must be linearized to view-space distances.

#### Depth linearization

A Vulkan perspective projection maps view-space depth `z_view` to clip depth:

```
z_clip = (far * z_view + near * far) / (z_view * (near - far))
z_ndc  = z_clip  (Vulkan depth range is [0, 1])
```

Inverting this:

```
z_view = near * far / (far - z_ndc * (far - near))
```

The optical path length is then:

```
optical_depth = z_view(terrain) - z_view(water_surface)
```

This is the distance light travels through the water column along the view ray.
At glancing angles the path is much longer than the vertical depth, making
shallow water at the far side of a pond appear darker and more opaque than the
same depth viewed from directly above.

#### Exponential absorption

Real water absorbs light exponentially with distance (Beer-Lambert law):

```
transmittance = e^(-absorption * distance)
```

The shader uses `depth_factor = 1 - e^(-optical_depth * 0.5)` as a [0, 1]
absorption factor that drives both colour and opacity. The coefficient 0.5
gives visible colour at ~1m depth and near-full absorption at ~5m.

### 2. Fresnel reflectance

Water surfaces reflect more light at glancing angles and transmit more when
viewed straight on. This is the Fresnel effect, approximated with the Schlick
formula:

```
F = F0 + (1 - F0) * (1 - cos(theta))^5
```

where:
- `F0` is the reflectance at normal incidence (0.02 for water, derived from
  IOR 1.33 via `F0 = ((n-1)/(n+1))^2`)
- `theta` is the angle between the surface normal and the view direction
- `cos(theta) = dot(N, V)`

At normal incidence (looking straight down), F = 0.02 = 2% reflection. At
grazing angles, F approaches 1.0 = 100% reflection.

The reflected colour is approximated from the reflected view direction:

```
R = reflect(-V, N)
colour = mix(horizon_colour, zenith_colour, max(R.y, 0))
```

This gives bright horizon reflections at shallow angles and blue sky reflections
when looking down at rippled surfaces.

The final colour blends the water body colour with the reflection:

```
colour = mix(water_colour, reflection_colour, F)
```

### 3. Specular highlights

Sun specular is computed with the Blinn-Phong model:

```
H = normalize(L + V)           // half-vector
spec = max(dot(N, H), 0)^256   // sharp highlight
```

The high exponent (256) produces tight, bright glints on wave crests. The
specular term is additive (not multiplied by the base colour) so highlights
remain white regardless of the water tint.

The half-vector `H` is the direction exactly between the light and view
directions. When the surface normal aligns with `H`, the viewer sees a direct
specular reflection of the sun. Ripples from the wave simulation cause the
normal to vary across the surface, breaking the specular into many small
glints rather than a single blob.

### 4. Diffuse lighting

Basic Lambertian diffuse modulates the overall brightness:

```
NdotL = max(dot(N, L), 0)
colour *= 0.5 + 0.5 * NdotL
```

The 0.5 ambient floor prevents water from going fully black on surfaces facing
away from the sun.

---

## Colour palette

| Role            | RGB                | Notes                              |
|-----------------|--------------------|------------------------------------|
| Shallow water   | (0.20, 0.55, 0.50) | Light turquoise                   |
| Deep water      | (0.02, 0.12, 0.22) | Dark navy                         |
| Horizon reflect | (0.60, 0.70, 0.80) | Bright grey-blue                  |
| Zenith reflect  | (0.25, 0.40, 0.70) | Mid blue                          |
| Sun specular    | (1.00, 0.95, 0.80) | Warm white                        |

Opacity ranges from 0.15 (shallow, transparent edge) to 0.9 (deep, nearly
opaque).

---

## Module structure

```
src/rendering/water/
    mod.rs       -- re-exports
    pipeline.rs  -- Vulkan pipeline, descriptor set for depth input attachment
    renderer.rs  -- mesh generation, draw call, push constant upload
    vertex.rs    -- WaterVertex (position + normal), vertex input layout
```

Shaders:
```
shader/water.vert  -- transforms vertices, passes normal + world pos
shader/water.frag  -- Fresnel, specular, volumetric depth, alpha blend
```
