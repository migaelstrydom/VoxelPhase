# Water Rendering

This module draws every body of water from a static mesh. Meshes are built
from the hydrology network's basins (`src/water/`, see
`docs/WATER_HYDROLOGY_DESIGN.md` §15) when their topology changes, never per
frame; each draw pushes its body's current level.

---

## Pipeline architecture

Water is drawn inside the HDR scene pass, between the blended surfaces beyond
its surface and those this side of it. It refracts a copy of the scene, so the
pass is ended once for the copy and resumed:

```
scene pass                          resumed scene pass
+---------------------------+       +------------------------------+
| Sky, terrain, models      |       | Water (reads the copy)       |
| Blended + particles       | copy  | Blended + particles          |
|   beyond the water        |------>|   this side of the water     |
+---------------------------+       +------------------------------+
         colour, depth ──▶ RefractionCopy (only the water's footprint)
```

`WaterDivide` (`divide.rs`) sorts each blended draw and particle by the water
level under it: from above, what is under the surface is beyond it; from
below, the reverse. A mesh crossing the surface (an ice floe) is drawn in both
passes, each half clipped at the level through `gl_ClipDistance`
(`triangle.vert`'s `clipPlane`). So smoke over a lake covers it, the lake shows
through the dry part of a floe, and what is under the water is tinted and
refracted by it. Rivers and falls are not in the divide; what stands in them
counts as this side.

`WaterRenderer::prepare` syncs meshes, uploads ripples and plans the frame's
draws, leaving out tiles outside the view. A frame with no water in view is
never split. `ScreenFootprint` (`footprint.rs`) projects the planned tiles'
boxes, and the copy is limited to that rectangle plus the refraction's reach;
a river or fall, whose draws carry no bounds, widens it to the whole screen.

The water writes scene radiance like everything else in the pass; the resolve
tonemaps it with the rest of the scene.

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
frame: per awake tile, 32 × 32 heights with a two-cell apron (36 × 36),
then the floor under each of its 16 × 16 columns, NaN where the tile's body
holds no water, then the floor at each of its 17 × 17 column corners. The
apron reaches as far past an edge as the normal's central difference does,
so two awake tiles take the same slope, not just the same height, along
their shared edge; otherwise the crease shows in the sun's glare. The fragment
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

The fragment shader reads the depth behind it from the refraction copy:

```glsl
float terrainDepthRaw = texture(depthSampler, screenUV).r;
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
    mod.rs          -- re-exports
    basin_mesher.rs -- basin meshes and the WaterScene trait
    ocean_mesher.rs, ocean_ring.rs, reach_mesher.rs, fall_mesher.rs
    divide.rs       -- which side of the water a blended surface lies on
    footprint.rs    -- the screen rectangle the water can read
    pipeline.rs     -- Vulkan pipelines, descriptor set for the refraction copy
    renderer.rs     -- mesh sync, frame plan, draw recording
    vertex.rs       -- vertex layouts
```

Shaders:
```
shader/water.vert, ripple.vert, river.vert -- surfaces
shader/water.frag  -- Fresnel, specular, volumetric depth, refraction
shader/fall.vert, fall.frag -- a fall's sheet
```
