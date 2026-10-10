# Water Rendering

This module draws the game's water: the shallow-water field's surface
(`src/water/`, see `docs/WATER_FIELD_SPIKE.md`), one surface per 8 m chunk
whose vertices carry the water's own height, and the sea's ring past the
map's open edges, a still surface whose level is pushed per draw.

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
refracted by it. The divide holds the still draws (the sea's ring, a bench's pool) and every
8 m tile of the field's water that stands still (`WaterScene::still_water`).
Running water is not in it, so what stands in a river counts as this side.

`WaterRenderer::prepare` syncs meshes, uploads ripples and plans the frame's
draws, leaving out tiles outside the view. A frame with no water in view is
never split. `ScreenFootprint` (`footprint.rs`) projects the planned tiles'
boxes, and the copy is limited to that rectangle plus the refraction's reach;
a river surface, whose draws carry no bounds, widens it to the whole screen.

The water writes scene radiance like everything else in the pass; the resolve
tonemaps it with the rest of the scene.

## Meshes

Anything implementing `WaterScene` (`scene.rs`) can be drawn: the game's
`WaterWorld` through `FieldScene`, and the visual bench's fixed `ScenePool`.
It offers two kinds of surface:

- **Still surfaces** (`WaterMesh`): one quad per column, grouped into 8 m
  tiles, one draw per (body, tile). The level and swell are push constants,
  so a level change needs no new mesh; the mesh is built again only when its
  key changes. The sea's ring (`ocean_ring.rs`) is one.

  ```
  x, z = column corners (0.5 m lattice)
  y    = the body's level, pushed per draw
  floor (per vertex) = the lowest floor of the column's span
  ```

- **River surfaces** (`RiverMesh`, `river.vert`): every vertex at the water's
  own height, with its depth (negative over a bank, so the shore depth cuts
  a contour) and its flow, one draw per surface.

## The field

The game's water is a shallow-water field (`docs/WATER_FIELD_SPIKE.md`), drawn
through the river pipeline by `field_mesher.rs`: one surface per 8 m chunk,
keyed by a version the water moves only when that chunk's depths have moved
by more than a millimetre. `RiverParts` keeps each river surface apart. One
whose version moved is read again in place every frame: each vertex records
the cells it was built from (`field_sources.rs`), so while the water keeps
its shape (no cell going wet or dry, no group of surfaces splitting or
joining, no edit relaying the cells) the heights, depths and flow are read
again without the graph. Only a surface whose shape changed is built again,
at most eight a frame in the game (stalest first; the offline viewers lift
the cap). The surfaces are then joined for upload. Past the map's open edges
the sea is the ocean ring.

## Swell and ripples

The vertex shader adds the body's swell (`swell.glsl`, a sum of five sines
shared with the CPU so buoyancy floats bodies on the drawn surface), faded
out over the last metre of depth using the vertex's floor. A river surface
(`river.vert`) takes the sea's swell by how near sea level its water stands
(`water::waves::swell_share`), so lakes above the sea stay still.

The field's ripple tiles wake only over still water, and each is drawn on
its chunk's own river surface (`RippleTileView::surface`): `river.vert`
lifts each vertex by the tile's ripples and `water.frag` takes their normal
per pixel, so a rippling chunk is the same surface as its neighbours.

A still draw's tile (a bench's pool) is not drawn from the coarse mesh. Instead a
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
| 128    | 16   | Vertex   | Body: level, swell amplitude, phase, clock — per draw; a river surface: shore depth, —, —, clock |
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
    scene.rs        -- the WaterScene trait and still-surface meshes
    river_mesh.rs   -- river-surface meshes
    field_mesher.rs -- the field's surface, per chunk
    field_sources.rs -- where each vertex of the field's surface reads its water
    river_parts.rs  -- river surfaces read again or rebuilt when their version moves
    ocean_ring.rs   -- the sea past the map's open edges
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
```
