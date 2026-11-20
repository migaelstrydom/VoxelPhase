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

Surfaces replace the pixel and write depth; they composite with the copy
themselves (refraction, absorption, their soft edges). Falls' curtains and
jets are drawn after every surface by a second pipeline that blends by alpha
and writes no depth (`Composite` in `pipeline.rs`), so a curtain of strands
shows the pool it falls into, which the copy does not hold. It culls back
faces: falling water is a body with a top face and an under face, and only
the one turned to the eye is drawn. Falls side by side, or one inside
another's arc, would be blended in the order they come, a far one over a
near one; so first a depth-only pass (`SHEET_DEPTHS` in `water.frag`) lays
the depth of every curtain where it hides at least `SHEET_SOLID` of what is
behind it, and only the nearest solid water is blended there. It reads only
how much the water hides (`sheetCover`), not how it is lit, so it costs a
fraction of the blended pass, and saves as much again in falls not shaded
behind it. Clearer water hides nothing and is blended in order, as before.

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
  a contour) and its flow, one draw per surface. A surface's falling sheets
  are indexed apart (`sheet_indices`); `RiverParts` uploads every surface's
  triangles, then every sheet's, and each surface has a draw for each.

## The field

The game's water is a shallow-water field (`docs/WATER_FIELD_SPIKE.md`), drawn
through the river pipeline by `field_mesher.rs`: one surface per 8 m chunk,
keyed by a version the water moves only when that chunk's depths have moved
by more than can be seen: a millimetre, or two parts in a thousand of the
water's depth where that is more, up to half a centimetre
(`water/field/versions.rs`). Deep water simmers by millimetres wherever
anything runs into it, and reading every such surface again each frame was
most of what drawing water_park's rivers cost (4.1 ms a frame of 6.9 at
rest, against 2.2 of 4.1 now). What a chunk tells its neighbours of the
cells along its border keeps to the millimetre, so their surfaces still
meet. `RiverParts` keeps each river surface apart. One
whose version moved is read again in place every frame: each vertex records
the cells it was built from (`field_sources.rs`), so while the water keeps
its shape (no cell going wet or dry, no group of surfaces splitting or
joining, no edit relaying the cells) the heights, depths and flow are read
again without the graph. Only a surface whose shape changed is built again,
at most eight a frame in the game (stalest first; the offline viewers lift
the cap). The surfaces are then joined for upload. Past the map's open edges
the sea is the ocean ring.

Where the field's water falls (`water::field::Fall`: one per face it goes
over, with the velocity it leaves at and the arc it flies down to where it
lands), the mesher hangs a curtain from the cell's edge (`curtain.rs`). The
field decides where water falls and where it comes down, and pours it there;
the mesher only draws it.

```
        end 0      middle      end 1       three strands down each fall: its
   lip ───●──────────●──────────●───       own arc from the middle of its lip,
          ┃╲         ┃╲         ┃╲         and at each end the mean of its arc
          ┃ ╲        ┃ ╲        ┃ ╲        and the arc of the fall its lip runs
   foot ~~┃~~●~~~~~~~┃~~●~~~~~~~┃~~●~~     on into (`WaterField::fall_beside`)
```

Two falls side by side share the strand between them, straight on or round a
corner, so the falls of one lip are one watertight body, and a lip running
across the lattice (a staircase of edges facing two ways) is a straight
curtain from its second row. Its first row hangs from the surface: the middle
of each edge, the cell's corners, and where the water goes over both edges at
a corner, the middle of the cut that takes the corner off (`lay_cell`).
Where the lower column's own floor carries the ledge on, the surface is
carried out over it to where the water really leaves.

Each strand's top face follows its arc in `CURTAIN_ROWS` rows; its under face
lies below by the water's thickness, the flow over the lip by its speed,
thinning as it falls. A free end is closed by a cap between the two faces,
so thick water is seen to be thick from its side, and frays out instead
where the water is thin (`fray`). Water driven out of a hole (`Fall::driven`)
is a jet: its body starts back inside the hole, it is white from the lip
(`air`), and the cell it bursts from draws no surface, nor lifts the corners
of the cells beside it towards its roof (`DrawnCells::drawn`). A wall in an
arc's way is left to hide the rest of it.

A jet fills the hole it bursts out of, however thin it flies (`Fall::opening`):
its body is drawn from the hole's floor to its roof behind the mouth and
draws in to its own thickness over the first metre of its flight
(`Tongue::plug`), or the far end of the hole showed over its top. One drowned
by the water outside (`Fall::drowned`) is drawn the same for a short way on
into that water, so the hole's mouth is closed over the pool for as long as
its top stands clear; before, the far side showed through a hole half under
water.

A fall whose lip lies well under its cell's surface (`SUNK_LIP`) leaves by an
opening in the water's side: a slot at the foot of a lake. Its curtain hangs
from its own arc, as a jet's out of a hole does, and the surface over it is
laid as if nothing went over that edge. Hung from the surface it was a sheet
from the top of the lake down to the slot.

Water under a roof and water standing higher than that roof beside it are
two sheets of water, one over the roof, and no corner joins them
(`Meeting::roof`), though the face between them is open: a pocket under a
slab beside a lake. Joined, the pocket's surface ramped up to the lake's.

Water run full in a hole between two waters (a conduit's full cells) has no
surface of its own, but is drawn: at the head it is under
(`DrawnCells::surface`), which falls along the hole from the level behind it
to the level it runs out into. Its corners weigh nothing in the corners they
share (`Meeting::weight`), so they take the water's level on either side and
pull none of it down. The surface lies over the cell's roof, where the rock
hides it wherever the roof is really there; without it the water behind and
the water in front each stopped flat at the roofed column and left an open
step between them. Any other cell filled to its roof (a sealed cavity, the
water under a slab) draws none.

A fall has a life (`Fall::age`, `Fall::spent`): new, its water has fallen
only as far as its age lets it; once it stops pouring, what is in the air
goes on down. Its vertices carry when it began and when it stopped, on the
water's clock (`life`), and the draw shows only the stretch between its first
water and its last, torn at both ends. A spent fall's curtain hangs from its
own arc, since the surface it left is no longer shaped by it, runs on only
into other spent falls, is left as built, and is folded to a point once its
fall is gone, so its going builds nothing again. Nothing is read again as a
fall ages: the clock does it.

Seen from below its lip, a thick fall shows its under face near the top and
its top face lower down. The under face is white from the lip and lit
through from above (`SHEET_THROUGH`), and neither face mirrors the sky whole
where it is seen edge on (`SHEET_MAX_MIRROR`), or the line where one hands
over to the other is drawn across the fall as a bright wire.

A curtain's vertices carry `sheet` = (drop below the lip, whole fall, side,
thickness), the `normal` of their face, where their water left its lip (in
`flow`), the way it flies (in `face`) and its fall's `life`. A curtain is read again as a whole
from its fall (`SurfaceSources::curtains`); the surface's shape holds while
its chunk's falls and the falls they run on into are the same.

A source's jet (`jet.rs`) is a tube of `JET_ROWS` rings of `JET_RING`
vertices round its path (`water::falling::FallPath`, the jet the physics
strikes bodies with) down to the water it lands in, its section carrying
the discharge at the water's speed there, spreading and flattening as it
falls. It is built into the chunk it lands in; refreshing it recomputes the
arc to the landing water's level.

## Swell and ripples

The vertex shader adds the body's swell (`swell.glsl`, a sum of five sines
shared with the CPU so buoyancy floats bodies on the drawn surface), faded
out over the last metre of depth using the vertex's floor. A river surface
(`river.vert`) takes the sea's swell by how near sea level its water stands
(`water::waves::swell_share`), so lakes above the sea stay still.

The field's ripple tiles wake on still and running water alike (the current
carries them, `water/surface/ripple_tiles.rs`), and each is drawn on
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

### 5. Soft edges

Shorelines are found per pixel, not meshed. Each wet cell's surface runs on
as a skirt over the dry columns beside it (`FieldScene::skirt`), each quarter
of a column laid by one wet cell so skirts never overlap. A film's skirt
(under `FILM_DEPTH`) runs its depth out to nothing across the dry column, so
a front's thin edge ends there even where the floor ahead is level with it
and no bank rises to meet it. The terrain hides
the skirt wherever the ground stands above the water. `water.frag` reads how
far below the surface the ground behind each pixel lies from the scene's
depth copy (`groundDepth`, which needs the 32-bit depth buffer), and how
steeply it rises (`groundSlope`, from the neighbouring pixels on the side
nearer in depth, so a silhouette in front of the water is not read as a
slope). Where the field has the water shallow (`GROUND_TRUSTED_DEPTH`) it
takes that as the water's depth where it is shallower, so a beach's edge
fades to the pixel. Where the field has it deep it keeps the field's depth:
whatever stands shallow in deep water is a body, which stays under the
water, and a wall cuts the water with only the contact softness below.
Shallow water lies on the ground, so where the ground behind it lies well
below the field's depth (`LIFTED_GAP`) the surface is in the air and is not
drawn: a film on a bank steeper than one quad per column can follow stands
its quad out over the slope below it. Surfaces with no field depth of their
own (sea and ripple tiles) are taken as deep.

A skirt can lie over a dry hollow beside the water only where a ridge
thinner than a column parts them; it is not dropped there.

Wherever a surface thins out it fades into the copy rather than being cut:

- past the shore depth, over `SHORE_SOFTNESS` of ground (the depth's
  gradient per pixel turns it into depth), so a shoreline does not follow its
  triangles; films thinner than `SHORE_SOFTNESS_DEPTH` read as wet ground;
- over the first `CONTACT_SOFTNESS` of view ray through the water, where it
  meets a bank, a body or the floor under a thin film.

Both edges wander with noise near an edge (`FOAMY_DEPTH`), so they lap
rather than draw lines. The edge's froth fades out as the ground under it
steepens (`FROTHING_SLOPE`): a beach froths, a wall does not.

### 6. Foam

Foam is froth: patches of noise carrying a lace of drifting cells
(`foamLace`), carried by the flow. Where water runs white the lace is drawn
out along the current (`streakedLace`): in fixed frames an eighth of a turn
apart, the two nearest the flow faded into each other, because a frame turned
to the flow at each pixel shears the pattern wherever the flow turns. It frills
the edge where water thins over a beach or an advancing front (measured on
the ground, so it never outlines a wall), and covers water running faster
than `WHITE_WATER_SPEED`. It is worked out only where it can show.

The field also keeps foam of its own (`water::field::foam`): made where falls
land, where fast water runs into slower and at a surge's front, carried with
the water and fading over a few seconds. Each corner of the surface carries
its cells' (in `air`, which on a curtain is the air in the falling water),
and the shader lays it as billows that close up where it is thick
(`RIDING_FOAM_FULL`) and break into patches as it thins, laced inside, over
water turned pale by the air churned into it (`AERATED_COLOR`), which follows
the billows too, so its edge is not the straight edge of the cells that hold
it. It moves with the current because the billows' noise is carried by the
flow, like the lace. A chunk whose foam changed by more than can be seen has
its version moved, and where the change reaches its border, so have those
round it (`WaterWorld::note_foam`).

### 7. Falling water

Falling water (`shadeSheet`) is clear glass as it leaves the lip, as deep
and as opaque as it is thick, turning to white water over `SHEET_GLASS_DROP`
(at once, where it left its lip already white: `air`): two layers of streaks
falling at their own speeds, with air between them, sparser where little
pours. The streaks hang from where the water left its lip and are laid along
whichever of the world's axes the curtain runs along (both, faded into each
other, between them), so they run straight down however the curtain is thrown
and on from each fall to the next round any lip. Their slope tilts the face
they lie on (`embossed`), so the ropes of white water stand out of the
curtain; seen edge on they are left out before they alias. It is lit by its
face's normal, wrapped (`SHEET_WRAP`), breaks up over its foot, frays out at
the frayed ends of a fall and fades where it runs against rock.

Where falling water lands, a patch of foam lies on the water round its foot
(`curtain::Strand::plunge`, `Jet::plunge`): from behind the foot, under the
fall, to out past it. Its vertices' drop is past their fall by 1 and where
across the patch they lie, which is how the shader tells foam (`shadePlunge`)
from the fall itself: solid at the foot, in billows further out, gone before
its edge. Spray and mist are particles, thrown where the falls land
(`systems/fall_spray.rs`), on a fixed budget shared by every fall.

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
    audit.rs        -- a drawn mesh checked against its water: nothing drawn where none is
    ocean_ring.rs   -- the sea past the map's open edges
    curtain.rs      -- a fall's curtain: strands, top and under faces, caps, foam
    jet.rs          -- a source's jet, drawn as a tube of falling water
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
