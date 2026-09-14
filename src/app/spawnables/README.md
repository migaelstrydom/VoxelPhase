# Spawnables

Spawnables are prefab objects that can be placed in level files (`.level.ron`).
Each spawnable owns its geometry, materials, and physics setup.

## Adding a new spawnable

### 1. Create the definition struct

Create `src/app/spawnables/my_thing.rs` with a `Deserialize` struct and a
`Spawnable` impl. The struct fields become the parameters available in level
files. Use `#[serde(default = "...")]` for optional parameters.

```rust
#[derive(Deserialize)]
pub struct MyThingDef {
    pub pos: (f32, f32, f32),
    #[serde(default = "MyThingDef::default_size")]
    pub size: f32,
}
```

The `Spawnable` trait (in `spawnable.rs`) has three methods:

- `material_count()` — how many materials this spawnable needs (called during
  pre-allocation).
- `create_materials(ctx)` — generate procedural textures and register materials
  via `ctx.textures` and `ctx.materials`. Must return exactly `material_count()`
  entries.
- `spawn(world, materials)` — create ECS entities and physics bodies. The
  `materials` slice contains the IDs returned by `create_materials`.

### 2. Register in `mod.rs`

Add `mod my_thing;` and `pub use my_thing::MyThingDef;` to
`src/app/spawnables/mod.rs`.

### 3. Add the level-file variant

In `src/level/data.rs`:

1. Add `MyThingDef` to the `use crate::app::spawnables::{...}` import.
2. Add a variant to the `LevelObject` enum with `#[serde(default)]` attributes
   matching your struct's defaults.
3. Add a conversion arm in `LevelObject::to_spawnable()` that constructs your
   `MyThingDef`.

### 4. Place it in a level

```ron
MyThing(
    pos: (5.0, 0.0, 3.0),
    size: 2.0,
),
```

## Required imports

Every spawnable needs a subset of these (copy what you need):

```rust
use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, convex_solid_model, SolidFace};
use super::shared::textures::Rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    Flammable, ModelInstance, Orientation, Position, Renderable, RigidBodyComponent,
    TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::utils::noise::fbm_2d_periodic;
```

## Geometry and models

### Shared model builders (`shared/models.rs`)

- `cuboid_model(half_extents, material)` — single axis-aligned box.
  Returns `Arc<Model>`.
- `compound_cuboid_model(boxes, material)` — multiple boxes, one material.
  `boxes` is `&[(Vector3<f32>, MaterialId)]` — each entry is `(half_extents, offset)`.
- `multi_material_compound_cuboid_model(boxes)` — multiple boxes, per-box
  materials. `boxes` is `&[(Vector3<f32>, Vector3<f32>, MaterialId)]` —
  `(half_extents, offset, material)`.
- `multi_material_rotated_compound_cuboid_model(boxes)` — same but with per-box
  rotation quaternions. `boxes` is
  `&[(Vector3<f32>, Vector3<f32>, UnitQuaternion<f32>, MaterialId)]`.
- `convex_solid_model(vertices, faces, material)` — arbitrary convex shape from
  vertex + face definitions. Uses per-face planar UV projection.
  `vertices: &[Vector3<f32>]`, `faces: &[SolidFace]`, `material: MaterialId`.
  Returns `Arc<Model>`.
- `build_convex_hull(vertices, faces)` — builds a `ConvexHull` for the physics
  collider from the same vertex/face data. Returns `ConvexHull` (wrap in
  `Arc::new()` for `ColliderDesc::convex_hull()`).

All model functions return `Arc<Model>`.

### Prefer boxes over convex hulls

Use `ColliderDesc::box_shape` + `cuboid_model` when the shape is a box. Box
colliders are cheaper than convex hulls in the narrowphase and solver. Only use
`ColliderDesc::convex_hull` for genuinely non-box shapes (columns, wedges,
arches, polyhedra).

### Face definitions for convex shapes

`SolidFace` defines a face by vertex indices plus an `opposite_vertex` — the
index of any vertex known to be on the interior side of the face. This is used
to ensure outward-facing normals and correct winding.

```rust
SolidFace {
    vertex_indices: vec![0, 1, 2, 3],  // CCW when viewed from outside
    opposite_vertex: 5,                 // any vertex behind this face
}
```

### Triangle winding order

The Vulkan pipeline is configured with:
- **Front face:** `COUNTER_CLOCKWISE` (in clip space)
- **Cull mode:** `BACK`

Because the projection matrix flips Y for Vulkan, **vertices that appear
clockwise in world space become counter-clockwise in clip space**. This means:

- The cube index pattern per quad (vertices 0-3) is: `[0, 2, 1, 0, 3, 2]`
- When building custom meshes, match this convention or triangles will be
  invisible (back-face culled).
- `convex_solid_model` handles winding automatically via the `opposite_vertex`
  mechanism — you don't need to worry about it when using `SolidFace`.
- If you build a custom mesh (like the temple's fluted column model with
  cylindrical UVs), you must get the winding right yourself.

### Custom mesh building

For shapes that need custom UV mapping (cylinders, spheres, etc.) where
`convex_solid_model`'s planar UV projection isn't suitable, build the mesh
directly using `Vertex`, `MeshPrimitive`, `ModelPart`, and `Model`:

```rust
let parts = vec![ModelPart::new(vec![
    MeshPrimitive {
        vertices: my_vertices,   // Vec<Vertex>
        indices: my_indices,     // Vec<u32>
        material: my_material,   // MaterialId
    },
])];
let model = Arc::new(Model::flat(parts));
```

`Vertex` fields:
```rust
Vertex {
    pos: Vector4::new(x, y, z, 1.0),    // local-space position
    color: Colour::WHITE.to_vec4(),      // vertex colour (usually white with textures)
    tex_coords: Vector2::new(u, v),      // UV coordinates
    normal: Vector3::new(nx, ny, nz),    // surface normal
}
```

Multiple `MeshPrimitive`s in one `ModelPart` share a single draw call but can
have different materials. Use this for multi-material objects (e.g. fence post
barrel + caps).

## Procedural textures

### Shared helpers (`shared/textures.rs`)

- `Rgb` — colour struct with `scale(f32)`, `lerp(other, t)`, `write_rgba(&mut Vec<u8>)`.
- `TextureRng::new(seed)` — a texture's own source of variation. `range(lo, hi)`,
  `unit()`, `u32()`, `pick(n)`, `flip()`. Never draw from the global `rand`: a
  texture that does is a function of how many textures were baked before it, so
  adding an object to a level silently repaints everything after it.
- `seed_from_position(pos, index)` / `seed_from_ground(pos, index)` — the stable
  identity to seed with. `index` separates several textures baked for one
  object, such as the blocks of a stack.
- `hue_to_rgb(h, s, v)` — HSV to RGB. `h` is in 0..6 (not 0..360).
- `edge_vignette(u, v)` — subtle darkening at texture edges.
- `border_band(u, v, width)` — darkening band at a given inset distance.
- `crack_pattern(u, v, seed)` — crack-like noise.
- `rivet_pattern(u, v, size)` — rivet dots.
- `plank_border(u, v, width)` — plank divider.
- `hash_pair(a, b)` — deterministic integer hash for per-element variation
  (e.g. per-brick colour shifts).
- `fbm_2d_periodic(u, v, octaves, persistence, lacunarity, seed, period)` —
  tileable fractal Brownian motion noise (from `crate::utils::noise`). The
  `period` parameter (`Option<u32>`) controls tiling; use `Some(N)` where N
  matches the frequency multiplier for seamless tiling.

### Material consistency

Create one material per visual type and share it across all pieces of that type.
For example, the house uses 3 materials (stone, brick, slate) shared across 18
physics bodies. Don't create a separate material per body — it wastes texture
memory and produces inconsistent visuals.

### See-through materials

A material that lets light through is declared once, on its substance:

```rust
pub const ICE: Substance = Substance {
    // ...
    transparency: Transparency::ICE,   // opacity 0.62, refractive index 1.31
};
```

Nothing else has to change. The material's transparency is what routes its
draws into the sorted blended pass (`src/rendering/transparency/`), so a
spawnable built the ordinary way — `ctx.patterned(&substance, ...)`, or
`substance.material(texture)` — comes out transparent wherever it is drawn, and
no call site can forget. `ice/` is the worked example: one block type,
and a wall and an igloo built out of it.

Three things behave differently for a blended object, all of them deliberate:

- **It casts no shadow.** A shadow map stores one depth per texel, so a
  transmissive caster could only throw a solid shadow — a hard black bite out
  of whatever stands behind something you can see straight through.
- **It writes depth**, unlike most blended geometry. It can, because the
  draws are sorted before they are recorded, and it must, because water, fire
  and particles are drawn after the scene resolves and test against that depth
  — without it the water surface paints straight over an ice cube standing in
  a pond. The cost is the other direction: those effects cannot be seen
  *through* the glass, so water behind an ice cube shows as the riverbed
  behind it rather than as water.
- **Both of its sides are drawn**, so its mesh must be closed and every
  triangle wound outwards. On an opaque object a single inside-out triangle is
  invisible (it is culled); here it is not. Worth a test when you build the
  mesh by hand — see `ice/block.rs`.

What tells two transmissive materials apart is mostly *not* the index — ice
and glass differ by 0.2 there and by a factor of five in opacity. Past that,
it is the pattern: a clear material shows you whatever structure is inside it,
so ice carries directional fracture planes (`Layer::Streak`) and white frost,
and without them a block of ice at any opacity is a block of glass.

Sorting is per draw, by distance to the mesh's centre, plus a back-faces-then-
front-faces split within each mesh. That is exact for a convex shape. Two
blended objects that interpenetrate, or a concave blended mesh, will still
composite wrong where they cross — and now that blended geometry writes depth,
wrong there means hard occlusion rather than a soft blending error.

`visual_bench -- ice` is where all of this is checked: `three_deep` and
`overlap` for the sort, `in_water` for the agreement with the passes drawn
after the scene resolves.

### Texture creation pattern

```rust
fn create_my_material(
    textures: &TextureManager,
    materials: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_my_texture();  // -> Vec<u8> (RGBA)
    let texture = textures.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    Ok(materials.register(Material::textured(texture)))
}
```

The last `bool` parameter enables mipmapping.

### Typical texture generation structure

```rust
const TEXTURE_SIZE: u32 = 256;  // or 128 for simpler materials

fn generate_my_texture(seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base_colour = Rgb::new(0.5, 0.4, 0.3);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Use fbm_2d_periodic for natural variation.
            let noise = fbm_2d_periodic(
                u * 6.0, v * 6.0,  // frequency
                3,                  // octaves
                0.5,                // persistence (amplitude falloff)
                2.0,                // lacunarity (frequency multiplier)
                seed,               // PRNG seed
                Some(6),            // period (match frequency for tiling)
            );

            let colour = base_colour.scale(0.85 + noise * 0.15);
            colour.write_rgba(&mut pixels);
        }
    }
    pixels
}
```

Use `seed.wrapping_add(N)` for multiple independent noise layers from the same
base seed.

## Physics

### Body and collider setup

```rust
let mut physics = world.write_resource::<PhysicsResource>();
let body = physics.world.create_body(
    RigidBodyDesc::dynamic()
        .position(pos)              // Point3
        .rotation(orientation)      // UnitQuaternion (optional)
        .linear_damping(0.01)
        .angular_damping(0.005),
);
physics.world.attach_collider(
    body,
    ColliderDesc::box_shape(half_extents)   // or ::convex_hull(arc_hull)
        .density(800.0)
        .restitution(0.1)
        .friction(0.7),
);
```

### Collider types

- `ColliderDesc::box_shape(half_extents)` — OBB, cheapest.
  `half_extents: Vector3<f32>`.
- `ColliderDesc::sphere(radius)` — sphere. `radius: f32`.
- `ColliderDesc::capsule(half_height, radius)` — capsule. Both `f32`.
- `ColliderDesc::convex_hull(arc_hull)` — arbitrary convex shape.
  `arc_hull: Arc<ConvexHull>`.

All collider types support these builder methods:
- `.density(f32)` — mass = density x volume.
- `.restitution(f32)` — bounciness (0 = no bounce, 1 = perfectly elastic).
- `.friction(f32)` — surface friction (see below).
- `.offset_translation(Vector3<f32>)` — offset the collider relative to the
  body origin. Used for anchored objects where only the exposed portion should
  collide.

### Friction

Friction values are not clamped — any positive `f32` works. The combined
friction between two surfaces is `sqrt(a * b)` (geometric mean). Real-world
static friction coefficients regularly exceed 1.0 (e.g. rubber on concrete is
~1.7). Use higher values for surfaces that need to grip, like roof panels on
sloped entablature.

### Rotated bodies

If a body has an initial rotation (e.g. sloped roof panels), set it on both
the physics body and the ECS component:

```rust
let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);
// ...
RigidBodyDesc::dynamic().rotation(rotation)
// ...
.with(Orientation(rotation))
```

The `Orientation` component is synced from physics each frame, but the initial
value must match or you get a one-frame visual pop.

### Compound bodies vs separate entities

- **Separate entities** (house, temple): each piece is an independent rigid body.
  Any piece can be knocked loose. Simpler to set up.
- **Compound bodies** (table, plank bridge): one physics body with multiple
  colliders at offsets. Add `CompoundFracture` + `FractureJoint`s if you want
  pieces to break apart under force.

### Realistic densities (kg/m^3)

| Material  | Density |
|-----------|---------|
| Wood      | 500-700 |
| Brick     | 1800    |
| Concrete  | 2400    |
| Granite   | 2700    |
| Marble    | 2700    |
| Steel     | 7800    |

## Terrain-anchored objects

For objects pinned to the terrain surface (fence posts, menhirs, play wheels),
use `(f32, f32)` for the position (x, z) and query the terrain for Y:

```rust
pub pos: (f32, f32),  // not (f32, f32, f32)

// In spawn():
let surface_y = {
    let terrain = world.read_resource::<TerrainWorld>();
    terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
};
let Some(surface_y) = surface_y else {
    return Vec::new();
};
```

### Constraint setup

Use `world_fixed` to pin a body at a fixed world position and orientation
(all 6 DOF locked):

```rust
let fixed_handle = physics.world.create_constraint(
    ConstraintKind::world_fixed(
        body_handle,
        Point3::new(x, buried_y, z),       // world anchor position
        Vector3::new(0.0, -half_height, 0.0), // body-local anchor
        0.0,                                // compliance (rigid)
        f32::MAX,                           // max impulse (unbreakable)
    )
);
```

### TerrainAnchored component

Attach the `TerrainAnchored` component so the terrain system can release the
constraints when the terrain beneath is destroyed:

```rust
.with(TerrainAnchored {
    anchor_handle: fixed_handle,    // ConstraintHandle
    upright_handle: fixed_handle,   // same handle (second remove is harmless)
    anchor_world: Point3::new(      // sample point checked each frame
        x,
        surface_y - 0.1,           // slightly below surface
        z,
    ),
    released_collider: Some(full_collider),  // swapped in on release
    released_model: None,           // or Some(model) to swap visual on release
})
```

### Anchored vs released collider pattern

While anchored, the collider covers only the exposed portion (offset upward).
When released, the full-size collider is swapped in:

```rust
let exposed_half_height = (full_height - buried_depth) / 2.0;
let collider_offset_y = half_height - exposed_half_height;

let anchored_collider = ColliderDesc::capsule(exposed_half_height, radius)
    .density(density)
    .offset_translation(Vector3::new(0.0, collider_offset_y, 0.0));

let released_collider = ColliderDesc::capsule(half_height, radius)
    .density(density);
```

## ECS entity setup

Every spawned entity needs at minimum:

```rust
world.create_entity()
    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
    .with(Velocity(Vector3::zeros()))
    .with(Orientation::default())       // or Orientation(rotation)
    .with(RigidBodyComponent(body_handle))
    .with(ModelInstance::new(model))
    .with(Renderable)
    .build()
```

Optional components:
- `Flammable::wood()` — makes the entity catch fire.
- `TerrainAnchored { ... }` — terrain-pinned with release on destruction.

## Reference implementations

| Pattern | Example file | Key feature |
|---------|-------------|-------------|
| Simple free body | `tetrahedron.rs` | ConvexHull collider + `convex_solid_model` |
| Terrain-anchored | `fence_post.rs` | Fixed + TerrainAnchored |
| Custom mesh | `fence_post.rs` | Hand-built barrel + cap meshes with `MeshPrimitive` |
| Multi-entity | `pyramid.rs` | Grid of independent bodies from one spawnable |
| Compound body | `table.rs` | Multiple colliders on one body + fracture |
| Box-based | `box_object.rs` | `cuboid_model` + `ColliderDesc::box_shape` |
| Swept curved mesh | `banana.rs` | Parametric surface + capsule-chain compound collider |
