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

The `Spawnable` trait has three methods:

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

## Geometry and models

### Shared model builders (`shared/models.rs`)

- `cuboid_model(half_extents, material)` — single axis-aligned box.
- `compound_cuboid_model(boxes, material)` — multiple boxes, one material.
- `multi_material_compound_cuboid_model(boxes)` — multiple boxes, per-box
  materials.
- `multi_material_rotated_compound_cuboid_model(boxes)` — same but with per-box
  rotation quaternions.
- `convex_solid_model(vertices, faces, material)` — arbitrary convex shape from
  vertex + face definitions. Uses per-face planar UV projection.
- `build_convex_hull(vertices, faces)` — builds a `ConvexHull` for the physics
  collider from the same vertex/face data.

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

## Procedural textures

### Shared helpers (`shared/textures.rs`)

- `Rgb` — colour struct with `scale`, `lerp`, `write_rgba`.
- `rand_range`, `rand_u32` — random values for per-instance variation.
- `hash_pair(a, b)` — deterministic integer hash for per-element variation
  (e.g. per-brick colour shifts).
- `edge_vignette(u, v)` — subtle darkening at texture edges.
- `border_band(u, v, width)` — darkening band at a given inset distance.
- `fbm_2d_periodic(...)` — tileable fractal Brownian motion noise (from
  `crate::utils::noise`). The `period` parameter controls tiling.

### Material consistency

Create one material per visual type and share it across all pieces of that type.
For example, the house uses 3 materials (stone, brick, slate) shared across 18
physics bodies. Don't create a separate material per body — it wastes texture
memory and produces inconsistent visuals.

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
- `ColliderDesc::sphere(radius)` — sphere.
- `ColliderDesc::capsule(half_height, radius)` — capsule.
- `ColliderDesc::convex_hull(arc_hull)` — arbitrary convex shape.

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
| Marble    | 2700    |
| Steel     | 7800    |

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
