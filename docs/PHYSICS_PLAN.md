# Rigid Body Physics Engine Implementation Plan

This document outlines the incremental implementation plan for adding a proper rigid body physics engine to RustDude. The goal is to support Tears of the Kingdom-style physics where players can stick arbitrary rigid objects together and interact with them to solve puzzles.

## Design Principles

1. **Handle-based API** - Game code never holds direct references to physics objects; only opaque handles
2. **Descriptor pattern** - Create objects via immutable descriptors, not setters
3. **Single ownership** - PhysicsWorld owns all physics state; game code interacts through handles
4. **Clean separation** - Physics engine is a standalone module with a well-defined API boundary
5. **Incremental delivery** - Each phase produces a working, testable feature

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│  Game Layer (ECS)                                                   │
│  ├─ RigidBodyHandle component (opaque handle to rigid body)        │
│  ├─ PhysicsSyncSystem (syncs ECS ↔ PhysicsWorld)                   │
│  └─ Game systems query/command PhysicsWorld via handles            │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              │ Clean API boundary
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│  PhysicsWorld (owns all physics state)                             │
│  ├─ Bodies: Arena<RigidBody>                                       │
│  ├─ Colliders: Arena<Collider>                                     │
│  ├─ Joints: Arena<Joint>  (Phase 4+)                               │
│  └─ Pipeline                                                       │
│       ├─ Integration (velocity → position)                         │
│       ├─ Collision Detection                                       │
│       │    ├─ vs StaticGeometry (terrain)                         │
│       │    └─ vs other bodies                                      │
│       └─ Solver (constraint resolution)                            │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              │ StaticGeometry trait
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│  TerrainManager (implements StaticGeometry)                        │
│  ├─ query_sphere(center, radius) → Vec<Contact>                    │
│  ├─ sweep_sphere(start, end, radius) → Option<SweptContact>        │
│  └─ (future: query_box, sweep_box, etc.)                           │
└─────────────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
src/physics/
├── mod.rs                  # Public API, re-exports
├── world.rs                # PhysicsWorld implementation
├── body.rs                 # RigidBody, BodyType, mass properties
├── collider.rs             # Collider, ColliderDesc, shapes
├── handle.rs               # Handle types (RigidBodyHandle, ColliderHandle, etc.)
├── math.rs                 # Isometry, inertia tensor calculations
├── static_geometry.rs      # StaticGeometry trait
├── pipeline/
│   ├── mod.rs
│   ├── integration.rs      # Velocity/position integration
│   ├── collision.rs        # Collision detection orchestration
│   └── solver.rs           # Constraint solver
├── collision/
│   ├── mod.rs
│   ├── sphere_sphere.rs    # Sphere-sphere detection
│   ├── sat.rs              # SAT for boxes (Phase 2)
│   ├── gjk.rs              # GJK algorithm (Phase 7)
│   ├── epa.rs              # EPA algorithm (Phase 7)
│   └── manifold.rs         # Contact manifold (Phase 3)
└── joint/                  # (Phase 4+)
    ├── mod.rs
    ├── fixed.rs
    ├── revolute.rs
    └── spherical.rs
```

## Core API

```rust
// === Handles (opaque, generational) ===
pub struct RigidBodyHandle(generational_arena::Index);
pub struct ColliderHandle(generational_arena::Index);
pub struct JointHandle(generational_arena::Index);  // Phase 4+

// === Descriptors ===
pub struct RigidBodyDesc {
    pub body_type: BodyType,           // Dynamic, Kinematic, Static
    pub position: Point3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub linear_velocity: Vector3<f32>,
    pub angular_velocity: Vector3<f32>,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
}

pub struct ColliderDesc {
    pub shape: ColliderShape,
    pub offset: Isometry3<f32>,        // Local transform relative to body
    pub density: f32,                  // For mass/inertia calculation
    pub restitution: f32,
    pub friction: f32,
}

pub enum ColliderShape {
    Sphere { radius: f32 },
    Box { half_extents: Vector3<f32> },           // Phase 2
    Capsule { half_height: f32, radius: f32 },    // Phase 6
    ConvexHull { points: Arc<Vec<Point3<f32>>> }, // Phase 7
}

// === Main Interface ===
impl PhysicsWorld {
    pub fn new(config: PhysicsConfig) -> Self;

    // Body management
    pub fn create_body(&mut self, desc: RigidBodyDesc) -> RigidBodyHandle;
    pub fn remove_body(&mut self, handle: RigidBodyHandle) -> bool;
    pub fn body(&self, handle: RigidBodyHandle) -> Option<&RigidBody>;
    pub fn body_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody>;

    // Collider management
    pub fn attach_collider(&mut self, body: RigidBodyHandle, desc: ColliderDesc) -> Option<ColliderHandle>;
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> bool;

    // Simulation
    pub fn step(&mut self, dt: f32, static_geometry: &dyn StaticGeometry);

    // Queries
    pub fn cast_ray(&self, origin: Point3<f32>, dir: Vector3<f32>, max_dist: f32) -> Option<RayCastHit>;
}

// === Static Geometry Interface ===
pub trait StaticGeometry {
    fn query_sphere(&self, center: Point3<f32>, radius: f32) -> Vec<StaticContact>;
    fn sweep_sphere(&self, start: Point3<f32>, end: Point3<f32>, radius: f32) -> Option<SweptContact>;
    // Added incrementally as we add shapes:
    // fn query_box(...) -> Vec<StaticContact>;      // Phase 2
    // fn sweep_box(...) -> Option<SweptContact>;    // Phase 2
}
```

## Implementation Phases

### Phase 1: Foundation + Rotating Spheres
**Goal:** Beach balls spin when they bounce on terrain.

**What to build:**
- `PhysicsWorld` core structure with generational arena
- `RigidBody` with position, orientation (quaternion), linear/angular velocity
- `RigidBodyHandle` and `ColliderHandle` types
- Sphere collider with proper inertia tensor (I = 2/5 * m * r²)
- `StaticGeometry` trait
- `TerrainManager` implements `StaticGeometry`
- Sphere-sphere collision with angular velocity response
- Basic impulse solver (single iteration)
- `PhysicsSyncSystem` for ECS integration

**Acceptance criteria:**
- Beach balls bounce on terrain and spin realistically
- Two balls collide, spin transfers based on contact point offset
- Ball rolling down a slope spins appropriately
- Game code only uses handles (no direct physics access)

**Systems to remove:**
- `DynamicTerrainCollisionSystem`
- `DynamicDynamicCollisionSystem`
- `MotionPredictionSystem`
- `TerrainCollisionSystem`
- `PenetrationResolutionSystem`
- `GravitySystem` (gravity moves into PhysicsWorld)
- `VelocityIntegrationSystem` (integration moves into PhysicsWorld)

**Components to remove:**
- `MotionState` (replaced by internal physics state)
- `Gravity` (replaced by `gravity_scale` in RigidBodyDesc)
- `Acceleration` (forces applied directly to bodies)

**Components to keep:**
- `Position` (synced from physics)
- `Velocity` (synced from physics)
- `PhysicsBody` → renamed/repurposed as material properties only

**New components:**
- `RigidBodyHandle` (just wraps the handle)
- `Orientation` (UnitQuaternion, synced from physics)

---

### Phase 2: Box Colliders + SAT
**Goal:** Crates that tumble down hills.

**What to build:**
- Box shape with half-extents
- Box inertia tensor (I_xx = m/12 * (h² + d²), etc.)
- OBB vs OBB collision via Separating Axis Theorem (SAT)
- OBB vs Sphere collision
- Extend `StaticGeometry` with `query_box`, `sweep_box`
- Face/edge/vertex contact point generation

**Acceptance criteria:**
- Spawn wooden crates that tumble realistically
- Boxes collide correctly with spheres
- Boxes spin when hit off-center
- Boxes rest on terrain slopes

---

### Phase 3: Contact Manifolds + Stable Stacking
**Goal:** Boxes stack without jittering.

**What to build:**
- Persistent contact manifold (survives across frames)
- Contact point caching with warm starting
- Multiple contact points per collision pair
- Increase solver iterations (4-8)
- Position correction (Baumgarte stabilization or split impulse)

**Acceptance criteria:**
- Stack 5 boxes on top of each other
- Stack remains stable indefinitely (no jitter, no drift)
- Push bottom box, stack topples realistically

---

### Phase 4: Joint Constraints
**Goal:** Stick objects together.

**What to build:**
- Constraint trait/interface
- Contact constraints refactored to use constraint interface
- Fixed joint (rigid attachment)
- Ball-socket joint (rotation around point)
- Distance constraint
- Joint creation/removal API

**Acceptance criteria:**
- Attach two boxes with fixed joint → move as one body
- Attach with ball-socket → swing like a flail
- Create a chain of boxes connected by joints

---

### Phase 5: Revolute (Hinge) Joints + Motors
**Goal:** Doors, levers, rotating platforms.

**What to build:**
- Hinge joint with axis constraint
- Joint limits (min/max angle)
- Joint motors (target velocity or position)
- Angular spring/damper

**Acceptance criteria:**
- Create a door that swings on hinges
- Door stops at 90° limit
- Create a motorized rotating platform

---

### Phase 6: Capsule Colliders
**Goal:** Elongated objects, better character proxies.

**What to build:**
- Capsule shape (cylinder + hemispherical caps)
- Capsule inertia tensor
- Capsule vs Capsule collision
- Capsule vs Sphere collision
- Capsule vs Box collision
- Extend `StaticGeometry` for capsules

**Acceptance criteria:**
- Spawn capsule-shaped objects (logs, pills)
- They roll and tumble correctly
- Collide properly with boxes and spheres

---

### Phase 7: GJK + EPA for Convex Hulls
**Goal:** Arbitrary convex mesh colliders.

**What to build:**
- Support mapping interface for all shapes
- GJK algorithm for convex-convex intersection
- EPA algorithm for penetration depth/normal
- ConvexHull shape type with precomputed support data

**Acceptance criteria:**
- Load arbitrary convex meshes as colliders
- They collide correctly with all other shapes
- Performance acceptable for reasonable poly counts

---

### Phase 8: Compound Colliders
**Goal:** Multi-shape bodies (table = box + 4 cylinders).

**What to build:**
- Multiple colliders per body
- Compound inertia tensor calculation
- Local transforms per collider
- Compound AABB for broad phase

**Acceptance criteria:**
- Build a table from primitives, behaves as single body
- Mass distribution is correct (tips over realistically)

---

### Phase 9: Broad Phase Optimization
**Goal:** Handle 100+ dynamic bodies efficiently.

**What to build:**
- Spatial hash grid or dynamic BVH
- Pair management (sleeping pairs, new pairs)
- Island detection for sleeping
- O(n log n) collision pair generation

**Acceptance criteria:**
- 100 boxes in a pile, stable 60 FPS
- 200 boxes, still playable

---

### Phase 10: Rotational CCD
**Goal:** Fast-spinning objects don't clip.

**What to build:**
- Conservative advancement for rotational motion
- CCD activation threshold (only for fast objects)
- Substep integration for CCD cases

**Acceptance criteria:**
- Fast-spinning box doesn't clip through floor
- Performance impact minimal for slow objects

---

## ECS Integration Pattern

```rust
// Component: just stores the handle
#[derive(Component)]
pub struct RigidBodyComponent(pub RigidBodyHandle);

// New component for orientation
#[derive(Component)]
pub struct Orientation(pub UnitQuaternion<f32>);

// System: single point of physics<->ECS sync
pub struct PhysicsSyncSystem;

impl<'a> System<'a> for PhysicsSyncSystem {
    type SystemData = (
        Write<'a, PhysicsWorld>,
        Option<Read<'a, TerrainManager>>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Orientation>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, Time>,
    );

    fn run(&mut self, (mut physics, terrain, mut positions, mut velocities,
                       mut orientations, bodies, time): Self::SystemData) {
        // Step physics with terrain as static geometry
        if let Some(ref terrain) = terrain {
            physics.step(time.delta_seconds(), terrain.as_ref());
        }

        // Sync results back to ECS
        for (pos, vel, orient, body) in
            (&mut positions, &mut velocities, &mut orientations, &bodies).join()
        {
            if let Some(rb) = physics.body(body.0) {
                pos.0 = rb.position().translation.vector;
                vel.0 = rb.linear_velocity();
                orient.0 = rb.position().rotation;
            }
        }
    }
}
```

## Key Formulas

### Inertia Tensors (diagonal, local frame)

**Solid Sphere:**
```
I = (2/5) * m * r²  (all axes)
```

**Solid Box (half-extents hx, hy, hz):**
```
I_xx = (1/12) * m * (4*hy² + 4*hz²)
I_yy = (1/12) * m * (4*hx² + 4*hz²)
I_zz = (1/12) * m * (4*hx² + 4*hy²)
```

**Solid Capsule (half-height h, radius r):**
```
// Cylinder part + hemisphere parts
// See Real-Time Collision Detection, Ericson
```

### Impulse Response

**Linear impulse at contact:**
```
j = -(1 + e) * v_rel · n / (1/m_a + 1/m_b + (I_a⁻¹(r_a × n) × r_a + I_b⁻¹(r_b × n) × r_b) · n)
```

**Where:**
- `e` = coefficient of restitution
- `v_rel` = relative velocity at contact point
- `n` = contact normal
- `r_a`, `r_b` = vectors from center of mass to contact point
- `I_a`, `I_b` = inertia tensors in world frame

**Apply impulse:**
```
Δv_a = j * n / m_a
Δω_a = I_a⁻¹ * (r_a × j * n)
Δv_b = -j * n / m_b
Δω_b = -I_b⁻¹ * (r_b × j * n)
```

### Quaternion Integration

```
q' = q + (dt/2) * Quaternion(0, ω) * q
q' = normalize(q')
```

## References

- Erin Catto's GDC presentations (Box2D author)
- "Game Physics Engine Development" by Ian Millington
- "Real-Time Collision Detection" by Christer Ericson
- "A Fast and Robust GJK Implementation" by Gino van den Bergen
- Rapier physics engine source code (Rust reference implementation)

## Progress Tracking

| Phase | Status | Notes |
|-------|--------|-------|
| 1 | Complete | Foundation + rotating spheres |
| 2 | Not Started | Box colliders |
| 3 | Not Started | Contact manifolds |
| 4 | Not Started | Joint constraints |
| 5 | Not Started | Hinge joints |
| 6 | Not Started | Capsule colliders |
| 7 | Not Started | GJK/EPA |
| 8 | Not Started | Compound colliders |
| 9 | Not Started | Broad phase |
| 10 | Not Started | Rotational CCD |

## Phase 1 Implementation Notes

**Completed 2024-01-XX:**

Files created:
- `src/physics/mod.rs` - Public API
- `src/physics/handle.rs` - RigidBodyHandle, ColliderHandle
- `src/physics/math.rs` - Inertia tensors, quaternion integration
- `src/physics/body.rs` - RigidBody, RigidBodyDesc, BodyType
- `src/physics/collider.rs` - Collider, ColliderDesc, ColliderShape
- `src/physics/static_geometry.rs` - StaticGeometry trait
- `src/physics/world.rs` - PhysicsWorld
- `src/physics/pipeline/mod.rs` - Pipeline orchestration
- `src/physics/pipeline/integration.rs` - Force/velocity integration
- `src/physics/pipeline/solver.rs` - Sequential impulse solver
- `src/physics/collision/mod.rs` - Collision module
- `src/physics/collision/sphere_sphere.rs` - Sphere-sphere detection
- `src/systems/physics_sync.rs` - PhysicsSyncSystem, PhysicsResource

Files modified:
- `src/main.rs` - Added physics module
- `src/components.rs` - Added Orientation, RigidBodyComponent
- `src/terrain/manager.rs` - Implemented StaticGeometry trait
- `src/systems/mod.rs` - Exported new system
- `src/systems/render.rs` - Support for Orientation component
- `src/app/world_builder.rs` - Register new components and PhysicsResource
- `src/app/entity_spawner.rs` - Beach balls use new physics
- `src/app/dispatcher_builder.rs` - Simplified dispatcher with PhysicsSyncSystem

Old systems kept for player (biped):
- GravitySystem, VelocityIntegrationSystem, MotionPredictionSystem, BipedCollisionSystem

Old systems no longer used (but kept in codebase):
- DynamicTerrainCollisionSystem, DynamicDynamicCollisionSystem
- TerrainCollisionSystem, PenetrationResolutionSystem

**Known limitations:**
- Player-ball collision not implemented (player uses old system)
- Single solver iteration (stacking stability comes in Phase 3)
- Only sphere colliders supported
