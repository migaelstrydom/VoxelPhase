# Destructible Bodies Plan

Design document for compound colliders and runtime fracture — enabling concave rigid bodies and destructible objects like tables and barricades.

---

## Status

Phase 1 (compound colliders) is implemented. Phase 2 (compound fracture) is the next step.

---

## Goal

A table (or any concave object) that behaves as a perfectly rigid body under normal conditions, then breaks apart instantly when hit by a grenade or large impact. The system should generalize to any prefractured destructible object.

---

## Phased approach

| Phase | Feature | What it enables |
|-------|---------|-----------------|
| 1 | Compound colliders | Concave rigid bodies (tables, chairs, barricades) as a single body with multiple shapes. Perfectly rigid, zero solver cost. |
| 2 | Compound fracture | Compound bodies that monitor per-child forces and split into independent bodies on impact. Perfectly rigid until hit, then pieces fly off instantly. |

Each phase is independently useful and shippable.

---

## Phase 1: Compound colliders

### Problem

Every collider currently uses the body's position and rotation directly. The `offset` field on `Collider` exists and `world_center()` applies it for translation, but the narrowphase uses `body.rotation()` without composing the collider's offset rotation. This means offset colliders with non-identity rotation (e.g. a table leg rotated 90°) would have the wrong orientation in collision detection.

Additionally, `recompute_mass_properties()` sums local inertia tensors directly without applying the parallel axis theorem, so compound bodies with offset colliders would have incorrect mass distribution.

### Work items

#### 1a. Compose collider offset rotation in narrowphase

Add a `world_rotation()` method to `Collider` (or a `world_transform()` returning `Isometry3`) that composes the body rotation with the collider offset rotation:

```rust
pub fn world_transform(
    &self,
    body_position: Point3<f32>,
    body_rotation: UnitQuaternion<f32>,
) -> Isometry3<f32> {
    let body_iso = Isometry3::from_parts(body_position.coords.into(), body_rotation);
    body_iso * self.offset
}
```

Update all narrowphase call sites:
- `dynamic_contacts.rs` `ColliderState`: use composed rotation instead of `body.rotation()`
- `static_contacts.rs`: same — every place that passes `body.rotation()` for shape orientation
- CCD sweep: verify it uses the composed transform

This is the critical correctness fix. Without it, rotated child colliders collide with the wrong orientation.

#### 1b. Parallel axis theorem in mass aggregation

`recompute_mass_properties()` currently has a comment: "For now, just add local inertias (ignoring offset transforms)". Fix this using the parallel axis theorem:

```
I_total = Σ (R_i · I_local_i · R_iᵀ + m_i · [d_i² · E - d_i ⊗ d_i])
```

Where `R_i` is the collider's offset rotation, `d_i` is the offset translation, and `m_i` is the collider's mass.

This is essential for compound bodies — a table with legs at the corners has very different rotational inertia than a point mass at the center.

#### 1c. Collider offset in rendering

The rendering system needs to apply the collider offset when drawing compound bodies. Currently each collider is rendered at the body's transform. For compound bodies, each child shape needs `body_transform * collider.offset` as its model matrix.

#### 1d. Builder API

Add a convenience builder for compound collider descriptions:

```rust
// Example: table top + 4 legs
let top = ColliderDesc::box_shape(Vector3::new(0.5, 0.025, 0.3))
    .offset_translation(Vector3::new(0.0, 0.375, 0.0));

let leg = ColliderDesc::box_shape(Vector3::new(0.03, 0.175, 0.03));
let legs = [
    leg.clone().offset_translation(Vector3::new(-0.4, 0.175, -0.22)),
    leg.clone().offset_translation(Vector3::new( 0.4, 0.175, -0.22)),
    leg.clone().offset_translation(Vector3::new(-0.4, 0.175,  0.22)),
    leg.clone().offset_translation(Vector3::new( 0.4, 0.175,  0.22)),
];

let table = world.create_body(RigidBodyDesc::dynamic(table_center));
world.attach_collider(table, top);
for l in legs { world.attach_collider(table, l); }
```

Add `offset_translation()` and `offset_rotation()` builder methods to `ColliderDesc`.

#### 1e. Bench scenario

Add a `CompoundBody` bench scenario: a table (5 boxes) spawned above a flat surface. Verify it settles correctly, doesn't jitter, and contact impulses are reasonable. This exercises the offset rotation, parallel axis theorem, and multi-collider narrowphase in one test.

### Implementation notes

All work items (1a–1e) are complete, plus CCD multi-collider support.

**Files changed:**
- `src/physics/collider.rs` — Added `world_transform()`, `offset()` accessor, `offset_translation()`/`offset_rotation()` builders on `ColliderDesc`.
- `src/physics/narrowphase/dynamic_contacts.rs` — `ColliderState` uses composed rotation via `world_transform()`. Added self-collision filtering (skip pairs on same body).
- `src/physics/narrowphase/static_contacts.rs` — Same composed rotation fix for box/capsule vs static.
- `src/physics/world.rs` — `recompute_mass_properties()` now applies parallel axis theorem with rotated inertia + Steiner term.
- `src/physics/ccd/sweep_clamp.rs` — Emits one `CcdCandidate` per collider (was only first). Uses composed transforms for pre/post centers.
- `src/bin/bench_viewer.rs` — `draw_bodies()` applies collider offsets. Registered `compound_table` scenario.
- `src/physics/bench_harness/scenarios.rs` — `CompoundTableScenario` (table: 5 boxes on flat grid).

**Spawners added:**
- `src/app/spawners/table_entity.rs` — `spawn_table()` with procedural wood texture, compound body (5 box colliders).
- `src/level/data.rs` — `LevelObject::Table` variant with serde defaults.

### What this doesn't do

- No destruction. The table is a single rigid body — it can't break apart.
- No per-child force monitoring (that's Phase 2).

---

## Phase 2: Compound fracture

### Design

A compound body monitors the contact impulses on each child collider. When the impulse on a child exceeds a threshold, the compound body splits instantly into independent bodies — no intermediate weld state, no wobble phase.

#### CompoundFracture component

This is gameplay-level logic, not physics engine internals. It sits in the ECS layer:

```rust
pub struct CompoundFracture {
    /// Per-child-collider break threshold (impulse magnitude).
    pub child_thresholds: Vec<f32>,
    /// Whether this body has already been fractured (one-shot).
    pub fractured: bool,
}
```

#### Per-child impulse tracking

The solver already computes impulses per contact manifold, and manifolds reference collider handles. After solving, sum the impulse magnitudes per collider handle. This data flows out of `PhysicsWorld` as a per-frame query:

```rust
pub fn collider_impulse_magnitudes(&self) -> &HashMap<ColliderHandle, f32>
```

Computed once after the last substep from the cached manifolds.

#### The swap

When a child collider's impulse exceeds its threshold:

1. Record the compound body's position, rotation, and velocities.
2. Remove the compound body.
3. For each former child collider, create a new dynamic body:
   - Position = old body's world transform * child's local offset
   - Rotation = old body's rotation * child's offset rotation
   - Velocity = old body's linear velocity + ω × (child world center - body center)
   - Single collider with identity offset (the shape is now the body's own)
   - Mass = child collider's mass
4. The pieces fly apart naturally from the impact forces. No weld joints, no gradual weakening — the structure was rigid and now it's not.

#### Break events

The fracture system should emit events so gameplay code can react — play a sound, spawn particles, apply an outward impulse to the freed pieces, etc.:

```rust
pub struct FractureEvent {
    /// The compound body that fractured.
    pub body: RigidBodyHandle,
    /// The child collider that received the breaking impulse.
    pub impact_child: ColliderHandle,
    /// The impulse magnitude that caused the fracture.
    pub impulse_magnitude: f32,
}
```

#### Tuning

The break threshold needs tuning per object. A wooden table leg should break more easily than a steel beam. The threshold is in impulse units (N·s), so it scales with mass — heavier objects naturally need more impulse to break, which is physically correct.

#### Table conversion

The table spawnable (`src/app/spawnables/table.rs`) is already a compound body (top slab + 4 legs). Adding `CompoundFracture` to the table entity enables destruction without any physics-level changes — just attach the component with per-child thresholds.

#### Bench scenario

A table (compound body, 5 children) on a flat surface. A sphere is launched into one leg. Expected sequence:
1. Before impact: table is a single compound body, perfectly rigid.
2. Impact exceeds fracture threshold on the hit leg.
3. Compound body is replaced with 5 independent bodies.
4. The hit leg flies off. Other pieces scatter based on the impact.

---

## Narrowphase considerations

### Multi-collider broadphase

The dynamic broadphase (sort-and-sweep on bounding spheres) already works per-collider via `ColliderState`, so multiple colliders on one body are handled correctly. Each child gets its own bounding sphere and participates independently in pair generation.

### Self-collision filtering

When a body has multiple colliders, the narrowphase must skip pairs where both colliders belong to the same body. Currently `collect_collider_states_into` stores `body_handle` per state — the pair dispatch checks `a.body_handle != b.body_handle` before generating contacts.

### Collider-to-body mapping

After the compound fracture swap, the old collider handles are invalidated (the old body is removed). New colliders are created for each child body. Any system caching collider handles (manifold cache, sleep manager) needs to handle this gracefully. The manifold cache already handles missing colliders via generational arena lookups — stale handles simply fail the lookup and the cached manifold is dropped.

---

## Performance notes

- **Compound colliders** (Phase 1): zero solver cost. The narrowphase does slightly more work (N child shapes per body instead of 1), but this is the same work it would do for N separate bodies. Mass property computation is once at creation time.
- **Fracture swap** (Phase 2): one-time cost when fracture triggers. Involves arena operations (remove body, create N bodies). Not performance-sensitive since it happens once per destructible object.
- **Per-child impulse tracking** (Phase 2): iteration over cached manifolds, one sum per collider. Linear in contact count, runs once per frame.

---

## Dependencies and risks

| Risk | Phase | Mitigation |
|------|-------|------------|
| Collider offset rotation not tested in all narrowphase paths | 1 | Thorough bench scenario with rotated child colliders. Test all shape pair combinations. |
| Parallel axis theorem incorrect → wrong inertia → instability | 1 | Compare compound body inertia against equivalent single-body inertia for simple geometries (e.g. two identical boxes at known offsets). |
| Compound-to-separate swap causes visual pop | 2 | The swap should be imperceptible if transforms are computed correctly from the old body's state. Any pop indicates a transform computation bug. |
| Manifold cache invalidation during swap | 2 | Generational arena handles this — stale handles fail lookup gracefully. Verify warm-start doesn't produce artifacts on the first frame after swap. |
| Break threshold tuning is unintuitive | 2 | Provide `FractureEvent` with impulse magnitude so designers can log actual values and set thresholds empirically. |

---

## What to defer

- **Convex hull collider shape** — needed for convex decomposition of arbitrary meshes, but not for hand-authored destructible objects built from boxes/capsules/spheres.
- **Automatic convex decomposition** (V-HACD) — content pipeline step, only useful once convex hull shapes exist.
- **Fracture debris cleanup** — small pieces should be removed after a timeout to avoid unbounded body count. Simple timer-based removal, implement when destruction is in gameplay.
- **Nested compound bodies** — a compound body where a child is itself a compound. Not needed initially; each child should be a primitive shape.
- **Partial fracture** — only detaching the impacted child while keeping the rest as a compound. Would require splitting a compound body into a smaller compound + separate body. More complex but more realistic. Defer until full fracture is proven.

---

## Weld joint retrospective

Weld joints were implemented and then removed after investigation revealed fundamental issues with flush-touching welded bodies.

### What happened

Weld joints constrain two separate bodies to maintain a fixed relative position and orientation. They worked correctly for bodies with gaps between them (e.g. the player grab system's FollowPoint constraint). However, for pre-authored structures where collider surfaces touch (e.g. posts flush against planks), two problems emerged:

1. **Baumgarte energy injection.** The weld constraint used Baumgarte stabilization (velocity bias) for position correction, while contacts used NGS (direct position adjustment). The velocity bias injected energy into the system, causing welded structures to slowly gain momentum and eventually fly apart. This was fixed by the solver constraint refactor (see `SOLVER_CONSTRAINT_REFACTOR_PLAN.md`), which removed Baumgarte from constraint rows.

2. **Contact-weld fighting.** Even after fixing the energy injection, contacts between flush-touching welded bodies generated separation forces that fought the weld constraint. The PGS solver converged to an equilibrium where the contact and weld impulses partially cancelled, but the residual created a slow rocking instability. Over ~100 frames, the rocking grew until a body penetrated the ground deeply enough to trigger a massive corrective impulse, launching the entire structure.

The second problem is fundamental to flush-touching welded bodies: the narrowphase generates contacts that oppose the weld, creating conflicting constraints the solver cannot fully reconcile. Contact filtering was considered but rejected — it would mask the conflict for rigid welds but break soft/compliant welds where inter-body contacts are physically meaningful.

### Conclusion

Flush-touching rigid structures should be compound bodies (one body, multiple colliders). Compound bodies are perfectly rigid with zero solver cost, and the self-collision filter naturally prevents contacts between children of the same body. The weld joint's use case — transient post-fracture connections — was eliminated by the simpler design of instant fracture (compound → separate bodies, no intermediate weld state).

The FollowPoint constraint remains for the player grab system, where the two bodies are not flush-touching and the bounded impulse limits prevent instability.

### What was removed

- `ConstraintKind::Weld` variant and all associated code (`weld.rs`, expansion arm, position correction, bench scenarios and tests)
- `WeldedBodies` contact filtering (`welded_bodies.rs`, narrowphase parameter)
- Barricade spawnable (replaced by compound-body table as the primary destructible object)
