# Grab System Plan

Design document for player object grabbing, carrying, and throwing.

---

## Status

Complete — all planned features implemented. Tuning of `GrabConfig` parameters ongoing.

---

## Feature summary

The player can grab dynamic objects (boxes, beach balls, capsules) with right-click. While held, the object is pulled to a fixed point in front of the player via a physics constraint. Left-click while holding performs a throw. Releasing right-click drops the object naturally.

The right hand animates to reach toward the grab point on right-click, regardless of whether an object is in range (swing-and-miss on empty grabs).

---

## Controls

| Input | Current binding | New binding |
|-------|----------------|-------------|
| Jump | Space / Right-click | Space only |
| Grab (hold) | — | Right-click (hold) |
| Drop | — | Release right-click |
| Throw | — | Left-click while holding |
| Grenade | Left-click | Left-click (only when not holding) |

Right-click is freed by removing it from the jump binding in `GameplayActions::from_input_state()`.

---

## Architecture

### Layered player state machine

The player currently has a single `PlayerMoveState` enum for locomotion. Grabbing is orthogonal to locomotion — the player can grab while grounded, airborne, or jumping — but not all combinations are valid (e.g. a future Swimming state should force-drop held objects).

Rather than a separate `GrabState` component in its own module, we use a **layered state machine**: a single `PlayerState` struct holding two orthogonal enums, with a compatibility table that governs valid combinations.

#### State enums

```rust
/// Locomotion layer — how the player is moving through the world.
pub enum LocomotionState {
    Grounded,
    Launching,
    CoyoteTime(f32),
    Airborne,
    // future: Swimming, Climbing, etc.
}

/// Arm/action layer — what the player's hands are doing.
pub enum ArmState {
    /// Normal arm swing / idle.
    Idle,
    /// Right hand reaching toward grab point (animation plays regardless of hit).
    Reaching {
        elapsed: f32,
        /// Body and surface hit point from the probe (None = swing-and-miss).
        target: Option<(RigidBodyHandle, Point3<f32>)>,
    },
    /// Object attached via constraint.
    Holding {
        target_body: RigidBodyHandle,
        constraint: ConstraintHandle,
    },
}
```

#### Composite state and compatibility

```rust
pub struct PlayerState {
    pub locomotion: LocomotionState,
    pub arm: ArmState,
}

impl PlayerState {
    /// Whether the given arm state is permitted with the current locomotion.
    fn arm_state_allowed(&self, arm: &ArmState) -> bool {
        match self.locomotion {
            // Swimming forces arm to Idle (drop held objects).
            // LocomotionState::Swimming => matches!(arm, ArmState::Idle),
            _ => true,
        }
    }
}
```

When a locomotion transition occurs that invalidates the current arm state (e.g. entering Swimming while Holding), the transition method force-drops the held object (removes the constraint) and resets arm to `Idle`. This keeps the compatibility rules declarative and in one place.

#### Where it lives

`PlayerState` is a new ECS component on the player entity, defined in `src/player/components.rs`. It replaces the current `PlayerMoveState` that lives on `PlayerTargetState`.

`PlayerTargetState` remains a pure intent struct — it carries what the player *wants* to do (`direction`, `jump`, `grab_held`, etc.) and is written by `PlayerInputSystem`. It does not contain the state machine.

`PlayerState` is the actual state, owned and transitioned by `PlayerControlSystem` (renamed from `PlayerMotionSystem`). The control system reads `PlayerTargetState` as input and decides which transitions to apply to `PlayerState`.

```
PlayerInputSystem:  raw input → PlayerTargetState (intent)
PlayerControlSystem: PlayerTargetState + PlayerState → state transitions, physics calls
```

Grab-specific logic (probe, constraint creation, throw impulse) lives in `src/player/grab.rs` as helper functions called from `PlayerControlSystem`, not a separate ECS system.

### Physics-driven player turning

The hold point is a fixed offset from the player body's position and physical rotation. For the held object to respond naturally to the player's mass and inertia, the player's turning must go through the physics solver rather than being an instant animation-level yaw snap.

#### Current behavior

Mouse input → instant yaw on `Rotation` component → camera follows. The player capsule's physics body does not rotate around Y; facing is purely an animation/movement concept.

#### New behavior

Mouse input → target yaw stored as intent → `PlayerControlSystem` applies an angular velocity drive around Y to the player's physics body → the solver integrates the rotation → the hold point (derived from the body's physical rotation) moves smoothly.

The camera is unaffected — it still orbits based on mouse input directly, independent of the player body's rotation. The player can still move in any direction before the turn completes (linear velocity drive is unchanged).

#### Angular velocity drive

The player body already uses linear velocity drive for movement. The same mechanism extends to yaw:

```rust
// In PlayerControlSystem, each frame:
let current_yaw = /* extract yaw from body.rotation() */;
let target_yaw = /* from mouse/camera input */;
let yaw_error = wrap_angle(target_yaw - current_yaw);
let target_angular_vel = yaw_error * turn_aggression; // proportional control
body.set_angular_velocity(Vector3::new(0.0, target_angular_vel, 0.0));
// (Or use a clamped velocity drive if available for angular)
```

The `turn_aggression` parameter controls how snappy the turn feels. High values = nearly instant when unloaded. When holding a heavy object, the `FollowPoint` constraint resists the rotation (via the angular Jacobian on the grab point), and the solver naturally limits the actual turn rate.

**Tuning:** The player capsule's moment of inertia around Y determines the baseline resistance. A capsule has relatively low Y-axis inertia, so turns will be snappy without load. The held object's inertia, coupled through the `FollowPoint` constraint at the grab point offset, provides the resistance.

#### Why this works

The `FollowPoint` constraint binds the object's grab point to the hold point on the player's capsule surface. The hold point is derived from the player body's physical rotation. When the player turns:

1. Angular velocity drive applies torque to the player body around Y.
2. The `FollowPoint` constraint couples the held object to the player's surface.
3. The held object's inertia resists the motion at the grab point.
4. The solver resolves the interaction — the player turns more slowly with heavy objects.
5. Everything is dynamic, no infinite-mass bodies, no special cases.

#### KeepUpright interaction

No conflict. KeepUpright constrains tilt (the two axes perpendicular to Y). The angular velocity drive operates on Y spin only. They are orthogonal.

#### Config

```rust
pub struct GrabConfig {
    // ... existing fields ...
    /// Proportional gain for the yaw angular velocity drive.
    /// Higher = snappier turns when unloaded.
    pub turn_aggression: f32,       // 10.0
}
```

This may alternatively live on `PlayerConfig` since it affects the player at all times, not just during grabbing. However, the turn behavior only changes the feel when holding objects — when unloaded, high aggression makes it indistinguishable from instant snap.

### GrabConfig

```rust
pub struct GrabConfig {
    pub grab_range: f32,            // 2.0   — max probe distance
    pub probe_radius: f32,          // 0.3   — swept-sphere radius
    pub hold_distance: f32,         // 1.0   — hold point forward offset
    pub hold_height: f32,           // 0.3   — hold point vertical offset
    pub compliance: f32,            // 0.0   — positional softness
    pub max_force: f32,             // 500.0 — positional impulse clamp
    pub angular_compliance: f32,    // 0.0   — angular softness
    pub angular_max_impulse: f32,   // 50.0  — angular torque clamp
    pub reach_duration: f32,        // 0.15  — reach animation time
    pub throw_impulse: f32,         // 1000.0 — throw forward impulse
    pub debug_draw: bool,           // true  — debug overlays
}
```

This is an ECS resource, since there's only one player.

---

## Constraint type: `FollowPoint`

A two-body constraint that drives a point on body_b toward a point on body_a, with angular orientation locking. Expands to 6 constraint rows: 3 positional (X, Y, Z) + 3 angular (X, Y, Z).

### `ConstraintKind`

```rust
ConstraintKind::FollowPoint {
    body_a: RigidBodyHandle,
    local_anchor_a: Vector3<f32>,
    body_b: RigidBodyHandle,
    local_anchor_b: Vector3<f32>,
    compliance: f32,
    max_impulse: f32,
    relative_orientation: UnitQuaternion<f32>,
    angular_compliance: f32,
    angular_max_impulse: f32,
}
```

`body_a` is the player, `body_b` is the held object. `local_anchor_a` is the hold point offset in the player's local frame (computed from `hold_distance` and `hold_height`). `local_anchor_b` is the surface grab point in the held body's local frame. `relative_orientation` is the snapshot `R_a⁻¹ * R_b` captured at grab time.

### Positional row expansion (rows 0–2)

For each axis `i` in {X, Y, Z}:

```
r_a = body_a.rotation() * local_anchor_a
r_b = body_b.rotation() * local_anchor_b
hold_point = body_a.position() + r_a
grab_point = body_b.position() + r_b

lin_jac_a = -unit_axis[i]       // moving A reduces the gap
ang_jac_a = -(r_a × unit_axis[i])
lin_jac_b = +unit_axis[i]       // moving B increases the gap
ang_jac_b = r_b × unit_axis[i]

error = grab_point[i] - hold_point[i]
```

The two-body formulation gives Newton's third law automatically — the solver applies equal and opposite impulses to both bodies. Heavy held objects resist the player's movement and turning.

### Angular row expansion (rows 3–5)

For each axis `i` in {X, Y, Z}:

```
target_b_rot = body_a.rotation() * relative_orientation
error_q = body_b.rotation() * target_b_rot⁻¹
angular_error = 2 * error_q.vector() * sign(error_q.w)

ang_jac_a = -unit_axis[i]
ang_jac_b = +unit_axis[i]
lin_jac_a = lin_jac_b = (0,0,0)   // pure angular, no linear coupling

error = angular_error[i]
```

The angular error is computed in world space (not body A's local frame) to match the world-space Jacobian axes. The `angular_max_impulse` clamp controls orientation stiffness: light objects lock orientation, heavy objects droop under gravity.

### Projection

`FollowPoint` does not need post-solve projection. PGS rows alone suffice.

### Warm-start

Not supported. Warm-start was tested and injects excess energy (cached impulses push in stale directions as bodies rotate between substeps). The expand function does not read warm-start impulses.

### Files

| File | Description |
|------|-------------|
| `src/physics/constraint/types.rs` | `FollowPoint` variant with two-body + angular fields |
| `src/physics/constraint/follow_point.rs` | 6-row expansion (3 positional + 3 angular) |
| `src/physics/constraint/expand.rs` | Match arm passing both bodies and angular params |
| `src/physics/constraint/projection.rs` | No-op match arm |
| `src/physics/constraint/mod.rs` | `mod follow_point` |

---

## Physics world: body-identifying raycast

The existing `ProbeTarget::swept_probe()` returns a `ProbeHit` but does not identify *which body* was hit. Grab needs to know the body handle.

### New method on `PhysicsWorld`

```rust
pub struct BodyProbeHit {
    pub body: RigidBodyHandle,
    pub hit: ProbeHit,
}

impl PhysicsWorld {
    /// Swept-sphere probe that identifies which body was hit.
    /// Skips static bodies and any body in `exclude`.
    pub fn probe_bodies(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
        radius: f32,
        exclude: &[RigidBodyHandle],
    ) -> Option<BodyProbeHit> { ... }
}
```

This is a refactor of the existing `ProbeTarget::swept_probe()` implementation — same loop over bodies/colliders, but returns the body handle alongside the hit. The existing `ProbeTarget` impl can delegate to this, discarding the handle.

---

## Grab logic

### Integration with PlayerControlSystem

Grab logic lives in `src/player/grab.rs` as helper functions called from `PlayerControlSystem` (renamed from `PlayerMotionSystem`). This keeps all player state transitions (locomotion and arm) in one place, avoiding split ownership of `PlayerState`.

`PlayerControlSystem` reads `PlayerTargetState` (intent) and owns all transitions on `PlayerState`. It handles both locomotion (grounded → airborne, etc.) and arm state (idle → reaching → holding). Grab helpers are called with `&mut PlayerState` and `&mut PhysicsResource`.

**Runs after:** `PlayerInputSystem` (so intent is available)
**Runs before:** physics step

### Per-frame logic

```
match player_state.arm:
    Idle:
        if grab_just_pressed:
            compute grab ray (player position + facing)
            probe = physics.probe_bodies(ray, exclude=[player_body])
            transition arm to Reaching { target: probe.map(|h| (h.body, h.hit.point)) }

    Reaching { elapsed, target }:
        elapsed += dt
        if elapsed >= reach_duration:
            if target is Some((body, hit_point)) AND grab still held:
                // Convert surface hit point to body-local offset
                let world_offset = hit_point - body.position()
                let local_anchor = body.rotation().inverse() * world_offset
                // Hold point derived from player body's physical rotation
                let hold_point = compute_hold_point(player_body, config)
                create FollowPoint constraint on body with local_anchor, target=hold_point
                optionally increase body angular damping
                transition arm to Holding { body, constraint }
            else:
                transition arm to Idle

    Holding { target_body, constraint }:
        if grab released:
            remove constraint
            restore body angular damping
            transition arm to Idle
        else if left_click just pressed:
            remove constraint
            apply throw impulse to body (facing direction * throw_impulse)
            restore body angular damping
            transition arm to Idle
        else:
            // Hold point tracks player body's physical rotation — no smoothing needed
            let hold_point = compute_hold_point(player_body, config)
            update constraint target to hold_point
            (grenade throw is suppressed — left-click consumed by throw)
```

### Hold point calculation

The hold point is derived from the player body's physical position and rotation — not the animation-level yaw. Since the player's turning is now physics-driven, this point moves smoothly through the solver.

```rust
fn compute_hold_point(player_body: &RigidBody, config: &GrabConfig) -> Point3<f32> {
    let pos = player_body.position();
    let facing = player_body.rotation() * Vector3::z(); // or extract yaw forward
    pos + facing * config.hold_distance
        + Vector3::y() * config.hold_height
}
```

No smoothing or interpolation is needed — the physics solver ensures the player body's rotation changes continuously, so the hold point never teleports.

### Updating the constraint target each frame

Each frame, the hold point is written to the `FollowPoint` constraint:

```rust
let hold_point = compute_hold_point(player_body, config);

if let Some(constraint) = physics.world.constraint_mut(handle) {
    if let ConstraintKind::FollowPoint { ref mut target, .. } = constraint.kind {
        *target = hold_point;
    }
}
```

The constraint expansion happens once per frame (in `update_contacts()`), so the updated target is picked up on the next physics frame.

---

## Animation integration

### Right hand override in `BipedController`

The grab logic computes a target position for the right hand and communicates it to the biped controller. The controller overrides its normal arm animation when a grab target is active.

#### New field on `BipedState`

```rust
/// If Some, the right hand is being driven to this world-space position
/// (overrides gait-based arm swing). Used by the grab system.
pub grab_hand_target: Option<Point3<f32>>,
```

#### Animation behavior by phase

**Reaching (swing-and-miss or pre-grab):**
The right hand swings forward toward the grab point over `reach_duration` seconds. This happens regardless of whether there's an object to grab — the animation is the same either way, which looks natural.

```rust
// During Reaching phase:
let reach_target = player_position
    + facing * config.hold_distance
    + Vector3::y() * config.hold_height;

// Lerp from current hand position toward reach target
let t = (elapsed / config.reach_duration).min(1.0);
biped.state.grab_hand_target = Some(lerp(current_hand_pos, reach_target, t));
```

**Holding:**
Right hand tracks the hold point (same as the constraint target, derived from the player body's physical rotation). The IK solver positions the elbow automatically.

```rust
biped.state.grab_hand_target = Some(hold_point);
```

**Idle / after release:**
```rust
biped.state.grab_hand_target = None;
```
Normal arm gait resumes.

#### Controller changes

In `update_idle_upper_body()`, `update_walking()`, and `update_falling()` — after computing the normal right hand position, check for an override:

```rust
if let Some(target) = self.state.grab_hand_target {
    self.state.right_hand.position = target;
}
```

This is a minimal change to `BipedController` — the existing IK solver handles elbow positioning automatically from the hand target.

---

## Input changes

### `GameplayActions`

```rust
pub struct GameplayActions {
    // ... existing fields ...

    /// Grab button held this frame.
    pub grab_held: bool,
    /// Grab button just pressed this frame.
    pub grab_just_pressed: bool,
    /// Grab button just released this frame.
    pub grab_just_released: bool,
}
```

### Binding changes in `from_input_state()`

```rust
// Jump — Space only (remove Right mouse button)
jump: input.is_key_just_pressed(KeyCode::Space),

// Grab — Right mouse button
grab_held: input.is_mouse_button_pressed(MouseButton::Right)
    && input.is_mouse_captured(),
grab_just_pressed: input.is_mouse_button_just_pressed(MouseButton::Right)
    && input.is_mouse_captured(),
grab_just_released: input.is_mouse_button_just_released(MouseButton::Right),
```

### Throw intent routing

Left-click is a context-dependent action: it throws a held object if grabbing, or throws a grenade if not. The intent flows through three layers:

1. **`GameplayActions`** — raw input mapping. `throw` (was `throw_grenade`) is true when left-click is just pressed. It does not know what kind of throw.
2. **`PlayerInputSystem`** — copies `GameplayActions::throw` onto `PlayerTargetState::throw`.
3. **`PlayerControlSystem`** — resolves `throw` based on arm state. If `Holding`, consumes it for a grab-throw. Otherwise, sets `PlayerTargetState::throw_grenade = true`.
4. **`GrenadeSpawnSystem`** — reads `PlayerTargetState::throw_grenade`. No `GameplayActions` dependency, no `ArmState` check needed.

This eliminates the ordering race between `PlayerControlSystem` and `GrenadeSpawnSystem` — `PlayerControlSystem` always runs first (via the dispatcher dependency chain) and the resolved `throw_grenade` flag is unambiguous.

---

## Weight and physics behavior

No artificial weight limit. The `FollowPoint` constraint's `max_impulse` bound naturally limits the force the grab can exert. Heavy objects:
- Move slowly toward the hold point (low acceleration from clamped impulse on high mass)
- May not reach the hold point at all if too heavy
- Pull the player toward them via Newton's third law (the solver applies equal and opposite forces)

This means the player gets "stuck" to very heavy objects and can't move, which is the desired behavior.

---

## Throw mechanics

When left-click is pressed during `Holding`:

1. Remove the `FollowPoint` constraint.
2. Apply an impulse to the held body: `facing * throw_impulse`.
3. The object is now free and flies in the facing direction.
4. Transition arm to `Idle`.

The impulse is applied via `body.apply_impulse()`, which correctly accounts for mass — a heavy box gets less velocity than a light beach ball from the same impulse, which feels physical.

**Throw impulse tuning.** The original value (15.0) was far too low — `apply_impulse` multiplies by `inv_mass`, so a 50 kg box got only 0.3 m/s. Bumped to 1000.0. Light objects (beach balls) now fly fast, which is balanced by `linear_damping` on the body (beach ball: 0.5). Air drag scales with `inv_mass`, so light objects bleed speed quickly while heavy objects are barely affected.

A throw animation for the right hand (forward flick) can be added later as polish, but the basic mechanic works without it.

---

## Edge cases

**Object destroyed while held:** If the held body is removed from the physics world (e.g. terrain destruction), `constraint_mut()` will return `None`. The grab logic should detect this and transition arm to `Idle`.

**Object goes to sleep while held / grabbing a sleeping object:** Handled by constraint-aware sleep (see below).

**Player dies / respawns while holding:** Reset arm state to `Idle` and remove the constraint if one exists.

**Grabbing the player's own body:** The `probe_bodies()` exclude list prevents this.

**Multiple grab attempts:** Only one object can be held at a time. Right-click while holding does nothing (the `Holding` branch doesn't check for right-click press).

**Locomotion transition while holding:** When a locomotion state transition would invalidate the current arm state (e.g. future Swimming), the transition method in `PlayerState` force-drops via the compatibility table — removes the constraint and resets arm to `Idle`.

---

## Constraint-aware sleep

The sleep manager must be aware of constraints. Without this, grabbing a sleeping body accumulates forces silently, and holding a body still lets it fall asleep while the constraint is still active.

### Changes (both in the physics engine, not grab-specific)

**1. `create_constraint()` wakes referenced bodies.**

When a constraint is created, wake all bodies it references. This is the moment the body becomes externally stimulated — the right place to trigger a wake regardless of constraint type.

```rust
// In PhysicsWorld::create_constraint():
for handle in kind.referenced_bodies() {
    if let Some(body) = self.bodies.get_mut(handle.0) {
        body.wake();
    }
}
```

**2. Sleep manager skips bodies with active constraints.**

During the sleep eligibility check, query the constraint arena for any active constraint referencing the body. If found, the body stays awake regardless of velocity history.

```rust
// In sleep eligibility check:
fn has_active_constraint(&self, body: RigidBodyHandle) -> bool {
    self.constraints.iter().any(|(_, c)| c.active && c.kind.references_body(body))
}
```

`references_body()` already exists on `ConstraintKind`.

### Limitation

This is a conservative policy: any active constraint prevents sleep. This is correct for grab (FollowPoint) and player upright (KeepUpright, though the player never sleeps anyway), but overly conservative for future two-body constraints where both bodies are at rest. For example, a "glue" constraint between two resting bodies should allow the pair to sleep together as a unit.

The proper fix for that future case is **constraint-aware island building**: constraints become edges in the contact graph, and entire islands (groups of bodies connected by contacts and constraints) sleep/wake as a unit. This is already noted as deferred work in `CONSTRAINT_SYSTEM_PLAN.md` ("Constraint islands for sleeping"). The conservative policy here is a correct interim solution until island-based sleep is implemented.

### Files changed

| File | Change |
|------|--------|
| `src/physics/world.rs` | Wake referenced bodies in `create_constraint()` |
| Sleep manager (wherever eligibility is checked) | Skip bodies with active constraints |

---

## Files summary

| File | Status | Description |
|------|--------|-------------|
| `src/player/components.rs` | Modify | Layered `PlayerState` (`LocomotionState` + `ArmState`). `ArmState::Holding` has `target_body` and `constraint` (no anchor body). |
| `src/player/grab.rs` | New | `GrabConfig`, grab/drop/throw helpers, `hold_point_local_anchor()`, `desired_hold_point()` |
| `src/physics/constraint/types.rs` | Modify | Two-body `FollowPoint` variant with angular orientation fields, 6 rows |
| `src/physics/constraint/follow_point.rs` | New | 6-row expansion: 3 positional + 3 angular |
| `src/physics/constraint/expand.rs` | Modify | `FollowPoint` match arm passing both bodies and angular params |
| `src/physics/constraint/projection.rs` | Modify | No-op match arm |
| `src/physics/constraint/mod.rs` | Modify | `mod follow_point` |
| `src/physics/world.rs` | Modify | `probe_bodies()`, `BodyProbeHit`, constraint-aware sleep |
| `src/input/actions.rs` | Modify | Add grab inputs, remove right-click from jump |
| `src/biped/state.rs` | Modify | Add `grab_hand_target: Option<Point3<f32>>` |
| `src/biped/controller.rs` | Modify | Apply hand override after normal arm animation |
| `src/biped/systems.rs` | Modify | `compute_grab_hand_target()` for reach/hold animation |
| `src/app/spawners/player.rs` | Modify | Use new `PlayerState` |
| `src/systems/player_input.rs` | Modify | Write grab intent to `PlayerTargetState` |
| `src/systems/player_control.rs` | New | `PlayerControlSystem`: locomotion + arm state transitions, angular velocity drive, grab logic, debug viz |

---

## Implementation order

1. **Player state refactor** — replace `PlayerMoveState` with layered `PlayerState` (`LocomotionState` + `ArmState`). Separate intent (`PlayerTargetState`) from actual state (`PlayerState`). Rename `PlayerMotionSystem` to `PlayerControlSystem`.
2. **Input changes** — add grab actions to `GameplayActions`, add `grab_held` to `PlayerTargetState`, remove right-click from jump.
3. **`FollowPoint` constraint** — types, expansion with `local_anchor` and angular Jacobians, wiring. Testable via bench harness.
4. **`probe_bodies()`** — body-identifying raycast on PhysicsWorld.
5. **Physics-driven turning** — angular velocity drive around Y in `PlayerControlSystem`. Derive hold point from player body's physical rotation. Hold point derivation should use the body's rotation even before grab is implemented, to validate the turning feel in isolation.
6. **Grab logic** — `src/player/grab.rs`: grab/drop/throw helpers, `compute_hold_point()`, integration into `PlayerControlSystem`. The `FollowPoint` constraint targets the hold point directly on the player capsule surface.
7. **Animation** — hand override on BipedController. Polish layer.

---

## Implementation notes

### What's done (2026-03-22)

Steps 1–4 and 6–7 from the original implementation order are complete, plus constraint-aware sleep and surface-point grabbing. The grab system is functional end-to-end: right-click probes, creates a FollowPoint constraint at the surface hit point with angular Jacobians, left-click throws, release drops, grenade is suppressed while holding, and the right hand animates toward the hold point. Sleeping bodies wake on grab, and constrained bodies are prevented from sleeping.

**Physics-driven turning implemented.** The player's yaw is now driven by an angular velocity drive through the physics solver. `PlayerControlSystem` extracts the current yaw from the physics body's quaternion, computes a yaw error against the target direction, and sets an angular velocity target via the `VelocityDriven` component. The `Rotation` component is coupled to the physics body's actual facing. `turn_aggression` (default 20.0) lives on `PlayerConfig`.

**Angular velocity drive implemented.** `RigidBody` now supports `angular_velocity_drive` alongside the existing linear velocity drive, using the same drive-before-external + target-shifting pattern in `integrate_forces`. The `VelocityDriven` ECS component carries `angular_velocity` and `angular_max_accel`. `PhysicsWorld::set_body_velocity_drive` accepts both linear and angular drive parameters.

**Player capsule tuned for physics-driven control.** Friction set to 0.0 (movement is entirely velocity-driven, no friction needed). Y-axis inertia scaled 50x via `scale_local_inertia` to prevent contact-induced yaw spin from wall collisions. Angular damping set to 0.95 (was 1.0, which zeroed angular velocity every substep and killed the angular drive).

### What's done (2026-03-23)

**Two-body FollowPoint constraint.** Replaced the kinematic anchor body (`HoldPointStrategy`) with a direct two-body constraint connecting the player body (`body_a` with `local_anchor_a`) to the held body (`body_b` with `local_anchor_b`). Newton's third law reaction forces are now automatic — the solver mediates turning resistance, movement drag, and all coupling between player and held object. The `HoldPointStrategy` enum, `anchor_max_speed` config field, and anchor body lifecycle have been removed.

**Angular orientation lock on FollowPoint.** The constraint now produces 6 rows: 3 positional + 3 angular. At grab time, the relative orientation `R_a⁻¹ * R_b` is captured as a snapshot. The angular rows drive the current relative orientation back toward this snapshot. The angular error is computed in world space (`R_b * (R_a * R_rel)⁻¹`) to match the world-space Jacobian axes. Separate `angular_compliance` and `angular_max_impulse` fields on `GrabConfig` control softness. Light objects (low inertia) lock orientation; heavy objects droop under gravity as the torque clamp is exceeded.

### Remaining work

All planned features are implemented. The grab system is feature-complete.

### Known limitations

**Warm-start not supported on FollowPoint.** Warm-start was tested twice (once with the old single-body constraint, once with the two-body constraint) and injected excess energy both times, causing arcing/oscillation. The `follow_point::expand` function does not accept warm-start impulses — `accumulated_impulse` is always zeroed. The write-back in `expand::write_back_constraints` still runs but the values are discarded on the next expand. Root cause is likely that the Jacobian directions change between substeps/frames as the bodies rotate, so cached impulses push in stale directions.

### Bugs found and fixed during implementation

**FollowPoint bias sign was inverted.** The initial implementation used `bias = -(beta / dt) * error` (matching KeepUpright's convention). However, the solver equation is `lambda = eff_mass * -(cdot + bias)`, which targets `cdot = -bias`. For a linear positional constraint where `error = body_pos - target`, this drove velocity proportional to the displacement (away from target). Fixed by flipping to `bias = (beta / dt) * error`. KeepUpright doesn't have this issue because its angular error and Jacobian have aligned sign conventions.

**Animation facing direction was mirrored.** `compute_grab_hand_target()` in `src/biped/systems.rs` used `(-sin(yaw), 0, cos(yaw))` while the rest of the codebase uses `(sin(yaw), 0, cos(yaw))`. This caused the hand to animate toward a mirrored position. Fixed by removing the stray negation.

**Two-body FollowPoint Jacobian signs were inverted.** When converting from single-body to two-body, the initial implementation used `lin_jac_a = +axis, lin_jac_b = -axis`. The correct signs for `error = point_b - point_a` are `lin_jac_a = -axis` (moving A reduces the gap) and `lin_jac_b = +axis` (moving B increases it). Same inversion on the angular Jacobians. Symptom: grabbing an object flung the player backwards.

**Angular orientation error was computed in the wrong frame.** The error quaternion `(R_a⁻¹ * R_b) * R_rel⁻¹` produces an axis-angle vector in body A's local frame, but the angular Jacobians are world-space axes. The correction pushed around the wrong axes, causing violent oscillation. Fixed by computing the error in world space: `R_b * (R_a * R_rel)⁻¹`.

### Structural choices

**PlayerControlSystem split into its own file.** The plan suggested renaming `PlayerMotionSystem` to `PlayerControlSystem` in the same file. Instead, `PlayerControlSystem` was extracted to `src/systems/player_control.rs` and `PlayerInputSystem` stays in `src/systems/player_input.rs`. This keeps the growing control system (locomotion + arm state transitions + grab logic) separate from the simple input-to-intent mapping.

**Throw intent routed through PlayerTargetState.** `GameplayActions::throw_grenade` was renamed to `throw` (context-neutral). `PlayerInputSystem` writes it to `PlayerTargetState::throw`. `PlayerControlSystem` resolves it: if `Holding`, consumes it for a grab-throw; otherwise sets `PlayerTargetState::throw_grenade = true`. `GrenadeSpawnSystem` reads only the resolved `throw_grenade` flag — no `GameplayActions`, `ArmState`, or `PlayerState` dependency. This replaced the earlier approach of checking `ArmState::Holding` in `GrenadeSpawnSystem`, which had an ordering race (grenade system always ran after player control, so the arm was already `Idle` on the throw frame).

**GrabConfig is a separate ECS resource.** Rather than adding grab parameters to `PlayerConfig`, grab configuration lives in its own `GrabConfig` resource in `src/player/grab.rs`. This follows the project's preference for moving configuration into the component that owns it.

**Debug visualization gated behind `GrabConfig::debug_draw`.** Shows probe ray (cyan line), probe tip (cyan sphere at max range), desired hold point (yellow sphere), hit point on surface during reach (orange sphere), and grab point on held body + constraint stretch line (green) when enabled.
