# Swimming — Design Notes

## Pre-reading

Before starting, read these in order:

- `CLAUDE.md` — project conventions and coding standards.
- `docs/ANIMATION_PROJECT.md` — design lessons distilled from the player
  moveset work. Most of those lessons apply verbatim here.
- `src/player/components.rs` — `LocomotionState`, `MovementRule`,
  `AirSteering`, `Timer`, the `tick` / `movement_rule` methods on
  `LocomotionState`.
- `src/player/config.rs` — `PlayerConfig`, where new tunables go.
- `src/systems/player_control.rs` — the driver that orchestrates the FSM,
  input-grace timers, and movement-rule application.
- `src/water/buoyancy.rs` — where `submerged_fraction` is computed per
  body. This is the physics hook you'll expose.
- Commit `2c5dc66` ("Expand player moveset: crouch/sprint, variable jump,
  long jump") — the long-jump addition is the template for adding a new
  maneuver to this FSM. Read the diff end-to-end before starting.

The existing FSM is small enough to hold in your head. Do that before
editing anything.

## Context

The physics engine already handles water: buoyancy is applied per-body
based on submerged fraction (see `src/water/buoyancy.rs`), and drag scales
with submersion too. If the player jumps in today, physics pushes them back
to the surface and airborne steering lets them drift around. That's the
"accidental fall-in" case.

What's missing is **deliberate control** — tuning swim speed independently
of air speed, diving underwater, and staying underwater while crouch is held.

## Guiding principle

The controller's job in water is **intent translation**, not physics.
Physics already owns buoyancy and drag. The controller just needs to:

1. Detect when the player is swimming (vs airborne/grounded).
2. Translate input into a 3D target velocity.
3. Steer toward that target at a modest acceleration so physics forces
   (buoyancy, drag) still do visible work.

Anything more — snapping velocity, overriding buoyancy, teleporting on
surface contact — fights the physics layer and will feel wrong. The player
will notice even if they can't articulate why.

## FSM integration

Add a new `LocomotionState::Swimming` variant, sibling to `Grounded` /
`Launching` / `CoyoteTime` / `Airborne`.

**Why a new state, not a flag on Airborne:** jump and crouch mean something
different in water. Jump → swim up; crouch → swim down. That's not a
tuning tweak, it's a different input-to-effect mapping — which is exactly
what `LocomotionState` variants encode. Also keeps future features
(stamina, swim animations, underwater abilities) local to one variant
rather than scattered `if submerged` checks throughout the driver.

Transitions:

| From     | To       | Condition                                  |
|----------|----------|--------------------------------------------|
| Airborne | Swimming | submerged fraction > `swim_enter_depth`    |
| Swimming | Airborne | submerged fraction < `swim_exit_depth`     |
| Swimming | Grounded | touched bottom (v2 — skip for v1)          |

`swim_enter_depth > swim_exit_depth` (hysteresis) to avoid waterline
flicker when floating at the surface. Boolean state transitions driven by
a continuous quantity always need a deadband; without it, physics noise
toggles the state every few frames.

## Movement rule — the key addition

Every existing `MovementRule` targets only planar (XZ) velocity; gravity
owns Y. Swimming needs a 3D target (input drives vertical motion too).

Add a new variant to `MovementRule`:

```rust
// In src/player/components.rs, alongside Direct / AirSteerClampUp / etc.
Volumetric { target: Vector3<f32>, accel: f32 }
```

Don't parameterize the existing struct — a new variant is the pattern used
for every other state-specific movement rule and keeps the cases distinct.

**Crucially: do not snap.** The existing `Locked` variant (used by long
jump) sets `accel = f32::INFINITY`, which in the driver snaps velocity.
In water that means infinite force overriding buoyancy and drag every
frame — exactly what the guiding principle forbids. `Volumetric` uses a
finite accel (around `swim_accel` in config) so physics still shapes
motion.

The driver's `apply_movement_rule` (in `src/systems/player_control.rs`)
will need a `Volumetric` arm that steers all three axes toward the target,
unlike the XZ-only rules.

## Input mapping in Swimming

Build the 3D target from input:

- **XZ**: `move_dir * swim_speed` (camera-relative, same derivation as
  `PlayerInputSystem` already does for walking).
- **Y**: `+swim_vertical_speed` if `jump_held`, `-swim_vertical_speed` if
  `crouch` (held, not just-pressed), else `0`.

Notes:

- Use `jump_held`, not `jump_just_pressed`. Holding jump continuously
  pushes up; releasing lets physics decide (buoyancy will carry you up
  anyway, just slower).
- Crouch is already "held" in `PlayerTargetState.crouch` — same rule.
  Release and buoyancy wins; you bob back up.
- Diving to stay under works because the target vy is negative and
  buoyancy fights it but doesn't overwhelm it (provided
  `swim_vertical_speed` is tuned high enough). No need to disable
  buoyancy.

## What to skip in the driver for Swimming

Grace timers (`jump_buffer`, `crouch_buffer`, `crouch_lockout`) and coyote
logic are meaningless in water. Add a method:

```rust
impl LocomotionState {
    pub fn uses_grace_timers(&self) -> bool {
        !matches!(self, LocomotionState::Swimming { .. })
    }
}
```

…and short-circuit the grace-timer / coyote sections in
`PlayerControlSystem::run` when it returns false. Cleaner than sprinkling
`matches!` checks at each point.

Jump cutoff, long-jump preconditions, and `air_speed` latching all fire
only in airborne states, so they'll naturally skip Swimming with no
changes to their own logic.

## Physics hook needed

`src/water/buoyancy.rs` already computes `submerged_fraction` per body.
Expose a read API — either:

- A method `PhysicsWorld::submerged_fraction(body: RigidBodyHandle) -> f32`
  that queries the water/buoyancy layer. Simple; one call per frame from
  the player controller.
- Or have the water coupling write the fraction onto a component
  (`SubmersionComponent` or similar) each physics step, and have the
  player controller read it from ECS. Slightly more ECS-idiomatic but
  more plumbing.

Start with the query API. If profiling shows it's a hot path, switch to
the component approach.

## Scope

### v1 — "more control"

Minimum viable swimming. Delivers tunable swim speed + deliberate
dive/surface. Should feel like the player has intent in water instead of
flailing.

- `LocomotionState::Swimming` variant.
- Submersion threshold transitions (Airborne↔Swimming) with hysteresis.
- `Volumetric` movement rule (3D target, finite accel).
- Input mapping: jump=up, crouch=down, move=planar.
- `uses_grace_timers()` driver short-circuit.
- Config additions: `swim_speed`, `swim_vertical_speed`, `swim_accel`,
  `swim_enter_depth`, `swim_exit_depth`.
- Physics hook exposing `submerged_fraction`.

Estimate: ~100 lines across `components.rs`, `config.rs`,
`player_control.rs`, plus a small physics getter. Mirror the structure of
the long-jump addition in commit `2c5dc66`: one new variant, one new
movement rule variant, one new arm in each method on `LocomotionState`,
driver handles the effect. Nothing existing should be edited beyond
adding match arms.

### v2 — polish and features

Add as actual gameplay requires them, not speculatively.

- **Surface vs underwater sub-mode.** If surface swimming needs distinct
  behavior (slower, arm-stroke cadence, head-above-water camera), split
  `Swimming { mode: Surface | Underwater }`. Start unified; only split if
  the feel demands it.
- **Breach / dolphin-leap.** Holding jump while surfacing adds an upward
  impulse at the waterline for a satisfying exit. Without this you just
  drift out.
- **Breath / stamina timer.** Separate `Timer` field on `PlayerState`,
  ticks down while fully submerged, resets on surface. Independent of the
  FSM.
- **Touch-bottom → Grounded.** If shallow-water wading is wanted, add a
  depth+contact check to transition Swimming → Grounded when feet touch.
- **Swim-specific animations.** Falls out of the animation-project work.
- **Underwater grab tweaks.** Reduced throw impulse, restricted reach.
  Either a multiplier in `GrabConfig` or a method on `LocomotionState`
  returning a grab-strength scale.

## Open-closed check

Adding Swimming should be purely additive. The implementation should only:

- Add one `LocomotionState` variant.
- Add one `MovementRule` variant.
- Add one driver short-circuit (via `uses_grace_timers`).
- Add one physics-layer getter.
- Add arms to the existing `tick` / `movement_rule` methods.

No edits to `Grounded` / `Launching` / `Airborne` / `CoyoteTime` logic, no
edits to long-jump code, no edits to jump-cutoff or buffer logic. If the
implementation forces changes to any of those, stop and ask — the design
is wrong somewhere.

## Open questions for implementation time

1. **Is the physics getter cheap enough to call every frame?** If it does
   significant work (triangle queries, collider iteration) it may be
   faster to have the water-coupling system write the fraction onto a
   component once per physics step and have the controller read from ECS.
2. **Should the `Swimming` variant carry depth / surface data?** Pro:
   available to animation and UI without re-querying. Con: must stay in
   sync with physics. Probably unnecessary for v1 — keep it a marker
   variant and query when needed.
3. **What's the right `swim_vertical_speed`?** Needs to be high enough to
   overcome buoyancy at typical submersion depths but low enough that
   surface swim doesn't rocket you skyward when jump is held. Tune
   empirically; a shallow test pool in a level is helpful.
