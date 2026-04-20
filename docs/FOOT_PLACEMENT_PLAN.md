# Foot Placement Plan

Design for a unified procedural foot placement system that replaces gait-driven foot trajectories with body-motion-driven stepping. Grounded in the inverted-pendulum biomechanics literature (Pratt et al. 2006, Koolen et al. 2012) rather than ad-hoc heuristics.

---

## Problem statement

The current animator produces foot positions as a byproduct of `PoseState::Grounded::sample` — the stride wheel + gait preset dictate where each foot goes each frame. Transitions between gaits (and between grounded / airborne) then rely on a crossfade to smooth the handoff. This architecture has three recurring symptoms, all of which surface as "feet in the wrong place":

1. **Walking → idle foot slide.** When the FSM switches from `Gait::Walk` to `Gait::Idle`, feet do not plant naturally under the hips. They snap to the most recent probe hit and the crossfade drags them across the terrain. The `anchor_feet_under_hips` helper was intended to fix this but `replant_foot` overwrites the anchor on the same frame (see `animator.rs:204-216`).
2. **Post-landing slide.** On landing, the capsule retains horizontal momentum and slides for several frames before coming to rest. The stride wheel is not advancing (character is idle per the intent mapping), so the feet are frozen in world space while the body slides past them.
3. **Incline slide.** On a slope, the capsule slides downhill under gravity. Same underlying issue as #2 — feet stay pinned while the body moves out from under them.

All three symptoms share a root cause: **the current system has no unified concept of "my planted foot is now in the wrong place; take a step."** Foot motion is driven only when a gait cycle is running, and gait cycles are driven only when intent-scaled speed is above `idle_threshold`. Any time the body moves without the gait FSM driving motion, the feet are orphaned.

### Why the FSM approach keeps hitting these cases

`PoseState::Grounded { gait }` conflates two orthogonal concerns: *intent classification* (is the player standing, walking, sprinting, crouching?) and *foot trajectory production* (where should each foot be this frame?). The first is a property of player input; the second is a property of pelvis motion in world space. When the two diverge — stationary intent but moving body, or switching intent mid-stride — the FSM has no clean answer and we accumulate if-else patches.

---

## Solution: capture-point stepping

Each foot runs an independent miniature FSM — `Planted` or `Stepping` — with transitions driven purely by the geometric mismatch between where the foot is planted and the **capture point** of the current body motion. The gait FSM no longer owns foot xy trajectories; it becomes a purely stylistic layer (step height, hip sway, torso pitch, arm swing).

This is the well-trodden purely-procedural approach (Overgrowth, Rain World; David Rosen's GDC 2014 talk is the canonical reference), grounded in the humanoid-robotics literature on capture-point stepping.

### The capture point

For a linear inverted pendulum of effective height `h` with horizontal CoM velocity `v`, the **capture point** is the xy location where a foot must be planted to bring the body to rest:

```text
capture_point = CoM_xz + v_xz * sqrt(h / g)
```

Derived by Pratt et al. 2006 ("Capture Point: A Step toward Humanoid Push Recovery") and generalised in Koolen et al. 2012 ("Capturability-based analysis and control of legged locomotion"). For this project:

- `CoM_xz` is approximated by the pelvis xz position.
- `h` is the effective pendulum height — pelvis y minus the support foot y. Falls back to `standing_height()` when no foot is planted.
- `v_xz` is the horizontal pelvis velocity supplied by physics.
- `g` is gravity magnitude.

The coefficient `sqrt(h/g)` is a derived constant (≈ 0.35 s for a 1.2 m pelvis under Earth gravity), not a tuned parameter. One knob replaces two.

**Why this is strictly better than `velocity * predict_time`:**

- **Push/slide recovery is physically correct.** On a hard landing slide, feet step to a location that actually stabilises the body. Heuristic prediction can't distinguish "walking" from "shoved" and places feet arbitrarily in both cases.
- **Slope behaviour falls out.** Gravity tilts the effective pendulum motion and the capture point shifts downhill by the biomechanically-correct amount.
- **Speed-to-stride relationship is principled.** Stride length emerges from `v * sqrt(h/g)`, which is the natural human relationship, not a tuning curve.

### Turn-in-place

Capture point assumes translational motion. If the player spins facing at zero velocity, `capture_point` collapses to the CoM and feet never step — legs would eventually cross. Real humans step around their CoM to match facing.

Fix: augment the ideal foot target with a **yaw-driven term**:

```text
ideal_xz = capture_point
         + facing_right * yaw_rate * k_yaw * hip_width
         + facing * sign_per_foot * hip_width
```

- The first yaw term pulls each foot sideways in proportion to turn rate — the foot on the turning-outside steps further out, mirroring how a human pivots.
- The last term is the neutral stance offset: each foot sits `hip_width` to its own side of the CoM.

A step can also be *triggered* by accumulated facing change, not only by planted-error magnitude — otherwise slow spins accumulate below threshold. Trigger on `max(planted_error, |Δfacing since plant| * hip_width * k_turn_trigger)`.

### Core loop (per foot, per frame)

```text
stride_offset = stride_gain · (capture_point - pelvis)
              + turn_in_place_offset(facing, yaw_rate)      // clamped to max_reach
ideal_xz      = pelvis + stride_offset + stance_offset(facing, foot_side)

symmetric_trigger = 2 · stride_gain · |v| · √(h/g)
trigger_threshold = max(symmetric_trigger, settle_trigger)

trigger = max(||planted_xz - ideal_xz||,
              |Δfacing_since_plant| · hip_width · k_turn_trigger)

Planted:
    if trigger ≥ trigger_threshold AND other_foot.is_planted:
        duration  = clamp(trigger_threshold / speed, min_step, max_step)
        travel    = (ceil(duration/dt) - 1) · dt · v      // discrete preview
        to        = ideal_xz + travel                     // land where the hip WILL be
        → Stepping { from: planted_xz, to, t: 0, duration }

Stepping:
    t += dt
    foot.position = swing_trajectory(from, to, t / duration, step_height(preset))
    if t ≥ duration:
        → Planted at `to`
```

Two quantities marked by the LIP: the **plant target** is at `stride_gain · v/ω` ahead of the hip (not at the full capture point — that would arrest motion; see `stride_gain` below), and the **trigger threshold** `2·stride_gain·v/ω` is exactly the distance at which the planted foot is mirror-symmetric about the hip. Preview shifts the target by the distance the hip will travel during the swing, so discretisation doesn't tilt the cycle forward.

`swing_trajectory` is a smoothstep horizontal + parabolic lift arc. Minimum-jerk refinement deferred.

#### Stride gain: planting short of the capture point

The capture point is where a foot planted *now* would arrest body motion. Plant exactly at CP → body stops. For continuous walking you need the body to pass *over* the foot, which means planting short of CP so the LIP's divergent component survives the step and the body keeps moving forward. `stride_gain ∈ (0, 1]` is the fraction of the CP offset the foot actually plants at. A per-gait knob on `GaitPreset`: walk ≈ 0.5, sprint ≈ 0.35, crouch ≈ 0.7. Lower gain = longer, more committed strides; higher gain = shorter, more controlled strides.

Because `symmetric_trigger` scales with the same quantity as `stride_offset`, lowering `stride_gain` shrinks both plant-ahead and trigger-behind distances together and the cycle stays symmetric around the hip.

### Why this unifies the three failure cases

1. **Walking → idle.** Speed drops to near-zero; capture point collapses onto the pelvis. Each foot's planted_xz is now offset from the pelvis by roughly half a stride — the next foot to exceed `step_trigger` takes one final step under the hip, then the other follows. No snap, no crossfade slide, no special case.
2. **Landing slide.** The body has horizontal velocity but no gait intent. Capture point sits ahead of the hip along the slide vector; feet step to catch up at whatever cadence the slide speed dictates. When momentum bleeds out, the last step lands on the pelvis-resting capture point.
3. **Incline slide.** Velocity vector points downhill under gravity; capture point shifts downhill by exactly the amount needed to stabilise against the slide. Same mechanism, no slope-specific logic.

Normal walking, sprinting, and push-recovery are the same mechanism scaled by input.

---

## Architectural change

### What goes away

- `stride_wheel` module's role as a trajectory driver. (Arm-phase reference moves into the upper-body ctx or derives from foot state directly.)
- `anchor_feet_under_hips` and `replant_foot` — both become redundant. The idle case is handled by the same loop as the moving case.
- `IDLE_PLANT_SNAP` hysteresis — `step_trigger` *is* the hysteresis, shared across all cases.
- The `is_idle_gait` transition branches in `update()`.
- Foot xy output from `PoseState::Grounded::sample`. Y (lift arc during Stepping, optional bob during Planted) stays with the pose layer; xy comes from the foot placer.

### What stays

- `PoseState` as an FSM over `{Grounded, Launching, Landing, Airborne}`. Its job shrinks: it drives *stylistic* channels and splice timers (anticipation, follow-through), not foot xy.
- `Gait` as a parameter (not an FSM) — its presets feed `step_height`, `step_trigger` multiplier, hip sway, torso pitch, arm amplitude. Exactly the "parameterize before splitting" guidance from `ANIMATION_PROJECT.md:8`.
- `UpperState` untouched.
- Crossfade infrastructure — still needed for gait-preset transitions (walk→sprint changes step height and torso pitch) and air↔ground handoffs. But it no longer has to blend foot xy, only stylistic channels, which is where crossfade is actually well-behaved.

### Module shape

```text
src/animation/foot_placer/
    mod.rs
    placer.rs          // FootPlacer: per-foot Planted/Stepping state + tick
    capture_point.rs   // pure math: capture point + turn-in-place
    swing.rs           // minimum-jerk swing trajectories + probe-based
                       // collision avoidance
```

`FootPlacer` is owned by `CharacterAnimator`, ticked once per frame *before* the pose FSM samples. Its output (current foot position + orientation + `is_stepping` flag per foot) feeds into the pose sample ctx, so stylistic layers can react (subtle lean during single-support, etc.).

```text
update() flow becomes:
  1. process_contacts              (unchanged)
  2. foot_placer.tick(dt, pelvis, velocity, facing, yaw_rate,
                      contacts, gait_preset)
  3. compute next PoseState / UpperState       (unchanged)
  4. begin crossfades if needed    (unchanged)
  5. sample pose + upper            (pose ctx now reads foot_placer output)
  6. blend crossfades               (unchanged)
  7. apply fragment to skeleton     (unchanged)
```

### Interaction with airborne states

Airborne feet are not planted. `FootPlacer` exposes `suspend()` that freezes step state when `PoseState` is `Launching` or `Airborne`. During the Landing splice, feet resume from wherever Airborne left them and are immediately eligible to step — planted error will almost always exceed threshold on the first post-landing frame, which is exactly what we want (feet catch up to the slide via capture-point stepping).

For anticipatory landing IK (the airborne feet-gap problem), `FootPlacer` exposes `reach_toward_ground(probe_hit, weight)`. Orthogonal to this plan; mentioned to confirm composition.

---

## Foot orientation (ankle IK)

Foot xz placement isn't enough — a flat-footed character on a 30° slope reads as wrong even if the target position is correct. Ankle orientation needs to align with the local ground during planting and relax to neutral during swing.

Per foot, maintain a `foot_orientation: UnitQuaternion<f32>` that slerps between:

- **Neutral** — pelvis-aligned, flat. Used during the middle 70% of swing.
- **Ground-aligned** — rotation that maps `(0,1,0)` onto the contact normal, preserving heading. Used when planted and during the final 15% of swing (foot lock-in).
- **Takeoff-aligned** — interpolated toward neutral during the first 15% of swing, away from the previous ground-aligned pose.

Ground normal comes from `FootState.ground_normal` (already populated in `process_contacts`). Lerp rates are per-preset (crouched walk has stronger ground-hugging than sprint).

This is a small addition — ~40 lines — but it's the difference between "feet are in the right place" and "character is actually walking on this terrain."

---

## Swing-leg collision (known gap)

A minimum-jerk arc from A to B can still clip terrain: stepping onto a raised block, walking up stairs, or swinging over a lip of geometry. Fully solving this is swing-trajectory optimisation (expensive, not justified here). The pragmatic mitigation:

**Predictive probing.** At step start, fire 2–3 probes along the planned swing arc (at t = 0.3, 0.6, 0.85). If any probe registers a contact above the current arc height, raise the peak lift of the committed swing to clear it, or offset the midpoint laterally. This handles the common voxel-world case (stepping onto a block) without requiring a full obstacle-avoidance solver.

Leg-on-leg collision (swing foot passing through the stance leg) is prevented implicitly by the single-foot-stepping invariant plus stance offset — swing paths never pass the CoM centreline toward the opposite hip.

This is a Stage-3+ refinement; not required for the initial cutover.

---

## Tunables

Two layers. Per-gait style knobs live on `GaitPreset`; placer-wide machinery lives on `FootPlacerConfig`.

### `GaitPreset` (per-gait)

| name | rough default | effect |
|---|---|---|
| `stride_gain` | walk 0.5 / sprint 0.35 / crouch 0.7 | fraction of CP the foot plants at. Drives both plant-ahead and trigger-behind distance symmetrically. Lower = longer strides, less controlled; higher = shorter strides, more controlled. |
| `step_height` | walk 0.15 / sprint 0.18 / crouch 0.06 | peak foot lift during swing. |
| `pelvis_crouch_offset`, `torso_pitch`, `head_bob_amplitude`, `arm_swing_amplitude`, `shoulder_twist_max` | — | stylistic channels, unrelated to stepping. |

### `FootPlacerConfig` (rig-wide)

| name | rough default | effect |
|---|---|---|
| `settle_trigger` | 0.05 m | always-on trigger floor. Dominant only at very low speeds; at rest it pulls off-centre feet to neutral stance. |
| `k_yaw` | 0.4 | how much yaw rate pulls the ideal sideways during turn-in-place. |
| `k_turn_trigger` | 0.2 | how much accumulated facing change (in rad · hip_width units) contributes to the trigger. |
| `min_step_duration` | 0.08 s | floor on a step's airtime. |
| `max_step_duration` | 0.1 s | ceiling on a step's airtime. |
| `max_stride_reach_ratio` | 1.5 | `stride` (capture + turn) clamped to this × leg length. `stance` is preserved through the clamp — otherwise lateral foot spacing would collapse at speed. |

The main trigger is `2 · stride_gain · |v| · √(h/g)`, not a configured value — it's derived from rig geometry and world gravity. `settle_trigger` is just a floor.

Step duration is `clamp(trigger_threshold / speed, min, max)`. At steady walking speed the symmetric trigger dominates and duration ≈ clamped near `max`, giving a consistent cadence.

### Safety invariants

- Only one foot may be `Stepping` at any time. Second trigger waits.
- A `Stepping` foot is never interrupted by a new target — it always completes to its committed `to`. Target prediction bakes in capture point + swing-preview at step start; minor error over retargeting mid-step. (If this proves wrong on sudden direction changes, add a retarget hook.)
- Vertical ankle position during `Planted` tracks pelvis-relative `ankle_y = pelvis.y − standing_height` every frame (`sync_planted_y`). Freezing y at plant time left feet stuck above or below the terrain after landing recoil; the per-frame sync lets feet follow the body's vertical settle.

---

## Migration plan

Staged so each step is independently testable and the character stays animating at every intermediate state.

### Stage 1: Introduce `FootPlacer` in parallel ✅

Module with capture-point math and turn-in-place, wired into `update()` with a debug overlay but not driving the skeleton.

### Stage 2: Switch foot xz source ✅

`PoseState::Grounded::sample` now reads feet from `AnimationState.left/right.position`, mirrored from the placer each frame. `anchor_feet_under_hips`, `replant_foot`, `IDLE_PLANT_SNAP`, and the `is_idle_gait` branches in `update()` are gone. See the Implementation Notes below for the physically-grounded refinements that followed (stride gain, preview, planted-y sync, airborne resume).

### Stage 3a: Ankle IK ✅

Foot-orientation slerp landed. `PlacerFoot::up` / `takeoff_up` plus a three-segment swing target (takeoff-ease → neutral → landing-ease) drive a per-frame exponential chase toward the ground normal. Mirrored into `FootState::up` and consumed by `add_foot_capsule_to_mesh`, which now builds its basis from the foot's own up-axis so capsules tilt with slopes/stairs.

### Stage 3b: Swing-leg collision

Predictive probes along the committed swing arc to lift the peak over obstacles. Deferred until clipping is actually visible in play.

### Stage 4: Collapse the stride wheel ✅

Stride phase is now derived from the placer's per-foot `Stepping { t, duration }` state. Right swinging → phase in `[0, PI]`; left swinging → `[PI, TAU]`; both planted → phase holds so arms coast rather than snap to rest. `stride_wheel.rs` is gone; `stride_sync.rs` retains only the phase-consuming helpers (`phase_from_placer`, `update_hand`, `compute_shoulder_twist`, `compute_head_bob`, `compute_head_tilt`). Foot-probe swing selection now queries `placer.left/right.is_planted()` directly. `AnimationState::wheel_angle` renamed to `stride_phase`. `Gait::drives_stride_cycle` deleted (no longer needed).

### Stage 5: Pelvis planner — deferred

Implemented and reverted. The math worked (vertical `amp · 0.5 · (1 − cos(2·phase))`, lateral `−sin(phase) · right · amp`, anticipatory forward lean during the first third of a step, all scaled by `stride_activity`). The problem is frequency: with `min/max_step_duration = 0.08–0.10 s`, the stride cycle runs at 5–6 Hz and the vertical channel peaks at 10–12 Hz. At a 30 Hz monitor that's 2–3 frames per cycle — below anything that reconstructs as smooth motion; it aliases into flicker.

Torso lean is a separate concern and already parameterised on `GaitPreset::torso_pitch` (walk = 0.08, sprint = 0.20, crouch = 0.30 rad). No planner needed for that channel.

Revisit conditions: if the gait cadence slows (longer `min/max_step_duration`) or the render rate goes up, or if we want pelvis motion specifically for slow walks, rebuild the module — the derivation is preserved above. An amplitude-gate on step duration (`smoothstep(0.08, 0.25, step_duration)`) would let it turn on automatically once the cadence drops into the visible band.

### Stage 5 (original): Pelvis planner — reference design

This is the stage that distinguishes "placed correctly" from "walks convincingly." Add a `PelvisPlanner` component that *computes* pelvis offset relative to the physics-supplied capsule position, rather than treating pelvis as a fixed input.

The pelvis in real walking is not static — it traces an inverted-pendulum arc: rises over the support foot, falls between steps, shifts laterally into single-support, and leans slightly in anticipation of the next step. The planner reads `FootPlacer` support state (which foot is planted, stepping phase, CoM vs support polygon) and produces:

- **Pelvis y offset** — sinusoidal rise/fall per stride cycle; amplitude from preset.
- **Pelvis lateral sway** — shifts toward the support foot during single-support; computed from which foot(s) are planted.
- **Pelvis anticipatory lean** — small forward lean during the first third of a step (weight shifting toward swing target).

`PoseState::Grounded::sample` already emits a `pelvis_offset`; the planner's output composes with it the same way. The difference is that the offset is now derived from foot-support state, not from a gait preset's crouch depth.

This stage is what pushes the system past Overgrowth-tier toward biomechanically-grounded procedural locomotion. It's also the stage that will make the character's upper body move the way people instinctively expect — head bob, shoulder roll, arm swing all react to pelvis arc automatically because they're all pelvis-relative.

### Stage 5.5: Torso-yaw decoupling (follow-up)

Turning in place currently pins visible torso yaw to the physics/input yaw, so the upper body rotates continuously while feet step discretely — one foot always looks stretched until the next trigger catches up. Fix by giving the animator its own `rendered_yaw` that chases `input_yaw` with a cap (`max_yaw_offset_rad`, on the order of 15–25°). When the cap hits, force a step; when the step completes, let `rendered_yaw` relax back toward `input_yaw`. The `stance_offset` and `capture_point` math then run against `rendered_yaw`, so the placer sees the yaw the feet actually believe in. Independent of Stage 5, but shares the pelvis planner as a natural home.

### Stage 6: Landing anticipation (optional follow-up)

Add the `reach_toward_ground` hook for the airborne feet-gap case. Natural next step once `FootPlacer` owns foot position.

---

## Implementation notes (for future sessions)

Stages 1 and 2 are landed. The placer owns foot xz for all grounded states; the stride wheel is reduced to a head-bob / head-tilt / arm-swing reference. Airborne states (`Launching` / `Airborne`) suspend the placer; `Landing` samples feet from the landing pose directly (pelvis-tracking at impact `ground_y`).

### Current state

**Code in place (`src/animation/foot_placer/`):**
- `mod.rs` — re-exports
- `config.rs` — `FootPlacerConfig` (lives on `CharacterRigConfig::foot_placer`)
- `capture_point.rs` — pure math: `capture_point_xz`, `turn_in_place_offset`, `stance_offset`. `GRAVITY` const = 9.81 (matches `PhysicsConfig::gravity` default magnitude).
- `swing.rs` — `swing_position`: smoothstepped horizontal + parabolic lift. Minimum-jerk refinement deferred.
- `placer.rs` — `FootPlacer`, per-foot `Planted`/`Stepping` FSM. Trace tests at the bottom under `#[cfg(test)] mod trace` — run with `cargo test --lib foot_placer::placer::trace -- --ignored --nocapture`.

**Integration (`src/animation/animator.rs`):**
- `update()` flow: `process_contacts` → compute `next_pose` → `tick_foot_placer(next_pose)` → `mirror_placer_into_state` → sample pose/upper with placer output already baked into `AnimationState.left/right.position` → blend crossfades → apply to skeleton.
- `FootPlacer::set_suspended(true)` while `next_pose` is `Launching` / `Airborne`. On the resume edge (`true → false`) the next tick calls `replant_at_stance`, snapping both feet to neutral stance under the current pelvis. Without this, airborne→grounded resumes from pre-takeoff foot positions.
- `stride_gain` comes from `gait_preset_for(next_pose)`; airborne-adjacent states fall back to the walk preset (placer is suspended there anyway).

**Debug overlay (`src/animation/systems.rs::draw_foot_placer_overlay`):**
- Green sphere = `ideal_xz` (stride_gain · capture point + stance + turn offsets)
- Yellow sphere = `planted_position`
- Magenta line = planted → ideal (error vector)
- Orange sphere + line = mid-swing position and from→to, only while `Stepping`

### Refinements landed on top of the Stage-2 skeleton

Each was a visible symptom discovered in play, with a physically-grounded fix:

1. **`stride_gain` (per preset).** Foot stayed in front of torso at all times. Planting at the full capture point is the *stopping* foothold (LIP → zero divergent component), so the body can't walk over it. Scaling the CP offset by `stride_gain ∈ (0,1)` preserves the divergent component and lets the hip pass over the planted foot.
2. **Symmetric, speed-proportional trigger.** Foot either dragged too far behind or plant-ahead/trigger-behind distances were asymmetric depending on whether fixed `step_trigger` or `2·gain·v/ω` dominated. Fixed by using only `max(symmetric_trigger, settle_trigger)` — the two quantities compose additively across the speed range, with `settle_trigger` acting as a rest-stance floor and `symmetric_trigger` owning everything above a crawl. `step_trigger` as a separate knob is gone.
3. **Reach clamp preserves stance width.** Applying `max_stride_reach` to `cp + turn + stance` uniformly shrank lateral foot spacing at speed (clamped vector scaled x *and* z). Clamp the stride portion only; always add `stance` after.
4. **Swing preview.** Foot plants at `+s` ahead of hip *at plant time*, not at trigger time. Without preview, hip travels `v·duration` during swing and the cycle tilts backward by that amount. `to = ideal + velocity · (ceil(duration/dt) − 1) · dt`. Discretisation matters: using `v·duration` naively overshoots by one tick's worth of motion (`v·dt`), tilting the cycle forward at high speed.
5. **Planted-y syncs to pelvis every frame.** Freezing `planted_position.y` at plant time left feet stuck above/below terrain after landing recoil or any physics y-settle. `sync_planted_y` pulls planted y to `ctx.ankle_y` each tick. Swing arc still owns y during `Stepping`.
6. **Airborne resume.** Suspending the placer in air + resuming from wherever the feet happened to be at takeoff snapped feet backward on landing. Resume path re-plants both feet at stance under the current pelvis (`replant_at_stance`, driven by a `resuming` flag latched in `set_suspended`).
7. **Ankle IK (Stage 3a).** Per-foot `up` and `takeoff_up` on `PlacerFoot`; target is phase-dependent (planted → ground normal; swing → takeoff-ease → neutral → landing-ease across configurable fractions). Exponential chase at `FootPlacerConfig::ankle_slerp_rate`. Mirrored into `FootState::up` and consumed by `add_foot_capsule_to_mesh`, which now builds its basis from the foot's own up-axis. Avoided full quaternion slerp — for the angles involved (slopes, not somersaults), lerp-and-renormalise on the up vector is indistinguishable.
8. **Stride phase from placer (Stage 4).** `AnimationState::wheel_angle` → `stride_phase`. Derived from placer's per-foot stepping state: right swinging → `[0, PI]`, left swinging → `[PI, TAU]`, both planted → hold. `stride_wheel.rs` collapsed into `stride_sync.rs` (phase derivation + phase-consuming helpers only). Probe aim (`configure_probes`) now queries `placer.foot.is_planted()` directly. `Gait::drives_stride_cycle` deleted.
9. **Stride activity (decouples upper body from gait FSM).** Added `AnimationState::stride_activity` ∈ `[0, 1]`: snaps to 1 while any foot is stepping, exp-decays (τ ≈ 0.25 s) when both planted. Scales arm-swing amplitude, shoulder twist, and head bob. `PoseState::cycle()` now returns `Some(Stride)` for all `Grounded` variants (including `Idle`); `natural_hands` always computes both rest-hang and swinging targets and lerps by activity. Fixes slope-slide flicker where gait oscillated Idle↔Walk across `idle_threshold` and hands/head snapped between poses even though the placer was stepping correctly.
10. **Head bob cadence.** Original `cos(phase).abs()` gave two bobs per stride cycle (one per foot strike). Replaced with `(1 − cos(phase)) · 0.5` — one bob per full cycle (peak at `phase = PI`).
11. **Per-foot terrain-following y.** Planted y was synced to a shared `pelvis − standing_height`, so feet stayed flat across slopes and stairs. `PlacerCtx` now carries `left_ground_y` / `right_ground_y` (`Option<f32>` from `FootState.ground_contact.y`); `sync_planted_y` uses the per-foot terrain surface when a probe has hit, falling back to the pelvis-relative value otherwise. Explicitly called out as a known gap in Tradeoffs but never wired until now.
12. **Foot semantic: centre, not ankle; sole submerged by one radius.** `add_foot_capsule_to_mesh` used to draw the capsule *below* `position` with `position.y` as the top tangent — i.e. `position` was conceptually the ankle and the sole sat `2·FOOT_CAPSULE_RADIUS` underneath. The capsule is now drawn centred on `position`, so `position.y` is the foot centre and the sole sits one radius below. Combined with (11), this gives a half-submerged look on terrain that reads better on voxel geometry than a capsule balanced exactly on the surface. Knock-on fixes: `landing_ground_y` snapshot in `animator.rs` no longer adds `FOOT_HEIGHT` to the probe hit (would have floated landing feet a full diameter up); `PlacerCtx::ankle_y` renamed to `foot_y_fallback` with semantic now "terrain surface under the rest pose" = `pelvis − standing_height − FOOT_HEIGHT`; `FOOT_HEIGHT` / `FOOT_CAPSULE_RADIUS` doc blocks updated. Side effect: hip→foot distance grew by `FOOT_HEIGHT`, so IK reaches further and legs look more extended at rest — intentional, matches the submerged look, but the rig's `standing_height_ratio` may want trimming by ~`FOOT_HEIGHT / leg_length` if knees start locking.

### Gotchas already hit (don't re-burn these)

1. **Coordinate convention.** `facing.cross(&Vector3::y())` is "right" and points to **−x when facing +z**. `FootPlacer::new` must initialise feet via `stance_offset` with the same sign convention, or left/right start swapped and every ideal pulls feet across the body.
2. **Trigger-before-advance.** `try_trigger_step` must run *before* `advance_stepping`. If advance runs first, a completing foot transitions Stepping→Planted mid-tick and its now-huge error retriggers it the same frame, starving the other foot.
3. **Alternation preference.** At high speed, a freshly-planted foot's error immediately exceeds the trigger. Whichever foot is evaluated first re-fires forever. Fixed by tracking `last_planted_side` and evaluating the *other* side first.
4. **No trigger floor larger than `settle_trigger` at speed.** A floor like the old `step_trigger` broke the symmetric-plant property: `stride_offset` scaled with gain but the trigger stayed pinned at the floor, so the foot ended up further behind the hip than it was ahead.
5. **Don't project `to` by raw `duration`.** Use `(ceil(duration/dt) − 1) · dt` — see refinement 4 above.

### Known-open follow-ups

- **Non-zero spawn yaw and variable dt** are not exercised by the trace harness. Real game inputs should be covered before the next major change.
- **Swing-leg collision (Stage 3b).** Ankle IK landed; swing-arc clipping on raised voxels / stairs is still open.
- **Pelvis planner (Stage 5).** Pelvis is still a passive physics output; biomechanical rise/fall/sway not yet derived from support state.
- **Mid-step retargeting.** Currently we commit `to` at trigger and never retarget. Sharp direction reversals mid-swing could look wrong; not yet observed in gameplay.

### Resume checklist

Before touching any code:
1. `cargo test --lib foot_placer::placer::trace -- --ignored --nocapture` — confirm the walk/sprint/landing/walk-to-idle traces still produce symmetric `±s` foot offsets around the hip.
2. Run the game, watch the debug overlay. The ideal-sphere (green) sits roughly `stride_gain · v/ω` ahead of the hip; planted-sphere (yellow) should oscillate between `+s` ahead and `−s` behind symmetrically.
3. Skim `animator.rs::tick_foot_placer` and confirm the airborne-suspend branch still matches current `PoseState` variants.

---

## Tradeoffs & open questions

**This is a real refactor, not a patch.** The current `PoseState::Grounded::sample` produces a complete foot trajectory; the new design requires splitting that across two layers and accepting that the layered system has its own tuning surface. The capture-point math itself is ~10 lines; the work is in restructuring who owns what.

**Gait presets become thinner.** A preset now controls `step_height`, `stride_gain`, and stylistic channels only. Stride length emerges from `stride_gain · v · √(h/g)`, not from a preset-supplied `stride_length`. Different gaits distinguish themselves by picking different `stride_gain` values — crouch uses a higher gain (shorter, more controlled strides), sprint a lower gain (longer, more committed strides). `GaitPreset.stride_length` and `frequency_mul` still exist but are dead code; safe to delete.

**No retargeting mid-step** may look wrong on sharp direction changes. Low-risk to start without; easy to add later if the capture point shifts significantly during a step's swing phase.

**Per-foot independence on uneven terrain.** With both feet running independent mini-FSMs, ground-contact y per foot can differ. This is a feature (one foot on a step, one on the floor) but means foot rendering must use each foot's own contact y, not a shared ankle_y. `FootState.ground_contact` already supports this.

**Capture point assumes a linear inverted pendulum.** Real human walking is closer to a compass gait + pendulum; the LIP approximation loses accuracy at high speeds and on steep slopes. For a stylised voxel character this is well within acceptable error. Full compass-gait math is Stage-N+ territory if ever needed.

---

## What this is *not*

- Not motion matching. No animation database, no feature-vector search.
- Not a learned controller. No RL, no mocap imitation.
- Not full-body dynamics. Capsule motion is still owned by the physics engine; this system reacts to where the capsule ends up.
- Not a leg-IK rewrite. Knee-bend IK is unchanged; this only affects the *target* position the IK solves toward.

The explicit design axis: **purely procedural, biomechanically grounded.** Upgrades from heuristic procedural animation (Overgrowth tier) to inverted-pendulum-grounded procedural animation without crossing into data-driven or learned territory.

---

## References

- Pratt, J., Carff, J., Drakunov, S., Goswami, A. (2006). *Capture Point: A Step toward Humanoid Push Recovery.* IEEE-RAS Int'l Conf. Humanoid Robots. [The capture-point paper.]
- Koolen, T., de Boer, T., Rebula, J., Goswami, A., Pratt, J. (2012). *Capturability-based analysis and control of legged locomotion.* Int'l Journal of Robotics Research. [N-step capturability generalisation.]
- Rosen, D. (2014). *An Indie Approach to Procedural Animation.* GDC talk. [The canonical procedural-locomotion game-dev reference.]
