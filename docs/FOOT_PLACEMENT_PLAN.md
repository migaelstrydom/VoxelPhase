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
    placer.rs          // FootPlacer: substep loop + per-foot Planted/Stepping FSM
    timing.rs          // GaitTiming: trigger/cycle/duty/swing derived from speed
    clock.rs           // GaitClock: phase clock + latched stance-window releases
    capture_point.rs   // pure math: capture point + turn-in-place
    swing.rs           // swing arc: smoothstep horizontal + parabolic lift
    sim.rs             // (test) scenario simulator over analytic terrain
    scenarios.rs       // (test) named scenario registry (terrain + inputs)
    trace.rs           // (test) stdout traces + CSV export for plotting
    invariants.rs      // (test) asserting scenario sweeps
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
| `max_substep_dt` | 1/240 s | upper bound on the placer's internal simulation step (frame split into equal substeps). |
| `intent_speed_floor` | 0.5 m/s | gait-clock speed floor while movement intent is held — lets the first step fire before physics catches up. |
| `moving_speed_threshold` | 0.15 m/s | speed above which the body counts as moving without intent (slides); filters physics jitter. |
| `settle_trigger` | 0.05 m | always-on trigger floor. Dominant only at very low speeds; at rest it pulls off-centre feet to neutral stance. |
| `k_yaw` | 0.4 | how much yaw rate pulls the ideal sideways during turn-in-place. |
| `k_turn_trigger` | 0.2 | how much accumulated facing change (in rad · hip_width units) contributes to the trigger. |
| `min_step_duration` | 0.12 s | floor on a step's airtime (≥ 3–4 display frames at 30 Hz). |
| `max_step_duration` | 0.4 s | ceiling on a step's airtime; also the duration of idle settle steps. |
| `max_leg_stretch_ratio` | 1.15 | Maximum hip→foot distance as a fraction of leg length (slightly >1 allows heel/toe extension). Drives both the horizontal stride budget for ideal targets (Pythagoras with `standing_height`) and the overstretch release that forces a planted foot to step when the hip slides too far from it. `stance` is preserved through the stride clamp — otherwise lateral foot spacing would collapse at speed. |
| `takeoff_stagger_fraction` | 0.4 | Minimum time between any two takeoffs while moving, as a fraction of the current swing duration. Prevents simultaneous releases from phase-locking the feet into a two-footed hop (gotcha 11). |
| `min_stance_fraction` | 0.6 | Minimum stance age before a non-overstretch release may fire, as a fraction of the current swing duration. Kills one-frame stances when a turn trigger re-arms the instant a foot lands mid-turn. Swing-relative so legitimate short sprint stances are unaffected. |
| `overstretch_hard_margin` | 0.03 m | Stretch past the budget at which an opening leg fires even inside the takeoff stagger window — bounds the worst-case stretch at sprint-speed reversals. |
| `swing_obstacle_clearance` | 0.03 m | Minimum mid-swing height above the probed surface, faded to zero at the endpoints (Stage 3b clamp). |

The main trigger is `2 · stride_gain · |v| · √(h/g)`, not a configured value — it's derived from rig geometry and world gravity. `settle_trigger` is just a floor.

Step duration is `(1 − duty_factor) · cycle_time` clamped to `[min, max]` — the swing fills its share of the gait cycle (see `timing.rs`), rather than being an independent knob that can desync from the cadence.

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

### Stage 3b: Swing-leg collision ✅

Landed as terrain-aware swings rather than extra probes along the arc — the
single existing foot probe is re-aimed at the *landing target* while
stepping (`PlacerFoot::probe_anchor`), giving the swing a live height
estimate of where it will plant. Three mechanisms compose:

1. **Landing-height chase.** `to.y` chases the probed floor height
   (extrapolated along the contact's tangent plane to the target xz) for
   the *entire* swing — a vertical correction cannot skate, but a stale
   height pops at plant. Initialised at step start from the same plane,
   so uniform slopes are exact from the first substep.
2. **Apex raise.** When the landing is higher than the takeoff, peak lift
   becomes `step_height + rise/2`, so the arc clears the higher tread by
   a full step height rather than just the lerp baseline's midpoint.
3. **Clearance clamp.** The rendered swing y is clamped to the probed
   surface plus `swing_obstacle_clearance · sin(π·u)` — risers and bumps
   between the endpoints push the foot over instead of cutting through.
   Faded at the endpoints so takeoff and plant stay on the surface.

All three gate on a floor-like contact (`normal.y > 0.6`); wall hits do
not steer swing heights. Pinned by `feet_never_clip_terrain` and
`plants_land_without_vertical_pop` across the whole scenario registry
(stairs, rough ground, hills, 20% inclines both directions).

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

### Stage 7: Phase authority ✅ (the antiphase fix)

In-game finding (2026-06): on triangulated terrain the legs settled into
a *stable* off-antiphase split — takeoffs at 0.6π/1.4π instead of π/π —
whose **antiphase error** varied with run direction, only hopping between
attractors when a random early step occurred. Diagnosed from real
recordings (see record-and-replay): the time-based `min_stance` gate
delayed one foot's scheduled release every cycle (predicted block of
0.231 s matched the observed 0.234 s takeoff gap), and
`resync_to_takeoff` then *adopted* the corrupted timing as the new
schedule — a self-consistent limit cycle.

Fix — the clock holds timing authority; landed as four changes:

1. **No takeoff resync.** The phase free-runs at ω(speed); step events
   never re-anchor it. A reactive (turn/overstretch) or gate-delayed step
   costs one odd stance and the fixed schedule pulls the foot straight
   back. The phase is *set* only where no rhythm exists: the idle→moving
   edge and the landing replant, both via `seed_to_release` (one foot
   released now, the other exactly half a cycle later).
2. **Scheduled releases drop the stance-age gate.** Under a phase-true
   schedule, stance ≥ duty·cycle holds by construction; the wall-clock
   gate was the thing fighting the schedule. (`min_stance` remains for
   Turn and idle-settle steps.)
3. **Overstretch fires on actual stretch only.** The predictive lead
   (`distance + radial·gap_remaining`) read 0.3 m into the future at
   sprint speed and, with sticky latches, re-paced the gait reactively.
4. **Cadence respects reach** (`STRIDE_REACH_SAFETY` in `timing.rs`).
   The stride is capped so the scheduled plant-ahead `2·duty·s` fits
   0.9× the horizontal reach budget — short legs at speed take faster,
   shorter steps instead of letting the stretch release pace the gait.
   The budget itself now uses the true rest vertical
   (`pelvis.y − foot_y_fallback`, ≈ standing_height + FOOT_HEIGHT); the
   bare `standing_height` overstated reach by ~30%. `ideal_target` caps
   its capture-point term at the same plant-ahead so planner and
   schedule agree.

Self-healing property: when a transient leaves the feet ahead of the
clock, reactive pacing (at full reach) is inherently *slower* than the
schedule (at 0.9× reach), so the phase drifts back into alignment. On
top of that, **bounded catch-up** (`phase_debt` / `CATCH_UP_RATE` in
`clock.rs`) accelerates recapture: each reactive fire ahead of its
window edge measures the feet's lead as a fresh phase debt, and the
clock runs 1.35× until it's paid (a scheduled fire clears it; late
fires and idle leave it alone). Still rate-only and one-directional —
the phase is never set by step events, so the old attractor cannot
re-form. Measured on the recorded reversal stream: post-landing
scramble recovers in 3 steps (~0.45 s, was ~1.7 s), the mid-run
reactive episodes vanish entirely, and about-faces are single-step
excursions.

Validation: replayed all three player recordings through old vs new —
steady mean antiphase offset 0.99–1.00π (sd 0.03–0.05 on straight runs)
vs stable 0.6π splits before; every registry scenario at 1.00 exactly;
pinned by `steady_takeoffs_are_antiphase` (+ sub-frame-corrected
analysis in `scratch/antiphase2.py`, raw in `scratch/antiphase.py`).
The 35 rad/s arm-phase slew in `stride_sync` is retained as a safety.

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
13. **Terrain-aware swing landings (Stage 3b).** Probes aim at each foot's `probe_anchor` (landing target while stepping); `PlacerCtx` carries the full contact point so heights extrapolate along the floor's tangent plane. The landing height chases the probed surface all swing, the apex rises for upward steps, and a bell-faded clamp keeps the arc above risers. Without this, every step on a 20% incline popped ~5 cm at plant and stair risers were clipped through. `probe_length_factor` 1.3 → 1.8 so landing-aimed probes still reach downhill targets.
14. **Release-rule hardening for violent inputs.** Stance-age gate (`min_stance_fraction`) on non-overstretch releases, opening gate + stagger-lead on overstretch, hard-margin stagger bypass, and `Turn < Scheduled < Overstretch` priority. Found via the running about-face scenario and the random-input fuzz — see gotchas 12–14.
15. **Duty-scaled plant-ahead (fixes forward-tilted legs).** A stance lasts `duty` of the cycle = `4·duty·s` of hip travel, so planting at `+s` is hip-symmetric only at duty 0.5. At running duty (0.38) the stance spanned `+s … −0.52s` — the whole leg cycle sat ahead of the torso and read as a permanent forward tilt in game (the 5 m/s "walk" is in the running-duty band). Plant-ahead is now `2·duty·s` (`plant_ahead_distance`), which is symmetric at every duty; cycle distance stays `4s`, so cadence is unchanged. Verified numerically: per-foot mean stance offset along travel is ≤ 0.023 m at a 40 m/s² start to 5 m/s. Pinned by `run_stance_is_hip_symmetric`.
16. **Displacement-gated resume + split replant (fixes in-phase legs over seams).** Triangulated-terrain seams drop ground contact for a frame; the resulting suspend/resume replanted *both feet at the same spot* at speed, both legs hit the stretch release in unison, and the recovery phase-locked into a two-footed gallop re-seeded at every seam. Resume now skips the replant entirely when the pelvis moved less than the horizontal reach budget during the suspension (`prev_pelvis` freezes while suspended, so the displacement is free); a real jump's replant splits the stance along travel (left forward, right back, each by `plant_ahead_distance`, clock phase 0 releasing the back foot first). Pinned by `seam_blip_does_not_break_gait` and `hard_accel_run_settles_into_even_rhythm`.
17. **Slewed stride phase (fixes arm snapping).** `stride_sync::phase_from_placer`'s raw phase is discontinuous when swings overlap (flight phases): tracking switches feet mid-swing and the phase jumps, snapping the arms — clearly visible at high display rates. The phase now chases the raw target at ≤ 35 rad/s (above the fastest legitimate phase speed ≈ 26 rad/s, so clean gaits track exactly), staying continuous through tracking switches.

### Cadence rework: substepped clock + derived timing

The first phase-clock implementation produced frantic clumped takeoffs
(both feet airborne at walking speed, ~15 steps/s). Root cause was an
intent-speed floor of `settle_trigger / dt` — frame-rate dependent and
huge (3 m/s at 60 fps) — feeding both the phase clock and the duty
factor. The structural rework that replaced it:

1. **Internal substepping (`FootPlacer::tick`).** Each render frame is
   split into equal substeps `≤ max_substep_dt` (default 1/240 s, capped
   at 64 substeps). Pelvis and yaw are lerped from the previous frame's
   cached values; velocity, intent, and probe data are held constant.
   Step sequencing is therefore identical at 30 Hz and 240 Hz, and the
   `ceil(duration/dt)` discrete-preview compensation became unnecessary
   (`to = ideal + v·duration`, residual ≤ `v·substep_dt`).
2. **One cadence source of truth (`timing.rs`).** `GaitTiming::derive`
   computes trigger threshold (`max(2·gain·v/ω, settle_trigger)`),
   cycle distance (`2·trigger`), duty factor (Froude smoothstep), and
   swing duration (`(1−duty)·cycle_time`, clamped) from one gait speed.
   The phase clock, the step trigger, and the swing length cannot
   desync — `GaitPreset::stride_length`/`frequency_mul` are deleted;
   `stride_gain` alone sets both stride and cadence (cycle travel =
   `4·gain·v/ω`, so cycle *time* is speed-independent above the settle
   floor, ≈ `4·gain·√(h/g)`).
3. **Latched releases (`clock.rs`).** Stance-window exits latch a
   pending release instead of edge-triggering; a release blocked by the
   continuous-support guard (other foot mid-swing, duty ≥ 0.5) fires as
   soon as the guard clears instead of being dropped for a full cycle.
   Stale latches are cleared when movement stops. At most one step
   fires per substep, so a same-instant double release resolves as two
   takeoffs milliseconds apart, never a simultaneous double-flight.
4. **Sane intent floor.** The gait speed is
   `max(|v_xz|, intent_speed_floor)` while intent is held (0.5 m/s
   default), plain `|v_xz|` otherwise; `moving` additionally requires
   `|v_xz| > moving_speed_threshold` to ignore physics jitter. The
   scheduled-side / phase-step-consumed / next-step-side machinery is
   gone — alternation falls out of the clock, and idle settling picks
   the larger-error foot with the other-planted guard.
5. **Retuned for the rig.** `stride_gain`: walk 0.4, sprint 0.3, crouch
   0.7. `min/max_step_duration`: 0.12/0.4 s (a swing now spans ≥ 3–4
   display frames at 30 Hz). The anticipatory reach preview
   (`anticipatory_preview_time`/`anticipatory_reach_ratio`) is deleted —
   it only ever fed pre-lift, and the clock's schedule covers its job.

`facing` left `PlacerCtx` (derived from `yaw`, which substep
interpolation owns). Trace tests grew 30 Hz variants
(`trace_walk_forward_30hz`, `trace_run_30hz`); `timing.rs` and
`clock.rs` carry one shallow unit test per code path.

### CoyoteTime pose mapping (airborne flicker fix)

Running across geometry seams occasionally flicked the character into
the airborne pose, then a Landing splice + neutral replant. The physics
never jittered — `LocomotionState::CoyoteTime` bridged the one-frame
contact loss as designed — but `next_pose_state` mapped CoyoteTime to
`PoseState::Airborne { Fall }`, suspending the placer and splicing a
Landing on the way back. CoyoteTime now maps to the same Grounded gait
as `Grounded`; the fall pose starts only when the grace period expires
into real `Airborne`. Covered by a unit test in `animator.rs`
(`coyote_time_keeps_grounded_pose`). Lesson: any state the *physics*
treats as grounded-grace must be grounded for the animation too, or the
grace period is defeated visually.

### Test infrastructure (cfg(test) modules in `foot_placer/`)

- `sim.rs` — shared scenario simulator: integrates the pelvis from
  per-frame `(velocity, yaw, intent)`, derives per-foot ground contacts
  and normals from an analytic terrain height function (probing at each
  foot's `probe_anchor`, matching the game), snapshots placer state per
  display frame. Supports per-frame variable dt (`simulate_var_dt`) and
  per-gait parameters (`simulate_gait` — crouch uses gain 0.7 / lift
  0.06). Intent is supplied separately from velocity so anticipation
  cases are expressible; initial facing comes from `input(0).yaw`.
- `scenarios.rs` — the named scenario registry, all at 30 Hz (the worst
  display rate the placer must look natural at): walk, run, game-accel
  hard start (40 m/s²), start/stop, turn-in-place, walking 90° turn,
  standing reversal, running about-face, landing slide, 20% incline up
  + down, rough ground (±8 cm bumps), rolling hills (0.4 m / 8 m),
  voxel stairs (0.12 m risers), crouch walk, 45°-spawn-yaw diagonal
  walk. `sim::simulate_with_suspend` additionally drives per-frame
  placer suspension for seam-blip / hop modelling.
- `trace.rs` (`#[ignore]`) — `trace_scenarios` prints per-frame state
  for every scenario; `export_csv` writes one CSV per scenario to
  `scratch/foot_traces/` (includes terrain height under each foot).
  `scratch/plot_traces.py` (venv: matplotlib) renders each CSV to a
  4-panel PNG: foot y vs terrain (pops/clipping), world position along
  the travel axis (planted segments must be horizontal = no slide),
  top-down paths with plant markers, and a stance timeline. This is the
  fastest way to *see* gait quality without running the game.
- `invariants.rs` — asserting scenario tests pinning the properties
  that kept regressing: single-support at walking duty, strict L/R
  takeoff alternation, even takeoff spacing, step-count parity across
  display rates and under jittered frame times, prompt first step from
  a standing start, both feet reshuffling on a 180° turn-in-place, no
  instant re-lifts through standing *and* running reversals, the leg
  stretch budget across run/reversal/slide/incline, clean settling
  after a stop, crouch-gait support rules, spawn-yaw equivalence — plus
  registry-wide sweeps (no planted-foot slide, no terrain clipping,
  bounded vertical pop at plant) and a 20 s deterministic random-input
  fuzz over rough terrain asserting the universal properties.

### Record-and-replay (game ↔ offline bridge)

The synthetic scenarios are open-loop; the game is a closed loop through
physics, probes and the locomotion FSM. When the game looks wrong but the
tests pass, capture the real input stream and replay it offline:

1. Record in game: `PLACER_REC=scratch/recordings/<name>.csv cargo run`.
   `recorder.rs` (compiled into the game, ~zero cost when the env var is
   unset) writes every `PlacerCtx` field + the suspend flag + pose tag per
   tick, plus the placer's outputs, with exact f32 round-tripping.
2. Replay offline: `PLACER_REC_CSV=<recording> cargo test --lib
   foot_placer::replay -- --ignored --nocapture`. Rebuilds the placer from
   the recorded `# init` line, feeds the exact stream, prints the max
   divergence vs the in-game outputs (must be ~0 — nonzero means the
   recording misses an input), and writes
   `scratch/foot_traces/replay_<name>.csv` for `scratch/plot_traces.py`.
3. `record_replay_round_trip` (always-on test) pins the loop bit-exactly
   without the game.

Caveat: the replay assumes the default `FootPlacerConfig`; mirror any
config tuning into `replay.rs` before trusting the divergence number.

### Gotchas already hit (don't re-burn these)

1. **Coordinate convention.** `facing.cross(&Vector3::y())` is "right" and points to **−x when facing +z**. `FootPlacer::new` must initialise feet via `stance_offset` with the same sign convention, or left/right start swapped and every ideal pulls feet across the body.
2. **Trigger-before-advance.** `try_trigger_step` must run *before* `advance_stepping`. If advance runs first, a completing foot transitions Stepping→Planted mid-tick and its now-huge error retriggers it the same frame, starving the other foot.
3. **Alternation preference.** At high speed, a freshly-planted foot's error immediately exceeds the trigger. Whichever foot is evaluated first re-fires forever. Fixed by tracking `last_planted_side` and evaluating the *other* side first.
4. **No trigger floor larger than `settle_trigger` at speed.** A floor like the old `step_trigger` broke the symmetric-plant property: `stride_offset` scaled with gain but the trigger stayed pinned at the floor, so the foot ended up further behind the hip than it was ahead.
5. **Don't project `to` by raw `duration`.** ~~Use `(ceil(duration/dt) − 1) · dt`~~ — superseded by substepping; raw `duration` is now correct to within one substep.
6. **Never put `dt` inside a speed/threshold formula.** The old `settle_trigger / dt` intent floor made cadence frame-rate dependent and was the root cause of the clumped-takeoff bug. Rates and thresholds must be expressed in sim-time units; only integration multiplies by `dt`.
7. **`planted_yaw` must be set at *landing*, not takeoff.** When it was assigned in `start_step`, a fast turn swept a large yaw angle mid-swing, so the foot landed already past the turn trigger and re-lifted after one substep of stance. The reference is now `yaw_from_facing(to_forward)` at the plant in `advance_stepping` (the orientation the foot actually landed with). Repro: `trace_reverse_direction_30hz`; pinned by `reverse_direction_settles_into_rhythm`.
8. **Drop clock releases that latch mid-swing.** The latch exists to survive coarse frames, but if a foot's stance-window exit happens while that foot is *already swinging* (gait accelerating through a speed ramp, or a turn step fired ahead of schedule), the pending release fired the instant the foot landed — another one-substep stance. The placer now consumes any pending release for a foot that is mid-swing; the next regular window exit re-arms it.
9. **~~The clock must not free-run against the feet~~ — INVERTED by Stage 7.** `resync_to_takeoff` (every step fire re-anchored the phase to that foot's window edge) was introduced because off-schedule steps left a foot stranded ~0.76 m behind the pelvis waiting for a far-away window. But making the clock a follower meant every gate or reactive trigger that delayed a step reshaped the schedule itself, and off-antiphase timings became stable attractors (the in-game 0.6π/1.4π split). The stranded-foot problem is instead solved by the escape valves: the overstretch release caps how far behind a foot can get, and the mid-swing pending-drop (gotcha 8) absorbs the window/foot collisions. The clock free-runs; feet are pulled to it.
10. **Turn steps must be single-support.** Releases carry a `ReleaseKind` (`Scheduled` / `Overstretch` / `Turn`, with upgrade priority in that order). `Turn` releases only fire while the other foot is planted, regardless of duty factor — during a fast reversal the body crosses the walk/run duty threshold mid-turn, and without this rule a turn step would lift the second foot while the first was still mid-swing (both feet visibly moving forward at once). `Overstretch` fires under normal gait rules like `Scheduled`: delaying it would stretch the leg without bound (e.g. replant-at-stance after landing while moving fast, where the second foot *must* lift before the first lands — that's just running).
11. **Stagger all takeoffs or the gait hops.** When both feet hit their release conditions together (instant speed change, or both replanted at the same spot after a landing), they took off in phase, landed in phase, hit the overstretch trigger together again — a *stable* two-footed-hop attractor, plainly visible in `trace_run_30hz` as both feet swinging forward simultaneously. `select_step` now refuses any takeoff within `takeoff_stagger_fraction × swing_duration` of the previous one; the scheduled windows then pull the feet back to antiphase within one cycle. Corollary: feet planted together at speed must either hop or briefly overstretch on the push-off step — the placer deliberately chooses the stretch, which is why the slide invariant exempts the first cycle. Pinned by `run_takeoffs_stagger_and_alternate`. Lesson: after any trigger/support-rule change, *read the steady-state traces end to end* — the bug was in the most basic scenario, not the new one.
12. **Overstretch needs an opening gate (and must not fire at landing).** A foot that lands far ahead during a hard deceleration (about-face) is momentarily beyond the stretch budget, but the pelvis is *closing* on it and the stretch resolves by itself. Firing there re-lifted the foot after one frame of contact. The release now requires the radial speed of hip-away-from-foot to be positive, and leads the trigger by `radial_speed · stagger_remaining` so the step fires the instant the stagger gap expires instead of overshooting the limit while blocked. Repro was `reverse_running`; pinned by `running_reversal_settles_into_rhythm`.
13. **Overstretch must outrank Scheduled in `ReleaseKind`.** `request_release` upgrades by `Ord` and never downgrades. With Scheduled at the top of the order, a foot whose stance window had already exited could not be upgraded to Overstretch — its release stayed gated by the stagger + stance rules while the leg blew past the hard stretch limit (found by the random-input fuzz at a sprint-speed direction flip, not by any hand-written scenario). Order is now `Turn < Scheduled < Overstretch`, because Overstretch carries bypass powers (fresh-plant stance gate always, takeoff stagger past `overstretch_hard_margin`). Pinned by `random_inputs_hold_core_invariants`.
14. **A fresh plant needs a stance-age gate.** During a fast turn, a foot that has just landed immediately re-accumulates yaw error (and the clock can re-release it within a frame), producing one-frame stances. `min_stance_fraction × swing_duration` of contact is now required before any non-overstretch release fires. The gate must be swing-relative, not absolute: legitimate sprint stances (~0.1 s) are shorter than any reasonable fixed floor.
15. **Test with the real `ground_accel` (40 m/s²), not a gentle ramp.** The original run scenarios ramped at 10 m/s² and hid both the forward-tilt bias and the gallop dynamics — game starts reach 5 m/s in 0.125 s, faster than half a gait cycle, which is a different regime for the reactive releases. `run_hard_accel` is the game-faithful scenario; prefer it for anything trigger-related.
16. **Grounding is contact-event based — sleep needs a carry-over.** A sleeping body generates no narrowphase events, so `PhysicsWorld::grounded_handles` reported it airborne while it rested on the floor. Locomotion then walked Grounded → CoyoteTime → Airborne and stayed there (jumps are only consumed in Grounded/CoyoteTime — the player was soft-locked). Sleeping bodies now carry over their last awake support state.
17. **Time-based gates must not police a phase-based schedule.** `min_stance` (wall-clock stance age) blocked one foot's scheduled release every cycle at run speed; with the takeoff resync the delay became the schedule. Express step-firing rules in phase terms or drop them where the schedule already guarantees the property. Same lesson for the predictive overstretch lead: any trigger that reads time-into-the-future re-paces the gait when latches are sticky.
18. **Keep planner, schedule and stretch release on one geometry.** Three reach numbers existed: the planner's budget (from bare `standing_height` — 30% too big), the schedule's plant-ahead (uncapped `2·duty·s`), and the release's actual 3D hip→foot limit. At 5 m/s the schedule demanded more than the real reach, so the stretch release — not the clock — paced the gait (persistent antiphase wobble). One source of truth: true-vertical reach budget → caps stride in `GaitTiming::derive` → `plant_ahead_distance(timing)` used by planner and replant alike.
19. **CoyoteTime must keep ground handling.** It used to steer with the air model (`air_speed`/`air_steer_speed`); since every terrain-seam crossing dips into coyote for a frame or two, that injected a velocity perturbation at the seam-crossing rate — direction-dependent, and strong enough to entrain the event-driven step releases (in-game symptom: near-in-phase legs whose antiphase error varied with run direction). `movement_rule` now keeps `ground_speed`/`ground_accel` in coyote, retaining only `clamp_up`.

### Known-open follow-ups

- **Slope-scramble validation.** The phase-authority + catch-up stack is
  validated on flat-ground recordings; sloped scrambling (frequent in
  play) additionally stresses the reach geometry and probe behaviour.
  Needs one in-game recording on slopes as acceptance data.
- **Pelvis planner (Stage 5).** Pelvis is still a passive physics output; biomechanical rise/fall/sway not yet derived from support state.
- **Torso-yaw decoupling (Stage 5.5).** Visible torso yaw is still pinned to input yaw while feet step discretely.
- **In-game probe misses on deep drop-offs.** A swing landing more than ~`0.39 m` below the feet exceeds the probe length (`probe_length_factor` = 1.8) and falls back to the flat pelvis-relative height. Walking off a cliff edge transitions to Airborne anyway, so this has no visible window so far.

### Resume checklist

Before touching any code:
1. `cargo test --lib foot_placer` — the invariant sweeps cover every scenario; all must pass.
2. For anything visual: `cargo test --lib foot_placer::trace::export_csv -- --ignored`, then `.venv/bin/python3 scratch/plot_traces.py`, and read the PNGs in `scratch/foot_traces/plots/`. Planted segments horizontal in panel 2, foot y never below the dotted terrain in panel 1, no sliver bars in the stance timeline.
3. Run the game, watch the debug overlay. The ideal-sphere (green) sits roughly `stride_gain · v/ω` ahead of the hip; planted-sphere (yellow) should oscillate between `+s` ahead and `−s` behind symmetrically.
4. Skim `animator.rs::tick_foot_placer` and confirm the airborne-suspend branch still matches current `PoseState` variants.

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
