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
ideal_xz = capture_point(pelvis, velocity, support_height)
         + turn_in_place_offset(facing, yaw_rate)
         + stance_offset(facing, foot_side)

trigger = max(||planted_xz - ideal_xz||,
              |Δfacing_since_plant| * hip_width * k_turn_trigger)

Planted:
    if trigger > step_trigger AND other_foot.is_planted:
        → Stepping {
              from: planted_xz,
              to:   ideal_xz,     // committed at step start; no retarget
              t:    0,
              duration: step_duration(speed, yaw_rate),
          }

Stepping:
    t += dt
    foot.position = swing_trajectory(from, to, t / duration, step_height(preset))
    if t >= duration:
        → Planted at `to`
```

`swing_trajectory` is a minimum-jerk-style arc (or cycloidal) rather than a straight parabola — smooths lift-off and plant, hides the keyframe-like look of pure parabolic lifts.

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

All live on `CharacterRigConfig` (or a sub-struct `FootPlacerConfig`):

| name | rough default | effect |
|---|---|---|
| `step_trigger` | 0.25 m | error magnitude (or equivalent from yaw) that fires a step. Larger = more stable planting, lazier reaction. |
| `k_yaw` | 0.4 | how much yaw rate pulls feet sideways during turn-in-place. |
| `k_turn_trigger` | 0.3 | how much accumulated facing change (in rad · hip_width units) counts toward `step_trigger`. |
| `min_step_duration` | 0.18 s | floor on a step's airtime (prevents teleport-stepping at high speeds). |
| `max_step_duration` | 0.45 s | ceiling (prevents interminably-lifted feet when speed drops mid-step). |
| `max_stride_reach` | 1.5 × leg_length | clamp on `ideal_xz` offset from hip. Pathology guard. |
| `step_height` | 0.08 m (walk) / 0.14 m (sprint) | per-preset peak lift. Sourced from `GaitPreset`. |

Step duration scales inversely with speed within the [min, max] range, so sprint cadence is naturally faster than walk cadence.

Note: `sqrt(h/g)` is *not* a tunable — it's derived from rig geometry and world gravity, following the capture-point result.

### Safety invariants

- Only one foot may be `Stepping` at any time. Second trigger waits.
- A `Stepping` foot is never interrupted by a new target — it always completes to its committed `to`. Target prediction at step-start bakes in capture point at that instant; we accept minor error rather than retarget mid-step. (If this proves visibly wrong on sudden direction changes, add a retarget hook, but start without.)
- Vertical ankle position during `Planted` stays pelvis-relative by default (`pelvis.y - standing_height`). Per-foot ground-contact y overrides only when `FootState.ground_contact` exists and differs meaningfully (uneven terrain, stairs). Matches the convention documented in `animator.rs:476-480`.

---

## Migration plan

Staged so each step is independently testable and the character stays animating at every intermediate state.

### Stage 1: Introduce `FootPlacer` in parallel

Add the module with capture-point math and turn-in-place. Wire it into `update()` but do **not** read from it yet — feet still come from `PoseState::Grounded::sample`. Log `foot_placer` output vs current output each frame (debug overlay): visualise where steps *would* have fired. Tune `step_trigger` against real gameplay (walking, sprinting, landing slide, incline, spinning in place) until step cadence matches what looks natural.

### Stage 2: Switch foot xz source

Flip `PoseState::Grounded::sample` to read xz from `FootPlacer` instead of computing from stride wheel. Delete `anchor_feet_under_hips`, `replant_foot`, the `is_idle_gait` branches in `update()`. Gait variants now differ only in stylistic channels.

Expected regressions: gait-preset crossfades may temporarily jar if step height is large and the blend lands mid-step. Tune crossfade duration; add step-height interpolation inside `FootPlacer` so changing preset smoothly morphs the active step's arc.

### Stage 3: Ankle IK + swing-leg collision

Add the foot-orientation slerp described above. Add predictive probes along the committed swing arc and lift the peak to clear obstacles. At this point feet behave correctly on slopes, stairs, and uneven voxel geometry.

### Stage 4: Collapse the stride wheel

The arm-swing phase reference is the only remaining consumer. Derive arm phase from `FootPlacer` (foot.position relative to hip, normalised) or from step-completion events. Delete `stride_wheel` module.

### Stage 5: Pelvis planner (the biomechanics upgrade)

This is the stage that distinguishes "placed correctly" from "walks convincingly." Add a `PelvisPlanner` component that *computes* pelvis offset relative to the physics-supplied capsule position, rather than treating pelvis as a fixed input.

The pelvis in real walking is not static — it traces an inverted-pendulum arc: rises over the support foot, falls between steps, shifts laterally into single-support, and leans slightly in anticipation of the next step. The planner reads `FootPlacer` support state (which foot is planted, stepping phase, CoM vs support polygon) and produces:

- **Pelvis y offset** — sinusoidal rise/fall per stride cycle; amplitude from preset.
- **Pelvis lateral sway** — shifts toward the support foot during single-support; computed from which foot(s) are planted.
- **Pelvis anticipatory lean** — small forward lean during the first third of a step (weight shifting toward swing target).

`PoseState::Grounded::sample` already emits a `pelvis_offset`; the planner's output composes with it the same way. The difference is that the offset is now derived from foot-support state, not from a gait preset's crouch depth.

This stage is what pushes the system past Overgrowth-tier toward biomechanically-grounded procedural locomotion. It's also the stage that will make the character's upper body move the way people instinctively expect — head bob, shoulder roll, arm swing all react to pelvis arc automatically because they're all pelvis-relative.

### Stage 6: Landing anticipation (optional follow-up)

Add the `reach_toward_ground` hook for the airborne feet-gap case. Natural next step once `FootPlacer` owns foot position.

---

## Tradeoffs & open questions

**This is a real refactor, not a patch.** The current `PoseState::Grounded::sample` produces a complete foot trajectory; the new design requires splitting that across two layers and accepting that the layered system has its own tuning surface. The capture-point math itself is ~10 lines; the work is in restructuring who owns what.

**Gait presets become thinner.** Today a preset controls step length, lift, hip drop, torso pitch, arm swing. After this change it controls step *height* and stylistic channels only — stride length emerges from `v * sqrt(h/g)`, not from the preset. This is correct biomechanically but means the "crouch walks with tiny steps" look must come from a smaller `step_trigger` multiplier under crouch, not a smaller stride-wheel amplitude.

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
