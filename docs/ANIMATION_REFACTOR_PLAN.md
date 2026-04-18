# Character Animation Refactor Plan

## Motivation

The current `src/biped/` module is the original "get a character rig on screen"
code. It has grown past what its structure can support: the driver re-derives
FSM information the `PlayerControlSystem` already owns, upper body and lower
body logic are entangled inside a single `LocomotionMode`, transitions between
modes are instantaneous (snap), and new animations (jump, long-jump, crouch,
sprint, landing) can't be cleanly expressed against the current shape.

This doc describes a refactor that:

1. Renames the module to reflect what it actually is (humanoid character
   animation, not "biped").
2. Splits the single `LocomotionMode` into two orthogonal FSMs matching the
   axes that already exist in `PlayerState`.
3. Moves behaviour onto the enums (tick / sample) so adding a state is a
   localised change.
4. Introduces a small, SOLID-friendly crossfade primitive for smooth
   transitions, designed so non-linear blends and per-fragment blend policies
   can be added later without rewriting the driver.
5. Uses `PlayerState` as the single source of truth — no re-deriving movement
   mode from velocity + ground contact inside the animation layer.

Applies the lessons captured in `docs/ANIMATION_PROJECT.md` throughout.

## Naming

The current module is called `biped`. The character is a humanoid, and the
module is really "procedural, state-driven character animation" — not
specifically about two legs. Proposed rename:

```
src/biped/   →   src/animation/
```

Types get matching renames:

| Old                        | New                           |
| -------------------------- | ----------------------------- |
| `BipedController`          | `CharacterAnimator`           |
| `BipedState`               | `AnimationState`              |
| `BipedSkeleton`            | `Skeleton`                    |
| `BipedConfig`              | `CharacterRigConfig`          |
| `BipedAnimationSystem`     | `CharacterAnimationSystem`    |
| `BipedProbeConfigSystem`   | `AnimationProbeConfigSystem`  |

Humanoid-specific bits (two-legged rig, stride wheel, biomechanical gait)
live in `animation::humanoid` so a future quadruped or vehicle rig can be
added as siblings without tangling.

```
src/animation/
  mod.rs                  (use-only, per project convention)
  animator.rs             ← CharacterAnimator: thin driver
  pose.rs                 ← PoseFragment, BlendPolicy, Crossfade
  state.rs                ← AnimationState (runtime phase: wheel, feet, hands)
  config.rs               ← CharacterRigConfig + Gait presets
  systems.rs              ← ECS glue
  humanoid/
    mod.rs
    skeleton.rs           ← joint positions, IK
    mesh.rs               ← generate_character_mesh (moved out of skeleton.rs)
    stride_wheel.rs       ← wheel math primitives (no state mutation)
    gait.rs               ← GaitCycle, keyframes
    pose_state.rs         ← PoseState FSM (lower body + core)
    upper_state.rs        ← UpperState FSM (arms + torso overlay)
```

The humanoid submodule owns the two FSMs because they encode a humanoid gait
(feet, hands, stride wheel, shoulder twist). The outer `animation` module
owns the generic machinery (pose fragments, blend policy, driver shape) so
alternative rigs reuse it.

## Design

### Two orthogonal FSMs

Matches the split we already have in `PlayerState`:

```
PoseState                         UpperState
─────────────────────────         ─────────────────────────────
  Grounded { gait }                 Swinging                   ← coupled to PoseState
  Launching { t, kind, takeoff }    Reaching { t, target }
  Airborne  { kind, takeoff }       Holding   { target }
  Landing   { t, kind }             Braced                     ← falling / landing
```

where

```rust
enum Gait { Idle, Walk, Sprint, Crouch { walking: bool } }
enum AirKind { Jump, LongJump, Fall }
struct Takeoff { facing: Vector3<f32>, air_speed: f32 }   // latched at liftoff
```

Parameters (stride_length, frequency, step_height, arm_swing_amplitude, head
bob amplitude) live on `Gait` presets in config — `Walk` vs `Sprint` vs
`Crouch` do not justify distinct state variants (lesson #4). Transitioning
from Walk to Sprint is a `Gait` field change inside `Grounded`; the pose
fragment the state emits differs only in its parameters. Crossfading between
two `Grounded { gait: A }` and `Grounded { gait: B }` is handled by the
generic blend machinery — no bespoke code per gait.

### Pose fragments

Each state emits a `PoseFragment`:

```rust
pub struct PoseFragment {
    pub feet:           Option<FeetPose>,      // both feet or none
    pub hands:          Option<HandsPose>,     // both hands or none
    pub pelvis_offset:  Option<Vector3<f32>>,  // e.g. landing squash
    pub shoulder_twist: Option<f32>,
    pub head_tilt:      Option<Vector2<f32>>,
    pub head_bob:       Option<f32>,
}
```

`None` means "this state does not drive that channel". The driver composes
a final pose by layering the `PoseState` fragment (authoritative for feet,
pelvis, head) with the `UpperState` fragment (authoritative for hands, torso
overlay). Holding an object simply means `UpperState::Holding` overwrites
the hands channel — no special-case post-update mutation.

This is the Interface Segregation principle applied to pose: states only
publish the channels they care about.

### Behaviour on the enums

Each FSM variant implements:

```rust
trait PoseStateLike {
    fn tick(self, ctx: &TickCtx) -> Self;            // transitions + timers
    fn sample(&self, ctx: &SampleCtx) -> PoseFragment;
    /// The cycle phase this state exposes for upper-body sync (stride wheel
    /// while walking, stroke phase while swimming, None otherwise).
    fn cycle(&self) -> Option<Cycle>;
}
```

```rust
pub struct TickCtx<'a> {
    pub dt: f32,
    pub player: &'a PlayerState,
    pub target: &'a PlayerTargetState,
    pub velocity: Vector3<f32>,
    pub horizontal_speed: f32,
}

pub struct SampleCtx<'a> {
    pub rig: &'a CharacterRigConfig,
    pub anim: &'a AnimationState,       // wheel_angle, feet, hands, pelvis, facing
}

pub struct Cycle {
    pub phase: f32,      // [0, TAU)
    pub kind: CycleKind, // Stride | Stroke | ...
}
```

`Cycle` is the generic handshake between the two FSMs: `UpperState::Swinging`
reads `PoseState::cycle()` rather than `AnimationState::wheel_angle` directly.
That removes the one place the sketch otherwise bakes in a humanoid-gait
assumption, and makes adding swimming (where feet/hands run off a stroke
cycle, not a stride wheel) a scoped change.

Adding a new state = one arm per method, following the pattern established
in `LocomotionState::{tick, movement_rule, allows_jump_cutoff}` (lesson #3).

### Mapping from PlayerState (driver stays thin)

`CharacterAnimator::update` does this and nothing else:

1. Read `PlayerState.locomotion` + `PlayerState.arm` + crouch/sprint intent
   from `PlayerTargetState` + `wants_to_walk`.
2. Map to `PoseState` and `UpperState`:
   - `LocomotionState::Grounded` → `PoseState::Grounded { gait }` where
     `gait` comes from target.crouch / target.sprint / speed.
   - `LocomotionState::Launching { allow_cutoff: true, .. }` →
     `PoseState::Launching { kind: Jump, takeoff }`.
   - `LocomotionState::Launching { steering: Locked, .. }` →
     `PoseState::Launching { kind: LongJump, takeoff }`.
   - `LocomotionState::Airborne { .. }` → `PoseState::Airborne { kind, takeoff }`
     (kind carries over from Launching; Fall if vy < threshold and no takeoff).
   - `LocomotionState::CoyoteTime` → `PoseState::Airborne { kind: Fall, .. }`
     (with a tiny grace where feet still appear planted — but expressed as a
     short `Landing`-style timer, not a bool flag, per lesson #5).
   - `ArmState` maps one-to-one to `UpperState` variants.
3. If the mapped variant differs from the current one, snapshot the current
   pose fragment and start a crossfade.
4. Tick both FSMs.
5. Sample both FSMs (after the tick — lesson #7 — so the Landing frame sees
   the Landing-state pose, not the stale airborne pose).
6. Compose and blend, write to `Skeleton`.

Crucially: `CharacterAnimator` never decides "am I walking?" from speed. It
trusts `PlayerState`. The only speed-derived choice is picking a `Gait`
preset inside `Grounded`, which is a parameter, not a state (lesson #1).

### Crossfade primitive (SOLID, extensible)

Start simple, linear, and pluggable:

```rust
pub trait BlendPolicy {
    /// Return 0..=1 given elapsed/duration. Default is linear.
    fn weight(&self, elapsed: f32, duration: f32) -> f32;
}

pub struct Linear;
impl BlendPolicy for Linear {
    fn weight(&self, e: f32, d: f32) -> f32 { (e / d).clamp(0.0, 1.0) }
}

pub struct Crossfade<P: BlendPolicy = Linear> {
    pub from: PoseFragment,
    pub to_duration: f32,
    pub elapsed: f32,
    pub policy: P,
}
```

`PoseFragment` gets a `lerp(&self, other: &Self, t: f32) -> Self` that
interpolates every populated channel and prefers `other` for channels only
it has. `Crossfade::sample(current: &PoseFragment)` returns the blended
fragment.

Why this shape:

- **Open/Closed:** swapping Linear for `EaseInOut`, `Spring`, or a
  per-channel policy requires zero driver changes.
- **Single Responsibility:** `Crossfade` holds from-fragment + time;
  `BlendPolicy` owns the curve; `PoseFragment::lerp` owns per-channel
  interpolation.
- **Liskov:** any `BlendPolicy` works anywhere a `Linear` does.
- **Interface Segregation:** driver only depends on `BlendPolicy::weight`,
  not on a fat AnimTrack trait.
- **Dependency Inversion:** states emit `PoseFragment`, not "set foot X to
  Y"; blending happens at the composed fragment level, not by poking state
  internals.

Future extensions (non-linear curves, per-channel crossfades where feet
blend faster than arms, trigger-based anticipation poses) slot in as new
`BlendPolicy` impls or a `Crossfade` variant that holds per-channel
durations. No driver edit.

### Anticipation and follow-through

Lesson #9 says forgiveness buffers pair up. Applied here:

- **Launching** is anticipation — 80–120 ms state before liftoff with knees
  bent / arms raised. Entry is the frame `PlayerState` transitioned from
  `Grounded` to `Launching`. Exit is on the timer, not on `!is_grounded`,
  because physics might separate before or after our ideal anticipation
  window. The FSM owns the event (lesson #6: don't drive anim events from
  physics queries).
- **Landing** is follow-through — 100–150 ms squash after first ground
  contact out of `Airborne`. Driver detects this by comparing previous-tick
  `PoseState` variant (Airborne) to this-tick (Grounded), and on that edge
  splices in `Landing { t: duration, kind: prev_kind }`. `Landing` emits a
  pelvis-offset channel and keeps the feet planted. When its timer expires
  it transitions to `Grounded`.

### Latching at takeoff

Lesson #10: `Takeoff { facing, air_speed }` is captured once when entering
`Launching`. Sampling in `Airborne` reads the takeoff snapshot, not live
velocity, so the airborne pose doesn't jitter near apex. When `Airborne`
exits to `Landing`, it passes `kind` along so landing style matches the
jump style (long-jump landing ≠ regular hop landing).

### Runtime phase state

`AnimationState` (renamed from `BipedState`) loses its `LocomotionMode` /
`prev_mode` / `mode_time` fields — those are now owned by the FSMs and the
`Crossfade`. What remains is the ongoing phase the FSMs mutate:

- `wheel_angle` (stride wheel)
- `left`, `right`: `FootState` (position, planted_position, normal, ground
  contact from probes)
- `left_hand`, `right_hand`: `HandState`
- `pelvis_position`, `facing`, `is_grounded`

Everything ephemeral (shoulder twist this frame, head bob this frame) moves
into the sampled `PoseFragment` and is no longer stored.

### Grab integration

`ArmState::Reaching`/`Holding` become `UpperState::Reaching`/`Holding`. The
per-frame `compute_grab_hand_target` helper in `biped/systems.rs` folds
into `UpperState::sample()`. Both hands are written by `UpperState` — no
post-update right-hand override, no left-hand drift (a minor existing bug
where the left hand keeps swinging while holding is fixed for free).

### ECS shape (unchanged)

`CharacterAnimator` stays a component. `AnimationProbeConfigSystem` and
`CharacterAnimationSystem` keep the same scheduling (probe config before
sensing, animation update after). The diff is internal to the module.

## Migration plan

Incremental, each step independently testable (lesson #16).

1. **Rename + relocate.** `src/biped/` → `src/animation/`, type renames, no
   behaviour change. Purely mechanical; compile + playtest to confirm no
   regressions.
2. **Introduce `PoseFragment` + `Skeleton::apply_fragment`.** Refactor the
   current `update_from_state` path to go through a fragment, but keep the
   old `LocomotionMode` enum driving things for now. Still no user-visible
   change.
3. **Extract `PoseState` FSM.** Replace `LocomotionMode` with `PoseState`,
   wired to `PlayerState` (not speed heuristics). Still one state emitting
   one fragment. Instant transitions. At this point Idle/Walk/Falling work
   as before; Dragged is gone (was a duplicate of Walking).
4. **Extract `UpperState` FSM.** Split hands out of `PoseState` into
   `UpperState`. Move grab target computation into `UpperState::Holding`.
   Left-hand-swings-during-hold bug disappears.
5. **Add `Crossfade` + `BlendPolicy` (Linear only).** Snapshot previous
   fragment on FSM transitions, blend for ~100 ms. First visually smooth
   version.
6. **Add `Gait` parameterization.** Walk/Sprint/Crouch as Gait presets
   driving a single `Grounded` state. Crossfade handles Walk→Sprint etc.
   automatically.
7. **Add `Launching` and `Landing`.** Anticipation + follow-through.
   Includes the prev-variant snapshot in the driver to fire Landing on the
   Airborne→Grounded edge.
8. **Add `AirKind` + `Takeoff` latch.** Differentiate Jump / LongJump /
   Fall poses. Long-jump landing gets a longer Landing window with more
   squash.

Each step leaves the codebase working. Steps 1–4 are pure refactor; 5
onward adds visible animation polish.

## Open questions (deferred until implementation)

- Exact crossfade durations per transition. Needs playtesting. Start with
  a single 100 ms default, override per-transition only if something looks
  wrong.
- Should `Gait::Crouch { walking: false }` be a separate `Gait::CrouchIdle`
  preset, or handled by amplitude-scaling Walk? Guess: a preset.
- Foot/hand slide during a crossfade — crossfade snapshots are now
  pelvis-relative: the `from` fragment's world-space spatial channels
  (`feet`, `hands`) are translated by `current_pelvis - from_pelvis` before
  lerping. This fixes sticky feet during jumps and the worst of the
  walk→idle slide. Only position delta is applied — rapid rotation during a
  blend will still cause the `from` spatial channels to lag (they stay in
  the snapshot-time facing). Upgrading to a full pelvis-local frame
  (position + facing) is future work if it becomes visible.
- Whether `CoyoteTime` deserves a distinct pose variant or just reuses
  `Airborne { kind: Fall }`. Current guess: reuse.

## Implementation reference (for the implementer)

This section exists so whoever implements this doesn't have to reconstruct
intent from the prose above. Read `src/biped/controller.rs`,
`src/biped/state.rs`, `src/biped/stride_wheel.rs`, `src/biped/systems.rs`,
`src/player/components.rs`, and `src/systems/player_control.rs` before
starting — they're the full surface area.

### What to preserve

- `stride_wheel.rs` math (`advance_wheel`, `is_swinging`, `compute_shoulder_twist`,
  `compute_head_tilt`, `compute_head_bob`, `update_foot`, `update_hand`) is
  correct. Keep these as free functions. The only thing to delete from that
  file is `handle_idle` — its `wheel_angle = 0.0` snap is replaced by
  crossfading out of the walking pose.
- `gait.rs` — `GaitCycle` and its keyframes are fine. `GaitCycle::walking`
  and `GaitCycle::arm_swing` become factory methods parameterized by a
  `Gait` preset.
- `skeleton.rs` geometry and `generate_biped_mesh` are fine. `Skeleton`
  gains an `apply_fragment(&PoseFragment, &AnimationState, &CharacterRigConfig)`
  method that replaces `update_from_state` — the fragment is authoritative
  for anything it sets, the `AnimationState` provides feet/hands/pelvis.
- `config.rs` — keep all tuning fields. Add `Gait` preset table.
- ECS wiring in `systems.rs`. The two systems and their scheduling stay.

### What to delete

- `LocomotionMode` enum in `state.rs`. `prev_mode`, `mode_time`, and
  `set_mode()` go with it — that bookkeeping moves into the FSMs and the
  `Crossfade`.
- `BipedController::determine_locomotion_mode` — replaced by a map from
  `PlayerState` in the driver (see mapping table below).
- `BipedController::update_idle_upper_body`, `update_falling`,
  `update_walking` — their logic moves onto `PoseState`/`UpperState` arms.
- The `if let Some(target) = self.state.grab_hand_target` override in
  `update()` — that behaviour moves into `UpperState::Holding::sample`.
- `compute_grab_hand_target` helper in `biped/systems.rs` — folds into
  `UpperState`.
- `FootState::planted_position` — replaced by the foot-locking step (step
  5+; until then the planted idea lives only as "feet don't move while the
  fragment doesn't drive them").
- `BipedState::grab_hand_target` field — moves into `UpperState::Holding`.

### Concrete mapping: PlayerState → PoseState / UpperState

Computed once per frame in `CharacterAnimator::update` (the driver):

| PlayerState.locomotion                                                 | PoseState                                          |
| ---------------------------------------------------------------------- | -------------------------------------------------- |
| `Grounded` + no move intent + speed < idle_threshold                   | `Grounded { gait: Idle }`                          |
| `Grounded` + crouch held                                               | `Grounded { gait: Crouch { walking: moving } }`    |
| `Grounded` + sprint held + moving                                      | `Grounded { gait: Sprint }`                        |
| `Grounded` + moving (default)                                          | `Grounded { gait: Walk }`                          |
| `Launching { allow_cutoff: true, .. }` (first frame only)              | `Launching { t: 0, kind: Jump, takeoff }`          |
| `Launching { steering: Locked, .. }` (first frame only)                | `Launching { t: 0, kind: LongJump, takeoff }`      |
| `Airborne { .. }` with a prior `Launching` kind                        | `Airborne { kind, takeoff }`                       |
| `Airborne { .. }` with no prior launch (walked off edge)               | `Airborne { kind: Fall, takeoff }`                 |
| `CoyoteTime(_)`                                                        | `Airborne { kind: Fall, takeoff }`                 |
| *(edge: prev was `Airborne`, this is `Grounded`)*                      | `Landing { t: 0, kind: prev_kind }` — spliced in   |

`takeoff` is latched at the moment `PoseState` enters `Launching`. The
driver keeps a `last_locomotion: LocomotionState` to detect the transition
edges (Airborne→Grounded for Landing, Grounded→Launching for anticipation).

| PlayerState.arm            | UpperState                           |
| -------------------------- | ------------------------------------ |
| `Idle` + PoseState airborne| `Braced`                             |
| `Idle` (otherwise)         | `Swinging`                           |
| `Reaching { elapsed, t }`  | `Reaching { elapsed, target: t }`    |
| `Holding { .. }`           | `Holding { hold_height, target_pos }`|

`target_pos` is the world-space grab-hand target; computation is what
`compute_grab_hand_target` does today, moved into the driver's map step or
into `UpperState::sample` (implementer's choice, but it must read the same
inputs — pelvis, facing, `GrabConfig`).

### PoseFragment

```rust
#[derive(Debug, Clone, Default)]
pub struct PoseFragment {
    pub feet:           Option<FeetPose>,
    pub hands:          Option<HandsPose>,
    pub pelvis_offset:  Option<Vector3<f32>>,
    pub shoulder_twist: Option<f32>,
    pub head_tilt:      Option<Vector2<f32>>,
    pub head_bob:       Option<f32>,
}

#[derive(Debug, Clone)]
pub struct FeetPose { pub left: Point3<f32>, pub right: Point3<f32> }

#[derive(Debug, Clone)]
pub struct HandsPose { pub left: Point3<f32>, pub right: Point3<f32> }
```

`PoseFragment::lerp(&self, other: &Self, t: f32) -> Self`: per channel,

- both `Some(a)`, `Some(b)` → `Some(lerp(a, b, t))`
- only one side set → that side, unmodified (channel is "still driven")
- both `None` → `None`

Composition of `PoseState` and `UpperState` fragments: `UpperState` wins on
`hands` and `shoulder_twist` if set; `PoseState` wins everywhere else.
Simple "overlay" semantics — no weight map yet.

### Crossfade placement

One `Crossfade<PoseFragment>` slot per FSM (so lower and upper blend
independently). On any variant-kind change in that FSM:

1. Sample the outgoing state one last time into a `from` fragment.
2. Transition to the new variant.
3. Set `Crossfade { from, elapsed: 0, to_duration: 0.1, policy: Linear }`.

Each frame:
- Tick FSM.
- Sample new state → `to`.
- If crossfade active: output = `from.lerp(to, policy.weight(elapsed, to_duration))`,
  then tick elapsed; clear when `elapsed >= to_duration`.
- Else: output = `to`.

"Variant-kind change" is a coarse enum discriminant comparison — a
`Grounded { gait: Walk }` → `Grounded { gait: Sprint }` is also a
crossfade trigger. Use a helper like `PoseState::discriminant_key()` that
returns `(variant, gait_or_kind)` and compare those.

### File-by-file scope for each migration step

**Step 1 (rename + relocate).** Mechanical. `git mv src/biped src/animation`,
then file renames where the contents are rig-specific (move to `humanoid/`):

```
animation/
  mod.rs              (renamed, updated use tree)
  animator.rs         (← controller.rs, type renamed)
  state.rs            (← state.rs, types renamed)
  config.rs           (← config.rs, types renamed)
  systems.rs          (← systems.rs, types renamed)
  humanoid/
    mod.rs
    skeleton.rs       (← skeleton.rs; split mesh.rs out if easy, else punt)
    stride_wheel.rs
    gait.rs
```

Update all call sites (grep `Biped`). `lib.rs` `pub mod biped;` →
`pub mod animation;`. `PoseState`/`UpperState`/etc. don't exist yet — this
step is zero-behaviour-change.

**Step 2 (PoseFragment plumbing).** Add `animation/pose.rs` with
`PoseFragment`, `FeetPose`, `HandsPose`, `Cycle`, `BlendPolicy`/`Linear`,
`Crossfade` (unused yet). Add `Skeleton::apply_fragment`. Have the old
`LocomotionMode` branches build a fragment and route through the new path.
Still no user-visible change.

**Step 3 (PoseState FSM).** Create `animation/humanoid/pose_state.rs`.
Implement the mapping table above in `CharacterAnimator::update`. Each
variant's `sample` takes the old `update_walking`/`update_idle_upper_body`
(lower-body portion)/`update_falling` math and emits a fragment. Delete
`LocomotionMode` and its bookkeeping. Crossfade remains inactive (instant
transitions) — still no visual smoothing.

**Step 4 (UpperState FSM).** Create `animation/humanoid/upper_state.rs`.
Split the upper-body math out of the walking/idle/falling arms into
`Swinging`/`Braced::sample`. Port grab-target computation into `Reaching`
and `Holding`. Delete the post-update `grab_hand_target` override and the
`compute_grab_hand_target` helper. Both hands are now driven by `UpperState`
— the left-arm-swings-while-holding bug disappears.

**Step 5 (Crossfade active).** Wire `Crossfade` into the driver. Snapshot
on discriminant-key change, blend 100 ms, linear. Playtest: walk↔idle,
walk↔sprint, ground↔air should all look noticeably smoother.

**Steps 6–8 (new states).** `Gait` presets, then `Launching`/`Landing`,
then `AirKind`/`Takeoff`. Each adds one variant or one parameter at a time
and should be independently testable.

### Things to verify by playing, not by reading

From lesson #16: the user will feel bugs the implementer won't. After each
step, `cargo run` and:

- Walk around, stop, turn (does idle look still? do feet plant?)
- Jump from standing (does the hop look springy or robotic?)
- Run + jump, long jump (does the long jump arc look committed? landing?)
- Walk off an edge (coyote time — any lurch at the edge?)
- Grab and hold a crate while walking (both hands on it? does left arm
  still swing? it shouldn't, post step 4)
- Crouch, crouch-walk, sprint, sprint↔walk transitions (post step 6)

Don't claim a step is done without at least the relevant subset playtested.

## Non-goals

- Skeletal animation clip playback (keyframed .glb/.fbx). This system stays
  procedural; the architecture leaves room but we're not building that now.
- Ragdoll transitions / physics-driven recovery poses. Future work.
- Per-bone weight maps for partial-body overrides beyond the two-FSM split.
  If we need them, they arrive as a third FSM layer or as a `PoseFragment`
  extension; not in scope for this refactor.
