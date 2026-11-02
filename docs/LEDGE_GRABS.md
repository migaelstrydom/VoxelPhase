# Ledge grabs

The player catches edges from the air and the water, hangs from them,
shimmies along them, and leaves by dropping, pulling up or jumping backwards.
A chest-high box is climbed with the same move: a ledge low enough to skip the
hang. This document is the design. The state machine it slots into is
`docs/LOCOMOTION_STATES.md`, which already reserves `Ledge`.

## What it is for

- **Terrain ledges.** A jump that falls short is caught, and so is a fall past
  an edge the hands meet.
- **Moving platforms.** The hands hold the platform, the platform carries the
  body, and letting go leaves with the platform's velocity.
- **Climbing out of the water.** A bank too high to jump onto from treading
  (higher than `jump_ashore`'s 0.3 m quay) is a Ledge, grabbed from Swim.
- **Destruction.** A ledge blasted away under the hands drops the player. A
  blast that opens a new edge makes a new way up.
- **Loose bodies.** The grip is a physics joint, so a light crate on a shelf
  is dragged down by the body hanging from it, and a swinging platform swings
  the hanger.

Out of scope for the first version: turning corners while shimmying (the
shimmy stops at one), climbing walls or ladders, grabbing from a walk with no
jump (OnFoot never enters a Ledge), and creatures (only a body with a
`LedgeSensor` senses edges).

## The body, in numbers

Every threshold below is measured against these numbers, so they are worth
having in one place.

| | |
|---|---|
| Capsule | 1.0 m tall (`collider_half_height` 0.5, caps included), radius 0.25 m; its centre 0.5 m above the feet |
| Shoulders | 0.75 m above the feet: legs 0.5 + torso 0.25 (`CharacterRigConfig`) |
| Arm | 0.32 m (0.18 + 0.14); hands overhead reach 1.07 m above the feet |
| Jump | `jump_speed` 7 m/s: about 2.5 m of rise at full hold, 0.5 m for a tap |

Hanging with the hands on the edge and the arms near straight puts the
body's centre about 0.53 m under the edge and the feet about 1.03 m under it.

Heights in this document are measured from the capsule's centre, not the
feet: the rig's feet tuck in the air, and the capsule is what the physics
holds. The shoulders are 0.25 m above the centre, the chest about 0.05 m
below them, and the hands reach 0.57 m above it.

## Components

```text
                       PhysicsWorld (casts)
                              │
   LedgeSensor ───────────────┤  per frame, from the hands
   (edge test, prediction,    │
    released-edge memory)     ▼
        │ Requests.edge: Option<Edge>          (rule 4: the hands ask)
        │ prediction: Option<Edge> ──────────► arms (reach ahead)
        ▼
   LocomotionState::tick ──► Ledge { edge, phase }
   LocomotionState::enter ─► EntryEffects (velocity matched to the edge,
        │                                  back-jump launch)
        ▼
   CharacterControlSystem
        ├── Grip::reconcile(state): the BallJoint between the body and the
        │        edge's body, made to match the state every frame (exists
        │        in a Ledge, anchored by the phase; gone otherwise)
        ├── facing: -outward in a Ledge, whatever the stick says
        └── roles: planner Off, legs Off (hang), arms Grip, pose by phase
```

Each piece has one owner:

- **`Edge`** (value type, `src/character/ledge/`): what the hands found.
- **`LedgeSensor`** (component, same module): finds edges at the hands, predicts
  edges along the arc, re-checks the held edge, and remembers an edge just let
  go. It owns its configuration (`LedgeConfig`).
- **`Ledge`** (state, `src/character/locomotion/ledge.rs`): the phases, the
  input timing, and the exits.
- **`Grip`** (`src/character/ledge/grip.rs`): the joint and its anchors. Each
  frame it makes the joint match the state: in a Ledge with no joint it
  creates one, and in any other state it removes one. It moves the anchors for
  the phase. It decides nothing. A joint the solver has deactivated, or one
  whose edge body is gone, it reports as lost, and that becomes a Fall on the
  next tick, the same way as a lost edge. Reconciling, rather than reacting
  to entry effects, is needed because a phase change inside a Ledge has no
  entry effects (`enter` returns none for the same state moved on).

## The edge

An edge is the lip of a surface the hands can hold. There are two tiers:

- **Grippable**, which enters the Ledge: you can hang, shimmy, drop and
  back-jump.
  - The top behind the lip faces up within `grip_slope` (45°) of vertical.
  - The top is at least `grip_depth` (4 cm) deep behind the lip. That is a
    finger hold, not a foothold.
  - Below the lip, the surface is steeper than the walkable slope, or there is
    nothing there. A slab's free edge or a platform's rim is a grippable lip
    with nothing below it.
  - Above the lip, open air for at least the hand's height (5 cm), so a step
    in a wall that carries on up is not a lip.
- **Climbable**, which also allows a pull-up:
  - The top is standable: within the walk slope, and at least `stand_depth`
    (0.35 m, the capsule's radius plus a margin) deep.
  - The capsule fits standing there: a cast up from the stand point is clear
    for the body's height.

  A narrow lip, or a ledge under a low roof, is grippable but not climbable.
  Pushing towards it strains and never commits.

```rust
/// A lip the hands can hold, in the frame of the body it belongs to.
pub struct Edge {
    /// The body the lip is on; `None` for terrain and other static geometry.
    pub body: Option<RigidBodyHandle>,
    /// Where each hand holds, on the top just behind the lip, in that body's
    /// frame (the world's for terrain).
    pub hands: [Point3<f32>; 2],
    /// Horizontal, pointing out of the face, away from the top: the way a
    /// drop or back-jump leaves. In the body's frame.
    pub outward: UnitVector3<f32>,
    /// Whether the top is standable and has room for the body.
    pub climbable: bool,
    /// The body's footing, as the planner reads it (`Fixed` or `Loose`).
    pub footing: Footing,
}
```

An edge kept in its own body's frame is what makes a moving platform or a
tipping crate work. Read in the world frame each frame, it is wherever that
body has taken it.

### Sensing it

The sensor casts from the hands. It doesn't use the planner's strip, which
looks along the walking path from the body's own layer, stands aside on
carriers, and is off in the air where most grabs start. It reuses the
planner's casting core. `PhysicsCaster` was tied to a `PathFrame`. Its
world-space part (one sphere cast that sees every collider, the body hit,
and its `Footing`) is now `WorldCaster` (`src/traversal/world_caster.rs`).
`PhysicsCaster` wraps it with the path frame, and the sensor uses it
directly. If the planner later gets a 3D local patch, an `edges(region)`
query on that patch is where this sensor moves.

The strip passes through bodies moving faster than `IN_FLIGHT_SPEED`
(2 m/s), unless they are carriers. The sensor needs a different policy:

- **The held edge's body is always seen**, however fast it moves. A crate
  dragged off its shelf, or a swinging platform that isn't a carrier, would
  otherwise vanish from the re-check and end the hang.
- **What the grab system holds is excluded**, as is the player's own body.
  The held object is still in the hands when the sensor first runs.

`WorldCaster` takes its bodies always seen as a parameter (today's
`carriers`), and the sensor adds the held edge's body to them.

For each hand, at the hand's reach point (the shoulder, plus the arm along the
body's heading, at the height being tested):

1. **Face.** A small sphere (radius 3 cm) cast forward along the heading finds
   the face, its point and its normal. A miss means no face: the lip may hang
   free, so go on to step 2 at a fixed reach ahead.
2. **Top.** A cast down from the top of the reach band, `grip_depth` beyond
   the face, finds the top and its normal.
3. **Lip.** A cast forward just above the top (+5 cm) must be clear past the
   hand. Otherwise the top is a step in a wall that carries on up.
4. **Climbable.** A capsule-radius cast down at `stand_depth` beyond the lip,
   and a cast up from there for the body's height.

`outward` is the horizontal part of the face normal from step 1, averaged
over both hands. With no face (a free lip), it comes from the line between
the two hands' lip points: a 0.18 m baseline on marching-cubes noise is too
short to trust where a face is available. On bumpy terrain each hand holds its
own lip point, so the hands sit on the real geometry. The edge is grippable
only if both hands find a lip and the two lip heights differ by at most
`hand_step` (4 cm, about 13° of tilt over the hands' spacing).

All of that is about a dozen casts per test position. The planner's strip
costs about 3000 for comparison.

### When the hands catch

An edge at the hands is grabbed automatically when all of these hold:

- **The hands are free.** `LocomotionState::hands_free()`, a new question for
  every state to answer (rule 9 of the state machine): true in Jump, Fall and
  Swim, false otherwise. An object held by the grab system is dropped on
  entry.
- **The lip is in the catch band.** It is between the chest (0.2 m above the
  capsule's centre) and the overhead reach (0.57 m above it), plus
  `catch_stretch` (8 cm) for a hand that just snags it. Above the shoulders
  the grab is a hang. Below them, with the stick pushing towards the edge, it
  is a mantle (see *Input*).
- **The body is not rising fast.** Vertical speed must be at most `catch_rise`
  (1 m/s). A jump rising past a ledge carries on to its apex and lands on top
  if it clears it. It is caught only if it comes back down past the lip.
- **The body faces the edge.** The heading is within `catch_facing` (60°) of
  into the face. A body that walks off a cliff faces away from the lip and
  falls past it, which is what we want.
- **The fall isn't too fast.** The speed relative to the edge's body is under
  `catch_speed` (12 m/s, a drop of about 7 m).
- **The edge wasn't just let go.** After a drop or back-jump, the edge let go
  can't be grabbed again for `regrab_lock` (0.3 s). The sensor keeps that
  memory, not the state, so a Fall is the same Fall however it began.

### Predicting it

In Jump and Fall the body is ballistic, apart from air steering. The sensor
steps the hands along the body's current velocity under gravity for
`reach_ahead` (0.3 s), in 0.05 s steps, and runs the edge test at each step
until one passes. A predicted edge is the arms' to read. The hands reach for
it before they get there, so a grab looks intended rather than snapped. The
prediction never enters a Ledge itself. The real test at the hands still
decides that.

## The state

```text
  Jump, Fall, Swim ──(an edge at the hands)──► Ledge { edge, phase }

  Grab ──(settled)──► Hang ──(stick towards a climbable edge, held)──► PullUp ──► OnFoot
   │                   ├──(stick away, held)──► Fall
   │                   ├──(jump pressed)──► Jump (back-jump)
   │                   └──(edge lost)──► Fall
   └──(below the shoulders, stick towards, climbable)──► PullUp  (the mantle)
```

```rust
pub struct Ledge {
    pub edge: Edge,
    pub phase: LedgePhase,
}

pub enum LedgePhase {
    /// The hands landing on the edge; the stick is ignored for `settle`.
    Grab { elapsed: f32 },
    /// Hanging. `lean` is a drop or pull-up being held towards commitment.
    Hang { lean: Option<Lean> },
    /// Climbing onto the top, along the grip's lift path.
    PullUp { elapsed: f32 },
}

pub struct Lean {
    pub towards: LeanTowards, // Drop or PullUp
    pub held: f32,
}
```

`Edge` comes in through a new request, `Requests::edge: Option<Edge>`. The
sensor fills it each frame: a catchable edge while airborne or swimming, and
the held edge re-checked while in a Ledge. A held edge that the re-check no
longer finds on the same body (`Edge::body`) is a Fall. That one rule covers:

- a blasted ledge;
- a crate that tipped until its top no longer faces up;
- a body that was removed;
- a terrain lip blasted loose as a `Fragment`, which is no longer terrain but
  a body of its own.

A lost grip (see *Grip*) arrives the same way, as a request.

### Input

The stick is read against the edge, not the camera. The camera-relative move
direction (already world-space in `Requests::move_dir`) is split against
`outward`:

| Stick | Within | Does |
|---|---|---|
| Towards the face | `lean_cone` (±45°) of `-outward` | lean to pull up; commits after `commit_hold` |
| Away from the face | `lean_cone` of `outward` | lean to drop; commits after `commit_hold` |
| Along the edge | anything between | shimmy at `shimmy_speed` along the edge |
| Jump pressed | | back-jump, at once |

- **Settle.** For `settle` (0.3 s) after the grab, the stick is ignored, so a
  stick still held from the jump doesn't drop or pull up the moment the hands
  land. Entering a Ledge also spends the jump buffer (`consume_jump`). A press
  buffered on the way up would otherwise back-jump on the first frame of the
  catch. A jump pressed during the settle is honoured, because it is a fresh
  press.
- **Commit.** After `commit_hold` (0.2 s) of holding in a cone, the move
  commits. The pose answers during the hold: it leans out for a drop and bends
  the arms for a pull-up. Released early, the lean eases back. The pause reads
  as effort because you can see it.
- **No room above.** On an edge that isn't climbable, the pull-up lean
  strains and never commits.
- **Mantle.** An edge caught below the shoulders, climbable, with the stick
  already in the pull-up cone, skips Grab and the hold and goes straight to
  PullUp. A chest-high box flows out of the run, while a real hang still
  takes effort.
- **Back-jump.** A Jump with a new kind, `JumpKind::BackJump`. It leaves along
  `outward` at `back_jump_speed` (4 m/s) with `jump_speed` upward, and the
  heading turns to face away from the wall. That needs a new entry effect,
  `EntryEffects::planar_velocity`: today a jump only sets the vertical speed
  and caps the air speed. Its air steering, and the cut of the rise on an
  early release, are as for any jump.

All the numbers are feel numbers, to tune in play.

### Answers to the questions that cut across states

| Question | Ledge's answer |
|---|---|
| `water_takes` | false. A body hanging over water with its legs in it stays hanging. A drop is a Fall, and the water takes that the next tick. |
| `hands_free` | false |
| `allows_jump_cutoff` | false |
| `movement_rule` | no authority anywhere: no steering in the air and no traction at contacts. That is a new form of `MovementRule`. Today's rules all drive traction toward some target, and once a pull-up's capsule touches the top, traction at five times μN would fight the joint. The joint moves the body. |
| `tag` | `ledge_grab`, `ledge_hang`, `ledge_pull_up` |

Two of the controller's per-state choices take a Ledge into account:

- **Facing.** The yaw drive faces `-outward`, as a thrower faces its throw.
  Otherwise it would turn to the stick, and a shimmy would turn the body along
  the wall.
- **Pitch.** Upright, straight away. A swimmer caught by an edge is lying
  over, and easing back up at the swim's `stand_rate` would start the hang
  leaning.

`LOCOMOTION_STATES.md` lists Ledge leaving for Swim. With drop always being a
Fall, that exit isn't needed and is struck from its table.

## The grip

The grip is one `ConstraintKind::BallJoint`. Both anchors are placed so the
geometry is right however the body has turned:

- **`body_b`** is the player. Its anchor is on the body's axis, 0.53 m above
  the centre. It never moves and never turns with the yaw.
- **`body_a`** is the edge's body, or `None` for terrain. Its anchor is in the
  edge's frame: the midpoint of the hands' lip points, moved out along
  `outward` by `collider_radius + face_gap` (1 cm) plus the hands' inset
  behind the lip. That puts the capsule 1 cm clear of the face, so it has no
  friction against it, whichever way the body faced at the catch.

Rotation is left free by the joint. The player's `KeepAttitude` holds it
upright, and the yaw drive turns it to face `-outward` about its own axis,
which is where the anchor is.

Every phase moves only the edge-side anchor, so every path is in the edge's
frame:

- **Grab.** The joint is rigid from the start. A spring catch dips by about
  speed × time constant: 0.6 m at 5 m/s with a 0.12 s constant, and 1.4 m at
  12 m/s. Stiffening it afterwards would then yank the body back up. Instead,
  entering a Ledge sets the body's velocity to the edge body's velocity at the
  anchor (a new entry effect, `EntryEffects::match_velocity`), so the rigid
  joint starts with nothing to stop. The dip is the pose's job, a transient in
  the arms and knees.
- **Hang.** The body's weight goes into the edge's body: a light crate is
  pulled off its shelf, and a platform carries the hanger and feels them.
  With the catch's speed already matched, the joint's largest load is a hit
  while hanging. So it is breakable at `grip_strength` (four body weights,
  turned into `max_impulse` per substep). Rubble landing on a hanger tears
  them off rather than being solved between a rigid joint and its contacts.
  The solver only marks a broken joint inactive, and `Grip` reports that as a
  lost grip.
- **Shimmy.** The anchor slides along the edge at `shimmy_speed`. Each frame
  the sensor tests the edge at the hands' next position. Where it fails (a
  corner, the end of the lip, a gap), the anchor stops there.
- **PullUp.** The anchor moves along a lift path:
  - first up, until the capsule's bottom is `lip_clearance` (3 cm) above the
    lip, keeping `face_gap` off the face;
  - then inboard, until the body's axis is over the stand point that the
    climbable test checked (`stand_depth` behind the lip).

  It takes `pull_up_time` (0.6 s), or `mantle_time` (0.3 s) for a mantle. The
  joint is the motor, so the climb pushes down on whatever it climbs. The
  path's velocity is the target, and the joint's position correction only
  trims drift. At the end the joint is removed. Entering OnFoot from a Ledge
  matches the body's velocity to the edge's, as the catch did, which drops
  whatever correction velocity the joint built up. The body then stands on
  the top, its contacts ground it, and the state is OnFoot.
- **Leaving.** Drop, back-jump or a lost edge removes the joint, and the body
  leaves with the velocity it has. On a platform, that is the platform's.

If the edge's body is removed, `PhysicsWorld::remove_body` already removes
every constraint that references it. `Grip` finds its handle gone, and the
sensor's re-check fails the same frame.

## Consumers

The roles follow the state machine's rule 8: an arm for every state and no
wildcards.

| Consumer | Ledge |
|---|---|
| Planner (`PlanRole`) | Off |
| Legs | Off. The pose hangs them from the hips, knees bent a little towards the wall. |
| Arms (`UpperState`) | Grip: each hand held to its `Edge::hands` point, read through the edge's body each frame, with the elbows solved to them (the hand clamp from c553979). In Jump and Fall, a predicted edge turns Brace into Reach towards it. |
| Pose | By phase: the Grab catch, the Hang with its lean, and the PullUp following the lift path |
| Grab system | `arm_state_allowed`: only `Idle` in a Ledge; entering one releases what is held |

## Testing

**Unit tests**, in `src/character/ledge/` and `locomotion/tests.rs`:
- The edge test on synthetic geometry (`SolidsGeometry` blocks):
  - wall tops at heights through the band;
  - a 3 cm lip, which is not grippable;
  - a 5 cm lip, which is grippable and not climbable;
  - a top under a low roof, which is grippable and not climbable;
  - a sloped top at 40° and 50°;
  - a step in a wall that carries on up;
  - a free slab edge;
  - two hands at lip heights that differ by more than `hand_step`.
- The prediction finds an edge the arc will meet, and no edge on an arc that
  clears the wall.
- The state machine, on scripted facts and requests:
  - settle, then commit, then release early;
  - the mantle shortcut;
  - a back-jump on the first frame of Hang;
  - edge lost to Fall;
  - the regrab lock;
  - `hands_free` from every state.

**`character_viewer` scenarios** (real game, scripted pilot):

| Scenario | Checks |
|---|---|
| `ledge_short_jump` | a jump that falls short of a 1.6 m wall catches it and hangs at rest within 0.5 s |
| `ledge_fall_past` | a fall down a wall's face, facing it, catches its lip; facing away, it doesn't |
| `ledge_drop`, `ledge_pull_up`, `ledge_back_jump` | each exit, with its timing; a stick released before `commit_hold` doesn't commit |
| `ledge_shimmy` | shimmies to the lip's end and stops |
| `ledge_narrow_lip` | hangs from a lip it can't stand on; pushing towards it never commits |
| `ledge_mantle` | runs and jumps late into a chest-high box: on top without a hang |
| `ledge_from_water` | swims to a 1 m bank and climbs out |
| `ledge_platform` | catches a moving platform's rim, rides it, drops off with its velocity |
| `ledge_light_crate` | hangs from a light crate on a shelf, which comes down |
| `ledge_blasted` | a blast takes the lip under the hands, and the body falls |

These need stations: a small level, `ledge_garden`, with walls at a few
heights, a narrow lip, a low-roofed ledge, a chest-high box, a crate on a
shelf, a moving platform, and a pool with a 1 m bank.

## Steps

Each step ships with the game working, and its tests.

1. **`WorldCaster`.** The world-space core split out of `PhysicsCaster`. No
   change in behaviour: captured fixtures stay identical (done).
2. **The edge test and `LedgeSensor`.** Grippable and climbable, the
   released-edge memory, the held edge always seen, and an F3 overlay of
   sensed edges. Unit tests on synthetic geometry.
3. **The `Ledge` state.** Phases, input timing, exits, `hands_free`, the new
   movement rule and entry effects, and every consumer's mapping. Unit tests
   on scripted facts.
4. **The grip.** Catch, hang, drop and back-jump, and `ledge_garden`'s first
   walls.
5. **Shimmy.**
6. **Pull-up and mantle.**
7. **Arms and pose.** Hands on the edge, the arc prediction and the reach
   ahead (cosmetic, so it waits until here), and the hang and lean poses.
8. **Water, platforms, loose bodies and blasts**, and their scenarios.

Steps 1 to 6 need no input except feel. Step 7 is about looks, and is best
judged in play.

## Open questions

1. **Back-jump steering.** Straight out along `outward`, or steerable by the
   stick at the press, as a wall-kick?
2. **Swinging under a free lip.** `KeepAttitude` holds the body upright, but
   the ball joint still lets it swing like a pendulum about the hands. Under a
   free lip (a slab's edge, a platform's rim) there is no face to stop it.
   Should it swing freely, or be damped?
3. **Crouch to drop.** A button press as well as the stick held away? It's
   faster and needs no hold, but it takes away the effort the hold gives.
