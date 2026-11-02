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
- **Loose bodies.** The grip is a physics joint, so a crate on a shelf is
  dragged down by the body hanging from it, and a swinging platform swings
  the hanger. Only a body that bears the hanger has an edge to hold: a
  moving one must weigh `bearing_mass_ratio` (3) times the hanger. A lighter
  one would be dragged about by the grip, as a cardboard box would crumple.

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
    /// Where the body's centre stands after a pull-up; `None` if the edge
    /// isn't climbable. The pull-up's path ends here.
    pub stand: Option<Point3<f32>>,
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

Each hand feels from the body's axis, moved `hand_offset` (0.15 m) to its
side, with spheres of 2 cm radius (`src/character/ledge/probe.rs`):

1. **A top ahead.** Casts down through the catch band, 5 cm apart from the
   capsule's surface out to `reach_ahead` (0.25 m) beyond it and a bevel's
   width (0.3 m) further. The nearest top the fingers hold (within
   `grip_slope`, 40°), followed up while it keeps rising, gives the lip: a
   voxel mesh bevels a lip over about a voxel, and a shallow bevel is held
   at its crest, not wherever it first enters the band, which moved with the
   body (a body pushing into the Ledge Garden's box rode up the bevel, lost
   the edge and bobbed).
2. **The face.** A cast forward finds the face, nearest first at 6 cm, 15 cm
   and 30 cm under the lip: just under a free slab's sharp edge, and below a
   bevel. It must be steeper than `face_slope`, face the body, and be no
   further out than the top.
3. **The hold.** A cast down `grip_depth` behind the face, or where the top
   begins if that is further in, must find the top again, within a few
   centimetres of the lip. That is where the hand holds; how far it is
   behind the face (`Edge::face_depth`) places the hanging body off the
   face, not off the hands.
4. **Room for the hand.** A cast forward `hand_clearance` above the hold must
   be clear to the hand. Otherwise the top is a step in a wall that carries
   on up.
5. **Climbable.** A capsule-radius cast down at `stand_depth` behind the lip
   must find a top within `stand_slope`, and a cast up from there must be
   clear for the body's height.

`outward` is the horizontal part of the faces' normals, summed over both
hands. On bumpy terrain each hand holds its own lip point, so the hands sit on
the real geometry. The edge is grippable only if both hands find a lip on the
same body, and the two lip heights differ by at most `hand_step` (4 cm, about
8° over the hands' 0.3 m spacing).

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
  is a mantle (see *Input*), which reaches down to `mantle_low` (0.05 m
  above the centre).
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
frame. Each frame the anchor stands where the body should be, and the
joint's `anchor_velocity` (new on `BallJoint`) says how fast it moves on to
the next point of its path. The joint's rows carry the body along at that
speed, and the position correction meets only drift.

Moving the anchor a frame's travel at a time instead left the body to the
position pass alone. That exposed an engine bug, now fixed: the ball, fixed
and hinge joints' velocity bias had its sign inverted (`−β/dt · error`
under a solver whose rows converge where `J·v + bias` is zero), pushing
bodies away from their anchors while the position pass dragged them back. A
body carried up at 1 m/s reported −1.46 m/s, and a pull-up −6 m/s while
rising. Rigid joints now carry no velocity bias and the position pass
corrects their drift, as for contacts (`docs/PHYSICS_ARCHITECTURE.md`). A
pull-up rising at 4 m/s now reports 4.2.

- **Grab.** The joint is rigid from the start. A spring catch dips by about
  speed × time constant: 0.6 m at 5 m/s with a 0.12 s constant, and 1.4 m at
  12 m/s. Stiffening it afterwards would then yank the body back up. Instead,
  entering a Ledge sets the body's velocity to the edge body's velocity at the
  anchor (`EntryEffects::match_edge`), so the rigid joint starts with nothing
  to stop. The joint is made where the body was caught, and its end eases to
  the hang over the `settle`: in the Ledge Garden the body sinks 0.3 m onto
  its arms with no snap.
- **Hang.** The body's weight goes into the edge's body: a crate is
  pulled off its shelf, and a platform carries the hanger and feels them.
  The joint is unbreakable. A breakable joint's limit is an impulse per
  substep, and moving the edge end once a frame (the catch's ease, a shimmy,
  a pull-up) asks for a velocity change within one substep: about 24 g to
  start a 1 m/s shimmy, far past any limit a hit should tear at. Tearing the
  grip under a hit (rubble landing on a hanger) is open; it needs the load
  measured over a frame, not a substep. The solver marks a broken joint
  inactive, and `Grip` reports that as a lost grip, so a limit can come back
  without changing the state.
- **Shimmy.** The held edge's hands slide along it at `shimmy_speed`, and
  the anchor hangs under them. The Ledge state's edge is the one place the
  hands are: each frame the sensor finds the lip again, which gives its
  height, face and way out, and the state keeps its own place along it
  (`Edge::held_along`). Re-centred on the body instead, the hands would
  follow it as it drifts behind a moving edge, and slip along the lip. Each
  frame the sensor tests the edge `shimmy_probe` (0.1 m) either way. Where
  it fails (a corner, the end of the lip, a gap), the hands stop.
- **PullUp.** The anchor moves along a lift path:
  - first up, until the capsule's bottom is `lip_clearance` (3 cm) above the
    lip, keeping `face_gap` off the face;
  - then inboard, until the body's axis is over the stand point that the
    climbable test checked (`stand_depth` behind the lip).

  It takes `pull_up_time` (0.6 s), or `mantle_time` (0.3 s) for a mantle. The
  joint is the motor, so the climb pushes down on whatever it climbs. A
  mantle starts from where the body stands, so its path starts there. At the
  end the joint is removed. Entering OnFoot from a Ledge
  matches the body's velocity to the edge's, as the catch did, which drops
  whatever correction velocity the joint built up. The body then stands on
  the top, its contacts ground it, and the state is OnFoot.
- **Leaving.** Drop, back-jump or a lost edge removes the joint, and the body
  leaves with the velocity it has. On a platform, that is the platform's.

**Bodies too light to hold.** An edge on a moving body lighter than
`bearing_mass_ratio` (3) times the hanger is not sensed (`probe::bears`).
The joint moves the hanger along paths in the edge's frame with whatever
force it takes, so a light body is the one that moves: an 8 kg box turned
under a mantle, turned its path with it and was flung off the world, and a
70 kg box rocked against the hanger's capsule. A body that heavy still
answers a hold: a fast mantle onto the 250 kg box tips it towards the
climber, and it settles in half a second.

Whatever velocity entering a state sets on the body (the catch's match to
the edge, the back-jump's whole launch, the pull-up's match at the end), the
edge's body takes the equal and opposite impulse at the grip
(`EntryEffects::push_against`, `match_edge`), unless the body is on its
feet: a mantle begun walking stops on its feet, not its hands. A catch
tugs a platform's deck, and a back-jump kicks it away, as a jump from its
top does. The back-jump's rise is set with
the launch, not conjured as a jump with nothing under it would be.

If the edge's body is removed, `PhysicsWorld::remove_body` already removes
every constraint that references it. `Grip` finds its handle gone, and the
sensor's re-check fails the same frame.

## Consumers

The roles follow the state machine's rule 8: an arm for every state and no
wildcards.

| Consumer | Ledge |
|---|---|
| Planner (`PlanRole`) | Off |
| Legs | Off. The pose draws the feet, and the legs are handed them (`FootPlacer::hold_feet`), so a climber's feet on the top are where the gait picks up. |
| Arms (`UpperState`) | `Gripping`: the hands on their holds on the lip, read through the edge's body each frame, the elbows solved to them. In Jump and Fall, `Braced` arms rise to the holds of an edge the sensor finds along the arc (`LedgeSensor::ahead`), from 0.3 s out. |
| Pose | `Hanging`: the posture, the legs' swing, and the climb (`animation/humanoid/hang.rs`) |

### Animation

`animation/humanoid/hang.rs`. The animation system reads the held edge from
physics into a `LedgeReading` (the hands, the face, the stand point, the
phase and lean, and the edge's velocity), so the animator stays
physics-free. Nothing is a canned clip:

- **Posture.** The shoulders go where the physics body puts them, the figure
  leaning in towards the wall (it is slimmer than the capsule). They are drawn
  in to the hands only as far as the arms are long, and less far again while
  leaning to pull, which lifts the figure. The chest stays off the face. The
  catch shows the body sinking onto its arms as the joint eases it to the
  hang.
- **Legs.** A damped pendulum from the hips, set swinging by every change in
  the body's velocity relative to the edge: the catch, a platform's reversal.
  The knees give on a catch and settle. With a wall below (one cast from the
  shins, `LedgeSensor::wall_below`), the legs hang in to it and the toes rest
  on it, never in it. Those feet are anchored to the world, so a back-jump or
  drop blends out of them as a kick off the wall.
- **Hands.** Planted on the edge like feet on the ground. A hand left 7 cm
  from its place lifts and moves past it in an arc, one hand at a time: hand
  over hand along a shimmy, a shuffle as a turning deck carries the body.
- **Climb.** The capsule has to clear the lip, which puts its pelvis higher
  than hands on the lip can reach. So the figure follows its own path, held
  to the edge: from the hang (or, for a mantle, from standing), to over the
  lip (hips at the lip, chest over the top, shoulders above the hands, the
  lead knee up and its foot on the top), to standing. The hands stay
  planted, so the elbows bend through the pull and press at the top. The
  trailing leg steps up, then the lead foot moves from the lip to stand
  under its hip.
- **Back-jump.** One going straight out turns in the air to face its flight.
| Grab system | `arm_state_allowed`: only `Idle` in a Ledge; entering one releases what is held |

## Testing

**Unit tests** (done):
- The edge test on synthetic geometry, `src/character/ledge/tests.rs`:
  - wall tops at heights through the band, and outside it;
  - a 2 cm lip, which is not grippable;
  - a 6 cm lip, which is grippable and not climbable;
  - a top under a low roof, which is grippable and not climbable;
  - a top tilted at 30° and 50°;
  - a free slab edge;
  - a terrain wall, as static geometry;
  - two hands at lip heights that differ by 2 cm and by 6 cm;
  - an edge carried by its body when the body moves and turns.
- The same on the Traversal Garden's cave roof as voxel terrain,
  `garden_tests.rs` (`traversal_scenarios`).
- The state machine on scripted facts, `locomotion/ledge_tests.rs`:
  - catching from a fall, not from a Hop or a jump still launching;
  - the water giving up a swimmer to an edge, and never taking a hanger;
  - the mantle, on foot and from the air, and what does not mantle;
  - the settle, a drop committed and one let go early;
  - a pull-up committed only where the body can stand;
  - a shimmy leaning nowhere;
  - crouch dropping at once;
  - the back-jump, straight or steered by the stick;
  - an edge lost, or found on another body;
  - a pull-up run onto its feet at the edge's velocity;
  - `hands_free`.

**`character_viewer` scenarios** in the Ledge Garden
(`levels/ledge_garden.level.ron`, `character_viewer/ledge_scenarios.rs`):

| Scenario | Result |
|---|---|
| `ledge_hang` | a full jump at a 3.2 m wall (its top meshes at 3.05 m) catches the lip falling from the apex and hangs at rest, the centre 0.53 m under the lip |
| `ledge_jump_mantle` | the same jump pushing into the wall climbs it from the top of the jump |
| `ledge_drop` | the stick held away drops after 0.2 s |
| `ledge_crouch_drop` | crouch drops on the press |
| `ledge_pull_up` | the stick held in pulls up after 0.2 s, onto the top in 0.6 s |
| `ledge_back_jump` | jump leaves straight out from the wall |
| `ledge_shimmy` | 1.5 m along the lip in 1.5 s |
| `ledge_heavy_shimmy` | along a 14 t box's lip until the hands reach its end |
| `ledge_no_room` | under a roof, pushing in only strains |
| `ledge_mantle` | walking into a 0.72 m box climbs onto it |
| `ledge_light_mantle` | walking into an 8 kg box 0.66 m high shoves it: too light to hold |
| `ledge_heavy_mantle` | walking into a 250 kg box 0.66 m high climbs onto it; the box tips under the climb and settles |
| `ledge_light_tall` | a jump at a 70 kg box 3.1 m high does not catch it: too light to hold |
| `ledge_heavy_tall` | a 14 t box 3.1 m high: hang, pull up, stand on top |
| `ledge_platform_lift` | catches the rim of a platform rising from 1 m to 3.5 m, rides it up and back down, and stands when its feet reach the ground |
| `ledge_platform_side`, `ledge_platform_diagonal` | catches the rim of a platform going side to side, and one going diagonally to the side and up, and rides it; at a reversal the hanger's momentum tugs the deck's edge and turns the 270 kg deck a little |
| `ledge_platform_still` | hangs from a platform holding still, pulls up and stands on the deck, which tips under them |

The Traversal Garden's `ledge_walk` (its 0.7 m ledge) now crosses on 5 of 5
starts by mantling. Its baseline in `docs/traversal/BASELINE.md` was 0 of 5,
bouncing against the wall.

Still to build, with step 8: `ledge_from_water` (swim to a 1 m bank and
climb out), dropping off a moving platform with its velocity,
`ledge_crate` (hang from a crate on a shelf, heavy enough to hold, which
comes down),
`ledge_blasted` (a blast takes the lip, and the body falls), and a fall down
a wall's face that catches its lip.

## Steps

Each step ships with the game working, and its tests.

1. **`WorldCaster`.** The world-space core split out of `PhysicsCaster`. No
   change in behaviour: captured fixtures stay identical (done).
2. **The edge test and `LedgeSensor`.** Grippable and climbable, sensed
   every frame for the player and drawn with the plan's overlay (F6): the
   hands yellow, the way out a yellow line, the stand point green. Unit tests
   on boxes, and on the Traversal Garden's cave roof as voxel terrain
   (`traversal_scenarios`) (done).
3. **The `Ledge` state.** Phases, input timing, exits, `hands_free`, the new
   movement rule and entry effects, and every consumer's mapping. The
   sensor's catch conditions (rise, facing, speed, the regrab lock) and the
   held edge always seen, which need the state. Unit tests on scripted
   facts (done).
4. **The grip.** Catch, hang, drop and back-jump, and `ledge_garden`'s first
   walls (done).
5. **Shimmy** (done).
6. **Pull-up and mantle** (done).
7. **Arms and pose.** Hands on the edge, the arc prediction and the reach
   ahead, and the hang, lean and climb poses (done).
8. **Water, platforms, loose bodies and blasts**, and their scenarios.

Steps 1 to 6 need no input except feel. Step 7 is about looks, and is best
judged in play.

## Decisions

Taken with the user on 2026-11-02:

1. **Back-jump steering.** With the stick neutral or pushed towards the
   ledge at the press, the back-jump goes straight out along `outward`,
   locked for `back_jump_lock`. With the stick pushed any other way, it
   leaves along `outward` and is steered by the stick, as any jump is.
2. **Swinging under a free lip.** It swings freely on the ball joint.
3. **Mantle from OnFoot.** A body on its feet, pushing the stick into a
   climbable edge under its shoulders, mantles it: OnFoot enters a Ledge
   straight into PullUp. The sensor therefore runs on foot too, and
   `hands_free` is true in OnFoot. A jump that reaches a chest-high box
   clears it, so this is how mantles mostly happen. A mantle reaches lower
   than a catch: from `mantle_low` (0.05 m above the capsule's centre, lips
   from 0.55 m above the feet), because on foot a 0.7 m box's lip measured
   right at the chest bound and was missed. A grab from the air still needs
   the lip at the chest (`catch_low`).
4. **Crouch drops at once.** Pressing crouch while hanging drops
   immediately. Holding the stick away still takes `commit_hold`.
