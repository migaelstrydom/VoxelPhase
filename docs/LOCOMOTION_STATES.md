# Locomotion states

One state machine says what the character is doing. Everything else (the
traversal planner, the legs, the arms and the pose) responds to it and never
decides for itself whether the body has left the ground.

This is a restructuring, not a faithful refactor. The locomotion system hides
more bugs than any other part of the game, and its feel and animation are not
finished, so the aim is a system simple enough that tuning it is easy. A
behaviour of today's code that only a special case can reproduce is assumed to
be a quirk, not a requirement.

## Why

Four state machines interpret a character's motion today, each from its own
inputs:

```
LocomotionState (character)  Grounded · Launching · CoyoteTime · Airborne · Swimming
   │
   ├─► next_pose_state (+ the animator's own `afloat` test) ─► PoseState
   │        │                      Grounded · Launching · Airborne · Landing · Swimming
   │        └─► airborne: bool ─► FootPlacer `Flight`
   │                               Grounded · Flying · Hanging · TouchedDown
   │                                   ▲ PlannedSteps, each frame
   └─(never read)                      │
TraversalPlanSystem: plans while the body moves and is not riding a carrier;
                     hands footfalls while tracking and the plan crosses;
                     steers the intent and can ask for a takeoff
```

Each has its own rules for leaving the ground, and they disagree:
- The planner kept planning for a swimmer and steering it.
- The legs took a swim for a flight, kept the shore's planted feet, and flew
  the footfalls the plan handed them.
- Leaving the water over a drop carried all of that into the fall, and the
  legs were drawn reaching back up to the edge.

Each such bug was patched where it showed (a mid-air plant in `fly_swing`,
`swings_onto_footfall`), adding a rule to one machine for a state another one
owns. "Cancelling the plan" happens in three files, on three different
signals.

The next abilities would each add a case to every one of those machines:
grabbing a ledge, hanging, shimmying, pulling up, dropping, and climbing and
jumping out of the water.

## Rules

1. **Only `LocomotionState::tick` changes the state, and only the machine's
   own `enter(from, to)` says what changing it does.** `tick`'s inputs are
   facts (grounding, the water at the body, an edge at the hands) and requests
   (jump pressed, a plan's takeoff, grab, drop), and it returns the next state.
   `enter` returns the entry effects: an impulse, an air-speed cap, and the
   effects the driver applies on its own today:
   - clearing the jump buffer;
   - cancelling ground forgiveness;
   - a buffered tap's cut rise;
   - the crouch lockout after a long jump.

   Deciding when to change and what changing does are kept apart, the same
   split every consumer has (rule 3).
2. **Every other part maps the state to a role.** The planner, the legs, the
   arms and the pose each have a small role enum and one function from the
   state (and, for the arms, the grab state) to it. None of them decides its
   own role. The same goes for any system that needs to know what the
   character is doing, such as the camera, the grab system, or a vehicle
   finding its rider. It reads the state, never its own guess from grounding,
   contacts or position.
3. **A role change is handled in one place per consumer.** Each consumer
   keeps the role it had last frame, and when the mapping changes it calls its
   one `enter(from, to)`. That is where state carried across frames is kept or
   dropped: a plan, a gait clock, swings, footfall ids.
4. **Consumers ask; they do not switch.** A consumer that wants something of
   the state (the plan wants to hop or has moved a Hop's landing, the hands
   found an edge) passes a request to the next `tick`. The requests are named
   fields of `LocomotionInput`, like `takeoff` today.
5. **A state is where the rules differ.** Walking, running, crouching and
   standing are gaits inside one state, because the same rules move the body
   and only the speed and pose differ. Shimmying is moving along a ledge while
   hanging from it, not a state of its own.
6. **Transitions may start transients; transients decide nothing.** A landing
   squash, a water-entry plunge, a blend between poses: the pose plays them on
   a role change, with their own timers, and nothing reads them back.
7. **No consumer reads another's role.** Consumers may share data (the planner
   reads where the feet are and which footfall each stands on) but never
   whether another one thinks the body is flying.
8. **Mappings name every state; no wildcards.** A role mapping is a `match`
   with an arm for each state and no `_ =>`. Adding a state then fails to
   compile at every consumer until someone decides its role there. Today's
   mappings default silently (`map_upper_state` sends anything unlisted to
   `Braced`), which is how a swim came to be treated as a flight.
9. **Rules that cut across states are questions the state answers.** The
   water taking the body, and the hands being free to grab an edge, apply
   from many states. Each is a method on the state with an exhaustive
   `match`, such as `water_takes(&self) -> bool` and
   `hands_free(&self) -> bool`, not a list of exceptions inside the water rule
   or the ledge sensor. A new state answers each one, or does not compile.
10. **A state does not care how it was entered.** A Ledge is the same Ledge
    whether it was grabbed from a fall or from the water; Hop and Jump share
    one takeoff type, so "down" means the same for both. Where the previous
    state matters (feet landing from a Hop or a fall, a squash scaled by what
    landed, the crouch lockout after a long jump), only an `enter(from, to)`
    looks at it, never the state's own rules.

## States

The gameplay state, with what each consumer makes of it. The "Leaves for"
column gives the request or fact that causes each transition.

| State | Entered from | Leaves for | Planner | Legs | Arms |
|---|---|---|---|---|---|
| `OnFoot { grace }` | Hop, Jump, Fall (down), Swim (feet down), Ledge (pull-up done) | Hop (a plan's takeoff), Jump (jump pressed), Fall (`grace` runs out), Swim (deep water) | On | Step | Swing |
| `Hop { phase, landing }` | OnFoot (a plan's takeoff) | OnFoot (down), Fall (landing missed), Swim | On | Fly | Swing |
| `Jump { kind, phase, steering }` | OnFoot, Swim (jump out of the water) | OnFoot (down), Swim (falling into deep water), Ledge (an edge at the hands) | Off | Off | Brace |
| `Fall { steering }` | OnFoot (`grace` runs out), Hop (landing missed), Swim (out over a drop), Ledge (drop) | OnFoot (down), Swim, Ledge (an edge at the hands) | Off | Off | Brace |
| `Swim` | OnFoot, Fall, Jump falling, Ledge, once the water is deep | OnFoot, Fall, Jump, Ledge | Off | Off | Stroke |
| `Ledge { edge, phase }` | Jump, Fall, Swim (an edge at the hands) | Fall (drop), OnFoot (pull-up done), Swim | Off | Off | Grip |

### Takeoffs: Hop and Jump

Hop and Jump take off the same way. Each has a `phase`:
- **`Launch`** while the body is still in contact: at most `launch_window`,
  ending when the body leaves the ground.
- **`Flight`** after that.

"Down" means grounded during `Flight`, so a body still on the ground at
takeoff, or a stance foot left on a lip, does not count as landed. Every
takeoff cancels ground forgiveness, so grounding drops as soon as the contacts
do.

### OnFoot

On the ground, with a gait chosen from the intent and the speed. `grace` is
the time left after contact is lost (today's `CoyoteTime`): still OnFoot,
still able to jump or long jump, with upward velocity cancelled. When `grace`
runs out the body falls. Allowing a long jump in grace is new; it falls out of
grace being OnFoot.

### Hop

A flight the plan took off on: a running leap, a hop up a step, or a planned
roll-off at a lip.

**Which flights request a takeoff.** The tracker asks for one where a phase
the plan stands on hands over to a flight within half a frame. A hop asks as
it does today, with the climb it needs over the body's own. A roll-off asks
with no added climb: `PlannedTakeoff`'s vertical speed becomes optional. A plan
made in the air starts in flight and asks for nothing. An unplanned walk-off is
grace, then Fall.

**The landing.** The Hop carries its `landing` (which foot, where, and when)
from the takeoff. The planner may move it during the flight through one
request, `LocomotionInput::landing`. It is the only source the legs fly the
feet onto. A plan that goes quiet for a frame leaves the landing where it was.
Ending the Hop on a quiet frame instead made landings worse on three stations
when the legs were tried that way (2026-10-30).

**A missed landing.** The Hop is a Fall when its landing time has passed with
the body still in the air, or when the landing is out of a leg's reach of
where the hip comes down (the stick steered away).

**Touching down.** The Hop goes straight back to OnFoot, with the landing foot
already on its footfall. There is no landing state between them.

The Hop replaces `LaunchKind::Leap`, the placer's `Flight` machine and its
fly-on rule, and the patches for falling past a landing.

### Jump

A jump the player pressed for. `kind` is a jump, a long jump, or a jump out of
the water. Jumping in grace goes straight to `phase: Flight`, as it does today.
The planner is off from the press until the body is back on its feet, so the
plan has no say in the jump's arc or its landing. The feet land under the
body, where the probes find the ground. The water does not take a Jump while
it is rising, so a jump out of the water is not caught by the water it left.

### Fall

In the air without a jump or a plan: walked off an edge, missed a Hop's
landing, dropped from a ledge, or left the water over a drop. Air steering is
as today.

### Swim

In water too deep to stand in. The animator's `afloat` test goes, and its job
moves to the exit rule. A swimmer standing up in wadeable water stays in Swim
until its feet are down. It leaves for Fall only when the water is gone or it
is `leave_height` above the level. Today `leaves_water` exits as soon as the
water is shallower than `stand_depth`, feet down or not, and the animator
covers that gap with a tread. Entering the water is unchanged, apart from
ignoring a rising Jump.

An edge at the hands is a Ledge, which is how climbing out of the water
works: the same grab and pull-up as from the air, with the water holding the
body up instead of the arms.

### Ledge

Hands on an edge. `phase` is one of:
- `Grab`: the hands reaching onto the edge.
- `Hang`: the stick moves the body along the edge, which is shimmying.
- `PullUp`.

A drop is a Fall, and a finished pull-up is OnFoot on the edge's floor.
Entering a Ledge drops whatever the hands hold. Which edges count, and how the
hands find them, is for the ledge project's design. What matters here is that
the edge comes from a hand sensor of its own, not the planner, because most
grabs start in a Jump or a Fall, where the planner is off.

## The consumers

### Planner (`src/traversal/`)

```rust
enum PlanRole { Off, On }
```

`On` plans, hands the legs their footfalls, paces the drive and asks for
takeoffs and Hop landings. In a Hop the stick sets the pace, which the tracker
already reads from the plan being in flight. `Off` does nothing at all: no
strip, no plan.

The planner has one `forget()`, which drops the plan, the previous path and
the landing bookkeeping. The state machine reaches it only through
`enter(On, Off)`. The planner also calls it itself when it has nothing to
plan:
- standing still;
- riding a carrier, where it returns early as if refusing, because it handles
  carriers badly.

Neither of those is a role.

### Legs (`FootPlacer`, `LeggedLocomotion`)

```rust
enum LegMode { Step, Fly, Off }
```

| Mode | The placer | The probes |
|---|---|---|
| Step | the gait: clock, swings, capture point, the plan's footfalls if it has any | aim at the feet |
| Fly | the landing foot swings onto the Hop's landing, the other trails | aim at the swing targets |
| Off | idle; the pose draws the legs | straight down |

`enter` is the whole of what is now the `Flight` machine, `touch_down`, the
fly-on rule and the held-swing rule:
- **`enter(Fly, Step)`** lands the flown feet as they are.
- **`enter(Off, Step)`** keeps each planted foot that is still within its
  leg's reach of its hip, and plants the others at stance. A short drop, or a
  creature's grounding flickering, therefore does not move feet that are still
  under the body.
- **`enter(_, Off)`** drops every swing and footfall.

Creatures keep using `LeggedLocomotion`, mapping their own grounding to Step
or Off. They have no plan and no water.

### Arms (`UpperState`)

The arms' role is a function of both the state and `ArmState` (the grab
system: reaching for or holding an object). Reaching and holding win where
they are allowed, as today. Otherwise:

| State | Arms |
|---|---|
| OnFoot, Hop | Swing |
| Jump, Fall | Brace |
| Swim | Stroke |
| Ledge | Grip |

Today a Hop braces the arms. Swinging through it is deliberate: a hop is a
running leap that flows out of the run and back into it. Which `ArmState`s each
state allows is checked in one place, as `CharacterState::arm_state_allowed`
is now.

### Pose (`PoseState`)

The pose is a mapping of the state: OnFoot by gait, Hop, Jump by kind and
phase, Fall, Swim by stroke, Ledge by phase. On top of that are the transients
it plays on a change:
- a landing squash on entering OnFoot from the air, scaled by what landed:
  light for a Hop, as the Leap's is today, and heavy for a long jump;
- the water-entry plunge on entering Swim from the air.

The pose no longer splices in a `Landing` state, runs its own water test, or
reads whether the placer is flying the feet (`feet_flown`). During a Hop the
legs are always the placer's, and outside one never.

## Structure

Where the pieces live, and which way the dependencies point.

```
character (gameplay)                         consumers (each owns its mapping)
─────────────────────────────────────        ────────────────────────────────────────
Conditioning                                 traversal/   PlanRole::of(&state)
  jump and crouch buffers, ground            animation/   LegMode, arms' role, pose
  forgiveness, crouch lockout                             ::of(&state, …)
  raw intent + contacts ─► facts, requests   sled, camera, grab: read the state
        │
        ▼                                    generic parts see only their own role:
LocomotionState  (enum over state structs)     LeggedLocomotion / FootPlacer ◄─ LegMode
  tick     ─► next state                       traversal_planner crate        ◄─ nothing
  enter    ─► entry effects ─► Conditioning,
              driver                         request types are the character's:
  movement_rule, water_takes, hands_free       PlannedTakeoff, LandingUpdate, EdgeAt;
                                               producers fill them
```

- **One file per state.** `src/character/locomotion/` holds the enum and the
  machine's `tick` and `enter`, and a struct per state (`OnFoot`, `Hop`,
  `Jump`, `Fall`, `Swim`, `Ledge`) in its own file. Each file holds that
  state's exits, its movement rule and its answers to the cross-cutting
  questions. A state's rules can then be read in one place, the enum
  dispatches to them, and the table above is the overview.
- **Conditioning is its own stage.** The jump and crouch buffers, ground
  forgiveness and the crouch lockout turn the raw intent and contacts into
  the facts and requests `tick` reads. Today these are loose timers on
  `CharacterState`, ticked by the driver. `enter`'s effects reset them
  (consume the jump, cancel forgiveness, arm the lockout), and nothing else
  does.
- **Mappings live with their consumer.** `PlanRole::of` is in
  `src/traversal/`, and the leg, arm and pose mappings are in the humanoid
  animator. The character module depends on none of them. `LeggedLocomotion`
  and `FootPlacer` take a `LegMode` and never see `LocomotionState`, which is
  what lets creatures use them with their own grounding. The planner crate
  sees neither.
- **The requests are the character's types.** `PlannedTakeoff` already lives
  in `src/character/`; the Hop's landing update and a found edge join it. The
  tracker and the hand sensor fill them in, and the machine never depends on
  either.

Two alternatives were considered and rejected:
- **Narrower interfaces.** Consumers could read a few facts from the state
  (on the ground, in the air, in the water) instead of the whole enum. But the
  planner needs Hop apart from Jump, and the legs need Hop apart from Fall:
  facts coarse enough to stay few lose exactly those distinctions, and they
  drop rule 8's compile check.
- **States open for extension.** States could be trait objects that know
  their own roles, so adding one touches no consumer. That would put the
  planner's, the legs' and the pose's concepts inside the character module,
  pointing the dependencies the wrong way, and lose the exhaustive matches.
  The design accepts that a new state edits every consumer, because each edit
  is a decision the compiler asks for. A new consumer touches nothing in the
  machine.

## A frame

```
player_input / brain ─► CharacterIntent
traversal_plan        PlanRole from the state as last ticked: plans,
                      paces the intent, may request a takeoff or a landing
character_control     conditioning ─► facts, requests
                      LocomotionState::tick ─► next state; enter(from, to) ─► effects
                      the state's movement rule ─► the drive
physics
animation             LegMode, the arms' role, the pose from the state;
                      enter() on each role that changed; transients
```

The planner sees the state a frame late, as it already does. Its
`enter(On, Off)` runs on its first frame after the transition. Nothing it
hands the legs in between matters: the legs mapped the new state first, and
what the planner hands them lasts only one tick.

## What goes

- The placer's `Flight` machine, `set_suspended`, `touch_down`, `left_behind`,
  the fly-on rule and `swings_onto_footfall`; the animator's `feet_flown` and
  its blend.
- `CoyoteTime` as a separate state (it is OnFoot's `grace`), `Launching` as a
  separate state (it is a takeoff's phase), and `LaunchKind::Leap` (it is a
  Hop).
- The animator's `afloat` test and the `Landing` splice in `next_pose_state`.
- The traversal system's planning for bodies that are falling, jumping or
  swimming. `stand_aside` becomes the planner's `forget()`.
- The planner design's ledge, hang and climb phases
  (`docs/traversal/PLANNER_DESIGN.md`): the planner is not extended past
  walking and hopping.

## What stays

- The traction drive and the movement rule per state.
- The planner and its tracker, for OnFoot and Hop.
- The placer's gait (clock, swings, capture point), for Step and Fly.
- Swimming's movement and attitude.
- The grab system.
- The pose samplers and the skeleton.

## Steps

Each step ships with the game working, and its tests.

1. **The state machine.** `src/character/locomotion/` with `OnFoot`, `Hop`,
   `Jump`, `Fall` and `Swim`, each in its own file. It includes:
   - the takeoff phases;
   - the water's exit rule;
   - `enter` with the entry effects;
   - the conditioning stage;
   - the tracker's takeoff and landing requests. A table test over
   the transitions. The existing consumers are fed from the new state through
   a temporary mapping to what they read now.

   Done (2026-11-01). Where it differs from the text above:
   - `Fall` carries no `steering`: nothing enters it with a lock, since a
     long jump stays a Jump until it is down.
   - `SwimConfig::stand_depth` is gone. The exit rule needs only
     `swim_depth` (wadeable, with the feet down) and `leave_height`.
   - A missed Hop's slack is `LocomotionConfig::hop_landing_slack`
     (0.05 s), and its reach comes with the landing
     (`PlannedLanding::reach`, from the planner's leg reach and hip height),
     judged across the ground.
   - The tracker does not yet ask for roll-offs, though `PlannedTakeoff`
     carries an optional climb and the machine takes them. Asking made the
     garden worse (`lip15_run` snagged 27 frames, not 7, and `scree_run` took
     2.72 s, not 2.47) for two reasons. The plan models a roll-off as keeping
     its rise for the forgiveness window, after which grace cancels it
     (`BodyDims::rise_held`), and a Hop does neither. And under the temporary
     mapping a Hop's launch is a leap pose, which suspends the placer at the
     lip. Roll-offs became Hops with step 3.
   - With that left out, the garden (`--track --starts 3`) and the turning
     runs measure as before, to the third decimal. The one deliberate change
     shows in `ashore`: the swimmer crawls until its feet are down, then
     walks. It no longer goes out of Swim into the air, a tread and a
     landing.
2. **The planner's role.** `PlanRole`, `enter` and `forget()`, replacing the
   traversal system's own conditions.

   Done (2026-11-01). `PlanRole::of` is in `src/traversal/role.rs`, and
   `TraversalPlanState` keeps last frame's role and its `enter`. `forget()`
   also drops the strip's kept casts and fetched scenery, since terrain
   changes go unwatched while the planner is off. Standing still forgets
   too, so a body that starts moving again lays a fresh path. Body motion
   across the garden and the turning runs is unchanged. The feet move
   both ways. On scree, fewer footfalls are moved under a swinging foot
   (20 to 2 walking) and slip is lower running. Where a body comes off a
   brow into grace and a short Fall (`lip8_walk`, `lip15_run`), the landing
   foot is no longer handed the plan's footfall, and the worst miss grows
   (0.14 to 0.39 m on `lip8_walk`). Those flights become Hops, with the
   planner on, once roll-offs do (step 3).
3. **The legs' mode.** `LegMode` and its `enter`, replacing `Flight` and the
   patches. Until step 4, the pose's `feet_flown` is read from `LegMode::Fly`.
   The garden and turning summaries will move; they are judged by whether the
   gait looks and plays better, not by matching the baseline.

   Done (2026-11-01), together with roll-offs becoming Hops. `LegMode` is in
   `src/animation/foot_placer/mode.rs`. The humanoid animator maps the state
   to it (`leg_mode`), and creatures map their grounding to Step or Off.
   The placer keeps the mode it was last set and enters a change on its
   next tick. Where it differs from the text above:
   - **Entering Off drops the footfalls but keeps the swings, frozen.**
     Creatures map raw grounding, which chatters, and dropping a swing on
     every blip would jump that foot to stance. Off → Step judges every
     foot by where it is, planted or hung mid-swing: within its leg's
     stretch of its hip it is kept, otherwise it is planted at stance. A
     swing's landing is not what is judged; at speed it lies past the leg's
     reach.
   - **`feet_flown` is the placer's `flies_to_landing`**: in Fly, with a foot
     swinging onto the landing. A landing out of reach of where the body
     comes down flies no foot, and the pose draws the legs until it is in
     reach again.
   - **A roll-off Hop keeps its rise for its own time**,
     `LocomotionConfig::roll_off_rise_held` (0.08 s), counted from losing
     contact (`Facts::since_contact`), and then cancels it. The planner's
     `BodyDims::rise_held` reads the same number. That makes how crests
     feel a dial of its own, separate from ground forgiveness. Two things
     had doubled the held rise. A roll-off has no impulse, so the frame
     after takeoff is still on the brow, and that contact re-armed the
     forgiveness the takeoff had cancelled; a cancelled window now stays
     closed until the contact ends. And a plan made in the air asked for
     its roll-off after the body had left the brow, so the rise had already
     been held.
   - **The contract's brow check runs with the plan driving.** Untracked, it
     measured forgiveness, which only coincides with `rise_held`.

   - **A Hop's landing carries the plan's name for its footfall**
     (`PlannedLanding::footfall`, through `FlightLanding` to the swing). A
     body comes down a frame or so after the plan has it land; by then the
     plan's next step for that foot no longer lands a flight, so the legs
     cannot take the name from it. A foot planted without its name is not
     the plan's, and the planner drops its commitment and lays new
     footfalls under the swinging feet.

   Measured (`--track --starts 3`) against step 2: every station that
   crossed still crosses (the ledge and cave lip are step 6), and the
   turning runs and the contract (47 rows) are unchanged. Lip landings are
   fixed: the worst miss on `lip8_walk` falls from 0.39 to 0.05 m, and on
   `lip15_walk` from 0.49 to 0. The planks and rough field land as well as
   they did, and `rough_run` better (worst miss 0.03 to 0). Bounce is lower
   on the lips and slopes. One thing is worse: a roll-off's air speed is
   the plan's, off the brow of the 15 cm lip at a run 6.2 m/s, where grace
   kept chasing 8 m/s. `lip15_run` snags 26 frames, not 7, for 0.01 s. The
   scree takes 2-3 % longer.
4. **The pose and the arms.** Mappings and transients, replacing the splice,
   the `afloat` test and `feet_flown`.
5. **Out of the water.** Jump from Swim.
6. **Ledges.** The hand sensor, then `Ledge` with Grab, Hang and PullUp, from
   the air and from the water.

A `character_viewer` scenario that swims off the edge of island_sea and falls
checks steps 1 to 4 together: the planner off, the legs off, and nothing
reaching back to the edge.

## Open questions

1. **`GroundForgiveness` and `grace`.** Both bridge brief losses of contact.
   Forgiveness holds grounding true before the state machine sees the loss;
   grace keeps the state after it does. One mechanism would do. Which one, and
   how long, is a feel question.
2. **A missed Hop's slack.** How long past the plan's landing time a Hop may
   still touch down before it is a Fall. Zero is simplest, but a body landing
   a frame late on a step's lip should probably still count.
3. **Landing as a state.** Not planned: Jump and Fall go straight to OnFoot,
   and the feet plant under the body. If landings need placed feet, or a
   gameplay cost (a heavy landing that slows the body), a `Landing` state with
   the planner on is where that goes.
4. **Creatures' grace.** Creatures feed raw grounding to their legs. The keep-
   what-is-in-reach rule stops their feet jumping, but a short grace like the
   player's might read better. That's a question for when creatures get a
   state machine of their own.
