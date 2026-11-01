# Traversal planner design

How the player crosses broken ground: lips, plank bridges, stepping stones,
rubble that gives way: walking and hopping, nothing more. Ledge grabs,
hanging and climbing are locomotion states with the planner off
(`docs/LOCOMOTION_STATES.md`), not planner moves. This supersedes
`docs/TRAVERSAL_PLAN.md`, which was abandoned on 2026-10-19.

## Why the last attempt failed, and what this does differently

The last attempt held the body up on a spring (the suspension) and put a move
state machine on top (Gait/Fall/Land/Drop/Halt). The spring sagged under load,
so the drawn legs clipped into terrain, and its downward cast found steep
slopes beside the body and held it hovering. Every move inherited those
faults, and every pair of moves needed its own handover rules. The edge cases
multiplied with the number of moves.

This design keeps three rules:

1. **The capsule is the body.** It stands on its contacts and is moved by the
   traction drive. No spring and no kinematic override. A plan is a reference
   the body *tracks*: when rubble gives way, the body really moves with it and
   the planner replans from where the body actually is.
2. **One plan type, not a set of moves.** A step up, a hop from plank to
   plank and a step down are the same thing, a footfall, at different heights
   and gaps. The plan is a list of contact phases (who is touching what) and
   flight phases (ballistic arcs). The drive and the animation read phases;
   neither knows a move's name.
4. **2D in its search, 3D in its data.** Footfalls are ordered by progress
   along a path, which keeps the search small and gives the side view as the
   picture to debug with. But footfalls, surfaces and normals are 3D: a foot
   lands to either side of the path, and a stone off the line or a rock tilted
   sideways is seen as such.
3. **The planner is pure.** It takes a section of the ground ahead and a body
   state and returns a plan. No physics, ECS or GPU. That lets it run unchanged in the game, in
   a browser tool, in tests and in a fuzzer, and edge cases become fixtures
   instead of memories.

## Architecture

```
                         GAME (voxel_phase)                    │ PURE (traversal_planner crate)
                                                               │
 CharacterIntent ──► PathPredictor ──► SectionFrame            │
 body state             (the arc the stick steers along)       │
                              │                                │
                              ▼                                │
 PhysicsWorld ───► PhysicsCaster ─── sphere casts down / up ──►│ Section: a strip of lanes
 (terrain, rubble,   (memoised per frame)  in every lane       │   surfaces in (s, d, z), each with
  spawnables)                                                  │   its footing (fixed / loose)
                                                               │      │
                                                               │      ▼ read only through SectionQuery
                                       previous Plan ─────────►│ Planner: body path, then footfalls
                                                               │      │   (body moves: ride, fly;
                                                               │      │    nothing else)
                                                               │      ▼
                              ┌─────────────────────────────── │ Plan: [Phase]  (committed prefix
                              │                     │          │        + tentative rest)
                              ▼                     ▼          │
                        PlanTracker            FootPlacer      │ also consumed by:
                        (traction drive        (footfalls; a   │   section_svg (CLI),
                         command, takeoffs)     flight's landing time)  fixture tests, fuzzer
                              │                                │
                              ▼                                │
                        PlanOverlay (DebugOverlays), capture key ──► scene file (RON) ──► fixtures
```

The planner crate does not depend on the game. The game depends on it.

## The section: a strip of ground ahead

A `SectionFrame` maps the world to path coordinates: `s` is the distance along
the predicted path, `d` the signed offset across it (positive to the left),
`z` the height. The path is a straight line laid from the body along the
heading (or along a wall the heading meets), and laid afresh once the
heading has turned 2° from it, the body has drifted 2 cm off its line
along a heading turned from it at all, been shoved 0.2 m off it, or
climbed or dropped far from where it was laid. A gentle turn, the stick
swung by the camera, relays it every few frames; a nudge of the camera
under 2° relays it once the body has drifted 2 cm (it drifts 1.7 cm a
metre per degree, and the feet are planned about the line, so had stood up
to 0.2 m beside the body on a long straight run). Rough ground and scree
shove the body off its line and back without turning its heading; relaying
on those 2 cm made scree relay forty times a crossing, each relay moving
the kept footfalls off the footholds between the stones, so a shove keeps
the line to 0.2 m.

A path laid afresh carries the last plan onto it, turned with the path: each
footfall keeps its offset across the path and the body's path is laid along
the new line, so commitments survive a turn. Kept where they were in the
world, the footfalls would end up further outside the body's curve with
every relay (they did, by 0.2 m at a 6 m radius, out of the strip); turned
with the path, a footfall's error is the curve's offset from the line ahead,
which shrinks to nothing as the body comes up to it. Past 10° of turn the
plan is made anew. The strip's casts are keyed to the path, so a relay casts
it again; one carrying a plan is cast to its full reach at once (about 3000
casts, 3 ms), since a plan carried with less in sight refuses what it
crossed a frame before. Only a path with no plan starts short and grows a
metre a frame. Casting a world-fixed patch instead, which a turned path
reads without casting, is the production answer (*Beyond one path*).

The section is a **strip** of lanes at fixed offsets `d` across the path. The
first version has five lanes over ±0.3 m: the centre lane for the body, and
all five for the feet, which land about ±0.1 m either side of the path and
step wider or cross over to reach a stone. The body floor and the ceiling
are one profile each along the path itself; each lane has a profile of
footholds. A profile is a list of **surfaces**: runs of samples on one
body, split where the body under them changes, where a cast finds nothing and (for
footholds) where the ground is too steep to stand on, so a surface's ends are
its edges.

| Field | Meaning |
|---|---|
| `footing` | `Fixed`, or `Loose { mass, resting }` for rubble and spawnables: physical facts, not the engine's identity for the body. The planner's cost weighs them; the game only reports them |
| `points` | samples in increasing `s`: the height of the cast sphere's centre at rest, and the full 3D contact normal in the path frame (it keeps cross-slope, so a rock tilted sideways reads as tilted) |

Heights are the cast sphere's *centre*, so a layer is the space that centre
can occupy, and clearance is a comparison of centre heights.

Each lane holds surfaces, not a height per `s`, because ledges, walls and
overhangs are not heightfields. Gait planning reads only floors and ceilings.

**One layer: the body's.** The strip samples the open space the body is in,
followed out from it station by station: each station's sphere rises from the
floor before it, and the floor is cast down from under whatever it meets. So
under a slab the floor is the ground and the slab's underside the ceiling (a
tunnel, or the garden's cave lip refused for its headroom), not the slab's top
as a wall. Only a face that leans down is a ceiling; a wall corner grazed on
the way up bounds the layer but hangs over nothing. Where the sphere has no
room to start (a crack between two balls), the layer has closed and the
station is read from the top. What roofs the layer over, where something does,
is kept as the **upper floor**: a flight may pass over it (the bulge of a
ball over the trench at its foot) but the body does not ride or land on it.
Moving between layers (onto a slab from under it) is not planned. A
floor cast that meets a face too steep to ride, leaning across the path, is
cast again 2 cm off it: a sphere in the crease between the ground and a bank
beside the path touches the bank first, and if the cast moved off lands on
ridable ground no more than a centimetre lower, that is the floor. A bank the
sphere is held up on, or a slope along the path, stays steep.

Casts that start inside something pass out of it unseen, the game's sweeps
and the authored prisms alike; an authored `Slab` is a floor prism and a roof
prism over one footprint, which the start-height rule reads as a slab.

### Reading the section: `SectionQuery`

Transition models and the planner never read lanes directly. They ask a
`SectionQuery`:

| Query | Answers |
|---|---|
| `floor(s)` | the body floor under the path, with its normal and footing |
| `footholds(window)` | every standable foothold in a window along and across the path, with its normal, footing and margin to an edge, and, when the window names a foot's line, footholds on that line wherever the lanes either side hold the same surface; the planner judges which is best |
| `clears(motion)` | whether the capsule along a grounded stretch or flight arc stays on or above the body floor and under the ceiling |

The strip of lanes is the first implementation behind it. A real 3D height
patch around the body can replace it later (see *Beyond one path*) without
touching the models, the plan types or the tracker.

### Layers, and why the cast radius matters

The game's `PhysicsCaster` casts with `PhysicsWorld::cast_sphere`, which
sees every collider, so rubble and spawnables are in the section the same way
terrain is. There is no SDF requirement.

The cast radius acts as a filter. A sphere of radius `r` cast down onto a gap
of width `w < 2r` rests on both lips and sinks `r - sqrt(r² - (w/2)²)` into
it. So the section carries two floor layers, cast at two radii (the body
floor in the centre lane only, footholds in every lane):

| Layer | Cast radius | What it is | On the plank bridge (gaps 0.22–0.69 m) |
|---|---|---|---|
| `BodyFloor` | capsule radius (0.25 m) | where the capsule can rest; the body's path must clear it | sinks 2.5 cm at the 0.22 m gap, 10 cm at 0.4 m; falls through gaps over 0.5 m |
| `Foothold` | foot radius (about 0.05 m) | where a foot can land | every plank gap shows; footfalls land on planks |
| `Ceiling` | capsule radius, cast up | headroom; flight arcs stay under it | |

This is the "closing" filter from the design discussion, built from colliders
the game already has. Fine clutter disappears from the body's floor but stays
visible to the feet.

## The plan

```text
Plan
  phases: [Phase]        in time order from now to the horizon
  committed: usize       phases [0, committed) are not replanned

Phase
  start, duration                     seconds from now
  contacts: [Contact]                 who touches what during the phase; empty in flight
  motion: Motion                      its variant is the phase's kind:
    Grounded { from, to, speeds }       rides the body floor from s = from to to
    Flight { takeoff, gravity }         ballistic from the takeoff state

Contact
  limb: LeftFoot | RightFoot
  at: (s, d, z) on a surface, with its normal and footing
```

A contact's `d` is where that foot actually lands, not its lane: the planner
picks it within a window around the foot's natural offset, and the foot
placer lands on it. The lanes are where the ground is sampled, not where a
foot may go: on ground running unbroken between two lanes the query gives
footholds on each foot's own line, between them, so on open ground the
feet land under the hips rather than both drawn onto the nearest lane
(the centre one, for these hips). Where the lanes disagree, at a plank's
edge or on rubble, only the lanes' footholds are offered.

The plan says where the feet land; the legs say when. The game hands the
foot placer each foot's next footfall (`PlannedSteps`, through
`LeggedLocomotion::follow`), and a foot lifts on the gait's own clock, as it
does without a plan, and swings onto that footfall instead of where the
capture point would put it. The one time the plan owns is a flight's
landing: that footfall carries the time the body comes down, and a flight
the plan lands is the landing foot's swing. The placer, suspended for the
flight, swings that foot onto the landing footfall and plants it there as
the body comes down, and lifts the foot it pushed off with to trail onto its
own next footfall half a step later; the airborne pose draws the feet where
the placer has them instead of hanging them from the hips, and the landing
replants nothing. A planned footfall further behind the hip, along the way
the body travels, than the leg reaches when it lands is not taken: a plan
laid along a heading the body has not yet turned to (the stick swung back,
on the ground or in the air) lands the feet behind a body still travelling
the old way, and the legs then step, or hang, as with no plan. A footfall
ahead is always taken, however far the leg stretches to it, as the legs
land it in their own time (a reach limit both ways rejected ordinary
footfalls and doubled the miss). Feet that begin a flight hanging, or give
up a landing, hang until touchdown: the placer still holds them where they
left the ground, and flying them from there jumped them a metre and more.
A flight the plan stops landing part way (a long drop's plan going quiet)
flies on to the landing it last had: left where the placer last put them,
the feet hung in the world as the body fell past them, up to 2.5 m over
the pelvis, and dropping the flight on a plan quiet for a frame missed
landings it then came back to. A swing carries its own motion (`foot_placer::Swing`): each substep the
foot steers from where it is, at the speed it has, onto its landing over
the time left, along the cubic that is smoothstep for a target that holds
still. Whatever moves the landing or its time mid-swing (a hop planned
while it swings, a flight taking it up, a plan handing a leap's landing
from foot to foot every frame over a slope's crest) bends the path; it
never moves the foot, which the swing's old arc, recomputed from its
endpoints, did by up to 0.7 m. The lift is a bell over the swing's
progress and a retime keeps the progress, so no change of plan lifts the
foot again; the drawn foot is kept below the hip and within the leg's
stretch. A swing within its last frame keeps its landing: `Legs` has told
the plan the foot stands there. A swing with nowhere it can plant is held,
due again every substep, after a landing gathering under the hip. The
probe's floor is taken only within 0.6 m of where it struck: carried along
its plane to a footfall a leap away it aims the foot half a metre over a
slope's shelf, and a probe still aimed where a replanted foot was puts the
floor anywhere. A swing onto a planned footfall aims at the footfall's own
height, which the plan sensed. A landing that leaves a planted foot out of
the leg's reach replants at stance. These hold under any plan: a fuzz
(`foot_placer::plan_fuzz`) drives the legs with plans that churn, swap
feet, withdraw, and hand flights' landings between the feet or drop them,
some steered back in the air, and asserts that the feet stay below the hips, within the
leg's stretch, and move as smoothly as with no plan. The drawn feet blend whenever
they change between hanging and flown. An unplanned fall keeps the
old resume: a replant at stance. The plan in turn starts from the
feet the legs have down (`traversal::Legs`: the foot planted last holds the
body, the other lands next), so the two never argue about which foot is
where; a last plan that does not stand on those feet is dropped rather than
kept. Between steps at a run neither foot is down, and a foot whose swing
ends within the frame lands now: the plan, a frame on, has taken that
footfall as made, and the legs land it a frame later. Read as no foot
down, the plan was dropped and planned afresh with that foot's next
footfall a whole step on (a 0.45 m miss in the turning runs). Holding the plan's clock over the legs was tried first (milestone F)
and cut the stance in half at a run: the plan's cadence can only ever
approximate the legs', and the legs must not follow it.

**The footshake.** The legs land a footfall on their clock and the plan on
its own, a few frames apart either way, and the two agree on which
footfall a foot is on by name rather than by where the foot is. Every
footfall of a plan carries an id (`Contact::footfall`), kept while the
plan keeps the footfall and new for each the planner makes, numbered past
every id the last plan and the legs hold. The legs are handed each step
with its id; a swing remembers the step it steers onto, and a foot that
plants within 10 cm of it stands on it (`PlacerFoot::stands_on`). The
legs report the planted foot's footfall and the lead foot's aim, its id
and the seconds until it lands (`traversal::Legs`). A last plan is kept
while the legs stand on its stance footfall, or are ahead (they have
landed its next one), or behind (the foot they land next swings onto its
stance footfall and is down before the plan's next hop pushes off it). A
foot the plan has landed that the legs have not is handed the stance
footfall, not its next one, which turned it in mid-swing. A foot the legs
put down where no plan had it is named by the next plan that stands on
it. Matched by position and time instead, the two disagreed for every
frame of the lag, and each disagreement dropped the commitment and
planned afresh, moving the footfall under a swinging foot: up to 1.1 m on
the plank bridge, counted by the garden's summaries as a late move
(`moved`: a swinging foot's footfall moving over 5 cm in a frame). A fresh
plan still made weighs moving the footfall a foot swings onto at
`swinging_churn`, five times a tentative footfall's.

Three things seen in play look like the legs following the plan but happen
untracked, and on main, alike, so they are left to their own projects
(`docs/TODO.md`): the feet's pop at a landing and a planned hop's launch,
the crouch-walk's trailing foot, and a body resting on an edge with its
centre past it.

The plank bridge in this form, with no special cases:

```
 Support(R on ground) ─► Flight ─► Support(L on plank 1) ─► Flight ─► Support(R on plank 2) ...
        step up = a footfall higher than the last        hop = a footfall across a gap
 ... ─► Support(L on last plank) ─► Support(R on ground)   step down = a footfall lower
```

The feet alternate because footfalls alternate. Walking on flat ground
produces the same list with double-support phases and no flights, so ordinary
walking does not need a separate mode.

### Rises are flights (first version)

The capsule only rises on its contacts, and its contacts cannot lift it
straight up a lip. So in the first version **every rise is a short flight**: the
tracker sets the takeoff velocity the plan chose. Clearing a rise `h` with a
margin `c` takes `vz = sqrt(2g(h + c))`: about 1.0 m/s for a 3 cm lip with a
2 cm margin, and 1.9 m/s for 15 cm with 3 cm. That is less than the
uncontrolled hops the contacts produce today (1.2 m/s on a 3 cm lip), and the
legs can hide it as a push-off. If it reads badly, the alternative is a
vertical lift during support, with its reaction applied to the body under the
planted foot so rubble is still pushed on. Judge this in milestone E
(*Tracking through contacts*), by eye and from `traversal --summary`.

### The body model is a contract with the drive

The planner predicts what the physics will do with the body, and those
predictions are engine behaviour written down as numbers:

- a grounded phase has the capsule's bottom sphere ride the body floor the
  cast found, which holds only while its contacts behave like the cast
  sphere, and only where the floor falls no faster than gravity follows;
- up a slope the drive keeps the asked speed times cos²θ across the ground
  (`BodyDims::ground_speed`), and `max_ground_slope_degrees` claims how steep
  a floor it is ridden up;
- a flight is ballistic, with no air steering and no jump forgiveness
  bending the arc.

Nothing in the planner can check these, and if the drive changes, plans go
quietly wrong and show up as tracking error that looks like a planner bug. So
game-side bench tests measure the drive and assert it agrees with
`GaitParams`:

| Check | Measures |
|---|---|
| Slope | ramps of 30–58° at walk and run: whether the capsule gets up, and its speed across the ground against `ground_speed` |
| Takeoff | the takeoff velocity the body actually leaves with, against the one commanded, and the arc it flies against the ballistic one; a jump in the coyote window against one on flat ground |
| Acceleration | a run-up from standing on flat ground at walk and run: the mean to 90% of the gait's speed, against `max_acceleration`, which must not claim more nor lag it by over a quarter |
| Ride | the capsule's height over plank gaps and lips, against the body floor the cast reported, where a plan rides |

When someone tunes the drive, these fail and name the number to update. They
are `character_viewer contract` (`src/character_viewer/contract.rs`, on
`levels/drive_contract.level.ron`): the real game, so the Vulkan environment.

Measured on 2026-10-24:

- **Slope:** the cos²θ law holds to 0.1 m/s from 30° to 58°, and the capsule
  gets up 54°. The limit stays 40°: a slope met at speed is a collision the
  rides do not model (the capsule stalls a frame and pops up), and a voxel
  lip's face is a 45° bevel, which must stay a hop.
- **Takeoff:** a jump leaves at 7.00 m/s and flies ballistic at 9.81 m/s² to
  under a millimetre; one pressed 33 ms after running off an edge climbs the
  same.
- **Acceleration** (2026-10-25): 21.4 m/s² walking, 21.7 running, from
  standing to 90% of speed; `max_acceleration` is 20. It had been guessed at
  8, so a tracked body took 0.6 s to reach a walk the untracked one reached in
  0.15. The drive's frame-to-frame gain depends on the surface (44 m/s² on
  the garden's grass at first, 107 on the Stack Yard's stone); the mean on
  the contract's flat lane is what is claimed.
- **Ride:** fails, by collisions the plan avoids or does not model. Driven
  straight at a 25 or 40 cm lip (which a plan hops) the capsule stalls and
  pops 6–13 cm; carried up a gap's far brow after rolling off (which a plan
  does), it pops 5–8 cm. The first needs the body tracking the plan (E); the
  second is either modelled as a small hop or damped by the drive in E.

## The planner

The planner works in two stages: the body's path first, then the footfalls
along it.

```text
  section ──► BodySearch ──► BodyPath ──► FootfallSearch ──► footfalls
                                 └───────────────┬───────────────┘
                                             build_plan ──► Plan
```

**The body's path** is searched over stations every 5 cm along the path,
each with the body's speed (merged in 0.25 m/s bands). From a station the
body either rides to the next, pressing toward the asked-for speed, or it
flies to a landing up to a step ahead, landings every 10 cm, its takeoff
climb set by the landing and its arc required to clear the floor and
ceiling. The plan never slows the body below what the drive gives: the
player's stick owns speed, the plan owns hops. A landing, or a ride
meeting a steeper face, keeps only the speed along the floor it meets, and
what it loses is a cost. Leaving a brow while rising, the body carries its
climb until the drive cancels the rise (`BodyDims::rise_held`), so a short
face at a run is a pop the plan sees, not a ride; a push only adds to the
climb the body carries. Flights are
tried only where the ride within a step ahead breaks, gets rough or falls
away faster than the body skims, so open ground is walked at speed. The feet are asked only what they
can do near a station, worked out once per station: a station is ridden
through only with a foothold within half a step (so the capsule is never
carried over a void propped on the corners of stones either side), and a
flight needs a foothold just behind its takeoff and just past its landing,
and is short enough for one step to span them.

**The footfalls** are searched along that path. Each is a foot landing on a
foothold as the body passes a point of its path: every flight ends with one,
pushed off from a stance foot at or behind the takeoff, and on ridden
stretches the body is behind the landing foot by half the step, a quarter, or
none. The body's height is the path's, so a candidate costs no physics: only
reach, rise and drop, edge margins, footing, whether the body rests on ground
a foot is on, and churn. Where the feet cannot land a flight, that flight is
vetoed and the body's path searched again, up to four times, before the
planner refuses. So the capsule is held up by the ground under it at every
landing, a gap wider than the capsule is flown rather than straddled, and
each flight is one a foot pushes off and lands.

- **Search.** Both stages keep the cheapest way to each cell and expand the
  cheapest first, stopping at the first that reaches the goal: Dijkstra. The
  body search adds an A* bound, the slowdown a slow body must still pay to
  regain speed. Footfall cells are a foothold every few cm along `s` (3.5 %
  of the natural step, 4 to 10 cm), its lane, its foot, and the band the
  body landed in. A cold plan takes up to 4.5 ms on the fixtures at every
  speed (median 0.6 to 0.8 ms): speed bands of 0.25 m/s and braking spans
  from half a band to a full one, which replanning needs to reach the last
  plan's speeds, roughly doubled it.
- **Steps.** The plan's footfalls are the legs': its natural step at each
  speed is the step the animation's foot placer plans on level ground
  (`GaitTiming::level_step`, the stride the leg's reach caps), which the
  game hands the planner in `StrideLimits::natural`; the planner's default
  table is the player's, and a game test holds the two together. The
  player's legs step 0.73 m at a walk and a sprint alike, so a sprint is
  ten steps a second, and a hop is a running leap: its step is judged by
  the ground either side of its flight, not as an overlong stride. With
  each step comes the stance it is taken with, the rig's plant-ahead
  distance (`GaitTiming::plant_ahead`, `NaturalStep::plant_ahead`): a foot
  plants that far ahead of the hips and lifts that far behind them, 0.28 m
  of a 0.73 m step at 5 m/s, so a run's stance foot is off the ground
  before the next lands. The body lands a `landing_leads` fraction of that
  behind a footfall, and a footfall is within reach where it lands and
  where it lifts, the lower of the two binding, which on a slope is where it
  lifts. Judging both feet from a body midway between them, as before,
  capped every step at 0.62 m and had the plan's cadence 15 % ahead of the
  legs'.
- **Legs.** Everything else about the feet is the rig's too, passed the same
  way and held to the planner's defaults by the same test: the longest step
  without a flight (0.81 m, the leg's level reach over the sprint's stance
  share), each foot under its hip (0.12 m to its side), reaching half the leg's level reach
  to either side, and a foot in reach of the body when it is within the
  straightened leg (0.525 m) of a hip 0.425 m above the capsule's bottom.
  A flight, hop or roll-off, has a hop's feet: the last near the brow, the
  next at the landing, and no stride limit between them. These legs only
  just reach the ground under the capsule: up a slope past about 39° even
  the gait's shortest step (its slowest walk's) puts the foot behind out of
  reach of the body resting there (`GaitParams::steps_up`), so the planner
  refuses it and the stick and the drive carry the body up (`slope_38deg`,
  `slope_40deg`), until a sliding or scrambling motion takes such slopes
  over. Up to that limit it plans steps shorter than the legs' clock takes
  (8 cm at 3 m/s on `slope_38deg`): the plan's cadence on a steep slope is
  not yet the legs'. The limits had been a human
  runner's (2.5 m strides, a foot 0.35 m below the body), which hid that.
- **Horizon.** Counted in footfalls, a flight as one step: a plan carries
  the body three natural steps and then as far as the longest flight
  (`max_length`), and a step past its last landing. The footfalls it
  settles are chosen knowing every flight that could take off past them,
  so no takeoff they rule out goes unseen (with a horizon of three steps
  alone, a sprint settled its footfalls past the best takeoff for the 40 cm
  lip before the lip came into the plan), and a landing is known to lead
  on. Flights are searched as far as the strip was sampled (10 m by
  default), so a drop's landing is found wherever it can be seen. When no
  plan finishes, the planner refuses and returns the cheapest plan to the
  furthest point reached, which is where the way runs out.
- **In the air.** A plan made with the body in the air starts with the arc
  it is on, at its own climb, to where that arc meets the floor: nothing
  else is open to it, and nothing pushes it.
- **Pace.** Pressing or braking as above; running below the asked-for speed,
  on the ground or in the air, costs `slowdown`. That is how a sprint meets
  stepping stones.
- **Cost.** What can be measured is charged as the time it loses: running
  slower than the drive presses for there (`rules::time_lost`), and, for a
  plan that ends slow, the time still to lose regaining speed at the
  drive's acceleration, which is also the search's lower bound on what is
  left. A hop costs no time and a face or a bump little (the contract's
  `hop: cost` and `face: time` rows measure them), so the rest are
  preferences, priced in seconds: the jolt of meeting a face or landing on one,
  bumps ridden, flights and their climb, steps off the reachable step (the
  gait's, or the longest the legs reach on that ground if shorter, so a
  slope is not charged for the short steps it forces), feet off their
  natural offset, edges, and footing: `Static` is cheapest, then heavy
  sleeping bodies, then light awake ones. A flight costs 0.3 for being one,
  its takeoff and landing (a sweep over the garden kept 0.3: cheaper, and
  the plan skims plank gaps the legs then miss), 0.5 a second in the air,
  where the player cannot jump, and 0.1 per (m/s)² of climb, since a high
  hop reads as a deliberate jump and lands harder (without it a walk leapt
  crate to crate and stalled on one that moved). A typical 1.5 m/s hop costs
  0.68, a 3 m/s one 1.5. A leap off a brow into what is out of sight is
  charged the airtime it is known to fly, rising and falling back to its
  own height, so where it leaves does not change its price. Meeting a face
  costs 0.15 per (m/s)² of speed it takes, the jolt: a 38° face at a run
  about a high hop (1.4 with its time), though a clean face loses only
  0.13 s; at 0.08 a run rode into the rough field's rocks, which threw it.
  Plus a **churn cost**: diverging from what the previous plan settled,
  its next tentative footfall and the flights landing on its first two
  footfalls, and only where it chose them knowing what lay past them: a
  plan's last footfall, and a flight landing on it, are where its search
  stopped (the goal, or the end of the strip in sight), not choices. Past
  them each frame chooses again with what it now sees. That cost, together
  with commitment, keeps the plan from flickering between two nearly equal
  options without holding on to guesses.
- **Commitment.** The current phase and the next footfall are committed. Each
  frame the planner keeps the last plan's phases up to the committed footfall
  and plans only after it, from that footfall, toward a goal measured from
  the body now; once it lands, the commitment rolls on to the next. If the
  body strays from the committed path past a tolerance (the rubble shifted,
  the body was hit), or the kept phases no longer clear or stand on a
  foothold, commitment is dropped and the planner replans from the actual
  state; if no plan goes on from the kept phases but a fresh one does, it is
  dropped too. Each such event is counted, because they are where the edge
  cases will be. The next tentative footfall is kept too while it holds,
  if the last plan settled it. The body search charges churn for leaving
  the flights the last plan settled and offers its rides again as they
  were, so a replan keeps to it unless something is better.

### Why body moves are not the old move state machine

A move state machine has states with entry and exit rules and a handover for
every pair, so cases grow with the square of the move count. Here a body move
only *proposes* edges out of a station, and the searches validate every
proposal by the same rules. Adding a move adds edges and touches nothing else.
What the drive and the legs consume is `Phase` data, of two kinds: Support
and Flight. Moves that are not walking or hopping (a jump, a grab, a climb) are
locomotion states the planner stays out of (`docs/LOCOMOTION_STATES.md`).

## Beyond one path

The strip only sees ground near the arc the stick points along. In a boulder
field the best way through may lie off it. Two extensions, in order of cost,
both behind `SectionQuery`:

1. **A fan of arcs.** Sample a strip along a few headings either side of the
   stick's, plan each, and take the cheapest, with a cost for straying from
   the stick so the player stays in charge (the dynamic window approach from
   robotics). Each arc is still a strip, so nothing else changes. Its cost is
   the number of arcs times one strip's.
2. **Footstep planning on a local height patch.** A 3D patch around the body
   behind the same queries, and a search ordered by distance to a goal
   instead of progress along one path. Only if the fan of arcs fails on the
   rubble and scree lanes.

## Not extended: ledges, hanging, climbing

The planner is not extended past walking and hopping: planning more moves
as phases, each with its own executor, would cost more to maintain than it
returns. Ledge grabs, hanging, pull-ups and climbing are locomotion states
with the planner off; see `docs/LOCOMOTION_STATES.md`.

## Code layout

| Where | What |
|---|---|
| `crates/traversal_planner/` | pure: `Section`, `SectionQuery` and `StripQuery` over a sampled strip, `Plan`, `Planner` (`BodySearch`, `FootfallSearch`), `GaitParams`, the scene format, `sample_section` over a `SphereCaster`, `ShapeCaster` for authored scenes, the SVG writer, `section_svg` bin |
| `src/traversal/` | game side: `PathPredictor`, `PhysicsCaster` (a `SphereCaster` over `PhysicsWorld`), `PlanTracker`, `PlanOverlay`, the capture key; `section_capture` bin |

The repo is a Cargo workspace with the game crate at its root and the planner
under `crates/`. `default-members` is the game, so `cargo test` and `cargo run`
work as before. The planner crate depends on `serde`/`ron`, `smallvec` and
`rustc-hash` only, so it cannot reach the game: the boundary that keeps it
pure is the crate's, and its tests build and run without the engine.

`GaitParams` (stride, rise and drop limits, foot and body radii, flight
margins) lives in the planner crate. The game fills it in from the rig's leg
dimensions and the capsule.

## Tooling

### Scene files

One RON format, shared by every tool and test:

```text
TraversalScene(
  section: Authored([...shapes...]) | Captured(Section),
  body: (s, z, speed, climb),        the body is on the path, so it has no d
  lead: LeftFoot | RightFoot,        the foot that lands next
  intent: (speed, stop),
  params: GaitParams(...),
)
```

Authored scenes are side-view shapes (boxes, ramps, polylines), each `Static`
or `Loose(mass)`, with a sideways extent `d_min..d_max` that defaults to the
whole strip. A plain gait scene is then a pure side view, and a stone off the
line is one shape with a narrow extent. Shapes can be tilted across the path
too. Captured scenes carry the game's section verbatim.

One procedure samples every section, `sample_section`, over a `SphereCaster`:
sphere casts down and up at a point of the path. The game implements it over
`PhysicsWorld::cast_sphere`; authored scenes implement it with `ShapeCaster`,
exact casts against their shapes (a sphere coming down first touches a
shape's top face, one of its top edges or a corner). So where surfaces split,
which samples count as footholds and what the layers mean are written once.
The caster tells bodies apart by an opaque `BodyKey` that only the sampler
compares, to split surfaces at a change of body.

### `section_svg` (CLI)

`cargo run -p traversal_planner --bin section_svg -- scene.ron -o out.svg`
draws two views sharing the `s` axis: the **side view** on top, and a
**top-down strip** under it, showing each lane's footholds and where each
footfall lands sideways. Without the top view, the tools would hide exactly
the lateral cases a side view gets wrong. Both views draw:
- the layers (body floor, footholds, ceiling), coloured by footing;
- footfalls, coloured left or right, solid when committed and faded when
  tentative;
- flight arcs, and the body path with the capsule drawn at each phase
  boundary;
- the cost of each phase.

This is how I read a scene.

### Fixtures and properties

`crates/traversal_planner/fixtures/*.ron`: every bad case found in
play or captured in the game goes here. Fixtures come from three sources:
authored scenes that each isolate one rule (a lip, a gap, a vertical wall);
captures of settled physics scenes (rubble piles, tumbled crates, crater
rims), which no hand-authored shape describes honestly; and the fuzzer.

Each fixture states its expected outcome, because refusing is a valid answer
and the one that keeps the planner from inventing a way through:
- **crosses**: a plan reaches the end of the strip;
- **refuses**: no plan on this heading (a wall taller than a hop, a pile
  only a ledge grab would climb);
- **avoids moving footing**: no footfall on loose footing that is not at
  rest.

The tests check properties of every fixture's plan rather than exact output
(`properties::check_plan`, `check_replan`), each a `Rule`:
- phases follow on in time and along the path, and each clears the floor and
  ceiling, riding only where the drive can push the body;
- feet alternate; every footfall is on a foothold the section has, on
  footing that is not moving, within reach of the body as it lands;
- steps, rises, drops and sideways reach are within `GaitParams`, and every
  takeoff climbs no faster than they allow;
- replanning a frame later on the same ground, the body having tracked the
  plan, keeps the commitment, and leaves the next tentative footfall within
  a cell and a half of the lattice unless it is the last (the goal slides
  ahead with the body, so the last one moves).

`tests/properties.rs` breaks a good plan on purpose for each rule, to prove
the check can fail. The tests run in milliseconds and always run.

### Fuzzer

`fuzz::generate(seed)` lays a random course (runs, lips and drops, gaps over
voids, plank bridges laid askew, stepping stones on and off the line, tilted
and loose stones, ramps, overhangs) and plans it at a crouch, a walk or a
sprint; `fuzz::try_scene` checks the plan, or a refusal's partial plan,
against every rule, and a plan's replan a frame later. `plan_fuzz` sweeps
seeds and, with `--write`, shrinks each failing scene to the fewest shapes
that still break its rule (`fuzz::shrink`) and writes it as a fixture. A
300-seed sweep runs with the tests.

Its first sweep found 58 failures in 500 seeds: plans starting with a flight
dropped the planted foot; a refusal kept a flight its feet never landed;
the body search stations moved with the body, so a frame later its takeoffs
were not the last plan's; it let a foot push off from before the last
touchdown; the commitment held only the footfall, not the phases before it;
the goal of a replan from the committed footfall ran a step further than
the last plan's. Seeds 0 to 1999 are clean; ten in 10,000 still move the
next tentative footfall on replanning (2754, 3489, 4621, 5944, 7719, 8097,
8237, 9191, 9291 by 0.18 to 1.4 m; 7264 drops its commitment), each the
search from the committed footfall missing the last plan's continuation in
its own discretisation. A plan takes 1.2 ms median, 8 ms at the 99th
percentile and up to 31 ms on fuzzed courses.

### Offline capture

`section_capture` (a game binary, headless, no GPU): loads a level or spawns a
pile, lets physics settle it, samples the strip from a given start and heading
through `PhysicsCaster`, and writes a `Captured` scene. It is how real rubble
reaches the fixtures before the game runs the planner, and the first measure
of what the game's casts cost. It reuses `level_check`'s `RestTrial` to spawn
and settle, and blasts as a grenade shoves bodies (a radial impulse). The layer is
followed from the floor under the start, found from above, so a capture
started under an overhang reads the overhang's top. The command that made a
capture heads its file, and a capture is deterministic: the same command
writes the same file.

### In game

Built so far (`src/traversal/`): `TraversalPlanSystem` plans the player's way
every frame, drawn on **F6**; **F7** writes the strip, body and plan to
`scratch/captures/`; the costs go to `DebugLog` under `Traversal/Plan`. The
garden check (`cargo test --release --lib --features traversal_scenarios
garden_station`) captures every station at walk and sprint and plans each
against its authored twin, first comparing the two strips (the body floor's
median height, and where only one has a ceiling) so a station read as other
ground fails even when both refuse; the lips are authored as the voxels mesh
them, sloped across the voxel at the face. The Stack Yard check
(`stack_yard_places`) plans places that planned wrongly in play to the
outcome they should have: the ball pit from its rim, and running along the
pyramid's first step pressed into the next step's face. A loose body is
resting footing when it is asleep or its surface moves no faster than
5 cm/s: whatever the player stands on is awake, so a floor of dynamic bodies
(the temple's stylobate, the pyramid's blocks) read as moving under the body
and every plan on it was refused, which captures of the settled level never
showed. The strip's casts
pass through dynamic bodies whose surface moves faster than 2 m/s: a blast's
pieces in flight are not ground, and are seen again where they slow, which
keeps a blast's first second to 0.4 ms of sampling a frame (median; 1.6 ms
at the 99th percentile) instead of 4.

Carriers (`src/traversal/carriers.rs`: moving platforms, which cruise at 2
to 2.5 m/s) are a stopgap until the planner knows footing that moves. Read
as loose bodies, a deck straddled both thresholds as it tipped and turned: over 2 m/s the
casts passed through it and the plan fell through the deck; under, it was
footing in motion and the plan refused. The strip now always sees a carrier and reads it as
fixed ground where it stands this frame. While the player rides one, from
standing on it until standing on other ground, the air between included,
there is no plan: the body's speed is the world's, which made a body
standing on a deck plan a walk along the patrol and the legs follow its
footfalls, splaying them (riding the Traversal Garden's ferry, the feet
0.25 m out from the body on average against 0.12 with the plan aside,
`character_viewer ride_ferry --track`). Untracked, the drive and the legs
step in the support's frame. Planning on a carrier, boarding one and leaving
it is a milestone of its own: footing whose place is known over time, its
footfalls predicted where the deck will be.

**Tracking (E, first version), on by default, F8 toggles it.** The plan system runs
between the player's input and `CharacterControlSystem`: it plans from the
player's intent, then rewrites that intent (`tracking.rs`). On the ground
the plan only holds the body back: where it is speeding up the stick sets
the pace and the drive accelerates as it does untracked (its first frames
on firm ground outrun the 20 m/s² mean the plan assumes, and asking the
plan's speed made every start, turn and landing feel sluggish); where it
holds below the asked speed or slows, the intent asks its speed one frame
ahead, more up or down a grade by the drive contract's `ground_speed` so
the drive delivers it. Which of the two (or a refusal's braking) sets the
pace shows on screen as `Traversal track`. The intent is steered by the
stick, the path following within 2° (see *The section*): steering the
intent along the path, which was relaid only every 10°, turned the body in
steps. Along a wall it slides
on, the intent follows the path. A ride changes speed at constant
acceleration (`v² = v₀² + 2ad`, not `a·d/v` a station at a time, which from
standing jumped to 1.6 m/s in 5 cm), and consecutive rides are one phase only
where their accelerations agree: merged regardless, a press to speed and the
run held there became one slow ramp, and the body tracked the ramp. A hop starting in the frame becomes a
`PlannedTakeoff` on the intent, which the locomotion state machine launches
as a leap: the plan's climb, its air speed capped at the plan's, no cutoff,
and the stick steering as in any flight, so a hop stays responsive; steered
off the plan's arc, the body is replanned from where it goes. In flight the
stick sets the pace too: a plan's flight holds its takeoff speed, and pacing
the flight at it kept a jump from standing at 0.3 m/s. A roll-off
asks for nothing (the body runs off the edge). A refused plan knows no way
through, so it leaves the body to the drive: the stick sets the pace and
nothing takes off, which is never worse than no plan (on the garden's
scree, taking off on a refusal's hops had stalled the body on the rocks).
A jump the player presses still jumps. `character_viewer traversal --summary
--track` runs the garden stations with it. A hop is taken off on the frame
nearest its planned start, half a frame early at most.

**Edges are jumped off.** A run at an edge leaves it at speed: a footfall
may land up to `max_drop` (8 m) below the last, and a flight off an edge may
land past a stride by as far as the body carries falling to its landing:
a jump, its feet a push-off and a landing within a foot's slack of where the
body comes down, not a stride. Level hops reach as far as the drive's climb
carries the body (`GaitParams::longest_hop`). Over the Stack Yard's voussoir arch (`stack_yard_voussoir_arch`) the
plan had hopped the top stone and then braked to 2 m/s to drop off the far
side within a stride; walking, it now runs up to the crest and leaves it
all but unpushed.

**A flight clears what it does not land on.** A flight keeps
`clearance` (3 cm) over the floor, except near its ends, where an arc meets
the floor: there it may run along the surface it takes off from or lands on
(normals within 5°), and after its takeoff over floor falling away beneath
both that surface and the line it leaves along, as off a rounded brow. Any
other face near its ends is struck, not landed on: a plank's bevel, a lip's
rounded top corner met level with the top. Letting the arc touch the floor
anywhere near its ends had planned hops that struck those corners and
popped the body up, and the garden's plank and lip runs lose nothing now.

**A landing out of sight is leapt into.** Where the floor is seen to drop
away within a stride of a brow the body can ride no further from, and an
arc pushed off it with `leap_climb` (1.5 m/s) meets no floor as far as the
strip was sampled, the body leaps: the plan ends in the air, its feet
carrying the body to the push-off and none landing after it, as a runner
jumps off an edge without seeing the landing. Its climb is set, not
chosen, so it costs only a flight and is not traded against a hop's. Once
the ground below comes into the search, the leap is a hop that lands. Only
able to land on what it saw, the plan had taken off 1.9 m before the
Traversal Garden's cave roof at a run, on a leap whose landing was the end
of the strip (`garden_roof_edge`). Where nothing is sampled past the brow,
or the floor ahead rises (the voussoir arch's corners), there is nothing
seen to leap off.

Against the untracked runs on 2026-10-25 (late morning):
- **better:** the 25 and 40 cm lips, crossed at speed instead of stalled
  (25 cm at a walk: lowest speed 4.55 m/s against 1.07; 40 cm at a run: 5.53
  against 0.14), the rough lane (1.70 s against 1.87 running), the wide
  planks at a run;
- **level:** the 3, 8 and 15 cm lips, both slopes (the 50° one barged up
  once its refusal has stopped the body);
- **worse:** the narrow planks (2.53 s against 2.20 walking, 1.47 against
  1.30 running) and the wide ones at a walk (2.40 against 2.20), whose
  landings bounce off plank edges by a frame's luck; the 25 cm lip at a run
  lands on its edge 2 cm low after a takeoff a 30 Hz frame late, and loses
  speed; and the scree, climbed but too slowly to reach its shelf in the
  run. The scree's rocks stand 0.6 m above its slope: a planned hop is
  capped at 3 m/s of climb (0.45 m), and even uncapped the feet find no
  landing on their tops. Ledge and cave are refused either way.

The drive contract's ride rows run tracked: none sinks below the floor, four
of eight hold (six at 8 m/s²), and the rest ride 3-9 cm high. The 40 cm lip at a
walk shows why: the plan hops the bevel's 45° face and lands on its 30°
part, which is ridable; landing on a rising floor at speed turns the body's
forward speed into climb, and at the lip's top it is thrown. A cap on the
body's rise while it rides did not help (the throw comes from the landing,
in flight) and was taken out. What would: landings only where the floor
rises no faster than the arc arrives, or the landing's kick modelled.

Riding a slope at the limit needed three planner fixes: a ride is charged
for running slower than the slope lets it (`ground_speed`), not the asked
speed; its `vertical_travel` is the height gained and lost beyond its net
climb, so a steady slope is not priced as bumps; and a floor may read a
degree steeper than `max_ground_slope_degrees` (`BodyDims::steepest_ride`),
since a mesh facets a slope built at the limit either side of it. The
strip's casts reach above where the path was laid by the steepest rise the
body rides along the strip's length (`strip_reach`), so a long slope is not
cut off where it passes 3 m. Planned:

- **`PlanOverlay`**: the plan through `DebugOverlays`. Footfalls as discs
  (left and right coloured, committed solid, tentative faded), flight arcs,
  the body path fading out towards the horizon, section samples coloured by
  footing, the body's actual trail beside its planned one, and a flash when
  the next footfall changes.
- **Capture key**: writes the current section, body state, intent, params and
  plan to a scene file that `section_svg` and the fixtures load.
- **`DebugLog`** under `Traversal/`: planner time, section time, churn,
  divergence events.
- **`character_viewer`**: the overlay drawn in its filmstrips, and churn per
  second, tracking error and divergence counts added to
  `traversal --summary`, judged against `docs/traversal/BASELINE.md`.

## Milestones

Each one ends when its exit check passes. No milestone moves the player
until E.

| | Milestone | Exit check |
|---|---|---|
| A | Workspace, planner crate: `Section` strip and `SectionQuery`, `Plan`, the planner, scene format, `section_svg` with both views; `PhysicsCaster` and `section_capture` | authored: the Traversal Garden plank bridge (yawed, loose planks), lips of 3–40 cm, stepping stones in line and zigzagged off it, a stone tilted across the path, a vertical wall (refused); captured: a crate pile in a crater and two or three other settled rubble scenes, each with its expected outcome; all read as SVGs |
| B | Property tests, a breaking fixture for each, fuzzer | fuzz sweep clean; every property shown able to fail |
| C | Web tool (dropped 2026-10-26: the plan drawn in game proved the better tool) | published; you can author and tune the bridge in it |
| D | `PathPredictor` in game, `PlanOverlay`, capture key, the drive contract checks; plan drawn but not acted on | captures of every Traversal Garden station plan the same as their authored versions; the drive contract checks pass, with `GaitParams` set from what they measured; section (five lanes) + plan under 0.5 ms per frame |
| E | `PlanTracker`: drive and takeoffs through contacts (closed 2026-10-26, against the bare drive; four stations open, in `BASELINE.md`) | `traversal --summary` beats the baseline on every station, churn and divergence measured, rubble lane crossed |
| F | `FootPlacer` lands on the plan's footfalls, lifting on its own clock; a planned flight is the landing foot's swing; the plan starts from the legs' stance (exit check met 2026-10-26, in `BASELINE.md`) | the legs alternate plank to plank; clipping and unplanted legs measured as in the baseline |

## Open questions

- **Planning cost:** settled (2026-10-30) at the cost measured in the
  game's world by `traversal_perf` over the garden: a plan's median is
  0.3–0.5 ms on ordinary ground, its worst about 2–3 ms (lips, planks,
  rough ground, crates), 5.5 ms on the scree, which the game will rarely
  ask for; the strip adds 0.1 ms, 0.6 ms at worst. D's 0.5 ms was given up
  as a worst case. Nearly all of it is the footfall search's breadth (some
  1,200 expansions on the scree); fewer states, such as one body lead per
  foothold, is what would cut it further, at the price of re-baselining
  every plan.
- **Coupling of the two stages:** the body search asks the feet only coarse
  questions (a foothold near a station, behind a takeoff, past a landing).
  Where those pass but no footfalls follow, a veto and a fresh body search
  fix it, up to four times. If captures show refusals the joint search would
  have crossed, the body search needs a finer question, not more vetoes.
- **Replanning stability:** 0.1 % of fuzzed scenes still move the next
  tentative footfall on a replan a frame later (seeds under *Fuzzer*). The
  last plan's tail is not kept whole: it was tried, and it held on to
  choices made at the edge of sight, such as a leap before an edge whose
  landing was not yet seen, long after better ones came into view. Searching
  the tail on the last search's own lattice is still open.
- **Past a refusal:** settled: edges are jumped off up to `max_drop`, with
  enthusiasm, a drop whose landing is out of sight is leapt into, and a
  refusal that is left (a wall, a drop with nothing sampled below it)
  leaves the pace to the stick.
- **Small gaps:** settled by roll-offs. A gap the body's arc crosses at its
  speed is rolled over flat (or hopped, at the same cost), never dipped into;
  a wide one is hopped.
- **Coyote time:** a planned hop (0.15–0.25 s) falls inside the 0.16 s the
  locomotion state machine still treats as ground (`ground_forgiveness_window`
  then `CoyoteTime`), whose `clamp_up` cancels the takeoff climb, whose
  steering bends the arc, and whose jump press fires a full jump mid-hop. In
  E, a planned takeoff becomes a locomotion state of its own (no clamp, no
  steering, a jump press replaces the hop with a jump); coyote time stays for
  walk-offs the plan did not plan. D's drive checks add a commanded takeoff
  that must leave unclamped.

- **Rises:** flights (first version) or a lift with its reaction applied
  through the planted foot. Settled in E.
- **Path prediction:** how far to trust the stick's curvature for the section
  frame on a hard turn, and what the plan does when the player stops or
  reverses (a plan that ends in a stop, and a churn cost waived on reversal).
  Walls are settled: a path is laid along a wall the heading meets within 30°
  of parallel, as the capsule slides on it, its line kept 3 cm clear
  (`WallSlide`). The walls are what the capsule's top sphere meets, so lips
  lower than its middle stay hops; the bottom sphere keeps the line 3 cm off
  what it stands against and follows a face it meets within 5° of parallel
  (a bank's foot converging on the path, not a lip to hop). A wall is any
  face steeper than the body rides. Walls are looked for 2 m ahead, about a
  sprint's stopping distance: over the strip's 10 m, a column far off turned
  the path along its tangent. The 30° is to tune in play.
- **Lateral footing:** the strip's width and lane count (five over ±0.3 m to
  start) and how far the foot placer may move a foot from its planned
  footfall are judged on the scree and rubble lanes in D and F. If the best
  way through lies off the stick's arc, the fan of arcs comes next (*Beyond
  one path*).
- **Section frame per phase**, for shimmying along a ledge (climbing only).
