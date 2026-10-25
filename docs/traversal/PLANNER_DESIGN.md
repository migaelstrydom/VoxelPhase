# Traversal planner design

How the player crosses broken ground: lips, plank bridges, stepping stones,
rubble that gives way. Later the same planner handles ledge grabs, hanging and
climbing. This supersedes `docs/TRAVERSAL_PLAN.md`, which was abandoned on
2026-10-19.

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
                                                               │      │    later grab, pull up, climb)
                                                               │      ▼
                              ┌─────────────────────────────── │ Plan: [Phase]  (committed prefix
                              │                     │          │        + tentative rest)
                              ▼                     ▼          │
                        PlanTracker            FootPlacer      │ also consumed by:
                        (traction drive        (footfalls and  │   section_svg (CLI), web tool (wasm),
                         command, takeoffs)     their timing)  │   fixture tests, fuzzer
                              │                                │
                              ▼                                │
                        PlanOverlay (DebugOverlays), capture key ──► scene file (RON) ──► web tool / fixtures
```

The planner crate does not depend on the game. The game depends on it.

## The section: a strip of ground ahead

A `SectionFrame` maps the world to path coordinates: `s` is the distance along
the predicted path, `d` the signed offset across it (positive to the left),
`z` the height. In the first version the path is the arc the stick is
steering along, from the body's position and velocity, so turns sample the
ground the body will actually cross.

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
overhangs are not heightfields. Gait planning reads only floors and ceilings,
but the type does not have to change when climbing arrives.

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
Moving between layers (onto a slab from under it) is climbing's, later. A
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
| `footholds(window)` | every standable foothold in a window along and across the path, with its normal, footing and margin to an edge; the planner judges which is best |
| `clears(motion)` | whether the capsule along a grounded stretch or flight arc stays on or above the body floor and under the ceiling |

Climbing adds `edges(s range)`, the grabbable edges, with the wall pass.

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

Later, climbing adds a **wall pass** (casts forward at several heights) and
**edges**: convex corners where a floor meets the top of a wall, which can be
grabbed. Both are new sampler passes that add surfaces and edges to the same
`Section`.

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
                                        (later: Hang, Climb)

Contact
  limb: LeftFoot | RightFoot          (later: LeftHand | RightHand)
  at: (s, d, z) on a surface, with its normal and footing
```

A contact's `d` is where that foot actually lands, not its lane: the planner
picks it within a window around the foot's natural offset, and the foot
placer can still refine it on the real ground.

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
each with the body's speed (merged in 0.5 m/s bands). From a station the
body either rides to the next, pressing toward the asked-for speed, or brakes
for as many stations as it takes to slow by a band; or it flies to a landing
up to a step ahead, landings every 10 cm, its takeoff climb set by the
landing and its arc required to clear the floor and ceiling. Braking and
flights are tried only where the ride within a step ahead breaks or gets
rough, so open ground is walked at speed. The feet are asked only what they
can do near a station, worked out once per station: a station is ridden
through only with a foothold within half a step (so the capsule is never
carried over a void propped on the corners of stones either side), and a
flight needs a foothold just behind its takeoff and just past its landing,
and is short enough for one step to span them. New ways for the body to move
(grabbing a ledge, pulling up, climbing) are new kinds of edge out of a
station.

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
- **Horizon.** A plan carries the body at least three natural steps, and at
  least as far as the asked-for speed would in 0.6 s: the legs need the next
  two or three footfalls, and the body time to see an obstacle and set up a
  hop or stop. A plan that only slows to a stop before an obstacle covers
  less, so it is a refusal. The strip samples one step past that: 10 m by
  default, a sprint's reach (`PlannerSettings::reach`). When no plan
  finishes, the planner refuses and returns the cheapest plan to the furthest
  point reached, which is where the way runs out.
- **Pace.** Pressing or braking as above; running below the asked-for speed,
  on the ground or in the air, costs `slowdown`. That is how a sprint meets
  stepping stones.
- **Cost.** Time to the horizon, deviation from the speed the stick asks
  for, uneven stride lengths, feet far from their natural offset (a
  cross-step costs more), flights (they cost a little), and footing:
  `Static` is cheapest, then heavy sleeping bodies, then light awake ones.
  Plus a **churn cost**: diverging from the previous plan's tentative
  footfalls. That cost, together with commitment, keeps the plan from
  flickering between two nearly equal options.
- **Commitment.** The current phase and the next footfall are committed. Each
  frame the planner keeps the last plan's phases up to the committed footfall
  and plans only after it, from that footfall, toward a goal measured from
  the body now; once it lands, the commitment rolls on to the next. If the
  body strays from the committed path past a tolerance (the rubble shifted,
  the body was hit), or the kept phases no longer clear or stand on a
  foothold, commitment is dropped and the planner replans from the actual
  state; if no plan goes on from the kept phases but a fresh one does, it is
  dropped too. Each such event is counted, because they are where the edge
  cases will be. The body search charges churn for leaving the last plan's
  body path and offers its rides again as they were, so a replan keeps to
  it unless something is better.

### Why body moves are not the old move state machine

A move state machine has states with entry and exit rules and a handover for
every pair, so cases grow with the square of the move count. Here a body move
only *proposes* edges out of a station, and the searches validate every
proposal by the same rules. Adding a move adds edges and touches nothing else.
What the drive and the legs consume is `Phase` data, and its set of kinds
grows slowly: Support, Flight, then Hang and Climb.

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

## Extension: ledges, hanging, climbing

What each future ability adds. Each is an addition; nothing already
built changes shape.

| Ability | Section | Plan | Model | Executor |
|---|---|---|---|---|
| Ledge grab | wall pass, `Edge`s | hand contacts | `Grab`: a flight ending in hands on an edge | `Hang` |
| Hang, drop | | `Hang` phase | `Drop`: hang to flight | `Hang` holds a hand constraint to the edge's body (the joint arena), so hanging from rubble pulls on it |
| Pull up / mantle | | | `PullUp`: hang to support on the edge's floor | `Hang` to `Support` |
| Climb | wall surfaces tagged climbable | `Climb` phase, mixed hand and foot contacts | `ClimbReach` | `Climb` |

The tracker dispatches on phase kind through a `PhaseExecutor` per kind,
registered like the models.

One limit to know now: shimmying along a ledge, turning a corner while
hanging, and cross-stepping along a beam are sideways moves, so they don't fit
a section taken along the walking path. The fix is a section frame chosen per phase (along the ledge while
hanging). That is the main open question for climbing; the first version
takes its frame from the predicted path only.

## Code layout

| Where | What |
|---|---|
| `crates/traversal_planner/` | pure: `Section`, `SectionQuery` and `StripQuery` over a sampled strip, `Plan`, `Planner` (`BodySearch`, `FootfallSearch`), `GaitParams`, the scene format, `sample_section` over a `SphereCaster`, `ShapeCaster` for authored scenes, the SVG writer, `section_svg` bin |
| `crates/traversal_planner_web/` | the planner compiled to wasm behind a JSON boundary (`api`, `ffi`), the time scrub, and the bench's HTML page |
| `src/traversal/` | game side: `PathPredictor`, `PhysicsCaster` (a `SphereCaster` over `PhysicsWorld`), `PlanTracker`, `PlanOverlay`, the capture key; `section_capture` bin |

The repo becomes a Cargo workspace with the game crate at its root and the two
crates under `crates/`. `default-members` stays the game, so `cargo test` and
`cargo run` keep working, and CLAUDE.md gains the planner's test and tool
commands. The planner crate depends on `nalgebra` and `serde`/`ron` only, all
of which build for `wasm32-unknown-unknown`.

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

### The web tool (wasm)

The Traversal Planner Bench, a private artifact: the real planner in a
browser page.
- the same side view and top-down strip as `section_svg`, from the same code;
- drag obstacles in either view: move them, drag their ends or top to resize
  them in the side view, their sides to widen them in the strip; click one to
  edit it (form, extent, top, tilt, yaw, footing, mass), or add blocks,
  stones, planks, ramps and roofs;
- sliders for every `GaitParams` and `PlannerSettings` number, speed presets
  for crouch, walk and sprint, and the body's place and lead foot;
- replans live on every change, with the time it took;
- **time scrub**: runs the body along its plans, replanning every frame
  against the last plan, and logs every commitment dropped and every next
  footfall that moved;
- **perturb**: while running, loose footing drops and slides under each foot
  that lands on it, which the next plans must deal with;
- import and export the scene file, captures included (a captured scene's
  ground is the game's own and is not edited).

`crates/traversal_planner_web` compiles the planner to `wasm32` with no
bindings crate: the page writes a JSON request into memory from `alloc`,
calls `call(op, ...)`, and reads the JSON answer (`api::Op` lists the
operations). The page is `crates/traversal_planner_web/page/index.html`; it
loads `planner.wasm` published beside it.

### Fixtures and properties

`crates/traversal_planner/fixtures/*.ron`: every bad case found in the web
tool or captured in the game goes here. Fixtures come from three sources:
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

**Tracking (E, first version), F8, off by default.** The plan system runs
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
stick: the path is relaid only once the heading has turned
10°, and steering along it turned the body in steps. Along a wall it slides
on, the intent follows the path. A ride changes speed at constant
acceleration (`v² = v₀² + 2ad`, not `a·d/v` a station at a time, which from
standing jumped to 1.6 m/s in 5 cm), and consecutive rides are one phase only
where their accelerations agree: merged regardless, a press to speed and the
run held there became one slow ramp, and the body tracked the ramp. A hop starting in the frame becomes a
`PlannedTakeoff` on the intent, which the locomotion state machine launches
locked: the plan's climb, its planar velocity held for the flight, no
steering, no cutoff. A roll-off asks for nothing (the body runs off the edge).
A refused plan is followed to where it ran out, braked to stop there. Then
it lets go: with under 5 cm of plan left, or the body down to 0.5 m/s,
the stick drives as untracked, so a strip read wrongly never holds the body
(it had, beside a bank and against the corner where a ramp meets its
plateau), and a held push carries on over the edge. A
jump the player presses still jumps. `character_viewer traversal --summary
--track` runs the garden stations with it. A hop is taken off on the frame
nearest its planned start, half a frame early at most.

**Edges are jumped off.** A run at an edge leaves it at speed: a footfall
may land up to `max_drop` (8 m) below the last, and a flight off an edge may
land past a stride by as far as the body carries falling to its landing
(`rules::jumped`): a jump, its feet a push-off and a landing within a foot's
slack of where the body comes down, not a stride. Level hops stay within a
stride. Over the Stack Yard's voussoir arch (`stack_yard_voussoir_arch`) the
plan had hopped the top stone and then braked to 2 m/s to drop off the far
side within a stride; it now runs the crest and jumps off at 5 or 7 m/s.

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
  plan to a scene file that the web tool and fixtures load.
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
| C | Web tool | published; you can author and tune the bridge in it |
| D | `PathPredictor` in game, `PlanOverlay`, capture key, the drive contract checks; plan drawn but not acted on | captures of every Traversal Garden station plan the same as their authored versions; the drive contract checks pass, with `GaitParams` set from what they measured; section (five lanes) + plan under 0.5 ms per frame |
| E | `PlanTracker`: drive and takeoffs through contacts | `traversal --summary` beats the baseline on every station, churn and divergence measured, rubble lane crossed |
| F | `FootPlacer` reads footfalls and their timing from the plan | the legs alternate plank to plank; clipping and unplanted legs measured as in the baseline |

## Open questions

- **Planning cost:** a cold plan takes up to 4.5 ms on the fixtures, and
  1.2 ms median on fuzzed courses, against D's 0.5 ms for section and plan
  together. In game, last frame's plan bounds
  the search (warm start), which is still to be built and measured there.
- **Coupling of the two stages:** the body search asks the feet only coarse
  questions (a foothold near a station, behind a takeoff, past a landing).
  Where those pass but no footfalls follow, a veto and a fresh body search
  fix it, up to four times. If captures show refusals the joint search would
  have crossed, the body search needs a finer question, not more vetoes.
- **Replanning stability:** 0.1 % of fuzzed scenes still move the next
  tentative footfall on a replan a frame later (seeds under *Fuzzer*). The
  candidates: keep the last plan's tail whole while it still holds and the
  request has not changed, planning only past its end; or search the tail on
  the last search's own lattice.
- **Past a refusal:** settled: edges are jumped off up to `max_drop`, with
  enthusiasm, and a refusal that is left (a drop deeper, a wall) brakes the
  body to where it ran out and then leaves it to the stick.
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
