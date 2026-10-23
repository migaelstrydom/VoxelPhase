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
 PhysicsWorld ───► SectionSampler ── sphere casts down / up ──►│ Section: a strip of lanes
 (terrain, rubble,   (memoised per frame)  in every lane       │   surfaces in (s, d, z), each with
  spawnables)                                                  │   its support (static / body)
                                                               │      │
                                                               │      ▼ read only through SectionQuery
                                       previous Plan ─────────►│ Planner ◄── TransitionModels
                                                               │      │       (Stride; later Grab,
                                                               │      │        PullUp, Drop, Climb)
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
footholds. A profile is a list of **surfaces**: runs of samples with one
support, split where the support changes, where a cast finds nothing and (for
footholds) where the ground is too steep to stand on, so a surface's ends are
its edges.

| Field | Meaning |
|---|---|
| `support` | `Static`, or `Body { id, mass, asleep }` for rubble and spawnables |
| `points` | samples in increasing `s`: the height of the cast sphere's centre at rest, and the full 3D contact normal in the path frame (it keeps cross-slope, so a rock tilted sideways reads as tilted) |

Heights are the cast sphere's *centre*, so a layer is the space that centre
can occupy, and clearance is a comparison of centre heights.

Each lane holds surfaces, not a height per `s`, because ledges, walls and
overhangs are not heightfields. Gait planning reads only floors and ceilings,
but the type does not have to change when climbing arrives.

### Reading the section: `SectionQuery`

Transition models and the planner never read lanes directly. They ask a
`SectionQuery`:

| Query | Answers |
|---|---|
| `floor(s)` | the body floor under the path, with its normal and support |
| `footholds(window)` | every standable foothold in a window along and across the path, with its normal, support and margin to an edge; the planner judges which is best |
| `clears(motion)` | whether the capsule along a grounded stretch or flight arc stays on or above the body floor and under the ceiling |

Climbing adds `edges(s range)`, the grabbable edges, with the wall pass.

The strip of lanes is the first implementation behind it. A real 3D height
patch around the body can replace it later (see *Beyond one path*) without
touching the models, the plan types or the tracker.

### Layers, and why the cast radius matters

`SectionSampler` builds the section from `PhysicsWorld::cast_sphere`, which
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
  kind: Support | Flight              (later: Hang, Climb)
  start, duration                     seconds from now
  contacts: [Contact]                 who touches what during the phase
  body: BodyPath                      Support: reference (s, z, ṡ) over the phase
                                      Flight: takeoff state, then ballistic

Contact
  limb: LeftFoot | RightFoot          (later: LeftHand | RightHand)
  at: (s, d, z) on a surface, with its normal and support
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

## The planner

The planner searches over **plan states**: the contacts at a phase boundary,
which foot is next, and the body's `(s, z, ṡ, ż)`. Successors come from
`TransitionModel`s:

```rust
trait TransitionModel {
    /// Candidate next phases from `from`, each with its cost. Geometry checks
    /// common to every model (body path clears BodyFloor and Ceiling, contacts
    /// lie on a surface of the right layer) are done by the planner, not here.
    fn expand(&self, from: &PlanState, section: &Section, out: &mut Vec<Candidate>);
}
```

The first version has one model, `Stride`. It proposes the next footfall from
`footholds`, within stride length, rise and drop limits and a sideways
window around the foot's natural offset (wide enough for a cross-step), with
a flight if the gap or rise needs one.

- **Search.** Candidate footfalls are discretised every few cm along `s` and
  by lane across it, and a beam search runs out to the horizon, about 3 m or
  two to three strides. Ordering by `s` is what keeps the search small: the
  next footfall is always further along than the last. The beam width and expansion count are bounded, which
  bounds the cost per frame.
- **Cost.** Time to the horizon, deviation from the speed the stick asks
  for, uneven stride lengths, feet far from their natural offset (a
  cross-step costs more), flights (they cost a little), and footing:
  `Static` is cheapest, then heavy sleeping bodies, then light awake ones.
  Plus a **churn cost**: diverging from the previous plan's tentative
  footfalls. That cost, together with commitment, keeps the plan from
  flickering between two nearly equal options.
- **Commitment.** The current phase and the next footfall are committed. Each
  frame the planner replans only after the committed prefix. If the body
  strays from the committed path past a tolerance (the rubble shifted, the
  body was hit), commitment is dropped and the planner replans from the
  actual state. Each such event is counted, because they are where the edge
  cases will be.

### Why transition models are not the old move state machine

A move state machine has states with entry and exit rules and a handover for
every pair, so cases grow with the square of the move count. Here a model only
*proposes* phases, and the planner validates every proposal by the same rules.
Adding a model adds candidates and touches nothing else. What the drive and the
legs consume is `Phase` data, and its set of kinds grows slowly: Support,
Flight, then Hang and Climb.

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
| `crates/traversal_planner/` | pure: `Section`, `SectionQuery`, `Plan`, `Planner`, `TransitionModel`s, `GaitParams`, the scene format, a strip sampler for authored scenes, the SVG writer, `section_svg` bin |
| `crates/traversal_planner_web/` | the wasm front end (`wasm-bindgen`) and its HTML page |
| `src/traversal/` | game side: `PathPredictor`, `SectionSampler` over `PhysicsWorld`, `PlanTracker`, `PlanOverlay`, the capture key |

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
too. The crate's strip sampler casts spheres at the same two radii and the
same lanes as the game, so an authored scene's section follows the same rules
as a real one. Captured scenes carry the game's section verbatim.

### `section_svg` (CLI)

`cargo run -p traversal_planner --bin section_svg -- scene.ron -o out.svg`
draws two views sharing the `s` axis: the **side view** on top, and a
**top-down strip** under it, showing each lane's footholds and where each
footfall lands sideways. Without the top view, the tools would hide exactly
the lateral cases a side view gets wrong. Both views draw:
- the layers (body floor, footholds, ceiling), coloured by support;
- footfalls, coloured left or right, solid when committed and faded when
  tentative;
- flight arcs, and the body path with the capsule drawn at each phase
  boundary;
- the cost of each phase.

This is how I read a scene.

### The web tool (wasm)

The real planner in a browser page, published as a private artifact:
- the same side view and top-down strip as `section_svg`;
- drag, resize and add obstacles in either view (length and height in the
  side view, sideways extent in the strip); mark them static or loose;
- sliders for `GaitParams` and the cost weights; edit the body state;
- replan live;
- **time scrub**: step the body along its plan as if tracking were perfect,
  replanning each frame, to see commitment and churn over time;
- **perturb**: make a loose support drop or slide as a foot lands, to
  exercise the replanning that rubble giving way will force;
- import and export the scene file, including the game's captures.

### Fixtures and properties

`crates/traversal_planner/fixtures/*.ron`: every bad case found in the web
tool or captured in the game goes here. The tests check properties of every
fixture's plan rather than exact output:
- every flight arc and support path clears `BodyFloor` and `Ceiling`;
- feet alternate, and every footfall is on a `Foothold` surface at its own
  `d`;
- strides, rises and sideways reach are within `GaitParams`;
- replanning one frame later keeps the committed prefix and changes the
  next tentative footfall only when the section changed.

Each property gets a fixture that is built to break it, to prove the check
can fail. The tests run in milliseconds and always run.

### Fuzzer

Random scenes (planks, steps, gaps, slopes, stones on and off the line,
cross-tilted rocks, overhangs) checked
against the same properties, shrinking a failing seed to a minimal scene and
writing it out as a fixture. It lives in the planner crate.

### In game

- **`PlanOverlay`**: the plan through `DebugOverlays`. Footfalls as discs
  (left and right coloured, committed solid, tentative faded), flight arcs,
  the body path fading out towards the horizon, section samples coloured by
  support, the body's actual trail beside its planned one, and a flash when
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
| A | Workspace, planner crate: `Section` strip and `SectionQuery`, `Plan`, `Stride`, beam search, scene format, `section_svg` with both views | the plank bridge, lips of 3–40 cm, stepping stones in line and zigzagged off it, and a stone tilted across the path as authored scenes, read as SVGs |
| B | Property tests, a breaking fixture for each, fuzzer | fuzz sweep clean; every property shown able to fail |
| C | Web tool | published; you can author and tune the bridge in it |
| D | `SectionSampler` and `PathPredictor` in game, `PlanOverlay`, capture key; plan drawn but not acted on | captures of every Traversal Garden station plan the same as their authored versions; section (five lanes) + plan under 0.5 ms per frame |
| E | `PlanTracker`: drive and takeoffs through contacts | `traversal --summary` beats the baseline on every station, churn and divergence measured, rubble lane crossed |
| F | `FootPlacer` reads footfalls and their timing from the plan | the legs alternate plank to plank; clipping and unplanted legs measured as in the baseline |

## Open questions

- **Rises:** flights (first version) or a lift with its reaction applied
  through the planted foot. Settled in E.
- **Path prediction:** how far to trust the stick's curvature for the section
  frame on a hard turn, and what the plan does when the player stops or
  reverses (a plan that ends in a stop, and a churn cost waived on reversal).
- **Lateral footing:** the strip's width and lane count (five over ±0.3 m to
  start) and how far the foot placer may move a foot from its planned
  footfall are judged on the scree and rubble lanes in D and F. If the best
  way through lies off the stick's arc, the fan of arcs comes next (*Beyond
  one path*).
- **Section frame per phase**, for shimmying along a ledge (climbing only).
