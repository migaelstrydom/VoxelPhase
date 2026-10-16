# Traversal — Plan

How a character crosses arbitrary, mutable terrain: stepping up lips, running
over gapped planks, scrambling steep ground, grabbing ledges and climbing. This
document is the plan for the whole project and the design for its first
engine change. Phase 0's measurements live in `docs/traversal/`.

> **The project in one sentence:** the legs hold the body up, and a planner
> decides where the legs go.

---

## 1. The vision

The terrain is mutable, so it can be anything: jagged, steep, ledged,
overhung, cave-riddled, freshly blasted. The character should flow across it
at speed, and where there is no path, the player makes one with a grenade.

| Want | What it means in play |
|---|---|
| **Stride** | Small gaps, lips and rough ground are crossed without the player noticing. |
| **Hop** | Gaps a running stride can clear are hopped automatically, *only at speed*. Walking steps carefully; running flows. |
| **Jump** | Big gaps need the jump button. The game aims the arc at the landing the player plausibly meant (jump assist), so a near-miss lands. |
| **Scramble and slide** | Steep ground is climbed with hands, and steep loose ground can be slid down under control. |
| **Ledges** | Ledges are grabbed, hung from, shimmied along and mantled; a fall past one catches it. |
| **Climb** | Rough enough terrain is climbable, at a grip cost; ice and wet rock cost more. A wall kick leaves a climb. |
| **Recovery** | Near-misses stumble or catch rather than fail; big drops end in a landing roll. |
| **Dynamic footing** | Planks dip under a landing, rubble shifts underfoot, and the reaction goes into them. |
| **Creatures** | The same machinery, through the intent seam. Whether a creature can climb is gameplay: an escape route from some, a threat from others. |

The defining property: **affordances come from geometry and material, never
from authoring.** A grenade that roughens a cliff makes handholds; one that
breaks a lip makes a ledge. Blasting your own route is a core verb, and it
falls out of the terrain being mutable rather than being built on top.

### 1.1 Rules of feel

These outrank realism wherever they conflict.

1. **Predictable.** The same input on the same terrain gives the same move.
2. **Never auto-hop into a fall.** An automatic move never leaves the
   character worse off than standing still would have.
3. **Slowing near an edge stops.** Easing off the stick at a gap is always a
   stop, never a hop.
4. **The jump button is the commitment.** Anything that risks a fall is the
   player's decision.
5. **The body and the feet tell the same story.** What the body does is what
   the feet were planned to do; neither follows the other after the fact.

---

## 2. Where things stand

The player is a capsule (radius 0.25 m, half-height 0.5 m) that rests on its
round bottom. The traction drive (`docs/TRACTION_DRIVE_DESIGN.md`) moves it by
friction rows with a non-zero target at its contacts, and the Support Set,
built from those contacts, is the one source of grounding. Animation follows:
the foot placer plans steps around a body it does not hold up, and
`PelvisDrop` lowers the pelvis where the ground under a hip is too far.

The capsule's bottom is the character's foot, and every symptom traces to it:

| Symptom | Cause |
|---|---|
| Snagging in the plank bridge's gaps | The capsule drops into a gap and meets the far plank's edge. |
| No step-up | Since crease contacts, lips of 3–15 cm are climbed with a hop and lose speed; 25 cm stops the walker. |
| Jitter on jagged ground | Every ridge is a collision the round bottom rides over. |
| Feet that disagree with the body | The legs never carry the weight, so the animation can only chase the body. |

The phase 0 baseline (`docs/traversal/BASELINE.md`) puts numbers on each.

---

## 3. Architecture

```
  StaticGeometry + dynamic bodies
           │ casts against both (local, on demand)
           ▼
  ┌─────────────────────┐
  │ AffordanceSampler   │──► footholds · handholds · ledges · walls
  └─────────────────────┘    (from geometry and material: slope,
           │                  roughness, lip shape, grip)
           ▼
  ┌─────────────────────┐ ◄── CharacterIntent, velocity, grip, state
  │ TraversalPlanner    │
  └─────────────────────┘
     │ the move: stride · hop · jump-assist · scramble · slide ·
     │           mantle · hang · climb
     │ the next supports: where each foot and hand goes, and when
     ├───────────────────────────────┐
     ▼                               ▼
  Body (physics)                  Animation
  Stance: a suspension holds      foot placer and hands take the
  the body at ride height;        plan's targets instead of
  drive and jump unchanged;       inventing their own
  hop arcs, wall and ledge holds
```

The plan is the single seam between deciding and doing. Physics never asks
animation anything (the traction drive's rule, §6.6 there), and animation
never decides where a foot goes that the body was not told about.

The planner reads `CharacterIntent`, so creatures get every move through the
intent seam with no second path. Which moves a creature may make is a
declaration on it, like its `Actuator`.

---

## 4. Phases

| # | Phase | Delivers | Done when |
|---|---|---|---|
| 0 | **Measure** | The traversal garden level, `character_viewer` scenarios per station, traversal metrics, a baseline | Every symptom in §2 has a number |
| 1 | **Stance** | The legs hold the body up: a suspension constraint at ride height, a raised capsule, step-up for free; on a branch, gated (§5.12) | §5.13: plank bridge with no snag, 25 cm lip with no hop, rough-field bounce halved |
| 2 | **Foothold planning** | Look-ahead footholds along the velocity, stride fitted to land on them, the stance riding on the planned foothold, automatic hops at speed | Running across a wide-gap bridge lands on planks; walking stops at the gap |
| 3 | **Jump assist and landings** | Landing selection for the jump button, arc nudge within a budget, landing roll, stumble | A slightly short jump at a ledge lands |
| 4 | **Scramble and slide** | Hands on steep ground, a controllable slide on loose steep ground | 45–60° slopes climbable at a cost; scree descent flows |
| 5 | **Ledges** | Ledge detection, grab, hang, shimmy, mantle, catch on a fall past | Mantles the garden's chest-high wall; catches the lip when walking off |
| 6 | **Climb** | Handhold field from roughness and material, grip budget, wall kick | Climbs a grenade-roughened cliff; ice refuses |
| 7 | **Dynamic footing, creatures** | Support on moving bodies with reaction, creature traversal declarations | A plank dips under a landing; a climbing creature follows up a wall |

Phases 0–2 are the risky core and go in order. 3–7 build on 2 and can be
ordered by play-testing.

Each phase extends the phase 0 suite with its own stations and metrics before
it is built, and is judged against the baseline.

---

## 5. Phase 1 — Stance: the legs hold the body up

### 5.1 The idea

The body stops resting on its collider. It rides at a **ride height** above
the ground on a **suspension**: a general physics constraint in which a shape
cast holds a body up on a spring, with friction where it stands. It is the
textbook raycast-wheel of vehicle physics, with nothing in it about
characters.

The character declares a **stance**, which builds a suspension and steers its
rest length. The capsule is raised so its bottom sits at about knee height,
and from then on it is only the contact envelope: it bumps into walls and is
what gets hit.

What that gives:
- **The gaps between planks** stop mattering: nothing touches them.
- **Steps below the capsule's bottom** become a change in ride height, with no
  step-up logic.
- **Jagged ground** is smoothed by the spring's damping.
- **Taller obstacles** still meet the capsule.

This repeats the move the project made for buoyancy: a body's colliders are
its contact envelope, and where that is not the body, something else carries
the weight (CLAUDE.md, "Bodies vs their bulk").

### 5.2 Terms

| Term | Layer | Definition |
|---|---|---|
| **Suspension** | Physics | A constraint: a cast from an anchor on a body, along an axis, holding the body at a rest length above what it hits. It is a one-sided spring, with friction and torsion rows bounded by the spring's own load. It knows nothing of legs. |
| **Rest length** | Physics | The anchor's distance above the hit at which the spring is at rest under the body's own weight. The owner may change it every frame. |
| **Coupled bound** | Physics | A row's impulse bound given as `μ ×` another row's live accumulated impulse, rather than as fixed numbers. This is how friction is bounded by load. |
| **Constraint support** | Physics | A support in the Support Set that is a constraint's row rather than a contact. |
| **Stance** | Character | The character's declaration that legs hold it up: ride height, reach, ramp length, maximum leg speed and force, spring frequency and damping ratio, foot radius. It builds the suspension and writes its rest length each frame. |
| **Ride height** | Character | Height of the body's origin above the ground the legs stand on, at rest. |
| **Ride target** | Character | The rest length the stance asks for this frame: a rate-limited, look-ahead trajectory toward ride height (§5.7). |

### 5.3 D1 — How the legs reach the solver: a suspension constraint

**Considered and rejected:**

| Option | Why not |
|---|---|
| A "leg contact" (a synthetic manifold kind) | It teaches the general contact pipeline about characters, and every manifold consumer must learn what a leg is. |
| Leg rows without coupled bounds | Traction's budget `μ·N` needs the load live inside the iteration loop. Bounds fixed at expansion cannot give it. |
| Medium anchor, no engine change | The reaction is no longer solved, grip is no longer `μ·N`, and grounding gets a second source, which the traction design removed on purpose. |
| Kinematic character controller | It discards the traction drive and the honest physics built on it (carried by platforms, pushing planks). |
| Soft contacts as a material | It touches every contact in the hot path, and a foot collider still snags in gaps. |

**Chosen: a suspension constraint.**

The honest trade with a leg contact:
- **A leg contact** would not have to know about characters (a manifold
  sourced from a cast is general), but its soft normal fights the contact
  block solver and NGS in the hottest code there is.
- **The constraint route** leaves that code alone. In exchange it re-plumbs
  the drive's consumers (support, plan, ledger), the energy audit and the
  friction cone, all off the hot path.
- **Precedent:** the constraint library already holds a body against the
  world: `MediumDrive` counts as a load anchor (`anchors_position`,
  `constraint/types.rs`).

It is general physics in the constraint library, beside `FollowPoint` and
`keep_upright`, and needs four general additions:
1. **One soft row form.** Every soft row carries Box2D v3's softness triple
   (bias rate, mass scale, impulse scale), computed from the substep length,
   with two constructors:
   - **from compliance,** for today's `KeepUpright` and `FollowPoint`;
   - **from frequency and damping ratio, `(ω, ζ)`,** for springs.

   Today's `compliance/dt²` damps implicitly, so its feel depends on the
   substep count (up to 12 per frame). The new form does not, and it
   replaces rather than sits beside it. The change is not bit-identical for
   existing compliant rows. It lands first on the branch, as a commit of its
   own, and the bench suite must pass on it before anything else is built.
2. **Per-substep row refresh.** Constraint rows are expanded once per frame
   (`solver.prepare` in `world.rs`), and their bias is reused every substep.
   Joints get away with this because NGS repairs position, and NGS hides
   that they do not refresh. A spring cannot, because its force must follow
   its compression. Before each substep's velocity solve, a refreshed row
   recomputes its bias from the bodies' motion since expansion; its soft
   coefficients depend only on the substep length. The refresh lives beside
   the contacts' `solver/closing_allowance.rs`. It is opt-in for this phase,
   and the direction is every row refreshed. That is a later change, measured
   on the bench suite, so opt-in is not meant to be permanent.
3. **Coupled friction.**
   - **Pairing.** A row may name a sibling, `coupled_to: Option<u32>`: the
     sibling's `row_index` within the same constraint, resolved to a slot
     when rows are prepared, as ODE's `findex` does. This survives any
     reordering of the row slice.
   - **The cone.** A friction pair is bounded together as a Coulomb cone in
     the hit's tangent plane, by the cone projection `solver/friction.rs`
     already uses for contacts. That projection is extracted so both share
     it. Two rows clamped separately would give a square cone, with √2 more
     grip on the diagonals.
   - **The torsion row** is bounded by `g·μ·r` times the load.
   - **Uses:** wheels, joint friction and rope grip are all the same
     feature.
4. **Constraint supports.** The Support Set holds supports from constraints
   as well as contacts, and the traction planner drives them (§5.6).

### 5.4 The suspension

```
        anchor ●  (on the body; for a character, its centre of mass)
               │  rest length                 ┐
               │                              │ reach (rest length + slack)
               │                              │
               ○  cast shape (a small sphere) │
  ground ──────┴───────────────────────────── ┘
```

**Declaration** (physics, per suspension):

| Field | Meaning |
|---|---|
| `body`, `anchor` | The body held up, and the point on it the cast starts from. |
| `axis` | The direction the spring pushes and the cast runs (against it). A vehicle uses its body's down axis; a character uses gravity (D2). Friction does not follow it: it is always in the hit's tangent plane. |
| `cast_radius` | The cast shape's radius. |
| `rest_length`, `slack` | The spring's rest length, and how much further the cast reaches. |
| `spring` | `(ω, ζ)`, against the body's own mass: `k = m·ω²`. |
| `max_force` | Bound on the spring row's impulse per substep, so nothing launches a body. |
| `exclude` | Bodies the cast ignores: for a character, what it holds, is jointed to, or carries on its head. |
| `friction`, `torsion_radius` | Whether it grips, and the patch radius of its torsional row. μ comes from the materials at the hit, as a contact's does. |

**Each frame**, before the Support Set is resolved (`world.rs`):
- **The world casts each suspension** against `StaticGeometry` and bodies,
  once per frame, as the narrowphase runs once per frame. The cast reaches
  `rest_length + slack` plus the frame's travel along the axis. The hit
  (point, normal, partner, materials) is stored on the constraint.
- **Then the suspension expands to rows** against what it hit:
  world-anchored for static ground, two-body for a body. Rows are emitted in
  the order spring → friction pair → torsion, and the coupled rows name the
  spring by `coupled_to`. Joint rows solve before contacts in every
  iteration (`solver/island_solver.rs`), in slice order, so the spring's
  impulse is current when its friction is bounded.

The rows:
- **Spring row:** soft, push-only, bounded by `[0, max_force·dt]`, refreshed
  per substep.
- **Friction pair:** in the hit's tangent plane, at the hit point, coupled
  to the spring row (`coupled_to`). The cone bounds the whole tangential
  reaction: the friction rows plus the spring's own share along the surface.
  - The load is the spring's share along the surface normal,
    `N = λ_s·(axis·n)`.
  - The tangential reaction is `λ_s·axis_t + λ_t`, where `axis_t` is the
    axis's part along the surface.
  - The bound is `|λ_s·axis_t + λ_t| ≤ g·μ·N`.

  This is Coulomb friction for any spring axis. A vertical spring on a slope
  steeper than `atan μ` slides, as a body on that slope must. The tangent
  basis is fixed against a world reference projected into the plane, so the
  warm impulses mean the same thing from frame to frame.
- **Torsion row:** about the hit normal, coupled by `g·μ·r·N`. `Iterative`,
  never `HardProjection`.

None of these rows takes NGS position correction (the new kind is skipped
there, as compliant `KeepUpright` is). None takes shock-propagation mass
scaling, which applies to contacts only.

**Warm starting** uses the constraint's warm-impulse slots, which survive
the cast crossing triangles: the suspension is one constraint, whatever
triangle it lands on. They are zeroed when the hit partner changes or there
is no hit, so stepping from rock onto a light can does not warm-start the
body's whole weight into the can. The friction warm impulse is clamped to the
cone of the spring's warm impulse.

**Where the forces act.**
- **The spring** acts along the axis through the anchor, on both bodies. The
  partner's lever arm is the cast sphere's centre at the hit, which lies on
  the axis line, not the hit point itself, which can sit up to the cast
  radius off it on a slope.
- **Friction and torsion** act at the hit point, for both bodies. It is an ordinary two-point
constraint, so it conserves linear and angular momentum and creates no energy.
- **A character's anchor is its centre of mass,** so its spring does not tip
  it.
- **Traction at the foot leans it,** by the `height × traction` couple, as
  a real foot's grip would. `keep_upright` holds the character against that,
  as it holds it against contacts today.
- **If the lean shows** (gate 4 measures it), the general fix is a
  per-axis rotation lock, Jolt's allowed degrees of freedom. The engine has
  none today. It is not a special case in the suspension.

**Support and grounding.**
- A suspension that hit within reach is a constraint support.
- In the slack band it pushes nothing (`N = 0`). It grounds the body but
  grips nothing, because the coupled bound is zero.
- It is a floor only where the hit's normal counts as floor, by the Support
  Set's own rule.

**Warm starting** uses the constraint's existing warm-impulse slots: the
suspension is one constraint, whatever triangle the cast lands on.

### 5.5 What it costs the engine

- **The contact solver is untouched:** no new field, no new branch.
- **Constraint rows pay only where used.** Rows without a coupled bound or
  spring parameterisation are solved as today. The new branches are per
  constraint row, and a frame has a handful of joint rows against hundreds
  of contact rows.
- **A suspension costs** one cast per frame and four rows per iteration.
- **The Support Set** gains a source tag per support, read per supported body.

**Held by measurement:**
- Bench scenes with no suspension stay bit-for-bit unchanged **relative to
  the soft-row commit** (§5.3), not to main: that commit changes every scene
  with a compliant row, the player's `keep_upright` included.
- `physics_perf` and the bench timings are compared before and after; any
  regression is a defect.
- A suspension's own cost is reported.

### 5.6 What a suspension is to the rest of the engine

**The drive-aware surface (R11).** The traction design budgets the general
engine code that can tell a driven body from an undriven one
(`physics/drive/mod.rs`). Three sites remain today, and this plan adds none:
- **The Support Set's source tag** says whether a support is a contact or a
  constraint, not whether a body is driven. It lives in `drive/` anyway,
  which is drive-aware by construction.
- **The suspension's traction field** (target velocity, spin, gain) is a
  **parameter**, like `SolverContact::traction`. Every suspension carries it,
  expansion reads it unconditionally, and its default (zero target, gain 1)
  is plain friction.
- **The ledger and energy audit** read every constraint row's work
  unconditionally. The ledger is in `drive/`, and `contact_work.rs` does not
  ask which bodies are driven.

| Consumer | Change |
|---|---|
| Support Set (`drive/support.rs`) | `SupportContact.site` becomes a source: a contact site, or a constraint and its rows. A suspension's weight in `mean_normal` is its spring's load, in place of `depth + bias`, so it mixes with the capsule's contacts by what each holds. Both resolves (`world.rs`, frame and end of step) see constraint supports. |
| Traction planner (`drive/plan.rs`) | Runs per frame, before rows expand. A second pass over constraint supports writes the target velocity, spin and gain onto the persistent constraint, and expansion reads them into the rows' bias and cone. |
| Ledger (`drive/ledger.rs`) | `record_substep` reads constraint supports as well as manifolds. Drive-gain borrowing through a suspension is booked like a contact's. |
| Grip (`drive/grip.rs`) | No change: a hit that is not a floor makes no friction rows. |
| Jump (`drive/allowance.rs`) | No change: it needs only the support's point and partner. |
| Energy audit (`contact_work.rs`) | Extended to constraint rows. The spring's stored energy is its own line, not created energy. |
| Sleep (`sleep/`) | Islands are built from manifolds only, and a two-body constraint keeps both bodies awake (`sleep/manager.rs`). A world-anchored suspension permits sleep. A two-body one keeps both awake, like every joint today, until constraint-aware islands (`CONSTRAINT_SYSTEM_PLAN.md`) are built. That is out of scope here. |
| Sleeping partners | Standing on a sleeping body wakes it; `expand_constraints` skips sleeping bodies' rows. |
| NGS, shock propagation | Skipped, by kind (§5.4). |
| CCD | Unchanged: CCD guards the colliders, and the suspension is not one. |
| Impacts, contact events | None. An owner wanting landing feedback reads the compression. |
| Debug draw | The cast, the hit, and the spring's load. |

### 5.7 The stance: spring and ride target

The stance is the character's policy over a general suspension.

**Spring.** `k = m·ω²` gives a sag under the body's own weight of `g/ω²`,
about 1.6 cm at 4 Hz, and the stance offsets the rest length by it. Over a
light dynamic partner the partner gives as well, which is wanted (a plank
dips), and the bench measures how far.

**The ride target is a trajectory, not the ground.** A cast straight down
sees a 25 cm lip as a step input. At 4 Hz, critically damped, that peaks near
`ω·x/e ≈ 2.3 m/s`, which is a hop. Re-engaging the spring deeply compressed
is worse: at 0.3 m it asks `ω²·0.3 ≈ 190 m/s²` (19 g). That happens
bottoming out on the capsule, when the legs come back after a jump or a swim
(coming ashore low), and after a teleport. One rule covers all of these:
- **Look ahead.** Ground height is the highest floor a short forward sweep of
  the foot finds over the next ramp length `L` along the horizontal velocity.
  The body starts rising before the lip and is at height as the lip passes
  under it. The vertical speed this asks for is `v·Δh/L`.
- **Rate limit.** The rest length moves toward its goal at no more than a
  maximum leg speed, starting from the body's current height whenever the
  legs come down. A deep re-engage stands the body up rather than firing it.
- **Force limit.** The suspension's `max_force` bounds whatever the target
  does, and the capsule is the backstop beyond it. The rate limit, not the
  force cap, is what prevents launches, so the cap is sized for landings.
  Stopping from `v` under a cap `a` takes `v²/(2(a − g))`. Against a stroke
  of 0.2 m (inside the 0.3 m clearance):

  | Drop | Speed | Cap needed |
  |---|---|---|
  | 1 m | 4.4 m/s | 6 g |
  | 2 m | 6.3 m/s | 11 g |

  The starting cap is **10 g**: drops to about 2 m land on the legs, and
  bigger ones bottom out on the capsule, as every landing does today. A
  critically damped 4 Hz spring peaks at touchdown, `2ζω·v ≈ 22 g` from 1 m,
  so the cap clips the first substeps of a landing. Gate 2 measures whether
  that reads as a firm landing or a knock.

Stepping down needs none of this: the push-only spring lets the body fall to
the lower ground and catches it.

### 5.8 D2 — Which way the legs push

The spring's axis and the friction plane are separate questions. Friction is
always Coulomb in the hit's tangent plane (§5.4). The axis says only which way
the spring pushes.

| Axis | Effect |
|---|---|
| **Gravity (recommended for characters)** | The ride height is a height, measured straight down, on any slope. The jump already acts along world up (`drive/allowance.rs`). On a slope, part of the spring's push lies along the surface and uses up some of the friction budget, as a real vertical leg's would. |
| Ground normal | The spring follows the ground, so the ride height is measured across a slope, not down it. |

**Standing still on a slope is the traction drive's job, as it is today with
contacts:**
- A zero drive target holds the body within `g·μ·N`.
- Rock holds a character on any floor-grade slope (the player's gain is 5).
- An idle body on ice, or a sled on an icy slope, slides as Coulomb says it
  must.
- The drive's commanded velocity is realised through the tangent-plane cone,
  as at contacts. Whether uphill should be slower stays a locomotion rule:
  the traction design's deferred **Effort** seam.

With friction in the tangent plane, phase 4's slide needs no special
support in the suspension either.

### 5.9 The capsule and the body

- **Clearance.** The capsule's bottom sits at step clearance (about 0.3 m to
  start) above the ground at ride height. Below that, the suspension climbs;
  above it, the capsule meets the obstacle as a wall. The garden's 25 cm lip
  is a step, and 40 cm waits for phase 2 or 5.
- **The capsule keeps its top**, shortening from below. Crouching under
  overhangs is out of scope for phase 1.
- **The centre of mass is pinned.** The body's origin is its centre of mass,
  and raising the collider would move it by about 0.15 m. The player's mass
  comes from its figure (`BulkShape`), and the centre of mass is pinned to
  the figure, so the animation's pelvis offset and buoyancy's figure stay
  put.
- **Backstop.** A landing beyond `max_force` lands on the capsule, as every
  landing does today.

### 5.10 What changes for the character

| Area | Change |
|---|---|
| `LocomotionConfig` | Gains the stance tuning. Where the collider placed the body, the stance does. |
| Character FSM | **Launching** lifts the legs (suspension off) until the body rises clear, so the spring does not soak up the jump. **Swimming** lifts them too. **Airborne** keeps them down to catch the landing. Coyote time and jump buffering are unchanged: they read the same Support Set. |
| Jump | Unchanged: the normal impulse goes through the Support Set, now holding the suspension. A jump off a plank still pushes the plank. |
| Grab, carry, throw | Unchanged, apart from the cast's exclusions. |
| Animation | The pelvis hangs from the body as today, and the foot placer's probes keep finding landings. `PelvisDrop` stays: it is about a hip's ground, which the suspension does not see. Landing squash can read the compression. |
| Creatures | Opt in by declaring a stance. |

### 5.11 Where it sits in the pipeline

```
 frame:   narrowphase ──► manifolds ──────┐
          suspension casts ──► hits ──────┴──► SupportResolver ──► SupportSets
          traction planner ──► targets on contacts and on suspensions
          expand constraints ──► rows (suspensions read hit, target, gain)
 substep: integrate forces; carry compressions forward
          refresh soft rows (springs) and closing allowances
          solver: constraint rows first (springs, coupled cones) · contacts (unchanged)
          NGS (contacts and rigid joints), integrate, CCD
```

The physics stays ECS-free. A suspension is created and steered through
`PhysicsWorld` like any constraint. The stance lives in `src/character/`,
writes the rest length and legs-up/down from `CharacterControlSystem`, and is
the only place that knows about legs.

### 5.12 Delivery: a branch, gated

Phase 1 is built on the branch `suspension` and lands on main by
fast-forward. It lands only when every gate below has passed and the user has
play-tested it.

**Gate 1: engine pieces, unit-tested.** Spring parameterisation,
per-substep row refresh, coupled friction, constraint supports, and the
suspension constraint itself. Each comes
with tests that can fail:
- **Spring:** frequency and damping measured from a released spring match
  `(ω, ζ)`, at 1, 4 and 12 substeps.
- **Coupled friction:** a coupled pair slips at exactly `g·μ·N`, in any direction.
- **Soft row form:** the unified form lands first, as a commit of its own.
  It changes how `keep_upright` and the grab's `FollowPoint` feel, so before
  anything else is built on it:
  - the full bench suite passes;
  - every `character_viewer` and `anim_viewer` scenario holds;
  - the user play-tests standing, grabbing and carrying.
- **Coulomb on slopes:** an idle body on a vertical spring slides on a slope
  steeper than `atan μ` and holds on a shallower one.
- **Constraint supports:** a constraint support is driven, grounded and
  jumped from like a contact.

**Gate 2: bench harness scenarios** (`physics/bench_harness/`), each with
assertions and each visible in `bench_viewer`:

| Scenario | Asserts |
|---|---|
| Drop and settle | Sag within 10% of `g/ω²`; overshoot within 10% of what `ζ` predicts; total energy never above its starting value plus 1% |
| Spring tracks the substep | Spring impulse per substep follows compression (fails if rows are not refreshed) |
| Substep invariance | Settle height within 2 mm and landing overshoot within 10% across 4, 8 and 12 substeps |
| Lip table under a driven walker | No hop below the clearance; clipped at the capsule above it |
| Gapped floor | Speed within 5% across gaps narrower than the cast's reach |
| Slope hold, ice under drive | Holds on rock; slips at `μ·N` on ice, within 5% |
| Diagonal on ice | Slips at `μ·N` along a diagonal too (the cone, not a square) |
| Light partner | A plank under a landing dips; it is neither crushed nor launched |
| Rock onto a light can | No warm-start kick into the can |
| Kinematic platform | A stander is carried at the platform's velocity |
| Jump off a body | The body's momentum change equals minus the jumper's, within 1% |
| Drive gain through a suspension | Borrowing past `μN` is booked in the ledger |
| Suspension on suspension | The player on the sled: stable, with no shock-propagation scaling on constraint rows |
| Sleeping partner | Standing on a sleeping body wakes it |
| Rest | On static ground, a standing body sleeps within 2 s |
| Max force | A drop beyond the cap lands on the collider, without a launch |
| Perf | Scenes without suspensions: bit-for-bit against the soft-row commit, and median frame time within 2% of main; a suspension's cost reported |

**Gate 3: the hover sled, a spawnable in the test arena.** It proves the
suspension is general before any character uses it, and it is a toy worth
playing with, so it has to look the part. Its whole look is driven by the
suspension's state: what you see is what the physics is doing.

*The body.*
- A low, rounded hull in the game's injection-moulded plastic (a saturated
  primary, with chrome trim), not a box.
- Four hover pods under its corners. Each is a suspension anchored at its
  corner, so the sled tips, pitches into acceleration and rocks back.
- Each pod's face is an emissive ring. It is opaque, so it blooms, and it
  brightens with that pod's load.

*The force field*, one per pod, from the pod to where its cast hit:

| Element | Driven by | Look |
|---|---|---|
| **Field column** | Cast length, compression, load | A translucent blended column from the pod to the ground. It flares as it is compressed and has a Fresnel rim. Bands of light scroll down it, faster and denser with load. Colour runs from cool cyan at rest to white-hot near `max_force`, and a pod that bottoms out flashes. |
| **Ground ring** | Hit point, normal, spring oscillation | A thin emissive ring lying on the ground at the hit, so it blooms. It is tilted to the ground's normal, and its radius breathes with the spring. On a slope it is an ellipse; over a gap it vanishes and reappears on the far plank. |
| **Underglow** | Total load | A point light under the hull that lights the ground it rides over. |
| **Kick-up** | Load × ground speed, ground material | Dust and grit thrown out from each ground ring by the particle system. |
| **Strain** | Load spikes (a landing, the player jumping on, a grenade) | The columns flicker and the rings pulse outward. |

The field must stay readable, so nothing in it is decorative noise: a viewer
should be able to see which pod is carrying the weight.

**Over water (stretch):** where a column crosses the water surface, it
presses ripples into it through the water's disturbance input. The
suspension still casts to the bed. Hovering on the water itself would need
casts against fluids, and that is out of scope.

*Construction.*
- **Readout.** Physics publishes each suspension's state (hit, compression,
  load fraction) as a readout. A `ForceFieldVisual` component turns it into
  draws, so any suspension can opt into the look later.
- **Rendering.**
  - **Columns:** blended draws through the `TransparentQueue`, with a new
    `force_field` shader for the bands and the Fresnel rim.
  - **Bloom:** carried by the opaque emissive rings, since only opaque
    surfaces bloom.
  - **Kick-up:** emitters in the existing particle system.

*Verified.*
- `level_check` passes: placement and rest.
- A `visual_bench` scene shows the sled at rest, compressed and over a gap.
- A `character_viewer` filmstrip shows the player jumping on and riding it
  over the plank bridge.
- Then the user's screenshot from the game, because the bench has flattered
  before.

*Behaviour it must show.*
- It rides over the plank bridge's gaps.
- It tips and recovers when pushed off-centre.
- It dips when the player jumps on it, and carries them.
- It reacts to a grenade.

**Gate 4: the character stance.** Only then does the player get its stance,
judged by §5.13.

### 5.13 Acceptance: the character

Against the phase 0 baseline (`docs/traversal/BASELINE.md`):

| Check | Passes when |
|---|---|
| Plank bridge, run speed | No snag frames; horizontal speed within 5% of commanded throughout |
| Lip table, 3–25 cm, walk and run | No airborne frame; peak vertical speed ≤ 1.25 × `v·Δh/L`; overshoot above ride height under 2 cm |
| Lip table, 40 cm | Still stops (a phase 2 or 5 move) |
| Rough field | Vertical RMS acceleration ≤ 50% of the baseline |
| Landing from a 1 m drop | Overshoot under 3 cm; no second airborne frame |
| Standing still on static ground | Asleep within 2 s; energy audit clean |
| Regression | Every existing `character_viewer` and `anim_viewer` scenario holds or improves; the physics bench suite holds, with the traction drive and moving platform tests included |
| By eye | Landing, swimming ashore and jumping in filmstrips |
| Play-test | The user plays it; feel decides the tuning |

### 5.14 Open questions the gates answer

1. Which `ω`, `ζ`, ramp length and maximum leg speed feel like legs, and is
   the capsule ever clipped at run speed?
2. What cast radius bridges the arena's plank gaps without making a real gap
   look like ground? Phase 2 replaces this with planned footholds.
3. Does a light partner (a can, a small plank) get crushed or launched?
4. Is the energy audit clean with springs in it?
5. What does a cast cost, per suspension, in `physics_perf`?

---

## 6. Phase 2 — Foothold planning (outline)

This is designed in full once phase 1 has landed, because the stance's
measured behaviour shapes it. The intended shape:

- **AffordanceSampler, footholds only.** Candidate footholds ahead of the
  body along its velocity, within the reach of the next few strides. It casts
  against terrain and bodies, and judges each foothold by slope, flatness
  over a foot's area, and clearance above it.
- **TraversalPlanner, stride and hop.** It fits the next steps to landable
  footholds:
  - If a stride can be lengthened or shortened to land on a foothold, it is
    (the foot placer's step length becomes the plan's).
  - If the next foothold is beyond a stride but within a hop at the current
    speed, and the speed is above the hop threshold, it hops. The hop is a
    small normal impulse through the existing jump channel, timed to the
    gait.
  - Otherwise, a walk stops at the edge (§1.1 rule 3) and a run carries on
    only if the player jumps (rule 4).
- **The stance rides on the plan.** The suspension is cast at the planned
  stance foothold rather than straight down, so the body is held by the foot
  that is really down. This is §1.1 rule 5 made literal.
- **The foot placer consumes the plan.** Landing targets come from the plan.
  The placer keeps its timing, swing and capture-point balance.

Open questions for its design:
- how far ahead to look at run speed;
- how to keep the plan stable from frame to frame (hysteresis on footholds);
- what a hop does to the gait clock.

## 7. Phases 3–7 (sketch)

- **3. Jump assist and landings.** On the jump button, the planner picks the
  landing the player most plausibly meant (along the stick and camera, within
  the jump's reach). It nudges the arc toward it with a bounded air-control
  Allowance; the traction design's allowance budget is the home for this.
  Landings read the stance's compression: a soft bend, a roll past a fall
  height, a stumble when a foot lands short.
- **4. Scramble and slide.** A second stance mode for ground past the floor
  limit: hands join the Support Set as further suspensions, at a grip cost. A slide
  is a stance with low traction and steering, entered deliberately on loose
  steep ground.
- **5. Ledges.** The sampler finds lips: a near-horizontal top within reach
  over a near-vertical face. A hang is a hand-anchored hold (the grab
  system's `FollowPoint` already holds a point to a point). Shimmy moves the
  anchor along the lip, and mantle is a scripted lift into a stance on top.
  Falling past a lip within reach catches it.
- **6. Climb.** Handholds from terrain roughness over a face, scaled by the
  material's grip. A grip budget drains while holding, faster on ice and wet
  rock. Hands and feet move between holds the planner picks. A wall kick is a
  jump along the face's normal.
- **7. Dynamic footing and creatures.**
  - Suspensions on moving bodies already carry their velocity and reaction
    after phase 1; this phase makes it look right (a plank dipping, rubble
    rolling underfoot) and safe (no launches).
  - Creatures declare which moves they have.

## 8. Risks

| Risk | Mitigation |
|---|---|
| The stance changes how the character feels everywhere: landings, slopes, pushes, platforms, swimming ashore. | Phase 0 metrics, the full scenario suite, and a play-test before tuning is fixed. The capsule backstop keeps the worst case at today's behaviour. |
| A general suspension misbehaves where nothing character-shaped tests it. | Gates 2 and 3 (§5.12): bench scenarios and the hover sled prove it before the character uses it. |
| The spring launches or bounces the body (deep re-engage, step inputs). | The ride target is rate-limited and the leg force is bounded (§5.7). |
| The suspension lets the character stand on anything that counts as floor. | The floor rule is explicit (§5.4); friction is Coulomb in the tangent plane, so ice slopes slide (§5.8). |
| Auto-hop surprises the player. | §1.1's rules are scenario assertions from phase 2 on, not only intentions. |
| Look-ahead casts cost frame time on many creatures. | Casts are local and per frame, not per substep. Measure with `physics_perf` when creatures opt in. |
