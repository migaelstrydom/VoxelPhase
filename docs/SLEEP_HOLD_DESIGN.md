# Sleeping Under a Push

How a resting body stays asleep, or falls asleep, while whatever touches it can't move
it: the player leaning on it, a ball rolling across it, a grenade landing on it. And how it
wakes on the frame something can.

Status: parked (2026-10-31). Nothing here is built. The case that prompted it, Stack
Yard's temple, was fixed in content instead: the stylobate is a static body standing on a
`Foundation` of bedrock, and the temple walk costs 0.47 ms a frame. The problem comes back
with any large dynamic body that something keeps touching.

## 1. The problem

Today nothing that an awake body touches can sleep:

- **Islands.** An island of touching dynamic bodies sleeps only when every body in it is
  a candidate (`SleepManager::update_sleep_states`). The player is never a candidate:
  its KeepUpright forbids it (`ConstraintKind::permits_sleep`), and `set_body_drive` wakes
  it every frame. A rolling ball isn't a candidate while it rolls. Whatever either one
  touches shares its island and stays awake.
- **Waking.** Any awake dynamic body touching a sleeper wakes it, however light the touch
  (`SleepManager::wake_touched`).

Stack Yard's temple shows the cost. Its stylobate is one dynamic body of about 19 × 28 m
on 0.5 m terrain, and the player can't climb its 0.33 m bottom step. Pressed against it,
the temple stays awake as long as the player does. Measured with `character_viewer`, the
player sprinting into the step along +z and then standing there:

| | physics per frame | manifolds |
|---|---|---|
| today | 3.83 ms | 76 |
| temple asleep, player still blocked by the step (scratch build, §2) | 0.47 ms | 2 |

Every body in the temple is a sleep candidate the whole time. They are still; only the
player's island holds them awake. A ball rolling across a dynamic floor, or a crate the
player leans on, does the same on any terrain.

## 2. What a scratch build showed

Two hacks in the sleep manager, measured on the same walk:

1. **Contacts with the player don't join islands.** The temple falls asleep 30 substeps
   after it stops, and `wake_touched` wakes it again on the next frame. It sleeps for 2
   substeps out of every 32, and the frame costs the same as today (3.9 ms).
2. **And the player wakes nothing.** The temple stays asleep and the frame costs 0.47 ms.
   The player is still stopped by the step, but creeps 2 cm into it over 6 s. That's
   probably because the solver treats the sleeper as a movable body with its own mass: the
   player's push changes its velocity, and neither integration nor the sleep manager ever
   applies or clears that velocity.

So the island rule alone saves nothing, and a sleeper that isn't woken must not move in
the solve either.

The solver path for an awake body pushing a sleeping one is otherwise unused today:
`wake_touched` wakes the sleeper and regenerates its contacts before the solve
(`PhysicsWorld::generate_contacts`). Nothing in the solver knows which bodies sleep,
beyond leaving out joints whose bodies all sleep.

## 3. The idea

**A still body need not move while nothing touching it can push it harder, this frame,
than its supports can hold. Then it may sleep, and to whatever touches it, it's ground.**

That replaces both of today's rules, for every body, not just the player:

1. **Holding.** A still body *holds* an awake body touching it when the hardest push that
   body's island could put through the contact during the frame (§3.1), added to the still
   body's weight, lies in the wrench cone of the still body's supports (§3.2).
2. **Islands.** A contact joins a still body to an awake body's island only if it doesn't
   hold that body. The still body can then fall asleep while the other keeps moving.
3. **Waking.** A sleeper wakes when an awake body touches it that it doesn't hold. The
   test runs in the narrowphase, before the solve, as `wake_touched` does today.
4. **Ground.** In the solve, a sleeper has zero inverse mass and inertia. To shock
   propagation and the statics gate it's ground.

The test is the same for falling asleep (rule 2) and staying asleep (rule 3), so a body
can't fall asleep under a push that would wake it on the next frame.

**Nothing is late.** The push in the test is a bound on everything the toucher's island
can deliver during the whole frame, impacts included, not a measurement of last frame's.
So wherever the sleeper holds, today's rules would have left it in place too, and wherever
it doesn't, it wakes on the same frame today's rules would have woken it.

### 3.1 The push bound

The push an awake island can put through one contact is limited by two things: what it
can deliver by stopping or reversing its own motion, and what outside forces add during
the frame.

```text
J_n  ≤  (1 + e) · ( √(2·M·K) + F·Δt )
```

- `M`, `K`: the island's total mass and kinetic energy, linear and angular, at the start
  of the frame. An impulse along `n` that stops the approach of a point with effective
  mass `m_eff` and normal speed `v_n` is `m_eff·v_n`, and the energy it takes,
  `½·m_eff·v_n²`, can't exceed `K`; `m_eff ≤ M`.
- `F`: the outside forces the island's bodies can feel during the frame: gravity, and the
  declared authority of every drive and motor on them (a support drive's traction, a
  medium drive's row bounds). A joint with no impulse bound makes `F` unbounded, so its
  island holds nothing.
- `e`: the pair's restitution, since a bounce returns the impulse.
- `Δt`: the frame.

The contact's own friction cone limits the direction: the push on the still body is
`J_n·(−n) + f`, with `|f| ≤ μ·J_n`, plus the torsional friction the solver allows about
`n`. The set of possible pushes over a manifold is the convex hull of `J_n_max` times each
friction-pyramid edge (and torsion sign) at each contact point. Holding is a convex
condition, so the still body holds every possible push exactly when it holds each vertex
of that hull: at most four points × eight edges per manifold.

The bound uses the toucher's whole island, not just the toucher, because a box shoved by
a stack behind it delivers the stack's momentum in the same solve. The island is the one
built from this frame's manifolds, speculative contacts included.

Before building, this derivation gets written out in full, with the energy that outside
forces add during the frame and the case of a bounce off other ground, and checked
numerically on each case of unit test 3 (§5.1).

### 3.2 Supports

A still body's *supports* are its contacts with ground: static geometry and static bodies.
Kinematic bodies are not ground; they move.

A body with a contact to any dynamic body, or with an active joint, has no supports in
this sense and holds nothing. That keeps the test sound: the load on a body resting on
another dynamic body passes into that body, and a single-body test can't see whether it
holds.

Stacks still work out through the wake loop that exists today. Lean on a temple column
and the column wakes, since it rests on the stylobate. The awake column then touches the
sleeping stylobate, and `wake_touched` tests the stylobate against the column's island,
which is the column and the player. The stylobate holds, so only the column is awake.

## 4. Components

```text
        narrowphase                    solve                  end of frame
 ┌────────────────────────┐   ┌────────────────────────┐  ┌──────────────────────────┐
 │ wake_touched           │   │ rows with a sleeper:   │  │ update_sleep_states      │
 │  awake toucher of a    │   │  sleeper inv mass 0,   │  │  still body + awake      │
 │  sleeper: PushBound vs │   │  ground to shock prop. │  │  toucher: PushBound vs   │
 │  RestingSupport        │   └────────────────────────┘  │  current supports        │
 │  not held → wake       │                               │  held → no island edge   │
 └───────────▲────────────┘                               │  falling asleep: record  │
             │                                            │  RestingSupport          │
             └────────────────────────────────────────────┴──────────────────────────┘
                              both ask HoldTest
```

### 4.1 `PushBound`

The bound of §3.1 for one island and one manifold: `J_n_max` and the hull's vertices as
wrenches about the still body's centre of mass. It reads the island's bodies, their drives
and motors, and the manifold's points, normal, friction and restitution.

### 4.2 `RestingSupport`

A still body's supports: each contact's point, normal and friction, and the body's weight
as a force and a torque about its centre of mass. Built from the current manifolds for
the island rule, and kept when the body falls asleep for the wake rule. Waking drops it.
A body with no supports (§3.2) has none.

### 4.3 `HoldTest`

Whether a `RestingSupport` holds a `PushBound`: the weight plus each vertex must lie in the
supports' wrench cone. It uses `ContactWrenchCone`, which today is private to
`solver/shock_propagation/`; it moves to a `physics/statics/` module that both the solver
and the sleep manager can reach. It keeps the gate's certificate shortcut and its
hysteresis: a sleeper wakes only when a vertex is outside the cone by more than the
tolerance, and a body falls asleep only when every vertex is inside by more.

### 4.4 Sleep manager

- `IslandBuilder::build` leaves out a manifold between a candidate and a non-candidate
  when the candidate holds the other's push.
- `update_sleep_states` keeps each sleeping body's `RestingSupport`.
- `wake_touched` wakes a sleeper only if it doesn't hold its toucher's push.

### 4.5 Solver: sleepers are ground

`ConstraintSolver::prepare` already receives the sleeping set. A sleeper's mass properties
read as zero in any row it shares with an awake body, and its velocity is never written.
`statics_gate::is_ground` counts it as ground.

## 5. Tests

Each component's claim is exact, and each test checks it against an answer worked out
by hand. `physics_fuzz` keeps running as it does today, but no claim here depends on it.

### 5.1 Unit tests

1. **A sleeper in the solve is static geometry.** An awake body against a sleeper, and the
   same body against a static body of the same shape and pose: the awake body's velocities
   match to floating-point precision, and the sleeper's velocity stays exactly zero. The
   contact does no work on the sleeper (`ContactWorkLedger`).
2. **`HoldTest` against textbook statics**, 1 % either side of each threshold, for staying
   asleep and for falling asleep separately:
   - sliding: a block of mass `m`, friction `μ`, pushed sideways with `F`: held up to
     `F = μ·m·g`;
   - tipping: the same block, width `w`, pushed at height `h`: held up to
     `F·h = m·g·w/2`, so a slender column tips before it slides;
   - an overhanging plank: a load `P` at distance `d` beyond the support edge, the plank's
     centre of mass `c` inside it: held up to `P·d = m·g·c`.
3. **`PushBound` is never exceeded.** Each case runs the solver against static geometry and
   compares the normal impulse it delivered over the frame with the bound:
   - one box at three masses, three speeds and three restitutions, square on and glancing;
   - a box shoved by stacks of 1, 2, 4 and 8 boxes behind it;
   - a spinning ball meeting the ground, which turns spin into speed;
   - a driven body pushing off the ground with its full traction;
   - a box falling for the whole frame before it lands.
4. **`RestingSupport`** records exactly the contacts with ground, none from the body
   touching it, and nothing for a body that touches a dynamic body or has a joint.
5. **Islands:** a candidate and a non-candidate are joined exactly when `HoldTest` fails.

### 5.2 Scenario tests

Each runs twice, under today's rules and the new ones, and compares every body's path.
Where a sleeper holds, the claim is that today's rules leave it where it is too, so the
other bodies' paths agree within 1 mm and the sleeper doesn't move at all (today it
jitters). Where it doesn't hold, it wakes on the same frame as today and the runs are
identical.

| Scenario | New rules | Checked by |
|---|---|---|
| Driven body pinned against a heavy block | block asleep, 2 manifolds | driven body's path within 1 mm; no creep into the block |
| Grenade, box and player each landing on a heavy slab | slab never wakes | the faller's path within 1 mm; the slab doesn't move |
| Ball rolling across a heavy slab | slab never wakes | the ball's path within 1 mm |
| Box shoved by a stack into a light crate | crate wakes on the impact frame | identical runs |
| Driven body pushing a light crate along the floor | crate wakes on first contact | identical runs |
| Driven body walking onto an overhanging plank | plank wakes when the load would tip it | identical runs |
| Driven body leaning on a column on a slab | column wakes, slab stays asleep | the column's and body's paths within 1 mm |
| Stack Yard: walk into the temple (headless, `level_scenarios`) | stylobate asleep while pinned | `is_sleeping`, manifold count |

No test asserts a time; the temple's cost is measured by hand once, at the end.

## 6. Plan

1. **Sleepers are ground** (§4.5) with unit test 1.
2. **`physics/statics/`**: move `ContactWrenchCone` out of the solver, unchanged; the
   statics gate's tests pass as they are.
3. **`PushBound`**: the derivation of §3.1 written out, then the component with unit
   test 3.
4. **`RestingSupport` and `HoldTest`** with unit tests 2 and 4.
5. **Sleep manager** (§4.4) with unit test 5 and the scenarios of §5.2.
6. **Docs**: the Sleep section of `PHYSICS_ARCHITECTURE.md`, and this file's status.

## 7. Open questions

- **Stacks.** A body resting on another dynamic body holds nothing (§3.2), so a crate on a
  crate wakes at any touch, as today, and stays awake while touched. Carrying the push down
  through the stack, as the statics gate does, would let it hold; that's a later step,
  once the single-body case is measured.
- **CCD.** A fast body that CCD brings into contact with the toucher's island during the
  frame isn't in the island the bound was built from. Either the island takes in every
  CCD-active body its swept bounds reach, or a sleeper wakes whenever one does. A test for
  it belongs with unit test 3.
- **How tight the bound is.** It assumes the whole island's energy and every outside force
  go into one contact. Against a heavy body that costs nothing. Against a light one it
  wakes sooner than needed, which costs time but not correctness. If that matters, the
  bound can be tightened case by case.

## 8. Considered and set aside

- **Rules for drivers only.** The player never sleeps, but a rolling ball or a sliding box
  holds a floor awake just as long while it moves, and a grenade landing on the temple
  floor would still wake it.
- **Measuring last frame's push instead of bounding this frame's.** Simpler, but it makes
  a growing push wake the sleeper a frame late, and needs a separate rule for impacts.
- **Waking on the toucher's motion alone.** A capsule slides across whatever it runs on,
  so running across the stylobate would wake it on every frame. Counting only motion
  along the normal misses changes in load: a player walking onto an overhanging plank
  would find it frozen in place.
- **Welding big foundations to the world.** It removes the terrain contacts while the
  weld holds, but a grenade releases the weld (`TerrainAnchorSystem`), and the body is
  back on terrain, awake under the player. An unbreakable weld leaves the body hanging
  over a crater. Whether to weld a building is a content choice, not a fix for this.
- **Merging flat terrain into large faces.** It makes each frame a body spends awake
  cheaper, but does nothing about how many frames it spends awake, and does nothing on
  natural terrain.
