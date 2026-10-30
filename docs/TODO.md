# TODO

Tracked future work for Voxel Phase.

---

## Constraint system

Add a generalized constraint system (joints, orientation locks, distance constraints) to the physics solver. Immediate need: keep the player capsule upright.

Design doc: [CONSTRAINT_SYSTEM_PLAN.md](CONSTRAINT_SYSTEM_PLAN.md)

### Deferred work

Items explicitly deferred from the initial implementation. These should be addressed before the corresponding features ship.

- **Constraint islands for sleeping** — constraints must participate in island building so that a sleeping body constrained to an awake body gets woken. Safe to skip only while all constraints are world-anchored on always-awake bodies (keep-upright on player). **Must be wired in before any two-body constraint (distance, hinge) ships.**
- **Constraint-aware shock propagation** — the conditioner may need to see constraint edges to compute correct BFS depths when a constraint bridges two contact sub-graphs. Not needed for world-anchored constraints.
- **Per-substep row re-expansion** — rows are expanded once per frame. A spring row recomputes its bias every substep through `RowRefresh` (`solver/row_refresh.rs`); if multi-substep drift causes visible artifacts with rigid positional constraints (distance, hinge), they can opt in the same way.
- **Hinge, ball-joint, prismatic, motor variants** — add as needed, the row-based infrastructure supports them.
- **Constraint breaking** — max force thresholds for breakable joints.
- **Constraint debug visualization** — rendering constraint axes, limits, error vectors in the debug overlay.

---

## Speculative contacts: no prediction for a collider already touching the ground

Static-geometry prediction runs only for a collider with no static contacts this frame,
so a ball rolling along the floor toward a wall at band speeds (roughly 2–18 m/s for a
0.2 m sphere at 60 Hz) gets no speculative contact against the wall and overlaps it by up
to a frame's travel before the discrete manifold sees it. Body pairs are not affected —
each pair is predicted on its own.

The fix is to predict against the geometry the current manifold does not already cover
and merge the two, which needs a rule for a manifold that holds both real and predicted
points under the four-point limit.

---

## Terraced slopes in generated terrain

Sloped ground carries faint terraces — bands running along the contours, spaced
like the voxel lattice. Flat ground is clean, which is the diagnostic detail: a
surface at constant height has nothing to step between, so this is the surface
stepping between lattice planes rather than the general mottling that a
quantised density field would produce everywhere.

Long-standing and previously invisible. It surfaced when terrain gained detail
normals (VISUAL_DIRECTION.md §4.2), which give the eye enough surface structure
to read the banding; it is *not* caused by them, and survives setting the detail
strength to zero.

### Suspected cause, unverified

`generate_terrain` writes an exact sub-voxel density for the topmost solid voxel
of each column and its air cap, so the *vertical* crossing lands on the true
surface height. Neighbouring columns of differing height still carry saturated
±1 densities, so marching cubes' *horizontal* edges interpolate between
saturated endpoints and land on the lattice rather than on the surface. On flat
ground neighbouring columns agree and nothing steps; on a slope every column
boundary is a potential step.

If that is right, the fix is to give the samples flanking a surface crossing a
signed distance to the *surface* rather than to the top of their own column —
the same principle as the carve path, which already computes a real distance
(`csg::carve_density`, and the note on `Voxel::air`).

### Why it matters beyond looks

§4.4 slope zoning thresholds on the surface normal, and this artifact lives in
the normal. A zoning band tracking terraces instead of geometry would be much
harder to diagnose once the two are layered.

Reproduce with `cargo run --bin visual_bench -- terrain_forms` and look at the
open ground in the `slope sweep` tile.

---

## Sliding off an edge

A body standing on an edge stays there with its centre well past it, the
capsule's bottom sphere resting on the corner and the feet over nothing. It
should slide off once its centre is past the edge. This is the character
controller, not the traversal planner or the legs: it happens on main too,
with no plan driving.

### Sketch

- **Detect** a body held up by its bottom sphere's contact on an edge alone,
  whose centre lies horizontally further from that contact than a foot can
  stand (about 0.15 m, the stance half width plus a little), with no floor
  within step height in a short cast straight down from the centre.
- **Respond** with a gentle outward push, horizontal and away from the
  contact, growing with the overhang, and stop the drive holding the body
  in place while it lasts, so it tips off.
- **Test** with a scenario each way: the capsule's centre 0.2 m past a ledge
  slides off within a set time; 0.05 m past, it stays.

The planner already never plans a stop past an edge, so the two do not meet.

---

## Feet snap at a landing and a hop's launch

A landing foot jumps 0.24–0.29 m relative to the pelvis in a frame
(a swing moves about 0.13 m a frame), and a planned hop's launch pose
0.29 m. Measured with `character_viewer` on `jump_straight`, `air_reverse`
and `hop_reverse`, with and without `--track`: the same untracked, so not
the planner's.

The landing pop is deliberate: `lerp_feet` (`src/animation/pose.rs`) takes
hip-anchored feet to world-placed ones in one frame, so a landing foot does
not skate along the ground through the 0.1 s crossfade. A fix has to
blend hanging feet onto their landing spot before touchdown rather than
across it. The launch snap is not yet looked into.

---

## Crouch-walk drags the trailing foot

Crouch-walking, the trailing foot lifts 0.40 m behind the hip but plants only
0.26 m ahead, so the legs read as dragging behind (`character_viewer
crouch_walk`, untracked). It is the foot placer's own gait at crouch. Suspected,
unverified: a foot may not lift until the other has landed, which holds the
trailing foot back at crouch's slow cadence. The crouch's pelvis offset
(0.12 m, both crouch presets) is bounded by the knee clearing the ground
(`a_crouched_knee_clears_the_ground_wherever_the_foot_is`), so the fix is in
the gait, not the pelvis.

---

## Jump is lost through a planned hop

A planned hop leaves the ground on its own, and while it is in the air the
jump button does nothing: on a plank bridge, a press that falls in a hop is
lost (0.28 s for a 1.5 m/s hop, 0.6 s at 3 m/s). The planner charges each
second in the air for it (`CostWeights::airborne`), which keeps hops short,
but cannot remove it. Proposed: a jump pressed within about 0.1 s of a
planned hop's takeoff turns the hop into that jump.

---

## A run over loose crates flies high and lands hard

On `crates_run` the body flies crate to crate with an apex about 0.8 m over
its standing height, loses speed along the path in the air (5.0 to 3.4 m/s
in one flight) and lands at 3–4.4 m/s downward. An 11 kg crate takes the
landing, is thrown out from under the body with spin, and throws the body
back up at about 3 m/s; the landing foot pushes off where the crate was for
2–3 frames. `traversal --summary` counts those plants off footing, since it
reads the floor after the frame's physics. Why the plan chooses such high
flights over light footing, or whether the drive flies higher than planned,
is not known. One `crates_uneven_run` start in 30 does not get across; not
looked at.
