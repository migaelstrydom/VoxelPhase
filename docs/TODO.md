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

## A run over loose crates flies high and lands hard

Running the Traversal Garden's evenly set light crates (z = 74, no longer a
`character_viewer` scenario), the body flies crate to crate with an apex about 0.8 m over
its standing height, loses speed along the path in the air (5.0 to 3.4 m/s
in one flight) and lands at 3–4.4 m/s downward. An 11 kg crate takes the
landing, is thrown out from under the body with spin, and throws the body
back up at about 3 m/s; the landing foot pushes off where the crate was for
2–3 frames. `traversal --summary` counts those plants off footing, since it
reads the floor after the frame's physics. Why the plan chooses such high
flights over light footing, or whether the drive flies higher than planned,
is not known. Running the uneven crates, one start in 30 did not get across;
not looked at.
