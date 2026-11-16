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

## Crouch-walk has a flight phase

Crouch-walking at 2 m/s, both feet are off the ground for three or four
frames of every step (`character_viewer crouch_walk --feet`). The gait takes
its duty factor from the Froude number, and on legs half a metre long 2 m/s
is already a run (`FootPlacerConfig::duty_froude_end`), so a crouch scurries
where it should creep. The stance itself is symmetric (0.23 m ahead to
0.22 m behind). A crouch preset that pins the duty factor above a half
would fix it; `crouch_walk_alternates_with_continuous_support` holds only
at the slower speed the invariant walks at.

---

## Braking hard, a planned step is given up for a short one

A body that brakes from a run to a crouch in a few frames stops short of the
footfall its swinging foot was aimed at. The foot waits over a step only for
a body about to arrive (`placer::LONGEST_WAIT`), and otherwise lands short
under the hip, off the plan. Following the plan's footfalls as it replans
for the slower body would land it on one.

---

## Planning costs most while turning

`traversal_perf --turn 90` lays the path afresh every other frame, and each
relay casts the whole strip again: 3000 casts and 3 to 5 ms on those frames,
against a median of 0.3 ms. A cache of cast heights kept in the world, not
along the path, would let a relaid path reuse the casts of the one before
it.

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
