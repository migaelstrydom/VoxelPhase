# Impact meter

How hard something was struck. The physics records each blow once, in
quantities a rigid solver cannot get wrong (`PhysicsWorld::hits()` and
`shoves()`, `src/physics/hit/`); each reader (harm, grenades, glass,
cleave, fracture, creatures, later sound and dust) judges it by its own
rule. This file says why blows are not read from contact impulses, what a
hit is, and which readers judge by hits.

## Why not the solver's impulse

A rigid solver gets *how much* momentum a contact moves right, and invents
*when*. An arrest lands in the substep the pair arrives in or is split with
the next, and that substep may fall in the next frame. A manifold's impulse
is shared between its contacts however the solve happens to share it, and
summed with the weight the contact carries. Wherever a joint or a drive
pushes a body into a contact, the contact's impulse is that push, with
nothing struck. Any meter that reads a frame's impulse
(`ImpactLedger::total_impulse`, `peak_impulse`, a frame-to-frame rise in
either, impulse over `dt`) reads that invention, and has to be tuned around
it with deadbands for weight, churn, sleeping and waking.

What a rigid solver does get exactly is how fast a pair was closing just
before the solve that stopped it: the normal velocity it already measures to
decide restitution. Everything else a reader needs (how much each side's
velocity changed, how hard and how sharply the pair pressed) follows from
that speed, the bodies' masses and the materials, without the solve.

## Measurements

Taken in November 2026 with a measuring harness that was not kept. What
they established about hits is held by the unit tests in
`src/physics/hit/`: a hit's approach is the speed its body arrived at,
wherever in a frame it arrived and through CCD too; a face, an edge and a
corner land on the patch they are; a resting body, a slow touch and a woken
stack are not hits; a pair's effective mass shares the blow; a blast's
shove is the impulse it gave.

### Drops on flat ground

A sphere, a flat box with a hair of tilt and a box landing on its corner,
from 0.5, 2 and 8 m, at 30, 60 and 144 Hz, 12 trials each differing only in
where in the frame the body arrives. Each reading over its analytic value (v
for speeds, m·v for impulses), `mean [min..max]`; a sample:

| Shape, h, Hz | approach/v | frame impulse/mv | peak contact/mv | frame-to-frame rise/mv | impulse over the contact/mv |
|---|---|---|---|---|---|
| sphere, 2 m, 60 | 1.00 [1.00..1.00] | 0.91 [0.58..1.02] | 0.73 [0.50..1.00] | 0.87 [0.56..1.00] | 0.97 [0.96..0.99] |
| flat box, 2 m, 60 | 1.02 [1.00..1.22] | 0.89 [0.64..1.02] | 0.31 [0.20..0.50] | 0.79 [0.34..1.00] | 0.98 [0.96..0.99] |
| corner box, 8 m, 144 | 1.00 [1.00..1.00] | 0.78 [0.49..0.96] | 0.73 [0.49..0.96] | 0.71 [0.37..0.96] | 1.10 [1.05..1.14] |

- Approach speed is exact to 1 % in every trial. The flat box's outliers
  (up to 1.29 at 0.5 m) are its second edge slapping down after the tilt
  tips it: a real second blow.
- Every per-frame impulse meter swings 2–3× on frame phase alone.
- A flat box's peak contact reads 0.13–0.53: four contacts share the
  impulse however the solve goes.
- Impulse summed over the whole contact is steady to about 5 % for a direct
  hit, once the weight carried is taken out (and a sleeping body carries
  none). The corner box's sum includes its topple onto a face.

### Face, edge and corner

A 0.5 m box dropped 2 m flat (0.2° and 2° of tilt), on an edge and on a
corner, 12 trials at 60 and 144 Hz, on the ground mesh's diagonal and clear
of it (alike). The hit's patch:

| Landing | Contacts in the arriving frame | Hit's patch |
|---|---|---|
| flat, 0.2° | 3.83 [2..4] | a face in 11 of 12; an edge where the narrowphase gave that frame only two contacts |
| flat, 2° | 2.83–3.67 [2..4] | an edge in 8–9 of 12: an edge strike and then a slap, which is what it is |
| edge | 2 | a 0.5 m line in every trial |
| corner | 1 | a point in every trial |

### An impact passed down a stack

A crate dropped onto a crate standing on a slab: the slab is a pane under a
load.

| h, Hz | slab approach/v | slab frame impulse/mv | slab impulse over the contact/mv |
|---|---|---|---|
| 0.5 m, 60 | 0.01 | 1.00 [0.70..1.88] | −0.69 [−1.13..0.32] |
| 2 m, 60 | 0.01 | 1.38 [0.61..1.77] | 0.54 [−0.26..1.02] |
| 8 m, 60 | 0.00 | 1.53 [0.56..1.72] | 1.40 [0.42..1.77] |

Nothing approaches the slab, so no hit reaches it, and its impulses swing
3–4× whatever is taken out of them. A likely cause, not confirmed: shock
propagation scales a contact's impulse differently on its two sides, so the
solver does not pass an impact down a stack as momentum. No contact
quantity measures a transmitted impact (see *Limits*).

### The player

`character_viewer` scenarios, the body's hardest frame (56.4 kg):

| Scenario | Contact impulse / m | Approach | Velocity change |
|---|---|---|---|
| `jump_standing` (full jump, flat) | 7.01 | 6.97 | 6.85, one frame |
| `jump_straight` (running jump) | 4.61 | 4.49 | 4.44, one frame |
| `ledge_drop` | 5.19 + 1.50 | 6.52 | 5.03 + 1.66, two frames |
| `shelf_sprint_off` (5.6 m shelf) | 10.49 | 10.37 | 10.33, one frame |
| `drop_reverse` (back onto a 40 cm step at 8 m/s) | 5.92 | 5.68 | 5.80, thrown up at 4 m/s |
| `ledge_jump_mantle` (pull-up) | 10.01 | 0.00 | 0.15 |
| `trampoline` | 1.55 | 0.01 | spread by the bed |
| `fall_in` (into deep water at 6 m/s) | none | none | ≤ 0.75 a frame over ~10 frames |
| `jump_in`, `trampoline` takeoff | none | none | 6.87, the jump itself |

- Landing on the feet stops the body in one frame: a full jump lands at
  7 m/s.
- A pull-up reads as a 10 m/s blow by impulse (the pull-up joint pushing
  the body into the top) and zero by approach.
- Water and a trampoline spread a stop over many frames.
- The player's own jump changes their velocity by 6.9 m/s in one frame,
  through the drive's allowance, not a contact: a meter that differences
  velocity sees it, a contact meter does not.

## Design

```text
  physics (no ECS) ────────────────────────────────────────────────────────────
  solver ── approach_speed on each contact, from the normal velocity
            it measures to decide restitution ─┐
                                                 ▼
  HitRecorder, after every solve (main and CCD), per pair, per frame:
    a contact that closed faster than HitConfig::speed and took impulse
    ──▶ Hit { pair, point, normal, approach, restitution, give,
              bluntness, effective_mass, patch }        PhysicsWorld::hits()

  blasts, as they are applied
    ──▶ Shove { body, impulse, point }                  PhysicsWorld::shoves()

  ImpactLedger: impulse per body and collider per frame:
    the load contacts carry, not blows                  PhysicsWorld::impacts()

  severity, Hit's methods: Δv of a side, peak force, duration,
    contact area, peak pressure
  ──────────────────────────────────────────────────────────────────────────────
  readers (game), each with its own rule; see Readers
```

Nothing in the physics knows a reader exists, and no reader recomputes a
contact. Load (what a contact carries) and impact (how hard it closed) are
kept apart: the ledger for one, hits for the other.

### `Hit`

One per colliding pair (body and collider each side, or static geometry)
per frame, from the substep it closed fastest in.

| Field | What it is |
|---|---|
| `body_a`, `collider_a` | `None` for static geometry |
| `body_b`, `collider_b` | `collider_b` is `None` for a contact from a CCD sweep, which keeps no collider |
| `point` | centre of the patch |
| `normal` | towards `body_b` |
| `approach` | closing speed along the normal, m/s: the fastest of the patch's contacts that took impulse |
| `restitution` | the pair's, as solved |
| `give` | how far the pair yields before it stops, m: both sides' gives in series (their sum) |
| `bluntness` | radius of the sharper side's feature, m |
| `effective_mass` | the pair's mass along the normal at `point`, kg: `1 / Σ (1/m + (r×n)·I⁻¹(r×n))` over the sides that can move |
| `patch` | `Point`, `Line { length }` or `Face { area }` |
| `continues` | the pair was hit the frame before too: the rest of a stop the solver split across two frames (it lets a body reach the surface still closing, and stops it the next frame), not a new blow. Readers that count blows skip it |

- **A hit is confirmed, not predicted**: a contact closing faster than
  `HitConfig::speed` (0.5 m/s) that took impulse, as Box2D v3's hit events
  are. A resting contact closes at one substep of gravity (0.04 m/s), a
  waking body at about the same, a joint pushing into a contact at none;
  none is a hit. A stop split across two frames is one blow, its second
  hit marked `continues`; no deadband has to tell one blow from the next.
- **The patch** is the pair's contacts that took impulse or are within the
  pair's give of touching, measured across the normal. A pair's contacts
  against static geometry merge across manifolds.
- **Effective mass** is worked out from the bodies at `point`, not taken
  from the solver's rows, which shock propagation scales.

### `Shove`

What a blast gave a body as the step applied it: the impulse (the body's
change of velocity is that over its mass) and where it struck. A blast
strikes a body at its centre of mass; if blasts come to load each part of a
body's bulk at its own point, each part's share is one more `Shove`.

### Materials

| Property | Where | Default |
|---|---|---|
| `give`, m | `ColliderMaterial`, `StaticSurface` | `HARD_GIVE` 1 mm; yielding ground `YIELDING_GIVE` 3 cm |
| `bluntness`, m | `ColliderDesc::bluntness`, else the shape's | a sphere's or capsule's radius; a box's or hull's `CORNER_ROUNDING`, 1 cm |

A pair's give is combined where its restitution and friction are
(`PairHeader`): a collider pair's two gives, or a collider's and the
surface it meets (`StaticSurface::give_with`). Static geometry with no
surface adds no give, as it adds no friction. CCD's sweeps against static
geometry do not ask what the ground is and take it as hard.

### Severity

`Hit`'s methods, each tested by numbers:

- **`delta_v(m)`**, the change of velocity of a side weighing m:
  `(1 + e) · approach · m_eff / m`. A body landing square on the ground
  with no bounce loses exactly its approach.
- **`peak_force`**: `m_eff · approach² / (2 · give)`, what stops the pair
  at a constant rate over its give. **`duration`**: `2 · give / approach`.
- **`contact_area`**: a face's own; a line `length × 2·sqrt(bluntness ·
  give)`; a point `π · bluntness · give` (Hertz: contact radius² ≈ radius ×
  indentation).
- **`peak_pressure`**: force over area.

A 125 kg crate at 5 m/s onto a pane (gives 1 mm and 3 mm) stops with
390 kN either way: 1.6 MPa face-on over 0.25 m², about 3 GPa on a 1 cm
corner. Glass that cracks locally above some tens of MPa takes the corner
and not the face, and bends under the 390 kN either way.

The constant-rate law is the simplest that orders blows rightly; it gives a
face and a corner the same force. Elastic contact (Hertz, a flat punch)
makes a bigger patch stiffer and its force larger by `√(size ratio)`. A
reader that needs that changes these methods and nothing else.

### Cost

Measured November 2026, interleaved release builds of `physics_perf`
(stack of 15 layers blasted, four igloos blown apart):

- The solver's `approach_speed`: within noise. (Measuring it again on the
  ledger's side, as a first version did, cost 7–9 % of the solve on a
  blast frame.)
- Recording hits: the solve stage's mean 1.5–2 % (about 0.008 ms) higher,
  its worst frame within noise. Final-state fingerprints unchanged.
- The four igloos peak at 151 hits in a frame out of 3444 contacts, about
  13 a frame over the run. A `Hit` is 144 bytes and a `Shove` 40, so a
  frame's records are tens of kilobytes at most, in buffers whose capacity
  is kept from frame to frame.

Hits and shoves are recorded always, not on request. A switch would cost
nothing to test (one branch per solve, not per contact), but would save
nothing in a game whose readers judge hits every frame, and a reader whose
switch was off would judge an empty list without a sound. If the 0.008 ms ever matters, the recorder's scan of every
contact can be cut to the manifolds the solver's own pass marks as fast,
without changing what it records.

## Readers

| Reader | Judges by hits as | Status |
|---|---|---|
| Grenade detonation | its side's Δv ≥ its detonation speed, so a glancing blow or a light body it strikes does not set it off | moved over (`projectile/detonation.rs`) |
| Harm (the player) | the player's Δv from every hit and shove, judged by how sudden it was over the pair's give (and the limbs', braced) | moved over (`src/harm/`); being tuned in play |
| Glass | pressure: a local crack; force: bending fatigue | its own meters: impulse over `dt` against a bearing (`glass/fatigue.rs`), and velocity change and jerk (`glass/jolt.rs`) |
| Cleave | force at the struck block | its own meter: fracture's spike, or m × Δv frame to frame (`BlowMeasure`, `cleave/`) |
| Fracture | force at a part, into `BondLoads` | its own meter: frame-to-frame rise in impulse less deadbands (`fracture/contact_load.rs`) |
| Creature damage | Δv of the creature's side | its own meter: speed lost frame to frame (`damage/sources.rs`) |

Readers move over in the table's order, one at a time, each judged with the
tool that owns its area (`character_viewer` `room_*` throws, `rubble_viewer`,
`physics_fuzz`) before the next. A reader that has moved over loses its own
meter and its deadbands; this table says where each stands.

## Limits

- **Impacts passed down a stack** reach no contact, so no hit. Options: pass
  a hit on to what the struck body rests on (`support_sets`), or solve loads
  over a compound's bonds as NVIDIA Blast's stress solver does.
- **Water** stops a body with drag, not a contact, spread over many frames;
  whether harm reads it is the harm design's question.
- **Which side is sharp.** A manifold does not say whose corner met whose
  face; a hit takes the sharper collider's bluntness.
- **A face landing a hair off flat** reads as an edge when the narrowphase
  gives the arriving frame only the leading edge's contacts (one trial in
  twelve at 0.2°). A speculative-manifold question, worth chasing if a
  reader cares about faces landing a hair off flat.
