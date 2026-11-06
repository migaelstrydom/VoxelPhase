# Compound Parts

A compound is an entity whose body is made of parts held together by bonds:
anything that can come apart. A part is a rigid cluster of colliders that
never comes apart; what it weighs and what fluids and blasts see of it ride
on its colliders. Fracture judges loads and splits the graph of parts; it
never draws anything and never rebuilds a collider. How a part looks is
kept by collider handle and drawn by a system of its own.

Status: design, reviewed twice (SOLID and general design), for the lead's
review. Branch `compound-parts`.

---

## Why

Four things in today's code point to one missing idea: a part that owns
what it is, independent of the body it is on and of the order of that
body's collider list.

**Fracture draws.** A compound is drawn by two pipelines. At spawn each
spawnable builds its own model (the bridge from its boxes, a stone block
from its weathered hull; the framed window already uses fracture's own
`compound_model_of`). After any break, `FractureSystem` throws that model
away and redraws the remnant and every freed piece from their collider
shapes (`compound_model_of`, `freed_piece_model`), through the compound's
`PieceStyle`, a material per child and a texture setting. The second
pipeline has to reproduce the first, or the object changes look as it
breaks. Much of `CompoundFracture` exists to make it:

| Field | What it patches |
|---|---|
| `materials` | the bridge's beams came away wearing the planks' material |
| `style` | a piece of a non-cuboid object changed shape as it broke |
| `texture_anchor` | re-centring the body slid the pattern across the remnant |
| `whole_shape` | a broken block's mesh could not tell old faces from new ones |
| `model_stale` | redraw once per frame, after cleaving and splitting |

`fracture` imports `app::spawnables::shared::models` to do it: the engine's
destruction layer depends on the game's spawnables. A child whose shape no
style draws (a capsule, a sphere) comes free as a body with no entity at all
(`fracture/systems.rs:509`): never synced, drawn or culled. That is why the
plants could not use fracture.

**A group comes apart into its children.** The split finds the connected
groups correctly, then spawns one body per child of every group but the one
with the most children (`spawn_freed_piece` per child). Two planks still
joined fly as two bodies, against what their bond says; a branch would land
as a stick and a handful of loose core spheres. Each freed child gets a
fresh dynamic body with fixed damping (`systems.rs:489`), losing the body
state it was part of: gravity scale, damping, `ignores_static`, bulk.

**Bulk belongs to the body, so it cannot follow a split.** `BulkShape` is
declared on the body (`RigidBodyDesc::bulk`) and overrides its colliders
whole. Nothing says which share of it belonged to a child, so
`detach_collider` throws it away (`clear_bulk`) and the body falls back to
its colliders. The plants branch added `PhysicsWorld::set_bulk` to declare
what was left: a workaround for the model, not a fix.

**Everything per-child is indexed by position.** Detaching swaps the last
collider into the hole, so every structure indexed by a child's place in
the list is remapped on every split, and each has had to learn to: the
compound's joints, materials and kicks, `ContactLoadTracker`,
`BreakDepths`, `FatigueTracker`, and `BrittleSheet`'s and `BrittleSolid`'s
own loads, damage and depths.

The plants branch carries its own break system (`BranchBreakSystem`) and a
blast rule for a bulk not on any body (`BulkShape::blast`), because fracture
could not hold a tree together as it broke. Both go once trees are ordinary
compounds, and branches then break under landings too, through the contact
loads fracture already judges.

---

## Terms

Each word means one thing throughout.

| Term | Layer | Meaning |
|---|---|---|
| **collider** | physics | A shape fixed to a body at an offset. It never moves relative to its body. |
| **body** | physics | A rigid body: one pose, one velocity. Its colliders move with it as one; its mass is theirs. |
| **constraint** | physics | A rule the solver enforces on one body against another body or the world: hold these two anchor points together, keep this attitude, stay within this spring's reach. Anchored on bodies (a point in each body's frame), never on colliders. |
| **weld** | physics | A constraint of kind `Fixed`: holds position and attitude completely. A tree's foot in the ground is a weld to the world. |
| **part** | ECS | One or more colliders on one body that never come apart: a plank; a branch with its crown. Named by a `PartId`. |
| **bond** | ECS | Fracture's record that two parts on the same body are stuck together and may come apart past a load, with the place they meet (its anchor). Not a constraint: the body's rigidity is what holds the two parts physically; the bond only says when to stop. (Today's `FractureJoint`, renamed so it is not mistaken for a physics joint.) |
| **tie** | ECS | Fracture's record that a constraint holds the compound by one of its parts: which constraint, which part. A tree's weld into the ground is a tie. |
| **compound** | ECS | An entity whose body is made of parts joined by bonds: anything that can come apart. |
| **group** | (a value) | The parts still connected through unbroken bonds, found during a split and gone after it. |
| **piece** | ECS | A group on a body of its own, with an entity of its own. Before any break, the compound is one piece. |

---

## The model

```text
  physics (no ECS, no meshes)        ECS
  ───────────────────────────        ──────────────────────────────────────────
  RigidBody                          CompoundFracture (on the piece's entity)
   └─ colliders                        graph: PartGraph (pure, no ECS)
       Collider                          parts: PartId ──▶ colliders
        ├─ shape, offset, material       bonds: (part, part, anchor, threshold)
        └─ bulk (optional)               ties:  (constraint, part)
                                         groups()
  PhysicsWorld::split_off(             load state: contact tracker, deadband,
    body, colliders,                     kicks, release, records keyed by
    constraints, traits)                 ColliderHandle
    ──▶ new body, same colliders       piece policy: PieceTraits, debris
        and handles, listed
        constraints rebound,         FractureSystem
        same motion                    loads at each bond's anchor ──▶
                                         broken bonds
                                       groups ──▶ kept: the tied one, else
                                         the heaviest
                                       every other: split_off, taking its
                                         ties ──▶ new entity, its share of
                                         the compound and the looks,
                                         PieceSplit

                                     PartLooks (component, on the piece)
                                       how each drawn collider looks

                                     PartDrawSystem
                                       changed PartLooks ──▶ ModelInstance,
                                       merged by material

                                     owners (glass, cleave, plants, terrain
                                     anchoring) react to PieceSplit
```

### What is not a compound

Nothing in the physics engine changes for compounds: a body is already a
list of colliders, a crate's list one long, and every stage (narrowphase,
mass, bulk, blasts, `split_off`) treats every body alike. A compound is an
ECS idea: an entity with a `CompoundFracture`. A crate, a barrel or a
boulder has none, keeps the model its spawnable gives it, and nothing here
touches it. An unbroken stone block is a compound of one part: it has a
`CompoundFracture` because it can cleave.

### `PartGraph`: parts, bonds and ties

The graph is a plain struct with no ECS in it, so the rules about what
breaks and what stays are tested on their own.

```rust
/// Names a part for its whole life: assigned when the part is made, never
/// reused, never an index into anything.
pub struct PartId(u32);

/// One unit of a compound that never comes apart: a branch with the crown
/// it holds, a plank, a pane cell.
pub struct Part {
    pub id: PartId,
    /// Its colliders, which keep their handles for the part's whole life,
    /// whichever body they are on.
    pub colliders: SmallVec<[ColliderHandle; 4]>,
}

pub struct Bond {
    pub a: PartId,
    pub b: PartId,
    /// Where the parts meet, in the body's frame. Moved with the body's
    /// origin whenever it moves (a re-centre, a split), as constraint
    /// anchors are.
    pub anchor: Point3<f32>,
    /// How far from the anchor a contact spike still counts fully, m:
    /// today's `JointAnchor::scale`, stated rather than guessed from the
    /// smaller child.
    pub reach: f32,
    /// Blast loading, N·s/m², past which it breaks.
    pub threshold: f32,
}

/// A constraint holding the compound by one of its parts.
pub struct Tie {
    pub constraint: ConstraintHandle,
    pub part: PartId,
}
```

A tree's branch is one part: its capsule and its crown's core pieces. Its
trunk is one part with its roots and the main crown. There are no bonds of
infinite strength holding clusters together; a cluster is a part.

(Today's `FractureJoint` doc says its threshold is in N·s; what it is
compared with is a loading per area, `systems.rs:121`. Corrected here.)

### Load is judged at the bond

A bond's blast load is the blast's loading per area at its anchor. Its
contact load is the largest spike either part's colliders took, falling off
with distance from where it landed to the anchor over the bond's reach, as
contact locality does now (`ChildLoads::contact_load`). One place for both
loads, and no rule is needed to combine a many-collider part's colliders
into one number.

Measured at the fork, a branch feels a blast by its crown less than its
crown does: the lever a crown makes of its branch is not modelled, and a
branch's threshold is tuned with that in mind. Today's rule (each endpoint's
loading at its own centre) does no better: it reads the crown's loading,
but crown or twig, the number is the same.

The contact tracker's deadband, which keeps a body's own weight from
reading as load, applies to a body free to move. A body held by ties to the
world carries its weight through them, not through its colliders' contacts,
so like a static body today (`contact_load.rs:124`) it gets none: the rule
becomes "no deadband for a static or world-tied body".

### Ties: constraints that hold the compound

Nothing in the game is a static body; a thing that must not move is a
dynamic body welded where it stands. A constraint is anchored on a body,
not on a collider, so the physics engine cannot say which part of a
compound a constraint holds it by. A tie records it.

**Why ties are needed.** Take the glass floor in the Test Arena: one body,
welded to the world along both long edges, a weld per edge cell once it has
crazed. A footstep breaks a crack right across it. Two groups, each held by
the welds along its own edge. Only one group can keep the original body;
the other moves to a new body through `split_off`. Its welds are on the
original body still, holding it by points where that half used to be, and
the half that moved, which is what they should hold, has nothing holding
it. The welds have to move with the parts they hold. Ties say which parts
those are, and `split_off` rebinds the constraints it is given onto the new
body. Each tie holds one part and a part is in exactly one group, so every
constraint has exactly one place to go.

**Which kinds can be ties.** A tie's constraint is rebound to a new body by
`ConstraintKind::rebind(from, to, shift)`: the end on `from` moves to `to`,
its anchor re-expressed in `to`'s frame. The new body starts with the old
one's rotation, so only anchors move: a `Fixed` weld's reference rotation
and a hinge's axes, held in the body's frame, are unchanged. `rebind`
supports the kinds `shift_anchor` supports (`Fixed`, `BallJoint`, `Hinge`,
`FollowPoint`); a tie of any other kind is refused when the tie is made.
Springs, sliders and suspensions as ties wait for a compound that needs one.

### Bonds that are constraints: out of scope, not shut out

Every bond in this design joins two parts on one body: the body's rigidity
holds them, and the bond is only fracture's record of when to stop. A rope
bridge's planks, or a sign hanging on springs, are parts on different
bodies held together by constraints. Those are out of scope. When they
come, they are a second kind of bond, between parts on different bodies,
which the constraint holds and which breaks when the constraint gives
(`max_impulse`, which the engine already has) rather than by fracture's
load judgement. Nothing else changes: parts and looks are keyed by collider
handle whatever body they are on, groups are still found over bonds of
either kind, and a group still becomes a piece through `split_off` and
`PieceSplit`. What keeps that door open is that nothing outside the split
itself assumes a compound is one body.

### Which group stays

The group holding ties stays on the original body and entity, so whatever
the owner put beside the ties stays with them. Without ties, the heaviest
group stays. Today's rule (most children) would leave a tree's heavy,
many-collider branch on the trunk's entity. When more than one group holds
ties (the glass floor cracked across), the heaviest of them stays and the
others move, taking their ties.

### Release

Today a compound "released" frees every child and deletes its entity: a
pane's last shards would otherwise hang in an empty frame, held by a static
body (`glass/remnant.rs`). Under this design, release removes the
compound's ties' constraints and drops every bond; the split then runs as
usual, so every part becomes a piece and the heaviest keeps the original
body. No body is ever left empty, and the shards fall cell by cell as they
do today.

### What a new piece gets

- **Its body,** from `split_off` with the compound's `PieceTraits`: damping,
  gravity scale and filter. The engine fills in pose and motion. A piece
  that takes ties keeps the old body's `ignores_static` (a tied tree trunk
  stays bedded in the ground it is welded into); one that takes none starts
  with it off.
- **Its ties,** if its parts held any, with their constraints rebound.
- **Its kick:** each of its parts' blast impulse (that part's loading times
  the area its own colliders' envelope shows, `PhysicsWorld::impulse_on`
  over a set of colliders) plus any sever kick, applied at that part's
  centre of mass. A group struck off-centre spins.
- **Its share of the compound:** the parts it took, the bonds among them,
  and every per-part record. Keyed by handle and `PartId`, a share is a
  filter, not a remap.
- **Its share of the looks** (`PartLooks`), the same way.
- **`Debris`,** if the compound sheds debris, sized by its parts' volume
  together, as debris is ranked by size today.
- **A `PieceSplit` event** (`from: Entity, piece: Entity`), after which each
  owner adds what is its own: a glass cluster's `BrittleSheet` with its
  share of the sheet's records, so it can craze again; a branch's
  `PlantPiece`; and `TerrainAnchored`'s share (below).

**`TerrainAnchored` after a split.** It lists constraints to remove when
the ground goes and calls `set_ignores_static` on its entity's body. A
system in `terrain_anchor` reacts to `PieceSplit`: the constraints that
moved with a piece, and their anchor points, go to a `TerrainAnchored` on
the piece's entity. `released_model` is not used by compounds, whose looks
are their parts'.

### How parts look: `PartLooks`

```rust
/// How each drawn collider of a piece looks, in the collider's own frame.
/// A component on the piece's entity, keyed by handle: a piece's share of
/// it is a filter, and it goes when the entity goes.
pub struct PartLooks {
    looks: FxHashMap<ColliderHandle, Arc<Look>>,
}

/// Mesh primitives, each with its own material, texture coordinates baked.
pub struct Look {
    pub primitives: Vec<MeshPrimitive>,
}
```

`Look` and `PartLooks` live in `model`, beside the mesh types: drawing is
not fracture's. A collider with no entry is not drawn (a crown's inner core
pieces). A component rather than a world-wide store: it dies with its
entity (the debris cull, any despawn) with nothing to forget, and a change
to it is a change to a component, which marks it for redrawing.

**`PartDrawSystem`** builds the model of every entity whose `PartLooks`
changed or whose body re-centred: each look placed at its collider's
current offset and merged by material into one primitive each
(`assemble_by_material` does this today; a cleaved dome is hundreds of
wedges, so merging is the default). It depends on `PartLooks` alone, not on
`CompoundFracture`. It runs after fracture, cleave, glass and the owners'
`PieceSplit` systems and before render, and it refreshes the entity's
`Position` and `Orientation` from the body, as `refresh_transform` does
now: a model rebuilt about a re-centred origin and drawn at last frame's
origin blinks sideways for a frame.

A spawnable no longer builds a compound's model; it gives its entity a
`PartLooks`. Texture coordinates are baked into each look when it is made,
so the pattern stays on the material wherever the part goes:
`texture_anchor` goes.

### Cutting makes parts

Cleaving (`cleave`) and crazing (`glass`) replace one part with several
whose shapes nobody authored. The cutter makes their colliders, their bulk
and their looks at the moment it cuts, through a `FaceMesher` the spawnable
implements and hands the cutter's own component (`BrittleSolid`,
`BrittleSheet`): how to mesh a new piece, in which texture frame, knowing
the whole it was cut from. What `PieceStyle`, `whole_shape` and
`texture_anchor` do today moves there. A new look for broken stone or ice
lives in the spawnable's file, not in `shared/models.rs`.

The cutter also passes the old part's bonds and ties on: each goes to the
new part nearest its anchor (a bond's anchor is in the body frame, which
cutting does not move), and the bonds among the new parts are the cutter's
to make, as they are today. `split_child` becomes "replace a part with
these parts".

### Owners change looks themselves

A branch broken off shows its snapped base. That is not fracture's concern:
the plants' own system reacts to `PieceSplit` and, for a branch whose parent
is no longer on its body, replaces its capsule's look with the snapped one.
The owner keeps its own map from limbs to `PartId`s.

---

## Physics changes

Two, both general.

### Bulk on the collider

```rust
ColliderDesc::capsule(h, r).bulk(bulk)
```

A collider carries, optionally, what weighs it and what fluids and blasts
see: mass parts, envelope volumes (possibly none), drag coefficient and
solidity, the same pieces `BulkShape` holds today. A collider without one
stands for itself, as now. The body sums its colliders.

Decisions:

- **Frame.** A collider's bulk volumes are in the collider's frame and are
  composed with its offset, rotation included, wherever the body reads
  them. `EnvelopePart` and `MassPartRef` therefore carry their placement by
  value, not by reference. `Volume`'s doc changes from "the body frame" to
  "the collider's frame".
- **Mass.** `Collider::mass()` and its inertia report the declared mass
  parts when there are any, the shape's otherwise: a collider's mass is
  what the body sums. No fractured compound declares bulk before step 5, so
  fracture's density-from-mass rebuild (`snapshot_child`), which goes in
  step 4, never meets a declared mass.
- **Empty envelope.** `envelope: Some(vec![])` declares a collider that
  displaces nothing and catches no blast: a tree's wood, which today's
  body-wide bulk leaves out by covering only the leaves. A body whose
  colliders show no envelope at all floats on nothing and catches nothing,
  as `systems/water.rs:270` already handles. Every tree piece holds a crown
  (crowns no longer break off), so no piece is ever bare wood.
- **Solidity over a mixed envelope.** Each envelope part carries its
  collider's solidity. A blast strikes, for each solidity value among the
  parts, that value times the union of those parts' silhouettes; the sum is
  capped at the whole union's area. A body whose parts share one solidity
  (every body today: a clump's overlapping lobes, a tree's crowns) gets
  exactly today's result, solidity times the union.
- **A declaration not shaped like its collider** (a critter's figure on its
  torso collider, a character's figure on its capsule) is re-expressed in
  that collider's frame; it then travels with that collider.
- **A loose clump's core** is one collider per core piece, each piece of
  `CorePiece::filling(lobe, core_fraction)` carrying as bulk the same piece
  of `CorePiece::filling(lobe, 1.0)` (filling yields the same pieces at any
  scale) and its share of the lobe's mass.
- **Colliders re-attached from a description** carry their bulk in
  `ColliderDesc`. The paths that rebuild a collider from its shape
  (`collider_desc_from_shape`, `ChildSubstance`) do not, and do not need
  to: nothing they rebuild declares bulk, and both go in step 4.

Removed: `RigidBodyDesc::bulk`, the body's `bulk`, `bulk_mut`,
`clear_bulk`, `BulkShape::shift`, and from the plants branch `set_bulk` and
`Joinery::declare_bulk`. `BulkShape` stays: a collider's bulk is one, and
`LeafCore::bulk` still builds one for `BulkShape::blast` until step 5.

Declarations to move:

| Where | Today | Becomes |
|---|---|---|
| player (`spawners/player.rs`) | `figure` | on its capsule |
| barrel | hollow shell (two declarations) | on its hull |
| heart critter, peeper | `fluff`, `stilts_and_eye` | on the torso collider |
| loose clump (`LeafCore::spawn`) | full lobes | each core piece its full-size piece |
| tree (`plants/bodies.rs`) | crowns' lobes, re-declared after a break | each crown's core pieces as a clump's; wood an empty envelope; no re-declaring |
| tests: `physics/bulk.rs`, `impulses.rs`, `atmosphere.rs` | body bulk | collider bulk |
| `CLAUDE.md` "Bodies vs their bulk" | body | collider |

### `split_off`

```rust
/// What a body split off another is, beyond where it is and how it moves.
pub struct BodyTraits {
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub filter: CollisionFilter,
    pub ignores_static: bool,
}

/// Moves `colliders` off `body` onto a new body with `traits`, unchanged and
/// keeping their handles, placed and moving as they were, and rebinds
/// `constraints` with them. Both bodies re-centre on what they hold.
pub fn split_off(
    &mut self,
    body: RigidBodyHandle,
    colliders: &[ColliderHandle],
    constraints: &[ConstraintHandle],
    traits: BodyTraits,
) -> Option<Split>

/// What a split did: the new body, and how each body's frame moved, for
/// whoever keeps points in those frames (bond anchors).
pub struct Split {
    pub body: RigidBodyHandle,
    /// How far the kept body's origin moved, in its own frame.
    pub kept_moved: Vector3<f32>,
    /// The new body's frame in the old body's frame as it was.
    pub new_frame: Isometry3<f32>,
}
```

- **Arguments.** `None`, with nothing changed, if `colliders` is empty, is
  every collider of `body`, or names a collider not on `body`; a constraint
  listed that does not reference `body`, or of a kind `rebind` does not
  support, is refused the same way.
- **Contact caches.** The moved colliders' cached manifolds are purged, as
  `detach_collider` does: warm-starting a new pair of bodies from contacts
  the old one made is wrong.
- **Sleep.** Both bodies are woken.
- **Epochs.** The pose epoch and every cache keyed on a body's colliders
  (drag areas, the broadphase) are invalidated for both.
- **Overlap.** Parts that overlapped within one body (a branch inside the
  trunk at its fork) start overlapping as two. Measured on the plants
  branch, a branch broken off at its fork is pushed out at 0.23 m/s, not
  flung. Flush faces cut by a cutter keep the hairline gap they have today
  (`CLEAVE_INSET`), since a body that starts a sweep touching another is
  clamped by CCD.

Shape, material, filter, bulk and handle survive the move, and so does each
rebound constraint's handle.

---

## What each compound becomes

| Compound | Parts | Ties | Owner reacts to `PieceSplit` |
|---|---|---|---|
| plank bridge | each beam, each plank | none | no |
| table | top, each leg | none | no |
| voussoir arch | each voussoir | none | no |
| stone block | the block, then its wedges | none | cleave: `BrittleSolid` share |
| ice block, igloo | each block, then wedges | none | cleave |
| glass sheet, held (`fixed` today) | the pane, then its cells | a weld to the world per held edge; after crazing, per edge cell | glass: `BrittleSheet` share |
| glass sheet, loose | the pane, then its cells | none | glass |
| framed window | frame, pane, then cells | none today | glass |
| tree | trunk with roots and main crown; each branch with its crown | the trunk's weld into the ground | plants: `PlantPiece`, `Foliage`, snapped look; terrain anchoring |

---

## Behaviour that changes

- **A group still joined comes off as one body.** Two planks joined to each
  other fly as one; a window's shards still joined stay a cluster. This is
  what the bonds say, but it changes what players see. The physics bench,
  the rubble and level scenario suites and the glass and cleave tests are
  the check; tests that count pieces get their counts rethought, not
  loosened.
- **A piece is told what body it is,** rather than given fixed damping.
- **The kept group is the tied or heaviest one,** not the one with most
  children, and ties follow their parts.
- **No static compounds.** A held glass sheet (`fixed` today) is a dynamic
  body welded to the world along its held edges, not a static body. The
  Test Arena's upright pane, which hung in mid-air by it, lies on the ground
  instead; its glass floor, which spans a drop on purpose, is welded along
  its long edges.
- **Bonds break by the loads at their anchors,** blast and contact alike.
- **Capsule and sphere children are drawn and culled** like any other.
- **Pieces look exactly as they did whole,** with no redraw from shapes.
- **Branches break under landings** as well as blasts.

---

## Migration

Each step leaves the game working and is merged on its own.

1. **Bulk on the collider.** Everything under "Bulk on the collider" above:
   collider bulk and its frame, collider mass, empty envelopes, the
   solidity rule, every declaration moved, body bulk and the plants'
   `set_bulk` and `declare_bulk` deleted. `BulkShape::blast` stays until
   step 5. Buoyancy, drag and blast numbers unchanged for every body: the
   water, foliage and plants scenarios and the physics tests are the check.
2. **`split_off` and `rebind`.** Tests: handles kept, motion and pose kept,
   both bodies re-centred and the shifts reported, listed constraints
   rebound with their handles and the rest left, the refusals, caches
   purged, both bodies woken. Nothing uses it yet.
3. **Re-key by handle and `PartId`.** `CompoundFracture`'s joints and
   records, `ContactLoadTracker`, `BreakDepths`, `FatigueTracker`, and the
   glass and cleave components' records; `remap_children` and every
   `reindex` deleted. Behaviour unchanged: tests change only where they
   build joints by index.
4. **Parts, looks and drawing.** `PartGraph`, bonds with anchors, ties,
   loads at anchors, the kept-group rule, release, groups split whole
   through `split_off`, `PieceTraits`, `PieceSplit`, `PartLooks` and
   `PartDrawSystem`, `TerrainAnchored`'s share. Every compound moves at
   once, since after step 3 they share one code path: box compounds give
   their entities `PartLooks` at spawn; cleave and glass make looks through
   `FaceMesher` as they cut and react to `PieceSplit`. Held glass sheets
   become world-welded, with the deadband rule for tied bodies; the Test
   Arena's upright pane is laid on the ground and its floor welded along
   its edges. Delete `PieceStyle` from fracture, `compound_model_of`,
   `freed_piece_model`, `collider_desc_from_shape`, `materials`, `style`,
   `texture_anchor`, `whole_shape`, `model_stale`, the glass sheet's static
   body, and fracture's import of `spawnables`. Update the spawnables
   README, the `weathered_stone` bench scene and the cleave tests that read
   `texture_anchor`.
5. **Trees onto it.** Trees declare a `CompoundFracture`; the plants react
   to `PieceSplit`. Delete `BranchBreakSystem`, the break path of
   `PlantPieceBody`, `LeafCore::bulk`'s use there and `BulkShape::blast`.
   Calibrate a branch's bond in per-area units (today's 250 N·s on the
   crowns) and its contact threshold against a felled tree landing on its
   crowns.

Step 4 is the large one. Doing every compound at once avoids two fracture
code paths side by side; step 3 is what makes that safe, by landing the
index-to-handle change with no behaviour change first. Static panes go in
step 4, not earlier, because a held pane needs ties to stay up.

---

## Out of scope

- **Bonds that are constraints** (rope bridges, things on springs): see
  "Bonds that are constraints" above.
- **Ties of kinds `rebind` does not support** (springs, sliders,
  suspensions): refused until a compound needs one.
- **The level format.** Adding a spawnable still means a `LevelObject`
  variant repeating its Def's fields and several match arms in
  `level/data.rs`. Fixing that (variants wrapping the Def, per-object facts
  as `Spawnable` methods) is its own branch.
