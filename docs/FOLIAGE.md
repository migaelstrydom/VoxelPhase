# Foliage

Soft greenery for the Garden-of-Earthly-Delights look: topiary shrubs and
loose clumps of leaves you can wade through, blow loose and kick about. The
code is `src/foliage/`; the play-test level is
`levels/shrub_garden.level.ron`, and the Test Arena has a corner of them
beside the hover sled and the plank bridge, among everything else.

## The idea: a firm core inside a soft skin

Nothing in the engine is soft. A clump of foliage is a small rigid **core**,
which physics sees, inside a **skin** of leaves, which only drag and drawing
see.

```
visible lobes  ──  drag volume (rides with the clump)
   core        ──  colliders: one sphere per lobe, 60% of its radius
```

| Real physics | Drawn, or a rule |
|---|---|
| The core's spheres: a dynamic body once loose | The nubbly skin, built once from the lobes |
| Leaves dragging whatever overlaps a lobe (`FoliageDrag`) | The skin pressed onto what the core rests on (`Press`) |
| The world's air dragging a loose clump through its whole lobes (its bulk) | A squash spring kicked by each landing (`Squash`) |
| A blast's shove on a freshly uprooted core | Leaves shed on landings and uprooting (particles) |
| | Leaves parted round whatever pushes through them (`Parting`) |
| | A wading character's shorter stride (`leaf_depth`) |

A rooted shrub has no body at all. Nothing collides with it; it is a drag
volume and a mesh. A blast within 60% of its knockback radius gives it a core
(`FoliageUprootSystem`), and the shove the explosion queued throws that core
on the next physics step.

## Why each piece is shaped the way it is

- **A cluster of spheres, not one ball.** The engine's friction has no
  rolling resistance, so a single sphere rolls off every slope. A lumpy
  cluster tumbles like a sack and stops. On a 25° slope a clump stops at the
  foot, while a ball of the same mass and size rolls 5 m or more past it
  (`a_clump_tumbles_down_a_slope_and_stops_where_a_ball_rolls_on`).
- **The world's air, through the clump's whole lobes.** A loose clump's
  core declares its lobes as its bulk (`LeafCore::bulk`), so the engine's
  air (`physics::Atmosphere`) and water see a ball of leaves, not the core.
  A blast's shove is an impulse whatever the mass, so a clump 1.5 m from a
  grenade leaves at about 60 m/s; honest air brings it down about 50 m away
  (without air: hundreds). Shorter needs a blast that pushes by a body's
  exposed area, which would change every blast in the game.
- **Cores are in their own collision layer** (`collision_layers::FOLIAGE`).
  A grenade does not meet it: it passes through a loose clump slowed by the
  leaves, as through a rooted shrub, and its predicted flight looks past
  cores too. Characters and everything else still meet a core.
- **Drag is quadratic and proportional to cross-section.** It is the same
  law as water's drag, at a quarter of water's density (120 kg/m³). A grenade
  thrown at 12 m/s through a shrub comes out at about 9.6 m/s.
- **A stride rule on top of the drag.** The player's drive pushes five times
  harder through its feet than friction allows (`Actuator::drive_gain`).
  That is enough to walk through any honest leaf drag at full speed: the
  drag alone took 1% off a walk. So a character also measures how deep it
  stands in leaves, as it measures water, and its stride shortens by up to
  90% of that depth. Through the middle of a hedge shrub a 5 m/s walk drops
  to about 2.7 m/s.
- **Force providers accumulate.** A body can be in water and in leaves at
  once. `PhysicsWorld` now sums every provider's force on a body and
  combines their media (`MediumDrag::combined`, exact for linear drags). A
  body only one provider affects gets that provider's output bit for bit.
- **Pressed, not collided.** Where the skin would pass into the ground it is
  eased onto the ground's plane, with a soft knee, and its normals turned to
  face the ground. Contact planes are held for 0.2 s, because contacts
  flicker, and they are frozen while the clump sleeps, so a sleeping clump
  stays pressed.
- **Loose clumps spawn at rest.** A loose clump's resting pose is computed
  from its core (`LeafCore::resting_pose`): the least of `max(c·d + r)` over
  down directions `d`, which is the face of the sphere cluster it settles
  onto. It is drawn pressed from the first frame. Shrubs cannot be `Dropped`:
  they take their height from the terrain.

## Measured (foliage_scenarios, character_viewer `shrub_*`)

- Dropped from 1 m: no rebound, asleep at 1.9 s.
- 25° and 35° slopes: stops within 1 m of the foot; asleep within 6 s.
- Kicked at 4 m/s on flat ground: stops after about 1.2 m.
- A grenade 1.5 m away: thrown about 50 m.
- A grenade thrown at 12 m/s through a loose clump: through at 9.6 m/s, the
  clump untouched.
- In game, a big rooted shrub torn loose by a grenade beside it rose 3.4 m,
  landed 6.5 m away and stopped dead.

## Open

- **No reaction on the leaves.** Something pushing through a loose clump's
  leaves is dragged but does not drag the clump; only its core is ever
  pushed. Leaves part round a passer, to its capsule plus 10 cm, and ease
  back over 0.2 s, but the parted skin is smooth where it hugs the capsule.
- **Leaf look.** The leaves are geometry and vertex colour on a matte
  material. There is no translucency or wrap lighting, which is most of what
  makes sunlit foliage glow at its edges.
- **Blast distance.** About 50 m for a clump right by a grenade (see
  above).
- **The traversal planner's casts see cores.** Right while characters meet
  them; revisit if a clump on the path makes the plan hop it.
- **Upload while moving.** A still clump is a `ModelInstance`, uploaded
  once. A moving clump's skin (about 6–10k vertices) is a `ProceduralMesh`,
  rebuilt and uploaded every frame it moves. Fine for a few tumbling at
  once.
- **Trees** are the next step. The trunk-and-limb compound with fracture
  joints and the canopy as clumps were discussed but not built.
