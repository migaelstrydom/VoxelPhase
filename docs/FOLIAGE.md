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
   core        ──  colliders: a sphere per round lobe, a capsule along each
                   long axis of a stretched one, 80% of its size
```

| Real physics | Drawn, or a rule |
|---|---|
| The core's spheres: a dynamic body once loose | The nubbly skin, built once from the lobes and drawn as grown |
| Leaves dragging whatever is in a lobe (`FoliageDrag`) | Leaves shed on landings, uprooting and by whatever moves through a clump, more per metre the faster it goes (particles, `rustle.rs`); a trail of them behind a clump flying faster than 4 m/s (`trail.rs`); at most 300 alive at once |
| The world's air dragging a loose clump through its whole lobes (its bulk) | A wading character's shorter stride (`leaf_depth`) |
| A blast's shove on a freshly uprooted core | |

A rooted shrub has no body at all. Nothing collides with it; it is a drag
volume and a mesh. A blast within 60% of its knockback radius gives it a core
(`FoliageUprootSystem`), and the shove the explosion queued throws that core
on the next physics step. A tree's crowns are clumps riding on its limbs'
bodies (`Rooting::Loose` with an offset, `src/plants/`), each limb's body
carrying its crown's core.

## Why each piece is shaped the way it is

- **A cluster of spheres, not one ball.** The engine's friction has no
  rolling resistance, so a single sphere rolls off every slope. A lumpy
  cluster tumbles like a sack and stops. On a 25° slope a clump stops at the
  foot, while a ball of the same mass and size rolls 5 m or more past it
  (`a_clump_tumbles_down_a_slope_and_stops_where_a_ball_rolls_on`).
- **The world's air, through the clump's whole lobes.** A loose clump's
  core declares its lobes as its bulk (`LeafCore::bulk`), dragging as a
  porous bush (`LeafCore::drag_coefficient`, 1.0, about twice a smooth
  ball's), so the engine's air (`physics::Atmosphere`) and water see a ball
  of leaves, not the core. A blast strikes a body by the area it shows it,
  but leaves let its shock front through (`LeafCore::solidity`, 0.3:
  `BulkShape::porous`), so the game's grenade 1.5 m from a clump throws it
  at about 28 m/s, where a solid clump would leave at 95, and it comes to
  rest about 32 m away.
- **Cores are in their own collision layer** (`collision_layers::FOLIAGE`),
  which everything meets, a grenade and its predicted flight included: a
  grenade passes through a rooted shrub, which has no core, slowed by the
  leaves, and strikes a loose clump's core as it would any body. A tree's
  limbs and crowns are one body, so they never meet each other until a
  branch breaks off (`docs/PLANTS_DESIGN.md`).
- **Leaves are a fluid.** They drag by the law the air and water drag by
  (`DragArea::exposed`), at a quarter of water's density (250 kg/m³): a
  body by the silhouette of its own parts, each by how much of it is in the
  leaves, measured at points spread through it. A plank sliding past a
  shrub is not dragged until it is in the leaves. A grenade thrown at
  12 m/s through a shrub comes out at about 10.8 m/s.
- **The drag acts on both sides.** A body in a loose clump's leaves and the
  clump are each pulled towards their shared, momentum-weighted velocity,
  so a thrown clump nudges what it passes and slows, and cannot haul a
  heavier body along (`a_thrown_clump_does_not_haul_a_heavier_body`). A
  rooted shrub is immovable. Only the linear drag is shared: the leaves'
  pull on a body's spin has no reaction.
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
- **The skin does not bend.** Leaves pass through whatever wades through
  them, and a resting clump's skin sinks into what it rests on by a fifth
  of a lobe's radius: the core is 80% of each lobe, as large as it can be
  while the cluster still tumbles to a stop rather than rolling. Bending the
  skin (pressing, squashing, parting) is designed in
  `docs/PLANTS_DESIGN.md` and parked.
- **Lobes are ellipsoids.** A shrub given a `height` (`Shrub(..., height:
  Some(2.8))`) is grown and then stretched upright to it: tall lobes make a
  cypress column, flat ones a mound. Skin vertices and bumps are sized in
  metres, so a tall lobe's nubs are the size of a round one's. The Test
  Arena has two columns and a mound east of the big shrub.
- **Loose clumps spawn at rest.** A loose clump's resting pose is computed
  from its core (`LeafCore::resting_pose`): the least of `max(c·d + r)` over
  down directions `d`, which is the face of the sphere cluster it settles
  onto. Shrubs cannot be `Dropped`:
  they take their height from the terrain.

## Measured (foliage_scenarios, character_viewer `shrub_*`)

- Dropped from 1 m: no rebound, asleep at 1.9 s.
- 25° and 35° slopes: stops within 1 m of the foot; asleep within 6 s.
- Kicked at 4 m/s on flat ground: stops after about 1.2 m.
- A grenade 1.5 m away: thrown at about 28 m/s, resting about 32 m away.
- A grenade thrown at 12 m/s through a rooted shrub: through at about
  10.8 m/s.
- In game, a big rooted shrub torn loose by a grenade beside it rose 3.4 m,
  landed 6.5 m away and stopped dead.

## Open

- **Leaf look.** The bumps are geometry; the leaves are shading
  (`SurfaceSource::FOLIAGE`, `shader/foliage.glsl`): a normal tilted leaf by
  leaf from noise in the clump's frame, diffuse wrapped past the terminator,
  and the sun through the edges of a backlit clump. Tuned by eye on two
  views; its constants head the shader. Shed leaves are leaf-shaped
  particles that flutter (`Silhouette::Leaf`).
- **The traversal planner's casts see cores.** Right while characters meet
  them; revisit if a clump on the path makes the plan hop it.
- **Trees** are the next step: `docs/PLANTS_DESIGN.md`.
