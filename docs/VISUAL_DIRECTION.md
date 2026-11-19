# Visual direction

What the game should look like, and the rules learned getting there. Why the
world looks this way is in [WORLD_AND_STORY.md](WORLD_AND_STORY.md);
lighting infrastructure (shadows, occlusion) is in
[LIGHTING_PLAN.md](LIGHTING_PLAN.md).

## The target look

Hieronymus Bosch's *Garden of Earthly Delights*, come to life. The central
panel is a bright, crowded garden under a high horizon: pale meadows, lakes
and pools, fantastical pink and blue structures, outsized fruit and birds,
glassy spheres and bubbles, and a fountain of life. The register is
painterly and strange, and its colours are clear rather than gloomy.

The world has two registers, and the contrast between them tells the story:

| | The garden | Reason |
|---|---|---|
| What | Terrain, plants, water, creatures, fruit, Bosch's structures | Greek architecture, the hero, his lantern and grenades, the Platonic solids, the braziers |
| Look | Painterly: sculpted, soft, organic, warm | More realistic than painterly: honed marble, true proportions, crisp edges |

No level is precious. Serious level design waits on the look and on engine
features, so any level may be reshaped or rebuilt to serve it.

## Checking the look

`visual_bench <scene>` renders bench scenes headlessly with the real
shaders. Scenes must be deterministic (fixed camera, sun and geometry) or
before/after comparisons are worthless; sweep scenes render a parameter
ladder as one image, which is how values get chosen. `level_viewer` shows a
whole level lit by its own daylight. A bench has flattered a look before, so
ask for a game screenshot before believing one is fixed.

An earlier target, a crisp plastic look, and the history of what landed for
it are in git history (`git show 2c9d98a:docs/VISUAL_DIRECTION.md`).

## Open

- **How painterly is the garden?** The direction is set; how far the
  shading departs from physically based rendering is not.
- **Greek architecture in the garden.** Realistic is decided. How marble
  sits under the garden's warm light, and whether it ever takes on the
  garden's colours, needs prototypes.
