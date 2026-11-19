# World and story

What the game is about, who the player is, and what a level asks of them.
How it looks is in [VISUAL_DIRECTION.md](VISUAL_DIRECTION.md), and how
rooms play as puzzles in [GAME_DESIGN.md](GAME_DESIGN.md).

The first part records what has been decided. The second part lists ideas
that were proposed but not adopted.

## Decided

### The premise

The world is a mad Bosch garden: organic, hybrid, fleshy, teeming. The hero
brings logic and reason to it. Reason is represented by classical Greek
architecture, which looks more realistic than the painterly garden around
it.

The hero is simply right. It's a platformer, and the game doesn't question
whether order should win.

### Telling it

Wordless. No text, narration or dialogue in levels: the story is told
through the environment and the hero's actions.

### The goal of a level

Every level holds the five Platonic solids: tetrahedron, cube, octahedron,
icosahedron and dodecahedron. They glow and float. **All five must be
collected to complete a level.** Then the hero reaches the level's Greek
temple.

In code, the solids replace the gem collectible (`src/objective/`): the
temple is a `Goal` with `required_gems: 5`.

Collectibles where only some are needed may come later. None exist yet.

### The objective is not part of the room

GAME_DESIGN's rule that nothing acts at a distance governs how a room
changes. The objective (solids, temple, brazier) is the game's score, kept
outside the room's physics:

- Taking a solid opens, moves and builds nothing in the room.
- The temple stands whole from the start, where it can be seen from across
  the level. Arriving there before all five solids are in does nothing.
- The temple is as breakable as everything else. If the hero breaks it, or
  lets it be broken (a breached dam washing it away), the level can no
  longer be won and must be restarted. Actions have consequences.
- A brazier is lit by arriving; it does nothing to the room.

So the temple is not an exception to the rule: nothing it does is a
mechanism a puzzle could use.

### Challenge rooms

Besides full levels there are challenge rooms with a single goal: a bronze
tripod brazier, somewhere the room makes hard to reach. The hero lights it
with his lantern, and that completes the room. An unlit bowl on a ledge reads
as the goal without words, and its flare reads as success. The prototype
rooms in `levels/rooms/` mark it with a bare `Goal` for now.

### The hero

A bearded philosopher in white robes. He throws *grenades of
enlightenment*: the game's grenade, in its role in the story.

He carries a lantern in one hand and throws with the other. He puts the
lantern away whenever he needs both hands, e.g. to climb, hang from a ledge
or swim. The hand's job (holding the lantern, throwing, gripping) is an arm
role read from `LocomotionState`, like the rest of the pose
([LOCOMOTION_STATES.md](LOCOMOTION_STATES.md)).

Things to settle when the lantern is built:

- **Which hand:** lantern in the left, throw with the right.
- **Where it is put away:** it must stay visible, or it seems to vanish.
  A hook on the belt is the obvious place.
- **Carrying objects:** whether carrying (GAME_DESIGN's *Carry*) takes one
  hand or two decides whether the lantern stays out.
- **A steady light:** if the light follows the hand exactly, every shadow
  in the scene wobbles with the arm's swing. Keep the lantern level in a
  swinging hand, and give the light, and the flame, a little lag.
- **Under water:** it goes out and relights when he comes ashore, or it
  just stays put away while he swims.

## Proposed, not adopted

### For the hero

- **A knee-length chiton** rather than full-length robes. It keeps the legs
  readable for foot placement, ledge grabs and swimming, and the hem can
  swing without cloth simulation.
- **Grenades that look like reason**, not like grenades: a small glowing
  tetrahedron, a sealed scroll, a clay amphora. The burst flashes
  white-gold geometric light against the garden's organic colours.
- **The lantern as a gameplay cue:** it brightens near a solid, sputters in
  water and lights the dark caves where solids are hidden.

### For the solids

- **Plato's elements give each solid its own challenge.** In the *Timaeus*,
  each solid is an element. Then collecting five in every level isn't a
  checklist:

  | Solid | Element | Behaviour | Challenge |
  |---|---|---|---|
  | Tetrahedron | Fire | rises, flickers | sealed inside something only a blast opens |
  | Cube | Earth | heavy, low, half-buried | dig it out, or bring a slope down to reach it |
  | Octahedron | Air | high, rides currents | traversal: ledges, hops, the hover sled |
  | Icosahedron | Water | bobs, drifts with the current | drain, divert or follow the water |
  | Dodecahedron | Cosmos | appears only once the other four are taken | the level's finale |

- **Bosch's containers hold the solids.** The painting is full of things
  people are trapped in: glass spheres, bubbles, mussel shells, giant
  strawberries and cherries, hollow egg-men. A solid glowing inside a
  translucent fruit reads at a glance and asks for a grenade.

### For progress and structure

- **Progress shown in the world, not in a HUD counter.** Each solid taken
  is seen to fly to the temple. The armillary sphere (below) is one place
  for it to land. The temple itself doesn't change, because of the rule
  above.
- **The closed triptych as title or hub.** The panel's outer shutters show
  the world on the third day of creation, in grey, inside a glass sphere.
  Opening the shutters starts the game.
- **Three panels as three acts:** Eden, the Garden, Hell, getting harder and
  stranger.

### The armillary sphere

An armillary sphere (nested rings that model the heavens) is wanted
somewhere. Two places it could go:

- **At the heart of each temple, holding the solids.** Kepler's
  *Mysterium Cosmographicum* (1596) nests the five Platonic solids between
  the spheres of the six planets: Saturn, cube, Jupiter, tetrahedron, Mars,
  dodecahedron, Earth, icosahedron, Venus, octahedron, Mercury. Each solid
  taken flies to the temple and settles into its place among the rings. The
  dodecahedron completes it, and the sphere starts turning. The solids get a
  destination, and the level's finale an image.
- **In the hub, as the whole game's progress.** The world in a glass sphere
  on the triptych's closed shutters gains a ring for each act completed, and
  turns once the game is finished. Lit braziers from the challenge rooms
  could hang on it as stars.

### Other collectibles

- **Pages of Euclid's *Elements*.** Loose pages hover and flutter in the
  air, each showing a geometric figure (the Pythagorean square, a bisected
  angle, a circle through three points), so no text is needed to read them.
  These would be the optional collectibles, where only some are needed or
  none are required. The *Elements* has thirteen books, which gives a
  natural number of sets to complete.

### Motifs

- **The owl.** It appears throughout Bosch's work, usually as an ill omen,
  and it is Athena's bird of wisdom. An owl whose role, guide or watcher, is
  never clear ties the two worlds together.
- **Creatures as agents of chaos:** hybrids that carry, swallow or hoard
  solids, e.g. a nest of stolen geometry.
