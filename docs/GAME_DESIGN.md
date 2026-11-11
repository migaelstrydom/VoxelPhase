# Game design: rules and puzzle ideas

A first draft, to be marked up. It is a hypothesis: nothing here has been
played. Ideas marked **(build)** need something the engine does not have yet;
the list of those is at the end.

## The premise

An environmental puzzle game in a Bosch garden. The player does not fight the
garden's creatures; they rearrange the garden. Each room has an exit that
cannot be reached as the room stands, and the means to change the room.

## The one law

**Things only go down.** Earth falls, water runs downhill, a blast only
removes. Height is the currency: of the player, of objects, and of water. A
room is hard because the exit is up and everything the player can do cheaply
leads down.

## Rules

What the player can do:

| Verb | What it does |
|---|---|
| Move | Walk, jump, grab and climb ledges, swim |
| Blast | Throw a grenade. It digs soft ground, breaks what is breakable, and shoves loose things |
| Push | Shove a loose object by walking into it or blasting beside it |
| Ride | Stand on something that moves: a platform, a raft, a sled, a trampoline |
| Bait | Be seen by a creature, so that it comes, throws or snaps |
| Reset | At the room's shrine, put the room back as it began **(build)** |

What the world does back:

1. **Soft ground digs; carved stone and bedrock do not.** Stone and bronze are
   the room's fixed walls and its machinery.
2. **Water is conserved and seeks its level.** It leaves a breached pool and
   collects in whatever is lower.
3. **Light things float; heavy things sink.** A floating thing rises and falls
   with the water under it.
4. **Loose things obey physics.** They stack, topple, roll, block and weigh.
5. **Only lifters give height back:** trampolines, moving platforms, pumps and
   springs, rising water, the hover sled. A room has a fixed few.
6. **Creatures follow their nature, always.** A roller rolls at whoever it
   sees; a piper throws at them; a bell-carrier walks at them and cannot be
   hurt; a mussel shuts on whatever steps in.
7. **Deep ground is wet.** A tunnel dug below the water table floods, so
   digging under a room is not a way round it **(build)**.

What is scarce, and so what a puzzle spends:

- **Height:** once a ledge is gone or a pool is drained, it stays that way.
- **Water:** one reservoir fills one thing.
- **Blast:** a room gives a few grenades.
- **Lifters:** few, and each can be in one place.
- **Safety:** ground a creature is crossing is not ground the player can use.

## Puzzle ideas by cluster

Each idea is filed under what the player mainly manipulates, and tagged with
anything else it leans on. Kind is **T** (teaches a rule alone), **X** (tests
it with a complication) or **C** (combines clusters). Spends is the scarce
thing the puzzle turns on.

### Earth

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| E1 | A dirt bank blocks the path, with grenades lying in front of it. Blast through. | T | | Blast |
| E2 | Two banks, one earth and one carved stone. Only the earth gives; the way on is behind it. | T | | Blast |
| E3 | The exit is on a ledge too high to grab. Blast the overhang above so its rubble makes a ramp. | X | | Height |
| E4 | Three grenades, four banks. One bank hides nothing; read the room before spending. | X | | Blast |
| E5 | The only path up is an earth stair beside a tempting bank. Blasting the bank drops the stair: the room teaches that a blast can cost height. | X | | Height |
| E6 | Dig down to pass under a stone wall, and meet water. The tunnel has to stay shallow and come up early. | X | Water | Height |

### Water

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| W1 | A pool sits above a dry hollow that bars the way. Breach the pool's bank; the hollow fills and can be swum. | T | Earth | Water |
| W2 | The reverse: a flooded passage too long to swim. Breach its low side to drain it. | T | Earth | Water |
| W3 | One pool, two hollows, and a ridge between them. Which bank is breached decides which fills. | X | Earth | Water |
| W4 | A stream runs past the place water is wanted. Blast a channel to turn it, or drop earth to dam it. | X | Earth | Blast |
| W5 | A pump raises water as long as it is fed. Keep its basin from draining while using what it lifts. | X | Machines | Water |
| W6 | Fill a shaft from a spring to swim up to the top of it. | C | Body | Water |

### Objects

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| O1 | A crate and a ledge just out of reach. Push the crate under it. | T | | |
| O2 | A crate floats. Flood the hollow it sits in and it comes up to the far ledge with the player on it. | C | Water | Water |
| O3 | A trampoline is in the wrong place. Push it to the right one. | T | | Lifters |
| O4 | A trampoline sits in a dry basin across a gap. Fill the basin and it floats over. | C | Water | Lifters |
| O5 | A stone ball rests on a slope behind a wedge of earth. Blast the wedge; the ball rolls and breaks what it meets. | C | Earth | Blast |
| O6 | A seesaw with a heavy thing on the far end. Drop onto the near end from height to throw it. | X | Body | Height |

### Machines

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| M1 | A moving platform crosses a gap. Ride it. | T | | |
| M2 | A platform's track runs behind an earth bank that stops it short. Clear the bank and it completes its run. | C | Earth | Blast |
| M3 | Two platforms out of step. Cross both by timing the change. | X | Body | |
| M4 | A platform passes under a ledge with a crate on it. Push the crate so the platform carries it where it is needed. | C | Objects | Lifters |
| M5 | The hover sled crosses water but not a rise. Lower the rise or raise the water. | C | Earth, Water | Height |

### Creatures

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| C1 | A roller in a pen, seen through a gap, with a soft wall between it and the exit. Stand behind the wall. | T | | Safety |
| C2 | A roller patrols the hollow the player must cross. Crater its path and it is stranded in the hole. | C | Earth | Blast |
| C3 | The same hollow with no grenades and a pool above it. Flood it. | C | Water | Water |
| C4 | A bell-carrier follows, slowly. Lead it onto a float, a seesaw or a weak earth bridge: it is the heaviest thing in the room. | C | Objects | Safety |
| C5 | A piper on a rock and no grenades anywhere. Stand where its throw should land. | C | Earth | Safety |
| C6 | A mussel beside a path a stalker walks. Draw the stalker across it. | X | | Safety |
| C7 | Egg-walkers are light and quick: they cross a float that will not bear the player, and trip what the player cannot reach. | C | Objects | |

### Body

The acrobatic layer. Mostly a way to make another cluster's room harder, with
a few rooms of its own early on to teach the moves.

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| B1 | A wall with a lip. Jump, catch it, climb. | T | | |
| B2 | Shimmy along a ledge past a gap in the floor. | T | | |
| B3 | Swim under a low arch and come up the far side. | T | Water | |
| B4 | Any earlier room again with the footing narrowed, the water rising, or a creature loose. | X | any | Safety |

## Order

Clusters are taught in the order their rules depend on each other. Each
chapter teaches its cluster, tests it, then combines it with what came before.

```text
  Body ──► Earth ──► Water ──► Objects ──► Machines ──► Creatures
  move     blasting  blasting  floating    platforms    everything
           digs      moves     needs       carry        is something
                     water     water       objects      to aim them at
```

A first pass at the opening, one new idea per room, with an easier room after
each hard one:

| Room | Idea | Why here |
|---|---|---|
| 1 | B1 | Move, alone, nothing to fail |
| 2 | E1 | The grenade, alone |
| 3 | E2 | What it cannot cut |
| 4 | E3 | First real thought: blast to gain height |
| 5 | B2 | Rest |
| 6 | W1 | Blast moves water |
| 7 | E5 | Blast can cost height: the reset is taught here |
| 8 | W3 | First room with a wrong answer that looks right |
| 9 | O1 | Rest; pushing |
| 10 | O2 | First combination: water lifts an object |
| 11 | C1 | First creature, behind a wall |
| 12 | C3 | Finale: water against a roller |

## What has to be built

| Need | For | State |
|---|---|---|
| Death and respawn | Any room with a creature | Death exists, switched off; no respawn |
| Room reset at a shrine | Every room, since actions are irreversible | Not built |
| Grenades as a limited pickup | E4 and the Blast economy | Grenades are free |
| Room exit | Every room | A `Goal` object exists; untested as a room exit |
| Flooding below a water table | Rule 7 | Not built; bedrock exists as a stopgap floor |
| Carved stone that blasts cannot break | Rule 1 | Bedrock is blast-proof; the temple and fountain are breakable by design, so room hardware needs a blast-proof kind |
| Carrying an object | Would widen Objects a lot | Only a plan (`GRAB_SYSTEM_PLAN.md`); the ideas above use pushing only |
| Weight triggers (a plate that opens something) | Would widen C4, O-ideas | Not built; the ideas above avoid it |

## Open questions

1. **Is resetting a room free?** Free keeps experiments cheap; a cost makes
   the player think first. Draft assumes free.
2. **Do creatures reset with the room?** Draft assumes yes.
3. **Can the player be hurt by their own blast?** It changes how E-rooms feel.
4. **Carrying.** Pushing alone is clumsy for a trampoline on rough ground. It
   may be worth building before the Objects chapter is designed in detail.
5. **How big is a room?** Draft assumes one screen or two: the whole problem
   visible from the shrine.
