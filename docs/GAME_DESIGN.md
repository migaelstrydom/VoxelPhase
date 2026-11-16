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
| Carry | Grab a loose object, which holds it to the player until let go |
| Ride | Stand on something that moves: a platform, a raft, a sled, a trampoline |
| Bait | Be seen by a creature, so that it comes, throws or snaps |
| Reset | At the room's shrine, put the room back as it began **(build)** |

What the world does back:

1. **Everything breaks.** Ground digs, stonework cleaves, glass shatters,
   trees snap. Nothing is protected by being unbreakable: what protects a
   thing is that the player needs it whole.
2. **Water is conserved and seeks its level.** It leaves a breached pool and
   collects in whatever is lower.
3. **Light things float; heavy things sink.** A floating thing rises and falls
   with the water under it.
4. **Loose things obey physics, and only physics.** They stack, topple, roll,
   block, float and weigh. Nothing acts at a distance: there are no plates
   that open doors. A weight does its work where it rests.
5. **Only lifters give height back:** trampolines, moving platforms, pumps and
   springs, rising water, the hover sled. A room has a fixed few.
6. **Creatures follow their nature, always.** A roller rolls at whoever it
   sees; a piper throws at them; a bell-carrier walks at them and cannot be
   hurt; a mussel shuts on whatever steps in.
7. **The ground is an island, and has an underside.** Rooms are floating
   islands. Dig deep enough and the hole comes out underneath: the player,
   the water and anything loose fall through it. Digging under a problem
   is a way down, never a way round.

What is scarce, and so what a puzzle spends:

- **Height:** once a ledge is gone or a pool is drained, it stays that way.
- **Water:** one reservoir fills one thing.
- **Blast:** a room gives a few grenades.
- **Lifters:** few, and each can be in one place.
- **Safety:** ground a creature is crossing is not ground the player can use.

## What makes a room a puzzle here

A room where the obvious tool simply works is a lesson, and the opening needs
a few. A puzzle is a room where **the thing in the way and the thing needed
are physically joined**, so the obvious tool spoils what it was meant to win.
The engine's worth is that those joins are real: the ledge really is what
holds the boxes up, the flotsam really does ride the water.

The patterns found so far:

| Pattern | The join | Example |
|---|---|---|
| **Shared support** | What is in the way stands on what is needed | A stack of boxes blocks a ledge. A grenade clears the boxes and the ledge with them. They have to come down another way. |
| **Unwanted passenger** | Releasing one thing releases another | Breaching a dam sends the water where it is wanted, and the driftwood behind the dam with it, which jams the channel. Clear the driftwood first, or catch it on the way. |
| **The clock is the key** | The hazard is also the means | A reservoir fills slowly and will drown the low path. It cannot be breached, because the way out is a float that only reaches the top when it is full. |
| **Too much** | The right act at the wrong strength fails | A fountain's tower cleaves into blocks big enough to climb, or shatters into rubble that is not. Where and how hard decides which. |
| **Which way it falls** | A release has a direction | A tree becomes a bridge if it falls across the gap. Which side of its roots is dug decides where it lands. |
| **One thing, two jobs** | The only lifter, float or weight is needed twice | The crate that plugs the outlet is the crate that reaches the ledge. Something else has to plug the outlet. |
| **Nothing underneath** | The ground itself is finite | An island is thin in places. A blast too many under a pool, a stack or the player's own feet sends it through to the sky. |
| **Borrowed force** | A creature does what the player cannot | A roller will follow down a groove dug for it into a bank the player cannot reach. A bell-carrier walking onto one end of a seesaw throws what is on the other. |

## Authoring rules

Learnt from building rooms. Each is a number or a habit a room is laid out
against.

- **The player's reach.** A full jump catches a lip up to about 3.2 m above
  the feet; anything 3.7 m or more is out of reach. `level_check` prints the
  jump envelope.
- **A throw needs room.** A grenade arms 0.15 s after it leaves the hand,
  about 3 m at throw speed. Nearer than that it bounces off unarmed. Every
  target needs a clear line from 3 m or more, which makes rooms bigger and
  more open than one screen.
- **Cut the support, not the mass.** What a grenade must sever should be
  slender: a leg, a neck, a thin root. The mass it holds can be any size.
  A wide root, or rock (about 2.5 times the cost of dirt per cubic metre),
  only dents.
- **Name the join, not the shape.** An idea says what is joined to what.
  Which volume builds it is found out in `rubble_viewer`, because only some
  shapes cut the way they look as if they should (E3's overhang did not).
- **Leave height for the player's own craters.** Blasting near where the
  player will stand lowers that ground by about half a metre. Every height
  a room depends on keeps that much to spare.
- **A fall lands roughly.** What comes down may crack, tilt or roll. Give it
  a generous place to land, not an exact one. Whether it cracks or drops
  clean is a matter of style, set by how the rubble planner splits pieces.
- **Every wall is a staircase.** A crater near the top of a wall is a
  foothold, so a wall the player must not climb is one no crater brings in
  reach, or one that leads nowhere. W1's first exit sat past the hollow, and
  one grenade in the far wall made a step out of it. Nor may dry ground stand
  level with a goal across a gap: a running jump carries over 9 m. W1's exit
  is now an island hanging in a closed pit, 9 m from its walls, and the
  flood floats the player up to it. Everything is
  destructible, so nothing is airtight: aim for every way round the puzzle
  costing far more than the way through. Only boulders build height: scree
  leaves nothing where it lands, so a thin hanging thing sheds nothing tall.
- **A drowned hole stops flowing.** The water field passes nothing through
  an opening once the water on its far side has risen over it, whatever head
  stands behind it. A breach that must fill something to a height has to be
  above that height: W1's pit stops rising at the top of a hole cut at chest
  height, short of the island, and fills to it from a hole 4.5 m up.
- **A terrace wall is 2 m thick or more.** The slopes either side of a
  plateau's edge wear a 1 m wall down; W1's reservoir leaked at 10.83 through
  one 7 m tall. A volume (a `Platform` dam) is not worn, so a dam can be
  thinner. `level_check` reports where a pool spills: run it first.
- **A breach is a hole, and a hole has a size.** A grenade in 1 m of sand
  opens a hole a reservoir empties through in about a minute; in 2 m it only
  trickles. Only the water above the hole's floor goes, so a reservoir's
  floor belongs below where the player's throw will strike its dam.
- **Bed every volume.** A volume that ends exactly on a floor is not joined
  to it: `rubble_viewer` reports it standing free as authored. Run it half a
  metre into the ground.
- **What falls lands on what is beside it.** M2's column, cut, dropped its
  8 m top on the docked platform and knocked it into the chasm. Leave nothing
  tall above a cut, or nothing that matters under it.
- **A floated thing drifts with the flood.** O2's crate, floated in an open
  pit, was carried to the far wall by the inflow. Float things in a well that
  guides them.
- **A machine does not wait.** A moving platform freed of what blocked it sets
  off at once and never pauses at a stop; the player boards it on its way
  back.
- **Not every loose thing is pushed.** Walked into, a trampoline is walked
  under; it has to be carried.
- **Build islands from platforms.** An `Island` volume takes its materials
  from the layers under the floor below it, and comes out earth on top and
  grass beneath; two `Platform`s, grass over dirt, read right.
- **Volumes come out larger than the sum.** Smoothed edges make a hollow hold
  more than its footprint times its depth. Size the water by measuring it
  in `water_viewer`, not by arithmetic.

### Testing a room

| Question | Tool |
|---|---|
| Does it cut, and what falls? | `rubble_viewer`, in milliseconds; a room's promise is kept there as a scenario |
| Where does the water go, and how high? | `water_viewer` with the game's grenade, probes in the room, in seconds; its promise kept as a test |
| Can the real body do this step? | `character_viewer`, for steps that come out the same every time: throws (aimed up or down), fixed jumps, carrying (`grabbing()`); the `room_*` scenarios play each room's way through |
| Can a person find the way, and does it read? | Play it. R rebuilds the room from its file, so an edit is one key away |

## Puzzle ideas by cluster

Each idea is filed under what the player mainly manipulates, and tagged with
anything else it leans on. Kind is **T** (teaches a rule alone), **X** (tests
it with a complication) or **C** (combines clusters). Spends is the scarce
thing the puzzle turns on.

### Earth

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| E1 | A dirt bank blocks the path, with grenades lying in front of it. Blast through. | T | | Blast |
| E2 | A thin bank and a hillside. A grenade opens the bank and only dents the hill: digging costs blast by the metre. **Prototyped:** `e2_thin_bank`, a plugged notch in a cliff. | T | | Blast |
| E3 | The exit is on a ledge too high to grab. A slab stands beside it on thin legs, as high and as out of reach. Blow the legs and it comes down as a step. **Prototyped:** `levels/rooms/e3_rubble_ramp.level.ron`. | X | | Height |
| E4 | Three grenades, four banks. One bank hides nothing; read the room before spending. | X | | Blast |
| E5 | The only path up is an earth stair beside a tempting bank. Blasting the bank drops the stair: the room teaches that a blast can cost height. **Prototyped:** `e5_bank_and_stair`: the bank is the stair's first step, and the stair is rock, so a crater in it is no foothold. | X | | Height |
| E6 | The island is thin under the path and thick under the hill. A tunnel has to follow the thick ground or it opens onto the sky. | X | | Height |
| E9 | The exit is on a lower island, straight below. Find where to dig through so the fall ends on it. A hole is a one-way door. **Prototyped:** `e9_one_way_down`: three grenades, each into the last crater, go through 2.5 m of earth. | X | Body | Height |
| E7 | A stack of boxes blocks an earth ledge that is the only way on. A grenade would take the ledge too: carry them off, or topple them from the side. **Prototyped:** `e7_crates_on_the_corner`: the ledge is a walkway hanging from its corner, and the crates stand on the corner. | X | Objects | Height |
| E8 | A tree stands by a gap. Dig under the roots on the far side and it falls across as a bridge; dig the near side and it falls away. Not buildable yet: a lollipop tree's trunk is 0.07 m thick, and a menhir tall enough to span a gap past a running jump is anchored 3 m down and rolls once down. | X | Objects | Blast |

### Water

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| W1 | A pool sits above a dry hollow that bars the way. Breach the pool's bank; the hollow fills and floats the player up to the exit, an island hanging in it. **Prototyped:** `levels/rooms/w1_flood_the_hollow.level.ron`. | T | Earth | Water |
| W2 | The reverse: a flooded passage too long to swim. Breach its low side to drain it. **Prototyped:** `w2_drain_the_pit`: the exit is on the floor of a pit 6 m under water, where a swimmer cannot reach it. | T | Earth | Water |
| W3 | One pool, two hollows, and a ridge between them. Which bank is breached decides which fills. **Prototyped:** `levels/rooms/w3_which_bank.level.ron`. | X | Earth | Water |
| W4 | A stream runs past the place water is wanted. Blast a channel to turn it, or drop earth to dam it. | X | Earth | Blast |
| W5 | A pump raises water as long as it is fed. Keep its basin from draining while using what it lifts. | X | Machines | Water |
| W6 | Fill a shaft from a spring to swim up to the top of it. | C | Body | Water |
| W7 | Driftwood lies behind a dam. Breached as it is, the wood rides out and jams the channel the water was meant to fill. Fish it out first, or leave a grating of earth for it to catch on. | X | Objects | Water |
| W8 | A reservoir fills slowly and will drown the low path, and cannot be breached: the float that reaches the exit only gets there when it is full. Work below while it rises. | X | Objects | Safety |
| W10 | A pool sits on thin ground. A blast on its bed empties it through the island onto whatever is below: a basin on the next island down, or a creature. | C | Earth | Water |
| W9 | A boulder sits on an earth shelf above a spring. Undermine the shelf and the boulder drops to plug it. | C | Earth, Objects | Height |

### Objects

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| O1 | A crate and a ledge just out of reach. Push the crate under it. **Prototyped:** `o1_crate_step`. | T | | |
| O2 | A crate floats. Flood the hollow it sits in and it comes up to the far ledge with the player on it. **Prototyped:** `o2_raft`: the hollow is a well, because a crate floated in open water drifts with the flood. | C | Water | Water |
| O3 | A trampoline is in the wrong place. Push it to the right one. **Prototyped:** `o3_trampoline`: carried, since walked into, a trampoline is walked under. | T | | Lifters |
| O4 | A trampoline sits in a dry basin across a gap. Fill the basin and it floats over. | C | Water | Lifters |
| O5 | A stone ball rests on a slope behind a wedge of earth. Blast the wedge; the ball rolls and breaks what it meets. | C | Earth | Blast |
| O6 | A seesaw with a heavy thing on the far end. Drop onto the near end from height to throw it. | X | Body | Height |
| O7 | One crate. It is wanted as a plug in the outlet and as a step to the ledge. Find the other thing in the room that can plug. | X | Water | Lifters |
| O8 | A trampoline on a float. Too little water and the bounce falls short; too much and the float rides over the lip and is lost. | C | Water | Water |
| O10 | A crate is wanted on the island below. Carry it to the hole and drop it through, where it will land and not roll off. | C | Earth | Height |
| O9 | A fountain tower stands by a wall. Cleave it at the neck and its blocks fall as a stair; blast it at the base and it shatters too small to climb. | X | Earth | Blast |

### Machines

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| M1 | A moving platform crosses a gap. Ride it. | T | | |
| M2 | A platform's track runs behind an earth bank that stops it short. Clear the bank and it completes its run. **Prototyped:** `m2_platform_and_column`: a column against its dock. | C | Earth | Blast |
| M3 | Two platforms out of step. Cross both by timing the change. **Prototyped:** `m3_out_of_step`. | X | Body | |
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
| C8 | A roller cannot be led to a far bank the player cannot reach. Dig a groove that carries it there once it is rolling. | C | Earth | Blast |
| C9 | A bell-carrier and a seesaw: lead it onto the low end and whatever sits on the other is thrown. | C | Objects | Safety |
| C7 | Egg-walkers are light and quick: they cross a float that will not bear the player, and trip what the player cannot reach. | C | Objects | |

### Body

The acrobatic layer. Mostly a way to make another cluster's room harder, with
a few rooms of its own early on to teach the moves.

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| B1 | A wall with a lip. Jump, catch it, climb. | T | | |
| B2 | Shimmy along a ledge past a gap in the floor. **Prototyped:** `b2_shimmy`. | T | | |
| B3 | Swim under a low arch and come up the far side. | T | Water | |
| B4 | Any earlier room again with the footing narrowed, the water rising, or a creature loose. | X | any | Safety |

### Finding out

An island has an underside and a room may be stacked, so the player cannot
always see what a hole leads to. Finding out is part of the puzzle, and it is
done with the same physics as everything else. The habit it teaches is the
one the whole game wants: test with something cheap before committing
something that cannot be got back.

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| F1 | A hole in the floor and a ball beside it. Drop the ball: it bounces back up, so there is ground close below. A second hole swallows its ball. | T | | |
| F2 | How long the ball takes says how far down; where it comes back says whether the ground below is level. Choose between three holes. | X | | |
| F3 | Send a hopping creature down instead. It lands and wanders, and where it goes shows how far the lower island reaches. | X | Creatures | |
| F4 | Breach a little water into the hole. If it pools below there is a basin; if it keeps falling there is not. | X | Water | Water |
| F5 | With the sun low, the island's shadow lies on the one beneath. Look over the edge to see where the lower ground is, and is not. | T | | |
| F6 | A carried mirror shows what is under a ledge or round a corner **(build)**. | T | Objects | |
| F7 | One grenade on the floor shows how thick the island is there: a shallow crater, or daylight. | X | Earth | Blast |

## Second batch

More ideas, not yet filed into the tables above or the opening. **(check)**
marks one that depends on behaviour not yet confirmed in the engine.

### Earth and rubble

| Idea | Pattern | Also |
|---|---|---|
| Blasting a cliff drops boulders. Dropped in the right place they dam the stream below: height is spent to buy a dam. | One thing, two jobs | Water |
| A boulder is wanted as a step, but the blast that frees it also sends it rolling off the island. Dig it a hollow to land in first. | Too much | Objects |
| An earth bridge will bear the player and not the bell-carrier. Cross first, and let it follow. | Shared support | Creatures |
| The exit ledge is held up by the same bank that blocks the way to it. Take only as much as leaves it standing. | Shared support | |

### Water

| Idea | Pattern | Also |
|---|---|---|
| Falling water pushes what it lands on. Turn a fall so it drives a float across a still pool. | Borrowed force | Objects |
| A spring's jet holds a ball up. Something light set on the jet rides at its top: a lifter made from water. | One thing, two jobs | Objects |
| A sealed barrel wants to rise. Wedged under a sunk platform, it lifts the platform when the basin floods. | Borrowed force | Objects |
| Glass holds a pool back and shows what is behind it. Breaking it is a breach that no earth can mend. | Too much | Objects |
| Two basins joined by a channel find one level. Raising one by dropping rubble into it raises the other. | Borrowed force | Earth |
| A blast in still water shoves a float away from it. With no current, that is the only oar. | Borrowed force | Earth |

### Objects

| Idea | Pattern | Also |
|---|---|---|
| A tower of blocks is the only stair, and its blocks are the only weights. Take what can be spared without felling it. | Shared support | |
| A plank carried to a gap is a bridge; the same plank is the lever under the boulder. | One thing, two jobs | |
| An ice block pushed on level ground slides until something stops it. Arrange the stop before the push. | Too much | |
| A pendulum's bob will carry the player across, or, cut loose at the top of its swing, fly where a weight is wanted. | Which way it falls | Earth |
| A grenade thrown onto a trampoline goes where the player cannot throw. | Borrowed force | Earth |
| A grenade set rolling down a channel reaches a bank at the far end of it **(check)**. | Borrowed force | Earth |
| A branch blasted off a tree at its fork is a plank; the tree left standing is still the ladder. | Shared support | Earth |

### Creatures

| Idea | Pattern | Also |
|---|---|---|
| A peeper sees a long way and nothing through earth. Drop a bank between it and the path. | Borrowed force | Earth |
| A stalker's beak reaches over a wall. Stand so that its strike knocks the crate off the far ledge **(check)**. | Borrowed force | Objects |
| A fish-walker follows into water. Lead it onto the float the player needs sunk, or off the one they need to rise. | Borrowed force | Water |
| A flying fish stoops on anything in the open. Cross under cover, or dig cover where there is none. | Nothing underneath | Earth |
| A mussel holds whatever it shuts on. Shut on a float's tether or a rolling ball, it is an anchor **(check)**. | Borrowed force | Objects |
| A piper throws at what it sees. Seen from behind the dam, its bombs breach it; seen from behind the only ledge, they take that instead. | Shared support | Earth |
| A roller in a crater is stranded until the crater floods. Then it floats out **(check)**. | Unwanted passenger | Water |

### Third batch

| Idea | Pattern | Also |
|---|---|---|
| A piper is slowly bombing the platforms the crossing needs. Cross before they are gone, or stop it seeing them. | The clock is the key | Creatures, Machines |
| A creature stands in a dry channel below a dam. Burst the dam and it is washed off the island. | Borrowed force | Water, Creatures |
| A creature throws the player's grenades back. Throw round it, over it, or so that its return lands where the blast is wanted **(build)**. | Borrowed force | Creatures |
| A crate dropped in a stream will be carried to the bottom, where it is needed to cross. Get there before it goes over the edge. | Unwanted passenger | Water, Body |

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
| 3 | E2 | What a grenade costs |
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
| Room reset at a shrine | Every room, since actions are irreversible | R rebuilds the room from its file; no shrine |
| Grenades as a limited pickup | E4 and the Blast economy | Grenades are free |
| Room exit | Every room | A `Goal` object exists; untested as a room exit |

## Open questions

1. **Is resetting a room free?** Free keeps experiments cheap; a cost makes
   the player think first. Draft assumes free.
2. **Do creatures reset with the room?** Draft assumes yes.
3. **Can the player be hurt by their own blast?** It changes how E-rooms feel.
   It already comes up: a grenade thrown at something nearer than it flies
   before arming bounces back and goes off at the thrower's feet. Fine for
   now; to be tuned.
4. **How big is a room?** Draft assumes one screen or two: the whole problem
   visible from the shrine.
