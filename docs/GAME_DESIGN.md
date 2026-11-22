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
   that open doors. A weight does its work where it rests. The level's
   objective (solids, temple, brazier) sits outside this rule because it
   never changes the room ([WORLD_AND_STORY.md](WORLD_AND_STORY.md)).
5. **Only lifters give height back safely:** trampolines, moving platforms,
   pumps and springs, rising water, the hover sled. A room has a fixed few.
   A blast can throw the player up too, but never safely (see *Harm*).
6. **Creatures follow their nature, always.** A roller rolls at whoever it
   sees; a piper throws at them; a bell-carrier walks at them and cannot be
   hurt; a mussel shuts on whatever steps in.
7. **The ground is an island, and has an underside.** Rooms are floating
   islands. Dig deep enough and the hole comes out underneath: the player,
   the water and anything loose fall through it. Digging under a problem
   is a way down, never a way round.
8. **Only impacts harm.** Whatever strikes or stops the player (the ground
   after a fall, a falling pillar, a blast, a roller) harms them by how hard
   it hits, and by nothing else (see *Harm*).

What is scarce, and so what a puzzle spends:

- **Height:** once a ledge is gone or a pool is drained, it stays that way.
- **Water:** one reservoir fills one thing.
- **Blast:** a room gives a few grenades.
- **Lifters:** few, and each can be in one place.
- **Safety:** ground a creature is crossing is not ground the player can use.

## Harm

Harm is the physics engine's verdict, like everything else in a room. The
game is built around its physics, so what hurts the player is decided by
the same simulation that moves every stone and every drop of water, with no
rules of its own. A player who can read the physics can read the danger.

There is no health bar. Every impact on the player is judged by one
measure, the sudden change in their velocity (the impulse divided by their
mass), whatever caused it, and falls into one of three bands:

| Impact | Result |
|---|---|
| Light | Shoved; they stagger, and keep control |
| Heavy | Knocked down: for a few seconds they tumble with no control, and land however physics lands them |
| Deadly | The room restarts |

Two thresholds price everything, with no special cases:

- **Falls** are judged on landing. Water breaks a fall. The deadly height
  sits well above anything a jump reaches, so a fatal fall is a clear
  misjudgement, never a surprise.
- **Heavy things:** a glancing blow from a falling pillar knocks the player
  down; being pinned or crushed kills.
- **Creatures** push, trap and knock down. They do not kill directly: the
  danger is where their shove sends the player (off a ledge, into water,
  under a falling stone).
- **Blasts, the player's own included,** shove, knock down or kill by how
  close they are. Nothing stops a blast throwing the player upward, and the
  physics is not cheated to prevent it. The heavy band prices it instead:
  it starts below the player's own jump takeoff speed (about 4.4 m/s for a
  1 m jump), so any blast that throws them higher than they can jump also
  knocks them down. They fly uncontrolled, and the fall law judges the
  landing. Blast-jumping is a desperate gamble, never a trick: a room's
  heights must allow for a player who tries it.

### How a knock looks

A heavy hit takes the player off their feet, and how they tumble is physics
too:

- **Spin comes from where the blow lands.** A blow far from the centre of
  mass turns the body by its lever arm. A blow at the head or the feet sends
  the player spinning; one at the middle shoves them flat with little turn.
  A blast at their feet strikes the legs harder than the head and tips them
  over.
- **The limbs follow the motion.** They trail behind the change in velocity
  and fling outward with the spin. A full frontal hit, with no spin, leaves
  the limbs trailing back, splayed; a blow on an end gives a flailing,
  spinning tumble.

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
- **Ground is a liability.** Every piece of terrain is something the player
  can sculpt into a stair or tunnel through. The first playtest batch found
  the crater stair again in O1, O3 and E5, and a tunnel round B2's wall
  through the ground behind it. Cut terrain back: islands, open space, and
  nothing solid next to a route that is not part of it. Accept a crater stair
  as the terrain's answer where a room is not about height, and use
  spawnables (a temple's blocks) where it must be.
- **Mind the running jump.** A running jump crossed M3's 12 m gap to its
  pillar. Until the jump is tuned, measure every gap a room depends on
  against it.
- **A drowned hole stops flowing.** The water field passes nothing through
  an opening once the water on its far side has risen over it, whatever head
  stands behind it. A breach that must fill something to a height has to be
  above that height: W1's pit stops rising at the top of a hole cut at chest
  height, short of the island, and fills to it from a hole 4.5 m up.
- **A stream needs somewhere to go.** A spring never stops; a channel whose
  sink does not drain it fills and spills over everything. Run the channel
  past the room's edge and make the sink box generous, then watch a long run
  in `level_viewer` before trusting it.
- **Soft ground takes grip and bounce.** Grass, dirt and sand give
  whatever touches them their own friction and the lower of the two
  restitutions: ice grips on them, and a ball dies. Lay rock where a thing
  must slide or bounce.
- **A plateau's edge sheds water.** Its slopes round a ridge's top off, so a
  flood let onto one pours off the sides where it arrives. Give a flood
  curbs (`Platform`s, which are not worn) to keep it running where it is
  wanted.
- **A jet ignores what hangs over it.** Falling water follows its arc and
  lands on the first top it comes down onto; a slab over a spring does not
  cap it.
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
| E1 | A dirt bank blocks the path, with grenades lying in front of it. Blast through. **Prototyped:** `e1_blast_through`: the bank stands across a strip of island, past both edges. Played: works; very basic. | T | | Blast |
| E2 | A thin bank and a hillside. A grenade opens the bank and only dents the hill: digging costs blast by the metre. **Prototyped:** `e2_thin_bank`, a plugged notch in a cliff. Played: works, and a plug is a good way to hide a goal. | T | | Blast |
| E3 | The exit is on a ledge too high to grab. A slab stands beside it on thin legs, as high and as out of reach. Blow the legs and it comes down as a step. **Prototyped:** `levels/rooms/e3_rubble_ramp.level.ron`. | X | | Height |
| E4 | Three grenades, four banks. One bank hides nothing; read the room before spending. | X | | Blast |
| E5 | The only path up is an earth stair beside a tempting bank. Blasting the bank drops the stair: the room teaches that a blast can cost height. **Prototyped:** `e5_bank_and_stair`: the bank is the stair's first step, and the stair is rock, so a crater in it is no foothold. Played: nothing tempts a blast at the bank, and the crater stair climbs the cliff beside it. Wants something that draws the throw (a stationary creature on the bank) and a landing that hangs free. | X | | Height |
| E6 | The island is thin under the path and thick under the hill. A tunnel has to follow the thick ground or it opens onto the sky. **Prototyped:** `e6_through_the_hill`. Played: the tunnel went through without holing a 2 m floor; the floor is now 1 m. | X | | Height |
| E9 | The exit is on a lower island, straight below. Find where to dig through so the fall ends on it. A hole is a one-way door. **Prototyped:** `e9_one_way_down`: three grenades, each into the last crater, go through 2.5 m of earth. Played: works, but free grenades make the guess cheap. Wants limited grenades, or ground that shatters at the first blast (glass), so the player must stand right before throwing. | X | Body | Height |
| E7 | A stack of boxes blocks an earth ledge that is the only way on. A grenade would take the ledge too: carry them off, or topple them from the side. **Prototyped:** `e7_crates_on_the_corner`: the ledge is a walkway hanging from its corner, and the crates stand on the corner. Played: works; the player blasted first and the walkway dropped, as meant. | X | Objects | Height |
| E8 | A tree stands by a gap. Dig under the roots on the far side and it falls across as a bridge; dig the near side and it falls away. Not buildable yet: a lollipop tree's trunk is 0.07 m thick, and a menhir tall enough to span a gap past a running jump is anchored 3 m down and rolls once down. | X | Objects | Blast |

### Water

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| W1 | A pool sits above a dry hollow that bars the way. Breach the pool's bank; the hollow fills and floats the player up to the exit, an island hanging in it. **Prototyped:** `levels/rooms/w1_flood_the_hollow.level.ron`. | T | Earth | Water |
| W2 | The reverse: a flooded passage too long to swim. Breach its low side to drain it. **Prototyped:** `w2_drain_the_pit`: the exit is on the floor of a pit 6 m under water, where a swimmer cannot reach it. Played: works. | T | Earth | Water |
| W3 | One pool, two hollows, and a ridge between them. Which bank is breached decides which fills. **Prototyped:** `levels/rooms/w3_which_bank.level.ron`. | X | Earth | Water |
| W4 | A stream runs past the place water is wanted. Blast a channel to turn it, or drop earth to dam it. **Prototyped:** `w4_turn_the_stream`: a spring's stream beside a closed pit, W1's island hanging in it; it fills in about four minutes. Played: works, and slow, which is physics; a piper on the rim now gives the wait something to dodge. | X | Earth | Blast |
| W5 | A pump raises water as long as it is fed. Keep its basin from draining while using what it lifts. | X | Machines | Water |
| W6 | Fill a shaft from a spring to swim up to the top of it. | C | Body | Water |
| W7 | Driftwood lies behind a dam. Breached as it is, the wood rides out and jams the channel the water was meant to fill. Fish it out first, or leave a grating of earth for it to catch on. | X | Objects | Water |
| W8 | A reservoir fills slowly and will drown the low path, and cannot be breached: the float that reaches the exit only gets there when it is full. Work below while it rises. | X | Objects | Safety |
| W10 | A pool sits on thin ground. A blast on its bed empties it through the island onto whatever is below: a basin on the next island down, or a creature. | C | Earth | Water |
| W9 | A boulder sits on an earth shelf above a spring. Undermine the shelf and the boulder drops to plug it. | C | Earth, Objects | Height |

### Objects

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| O1 | A crate and a ledge just out of reach. Push the crate under it. **Prototyped:** `o1_crate_step`. Played: a crater lower in the cliff is a stair too; push-only needs spawnables in place of terrain. | T | | |
| O2 | A crate floats. Flood the hollow it sits in and it comes up to the far ledge with the player on it. **Prototyped:** `o2_raft`: the hollow is a well, because a crate floated in open water drifts with the flood. Played: works. | C | Water | Water |
| O3 | A trampoline is in the wrong place. Push it to the right one. **Prototyped:** `o3_trampoline`: carried, since walked into, a trampoline is walked under. Played: the crater stair, as in O1. | T | | Lifters |
| O4 | A trampoline sits in a dry basin across a gap. Fill the basin and it floats over. | C | Water | Lifters |
| O5 | A stone ball rests on a slope behind a wedge of earth. Blast the wedge; the ball rolls and breaks what it meets. | C | Earth | Blast |
| O6 | A seesaw with a heavy thing on the far end. Drop onto the near end from height to throw it. | X | Body | Height |
| O7 | One crate. It is wanted as a plug in the outlet and as a step to the ledge. Find the other thing in the room that can plug. | X | Water | Lifters |
| O8 | A trampoline on a float. Too little water and the bounce falls short; too much and the float rides over the lip and is lost. | C | Water | Water |
| O10 | A crate is wanted on the island below. Carry it to the hole and drop it through, where it will land and not roll off. **Prototyped:** `o10_crate_down_the_hole`. Played: works. | C | Earth | Height |
| O9 | A fountain tower stands by a wall. Cleave it at the neck and its blocks fall as a stair; blast it at the base and it shatters too small to climb. | X | Earth | Blast |

### Machines

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| M1 | A moving platform crosses a gap. Ride it. **Prototyped:** `m1_ride_across`. Played: works; a running jump does not make 18 m. | T | | |
| M2 | A platform's track runs behind an earth bank that stops it short. Clear the bank and it completes its run. **Prototyped:** `m2_platform_and_column`: a column against its dock. Played: fun; a grenade threw the 100 kg deck off its track, so it is now 800 kg. | C | Earth | Blast |
| M3 | Two platforms out of step. Cross both by timing the change. **Prototyped:** `m3_out_of_step`. Played: plain platforming, the speeds do not matter, and a running jump reaches the pillar without the platforms. | X | Body | |
| M4 | A platform passes under a ledge with a crate on it. Push the crate so the platform carries it where it is needed. **Prototyped:** `m4_crate_on_the_platform`: the crate starts on the near island, and the exit hangs past the far one. Played: works, but the player carried the crate across; it would have stayed on the deck. | C | Objects | Lifters |
| M5 | The hover sled crosses water but not a rise. Lower the rise or raise the water. | C | Earth, Water | Height |

### Creatures

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| C1 | A roller in a pen, seen through a gap, with a soft wall between it and the exit. Stand behind the wall. | T | | Safety |
| C2 | A roller patrols the hollow the player must cross. Crater its path and it is stranded in the hole. **Prototyped:** `c2_roller_in_the_hollow`: the hollow's sides are open to the sky, so a hit throws the player off. Played: too easy to run past the roller in a 16 m hollow; now 5 m wide and 18 m long. | C | Earth | Blast |
| C3 | The same hollow with no grenades and a pool above it. Flood it. | C | Water | Water |
| C4 | A bell-carrier follows, slowly. Lead it onto a float, a seesaw or a weak earth bridge: it is the heaviest thing in the room. **Prototyped:** `c4_bell_on_the_bridge`: it stands on the only bridge; shove it off, dig it out, or draw it off and go round. Played: a blast on the bridge dropped the far island as rubble, since nothing holds a floating island up but the bridge; and the carrier is jumped over. Wants islands that hold without the bridge, and a carrier that fills the way (a low roof, a narrow ledge). | C | Objects | Safety |
| C5 | A piper on a rock and no grenades anywhere. Stand where its throw should land. | C | Earth | Safety |
| C6 | A mussel beside a path a stalker walks. Draw the stalker across it. **Prototyped:** `c6_mussel_and_stalker`. Played: the mussel is jumped over. Wants a mussel that fills the way, under a low roof. | X | | Safety |
| C8 | A roller cannot be led to a far bank the player cannot reach. Dig a groove that carries it there once it is rolling. | C | Earth | Blast |
| C9 | A bell-carrier and a seesaw: lead it onto the low end and whatever sits on the other is thrown. | C | Objects | Safety |
| C7 | Egg-walkers are light and quick: they cross a float that will not bear the player, and trip what the player cannot reach. | C | Objects | |

### Body

The acrobatic layer. Mostly a way to make another cluster's room harder, with
a few rooms of its own early on to teach the moves.

| # | Idea | Kind | Also | Spends |
|---|---|---|---|---|
| B1 | A wall with a lip. Jump, catch it, climb. **Prototyped:** `b1_up_a_lip`. Played: works; very basic. | T | | |
| B2 | Shimmy along a ledge past a gap in the floor. **Prototyped:** `b2_shimmy`. Played: the roof sat 0.5 m back and the player climbed into the gap; and a tunnel through the wall from behind went round. Roof now flush, chasm behind the wall too. | T | | |
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
| F1 | A hole in the floor and a ball beside it. Drop the ball: it bounces back up, so there is ground close below. A second hole swallows its ball. **Prototyped:** `f1_drop_the_ball`. Played: the ball did not bounce: grass and dirt take a bounce. The lower island is now rock, a metre under the upper, so the ball rests in sight at the bottom of the hole. | T | | |
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
| The exit ledge is held up by the same bank that blocks the way to it. Take only as much as leaves it standing. **Prototyped:** `e10_shared_support`: the exit is a slab jutting from the bank. Played: does not read as "take only as much as you need". | Shared support | |

### Water

| Idea | Pattern | Also |
|---|---|---|
| Falling water pushes what it lands on. Turn a fall so it drives a float across a still pool. | Borrowed force | Objects |
| A spring's jet holds a ball up. Something light set on the jet rides at its top: a lifter made from water. **Prototyped:** `w11_jet_lift`, as a check that a jet lifts the player. A cap over the spring cannot hold it back: a jet passes through what hangs over it. Played: the jet pushes down, not up: a stream meets a body on the falling half of its arc only, and a straight-up jet's falling half comes down on itself. | One thing, two jobs | Objects |
| A sealed barrel wants to rise. Wedged under a sunk platform, it lifts the platform when the basin floods. | Borrowed force | Objects |
| Glass holds a pool back and shows what is behind it. Breaking it is a breach that no earth can mend. | Too much | Objects |
| Two basins joined by a channel find one level. Raising one by dropping rubble into it raises the other. | Borrowed force | Earth |
| A blast in still water shoves a float away from it. With no current, that is the only oar. | Borrowed force | Earth |

### Objects

| Idea | Pattern | Also |
|---|---|---|
| A tower of blocks is the only stair, and its blocks are the only weights. Take what can be spared without felling it. | Shared support | |
| A plank carried to a gap is a bridge; the same plank is the lever under the boulder. | One thing, two jobs | |
| An ice block pushed on level ground slides until something stops it. Arrange the stop before the push. **Prototyped:** `o11_ice_slide`: a crater is the stop. Played: ice does not slide on grass, by design (soft ground keeps its own grip); the strip is now rock. | Too much | |
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
| A piper is slowly bombing the platforms the crossing needs. Cross before they are gone, or stop it seeing them. **Prototyped:** `c10_piper_and_the_stones`: four stepping stones, one hit each. Played: works. | The clock is the key | Creatures, Machines |
| A creature stands in a dry channel below a dam. Burst the dam and it is washed off the island. **Prototyped:** `c11_washed_off`, with a stalker: whether a flood moves one is the open question. Played: the flood does not move the stalker at all. | Borrowed force | Water, Creatures |
| A creature throws the player's grenades back. Throw round it, over it, or so that its return lands where the blast is wanted **(build)**. | Borrowed force | Creatures |
| A crate dropped in a stream will be carried to the bottom, where it is needed to cross. Get there before it goes over the edge. | Unwanted passenger | Water, Body |

### Not prototyped

Left out of the prototyping on purpose, as not worth a room yet or not
buildable:

- **Rolling a grenade down a channel (check).** A tunnel too low to walk into
  but wide enough to throw into needs no rolling; forcing the roll needs
  geometry no room was found for.
- **The tower that is both stair and weights.** Nothing in the engine weighs
  anything down (the seesaw is too small), so the weights have no use.
- **The peeper.** Every island room puts it on ground it cannot leave or
  ground the player shares; with death off, being seen costs nothing.
- **Glass holding back water, a barrel lifting a platform, rubble raising a
  linked basin.** Bodies do not hold back the water field, and rubble is too
  little to raise a level.
- **E4, F2–F7, O4–O9, M5, W5–W10, B3, B4, C1, C3, C5, C7–C9.** Variations on
  what the prototyped rooms showed, or waiting on limited grenades, death
  and respawn, or diving.

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
| Impact bands | *Harm*: shoves, knockdowns and deaths judged by the player's change in velocity | Not started |
| Knocked-down state | *Harm*: a heavy hit tumbles the player out of control | Not started |
| Room reset at a shrine | Every room, since actions are irreversible | R rebuilds the room from its file; no shrine |
| Grenades as a limited pickup | E4 and the Blast economy | Grenades are free |
| Room exit | Every room | A `Goal` object exists; its trigger reaches 3 m below it, to rework |
| A level with no ground | Sky rooms built only of floating pieces, so no route has terrain beside it to sculpt | Every terrain has a base height; islands are volumes over it |
| Ground that shatters | E9: a surface (glass) that breaks at the first blast, so a guess costs the room | Not started |
| Jump tuning | M3 and every gap: a running jump carries over 12 m | Not started |
| Islands that hold by themselves | C4: a floating island joined to another by a bridge falls as rubble when the bridge is cut | Not started |
| Water moves creatures | C11: a flood leaves a stalker where it stands | Not started |
| A jet that rises | W11: a body in an upward jet is struck by its falling half and pushed down | Not started |

## Open questions

1. **Is resetting a room free?** Free keeps experiments cheap; a cost makes
   the player think first. Draft assumes free.
2. **Do creatures reset with the room?** Draft assumes yes.
3. **How big is a room?** Draft assumes one screen or two: the whole problem
   visible from the shrine.
