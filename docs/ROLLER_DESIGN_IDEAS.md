# Roller Design Ideas

An idea backlog, not a plan. Nothing here is committed to or scheduled, and the
ordering is by theme rather than by priority. Each entry records what the idea
leans on and what is likely to go wrong with it, because that is the part that
gets forgotten between the conversation and the attempt.

Implementation lives in `src/creature/roller.rs` (locomotion) and
`src/app/creatures/roller.rs` (the definition). See `project_creature_system`
in the assistant's project memory for the architecture and its gotchas.

## Current state

A roller is a living boulder that hunts by rolling. It has no skeleton, which
is why it was the first creature: it exercises the brain, perception, steering
and intent layers without needing a gait.

- Motion is **angular impulse only**. Friction converts spin into travel, so the
  creature is subject to the real surface — it accelerates into a crater and
  struggles out of one. That behaviour is free and is most of the character.
- Intent comes from `BrainSystem` through `CharacterIntent`, the same struct the
  keyboard fills for the player. The seam is agnostic at both ends.
- **It cannot be killed.** No `Health`, no `Flammable`. You beat one by
  outrunning it or by cratering the ground and stranding it. Explosion knockback
  is applied to `Velocity` independently of damage, so grenades remain useful —
  as a way to *move* it. This is deliberate; see the commit that removed them.
- Two authored dials: `speed` (flat-ground top speed) and `spin_up_time`
  (acceleration), the latter clamped to what friction can transmit.
- `AlertTelegraph` gives the 0.6 s alert beat a hop and a shudder, since a
  sphere has no face to pull.

## The design thread

The trapping discovery — blow a hole, strand the roller — was more fun than
killing one. What made it work is that the roller stopped being an obstacle and
became **something you aim**. Ideas 2, 8 and 9 below are all the same instinct
from different angles, and that is probably the thread to pull first.

The corollary is that damage is the *least* interesting verb available here, and
several of these ideas get worse if a health bar is reintroduced.

## Shared blockers

Two things gate multiple ideas. Worth doing early for that reason alone.

**Contact-driven fracture.** `FractureSystem` currently reads only the explosion
impulse queue, so nothing breaks from being hit — a roller cannot smash a
fracturable at any speed. The solver already computes what is needed
(`SolverContact::accumulated_normal_impulse`); `ContactEvent` just does not carry
it up. Plumbing that through upgrades every fracturable already in the levels,
not only roller interactions. Risk: every existing fracturable gains a new way to
break, so thresholds will need a pass.

**Sustained luring.** `Perception::memory_duration` is 5 s, so a roller forgets
you almost immediately once out of sight and cannot be led anywhere. Any puzzle
that involves bringing a roller to a place more than a few seconds away is
unsolvable until there is a stimulus it will follow. This is the "stimulus bus"
step of the original creature plan.

## Ideas

### 1. Ride it

Point the grab system's constraint at a roller and the player is standing on a
2.2-tonne ball trying to stay on top. The humanoid animation and foot placer
would be doing genuinely novel work. The enemy becomes a vehicle nobody quite
controls.

- **Leans on:** `character::grab`, the foot placer, `KeepUpright`.
- **Catch:** the best idea here and the most likely to eat a month and feel bad.
  Prototype scrappily before committing to it.

### 2. It chases grenades

Make the stimulus literal: a thrown grenade is more interesting than the player
is. Grenades become bait, the roller sprints toward something about to explode,
and the primary tool gains a second use with nothing new to learn.

- **Leans on:** the stimulus bus (see blockers), `PerceptionSystem` targeting.
- **Catch:** needs perception to target non-player entities, which is the one
  place the "targets are the player" assumption is currently hard-coded.
- **Note:** best fun-per-line ratio on this list.

### 3. Splitting

A hard enough hit breaks one roller into two smaller, faster ones. Asteroids,
but physical. Inverts the trapping discovery — blasting one becomes actively the
wrong move and the pit becomes the only real answer.

- **Leans on:** contact-driven fracture (see blockers).
- **Catch:** unkillable + splitting means population only ever grows. Needs a
  floor on size, and probably an answer to "what removes the smallest ones".

### 4. They carve the terrain

A boulder that heavy should leave ruts. They are in constant contact with a
destructible SVO — bleed a little material where they roll and the arena erodes
into visible paths over a long fight. The level ends up recording what happened
in it.

- **Leans on:** terrain destruction, which is already fast enough per grenade but
  has never been asked for a continuous trickle.
- **Catch:** cost. Destruction is ~15 ms per grenade; a per-frame trickle needs a
  much cheaper path, probably batched and amortised.
- **Note:** the most distinctive-to-this-engine idea on the list.

### 5. Lava roller

A variant that ignites what it touches, driving the GPU fire sim from a moving
source. It stops being a thing that chases and becomes a thing that ruins the
room — the player dodges where it has been, not where it is.

- **Leans on:** `src/fire/`, `Flammable` on *other* objects.
- **Catch:** the roller itself deliberately lost `Flammable`; this variant sets
  fires without being subject to them, which needs the distinction to be clean.

### 6. Tethered pair

Two rollers joined by a distance constraint. Sweeps the arena like a bolas, the
tether catches on terrain, and trapping one strands both.

- **Leans on:** the constraint arena, which already supports distance joints.
- **Catch:** two independent brains pulling against a shared constraint may just
  produce a stalemate. Possibly one brain driving both bodies.
- **Note:** very high spectacle for how little new code it is.

### 7. Dormant scenery

They start asleep and look like the level — indistinguishable from the boulders
already scattered around, until one wakes. The sleep system was built for
performance; using it as an ambush mechanic costs nearly nothing and permanently
changes how the player reads a rock.

- **Leans on:** the sleep system, and the fact that rollers already sleep when
  they lose interest.
- **Catch:** needs the roller's material to actually match nearby scenery, which
  is a texture/authoring question more than a code one.
- **Note:** nearly free, and it changes every level that already exists.

### 8. Friction as level design

Friction is per-collider and is exactly what converts torque into travel. An ice
patch makes a roller spin helplessly in place; mud bogs it down. A counter the
*level* provides rather than one the player carries.

- **Leans on:** per-surface friction on terrain, which may not exist yet at
  material granularity.
- **Catch:** the player walks on the same surfaces. Ice that stops a roller also
  makes the player skid, which may be a feature.

### 9. Roller golf

Flip the goal: the level needs a roller delivered somewhere — through a glass
wall, onto a pressure plate, into a pit that opens a door. The enemy becomes a
resource with a mind of its own, and every existing tool becomes a way of aiming
it.

- **Leans on:** sustained luring (see blockers), and some notion of a trigger
  volume, which does not exist yet.
- **Note:** the cheapest way to turn rollers into *levels* rather than
  encounters.

### 10. Possess one

The intent seam is source-agnostic at both ends. Swapping which system writes a
roller's `CharacterIntent` is close to trivial, and the result is playing as a
boulder with no ability to stop.

- **Leans on:** the intent seam, exactly as designed.
- **Catch:** the camera. `FollowTarget` assumes an upright character.
- **Note:** the free thing the AI architecture bought.

## Also discussed

Smaller notes from the same conversation, kept so they are not rediscovered.

- **Glass walls.** Gate them on **impulse, not creature type**. A type check is a
  lock with one key; an energy threshold is a material property, and then a
  roller, a thrown crate, a grenade-launched object and a roller that built speed
  downhill all compose against it for the same authoring effort.
- **Downhill is already dangerous.** `max_spin` caps *driven* spin, not gravity.
  A roller descending a slope exceeds its own top speed and nothing clamps it.
  There is probably already a terrifying encounter in the existing crater.
- **Rollers collide with each other.** Two converging on the player knock each
  other off course, for free. Possibly worth a level that arranges it.
- **Water.** At 2400 kg/m³ they sink hard. Whether one rolls along a pool floor
  or simply sits there is unknown — could be a free trap, could be a physics
  embarrassment. Cheap to find out by play-testing.
- **Do not build a Nest of rollers** while they remain unkillable and permanent.

## Picks

If forced to choose three: **#2** for fun-per-line, **#4** because nothing else
can do it, **#7** because it is nearly free and retroactively improves every
existing level. **#1** is the best idea and the riskiest.
