# Camera clearance — what was tried, and why it is off

`CameraClearance` (`src/camera/clearance.rs`) keeps the follow camera out of the
scenery. It is switched off by default (`enabled: false`): the camera passes through
terrain, as it always has. This records the two attempts and why neither was good to
play, so the next attempt starts from here.

## Attempt 1: snap in front of whatever is in the way

A 0.3 m sphere cast from the pivot to the camera each frame; the camera stands where
it first touches terrain or a static body (moving bodies are ignored). Easing back
out was left to the follow smoothing.

**Failed on:** anything sliding between camera and player — a stalagmite in a cave —
snapped the camera in and out. Jarring.

## Attempt 2: sweep the lens, wait out blockers, cut through, ease out

- The camera is swept as a sphere from where it was to where it is going, and slides
  along what it touches.
- A blocker on the pivot-to-camera sight line is acted on only after 0.25 s, and only
  if it is at least 1 m thick (measured by casting back from the camera).
- Acting means cutting through the blocker to its near side; when the view clears,
  the allowed reach eases back out on a critically damped spring (0.5 s).

The cut is forced: a blocker always stands between camera and pivot, so no smooth path
in front of it keeps the lens out of rock. A smooth pull-in and "never inside rock"
cannot both hold.

**Failed on:**

1. **Zoom became asymmetric.** The eased reach was tracked even with nothing in the
   way, and the camera stood at `min(reach, wanted distance)`. Zooming in shrank the
   wanted distance at once; zooming out raised it, but the reach followed on its 0.5 s
   spring, so zoom-out was painfully slow and zoom-in felt different. *Lesson:* a
   clearance limit must exist only while something limits it, and must never filter
   the player's own zoom.
2. **Tunnels: snapping and jitter.** In a tunnel or cave, the orbit position the
   follow wants is usually inside the rock. So the sight line is blocked by thick rock
   almost all the time: wait 0.25 s pressed against the wall, cut in, ease out, get
   stopped by the wall, be blocked again — a cycle. Meanwhile sweep-and-slide against
   curved walls and ceilings jitters between faces. Blasted tunnels make this the
   common case in this game, not the edge case. *Lesson:* reacting to occlusion
   *events* breaks down where the wanted position is never reachable. The camera needs
   a continuous sense of how much room there is.

## For the next attempt

- Separate the player's zoom (wanted distance) from what the scenery allows, and
  apply the scenery's limit only while it binds.
- Treat room as a continuous, smoothed signal: several casts (whiskers) around the
  sight line, the available distance taken from them with hysteresis, so a confined
  space settles the camera close in and keeps it there, rather than toggling.
- Consider changing the orbit itself in confined spaces (pitch up, or pull to a
  shoulder view) rather than only the distance.
- Measure before iterating: the headless tools use their own camera, so a scripted
  camera scenario is needed (camera driven through a blasted tunnel, past a pillar,
  zoomed in and out), reporting per-frame camera speed and jerk. Neither failure
  above was visible in the unit tests.

The building blocks stay: `PhysicsWorld::cast_sphere` with `CastObstacles::Scenery`,
the sweep-and-slide, the thin-occluder measure, and the critically damped spring.
