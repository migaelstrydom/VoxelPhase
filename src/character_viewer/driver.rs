//! Runs a scenario in the real game and records what the character did.
//!
//! ```text
//!   Scenario ─▶ GameHarness::open(level)          (the game, headless)
//!                  │
//!                  ├─ take the level's player: drop its `Player` marker so the
//!                  │  keyboard no longer writes its intent, and stand it at
//!                  │  the scenario's start
//!                  │
//!   per frame:     ├─ Pilot ─CharacterIntent─▶ the character
//!                  ├─ CameraRig ─▶ place the camera on the character
//!                  ├─ step: run_frame (every system, the renderer included)
//!                  └─ record ─▶ FrameRecord, and a Tile when one is due
//! ```
//!
//! Nothing below the intent is stood in for. The character is the player the
//! level spawns, with its capsule, constraint, actuator and animator; physics,
//! buoyancy and the water are the game's own, and each tile is a frame the
//! game drew. What the tool contributes is the brain and the camera.

use std::path::Path;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use specs::{Entity, Join, WorldExt};

use crate::animation::{CharacterAnimator, CharacterRigConfig};
use crate::character::{CharacterIntent, CharacterState, Grounding, Immersion, LocomotionConfig};
use crate::components::{Position, RigidBodyComponent, Rotation};
use crate::core::error::{EngineError, EngineResult};
use crate::level::load_level;
use crate::player::Player;
use crate::render_perf::GameHarness;
use crate::systems::PhysicsResource;
use crate::water::WaterWorld;

use super::pilot::Pilot;
use super::record::{tilt_degrees, FrameRecord, Take, Tile};
use super::scenario::Scenario;

/// How a run is played and filmed.
#[derive(Clone, Copy, Debug)]
pub struct RunConfig {
    /// Display rate. The game is locked to 30 Hz by the display it runs on.
    pub frame_rate: f32,
    /// Render target size of each tile.
    pub width: u32,
    pub height: u32,
    /// How many tiles to film, evenly spaced across `from..to` of the run.
    /// Zero films nothing and skips the readback.
    pub tiles: usize,
    /// Fraction of the script where filming starts and stops.
    pub from: f32,
    pub to: f32,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            frame_rate: 30.0,
            width: 480,
            height: 320,
            tiles: 12,
            from: 0.0,
            to: 1.0,
        }
    }
}

/// Play `scenario` to the end of its script.
pub fn run(scenario: &Scenario, config: &RunConfig) -> EngineResult<Take> {
    let level = load_level(Path::new(scenario.level))
        .map_err(|e| EngineError::InvalidState(format!("{}: {e:?}", scenario.level)))?;
    let dt = 1.0 / config.frame_rate.max(1.0);
    let mut harness = GameHarness::open(&level, config.width, config.height, dt)?;

    let character = take_character(&mut harness)?;
    place_character(&mut harness, character, scenario)?;

    let duration = scenario.script.duration();
    let tile_times = tile_times(duration, config);
    let mut next_tile = 0;

    let mut pilot = Pilot::new(&scenario.script);
    let mut frames = Vec::new();
    let mut tiles = Vec::new();
    let mut time = 0.0;
    let mut body = body_position(&harness, character);
    let mut surface = None;

    while let Some((intent, beat)) = pilot.intent(time, dt) {
        drive(&mut harness, character, intent);
        let (eye, look) = scenario.camera.frame(body, surface);
        harness.place_camera(eye, look);
        harness.step();

        let record = record(&harness, character, frames.len(), time, beat.label)?;
        body = record.body;
        surface = record.level;

        if next_tile < tile_times.len() && time + 0.5 * dt >= tile_times[next_tile] {
            next_tile += 1;
            tiles.push(Tile {
                caption: caption(&record),
                image: harness.snapshot()?,
            });
        }

        frames.push(record);
        time += dt;
    }

    Ok(Take {
        scenario: scenario.name.to_string(),
        frames,
        tiles,
    })
}

/// Times at which to film a tile.
fn tile_times(duration: f32, config: &RunConfig) -> Vec<f32> {
    if config.tiles == 0 {
        return Vec::new();
    }
    let (from, to) = (config.from * duration, config.to * duration);
    let span = (to - from).max(0.0);
    let steps = (config.tiles - 1).max(1) as f32;
    (0..config.tiles)
        .map(|i| from + span * i as f32 / steps)
        .collect()
}

/// The level's player, with the keyboard disconnected from it.
///
/// Dropping `Player` leaves everything that makes it a character — intent,
/// state, actuator, animator — and removes only the one system that writes
/// its intent, which is what makes the pilot its brain.
fn take_character(harness: &mut GameHarness) -> EngineResult<Entity> {
    let world = harness.world_mut();
    let entity = {
        let entities = world.entities();
        let players = world.read_storage::<Player>();
        (&entities, &players).join().map(|(e, _)| e).next()
    }
    .ok_or_else(|| EngineError::InvalidState("the level spawned no player".to_string()))?;
    world.write_storage::<Player>().remove(entity);
    Ok(entity)
}

/// Stand the character at the scenario's start, facing its way: on the ground,
/// or floating at the surface if the start is water too deep to stand in.
fn place_character(
    harness: &mut GameHarness,
    character: Entity,
    scenario: &Scenario,
) -> EngineResult<()> {
    let (x, z) = scenario.start;
    let ground = harness
        .surface_at(x, z)
        .ok_or_else(|| EngineError::InvalidState(format!("no ground at ({x}, {z})")))?;
    let world = harness.world_mut();
    let half_height = world
        .read_storage::<LocomotionConfig>()
        .get(character)
        .map(|c| c.collider_half_height)
        .unwrap_or(0.5);
    let standing = ground.y + half_height;
    let surface = world
        .try_fetch::<WaterWorld>()
        .and_then(|water| water.query().level_at(Point3::new(x, standing, z)));
    let y = match surface {
        Some(level) if level > standing => level,
        _ => standing,
    };
    let position = Point3::new(x, y, z);
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), scenario.yaw);

    let handle = world
        .read_storage::<RigidBodyComponent>()
        .get(character)
        .map(|b| b.0)
        .ok_or_else(|| EngineError::InvalidState("the player has no body".to_string()))?;
    {
        let mut physics = world.write_resource::<PhysicsResource>();
        let body = physics
            .world
            .body_mut(handle)
            .ok_or_else(|| EngineError::InvalidState("the player's body is gone".to_string()))?;
        body.set_position(position);
        body.set_rotation(rotation);
        body.set_linear_velocity(Vector3::zeros());
        body.set_angular_velocity(Vector3::zeros());
    }
    if let Some(p) = world.write_storage::<Position>().get_mut(character) {
        p.0 = position.coords;
    }
    if let Some(r) = world.write_storage::<Rotation>().get_mut(character) {
        r.0 = scenario.yaw;
    }
    // An animator built where the level spawned the player would open the run
    // with both feet planted somewhere else and a stride to recover them.
    let animator = CharacterAnimator::new(
        CharacterRigConfig::default(),
        position,
        half_height,
        scenario.yaw,
    );
    world
        .write_storage::<CharacterAnimator>()
        .insert(character, animator)
        .map_err(|e| EngineError::InvalidState(format!("{e:?}")))?;
    Ok(())
}

fn drive(harness: &mut GameHarness, character: Entity, intent: CharacterIntent) {
    let world = harness.world_mut();
    if let Some(slot) = world.write_storage::<CharacterIntent>().get_mut(character) {
        *slot = intent;
    }
}

fn body_position(harness: &GameHarness, character: Entity) -> Point3<f32> {
    harness
        .world()
        .read_storage::<Position>()
        .get(character)
        .map(|p| Point3::from(p.0))
        .unwrap_or_else(Point3::origin)
}

/// Read the frame the game just ran off the character.
fn record(
    harness: &GameHarness,
    character: Entity,
    index: usize,
    time: f32,
    beat: &'static str,
) -> EngineResult<FrameRecord> {
    let world = harness.world();
    let missing = |what: &str| EngineError::InvalidState(format!("the character has no {what}"));

    let states = world.read_storage::<CharacterState>();
    let state = states.get(character).ok_or_else(|| missing("state"))?;
    let animators = world.read_storage::<CharacterAnimator>();
    let animator = animators
        .get(character)
        .ok_or_else(|| missing("animator"))?;
    let handle = world
        .read_storage::<RigidBodyComponent>()
        .get(character)
        .map(|b| b.0)
        .ok_or_else(|| missing("body"))?;
    let grounded = world
        .read_storage::<Grounding>()
        .get(character)
        .map_or(false, |g| g.is_grounded);

    let physics = world.read_resource::<PhysicsResource>();
    let body = physics
        .world
        .body(handle)
        .ok_or_else(|| missing("live body"))?;
    let position = body.position();

    let water = world
        .read_storage::<Immersion>()
        .get(character)
        .and_then(|i| i.water);

    Ok(FrameRecord {
        index,
        time,
        beat,
        locomotion: state.locomotion.tag().to_string(),
        pose: animator.pose_state.tag(),
        upper: format!("{:?}", animator.upper_state.transition_key()).to_lowercase(),
        body: position,
        velocity: body.linear_velocity(),
        body_pitch: tilt_degrees(&body.rotation()),
        pitch_asked: state.body_pitch.to_degrees(),
        grounded,
        surface: water.map(|w| w.surface),
        level: water.map(|w| w.level),
        floor: water.map(|w| w.floor),
        pelvis: animator.skeleton.pelvis,
        head: animator.skeleton.head,
    })
}

fn caption(record: &FrameRecord) -> String {
    format!(
        "{:.2}s {} | {} {} | {:.0}°",
        record.time, record.beat, record.locomotion, record.pose, record.body_pitch
    )
}
