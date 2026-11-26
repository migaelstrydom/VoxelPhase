//! Blows on the player's own body in a stepped world: landings and blasts.

use nalgebra::{Point3, Vector3};
use specs::{Builder, RunNow, World, WorldExt};

use super::{worst_blow, Band, Blow, Death, Harm, HarmConfig, HarmSystem};
use crate::app::spawners::create_player_body;
use crate::character::{CharacterState, LocomotionConfig};
use crate::components::RigidBodyComponent;
use crate::damage::Dead;
use crate::debug::{DebugLines, DebugLog};
use crate::explosion::Explosion;
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::{
    Atmosphere, PhysicsConfig, PhysicsImpulse, PhysicsWorld, RigidBodyHandle, SequentialStepper,
    Stepper,
};
use crate::systems::{LevelRestart, PhysicsResource};
use crate::time::Time;

const FRAME_DT: f32 = 1.0 / 60.0;

/// Flat ground in vacuum, the player's body on it, stepped as the game steps.
struct Yard {
    world: PhysicsWorld,
    stepper: SequentialStepper,
    ground: FlatQuadGeometry,
    debug: DebugLines,
    body: RigidBodyHandle,
    config: HarmConfig,
}

impl Yard {
    /// The player's body standing with its feet `height` m off the ground,
    /// clear of the ground quad's diagonal (x = z).
    fn new(height: f32) -> Self {
        let mut config = PhysicsConfig::default();
        config.atmosphere = Atmosphere::VACUUM;
        let mut world = PhysicsWorld::new(config);
        let locomotion = LocomotionConfig::player();
        let centre = Point3::new(3.0, locomotion.collider_half_height + height, 1.0);
        let (body, _) = create_player_body(&mut world, centre, &locomotion);
        world.wake_body(body);
        Self {
            world,
            stepper: SequentialStepper::new(1.0 / 240.0, 12),
            ground: FlatQuadGeometry::new(50.0),
            debug: DebugLines::default(),
            body,
            config: HarmConfig::default(),
        }
    }

    fn step(&mut self, impulses: &[PhysicsImpulse]) {
        self.stepper.step(
            &mut self.world,
            FRAME_DT,
            &self.ground,
            impulses,
            &[],
            &mut self.debug,
        );
    }

    fn mass(&self) -> f32 {
        self.world.body(self.body).unwrap().mass()
    }

    /// Step until the body is struck, at most `seconds`: the worst blow of
    /// that step, read as it would be `in_control` or not.
    fn first_blow(&mut self, seconds: f32, in_control: bool) -> Option<Blow> {
        for _ in 0..(seconds / FRAME_DT) as usize {
            let before = self.world.body(self.body).unwrap().linear_velocity();
            self.step(&[]);
            let blow = worst_blow(
                &self.world,
                self.body,
                self.mass(),
                before,
                in_control,
                &self.config,
            );
            if blow.is_some() {
                return blow;
            }
        }
        None
    }
}

/// The drop from which a body lands at `speed`, m.
fn drop_for(speed: f32) -> f32 {
    speed * speed / (2.0 * 9.81)
}

/// Upright, the body lands on its feet and is braced: a full jump's
/// landing is light, a drop past what the legs take knocks it down.
#[test]
fn the_player_lands_a_jump_on_its_feet_and_a_long_drop_knocks_it_down() {
    let mut jump = Yard::new(drop_for(7.0));
    let blow = jump.first_blow(2.0, true).expect("the landing");
    assert!(blow.braced);
    assert!(
        (blow.delta_v - 7.0).abs() < 0.2,
        "landed at {}",
        blow.delta_v
    );
    assert_eq!(blow.band(&jump.config), Band::Light);

    let mut drop = Yard::new(drop_for(12.0));
    let blow = drop.first_blow(3.0, true).expect("the landing");
    assert_eq!(blow.band(&drop.config), Band::Heavy, "{blow:?}");
}

/// The same jump's landing, limp, knocks the body down.
#[test]
fn a_limp_body_is_knocked_down_by_a_jumps_landing() {
    let mut yard = Yard::new(drop_for(7.0));
    let blow = yard.first_blow(2.0, false).expect("the landing");
    assert!(!blow.braced);
    assert_eq!(blow.band(&yard.config), Band::Heavy);
}

/// A grenade's blast knocks the body down close in (to 2.75 m), however
/// close, and only shoves it out to the edge of its reach (5 m). Within
/// about 1.4 m its shove is as sudden as a deadly hit: it kills only by
/// what it throws the body into.
#[test]
fn a_blast_knocks_down_close_and_shoves_at_its_edge() {
    for (distance, band) in [(1.0, Band::Heavy), (2.5, Band::Heavy), (4.0, Band::Light)] {
        let mut yard = Yard::new(0.0);
        yard.step(&[]);
        let centre = yard.world.body(yard.body).unwrap().position();
        let blast = Explosion::new(centre - Vector3::x() * distance).physics_impulse();
        yard.step(&[blast]);
        let blow = worst_blow(
            &yard.world,
            yard.body,
            yard.mass(),
            Vector3::zeros(),
            true,
            &yard.config,
        )
        .expect("the blast's shove");
        if distance < 1.4 {
            assert!(
                blow.sudden(&yard.config) >= yard.config.deadly,
                "{distance} m: {:.1} m/s",
                blow.delta_v
            );
        }
        assert_eq!(
            blow.band(&yard.config),
            band,
            "{distance} m: {:.1} m/s",
            blow.delta_v
        );
    }
}

/// A creature's strike is judged as a shove: a peck that changes the
/// body's speed by 6 m/s knocks it down, and one as sudden as a deadly hit
/// still only knocks it down.
#[test]
fn a_strike_knocks_down_and_never_kills() {
    for (speed, band) in [(3.0, Band::Light), (6.0, Band::Heavy), (12.0, Band::Heavy)] {
        let mut yard = Yard::new(0.0);
        yard.step(&[]);
        let centre = yard.world.body(yard.body).unwrap().position();
        yard.step(&[PhysicsImpulse::Strike {
            body: yard.body,
            impulse: Vector3::x() * speed * yard.mass(),
            point: centre,
        }]);
        let blow = worst_blow(
            &yard.world,
            yard.body,
            yard.mass(),
            Vector3::zeros(),
            true,
            &yard.config,
        )
        .expect("the strike's shove");
        assert!((blow.delta_v - speed).abs() < 1e-3, "{blow:?}");
        assert_eq!(blow.band(&yard.config), band, "{speed} m/s");
    }
}

/// What a body whose harm ends in `death` comes to, `seconds` after it is
/// dropped from where it lands at `speed`, judged by `HarmSystem` as it is
/// in the game: whether the room restarted, and how long its corpse
/// lingers if it died. It is no character, so it cannot brace.
fn dropped_and_judged(death: Death, speed: f32, seconds: f32) -> (bool, Option<Option<f32>>) {
    let Yard {
        world: physics,
        mut stepper,
        ground,
        mut debug,
        body,
        ..
    } = Yard::new(drop_for(speed));
    let mut world = World::new();
    world.register::<Harm>();
    world.register::<Dead>();
    world.register::<RigidBodyComponent>();
    world.register::<CharacterState>();
    world.insert(Time::fixed(FRAME_DT));
    world.insert(LevelRestart::default());
    world.insert(DebugLog::default());
    world.insert(PhysicsResource::new(
        physics,
        Box::new(SequentialStepper::new(1.0 / 240.0, 12)),
    ));
    let entity = world
        .create_entity()
        .with(Harm {
            death,
            ..Harm::default()
        })
        .with(RigidBodyComponent(body))
        .build();

    for _ in 0..(seconds / FRAME_DT) as usize {
        let mut physics = world.write_resource::<PhysicsResource>();
        stepper.step(&mut physics.world, FRAME_DT, &ground, &[], &[], &mut debug);
        drop(physics);
        HarmSystem.run_now(&world);
    }
    let restarted = world.read_resource::<LevelRestart>().requested;
    let lingers = world
        .read_storage::<Dead>()
        .get(entity)
        .map(|dead| dead.lingers);
    (restarted, lingers)
}

/// A deadly landing restarts the room for the player, a beat later; a
/// creature's body instead dies where it lies, to be cleared away later.
#[test]
fn a_deadly_blow_restarts_the_player_s_room_and_kills_a_creature() {
    let deadly = 2.0 * HarmConfig::default().deadly;
    assert_eq!(
        dropped_and_judged(Death::RestartsRoom, deadly, 3.0),
        (true, None)
    );
    let creature = Death::Dies { lingers: 6.0 };
    assert_eq!(
        dropped_and_judged(creature, deadly, 3.0),
        (false, Some(Some(6.0)))
    );
}

/// A heavy landing kills nothing: the creature is knocked down, and lives.
#[test]
fn a_heavy_blow_kills_no_creature() {
    let heavy = 0.5 * (HarmConfig::default().heavy + HarmConfig::default().deadly);
    let creature = Death::Dies { lingers: 6.0 };
    assert_eq!(dropped_and_judged(creature, heavy, 3.0), (false, None));
}
