//! Blows on the player's own body in a stepped world: landings and blasts.

use nalgebra::{Point3, Vector3};

use super::{worst_blow, Band, Blow, HarmConfig};
use crate::app::spawners::create_player_body;
use crate::character::LocomotionConfig;
use crate::debug::DebugLines;
use crate::explosion::Explosion;
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::{
    Atmosphere, PhysicsConfig, PhysicsImpulse, PhysicsWorld, RigidBodyHandle, SequentialStepper,
    Stepper,
};

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
