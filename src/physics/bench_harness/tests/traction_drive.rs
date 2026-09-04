//! Stage 0 acceptance tests for the traction drive (docs/TRACTION_DRIVE_DESIGN.md §9).
//!
//! These eight scenarios pin down what `VelocityDriven` does *today*, before
//! the mechanism underneath it is replaced. The current behaviour is only
//! partly understood and partly accidental, so without them a regression and a
//! correction are indistinguishable.
//!
//! Two kinds of assertion live here and the difference matters:
//!
//! - **Specification** — this must remain true through every later stage.
//! - **Characterisation** — this is what the engine does now. Some of these
//!   record behaviour the design intends to *change* (`edge_walk`'s sign is
//!   the headline), and others are numbers that depend on decision D1 (§11).
//!   A characterisation failing is a signal to read the design, not a bug.
//!
//! Every test drives the real `PhysicsWorld` directly, replaying the two ECS
//! systems that feed it — `CharacterControlSystem` writing the walk rule into
//! `Velocity`, and `MovingPlatformSystem` overwriting all three axes — by hand
//! in dispatcher order. That keeps the ECS out of the harness while exercising
//! exactly the round trip §2 of the design describes: last frame's *measured*
//! velocity, partially overwritten, handed back down as this frame's target.

use nalgebra::{Point3, UnitVector3, Vector3};

use super::super::geometry::{FlatQuadGeometry, WallAndFloorGeometry};
use crate::character::LocomotionConfig;
use crate::debug::DebugLines;
use crate::physics::constraint::ConstraintKind;
use crate::physics::world::PhysicsConfig;
use crate::physics::{
    ColliderDesc, FrictionModel, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry,
};
use crate::platform::MovingPlatform;

/// One render frame, matching the game's fixed step budget.
const FRAME_DT: f32 = 1.0 / 60.0;
const SUBSTEPS: u32 = 4;
const SUBSTEP_DT: f32 = FRAME_DT / SUBSTEPS as f32;

/// Platform half extents, as spawned by the level loader's lift.
const PLATFORM_HALF_EXTENTS: Vector3<f32> = Vector3::new(2.0, 0.3, 2.0);
/// The platform motor's acceleration budget (`VelocityDriven::max_accel` on a
/// platform entity).
const MOTOR_MAX_ACCEL: f32 = 40.0;

/// A world with sleeping disabled — a sleeping body stops answering the drive,
/// which turns every one of these measurements into noise.
fn bench_world() -> PhysicsWorld {
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    PhysicsWorld::new(config)
}

fn position_of(world: &PhysicsWorld, body: RigidBodyHandle) -> Vector3<f32> {
    let p = world.body(body).unwrap().position();
    Vector3::new(p.x, p.y, p.z)
}

fn linear_velocity_of(world: &PhysicsWorld, body: RigidBodyHandle) -> Vector3<f32> {
    world.body(body).unwrap().linear_velocity()
}

/// Advance one render frame: refresh contacts once, then run the substeps.
fn advance(world: &mut PhysicsWorld, geometry: &dyn StaticGeometry, debug: &mut DebugLines) {
    world.update_contacts(SUBSTEP_DT, SUBSTEPS, geometry, &[], debug);
    for _ in 0..SUBSTEPS {
        world.substep(SUBSTEP_DT, geometry, &[]);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Rigs: the two ECS writers, replayed by hand
// ═══════════════════════════════════════════════════════════════════════════

/// A character body driven the way `CharacterControlSystem` drives the player:
/// the measured velocity is read back, `x` and `z` are steered toward the gait
/// target at `ground_accel`, and `y` is left exactly as physics left it.
struct Walker {
    body: RigidBodyHandle,
    config: LocomotionConfig,
    /// Unit walk direction in the XZ plane; zero means "stand still".
    intent: Vector3<f32>,
}

impl Walker {
    /// Spawn the player's body: capsule, held upright, gripping only what
    /// holds it up.
    fn spawn(world: &mut PhysicsWorld, feet_at: Vector3<f32>) -> Self {
        let config = LocomotionConfig::player();
        // `ColliderShape::Capsule::half_height` spans the caps too, so the
        // centre sits exactly that far above the soles.
        let centre = feet_at + Vector3::new(0.0, config.collider_half_height, 0.0);
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::from(centre))
                .gravity_scale(1.0)
                .linear_damping(0.0)
                .angular_damping(0.95),
        );
        let _ = world.attach_collider(
            body,
            ColliderDesc::capsule(config.collider_half_height, config.collider_radius)
                .density(800.0)
                .restitution(0.0)
                .friction_model(FrictionModel::Isotropic(0.8)),
        );
        // The player's actuator declaration, replayed by hand: nothing that is
        // not holding them up may drag them.
        world.set_body_non_support_grip(body, 0.0);
        world
            .body_mut(body)
            .unwrap()
            .scale_local_inertia(Vector3::new(1.0, 50.0, 1.0));
        let _ = world.create_constraint(ConstraintKind::KeepUpright {
            body,
            target_up: UnitVector3::new_normalize(Vector3::y()),
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });

        Self {
            body,
            config,
            intent: Vector3::zeros(),
        }
    }

    fn walking(mut self, direction: Vector3<f32>) -> Self {
        self.intent = direction.normalize();
        self
    }

    /// The same character with the stick released.
    fn standing(&self) -> Self {
        Self {
            body: self.body,
            config: self.config.clone(),
            intent: Vector3::zeros(),
        }
    }

    fn mass(&self, world: &PhysicsWorld) -> f32 {
        world.body(self.body).unwrap().mass()
    }

    /// `CharacterControlSystem` then `PhysicsSyncSystem`, once per frame.
    fn drive(&self, world: &mut PhysicsWorld) {
        let measured = linear_velocity_of(world, self.body);
        let target_planar = self.intent * self.config.walk_speed;
        let max_delta = self.config.ground_accel * FRAME_DT;
        let target = Vector3::new(
            move_toward(measured.x, target_planar.x, max_delta),
            measured.y,
            move_toward(measured.z, target_planar.z, max_delta),
        );
        let _ = world.set_body_velocity_drive(self.body, target, Vector3::zeros(), 500.0, 500.0);
    }
}

fn move_toward(current: f32, target: f32, max_delta: f32) -> f32 {
    let diff = target - current;
    if diff.abs() <= max_delta {
        target
    } else {
        current + diff.signum() * max_delta
    }
}

/// A powered platform, driven the way `MovingPlatformSystem` drives it: the
/// route servo overwrites all three axes of the target every frame.
struct Platform {
    body: RigidBodyHandle,
    route: MovingPlatform,
}

impl Platform {
    fn spawn(world: &mut PhysicsWorld, from: Vector3<f32>, to: Vector3<f32>, speed: f32) -> Self {
        let body = Self::spawn_body(world, from, 1.0);
        Self {
            body,
            route: MovingPlatform::new(from, to, speed),
        }
    }

    fn spawn_body(
        world: &mut PhysicsWorld,
        at: Vector3<f32>,
        gravity_scale: f32,
    ) -> RigidBodyHandle {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::from(at))
                .gravity_scale(gravity_scale)
                .linear_damping(0.0)
                .angular_damping(0.05),
        );
        let _ = world.attach_collider(
            body,
            ColliderDesc::box_shape(PLATFORM_HALF_EXTENTS)
                .density(300.0)
                .friction(0.9),
        );
        // Tilt is locked; yaw is deliberately free, which is what makes the
        // edge-walk sign observable at all.
        let _ = world.create_constraint(ConstraintKind::KeepUpright {
            body,
            target_up: UnitVector3::new_normalize(Vector3::y()),
            compliance: 0.0,
            max_impulse: f32::INFINITY,
        });
        body
    }

    /// The deck surface: where a passenger's feet go.
    fn deck_y(&self, world: &PhysicsWorld) -> f32 {
        position_of(world, self.body).y + PLATFORM_HALF_EXTENTS.y
    }

    fn drive(&mut self, world: &mut PhysicsWorld) {
        let position = position_of(world, self.body);
        self.route.update_heading(&position);
        let _ = world.set_body_velocity_drive(
            self.body,
            self.route.target_velocity(&position),
            Vector3::zeros(),
            MOTOR_MAX_ACCEL,
            0.0,
        );
    }
}

/// A pushable box resting on the ground.
fn spawn_crate(
    world: &mut PhysicsWorld,
    centre: Vector3<f32>,
    half_extents: Vector3<f32>,
    density: f32,
) -> RigidBodyHandle {
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::from(centre))
            .gravity_scale(1.0)
            .linear_damping(0.0)
            .angular_damping(0.05),
    );
    let _ = world.attach_collider(
        body,
        ColliderDesc::box_shape(half_extents)
            .density(density)
            .restitution(0.0)
            .friction(0.8),
    );
    body
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Vertical lift carry — passenger rides at platform speed, no slip
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification.** A passenger standing on a climbing lift rides it: the two
/// bodies hold the same vertical speed and the gap between them does not open.
///
/// This works today for the reason §2 of the requirements gives — the walk rule
/// never writes `y`, so the drive target's vertical component is "keep doing
/// what you are already doing", and the drive ratifies the carry instead of
/// braking it.
#[test]
fn vertical_lift_carry() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let mut platform = Platform::spawn(
        &mut world,
        Vector3::new(0.0, 6.0, 0.0),
        Vector3::new(0.0, 30.0, 0.0),
        2.0,
    );
    let deck = platform.deck_y(&world);
    let walker = Walker::spawn(&mut world, Vector3::new(0.0, deck + 0.02, 0.0));

    // Half a second to settle the passenger on to the deck, then measure.
    for _ in 0..30 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }

    let settle_gap = position_of(&world, walker.body).y - position_of(&world, platform.body).y;
    let mut worst_gap_error: f32 = 0.0;
    let mut worst_speed_error: f32 = 0.0;

    for _ in 0..240 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);

        let gap = position_of(&world, walker.body).y - position_of(&world, platform.body).y;
        worst_gap_error = worst_gap_error.max((gap - settle_gap).abs());
        let slip =
            linear_velocity_of(&world, walker.body).y - linear_velocity_of(&world, platform.body).y;
        worst_speed_error = worst_speed_error.max(slip.abs());
    }

    let climb = linear_velocity_of(&world, platform.body).y;
    eprintln!(
        "vertical carry: climb={climb:.4} worst_gap_error={worst_gap_error:.4} \
         worst_slip={worst_speed_error:.4}"
    );

    assert!(climb > 1.5, "lift should still be climbing: {climb:.4}");
    assert!(
        worst_gap_error < 0.02,
        "passenger should not slip on the deck: gap moved {worst_gap_error:.4} m"
    );
    assert!(
        worst_speed_error < 0.25,
        "passenger should ride at the platform's speed: worst slip \
         {worst_speed_error:.4} m/s"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Reversal hover — passenger goes ballistic, platform catches them
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification.** When the lift reverses at the top, the passenger keeps
/// rising — their drive target still asserts the velocity it last measured —
/// and the platform catches them on the way back down.
///
/// The hover is not a bug. It is the drive target holding the last measured
/// velocity while `integrate_forces` bleeds gravity into it one substep at a
/// time. What must remain true is that it *ends*: the passenger comes back down
/// and lands on the deck rather than sailing away.
#[test]
fn reversal_hover() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let mut platform = Platform::spawn(
        &mut world,
        Vector3::new(0.0, 6.0, 0.0),
        Vector3::new(0.0, 12.0, 0.0),
        2.0,
    );
    let deck = platform.deck_y(&world);
    let walker = Walker::spawn(&mut world, Vector3::new(0.0, deck + 0.02, 0.0));

    for _ in 0..30 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }
    let riding_gap = position_of(&world, walker.body).y - position_of(&world, platform.body).y;

    // Run to the reversal and past it.
    let mut heading = platform.route.heading;
    let mut frames_to_reversal = 0;
    while platform.route.heading == heading && frames_to_reversal < 600 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        frames_to_reversal += 1;
    }
    heading = platform.route.heading;
    assert!(frames_to_reversal < 600, "lift never reached its endpoint");

    let mut peak_gap = riding_gap;
    let mut rose_after_reversal = false;
    let mut reunited = false;
    for _ in 0..180 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);

        let gap = position_of(&world, walker.body).y - position_of(&world, platform.body).y;
        peak_gap = peak_gap.max(gap);
        if linear_velocity_of(&world, walker.body).y > 0.1 {
            rose_after_reversal = true;
        }
        if rose_after_reversal && (gap - riding_gap).abs() < 0.02 {
            reunited = true;
        }
    }

    eprintln!(
        "reversal hover: riding_gap={riding_gap:.4} peak_gap={peak_gap:.4} \
         hovered={rose_after_reversal} caught={reunited} heading={heading:.1}"
    );

    assert!(
        peak_gap > riding_gap + 0.05,
        "passenger should go ballistic at the reversal: peak gap {peak_gap:.4} \
         vs riding {riding_gap:.4}"
    );
    assert!(
        reunited,
        "platform should catch the passenger again: peak gap {peak_gap:.4}, \
         riding gap {riding_gap:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Horizontal lift carry — passenger holds station on a running platform
// ═══════════════════════════════════════════════════════════════════════════

/// **Characterisation.** The case §2 of the requirements calls untested: on a
/// horizontal run, the walk rule *does* write the axes the carry acts on, so an
/// idle passenger's drive target is steered toward zero world velocity at
/// `ground_accel` while friction drags them along with the deck.
///
/// This test records who wins. It is the one number in Stage 0 that nobody
/// currently knows, and R4 is the requirement that will change it: after the
/// traction drive, "stand still" means still *relative to the deck*.
#[test]
fn horizontal_lift_carry() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let mut platform = Platform::spawn(
        &mut world,
        Vector3::new(-20.0, 10.0, 0.0),
        Vector3::new(20.0, 10.0, 0.0),
        3.0,
    );
    let deck = platform.deck_y(&world);
    let walker = Walker::spawn(&mut world, Vector3::new(0.0, deck + 0.02, 0.0));

    for _ in 0..30 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }
    let start_offset = position_of(&world, walker.body) - position_of(&world, platform.body);

    let mut worst_drift: f32 = 0.0;
    for _ in 0..180 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        let offset = position_of(&world, walker.body) - position_of(&world, platform.body);
        worst_drift = worst_drift.max((offset - start_offset).norm());
    }

    let platform_speed = linear_velocity_of(&world, platform.body).x;
    let walker_speed = linear_velocity_of(&world, walker.body).x;
    let on_deck = position_of(&world, walker.body).y > platform.deck_y(&world) - 0.1;
    eprintln!(
        "horizontal carry: platform={platform_speed:.4} passenger={walker_speed:.4} \
         drift={worst_drift:.4} still_aboard={on_deck}"
    );

    // The answer, recorded: the passenger is not carried at all. The walk rule
    // writes the two axes the carry acts on, steering the drive target to zero
    // *world* velocity at 40 m/s², so the deck slides out from under a standing
    // passenger in under a second and drops them off the back.
    //
    // This is what R4 (support-relative targets) exists to fix: after the
    // traction drive, "stand still" means still relative to the deck, and this
    // test inverts into the specification its name claims — drift near zero and
    // `still_aboard` true.
    assert!(
        walker_speed.abs() < 1.0,
        "characterisation: the passenger is braked to a standstill in world \
         space rather than carried ({walker_speed:.4} m/s against a deck \
         running at {platform_speed:.4})"
    );
    assert!(
        worst_drift > 2.0,
        "characterisation: the passenger should be left behind by the deck. \
         Drift {worst_drift:.4} m — if this now fails because the drift went to \
         zero, R4 has landed and this test becomes a specification."
    );
    assert!(
        !on_deck,
        "characterisation: the passenger falls off the back of a running \
         platform (y = {:.3}, deck {:.3})",
        position_of(&world, walker.body).y,
        platform.deck_y(&world)
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Cruise under load — authored speed against gravity and a passenger
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification.** A loaded lift climbs at its authored speed less one frame
/// of gravity, exactly as the unloaded one does.
///
/// The passenger's weight costs the platform nothing today because the motor is
/// reactionless: the load's contact impulse is never folded into the drive
/// target, only gravity is. After Stage 4 the motor becomes a medium-anchored
/// row and this must still hold — a lift is a thruster, and a thruster does not
/// care what it carries.
#[test]
fn cruise_under_load() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let gravity = world.config().gravity.norm();
    let mut debug = DebugLines::default();

    let speed = 2.0;
    let mut platform = Platform::spawn(
        &mut world,
        Vector3::new(0.0, 10.0, 0.0),
        Vector3::new(0.0, 60.0, 0.0),
        speed,
    );
    let deck = platform.deck_y(&world);
    let walker = Walker::spawn(&mut world, Vector3::new(0.0, deck + 0.02, 0.0));

    for _ in 0..120 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }

    let expected = speed - gravity * FRAME_DT;
    let climb = linear_velocity_of(&world, platform.body).y;
    let load = walker.mass(&world);
    eprintln!(
        "cruise under load: climb={climb:.4} expected={expected:.4} \
         passenger_mass={load:.1}kg"
    );

    assert!(
        (climb - expected).abs() < 0.05,
        "loaded lift should hold cruise speed less one frame of gravity: \
         {climb:.4} vs {expected:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Edge walk — which way does the platform yaw?
// ═══════════════════════════════════════════════════════════════════════════

/// **Characterisation, and the one the design exists to overturn.** A character
/// walking `+X` along the `+Z` edge of a free-yawing platform should torque it
/// `−Y`: the platform is what pushes the walker, so the reaction recoils it the
/// other way (requirements R1).
///
/// Today the drive is reactionless, so the walker behaves as a conveyor belt —
/// friction drags the deck *forwards* with them — and the platform yaws `+Y`.
/// This test asserts the wrong sign on purpose. When Stage 5 lands, it flips,
/// and this assertion is rewritten as a specification.
///
/// The platform hovers (`gravity_scale = 0`, no route servo) so that nothing
/// but the walker's friction touches its yaw.
#[test]
fn edge_walk_yaws_the_platform_with_the_walker() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let platform_centre = Vector3::new(0.0, 5.0, 0.0);
    let platform = Platform::spawn_body(&mut world, platform_centre, 0.0);
    let deck = platform_centre.y + PLATFORM_HALF_EXTENTS.y;

    // Feet near the +Z edge of the deck, walking +X across it.
    let walker = Walker::spawn(
        &mut world,
        Vector3::new(-1.5, deck + 0.02, PLATFORM_HALF_EXTENTS.z - 0.4),
    )
    .walking(Vector3::x());

    // Stand still first so the passenger settles without a lateral kick.
    let idle = walker.standing();
    for _ in 0..30 {
        idle.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }

    let mut peak_yaw: f32 = 0.0;
    for _ in 0..120 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        let yaw = world.body(platform).unwrap().angular_velocity().y;
        if yaw.abs() > peak_yaw.abs() {
            peak_yaw = yaw;
        }
    }

    let walker_speed = linear_velocity_of(&world, walker.body).x;
    eprintln!(
        "edge walk: walker vx={walker_speed:.4} peak platform yaw={peak_yaw:.5} rad/s \
         (conservation demands negative)"
    );

    assert!(
        walker_speed > 1.0,
        "the walker should actually be walking: vx = {walker_speed:.4}"
    );
    assert!(
        peak_yaw > 0.0,
        "characterisation: the reactionless drive drags the deck along with the \
         walker, yawing it +Y. Conservation demands −Y — if this now fails with \
         a negative yaw, the traction drive has landed and this assertion should \
         be inverted into a specification. Measured {peak_yaw:.5} rad/s"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Crate push — how fast does a pushed crate go?
// ═══════════════════════════════════════════════════════════════════════════

/// **Characterisation (gated on D1).** A driven character walks into a crate
/// and pushes it. The requirement wants push speed to follow the mass ratio;
/// that claim is only honest once the drive is bounded by the contact, and
/// today it is not — the drive restores the character's velocity from an
/// infinite account every substep, so the crate is pushed at close to walk
/// speed regardless of how heavy it is.
///
/// What this records is the *current* answer plus the absence of jitter, which
/// is the half of the assertion that must survive D1 either way.
#[test]
fn crate_push() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let walker = Walker::spawn(&mut world, Vector3::new(0.0, 0.02, 0.0)).walking(Vector3::x());
    // Wide across the direction of travel: an off-centre hit squirts a cubic
    // crate sideways out of contact within a second, which measures the
    // walker's aim rather than the drive's authority.
    let crate_half = Vector3::new(0.4, 0.4, 1.5);
    let crate_body = spawn_crate(
        &mut world,
        Vector3::new(1.5, crate_half.y + 0.02, 0.0),
        crate_half,
        400.0,
    );

    let walker_mass = walker.mass(&world);
    let crate_mass = world.body(crate_body).unwrap().mass();
    let mass_ratio_speed = walker.config.walk_speed * walker_mass / (walker_mass + crate_mass);

    // Long enough for the walker to reach the crate, punt it, and catch up.
    let mut peak_push: f32 = 0.0;
    let mut peak_overtake: f32 = 0.0;
    for _ in 0..300 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        let push = linear_velocity_of(&world, crate_body).x;
        peak_push = peak_push.max(push);
        peak_overtake = peak_overtake.max(push - linear_velocity_of(&world, walker.body).x);
    }

    eprintln!(
        "crate push: walker={walker_mass:.1}kg crate={crate_mass:.1}kg \
         peak={peak_push:.4} walk_speed={:.1} mass_ratio_speed={mass_ratio_speed:.4} \
         peak_overtake={peak_overtake:.4}",
        walker.config.walk_speed
    );

    // Specification, whatever D1 decides: the crate cannot outrun the thing
    // pushing it. A crate that does is receiving momentum from nowhere.
    assert!(
        peak_overtake < 0.1,
        "the crate should never outrun the walker: overtook by \
         {peak_overtake:.4} m/s"
    );
    // Characterisation (gated on D1, §11). The crate is not pushed, it is
    // *punted*: one contact throws a 768 kg box to nearly the walker's own
    // walk speed, because the reactionless drive restores the walker's
    // velocity from an infinite account every substep. Mass ratio does not
    // enter into it.
    //
    // Under a contact-bounded drive this peak drops toward
    // `mass_ratio_speed`. What that number becomes is exactly what D1 decides,
    // so this assertion is promoted or rewritten then — not before.
    assert!(
        peak_push > 3.0 * mass_ratio_speed,
        "characterisation: the punt should far exceed the mass-ratio speed. \
         peak {peak_push:.4} vs mass-ratio {mass_ratio_speed:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Stack stability — a driven body resting on a stack does not excite it
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification.** An idle driven character standing on a box stack must not
/// stir it. The drive re-asserts a velocity every substep; if any of that leaks
/// into the stack, the tower walks or topples.
///
/// This is one of the two tests that decide whether the warm-start and
/// restitution special cases in `pipeline/solver.rs` can be deleted at Stage 7.
#[test]
fn stack_stability_under_a_driven_body() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let half = 0.5;
    let mut boxes = Vec::new();
    for i in 0..3 {
        boxes.push(spawn_crate(
            &mut world,
            Vector3::new(0.0, half + i as f32 * (2.0 * half + 0.002), 0.0),
            Vector3::repeat(half),
            500.0,
        ));
    }
    let top_of_stack = 3.0 * (2.0 * half) + 0.01;
    let walker = Walker::spawn(&mut world, Vector3::new(0.0, top_of_stack, 0.0));

    // Let the stack take the load.
    for _ in 0..120 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }
    let settled: Vec<Vector3<f32>> = boxes.iter().map(|b| position_of(&world, *b)).collect();

    let mut worst_lateral: f32 = 0.0;
    let mut worst_speed: f32 = 0.0;
    for _ in 0..300 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        for (b, start) in boxes.iter().zip(&settled) {
            let p = position_of(&world, *b);
            worst_lateral = worst_lateral.max((p.x - start.x).hypot(p.z - start.z));
            worst_speed = worst_speed.max(linear_velocity_of(&world, *b).norm());
        }
    }

    eprintln!(
        "stack under driven body: worst lateral drift={worst_lateral:.4} m, \
         worst box speed={worst_speed:.4} m/s"
    );

    assert!(
        worst_lateral < 0.05,
        "a standing driven body should not walk the stack sideways: \
         {worst_lateral:.4} m"
    );
    assert!(
        worst_speed < 0.5,
        "the stack should stay quiet under a driven body: {worst_speed:.4} m/s"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Driven body at rest — no jitter or restitution popping
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification.** A driven body commanded to stand still, wedged against a
/// wall, stays put: no buzzing on the floor, no popping off the wall.
///
/// This is what the warm-start persistence rule and the restitution suppression
/// added by `84a49b1` exist to protect, and it is the test that says whether
/// they can go when the drive stops re-asserting approach velocity every
/// substep.
#[test]
fn driven_body_at_rest_against_wall_and_floor() {
    let geometry = WallAndFloorGeometry::new(32.0, 6.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    // Standing on the floor with a shoulder on the wall. The wall in
    // `WallAndFloorGeometry` stands at x = 0 and the floor extends in +x, so
    // the body sits just clear of it and is pushed back by its own drive.
    let walker = Walker::spawn(&mut world, Vector3::new(0.4, 0.02, 0.0)).walking(-Vector3::x());

    for _ in 0..120 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }
    let settled = position_of(&world, walker.body);

    let mut worst_speed: f32 = 0.0;
    let mut worst_vertical: f32 = 0.0;
    let mut worst_offset: f32 = 0.0;
    for _ in 0..300 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        let v = linear_velocity_of(&world, walker.body);
        worst_speed = worst_speed.max(v.norm());
        worst_vertical = worst_vertical.max(v.y.abs());
        worst_offset = worst_offset.max((position_of(&world, walker.body) - settled).norm());
    }

    eprintln!(
        "driven body at rest: resting at x={:.4} (wall at 0, radius {:.2}), \
         worst speed={worst_speed:.4} vertical={worst_vertical:.4} drift={worst_offset:.4}",
        settled.x, walker.config.collider_radius
    );

    // Without this the rest of the test is vacuous: a body that never reached
    // the wall is trivially quiet.
    assert!(
        settled.x < walker.config.collider_radius + 0.05,
        "the body should be resting against the wall: x = {:.4}",
        settled.x
    );

    assert!(
        worst_vertical < 0.2,
        "a body held against a wall should not pop off the floor: \
         {worst_vertical:.4} m/s"
    );
    assert!(
        worst_offset < 0.05,
        "a body driven into a wall should hold station: drift \
         {worst_offset:.4} m"
    );
}
