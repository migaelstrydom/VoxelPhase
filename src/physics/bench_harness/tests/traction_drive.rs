//! Acceptance tests for the traction drive (docs/TRACTION_DRIVE_DESIGN.md §9).
//!
//! Written at Stage 0 against the old reactionless drive, so that a regression
//! and a correction could be told apart while it was replaced. Stage 5
//! replaced it: a drive is now friction with a non-zero target, solved at the
//! contacts holding a body up, and three of these read the other way round
//! than they did.
//!
//! Two kinds of assertion live here and the difference matters:
//!
//! - **Specification** — this must remain true through every later stage.
//! - **Characterisation** — this is what the engine does now, recorded rather
//!   than required. A characterisation failing is a signal to read the design,
//!   not a bug.
//!
//! `horizontal lift carry`, `edge walk` and `crate push` were characterisations
//! of the old mechanism, asserted in their broken form on purpose. All three
//! are specifications now, and each carries the number it used to print
//! alongside the one it prints today.
//!
//! Every test drives the real `PhysicsWorld` directly, replaying the two ECS
//! systems that feed it — `CharacterControlSystem` writing the walk rule into
//! `DriveIntent`, and `MovingPlatformSystem` writing the route servo's target —
//! by hand in dispatcher order. That keeps the ECS out of the harness while
//! exercising the seam the design is about.

use nalgebra::{Point3, UnitVector3, Vector3};

use super::super::geometry::{FlatQuadGeometry, RampGeometry, WallAndFloorGeometry};
use crate::character::LocomotionConfig;
use crate::debug::DebugLines;
use crate::physics::constraint::ConstraintKind;
use crate::physics::world::PhysicsConfig;
use crate::physics::{
    ColliderDesc, DriveCommand, FrictionModel, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
    StaticGeometry,
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

/// The player's declared drive gain (§11, decision D1). The honest bound on
/// this project's terrain is `0.8 · g = 7.85 m/s²`; five times it is 39.2,
/// which is the 40 m/s² `LocomotionConfig::ground_accel` the game was tuned
/// around and the reason that value is a specification rather than an
/// accident.
const WALKER_DRIVE_GAIN: f32 = 5.0;

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
/// the gait's planar target goes down as a velocity **relative to whatever is
/// holding the character up**, at the player's declared drive gain. Nothing is
/// read back and nothing is rate-limited here — the traction rows ramp the
/// body toward the target at the contact's own budget.
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
        let target = self.intent * self.config.walk_speed;
        let _ = world.set_body_drive(
            self.body,
            &DriveCommand::support(target, Vector3::zeros(), 500.0, 500.0)
                .with_drive_gain(WALKER_DRIVE_GAIN),
        );
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
        let _ = world.set_body_drive(
            self.body,
            &DriveCommand::medium(
                self.route.target_velocity(&position),
                Vector3::zeros(),
                MOTOR_MAX_ACCEL,
                0.0,
            ),
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
/// Nothing in the drive is involved: a tangential row is tangential by
/// construction, so a vertical carry on a level deck is the normal row and the
/// passenger's weight, exactly as it would be for a crate. The scenario earns
/// its place by saying so — whatever the drive does to the horizontal axes, it
/// must not reach this one.
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
/// rising and the platform catches them on the way back down.
///
/// The hover is not a bug and its cause is now the plain one: the deck stops
/// pushing up, the passenger is briefly a projectile, and gravity brings them
/// back. Under the old drive it was an artefact instead — the target held the
/// last measured velocity while force integration bled gravity into it a
/// substep at a time — which is why the gap it opens moved slightly at Stage 4
/// and again here. What must remain true is that it *ends*: the passenger comes
/// back down and lands on the deck rather than sailing away.
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

/// **Specification, inverted at Stage 5.** An idle passenger on a horizontal
/// run is carried by the deck and holds station on it.
///
/// Stage 0 recorded the opposite, and the inversion is the whole of R4. Under
/// the old drive the walk rule wrote the two axes the carry acts on, steering
/// the passenger's target toward zero *world* velocity at 40 m/s² while
/// friction tried to drag them along; the deck slid out from under them in
/// under a second and dropped them off the back, 12.79 m of drift. The target
/// is now stated relative to whatever holds the body up, so "stand still"
/// means still *relative to the deck*: an idle passenger asks for zero across
/// the surface they are on, the tangential rows deliver exactly that, and the
/// drift is 0.0005 m instead of 12.79.
///
/// One thing had to be fixed before the inversion could mean anything. The
/// Stage 0 rig spawned the passenger at the world origin while the platform's
/// route *started* at x = −20, so the passenger was never on the deck at all —
/// it fell 10 m and lay on the floor while the platform ran away overhead, and
/// the drift it recorded was the platform's own travel. The passenger now
/// spawns over the deck.
#[test]
fn horizontal_lift_carry() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let start = Vector3::new(-20.0, 10.0, 0.0);
    let mut platform = Platform::spawn(&mut world, start, Vector3::new(20.0, 10.0, 0.0), 3.0);
    let deck = platform.deck_y(&world);
    // Over the deck, not over where the deck's route happens to be centred.
    let walker = Walker::spawn(&mut world, Vector3::new(start.x, deck + 0.02, start.z));

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

    assert!(
        (walker_speed - platform_speed).abs() < 0.05,
        "the passenger should be carried at the deck's speed: {walker_speed:.4} \
         against a deck running at {platform_speed:.4}"
    );
    assert!(
        worst_drift < 0.05,
        "the passenger should hold station on the deck: drift {worst_drift:.4} m"
    );
    assert!(
        on_deck,
        "the passenger should still be aboard (y = {:.3}, deck {:.3})",
        position_of(&world, walker.body).y,
        platform.deck_y(&world)
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Cruise under load — authored speed against gravity and a passenger
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification.** A loaded lift climbs at its authored speed: a lift is a
/// thruster, and a thruster does not care what it carries.
///
/// The passenger's weight costs the platform nothing under either mechanism —
/// the medium anchor's reaction goes into the world, so a 131 kg load is not
/// something the motor has to argue with.
///
/// The *shortfall* did move at Stage 4, and it moved to zero. Before the medium
/// anchor the lift held 1.8365 m/s against an authored 2.0, one frame of gravity
/// short: the drive ran ahead of the solve in `integrate_forces` and then had
/// gravity folded into its target so it would not fight it, which left the
/// frame's fall standing. A motor row is solved *alongside* gravity rather than
/// before it, so there is nothing left over to absorb and the lift simply holds
/// its speed.
#[test]
fn cruise_under_load() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
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

    let climb = linear_velocity_of(&world, platform.body).y;
    let load = walker.mass(&world);
    eprintln!("cruise under load: climb={climb:.4} authored={speed:.4} passenger_mass={load:.1}kg");

    assert!(
        (climb - speed).abs() < 0.05,
        "loaded lift should hold its authored cruise speed: {climb:.4} vs {speed:.4}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Edge walk — which way does the platform yaw?
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification, inverted at Stage 5 — the one the design exists to
/// overturn.** A character walking `+X` along the `+Z` edge of a free-yawing
/// platform torques it `−Y`: the platform is what pushes the walker, so the
/// reaction recoils it the other way (requirements R1).
///
/// Stage 0 measured `+0.41128 rad/s` — the wrong sign, asserted on purpose.
/// The old drive was reactionless, so nothing but ordinary friction touched
/// the deck and the walker behaved as a conveyor belt, dragging it *forwards*.
/// The drive is now an impulse exchange at the contact points, so the reaction
/// lands where the feet are: `+X` at `+Z` gives `r × F` along `−Y`, and the
/// platform yaws `−0.12183 rad/s` away from the walker.
///
/// The magnitude is not the specification and should not be pinned. It is set
/// by how much tangential impulse the walk actually needs, which is a
/// transient — a body already at walk speed asks the row for nothing — and by
/// the platform's yaw inertia. The *sign* is the conservation claim, and it is
/// what R1 buys.
///
/// The platform hovers (`gravity_scale = 0`, no route servo) so that nothing
/// but the walker's own contact touches its yaw.
#[test]
fn edge_walk_yaws_the_platform_against_the_walker() {
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
        peak_yaw < 0.0,
        "walking +X at the +Z edge must recoil the platform −Y: the walker \
         pushes off the deck and the deck is pushed the other way. Measured \
         {peak_yaw:.5} rad/s"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Crate push — how fast does a pushed crate go?
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification as of Stage 5.** A driven character walks into a crate and
/// pushes it. The requirement wants push speed to follow the mass ratio, and
/// once the drive is bounded by the contact it does.
///
/// Stage 0 measured a *punt*: 4.70 m/s on a 768 kg box, 6.5× the mass-ratio
/// speed and near the walker's own 5.0. The drive was reactionless, restoring
/// the walker's velocity from an infinite account every substep, so mass did
/// not enter into it.
///
/// §11's ledger predicted this stayed a characterisation, on the grounds that
/// `drive_gain: 5.0` lets the walker transmit five times the tangential force.
/// It does — and it is still not enough to shove this crate. The walker's feet
/// can supply at most `0.8 · 5 · m_w · g ≈ 5.1 kN`, while the crate's own
/// friction against the ground resists `0.8 · m_c · g ≈ 6.0 kN`. So there is
/// no sustained push at all: what the crate gets is the inelastic transfer of
/// the walker's momentum at the moment of contact, which *is* the mass ratio.
/// The prediction was right about the mechanism and wrong about which side of
/// the crate's own friction 5× lands on.
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

    // The gain's ledger (§11): a walker leaning on a crate it cannot move
    // saturates its rows, and the impulse beyond the honest bound is exactly
    // what the instrumentation exists to make watchable.
    let usage = world
        .traction_usage(walker.body)
        .copied()
        .expect("a gained actuator reports what it borrowed");
    eprintln!(
        "crate push: borrowed {:.0} N over {} driving frames, saturated {:.0}%",
        usage.borrowed_force,
        usage.driving_frames,
        usage.saturation() * 100.0
    );
    assert!(
        usage.borrowed_force > 0.0,
        "a walker driving at 5x should be recorded borrowing impulse"
    );
    assert!(
        usage.saturation() > 0.5,
        "a walker leaning on an immovable crate runs its rows at the bound: \
         saturated {:.0}% of frames",
        usage.saturation() * 100.0
    );

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
    // The punt is gone. The peak is the inelastic transfer and nothing more:
    // never above it, because momentum is conserved, and close below it,
    // because the crate's own friction starts bleeding the speed away on the
    // same frame it receives it. Measured 0.7069 against 0.7281.
    assert!(
        peak_push <= mass_ratio_speed * 1.02,
        "the crate cannot receive more than the walker's momentum: peak \
         {peak_push:.4} vs mass-ratio {mass_ratio_speed:.4}"
    );
    assert!(
        peak_push > mass_ratio_speed * 0.85,
        "push speed should follow the mass ratio: peak {peak_push:.4} vs \
         mass-ratio {mass_ratio_speed:.4}"
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
/// they can go now that the drive has stopped re-asserting approach velocity
/// every substep. It is quiet to the digit either way, which is Stage 7's cue
/// to try deleting them.
///
/// The resting position moved 5 mm closer to the wall at Stage 5 — x = 0.2574
/// against 0.2621 — because the body is now held there by a tangential row at
/// its feet rather than by a velocity re-asserted ahead of the solve, and the
/// two balance against the wall's normal row at slightly different depths.
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

// ═══════════════════════════════════════════════════════════════════════════
// 9. Slope limits — where standing ends and where walking ends
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification, and §11's second Stage 5 check.** A character stands on and
/// walks up a 50° slope, and both limits are the same number.
///
/// §11's ledger predicted the two would diverge: grip at `μ = 0.8` gives a
/// standing limit of `atan(0.8) ≈ 39°` while a drive at `5μ` gives a walking
/// limit of `atan(4.0) ≈ 76°`, so a character would slide when idle and climb
/// when moving. As built they do not diverge, because the gain is a property
/// of the body driving through a contact rather than of the target it happens
/// to be asking for: a released stick commands *zero relative velocity across
/// the support*, which is a brake, and a brake at the honest bound is a fifth
/// of the authority the same body has when accelerating. Both limits are
/// therefore `atan(4.0) ≈ 76°`, and what actually decides whether a slope is
/// walkable is `SupportConfig::min_support_cosine` — the 60° cone that says
/// whether the contact holds the body up at all.
///
/// 50° is chosen to sit inside the 39°–76° band the ledger names and inside
/// the 60° support cone. Every level in `levels/` holds real area there.
#[test]
fn a_slope_a_character_can_walk_up_is_one_it_can_stand_on() {
    // run 10, rise 10·tan(50°): a 50° ramp climbing toward +Z.
    let slope_degrees: f32 = 50.0;
    let run = 10.0;
    let geometry = RampGeometry::new(8.0, run, run * slope_degrees.to_radians().tan());
    let mut debug = DebugLines::default();

    let start_z = 5.0;
    let surface_y = |z: f32| z * slope_degrees.to_radians().tan();

    // Standing: the stick is released, which asks for zero velocity across the
    // slope. If grip were the honest 0.8 the character would slide.
    let mut world = bench_world();
    let idle = Walker::spawn(
        &mut world,
        Vector3::new(0.0, surface_y(start_z) + 0.02, start_z),
    );
    for _ in 0..180 {
        idle.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }
    let stood_z = position_of(&world, idle.body).z;

    // Walking uphill from the same place.
    let mut world = bench_world();
    let climber = Walker::spawn(
        &mut world,
        Vector3::new(0.0, surface_y(start_z) + 0.02, start_z),
    )
    .walking(Vector3::z());
    for _ in 0..180 {
        climber.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }
    let climbed_z = position_of(&world, climber.body).z;

    eprintln!(
        "slope limits at {slope_degrees:.0}°: idle drifted {:.4} m, walker advanced {:.4} m",
        stood_z - start_z,
        climbed_z - start_z
    );

    assert!(
        (stood_z - start_z).abs() < 0.2,
        "an idle character should hold its place on a {slope_degrees:.0}° slope: \
         moved {:.4} m",
        stood_z - start_z
    );
    assert!(
        climbed_z > start_z + 1.0,
        "a character should climb a {slope_degrees:.0}° slope: advanced {:.4} m",
        climbed_z - start_z
    );
}
