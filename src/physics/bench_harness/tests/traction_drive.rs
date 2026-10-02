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
    Allowance, ColliderDesc, DriveCommand, FrictionModel, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle, StaticGeometry, VerticalVerbs,
};
use crate::platform::{DeckSuspension, MovingPlatform, REFERENCE_LOAD_KG};

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

/// The player's yaw allowance, in rad/s² (`spawners/player.rs`). A capsule's
/// supports are a point, so no torsional row can turn it (§6.2) and this is
/// the whole of its angular authority.
const YAW_AUTHORITY: f32 = 500.0;

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
    /// The airborne authority this character was granted, or `None` for one
    /// that was granted none.
    allowance: Option<Allowance>,
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

        let allowance = Some(Allowance::character(
            config.air_steer_speed,
            YAW_AUTHORITY,
            config.jump_speed,
        ));

        Self {
            body,
            config,
            intent: Vector3::zeros(),
            allowance,
        }
    }

    fn walking(mut self, direction: Vector3<f32>) -> Self {
        self.intent = direction.normalize();
        self
    }

    /// The same character with no non-conservative authority at all — a body
    /// whose actuator was never granted an allowance.
    fn ungranted(mut self) -> Self {
        self.allowance = None;
        self
    }

    /// The same character with the stick released.
    fn standing(&self) -> Self {
        Self {
            body: self.body,
            config: self.config.clone(),
            intent: Vector3::zeros(),
            allowance: self.allowance,
        }
    }

    fn mass(&self, world: &PhysicsWorld) -> f32 {
        world.body(self.body).unwrap().mass()
    }

    /// `CharacterControlSystem` then `PhysicsSyncSystem`, once per frame.
    fn drive(&self, world: &mut PhysicsWorld) {
        self.drive_with(world, VerticalVerbs::default());
    }

    /// The same, on a frame where a discrete verb fired.
    fn drive_with(&self, world: &mut PhysicsWorld, verbs: VerticalVerbs) {
        let target = self.intent * self.config.walk_speed;
        let mut command = DriveCommand::support(target, Vector3::zeros(), 500.0, 500.0)
            .with_drive_gain(WALKER_DRIVE_GAIN);
        command.allowance.budget = self.allowance;
        command.allowance.verbs = verbs;
        command.allowance.steer_accel = Some(self.config.air_steer_speed);
        let _ = world.set_body_drive(self.body, &command);
    }

    /// The jump verb, as `CharacterControlSystem` issues it.
    fn jump(&self) -> VerticalVerbs {
        VerticalVerbs {
            jump_speed: Some(self.config.jump_speed),
            ..Default::default()
        }
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
            route: MovingPlatform::shuttle(from, to, speed),
        }
    }

    fn spawn_body(
        world: &mut PhysicsWorld,
        at: Vector3<f32>,
        gravity_scale: f32,
    ) -> RigidBodyHandle {
        Self::spawn_body_with_suspension(world, at, gravity_scale, DeckSuspension::RIGID)
    }

    fn spawn_body_with_suspension(
        world: &mut PhysicsWorld,
        at: Vector3<f32>,
        gravity_scale: f32,
        suspension: DeckSuspension,
    ) -> RigidBodyHandle {
        let tuning = suspension.tune(&PLATFORM_HALF_EXTENTS);
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::from(at))
                .gravity_scale(gravity_scale)
                .linear_damping(0.0)
                .angular_damping(tuning.angular_damping),
        );
        let _ = world.attach_collider(
            body,
            ColliderDesc::box_shape(PLATFORM_HALF_EXTENTS)
                .density(300.0)
                .friction(0.9),
        );
        // Mirrors what `MovingPlatformDef::spawn` does, and for the same
        // reason: the tensor to scale does not exist until the collider is on.
        if let Some(body) = world.body_mut(body) {
            body.scale_local_inertia(Vector3::new(1.0, tuning.yaw_inertia_scale, 1.0));
        }
        // Tilt is held, as softly as the suspension asks; yaw is deliberately
        // free, which is what makes the edge-walk sign observable at all.
        let _ = world.create_constraint(ConstraintKind::KeepUpright {
            body,
            target_up: UnitVector3::new_normalize(Vector3::y()),
            compliance: tuning.compliance,
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
                self.route.target_velocity(&position, FRAME_DT),
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
    let mut heading = platform.route.target_index();
    let mut frames_to_reversal = 0;
    while platform.route.target_index() == heading && frames_to_reversal < 600 {
        platform.drive(&mut world);
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        frames_to_reversal += 1;
    }
    heading = platform.route.target_index();
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

/// Yaw resistance slows the slew without welding it shut.
///
/// The dial exists because `KeepUpright` leaves yaw free and nothing else
/// opposes it, so a light deck is turned by whoever walks across it. What it
/// must not do is become a lock: the recoil above is real physics, and a
/// platform that answered a shove with nothing at all would read as scenery.
/// So the assertion is on both halves — much smaller, and still there, and
/// still the right way round.
#[test]
fn yaw_resistance_slows_the_edge_walk_slew_without_stopping_it() {
    let peak_yaw = |yaw_resistance: f32| {
        let geometry = FlatQuadGeometry::new(64.0);
        let mut world = bench_world();
        let mut debug = DebugLines::default();

        let platform_centre = Vector3::new(0.0, 5.0, 0.0);
        let platform = Platform::spawn_body_with_suspension(
            &mut world,
            platform_centre,
            0.0,
            DeckSuspension {
                yaw_resistance,
                ..DeckSuspension::RIGID
            },
        );
        let deck = platform_centre.y + PLATFORM_HALF_EXTENTS.y;

        let walker = Walker::spawn(
            &mut world,
            Vector3::new(-1.5, deck + 0.02, PLATFORM_HALF_EXTENTS.z - 0.4),
        )
        .walking(Vector3::x());

        let idle = walker.standing();
        for _ in 0..30 {
            idle.drive(&mut world);
            advance(&mut world, &geometry, &mut debug);
        }

        let mut peak: f32 = 0.0;
        for _ in 0..120 {
            walker.drive(&mut world);
            advance(&mut world, &geometry, &mut debug);
            let yaw = world.body(platform).unwrap().angular_velocity().y;
            if yaw.abs() > peak.abs() {
                peak = yaw;
            }
        }
        peak
    };

    let free = peak_yaw(1.0);
    let stiff = peak_yaw(20.0);
    eprintln!("edge walk yaw: free={free:.5} rad/s, 20x resistance={stiff:.5} rad/s");

    assert!(
        stiff < 0.0,
        "the recoil must survive the dial, sign and all: {stiff:.5} rad/s"
    );
    assert!(
        stiff.abs() < free.abs() * 0.5,
        "20x the yaw inertia should more than halve the slew: {stiff:.5} against {free:.5}"
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

// ═══════════════════════════════════════════════════════════════════════════
// 10-14. The airborne half: jumping, jump shaping and air steering
// ═══════════════════════════════════════════════════════════════════════════
//
// The bench had no jump verb until Stage 6, which is why several of the
// design's claims about jumping went untested for five stages. These five
// scenarios are that gap closed. All of them are specifications.

/// Run one jump and report what it did: the peak height above the takeoff
/// point, and the velocity on the frame the verb fired.
struct JumpArc {
    peak_rise: f32,
    takeoff_velocity: Vector3<f32>,
}

/// Settle a walker on `geometry`, jump, and follow the arc to its apex.
fn take_off(
    world: &mut PhysicsWorld,
    geometry: &dyn StaticGeometry,
    walker: &Walker,
    verbs: VerticalVerbs,
) -> JumpArc {
    let mut debug = DebugLines::default();
    for _ in 0..60 {
        walker.drive(world);
        advance(world, geometry, &mut debug);
    }

    let start = position_of(world, walker.body);
    walker.drive_with(world, verbs);
    advance(world, geometry, &mut debug);
    let takeoff_velocity = linear_velocity_of(world, walker.body);

    let mut peak_rise = position_of(world, walker.body).y - start.y;
    for _ in 0..120 {
        walker.drive(world);
        advance(world, geometry, &mut debug);
        peak_rise = peak_rise.max(position_of(world, walker.body).y - start.y);
    }

    JumpArc {
        peak_rise,
        takeoff_velocity,
    }
}

/// **Specification.** A jump from flat ground leaves at the speed it asked for
/// and reaches the height that speed implies.
///
/// The verb is a *speed along the support normal*, not an impulse added to
/// whatever the body was doing, so the same press is the same jump whether the
/// character was walking, standing or settling onto the floor.
#[test]
fn a_jump_from_flat_ground_reaches_the_height_its_speed_implies() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let walker = Walker::spawn(&mut world, Vector3::new(0.0, 0.02, 0.0));
    let jump_speed = walker.config.jump_speed;

    let arc = take_off(&mut world, &geometry, &walker, walker.jump());
    let gravity = -world.config().gravity.y;
    let ballistic_rise = jump_speed * jump_speed / (2.0 * gravity);

    eprintln!(
        "flat jump: takeoff vy={:.4} (asked {jump_speed:.1}) peak rise={:.4} m \
         (ballistic {ballistic_rise:.4})",
        arc.takeoff_velocity.y, arc.peak_rise
    );

    assert!(
        (arc.takeoff_velocity.y - jump_speed).abs() < 0.5,
        "a jump should leave at the speed it asked for: {:.4} vs {jump_speed:.1}",
        arc.takeoff_velocity.y
    );
    assert!(
        (arc.peak_rise - ballistic_rise).abs() < 0.2,
        "and rise as ballistics say it must: {:.4} vs {ballistic_rise:.4}",
        arc.peak_rise
    );
}

/// **Specification.** Jumping off a deck pushes the deck down (R1).
///
/// The whole content of §6.3: a jump is delivered through the Support Set as
/// an impulse exchange at the contact points, so the reaction lands on whatever
/// was holding the jumper up, at the mass ratio and nowhere else. The deck is
/// unpowered and weightless-in-gravity on purpose — a motorised lift absorbs
/// the whole kick inside the frame it arrives, which says something about the
/// motor rather than about the jump.
#[test]
fn a_jump_off_a_deck_pushes_the_deck_down() {
    let kick = |jumping: bool| {
        let geometry = FlatQuadGeometry::new(64.0);
        let mut world = bench_world();
        let mut debug = DebugLines::default();
        let deck = Platform::spawn_body(&mut world, Vector3::new(0.0, 6.0, 0.0), 0.0);
        let deck_y = position_of(&world, deck).y + PLATFORM_HALF_EXTENTS.y;
        let walker = Walker::spawn(&mut world, Vector3::new(0.0, deck_y + 0.02, 0.0));

        for _ in 0..60 {
            walker.drive(&mut world);
            advance(&mut world, &geometry, &mut debug);
        }

        let before = linear_velocity_of(&world, deck).y;
        if jumping {
            walker.drive_with(&mut world, walker.jump());
        } else {
            walker.drive(&mut world);
        }
        advance(&mut world, &geometry, &mut debug);
        let after = linear_velocity_of(&world, deck).y;
        let ratio = world.body(walker.body).unwrap().mass() / world.body(deck).unwrap().mass();
        (before - after, ratio)
    };

    let (control, _) = kick(false);
    let (jumped, mass_ratio) = kick(true);
    let predicted = 7.0 * mass_ratio;
    eprintln!(
        "jump off a deck: deck slowed {jumped:.4} m/s, control {control:.4}, \
         mass ratio predicts {predicted:.4}"
    );

    assert!(
        (jumped - control - predicted).abs() < 0.1 * predicted,
        "the deck should take the jump's reaction at the mass ratio: \
         {:.4} m/s against {predicted:.4}",
        jumped - control
    );
}

/// **Specification.** A jump leaves along the world's up, so a jump from a
/// slope goes straight up and as high as one from the flat (§10.2).
///
/// Leaving along the support normal cost `cos²θ` of the height and threw the
/// player downhill, which made a bowl's walls push a jump back into the bowl.
#[test]
fn a_jump_from_a_slope_leaves_straight_up() {
    let slope_degrees: f32 = 30.0;
    let run = 10.0;
    let tangent = slope_degrees.to_radians().tan();
    let geometry = RampGeometry::new(8.0, run, run * tangent);
    let start_z = 5.0;

    let mut world = bench_world();
    let walker = Walker::spawn(
        &mut world,
        Vector3::new(0.0, start_z * tangent + 0.02, start_z),
    );
    let jump_speed = walker.config.jump_speed;
    let arc = take_off(&mut world, &geometry, &walker, walker.jump());

    let gravity = -world.config().gravity.y;
    let flat_rise = jump_speed * jump_speed / (2.0 * gravity);

    eprintln!(
        "slope jump at {slope_degrees:.0}°: takeoff v=({:.4}, {:.4}, {:.4}), \
         peak rise {:.4} m (flat {flat_rise:.4})",
        arc.takeoff_velocity.x, arc.takeoff_velocity.y, arc.takeoff_velocity.z, arc.peak_rise
    );

    let lateral = Vector3::new(arc.takeoff_velocity.x, 0.0, arc.takeoff_velocity.z).magnitude();
    assert!(
        lateral < 0.2,
        "a slope jump should gain no direction the player did not ask for: {lateral:.4} m/s across"
    );
    assert!(
        (arc.takeoff_velocity.y - jump_speed).abs() < 0.2,
        "and leave at the speed it asked for: {:.4} m/s against {jump_speed:.4}",
        arc.takeoff_velocity.y
    );
    assert!(
        (arc.peak_rise - flat_rise).abs() < 0.15,
        "and rise as high as from the flat: {:.4} m against {flat_rise:.4}",
        arc.peak_rise
    );
}

/// **Specification.** A jump the Support Set cannot deliver is conjured out of
/// the allowance instead of being swallowed — decision D2a — and a body with
/// no allowance gets nothing.
///
/// This is the coyote jump, the stale-grounding jump and every other
/// unsupported one, under the single rule D2a states them as. The ledger is
/// part of the assertion: R8's requirement is that the cheat be counted, and a
/// silently conservative-or-not jump is exactly what D2a asks to be watched.
#[test]
fn a_jump_with_no_support_is_conjured_out_of_the_allowance() {
    let airborne = |granted: bool| {
        let geometry = FlatQuadGeometry::new(64.0);
        let mut world = bench_world();
        let mut debug = DebugLines::default();
        let mut walker = Walker::spawn(&mut world, Vector3::new(0.0, 20.0, 0.0));
        if !granted {
            walker = walker.ungranted();
        }

        // Fall clear of the floor for a quarter of a second, then ask.
        for _ in 0..15 {
            walker.drive(&mut world);
            advance(&mut world, &geometry, &mut debug);
        }
        let before = linear_velocity_of(&world, walker.body).y;
        walker.drive_with(&mut world, walker.jump());
        advance(&mut world, &geometry, &mut debug);
        let after = linear_velocity_of(&world, walker.body).y;
        let usage = world
            .allowance_usage(walker.body)
            .map(|usage| (usage.unsupported_jumps, usage.supported_jumps))
            .unwrap_or((0, 0));
        (before, after, usage)
    };

    let (fell, granted, granted_jumps) = airborne(true);
    let (_, ungranted, ungranted_jumps) = airborne(false);
    eprintln!(
        "unsupported jump: falling at {fell:.4} m/s, granted leaves at {granted:.4}, \
         ungranted at {ungranted:.4}; ledger {granted_jumps:?} vs {ungranted_jumps:?}"
    );

    assert!(
        granted > 6.5,
        "an allowance should deliver the jump the contacts could not: {granted:.4} m/s"
    );
    assert_eq!(
        granted_jumps,
        (1, 0),
        "and it should be counted as conjured, not pushed off something"
    );
    assert!(
        ungranted < fell,
        "a body with no allowance keeps falling: {ungranted:.4} against {fell:.4} m/s"
    );
    assert_eq!(ungranted_jumps, (0, 0), "and spends nothing");
}

/// **Specification.** Air steering is authority the allowance grants and
/// bounds: an airborne character accelerates across the fall at its declared
/// rate and stops at the speed it asked for.
#[test]
fn air_steering_ramps_at_its_budget_and_no_further() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();
    let walker = Walker::spawn(&mut world, Vector3::new(0.0, 40.0, 0.0))
        .walking(Vector3::new(1.0, 0.0, 0.0));
    let rate = walker.config.air_steer_speed;
    let walk_speed = walker.config.walk_speed;

    let mut after_half_a_second = 0.0;
    for frame in 0..120 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        if frame == 29 {
            after_half_a_second = linear_velocity_of(&world, walker.body).x;
        }
    }
    let settled = linear_velocity_of(&world, walker.body).x;

    eprintln!(
        "air steering: {after_half_a_second:.4} m/s after 0.5 s at {rate:.1} m/s², \
         settled at {settled:.4} against a {walk_speed:.1} m/s target"
    );

    assert!(
        (after_half_a_second - rate * 0.5).abs() < 0.3,
        "air steering should ramp at its declared rate: {after_half_a_second:.4} m/s \
         against {:.4}",
        rate * 0.5
    );
    assert!(
        (settled - walk_speed).abs() < 0.05,
        "and stop at the speed it was asked for: {settled:.4} against {walk_speed:.1}"
    );
}

/// **Specification.** A body that declares a contact patch turns on it, and one
/// that does not cannot (R10, §6.2).
///
/// The torsional row is the angular projection of the same mechanism as the
/// tangential one, and this is the whole of what it is good for: a body whose
/// supports really are wider than the single point standing for them. Note
/// what it is *not* needed for — a manifold with several contacts already
/// yaws under its own tangential rows, at their own lever arms, which is why
/// this scenario uses a sphere. A capsule declares nothing, has one contact,
/// and turns with its allowance instead; that is every character in the game.
#[test]
fn a_declared_contact_patch_is_what_a_torsional_row_turns_on() {
    let turned = |patch_radius: f32| {
        let geometry = FlatQuadGeometry::new(64.0);
        let mut world = bench_world();
        let mut debug = DebugLines::default();

        // A sphere, so the contact really is a point: `ω × r` at a contact
        // directly below the centre has no component about the vertical, so
        // the tangential rows supply no yaw at all and whatever turns this
        // body came from the torsional one.
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 0.501, 0.0))
                .gravity_scale(1.0)
                .linear_damping(0.0)
                .angular_damping(0.0),
        );
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(0.5)
                .density(200.0)
                .restitution(0.0)
                .friction(0.8),
        );

        for _ in 0..120 {
            let mut command =
                DriveCommand::support(Vector3::zeros(), Vector3::new(0.0, 2.0, 0.0), 500.0, 500.0);
            command.patch_radius = patch_radius;
            let _ = world.set_body_drive(body, &command);
            advance(&mut world, &geometry, &mut debug);
        }
        world.body(body).unwrap().angular_velocity().y
    };

    let (declared, undeclared) = (turned(0.3), turned(0.0));
    eprintln!(
        "torsional row: a 0.3 m patch reached {declared:.4} rad/s, a point contact \
         {undeclared:.4} rad/s (both commanded 2.0)"
    );

    assert!(
        declared > 0.5,
        "a declared patch should be turned by its torsional rows: {declared:.4} rad/s"
    );
    assert!(
        undeclared.abs() < 0.05,
        "and a point contact should have nothing to turn on: {undeclared:.4} rad/s"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// 16. What a slope costs — the deferred Effort question, measured
// ═══════════════════════════════════════════════════════════════════════════

/// **Specification, and the answer to a deferred question.**
///
/// Traction and grip share one budget (R7), so §10.3 anticipated that "walking
/// uphill will cost speed" and the requirements left a seam for an **Effort**
/// authority to bias the drive and pay that cost back. That was a prediction.
/// This measures it, and the prediction is wrong in its cause.
///
/// A slope *does* cost pace, by `cos θ` — but **uphill and downhill cost the
/// same**, to three decimal places, which is the whole finding. Gravity is not
/// what is taking it. What is taking it is that the gait states its target as
/// a horizontal vector while a tangential row can only deliver the part of it
/// that lies in the contact's own plane, and the in-plane part of a horizontal
/// `v` on a `θ` slope is `v·cos θ`. The shared budget never binds: a contact
/// carries `N = m·g·cos θ`, so the drive has `μ·gain·m·g·cos θ` against a pull
/// of `m·g·sin θ`, and that stays positive to `tan θ = 4`, i.e. 76° — outside
/// the 60° cone that decides whether a surface is a support at all.
///
/// So an Effort authority is not needed, and would be the wrong shape: there
/// is no lost work to pay back, only a command stated in the wrong frame. If
/// the game wants full pace on a slope the correction is conservative and
/// free — state the gait's target in the support's tangent plane rather than
/// in world XZ, which is the `MovementRule::target` convention §8's R6 row
/// already flags. That is a feel decision and a play-test, not a mechanism.
#[test]
fn a_slope_costs_a_walker_the_same_uphill_as_down() {
    let sample = |slope_degrees: f32, heading: Vector3<f32>| {
        let run = 30.0;
        let tangent = slope_degrees.to_radians().tan();
        let geometry = RampGeometry::new(20.0, run, run * tangent);
        let mut world = bench_world();
        let mut debug = DebugLines::default();

        // Start where the walker has ramp ahead of it: three seconds at walk
        // speed is fifteen metres, and a sample taken off the end of the ramp
        // would be a sample of flat ground.
        let start_z = if heading.z < 0.0 { run - 2.0 } else { 2.0 };
        let walker = Walker::spawn(
            &mut world,
            Vector3::new(0.0, start_z * tangent + 0.02, start_z),
        )
        .walking(heading);

        // Two seconds to reach a steady state, then measure over one more.
        for _ in 0..120 {
            walker.drive(&mut world);
            advance(&mut world, &geometry, &mut debug);
        }
        let from = position_of(&world, walker.body);
        for _ in 0..60 {
            walker.drive(&mut world);
            advance(&mut world, &geometry, &mut debug);
        }
        (position_of(&world, walker.body) - from).magnitude()
    };

    let walk_speed = LocomotionConfig::player().walk_speed;
    eprintln!("slope cost (fraction of {walk_speed:.1} m/s held, measured over 1 s):");
    for slope_degrees in [0.0_f32, 10.0, 20.0, 30.0, 40.0, 50.0] {
        let uphill = sample(slope_degrees, Vector3::z()) / walk_speed;
        let across = sample(slope_degrees, Vector3::x()) / walk_speed;
        let downhill = sample(slope_degrees, -Vector3::z()) / walk_speed;
        let projection = slope_degrees.to_radians().cos();
        eprintln!(
            "  {slope_degrees:>4.0}°  uphill {uphill:.3}  across {across:.3}  \
             downhill {downhill:.3}  (cos θ = {projection:.3})"
        );

        assert!(
            (uphill - downhill).abs() < 0.02,
            "gravity should cost a slope nothing the drive cannot afford: \
             {slope_degrees:.0}° uphill {uphill:.3} against downhill {downhill:.3}"
        );
        assert!(
            (uphill - projection).abs() < 0.02,
            "and what it does cost should be the projection of a horizontal \
             command: {slope_degrees:.0}° held {uphill:.3} against cos θ {projection:.3}"
        );
        assert!(
            across > 0.98,
            "a heading already in the tangent plane should cost nothing: \
             {slope_degrees:.0}° held {across:.3}"
        );
    }
}

/// A passenger rides out the wobble their own landing caused.
///
/// The deck suspension gives the platform a real tilt, and a tilt is a slope:
/// the thing it must not do is decant the player over the side, or throw them
/// off it. The walker stands still and the deck is kicked as hard as a landing
/// kicks it, which is the worst case — a walker with the stick forward has
/// traction to argue with, one standing still has only friction.
#[test]
fn a_passenger_rides_out_the_deck_wobble() {
    let geometry = FlatQuadGeometry::new(64.0);
    let mut world = bench_world();
    let mut debug = DebugLines::default();

    let platform_centre = Vector3::new(0.0, 5.0, 0.0);
    let platform = Platform::spawn_body_with_suspension(
        &mut world,
        platform_centre,
        0.0,
        DeckSuspension {
            tilt_degrees: 2.0,
            damping: 0.5,
            yaw_resistance: DeckSuspension::default_yaw_resistance(),
        },
    );
    let deck = platform_centre.y + PLATFORM_HALF_EXTENTS.y;

    // Standing out towards the +X edge, where the swing is largest.
    let start = Vector3::new(1.5, deck + 0.02, 0.0);
    let walker = Walker::spawn(&mut world, start).standing();
    for _ in 0..60 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
    }

    // Everything is measured in the deck's own frame, rotation included. Two
    // things would otherwise be read as the passenger failing: the platform is
    // unpowered here and sinks under their weight for the whole run, and a
    // deck tilted by 6° drops its +X edge 0.17 m below its centre. A passenger
    // riding either of those perfectly is doing exactly what they should.
    let on_deck = |world: &PhysicsWorld| {
        let body = world.body(platform).unwrap();
        body.rotation().inverse() * (position_of(world, walker.body) - body.position().coords)
    };
    let settled = on_deck(&world);

    // The kick their landing would have delivered.
    world
        .body_mut(platform)
        .unwrap()
        .apply_angular_impulse(Vector3::new(
            0.0,
            0.0,
            -REFERENCE_LOAD_KG * 7.0 * PLATFORM_HALF_EXTENTS.x,
        ));

    let mut max_drift = 0.0f32;
    let mut max_lift = 0.0f32;
    let mut max_sink = 0.0f32;
    for _ in 0..480 {
        walker.drive(&mut world);
        advance(&mut world, &geometry, &mut debug);
        let p = on_deck(&world);
        max_drift = max_drift.max((p - settled).xz().magnitude());
        max_lift = max_lift.max(p.y - settled.y);
        max_sink = max_sink.max(settled.y - p.y);
    }

    let end = on_deck(&world);
    eprintln!(
        "wobble ride: drift {max_drift:.3} m, lift {max_lift:.3} m, sink \
         {max_sink:.3} m, ended {:.3} m above the deck centre",
        end.y
    );

    assert!(
        max_sink < 0.1,
        "the passenger should stay on top of the deck, not sink {max_sink:.3} m into it"
    );
    assert!(
        (end.y - settled.y).abs() < 0.05,
        "the passenger should end where they started on the deck, {:.3} m off",
        end.y - settled.y
    );
    assert!(
        max_drift < 0.5,
        "a wobble should not decant the passenger over the side: drifted {max_drift:.3} m"
    );
    assert!(
        max_lift < 0.3,
        "a wobble should not launch the passenger: lifted {max_lift:.3} m"
    );
}
