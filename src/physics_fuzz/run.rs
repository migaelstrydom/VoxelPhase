//! Play one case and judge what the engine did with it.
//!
//! ```text
//!   build the structure ─▶ settle ─▶ disturb ─▶ step, frame by frame
//!                                       │          │  EnergyAudit, floor, finite
//!                                       │          ▼
//!                  knocks, ball, walker ┘       Verdict
//! ```
//!
//! The knocks and the ball's launch happen on the frame the audit begins, so
//! the energy they bring is where it starts. The walker is not audited: what
//! it gives the structure is work done through its contacts, which the audit
//! is told about and does not count as a gain.

use std::fmt;

use nalgebra::{Point3, Vector3};

use crate::debug::DebugLines;
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::{
    energy_per_kg, ColliderDesc, EnergyAudit, PhysicsConfig, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle, SequentialStepper, Stepper,
};

use super::case::{Case, Projectile};
use super::walker::Walker;

/// The engine's fixed step, and the most of them one frame may take: the
/// game's.
const FIXED_DT: f32 = 1.0 / 240.0;
const MAX_SUBSTEPS: u32 = 12;

/// How long the structure stands before anything disturbs it, in seconds.
const SETTLE_SECONDS: f32 = 2.0;

/// How long the case runs once disturbed, in seconds.
pub const RUN_SECONDS: f32 = 10.0;

/// Half the side of the floor every case stands on, in metres: wide enough
/// that a ball rolling for the whole run never reaches its edge.
const FLOOR_HALF_SIZE: f32 = 500.0;

/// How much energy per kilogram a group of the structure's bodies may regain,
/// beyond what the walker gave it, before it is reported, in J/kg.
///
/// Far above rounding and position correction, which lift a settling block by
/// millimetres. The failures this exists for are tens of J/kg — a tower's top
/// thrown metres into the air.
pub const ENERGY_GAIN_LIMIT: f32 = 0.5;

/// What the engine did that physics does not.
#[derive(Clone, Debug, PartialEq)]
pub enum Violation {
    /// A body's state became NaN or infinite.
    NonFinite,
    /// A group of the structure's bodies regained this much energy per
    /// kilogram that nothing gave it.
    GainedEnergy(f32),
    /// The body this far through the list of the structure's bodies (the
    /// ball last) went this far below the floor, its centre at this point.
    UnderFloor {
        body: usize,
        depth: f32,
        at: Point3<f32>,
    },
}

impl fmt::Display for Violation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Violation::NonFinite => write!(f, "a body became non-finite"),
            Violation::GainedEnergy(gain) => write!(
                f,
                "regained {gain:.2} J/kg from nowhere, as if it rose {:.0} cm",
                gain / 9.81 * 100.0
            ),
            Violation::UnderFloor { body, depth, at } => write!(
                f,
                "body {body} went {depth:.2} m under the floor, at ({:.1}, {:.1})",
                at.x, at.z
            ),
        }
    }
}

/// How a case is played, beyond what the case itself draws.
#[derive(Clone, Copy, Debug)]
pub struct RunOptions {
    /// Let bodies sleep, as the game does. Off, a finding that goes away was
    /// caused by waking.
    pub sleep: bool,
    /// Record every frame, for reading one case closely.
    pub trace: bool,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self {
            sleep: true,
            trace: false,
        }
    }
}

/// How one case went.
#[derive(Clone, Debug)]
pub struct Verdict {
    pub violations: Vec<Violation>,
    /// The most energy per kilogram any group regained unexplained, reported
    /// or not: where a working engine sits against the limit.
    pub worst_gain: f32,
    /// Every frame after the disturbance, if the run was traced.
    pub trace: Vec<TraceFrame>,
}

/// One frame of a traced run, over all the structure's bodies together.
#[derive(Clone, Debug)]
pub struct TraceFrame {
    /// Seconds since the disturbance.
    pub time: f32,
    /// Their energy per kilogram: kinetic, spin and height.
    pub energy: f32,
    /// Work bodies outside them did on them this frame, per kilogram of them.
    pub received: f32,
    /// The fastest of them, by its place in the list, its speed and height.
    pub fastest: (usize, f32, f32),
    /// How many of them are asleep.
    pub asleep: usize,
}

impl fmt::Display for TraceFrame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (body, speed, height) = self.fastest;
        write!(
            f,
            "{:6.3} s  {:9.4} J/kg  received {:+8.4}  fastest {body:3} at {speed:5.2} m/s, \
             y {height:5.2}  asleep {}",
            self.time, self.energy, self.received, self.asleep
        )
    }
}

/// Play `case` and judge it.
pub fn run(case: &Case, options: RunOptions) -> Verdict {
    let mut config = PhysicsConfig::default();
    config.deterministic_contact_ordering = case.ordered_contacts;
    config.sleep.enabled = options.sleep;
    let mut world = PhysicsWorld::new(config);
    let floor = FlatQuadGeometry::new(FLOOR_HALF_SIZE);
    let mut stepper = SequentialStepper::new(FIXED_DT, MAX_SUBSTEPS);
    let mut debug = DebugLines::default();
    let mut step = |world: &mut PhysicsWorld| {
        stepper.step(world, case.frame_dt, &floor, &[], &[], &mut debug);
    };

    // Bodies are made asleep, as the game's are; a structure built in the
    // air would hang there.
    let mut bodies = case.structure.build(&mut world);
    for &body in &bodies {
        world.wake_body(body);
    }
    for _ in 0..frames(SETTLE_SECONDS, case.frame_dt) {
        step(&mut world);
    }

    let (centre, reach, height) = extent(&world, &bodies);
    for knock in &case.knocks {
        let index = ((knock.body * bodies.len() as f32) as usize).min(bodies.len() - 1);
        let body = bodies[index];
        world.wake_body(body);
        if let Some(rigid) = world.body_mut(body) {
            let velocity = rigid.linear_velocity() + knock.velocity;
            rigid.set_linear_velocity(velocity);
        }
    }
    if let Some(projectile) = &case.projectile {
        bodies.push(throw(&mut world, projectile, centre, reach, height));
    }
    let mut walker = case
        .walker
        .clone()
        .map(|script| Walker::spawn(&mut world, script, centre, reach));

    let mut audit = EnergyAudit::begin(&mut world, bodies.iter().map(|&b| vec![b]).collect());
    let mut under_floor: Option<(usize, f32, Point3<f32>)> = None;
    let mut non_finite = false;
    let mut trace = Vec::new();
    let run_frames = frames(RUN_SECONDS, case.frame_dt);
    for frame in 0..run_frames {
        if let Some(walker) = &mut walker {
            walker.drive(&mut world, frame as f32 * case.frame_dt);
        }
        step(&mut world);
        audit.observe(&world);
        if options.trace {
            trace.push(trace_frame(
                &world,
                &bodies,
                (frame + 1) as f32 * case.frame_dt,
            ));
        }
        for (index, body) in bodies.iter().enumerate() {
            let Some(at) = world.body(*body).map(|b| b.position()) else {
                continue;
            };
            if at.y < 0.0 && under_floor.map_or(true, |(_, depth, _)| -at.y > depth) {
                under_floor = Some((index, -at.y, at));
            }
        }
        non_finite |= walker
            .as_ref()
            .and_then(|w| world.body(w.body()))
            .is_some_and(|body| !body.position().coords.iter().all(|c| c.is_finite()));
    }

    non_finite |= (0..bodies.len()).any(|object| audit.is_non_finite(object));
    let worst_gain = audit.gains().into_iter().flatten().fold(0.0, f32::max);

    let mut violations = Vec::new();
    if non_finite {
        violations.push(Violation::NonFinite);
    } else {
        if worst_gain > ENERGY_GAIN_LIMIT {
            violations.push(Violation::GainedEnergy(worst_gain));
        }
        if let Some((body, depth, at)) = under_floor {
            violations.push(Violation::UnderFloor { body, depth, at });
        }
    }
    Verdict {
        violations,
        worst_gain,
        trace,
    }
}

fn trace_frame(world: &PhysicsWorld, bodies: &[RigidBodyHandle], time: f32) -> TraceFrame {
    let mass: f32 = bodies
        .iter()
        .filter_map(|&b| world.body(b))
        .map(|body| body.mass())
        .sum();
    let received: f32 = world
        .contact_work()
        .iter()
        .filter(|(on, by, _)| bodies.contains(on) && by.is_some_and(|by| !bodies.contains(&by)))
        .map(|(_, _, work)| work)
        .sum();
    let fastest = bodies
        .iter()
        .enumerate()
        .filter_map(|(index, &b)| world.body(b).map(|body| (index, body)))
        .map(|(index, body)| (index, body.linear_velocity().norm(), body.position().y))
        .fold((0, 0.0, 0.0), |best, candidate| {
            if candidate.1 > best.1 {
                candidate
            } else {
                best
            }
        });
    TraceFrame {
        time,
        energy: energy_per_kg(world, bodies),
        received: received / mass.max(f32::MIN_POSITIVE),
        fastest,
        asleep: bodies.iter().filter(|&&b| world.is_sleeping(b)).count(),
    }
}

/// Frames in `seconds` at `frame_dt`.
fn frames(seconds: f32, frame_dt: f32) -> usize {
    (seconds / frame_dt).round() as usize
}

/// Where the bodies stand on the floor, how far they reach from there
/// across it, and how high they stand: the centre of their footprint, the
/// distance to the farthest body's centre plus half a metre for its size, and
/// the highest body's centre.
fn extent(world: &PhysicsWorld, bodies: &[RigidBodyHandle]) -> (Point3<f32>, f32, f32) {
    let positions: Vec<Point3<f32>> = bodies
        .iter()
        .filter_map(|&b| world.body(b))
        .map(|body| body.position())
        .collect();
    let count = positions.len().max(1) as f32;
    let sum = positions
        .iter()
        .fold(Vector3::zeros(), |sum, p| sum + p.coords);
    let centre = Point3::new(sum.x / count, 0.0, sum.z / count);
    let reach = positions
        .iter()
        .map(|p| Vector3::new(p.x - centre.x, 0.0, p.z - centre.z).norm())
        .fold(0.0, f32::max)
        + 0.5;
    let height = positions.iter().map(|p| p.y).fold(0.0, f32::max);
    (centre, reach, height)
}

/// Launch the case's ball at the structure, from just beyond its reach.
fn throw(
    world: &mut PhysicsWorld,
    projectile: &Projectile,
    centre: Point3<f32>,
    reach: f32,
    height: f32,
) -> RigidBodyHandle {
    let toward = Vector3::new(projectile.bearing.cos(), 0.0, projectile.bearing.sin());
    let target = Point3::new(centre.x, height * projectile.height, centre.z);
    let from = target - toward * (reach + 2.0);
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(from)
            .linear_velocity(toward * projectile.speed),
    );
    world.attach_collider(
        body,
        ColliderDesc::sphere(projectile.radius)
            .density(projectile.density)
            .restitution(0.3)
            .friction(0.5),
    );
    body
}
