//! The drive contract: what the traversal planner's `GaitParams` claim the
//! traction drive does, measured in the real game.
//!
//! ```text
//!   slope    ramps of 30..58° climbed at walk and run ─▶ climbed up to max_ground_slope_degrees,
//!                                                        at BodyDims::ground_speed
//!   accel    a run-up from standing on flat ground    ─▶ max_acceleration, a little under it
//!   takeoff  a jump on flat ground at walk and run    ─▶ the arc it flies: gravity,
//!            and one in the coyote window off a deck     takeoff climb ≥ max_takeoff_climb,
//!                                                        no bend, no lost climb
//!   ride     the garden's lips and plank bridges      ─▶ the capsule's height against
//!                                                        the floor a cast finds under it
//!   face     short faces of 20..38° met at walk and   ─▶ the speed kept along the face,
//!            run, up to a shelf                          and the rise off its brow cut
//!                                                        after BodyDims::rise_held; the
//!                                                        time lost, charged at least
//!   hop      a planned hop on flat ground at walk     ─▶ the time it loses, charged at
//!            and run, climbing 1.5 and 3 m/s             least by costs.flight
//! ```
//!
//! Each check names the number it measured and the claim it holds it to, so
//! when the drive is tuned the failing row says what to update.

use std::f32::consts::FRAC_PI_2;

use traversal_planner::params::STANDARD_GRAVITY;
use traversal_planner::planner::recovery_time_lost;
use traversal_planner::{GaitParams, PlannerSettings};

use crate::anim_viewer::{Beat, Script};
use crate::animation::CharacterRigConfig;
use crate::character::LocomotionConfig;
use crate::core::error::{EngineError, EngineResult};
use crate::traversal::player_gait;

use super::driver::{run, RunConfig, RIDE_SLOPE_RUN};
use super::record::FrameRecord;
use super::scenario::{CameraRig, Judged, Scenario};
use super::scenarios::find;

const LEVEL: &str = "levels/drive_contract.level.ron";
/// Where every lane's run starts, in x.
const START_X: f32 = -5.0;
/// Slope lanes: angle in degrees, and the lane's z.
const SLOPES: [(f32, f32); 8] = [
    (30.0, 0.0),
    (34.0, 8.0),
    (38.0, 16.0),
    (42.0, 24.0),
    (46.0, 32.0),
    (50.0, -8.0),
    (54.0, -16.0),
    (58.0, -24.0),
];
/// The x span of a slope's middle, where the body's speed up it is judged:
/// clear of the foot and the brow.
const SLOPE_MIDDLE: std::ops::Range<f32> = 8.0..12.0;
/// Past this x the body is on a slope's shelf: it got up.
const SHELF_X: f32 = 14.0;
/// Largest share by which the speed across the ground up a slope may differ
/// from the plan's.
const SLOPE_SPEED_TOLERANCE: f32 = 0.1;
/// How long a slope run lasts, in seconds.
const SLOPE_RUN: f32 = 7.0;
const FLAT_Z: f32 = 40.0;
const DECK_Z: f32 = 48.0;
/// Where the deck ends, in x.
const DECK_EDGE: f32 = 7.0;
/// How long the run-up before a flat jump lasts, in seconds: up to speed.
const RUN_UP: f32 = 2.5;
/// How long a jump beat lasts, in seconds: well past landing.
const JUMP_BEAT: f32 = 2.5;
/// Garden scenarios whose rides are compared with the cast floor.
const RIDES: [&str; 8] = [
    "lip25_walk",
    "lip25_run",
    "lip40_walk",
    "lip40_run",
    "planks_walk",
    "planks_run",
    "planks_wide_walk",
    "planks_wide_run",
];
/// Furthest an arc may stray from the ballistic one fitted to it, in metres.
/// How far a frame's change of vertical speed may be from gravity's, as a
/// share of it, for the body to count as falling freely.
const FREE_FALL_SLACK: f32 = 0.5;
const ARC_TOLERANCE: f32 = 0.03;
/// Fastest a body counts as standing, in m/s.
const AT_REST: f32 = 0.05;
/// Share of its gait's speed a run-up is timed to.
const ACCELERATED: f32 = 0.9;
/// Largest share by which the drive's acceleration may exceed the claimed:
/// a plan that brakes and speeds up more gently than the drive does holds
/// the body back.
const ACCELERATION_SLACK: f32 = 0.25;
/// Largest share by which the effective gravity may differ from the claimed.
const GRAVITY_TOLERANCE: f32 = 0.02;
/// Largest share of its takeoff speed a flight may gain or lose across.
const AIR_SPEED_TOLERANCE: f32 = 0.05;
/// Largest share of a flat jump's climb a coyote jump may lose.
const COYOTE_TOLERANCE: f32 = 0.05;
/// Face lanes: angle in degrees, and the lane's z. Each face rises 0.3 m
/// from x = [`FACE_X`] to a shelf.
const FACES: [(f32, f32); 3] = [(20.0, 56.0), (30.0, 64.0), (38.0, 72.0)];
const FACE_X: f32 = 7.0;
/// Largest share by which the speed across the ground kept meeting a face
/// may differ from the claimed.
const FACE_SPEED_TOLERANCE: f32 = 0.05;
/// How long a face run lasts, in seconds: well past the brow.
const FACE_RUN: f32 = 4.0;
/// How far past a face its crossing is timed to, in metres: past the brow
/// and the speed regained after it.
const FACE_TIMED: f32 = 5.0;
/// Furthest the grounded capsule may be from the cast floor, in metres.
const RIDE_TOLERANCE: f32 = 0.03;
/// How long into the run a hop's cost is measured from, in seconds: up to
/// speed by then.
const HOP_AT: f32 = 1.5;
/// Climbs of the hops measured, in m/s: a typical planned hop's, and twice
/// it.
const HOP_CLIMBS: [f32; 2] = [1.5, 3.0];
/// How far past the takeoff the crossing is timed to, in metres: past the
/// landing and the speed regained after it.
const HOP_RUN: f32 = 5.0;
/// How long the run goes on after the takeoff, in seconds.
const HOP_TAIL: f32 = 2.0;

/// One measurement, against what the planner claims.
#[derive(Debug)]
pub struct ContractCheck {
    pub name: String,
    pub measured: String,
    pub claim: String,
    pub holds: bool,
}

impl ContractCheck {
    pub fn header() -> String {
        format!(
            "{:<22} {:<5} {:<44} {}",
            "check", "holds", "measured", "claim"
        )
    }

    pub fn row(&self) -> String {
        format!(
            "{:<22} {:<5} {:<44} {}",
            self.name,
            if self.holds { "yes" } else { "NO" },
            self.measured,
            self.claim
        )
    }
}

/// Walking or running, as the pilot asks.
#[derive(Clone, Copy)]
enum Gait {
    Walk,
    Run,
}

impl Gait {
    const BOTH: [Gait; 2] = [Gait::Walk, Gait::Run];

    fn beat(self, duration: f32) -> Beat {
        match self {
            Gait::Walk => Beat::walk(duration),
            Gait::Run => Beat::run(duration),
        }
    }

    fn jump(self, duration: f32) -> Beat {
        Beat {
            jump: true,
            ..self.beat(duration)
        }
        .named("jump")
    }

    fn speed(self) -> f32 {
        let config = LocomotionConfig::default();
        match self {
            Gait::Walk => config.walk_speed,
            Gait::Run => config.walk_speed * config.sprint_speed_mul,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Gait::Walk => "walk",
            Gait::Run => "run",
        }
    }
}

/// Runs every check against the player's `GaitParams`.
pub fn check_contract(config: &RunConfig) -> EngineResult<Vec<ContractCheck>> {
    let params = player_gait(&LocomotionConfig::default(), &CharacterRigConfig::default());
    let mut checks = slope(config, &params)?;
    for gait in Gait::BOTH {
        checks.extend(takeoff(config, &params, gait)?);
    }
    checks.push(coyote(config)?);

    for name in RIDES {
        checks.push(ride(config, &params, name)?);
    }
    for gait in Gait::BOTH {
        let flat = flat_crossing(config, gait, FACE_X + FACE_TIMED)?;
        for (angle, lane_z) in FACES {
            checks.extend(face(config, &params, angle, lane_z, gait, flat)?);
        }
    }
    for gait in Gait::BOTH {
        for climb in HOP_CLIMBS {
            checks.push(hop(config, gait, climb)?);
        }
    }
    Ok(checks)
}

fn lane(name: &'static str, lane_z: f32, script: Script) -> Scenario {
    Scenario {
        name,
        summary: "drive contract",
        level: LEVEL,
        start: (START_X, lane_z),
        yaw: FRAC_PI_2,
        script,
        camera: CameraRig::side().with_distance(5.0),
        judged: Judged::Motion,
    }
}

/// Every slope lane at walk and run: whether the body gets up it, and the
/// speed across the ground it keeps up its middle. A plan rides a slope no
/// steeper than the claim at the speed `BodyDims::ground_speed` gives it; a
/// steeper one it never rides.
fn slope(config: &RunConfig, params: &GaitParams) -> EngineResult<Vec<ContractCheck>> {
    let claim = params.body.max_ground_slope_degrees;
    let mut checks = Vec::new();
    for (angle, lane_z) in SLOPES {
        let grade = angle.to_radians().tan();
        let mut held = Vec::new();
        for gait in Gait::BOTH {
            let script = Script::new()
                .then(Beat::stand(0.3))
                .then(gait.beat(SLOPE_RUN));
            let take = run(&lane("slope", lane_z, script), config)?;
            let across: Vec<f32> = take
                .frames
                .iter()
                .filter(|frame| SLOPE_MIDDLE.contains(&frame.body.x))
                .map(|frame| frame.velocity.x)
                .collect();
            let mean = across.iter().sum::<f32>() / across.len().max(1) as f32;
            let climbed = take.frames.iter().any(|frame| frame.body.x > SHELF_X);
            let planned = params.body.ground_speed(gait.speed(), grade);
            held.push((gait, mean, planned, climbed));
        }
        let rideable = angle <= claim;
        let agrees = held.iter().all(|&(_, mean, planned, climbed)| {
            climbed && (mean / planned - 1.0).abs() <= SLOPE_SPEED_TOLERANCE
        });
        checks.push(ContractCheck {
            name: format!("slope {angle}°"),
            measured: held
                .iter()
                .map(|(gait, mean, planned, climbed)| {
                    format!(
                        "{} {mean:.1}{} (plan {planned:.1})",
                        gait.name(),
                        if *climbed { "" } else { " stuck" }
                    )
                })
                .collect::<Vec<_>>()
                .join(", ")
                + " m/s",
            claim: if rideable {
                format!(
                    "≤ max_ground_slope_degrees {claim}: climbed, within {:.0}% of the plan",
                    SLOPE_SPEED_TOLERANCE * 100.0
                )
            } else {
                format!("> max_ground_slope_degrees {claim}: never ridden")
            },
            holds: !rideable || agrees,
        });
    }
    Ok(checks)
}

/// A ballistic arc fitted to a flight: height `y0 + v0·t − ½·g·t²` from the
/// frame the jump was pressed on.
#[derive(Debug)]
struct Arc {
    v0: f32,
    gravity: f32,
    /// Furthest a frame strays from it, in metres.
    worst: f32,
    /// Planar speed at the first and last frame in the air.
    speeds: (f32, f32),
}

/// The flight that starts on the frame jump was pressed, up to the last
/// frame in free fall: past it something held the body, though grounding
/// may say so a frame late.
fn flight(frames: &[FrameRecord]) -> Option<Arc> {
    let pressed = frames.iter().position(|frame| frame.beat == "jump")?;
    // Each frame is recorded after its step: the launch is at the start of
    // the press frame's step, from where the frame before it left the body.
    let start = &frames[pressed.checked_sub(1)?];
    let falling = |pair: &[FrameRecord]| {
        let dt = pair[1].time - pair[0].time;
        let change = pair[1].velocity.y - pair[0].velocity.y;
        (change + STANDARD_GRAVITY * dt).abs() < FREE_FALL_SLACK * STANDARD_GRAVITY * dt
    };
    let held = frames[pressed..]
        .windows(2)
        .position(|pair| !falling(pair))
        .map_or(frames.len(), |index| pressed + index + 1);
    let air = &frames[pressed..held];
    if air.len() < 4 {
        return None;
    }
    let samples: Vec<(f32, f32)> = air
        .iter()
        .map(|frame| (frame.time - start.time, frame.body.y - start.body.y))
        .collect();
    let (y0, v0, half_g) = fit_quadratic(&samples)?;
    let worst = samples
        .iter()
        .map(|&(t, y)| (y0 + v0 * t + half_g * t * t - y).abs())
        .fold(0.0, f32::max);
    let planar = |frame: &FrameRecord| frame.velocity.x.hypot(frame.velocity.z);
    Some(Arc {
        v0,
        gravity: -2.0 * half_g,
        worst,
        speeds: (planar(&air[0]), planar(&air[air.len() - 1])),
    })
}

/// Least-squares `y = a + b·t + c·t²` through `samples`.
fn fit_quadratic(samples: &[(f32, f32)]) -> Option<(f32, f32, f32)> {
    let mut m = [[0.0f64; 3]; 3];
    let mut r = [0.0f64; 3];
    for &(t, y) in samples {
        let basis = [1.0, t as f64, (t * t) as f64];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += basis[i] * basis[j];
            }
            r[i] += basis[i] * y as f64;
        }
    }
    let det = |m: &[[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let whole = det(&m);
    if whole.abs() < 1.0e-12 {
        return None;
    }
    let solve = |column: usize| {
        let mut replaced = m;
        for row in 0..3 {
            replaced[row][column] = r[row];
        }
        (det(&replaced) / whole) as f32
    };
    Some((solve(0), solve(1), solve(2)))
}

/// A jump at speed on flat ground: the arc against a ballistic one, its
/// gravity against the claimed, its climb against the most a plan asks; and
/// the run-up's acceleration from standing against the plan's.
fn takeoff(
    config: &RunConfig,
    params: &GaitParams,
    gait: Gait,
) -> EngineResult<Vec<ContractCheck>> {
    let script = Script::new()
        .then(Beat::stand(0.3))
        .then(gait.beat(RUN_UP))
        .then(gait.jump(JUMP_BEAT));
    let take = run(&lane("takeoff", FLAT_Z, script), config)?;
    let arc = flight(&take.frames).ok_or_else(|| no_flight("the flat jump"))?;
    let accelerated = acceleration(&take.frames, gait.speed()).ok_or_else(|| {
        EngineError::InvalidState("the run-up never reached its speed".to_string())
    })?;
    let claimed = params.body.max_acceleration;
    let gait = gait.name();
    let gained = arc.speeds.1 / arc.speeds.0.max(1.0e-3) - 1.0;
    Ok(vec![
        ContractCheck {
            name: format!("acceleration {gait}"),
            measured: format!("{accelerated:.1} m/s²"),
            claim: format!(
                "max_acceleration {claimed} ≤ it, within {:.0}%",
                ACCELERATION_SLACK * 100.0
            ),
            holds: claimed <= accelerated && accelerated <= claimed * (1.0 + ACCELERATION_SLACK),
        },
        ContractCheck {
            name: format!("takeoff {gait}: climb"),
            measured: format!("{:.2} m/s", arc.v0),
            claim: format!("max_takeoff_climb {} ≤ it", params.flight.max_takeoff_climb),
            holds: params.flight.max_takeoff_climb <= arc.v0,
        },
        ContractCheck {
            name: format!("takeoff {gait}: gravity"),
            measured: format!("{:.2} m/s²", arc.gravity),
            claim: format!("gravity {} within {GRAVITY_TOLERANCE:.0e}", params.gravity),
            holds: (arc.gravity / params.gravity - 1.0).abs() <= GRAVITY_TOLERANCE,
        },
        ContractCheck {
            name: format!("takeoff {gait}: arc"),
            measured: format!(
                "strays {:.3} m; speed {:.2} → {:.2} m/s",
                arc.worst, arc.speeds.0, arc.speeds.1
            ),
            claim: format!(
                "ballistic: within {ARC_TOLERANCE} m, speed within {:.0}%",
                AIR_SPEED_TOLERANCE * 100.0
            ),
            holds: arc.worst <= ARC_TOLERANCE && gained.abs() <= AIR_SPEED_TOLERANCE,
        },
    ])
}

/// The mean acceleration from standing to `ACCELERATED` of `speed`, in m/s²:
/// from the last frame at rest to the first that far up.
fn acceleration(frames: &[FrameRecord], speed: f32) -> Option<f32> {
    let start = frames
        .iter()
        .take_while(|frame| frame.planar_speed() <= AT_REST)
        .last()?;
    let reached = frames
        .iter()
        .find(|frame| frame.planar_speed() >= ACCELERATED * speed)?;
    Some(reached.planar_speed() / (reached.time - start.time).max(f32::EPSILON))
}

/// A jump pressed the frame after running off the deck's edge, against one
/// on flat ground: coyote time must not cut the climb.
fn coyote(config: &RunConfig) -> EngineResult<ContractCheck> {
    let probe = Script::new()
        .then(Beat::stand(0.3))
        .then(Gait::Walk.beat(RUN_UP + 2.0));
    let take = run(&lane("coyote_probe", DECK_Z, probe), config)?;
    let off = take
        .frames
        .iter()
        .find(|frame| frame.body.x > DECK_EDGE && !frame.grounded)
        .ok_or_else(|| no_flight("the run off the deck"))?;
    let dt = 1.0 / config.frame_rate;
    let jump_at = off.time + dt;
    let script = Script::new()
        .then(Beat::stand(0.3))
        .then(Gait::Walk.beat(jump_at - 0.3))
        .then(Gait::Walk.jump(JUMP_BEAT));
    let flat = Script::new()
        .then(Beat::stand(0.3))
        .then(Gait::Walk.beat(RUN_UP))
        .then(Gait::Walk.jump(JUMP_BEAT));
    let flat = flight(&run(&lane("takeoff", FLAT_Z, flat), config)?.frames)
        .ok_or_else(|| no_flight("the flat jump"))?;
    let frames = run(&lane("coyote", DECK_Z, script), config)?.frames;
    let late = (jump_at - off.time) * 1.0e3;
    let (measured, holds) = match flight(&frames) {
        Some(coyote) => (
            format!("{:.2} m/s, pressed {late:.0} ms off the edge", coyote.v0),
            (coyote.v0 / flat.v0 - 1.0).abs() <= COYOTE_TOLERANCE,
        ),
        None => {
            let rose = frames
                .iter()
                .filter(|frame| frame.time >= jump_at)
                .map(|frame| frame.velocity.y)
                .fold(f32::MIN, f32::max);
            (
                format!("no jump {late:.0} ms off the edge: rose at {rose:.2} m/s at most"),
                false,
            )
        }
    };
    Ok(ContractCheck {
        name: "takeoff: coyote".to_string(),
        measured,
        claim: format!(
            "climb within {:.0}% of flat ground's {:.2} m/s",
            COYOTE_TOLERANCE * 100.0,
            flat.v0
        ),
        holds,
    })
}

/// The garden scenario `name`, the plan driving the body: how far its
/// grounded capsule rides above or sinks below the floor a cast finds under
/// it, across the station, where the plan says it rides. Where the floor
/// under it rises steeper than a plan rides, as up a lip's face, the plan
/// flies; where it falls away faster than gravity follows, as into a gap,
/// the plan rolls off it until the body is back on the floor. Neither is
/// judged, nor a frame leaving on a planned takeoff. Where it falls away by
/// less than the body skims, the plan rides on over it, and the capsule may
/// lift off by that much.
fn ride(config: &RunConfig, params: &GaitParams, name: &str) -> EngineResult<ContractCheck> {
    let scenario = find(name).ok_or_else(|| EngineError::InvalidState(format!("no {name}")))?;
    let station = scenario
        .judged
        .station()
        .ok_or_else(|| EngineError::InvalidState(format!("{name} crosses no station")))?;
    let tracked = RunConfig {
        track: true,
        ..*config
    };
    let take = run(&scenario, &tracked)?;
    let greatest_rise = params.body.max_ground_slope_degrees.to_radians().tan() * RIDE_SLOPE_RUN;
    let (mut above, mut below) = (0.0f32, 0.0f32);
    let mut rolling = false;
    for frame in &take.frames {
        let along = frame.body.coords.dot(&station.axis);
        if along < station.enter - 1.0 || along > station.leave + 1.0 {
            continue;
        }
        let (Some(floor), Some(behind)) = (frame.ride_floor, frame.ride_floor_behind) else {
            continue;
        };
        let speed = frame.velocity.x.hypot(frame.velocity.z).max(0.1);
        let fall = 0.5 * params.gravity * (RIDE_SLOPE_RUN / speed).powi(2);
        if behind - floor > fall + params.body.skim_height {
            rolling = true;
        }
        if rolling && (frame.bottom - floor).abs() > RIDE_TOLERANCE {
            continue;
        }
        rolling = false;
        if !frame.grounded || frame.locomotion != "grounded" || floor - behind > greatest_rise {
            continue;
        }
        above = above.max(frame.bottom - floor);
        below = below.max(floor - frame.bottom);
    }
    Ok(ContractCheck {
        name: format!("ride {name}"),
        measured: format!("{above:.3} m above, {below:.3} m below"),
        claim: format!(
            "on the cast floor within {RIDE_TOLERANCE} m, skimming {} m above",
            params.body.skim_height
        ),
        holds: above <= RIDE_TOLERANCE + params.body.skim_height && below <= RIDE_TOLERANCE,
    })
}

/// A face lane at `gait`: the speed across the ground the body keeps meeting
/// the face, against its share along the face's slope (`landed_speed` in the
/// planner: `v·cos²θ`), and how far it rises after leaving the brow, against
/// its climb there held for `BodyDims::rise_held` and then cut. And how
/// much later than on flat ground (`flat`) it crosses a line past the brow,
/// against what the planner charges for the speed the face takes.
fn face(
    config: &RunConfig,
    params: &GaitParams,
    angle: f32,
    lane_z: f32,
    gait: Gait,
    flat: f32,
) -> EngineResult<Vec<ContractCheck>> {
    let script = Script::new()
        .then(Beat::stand(0.3))
        .then(gait.beat(FACE_RUN));
    let frames = run(&lane("face", lane_z, script), config)?.frames;
    let missing = || EngineError::InvalidState(format!("face {angle}° {}: no", gait.name()));
    let before = frames
        .iter()
        .rev()
        .find(|frame| frame.body.x < FACE_X - 0.5 && frame.grounded)
        .ok_or_else(missing)?;
    let met_at = frames
        .iter()
        .position(|frame| frame.body.x > FACE_X - 0.5 && frame.velocity.y > 0.1 * before.velocity.x)
        .ok_or_else(missing)?;
    let met = &frames[met_at];
    let cos = angle.to_radians().cos();
    let claimed = before.velocity.x * cos * cos;
    let left_at = met_at
        + frames[met_at..]
            .iter()
            .position(|frame| !frame.grounded)
            .ok_or_else(missing)?;
    let left = &frames[left_at];
    let peak = frames[left_at..]
        .iter()
        .take_while(|frame| frame.velocity.y > -1.0)
        .map(|frame| frame.bottom)
        .fold(left.bottom, f32::max);
    // Contact was lost on average half a frame before the first frame that
    // reports it: that much of the held rise is already spent there.
    let frame_time = left.time - frames[left_at - 1].time;
    let held = params.body.rise_held - 0.5 * frame_time;
    let (climb, gravity) = (left.velocity.y, params.gravity);
    let claimed_rise = if climb - gravity * held > 0.0 {
        climb * held - 0.5 * gravity * held * held
    } else {
        climb.max(0.0).powi(2) / (2.0 * gravity)
    };
    let crossed = crossing(&frames, FACE_X + FACE_TIMED).ok_or_else(missing)?;
    let lost = crossed - flat;
    let (speed, weight) = (
        before.velocity.x,
        PlannerSettings::default().costs.landing_loss,
    );
    let charged = weight * (speed - claimed).powi(2)
        + recovery_time_lost(claimed, speed, params.body.max_acceleration);
    let name = format!("face {angle}° {}", gait.name());
    Ok(vec![
        ContractCheck {
            name: format!("{name}: met"),
            measured: format!(
                "{:.2} → {:.2} m/s across",
                before.velocity.x, met.velocity.x
            ),
            claim: format!(
                "v·cos²θ {claimed:.2} within {:.0}%",
                FACE_SPEED_TOLERANCE * 100.0
            ),
            holds: (met.velocity.x / claimed - 1.0).abs() <= FACE_SPEED_TOLERANCE,
        },
        ContractCheck {
            name: format!("{name}: brow"),
            measured: format!("climb {climb:.2} m/s, rises {:.3} m", peak - left.bottom),
            claim: format!(
                "held {} s: {claimed_rise:.3} within {ARC_TOLERANCE} m",
                params.body.rise_held
            ),
            holds: (peak - left.bottom - claimed_rise).abs() <= ARC_TOLERANCE,
        },
        ContractCheck {
            name: format!("{name}: time"),
            measured: format!("{lost:.3} s lost against flat ground"),
            claim: format!("costs.landing_loss and regaining v·cos²θ: {charged:.3} s covers it"),
            holds: lost <= charged,
        },
    ])
}

/// When a run on flat ground at `gait` crosses `x`, scripted as the face
/// runs are.
fn flat_crossing(config: &RunConfig, gait: Gait, x: f32) -> EngineResult<f32> {
    let script = Script::new()
        .then(Beat::stand(0.3))
        .then(gait.beat(FACE_RUN));
    let frames = run(&lane("flat", FLAT_Z, script), config)?.frames;
    crossing(&frames, x)
        .ok_or_else(|| EngineError::InvalidState(format!("a flat run never crossed x = {x:.1}")))
}

/// What a planned hop costs on level ground: how much later a body that
/// hops at `climb` crosses a line `HOP_RUN` past its takeoff than one
/// that runs on. The planner prices a hop at `costs.flight` seconds, which
/// must cover it.
fn hop(config: &RunConfig, gait: Gait, climb: f32) -> EngineResult<ContractCheck> {
    let script = |tail: Beat| {
        Script::new()
            .then(Beat::stand(0.3))
            .then(gait.beat(HOP_AT))
            .then(tail)
    };
    let plain = run(&lane("hop", FLAT_Z, script(gait.beat(HOP_TAIL))), config)?;
    let hopped = run(
        &lane(
            "hop",
            FLAT_Z,
            script(
                gait.beat(HOP_TAIL)
                    .hopping(gait.speed(), climb)
                    .named("hop"),
            ),
        ),
        config,
    )?;
    let takeoff = hopped
        .frames
        .iter()
        .find(|frame| frame.beat == "hop")
        .ok_or_else(|| EngineError::InvalidState("the hop never came".to_string()))?
        .body
        .x;
    let airborne = hopped
        .frames
        .iter()
        .skip_while(|frame| frame.beat != "hop")
        .take_while(|frame| frame.body.x < takeoff + HOP_RUN)
        .filter(|frame| !frame.grounded)
        .count();
    let line = takeoff + HOP_RUN;
    let crossed = |frames: &[FrameRecord]| crossing(frames, line);
    let (Some(ran), Some(hopped)) = (crossed(&plain.frames), crossed(&hopped.frames)) else {
        return Err(EngineError::InvalidState(format!(
            "a run never crossed x = {line:.1}"
        )));
    };
    let lost = hopped - ran;
    let price = PlannerSettings::default().costs.flight;
    Ok(ContractCheck {
        name: format!("hop {} {climb}: cost", gait.name()),
        measured: format!("{lost:.3} s lost over {airborne} frames in the air"),
        claim: format!("climbing {climb} m/s; costs.flight {price} s covers it"),
        holds: airborne > 0 && lost <= price,
    })
}

/// When the body first crosses `x`, interpolated between frames.
fn crossing(frames: &[FrameRecord], x: f32) -> Option<f32> {
    frames.windows(2).find_map(|pair| {
        let (before, after) = (&pair[0], &pair[1]);
        (before.body.x < x && after.body.x >= x).then(|| {
            let fraction = (x - before.body.x) / (after.body.x - before.body.x);
            before.time + fraction * (after.time - before.time)
        })
    })
}

fn no_flight(what: &str) -> EngineError {
    EngineError::InvalidState(format!("{what} never left the ground and landed"))
}
