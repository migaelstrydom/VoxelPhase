//! The placer under a random plan of the way ahead: footfalls that hold,
//! churn, swap feet, withdraw, and flights whose landing passes from foot
//! to foot, some of them steered back in the air. Whatever the plan does,
//! the drawn feet must keep to the leg's envelope and move as smoothly as
//! they do with no plan at all.
//!
//! Test-only module (`#[cfg(test)]` in `mod.rs`).

use nalgebra::{Point3, Vector3};

use super::capture_point::stance_offset;
use super::config::FootPlacerConfig;
use super::placer::{facing_from_yaw, PlacerFoot};
use super::planned::{PlannedStep, PlannedSteps};
use super::scenarios;
use super::sim::{simulate_planned, Frame, GaitParams, Input, RigDims, HIP_WIDTH, LEG_LENGTH};

const FPS: f32 = 60.0;
const FRAMES: usize = 900;
const SEEDS: u64 = 40;

/// Deterministic LCG, uniform in [0, 1).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32) / (1u64 << 24) as f32
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.next()
    }
}

/// What the plan does through one window of frames.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode {
    /// No plan: the legs step on their own.
    Unplanned,
    /// Footfalls fixed in the world.
    Steady,
    /// Footfalls that move every frame.
    Churn,
    /// Each foot given the other's footfall.
    Swapped,
    /// The body in the air, its landing handed between the feet.
    Flight,
}

/// One window of the script: the body's target motion and the plan's mode.
#[derive(Clone, Copy, Debug)]
struct Window {
    start: usize,
    velocity: Vector3<f32>,
    yaw: f32,
    mode: Mode,
    /// How far ahead of the hip each foot's footfall lies.
    ahead: [f32; 2],
    /// The frame a flight lands on.
    lands: usize,
    /// Whether the plan lands the flight along the way the body was going,
    /// as it does until the player steers back in the air.
    along_old: bool,
}

/// A seed's script: its windows, the body's motion frame by frame, and
/// where that motion has the pelvis, so a plan can know where the body
/// comes down.
struct Script {
    windows: Vec<Window>,
    inputs: Vec<(Vector3<f32>, f32)>,
    pelvis_xz: Vec<Vector3<f32>>,
}

impl Script {
    fn new(seed: u64, planned: bool) -> Self {
        let mut rng = Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) ^ 0x5DEECE66D);
        let mut windows = Vec::new();
        let mut start = 0;
        while start < FRAMES {
            let len = rng.range(10.0, 40.0) as usize;
            let speed = if rng.next() < 0.2 {
                0.0
            } else {
                rng.range(0.5, 8.0)
            };
            let yaw = rng.range(-1.0, 1.0) * std::f32::consts::PI;
            let mode = if !planned {
                Mode::Unplanned
            } else {
                match rng.next() {
                    r if r < 0.15 => Mode::Unplanned,
                    r if r < 0.45 => Mode::Steady,
                    r if r < 0.65 => Mode::Churn,
                    r if r < 0.75 => Mode::Swapped,
                    _ => Mode::Flight,
                }
            };
            let window = Window {
                start,
                velocity: Vector3::new(yaw.sin(), 0.0, yaw.cos()) * speed,
                yaw,
                mode,
                ahead: [rng.range(-0.3, 0.5), rng.range(-0.3, 0.5)],
                lands: start + len,
                along_old: false,
            };
            // Half the flights are steered back half way through: the plan
            // lands them along the way the body was going until it turns,
            // then where it comes down.
            if mode == Mode::Flight && rng.next() < 0.5 {
                let half = len / 2;
                windows.push(Window {
                    along_old: true,
                    ..window
                });
                windows.push(Window {
                    start: start + half,
                    velocity: -window.velocity,
                    ..window
                });
            } else {
                windows.push(window);
            }
            start += len;
        }

        let dt = 1.0 / FPS;
        let mut velocity = Vector3::zeros();
        let mut yaw = 0.0f32;
        let mut inputs = Vec::with_capacity(FRAMES);
        let mut pelvis_xz = Vec::with_capacity(FRAMES);
        let mut at = Vector3::zeros();
        for f in 0..FRAMES {
            let window = windows[Self::index(&windows, f)];
            let dv = window.velocity - velocity;
            let max_dv = 20.0 * dt;
            velocity += if dv.norm() > max_dv {
                dv.normalize() * max_dv
            } else {
                dv
            };
            let turn = super::placer::angle_diff(window.yaw, yaw);
            yaw += turn.clamp(-10.0 * dt, 10.0 * dt);
            at += velocity * dt;
            inputs.push((velocity, yaw));
            pelvis_xz.push(at);
        }
        Self {
            windows,
            inputs,
            pelvis_xz,
        }
    }

    fn index(windows: &[Window], f: usize) -> usize {
        windows.partition_point(|w| w.start <= f) - 1
    }

    fn window(&self, f: usize) -> Window {
        self.windows[Self::index(&self.windows, f)]
    }

    /// Where the pelvis is over the ground at frame `f`, or the last frame.
    fn pelvis_at(&self, f: usize) -> Point3<f32> {
        Point3::from(self.pelvis_xz[f.min(FRAMES - 1)])
    }
}

/// The plan's footfalls for one frame, kept between frames so steady
/// footfalls stay put in the world.
struct Planner {
    rng: Rng,
    window_start: Option<usize>,
    fixed: [Point3<f32>; 2],
    landing_foot: usize,
}

impl Planner {
    fn new(seed: u64) -> Self {
        Self {
            rng: Rng(seed ^ 0xA5A5_5A5A),
            window_start: None,
            fixed: [Point3::origin(); 2],
            landing_foot: 0,
        }
    }

    fn steps(&mut self, script: &Script, f: usize) -> PlannedSteps {
        let window = script.window(f);
        let on_ground = |p: Point3<f32>| Point3::new(p.x, scenarios::rough(p.x, p.z), p.z);
        if self.window_start != Some(window.start) {
            self.window_start = Some(window.start);
            let facing = facing_from_yaw(window.yaw);
            let forward = Vector3::new(facing.x, 0.0, facing.y);
            let side = Vector3::new(forward.z, 0.0, -forward.x);
            for (i, sign) in [(0, 1.0), (1, -1.0)] {
                let comes_down = if window.mode != Mode::Flight {
                    script.pelvis_at(f + self.rng.range(4.0, 20.0) as usize)
                } else if window.along_old {
                    let flown = (window.lands - f) as f32 / FPS;
                    script.pelvis_at(f) + script.inputs[f].0 * flown
                } else {
                    script.pelvis_at(window.lands)
                };
                let at = comes_down
                    + forward * window.ahead[i]
                    + side * (sign * 0.12 + self.rng.range(-0.05, 0.05));
                self.fixed[i] = on_ground(at);
            }
            self.landing_foot = (self.rng.next() < 0.5) as usize;
        }
        let step = |at: Point3<f32>| Some(PlannedStep { at, landing: None });
        match window.mode {
            Mode::Unplanned => PlannedSteps::default(),
            Mode::Steady => PlannedSteps {
                left: step(self.fixed[0]),
                right: step(self.fixed[1]),
            },
            Mode::Swapped => PlannedSteps {
                left: step(self.fixed[1]),
                right: step(self.fixed[0]),
            },
            Mode::Churn => {
                let mut jitter = |p: Point3<f32>| {
                    on_ground(
                        p + Vector3::new(self.rng.range(-0.3, 0.3), 0.0, self.rng.range(-0.3, 0.3)),
                    )
                };
                PlannedSteps {
                    left: step(jitter(self.fixed[0])),
                    right: step(jitter(self.fixed[1])),
                }
            }
            Mode::Flight => {
                if self.rng.next() < 0.5 {
                    self.landing_foot = 1 - self.landing_foot;
                }
                let lands_in = (window.lands - f) as f32 / FPS;
                let mut steps = [step(self.fixed[0]), step(self.fixed[1])];
                steps[self.landing_foot] = Some(PlannedStep {
                    at: self.fixed[self.landing_foot],
                    landing: Some(lands_in),
                });
                PlannedSteps {
                    left: steps[0],
                    right: steps[1],
                }
            }
        }
    }
}

fn run(seed: u64, planned: bool) -> (Script, Vec<Frame>) {
    let script = Script::new(seed, planned);
    let mut planner = Planner::new(seed);
    let frames = simulate_planned(
        RigDims::player(),
        FootPlacerConfig::default(),
        FRAMES,
        |_| 1.0 / FPS,
        GaitParams::walk(),
        |x, z| Some(scenarios::rough(x, z)),
        |f| {
            let (velocity, yaw) = script.inputs[f];
            Input::moving(velocity, yaw)
        },
        |f| script.window(f).mode == Mode::Flight,
        |f, _, _| planner.steps(&script, f),
    );
    (script, frames)
}

/// The worst of one measure: its value and where.
#[derive(Clone, Copy, Debug, Default)]
struct Worst {
    value: f32,
    seed: u64,
    frame: usize,
}

impl Worst {
    fn see(&mut self, value: f32, seed: u64, frame: usize) {
        if value > self.value {
            *self = Self { value, seed, frame };
        }
    }
}

/// How far the drawn feet strayed from the leg's envelope.
#[derive(Debug, Default)]
struct Envelope {
    /// The most a foot's motion against the pelvis changed from one frame
    /// to the next, in metres a frame. A foot that jumps changes it by the
    /// jump; one swung however fast, smoothly, by no more than the gait's
    /// own steps do.
    kink: Worst,
    /// The furthest a foot was from its hip.
    stretch: Worst,
    /// The highest a foot was above its hip.
    above_hip: Worst,
}

impl Envelope {
    fn measure(&mut self, seed: u64, frames: &[Frame]) {
        let hip = |s: &Frame, foot: &PlacerFoot| {
            let off = stance_offset(facing_from_yaw(s.yaw), HIP_WIDTH, foot.side.side_sign());
            Point3::new(s.pelvis.x + off.x, s.pelvis.y, s.pelvis.z + off.y)
        };
        for (i, s) in frames.iter().enumerate() {
            if !s.feet_placed {
                continue;
            }
            for foot in [&s.left, &s.right] {
                let hip = hip(s, foot);
                self.stretch.see((foot.position - hip).norm(), seed, i);
                self.above_hip.see(foot.position.y - hip.y, seed, i);
            }
            if i > 1 && frames[i - 1].feet_placed && frames[i - 2].feet_placed {
                let (a, b) = (&frames[i - 2], &frames[i - 1]);
                let feet = [(&a.left, &b.left, &s.left), (&a.right, &b.right, &s.right)];
                for (fa, fb, fc) in feet {
                    let before = (fb.position - b.pelvis) - (fa.position - a.pelvis);
                    let after = (fc.position - s.pelvis) - (fb.position - b.pelvis);
                    self.kink.see((after - before).norm(), seed, i);
                }
            }
        }
    }
}

fn envelope(planned: bool) -> Envelope {
    let mut envelope = Envelope::default();
    for seed in 0..SEEDS {
        let (_, frames) = run(seed, planned);
        envelope.measure(seed, &frames);
    }
    envelope
}

/// Whatever the plan does, the feet move as they do without one: no foot
/// jumps, none rises over its hip, and none is drawn further from the hip
/// than the leg's stretch with the slack a swing is given.
#[test]
fn random_plans_keep_the_feet_in_the_legs_envelope() {
    let reach = LEG_LENGTH * FootPlacerConfig::default().max_leg_stretch_ratio + 0.1;
    let unplanned = envelope(false);
    let planned = envelope(true);
    for (plan, envelope) in [("without a plan", &unplanned), ("with a plan", &planned)] {
        let Envelope {
            stretch, above_hip, ..
        } = envelope;
        assert!(
            above_hip.value <= 0.01,
            "{plan}, a foot rose {:.2} m over its hip (seed {}, frame {})",
            above_hip.value,
            above_hip.seed,
            above_hip.frame
        );
        assert!(
            stretch.value <= reach,
            "{plan}, a foot was drawn {:.2} m from its hip, past {reach:.2} (seed {}, frame {})",
            stretch.value,
            stretch.seed,
            stretch.frame
        );
    }
    let (kink, gait) = (planned.kink, unplanned.kink.value);
    assert!(
        kink.value <= gait + 0.05,
        "with a plan, a foot's motion changed by {:.2} m in a frame, against {gait:.2} m \
         in the gait's own steps (seed {}, frame {})",
        kink.value,
        kink.seed,
        kink.frame
    );
}
