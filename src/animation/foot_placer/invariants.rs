//! Asserting scenario tests: the gait properties that keep breaking
//! are pinned here so regressions fail `cargo test` instead of
//! needing a trace eyeball.
//!
//! Test-only module (`#[cfg(test)]` in `mod.rs`).

use nalgebra::{Point3, Vector2, Vector3};

use super::capture_point::stance_offset;
use super::config::FootPlacerConfig;
use super::placer::{facing_from_yaw, planar_distance, FootPhase, FootSide, PlacerFoot};
use super::scenarios;
use super::sim::{
    self, simulate, simulate_gait, simulate_over, simulate_var_dt, simulate_with_suspend, takeoffs,
    Frame, GaitParams, Input,
};

/// Walk at 1 m/s: duty > 0.5, so one foot must always be planted.
#[test]
fn walk_never_lifts_both_feet() {
    let frames = simulate(180, 60.0, scenarios::flat, |f| {
        if f < 30 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 1.0), 0.0)
        }
    });
    for (i, s) in frames.iter().enumerate() {
        assert!(
            s.left.is_planted() || s.right.is_planted(),
            "both feet airborne at walking speed (frame {i})"
        );
    }
}

/// Takeoffs strictly alternate L/R during a steady walk.
#[test]
fn walk_takeoffs_alternate() {
    let frames = simulate(180, 60.0, scenarios::flat, |f| {
        if f < 30 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 1.0), 0.0)
        }
    });
    let events = takeoffs(&frames);
    assert!(events.len() >= 4, "expected several steps, got {events:?}");
    for pair in events.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot stepped twice in a row: {events:?}"
        );
    }
}

/// Steady-state takeoffs are evenly spaced (half a gait cycle apart,
/// within frame quantisation).
#[test]
fn walk_takeoffs_evenly_spaced() {
    let frames = simulate(240, 60.0, scenarios::flat, |f| {
        if f < 30 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 1.0), 0.0)
        }
    });
    let events = takeoffs(&frames);
    // Skip the start transient: first two events.
    let steady: Vec<usize> = events.iter().skip(2).map(|(_, f)| *f).collect();
    assert!(steady.len() >= 4, "not enough steady-state steps");
    let gaps: Vec<usize> = steady.windows(2).map(|w| w[1] - w[0]).collect();
    let min = *gaps.iter().min().unwrap() as f32;
    let max = *gaps.iter().max().unwrap() as f32;
    assert!(
        max / min.max(1.0) <= 1.6,
        "uneven takeoff spacing (frames): {gaps:?}"
    );
}

/// The same walk produces the same number of steps at 30 Hz and
/// 60 Hz display rates (substepping = frame-rate independence).
#[test]
fn step_count_is_display_rate_independent() {
    let walk_60 = simulate(180, 60.0, scenarios::flat, |f| {
        if f < 30 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 1.0), 0.0)
        }
    });
    let walk_30 = simulate(90, 30.0, scenarios::flat, |f| {
        if f < 15 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 1.0), 0.0)
        }
    });
    let n60 = takeoffs(&walk_60).len() as i32;
    let n30 = takeoffs(&walk_30).len() as i32;
    assert!(
        (n60 - n30).abs() <= 1,
        "step count diverges across display rates: 60Hz={n60} 30Hz={n30}"
    );
}

/// Intent held from rest fires the first step quickly, before the
/// body has accelerated to full speed.
#[test]
fn stationary_start_steps_promptly() {
    let frames = simulate(60, 60.0, scenarios::flat, |f| {
        let t = f as f32 / 60.0;
        let speed = (t / 0.5).min(1.0) * 5.0;
        Input {
            velocity: Vector3::new(0.0, 0.0, speed),
            yaw: 0.0,
            intent: Vector3::new(0.0, 0.0, 1.0),
        }
    });
    let events = takeoffs(&frames);
    assert!(
        events.first().map(|(_, f)| *f <= 30).unwrap_or(false),
        "no step within 0.5s of start intent: {events:?}"
    );
}

/// A stationary 180° turn reshuffles both feet (settle trigger on
/// rotating stance offsets) without ever lifting both at once.
#[test]
fn turn_in_place_steps_both_feet() {
    let frames = simulate(150, 60.0, scenarios::flat, |f| {
        let t = f as f32 / 60.0;
        let yaw = std::f32::consts::PI * (t / 1.5).min(1.0);
        Input {
            velocity: Vector3::zeros(),
            yaw,
            intent: Vector3::zeros(),
        }
    });
    let events = takeoffs(&frames);
    assert!(
        events.iter().any(|(s, _)| *s == FootSide::Left),
        "left foot never stepped during 180° turn"
    );
    assert!(
        events.iter().any(|(s, _)| *s == FootSide::Right),
        "right foot never stepped during 180° turn"
    );
    for (i, s) in frames.iter().enumerate() {
        assert!(
            s.left.is_planted() || s.right.is_planted(),
            "both feet airborne during turn-in-place (frame {i})"
        );
    }
}

/// A standing-start reversal (facing +x, walking off toward −x with
/// a fast 180° turn) must not flail: no foot re-lifts after a
/// near-instant stance, and once the turn completes the takeoffs
/// settle into strict L/R alternation. Pins two past bugs: a stale
/// `planted_yaw` (set at takeoff, not landing) re-releasing feet via
/// the turn trigger, and clock releases latched mid-swing firing the
/// moment the foot lands.
#[test]
fn reverse_direction_settles_into_rhythm() {
    use std::f32::consts::{FRAC_PI_2, PI};
    let frames = simulate(150, 60.0, scenarios::flat, |f| {
        let tau = f as f32 / 60.0 - 0.5;
        if tau < 0.0 {
            return Input {
                velocity: Vector3::zeros(),
                yaw: FRAC_PI_2,
                intent: Vector3::zeros(),
            };
        }
        let speed = (tau / 0.5).min(1.0) * 5.0;
        let yaw = -FRAC_PI_2 + PI * (-10.0 * tau).exp();
        Input {
            velocity: Vector3::new(-speed, 0.0, 0.0),
            yaw,
            intent: Vector3::new(-1.0, 0.0, 0.0),
        }
    });

    assert_min_stance_frames(&frames, 3);

    // After the turn transient (t >= 1.0s) takeoffs strictly alternate.
    let steady: Vec<_> = takeoffs(&frames)
        .into_iter()
        .filter(|(_, f)| *f >= 60)
        .collect();
    assert!(steady.len() >= 4, "not enough steady steps: {steady:?}");
    for pair in steady.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot stepped twice in a row after reversal: {steady:?}"
        );
    }

    // No overstayed stances: a planted foot must never trail the
    // pelvis by more than the leg can plausibly reach. The clock
    // desync bug left feet planted ~0.76 m behind (leg length is
    // 0.5) before a giant catch-up swing.
    for (i, s) in frames.iter().enumerate() {
        for foot in [&s.left, &s.right] {
            if foot.is_planted() {
                let trail = planar_distance(s.pelvis, foot.planted_position);
                assert!(
                    trail <= 0.55,
                    "planted foot {} m from pelvis at frame {i} (overstayed stance)",
                    trail
                );
            }
        }
    }

    // The turn itself is single-support: below running speed one
    // foot must stay planted while the other repositions. Flight
    // phases are only legitimate once it's a genuine run.
    for (i, s) in frames.iter().enumerate() {
        let speed = Vector2::new(s.velocity.x, s.velocity.z).magnitude();
        if speed <= 2.5 {
            assert!(
                s.left.is_planted() || s.right.is_planted(),
                "both feet airborne at {speed:.1} m/s during turn (frame {i})"
            );
        }
    }
}

/// A steady 5 m/s run keeps the feet in antiphase: takeoffs strictly
/// alternate and are never near-simultaneous. Pins the two-footed
/// hop bug, where a reactive release fired the second foot right
/// behind the first and the gait phase-locked into synchronized
/// hops (both feet swinging forward together).
#[test]
fn run_takeoffs_stagger_and_alternate() {
    let frames = simulate(180, 60.0, scenarios::flat, |f| {
        let t = f as f32 / 60.0;
        let speed = (t / 0.5).min(1.0) * 5.0;
        Input {
            velocity: Vector3::new(0.0, 0.0, speed),
            yaw: 0.0,
            intent: Vector3::new(0.0, 0.0, 1.0),
        }
    });
    let events = takeoffs(&frames);
    assert!(events.len() >= 6, "expected several steps: {events:?}");
    for pair in events.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot took off twice in a row: {events:?}"
        );
        assert!(
            pair[1].1 - pair[0].1 >= 3,
            "near-simultaneous takeoffs (hop): {events:?}"
        );
    }
}

/// Hip→foot distance stays within the configured stretch budget
/// across walking, running, reversal, slide, and incline scenarios.
/// Pins the stride reach budget and the overstretch release.
/// Swinging feet get a small extra allowance: a reactive release
/// can fire at the limit and the pelvis keeps moving for the first
/// instants of the swing before the arc catches up.
///
/// Scenarios use realistic acceleration (the game ramps at ~10
/// m/s²); an instantaneous 0→5 m/s teleport cannot satisfy both
/// the stretch budget and staggered takeoffs at once. The slide's
/// first 0.4 s is exempt for the same reason: feet planted together
/// at 4 m/s must either hop or briefly overstretch on the push-off
/// step, and the placer deliberately chooses the stretch.
#[test]
fn legs_never_overstretch() {
    use std::f32::consts::{FRAC_PI_2, PI};

    let run = simulate(150, 60.0, scenarios::flat, |f| {
        let t = f as f32 / 60.0;
        let speed = (t / 0.5).min(1.0) * 5.0;
        Input {
            velocity: Vector3::new(0.0, 0.0, speed),
            yaw: 0.0,
            intent: Vector3::new(0.0, 0.0, 1.0),
        }
    });
    let reversal = simulate(150, 60.0, scenarios::flat, |f| {
        let tau = f as f32 / 60.0 - 0.5;
        if tau < 0.0 {
            return Input {
                velocity: Vector3::zeros(),
                yaw: FRAC_PI_2,
                intent: Vector3::zeros(),
            };
        }
        let speed = (tau / 0.5).min(1.0) * 5.0;
        let yaw = -FRAC_PI_2 + PI * (-10.0 * tau).exp();
        Input {
            velocity: Vector3::new(-speed, 0.0, 0.0),
            yaw,
            intent: Vector3::new(-1.0, 0.0, 0.0),
        }
    });
    let slide = simulate(90, 60.0, scenarios::flat, |f| {
        let t = f as f32 / 60.0;
        let speed = (4.0 * (1.0 - t)).max(0.0);
        Input {
            velocity: Vector3::new(0.0, 0.0, speed),
            yaw: 0.0,
            intent: Vector3::zeros(),
        }
    });
    let incline = simulate(120, 60.0, scenarios::incline_20, |f| {
        if f < 15 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 2.0), 0.0)
        }
    });
    let steep_up = simulate(150, 60.0, scenarios::incline_45, |f| {
        if f < 15 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 2.0), 0.0)
        }
    });
    // Game-accel sprint down a 45% grade — the slope-scramble regime
    // where the flat reach budget let plants land beyond the leg.
    let steep_down = simulate(180, 60.0, scenarios::decline_45, |f| {
        if f < 15 {
            Input::still()
        } else {
            let speed = (40.0 * (f - 15) as f32 / 60.0).min(5.0);
            Input::moving(Vector3::new(0.0, 0.0, speed), 0.0)
        }
    });

    let cfg = FootPlacerConfig::default();
    let max_ext = sim::LEG_LENGTH * cfg.max_leg_stretch_ratio;
    for (name, frames, skip) in [
        ("run", &run, 0),
        ("reversal", &reversal, 0),
        ("slide", &slide, 24),
        ("incline", &incline, 0),
        ("steep_up", &steep_up, 0),
        ("steep_down", &steep_down, 0),
    ] {
        assert_stretch_within(name, frames, max_ext, skip);
    }
}

/// Planted feet sit on the terrain surface while walking an incline.
#[test]
fn incline_planted_feet_track_terrain() {
    let frames = simulate(120, 60.0, scenarios::incline_20, |f| {
        if f < 15 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 2.0), 0.0)
        }
    });
    for (i, s) in frames.iter().enumerate().skip(30) {
        for foot in [&s.left, &s.right] {
            if foot.is_planted() {
                let surface =
                    scenarios::incline_20(foot.planted_position.x, foot.planted_position.z);
                assert!(
                    (foot.planted_position.y - surface).abs() < 0.02,
                    "planted foot off terrain surface at frame {i}: y={} surface={surface}",
                    foot.planted_position.y
                );
            }
        }
    }
}

/// After stopping, feet settle near neutral stance and stop stepping.
#[test]
fn walk_to_idle_settles_without_micro_stepping() {
    let frames = simulate(240, 60.0, scenarios::flat, |f| {
        if f < 60 {
            Input::moving(Vector3::new(0.0, 0.0, 1.0), 0.0)
        } else {
            Input::still()
        }
    });
    // Allow 1s after the stop for finishing/settle steps; the last
    // second must be entirely quiet.
    let late_events: Vec<_> = takeoffs(&frames)
        .into_iter()
        .filter(|(_, f)| *f >= 180)
        .collect();
    assert!(
        late_events.is_empty(),
        "feet still stepping long after stop: {late_events:?}"
    );
    let last = frames.last().unwrap();
    for foot in [&last.left, &last.right] {
        let err = planar_distance(foot.ideal_xz, foot.planted_position);
        assert!(
            err < 0.06,
            "foot settled far from neutral stance: err={err}"
        );
    }
}

// === Sweeps over the scenario registry ===

/// A planted foot is an anchor: its xz must not move between frames
/// while it stays planted, in any scenario. Horizontal foot slide is
/// the single most visible artefact this system exists to remove.
#[test]
fn planted_feet_never_slide() {
    for sc in scenarios::all() {
        let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);
        for (i, pair) in frames.windows(2).enumerate() {
            for foot_of in [(|s: &Frame| &s.left) as fn(&Frame) -> &PlacerFoot, |s| {
                &s.right
            }] {
                let (a, b) = (foot_of(&pair[0]), foot_of(&pair[1]));
                if a.is_planted() && b.is_planted() {
                    let slide = planar_distance(a.planted_position, b.planted_position);
                    assert!(
                        slide < 1e-4,
                        "{}: planted foot slid {slide:.4} m at frame {}",
                        sc.name,
                        i + 1
                    );
                }
            }
        }
    }
}

/// No foot may dip below the terrain surface, planted or swinging, in
/// any scenario. Pins the swing-arc terrain clearance (stairs, rough
/// ground, uphill walks).
#[test]
fn feet_never_clip_terrain() {
    for sc in scenarios::all() {
        let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);
        for (i, s) in frames.iter().enumerate() {
            for foot in [&s.left, &s.right] {
                let surface = (sc.height)(foot.position.x, foot.position.z);
                assert!(
                    foot.position.y >= surface - 0.02,
                    "{}: foot at y={:.3} below terrain {:.3} at frame {i}",
                    sc.name,
                    foot.position.y,
                    surface
                );
            }
        }
    }
}

/// A landing foot must arrive at the terrain surface, not teleport to
/// it: the vertical gap between the final swing target and the actual
/// terrain at the plant point stays small in every scenario. Pins the
/// terrain-aware landing target (without it, swings aim at a flat
/// pelvis-relative height and pop up/down at plant on any slope).
#[test]
fn plants_land_without_vertical_pop() {
    for sc in scenarios::all() {
        let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);
        for (i, pair) in frames.windows(2).enumerate() {
            for foot_of in [(|s: &Frame| &s.left) as fn(&Frame) -> &PlacerFoot, |s| {
                &s.right
            }] {
                let (a, b) = (foot_of(&pair[0]), foot_of(&pair[1]));
                if let (FootPhase::Stepping { to, .. }, FootPhase::Planted) = (a.phase, b.phase) {
                    let pop = (b.planted_position.y - to.y).abs();
                    assert!(
                        pop <= 0.03,
                        "{}: foot popped {pop:.3} m vertically at plant (frame {})",
                        sc.name,
                        i + 1
                    );
                }
            }
        }
    }
}

/// Crouch walking: slow, controlled gait — strict alternation and at
/// least one foot always planted.
#[test]
fn crouch_walk_alternates_with_continuous_support() {
    let sc = scenarios::all()
        .into_iter()
        .find(|s| s.name == "crouch_walk")
        .unwrap();
    let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);
    for (i, s) in frames.iter().enumerate() {
        assert!(
            s.left.is_planted() || s.right.is_planted(),
            "both feet airborne during crouch walk (frame {i})"
        );
    }
    let events = takeoffs(&frames);
    assert!(
        events.len() >= 4,
        "expected several crouch steps: {events:?}"
    );
    for pair in events.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot stepped twice in a row crouching: {events:?}"
        );
    }
}

/// Walking with a non-zero spawn yaw (45° diagonal) behaves like the
/// same walk along +z: same step count, strict alternation.
#[test]
fn spawn_yaw_does_not_change_gait() {
    let diagonal = scenarios::all()
        .into_iter()
        .find(|s| s.name == "diagonal_yaw45")
        .unwrap();
    let diag_frames = simulate_gait(
        diagonal.frames,
        diagonal.fps,
        diagonal.gait,
        diagonal.height,
        diagonal.input,
    );
    let straight_frames = simulate(diagonal.frames, diagonal.fps, scenarios::flat, |f| {
        if f < 15 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 1.5), 0.0)
        }
    });

    let n_diag = takeoffs(&diag_frames).len() as i32;
    let n_straight = takeoffs(&straight_frames).len() as i32;
    assert!(
        (n_diag - n_straight).abs() <= 1,
        "diagonal walk steps {n_diag} != straight walk steps {n_straight}"
    );
    for pair in takeoffs(&diag_frames).windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot stepped twice in a row on diagonal walk"
        );
    }
}

/// Jittered display frame times (24–45 Hz) produce the same step count
/// as a fixed 30 Hz display: the substep loop owns sequencing.
#[test]
fn variable_dt_step_count_matches_fixed() {
    let input = |f: usize| {
        if f < 15 {
            Input::still()
        } else {
            Input::moving(Vector3::new(0.0, 0.0, 1.5), 0.0)
        }
    };
    // Deterministic jitter cycling 24/45/30/36 Hz; mean ≈ 1/31 s.
    let jitter = [1.0 / 24.0, 1.0 / 45.0, 1.0 / 30.0, 1.0 / 36.0];
    let var = simulate_var_dt(
        90,
        |f| jitter[f % jitter.len()],
        GaitParams::walk(),
        scenarios::flat,
        input,
    );
    let fixed = simulate(90, 30.0, scenarios::flat, input);

    // Compare steps per simulated second (total sim time differs
    // slightly between the two dt patterns).
    let var_rate = takeoffs(&var).len() as f32 / var.last().unwrap().time;
    let fixed_rate = takeoffs(&fixed).len() as f32 / fixed.last().unwrap().time;
    assert!(
        (var_rate - fixed_rate).abs() / fixed_rate <= 0.2,
        "step cadence diverges under jittered dt: var={var_rate:.2}/s fixed={fixed_rate:.2}/s"
    );
}

/// An about-face at full running speed: the gait must decelerate,
/// reverse and re-accelerate without flailing — minimum stances hold,
/// the leg stretch budget holds (with swing slack), and once the new
/// direction is established the takeoffs alternate strictly.
#[test]
fn running_reversal_settles_into_rhythm() {
    let sc = scenarios::all()
        .into_iter()
        .find(|s| s.name == "reverse_running")
        .unwrap();
    let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);

    assert_min_stance_frames(&frames, 2);

    let cfg = FootPlacerConfig::default();
    let max_ext = sim::LEG_LENGTH * cfg.max_leg_stretch_ratio;
    assert_stretch_within("reverse_running", &frames, max_ext, 0);

    // After the reversal is established (t >= 2.0 s) takeoffs alternate.
    let steady: Vec<_> = takeoffs(&frames)
        .into_iter()
        .filter(|(_, f)| *f as f32 / sc.fps >= 2.0)
        .collect();
    assert!(steady.len() >= 3, "not enough steady steps: {steady:?}");
    for pair in steady.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot stepped twice in a row after running reversal: {steady:?}"
        );
    }
}

/// Fuzz sweep: 20 s of pseudo-random but game-plausible inputs
/// (bounded acceleration and yaw rate, intermittent intent) over rough
/// terrain at 30 Hz. Asserts the properties that must hold for *any*
/// input: planted feet never slide, feet never clip the terrain, and
/// the leg stretch budget holds.
#[test]
fn random_inputs_hold_core_invariants() {
    // Deterministic LCG so the sequence is reproducible.
    let mut seed: u64 = 0x5DEECE66D;
    let mut rand = move || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 33) as f32) / (1u64 << 30) as f32 - 1.0 // 31 bits → [-1, 1)
    };

    // Piecewise-constant targets, re-rolled every ~0.5 s; velocity and
    // yaw chase them with game-like rates (20 m/s², 10 rad/s).
    let frames = 600;
    let dt = 1.0 / 30.0;
    let mut targets = Vec::with_capacity(frames);
    let mut target_v = Vector3::zeros();
    let mut target_yaw = 0.0f32;
    let mut intent_on = true;
    let mut velocity = Vector3::zeros();
    let mut yaw = 0.0f32;
    for f in 0..frames {
        if f % 15 == 0 {
            let speed = (rand() * 0.5 + 0.5) * 5.0; // [0, 5]
            let dir_yaw = rand() * std::f32::consts::PI;
            intent_on = rand() > -0.5; // 75% of windows have intent
            target_v = if intent_on {
                Vector3::new(dir_yaw.sin(), 0.0, dir_yaw.cos()) * speed
            } else {
                Vector3::zeros()
            };
            target_yaw = dir_yaw;
        }
        let dv = target_v - velocity;
        let max_dv = 20.0 * dt;
        velocity += if dv.norm() > max_dv {
            dv.normalize() * max_dv
        } else {
            dv
        };
        let dyaw = (target_yaw - yaw).rem_euclid(std::f32::consts::TAU);
        let dyaw = if dyaw > std::f32::consts::PI {
            dyaw - std::f32::consts::TAU
        } else {
            dyaw
        };
        yaw += dyaw.clamp(-10.0 * dt, 10.0 * dt);
        targets.push((velocity, yaw, intent_on));
    }

    let frames = simulate_gait(
        frames,
        30.0,
        GaitParams::walk(),
        scenarios::rough,
        move |f| {
            let (velocity, yaw, intent_on) = targets[f];
            Input {
                velocity,
                yaw,
                intent: if intent_on {
                    velocity
                } else {
                    Vector3::zeros()
                },
            }
        },
    );

    let cfg = FootPlacerConfig::default();
    let max_ext = sim::LEG_LENGTH * cfg.max_leg_stretch_ratio;
    for (i, pair) in frames.windows(2).enumerate() {
        for foot_of in [(|s: &Frame| &s.left) as fn(&Frame) -> &PlacerFoot, |s| {
            &s.right
        }] {
            let (a, b) = (foot_of(&pair[0]), foot_of(&pair[1]));
            if a.is_planted() && b.is_planted() {
                let slide = planar_distance(a.planted_position, b.planted_position);
                assert!(
                    slide < 1e-4,
                    "fuzz: planted foot slid {slide:.4} m at frame {}",
                    i + 1
                );
            }
        }
    }
    for (i, s) in frames.iter().enumerate() {
        for foot in [&s.left, &s.right] {
            let surface = scenarios::rough(foot.position.x, foot.position.z);
            assert!(
                foot.position.y >= surface - 0.02,
                "fuzz: foot at y={:.3} below terrain {:.3} at frame {i}",
                foot.position.y,
                surface
            );
        }
    }
    assert_stretch_within("fuzz", &frames, max_ext, 0);
}

/// A game-faithful hard start (40 m/s² to 5 m/s) settles into a steady
/// run: strict L/R alternation and evenly spaced takeoffs once the
/// transient is over. Pins the in-phase / galloping gait seen in game
/// when reactive releases fight the schedule during violent
/// acceleration.
#[test]
fn hard_accel_run_settles_into_even_rhythm() {
    let sc = scenarios::all()
        .into_iter()
        .find(|s| s.name == "run_hard_accel")
        .unwrap();
    let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);

    // Steady window: everything after 1.5 s.
    let steady: Vec<_> = takeoffs(&frames)
        .into_iter()
        .filter(|(_, f)| *f as f32 / sc.fps >= 1.5)
        .collect();
    assert!(steady.len() >= 6, "not enough steady steps: {steady:?}");
    for pair in steady.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot took off twice in a row at steady run: {steady:?}"
        );
    }
    let gaps: Vec<usize> = steady.windows(2).map(|w| w[1].1 - w[0].1).collect();
    let min = *gaps.iter().min().unwrap() as f32;
    let max = *gaps.iter().max().unwrap() as f32;
    assert!(
        max / min.max(1.0) <= 1.7,
        "galloping takeoff spacing at steady run (frames): {gaps:?}"
    );
}

/// Steady-state takeoffs split the gait cycle in half: for consecutive
/// takeoffs A→B→A, the offset `2·(t_B − t_A)/(t_A' − t_A)` must stay
/// near π (1.0 in these units). Pins the phase-authority property the
/// in-game recordings exposed: with step events allowed to re-anchor
/// the clock, off-antiphase splits (0.6/1.4) were *stable* attractors.
/// Bounds allow ±1 frame of 30 Hz takeoff quantisation.
#[test]
fn steady_takeoffs_are_antiphase() {
    for name in [
        "walk_flat",
        "run_flat",
        "run_hard_accel",
        "incline_up",
        "rough",
    ] {
        let sc = scenarios::all()
            .into_iter()
            .find(|s| s.name == name)
            .unwrap();
        let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);
        let steady: Vec<usize> = takeoffs(&frames)
            .into_iter()
            .filter(|(_, f)| *f as f32 / sc.fps >= 1.5)
            .map(|(_, f)| f)
            .collect();
        assert!(steady.len() >= 5, "{name}: not enough steady steps");
        for w in steady.windows(3) {
            let offset = 2.0 * (w[1] - w[0]) as f32 / (w[2] - w[0]) as f32;
            assert!(
                (0.75..=1.33).contains(&offset),
                "{name}: takeoff split {offset:.2}π at frame {} (antiphase lost)",
                w[0]
            );
        }
    }
}

/// At steady run speed the stance is hip-symmetric: each foot's
/// planted position, measured along the travel direction relative to
/// the pelvis, averages out near zero. Pins the duty-scaled plant-ahead
/// distance — without it the whole stance sat ahead of the hips at
/// running duty and the legs read as permanently tilted forward.
#[test]
fn run_stance_is_hip_symmetric() {
    let sc = scenarios::all()
        .into_iter()
        .find(|s| s.name == "run_hard_accel")
        .unwrap();
    let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);

    for foot_of in [(|s: &Frame| &s.left) as fn(&Frame) -> &PlacerFoot, |s| {
        &s.right
    }] {
        let mut sum = 0.0;
        let mut n = 0usize;
        for s in frames.iter().filter(|s| s.time >= 1.5) {
            let foot = foot_of(s);
            if foot.is_planted() {
                // Travel is +z in this scenario.
                sum += foot.planted_position.z - s.pelvis.z;
                n += 1;
            }
        }
        assert!(n > 10, "not enough planted samples");
        let mean = sum / n as f32;
        assert!(
            mean.abs() <= 0.06,
            "stance biased {mean:.3} m along travel (legs tilted)"
        );
    }
}

/// A one-frame airborne blip (terrain-seam contact loss) while running
/// must not disturb the gait: the planted foot keeps its anchor through
/// the blip (no replant teleport) and the takeoffs stay alternating and
/// evenly spaced afterwards. Pins the displacement-gated resume.
#[test]
fn seam_blip_does_not_break_gait() {
    let blip_frame = 45; // t = 1.5 s, mid steady run
    let input = |f: usize| {
        let speed = (40.0 * f as f32 / 30.0).min(5.0);
        Input {
            velocity: Vector3::new(0.0, 0.0, speed),
            yaw: 0.0,
            intent: Vector3::new(0.0, 0.0, 1.0),
        }
    };
    let frames = simulate_with_suspend(
        120,
        |_| 1.0 / 30.0,
        GaitParams::walk(),
        scenarios::flat,
        input,
        |f| f == blip_frame,
    );

    // Anchors survive the blip: any foot planted on both sides of the
    // blip keeps its planted position.
    let (before, after) = (&frames[blip_frame - 1], &frames[blip_frame + 1]);
    for foot_of in [(|s: &Frame| &s.left) as fn(&Frame) -> &PlacerFoot, |s| {
        &s.right
    }] {
        let (a, b) = (foot_of(before), foot_of(after));
        if a.is_planted() && b.is_planted() {
            assert!(
                planar_distance(a.planted_position, b.planted_position) < 1e-4,
                "planted anchor teleported across a seam blip"
            );
        }
    }

    // Gait rhythm continues: alternation + even spacing after the blip.
    let steady: Vec<_> = takeoffs(&frames)
        .into_iter()
        .filter(|(_, f)| *f > blip_frame + 3)
        .collect();
    assert!(steady.len() >= 5, "gait died after seam blip: {steady:?}");
    for pair in steady.windows(2) {
        assert_ne!(
            pair[0].0, pair[1].0,
            "same foot took off twice in a row after seam blip: {steady:?}"
        );
    }
    let gaps: Vec<usize> = steady.windows(2).map(|w| w[1].1 - w[0].1).collect();
    let min = *gaps.iter().min().unwrap() as f32;
    let max = *gaps.iter().max().unwrap() as f32;
    assert!(
        max / min.max(1.0) <= 1.7,
        "galloping after seam blip (frames): {gaps:?}"
    );
}

/// Walking off a ledge: no foot may plant where the landing probe found
/// no ground.
///
/// Grounding used to come from those probes, so a step aimed past an
/// edge turned the character airborne and the placer suspended itself.
/// Support now comes from the capsule's contacts, which are still on the
/// ledge, so nothing upstream vetoes the step any more — the placer must
/// refuse the target itself and shorten the step.
#[test]
fn steps_never_plant_where_there_is_no_ground() {
    const LEDGE_Z: f32 = 1.0;
    let terrain = |_x: f32, z: f32| (z < LEDGE_Z).then_some(0.0);
    let frames = simulate_over(
        120,
        |_| 1.0 / 60.0,
        GaitParams::walk(),
        terrain,
        |f| {
            if f < 15 {
                Input::still()
            } else {
                Input::moving(Vector3::new(0.0, 0.0, 1.5), 0.0)
            }
        },
        |_| false,
    );

    let mut worst = f32::NEG_INFINITY;
    for (i, s) in frames.iter().enumerate() {
        for foot in [&s.left, &s.right] {
            if !foot.is_planted() {
                continue;
            }
            worst = worst.max(foot.planted_position.z);
            assert!(
                foot.planted_position.z < LEDGE_Z + 0.02,
                "foot planted {:.3} m past the ledge at frame {i}",
                foot.planted_position.z - LEDGE_Z
            );
        }
    }
    // The pelvis walks well past the edge, so an unshortened gait would
    // have planted far out over the void — the assertion above is only
    // meaningful because the body got there.
    let last = frames.last().unwrap();
    assert!(
        last.pelvis.z > LEDGE_Z + 0.5,
        "the body never reached the ledge ({:.2} m)",
        last.pelvis.z
    );
    assert!(worst.is_finite(), "no foot was ever planted");
}

/// The shortening rule must not disturb ordinary walking: over terrain
/// that is everywhere, no step is ever marked shortened.
#[test]
fn steps_over_solid_ground_are_never_shortened() {
    let frames = simulate_over(
        180,
        |_| 1.0 / 60.0,
        GaitParams::walk(),
        |x, z| Some(scenarios::rough(x, z)),
        |f| {
            if f < 20 {
                Input::still()
            } else {
                Input::moving(Vector3::new(0.0, 0.0, 2.0), 0.0)
            }
        },
        |_| false,
    );
    for (i, s) in frames.iter().enumerate() {
        assert!(
            !s.left.landing_shortened && !s.right.landing_shortened,
            "a step was shortened over solid ground at frame {i}"
        );
    }
}

// === Shared assertion helpers ===

/// Every plant must hold for at least `min_frames` before the next
/// takeoff of the same foot.
fn assert_min_stance_frames(frames: &[Frame], min_frames: usize) {
    for (label, foot_of) in [
        ("left", (|s: &Frame| &s.left) as fn(&Frame) -> &PlacerFoot),
        ("right", |s| &s.right),
    ] {
        let mut plant_frame = None;
        for (i, pair) in frames.windows(2).enumerate() {
            let (a, b) = (foot_of(&pair[0]), foot_of(&pair[1]));
            if a.is_stepping() && b.is_planted() {
                plant_frame = Some(i + 1);
            }
            if a.is_planted() && b.is_stepping() {
                if let Some(p) = plant_frame.take() {
                    assert!(
                        i + 1 - p >= min_frames,
                        "{label} foot re-lifted after {} frames of stance (frame {})",
                        i + 1 - p,
                        i + 1
                    );
                }
            }
        }
    }
}

/// Hip→foot distance stays within `max_ext` (+slack) for all frames
/// past `skip`.
fn assert_stretch_within(name: &str, frames: &[Frame], max_ext: f32, skip: usize) {
    for (i, s) in frames.iter().enumerate().skip(skip) {
        let facing = facing_from_yaw(s.yaw);
        for foot in [&s.left, &s.right] {
            let off = stance_offset(facing, sim::HIP_WIDTH, foot.side.side_sign());
            let hip = Point3::new(s.pelvis.x + off.x, s.pelvis.y, s.pelvis.z + off.y);
            let stretch = (hip - foot.position).norm();
            // Slack: the release trigger is evaluated once per
            // substep, so the crossing can overshoot by one
            // substep of pelvis travel (~0.02 m at 5 m/s).
            let limit = if foot.is_planted() {
                max_ext + 0.04
            } else {
                max_ext + 0.1
            };
            assert!(
                stretch <= limit,
                "{name}: {:?} foot stretched to {stretch:.3} (limit {limit:.3}) at frame {i} \
                 [planted={}, other_planted={}, speed={:.2}, vel=({:.2},{:.2})]",
                foot.side,
                foot.is_planted(),
                if foot.side == FootSide::Left {
                    s.right.is_planted()
                } else {
                    s.left.is_planted()
                },
                Vector2::new(s.velocity.x, s.velocity.z).magnitude(),
                s.velocity.x,
                s.velocity.z,
            );
        }
    }
}
