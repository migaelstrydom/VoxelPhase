//! Named test scenarios shared by traces, invariant sweeps and the CSV
//! exporter. Each scenario fixes a display rate, gait parameters, an
//! analytic terrain height field and a per-frame input stream, so every
//! consumer exercises identical conditions.
//!
//! Test-only module (`#[cfg(test)]` in `mod.rs`).

use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use nalgebra::Vector3;

use super::sim::{GaitParams, Input};

/// A self-contained placer scenario.
pub struct Scenario {
    pub name: &'static str,
    pub frames: usize,
    pub fps: f32,
    pub gait: GaitParams,
    pub height: fn(f32, f32) -> f32,
    pub input: fn(usize) -> Input,
}

/// Every scenario, for sweep-style invariants and CSV export. All run
/// at 30 Hz — the worst display rate the placer must look natural at.
pub fn all() -> Vec<Scenario> {
    vec![
        Scenario {
            name: "walk_flat",
            frames: 90,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: walk_1ms,
        },
        Scenario {
            name: "run_flat",
            frames: 90,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: run_ramp_5ms,
        },
        Scenario {
            name: "run_hard_accel",
            frames: 120,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: run_hard_accel_5ms,
        },
        Scenario {
            name: "start_stop",
            frames: 120,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: start_stop_2ms,
        },
        Scenario {
            name: "turn_in_place",
            frames: 75,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: turn_in_place_180,
        },
        Scenario {
            name: "turn_walk_90",
            frames: 90,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: turn_walk_90,
        },
        Scenario {
            name: "reverse_stand",
            frames: 75,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: reverse_from_stand,
        },
        Scenario {
            name: "reverse_running",
            frames: 105,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: reverse_while_running,
        },
        Scenario {
            name: "landing_slide",
            frames: 60,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: slide_4ms_to_rest,
        },
        Scenario {
            name: "incline_up",
            frames: 90,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: incline_20,
            input: walk_2ms_after_pause,
        },
        Scenario {
            name: "incline_down",
            frames: 90,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: decline_20,
            input: walk_2ms_after_pause,
        },
        Scenario {
            name: "steep_up",
            frames: 120,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: incline_45,
            input: walk_2ms_after_pause,
        },
        Scenario {
            name: "steep_down_run",
            frames: 120,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: decline_45,
            input: run_ramp_5ms,
        },
        Scenario {
            name: "rough",
            frames: 120,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: rough,
            input: walk_1p5ms,
        },
        Scenario {
            name: "hills",
            frames: 180,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: hills,
            input: walk_2ms_after_pause,
        },
        Scenario {
            name: "stairs",
            frames: 120,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: stairs,
            input: walk_1p5ms,
        },
        Scenario {
            name: "crouch_walk",
            frames: 90,
            fps: 30.0,
            gait: GaitParams::crouch(),
            height: flat,
            input: walk_1ms,
        },
        Scenario {
            name: "diagonal_yaw45",
            frames: 90,
            fps: 30.0,
            gait: GaitParams::walk(),
            height: flat,
            input: walk_diagonal_45,
        },
    ]
}

// === Terrains ===

pub fn flat(_x: f32, _z: f32) -> f32 {
    0.0
}

/// 20% grade rising along +z.
pub fn incline_20(_x: f32, z: f32) -> f32 {
    0.2 * z
}

/// 20% grade falling along +z.
pub fn decline_20(_x: f32, z: f32) -> f32 {
    -0.2 * z
}

/// 45% grade rising along +z — the steep-scramble regime where the
/// reach budget must shrink with slope.
pub fn incline_45(_x: f32, z: f32) -> f32 {
    0.45 * z
}

/// 45% grade falling along +z.
pub fn decline_45(_x: f32, z: f32) -> f32 {
    -0.45 * z
}

/// Bumpy ground: ±~8 cm undulations at sub-metre wavelengths.
pub fn rough(x: f32, z: f32) -> f32 {
    0.05 * (x * 5.3).sin() * (z * 4.1).cos() + 0.03 * (z * 7.7 + 1.3).sin()
}

/// Rolling hills: 0.4 m amplitude, 8 m wavelength along z.
pub fn hills(_x: f32, z: f32) -> f32 {
    0.4 * (z * TAU / 8.0).sin()
}

/// Voxel-style staircase: flat until z = 1, then 0.12 m risers every
/// 0.5 m of run.
pub fn stairs(_x: f32, z: f32) -> f32 {
    if z < 1.0 {
        0.0
    } else {
        0.12 * (1.0 + ((z - 1.0) / 0.5).floor())
    }
}

// === Input streams (30 Hz frame indexing) ===

fn t30(f: usize) -> f32 {
    f as f32 / 30.0
}

fn walk_1ms(f: usize) -> Input {
    if f < 15 {
        Input::still()
    } else {
        Input::moving(Vector3::new(0.0, 0.0, 1.0), 0.0)
    }
}

fn walk_1p5ms(f: usize) -> Input {
    if f < 15 {
        Input::still()
    } else {
        Input::moving(Vector3::new(0.0, 0.0, 1.5), 0.0)
    }
}

fn walk_2ms_after_pause(f: usize) -> Input {
    if f < 15 {
        Input::still()
    } else {
        Input::moving(Vector3::new(0.0, 0.0, 2.0), 0.0)
    }
}

/// Realistic start: intent held from frame 0, velocity ramps 0 → 5 m/s
/// over 0.5 s.
fn run_ramp_5ms(f: usize) -> Input {
    let speed = (t30(f) / 0.5).min(1.0) * 5.0;
    Input {
        velocity: Vector3::new(0.0, 0.0, speed),
        yaw: 0.0,
        intent: Vector3::new(0.0, 0.0, 1.0),
    }
}

/// Game-faithful start: `LocomotionConfig::ground_accel` is 40 m/s², so a
/// standing start hits the 5 m/s gait speed in 0.125 s — faster than
/// half a gait cycle.
fn run_hard_accel_5ms(f: usize) -> Input {
    let speed = (40.0 * t30(f)).min(5.0);
    Input {
        velocity: Vector3::new(0.0, 0.0, speed),
        yaw: 0.0,
        intent: Vector3::new(0.0, 0.0, 1.0),
    }
}

/// Accelerate to 2 m/s, walk, hard stop at t = 2 s.
fn start_stop_2ms(f: usize) -> Input {
    let t = t30(f);
    if t >= 2.0 {
        return Input::still();
    }
    let speed = (t / 0.3).min(1.0) * 2.0;
    Input {
        velocity: Vector3::new(0.0, 0.0, speed),
        yaw: 0.0,
        intent: Vector3::new(0.0, 0.0, 1.0),
    }
}

/// Stationary 180° turn over 1.5 s, then hold.
fn turn_in_place_180(f: usize) -> Input {
    let yaw = PI * (t30(f) / 1.5).min(1.0);
    Input {
        velocity: Vector3::zeros(),
        yaw,
        intent: Vector3::zeros(),
    }
}

/// Walk +z at 2 m/s for 1 s, carve a 90° left turn over 0.5 s, then
/// continue straight.
fn turn_walk_90(f: usize) -> Input {
    let t = t30(f);
    let yaw = FRAC_PI_2 * ((t - 1.0).clamp(0.0, 0.5) / 0.5);
    let dir = Vector3::new(yaw.sin(), 0.0, yaw.cos());
    Input::moving(dir * 2.0, yaw)
}

/// Stand facing +x; at t = 0.5 s "left" is pressed: accelerate 0 → 5 m/s
/// along −x while a yaw P-drive (rate 10/s) swings the model 180°.
fn reverse_from_stand(f: usize) -> Input {
    let tau = t30(f) - 0.5;
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
}

/// About-face at speed: run +z at 5 m/s for 1 s, then the stick reverses
/// — velocity decelerates through zero at 20 m/s² and accelerates to
/// −5 m/s while a yaw P-drive flips the facing 180°.
fn reverse_while_running(f: usize) -> Input {
    let t = t30(f);
    if t < 1.0 {
        let speed = (t / 0.5).min(1.0) * 5.0;
        return Input {
            velocity: Vector3::new(0.0, 0.0, speed),
            yaw: 0.0,
            intent: Vector3::new(0.0, 0.0, 1.0),
        };
    }
    let tau = t - 1.0;
    let v = (5.0 - 20.0 * tau).max(-5.0);
    let yaw = PI - PI * (-10.0 * tau).exp();
    Input {
        velocity: Vector3::new(0.0, 0.0, v),
        yaw,
        intent: Vector3::new(0.0, 0.0, -1.0),
    }
}

/// Uncommanded slide: 4 m/s decaying to rest over 1 s, no intent.
fn slide_4ms_to_rest(f: usize) -> Input {
    let speed = (4.0 * (1.0 - t30(f))).max(0.0);
    Input {
        velocity: Vector3::new(0.0, 0.0, speed),
        yaw: 0.0,
        intent: Vector3::zeros(),
    }
}

/// Walk 1.5 m/s along (1, 0, 1)/√2 with a 45° spawn yaw from frame 0.
fn walk_diagonal_45(f: usize) -> Input {
    let dir = Vector3::new(FRAC_PI_4.sin(), 0.0, FRAC_PI_4.cos());
    if f < 15 {
        Input {
            velocity: Vector3::zeros(),
            yaw: FRAC_PI_4,
            intent: Vector3::zeros(),
        }
    } else {
        Input::moving(dir * 1.5, FRAC_PI_4)
    }
}
