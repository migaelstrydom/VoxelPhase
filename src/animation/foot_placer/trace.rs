//! Eyeball traces and CSV export over the shared scenario registry.
//!
//! - `trace_scenarios` prints per-frame placer state for every scenario:
//!   `cargo test --lib foot_placer::trace -- --ignored --nocapture`
//! - `export_csv` writes one CSV per scenario to `scratch/foot_traces/`
//!   for plotting (see `scratch/plot_traces.py`).
//!
//! Test-only module (`#[cfg(test)]` in `mod.rs`).

use std::fmt::Write as _;

use super::placer::{planar_distance, FootPhase};
use super::scenarios;
use super::sim::{simulate_gait, Frame};

fn phase_label(p: &FootPhase) -> &'static str {
    match p {
        FootPhase::Planted => "PLANTED",
        FootPhase::Stepping { .. } => "STEP   ",
    }
}

fn print_frames(label: &str, fps: f32, frames: &[Frame]) {
    println!("\n=== {label} (display {fps:.0} Hz) ===");
    println!(
        "frame  t     pelvisXZ          velXZ           yaw    phase  L[phase pos.xz plantedXYZ idealXZ err]                     R[...]"
    );
    for (f, s) in frames.iter().enumerate() {
        let l = &s.left;
        let r = &s.right;
        let l_err = planar_distance(l.ideal_xz, l.planted_position);
        let r_err = planar_distance(r.ideal_xz, r.planted_position);
        println!(
            "{f:>4}  {:>5.2}  ({:>5.2},{:>5.2})  ({:>4.2},{:>4.2})  {:>5.2}  {:>4.2}  L[{} ({:>5.2},{:>5.2}) p=({:>5.2},{:>5.2},{:>5.2}) i=({:>5.2},{:>5.2}) e={:>4.2}]  R[{} ({:>5.2},{:>5.2}) p=({:>5.2},{:>5.2},{:>5.2}) i=({:>5.2},{:>5.2}) e={:>4.2}]",
            s.time,
            s.pelvis.x, s.pelvis.z,
            s.velocity.x, s.velocity.z,
            s.yaw,
            s.gait_phase,
            phase_label(&l.phase), l.position.x, l.position.z, l.planted_position.x, l.planted_position.y, l.planted_position.z, l.ideal_xz.x, l.ideal_xz.z, l_err,
            phase_label(&r.phase), r.position.x, r.position.z, r.planted_position.x, r.planted_position.y, r.planted_position.z, r.ideal_xz.x, r.ideal_xz.z, r_err,
        );
    }
}

#[test]
#[ignore]
fn trace_scenarios() {
    for sc in scenarios::all() {
        let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);
        print_frames(sc.name, sc.fps, &frames);
    }
}

/// Write one CSV per scenario, including the terrain height directly
/// under each foot, for offline plotting.
#[test]
#[ignore]
fn export_csv() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/scratch/foot_traces");
    std::fs::create_dir_all(dir).unwrap();

    for sc in scenarios::all() {
        let frames = simulate_gait(sc.frames, sc.fps, sc.gait, sc.height, sc.input);
        let mut csv = String::new();
        csv.push_str(
            "time,pelvis_x,pelvis_y,pelvis_z,vel_x,vel_z,yaw,gait_phase,\
             l_step,l_x,l_y,l_z,l_terrain,l_planted_x,l_planted_y,l_planted_z,l_ideal_x,l_ideal_z,\
             r_step,r_x,r_y,r_z,r_terrain,r_planted_x,r_planted_y,r_planted_z,r_ideal_x,r_ideal_z\n",
        );
        for s in &frames {
            let l = &s.left;
            let r = &s.right;
            writeln!(
                csv,
                "{},{},{},{},{},{},{},{},\
                 {},{},{},{},{},{},{},{},{},{},\
                 {},{},{},{},{},{},{},{},{},{}",
                s.time,
                s.pelvis.x,
                s.pelvis.y,
                s.pelvis.z,
                s.velocity.x,
                s.velocity.z,
                s.yaw,
                s.gait_phase,
                u8::from(l.is_stepping()),
                l.position.x,
                l.position.y,
                l.position.z,
                (sc.height)(l.position.x, l.position.z),
                l.planted_position.x,
                l.planted_position.y,
                l.planted_position.z,
                l.ideal_xz.x,
                l.ideal_xz.z,
                u8::from(r.is_stepping()),
                r.position.x,
                r.position.y,
                r.position.z,
                (sc.height)(r.position.x, r.position.z),
                r.planted_position.x,
                r.planted_position.y,
                r.planted_position.z,
                r.ideal_xz.x,
                r.ideal_xz.z,
            )
            .unwrap();
        }
        let path = format!("{dir}/{}.csv", sc.name);
        std::fs::write(&path, csv).unwrap();
        println!("wrote {path}");
    }
}
