//! Printing what the metrics found, and exporting the raw frames.
//!
//! Two audiences, two formats. The text report is for reading now: a verdict
//! per property, ordered worst first, with the number that produced it. The CSV
//! is for looking harder later — one row per frame, everything recorded, so a
//! suspicion raised by the report can be plotted rather than argued about.

use std::fmt::Write as _;
use std::path::Path;

use crate::core::error::{EngineError, EngineResult};

use super::metrics::{FootMetrics, GaitMetrics, Verdict};
use super::take::{Side, Take};

/// The full report for one take's worth of analysis windows.
pub fn report(windows: &[GaitMetrics]) -> String {
    let mut out = String::new();
    for (index, window) in windows.iter().enumerate() {
        if index > 0 {
            out.push('\n');
        }
        out.push_str(&window_report(window));
    }
    out
}

fn window_report(m: &GaitMetrics) -> String {
    let mut out = String::new();

    let _ = writeln!(
        out,
        "{} / {}  ({} over {}, {:.1}s, {:.2} m/s mean)",
        m.scenario, m.window, m.frames, m.ground, m.duration, m.mean_speed
    );
    let _ = writeln!(
        out,
        "  rig       pelvis rides {:.0} mm up, rig designed for {:.0} mm, legs {:.0} mm long",
        m.ride_height * 1000.0,
        m.designed_height * 1000.0,
        m.leg_length * 1000.0
    );
    let _ = writeln!(
        out,
        "  support   double {:.0}%   flight {:.0}%   airborne {:.0}%",
        m.double_support * 100.0,
        m.flight * 100.0,
        m.airborne_fraction * 100.0
    );
    let _ = writeln!(
        out,
        "  cycle     {:.2} m measured vs {:.2} m planned{}",
        m.cycle_distance_measured,
        m.cycle_distance_designed,
        match m.takeoff_split {
            Some(split) => format!("   takeoff split {:.2}", split),
            None => String::new(),
        }
    );

    for foot in [&m.left, &m.right] {
        out.push_str(&foot_line(foot));
    }

    // Worst first: the point of a report is that the first line is the one to
    // act on.
    let mut checks: Vec<_> = m.checks.iter().collect();
    checks.sort_by(|a, b| b.verdict.cmp(&a.verdict));
    for check in checks {
        let _ = writeln!(
            out,
            "  [{}] {:<14} {}",
            check.verdict.tag(),
            check.name,
            check.detail
        );
    }

    out
}

fn foot_line(foot: &FootMetrics) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "  {:<9} {} steps   stance {:.0} ms   swing {:.0} ms   duty {:.2}   reach {:.0}%   \
         trail {:.0} cm   lead {:.0} cm   swing speed {:.1}x body",
        foot.side.label(),
        foot.steps,
        foot.stance_mean * 1000.0,
        foot.swing_mean * 1000.0,
        foot.duty_measured,
        foot.extension_peak * 100.0,
        foot.trail_max * 100.0,
        foot.lead_max * 100.0,
        foot.swing_speed_ratio
    );
    out
}

/// One line per scenario, for reading a whole catalogue run at a glance.
pub fn summary_line(m: &GaitMetrics) -> String {
    let worst = m
        .checks
        .iter()
        .filter(|c| c.verdict == m.verdict())
        .map(|c| c.name)
        .next()
        .unwrap_or("-");
    format!(
        "{} {:<16} duty {:.2}/{:.2}  reach {:>3.0}%  over {:>3.0}mm  {}",
        m.verdict().tag(),
        m.scenario,
        m.left.duty_measured.max(m.right.duty_measured),
        m.left.duty_designed,
        m.left.extension_peak.max(m.right.extension_peak) * 100.0,
        m.left.overreach_max.max(m.right.overreach_max) * 1000.0,
        if m.verdict() == Verdict::Ok {
            "".to_string()
        } else {
            format!("worst: {worst}")
        }
    )
}

/// Write every recorded frame as CSV.
///
/// Deliberately wide. The whole reason to keep a per-frame record is that the
/// column nobody thought to print is the one the next bug needs.
pub fn write_csv(take: &Take, path: &Path) -> EngineResult<()> {
    let mut out = String::new();
    out.push_str(
        "frame,time,beat,pose,grounded,pelvis_x,pelvis_y,pelvis_z,speed,vy,yaw,\
         gait_phase,stride_phase,stride_activity,duty_planned,swing_planned,trigger,cycle_planned",
    );
    for side in Side::BOTH {
        let s = side.label();
        let _ = write!(
            out,
            ",{s}_planted,{s}_x,{s}_y,{s}_z,\
             {s}_want_x,{s}_want_y,{s}_want_z,\
             {s}_anchor_x,{s}_anchor_z,{s}_ideal_x,{s}_ideal_z,{s}_hip_x,{s}_hip_y,{s}_hip_z,\
             {s}_extension,{s}_overreach,{s}_since_plant,{s}_pre_lift"
        );
    }
    out.push('\n');

    for frame in &take.frames {
        let _ = write!(
            out,
            "{},{:.4},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
            frame.index,
            frame.time,
            frame.beat,
            frame.pose,
            frame.grounded as u8,
            frame.pelvis.x,
            frame.pelvis.y,
            frame.pelvis.z,
            frame.horizontal_speed(),
            frame.velocity.y,
            frame.yaw,
            frame.gait_phase,
            frame.stride_phase,
            frame.stride_activity
        );
        match frame.timing {
            Some(t) => {
                let _ = write!(
                    out,
                    ",{:.4},{:.4},{:.4},{:.4}",
                    t.duty_factor, t.swing_duration, t.trigger_threshold, t.cycle_distance
                );
            }
            None => out.push_str(",,,,"),
        }
        for side in Side::BOTH {
            let foot = frame.foot(side);
            let _ = write!(
                out,
                ",{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},\
                 {:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                foot.planted as u8,
                foot.rendered.x,
                foot.rendered.y,
                foot.rendered.z,
                foot.requested.x,
                foot.requested.y,
                foot.requested.z,
                foot.anchor.x,
                foot.anchor.z,
                foot.ideal.x,
                foot.ideal.z,
                foot.hip.x,
                foot.hip.y,
                foot.hip.z,
                foot.extension(),
                foot.overreach(take.leg_length),
                foot.since_plant.min(999.0),
                foot.pre_lift
            );
        }
        out.push('\n');
    }

    std::fs::write(path, out)
        .map_err(|e| EngineError::InvalidState(format!("writing {}: {e}", path.display())))
}
