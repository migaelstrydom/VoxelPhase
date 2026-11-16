//! A run, as text.

use std::fmt::Write;

use super::record::{FrameRecord, Take};
use super::traversal::{summarise, TraversalSummary};

/// Every state change the run went through, then a row every `every` frames,
/// then how it crossed its station if it has one. With `feet`, each row ends
/// with each foot against the pelvis: planted (`P`) or swinging (`s`), where
/// it was drawn, and after `a` where the placer anchors it.
pub fn report(take: &Take, every: usize, feet: bool) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "== {} ({} frames)", take.scenario, take.frames.len());
    let _ = writeln!(out, "-- transitions");
    let mut previous: Option<&FrameRecord> = None;
    for frame in &take.frames {
        if let Some(p) = previous {
            for (what, before, after) in [
                (
                    "locomotion",
                    p.locomotion.as_str(),
                    frame.locomotion.as_str(),
                ),
                ("pose", p.pose, frame.pose),
                ("upper", p.upper.as_str(), frame.upper.as_str()),
            ] {
                if before != after {
                    let _ = writeln!(
                        out,
                        "  {:6.2}s {:10} {:>10} -> {:<10} depth {} pitch {:4.0}°",
                        frame.time,
                        what,
                        before,
                        after,
                        depth(frame),
                        frame.body_pitch
                    );
                }
            }
        }
        previous = Some(frame);
    }
    let _ = writeln!(out, "-- frames");
    let _ = writeln!(
        out,
        "  {:>6} {:8} {:10} {:12} {:9} {:>5} {:>3} {:>6} {:>6} {:>7} {:>7} {:>6} {:>6} {:>20} {:>12}",
        "t",
        "beat",
        "locomotion",
        "pose",
        "upper",
        "pitch",
        "gnd",
        "depth",
        "wave",
        "body-sf",
        "pelv-sf",
        "speed",
        "vy",
        "body",
        "others v/w"
    );
    for frame in take.frames.iter().step_by(every.max(1)) {
        let _ = writeln!(
            out,
            "  {}{}",
            row(frame),
            if feet {
                feet_columns(frame)
            } else {
                String::new()
            }
        );
    }
    if let Some(summary) = traversal(take) {
        let _ = writeln!(out, "-- traversal");
        let _ = writeln!(out, "  {}", TraversalSummary::header_with_feet());
        let _ = writeln!(out, "  {}", summary.row(&take.scenario));
    }
    out
}

/// How the run crossed its station, if its scenario has one.
pub fn traversal(take: &Take) -> Option<TraversalSummary> {
    take.station
        .as_ref()
        .map(|station| summarise(&take.frames, station))
}

fn row(f: &FrameRecord) -> String {
    let above = |y: f32| {
        f.surface
            .map(|s| format!("{:+7.2}", y - s))
            .unwrap_or_else(|| "      -".to_string())
    };
    format!(
        "{:6.2} {:8} {:10} {:12} {:9} {:2.0}/{:2.0} {:>3} {} {} {} {} {:6.2} {:+6.2} {:6.2} {:6.2} {:6.2} {:6.2}/{:5.2}",
        f.time,
        f.beat,
        f.locomotion,
        f.pose,
        f.upper,
        f.body_pitch,
        f.pitch_asked,
        if f.grounded { "yes" } else { "" },
        depth(f),
        wave(f),
        above(f.body.y),
        above(f.pelvis.y),
        f.planar_speed(),
        f.velocity.y,
        f.body.x,
        f.body.y,
        f.body.z,
        f.others.0,
        f.others.1
    )
}

/// Each foot against the pelvis: planted or swinging, where it was drawn, and
/// where the placer anchors it.
fn feet_columns(f: &FrameRecord) -> String {
    f.feet
        .iter()
        .map(|foot| {
            let drawn = foot.position - f.pelvis;
            let anchor = foot.anchor - f.pelvis;
            format!(
                " {}{:+5.2},{:+5.2},{:+5.2} a{:+5.2},{:+5.2},{:+5.2}",
                if foot.planted { "P" } else { "s" },
                drawn.x,
                drawn.y,
                drawn.z,
                anchor.x,
                anchor.y,
                anchor.z
            )
        })
        .collect()
}

/// How far the surface stands over the still water level: swell and ripples.
fn wave(f: &FrameRecord) -> String {
    f.surface
        .zip(f.level)
        .map(|(surface, level)| format!("{:+6.2}", surface - level))
        .unwrap_or_else(|| "     -".to_string())
}

fn depth(f: &FrameRecord) -> String {
    f.water_depth()
        .map(|d| format!("{d:6.2}"))
        .unwrap_or_else(|| "     -".to_string())
}
