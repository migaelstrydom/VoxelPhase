//! A run, as text.

use std::fmt::Write;

use super::record::{FrameRecord, Take};

/// Every state change the run went through, then a row every `every` frames.
pub fn report(take: &Take, every: usize) -> String {
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
        "  {:>6} {:8} {:10} {:12} {:9} {:>5} {:>3} {:>6} {:>6} {:>7} {:>7} {:>6} {:>6}",
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
        "vy"
    );
    for frame in take.frames.iter().step_by(every.max(1)) {
        let _ = writeln!(out, "  {}", row(frame));
    }
    out
}

fn row(f: &FrameRecord) -> String {
    let above = |y: f32| {
        f.surface
            .map(|s| format!("{:+7.2}", y - s))
            .unwrap_or_else(|| "      -".to_string())
    };
    format!(
        "{:6.2} {:8} {:10} {:12} {:9} {:2.0}/{:2.0} {:>3} {} {} {} {} {:6.2} {:+6.2}",
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
        f.velocity.y
    )
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
