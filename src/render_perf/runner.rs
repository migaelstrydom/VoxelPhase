use std::collections::VecDeque;

use image::RgbaImage;

use crate::core::error::{EngineError, EngineResult};
use crate::level::Level;

use super::harness::GameHarness;
use super::record::{RenderFrameRecord, RenderRun};
use super::scenario::GrenadeBlast;

/// How a run is played and drawn.
#[derive(Debug, Clone, Copy)]
pub struct RunConfig {
    /// Simulated seconds per frame; the game's 60 Hz by default.
    pub frame_dt: f32,
    /// Simulated seconds to run for.
    pub duration: f32,
    /// Render target size. The default is what a 1200 × 800 game window
    /// renders at on a Retina display.
    pub width: u32,
    pub height: u32,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            frame_dt: 1.0 / 60.0,
            duration: 6.0,
            width: 2400,
            height: 1600,
        }
    }
}

/// Play `scenario` on `level` and record every frame.
///
/// Returns the run and, if asked for, an image of its last frame — taken
/// after the timed frames, since reading pixels back waits on the GPU.
///
/// The last frames run are not in the record: their profiles are published
/// at the start of frames that never come.
pub fn run(
    level: &Level,
    level_name: &str,
    scenario: &GrenadeBlast,
    config: RunConfig,
    snapshot: bool,
) -> EngineResult<(RenderRun, Option<RgbaImage>)> {
    let mut harness = GameHarness::open(level, config.width, config.height, config.frame_dt)?;

    let (x, z) = scenario.site;
    let surface = harness.surface_at(x, z).ok_or_else(|| {
        EngineError::InvalidState(format!("no terrain surface at the site ({x}, {z})"))
    })?;
    let (eye, look) = scenario.camera(surface);
    harness.place_camera(eye, look);

    let frame_count = (config.duration / config.frame_dt).round() as usize;
    let mut grenade = None;
    let mut dropped_at = scenario.drop_at;
    let mut detonated_at = None;
    let mut frames = Vec::with_capacity(frame_count);
    // Frames run whose render profile has not been published yet, with the
    // renderer's number for each.
    let mut pending: VecDeque<(u64, RenderFrameRecord)> = VecDeque::new();

    for _ in 0..frame_count {
        if grenade.is_none() && harness.sim_time() >= scenario.drop_at {
            dropped_at = harness.sim_time();
            let position = surface + nalgebra::Vector3::y() * scenario.lift;
            grenade = Some(harness.drop_grenade(position)?);
        }

        let sample = harness.step();
        let sim_time = harness.sim_time();

        if let (Some(entity), None) = (grenade, detonated_at) {
            if !harness.is_alive(entity) {
                detonated_at = Some(sim_time);
            }
        }

        pending.push_back((
            sample.render_frame,
            RenderFrameRecord {
                sim_time,
                timing: sample.timing,
                physics: sample.physics,
                profile: Default::default(),
            },
        ));

        // Every frame up to the one just published is complete. One whose
        // profile was never published (a frame abandoned before submit) is
        // dropped rather than given another frame's.
        let published = sample.finished_render;
        while pending
            .front()
            .is_some_and(|(frame, _)| *frame <= published.frame)
        {
            let (frame, mut record) = pending.pop_front().expect("front checked above");
            if frame == published.frame {
                record.profile = published.clone();
                frames.push(record);
            }
        }
    }

    let image = if snapshot {
        Some(harness.snapshot()?)
    } else {
        None
    };

    let run = RenderRun {
        level: level_name.to_string(),
        width: config.width,
        height: config.height,
        frame_dt: config.frame_dt,
        dropped_at,
        detonated_at,
        frames,
    };
    Ok((run, image))
}
