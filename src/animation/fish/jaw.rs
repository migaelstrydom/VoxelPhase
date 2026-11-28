//! A fish's mouth: a lower jaw that works a little with every breath and
//! now and then drops wide in a slow gulp, as a carp's does at the
//! surface.
//!
//! ```text
//!   gape ▲      gulp                    gulp, and again
//!   wide │     ╭──╮                    ╭──╮╭──╮
//!        │    ╱    ╲                  ╱    ╲╱  ╲
//!        │~~~╱~~~~~~╲~~~~~~~~~~~~~~~~╱~~~~~~~~~~╲~~~   breath
//!        └──────────────────────────────────────────▶ clock
//!             between gulps: a seeded wait
//! ```

use std::f32::consts::TAU;

use nalgebra::{UnitQuaternion, Vector3};

use super::rig::hashed;
use crate::model::Transform;

/// Share of a gulp spent opening, and where it starts to close.
const OPENED: f32 = 0.35;
const CLOSING: f32 = 0.5;
/// The pause before a gulp that follows straight on from another, s.
const AGAIN_AFTER: f32 = 0.15;

/// A lower jaw that breathes and gulps, turning on a hinge across the
/// corners of its mouth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GulpingJaw {
    /// The model's part it is.
    pub part: usize,
    /// A point on its hinge, in the entity's frame.
    pub pivot: Vector3<f32>,
    /// Its hinge's axis, in the entity's frame, signed so that a positive
    /// turn opens its mouth.
    pub axis: Vector3<f32>,
    /// How far it drops in a gulp, radians.
    pub wide: f32,
    /// How far it drops with each breath, radians.
    pub breath: f32,
    /// Breaths a second.
    pub breath_rate: f32,
    /// How long a gulp takes, opening and closing, s.
    pub gulp_for: f32,
    /// Shortest and longest it waits between gulps, s.
    pub between: (f32, f32),
    /// Share of its gulps followed straight away by another.
    pub again: f32,
    /// Seed of its gulps.
    pub seed: u32,
}

/// A jaw and where it is through its gulps.
#[derive(Debug, Clone, Copy)]
pub struct Mouth {
    /// The jaw.
    pub jaw: GulpingJaw,
    /// When the gulp under way began, on the rig's clock.
    started: Option<f32>,
    /// When it next gulps, on the rig's clock.
    next: f32,
    /// How many gulps it has made.
    gulps: u32,
}

impl Mouth {
    /// `jaw` shut, its first gulp a wait away.
    pub fn new(jaw: GulpingJaw) -> Self {
        let (shortest, longest) = jaw.between;
        Self {
            jaw,
            started: None,
            next: shortest + (longest - shortest) * hashed(jaw.seed, 0),
            gulps: 0,
        }
    }

    /// Carry it on to `clock`: begin the gulp that is due, and finish the
    /// one that is over.
    pub fn advance(&mut self, clock: f32) {
        if let Some(started) = self.started {
            if clock - started < self.jaw.gulp_for {
                return;
            }
            self.started = None;
            self.gulps += 1;
            let roll = |salt: u32| hashed(self.jaw.seed.wrapping_add(self.gulps * 7919), salt);
            let (shortest, longest) = self.jaw.between;
            self.next = clock
                + if roll(1) < self.jaw.again {
                    AGAIN_AFTER
                } else {
                    shortest + (longest - shortest) * roll(2)
                };
        }
        if clock >= self.next {
            self.started = Some(clock);
        }
    }

    /// Gulp now, unless it is gulping already.
    pub fn gulp(&mut self, clock: f32) {
        self.started.get_or_insert(clock);
    }

    /// How far open it is at `clock`, radians.
    pub fn gape(&self, clock: f32) -> f32 {
        let jaw = &self.jaw;
        let breath = jaw.breath * 0.5 * (1.0 - (TAU * jaw.breath_rate * clock).cos());
        let gulp = self.started.map_or(0.0, |started| {
            let through = ((clock - started) / jaw.gulp_for).clamp(0.0, 1.0);
            jaw.wide * gulp_shape(through)
        });
        breath.max(gulp)
    }

    /// The jaw's pose at `clock`.
    pub fn pose(&self, clock: f32) -> Transform {
        let turn = UnitQuaternion::from_scaled_axis(self.jaw.axis * self.gape(clock));
        Transform {
            translation: self.jaw.pivot - turn * self.jaw.pivot,
            rotation: turn,
            scale: Vector3::repeat(1.0),
        }
    }
}

/// How open a gulp is, 0 to 1, `through` it from 0 to 1: it opens, hangs
/// open a moment, and closes.
fn gulp_shape(through: f32) -> f32 {
    let ease = |t: f32| {
        let t = t.clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    };
    if through < OPENED {
        ease(through / OPENED)
    } else if through < CLOSING {
        1.0
    } else {
        1.0 - ease((through - CLOSING) / (1.0 - CLOSING))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const JAW: GulpingJaw = GulpingJaw {
        part: 2,
        pivot: Vector3::new(0.0, 0.0, 1.0),
        axis: Vector3::new(1.0, 0.0, 0.0),
        wide: 0.4,
        breath: 0.05,
        breath_rate: 0.5,
        gulp_for: 1.5,
        between: (3.0, 8.0),
        again: 0.3,
        seed: 9,
    };

    #[test]
    fn a_gulp_opens_wide_drops_the_chin_and_shuts_again() {
        let mut mouth = Mouth::new(JAW);
        // On a breath's trough, so only the gulp opens it.
        let clock = 4.0;
        mouth.gulp(clock);
        assert!(mouth.gape(clock).abs() < 1e-6);
        let widest = clock + JAW.gulp_for * 0.42;
        assert!((mouth.gape(widest) - JAW.wide).abs() < 1e-6);
        let chin = Vector3::new(0.0, 0.0, 1.3);
        let posed = mouth.pose(widest);
        let dropped = posed.rotation * chin + posed.translation;
        assert!(dropped.y < -0.1, "the chin went to {dropped:?}");
        // The hinge stays where it is.
        let hinge = posed.rotation * JAW.pivot + posed.translation;
        assert!((hinge - JAW.pivot).norm() < 1e-6);
        mouth.advance(clock + JAW.gulp_for);
        assert!(mouth.gape(clock + JAW.gulp_for) < JAW.breath + 1e-6);
    }

    #[test]
    fn between_gulps_it_only_breathes() {
        let mut mouth = Mouth::new(JAW);
        let (mut clock, mut starts) = (0.0, Vec::new());
        while clock < 120.0 {
            let was = mouth.started;
            mouth.advance(clock);
            if was.is_none() && mouth.started.is_some() {
                starts.push(clock);
            }
            if mouth.started.is_none() {
                assert!(mouth.gape(clock) <= JAW.breath + 1e-6);
            }
            clock += 1.0 / 60.0;
        }
        assert!(starts.len() > 8, "{} gulps in two minutes", starts.len());
        for pair in starts.windows(2) {
            let wait = pair[1] - pair[0] - JAW.gulp_for;
            let again = wait < AGAIN_AFTER + 0.05;
            let between = (JAW.between.0 - 0.05..JAW.between.1 + 0.05).contains(&wait);
            assert!(again || between, "it waited {wait} s");
        }
    }
}
