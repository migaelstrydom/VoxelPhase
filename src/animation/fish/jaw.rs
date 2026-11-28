//! A fish's mouth: a jaw (`model::Jaw`) that works a little with every
//! breath and now and then drops wide in a slow gulp, as a carp's does at
//! the surface.
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
use std::sync::Arc;

use super::rig::hashed;
use crate::model::{Jaw, MeshPrimitive};

/// Share of a gulp spent opening, and where it starts to close.
const OPENED: f32 = 0.35;
const CLOSING: f32 = 0.5;
/// The pause before a gulp that follows straight on from another, s.
const AGAIN_AFTER: f32 = 0.15;

/// How a mouth works: a little with every breath, and now and then wide.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gulps {
    /// How far its jaw drops in a gulp, radians.
    pub wide: f32,
    /// How far its jaw drops with each breath, radians.
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

/// A jaw, how it gulps, and where it is through its gulps.
#[derive(Clone)]
pub struct Mouth {
    /// The flesh that opens.
    pub jaw: Arc<Jaw>,
    /// How it gulps.
    pub gulps: Gulps,
    /// When the gulp under way began, on the rig's clock.
    started: Option<f32>,
    /// When it next gulps, on the rig's clock.
    next: f32,
    /// How many gulps it has made.
    made: u32,
}

impl Mouth {
    /// `jaw` as sculpted, its first gulp a wait away.
    pub fn new(jaw: Arc<Jaw>, gulps: Gulps) -> Self {
        let (shortest, longest) = gulps.between;
        Self {
            jaw,
            gulps,
            started: None,
            next: shortest + (longest - shortest) * hashed(gulps.seed, 0),
            made: 0,
        }
    }

    /// Carry it on to `clock`: begin the gulp that is due, and finish the
    /// one that is over.
    pub fn advance(&mut self, clock: f32) {
        let gulps = self.gulps;
        if let Some(started) = self.started {
            if clock - started < gulps.gulp_for {
                return;
            }
            self.started = None;
            self.made += 1;
            let roll = |salt: u32| hashed(gulps.seed.wrapping_add(self.made * 7919), salt);
            let (shortest, longest) = gulps.between;
            self.next = clock
                + if roll(1) < gulps.again {
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

    /// How far its jaw is open at `clock` past where it was sculpted,
    /// radians.
    pub fn gape(&self, clock: f32) -> f32 {
        let gulps = &self.gulps;
        let breath = gulps.breath * 0.5 * (1.0 - (TAU * gulps.breath_rate * clock).cos());
        let gulp = self.started.map_or(0.0, |started| {
            let through = ((clock - started) / gulps.gulp_for).clamp(0.0, 1.0);
            gulps.wide * gulp_shape(through)
        });
        breath.max(gulp)
    }

    /// Shape `out`, one mesh to each of the jaw's, as it is at `clock`.
    pub fn shape(&self, clock: f32, out: &mut [MeshPrimitive]) {
        self.jaw.shape(self.gape(clock), out);
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
    use nalgebra::{Vector2, Vector3, Vector4};

    use super::*;
    use crate::rendering::material::MaterialId;
    use crate::rendering::vertex::Vertex;

    const GULPS: Gulps = Gulps {
        wide: 0.4,
        breath: 0.05,
        breath_rate: 0.5,
        gulp_for: 1.5,
        between: (3.0, 8.0),
        again: 0.3,
        seed: 9,
    };

    /// A chin 0.3 m ahead of a hinge across X, all of it the jaw's.
    fn mouth() -> Mouth {
        let chin = Vertex {
            pos: Vector3::new(0.0, 0.0, 1.3),
            color: Vector4::repeat(1.0),
            tex_coords: Vector2::zeros(),
            normal: Vector3::z(),
            ao: 1.0,
        };
        let mesh = MeshPrimitive {
            vertices: vec![chin],
            indices: Vec::new(),
            material: MaterialId(0),
        };
        let jaw = Jaw::new(
            vec![mesh],
            vec![vec![1.0]],
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::x(),
        );
        Mouth::new(Arc::new(jaw), GULPS)
    }

    #[test]
    fn a_gulp_opens_wide_drops_the_chin_and_shuts_again() {
        let mut mouth = mouth();
        // On a breath's trough, so only the gulp opens it.
        let clock = 4.0;
        mouth.gulp(clock);
        assert!(mouth.gape(clock).abs() < 1e-6);
        let widest = clock + GULPS.gulp_for * 0.42;
        assert!((mouth.gape(widest) - GULPS.wide).abs() < 1e-6);
        let mut shaped = mouth.jaw.rest().to_vec();
        mouth.shape(widest, &mut shaped);
        let chin = shaped[0].vertices[0].pos;
        assert!(chin.y < -0.1, "the chin went to {chin:?}");
        mouth.advance(clock + GULPS.gulp_for);
        assert!(mouth.gape(clock + GULPS.gulp_for) < GULPS.breath + 1e-6);
    }

    #[test]
    fn between_gulps_it_only_breathes() {
        let mut mouth = mouth();
        let (mut clock, mut starts) = (0.0, Vec::new());
        while clock < 120.0 {
            let was = mouth.started;
            mouth.advance(clock);
            if was.is_none() && mouth.started.is_some() {
                starts.push(clock);
            }
            if mouth.started.is_none() {
                assert!(mouth.gape(clock) <= GULPS.breath + 1e-6);
            }
            clock += 1.0 / 60.0;
        }
        assert!(starts.len() > 8, "{} gulps in two minutes", starts.len());
        for pair in starts.windows(2) {
            let wait = pair[1] - pair[0] - GULPS.gulp_for;
            let again = wait < AGAIN_AFTER + 0.05;
            let between = (GULPS.between.0 - 0.05..GULPS.between.1 + 0.05).contains(&wait);
            assert!(again || between, "it waited {wait} s");
        }
    }
}
