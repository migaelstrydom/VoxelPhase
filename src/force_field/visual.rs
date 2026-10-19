//! The component that gives a body's suspensions a visible field.

use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage};

use crate::particles::ParticlePool;
use crate::physics::ConstraintHandle;
use crate::rendering::colour::Colour;

use super::kick_up::KickUp;
use super::pod::{PodField, PodLook, PodSample};
use super::style::FieldStyle;

/// How far above the ground the underglow sits, m: close enough to light
/// the ground it rides over, far enough not to burn a spot into it.
const UNDERGLOW_HEIGHT: f32 = 0.15;

/// A force field under some of a body's suspensions: one column, ring and
/// spray per suspension, all read from its readout every frame.
///
/// Any suspension can wear one. It knows nothing of what the suspensions
/// hold up; `ForceFieldSystem` reads them and this turns what it read into
/// looks, which the renderer draws.
#[derive(Component)]
#[storage(DenseVecStorage)]
pub struct ForceFieldVisual {
    /// How the field looks.
    pub style: FieldStyle,
    /// The suspensions shown, each with its field.
    pods: Vec<(ConstraintHandle, PodField)>,
    /// This frame's look of each pod that still has a suspension.
    looks: Vec<PodLook>,
    /// What the field throws up off the ground.
    kick_up: KickUp,
}

/// The light a field casts on the ground under it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Underglow {
    pub position: Point3<f32>,
    pub colour: Colour,
    /// As luminance.
    pub intensity: f32,
}

impl ForceFieldVisual {
    /// A field under each of `suspensions`.
    pub fn new(style: FieldStyle, suspensions: impl IntoIterator<Item = ConstraintHandle>) -> Self {
        let pods: Vec<_> = suspensions
            .into_iter()
            .enumerate()
            .map(|(i, handle)| (handle, PodField::new(i as u32 + 1)))
            .collect();
        Self {
            style,
            looks: Vec::with_capacity(pods.len()),
            pods,
            kick_up: KickUp::default(),
        }
    }

    /// The suspensions shown, in order.
    pub fn suspensions(&self) -> impl Iterator<Item = ConstraintHandle> + '_ {
        self.pods.iter().map(|(handle, _)| *handle)
    }

    /// Follow this frame's samples, one per suspension in order, `dt`
    /// seconds after the last. A suspension that is gone has no sample and
    /// no look.
    pub fn update(&mut self, samples: &[Option<PodSample>], dt: f32) {
        self.looks.clear();
        for ((_, field), sample) in self.pods.iter_mut().zip(samples) {
            if let Some(sample) = sample {
                field.update(sample, &self.style, dt);
                self.looks.push(field.look(&self.style));
            }
        }
    }

    /// This frame's looks.
    pub fn looks(&self) -> &[PodLook] {
        &self.looks
    }

    /// Spend `dt`'s kick-up into `pool`.
    pub fn kick_up(&mut self, dt: f32, pool: &mut ParticlePool) {
        self.kick_up.emit(&self.looks, dt, pool);
    }

    /// The light under the field. See [`Underglow::of`].
    pub fn underglow(&self) -> Option<Underglow> {
        Underglow::of(&self.looks, &self.style, self.pods.len())
    }
}

impl Underglow {
    /// The light under a field of `pods` pods with these `looks`: over the
    /// middle of its ground rings, in their mean colour, as bright as their
    /// mean heat and the share of pods over ground. `None` with no pod over
    /// any ground.
    pub fn of(looks: &[PodLook], style: &FieldStyle, pods: usize) -> Option<Self> {
        let grounded: Vec<_> = looks
            .iter()
            .filter_map(|look| look.ground_ring.map(|ring| (look, ring)))
            .collect();
        if grounded.is_empty() {
            return None;
        }
        let count = grounded.len() as f32;
        let centre = grounded
            .iter()
            .fold(Vector3::zeros(), |sum, (_, ring)| sum + ring.centre.coords)
            / count;
        let up = grounded
            .iter()
            .fold(Vector3::zeros(), |sum, (_, ring)| sum + ring.normal)
            .normalize();
        let colour = grounded
            .iter()
            .fold(Vector3::zeros(), |sum, (look, _)| sum + look.colour)
            / count;
        let heat = grounded.iter().map(|(look, _)| look.heat).sum::<f32>() / count;
        let share = count / pods.max(1) as f32;
        Some(Self {
            position: Point3::from(centre + up * UNDERGLOW_HEIGHT),
            colour: Colour::rgb(colour.x, colour.y, colour.z),
            intensity: style.light_intensity.at(heat) * share,
        })
    }
}
