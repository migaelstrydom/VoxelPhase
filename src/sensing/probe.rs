//! Probe definitions for terrain sensing.
//!
//! Probes are defined in world coordinates and have opaque tags
//! that the sensing system passes through without interpretation.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

/// A ray probe in world coordinates.
///
/// The sensing system executes probes without knowing their purpose.
/// The `tag` field is passed through to results for the caller to interpret.
#[derive(Debug, Clone)]
pub struct Probe {
    /// Opaque identifier - the sensing system doesn't interpret this.
    /// Used by the caller to match results back to their meaning.
    pub tag: u32,
    /// Probe origin in world space.
    pub origin: Point3<f32>,
    /// Probe direction in world space (should be normalized).
    pub direction: Vector3<f32>,
    /// Maximum probe distance.
    pub length: f32,
}

/// A contact candidate from a terrain probe.
///
/// The `tag` matches the probe that generated this result.
#[derive(Debug, Clone)]
pub struct ContactCandidate {
    /// Tag from the probe that generated this contact.
    pub tag: u32,
    /// Contact point in world space.
    pub point: Point3<f32>,
    /// Surface normal at contact point.
    pub normal: Vector3<f32>,
    /// Distance from probe origin to contact.
    pub distance: f32,
}

/// Collection of contact candidates from probes.
#[derive(Component, Debug, Default, Clone)]
#[storage(VecStorage)]
pub struct ContactCandidates {
    pub candidates: Vec<ContactCandidate>,
}

/// Set of probes to execute each frame.
///
/// This component is written by animation controllers and read by
/// the sensing system. Probes are in world coordinates.
#[derive(Component, Debug, Default, Clone)]
#[storage(VecStorage)]
pub struct SensorSet {
    pub probes: Vec<Probe>,
}

impl SensorSet {
    /// Create a new sensor set with the given probes.
    pub fn new(probes: Vec<Probe>) -> Self {
        Self { probes }
    }
}

/// The result of a probe sweep hitting a collidable surface.
#[derive(Debug, Clone)]
pub struct ProbeHit {
    /// Parametric time of contact along the probe, in [0, 1].
    /// 0 = at the origin, 1 = at full probe length.
    pub t: f32,
    /// World-space contact point on the surface.
    pub point: Point3<f32>,
    /// Surface normal at the contact point, pointing away from the surface.
    pub normal: Vector3<f32>,
}

/// Any collidable surface that can be queried by ray probes.
pub trait ProbeTarget {
    /// Cast a ray from `origin` along `direction` for up to `length` units,
    /// returning the earliest surface hit if any.
    fn raycast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<ProbeHit>;
}

/// Several probe targets queried as one: the earliest hit across all of them.
///
/// Nothing in the world is a single surface. Terrain and the rigid bodies are
/// separate stores with separate ray tests, and every caster — a foot probe, a
/// predicted trajectory — wants the first thing hit, whichever store it came
/// from. Being a [`ProbeTarget`] itself, a set composes with another set.
pub struct ProbeSet<'a> {
    targets: Vec<&'a dyn ProbeTarget>,
}

impl<'a> ProbeSet<'a> {
    /// Build a set from the targets that exist. `None` entries are dropped, so
    /// callers can hand over optional resources without pre-filtering.
    pub fn new(targets: impl IntoIterator<Item = Option<&'a dyn ProbeTarget>>) -> Self {
        Self {
            targets: targets.into_iter().flatten().collect(),
        }
    }

    /// True when no target was supplied, in which case every cast misses.
    pub fn is_empty(&self) -> bool {
        self.targets.is_empty()
    }
}

impl ProbeTarget for ProbeSet<'_> {
    fn raycast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<ProbeHit> {
        let mut earliest: Option<ProbeHit> = None;
        for target in &self.targets {
            if let Some(hit) = target.raycast(origin, direction, length) {
                if earliest.as_ref().map_or(true, |e| hit.t < e.t) {
                    earliest = Some(hit);
                }
            }
        }
        earliest
    }
}
