//! What each bond of a compound is being asked to hold, this frame, judged
//! where the bond is.
//!
//! ```text
//!   blasts ────────────▶ loading per area at the anchor ─┐
//!                                                         ├─▶ BondLoads::breaks
//!   contact spikes ────▶ largest on either part, felt at ─┘
//!   (per collider)        the anchor over the bond's reach
//! ```
//!
//! Two loads arrive by different routes and are *not* interchangeable numbers,
//! which is why they are kept apart all the way to the threshold test:
//!
//! - A **blast** is an analytic loading with its own distance falloff, read
//!   at the anchor, so a bond feels what reaches the place it holds.
//! - A **contact spike** is momentum the solver actually pushed through a
//!   collider. For a heavy compound it is far larger — several tonnes of
//!   bridge settling puts more through one beam than any grenade — because
//!   it is the whole body's momentum funnelling through a few contacts.
//!
//! A single threshold cannot serve both: the plank bridge is authored to come
//! apart under an 8 N·s/m² blast, and its own weight moves 1700 N·s through a
//! beam when it settles. So blast is tested against the bond's `threshold`
//! and contact against the compound's `contact_threshold`.
//!
//! ## Locality
//!
//! A contact spike is felt at the bond as it falls off with distance from
//! where it landed, over the bond's `reach`. A part may be long and carry
//! many bonds — the bridge's beams run its whole length and every plank
//! hangs off them — and an impact at one end does not load a bond at the
//! other. Without this, one knock on a beam breaks all sixteen of its bonds
//! at once and the bridge does not shed planks, it ceases to exist.

use nalgebra::{Isometry3, Point3};
use rustc_hash::FxHashMap;

use super::contact_load::ContactSpike;
use super::parts::{Bond, PartGraph};
use crate::physics::{ColliderHandle, PhysicsImpulse};

/// The loads on one compound's bonds this frame.
pub struct BondLoads<'a> {
    /// This frame's contact spike on each collider.
    spikes: FxHashMap<ColliderHandle, ContactSpike>,
    /// This frame's blasts.
    blasts: &'a [PhysicsImpulse],
    /// The body's pose, which places bond anchors in the world.
    pose: Isometry3<f32>,
    /// Contact spike, at the bond, above which a bond lets go. `None` falls
    /// back to the bond's own blast threshold.
    contact_threshold: Option<f32>,
}

impl<'a> BondLoads<'a> {
    pub fn new(
        spikes: impl IntoIterator<Item = (ColliderHandle, ContactSpike)>,
        blasts: &'a [PhysicsImpulse],
        pose: Isometry3<f32>,
        contact_threshold: Option<f32>,
    ) -> Self {
        Self {
            spikes: spikes.into_iter().collect(),
            blasts,
            pose,
            contact_threshold,
        }
    }

    /// Whether `bond` is being asked to hold more than it can.
    pub fn breaks(&self, graph: &PartGraph, bond: &Bond) -> bool {
        if self.blast_load(bond) > bond.threshold {
            return true;
        }
        // A bond never lets go below its own number under any load: the
        // compound's contact threshold is authored for its brittle bonds,
        // and a frame welded together at a million must not fall apart
        // because the pane in it cracks at twenty-five.
        let contact_threshold = self
            .contact_threshold
            .map_or(bond.threshold, |authored| authored.max(bond.threshold));
        self.contact_load(graph, bond) > contact_threshold
    }

    /// The blasts' loading per area at the bond's anchor, N·s/m².
    pub fn blast_load(&self, bond: &Bond) -> f32 {
        let at = self.pose * bond.anchor;
        self.blasts
            .iter()
            .filter_map(|blast| blast.loading_at(at))
            .map(|loading| loading.per_area.magnitude())
            .sum()
    }

    /// The contact spike this bond feels, N·s: the largest any collider of
    /// either part took, falling off with distance from where it landed to
    /// the anchor over the bond's reach.
    ///
    /// The quantity a compound's `contact_threshold` is measured against, so
    /// the tests that choose that number read it directly.
    pub fn contact_load(&self, graph: &PartGraph, bond: &Bond) -> f32 {
        let anchor = self.pose * bond.anchor;
        [bond.a, bond.b]
            .iter()
            .filter_map(|id| graph.part(*id))
            .flat_map(|part| part.colliders.iter())
            .filter_map(|collider| self.spikes.get(collider))
            .map(|spike| felt_at(spike, anchor, bond.reach))
            .fold(0.0, f32::max)
    }
}

/// A spike as felt at `anchor`, falling off with distance from where it
/// landed over `reach`.
fn felt_at(spike: &ContactSpike, anchor: Point3<f32>, reach: f32) -> f32 {
    if spike.magnitude <= 0.0 || reach <= 0.0 {
        return spike.magnitude;
    }
    let ratio = (anchor - spike.point).magnitude() / reach;
    spike.magnitude / (1.0 + ratio * ratio)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fracture::parts::PartId;

    fn spike(magnitude: f32, z: f32) -> ContactSpike {
        ContactSpike {
            magnitude,
            point: Point3::new(0.0, 0.0, z),
        }
    }

    /// The bridge's shape: one long beam, two planks at its ends, each bond
    /// where a plank crosses the beam.
    fn beam_and_planks() -> (PartGraph, Bond, Bond) {
        let mut graph = PartGraph::default();
        let beam = graph.add_part([ColliderHandle::for_test(0)]);
        let near = graph.add_part([ColliderHandle::for_test(1)]);
        let far = graph.add_part([ColliderHandle::for_test(2)]);
        let bond = |plank: PartId, z: f32| Bond {
            a: beam,
            b: plank,
            anchor: Point3::new(0.0, 0.0, z),
            reach: 2.0,
            threshold: 8.0,
        };
        let (near_bond, far_bond) = (bond(near, -3.5), bond(far, 3.5));
        graph.bond(near_bond);
        graph.bond(far_bond);
        (graph, near_bond, far_bond)
    }

    /// A hit at one end of a long part does not reach its distant bonds.
    #[test]
    fn a_hit_on_a_long_part_does_not_reach_its_distant_bonds() {
        let (graph, near, far) = beam_and_planks();
        let loads = BondLoads::new(
            [(ColliderHandle::for_test(0), spike(10_000.0, -3.5))],
            &[],
            Isometry3::identity(),
            Some(1_000.0),
        );
        assert!(loads.breaks(&graph, &near), "the struck end should let go");
        assert!(!loads.breaks(&graph, &far), "the far end should hold");
    }

    /// A blast is read where the bond is: one whose reach ends short of a
    /// bond does not break it, however hard it strikes the parts' far ends.
    #[test]
    fn a_blast_is_judged_at_the_anchor() {
        let (graph, near, far) = beam_and_planks();
        let blasts = [PhysicsImpulse::radial(
            Point3::new(0.0, 0.0, -4.0),
            2.0,
            100.0,
            0.0,
        )];
        let loads = BondLoads::new([], &blasts, Isometry3::identity(), Some(1_000.0));
        assert!(loads.blast_load(&near) > near.threshold);
        assert!(loads.breaks(&graph, &near));
        assert_eq!(loads.blast_load(&far), 0.0);
        assert!(!loads.breaks(&graph, &far));
    }

    /// Anchors are in the body's frame, so they are judged where the body
    /// has taken them.
    #[test]
    fn anchors_are_placed_by_the_body() {
        let (graph, near, _) = beam_and_planks();
        let blasts = [PhysicsImpulse::radial(
            Point3::new(10.0, 0.0, -3.5),
            1.0,
            100.0,
            0.0,
        )];
        let moved = Isometry3::translation(10.0, 0.0, 0.0);
        let loads = BondLoads::new([], &blasts, moved, None);
        assert!(loads.breaks(&graph, &near));
    }

    /// With no contact threshold authored, contact is judged on the bond's
    /// own number — the behaviour of a compound that has not been measured.
    #[test]
    fn an_unauthored_contact_threshold_falls_back_to_the_bond() {
        let (graph, near, _) = beam_and_planks();
        let loads = BondLoads::new(
            [(ColliderHandle::for_test(1), spike(50.0, -3.5))],
            &[],
            Isometry3::identity(),
            None,
        );
        assert!(loads.breaks(&graph, &near));
        let strong = Bond {
            threshold: 500.0,
            ..near
        };
        assert!(!loads.breaks(&graph, &strong));
    }
}
