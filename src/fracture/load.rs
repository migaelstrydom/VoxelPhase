//! What each joint of a compound is being asked to hold, this frame.
//!
//! ```text
//!   blast impulse ─┐
//!                  ├─▶ ChildLoad (per collider) ─▶ ChildLoads::breaks(joint)
//!   contact spike ─┘        + where it landed
//! ```
//!
//! Two loads arrive by different routes and are *not* interchangeable numbers,
//! which is why they are kept apart all the way to the threshold test:
//!
//! - A **blast** is an analytic impulse with its own distance falloff, so what
//!   a child receives is already local to the explosion. A grenade delivers on
//!   the order of a thousand N·s at its centre.
//! - A **contact spike** is momentum the solver actually pushed through a
//!   child. For a heavy compound it is far larger — several tonnes of bridge
//!   settling puts more through one beam than any grenade — because it is the
//!   whole body's momentum funnelling through a few contacts.
//!
//! A single threshold cannot serve both: the plank bridge is authored to come
//! apart under an 8 N·s blast, and its own weight moves 1700 N·s through a beam
//! when it settles. So blast is tested against the joint's `threshold` and
//! contact against the compound's `contact_threshold`.
//!
//! ## Locality
//!
//! A contact spike is also attributed to *where it landed*. A child may be long
//! and carry many joints — the bridge's beams run its whole length and every
//! plank hangs off them — and an impact at one end does not load a joint at the
//! other. Without this, one knock on a beam breaks all sixteen of its joints at
//! once and the bridge does not shed planks, it ceases to exist.

use nalgebra::Point3;

use super::components::FractureJoint;

/// How hard one child of a compound is being pushed, and where.
#[derive(Debug, Clone)]
pub struct ChildLoad {
    /// Impulse from explicit sources (explosions), already distance-attenuated.
    pub blast: f32,
    /// Contact impulse spike this frame, before any locality attenuation.
    pub contact: f32,
    /// Where this child's contact impulse landed, in world space.
    pub contact_point: Point3<f32>,
    /// Centre of the child in world space.
    pub centre: Point3<f32>,
    /// Bounding radius of the child, the length scale of its own structure.
    pub radius: f32,
}

impl Default for ChildLoad {
    fn default() -> Self {
        Self {
            blast: 0.0,
            contact: 0.0,
            contact_point: Point3::origin(),
            centre: Point3::origin(),
            radius: 0.0,
        }
    }
}

/// The loads on every child of one compound body, ready to judge its joints.
#[derive(Debug, Default)]
pub struct ChildLoads {
    children: Vec<ChildLoad>,
    /// Contact spike, at the joint, above which a joint lets go. `None` falls
    /// back to the joint's own blast threshold.
    contact_threshold: Option<f32>,
}

impl ChildLoads {
    pub fn new(children: Vec<ChildLoad>, contact_threshold: Option<f32>) -> Self {
        Self {
            children,
            contact_threshold,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.children.is_empty()
    }

    /// Whether this joint is being asked to hold more than it can.
    ///
    /// Either endpoint can break it, and either load can do the breaking.
    pub fn breaks(&self, joint: &FractureJoint) -> bool {
        let (Some(a), Some(b)) = (
            self.children.get(joint.child_a),
            self.children.get(joint.child_b),
        ) else {
            return false;
        };

        if a.blast.max(b.blast) > joint.threshold {
            return true;
        }

        // A joint never lets go below its own number under any load: the
        // compound's contact threshold is authored for its brittle joints,
        // and a frame welded together at a million N·s must not fall apart
        // because the pane in it cracks at twenty-five.
        let contact_threshold = self
            .contact_threshold
            .map_or(joint.threshold, |authored| authored.max(joint.threshold));
        self.contact_load(joint) > contact_threshold
    }

    /// The contact spike this joint actually feels, after locality: the larger
    /// of what its two endpoints deliver to where the joint sits.
    ///
    /// This is the quantity a compound's `contact_threshold` is measured
    /// against, so the tests that choose that number read it directly.
    pub fn contact_load(&self, joint: &FractureJoint) -> f32 {
        let (Some(a), Some(b)) = (
            self.children.get(joint.child_a),
            self.children.get(joint.child_b),
        ) else {
            return 0.0;
        };
        let anchor = joint_anchor(a, b);
        self.contact_at(a, &anchor).max(self.contact_at(b, &anchor))
    }

    /// A child's contact spike as felt at `anchor`, falling off with distance
    /// from where the impulse actually landed.
    fn contact_at(&self, child: &ChildLoad, anchor: &JointAnchor) -> f32 {
        if child.contact <= 0.0 || anchor.scale <= 0.0 {
            return child.contact;
        }
        let ratio = (anchor.position - child.contact_point).magnitude() / anchor.scale;
        child.contact / (1.0 + ratio * ratio)
    }
}

/// Where a joint sits, and over what distance a hit near it stops being near.
struct JointAnchor {
    position: Point3<f32>,
    scale: f32,
}

/// A joint lives on the smaller of the two children it connects, and the length
/// scale that matters is that child's own size.
///
/// The smaller one is the local structural element: a plank bolted to a beam is
/// a joint at the plank, not at the middle of an eight-metre beam.
fn joint_anchor(a: &ChildLoad, b: &ChildLoad) -> JointAnchor {
    let local = if a.radius <= b.radius { a } else { b };
    JointAnchor {
        position: local.centre,
        scale: local.radius * 2.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn child(contact: f32, centre: f32, radius: f32, contact_point: f32) -> ChildLoad {
        ChildLoad {
            blast: 0.0,
            contact,
            contact_point: Point3::new(0.0, 0.0, contact_point),
            centre: Point3::new(0.0, 0.0, centre),
            radius,
        }
    }

    fn joint(threshold: f32) -> FractureJoint {
        FractureJoint {
            child_a: 0,
            child_b: 1,
            threshold,
        }
    }

    /// The bridge's shape: one long beam, many small planks along it, every
    /// joint sharing the beam. A hit at one end must not reach the other.
    #[test]
    fn a_hit_on_a_long_child_does_not_reach_its_distant_joints() {
        let beam = child(10_000.0, 0.0, 4.0, -3.5);
        let near_plank = child(0.0, -3.5, 1.0, 0.0);
        let far_plank = child(0.0, 3.5, 1.0, 0.0);

        let near = ChildLoads::new(vec![beam.clone(), near_plank], Some(1_000.0));
        let far = ChildLoads::new(vec![beam, far_plank], Some(1_000.0));

        assert!(near.breaks(&joint(8.0)), "the struck end should let go");
        assert!(!far.breaks(&joint(8.0)), "the far end should hold");
    }

    /// Blast keeps its own threshold, which for a rickety structure is far
    /// below anything contact produces.
    #[test]
    fn blast_and_contact_are_judged_on_their_own_scales() {
        let mut a = child(0.0, 0.0, 1.0, 0.0);
        a.blast = 20.0;
        let b = child(0.0, 0.0, 1.0, 0.0);
        let loads = ChildLoads::new(vec![a, b], Some(1_000.0));

        // 20 N·s of blast clears an 8 N·s joint even though the contact
        // threshold it would never reach is a hundred times larger.
        assert!(loads.breaks(&joint(8.0)));
    }

    /// With no contact threshold authored, contact is judged on the joint's
    /// own number — the behaviour of a compound that has not been measured.
    #[test]
    fn an_unauthored_contact_threshold_falls_back_to_the_joint() {
        let a = child(50.0, 0.0, 1.0, 0.0);
        let b = child(0.0, 0.0, 1.0, 0.0);
        let loads = ChildLoads::new(vec![a, b], None);

        assert!(loads.breaks(&joint(8.0)));
        assert!(!loads.breaks(&joint(500.0)));
    }

    #[test]
    fn a_joint_naming_a_child_that_is_gone_does_not_break() {
        let loads = ChildLoads::new(vec![child(1e9, 0.0, 1.0, 0.0)], Some(1.0));
        assert!(!loads.breaks(&joint(1.0)));
    }

    /// The anchor is the smaller child regardless of which side it is on.
    #[test]
    fn the_joint_sits_on_the_smaller_child() {
        let big = child(0.0, 0.0, 4.0, 0.0);
        let small = child(0.0, 3.0, 0.5, 0.0);
        let anchor = joint_anchor(&big, &small);
        assert_eq!(anchor.position, Point3::new(0.0, 0.0, 3.0));
        assert_eq!(anchor.scale, 1.0);
        let swapped = joint_anchor(&small, &big);
        assert_eq!(swapped.position, anchor.position);
    }
}
