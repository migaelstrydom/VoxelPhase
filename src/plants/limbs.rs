//! A plant's woody frame: limbs, each a rod from a base to a tip.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use crate::physics::ColliderDesc;

/// A root collides as a capsule this share of its radius at the trunk: it
/// tapers to a point, so its mean radius is well under its widest.
const ROOT_COLLIDER_RADIUS: f32 = 0.6;

/// What a limb is to the plant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimbKind {
    /// Grows from the ground; everything else grows from it.
    Trunk,
    /// Grows from another limb above the ground and holds up leaves: a blast
    /// can break it off.
    Branch,
    /// Grows down and out from the trunk's foot, buried while the tree
    /// stands.
    Root,
}

/// One limb: a rod from `base` to `tip`, in the plant's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limb {
    /// Where it grows from: below the ground for a trunk, a fork for a
    /// branch, the trunk's foot for a root.
    pub base: Point3<f32>,
    /// Where it ends: inside the leaf mass it holds up, or in the soil.
    pub tip: Point3<f32>,
    /// Its radius at its base, m.
    pub radius: f32,
    /// The limb it grows from, by index, or `None` for the trunk.
    pub parent: Option<usize>,
    /// What it is.
    pub kind: LimbKind,
}

impl Limb {
    /// Its midpoint, where its body's origin is.
    pub fn midpoint(&self) -> Point3<f32> {
        self.base + (self.tip - self.base) * 0.5
    }

    /// The radius it collides with, m.
    pub fn collider_radius(&self) -> f32 {
        match self.kind {
            LimbKind::Root => self.radius * ROOT_COLLIDER_RADIUS,
            LimbKind::Trunk | LimbKind::Branch => self.radius,
        }
    }

    /// Where its collider's frame is in the plant's: at its midpoint, with
    /// y along it.
    pub fn pose(&self) -> Isometry3<f32> {
        let turn = UnitQuaternion::rotation_between(&Vector3::y(), &(self.tip - self.base))
            .unwrap_or_else(UnitQuaternion::identity);
        Isometry3::from_parts(Translation3::from(self.midpoint().coords), turn)
    }

    /// A capsule collider along it, placed at [`Limb::pose`] in the plant's
    /// frame.
    pub fn collider(&self) -> ColliderDesc {
        let radius = self.collider_radius();
        let mut collider =
            ColliderDesc::capsule((self.tip - self.base).magnitude() * 0.5 + radius, radius);
        collider.offset = self.pose();
        collider
    }
}

/// Every limb of a plant: a trunk and any branches and roots off it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Limbs {
    /// The trunk first, then branches and roots, each after the limb it
    /// grows from.
    pub limbs: Vec<Limb>,
}

impl Limbs {
    /// Whether limb `l`, in a piece holding `held`, is a branch broken off
    /// the limb it grew from.
    pub fn broken_off(&self, l: usize, held: &[usize]) -> bool {
        let limb = &self.limbs[l];
        limb.kind == LimbKind::Branch && limb.parent.is_some_and(|p| !held.contains(&p))
    }

    /// `limb` and every limb growing from it, however indirectly, in limb
    /// order.
    pub fn subtree(&self, limb: usize) -> Vec<usize> {
        let mut within = vec![false; self.limbs.len()];
        within[limb] = true;
        // Each limb comes after its parent, so one pass finds them all.
        for (index, l) in self.limbs.iter().enumerate().skip(limb + 1) {
            if l.parent.is_some_and(|p| within[p]) {
                within[index] = true;
            }
        }
        (0..self.limbs.len()).filter(|&i| within[i]).collect()
    }
}
