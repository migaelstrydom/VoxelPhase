//! A plant's woody frame: limbs, each a rod from a base to a tip.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::geometry::generate_tube;
use crate::physics::ColliderDesc;
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// Sides round a limb's drawn tube: round enough for a slender trunk.
const LIMB_SIDES: u32 = 12;

/// One limb: a rod from `base` to `tip`, in the plant's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limb {
    /// Where it grows from: the ground for a trunk, a fork for a branch.
    pub base: Point3<f32>,
    /// Where it ends, inside the leaf mass it holds up.
    pub tip: Point3<f32>,
    /// Its radius, m.
    pub radius: f32,
    /// The limb it grows from, by index, or `None` for the trunk.
    pub parent: Option<usize>,
}

impl Limb {
    /// Its midpoint, where its body's origin is.
    pub fn midpoint(&self) -> Point3<f32> {
        self.base + (self.tip - self.base) * 0.5
    }

    /// A capsule collider along it, centred on its midpoint.
    pub fn collider(&self) -> ColliderDesc {
        let along = self.tip - self.base;
        let turn = UnitQuaternion::rotation_between(&Vector3::y(), &along)
            .unwrap_or_else(UnitQuaternion::identity);
        ColliderDesc::capsule(along.magnitude() * 0.5 + self.radius, self.radius)
            .offset_rotation(turn)
    }

    /// It drawn as a tube in `colour`, centred on its midpoint. Its ends are
    /// open: a trunk's foot is in the ground and every tip is inside leaves.
    pub fn mesh(&self, colour: Colour) -> (Vec<Vertex>, Vec<u32>) {
        let middle = self.midpoint().coords;
        let ends = [self.base - middle, self.tip - middle];
        generate_tube(&ends, self.radius, LIMB_SIDES, false, colour)
    }
}

/// Every limb of a plant: a trunk and any branches off it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Limbs {
    /// The trunk first, then branches, each after the limb it grows from.
    pub limbs: Vec<Limb>,
}
