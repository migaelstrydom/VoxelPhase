//! A plant's woody frame: limbs, each a rod from a base to a tip.

use nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};

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
}

impl Limb {
    /// A capsule collider along it, in the plant's frame.
    pub fn collider(&self) -> ColliderDesc {
        let along = self.tip - self.base;
        let midpoint = self.base + along * 0.5;
        let turn = UnitQuaternion::rotation_between(&Vector3::y(), &along)
            .unwrap_or_else(UnitQuaternion::identity);
        ColliderDesc::capsule(along.magnitude() * 0.5 + self.radius, self.radius)
            .offset_translation(Translation3::from(midpoint).vector)
            .offset_rotation(turn)
    }
}

/// Every limb of a plant: a trunk and any branches off it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Limbs {
    /// The trunk first, then branches.
    pub limbs: Vec<Limb>,
}

impl Limbs {
    /// A capsule collider per limb, in the plant's frame.
    pub fn colliders(&self) -> Vec<ColliderDesc> {
        self.limbs.iter().map(Limb::collider).collect()
    }

    /// The limbs drawn as tubes in `colour`, in the plant's frame. Their ends
    /// are open: a trunk's foot is in the ground and every tip is inside
    /// leaves.
    pub fn mesh(&self, colour: Colour) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        for limb in &self.limbs {
            let (v, i) = generate_tube(
                &[limb.base, limb.tip],
                limb.radius,
                LIMB_SIDES,
                false,
                colour,
            );
            let base = vertices.len() as u32;
            vertices.extend(v);
            indices.extend(i.into_iter().map(|index| base + index));
        }
        (vertices, indices)
    }
}
