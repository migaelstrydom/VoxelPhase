use nalgebra::{Matrix4, Vector3};

use crate::rendering::geometry_draw::GeometryDraw;
use crate::rendering::reflection::atlas::PROBE_RESOLUTION;
use crate::rendering::reflection::face::CubeFace;
use crate::rendering::reflection::owner::ProbeOwner;
use crate::rendering::transparency::MeshBounds;

/// The smallest a sphere may look from a probe and still be captured: its
/// radius over its distance, as a tangent. Half a texel at the middle of a
/// face, where texels are largest; anything smaller would cover no texel
/// centre, and costs a draw to write nothing.
const MIN_APPARENT_SIZE: f32 = 1.0 / PROBE_RESOLUTION as f32;

/// A sphere enclosing a draw's geometry, in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingSphere {
    pub centre: Vector3<f32>,
    pub radius: f32,
}

impl BoundingSphere {
    /// The sphere around a model-space box, carried into world space by
    /// `model`.
    ///
    /// The radius is scaled by the model's largest axis scale, so a
    /// non-uniformly scaled mesh gets a sphere that is loose but never too
    /// small.
    pub fn around(bounds: &MeshBounds, model: &Matrix4<f32>) -> Self {
        let centre = model.transform_point(&bounds.centre().into()).coords;
        let scale = (0..3)
            .map(|axis| model.fixed_view::<3, 1>(0, axis).norm())
            .fold(0.0_f32, f32::max);
        Self {
            centre,
            radius: bounds.half_diagonal() * scale,
        }
    }

    /// Whether any of the sphere lies within `distance` of `point`.
    pub fn within(&self, point: &Vector3<f32>, distance: f32) -> bool {
        (self.centre - point).norm() - self.radius <= distance
    }

    /// Whether the sphere looks big enough from `point` to cover a probe
    /// texel.
    pub fn visible_from(&self, point: &Vector3<f32>) -> bool {
        self.radius >= (self.centre - point).norm() * MIN_APPARENT_SIZE
    }
}

/// How far a draw's geometry extends, as far as a probe deciding whether it
/// can see it needs to know.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reach {
    /// Everywhere a probe might be: the terrain, which surrounds every probe
    /// and which no sphere describes usefully.
    Everywhere,
    /// Within a sphere.
    Within(BoundingSphere),
}

/// One opaque draw, as the probes see it: the draw itself, where its geometry
/// is, and whose it is.
#[derive(Clone, Copy, Debug)]
pub struct ProbeCaster {
    /// The draw, already committed to the frame: recorded again into each
    /// probe face that sees it.
    pub geometry: GeometryDraw,
    /// Where its geometry lies.
    pub reach: Reach,
    /// The object it belongs to, which its own probe leaves out: a probe sits
    /// inside the object it serves, and would otherwise see nothing but it.
    pub owner: Option<ProbeOwner>,
}

impl ProbeCaster {
    /// Whether a probe at `centre`, serving `probe_owner`, sees this draw
    /// through `face`, given how far the probe looks.
    pub fn seen_by(
        &self,
        probe_owner: ProbeOwner,
        centre: &Vector3<f32>,
        face: CubeFace,
        reach: f32,
        near: f32,
    ) -> bool {
        if self.owner == Some(probe_owner) {
            return false;
        }
        match self.reach {
            Reach::Everywhere => true,
            Reach::Within(sphere) => {
                sphere.within(centre, reach)
                    && sphere.visible_from(centre)
                    && face.sees(centre, &sphere, near)
            }
        }
    }
}
