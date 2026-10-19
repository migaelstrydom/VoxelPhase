//! What a suspension's cast found, and the frame its rows are built in.

use nalgebra::{Point3, Vector3};

use crate::physics::handle::RigidBodyHandle;

/// What a suspension's cast hit, this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SuspensionHit {
    /// Where the cast sphere touched, in world space.
    pub point: Point3<f32>,
    /// Surface normal at the touch, out of the surface and toward the body.
    pub normal: Vector3<f32>,
    /// The body touched; `None` for static geometry and static bodies.
    pub partner: Option<RigidBodyHandle>,
    /// Friction coefficient where the foot meets the surface.
    pub friction: f32,
    /// The spring's axis in world space when the cast was made.
    pub axis: Vector3<f32>,
    /// The anchor in world space when the cast was made.
    pub anchor: Point3<f32>,
    /// Distance from the anchor to the cast sphere's centre at the touch.
    pub length: f32,
}

impl SuspensionHit {
    /// The cast sphere's centre at the touch: where the spring acts on the
    /// partner.
    pub fn sphere_centre(&self) -> Point3<f32> {
        self.anchor - self.axis * self.length
    }

    /// The share of the spring's impulse that presses into the surface:
    /// `axis·n`, never negative.
    pub fn load_factor(&self) -> f32 {
        self.axis.dot(&self.normal).max(0.0)
    }

    /// The hit's tangent basis, fixed against a world reference projected
    /// into its plane so the friction rows' warm impulses keep their meaning
    /// from frame to frame.
    pub fn tangents(&self) -> (Vector3<f32>, Vector3<f32>) {
        tangent_basis(&self.normal)
    }
}

/// A tangent basis for the plane with normal `normal`: world X projected into
/// it, or world Z where the normal is too near X for that to be well posed.
fn tangent_basis(normal: &Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let reference = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::z()
    };
    let t1 = (reference - normal * reference.dot(normal)).normalize();
    let t2 = normal.cross(&t1);
    (t1, t2)
}
