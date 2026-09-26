//! What a static triangle is made of, as the geometry that owns it names it.
//!
//! The collision library carries the id and never reads it: a triangle keeps
//! it through the seam filter, and every contact generated against that
//! triangle takes it. Only the static geometry that issued an id can say what
//! it means (`StaticGeometry::surface` in the physics engine), so terrain can
//! number its voxel materials without the collision code knowing what grass is.

/// Opaque identifier of a static triangle's surface material.
///
/// `SurfaceId::UNSPECIFIED` — the default — is geometry that stands for no
/// material in particular: a bench ramp, a test quad. A contact against it
/// behaves as the collider's own material says.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SurfaceId(pub u8);

impl SurfaceId {
    /// Geometry with no material of its own.
    pub const UNSPECIFIED: Self = Self(0);
}
