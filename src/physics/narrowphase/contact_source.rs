use crate::physics::pipeline::solver::ContactConstraint;

/// Triangle feature that produced a terrain contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactFeature {
    Face,
    Edge(u8),
    Vertex(u8),
}

/// Provenance for a contact generated against a mesh patch triangle.
#[derive(Debug, Clone, Copy)]
pub struct ContactSource {
    pub triangle_idx: u32,
    pub feature: ContactFeature,
}

/// Contact plus mesh-feature provenance used by filtering/manifold stages.
#[derive(Debug, Clone)]
pub struct SourcedContact {
    pub constraint: ContactConstraint,
    pub source: ContactSource,
}
