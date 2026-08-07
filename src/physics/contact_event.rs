use nalgebra::{Point3, Vector3};

use super::handle::RigidBodyHandle;
use super::pipeline::pair::{PairHeader, SolverContact};

/// Contact event produced by collision detection.
#[derive(Debug, Clone)]
pub struct ContactEvent {
    pub body_a: Option<RigidBodyHandle>,
    pub body_b: RigidBodyHandle,
    pub point: Point3<f32>,
    pub normal: Vector3<f32>,
    pub raw_normal: Vector3<f32>,
    pub depth: f32,
    pub source: ContactSource,
}

impl ContactEvent {
    pub(crate) fn from_solver(
        header: &PairHeader,
        contact: &SolverContact,
        source: ContactSource,
    ) -> Self {
        Self {
            body_a: header.body_a,
            body_b: header.body_b,
            point: contact.point,
            normal: contact.normal,
            raw_normal: contact.raw_normal,
            depth: contact.depth,
            source,
        }
    }
}

/// Source of the contact event in the pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactSource {
    Narrowphase,
    Ccd,
}
