//! Body parts built from ellipsoids, for creatures modelled as a handful of
//! rounded pieces: a fish's body, fins and eyes; a turtle's shell and flippers.

use nalgebra::{UnitQuaternion, Vector3};

use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::model::MeshPrimitive;
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;

/// Segments and rings of the sphere every part is made from. Enough for a
/// smooth silhouette at the size these creatures are seen at.
const SEGMENTS: u32 = 16;
const RINGS: u32 = 12;

/// One ellipsoid of a creature's body, in the body's frame (+Z forward,
/// +Y up).
pub struct Ellipsoid {
    /// Semi-axes along the part's own x, y and z, m.
    pub radii: Vector3<f32>,
    /// Where its centre sits in the body's frame, m.
    pub centre: Vector3<f32>,
    /// Turn about the body's up axis, radians: a flipper swept back.
    pub yaw: f32,
    /// Tilt about the body's forward axis, radians: a fin canted out.
    pub roll: f32,
}

impl Ellipsoid {
    /// An ellipsoid on the body's axes.
    pub fn new(radii: (f32, f32, f32), centre: (f32, f32, f32)) -> Self {
        Self {
            radii: Vector3::new(radii.0, radii.1, radii.2),
            centre: Vector3::new(centre.0, centre.1, centre.2),
            yaw: 0.0,
            roll: 0.0,
        }
    }

    /// Turned `yaw` radians about the body's up axis.
    pub fn yawed(mut self, yaw: f32) -> Self {
        self.yaw = yaw;
        self
    }

    /// Tilted `roll` radians about the body's forward axis.
    pub fn rolled(mut self, roll: f32) -> Self {
        self.roll = roll;
        self
    }

    /// The part as a mesh in `colour` and `material`. The colour is the
    /// vertices' — a material's own colour is not applied — so one material
    /// per finish serves every part. Its shape is baked into the vertices,
    /// normals included (by the inverse scale), so a part drawn without a
    /// transform of its own still shades as the ellipsoid it is.
    pub fn mesh(&self, colour: Colour, material: MaterialId) -> MeshPrimitive {
        let turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), self.yaw)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.roll);
        let mut vertices = generate_sphere_vertices(1.0, SEGMENTS, RINGS, colour);
        for vertex in &mut vertices {
            vertex.pos = turn * vertex.pos.component_mul(&self.radii) + self.centre;
            vertex.normal = (turn * vertex.normal.component_div(&self.radii)).normalize();
        }
        MeshPrimitive {
            vertices,
            indices: generate_sphere_indices(SEGMENTS, RINGS),
            material,
        }
    }
}
