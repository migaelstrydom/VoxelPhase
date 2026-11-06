//! Body parts built from ellipsoids, for creatures modelled as a handful of
//! rounded pieces: a fish's body, fins and eyes; a turtle's shell, flippers
//! and the studs on its shell.

use nalgebra::{UnitQuaternion, Vector3};

use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::model::MeshPrimitive;
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;

use super::body_warp::{rebuild_normals, warp_point, Warp};

/// Segments round a part per metre of its longest semi-axis, and the
/// fewest and most any part gets: enough for a smooth silhouette at the size
/// these creatures are seen at, and on a big part enough vertices to carry
/// the paint baked into them ([`super::body_paint`]).
const SEGMENTS_PER_METRE: f32 = 80.0;
const MIN_SEGMENTS: u32 = 16;
const MAX_SEGMENTS: u32 = 32;

/// One ellipsoid of a creature's body, in the body's frame (+Z forward,
/// +Y up).
#[derive(Clone, Copy, Debug)]
pub struct Ellipsoid {
    /// Semi-axes along the part's own x, y and z, m.
    pub radii: Vector3<f32>,
    /// Where its centre sits in the body's frame, m.
    pub centre: Vector3<f32>,
    /// How the part's own axes are turned from the body's.
    pub turn: UnitQuaternion<f32>,
}

impl Ellipsoid {
    /// An ellipsoid on the body's axes.
    pub fn new(radii: (f32, f32, f32), centre: (f32, f32, f32)) -> Self {
        Self {
            radii: Vector3::new(radii.0, radii.1, radii.2),
            centre: Vector3::new(centre.0, centre.1, centre.2),
            turn: UnitQuaternion::identity(),
        }
    }

    /// A sphere of `radius` at `centre`.
    pub fn ball(radius: f32, centre: Vector3<f32>) -> Self {
        Self {
            radii: Vector3::repeat(radius),
            centre,
            turn: UnitQuaternion::identity(),
        }
    }

    /// Turned `yaw` radians about the body's up axis: a flipper swept back.
    pub fn yawed(mut self, yaw: f32) -> Self {
        self.turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw) * self.turn;
        self
    }

    /// Tilted `pitch` radians about its own sideways axis, nose down for a
    /// positive pitch: a beak hooked down.
    pub fn pitched(mut self, pitch: f32) -> Self {
        self.turn *= UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pitch);
        self
    }

    /// Tilted `roll` radians about its own forward axis: a fin canted out.
    pub fn rolled(mut self, roll: f32) -> Self {
        self.turn *= UnitQuaternion::from_axis_angle(&Vector3::z_axis(), roll);
        self
    }

    /// The point on its surface `azimuth` radians round from its own +Z
    /// towards +X and `elevation` radians up from its equator, with the
    /// outward normal there, both in the body's frame.
    pub fn surface_at(&self, azimuth: f32, elevation: f32) -> (Vector3<f32>, Vector3<f32>) {
        let unit = Vector3::new(
            elevation.cos() * azimuth.sin(),
            elevation.sin(),
            elevation.cos() * azimuth.cos(),
        );
        let point = self.turn * unit.component_mul(&self.radii) + self.centre;
        let normal = (self.turn * unit.component_div(&self.radii)).normalize();
        (point, normal)
    }

    /// Roughly how far `point` (in the body's frame) lies outside its
    /// surface, m: negative inside. A first-order estimate, exact on a
    /// sphere and close enough near any ellipsoid's surface, which is the
    /// only place it is asked about.
    pub fn distance_outside(&self, point: Vector3<f32>) -> f32 {
        let local = self.turn.inverse() * (point - self.centre);
        let unit = local.component_div(&self.radii);
        let size = unit.norm();
        if size < 1e-6 {
            return -self.radii.min();
        }
        let gradient = unit.component_div(&self.radii) / size;
        (size - 1.0) / gradient.norm()
    }

    /// The part as a mesh in `colour` and `material`. The colour is the
    /// vertices' — a material's own colour is not applied — so one material
    /// per finish serves every part. Its shape is baked into the vertices,
    /// normals included (by the inverse scale), so a part drawn without a
    /// transform of its own still shades as the ellipsoid it is.
    pub fn mesh(&self, colour: Colour, material: MaterialId) -> MeshPrimitive {
        let segments = ((self.radii.max() * SEGMENTS_PER_METRE).round() as u32)
            .clamp(MIN_SEGMENTS, MAX_SEGMENTS);
        let rings = (segments * 3).div_ceil(4);
        let mut vertices = generate_sphere_vertices(1.0, segments, rings, colour);
        for vertex in &mut vertices {
            vertex.pos = self.turn * vertex.pos.component_mul(&self.radii) + self.centre;
            vertex.normal = (self.turn * vertex.normal.component_div(&self.radii)).normalize();
        }
        MeshPrimitive {
            vertices,
            indices: generate_sphere_indices(segments, rings),
            material,
        }
    }
}

/// A part of a creature's body: an ellipsoid, the warps that bend it off
/// that ellipsoid, and its paint.
#[derive(Clone, Debug)]
pub struct BodyPart {
    /// The ellipsoid it starts as, which is also what other parts' shade
    /// is judged against.
    pub shape: Ellipsoid,
    /// Warps applied to it, in order.
    pub warps: Vec<Warp>,
    /// Its vertex colour.
    pub colour: Colour,
    /// What it is drawn in.
    pub material: MaterialId,
}

impl BodyPart {
    /// `shape` unwarped, in `look`: its colour and material.
    pub fn new(shape: Ellipsoid, look: (Colour, MaterialId)) -> Self {
        Self {
            shape,
            warps: Vec::new(),
            colour: look.0,
            material: look.1,
        }
    }

    /// With `warp` applied after those it already has.
    pub fn warped(mut self, warp: Warp) -> Self {
        self.warps.push(warp);
        self
    }

    /// The point on its warped surface over [`Ellipsoid::surface_at`], with
    /// the ellipsoid's normal there: for setting something on it.
    pub fn surface_at(&self, azimuth: f32, elevation: f32) -> (Vector3<f32>, Vector3<f32>) {
        let (point, normal) = self.shape.surface_at(azimuth, elevation);
        (self.warp(point, normal), normal)
    }

    /// Its mesh, warped.
    pub fn mesh(&self) -> MeshPrimitive {
        let mut mesh = self.shape.mesh(self.colour, self.material);
        if self.warps.is_empty() {
            return mesh;
        }
        for vertex in &mut mesh.vertices {
            vertex.pos = self.warp(vertex.pos, vertex.normal);
        }
        rebuild_normals(&mut mesh);
        mesh
    }

    /// `point` on its ellipsoid, with `normal` there, moved by its warps.
    fn warp(&self, point: Vector3<f32>, normal: Vector3<f32>) -> Vector3<f32> {
        let shape = &self.shape;
        warp_point(
            &self.warps,
            shape.centre,
            shape.turn,
            shape.radii,
            point,
            normal,
        )
    }
}
