//! Building a rig's mesh out of rods, blobs and soles.
//!
//! A procedural character is drawn by walking its joints and dropping a
//! primitive at each one. The primitives are the same whoever is walking:
//! a sphere at a joint, a rod between two of them, a capsule lying flat on
//! the ground for a foot. What differs between rigs is which joints exist
//! and what colour they are, and that stays with the rig.

use nalgebra::{Point3, Vector3};

use crate::geometry::{
    generate_capsule, generate_cylinder, generate_sphere_indices, generate_sphere_vertices,
};
use crate::rendering::{colour::Colour, vertex::Vertex};

/// The dimensions of a rendered foot.
///
/// Convention: the foot's position is its *centre* — the capsule is drawn
/// centred on it, so the sole ends up one `radius` below. When the placer
/// tracks per-foot terrain contact the sole sits one radius below the
/// ground, which is the half-submerged look a stylised rig wants.
#[derive(Clone, Copy, Debug)]
pub struct FootShape {
    /// Capsule radius. Half the rendered thickness of the foot.
    pub radius: f32,
    /// Distance from centre to hemisphere endcap centre along the toe
    /// axis.
    pub half_length: f32,
    /// Forward shift of the capsule centre from the ankle, so the heel is
    /// shorter than the toe. Toe extent = `half_length + offset`; heel
    /// extent = `half_length - offset`.
    pub ankle_forward_offset: f32,
}

impl FootShape {
    /// Vertical thickness of the foot, top tangent to bottom tangent.
    /// Converts a terrain-surface y to a foot-centre y: the centre sits
    /// half of this above the sole.
    pub fn height(&self) -> f32 {
        2.0 * self.radius
    }
}

/// An accumulating triangle mesh for one rig.
pub struct RigMesh {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
    /// Tessellation of every primitive added to this mesh.
    segments: u32,
}

impl RigMesh {
    pub fn new(segments: u32) -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            segments,
        }
    }

    /// A joint blob.
    pub fn sphere(&mut self, centre: Point3<f32>, radius: f32, colour: Colour) {
        let base = self.vertices.len() as u32;
        let verts = generate_sphere_vertices(radius, self.segments, self.segments, colour);
        let indices = generate_sphere_indices(self.segments, self.segments);

        for mut vert in verts {
            vert.pos += centre.coords;
            self.vertices.push(vert);
        }
        self.indices.extend(indices.iter().map(|i| i + base));
    }

    /// A bone between two joints.
    pub fn rod(&mut self, start: Point3<f32>, end: Point3<f32>, radius: f32, colour: Colour) {
        let base = self.vertices.len() as u32;
        let (verts, indices) = generate_cylinder(start, end, radius, self.segments, colour);

        self.vertices.extend(verts);
        self.indices.extend(indices.iter().map(|i| i + base));
    }

    /// A foot: a capsule lying along its toe axis, tipped so its sole
    /// follows `up`.
    ///
    /// Built from `up` rather than kept horizontal so a foot on a slope
    /// tips its toe along the ground instead of cutting into it.
    pub fn foot(
        &mut self,
        shape: FootShape,
        position: Point3<f32>,
        forward: Vector3<f32>,
        up: Vector3<f32>,
        colour: Colour,
    ) {
        let base = self.vertices.len() as u32;
        let (verts, indices) = generate_capsule(
            shape.half_length,
            shape.radius,
            self.segments,
            (self.segments / 2).max(4),
            colour,
        );

        // Right-handed basis: local Y → toe direction in the ground plane,
        // local X → sole normal, local Z → lateral.
        let up = up.try_normalize(1e-4).unwrap_or_else(Vector3::y);
        let forward_h = project_to_horizontal(forward);
        let lateral = up.cross(&forward_h).try_normalize(1e-4).unwrap_or_else(|| {
            // Foot up is already aligned with the intended forward (nearly
            // vertical foot). Fall back to facing-cross-Y lateral.
            Vector3::y().cross(&forward_h).normalize()
        });
        let axis = lateral.cross(&up).normalize();

        let centre = position + axis * shape.ankle_forward_offset;

        for mut vert in verts {
            let local = vert.pos;
            vert.pos = (centre + up * local.x + axis * local.y + lateral * local.z).coords;
            // Rotate the vertex normal with the same basis so lighting
            // reflects the orientation.
            let n = vert.normal;
            vert.normal = up * n.x + axis * n.y + lateral * n.z;
            self.vertices.push(vert);
        }
        self.indices.extend(indices.iter().map(|i| i + base));
    }

    pub fn into_parts(self) -> (Vec<Vertex>, Vec<u32>) {
        (self.vertices, self.indices)
    }
}

/// A direction flattened into the ground plane, defaulting to +Z when it
/// had no horizontal component to keep.
pub fn project_to_horizontal(v: Vector3<f32>) -> Vector3<f32> {
    let planar = Vector3::new(v.x, 0.0, v.z);
    planar
        .try_normalize(1e-4)
        .unwrap_or_else(|| Vector3::new(0.0, 0.0, 1.0))
}

/// The rig's lateral axis for a body facing `facing`.
#[inline]
pub fn right_vector(facing: Vector3<f32>) -> Vector3<f32> {
    facing.cross(&Vector3::y()).normalize()
}
