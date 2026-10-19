//! The column mesh: a flared tube from a pod to its ground, packed for
//! `shader/force_field.frag`.
//!
//! ```text
//!      pod ═══╤═══   top_radius
//!            ╱ ╲     bands scroll down, s = 0 → 1
//!           ╱   ╲
//!   ground ╱_____╲   foot_radius: flares as the spring is pressed in
//! ```
//!
//! Built in world space every frame, since every part of it moves. The
//! vertex channels carry what the shader needs, and nothing it does not:
//!
//! | Channel        | Carries                                              |
//! |----------------|------------------------------------------------------|
//! | `color.rgb`    | the field's colour times its radiance                 |
//! | `color.a`      | its presence, 0..1                                    |
//! | `tex_coords.x` | the band coordinate: metres down / spacing − phase    |
//! | `tex_coords.y` | how far down, 0 at the pod and 1 at the ground        |
//! | `normal`       | outward, for the Fresnel rim                          |
//! | `ao`           | strain, 0..1                                          |

use nalgebra::{Vector2, Vector3, Vector4};

use crate::rendering::vertex::Vertex;

use super::pod::PodLook;

/// Vertices around the column.
const SEGMENTS: u32 = 20;
/// Rings down it: enough that the flare is a curve.
const RINGS: u32 = 10;
/// How the flare grows down the column: a little past 1, so it widens like
/// a beam thrown at the ground and bells out a touch where it strikes.
const FLARE_EXPONENT: f32 = 1.6;

/// The frame's columns, as one mesh.
#[derive(Debug, Default, Clone)]
pub struct ColumnMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl ColumnMesh {
    /// Every column among `looks`.
    pub fn of(looks: &[PodLook]) -> Self {
        let mut mesh = Self::default();
        for look in looks {
            mesh.add(look);
        }
        mesh
    }

    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    /// Add `look`'s column, if it has one.
    fn add(&mut self, look: &PodLook) {
        let Some(column) = look.column else {
            return;
        };
        let down = -look.axis;
        let (across, along) = basis(&look.axis);
        let colour = look.colour * column.radiance;
        let first = self.vertices.len() as u32;

        for ring in 0..=RINGS {
            let s = ring as f32 / RINGS as f32;
            let depth = s * column.length;
            let radius = column.top_radius
                + (column.foot_radius - column.top_radius) * s.powf(FLARE_EXPONENT);
            let centre = look.anchor + down * depth;
            let band = depth / column.band_spacing - column.phase;
            for segment in 0..=SEGMENTS {
                let angle = segment as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
                let outward = across * angle.cos() + along * angle.sin();
                self.vertices.push(Vertex {
                    pos: (centre + outward * radius).coords,
                    color: Vector4::new(colour.x, colour.y, colour.z, column.presence),
                    tex_coords: Vector2::new(band, s),
                    normal: outward,
                    ao: column.strain,
                });
            }
        }

        let stride = SEGMENTS + 1;
        for ring in 0..RINGS {
            for segment in 0..SEGMENTS {
                let a = first + ring * stride + segment;
                let b = a + stride;
                self.indices
                    .extend_from_slice(&[a, b, a + 1, a + 1, b, b + 1]);
            }
        }
    }
}

/// Two unit vectors square to `axis` and to each other.
pub fn basis(axis: &Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let reference = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::z()
    };
    let across = axis.cross(&reference).normalize();
    let along = axis.cross(&across);
    (across, along)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::force_field::pod::ColumnLook;
    use nalgebra::Point3;

    fn look(column: Option<ColumnLook>) -> PodLook {
        PodLook {
            anchor: Point3::new(0.0, 1.0, 0.0),
            axis: Vector3::y(),
            colour: Vector3::new(0.2, 0.8, 1.0),
            heat: 0.2,
            pod_glow: 2.0,
            column,
            ground_ring: None,
            pulse_ring: None,
            ground_speed: 0.0,
        }
    }

    const COLUMN: ColumnLook = ColumnLook {
        length: 0.5,
        top_radius: 0.1,
        foot_radius: 0.3,
        radiance: 2.0,
        presence: 1.0,
        band_spacing: 0.1,
        phase: 0.25,
        strain: 0.0,
    };

    #[test]
    fn the_column_runs_from_the_pod_to_its_foot_and_flares() {
        let mesh = ColumnMesh::of(&[look(Some(COLUMN))]);
        let top = mesh
            .vertices
            .iter()
            .map(|v| v.pos.y)
            .fold(f32::MIN, f32::max);
        let bottom = mesh
            .vertices
            .iter()
            .map(|v| v.pos.y)
            .fold(f32::MAX, f32::min);
        assert!((top - 1.0).abs() < 1e-6 && (bottom - 0.5).abs() < 1e-6);

        let widest = |y: f32| {
            mesh.vertices
                .iter()
                .filter(|v| (v.pos.y - y).abs() < 1e-4)
                .map(|v| Vector2::new(v.pos.x, v.pos.z).norm())
                .fold(0.0, f32::max)
        };
        assert!((widest(1.0) - 0.1).abs() < 1e-5);
        assert!((widest(0.5) - 0.3).abs() < 1e-5);
    }

    /// Bands are a whole band apart every `band_spacing` metres down, and
    /// scrolling the phase moves them down.
    #[test]
    fn the_band_coordinate_counts_bands_down_the_column() {
        let mesh = ColumnMesh::of(&[look(Some(COLUMN))]);
        let at_pod = mesh.vertices.first().unwrap().tex_coords.x;
        let at_foot = mesh.vertices.last().unwrap().tex_coords.x;
        assert!((at_pod + 0.25).abs() < 1e-6);
        assert!((at_foot - at_pod - 5.0).abs() < 1e-5);
    }

    #[test]
    fn a_pod_with_no_column_adds_nothing() {
        assert!(ColumnMesh::of(&[look(None)]).is_empty());
    }

    /// Every triangle's corners are in the mesh.
    #[test]
    fn the_indices_stay_in_the_mesh() {
        let mesh = ColumnMesh::of(&[look(Some(COLUMN)), look(Some(COLUMN))]);
        let count = mesh.vertices.len() as u32;
        assert!(mesh.indices.iter().all(|&i| i < count));
        assert_eq!(mesh.indices.len() % 3, 0);
    }
}
