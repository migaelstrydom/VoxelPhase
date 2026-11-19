//! The shafts' mesh: a tube from each shaft's foot to its top, packed for
//! `shader/light_shaft.frag`.
//!
//! Built in world space every frame. The vertex channels carry what the
//! shader needs:
//!
//! | Channel        | Carries                                         |
//! |----------------|-------------------------------------------------|
//! | `color.rgb`    | the shaft's colour times its radiance           |
//! | `color.a`      | its presence, 0..1                              |
//! | `tex_coords.x` | metres up the shaft, less how far streaks rose  |
//! | `tex_coords.y` | how far up, 0 at the foot and 1 at the top      |
//! | `normal`       | outward, for how squarely it is seen            |

use std::f32::consts::TAU;

use nalgebra::{Vector2, Vector3, Vector4};

use crate::rendering::vertex::Vertex;

use super::shaft::LightShaft;

/// Vertices round a shaft, and rings up it: enough rings that its fade and
/// its widening are curves.
const SEGMENTS: u32 = 24;
const RINGS: u32 = 16;

/// The frame's shafts, as one mesh.
#[derive(Debug, Default, Clone)]
pub struct ShaftMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl ShaftMesh {
    /// Every shaft among `shafts`, each with its foot at the world point
    /// given with it. A shaft with nothing of it shown is left out.
    pub fn of<'a>(shafts: impl IntoIterator<Item = (Vector3<f32>, &'a LightShaft)>) -> Self {
        let mut mesh = Self::default();
        for (foot, shaft) in shafts {
            if shaft.look.presence > 0.0 {
                mesh.add(foot, shaft);
            }
        }
        mesh
    }

    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    fn add(&mut self, foot: Vector3<f32>, shaft: &LightShaft) {
        let look = shaft.look;
        let colour = look.colour * look.radiance;
        let first = self.vertices.len() as u32;
        for ring in 0..=RINGS {
            let s = ring as f32 / RINGS as f32;
            let up = s * shaft.height;
            let radius = shaft.foot_radius + (shaft.top_radius - shaft.foot_radius) * s;
            for segment in 0..=SEGMENTS {
                let angle = segment as f32 / SEGMENTS as f32 * TAU;
                let out = Vector3::new(angle.cos(), 0.0, angle.sin());
                self.vertices.push(Vertex {
                    pos: foot + out * radius + Vector3::y() * up,
                    color: Vector4::new(colour.x, colour.y, colour.z, look.presence),
                    tex_coords: Vector2::new(up - shaft.risen, s),
                    normal: out,
                    ao: 0.0,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::light_shaft::ShaftLook;

    #[test]
    fn a_shaft_stands_from_its_foot_and_one_not_shown_is_left_out() {
        let look = ShaftLook {
            colour: Vector3::new(1.0, 1.0, 1.0),
            radiance: 0.5,
            presence: 1.0,
        };
        let shown = LightShaft::new(Vector3::zeros(), 20.0, (0.3, 1.0), look);
        let hidden = LightShaft::new(Vector3::zeros(), 20.0, (0.3, 1.0), ShaftLook::NONE);
        let foot = Vector3::new(5.0, 2.0, -3.0);
        let mesh = ShaftMesh::of([(foot, &shown), (foot, &hidden)]);

        assert!(!mesh.is_empty());
        let count = mesh.vertices.len() as u32;
        assert!(mesh.indices.iter().all(|&i| i < count));
        let (low, high) = mesh
            .vertices
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), v| {
                (lo.min(v.pos.y), hi.max(v.pos.y))
            });
        assert!((low - foot.y).abs() < 1e-5 && (high - foot.y - 20.0).abs() < 1e-4);
        assert_eq!(mesh.vertices.len() as u32, (RINGS + 1) * (SEGMENTS + 1));
    }
}
