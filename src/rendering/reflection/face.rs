use nalgebra::{Matrix4, Vector3, Vector4};

use crate::rendering::reflection::caster::BoundingSphere;

/// One face of a cube map, in the order Vulkan lays a cube's six layers out.
///
/// A face is a 90° square view from the probe's centre along one axis. The
/// hard part is the orientation within the face: a cube map is sampled by
/// direction, and the texel a direction lands on is fixed by the Vulkan spec
/// (§16.5.4, "Cube Map Face Selection"), not by any camera convention. Each
/// face's `right` and `down` axes are read straight from that table, so that
/// a face rendered through [`Self::view_proj`] puts every direction on the
/// texel a later lookup of it reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CubeFace {
    PosX,
    NegX,
    PosY,
    NegY,
    PosZ,
    NegZ,
}

impl CubeFace {
    pub const COUNT: usize = 6;

    /// Every face, in layer order.
    pub const ALL: [CubeFace; Self::COUNT] = [
        CubeFace::PosX,
        CubeFace::NegX,
        CubeFace::PosY,
        CubeFace::NegY,
        CubeFace::PosZ,
        CubeFace::NegZ,
    ];

    /// The face's layer within its cube: 0 to 5.
    pub fn layer(self) -> u32 {
        self as u32
    }

    /// The axis the face looks along: the major axis of every direction it
    /// holds.
    pub fn forward(self) -> Vector3<f32> {
        match self {
            CubeFace::PosX => Vector3::x(),
            CubeFace::NegX => -Vector3::x(),
            CubeFace::PosY => Vector3::y(),
            CubeFace::NegY => -Vector3::y(),
            CubeFace::PosZ => Vector3::z(),
            CubeFace::NegZ => -Vector3::z(),
        }
    }

    /// The world axis that runs along the face's texel columns, left to right:
    /// the spec's `sc`.
    pub fn right(self) -> Vector3<f32> {
        match self {
            CubeFace::PosX => -Vector3::z(),
            CubeFace::NegX => Vector3::z(),
            CubeFace::PosY | CubeFace::NegY | CubeFace::PosZ => Vector3::x(),
            CubeFace::NegZ => -Vector3::x(),
        }
    }

    /// The world axis that runs down the face's texel rows, top to bottom: the
    /// spec's `tc`.
    pub fn down(self) -> Vector3<f32> {
        match self {
            CubeFace::PosY => Vector3::z(),
            CubeFace::NegY => -Vector3::z(),
            _ => -Vector3::y(),
        }
    }

    /// World space to this face's clip space, for a probe at `centre`.
    ///
    /// Built from the face's axes rather than from a look-at and a
    /// perspective matrix: clip x and y *are* the spec's `sc` and `tc`, w is
    /// the distance along the face's axis, and depth runs 0 to 1 from `near`
    /// to `far`. Vulkan's framebuffer puts NDC y = -1 on its top row, which
    /// is the row the spec's t = 0 names, so nothing needs flipping.
    ///
    /// The result is mirrored relative to the scene camera — a cube map is
    /// seen from inside — so triangles that face the probe arrive with the
    /// opposite winding. The capture pipeline declares its front face
    /// accordingly.
    pub fn view_proj(self, centre: &Vector3<f32>, near: f32, far: f32) -> Matrix4<f32> {
        let right = self.right();
        let down = self.down();
        let forward = self.forward();
        let depth_scale = far / (far - near);
        let depth_bias = -far * near / (far - near);

        let row = |axis: Vector3<f32>| Vector4::new(axis.x, axis.y, axis.z, -axis.dot(centre));
        let along = row(forward);
        let depth = along * depth_scale + Vector4::new(0.0, 0.0, 0.0, depth_bias);

        Matrix4::from_rows(&[
            row(right).transpose(),
            row(down).transpose(),
            depth.transpose(),
            along.transpose(),
        ])
    }

    /// Whether any of `sphere` falls inside this face's view from `centre`,
    /// nearer than `near` excepted.
    ///
    /// The face's frustum is the 90° pyramid around its axis, bounded by the
    /// four planes where a side coordinate equals the forward one. A sphere is
    /// outside when it lies wholly beyond any of them.
    pub fn sees(self, centre: &Vector3<f32>, sphere: &BoundingSphere, near: f32) -> bool {
        let offset = sphere.centre - centre;
        let x = self.right().dot(&offset);
        let y = self.down().dot(&offset);
        let z = self.forward().dot(&offset);
        // A side plane's normal is (±1, 0, -1)/√2 in the face's frame, so the
        // sphere's distance past it is (±x - z)/√2.
        let slack = sphere.radius * std::f32::consts::SQRT_2;
        z + sphere.radius > near
            && x - z <= slack
            && -x - z <= slack
            && y - z <= slack
            && -y - z <= slack
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The face a direction is looked up in and the (s, t) it lands on,
    /// written out as the spec's table rather than through the face's axes.
    fn spec_lookup(d: Vector3<f32>) -> (CubeFace, f32, f32) {
        let (ax, ay, az) = (d.x.abs(), d.y.abs(), d.z.abs());
        let (face, sc, tc, ma) = if ax >= ay && ax >= az {
            if d.x > 0.0 {
                (CubeFace::PosX, -d.z, -d.y, ax)
            } else {
                (CubeFace::NegX, d.z, -d.y, ax)
            }
        } else if ay >= az {
            if d.y > 0.0 {
                (CubeFace::PosY, d.x, d.z, ay)
            } else {
                (CubeFace::NegY, d.x, -d.z, ay)
            }
        } else if d.z > 0.0 {
            (CubeFace::PosZ, d.x, -d.y, az)
        } else {
            (CubeFace::NegZ, -d.x, -d.y, az)
        };
        (face, (sc / ma + 1.0) * 0.5, (tc / ma + 1.0) * 0.5)
    }

    #[test]
    fn a_rendered_direction_lands_where_a_lookup_reads_it() {
        let centre = Vector3::new(3.0, -1.0, 7.5);
        let directions = [
            Vector3::new(1.0, 0.2, -0.3),
            Vector3::new(-1.0, 0.5, 0.4),
            Vector3::new(0.3, 1.0, -0.6),
            Vector3::new(-0.2, -1.0, 0.7),
            Vector3::new(0.6, -0.1, 1.0),
            Vector3::new(-0.4, 0.3, -1.0),
        ];
        for d in directions {
            let (face, s, t) = spec_lookup(d);
            let world = centre + d * 4.0;
            let clip = face.view_proj(&centre, 0.1, 100.0) * world.push(1.0);
            let (u, v) = (clip.x / clip.w, clip.y / clip.w);
            // NDC -1..1 maps onto texel coordinates 0..1, top row first.
            assert!(((u + 1.0) * 0.5 - s).abs() < 1e-5, "{face:?} s for {d:?}");
            assert!(((v + 1.0) * 0.5 - t).abs() < 1e-5, "{face:?} t for {d:?}");
        }
    }

    #[test]
    fn depth_runs_zero_to_one_from_near_to_far() {
        let centre = Vector3::new(1.0, 2.0, 3.0);
        for face in CubeFace::ALL {
            let view_proj = face.view_proj(&centre, 0.5, 50.0);
            let depth = |distance: f32| {
                let clip = view_proj * (centre + face.forward() * distance).push(1.0);
                clip.z / clip.w
            };
            assert!(depth(0.5).abs() < 1e-5);
            assert!((depth(50.0) - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn every_face_is_mirrored_relative_to_a_camera() {
        // A camera's right is forward × up, and Vulkan's clip y points down,
        // so its (right, down, forward) has right = down × forward. A cube
        // face has the opposite handedness, which flips triangle winding.
        for face in CubeFace::ALL {
            let handedness = face.right().dot(&face.down().cross(&face.forward()));
            assert!((handedness + 1.0).abs() < 1e-6, "{face:?}");
        }
    }

    #[test]
    fn a_sphere_is_seen_only_by_the_faces_it_reaches_into() {
        let centre = Vector3::zeros();
        let ahead = BoundingSphere {
            centre: Vector3::new(5.0, 0.0, 0.0),
            radius: 0.5,
        };
        let seen: Vec<CubeFace> = CubeFace::ALL
            .into_iter()
            .filter(|face| face.sees(&centre, &ahead, 0.05))
            .collect();
        assert_eq!(seen, vec![CubeFace::PosX]);

        let straddling = BoundingSphere {
            centre: Vector3::new(5.0, 5.0, 0.0),
            radius: 0.5,
        };
        assert!(CubeFace::PosX.sees(&centre, &straddling, 0.05));
        assert!(CubeFace::PosY.sees(&centre, &straddling, 0.05));
        assert!(!CubeFace::NegX.sees(&centre, &straddling, 0.05));

        let around = BoundingSphere {
            centre: Vector3::new(0.1, 0.0, 0.0),
            radius: 2.0,
        };
        assert!(CubeFace::ALL
            .into_iter()
            .all(|face| face.sees(&centre, &around, 0.05)));
    }
}
