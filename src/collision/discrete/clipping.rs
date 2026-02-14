//! Sutherland-Hodgman polygon clipping and OBB face utilities.
//!
//! Used by OBB-OBB and OBB-patch manifold generation to clip incident faces
//! against reference face side planes.

use nalgebra::{Point3, Vector3};

use crate::collision::obb::Obb;

/// Clip a convex polygon against a half-plane.
///
/// Keeps the portion of the polygon on the inside (non-negative side) of the plane.
/// The plane is defined by a point on the plane and an inward-pointing normal.
///
/// Returns the clipped polygon vertices in winding order.
pub fn clip_polygon(
    polygon: &[Point3<f32>],
    plane_point: Point3<f32>,
    plane_normal: Vector3<f32>,
) -> Vec<Point3<f32>> {
    if polygon.is_empty() {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(polygon.len() + 2);

    for i in 0..polygon.len() {
        let p1 = polygon[i];
        let p2 = polygon[(i + 1) % polygon.len()];
        let d1 = (p1 - plane_point).dot(&plane_normal);
        let d2 = (p2 - plane_point).dot(&plane_normal);
        let inside1 = d1 >= 0.0;
        let inside2 = d2 >= 0.0;

        if inside1 && inside2 {
            result.push(p2);
        } else if inside1 && !inside2 {
            let t = d1 / (d1 - d2);
            result.push(p1 + (p2 - p1) * t);
        } else if !inside1 && inside2 {
            let t = d1 / (d1 - d2);
            result.push(p1 + (p2 - p1) * t);
            result.push(p2);
        }
    }

    result
}

/// Get the 4 vertices of an OBB face, the face center, and the face tangent/bitangent.
///
/// `axis_index` selects which OBB axis is the face normal (0=X, 1=Y, 2=Z).
/// `sign` selects which of the two parallel faces (+1.0 or -1.0).
///
/// Returns `(vertices, center, tangent_u, tangent_v, half_u, half_v)`.
pub fn obb_face(
    obb: &Obb,
    axis_index: usize,
    sign: f32,
) -> ObbFace {
    let axes = obb.axes();
    let he = obb.half_extents;

    let (u_idx, v_idx) = match axis_index {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };

    let normal = axes[axis_index] * sign;
    let center = obb.center + normal * he[axis_index];
    let u = axes[u_idx];
    let v = axes[v_idx];
    let hu = he[u_idx];
    let hv = he[v_idx];

    let vertices = [
        center + u * hu + v * hv,
        center - u * hu + v * hv,
        center - u * hu - v * hv,
        center + u * hu - v * hv,
    ];

    ObbFace {
        vertices,
        center,
        normal,
        tangent_u: u,
        tangent_v: v,
        half_u: hu,
        half_v: hv,
    }
}

/// A face of an OBB with geometry needed for clipping.
pub struct ObbFace {
    /// The 4 corner vertices of the face (CCW winding from outside).
    pub vertices: [Point3<f32>; 4],
    /// Center of the face.
    pub center: Point3<f32>,
    /// Outward face normal.
    pub normal: Vector3<f32>,
    /// First tangent direction on the face plane.
    pub tangent_u: Vector3<f32>,
    /// Second tangent direction on the face plane.
    pub tangent_v: Vector3<f32>,
    /// Half-extent along tangent_u.
    pub half_u: f32,
    /// Half-extent along tangent_v.
    pub half_v: f32,
}

impl ObbFace {
    /// Clip a polygon against this face's 4 side planes.
    ///
    /// The side planes are the edges of the face, extruded inward.
    pub fn clip_against_sides(&self, polygon: &[Point3<f32>]) -> Vec<Point3<f32>> {
        let mut clipped = polygon.to_vec();
        clipped = clip_polygon(&clipped, self.center + self.tangent_u * self.half_u, -self.tangent_u);
        clipped = clip_polygon(&clipped, self.center - self.tangent_u * self.half_u, self.tangent_u);
        clipped = clip_polygon(&clipped, self.center + self.tangent_v * self.half_v, -self.tangent_v);
        clip_polygon(&clipped, self.center - self.tangent_v * self.half_v, self.tangent_v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    #[test]
    fn clip_polygon_keeps_inside() {
        let polygon = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ];
        // Clip: keep x >= 1.0
        let result = clip_polygon(&polygon, Point3::new(1.0, 0.0, 0.0), Vector3::x());
        assert!(!result.is_empty());
        for p in &result {
            assert!(p.x >= 1.0 - 1e-6, "Point should be on inside, got x={}", p.x);
        }
    }

    #[test]
    fn clip_polygon_removes_outside() {
        let polygon = vec![
            Point3::new(-2.0, 0.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(-2.0, 1.0, 0.0),
        ];
        // Clip: keep x >= 0.0
        let result = clip_polygon(&polygon, Point3::origin(), Vector3::x());
        assert!(result.is_empty());
    }

    #[test]
    fn clip_polygon_partial() {
        let polygon = vec![
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
        ];
        // Clip: keep x >= 0.0
        let result = clip_polygon(&polygon, Point3::origin(), Vector3::x());
        assert_eq!(result.len(), 4);
        for p in &result {
            assert!(p.x >= -1e-6);
        }
    }

    #[test]
    fn obb_face_identity_box() {
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let face = obb_face(&obb, 0, 1.0); // +X face
        assert!((face.center - Point3::new(1.0, 0.0, 0.0)).magnitude() < 1e-6);
        assert!((face.normal - Vector3::x()).magnitude() < 1e-6);
        assert_eq!(face.vertices.len(), 4);
    }

    #[test]
    fn obb_face_clip_against_sides() {
        let obb = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let face = obb_face(&obb, 0, 1.0);
        // A large polygon that extends well beyond the face should be clipped to the face area.
        let large_poly: Vec<Point3<f32>> = vec![
            Point3::new(1.0, 5.0, 5.0),
            Point3::new(1.0, -5.0, 5.0),
            Point3::new(1.0, -5.0, -5.0),
            Point3::new(1.0, 5.0, -5.0),
        ];
        let clipped = face.clip_against_sides(&large_poly);
        assert!(!clipped.is_empty());
        for p in &clipped {
            assert!(p.y.abs() <= 1.0 + 1e-5);
            assert!(p.z.abs() <= 1.0 + 1e-5);
        }
    }
}
