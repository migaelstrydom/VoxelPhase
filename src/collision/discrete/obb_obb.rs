//! OBB vs OBB collision using SAT with Sutherland-Hodgman face clipping (tier 2).
//!
//! Tests 15 candidate axes (3+3 face normals, 9 edge-edge cross products).
//! Correctly distinguishes face-face and edge-edge minimum-penetration axes,
//! generating appropriate contact geometry for each case.

use nalgebra::{Point3, Vector3};

use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::discrete::clipping::obb_face;
use crate::collision::obb::Obb;
use crate::collision::sat::{SatCache, AXIS_EPS, OVERLAP_EPS};
use crate::collision::segment::segment_segment_closest_points;

/// Minimum penetration axis category.
#[derive(Debug, Clone, Copy)]
enum MinAxis {
    /// Face normal on OBB A (axis index 0-2).
    FaceA(usize),
    /// Face normal on OBB B (axis index 0-2).
    FaceB(usize),
    /// Edge-edge cross product: edge `a_idx` on A × edge `b_idx` on B.
    EdgeEdge { a_idx: usize, b_idx: usize },
}

/// Test two OBBs against each other using SAT, with optional cache for early-out.
///
/// When a `SatCache` is provided, the cached separating axis (if any) is tested
/// first. If it still separates the pair, the function returns immediately without
/// testing the remaining 14 axes. For stable non-colliding pairs this reduces
/// the cost to a single axis test.
///
/// The cache is updated on return:
/// - Separated pair → stores the best separating axis for next frame.
/// - Colliding pair → clears the cache (`separating_axis = None`).
pub fn obb_obb_manifold_cached(
    a: &Obb,
    b: &Obb,
    contact_margin: f32,
    cache: &mut SatCache,
) -> ContactManifold {
    let center_dir = b.center - a.center;

    // Try the cached separating axis first.
    if let Some(cached_axis) = cache.separating_axis {
        let distance = center_dir.dot(&cached_axis).abs();
        let overlap =
            a.project_half_extent(&cached_axis) + b.project_half_extent(&cached_axis)
                + 2.0 * contact_margin - distance;
        if overlap < -OVERLAP_EPS {
            // Still separated on this axis — early out.
            return ContactManifold::empty();
        }
    }

    // Cached axis didn't separate (or no cache). Run full SAT.
    let manifold = obb_obb_manifold_inner(a, b, contact_margin, &center_dir, cache);
    manifold
}

/// Test two OBBs against each other using SAT.
///
/// Returns a manifold of up to 4 contact points. Normal points from A toward B.
/// Correctly handles face-face, face-edge, and edge-edge contact configurations.
///
/// # Arguments
/// * `a` — first OBB in world space
/// * `b` — second OBB in world space
/// * `contact_margin` — inflation for speculative contacts
pub fn obb_obb_manifold(a: &Obb, b: &Obb, contact_margin: f32) -> ContactManifold {
    let center_dir = b.center - a.center;
    obb_obb_manifold_inner(a, b, contact_margin, &center_dir, &mut SatCache::new())
}

/// Core SAT implementation shared by cached and uncached entry points.
fn obb_obb_manifold_inner(
    a: &Obb,
    b: &Obb,
    contact_margin: f32,
    center_dir: &Vector3<f32>,
    cache: &mut SatCache,
) -> ContactManifold {
    let axes_a = a.axes();
    let axes_b = b.axes();

    let mut best_overlap = f32::MAX;
    let mut best_axis = Vector3::zeros();
    let mut best_category = MinAxis::FaceA(0);
    let mut best_edge_candidate: Option<(usize, usize, Vector3<f32>, f32)> = None;

    // Track the best separating axis seen during the full test. If we find
    // a separating axis, we'll store it in the cache for next frame.
    let mut best_separating_axis: Option<(Vector3<f32>, f32)> = None;
    let mut update_separating = |axis: Vector3<f32>, overlap: f32| {
        if overlap < 0.0 {
            if best_separating_axis.map_or(true, |(_, best)| overlap < best) {
                best_separating_axis = Some((axis, overlap));
            }
        }
    };

    // Test face normals of A (axes 0-2).
    for i in 0..3 {
        let result = test_face_axis(&axes_a[i], a, b, center_dir, contact_margin);
        match result {
            None => {
                // Separated on this axis. Cache it.
                let axis = axes_a[i].normalize();
                cache.separating_axis = Some(axis);
                return ContactManifold::empty();
            }
            Some((axis, overlap)) => {
                update_separating(axis, overlap);
                if overlap < best_overlap {
                    best_overlap = overlap;
                    best_axis = axis;
                    best_category = MinAxis::FaceA(i);
                }
            }
        }
    }

    // Test face normals of B (axes 3-5).
    for i in 0..3 {
        let result = test_face_axis(&axes_b[i], a, b, center_dir, contact_margin);
        match result {
            None => {
                let axis = axes_b[i].normalize();
                cache.separating_axis = Some(axis);
                return ContactManifold::empty();
            }
            Some((axis, overlap)) => {
                update_separating(axis, overlap);
                if overlap < best_overlap {
                    best_overlap = overlap;
                    best_axis = axis;
                    best_category = MinAxis::FaceB(i);
                }
            }
        }
    }

    // Test 9 edge-edge cross product axes.
    // Apply a small bias to prefer face axes over edge axes when overlaps are similar.
    // This prevents noisy edge-edge contacts when a face-face solution is nearly as good.
    let face_best = best_overlap;
    for i in 0..3 {
        for j in 0..3 {
            let cross = axes_a[i].cross(&axes_b[j]);
            let len_sq = cross.magnitude_squared();
            if len_sq < AXIS_EPS {
                continue;
            }
            let len = len_sq.sqrt();
            let mut axis = cross / len;

            if axis.dot(center_dir) < 0.0 {
                axis = -axis;
            }

            let distance = center_dir.dot(&axis).abs();
            let overlap =
                a.project_half_extent(&axis) + b.project_half_extent(&axis) + 2.0 * contact_margin
                    - distance;

            if overlap < -OVERLAP_EPS {
                cache.separating_axis = Some(axis);
                return ContactManifold::empty();
            }

            update_separating(axis, overlap);

            if best_edge_candidate
                .map(|(_, _, _, best_edge_overlap)| overlap < best_edge_overlap)
                .unwrap_or(true)
            {
                best_edge_candidate = Some((i, j, axis, overlap));
            }

            // Bias: prefer edge axis only if it's meaningfully better than the best face axis.
            let biased_overlap = overlap + OVERLAP_EPS;
            if biased_overlap < best_overlap && biased_overlap < face_best {
                best_overlap = overlap;
                best_axis = axis;
                best_category = MinAxis::EdgeEdge { a_idx: i, b_idx: j };
            }
        }
    }

    if best_overlap > f32::MAX * 0.5 {
        cache.separating_axis = best_separating_axis.map(|(axis, _)| axis);
        return ContactManifold::empty();
    }

    // Pair is colliding — clear the cache.
    cache.separating_axis = None;

    let normal = best_axis.normalize();
    let geometric_depth = best_overlap - 2.0 * contact_margin;

    match best_category {
        MinAxis::FaceA(_) | MinAxis::FaceB(_) => {
            let manifold = face_face_contacts(
                a,
                b,
                &axes_a,
                &axes_b,
                normal,
                contact_margin,
                &best_category,
            );
            if manifold.is_empty() {
                if let Some((a_idx, b_idx, edge_axis, edge_overlap)) = best_edge_candidate {
                    // Fallback for near-degenerate face clipping: preserve contact when SAT
                    // says overlap exists but polygon clipping produced no points.
                    return edge_edge_contact(
                        a,
                        b,
                        &axes_a,
                        &axes_b,
                        a_idx,
                        b_idx,
                        edge_axis.normalize(),
                        edge_overlap - 2.0 * contact_margin,
                    );
                }
            }
            manifold
        }
        MinAxis::EdgeEdge { a_idx, b_idx } => edge_edge_contact(
            a,
            b,
            &axes_a,
            &axes_b,
            a_idx,
            b_idx,
            normal,
            geometric_depth,
        ),
    }
}

/// Test a single face axis for SAT overlap.
fn test_face_axis(
    raw_axis: &Vector3<f32>,
    a: &Obb,
    b: &Obb,
    center_dir: &Vector3<f32>,
    contact_margin: f32,
) -> Option<(Vector3<f32>, f32)> {
    let len_sq = raw_axis.magnitude_squared();
    if len_sq < AXIS_EPS {
        return Some((Vector3::y(), f32::MAX));
    }
    let mut axis = *raw_axis / len_sq.sqrt();
    let distance = center_dir.dot(&axis).abs();
    let overlap =
        a.project_half_extent(&axis) + b.project_half_extent(&axis) + 2.0 * contact_margin
            - distance;

    if overlap < -OVERLAP_EPS {
        return None;
    }

    if axis.dot(center_dir) < 0.0 {
        axis = -axis;
    }

    Some((axis, overlap))
}

/// Generate face-face contacts via Sutherland-Hodgman clipping.
fn face_face_contacts(
    a: &Obb,
    b: &Obb,
    axes_a: &[Vector3<f32>; 3],
    axes_b: &[Vector3<f32>; 3],
    normal: Vector3<f32>,
    contact_margin: f32,
    category: &MinAxis,
) -> ContactManifold {
    // Determine reference and incident faces.
    let (ref_obb, inc_obb, ref_axes, inc_axes, flip_normal) = match category {
        MinAxis::FaceA(_) => (a, b, axes_a, axes_b, false),
        MinAxis::FaceB(_) => (b, a, axes_b, axes_a, true),
        _ => unreachable!(),
    };

    let ref_normal = if flip_normal { -normal } else { normal };

    // Find the reference face: the face of ref_obb most aligned with ref_normal.
    let (ref_axis_idx, ref_axis_sign) = find_most_aligned_face(ref_axes, ref_normal);
    let ref_face = obb_face(ref_obb, ref_axis_idx, ref_axis_sign);

    // Find the incident face: the face of inc_obb most opposed to ref_normal.
    let (inc_axis_idx, inc_axis_sign) = find_most_opposed_face(inc_axes, ref_normal);
    let inc_face = obb_face(inc_obb, inc_axis_idx, inc_axis_sign);

    // Clip incident face against reference face side planes.
    let clipped = ref_face.clip_against_sides(&inc_face.vertices);

    // Project clipped points onto the reference plane, keep those behind it.
    let mut points = smallvec::SmallVec::<[ContactPoint; 4]>::new();
    let ref_face_idx = (ref_axis_idx as u32) * 2 + if ref_axis_sign > 0.0 { 0 } else { 1 };
    let inc_face_idx = (inc_axis_idx as u32) * 2 + if inc_axis_sign > 0.0 { 0 } else { 1 };

    // Accept points that are behind the reference face (penetrating) or within
    // the margin skin (speculative). Points within margin get raw_depth < 0.
    let margin_tolerance = 2.0 * contact_margin + OVERLAP_EPS;
    for p in clipped {
        let signed_dist = (p - ref_face.center).dot(&ref_face.normal);
        if signed_dist <= margin_tolerance {
            let raw_depth = -signed_dist;
            let feature_id = if flip_normal {
                FeatureId::from_face_pair(inc_face_idx, ref_face_idx)
            } else {
                FeatureId::from_face_pair(ref_face_idx, inc_face_idx)
            };
            points.push(ContactPoint::new(p, normal, raw_depth, feature_id));
        }
    }

    // Reduce to 4 contacts if needed.
    if points.len() > 4 {
        reduce_to_four(&mut points);
    }

    sort_contacts_deterministic(&mut points);

    ContactManifold::from_vec(points)
}

/// Generate a single edge-edge contact point.
fn edge_edge_contact(
    a: &Obb,
    b: &Obb,
    axes_a: &[Vector3<f32>; 3],
    axes_b: &[Vector3<f32>; 3],
    a_axis_idx: usize,
    b_axis_idx: usize,
    normal: Vector3<f32>,
    geometric_depth: f32,
) -> ContactManifold {
    // For each tested axis pair, there are 4 candidate edges on each OBB.
    // Search all 16 combinations and keep the closest segment pair.
    let ((_, _, edge_a_variant), (_, _, edge_b_variant), pa, pb) =
        find_best_edge_pair(a, axes_a, a_axis_idx, b, axes_b, b_axis_idx);
    let point = Point3::from((pa.coords + pb.coords) * 0.5);

    let feature_id = FeatureId::from_edge_pair(
        edge_index(a_axis_idx, edge_a_variant),
        edge_index(b_axis_idx, edge_b_variant),
    );

    ContactManifold::single(ContactPoint::new(
        point,
        normal,
        geometric_depth,
        feature_id,
    ))
}

/// Find the most aligned face of an OBB to a given direction.
fn find_most_aligned_face(axes: &[Vector3<f32>; 3], direction: Vector3<f32>) -> (usize, f32) {
    let mut best_idx = 0;
    let mut best_dot = 0.0f32;
    let mut best_sign = 1.0f32;
    for i in 0..3 {
        let dot = direction.dot(&axes[i]);
        if dot.abs() > best_dot.abs() {
            best_dot = dot;
            best_idx = i;
            best_sign = if dot >= 0.0 { 1.0 } else { -1.0 };
        }
    }
    (best_idx, best_sign)
}

/// Find the most opposed face of an OBB to a given direction.
fn find_most_opposed_face(axes: &[Vector3<f32>; 3], direction: Vector3<f32>) -> (usize, f32) {
    let mut best_idx = 0;
    let mut best_dot = 0.0f32;
    let mut best_sign = 1.0f32;
    for i in 0..3 {
        let dot = direction.dot(&axes[i]);
        if dot.abs() > best_dot.abs() {
            best_dot = dot;
            best_idx = i;
            // Incident face: opposite to the reference normal.
            best_sign = if dot >= 0.0 { -1.0 } else { 1.0 };
        }
    }
    (best_idx, best_sign)
}

/// Build one of the 4 edge variants for the selected OBB axis.
///
/// Returns `(start, end, variant_index)`.
fn edge_variant(
    obb: &Obb,
    axes: &[Vector3<f32>; 3],
    axis_idx: usize,
    variant: usize,
) -> (Point3<f32>, Point3<f32>, usize) {
    let (perp0, perp1) = match axis_idx {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };

    let edge_dir = axes[axis_idx] * obb.half_extents[axis_idx];
    let he0 = obb.half_extents[perp0];
    let he1 = obb.half_extents[perp1];

    // The 4 edges differ by sign of perpendicular axes.
    let signs: [(f32, f32); 4] = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];

    let (s0, s1) = signs[variant];
    let offset = axes[perp0] * (he0 * s0) + axes[perp1] * (he1 * s1);
    let start = obb.center + offset - edge_dir;
    let end = obb.center + offset + edge_dir;

    (start, end, variant)
}

/// Find the closest edge-edge pair among the 4x4 candidates for two axis families.
///
/// Returns:
/// `((a_start, a_end, a_variant), (b_start, b_end, b_variant), point_on_a, point_on_b)`.
fn find_best_edge_pair(
    a: &Obb,
    axes_a: &[Vector3<f32>; 3],
    a_axis_idx: usize,
    b: &Obb,
    axes_b: &[Vector3<f32>; 3],
    b_axis_idx: usize,
) -> (
    (Point3<f32>, Point3<f32>, usize),
    (Point3<f32>, Point3<f32>, usize),
    Point3<f32>,
    Point3<f32>,
) {
    let mut best_a = edge_variant(a, axes_a, a_axis_idx, 0);
    let mut best_b = edge_variant(b, axes_b, b_axis_idx, 0);
    let mut best_pa = best_a.0;
    let mut best_pb = best_b.0;
    let mut best_dist_sq = f32::MAX;

    for a_variant in 0..4 {
        let edge_a = edge_variant(a, axes_a, a_axis_idx, a_variant);
        for b_variant in 0..4 {
            let edge_b = edge_variant(b, axes_b, b_axis_idx, b_variant);
            let (pa, pb) = segment_segment_closest_points(edge_a.0, edge_a.1, edge_b.0, edge_b.1);
            let dist_sq = (pa - pb).magnitude_squared();
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_a = edge_a;
                best_b = edge_b;
                best_pa = pa;
                best_pb = pb;
            }
        }
    }

    (best_a, best_b, best_pa, best_pb)
}

/// Encode an edge variant as a unique edge index for FeatureId.
///
/// There are 12 edges total (4 per axis). edge_index = axis * 4 + variant.
fn edge_index(axis: usize, variant: usize) -> u32 {
    (axis * 4 + variant) as u32
}

/// Deterministically order contact points for solver stability.
fn sort_contacts_deterministic(points: &mut smallvec::SmallVec<[ContactPoint; 4]>) {
    const POSITION_QUANT: f32 = 10_000.0;
    let quantize = |v: f32| (v * POSITION_QUANT).round() as i32;

    points.sort_by(|a, b| {
        a.feature_id
            .0
            .cmp(&b.feature_id.0)
            .then_with(|| b.raw_depth.total_cmp(&a.raw_depth))
            .then_with(|| quantize(a.point.x).cmp(&quantize(b.point.x)))
            .then_with(|| quantize(a.point.y).cmp(&quantize(b.point.y)))
            .then_with(|| quantize(a.point.z).cmp(&quantize(b.point.z)))
    });
}

/// Reduce a set of contact points to at most 4 using area-maximizing selection.
fn reduce_to_four(points: &mut smallvec::SmallVec<[ContactPoint; 4]>) {
    if points.len() <= 4 {
        return;
    }

    // 1. Deepest.
    let mut selected = [0usize; 4];
    let mut best_depth = -f32::MAX;
    for (i, p) in points.iter().enumerate() {
        if p.raw_depth > best_depth {
            best_depth = p.raw_depth;
            selected[0] = i;
        }
    }

    // 2. Farthest from first.
    let mut best_dist_sq = 0.0f32;
    selected[1] = selected[0];
    for (i, p) in points.iter().enumerate() {
        if i == selected[0] {
            continue;
        }
        let d = (p.point - points[selected[0]].point).magnitude_squared();
        if d > best_dist_sq {
            best_dist_sq = d;
            selected[1] = i;
        }
    }

    // 3. Maximizes triangle area with first two.
    let mut best_area = 0.0f32;
    selected[2] = selected[0];
    let p0 = points[selected[0]].point;
    let p1 = points[selected[1]].point;
    for (i, p) in points.iter().enumerate() {
        if i == selected[0] || i == selected[1] {
            continue;
        }
        let area = (p1 - p0).cross(&(p.point - p0)).magnitude_squared();
        if area > best_area {
            best_area = area;
            selected[2] = i;
        }
    }

    // 4. Maximizes minimum distance to selected set.
    let mut best_spread = 0.0f32;
    selected[3] = selected[0];
    for (i, p) in points.iter().enumerate() {
        if i == selected[0] || i == selected[1] || i == selected[2] {
            continue;
        }
        let min_d = selected[..3]
            .iter()
            .map(|&s| (p.point - points[s].point).magnitude_squared())
            .fold(f32::INFINITY, f32::min);
        if min_d > best_spread {
            best_spread = min_d;
            selected[3] = i;
        }
    }

    // Deduplicate selected indices and collect.
    let mut unique = smallvec::SmallVec::<[usize; 4]>::new();
    for &idx in &selected {
        if !unique.contains(&idx) {
            unique.push(idx);
        }
    }

    let kept: smallvec::SmallVec<[ContactPoint; 4]> = unique.iter().map(|&i| points[i]).collect();
    *points = kept;
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn unit_box_at(pos: Point3<f32>) -> Obb {
        Obb::new(pos, UnitQuaternion::identity(), Vector3::new(1.0, 1.0, 1.0))
    }

    fn vertical_rod_at(pos: Point3<f32>) -> Obb {
        Obb::new(pos, UnitQuaternion::identity(), Vector3::new(0.1, 2.0, 0.1))
    }

    #[test]
    fn face_to_face_contact() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(1.5, 0.0, 0.0));
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(!m.is_empty());
        for cp in &m.points {
            assert!(
                cp.normal.x > 0.9,
                "Normal should point +X, got {:?}",
                cp.normal
            );
            assert!(
                (cp.raw_depth - 0.5).abs() < 0.1,
                "Expected ~0.5 depth, got {}",
                cp.raw_depth
            );
        }
    }

    #[test]
    fn separated_boxes() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(5.0, 0.0, 0.0));
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(m.is_empty());
    }

    #[test]
    fn stacked_boxes() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(0.0, 1.8, 0.0));
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(!m.is_empty());
        for cp in &m.points {
            assert!(cp.normal.y > 0.9, "Stacked normal should point +Y");
        }
    }

    #[test]
    fn rotated_box_contact() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4);
        let b = Obb::new(Point3::new(2.0, 0.0, 0.0), rot, Vector3::new(1.0, 1.0, 1.0));
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(!m.is_empty(), "Rotated box should overlap");
    }

    #[test]
    fn max_four_contacts() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(0.0, 1.5, 0.0));
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(
            m.len() <= 4,
            "Should reduce to at most 4 contacts, got {}",
            m.len()
        );
    }

    #[test]
    fn axis_aligned_stacking_four_contacts() {
        // Two identical axis-aligned boxes stacked → should produce 4 face-face contacts.
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(0.0, 1.9, 0.0));
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert_eq!(m.len(), 4, "Expected 4 face-face contacts, got {}", m.len());
        for cp in &m.points {
            assert!(cp.normal.y > 0.9);
        }
    }

    #[test]
    fn edge_edge_contact() {
        // Two OBBs rotated so only edges overlap (no face-face contact).
        // Rotate A 45° around Y, B 45° around Y in the opposite direction.
        let rot_a =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4);
        let rot_b =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -std::f32::consts::FRAC_PI_4);
        let a = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            rot_a,
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(2.7, 0.0, 0.0),
            rot_b,
            Vector3::new(1.0, 1.0, 1.0),
        );
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(!m.is_empty(), "Edge-edge should produce contact");
    }

    #[test]
    fn margin_catches_separated_boxes() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(2.05, 0.0, 0.0));
        // Without margin: separated.
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(m.is_empty());
        // With margin: caught.
        let m = obb_obb_manifold(&a, &b, 0.1);
        assert!(!m.is_empty());
        for cp in &m.points {
            assert!(cp.raw_depth < 0.0, "Should be margin-only contact");
            assert_eq!(cp.depth, 0.0);
        }
    }

    #[test]
    fn feature_ids_are_stable() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(1.5, 0.0, 0.0));
        let m1 = obb_obb_manifold(&a, &b, 0.0);
        let m2 = obb_obb_manifold(&a, &b, 0.0);
        assert_eq!(m1.len(), m2.len());
        for (a, b) in m1.points.iter().zip(m2.points.iter()) {
            assert_eq!(a.feature_id, b.feature_id);
        }
    }

    #[test]
    fn symmetric_result() {
        // obb_obb_manifold(a, b) and obb_obb_manifold(b, a) should give consistent results.
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(1.5, 0.3, 0.1));
        let m_ab = obb_obb_manifold(&a, &b, 0.0);
        let m_ba = obb_obb_manifold(&b, &a, 0.0);
        assert_eq!(m_ab.len(), m_ba.len());
        // Normals should be opposite.
        if !m_ab.is_empty() {
            let n_ab = m_ab.points[0].normal;
            let n_ba = m_ba.points[0].normal;
            assert!(
                (n_ab + n_ba).magnitude() < 0.1,
                "Normals should be opposite: {:?} vs {:?}",
                n_ab,
                n_ba,
            );
        }
    }

    #[test]
    fn penetrating_both_faces() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = vertical_rod_at(Point3::new(1.0, 0.0, 0.0));

        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(!m.is_empty());
        for cp in &m.points {
            assert!(cp.normal.x > 0.9);
        }
    }

    fn point_inside_obb(obb: &Obb, point: Point3<f32>, tolerance: f32) -> bool {
        let d = point - obb.center;
        let axes = obb.axes();
        (d.dot(&axes[0])).abs() <= obb.half_extents[0] + tolerance
            && (d.dot(&axes[1])).abs() <= obb.half_extents[1] + tolerance
            && (d.dot(&axes[2])).abs() <= obb.half_extents[2] + tolerance
    }

    #[test]
    fn rotated_overlap_contacts_stay_on_both_boxes() {
        let rot_a =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4);
        let rot_b =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -std::f32::consts::FRAC_PI_4);
        let a = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            rot_a,
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(2.7, 0.0, 0.0),
            rot_b,
            Vector3::new(1.0, 1.0, 1.0),
        );
        let m = obb_obb_manifold(&a, &b, 0.0);
        assert!(!m.is_empty(), "Expected rotated overlap contact");
        for cp in &m.points {
            assert!(
                point_inside_obb(&a, cp.point, 1e-3),
                "Point should stay within OBB A bounds: {:?}",
                cp.point
            );
            assert!(
                point_inside_obb(&b, cp.point, 1e-3),
                "Point should stay within OBB B bounds: {:?}",
                cp.point
            );
        }
    }

    #[test]
    fn edge_intersection_cases_generate_contacts() {
        let cases = [
            (
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.65),
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -0.8),
                Point3::new(2.45, 0.05, 0.0),
            ),
            (
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.5),
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.6),
                Point3::new(0.1, 2.35, 0.15),
            ),
            (
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.55),
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -0.5),
                Point3::new(0.2, 0.1, 2.3),
            ),
        ];

        for (rot_a, rot_b, center_b) in cases {
            let a = Obb::new(
                Point3::new(0.0, 0.0, 0.0),
                rot_a,
                Vector3::new(1.0, 1.0, 1.0),
            );
            let b = Obb::new(center_b, rot_b, Vector3::new(1.0, 1.0, 1.0));
            let m = obb_obb_manifold(&a, &b, 0.0);
            assert!(
                !m.is_empty(),
                "Expected contact for case center={:?}, rot_a={:?}, rot_b={:?}",
                center_b,
                rot_a,
                rot_b
            );
        }
    }

    #[test]
    fn repeated_calls_return_stable_contact_ordering() {
        let rot_a = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.65);
        let rot_b = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -0.55)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.35);
        let a = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            rot_a,
            Vector3::new(1.0, 1.0, 1.0),
        );
        let b = Obb::new(
            Point3::new(1.9, 0.25, 0.15),
            rot_b,
            Vector3::new(1.0, 1.0, 1.0),
        );

        let m1 = obb_obb_manifold(&a, &b, 0.0);
        let m2 = obb_obb_manifold(&a, &b, 0.0);
        assert_eq!(m1.len(), m2.len(), "Contact count should be deterministic");

        for (cp1, cp2) in m1.points.iter().zip(m2.points.iter()) {
            assert_eq!(
                cp1.feature_id, cp2.feature_id,
                "Feature ordering should be stable"
            );
            assert!(
                (cp1.point - cp2.point).magnitude() <= 1e-6,
                "Point ordering/value should be stable: {:?} vs {:?}",
                cp1.point,
                cp2.point
            );
        }
    }

    // ── SAT cache tests ────────────────────────────────────────────────

    #[test]
    fn sat_cache_early_out_for_separated_pair() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b = unit_box_at(Point3::new(5.0, 0.0, 0.0));

        // First call: no cache, pair is separated. Should populate the cache.
        let mut cache = SatCache::new();
        let m = obb_obb_manifold_cached(&a, &b, 0.0, &mut cache);
        assert!(m.is_empty());
        assert!(
            cache.separating_axis.is_some(),
            "Cache should store a separating axis for separated pairs"
        );

        // Second call with cache: should early-out on the cached axis.
        let m2 = obb_obb_manifold_cached(&a, &b, 0.0, &mut cache);
        assert!(m2.is_empty());
        assert!(cache.separating_axis.is_some());
    }

    #[test]
    fn sat_cache_invalidated_when_pair_collides() {
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let b_far = unit_box_at(Point3::new(5.0, 0.0, 0.0));
        let b_close = unit_box_at(Point3::new(1.5, 0.0, 0.0));

        // Build cache with separated pair.
        let mut cache = SatCache::new();
        let _ = obb_obb_manifold_cached(&a, &b_far, 0.0, &mut cache);
        assert!(cache.separating_axis.is_some());

        // Now test colliding pair with stale cache — must still detect collision.
        let m = obb_obb_manifold_cached(&a, &b_close, 0.0, &mut cache);
        assert!(!m.is_empty(), "Must detect collision even with stale cache");
        assert!(
            cache.separating_axis.is_none(),
            "Cache should be cleared when pair is colliding"
        );
    }

    #[test]
    fn sat_cache_produces_same_results_as_uncached() {
        // Verify cached and uncached paths produce identical manifolds across
        // a sweep of random configurations.
        let mut state: u32 = 0xDEADBEEF;
        let mut next_f32 = || {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            (state as f32) / (u32::MAX as f32)
        };
        let angle = |u: f32| (u * 2.0 - 1.0) * std::f32::consts::PI;
        let span = |u: f32, r: f32| (u * 2.0 - 1.0) * r;

        for _ in 0..256 {
            let rot_a = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle(next_f32()))
                * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle(next_f32()));
            let rot_b = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle(next_f32()))
                * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle(next_f32()));
            let a = Obb::new(Point3::origin(), rot_a, Vector3::new(1.0, 1.0, 1.0));
            let b = Obb::new(
                Point3::new(span(next_f32(), 3.0), span(next_f32(), 3.0), span(next_f32(), 3.0)),
                rot_b,
                Vector3::new(1.0, 1.0, 1.0),
            );

            let uncached = obb_obb_manifold(&a, &b, 0.02);

            let mut cache = SatCache::new();
            let cached = obb_obb_manifold_cached(&a, &b, 0.02, &mut cache);

            assert_eq!(
                uncached.len(),
                cached.len(),
                "Cached and uncached contact counts must match"
            );
            for (uc, cc) in uncached.points.iter().zip(cached.points.iter()) {
                assert_eq!(uc.feature_id, cc.feature_id);
                assert!(
                    (uc.raw_depth - cc.raw_depth).abs() < 1e-6,
                    "Depth mismatch: {} vs {}",
                    uc.raw_depth,
                    cc.raw_depth
                );
            }
        }
    }

    #[test]
    fn sat_cache_axis_stays_valid_across_small_movements() {
        // Simulate a pair drifting apart over several frames.
        let a = unit_box_at(Point3::new(0.0, 0.0, 0.0));
        let mut cache = SatCache::new();

        for i in 0..10 {
            let x = 3.0 + i as f32 * 0.1; // moving further apart
            let b = unit_box_at(Point3::new(x, 0.0, 0.0));
            let m = obb_obb_manifold_cached(&a, &b, 0.0, &mut cache);
            assert!(m.is_empty(), "Should be separated at x={x}");
            assert!(cache.separating_axis.is_some());
        }
    }

    #[test]
    fn sat_cache_throughput_separated_pairs() {
        // Micro-benchmark: measure cached vs uncached throughput for separated
        // pairs. Uses rotated boxes so the separating axis is NOT the first face
        // normal tested — this is where caching helps most.
        let rot_a = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.7)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.4);
        let rot_b = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.6)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3);
        let a = Obb::new(Point3::origin(), rot_a, Vector3::new(1.0, 1.0, 1.0));
        let b = Obb::new(Point3::new(3.5, 0.5, 0.2), rot_b, Vector3::new(1.0, 1.0, 1.0));

        // Verify they're actually separated.
        assert!(obb_obb_manifold(&a, &b, 0.0).is_empty());

        let iterations = 10_000;

        // Warm up the cache.
        let mut cache = SatCache::new();
        let _ = obb_obb_manifold_cached(&a, &b, 0.0, &mut cache);

        // Uncached path.
        let t0 = std::time::Instant::now();
        for _ in 0..iterations {
            let m = obb_obb_manifold(&a, &b, 0.0);
            std::hint::black_box(&m);
        }
        let uncached_ns = t0.elapsed().as_nanos();

        // Cached path.
        let t1 = std::time::Instant::now();
        for _ in 0..iterations {
            let m = obb_obb_manifold_cached(&a, &b, 0.0, &mut cache);
            std::hint::black_box(&m);
        }
        let cached_ns = t1.elapsed().as_nanos();

        let ratio = uncached_ns as f64 / cached_ns.max(1) as f64;
        eprintln!(
            "SAT cache throughput: uncached={uncached_ns}ns, cached={cached_ns}ns, \
             speedup={ratio:.2}x over {iterations} iterations"
        );

        // Cached should be faster. In release mode the gain is smaller since
        // the uncached path is already heavily optimized by LLVM.
        assert!(
            ratio > 1.1,
            "Cached path should be faster: speedup was only {ratio:.2}x"
        );
    }

    #[test]
    fn obb_obb_colliding_throughput() {
        // Measures manifold generation for overlapping OBBs (exercises the
        // full clipping path: find faces → clip_against_sides → project).
        let rot_a = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.3)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.15);
        let rot_b = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -0.4)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let a = Obb::new(Point3::origin(), rot_a, Vector3::new(1.0, 0.8, 1.2));
        let b = Obb::new(
            Point3::new(1.6, 0.1, 0.05),
            rot_b,
            Vector3::new(0.9, 1.0, 0.7),
        );

        // Verify they actually collide.
        assert!(!obb_obb_manifold(&a, &b, 0.02).is_empty());

        let iterations = 100_000;

        let t0 = std::time::Instant::now();
        for _ in 0..iterations {
            let m = obb_obb_manifold(&a, &b, 0.02);
            std::hint::black_box(&m);
        }
        let elapsed = t0.elapsed();

        let ns_per_call = elapsed.as_nanos() / iterations as u128;
        eprintln!(
            "obb_obb_manifold (colliding) throughput: {ns_per_call}ns/call \
             ({iterations} iterations in {:.1}ms)",
            elapsed.as_secs_f64() * 1000.0
        );
    }

    #[test]
    fn seeded_sweep_preserves_contact_invariants() {
        let mut state: u32 = 0xC0FFEE;
        let mut next_f32 = || {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            (state as f32) / (u32::MAX as f32)
        };
        let angle = |u: f32| (u * 2.0 - 1.0) * std::f32::consts::PI;
        let span = |u: f32, r: f32| (u * 2.0 - 1.0) * r;

        for _ in 0..256 {
            let rot_a = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle(next_f32()))
                * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle(next_f32()))
                * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle(next_f32()));
            let rot_b = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle(next_f32()))
                * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), angle(next_f32()))
                * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle(next_f32()));

            let a = Obb::new(
                Point3::new(0.0, 0.0, 0.0),
                rot_a,
                Vector3::new(1.0, 1.0, 1.0),
            );
            let b = Obb::new(
                Point3::new(
                    span(next_f32(), 2.2),
                    span(next_f32(), 2.2),
                    span(next_f32(), 2.2),
                ),
                rot_b,
                Vector3::new(1.0, 1.0, 1.0),
            );

            let m1 = obb_obb_manifold(&a, &b, 0.0);
            let m2 = obb_obb_manifold(&a, &b, 0.0);
            assert_eq!(m1.len(), m2.len(), "Contact count should be repeatable");
            assert!(m1.len() <= 4, "Manifold must contain at most 4 points");

            for (cp1, cp2) in m1.points.iter().zip(m2.points.iter()) {
                assert_eq!(
                    cp1.feature_id, cp2.feature_id,
                    "Feature ordering should be repeatable"
                );
            }

            for cp in &m1.points {
                assert!(
                    cp.point.x.is_finite() && cp.point.y.is_finite() && cp.point.z.is_finite(),
                    "Sweep point should be finite: {:?}",
                    cp.point
                );
                assert!(
                    cp.normal.x.is_finite() && cp.normal.y.is_finite() && cp.normal.z.is_finite(),
                    "Sweep normal should be finite: {:?}",
                    cp.normal
                );
                assert!(
                    (cp.normal.magnitude() - 1.0).abs() <= 1e-3,
                    "Sweep normal should be unit length: {:?}",
                    cp.normal
                );
                assert!(
                    cp.raw_depth >= -2.0 * OVERLAP_EPS,
                    "Raw depth should be non-negative for margin=0 (within epsilon), got {}",
                    cp.raw_depth
                );
            }
        }
    }
}
