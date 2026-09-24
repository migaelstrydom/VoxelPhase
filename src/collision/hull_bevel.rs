//! Cutting the edges off a convex hull.
//!
//! A polished surface shows almost nothing on a flat face, so a glossy solid
//! with square edges reads as a grey slab: the highlight lives on the narrow
//! facets between the faces, which catch the key light at whatever angle the
//! solid has tumbled to. Ice blocks are authored that way and are drawn from a
//! mesh built to match — but the moment a block is cleaved it is drawn from
//! its *hulls* instead, and a hull cut by a plane has square edges. The block
//! shed its chamfer at the one instant the player was looking at it.
//!
//! The chamfer is a sequence of half-space cuts, one per edge and one per
//! vertex, all measured off the original solid and applied to whatever is left
//! of it:
//!
//! ```text
//!   ConvexHull ──▶ bevel_planes ──┬──▶ 1 per edge   (the facet between faces)
//!                                 └──▶ 1 per vertex (the facet between facets)
//!                       │
//!                       ▼
//!          HullDraft::trim, in turn ──▶ build once ──▶ ConvexHull, chamfered
//! ```
//!
//! The cuts are made on a [`HullDraft`] and the hull is built once at the end:
//! building is most of the cost of a cut, and a wedge takes twenty or so.
//!
//! For drawing, not for colliding. The chamfered solid is strictly smaller
//! than the one it came from, so a piece drawn with it still sits inside its
//! collider — the same bargain the authored ice block makes, where the mesh is
//! bevelled and the collider is the full box.

use nalgebra::Vector3;

use super::convex_hull::{ConvexHull, HullEdgeAdj};
use super::hull_split::{HullDraft, Plane};

/// Shortest edge, as a fraction of the hull's smallest bounding dimension,
/// that is worth chamfering. Below this the facet is narrower than the
/// vertex-welding tolerance the hull builder works to, and the cut is refused
/// anyway; skipping it early keeps the sliver out of the builder.
const MIN_EDGE_FRACTION: f32 = 0.01;

/// Sharpest corner of a face that is worth insetting, as the sine of half its
/// angle. A needle-sharp corner insets to a point a long way off — the inset
/// runs as `1 / sin` — and a facet laid through it would take most of the
/// solid with it.
const MIN_CORNER_SIN: f32 = 0.05;

/// Cut every edge and corner of `hull` back by `bevel`.
///
/// `bevel` is measured across each face, so two faces meeting at a right angle
/// each lose a band that wide. Cuts the builder refuses — a facet that would
/// be wider than the piece it is cut from, one that collapses a face — are
/// skipped, leaving that edge sharp rather than losing the piece: a solid with
/// one square edge is a worse drawing than a chamfered one and a far better
/// one than nothing.
pub fn bevel_hull(hull: &ConvexHull, bevel: f32) -> ConvexHull {
    bevel_hull_by_face(hull, &vec![bevel; hull.faces.len()])
}

/// The same, with a chamfer width chosen per face.
///
/// An edge is cut back by the narrower of the two faces it joins, and a corner
/// by the narrowest of the faces meeting there, so a face asking for no
/// chamfer keeps every one of its edges square. That is what lets an object
/// chamfer the faces it was authored with and leave the faces it was broken
/// along alone — a fracture surface that catches the light as brightly as the
/// polished ones reads as a gap, not a crack.
///
/// `widths` is indexed in step with `hull.faces`; a short list leaves the
/// remaining faces unchamfered.
pub fn bevel_hull_by_face(hull: &ConvexHull, widths: &[f32]) -> ConvexHull {
    let widest = widths.iter().copied().fold(
        0.0f32,
        |acc, w| if w.is_finite() { acc.max(w) } else { acc },
    );
    if widest <= 0.0 {
        return hull.clone();
    }

    // Cut at unit scale and scale back. The cutter judges a face by the span
    // it covers and a piece by how thin it is, both as absolute distances,
    // because both guard a *collider* — and a collider that small is one the
    // engine cannot hold whatever it was cut from. A chamfer is a drawing, and
    // the only sensible reading of "too small" for one is relative to the
    // solid: without this the corner facets of a block 30cm across came out
    // below the cutter's floor and were dropped, while the same block drawn a
    // metre across kept them.
    let scale = extent_of(hull);
    if scale <= 0.0 || !scale.is_finite() {
        return hull.clone();
    }

    let unit: Vec<f32> = (0..hull.faces.len())
        .map(|face| widths.get(face).copied().unwrap_or(0.0).max(0.0) / scale)
        .collect();

    let unit_hull = hull.scaled(1.0 / scale);
    let mut chamfered = HullDraft::of(&unit_hull);
    for plane in bevel_planes(&unit_hull, &unit) {
        if let Some(trimmed) = chamfered.trim(plane) {
            chamfered = trimmed;
        }
    }
    chamfered.build().scaled(scale)
}

/// The half-space that cuts each edge and each corner of `hull` back by
/// `bevel`.
///
/// Every plane is measured off `hull` itself rather than off the partly
/// chamfered solid, which is what makes the result independent of the order
/// the cuts are applied in: the chamfered solid is the intersection of the
/// hull with all of them.
fn bevel_planes(hull: &ConvexHull, widths: &[f32]) -> Vec<Plane> {
    let centre = hull.centroid();
    let mut planes = Vec::with_capacity(hull.edges.len() + hull.vertices.len());
    let by_face = FaceWidths::of(hull, widths);

    let shortest = extent_of(hull) * MIN_EDGE_FRACTION;
    for edge in &hull.edges {
        let bevel = by_face.at_edge(edge);
        if bevel <= 0.0 {
            continue;
        }
        let (a, b) = (
            hull.vertices[edge.v0 as usize],
            hull.vertices[edge.v1 as usize],
        );
        let along = b - a;
        if along.magnitude() <= shortest {
            continue;
        }
        let Some(normal) = unit(edge.normal_a + edge.normal_b) else {
            continue;
        };
        // The direction across face A, away from this edge and into the face.
        // The facet has to start `bevel` along it, which is what "cut back by
        // `bevel`" means; the same point comes out of face B, because the
        // facet's normal is the bisector and sits at the same angle to both.
        let Some(across) = unit(along.cross(&edge.normal_a)) else {
            continue;
        };
        let across = if across.dot(&(centre - a)) < 0.0 {
            -across
        } else {
            across
        };
        planes.push(Plane::through(a + across * bevel, normal));
    }

    // And one per corner, between the facets the edge cuts leave. Without
    // these a chamfered box has eight sharp points where it used to have eight
    // triangles — the facets that catch the parts of the sky the edges miss.
    planes.extend(corner_planes(hull, &by_face));

    planes
}

/// The chamfer width of each face, and the widths that follow from it for the
/// edges and corners between them.
struct FaceWidths {
    /// Width per face, indexed in step with the hull's faces.
    per_face: Vec<f32>,
    /// Width per vertex: the narrowest of the faces meeting there.
    per_vertex: Vec<f32>,
    /// Which faces carry each normal, so an edge can find its own two.
    normals: Vec<Vector3<f32>>,
}

impl FaceWidths {
    fn of(hull: &ConvexHull, widths: &[f32]) -> Self {
        let per_face: Vec<f32> = (0..hull.faces.len())
            .map(|face| widths.get(face).copied().unwrap_or(0.0).max(0.0))
            .collect();
        let mut per_vertex = vec![f32::INFINITY; hull.vertices.len()];
        for (face, width) in hull.faces.iter().zip(&per_face) {
            for &index in &face.vertex_indices {
                per_vertex[index as usize] = per_vertex[index as usize].min(*width);
            }
        }
        Self {
            per_face,
            per_vertex,
            normals: hull.faces.iter().map(|f| f.normal).collect(),
        }
    }

    /// The narrower of the two faces the edge joins.
    ///
    /// Matched by normal, because [`HullEdgeAdj`] carries its neighbours'
    /// normals rather than their indices.
    fn at_edge(&self, edge: &HullEdgeAdj) -> f32 {
        self.for_normal(edge.normal_a)
            .min(self.for_normal(edge.normal_b))
    }

    fn for_normal(&self, normal: Vector3<f32>) -> f32 {
        self.normals
            .iter()
            .position(|n| (n - normal).magnitude() < 1e-4)
            .map_or(0.0, |face| self.per_face[face])
    }

    /// The narrowest of the faces meeting at a vertex.
    fn at_vertex(&self, index: usize) -> f32 {
        self.per_vertex
            .get(index)
            .copied()
            .filter(|w| w.is_finite())
            .unwrap_or(0.0)
    }
}

/// The half-space that cuts each corner of `hull` back by `bevel`.
///
/// A corner facet is laid where the chamfered faces around it already end: it
/// passes through each incident face's *inset corner*, the point in that face
/// `bevel` from both of its edges at the vertex. Cutting at a flat distance
/// from the vertex instead would work for a right-angled corner and leave a
/// sliver or nothing at all everywhere else — which for a narrow chamfer is
/// nothing at all, because the facet comes out smaller than the hull builder
/// will accept as a face.
///
/// The inset corners of a vertex with more than three faces need not be
/// coplanar, so the facet is laid through their mean along the vertex's own
/// outward direction. A cube corner, where they are coplanar, gets exactly the
/// facet a bevelled box is authored with.
fn corner_planes(hull: &ConvexHull, widths: &FaceWidths) -> Vec<Plane> {
    let mut normals = vec![Vector3::zeros(); hull.vertices.len()];
    let mut insets = vec![Vector3::zeros(); hull.vertices.len()];
    let mut counts = vec![0u32; hull.vertices.len()];

    for face in &hull.faces {
        let corners = &face.vertex_indices;
        for (position, &index) in corners.iter().enumerate() {
            let index = index as usize;
            normals[index] += face.normal;
            let vertex = hull.vertices[index];
            let before =
                hull.vertices[corners[(position + corners.len() - 1) % corners.len()] as usize];
            let after = hull.vertices[corners[(position + 1) % corners.len()] as usize];
            let (Some(back), Some(forth)) = (unit(before - vertex), unit(after - vertex)) else {
                continue;
            };
            let Some(bisector) = unit(back + forth) else {
                continue;
            };
            // Both edges of the face at this vertex, moved `bevel` into the
            // face, cross on the bisector this far along it.
            let half_angle_sin = ((1.0 - back.dot(&forth)) * 0.5).max(0.0).sqrt();
            if half_angle_sin < MIN_CORNER_SIN {
                continue;
            }
            let bevel = widths.at_vertex(index);
            if bevel <= 0.0 {
                continue;
            }
            insets[index] += vertex + bisector * (bevel / half_angle_sin);
            counts[index] += 1;
        }
    }

    (0..hull.vertices.len())
        .filter(|&index| counts[index] > 0)
        .filter_map(|index| {
            let normal = unit(normals[index])?;
            Some(Plane::through(insets[index] / counts[index] as f32, normal))
        })
        .collect()
}

/// The smallest dimension of the hull's bounding box, the measure a bevel on
/// an unknown solid is sized against.
pub fn extent_of(hull: &ConvexHull) -> f32 {
    let mut low = Vector3::repeat(f32::INFINITY);
    let mut high = Vector3::repeat(f32::NEG_INFINITY);
    for vertex in &hull.vertices {
        low = low.inf(vertex);
        high = high.sup(vertex);
    }
    (high - low).min()
}

/// `v` normalized, or `None` if it is too short to have a direction.
fn unit(v: Vector3<f32>) -> Option<Vector3<f32>> {
    let length = v.magnitude();
    (length > 1e-6).then(|| v / length)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::convex_hull::cube_hull;
    use crate::collision::hull_split::{split_hull, Plane};

    fn cube() -> ConvexHull {
        cube_hull(Vector3::new(0.5, 0.5, 0.5))
    }

    fn slab() -> ConvexHull {
        cube_hull(Vector3::new(1.2, 0.25, 0.6))
    }

    /// The whole point of the exercise: a chamfered box has a facet on every
    /// edge and every corner, which is where a polished surface shows its
    /// highlight. Six faces, twelve edge facets, eight corner facets.
    #[test]
    fn a_chamfered_box_has_a_facet_on_every_edge_and_corner() {
        for hull in [cube(), slab()] {
            let bevelled = bevel_hull(&hull, extent_of(&hull) * 0.14);
            assert_eq!(
                bevelled.faces.len(),
                6 + 12 + 8,
                "faces: {:?}",
                bevelled.faces.len()
            );
        }
    }

    /// No vertex may be left where three faces of the box met, or the chamfer
    /// never happened at that corner.
    #[test]
    fn no_vertex_is_left_on_a_sharp_corner() {
        let half = Vector3::new(0.5, 0.5, 0.5);
        let bevelled = bevel_hull(&cube_hull(half), 0.07);
        for vertex in &bevelled.vertices {
            let at_extent = (0..3)
                .filter(|&axis| (vertex[axis].abs() - half[axis]).abs() < 1e-4)
                .count();
            assert!(at_extent <= 1, "vertex {vertex:?} still sits on a corner");
        }
    }

    /// The chamfered solid must fit inside the one it came from, because the
    /// collider is still the original: a drawing that pokes out of its
    /// collider visibly sinks into whatever it lands on.
    #[test]
    fn the_chamfer_only_ever_takes_material_away() {
        let hull = slab();
        let bevelled = bevel_hull(&hull, 0.03);
        for vertex in &bevelled.vertices {
            for face in &hull.faces {
                let anchor = hull.vertices[face.vertex_indices[0] as usize];
                assert!(
                    (vertex - anchor).dot(&face.normal) <= 1e-4,
                    "vertex {vertex:?} is outside the hull it was cut from"
                );
            }
        }
        assert!(bevelled.compute_volume() < hull.compute_volume());
    }

    /// And it must take away only a little. A chamfer that ate the solid would
    /// pass every test above and look like a gemstone.
    #[test]
    fn a_narrow_chamfer_costs_a_little_volume() {
        let hull = cube();
        let bevelled = bevel_hull(&hull, 0.07);
        let kept = bevelled.compute_volume() / hull.compute_volume();
        assert!((0.9..1.0).contains(&kept), "kept {kept} of the volume");
    }

    /// Asked for more chamfer than the solid can give, it must hand back
    /// something the engine can still hold rather than a sliver or a panic.
    /// The cuts that cannot be made are skipped, so the worst case is the
    /// solid it started with.
    #[test]
    fn an_absurd_chamfer_is_refused_rather_than_ruinous() {
        let hull = cube();
        for bevel in [0.4, 0.5, 1.0, 10.0] {
            let bevelled = bevel_hull(&hull, bevel);
            assert!(bevelled.compute_volume() > 0.0, "bevel {bevel}");
            assert!(bevelled.vertices.len() >= 4, "bevel {bevel}");
        }
    }

    /// The same solid chamfered by the same fraction must come out the same
    /// shape whatever size it is drawn at. The cutter's own floors are
    /// absolute distances — they guard colliders, where absolute is right —
    /// and a chamfer that inherited them silently dropped the corner facets
    /// off anything smaller than about half a metre.
    #[test]
    fn a_chamfer_is_the_same_shape_at_any_size() {
        let shape = Vector3::new(0.4, 0.25, 0.32);
        let faces: Vec<usize> = [0.05_f32, 0.5, 1.0, 8.0]
            .iter()
            .map(|scale| {
                let hull = cube_hull(shape * *scale);
                bevel_hull(&hull, extent_of(&hull) * 0.07).faces.len()
            })
            .collect();
        assert_eq!(
            faces,
            vec![6 + 12 + 8; 4],
            "facets lost with scale: {faces:?}"
        );
    }

    /// The case it exists for: the wedges a cleaved ice block comes apart
    /// into. Every one of them must chamfer without panicking the hull
    /// builder, whatever angle it was cut at.
    #[test]
    fn every_wedge_of_a_cut_block_can_be_chamfered() {
        let hull = cube_hull(Vector3::new(0.35, 0.22, 0.3));
        let mut cut = 0;
        let mut chamfered = 0;
        for salt in 0..400u32 {
            let normal = jitter_direction(salt);
            let offset = (salt % 13) as f32 / 13.0 - 0.5;
            let plane = Plane::through(normal * offset * 0.3, normal);
            let Some(halves) = split_hull(&hull, plane) else {
                continue;
            };
            for piece in [halves.front, halves.back] {
                let bevelled = bevel_hull(&piece, extent_of(&piece) * 0.14);
                assert!(bevelled.compute_volume() > 0.0);
                assert!(bevelled.compute_volume() <= piece.compute_volume() + 1e-6);
                cut += 1;
                if bevelled.faces.len() > piece.faces.len() {
                    chamfered += 1;
                }
            }
        }
        assert!(cut > 200, "only {cut} wedges were cut at all");
        assert!(
            chamfered * 100 >= cut * 95,
            "only {chamfered} of {cut} wedges came back with facets"
        );
    }

    /// A spread of directions from one integer, so the sweep above covers
    /// angles rather than axes.
    fn jitter_direction(salt: u32) -> Vector3<f32> {
        let a = salt as f32 * 2.399_963;
        let z = (salt as f32 / 400.0) * 2.0 - 1.0;
        let r = (1.0 - z * z).max(0.0).sqrt();
        Vector3::new(r * a.cos(), r * a.sin(), z).normalize()
    }
}
