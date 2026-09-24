//! The shape of an old stone block, as a field to project a mesh onto.
//!
//! A dressed block is a convex hull, and a convex hull drawn as it collides is
//! a box of flat faces meeting at knife edges — what a stone looks like the
//! day the mason finishes it. Centuries take that away in a particular order:
//! the arrises round over first, then chip, and the faces go soft and uneven.
//! A block that has just broken shows none of that on the break, which is raw,
//! sharp-edged and rough.
//!
//! All of it is expressed as one scalar field over space, negative inside the
//! stone and zero on its weathered surface:
//!
//! ```text
//!   F(p) = max( smooth_max(dressed faces), max(fresh faces) ) + wear(p)
//!               ─────────┬──────────        ───────┬───────     ───┬───
//!               arrises rounded over       breaks stay sharp    ≥ 0: only
//!                                                               ever cuts in
//! ```
//!
//! Every term is at least the hull's own plane distance, so the surface never
//! stands proud of the collider: a weathered block rests on what it looks like
//! it rests on, to within the wear.
//!
//! Everything scales with the block's thickness, so a small arch and a huge
//! one wear the same *proportion* of their stone. Absolute centimetres would
//! leave a five-metre voussoir with razor-sharp edges and a half-metre one
//! with its arrises eaten away.

use nalgebra::Vector3;

use crate::collision::convex_hull::ConvexHull;
use crate::utils::noise::fbm_perlin_3d;

/// How far the arrises are rounded back, as a fraction of the block's
/// thickness. The inset at an edge is `ln 2` times this.
const ROUNDING: f32 = 0.022;

/// How far back from an arris chipping reaches, as a fraction of thickness.
///
/// Set against [`CHIP_DEPTH`], not against the rounding: a chip's depth
/// falls off over this distance, and a depth that changes faster than a
/// metre per metre across the surface is an undercut — the drawing folds
/// under itself. This keeps the slope of a chip wall below one.
const CHIP_REACH: f32 = 0.12;

/// How far from an arris the direction points are projected along keeps
/// turning towards the diagonal, as a multiple of [`CHIP_REACH`].
///
/// Everything within reach of a chip may be cut in by the depth of one, and a
/// point pushed straight in from one face by more than its distance from the
/// arris passes the points pushed in from the other face: the drawing folds,
/// and the fold shows as a hole. Tied to the rounding — a centimetre — it
/// folded a couple of hundred triangles on every voussoir.
const PROJECTION_SPREAD: f32 = 1.5;

/// Deepest a chip bites into an arris, as a fraction of thickness.
const CHIP_DEPTH: f32 = 0.065;

/// Chips per thickness along an arris, roughly.
const CHIP_FREQUENCY: f32 = 3.0;

/// How uneven a weathered face is, as a fraction of thickness: the hollows
/// rain and frost wear into a face that was once dressed flat.
const UNDULATION_DEPTH: f32 = 0.014;

/// Undulations per thickness across a face.
const UNDULATION_FREQUENCY: f32 = 3.0;

/// Half the relief of a fresh break, as a fraction of thickness. The two
/// sides of a break interlock, so the gap between them is twice this.
const FRACTURE_RELIEF: f32 = 0.016;

/// How far the rounding of an arris swings either side of [`ROUNDING`] along
/// its length, as a fraction. An arris rounded to one radius all the way
/// along reads as a machined chamfer, not as wear.
const ROUNDING_SWING: f32 = 0.55;

/// Changes of rounding per thickness along an arris.
const ROUNDING_FREQUENCY: f32 = 2.0;

/// How much harder an arris facing the sky is worn than one facing the
/// ground, as a fraction of the chip depth. Rain and frost work from above.
const EXPOSED_WEAR: f32 = 0.6;

/// How far the relief field of a fresh break is stretched before clamping.
/// Gradient noise summed over octaves rarely strays past half its range, and
/// relief that never reaches its bound is relief left unused.
const FRACTURE_CONTRAST: f32 = 1.8;

/// Bumps per thickness across a fresh break.
const FRACTURE_FREQUENCY: f32 = 2.5;

/// Seeds for the three fields, so they do not line up with each other.
const UNDULATION_SEED: u32 = 401;
const CHIP_SEED: u32 = 877;
const FRACTURE_SEED: u32 = 1301;
const ROUNDING_SEED: u32 = 1627;

/// Two planes this close in normal (dot product) and offset (fraction of
/// thickness) are the same plane. Loose enough to take the hairline a
/// cleaved piece is shrunk by, tight enough that a cut tilted by a few
/// degrees off a face is never mistaken for it.
const SAME_NORMAL: f32 = 0.9995;
const SAME_OFFSET: f32 = 0.02;

/// An arbitrary direction no cut plane will plausibly lie square to. Which
/// side of a break is "up" in its relief is decided against it, so that both
/// halves of one break agree and their bumps interlock.
const BREAK_SIDE: Vector3<f32> = Vector3::new(0.5773, 0.6181, 0.5331);

/// One face plane of a piece.
#[derive(Clone, Copy, Debug)]
struct Facet {
    /// Outward unit normal.
    normal: Vector3<f32>,
    /// `normal · p` on the plane.
    offset: f32,
    /// Whether this is a surface the stone broke along rather than one the
    /// mason dressed.
    fresh: bool,
}

impl Facet {
    fn distance(&self, p: &Vector3<f32>) -> f32 {
        self.normal.dot(p) - self.offset
    }
}

/// What the wear is doing at a point: the numbers the patina is painted from.
#[derive(Clone, Copy, Debug, Default)]
pub struct Wear {
    /// How far the surface was cut back here, in metres.
    pub depth: f32,
    /// How far into the rounding of an arris the point is: one on the arris,
    /// zero out on an open face.
    pub arris: f32,
    /// How much of the point is fresh break rather than old surface.
    pub fresh: f32,
    /// How deep in its face's hollows the point sits, zero to one.
    pub hollow: f32,
}

/// The weathered surface of one piece of stone.
pub struct StoneSurface {
    facets: Vec<Facet>,
    /// The thickness everything is proportioned to, in metres.
    thickness: f32,
    /// Inverse sharpness of the arris rounding, in metres.
    rounding: f32,
}

impl StoneSurface {
    /// The surface of `piece`, whose vertices are already in the stone's
    /// texture frame, cut from `whole` in that same frame.
    ///
    /// A piece with no `whole` is taken to be whole: every face is old.
    pub fn new(piece: &ConvexHull, whole: Option<&ConvexHull>) -> Self {
        let original = whole.unwrap_or(piece);
        let thickness = thickness_of(original).max(1e-3);
        let original_planes: Vec<(Vector3<f32>, f32)> = original
            .faces
            .iter()
            .map(|face| {
                let point = original.vertices[face.vertex_indices[0] as usize];
                (face.normal, face.normal.dot(&point))
            })
            .collect();

        let facets = piece
            .faces
            .iter()
            .map(|face| {
                let point = piece.vertices[face.vertex_indices[0] as usize];
                let offset = face.normal.dot(&point);
                let fresh = !original_planes.iter().any(|(normal, other)| {
                    normal.dot(&face.normal) > SAME_NORMAL
                        && (offset - other).abs() < SAME_OFFSET * thickness
                });
                Facet {
                    normal: face.normal,
                    offset,
                    fresh,
                }
            })
            .collect();

        Self {
            facets,
            thickness,
            rounding: ROUNDING * thickness,
        }
    }

    /// The thickness the wear is proportioned to.
    pub fn thickness(&self) -> f32 {
        self.thickness
    }

    /// The field: negative inside the weathered stone, zero on its surface.
    pub fn field(&self, p: &Vector3<f32>) -> f32 {
        self.shape_and_wear(p).0
    }

    /// What the wear is doing at `p`.
    pub fn wear(&self, p: &Vector3<f32>) -> Wear {
        self.shape_and_wear(p).1
    }

    /// The direction a point on the hull moves in to reach the weathered
    /// surface: inward, and a continuous function of position, so two faces
    /// sharing an edge move their shared vertices identically.
    ///
    /// It turns from one face's normal to the next over the whole reach of a
    /// chip, not only over the rounding — see [`PROJECTION_SPREAD`].
    pub fn inward(&self, p: &Vector3<f32>) -> Vector3<f32> {
        let distances: Vec<f32> = self.facets.iter().map(|f| f.distance(p)).collect();
        let top = distances.iter().copied().fold(f32::MIN, f32::max);
        let mut sum = Vector3::zeros();
        for (facet, d) in self.facets.iter().zip(&distances) {
            sum += facet.normal
                * ((d - top) / (self.thickness * CHIP_REACH * PROJECTION_SPREAD)).exp();
        }
        let length = sum.magnitude();
        if length > 1e-6 {
            -sum / length
        } else {
            Vector3::zeros()
        }
    }

    /// Move a point on the hull onto the weathered surface.
    ///
    /// The field falls at close to one metre per metre along [`inward`], so
    /// a secant search from the hull converges in a handful of steps. Held
    /// inside a bracket so that a steep chip wall cannot throw it outside the
    /// stone.
    ///
    /// [`inward`]: Self::inward
    pub fn project(&self, p: &Vector3<f32>) -> Vector3<f32> {
        let direction = self.inward(p);
        let at = |t: f32| self.field(&(p + direction * t));

        let (mut lo, mut f_lo) = (0.0f32, at(0.0));
        if f_lo <= 0.0 {
            return *p;
        }
        let reach = self.thickness
            * (CHIP_DEPTH * (1.0 + EXPOSED_WEAR) + UNDULATION_DEPTH + 4.0 * FRACTURE_RELIEF)
            + self.rounding * (1.0 + ROUNDING_SWING) * 3.0;
        let (mut hi, mut f_hi) = (reach, at(reach));
        if f_hi > 0.0 {
            return p + direction * hi;
        }

        // Illinois-style false position: the secant, with the stale end's
        // weight halved so a curved field cannot stall one side.
        let tolerance = self.thickness * 1e-5;
        let mut last_side = 0i8;
        for _ in 0..24 {
            let t = (lo * f_hi - hi * f_lo) / (f_hi - f_lo);
            let f = at(t);
            if f.abs() < tolerance {
                return p + direction * t;
            }
            if f > 0.0 {
                lo = t;
                f_lo = f;
                if last_side == 1 {
                    f_hi *= 0.5;
                }
                last_side = 1;
            } else {
                hi = t;
                f_hi = f;
                if last_side == -1 {
                    f_lo *= 0.5;
                }
                last_side = -1;
            }
        }
        p + direction * (lo * f_hi - hi * f_lo) / (f_hi - f_lo)
    }

    /// The outward surface normal at `p`, from the field's gradient.
    ///
    /// Four samples on a tetrahedron rather than six on the axes: the same
    /// accuracy for a smooth field, and this is the most expensive thing done
    /// per vertex.
    pub fn normal(&self, p: &Vector3<f32>) -> Vector3<f32> {
        let h = self.thickness * 2e-3;
        let k = [
            Vector3::new(1.0, -1.0, -1.0),
            Vector3::new(-1.0, -1.0, 1.0),
            Vector3::new(-1.0, 1.0, -1.0),
            Vector3::new(1.0, 1.0, 1.0),
        ];
        let gradient: Vector3<f32> = k.iter().map(|k| k * self.field(&(p + k * h))).sum();
        let length = gradient.magnitude();
        if length > 1e-12 {
            gradient / length
        } else {
            Vector3::y()
        }
    }

    /// The field at `p`, and what the wear is doing there.
    fn shape_and_wear(&self, p: &Vector3<f32>) -> (f32, Wear) {
        let mut dressed_top = f32::MIN;
        let mut fresh_top = f32::MIN;
        let mut fresh_normal = Vector3::zeros();
        for facet in &self.facets {
            let d = facet.distance(p);
            if facet.fresh {
                if d > fresh_top {
                    fresh_top = d;
                    fresh_normal = facet.normal;
                }
            } else {
                dressed_top = dressed_top.max(d);
            }
        }

        // Log-sum-exp: a smooth maximum that equals the plain maximum out on
        // a face and rounds over where two faces compete. Its softness varies
        // along the arris, so no two stretches of an edge are worn alike.
        let r = ROUNDING_FREQUENCY / self.thickness;
        let swing = fbm_perlin_3d(p.x * r, p.y * r, p.z * r, 2, 0.5, ROUNDING_SEED);
        let rounding = self.rounding * (1.0 + ROUNDING_SWING * (swing * 2.0).clamp(-1.0, 1.0));
        let (mut near, mut wide, mut up) = (0.0f32, 0.0f32, 0.0f32);
        let chip_reach = self.thickness * CHIP_REACH;
        for facet in self.facets.iter().filter(|f| !f.fresh) {
            let d = facet.distance(p) - dressed_top;
            near += (d / rounding).exp();
            let weight = (d / chip_reach).exp();
            wide += weight;
            up += weight * facet.normal.y;
        }
        let has_dressed = dressed_top > f32::MIN;
        let dressed = if has_dressed {
            dressed_top + rounding * near.ln()
        } else {
            f32::MIN
        };
        // Which way the surface nearby faces the sky, blended across arrises
        // so the extra wear on a top edge has no step in it.
        let exposure = if has_dressed {
            (up / wide).max(0.0)
        } else {
            0.0
        };
        // The old surface and the break are each worn on their own and then
        // intersected, as two solids. Blending their wear across the crease
        // instead puts a step in the field wherever a deep chip on an old
        // arris runs into a shallow break — and a step is a fold in the
        // drawing, a hole at the corner of every cracked stone.
        let mut wear = Wear::default();
        let mut old_surface = f32::MIN;
        if has_dressed {
            // Near an arris, within reach of a chip.
            let arris =
                ((chip_reach * wide.ln()) / (chip_reach * std::f32::consts::LN_2)).clamp(0.0, 1.0);
            let s = UNDULATION_FREQUENCY / self.thickness;
            let hollow =
                0.5 + 0.5 * fbm_perlin_3d(p.x * s, p.y * s, p.z * s, 3, 0.5, UNDULATION_SEED);
            let c = CHIP_FREQUENCY / self.thickness;
            // Broad in the noise's range and only two octaves: a chip is a
            // steep-walled bite, but a wall steeper than the direction a
            // point is projected along folds the drawing under itself.
            let chip_field = fbm_perlin_3d(p.x * c, p.y * c, p.z * c, 2, 0.45, CHIP_SEED);
            let chip = smoothstep(-0.1, 0.5, chip_field) * arris * (1.0 + EXPOSED_WEAR * exposure);
            let depth = (hollow * UNDULATION_DEPTH + chip * CHIP_DEPTH) * self.thickness;
            old_surface = dressed + depth;
            wear.depth = depth;
            wear.arris = arris;
            wear.hollow = hollow;
        }

        let mut fresh_surface = f32::MIN;
        if fresh_top > f32::MIN {
            // Signed relief about the break plane, read from one side for
            // both halves so that one's bump is the other's hollow.
            let f = FRACTURE_FREQUENCY / self.thickness;
            let relief = fbm_perlin_3d(p.x * f, p.y * f, p.z * f, 5, 0.55, FRACTURE_SEED)
                * FRACTURE_CONTRAST;
            let side = if fresh_normal.dot(&BREAK_SIDE) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            let depth = FRACTURE_RELIEF * self.thickness * (1.0 - side * relief.clamp(-1.0, 1.0));
            fresh_surface = fresh_top + depth;

            // How much of this point is fresh break: a narrow blend across
            // the crease, for the colour's sake only.
            let blend = self.thickness * 0.004;
            wear.fresh = if has_dressed {
                smoothstep(-blend, blend, fresh_surface - old_surface)
            } else {
                1.0
            };
            wear.depth += (depth - wear.depth) * wear.fresh;
        }

        (old_surface.max(fresh_surface), wear)
    }
}

/// How thick a hull is: its least extent across its own face normals.
fn thickness_of(hull: &ConvexHull) -> f32 {
    hull.faces
        .iter()
        .map(|face| {
            let (near, far) = hull
                .vertices
                .iter()
                .fold((f32::MAX, f32::MIN), |(n, f), v| {
                    let d = face.normal.dot(v);
                    (n.min(d), f.max(d))
                });
            far - near
        })
        .fold(f32::MAX, f32::min)
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::convex_hull::cube_hull;
    use crate::collision::hull_split::{split_hull, Plane};

    fn block() -> ConvexHull {
        cube_hull(Vector3::new(0.6, 0.25, 0.3))
    }

    /// The whole point of building the field from the hull's planes: the
    /// drawing never stands proud of the collider, so a block never looks as
    /// though it floats over, or sinks into, what it is resting on.
    #[test]
    fn the_weathered_surface_never_leaves_the_hull() {
        let hull = block();
        let surface = StoneSurface::new(&hull, None);
        for face in &hull.faces {
            let corners: Vec<Vector3<f32>> = face
                .vertex_indices
                .iter()
                .map(|&i| hull.vertices[i as usize])
                .collect();
            let centre = corners.iter().sum::<Vector3<f32>>() / corners.len() as f32;
            for corner in &corners {
                for step in 0..=10 {
                    let p = centre + (corner - centre) * (step as f32 / 10.0);
                    let on = surface.project(&p);
                    for other in &hull.faces {
                        let q = hull.vertices[other.vertex_indices[0] as usize];
                        assert!(
                            other.normal.dot(&(on - q)) <= 1e-4,
                            "{on:?} stands outside the face with normal {:?}",
                            other.normal
                        );
                    }
                }
            }
        }
    }

    /// Projection lands on the surface it projects onto.
    #[test]
    fn a_projected_point_lies_on_the_field() {
        let hull = block();
        let surface = StoneSurface::new(&hull, None);
        for corner in &hull.vertices {
            let on = surface.project(corner);
            assert!(
                surface.field(&on).abs() < 1e-4,
                "field {} at {on:?}",
                surface.field(&on)
            );
        }
    }

    /// Old arrises round over; the middle of a face stays near the plane.
    #[test]
    fn corners_wear_away_more_than_faces() {
        let hull = block();
        let surface = StoneSurface::new(&hull, None);
        let corner = Vector3::new(0.6, 0.25, 0.3);
        let middle = Vector3::new(0.0, 0.25, 0.0);
        let corner_loss = (surface.project(&corner) - corner).magnitude();
        let middle_loss = (surface.project(&middle) - middle).magnitude();
        assert!(
            corner_loss > middle_loss * 2.0,
            "corner lost {corner_loss}, face lost {middle_loss}"
        );
        assert!(middle_loss < 0.5 * 0.05, "a face lost {middle_loss}");
    }

    fn halves() -> (ConvexHull, ConvexHull) {
        let normal = Vector3::new(1.0, 0.2, 0.1).normalize();
        let split = split_hull(
            &block(),
            Plane {
                normal,
                offset: 0.05,
            },
        )
        .expect("splits");
        (split.front, split.back)
    }

    /// The faces a block broke along are told from the ones it was made with.
    #[test]
    fn a_break_is_recognised_as_fresh_and_the_old_faces_are_not() {
        let whole = block();
        let (front, _) = halves();
        let surface = StoneSurface::new(&front, Some(&whole));
        let fresh = surface.facets.iter().filter(|f| f.fresh).count();
        assert_eq!(fresh, 1, "one cut should make one fresh face");
        assert_eq!(surface.facets.len() - fresh, front.faces.len() - 1);
    }

    /// The two halves of one break interlock: wherever one is cut deep, the
    /// other is cut shallow, so they close up into a hairline rather than a
    /// groove.
    #[test]
    fn the_two_halves_of_a_break_interlock() {
        let whole = block();
        let (front, back) = halves();
        let a = StoneSurface::new(&front, Some(&whole));
        let b = StoneSurface::new(&back, Some(&whole));
        let normal = Vector3::new(1.0, 0.2, 0.1).normalize();
        let on_cut = |y: f32, z: f32| {
            let lateral = Vector3::new(0.0, y, z);
            lateral - normal * (normal.dot(&lateral) - 0.05)
        };
        for (y, z) in [(0.0, 0.0), (0.1, -0.1), (-0.12, 0.2), (0.05, 0.15)] {
            let p = on_cut(y, z);
            let gap = a.wear(&p).depth + b.wear(&p).depth;
            let expected = 2.0 * FRACTURE_RELIEF * a.thickness();
            assert!(
                (gap - expected).abs() < expected * 0.05,
                "the halves stand {gap} apart at {p:?}, expected {expected}"
            );
        }
    }
}
