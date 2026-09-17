//! Where a struck block breaks, and into how many pieces.
//!
//! ```text
//!        hit                    hit                    hit
//!         │                      │                      │
//!   ┌─────▼─────┐          ┌─────▼─────┐          ┌─────▼─────┐
//!   │           │          │    ╱      │          │  ╲ ╱      │
//!   │           │   ──▶    │   ╱       │   ──▶    │   ╳       │
//!   │           │          │  ╱        │          │  ╱ ╲      │
//!   └───────────┘          └─╱─────────┘          └─╱───╲─────┘
//!      a block            one cleave, tilted     two, and three pieces
//! ```
//!
//! Ice is not glass. A pane is a plane and breaks into a web of cells; a
//! block is a solid and breaks the way a solid does — along a few surfaces
//! that run right through it, leaving wedges. So the rule here is not a
//! pattern but a short list of cut planes, and the whole of the geometry is
//! [`split_hull`].
//!
//! Two decisions make the pieces look like broken ice rather than sawn
//! timber. The cut is *tilted* off the block's own axes, so the faces of a
//! piece are not the faces of a box and the chunks read as fracture rather
//! than as joinery. And the cut runs near, but not through, the point of the
//! hit, so the break is visibly caused by the blow and not by geometry.

use nalgebra::{Unit, UnitQuaternion, Vector3};

use crate::collision::convex_hull::{cube_hull, ConvexHull};
use crate::collision::hull_split::{split_hull, Plane};
use crate::physics::ColliderShape;

/// How many placements of a cut to try before giving a block up as unbreakable.
///
/// A cut is refused when it would leave a wafer or a sliver, which depends on
/// where it fell; another draw usually lands somewhere the block can take.
const PLACEMENT_ATTEMPTS: u32 = 6;

/// How far along the way from the block's centre to the hit the cut passes.
/// All the way through the hit tends to shave the surface the blow landed on
/// rather than break the block under it.
const HIT_BIAS: f32 = 0.45;

/// Least share of the piece, measured across the cut, that each side of the
/// cut must keep.
///
/// A blow on a corner pulls the cut towards that corner, and on a small
/// block it can pull it past the point where one side is a sliver the engine
/// will not take. Clamping the plane into the middle of the piece breaks the
/// block wherever it was hit and still leaves two pieces worth having;
/// without it, a couple of the dome's blocks could not be broken at all.
const MIN_SHARE: f32 = 0.3;

/// How a solid block comes apart when it is hit hard enough.
#[derive(Debug, Clone, Copy)]
pub struct CleaveRule {
    /// Contact spike or blast, in N·s, above which a block cleaves.
    pub threshold: f32,
    /// Most pieces one blow makes. Two means a single cut; three, two cuts,
    /// the second across the larger piece.
    pub pieces: u32,
    /// How far, in degrees, a cut tilts off the block's own axes. Zero saws
    /// a block into boxes; a few degrees is enough that a piece reads as a
    /// wedge, and too many makes spikes.
    pub tilt: f32,
    /// A piece smaller than this, in m³, is not worth a body of its own: a
    /// block this size is knocked out whole instead of cleaved.
    pub min_volume: f32,
}

impl Default for CleaveRule {
    fn default() -> Self {
        Self {
            threshold: 12.0,
            pieces: 3,
            tilt: 14.0,
            min_volume: 0.004,
        }
    }
}

/// One piece of a cleaved block: its hull about its own centre of volume, and
/// where that centre sits in the frame the block was given in.
///
/// Split in two because a collider's offset is taken to be the centre of mass
/// of what it holds, so a piece cut out of a block has to be handed over
/// already moved onto its own centre.
pub struct CleavePiece {
    pub hull: ConvexHull,
    pub centre: Vector3<f32>,
}

impl CleaveRule {
    /// The pieces `shape` breaks into around `hit`, both given in the child's
    /// own frame, in a layout chosen by `salt`.
    ///
    /// `None` if the block is too small to be worth breaking, or if no
    /// placement gave pieces the physics engine can hold — the caller's cue
    /// to knock the block out whole instead.
    pub fn cleave(
        &self,
        shape: &ColliderShape,
        hit: Vector3<f32>,
        salt: u32,
    ) -> Option<Vec<CleavePiece>> {
        let hull = hull_of(shape)?;
        if hull.compute_volume() < self.min_volume * 2.0 {
            return None;
        }

        (0..PLACEMENT_ATTEMPTS).find_map(|attempt| {
            let mut jitter = Jitter::new(salt.wrapping_add(attempt));
            let mut pieces = vec![hull.clone()];
            for _ in 1..self.pieces.max(2) {
                // Always cut the largest piece: cutting a wedge that is
                // already small makes two pieces too small to keep, and the
                // block ends up as one big chunk and two crumbs.
                let largest = pieces
                    .iter()
                    .enumerate()
                    .max_by(|a, b| a.1.compute_volume().total_cmp(&b.1.compute_volume()))
                    .map(|(index, _)| index)?;
                let halves = split_hull(
                    &pieces[largest],
                    self.plane(&pieces[largest], hit, &mut jitter),
                )?;
                pieces.swap_remove(largest);
                pieces.push(halves.front);
                pieces.push(halves.back);
            }
            let sound = pieces
                .iter()
                .all(|piece| piece.compute_volume() >= self.min_volume);
            sound.then(|| {
                pieces
                    .into_iter()
                    .map(|hull| {
                        let centre = hull.centroid();
                        CleavePiece {
                            hull: hull.translated(-centre),
                            centre,
                        }
                    })
                    .collect()
            })
        })
    }

    /// A cut plane across `piece`: square to its longest axis, tilted off it,
    /// and passing between the piece's centre and the hit.
    ///
    /// Square to the longest axis because a cut along it would pare a slab
    /// into wafers; across it, the two pieces are both stout.
    fn plane(&self, piece: &ConvexHull, hit: Vector3<f32>, jitter: &mut Jitter) -> Plane {
        let centre = piece.centroid();
        let axis = longest_axis(piece);

        // Tilt by a fixed angle in a random direction square to the axis, so
        // every cut is as slanted as the rule asks and no two are alike.
        let square = if axis.x.abs() < 0.9 {
            axis.cross(&Vector3::x()).normalize()
        } else {
            axis.cross(&Vector3::y()).normalize()
        };
        let spin = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(axis),
            jitter.range(0.0, std::f32::consts::TAU),
        );
        let tilt = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(spin * square),
            self.tilt.to_radians() * jitter.range(0.6, 1.0),
        );
        let normal = tilt * axis;

        // Between the centre and the hit, and never so near an end that the
        // cut shaves rather than breaks.
        let towards_hit = (hit - centre) * HIT_BIAS;
        let wander = square * (jitter.unit() - 0.5) * 0.1;
        let wanted = normal.dot(&(centre + towards_hit + wander));

        let (near, far) = span_along(piece, normal);
        let margin = (far - near) * MIN_SHARE;
        Plane {
            normal,
            offset: wanted.clamp(near + margin, far - margin),
        }
    }
}

/// The hull of a child's shape: boxes are the common case and become their
/// own eight corners, hulls are already what is wanted, and nothing else can
/// be cleaved.
fn hull_of(shape: &ColliderShape) -> Option<ConvexHull> {
    match shape {
        ColliderShape::Box { half_extents } => Some(cube_hull(*half_extents)),
        ColliderShape::ConvexHull { hull } => Some((**hull).clone()),
        _ => None,
    }
}

/// How far the hull reaches each way along `direction`.
fn span_along(hull: &ConvexHull, direction: Vector3<f32>) -> (f32, f32) {
    hull.vertices
        .iter()
        .fold((f32::MAX, f32::MIN), |(near, far), v| {
            let d = direction.dot(v);
            (near.min(d), far.max(d))
        })
}

/// The direction across which the hull is longest, chosen from its own face
/// normals.
///
/// Not the direction of the furthest corner, which for any box is a body
/// diagonal: cutting a block corner to corner leaves wedges too thin for the
/// engine to take, and on the smaller blocks of a dome it leaves nothing the
/// engine will take at all. Measured across the faces instead, a block is
/// longest along the axis it is actually longest along.
fn longest_axis(hull: &ConvexHull) -> Vector3<f32> {
    hull.faces
        .iter()
        .map(|face| {
            let (near, far) = span_along(hull, face.normal);
            (face.normal, far - near)
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(normal, _)| normal)
        .unwrap_or_else(Vector3::x)
}

/// A small deterministic noise source, so a break depends only on where the
/// block was hit and not on anything that happened before. Its own rather
/// than shared with the texture generators, which sit above the physics.
struct Jitter {
    state: u32,
}

impl Jitter {
    fn new(salt: u32) -> Self {
        Self {
            state: salt.wrapping_mul(0x9E37_79B9) ^ 0x5bd1_e995,
        }
    }

    /// Uniform in `[0, 1)`.
    fn unit(&mut self) -> f32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        (self.state >> 8) as f32 / (1u32 << 24) as f32
    }

    /// Uniform in `[lo, hi)`.
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + self.unit() * (hi - lo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block() -> ColliderShape {
        ColliderShape::Box {
            half_extents: Vector3::new(0.3, 0.15, 0.2),
        }
    }

    fn volume(shape: &ColliderShape) -> f32 {
        hull_of(shape).expect("a box is a hull").compute_volume()
    }

    #[test]
    fn a_struck_block_comes_apart_into_the_asked_for_pieces() {
        let rule = CleaveRule::default();
        let pieces = rule
            .cleave(&block(), Vector3::new(0.25, 0.1, 0.0), 7)
            .expect("a block this size cleaves");
        assert_eq!(pieces.len(), 3);
        let total: f32 = pieces.iter().map(|p| p.hull.compute_volume()).sum();
        assert!(
            (total - volume(&block())).abs() < 1e-4,
            "the pieces make {total} of {}",
            volume(&block())
        );
    }

    #[test]
    fn two_pieces_when_two_are_asked_for() {
        let rule = CleaveRule {
            pieces: 2,
            ..CleaveRule::default()
        };
        let pieces = rule
            .cleave(&block(), Vector3::new(0.0, 0.1, 0.0), 3)
            .expect("cleaves");
        assert_eq!(pieces.len(), 2);
    }

    /// Every piece is handed over already sitting on its own centre of
    /// volume, because that is where the physics engine will assume its mass
    /// is. A piece still carrying the block's frame would put the body's
    /// centre of mass somewhere it is not.
    #[test]
    fn each_piece_is_centred_on_itself() {
        let pieces = CleaveRule::default()
            .cleave(&block(), Vector3::new(0.2, 0.0, 0.1), 11)
            .expect("cleaves");
        for piece in &pieces {
            assert!(
                piece.hull.centroid().magnitude() < 1e-4,
                "a piece is off its own centre by {}",
                piece.hull.centroid().magnitude()
            );
            assert!(
                piece.centre.magnitude() < 0.4,
                "a piece sits outside the block"
            );
        }
        // Put back where they came from, the pieces still fill the block.
        let spread: f32 = pieces
            .iter()
            .map(|p| p.centre.magnitude())
            .fold(0.0, f32::max);
        assert!(spread > 0.02, "every piece cannot be at the block's centre");
    }

    /// The point of asking for a tilt: no face of a piece is square to the
    /// block, so the chunks read as fracture rather than as sawn timber.
    #[test]
    fn the_cuts_are_tilted_off_the_blocks_own_axes() {
        let pieces = CleaveRule::default()
            .cleave(&block(), Vector3::new(0.25, 0.0, 0.0), 5)
            .expect("cleaves");
        let square_to_an_axis = |n: &Vector3<f32>| {
            [Vector3::x(), Vector3::y(), Vector3::z()]
                .iter()
                .any(|axis| n.dot(axis).abs() > 0.999)
        };
        let tilted = pieces
            .iter()
            .flat_map(|p| p.hull.faces.iter())
            .filter(|face| !square_to_an_axis(&face.normal))
            .count();
        assert!(tilted >= 2, "only {tilted} faces are off square");
    }

    /// The call the game makes, over every shape and every place a blow can
    /// land on one. A grenade in an igloo found a cut that panicked the hull
    /// builder; nothing here may panic, and a block struck anywhere must
    /// nearly always break.
    #[test]
    fn no_blow_anywhere_on_any_block_can_panic_or_leave_it_whole() {
        let mut state = 0x9e37_79b9u32;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state >> 8) as f32 / (1u32 << 24) as f32
        };
        // A cube, a slab, a plank, a post, and the proportions of the dome
        // blocks that failed first.
        let shapes = [
            Vector3::new(0.25, 0.25, 0.25),
            Vector3::new(0.5, 0.08, 0.4),
            Vector3::new(0.8, 0.12, 0.12),
            Vector3::new(0.12, 0.6, 0.12),
            Vector3::new(0.1486, 0.2317, 0.14),
        ];
        let rule = CleaveRule::default();
        let (mut tried, mut broke) = (0, 0);
        for half in shapes {
            let shape = ColliderShape::Box { half_extents: half };
            for salt in 0..200u32 {
                // Anywhere on or just outside the block, corners included.
                let hit = half.component_mul(&Vector3::new(
                    next() * 2.2 - 1.1,
                    next() * 2.2 - 1.1,
                    next() * 2.2 - 1.1,
                ));
                tried += 1;
                let Some(pieces) = rule.cleave(&shape, hit, salt) else {
                    continue;
                };
                broke += 1;
                let total: f32 = pieces.iter().map(|p| p.hull.compute_volume()).sum();
                let whole = 8.0 * half.x * half.y * half.z;
                assert!(
                    (total - whole).abs() < whole * 0.02,
                    "{half:?} struck at {hit:?} made {total} of {whole}"
                );
            }
        }
        assert!(
            broke * 100 >= tried * 98,
            "only {broke} of {tried} blows broke the block"
        );
    }

    #[test]
    fn a_block_too_small_to_be_worth_breaking_says_so() {
        let crumb = ColliderShape::Box {
            half_extents: Vector3::repeat(0.05),
        };
        assert!(CleaveRule::default()
            .cleave(&crumb, Vector3::zeros(), 1)
            .is_none());
    }

    #[test]
    fn a_shape_that_is_not_a_solid_is_not_cleaved() {
        let ball = ColliderShape::Sphere { radius: 0.4 };
        assert!(CleaveRule::default()
            .cleave(&ball, Vector3::zeros(), 1)
            .is_none());
    }

    /// A wedge cut from a block is itself cleavable, which is what lets a
    /// depth rule rather than the geometry decide when to stop.
    #[test]
    fn a_piece_of_a_block_can_be_cleaved_again() {
        let first = CleaveRule::default()
            .cleave(&block(), Vector3::new(0.2, 0.0, 0.0), 2)
            .expect("cleaves");
        let biggest = first
            .iter()
            .max_by(|a, b| a.hull.compute_volume().total_cmp(&b.hull.compute_volume()))
            .expect("a piece");
        let shape = ColliderShape::ConvexHull {
            hull: std::sync::Arc::new(biggest.hull.clone()),
        };
        assert!(CleaveRule::default()
            .cleave(&shape, Vector3::zeros(), 4)
            .is_some());
    }
}
