use crate::path::PathVec;
use crate::section::{BodyKey, CastHit, Footing, SphereCaster};

use super::authored::{AuthoredGround, Form, Shape};

/// The key of fixed ground: the base and every fixed shape, which like the
/// game's terrain are one body.
const FIXED_BODY: BodyKey = BodyKey(0);

/// How far a shape with no `across` extends to either side: past any strip.
const UNBOUNDED_ACROSS: f32 = 1.0e3;

/// A solid bounded by a planar face over a rectangle in `(s, d)`, extending
/// away from the face without end: down from a floor's top, up from a roof's
/// underside.
struct Prism {
    s: (f32, f32),
    d: (f32, f32),
    /// The face's plane: `z = z0 + slope_s * s + slope_d * d`.
    z0: f32,
    slope_s: f32,
    slope_d: f32,
    /// `+1` for a floor (solid below the face), `-1` for a roof (solid above).
    side: f32,
    body: BodyKey,
    footing: Footing,
}

impl Prism {
    fn from_shape(shape: &Shape, number: usize) -> Self {
        let body = match shape.footing {
            Footing::Fixed => FIXED_BODY,
            Footing::Loose { .. } => BodyKey(number as u64 + 1),
        };
        let d = shape
            .across
            .unwrap_or((-UNBOUNDED_ACROSS, UNBOUNDED_ACROSS));
        let (s, z0, slope_s, slope_d, side) = match shape.form {
            Form::Block {
                from,
                to,
                top,
                tilt,
            } => ((from, to), top, 0.0, tilt, 1.0),
            Form::Ramp {
                from,
                to,
                start_top,
                end_top,
            } => {
                let slope = (end_top - start_top) / (to - from);
                ((from, to), start_top - slope * from, slope, 0.0, 1.0)
            }
            Form::Roof { from, to, bottom } => ((from, to), bottom, 0.0, 0.0, -1.0),
        };
        Self {
            s,
            d,
            z0,
            slope_s,
            slope_d,
            side,
            body,
            footing: shape.footing,
        }
    }

    /// The face's point over `(s, d)`.
    fn face_point(&self, s: f32, d: f32) -> PathVec {
        PathVec::new(s, d, self.z0 + self.slope_s * s + self.slope_d * d)
    }

    fn corners(&self) -> [PathVec; 4] {
        [
            self.face_point(self.s.0, self.d.0),
            self.face_point(self.s.1, self.d.0),
            self.face_point(self.s.1, self.d.1),
            self.face_point(self.s.0, self.d.1),
        ]
    }

    fn covers(&self, s: f32, d: f32) -> bool {
        s >= self.s.0 && s <= self.s.1 && d >= self.d.0 && d <= self.d.1
    }

    /// Where a sphere of `radius` centred over `(s, d)`, moving toward the
    /// face from its open side, first touches the prism: its centre height
    /// and the point touched.
    ///
    /// Moving toward a convex solid, the sphere first touches its face, one
    /// of the face's edges or one of its corners; every touch found on one of
    /// those is no later than the first, so the first is the one furthest
    /// out on the open side.
    fn first_touch(&self, s: f32, d: f32, radius: f32) -> Option<(f32, PathVec)> {
        let mut first: Option<(f32, PathVec)> = None;
        let mut consider = |centre_z: f32, touched: PathVec| {
            if first.map_or(true, |(best, _)| self.side * (centre_z - best) > 0.0) {
                first = Some((centre_z, touched));
            }
        };

        let normal = PathVec::new(-self.slope_s, -self.slope_d, 1.0).normalized() * self.side;
        let face_s = s - radius * normal.s;
        let face_d = d - radius * normal.d;
        if self.covers(face_s, face_d) {
            let touched = self.face_point(face_s, face_d);
            consider(touched.z + radius * normal.z, touched);
        }

        let corners = self.corners();
        for (i, &start) in corners.iter().enumerate() {
            let end = corners[(i + 1) % corners.len()];
            if let Some((centre_z, touched)) = edge_touch(start, end, s, d, radius, self.side) {
                consider(centre_z, touched);
            }
            let across = (s - start.s).powi(2) + (d - start.d).powi(2);
            if across <= radius * radius {
                consider(
                    start.z + self.side * (radius * radius - across).sqrt(),
                    start,
                );
            }
        }
        first
    }
}

/// Where a sphere of `radius` centred over `(s, d)` touches the segment from
/// `start` to `end`, approaching from `side` (`+1` from above): its centre
/// height and the point touched.
///
/// The centre `C = (s, d, z)` is `radius` from the segment's line where
/// `|w|² - (w·u)² = r²`, with `w = C - start` and `u` the unit direction.
/// Splitting `w = w0 + z' ẑ` gives a quadratic in `z' = z - start.z`.
fn edge_touch(
    start: PathVec,
    end: PathVec,
    s: f32,
    d: f32,
    radius: f32,
    side: f32,
) -> Option<(f32, PathVec)> {
    let along = end - start;
    let length = along.length();
    if length < f32::EPSILON {
        return None;
    }
    let u = along * (1.0 / length);
    let w0 = PathVec::new(s - start.s, d - start.d, 0.0);
    let a = w0.dot(u);
    // |w0|² - a², taken as the square of w0's part across the line: on a
    // shape that spans the strip the two terms are each near 10⁶.
    let w0_across = w0 - u * a;
    let quadratic = 1.0 - u.z * u.z;
    let linear = -2.0 * a * u.z;
    let constant = w0_across.dot(w0_across) - radius * radius;
    let discriminant = linear * linear - 4.0 * quadratic * constant;
    if quadratic < f32::EPSILON || discriminant < 0.0 {
        return None;
    }
    let root = discriminant.sqrt();
    let rise = (-linear + side * root) / (2.0 * quadratic);
    let t = a + rise * u.z;
    if !(0.0..=length).contains(&t) {
        return None;
    }
    Some((start.z + rise, start + u * t))
}

/// Exact sphere casts against an authored scene's shapes: the authored
/// counterpart of the game's casts in its physics world.
pub struct ShapeCaster {
    base: Option<f32>,
    floors: Vec<Prism>,
    roofs: Vec<Prism>,
}

impl ShapeCaster {
    pub fn new(ground: &AuthoredGround) -> Self {
        let (roofs, floors) = ground
            .shapes
            .iter()
            .enumerate()
            .map(|(number, shape)| Prism::from_shape(shape, number))
            .partition(|prism| prism.side < 0.0);
        Self {
            base: ground.base,
            floors,
            roofs,
        }
    }

    fn hit(
        centre: PathVec,
        touched: PathVec,
        radius: f32,
        body: BodyKey,
        footing: Footing,
    ) -> CastHit {
        CastHit {
            z: centre.z,
            normal: (centre - touched) * (1.0 / radius),
            body,
            footing,
        }
    }
}

impl SphereCaster for ShapeCaster {
    fn cast_down(&self, s: f32, d: f32, radius: f32) -> Option<CastHit> {
        let mut best = self.base.map(|z| CastHit {
            z: z + radius,
            normal: PathVec::UP,
            body: FIXED_BODY,
            footing: Footing::Fixed,
        });
        for prism in &self.floors {
            if let Some((z, touched)) = prism.first_touch(s, d, radius) {
                if best.map_or(true, |hit| z > hit.z) {
                    best = Some(Self::hit(
                        PathVec::new(s, d, z),
                        touched,
                        radius,
                        prism.body,
                        prism.footing,
                    ));
                }
            }
        }
        best
    }

    fn cast_up(&self, s: f32, d: f32, radius: f32) -> Option<CastHit> {
        let mut best: Option<CastHit> = None;
        for prism in &self.roofs {
            if let Some((z, touched)) = prism.first_touch(s, d, radius) {
                if best.map_or(true, |hit| z < hit.z) {
                    best = Some(Self::hit(
                        PathVec::new(s, d, z),
                        touched,
                        radius,
                        prism.body,
                        prism.footing,
                    ));
                }
            }
        }
        best
    }
}
