//! How a clump's leaves give: flattened against what it rests on, and
//! squashed for a moment when it lands.
//!
//! ```text
//!   contacts ──▶ Press (planes, held a moment) ─┐
//!   impacts  ──▶ Squash (a damped spring)  ─────┴─▶ deform(rest skin) ──▶ drawn skin
//! ```
//!
//! The core sits inside the leaves, so the drawn skin reaches past what
//! physics stops. Where it would pass into the ground, it is pressed onto
//! the ground's plane instead, with a soft knee so the flattening rounds
//! off into the clump rather than cutting it. A landing kicks a spring that
//! squashes the clump along the impact and lets it wobble back. Neither
//! changes what physics sees.

use nalgebra::{Isometry3, Point3, Vector3};
use smallvec::SmallVec;

use crate::rendering::vertex::Vertex;

/// A surface the clump is pressed against, in the world.
#[derive(Debug, Clone, Copy)]
pub struct ContactPlane {
    /// A point on the surface.
    pub point: Point3<f32>,
    /// The surface's normal, out of it and into the clump.
    pub normal: Vector3<f32>,
    /// Seconds the plane is held after its contact was last seen.
    pub held: f32,
}

/// The surfaces a clump is pressed against.
#[derive(Debug, Clone, Default)]
pub struct Press {
    planes: SmallVec<[ContactPlane; 4]>,
}

/// How a clump's skin gives.
#[derive(Debug, Clone, Copy)]
pub struct Give {
    /// How far above a surface, in metres, the leaves start to flatten.
    pub knee: f32,
    /// Seconds a surface keeps pressing after its contact stops: contacts
    /// flicker frame to frame, and the skin should not.
    pub hold: f32,
    /// How fast the squash spring rings, Hz.
    pub squash_hz: f32,
    /// The squash spring's damping ratio.
    pub squash_damping: f32,
    /// Squash rate per m/s of impact: how hard a landing kicks the spring.
    pub squash_per_impact: f32,
    /// The furthest the clump squashes, as a share of its height.
    pub squash_limit: f32,
    /// Share of a pressed vertex's push that spreads it sideways, as
    /// pressed leaves bulge out round a contact.
    pub bulge: f32,
}

impl Default for Give {
    fn default() -> Self {
        Self {
            knee: 0.07,
            hold: 0.2,
            squash_hz: 3.5,
            squash_damping: 0.3,
            squash_per_impact: 0.9,
            squash_limit: 0.35,
            bulge: 0.35,
        }
    }
}

/// Two contacts whose normals are this close are one surface.
const SAME_SURFACE: f32 = 0.9;

impl Press {
    /// A contact seen this frame: a new surface, or one already pressing.
    pub fn touch(&mut self, point: Point3<f32>, normal: Vector3<f32>, give: &Give) {
        if let Some(plane) = self
            .planes
            .iter_mut()
            .find(|p| p.normal.dot(&normal) > SAME_SURFACE)
        {
            plane.point = point;
            plane.normal = normal;
            plane.held = give.hold;
            return;
        }
        let plane = ContactPlane {
            point,
            normal,
            held: give.hold,
        };
        if self.planes.len() < self.planes.inline_size() {
            self.planes.push(plane);
        } else if let Some(weakest) = self
            .planes
            .iter_mut()
            .min_by(|a, b| a.held.total_cmp(&b.held))
        {
            *weakest = plane;
        }
    }

    /// Let `dt` pass: surfaces no longer touched fade out.
    pub fn age(&mut self, dt: f32) {
        for plane in &mut self.planes {
            plane.held -= dt;
        }
        self.planes.retain(|p| p.held > 0.0);
    }

    /// The surfaces pressing, each with how firmly (0..1).
    fn pressing<'a>(&'a self, give: &'a Give) -> impl Iterator<Item = (&'a ContactPlane, f32)> {
        self.planes
            .iter()
            .map(move |p| (p, (p.held / give.hold).clamp(0.0, 1.0)))
    }

    /// Whether nothing presses.
    pub fn is_empty(&self) -> bool {
        self.planes.is_empty()
    }
}

/// A damped spring in how far the clump is squashed along an axis.
#[derive(Debug, Clone, Copy, Default)]
pub struct Squash {
    /// Share of its height the clump is squashed by: positive flattened
    /// along `axis`, negative stretched.
    pub amount: f32,
    /// Rate of change of `amount`, per second.
    pub rate: f32,
    /// The axis it squashes along, in the clump's frame.
    pub axis: Vector3<f32>,
    /// The point it squashes towards, in the clump's frame: where it landed.
    pub anchor: Point3<f32>,
}

impl Squash {
    /// A landing at `speed` m/s on a surface whose normal is `axis`, at
    /// `anchor`, both in the clump's frame.
    pub fn kick(&mut self, axis: Vector3<f32>, anchor: Point3<f32>, speed: f32, give: &Give) {
        if self.amount.abs() < 0.02 || speed * give.squash_per_impact > self.rate.abs() {
            self.axis = axis;
            self.anchor = anchor;
        }
        self.rate += speed * give.squash_per_impact;
    }

    /// Let `dt` pass.
    pub fn step(&mut self, dt: f32, give: &Give) {
        let omega = std::f32::consts::TAU * give.squash_hz;
        let accel = -omega * omega * self.amount - 2.0 * give.squash_damping * omega * self.rate;
        self.rate += accel * dt;
        self.amount = (self.amount + self.rate * dt).clamp(-give.squash_limit, give.squash_limit);
    }

    /// Whether the clump is near enough its own shape to draw it so.
    pub fn is_settled(&self) -> bool {
        self.amount.abs() < 1e-3 && self.rate.abs() < 1e-2
    }
}

/// A pressing surface in the clump's own frame.
struct LocalPlane {
    point: Point3<f32>,
    normal: Vector3<f32>,
    /// How firmly it presses, 0..1.
    firmness: f32,
}

/// The skin `rest` bent by `squash` and pressed onto `press`'s surfaces,
/// for a clump at `pose`, into `out`.
pub fn deform(
    rest: &[Vertex],
    pose: &Isometry3<f32>,
    press: &Press,
    squash: &Squash,
    give: &Give,
    out: &mut Vec<Vertex>,
) {
    let to_local = pose.inverse();
    let planes: SmallVec<[LocalPlane; 4]> = press
        .pressing(give)
        .map(|(p, firmness)| LocalPlane {
            point: to_local * p.point,
            normal: to_local * p.normal,
            firmness,
        })
        .collect();
    let squashing = !squash.is_settled();

    out.clear();
    out.extend(rest.iter().map(|vertex| {
        let mut v = *vertex;
        let mut position = Point3::from(v.pos);
        if squashing {
            let (p, n) = squashed(position, v.normal, squash);
            position = p;
            v.normal = n;
        }
        for plane in &planes {
            let (p, n) = pressed(position, v.normal, plane, give);
            position = p;
            v.normal = n;
        }
        v.pos = position.coords;
        v
    }));
}

/// A point and its normal, squashed along the squash's axis towards its
/// anchor and spread across it to keep roughly its volume.
fn squashed(
    position: Point3<f32>,
    normal: Vector3<f32>,
    squash: &Squash,
) -> (Point3<f32>, Vector3<f32>) {
    let axis = squash.axis;
    let along_scale = 1.0 - squash.amount;
    let across_scale = 1.0 + 0.5 * squash.amount;
    let offset = position - squash.anchor;
    let along = offset.dot(&axis);
    let across = offset - axis * along;
    let moved = squash.anchor + axis * (along * along_scale) + across * across_scale;
    // Normals take the inverse scale, so a flattened ball's sides turn up.
    let n_along = normal.dot(&axis);
    let n_across = normal - axis * n_along;
    let turned = axis * (n_along / along_scale) + n_across / across_scale;
    (moved, turned.try_normalize(1e-9).unwrap_or(normal))
}

/// A point and its normal pressed onto a surface: untouched above the knee,
/// eased onto the surface below it, never through it.
fn pressed(
    position: Point3<f32>,
    normal: Vector3<f32>,
    plane: &LocalPlane,
    give: &Give,
) -> (Point3<f32>, Vector3<f32>) {
    let (point, surface, firmness) = (plane.point, plane.normal, plane.firmness);
    let height = (position - point).dot(&surface);
    if height >= give.knee {
        return (position, normal);
    }
    let knee = give.knee;
    let eased = knee * ((height - knee) / knee).exp();
    let lift = (eased - height) * firmness;
    let across = (position - point) - surface * height;
    let spread = across
        .try_normalize(1e-6)
        .map_or(Vector3::zeros(), |d| d * lift * give.bulge);
    // Leaves pressed flat on a surface face into it: the clump's underside.
    let flatness = ((knee - height) / knee).clamp(0.0, 1.0) * firmness;
    let turned = normal.lerp(&-surface, flatness);
    (
        position + surface * lift + spread,
        turned.try_normalize(1e-9).unwrap_or(-surface),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vertex(at: Vector3<f32>) -> Vertex {
        Vertex {
            pos: at,
            color: nalgebra::Vector4::repeat(1.0),
            tex_coords: nalgebra::Vector2::zeros(),
            normal: at.normalize(),
            ao: 1.0,
        }
    }

    /// Leaves below the ground are pressed onto it, never through; leaves
    /// well above it are left alone.
    #[test]
    fn pressed_leaves_stop_at_the_surface() {
        let give = Give::default();
        let mut press = Press::default();
        press.touch(Point3::new(0.0, -0.3, 0.0), Vector3::y(), &give);
        let rest = [
            vertex(Vector3::new(0.0, -0.5, 0.0)),
            vertex(Vector3::new(0.0, 0.5, 0.0)),
        ];
        let mut out = Vec::new();
        deform(
            &rest,
            &Isometry3::identity(),
            &press,
            &Squash::default(),
            &give,
            &mut out,
        );
        assert!(out[0].pos.y > -0.3, "a leaf went through: {}", out[0].pos.y);
        assert!(out[0].pos.y < -0.3 + give.knee);
        assert_eq!(out[1].pos, rest[1].pos);
    }

    /// A surface stops pressing a moment after its contact stops.
    #[test]
    fn an_untouched_surface_lets_go() {
        let give = Give::default();
        let mut press = Press::default();
        press.touch(Point3::origin(), Vector3::y(), &give);
        press.age(give.hold * 0.5);
        assert!(!press.is_empty());
        press.age(give.hold);
        assert!(press.is_empty());
    }

    /// A kicked squash rings down to nothing.
    #[test]
    fn a_squash_rings_down() {
        let give = Give::default();
        let mut squash = Squash::default();
        squash.kick(Vector3::y(), Point3::origin(), 3.0, &give);
        let mut peak = 0.0f32;
        for _ in 0..240 {
            squash.step(1.0 / 60.0, &give);
            peak = peak.max(squash.amount);
        }
        assert!(peak > 0.05, "it barely squashed: {peak}");
        assert!(squash.is_settled(), "still ringing: {squash:?}");
    }
}
