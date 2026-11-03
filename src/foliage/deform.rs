//! How a clump's leaves give: flattened against what it rests on, squashed
//! for a moment when it lands, and parted round whatever pushes through.
//!
//! ```text
//!   contacts ──▶ Press (planes, held a moment) ──┐
//!   impacts  ──▶ Squash (a damped spring)  ──────┼─▶ deform(rest skin) ──▶ drawn skin
//!   passers  ──▶ Parting (capsules, held) ───────┘
//! ```
//!
//! The core sits inside the leaves, so the drawn skin reaches past what
//! physics stops. Where it would pass into the ground, it is pressed onto
//! the ground's plane instead, with a soft knee so the flattening rounds
//! off into the clump rather than cutting it. A landing kicks a spring that
//! squashes the clump along the impact and lets it wobble back. A body in
//! its leaves opens a hole round itself: every leaf moves out from the
//! body's axis by a smooth map that never folds the skin over itself, most
//! near the body and fading with distance, and the leaves ease back once it
//! has gone. None of it changes what physics sees.

use nalgebra::{Isometry3, Point3, Vector3};
use smallvec::SmallVec;

use super::skin::Skin;
use crate::physics::RigidBodyHandle;
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

/// Something pushing through a clump's leaves: a capsule, as the segment
/// at its core and its radius, in the world. A sphere is a capsule whose
/// segment is a point.
#[derive(Debug, Clone, Copy)]
pub struct Pusher {
    /// One end of the core segment.
    pub a: Point3<f32>,
    /// The other end.
    pub b: Point3<f32>,
    /// Radius round the segment.
    pub radius: f32,
}

/// The bodies pushing through a clump's leaves, each held a moment after it
/// was last seen there so the leaves ease back rather than snap.
#[derive(Debug, Clone, Default)]
pub struct Parting {
    pushers: SmallVec<[(RigidBodyHandle, Pusher, f32); 4]>,
}

impl Parting {
    /// `body` is in the leaves this frame, as `pusher`.
    pub fn push(&mut self, body: RigidBodyHandle, pusher: Pusher, give: &Give) {
        if let Some(entry) = self.pushers.iter_mut().find(|e| e.0 == body) {
            entry.1 = pusher;
            entry.2 = give.hold;
        } else if self.pushers.len() < self.pushers.inline_size() {
            self.pushers.push((body, pusher, give.hold));
        }
    }

    /// Let `dt` pass: bodies no longer in the leaves let them go.
    pub fn age(&mut self, dt: f32) {
        for entry in &mut self.pushers {
            entry.2 -= dt;
        }
        self.pushers.retain(|e| e.2 > 0.0);
    }

    /// Whether nothing is in the leaves.
    pub fn is_empty(&self) -> bool {
        self.pushers.is_empty()
    }
}

/// A pusher in the clump's own frame, with how firmly it pushes.
struct LocalPusher {
    a: Point3<f32>,
    b: Point3<f32>,
    radius: f32,
    firmness: f32,
}

/// Radius of the hole a pusher opens in the leaves, as a share of its own:
/// the leaves on its axis are moved this far out, and those further out
/// less, so the hole is about the body's size.
const PARTING_SHARE: f32 = 0.85;
/// How dark the leaves a pusher squeezes in round itself are, at most: they
/// are the clump's inside, shaded by the leaves round them.
const PARTED_SHADE: f32 = 0.65;

/// A pressing surface in the clump's own frame.
struct LocalPlane {
    point: Point3<f32>,
    normal: Vector3<f32>,
    /// How firmly it presses, 0..1.
    firmness: f32,
}

/// Everything bending a clump's skin.
#[derive(Debug, Clone, Default)]
pub struct Bending {
    /// The surfaces it is pressed against.
    pub press: Press,
    /// Its squash after a landing.
    pub squash: Squash,
    /// The bodies pushing through its leaves.
    pub parting: Parting,
}

impl Bending {
    /// Whether nothing bends the skin: it is drawn as it was grown.
    pub fn is_at_rest(&self) -> bool {
        self.press.is_empty() && self.squash.is_settled() && self.parting.is_empty()
    }

    /// Whether the bend changes from frame to frame even with the clump
    /// still: ringing from a landing, or parted by something that may move
    /// on.
    pub fn is_changing(&self) -> bool {
        !self.squash.is_settled() || !self.parting.is_empty()
    }
}

/// `skin` as `bending` bends it: squashed, parted round the bodies in its
/// leaves and pressed onto the surfaces under it, for a clump at `pose`,
/// into `out`.
pub fn deform(
    skin: &Skin,
    pose: &Isometry3<f32>,
    bending: &Bending,
    give: &Give,
    out: &mut Vec<Vertex>,
) {
    let Bending {
        press,
        squash,
        parting,
    } = bending;
    let to_local = pose.inverse();
    let planes: SmallVec<[LocalPlane; 4]> = press
        .pressing(give)
        .map(|(p, firmness)| LocalPlane {
            point: to_local * p.point,
            normal: to_local * p.normal,
            firmness,
        })
        .collect();
    let pushers: SmallVec<[LocalPusher; 4]> = parting
        .pushers
        .iter()
        .map(|(_, p, held)| LocalPusher {
            a: to_local * p.a,
            b: to_local * p.b,
            radius: p.radius,
            firmness: (held / give.hold).clamp(0.0, 1.0) * PARTING_SHARE,
        })
        .collect();
    let squashing = !squash.is_settled();

    out.clear();
    out.extend(
        skin.vertices
            .iter()
            .zip(&skin.sprigs)
            .map(|(vertex, &sprig)| {
                let mut v = *vertex;
                let mut position = Point3::from(v.pos);
                if squashing {
                    let (p, n) = squashed(position, v.normal, squash);
                    position = p;
                    v.normal = n;
                }
                for pusher in &pushers {
                    let parting = parted(position, v.normal, pusher);
                    position = parting.position;
                    v.normal = parting.normal;
                    v.ao *= 1.0 - PARTED_SHADE * (1.0 - parting.squeeze);
                }
                for plane in &planes {
                    let (p, n) = pressed(position, v.normal, sprig, plane, give);
                    position = p;
                    v.normal = n;
                }
                v.pos = position.coords;
                v
            }),
    );
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

/// A point moved out of a pusher's way.
struct Parted {
    position: Point3<f32>,
    normal: Vector3<f32>,
    /// How much the skin there is squeezed towards the pusher's axis: 1
    /// untouched, towards 0 on the axis.
    squeeze: f32,
}

/// A point and its normal moved out of a pusher's way.
///
/// A leaf `d` from the pusher's axis moves out to `√(d² + c²)`, where `c` is
/// the hole's radius: one on the axis moves out by `c`, one far away hardly
/// at all. The map only ever grows with `d`, so leaves keep their order out
/// from the axis and the skin never folds over itself, however deep the
/// body is. It compresses the skin towards the axis by `a = d/√(d² + c²)`
/// and stretches it round the axis by `1/a`; the normal takes the inverse
/// of that, so each leaf keeps its shading, turned as the skin is.
///
/// Out from a capsule's axis, not from its rounded ends: leaves part round a
/// body wading through them and close over nothing, so a body opens a
/// channel through a clump rather than wearing the leaves as a lid. Past its
/// ends the hole closes over its radius. A sphere (a pusher whose segment is
/// a point) opens a round hole, every way.
fn parted(position: Point3<f32>, normal: Vector3<f32>, pusher: &LocalPusher) -> Parted {
    let untouched = Parted {
        position,
        normal,
        squeeze: 1.0,
    };
    let axis = pusher.b - pusher.a;
    let length = axis.magnitude();
    let (core, along, closing) = if length > 1e-6 {
        let along = axis / length;
        let t = (position - pusher.a).dot(&along);
        let past = (-t).max(t - length).max(0.0);
        let closing = (1.0 - past / pusher.radius).clamp(0.0, 1.0);
        (
            pusher.a + along * t.clamp(0.0, length),
            Some(along),
            closing,
        )
    } else {
        (pusher.a, None, 1.0)
    };
    let hole = pusher.radius * pusher.firmness * closing;
    if hole <= 0.0 {
        return untouched;
    }
    let mut offset = position - core;
    if let Some(along) = along {
        offset -= along * offset.dot(&along);
    }
    let distance = offset.magnitude();
    if distance < 1e-5 {
        return untouched;
    }
    let out = offset / distance;
    let moved = (distance * distance + hole * hole).sqrt();
    let squeeze = distance / moved;
    // The normal through the inverse transpose: out from the axis it is
    // divided by the squeeze, round it multiplied by it, along it unchanged.
    let n_out = normal.dot(&out);
    let n_along = along.map_or(0.0, |a| normal.dot(&a));
    let n_round = normal - out * n_out - along.map_or(Vector3::zeros(), |a| a * n_along);
    let turned = out * (n_out / squeeze)
        + n_round * squeeze
        + along.map_or(Vector3::zeros(), |a| a * n_along);
    Parted {
        position: position + out * (moved - distance),
        normal: turned.try_normalize(1e-9).unwrap_or(normal),
        squeeze,
    }
}

/// A point and its normal pressed onto a surface: untouched above the knee,
/// eased onto the surface below it, never through it.
fn pressed(
    position: Point3<f32>,
    normal: Vector3<f32>,
    sprig: f32,
    plane: &LocalPlane,
    give: &Give,
) -> (Point3<f32>, Vector3<f32>) {
    let (point, surface, firmness) = (plane.point, plane.normal, plane.firmness);
    let height = (position - point).dot(&surface);
    if height >= give.knee {
        return (position, normal);
    }
    let knee = give.knee;
    let depth = ((knee - height) / knee).clamp(0.0, 1.0);
    // Eased onto the surface, plus its own sprig: pressed leaves stay leafy.
    let eased = knee * ((height - knee) / knee).exp() + sprig * depth;
    let lift = (eased - height) * firmness;
    let across = (position - point) - surface * height;
    let spread = across
        .try_normalize(1e-6)
        .map_or(Vector3::zeros(), |d| d * lift * give.bulge);
    // Leaves pressed flat on a surface face into it: the clump's underside.
    let flatness = depth * firmness;
    let turned = normal.lerp(&-surface, flatness);
    (
        position + surface * lift + spread,
        turned.try_normalize(1e-9).unwrap_or(-surface),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A skin of these vertices, without sprigs or triangles.
    fn skin_of(vertices: &[Vertex]) -> Skin {
        Skin {
            vertices: vertices.to_vec(),
            indices: Vec::new(),
            sprigs: vec![0.0; vertices.len()],
        }
    }

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
        let bending = Bending {
            press,
            ..Bending::default()
        };
        deform(
            &skin_of(&rest),
            &Isometry3::identity(),
            &bending,
            &give,
            &mut out,
        );
        assert!(out[0].pos.y > -0.3, "a leaf went through: {}", out[0].pos.y);
        assert!(out[0].pos.y < -0.3 + give.knee);
        assert_eq!(out[1].pos, rest[1].pos);
    }

    /// Leaves in a body's way are moved out to its surface, and leaves well
    /// clear of it hardly at all; leaves keep their order out from the body,
    /// so the skin never folds.
    #[test]
    fn a_body_parts_the_leaves_round_it() {
        let give = Give::default();
        let mut parting = Parting::default();
        let pusher = Pusher {
            a: Point3::new(0.0, -0.25, 0.0),
            b: Point3::new(0.0, 0.25, 0.0),
            radius: 0.25,
        };
        parting.push(
            RigidBodyHandle(generational_arena::Index::from_raw_parts(0, 0)),
            pusher,
            &give,
        );
        let rest: Vec<Vertex> = (0..=40)
            .map(|i| vertex(Vector3::new(0.01 + i as f32 * 0.05, 0.0, 0.0)))
            .collect();
        let mut out = Vec::new();
        let bending = Bending {
            parting,
            ..Bending::default()
        };
        deform(
            &skin_of(&rest),
            &Isometry3::identity(),
            &bending,
            &give,
            &mut out,
        );
        assert!(
            out[0].pos.x >= 0.25 * PARTING_SHARE,
            "a leaf in the body's way is still inside it, at {:.3}",
            out[0].pos.x
        );
        let far = out.last().unwrap().pos.x - rest.last().unwrap().pos.x;
        assert!(far < 0.02, "a leaf 2 m away moved {far:.3} m");
        for pair in out.windows(2) {
            assert!(pair[1].pos.x > pair[0].pos.x, "the skin folded");
        }
    }

    /// The hole closes past a capsule's ends: a leaf above its head stays
    /// put.
    #[test]
    fn leaves_close_over_nothing_past_a_capsules_end() {
        let give = Give::default();
        let mut parting = Parting::default();
        parting.push(
            RigidBodyHandle(generational_arena::Index::from_raw_parts(0, 0)),
            Pusher {
                a: Point3::new(0.0, -0.25, 0.0),
                b: Point3::new(0.0, 0.25, 0.0),
                radius: 0.25,
            },
            &give,
        );
        let above = [vertex(Vector3::new(0.1, 0.6, 0.0))];
        let mut out = Vec::new();
        let bending = Bending {
            parting,
            ..Bending::default()
        };
        deform(
            &skin_of(&above),
            &Isometry3::identity(),
            &bending,
            &give,
            &mut out,
        );
        assert_eq!(out[0].pos, above[0].pos);
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
