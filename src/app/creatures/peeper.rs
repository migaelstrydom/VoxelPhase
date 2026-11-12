//! Peeper — a one-eyed stalker that walks on stilts.
//!
//! The counterweight to the roller. A roller is heavy, blind-ish, and
//! unkillable; you deal with one by using the terrain. A peeper is the
//! opposite on every axis: it is fragile, it sees a long way, and it can
//! be killed outright — but it is *tall*, which means it spots you across
//! ground a roller would never see you over, and its neck reaches further
//! than its body suggests.
//!
//! Its one trick is the eye. A peeper with nothing to look at stands
//! dozing with its lid shut, which makes it a landmark rather than a
//! threat: the player can creep past one, and the snap of the eye opening
//! is the whole alert beat. That is why it carries no
//! [`AlertTelegraph`](crate::creature::AlertTelegraph) — the hop-and-
//! shiver exists for creatures with no face to pull, and this one is
//! mostly face.
//!
//! Everything below the eye is shared: it walks on the same
//! [`LeggedLocomotion`](crate::animation::LeggedLocomotion) as the player
//! and the heart critter, through the same character control chain, driven
//! by the same [`Brain`] as the roller. `visual_bench peeper` shows it.
//!
//! ```text
//!   eye      a man's eye the size of his head, bloodshot, rolling in its
//!            socket to follow what it watches                    (Eye)
//!   lids     an upper and a lower, heavy, raw at their rims, a few
//!            lashes left; shut when it dozes                     (Hinge)
//!   socket   a cuff of creased skin the lids slide under, torn open
//!            behind, and the muscles that grip the ball there    (Head)
//!   neck     cords and a windpipe under thin skin                (Neck)
//!   rump     a plucked carcass with its wing stubs and a few
//!            quills left in its tail                             (Hips)
//!   legs     a wader's                                           (the bones)
//! ```

use std::f32::consts::{PI, TAU};
use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Entity, World};

use super::anatomy;
use super::bird_anatomy::{
    bristles, cord, placed, plucked, shade_relief, surface_toward, BirdLeg, Bristle, PLUCKED,
};
use super::likeness;
use super::palette::{self, Paint, BESTIARY};
use super::stilt_walker::{Peck, StiltWalker};
use crate::animation::stilts::{Carriage, Dress, Drive, Joint, StiltRigConfig, Worn};
use crate::animation::FootPlacerConfig;
use crate::app::sculpt::{
    scatter, Blush, Clay, Ellipsoid, Grime, Lump, Mottle, Relief, Spots, VertexPaint,
};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::creature::Brain;
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;
use crate::utils::noise::fbm_perlin_3d;

/// The name its look is kept under.
const KIND: &str = "peeper";

/// Spindly. A peeper is legs and air, and a blast should send it
/// cartwheeling — being thrown about is most of what makes it fun to
/// fight.
const DENSITY: f32 = 300.0;

/// Yaw authority, in rad/s². Below the heart critter's: a peeper pivots
/// deliberately, so circling it is a real option.
const TURN_AUTHORITY: f32 = 420.0;

/// Seconds of wind-up before a peck lands, and of recovery after. The
/// wind-up is long on purpose: it is the warning, and the head rearing
/// back over the haunch is unmistakable.
const PECK_WINDUP: f32 = 0.5;
const PECK_RECOVERY: f32 = 0.85;

/// Radius of the eye that is its head, m, and how far it can roll off the
/// way the head faces, radians.
const EYE_RADIUS: f32 = 0.2;
const EYE_SWIVEL: f32 = 0.85;
/// How wide the iris is, as the angle from the eye's axis to its edge,
/// radians, and how much of that the pupil takes.
const IRIS_ANGLE: f32 = 0.5;
const PUPIL_SHARE: f32 = 0.3;
/// How far the cornea stands proud of the ball, as a share of its radius,
/// and how far past the iris its swelling runs, as a share of the iris.
const CORNEA: f32 = 0.05;
const CORNEA_REACH: f32 = 1.3;
/// Rings the eye is drawn with: across its pupil, across its iris, over
/// the white as far as `EYE_FRONT` radians from its axis, and behind that
/// where it shows only through the tears in the socket. Segments round it:
/// enough to carry a vein.
const EYE_RINGS: [usize; 4] = [4, 20, 30, 32];
const EYE_FRONT: f32 = 1.5;
const EYE_SEGMENTS: usize = 176;
/// Half the width of the edge between the pupil and the iris and between
/// the iris and the white, radians: a ring is laid either side of each.
const EYE_EDGE: f32 = 0.004;

/// The eye's paints (linear): its white, the white gone yellow toward the
/// back, blood, the iris, the iris round the pupil, and the milky arc age
/// lays inside its rim.
const SCLERA: Vector3<f32> = Vector3::new(0.74, 0.69, 0.57);
const JAUNDICE: Vector3<f32> = Vector3::new(0.7, 0.55, 0.33);
const BLOOD: Vector3<f32> = Vector3::new(0.46, 0.04, 0.035);
const IRIS: Vector3<f32> = Vector3::new(0.2, 0.23, 0.19);
const IRIS_HEART: Vector3<f32> = Vector3::new(0.3, 0.17, 0.05);
const ARCUS: Vector3<f32> = Vector3::new(0.42, 0.46, 0.47);
/// Where a vessel has burst in the white, as the way there from the eye's
/// middle, and how far the blood has spread, as a share of the radius.
const BURST_WAY: Vector3<f32> = Vector3::new(-0.6, -0.42, 0.68);
const BURST_REACH: f32 = 0.26;

/// The eye, its lids and its socket lie one inside the next, each a solid
/// whose skin shows only where the one outside it is cut away. Radius of
/// the lids' skin, clear of the cornea, and of the socket's, clear of the
/// lids, m.
const LID_RADIUS: f32 = EYE_RADIUS * (1.0 + CORNEA) + 0.008;
const SOCKET_RADIUS: f32 = LID_RADIUS + 0.012;
/// How far inside the eye's own surface a lid's or the socket's skin is
/// no longer kept: it can never be seen there.
const BURIED: f32 = 0.008;
/// Side of the grid the lids and the socket are skinned from, m.
const LID_GRAIN: f32 = 0.0065;
const SOCKET_GRAIN: f32 = 0.007;

/// Skin of the lids and of the socket, and the raw edge of either
/// (linear).
const LID_SKIN: (f32, f32, f32) = (0.42, 0.27, 0.24);
const SOCKET_SKIN: (f32, f32, f32) = (0.47, 0.35, 0.29);
const RAW: (f32, f32, f32) = (0.46, 0.09, 0.075);
/// The brown blots age leaves on skin.
const AGE_SPOTS: Spots = Spots {
    frequency: 34.0,
    presence: 0.3,
    radius: (0.14, 0.42),
    tint: Vector3::new(0.66, 0.5, 0.4),
    salt: 237,
};
/// How far along a lid's edge its lashes grow, radians either side of its
/// middle: as far as the socket lets the edge show.
const LASH_REACH: f32 = 0.72;
/// Muscle, the tendon it ends in, a vein, and the gristle of a windpipe
/// (linear).
const MUSCLE: (f32, f32, f32) = (0.3, 0.045, 0.04);
const TENDON: (f32, f32, f32) = (0.58, 0.45, 0.35);
const VEIN: (f32, f32, f32) = (0.2, 0.1, 0.17);
const GRISTLE: (f32, f32, f32) = (0.56, 0.45, 0.38);
/// Skin of the neck, and the bone of the breast showing through the skin
/// over it (linear).
const NECK_SKIN: (f32, f32, f32) = (0.5, 0.33, 0.27);
const KEEL: (f32, f32, f32) = (0.62, 0.52, 0.36);
/// A lash, and a quill left in the rump (linear).
const LASH: (f32, f32, f32) = (0.035, 0.03, 0.028);
const QUILL: (f32, f32, f32) = (0.62, 0.56, 0.42);

/// Where the muscles that grip the eye gather, low on the back of the
/// ball: the way there from the eye's middle.
const KNOT_WAY: Vector3<f32> = Vector3::new(0.0, -0.93, -0.36);

/// Its legs: a wader's, sallow and thin.
const LEGS: BirdLeg = BirdLeg {
    girth: 0.05,
    shank: 0.0155,
    toe: 0.15,
    skin: PLUCKED,
    hide: (0.5, 0.4, 0.24),
    claw: (0.06, 0.05, 0.045),
    grain: 0.0039,
    drumstick_grain: 0.0055,
};

/// 0 at or below `from`, 1 at or above `to`, and smooth between.
fn ease(from: f32, to: f32, x: f32) -> f32 {
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// How near a line of the noise `field` a point is, 0 to 1: `thin` sets
/// how narrow the line.
fn ridge(field: f32, thin: f32) -> f32 {
    (1.0 - field.abs() * thin).clamp(0.0, 1.0)
}

/// The colour of the eye `off` radians from its axis and `round` radians
/// round it.
fn eye_paint(off: f32, round: f32) -> Vector3<f32> {
    let pupil = IRIS_ANGLE * PUPIL_SHARE;
    let (sin, cos) = round.sin_cos();
    if off < pupil {
        return Vector3::repeat(0.008);
    }
    if off < IRIS_ANGLE {
        let out = (off - pupil) / (IRIS_ANGLE - pupil);
        // Fibres running out from the pupil, hollows between them, an
        // ochre heart, and a dark ring where the iris meets the white.
        let fibre = fbm_perlin_3d(cos * 21.0, sin * 21.0, out * 1.6, 3, 0.6, 201);
        let hollow = fbm_perlin_3d(cos * 7.0, sin * 7.0, out * 5.0, 2, 0.5, 202).max(0.0);
        let heart = 1.0 - ease(0.05, 0.42, out + 0.2 * fibre);
        let ring = ease(0.72, 1.0, out);
        let arc = ease(0.5, 0.68, out) * (1.0 - ease(0.74, 0.9, out));
        let lit = (0.95 - 0.2 * out + 0.6 * fibre) * (1.0 - 0.7 * hollow) * (1.0 - 0.8 * ring);
        // A ruff of darker stuff round the pupil's own edge.
        let ruff = 0.55 + 0.45 * ease(0.0, 0.07, out);
        return IRIS.lerp(&IRIS_HEART, heart).lerp(&ARCUS, 0.3 * arc) * lit.max(0.12) * ruff;
    }
    // The white: vessels that come up from behind and thin toward the
    // iris, finer ones between them, and the whole of it redder toward the
    // back and toward its two corners. Behind, where they come from, they
    // run every way.
    let at = Vector3::new(off.sin() * cos, off.sin() * sin, off.cos());
    let behind = ((off - IRIS_ANGLE) / (EYE_FRONT - IRIS_ANGLE)).clamp(0.0, 1.0);
    let trunk = ridge(
        fbm_perlin_3d(cos * 4.5, sin * 4.5, off * 2.4, 3, 0.55, 203),
        5.0,
    );
    let twig = ridge(
        fbm_perlin_3d(cos * 11.0, sin * 11.0, off * 4.5, 3, 0.55, 204),
        6.0,
    );
    let web = ridge(
        fbm_perlin_3d(at.x * 3.6, at.y * 3.6, at.z * 3.6, 3, 0.55, 206),
        2.8,
    );
    let corner = cos * cos;
    let fore = 1.0 - ease(0.85, 1.0, behind);
    let red = fore
        * (trunk.powf(1.5) * (0.3 + 0.7 * behind) * (0.6 + 0.4 * corner)
            + 0.55 * twig * twig * behind * (0.4 + 0.6 * corner))
        + (1.0 - fore) * 0.8 * web * web
        + 0.3 * behind * behind * (0.45 + 0.55 * corner);
    let mut colour = SCLERA
        .lerp(&JAUNDICE, 0.65 * behind)
        .lerp(&BLOOD, red.clamp(0.0, 0.92));
    // The shadow the rim of the iris lays on the white.
    colour *= 0.66 + 0.34 * ease(0.0, 0.06, off - IRIS_ANGLE);
    // The burst vessel: a blot with a torn edge.
    let torn = 0.3 * fbm_perlin_3d(at.x * 13.0, at.y * 13.0, at.z * 13.0, 2, 0.5, 205);
    let from_burst = (at - BURST_WAY.normalize()).norm() / BURST_REACH;
    let blot = 1.0 - ease(0.45, 1.0, from_burst + torn);
    colour.lerp(&(BLOOD * 0.8), 0.9 * blot)
}

/// The eye: a ball about its own middle, looking along +Z, drawn as rings
/// about that axis so that the pupil and the iris have clean edges and the
/// front of it is finer than the back.
fn eyeball() -> MeshPrimitive {
    let pupil = IRIS_ANGLE * PUPIL_SHARE;
    // The rings' angles from the axis, a pair close either side of each
    // edge.
    let spans = [
        (0.0, pupil - EYE_EDGE),
        (pupil + EYE_EDGE, IRIS_ANGLE - EYE_EDGE),
        (IRIS_ANGLE + EYE_EDGE, EYE_FRONT),
        (EYE_FRONT, PI),
    ];
    let mut offs = Vec::new();
    for (span, rings) in spans.iter().zip(EYE_RINGS) {
        for ring in 0..rings {
            offs.push(span.0 + (span.1 - span.0) * ring as f32 / (rings - 1) as f32);
        }
    }
    // The first is the pole in front and the last the pole behind.
    let (first, last) = (offs.remove(0), offs.pop().unwrap_or(PI));

    let swelling = IRIS_ANGLE * CORNEA_REACH;
    let point = |off: f32, round: f32| {
        let (sin_off, cos_off) = off.sin_cos();
        let (sin, cos) = round.sin_cos();
        let out = Vector3::new(sin_off * cos, sin_off * sin, cos_off);
        // The way round the ball from the axis: the cornea's swelling
        // tips the surface that way.
        let away = Vector3::new(cos_off * cos, cos_off * sin, -sin_off);
        let x = (off / swelling).min(1.0);
        let swell = 1.0 + CORNEA * (1.0 - x * x).powi(2);
        let slope = -4.0 * CORNEA * (1.0 - x * x) * x / swelling;
        let colour = eye_paint(off, round);
        Vertex {
            pos: out * EYE_RADIUS * swell,
            color: Vector4::new(colour.x, colour.y, colour.z, 1.0),
            tex_coords: Vector2::zeros(),
            normal: (out * swell - away * slope).normalize(),
            ao: 1.0,
        }
    };

    let mut vertices = vec![point(first, 0.0)];
    for off in &offs {
        for segment in 0..EYE_SEGMENTS {
            vertices.push(point(*off, segment as f32 / EYE_SEGMENTS as f32 * TAU));
        }
    }
    vertices.push(point(last, 0.0));

    let segments = EYE_SEGMENTS as u32;
    let rings = offs.len() as u32;
    let at = |ring: u32, segment: u32| 1 + ring * segments + segment % segments;
    let behind = 1 + rings * segments;
    let mut indices = Vec::new();
    for segment in 0..segments {
        indices.extend([0, at(0, segment), at(0, segment + 1)]);
        for ring in 0..rings - 1 {
            let (a, b) = (at(ring, segment), at(ring, segment + 1));
            let (c, d) = (at(ring + 1, segment), at(ring + 1, segment + 1));
            indices.extend([a, c, b, b, c, d]);
        }
        indices.extend([behind, at(rings - 1, segment + 1), at(rings - 1, segment)]);
    }
    MeshPrimitive {
        vertices,
        indices,
        material: Paint::Wet.id(),
    }
}

/// The point on the unit ball `round` radians along the great circle
/// through the eye's two corners that passes `lift` radians above straight
/// ahead: 0 is its middle, a quarter turn either way a corner.
fn on_arc(lift: f32, round: f32) -> Vector3<f32> {
    Vector3::new(
        round.sin(),
        round.cos() * lift.sin(),
        round.cos() * lift.cos(),
    )
}

/// The way off that great circle's plane toward the crown.
fn above_arc(lift: f32) -> Vector3<f32> {
    Vector3::new(0.0, lift.cos(), -lift.sin())
}

/// A fold of skin laid along that great circle at `radius` from the eye's
/// middle, `thick` at its middle and thinning toward the corners. It
/// wanders `sway` radians off the circle, each fold in its own way by its
/// `seed`.
fn fold(lift: f32, radius: f32, thick: f32, sway: f32, seed: f32) -> Vec<(Vector3<f32>, f32)> {
    const STEPS: usize = 22;
    const REACH: f32 = 1.45;
    (0..=STEPS)
        .map(|step| {
            let round = REACH * (2.0 * step as f32 / STEPS as f32 - 1.0);
            let off = sway * ((round * 2.3 + seed).sin() + 0.5 * (round * 5.1 + 2.0 * seed).sin());
            (
                on_arc(lift + off, round) * radius,
                thick * (0.5 + 0.5 * round.cos()) * (1.0 + 0.2 * (round * 3.7 + seed).sin()),
            )
        })
        .collect()
}

/// `mesh` without the triangles that lie wholly `BURIED` or deeper inside
/// the eye, where nothing can see them.
fn unburied(mesh: MeshPrimitive) -> MeshPrimitive {
    let shows = |index: &u32| mesh.vertices[*index as usize].pos.norm() > EYE_RADIUS - BURIED;
    let mut renumbered = vec![u32::MAX; mesh.vertices.len()];
    let (mut vertices, mut indices) = (Vec::new(), Vec::new());
    let seen = mesh
        .indices
        .chunks_exact(3)
        .filter(|triangle| triangle.iter().any(shows));
    for &index in seen.flatten() {
        let slot = &mut renumbered[index as usize];
        if *slot == u32::MAX {
            *slot = vertices.len() as u32;
            vertices.push(mesh.vertices[index as usize]);
        }
        indices.push(*slot);
    }
    MeshPrimitive {
        vertices,
        indices,
        material: mesh.material,
    }
}

/// One lid of the eye: a slice of a ball a little larger than the eye that
/// turns about the line through the eye's two corners, the head's X axis.
/// Its angles are about that line, from straight ahead up toward the crown.
struct Lid {
    /// Where its free edge lies with the eye open.
    edge: f32,
    /// Where it ends, out of sight under the socket.
    back: f32,
    /// How far it turns to shut: positive brings the crown forward.
    shut: f32,
    /// How far the line it turns about is rolled off the level, radians:
    /// a lid that hangs lower at one corner.
    droop: f32,
    /// Places for a lash along its edge, some of them empty.
    lashes: u32,
    /// Seed of its creases and of which lashes it has kept.
    seed: u32,
}

/// The upper lid, which never quite clears the iris, and the lower, which
/// rises less far to meet it.
const UPPER_LID: Lid = Lid {
    edge: 0.47,
    back: 1.86,
    shut: 0.61,
    droop: 0.12,
    lashes: 34,
    seed: 211,
};
const LOWER_LID: Lid = Lid {
    edge: -0.52,
    back: -1.4,
    shut: -0.4,
    droop: 0.06,
    lashes: 20,
    seed: 223,
};

impl Lid {
    /// +1 for a lid that lies above its edge, −1 for one below.
    fn up(&self) -> f32 {
        (self.back - self.edge).signum()
    }

    /// How it is rolled off the level.
    fn roll(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.droop)
    }

    /// The joint it rides.
    fn hinge(&self) -> Joint {
        Joint::Hinge {
            pivot: Vector3::zeros(),
            axis: self.roll() * Vector3::x(),
            drive: Drive::Lid { shut: self.shut },
        }
    }

    /// Its skin, the wet margin inside its edge, and its lashes.
    fn meshes(&self) -> Vec<MeshPrimitive> {
        let level = self.level();
        level
            .into_iter()
            .map(|mesh| placed(mesh, self.roll(), Vector3::zeros()))
            .collect()
    }

    /// The same, before it is rolled off the level: turning about X.
    fn level(&self) -> Vec<MeshPrimitive> {
        let up = self.up();
        let seed = self.seed as f32;
        // A slice of a ball with a roll of flesh along its edge and two
        // folds above that, cut off at the edge and where it ends.
        let mut slice = Clay::new(0.012).add(
            Lump::Ball(Ellipsoid::ball(LID_RADIUS, Vector3::zeros())),
            LID_SKIN,
        );
        for (above, sunk, thick, sway) in [
            (0.05, 0.0055, 0.0095, 0.0),
            (0.25, 0.0065, 0.011, 0.035),
            (0.52, 0.006, 0.009, 0.05),
        ] {
            let lift = self.edge + up * above;
            let roll = fold(lift, LID_RADIUS - sunk, thick, sway, seed + above);
            slice = cord(slice, &roll, LID_SKIN);
        }
        let smooth = slice
            .cut(
                Lump::half_space(Vector3::zeros(), above_arc(self.edge) * up),
                0.004,
            )
            .cut(
                Lump::half_space(Vector3::zeros(), -above_arc(self.back) * up),
                0.0,
            );
        // Creased the way it folds, as a lid is that has been held open
        // too long.
        const CREASE: f32 = 0.0019;
        let creased = smooth.clone().relief(Relief::Folds {
            along: Vector3::x(),
            frequency: 42.0,
            depth: CREASE,
            seed: self.seed,
        });
        let mut skin = unburied(anatomy::flesh(&creased, LID_GRAIN));
        shade_relief(
            &mut skin,
            &smooth,
            (-CREASE, CREASE),
            Vector3::new(0.76, 0.66, 0.66),
            Vector3::new(1.05, 1.02, 1.0),
        );
        for vertex in &mut skin.vertices {
            // Raw at its margin, and the whole face of its edge with it.
            let out = vertex.pos.normalize();
            let from_edge = (out.y.atan2(out.z) - self.edge) * up;
            let under = ease(EYE_RADIUS + 0.011, EYE_RADIUS + 0.005, vertex.pos.norm());
            let raw = (0.8 * (1.0 - ease(0.0, 0.05, from_edge))).max(under);
            vertex.color.x *= 1.0 - 0.1 * raw;
            vertex.color.y *= 1.0 - 0.6 * raw;
            vertex.color.z *= 1.0 - 0.58 * raw;
        }
        // The bag of it: bruised where the blood has pooled.
        Blush {
            at: on_arc(self.edge + up * 0.4, 0.25) * LID_RADIUS,
            radius: 0.17,
            tint: Vector3::new(0.72, 0.62, 0.8),
        }
        .paint(&mut skin);

        // Wet where it lies against the eye.
        let margin = fold(
            self.edge - up * 0.004,
            EYE_RADIUS + 0.006,
            0.0062,
            0.0,
            seed,
        );
        let margin = cord(Clay::new(0.004), &margin, RAW).skin(0.0045, Paint::Wet.id());

        let mut lashes = Vec::new();
        for place in 0..self.lashes {
            // A lid this old has lost some, and the rest stand in tufts.
            let share = (place as f32 + 0.5) / self.lashes as f32;
            let kept = 0.45 - 0.5 * (2.0 * share - 1.0).powi(2);
            if scatter(place, self.seed) > kept {
                continue;
            }
            let round = LASH_REACH * (2.0 * share - 1.0) + 0.03 * scatter(place, self.seed + 1);
            let out = on_arc(self.edge + up * 0.012, round);
            let over = -above_arc(self.edge) * up;
            let long = 0.02 + 0.03 * (0.5 + 0.5 * scatter(place, self.seed + 2)).powi(2);
            let grows = out * (0.75 + 0.3 * scatter(place, self.seed + 3))
                + over * 0.7
                + Vector3::x() * 0.35 * scatter(place, self.seed + 4);
            lashes.push(Bristle {
                root: out * (LID_RADIUS - 0.003),
                grows: grows.normalize() * long,
                bend: (out * 0.4 - over * 0.2) * long,
                thick: 0.0012,
            });
        }
        vec![skin, margin, bristles(&lashes, LASH, Paint::Horn.id())]
    }
}

/// The socket: creased skin drawn over the ball, which the lids slide
/// under, open in front where the eye looks out and torn open behind,
/// where the ball lies bare. In the head's frame.
fn socket() -> MeshPrimitive {
    let r = EYE_RADIUS;
    let outer = SOCKET_RADIUS;
    // What the eye looks out through, and the tears behind.
    let opening = Ellipsoid::new((1.0 * r, 0.9 * r, 0.9 * r), (0.0, 0.04 * r, 0.95 * r));
    let tears = [
        Ellipsoid::ball(0.9 * r, Vector3::new(0.0, 0.1 * r, -1.0 * r)),
        Ellipsoid::ball(0.62 * r, Vector3::new(0.1 * r, -0.72 * r, -0.72 * r)),
        Ellipsoid::ball(0.3 * r, Vector3::new(-0.7 * r, 0.62 * r, -0.74 * r)),
        Ellipsoid::ball(0.24 * r, Vector3::new(0.92 * r, -0.05 * r, -0.78 * r)),
    ];

    let mut cuff = Clay::new(0.02).add(
        Lump::Ball(Ellipsoid::ball(outer, Vector3::zeros())),
        SOCKET_SKIN,
    );
    // The brow, heavier to one side, the furrows above it and the bags
    // under the eye: each a fold as (lift, sunk, thick, sway).
    let brow: Vec<_> = (0..=14)
        .map(|step| {
            let round = 1.25 * (step as f32 / 7.0 - 1.0);
            let lift = 1.0 - 0.1 * round + 0.05 * (round * 2.3).sin();
            (
                on_arc(lift, round) * (outer - 0.006),
                0.017 * (0.6 + 0.4 * round.cos()) * (1.0 + 0.18 * round),
            )
        })
        .collect();
    cuff = cord(cuff, &brow, SOCKET_SKIN);
    cuff = cuff.smoothing(0.012);
    for (lift, sunk, thick, sway) in [
        (1.3, 0.004, 0.0085, 0.05),
        (1.58, 0.0035, 0.007, 0.07),
        (1.9, 0.003, 0.006, 0.06),
        (-0.93, 0.006, 0.013, 0.03),
        (-1.14, 0.004, 0.0085, 0.05),
        (-1.4, 0.003, 0.006, 0.05),
    ] {
        let furrow = fold(lift, outer - sunk, thick, sway, 7.0 * lift);
        cuff = cord(cuff, &furrow, SOCKET_SKIN);
    }
    // Crow's feet: short creases fanning back from each corner.
    cuff = cuff.smoothing(0.006);
    for side in [-1.0f32, 1.0] {
        for (from, to) in [(0.3f32, 0.75f32), (0.05, 0.12), (-0.22, -0.6)] {
            let crease = [
                (on_arc(from, side * 1.02) * (outer - 0.002), 0.0055),
                (
                    on_arc(0.5 * (from + to), side * 1.2) * (outer - 0.002),
                    0.0045,
                ),
                (on_arc(to, side * 1.4) * (outer - 0.002), 0.002),
            ];
            cuff = cord(cuff, &crease, SOCKET_SKIN);
        }
    }
    // Veins standing under the skin of the temple and the crown: each from
    // one place to another on the ball, as (round, lift), wandering.
    for (from, to, wander, seed) in [
        ((0.9f32, 0.5f32), (0.25f32, 1.9f32), 0.16f32, 1.0f32),
        ((-1.1, 0.2), (-0.35, 2.1), 0.2, 2.0),
        ((-1.25, -0.5), (-0.9, 1.2), 0.12, 3.0),
        ((1.3, -0.3), (0.9, -1.5), 0.15, 4.0),
    ] {
        const STEPS: usize = 14;
        let vein: Vec<_> = (0..=STEPS)
            .map(|step| {
                let t = step as f32 / STEPS as f32;
                let round = from.0 + (to.0 - from.0) * t + wander * (t * 9.0 + seed).sin();
                let lift = from.1 + (to.1 - from.1) * t + wander * (t * 7.0 + seed).cos();
                (on_arc(lift, round) * (outer - 0.002), 0.0046 - 0.002 * t)
            })
            .collect();
        cuff = cord(cuff, &vein, VEIN);
    }

    let mut smooth = cuff.cut(Lump::Ball(opening), 0.007);
    for tear in tears {
        smooth = smooth.cut(Lump::Ball(tear), 0.004);
    }
    const CREASE: f32 = 0.0014;
    let creased = smooth.clone().relief(Relief::Folds {
        along: Vector3::x(),
        frequency: 30.0,
        depth: CREASE,
        seed: 235,
    });
    let mut skin = unburied(anatomy::flesh(&creased, SOCKET_GRAIN));
    shade_relief(
        &mut skin,
        &smooth,
        (-CREASE, CREASE),
        Vector3::new(0.8, 0.72, 0.7),
        Vector3::new(1.04, 1.02, 1.0),
    );
    AGE_SPOTS.paint(&mut skin);
    for vertex in &mut skin.vertices {
        let front = opening.distance_outside(vertex.pos);
        let torn = tears
            .iter()
            .map(|tear| tear.distance_outside(vertex.pos))
            .fold(f32::MAX, f32::min);
        // Dark round the eye, as skin is that has not slept.
        let tired = 1.0 - ease(0.0, 0.085, front);
        vertex.color.x *= 1.0 - 0.3 * tired;
        vertex.color.y *= 1.0 - 0.42 * tired;
        vertex.color.z *= 1.0 - 0.34 * tired;
        // Raw where it is torn, and the flesh under the skin laid open
        // there.
        let under = ease(outer - 0.004, outer - 0.011, vertex.pos.norm());
        let raw = (1.0 - ease(0.0, 0.022, torn)).max(under * ease(0.0, 0.03, front));
        vertex.color.x *= 1.0 - 0.15 * raw;
        vertex.color.y *= 1.0 - 0.68 * raw;
        vertex.color.z *= 1.0 - 0.68 * raw;
    }
    skin
}

/// The muscles that grip the ball where the socket is torn open: straps
/// from under the socket's edge, down the bare back of the eye, to a knot
/// low behind it. In the head's frame.
fn grip() -> MeshPrimitive {
    let r = EYE_RADIUS;
    let gather = KNOT_WAY.normalize();
    // The way from the eye's middle to where each strap comes out from
    // under the socket, and how thick its belly is, m.
    let straps: [(Vector3<f32>, f32); 5] = [
        (Vector3::new(0.06, 0.81, -0.59), 0.024),
        (Vector3::new(-0.68, 0.35, -0.64), 0.021),
        (Vector3::new(0.7, 0.25, -0.67), 0.02),
        (Vector3::new(-0.8, -0.3, -0.52), 0.017),
        (Vector3::new(0.82, -0.25, -0.5), 0.019),
    ];
    let mut clay = Clay::new(0.009);
    for (from, belly) in straps {
        const STEPS: usize = 7;
        let from = from.normalize();
        let angle = from.dot(&gather).clamp(-1.0, 1.0).acos();
        let strap: Vec<_> = (0..=STEPS)
            .map(|step| {
                let t = step as f32 / STEPS as f32;
                let way =
                    (from * ((1.0 - t) * angle).sin() + gather * (t * angle).sin()) / angle.sin();
                // Standing off the ball most at its belly.
                let stand = 0.004 + 0.012 * (PI * t).sin();
                (
                    way * (r + stand),
                    belly * (0.42 + 0.58 * (PI * t.powf(0.8)).sin()),
                )
            })
            .collect();
        // A tendon where it leaves the socket, then meat.
        clay = cord(clay, &strap[..2], TENDON);
        clay = cord(clay, &strap[1..], MUSCLE);
    }
    // The knot they gather in.
    let knot = gather * (r + 0.018);
    clay = clay.smoothing(0.02);
    for (aside, size) in [
        (Vector3::zeros(), 0.04),
        (Vector3::new(0.03, 0.02, 0.0), 0.028),
        (Vector3::new(-0.034, 0.012, 0.004), 0.026),
    ] {
        clay = clay.add(Lump::Ball(Ellipsoid::ball(size, knot + aside)), MUSCLE);
    }
    // A vein across them.
    clay = clay.smoothing(0.006);
    let vein: Vec<_> = (0..=10)
        .map(|step| {
            let t = step as f32 / 10.0;
            let way = Vector3::new(
                -0.55 + 0.9 * t,
                0.2 - 0.75 * t + 0.1 * (t * 8.0).sin(),
                -0.8,
            );
            (way.normalize() * (r + 0.026), 0.0042)
        })
        .collect();
    clay = cord(clay, &vein, VEIN);

    let mut mesh = clay
        .relief(Relief::Lumps {
            frequency: 55.0,
            depth: 0.001,
            seed: 241,
        })
        .skin(0.0075, Paint::Wet.id());
    Mottle {
        frequency: 30.0,
        value: 0.16,
        warmth: 0.1,
        seed: 242,
    }
    .paint(&mut mesh);
    mesh
}

/// The neck: a windpipe and cords under thin plucked skin, from the
/// shoulder up to the underside of the socket and to the knot behind it.
/// In the neck's frame, the eye's middle `rig.neck_length` up it.
fn neck(rig: &StiltRigConfig) -> MeshPrimitive {
    let head = Vector3::y() * rig.neck_length;
    let knot = head + KNOT_WAY.normalize() * (EYE_RADIUS + 0.018);
    let throat = head - Vector3::y() * (SOCKET_RADIUS - 0.012);
    // The column of it, swelling where it roots in the rump and forking
    // above: forward to the throat and back to the knot.
    let root = Vector3::new(0.0, -0.02, 0.0);
    let fork = Vector3::new(0.0, 0.6 * throat.y, -0.012);
    let limb =
        |from: Vector3<f32>, to: Vector3<f32>, radii: (f32, f32)| Lump::Limb { from, to, radii };
    let mut clay = Clay::new(0.028)
        .add(Lump::Ball(Ellipsoid::ball(0.05, root)), NECK_SKIN)
        .add(limb(root, fork, (0.031, 0.019)), NECK_SKIN)
        .add(
            limb(
                fork,
                throat + Vector3::new(0.0, -0.01, -0.008),
                (0.019, 0.021),
            ),
            NECK_SKIN,
        )
        .add(limb(fork, knot, (0.018, 0.03)), RAW);
    // Slack skin gathered in rings where it roots: each as its height, how
    // wide it lies and how thick it is.
    clay = clay.smoothing(0.008);
    for (height, girth, thick) in [
        (0.02f32, 0.041f32, 0.0075f32),
        (0.042, 0.034, 0.006),
        (0.062, 0.029, 0.005),
    ] {
        let ring: Vec<_> = (0..=12)
            .map(|step| {
                let round = step as f32 / 12.0 * TAU;
                let sag = 0.006 * (2.0 * round + 90.0 * height).sin();
                let at = Vector3::new(round.sin() * girth, height + sag, round.cos() * girth);
                (at, thick)
            })
            .collect();
        clay = cord(clay, &ring, NECK_SKIN);
    }
    // The windpipe: rings of gristle down the front, no two alike.
    clay = clay.smoothing(0.0045);
    const RINGS: u32 = 15;
    let low = Vector3::new(0.0, 0.032, 0.022);
    let high = throat + Vector3::new(0.0, 0.0, 0.012);
    for ring in 0..RINGS {
        let at = low.lerp(&high, ring as f32 / (RINGS - 1) as f32)
            + Vector3::x() * 0.0015 * scatter(ring, 253);
        let wide = 0.0125 * (1.0 + 0.12 * scatter(ring, 252));
        clay = clay.add(
            Lump::mass((wide, 0.0056, wide), (at.x, at.y, at.z)),
            GRISTLE,
        );
    }
    // The cords: each from a place on the shoulder, as (x, z), to where it
    // takes hold, drawn in toward the neck's middle on the way.
    clay = clay.smoothing(0.011);
    for (from, to, thick) in [
        (
            (0.032f32, -0.014f32),
            knot + Vector3::new(0.045, 0.0, -0.01),
            0.0095f32,
        ),
        (
            (-0.032, -0.014),
            knot + Vector3::new(-0.045, 0.005, -0.004),
            0.009,
        ),
        (
            (0.03, 0.02),
            throat + Vector3::new(0.075, 0.03, 0.035),
            0.0085,
        ),
        (
            (-0.028, 0.022),
            throat + Vector3::new(-0.08, 0.026, 0.02),
            0.009,
        ),
        (
            (0.0, -0.034),
            knot + Vector3::new(0.0, -0.02, -0.02),
            0.0105,
        ),
    ] {
        let from = Vector3::new(from.0, -0.01, from.1);
        let mut waist = from.lerp(&to, 0.5);
        waist.x *= 0.62;
        waist.z *= 0.7;
        clay = cord(
            clay,
            &[(from, thick), (waist, 0.8 * thick), (to, 1.15 * thick)],
            TENDON,
        );
    }
    // A vein up one side of it.
    clay = clay.smoothing(0.005);
    let vein: Vec<_> = (0..=10)
        .map(|step| {
            let t = step as f32 / 10.0;
            let round = 0.9 + 0.5 * (5.0 * t).sin();
            let girth = 0.03 - 0.008 * t;
            let at = Vector3::new(round.sin() * girth, 0.01 + 0.15 * t, round.cos() * girth);
            (at, 0.0042)
        })
        .collect();
    clay = cord(clay, &vein, VEIN);

    let mut mesh = plucked(
        &clay.relief(Relief::Lumps {
            frequency: 55.0,
            depth: 0.0013,
            seed: 251,
        }),
        0.005,
    );
    // Rawer toward the eye, where the skin gives out.
    let top = knot.y;
    for vertex in &mut mesh.vertices {
        let raw = 0.8 * ease(top - 0.13, top, vertex.pos.y);
        vertex.color.x *= 1.0 - 0.15 * raw;
        vertex.color.y *= 1.0 - 0.6 * raw;
        vertex.color.z *= 1.0 - 0.6 * raw;
    }
    mesh
}

/// The rump: a plucked carcass carried tail down, its breast and keel, the
/// stump the neck stands on, the stub of its tail and two wings that never
/// grew. Its skin, then the quills left in it. In the hips' frame.
fn rump(rig: &StiltRigConfig) -> Vec<MeshPrimitive> {
    let hip = rig.hip_width;
    let carcass = Ellipsoid::new((0.086, 0.078, 0.145), (0.0, 0.055, -0.055)).pitched(-0.3);
    let mut smooth = Clay::new(0.045)
        .add(Lump::Ball(carcass), PLUCKED)
        .add(
            Lump::mass((0.064, 0.07, 0.066), (0.0, 0.035, 0.06)),
            PLUCKED,
        )
        .add(
            Lump::limb(
                (0.0, 0.07, 0.0),
                (0.0, rig.haunch_height + 0.012, 0.0),
                (0.055, 0.036),
            ),
            PLUCKED,
        )
        .add(Lump::ball(0.054, (hip, 0.0, -0.014)), PLUCKED)
        .add(Lump::ball(0.054, (-hip, 0.0, -0.014)), PLUCKED)
        .smoothing(0.018)
        // The keel of its breast, sharp under the skin, and the stub of its
        // tail.
        .add(
            Lump::limb((0.0, 0.1, 0.098), (0.0, -0.035, 0.086), (0.011, 0.016)),
            KEEL,
        )
        .add(
            Lump::limb((0.0, 0.02, -0.17), (0.0, 0.072, -0.243), (0.036, 0.018)),
            anatomy::FLUSH,
        )
        .smoothing(0.012);
    // It is starved: its backbone stands in a row of knobs from the neck
    // to the tail, and its ribs show down each flank.
    let middle = Vector3::new(0.0, 0.055, -0.055);
    let body = smooth.clone();
    smooth = smooth.smoothing(0.007);
    for knob in 0..8 {
        let lean = 0.35 + 0.15 * knob as f32;
        let way = Vector3::new(0.0, lean.cos(), -lean.sin());
        let at = surface_toward(&body, middle, way, 0.3) - way * 0.005;
        smooth = smooth.add(Lump::Ball(Ellipsoid::ball(0.0105, at)), KEEL);
    }
    for side in [-1.0f32, 1.0] {
        for rib in 0..4 {
            let back = -0.015 - 0.032 * rib as f32;
            let spine = Vector3::new(0.0, middle.y + 0.31 * (back - middle.z), back);
            let bone: Vec<_> = (0..=5)
                .map(|step| {
                    let round = 0.3 + 0.22 * step as f32;
                    let way = Vector3::new(side * round.sin(), round.cos(), -0.12 * step as f32);
                    let at = surface_toward(&body, spine, way, 0.3) - way.normalize() * 0.003;
                    (at, 0.0058 - 0.0005 * step as f32)
                })
                .collect();
            smooth = cord(smooth, &bone, PLUCKED);
        }
    }
    // The wings: each folded twice on itself against the flank, all bone
    // and joint, one hung a little lower than the other.
    smooth = smooth.smoothing(0.011);
    for (side, sag) in [(1.0f32, 0.0f32), (-1.0, 0.016)] {
        let at = |x: f32, y: f32, z: f32| Vector3::new(side * x, y - sag, z);
        let wing = [
            (at(0.068, 0.124, 0.03), 0.017),
            (at(0.094, 0.062, -0.1), 0.0115),
            (at(0.094, 0.058, -0.105), 0.0145),
            (at(0.103, 0.116, -0.012), 0.0085),
            (at(0.104, 0.12, -0.008), 0.0115),
            (at(0.1, 0.05, -0.088), 0.003),
        ];
        smooth = cord(smooth, &wing, PLUCKED);
    }
    // Slack skin: a fold in each groin, and two across its back toward the
    // tail.
    smooth = smooth.smoothing(0.009);
    for side in [-1.0f32, 1.0] {
        let groin = [
            (Vector3::new(side * 0.028, -0.022, 0.07), 0.006),
            (Vector3::new(side * 0.07, -0.004, 0.05), 0.0085),
            (Vector3::new(side * 0.1, 0.04, 0.012), 0.005),
        ];
        smooth = cord(smooth, &groin, PLUCKED);
    }
    for (back, sunk) in [(-0.11f32, 0.0f32), (-0.15, 0.012)] {
        let fold: Vec<_> = (-4..=4)
            .map(|step| {
                let across = step as f32 * 0.02;
                let droop = 0.5 * across * across / 0.08;
                let height = 0.108 - sunk - droop + 0.3 * (back + 0.11);
                (Vector3::new(across, height, back), 0.0062)
            })
            .collect();
        smooth = cord(smooth, &fold, PLUCKED);
    }
    // Gooseflesh.
    const PIMPLE: f32 = 0.003;
    let pimpled = smooth.clone().relief(Relief::Scales {
        size: 0.014,
        depth: PIMPLE,
    });
    let mut skin = plucked(&pimpled, 0.0055);
    shade_relief(
        &mut skin,
        &smooth,
        (-PIMPLE, 0.0),
        Vector3::new(0.7, 0.6, 0.6),
        Vector3::new(1.06, 0.98, 0.9),
    );
    // Chafed raw about the tail, and bruised along one flank.
    Blush {
        at: Vector3::new(0.0, 0.03, -0.2),
        radius: 0.09,
        tint: Vector3::new(1.0, 0.62, 0.6),
    }
    .paint(&mut skin);
    Blush {
        at: Vector3::new(0.085, 0.03, -0.03),
        radius: 0.08,
        tint: Vector3::new(0.66, 0.58, 0.78),
    }
    .paint(&mut skin);
    // Fouled underneath.
    Grime {
        below: -0.03,
        fade: 0.07,
        tint: Vector3::new(0.62, 0.52, 0.42),
    }
    .paint(&mut skin);

    // The quills it has kept: a few long ones in its tail, and broken
    // stubs over its back.
    let tail = Vector3::new(0.0, 0.055, -0.22);
    let mut quills = Vec::new();
    for quill in 0..7u32 {
        let fan = (quill as f32 - 3.0) * 0.22 + 0.06 * scatter(quill, 261);
        let way = Vector3::new(fan.sin(), 0.5 + 0.25 * scatter(quill, 262), -fan.cos());
        let long = 0.05 + 0.07 * (0.5 + 0.5 * scatter(quill, 263));
        quills.push(Bristle {
            root: surface_toward(&smooth, tail, way, 0.1) - way.normalize() * 0.004,
            grows: way.normalize() * long,
            bend: Vector3::new(0.0, -0.3 * long, 0.0),
            thick: 0.003,
        });
    }
    for stub in 0..30u32 {
        let way = Vector3::new(
            scatter(stub, 264),
            0.1 + 0.9 * (0.5 + 0.5 * scatter(stub, 265)),
            -0.3 + scatter(stub, 266),
        );
        let long = 0.01 + 0.018 * (0.5 + 0.5 * scatter(stub, 267));
        quills.push(Bristle {
            root: surface_toward(&smooth, middle, way, 0.3) - way.normalize() * 0.003,
            grows: (way.normalize() + Vector3::new(0.0, 0.2, -0.5)).normalize() * long,
            bend: Vector3::zeros(),
            thick: 0.0018,
        });
    }
    vec![skin, bristles(&quills, QUILL, Paint::Horn.id())]
}

/// A stalking creature that pecks.
#[derive(Deserialize)]
pub struct PeeperDef {
    /// Position (x, z). Y comes from the terrain surface, since a creature
    /// authored at a hand-picked height would either hang in the air or
    /// start embedded once the terrain around it is reshaped.
    pub pos: (f32, f32),

    /// Flat-ground top speed in m/s. The player walks at 5.0 and sprints
    /// at 8.0, so a peeper is outrunnable — it is a thing you leave
    /// behind, not a thing you escape.
    #[serde(default = "PeeperDef::default_speed")]
    pub speed: f32,
    /// How far it can see. Long: being tall is the creature's advantage.
    #[serde(default = "PeeperDef::default_sight_range")]
    pub sight_range: f32,
    /// Hit points a peck takes off.
    #[serde(default = "PeeperDef::default_peck_damage")]
    pub peck_damage: f32,
    /// Hit points it can take before it dies.
    #[serde(default = "PeeperDef::default_health")]
    pub health: f32,
}

impl PeeperDef {
    pub fn default_speed() -> f32 {
        3.4
    }
    pub fn default_sight_range() -> f32 {
        28.0
    }
    pub fn default_peck_damage() -> f32 {
        14.0
    }
    pub fn default_health() -> f32 {
        55.0
    }

    /// Its proportions: almost all leg, with an eye the size of a melon.
    pub fn rig() -> StiltRigConfig {
        StiltRigConfig {
            upper_leg_length: 0.52,
            lower_leg_length: 0.5,
            hip_width: 0.07,
            leg_radius: 0.026,
            haunch_height: 0.16,
            haunch_radius: 0.085,
            neck_length: 0.42,
            neck_radius: 0.035,
            head_radius: 0.22,
            rest_pitch: 0.2,
            rear_pitch: -0.55,
            strike_pitch: 1.35,
            lid_rate: 11.0,
            standing_height_ratio: 0.86,
            step_height: 0.22,
            probe_length_factor: 1.3,
            stride_gain: 0.6,
            knee_bend: -1.0,
            foot_placer: FootPlacerConfig::default(),
            carriage: Carriage::default(),
        }
    }

    /// Its dress, against the palette's slots.
    pub fn sculpt() -> Dress {
        let rig = Self::rig();
        let eye = Joint::Eye {
            socket: Vector3::zeros(),
            swivel: EYE_SWIVEL,
        };
        let mut pieces = vec![
            (Worn::on(Joint::Hips).breathing(0.045), rump(&rig)),
            (Worn::on(Joint::Neck), vec![neck(&rig)]),
            (Worn::on(Joint::Head), vec![socket(), grip()]),
            (Worn::on(eye), vec![eyeball()]),
            (Worn::on(UPPER_LID.hinge()), UPPER_LID.meshes()),
            (Worn::on(LOWER_LID.hinge()), LOWER_LID.meshes()),
        ];
        pieces.extend(LEGS.legs(&rig));
        Dress::of(pieces)
    }

    /// Its dress against the palette's slots, sculpted once: for tools
    /// that draw it in the palette's own materials.
    pub fn dress() -> Arc<Dress> {
        likeness::dress_sculpted(KIND, Self::sculpt)
    }

    fn walker(&self, materials: &[MaterialId]) -> StiltWalker {
        StiltWalker {
            at: self.pos,
            rig: Self::rig(),
            dress: likeness::dress(KIND, materials, Self::sculpt),
            speed: self.speed,
            sight_range: self.sight_range,
            density: DENSITY,
            turn_authority: TURN_AUTHORITY,
            peck: Some(Peck {
                damage: self.peck_damage,
                windup: PECK_WINDUP,
                recovery: PECK_RECOVERY,
            }),
            health: Some(self.health),
            brain: Brain::hunter,
        }
    }
}

impl Spawnable for PeeperDef {
    fn material_count(&self) -> usize {
        Paint::ALL.len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(BESTIARY, palette::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        self.walker(materials).spawn(world)
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use super::*;

    #[test]
    fn its_dress_is_tens_of_thousands_of_triangles_and_every_part_is_worn() {
        let started = Instant::now();
        let dress = PeeperDef::sculpt();
        println!("a peeper is sculpted in {:?}", started.elapsed());
        assert_eq!(dress.worn.len(), dress.model.parts.len());
        for (worn, part) in dress.worn.iter().zip(&dress.model.parts) {
            let triangles: usize = part.primitives.iter().map(|m| m.indices.len() / 3).sum();
            println!("  {:?}: {triangles}", worn.joint);
        }
        let triangles = dress.triangles();
        println!("a peeper is {triangles} triangles");
        assert!(
            (30_000..210_000).contains(&triangles),
            "{triangles} triangles"
        );
    }

    #[test]
    fn its_eye_is_dark_at_its_pupil_and_wound_to_face_outward() {
        let eye = eyeball();
        assert!(eye.vertices[0].color.x < 0.05, "no pupil ahead");
        for triangle in eye.indices.chunks_exact(3) {
            let [a, b, c] = [0, 1, 2].map(|k| eye.vertices[triangle[k] as usize].pos);
            let face = (b - a).cross(&(c - a));
            assert!(face.dot(&(a + b + c)) >= 0.0, "the eye is inside out");
        }
    }

    #[test]
    fn its_lids_meet_when_they_shut() {
        let upper = UPPER_LID.edge - UPPER_LID.shut;
        let lower = LOWER_LID.edge - LOWER_LID.shut;
        assert!(
            upper <= lower,
            "a gap between the lids: {upper} over {lower}"
        );
        assert!(lower - upper < 0.1, "the lids cross: {upper} over {lower}");
    }
}
