//! The parts of birds creatures are put together from: a wader's long
//! scaled legs and clawed feet, plucked skin, and the bristles and quills
//! left standing in it.
//!
//! ```text
//!   BirdLeg ── drumstick, tarsus, foot   modelled for the stilt rig's bones
//!      │                                 (`animation::stilts::Joint`) on a rig
//!      │                                 whose knees break backward: the rig's
//!      │                                 knee is a bird's hock
//!      └────── legs()                    both legs, each part on its bone
//!   plucked()        a clay form skinned and painted as plucked skin
//!   shade_relief()   dirt in the creases of a relief, wear on its crests
//!   bristles()       stiff hairs, quills and threads, as one mesh
//!   surface_toward() where a form's surface is, walking out of it or
//!   surface_inward() into it, to root something there
//!   cord()           a tendon, a vein or a roll of skin pressed onto a form
//!   placed()         a mesh modelled in one place, set in another
//! ```
//!
//! A bird's leg is not a man's turned round. What bends backward is its
//! ankle; above that is a drumstick, meat at the top and bare shank below,
//! and under it one long bone in plates of scale down to four toes.

use std::f32::consts::{PI, TAU};

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};

use super::palette::Paint;
use crate::animation::stilts::{Joint, StiltRigConfig, Worn};
use crate::animation::FootSide;
use crate::app::sculpt::{
    scatter, Blush, Clay, Ellipsoid, Grime, Lump, Mottle, Relief, Spots, Veins, VertexPaint,
};
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// Plucked skin as it is pressed in clay (linear): sallow, with the blood
/// showing through.
pub const PLUCKED: (f32, f32, f32) = (0.56, 0.41, 0.31);

/// The uneven colour of plucked skin.
const PLUCKED_MOTTLE: Mottle = Mottle {
    frequency: 14.0,
    value: 0.18,
    warmth: 0.14,
    seed: 141,
};

/// Broad patches where it is chafed red or gone grey with cold.
const PLUCKED_BLOTCHES: Mottle = Mottle {
    frequency: 5.5,
    value: 0.22,
    warmth: 0.38,
    seed: 142,
};

/// Veins under it: thin skin hides little.
const PLUCKED_VEINS: Veins = Veins {
    frequency: 12.0,
    width: 0.04,
    tint: Vector3::new(0.7, 0.74, 0.92),
    seed: 143,
};

/// The pits the feathers came out of, each a little inflamed.
const FOLLICLES: Spots = Spots {
    frequency: 150.0,
    presence: 0.7,
    radius: (0.18, 0.32),
    tint: Vector3::new(0.76, 0.52, 0.47),
    salt: 144,
};

/// Sores and scabs, where it has pecked at itself.
const SORES: Spots = Spots {
    frequency: 21.0,
    presence: 0.14,
    radius: (0.14, 0.3),
    tint: Vector3::new(0.58, 0.3, 0.27),
    salt: 145,
};

/// `clay` skinned from a grid of `grain` metres and painted as the skin of
/// a plucked bird.
pub fn plucked(clay: &Clay, grain: f32) -> MeshPrimitive {
    let mut mesh = clay.skin(grain, Paint::Skin.id());
    PLUCKED_MOTTLE.paint(&mut mesh);
    PLUCKED_BLOTCHES.paint(&mut mesh);
    PLUCKED_VEINS.paint(&mut mesh);
    FOLLICLES.paint(&mut mesh);
    SORES.paint(&mut mesh);
    mesh
}

/// Paint `mesh` by how far its relief stands above or below `smooth`, the
/// same form before the relief was raised on it: `crease` where it is sunk
/// `depths.0` metres or more, `crest` where it stands `depths.1` or more
/// proud, and between them in between. Each multiplies the paint (linear).
pub fn shade_relief(
    mesh: &mut MeshPrimitive,
    smooth: &Clay,
    depths: (f32, f32),
    crease: Vector3<f32>,
    crest: Vector3<f32>,
) {
    for vertex in &mut mesh.vertices {
        // On the raised surface, the smooth form's distance is the raising.
        let raised = smooth.distance(vertex.pos);
        let t = ((raised - depths.0) / (depths.1 - depths.0)).clamp(0.0, 1.0);
        let tint = crease.lerp(&crest, t * t * (3.0 - 2.0 * t));
        vertex.color.x *= tint.x;
        vertex.color.y *= tint.y;
        vertex.color.z *= tint.z;
    }
}

/// The point where `clay`'s surface is met walking from `inside`, a point
/// within it, the way `toward` points, no further than `reach` metres.
pub fn surface_toward(
    clay: &Clay,
    inside: Vector3<f32>,
    toward: Vector3<f32>,
    reach: f32,
) -> Vector3<f32> {
    let toward = toward.normalize();
    let (mut within, mut without) = (0.0, reach);
    for _ in 0..22 {
        let middle = 0.5 * (within + without);
        if clay.distance(inside + toward * middle) < 0.0 {
            within = middle;
        } else {
            without = middle;
        }
    }
    inside + toward * within
}

/// The first point of `clay`'s surface met walking from `outside`, a point
/// clear of it, the way `toward` points, no further than `reach` metres.
/// Nothing, if the walk never enters the form.
pub fn surface_inward(
    clay: &Clay,
    outside: Vector3<f32>,
    toward: Vector3<f32>,
    reach: f32,
) -> Option<Vector3<f32>> {
    const STRIDES: usize = 96;
    let toward = toward.normalize();
    let stride = reach / STRIDES as f32;
    let entered = (1..=STRIDES)
        .map(|step| step as f32 * stride)
        .find(|walked| clay.distance(outside + toward * *walked) < 0.0)?;
    let (mut without, mut within) = (entered - stride, entered);
    for _ in 0..16 {
        let middle = 0.5 * (within + without);
        if clay.distance(outside + toward * middle) < 0.0 {
            within = middle;
        } else {
            without = middle;
        }
    }
    Some(outside + toward * within)
}

/// `clay` with a cord pressed on through `points`, each a place and the
/// cord's radius there, in `colour` (linear): a tendon, a vein, a roll of
/// skin.
pub fn cord(mut clay: Clay, points: &[(Vector3<f32>, f32)], colour: (f32, f32, f32)) -> Clay {
    for pair in points.windows(2) {
        let limb = Lump::Limb {
            from: pair[0].0,
            to: pair[1].0,
            radii: (pair[0].1, pair[1].1),
        };
        clay = clay.add(limb, colour);
    }
    clay
}

/// `mesh` turned by `turn` about its origin and then moved by `shift`: a
/// part modelled where it is easiest to, set where it belongs.
pub fn placed(
    mut mesh: MeshPrimitive,
    turn: UnitQuaternion<f32>,
    shift: Vector3<f32>,
) -> MeshPrimitive {
    for vertex in &mut mesh.vertices {
        vertex.pos = turn * vertex.pos + shift;
        vertex.normal = turn * vertex.normal;
    }
    mesh
}

/// A stiff hair, a quill or a thread.
#[derive(Clone, Copy, Debug)]
pub struct Bristle {
    /// Where it is rooted.
    pub root: Vector3<f32>,
    /// The way it grows from its root, as long as the bristle is, m.
    pub grows: Vector3<f32>,
    /// How far its tip is pulled off that line: a droop, a curl. m.
    pub bend: Vector3<f32>,
    /// Its radius at the root, m. It tapers to a point.
    pub thick: f32,
}

/// Rings and sides a bristle is drawn with.
const BRISTLE_RINGS: usize = 4;
const BRISTLE_SIDES: usize = 5;

/// How much of the light a bristle keeps at its root, in among the others.
const BRISTLE_ROOT_SHADE: f32 = 0.55;

/// `all` as one mesh in `colour` (linear) and `material`.
pub fn bristles(all: &[Bristle], colour: (f32, f32, f32), material: MaterialId) -> MeshPrimitive {
    let mut vertices = Vec::with_capacity(all.len() * (BRISTLE_RINGS * BRISTLE_SIDES + 1));
    let mut indices = Vec::new();
    for bristle in all {
        let base = vertices.len() as u32;
        let at = |t: f32| bristle.root + bristle.grows * t + bristle.bend * (t * t);
        let along_at = |t: f32| {
            (bristle.grows + bristle.bend * (2.0 * t))
                .try_normalize(1e-9)
                .unwrap_or_else(Vector3::y)
        };
        // A side to start the rings from, square to the bristle.
        let spare = if along_at(0.0).y.abs() < 0.9 {
            Vector3::y()
        } else {
            Vector3::x()
        };
        let vertex = |pos: Vector3<f32>, normal: Vector3<f32>, t: f32| Vertex {
            pos,
            color: Vector4::new(colour.0, colour.1, colour.2, 1.0),
            tex_coords: Vector2::zeros(),
            normal,
            ao: BRISTLE_ROOT_SHADE + (1.0 - BRISTLE_ROOT_SHADE) * t,
        };
        for ring in 0..BRISTLE_RINGS {
            let t = ring as f32 / BRISTLE_RINGS as f32;
            let along = along_at(t);
            let side = along.cross(&spare).normalize();
            let other = along.cross(&side);
            let radius = bristle.thick * (1.0 - 0.8 * t);
            for step in 0..BRISTLE_SIDES {
                let angle = step as f32 / BRISTLE_SIDES as f32 * TAU;
                let out = side * angle.cos() + other * angle.sin();
                vertices.push(vertex(at(t) + out * radius, out, t));
            }
        }
        vertices.push(vertex(at(1.0), along_at(1.0), 1.0));

        let sides = BRISTLE_SIDES as u32;
        let corner = |ring: u32, step: u32| base + ring * sides + step % sides;
        for ring in 0..BRISTLE_RINGS as u32 - 1 {
            for step in 0..sides {
                let (a, b) = (corner(ring, step), corner(ring, step + 1));
                let (c, d) = (corner(ring + 1, step), corner(ring + 1, step + 1));
                indices.extend([a, b, c, c, b, d]);
            }
        }
        let (last, tip) = (
            BRISTLE_RINGS as u32 - 1,
            base + BRISTLE_RINGS as u32 * sides,
        );
        for step in 0..sides {
            indices.extend([corner(last, step), corner(last, step + 1), tip]);
        }
    }
    MeshPrimitive {
        vertices,
        indices,
        material,
    }
}

/// +1 for the side whose inner edge lies toward −X, −1 for the other. The
/// rig hangs the left leg on +X.
fn inward(side: FootSide) -> f32 {
    match side {
        FootSide::Left => -1.0,
        FootSide::Right => 1.0,
    }
}

/// A turn of `angle` radians about the X axis.
fn about_x(angle: f32) -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle)
}

/// Dirt and dead skin in the creases between scales, and their crowns
/// rubbed pale (each multiplies the hide).
const SCALE_CREASE: Vector3<f32> = Vector3::new(0.42, 0.38, 0.36);
const SCALE_CROWN: Vector3<f32> = Vector3::new(1.12, 1.1, 1.04);

/// A toe of a bird's foot.
struct Toe {
    /// Which way it points, radians from straight ahead toward the outer
    /// side of the foot.
    splay: f32,
    /// How long and how thick it is, against the middle toe.
    long: f32,
    thick: f32,
    /// How many bones it has.
    bones: usize,
}

/// Three toes forward, the inner the shortest of them, and one behind, set
/// a little to the inside.
const TOES: [Toe; 4] = [
    Toe {
        splay: -0.66,
        long: 0.74,
        thick: 0.94,
        bones: 3,
    },
    Toe {
        splay: 0.05,
        long: 1.0,
        thick: 1.0,
        bones: 3,
    },
    Toe {
        splay: 0.6,
        long: 0.85,
        thick: 0.9,
        bones: 3,
    },
    Toe {
        splay: -(PI - 0.3),
        long: 0.42,
        thick: 0.86,
        bones: 2,
    },
];

/// A bird's leg: how it is built and coloured.
#[derive(Clone, Copy, Debug)]
pub struct BirdLeg {
    /// Radius of the drumstick's meat at the hip, m.
    pub girth: f32,
    /// Radius of the shank, the bare bone below the meat, m. The hock, the
    /// scales and the toes are sized from it.
    pub shank: f32,
    /// Length of the middle toe, m.
    pub toe: f32,
    /// The plucked skin over the meat (linear).
    pub skin: (f32, f32, f32),
    /// The scaled hide of the shank and the toes (linear).
    pub hide: (f32, f32, f32),
    /// The claws (linear).
    pub claw: (f32, f32, f32),
    /// Side of the grid the shank and the foot are skinned from, m. A
    /// quarter of the shank's radius still holds a scale.
    pub grain: f32,
    /// Side of the grid the drumstick is skinned from, m: coarser, and
    /// coarser still for one that is out of sight under a robe.
    pub drumstick_grain: f32,
}

impl BirdLeg {
    /// How far above the ground the shank ends and the toes spread from:
    /// the rig's [`StiltRigConfig::ankle_height`].
    pub fn ankle_height(&self) -> f32 {
        1.2 * self.shank
    }

    /// The scales of the hide.
    fn scales(&self) -> Relief {
        Relief::Scales {
            size: 0.72 * self.shank,
            depth: 0.1 * self.shank,
        }
    }

    /// `smooth` raised into scales, skinned and painted as hide wherever
    /// `bare` says the leg is bare of plucked skin: 1 for scaled hide, 0
    /// for skin, whose scales are only gooseflesh.
    fn hide_of(
        &self,
        smooth: &Clay,
        grain: f32,
        bare: impl Fn(Vector3<f32>) -> f32,
    ) -> MeshPrimitive {
        let mut mesh = plucked(&smooth.clone().relief(self.scales()), grain);
        let plucked: Vec<_> = mesh.vertices.iter().map(|vertex| vertex.color).collect();
        shade_relief(
            &mut mesh,
            smooth,
            (-0.1 * self.shank, 0.0),
            SCALE_CREASE,
            SCALE_CROWN,
        );
        for (vertex, plucked) in mesh.vertices.iter_mut().zip(plucked) {
            let bare = bare(vertex.pos).clamp(0.0, 1.0);
            vertex.color = plucked.lerp(&vertex.color, bare);
        }
        mesh
    }

    /// The upper leg, `length` long: the hip at its origin and the hock
    /// `length` below it, its back toward −Z. Meat under plucked skin at
    /// the top, then bare shank with the tendon standing off behind it, down
    /// to the knob of the hock.
    pub fn drumstick(&self, length: f32) -> MeshPrimitive {
        let (l, g, s) = (length, self.girth, self.shank);
        let smooth = Clay::new(0.5 * g)
            .add(
                Lump::limb((0.0, 0.0, 0.0), (0.0, -0.4 * l, 0.0), (0.8 * g, 0.4 * g)),
                self.skin,
            )
            // The muscle that draws the hock up, behind the bone.
            .add(
                Lump::mass((0.66 * g, 0.2 * l, 0.72 * g), (0.0, -0.2 * l, -0.36 * g)),
                self.skin,
            )
            .add(
                Lump::limb((0.0, -0.3 * l, 0.0), (0.0, -l, 0.0), (1.3 * s, 1.05 * s)),
                self.hide,
            )
            .smoothing(0.5 * s)
            .add(
                Lump::limb(
                    (0.0, -0.36 * l, -1.5 * s),
                    (0.0, -l + s, -1.25 * s),
                    (0.6 * s, 0.5 * s),
                ),
                self.hide,
            )
            // The hock: the two heads of the bone, and the heel behind.
            .smoothing(0.6 * s)
            .add(Lump::ball(1.5 * s, (0.0, -l, 0.0)), self.hide)
            .add(
                Lump::ball(0.95 * s, (1.0 * s, -l + 0.25 * s, -0.4 * s)),
                self.hide,
            )
            .add(
                Lump::ball(0.95 * s, (-1.0 * s, -l - 0.1 * s, -0.4 * s)),
                self.hide,
            )
            .add(
                Lump::ball(0.85 * s, (0.0, -l - 0.2 * s, -1.25 * s)),
                self.hide,
            );
        // Skin over the meat, giving way to hide where the shank begins.
        let mut mesh = self.hide_of(&smooth, self.drumstick_grain, |at| (-at.y / l - 0.26) / 0.2);
        // A joint that has carried it too far: swollen and red.
        Blush {
            at: Vector3::new(0.0, -l, -0.5 * s),
            radius: 4.0 * s,
            tint: Vector3::new(1.0, 0.7, 0.66),
        }
        .paint(&mut mesh);
        mesh
    }

    /// The lower leg, `length` long: the hock at its origin and the ground
    /// `length` below it, its back toward −Z. One long bone with a row of
    /// plates down its front and a tendon down its back, ending where the
    /// toes spread from.
    pub fn tarsus(&self, length: f32) -> MeshPrimitive {
        let (l, s) = (length, self.shank);
        let ankle = -l + self.ankle_height();
        let mut smooth = Clay::new(0.6 * s)
            .add(Lump::ball(1.45 * s, (0.0, 0.0, 0.0)), self.hide)
            .add(Lump::ball(0.9 * s, (0.0, 0.5 * s, -1.2 * s)), self.hide)
            .add(
                Lump::limb((0.0, 0.0, 0.0), (0.0, ankle, 0.0), (1.0 * s, 0.86 * s)),
                self.hide,
            )
            .add(Lump::ball(1.2 * s, (0.0, ankle, 0.0)), self.hide)
            .smoothing(0.3 * s)
            .add(
                Lump::limb(
                    (0.0, -2.0 * s, -0.95 * s),
                    (0.0, ankle + 2.0 * s, -0.82 * s),
                    (0.44 * s, 0.4 * s),
                ),
                self.hide,
            )
            .smoothing(0.14 * s);
        // The plates down its front, each lapping the one below.
        let rise = 1.05 * s;
        let plates = ((l - 5.5 * s) / rise) as usize;
        for plate in 0..plates {
            let y = -2.6 * s - rise * plate as f32;
            smooth = smooth.add(
                Lump::Ball(Ellipsoid {
                    radii: Vector3::new(0.86 * s, 0.64 * rise, 0.3 * s),
                    centre: Vector3::new(0.1 * s * scatter(plate as u32, 151), y, 0.84 * s),
                    turn: about_x(-0.3),
                }),
                self.hide,
            );
        }
        self.hide_of(&smooth, self.grain, |_| 1.0)
    }

    /// The foot, standing on y = 0 with the leg coming down onto its
    /// origin: three toes spread forward (+Z) and one behind, each jointed
    /// and padded and ending in a claw. The skin, then the claws.
    pub fn foot(&self, side: FootSide) -> Vec<MeshPrimitive> {
        let s = self.shank;
        let outer = -inward(side);
        let ball = Vector3::new(0.0, self.ankle_height(), 0.0);
        let mut smooth = Clay::new(0.5 * s)
            .add(Lump::ball(1.25 * s, (ball.x, ball.y, ball.z)), self.hide)
            // The pad it stands on, under where the toes meet.
            .add(
                Lump::mass((1.45 * s, 0.7 * s, 1.5 * s), (0.0, 0.72 * s, 0.25 * s)),
                self.hide,
            );
        let mut claws = Clay::new(0.12 * s);
        for toe in &TOES {
            let along = Vector3::new(outer * toe.splay.sin(), 0.0, toe.splay.cos());
            let reach = self.toe * toe.long;
            let thick = 0.74 * s * toe.thick;
            // A pad under each joint and under the tip, each smaller than
            // the last, and the bone between them arched clear of the
            // ground.
            let pad = |joint: usize| thick * (1.0 - 0.1 * joint as f32);
            let at = |joint: usize| {
                let share = joint as f32 / toe.bones as f32;
                let out = 0.9 * s + reach * share.powf(0.85);
                along * out + Vector3::y() * 0.96 * pad(joint)
            };
            let mut from = ball;
            for joint in 1..=toe.bones {
                let to = at(joint);
                let arch = Vector3::y() * 0.3 * pad(joint);
                let (a, b) = (from + arch, to + arch);
                smooth = smooth
                    .smoothing(0.45 * thick)
                    .add(
                        Lump::limb(
                            (a.x, a.y, a.z),
                            (b.x, b.y, b.z),
                            (0.84 * pad(joint - 1), 0.8 * pad(joint)),
                        ),
                        self.hide,
                    )
                    .smoothing(0.3 * thick)
                    .add(Lump::ball(pad(joint), (to.x, to.y, to.z)), self.hide);
                from = to;
            }
            // The claw: out of the top of the last pad, and hooked down to
            // the ground.
            let tip = pad(toe.bones);
            let long = 2.9 * s * toe.long.max(0.7);
            let root = from + along * 0.4 * tip + Vector3::y() * 0.3 * tip;
            let crook = root + along * 0.55 * long + Vector3::y() * 0.12 * long;
            let point = root + along * long - Vector3::y() * (root.y - 0.1 * tip);
            claws = claws
                .add(
                    Lump::limb(
                        (root.x, root.y, root.z),
                        (crook.x, crook.y, crook.z),
                        (0.66 * tip, 0.46 * tip),
                    ),
                    self.claw,
                )
                .add(
                    Lump::limb(
                        (crook.x, crook.y, crook.z),
                        (point.x, point.y, point.z),
                        (0.46 * tip, 0.07 * tip),
                    ),
                    self.claw,
                );
        }
        let mut skin = self.hide_of(&smooth, self.grain, |_| 1.0);
        // It has walked here.
        Grime {
            below: 0.6 * s,
            fade: 2.6 * s,
            tint: Vector3::new(0.5, 0.42, 0.34),
        }
        .paint(&mut skin);
        let mut claws = claws.skin(0.6 * self.grain, Paint::Horn.id());
        Mottle {
            frequency: 60.0,
            value: 0.3,
            warmth: 0.12,
            seed: 152,
        }
        .paint(&mut claws);
        vec![skin, claws]
    }

    /// Both legs for a walker of proportions `rig`: drumsticks, shanks and
    /// feet, each worn on its own bone.
    pub fn legs(&self, rig: &StiltRigConfig) -> Vec<(Worn, Vec<MeshPrimitive>)> {
        let drumstick = self.drumstick(rig.upper_leg_length);
        let tarsus = self.tarsus(rig.lower_leg_length);
        let mut legs = Vec::new();
        for side in [FootSide::Left, FootSide::Right] {
            legs.push((Worn::on(Joint::Thigh(side)), vec![drumstick.clone()]));
            legs.push((Worn::on(Joint::Shin(side)), vec![tarsus.clone()]));
            legs.push((Worn::on(Joint::Foot(side)), self.foot(side)));
        }
        legs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEG: BirdLeg = BirdLeg {
        girth: 0.05,
        shank: 0.016,
        toe: 0.16,
        skin: PLUCKED,
        hide: (0.5, 0.42, 0.26),
        claw: (0.08, 0.07, 0.06),
        grain: 0.0035,
        drumstick_grain: 0.0045,
    };

    fn span(mesh: &MeshPrimitive, axis: usize) -> (f32, f32) {
        mesh.vertices
            .iter()
            .fold((f32::MAX, f32::MIN), |(low, high), v| {
                (low.min(v.pos[axis]), high.max(v.pos[axis]))
            })
    }

    #[test]
    fn a_leg_reaches_from_hip_to_ground_and_stands_on_its_toes() {
        let (low, _) = span(&LEG.drumstick(0.5), 1);
        assert!((-0.56..-0.5).contains(&low), "a drumstick ends at {low}");
        let (low, high) = span(&LEG.tarsus(0.5), 1);
        assert!((-0.51..-0.46).contains(&low), "a shank ends at {low}");
        assert!(high > 0.01, "no hock on it");

        let foot = LEG.foot(FootSide::Left);
        let (sole, _) = span(&foot[0], 1);
        assert!(sole.abs() < 0.006, "its sole is at {sole}");
        // The middle toe and its claw reach furthest forward, and there is
        // a toe behind.
        let (back, _) = span(&foot[0], 2);
        let (_, front) = span(&foot[1], 2);
        assert!((0.19..0.25).contains(&front), "its claws reach {front}");
        assert!(back < -0.06, "no hind toe: {back}");
        // The hind toe is set to the inside: −X for a left foot.
        let hindmost = foot[0]
            .vertices
            .iter()
            .min_by(|a, b| a.pos.z.total_cmp(&b.pos.z))
            .unwrap();
        assert!(
            hindmost.pos.x < -0.01,
            "its hind toe is at x {}",
            hindmost.pos.x
        );
    }

    #[test]
    fn bristles_taper_from_their_roots_and_face_outward() {
        let mesh = bristles(
            &[Bristle {
                root: Vector3::new(1.0, 0.0, 0.0),
                grows: Vector3::new(0.0, 0.1, 0.0),
                bend: Vector3::new(0.02, 0.0, 0.0),
                thick: 0.004,
            }],
            (0.1, 0.1, 0.1),
            Paint::Horn.id(),
        );
        let tip = mesh.vertices.last().unwrap().pos;
        assert!((tip - Vector3::new(1.02, 0.1, 0.0)).norm() < 1e-5);
        for triangle in mesh.indices.chunks_exact(3) {
            let [a, b, c] = [0, 1, 2].map(|k| mesh.vertices[triangle[k] as usize]);
            let face = (b.pos - a.pos).cross(&(c.pos - a.pos));
            assert!(
                face.dot(&(a.normal + b.normal + c.normal)) > 0.0,
                "a face is inside out"
            );
        }
    }

    #[test]
    fn relief_is_shaded_dark_in_its_creases() {
        let smooth = Clay::new(0.0).add(Lump::ball(0.1, (0.0, 0.0, 0.0)), (1.0, 1.0, 1.0));
        let scaled = smooth.clone().relief(Relief::Scales {
            size: 0.03,
            depth: 0.006,
        });
        let mut mesh = scaled.skin(0.004, Paint::Skin.id());
        shade_relief(
            &mut mesh,
            &smooth,
            (-0.006, 0.0),
            Vector3::repeat(0.2),
            Vector3::repeat(1.0),
        );
        let (sunk, proud): (Vec<&Vertex>, Vec<&Vertex>) =
            mesh.vertices.iter().partition(|v| v.pos.norm() < 0.097);
        let mean = |set: &[&Vertex]| set.iter().map(|v| v.color.x).sum::<f32>() / set.len() as f32;
        assert!(!sunk.is_empty() && !proud.is_empty());
        assert!(mean(&sunk) < 0.75 * mean(&proud));
    }
}
