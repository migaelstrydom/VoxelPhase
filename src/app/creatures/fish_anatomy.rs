//! The parts of a fish the fish-walker and the flying fish share: fins and
//! wings as a web stretched on rays, a glassy eye, blunt teeth, and the
//! paint of a hide that is dark above and pale below.
//!
//! ```text
//!   Fin        rays ──web between, scalloped, torn, holed──▶ a thin two-sided mesh
//!   glass_eye  anatomy::eyeball, wide of iris and filmed over
//!   crown      one blunt tooth, as a lump for clay
//!   Hide       a fish's skin painted on a skinned form: Countershade (back
//!              and belly, by height), patches, pores, freckles, sores
//!   flay       raw flesh painted where a form is cut open
//!   Crevices   paint settled in a skin's creases and rubbed off its ridges
//!   Runoff     what has run down a limb from above
//!   moved      a mesh turned and shifted: a part modelled in one frame, worn in another
//!   surface_along  where a clay form's surface is, out from a point inside it
//! ```
//!
//! A fin is not clay: a web is thinner than any grid clay could be skinned
//! from, and its edge is where a fish shows its age, so it is meshed straight
//! from its rays.

use std::f32::consts::PI;

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};

use super::anatomy::{self, EyeLook};
use crate::app::sculpt::{Clay, Ellipsoid, Lump, Mottle, Spots, VertexPaint};
use crate::model::MeshPrimitive;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;
use crate::utils::noise::fbm_perlin_3d;

/// How wide a ray's ridge is, as a share of the gap to the next ray.
const RAY_WIDTH: f32 = 0.24;
/// The thinnest a web is skinned, m: thinner and its two faces fight.
const THINNEST: f32 = 0.0004;
/// How much of the light a web keeps where it leaves the body.
const ROOT_SHADE: f32 = 0.5;
/// How wide the bright ring round an eye's pupil is, radians on the eye.
const RING_WIDTH: f32 = 0.035;

/// Raw flesh (linear): gills, the inside of a mouth, the lip of a wound;
/// and what it darkens to further in.
pub const RAW: (f32, f32, f32) = (0.2, 0.02, 0.02);
pub const GULLET: (f32, f32, f32) = (0.025, 0.004, 0.005);

/// A place or a colour given as three numbers, as a vector.
pub fn point(p: (f32, f32, f32)) -> Vector3<f32> {
    Vector3::new(p.0, p.1, p.2)
}

/// `p` on a creature's right, mirrored to its left for `side` −1.
pub fn sided(side: f32, p: (f32, f32, f32)) -> (f32, f32, f32) {
    (side * p.0, p.1, p.2)
}

/// Paint `mesh` raw wherever `raw` gives, for a vertex's place and normal,
/// how far in it is: 0 at the lip of a wound, 1 in the dark of it.
pub fn flay(mesh: &mut MeshPrimitive, raw: impl Fn(Vector3<f32>, Vector3<f32>) -> Option<f32>) {
    for vertex in &mut mesh.vertices {
        if let Some(depth) = raw(vertex.pos, vertex.normal) {
            let colour = point(RAW).lerp(&point(GULLET), depth.clamp(0.0, 1.0));
            vertex.color.x = colour.x;
            vertex.color.y = colour.y;
            vertex.color.z = colour.z;
        }
    }
}

/// `mesh` turned by `turn` and then moved by `shift`.
pub fn moved(
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

/// One ray of a fin: a spine the web is stretched on.
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    /// Where it leaves the body.
    pub root: Vector3<f32>,
    /// Where it ends.
    pub tip: Vector3<f32>,
    /// How far its middle bows off the straight line between them, m.
    pub bow: Vector3<f32>,
}

impl Ray {
    /// A straight ray from `root` to `tip`.
    pub fn new(root: Vector3<f32>, tip: Vector3<f32>) -> Self {
        Self {
            root,
            tip,
            bow: Vector3::zeros(),
        }
    }

    /// Bowed `bow` off straight at its middle.
    pub fn bowed(mut self, bow: Vector3<f32>) -> Self {
        self.bow = bow;
        self
    }

    /// The point `v` of the way along it.
    pub fn at(&self, v: f32) -> Vector3<f32> {
        self.root + (self.tip - self.root) * v + self.bow * (4.0 * v * (1.0 - v))
    }
}

/// A tear in a fin, from its edge inward.
#[derive(Clone, Copy, Debug)]
pub struct Tear {
    /// Where across the fin it is, in rays: 2.5 is half way between the
    /// third ray and the fourth, and a whole number is a ray broken short.
    pub at: f32,
    /// How wide it is at the edge, in rays.
    pub width: f32,
    /// How much of the fin's length it takes there, 0 to 1.
    pub depth: f32,
}

/// A hole worn through a fin's web.
#[derive(Clone, Copy, Debug)]
pub struct Hole {
    /// Where across the fin it is, in rays, and how far along them, 0 to 1.
    pub at: (f32, f32),
    /// How far it reaches across, in rays, and along, as a share.
    pub reach: (f32, f32),
}

/// What a fin is painted in (linear).
#[derive(Clone, Copy, Debug)]
pub struct FinPaint {
    /// Its rays.
    pub ray: (f32, f32, f32),
    /// The web where it leaves the body.
    pub web: (f32, f32, f32),
    /// The web at its edge.
    pub edge: (f32, f32, f32),
}

/// A fin or a wing: a web of skin stretched on rays.
///
/// Its front is the face that looks along `across × along`: across in the
/// rays' order, along from their roots to their tips.
#[derive(Clone, Debug)]
pub struct Fin {
    /// Its rays, in order across it. Two at least.
    pub rays: Vec<Ray>,
    /// Half the thickness of a ray where it leaves the body, m.
    pub ray_girth: f32,
    /// Half the thickness of the web there, m. Both thin toward the edge.
    pub web: f32,
    /// How far the web's edge hangs back between two rays' tips, as a
    /// share of their length.
    pub scallop: f32,
    /// How far the web bellies toward the fin's front between two rays,
    /// m; negative for the back.
    pub belly: f32,
    /// How ragged the edge is, as a share of the fin's length.
    pub fray: f32,
    pub tears: Vec<Tear>,
    pub holes: Vec<Hole>,
    pub paint: FinPaint,
    /// Seed of its raggedness.
    pub seed: u32,
}

impl Fin {
    /// A whole fin on `rays`, as thin as a carp's.
    pub fn on(rays: Vec<Ray>, paint: FinPaint, seed: u32) -> Self {
        Self {
            rays,
            ray_girth: 0.003,
            web: 0.0011,
            scallop: 0.15,
            belly: 0.0,
            fray: 0.18,
            tears: Vec::new(),
            holes: Vec::new(),
            paint,
            seed,
        }
    }

    /// With rays `ray_girth` and a web `web` thick at the root, each by
    /// half.
    pub fn thick(mut self, ray_girth: f32, web: f32) -> Self {
        self.ray_girth = ray_girth;
        self.web = web;
        self
    }

    /// With its edge hung back `scallop` between rays and ragged by `fray`.
    pub fn edged(mut self, scallop: f32, fray: f32) -> Self {
        self.scallop = scallop;
        self.fray = fray;
        self
    }

    /// With its web bellied `belly` metres toward its front.
    pub fn bellied(mut self, belly: f32) -> Self {
        self.belly = belly;
        self
    }

    /// Torn: each tear where it is across the fin, how wide and how deep.
    pub fn torn(mut self, tears: &[(f32, f32, f32)]) -> Self {
        self.tears = tears
            .iter()
            .map(|&(at, width, depth)| Tear { at, width, depth })
            .collect();
        self
    }

    /// Worn through: each hole where it is and how far it reaches.
    pub fn holed_at(mut self, holes: &[((f32, f32), (f32, f32))]) -> Self {
        self.holes = holes
            .iter()
            .map(|&(at, reach)| Hole { at, reach })
            .collect();
        self
    }

    /// How much of the rays' length the web reaches `u` rays across.
    fn reach(&self, u: f32) -> f32 {
        let between = (u.fract() * PI).sin().max(0.0);
        let mut reach = 1.0 - self.scallop * between.powf(0.8);
        let ragged = fbm_perlin_3d(u * 3.7, 0.37, 0.0, 3, 0.6, self.seed);
        reach *= 1.0 - self.fray * (0.5 + 0.5 * ragged) * (0.35 + 0.65 * between);
        for tear in &self.tears {
            let off = ((u - tear.at) / (0.5 * tear.width)).abs().min(1.0);
            reach *= 1.0 - tear.depth * (1.0 - off).powf(1.4);
        }
        reach.max(0.02)
    }

    /// How much of a ray's ridge stands at `u` rays across, 0 to 1.
    fn ridge(u: f32) -> f32 {
        let bump = |off: f32| (1.0 - (off / RAY_WIDTH).powi(2)).max(0.0).powi(2);
        let between = u.fract();
        bump(between) + bump(1.0 - between)
    }

    /// Whether the web is worn through at (`u`, `v`).
    fn holed(&self, u: f32, v: f32) -> bool {
        self.holes.iter().any(|hole| {
            // A ragged round, not a punched one.
            let ragged = 1.0 + 0.45 * fbm_perlin_3d(u * 5.0, v * 9.0, 0.5, 2, 0.5, self.seed);
            let across = (u - hole.at.0) / (hole.reach.0 * ragged);
            let along = (v - hole.at.1) / (hole.reach.1 * ragged);
            across * across + along * along < 1.0
        })
    }

    /// The fin as a mesh in `material`: `columns` quads across each gap
    /// between two rays and `rows` along them, on each face.
    pub fn mesh(&self, columns: usize, rows: usize, material: MaterialId) -> MeshPrimitive {
        let gaps = self.rays.len() - 1;
        let (across, along) = (gaps * columns, rows);
        let wide = across + 1;
        let index = |iu: usize, iv: usize| iu + iv * wide;

        // The web's middle sheet, how thick it is there, and its paint.
        let mut middle = Vec::with_capacity(wide * (along + 1));
        let mut half_thick = Vec::with_capacity(middle.capacity());
        let mut paint = Vec::with_capacity(middle.capacity());
        for iv in 0..=along {
            for iu in 0..=across {
                let u = iu as f32 / columns as f32;
                let gap = (u.floor() as usize).min(gaps - 1);
                let between = u - gap as f32;
                let v = iv as f32 / along as f32 * self.reach(u);
                let (near, far) = (self.rays[gap], self.rays[gap + 1]);
                let (a, b) = (near.at(v), far.at(v));
                let run = (near.tip - near.root + far.tip - far.root) * 0.5;
                let out = (b - a).cross(&run).try_normalize(1e-9);
                let bellied =
                    out.unwrap_or_else(Vector3::zeros) * self.belly * (between * PI).sin() * v;
                middle.push(a + (b - a) * between + bellied);

                let ridge = Self::ridge(u).min(1.0);
                let thick = self.web * (1.0 - 0.75 * v) + self.ray_girth * ridge * (1.0 - 0.7 * v);
                half_thick.push(thick.max(THINNEST));

                let colour = |c: (f32, f32, f32)| Vector3::new(c.0, c.1, c.2);
                let web = colour(self.paint.web).lerp(&colour(self.paint.edge), v * v);
                let shade = ROOT_SHADE + (1.0 - ROOT_SHADE) * (v / 0.3).min(1.0);
                // The crease where web meets ray holds a little shadow.
                let crease = 1.0 - 0.22 * (4.0 * ridge * (1.0 - ridge)).max(0.0);
                paint.push((web.lerp(&colour(self.paint.ray), ridge), shade * crease));
            }
        }

        // A sheet's normals, from its neighbours on the grid.
        let normals = |sheet: &[Vector3<f32>]| -> Vec<Vector3<f32>> {
            let mut normals = vec![Vector3::zeros(); sheet.len()];
            for iv in 0..=along {
                for iu in 0..=across {
                    let du = sheet[index((iu + 1).min(across), iv)]
                        - sheet[index(iu.saturating_sub(1), iv)];
                    let dv = sheet[index(iu, (iv + 1).min(along))]
                        - sheet[index(iu, iv.saturating_sub(1))];
                    normals[index(iu, iv)] = du.cross(&dv);
                }
            }
            // Where the rays leave one point the sheet has no width, and
            // takes the lie of the row beyond.
            for iv in 0..=along {
                for iu in 0..=across {
                    if normals[index(iu, iv)].norm() < 1e-10 && iv < along {
                        normals[index(iu, iv)] = normals[index(iu, iv + 1)];
                    }
                }
            }
            normals
                .into_iter()
                .map(|n| n.try_normalize(1e-12).unwrap_or_else(Vector3::y))
                .collect()
        };
        let lie = normals(&middle);
        let face = |sign: f32| -> Vec<Vector3<f32>> {
            (0..middle.len())
                .map(|at| middle[at] + lie[at] * (sign * half_thick[at]))
                .collect()
        };
        let (front, back) = (face(1.0), face(-1.0));
        let (front_normals, back_normals) = (normals(&front), normals(&back));

        let count = middle.len() as u32;
        let mut vertices = Vec::with_capacity(2 * middle.len());
        for (sheet, sheet_normals, sign) in
            [(&front, &front_normals, 1.0), (&back, &back_normals, -1.0)]
        {
            for at in 0..middle.len() {
                let (colour, shade) = paint[at];
                vertices.push(Vertex {
                    pos: sheet[at],
                    color: Vector4::new(colour.x, colour.y, colour.z, 1.0),
                    tex_coords: Vector2::zeros(),
                    normal: sheet_normals[at] * sign,
                    ao: shade,
                });
            }
        }

        let mut indices = Vec::new();
        for iv in 0..along {
            for iu in 0..across {
                let u = (iu as f32 + 0.5) / columns as f32;
                let v = (iv as f32 + 0.5) / along as f32;
                if self.holed(u, v) {
                    continue;
                }
                let [a, b, c, d] = [
                    index(iu, iv),
                    index(iu + 1, iv),
                    index(iu, iv + 1),
                    index(iu + 1, iv + 1),
                ]
                .map(|at| at as u32);
                indices.extend([a, b, c, b, d, c]);
                indices.extend([a, c, b, b, c, d].map(|at| at + count));
            }
        }
        // The two faces joined round the fin's edge.
        let mut rim: Vec<usize> = (0..across).map(|iu| index(iu, 0)).collect();
        rim.extend((0..along).map(|iv| index(across, iv)));
        rim.extend((0..across).map(|iu| index(across - iu, along)));
        rim.extend((0..along).map(|iv| index(0, along - iv)));
        for step in 0..rim.len() {
            let (p, q) = (rim[step] as u32, rim[(step + 1) % rim.len()] as u32);
            indices.extend([p, p + count, q + count, p, q + count, q]);
        }

        MeshPrimitive {
            vertices,
            indices,
            material,
        }
    }
}

/// How a fish's eye is coloured.
#[derive(Clone, Copy, Debug)]
pub struct GlassLook {
    /// The iris (linear): a fish's is metal, gold or tin.
    pub iris: (f32, f32, f32),
    /// How wide the iris is, as the angle from the eye's axis to its edge,
    /// radians, and how much of that the pupil takes.
    pub iris_angle: f32,
    pub pupil_share: f32,
    /// How bloodshot what shows round the iris is, 0 to 1.
    pub bloodshot: f32,
    /// How filmed over it is, 0 to 1: the eye of a fish that has been out
    /// of the water too long, or one gone blind.
    pub film: f32,
}

/// A fish's eye of `radius` about its own middle, looking along +Z: round,
/// flat and unlidded, most of it iris and most of that pupil, with a bright
/// ring where the two meet and a film over the whole.
pub fn glass_eye(radius: f32, look: GlassLook) -> MeshPrimitive {
    let mut eye = anatomy::eyeball(
        radius,
        EyeLook {
            iris: look.iris,
            iris_angle: look.iris_angle,
            pupil_share: look.pupil_share,
            bloodshot: look.bloodshot,
        },
    );
    let film = Vector3::new(0.5, 0.56, 0.58);
    let pupil = look.iris_angle * look.pupil_share;
    for vertex in &mut eye.vertices {
        let off = (vertex.pos.z / vertex.pos.norm()).clamp(-1.0, 1.0).acos();
        let mut colour = vertex.color.xyz();
        // The bright ring round the pupil.
        let ring = (1.0 - ((off - pupil - RING_WIDTH) / RING_WIDTH).abs()).max(0.0);
        colour = colour.lerp(&Vector3::new(0.7, 0.5, 0.18), 0.45 * ring * ring);
        // The film lies thickest over the pupil, and unevenly.
        let at = vertex.pos / radius;
        let patchy = 0.75 + 0.5 * fbm_perlin_3d(at.x * 2.4, at.y * 2.4, at.z * 2.4, 2, 0.5, 97);
        let over = 1.0 - (off / (look.iris_angle * 1.3)).min(1.0).powi(2);
        colour = colour.lerp(
            &film,
            (look.film * patchy * (0.35 + 0.65 * over)).clamp(0.0, 1.0),
        );
        vertex.color = Vector4::new(colour.x, colour.y, colour.z, 1.0);
    }
    eye
}

/// One blunt tooth, as a man's is: a crown `size` (across, tall, thick)
/// whose root is at `root`, standing along `up` with its face to `out`.
pub fn crown(
    root: Vector3<f32>,
    up: Vector3<f32>,
    out: Vector3<f32>,
    size: (f32, f32, f32),
) -> Lump {
    let up = up.normalize();
    let out = (out - up * out.dot(&up)).normalize();
    let across = up.cross(&out);
    Lump::Ball(Ellipsoid {
        radii: Vector3::new(0.5 * size.0, 0.5 * size.1, 0.5 * size.2),
        centre: root + up * (0.42 * size.1),
        turn: UnitQuaternion::from_basis_unchecked(&[across, up, out]),
    })
}

/// A fish's hide painted by height: dark along the back, pale under the
/// belly, whatever colour the flank between them is.
#[derive(Clone, Copy, Debug)]
pub struct Countershade {
    /// Heights of the belly's lowest and the back's highest, m.
    pub span: (f32, f32),
    /// What the paint is multiplied by along the back, and under the
    /// belly (linear).
    pub back: Vector3<f32>,
    pub belly: Vector3<f32>,
    /// How much of the way up the flank the belly's paint has faded out
    /// by, and the back's has begun at, 0 to 1.
    pub flank: (f32, f32),
}

impl Countershade {
    /// Paint `mesh`'s vertex colours.
    pub fn paint(&self, mesh: &mut MeshPrimitive) {
        for vertex in &mut mesh.vertices {
            // A surface that faces up is back wherever it is, and one that
            // faces down is belly: a fin's root, the swell of a hip.
            let height = (vertex.pos.y - self.span.0) / (self.span.1 - self.span.0);
            let up = (height + 0.12 * vertex.normal.y).clamp(0.0, 1.0);
            // The line between flank and back wanders.
            let at = vertex.pos * 9.0;
            let wander = 0.08 * fbm_perlin_3d(at.x, at.y, at.z, 2, 0.5, 71);
            let ease = |t: f32| {
                let t = t.clamp(0.0, 1.0);
                t * t * (3.0 - 2.0 * t)
            };
            let belly = 1.0 - ease((up + wander) / self.flank.0);
            let back = ease((up + wander - self.flank.1) / (1.0 - self.flank.1));
            let wash = Vector3::repeat(1.0)
                .lerp(&self.belly, belly)
                .lerp(&self.back, back);
            vertex.color.x *= wash.x;
            vertex.color.y *= wash.y;
            vertex.color.z *= wash.z;
        }
    }
}

/// A fish's hide, painted on a skinned form over its own colour: dark
/// along the back and pale under the belly, patched, pored and freckled,
/// and as sick as it is: the pale bloom of fungus, and sores where the
/// blood has come to the skin.
#[derive(Clone, Copy, Debug)]
pub struct Hide {
    /// Heights of the belly's lowest and the back's highest, m.
    pub span: (f32, f32),
    /// What the paint is multiplied by along the back, and under the
    /// belly (linear).
    pub back: (f32, f32, f32),
    pub belly: (f32, f32, f32),
    /// How much of its fungus and its sores it has, 0 to 1.
    pub sick: f32,
    /// Seed of its patches.
    pub seed: u32,
}

impl Hide {
    /// Paint `mesh`'s vertex colours.
    pub fn paint(&self, mesh: &mut MeshPrimitive) {
        Countershade {
            span: self.span,
            back: point(self.back),
            belly: point(self.belly),
            flank: (0.4, 0.52),
        }
        .paint(mesh);
        Mottle {
            frequency: 6.0,
            value: 0.3,
            warmth: 0.22,
            seed: self.seed,
        }
        .paint(mesh);
        // Pores, freckles and blotches; then fungus, and sores.
        let marks = [
            (130.0, 0.5, (0.2, 0.45), (0.62, 0.6, 0.55)),
            (38.0, 0.3, (0.14, 0.4), (0.45, 0.42, 0.38)),
            (13.0, 0.3, (0.25, 0.5), (0.6, 0.66, 0.56)),
            (8.0, 0.2 * self.sick, (0.2, 0.46), (1.5, 1.6, 1.7)),
            (5.0, 0.16 * self.sick, (0.16, 0.4), (1.15, 0.42, 0.38)),
        ];
        for (layer, (frequency, presence, radius, tint)) in marks.into_iter().enumerate() {
            Spots {
                frequency,
                presence,
                radius,
                tint: point(tint),
                salt: self.seed + 1 + layer as u32,
            }
            .paint(mesh);
        }
    }
}

/// Paint that has settled in a skin's creases and rubbed off its ridges:
/// what makes scales read as scales and folds as folds, where the light
/// alone would lose them.
#[derive(Clone, Copy, Debug)]
pub struct Crevices {
    /// Share of the paint's light the deepest crease loses.
    pub depth: f32,
    /// Share by which the sharpest ridge is lightened.
    pub lift: f32,
}

impl Crevices {
    /// How hollow a vertex must lie among its neighbours, as a share of
    /// its distance from them, to count as the deepest crease.
    const DEEPEST: f32 = 0.22;

    /// Paint `mesh`'s vertex colours.
    pub fn paint(&self, mesh: &mut MeshPrimitive) {
        // Where each vertex's neighbours are, on average, and how far.
        let count = mesh.vertices.len();
        let mut around = vec![(Vector3::zeros(), 0.0f32, 0.0f32); count];
        for triangle in mesh.indices.chunks_exact(3) {
            for corner in 0..3 {
                let (a, b) = (
                    triangle[corner] as usize,
                    triangle[(corner + 1) % 3] as usize,
                );
                let (pa, pb) = (mesh.vertices[a].pos, mesh.vertices[b].pos);
                let apart = (pb - pa).norm();
                for (at, other) in [(a, pb), (b, pa)] {
                    around[at].0 += other;
                    around[at].1 += apart;
                    around[at].2 += 1.0;
                }
            }
        }
        for (vertex, (sum, apart, neighbours)) in mesh.vertices.iter_mut().zip(around) {
            if neighbours == 0.0 || apart <= 0.0 {
                continue;
            }
            // In a crease its neighbours stand above it; on a ridge, below.
            let hollow = (sum / neighbours - vertex.pos).dot(&vertex.normal) / (apart / neighbours);
            let share = (hollow / Self::DEEPEST).clamp(-1.0, 1.0);
            let wash = if share > 0.0 {
                1.0 - self.depth * share
            } else {
                1.0 - self.lift * share
            };
            vertex.color.x *= wash;
            vertex.color.y *= wash;
            vertex.color.z *= wash;
        }
    }
}

/// What has run down a limb from above and dried on it in streaks: the
/// stain whole above a height, and trailing out below it.
#[derive(Clone, Copy, Debug)]
pub struct Runoff {
    /// Height above which the stain is whole, and how far below that its
    /// streaks reach, m.
    pub above: f32,
    pub trail: f32,
    /// The colour it multiplies the paint by (linear).
    pub tint: Vector3<f32>,
    /// Seed of the streaks.
    pub seed: u32,
}

impl VertexPaint for Runoff {
    fn mark(&self, point: Vector3<f32>) -> (Vector3<f32>, f32) {
        // Streaks: noise drawn out along the fall, so each runs its own
        // length down.
        let streak = fbm_perlin_3d(
            point.x * 60.0,
            point.y * 4.0,
            point.z * 60.0,
            2,
            0.5,
            self.seed,
        );
        let reach = self.trail * (0.55 + 0.9 * streak).clamp(0.05, 1.0);
        let t = ((self.above - point.y) / reach).clamp(0.0, 1.0);
        (self.tint, 1.0 - t * t * (3.0 - 2.0 * t))
    }
}

/// Where `clay`'s surface is met, going out from `from` inside it along
/// `along`.
pub fn surface_along(clay: &Clay, from: Vector3<f32>, along: Vector3<f32>) -> Vector3<f32> {
    let along = along.normalize();
    let (mut inside, mut outside) = (0.0, 2.0);
    for _ in 0..28 {
        let middle = 0.5 * (inside + outside);
        if clay.distance(from + along * middle) < 0.0 {
            inside = middle;
        } else {
            outside = middle;
        }
    }
    from + along * inside
}

#[cfg(test)]
mod tests {
    use rustc_hash::FxHashMap;

    use super::*;

    const PAINT: FinPaint = FinPaint {
        ray: (1.0, 1.0, 1.0),
        web: (0.5, 0.5, 0.5),
        edge: (0.2, 0.2, 0.2),
    };

    /// A fan of four rays in the plane z = 0, from the origin toward +Y.
    fn fan() -> Fin {
        let rays = (0..4)
            .map(|ray| {
                let x = -0.15 + 0.1 * ray as f32;
                Ray::new(Vector3::new(0.2 * x, 0.0, 0.0), Vector3::new(x, 0.3, 0.0))
            })
            .collect();
        Fin::on(rays, PAINT, 1).thick(0.004, 0.001).edged(0.2, 0.0)
    }

    #[test]
    fn a_fin_is_a_closed_thin_sheet_facing_out_on_both_faces() {
        let mesh = fan().mesh(6, 10, MaterialId(0));
        // Every edge shared by two triangles: nothing to see into.
        let mut edges: FxHashMap<(u32, u32), u32> = FxHashMap::default();
        for t in mesh.indices.chunks_exact(3) {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *edges.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        assert!(edges.values().all(|shared| *shared == 2));
        // Rays in order toward +X, running to +Y: its front faces +Z.
        let half = mesh.vertices.len() / 2;
        for (at, vertex) in mesh.vertices.iter().enumerate() {
            let front = if at < half { 1.0 } else { -1.0 };
            assert!(vertex.normal.z * front > 0.5, "{:?}", vertex.normal);
            assert!(vertex.pos.z.abs() <= 0.0051);
        }
        for t in mesh.indices.chunks_exact(3).take(2 * 18 * 10 * 2) {
            let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.vertices[i as usize]);
            let facing = (b.pos - a.pos).cross(&(c.pos - a.pos));
            assert!(facing.dot(&a.normal) >= 0.0, "a face is wound inward");
        }
    }

    #[test]
    fn a_tear_shortens_the_web_and_a_hole_opens_it() {
        let whole = fan();
        let reach_at = |fin: &Fin, u: f32| fin.reach(u);
        let mut torn = fan();
        torn.tears.push(Tear {
            at: 1.5,
            width: 0.6,
            depth: 0.7,
        });
        assert!(reach_at(&torn, 1.5) < 0.4 * reach_at(&whole, 1.5));
        assert!((reach_at(&torn, 0.5) - reach_at(&whole, 0.5)).abs() < 1e-6);

        let mut holed = fan();
        holed.holes.push(Hole {
            at: (1.5, 0.5),
            reach: (0.3, 0.15),
        });
        let triangles = |fin: &Fin| fin.mesh(6, 10, MaterialId(0)).indices.len() / 3;
        assert!(triangles(&holed) < triangles(&whole));
    }

    #[test]
    fn a_crown_stands_on_its_root_along_its_up() {
        let tooth = crown(
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::y(),
            Vector3::z(),
            (0.008, 0.01, 0.005),
        );
        let Lump::Ball(shape) = tooth else {
            panic!("a crown is a ball");
        };
        assert!(shape.distance_outside(Vector3::new(1.0, 2.0092, 3.0)).abs() < 2e-4);
        assert!(
            shape
                .distance_outside(Vector3::new(1.004, 2.0042, 3.0))
                .abs()
                < 2e-4
        );
    }
}
