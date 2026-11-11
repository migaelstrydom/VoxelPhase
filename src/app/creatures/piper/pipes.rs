//! Pipes of dark turned wood, jointed and ferruled in brass: the drones
//! that stand out of the bag, the chanter and the blowpipe.
//!
//! ```text
//!   Pipe { foot, tip, girth, joints, stock, mouth, holes }
//!     ├── wood   shaft, stock, a swelling and beads at each joint, bored
//!     │          and holed, its grain run along it
//!     └── brass  a ferrule over each joint and on the stock, and a flared
//!                bell at the mouth
//! ```

use nalgebra::Vector3;

use super::bag::CORE;
use super::frame::{MUZZLE, RADIUS};
use super::shaping::{ease, paint, placed, surface, triple, turned_to, vector};
use crate::app::creatures::palette::Paint;
use crate::app::sculpt::{Clay, Lump};
use crate::model::MeshPrimitive;
use crate::utils::noise::fbm_perlin_3d;

/// Side of the grid a pipe is skinned from, m; a thin pipe's is finer.
const PIPE_GRAIN: f32 = 0.009;

/// The dark of a pipe's wood and the light of its grain; brass where it is
/// rubbed, where it has dulled, and down a pipe's bore (linear).
const WOOD: Vector3<f32> = Vector3::new(0.06, 0.03, 0.018);
const WOOD_GRAIN: Vector3<f32> = Vector3::new(0.2, 0.1, 0.05);
const BRASS: Vector3<f32> = Vector3::new(0.95, 0.72, 0.34);
const BRASS_DULL: Vector3<f32> = Vector3::new(0.3, 0.2, 0.08);
const BORE: Vector3<f32> = Vector3::new(0.03, 0.022, 0.016);

/// How a pipe ends.
#[derive(Clone, Copy)]
enum Mouth {
    /// Cut off square, its bore showing.
    Plain,
    /// A flared bell of brass, this far across at its rim from its middle,
    /// m.
    Flared(f32),
    /// Snapped off.
    Broken,
}

/// A pipe of dark turned wood, jointed and ferruled in brass.
pub(super) struct Pipe {
    /// Where it starts and where its mouth is, in the piper's frame.
    pub(super) foot: Vector3<f32>,
    pub(super) tip: Vector3<f32>,
    /// Radius of its shaft, m.
    pub(super) girth: f32,
    /// Where along it (0 at its foot, 1 at its tip) it is jointed: each
    /// joint a swelling of the wood with a brass ferrule over it.
    joints: &'static [f32],
    /// How far its stock, the socket the bag is tied round, reaches below
    /// its foot and above it, m. Nothing for a pipe with no stock.
    pub(super) stock: Option<(f32, f32)>,
    /// How it ends.
    mouth: Mouth,
    /// Finger holes down its front.
    holes: usize,
}

impl Pipe {
    fn long(&self) -> f32 {
        (self.tip - self.foot).norm()
    }

    /// Its wood, about its foot with +Y along it.
    fn wood(&self) -> MeshPrimitive {
        let (g, long) = (self.girth, self.long());
        let dark = triple(WOOD);
        // Under a bell the shaft stops at the bulb the brass is fitted to.
        let shaft = match self.mouth {
            Mouth::Flared(_) => long - 4.5 * g,
            Mouth::Plain | Mouth::Broken => long,
        };
        let mut wood = Clay::new(0.3 * g).add(
            Lump::limb((0.0, 0.0, 0.0), (0.0, shaft, 0.0), (1.05 * g, 0.95 * g)),
            dark,
        );
        if let Some((below, above)) = self.stock {
            wood = wood
                .add(
                    Lump::limb((0.0, -below, 0.0), (0.0, above, 0.0), (1.9 * g, 1.8 * g)),
                    dark,
                )
                .add(
                    Lump::mass((2.15 * g, 0.5 * g, 2.15 * g), (0.0, above, 0.0)),
                    dark,
                );
        }
        for joint in self.joints {
            let at = joint * long;
            wood = wood
                // The swelling the next length's tenon sits in, and a bead
                // turned above and below it.
                .add(
                    Lump::mass((1.55 * g, 1.7 * g, 1.55 * g), (0.0, at - 0.6 * g, 0.0)),
                    dark,
                )
                .add(
                    Lump::mass((1.4 * g, 0.4 * g, 1.4 * g), (0.0, at + 2.6 * g, 0.0)),
                    dark,
                )
                .add(
                    Lump::mass((1.35 * g, 0.35 * g, 1.35 * g), (0.0, at - 3.4 * g, 0.0)),
                    dark,
                );
        }
        match self.mouth {
            // The bulb the brass is fitted over.
            Mouth::Flared(_) => {
                wood = wood.add(
                    Lump::mass((1.6 * g, 2.2 * g, 1.6 * g), (0.0, long - 4.5 * g, 0.0)),
                    dark,
                );
            }
            Mouth::Plain => {
                wood = wood
                    .add(
                        Lump::mass((1.5 * g, 0.6 * g, 1.5 * g), (0.0, long - 0.8 * g, 0.0)),
                        dark,
                    )
                    .cut(
                        Lump::limb(
                            (0.0, long + g, 0.0),
                            (0.0, long - 2.0 * g, 0.0),
                            (0.55 * g, 0.5 * g),
                        ),
                        0.0,
                    );
            }
            // Bitten off on a slant, with a split run down from it.
            Mouth::Broken => {
                wood = wood
                    .cut(Lump::ball(2.4 * g, (1.5 * g, long + 1.1 * g, 0.6 * g)), 0.0)
                    .cut(
                        Lump::ball(1.1 * g, (-0.9 * g, long + 0.25 * g, -0.5 * g)),
                        0.0,
                    )
                    .cut(
                        Lump::limb(
                            (-0.2 * g, long, 1.0 * g),
                            (0.1 * g, long - 6.0 * g, 1.05 * g),
                            (0.3 * g, 0.1 * g),
                        ),
                        0.0,
                    );
            }
        }
        for hole in 0..self.holes {
            let at = long * (0.3 + 0.52 * hole as f32 / self.holes.max(2) as f32);
            wood = wood.cut(Lump::ball(0.42 * g, (0.0, at, 1.02 * g)), 0.001);
        }
        let mut mesh = wood.skin(PIPE_GRAIN.min(0.5 * g), Paint::Wood.id());
        // Grain that runs the length of it, pale where hands have polished
        // the high places, black with old oil in the turnings.
        paint(&mut mesh, |vertex| {
            let p = vertex.pos;
            let streak = fbm_perlin_3d(p.x * 90.0, p.y * 6.0, p.z * 90.0, 3, 0.5, 71);
            let proud = ease(1.15 * g, 1.6 * g, p.xz().norm());
            let lit = (0.45 + 0.45 * streak + 0.35 * proud).clamp(0.0, 1.0);
            WOOD.lerp(&WOOD_GRAIN, lit) * (0.35 + 0.65 * vertex.ao)
        });
        mesh
    }

    /// Its brass, about its foot with +Y along it: a ferrule over each
    /// joint and on its stock, and its bell if it has one.
    fn brass(&self) -> Vec<MeshPrimitive> {
        let (g, long) = (self.girth, self.long());
        let white = (1.0, 1.0, 1.0);
        let mut ferrules = Clay::new(0.1 * g);
        let mut any = false;
        if let Some((_, above)) = self.stock {
            ferrules = ferrules.add(
                Lump::limb(
                    (0.0, above - 1.6 * g, 0.0),
                    (0.0, above - 0.9 * g, 0.0),
                    (2.05 * g, 2.05 * g),
                ),
                white,
            );
            any = true;
        }
        for joint in self.joints {
            let at = joint * long;
            ferrules = ferrules.add(
                Lump::limb(
                    (0.0, at - 0.5 * g, 0.0),
                    (0.0, at + 0.5 * g, 0.0),
                    (1.72 * g, 1.72 * g),
                ),
                white,
            );
            any = true;
        }
        let mut brass = Vec::new();
        if any {
            brass.push(ferrules.skin(PIPE_GRAIN.min(0.5 * g), Paint::Bronze.id()));
        }
        if let Mouth::Flared(rim) = self.mouth {
            // The bell: a stack of discs that widens to the rim and past
            // it, hollowed, and cut off at the rim.
            let wall = (0.2 * rim).clamp(0.012, 0.018);
            let (from, far) = (long - 4.6 * g, 4.0);
            let mut bell = Clay::new(0.03);
            for disc in 0..8 {
                let t = disc as f32 / 6.0;
                let radius = 1.3 * g + (rim - 1.3 * g) * t.powf(1.9);
                bell = bell.add(
                    Lump::mass((radius, 0.03, radius), (0.0, from + (long - from) * t, 0.0)),
                    white,
                );
            }
            let bell = bell
                .hollow(wall)
                .cut(Lump::ball(far, (0.0, long + far, 0.0)), 0.003);
            brass.push(bell.skin(PIPE_GRAIN.min(0.5 * wall), Paint::Bronze.id()));
        }
        for mesh in &mut brass {
            paint(mesh, |vertex| {
                let p = vertex.pos;
                let inward = -vertex.normal.dot(&Vector3::new(p.x, 0.0, p.z).normalize());
                if inward > 0.3 {
                    return BORE;
                }
                let worn = fbm_perlin_3d(p.x * 40.0, p.y * 40.0, p.z * 40.0, 2, 0.5, 73);
                let closed = (1.0 - vertex.ao) / 0.75;
                BRASS.lerp(
                    &BRASS_DULL,
                    (0.25 + 0.45 * worn + 1.2 * closed).clamp(0.0, 1.0),
                )
            });
        }
        brass
    }

    /// The pipe where it stands on the piper.
    pub(super) fn meshes(&self) -> Vec<MeshPrimitive> {
        let turn = turned_to(self.tip - self.foot, Vector3::z());
        let mut meshes = vec![self.wood()];
        meshes.extend(self.brass());
        meshes
            .into_iter()
            .map(|mesh| placed(mesh, turn, self.foot))
            .collect()
    }
}

/// The drones that stand out of the top of the bag: the bass, whose
/// mouth a bomb leaves by, a tenor, and a tenor snapped off short.
pub(super) fn drones(belly: &Clay) -> [Pipe; 3] {
    let core = vector(CORE);
    let foot = |along: (f32, f32, f32)| surface(belly, core, vector(along)).0;
    let bass = foot((0.03, 1.0, -0.2));
    let tenor = foot((-0.55, 0.8, -0.2));
    let stump = foot((0.52, 0.72, -0.5));
    let stock = Some((0.13, 0.17));
    [
        Pipe {
            foot: bass,
            tip: Vector3::y() * MUZZLE,
            girth: 0.023,
            joints: &[0.36, 0.68],
            stock,
            mouth: Mouth::Flared(0.092),
            holes: 0,
        },
        Pipe {
            foot: tenor,
            tip: tenor + Vector3::new(-0.42, 1.0, -0.12).normalize() * 0.64,
            girth: 0.02,
            joints: &[0.5],
            stock,
            mouth: Mouth::Flared(0.066),
            holes: 0,
        },
        Pipe {
            foot: stump,
            tip: stump + Vector3::new(0.5, 1.0, -0.4).normalize() * 0.4,
            girth: 0.02,
            joints: &[0.56],
            stock,
            mouth: Mouth::Broken,
            holes: 0,
        },
    ]
}

/// The chanter, hanging down in front, and the two lengths of the
/// blowpipe: from the bag up to its elbow, and from the elbow to the
/// lips, which blow along `blow`.
pub(super) fn small_pipes(belly: &Clay, lips: Vector3<f32>, blow: Vector3<f32>) -> [Pipe; 3] {
    let core = vector(CORE);
    let elbow = lips + blow * 0.14;
    let chanter = surface(belly, core, Vector3::new(0.3, -0.02, 1.0)).0;
    [
        Pipe {
            foot: chanter,
            tip: Vector3::new(chanter.x + 0.06, -RADIUS + 0.05, chanter.z + 0.19),
            girth: 0.015,
            joints: &[0.2],
            stock: Some((0.1, 0.05)),
            mouth: Mouth::Flared(0.027),
            holes: 5,
        },
        Pipe {
            foot: surface(belly, core, Vector3::new(-0.1, 0.08, 1.0)).0,
            tip: elbow,
            girth: 0.014,
            joints: &[0.93],
            stock: Some((0.1, 0.05)),
            mouth: Mouth::Plain,
            holes: 0,
        },
        Pipe {
            foot: elbow,
            tip: lips - blow * 0.012,
            girth: 0.011,
            joints: &[],
            stock: None,
            mouth: Mouth::Plain,
            holes: 0,
        },
    ]
}
