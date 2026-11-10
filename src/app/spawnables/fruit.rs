//! Giant garden fruit: a cherry, a peach, a blackberry, each the size of a
//! barrel, to float on a lake or lie on a lawn.
//!
//! ```text
//!   FruitKind ──parts──▶ SculptedParts ──painted_model──▶ Model
//!   FruitDef ──spawn──▶ WeightedBall: one sphere, floating, stalk up
//! ```
//!
//! `visual_bench fruit` shows them.

use std::f32::consts::{FRAC_PI_2, TAU};

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::shared::weighted_ball::WeightedBall;
use super::{MaterialCtx, Spawnable};
use crate::app::sculpt::{
    painted_model, scatter, Clouds, ContactShade, Ellipsoid, Mottle, SculptedPart, SkinPattern,
    Spots, Veins, Warp,
};
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::resources::textures::TextureManager;

/// A fruit's flesh: soft, grippy, and light enough, with its hollow, to
/// float with most of it out of the water.
const FLESH: PhysicalSurface = PhysicalSurface {
    restitution: 0.25,
    friction: 0.6,
    density: 600.0,
};

/// A cherry's skin: taut and wet-looking, but short of a mirror, which
/// would cut it in two along the horizon it reflects.
const GLOSS: SurfaceFinish = SurfaceFinish {
    roughness: 0.3,
    metallic: 0.0,
};

/// A drupelet's skin: a soft bloom, so each carries a glint the size of
/// itself and not a pinhead.
const BLOOM: SurfaceFinish = SurfaceFinish {
    roughness: 0.4,
    metallic: 0.0,
};

/// A peach's skin: down, with no highlight to speak of.
const DOWN: SurfaceFinish = SurfaceFinish {
    roughness: 0.9,
    metallic: 0.0,
};

/// Down's microstructure: fine and soft.
const DOWN_GRAIN: GrainSpec = GrainSpec::STONE.with_scale(1.4).with_strength(0.2);

/// Stalks and leaves: matte.
const STALK: SurfaceFinish = SurfaceFinish {
    roughness: 0.85,
    metallic: 0.0,
};

/// Texels round a fruit's skin pattern.
const PATTERN_WIDTH: u32 = 512;

/// How many materials [`FruitDef::materials`] makes.
const MATERIAL_COUNT: usize = 2;

/// No veins: a fruit's skin has none.
const NO_VEINS: Veins = Veins {
    frequency: 1.0,
    width: 0.0,
    tint: Vector3::new(1.0, 1.0, 1.0),
    seed: 0,
};

/// Rings of drupelets on a blackberry, as elevations above its equator,
/// radians, and the spacing between neighbours round a ring, as a share of
/// the fruit's radius.
const DRUPELET_RINGS: [f32; 7] = [-1.25, -0.85, -0.45, -0.05, 0.35, 0.75, 1.12];
const DRUPELET_SPACING: f32 = 0.36;

/// A drupelet's radius, as a share of the fruit's, and how far one strays
/// from it, as a share of that.
const DRUPELET_RADIUS: f32 = 0.2;
const DRUPELET_SIZE_SPREAD: f32 = 0.18;

/// The share of drupelets still red.
const UNRIPE_SHARE: f32 = 0.1;

/// Which fruit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
pub enum FruitKind {
    /// Deep red and glossy, on a long bent stalk.
    Cherry,
    /// Downy rose and gold, cleft down one side, with a leaf.
    Peach,
    /// A cluster of black drupelets, a few still red, under a green calyx.
    Blackberry,
}

impl FruitKind {
    /// Every kind, for tools that show them all.
    pub const ALL: [FruitKind; 3] = [FruitKind::Cherry, FruitKind::Peach, FruitKind::Blackberry];

    /// Its skin's finish and grain.
    fn skin(self) -> (SurfaceFinish, GrainSpec) {
        match self {
            FruitKind::Cherry => (GLOSS, GrainSpec::NONE),
            FruitKind::Blackberry => (BLOOM, GrainSpec::NONE),
            FruitKind::Peach => (DOWN, DOWN_GRAIN),
        }
    }

    /// The pattern painted on its skin. Frequencies are per radius of the
    /// part painted, so the same pattern serves a fruit of any size.
    fn pattern(self) -> SkinPattern {
        let none = Spots {
            frequency: 1.0,
            presence: 0.0,
            radius: (0.1, 0.1),
            tint: Vector3::new(1.0, 1.0, 1.0),
            salt: 0,
        };
        match self {
            // Pale lenticels and a little cloud under the shine.
            FruitKind::Cherry => SkinPattern {
                width: PATTERN_WIDTH,
                clouds: Clouds {
                    frequency: 3.0,
                    value: 0.14,
                    seed: 41,
                },
                veins: NO_VEINS,
                freckles: none,
                pores: Spots {
                    frequency: 22.0,
                    presence: 0.3,
                    radius: (0.1, 0.2),
                    tint: Vector3::new(1.5, 1.25, 1.1),
                    salt: 42,
                },
            },
            // A blush in broad soft patches, and red freckles over it.
            FruitKind::Peach => SkinPattern {
                width: PATTERN_WIDTH,
                clouds: Clouds {
                    frequency: 2.5,
                    value: 0.1,
                    seed: 51,
                },
                veins: NO_VEINS,
                freckles: Spots {
                    frequency: 1.7,
                    presence: 0.6,
                    radius: (0.55, 0.95),
                    tint: Vector3::new(0.95, 0.5, 0.55),
                    salt: 52,
                },
                pores: Spots {
                    frequency: 13.0,
                    presence: 0.5,
                    radius: (0.15, 0.35),
                    tint: Vector3::new(0.86, 0.6, 0.56),
                    salt: 53,
                },
            },
            FruitKind::Blackberry => SkinPattern {
                width: PATTERN_WIDTH,
                clouds: Clouds {
                    frequency: 2.0,
                    value: 0.2,
                    seed: 61,
                },
                veins: NO_VEINS,
                freckles: none,
                pores: none,
            },
        }
    }

    /// Patches over the whole fruit, of radius `radius`: a peach's blush is
    /// this, a cherry's is only a little unevenness.
    fn mottle(self, radius: f32) -> Mottle {
        let (frequency, value, warmth) = match self {
            FruitKind::Cherry => (2.0, 0.22, 0.1),
            FruitKind::Peach => (1.3, 0.16, 0.34),
            FruitKind::Blackberry => (2.5, 0.2, 0.1),
        };
        Mottle {
            frequency: frequency / radius,
            value,
            warmth,
            seed: 7,
        }
    }
}

/// A giant fruit. `pos` is its centre.
#[derive(Deserialize)]
pub struct FruitDef {
    pub pos: (f32, f32, f32),
    pub kind: FruitKind,
    /// Radius of the sphere it collides as, m.
    #[serde(default = "FruitDef::default_radius")]
    pub radius: f32,
}

impl FruitDef {
    pub fn default_radius() -> f32 {
        0.5
    }

    /// A `kind` of fruit's materials, in the order [`Self::model`] indexes
    /// them: its skin, painted with its pattern, and a matte one for stalk
    /// and leaf. Bakes the pattern's texture.
    pub fn materials(kind: FruitKind, textures: &TextureManager) -> EngineResult<Vec<Material>> {
        let pattern = kind.pattern();
        let texture =
            textures.create_from_rgba(pattern.width, pattern.height(), &pattern.texture(), true)?;
        let (finish, grain) = kind.skin();
        Ok(vec![
            Material::textured(texture)
                .with_finish(finish)
                .with_grain(grain),
            Material::coloured(Colour::WHITE).with_finish(STALK),
        ])
    }

    /// How a `kind` of fruit of `radius` looks, drawn in `materials`
    /// (registered from [`Self::materials`], in order), round its centre
    /// with its stalk up.
    pub fn model(kind: FruitKind, radius: f32, materials: &[MaterialId]) -> Model {
        let looks = Looks {
            skin: materials[0],
            stalk: materials[1],
        };
        let parts = match kind {
            FruitKind::Cherry => cherry(radius, looks),
            FruitKind::Peach => peach(radius, looks),
            FruitKind::Blackberry => blackberry(radius, looks),
        };
        let shade = ContactShade {
            reach: 0.12 * radius,
            depth: 0.65,
            floor: 0.3,
        };
        painted_model(&parts, &kind.mottle(radius), &shade)
    }
}

/// A fruit's two materials.
#[derive(Clone, Copy)]
struct Looks {
    skin: MaterialId,
    stalk: MaterialId,
}

impl Looks {
    /// Stalk wood and leaf green.
    fn wood(self) -> (Colour, MaterialId) {
        (Colour::new(0.24, 0.25, 0.07, 1.0), self.stalk)
    }

    fn leaf(self) -> (Colour, MaterialId) {
        (Colour::new(0.17, 0.33, 0.07, 1.0), self.stalk)
    }
}

/// A point `(x, y, z)` in shares of `radius`.
fn at(radius: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    (x * radius, y * radius, z * radius)
}

/// A cherry: heart-shaped, narrowing to its tip, with a faint seam down
/// one side and a dimple where a long bent stalk leaves it.
fn cherry(r: f32, looks: Looks) -> Vec<SculptedPart> {
    let red = (Colour::new(0.36, 0.014, 0.03, 1.0), looks.skin);
    let up = Vector3::y();
    vec![
        SculptedPart::new(Ellipsoid::new(at(r, 0.97, 1.0, 0.97), (0.0, 0.0, 0.0)), red)
            .warped(Warp::Taper {
                along: -up,
                tip: 0.8,
            })
            .warped(Warp::Cleft {
                pole: up,
                side: Vector3::z(),
                width: 0.3,
                depth: 0.04,
            })
            .warped(Warp::Dimple {
                at: up,
                width: 0.3,
                depth: 0.17 * r,
            })
            .warped(Warp::Lumps {
                frequency: 1.6 / r,
                amplitude: 0.02 * r,
                seed: 5,
            }),
        SculptedPart::new(
            Ellipsoid::new(at(r, 0.035, 0.46, 0.035), at(r, 0.0, 1.25, 0.0)),
            looks.wood(),
        )
        .warped(Warp::Bend {
            along: up,
            toward: Vector3::x(),
            amount: 0.3 * r,
        }),
        // The knuckle it was picked at.
        SculptedPart::new(
            Ellipsoid::new(at(r, 0.075, 0.05, 0.06), at(r, 0.3, 1.7, 0.0)),
            looks.wood(),
        ),
    ]
}

/// A peach: cleft down one side, dimpled at the top, with a stub of stalk
/// and one leaf.
fn peach(r: f32, looks: Looks) -> Vec<SculptedPart> {
    let rose = (Colour::new(0.86, 0.44, 0.26, 1.0), looks.skin);
    let up = Vector3::y();
    let forward = Vector3::z();
    vec![
        SculptedPart::new(Ellipsoid::new(at(r, 1.0, 0.92, 1.0), (0.0, 0.0, 0.0)), rose)
            .warped(Warp::Cleft {
                pole: up,
                side: forward,
                width: 0.24,
                depth: 0.09,
            })
            .warped(Warp::Dimple {
                at: up,
                width: 0.34,
                depth: 0.15 * r,
            })
            .warped(Warp::Lumps {
                frequency: 1.2 / r,
                amplitude: 0.025 * r,
                seed: 9,
            }),
        SculptedPart::new(
            Ellipsoid::new(at(r, 0.05, 0.13, 0.05), at(r, 0.0, 0.86, 0.0)),
            looks.wood(),
        ),
        SculptedPart::new(
            Ellipsoid::new(at(r, 0.2, 0.03, 0.4), at(r, 0.05, 1.0, -0.36))
                .yawed(2.9)
                .pitched(-0.35),
            looks.leaf(),
        )
        .warped(Warp::Taper {
            along: forward,
            tip: 0.12,
        })
        .warped(Warp::Bend {
            along: forward,
            toward: -up,
            amount: 0.1 * r,
        }),
    ]
}

/// A blackberry: drupelets in rings round a core that narrows to its tip,
/// under a calyx of five sepals and a stub of stalk.
fn blackberry(r: f32, looks: Looks) -> Vec<SculptedPart> {
    let black = Colour::new(0.035, 0.015, 0.07, 1.0);
    let unripe = Colour::new(0.2, 0.012, 0.04, 1.0);
    let core = SculptedPart::new(
        Ellipsoid::new(at(r, 0.78, 0.84, 0.78), at(r, 0.0, -0.04, 0.0)),
        (black, looks.skin),
    )
    .warped(Warp::Taper {
        along: -Vector3::y(),
        tip: 0.7,
    });

    let mut parts = Vec::new();
    for (ring, &elevation) in DRUPELET_RINGS.iter().enumerate() {
        let round = TAU * 0.78 * elevation.cos();
        let count = ((round / DRUPELET_SPACING).round() as usize).max(3);
        let stagger = if ring % 2 == 0 { 0.0 } else { 0.5 };
        for i in 0..count {
            let key = (ring * 64 + i) as u32;
            let azimuth = (i as f32 + stagger + 0.2 * scatter(key, 0)) / count as f32 * TAU;
            let (point, normal) = core.surface_at(azimuth, elevation + 0.05 * scatter(key, 1));
            let radius = DRUPELET_RADIUS * r * (1.0 + DRUPELET_SIZE_SPREAD * scatter(key, 2));
            let colour = if scatter(key, 3) * 0.5 + 0.5 < UNRIPE_SHARE {
                unripe
            } else {
                black
            };
            parts.push(SculptedPart::new(
                Ellipsoid::ball(radius, point - normal * radius * 0.2),
                (colour, looks.skin),
            ));
        }
    }
    // One at the tip, where the rings close.
    let (tip, _) = core.surface_at(0.0, -FRAC_PI_2);
    parts.push(SculptedPart::new(
        Ellipsoid::ball(DRUPELET_RADIUS * r, tip),
        (black, looks.skin),
    ));

    for i in 0..5 {
        let azimuth = i as f32 / 5.0 * TAU;
        let out = Vector3::new(azimuth.sin(), 0.0, azimuth.cos());
        let centre = out * 0.24 * r + Vector3::y() * 0.9 * r;
        parts.push(
            SculptedPart::new(
                Ellipsoid::new(at(r, 0.1, 0.025, 0.24), (centre.x, centre.y, centre.z))
                    .yawed(azimuth)
                    .pitched(0.3),
                looks.leaf(),
            )
            .warped(Warp::Taper {
                along: Vector3::z(),
                tip: 0.15,
            }),
        );
    }
    parts.push(SculptedPart::new(
        Ellipsoid::new(at(r, 0.05, 0.16, 0.05), at(r, 0.0, 1.02, 0.0)),
        looks.wood(),
    ));
    parts.push(core);
    parts
}

impl Spawnable for FruitDef {
    fn validate(&self) -> Result<(), String> {
        if self.radius > 0.0 {
            Ok(())
        } else {
            Err(format!(
                "a fruit's radius must be positive, not {}",
                self.radius
            ))
        }
    }

    fn material_count(&self) -> usize {
        MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(Self::materials(self.kind, ctx.textures)?
            .into_iter()
            .map(|material| ctx.materials.register(material))
            .collect())
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let ball = WeightedBall {
            radius: self.radius,
            surface: FLESH,
        };
        let centre = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        vec![ball.spawn(
            world,
            centre,
            Self::model(self.kind, self.radius, materials),
        )]
    }
}
