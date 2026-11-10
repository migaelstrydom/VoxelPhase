//! A glazed orb: a lapis sphere banded and studded in gilt, with a finial
//! on top, the kind a painted garden floats on its lakes.
//!
//! ```text
//!   GlazedOrbDef ──model──▶ SculptedParts ──painted_model──▶ Model
//!                ──spawn──▶ WeightedBall: one sphere, floating, finial up
//! ```
//!
//! `visual_bench fruit` shows it beside the fruit.

use std::f32::consts::TAU;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::shared::weighted_ball::WeightedBall;
use super::{MaterialCtx, Spawnable};
use crate::app::sculpt::{painted_model, scatter, ContactShade, Ellipsoid, Mottle, SculptedPart};
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::rendering::physical_finish::PhysicalSurface;

/// Fired clay, hollow: hard, slick, and floating high.
const CERAMIC: PhysicalSurface = PhysicalSurface {
    restitution: 0.3,
    friction: 0.4,
    density: 450.0,
};

/// The glaze: a deep shine, softer than glass.
const GLAZE: SurfaceFinish = SurfaceFinish {
    roughness: 0.3,
    metallic: 0.0,
};

/// The glaze's microstructure: faint, so the highlight is not a mirror's.
const GLAZE_GRAIN: GrainSpec = GrainSpec::STONE.with_scale(1.0).with_strength(0.1);

/// Gilding. Half metallic reads as gold; fully metallic goes olive against
/// a blue sky.
const GILT: SurfaceFinish = SurfaceFinish {
    roughness: 0.35,
    metallic: 0.5,
};

/// How many materials [`GlazedOrbDef::materials`] makes.
const MATERIAL_COUNT: usize = 2;

/// Rings of studs: each ring's elevation above the equator, radians, and
/// how many studs it holds.
const STUD_RINGS: [(f32, usize); 2] = [(0.62, 10), (-0.62, 10)];

/// A stud's radius, as a share of the orb's.
const STUD_RADIUS: f32 = 0.06;

/// A glazed orb. `pos` is its centre.
#[derive(Deserialize)]
pub struct GlazedOrbDef {
    pub pos: (f32, f32, f32),
    /// Radius of the sphere it collides as, m.
    #[serde(default = "GlazedOrbDef::default_radius")]
    pub radius: f32,
}

impl GlazedOrbDef {
    pub fn default_radius() -> f32 {
        0.55
    }

    /// Its materials, in the order [`Self::model`] indexes them: the glaze
    /// and the gilt.
    pub fn materials() -> Vec<Material> {
        let plain = || Material::coloured(Colour::WHITE);
        vec![
            plain().with_finish(GLAZE).with_grain(GLAZE_GRAIN),
            plain().with_finish(GILT),
        ]
    }

    /// How an orb of `radius` looks, drawn in `materials` (registered from
    /// [`Self::materials`], in order), round its centre with its finial up.
    pub fn model(radius: f32, materials: &[MaterialId]) -> Model {
        let r = radius;
        let lapis = (Colour::new(0.015, 0.06, 0.33, 1.0), materials[0]);
        let gold = (Colour::new(1.0, 0.62, 0.18, 1.0), materials[1]);
        let sphere = Ellipsoid::ball(r, Vector3::zeros());
        let scaled = |x: f32, y: f32, z: f32| (x * r, y * r, z * r);

        let mut parts = vec![
            // A band round the equator.
            SculptedPart::new(
                Ellipsoid::new(scaled(1.015, 0.075, 1.015), (0.0, 0.0, 0.0)),
                gold,
            ),
            // The finial: a collar on the crown, a stem, a ball and a point.
            SculptedPart::new(
                Ellipsoid::new(scaled(0.26, 0.06, 0.26), scaled(0.0, 0.975, 0.0)),
                gold,
            ),
            SculptedPart::new(
                Ellipsoid::new(scaled(0.04, 0.15, 0.04), scaled(0.0, 1.13, 0.0)),
                gold,
            ),
            SculptedPart::new(Ellipsoid::ball(0.09 * r, Vector3::y() * 1.3 * r), gold),
            SculptedPart::new(
                Ellipsoid::new(scaled(0.022, 0.1, 0.022), scaled(0.0, 1.45, 0.0)),
                gold,
            ),
        ];
        for (ring, &(elevation, count)) in STUD_RINGS.iter().enumerate() {
            for i in 0..count {
                let key = (ring * 64 + i) as u32;
                let azimuth = (i as f32 + 0.5 * ring as f32) / count as f32 * TAU;
                let (point, normal) = sphere.surface_at(azimuth, elevation);
                let stud = STUD_RADIUS * r * (1.0 + 0.15 * scatter(key, 0));
                parts.push(SculptedPart::new(
                    Ellipsoid::ball(stud, point - normal * stud * 0.4),
                    gold,
                ));
            }
        }
        parts.push(SculptedPart::new(sphere, lapis));

        // Glaze pools unevenly: a little lighter and darker, no warmer.
        let mottle = Mottle {
            frequency: 1.6 / r,
            value: 0.3,
            warmth: 0.0,
            seed: 13,
        };
        let shade = ContactShade {
            reach: 0.07 * r,
            depth: 0.55,
            floor: 0.35,
        };
        painted_model(&parts, &mottle, &shade)
    }
}

impl Spawnable for GlazedOrbDef {
    fn validate(&self) -> Result<(), String> {
        if self.radius > 0.0 {
            Ok(())
        } else {
            Err(format!(
                "an orb's radius must be positive, not {}",
                self.radius
            ))
        }
    }

    fn material_count(&self) -> usize {
        MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(Self::materials()
            .into_iter()
            .map(|material| ctx.materials.register(material))
            .collect())
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let ball = WeightedBall {
            radius: self.radius,
            surface: CERAMIC,
        };
        let centre = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        vec![ball.spawn(world, centre, Self::model(self.radius, materials))]
    }
}
