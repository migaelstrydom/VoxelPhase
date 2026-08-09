//! Do materials that behave differently *look* different?
//!
//! One sphere per physics archetype, each shaded with the finish
//! [`PhysicalSurface`] derives from its collider parameters — nothing here is
//! authored by hand. The sheet answers the only question the derivation has to
//! get right: can a player tell these apart at a glance, and does what they see
//! predict what the object will do?
//!
//! Read it as a set, not tile by tile. A single tile always looks plausible;
//! the failure mode is two rows that render alike.

use nalgebra::{Point3, Vector3};

use crate::core::error::EngineResult;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::rendering::colour::Colour;
use crate::rendering::material::Material;
use crate::rendering::physical_finish::PhysicalSurface;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};

/// An archetype: what it is, how it behaves, and what colour a level would
/// plausibly give it. Colour is authored — the derivation only decides finish —
/// but a metal shows its metalness through the albedo it tints, so a flat grey
/// set would hide half of what is being judged.
struct Archetype {
    name: &'static str,
    physics: PhysicalSurface,
    albedo: Colour,
}

/// Deliberately spread across the space the engine's spawnables actually use,
/// with two pairs that differ in only one parameter: stone and rubber share a
/// friction, ice and rubber sit at opposite ends of it.
const ARCHETYPES: [Archetype; 6] = [
    Archetype {
        name: "ice",
        physics: PhysicalSurface {
            restitution: 0.05,
            friction: 0.05,
            density: 917.0,
        },
        albedo: Colour {
            r: 0.62,
            g: 0.78,
            b: 0.86,
            a: 1.0,
        },
    },
    Archetype {
        name: "cardboard",
        physics: PhysicalSurface {
            restitution: 0.05,
            friction: 0.7,
            density: 50.0,
        },
        albedo: Colour {
            r: 0.62,
            g: 0.48,
            b: 0.32,
            a: 1.0,
        },
    },
    Archetype {
        name: "wood",
        physics: PhysicalSurface {
            restitution: 0.2,
            friction: 0.5,
            density: 500.0,
        },
        albedo: Colour {
            r: 0.52,
            g: 0.34,
            b: 0.18,
            a: 1.0,
        },
    },
    Archetype {
        name: "rubber",
        physics: PhysicalSurface {
            restitution: 0.85,
            friction: 0.9,
            density: 1100.0,
        },
        albedo: Colour {
            r: 0.55,
            g: 0.11,
            b: 0.13,
            a: 1.0,
        },
    },
    Archetype {
        name: "stone",
        physics: PhysicalSurface {
            restitution: 0.1,
            friction: 0.9,
            density: 2600.0,
        },
        albedo: Colour {
            r: 0.48,
            g: 0.47,
            b: 0.44,
            a: 1.0,
        },
    },
    Archetype {
        name: "steel",
        physics: PhysicalSurface {
            restitution: 0.4,
            friction: 0.35,
            density: 7800.0,
        },
        albedo: Colour {
            r: 0.66,
            g: 0.67,
            b: 0.70,
            a: 1.0,
        },
    },
];

/// Sphere tessellation, high enough that the silhouette is never what is being
/// judged. Matches `material_grid` so the two sheets are comparable.
const SEGMENTS: u32 = 48;
const RINGS: u32 = 32;

pub struct PhysicsFinish;

impl VisualScene for PhysicsFinish {
    fn name(&self) -> &str {
        "physics_finish"
    }

    fn description(&self) -> &str {
        "Finishes derived from collider parameters. Can you see what a thing will do?"
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let indices = generate_sphere_indices(SEGMENTS, RINGS);

        let camera =
            SceneCamera::looking_at(Point3::new(0.0, 0.6, 3.4), Point3::new(0.0, 0.0, 0.0));

        // The same sun as `material_grid`: off to one side and above, so the
        // highlight lands where a roughness difference is easiest to read.
        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.5, 0.6, 0.62));

        let shots = ARCHETYPES
            .iter()
            .map(|archetype| {
                let finish = archetype.physics.finish();

                let label = format!(
                    "{}  mu {:.2}  e {:.2}  rho {:.0}  ->  rough {:.2}  metal {:.2}",
                    archetype.name,
                    archetype.physics.friction,
                    archetype.physics.restitution,
                    archetype.physics.density,
                    finish.roughness,
                    finish.metallic,
                );

                let surface = Material::coloured(archetype.albedo)
                    .with_finish(finish)
                    .surface_params();

                let mesh = SceneMesh::new(
                    generate_sphere_vertices(1.0, SEGMENTS, RINGS, archetype.albedo),
                    indices.clone(),
                )
                .with_surface(surface);

                SceneShot::new(label, camera)
                    .with_environment(environment.clone())
                    .with_mesh(mesh)
            })
            .collect();

        Ok(shots)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Guards the plumbing rather than the look: if this holds, a sheet that
    /// reads wrong is a calibration problem, never a lost parameter.
    #[test]
    fn each_tile_ships_its_derived_finish() {
        for archetype in &ARCHETYPES {
            let finish = archetype.physics.finish();
            let params = Material::coloured(archetype.albedo)
                .with_finish(finish)
                .surface_params();

            assert_eq!(params.surface[0], finish.roughness, "{}", archetype.name);
            assert_eq!(params.surface[1], finish.metallic, "{}", archetype.name);
        }
    }
}
