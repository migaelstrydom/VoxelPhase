//! Menhir spawnable — terrain-anchored standing stone.
//!
//! An egg-shaped monolith pinned to the terrain via a Fixed constraint,
//! drawn as old weathered stone. The collider is a UV sphere whose horizontal
//! radius blends linearly from `bottom_radius` (wide base) to `top_radius`
//! (narrow tip) as a function of latitude, producing a natural egg profile.
//!
//! The bottom portion is buried below the surface. When the terrain beneath
//! is destroyed, the anchor releases and the menhir topples as a free body.

use std::f32::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, SolidFace};
use super::shared::textures::seed_from_ground;
use super::stone::{weathered_model, StoneTexture};
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Longitude segments (around the equator).
const SEGMENTS: usize = 8;
/// Latitude rings between the poles (exclusive of poles themselves).
const RINGS: usize = 6;
/// Fraction of the menhir buried below the terrain surface.
const BURIED_FRACTION: f32 = 0.2;

#[derive(Deserialize)]
pub struct MenhirDef {
    /// Position (x, z). Y is determined by terrain surface height.
    pub pos: (f32, f32),
    /// Half-height of the full stone (including buried portion).
    #[serde(default = "MenhirDef::default_half_height")]
    pub half_height: f32,
    /// Horizontal radius at the widest point of the bottom half.
    #[serde(default = "MenhirDef::default_bottom_radius")]
    pub bottom_radius: f32,
    /// Horizontal radius at the widest point of the top half.
    #[serde(default = "MenhirDef::default_top_radius")]
    pub top_radius: f32,
    /// Stone density (kg/m^3). Default is granite's.
    #[serde(default = "MenhirDef::default_density")]
    pub density: f32,
}

impl MenhirDef {
    pub fn default_half_height() -> f32 {
        1.0
    }
    pub fn default_bottom_radius() -> f32 {
        0.4
    }
    pub fn default_top_radius() -> f32 {
        0.25
    }
    pub fn default_density() -> f32 {
        2700.0
    }

    /// The one declaration of what this object is made of. The collider takes
    /// the coefficients and the material takes the finish and grain, so the two
    /// cannot drift apart.
    ///
    /// The density stays authored per instance — a level may want a heavier
    /// stone — while everything about how sarsen looks comes from the library.
    fn substance(&self) -> Substance {
        substance::SARSEN.with_density(self.density)
    }
}

impl Spawnable for MenhirDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // A seed per stone, so two menhirs in a level are not the same rock
        // twice — which is also what keeps them out of each other's cache entry.
        let texture = StoneTexture::WEATHERED.with_seed(seed_from_ground(self.pos, 0));
        Ok(vec![texture.material(ctx, &self.substance())?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let material = materials[0];

        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };

        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let full_height = self.half_height * 2.0;
        let buried_depth = full_height * BURIED_FRACTION;

        let center_y = surface_y - buried_depth + self.half_height;
        let initial_pos = Point3::new(self.pos.0, center_y, self.pos.1);

        let egg = EggParams {
            half_height: self.half_height,
            bottom_radius: self.bottom_radius,
            top_radius: self.top_radius,
        };

        let hull = Arc::new(build_egg_hull(&egg));
        let thickness = 2.0 * self.bottom_radius.min(self.top_radius);
        let model = weathered_model(
            &hull,
            initial_pos.coords,
            StoneTexture::WEATHERED.uvs(thickness),
            material,
        );

        let collider = ColliderDesc::convex_hull(hull).of(&self.substance());

        let (body_handle, anchor_handle, upright_handle) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            // Welded with its foot in the ground, which it passes through
            // until released: the weld holds it where the ground would.
            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.05)
                .ignores_static(true);

            let body_handle = physics.world.create_body(body_desc);
            physics.world.attach_collider(body_handle, collider);

            let local_anchor = Vector3::new(0.0, -self.half_height, 0.0);
            let world_anchor = Point3::new(self.pos.0, surface_y - buried_depth, self.pos.1);

            let fixed_handle = physics.world.create_constraint(ConstraintKind::world_fixed(
                body_handle,
                world_anchor,
                local_anchor,
                &UnitQuaternion::identity(),
                0.0,
                f32::MAX,
            ));

            (body_handle, fixed_handle, fixed_handle)
        };

        let anchor_check = Point3::new(self.pos.0, surface_y - 0.1, self.pos.1);

        vec![world
            .create_entity()
            .with(Position(Vector3::new(
                initial_pos.x,
                initial_pos.y,
                initial_pos.z,
            )))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(TerrainAnchored {
                anchor_handle,
                upright_handle,
                anchor_points: vec![anchor_check],
                released_model: None,
            })
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Geometry — UV sphere shaped into an egg
// ---------------------------------------------------------------------------

/// Parameters defining the egg profile.
struct EggParams {
    half_height: f32,
    bottom_radius: f32,
    top_radius: f32,
}

impl EggParams {
    /// Horizontal radius at latitude `phi` (−π/2 = south pole, +π/2 = north).
    /// Linearly blends between `bottom_radius` and `top_radius` based on
    /// height, multiplied by the ellipse envelope `cos(phi)`.
    fn radius_at(&self, phi: f32) -> f32 {
        let t = (phi.sin() + 1.0) * 0.5; // 0 at south, 1 at north
        let r = self.bottom_radius + (self.top_radius - self.bottom_radius) * t;
        r * phi.cos()
    }

    /// Egg surface point at latitude `phi` and longitude `theta`.
    fn point(&self, phi: f32, theta: f32) -> Vector3<f32> {
        let r = self.radius_at(phi);
        Vector3::new(
            r * theta.cos(),
            self.half_height * phi.sin(),
            r * theta.sin(),
        )
    }
}

/// Build the egg's convex hull: `SEGMENTS` longitude and `RINGS` latitude
/// divisions.
fn build_egg_hull(egg: &EggParams) -> crate::collision::ConvexHull {
    let n = SEGMENTS;
    let r = RINGS;
    let mut vertices = Vec::with_capacity(r * n + 2);

    // South pole.
    vertices.push(Vector3::new(0.0, -egg.half_height, 0.0));

    for ri in 0..r {
        let phi = -FRAC_PI_2 + (ri as f32 + 1.0) / (r as f32 + 1.0) * std::f32::consts::PI;
        for si in 0..n {
            let theta = si as f32 * TAU / n as f32;
            vertices.push(egg.point(phi, theta));
        }
    }

    // North pole.
    let north = vertices.len();
    vertices.push(Vector3::new(0.0, egg.half_height, 0.0));

    let mut faces = Vec::new();

    // Vertex index helper: 0 = south pole, 1..=r*n = ring verts, north = last.
    let rv = |ri: usize, si: usize| -> usize { 1 + ri * n + si };

    // South fan triangles.
    for si in 0..n {
        let next = (si + 1) % n;
        faces.push(SolidFace {
            vertex_indices: vec![0, rv(0, next), rv(0, si)],
            opposite_vertex: north,
        });
    }

    // Side quads.
    for ri in 0..(r - 1) {
        for si in 0..n {
            let next = (si + 1) % n;
            faces.push(SolidFace {
                vertex_indices: vec![rv(ri, si), rv(ri, next), rv(ri + 1, next), rv(ri + 1, si)],
                opposite_vertex: rv(ri, (si + n / 2) % n),
            });
        }
    }

    // North fan triangles.
    let last = r - 1;
    for si in 0..n {
        let next = (si + 1) % n;
        faces.push(SolidFace {
            vertex_indices: vec![rv(last, si), rv(last, next), north],
            opposite_vertex: 0,
        });
    }

    build_convex_hull(&vertices, &faces)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::spawnables::stone::inside_out;
    use crate::rendering::vertex::Vertex;

    fn drawing(egg: &EggParams) -> Vec<(Vec<Vertex>, Vec<u32>)> {
        let hull = build_egg_hull(egg);
        let thickness = 2.0 * egg.bottom_radius.min(egg.top_radius);
        let model = weathered_model(
            &hull,
            Vector3::new(15.0, 1.2, 0.0),
            StoneTexture::WEATHERED.uvs(thickness),
            MaterialId(0),
        );
        model
            .parts
            .iter()
            .flat_map(|part| part.primitives.iter())
            .map(|p| (p.vertices.clone(), p.indices.clone()))
            .collect()
    }

    /// The egg's facets meet at shallow angles, many to a vertex, unlike the
    /// arch's blocks; its weathered drawing must still have no fold in it.
    #[test]
    fn a_menhir_is_drawn_right_side_out() {
        for egg in [
            EggParams {
                half_height: MenhirDef::default_half_height(),
                bottom_radius: MenhirDef::default_bottom_radius(),
                top_radius: MenhirDef::default_top_radius(),
            },
            EggParams {
                half_height: 2.0,
                bottom_radius: 0.8,
                top_radius: 0.5,
            },
        ] {
            for (vertices, indices) in drawing(&egg) {
                assert_eq!(inside_out(&vertices, &indices), 0);
            }
        }
    }
}
