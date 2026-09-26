//! Rock spawnable — a small field stone bedded in the terrain.
//!
//! ```text
//!   seed ──▶ RockCarving::carve ──▶ hull ─┬─▶ weathered drawing (old stone)
//!                                         ├─▶ released collider: the whole rock
//!                                         └─▶ trimmed at the ground ──▶ anchored collider
//!   ground under the footprint ──▶ bedding depth ──▶ Fixed to the world
//!                              └──▶ anchor points: loose once any is exposed
//! ```
//!
//! Every rock is a different stone cut from the same recipe: its shape, its
//! yaw and a slight lean all follow from its seed, which defaults to one taken
//! from where it stands. Its lower part is sunk into the ground and it is
//! welded there, like the menhir and the fence post, until the terrain under
//! it is blown away; then it is a loose stone.
//!
//! While bedded its collider is only the part above ground, so it never
//! presses into the terrain it is fixed to. On release the whole stone takes
//! over.

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::carving::{thickness, RockCarving};
use crate::app::spawnables::shared::textures::{seed_from_ground, TextureRng};
use crate::app::spawnables::stone::{weathered_model, StoneTexture};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::collision::convex_hull::ConvexHull;
use crate::collision::hull_split::{HullDraft, Plane};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Most a rock leans off upright, in degrees. Enough that a row of them does
/// not all sit square to the sky.
const MAX_LEAN: f32 = 20.0;

/// Share of its footprint's reach at which the ground under a rock is
/// sampled, besides its centre. Inside the edge, where the stone is thick
/// enough that ground a little lower would show under it.
const FOOTPRINT_SAMPLE: f32 = 0.7;

/// How far below the ground the terrain is checked, as a share of how deep
/// the rock is bedded. Once a blast has taken the ground down past this
/// anywhere under the rock, it is loose.
const RELEASE_DEPTH: f32 = 0.5;

/// How far the ground cut is raised each time a plane is refused, as a share
/// of the rock's size, and how many times it is tried.
const GROUND_CUT_STEP: f32 = 0.025;
const GROUND_CUT_TRIES: usize = 4;

#[derive(Deserialize)]
pub struct RockDef {
    /// Position (x, z). Y is determined by terrain surface height.
    pub pos: (f32, f32),
    /// Length of the rock's longest side, in metres.
    #[serde(default = "RockDef::default_size")]
    pub size: f32,
    /// Which rock: shape, yaw and lean all follow from it. Defaults to one
    /// taken from `pos`, so every rock in a level differs and stays the same
    /// from one load to the next.
    #[serde(default)]
    pub seed: Option<u32>,
    /// Share of the rock's height sunk into the ground.
    #[serde(default = "RockDef::default_bury")]
    pub bury: f32,
    /// Stone density (kg/m^3).
    #[serde(default = "RockDef::default_density")]
    pub density: f32,
}

impl RockDef {
    pub fn default_size() -> f32 {
        0.4
    }
    pub fn default_bury() -> f32 {
        0.3
    }
    pub fn default_density() -> f32 {
        substance::SARSEN.physics.density
    }

    /// The same weathered stone as the menhir; only the density is per rock.
    fn substance(&self) -> Substance {
        substance::SARSEN.with_density(self.density)
    }

    fn seed(&self) -> u32 {
        self.seed.unwrap_or_else(|| seed_from_ground(self.pos, 0))
    }

    /// How the rock sits: a turn about the vertical and a lean off it, both
    /// drawn from the rock's seed after its shape.
    fn pose(&self) -> UnitQuaternion<f32> {
        let mut rng = TextureRng::new(self.seed().rotate_left(16));
        let yaw = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), rng.range(0.0, TAU));
        let lean_axis = rng.range(0.0, TAU);
        let lean = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(Vector3::new(lean_axis.cos(), 0.0, lean_axis.sin())),
            rng.range(0.0, MAX_LEAN.to_radians()),
        );
        lean * yaw
    }
}

impl Spawnable for RockDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // One texture for every rock: each is textured from where it stands,
        // so each reads its own part of it, and a rockery of thirty stones
        // bakes one texture rather than thirty.
        Ok(vec![
            StoneTexture::WEATHERED.material(ctx, &self.substance())?
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let hull = RockCarving::GARDEN.carve(self.size, self.seed());
        let pose = self.pose();

        let posed: Vec<Vector3<f32>> = hull.vertices.iter().map(|v| pose * v).collect();
        let lowest = posed.iter().map(|v| v.y).fold(f32::INFINITY, f32::min);
        let highest = posed.iter().map(|v| v.y).fold(f32::NEG_INFINITY, f32::max);
        let reach = posed.iter().map(|v| v.xz().norm()).fold(0.0_f32, f32::max);

        let Some(ground) = Ground::under(
            &world.read_resource::<TerrainWorld>(),
            self.pos,
            reach * FOOTPRINT_SAMPLE,
        ) else {
            return Vec::new();
        };

        // Bedded from the lowest ground under it, so no edge stands clear of
        // a slope.
        let bedding = (highest - lowest) * self.bury;
        let centre = Point3::new(self.pos.0, ground.lowest() - bedding - lowest, self.pos.1);

        let model = weathered_model(
            &hull,
            centre.coords,
            StoneTexture::WEATHERED.uvs(thickness(&hull)),
            materials[0],
        );

        // Trimmed at the highest ground under it, so the bedded collider is
        // clear of the terrain everywhere.
        let anchored_collider = above_ground(&hull, pose, ground.highest() - centre.y, self.size)
            .map(|(exposed, offset)| {
                ColliderDesc::convex_hull(Arc::new(exposed))
                    .of(&self.substance())
                    .offset_translation(offset)
            })
            .unwrap_or_else(|| {
                ColliderDesc::convex_hull(Arc::new(hull.clone())).of(&self.substance())
            });
        let released_collider = ColliderDesc::convex_hull(Arc::new(hull)).of(&self.substance());

        let (body_handle, anchor_handle) = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(centre)
                    .rotation(pose)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.05),
            );
            physics
                .world
                .attach_collider(body_handle, anchored_collider);

            let anchor_handle = physics.world.create_constraint(ConstraintKind::world_fixed(
                body_handle,
                centre,
                Vector3::zeros(),
                0.0,
                f32::MAX,
            ));
            (body_handle, anchor_handle)
        };

        // Checked under the centre and round the footprint, each a little
        // below the surface there: the rock is loose once a blast has taken
        // the ground down under any part of it.
        let anchor_points = ground
            .samples
            .iter()
            .map(|p| p - Vector3::y() * bedding * RELEASE_DEPTH)
            .collect();

        vec![world
            .create_entity()
            .with(Position(centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(pose))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(TerrainAnchored {
                anchor_handle,
                upright_handle: anchor_handle,
                anchor_points,
                released_collider: Some(released_collider),
                released_model: None,
            })
            .build()]
    }
}

/// The terrain under a rock: its surface at the centre and at four points
/// round the footprint.
struct Ground {
    /// Points on the surface, the centre first.
    samples: Vec<Point3<f32>>,
}

impl Ground {
    /// Sampled at the centre and at four points `radius` out from it. `None`
    /// if there is no ground at the centre.
    fn under(terrain: &TerrainWorld, pos: (f32, f32), radius: f32) -> Option<Self> {
        let centre = terrain.mesh_surface_height_at(pos.0, pos.1)?;
        let mut samples = vec![Point3::new(pos.0, centre, pos.1)];
        for (dx, dz) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
            let (x, z) = (pos.0 + dx * radius, pos.1 + dz * radius);
            if let Some(y) = terrain.mesh_surface_height_at(x, z) {
                samples.push(Point3::new(x, y, z));
            }
        }
        Some(Self { samples })
    }

    fn lowest(&self) -> f32 {
        self.samples
            .iter()
            .map(|p| p.y)
            .fold(f32::INFINITY, f32::min)
    }

    fn highest(&self) -> f32 {
        self.samples
            .iter()
            .map(|p| p.y)
            .fold(f32::NEG_INFINITY, f32::max)
    }
}

/// The part of `hull`, posed by `pose`, above the level `ground` (relative to
/// the hull's centre), in the hull's own frame: centred on its own centre of
/// volume, with that centre's offset to attach it at. `None` if no cut near
/// the ground leaves something the physics engine can hold.
///
/// Cut at unit size and scaled back: the draft's sliver checks are absolute,
/// and a facet of a small rock is smaller than they allow. A plane that
/// passes too near a corner is refused; it is raised a little and tried
/// again, since raising it keeps the collider clear of the terrain.
fn above_ground(
    hull: &ConvexHull,
    pose: UnitQuaternion<f32>,
    ground: f32,
    size: f32,
) -> Option<(ConvexHull, Vector3<f32>)> {
    let inverse = pose.inverse();
    let unit = HullDraft::of(&hull.scaled(1.0 / size));
    let exposed = (0..GROUND_CUT_TRIES).find_map(|attempt| {
        let level = ground / size + attempt as f32 * GROUND_CUT_STEP;
        // The plane's front is the side cut away: below the ground.
        unit.trim(Plane::through(
            inverse * Vector3::new(0.0, level, 0.0),
            inverse * -Vector3::y(),
        ))
    })?;
    let exposed = exposed.build().scaled(size);
    let offset = exposed.centroid();
    Some((exposed.translated(-offset), offset))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::spawnables::stone::inside_out;

    /// A rock has more corners than a box, meeting at every angle the
    /// carving allows; none of them may fold the drawing.
    #[test]
    fn a_rock_is_drawn_right_side_out() {
        for seed in 0..40 {
            let hull = RockCarving::GARDEN.carve(RockDef::default_size(), seed);
            let model = weathered_model(
                &hull,
                Vector3::new(seed as f32 * 1.37, 0.3, seed as f32 * -2.1),
                StoneTexture::WEATHERED.uvs(thickness(&hull)),
                MaterialId(0),
            );
            let folds: usize = model
                .parts
                .iter()
                .flat_map(|part| part.primitives.iter())
                .map(|p| inside_out(&p.vertices, &p.indices))
                .sum();
            assert_eq!(folds, 0, "seed {seed}");
        }
    }

    /// The bedded collider stops at the ground, or a step above it where the
    /// plane was moved off a corner: nothing of it below, and all of the rock
    /// above the ground still in it.
    #[test]
    fn the_bedded_collider_is_the_rock_above_ground() {
        for seed in 0..50 {
            let def = RockDef {
                pos: (seed as f32, 0.0),
                size: 0.4,
                seed: Some(seed),
                bury: 0.3,
                density: RockDef::default_density(),
            };
            let hull = RockCarving::GARDEN.carve(def.size, seed);
            let pose = def.pose();
            let ground = -0.05;
            let (exposed, offset) = above_ground(&hull, pose, ground, def.size).expect("trimmed");

            let lowest = exposed
                .vertices
                .iter()
                .map(|v| (pose * (v + offset)).y)
                .fold(f32::INFINITY, f32::min);
            let raised = GROUND_CUT_STEP * def.size * (GROUND_CUT_TRIES - 1) as f32;
            assert!(
                lowest >= ground - 1e-4 && lowest <= ground + raised + 1e-4,
                "seed {seed}: {lowest}"
            );

            let highest = |h: &ConvexHull, by: Vector3<f32>| {
                h.vertices
                    .iter()
                    .map(|v| (pose * (v + by)).y)
                    .fold(f32::NEG_INFINITY, f32::max)
            };
            assert!(
                (highest(&exposed, offset) - highest(&hull, Vector3::zeros())).abs() < 1e-4,
                "seed {seed}"
            );
        }
    }
}
