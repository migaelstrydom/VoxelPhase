//! The objective: the five Platonic solids and the brazier, on a lawn.
//!
//! The meshes and materials are the game's own (`PlatonicSolidDef::model`,
//! `BrazierDef::model`), so a shot here is what a level draws. The solids
//! hang as they hover, each held on a corner; the brazier is shown waiting
//! (its embers banked, a pale shaft of light over it) and lit (its coals
//! glowing, its shaft gold and its firelight on, but not the fire itself,
//! which is simulated only in the game), close to and from across a level.

use nalgebra::{Isometry3, Matrix4, Point3, Translation3, UnitQuaternion, Vector3};

use crate::app::spawnables::{BrazierDef, PlatonicSolidDef};
use crate::core::error::EngineResult;
use crate::geometry::PlatonicSolid;
use crate::light_shaft::ShaftMesh;
use crate::lighting::{ActiveLight, LightId};
use crate::model::Model;
use crate::objective::BrazierStage;
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceModulation};
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;

/// How high the solids hang, m, and how far apart.
const HANG: f32 = 1.1;
const SPACING: f32 = 1.1;

/// Where the brazier stands, behind the row of solids.
const BRAZIER_AT: Vector3<f32> = Vector3::new(0.0, 0.0, -2.2);

/// The brazier's firelight: where it is from the brazier's ground point,
/// how far it reaches and how bright it is (as the game's fire light).
const FIRELIGHT_AT: Vector3<f32> = Vector3::new(0.0, 1.2, 0.0);
const FIRELIGHT_RANGE: f32 = 7.8;
const FIRELIGHT: f32 = 1.4;

/// The embers' glow in a brazier that waits, as a share of a lit one's: a
/// moment of their breath.
const BANKED: f32 = 0.14;

pub struct Objective;

/// A model with the materials it is drawn in.
pub(super) struct Drawn {
    model: Model,
    materials: Vec<Material>,
}

impl Drawn {
    pub(super) fn new(
        materials: Vec<Material>,
        model: impl FnOnce(&[MaterialId]) -> Model,
    ) -> Self {
        let ids: Vec<MaterialId> = (0..materials.len() as u32).map(MaterialId).collect();
        Self {
            model: model(&ids),
            materials,
        }
    }

    /// Its meshes placed by `pose`, its glow scaled by `glow`.
    pub(super) fn meshes(&self, pose: Matrix4<f32>, glow: f32) -> Vec<SceneMesh> {
        self.model
            .parts
            .iter()
            .flat_map(|part| part.primitives.iter())
            .map(|primitive| {
                let material = &self.materials[primitive.material.0 as usize];
                let mesh = SceneMesh::new(primitive.vertices.clone(), primitive.indices.clone())
                    .with_transform(pose)
                    .with_surface(
                        material
                            .surface_params()
                            .modulated(SurfaceModulation::emissive(glow)),
                    );
                match &material.diffuse_texture {
                    Some(texture) => mesh.with_texture(texture.clone()),
                    None => mesh,
                }
            })
            .collect()
    }
}

impl VisualScene for Objective {
    fn name(&self) -> &str {
        "objective"
    }

    fn description(&self) -> &str {
        "The five Platonic solids hovering on a lawn, and the bronze brazier cold and lit."
    }

    fn shots(&self, _ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let solids: Vec<(PlatonicSolid, Drawn)> = PlatonicSolid::ALL
            .iter()
            .map(|&kind| {
                (
                    kind,
                    Drawn::new(PlatonicSolidDef::materials(kind), |ids| {
                        PlatonicSolidDef::model(kind, ids)
                    }),
                )
            })
            .collect();
        let brazier = Drawn::new(BrazierDef::materials(), BrazierDef::model);

        let solid_at = |index: usize| Vector3::new((index as f32 - 2.0) * SPACING, HANG, 0.0);
        let solid_meshes = || -> Vec<SceneMesh> {
            solids
                .iter()
                .enumerate()
                .flat_map(|(index, (kind, drawn))| {
                    let turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.5)
                        * PlatonicSolidDef::rest(*kind);
                    let pose = Isometry3::from_parts(Translation3::from(solid_at(index)), turn);
                    drawn.meshes(pose.to_homogeneous(), 1.0)
                })
                .collect()
        };
        let brazier_meshes = |lit: bool| {
            brazier.meshes(
                Isometry3::translation(BRAZIER_AT.x, BRAZIER_AT.y, BRAZIER_AT.z).to_homogeneous(),
                if lit { 1.0 } else { BANKED },
            )
        };
        // The shaft over it, as the brazier's own system aims it.
        let shaft = |lit: bool| {
            let stage = if lit {
                BrazierStage::Lit
            } else {
                BrazierStage::Ready
            };
            let shaft = BrazierDef::light_shaft(stage.shaft());
            ShaftMesh::of([(BRAZIER_AT + shaft.base, &shaft)])
        };

        // A low golden sun from ahead and to one side, as on the island.
        let day = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.5, 0.7))
            .with_sun_colour(Colour::new(1.0, 0.82, 0.6, 1.0));
        let firelit = day.clone().with_point_light(ActiveLight {
            id: LightId(0),
            position: BRAZIER_AT + FIRELIGHT_AT,
            range: FIRELIGHT_RANGE,
            colour: Colour::rgb(1.0, 0.65, 0.3),
            intensity: FIRELIGHT,
        });

        let shot = |label: &str,
                    eye: Point3<f32>,
                    target: Point3<f32>,
                    fov: f32,
                    lit: bool,
                    environment: &SceneEnvironment| {
            let mut meshes = solid_meshes();
            meshes.extend(brazier_meshes(lit));
            meshes.push(ground());
            SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(fov))
                .with_environment(environment.clone())
                .with_meshes(meshes)
                .with_light_shaft(shaft(lit))
        };

        let brazier_middle = Point3::from(BRAZIER_AT + Vector3::new(0.0, 0.6, 0.0));
        let mut shots = vec![
            shot(
                "together",
                Point3::new(1.5, 2.2, 6.5),
                Point3::new(0.0, 0.8, -0.8),
                42.0,
                false,
                &day,
            ),
            shot(
                "afar",
                Point3::new(18.0, 4.0, 22.0),
                Point3::new(0.0, 4.0, -2.0),
                50.0,
                false,
                &day,
            ),
            shot(
                "afar_lit",
                Point3::new(18.0, 4.0, 22.0),
                Point3::new(0.0, 4.0, -2.0),
                50.0,
                true,
                &firelit,
            ),
            shot(
                "solids",
                Point3::new(0.0, 1.5, 4.2),
                Point3::new(0.0, HANG, 0.0),
                40.0,
                false,
                &day,
            ),
        ];
        for index in 0..solids.len() {
            let at = Point3::from(solid_at(index));
            shots.push(shot(
                &format!("close_{:?}", solids[index].0).to_lowercase(),
                at + Vector3::new(0.5, 0.3, 1.3),
                at,
                35.0,
                false,
                &day,
            ));
        }
        shots.extend([
            shot(
                "brazier",
                Point3::from(BRAZIER_AT + Vector3::new(1.6, 1.7, 2.8)),
                brazier_middle,
                34.0,
                false,
                &day,
            ),
            shot(
                "brazier_lit",
                Point3::from(BRAZIER_AT + Vector3::new(-1.4, 1.9, 2.6)),
                brazier_middle,
                34.0,
                true,
                &firelit,
            ),
            shot(
                "paw",
                Point3::from(BRAZIER_AT + Vector3::new(0.4, 0.45, 1.3)),
                Point3::from(BRAZIER_AT + Vector3::new(0.0, 0.15, 0.47)),
                30.0,
                false,
                &day,
            ),
            shot(
                "into_the_bowl",
                Point3::from(BRAZIER_AT + Vector3::new(0.6, 2.0, 1.2)),
                Point3::from(BRAZIER_AT + Vector3::new(0.0, 1.0, 0.0)),
                32.0,
                true,
                &firelit,
            ),
        ]);
        Ok(shots)
    }
}
