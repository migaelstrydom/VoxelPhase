//! The hover sled spawnable: the hull, its four pods and their field.
//!
//! ```text
//!   HoverSledDef ──spawn──▶ hull body (bevelled box, plastic)
//!                              ├─ 4 × suspension, along the hull's up axis
//!                              ├─ HoverSled ─────────▶ HoverSledSystem (lean → thrust)
//!                              ├─ ForceFieldVisual ──▶ ForceFieldSystem (readout → look)
//!                              └─ PointLight (the underglow)
//! ```
//!
//! Each pod is a suspension anchored at its corner and pushing along the
//! hull's own up, so the sled tips with its load, pitches as it speeds up
//! and rocks back. The pods barely grip: it is a field, not a foot, and the
//! sled skates until its rider or the ground's light drag brings it to rest.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector2, Vector3};
use serde::Deserialize;
use smallvec::SmallVec;
use specs::{Builder, Entity, World, WorldExt};

use super::shape::{pod_anchors, sled_parts, HULL_HALF_EXTENTS};
use crate::app::spawnables::shared::orientation::Yaw;
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::collision::convex_hull::cube_hull;
use crate::collision::hull_bevel::bevel_hull;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::force_field::{FieldStyle, ForceFieldVisual};
use crate::lighting::PointLight;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{
    ColliderDesc, ColliderMaterial, ConstraintHandle, FrictionModel, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle, Spring, SuspensionAxis, SuspensionDesc,
};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::sled::{Glide, HoverSled};
use crate::systems::PhysicsResource;

/// The hull's mass, kg: about the player's, so a rider visibly presses it
/// down and a grenade throws it.
const HULL_MASS: f32 = 60.0;
/// How far the collider's edges are bevelled, m: enough that a plank's edge
/// rides it up rather than stopping it.
const HULL_BEVEL: f32 = 0.05;
/// Radius of each pod's cast sphere, m. A pod over a plank deck's narrower
/// gaps rides their edges, and over the wider ones it finds nothing; the
/// field shows which. Smaller, a pod dropped into every gap, the nose dived
/// and the next plank's edge, pushing back along its normal, stopped the
/// sled dead.
const POD_CAST_RADIUS: f32 = 0.2;
/// Each pod's rest length, from its face to its cast sphere's centre, m.
const POD_REST_LENGTH: f32 = 0.3;
/// How much further a pod reaches and still holds the sled, m.
const POD_SLACK: f32 = 0.15;
/// Each pod's spring, against the hull's mass. Under-damped, so the sled
/// bobs when it lands and settles within a second or two; stiff enough in
/// pitch that a pair of pods over a gap only dips the nose.
const POD_SPRING: Spring = Spring {
    angular_frequency: 2.4 * std::f32::consts::TAU,
    damping_ratio: 0.45,
};
/// The most each pod pushes with, as a multiple of the hull's weight: a hard
/// landing bottoms it out, onto the hull.
const POD_MAX_FORCE: f32 = 6.0;
/// The pods' grip, which meets the ground's as a collider's would: a light
/// drag, so the sled skates. It stops a shove within a metre or so, holds on
/// a gentle slope, and the rider's lean easily thrusts through it.
const POD_FRICTION: f32 = 0.01;
/// How quickly the sled stops spinning, 1/s. Its pods have no torsional
/// grip, so without it a sled set spinning spins on.
const ANGULAR_DAMPING: f32 = 0.8;
/// Gravity the pods are tuned against, m/s².
const GRAVITY: f32 = 9.81;

#[derive(Deserialize)]
pub struct HoverSledDef {
    /// Ground point under the sled's middle: it spawns hovering over it.
    pub pos: (f32, f32, f32),
    /// Rotation about `+Y`, in degrees: 0 points its nose along `+X`.
    #[serde(default)]
    pub yaw: f32,
}

impl HoverSledDef {
    /// Radius of each pod's cast sphere, and its rest length.
    pub const POD_CAST_RADIUS: f32 = POD_CAST_RADIUS;
    pub const POD_REST_LENGTH: f32 = POD_REST_LENGTH;

    /// Height of the hull's middle over the ground it hovers on, at rest:
    /// the pods' faces sit their rest length and cast radius up, less what
    /// the hull's weight presses them in.
    pub fn ride_height() -> f32 {
        let sag = GRAVITY / (pod_anchors().len() as f32 * POD_SPRING.angular_frequency.powi(2));
        let face = pod_anchors()[0].y;
        POD_CAST_RADIUS + POD_REST_LENGTH - sag - face
    }
}

impl Spawnable for HoverSledDef {
    fn material_count(&self) -> usize {
        sled_parts().len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // The colour is in the vertices, over the fallback white.
        Ok(sled_parts()
            .into_iter()
            .map(|part| {
                ctx.materials
                    .register(part.substance.shade(Material::coloured(Colour::WHITE)))
            })
            .collect())
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let yaw = Yaw::degrees(self.yaw);
        let ground = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let centre = ground + Vector3::y() * Self::ride_height();

        let primitives = sled_parts()
            .into_iter()
            .zip(materials)
            .map(|(part, &material)| MeshPrimitive {
                vertices: part.vertices,
                indices: part.indices,
                material,
            })
            .collect();
        let model = Arc::new(Model::flat(vec![ModelPart::new(primitives)]));

        let (hull, pods) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let built = build(&mut physics.world, centre, yaw.rotation());
            // A body starts asleep, and a sleeping suspension is never cast:
            // woken, the sled finds its ground on the first frame and shows
            // its field, then settles and sleeps on it.
            physics.world.wake_body(built.0);
            built
        };

        let deck = Vector2::new(HULL_HALF_EXTENTS.x, HULL_HALF_EXTENTS.z) * 0.85;
        vec![world
            .create_entity()
            .with(Position(centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(yaw.rotation()))
            .with(RigidBodyComponent(hull))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(HoverSled::new(deck, HULL_HALF_EXTENTS.y, Glide::HOVER))
            .with(ForceFieldVisual::new(FieldStyle::HOVER, pods))
            .with(PointLight::new(
                Colour::rgb(0.2, 0.78, 1.0),
                0.0,
                FieldStyle::HOVER.light_range,
            ))
            .build()]
    }
}

/// The sled's body and its four pods, at `centre` turned by `rotation`.
pub(super) fn build(
    physics: &mut PhysicsWorld,
    centre: Point3<f32>,
    rotation: UnitQuaternion<f32>,
) -> (RigidBodyHandle, Vec<ConstraintHandle>) {
    let hull = physics.create_body(
        RigidBodyDesc::dynamic()
            .position(centre)
            .rotation(rotation)
            .angular_damping(ANGULAR_DAMPING),
    );
    let collider = bevel_hull(&cube_hull(HULL_HALF_EXTENTS), HULL_BEVEL);
    let volume = 8.0 * HULL_HALF_EXTENTS.x * HULL_HALF_EXTENTS.y * HULL_HALF_EXTENTS.z;
    let _ = physics.attach_collider(
        hull,
        ColliderDesc::convex_hull(Arc::new(collider))
            .density(HULL_MASS / volume)
            .restitution(0.2)
            .friction(0.4),
    );
    let weight = physics.body(hull).map_or(0.0, |b| b.mass()) * GRAVITY;
    let pods = pod_anchors()
        .into_iter()
        .map(|anchor| {
            physics.create_suspension(SuspensionDesc {
                body: hull,
                anchor,
                axis: SuspensionAxis::Body(UnitVector3::new_normalize(Vector3::y())),
                cast_radius: POD_CAST_RADIUS,
                rest_length: POD_REST_LENGTH,
                slack: POD_SLACK,
                spring: POD_SPRING,
                max_force: POD_MAX_FORCE * weight,
                exclude: SmallVec::new(),
                friction: Some(ColliderMaterial {
                    restitution: 0.0,
                    friction: FrictionModel::isotropic(POD_FRICTION),
                }),
                torsion_radius: 0.0,
            })
        })
        .collect();
    (hull, pods)
}
