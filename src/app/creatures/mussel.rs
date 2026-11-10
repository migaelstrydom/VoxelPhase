//! Mussel — a shell the size of a cart, lying open on the path with
//! something pink and patient inside it.
//!
//! The ambush. It does nothing until someone steps between its valves, and
//! then it shuts on them, hard, and holds shut a while before it opens
//! again. Seen, it is walked round or blasted; it is for the path taken at
//! a run. `visual_bench bestiary` shows it.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::palette::Paint;
use super::sculpted::{all_round, ground_under, sculpt_material, Exhibit};
use crate::app::sculpt::{painted_model, ContactShade, Ellipsoid, Mottle, SculptedPart, Warp};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::SnapTrap;
use crate::damage::Health;
use crate::model::Model;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;

/// Half the shell's width, height when shut, and length, m.
const HALF: (f32, f32, f32) = (0.75, 0.2, 0.6);
/// Kilograms to the cubic metre: shell, and it stays where it lies.
const DENSITY: f32 = 2600.0;
/// Distance from its middle within which it bites, m.
const BITE_RADIUS: f32 = 0.6;
/// Hit points a bite takes off, and seconds it stays shut after one.
const BITE: f32 = 30.0;
const HOLD: f32 = 2.5;
/// How far the upper valve stands open, radians.
const GAPE: f32 = 1.0;
/// Hit points it can take.
const HEALTH: f32 = 70.0;

const MOTTLE: Mottle = Mottle {
    frequency: 6.0,
    value: 0.25,
    warmth: -0.2,
    seed: 71,
};
const SHADE: ContactShade = ContactShade {
    reach: 0.06,
    depth: 0.6,
    floor: 0.3,
};

/// A mussel.
#[derive(Deserialize)]
pub struct MusselDef {
    /// Position (x, z). It lies on the ground there.
    pub pos: (f32, f32),
}

impl MusselDef {
    /// How it looks with its upper valve `gape` radians open, drawn in
    /// `material`, about the middle of its lower valve; its hinge is at
    /// the back (−Z).
    pub fn model(material: MaterialId, gape: f32) -> Model {
        let look = |r: f32, g: f32, b: f32| (Colour::new(r, g, b, 1.0), material);
        let shell = look(0.1, 0.13, 0.22);
        let pearl = look(0.82, 0.78, 0.86);
        let flesh = look(0.85, 0.5, 0.5);
        let lip = look(0.6, 0.2, 0.25);

        // A valve is a shallow bowl: the dark outside, and a pale lining
        // set a little inside it, narrowing to the hinge.
        let valve = |sign: f32, turn: f32| -> Vec<SculptedPart> {
            let hinge = Vector3::new(0.0, 0.0, -HALF.2);
            let swing = nalgebra::UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -turn);
            let place = |radii: (f32, f32, f32), drop: f32, colour| {
                let mut shape = Ellipsoid::new(radii, (0.0, 0.0, 0.0));
                shape.centre = hinge + swing * Vector3::new(0.0, sign * drop, HALF.2);
                shape.turn = swing;
                SculptedPart::new(shape, colour).warped(Warp::Taper {
                    along: -Vector3::z(),
                    tip: 0.45,
                })
            };
            vec![
                place((HALF.0, 0.5 * HALF.1, HALF.2), 0.5 * HALF.1, shell),
                place(
                    (0.93 * HALF.0, 0.3 * HALF.1, 0.93 * HALF.2),
                    0.3 * HALF.1,
                    pearl,
                ),
            ]
        };
        let mut parts = valve(-1.0, 0.0);
        parts.extend(valve(1.0, gape));
        // What lives in it: a lump of flesh with a frilled lip, on the
        // lower valve.
        parts.push(
            SculptedPart::new(
                Ellipsoid::new((0.45, 0.09, 0.36), (0.0, -0.02, -0.04)),
                flesh,
            )
            .warped(Warp::Lumps {
                frequency: 6.0,
                amplitude: 0.02,
                seed: 9,
            }),
        );
        parts.push(
            SculptedPart::new(Ellipsoid::new((0.56, 0.03, 0.44), (0.0, -0.03, -0.02)), lip).warped(
                Warp::Scallop {
                    lobes: 14,
                    bulge: 0.08,
                    // The frill stands up only as far as the valve over it
                    // has opened.
                    lift: 0.3 * (gape / GAPE).min(1.0),
                },
            ),
        );
        painted_model(&parts, &MOTTLE, &SHADE)
    }
}

impl MusselDef {
    /// The states it is shown in by tools.
    pub fn exhibits() -> Vec<Exhibit> {
        vec![
            Exhibit::at_rest(
                "open",
                Arc::new(Self::model(Paint::Shell.id(), GAPE)),
                0.05,
                1.5,
            ),
            Exhibit::at_rest(
                "shut",
                Arc::new(Self::model(Paint::Shell.id(), 0.0)),
                0.05,
                1.5,
            ),
        ]
    }
}

impl Spawnable for MusselDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![ctx.materials.register(sculpt_material())])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let Some(ground) = ground_under(world, self.pos) else {
            return Vec::new();
        };
        // What collides is the lower valve alone, low enough to step onto:
        // the upper one is only drawn, or nothing could walk into the trap.
        let low = Vector3::new(HALF.0, 0.25 * HALF.1, HALF.2);
        let at = Point3::new(self.pos.0, ground + low.y, self.pos.1);
        let body = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(at)
                    .linear_damping(0.3)
                    .angular_damping(0.9),
            );
            physics.world.attach_collider(
                body,
                ColliderDesc::box_shape(low)
                    .density(DENSITY)
                    .friction(1.5)
                    .restitution(0.0),
            );
            body
        };
        let open = Arc::new(Self::model(materials[0], GAPE));
        let shut = Arc::new(Self::model(materials[0], 0.0));
        vec![world
            .create_entity()
            .with(Position(at.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(open.clone()))
            .with(Renderable)
            .with(all_round(3.0))
            .with(SnapTrap {
                radius: BITE_RADIUS,
                damage: BITE,
                hold: HOLD,
                shut_for: 0.0,
                open,
                shut,
            })
            .with(Health::new(HEALTH, 4.0))
            .build()]
    }
}
