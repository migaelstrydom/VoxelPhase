//! The ferry fish as a thing in a level: a powered platform that is a fish.
//!
//! ```text
//!   PoweredBody (deck hull + eye balls) ── MovingPlatform ── FacesTravel
//!     the same body, suspension and          its patrol        nose first
//!     motor every platform has
//!   PosedModel (body, wings, eyes) ─┐
//!   ProceduralMesh (fringe, tail) ──┴── FishRig: the wave, the wingbeat,
//!                                        the eyes' glances
//! ```

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World};

use super::super::likeness;
use super::super::palette::{self, Paint, BESTIARY};
use super::super::sculpted::Exhibit;
use super::form::{self, Deck, EYE_REACH};
use super::sculpt::{self, EYE_PARTS, LEFT_WING, RIGHT_WING};
use crate::animation::fish::{FinWave, FishRig, RippledFin, RovingEye};
use crate::app::spawnables::shared::finish::ColliderSurface;
use crate::app::spawnables::{MaterialCtx, PoweredBody, Spawnable};
use crate::components::{PoseFrame, PosedModel, ProceduralMesh, Renderable};
use crate::core::error::EngineResult;
use crate::creature::{Wing, WingBeat};
use crate::drive::Actuator;
use crate::model::{MeshPrimitive, Model, Ripple};
use crate::physics::ColliderDesc;
use crate::platform::{DeckSuspension, FacesTravel, MovingPlatform, RouteLoop, SeekMotion};
use crate::rendering::material::MaterialId;
use crate::rendering::physical_finish::PhysicalSurface;

/// The names its look is kept under: its model, and each fin.
const KIND: &str = "ferry_fish";
const FINS: [(&str, fn() -> Ripple, f32); 5] = [
    ("ferry_fish/dorsal", sculpt::dorsal, DORSAL_SWING),
    ("ferry_fish/anal", sculpt::anal, DORSAL_SWING),
    ("ferry_fish/brow_right", sculpt::brow_right, BROW_SWING),
    ("ferry_fish/brow_left", sculpt::brow_left, BROW_SWING),
    ("ferry_fish/tail", sculpt::tail, TAIL_SWING),
];

/// What it is made of: skin that holds a footing, and as heavy for its
/// size as a big fish is with the air in its bladder: about 330 kg, so a
/// player standing on its edge tips it and a grenade shoves it, but it
/// carries both.
const SURFACE: PhysicalSurface = PhysicalSurface {
    restitution: 0.1,
    friction: 0.9,
    density: 90.0,
};

/// Its motor: the most it changes its velocity by, m/s², which must clear
/// gravity with room to spare; and how hard it turns, rad/s².
const MOTOR_ACCEL: f32 = 40.0;
const TURN_ACCEL: f32 = 1.2;

/// How it turns to face its travel: no faster than a big fish swings round,
/// and not at all when it is hardly moving.
const TURNING: FacesTravel = FacesTravel {
    turn_rate: 0.55,
    stiffness: 1.6,
    moving: 0.4,
};

/// Its fins: how far the fringe, the short fringe round its head and the
/// tail swing at cruising speed, and the wave that runs down them.
const DORSAL_SWING: f32 = 0.3;
const BROW_SWING: f32 = 0.18;
const TAIL_SWING: f32 = 0.26;
const WAVE: FinWave = FinWave {
    wavelength: 1.5,
    idle_rate: 0.55,
    cruise_rate: 1.5,
    idle_swing: 0.45,
    curl: 0.7,
};

/// How its wings beat: slowly, the beat of something big.
const WING_RAISE: f32 = 0.42;
const WING_LOWER: f32 = 0.5;
const WING_FEATHER: f32 = 0.12;
const GLIDE_BEATS: f32 = 0.32;
const HARD_BEATS: f32 = 0.75;

/// How far its eyes turn from where they look at rest, radians; how much
/// of their glances each gives whoever is near; and how far off that is.
const EYE_TURN: f32 = 0.75;
const EYE_INTEREST: [f32; 2] = [0.75, 0.35];
const SIGHT: f32 = 30.0;

/// A ferry fish: a flatfish the size of a cart that swims through the air
/// along a patrol, with whoever stands on its back.
#[derive(Deserialize)]
pub struct FerryFishDef {
    /// The patrol, in world space: where its middle passes. It starts on
    /// the first waypoint, facing the second.
    pub waypoints: Vec<(f32, f32, f32)>,
    /// What happens at the end of the list. A fish swims round a circuit
    /// best: on a shuttle it turns about at either end, slowly.
    #[serde(default = "FerryFishDef::default_looping")]
    pub looping: RouteLoop,
    /// Cruise speed, m/s.
    #[serde(default = "FerryFishDef::default_speed")]
    pub speed: f32,
    /// Spin-up time, s, and with it how wide it corners: about
    /// `speed · spin_up`.
    #[serde(default = "FerryFishDef::default_spin_up")]
    pub spin_up: f32,
    /// How far its back tips under a player on its edge, and how fast the
    /// rocking that follows dies.
    #[serde(default = "FerryFishDef::default_suspension")]
    pub suspension: DeckSuspension,
}

impl FerryFishDef {
    pub fn default_looping() -> RouteLoop {
        RouteLoop::Circuit
    }

    pub fn default_speed() -> f32 {
        2.5
    }

    pub fn default_spin_up() -> f32 {
        1.2
    }

    /// A living back: it gives under a player more than a deck does, and
    /// rocks a little longer.
    pub fn default_suspension() -> DeckSuspension {
        DeckSuspension {
            tilt_degrees: 3.0,
            damping: 0.4,
            yaw_resistance: DeckSuspension::default_yaw_resistance(),
        }
    }

    /// Its model against the palette's slots.
    pub fn sculpt() -> Model {
        sculpt::model()
    }

    /// How its wings beat.
    fn wing_beat() -> WingBeat {
        let deck = Deck::get();
        let wing = |part: usize, side: f32| Wing {
            part,
            root: deck.placed(form::wing_root(side)),
            hinge: Vector3::z(),
            span: Vector3::new(side, 0.0, -0.35),
        };
        WingBeat {
            wings: [wing(RIGHT_WING, 1.0), wing(LEFT_WING, -1.0)],
            raise: WING_RAISE,
            lower: WING_LOWER,
            feather: WING_FEATHER,
            glide_rate: GLIDE_BEATS,
            hard_rate: HARD_BEATS,
            tuck_raise: 0.0,
            tuck_sweep: 0.0,
        }
    }

    /// Its rig, on `fins`, cruising at `speed`.
    fn rig(fins: Vec<Arc<Ripple>>, speed: f32) -> FishRig {
        let deck = Deck::get();
        let fins = fins
            .into_iter()
            .zip(FINS)
            .map(|(ripple, (_, _, swing))| RippledFin { ripple, swing })
            .collect();
        let eyes = EYE_PARTS
            .into_iter()
            .enumerate()
            .map(|(eye, part)| RovingEye {
                part,
                pivot: deck.placed(form::eye_centre(eye)),
                rest: form::eye_rest(eye),
                reach: EYE_TURN,
                interest: EYE_INTEREST[eye],
                seed: 261 + eye as u32,
            })
            .collect();
        FishRig::new(fins, WAVE, Some(Self::wing_beat()), eyes, speed, SIGHT)
    }

    /// The states tools show it in: gliding, its wings at the top of a
    /// beat and at the bottom, with its fins rippling as they do at speed.
    pub fn exhibits() -> Vec<Exhibit> {
        let model = likeness::model_sculpted(KIND, Self::sculpt);
        let fins = FINS
            .iter()
            .map(|(kind, sculpt, _)| likeness::ripple_sculpted(kind, *sculpt))
            .collect();
        let mut rig = Self::rig(fins, Self::default_speed());
        // A wingbeat at cruise is about 1.3 s: shown a quarter apart.
        let beat = 1.0 / HARD_BEATS;
        [("gliding", 0.3), ("wings_down", 0.5), ("wings_up", 0.5)]
            .into_iter()
            .map(|(name, share)| {
                rig.advance(beat * share, Self::default_speed(), None);
                let mut poses = vec![Default::default(); model.parts.len()];
                rig.pose(&mut poses);
                let mut shaped: Vec<MeshPrimitive> = Vec::new();
                rig.shape(&mut shaped);
                Exhibit {
                    name,
                    model: model.clone(),
                    poses,
                    shaped,
                    lift: 1.6,
                    reach: 2.3,
                }
            })
            .collect()
    }
}

impl Spawnable for FerryFishDef {
    fn material_count(&self) -> usize {
        Paint::ALL.len()
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(BESTIARY, palette::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let waypoints: Vec<Vector3<f32>> = self
            .waypoints
            .iter()
            .map(|&(x, y, z)| Vector3::new(x, y, z))
            .collect();
        let Some(&from) = waypoints.first() else {
            return Vec::new();
        };
        let ahead = waypoints.get(1).map_or(Vector3::z(), |next| next - from);
        let facing = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), ahead.x.atan2(ahead.z));

        let deck = Deck::get();
        let colliders = std::iter::once(
            ColliderDesc::convex_hull(deck.hull.clone()).with_physical_surface(SURFACE),
        )
        .chain(deck.eyes.iter().map(|&eye| {
            ColliderDesc::sphere(EYE_REACH)
                .offset_translation(eye)
                .with_physical_surface(SURFACE)
        }))
        .collect();
        let body = PoweredBody {
            at: Point3::from(from),
            facing,
            suspension: self.suspension,
            reach: deck.reach,
        };
        let platform = MovingPlatform::new(
            waypoints,
            self.looping,
            SeekMotion {
                speed: self.speed,
                tau: self.spin_up,
            },
        );

        let model = likeness::model(KIND, materials, Self::sculpt);
        let fins: Vec<Arc<Ripple>> = FINS
            .iter()
            .map(|(kind, sculpt, _)| likeness::ripple(kind, materials, *sculpt))
            .collect();
        let shaped = ProceduralMesh {
            primitives: fins.iter().map(|fin| fin.rest().clone()).collect(),
        };
        let rig = Self::rig(fins, self.speed);

        vec![body
            .spawn(
                world,
                colliders,
                platform,
                Actuator::medium(MOTOR_ACCEL, TURN_ACCEL),
            )
            .with(TURNING)
            .with(PosedModel::at_rest(model, PoseFrame::Entity))
            .with(shaped)
            .with(rig)
            .with(Renderable)
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn its_wings_are_hinged_where_their_rays_leave_the_body() {
        let deck = Deck::get();
        let beat = FerryFishDef::wing_beat();
        for (wing, side) in beat.wings.iter().zip([1.0, -1.0]) {
            for ray in sculpt::wing_rays(side) {
                let from_hinge = deck.placed(ray.root) - wing.root;
                let off = from_hinge - wing.hinge * from_hinge.dot(&wing.hinge);
                assert!(
                    off.norm() < 0.03,
                    "a ray rooted {} m off its hinge",
                    off.norm()
                );
            }
        }
    }

    #[test]
    fn it_weighs_about_a_third_of_a_tonne() {
        let deck = Deck::get();
        let volume =
            deck.hull.compute_volume() + 2.0 * 4.0 / 3.0 * std::f32::consts::PI * EYE_REACH.powi(3);
        let mass = volume * SURFACE.density;
        assert!((250.0..450.0).contains(&mass), "{mass} kg");
    }
}
