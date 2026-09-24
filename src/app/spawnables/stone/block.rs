//! One block of old stone, ready to be put into the world: a plain collider,
//! a weathered drawing, and the machinery to crack it in two.
//!
//! ```text
//!   StoneBlock ──▶ body + collider (hull or box, as authored)
//!       │
//!       ├──▶ ModelInstance   weathered_hull_mesh / weathered_box_mesh
//!       ├──▶ CompoundFracture  (a compound of one, keeping the whole shape
//!       │                       so a piece can tell old surface from break)
//!       └──▶ BrittleSolid      one cut, when the block is stopped as hard
//!                              as a long fall stops it
//! ```

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use super::mesh::{weathered_box_mesh, weathered_hull_mesh};
use crate::app::spawnables::shared::models::{
    assemble_by_material, piece_model, PieceHull, PiecePlacement, PlacedMesh, SurfaceUvs,
};
use crate::cleave::{BlowMeasure, BrittleSolid, CleaveRule};
use crate::collision::convex_hull::{cube_hull, ConvexHull};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::fracture::CompoundFracture;
use crate::model::Model;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::substance::{ColliderSubstance, Substance};
use crate::systems::PhysicsResource;

/// How fast a block has to be going when it is stopped to crack, in m/s.
///
/// Judged by [`BlowMeasure::Arrest`] — how hard the block itself is stopped —
/// not by the contact load it carries: a voussoir is squeezed by the thrust of
/// the whole arch, and the swing in that load when the arch shifts is many
/// times what a landing delivers. The test arena's arch cracked its crown
/// stones in place under a contact-load rule.
///
/// The threshold is set per block as its mass times this, so a stone's
/// strength is a speed and not an impulse: a five-metre voussoir and a
/// half-metre one crack from the same drop. About a metre of free fall — a
/// block tipped off a low wall survives. Lower than a drop from the crown
/// would suggest, because a collapsing arch does not drop its stones: they
/// tumble and slide down each other, and are stopped at 2–6 m/s. At 5.5 one
/// stone of twelve cracked when the default arch fell; at this, several do.
/// Well above the ~1.3 m/s a grenade shoves a small voussoir with, which is
/// the point: stone is knocked about by a blast and broken by the landing.
/// Not play-tested.
const CRACK_SPEED: f32 = 4.5;

/// How far a crack through stone tilts off square to the block, in degrees.
/// Stone breaks straighter than ice, but a square break reads as sawn.
const CRACK_TILT: f32 = 18.0;

/// Smallest piece of stone worth a body of its own, in m³. A block that would
/// crack into less is knocked about whole.
const MIN_PIECE_VOLUME: f32 = 0.01;

/// What a block's collider is.
pub enum StoneShape {
    /// A dressed block of any convex shape, in the block's own frame, centred
    /// on its centre of volume.
    Hull(Arc<ConvexHull>),
    /// A rectangular block, by half-extents.
    Box(Vector3<f32>),
}

impl StoneShape {
    fn hull(&self) -> ConvexHull {
        match self {
            Self::Hull(hull) => (**hull).clone(),
            Self::Box(half_extents) => cube_hull(*half_extents),
        }
    }

    fn collider(&self) -> ColliderDesc {
        match self {
            Self::Hull(hull) => ColliderDesc::convex_hull(hull.clone()),
            Self::Box(half_extents) => ColliderDesc::box_shape(*half_extents),
        }
    }
}

/// One block of weathered stone.
pub struct StoneBlock {
    /// Centre of the block, in world space.
    pub centre: Point3<f32>,
    /// How the block is turned in the world. Its shape is in its own frame.
    pub rotation: UnitQuaternion<f32>,
    pub shape: StoneShape,
    /// What it is made of: the collider's coefficients and the look.
    pub substance: Substance,
    /// Which of the caller's materials it wears.
    pub material: MaterialId,
    /// How its texture is laid on. Per-metre, so a piece that breaks off
    /// keeps the markings it had.
    pub uvs: SurfaceUvs,
}

impl StoneBlock {
    /// Create the body, the collider and the entity.
    pub fn spawn(&self, world: &mut World) -> Entity {
        let hull = self.shape.hull();
        let mass = self.substance.physics.density * hull.compute_volume();

        // The block's markings are read where it stands in the world, so that
        // no two blocks wear the same ones and neighbours read as cut from one
        // bed of stone.
        let anchor = self.centre.coords;
        let whole = Arc::new(hull.translated(anchor));
        let model = match &self.shape {
            StoneShape::Hull(hull) => weathered_model(hull, anchor, self.uvs, self.material),
            StoneShape::Box(half_extents) => piece_model(
                &PiecePlacement::new(*half_extents, anchor),
                weathered_box_mesh,
                self.uvs,
                self.material,
            ),
        };

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(self.centre)
                    .rotation(self.rotation)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            physics
                .world
                .attach_collider(body_handle, self.shape.collider().of(&self.substance));
            body_handle
        };

        // A compound of one block, as a lone ice block is: that is all a
        // block needs to be able to crack. Deliberately not
        // `shedding_debris` — the halves of a voussoir are the size of
        // boulders, and one fading out of existence reads as a bug.
        world
            .create_entity()
            .with(Position(self.centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(self.rotation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(
                CompoundFracture::boxes(Vec::new(), 1, self.material)
                    .with_piece_mesh(weathered_box_mesh)
                    .with_hull_mesh(weathered_hull_mesh)
                    .with_uvs(self.uvs)
                    .with_texture_anchor(anchor)
                    .with_whole_shape(whole),
            )
            .with(
                BrittleSolid::new(stone_cleaving(mass), self.material, 1)
                    .measured_by(BlowMeasure::Arrest),
            )
            .build()
    }
}

/// A whole stone of convex `hull`, in its own frame, drawn weathered.
///
/// `anchor` is where its markings are read from — where it stands in the world
/// — so that no two stones wear the same ones.
pub fn weathered_model(
    hull: &ConvexHull,
    anchor: Vector3<f32>,
    uvs: SurfaceUvs,
    material: MaterialId,
) -> Arc<Model> {
    let whole = hull.translated(anchor);
    let (vertices, indices) =
        weathered_hull_mesh(&PieceHull::new(hull, anchor).within(Some(&whole)), uvs);
    assemble_by_material(vec![PlacedMesh {
        vertices,
        indices,
        offset: Vector3::zeros(),
        rotation: UnitQuaternion::identity(),
        material,
    }])
}

/// How a stone block of `mass` kilograms cracks: once, clean through, and
/// only when it lands hard. See [`CRACK_SPEED`].
pub fn stone_cleaving(mass: f32) -> CleaveRule {
    CleaveRule {
        threshold: mass * CRACK_SPEED,
        pieces: 2,
        tilt: CRACK_TILT,
        min_volume: MIN_PIECE_VOLUME,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cleave::SolidCleaveSystem;
    use crate::debug::{DebugLines, DebugLog};
    use crate::fracture::FractureSystem;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{ColliderShape, PhysicsConfig, PhysicsImpulseQueue, PhysicsWorld};
    use crate::rendering::substance;
    use crate::time::Time;
    use specs::{Join, RunNow};

    const FRAME_DT: f32 = 1.0 / 60.0;

    /// A voussoir of the default arch, near enough: half a metre thick,
    /// 1.2 m deep, 0.6 m along the ring.
    const VOUSSOIR: Vector3<f32> = Vector3::new(0.3, 0.25, 0.6);

    /// One stone block, its lowest face `drop` metres above a floor.
    fn dropped_stone(drop: f32) -> World {
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Orientation>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CompoundFracture>();
        world.register::<BrittleSolid>();
        world.insert(Time::fixed(FRAME_DT));
        world.insert(PhysicsImpulseQueue::default());
        world.insert(DebugLog::default());

        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(config),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        StoneBlock {
            centre: Point3::new(0.0, VOUSSOIR.y + drop, 0.0),
            rotation: UnitQuaternion::identity(),
            shape: StoneShape::Box(VOUSSOIR),
            substance: substance::LIMESTONE,
            material: MaterialId(0),
            uvs: SurfaceUvs::PerMetre(0.5),
        }
        .spawn(&mut world);
        world
    }

    /// Run the game's break pipeline for `frames` frames and report how many
    /// pieces of stone there are, and whether every one is drawn.
    fn settle(world: &mut World, frames: usize) -> (usize, bool) {
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        for _ in 0..frames {
            {
                let mut physics = world.write_resource::<PhysicsResource>();
                let mut debug = DebugLines::default();
                stepper.step(
                    &mut physics.world,
                    FRAME_DT,
                    &geometry,
                    &[],
                    &[],
                    &mut debug,
                );
            }
            SolidCleaveSystem.run_now(world);
            FractureSystem.run_now(world);
            world.maintain();
        }
        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        let models = world.read_storage::<ModelInstance>();
        let mut pieces = 0;
        let mut all_drawn = true;
        for (entity, body) in (&world.entities(), &bodies).join() {
            pieces += physics
                .world
                .body(body.0)
                .map_or(0, |b| b.colliders().len());
            all_drawn &= models.get(entity).is_some_and(|m| {
                m.model
                    .parts
                    .iter()
                    .flat_map(|part| &part.primitives)
                    .any(|primitive| !primitive.indices.is_empty())
            });
        }
        (pieces, all_drawn)
    }

    /// A stone tipped off something low is knocked about, not broken.
    #[test]
    fn a_short_drop_leaves_a_stone_whole() {
        let mut world = dropped_stone(0.4);
        let (pieces, _) = settle(&mut world, 120);
        assert_eq!(pieces, 1);
    }

    /// A stone that falls from the crown of an arch cracks — once.
    #[test]
    fn a_long_fall_cracks_a_stone_in_two() {
        let mut world = dropped_stone(4.0);
        let (pieces, all_drawn) = settle(&mut world, 150);
        assert_eq!(pieces, 2, "a fallen stone should lie in two pieces");
        assert!(all_drawn, "a piece of the stone has no drawing");
    }

    /// One fracture plane: a cracked block is two pieces, never a shower.
    #[test]
    fn a_block_cracks_in_two() {
        let rule = stone_cleaving(700.0);
        let shape = ColliderShape::Box {
            half_extents: Vector3::new(0.25, 0.6, 0.3),
        };
        let pieces = rule
            .cleave(&shape, Vector3::new(0.0, 0.5, 0.0), 3)
            .expect("a block this size cracks");
        assert_eq!(pieces.len(), 2);
    }

    /// Strength is a speed, so a heavier block needs a bigger blow.
    #[test]
    fn a_heavier_block_takes_a_bigger_blow() {
        assert!(stone_cleaving(84_000.0).threshold > stone_cleaving(700.0).threshold * 100.0);
    }

    /// A grenade delivers about 1100 N·s at its centre. It must be able to
    /// knock a voussoir about without cracking it; only the landing does that.
    #[test]
    fn a_grenade_does_not_crack_a_voussoir() {
        const GRENADE_PEAK: f32 = 1100.0;
        let voussoir_mass = 2000.0 * 0.5 * 0.6 * 1.2;
        assert!(stone_cleaving(voussoir_mass).threshold > GRENADE_PEAK * 2.0);
    }
}
