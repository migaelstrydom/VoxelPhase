//! A grown plant spawned into the world: built as one compound body
//! (`plants::Joinery`) welded into the ground, on one entity with its parts'
//! looks and its leaves; its branches break off as pieces of their own
//! (`fracture`), which `plants::PlantShareSystem` gives their leaves.
//!
//! Shared by every kind of plant's spawnable; each finds its ground, grows
//! its `Plant` and hands it here.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::seed_from_ground;
use super::spawnable::MaterialCtx;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::plants::{
    BarkSpec, GrownPlant, Joinery, LimbKind, Plant, PlantDrawing, PlantLook, PlantPiece,
};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::resources::textures::TextureHandle;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Leaves are matte, with a faint sheen off the tops of the sprigs.
const LEAF_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};
/// Bark is smooth and dull, the painted trees' trunks catching only a soft
/// sheen.
const BARK_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.75,
    metallic: 0.0,
};
/// How far below the ground at its foot the trunk checks for terrain: inside
/// the solid, not on its surface.
const ANCHOR_PROBE: f32 = 0.1;
/// How many materials a plant registers: its leaves, then its bark.
pub const PLANT_MATERIALS: usize = 2;

/// A plant's materials, leaves then bark, for one grown from `seed`: its bark
/// is a tile of its own, painted from that seed, with its relief in alpha.
pub fn plant_materials(ctx: &mut MaterialCtx, seed: u32) -> EngineResult<Vec<MaterialId>> {
    let spec = BarkSpec::default();
    let size = spec.texture_size;
    let tile = ctx
        .textures
        .create_from_rgba(size, size, &spec.texture(seed), true)?;
    Ok(vec![
        ctx.materials.register(leaf_material()),
        ctx.materials.register(bark_material(tile)),
    ])
}

/// What a plant's leaves are drawn in: white, since the greens ride on the
/// skin's vertices.
pub fn leaf_material() -> Material {
    Material::coloured(Colour::WHITE)
        .with_finish(LEAF_FINISH)
        .as_foliage()
}

/// What a plant's bark is drawn in, wearing `tile` (`BarkSpec::texture`).
pub fn bark_material(tile: TextureHandle) -> Material {
    Material::textured(tile)
        .with_finish(BARK_FINISH)
        .with_relief(BarkSpec::default().relief_depth)
}

/// The plant `grow` makes from the seed of where it stands, spawned on the
/// ground at `pos`, drawn in `materials` ([`plant_materials`]). Nothing if
/// there is no ground there.
pub fn spawn_on_ground(
    world: &mut World,
    pos: (f32, f32),
    materials: &[MaterialId],
    grow: impl FnOnce(u32) -> Plant,
) -> Vec<Entity> {
    let seed = seed_from_ground(pos, 1);
    let look = PlantLook {
        leaves: materials[0],
        bark: materials[1],
    };
    let mut plant = grow(seed);
    let ground = {
        let terrain = world.read_resource::<TerrainWorld>();
        let Some(ground) = terrain.mesh_surface_height_at(pos.0, pos.1) else {
            return Vec::new();
        };
        sink_roots(
            &mut plant,
            |x, z| terrain.mesh_surface_height_at(pos.0 + x, pos.1 + z),
            ground,
        );
        ground
    };
    let root = Isometry3::translation(pos.0, ground, pos.1);
    let anchor = Point3::new(pos.0, ground - ANCHOR_PROBE, pos.1);
    vec![spawn_plant(world, plant, root, seed, look, anchor)]
}

/// Lowers each root's tip by however far the ground at it lies below the
/// ground at the plant's foot, `foot`, so a root reaching downhill stays as
/// deep under its own ground as one on the level. `ground_at` is the
/// ground's height at an offset from the foot.
fn sink_roots(plant: &mut Plant, ground_at: impl Fn(f32, f32) -> Option<f32>, foot: f32) {
    for limb in &mut plant.limbs.limbs {
        if limb.kind != LimbKind::Root {
            continue;
        }
        if let Some(ground) = ground_at(limb.tip.x, limb.tip.z) {
            limb.tip.y += (ground - foot).min(0.0);
        }
    }
}

/// `plant` built as one compound body and spawned with its frame at
/// `root`, its skins grown from `seed`. The trunk is welded into the ground,
/// let go when the terrain at `anchor` is blown away.
pub fn spawn_plant(
    world: &mut World,
    plant: Plant,
    root: Isometry3<f32>,
    seed: u32,
    look: PlantLook,
    anchor: Point3<f32>,
) -> Entity {
    let joinery = Joinery::default();
    let drawing = PlantDrawing::grow(&plant, seed, look);
    let all: Vec<usize> = (0..plant.limbs.limbs.len()).collect();
    let mut physics = world.write_resource::<PhysicsResource>();
    let mut built = joinery.build(&mut physics.world, &plant, root);
    let rooting = joinery.root(&mut physics.world, &plant, &mut built);
    let grown = Arc::new(GrownPlant {
        plant,
        joinery,
        drawing,
        colliders: built.colliders.clone(),
    });
    let looks = grown.looks(&physics.world, &all, &built.frame);
    let model = looks
        .drawn(&physics.world, built.body)
        .expect("a plant has bark to draw");
    drop(physics);
    let pose = root * built.frame.inverse();
    let mut entity = world
        .create_entity()
        .with(Position(pose.translation.vector))
        .with(Orientation(pose.rotation))
        .with(Velocity(Vector3::zeros()))
        .with(RigidBodyComponent(built.body))
        .with(ModelInstance::new(model))
        .with(Renderable)
        .with(looks)
        .with(joinery.fracture(built.graph))
        // Let go when the ground under its foot is blown away.
        .with(TerrainAnchored {
            constraints: vec![rooting],
            anchor_points: vec![anchor],
            released_model: None,
        });
    if let Some(clump) = grown.foliage(&all, built.body, built.frame) {
        entity = entity.with(clump);
    }
    entity.with(PlantPiece { plant: grown }).build()
}

#[cfg(test)]
mod tests {
    use specs::Join;

    use super::*;
    use crate::app::breakage;
    use crate::debug::DebugLines;
    use crate::explosion::Explosion;
    use crate::foliage::Foliage;
    use crate::model::PartLooks;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{
        ConstraintKind, PhysicsImpulseQueue, RigidBodyHandle, SequentialStepper, Stepper,
    };
    use crate::plants::{LimbBase, LollipopSpec};
    use crate::time::Time;

    fn world() -> World {
        let mut world = World::new();
        breakage::register(&mut world);
        world.insert(Time::fixed(1.0 / 60.0));
        world.insert(PhysicsResource::default());
        world
    }

    /// `frames` frames of physics, each followed by the break pipeline.
    fn step(world: &mut World, frames: usize) {
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        let ground = FlatQuadGeometry::new(50.0);
        let mut lines = DebugLines::default();
        for _ in 0..frames {
            {
                let mut queue = world.write_resource::<PhysicsImpulseQueue>();
                let impulses: Vec<_> = queue.drain().collect();
                let mut physics = world.write_resource::<PhysicsResource>();
                stepper.step(
                    &mut physics.world,
                    1.0 / 60.0,
                    &ground,
                    &impulses,
                    &[],
                    &mut lines,
                );
            }
            lines.clear();
            breakage::run(world);
        }
    }

    /// A blast beside a forked tree's side crown breaks that branch off at
    /// its fork: it is a piece of its own, drawn snapped, with its crown's
    /// leaves, thrown away from the blast; the tree it left, still welded at
    /// its foot, keeps its trunk and main crown and stands.
    #[test]
    fn a_blast_breaks_a_branch_off_and_leaves_the_tree_standing() {
        let mut world = world();
        let plant = LollipopSpec {
            crowns: 3,
            ..LollipopSpec::default()
        }
        .grow(4);
        let side = Point3::from(plant.masses[1].pose.translation.vector);
        let branch = plant.masses[1].limb;
        let look = PlantLook {
            leaves: MaterialId(0),
            bark: MaterialId(1),
        };
        let foot = Point3::new(0.0, -0.3, 0.0);
        let tree = spawn_plant(&mut world, plant, Isometry3::identity(), 4, look, foot);
        let (body, weld, grown) = {
            let pieces = world.read_storage::<PlantPiece>();
            let bodies = world.read_storage::<RigidBodyComponent>();
            let anchors = world.read_storage::<TerrainAnchored>();
            (
                bodies.get(tree).unwrap().0,
                anchors.get(tree).unwrap().constraints[0],
                pieces.get(tree).unwrap().plant.clone(),
            )
        };
        let outward = Vector3::new(side.x, 0.0, side.z).normalize();
        world
            .write_resource::<PhysicsImpulseQueue>()
            .push(Explosion::new(side + outward * 1.2).physics_impulse());
        step(&mut world, 1);

        let physics = world.read_resource::<PhysicsResource>();
        let pieces = world.read_storage::<PlantPiece>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        let all: Vec<(Entity, RigidBodyHandle)> = (&world.entities(), &pieces, &bodies)
            .join()
            .map(|(e, _, b)| (e, b.0))
            .collect();
        assert!(all.len() > 1, "no branch broke off");
        let (broken, broken_body) = *all
            .iter()
            .find(|(_, b)| grown.limbs_on(&physics.world, *b).contains(&branch))
            .expect("the branch is somewhere");
        assert_ne!(broken, tree, "the branch stayed on the tree");
        assert_eq!(
            grown.limbs_on(&physics.world, broken_body),
            vec![branch],
            "the branch came off with other limbs"
        );
        let kept = grown.limbs_on(&physics.world, body);
        assert!(kept.contains(&0) && !kept.contains(&branch));

        let foliage = world.read_storage::<Foliage>();
        assert!(foliage.contains(broken) && foliage.contains(tree));
        let looks = world.read_storage::<PartLooks>();
        let drawn = looks.get(broken).unwrap().get(grown.colliders.wood[branch]);
        let limb = &grown.plant.limbs.limbs[branch];
        let snapped = grown.drawing.bark(branch, LimbBase::Snapped, &limb.pose());
        assert_eq!(
            drawn.map(|look| look.primitives[0].vertices.len()),
            Some(snapped.primitives[0].vertices.len()),
            "the branch is not drawn snapped"
        );
        let flying = physics.world.body(broken_body).unwrap();
        assert!(
            flying.linear_velocity().dot(&-outward) > 1.0,
            "the branch was not thrown away from the blast: {:?}",
            flying.linear_velocity()
        );
        let at = physics.world.body(body).unwrap().position();
        drop((physics, pieces, bodies, foliage, looks));

        step(&mut world, 120);
        let physics = world.read_resource::<PhysicsResource>();
        let standing = physics.world.body(body).unwrap();
        let ConstraintKind::Fixed { local_anchor_b, .. } =
            physics.world.constraint(weld).unwrap().kind
        else {
            unreachable!("the trunk's weld")
        };
        let held = standing.position() + standing.rotation() * local_anchor_b;
        assert!(
            (held - foot).norm() < 0.01,
            "the weld is at {held:?}, not the foot"
        );
        let sagged = (standing.position() - at).norm();
        assert!(sagged < 0.02, "the tree moved {sagged:.3} m");
    }
}
