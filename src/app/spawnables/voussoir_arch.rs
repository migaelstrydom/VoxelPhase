//! Voussoir arch spawnable — semicircular masonry arch that locks under gravity.
//!
//! Creates a classic Roman-style arch from wedge-shaped stone blocks (voussoirs)
//! arranged in a semicircle, supported by two heavy abutment pillars. The arch
//! holds together purely through compressive forces and friction.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::shared::models::{build_convex_hull, SolidFace, SurfaceUvs};
use super::stone::{StoneBlock, StoneShape, StoneTexture};
use super::{MaterialCtx, Spawnable};
use crate::collision::convex_hull::ConvexHull;
use crate::core::error::EngineResult;
use crate::rendering::material::MaterialId;
use crate::rendering::substance::{self, Substance};

#[derive(Deserialize)]
pub struct VoussoirArchDef {
    /// World position of the ground-level center of the arch opening.
    pub base: (f32, f32, f32),
    /// Inner radius of the arch (half-width of the opening).
    #[serde(default = "VoussoirArchDef::default_inner_radius")]
    pub inner_radius: f32,
    /// Radial thickness of each voussoir block.
    #[serde(default = "VoussoirArchDef::default_thickness")]
    pub thickness: f32,
    /// Depth of the arch along the Z axis.
    #[serde(default = "VoussoirArchDef::default_depth")]
    pub depth: f32,
    /// Number of voussoir blocks (odd gives a centered keystone).
    #[serde(default = "VoussoirArchDef::default_num_voussoirs")]
    pub num_voussoirs: u32,
    /// Height of the abutment pillars supporting the arch.
    #[serde(default = "VoussoirArchDef::default_abutment_height")]
    pub abutment_height: f32,
    #[serde(default = "VoussoirArchDef::default_density")]
    pub density: f32,
    #[serde(default = "VoussoirArchDef::default_friction")]
    pub friction: f32,
}

impl Default for VoussoirArchDef {
    /// The arch a level gets when it gives nothing but a base, at the origin.
    fn default() -> Self {
        Self {
            base: (0.0, 0.0, 0.0),
            inner_radius: Self::default_inner_radius(),
            thickness: Self::default_thickness(),
            depth: Self::default_depth(),
            num_voussoirs: Self::default_num_voussoirs(),
            abutment_height: Self::default_abutment_height(),
            density: Self::default_density(),
            friction: Self::default_friction(),
        }
    }
}

impl VoussoirArchDef {
    pub fn default_inner_radius() -> f32 {
        2.0
    }
    pub fn default_thickness() -> f32 {
        0.5
    }
    pub fn default_depth() -> f32 {
        1.2
    }
    pub fn default_num_voussoirs() -> u32 {
        11
    }
    pub fn default_abutment_height() -> f32 {
        1.5
    }
    pub fn default_density() -> f32 {
        2000.0
    }
    pub fn default_friction() -> f32 {
        0.8
    }

    fn total_pieces(&self) -> usize {
        self.num_voussoirs as usize + 2
    }

    /// The voussoirs: dressed limestone, with the friction and density a level
    /// authors. Declared once so the collider and the stone they are rendered
    /// with cannot disagree about what they are.
    pub fn voussoir_substance(&self) -> Substance {
        substance::LIMESTONE
            .with_density(self.density)
            .with_friction(self.friction)
    }

    /// The abutment pillars: the same stone at twice the density, so they stay
    /// put under the arch's thrust. Twice as heavy and identical to look at,
    /// which is the separation the substance library exists to allow.
    pub fn abutment_substance(&self) -> Substance {
        self.voussoir_substance().with_density(self.density * 2.0)
    }
}

impl Spawnable for VoussoirArchDef {
    fn material_count(&self) -> usize {
        2
    }

    /// One material for the voussoirs and one for the abutments. They are the
    /// same stone and share one texture; they differ in density, which the
    /// finish is derived from. Blocks do not need a texture each to look
    /// different, because each reads the texture at its own place in the
    /// world.
    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let texture = StoneTexture::WEATHERED;
        Ok(vec![
            texture.material(ctx, &self.voussoir_substance())?,
            texture.material(ctx, &self.abutment_substance())?,
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let inner_r = self.inner_radius;
        let half_depth = self.depth / 2.0;
        let uvs = self.surface_uvs();

        // Arch center of curvature sits at the top of the abutment pillars.
        let center = Vector3::new(self.base.0, self.base.1 + self.abutment_height, self.base.2);

        let mut entities = Vec::with_capacity(self.total_pieces());

        // --- Voussoirs ---
        for i in 0..self.num_voussoirs {
            let (hull, centroid) = self.voussoir(i);
            entities.push(
                StoneBlock {
                    centre: Point3::from(center + centroid),
                    rotation: UnitQuaternion::identity(),
                    shape: StoneShape::Hull(Arc::new(hull)),
                    substance: self.voussoir_substance(),
                    material: materials[0],
                    uvs,
                }
                .spawn(world),
            );
        }

        // --- Abutment pillars ---
        let abutment_he =
            Vector3::new(self.thickness / 2.0, self.abutment_height / 2.0, half_depth);

        for side in [1.0f32, -1.0] {
            let centre = Point3::new(
                center.x + side * (inner_r + self.thickness / 2.0),
                self.base.1 + self.abutment_height / 2.0,
                center.z,
            );
            entities.push(
                StoneBlock {
                    centre,
                    rotation: UnitQuaternion::identity(),
                    shape: StoneShape::Box(abutment_he),
                    substance: self.abutment_substance(),
                    material: materials[1],
                    uvs,
                }
                .spawn(world),
            );
        }

        entities
    }
}

impl VoussoirArchDef {
    /// Voussoir `index`, counted from the right-hand springing: its hull about
    /// its own centre, and where that centre sits relative to the arch's
    /// centre of curvature.
    pub fn voussoir(&self, index: u32) -> (ConvexHull, Vector3<f32>) {
        let angle_step = std::f32::consts::PI / self.num_voussoirs as f32;
        let (arch_verts, faces) = voussoir_geometry(
            self.inner_radius,
            self.inner_radius + self.thickness,
            self.depth / 2.0,
            index as f32 * angle_step,
            (index + 1) as f32 * angle_step,
        );
        let centroid = arch_verts.iter().copied().sum::<Vector3<f32>>() / arch_verts.len() as f32;
        let local_verts: Vec<_> = arch_verts.iter().map(|v| v - centroid).collect();
        (build_convex_hull(&local_verts, &faces), centroid)
    }

    /// How the stone's texture is laid on every block: at one scale for the
    /// whole arch, set by its thickness.
    pub fn surface_uvs(&self) -> SurfaceUvs {
        StoneTexture::WEATHERED.uvs(self.thickness)
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Vertices and faces of a single voussoir (truncated wedge).
///
/// Computed in arch-local space where the arch center of curvature is at the
/// origin. The voussoir spans from `angle_start` to `angle_end` (radians,
/// measured counter-clockwise from the +X axis in the XY plane).
///
/// Returns 8 vertices and 6 quadrilateral faces.
fn voussoir_geometry(
    inner_radius: f32,
    outer_radius: f32,
    half_depth: f32,
    angle_start: f32,
    angle_end: f32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let cos_s = angle_start.cos();
    let sin_s = angle_start.sin();
    let cos_e = angle_end.cos();
    let sin_e = angle_end.sin();

    let vertices = vec![
        // Front face (z = +half_depth)
        Vector3::new(inner_radius * cos_s, inner_radius * sin_s, half_depth), // 0: inner-start
        Vector3::new(outer_radius * cos_s, outer_radius * sin_s, half_depth), // 1: outer-start
        Vector3::new(outer_radius * cos_e, outer_radius * sin_e, half_depth), // 2: outer-end
        Vector3::new(inner_radius * cos_e, inner_radius * sin_e, half_depth), // 3: inner-end
        // Back face (z = -half_depth)
        Vector3::new(inner_radius * cos_s, inner_radius * sin_s, -half_depth), // 4: inner-start
        Vector3::new(outer_radius * cos_s, outer_radius * sin_s, -half_depth), // 5: outer-start
        Vector3::new(outer_radius * cos_e, outer_radius * sin_e, -half_depth), // 6: outer-end
        Vector3::new(inner_radius * cos_e, inner_radius * sin_e, -half_depth), // 7: inner-end
    ];

    let faces = vec![
        // Front face (+Z). Opposite: any back vertex.
        SolidFace {
            vertex_indices: vec![0, 1, 2, 3],
            opposite_vertex: 4,
        },
        // Back face (-Z). Opposite: any front vertex.
        SolidFace {
            vertex_indices: vec![7, 6, 5, 4],
            opposite_vertex: 0,
        },
        // Start radial face. Opposite: an end-side vertex.
        SolidFace {
            vertex_indices: vec![0, 4, 5, 1],
            opposite_vertex: 3,
        },
        // End radial face. Opposite: a start-side vertex.
        SolidFace {
            vertex_indices: vec![3, 2, 6, 7],
            opposite_vertex: 0,
        },
        // Outer face. Opposite: an inner vertex.
        SolidFace {
            vertex_indices: vec![1, 5, 6, 2],
            opposite_vertex: 0,
        },
        // Inner face. Opposite: an outer vertex.
        SolidFace {
            vertex_indices: vec![0, 3, 7, 4],
            opposite_vertex: 1,
        },
    ];

    (vertices, faces)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cleave::{BrittleSolid, SolidCleaveSystem};
    use crate::components::{
        ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
    };
    use crate::debug::{DebugLines, DebugLog};
    use crate::fracture::{CompoundFracture, FractureSystem};
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{PhysicsConfig, PhysicsImpulseQueue, PhysicsWorld};
    use crate::systems::PhysicsResource;
    use crate::time::Time;
    use specs::{Join, RunNow, WorldExt};

    const FRAME_DT: f32 = 1.0 / 60.0;

    fn world_with_arch() -> (World, Vec<Entity>) {
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
        // Asleep, the ring would hang in the air when its abutment goes: a
        // removed body wakes nothing that was resting on it.
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(config),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));
        let arch = VoussoirArchDef::default();
        let stones = arch.spawn(&mut world, &[MaterialId(0), MaterialId(1)]);
        (world, stones)
    }

    fn run(world: &mut World, frames: usize) {
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
    }

    /// Pieces of stone per body, over every body in the world.
    fn pieces(world: &World) -> Vec<usize> {
        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        (&bodies)
            .join()
            .filter_map(|body| physics.world.body(body.0).map(|b| b.colliders().len()))
            .collect()
    }

    /// Standing, the arch carries its own thrust without a crack: the load a
    /// voussoir carries is not a blow, however much it swings as the ring
    /// beds in.
    #[test]
    fn a_standing_arch_does_not_crack_under_its_own_weight() {
        let (mut world, stones) = world_with_arch();
        run(&mut world, 180);
        assert_eq!(pieces(&world).iter().sum::<usize>(), stones.len());
    }

    /// Take an abutment away and the arch comes down. Some stones crack,
    /// once each: the fall is a few pieces more than the arch, never a
    /// shower of rubble.
    #[test]
    fn a_falling_arch_cracks_stones_but_does_not_shatter() {
        let (mut world, stones) = world_with_arch();
        run(&mut world, 30);
        let abutment = *stones.last().expect("the arch has abutments");
        {
            let body = world
                .read_storage::<RigidBodyComponent>()
                .get(abutment)
                .unwrap()
                .0;
            world
                .write_resource::<PhysicsResource>()
                .world
                .remove_body(body);
        }
        world.delete_entity(abutment).unwrap();
        run(&mut world, 360);

        let total: usize = pieces(&world).iter().sum();
        let whole = stones.len() - 1;
        assert!(total > whole, "no stone cracked in the fall");
        assert!(
            total <= whole * 2,
            "{total} pieces from {whole} stones: something broke more than once"
        );
    }

    /// Every stone of the arch, whole and cracked every way the game cracks
    /// it, is drawn right side out.
    ///
    /// Cracked stones used to show holes the size of a fist: a point near an
    /// arris was pushed straight in by the depth of a chip, past the points
    /// pushed in from the neighbouring face, and the drawing folded. Slivers
    /// on the crease of a break outlived that, until the wear was read where
    /// each projection ray begins and measured along it.
    #[test]
    fn every_stone_whole_or_cracked_is_drawn_right_side_out() {
        use crate::app::spawnables::shared::models::PieceHull;
        use crate::app::spawnables::stone::inside_out;
        use crate::app::spawnables::{stone_cleaving, weathered_hull_mesh};
        use crate::physics::ColliderShape;

        let arch = VoussoirArchDef {
            base: (44.0, 11.5, 44.0),
            ..VoussoirArchDef::default()
        };
        let centre = Vector3::new(44.0, 13.0, 44.0);
        for i in 0..arch.num_voussoirs {
            let (hull, offset) = arch.voussoir(i);
            let anchor = centre + offset;
            let whole = hull.translated(anchor);
            let (v, idx) = weathered_hull_mesh(
                &PieceHull::new(&hull, anchor).within(Some(&whole)),
                arch.surface_uvs(),
            );
            assert_eq!(inside_out(&v, &idx), 0, "voussoir {i} is folded whole");

            let shape = ColliderShape::ConvexHull {
                hull: Arc::new(hull.clone()),
            };
            for salt in 0..12u32 {
                let hit = Vector3::new(
                    ((salt * 7) % 5) as f32 * 0.1 - 0.2,
                    ((salt * 3) % 5) as f32 * 0.1 - 0.2,
                    ((salt * 11) % 7) as f32 * 0.15 - 0.45,
                );
                let Some(pieces) = stone_cleaving(1000.0).cleave(&shape, hit, salt) else {
                    continue;
                };
                for piece in pieces {
                    // Shrunk as the cleave system shrinks it.
                    let drawn = piece.hull.scaled(0.996);
                    let (v, idx) = weathered_hull_mesh(
                        &PieceHull::new(&drawn, anchor + piece.centre).within(Some(&whole)),
                        arch.surface_uvs(),
                    );
                    assert_eq!(inside_out(&v, &idx), 0, "voussoir {i} cut {salt} is folded");
                }
            }
        }
    }
}
