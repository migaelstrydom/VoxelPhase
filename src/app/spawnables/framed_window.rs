//! Framed window — a free-standing gothic window whose glass cracks.
//!
//! ```text
//!                 ▲ finial
//!               ╱   ╲            main arch: a lancet, banded in marble
//!             ╱  ○○○  ╲          oculus: a twelve-sided rose, banded
//!           ╱  ╱╲ ╱╲    ╲        two lights: smaller lancets, banded
//!    ▲     │  │  │  │  │    ▲    pinnacles on the buttresses
//!    █     │  │  │  │  │    █    buttresses: slate blocks either side
//!   ═══════════════════════════  sill, then a slate base slab it stands on
//! ```
//!
//! One dynamic compound body. The stone is welded together at a threshold
//! nothing in the game reaches; the three panes are glass children of the
//! same body, held to the frame at `frame_grip` and crazing where they are
//! hit exactly as a [`GlassSheet`](super::GlassSheetDef) does. Because the
//! body is dynamic the frame itself can be knocked over, and a frame that
//! stops dead loads its glass through the joints that carry it: a window
//! toppled onto its face shatters, one nudged does not.
//!
//! The tracery is honest about its gaps: the eyes between the oculus band,
//! the lights' heads and the main arch are open air, as pierced tracery is.

use std::f32::consts::PI;
use std::sync::Arc;

use nalgebra::{Point3, Vector2, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, PieceStyle, SolidFace};
use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::{MaterialCtx, Spawnable};
use crate::collision::convex_hull::ConvexHull;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::systems::compound_model_of;
use crate::fracture::{CompoundFracture, Deadband, FractureJoint};
use crate::glass::{BrittleSheet, ConvexPolygon, CrackWeb, CrazeRule, SheetFrame};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::pattern;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 128;

/// Joint threshold, in N·s, between pieces of stone: never reached.
const WELD: f32 = 1.0e6;
/// Hairline the glass sits inside its band, so no shard starts flush with
/// the stone and freezes against it in CCD.
const GLAZING_GAP: f32 = 0.0008;
/// Width of the stone band around each opening, in metres.
const BAND: f32 = 0.10;
/// Depth of the stonework through the window, in metres.
const DEPTH: f32 = 0.12;
/// Width of the tracery bars between the lights and around the oculus.
const BAR: f32 = 0.06;
/// Segments per side of a lancet's arch.
const ARCH_SEGMENTS: usize = 6;
const OCULUS_SIDES: usize = 12;

#[derive(Deserialize)]
pub struct FramedWindowDef {
    /// Centre of the base slab's underside: where the window stands.
    pub pos: (f32, f32, f32),

    /// Rotation about `+Y`, in degrees. The glass faces along `+Z` at zero.
    #[serde(default)]
    pub yaw: f32,

    /// Width of the glazed opening, in metres. The arch rises from the
    /// springing line by 0.87 of this: a pointed arch is as tall as it is
    /// wide, near enough.
    #[serde(default = "FramedWindowDef::default_width")]
    pub width: f32,

    /// Height of the opening's straight sides below the arch, in metres.
    #[serde(default = "FramedWindowDef::default_height")]
    pub height: f32,

    /// Glass thickness, in metres.
    #[serde(default = "FramedWindowDef::default_thickness")]
    pub thickness: f32,

    /// Contact spike, in N·s, that cracks a pane.
    #[serde(default = "FramedWindowDef::default_impact_threshold")]
    pub impact_threshold: f32,

    /// Blast impulse, in N·s, that breaks a shard free of its neighbours.
    #[serde(default = "FramedWindowDef::default_blast_threshold")]
    pub blast_threshold: f32,

    /// Impulse, in N·s, that pulls a shard out of the stone it touches.
    #[serde(default = "FramedWindowDef::default_frame_grip")]
    pub frame_grip: f32,
}

impl FramedWindowDef {
    pub fn default_width() -> f32 {
        1.2
    }

    pub fn default_height() -> f32 {
        1.1
    }

    pub fn default_thickness() -> f32 {
        0.02
    }

    pub fn default_impact_threshold() -> f32 {
        25.0
    }

    pub fn default_blast_threshold() -> f32 {
        8.0
    }

    /// Well above the impact threshold, so a blow on the stone next to a
    /// pane does not pull the pane out of its band.
    pub fn default_frame_grip() -> f32 {
        150.0
    }

    /// Half-extents, along the window's own `X` and `Z`, of the slab it
    /// stands on: the ground it needs.
    pub fn base_half_footprint(width: f32) -> (f32, f32) {
        (width * 0.5 + 0.4, 0.4)
    }

    fn crazing(&self) -> CrazeRule {
        CrazeRule {
            threshold: self.impact_threshold,
            web: CrackWeb {
                core_radius: 0.06,
                ring_growth: 1.7,
                rings: 4,
                background_spacing: 0.3,
            },
            min_area: 0.002,
        }
    }

    /// Every piece of the window, in the body's frame with the origin at the
    /// base's underside.
    fn design(&self) -> Design {
        let (half_base_w, half_base_d) = Self::base_half_footprint(self.width);
        let base_height = 0.24;
        let sill_height = 0.12;
        let sill_top = base_height + sill_height;
        let w = self.width;
        let opening = lancet(0.0, sill_top, w, self.height, ARCH_SEGMENTS);
        let apex = sill_top + self.height + w * (3.0f32).sqrt() * 0.5;

        let mut stone = Vec::new();
        stone.push(Stone::Block {
            half_extents: Vector3::new(half_base_w, base_height * 0.5, half_base_d),
            offset: Vector3::new(0.0, base_height * 0.5, 0.0),
            dark: true,
        });
        stone.push(Stone::Block {
            half_extents: Vector3::new(
                w * 0.5 + BAND + 0.18,
                sill_height * 0.5,
                DEPTH * 0.5 + 0.08,
            ),
            offset: Vector3::new(0.0, base_height + sill_height * 0.5, 0.0),
            dark: false,
        });
        let buttress_height = self.height + 0.35;
        let buttress_x = w * 0.5 + BAND + 0.1;
        for side in [-1.0, 1.0] {
            stone.push(Stone::Block {
                half_extents: Vector3::new(0.1, buttress_height * 0.5, DEPTH * 0.5 + 0.1),
                offset: Vector3::new(side * buttress_x, base_height + buttress_height * 0.5, 0.0),
                dark: true,
            });
            stone.push(Stone::Pinnacle {
                base_centre: Vector3::new(side * buttress_x, base_height + buttress_height, 0.0),
                half: 0.09,
                height: 0.32,
            });
        }
        stone.push(Stone::Pinnacle {
            base_centre: Vector3::new(0.0, apex + BAND * 1.3, 0.0),
            half: 0.07,
            height: 0.3,
        });
        for quad in band(&opening, BAND) {
            stone.push(Stone::Band(quad));
        }

        // Two lights under a rose: the tracery.
        let light_w = (w - BAR) * 0.5;
        let light_x = (light_w + BAR) * 0.5;
        let light_h = self.height * 0.8;
        let mut glass = Vec::new();
        for side in [-1.0, 1.0] {
            let light = lancet(
                side * light_x,
                sill_top,
                light_w,
                light_h,
                ARCH_SEGMENTS - 1,
            );
            for quad in band(&light, BAR * 0.5) {
                stone.push(Stone::Band(quad));
            }
            glass.push(light);
        }
        let light_apex = sill_top + light_h + light_w * (3.0f32).sqrt() * 0.5;
        let radius = (w * 0.5 - BAR) * 0.42;
        let rose = regular(
            Vector2::new(0.0, light_apex + BAR + radius),
            radius,
            OCULUS_SIDES,
        );
        for quad in band(&rose, BAR * 0.6) {
            stone.push(Stone::Band(quad));
        }
        glass.push(rose);

        Design { stone, glass }
    }
}

/// The window laid out, before it is a body.
struct Design {
    stone: Vec<Stone>,
    glass: Vec<ConvexPolygon>,
}

/// One piece of stonework.
enum Stone {
    /// A plain block; `dark` picks the slate over the marble.
    Block {
        half_extents: Vector3<f32>,
        offset: Vector3<f32>,
        dark: bool,
    },
    /// A quad of the band around an opening, in the window plane.
    Band(ConvexPolygon),
    /// A square pyramid standing on `base_centre`.
    Pinnacle {
        base_centre: Vector3<f32>,
        half: f32,
        height: f32,
    },
}

/// A pointed arch opening standing on `bottom`: straight sides up to the
/// springing line, then two arcs each centred on the opposite springer, so
/// the arch is as wide as its radius and the sides meet the arcs smoothly.
fn lancet(
    centre_x: f32,
    bottom: f32,
    width: f32,
    side_height: f32,
    segments: usize,
) -> ConvexPolygon {
    let half = width * 0.5;
    let springing = bottom + side_height;
    let mut points = vec![
        Vector2::new(centre_x - half, bottom),
        Vector2::new(centre_x + half, bottom),
    ];
    let arc = |centre_x: f32, from: f32, to: f32, points: &mut Vec<Vector2<f32>>| {
        for i in 0..=segments {
            let angle = from + (to - from) * i as f32 / segments as f32;
            points.push(Vector2::new(
                centre_x + width * angle.cos(),
                springing + width * angle.sin(),
            ));
        }
    };
    arc(centre_x - half, 0.0, PI / 3.0, &mut points);
    arc(centre_x + half, 2.0 * PI / 3.0, PI, &mut points);
    ConvexPolygon::new(points).expect("a lancet is convex")
}

/// A regular polygon.
fn regular(centre: Vector2<f32>, radius: f32, sides: usize) -> ConvexPolygon {
    let points = (0..sides)
        .map(|i| {
            let angle = 2.0 * PI * i as f32 / sides as f32;
            centre + Vector2::new(radius * angle.cos(), radius * angle.sin())
        })
        .collect();
    ConvexPolygon::new(points).expect("a regular polygon is convex")
}

/// The ring of quads `width` wide just outside `opening`, one per edge, with
/// mitred corners so neighbouring quads share their full sides.
fn band(opening: &ConvexPolygon, width: f32) -> Vec<ConvexPolygon> {
    let inner = opening.vertices();
    let n = inner.len();
    let outward = |i: usize| {
        let edge = inner[(i + 1) % n] - inner[i];
        Vector2::new(edge.y, -edge.x).normalize()
    };
    let outer: Vec<Vector2<f32>> = (0..n)
        .map(|i| {
            let before = outward((i + n - 1) % n);
            let after = outward(i);
            let bisector = before + after;
            inner[i] + bisector * (width / (1.0 + before.dot(&after)))
        })
        .collect();
    (0..n)
        .filter_map(|i| {
            let j = (i + 1) % n;
            ConvexPolygon::new(vec![inner[i], inner[j], outer[j], outer[i]])
        })
        .collect()
}

/// A square pyramid on a base of half-width `half`, apex `height` above it,
/// with its own origin at the base's centre.
fn pyramid(half: f32, height: f32) -> ConvexHull {
    let vertices = [
        Vector3::new(-half, 0.0, -half),
        Vector3::new(half, 0.0, -half),
        Vector3::new(half, 0.0, half),
        Vector3::new(-half, 0.0, half),
        Vector3::new(0.0, height, 0.0),
    ];
    let faces = [
        SolidFace {
            vertex_indices: vec![0, 1, 2, 3],
            opposite_vertex: 4,
        },
        SolidFace {
            vertex_indices: vec![0, 1, 4],
            opposite_vertex: 2,
        },
        SolidFace {
            vertex_indices: vec![1, 2, 4],
            opposite_vertex: 3,
        },
        SolidFace {
            vertex_indices: vec![2, 3, 4],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![3, 0, 4],
            opposite_vertex: 1,
        },
    ];
    build_convex_hull(&vertices, &faces)
}

impl Spawnable for FramedWindowDef {
    fn material_count(&self) -> usize {
        3
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let seed = |index| seed_from_position(self.pos, index);
        Ok(vec![
            ctx.patterned(&substance::GLASS, &pattern::GLASS, seed(0), TEXTURE_SIZE)?,
            ctx.patterned(&substance::MARBLE, &pattern::MARBLE, seed(1), TEXTURE_SIZE)?,
            ctx.patterned(&substance::SLATE, &pattern::SLATE, seed(2), TEXTURE_SIZE)?,
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let (glass_material, marble, slate) = (materials[0], materials[1], materials[2]);
        let origin = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let rotation = Yaw::degrees(self.yaw).rotation();
        let design = self.design();
        let stone_frame = SheetFrame::upright(DEPTH);
        let glass_frame = SheetFrame::upright(self.thickness);

        let mut physics = world.write_resource::<PhysicsResource>();
        let body = physics.world.create_body(
            RigidBodyDesc::dynamic()
                .position(origin)
                .rotation(rotation)
                .linear_damping(0.01)
                .angular_damping(0.005),
        );

        let mut child_materials = Vec::new();
        let mut joints = Vec::new();
        let mut stone_footprints: Vec<(usize, ConvexPolygon)> = Vec::new();

        for piece in &design.stone {
            let (desc, material, footprint) = match piece {
                Stone::Block {
                    half_extents,
                    offset,
                    dark,
                } => {
                    let (substance, material): (&Substance, MaterialId) = if *dark {
                        (&substance::SLATE, slate)
                    } else {
                        (&substance::MARBLE, marble)
                    };
                    let footprint = stone_frame
                        .polygon_of(&ColliderDesc::box_shape(*half_extents).shape, *offset)
                        .map(|(polygon, _)| polygon);
                    (
                        ColliderDesc::box_shape(*half_extents)
                            .offset_translation(*offset)
                            .of(substance),
                        material,
                        footprint,
                    )
                }
                Stone::Band(quad) => {
                    let (hull, centroid) = stone_frame.prism(quad);
                    (
                        ColliderDesc::convex_hull(Arc::new(hull))
                            .offset_translation(stone_frame.lift(centroid, 0.0))
                            .of(&substance::MARBLE),
                        marble,
                        Some(quad.clone()),
                    )
                }
                Stone::Pinnacle {
                    base_centre,
                    half,
                    height,
                } => (
                    ColliderDesc::convex_hull(Arc::new(pyramid(*half, *height)))
                        .offset_translation(*base_centre)
                        .of(&substance::MARBLE),
                    marble,
                    None,
                ),
            };
            let index = child_materials.len();
            physics.world.attach_collider(body, desc);
            child_materials.push(material);
            if index > 0 {
                joints.push(FractureJoint {
                    child_a: 0,
                    child_b: index,
                    threshold: WELD,
                });
            }
            if let Some(polygon) = footprint {
                stone_footprints.push((index, polygon));
            }
        }

        for pane in &design.glass {
            let Some(glazed) = pane.inset(GLAZING_GAP) else {
                continue;
            };
            let (hull, centroid) = glass_frame.prism(&glazed);
            let index = child_materials.len();
            physics.world.attach_collider(
                body,
                ColliderDesc::convex_hull(Arc::new(hull))
                    .offset_translation(glass_frame.lift(centroid, 0.0))
                    .of(&substance::GLASS),
            );
            child_materials.push(glass_material);
            for (stone, polygon) in &stone_footprints {
                if glazed.touches(polygon, 0.01) {
                    joints.push(FractureJoint {
                        child_a: index,
                        child_b: *stone,
                        threshold: self.frame_grip,
                    });
                }
            }
        }

        let child_count = child_materials.len();
        let model = compound_model_of(&physics, body, &child_materials, PieceStyle::default())
            .expect("a window has something to draw");
        drop(physics);

        let fracture = CompoundFracture::boxes(joints, child_count, marble)
            .with_piece_materials(child_materials)
            .breaking_on_impact_at(self.impact_threshold)
            .with_deadband(Deadband::OwnWeight)
            .shedding_debris();
        let sheet = BrittleSheet::whole(
            glass_frame,
            self.crazing(),
            self.blast_threshold,
            glass_material,
        )
        .held_by_frame(self.frame_grip)
        .with_deadband(Deadband::OwnWeight);

        vec![world
            .create_entity()
            .with(Position(origin.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(rotation))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(fracture)
            .with(sheet)
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use specs::{Join, RunNow};

    use super::*;
    use crate::debug::{DebugLines, DebugLog};
    use crate::fracture::{Debris, FractureSystem};
    use crate::glass::GlassCrackSystem;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{PhysicsConfig, PhysicsImpulseQueue, PhysicsWorld};
    use crate::time::Time;

    const FRAME_DT: f32 = 1.0 / 60.0;

    fn default_def() -> FramedWindowDef {
        FramedWindowDef {
            pos: (0.0, 0.0, 0.0),
            yaw: 0.0,
            width: FramedWindowDef::default_width(),
            height: FramedWindowDef::default_height(),
            thickness: FramedWindowDef::default_thickness(),
            impact_threshold: FramedWindowDef::default_impact_threshold(),
            blast_threshold: FramedWindowDef::default_blast_threshold(),
            frame_grip: FramedWindowDef::default_frame_grip(),
        }
    }

    /// A world with one window standing on flat ground, ready to step.
    fn standing_window() -> (World, Entity) {
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Orientation>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CompoundFracture>();
        world.register::<BrittleSheet>();
        world.register::<Debris>();
        world.insert(Time::fixed(FRAME_DT));
        world.insert(PhysicsImpulseQueue::default());
        world.insert(DebugLog::default());
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(PhysicsConfig::default()),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));
        let materials = [MaterialId(0), MaterialId(1), MaterialId(2)];
        let entity = default_def().spawn(&mut world, &materials)[0];
        (world, entity)
    }

    fn frame(world: &mut World, stepper: &mut SequentialStepper, geometry: &FlatQuadGeometry) {
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let mut debug = DebugLines::default();
            stepper.step(&mut physics.world, FRAME_DT, geometry, &[], &[], &mut debug);
        }
        GlassCrackSystem.run_now(world);
        FractureSystem::default().run_now(world);
        world.maintain();
    }

    fn glass_children(world: &World, entity: Entity) -> usize {
        let fractures = world.read_storage::<CompoundFracture>();
        let sheets = world.read_storage::<BrittleSheet>();
        let (Some(fracture), Some(sheet)) = (fractures.get(entity), sheets.get(entity)) else {
            return 0;
        };
        (0..fracture.child_count)
            .filter(|child| sheet.is_glass(fracture.material_of(*child)))
            .count()
    }

    fn freed_shards(world: &World) -> usize {
        world.read_storage::<Debris>().join().count()
    }

    /// A window left alone on the ground is a window: nothing settles out
    /// of it, and its own weight coming onto the base is not a blow.
    #[test]
    fn a_window_standing_on_the_ground_keeps_its_glass() {
        let (mut world, entity) = standing_window();
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let panes = glass_children(&world, entity);
        assert_eq!(panes, 3);

        for _ in 0..180 {
            frame(&mut world, &mut stepper, &geometry);
        }

        assert_eq!(
            glass_children(&world, entity),
            panes,
            "a pane cracked by itself"
        );
        assert_eq!(freed_shards(&world), 0);
    }

    /// Carried and swung about — as the grab's orientation lock does to a held
    /// body, with a torque budget that yanks even a tonne of stone — the
    /// glass is loaded by nothing but its own frame, and stays whole. Only a
    /// frame that strikes something passes its jolt on.
    #[test]
    fn a_window_swung_about_in_the_air_keeps_its_glass() {
        let (mut world, entity) = standing_window();
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let handle = world
            .read_storage::<RigidBodyComponent>()
            .get(entity)
            .expect("the window has a body")
            .0;
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            physics.world.wake_body(handle);
            let body = physics.world.body_mut(handle).expect("live body");
            body.set_position(Point3::new(0.0, 3.0, 0.0));
        }

        for i in 0..40 {
            {
                let mut physics = world.write_resource::<PhysicsResource>();
                let body = physics.world.body_mut(handle).expect("live body");
                body.set_linear_velocity(Vector3::zeros());
                let swing = if i % 2 == 0 { 1.5 } else { -1.5 };
                body.set_angular_velocity(Vector3::new(0.0, swing, 0.0));
            }
            frame(&mut world, &mut stepper, &geometry);
        }

        assert_eq!(
            glass_children(&world, entity),
            3,
            "a pane cracked in the air"
        );
        assert_eq!(freed_shards(&world), 0);
    }

    /// Tipped over onto its face, the frame stops dead on the ground and the
    /// jolt shatters the glass, while the stonework stays one body.
    #[test]
    fn a_window_toppled_onto_its_face_shatters_but_the_frame_holds() {
        let (mut world, entity) = standing_window();
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let stone = {
            let fractures = world.read_storage::<CompoundFracture>();
            fractures.get(entity).map_or(0, |f| f.child_count) - glass_children(&world, entity)
        };

        {
            let bodies = world.read_storage::<RigidBodyComponent>();
            let handle = bodies.get(entity).expect("the window has a body").0;
            let mut physics = world.write_resource::<PhysicsResource>();
            physics.world.wake_body(handle);
            let body = physics.world.body_mut(handle).expect("live body");
            body.set_angular_velocity(Vector3::new(2.5, 0.0, 0.0));
        }

        for _ in 0..140 {
            frame(&mut world, &mut stepper, &geometry);
        }

        assert!(
            freed_shards(&world) > 10,
            "only {} shards came out",
            freed_shards(&world)
        );
        let fractures = world.read_storage::<CompoundFracture>();
        let fracture = fractures.get(entity).expect("the frame is still there");
        let sheets = world.read_storage::<BrittleSheet>();
        let sheet = sheets
            .get(entity)
            .expect("the sheet component stays with the frame");
        let stone_left = (0..fracture.child_count)
            .filter(|child| !sheet.is_glass(fracture.material_of(*child)))
            .count();
        assert_eq!(stone_left, stone, "the stonework lost pieces");
    }

    #[test]
    fn a_lancet_is_as_tall_above_its_springing_as_it_is_wide_is_high() {
        let arch = lancet(0.0, 0.0, 1.0, 0.5, 6);
        let (low, high) = arch.bounds();
        assert!((low.x + 0.5).abs() < 1e-5 && (high.x - 0.5).abs() < 1e-5);
        assert!((high.y - (0.5 + 0.75f32.sqrt())).abs() < 1e-4);
    }

    #[test]
    fn a_band_wraps_its_opening_without_gaps() {
        let opening = lancet(0.0, 0.0, 1.0, 0.5, 6);
        let quads = band(&opening, 0.1);
        assert_eq!(quads.len(), opening.vertices().len());
        for (i, quad) in quads.iter().enumerate() {
            let next = &quads[(i + 1) % quads.len()];
            assert!(quad.touches(next, 0.05), "quad {i} does not meet the next");
            assert!(
                quad.touches(&opening, 0.05),
                "quad {i} does not meet the glass"
            );
        }
    }

    #[test]
    fn every_pane_is_held_by_some_stone() {
        let def = FramedWindowDef {
            pos: (0.0, 0.0, 0.0),
            yaw: 0.0,
            width: FramedWindowDef::default_width(),
            height: FramedWindowDef::default_height(),
            thickness: FramedWindowDef::default_thickness(),
            impact_threshold: 25.0,
            blast_threshold: 8.0,
            frame_grip: 150.0,
        };
        let design = def.design();
        assert_eq!(design.glass.len(), 3);
        for pane in &design.glass {
            let glazed = pane
                .inset(GLAZING_GAP)
                .expect("a pane survives its glazing gap");
            let held = design.stone.iter().any(|piece| match piece {
                Stone::Band(quad) => glazed.touches(quad, 0.01),
                _ => false,
            });
            assert!(held, "a pane touches no band");
        }
    }
}
