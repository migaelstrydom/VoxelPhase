//! Igloo — a corbelled dome of ice blocks with a doorway cut out of it.
//!
//! ```text
//!            ┌───┐              cap: one slab closing the eye of the dome
//!        ╭───┴───┴───╮
//!      ╭─┴─┬───┬───┬─┴─╮        rings: each course is a ring of blocks laid
//!    ╭─┴─┬─┴─┬─┴─┬─┴─┬─┴─╮      tangent to the dome, half a block out of
//!    │   │   │       │   │      phase with the course below it
//!    ┴───┴───┘  door └───┴
//! ```
//!
//! The dome is built on a mid-surface sphere: every block is a flat brick
//! whose centre lies on that sphere, tilted to match the surface there and
//! sized so that the blocks of a ring meet edge-to-edge at their inner faces.
//! Rings are generated from the ground up until the remaining hole is smaller
//! than a block, and a single slab caps what is left — which is how a real
//! igloo ends too.
//!
//! **The dome is one dynamic body, with the blocks welded into it as
//! children.** Ice has a friction coefficient of 0.06, the lowest of any
//! substance in the library, so a corbelled dome of separate blocks has
//! nothing whatsoever holding it together: it slides apart the moment the
//! simulation starts. Nothing about that says the igloo should be part of the
//! *world*, though — a static shell keeps standing in mid-air after the ground
//! under it is blown away. So the blocks are welded rather than fixed, the
//! same way a table's legs are welded to its top:
//!
//! ```text
//!   IglooDef::blocks ─┬─▶ colliders (offset + rotated) ─▶ one dynamic body
//!                     ├─▶ compound_model  ─────────────▶ one draw
//!                     └─▶ joints, block to block ──────▶ CompoundFracture
//! ```
//!
//! [`CompoundFracture`] carries a joint between every pair of blocks that
//! touch. A blast drops the joints within its reach, the system works out what
//! is still connected to what, and whatever has come loose is spawned as
//! separate bodies — so a grenade takes a hole out of the dome and leaves the
//! rest of it standing. Short of a blast the igloo is rigid, which is the
//! whole reason it can be a dome of frictionless bricks at all.

use std::f32::consts::{PI, TAU};

use nalgebra::{Matrix3, Point3, Rotation3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::super::shared::models::{compound_model, PiecePlacement};
use super::super::shared::orientation::Yaw;
use super::super::shared::textures::seed_from_position;
use super::super::{MaterialCtx, Spawnable};
use super::block::{ice, ice_cleaving, ice_hull_mesh, ice_material, ice_piece_mesh, ice_uvs};
use crate::cleave::{BrittleSolid, CleaveRule};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{CompoundFracture, FractureJoint};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::pattern::Spread;
use crate::rendering::substance::ColliderSubstance;
use crate::systems::PhysicsResource;

/// How many tiles of pattern a dome's one ice texture holds.
///
/// One, because a dome is built out of bricks and a brick is smaller than a
/// tile, so there is nothing for a second tile to cover. Named rather than
/// written twice: the texture that is baked and the coordinates every block
/// and every wedge addresses it with must come from the same number, or the
/// frost changes size somewhere in the dome.
const BRICK_SPREAD: Spread = Spread::ONE;

/// How far apart two blocks of a course may be and still count as touching,
/// as a fraction of their combined reach.
///
/// The joint left between neighbours is a few per cent of a block; a doorway
/// is most of one. Anything between the two does as the dividing line.
const ADJACENCY_SLACK: f32 = 0.25;

/// How much wider than tall a block is, measured along the dome's surface.
///
/// Bricks rather than tiles: a ring of near-square blocks reads as a mosaic,
/// and the courses stop being legible as courses.
const BLOCK_ASPECT: f32 = 1.8;

/// Fraction of the available arc a block actually occupies, leaving the rest
/// as a joint.
///
/// Blocks that meet exactly would interpenetrate as soon as the arithmetic
/// rounds the wrong way, and interpenetration is far more visible on
/// see-through geometry than a joint is.
const JOINT_FIT: f32 = 0.97;

/// The fewest blocks a ring is built from, however small it has become.
const MIN_RING_BLOCKS: usize = 5;

#[derive(Deserialize)]
pub struct IglooDef {
    /// Centre of the igloo's floor.
    pub pos: (f32, f32, f32),

    /// Outer radius of the dome, which is also its height.
    #[serde(default = "IglooDef::default_radius")]
    pub radius: f32,

    /// Thickness of the wall, front to back through a block.
    #[serde(default = "IglooDef::default_wall_thickness")]
    pub wall_thickness: f32,

    /// Height of one course, measured along the dome's surface.
    #[serde(default = "IglooDef::default_block_height")]
    pub block_height: f32,

    /// Width of the doorway. Zero for a sealed dome.
    #[serde(default = "IglooDef::default_door_width")]
    pub door_width: f32,

    /// Height of the doorway.
    #[serde(default = "IglooDef::default_door_height")]
    pub door_height: f32,

    /// Rotation about `+Y`, in degrees. The doorway faces the igloo's own
    /// `+X`, so this is the field that decides which way you can walk in.
    #[serde(default)]
    pub yaw: f32,

    /// Blast impulse, in N·s at a block, that tears that block's joints.
    ///
    /// A grenade delivers about 1100 N·s at its centre and falls off to
    /// nothing over its blast radius, so the default is a hole around the
    /// point of impact rather than the whole dome at once.
    #[serde(default = "IglooDef::default_fracture_threshold")]
    pub fracture_threshold: f32,
    /// Blow, in N·s, that breaks a single block into wedges rather than
    /// merely knocking it out of the dome. Above `fracture_threshold`, so a
    /// moderate hit takes blocks out whole and a hard one shatters them.
    #[serde(default = "IglooDef::default_cleave_threshold")]
    pub cleave_threshold: f32,
}

impl IglooDef {
    pub fn default_radius() -> f32 {
        2.5
    }

    pub fn default_wall_thickness() -> f32 {
        0.28
    }

    pub fn default_block_height() -> f32 {
        0.45
    }

    pub fn default_door_width() -> f32 {
        1.0
    }

    pub fn default_door_height() -> f32 {
        1.2
    }

    pub fn default_fracture_threshold() -> f32 {
        420.0
    }

    pub fn default_cleave_threshold() -> f32 {
        700.0
    }

    /// Radius of the surface every block's centre sits on.
    fn mid_radius(&self) -> f32 {
        (self.radius - self.wall_thickness * 0.5).max(self.wall_thickness)
    }

    /// The blocks of the dome, in the igloo's own frame.
    fn blocks(&self) -> Vec<DomeBlock> {
        let mid = self.mid_radius();
        let half_thickness = self.wall_thickness * 0.5;

        // Courses of equal arc, so the rings tile the dome's meridian exactly.
        let rings = ((mid * PI * 0.5) / self.block_height).round().max(2.0) as usize;
        let ring_arc = (PI * 0.5) / rings as f32;
        let half_height = mid * ring_arc * 0.5;

        let mut blocks = Vec::new();
        let mut open_radius = mid;
        // Courses actually laid, which is not `rings`: the loop stops early
        // when the hole left at the top is smaller than a block.
        let mut courses = 0;

        for ring in 0..rings {
            let elevation = (ring as f32 + 0.5) * ring_arc;
            let radius = mid * elevation.cos();

            // Once the hole is no wider than a block, the remaining rings
            // would be blocks reaching across their own dome. Cap instead.
            if radius < half_height * 2.0 {
                break;
            }

            let count = self.ring_block_count(radius, half_height);
            // Blocks of a ring close on each other at their inner faces
            // first, so that is the radius the width is fitted against.
            let half_width = (radius - half_thickness) * (PI / count as f32).tan() * JOINT_FIT;
            // Half a block of phase per course: a running bond, on a dome.
            let phase = if ring % 2 == 1 {
                PI / count as f32
            } else {
                0.0
            };

            // The course's lower edge, which is what decides whether the
            // doorway reaches it at all.
            let course_foot = mid * (elevation - ring_arc * 0.5).sin();

            for index in 0..count {
                let laid = Arc {
                    centre: phase + TAU * index as f32 / count as f32,
                    half_angle: half_width / radius,
                };

                let Some(arc) = self.doorway_cuts(laid, radius, course_foot) else {
                    continue;
                };

                blocks.push(DomeBlock {
                    centre: surface_point(mid, elevation, arc.centre),
                    half_extents: Vector3::new(
                        arc.half_angle * radius,
                        half_height,
                        half_thickness,
                    ),
                    rotation: surface_frame(elevation, arc.centre),
                    ring,
                    arc,
                });
            }

            open_radius = mid * ((ring + 1) as f32 * ring_arc).cos();
            courses = ring + 1;
        }

        blocks.push(self.cap(&blocks, open_radius, half_thickness, courses));
        blocks
    }

    /// How many blocks a ring of the given radius is divided into.
    fn ring_block_count(&self, radius: f32, half_height: f32) -> usize {
        let target_width = half_height * 2.0 * BLOCK_ASPECT;
        ((TAU * radius / target_width).round() as usize).max(MIN_RING_BLOCKS)
    }

    /// What the doorway leaves of a block laid at `arc`.
    ///
    /// A block wholly inside the opening is gone; one that straddles an edge
    /// of it is *trimmed back* to the part outside rather than dropped whole.
    /// Dropping is the easier rule and it is the wrong one: it widens the
    /// doorway by up to a block on each side, so the opening a level author
    /// asked for and the one the player walks through are different sizes.
    fn doorway_cuts(&self, arc: Arc, radius: f32, course_foot: f32) -> Option<Arc> {
        if self.door_width <= 0.0 || course_foot > self.door_height {
            return Some(arc);
        }

        // The doorway is a width, not an angle: half of it, taken round the
        // course, is where the jambs stand.
        let jamb = (self.door_width * 0.5 / radius).clamp(-1.0, 1.0).asin();

        let near = signed_azimuth(arc.centre - arc.half_angle);
        let far = signed_azimuth(arc.centre + arc.half_angle);
        // A block that wraps past the back of the dome is nowhere near the
        // door, and its folded edges would compare nonsensically.
        if near > far {
            return Some(arc);
        }

        let (outside_left, outside_right) = (near < -jamb, far > jamb);
        match (outside_left, outside_right) {
            // Clear of the opening on one side or the other.
            _ if near >= jamb || far <= -jamb => Some(arc),
            // Straddling a jamb: keep the part outside it.
            (true, false) => Some(Arc::between(near, -jamb)),
            (false, true) => Some(Arc::between(jamb, far)),
            // Spanning the whole opening, which only happens on a door
            // narrower than a block: keep the wider of the two jamb pieces.
            (true, true) => Some(
                [Arc::between(near, -jamb), Arc::between(jamb, far)]
                    .into_iter()
                    .max_by(|a, b| a.half_angle.total_cmp(&b.half_angle))
                    .expect("two pieces"),
            ),
            // Wholly within the opening.
            (false, false) => None,
        }
    }

    /// The slab that closes the eye of the dome.
    ///
    /// Laid on top of the rim rather than into it: the exact height of the
    /// last course's upper corners is where its underside goes, so the cap
    /// rests on the dome instead of growing through it.
    fn cap(
        &self,
        blocks: &[DomeBlock],
        open_radius: f32,
        half_thickness: f32,
        courses: usize,
    ) -> DomeBlock {
        let rim = blocks
            .iter()
            .map(DomeBlock::top)
            .fold(0.0_f32, |acc, top| acc.max(top));
        let half_width = open_radius + half_thickness;

        DomeBlock {
            centre: Point3::new(0.0, rim + half_thickness, 0.0),
            half_extents: Vector3::new(half_width, half_thickness, half_width),
            rotation: UnitQuaternion::identity(),
            // The course above the last one laid, covering the whole circle —
            // which is what makes it meet every block of the course below when
            // the joints are worked out.
            ring: courses,
            arc: Arc {
                centre: 0.0,
                half_angle: PI,
            },
        }
    }
}

impl Spawnable for IglooDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![ice_material(
            ctx,
            seed_from_position(self.pos, 0),
            BRICK_SPREAD,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let blocks = self.blocks();
        if blocks.is_empty() {
            return Vec::new();
        }

        // The body's origin goes on the dome's centre of mass, not on the
        // floor centre it was authored from. A body that pivots anywhere else
        // carries the inertia of an arm that is not there, and an igloo whose
        // ground is blown out from under it would tip about a point in the
        // air above its own doorway.
        let centre_of_mass = mass_centre(&blocks);
        let yaw = Yaw::degrees(self.yaw);
        let origin = yaw.place(self.pos, centre_of_mass.coords);

        let pieces: Vec<PiecePlacement> = blocks
            .iter()
            .map(|block| {
                PiecePlacement::new(block.half_extents, block.centre - centre_of_mass)
                    .rotated(block.rotation)
            })
            .collect();

        let substance = ice();
        let model = compound_model(&pieces, ice_piece_mesh, ice_uvs(BRICK_SPREAD), materials[0]);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(origin)
                    .rotation(yaw.rotation())
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );

            for piece in &pieces {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(piece.half_extents)
                        .of(&substance)
                        .offset_translation(piece.offset)
                        .offset_rotation(piece.rotation),
                );
            }

            body_handle
        };

        vec![world
            .create_entity()
            .with(Position(origin.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(yaw.rotation()))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(
                CompoundFracture::boxes(self.joints(&blocks), blocks.len(), materials[0])
                    .with_piece_mesh(ice_piece_mesh)
                    .with_hull_mesh(ice_hull_mesh)
                    .with_uvs(ice_uvs(BRICK_SPREAD)),
            )
            .with(BrittleSolid::new(
                CleaveRule {
                    threshold: self.cleave_threshold,
                    ..ice_cleaving()
                },
                materials[0],
                blocks.len(),
            ))
            .build()]
    }
}

impl IglooDef {
    /// Which blocks hold which others up, and how hard they have to be hit to
    /// stop doing so.
    ///
    /// One joint per pair of touching blocks, so the dome comes apart where it
    /// was struck: the fracture system drops the joints inside the blast,
    /// works out what is still connected to what, and lets the rest fall.
    fn joints(&self, blocks: &[DomeBlock]) -> Vec<FractureJoint> {
        let mut joints = Vec::new();
        for (index, block) in blocks.iter().enumerate() {
            for (other_index, other) in blocks.iter().enumerate().skip(index + 1) {
                if block.adjoins(other) {
                    joints.push(FractureJoint {
                        child_a: index,
                        child_b: other_index,
                        threshold: self.fracture_threshold,
                    });
                }
            }
        }
        joints
    }
}

/// The centre of mass of a set of blocks, which all share one density.
fn mass_centre(blocks: &[DomeBlock]) -> Point3<f32> {
    let mut volume = 0.0;
    let mut weighted = Vector3::zeros();
    for block in blocks {
        volume += block.volume();
        weighted += block.centre.coords * block.volume();
    }
    if volume <= 0.0 {
        return Point3::origin();
    }
    Point3::from(weighted / volume)
}

/// A block's extent round its course, as an angle: where its centre sits and
/// how far it reaches each way.
///
/// Laying out a ring and cutting a doorway out of it are both angle problems —
/// blocks meet their neighbours along the course, and the doorway takes a
/// contiguous piece of it — so the ring is built in angles and turned into
/// positions and half-extents once, at the end.
#[derive(Clone, Copy)]
struct Arc {
    centre: f32,
    half_angle: f32,
}

impl Arc {
    fn between(a: f32, b: f32) -> Self {
        Self {
            centre: (a + b) * 0.5,
            half_angle: (b - a).abs() * 0.5,
        }
    }
}

/// One block of the dome, in the igloo's own frame.
struct DomeBlock {
    centre: Point3<f32>,
    half_extents: Vector3<f32>,
    rotation: UnitQuaternion<f32>,
    /// Which course it belongs to, counted from the ground.
    ring: usize,
    /// Where it sits round that course.
    arc: Arc,
}

impl DomeBlock {
    /// Mass, up to the constant density every block shares.
    fn volume(&self) -> f32 {
        8.0 * self.half_extents.x * self.half_extents.y * self.half_extents.z
    }

    /// Whether two blocks touch, and so hold each other up.
    ///
    /// Two answers in one: blocks of the same course are neighbours when the
    /// gap between their ends is a joint rather than a doorway, and blocks of
    /// consecutive courses are neighbours when one sits over the other.
    fn adjoins(&self, other: &DomeBlock) -> bool {
        let reach = self.arc.half_angle + other.arc.half_angle;
        let separation = signed_azimuth(other.arc.centre - self.arc.centre).abs();

        match self.ring.abs_diff(other.ring) {
            0 => separation - reach < reach * ADJACENCY_SLACK,
            1 => separation < reach,
            _ => false,
        }
    }

    /// The highest point of the block, which is what the next thing stacked on
    /// the dome has to clear.
    fn top(&self) -> f32 {
        self.centre.y + self.vertical_reach()
    }

    /// The lowest point of the block: where the dome meets the ground.
    #[cfg(test)]
    fn bottom(&self) -> f32 {
        self.centre.y - self.vertical_reach()
    }

    /// How far the block reaches above and below its centre once turned.
    fn vertical_reach(&self) -> f32 {
        let axes = self.rotation.to_rotation_matrix();
        (0..3)
            .map(|axis| (axes[(1, axis)] * self.half_extents[axis]).abs())
            .sum()
    }
}

/// A point on the dome's mid-surface.
fn surface_point(radius: f32, elevation: f32, azimuth: f32) -> Point3<f32> {
    Point3::new(
        radius * elevation.cos() * azimuth.cos(),
        radius * elevation.sin(),
        radius * elevation.cos() * azimuth.sin(),
    )
}

/// The orientation of a block lying on the dome at this point: local `+Z` out
/// through the wall, local `+Y` up the meridian, local `+X` along the course.
fn surface_frame(elevation: f32, azimuth: f32) -> UnitQuaternion<f32> {
    let outward = Vector3::new(
        elevation.cos() * azimuth.cos(),
        elevation.sin(),
        elevation.cos() * azimuth.sin(),
    );
    let up_the_meridian = Vector3::new(
        -elevation.sin() * azimuth.cos(),
        elevation.cos(),
        -elevation.sin() * azimuth.sin(),
    );
    let along_the_course = up_the_meridian.cross(&outward);

    UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(Matrix3::from_columns(
        &[along_the_course, up_the_meridian, outward],
    )))
}

/// An azimuth folded into `(−π, π]`, so that "how far round from the doorway"
/// is a distance rather than a wrap-around.
fn signed_azimuth(azimuth: f32) -> f32 {
    let wrapped = azimuth.rem_euclid(TAU);
    if wrapped > PI {
        wrapped - TAU
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{PhysicsConfig, PhysicsWorld};
    use crate::rendering::material::MaterialId;

    pub fn igloo() -> IglooDef {
        IglooDef {
            pos: (0.0, 0.0, 0.0),
            radius: IglooDef::default_radius(),
            wall_thickness: IglooDef::default_wall_thickness(),
            block_height: IglooDef::default_block_height(),
            door_width: IglooDef::default_door_width(),
            door_height: IglooDef::default_door_height(),
            yaw: 0.0,
            fracture_threshold: IglooDef::default_fracture_threshold(),
            cleave_threshold: IglooDef::default_cleave_threshold(),
        }
    }

    /// The frame must be a rotation — orthonormal and right-handed — or every
    /// block it orients is sheared, which a quaternion cannot represent and
    /// which the collider would not agree with anyway.
    #[test]
    fn the_surface_frame_is_a_rotation() {
        for elevation in [0.0_f32, 0.3, 0.9, 1.5] {
            for azimuth in [0.0_f32, 1.0, 3.0, 5.5] {
                let axes = surface_frame(elevation, azimuth).to_rotation_matrix();
                let product = axes.matrix().transpose() * axes.matrix();
                assert!(
                    (product - Matrix3::identity()).norm() < 1e-5,
                    "elevation {elevation}, azimuth {azimuth}"
                );
            }
        }
    }

    /// A block's outward face must actually face outwards, and its top must
    /// lean inwards as the dome closes — the two things that make the courses
    /// corbel rather than stand as a cylinder.
    #[test]
    fn blocks_lie_along_the_dome() {
        let frame = surface_frame(0.6, 0.0).to_rotation_matrix();
        let outward = frame * Vector3::z();
        assert!(outward.x > 0.0, "{outward:?}");
        assert!(outward.y > 0.0, "{outward:?}");

        let up = frame * Vector3::y();
        assert!(up.y > 0.0, "{up:?}");
        assert!(up.x < 0.0, "top leans outwards: {up:?}");
    }

    /// The doorway must come out the width it was authored, not the width the
    /// block grid happens to round it to.
    #[test]
    fn the_doorway_is_cut_to_the_width_it_was_asked_for() {
        let igloo = igloo();
        let radius = igloo.mid_radius();
        let jamb = (igloo.door_width * 0.5 / radius).asin();
        let count = 17;
        let half_angle = PI / count as f32;

        let kept: Vec<Arc> = (0..count)
            .map(|index| Arc {
                centre: TAU * index as f32 / count as f32,
                half_angle,
            })
            .filter_map(|arc| igloo.doorway_cuts(arc, radius, 0.0))
            .collect();

        let mut nearest = f32::MAX;
        for arc in &kept {
            let near = signed_azimuth(arc.centre - arc.half_angle);
            let far = signed_azimuth(arc.centre + arc.half_angle);
            if near > far {
                // Wraps round the back, nowhere near the door.
                continue;
            }
            assert!(
                near >= jamb - 1e-4 || far <= -jamb + 1e-4,
                "an arc spanning {near}..{far} stands in a doorway of ±{jamb}"
            );
            nearest = nearest.min(near.abs().min(far.abs()));
        }
        assert!(
            (nearest - jamb).abs() < 1e-4,
            "the doorway opened to ±{nearest} instead of ±{jamb}"
        );
    }

    /// And the dome must actually lose blocks to it. A trim rule that trims
    /// nothing would pass every property above.
    #[test]
    fn the_doorway_takes_blocks_out_of_the_dome() {
        let mut sealed = igloo();
        sealed.door_width = 0.0;
        assert!(igloo().blocks().len() < sealed.blocks().len());
    }

    /// The cap must sit on the rim, not in it: blended geometry that
    /// interpenetrates composites wrong, and here it would do so at the one
    /// place every silhouette of the igloo passes through.
    #[test]
    fn the_cap_rests_on_the_rim() {
        let blocks = igloo().blocks();
        let cap = blocks.last().expect("an igloo has a cap");
        let rim = blocks[..blocks.len() - 1]
            .iter()
            .map(DomeBlock::top)
            .fold(0.0_f32, f32::max);

        let underside = cap.centre.y - cap.half_extents.y;
        assert!(
            (underside - rim).abs() < 1e-4,
            "cap underside {underside}, rim {rim}"
        );
    }

    /// Every block of the dome must actually be breakable, from wherever it
    /// is struck. The hull cutter refuses a cut that would leave a sliver
    /// and the rule tries again elsewhere, so a block whose proportions
    /// leave nowhere good to cut would simply never break — and a grenade in
    /// an igloo would knock blocks loose and never shatter one.
    #[test]
    fn every_block_of_the_dome_can_be_cleaved() {
        let rule = ice_cleaving();
        let mut refused = Vec::new();
        for (index, block) in igloo().blocks().iter().enumerate() {
            let shape = crate::physics::ColliderShape::Box {
                half_extents: block.half_extents,
            };
            // Struck on a face, on an edge, and dead centre.
            for (salt, hit) in [
                block
                    .half_extents
                    .component_mul(&Vector3::new(0.9, 0.0, 0.0)),
                block
                    .half_extents
                    .component_mul(&Vector3::new(0.8, 0.8, 0.0)),
                Vector3::zeros(),
            ]
            .into_iter()
            .enumerate()
            {
                if rule.cleave(&shape, hit, salt as u32).is_none() {
                    refused.push((index, block.half_extents, hit));
                }
            }
        }
        assert!(
            refused.is_empty(),
            "{} of the dome's blocks would not break: {:?}",
            refused.len(),
            &refused[..refused.len().min(3)]
        );
    }

    /// A dome of a few hundred pieces is a level's whole budget spent on one
    /// prop. The default igloo must stay well under that.
    #[test]
    fn the_default_igloo_is_a_prop_not_a_level() {
        let count = igloo().blocks().len();
        assert!((40..=140).contains(&count), "{count} blocks");
    }

    /// Every block must be joined to the rest, or the dome starts the level
    /// already in pieces: the fracture system splits off anything that is not
    /// connected to the largest group the first time it is disturbed.
    #[test]
    fn the_dome_is_one_connected_structure() {
        let igloo = igloo();
        let blocks = igloo.blocks();
        let fracture = CompoundFracture::boxes(igloo.joints(&blocks), blocks.len(), MaterialId(0));

        let components = fracture.connected_components();
        assert_eq!(
            components.len(),
            1,
            "the dome falls into {} pieces before anything touches it",
            components.len()
        );
    }

    /// The doorway must not be bridged. Two blocks either side of it are not
    /// holding each other up, and a joint saying they are would keep the
    /// jambs standing after the wall between them had gone.
    #[test]
    fn nothing_reaches_across_the_doorway() {
        let igloo = igloo();
        let blocks = igloo.blocks();

        for joint in igloo.joints(&blocks) {
            let (a, b) = (&blocks[joint.child_a], &blocks[joint.child_b]);
            if a.ring != b.ring || a.centre.y > igloo.door_height {
                continue;
            }
            let gap = signed_azimuth(b.arc.centre - a.arc.centre).abs()
                - a.arc.half_angle
                - b.arc.half_angle;
            assert!(
                gap * a.centre.coords.xz().norm() < igloo.door_width * 0.5,
                "a joint spans a gap of {gap} rad at door height"
            );
        }
    }

    /// The igloo must stand on the floor it was authored at: buried in it and
    /// the solver spends the level pushing it out, floating above it and it
    /// drops the moment the level starts.
    #[test]
    fn the_dome_stands_on_its_authored_floor() {
        let foot = igloo()
            .blocks()
            .iter()
            .map(DomeBlock::bottom)
            .fold(f32::MAX, f32::min);
        assert!(foot.abs() < 0.05, "the dome's lowest block sits at {foot}");
    }

    /// The whole point of the dome being one body: dropped on a floor it
    /// stands there, rather than sinking into it or squirming.
    ///
    /// Ice grips almost nothing — a coefficient of 0.06 — so a dome of loose
    /// blocks slides itself apart, and this is the test that says the object
    /// in the level is not that.
    #[test]
    fn the_dome_stands_where_it_was_put() {
        const DT: f32 = 1.0 / 60.0;

        let blocks = igloo().blocks();
        let origin = mass_centre(&blocks);

        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic().position(origin));
        for block in &blocks {
            world.attach_collider(
                body,
                ColliderDesc::box_shape(block.half_extents)
                    .of(&ice())
                    .offset_translation(block.centre - origin)
                    .offset_rotation(block.rotation),
            );
        }
        world.wake_body(body);

        let floor = FlatQuadGeometry::new(20.0);
        let mut debug = DebugLines::default();
        for _ in 0..120 {
            world.update_contacts(DT, 1, &floor, &[], &mut debug);
            world.substep(DT, &floor, &[]);
        }

        let settled = world.body(body).expect("the igloo").position();
        assert!(
            (settled.y - origin.y).abs() < 0.05,
            "the dome moved {:.3} m vertically",
            settled.y - origin.y
        );
        assert!(
            (settled - origin).xz().norm() < 0.05,
            "the dome slid {:.3} m sideways",
            (settled - origin).xz().norm()
        );
    }

    /// The body origin has to be the centre of mass. Anywhere else and the
    /// dome pivots about a phantom point when the ground goes out from under
    /// it — the failure the fracture system re-centres to avoid.
    #[test]
    fn the_origin_is_the_centre_of_mass() {
        let blocks = igloo().blocks();
        let centre = mass_centre(&blocks);

        let mut residual = Vector3::zeros();
        let mut volume = 0.0;
        for block in &blocks {
            residual += (block.centre - centre) * block.volume();
            volume += block.volume();
        }
        assert!((residual / volume).norm() < 1e-4, "{residual:?}");

        // And it is inside the dome, above the floor: a sanity check that the
        // weighting is by volume and not by block count.
        assert!(centre.y > 0.0 && centre.y < igloo().radius);
    }
}

#[cfg(test)]
mod contact_fracture {
    use super::tests::igloo;
    use super::*;
    use crate::debug::DebugLines;
    use crate::fracture::ContactLoadTracker;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};

    const FRAME_DT: f32 = 1.0 / 60.0;

    /// The dome as the fracture system sees it: one compound body of 88 ice
    /// blocks, on a flat quad, with sleep off so that a body standing still is
    /// actually solved rather than silently reporting nothing.
    fn largest_spike(height: f32, velocity: Vector3<f32>, frames: usize) -> f32 {
        let def = igloo();
        let blocks = def.blocks();
        let centre = mass_centre(&blocks);
        let substance = ice();

        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);

        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, height, 0.0))
                .linear_velocity(velocity)
                .linear_damping(0.01)
                .angular_damping(0.005),
        );
        let colliders: Vec<_> = blocks
            .iter()
            .map(|block| {
                world
                    .attach_collider(
                        body,
                        ColliderDesc::box_shape(block.half_extents)
                            .of(&substance)
                            .offset_translation(block.centre - centre)
                            .offset_rotation(block.rotation),
                    )
                    .expect("collider attaches to a live body")
            })
            .collect();

        let geometry = FlatQuadGeometry::new(80.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let mut debug = DebugLines::default();
        let mut tracker = ContactLoadTracker::new(colliders.len());
        let mut peak = 0.0f32;

        for _ in 0..frames {
            stepper.step(&mut world, FRAME_DT, &geometry, &[], &[], &mut debug);
            let spikes = tracker.advance(&world, body, &colliders, FRAME_DT);
            peak = peak.max(spikes.iter().fold(0.0f32, |m, s| m.max(s.magnitude)));
        }
        peak
    }

    /// The height a dome placed on the ground comes to rest at, since the body
    /// origin sits on the centre of mass rather than the floor.
    fn resting_height() -> f32 {
        mass_centre(&igloo().blocks()).y
    }

    /// An igloo that is standing there, or being shoved along the ground, must
    /// not be taking itself apart. Eight tonnes of ice in 88 pieces never
    /// reaches a perfectly static equilibrium — the solver keeps shuffling the
    /// load between blocks — and that shuffling must stay below the threshold
    /// the dome's joints are authored with.
    #[test]
    fn an_igloo_left_alone_does_not_shake_itself_apart() {
        let threshold = igloo().fracture_threshold;

        let standing = largest_spike(resting_height(), Vector3::zeros(), 300);
        assert!(
            standing < threshold,
            "a dome standing still spikes {standing}, past its own threshold {threshold}"
        );

        let shoved = largest_spike(resting_height(), Vector3::new(2.0, 0.0, 0.0), 300);
        assert!(
            shoved < threshold,
            "a dome being pushed spikes {shoved}, past its own threshold {threshold}"
        );
    }

    /// And the point of the exercise: shove it off a hill and it shatters.
    /// The margin wants to be wide, not marginal — a hole is opened wherever a
    /// child clears the threshold, so a hillside impact should clear it by
    /// enough that the dome comes apart properly (measured ~24900 N·s against a
    /// 420 N·s threshold).
    #[test]
    fn an_igloo_thrown_down_a_hill_shatters_on_impact() {
        let threshold = igloo().fracture_threshold;
        let impact = largest_spike(6.0, Vector3::new(6.0, -4.0, 0.0), 300);

        assert!(
            impact > threshold * 10.0,
            "a hillside impact spikes only {impact} against threshold {threshold}"
        );
    }
}
