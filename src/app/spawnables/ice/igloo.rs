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
//! **The blocks are fixed, not loose.** Ice has a friction coefficient of
//! 0.06, the lowest of any substance in the library, so a corbelled dome of it
//! has nothing whatsoever holding it up: the moment the simulation starts, an
//! igloo of dynamic blocks slumps into a puddle of bricks. A standing igloo is
//! a piece of the world, and the slipperiness is something the player meets on
//! the outside of it.

use std::f32::consts::{PI, TAU};

use nalgebra::{Matrix3, Point3, Rotation3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::super::shared::orientation::Yaw;
use super::super::shared::textures::hash_pair;
use super::super::{MaterialCtx, Spawnable};
use super::block::{ice_materials, Anchorage, IceBlock};
use crate::core::error::EngineResult;
use crate::rendering::material::MaterialId;

/// How many distinct ice textures a dome's blocks are drawn from.
const TEXTURE_VARIANTS: usize = 4;

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
                    variant: hash_pair(ring as i32, index as i32) as usize,
                });
            }

            open_radius = mid * ((ring + 1) as f32 * ring_arc).cos();
        }

        blocks.push(self.cap(&blocks, open_radius, half_thickness));
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
    fn cap(&self, blocks: &[DomeBlock], open_radius: f32, half_thickness: f32) -> DomeBlock {
        let rim = blocks
            .iter()
            .map(DomeBlock::top)
            .fold(0.0_f32, |acc, top| acc.max(top));
        let half_width = open_radius + half_thickness;

        DomeBlock {
            centre: Point3::new(0.0, rim + half_thickness, 0.0),
            half_extents: Vector3::new(half_width, half_thickness, half_width),
            rotation: UnitQuaternion::identity(),
            variant: blocks.len(),
        }
    }
}

impl Spawnable for IglooDef {
    fn material_count(&self) -> usize {
        TEXTURE_VARIANTS
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        ice_materials(ctx, self.pos, TEXTURE_VARIANTS)
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let yaw = Yaw::degrees(self.yaw);

        self.blocks()
            .into_iter()
            .map(|block| {
                let centre = yaw.place(self.pos, block.centre.coords);
                IceBlock::new(
                    centre,
                    block.half_extents,
                    materials[block.variant % materials.len()],
                )
                .rotated(yaw.rotation() * block.rotation)
                .anchored(Anchorage::Fixed)
                .spawn(world)
            })
            .collect()
    }
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
    variant: usize,
}

impl DomeBlock {
    /// The highest point of the block, which is what the next thing stacked on
    /// the dome has to clear.
    fn top(&self) -> f32 {
        let axes = self.rotation.to_rotation_matrix();
        let reach: f32 = (0..3)
            .map(|axis| (axes[(1, axis)] * self.half_extents[axis]).abs())
            .sum();
        self.centre.y + reach
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

    fn igloo() -> IglooDef {
        IglooDef {
            pos: (0.0, 0.0, 0.0),
            radius: IglooDef::default_radius(),
            wall_thickness: IglooDef::default_wall_thickness(),
            block_height: IglooDef::default_block_height(),
            door_width: IglooDef::default_door_width(),
            door_height: IglooDef::default_door_height(),
            yaw: 0.0,
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

    /// A dome of a few hundred bodies is a level's whole budget spent on one
    /// prop. The default igloo must stay well under that.
    #[test]
    fn the_default_igloo_is_a_prop_not_a_level() {
        let count = igloo().blocks().len();
        assert!((40..=140).contains(&count), "{count} blocks");
    }
}
