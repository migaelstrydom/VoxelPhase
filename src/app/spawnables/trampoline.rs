//! Trampoline spawnable: a round bed on a ring of coil springs, sprung from a
//! glossy hoop on splayed chrome legs.
//!
//! Two bodies. The frame — hoop, legs and feet — stands on the ground; the
//! bed is a light disc held to it by a `SpringSlider`, rigid in every
//! direction but up and down. Whatever lands on the bed drives it down, the
//! spring throws it back, and a jump at the bottom of the dip pushes the bed
//! deeper and comes back higher. Nothing is bouncy by restitution.
//!
//! The frame is drawn as a model; the bed and springs are redrawn every frame
//! by `trampoline::TrampolineBedSystem`, dipping and stretching as the bed
//! rides.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::orientation::Yaw;
use super::shared::textures::{hue_to_rgb, seed_from_position, Rgb, TextureRng};
use super::spawnable::{MaterialCtx, Spawnable};
use crate::collision::convex_hull::prism_hull;
use crate::components::{
    ModelInstance, Orientation, Position, ProceduralMesh, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices, generate_tube};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{
    ColliderDesc, ConstraintKind, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, Spring,
    SpringSlider,
};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::trampoline::{Trampoline, TrampolineShape, STEEL};

/// Side of the bed's fabric texture, in pixels.
const TEXTURE_SIZE: u32 = 256;

/// The load the spring is tuned against: about what the player weighs.
const DESIGN_LOAD_KG: f32 = 57.0;
/// How fast a jumper of [`DESIGN_LOAD_KG`] bobs on the bed. Stiff enough that
/// a landing from a few metres stays clear of the ground, soft enough that
/// the bed visibly takes it.
const BOUNCE_HZ: f32 = 2.2;
/// The spring's damping ratio: low, so a bounce keeps most of its height.
const BOUNCE_DAMPING: f32 = 0.03;

/// Density of the frame's colliders, which are solid stand-ins for tubes.
const FRAME_DENSITY: f32 = 120.0;
/// Mass of the bed: a few kilograms of fabric, light beside a jumper so a
/// landing gives little of its energy to the bed itself.
const BED_MASS_KG: f32 = 3.5;
/// Grip of the bed's fabric.
const BED_FRICTION: f32 = 0.9;
/// Facets round the bed's collider.
const BED_SIDES: u16 = 16;
/// Box colliders round the hoop.
const HOOP_SEGMENTS: u32 = 24;

#[derive(Deserialize)]
pub struct TrampolineDef {
    /// Ground point under the trampoline's centre: where its feet stand.
    pub pos: (f32, f32, f32),
    /// Rotation about `+Y`, in degrees. A rotated segment adds its own yaw.
    #[serde(default)]
    pub yaw: f32,
    /// Radius of the hoop's centreline, in metres.
    #[serde(default = "TrampolineDef::default_radius")]
    pub radius: f32,
}

impl TrampolineDef {
    pub fn default_radius() -> f32 {
        1.8
    }

    /// The hue the hoop and the bed's markings share, and a seed for the
    /// fabric's grain: chosen from the position, so a trampoline keeps its
    /// colours from one load to the next.
    fn palette(&self) -> (f32, u32) {
        let rng = &mut TextureRng::new(seed_from_position(self.pos, 0));
        (rng.range(0.0, 6.0), rng.u32())
    }

    /// The hoop's glossy plastic.
    fn plastic(&self) -> Colour {
        let rgb = hue_to_rgb(self.palette().0, 0.75, 0.95);
        Colour::rgb(rgb.r, rgb.g, rgb.b)
    }

    fn shape(&self) -> TrampolineShape {
        TrampolineShape::with_radius(self.radius)
    }

    /// The trampoline's ground frame in the world.
    fn ground_frame(&self) -> Isometry3<f32> {
        Isometry3::from_parts(
            Vector3::new(self.pos.0, self.pos.1, self.pos.2).into(),
            Yaw::degrees(self.yaw).rotation(),
        )
    }
}

impl Spawnable for TrampolineDef {
    fn material_count(&self) -> usize {
        3
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // [0] = bed fabric, [1] = chrome (legs, springs), [2] = hoop plastic
        let (hue, seed) = self.palette();
        let fabric_pixels = generate_bed_fabric(hue, seed);
        let fabric_texture =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &fabric_pixels, true)?;
        let fabric =
            ctx.materials.register(
                Material::textured(fabric_texture).with_finish(SurfaceFinish {
                    roughness: 0.6,
                    metallic: 0.0,
                }),
            );
        // Both colours ride on the frame's vertices.
        let chrome = ctx
            .materials
            .register(Material::coloured(Colour::WHITE).with_finish(SurfaceFinish::METAL));
        let hoop = ctx
            .materials
            .register(Material::coloured(Colour::WHITE).with_finish(SurfaceFinish::POLISHED));
        Ok(vec![fabric, chrome, hoop])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let shape = self.shape();
        let ground = self.ground_frame();
        let (fabric, chrome, plastic) = (materials[0], materials[1], materials[2]);

        let (frame, bed, ground_to_body) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            build_bodies(&mut physics.world, &shape, &ground)
        };

        let model = frame_model(&shape, ground_to_body, chrome, (plastic, self.plastic()));
        let trampoline = Trampoline {
            frame,
            bed,
            shape,
            ground_to_body,
            bed_material: fabric,
            spring_material: chrome,
        };
        // Drawn at rest until `TrampolineBedSystem` first runs, so a view
        // of the level as authored shows the bed.
        let at_rest = ProceduralMesh {
            primitives: trampoline.bed_primitives(0.0),
        };
        let frame_entity = body_entity(world, frame)
            .with(ModelInstance::new(model))
            .with(at_rest)
            .with(trampoline)
            .with(Renderable)
            .build();
        let bed_entity = body_entity(world, bed).build();
        vec![frame_entity, bed_entity]
    }
}

/// The trampoline's two bodies and the spring between them, its ground frame
/// at `ground`. Returns the frame, the bed, and what moves a point in the
/// ground frame into the frame body's own.
pub(crate) fn build_bodies(
    world: &mut PhysicsWorld,
    shape: &TrampolineShape,
    ground: &Isometry3<f32>,
) -> (RigidBodyHandle, RigidBodyHandle, Vector3<f32>) {
    // Laid out in the ground frame, then re-centred: a body's origin is its
    // centre of mass, which sits up among the hoop.
    let frame = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::from(ground.translation.vector))
            .rotation(ground.rotation),
    );
    for collider in frame_colliders(shape) {
        world.attach_collider(frame, collider);
    }
    let ground_to_body = -world.recenter_on_colliders(frame);

    let bed = world.create_body(
        RigidBodyDesc::dynamic()
            .position(ground * shape.bed_rest_centre())
            .rotation(ground.rotation),
    );
    let hull = prism_hull(
        BED_SIDES,
        shape.bed_collider_radius,
        shape.bed_thickness * 0.5,
    );
    let density = BED_MASS_KG / hull.compute_volume();
    world.attach_collider(
        bed,
        ColliderDesc::convex_hull(Arc::new(hull))
            .density(density)
            .restitution(0.0)
            .friction(BED_FRICTION),
    );

    world.create_constraint(ConstraintKind::SpringSlider(SpringSlider {
        body_a: Some(frame),
        body_b: bed,
        local_anchor_a: shape.bed_rest_centre().coords + ground_to_body,
        local_anchor_b: Vector3::zeros(),
        local_axis_a: Vector3::y_axis(),
        reference: UnitQuaternion::identity(),
        spring: Spring::from_hz(BOUNCE_HZ, BOUNCE_DAMPING),
        reference_mass: DESIGN_LOAD_KG,
    }));
    (frame, bed, ground_to_body)
}

/// An entity tracking `body`, posed where the body is.
fn body_entity(world: &mut World, body: RigidBodyHandle) -> specs::EntityBuilder<'_> {
    let (position, rotation) = {
        let physics = world.read_resource::<PhysicsResource>();
        let body = physics.world.body(body).expect("the body was just made");
        (body.position(), body.rotation())
    };
    world
        .create_entity()
        .with(Position(position.coords))
        .with(Velocity(Vector3::zeros()))
        .with(Orientation(rotation))
        .with(RigidBodyComponent(body))
}

/// The frame's colliders in its ground frame: capsules round the hoop and
/// along each run of each leg, and a ball on each foot. Capsules because the
/// rods are round: a box's corners would reach in toward the bed's edge and
/// hold it.
fn frame_colliders(shape: &TrampolineShape) -> Vec<ColliderDesc> {
    let mut colliders = Vec::new();
    let hoop = shape.hoop_path(HOOP_SEGMENTS);
    for (i, start) in hoop.iter().enumerate() {
        let end = hoop[(i + 1) % hoop.len()];
        colliders.push(rod(*start, end, shape.hoop_tube_radius));
    }
    for leg in 0..shape.leg_count {
        let path = shape.leg_path(leg);
        for run in path.windows(2) {
            colliders.push(rod(run[0], run[1], shape.leg_tube_radius));
        }
        let foot = *path.last().expect("a leg has a foot");
        colliders.push(
            ColliderDesc::sphere(shape.foot_radius)
                .offset_translation(foot.coords)
                .density(FRAME_DENSITY)
                .friction(0.8),
        );
    }
    colliders
}

/// A straight tube of `radius` from `start` to `end`.
fn rod(start: Point3<f32>, end: Point3<f32>, radius: f32) -> ColliderDesc {
    let run = end - start;
    let rotation = UnitQuaternion::rotation_between(&Vector3::y(), &run)
        .unwrap_or_else(UnitQuaternion::identity);
    ColliderDesc::capsule(run.norm() * 0.5, radius)
        .offset_translation(nalgebra::center(&start, &end).coords)
        .offset_rotation(rotation)
        .density(FRAME_DENSITY)
        .friction(0.6)
}

/// The frame as drawn: a glossy hoop with balls on its feet, chrome legs.
/// Built in the ground frame and moved by `ground_to_body` into the body's.
fn frame_model(
    shape: &TrampolineShape,
    ground_to_body: Vector3<f32>,
    chrome: MaterialId,
    (plastic, plastic_colour): (MaterialId, Colour),
) -> Arc<Model> {
    let shifted = |mut vertices: Vec<Vertex>| {
        for vertex in &mut vertices {
            vertex.pos += ground_to_body;
        }
        vertices
    };

    let (hoop_vertices, hoop_indices) = generate_tube(
        &shape.hoop_path(96),
        shape.hoop_tube_radius,
        14,
        true,
        plastic_colour,
    );
    let mut plastic_part = MeshPrimitive {
        vertices: shifted(hoop_vertices),
        indices: hoop_indices,
        material: plastic,
    };

    let mut legs = MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material: chrome,
    };
    const FOOT_SEGMENTS: u32 = 16;
    const FOOT_RINGS: u32 = 10;
    let foot_indices = generate_sphere_indices(FOOT_SEGMENTS, FOOT_RINGS);
    for leg in 0..shape.leg_count {
        let path = shape.leg_path(leg);
        let (vertices, indices) = generate_tube(&path, shape.leg_tube_radius, 10, false, STEEL);
        append(&mut legs, shifted(vertices), &indices);

        let foot = path.last().expect("a leg has a foot").coords + ground_to_body;
        let mut ball =
            generate_sphere_vertices(shape.foot_radius, FOOT_SEGMENTS, FOOT_RINGS, plastic_colour);
        for vertex in &mut ball {
            vertex.pos += foot;
        }
        append(&mut plastic_part, ball, &foot_indices);
    }

    Arc::new(Model::flat(vec![
        ModelPart::new(vec![plastic_part]),
        ModelPart::new(vec![legs]),
    ]))
}

/// Append a mesh to a primitive, re-basing its indices.
fn append(primitive: &mut MeshPrimitive, vertices: Vec<Vertex>, indices: &[u32]) {
    let base = primitive.vertices.len() as u32;
    primitive.vertices.extend(vertices);
    primitive
        .indices
        .extend(indices.iter().map(|index| index + base));
}

// ---------------------------------------------------------------------------
// Procedural bed fabric
// ---------------------------------------------------------------------------

/// The bed's fabric over its disc: a dark woven mat with a bright ring round
/// the sweet spot, a mark at the centre, and a hem in the hoop's colour.
/// `hue` is the hoop's.
fn generate_bed_fabric(hue: f32, seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let mat = Rgb::new(0.05, 0.06, 0.09);
    let accent = hue_to_rgb(hue, 0.7, 1.0);
    let hem = hue_to_rgb(hue, 0.8, 0.75);

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;
            let (cx, cy) = (u * 2.0 - 1.0, v * 2.0 - 1.0);
            let r = (cx * cx + cy * cy).sqrt();

            // A tight basket weave: alternate threads catch the light.
            let (tu, tv) = (u * size as f32 * 0.5, v * size as f32 * 0.5);
            let over = ((tu.floor() + tv.floor()) as i32 & 1) == 0;
            let thread = if over {
                0.5 + 0.5 * (tu.fract() * std::f32::consts::PI).sin()
            } else {
                0.5 + 0.5 * (tv.fract() * std::f32::consts::PI).sin()
            };
            let grain =
                crate::utils::noise::fbm_2d_periodic(u * 8.0, v * 8.0, 2, 0.5, 2.0, seed, Some(8));
            let mut c = mat.scale(0.75 + thread * 0.4 + grain * 0.15);

            let band = |inner: f32, outer: f32| {
                let soft = 1.5 / size as f32;
                ((r - inner) / soft).clamp(0.0, 1.0) * ((outer - r) / soft).clamp(0.0, 1.0)
            };
            c = c.lerp(accent, band(0.48, 0.53) * 0.9);
            c = c.lerp(accent, band(0.0, 0.07) * 0.9);
            c = c.lerp(hem, band(0.93, 1.2));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
