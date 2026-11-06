//! Plank bridge spawnable — rustic destructible bridge.
//!
//! Two long beams (stringers) run end to end. Cross-planks are laid across
//! them haphazardly: slightly rotated, varying widths, with small gaps.
//! The whole structure is a single compound body, a part per beam and per
//! plank, every plank bonded to both beams where it crosses them, so it
//! shatters on explosion.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::cuboid_look;
use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::shared::textures::TextureRng;
use super::shared::textures::*;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{Bond, CompoundFracture, PartGraph};
use crate::model::PartLooks;
use crate::physics::{ColliderDesc, ColliderHandle, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct PlankBridgeDef {
    /// World position of the bridge center (bottom of the beams).
    pub pos: (f32, f32, f32),
    /// Total length of the bridge along the Z axis.
    #[serde(default = "PlankBridgeDef::default_length")]
    pub length: f32,
    /// Lateral spacing between the two beam centers.
    #[serde(default = "PlankBridgeDef::default_beam_spacing")]
    pub beam_spacing: f32,
    /// Number of cross-planks.
    #[serde(default = "PlankBridgeDef::default_plank_count")]
    pub plank_count: u32,
    /// Half-extents of each stringer beam (x, y, z).
    #[serde(default = "PlankBridgeDef::default_beam_half_extents")]
    pub beam_half_extents: (f32, f32, f32),
    /// Half-extents of each cross-plank (x, y, z). X spans the bridge width.
    #[serde(default = "PlankBridgeDef::default_plank_half_extents")]
    pub plank_half_extents: (f32, f32, f32),
    #[serde(default = "PlankBridgeDef::default_density")]
    pub density: f32,
    /// Rotation about `+Y`, in degrees. 0 = the bridge runs along Z.
    ///
    /// Degrees rather than radians so it matches every other yaw in the level
    /// format — anchors, placements and the other oriented spawnables.
    #[serde(default)]
    pub yaw: f32,
    /// Blast loading, in N·s/m², that breaks a plank off. Small: a grenade
    /// anywhere near this bridge should take it apart.
    #[serde(default = "PlankBridgeDef::default_fracture_threshold")]
    pub fracture_threshold: f32,
    /// Contact spike, in N·s, that breaks a plank off.
    ///
    /// Three orders of magnitude above the blast threshold because the two
    /// measure different things: 2.7 tonnes of bridge settling five
    /// centimetres already pushes 1143 N·s through a bond, which is nothing
    /// a grenade could do.
    #[serde(default = "PlankBridgeDef::default_contact_threshold")]
    pub contact_threshold: f32,
}

impl PlankBridgeDef {
    pub fn default_length() -> f32 {
        8.0
    }
    pub fn default_beam_spacing() -> f32 {
        1.6
    }
    pub fn default_plank_count() -> u32 {
        8
    }
    pub fn default_beam_half_extents() -> (f32, f32, f32) {
        (0.18, 0.16, 4.0)
    }
    pub fn default_plank_half_extents() -> (f32, f32, f32) {
        (1.0, 0.12, 0.38)
    }
    pub fn default_density() -> f32 {
        500.0
    }
    pub fn default_fracture_threshold() -> f32 {
        8.0
    }

    /// Measured on the default bridge: it holds through a five-centimetre
    /// settle (1143 N·s at a joint) and sheds planks from a thirty-centimetre
    /// drop (3442 N·s). See `probe` below, which pins both.
    pub fn default_contact_threshold() -> f32 {
        2000.0
    }

    fn beam_he(&self) -> Vector3<f32> {
        // Override beam Z to half the bridge length.
        Vector3::new(
            self.beam_half_extents.0,
            self.beam_half_extents.1,
            self.length / 2.0,
        )
    }

    fn plank_he(&self) -> Vector3<f32> {
        // Override plank X to span beam spacing + overhang.
        let span_x = self.beam_spacing / 2.0 + self.beam_half_extents.0 + 0.25;
        Vector3::new(span_x, self.plank_half_extents.1, self.plank_half_extents.2)
    }

    /// The bridge's parts: each beam and each plank, every plank bonded to
    /// both beams where it crosses them, on top of the beam.
    ///
    /// Each collider comes with its offset in the body's frame, which is the
    /// frame bond anchors are held in.
    fn graph(
        &self,
        beams: &[(ColliderHandle, Vector3<f32>)],
        planks: &[(ColliderHandle, Vector3<f32>)],
    ) -> PartGraph {
        let mut graph = PartGraph::default();
        let beams: Vec<_> = beams
            .iter()
            .map(|(beam, at)| (graph.add_part([*beam]), *at))
            .collect();
        let reach = 2.0 * self.plank_he().norm();
        let beam_top = self.beam_he().y;
        for (plank, plank_at) in planks {
            let plank = graph.add_part([*plank]);
            for (beam, beam_at) in &beams {
                graph.bond(Bond {
                    a: *beam,
                    b: plank,
                    anchor: Point3::new(beam_at.x, beam_at.y + beam_top, plank_at.z),
                    reach,
                    threshold: self.fracture_threshold,
                    toughness: None,
                });
            }
        }
        graph
    }

    /// The one declaration of the bridge's timber physics. Beams and planks
    /// share it, so the collider and the wood they're rendered with cannot
    /// disagree about how bouncy or grippy this bridge is.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: RESTITUTION,
            friction: FRICTION,
            density: self.density,
        }
    }
}

/// Restitution of the bridge's timber colliders. Wood, not rubber.
const RESTITUTION: f32 = 0.05;
/// Friction of the bridge's timber colliders.
const FRICTION: f32 = 0.7;

impl Spawnable for PlankBridgeDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let beam_pixels = generate_beam_wood(seed_from_position(self.pos, 0));
        let beam_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &beam_pixels, true)?;
        let beam_mat = ctx
            .materials
            .register(Material::textured(beam_tex).with_derived_finish(self.surface()));

        let plank_pixels = generate_plank_wood(seed_from_position(self.pos, 1));
        let plank_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &plank_pixels, true)?;
        let plank_mat = ctx
            .materials
            .register(Material::textured(plank_tex).with_derived_finish(self.surface()));

        Ok(vec![beam_mat, plank_mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let center = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let beam_he = self.beam_he();
        let plank_he = self.plank_he();
        let beam_mat = materials[0];
        let plank_mat = materials[1];

        let beam_y = beam_he.y;
        let plank_y = beam_he.y * 2.0 + plank_he.y;
        let half_spacing = self.beam_spacing / 2.0;

        // Child 0 = left beam, child 1 = right beam.
        let beam_offsets = [
            Vector3::new(-half_spacing, beam_y, 0.0),
            Vector3::new(half_spacing, beam_y, 0.0),
        ];

        // Generate haphazard plank placements.
        let usable_length = self.length - plank_he.z * 2.0;
        let base_spacing = usable_length / (self.plank_count as f32);
        let start_z = -usable_length / 2.0;

        let seed = seed_from_position(self.pos, 2);
        let min_gap = 0.02;

        // Pre-compute per-plank randomised properties.
        struct PlankLayout {
            z: f32,
            half_w: f32,
            yaw: f32,
            /// Conservative Z half-footprint accounting for yaw rotation.
            z_footprint: f32,
            hash: u32,
        }

        let mut raw_planks: Vec<PlankLayout> = (0..self.plank_count)
            .map(|i| {
                let h = hash_pair(seed as i32, i as i32);
                let jitter_z = pseudo_range(h, 0) * base_spacing * 0.25;
                let width_factor = 0.75 + pseudo_range(h, 2).abs() * 0.50;
                let z = start_z + (i as f32 + 0.5) * base_spacing + jitter_z;
                let half_w = plank_he.z * width_factor;
                let yaw = pseudo_range(h, 3) * 0.18;
                let z_footprint = plank_he.x * yaw.abs().sin() + half_w * yaw.abs().cos();
                PlankLayout {
                    z,
                    half_w,
                    yaw,
                    z_footprint,
                    hash: h,
                }
            })
            .collect();

        // Push planks apart so no two overlap (using rotated footprints).
        for i in 1..raw_planks.len() {
            let required_z = raw_planks[i - 1].z
                + raw_planks[i - 1].z_footprint
                + raw_planks[i].z_footprint
                + min_gap;
            if raw_planks[i].z < required_z {
                raw_planks[i].z = required_z;
            }
        }

        let planks: Vec<(Vector3<f32>, Vector3<f32>, UnitQuaternion<f32>)> = raw_planks
            .iter()
            .map(|p| {
                let jitter_x = pseudo_range(p.hash, 1) * 0.12;
                let offset = Vector3::new(jitter_x, plank_y, p.z);
                let varied_he = Vector3::new(plank_he.x, plank_he.y, p.half_w);
                let rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), p.yaw);
                (offset, varied_he, rotation)
            })
            .collect();

        let orientation = Yaw::degrees(self.yaw).rotation();

        // Physics compound body.
        let mut physics = world.write_resource::<PhysicsResource>();
        let body_handle = physics.world.create_body(
            RigidBodyDesc::dynamic()
                .position(center)
                .rotation(orientation)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005),
        );
        let mut looks = PartLooks::default();
        let beams: Vec<ColliderHandle> = beam_offsets
            .iter()
            .filter_map(|&offset| {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(beam_he)
                        .offset_translation(offset)
                        .with_physical_surface(self.surface()),
                )
            })
            .collect();
        for beam in &beams {
            looks.insert(*beam, cuboid_look(beam_he, beam_mat));
        }
        let plank_colliders: Vec<ColliderHandle> = planks
            .iter()
            .filter_map(|&(offset, varied_he, rotation)| {
                let plank = physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(varied_he)
                        .offset_translation(offset)
                        .offset_rotation(rotation)
                        .with_physical_surface(self.surface()),
                )?;
                looks.insert(plank, cuboid_look(varied_he, plank_mat));
                Some(plank)
            })
            .collect();

        // The body's origin is its centre of mass, which the planks
        // lift above the bottom of the beams the parts were laid out from.
        let centre_of_mass = physics.world.recenter_on_colliders(body_handle);
        let placed = |colliders: &[ColliderHandle]| -> Vec<(ColliderHandle, Vector3<f32>)> {
            colliders
                .iter()
                .filter_map(|c| Some((*c, physics.world.collider(*c)?.offset().translation.vector)))
                .collect()
        };
        let graph = self.graph(&placed(&beams), &placed(&plank_colliders));
        let model = looks
            .drawn(&physics.world, body_handle)
            .expect("a bridge has something to draw");
        drop(physics);
        let centre = center + orientation * centre_of_mass;

        vec![world
            .create_entity()
            .with(Position(centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(orientation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(looks)
            .with(CompoundFracture::new(graph).breaking_on_impact_at(self.contact_threshold))
            .build()]
    }
}

/// Deterministic float in [-1, 1] from a hash and channel index.
fn pseudo_range(hash: u32, channel: u32) -> f32 {
    let h = hash
        .wrapping_mul(2654435761)
        .wrapping_add(channel.wrapping_mul(374761393));
    let h = (h ^ (h >> 16)).wrapping_mul(0x45d9f3b);
    ((h & 0xFFFF) as f32 / 32768.0) - 1.0
}

// ---------------------------------------------------------------------------
// Procedural rustic wood textures
// ---------------------------------------------------------------------------

/// Weathered beam wood — dark, knotty, with prominent grain.
fn generate_beam_wood(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.35, 0.24, 0.14);
    let seed = rng.u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Heavy longitudinal grain.
            let grain = fbm_2d_periodic(u * 4.0, v * 24.0, 4, 0.55, 2.0, seed, Some(4));
            let grain_factor = 0.75 + grain * 0.25;

            // Knots — dark patches.
            let knot = fbm_2d_periodic(u * 3.0, v * 3.0, 3, 0.6, 2.0, seed + 3, Some(3));
            let knot_dark = if knot > 0.65 {
                (knot - 0.65) / 0.35 * 0.25
            } else {
                0.0
            };

            // Weathering cracks.
            let crack = crack_pattern(u, v, seed + 7);

            let mut c = base.scale(grain_factor);
            c = c.scale(1.0 - knot_dark);
            c = c.scale(1.0 - crack * 0.15);
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Lighter plank wood — varied, with saw marks and nail holes.
fn generate_plank_wood(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Randomly pick between lighter wood tones for variety.
    let warmth = rng.range(-0.03, 0.03);
    let base = Rgb::new(0.52 + warmth, 0.38 + warmth * 0.7, 0.22 + warmth * 0.4);
    let seed = rng.u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Grain running along the plank length (U direction for cross-planks).
            let grain = fbm_2d_periodic(u * 20.0, v * 5.0, 3, 0.5, 2.0, seed, Some(20));
            let grain_factor = 0.82 + grain * 0.18;

            // Saw marks — faint horizontal lines.
            let saw = ((v * size as f32 * 1.5).sin() * 0.5 + 0.5).powf(8.0);
            let saw_factor = 1.0 - saw * 0.06;

            // Nail holes — two dark dots near the ends.
            let nail = nail_holes(u, v);

            // Subtle weathering.
            let weather = fbm_2d_periodic(u * 8.0, v * 8.0, 2, 0.4, 2.0, seed + 5, Some(8));
            let weather_factor = 0.92 + weather * 0.08;

            let mut c = base.scale(grain_factor * saw_factor * weather_factor);
            c = c.scale(1.0 - nail * 0.5);

            // Darken edges for that individual-plank look.
            let edge = border_band(u, v, 0.05);
            c = c.scale(1.0 - edge * 0.30);
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Two rustic nail-head dots near the left and right ends of a plank.
fn nail_holes(u: f32, v: f32) -> f32 {
    let nail_radius = 0.03;
    let positions = [(0.12, 0.5), (0.88, 0.5)];

    let mut best = 0.0f32;
    for &(nu, nv) in &positions {
        let du = u - nu;
        let dv = v - nv;
        let dist = (du * du + dv * dv).sqrt();
        if dist < nail_radius {
            best = best.max(1.0 - dist / nail_radius);
        }
    }
    best
}

#[cfg(test)]
mod contact_fracture {
    use super::*;
    use crate::debug::DebugLines;
    use crate::fracture::{BondLoads, ContactLoadTracker};
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};
    use nalgebra::{Isometry3, Point3};

    const FRAME_DT: f32 = 1.0 / 60.0;

    fn def() -> PlankBridgeDef {
        PlankBridgeDef {
            pos: (0.0, 0.0, 0.0),
            length: PlankBridgeDef::default_length(),
            beam_spacing: PlankBridgeDef::default_beam_spacing(),
            plank_count: PlankBridgeDef::default_plank_count(),
            beam_half_extents: PlankBridgeDef::default_beam_half_extents(),
            plank_half_extents: PlankBridgeDef::default_plank_half_extents(),
            density: PlankBridgeDef::default_density(),
            yaw: 0.0,
            fracture_threshold: PlankBridgeDef::default_fracture_threshold(),
            contact_threshold: PlankBridgeDef::default_contact_threshold(),
        }
    }

    /// Largest contact load any bond felt, and how many of the sixteen bonds
    /// the authored contact threshold would break, over one scenario.
    fn run(label: &str, height: f32, vel: Vector3<f32>) -> (f32, usize) {
        let d = def();
        let beam_he = d.beam_he();
        let plank_he = d.plank_he();
        let half_spacing = d.beam_spacing / 2.0;
        let beam_y = beam_he.y;
        let plank_y = beam_he.y * 2.0 + plank_he.y;

        {
            let tilted = label == "end_slam";
            let mut config = PhysicsConfig::default();
            config.sleep.enabled = false;
            let mut world = PhysicsWorld::new(config);
            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(Point3::new(0.0, height, 0.0))
                    .rotation(if tilted {
                        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.35)
                    } else {
                        UnitQuaternion::identity()
                    })
                    .linear_velocity(vel)
                    .linear_damping(0.01)
                    .angular_damping(0.005),
            );
            let mut colliders = Vec::new();
            for sx in [-half_spacing, half_spacing] {
                colliders.push(
                    world
                        .attach_collider(
                            body,
                            ColliderDesc::box_shape(beam_he)
                                .offset_translation(Vector3::new(sx, beam_y, 0.0))
                                .density(d.density),
                        )
                        .unwrap(),
                );
            }
            for i in 0..d.plank_count {
                let z = -3.5 + i as f32 * 1.0;
                colliders.push(
                    world
                        .attach_collider(
                            body,
                            ColliderDesc::box_shape(plank_he)
                                .offset_translation(Vector3::new(0.0, plank_y, z))
                                .density(d.density),
                        )
                        .unwrap(),
                );
            }
            // Bonds as the spawnable builds them: every plank to both beams.
            let placed: Vec<(ColliderHandle, Vector3<f32>)> = colliders
                .iter()
                .map(|c| (*c, world.collider(*c).unwrap().offset().translation.vector))
                .collect();
            let graph = d.graph(&placed[..2], &placed[2..]);
            let bonds = graph.bonds().to_vec();
            let mass = world.body(body).unwrap().mass();
            let geometry = FlatQuadGeometry::new(60.0);
            let mut stepper = SequentialStepper::new(FRAME_DT, 4);
            let mut debug = DebugLines::default();
            let mut tracker = ContactLoadTracker::default();
            let mut peak = 0.0f32;
            let mut peak_bond = 0.0f32;
            let mut broken = vec![false; bonds.len()];
            for _ in 0..240 {
                stepper.step(&mut world, FRAME_DT, &geometry, &[], &[], &mut debug);
                let spikes = tracker.advance(&world, body, &colliders, FRAME_DT);
                peak = peak.max(spikes.iter().fold(0.0f32, |m, s| m.max(s.magnitude)));

                let rigid = world.body(body).unwrap();
                let pose = Isometry3::from_parts(rigid.position().coords.into(), rigid.rotation());
                let loads = BondLoads::new(colliders.iter().copied().zip(spikes), &[], pose, None);
                for (idx, bond) in bonds.iter().enumerate() {
                    let load = loads.contact_load(&graph, bond);
                    peak_bond = peak_bond.max(load);
                    if load > d.contact_threshold {
                        broken[idx] = true;
                    }
                }
            }
            let _ = (mass, peak);
            (peak_bond, broken.iter().filter(|b| **b).count())
        }
    }

    /// The bridge is two substances, and breaking must not repaint it: each
    /// beam and plank wears its own material as its own look, which goes
    /// wherever its collider goes.
    #[test]
    fn beams_and_planks_keep_their_own_materials() {
        use specs::{World, WorldExt};

        let mut world = World::new();
        crate::app::breakage::register(&mut world);
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(PhysicsConfig::default()),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));
        let (beam, plank) = (MaterialId(7), MaterialId(9));
        let bridge = def().spawn(&mut world, &[beam, plank])[0];

        let physics = world.read_resource::<PhysicsResource>();
        let looks = world.read_storage::<PartLooks>();
        let looks = looks.get(bridge).expect("the bridge has looks");
        let body = world
            .read_storage::<RigidBodyComponent>()
            .get(bridge)
            .unwrap()
            .0;
        let worn: Vec<MaterialId> = physics
            .world
            .body(body)
            .unwrap()
            .colliders()
            .iter()
            .map(|c| looks.get(*c).expect("every part is drawn").primitives[0].material)
            .collect();
        assert_eq!(&worn[..2], &[beam, beam], "the beams");
        assert!(worn[2..].iter().all(|m| *m == plank), "every plank");
    }

    /// The bug this guards: the bridge came apart the moment it was touched.
    ///
    /// Two things caused it. Its joints are authored to fail under an 8 N·s
    /// blast, while 2.7 tonnes of bridge settling pushes over a thousand N·s
    /// through a beam; and every joint shares a beam, so one spike on a beam
    /// broke all sixteen at once. Contact now has its own threshold, and a
    /// spike is felt only near where it landed.
    #[test]
    fn a_bridge_that_is_stood_on_or_nudged_keeps_its_planks() {
        for (label, height, vel) in [
            ("rest", 0.0, Vector3::zeros()),
            ("nudge", 0.0, Vector3::new(1.5, 0.0, 0.0)),
            ("settle", 0.05, Vector3::zeros()),
        ] {
            let (load, broken) = run(label, height, vel);
            assert_eq!(
                broken, 0,
                "{label}: {broken} joints broke at a peak joint load of {load}"
            );
        }
    }

    /// It is still a rickety bridge: drop it and planks come off.
    #[test]
    fn a_bridge_dropped_on_its_end_sheds_planks() {
        let (load, broken) = run("drop30cm", 0.30, Vector3::zeros());
        assert!(
            broken > 0,
            "a thirty-centimetre drop broke nothing, at a peak joint load of {load}"
        );
    }

    /// And a hard landing takes most of it apart rather than a plank or two.
    #[test]
    fn a_bridge_slammed_into_the_ground_comes_apart() {
        let (_, broken) = run("slam", 2.0, Vector3::new(0.0, -8.0, 0.0));
        assert!(broken >= 8, "a hard slam broke only {broken} of 16 bonds");
    }
}
