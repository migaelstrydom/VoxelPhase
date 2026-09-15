//! The system that turns hits on a brittle sheet into cracks.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, WriteStorage};

use super::components::BrittleSheet;
use super::crazing::cells_within;
use super::polygon::ConvexPolygon;
use crate::components::{ModelInstance, RigidBodyComponent};
use crate::debug::DebugLog;
use crate::fracture::systems::compound_model_of;
use crate::fracture::{CompoundFracture, FractureJoint};
use crate::physics::{
    ColliderDesc, ColliderHandle, PhysicsImpulse, PhysicsImpulseQueue, RigidBodyHandle,
};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Fastest a knocked-out shard leaves, in m/s.
///
/// The spike a static sheet records is the whole momentum of whatever hit
/// it, and handing all of that to a few grams of glass would fire them like
/// bullets. Shards leave at about the speed of what came through.
const MAX_SHARD_SPEED: f32 = 5.0;

/// Shards must share at least this much edge, in metres, to hold each other.
const MIN_JOINT_OVERLAP: f32 = 0.01;

/// How far, in metres, each shard's collider stands back from the crack.
///
/// Two shards that share an edge exactly are two colliders in contact from
/// the moment one of them comes free, and a body that starts a sweep already
/// touching its neighbour is clamped at time zero by continuous collision
/// detection: it hangs in the air with its velocity climbing. A hairline of
/// clearance means no shard ever starts out touching another. It is also,
/// through the glass, the visible crack.
const CRACK_GAP: f32 = 0.0008;
/// Glass area, in m², a jolt's inertial impulse is scaled to before it is
/// judged against the craze threshold: about one window pane, so the
/// threshold authored for a blow on a pane means the same thing for a jolt.
const JOLT_REFERENCE_AREA: f32 = 0.5;

/// Cracks brittle sheets where they are hit, and hands the results to the
/// fracture system.
///
/// Runs after the physics step and *before* `FractureSystem`, so that a hit
/// on a large pane becomes a web of shards before the joints are judged: the
/// alternative is a whole pane falling out of a window as one slab.
pub struct GlassCrackSystem;

impl<'a> System<'a> for GlassCrackSystem {
    type SystemData = (
        specs::Write<'a, PhysicsResource>,
        WriteStorage<'a, BrittleSheet>,
        WriteStorage<'a, CompoundFracture>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        Read<'a, PhysicsImpulseQueue>,
        Read<'a, Time>,
        specs::Write<'a, DebugLog>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physics,
            mut sheets,
            mut fractures,
            bodies,
            mut models,
            impulse_queue,
            time,
            mut debug_log,
        ) = data;
        let dt = time.delta_seconds();
        let blasts = impulse_queue.last_impulses();
        let mut telemetry = GlassTelemetry::default();

        for (sheet, fracture, body_comp, model) in
            (&mut sheets, &mut fractures, &bodies, &mut models).join()
        {
            let body_handle = body_comp.0;
            let hits = gather_hits(
                &physics,
                sheet,
                fracture,
                body_handle,
                blasts,
                dt,
                &mut telemetry,
            );
            if hits.is_empty() {
                continue;
            }

            let mut cracked = false;
            for hit in hits {
                cracked |= crack(&mut physics, sheet, fracture, body_handle, &hit);
            }
            if cracked {
                if remnant_area(&physics, sheet, fracture, body_handle) < sheet.min_remnant_area {
                    let_go_of_the_glass(sheet, fracture);
                }
                if let Some(rebuilt) = compound_model_of(
                    &physics,
                    body_handle,
                    &fracture.materials,
                    fracture.piece_mesh,
                ) {
                    model.model = rebuilt;
                }
            }
        }

        debug_log.add("Glass/MaxSpike", format!("{:.1} N·s", telemetry.spike));
        debug_log.add("Glass/MaxJolt", format!("{:.1} N·s", telemetry.jolt));
        debug_log.add(
            "Glass/MaxStoneSpike",
            format!("{:.1} N·s", telemetry.stone_spike),
        );
    }
}

/// The largest loads any glass saw this frame, for the debug log: the
/// numbers to read off when tuning a threshold.
#[derive(Debug, Default, Clone, Copy)]
struct GlassTelemetry {
    /// Largest contact spike on a glass child, N·s.
    spike: f32,
    /// Largest jolt on a glass child, N·s, scaled to `JOLT_REFERENCE_AREA`.
    jolt: f32,
    /// Largest contact spike on a frame child, N·s.
    stone_spike: f32,
}

/// One thing that happened to one child this frame.
struct Hit {
    child: ColliderHandle,
    /// Where it landed, in world space.
    point: Point3<f32>,
    /// Radius around the point within which shards are knocked out.
    hole_radius: f32,
    /// Total impulse the knocked-out shards share, in world space.
    kick: Vector3<f32>,
}

/// Read this frame's loads and pick out the ones that craze a cell.
///
/// Reads every tracker every frame whether or not anything happens: a
/// baseline that stops advancing turns a quiet load into a spike later.
fn gather_hits(
    physics: &PhysicsResource,
    sheet: &mut BrittleSheet,
    fracture: &CompoundFracture,
    body_handle: RigidBodyHandle,
    blasts: &[PhysicsImpulse],
    dt: f32,
    telemetry: &mut GlassTelemetry,
) -> Vec<Hit> {
    let world = &physics.world;
    let Some(body) = world.body(body_handle) else {
        return Vec::new();
    };
    let body_pos = body.position();
    let body_rot = body.rotation();
    let handles: Vec<ColliderHandle> = body.colliders().to_vec();

    let spikes = sheet.contact_load.advance(world, body_handle, &handles, dt);
    let jolt = sheet
        .jolt
        .advance(body.linear_velocity(), body.angular_velocity());
    // Where the frame was struck this frame, if it was: the glass cracks
    // nearest to it. A jolt counts only then. The body is also yanked about
    // by things that are not collisions — a grab's orientation lock has the
    // torque to jerk a tonne of stone every frame — and glass carried in a
    // frame is not glass hit by one.
    let threshold = sheet.crazing.threshold;
    let frame_struck = handles
        .iter()
        .enumerate()
        .filter(|(index, _)| !sheet.is_glass(fracture.material_of(*index)))
        .map(|(index, _)| spikes[index])
        .max_by(|a, b| a.magnitude.total_cmp(&b.magnitude))
        .filter(|spike| spike.magnitude > threshold)
        .map(|spike| spike.point);
    telemetry.stone_spike = telemetry.stone_spike.max(
        handles
            .iter()
            .enumerate()
            .filter(|(index, _)| !sheet.is_glass(fracture.material_of(*index)))
            .map(|(index, _)| spikes[index].magnitude)
            .fold(0.0, f32::max),
    );
    let failed = match &sheet.fatigue {
        Some(rule) if dt > 0.0 => {
            let forces: Vec<f32> = handles
                .iter()
                .map(|h| {
                    world
                        .impacts()
                        .for_collider(*h)
                        .map_or(0.0, |i| i.total_impulse / dt)
                })
                .collect();
            sheet.damage.advance(rule, &forces, dt)
        }
        _ => Vec::new(),
    };

    let mut hits = Vec::new();
    for (index, handle) in handles.iter().enumerate() {
        if !sheet.is_glass(fracture.material_of(index)) {
            continue;
        }
        let impact = world.impacts().for_collider(*handle);
        let spike = spikes[index];
        telemetry.spike = telemetry.spike.max(spike.magnitude);

        // Whatever came through opens a hole at least its own size, centred
        // on where it bore down: the energy rule alone let a crate straddle
        // the hole it had punched at one of its corners.
        let impactor_radius = impact
            .and_then(|i| i.other)
            .and_then(|other| world.collider(other))
            .map_or(0.0, |c| c.shape().footprint_radius());
        let landed_at = impact.map_or(body_pos, |i| i.centre);

        let mut hit: Option<Hit> = None;
        if spike.magnitude > threshold {
            let normal = impact.map_or(Vector3::zeros(), |i| i.normal);
            hit = Some(Hit {
                child: *handle,
                point: landed_at,
                hole_radius: sheet
                    .crazing
                    .hole_radius(spike.magnitude)
                    .max(impactor_radius),
                kick: normal * spike.magnitude,
            });
        } else if let Some(collider) = world.collider(*handle) {
            let centre = Point3::from(
                collider
                    .world_transform(body_pos, body_rot)
                    .translation
                    .vector,
            );
            // The strongest blast decides where the web is centred; the
            // fracture system, reading the same blasts this frame, decides
            // which of the new shards it blows out.
            let strongest = blasts
                .iter()
                .filter_map(|blast| blast.impulse_at(centre).map(|v| (v.magnitude(), blast)))
                .max_by(|a, b| a.0.total_cmp(&b.0));
            if let Some((magnitude, PhysicsImpulse::Radial { center, .. })) = strongest {
                if magnitude > threshold {
                    hit = Some(Hit {
                        child: *handle,
                        point: *center,
                        hole_radius: 0.0,
                        kick: Vector3::zeros(),
                    });
                }
            }

            // A jarred body loads its glass through the joints that carry it:
            // the impulse it took to change the cell's velocity by more than
            // it changed last frame. A window falling on its face shatters.
            //
            // Unlike a blow, a jolt loads every square metre of glass alike,
            // so it is judged as one: the cell's inertial impulse scaled to
            // `JOLT_REFERENCE_AREA`, or a small pane would ride out what
            // shatters a large one beside it.
            let r = centre - body_pos;
            let area = sheet
                .frame
                .polygon_of(collider.shape(), collider.offset().translation.vector)
                .map_or(0.0, |(polygon, _)| polygon.area());
            let jolted = if area > 0.0 {
                collider.mass() * jolt.jerk.at(r).magnitude() * (JOLT_REFERENCE_AREA / area)
            } else {
                0.0
            };
            telemetry.jolt = telemetry.jolt.max(jolted);
            if let (None, Some(struck)) = (&hit, frame_struck) {
                if jolted > threshold {
                    // The whole cell is loaded, so the hole grows with the
                    // whole cell: twice the threshold empties it.
                    let excess = jolted / threshold - 1.0;
                    hit = Some(Hit {
                        child: *handle,
                        point: struck,
                        hole_radius: 2.0 * collider.shape().bounding_radius() * excess,
                        kick: -collider.mass() * jolt.delta.at(r),
                    });
                }
            }
        }

        if failed.contains(&index) {
            let point = landed_at;
            let hole_radius = sheet.fatigue_hole_radius.max(impactor_radius);
            hit = Some(match hit {
                Some(mut hit) => {
                    hit.hole_radius = hit.hole_radius.max(hole_radius);
                    hit
                }
                None => Hit {
                    child: *handle,
                    point,
                    hole_radius,
                    kick: Vector3::zeros(),
                },
            });
        }

        hits.extend(hit);
    }
    hits
}

/// Replace the hit child with a web of shards, or knock it out whole if it is
/// too small to crack. Returns whether the body's colliders changed.
fn crack(
    physics: &mut PhysicsResource,
    sheet: &mut BrittleSheet,
    fracture: &mut CompoundFracture,
    body_handle: RigidBodyHandle,
    hit: &Hit,
) -> bool {
    let world = &physics.world;
    let Some(body) = world.body(body_handle) else {
        return false;
    };
    let old_handles: Vec<ColliderHandle> = body.colliders().to_vec();
    let Some(child) = old_handles.iter().position(|h| *h == hit.child) else {
        // Already replaced by an earlier hit this frame.
        return false;
    };
    let Some(collider) = world.collider(hit.child) else {
        return false;
    };
    let frame = sheet.frame;
    let Some((cell, height)) =
        frame.polygon_of(collider.shape(), collider.offset().translation.vector)
    else {
        return false;
    };

    let hit_local = body
        .rotation()
        .inverse_transform_vector(&(hit.point - body.position()));
    let hit_plane = cell.closest_point(frame.to_plane(hit_local));
    let salt = hit_plane.x.to_bits() ^ hit_plane.y.to_bits().rotate_left(16);

    let Some(cells) = sheet.crazing.craze(&cell, hit_plane, frame.thickness, salt) else {
        // A shard, not a pane. It comes out whole if it was hit hard enough.
        if hit.hole_radius > 0.0 {
            fracture.sever(child, capped_kick(hit.kick, collider.mass(), 1));
        }
        return false;
    };

    // The neighbours' footprints, for deciding which new shards they join.
    let neighbours: Vec<(ColliderHandle, ConvexPolygon)> = old_handles
        .iter()
        .filter(|h| **h != hit.child)
        .filter_map(|h| {
            let c = world.collider(*h)?;
            let (polygon, _) = frame.polygon_of(c.shape(), c.offset().translation.vector)?;
            Some((*h, polygon))
        })
        .collect();

    let density = collider.mass() / collider.shape().compute_mass(1.0).max(1e-9);
    let material = *collider.material();

    // Swap the pane for its shards.
    let world = &mut physics.world;
    world.detach_collider(body_handle, hit.child);
    let mut new_handles = Vec::with_capacity(cells.len());
    for cell in &cells {
        let (hull, centroid) = frame.prism(&cell.inset(CRACK_GAP).unwrap_or_else(|| cell.clone()));
        let desc = ColliderDesc::convex_hull(Arc::new(hull))
            .offset_translation(frame.lift(centroid, height))
            .density(density)
            .restitution(material.restitution)
            .friction_model(material.friction);
        if let Some(handle) = world.attach_collider(body_handle, desc) {
            new_handles.push(handle);
        }
    }
    world.wake_body(body_handle);

    // Everything indexed by child follows the body's new collider order.
    let Some(body) = world.body(body_handle) else {
        return true;
    };
    let now: Vec<ColliderHandle> = body.colliders().to_vec();
    let order: Vec<Option<usize>> = now
        .iter()
        .map(|h| old_handles.iter().position(|old| old == h))
        .collect();
    fracture.reindex(&order);
    sheet.contact_load.reindex(&order);
    sheet.damage.reindex(&order);

    let index_of = |handle: ColliderHandle| now.iter().position(|h| *h == handle);
    for handle in &new_handles {
        if let Some(index) = index_of(*handle) {
            fracture.materials[index] = sheet.material;
        }
    }

    // Join each new shard to the shards and neighbours it touches.
    let mut footprints: Vec<(usize, ConvexPolygon)> = Vec::new();
    for (handle, polygon) in neighbours {
        if let Some(index) = index_of(handle) {
            footprints.push((index, polygon));
        }
    }
    let new_indices: Vec<usize> = new_handles.iter().filter_map(|h| index_of(*h)).collect();
    for (k, cell) in cells.iter().enumerate() {
        let Some(&index) = new_indices.get(k) else {
            continue;
        };
        for (other, polygon) in &footprints {
            if cell.touches(polygon, MIN_JOINT_OVERLAP) {
                let threshold = if sheet.is_glass(fracture.material_of(*other)) {
                    sheet.joint_threshold
                } else {
                    sheet.frame_grip
                };
                fracture.joints.push(FractureJoint {
                    child_a: index,
                    child_b: *other,
                    threshold,
                });
            }
        }
        footprints.push((index, cell.clone()));
    }

    // Knock out what was directly under the hit.
    let under = cells_within(&cells, hit_plane, hit.hole_radius);
    let count = under.len();
    for k in under {
        if let Some(&index) = new_indices.get(k) {
            let mass = world.collider(new_handles[k]).map_or(0.0, |c| c.mass());
            fracture.sever(index, capped_kick(hit.kick, mass, count));
        }
    }

    true
}

/// Area of what will still be the sheet after this frame's split: the largest
/// connected group of cells, which is the one the fracture system keeps.
fn remnant_area(
    physics: &PhysicsResource,
    sheet: &BrittleSheet,
    fracture: &CompoundFracture,
    body_handle: RigidBodyHandle,
) -> f32 {
    let Some(body) = physics.world.body(body_handle) else {
        return 0.0;
    };
    let handles = body.colliders();
    let area_of = |child: &usize| -> f32 {
        handles
            .get(*child)
            .and_then(|h| physics.world.collider(*h))
            .and_then(|c| {
                sheet
                    .frame
                    .polygon_of(c.shape(), c.offset().translation.vector)
            })
            .map_or(0.0, |(polygon, _)| polygon.area())
    };
    fracture
        .connected_components()
        .iter()
        .map(|component| {
            component
                .iter()
                .filter(|child| sheet.is_glass(fracture.material_of(**child)))
                .map(area_of)
                .sum::<f32>()
        })
        .fold(0.0, f32::max)
}

/// Drop whatever glass is left. A bare sheet has nothing else, so its body
/// retires with the glass; a framed one keeps its frame standing, empty.
fn let_go_of_the_glass(sheet: &BrittleSheet, fracture: &mut CompoundFracture) {
    let glass: Vec<usize> = (0..fracture.child_count)
        .filter(|child| sheet.is_glass(fracture.material_of(*child)))
        .collect();
    if glass.len() == fracture.child_count {
        fracture.release();
    } else {
        for child in glass {
            fracture.sever(child, Vector3::zeros());
        }
    }
}

/// A shard's share of the hit's impulse, held to `MAX_SHARD_SPEED`.
fn capped_kick(kick: Vector3<f32>, shard_mass: f32, shares: usize) -> Vector3<f32> {
    let share = kick / shares.max(1) as f32;
    let limit = shard_mass * MAX_SHARD_SPEED;
    let magnitude = share.magnitude();
    if magnitude > limit && magnitude > 0.0 {
        share * (limit / magnitude)
    } else {
        share
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector2};
    use specs::{Builder, RunNow, World, WorldExt};

    use super::super::crazing::CrazeRule;
    use super::super::fatigue::FatigueRule;
    use super::super::pane::SheetFrame;
    use super::super::web::CrackWeb;
    use crate::components::{Orientation, Position, Renderable, Velocity};
    use crate::debug::DebugLines;
    use crate::fracture::FractureSystem;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{PhysicsConfig, PhysicsWorld, RigidBodyDesc};
    use crate::rendering::material::MaterialId;

    const FRAME_DT: f32 = 1.0 / 60.0;
    const SHEET_HEIGHT: f32 = 1.0;

    /// A fixed, lying pane 2 m by 1.5 m at `SHEET_HEIGHT`, as the spawnable
    /// builds one, in a world with a floor far below.
    fn glass_floor(
        impact_threshold: f32,
        fatigue: Option<FatigueRule>,
    ) -> (World, RigidBodyHandle) {
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Orientation>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CompoundFracture>();
        world.register::<BrittleSheet>();
        world.insert(Time::fixed(FRAME_DT));
        world.insert(PhysicsImpulseQueue::default());
        world.insert(DebugLog::default());

        let frame = SheetFrame::lying(0.015);
        let half_extents = frame.box_half_extents(0.75, 1.0);
        // Sleep is off: a body set down at rest is put to sleep on its first
        // frame, and a sleeping body loads nothing.
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut physics_world = PhysicsWorld::new(config);
        let body = physics_world.create_body(RigidBodyDesc::static_body().position(Point3::new(
            0.0,
            SHEET_HEIGHT,
            0.0,
        )));
        physics_world.attach_collider(
            body,
            ColliderDesc::box_shape(half_extents)
                .density(2500.0)
                .restitution(0.1)
                .friction(0.4),
        );
        world.insert(PhysicsResource::new(
            physics_world,
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        let crazing = CrazeRule {
            threshold: impact_threshold,
            web: CrackWeb {
                core_radius: 0.08,
                ring_growth: 1.7,
                rings: 5,
                background_spacing: 0.35,
            },
            min_area: 0.002,
        };
        let mut sheet = BrittleSheet::whole(frame, crazing, 8.0, MaterialId(0));
        if let Some(rule) = fatigue {
            sheet = sheet.wearing_out_under(rule, 0.35);
        }
        let model =
            crate::app::spawnables::shared::models::cuboid_model(half_extents, MaterialId(0));
        world
            .create_entity()
            .with(Position(Vector3::new(0.0, SHEET_HEIGHT, 0.0)))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(UnitQuaternion::identity()))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(
                CompoundFracture::boxes(Vec::new(), 1, MaterialId(0))
                    .breaking_on_impact_at(impact_threshold),
            )
            .with(sheet)
            .build();
        (world, body)
    }

    /// A box of `mass` kg dropped from `drop` metres above the pane at
    /// `(x, z)`, as a plain dynamic body outside the ECS.
    fn drop_box(world: &mut World, x: f32, z: f32, drop: f32, mass: f32) -> RigidBodyHandle {
        let half = 0.15;
        let density = mass / (8.0 * half * half * half);
        let mut physics = world.write_resource::<PhysicsResource>();
        let body = physics
            .world
            .create_body(RigidBodyDesc::dynamic().position(Point3::new(
                x,
                SHEET_HEIGHT + 0.0075 + half + drop,
                z,
            )));
        physics.world.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::repeat(half))
                .density(density)
                .restitution(0.0),
        );
        body
    }

    /// One game frame: physics, then the two systems in dispatcher order.
    fn frame(world: &mut World, stepper: &mut SequentialStepper, geometry: &FlatQuadGeometry) {
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let mut debug = DebugLines::default();
            stepper.step(&mut physics.world, FRAME_DT, geometry, &[], &[], &mut debug);
        }
        GlassCrackSystem.run_now(world);
        FractureSystem.run_now(world);
        world.maintain();
    }

    /// Whether some child of the sheet still covers the point `(x, z)`.
    fn covered(world: &World, body: RigidBodyHandle, x: f32, z: f32) -> bool {
        let physics = world.read_resource::<PhysicsResource>();
        let sheets = world.read_storage::<BrittleSheet>();
        let sheet = (&sheets).join().next().expect("the sheet is still there");
        let Some(body) = physics.world.body(body) else {
            return false;
        };
        let point = sheet.frame.to_plane(Vector3::new(x, 0.0, z));
        body.colliders().iter().any(|h| {
            let c = physics.world.collider(*h).expect("live collider");
            sheet
                .frame
                .polygon_of(c.shape(), c.offset().translation.vector)
                .is_some_and(|(polygon, _)| polygon.contains(point))
        })
    }

    fn child_count(world: &World, body: RigidBodyHandle) -> usize {
        let physics = world.read_resource::<PhysicsResource>();
        physics.world.body(body).map_or(0, |b| b.colliders().len())
    }

    fn body_height(world: &World, body: RigidBodyHandle) -> f32 {
        let physics = world.read_resource::<PhysicsResource>();
        physics
            .world
            .body(body)
            .map_or(f32::NAN, |b| b.position().y)
    }

    /// What is left of a pane once it is smaller than its remnant limit comes
    /// free as shards, and the pane's own entity and body are gone.
    #[test]
    fn a_pane_worn_down_below_its_limit_lets_go_of_the_rest() {
        let (mut world, sheet) = glass_floor(25.0, None);
        for pane in (&mut world.write_storage::<BrittleSheet>()).join() {
            pane.min_remnant_area = 100.0;
        }
        let _crate = drop_box(&mut world, -0.7, -0.5, 1.0, 20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        for _ in 0..120 {
            frame(&mut world, &mut stepper, &geometry);
        }

        assert_eq!(
            world.read_storage::<BrittleSheet>().join().count(),
            0,
            "the pane entity should be gone"
        );
        assert_eq!(
            child_count(&world, sheet),
            0,
            "the pane body should be gone"
        );
        let freed = (
            &world.read_storage::<RigidBodyComponent>(),
            &world.read_storage::<Renderable>(),
        )
            .join()
            .count();
        assert!(freed > 8, "only {freed} shards were freed");
    }

    /// The headline behaviour: a crate through one corner leaves a hole at
    /// that corner, a web around it, and the far corner in one piece.
    #[test]
    fn a_crate_through_one_corner_leaves_the_far_corner_whole() {
        let (mut world, sheet) = glass_floor(25.0, None);
        let crate_body = drop_box(&mut world, -0.7, -0.5, 1.0, 20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        for _ in 0..120 {
            frame(&mut world, &mut stepper, &geometry);
        }

        let children = child_count(&world, sheet);
        assert!(children > 8, "the pane crazed into {children} shards");
        assert!(
            !covered(&world, sheet, -0.7, -0.5),
            "the shards under the crate should be gone"
        );
        assert!(
            covered(&world, sheet, 0.7, 0.5),
            "the far corner should still be glass"
        );
        assert!(
            body_height(&world, crate_body) < SHEET_HEIGHT - 0.5,
            "the crate should have fallen through, not at {}",
            body_height(&world, crate_body)
        );
    }

    /// Standing still on a floor that cannot bear you is how you fall
    /// through it; the same weight resting on a pane that can bear it
    /// changes nothing.
    #[test]
    fn a_weight_that_stays_put_falls_through_a_floor_that_cannot_bear_it() {
        let tired = FatigueRule {
            bearing: 400.0,
            endurance: 0.6,
        };
        let (mut world, sheet) = glass_floor(25.0, Some(tired));
        // Placed, not dropped: 80 kg set down from a hand's width.
        let weight = drop_box(&mut world, 0.2, 0.3, 0.02, 80.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        let mut fell_at = None;
        for f in 0..240 {
            frame(&mut world, &mut stepper, &geometry);
            if body_height(&world, weight) < SHEET_HEIGHT - 0.3 {
                fell_at = Some(f);
                break;
            }
        }
        let fell_at = fell_at.expect("the weight fell through");
        assert!((30..200).contains(&fell_at), "fell at frame {fell_at}");
        assert!(
            covered(&world, sheet, -0.6, -0.6),
            "the rest of the floor should hold"
        );

        let (mut world, sheet) = glass_floor(25.0, None);
        let weight = drop_box(&mut world, 0.2, 0.3, 0.02, 80.0);
        for _ in 0..240 {
            frame(&mut world, &mut stepper, &geometry);
        }
        assert_eq!(
            child_count(&world, sheet),
            1,
            "an untiring pane stays whole"
        );
        assert!(body_height(&world, weight) > SHEET_HEIGHT);
    }

    #[test]
    fn a_kick_is_shared_and_capped() {
        let kick = Vector3::new(0.0, -100.0, 0.0);
        let shared = capped_kick(kick, 10.0, 4);
        assert!((shared.y + 25.0).abs() < 1e-6);
        let capped = capped_kick(kick, 0.2, 4);
        assert!((capped.magnitude() - 1.0).abs() < 1e-6);
        assert!(capped.y < 0.0);
    }

    #[test]
    fn a_hit_off_the_pane_is_pulled_onto_its_edge() {
        let pane = ConvexPolygon::rectangle(Vector2::zeros(), 1.0, 1.0);
        let on_edge = pane.closest_point(Vector2::new(1.5, 0.0));
        assert_eq!(on_edge, Vector2::new(1.0, 0.0));
    }
}
