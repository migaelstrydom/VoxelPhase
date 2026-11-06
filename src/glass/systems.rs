//! The systems that turn hits on a brittle sheet into cracks, and give a
//! piece split off a sheet its share of it.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, WriteStorage};

use super::components::BrittleSheet;
use super::crazing::cells_within;
use super::held::{weld_along, HeldEdge};
use super::polygon::ConvexPolygon;
use super::remnant::Remnant;
use crate::components::RigidBodyComponent;
use crate::debug::DebugLog;
use crate::fracture::{replace_part, Bond, CompoundFracture, PieceSplits};
use crate::model::PartLooks;
use crate::physics::{
    ColliderHandle, ConstraintHandle, PhysicsImpulse, PhysicsImpulseQueue, RigidBodyHandle,
};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Fastest a knocked-out shard leaves, in m/s.
///
/// The spike a held sheet records is the whole momentum of whatever hit
/// it, and handing all of that to a few grams of glass would fire them like
/// bullets. Shards leave at about the speed of what came through.
const MAX_SHARD_SPEED: f32 = 5.0;

/// Shards must share at least this much edge, in metres, to hold each other.
const MIN_JOINT_OVERLAP: f32 = 0.01;

/// Glass area, in m², a jolt's inertial impulse is scaled to before it is
/// judged against the craze threshold: about one window pane, so the
/// threshold authored for a blow on a pane means the same thing for a jolt.
const JOLT_REFERENCE_AREA: f32 = 0.5;

/// Cracks brittle sheets where they are hit, and hands the results to the
/// fracture system.
///
/// Runs after the physics step and *before* `FractureSystem`, so that a hit
/// on a large pane becomes a web of shards before the bonds are judged: the
/// alternative is a whole pane falling out of a window as one slab.
pub struct GlassCrackSystem;

impl<'a> System<'a> for GlassCrackSystem {
    type SystemData = (
        specs::Write<'a, PhysicsResource>,
        WriteStorage<'a, BrittleSheet>,
        WriteStorage<'a, CompoundFracture>,
        WriteStorage<'a, PartLooks>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, PhysicsImpulseQueue>,
        Read<'a, Time>,
        specs::Write<'a, DebugLog>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physics,
            mut sheets,
            mut fractures,
            mut looks,
            bodies,
            impulse_queue,
            time,
            mut debug_log,
        ) = data;
        let dt = time.delta_seconds();
        let blasts = impulse_queue.last_impulses();
        let mut telemetry = GlassTelemetry::default();

        for (sheet, fracture, looks, body_comp) in
            (&mut sheets, &mut fractures, &mut looks, &bodies).join()
        {
            let body_handle = body_comp.0;
            if sheet.whole_glass_area.is_none() {
                // The pane as spawned, and the structure it was spawned
                // with. Seeding the census here is what keeps an uncracked
                // pane from being judged: it is one cell, and one cell is
                // the very thing the remnant rule lets go of.
                sheet.whole_glass_area = Some(remnant(&physics, sheet, fracture, body_handle).area);
                sheet.last_census = census(&physics, fracture, body_handle);
            }

            let hits = gather_hits(&physics, sheet, body_handle, blasts, dt, &mut telemetry);
            for hit in hits {
                crack(&mut physics, sheet, fracture, looks, body_handle, &hit);
            }

            // Judge what is left whenever the glass has changed at all, not
            // only when this system cracked it: shards are severed under hits
            // that craze nothing, and the fracture system breaks bonds of
            // its own accord. A pane chipped away by either road has to reach
            // the same end as one crazed to bits.
            let census = census(&physics, fracture, body_handle);
            if census != sheet.last_census {
                sheet.last_census = census;
                let left = remnant(&physics, sheet, fracture, body_handle);
                let whole = sheet.whole_glass_area.unwrap_or(0.0);
                if !sheet.remnant.holds(left, whole) {
                    let_go_of_the_glass(&physics, sheet, fracture, body_handle);
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

/// Gives a piece split off a sheet its share of the sheet, so a cluster of
/// shards can craze, wear out and let go as the sheet did; and leaves the
/// sheet the records of what it kept.
///
/// Runs after `FractureSystem`, which sends the splits.
pub struct SheetShareSystem;

impl<'a> System<'a> for SheetShareSystem {
    type SystemData = (
        Read<'a, PieceSplits>,
        WriteStorage<'a, BrittleSheet>,
        ReadStorage<'a, CompoundFracture>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, PhysicsResource>,
    );

    fn run(&mut self, (splits, mut sheets, fractures, bodies, physics): Self::SystemData) {
        if splits.is_empty() {
            return;
        }
        let standing = |entity| -> Option<Standing> {
            let body = bodies.get(entity)?.0;
            let fracture = fractures.get(entity)?;
            let children = physics.world.body(body)?.colliders().to_vec();
            let welds = fracture.graph.ties().iter().map(|t| t.constraint).collect();
            let census = (children.len(), fracture.graph.bonds().len());
            Some((children, welds, census))
        };

        let mut sources = Vec::new();
        for split in splits.iter() {
            let Some(sheet) = sheets.get(split.from) else {
                continue;
            };
            if !sources.contains(&split.from) {
                sources.push(split.from);
            }
            let Some((children, welds, census)) = standing(split.piece) else {
                continue;
            };
            // A piece still welded up is judged as a sheet at once: a lone
            // shard is not one, and must not hang where the pane was.
            if !welds.is_empty() {
                let share = sheet.share(&children, &welds, (0, 0));
                let _ = sheets.insert(split.piece, share);
            } else if sheet.worth_sharing(&children) {
                let share = sheet.share(&children, &welds, census);
                let _ = sheets.insert(split.piece, share);
            }
        }
        for source in sources {
            if let (Some(sheet), Some((children, welds, _))) =
                (sheets.get_mut(source), standing(source))
            {
                sheet.retain(&children, &welds);
            }
        }
    }
}

/// How a sheet's piece stands after a split: its cells, its welds, and its
/// census.
type Standing = (Vec<ColliderHandle>, Vec<ConstraintHandle>, (usize, usize));

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
        .zip(&spikes)
        .filter(|(handle, _)| !sheet.is_glass(**handle))
        .map(|(_, spike)| *spike)
        .max_by(|a, b| a.magnitude.total_cmp(&b.magnitude))
        .filter(|spike| spike.magnitude > threshold)
        .map(|spike| spike.point);
    telemetry.stone_spike = telemetry.stone_spike.max(
        handles
            .iter()
            .zip(&spikes)
            .filter(|(handle, _)| !sheet.is_glass(**handle))
            .map(|(_, spike)| spike.magnitude)
            .fold(0.0, f32::max),
    );
    let failed = match &sheet.fatigue {
        Some(rule) if dt > 0.0 => {
            let forces: Vec<(ColliderHandle, f32)> = handles
                .iter()
                .map(|h| {
                    let force = world
                        .impacts()
                        .for_collider(*h)
                        .map_or(0.0, |i| i.total_impulse / dt);
                    (*h, force)
                })
                .collect();
            sheet.damage.advance(rule, &forces, dt)
        }
        _ => Vec::new(),
    };

    let mut hits = Vec::new();
    for (index, handle) in handles.iter().enumerate() {
        if !sheet.is_glass(*handle) {
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
                .filter_map(|blast| {
                    blast
                        .loading_at(centre)
                        .map(|loading| (loading.per_area.magnitude(), blast))
                })
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

            // A jarred body loads its glass through the bonds that carry it:
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

        if failed.contains(handle) {
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

/// Replace the hit cell with a web of shards, or knock it out whole if it is
/// too small to crack.
///
/// The shards are bonded where they touch; the cell's own bonds go to the
/// shard nearest each, and its welds to the world are made again, one for
/// each shard along a held edge.
fn crack(
    physics: &mut PhysicsResource,
    sheet: &mut BrittleSheet,
    fracture: &mut CompoundFracture,
    looks: &mut PartLooks,
    body_handle: RigidBodyHandle,
    hit: &Hit,
) {
    let world = &physics.world;
    let Some(body) = world.body(body_handle) else {
        return;
    };
    let Some(part) = fracture.graph.part_of(hit.child) else {
        return;
    };
    if !body.colliders().contains(&hit.child) {
        // Already replaced by an earlier hit this frame.
        return;
    }
    let Some(collider) = world.collider(hit.child) else {
        return;
    };
    let depth = sheet.depths.of(hit.child);
    let frame = sheet.frame;
    let old_offset = *collider.offset();
    let Some((cell, height)) = frame.polygon_of(collider.shape(), old_offset.translation.vector)
    else {
        return;
    };

    let hit_local = body
        .rotation()
        .inverse_transform_vector(&(hit.point - body.position()));
    let hit_plane = cell.closest_point(frame.to_plane(hit_local));
    let salt = hit_plane.x.to_bits() ^ hit_plane.y.to_bits().rotate_left(16);

    // A shard rather than a pane — too small to crack, or already as broken
    // down as this sheet goes. It comes out whole if it was hit hard enough.
    let cells = sheet
        .may_craze(hit.child)
        .then(|| sheet.crazing.craze(&cell, hit_plane, frame.thickness, salt))
        .flatten();
    let Some(cells) = cells else {
        if hit.hole_radius > 0.0 {
            fracture.sever(part, capped_kick(hit.kick, collider.mass(), 1));
        }
        return;
    };

    // The welds holding the cell go with it; its shards along the held
    // edges are welded afresh below.
    let held: Vec<HeldEdge> = fracture
        .graph
        .ties_of(&[part])
        .into_iter()
        .filter_map(|weld| {
            fracture.graph.untie(weld);
            physics.world.remove_constraint(weld);
            sheet.held.take(weld)
        })
        .collect();

    // Swap the pane for its shards. Those under the hit come out at once, so
    // they inherit none of what the pane held.
    let under = cells_within(&cells, hit_plane, hit.hole_radius);
    let mut shards = sheet.glass.pieces(
        hit.child,
        &old_offset,
        cells.iter().map(|cell| {
            let (hull, offset) = frame.shard(cell, height);
            (
                Arc::new(hull),
                Isometry3::translation(offset.x, offset.y, offset.z),
            )
        }),
    );
    for k in &under {
        if let Some(shard) = shards.get_mut(*k) {
            shard.inherits = false;
        }
    }
    let Some(made) = replace_part(physics, fracture, looks, body_handle, part, shards) else {
        return;
    };
    sheet.glass.record(hit.child, &made);
    for piece in &made {
        sheet.depths.set(piece.collider, depth + 1);
    }

    // Bond each new shard to the shards it touches.
    for (k, first) in made.iter().enumerate() {
        for second in &made[k + 1..] {
            let (a, b) = (&cells[first.index], &cells[second.index]);
            if a.touches(b, MIN_JOINT_OVERLAP) {
                let middle = (a.centroid() + b.centroid()) * 0.5;
                fracture.graph.bond(Bond {
                    a: first.part,
                    b: second.part,
                    anchor: Point3::from(frame.lift(middle, height)),
                    reach: 2.0 * radius_of(a).min(radius_of(b)),
                    threshold: sheet.joint_threshold,
                });
            }
        }
    }

    // Weld the shards along each held edge where they lie on it.
    for edge in &held {
        for piece in made.iter().filter(|piece| !under.contains(&piece.index)) {
            if let Some((weld, stretch)) = weld_along(
                &mut physics.world,
                body_handle,
                &frame,
                &cells[piece.index],
                edge,
            ) {
                fracture.graph.tie(weld, piece.part);
                sheet.held.insert(weld, stretch);
            }
        }
    }

    // Knock out what was directly under the hit.
    let count = under.len();
    for k in under {
        if let Some(piece) = made.iter().find(|piece| piece.index == k) {
            let mass = physics
                .world
                .collider(piece.collider)
                .map_or(0.0, |c| c.mass());
            fracture.sever(piece.part, capped_kick(hit.kick, mass, count));
        }
    }
}

/// How far a cell reaches from its centroid, m.
fn radius_of(cell: &ConvexPolygon) -> f32 {
    let centre = cell.centroid();
    cell.vertices()
        .iter()
        .map(|v| (v - centre).norm())
        .fold(0.0, f32::max)
}

/// How much glass is left and in how many pieces: the group of cells with
/// the most glass. Groups are ranked by area, so the remnant is the biggest
/// piece of glass rather than the most numerous.
fn remnant(
    physics: &PhysicsResource,
    sheet: &BrittleSheet,
    fracture: &CompoundFracture,
    body_handle: RigidBodyHandle,
) -> Remnant {
    let Some(body) = physics.world.body(body_handle) else {
        return Remnant::default();
    };
    let area_of = |child: &ColliderHandle| -> f32 {
        physics
            .world
            .collider(*child)
            .and_then(|c| {
                sheet
                    .frame
                    .polygon_of(c.shape(), c.offset().translation.vector)
            })
            .map_or(0.0, |(polygon, _)| polygon.area())
    };
    fracture
        .graph
        .groups()
        .iter()
        .map(|group| {
            let colliders = fracture.graph.colliders_of(group);
            let glass: Vec<&ColliderHandle> = colliders
                .iter()
                .filter(|child| body.colliders().contains(child) && sheet.is_glass(**child))
                .collect();
            Remnant {
                area: glass.iter().copied().map(area_of).sum(),
                cells: glass.len(),
            }
        })
        .fold(Remnant::default(), |best, group| {
            if group.area > best.area {
                group
            } else {
                best
            }
        })
}

/// A cheap signature of the sheet's structure: how many children it carries
/// and how many bonds still hold them. Anything that takes glass off the
/// sheet moves one of the two, and nothing else does, so comparing it with
/// the last one judged says exactly when to judge again.
fn census(
    physics: &PhysicsResource,
    fracture: &CompoundFracture,
    body_handle: RigidBodyHandle,
) -> (usize, usize) {
    let children = physics
        .world
        .body(body_handle)
        .map_or(0, |b| b.colliders().len());
    (children, fracture.graph.bonds().len())
}

/// Drop whatever glass is left. A bare sheet has nothing else, so it gives
/// itself up — welds and all — and falls as shards; a framed one keeps its
/// frame standing, empty.
fn let_go_of_the_glass(
    physics: &PhysicsResource,
    sheet: &BrittleSheet,
    fracture: &mut CompoundFracture,
    body_handle: RigidBodyHandle,
) {
    let children = physics
        .world
        .body(body_handle)
        .map_or(&[][..], |body| body.colliders());
    let glass: Vec<ColliderHandle> = children
        .iter()
        .copied()
        .filter(|child| sheet.is_glass(*child))
        .collect();
    if glass.len() == children.len() {
        fracture.release();
    } else {
        for child in glass {
            if let Some(part) = fracture.graph.part_of(child) {
                fracture.sever(part, Vector3::zeros());
            }
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
    use nalgebra::Vector2;
    use specs::{Entity, World, WorldExt};

    use super::super::fatigue::FatigueRule;
    use crate::app::breakage;
    use crate::app::spawnables::{GlassSheetDef, Spawnable};
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};
    use crate::rendering::material::MaterialId;

    const FRAME_DT: f32 = 1.0 / 60.0;
    const SHEET_HEIGHT: f32 = 1.0;

    /// A held, lying pane 2 m by 1.5 m at `SHEET_HEIGHT`, spawned as a level
    /// spawns one — welded to the world along its two long edges — in a
    /// world with a floor far below.
    fn glass_floor(impact_threshold: f32, fatigue: Option<FatigueRule>) -> (World, Entity) {
        let mut world = World::new();
        breakage::register(&mut world);
        world.insert(Time::fixed(FRAME_DT));
        // Sleep is off: a body set down at rest is put to sleep on its first
        // frame, and a sleeping body loads nothing.
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(config),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        let def = GlassSheetDef {
            pos: (0.0, SHEET_HEIGHT, 0.0),
            size: (2.0, 1.5),
            thickness: GlassSheetDef::default_thickness(),
            yaw: 0.0,
            lying: true,
            fixed: true,
            impact_threshold,
            blast_threshold: GlassSheetDef::default_blast_threshold(),
            bearing: fatigue.map_or(0.0, |rule| rule.bearing),
            endurance: fatigue.map_or(GlassSheetDef::default_endurance(), |rule| rule.endurance),
        };
        let sheet = def.spawn(&mut world, &[MaterialId(0)])[0];
        (world, sheet)
    }

    fn body_of(world: &World, entity: Entity) -> RigidBodyHandle {
        world
            .read_storage::<RigidBodyComponent>()
            .get(entity)
            .expect("the sheet has a body")
            .0
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

    /// One game frame: physics, then the break pipeline in dispatcher order.
    fn frame(world: &mut World, stepper: &mut SequentialStepper, geometry: &FlatQuadGeometry) {
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let mut debug = DebugLines::default();
            stepper.step(&mut physics.world, FRAME_DT, geometry, &[], &[], &mut debug);
        }
        breakage::run(world);
    }

    /// Whether some child of the sheet's body still covers the point `(x, z)`.
    fn covered(world: &World, sheet: Entity, x: f32, z: f32) -> bool {
        let physics = world.read_resource::<PhysicsResource>();
        let sheets = world.read_storage::<BrittleSheet>();
        let pane = sheets.get(sheet).expect("the sheet is still there");
        let Some(body) = physics.world.body(body_of(world, sheet)) else {
            return false;
        };
        let point = pane.frame.to_plane(Vector3::new(x, 0.0, z));
        body.colliders().iter().any(|h| {
            let c = physics.world.collider(*h).expect("live collider");
            pane.frame
                .polygon_of(c.shape(), c.offset().translation.vector)
                .is_some_and(|(polygon, _)| polygon.contains(point))
        })
    }

    fn children(world: &World, sheet: Entity) -> Vec<ColliderHandle> {
        let physics = world.read_resource::<PhysicsResource>();
        physics
            .world
            .body(body_of(world, sheet))
            .map_or_else(Vec::new, |b| b.colliders().to_vec())
    }

    fn child_count(world: &World, sheet: Entity) -> usize {
        children(world, sheet).len()
    }

    fn body_height(world: &World, body: RigidBodyHandle) -> f32 {
        let physics = world.read_resource::<PhysicsResource>();
        physics
            .world
            .body(body)
            .map_or(f32::NAN, |b| b.position().y)
    }

    /// Every entity with a body: its body, and whether the world holds it up.
    fn bodies(world: &World) -> Vec<(Entity, RigidBodyHandle, bool)> {
        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        (&world.entities(), &bodies)
            .join()
            .map(|(e, b)| (e, b.0, physics.world.is_held_by_world(b.0)))
            .collect()
    }

    /// No body the glass ever left is empty, and none is lost.
    fn assert_no_empty_body(world: &World) {
        let physics = world.read_resource::<PhysicsResource>();
        for (entity, body, _) in bodies(world) {
            let colliders = physics.world.body(body).map_or(0, |b| b.colliders().len());
            assert!(colliders > 0, "{entity:?} is left with an empty body");
        }
    }

    /// The expense of a broken sheet is the children it carries, and a sheet
    /// that crazed its shards again on every hit grew them without bound. A
    /// cell records how deep in the crazing it was born, and the record has
    /// to survive the reshuffling of the body's collider list that every
    /// break and every severed shard causes.
    #[test]
    fn every_shard_of_a_broken_pane_is_marked_as_already_crazed() {
        let (mut world, sheet) = glass_floor(25.0, None);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        let _crate = drop_box(&mut world, -0.5, -0.5, 1.0, 20.0);
        for _ in 0..60 {
            frame(&mut world, &mut stepper, &geometry);
        }

        let shards = children(&world, sheet);
        let count = shards.len();
        assert!(
            count > 10,
            "the pane should have crazed into a web, not {count} children"
        );

        let sheets = world.read_storage::<BrittleSheet>();
        let pane = sheets.get(sheet).expect("the pane is still there");
        assert_eq!(pane.max_craze_depth, 1, "a cell cracks once, by default");
        for (index, shard) in shards.iter().enumerate() {
            assert!(
                !pane.may_craze(*shard),
                "shard {index} of {count} would craze again"
            );
        }
    }

    /// The literal complaint that coarse shards brought back: a held pane
    /// picked apart leaves one shard hanging in mid-air, held by the welds
    /// behind it. It has to fall, and it has to fall when the glass was
    /// taken off by something other than a craze — here by severing, as a
    /// hit that knocks a shard out whole does.
    #[test]
    fn the_last_shard_of_a_picked_apart_pane_does_not_hang_there() {
        let (mut world, sheet) = glass_floor(25.0, None);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        let _crate = drop_box(&mut world, -0.5, -0.5, 1.0, 20.0);
        for _ in 0..60 {
            frame(&mut world, &mut stepper, &geometry);
        }
        assert!(
            child_count(&world, sheet) > 10,
            "the pane should have crazed into a web first"
        );

        // Take the shards off one at a time, as hits on a broken pane do,
        // and never craze again.
        let held = |world: &World| bodies(world).iter().filter(|b| b.2).count();
        for _ in 0..40 {
            if held(&world) == 0 {
                break;
            }
            {
                let Some(&first) = children(&world, sheet).first() else {
                    break;
                };
                let mut fractures = world.write_storage::<CompoundFracture>();
                let fracture = fractures.get_mut(sheet).expect("the sheet can break");
                let part = fracture.graph.part_of(first).expect("a shard is a part");
                fracture.sever(part, Vector3::zeros());
            }
            for _ in 0..4 {
                frame(&mut world, &mut stepper, &geometry);
            }
        }

        assert_eq!(
            held(&world),
            0,
            "{} pieces of the pane still hang from welds",
            held(&world)
        );
        assert_no_empty_body(&world);
        let body = body_of(&world, sheet);
        for _ in 0..30 {
            frame(&mut world, &mut stepper, &geometry);
        }
        assert!(
            body_height(&world, body) < SHEET_HEIGHT - 0.2,
            "the last of the pane hangs at {}",
            body_height(&world, body)
        );
    }

    /// What is left of a pane once it is smaller than its remnant limit
    /// gives up its welds and falls as shards; no body is left empty.
    #[test]
    fn a_pane_worn_down_below_its_limit_lets_go_of_the_rest() {
        let (mut world, sheet) = glass_floor(25.0, None);
        for pane in (&mut world.write_storage::<BrittleSheet>()).join() {
            pane.remnant.min_area = 100.0;
        }
        let _crate = drop_box(&mut world, -0.7, -0.5, 1.0, 20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        for _ in 0..120 {
            frame(&mut world, &mut stepper, &geometry);
        }

        let all = bodies(&world);
        assert!(
            all.iter().all(|(_, _, held)| !held),
            "a piece of the pane is still welded up"
        );
        assert_no_empty_body(&world);
        let pieces = all.len();
        assert!(pieces > 8, "only {pieces} pieces came down");
        assert!(
            body_height(&world, body_of(&world, sheet)) < SHEET_HEIGHT - 0.3,
            "the pane's own body did not fall"
        );
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
        // The rest of the floor holds: what is left of the sheet is still
        // welded where it was, and is most of a floor.
        let body = body_of(&world, sheet);
        assert!(
            world
                .read_resource::<PhysicsResource>()
                .world
                .is_held_by_world(body),
            "the rest of the floor came down"
        );
        assert!((body_height(&world, body) - SHEET_HEIGHT).abs() < 0.01);
        let left = {
            let sheets = world.read_storage::<BrittleSheet>();
            let fractures = world.read_storage::<CompoundFracture>();
            let physics = world.read_resource::<PhysicsResource>();
            remnant(
                &physics,
                sheets.get(sheet).unwrap(),
                fractures.get(sheet).unwrap(),
                body,
            )
        };
        assert!(
            left.area > 0.25 * 3.0,
            "only {} m² of the 3 m² floor is left",
            left.area
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

    /// A held floor cracked right across is two pieces, each welded along
    /// its own stretch of the held edges, and both stay where they were.
    /// Each takes its share of the sheet: its own cells, its own welds.
    #[test]
    fn a_welded_pane_cracked_across_stays_up_as_two_pieces() {
        let (mut world, sheet) = glass_floor(25.0, None);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let body = body_of(&world, sheet);

        // Craze the pane where it stands, knocking nothing out.
        {
            let pane = children(&world, sheet)[0];
            let mut physics = world.write_resource::<PhysicsResource>();
            let mut sheets = world.write_storage::<BrittleSheet>();
            let mut fractures = world.write_storage::<CompoundFracture>();
            let mut looks = world.write_storage::<PartLooks>();
            crack(
                &mut physics,
                sheets.get_mut(sheet).unwrap(),
                fractures.get_mut(sheet).unwrap(),
                looks.get_mut(sheet).unwrap(),
                body,
                &Hit {
                    child: pane,
                    point: Point3::new(0.1, SHEET_HEIGHT, 0.05),
                    hole_radius: 0.0,
                    kick: Vector3::zeros(),
                },
            );
        }
        let welds = world
            .read_storage::<CompoundFracture>()
            .get(sheet)
            .unwrap()
            .graph
            .ties()
            .len();
        assert!(welds > 4, "only {welds} shards were welded along the edges");

        // Crack it across: every bond between a shard left of x = 0 and one
        // right of it goes.
        {
            let physics = world.read_resource::<PhysicsResource>();
            let mut fractures = world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(sheet).unwrap();
            let side = |part| {
                let collider = fracture.graph.colliders_of(&[part])[0];
                physics
                    .world
                    .collider(collider)
                    .unwrap()
                    .offset()
                    .translation
                    .x
                    > 0.0
            };
            let across: Vec<bool> = fracture
                .graph
                .bonds()
                .iter()
                .map(|bond| side(bond.a) != side(bond.b))
                .collect();
            let mut across = across.into_iter();
            fracture
                .graph
                .break_bonds(|_| across.next().unwrap_or(false));
            fracture.split_pending = true;
        }
        for _ in 0..90 {
            frame(&mut world, &mut stepper, &geometry);
        }

        let glass: Vec<(Entity, RigidBodyHandle, bool)> = bodies(&world)
            .into_iter()
            .filter(|(e, _, _)| world.read_storage::<BrittleSheet>().contains(*e))
            .collect();
        assert_eq!(glass.len(), 2, "the floor came apart into {glass:?}");
        for (entity, body, held) in &glass {
            assert!(held, "{entity:?} is not held up");
            let height = body_height(&world, *body);
            assert!(
                (height - SHEET_HEIGHT).abs() < 0.01,
                "{entity:?} sagged to {height}"
            );
            let physics = world.read_resource::<PhysicsResource>();
            let sheets = world.read_storage::<BrittleSheet>();
            let fractures = world.read_storage::<CompoundFracture>();
            let own = physics.world.body(*body).unwrap().colliders();
            assert!(
                own.iter()
                    .all(|c| sheets.get(*entity).unwrap().is_glass(*c)),
                "a piece's sheet does not know its own cells"
            );
            for tie in fractures.get(*entity).unwrap().graph.ties() {
                let holds = physics.world.constraint(tie.constraint).unwrap();
                assert!(
                    holds.kind.references_body(*body),
                    "a weld holds the other half"
                );
            }
        }
        assert_no_empty_body(&world);
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
