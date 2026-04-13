//! Physics debug visualization and logging.

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;

use super::body::RigidBody;
use super::contact_event::{ContactEvent, ContactSource};
use super::handle::RigidBodyHandle;
use super::pipeline::pair::{PairHeader, PairManifold, SolverManifold};
use crate::debug::{DebugLines, DebugLog, DebugOverlays};
use crate::rendering::Colour;

// Ignore unused fields and structures for the whole file, since these are only used for debugging.
#[allow(unused)]

/// Debug rendering configuration.
#[derive(Debug, Clone)]
pub struct PhysicsDebugConfig {
    /// Draw contact points and normals as debug overlays.
    pub draw_contacts: bool,
    /// Draw raw (pre-smoothed) contact normals for comparison.
    pub draw_contact_raw_normals: bool,
    /// Draw sleep state markers over sleeping bodies.
    pub draw_sleeping: bool,
}

impl Default for PhysicsDebugConfig {
    fn default() -> Self {
        Self {
            draw_contacts: true,
            draw_contact_raw_normals: false,
            draw_sleeping: false,
        }
    }
}

/// Handles debug visualization and logging for the physics engine.
pub struct PhysicsDebugger {
    config: PhysicsDebugConfig,
    last_snapshot: Option<ContactDebugSnapshot>,
}

impl PhysicsDebugger {
    pub fn new(config: PhysicsDebugConfig) -> Self {
        Self {
            config,
            last_snapshot: None,
        }
    }

    /// Update debug statistics from the current frame's contact data.
    pub fn update(
        &mut self,
        bodies: &Arena<RigidBody>,
        raw_manifolds: &[PairManifold],
        merged_manifolds: &[SolverManifold],
        active_manifolds: &[SolverManifold],
        events: &[ContactEvent],
    ) {
        self.last_snapshot = Some(ContactDebugSnapshot::new(
            bodies,
            raw_manifolds,
            merged_manifolds,
            active_manifolds,
            events,
        ));
    }

    /// Record post-solve body statistics for the primary body.
    pub fn update_post_solve(&mut self, bodies: &Arena<RigidBody>, manifolds: &[SolverManifold]) {
        if let Some(snapshot) = self.last_snapshot.as_mut() {
            if let Some(primary) = snapshot.primary_body.as_ref() {
                snapshot.post_solve_body =
                    compute_post_solve_body_stats(bodies, manifolds, primary.handle);
            }
        }
    }

    /// Write debug statistics to the debug log resource.
    pub fn write_debug_log(
        &self,
        debug_log: &mut DebugLog,
        contact_margin: f32,
        warm_start_depth_slop: f32,
    ) {
        let Some(snapshot) = &self.last_snapshot else {
            debug_log.add("Physics/Status", "No data yet");
            return;
        };

        debug_log.add("Physics/Events/Total", snapshot.events.total.to_string());
        debug_log.add("Physics/Events/CCD", snapshot.events.ccd.to_string());
        debug_log.add(
            "Physics/Events/Narrowphase",
            snapshot.events.narrow.to_string(),
        );

        debug_log.add("Physics/Raw/Total", snapshot.raw.total.to_string());
        debug_log.add("Physics/Raw/Static", snapshot.raw.static_count.to_string());
        debug_log.add("Physics/Raw/NegRaw", snapshot.raw.neg_raw.to_string());
        debug_log.add("Physics/Raw/WarmUsed", snapshot.raw.warm_used.to_string());
        debug_log.add(
            "Physics/Raw/DepthRange",
            format!("[{:.5}, {:.5}]", snapshot.raw.min_raw, snapshot.raw.max_raw),
        );

        debug_log.add("Physics/Merged/Total", snapshot.merged.total.to_string());
        debug_log.add(
            "Physics/Merged/Static",
            snapshot.merged.static_count.to_string(),
        );
        debug_log.add(
            "Physics/Merged/WarmUsed",
            snapshot.merged.warm_used.to_string(),
        );

        debug_log.add("Physics/Active/Total", snapshot.active.total.to_string());
        debug_log.add(
            "Physics/Active/Static",
            snapshot.active.static_count.to_string(),
        );
        debug_log.add(
            "Physics/Active/WarmUsed",
            snapshot.active.warm_used.to_string(),
        );
        debug_log.add(
            "Physics/Config/ContactMargin",
            format!("{:.5}", contact_margin),
        );
        debug_log.add(
            "Physics/Config/WarmStartDepthSlop",
            format!("{:.5}", warm_start_depth_slop),
        );

        if let Some(body_stats) = &snapshot.primary_body {
            debug_log.add(
                "Physics/Body/Top/Handle",
                format!("{:?}", body_stats.handle),
            );
            debug_log.add(
                "Physics/Body/Top/Counts",
                format!(
                    "total={} static={}",
                    body_stats.total, body_stats.static_count
                ),
            );
            debug_log.add(
                "Physics/Body/Top/Depth",
                format!(
                    "min={:.5} max={:.5} avg={:.5}",
                    body_stats.min_depth, body_stats.max_depth, body_stats.avg_depth
                ),
            );
            debug_log.add(
                "Physics/Body/Top/RelN",
                format!(
                    "min={:.5} max={:.5} avg={:.5}",
                    body_stats.rel_n_min, body_stats.rel_n_max, body_stats.rel_n_avg
                ),
            );
            debug_log.add(
                "Physics/Body/Top/Normal",
                format!(
                    "avg=({:.3},{:.3},{:.3}) avgLen={:.3} minDot={:.3}",
                    body_stats.normal_avg.x,
                    body_stats.normal_avg.y,
                    body_stats.normal_avg.z,
                    body_stats.normal_avg_len,
                    body_stats.normal_min_dot
                ),
            );
            debug_log.add(
                "Physics/Body/Top/Speed",
                format!(
                    "linear={:.5} angular={:.5}",
                    body_stats.linear_speed, body_stats.angular_speed
                ),
            );
        } else {
            debug_log.add("Physics/Body/Top/Handle", "None");
        }

        if let Some(post) = &snapshot.post_solve_body {
            debug_log.add(
                "Physics/Body/Top/PostSolveRelN",
                format!(
                    "min={:.5} max={:.5} avg={:.5}",
                    post.rel_n_min, post.rel_n_max, post.rel_n_avg
                ),
            );
            debug_log.add(
                "Physics/Body/Top/PostSolveSpeed",
                format!(
                    "linear={:.5} angular={:.5}",
                    post.linear_speed, post.angular_speed
                ),
            );
        }
    }

    /// Add contact debug overlays to the scene.
    pub fn add_contact_overlays(
        &self,
        events: &[ContactEvent],
        overlays: &mut DebugOverlays,
        debug_lines: &mut DebugLines,
    ) {
        if !self.config.draw_contacts {
            return;
        }
        let normal_scale = 0.3;
        let mut smoothed_count = 0usize;
        let mut max_angle_deg = 0.0f32;

        for contact in events {
            let colour = match contact.source {
                ContactSource::Narrowphase => Colour::RED.with_alpha(0.5),
                ContactSource::Ccd => Colour::BLUE.with_alpha(0.5),
            };
            overlays.add_sphere(contact.point, 0.06, colour);
            if contact.normal.magnitude_squared() > 1e-8 {
                overlays.add_line(
                    contact.point,
                    contact.point + contact.normal * normal_scale,
                    colour,
                );
            }
            if self.config.draw_contact_raw_normals
                && contact.source == ContactSource::Narrowphase
                && contact.raw_normal.magnitude_squared() > 1e-8
            {
                let dot = contact.normal.dot(&contact.raw_normal).clamp(-1.0, 1.0);
                let angle_deg = dot.acos() * 180.0 / std::f32::consts::PI;
                smoothed_count += 1;
                max_angle_deg = max_angle_deg.max(angle_deg);
                overlays.add_line(
                    contact.point,
                    contact.point + contact.raw_normal * normal_scale,
                    Colour::YELLOW,
                );
            }
        }

        if self.config.draw_contact_raw_normals {
            debug_lines.add("Contacts/Smoothed", smoothed_count.to_string());
            debug_lines.add(
                "Contacts/MaxNormalDeltaDeg",
                format!("{:.3}", max_angle_deg),
            );
        }
    }

    /// Add sleep state debug overlays to the scene.
    pub fn add_sleep_overlays(
        &self,
        bodies: &Arena<RigidBody>,
        sleeping_handles: &[RigidBodyHandle],
        overlays: &mut DebugOverlays,
    ) {
        if !self.config.draw_sleeping {
            return;
        }
        let colour = Colour::new(0.6, 0.65, 1.0, 1.0);
        for handle in sleeping_handles {
            if let Some(body) = bodies.get(handle.0) {
                let pos = body.position();
                let marker_pos = Point3::new(pos.x, pos.y + 0.6, pos.z);
                overlays.add_sphere(marker_pos, 0.08, colour);
            }
        }
    }
}

// === Internal Debug Statistics Structs ===

#[allow(unused)]
#[derive(Debug, Clone)]
struct ContactSample {
    depth: f32,
    raw_depth: f32,
    rel_n: f32,
    normal_dot_up: f32,
    warm_used: bool,
    point: Point3<f32>,
    normal: Vector3<f32>,
}

#[allow(unused)]
#[derive(Debug, Clone)]
struct ContactDebugStats {
    total: usize,
    static_count: usize,
    neg_raw: usize,
    steep_margin: usize,
    warm_used: usize,
    min_raw: f32,
    max_raw: f32,
    min_depth: f32,
    max_depth: f32,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_avg: f32,
    samples: Vec<ContactSample>,
}

#[allow(unused)]
#[derive(Debug, Clone)]
struct BodyContactStats {
    handle: RigidBodyHandle,
    total: usize,
    static_count: usize,
    min_depth: f32,
    max_depth: f32,
    avg_depth: f32,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_avg: f32,
    normal_avg: Vector3<f32>,
    normal_avg_len: f32,
    normal_min_dot: f32,
    linear_speed: f32,
    angular_speed: f32,
}

#[allow(unused)]
#[derive(Debug, Clone)]
struct BodyPostSolveStats {
    handle: RigidBodyHandle,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_avg: f32,
    linear_speed: f32,
    angular_speed: f32,
}

#[allow(unused)]
#[derive(Clone)]
struct BodyContactAccum {
    total: usize,
    static_count: usize,
    depth_sum: f32,
    min_depth: f32,
    max_depth: f32,
    rel_n_sum: f32,
    rel_n_min: f32,
    rel_n_max: f32,
    rel_n_count: usize,
    normal_sum: Vector3<f32>,
}

impl BodyContactAccum {
    fn new() -> Self {
        Self {
            total: 0,
            static_count: 0,
            depth_sum: 0.0,
            min_depth: f32::INFINITY,
            max_depth: f32::NEG_INFINITY,
            rel_n_sum: 0.0,
            rel_n_min: f32::INFINITY,
            rel_n_max: f32::NEG_INFINITY,
            rel_n_count: 0,
            normal_sum: Vector3::zeros(),
        }
    }

    fn finish(
        self,
    ) -> (
        usize,
        usize,
        f32,
        f32,
        f32,
        f32,
        f32,
        f32,
        Vector3<f32>,
        f32,
    ) {
        let avg_depth = if self.total > 0 {
            self.depth_sum / self.total as f32
        } else {
            0.0
        };
        let rel_n_avg = if self.rel_n_count > 0 {
            self.rel_n_sum / self.rel_n_count as f32
        } else {
            0.0
        };
        let normal_avg_len = if self.total > 0 {
            self.normal_sum.magnitude() / self.total as f32
        } else {
            0.0
        };
        (
            self.total,
            self.static_count,
            self.min_depth,
            self.max_depth,
            avg_depth,
            self.rel_n_min,
            self.rel_n_max,
            rel_n_avg,
            self.normal_sum,
            normal_avg_len,
        )
    }
}

impl ContactDebugStats {
    fn empty() -> Self {
        Self {
            total: 0,
            static_count: 0,
            neg_raw: 0,
            steep_margin: 0,
            warm_used: 0,
            min_raw: 0.0,
            max_raw: 0.0,
            min_depth: 0.0,
            max_depth: 0.0,
            rel_n_min: 0.0,
            rel_n_max: 0.0,
            rel_n_avg: 0.0,
            samples: Vec::new(),
        }
    }
}

#[derive(Debug, Clone)]
struct ContactEventStats {
    total: usize,
    ccd: usize,
    narrow: usize,
}

impl ContactEventStats {
    fn from_events(events: &[ContactEvent]) -> Self {
        let total = events.len();
        let ccd = events
            .iter()
            .filter(|e| e.source == ContactSource::Ccd)
            .count();
        let narrow = events
            .iter()
            .filter(|e| e.source == ContactSource::Narrowphase)
            .count();
        Self { total, ccd, narrow }
    }
}

#[derive(Debug, Clone)]
struct ContactDebugSnapshot {
    raw: ContactDebugStats,
    merged: ContactDebugStats,
    active: ContactDebugStats,
    events: ContactEventStats,
    primary_body: Option<BodyContactStats>,
    post_solve_body: Option<BodyPostSolveStats>,
}

impl ContactDebugSnapshot {
    fn new(
        bodies: &Arena<RigidBody>,
        raw: &[PairManifold],
        merged: &[SolverManifold],
        active: &[SolverManifold],
        events: &[ContactEvent],
    ) -> Self {
        Self {
            raw: compute_raw_stats(bodies, raw),
            merged: compute_solver_stats(bodies, merged),
            active: compute_solver_stats(bodies, active),
            events: ContactEventStats::from_events(events),
            primary_body: compute_primary_body_stats(bodies, active),
            post_solve_body: None,
        }
    }
}

// === Helper Functions ===

/// A flattened contact view used by the debug stats functions.
struct FlatContact<'a> {
    header: &'a PairHeader,
    point: Point3<f32>,
    normal: Vector3<f32>,
    depth: f32,
    raw_depth: f32,
    warm_normal_impulse: f32,
}

fn relative_normal_velocity(
    bodies: &Arena<RigidBody>,
    header: &PairHeader,
    point: Point3<f32>,
    normal: &Vector3<f32>,
) -> Option<f32> {
    let body_b = bodies.get(header.body_b.0)?;
    let (pos_a, vel_a, ang_a) = match header.body_a {
        Some(handle) => {
            let body_a = bodies.get(handle.0)?;
            (
                body_a.position(),
                body_a.linear_velocity(),
                body_a.angular_velocity(),
            )
        }
        None => (point, Vector3::zeros(), Vector3::zeros()),
    };
    let r_a = point - pos_a;
    let r_b = point - body_b.position();
    let vel_at_a = vel_a + ang_a.cross(&r_a);
    let vel_at_b = body_b.linear_velocity() + body_b.angular_velocity().cross(&r_b);
    let rel_vel = vel_at_b - vel_at_a;
    Some(rel_vel.dot(normal))
}

fn compute_stats_from_flat(
    bodies: &Arena<RigidBody>,
    contacts: &[FlatContact],
) -> ContactDebugStats {
    if contacts.is_empty() {
        return ContactDebugStats::empty();
    }

    let total = contacts.len();
    let static_count = contacts
        .iter()
        .filter(|c| c.header.body_a.is_none())
        .count();
    let neg_raw = contacts.iter().filter(|c| c.raw_depth < 0.0).count();
    let warm_used = contacts
        .iter()
        .filter(|c| c.warm_normal_impulse.abs() > 1e-6)
        .count();
    let up = Vector3::y();
    let steep_margin = contacts
        .iter()
        .filter(|c| c.raw_depth < 0.0 && c.normal.dot(&up).abs() < 0.9)
        .count();

    let mut min_raw = f32::INFINITY;
    let mut max_raw = f32::NEG_INFINITY;
    let mut min_depth = f32::INFINITY;
    let mut max_depth = f32::NEG_INFINITY;
    let mut rel_n_min = f32::INFINITY;
    let mut rel_n_max = f32::NEG_INFINITY;
    let mut rel_n_sum = 0.0;
    let mut rel_n_count = 0usize;

    for c in contacts {
        min_raw = min_raw.min(c.raw_depth);
        max_raw = max_raw.max(c.raw_depth);
        min_depth = min_depth.min(c.depth);
        max_depth = max_depth.max(c.depth);
        if let Some(rel_n) = relative_normal_velocity(bodies, c.header, c.point, &c.normal) {
            rel_n_min = rel_n_min.min(rel_n);
            rel_n_max = rel_n_max.max(rel_n);
            rel_n_sum += rel_n;
            rel_n_count += 1;
        }
    }

    let rel_n_avg = if rel_n_count > 0 {
        rel_n_sum / rel_n_count as f32
    } else {
        0.0
    };

    let mut indices: Vec<usize> = (0..contacts.len()).collect();
    indices.sort_by(|a, b| contacts[*b].depth.total_cmp(&contacts[*a].depth));

    let mut samples = Vec::new();
    for idx in indices.into_iter().take(3) {
        let c = &contacts[idx];
        let rel_n = relative_normal_velocity(bodies, c.header, c.point, &c.normal).unwrap_or(0.0);
        let normal_dot_up = c.normal.dot(&up);
        samples.push(ContactSample {
            depth: c.depth,
            raw_depth: c.raw_depth,
            rel_n,
            normal_dot_up,
            warm_used: c.warm_normal_impulse.abs() > 1e-6,
            point: c.point,
            normal: c.normal,
        });
    }

    ContactDebugStats {
        total,
        static_count,
        neg_raw,
        steep_margin,
        warm_used,
        min_raw,
        max_raw,
        min_depth,
        max_depth,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        samples,
    }
}

/// Compute stats from raw `PairManifold` (pre-cache, no warm-start data).
fn compute_raw_stats(bodies: &Arena<RigidBody>, manifolds: &[PairManifold]) -> ContactDebugStats {
    let flat: Vec<FlatContact> = manifolds
        .iter()
        .flat_map(|m| {
            m.manifold.points.iter().map(move |cp| FlatContact {
                header: &m.header,
                point: cp.point,
                normal: cp.normal,
                depth: cp.depth.max(0.0),
                raw_depth: cp.depth,
                warm_normal_impulse: 0.0,
            })
        })
        .collect();
    compute_stats_from_flat(bodies, &flat)
}

/// Compute stats from solver manifolds (post-cache, with warm-start data).
fn compute_solver_stats(
    bodies: &Arena<RigidBody>,
    manifolds: &[SolverManifold],
) -> ContactDebugStats {
    let flat: Vec<FlatContact> = manifolds
        .iter()
        .flat_map(|m| {
            m.contacts.iter().map(move |c| FlatContact {
                header: &m.header,
                point: c.point,
                normal: c.normal,
                depth: c.depth,
                raw_depth: c.raw_depth,
                warm_normal_impulse: c.warm_normal_impulse,
            })
        })
        .collect();
    compute_stats_from_flat(bodies, &flat)
}

fn compute_primary_body_stats(
    bodies: &Arena<RigidBody>,
    manifolds: &[SolverManifold],
) -> Option<BodyContactStats> {
    let has_contacts = manifolds.iter().any(|m| !m.contacts.is_empty());
    if !has_contacts {
        return None;
    }

    let mut per_body: FxHashMap<RigidBodyHandle, BodyContactAccum> = FxHashMap::default();
    for m in manifolds {
        for c in &m.contacts {
            let entry = per_body
                .entry(m.header.body_b)
                .or_insert_with(BodyContactAccum::new);
            entry.total += 1;
            if m.header.body_a.is_none() {
                entry.static_count += 1;
            }
            entry.depth_sum += c.depth;
            entry.min_depth = entry.min_depth.min(c.depth);
            entry.max_depth = entry.max_depth.max(c.depth);
            if let Some(rel_n) = relative_normal_velocity(bodies, &m.header, c.point, &c.normal) {
                entry.rel_n_sum += rel_n;
                entry.rel_n_min = entry.rel_n_min.min(rel_n);
                entry.rel_n_max = entry.rel_n_max.max(rel_n);
                entry.rel_n_count += 1;
            }
            entry.normal_sum += c.normal;
        }
    }

    let (&handle, accum) = per_body.iter().max_by_key(|(_, stats)| stats.total)?;
    let (
        total,
        static_count,
        min_depth,
        max_depth,
        avg_depth,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        normal_sum,
        normal_avg_len,
    ) = accum.clone().finish();

    let normal_avg = if normal_sum.magnitude() > 1e-6 {
        normal_sum.normalize()
    } else {
        Vector3::zeros()
    };

    let mut normal_min_dot = 1.0f32;
    if normal_avg.magnitude() > 1e-6 {
        for m in manifolds {
            if m.header.body_b != handle {
                continue;
            }
            for c in &m.contacts {
                normal_min_dot = normal_min_dot.min(c.normal.dot(&normal_avg));
            }
        }
    } else {
        normal_min_dot = 0.0;
    }

    let body = bodies.get(handle.0)?;
    let linear_speed = body.linear_velocity().magnitude();
    let angular_speed = body.angular_velocity().magnitude();

    Some(BodyContactStats {
        handle,
        total,
        static_count,
        min_depth,
        max_depth,
        avg_depth,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        normal_avg,
        normal_avg_len,
        normal_min_dot,
        linear_speed,
        angular_speed,
    })
}

fn compute_post_solve_body_stats(
    bodies: &Arena<RigidBody>,
    manifolds: &[SolverManifold],
    handle: RigidBodyHandle,
) -> Option<BodyPostSolveStats> {
    let mut rel_n_min = f32::INFINITY;
    let mut rel_n_max = f32::NEG_INFINITY;
    let mut rel_n_sum = 0.0;
    let mut rel_n_count = 0usize;

    for m in manifolds {
        if m.header.body_b != handle {
            continue;
        }
        for c in &m.contacts {
            if let Some(rel_n) = relative_normal_velocity(bodies, &m.header, c.point, &c.normal) {
                rel_n_min = rel_n_min.min(rel_n);
                rel_n_max = rel_n_max.max(rel_n);
                rel_n_sum += rel_n;
                rel_n_count += 1;
            }
        }
    }

    if rel_n_count == 0 {
        return None;
    }

    let rel_n_avg = rel_n_sum / rel_n_count as f32;
    let body = bodies.get(handle.0)?;
    let linear_speed = body.linear_velocity().magnitude();
    let angular_speed = body.angular_velocity().magnitude();

    Some(BodyPostSolveStats {
        handle,
        rel_n_min,
        rel_n_max,
        rel_n_avg,
        linear_speed,
        angular_speed,
    })
}
