use std::cell::{Cell, RefCell};

use nalgebra::{Point3, UnitQuaternion, Vector3};
use rustc_hash::FxHashMap;
use traversal_planner::section::{sample_section, CastHit, Section, SphereCaster, StripLayout};
use traversal_planner::GaitParams;

use crate::physics::{PhysicsWorld, RigidBodyHandle};

use super::path_frame::PathFrame;
use super::physics_caster::{is_resting, IN_FLIGHT_SPEED};

/// How far a body moves before the casts around it are dropped, in metres.
const MOVED: f32 = 0.01;
/// How far it turns before they are, in radians: a centimetre at the corner
/// of a crate.
const TURNED: f32 = 0.0175;

/// A body's place when the casts around it were last dropped.
#[derive(Clone, Copy)]
struct Pose {
    position: Point3<f32>,
    rotation: UnitQuaternion<f32>,
    /// Radius of a sphere round the body, in metres.
    reach: f32,
    /// Whether the strip's casts see it: not in flight.
    settled: bool,
    /// Whether its casts read it as footing to stand on: resting, or a
    /// carrier.
    resting: bool,
}

impl Pose {
    fn moved_from(&self, now: &Pose) -> bool {
        (now.position - self.position).norm() > MOVED
            || now.rotation.angle_to(&self.rotation) > TURNED
    }
}

/// How far behind the strip's back end kept casts are dropped, in metres.
const STALE_BEHIND: f32 = 0.5;

/// A cast the strip makes: which way, where on the path and from what
/// height, how wide. The strip's stations lie on a grid fixed along the path,
/// and each starts from the floor sampled before it, so the same station
/// casts with bit-identical numbers every frame while nothing near it moves.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct CastKey {
    up: bool,
    s: u32,
    d: u32,
    radius: u32,
    from: u32,
}

impl CastKey {
    fn new(up: bool, s: f32, d: f32, radius: f32, from: f32) -> Self {
        Self {
            up,
            s: s.to_bits(),
            d: d.to_bits(),
            radius: radius.to_bits(),
            from: from.to_bits(),
        }
    }
}

/// The strip's casts kept from frame to frame along one path, so a frame
/// casts only the stations new in reach and those near something that moved.
///
/// ```text
///   frame N:   [ cast every station ]
///   frame N+1:   [ kept ..................... | new ]   body moved on
///                       [ recast ]                     a stone moved here
/// ```
///
/// Kept casts are dropped around a body each time it has moved a centimetre,
/// where the terrain changed, and all at once when the path is laid afresh.
pub struct SectionCache {
    frame: Option<PathFrame>,
    hits: FxHashMap<CastKey, Option<CastHit>>,
    /// Where each body awake in the world was when the casts around it were
    /// last dropped.
    poses: FxHashMap<RigidBodyHandle, Pose>,
}

impl Default for SectionCache {
    fn default() -> Self {
        Self {
            frame: None,
            hits: FxHashMap::default(),
            poses: FxHashMap::default(),
        }
    }
}

/// A caster that answers from the cache, and casts only on a miss.
struct Remembering<'a, C> {
    caster: &'a C,
    hits: &'a RefCell<FxHashMap<CastKey, Option<CastHit>>>,
    casts: &'a Cell<usize>,
}

impl<C: SphereCaster> Remembering<'_, C> {
    fn cast(&self, key: CastKey, cast: impl FnOnce() -> Option<CastHit>) -> Option<CastHit> {
        if let Some(hit) = self.hits.borrow().get(&key) {
            return *hit;
        }
        let hit = cast();
        self.casts.set(self.casts.get() + 1);
        self.hits.borrow_mut().insert(key, hit);
        hit
    }
}

impl<C: SphereCaster> SphereCaster for Remembering<'_, C> {
    fn cast_down(&self, s: f32, d: f32, radius: f32, from: f32) -> Option<CastHit> {
        self.cast(CastKey::new(false, s, d, radius, from), || {
            self.caster.cast_down(s, d, radius, from)
        })
    }

    fn cast_up(&self, s: f32, d: f32, radius: f32, from: f32) -> Option<CastHit> {
        self.cast(CastKey::new(true, s, d, radius, from), || {
            self.caster.cast_up(s, d, radius, from)
        })
    }
}

impl SectionCache {
    /// The path the kept casts were made along.
    pub fn frame(&self) -> Option<&PathFrame> {
        self.frame.as_ref()
    }

    /// Starts afresh along `frame`, dropping every kept cast.
    pub fn lay_path(&mut self, frame: PathFrame) {
        self.frame = Some(frame);
        self.hits.clear();
    }

    /// Drops what moving bodies (other than `ignore`) and changed terrain
    /// may have made stale. A body is forgotten around once it has moved or
    /// turned past [`MOVED`] or [`TURNED`] since it last was, both where it
    /// was then and where it is now; till then its casts are good to within
    /// that. A body in flight ([`IN_FLIGHT_SPEED`]) is not seen by the casts:
    /// it is forgotten around where it took off and where it slows again,
    /// not on the way. One that comes to rest or starts moving is forgotten
    /// around, as the casts' footing says which it is. A carrier is always
    /// seen and always firm, so only its moving makes casts stale. A body
    /// that was removed gets a last look.
    pub fn forget_what_moved(
        &mut self,
        world: &PhysicsWorld,
        ignore: &[RigidBodyHandle],
        carriers: &[RigidBodyHandle],
        terrain_changes: &[(Point3<f32>, Point3<f32>)],
    ) {
        let mut stale: Vec<(Point3<f32>, f32)> = Vec::new();
        let mut seen = Vec::new();
        for (index, body) in world.bodies().iter() {
            let handle = RigidBodyHandle(index);
            if ignore.contains(&handle) || !body.is_dynamic() {
                continue;
            }
            let awake = !world.is_sleeping(handle);
            if !awake && !self.poses.contains_key(&handle) {
                continue;
            }
            seen.push(handle);
            let carrier = carriers.contains(&handle);
            let now = Pose {
                position: body.position(),
                rotation: body.rotation(),
                reach: world.reach(handle),
                settled: carrier || !awake || world.surface_speed(handle) <= IN_FLIGHT_SPEED,
                resting: carrier || is_resting(world, handle),
            };
            match self.poses.get(&handle) {
                None => {
                    // Woken this frame, where the casts saw it at rest.
                    if !now.settled || !now.resting {
                        stale.push((now.position, now.reach));
                    }
                    self.poses.insert(handle, now);
                }
                Some(then)
                    if then.settled != now.settled
                        || then.resting != now.resting
                        || (now.settled && then.moved_from(&now)) =>
                {
                    stale.push((then.position, then.reach));
                    stale.push((now.position, now.reach));
                    self.poses.insert(handle, now);
                }
                Some(_) => {}
            }
            if !awake {
                self.poses.remove(&handle);
            }
        }
        self.poses.retain(|handle, then| {
            let kept = seen.contains(handle);
            if !kept {
                stale.push((then.position, then.reach));
            }
            kept
        });
        let boxes = stale
            .into_iter()
            .map(|(centre, radius)| {
                let reach = Vector3::new(radius, radius, radius);
                (centre - reach, centre + reach)
            })
            .chain(terrain_changes.iter().copied())
            .collect::<Vec<_>>();
        for (low, high) in boxes {
            self.forget_box(low, high);
        }
    }

    /// Drops the kept casts whose column could touch the world box from
    /// `low` to `high`.
    fn forget_box(&mut self, low: Point3<f32>, high: Point3<f32>) {
        let Some(frame) = self.frame else {
            return;
        };
        let corners = [
            Point3::new(low.x, low.y, low.z),
            Point3::new(high.x, low.y, low.z),
            Point3::new(low.x, low.y, high.z),
            Point3::new(high.x, low.y, high.z),
        ];
        let (mut s, mut d) = ((f32::MAX, f32::MIN), (f32::MAX, f32::MIN));
        for corner in corners {
            let at = frame.path_point(corner);
            s = (s.0.min(at.s), s.1.max(at.s));
            d = (d.0.min(at.d), d.1.max(at.d));
        }
        self.hits.retain(|key, _| {
            // A sphere cast at a station reaches its radius around it.
            let radius = f32::from_bits(key.radius);
            let (cast_s, cast_d) = (f32::from_bits(key.s), f32::from_bits(key.d));
            cast_s + radius < s.0
                || cast_s - radius > s.1
                || cast_d + radius < d.0
                || cast_d - radius > d.1
        });
    }

    /// The strip around `origin` along the laid path, in the layer of a body
    /// whose bottom-sphere centre is at `body_z`, casting only what is not
    /// kept. Returns it and how many casts it made.
    pub fn sample(
        &mut self,
        caster: &impl SphereCaster,
        layout: &StripLayout,
        params: &GaitParams,
        origin: f32,
        body_z: f32,
    ) -> (Section, usize) {
        // Stations the body has left behind are not coming back.
        let behind = origin - layout.behind - STALE_BEHIND;
        self.hits.retain(|key, _| f32::from_bits(key.s) >= behind);
        // Moved into a cell for the sampling, which casts through `&self`.
        let hits = RefCell::new(std::mem::take(&mut self.hits));
        let casts = Cell::new(0);
        let remembering = Remembering {
            caster,
            hits: &hits,
            casts: &casts,
        };
        let section = sample_section(&remembering, layout, params, origin, body_z);
        self.hits = hits.into_inner();
        (section, casts.get())
    }
}
