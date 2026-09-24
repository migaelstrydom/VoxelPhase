//! Choosing what a grab reaches for.
//!
//! A single ray from the capsule's centre only sees things at waist height:
//! anything shorter than the capsule's half-height, lying on the ground the
//! character stands on, passes underneath it. The probe instead looks at
//! everything inside a reach volume and picks one:
//!
//! ```text
//!   gather ─► every dynamic collider whose bounds overlap the reach volume
//!   score  ─► distance ahead, plus penalties for sitting off to the side or low
//!   aim    ─► ray from the origin to the collider's centre; the best candidate
//!             whose ray lands on its own body, inside the volume, wins
//! ```

use nalgebra::{Point3, Vector3};

use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// The reach volume and the rule that picks one body out of it.
#[derive(Debug, Clone)]
pub struct GrabProbe {
    /// How far ahead of the origin the reach volume extends (m).
    pub range: f32,
    /// Half the reach volume's width across the facing (m).
    pub half_width: f32,
    /// How far above the origin the volume reaches (m).
    pub height_above: f32,
    /// Gap between the soles and the bottom of the volume (m). A hit this
    /// close to the soles is the top of whatever the character stands on, and
    /// that is never what a grab means.
    pub floor_clearance: f32,
    /// Score added per metre a candidate sits off to the side. Above 1, a
    /// thing straight ahead beats a nearer one beside it.
    pub side_weight: f32,
    /// Score added per metre a candidate sits below the origin. Small: it
    /// only breaks near-ties in favour of what is at hand height.
    pub low_weight: f32,
}

impl Default for GrabProbe {
    fn default() -> Self {
        Self {
            range: 2.0,
            half_width: 0.5,
            height_above: 0.6,
            floor_clearance: 0.05,
            side_weight: 1.5,
            low_weight: 0.25,
        }
    }
}

/// Where a reach starts and which way it faces.
///
/// Local coordinates are `(ahead, side, up)`: `ahead` along the horizontal
/// facing, `side` across it, `up` along world Y.
#[derive(Debug, Clone, Copy)]
pub struct ReachFrame {
    /// Where the reach starts: the capsule's centre.
    pub origin: Point3<f32>,
    /// Horizontal unit facing.
    pub ahead: Vector3<f32>,
    /// Horizontal unit vector across the facing.
    pub side: Vector3<f32>,
    /// Distance from the origin down to the soles (m).
    pub ground_depth: f32,
}

impl ReachFrame {
    /// A frame at `origin` looking along the horizontal part of `facing`.
    /// Returns `None` when `facing` has no horizontal part to look along.
    pub fn new(origin: Point3<f32>, facing: Vector3<f32>, ground_depth: f32) -> Option<Self> {
        let ahead = Vector3::new(facing.x, 0.0, facing.z).try_normalize(1.0e-6)?;
        let side = Vector3::y().cross(&ahead);
        Some(Self {
            origin,
            ahead,
            side,
            ground_depth,
        })
    }

    /// `point` as `(ahead, side, up)` relative to the origin.
    pub fn to_local(&self, point: Point3<f32>) -> Vector3<f32> {
        let d = point - self.origin;
        Vector3::new(d.dot(&self.ahead), d.dot(&self.side), d.y)
    }

    /// A local `(ahead, side, up)` point back in world space.
    pub fn to_world(&self, local: Vector3<f32>) -> Point3<f32> {
        self.origin + self.ahead * local.x + self.side * local.y + Vector3::y() * local.z
    }
}

/// A collider in the reach volume, before it is aimed at.
struct Candidate {
    body: RigidBodyHandle,
    centre: Point3<f32>,
    score: f32,
}

impl GrabProbe {
    /// The body to grab and the surface point to hold it by, if anything is
    /// within reach. `exclude` holds the character's own body.
    pub fn find_target(
        &self,
        physics: &PhysicsWorld,
        frame: &ReachFrame,
        exclude: &[RigidBodyHandle],
    ) -> Option<(RigidBodyHandle, Point3<f32>)> {
        let mut candidates = self.gather(physics, frame, exclude);
        candidates.sort_by(|a, b| a.score.total_cmp(&b.score));
        candidates
            .iter()
            .find_map(|c| self.aim(physics, frame, c, exclude))
    }

    /// The reach volume's local bounds, `(min, max)` in `(ahead, side, up)`.
    pub fn local_bounds(&self, frame: &ReachFrame) -> (Vector3<f32>, Vector3<f32>) {
        let floor = -(frame.ground_depth - self.floor_clearance);
        (
            Vector3::new(0.0, -self.half_width, floor),
            Vector3::new(self.range, self.half_width, self.height_above),
        )
    }

    /// The reach volume's eight corners in world space, for debug drawing.
    /// Bit 0 of the index picks `ahead`, bit 1 `side`, bit 2 `up`.
    pub fn corners(&self, frame: &ReachFrame) -> [Point3<f32>; 8] {
        let (min, max) = self.local_bounds(frame);
        std::array::from_fn(|i| {
            let pick = |bit: usize, lo: f32, hi: f32| if i & (1 << bit) == 0 { lo } else { hi };
            frame.to_world(Vector3::new(
                pick(0, min.x, max.x),
                pick(1, min.y, max.y),
                pick(2, min.z, max.z),
            ))
        })
    }

    /// Every dynamic collider in front of the origin whose bounding sphere
    /// overlaps the reach volume, scored.
    fn gather(
        &self,
        physics: &PhysicsWorld,
        frame: &ReachFrame,
        exclude: &[RigidBodyHandle],
    ) -> Vec<Candidate> {
        let (min, max) = self.local_bounds(frame);
        let mut candidates = Vec::new();

        for (idx, body) in physics.bodies().iter() {
            let handle = RigidBodyHandle(idx);
            if !body.is_dynamic() || exclude.contains(&handle) {
                continue;
            }
            for ch in body.colliders() {
                let Some(collider) = physics.colliders_arena().get(ch.0) else {
                    continue;
                };
                let centre = collider.world_center(body.position(), body.rotation());
                let local = frame.to_local(centre);
                if local.x <= 0.0 {
                    continue;
                }
                let nearest = local.sup(&min).inf(&max);
                if (local - nearest).norm() > collider.shape().bounding_radius() {
                    continue;
                }
                candidates.push(Candidate {
                    body: handle,
                    centre,
                    score: self.score(local),
                });
            }
        }

        candidates
    }

    /// Lower is better: distance ahead, plus penalties for sitting off to the
    /// side or below the origin.
    fn score(&self, local: Vector3<f32>) -> f32 {
        local.x + self.side_weight * local.y.abs() + self.low_weight * (-local.z).max(0.0)
    }

    /// Ray from the origin to the candidate's centre. Holds only if the first
    /// body it meets is the candidate's own and the hit is inside the volume:
    /// otherwise something else is in the way, or the only part in reach is
    /// the top the character is standing on.
    fn aim(
        &self,
        physics: &PhysicsWorld,
        frame: &ReachFrame,
        candidate: &Candidate,
        exclude: &[RigidBodyHandle],
    ) -> Option<(RigidBodyHandle, Point3<f32>)> {
        let to_centre = candidate.centre - frame.origin;
        let length = to_centre.norm();
        let direction = to_centre.try_normalize(1.0e-6)?;
        let hit = physics.probe_bodies(frame.origin, direction, length, exclude)?;
        if hit.body != candidate.body {
            return None;
        }

        let (min, max) = self.local_bounds(frame);
        let local = frame.to_local(hit.hit.point);
        let inside = (0..3).all(|i| local[i] >= min[i] && local[i] <= max[i]);
        inside.then_some((candidate.body, hit.hit.point))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    /// The player's capsule: centre 0.5 m above the floor at y = 0.
    const GROUND_DEPTH: f32 = 0.5;

    fn frame() -> ReachFrame {
        ReachFrame::new(
            Point3::new(0.0, GROUND_DEPTH, 0.0),
            Vector3::z(),
            GROUND_DEPTH,
        )
        .unwrap()
    }

    fn cube(world: &mut PhysicsWorld, centre: Point3<f32>, half: f32) -> RigidBodyHandle {
        let handle = world.create_body(RigidBodyDesc::dynamic().position(centre));
        world.attach_collider(
            handle,
            ColliderDesc::box_shape(Vector3::repeat(half)).density(1000.0),
        );
        handle
    }

    #[test]
    fn a_knee_high_cube_on_the_floor_is_found() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let metal = cube(&mut world, Point3::new(0.0, 0.15, 1.0), 0.15);

        let (body, point) = GrabProbe::default()
            .find_target(&world, &frame(), &[])
            .expect("a 0.3 m cube a metre ahead is within reach");

        assert_eq!(body, metal);
        assert!(
            point.y < 0.31,
            "held by its top or front, got y = {}",
            point.y
        );
    }

    #[test]
    fn at_the_same_distance_the_one_at_hand_height_wins() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        cube(&mut world, Point3::new(0.2, 0.15, 1.2), 0.15);
        let chest_high = cube(&mut world, Point3::new(-0.2, 0.6, 1.2), 0.15);

        let (body, _) = GrabProbe::default()
            .find_target(&world, &frame(), &[])
            .unwrap();

        assert_eq!(body, chest_high);
    }

    #[test]
    fn a_nearer_floor_cube_beats_a_farther_chest_high_one() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let near = cube(&mut world, Point3::new(0.0, 0.15, 0.8), 0.15);
        cube(&mut world, Point3::new(0.0, 0.6, 1.8), 0.15);

        let (body, _) = GrabProbe::default()
            .find_target(&world, &frame(), &[])
            .unwrap();

        assert_eq!(body, near);
    }

    #[test]
    fn the_crate_underfoot_is_never_picked() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        // A crate whose top is exactly at the soles, reaching out in front.
        cube(&mut world, Point3::new(0.0, -0.5, 0.3), 0.5);

        assert!(GrabProbe::default()
            .find_target(&world, &frame(), &[])
            .is_none());
    }

    #[test]
    fn nothing_behind_or_out_of_range_is_found() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        cube(&mut world, Point3::new(0.0, 0.5, -1.0), 0.15);
        cube(&mut world, Point3::new(0.0, 0.5, 3.0), 0.15);
        cube(&mut world, Point3::new(1.5, 0.5, 1.0), 0.15);

        assert!(GrabProbe::default()
            .find_target(&world, &frame(), &[])
            .is_none());
    }

    #[test]
    fn an_excluded_body_is_ignored() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let own = cube(&mut world, Point3::new(0.0, 0.5, 0.5), 0.15);

        assert!(GrabProbe::default()
            .find_target(&world, &frame(), &[own])
            .is_none());
    }

    #[test]
    fn a_blocked_candidate_yields_to_the_blocker() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        // The floor cube straight ahead scores best, but a crate off to the
        // side stands across the ray aimed at it.
        cube(&mut world, Point3::new(0.0, 0.15, 1.2), 0.15);
        let blocker = cube(&mut world, Point3::new(0.3, 0.35, 0.9), 0.35);

        let (body, _) = GrabProbe::default()
            .find_target(&world, &frame(), &[])
            .unwrap();

        assert_eq!(body, blocker);
    }
}
