use nalgebra::{Point3, Vector3};

use crate::physics::{
    CastObstacles, CollisionFilter, Layers, PhysicsWorld, SphereCast, StaticGeometry,
};

/// Keeps the camera out of the scenery, and the scenery from hiding what it
/// looks at for long.
///
/// ```text
///   where the follow wants it ──► sight line ──► reach ──► sweep ──► camera
///                                 (pivot →       (cut short, (from where
///                                  camera)        eased out)  it was)
/// ```
///
/// Two jobs, answered differently:
///
/// - **The lens never enters rock.** The camera is swept as a sphere from
///   where it was to where it is going, stops where it touches, and slides
///   along the surface with the rest of its motion. Only what lies in the
///   camera's own path moves it.
/// - **What blocks the view is waited out.** A sight line is cast from the
///   pivot to the camera. Something thin across it — a stalagmite, a pillar
///   — is ignored, and something thick is only acted on once it has blocked
///   the view for a moment. Then the camera cuts through it to its near
///   side: a blocker always stands between camera and pivot, so there is no
///   smooth way in front of it that keeps the lens out of rock. Once the view
///   clears, the camera eases back out along a critically damped spring,
///   swept like any other move.
///
/// Only the scenery counts: terrain and static bodies. Crates, creatures and
/// rubble passing between are seen through rather than chased.
#[derive(Debug, Clone)]
pub struct CameraClearance {
    /// Whether the camera keeps clear of the scenery at all. Off by default:
    /// in play it zoomed out sluggishly and snapped and jittered in tunnels
    /// (`docs/CAMERA_CLEARANCE.md`). Off, the camera passes through terrain,
    /// which is also how geometry is inspected from any angle.
    pub enabled: bool,
    /// Radius of the sphere the camera is swept as, in metres. It must exceed
    /// the near plane's half-extent so the near plane stays out of what the
    /// sphere touches.
    pub radius: f32,
    /// How long something must block the view, in seconds, before the
    /// camera is drawn in front of it.
    pub occlusion_delay: f32,
    /// How thick, in metres along the sight line, a blocker must be to draw
    /// the camera in at all. Thinner ones are looked past.
    pub thin_occluder: f32,
    /// Time constant, in seconds, of the reach opening back out once the
    /// view is clear.
    pub ease_out_time: f32,
    /// How many times a frame the camera may slide along what it runs into.
    pub slide_passes: u32,
}

impl Default for CameraClearance {
    fn default() -> Self {
        Self {
            enabled: false,
            radius: 0.3,
            occlusion_delay: 0.25,
            thin_occluder: 1.0,
            ease_out_time: 0.5,
            slide_passes: 2,
        }
    }
}

/// What the camera keeps clear of: the static geometry, and the static bodies
/// among the physics world's.
#[derive(Clone, Copy)]
pub struct Scenery<'a> {
    /// The world whose static bodies count as scenery.
    pub physics: &'a PhysicsWorld,
    /// The level's static geometry.
    pub geometry: &'a dyn StaticGeometry,
}

/// What a camera's clearance carries from one frame to the next.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClearanceState {
    /// How long a thick blocker has stood across the sight line, in seconds.
    occluded_for: f32,
    /// How far from the pivot the camera may stand, in metres; `None` until
    /// the first frame sets it to the full distance. Cut short in front of a
    /// blocker, then eased back out.
    reach: Option<f32>,
    /// How fast the reach is changing, in m/s: the spring's velocity.
    reach_rate: f32,
}

/// How far short of a surface a sweep stops, so the next frame's sweep does
/// not start touching it.
const SKIN: f32 = 0.01;

impl CameraClearance {
    /// Where the camera stands this frame, having stood at `from`, when the
    /// follow would put it at `wanted` looking at `pivot`.
    pub fn resolve(
        &self,
        state: &mut ClearanceState,
        scenery: &Scenery<'_>,
        pivot: Point3<f32>,
        from: Point3<f32>,
        wanted: Point3<f32>,
        dt: f32,
    ) -> Point3<f32> {
        if !self.enabled {
            *state = ClearanceState::default();
            return wanted;
        }
        let offset = wanted - pivot;
        let length = offset.norm();
        if length <= f32::EPSILON {
            return wanted;
        }

        let blocker = self.blocker(scenery, pivot, wanted);
        state.occluded_for = match blocker {
            Some(_) => state.occluded_for + dt,
            None => 0.0,
        };
        let direction = offset / length;
        if let Some(near) = blocker.filter(|_| state.occluded_for >= self.occlusion_delay) {
            // In front of the blocker is clear by construction: the sight
            // line reached it from the pivot without touching anything.
            state.reach = Some(near);
            state.reach_rate = 0.0;
            return pivot + direction * near;
        }

        let reach = self.eased_reach(state, length, dt);
        let placed = pivot + direction * reach.min(length);
        self.sweep_and_slide(scenery, from, placed)
    }

    /// How far along the sight line from `pivot` to `camera` the camera can
    /// stand clear of a blocker thick enough to act on; `None` when the view
    /// is clear or only something thin crosses it.
    ///
    /// The thickness is measured by casting back from the camera: the face it
    /// meets is the far side of what blocks the view. Nothing met means the
    /// camera end is itself inside the scenery, which is as thick as a
    /// blocker gets.
    fn blocker(
        &self,
        scenery: &Scenery<'_>,
        pivot: Point3<f32>,
        camera: Point3<f32>,
    ) -> Option<f32> {
        let length = (camera - pivot).norm();
        let near = self.cast(scenery, pivot, camera)?;
        let far_side = self
            .cast(scenery, camera, pivot)
            .map_or(length, |back| length - back);
        // Both touches are sphere centres, a radius off either face.
        let thickness = far_side - near - 2.0 * self.radius;
        (thickness >= self.thin_occluder).then_some(near)
    }

    /// Open the reach back out toward `full` along a critically damped
    /// spring.
    fn eased_reach(&self, state: &mut ClearanceState, full: f32, dt: f32) -> f32 {
        let current = state.reach.unwrap_or(full);
        let reach = if current >= full {
            state.reach_rate = 0.0;
            full
        } else {
            smooth_damp(current, full, &mut state.reach_rate, self.ease_out_time, dt)
        };
        state.reach = Some(reach);
        reach
    }

    /// Move a sphere from `from` toward `to`, stopping where it touches the
    /// scenery and sliding the rest of the way along what it touched.
    fn sweep_and_slide(
        &self,
        scenery: &Scenery<'_>,
        from: Point3<f32>,
        to: Point3<f32>,
    ) -> Point3<f32> {
        let mut at = from;
        let mut remaining = to - from;
        for _ in 0..=self.slide_passes {
            let length = remaining.norm();
            if length <= f32::EPSILON {
                break;
            }
            let direction = remaining / length;
            let Some(hit) = self.cast_hit(scenery, at, direction, length) else {
                return at + remaining;
            };
            at += direction * (hit.distance - SKIN).max(0.0);
            let left = remaining - direction * hit.distance;
            remaining = left - hit.normal * left.dot(&hit.normal);
        }
        at
    }

    /// How far a sphere travels from `from` toward `to` before touching the
    /// scenery; `None` if it gets there.
    fn cast(&self, scenery: &Scenery<'_>, from: Point3<f32>, to: Point3<f32>) -> Option<f32> {
        let offset = to - from;
        let length = offset.norm();
        if length <= f32::EPSILON {
            return None;
        }
        self.cast_hit(scenery, from, offset / length, length)
            .map(|hit| hit.distance)
    }

    /// The first touch of a sphere cast `length` along `direction`.
    fn cast_hit(
        &self,
        scenery: &Scenery<'_>,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<crate::physics::SphereCastHit> {
        scenery.physics.cast_sphere(
            scenery.geometry,
            &SphereCast {
                origin,
                direction,
                length,
                radius: self.radius,
                exclude: &[],
                solid_bodies: false,
                filter: CollisionFilter::seeing(Layers::ALL),
                obstacles: CastObstacles::Scenery,
            },
        )
    }
}

/// A critically damped spring from `current` toward `target` with time
/// constant `time`, advanced by `dt`; `rate` is its velocity, carried between
/// calls. Never overshoots.
fn smooth_damp(current: f32, target: f32, rate: &mut f32, time: f32, dt: f32) -> f32 {
    let omega = 2.0 / time.max(1.0e-4);
    let x = omega * dt;
    let decay = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
    let change = current - target;
    let pull = (*rate + omega * change) * dt;
    *rate = (*rate - omega * pull) * decay;
    let next = target + (change + pull) * decay;
    if (target - current > 0.0) == (next > target) {
        *rate = 0.0;
        return target;
    }
    next
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::bench_harness::geometry::{EmptyGeometry, WallAndFloorGeometry};
    use crate::physics::{ColliderDesc, RigidBodyDesc};

    const PIVOT: Point3<f32> = Point3::new(2.0, 1.0, 0.0);
    const DT: f32 = 1.0 / 60.0;

    /// The clearance switched on, which is what most tests examine.
    fn active() -> CameraClearance {
        CameraClearance {
            enabled: true,
            ..CameraClearance::default()
        }
    }

    fn scene<'a>(physics: &'a PhysicsWorld, geometry: &'a dyn StaticGeometry) -> Scenery<'a> {
        Scenery { physics, geometry }
    }

    fn world_with_box(desc: RigidBodyDesc, half_extents: Vector3<f32>) -> PhysicsWorld {
        let mut world = PhysicsWorld::default();
        let body = world.create_body(desc.position(Point3::new(0.0, 1.0, 0.0)));
        world.attach_collider(body, ColliderDesc::box_shape(half_extents));
        world
    }

    /// Hold the camera's wanted position still for `seconds`, starting from
    /// `wanted` itself, and return where the camera ends up.
    fn settle(
        clearance: &CameraClearance,
        world: &PhysicsWorld,
        scenery: &dyn StaticGeometry,
        wanted: Point3<f32>,
        seconds: f32,
    ) -> Point3<f32> {
        let mut state = ClearanceState::default();
        let mut camera = wanted;
        for _ in 0..(seconds / DT).round() as u32 {
            camera = clearance.resolve(
                &mut state,
                &scene(world, scenery),
                PIVOT,
                camera,
                wanted,
                DT,
            );
        }
        camera
    }

    #[test]
    fn a_clear_view_leaves_the_camera_where_it_is() {
        let world = PhysicsWorld::default();
        let wall = WallAndFloorGeometry::new(10.0, 4.0);
        let wanted = Point3::new(6.0, 1.5, 1.0);
        let camera = settle(&active(), &world, &wall, wanted, 1.0);
        assert!((camera - wanted).norm() < 1.0e-4, "camera at {camera:?}");
    }

    #[test]
    fn a_camera_moving_into_rock_stops_at_its_surface() {
        let world = PhysicsWorld::default();
        let wall = WallAndFloorGeometry::new(10.0, 4.0);
        let clearance = active();
        let mut state = ClearanceState::default();
        let camera = clearance.resolve(
            &mut state,
            &scene(&world, &wall),
            PIVOT,
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-3.0, 1.0, 0.0),
            DT,
        );
        assert!(
            camera.x >= clearance.radius && camera.x < clearance.radius + 0.05,
            "camera at {camera:?}"
        );
    }

    #[test]
    fn a_camera_driven_slantwise_into_rock_slides_along_it() {
        let world = PhysicsWorld::default();
        let wall = WallAndFloorGeometry::new(10.0, 4.0);
        let clearance = CameraClearance {
            occlusion_delay: f32::INFINITY,
            ..active()
        };
        let mut state = ClearanceState::default();
        let camera = clearance.resolve(
            &mut state,
            &scene(&world, &wall),
            PIVOT,
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 2.0),
            DT,
        );
        assert!(
            camera.x >= clearance.radius - 1.0e-3,
            "inside the wall at {camera:?}"
        );
        assert!(camera.z > 1.5, "stuck rather than sliding: {camera:?}");
    }

    #[test]
    fn a_thin_occluder_is_looked_past() {
        let world = world_with_box(RigidBodyDesc::static_body(), Vector3::new(0.2, 0.5, 0.2));
        let wanted = Point3::new(-3.0, 1.0, 0.0);
        let camera = settle(&active(), &world, &EmptyGeometry, wanted, 2.0);
        assert!((camera - wanted).norm() < 1.0e-4, "camera at {camera:?}");
    }

    #[test]
    fn a_thick_occluder_draws_the_camera_in_only_after_the_delay() {
        let world = world_with_box(RigidBodyDesc::static_body(), Vector3::new(1.0, 0.5, 1.0));
        let clearance = active();
        let wanted = Point3::new(-3.0, 1.0, 0.0);

        let early = settle(&clearance, &world, &EmptyGeometry, wanted, 0.2);
        assert!(
            (early - wanted).norm() < 1.0e-4,
            "moved before the delay: {early:?}"
        );

        let late = settle(&clearance, &world, &EmptyGeometry, wanted, 2.0);
        assert!(
            late.x > 1.0 && late.x < 1.0 + clearance.radius + 0.05,
            "not drawn in front of the box: {late:?}"
        );
    }

    #[test]
    fn once_the_view_clears_the_camera_eases_back_out() {
        let mut world = PhysicsWorld::default();
        let wall =
            world.create_body(RigidBodyDesc::static_body().position(Point3::new(0.0, 1.0, 0.0)));
        world.attach_collider(wall, ColliderDesc::box_shape(Vector3::new(1.0, 0.5, 1.0)));
        let clearance = active();
        let wanted = Point3::new(-3.0, 1.0, 0.0);
        let mut state = ClearanceState::default();
        let mut camera = wanted;
        for _ in 0..60 {
            camera = clearance.resolve(
                &mut state,
                &scene(&world, &EmptyGeometry),
                PIVOT,
                camera,
                wanted,
                DT,
            );
        }
        assert!(camera.x > 1.0, "never drawn in: {camera:?}");

        world.remove_body(wall);
        let mut fastest = 0.0f32;
        for _ in 0..120 {
            let next = clearance.resolve(
                &mut state,
                &scene(&world, &EmptyGeometry),
                PIVOT,
                camera,
                wanted,
                DT,
            );
            fastest = fastest.max((next - camera).norm() / DT);
            camera = next;
        }
        assert!(
            (camera - wanted).norm() < 0.05,
            "never eased back: {camera:?}"
        );
        // Opening 4.3 m on a 0.5 s spring peaks near distance / time, not in
        // one frame (258 m/s).
        assert!(
            fastest < 20.0,
            "the camera snapped back at {fastest:.0} m/s"
        );
    }

    #[test]
    fn a_moving_body_is_seen_through() {
        let world = world_with_box(RigidBodyDesc::dynamic(), Vector3::new(1.0, 0.5, 1.0));
        let wanted = Point3::new(-3.0, 1.0, 0.0);
        let camera = settle(&active(), &world, &EmptyGeometry, wanted, 2.0);
        assert!((camera - wanted).norm() < 1.0e-4, "camera at {camera:?}");
    }

    #[test]
    fn disabled_the_camera_passes_through_rock() {
        let world = PhysicsWorld::default();
        let wall = WallAndFloorGeometry::new(10.0, 4.0);
        let clearance = CameraClearance::default();
        let mut state = ClearanceState::default();
        let wanted = Point3::new(-3.0, 1.0, 0.0);
        let camera = clearance.resolve(
            &mut state,
            &scene(&world, &wall),
            PIVOT,
            Point3::new(1.0, 1.0, 0.0),
            wanted,
            DT,
        );
        assert_eq!(camera, wanted);
    }
}
