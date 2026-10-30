use nalgebra::{Point3, Vector3};
use traversal_planner::plan::{Contact, Motion, Phase, Plan};
use traversal_planner::PathVec;

/// A straight path laid on the world: path coordinates `(s, d, z)` are
/// distance along the heading, offset to its left, and world height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathFrame {
    /// Where `s = 0, d = 0`; its height is ignored, since `z` is world height.
    origin: Point3<f32>,
    /// Horizontal unit vector along the path.
    forward: Vector3<f32>,
    /// Horizontal unit vector to the path's left.
    left: Vector3<f32>,
}

impl PathFrame {
    /// A path from `origin` along `heading_degrees`, turned about `+Y` from
    /// `+Z` as every yaw in the level format is.
    pub fn new(origin: Point3<f32>, heading_degrees: f32) -> Self {
        let (sin, cos) = heading_degrees.to_radians().sin_cos();
        let forward = Vector3::new(sin, 0.0, cos);
        Self {
            origin: Point3::new(origin.x, 0.0, origin.z),
            forward,
            left: Vector3::y().cross(&forward),
        }
    }

    /// A path from `origin` along the horizontal part of `heading`.
    pub fn along(origin: Point3<f32>, heading: Vector3<f32>) -> Self {
        let forward = Vector3::new(heading.x, 0.0, heading.z)
            .try_normalize(f32::EPSILON)
            .unwrap_or_else(Vector3::z);
        Self {
            origin: Point3::new(origin.x, 0.0, origin.z),
            forward,
            left: Vector3::y().cross(&forward),
        }
    }

    /// Horizontal unit vector along the path.
    pub fn forward(&self) -> Vector3<f32> {
        self.forward
    }

    pub fn world_point(&self, at: PathVec) -> Point3<f32> {
        self.origin + self.forward * at.s + self.left * at.d + Vector3::y() * at.z
    }

    pub fn path_vector(&self, world: Vector3<f32>) -> PathVec {
        PathVec::new(world.dot(&self.forward), world.dot(&self.left), world.y)
    }

    pub fn path_point(&self, world: Point3<f32>) -> PathVec {
        let offset = world - self.origin;
        PathVec::new(offset.dot(&self.forward), offset.dot(&self.left), world.y)
    }

    /// `plan`, made along this path, along `onto` instead, turned with the
    /// path: each place the plan names keeps its offset across the path and
    /// its height, and its distance along it is where its foot on this
    /// path's line falls along `onto`'s. Its speeds are what they are along
    /// `onto`.
    ///
    /// The body runs a curve the straight path cannot follow, so footfalls
    /// kept where they were in the world would end up further outside the
    /// body's way with every turn of the path; turned with it, a footfall's
    /// error is the curve's offset from the path ahead of the body, which
    /// shrinks to nothing as the body comes up to it.
    pub fn carry(&self, plan: &Plan, onto: &PathFrame) -> Plan {
        let along = self.forward.dot(&onto.forward);
        let s_on = |s: f32| {
            onto.path_point(self.world_point(PathVec::new(s, 0.0, 0.0)))
                .s
        };
        let contact = |contact: &Contact| Contact {
            at: PathVec::new(s_on(contact.at.s), contact.at.d, contact.at.z),
            ..*contact
        };
        let motion = |motion: Motion| match motion {
            Motion::Grounded {
                from,
                to,
                start_speed,
                end_speed,
            } => Motion::Grounded {
                from: s_on(from),
                to: s_on(to),
                start_speed: start_speed * along,
                end_speed: end_speed * along,
            },
            Motion::Flight {
                mut takeoff,
                gravity,
                rise_held,
            } => {
                takeoff.s = s_on(takeoff.s);
                takeoff.speed *= along;
                Motion::Flight {
                    takeoff,
                    gravity,
                    rise_held,
                }
            }
        };
        Plan {
            phases: plan
                .phases
                .iter()
                .map(|phase| Phase {
                    contacts: phase.contacts.iter().map(contact).collect(),
                    motion: motion(phase.motion),
                    ..phase.clone()
                })
                .collect(),
            ..plan.clone()
        }
    }
}

#[cfg(test)]
mod tests {
    use traversal_planner::plan::Limb;
    use traversal_planner::section::Footing;

    use super::*;

    #[test]
    fn a_heading_of_ninety_runs_along_x_with_its_left_toward_minus_z() {
        let frame = PathFrame::new(Point3::new(3.0, 7.0, 10.0), 90.0);
        let ahead = frame.world_point(PathVec::new(2.0, 0.5, 1.0));
        assert!(
            (ahead - Point3::new(5.0, 1.0, 9.5)).norm() < 1e-5,
            "{ahead}"
        );
        let back = frame.path_point(ahead);
        assert!((back - PathVec::new(2.0, 0.5, 1.0)).length() < 1e-5);
    }

    /// A plan riding from `s` 0 to 2 at 5 m/s, a foot landing at `s` 1.5,
    /// 0.12 to the left.
    fn ride() -> Plan {
        let foot = Contact {
            footfall: None,
            limb: Limb::LeftFoot,
            at: PathVec::new(1.5, 0.12, 0.0),
            normal: PathVec::new(0.0, 0.0, 1.0),
            footing: Footing::Fixed,
        };
        Plan {
            phases: vec![Phase {
                start: 0.0,
                duration: 0.4,
                contacts: [foot].into_iter().collect(),
                motion: Motion::Grounded {
                    from: 0.0,
                    to: 2.0,
                    start_speed: 5.0,
                    end_speed: 5.0,
                },
            }],
            committed: 1,
            target_speed: 5.0,
        }
    }

    #[test]
    fn a_plan_carried_onto_a_turned_path_turns_with_it() {
        let old = PathFrame::new(Point3::new(1.0, 0.0, 2.0), 90.0);
        let new = PathFrame::new(Point3::new(1.5, 0.0, 2.0), 92.0);
        let carried = old.carry(&ride(), &new);
        let foot = carried.phases[0].contacts[0];
        // The foot was 1 m ahead of the new origin and 0.12 m to the left;
        // it still is, a hair less along a line turned 2°.
        assert!((foot.at.s - 1.0).abs() < 1e-3, "{:?}", foot.at);
        assert!((foot.at.d - 0.12).abs() < 1e-6, "{:?}", foot.at);
        assert!((foot.normal - PathVec::new(0.0, 0.0, 1.0)).length() < 1e-6);
        let Motion::Grounded {
            from,
            to,
            start_speed,
            ..
        } = carried.phases[0].motion
        else {
            panic!("a ride");
        };
        // The old line's start is 0.5 m behind the new origin, its end 1.5 m
        // ahead, each a hair less along a line turned 2°.
        assert!((from + 0.5).abs() < 1e-3, "{from}");
        assert!((to - 1.5).abs() < 2e-3, "{to}");
        assert!((start_speed - 5.0 * 2f32.to_radians().cos()).abs() < 1e-5);
        assert_eq!(carried.committed, 1);
    }
}
