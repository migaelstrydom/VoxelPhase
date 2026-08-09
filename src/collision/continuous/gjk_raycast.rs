//! GJK-based translation-only continuous collision detection (CCD).
//!
//! Computes the time of impact (TOI) for two convex shapes translating
//! linearly over a timestep. Uses the Minkowski difference ray march
//! algorithm: repeatedly queries GJK for the closest distance and advances
//! along the relative displacement by that amount until the shapes touch.
//!
//! This gives any `ConvexSupport` shape CCD without shape-pair-specific code.
//! Rotation during the timestep is ignored (translation-only).

use nalgebra::{Point3, Vector3};

use crate::collision::discrete::epa::epa_penetration;
use crate::collision::discrete::gjk::{gjk_query, GjkResult};
use crate::collision::support::ConvexSupport;

/// Maximum iterations for the ray march loop.
const MAX_ITERATIONS: u32 = 64;
/// Distance tolerance: when GJK distance drops below this, declare contact.
const DISTANCE_TOLERANCE: f32 = 1e-4;

/// Result of a GJK raycast query.
#[derive(Debug, Clone)]
pub struct GjkRaycastHit {
    /// Time of impact in [0, 1].
    pub t: f32,
    /// Contact normal at impact (unit vector, from B toward A).
    pub normal: Vector3<f32>,
    /// Contact point (approximate witness on B's surface at TOI).
    pub point: Point3<f32>,
}

/// Compute the time of impact between two translating convex shapes.
///
/// Shape A moves from its current position along `displacement_a` over the
/// timestep. Shape B moves along `displacement_b`. Both shapes must implement
/// `ConvexSupport`. Returns `None` if the shapes don't collide during [0, 1].
///
/// The algorithm works on the Minkowski difference: the relative displacement
/// `rel_displacement = displacement_a - displacement_b` defines a ray from
/// the origin of the configuration space. We march along this ray using GJK
/// distance queries at each step as the conservative advancement distance.
///
/// # Arguments
/// * `a` — shape A at its start-of-timestep position
/// * `b` — shape B at its start-of-timestep position
/// * `displacement_a` — total displacement of A over the timestep
/// * `displacement_b` — total displacement of B over the timestep
pub fn gjk_raycast(
    a: &dyn ConvexSupport,
    b: &dyn ConvexSupport,
    displacement_a: Vector3<f32>,
    displacement_b: Vector3<f32>,
) -> Option<GjkRaycastHit> {
    // Already touching or overlapping at t=0: there is no march to run, so the
    // witness has to come from the query itself. Both branches below must
    // report a point on B's *surface* — a caller that gets B's centre instead
    // solves the contact metres from where the shapes meet, which for a large
    // obstacle sends the impulse almost entirely into torque.
    match gjk_query(a, b) {
        GjkResult::Intersecting { simplex } => {
            // GJK stops as soon as it encloses the origin and keeps no witness,
            // so expand the simplex to recover the surface points.
            let penetration = epa_penetration(a, b, 0.0, &simplex);
            return Some(GjkRaycastHit {
                t: 0.0,
                // EPA reports A toward B; this result is oriented B toward A.
                normal: -penetration.normal,
                point: penetration.witness_b,
            });
        }
        GjkResult::Separated {
            distance,
            closest_a,
            closest_b,
        } => {
            if distance < DISTANCE_TOLERANCE {
                let delta = closest_a - closest_b;
                let normal = if delta.magnitude_squared() > 1e-10 {
                    delta.normalize()
                } else {
                    estimate_separation_normal(a, b)
                };
                return Some(GjkRaycastHit {
                    t: 0.0,
                    normal,
                    point: closest_b,
                });
            }
        }
    }

    let rel_displacement = displacement_a - displacement_b;
    let rel_speed = rel_displacement.magnitude();

    // No relative motion — no collision if not already overlapping.
    if rel_speed < 1e-10 {
        return None;
    }

    // Create displaced shape wrappers that we'll advance through [0, 1].
    let mut t = 0.0f32;
    let mut last_normal = Vector3::zeros();

    for _ in 0..MAX_ITERATIONS {
        // Create shape views offset to their positions at time `t`.
        let offset_a = displacement_a * t;
        let offset_b = displacement_b * t;
        let shifted_a = TranslatedShape {
            shape: a,
            offset: offset_a,
        };
        let shifted_b = TranslatedShape {
            shape: b,
            offset: offset_b,
        };

        let result = gjk_query(&shifted_a, &shifted_b);

        match result {
            GjkResult::Intersecting { simplex } => {
                // Advancement overshot into overlap. Expand the simplex rather
                // than guessing: `support(-normal)` returns the shape's extreme
                // vertex in that direction, which on anything long or tall is
                // nowhere near where the two actually met.
                let penetration = epa_penetration(&shifted_a, &shifted_b, 0.0, &simplex);

                // EPA recovers the axis the shapes must separate along, but not
                // which way along it — the sign depends on how its polytope was
                // seeded, and comes back inverted often enough to matter. The
                // direction advancement was travelling picks the hemisphere.
                let approach = if last_normal.magnitude_squared() > 1e-10 {
                    last_normal.normalize()
                } else {
                    estimate_separation_normal(&shifted_a, &shifted_b)
                };
                let axis = -penetration.normal;
                let normal = if axis.dot(&approach) < 0.0 {
                    -axis
                } else {
                    axis
                };

                // Midway between the witnesses: unlike either witness alone,
                // this does not move if EPA labelled the shapes the other way
                // round, and at these depths the two are almost coincident.
                let point = nalgebra::center(&penetration.witness_a, &penetration.witness_b);
                return Some(GjkRaycastHit { t, normal, point });
            }
            GjkResult::Separated {
                distance,
                closest_a,
                closest_b,
            } => {
                if distance < DISTANCE_TOLERANCE {
                    let delta = closest_a - closest_b;
                    let normal = if delta.magnitude_squared() > 1e-10 {
                        delta.normalize()
                    } else if last_normal.magnitude_squared() > 1e-10 {
                        last_normal.normalize()
                    } else {
                        estimate_separation_normal(&shifted_a, &shifted_b)
                    };
                    return Some(GjkRaycastHit {
                        t,
                        normal,
                        point: closest_b,
                    });
                }

                // Conservative advancement: advance `t` by `distance / rel_speed`.
                // This is safe because GJK distance is the exact minimum distance
                // and the shapes can't move closer than `rel_speed` per unit time.
                let dt = distance / rel_speed;
                t += dt;

                if t > 1.0 {
                    return None;
                }

                // Track the separation direction for normal estimation.
                let delta = closest_a - closest_b;
                if delta.magnitude_squared() > 1e-10 {
                    last_normal = delta;
                }
            }
        }
    }

    // Didn't converge — conservative miss.
    None
}

/// A shape translated by an offset (for advancing shapes along their paths).
struct TranslatedShape<'a> {
    shape: &'a dyn ConvexSupport,
    offset: Vector3<f32>,
}

impl ConvexSupport for TranslatedShape<'_> {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        self.shape.support(direction) + self.offset
    }

    fn bounding_radius(&self) -> f32 {
        self.shape.bounding_radius()
    }
}

/// Estimate a separation normal from support queries.
fn estimate_separation_normal(a: &dyn ConvexSupport, b: &dyn ConvexSupport) -> Vector3<f32> {
    let center_a = estimate_center_point(a);
    let center_b = estimate_center_point(b);
    let delta = center_a - center_b;
    if delta.magnitude_squared() > 1e-10 {
        delta.normalize()
    } else {
        Vector3::y()
    }
}

/// Estimate shape center from support queries in 6 axis directions.
fn estimate_center_point(shape: &dyn ConvexSupport) -> Point3<f32> {
    let dirs = [
        Vector3::x(),
        -Vector3::x(),
        Vector3::y(),
        -Vector3::y(),
        Vector3::z(),
        -Vector3::z(),
    ];
    let sum: Vector3<f32> = dirs.iter().map(|d| shape.support(*d).coords).sum();
    Point3::from(sum / dirs.len() as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::continuous::swept_sphere_sphere;
    use crate::collision::obb::Obb;
    use crate::collision::support::SupportSphere;
    use nalgebra::UnitQuaternion;

    // --- Sphere-sphere cross-validation ---

    #[test]
    fn sphere_sphere_cross_validation() {
        let a = SupportSphere {
            center: Point3::new(-3.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(3.0, 0.0, 0.0),
            radius: 1.0,
        };

        let disp_a = Vector3::new(6.0, 0.0, 0.0);
        let disp_b = Vector3::zeros();

        let gjk_hit = gjk_raycast(&a, &b, disp_a, disp_b);
        let analytic = swept_sphere_sphere(
            Point3::new(-3.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            1.0,
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            1.0,
        );

        assert!(gjk_hit.is_some(), "GJK raycast should find collision");
        assert!(analytic.is_some(), "Analytic should find collision");

        let gjk_t = gjk_hit.unwrap().t;
        let analytic_t = analytic.unwrap();

        assert!(
            (gjk_t - analytic_t).abs() < 0.01,
            "TOI mismatch: GJK {} vs analytic {}",
            gjk_t,
            analytic_t,
        );
    }

    // --- OBB vs static plane (large thin OBB) ---

    #[test]
    fn obb_vs_plane() {
        // Box moving downward toward a large flat plane (thin OBB).
        let box_a = Obb::new(
            Point3::new(0.0, 5.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let plane = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(10.0, 0.05, 10.0),
        );

        let disp_a = Vector3::new(0.0, -10.0, 0.0);
        let disp_b = Vector3::zeros();

        let hit = gjk_raycast(&box_a, &plane, disp_a, disp_b);
        assert!(hit.is_some(), "Should detect collision");

        let h = hit.unwrap();
        // Geometric prediction: box bottom at y=4.5, plane top at y=0.05.
        // Distance to close = 4.45. With disp of 10 downward, t ≈ 4.45/10 = 0.445.
        assert!(
            (h.t - 0.445).abs() < 0.02,
            "TOI should be ~0.445, got {}",
            h.t,
        );
        assert!(
            h.normal.y > 0.9,
            "Normal should point up, got {:?}",
            h.normal,
        );
    }

    // --- OBB vs OBB ---

    #[test]
    fn obb_vs_obb() {
        let a = Obb::new(
            Point3::new(-3.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let b = Obb::new(
            Point3::new(3.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );

        let disp_a = Vector3::new(6.0, 0.0, 0.0);
        let disp_b = Vector3::zeros();

        let hit = gjk_raycast(&a, &b, disp_a, disp_b);
        assert!(hit.is_some(), "Should detect collision");

        let h = hit.unwrap();
        // Distance = 6 - 2*0.5 = 5.0. Speed = 6. t ≈ 5/6 ≈ 0.833.
        assert!(h.t > 0.0, "TOI should be positive");
        assert!(
            (h.t - 5.0 / 6.0).abs() < 0.02,
            "TOI should be ~0.833, got {}",
            h.t,
        );
        assert!(
            h.normal.x.abs() > 0.9,
            "Normal should be along X, got {:?}",
            h.normal,
        );
    }

    // --- Miss case ---

    #[test]
    fn miss_moving_apart() {
        let a = SupportSphere {
            center: Point3::new(-3.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(3.0, 0.0, 0.0),
            radius: 1.0,
        };

        // Moving apart.
        let disp_a = Vector3::new(-5.0, 0.0, 0.0);
        let disp_b = Vector3::new(5.0, 0.0, 0.0);

        let hit = gjk_raycast(&a, &b, disp_a, disp_b);
        assert!(hit.is_none(), "Should miss when moving apart");
    }

    // --- Already overlapping ---

    #[test]
    fn already_overlapping() {
        let a = SupportSphere {
            center: Point3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(0.5, 0.0, 0.0),
            radius: 1.0,
        };

        let disp_a = Vector3::new(1.0, 0.0, 0.0);
        let disp_b = Vector3::zeros();

        let hit = gjk_raycast(&a, &b, disp_a, disp_b);
        assert!(hit.is_some(), "Should detect already-overlapping");
        let h = hit.unwrap();
        assert!(h.t < 1e-6, "TOI should be ~0 for overlap, got {}", h.t);
    }

    // --- Grazing contact ---

    #[test]
    fn grazing_contact() {
        // Box barely clips the corner of another.
        let a = Obb::new(
            Point3::new(-5.0, 1.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );
        let b = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 0.5, 0.5),
        );

        // Moving A toward B with slight vertical overlap (gap of 0.0 in Y).
        let disp_a = Vector3::new(10.0, 0.0, 0.0);
        let disp_b = Vector3::zeros();

        let hit = gjk_raycast(&a, &b, disp_a, disp_b);
        assert!(hit.is_some(), "Should detect grazing contact");
    }

    // --- Zero relative velocity ---

    #[test]
    fn zero_relative_velocity() {
        let a = SupportSphere {
            center: Point3::new(-3.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(3.0, 0.0, 0.0),
            radius: 1.0,
        };

        // Same displacement — zero relative motion.
        let disp = Vector3::new(5.0, 0.0, 0.0);
        let hit = gjk_raycast(&a, &b, disp, disp);
        assert!(
            hit.is_none(),
            "Should not collide with zero relative velocity"
        );
    }

    // --- Both shapes moving toward each other ---

    #[test]
    fn both_moving_toward_each_other() {
        let a = SupportSphere {
            center: Point3::new(-5.0, 0.0, 0.0),
            radius: 1.0,
        };
        let b = SupportSphere {
            center: Point3::new(5.0, 0.0, 0.0),
            radius: 1.0,
        };

        let disp_a = Vector3::new(5.0, 0.0, 0.0);
        let disp_b = Vector3::new(-5.0, 0.0, 0.0);

        let hit = gjk_raycast(&a, &b, disp_a, disp_b);
        assert!(hit.is_some());

        let h = hit.unwrap();
        // Distance = 8, rel speed = 10, t ≈ 0.8.
        assert!((h.t - 0.8).abs() < 0.02, "TOI should be ~0.8, got {}", h.t,);
    }

    /// A contact reported at the centre of a large obstacle is worse than no
    /// contact at all: the solver applies the impulse metres from where the
    /// shapes actually touch, and anything reading the contact point — the
    /// grenade detonation rule, for one — places the event inside the obstacle.
    ///
    /// The t=0 path is where this bites, since it has no march to derive a
    /// witness from. Measured against a temple step: a 0.12 m sphere sunk into
    /// a 12 x 0.5 x 18 m box, whose centre is nine metres from the impact.
    ///
    /// Depths run from grazing to deep, because the shallow end is where the
    /// direction is hardest to recover — at true tangency EPA has no penetration
    /// to expand and cannot orient the normal at all. That case is left
    /// unhandled: a hit that shallow carries no impulse worth placing, and
    /// `is_tunnelling_hit` discards it if the direction does come back reversed.
    #[test]
    fn a_contact_at_time_zero_lands_on_the_obstacle_surface() {
        let step = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(6.0, 0.25, 9.0),
        );
        let face_y = 0.25;

        for depth in [0.001, 0.02, 0.08, 0.11] {
            let sphere = SupportSphere {
                center: Point3::new(2.0, face_y + 0.12 - depth, -3.0),
                radius: 0.12,
            };

            let hit = gjk_raycast(
                &sphere,
                &step,
                Vector3::new(0.0, -0.5, 0.0),
                Vector3::zeros(),
            )
            .unwrap_or_else(|| panic!("depth {depth}: should report a contact"));

            assert!(
                hit.t < 1e-6,
                "depth {depth}: TOI should be ~0, got {}",
                hit.t
            );
            assert!(
                (hit.point.y - face_y).abs() < 1e-3,
                "depth {depth}: contact should sit on the top face at y={face_y}, got {:?}",
                hit.point,
            );
            assert!(
                (hit.point.x - 2.0).abs() < 1e-3 && (hit.point.z + 3.0).abs() < 1e-3,
                "depth {depth}: contact drifted away from the sphere, got {:?}",
                hit.point,
            );
            assert!(
                hit.normal.y > 0.99,
                "depth {depth}: normal should point out of the top face, got {:?}",
                hit.normal,
            );
        }
    }

    /// The same demand as the t=0 case, for the branch that reaches contact by
    /// marching: when advancement overshoots into overlap, the hit must be
    /// reported where the shapes met.
    ///
    /// The displacement here is large enough to overshoot in one step, and the
    /// target is tall, so a contact point taken from a support query lands on a
    /// far corner metres from the sphere.
    #[test]
    fn a_contact_found_by_marching_lands_between_the_shapes() {
        let pillar = Obb::new(
            Point3::origin(),
            UnitQuaternion::identity(),
            Vector3::new(0.5, 3.0, 0.5),
        );
        let sphere = SupportSphere {
            center: Point3::new(2.0, 1.5, 0.3),
            radius: 0.2,
        };

        let hit = gjk_raycast(
            &sphere,
            &pillar,
            Vector3::new(-3.0, 0.0, 0.0),
            Vector3::zeros(),
        )
        .expect("should hit the pillar");

        assert!(
            (hit.point.y - 1.5).abs() < 0.25,
            "contact should be level with the sphere, got {:?}",
            hit.point,
        );
        assert!(
            (hit.point.x - 0.5).abs() < 0.25 && (hit.point.z - 0.3).abs() < 0.25,
            "contact should be on the near face beside the sphere, got {:?}",
            hit.point,
        );
        assert!(
            hit.normal.x > 0.9,
            "normal should point out of the near face, got {:?}",
            hit.normal,
        );
    }

    // --- Rotated OBB approaching ---

    #[test]
    fn rotated_obb_approaching() {
        let rot = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::y()),
            std::f32::consts::FRAC_PI_4,
        );
        let a = Obb::new(
            Point3::new(-5.0, 0.0, 0.0),
            rot,
            Vector3::new(0.5, 0.5, 0.5),
        );
        let b = Obb::new(
            Point3::new(0.0, 0.0, 0.0),
            UnitQuaternion::identity(),
            Vector3::new(1.0, 1.0, 1.0),
        );

        let disp_a = Vector3::new(10.0, 0.0, 0.0);
        let disp_b = Vector3::zeros();

        let hit = gjk_raycast(&a, &b, disp_a, disp_b);
        assert!(hit.is_some(), "Rotated OBB should hit");
        let h = hit.unwrap();
        assert!(
            h.t > 0.0 && h.t < 1.0,
            "TOI should be in (0, 1), got {}",
            h.t
        );
    }
}
