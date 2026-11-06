use nalgebra::{Point3, Vector3};

// === One-shot impulses (explosions, jump pads, etc.) ===

/// A one-shot impulse event applied to nearby bodies and then discarded.
#[derive(Debug, Clone, Copy)]
pub enum PhysicsImpulse {
    /// A blast wave expanding outward from a point with linear falloff. It
    /// pushes a body by the area the body shows it: a leaf and a boulder of
    /// one size are struck alike, and the leaf flies further for being
    /// lighter.
    Radial {
        /// Center of the impulse in world space.
        center: Point3<f32>,
        /// Radius of effect in meters.
        radius: f32,
        /// Impulse per area struck (N·s/m², Pa·s) at the center, falling off
        /// linearly to zero at the radius.
        strength: f32,
        /// Upward impulse factor (0 = none, 0.5 = half of strength added as vertical boost).
        upward_boost: f32,
    },
}

impl PhysicsImpulse {
    /// Create a radial impulse (e.g. explosion).
    pub fn radial(center: Point3<f32>, radius: f32, strength: f32, upward_boost: f32) -> Self {
        Self::Radial {
            center,
            radius,
            strength,
            upward_boost,
        }
    }

    /// The impulse on a body at `body_pos` whose silhouette seen along a
    /// direction is `exposed(direction)`, m². `None` outside the area of
    /// effect.
    pub fn impulse_on(
        &self,
        body_pos: Point3<f32>,
        exposed: impl FnOnce(Vector3<f32>) -> f32,
    ) -> Option<Vector3<f32>> {
        let loading = self.loading_at(body_pos)?;
        Some(loading.per_area * exposed(loading.travel))
    }

    /// How hard the blast strikes at `point`: what each square metre facing
    /// it is given. `None` outside the area of effect.
    pub fn loading_at(&self, point: Point3<f32>) -> Option<Loading> {
        match *self {
            Self::Radial {
                center,
                radius,
                strength,
                upward_boost,
            } => {
                if radius <= 0.0 || strength.abs() < 1e-6 {
                    return None;
                }
                let delta = point - center;
                let distance = delta.magnitude();
                if distance >= radius {
                    return None;
                }
                let (direction, falloff) = if distance < 1e-4 {
                    (Vector3::y(), 1.0)
                } else {
                    (delta / distance, 1.0 - (distance / radius))
                };
                let per_area = direction * strength * falloff
                    + Vector3::new(0.0, strength * falloff * upward_boost, 0.0);
                Some(Loading {
                    travel: direction,
                    per_area,
                })
            }
        }
    }
}

/// A blast's loading at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Loading {
    /// The way the wave travels there, unit length.
    pub travel: Vector3<f32>,
    /// The impulse it gives each square metre facing it, N·s/m².
    pub per_area: Vector3<f32>,
}

/// Queue of one-shot impulse events consumed each physics step.
///
/// After draining, a snapshot of the drained impulses is kept in
/// `last_impulses` so that downstream systems (e.g. fracture) can
/// query what was applied this frame.
#[derive(Default)]
pub struct PhysicsImpulseQueue {
    events: Vec<PhysicsImpulse>,
    last_impulses: Vec<PhysicsImpulse>,
}

impl PhysicsImpulseQueue {
    pub fn push(&mut self, event: PhysicsImpulse) {
        self.events.push(event);
    }

    pub fn drain(&mut self) -> impl Iterator<Item = PhysicsImpulse> + '_ {
        self.last_impulses.clear();
        self.last_impulses.extend(self.events.iter().copied());
        self.events.drain(..)
    }

    /// Impulses that were applied during the most recent physics step.
    pub fn last_impulses(&self) -> &[PhysicsImpulse] {
        &self.last_impulses
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::physics::{
        BulkShape, ColliderDesc, ColliderShape, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
        Volume,
    };

    /// A blast 3 m along -x from the origin, reaching 6 m, with no lift.
    fn blast() -> PhysicsImpulse {
        PhysicsImpulse::radial(Point3::new(-3.0, 0.0, 0.0), 6.0, 100.0, 0.0)
    }

    fn body_of(world: &mut PhysicsWorld, colliders: Vec<ColliderDesc>) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic());
        for collider in colliders {
            world.attach_collider(body, collider);
        }
        body
    }

    fn struck(colliders: Vec<ColliderDesc>) -> f32 {
        let mut world = PhysicsWorld::default();
        let body = body_of(&mut world, colliders);
        world.impulse_on(body, &blast()).unwrap().x
    }

    /// Two balls of a size are struck alike, whatever they weigh: the
    /// light one flies the faster.
    #[test]
    fn a_blast_strikes_by_size_not_weight() {
        let light = struck(vec![ColliderDesc::sphere(0.5).density(50.0)]);
        let heavy = struck(vec![ColliderDesc::sphere(0.5).density(2500.0)]);
        assert_eq!(light, heavy);
        let half_strength = 100.0 * 0.5;
        let exact = half_strength * std::f32::consts::PI * 0.25;
        assert!(
            (light - exact).abs() < 1e-3 * exact,
            "{light} against {exact}"
        );
    }

    /// A plate facing the blast is struck by its face, and edge-on by its
    /// edge.
    #[test]
    fn a_plate_is_struck_by_the_side_it_shows() {
        let face_on = struck(vec![ColliderDesc::box_shape(Vector3::new(0.025, 0.5, 0.5))]);
        let edge_on = struck(vec![ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.025))]);
        assert!(
            (face_on - 20.0 * edge_on).abs() < 1e-3 * face_on,
            "face on {face_on}, edge on {edge_on}"
        );
    }

    /// Two blocks side by side across the blast are struck twice as hard as
    /// one; one behind the other, the front one shields the back.
    #[test]
    fn a_body_in_parts_is_struck_by_its_silhouette() {
        let block = |x: f32, z: f32| {
            ColliderDesc::box_shape(Vector3::repeat(0.25))
                .offset_translation(Vector3::new(x, 0.0, z))
        };
        let one = struck(vec![block(0.0, 0.0)]);
        let abreast = struck(vec![block(0.0, -0.25), block(0.0, 0.25)]);
        let in_line = struck(vec![block(-0.25, 0.0), block(0.25, 0.0)]);
        assert!(
            (abreast - 2.0 * one).abs() < 1e-3 * one,
            "abreast {abreast}, one {one}"
        );
        assert!(
            (in_line - one).abs() < 1e-3 * one,
            "in line {in_line}, one {one}"
        );
    }

    /// A porous body catches its declared share of the blast on its
    /// silhouette; a solid one of the same shape catches all of it.
    #[test]
    fn a_porous_body_catches_its_share_of_a_blast() {
        let solid = struck(vec![ColliderDesc::sphere(0.5)]);
        let mut world = PhysicsWorld::default();
        let bush = world.create_body(RigidBodyDesc::dynamic());
        world.attach_collider(
            bush,
            ColliderDesc::sphere(0.3).bulk(
                BulkShape::displacing(vec![Volume::centred(ColliderShape::Sphere { radius: 0.5 })])
                    .porous(0.3),
            ),
        );
        let porous = world.impulse_on(bush, &blast()).unwrap().x;
        assert!(
            (porous - 0.3 * solid).abs() < 1e-3 * solid,
            "porous {porous}, solid {solid}"
        );
    }
}
