//! What static geometry is made of, and how it meets a collider.
//!
//! Friction belongs to a pair of surfaces, not to either one, and no single
//! rule for combining two coefficients gets both ice on ice and ice on grass
//! right. What decides it is which of the two gives way. Ground that yields —
//! grass, soil, sand — is ploughed by whatever sits on it, so its own grip is
//! what the contact has, whatever the object is made of: ice on grass holds
//! about as well as rubber on grass does. Two hard surfaces both matter, and
//! meet the way two colliders do.
//!
//! ```text
//!   contact ── SurfaceId ──▶ StaticGeometry::surface ──▶ Option<StaticSurface>
//!                                                             │
//!   collider ── ColliderMaterial ──────────────────▶ StaticSurface::meet
//!                                                             │
//!                                                  (restitution, friction)
//! ```
//!
//! Geometry with no surface of its own (`None`) leaves the collider's values
//! as they are.

use super::collider::{ColliderMaterial, FrictionModel};

/// How a static surface meets a collider resting on or sliding over it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceResponse {
    /// Gives way under a load: the object ploughs into it, so the surface's
    /// friction is the contact's, and soft ground can only take bounce away,
    /// never add it.
    Yielding,
    /// Hard: both surfaces matter, combined exactly as two colliders are
    /// ([`ColliderMaterial::combine`]).
    Rigid,
}

/// The material of a static surface, as the physics engine reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticSurface {
    /// Coefficient of friction of the surface itself.
    pub friction: f32,
    /// Coefficient of restitution of the surface itself.
    pub restitution: f32,
    /// Whether the surface gives way under what touches it.
    pub response: SurfaceResponse,
}

impl StaticSurface {
    /// Ground that gives way under a load.
    pub const fn yielding(friction: f32, restitution: f32) -> Self {
        Self {
            friction,
            restitution,
            response: SurfaceResponse::Yielding,
        }
    }

    /// Hard ground.
    pub const fn rigid(friction: f32, restitution: f32) -> Self {
        Self {
            friction,
            restitution,
            response: SurfaceResponse::Rigid,
        }
    }

    /// Restitution and friction where `collider` touches this surface, in
    /// the order [`ColliderMaterial::combine`] returns them.
    pub fn meet(&self, collider: &ColliderMaterial) -> (f32, f32) {
        match self.response {
            SurfaceResponse::Yielding => {
                (self.restitution.min(collider.restitution), self.friction)
            }
            SurfaceResponse::Rigid => ColliderMaterial::combine(
                collider,
                &ColliderMaterial {
                    restitution: self.restitution,
                    friction: FrictionModel::Isotropic(self.friction),
                },
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn material(friction: f32, restitution: f32) -> ColliderMaterial {
        ColliderMaterial {
            restitution,
            friction: FrictionModel::Isotropic(friction),
        }
    }

    #[test]
    fn yielding_ground_grips_everything_alike() {
        let grass = StaticSurface::yielding(0.45, 0.2);
        let (_, ice) = grass.meet(&material(0.06, 0.15));
        let (_, rubber) = grass.meet(&material(1.7, 0.8));
        assert_eq!(ice, 0.45);
        assert_eq!(rubber, 0.45);
    }

    #[test]
    fn yielding_ground_never_adds_bounce() {
        let grass = StaticSurface::yielding(0.45, 0.2);
        assert_eq!(grass.meet(&material(0.5, 0.8)).0, 0.2);
        assert_eq!(grass.meet(&material(0.5, 0.1)).0, 0.1);
    }

    #[test]
    fn rigid_ground_meets_like_a_collider() {
        let rock = StaticSurface::rigid(0.7, 0.3);
        let ice = material(0.06, 0.15);
        let (restitution, friction) = rock.meet(&ice);
        let expected = ColliderMaterial::combine(&ice, &material(0.7, 0.3));
        assert_eq!((restitution, friction), expected);
        assert!(friction > 0.06 && friction < 0.7);
    }
}
