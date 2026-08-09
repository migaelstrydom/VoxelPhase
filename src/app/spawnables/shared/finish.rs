//! The seam where a spawnable's physics and its appearance are declared once.
//!
//! A spawnable builds its collider in `spawn` and its material in
//! `create_materials`, which are different methods called at different times.
//! Left to themselves the two drift: a crate gets made bouncier and stays
//! looking like chalk. So each spawnable declares a single
//! [`PhysicalSurface`] constant and hands it to both sides through these
//! extensions — the collider takes its coefficients, the material takes the
//! finish they imply.
//!
//! The derived finish is a *default*. A spawnable that wants a different look
//! calls `with_finish` after, and the override is then visible in one place
//! next to the thing it overrides.

use crate::physics::ColliderDesc;
use crate::rendering::material::Material;
use crate::rendering::physical_finish::PhysicalSurface;

/// Configures a collider from a declared physical surface.
pub trait ColliderSurface {
    /// Set density, restitution and (isotropic) friction together.
    fn with_physical_surface(self, surface: PhysicalSurface) -> Self;
}

impl ColliderSurface for ColliderDesc {
    fn with_physical_surface(self, surface: PhysicalSurface) -> Self {
        self.density(surface.density)
            .restitution(surface.restitution)
            .friction(surface.friction)
    }
}

/// Shades a material from a declared physical surface.
pub trait MaterialSurface {
    /// Apply the finish this surface's physics implies.
    fn with_derived_finish(self, surface: PhysicalSurface) -> Self;
}

impl MaterialSurface for Material {
    fn with_derived_finish(self, surface: PhysicalSurface) -> Self {
        self.with_finish(surface.finish())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::FrictionModel;
    use nalgebra::{UnitQuaternion, Vector3};

    const RUBBER: PhysicalSurface = PhysicalSurface {
        restitution: 0.6,
        friction: 0.5,
        density: 100.0,
    };

    #[test]
    fn the_collider_takes_every_coefficient() {
        let desc = ColliderDesc::sphere(0.5).with_physical_surface(RUBBER);

        assert_eq!(desc.density, RUBBER.density);
        assert_eq!(desc.material.restitution, RUBBER.restitution);
        assert_eq!(
            desc.material
                .friction_at(&Vector3::y(), &UnitQuaternion::identity()),
            RUBBER.friction
        );
        assert!(matches!(
            desc.material.friction,
            FrictionModel::Isotropic(_)
        ));
    }

    #[test]
    fn the_material_takes_the_finish_that_collider_implies() {
        let material =
            Material::coloured(crate::rendering::colour::Colour::WHITE).with_derived_finish(RUBBER);

        assert_eq!(material.finish.roughness, RUBBER.finish().roughness);
        assert_eq!(material.finish.metallic, RUBBER.finish().metallic);
    }
}
