//! Turning an authored [`VolumeFeature`] into the solids that realise it.
//!
//! One place decides what a traversal primitive *is*, so generation and
//! `level_check` cannot drift apart on the question. Generation rasterises the
//! plan; the check reads the same plan to answer whether a gap is walked.

use crate::level::{VolumeFeature, VoxelMaterialId};

use super::path::PathSolid;
use super::platform::PlatformSolid;
use super::shaft::{ShaftBore, ShaftLedgeSolid};
use super::solid::TraversalSolid;
use super::staircase::StaircaseSolid;

/// One step of a traversal primitive's realisation.
pub enum RoutePart {
    /// Added to the grid.
    Solid(Box<dyn TraversalSolid>),
    /// Subtracted from it — a shaft's bore.
    Void(Box<dyn TraversalSolid>),
}

/// Everything needed to realise one traversal primitive, in application order.
pub struct RoutePlan {
    /// The single material the whole primitive is made of. Traversal
    /// primitives do not layer by depth: a route reads as built, and its
    /// material is how it reads differently from the ground it crosses.
    pub material: VoxelMaterialId,
    pub parts: Vec<RoutePart>,
}

impl RoutePlan {
    /// The solids a player could stand on — everything but the voids.
    pub fn walkable(&self) -> impl Iterator<Item = &dyn TraversalSolid> {
        self.parts.iter().filter_map(|part| match part {
            RoutePart::Solid(s) => Some(s.as_ref()),
            RoutePart::Void(_) => None,
        })
    }
}

/// Expand a volume feature into its traversal plan.
///
/// `None` for the landscape features, and also for a traversal primitive whose
/// parameters describe nothing at all — a one-point path, a zero-radius shaft.
/// Silently generating nothing is the right behaviour here because
/// `level_check` reports the degenerate parameters by name; failing generation
/// would take the whole level down over one bad number.
pub fn route_plan(volume: &VolumeFeature) -> Option<RoutePlan> {
    let (material, parts) = match volume {
        VolumeFeature::Path {
            points,
            width,
            thickness,
            profile,
            material,
        } => {
            let solid = PathSolid::new(points, *width, *thickness, *profile)?;
            (*material, vec![RoutePart::Solid(Box::new(solid))])
        }

        VolumeFeature::Platform {
            center,
            half_extents,
            thickness,
            material,
        } => {
            let solid = PlatformSolid::new(*center, *half_extents, *thickness)?;
            (*material, vec![RoutePart::Solid(Box::new(solid))])
        }

        VolumeFeature::Staircase {
            from,
            to,
            width,
            steps,
            thickness,
            material,
        } => {
            let solid = StaircaseSolid::new(*from, *to, *width, *steps, *thickness)?;
            (*material, vec![RoutePart::Solid(Box::new(solid))])
        }

        VolumeFeature::Shaft {
            center,
            from_y,
            to_y,
            radius,
            ledge,
            material,
        } => {
            let bore = ShaftBore::new(*center, *from_y, *to_y, *radius)?;
            let mut parts: Vec<RoutePart> = vec![RoutePart::Void(Box::new(bore))];
            // The bore is carved first and the ledge added back inside it, so
            // the ledge survives its own shaft.
            if let Some(l) = ledge {
                if let Some(solid) = ShaftLedgeSolid::new(
                    *center,
                    *from_y,
                    *to_y,
                    *radius,
                    l.width,
                    l.thickness,
                    l.pitch,
                    l.start_angle,
                ) {
                    parts.push(RoutePart::Solid(Box::new(solid)));
                }
            }
            (*material, parts)
        }

        _ => return None,
    };

    Some(RoutePlan { material, parts })
}
