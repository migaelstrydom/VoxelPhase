//! Drawing a heart critter.
//!
//! Rods for the legs and the ears, a swept heart for the torso, and a
//! second, smaller heart laid on the front and back of it as a marking.

use nalgebra::Vector3;

use crate::animation::rig::RigMesh;
use crate::geometry::{generate_heart, HeartSpec};
use crate::rendering::vertex::Vertex;

use super::config::CritterRigConfig;
use super::ears::EarSide;
use super::skeleton::CritterSkeleton;

pub fn generate_critter_mesh(
    skeleton: &CritterSkeleton,
    config: &CritterRigConfig,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut mesh = RigMesh::new(config.mesh_segments);

    add_torso(&mut mesh, skeleton, config);
    add_legs(&mut mesh, skeleton, config);
    add_ears(&mut mesh, skeleton, config);

    mesh.into_parts()
}

/// The heart, and the heart on the heart.
fn add_torso(mesh: &mut RigMesh, skeleton: &CritterSkeleton, config: &CritterRigConfig) {
    let frame = skeleton.torso_frame();

    let body = HeartSpec::new(
        config.torso_width,
        config.torso_height,
        config.torso_depth,
        config.fur_colour,
    );
    let (verts, indices) = generate_heart(&body);
    mesh.part(verts, indices, &frame);

    // The marking is a patch of the body's own surface, recoloured and
    // lifted clear of the fur. Taking it from the body is what makes it
    // lie flat: a separate smaller heart placed at the front would be
    // swallowed whole, since every ring of it is narrower than the body
    // ring at the same depth.
    let mark = HeartSpec {
        colour: config.mark_colour,
        ..body
    };
    for spec in [
        mark.front_marking(config.mark_scale, config.mark_relief),
        mark.back_marking(config.mark_scale, config.mark_relief),
    ] {
        let (verts, indices) = generate_heart(&spec);
        mesh.part(verts, indices, &frame);
    }
}

/// The pelvis stub, the two legs, and the feet.
fn add_legs(mesh: &mut RigMesh, skeleton: &CritterSkeleton, config: &CritterRigConfig) {
    let torso_bottom = skeleton.pelvis + Vector3::y() * config.pelvis_stub;
    mesh.rod(
        skeleton.pelvis,
        torso_bottom,
        config.pelvis_radius,
        config.fur_colour,
    );
    mesh.sphere(skeleton.pelvis, config.pelvis_radius, config.fur_colour);

    for leg in &skeleton.legs {
        mesh.rod(leg.hip, leg.knee, config.leg_radius, config.fur_colour);
        mesh.rod(leg.knee, leg.foot, config.leg_radius, config.fur_colour);
        mesh.sphere(leg.knee, config.knee_radius, config.fur_colour);
        mesh.foot(
            config.foot,
            leg.foot,
            leg.foot_forward,
            leg.foot_up,
            config.foot_colour,
        );
    }
}

/// Both ears, tapering to a rounded tip.
fn add_ears(mesh: &mut RigMesh, skeleton: &CritterSkeleton, config: &CritterRigConfig) {
    for side in EarSide::BOTH {
        let joints = skeleton.ears.joints(side);
        for (node, pair) in joints.windows(2).enumerate() {
            mesh.rod(
                pair[0],
                pair[1],
                skeleton.ears.radius_at(node),
                config.ear.colour,
            );
            mesh.sphere(
                pair[1],
                skeleton.ears.radius_at(node + 1),
                config.ear.colour,
            );
        }
    }
}
