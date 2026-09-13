//! Drawing a peeper.
//!
//! Rods for the legs, the neck and the feelers, and four nested spheres
//! for the eye: the white, the iris bulging out of it, the pupil on top of
//! that, and a lid sphere riding above the lot that slides down to close
//! it.
//!
//! The lid is a sphere rather than a cap because the eye is a sphere: one
//! slightly larger ball sitting a little above another covers exactly the
//! crescent a drooping eyelid covers, and moving it is the whole
//! expression.

use nalgebra::Vector3;

use crate::animation::rig::{right_vector, DroopSide, Frame, RigMesh};
use crate::geometry::{generate_dome, DomeSpec};
use crate::rendering::vertex::Vertex;

use super::config::PeeperRigConfig;
use super::skeleton::PeeperSkeleton;

/// How far the iris centre sits from the eye centre, in eye radii. Under 1
/// so it is seated in the eye rather than floating off it.
const IRIS_SEAT: f32 = 0.78;
/// The same for the pupil, which rides on the iris.
const PUPIL_SEAT: f32 = 0.94;

pub fn generate_peeper_mesh(
    skeleton: &PeeperSkeleton,
    config: &PeeperRigConfig,
    lid_close: f32,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut mesh = RigMesh::new(config.mesh_segments);

    add_legs(&mut mesh, skeleton, config);
    add_neck(&mut mesh, skeleton, config);
    add_eye(&mut mesh, skeleton, config, lid_close);
    add_feelers(&mut mesh, skeleton, config);

    mesh.into_parts()
}

/// The haunch, the two legs, and the feet.
fn add_legs(mesh: &mut RigMesh, skeleton: &PeeperSkeleton, config: &PeeperRigConfig) {
    mesh.rod(
        skeleton.pelvis,
        skeleton.shoulder,
        config.haunch_radius,
        config.hide_colour,
    );
    mesh.sphere(skeleton.pelvis, config.haunch_radius, config.hide_colour);
    mesh.sphere(
        skeleton.shoulder,
        config.haunch_radius * 0.8,
        config.hide_colour,
    );

    for leg in &skeleton.legs {
        mesh.rod(leg.hip, leg.knee, config.leg_radius, config.hide_colour);
        mesh.rod(leg.knee, leg.foot, config.leg_radius, config.hide_colour);
        mesh.sphere(leg.knee, config.knee_radius, config.hide_colour);
        mesh.foot(
            config.foot,
            leg.foot,
            leg.foot_forward,
            leg.foot_up,
            config.foot_colour,
        );
    }
}

fn add_neck(mesh: &mut RigMesh, skeleton: &PeeperSkeleton, config: &PeeperRigConfig) {
    mesh.rod(
        skeleton.shoulder,
        skeleton.eye,
        config.neck_radius,
        config.hide_colour,
    );
}

/// The eye: white, iris, pupil, and the lid that covers them.
fn add_eye(
    mesh: &mut RigMesh,
    skeleton: &PeeperSkeleton,
    config: &PeeperRigConfig,
    lid_close: f32,
) {
    let radius = config.eye_radius;
    let gaze = skeleton.gaze;

    mesh.sphere(skeleton.eye, radius, config.sclera_colour);

    let iris_radius = radius * config.iris_fraction;
    mesh.sphere(
        skeleton.eye + gaze * (radius * IRIS_SEAT),
        iris_radius,
        config.iris_colour,
    );
    mesh.sphere(
        skeleton.eye + gaze * (radius * PUPIL_SEAT),
        iris_radius * config.pupil_fraction,
        config.pupil_colour,
    );

    add_lid(mesh, skeleton, config, lid_close);
}

/// The eyelid: a dome sitting on the eye, concentric with it, that grows
/// down over it as the creature loses interest.
///
/// Concentric and hugging rather than a second ball parked on top — a ball
/// keeps a ball's silhouette however far it is moved, and reads as a hat.
fn add_lid(
    mesh: &mut RigMesh,
    skeleton: &PeeperSkeleton,
    config: &PeeperRigConfig,
    lid_close: f32,
) {
    let radius = config.eye_radius;

    // The lid tracks the eye's own axis rather than world up, so a pecking
    // peeper's lid stays over its eye instead of sliding off the side of
    // its head.
    let lid_up = right_vector(skeleton.facing)
        .cross(&skeleton.gaze)
        .try_normalize(1e-4)
        .unwrap_or_else(Vector3::y);

    let half_angle = config.lid_open_angle
        + (config.lid_shut_angle - config.lid_open_angle) * lid_close.clamp(0.0, 1.0);
    let thickness = radius * config.lid_thickness;

    let (vertices, indices) = generate_dome(&DomeSpec {
        outer_radius: radius + thickness,
        thickness,
        half_angle,
        segments: config.mesh_segments,
        rings: (config.mesh_segments / 2).max(3),
        colour: config.lid_colour,
    });

    // Right-handed, so the dome's faces wind outward the way the shared
    // sphere's do.
    mesh.part(
        vertices,
        indices,
        &Frame {
            origin: skeleton.eye,
            right: lid_up.cross(&skeleton.gaze),
            up: lid_up,
            forward: skeleton.gaze,
        },
    );
}

/// Both feelers, tapering to a rounded tip.
fn add_feelers(mesh: &mut RigMesh, skeleton: &PeeperSkeleton, config: &PeeperRigConfig) {
    for side in DroopSide::BOTH {
        let joints = skeleton.feelers.joints(side);
        for (node, pair) in joints.windows(2).enumerate() {
            mesh.rod(
                pair[0],
                pair[1],
                skeleton.feelers.radius_at(node),
                config.feeler.colour,
            );
            mesh.sphere(
                pair[1],
                skeleton.feelers.radius_at(node + 1),
                config.feeler.colour,
            );
        }
    }
}
