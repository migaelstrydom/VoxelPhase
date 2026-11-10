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
use crate::geometry::{generate_dome, generate_sphere_indices, generate_sphere_vertices, DomeSpec};
use crate::rendering::vertex::Vertex;
use crate::utils::noise::fbm_perlin_3d;

use super::config::StiltRigConfig;
use super::costume::Costume;
use super::skeleton::StiltSkeleton;

/// How far the iris centre sits from the eye centre, in eye radii. Under 1
/// so it is seated in the eye rather than floating off it.
const IRIS_SEAT: f32 = 0.78;
/// The same for the pupil, which rides on the iris.
const PUPIL_SEAT: f32 = 1.1;

/// Fewest segments the white is drawn with: its veins are painted on its
/// vertices, and need enough of them.
const VEIN_SEGMENTS: u32 = 36;
/// Veins to the eye's radius, how thin they are, and how strongly they show.
const VEIN_FREQUENCY: f32 = 2.6;
const VEIN_SHARPNESS: f32 = 9.0;
const VEIN_STRENGTH: f32 = 0.85;
const VEIN_SEED: u32 = 77;
/// How far forward the veins reach, in eye radii along the gaze.
const VEIN_FRONT: f32 = 0.55;
/// How far round the rim of the lid the lashes spread either side of
/// straight ahead, radians, and how thick a lash is, m.
const LASH_SPREAD: f32 = 1.25;
const LASH_RADIUS: f32 = 0.006;

/// The walker's mesh this frame: its legs, and over them the peeper's eye
/// or, if it wears one, its `costume`.
pub fn generate_stilt_mesh(
    skeleton: &StiltSkeleton,
    config: &StiltRigConfig,
    lid_close: f32,
    costume: Option<&Costume>,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut mesh = RigMesh::new(config.mesh_segments);

    add_legs(&mut mesh, skeleton, config);
    match costume {
        None => {
            add_haunch(&mut mesh, skeleton, config);
            add_neck(&mut mesh, skeleton, config);
            add_eye(&mut mesh, skeleton, config, lid_close);
            add_feelers(&mut mesh, skeleton, config);
        }
        Some(costume) => {
            if costume.shows_trunk {
                add_haunch(&mut mesh, skeleton, config);
                add_neck(&mut mesh, skeleton, config);
            }
            add_costume(&mut mesh, skeleton, costume);
        }
    }

    mesh.into_parts()
}

/// The head's frame: its middle, looking along the gaze, its crown square
/// to that. It tips with the neck, so what the head wears rears and spears
/// with it.
fn head_frame(skeleton: &StiltSkeleton) -> Frame {
    let up = right_vector(skeleton.facing)
        .cross(&skeleton.gaze)
        .try_normalize(1e-4)
        .unwrap_or_else(Vector3::y);
    Frame {
        origin: skeleton.eye,
        right: up.cross(&skeleton.gaze),
        up,
        forward: skeleton.gaze,
    }
}

fn add_costume(mesh: &mut RigMesh, skeleton: &StiltSkeleton, costume: &Costume) {
    let worn = [
        (&costume.head, head_frame(skeleton)),
        (
            &costume.body,
            Frame::upright(skeleton.pelvis, skeleton.facing),
        ),
    ];
    for (garment, frame) in worn {
        if !garment.indices.is_empty() {
            mesh.part(garment.vertices.clone(), garment.indices.clone(), &frame);
        }
    }
}

/// The stub the legs hang from.
fn add_haunch(mesh: &mut RigMesh, skeleton: &StiltSkeleton, config: &StiltRigConfig) {
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
}

/// The two legs and the feet.
fn add_legs(mesh: &mut RigMesh, skeleton: &StiltSkeleton, config: &StiltRigConfig) {
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

fn add_neck(mesh: &mut RigMesh, skeleton: &StiltSkeleton, config: &StiltRigConfig) {
    mesh.rod(
        skeleton.shoulder,
        skeleton.eye,
        config.neck_radius,
        config.hide_colour,
    );
}

/// The eye: white, iris, pupil, and the lid that covers them.
fn add_eye(mesh: &mut RigMesh, skeleton: &StiltSkeleton, config: &StiltRigConfig, lid_close: f32) {
    let radius = config.eye_radius;
    let gaze = skeleton.gaze;

    add_white(mesh, skeleton, config);

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

/// The white of the eye, bloodshot: veins branch forward over it from the
/// back, thick where they start and gone before they reach the iris.
fn add_white(mesh: &mut RigMesh, skeleton: &StiltSkeleton, config: &StiltRigConfig) {
    let segments = config.mesh_segments.max(VEIN_SEGMENTS);
    let mut vertices =
        generate_sphere_vertices(config.eye_radius, segments, segments, config.sclera_colour);
    let (white, blood) = (config.sclera_colour.to_vec4(), config.vein_colour.to_vec4());
    for vertex in &mut vertices {
        let at = vertex.pos / config.eye_radius;
        // A vein runs where a field of noise crosses zero.
        let field = fbm_perlin_3d(
            at.x * VEIN_FREQUENCY,
            at.y * VEIN_FREQUENCY,
            at.z * VEIN_FREQUENCY,
            3,
            0.55,
            VEIN_SEED,
        );
        let vein = (1.0 - field.abs() * VEIN_SHARPNESS).clamp(0.0, 1.0);
        // +Z is the gaze: none in front, most behind.
        let behind = ((VEIN_FRONT - at.z) / (VEIN_FRONT + 1.0)).clamp(0.0, 1.0);
        vertex.color = white.lerp(&blood, vein * vein * behind * VEIN_STRENGTH);
    }
    mesh.part(
        vertices,
        generate_sphere_indices(segments, segments),
        &head_frame(skeleton),
    );
}

/// Lashes: stiff hairs standing off the rim of the lid, over the front of
/// the eye.
fn add_lashes(
    mesh: &mut RigMesh,
    frame: &Frame,
    config: &StiltRigConfig,
    half_angle: f32,
    rim_radius: f32,
) {
    for lash in 0..config.lashes {
        // Spread across the front half of the rim.
        let across = (lash as f32 + 0.5) / config.lashes as f32 * 2.0 - 1.0;
        let round = across * LASH_SPREAD;
        // On the rim: `half_angle` down from the crown, `round` to one side
        // of straight ahead.
        let out = Vector3::new(
            half_angle.sin() * round.sin(),
            half_angle.cos(),
            half_angle.sin() * round.cos(),
        );
        // A lash leaves the lid along the rim's tangent, forward and out.
        let along = (out + Vector3::new(round.sin(), 0.25, round.cos()) * 0.9).normalize();
        let root = frame.point(out * rim_radius);
        let tip = frame.point(out * rim_radius + along * config.lash_length);
        mesh.rod(root, tip, LASH_RADIUS, config.pupil_colour);
    }
}

/// The eyelid: a dome sitting on the eye, concentric with it, that grows
/// down over it as the creature loses interest.
///
/// Concentric and hugging rather than a second ball parked on top — a ball
/// keeps a ball's silhouette however far it is moved, and reads as a hat.
fn add_lid(mesh: &mut RigMesh, skeleton: &StiltSkeleton, config: &StiltRigConfig, lid_close: f32) {
    let radius = config.eye_radius;

    // The lid tracks the eye's own axis rather than world up, so a pecking
    // peeper's lid stays over its eye instead of sliding off the side of
    // its head.
    let frame = head_frame(skeleton);

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
    mesh.part(vertices, indices, &frame);
    add_lashes(mesh, &frame, config, half_angle, radius + thickness);
}

/// Both feelers, tapering to a rounded tip.
fn add_feelers(mesh: &mut RigMesh, skeleton: &StiltSkeleton, config: &StiltRigConfig) {
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
