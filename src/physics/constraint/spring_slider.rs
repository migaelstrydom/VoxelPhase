//! Spring slider: body_b held on a line through body_a, sprung along it.
//!
//! A prismatic joint whose free axis carries a spring instead of being free:
//! a trampoline's bed on its frame, a seat on its post. Five degrees of
//! freedom are locked and the sixth pushes back toward rest.
//!
//! Produces 6 constraint rows:
//! - Rows 0-1: rigid, across the axis — keep body_b's anchor on the line.
//! - Row 2: the spring, along the axis — a soft row refreshed every substep,
//!   so its force follows the compression rather than the frame's start.
//! - Rows 3-5: rigid, angular — hold body_b at the reference rotation.
//!
//! Every row is velocity-only. NGS position correction pins anchors together,
//! which would undo the spring's travel along the axis.

use nalgebra::{Isometry3, Point3, UnitQuaternion, UnitVector3, Vector3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::fixed::orientation_error;
use super::primitives::{self, BodySide, RowParams};
use super::spring::Spring;
use super::types::{ConstraintRow, CorrectionMode, Enforcement, RowRefresh};

/// Rows a spring slider expands to.
pub const ROW_COUNT: usize = 6;

/// Row index of the spring along the axis.
const SPRING_ROW: usize = 2;

/// A spring slider's definition.
#[derive(Debug, Clone)]
pub struct SpringSlider {
    /// The body carrying the line (None for world-anchored).
    pub body_a: Option<RigidBodyHandle>,
    /// The sprung body.
    pub body_b: RigidBodyHandle,
    /// Where body_b's anchor rests: body-local on body_a, or a world
    /// position when world-anchored.
    pub local_anchor_a: Vector3<f32>,
    /// Body-local anchor on body_b.
    pub local_anchor_b: Vector3<f32>,
    /// The slide axis in body_a's local frame (world-space when
    /// world-anchored). Positive travel along it stretches the spring.
    pub local_axis_a: UnitVector3<f32>,
    /// The rotation the joint holds body_b at, relative to body_a's (or to
    /// the world when world-anchored): `rot_a⁻¹ · rot_b`.
    pub reference: UnitQuaternion<f32>,
    /// The spring along the axis.
    pub spring: Spring,
    /// The mass the spring's frequency is stated against, in kg: its
    /// stiffness is `reference_mass · ω²`. A load of this mass on body_b
    /// bobs at the spring's frequency.
    pub reference_mass: f32,
}

impl SpringSlider {
    /// The slide axis in world space.
    pub fn world_axis(&self, body_a: Option<&RigidBody>) -> Vector3<f32> {
        match body_a {
            Some(a) => a.rotation() * self.local_axis_a.into_inner(),
            None => self.local_axis_a.into_inner(),
        }
    }

    /// Where body_b's anchor rests, in world space.
    pub fn rest_point(&self, body_a: Option<&RigidBody>) -> Point3<f32> {
        match body_a {
            Some(a) => a.position() + a.rotation() * self.local_anchor_a,
            None => Point3::from(self.local_anchor_a),
        }
    }

    /// How far body_b's anchor is from rest along the axis, in metres:
    /// negative when the spring is compressed.
    pub fn extension(&self, body_a: Option<&RigidBody>, body_b: &RigidBody) -> f32 {
        let anchor_b = body_b.position() + body_b.rotation() * self.local_anchor_b;
        (anchor_b - self.rest_point(body_a)).dot(&self.world_axis(body_a))
    }

    /// Expand into 6 solver rows.
    ///
    /// `beta` is the position correction factor (from solver config), used
    /// by the rigid rows. Each row converges where `J·v + bias` is zero, so
    /// a bias of `rate · error` drives its error back toward zero.
    pub fn expand(
        &self,
        body_a: Option<(&RigidBody, RigidBodyHandle)>,
        body_b: &RigidBody,
        dt: f32,
        beta: f32,
        constraint_index: generational_arena::Index,
        warm_impulses: &[f32],
    ) -> SmallVec<[ConstraintRow; ROW_COUNT]> {
        let rigid_a = body_a.map(|(a, _)| a);
        let anchor_b = body_b.position() + body_b.rotation() * self.local_anchor_b;
        let error = anchor_b - self.rest_point(rigid_a);
        let axis = self.world_axis(rigid_a);

        // Body_a's side acts at the point on it that body_b's anchor
        // occupies, which slides away from the rest point as the spring
        // works.
        let side_a = match body_a {
            Some((a, handle)) => BodySide {
                handle: Some(handle),
                inv_mass: a.inv_mass(),
                inv_inertia: a.world_inv_inertia(),
                lever_arm: anchor_b - a.position(),
            },
            None => BodySide::world(),
        };
        let side_b = BodySide {
            handle: Some(self.body_b),
            inv_mass: body_b.inv_mass(),
            inv_inertia: body_b.world_inv_inertia(),
            lever_arm: anchor_b - body_b.position(),
        };
        let params = |row_index: usize| RowParams {
            constraint_index,
            row_index,
            warm_impulse: warm_impulses.get(row_index).copied().unwrap_or(0.0),
        };

        let mut rows = SmallVec::new();

        let (across_1, across_2) = perpendiculars(&axis);
        for (row_index, across) in [(0, across_1), (1, across_2)] {
            let mut row = primitives::lock_linear_axis(
                &side_a,
                &side_b,
                across,
                (beta / dt) * error.dot(&across),
                0.0,
                f32::INFINITY,
                &params(row_index),
            );
            row.correction_mode = CorrectionMode::VelocityOnly;
            rows.push(row);
        }

        let soft = self.spring.soft_step(self.reference_mass, dt);
        let extension = error.dot(&axis);
        let mut spring = primitives::lock_linear_axis(
            &side_a,
            &side_b,
            axis,
            soft.bias_rate * extension,
            soft.softness,
            f32::INFINITY,
            &params(SPRING_ROW),
        );
        spring.correction_mode = CorrectionMode::VelocityOnly;
        spring.refresh = Some(RowRefresh {
            error: extension,
            bias_rate: soft.bias_rate,
            pose_a: rigid_a.map(pose),
            pose_b: Some(pose(body_b)),
        });
        rows.push(spring);

        let turned = orientation_error(
            rigid_a.map(|a| a.rotation()),
            body_b.rotation(),
            &self.reference,
        );
        for (i, world_axis) in [Vector3::x(), Vector3::y(), Vector3::z()]
            .into_iter()
            .enumerate()
        {
            rows.push(primitives::lock_angular_axis(
                &side_a,
                &side_b,
                world_axis,
                (beta / dt) * turned.dot(&world_axis),
                0.0,
                f32::INFINITY,
                CorrectionMode::VelocityOnly,
                Enforcement::Iterative,
                &params(3 + i),
            ));
        }

        rows
    }
}

/// Where a body is, for a refreshed row to measure its motion from.
fn pose(body: &RigidBody) -> Isometry3<f32> {
    Isometry3::from_parts(body.position().coords.into(), body.rotation())
}

/// Two unit vectors perpendicular to `axis` and to each other.
fn perpendiculars(axis: &Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let seed = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let first = axis.cross(&seed).normalize();
    (first, axis.cross(&first))
}
