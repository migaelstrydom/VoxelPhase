//! Solver-ready contact rows, prepared once per substep.
//!
//! A contact's velocity rows are solved many times per substep — every outer
//! iteration, and the normal row several times within each — but almost
//! everything they read is fixed while the velocity phase runs: the bodies do
//! not move or turn until integration, so the lever arms, the world-space
//! inverse inertias, the tangent basis and every effective mass are the same
//! on the last pass as on the first. Only the velocities change.
//!
//! ```text
//!   SolverContact ──prepare──▶ ContactRow ──┬─▶ normal row    ┐
//!   (geometry)       once per   (constant    ├─▶ tangent rows  ├─ read live velocities,
//!                    substep     for the     ├─▶ torsional row │  write impulses through
//!                                substep)    └─▶ friction proj ┘  RowBody::apply_*
//! ```
//!
//! Each value is computed by the same expression the rows used to evaluate
//! inline, so preparing it once changes what the solver costs and nothing
//! about what it produces.

use generational_arena::{Arena, Index};
use nalgebra::{Matrix3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::body_pair::{is_kinematic_static, BodyPairState};
use super::impulse::compute_tangent_basis;

/// How one side of a contact answers an impulse, fixed for the substep.
#[derive(Debug, Clone, Copy)]
enum ImpulseResponse {
    /// Static, sleeping-kinematic, or massless: impulses change nothing.
    Immovable,
    /// A dynamic body, with its real (unscaled) inverse mass and world-space
    /// inverse inertia.
    Dynamic {
        inv_mass: f32,
        world_inv_inertia: Matrix3<f32>,
    },
    /// A kinematic body against static geometry: the normal row may push it
    /// out, linearly only. See `body_pair::is_kinematic_static`.
    KinematicOnStatic,
}

/// One body of a contact pair as the rows see it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RowBody {
    /// Arena slot, for the live velocities.
    index: Index,
    /// Contact point minus the body's position.
    lever: Vector3<f32>,
    /// How impulses change its velocities.
    response: ImpulseResponse,
}

impl RowBody {
    fn new(index: Index, body: &RigidBody, lever: Vector3<f32>, header: &PairHeader) -> Self {
        let response = if body.is_dynamic() && body.inv_mass() > 0.0 {
            ImpulseResponse::Dynamic {
                inv_mass: body.inv_mass(),
                world_inv_inertia: body.world_inv_inertia(),
            }
        } else if is_kinematic_static(body, header) {
            ImpulseResponse::KinematicOnStatic
        } else {
            ImpulseResponse::Immovable
        };
        Self {
            index,
            lever,
            response,
        }
    }

    pub fn lever(&self) -> Vector3<f32> {
        self.lever
    }

    /// Velocity of the body's material at the contact point, now.
    fn point_velocity(&self, bodies: &Arena<RigidBody>) -> Vector3<f32> {
        match bodies.get(self.index) {
            Some(body) => body.linear_velocity() + body.angular_velocity().cross(&self.lever),
            None => Vector3::zeros(),
        }
    }

    fn angular_velocity(&self, bodies: &Arena<RigidBody>) -> Vector3<f32> {
        bodies
            .get(self.index)
            .map_or_else(Vector3::zeros, |body| body.angular_velocity())
    }

    /// Apply an impulse at the contact point, as `RigidBody::apply_impulse_at_point`
    /// would, but with the inertia prepared for the substep.
    pub fn apply_impulse(&self, bodies: &mut Arena<RigidBody>, impulse: Vector3<f32>) {
        match self.response {
            ImpulseResponse::Immovable => {}
            ImpulseResponse::Dynamic {
                inv_mass,
                world_inv_inertia,
            } => {
                let Some(body) = bodies.get_mut(self.index) else {
                    return;
                };
                body.set_linear_velocity(body.linear_velocity() + impulse * inv_mass);
                let angular_impulse = self.lever.cross(&impulse);
                body.set_angular_velocity(
                    body.angular_velocity() + world_inv_inertia * angular_impulse,
                );
            }
            ImpulseResponse::KinematicOnStatic => {
                let Some(body) = bodies.get_mut(self.index) else {
                    return;
                };
                body.set_linear_velocity(body.linear_velocity() + impulse);
            }
        }
    }

    /// Apply an angular impulse, as `RigidBody::apply_angular_impulse` would.
    pub fn apply_angular_impulse(
        &self,
        bodies: &mut Arena<RigidBody>,
        angular_impulse: Vector3<f32>,
    ) {
        let ImpulseResponse::Dynamic {
            world_inv_inertia, ..
        } = self.response
        else {
            return;
        };
        if let Some(body) = bodies.get_mut(self.index) {
            body.set_angular_velocity(
                body.angular_velocity() + world_inv_inertia * angular_impulse,
            );
        }
    }
}

/// Everything about one contact that the velocity rows need and that does not
/// change until the bodies are integrated.
///
/// Masses and inertias in the effective-mass terms carry the manifold's shock
/// scales; the impulse a row applies is scaled per side by the same factors,
/// exactly as `apply_impulse_pair` scales it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct ContactRow {
    /// The static-or-dynamic first body; `None` for static geometry.
    pub body_a: Option<RowBody>,
    pub body_b: RowBody,
    /// Per-side shock propagation scale `(a, b)`.
    pub shock_scales: (f32, f32),
    /// Effective inverse mass along the normal, with the kinematic-on-static
    /// override applied.
    pub normal_inv_mass: f32,
    /// Orthonormal tangent basis perpendicular to the normal.
    pub tangents: (Vector3<f32>, Vector3<f32>),
    /// Effective inverse mass along each tangent. Real masses: friction never
    /// takes the kinematic override.
    pub tangent_inv_mass: (f32, f32),
    /// Effective inverse inertia about the normal, for the torsional row.
    pub torsional_inv_inertia: f32,
}

impl ContactRow {
    /// Prepare a contact's rows from the bodies' current pose.
    ///
    /// `None` when either body is gone, in which case every row of the contact
    /// is a no-op beyond the bookkeeping its solve function does first.
    pub fn prepare(
        bodies: &Arena<RigidBody>,
        header: &PairHeader,
        contact: &SolverContact,
        shock_scales: (f32, f32),
    ) -> Option<Self> {
        let state = BodyPairState::extract(bodies, header, contact.point, shock_scales)?;
        let body_b = bodies.get(header.body_b.0)?;
        let point = contact.point;
        let normal = contact.normal;

        let body_a = match header.body_a {
            Some(handle) => {
                let body = bodies.get(handle.0)?;
                Some(RowBody::new(handle.0, body, point - state.pos_a, header))
            }
            None => None,
        };

        let (normal_inv_mass_b, normal_inv_inertia_b) = if is_kinematic_static(body_b, header) {
            (1.0, Matrix3::zeros())
        } else {
            (state.inv_mass_b, state.inv_inertia_b)
        };
        let tangents = compute_tangent_basis(&normal);

        Some(Self {
            body_a,
            body_b: RowBody::new(header.body_b.0, body_b, point - state.pos_b, header),
            shock_scales,
            normal_inv_mass: state.effective_inv_mass_with_overrides(
                point,
                &normal,
                normal_inv_mass_b,
                normal_inv_inertia_b,
            ),
            tangents,
            tangent_inv_mass: (
                state.effective_inv_mass(point, &tangents.0),
                state.effective_inv_mass(point, &tangents.1),
            ),
            torsional_inv_inertia: normal
                .dot(&((state.inv_inertia_a + state.inv_inertia_b) * normal)),
        })
    }

    /// Relative velocity at the contact (B minus A), from the live velocities.
    pub fn relative_velocity(&self, bodies: &Arena<RigidBody>) -> Vector3<f32> {
        let at_a = self
            .body_a
            .map_or_else(Vector3::zeros, |a| a.point_velocity(bodies));
        self.body_b.point_velocity(bodies) - at_a
    }

    /// Relative angular velocity (B minus A), from the live velocities.
    pub fn relative_angular_velocity(&self, bodies: &Arena<RigidBody>) -> Vector3<f32> {
        let of_a = self
            .body_a
            .map_or_else(Vector3::zeros, |a| a.angular_velocity(bodies));
        self.body_b.angular_velocity(bodies) - of_a
    }

    /// Apply `impulse` to B and its opposite to A, each scaled by its shock
    /// factor — `apply_impulse_pair` against the prepared bodies.
    pub fn apply_impulse(&self, bodies: &mut Arena<RigidBody>, impulse: Vector3<f32>) {
        if let Some(a) = &self.body_a {
            a.apply_impulse(bodies, -impulse * self.shock_scales.0);
        }
        self.body_b
            .apply_impulse(bodies, impulse * self.shock_scales.1);
    }

    /// The angular counterpart of [`ContactRow::apply_impulse`].
    pub fn apply_angular_impulse(
        &self,
        bodies: &mut Arena<RigidBody>,
        angular_impulse: Vector3<f32>,
    ) {
        if let Some(a) = &self.body_a {
            a.apply_angular_impulse(bodies, -angular_impulse * self.shock_scales.0);
        }
        self.body_b
            .apply_angular_impulse(bodies, angular_impulse * self.shock_scales.1);
    }
}

/// The contact rows of a whole manifold slice, flat, with each manifold's
/// range. Rebuilt every substep; the buffers are kept to avoid reallocating.
#[derive(Debug, Default)]
pub(crate) struct ContactRows {
    /// One entry per contact, manifold by manifold, in contact order.
    rows: Vec<Option<ContactRow>>,
    /// Pre-solve relative normal velocity per contact, before any impulse this
    /// substep — what restitution is decided from.
    pre_solve_normal_velocity: Vec<f32>,
    /// Index of each manifold's first row.
    starts: Vec<usize>,
}

impl ContactRows {
    pub fn prepare<'a>(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: impl Iterator<Item = (&'a PairHeader, &'a [SolverContact], (f32, f32))>,
    ) {
        self.rows.clear();
        self.pre_solve_normal_velocity.clear();
        self.starts.clear();
        for (header, contacts, shock_scales) in manifolds {
            self.starts.push(self.rows.len());
            for contact in contacts {
                let row = ContactRow::prepare(bodies, header, contact, shock_scales);
                let vn = row.map_or(0.0, |row| {
                    row.relative_velocity(bodies).dot(&contact.normal)
                });
                self.rows.push(row);
                self.pre_solve_normal_velocity.push(vn);
            }
        }
    }

    /// The rows of manifold `manifold`, in contact order.
    pub fn manifold(&self, manifold: usize) -> &[Option<ContactRow>] {
        &self.rows[self.range(manifold)]
    }

    /// The pre-solve normal velocities of manifold `manifold`, in contact order.
    pub fn pre_solve_normal_velocities(&self, manifold: usize) -> &[f32] {
        &self.pre_solve_normal_velocity[self.range(manifold)]
    }

    fn range(&self, manifold: usize) -> std::ops::Range<usize> {
        let start = self.starts[manifold];
        let end = self
            .starts
            .get(manifold + 1)
            .copied()
            .unwrap_or(self.rows.len());
        start..end
    }
}
