//! The suspension: a body held up on a spring above whatever a cast from it
//! hits, with friction where it stands.
//!
//! ```text
//!         anchor ●  (on the body)
//!                │  rest length                 ┐
//!                │                              │ reach (rest length + slack)
//!                ○  cast sphere                 │
//!   ground ──────┴───────────────────────────── ┘
//!
//!   frame:    world casts the sphere ──► SuspensionHit (point, normal, partner, μ)
//!   expand:   spring ──► friction pair ──► torsion   (4 rows, in that order)
//!   substep:  spring bias refreshed from the motion since expansion
//! ```
//!
//! It is the raycast wheel of vehicle physics and knows nothing of legs.
//!
//! - **The spring** pushes along the axis, through the anchor on the body
//!   and through the cast sphere's centre on the partner (which lies on the
//!   axis line, unlike the hit point). It is soft (`Spring`, against the
//!   body's own mass), push-only, bounded by `max_force`, and refreshed every
//!   substep so its force follows its compression.
//! - **The friction pair** lies in the hit's tangent plane, at the hit point,
//!   and is bounded together with the spring's own share along the surface,
//!   `|λ_s·axis_t + λ_t| ≤ g·μ·N` with `N = λ_s·(axis·n)`: Coulomb friction
//!   for any spring axis. The tangent basis is fixed against a world
//!   reference, so warm impulses mean the same thing frame to frame.
//! - **The torsion row** turns about the hit normal, bounded by `g·μ·r·N`.
//!
//! None of the rows takes NGS position correction (they are `VelocityOnly`)
//! or shock-propagation mass scaling (which applies to contacts only).
//!
//! Warm impulses live in the constraint's slots, so they survive the cast
//! crossing triangles; they are zeroed when the partner changes or nothing is
//! hit, so stepping from rock onto a light can does not warm-start the body's
//! whole weight into the can.

mod cast;
mod declaration;
mod hit;
mod rows;
mod state;
#[cfg(test)]
mod tests;

pub(crate) use cast::cast_suspension;
pub use declaration::{SuspensionAxis, SuspensionDesc};
pub use hit::SuspensionHit;
pub use rows::{expand, ROW_COUNT};
pub use state::{Suspension, SuspensionReadout};
