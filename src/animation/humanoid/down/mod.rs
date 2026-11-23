//! A knocked-down body, drawn as it lies and as it moves: buckling where it
//! was knocked down standing, tucked while it tumbles end over end, a log
//! roll along the ground, spread out as it slides, then gathered on its knees
//! or sat back on its hands, and getting up from there as the body pitches
//! up.
//!
//! ```text
//!   Lying (the body's turn, the ground under it) + DownPhase
//!        │  DownFigure::advance (figure.rs): the body's spin, the ground
//!        │  kept through bounces
//!        ▼
//!   posture.rs: the posture it is in, with hysteresis, and each posture's
//!        │  weight eased toward that one
//!        ▼
//!   limbs.rs: each posture's trunk (pelvis lowered, torso leaned) and the
//!        │  feet and hands placed from it, in the body's current frame
//!        ▼
//!   DownPose, blended by weight once a frame ──▶ PoseState::Down (feet, trunk)
//!                                            ──▶ UpperState::Down (hands)
//! ```
//!
//! The capsule the body is drawn in is round and half a metre thick, and the
//! figure is a thin one: laid flat along the capsule's axis it would hang a
//! fifth of a metre over the ground. So no posture on the ground lies flat:
//! the limbs reach down to the ground the capsule rests on (ground.rs) and
//! hold the figure up where the capsule is.

mod figure;
mod ground;
mod limbs;
mod posture;

pub use figure::{DownFigure, DownPose, Lying};
pub use ground::GroundPlane;
