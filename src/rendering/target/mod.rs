//! Where a frame is rendered, and where it goes when it is finished.
//!
//! Split into two halves so the renderer does not care whether its output is
//! presented to a window or read back to a file:
//!
//! ```text
//!   FrameTargets            FrameOutput (trait)
//!   ────────────            ───────────────────
//!   HDR colour target       SwapchainOutput  → window
//!   depth buffer            OffscreenOutput  → PNG / raw pixels
//!   refraction copy
//!   framebuffers
//! ```
//!
//! `FrameTargets` owns everything the render passes draw into. `FrameOutput`
//! owns only the final images and the rules for acquiring and releasing them.

pub mod frame_targets;
pub mod images;
pub mod offscreen;
pub mod output;
pub mod refraction;
pub mod swapchain;
pub mod sync;

pub use frame_targets::FrameTargets;
pub use images::{ColorTarget, DepthBuffer};
pub use offscreen::OffscreenOutput;
pub use output::{AcquiredFrame, FrameOutput};
pub use refraction::RefractionCopy;
pub use swapchain::{SurfaceInfo, SwapchainOutput};
pub use sync::FrameSync;
