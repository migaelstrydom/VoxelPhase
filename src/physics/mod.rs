//! Rigid body physics engine.
//!
//! This module provides a handle-based physics simulation with:
//! - Rigid bodies with position, rotation, and velocities
//! - Collision shapes (sphere, with more to come)
//! - Collision detection and response
//! - Integration with external static geometry (terrain)
//!
//! # Architecture
//!
//! The physics engine is self-contained and communicates with the game via:
//! - Handles (opaque identifiers for physics objects)
//! - Descriptors (immutable specs for creating objects)
//! - The StaticGeometry trait (for terrain collision)
//!
//! # Usage
//!
//! ```ignore
//! use physics::{PhysicsWorld, PhysicsConfig, RigidBodyDesc, ColliderDesc};
//!
//! // Create the physics world
//! let mut physics = PhysicsWorld::new(PhysicsConfig::default());
//!
//! // Create a dynamic body
//! let body = physics.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 10.0, 0.0)));
//!
//! // Attach a sphere collider
//! physics.attach_collider(body, ColliderDesc::sphere(0.5).restitution(0.8));
//!
//! // Step simulation
//! physics.step(1.0 / 60.0, &terrain);
//!
//! // Read back state
//! let pos = physics.body(body).unwrap().position();
//! ```

pub mod bench_harness;
mod body;
mod broadphase;
pub mod ccd;
mod collider;
pub mod constraint;
mod contact_event;
mod debug;
pub mod drive;
mod force_provider;
pub mod grounding;
mod handle;
mod impact;
mod impulses;
mod math;
mod narrowphase;
mod pipeline;
mod sleep;
pub mod solver;
mod static_geometry;
pub mod stepping;
mod world;

pub use body::{BodyType, RigidBody, RigidBodyDesc};
pub use ccd::{CcdStrategy, SweepClampCcd};
pub use collider::{Collider, ColliderDesc, ColliderMaterial, ColliderShape, FrictionModel};
pub use constraint::{Constraint, ConstraintHandle, ConstraintKind};
pub use contact_event::{ContactEvent, ContactSource};
pub use drive::{
    Allowance, AllowanceCommand, AllowanceUsage, DriveCommand, NormalProjection, NormalVerbs,
    ReactionAnchor, SupportConfig, SupportContact, SupportResolver, SupportSet, SupportSets,
};
pub use force_provider::{ForceContext, ForceOutput, SubstepForceProvider};
pub use handle::{ColliderHandle, RigidBodyHandle};
pub use impact::{BodyImpact, ImpactLedger};
pub use impulses::{PhysicsImpulse, PhysicsImpulseQueue};
pub use solver::{
    ConstraintSolver, IdentityConditioner, ManifoldConditioner, ManifoldConditions, PgsNgsConfig,
    PgsNgsSolver, ShockPropagationConditioner, ShockPropagationConfig,
};
pub use static_geometry::StaticGeometry;
pub use stepping::{FixedTimestep, SequentialStepper, Stepper};
pub use world::{BodyProbeHit, PhysicsConfig, PhysicsWorld};
