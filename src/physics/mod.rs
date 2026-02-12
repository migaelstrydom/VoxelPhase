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

mod body;
mod collider;
mod collision;
mod debug;
pub mod grounding;
mod handle;
mod impulses;
mod math;
mod narrowphase;
mod pipeline;
mod sleep;
mod static_geometry;
mod world;

pub use body::RigidBodyDesc;
pub use collider::ColliderDesc;
pub use handle::RigidBodyHandle;
pub use impulses::{PhysicsImpulse, PhysicsImpulseQueue};
pub use narrowphase::ContactFeature;
pub use static_geometry::StaticGeometry;
pub use world::{ContactEvent, ContactSource, PhysicsWorld};
