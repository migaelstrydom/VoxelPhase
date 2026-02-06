# Physics Engine Architecture

## Ideal Architecture Overview

This section summarizes an idealized, stable, and scalable physics engine pipeline.
The goal is high stability for resting contacts and good performance for fast motion.

### Core Phases

- **Broadphase**
  - Maintain an acceleration structure (AABB tree, sweep-and-prune, or grid).
  - Generate candidate pairs for narrowphase and CCD.

- **Narrowphase**
  - Compute contact manifolds for overlapping pairs.
  - For convex shapes, use GJK/EPA or SAT-based approaches.
  - For triangle meshes, use localized triangle queries.

- **CCD / TOI (Time of Impact)**
  - For fast-moving or flagged pairs, compute TOI using CCD.
  - Use conservative advancement for general shapes, or exact TOI when available
    (e.g., swept spheres).
  - Handle initial overlap explicitly (TOI = 0.0).

- **Contact Manifolds**
  - Generate a stable set of contact points and normals per pair.
  - Use reduction to keep a small number of contacts for solver efficiency.

- **Contact Caches**
  - Cache manifolds and impulses per contact.
  - Reuse previous normals/points to stabilize rest contacts.
  - Warm-start the solver using cached impulses.

- **Constraint Solve**
  - Solve contacts and joints with an iterative solver (sequential impulses).
  - Use multiple iterations, warm-starting, and friction constraints.

- **Integration**
  - Predict positions into a buffer.
  - Resolve collisions and constraints.
  - Commit predicted positions at the end of the step.

### Stability Practices

- Warm-start impulses for resting contacts.
- Use multiple solver iterations.
- Cache manifolds to reduce normal/point jitter.
- Use position correction with limited bias.
- Apply CCD only to fast/important bodies to limit cost.

## Current Implementation in RustDude

This reflects the state of the engine as implemented in `src/physics/world.rs`
and related modules.

### Implemented

- **TOI / CCD for spheres**
  - Swept sphere vs triangle for static geometry.
  - Swept sphere vs sphere for dynamic pairs.
  - Initial overlap returns TOI = 0.0.

- **Predicted integration**
  - Positions are predicted; committed at end of step.

- **Event-driven collision resolution**
  - CCD events are collected and sorted by time.
  - Collisions resolved in time order.
  - Re-CCD per body up to a maximum iteration count.

- **Contact solving**
  - Sequential impulse solver with restitution and friction.
  - Position correction for penetration.

- **Contact caches**
  - Body-body manifold cache (normal only).
  - Static contact cache per body+collider (normal + point).
  - Warm-start cache of normal impulses with decay.

### Not Implemented Yet

- Broadphase (AABB tree or sweep-and-prune).
- Persistent multi-point manifolds per pair.
- Full warm-starting for friction impulses.
- Contact reduction and manifold pruning.
- Island building and multithreaded solve.
- Sleeping / deactivation for resting bodies.

### Notes

- The current CCD loop is designed for correctness and stability before speed.
- Static contact caching and warm-starting are in place to reduce jitter.
- Further stability improvements are expected from higher solver iteration count
  and friction warm-starting.
