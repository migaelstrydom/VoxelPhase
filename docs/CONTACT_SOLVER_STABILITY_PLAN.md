# Contact Solver Stability Plan

Design document for eliminating rotational jitter in resting, asymmetric-load contact stacks.

---

## Audience and purpose

This document is for:

- Junior engineers who need a clear mental model of why the jitter exists.
- Mid/senior engineers implementing and reviewing the fix.
- Future maintainers who need a long-lived explanation of solver design choices.

The goal is a professional-grade fix that aligns with patterns used by engines like Bullet and Rapier, while fitting RustDude's current architecture.

---

## Problem statement

In this scenario:

```text
          o     (heavy sphere, off-center)
  ----------------------
 |      platform box     |   (dynamic, thin in Y, wide in X/Z)
  ----------------------
 ------------------------   (flat static terrain)
```

the platform should settle. Instead, it exhibits persistent small rotational jitter.

### Reproduction

`src/physics/bench_harness.rs` now includes:

- `HeavySphereOnPlatformScenario`
- `heavy_sphere_on_platform_near_edge_settles_without_rotational_jitter`

This test currently fails with measurable tail angular speed, reproducing the in-game issue.

---

## Observed behavior and evidence

Diagnostics added in `solver.rs` show:

1. Contact points can remain stable, yet jitter persists.
2. Normal impulses at terrain support mostly contribute little direct torque.
3. Friction impulses produce alternating torque on the platform (sign and magnitude oscillate).
4. Warm tangential impulses are reused while contact normal/basis drifts slightly across frames.

That is sufficient to sustain angular chatter even when contact set membership is stable.

---

## Why this can happen physically and numerically

### Physical side

An off-center sphere applies a real external torque demand on the platform. The terrain reaction must cancel both:

- net force (vertical gravity load)
- net moment (about platform COM)

### Numerical side

The current solver is sequential-impulse with per-contact friction constraints. It approximates static equilibrium using:

- finite iterations
- per-contact Coulomb clamps
- warm-started cached impulses

This is robust for many cases, but under eccentric resting loads it can enter a limit cycle (small persistent oscillation), especially when friction impulses are basis-sensitive and solved independently per contact.

---

## Current solver model (simplified)

For a contact row with normal `n`:

- Relative velocity at contact:
  - `v_rel = (v_b + w_b x r_b) - (v_a + w_a x r_a)`
- Normal speed:
  - `v_n = dot(v_rel, n)`
- Effective inverse mass:
  - `k_n = m_a^-1 + m_b^-1 + n dot ( (I_a^-1 (r_a x n)) x r_a + (I_b^-1 (r_b x n)) x r_b )`
- Incremental normal impulse:
  - `d_lambda_n = -(v_n + restitution_term) / k_n`
  - `lambda_n = max(0, lambda_n_old + d_lambda_n)`

Friction is solved on two tangent axes `t1,t2`:

- `d_lambda_t1 = -dot(v_rel, t1) / k_t1`
- `d_lambda_t2 = -dot(v_rel, t2) / k_t2`
- Then projected to Coulomb disk:
  - `sqrt(lambda_t1^2 + lambda_t2^2) <= mu * lambda_n`

### Key weakness in current implementation

Warm friction is stored as scalar components `[lambda_t1, lambda_t2]` tied to a tangent basis from a previous frame, then reused in a newly computed basis. Small normal changes can rotate basis, making reused tangential impulses no longer represent the same physical world-space friction vector.

---

## Why penetration can still appear

Small penetration in this architecture is not always a bug by itself:

- contact margin is nonzero
- post-stabilization has slop/caps
- finite iterations do not enforce exact non-penetration

Persistent or visually obvious penetration together with jitter is a stability-quality issue, not necessarily a single collision-detection failure.

---

## Design goals

1. Remove persistent rotational jitter in resting asymmetric-load cases.
2. Preserve current broadphase/narrowphase/manifold architecture.
3. Keep behavior deterministic under `deterministic_contact_ordering`.
4. Improve, not regress, stacking and resting stability in existing tests.
5. Keep implementation incremental and reviewable.

## Non-goals

- Replacing the entire solver with a full MLCP or full NGS engine in one patch.
- Changing collision library geometry generation.
- Relying on sleep tuning as primary fix.

---

## Proposed professional fix

This is a staged implementation. Stage 1 and 2 are the core fix. Stage 3 is the stronger upgrade.

## Stage 1: World-space friction warmstart (required)

### Summary

Store and warm-start friction as a world-space tangent vector instead of basis-dependent scalar pair.

### Data model changes

Current cached friction:

- `warm_tangent_impulse: [f32; 2]`

New cached friction:

- `warm_friction_impulse_ws: Vector3<f32>` where `dot(warm_friction_impulse_ws, normal) = 0` (up to epsilon)

### Warm-start procedure

For each contact:

1. Read cached world-space friction `j_t_ws_prev`.
2. Project to current tangent plane:
   - `j_t_ws = j_t_ws_prev - dot(j_t_ws_prev, n) * n`
3. Clamp to current Coulomb limit from warm normal:
   - `|j_t_ws| <= mu * lambda_n_warm`
4. Apply warm impulse:
   - `j_warm = n * lambda_n_warm + j_t_ws`

### Why this helps

The cached friction direction remains physically meaningful across basis drift. This removes basis-rotation artifacts without using arbitrary angle thresholds.

---

## Stage 2: Manifold-level friction solve (required)

### Summary

Solve friction at the manifold level with a shared normal-load budget, rather than independent per-point friction rows.

### Motivation

Independent per-contact friction constraints can "fight" and generate net torque oscillation even with stable contacts.

### Formulation

For a manifold with contacts `i = 1..m`:

- Let each contact have tangent basis `t1_i, t2_i`.
- Unknown friction increments per contact:
  - `d_lambda_t_i = [d_lambda_t1_i, d_lambda_t2_i]`
- Total manifold friction magnitude is constrained by shared budget:
  - `sum_i |lambda_t_i| <= mu * sum_i lambda_n_i`
  - or practically, project concatenated tangential impulse vector onto a ball with radius `mu * sum_i lambda_n_i`.

This can be implemented as:

1. Compute unconstrained tangential update over all contacts.
2. Concatenate into vector `L_t`.
3. Project `L_t` to manifold Coulomb ball radius `R = mu * sum(lambda_n_i)`.
4. Apply per-contact impulse deltas derived from projected `L_t`.

### Practical note

A simpler first pass is to keep per-contact tangent basis but enforce shared budget with one projection step per manifold. This already removes much of the torque ping-pong.

---

## Stage 3: Block normal solve for multi-point support manifolds (recommended)

### Summary

Replace scalar per-contact normal solve with a small block solve for manifold normal constraints (2-4 contacts).

### Why

Sequential scalar rows converge slowly in coupled support problems and can oscillate under heavy eccentric loads.

### Formulation

Build normal Jacobian rows `J_n_i` for manifold contacts. Solve:

- `A * lambda_n = b`
- with complementarity `lambda_n >= 0`

Where:

- `A_ij = J_n_i * M^-1 * J_n_j^T`
- `b_i = -(v_n_i + bias_i)`

Use a projected Gauss-Seidel micro-iteration on this small block or a direct constrained solve with projection.

Even a few local iterations on this block per manifold greatly reduces rocking.

---

## How Bullet/Rapier-style engines address this class of issue

While implementations differ, mature engines generally combine:

1. Persistent manifolds with robust warmstart matching.
2. Friction handling that is stable under manifold contact coupling.
3. Carefully designed stabilization (ERP/split impulse) to avoid energy injection.
4. Constraint ordering and iteration heuristics tuned for stack stability.
5. Sleep as a final residual-motion terminator, not primary correctness tool.

RustDude's staged plan mirrors this progression and fits the existing pipeline.

---

## Implementation plan by file

## Stage 1 files

- `src/physics/pipeline/pair.rs`
  - Add world-space warm friction field on `SolverContact`.
  - Keep legacy fields temporarily only if needed for migration.
- `src/physics/pipeline/manifold.rs`
  - Cache/read/write world-space warm friction impulse.
  - Maintain backward compatibility during transition if required.
- `src/physics/pipeline/solver.rs`
  - Warm-start friction using world-space projection.
  - Friction solve writes world-space accumulated friction back.
- `src/physics/world.rs`
  - Initialize cold contacts with zero world-space warm friction.

## Stage 2 files

- `src/physics/pipeline/solver.rs` (primary)
  - Add manifold-level friction projection and shared budget.
  - Keep deterministic ordering.
- `src/physics/pipeline/pair.rs` / `manifold.rs`
  - Minimal shape adjustments for manifold friction state.

## Stage 3 files

- `src/physics/pipeline/solver.rs`
  - Add block normal solve path for manifolds with `m > 1`.

---

## Validation plan

### Existing tests that must remain green

- Resting/contact tests in `bench_harness.rs`
- Any OBB/stack/bowl/ramp stability tests

### Key target

- `heavy_sphere_on_platform_near_edge_settles_without_rotational_jitter`
  - Should pass with low tail angular speed.

### Additional tests to add

1. **Basis invariance warmstart test**
   - Reorient normal slightly across frames, verify warm friction impulse does not inject torque spikes.
2. **Shared-budget manifold friction test**
   - Two-contact support manifold under off-center load; verify reduced torque oscillation.
3. **Regression tests**
   - Sliding behavior still correct for low-friction scenarios.

### Metrics to track

- Tail max angular speed of platform.
- Contact manifold churn (already available).
- Penetration depth distribution in resting window.

---

## Risks and mitigations

1. **Risk:** Over-damping/sticky friction after shared budget.
   - **Mitigation:** Add benchmark coverage for sliding scenarios.
2. **Risk:** Determinism regressions.
   - **Mitigation:** Keep deterministic manifold/contact ordering and avoid hash-iteration dependence.
3. **Risk:** Performance regression from block solve.
   - **Mitigation:** Apply block solve only for `m > 1`, cap local iterations, benchmark.

---

## Rollout strategy

1. Land Stage 1 behind no feature flag (safe and localized).
2. Run full tests and benchmark exports.
3. Land Stage 2 with targeted tests and diagnostics.
4. Land Stage 3 only if Stage 1+2 still leave visible jitter in stress cases.

---

## Appendix A: Junior-friendly intuition

Think of each contact as a tiny "hand" pushing on the platform.

- Normal impulse = hand pushing up.
- Friction impulse = hand pushing sideways.

Today, each hand decides sideways push mostly on its own, and remembers it in local coordinates that can rotate a little between frames. If those remembered sideways pushes are reused in slightly rotated directions, the platform can keep getting tiny twisting nudges.

The fix is:

1. Remember sideways push in world direction (not local rotating coordinates).
2. Make the hands coordinate their total sideways push as one group (manifold), so they do not fight each other.

That is the core of "professional" contact stability for this problem.

---

## Appendix B: Glossary

- **Manifold:** up to 4 contact points for one collider pair.
- **Warm-start:** reuse previous-frame impulses to start near equilibrium.
- **PGS:** Projected Gauss-Seidel, iterative constraint solver.
- **Split impulse:** position-correction method that reduces artificial energy gain.
- **Coulomb friction cone/disk:** bound on tangential impulse magnitude by `mu * normal`.

