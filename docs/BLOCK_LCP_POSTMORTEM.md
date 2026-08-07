# Block LCP Solver: Postmortem

An attempt to replace PGS micro-iterations with an exact N-contact LCP solver for 2-4 contact manifolds. The implementation is mathematically correct and works for 2-contact manifolds, but causes cross-manifold convergence regressions for 3-4 contacts. Code preserved on branch `feature/block-lcp-solver`.

---

## Motivation

The solver has a 2-contact block solver (inline LCP with 4-case enumeration) and falls back to PGS micro-iterations for 3+ contacts. Box towers are unstable because 4-contact manifolds (box resting on surface) don't get the coupled solve. The goal was to extend the block solver to handle 3-4 contacts via full LCP enumeration.

## What was built

### `src/physics/pipeline/block_lcp.rs`

A pure math module implementing an N-contact LCP solver (N <= 4):

- **Algorithm**: Enumerate all 2^N subsets of active contacts (max 16 for N=4), ordered by decreasing popcount (prefer more contacts active). For each subset, solve the reduced linear system and check feasibility. First feasible solution wins.
- **Reduced system solves**: 1x1 direct division, 2x2 analytical inverse, 3x3 cofactor inverse, 4x4 cofactor inverse. All fixed-size arrays, no heap allocation.
- **LCP formulation**: `v = q + K*(a_new - a_old) >= 0`, `a_new >= 0`, `v * a_new = 0`, where `q[i] = vn[i] + rv[i]` (velocity + restitution bias).
- **Fallback**: Sequential clamped PGS pass if no LCP case is feasible.
- **Unit tests**: 13 tests validating 1x1 through 4x4 cases, including warm-start, mixed active/separating, and identity matrix.

### Solver integration

`solve_normal_block` replaced the old `solve_normal_block_2`:
- Builds N*N effective mass matrix K using `cross_effective_inv_mass_with_overrides`
- Delegates to `block_lcp::solve_lcp`
- Applies impulse deltas to bodies

## What went wrong

### 2-contact manifolds: works perfectly

The block LCP for N=2 produces identical results to the old inline `solve_normal_block_2`. All existing tests pass. This is expected since both solve the same 2x2 system with the same 4-case enumeration.

### 3-4 contact manifolds: cross-manifold convergence regression

Two test regressions:

| Test | Old (PGS micro-iter) | New (block LCP) | Limit |
|------|---------------------|-----------------|-------|
| `box_stack_settles_without_overlap` | passes (< 0.08) | 0.104 tail speed | 0.08 |
| `heavy_sphere_on_platform_near_edge` | 0.000003 angular | 0.018 angular | 0.005 |

**Symptom 1 — box stack drift**: Stacked boxes slowly rise at 0.003-0.007 m/s instead of settling to rest. Persistent upward drift, not transient.

**Symptom 2 — platform angular jitter**: A thin platform with a heavy sphere near its edge develops 0.018 rad/s angular oscillation (vs 0.000003 with old solver).

### Root cause analysis

The issue is **not** per-manifold accuracy — the block LCP gives the exact solution for each manifold in isolation. The issue is how the exact solve interacts with the outer (cross-manifold) iteration loop.

**Old behavior (PGS micro-iterations)**: 4 sequential passes per manifold per outer iteration. Each pass applies a small incremental correction. The body's velocity is updated between passes, and subsequent passes see the updated state. This incremental convergence acts as implicit SOR under-relaxation, which improves cross-manifold convergence stability.

**New behavior (block LCP)**: A single exact solve per manifold per outer iteration. The full correction is applied at once. When many manifolds share a body (e.g., a platform spanning many terrain triangles, or stacked boxes), each manifold's "exact jump" creates a larger velocity perturbation that adjacent manifolds must then correct. This amplifies inter-manifold oscillation that the outer iteration loop cannot damp sufficiently.

The key insight: **per-manifold exactness hurts cross-manifold convergence**. PGS's gradual convergence is not a bug — it's a feature that provides natural damping in the coupled multi-manifold system.

## Approaches tried

| Approach | box_stack | heavy_sphere | Notes |
|----------|-----------|--------------|-------|
| Pure block LCP (omega=1.0) | 0.104 (FAIL) | 0.018 (FAIL) | Baseline regression |
| SOR under-relaxation (omega=0.8, nc>2) | PASS | 0.025 (WORSE) | Relaxation helps stacks but hurts asymmetric loads |
| SOR under-relaxation (omega=0.8, all nc) | PASS | 0.025 (WORSE) | Same — omega doesn't help platform jitter |
| Block LCP + 1 PGS cleanup pass | 0.007 (PASS) | 0.018 (FAIL) | PGS cleanup helps stacks, not platforms |
| Block LCP + 3 PGS cleanup passes | similar | 0.018 (FAIL) | More passes don't help |
| Block LCP + 4 PGS cleanup passes | similar | 0.018 (FAIL) | Diminishing returns |

No combination of relaxation and PGS cleanup resolved both test cases simultaneously.

## Lessons learned

1. **Exact per-manifold solves are not always better.** In an iterative multi-manifold solver, the convergence properties of the per-manifold solve matter as much as its accuracy. PGS's incremental convergence provides natural damping that exact solvers lack.

2. **The 2-contact block solver works because manifolds rarely share both contacts.** With 2-contact manifolds, the coupling is local and the exact solve doesn't create large cross-manifold perturbations. With 3-4 contacts, the impulse magnitudes are larger and the cross-manifold effects are more pronounced.

3. **Under-relaxation doesn't uniformly help.** SOR with omega < 1 helped the box stack (where the issue was oscillation between vertically-coupled manifolds) but worsened the platform scenario (where the issue was rotational asymmetry across many terrain-face manifolds). The optimal omega depends on the contact topology.

4. **The right fix is probably not at the per-manifold solve level.** Professional engines address this with substepping (Box2D v3, bepuphysics2) or Temporal Gauss-Seidel (PhysX), which fundamentally change how cross-manifold coupling is resolved. See `SOLVER_CONVERGENCE_ANALYSIS.md`.

## Future directions

- **Substepping**: Running multiple physics substeps per frame with fewer iterations each would improve cross-manifold convergence regardless of the per-manifold solver. This is the approach taken by modern engines.
- **Graph-ordered solving**: Processing manifolds bottom-up in a contact graph (shock propagation) would let the block LCP's exact solve propagate correctly through stacks without oscillation.
- **Selective block LCP**: Use block LCP only for isolated manifolds (bodies with a single manifold) and PGS for bodies participating in multiple manifolds. Requires tracking manifold-per-body counts.
- **The block_lcp module is ready**: The N-contact LCP solver is correct, tested, and efficient. It's a valid building block for any of the above approaches. Branch: `feature/block-lcp-solver`.
