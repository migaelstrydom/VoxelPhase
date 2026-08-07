# Solver Convergence Analysis: Substepping vs. Iterations

Why smaller timesteps with fewer solver iterations converge better than larger timesteps with more iterations — and what professional physics engines actually do about it.

---

## How professional engines solve stack stability

| Engine | Solver type | Stack stability mechanism | Graph-ordered shock propagation? |
|--------|-------------|---------------------------|----------------------------------|
| **PhysX 4+** | TGS (Temporal Gauss-Seidel) | Substepping with velocity re-integration between passes; friction applied throughout | No |
| **Box2D v3** | TGS_Soft | Substepping + soft spring-damper constraints; 1 sweep per substep | No |
| **Rapier** | Non-linear PGS | Warm-start + sleep; docs recommend increasing substeps over iterations | No |
| **bepuphysics2** | Custom substepping | 1–2 iterations per substep; collision detection amortized across substeps | No |
| **Bullet** | PGS | Warm-start + split impulse + sleep. **Jitter is visible during settlement**, especially with high mass ratios. Sleep hides it after convergence. | No |

No major engine uses graph-ordered shock propagation. The technique originates from Guendelman et al. (2003) and Erleben (2007), primarily in offline animation / VFX contexts where the extra graph-build cost was acceptable. The game engine industry converged on a different answer: make the timestep smaller.

---

## The PGS linearization problem

### Setup

A rigid-body constraint `C(q) = 0` has velocity-level form:

```
Ċ = J(q) v = 0
```

where `J = ∂C/∂q` is the constraint Jacobian. PGS solves `m` such constraints by cycling through them in sequence, projecting each impulse onto its feasible set (`μ ≥ 0` for contacts), keeping all others fixed. This is block coordinate descent on the dual LCP.

The impulse that satisfies constraint `i` at the current linearization point is:

```
Δμᵢ = -K⁻¹ (J v + b)

where K = J M⁻¹ Jᵀ   (effective mass / constraint-space inertia)
      b = (β/h) C      (Baumgarte bias, β ∈ (0,1))
```

### The core issue: all iterations share the same linearization

Standard PGS loops over the same Jacobians `J(qⁿ)` evaluated at the **start-of-step** configuration `qⁿ`. Each iteration `k = 1, ..., N` refines the impulse vector `μ⁽ᵏ⁾` but never updates `q`.

Expand the true constraint around the start-of-step position:

```
C(qⁿ + δq) = C(qⁿ) + J(qⁿ) δq + ½ δqᵀ ∇²C δq + O(‖δq‖³)
```

PGS works only at the velocity level, discarding everything beyond the first-order term. Since `‖δq‖ = O(h)` (velocity is bounded), the dropped quadratic term is `O(h²)`.

**More iterations refine the solution to a better answer to a wrong question** — the wrong question being the linear approximation of the constraint manifold at `qⁿ`. You converge faster to a point that itself drifts from the true nonlinear surface as `h` grows.

---

## The O(h²) argument for substepping (Macklin et al. 2019)

### Per-step linearization error

Each PGS sweep approximates one step of the nonlinear backward-Euler system:

```
M(v⁺ - vⁿ)/h = f_ext + Jᵀ λ
C(xⁿ + h v⁺) = 0
```

by linearizing: `C(xⁿ + h v⁺) ≈ C(xⁿ) + h J(xⁿ) v⁺`. The truncation error in position is:

```
e_lin = ½ h² ẍᵀ ∇²C ẍ + O(h³) = O(h²)
```

### Substep scaling

Split a frame interval `H` into `n` substeps of size `h = H/n`, one sweep each:

```
Linearization error per substep:  O(h²) = O(H²/n²)
Accumulated over n substeps:      n · O(H²/n²) = O(H²/n)
```

Total error decreases as `1/n`. Doubling the substep count halves the accumulated linearization error.

### Why N iterations at fixed h cannot achieve this

With `N` iterations at step size `h = H`, the linearization error is `O(H²)` — determined by the step size, not the iteration count. Iterations improve the solution to the **fixed** linearized system. Even solving the linearized system exactly (infinite iterations) gives error `O(H²)`, not `O(H²/N)`.

### Empirical validation

From Macklin's hanging-chain benchmark (XPBD, 100 total solver evaluations):

| Configuration | Max position error |
|---------------|-------------------|
| 1 step, 100 iterations | 322.1 m |
| 100 substeps, 1 iteration each | 3.2 m |

A factor of ~100 improvement, exactly consistent with `O(H²/n)` scaling.

---

## Contact Jacobian staleness

For rigid-body contacts, the Jacobian depends on body orientations through the lever arm `r = x_contact - x_com`:

```
J = [nᵀ,  (r × n)ᵀ,  -nᵀ,  -(r' × n)ᵀ]
```

During PGS iterations at step size `H`:
- Orientations `θ_A, θ_B` are frozen at `qⁿ`.
- Lever arms `r(θ)` do not update.
- Contact normals `n(q)` do not update.

Body orientations change by `δθ ~ ω · H` during the step. For a stack of `L` bodies, the accumulated lever-arm error at the bottom is `O(L · ω · H)`. More iterations do not correct this — they refine impulses consistent with stale geometry.

With `n` substeps of size `h = H/n`, contact anchors are re-evaluated at each substep's current orientation. The lever-arm error per substep is `O(ω · H/n)`. The Jacobian stays fresh relative to the state the solver is actually minimizing over.

---

## The Nyquist bound

A constraint oscillating at frequency `f` Hz is unrepresentable unless `h < 1/(2f)`. This is the sampling theorem — not a convergence issue. A contact spring at 120 Hz cannot be resolved in a 60 Hz simulation regardless of how many PGS iterations are used.

Adding iterations does not recover Nyquist-limited modes. Only reducing `h` does.

Substepping at `n` substeps raises the effective Nyquist limit from `1/(2H)` to `n/(2H)`. With 4 substeps per 60 Hz frame, the solver can represent constraint frequencies up to 120 Hz instead of 30 Hz.

---

## TGS vs. PGS: what the position update buys

The core architectural distinction (from the PhysX 5.4 documentation):

- **PGS**: Each iteration processes the same constraint list against body velocities from the start of the step. The iteration is a linear fixed-point method on a fixed matrix `A⁽⁰⁾`.
- **TGS**: Subdivides `H` into `n` substeps. After each substep, velocities and positions are **integrated** before the next substep begins. The constraint matrix `A⁽ᵏ⁾` is re-evaluated at `q⁽ᵏ⁾` after each integration.

TGS is therefore a **nonlinear** solver (quasi-Newton character) even though each substep solves a linear subproblem. Its convergence is not governed by the spectral radius of a fixed matrix but by how closely the successive linearization points track the constraint manifold. This is the mechanism behind PhysX TGS's "enhanced handling of high-mass ratios" and "superior joint drive accuracy" — conditions where the constraint manifold has high curvature.

---

## Soft constraints and timestep sensitivity

### Spring-damper formulation (Catto GDC 2011)

A constraint with spring frequency `f` (Hz) and damping ratio `ζ` corresponds to:

```
ω = 2πf,    k = mω²,    c = 2mζω
```

The PGS velocity constraint with softness becomes:

```
J v⁺ + (β/h) C + (γ/h) λ_acc = 0
```

where:

```
γ = 1 / (c + hk)           -- softness / compliance
β = hk / (c + hk)          -- Baumgarte factor (derived, not tuned)
```

The effective mass becomes `K̃ = K + γ/h`, and the impulse:

```
Δμ = -K̃⁻¹ (J v + (β/h) C + (γ/h) μ_acc)
```

### Behaviour at small h

As `h → 0`:
- `γ → 1/c` (finite, determined by damping)
- `β → 0` (position correction vanishes per step)
- The constraint appears **hard** and relies on integration across multiple substeps to enforce position-level accuracy

This is exactly the regime where substepping is most effective: each soft constraint is corrected incrementally, and the bias accumulates correctly over the full interval.

### Behaviour at large h

As `h` grows:
- `γ → 1/(hk) → 0` (constraint becomes infinitely stiff in one step)
- The impulse from previous steps is rapidly discarded
- The solver becomes numerically overdamped and loses frequency response above `f_Nyquist = 1/(2h)`

---

## Summary: three orthogonal benefits of substepping

| Mechanism | Why iterations can't substitute | Why substeps help |
|---|---|---|
| **Linearization truncation** | Iterations refine solution to an `O(H²)`-accurate problem; error is `O(H²)` regardless of `N` | Each substep has `O(h²)` error; total `O(H²/n)` — improves as `1/n` |
| **Jacobian staleness** | Geometry frozen at `qⁿ`; stale lever arms, normals, and inertias regardless of `N` | Jacobians re-evaluated at each substep; error `O(ω H/n)` |
| **Nyquist limit** | Sampling theorem is absolute; no amount of iteration recovers `f > 1/(2H)` | Substeps raise Nyquist limit to `n/(2H)`; higher-frequency constraint modes become representable |

---

## Practical cost structure

The expensive part of a physics frame is broadphase collision detection and narrowphase contact generation. If substepping runs collision detection **once per frame** (as PhysX TGS and bepuphysics2 do), the cost of `n` substeps over 1 substep is roughly `n×` the solver work only. Broadphase and narrowphase stay at `1×`.

For a system with 8 solver iterations at 1/60s:

| Configuration | Solver evaluations per frame | Linearization error | Jacobian freshness |
|---|---|---|---|
| 1 substep, 8 iterations | 8 | `O(H²)` = `O(1/3600)` | Stale for full 16.7 ms |
| 4 substeps, 2 iterations | 8 | `O(H²/4)` = `O(1/14400)` | Refreshed every 4.2 ms |
| 8 substeps, 1 iteration | 8 | `O(H²/8)` = `O(1/28800)` | Refreshed every 2.1 ms |

Same total solver work. The bottom row has 8× less linearization error, 8× fresher Jacobians, and 8× higher Nyquist limit.

---

## The equivalence claim (and its limits)

Macklin et al. show that `n` substeps with one sweep each is "equivalent" to one step with `n` sweeps **when constraints are linear and the system is uncoupled**. If `C(q) = Jq - c` is exactly linear (constant `J`), then a single PGS sweep on a substep of size `h` produces the same residual reduction as a full-step sweep, because the effective mass `K` is identical.

The non-equivalence emerges at the nonlinear level: when `J = J(q)`, iterating at fixed `q` is categorically different from relinearizing at updated `q`. Rigid body rotation is the canonical example — the angular velocity Jacobian `r × n` changes continuously with orientation, so a large-step solver is solving the wrong linear system regardless of how many times it iterates.

---

## Implications for RustDude

RustDude's current configuration:
- `fixed_dt = 1/60s` (16.7 ms substep)
- `solver_iterations = 8`
- `max_substeps_per_frame = 4` (for frame accumulator catchup, not solver quality)
- Position correction: split-impulse post-stabilization with Baumgarte fallback
- Contacts computed once per frame via `update_contacts()`, reused across substeps

The architecture already supports `update_contacts()` once, then `substep()` N times. To exploit substepping for solver quality:

1. Reduce `fixed_dt` to `1/240s` (4 substeps per 60 Hz frame).
2. Reduce `solver_iterations` to 2.
3. Set `max_substeps_per_frame` to 8 (allows catchup when rendering lags).
4. Total solver evaluations per frame: 4 × 2 = 8 (unchanged from current 1 × 8).

This should reduce inter-manifold jitter in stacks without any new code — only config changes. The contact anchors will be 4× fresher, the linearization error 4× smaller, and the Nyquist limit 4× higher.

If further stability is needed, the next step is soft constraints (replacing the Baumgarte bias with a spring-damper formulation per Catto 2011), which is a separate project.

Shock propagation remains a valid fallback if substepping alone is insufficient or if the narrowphase cost makes frequent substeps prohibitive. The two techniques are complementary — shock propagation addresses inter-manifold coupling within a single substep, while substepping improves the accuracy of each substep's linearization.

---

## References

- Guendelman, E. et al. (2003). *Nonconvex Rigid Bodies with Stacking.* SIGGRAPH 2003. [PDF](https://graphics.stanford.edu/papers/rigid_bodies-sig03/rigid_bodies.pdf)
- Erleben, K. (2007). *Velocity-Based Shock Propagation for Multibody Dynamics Animation.* [ResearchGate](https://www.researchgate.net/publication/220184619_Velocity-based_shock_propagation_for_multibody_dynamics_animation)
- Macklin, M. et al. (2019). *Small Steps in Physics Simulation.* SCA 2019. [PDF](https://mmacklin.com/smallsteps.pdf)
- Catto, E. (2011). *Soft Constraints: Reinventing the Spring.* GDC 2011. [PDF](https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf)
- Catto, E. (2024). *Solver2D.* [Blog post](https://box2d.org/posts/2024/02/solver2d/)
- NVIDIA. *PhysX 5.4 — Rigid Body Dynamics.* [Docs](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html)
- Nordby, R. / bepuphysics2. *Substepping.* [GitHub](https://github.com/bepu/bepuphysics2/blob/master/Documentation/Substepping.md)
- Macklin, M. & Müller, M. *XPBD: Position-Based Simulation of Compliant Constrained Dynamics.* [PDF](https://matthias-research.github.io/pages/publications/XPBD.pdf)
