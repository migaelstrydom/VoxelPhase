# Traction Drive — Design

Design for the mechanism that replaces `VelocityDriven`. The requirements it
must satisfy are R1–R12 in
[`VELOCITY_DRIVE_AND_PLATFORMS.md`](VELOCITY_DRIVE_AND_PLATFORMS.md) §7; that
document also records why the present mechanism behaves as it does. This one
does not repeat the diagnosis. It answers a single question:

> Where does a character's momentum come from, and what is on the other end of
> it?

**The design in one sentence:** a drive is friction with a non-zero target.

Everything below follows from that. The engine already solves, per contact, a
tangential impulse that drives relative tangential velocity to zero under a
`μ·N` bound. A walking character wants the same impulse at the same point under
the same bound, driving relative tangential velocity to *five metres per second*
instead of zero. There is no second mechanism to build — there is one parameter
to unhard-code, and a command channel to feed it.

That thesis is about the *form* of the row. It says nothing about the
*magnitudes* the row produces against this project's tuning, and those decide
how the change feels rather than whether it is correct. §10 and §11 carry that
half. Both of the decisions there are settled — the character is a cartoon by
measured decision, and grounding has one source with a named allowance for what
that still strands. §11 records each with the alternatives it was chosen over,
because both are questions about how the game should feel rather than about
whether the mechanism is correct.

---

## 1. Terms

Every term coined by this design, defined before use. Terms already in the
codebase are marked *(existing)*.

| Term | Definition |
|---|---|
| **Drive Intent** | The command channel. What gameplay *wants*: a target velocity expressed relative to a reaction anchor, plus a target angular velocity. Write-only from gameplay's side, never read back. |
| **Body Motion** | The measurement channel. What the body *did*: linear and angular velocity as measured by the engine after solving. Read-only from gameplay's side. Replaces the read-modify-write role of today's `Velocity` *(existing)*. |
| **Actuator** | The per-entity declaration of *how* a body converts intent into momentum: which reaction anchor it uses, how much authority it has, how it grips a surface it is not standing on, and which allowances it is granted. A property of the entity, not a branch in the engine. |
| **Reaction Anchor** | The declared recipient of every equal-and-opposite impulse a drive applies. Exactly two kinds exist: **Support Anchor** and **Medium Anchor**. |
| **Support Anchor** | Reaction goes into the bodies the driven body is resting on, at the contact points. A character. Produces the correct torque on a platform by construction. |
| **Medium Anchor** | Reaction goes into the world — an infinite reservoir. A thruster, a rotor, a magnetically levitated lift. Honest but explicitly declared: the entity states that it burns an inexhaustible fuel. |
| **Support Set** | Per body, per frame: the supporting contacts, each with its contact point, normal and partner body. Derived from the manifolds the narrowphase already produced. Generalises today's `GroundingDetector` *(existing)*. Carries no weights — see §6.1 on why load distribution needs none. |
| **Contact Frame** | The orthonormal basis at a contact: the normal plus two tangents. The only basis any drive code is permitted to use. |
| **Target Relative Velocity** | The tangential relative velocity the drive asks for at one contact point, computed from the Drive Intent and the driven body's rigid-body kinematics. Zero is ordinary friction. |
| **Tangential Row** | The unified contact row that solves toward a Target Relative Velocity under the traction budget. Today's friction solve is the special case where the target is zero. |
| **Torsional Row** | The same idea about the contact normal: drives relative spin toward a target under a torsional budget. This is how a character turns. |
| **Traction Budget** | The impulse bound on a contact's tangential row, `μ_drive·N`. One budget, shared: a body spending it on drive has none left for grip. Its magnitude is set by the Drive Gain — §11. |
| **Drive Gain** | The per-actuator factor separating the drive's `μ_drive` from the contact's grip `μ`. Default 1.0; **the player is 5.0** (§11). It is not a coefficient of friction and does not model one — no friction formulation supplies this factor, and §11 works through why. It is the dial for how cartoon a character's acceleration is, and it is the design's second sanctioned cheat alongside **Allowance**: deliberate, named, confined to one scalar, and instrumented (§11). |
| **Non-Support Grip** | The per-actuator factor scaling this body's own tangential budget at contacts that are **not** in its Support Set. Default 1.0. The player sets it near zero so jumps along vertical surfaces are not grabbed. Successor to `FrictionModel::AxisBiased` *(existing)* — §6.7. |
| **Normal Impulse Command** | The command-channel entry for a jump: an impulse along the support normal, delivered through the Support Set so the reaction lands on whatever is being jumped off. §6.3. |
| **Allowance** | A named, bounded, opt-in, non-conservative authority. Lives in exactly one module with an explicit per-entity budget. The design's only sanctioned cheat (R8). Its scope is much larger than "edge case": an airborne body has *zero* drive authority, ground yaw needs one (§6.2), and an unsupported jump needs one (§6.3) — so allowances cover the whole jump arc, most of turning, and one of the two ways a jump starts. Two shapes are needed, not one: an **impulse** budget, and a **projection** budget that scales or clamps velocity along an axis (jump cutoff is `v *= factor`, which no fixed impulse can express — §6.3). |
| **Effort** | *Deferred (see requirements §8).* A named authority that biases the drive to compensate for slope. Not designed here; the architecture leaves a seam for it. |

---

## 2. Existing architecture — engine level

What runs today, per frame.

```mermaid
flowchart TB
    subgraph GP["Gameplay systems — once per frame"]
        Intent["CharacterIntent"]
        Char["CharacterControlSystem<br/>read-modify-write x,z"]
        Plat["MovingPlatformSystem<br/>overwrite x,y,z"]
        Vel["Velocity<br/><b>command AND measurement</b>"]
    end

    subgraph Sync["PhysicsSyncSystem"]
        Down["sync_velocity_driven_from_ecs"]
        Up["sync_physics_to_ecs"]
    end

    subgraph Frame["PhysicsWorld — once per frame"]
        NP["narrowphase + manifold cache"]
        Cond["ManifoldConditioner<br/>shock propagation"]
        Prep["solver.prepare<br/>expand_constraints"]
    end

    subgraph Sub["PhysicsWorld::substep — N times"]
        IF["integrate_forces<br/><b>drive applied here,<br/>no reaction partner</b>"]
        Shift["drive.target += external delta<br/><i>gravity absorbed, contacts not</i>"]
        Solve["PgsNgsSolver::solve<br/>normal, friction, joint rows"]
        Int["integrate_bodies + CCD"]
    end

    Intent --> Char --> Vel
    Plat --> Vel
    Vel --> Down --> IF
    NP --> Cond --> Prep --> IF
    IF --> Shift --> Solve --> Int
    Int --> Up
    Up -->|"last frame's measurement<br/>becomes this frame's command"| Vel

    style Vel fill:#4a5568,color:#fff
    style IF fill:#822727,color:#fff
    style Shift fill:#744210,color:#fff
```

Two structural faults, both visible in the diagram:

1. **The feedback edge** `Up ──► Vel`. There is one channel for two jobs, so a
   gameplay write is an edit to a measurement, and correctness depends on which
   axes each writer leaves alone (R5).
2. **`IF` has no outgoing momentum edge.** The drive is applied inside force
   integration, upstream of the solver, to one body. Nothing receives the
   reaction (R1), nothing bounds it (R7), and the solver's contact impulses land
   afterwards where the target shift can no longer see them.

## 3. Proposed architecture — engine level

```mermaid
flowchart TB
    subgraph GP["Gameplay systems — once per frame"]
        Intent["CharacterIntent"]
        Char["CharacterControlSystem"]
        Plat["MovingPlatformSystem"]
        DI["DriveIntent<br/><b>command only</b>"]
        Act["Actuator<br/>anchor + authority + allowances"]
        BM["BodyMotion<br/><b>measurement only</b>"]
    end

    subgraph Sync["PhysicsSyncSystem"]
        Down["push_drive_intent"]
        Up["pull_body_motion"]
    end

    subgraph Frame["PhysicsWorld — once per frame"]
        NP["narrowphase + manifold cache"]
        Cond["ManifoldConditioner"]
        Prep["solver.prepare<br/>expand_constraints<br/><i>+ medium rows</i>"]
    end

    subgraph Sub["PhysicsWorld::substep — N times"]
        IF["integrate_forces<br/><i>gravity, forces, drag — no drive</i>"]
        Alw["AllowanceApplier<br/><b>the only cheat, budgeted</b>"]
        SR["SupportResolver<br/>manifolds ──► SupportSet"]
        TP["TractionPlanner<br/>intent + SupportSet<br/>──► per-contact targets"]
        Solve["PgsNgsSolver::solve<br/>normal ──► tangential ──► torsional ──► joint rows"]
        Int["integrate_bodies + CCD"]
    end

    Intent --> Char --> DI
    Plat --> DI
    Act --> Down
    DI --> Down --> TP
    BM -.->|"read-only, gameplay decisions"| Char
    NP --> Cond --> Prep --> Solve
    NP --> SR --> TP --> Solve
    IF --> Alw --> Solve --> Int
    Int --> Up --> BM

    style DI fill:#2c5282,color:#fff
    style BM fill:#4a5568,color:#fff
    style Solve fill:#22543d,color:#fff
    style Alw fill:#744210,color:#fff
```

What changed:

- **The feedback edge is gone.** `DriveIntent` flows down, `BodyMotion` flows
  up, and the dotted edge back into `CharacterControlSystem` carries
  *decisions* (am I falling? how fast am I going?), never a value that is edited
  and returned. R5.
- **The drive moved from `integrate_forces` into the solver.** It is now an
  impulse exchange between two bodies at a contact point, subject to the same
  bounds and the same iteration as every other contact impulse. R1, R2, R7, R9,
  R10 are properties of *where the code lives*, not of what it computes.
- **`integrate_forces` no longer knows what a drive is.** The whole
  target-shift dance at `body.rs:451–460` exists to stop a pre-solve drive from
  fighting gravity. A drive solved alongside gravity's velocity does not need
  it. Deleted.
- **One box is coloured as a cheat.** `AllowanceApplier` is the single place
  non-conservative authority may be applied, and it is empty unless an entity's
  `Actuator` declares a budget. R8.

---

## 4. Existing architecture — component level

```mermaid
classDiagram
    class Velocity {
        +Vector3 0
    }
    class VelocityDriven {
        +f32 max_accel
        +Vector3 angular_velocity
        +f32 angular_max_accel
    }
    class CharacterControlSystem {
        +run()
        -apply_movement_rule()
    }
    class MovingPlatformSystem {
        +run()
    }
    class PhysicsSyncSystem {
        -sync_velocity_driven_from_ecs()
        -sync_physics_to_ecs()
    }
    class RigidBody {
        -Option~VelocityDrive~ velocity_drive
        -Option~VelocityDrive~ angular_velocity_drive
        +integrate_forces(dt, gravity)
    }
    class VelocityDrive {
        +Vector3 target
        +f32 max_accel
    }
    class PgsNgsSolver {
        +prepare()
        +solve()
        -warmStartRuleForDrivenBodies
        -restitutionSuppressionForDrivenBodies
    }
    class TangentSolveFriction {
        <<solver::friction>>
        +solve_friction_impulse()
        -targetIsImplicitlyZero
    }
    class GroundingDetector {
        +grounded_bodies() bool
        -testsNormalY
    }
    class BuoyancyBody {
        +bool is_velocity_driven
    }

    CharacterControlSystem --> Velocity : read-modify-write x,z
    MovingPlatformSystem --> Velocity : overwrite xyz
    PhysicsSyncSystem --> Velocity : writes measurement
    PhysicsSyncSystem --> VelocityDriven : reads accel limits
    PhysicsSyncSystem --> RigidBody : set_body_velocity_drive
    RigidBody *-- VelocityDrive
    RigidBody --> PgsNgsSolver : solved as a body
    PgsNgsSolver --> TangentSolveFriction
    PgsNgsSolver ..> VelocityDrive : special-cased
    GroundingDetector ..> CharacterControlSystem : Grounding
    BuoyancyBody ..> VelocityDrive : special-cased
```

Four places in `src/physics/` know a body is driven (`RigidBody::velocity_drive`,
the target shift, the warm-start persistence rule, the restitution suppression),
plus `integrate_forces` itself, plus the leak into `water/coupling.rs`. That is
the surface R11 caps.

## 5. Proposed architecture — component level

```mermaid
classDiagram
    direction TB

    class DriveIntent {
        <<ECS component — command>>
        +Vector3 linear_target
        +Vector3 angular_target
        +Option~f32~ normal_impulse
    }
    class Actuator {
        <<ECS component — declaration>>
        +ReactionAnchor anchor
        +f32 max_accel
        +f32 angular_max_accel
        +f32 drive_gain
        +f32 non_support_grip
        +Option~Allowance~ allowance
    }
    class BodyMotion {
        <<ECS component — measurement>>
        +Vector3 linear
        +Vector3 angular
    }
    class Allowance {
        +f32 air_accel_budget
        +f32 air_yaw_budget
    }

    class DriveCommand {
        <<physics::drive::command>>
        +Vector3 linear_target
        +Vector3 angular_target
        +ReactionAnchor anchor
        +DriveAuthority authority
    }
    class ReactionAnchor {
        <<enum>>
        Support
        Medium
    }
    class SupportResolver {
        <<physics::drive::support>>
        +resolve(manifolds, gravity_dir) SupportSet
    }
    class SupportSet {
        +SmallVec~SupportContact~ contacts
        +Vector3 mean_normal
        +is_empty() bool
    }
    class SupportContact {
        +Point3 point
        +Vector3 normal
        +Option~RigidBodyHandle~ partner
    }
    class TractionPlanner {
        <<physics::drive::plan>>
        +plan(command, SupportSet, body) TangentTargets
    }
    class TangentTarget {
        +Vector3 target_rel_velocity
        +f32 target_spin
    }
    class TangentialSolver {
        <<solver::tangential>>
        +solve(contact, target, budget)
    }
    class TorsionalSolver {
        <<solver::torsional>>
        +solve(contact, target_spin, budget)
    }
    class MediumRowBuilder {
        <<physics::drive::medium>>
        +expand(command, body) SmallVec~ConstraintRow~
    }
    class AllowanceApplier {
        <<physics::drive::allowance>>
        +apply(body, command, SupportSet, dt)
    }
    class PgsNgsSolver
    class ConstraintRow
    class PhysicsConfig {
        +gravity_direction()
    }

    note for TangentialSolver "Friction is the special case target = 0"
    note for MediumRowBuilder "World-anchored rows: 3 linear, plus 3 angular if commanded"

    DriveIntent --> DriveCommand : PhysicsSyncSystem
    Actuator --> DriveCommand : PhysicsSyncSystem
    Actuator *-- Allowance
    DriveCommand *-- ReactionAnchor
    BodyMotion <-- PgsNgsSolver : measured

    SupportResolver --> SupportSet
    PhysicsConfig --> SupportResolver : gravity_direction
    SupportSet *-- SupportContact
    SupportSet --> TractionPlanner
    DriveCommand --> TractionPlanner
    TractionPlanner --> TangentTarget
    TangentTarget --> TangentialSolver
    TangentTarget --> TorsionalSolver
    PgsNgsSolver --> TangentialSolver
    PgsNgsSolver --> TorsionalSolver
    DriveCommand --> MediumRowBuilder
    MediumRowBuilder --> ConstraintRow
    ConstraintRow --> PgsNgsSolver
    DriveCommand --> AllowanceApplier
    SupportSet --> AllowanceApplier
```

---

## 6. How it works

### 6.1 The tangential row

The whole design turns on one generalisation. Today, `solve_friction_impulse`
computes, for each contact:

```
Δλ = −(v_rel · t) / m_eff        bounded by  |λ| ≤ μ·N
```

The `−(v_rel · t)` is a target of zero, hard-coded by omission. The tangential
row makes it explicit:

```
Δλ = ((v_target − v_rel) · t) / m_eff     bounded by  |λ| ≤ μ·N
```

`v_target` is the Target Relative Velocity — what the drive wants the *relative*
motion at this contact to be. Friction is `v_target = 0`, unchanged in
behaviour and unchanged in cost.

The consequences are not incremental; they are the requirements:

| Because the row is… | We get |
|---|---|
| an impulse applied through `apply_impulse_pair` at the contact point | **R1** conservation, with the correct torque arm — walking `+X` at the `+Z` edge torques the platform `−Y`, because that is where the impulse lands |
| applied to a partner whose `inv_mass` is 0 when static or kinematic | **R2** silent absorption *as a mechanism*, with no code path of its own. R2's further claim that the player then accelerates "exactly as today" is not delivered by the row and is not achievable as worded; the magnitude is restored deliberately by the Drive Gain instead — §11 |
| expressed in **relative** velocity, per contact | **R4** support-relative targets — but *per contact*, not per body. See below: a body bridging two supports has no single anchor frame, and its world speed becomes an emergent outcome rather than a commanded one |
| bounded by `μ_drive·N` | **R7** authority bounded by the contact. Ice has low `μ`; mass ratio governs a crate push because the solver already computes it. **The absolute magnitude is the project's largest open risk — §10.1** |
| solved per contact, in the **Contact Frame** | **R6** for the drive's *basis* and, once §6.7 lands, for its *bound* too: the player's `μ` stops coming from `FrictionModel::AxisBiased` and its body-local up axis. No `x`, no `z`, no `Vector3::y()` anywhere in the drive path |
| one of several contacts, each independently bounded by its own `N` | **R9** distribution over supports, with no weighting term of any kind — see below |

#### The per-contact target must carry the commanded spin

With no weighting term, each support contact independently drives *contact-point*
relative velocity to its target. Contact-point velocity is `v + ω × r`, so two
contacts handed the same target vector do not merely constrain `v` — together
they pin `ω` about every axis perpendicular to their separation, at up to `μ·N`
each. The target is therefore not a vector per body but a vector per contact:

```
v_target(contact) = v_target_linear + ω_target × r_contact
```

where `r_contact` is the contact point relative to the driven body's centre of
mass. This is the well-posedness condition for a per-contact drive, and it is
what §1's definition of Target Relative Velocity means by "computed from the
Drive Intent and the driven body's rigid-body kinematics". A body-level target
vector, applied unchanged at every contact, is not a weaker version of this — it
is a different and incorrect constraint.

Getting it wrong has a specific and misleading failure mode. §6.2 concludes that
ground yaw comes from an Allowance, and an Allowance is applied *before* the
solve. A turning body gives its support contacts different tangential
velocities, so tangential rows built from a body-level target would spend their
budget undoing the yaw the Allowance had just produced. The character would turn
freely in the air and refuse to turn on the ground — which is precisely what
§6.2 predicts a weak torsional row would look like, so it would be diagnosed
there and the real cause would not be found. `TractionPlanner` owns this
equation; it is not an optimisation.

#### R9 needs no weighting term

`SupportContact` carries no share, weight or fraction, and R9 is satisfied
anyway. The mechanism is the per-contact bound: a row whose contact carries
little `N` saturates at its own `μ·N` and stops contributing, while the loaded
contact keeps pushing. Load distribution is what PGS already does.

Weighting is tempting and both forms of it are wrong. Scaling the *target* by a
share is plainly incorrect — two supports at 0.5 each would drive relative
velocity toward 2.5 m/s at both contacts, and the character would converge on
half the commanded speed. Scaling the *bound* double-counts the load, because
the bound is already proportional to `N`.

Carrying no weights also keeps the planner acyclic. A weight proportional to
accumulated normal impulse could only be computed *before* the solve, from last
frame's warm-start impulses — undefined on the substep a foot first lands, and a
division by zero in the normalisation. With nothing to weight, `TractionPlanner`
needs only the contact set and the target, and the bound is read live inside the
solve loop from `contact.accumulated_normal_impulse`, exactly as
`solve_friction_impulse` does today.

#### R4 is per-contact, and that is the honest reading

R4 asks for targets in the frame of the reaction partner. R9 asks for a body
bridging two supports. Together they describe a character with one foot on a
moving platform and one on terrain — where "5 m/s relative to the anchor" names
two different world velocities, and the two rows are mutually unsatisfiable.

The solver's answer is to split the difference according to each contact's
bound, which is defensible physics and matches what a real person straddling a
travelator experiences. But it means R4 is satisfied **per contact**, not per
body: there is no single support frame, and the character's world-space speed is
an *outcome* of the solve rather than a quantity gameplay commands.

The design accepts that reading, and one consequence follows for the ECS
surface: `BodyMotion` carries no `carrier_velocity` field. A single `Option<Vector3>`
over what is structurally a set would be the obvious thing for
`CharacterControlSystem` to use to reconstitute a world-space target — which is
the old read-modify-write loop in new clothing, and the exact place R5 would
erode first.

### 6.2 The torsional row

Turning is the same mechanism about the normal instead of within the tangent
plane: drive relative spin about the contact normal toward the intent's angular
target, bounded by torsional friction `μ_t · N · r`, where `r` is the contact
patch radius. As a *form*, R10 holds — one mechanism, two projections of it, no
second pump to build.

**As a magnitude, ground yaw through this row is zero.** Three separate facts
each suffice on their own:

- **There is no `r` to read.** `SolverContact` carries a `point`, not a patch
  (`pipeline/pair.rs:51`). A torsional bound of `μ_t · N · r` has no third term
  in this engine, and inventing one is a new geometric concept, not a
  parameter.
- **A capsule on terrain contacts at a point.** Even given an `r`, the true
  value is ~0.
- **The player's yaw inertia is deliberately 50×.** `spawners/player.rs:54` does
  `scale_local_inertia(Vector3::new(1.0, 50.0, 1.0))`, with `angular_damping(0.95)`
  on top. Whatever torsional impulse survives the first two points is then
  divided by fifty and damped.

The conclusion runs opposite to what the mechanism suggests: **the yaw allowance
is how the player turns on the ground, not a fallback for when the torsional row
disappoints.** The torsional row is worth building because it
is nearly free once the tangential row is generalised and because it is correct
for wide-contact bodies — a turntable, a tracked vehicle, a body lying prone —
but it will not turn the player.

R10 is therefore true in its anti-goal (there is no second reactionless pump)
and false in its literal reading (one mechanism does not cover both channels for
the player). The traceability table records it that way.

### 6.3 The impulse channel — jumping

A tangential row is tangential by construction, so it cannot express a jump. The
command channel therefore has two entries, not one:

```rust
pub struct DriveIntent {
    /// Continuous: solved as tangential and torsional rows each substep.
    pub linear_target: Vector3<f32>,
    pub angular_target: Vector3<f32>,
    /// Discrete: consumed once, applied along the support normal.
    pub normal_impulse: Option<f32>,
}
```

A **Normal Impulse Command** is delivered through the Support Set like any other
drive impulse: split across the supporting contacts, applied at the contact
points, with the equal and opposite impulse landing on each partner. Jumping off
a platform pushes the platform down, and jumping off the edge of one tips it —
both correct, both free, and both consequences of using the same delivery path
as traction rather than a second one.

This closes a hole that would otherwise have stopped Stage 1 dead. Today's jump
verbs reach directly into the shared `Velocity` on the `y` axis:

```rust
vel.0.y *= config.jump_cutoff_factor;   // character_control.rs:127 and :132
```

That is a read-modify-write on a measurement (what R5 outlaws) along a
privileged axis (what R6 forbids), and it is the second-most-used verb in the
game. Three variants need homes:

| Verb | Today | Under this design |
|---|---|---|
| Supported jump | sets `vel.0.y` | `normal_impulse`, delivered through the Support Set |
| **Coyote jump** | sets `vel.0.y` (`components.rs:208-214`) | **Allowance** — but not as its own case. `CoyoteTime` is by definition the state where support has just been lost, so the Support Set is empty and there is nothing to push off; D2a covers it under the general rule that any jump without support is an Allowance |
| Jump cutoff on early release | `vel.0.y *= factor` | airborne, so no support: an Allowance, and a **projection** one — the cut is proportional to current velocity, so the same 50% is a different impulse at every point on the arc. A fixed-impulse budget would under-cut a fast jump and reverse a slow one |
| Coyote-time `clamp_up` | zeroes positive `vel.0.y` | same: a projection Allowance, since by definition there is no support left |

The coyote jump deserves emphasis, because it makes the allowance budget carry
**the single largest non-conservative impulse in the game** — a full
`jump_speed: 7.0` — rather than the air-steering nudges the word suggests. Two
jumps that feel identical to the player are then conservative in one case and
not the other: an `edge walk` and a coyote hop off the same platform torque it
differently. That is permitted by R8 and it is exactly what R8 means by *named*,
so it is written here rather than found later.

#### Consumption protocol

An edge-triggered command entering a world that runs N substeps needs three
rules, and all three bite in Stage 1:

**Which substep fires it.** The first substep of the frame, once. Firing every
substep would make a jump N× too strong, and N varies with frame time, so the
bug would present as jump height depending on framerate.

**Who clears it.** `PhysicsSyncSystem` takes the command at the frame boundary
as it pushes — `Option::take`, on the gameplay side of the seam. The physics
world never writes to an ECS component, so `DriveIntent` stays write-only from
gameplay's side and R5's shape is preserved in both directions. This is the
reason the field is `Option<f32>` rather than a flag with a separate reset.

**What happens if it cannot be delivered.** It becomes an Allowance. A jump
commanded with an empty Support Set is delivered non-conservatively out of the
actuator's budget rather than through the contacts — **D2a** (§11), stated as a
general rule and not as a `CoyoteTime` special case.

The constraint that forces this: gameplay cannot re-assert an undelivered
command, because the FSM clears the jump buffer in the same frame it commits
(`character_control.rs:122-124`), before physics has tried to deliver anything.
An undeliverable jump would otherwise be swallowed outright rather than retried.
Nor can consumption be split between the two sides — under `take` the ECS-side
option is already `None`, so "physics did not consume it" could only mean
"physics did nothing with it". With the Allowance fallback the question dissolves:
delivery always succeeds by one route or the other, so `take` is unconditional
and the buffer clear is harmless.

Note what the second and third rows mean. `solve_friction_impulse` early-returns
when `accumulated_normal_impulse <= 0.0` (`friction.rs:21`), and traction
inherits that correctly — no load, no drive. So an airborne body has **zero**
drive authority of any kind, and every scrap of air control, jump shaping and
mid-air yaw is Allowance. R8's budget is not a garnish on this design; it is the
whole airborne half of the character controller, and it should be scoped that
way from the start.

### 6.4 The medium anchor

A platform's motor is not a foot. It pushes against nothing and is entitled to,
provided it says so. `ReactionAnchor::Medium` expands to ordinary
`ConstraintRow`s with `body_a: None` — the world-anchored form the constraint
system already supports for `world_fixed`, `world_hinge` and friends — with
`bias` set to the target velocity and `bounds` set from the actuator's
authority.

A `ConstraintRow` is **one degree of freedom** (`bias: f32`,
`accumulated_impulse: f32`, and `row_index` documented as "which row within the
parent constraint" — `constraint/types.rs:352`), so a Medium anchor is **three
rows** for a 3-vector `linear_target`, plus up to three more when the angular
target is non-zero. `MovingPlatform` sets angular to zero today
(`moving_platform.rs:146`), but `Actuator` declares an `angular_max_accel`, so
the builder must handle the angular case rather than assume it away. Every other
constraint in the system already expands to a row set; this one is no
exception.

The lift therefore does not recoil when it carries a passenger, and the
passenger still recoils onto the lift, exactly as R3 requires. Both are honest;
the difference is one enum variant on the entity, not a second code path in the
engine.

### 6.5 Ordering inside the substep

The traction budget is `μ·N`, and `N` is not known until the normal rows are
solved. This is not a new problem — friction has always had it, and the solver
already resolves it by solving normals first within each iteration and reading
`contact.accumulated_normal_impulse`. Traction inherits that ordering
unchanged:

```mermaid
sequenceDiagram
    autonumber
    participant W as PhysicsWorld::substep
    participant SR as SupportResolver
    participant TP as TractionPlanner
    participant S as PgsNgsSolver
    participant B as bodies

    W->>B: integrate_forces — gravity, forces, drag
    W->>SR: resolve(cached manifolds, gravity_dir)
    SR-->>TP: SupportSet per driven body
    W->>TP: plan(commands, support sets)
    TP-->>S: TangentTarget per contact
    loop solver iterations
        S->>B: normal rows — produces N
        S->>B: tangential rows — target, bound μ·N
        S->>B: torsional rows — target spin, torsional bound
        S->>B: joint + medium rows
    end
    W->>B: project_velocities, integrate_bodies, CCD
```

Nothing in this sequence is new machinery. Steps 2–4 are the addition; the loop
gains two row kinds and loses nothing.

### 6.6 The support set replaces grounding

`GroundingDetector::grounded_bodies` today tests `normal.y >= 0.5`. The
`SupportResolver` computes the same answer from the same manifolds against
`PhysicsConfig::gravity_direction()` — the normalise-with-fallback currently
open-coded at `world.rs:688`, lifted to a method as the requirements ask — and
returns the contacts themselves rather than a boolean.

**`Grounding` becomes a projection of the Support Set for every body, the player
included.** Today it is not: `ContactGroundingSystem` skips rigged humanoids
(`character/grounding_system.rs:17-21`, `!&animators` at :53), and for them
`CharacterAnimationSystem` writes `Grounding` from the foot probes
(`animation/systems.rs:124`). This design brings the player onto the Support Set
with everything else, for a reason that is not about which sensor is better.

#### Why the probe path cannot stay

The probes are not an independent sensor. Their aim is chosen by the animation
they feed:

```
foot_placer.probe_anchor()      // the placer's committed landing target
  → configure_probes()          // animator.rs:116-126, aim = that anchor
  → SensorProbeSystem raycast
  → animator.state.is_grounded  // animator.rs:185, "did either probe hit"
  → Grounding                   // animation/systems.rs:124
  → CharacterState FSM          // next frame, CharacterControlSystem
  → PoseState::Airborne
  → foot_placer.set_suspended() // animator.rs:325
  → probe_anchor()
```

`configure_probes` additionally branches on `is_suspended()`, aiming at the
visible foot rather than the landing target, so the placer's own state steers
where the sensor looks. This is a closed loop, and it does not stay inside
animation: it leaves through `Grounding` into the FSM, and from there into the
jump verbs and the drive. **A foot placement decision changes what the physics
does.**

That inverts the rule the rest of the engine keeps — animation responds to
physics, and nothing animation decides feeds back into it. The traction drive
makes the inversion worse rather than tolerable, because after Stage 5 the drive
is an impulse exchange at real contacts, and it would be gated by a sensor aimed
by the gait.

The failure it produces today is visible in the sentence that justifies it —
probes "resolve ledges and steps far better than a capsule's contact set does",
because they see the ledge the sole is over. Standing near a ledge with one foot
placed out past the edge, the probe misses and the character goes **airborne
while resting on solid ground**. The capsule is supported; the manifolds say so;
the state machine disagrees because of where the gait put a foot.

Under the Support Set the character stays grounded, which is both what the
contacts say and what is true. The ledge problem does not disappear — it moves to
where it belongs, as a foot-placer rule: a landing target whose probe finds no
ground within reach is not a legal target, and the step shortens. That is a
decision about where to put a foot, made from a probe, with no path into physics,
and it is testable offline in the existing `foot_placer/sim.rs` harness.

#### What the probes are, once they stop writing `Grounding`

They remain, and animation still needs them: the foot placer requires terrain
height and normal at a landing target that nothing is yet touching, and no
manifold can supply that. What changes is that the probe becomes a **rangefinder
only**. `CharacterAnimator` should take `is_grounded` from the `Grounding`
component rather than computing its own at `animator.rs:185`, leaving one source
of truth for support and one for height.

This is also the honest reading of what the probe answer meant. `ContactCandidate`
carries a `distance`, and `is_grounded` is "either probe hit within
`probe_length`" — so it answers *is there ground within reach of where I am about
to step*, not *am I touching something*. The forgiveness in that difference is
real and worth keeping, but it is currently an unmeasured side effect of a probe
length. Moving grounding to contacts converts it into an explicit, tunable
window in the FSM — see §9's Stage 2c for the chatter this must absorb.

#### What survives: staleness, not disagreement

`CharacterControlSystem` runs before `PhysicsSyncSystem` while the grounding
writer runs after it (`dispatcher_builder.rs:46,56,82,91`), so the FSM reads a
one-frame-old answer whichever sensor supplies it. It can still commit a jump in
a frame where the Support Set has since emptied.

So unifying the sensors removes the *structural* disagreement — the deliberate
lead of one sensor over another — and leaves ordinary staleness. That residue is
caught by an Allowance (D2a, §11): a jump with no Support Set is delivered
non-conservatively rather than dropped. It is a much smaller residue than two
sensors would leave, and it is budgeted rather than incidental.

R6 holds for the drive, and for grounding, which now has one contact-derived
source for every body.

**Support-relative speed is another projection of the same thing, and naming it
here is what keeps `carrier_velocity` dead.** §6.1 removed that field from
`BodyMotion` because a single `Option<Vector3>` over a structural set is where
R5 would erode first. But something downstream genuinely wants the quantity it
promised: animation consumes world velocity, so a walker holding station on a
platform running at 4 m/s reads as moving at 4 m/s. The visible symptom today is
head tilt — `compute_head_tilt` takes `velocity.magnitude()`
(`animation/humanoid/stride_sync.rs:105`) — and any gait or stride consumer of
world velocity has the same exposure.

The successor is `SupportSet::relative_speed(body)`: the driven body's velocity
measured against its supports, weighted by normal impulse for exactly the
averaging job the drive path refuses to do. It is a query on the Support Set,
computed where the contacts already are, and it is not a component. Naming it
now matters, because Stage 1 will otherwise grow `carrier_velocity` back the
first time an animator notices the sprinting passenger.

### 6.7 Non-support grip replaces `FrictionModel::AxisBiased`

The Support Set has a second consumer, and it retires the one piece of existing
machinery that stands between the drive and R6.

`FrictionModel::AxisBiased` (`collider.rs:107`) gives the player `floor: 0.8`
and `wall: 0.0`, interpolated by a contact normal's alignment with a body-local
up axis. The comment above it is candid about what it is for
(`spawners/player.rs:33-38`): floor friction keeps the player planted, and wall
friction is near zero "so jumps along vertical surfaces don't get grabbed". Four
things are wrong with where it lives, and only the first is about this design:

- **It reads a body-local axis.** `local_up: Vector3::y()`
  (`spawners/player.rs:45`). This is the sole reason R6's *bound* could not be
  discharged along with its basis.
- **It has a constraint as a hidden precondition.** Its own comment says the
  classification only works because `KeepUpright` holds body-Y aligned with
  world up. A ragdolled corpse — `DeathSystem` releases that constraint — keeps
  a friction model that assumes it is standing.
- **It is a character-controller tuning device inside the collider material
  system**, so it feeds the geometric-mean combine at `collider.rs:188` and
  silently alters friction in *both* directions. The crate the player leans on
  inherits the player's jump tuning.
- **It closes the extension point §7 advertises.** A wall anchor transmits
  `μ = 0`, so magnetic boots are dead on arrival however the drive is bounded.

What `AxisBiased` actually expresses is *floor versus not-floor* — which is the
identical classification `SupportResolver` performs one step earlier in the
substep, except against `PhysicsConfig::gravity_direction()`, which R6's own
carve-out explicitly sanctions. So the hack does not need to be replaced with a
new mechanism. It needs to be moved to where the answer already exists, and
restated as a property of the actuator rather than of the material:

```rust
pub struct Actuator {
    // …
    /// Scales this body's own tangential budget at contacts outside its
    /// Support Set. The player sets ~0 so jumps along vertical surfaces are
    /// not grabbed; magnetic boots would set 1.0. Default 1.0.
    pub non_support_grip: f32,
}
```

The player's collider goes back to `FrictionModel::Isotropic(0.8)`. The engine
gains one rule — *a body may scale its own tangential budget at contacts that do
not support it* — and the rule is one-sided by construction, because it modulates
an actuator's budget rather than the combined contact material.

Ordering is already correct. §6.5 resolves the Support Set before the solve, and
the tangential bound is read live inside the solve loop, so membership is known
by the time a budget is needed. A contact's `μ` is unchanged; what changes is
what *this* body is allowed to draw against it.

What this buys, in the terms the rest of the document uses:

| | Under `AxisBiased` | Under non-support grip |
|---|---|---|
| R6's bound | false — body-local up in the drive path | **discharged** |
| Ragdoll | carries a standing body's friction model | irrelevant — no constraint precondition |
| Crate the player leans on | inherits jump tuning through the combine | unaffected |
| Magnetic boots (§7) | `μ = 0`, impossible | `non_support_grip: 1.0` |
| Grip on ice | free | free — the collider's own `μ`, unwrapped |

One consequence reaches outside §6. With no controller hack occupying the
player's friction field, **the collider's own `μ` is a real material
coefficient**, and the drive can be bounded by it directly. D1 (§11) needs no
second coefficient to read.

---

## 7. SOLID

Not decoration; each principle names the specific failure it prevents here.

**Single responsibility.** The present `Velocity` has two — command and
measurement — and that is R5's entire complaint. The split into `DriveIntent`,
`Actuator` and `BodyMotion` gives each one job. Inside physics: `SupportResolver`
finds supports, `TractionPlanner` turns intent into per-contact targets,
`TangentialSolver` solves one row. A change to how supports are detected touches
one file.

**Open/closed.** `ReactionAnchor` is the extension point. A magnetic-boots
character that pushes against a wall, a swimmer pushing against water, a
grappling hook pushing against a rope — each is a new anchor kind, and none of
them modifies `TangentialSolver`, `PgsNgsSolver` or `RigidBody`. Contrast the
present design, where every new driven-body behaviour has meant another
`is_velocity_driven` branch.

The wall example turns on §6.7 rather than on the anchor abstraction. The
player's `FrictionModel::AxisBiased` sets `wall: 0.0` (`spawners/player.rs:44`),
and since the traction budget draws `μ` from the contact, a wall anchor bounded
by that material transmits exactly nothing — the extension point open, the bound
closing it. §6.7 makes the zero a property of the actuator, where a character
meant to grip walls sets it to one, instead of a property of every contact the
player makes.

**Liskov.** After this work a driven body *is* an ordinary rigid body through
the whole solve: nothing in `integrate_forces`, `integrate_bodies` or CCD needs
to ask whether it is driven, because a drive is now indistinguishable from any
other contact impulse. This is R11 restated as a type-level property.

**Sleeping is the exception, and it is an expected survivor rather than an
oversight.** `set_body_velocity_drive` wakes the body on every call
(`world.rs:523`), and that wake is load-bearing: `EnergyTracker::update_body`
(`sleep/energy.rs:29`) is purely velocity-based with no drive awareness, so a
driven body with a saturated bound and near-zero velocity — a character walking
into a wall, or leaning on a crate too heavy to move — falls below both
thresholds for `delay_frames` and sleeps. Once asleep its traction rows stop
being solved, and by the engine's own warning at `world.rs:288-292` any impulse
aimed at it is discarded. The player would freeze against the wall and stay
frozen after releasing the stick.

`push_drive_intent` therefore inherits the wake, and its condition is "does this
entity have a non-trivial `DriveIntent`" — a body-is-driven test living in the
sleep path. It gets the comment R11 asks for, naming that failure.

One apparent instance is not one. `water/coupling.rs:224` gates a wake — a
self-propelled thing pushes water behind it — which is a gameplay and VFX
categorisation, not a physics special case. It **relocates** to "does this entity
have an `Actuator`"; it does not disappear. It also lives in `src/water/`, and
R11 counts sites in `src/physics/`, so it was never inside the budget it might
otherwise be credited against.

**Interface segregation.** `CharacterControlSystem` takes `DriveIntent`
(write) and `BodyMotion` (read) and cannot express "edit the measurement".
`MovingPlatformSystem` takes `DriveIntent` alone. The compiler enforces R5
rather than a comment asking writers to leave the `y` axis alone.

**Dependency inversion.** The solver depends on the `TangentTarget` abstraction,
not on the concept of a character. Gameplay depends on `Actuator`, not on
`RigidBody`. Both follow the house pattern already set by `StaticGeometry`,
`ConstraintSolver` and `ManifoldConditioner`: the physics engine states what it
needs as a trait and lets the caller supply it.

---

## 8. Requirements traceability

| Req | Satisfied by | Mechanism or construction? |
|---|---|---|
| R1 Conservation | §6.1 tangential row via `apply_impulse_pair` at the contact point | construction |
| R2 Infinite-mass partners | `inv_mass == 0` on the partner | mechanism yes; **"accelerates as today" reworded** — R2 keeps its mechanism claim, and the magnitude is restored by `drive_gain` rather than promised by R7, §11 |
| R3 Declared reaction partner | `Actuator::anchor`, `ReactionAnchor` §6.4 | declaration |
| R4 Support-relative targets | row solves *relative* velocity §6.1 | **per contact, not per body** — a bridging body has no single anchor frame and its world speed is an outcome, §6.1 |
| R5 Command ≠ measurement | `DriveIntent` / `BodyMotion` split §3, §7 | type system |
| R6 No privileged coordinates | Contact Frame §6.1; `gravity_direction()` §6.6; `AxisBiased` leaves the drive path §6.7 | **basis and bound both** — construction, delivered by Stage 2b |
| R7 Authority bounded by contact | `μ_drive·N` row bounds §6.1 | construction; magnitude chosen — `drive_gain: 5.0`, with its ledger in §11 |
| R8 Cheats named and bounded | `Allowance` + `AllowanceApplier`, single module; `drive_gain` named and instrumented alongside it §11 | discipline, one file — the ledger is the whole airborne controller (§6.3), unsupported jumps (D2a), and one scalar per actuator |
| R9 Distribution over supports | per-contact `μ·N` bound alone, no weighting term §6.1 | construction |
| R10 Linear and angular unified | torsional row §6.2 | **form only** — ground yaw is an Allowance in practice, §6.2 |
| R11 Surface does not grow | see below | measured, gated |
| R12 Tests first | §9 stage 0 | process |

**R11 in detail.** Today's surface, from the requirements: `RigidBody::velocity_drive`,
the target shift, the warm-start persistence rule, the restitution suppression,
and the `integrate_forces` application. The requirements list five; there are
**six**. The sixth is the sleep wake at `world.rs:523` (§7), which the
requirements' own inventory misses — so the count to beat is higher than R11
states, and the sixth is the one site this design expects to keep.

The `water/coupling.rs` flag is outside that scope and is a relocation, not a
deletion (§7).

After this design: `RigidBody` carries no drive state at all; the target shift
and the `integrate_forces` application are deleted along with it. The
warm-start and restitution special cases in `pipeline/solver.rs` exist because a
pre-solve drive re-asserts approach velocity every substep, keeping persisted
contacts permanently above `restitution_velocity_threshold`. A body whose
velocity changes *only* through solved impulses does not do that, so the
expectation is that both can be deleted too. That expectation is a claim about
behaviour, not a proof — it is exactly what the `driven body at rest` and
`stack stability` acceptance tests exist to check. **If either special case must
survive, it survives with a comment naming the instability it prevents, and the
count is still lower than today.**

§6.7 is neutral on this count rather than a saving. `FrictionModel::AxisBiased`
is not in `src/physics/`'s drive surface — it is a collider material, and no
drive code branches on it — so retiring it removes no site. Reading
`non_support_grip` inside the tangential bound adds one, and it is a per-contact
budget scale rather than a body-is-driven test: it applies uniformly to grip and
drive, and asks only whether a contact is in the Support Set the solver already
built. It is a parameter, not a branch, and it should be justified as one in
Stage 7's audit.

---

## 9. Delivery plan

Sequenced so that each stage is independently verifiable and no stage leaves
the game unplayable.

**Stage 0 — Acceptance tests (R12, blocking). Landed.** All eight scenarios
from the requirements table live in
`src/physics/bench_harness/tests/traction_drive.rs`, driving the real
`PhysicsWorld` with `CharacterControlSystem` and `MovingPlatformSystem` replayed
by hand in dispatcher order. **Nothing else starts until this is green where it
should be green** — the current behaviour is partly accidental, so without this
a regression and a correction look identical.

What the eight measured against the current engine:

| Scenario | Kind | Measurement |
|---|---|---|
| vertical lift carry | specification | rides at 1.83 m/s with **zero** slip and zero gap movement |
| reversal hover | specification | gap opens 0.80 → 1.45 m at the reversal, platform catches them |
| horizontal lift carry | characterisation | **the passenger is not carried**: braked to 0.00 m/s under a 3 m/s deck, drifts 12.8 m along it and falls off the back |
| cruise under load | specification | 1.828 m/s against an authored 2.0 and a 131 kg passenger — the same one frame of gravity the unloaded lift sags by |
| edge walk | characterisation | platform yaws **+0.41 rad/s**, *with* the walker. Conservation demands the opposite sign (R1) |
| crate push | characterisation | a 768 kg crate is *punted* to 4.70 m/s — 6.5× the 0.73 m/s the mass ratio predicts, and near the walker's own 5.0 walk speed |
| stack stability | specification | no lateral drift, box speeds under 0.04 m/s |
| driven body at rest | specification | resting against the wall at x = 0.262, velocity identically zero |

The two defects the design exists to fix are asserted **in their broken form**,
with the inversion spelled out in the failure message: when `edge walk` starts
failing with a negative yaw and `horizontal lift carry` with a zero drift, the
traction drive has landed and those assertions become specifications.

`crate push`'s peak is compared against the mass-ratio speed rather than pinned
to a number, and under D1's settled gain of 5.0 that comparison stays a
**characterisation permanently** (§11's ledger): the player transmits five times
the tangential force through the contact, so the outcome is not the mass ratio
the requirement names. Its other assertion is a specification — the crate never
outruns the walker (measured overtake 0.006 m/s) — because that is a
conservation claim and survives any gain. A crate that outran its pusher would be
receiving momentum from nowhere at the far end of the same pump.

The `edge walk` acceleration precondition is a specification: 40 m/s² is the
value the project chose (§11), not an accident of the current drive.

Stage 0 **characterises** the engine as it stands, which is why it was built
before any decision below it was settled: it is the safety net for every stage
after it.

**Stage 1 — Channel split. Landed.** `DriveIntent`, `Actuator`, `BodyMotion`
live in `src/drive/`; `CharacterControlSystem` and `MovingPlatformSystem` write
intent only, and the ECS `VelocityDriven` component is gone — its two
declarations (`max_accel`, `angular_max_accel`) are the `Actuator`, its one
command (`angular_velocity`) is `DriveIntent::angular_target`. No physics
change: `PhysicsSyncSystem::push_drive_intent` folds the frame's command into
today's `set_body_velocity_drive`, and inherits its sleep wake (§7). R5 lands
alone, and the Stage 0 tests did not move.

Three things a later stage needs to know about the shape it inherits:

- **The jump verbs are all Allowances here, as D2a says they must be.** There is
  no Support Set yet, so no jump has support to push off, and the general rule
  covers every one of them without a `CoyoteTime` case. What arrives at the
  engine is still a *target velocity*, not an impulse — the drive underneath is
  a target chase until Stage 5 — so Stage 1's jump sets the target's component
  along the support normal rather than adding momentum to the body. Same
  numbers as before, one channel further out.
- **The support normal is read from gravity**, through the new
  `PhysicsConfig::gravity_direction()`, and nothing in the drive path writes a
  literal `Vector3::y()`. Stage 5 replaces the accessor call with the real
  support normal; the arithmetic that consumes it is already general (§10.2's
  slope jump falls out of it unchanged). A world with no gravity has no axis to
  jump along, and there the verbs are inert.
- **A projection acts on a rise only.** `vel.y *= factor` had a `vel.y > 0.0`
  guard at one of its two call sites and not the other, where the value was
  always positive anyway. `NormalProjection` carries the guard instead of the
  caller: cutting a *fall* short is not a verb anyone has, and the same
  multiplier applied downward reads as a parachute. Two cutoffs on one frame
  compose as a product, which is what the two unguarded call sites did.

One behaviour changed as a direct consequence, and it is the point of R5 rather
than a side effect: `ExplosionSystem` adds knockback to `Velocity`
(`explosion/systems.rs:93`), which for the player used to leak into the next
frame's drive target — an ECS-side blast on top of the `PhysicsImpulseQueue`
one the same explosion already queues. `Velocity` is now measurement only, so
that second blast is gone and the queued impulse is the whole of it. Grenade
jumping is weaker until the loop is either deleted or moved onto the impulse
channel where it belongs; the choice is a feel decision, not a mechanism one.

**Stage 2 — Support set.** `SupportResolver` and `SupportSet`; `GroundingDetector`
reimplemented on top of it against `gravity_direction()`. R6 for
contact-derived grounding. Still no change to how the drive is applied.

This stage covers `ContactGroundingSystem`'s consumers — rollers, drones,
anything without an animator. Bringing the player on is Stage 2c, which is a
behaviour change rather than a reimplementation and is sequenced separately for
that reason.

**Stage 2b — Non-support grip.** Retire `FrictionModel::AxisBiased` (§6.7): the
player's collider becomes `Isotropic(0.8)`, `Actuator` gains `non_support_grip`,
and the tangential bound scales by it at contacts outside the Support Set. R6's
bound lands here. It rides on Stage 2 because it needs the Support Set and
nothing else — in particular it does **not** need the drive to be solver-native,
so it is verifiable against the current engine.

`driven body at rest` is the direct check: it holds a driven body against a wall
and a floor and asserts the wall contact transmits no drive, which is today
`wall: 0.0` doing the work and afterwards `non_support_grip` doing it. If that
scenario's numbers do not move, the substitution is exact. Two second-order
effects have no test and want a play-test: wall-adjacent jumps, which is the
behaviour the hack was written for, and anything the player leans on, which
previously inherited the hack through the material combine and now does not.

It must land before Stage 5, because it decides what `μ` the traction budget
reads.

**Stage 2c — One grounding source.** Delete the `!&animators` filter from
`ContactGroundingSystem` and the `Grounding` write at `animation/systems.rs:124`,
so every body's grounding comes from the Support Set (§6.6). `CharacterAnimator`
takes `is_grounded` from the component instead of deriving it at
`animator.rs:185`; the probes stay, as a rangefinder for landing height and
normal.

This is the stage that cuts the animation→physics loop, and it is the one stage
whose risk is entirely about feel rather than about numbers. Three work items
come with it, and none of them is optional:

- **A step whose landing target has no ground is illegal.** The foot placer must
  shorten or re-aim it. Today the character flipping to airborne supplies this
  behaviour implicitly; afterwards nothing does. `foot_placer/sim.rs` and the
  invariants harness cover this offline, so it is testable before it is playable.
- **Grounding needs an explicit forgiveness window.** Contact grounding chatters
  where reach-based grounding was smooth — descending stairs, crossing MC terrain
  — because a capsule genuinely loses contact between footfalls. The leniency
  that `probe_length` supplied by accident becomes a named hysteresis or coyote
  window on the FSM side. `ContactGroundingSystem` already carries support
  forward for sleeping bodies, so the shape exists.
- **Head tilt and stride read the new answer.** Anything keyed on
  `Grounding`/`is_grounded` transitions sees a different transition rate.

Rollers and drones already run on contact grounding, so if the contact set were
too noisy to stand on, that would be visible before this stage rather than
during it.

Sequencing is free: this stage touches grounding and animation, Stage 2b touches
the tangential bound, and neither reads the other's output. It should land before
Stage 5 all the same, so that the drive and the FSM are agreed about support
before the drive starts being bounded by it.

**Stage 3 — Tangential row.** Generalise `solve_friction_impulse` to take a
target, with every call site passing zero. A pure refactor, and the stage's
entire value is that any movement in the bench numbers means a real bug — so pin
the expression form rather than trusting the algebra: the generalised row must
compute `(v_target - v_rel).dot(&t)` without reassociating into
`v_target.dot(&t) - v_rel.dot(&t)`. With `v_target = 0` the first form is
bit-identical to today's `-(v_rel · t)` except for the sign of zero; the second
is not guaranteed to be.

**Stage 4 — Medium anchor.** Platforms move to `ReactionAnchor::Medium`,
expanded as world-anchored `ConstraintRow`s. Platforms **drop `VelocityDriven`
at this stage**; the field survives in the engine for the player until Stage 5.
A platform carrying both a velocity drive and a medium row toward the same
target would get roughly twice the authority its actuator declares, and the
row's bounds would stop meaning anything — so the two must never overlap on one
body. `cruise under load`, `vertical lift carry` and `horizontal lift carry`
must hold. This is the lower-risk half of the change and it is independently
verifiable, so it goes first.

**Stage 5 — Traction.** `TractionPlanner` feeds real targets, carrying the
commanded spin per contact (§6.1); delete `RigidBody::velocity_drive`, the target
shift, and the `integrate_forces` application. `Actuator::drive_gain` lands here
at 5.0 for the player (§11), along with the instrumentation §11 requires of it.
`edge walk` flips sign. This is the risk-bearing stage, the first at which a
traction bound governs real behaviour, and by the time it arrives it is the
shortest possible diff.

Two checks belong to this stage rather than to play-testing, both from §11's
ledger: `stack stability` under a body with 5× tangential authority, and whether
any level holds slopes in the 39°–76° band where standing and walking limits
diverge — `cos_floor` is the dial if one does.

*Why this order.* Platforms have no drive other than `VelocityDriven`
(`moving_platform.rs:144`). Deleting it before they have a medium anchor stops
every platform in the game and turns three of the eight Stage 0 tests red for
reasons unrelated to what the stage changed — which is exactly the signal Stage
0 exists to protect.

**Stage 6 — Torsional row and allowances.** R8 and R10. Scope the allowance work
as the airborne character controller in full (§6.3), not as a garnish. Play-test
for feel; this is where the deferred **Effort** question (requirements §8) gets
answered with data instead of prediction.

**Stage 7 — Surface audit.** Count the drive-aware sites in `src/physics/`;
delete the `pipeline/solver.rs` special cases if the tests permit. Relocate
`BuoyancyBody::is_velocity_driven` to an `Actuator` test — it is a wake gate, not
a physics special case, and it was never in the R11 budget. Justify every
survivor in a comment. R11.

---

## 10. Known risks

### 10.1 The acceleration cliff — the largest risk in the project

The form of the tangential row says nothing about the numbers it produces
against this project's actual configuration, and those numbers decide how the
change feels.

`LocomotionConfig::ground_accel` is **40.0 m/s²** (`character/config.rs:81`).
Under R7 the steady-state traction bound on flat ground is `μ·m·g / m = μ·g`:

| Surface | How `μ` is combined | `μ` | Ceiling |
|---|---|---|---|
| Terrain | static contacts take the collider's own friction, no combine (`static_contacts.rs:93`) | 0.80 | **7.85 m/s²** |
| Moving platform | dynamic pairs take the geometric mean (`collider.rs:188`) of player 0.8 and platform 0.9 | 0.85 | **8.32 m/s²** |

That is a **~5× cut in both acceleration and braking**. A stop from
`walk_speed: 5.0` goes from roughly 0.13 s to 0.64 s. Stage 5 will not read as
"edge walk flips sign"; it will read as "the character became a curling stone".

A second consequence follows from the table rather than the headline: the
ceiling is a property of whatever the character is standing on. The same stick
input gives measurably different responsiveness on terrain, on a platform, and
on a crate. That may be desirable — it is what makes a surface feel like a
material — but it is new, and it is not something any current tuning value
anticipates.

This is **not** the deferred **Effort** question, which is about slopes. Nor is
it a missing friction concept: §11 works through the obvious candidate — a
static-versus-kinetic split — and finds that it argues for the low number rather
than against it.

**The risk was measured rather than argued, and it is real.** `ground_accel` was
set to 7.85 in the current engine — a one-line change, since the value passes
straight into the movement rule (`character_control.rs:165`) — and played. It is
far too slow. A humanoid's honest pace is not a fun pace, and no amount of
tuning `walk_speed` around the ceiling recovers it. **40 m/s² stands as the
correct value for this game.**

That resolves §10.1 from a risk into a cost. The character is a cartoon by
decision, `drive_gain: 5.0` is where the decision lives, and §11 records what it
buys and what it costs.

### 10.2 Jumping gains a direction it did not have

The same species as the cliff above: a magnitude consequence the form of the
design does not reveal, and one that decides feel rather than correctness.

Today `vel.0.y` means the player always jumps straight up, whatever they are
standing on. §6.3 delivers a jump along the **support normal**, so on a slope
they jump at an angle. `cos_floor` is 45° (`spawners/player.rs:46`), so any
surface up to 45° is a valid support, and at that limit the jump keeps
`cos 45° = 71%` of its vertical velocity — but height goes as `v²`, so it loses
**half its height**, while gaining lateral speed the player never asked for.

That may be exactly what is wanted: it makes slopes read as slopes. But it is a
decision, and if it is not wanted the correction is available and legal. R6's own
carve-out explicitly sanctions reading the world's gravity through an accessor,
so a documented blend of the support normal toward `-gravity_direction` is
permitted by the requirement rather than in tension with it. What R6 forbids is a
literal `Vector3::y()`, and the blend does not need one.

Migael's decision: accepted for now. After implementation, will playtest and
decide on further tuning.

### 10.3 Everything else

**KeepUpright sits downstream of every conservation claim.** The player's
upright constraint uses `Enforcement::HardProjection` with
`max_impulse: f32::INFINITY` (`spawners/player.rs:59-64`) — a post-solve velocity
rewrite (`constraint/types.rs:340`) that runs after the solve, outside it, and
non-conservatively. Traction at a single off-axis foot contact induces tilt; that
tilt is then silently eaten by the projection. R1 is satisfied *within* the
solve, and the design should not claim more than that until the interaction is
measured. The `edge walk` and `stack stability` scenarios are the place it will
show up.

**A character's contact patch is small.** A capsule resting on terrain may
produce a single contact, giving a small lever arm for the reaction. For yaw the
consequence is not "weak" but zero, and §6.2 treats the allowance as the ground
mechanism rather than a mitigation. Stage 6 measures it.

**Fresh contacts have no budget for one substep.** A contact created this
substep has `accumulated_normal_impulse` near zero at the first iteration, so
its traction bound starts near zero and grows as the normal row converges. This
is the same warm-up friction already has; the expectation is that it is
invisible, and the `crate push` jitter assertion is the check.

**The friction cone clamp becomes direction-dependent.** §6.1 writes the row as
a scalar per tangent, but the existing solve computes `delta_t1` and `delta_t2`
separately and then clamps the pair jointly to one cone (`friction.rs:47-59`).
With a non-zero target the clamp no longer merely shortens an opposing impulse —
it rotates a commanded one. "Unchanged in cost" is true; "unchanged in
behaviour" holds only for `v_target = 0`, which is the Stage 3 invariant and not
a general claim.

**Traction and grip share one budget.** They must — that is R7 — but it means a
character accelerating hard has less grip against a slope, and walking uphill
will cost speed. This is the *intended* physics and the reason the requirements
already anticipate an **Effort** parameter. It is a feel decision, deferred, and
the architecture leaves it as a bias on the target rather than a return to
privileged axes.

**The solver-native route has bitten before.** The current `integrate_forces`
formulation was itself arrived at after instabilities, so "put it in the solver"
is not automatically safer for having a nicer diagram. The mitigation is Stage
3: the row generalisation ships as a no-op refactor with the target pinned to
zero, so if the solve is going to destabilise, it does so behind a
single-commit revert — before any behaviour depends on it.

---

## 11. Decisions

Both are settled. They are kept here, with the alternatives they were chosen
over, because each is a decision about how the game should feel rather than a
consequence of the mechanism — and a reader who disagrees needs to see what was
weighed, not only what won.

- **D1** — `drive_gain: 5.0`. Decided by measurement, with a ledger of what it
  costs.
- **D2** — D2b then D2a: one grounding source, and an Allowance for the jumps
  staleness still strands.

### D1 — Does R7's bound win over R2's promise, and at what magnitude? **Settled.**

R2 promises that "a player walking on the world's terrain accelerates exactly as
they do today". R7 requires that no drive exceed `μ·N`. §10.1 shows the two
differ by a factor of five. Both are currently asserted, and both cannot stand.

**R7 wins on mechanism, and R2 is reworded to its real content** — "no separate
code path for infinite-mass partners". That much is settled: R2's parenthetical
is a magnitude claim wearing a mechanism claim's clothes, and no design can
deliver it alongside a contact-bounded drive.

#### The construction

The drive's bound is the contact's own friction, scaled by one per-actuator
factor:

```
μ_drive = μ_contact × actuator.drive_gain      // default 1.0
```

The alternative shape — a per-actuator coefficient multiplying a new isotropic
`collider.grip` field, separate from the `μ` used for grip — buys nothing here
and costs two coefficients per material that can drift apart if authors are
careless. Its only argument would be keeping a controller hack out of the drive
path, and §6.7 keeps it out of the material entirely, which is the same result
one layer earlier and without the second field.

So the collider's `μ` is the material coefficient the drive reads. Ice bites for
free, surface legibility (§10.1) is preserved for free, nothing exists to drift,
and a wall transmits drive — not because `drive_gain` can rescue a zero, which it
cannot, but because under §6.7 the wall's `μ` was never zero. A character meant
not to grip walls says so on its actuator.

#### The value: `drive_gain: 5.0` for the player

`ground_accel` was set to 7.85 in the current engine and played (§10.1). It is
far too slow to be fun. The honest ceiling is not a pace a game can be built on,
so **the player's `drive_gain` is 5.0**, restoring 39.2 m/s² on terrain and
41.6 m/s² on a platform — the present values, within tuning noise.

The rest of this section is the record of what that decision is and is not, so
that it stays a decision rather than becoming a number nobody can account for.

#### It is a cheat, and it is not a friction model

**No friction formulation supplies the missing factor of five.** The obvious
candidate is the static-versus-kinetic split this engine does not model
(`FrictionModel::evaluate` returns one coefficient regardless of sliding speed).
It does not help, for two reasons:

- **Magnitude.** μ_s/μ_k is typically 1.2–1.4×. Adding it moves the ceiling from
  7.85 to perhaps 10 m/s², not to 40.
- **Regime.** The static bound is the one that applies when sliding velocity is
  ~0. A planted foot is at zero sliding velocity, and a tangential row drives
  relative velocity toward its target, so the drive already sits in the static
  regime. There is nothing to promote it *from*: the 0.8 in §10.1's table is
  already the anchored coefficient, so "feet anchor rather than slide" argues
  for that number rather than for a larger one.

**The physical route to high acceleration is `N`, not `μ`, and it does not
survive averaging.** A runner's ground reaction reaches 2–3× body weight at
push-off, which is a genuine and large increase in the traction bound. But over a
full stride the average `N` must equal `mg` or the runner leaves the ground, so
the sustained ceiling really is `μg`. Elite sprinters accelerate at roughly
8–10 m/s². **§10.1's 7.85 m/s² is close to the correct number for a person, and
40 m/s² is a cartoon value.**

**A Coulomb cross-check says the same thing.** Under `μ_drive > μ_grip` you can
walk up a slope you cannot stand on (below). Real friction gives the opposite,
since μ_s > μ_k: you can stand on slopes you cannot walk up. The inversion is the
tell that `drive_gain > 1` is not modelling anything.

So the 5.0 is not physics and must never be documented as though it were. §1
defines it as Drive Gain and says outright that it does not model friction. A
parameter honest about being a lie is maintainable; a `μ_drive` that quietly is
not a `μ` invites exactly the reasoning above to be re-run by the next reader, at
the same cost.

#### Why this shape of cheat, and why not an Allowance

R8 already provides a mechanism for non-conservative authority, and putting the
lost responsiveness there — leaving `drive_gain` at 1.0 and spending a budgeted
impulse on the start-up transient — looks like the more disciplined option. It is
the wrong one, and the reason is worth stating because it is not obvious.

**Walking at constant velocity on flat ground requires no net tangential force.**
The row applies nothing once the body is at target speed. So nearly all the
momentum exchange this design exists to get right happens *during the transient*:
the start, the stop, the direction change. Route the transient through an
Allowance and `edge walk` stops torquing the platform for the same reason it
fails to today — the reactionless pump would have been rebuilt inside the box
marked "cheat".

That inverts the ranking of the two dials:

| | What it costs |
|---|---|
| `drive_gain > 1` | honesty about what `μ` means |
| Allowance | **conservation** — R1, and the reason for the redesign |

`drive_gain` changes exactly one scalar bound, so R1, R2, R4 and R9 all survive
intact: the drive remains a real impulse exchange at a real contact point, with
the correct torque arm, distributed across supports, silently absorbed by
infinite-mass partners. **The cheat is confined to one number in one place**,
which is what makes it auditable rather than merely admitted.

A third shape was considered and rejected: let the actuator command extra normal
load, raising `N` and with it `μ·N`. It is the closest to what real feet do
(§10.1's push-off spike), but it is more machinery, it pushes the support down in
a way that changes platform behaviour, and it buys nothing that scaling the bound
does not.

#### What it does not cost: surface legibility

Because the gain multiplies, relative surface response survives exactly. With
`drive_gain: 5.0`, terrain gives `0.8 × 5 = 4.0` → 39 m/s², and ice at `μ = 0.1`
gives `0.5` → 4.9 m/s². The 8:1 ratio is untouched: ice still reads as ice.

**Grip is not scaled.** A standing character on ice slides at the honest `μ`,
because only the drive's bound carries the gain. That asymmetry — cheat the push,
keep the resistance honest — is the whole of the decision, and it is the reason
§10.1's second consequence (surfaces become legible as materials) is a benefit
this design keeps rather than something the gain spends.

#### Making it visible

"Deliberate" is a property of the code, not of this document, so the decision
carries three obligations into implementation:

- **One named field, not a tuning constant.** `Actuator::drive_gain`, defaulting
  to 1.0, with a doc comment stating that it is not a friction coefficient and
  pointing here. Every entity that is a cartoon says so on its own actuator; an
  NPC that should feel heavy simply does not set it.
- **Instrumented at runtime.** The tangential solve knows both bounds, so it can
  report the impulse it applied beyond what `drive_gain: 1.0` would have
  permitted. A `DebugLog` entry per driven body — borrowed impulse per second,
  and the fraction of frames the rows ran saturated — turns "we cheat" into a
  number somebody can watch. This is cheap: both quantities are already in hand
  where the bound is computed.
- **Audited with the allowances.** Stage 7 counts drive-aware sites; the gain
  belongs in the same ledger as R8's budgets, because it is the same kind of
  claim about where authority comes from.

#### The ledger — what 5.0 costs

These follow from `μ_drive > μ_grip` itself, so no construction escapes them.
They are what "deliberate" means: each is a consequence somebody chose, not one
the engine sprang.

- **You can walk up a slope you cannot stand on.** Grip is `μ`, drive is `5μ`,
  so the standing limit is `atan(0.8) ≈ 39°` and the walking limit is
  `atan(4.0) ≈ 76°`. Stop moving and you slide; start moving and you climb. This
  is legible platformer behaviour and is **accepted**, with one check owed at
  Stage 5: if levels hold slopes in the 39°–76° band, `cos_floor`
  (`spawners/player.rs:46`, currently 45°) decides whether they are supports at
  all, and that is the dial for the anomaly rather than the gain.
- **`crate push` is permanently a characterisation.** The requirement's
  assertion, "push speed follows the mass ratio", is only honest when
  `μ_drive == μ_grip`. At 5× the player transmits five times the tangential force
  through the same contact, so the outcome is no longer the mass ratio the
  requirement names. The scenario keeps its specification half — the crate never
  outruns the walker, which is a conservation claim and survives any gain — and
  its mass-ratio half stays a recorded number.
- **A driver beats a bracer.** A character can push harder than it can resist
  being pushed, since only the drive carries the gain. Two characters shoving
  each other is decided by who is moving, not by mass. No scenario covers this;
  it is named here so it is recognised rather than debugged.
- **Stack stability sees 5× the shove.** A driven body leaning on a stack has
  five times the tangential authority the honest bound would give. The
  `stack stability` scenario is the check, and it is the one place the gain could
  turn a feel decision into an instability.

#### What Stage 0 does with this

Stage 0 **characterises the current engine**, and ground acceleration is 40 m/s²
both before and after this decision — so no Stage 0 assertion moves. What D1
settles is only which recorded numbers are *characterisations* (this is what the
engine does) and which are *specifications* (this must remain true):

| Assertion | Status |
|---|---|
| `edge walk` acceleration precondition | **specification** — 40 m/s² is now the intended value, not an accident |
| `crate push` mass ratio | **characterisation, permanently** — see the ledger above |
| `crate push` crate never outruns walker | specification, unchanged |

§8 marks R2 as reworded rather than open, and R7 as satisfied at a magnitude the
project chose. R6's bound is discharged by §6.7 at Stage 2b, independently.

---

### D2 — What happens to a jump the Support Set cannot deliver? **Settled: D2b then D2a.**

The FSM commits a jump from `Grounding` and clears the buffer in the same frame
(`character_control.rs:122-124`), before physics has tried to deliver anything.
If the Support Set is empty on the firing substep, the jump is swallowed: the
buffer is gone, `set_vy` was never applied, and the input is lost.

Two independent things cause it, and they want different answers.

**The structural cause is a sensor disagreement, and it is not accepted.** Today
the FSM and the drive read different sensors — foot probes for rigged humanoids,
contact manifolds for everything else — and the probe answer leads the contact
one. §6.6 removes this rather than working around it: the probe path is a closed
loop from foot placement back into physics, so a gait decision gates a jump. The
one grounding source lands at Stage 2c.

**The residual cause is ordinary staleness, and it is accepted.**
`CharacterControlSystem` runs before `PhysicsSyncSystem` while the grounding
writer runs after it (`dispatcher_builder.rs:46,56,82,91`), so the FSM reads a
one-frame-old answer from whatever sensor supplies it. A character grounded last
frame can be unsupported this one. `CoyoteTime` is that window with a name on it,
and it is wanted.

**Severity, stated precisely.** The consequence is a dropped input and up to
`launch_window: 0.2 s` of degraded ground control — not a lockup. `Launching`
gained a time-based exit in `8211d70` ("Bound the launch window so a jump cannot
trap the character"), so it promotes to `Airborne` regardless of grounding and
the next tick returns to `Grounded`. This matters for choosing a fix: a
swallowed jump does not justify the invasive option below, and a lockup would.

| | What it does | Cost |
|---|---|---|
| **D2a — Allowance fallback** | A jump commanded with an empty Support Set is delivered as an Allowance instead of through the contacts | A jump is silently non-conservative some of the time |
| **D2b — One grounding source** | The FSM and the drive read the same contact-derived answer | The FSM stops seeing the ledge the sole is over; foot placement must handle it instead, and grounding needs an explicit forgiveness window (§9, Stage 2c) |
| **D2c — Confirmed delivery** | The FSM holds the buffer until physics confirms | Needs a channel back from physics; `BodyMotion` is measurement-only, so this widens the seam R5 exists to keep narrow |

**The decision is D2b and D2a together**, in that order: D2b removes the
structural cause, D2a absorbs what staleness leaves. D2c is rejected — it buys
nothing D2a does not, and pays for it by widening the seam R5 exists to keep
narrow.

**D2b, because the probe path is a rule violation rather than a tuning choice.**
The engine's standing rule is that animation responds to physics and nothing
animation decides feeds back into it. The probe grounding path breaks it, and the
justification given for the path — probes see the ledge the sole is over — turns
out to describe the defect: with a foot placed past an edge the character goes
airborne while resting on solid ground, because of where the gait put a foot. The
capability that justified it is not lost, only relocated: keeping a foot off a
ledge is a foot placement decision, and it belongs to the foot placer, made from
the same probes, with no path into physics. §6.6 carries the argument; §9's
Stage 2c carries the work.

**D2a, because staleness remains and something must catch it.** It is not a new
mechanism — §6.3 already routes the coyote jump through an Allowance for exactly
this reason, and D2a is that rule stated once for every unsupported jump rather
than for the one state that happens to be named after it. A jump with no support
is a coyote jump whatever the FSM calls it. Nothing is dropped, the buffer clear
becomes harmless, no channel back is needed, and the `Option::take` ownership
question dissolves because the sync always consumes.

D2a's cost still belongs in R8's ledger: the same player action is conservative
or not depending on a one-frame timing the player cannot perceive. D2b makes that
budget small and mundane instead of structural, which is the difference between
an allowance that fires on a designed sensor lead and one that fires on a frame
boundary. **The budget shrinks; it does not vanish.** If it needs to shrink
further, the next move is ordering rather than mechanism — deriving `Grounding`
from the previous frame's manifolds *before* `CharacterControlSystem` rather than
after, which is a dispatcher edge, not a design change. That is held in reserve
rather than taken now: it is only worth doing if the allowance's jump traffic
turns out to be more than occasional, and the instrumentation §11 asks of
`drive_gain` should count this too.

#### Delivery

The two halves land at different stages, and the order is forced by neither —
they are independent, and each is useful alone.

| Half | Stage | What lands |
|---|---|---|
| **D2a** | Stage 1 | The jump verbs get their homes (§6.3). An unsupported jump — coyote, stale grounding, any other — is an Allowance. The sync always consumes, so `Option::take` is unconditional. |
| **D2b** | Stage 2c | One contact-derived grounding source; the animation→physics loop is cut (§6.6). |

D2a is what unblocks Stage 1, because that is where gameplay stops writing
velocity and every jump verb needs somewhere to go. Note that it must be written
as the *general* rule from the start — "a jump with no Support Set is an
Allowance" — and not as a `CoyoteTime` special case, or Stage 2c will change how
often it fires and the narrower rule will not cover what arrives.
