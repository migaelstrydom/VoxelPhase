use nalgebra::{UnitQuaternion, Vector3};

use super::super::framework::{run_scenario, BenchRunConfig, BenchSample, PhysicsBenchScenario};
use super::super::scenarios::BoxSlidesDownWallScenario;
use super::assertions::analyse_jitter;
use super::write_exports;

// ═══════════════════════════════════════════════════════════════════════════
// Analytical ODE for the "ladder sliding down a wall" problem
// ═══════════════════════════════════════════════════════════════════════════

/// Analytical ODE for the "ladder sliding down a wall" problem.
///
/// State: `[θ, θ̇]` where θ is the CCW tilt angle from vertical.
///
/// The constrained box has center of mass at:
///   x_c = L sin θ + a cos θ
///   y_c = L cos θ + a sin θ
///
/// **Frictionless case** (Lagrangian):
///   J(θ) θ̈ = 2mLa cos(2θ) θ̇² + mg(L sin θ − a cos θ)
///   where J(θ) = m(L² + a² − 2La sin 2θ) + I_z.
///
/// **With friction** (Newton + torque about CM): solve a 3×3 linear system
/// for (θ̈, N_w, N_f) at each step:
///
/// ```text
///   ┌ −m·u        1        −μ_f         ┐ ┌ θ̈ ┐   ┌ −m·w₁·θ̇²      ┐
///   │ −m·v        μ_w       1            │ │ Nw │ = │  mg − m·w₂·θ̇²  │
///   │  I      u+μ_w·w₁  −(p−μ_f·w₂)     │ └    ┘   └  0              ┘
///   └                                    ┘
/// ```
///
/// where u = L cosθ − a sinθ, v = −L sinθ + a cosθ, w₁ = L sinθ + a cosθ,
/// w₂ = L cosθ + a sinθ, p = L sinθ − a cosθ, I = I_z/m = (a² + L²)/3.
struct LadderOde {
    a: f32,
    big_l: f32,
    gravity: f32,
    mu_wall: f32,
    mu_floor: f32,
}

impl LadderOde {
    fn new(half_extents: Vector3<f32>, gravity: f32) -> Self {
        Self {
            a: half_extents.x,
            big_l: half_extents.y,
            gravity,
            mu_wall: 0.0,
            mu_floor: 0.0,
        }
    }

    fn with_friction(mut self, mu_wall: f32, mu_floor: f32) -> Self {
        self.mu_wall = mu_wall;
        self.mu_floor = mu_floor;
        self
    }

    /// Moment of inertia per unit mass about the Z axis (cuboid).
    fn i_per_mass(&self) -> f32 {
        (self.a * self.a + self.big_l * self.big_l) / 3.0
    }

    /// Helper quantities that appear repeatedly in the equations.
    fn coeffs(&self, theta: f32) -> LadderCoeffs {
        let (s, c) = (theta.sin(), theta.cos());
        let a = self.a;
        let l = self.big_l;
        LadderCoeffs {
            u: l * c - a * s,
            v: -l * s + a * c,
            w1: l * s + a * c,
            w2: l * c + a * s,
            p: l * s - a * c,
        }
    }

    /// Solve the 3×3 system for (θ̈, N_w, N_f) given (θ, θ̇).
    /// Returns (theta_ddot, n_wall_per_mass, n_floor_per_mass).
    fn solve(&self, theta: f32, theta_dot: f32) -> (f32, f32, f32) {
        let c = self.coeffs(theta);
        let g = self.gravity;
        let i = self.i_per_mass();
        let mw = self.mu_wall;
        let mf = self.mu_floor;
        let td2 = theta_dot * theta_dot;

        // Rows of [A | b] for the system A·x = b, where x = [θ̈, Nw, Nf].
        let a00 = -c.u;
        let a01 = 1.0;
        let a02 = -mf;
        let b0 = -c.w1 * td2;

        let a10 = -c.v;
        let a11 = mw;
        let a12 = 1.0;
        let b1 = g - c.w2 * td2;

        let a20 = i;
        let a21 = c.u + mw * c.w1;
        let a22 = -(c.p - mf * c.w2);
        let b2 = 0.0;

        // Cramer's rule.
        let det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20)
            + a02 * (a10 * a21 - a11 * a20);

        if det.abs() < 1e-12 {
            return (0.0, 0.0, 0.0);
        }

        let inv = 1.0 / det;

        let theta_ddot = inv
            * (b0 * (a11 * a22 - a12 * a21) - a01 * (b1 * a22 - a12 * b2)
                + a02 * (b1 * a21 - a11 * b2));

        let nw = inv
            * (a00 * (b1 * a22 - a12 * b2) - b0 * (a10 * a22 - a12 * a20)
                + a02 * (a10 * b2 - b1 * a20));

        let nf = inv
            * (a00 * (a11 * b2 - b1 * a21) - a01 * (a10 * b2 - b1 * a20)
                + b0 * (a10 * a21 - a11 * a20));

        (theta_ddot, nw, nf)
    }

    /// Center-of-mass position from angle.
    fn cm_position(&self, theta: f32) -> (f32, f32) {
        let a = self.a;
        let l = self.big_l;
        (
            l * theta.sin() + a * theta.cos(),
            l * theta.cos() + a * theta.sin(),
        )
    }

    /// Solve with stiction: if friction would reverse the slide direction
    /// (θ̇ ≈ 0 and θ̈ < 0), the system is in static equilibrium and θ̈ = 0.
    fn solve_with_stiction(&self, theta: f32, theta_dot: f32) -> (f32, f32, f32) {
        let (theta_ddot, nw, nf) = self.solve(theta, theta_dot);

        // If the body is at rest (or nearly so) and friction would push it
        // backwards, the system is stuck. Clamp to zero acceleration.
        if theta_dot.abs() < 1e-6 && theta_ddot < 0.0 {
            return (0.0, nw, nf);
        }

        (theta_ddot, nw, nf)
    }

    /// Integrate the ODE from θ₀ with θ̇₀ = 0 using RK4.
    /// Returns samples at each dt step: (time, θ, θ̇, x_c, y_c).
    /// Stops when the box separates from the wall (N_w ≤ 0) or the box
    /// sticks (friction holds it in static equilibrium).
    fn integrate(&self, theta0: f32, dt: f64, duration: f64) -> Vec<LadderSample> {
        let mut samples = Vec::new();
        let mut t = 0.0f64;
        let mut state = [theta0 as f64, 0.0f64]; // [θ, θ̇]

        while t <= duration + 1e-9 {
            let theta = state[0] as f32;
            let theta_dot = state[1] as f32;
            let (x_c, y_c) = self.cm_position(theta);
            let (_, nw, _) = self.solve_with_stiction(theta, theta_dot);

            samples.push(LadderSample {
                time: t as f32,
                theta,
                x_c,
                y_c,
            });

            // Wall separation: box lifts off the wall.
            if nw < -1e-6 {
                break;
            }

            // RK4 step (in f64 for precision).
            let f = |s: [f64; 2]| -> [f64; 2] {
                let th = s[0] as f32;
                let th_dot = s[1] as f32;
                let (th_ddot, _, _) = self.solve_with_stiction(th, th_dot);
                [s[1], th_ddot as f64]
            };

            let k1 = f(state);
            let s2 = [state[0] + 0.5 * dt * k1[0], state[1] + 0.5 * dt * k1[1]];
            let k2 = f(s2);
            let s3 = [state[0] + 0.5 * dt * k2[0], state[1] + 0.5 * dt * k2[1]];
            let k3 = f(s3);
            let s4 = [state[0] + dt * k3[0], state[1] + dt * k3[1]];
            let k4 = f(s4);

            state[0] += dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
            state[1] += dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);

            // Clamp θ̇ to non-negative: the ladder can't slide backwards.
            if state[1] < 0.0 {
                state[1] = 0.0;
            }

            t += dt;
        }

        samples
    }
}

struct LadderCoeffs {
    u: f32,
    v: f32,
    w1: f32,
    w2: f32,
    p: f32,
}

#[derive(Debug, Clone, Copy)]
struct LadderSample {
    time: f32,
    theta: f32,
    x_c: f32,
    y_c: f32,
}

/// Extract the tilt angle θ (CCW from vertical, around Z) from a quaternion.
fn extract_theta_z(q: &UnitQuaternion<f32>) -> f32 {
    // For a pure Z rotation: q = [cos(θ/2), 0, 0, sin(θ/2)].
    2.0 * q.quaternion().k.atan2(q.quaternion().w)
}

/// Compare simulation trajectory against analytical reference.
fn compare_against_ode(
    run: &super::super::framework::BenchRunResult,
    ref_samples: &[LadderSample],
    fixed_dt: f32,
    tol: f32,
) {
    let last_ref = ref_samples.last().unwrap();
    let ref_end_time = last_ref.time;

    let mut max_x_err = 0.0f32;
    let mut max_y_err = 0.0f32;
    let mut comparisons = 0usize;

    for sample in &run.samples {
        if sample.sim_time > ref_end_time - fixed_dt {
            break;
        }

        let ref_sample = ref_samples
            .iter()
            .min_by(|a, b| {
                (a.time - sample.sim_time)
                    .abs()
                    .partial_cmp(&(b.time - sample.sim_time).abs())
                    .unwrap()
            })
            .unwrap();

        let x_err = (sample.x - ref_sample.x_c).abs();
        let y_err = (sample.y - ref_sample.y_c).abs();
        max_x_err = max_x_err.max(x_err);
        max_y_err = max_y_err.max(y_err);
        comparisons += 1;
    }

    eprintln!("compared {comparisons} samples against analytical ODE");
    eprintln!("max position error: x={max_x_err:.6}, y={max_y_err:.6}");

    assert!(
        comparisons >= 10,
        "expected at least 10 comparison samples, got {comparisons}"
    );
    assert!(
        max_x_err < tol,
        "x_c diverges from analytical solution: max error {max_x_err:.6} (tol={tol})"
    );
    assert!(
        max_y_err < tol,
        "y_c diverges from analytical solution: max error {max_y_err:.6} (tol={tol})"
    );
}

/// Run jitter assertions on a wall-slide trajectory.
fn assert_no_wall_slide_jitter(samples: &[BenchSample], fixed_dt: f32, wall_contact_end: f32) {
    let jitter = analyse_jitter(samples, fixed_dt, wall_contact_end);
    eprintln!(
        "jitter: x_rev={} y_rev={} max_jerk=({:.1},{:.1}) contact_flips={} frames={}",
        jitter.x_reversals,
        jitter.y_reversals,
        jitter.max_x_jerk,
        jitter.max_y_jerk,
        jitter.contact_flips,
        jitter.frames_analysed,
    );
    assert_eq!(
        jitter.x_reversals, 0,
        "x moved backwards during wall slide ({} reversals = contact jitter)",
        jitter.x_reversals
    );
    assert_eq!(
        jitter.y_reversals, 0,
        "y moved upward during wall slide ({} reversals = contact jitter)",
        jitter.y_reversals
    );
    // Jerk threshold: for gravity g=9.81 at 60Hz, the baseline
    // gravitational jerk is ~g/dt² ≈ 35000. Allow 2× that to catch
    // spikes from contact flickering without false-positiving on
    // normal acceleration changes.
    let jerk_limit = 80000.0;
    assert!(
        jitter.max_x_jerk < jerk_limit,
        "x acceleration spike during wall slide: jerk={:.1} (limit={jerk_limit})",
        jitter.max_x_jerk
    );
    assert!(
        jitter.max_y_jerk < jerk_limit,
        "y acceleration spike during wall slide: jerk={:.1} (limit={jerk_limit})",
        jitter.max_y_jerk
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn box_slides_down_wall_frictionless_matches_analytical_ode() {
    let scenario = BoxSlidesDownWallScenario::new(0.0);
    let fixed_dt = 1.0 / 60.0;
    let duration = 3.0;

    let cfg = BenchRunConfig {
        fixed_dt,
        duration,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "box_slides_down_wall");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // Integrate the analytical reference with a fine time step.
    let ode = LadderOde::new(scenario.half_extents, 9.81);
    let ref_dt = (fixed_dt as f64) / 10.0;
    let ref_samples = ode.integrate(scenario.theta0, ref_dt, duration as f64);

    assert!(
        !ref_samples.is_empty(),
        "analytical ODE should produce samples"
    );

    let last_ref = ref_samples.last().unwrap();
    let sep_theta_deg = last_ref.theta.to_degrees();
    eprintln!(
        "wall separation at θ={sep_theta_deg:.2}°, t={:.4}s",
        last_ref.time
    );

    compare_against_ode(&run, &ref_samples, fixed_dt, 0.05);
    assert_no_wall_slide_jitter(&run.samples, fixed_dt, last_ref.time);

    // Verify the simulation extracts a reasonable tilt angle.
    let world_check = {
        let mut world = scenario.build_world();
        let handle = scenario.setup(&mut world);
        let body = world.body(handle).unwrap();
        let theta_initial = extract_theta_z(&body.rotation());
        (theta_initial - scenario.theta0).abs()
    };
    assert!(
        world_check < 1e-4,
        "initial rotation should match theta0: error={world_check:.6}"
    );

    // The box must not fall through the floor at any point.
    let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
    assert!(min_y > -0.1, "box fell through floor: min_y={min_y:.4}");
}

#[test]
fn box_slides_down_wall_with_friction_matches_analytical_ode() {
    // Critical μ for static equilibrium at θ=30° is ~0.25 (from
    // tanθ = 2μ/(1−μ²)). Use 0.2 so the ladder slides but is
    // noticeably slowed compared to frictionless.
    let friction = 0.2;
    let scenario = BoxSlidesDownWallScenario::new(friction);
    let fixed_dt = 1.0 / 60.0;
    let duration = 3.0;

    let cfg = BenchRunConfig {
        fixed_dt,
        duration,
        ..BenchRunConfig::default()
    };
    let run = run_scenario(&scenario, cfg);
    write_exports(&run, "box_slides_down_wall_friction");

    assert!(!run.samples.is_empty());
    assert_eq!(run.dropped_steps, 0);

    // Integrate the analytical reference with friction.
    let ode = LadderOde::new(scenario.half_extents, 9.81).with_friction(friction, friction);
    let ref_dt = (fixed_dt as f64) / 10.0;
    let ref_samples = ode.integrate(scenario.theta0, ref_dt, duration as f64);

    assert!(
        !ref_samples.is_empty(),
        "analytical ODE should produce samples"
    );

    let last_ref = ref_samples.last().unwrap();
    let sep_theta_deg = last_ref.theta.to_degrees();
    eprintln!(
        "with friction {friction}: wall separation at θ={sep_theta_deg:.2}°, t={:.4}s",
        last_ref.time
    );

    // Friction should delay separation: the box stays in contact longer
    // than the frictionless case (~0.48s).
    assert!(
        last_ref.time > 0.48,
        "friction should delay wall separation (t={:.4}s vs ~0.48s frictionless)",
        last_ref.time
    );

    // Wider tolerance than frictionless: iterative friction solving
    // accumulates error over the much longer contact phase.
    compare_against_ode(&run, &ref_samples, fixed_dt, 0.2);
    assert_no_wall_slide_jitter(&run.samples, fixed_dt, last_ref.time);

    // Compare against the frictionless reference to verify friction has
    // the expected qualitative effect: later separation and larger angle.
    let ode_frictionless = LadderOde::new(scenario.half_extents, 9.81);
    let ref_frictionless = ode_frictionless.integrate(scenario.theta0, ref_dt, duration as f64);
    let sep_time_frictionless = ref_frictionless.last().unwrap().time;

    assert!(
        last_ref.time > sep_time_frictionless * 1.5,
        "friction should significantly delay separation: \
         t_friction={:.4}s vs t_frictionless={:.4}s",
        last_ref.time,
        sep_time_frictionless
    );
    assert!(
        last_ref.theta > ref_frictionless.last().unwrap().theta,
        "friction should increase separation angle"
    );

    // The box must not fall through the floor.
    let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
    assert!(min_y > -0.1, "box fell through floor: min_y={min_y:.4}");
}
