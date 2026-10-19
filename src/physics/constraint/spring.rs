//! A spring stated by frequency and damping ratio, as the soft row's terms.
//!
//! A soft `ConstraintRow` solves
//! `λ = -(J·v + bias + s·λ_acc) / (K + s)`, with `K = J·M⁻¹·Jᵀ`. That is Box2D
//! v3's soft step: its mass scale `a₂/(1 + a₂)` is `K/(K + s)`, its impulse
//! scale `1/(1 + a₂)` is `s/(K + s)`, with `s = K/a₂`, and its bias rate is
//! carried in `bias`. A compliant joint fills those terms from a compliance;
//! a [`Spring`] fills them from `(ω, ζ)` and the mass the stiffness is stated
//! against.
//!
//! ```text
//!   Spring (ω, ζ) ──┐
//!   reference mass ─┼──► SoftStep { softness, bias_rate } ──► ConstraintRow
//!   substep dt ─────┘      s = 1/(h·m·ω·(2ζ + hω))           softness = s
//!                          bias_rate = ω/(2ζ + hω)            bias = bias_rate·C
//! ```
//!
//! The terms are the implicit step of `k = m·ω²`, `c = 2·m·ζ·ω` (Catto's soft
//! constraints): `s = 1/(h·(c + h·k))` and `bias_rate = k/(c + h·k)`. With the
//! reference mass equal to the row's own `1/K` they are exactly Box2D's. They
//! depend only on the substep length, so a row whose bias is refreshed every
//! substep behaves the same at any number of substeps per frame.

/// A damped spring, by its natural frequency and damping ratio.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Spring {
    /// Natural angular frequency `ω`, in rad/s, against the reference mass:
    /// the stiffness is `m·ω²`.
    pub angular_frequency: f32,
    /// Damping ratio `ζ`: `1.0` is critical, `0.0` undamped.
    pub damping_ratio: f32,
}

/// A soft row's two spring terms for one substep length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SoftStep {
    /// The row's softness `s`, which softens its effective mass to
    /// `1/(K + s)` and feeds its accumulated impulse back into the solve.
    pub softness: f32,
    /// Bias per unit of position error, in 1/s.
    pub bias_rate: f32,
}

impl Spring {
    /// A spring of `frequency_hz` cycles per second.
    pub fn from_hz(frequency_hz: f32, damping_ratio: f32) -> Self {
        Self {
            angular_frequency: std::f32::consts::TAU * frequency_hz,
            damping_ratio,
        }
    }

    /// Stiffness `k = m·ω²` against `reference_mass`, in N/m.
    pub fn stiffness(&self, reference_mass: f32) -> f32 {
        reference_mass * self.angular_frequency * self.angular_frequency
    }

    /// The soft row terms that make a row this spring over a substep of `dt`,
    /// with its stiffness stated against `reference_mass`.
    ///
    /// A spring with no frequency or no mass is no spring: its terms are a
    /// rigid row's, zero softness and zero bias rate.
    pub fn soft_step(&self, reference_mass: f32, dt: f32) -> SoftStep {
        let omega = self.angular_frequency;
        if omega <= 0.0 || reference_mass <= 0.0 || dt <= 0.0 {
            return SoftStep {
                softness: 0.0,
                bias_rate: 0.0,
            };
        }
        let a1 = 2.0 * self.damping_ratio + dt * omega;
        SoftStep {
            softness: 1.0 / (dt * reference_mass * omega * a1),
            bias_rate: omega / a1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Against the row's own effective mass the terms are Box2D v3's softness
    /// triple: mass scale `a₂/(1 + a₂)` and impulse scale `1/(1 + a₂)`.
    #[test]
    fn against_the_rows_own_mass_the_terms_are_box2ds() {
        let spring = Spring::from_hz(4.0, 0.7);
        let dt = 1.0 / 240.0;
        let mass = 3.5;
        let k = 1.0 / mass;
        let step = spring.soft_step(mass, dt);

        let omega = spring.angular_frequency;
        let a1 = 2.0 * spring.damping_ratio + dt * omega;
        let a2 = dt * omega * a1;
        let mass_scale = k / (k + step.softness);
        let impulse_scale = step.softness / (k + step.softness);
        assert!((mass_scale - a2 / (1.0 + a2)).abs() < 1e-6);
        assert!((impulse_scale - 1.0 / (1.0 + a2)).abs() < 1e-6);
        assert!((step.bias_rate - omega / a1).abs() < 1e-6);
    }

    #[test]
    fn no_frequency_is_a_rigid_row() {
        let step = Spring::from_hz(0.0, 1.0).soft_step(10.0, 1.0 / 240.0);
        assert_eq!(step.softness, 0.0);
        assert_eq!(step.bias_rate, 0.0);
    }
}
