//! One fuzz case, drawn whole from a seed: a structure, and what disturbs it.
//!
//! Everything random is drawn here, up front, so a seed names a case exactly
//! and replays it. Nothing in the run draws from the generator.

use std::fmt;

use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use super::structure::{Structure, StructureKind};
use super::walker::WalkerScript;

/// A body of the structure given a velocity at the moment the case begins.
#[derive(Clone, Debug)]
pub struct Knock {
    /// Which of the structure's bodies, as a fraction through their list.
    pub body: f32,
    /// The velocity it is given, in m/s.
    pub velocity: Vector3<f32>,
}

/// A ball thrown at the structure.
#[derive(Clone, Debug)]
pub struct Projectile {
    pub radius: f32,
    /// In kg/m³: from foam to steel.
    pub density: f32,
    /// Compass bearing it comes from, in radians.
    pub bearing: f32,
    /// How high up the structure it is aimed, as a fraction of its height.
    pub height: f32,
    /// In m/s.
    pub speed: f32,
}

/// One case.
pub struct Case {
    pub seed: u64,
    pub structure: Structure,
    /// Display frame, in seconds: the game runs at 30 or 60 Hz.
    pub frame_dt: f32,
    /// Whether contacts are solved in sorted order. The game does not sort;
    /// the order changes what a chaotic collapse does, so both are drawn.
    pub ordered_contacts: bool,
    pub knocks: Vec<Knock>,
    pub projectile: Option<Projectile>,
    pub walker: Option<WalkerScript>,
}

impl Case {
    /// The case `seed` names, around a structure of `kind` if one is given.
    pub fn generate(seed: u64, kind: Option<StructureKind>) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);
        let kind =
            kind.unwrap_or_else(|| StructureKind::ALL[rng.gen_range(0..StructureKind::ALL.len())]);
        let structure = Structure::generate(kind, &mut rng);
        let frame_dt = if rng.gen_bool(0.5) {
            1.0 / 30.0
        } else {
            1.0 / 60.0
        };
        let ordered_contacts = rng.gen_bool(0.5);

        // At least one disturbance; usually the walker, as play does.
        let mut walker = rng.gen_bool(0.7).then(|| WalkerScript::generate(&mut rng));
        let projectile = rng.gen_bool(0.3).then(|| Projectile {
            radius: rng.gen_range(0.08..0.3),
            density: rng.gen_range(100.0..8000.0),
            bearing: rng.gen_range(0.0..std::f32::consts::TAU),
            height: rng.gen_range(0.1..0.9),
            speed: rng.gen_range(4.0..20.0),
        });
        let knocks = (0..rng.gen_range(0..=2))
            .map(|_| Knock {
                body: rng.gen_range(0.0..1.0),
                velocity: Vector3::new(
                    rng.gen_range(-3.0..3.0),
                    rng.gen_range(-1.0..3.0),
                    rng.gen_range(-3.0..3.0),
                ),
            })
            .collect::<Vec<_>>();
        if walker.is_none() && projectile.is_none() && knocks.is_empty() {
            walker = Some(WalkerScript::generate(&mut rng));
        }

        Self {
            seed,
            structure,
            frame_dt,
            ordered_contacts,
            knocks,
            projectile,
            walker,
        }
    }
}

impl fmt::Display for Case {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} at {:.0} Hz{}",
            self.structure,
            1.0 / self.frame_dt,
            if self.ordered_contacts {
                ", ordered"
            } else {
                ""
            }
        )?;
        if let Some(walker) = &self.walker {
            write!(f, "; {walker}")?;
        }
        if let Some(p) = &self.projectile {
            write!(
                f,
                "; ball r {:.2} m at {:.0} kg/m³, {:.0} m/s",
                p.radius, p.density, p.speed
            )?;
        }
        if !self.knocks.is_empty() {
            write!(f, "; {} knock(s)", self.knocks.len())?;
        }
        Ok(())
    }
}
