//! The volume ledger: where every cubic metre of water came from and went.
//!
//! ```text
//!   Σ finite store volumes = V₀ + emitted − sunk − lost + ocean_in − ocean_out − discarded
//! ```
//!
//! Every movement of volume, by a link or by a topology change, goes through
//! [`VolumeLedger::transfer`]: debited from one account and credited to the
//! other with the same f64. That is the only place conservation can break,
//! and [`VolumeLedger::check`] says whether it did.

use std::fmt;

use crate::water::ids::StoreId;

/// Relative error at which the ledger is judged unbalanced.
pub const LEDGER_TOLERANCE: f64 = 1e-6;

/// The accounts volume can move between that are not finite stores.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Account {
    /// A finite store: a basin or a reach.
    Store(StoreId),
    /// An endless source: springs and sky sources.
    Emitted,
    /// An authored sink or open world edge.
    Sunk,
    /// Minor water lost to the ground and air (§7.7).
    Lost,
    /// The ocean.
    Ocean,
    /// Water with nowhere to go yet: above an outlet crest before outlets have
    /// links. Zero from stage 4a on.
    Discarded,
}

/// Running totals of every movement through the ledger.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VolumeLedger {
    /// Volume held in finite stores when the ledger opened.
    pub initial: f64,
    pub emitted: f64,
    pub sunk: f64,
    pub lost: f64,
    /// Volume the ocean gave to the stores.
    pub ocean_in: f64,
    /// Volume the stores gave to the ocean.
    pub ocean_out: f64,
    pub discarded: f64,
}

/// What the ledger balance says about the stores' current volume.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Balance {
    /// Σ finite store volume the ledger expects.
    pub expected: f64,
    /// Σ finite store volume actually held.
    pub held: f64,
}

impl Balance {
    /// Absolute imbalance, m³.
    pub fn error(&self) -> f64 {
        self.held - self.expected
    }

    /// Imbalance relative to the larger of the two totals (1 m³ at least,
    /// so an empty network is not judged on rounding).
    pub fn relative_error(&self) -> f64 {
        self.error().abs() / self.expected.abs().max(self.held.abs()).max(1.0)
    }

    pub fn is_balanced(&self) -> bool {
        self.relative_error() <= LEDGER_TOLERANCE
    }
}

impl fmt::Display for Balance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "held {:.6} m³, expected {:.6} m³ ({:+.3e})",
            self.held,
            self.expected,
            self.error()
        )
    }
}

impl VolumeLedger {
    /// A ledger opened on `initial` m³ of water in finite stores.
    pub fn open(initial: f64) -> Self {
        Self {
            initial,
            ..Self::default()
        }
    }

    /// Book a movement of `volume` from one account to another. Finite
    /// stores' own volumes are moved by the caller with the same number.
    pub fn transfer(&mut self, from: Account, to: Account, volume: f64) {
        self.book(from, -volume);
        self.book(to, volume);
    }

    /// Book volume entering the finite stores through the initial balance:
    /// water placed at load, before the ledger's first tick.
    pub fn place(&mut self, volume: f64) {
        self.initial += volume;
    }

    fn book(&mut self, account: Account, delta: f64) {
        match account {
            Account::Store(_) => {}
            // Volume leaving an endless source counts up what it emitted.
            Account::Emitted => self.emitted -= delta,
            Account::Sunk => self.sunk += delta,
            Account::Lost => self.lost += delta,
            // Volume credited to the ocean left the stores for it.
            Account::Ocean => {
                if delta > 0.0 {
                    self.ocean_out += delta;
                } else {
                    self.ocean_in -= delta;
                }
            }
            Account::Discarded => self.discarded += delta,
        }
    }

    /// The finite store total the ledger expects.
    pub fn expected(&self) -> f64 {
        self.initial + self.emitted - self.sunk - self.lost + self.ocean_in
            - self.ocean_out
            - self.discarded
    }

    /// Compare the ledger with what the stores hold.
    pub fn check(&self, held: f64) -> Balance {
        Balance {
            expected: self.expected(),
            held,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_transfer_between_stores_keeps_the_balance() {
        let mut ledger = VolumeLedger::open(10.0);
        ledger.transfer(Account::Store(StoreId(0)), Account::Store(StoreId(1)), 4.0);
        assert!(ledger.check(10.0).is_balanced());
    }

    #[test]
    fn losses_and_sources_move_the_expected_total() {
        let mut ledger = VolumeLedger::open(10.0);
        ledger.transfer(Account::Emitted, Account::Store(StoreId(0)), 3.0);
        ledger.transfer(Account::Store(StoreId(0)), Account::Lost, 1.0);
        ledger.transfer(Account::Store(StoreId(0)), Account::Discarded, 0.5);
        ledger.transfer(Account::Store(StoreId(0)), Account::Ocean, 2.0);
        assert_eq!(ledger.expected(), 10.0 + 3.0 - 1.0 - 0.5 - 2.0);
        assert!(!ledger.check(10.0).is_balanced());
    }
}
