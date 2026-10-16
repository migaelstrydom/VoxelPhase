//! Throws a character has committed to and not yet let go of.
//!
//! A throw is decided on the click and leaves the hand once the arm has come
//! over: the wind-up. Gameplay owns that moment; animation reads the strokes
//! to swing the arm around it. Throws can overlap, since a cooldown can be
//! shorter than a wind-up, so each stroke keeps its own clock, and they
//! alternate hands.
//!
//! A thrower faces its throw: from the click, through the release and for
//! [`FACE_THROW_AFTER`] beyond it, `facing` asks the body to turn to the aim.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

/// Seconds a thrower keeps facing its throw after letting go: the
/// follow-through, carried in the direction thrown.
pub const FACE_THROW_AFTER: f32 = 0.25;

/// Which hand a throw is made with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThrowingHand {
    Left,
    Right,
}

impl ThrowingHand {
    /// The other hand.
    pub fn other(self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Left,
        }
    }
}

/// One throw, from the click until it leaves the hand.
#[derive(Debug, Clone, Copy)]
pub struct ThrowStroke {
    /// Distinguishes this throw from every other the character has made.
    /// Rises by one per throw.
    pub id: u32,
    pub hand: ThrowingHand,
    /// Seconds since the throw was committed to.
    pub elapsed: f32,
    /// Seconds from the commit to the release.
    pub release_at: f32,
    /// Horizontal direction the throw is aimed, at the commit.
    pub aim: Vector3<f32>,
}

/// A character's throws in progress.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct Throws {
    /// Throws not yet released, oldest first.
    strokes: Vec<ThrowStroke>,
    /// Throws released by the last `advance`.
    released: Vec<ThrowStroke>,
    /// The hand the next throw is made with.
    next_hand: ThrowingHand,
    /// The id the next throw gets.
    next_id: u32,
    /// The aim of the throw last let go of, and how much longer to face it.
    after: Option<(Vector3<f32>, f32)>,
}

impl Default for Throws {
    fn default() -> Self {
        Self {
            strokes: Vec::new(),
            released: Vec::new(),
            next_hand: ThrowingHand::Right,
            next_id: 0,
            after: None,
        }
    }
}

impl Throws {
    /// Advance every stroke by `dt`, then commit to a new throw if `commit`
    /// is given: aimed along it, and leaving the hand `release_at` seconds
    /// from now. Returns the strokes released this frame.
    ///
    /// A throw committed this frame is not advanced by this frame's `dt`: it
    /// is released on the first frame at least `release_at` after the click,
    /// and with no delay it is released at once.
    pub fn advance(
        &mut self,
        dt: f32,
        commit: Option<Vector3<f32>>,
        release_at: f32,
    ) -> Vec<ThrowStroke> {
        for stroke in &mut self.strokes {
            stroke.elapsed += dt;
        }
        self.after = self
            .after
            .map(|(aim, left)| (aim, left - dt))
            .filter(|&(_, left)| left > 0.0);
        if let Some(aim) = commit {
            self.strokes.push(ThrowStroke {
                id: self.next_id,
                hand: self.next_hand,
                elapsed: 0.0,
                release_at,
                aim,
            });
            self.next_id = self.next_id.wrapping_add(1);
            self.next_hand = self.next_hand.other();
        }
        let (released, pending) = self
            .strokes
            .iter()
            .partition(|stroke| stroke.elapsed >= stroke.release_at);
        self.strokes = pending;
        self.released = released;
        if let Some(last) = self.released.last() {
            self.after = Some((last.aim, FACE_THROW_AFTER));
        }
        self.released.clone()
    }

    /// Throws winding up, oldest first.
    pub fn strokes(&self) -> &[ThrowStroke] {
        &self.strokes
    }

    /// Which way the thrower should face, while it is throwing: the newest
    /// throw in hand, or the one just let go of.
    pub fn facing(&self) -> Option<Vector3<f32>> {
        self.strokes
            .last()
            .map(|stroke| stroke.aim)
            .or(self.after.map(|(aim, _)| aim))
    }

    /// Every throw in hand this frame: those winding up and those just let
    /// go, which a throw with no wind-up only ever is.
    pub fn this_frame(&self) -> impl Iterator<Item = &ThrowStroke> {
        self.released.iter().chain(&self.strokes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// Frames from the click until the throw is released, the click's frame
    /// being frame zero.
    fn frames_to_release(delay: f32) -> usize {
        let mut throws = Throws::default();
        (0..100)
            .find(|&frame| {
                let commit = (frame == 0).then(Vector3::z);
                !throws.advance(DT, commit, delay).is_empty()
            })
            .expect("the throw is released")
    }

    /// The body turns to the throw from the click and keeps facing it
    /// through the follow-through, then is its own again.
    #[test]
    fn a_thrower_faces_its_throw_until_the_follow_through_is_done() {
        let mut throws = Throws::default();
        let aim = Vector3::x();
        throws.advance(DT, Some(aim), 0.1);
        assert_eq!(throws.facing(), Some(aim));
        let frames = ((0.1 + FACE_THROW_AFTER) / DT) as usize;
        for _ in 0..frames - 2 {
            throws.advance(DT, None, 0.1);
        }
        assert_eq!(throws.facing(), Some(aim), "still following through");
        for _ in 0..4 {
            throws.advance(DT, None, 0.1);
        }
        assert_eq!(throws.facing(), None);
    }

    #[test]
    fn no_delay_throws_on_the_click() {
        assert_eq!(frames_to_release(0.0), 0);
    }

    /// At 60 Hz, 0.12 s is the eighth frame after the click.
    #[test]
    fn a_throw_leaves_on_the_first_frame_its_wind_up_is_over() {
        assert_eq!(frames_to_release(0.12), 8);
    }

    #[test]
    fn overlapping_throws_keep_their_own_clocks_and_alternate_hands() {
        let mut throws = Throws::default();
        let delay = 3.5 * DT;
        let released: Vec<Vec<ThrowingHand>> = (0..8)
            .map(|frame| {
                let commit = (frame < 2).then(Vector3::z);
                throws
                    .advance(DT, commit, delay)
                    .iter()
                    .map(|s| s.hand)
                    .collect()
            })
            .collect();
        let expected: Vec<Vec<ThrowingHand>> = vec![
            vec![],
            vec![],
            vec![],
            vec![],
            vec![ThrowingHand::Right],
            vec![ThrowingHand::Left],
            vec![],
            vec![],
        ];
        assert_eq!(released, expected);
        assert!(throws.strokes().is_empty());
    }
}
