//! Cracking a fragment into pieces, the way rock breaks: a few seeds spread
//! over it, each piece grown from one through the fragment's own samples.
//!
//! ```text
//!   samples ──▶ seeds: farthest-point sampling, so pieces come out of a size
//!           │
//!           ▼
//!   every seed grows at once through face neighbours, each step costing
//!   a little more or less at random (Dijkstra), so seams wander like cracks
//!           │
//!           ▼
//!   pieces: each connected, each a patch of the fragment
//! ```
//!
//! Growing through the fragment, not across space, is what keeps a piece in
//! one piece: three stalactites under a deck are never one piece, since no
//! path joins them but through the deck, which another seed claims first. A
//! sample no seed reaches (joined to the rest only at an edge or a corner) is
//! a piece of its own.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

/// Splits a set of lattice samples into connected pieces.
#[derive(Debug, Clone, Copy)]
pub struct Cracker {
    /// How much one step of a crack may cost over another: 0 grows round
    /// pieces with straight seams, more makes them ragged.
    pub roughness: f32,
}

impl Default for Cracker {
    fn default() -> Self {
        Self { roughness: 1.5 }
    }
}

impl Cracker {
    /// `samples` cracked into `pieces` connected pieces (fewer if there are
    /// fewer samples; more where some are joined to the rest only at an
    /// edge or a corner).
    pub fn crack(&self, samples: &[[usize; 3]], pieces: usize) -> Vec<Vec<[usize; 3]>> {
        let index: HashMap<[usize; 3], usize> =
            samples.iter().enumerate().map(|(i, &s)| (s, i)).collect();
        let seeds = spread(samples, pieces.max(1));

        // Dijkstra from every seed at once; costs are scaled to integers so
        // the heap orders them exactly and the result does not depend on
        // float ties.
        let mut owner = vec![usize::MAX; samples.len()];
        let mut cost = vec![u64::MAX; samples.len()];
        let mut heap = BinaryHeap::new();
        for (piece, &seed) in seeds.iter().enumerate() {
            cost[seed] = 0;
            owner[seed] = piece;
            heap.push(Reverse((0u64, seed)));
        }
        while let Some(Reverse((at_cost, at))) = heap.pop() {
            if at_cost > cost[at] {
                continue;
            }
            for n in face_neighbours(samples[at]) {
                let Some(&next) = index.get(&n) else {
                    continue;
                };
                let step = (1000.0 * (1.0 + self.roughness * noise(n))) as u64;
                let through = at_cost + step;
                if through < cost[next] {
                    cost[next] = through;
                    owner[next] = owner[at];
                    heap.push(Reverse((through, next)));
                }
            }
        }

        let mut out: Vec<Vec<[usize; 3]>> = vec![Vec::new(); seeds.len()];
        for (i, &piece) in owner.iter().enumerate() {
            if piece == usize::MAX {
                out.push(vec![samples[i]]);
            } else {
                out[piece].push(samples[i]);
            }
        }
        out.retain(|piece| !piece.is_empty());
        out
    }
}

/// `count` of `samples`' indices spread as far apart as they go: the first
/// is the lowest sample, each next the one farthest from all chosen so far.
fn spread(samples: &[[usize; 3]], count: usize) -> Vec<usize> {
    if samples.is_empty() {
        return Vec::new();
    }
    let first = (0..samples.len()).min_by_key(|&i| samples[i]).unwrap_or(0);
    let mut seeds = vec![first];
    let mut nearest: Vec<usize> = samples
        .iter()
        .map(|s| distance2(*s, samples[first]))
        .collect();
    while seeds.len() < count.min(samples.len()) {
        let far = (0..samples.len())
            .max_by_key(|&i| (nearest[i], Reverse(i)))
            .unwrap_or(0);
        if nearest[far] == 0 {
            break;
        }
        seeds.push(far);
        for (i, s) in samples.iter().enumerate() {
            nearest[i] = nearest[i].min(distance2(*s, samples[far]));
        }
    }
    seeds
}

fn distance2(a: [usize; 3], b: [usize; 3]) -> usize {
    (0..3).map(|k| a[k].abs_diff(b[k]).pow(2)).sum()
}

/// The six face neighbours of `s` that have non-negative indices.
fn face_neighbours(s: [usize; 3]) -> impl Iterator<Item = [usize; 3]> {
    (0..3).flat_map(move |axis| {
        [s[axis].checked_sub(1), s[axis].checked_add(1)]
            .into_iter()
            .flatten()
            .map(move |i| {
                let mut n = s;
                n[axis] = i;
                n
            })
    })
}

/// A fixed value in [0, 1) for a sample: how hard it is to crack through.
fn noise(s: [usize; 3]) -> f32 {
    let mut h = (s[0] as u32).wrapping_mul(0x9E37_79B1)
        ^ (s[1] as u32).wrapping_mul(0x85EB_CA77)
        ^ (s[2] as u32).wrapping_mul(0xC2B2_AE3D);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2C1B_3C6D);
    h ^= h >> 12;
    (h >> 8) as f32 / (1u32 << 24) as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn connected(piece: &[[usize; 3]]) -> bool {
        let set: HashSet<[usize; 3]> = piece.iter().copied().collect();
        let mut seen = HashSet::from([piece[0]]);
        let mut stack = vec![piece[0]];
        while let Some(s) = stack.pop() {
            for n in face_neighbours(s) {
                if set.contains(&n) && seen.insert(n) {
                    stack.push(n);
                }
            }
        }
        seen.len() == piece.len()
    }

    /// A hemispherical shell two samples thick cracks into patches of about
    /// one size, each in one piece, taking every sample once; none is a ring
    /// round the whole dome.
    #[test]
    fn a_dome_cracks_into_patches() {
        let samples: Vec<[usize; 3]> = (0..26)
            .flat_map(|x| (0..14).flat_map(move |y| (0..26).map(move |z| [x, y, z])))
            .filter(|&[x, y, z]| {
                let d =
                    ((x as f32 - 12.5).powi(2) + (y as f32).powi(2) + (z as f32 - 12.5).powi(2))
                        .sqrt();
                (10.0..12.0).contains(&d)
            })
            .collect();
        let pieces = Cracker::default().crack(&samples, 8);
        assert_eq!(pieces.len(), 8);
        let total: usize = pieces.iter().map(Vec::len).sum();
        assert_eq!(total, samples.len());
        for piece in &pieces {
            assert!(connected(piece), "a piece is in several parts");
            let span = |a: usize| {
                piece.iter().map(|s| s[a]).max().unwrap()
                    - piece.iter().map(|s| s[a]).min().unwrap()
            };
            assert!(span(0) < 22 || span(2) < 22, "a piece rings the dome");
        }
        let sizes: Vec<usize> = pieces.iter().map(Vec::len).collect();
        let (lo, hi) = (sizes.iter().min().unwrap(), sizes.iter().max().unwrap());
        assert!(*hi < 4 * *lo, "pieces of very different sizes: {sizes:?}");
    }

    /// Three stalactites under a deck: however the deck is cracked, no
    /// piece holds two stalactites without the deck between them.
    #[test]
    fn pieces_are_never_joined_by_air() {
        let mut samples = Vec::new();
        for x in 0..16 {
            for z in 0..4 {
                samples.push([x, 10, z]);
                samples.push([x, 11, z]);
            }
        }
        for cx in [1, 7, 13] {
            for y in 3..10 {
                for x in cx..cx + 2 {
                    for z in 1..3 {
                        samples.push([x, y, z]);
                    }
                }
            }
        }
        for count in 2..6 {
            for piece in Cracker::default().crack(&samples, count) {
                assert!(connected(&piece), "{count} pieces: one is joined by air");
            }
        }
    }
}
