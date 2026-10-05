//! Which pieces a cut split off, found by racing searches out of it.
//!
//! Cutting a graph can split a component in two. To find the pieces without
//! walking the big one, start a search on each side of the cut, lowest sample
//! first (terrain is held up from below, so a search finds its anchor soonest),
//! and advance them alternately, one node each (Even and Shiloach's
//! decremental-connectivity trick). Searches that meet are one piece, and
//! merge. A search that runs out of nodes has closed off a whole piece, the
//! smaller of what it was racing, having walked that piece and nothing more.
//! The last one still running is the largest, and needs no walking at all.
//!
//! ```text
//!   seeds (nodes beside the cut)
//!     │  one search each
//!     ▼
//!   round-robin, one node per search per round
//!     ├─ two searches touch ──▶ merge (union-find)
//!     ├─ a search reaches an anchor ──▶ anchored: held, stops
//!     └─ a search's frontier empties ──▶ closed
//!     ▼
//!   stop: nothing running, or one search running and none anchored
//! ```
//!
//! An anchor (a sample no charge removes) outweighs any size, so while one
//! search is anchored the others run until they close, meet, anchor or exhaust
//! the budget. Without an anchor, the last search running is the largest piece
//! and the race stops there. Whatever is still running when the budget runs out
//! is neither closed nor known to be held, and the caller decides.
//!
//! The graph is the lattice, 6-connected, and `node` says what each sample is,
//! so the race knows nothing of densities or materials.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;

/// What one lattice sample is to the race.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Node {
    /// Not part of the graph.
    Off,
    /// Part of the graph.
    On,
    /// Part of the graph, and holds up whatever reaches it.
    Anchor,
}

/// A piece a search closed off: every node of one component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct ClosedPiece {
    /// Lowest lattice index of the piece on each axis.
    pub lo: [i32; 3],
    /// Highest lattice index of the piece on each axis.
    pub hi: [i32; 3],
    /// How many nodes it has.
    pub samples: usize,
}

/// How a race ended.
#[derive(Debug, Clone, Default)]
pub(super) struct RaceOutcome {
    /// Every piece some search closed off.
    pub closed: Vec<ClosedPiece>,
    /// Nodes visited over the whole race.
    pub visited: usize,
    /// Whether the race stopped on its budget with more than one search still
    /// running, or one running beside an anchored one.
    pub out_of_budget: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Running,
    Closed,
    Anchored,
}

/// A sample waiting to be visited, ordered so the lowest comes out first,
/// and of equals the first in.
type Waiting = (Reverse<i32>, Reverse<u64>, [i32; 3]);

/// One search, or several merged into one.
struct Search {
    /// Lowest first: terrain is held up from below, so a search heads for
    /// what holds it before spreading through what it holds up.
    frontier: BinaryHeap<Waiting>,
    samples: usize,
    lo: [i32; 3],
    hi: [i32; 3],
    state: State,
}

impl Search {
    fn starting_at(seed: [i32; 3], node: Node) -> Self {
        Self {
            frontier: BinaryHeap::from([waiting(seed, 0)]),
            samples: 1,
            lo: seed,
            hi: seed,
            state: if node == Node::Anchor {
                State::Anchored
            } else {
                State::Running
            },
        }
    }

    fn take(&mut self, sample: [i32; 3], node: Node, order: u64) {
        self.frontier.push(waiting(sample, order));
        self.samples += 1;
        self.stretch(sample, sample);
        if node == Node::Anchor {
            self.state = State::Anchored;
        }
    }

    /// Grow the bounds to take in the box from `lo` to `hi`.
    fn stretch(&mut self, lo: [i32; 3], hi: [i32; 3]) {
        self.lo = std::array::from_fn(|axis| self.lo[axis].min(lo[axis]));
        self.hi = std::array::from_fn(|axis| self.hi[axis].max(hi[axis]));
    }

    /// Fold `other` into this search.
    fn absorb(&mut self, other: Search) {
        debug_assert!(
            other.state != State::Closed && self.state != State::Closed,
            "a closed search walked its whole component, so nothing can meet it"
        );
        self.frontier.extend(other.frontier);
        self.samples += other.samples;
        self.stretch(other.lo, other.hi);
        if other.state == State::Anchored {
            self.state = State::Anchored;
        }
    }
}

/// Searches with union-find over their ids: a merged search lives at its root.
struct Searches {
    parent: Vec<usize>,
    searches: Vec<Option<Search>>,
}

impl Searches {
    fn root(&mut self, mut id: usize) -> usize {
        while self.parent[id] != id {
            self.parent[id] = self.parent[self.parent[id]];
            id = self.parent[id];
        }
        id
    }

    fn get(&mut self, id: usize) -> &mut Search {
        let root = self.root(id);
        self.searches[root]
            .as_mut()
            .expect("every root holds its search")
    }

    /// Merge the searches rooted at `a` and `b`; the root of the result.
    fn merge(&mut self, a: usize, b: usize) -> usize {
        let a_size = self.searches[a].as_ref().map_or(0, |s| s.frontier.len());
        let b_size = self.searches[b].as_ref().map_or(0, |s| s.frontier.len());
        let (keep, fold) = if a_size >= b_size { (a, b) } else { (b, a) };
        let folded = self.searches[fold].take().expect("a root holds its search");
        self.parent[fold] = keep;
        self.searches[keep]
            .as_mut()
            .expect("a root holds its search")
            .absorb(folded);
        keep
    }

    fn roots_in(&self, state: State) -> impl Iterator<Item = usize> + '_ {
        self.searches
            .iter()
            .enumerate()
            .filter_map(move |(id, s)| s.as_ref().filter(|s| s.state == state).map(|_| id))
    }
}

/// Race a search from every seed that is part of the graph, visiting at most
/// `budget` nodes.
pub(super) fn race(
    seeds: impl IntoIterator<Item = [i32; 3]>,
    node: impl Fn([i32; 3]) -> Node,
    budget: usize,
) -> RaceOutcome {
    let mut owner: FxHashMap<[i32; 3], usize> = FxHashMap::default();
    let mut all = Searches {
        parent: Vec::new(),
        searches: Vec::new(),
    };
    for seed in seeds {
        let kind = node(seed);
        if kind == Node::Off || owner.contains_key(&seed) {
            continue;
        }
        let id = all.searches.len();
        owner.insert(seed, id);
        all.parent.push(id);
        all.searches.push(Some(Search::starting_at(seed, kind)));
    }

    let mut visited = 0;
    let mut order = 0;
    let mut out_of_budget = false;
    // A search stops running by closing, anchoring or being merged into
    // another, and never starts again, so the running set only shrinks; and
    // an anchored search stays anchored, merged or not.
    let mut running: Vec<usize> = all.roots_in(State::Running).collect();
    let mut anchored = all.roots_in(State::Anchored).next().is_some();
    loop {
        running.retain(|&id| all.parent[id] == id && all.get(id).state == State::Running);
        if running.is_empty() || (running.len() == 1 && !anchored) {
            break;
        }
        if visited >= budget {
            out_of_budget = true;
            break;
        }
        for &id in &running {
            if all.root(id) != id || all.get(id).state != State::Running {
                continue;
            }
            let Some((_, _, sample)) = all.get(id).frontier.pop() else {
                all.get(id).state = State::Closed;
                continue;
            };
            visited += 1;
            for next in neighbours(sample) {
                let here = all.root(id);
                match owner.get(&next) {
                    Some(&other) => {
                        let there = all.root(other);
                        if there != here {
                            all.merge(here, there);
                        }
                    }
                    None => {
                        let kind = node(next);
                        if kind != Node::Off {
                            owner.insert(next, here);
                            order += 1;
                            all.get(here).take(next, kind, order);
                        }
                    }
                }
            }
            let search = all.get(id);
            if search.state == State::Running && search.frontier.is_empty() {
                search.state = State::Closed;
            }
            anchored |= search.state == State::Anchored;
        }
    }

    let closed = all
        .searches
        .iter()
        .flatten()
        .filter(|s| s.state == State::Closed)
        .map(|s| ClosedPiece {
            lo: s.lo,
            hi: s.hi,
            samples: s.samples,
        })
        .collect();
    RaceOutcome {
        closed,
        visited,
        out_of_budget,
    }
}

/// `sample` in a frontier, taken `order`th.
fn waiting(sample: [i32; 3], order: u64) -> Waiting {
    (Reverse(sample[1]), Reverse(order), sample)
}

/// The six samples sharing a face with `c`.
fn neighbours(c: [i32; 3]) -> impl Iterator<Item = [i32; 3]> {
    (0..3).flat_map(move |axis| {
        [-1, 1].map(|d| {
            let mut n = c;
            n[axis] += d;
            n
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn inside(lo: [i32; 3], hi: [i32; 3], c: [i32; 3]) -> bool {
        (0..3).all(|a| lo[a] <= c[a] && c[a] <= hi[a])
    }

    /// Two boxes, a gap between them on x: a small one and a big one.
    fn two_boxes(c: [i32; 3]) -> Node {
        if inside([0, 0, 0], [1, 1, 1], c) || inside([3, 0, 0], [40, 10, 10], c) {
            Node::On
        } else {
            Node::Off
        }
    }

    #[test]
    fn seeds_in_one_piece_close_nothing() {
        let r = race([[3, 0, 0], [10, 5, 5]], two_boxes, 1_000_000);
        assert!(r.closed.is_empty());
        assert!(!r.out_of_budget);
    }

    #[test]
    fn the_smaller_side_of_a_cut_closes_off() {
        let r = race([[1, 0, 0], [3, 0, 0]], two_boxes, 1_000_000);
        assert_eq!(
            r.closed,
            vec![ClosedPiece {
                lo: [0, 0, 0],
                hi: [1, 1, 1],
                samples: 8
            }]
        );
    }

    /// The race walks the small piece and stops: it never walks the big one.
    #[test]
    fn the_largest_piece_is_never_walked() {
        let r = race([[1, 0, 0], [3, 0, 0]], two_boxes, 1_000_000);
        assert!(r.visited < 30, "visited {}", r.visited);
    }

    /// Bedrock under the big box: an anchored search is held, so the other
    /// runs to the end even though it is the larger of the two.
    #[test]
    fn beside_an_anchor_even_the_largest_piece_closes() {
        let node = |c: [i32; 3]| {
            if inside([0, 0, 0], [0, 0, 0], c) {
                Node::Anchor
            } else if inside([0, 1, 0], [0, 1, 0], c) || inside([3, 0, 0], [20, 10, 10], c) {
                Node::On
            } else {
                Node::Off
            }
        };
        let r = race([[0, 1, 0], [3, 0, 0]], node, 1_000_000);
        assert_eq!(r.closed.len(), 1);
        assert_eq!(r.closed[0].samples, 18 * 11 * 11);
    }

    #[test]
    fn a_race_stops_on_its_budget() {
        let endless = |c: [i32; 3]| {
            if c[1] == 0 && c[2] == 0 {
                Node::On
            } else {
                Node::Off
            }
        };
        let node = |c: [i32; 3]| if c[0] == 0 { Node::Off } else { endless(c) };
        let r = race([[-1, 0, 0], [1, 0, 0]], node, 100);
        assert!(r.out_of_budget);
        assert!(r.closed.is_empty());
    }

    #[test]
    fn searches_that_meet_merge_and_close_as_one() {
        let ring = |c: [i32; 3]| {
            let on =
                c[2] == 0 && inside([0, 0, 0], [4, 4, 0], c) && !inside([1, 1, 0], [3, 3, 0], c);
            if on {
                Node::On
            } else {
                Node::Off
            }
        };
        let big = |c: [i32; 3]| {
            if inside([10, 0, 0], [60, 20, 20], c) {
                Node::On
            } else {
                ring(c)
            }
        };
        let r = race([[0, 0, 0], [4, 4, 0], [10, 0, 0]], big, 1_000_000);
        assert_eq!(r.closed.len(), 1);
        assert_eq!(r.closed[0].samples, 16);
    }
}
