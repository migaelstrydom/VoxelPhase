//! The parts of a compound, the bonds that hold them together and the ties
//! that hold them up: a plain graph with no ECS in it, so the rules about
//! what breaks off and what stays are tested on their own.
//!
//! ```text
//!   Part ── colliders that never come apart (a plank, a pane cell)
//!    │
//!   Bond ── two parts stuck together until a blast on what it tears off
//!    │       passes `threshold` or a contact spike felt at `anchor` passes
//!    │       `toughness`; the body's rigidity is what holds them
//!   Tie ─── a constraint holding the compound by one part
//!
//!   groups(): parts still connected through bonds
//!   kept_group(): the tied group, else the heaviest
//! ```

use nalgebra::{Isometry3, Point3};
use smallvec::SmallVec;

use crate::physics::{ColliderHandle, ConstraintHandle};

/// Names a part for its whole life: assigned when the part is made, never
/// reused within a graph and the graphs split off it, never an index into
/// anything.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PartId(u32);

/// One unit of a compound that never comes apart: a plank, a pane cell, a
/// frame of stone.
#[derive(Debug, Clone)]
pub struct Part {
    pub id: PartId,
    /// Its colliders, which keep their handles for the part's whole life,
    /// whichever body they are on.
    pub colliders: SmallVec<[ColliderHandle; 4]>,
}

/// Two parts on one body, stuck together until the load on what the bond
/// would tear off passes what it can hold (`fracture::load`).
#[derive(Debug, Clone, Copy)]
pub struct Bond {
    pub a: PartId,
    pub b: PartId,
    /// Where the parts meet, in the body's frame. Moved with the body's
    /// origin whenever it moves (a re-centre, a split), as constraint
    /// anchors are.
    pub anchor: Point3<f32>,
    /// How far from the anchor a contact spike still counts fully, m.
    pub reach: f32,
    /// Blast loading, N·s/m², past which it breaks: at the centre of mass
    /// of what it tears off, or at the anchor for a bond in a cycle.
    pub threshold: f32,
    /// Contact spike felt at the anchor, N·s, past which it breaks. `None`
    /// takes the compound's default
    /// ([`CompoundFracture::breaking_on_impact_at`](super::CompoundFracture::breaking_on_impact_at));
    /// with neither, it never breaks on contact.
    pub toughness: Option<f32>,
}

impl Bond {
    /// Whether this bond holds `part`.
    pub fn holds(&self, part: PartId) -> bool {
        self.a == part || self.b == part
    }
}

/// A constraint holding the compound by one of its parts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tie {
    pub constraint: ConstraintHandle,
    pub part: PartId,
}

/// A piece cut out of a part, for [`PartGraph::replace`]: its colliders and
/// where its centre is in the body's frame.
pub struct NewPart {
    pub colliders: SmallVec<[ColliderHandle; 4]>,
    pub centre: Point3<f32>,
    /// Whether it may inherit the old part's bonds and ties: false for a
    /// piece the cutter knocks out at once, whose bonds would only be cut.
    pub inherits: bool,
}

/// A compound's parts, bonds and ties.
#[derive(Debug, Clone, Default)]
pub struct PartGraph {
    parts: Vec<Part>,
    bonds: Vec<Bond>,
    ties: Vec<Tie>,
    /// The next id to hand out.
    next: u32,
}

impl PartGraph {
    /// Add a part of `colliders` and name it.
    pub fn add_part(&mut self, colliders: impl IntoIterator<Item = ColliderHandle>) -> PartId {
        let id = PartId(self.next);
        self.next += 1;
        self.parts.push(Part {
            id,
            colliders: colliders.into_iter().collect(),
        });
        id
    }

    /// Stick two parts together.
    pub fn bond(&mut self, bond: Bond) {
        self.bonds.push(bond);
    }

    /// Record that a constraint holds the compound by `part`.
    pub fn tie(&mut self, constraint: ConstraintHandle, part: PartId) {
        self.ties.push(Tie { constraint, part });
    }

    /// Forget a tie, returning the part it held.
    pub fn untie(&mut self, constraint: ConstraintHandle) -> Option<PartId> {
        let place = self.ties.iter().position(|t| t.constraint == constraint)?;
        Some(self.ties.remove(place).part)
    }

    /// Forget every tie holding `part`, returning their constraints.
    pub fn untie_part(&mut self, part: PartId) -> Vec<ConstraintHandle> {
        let (gone, kept): (Vec<Tie>, Vec<Tie>) = self.ties.iter().partition(|tie| tie.part == part);
        self.ties = kept;
        gone.into_iter().map(|tie| tie.constraint).collect()
    }

    pub fn parts(&self) -> &[Part] {
        &self.parts
    }

    pub fn part(&self, id: PartId) -> Option<&Part> {
        self.parts.iter().find(|part| part.id == id)
    }

    pub fn bonds(&self) -> &[Bond] {
        &self.bonds
    }

    pub fn ties(&self) -> &[Tie] {
        &self.ties
    }

    /// The part `collider` belongs to.
    pub fn part_of(&self, collider: ColliderHandle) -> Option<PartId> {
        self.parts
            .iter()
            .find(|part| part.colliders.contains(&collider))
            .map(|part| part.id)
    }

    /// Every collider of `parts`.
    pub fn colliders_of(&self, parts: &[PartId]) -> Vec<ColliderHandle> {
        parts
            .iter()
            .filter_map(|id| self.part(*id))
            .flat_map(|part| part.colliders.iter().copied())
            .collect()
    }

    /// The constraints of the ties holding `parts`.
    pub fn ties_of(&self, parts: &[PartId]) -> Vec<ConstraintHandle> {
        self.ties
            .iter()
            .filter(|tie| parts.contains(&tie.part))
            .map(|tie| tie.constraint)
            .collect()
    }

    /// Drop every bond `breaks` says has given way; returns how many did.
    pub fn break_bonds(&mut self, mut breaks: impl FnMut(&Bond) -> bool) -> usize {
        let before = self.bonds.len();
        self.bonds.retain(|bond| !breaks(bond));
        before - self.bonds.len()
    }

    /// Drop every bond holding `part`, so it comes away at the next split.
    pub fn cut_loose(&mut self, part: PartId) {
        self.bonds.retain(|bond| !bond.holds(part));
    }

    /// Give up everything that holds the compound together or up: every
    /// bond goes, and every tie, whose constraints are returned for the
    /// caller to remove.
    pub fn release(&mut self) -> Vec<ConstraintHandle> {
        self.bonds.clear();
        self.ties.drain(..).map(|tie| tie.constraint).collect()
    }

    /// The parts still joined to each other through bonds, each group in the
    /// order of `parts`, the groups in the order of their first part.
    pub fn groups(&self) -> Vec<Vec<PartId>> {
        let place = |id: PartId| self.parts.iter().position(|part| part.id == id);
        let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); self.parts.len()];
        for bond in &self.bonds {
            if let (Some(a), Some(b)) = (place(bond.a), place(bond.b)) {
                adjacent[a].push(b);
                adjacent[b].push(a);
            }
        }
        let mut visited = vec![false; self.parts.len()];
        let mut groups = Vec::new();
        for start in 0..self.parts.len() {
            if visited[start] {
                continue;
            }
            let mut group = Vec::new();
            let mut stack = vec![start];
            while let Some(node) = stack.pop() {
                if std::mem::replace(&mut visited[node], true) {
                    continue;
                }
                group.push(node);
                stack.extend(adjacent[node].iter().filter(|n| !visited[**n]));
            }
            group.sort_unstable();
            groups.push(group.into_iter().map(|node| self.parts[node].id).collect());
        }
        groups
    }

    /// Which of `groups` stays on the compound's own body: the heaviest of
    /// those holding ties, or the heaviest of all if none does. `weight`
    /// says what a part weighs. `None` only for no groups.
    pub fn kept_group(
        &self,
        groups: &[Vec<PartId>],
        weight: impl Fn(PartId) -> f32,
    ) -> Option<usize> {
        let tied = |group: &Vec<PartId>| self.ties.iter().any(|tie| group.contains(&tie.part));
        let heaviest = |candidates: &mut dyn Iterator<Item = usize>| {
            candidates.max_by(|a, b| {
                let weigh = |g: usize| groups[g].iter().map(|p| weight(*p)).sum::<f32>();
                weigh(*a).total_cmp(&weigh(*b))
            })
        };
        heaviest(&mut (0..groups.len()).filter(|g| tied(&groups[*g])))
            .or_else(|| heaviest(&mut (0..groups.len())))
    }

    /// The graph of `parts` alone: those parts, the bonds among them and
    /// their ties. Ids go on from where this graph's do, so neither graph
    /// names a new part as one the other has.
    pub fn share(&self, parts: &[PartId]) -> Self {
        Self {
            parts: self
                .parts
                .iter()
                .filter(|part| parts.contains(&part.id))
                .cloned()
                .collect(),
            bonds: self
                .bonds
                .iter()
                .filter(|bond| parts.contains(&bond.a) && parts.contains(&bond.b))
                .copied()
                .collect(),
            ties: self
                .ties
                .iter()
                .filter(|tie| parts.contains(&tie.part))
                .copied()
                .collect(),
            next: self.next,
        }
    }

    /// Re-express the anchors of the bonds among `parts` through `frame`: a
    /// point `p` in the old body frame becomes `frame * p` in the new one, as
    /// when those parts move to a body of their own or their body's origin
    /// moves.
    pub fn transform_anchors(&mut self, parts: &[PartId], frame: &Isometry3<f32>) {
        for bond in &mut self.bonds {
            if parts.contains(&bond.a) && parts.contains(&bond.b) {
                bond.anchor = frame * bond.anchor;
            }
        }
    }

    /// Replace `part` with `pieces`, a part each. Every bond and tie of the
    /// old part goes to the inheriting new part nearest its anchor
    /// (`tie_anchor` says where a tie's constraint holds the body), or goes
    /// if none inherits; the bonds among the new parts are the caller's to
    /// make. Returns the new parts' ids in the
    /// order given, or `None` if there is no such part.
    pub fn replace(
        &mut self,
        part: PartId,
        pieces: Vec<NewPart>,
        tie_anchor: impl Fn(ConstraintHandle) -> Option<Point3<f32>>,
    ) -> Option<Vec<PartId>> {
        let place = self.parts.iter().position(|p| p.id == part)?;
        self.parts.remove(place);
        let heirs: Vec<(Point3<f32>, bool)> = pieces
            .iter()
            .map(|piece| (piece.centre, piece.inherits))
            .collect();
        let ids: Vec<PartId> = pieces
            .into_iter()
            .map(|piece| self.add_part(piece.colliders))
            .collect();
        let nearest = |at: Point3<f32>| -> Option<PartId> {
            heirs
                .iter()
                .zip(&ids)
                .filter(|((_, inherits), _)| *inherits)
                .min_by(|a, b| {
                    (a.0 .0 - at)
                        .norm_squared()
                        .total_cmp(&(b.0 .0 - at).norm_squared())
                })
                .map(|(_, id)| *id)
        };

        let mut bonds = std::mem::take(&mut self.bonds);
        bonds.retain_mut(|bond| {
            if !bond.holds(part) {
                return true;
            }
            let Some(heir) = nearest(bond.anchor) else {
                return false;
            };
            if bond.a == part {
                bond.a = heir;
            }
            if bond.b == part {
                bond.b = heir;
            }
            bond.a != bond.b
        });
        self.bonds = bonds;

        let mut ties = std::mem::take(&mut self.ties);
        ties.retain_mut(|tie| {
            if tie.part != part {
                return true;
            }
            match tie_anchor(tie.constraint).and_then(nearest) {
                Some(heir) => {
                    tie.part = heir;
                    true
                }
                None => false,
            }
        });
        self.ties = ties;
        Some(ids)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use generational_arena::Arena;

    fn collider(slot: usize) -> ColliderHandle {
        ColliderHandle::for_test(slot)
    }

    fn constraint(slot: usize) -> ConstraintHandle {
        let mut arena = Arena::new();
        let mut last = None;
        for _ in 0..=slot {
            last = Some(arena.insert(()));
        }
        ConstraintHandle(last.unwrap())
    }

    fn bond(a: PartId, b: PartId, anchor: Point3<f32>) -> Bond {
        Bond {
            a,
            b,
            anchor,
            reach: 1.0,
            threshold: 1.0,
            toughness: None,
        }
    }

    /// Four parts in a row, `0 - 1 - 2 - 3`, bonded at their midpoints.
    fn row() -> (PartGraph, [PartId; 4]) {
        let mut graph = PartGraph::default();
        let ids = [0, 1, 2, 3].map(|i| graph.add_part([collider(i)]));
        for pair in ids.windows(2) {
            graph.bond(bond(pair[0], pair[1], Point3::origin()));
        }
        (graph, ids)
    }

    #[test]
    fn a_bonded_row_is_one_group_until_a_bond_breaks() {
        let (mut graph, [a, b, c, d]) = row();
        assert_eq!(graph.groups(), vec![vec![a, b, c, d]]);

        graph.break_bonds(|bond| bond.holds(b) && bond.holds(c));
        assert_eq!(graph.groups(), vec![vec![a, b], vec![c, d]]);
    }

    /// A part of many colliders is one node: its colliders never part.
    #[test]
    fn a_part_of_many_colliders_stays_one() {
        let mut graph = PartGraph::default();
        let branch = graph.add_part([collider(0), collider(1), collider(2)]);
        let twig = graph.add_part([collider(3)]);
        assert_eq!(graph.groups(), vec![vec![branch], vec![twig]]);
        assert_eq!(graph.part_of(collider(1)), Some(branch));
        assert_eq!(
            graph.colliders_of(&[branch]),
            vec![collider(0), collider(1), collider(2)]
        );
    }

    #[test]
    fn without_ties_the_heaviest_group_stays() {
        let (mut graph, [a, b, c, d]) = row();
        graph.cut_loose(a);
        let groups = graph.groups();
        assert_eq!(groups, vec![vec![a], vec![b, c, d]]);

        let weights = |heavy: PartId| move |p: PartId| if p == heavy { 100.0 } else { 1.0 };
        assert_eq!(graph.kept_group(&groups, weights(a)), Some(0));
        assert_eq!(graph.kept_group(&groups, weights(c)), Some(1));
    }

    /// The tied group stays even when it is the lighter one, and of several
    /// tied groups the heaviest.
    #[test]
    fn a_tied_group_stays_however_light() {
        let (mut graph, [a, b, c, d]) = row();
        graph.tie(constraint(0), d);
        graph.break_bonds(|bond| bond.holds(c) && bond.holds(d));
        let groups = graph.groups();
        assert_eq!(graph.kept_group(&groups, |_| 1.0), Some(1));

        graph.tie(constraint(1), a);
        let weight = |p: PartId| if p == b { 10.0 } else { 1.0 };
        assert_eq!(graph.kept_group(&groups, weight), Some(0));
    }

    /// Release gives up every bond and tie, so every part is a group.
    #[test]
    fn release_drops_every_bond_and_hands_back_every_tie() {
        let (mut graph, [a, b, c, d]) = row();
        graph.tie(constraint(0), a);
        graph.tie(constraint(1), d);

        let constraints = graph.release();

        assert_eq!(constraints, vec![constraint(0), constraint(1)]);
        assert!(graph.ties().is_empty());
        assert_eq!(graph.groups(), vec![vec![a], vec![b], vec![c], vec![d]]);
    }

    /// A share holds its parts, the bonds among them and their ties, and
    /// nothing that reaches across.
    #[test]
    fn a_share_takes_its_parts_bonds_and_ties() {
        let (mut graph, [a, b, c, d]) = row();
        graph.tie(constraint(0), d);
        let share = graph.share(&[c, d]);

        assert_eq!(share.parts().len(), 2);
        assert_eq!(share.bonds().len(), 1);
        assert!(share.bonds()[0].holds(c) && share.bonds()[0].holds(d));
        assert_eq!(
            share.ties(),
            &[Tie {
                constraint: constraint(0),
                part: d
            }]
        );
        let _ = (a, b);
    }

    /// Ids a share hands out are never ids its source already handed out.
    #[test]
    fn a_share_names_new_parts_afresh() {
        let (graph, ids) = row();
        let mut share = graph.share(&ids[2..]);
        let new = share.add_part([collider(9)]);
        assert!(!ids.contains(&new));
    }

    /// Anchors follow the frame their parts are in, and only theirs.
    #[test]
    fn anchors_follow_the_body_frame() {
        let (mut graph, [a, b, c, d]) = row();
        graph.cut_loose(b);
        graph.transform_anchors(&[c, d], &Isometry3::translation(1.0, 0.0, 0.0));
        let moved: Vec<_> = graph.bonds().iter().map(|bond| bond.anchor).collect();
        assert_eq!(moved, vec![Point3::new(1.0, 0.0, 0.0)]);
        graph.transform_anchors(&[a, b], &Isometry3::translation(0.0, 5.0, 0.0));
        assert_eq!(graph.bonds()[0].anchor, Point3::new(1.0, 0.0, 0.0));
    }

    /// A cutter's pieces take the old part's bonds and ties, each to the
    /// piece nearest where it holds.
    #[test]
    fn a_replaced_part_passes_each_bond_and_tie_to_the_nearest_piece() {
        let mut graph = PartGraph::default();
        let left = graph.add_part([collider(0)]);
        let block = graph.add_part([collider(1)]);
        let right = graph.add_part([collider(2)]);
        graph.bond(bond(left, block, Point3::new(-1.0, 0.0, 0.0)));
        graph.bond(bond(block, right, Point3::new(1.0, 0.0, 0.0)));
        graph.tie(constraint(0), block);

        let piece = |slot: usize, x: f32, inherits: bool| NewPart {
            colliders: [collider(slot)].into_iter().collect(),
            centre: Point3::new(x, 0.0, 0.0),
            inherits,
        };
        let pieces = vec![
            piece(3, -0.5, true),
            piece(4, 0.5, true),
            piece(5, 0.9, false),
        ];
        let ids = graph
            .replace(block, pieces, |_| Some(Point3::new(0.4, -0.5, 0.0)))
            .unwrap();

        assert!(graph.part(block).is_none());
        assert!(graph.bonds()[0].holds(left) && graph.bonds()[0].holds(ids[0]));
        assert!(graph.bonds()[1].holds(right) && graph.bonds()[1].holds(ids[1]));
        assert_eq!(graph.ties()[0].part, ids[1]);
        // The pieces are not bonded to each other until the cutter says so,
        // and the one knocked out inherits nothing, however near.
        assert_eq!(graph.groups().len(), 3);
        assert!(graph.bonds().iter().all(|bond| !bond.holds(ids[2])));
    }
}
