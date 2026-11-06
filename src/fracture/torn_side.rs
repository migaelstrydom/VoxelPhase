//! What each bond of a compound would tear off if it alone gave way: the
//! side a bond holds on, by which it is judged.
//!
//! ```text
//!   PartGraph ──▶ bridges of the part graph, parallel bonds merged
//!                 (Tarjan: one depth-first pass)
//!                   │
//!                   ▼  per bridge: the side without a tie, else the lighter
//!   TornSides ── per pair of parts: the torn-off side's run in visit order,
//!                its mass and centre of mass (prefix sums, never listed)
//! ```
//!
//! A bond in a cycle frees nothing on its own and has no torn-off side.

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;

use super::parts::{Bond, PartGraph, PartId};
use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// The side of a bond that comes away if it gives way.
#[derive(Debug, Clone, Copy)]
pub struct TornSide {
    /// Its parts' run in the visit order.
    run: Run,
    /// What it weighs, kg.
    pub mass: f32,
    /// Its centre of mass, in the frame the part centres were given in.
    pub centre: Point3<f32>,
}

/// A side's parts as positions in the visit order: one depth-first subtree,
/// or its component with that subtree taken out.
#[derive(Debug, Clone, Copy)]
enum Run {
    Within {
        start: usize,
        end: usize,
    },
    Around {
        component: (usize, usize),
        hole: (usize, usize),
    },
}

impl Run {
    fn contains(&self, at: usize) -> bool {
        let inside = |(start, end): (usize, usize)| (start..end).contains(&at);
        match *self {
            Self::Within { start, end } => inside((start, end)),
            Self::Around { component, hole } => inside(component) && !inside(hole),
        }
    }
}

/// The torn-off side of every bond of a graph that has one.
#[derive(Debug, Clone, Default)]
pub struct TornSides {
    /// Each part's position in the visit order.
    visited_at: FxHashMap<PartId, usize>,
    /// The torn-off side of each pair of parts joined by a bridge, keyed by
    /// the pair in order.
    sides: FxHashMap<(PartId, PartId), TornSide>,
}

impl TornSides {
    /// The torn-off sides of `graph`'s bonds. `mass_of` says what a part
    /// weighs and where its centre of mass is; a part it has no answer for
    /// weighs nothing.
    pub fn of(graph: &PartGraph, mass_of: impl Fn(PartId) -> Option<(f32, Point3<f32>)>) -> Self {
        let search = BridgeSearch::of(graph);
        let order = &search.order;

        // Running sums over the visit order: mass, mass moment and ties.
        let tied: Vec<bool> = order
            .iter()
            .map(|id| graph.ties().iter().any(|tie| tie.part == *id))
            .collect();
        let mut mass = vec![0.0f32; order.len() + 1];
        let mut moment = vec![Vector3::zeros(); order.len() + 1];
        let mut ties = vec![0usize; order.len() + 1];
        for (k, id) in order.iter().enumerate() {
            let (m, centre) = mass_of(*id).unwrap_or((0.0, Point3::origin()));
            mass[k + 1] = mass[k] + m;
            moment[k + 1] = moment[k] + centre.coords * m;
            ties[k + 1] = ties[k] + usize::from(tied[k]);
        }
        let sum = |(start, end): (usize, usize)| {
            (
                mass[end] - mass[start],
                moment[end] - moment[start],
                ties[end] - ties[start],
            )
        };

        let mut sides = FxHashMap::default();
        for bridge in &search.bridges {
            let (sub_mass, sub_moment, sub_ties) = sum(bridge.subtree);
            let (all_mass, all_moment, all_ties) = sum(bridge.component);
            let (rest_mass, rest_moment, rest_ties) = (
                all_mass - sub_mass,
                all_moment - sub_moment,
                all_ties - sub_ties,
            );
            let subtree_torn = match (sub_ties > 0, rest_ties > 0) {
                (false, true) => true,
                (true, false) => false,
                _ => sub_mass <= rest_mass,
            };
            let (run, mass, moment) = if subtree_torn {
                let (start, end) = bridge.subtree;
                (Run::Within { start, end }, sub_mass, sub_moment)
            } else {
                let run = Run::Around {
                    component: bridge.component,
                    hole: bridge.subtree,
                };
                (run, rest_mass, rest_moment)
            };
            if mass <= 0.0 {
                continue;
            }
            sides.insert(
                bridge.pair,
                TornSide {
                    run,
                    mass,
                    centre: Point3::from(moment / mass),
                },
            );
        }
        let visited_at = order.iter().enumerate().map(|(k, id)| (*id, k)).collect();
        Self { visited_at, sides }
    }

    /// The torn-off sides of `graph`, the compound on `body`, with each
    /// part weighed and placed in the world by its colliders.
    pub fn on_body(world: &PhysicsWorld, body: RigidBodyHandle, graph: &PartGraph) -> Self {
        Self::of(graph, |part| {
            world.mass_centre_of(body, &graph.colliders_of(&[part]))
        })
    }

    /// What `bond` tears off if it gives way; `None` for a bond in a cycle,
    /// which frees nothing on its own, or one whose side weighs nothing.
    pub fn of_bond(&self, bond: &Bond) -> Option<&TornSide> {
        self.sides.get(&pair(bond.a, bond.b))
    }

    /// Whether `side` holds `part`.
    pub fn holds(&self, side: &TornSide, part: PartId) -> bool {
        self.visited_at
            .get(&part)
            .is_some_and(|at| side.run.contains(*at))
    }
}

/// Two parts as a key, in order.
fn pair(a: PartId, b: PartId) -> (PartId, PartId) {
    (a.min(b), a.max(b))
}

/// A bridge found by the search: the pair it joins, the subtree beyond it
/// and the component it is in, as runs of the visit order.
struct Bridge {
    pair: (PartId, PartId),
    subtree: (usize, usize),
    component: (usize, usize),
}

/// Tarjan's bridge search over a part graph whose parallel bonds count as
/// one edge.
struct BridgeSearch {
    /// The parts in the order the search first reached them.
    order: Vec<PartId>,
    bridges: Vec<Bridge>,
}

impl BridgeSearch {
    fn of(graph: &PartGraph) -> Self {
        let parts = graph.parts();
        let place: FxHashMap<PartId, usize> =
            parts.iter().enumerate().map(|(i, p)| (p.id, i)).collect();
        let mut pairs: Vec<(usize, usize)> = graph
            .bonds()
            .iter()
            .filter_map(|bond| Some((*place.get(&bond.a)?, *place.get(&bond.b)?)))
            .filter(|(a, b)| a != b)
            .map(|(a, b)| (a.min(b), a.max(b)))
            .collect();
        pairs.sort_unstable();
        pairs.dedup();
        let mut adjacent: Vec<Vec<(usize, usize)>> = vec![Vec::new(); parts.len()];
        for (edge, (a, b)) in pairs.iter().enumerate() {
            adjacent[*a].push((*b, edge));
            adjacent[*b].push((*a, edge));
        }

        const UNSEEN: usize = usize::MAX;
        let mut discovered = vec![UNSEEN; parts.len()];
        let mut low = vec![0usize; parts.len()];
        let mut order = Vec::with_capacity(parts.len());
        let mut bridges = Vec::new();
        for root in 0..parts.len() {
            if discovered[root] != UNSEEN {
                continue;
            }
            let component_start = order.len();
            let first_bridge = bridges.len();
            // (node, the edge it was reached by, how many neighbours tried)
            let mut stack: Vec<(usize, usize, usize)> = vec![(root, UNSEEN, 0)];
            discovered[root] = order.len();
            low[root] = order.len();
            order.push(parts[root].id);
            while let Some(top) = stack.last_mut() {
                let (node, via, tried) = *top;
                if let Some(&(next, edge)) = adjacent[node].get(tried) {
                    top.2 += 1;
                    if edge == via {
                        continue;
                    }
                    if discovered[next] == UNSEEN {
                        discovered[next] = order.len();
                        low[next] = order.len();
                        order.push(parts[next].id);
                        stack.push((next, edge, 0));
                    } else {
                        low[node] = low[node].min(discovered[next]);
                    }
                    continue;
                }
                stack.pop();
                if let Some(&(parent, _, _)) = stack.last() {
                    low[parent] = low[parent].min(low[node]);
                    if low[node] > discovered[parent] {
                        bridges.push(Bridge {
                            pair: pair(parts[parent].id, parts[node].id),
                            subtree: (discovered[node], order.len()),
                            component: (0, 0),
                        });
                    }
                }
            }
            for bridge in &mut bridges[first_bridge..] {
                bridge.component = (component_start, order.len());
            }
        }
        Self { order, bridges }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{ColliderHandle, ConstraintHandle};
    use generational_arena::Arena;

    fn weld() -> ConstraintHandle {
        ConstraintHandle(Arena::new().insert(()))
    }

    fn bond(a: PartId, b: PartId) -> Bond {
        Bond {
            a,
            b,
            anchor: Point3::origin(),
            reach: 1.0,
            threshold: 1.0,
            toughness: None,
        }
    }

    /// `count` parts, each a collider of its own.
    fn parts(graph: &mut PartGraph, count: usize) -> Vec<PartId> {
        (0..count)
            .map(|i| graph.add_part([ColliderHandle::for_test(i)]))
            .collect()
    }

    /// The sides of `graph` with its parts weighing `masses`, centred one
    /// metre apart along x.
    fn sides(graph: &PartGraph, masses: &[f32]) -> TornSides {
        let ids: Vec<PartId> = graph.parts().iter().map(|p| p.id).collect();
        TornSides::of(graph, |id| {
            let i = ids.iter().position(|p| *p == id)?;
            Some((masses[i], Point3::new(i as f32, 0.0, 0.0)))
        })
    }

    fn held_parts(sides: &TornSides, bond: &Bond, all: &[PartId]) -> Vec<PartId> {
        let side = sides.of_bond(bond).expect("a torn-off side");
        all.iter()
            .copied()
            .filter(|p| sides.holds(side, *p))
            .collect()
    }

    /// A tied trunk with two branches off one fork and a twig on the first:
    /// each branch's bond tears off that branch and what grows on it, however
    /// heavy, and never the trunk.
    #[test]
    fn a_branch_tears_off_itself_and_what_grows_on_it() {
        let mut graph = PartGraph::default();
        let ids = parts(&mut graph, 4);
        let [trunk, first, second, twig] = [ids[0], ids[1], ids[2], ids[3]];
        graph.tie(weld(), trunk);
        let (to_first, to_second, to_twig) =
            (bond(trunk, first), bond(trunk, second), bond(first, twig));
        for b in [to_first, to_second, to_twig] {
            graph.bond(b);
        }
        let sides = sides(&graph, &[1.0, 50.0, 2.0, 30.0]);

        assert_eq!(held_parts(&sides, &to_first, &ids), vec![first, twig]);
        assert_eq!(held_parts(&sides, &to_second, &ids), vec![second]);
        assert_eq!(held_parts(&sides, &to_twig, &ids), vec![twig]);
        let side = sides.of_bond(&to_first).unwrap();
        assert_eq!(side.mass, 80.0);
        assert!((side.centre.x - (50.0 + 3.0 * 30.0) / 80.0).abs() < 1e-5);
    }

    /// Untied, a bridge tears off its lighter side, whichever way round the
    /// search found it.
    #[test]
    fn untied_the_lighter_side_comes_away() {
        let mut graph = PartGraph::default();
        let ids = parts(&mut graph, 3);
        let (left, right) = (bond(ids[0], ids[1]), bond(ids[1], ids[2]));
        graph.bond(left);
        graph.bond(right);

        let light_ends = sides(&graph, &[1.0, 5.0, 1.0]);
        assert_eq!(held_parts(&light_ends, &left, &ids), vec![ids[0]]);
        assert_eq!(held_parts(&light_ends, &right, &ids), vec![ids[2]]);

        let heavy_end = sides(&graph, &[10.0, 1.0, 1.0]);
        assert_eq!(held_parts(&heavy_end, &left, &ids), vec![ids[1], ids[2]]);
    }

    /// A bond in a cycle frees nothing on its own; one off the cycle still
    /// does, and parallel bonds between one pair are one connection.
    #[test]
    fn a_bond_in_a_cycle_tears_nothing_off() {
        let mut graph = PartGraph::default();
        let ids = parts(&mut graph, 4);
        let ring = [
            bond(ids[0], ids[1]),
            bond(ids[1], ids[2]),
            bond(ids[2], ids[0]),
        ];
        for b in ring {
            graph.bond(b);
        }
        // A pane held by its frame at two stones.
        let (stone, other_stone) = (bond(ids[0], ids[3]), bond(ids[3], ids[0]));
        graph.bond(stone);
        graph.bond(other_stone);
        let sides = sides(&graph, &[5.0, 5.0, 5.0, 1.0]);

        for b in &ring {
            assert!(sides.of_bond(b).is_none());
        }
        assert_eq!(held_parts(&sides, &stone, &ids), vec![ids[3]]);
        assert_eq!(held_parts(&sides, &other_stone, &ids), vec![ids[3]]);
    }

    /// Sides are found per group: a part in another group is never on a side.
    #[test]
    fn sides_stay_within_their_group() {
        let mut graph = PartGraph::default();
        let ids = parts(&mut graph, 4);
        let (first, second) = (bond(ids[0], ids[1]), bond(ids[2], ids[3]));
        graph.bond(first);
        graph.bond(second);
        let sides = sides(&graph, &[3.0, 1.0, 1.0, 3.0]);
        assert_eq!(held_parts(&sides, &first, &ids), vec![ids[1]]);
        assert_eq!(held_parts(&sides, &second, &ids), vec![ids[2]]);
    }
}
