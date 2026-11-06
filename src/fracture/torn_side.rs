//! What each bond of a compound would tear off if it alone gave way: the
//! side a bond holds on, by which it is judged.
//!
//! ```text
//!   PartGraph ──▶ its parts, parallel bonds merged, and one ground node
//!                 joined to every tied part (the world holds them all)
//!                   │  Tarjan: one depth-first pass finds the bridges
//!                   ▼  per bridge: the side away from the ground, else the
//!                      lighter
//!   TornSides ── per pair of parts: the torn-off side's run in visit order,
//!                its mass and centre of mass (prefix sums, never listed)
//! ```
//!
//! A bond in a cycle frees nothing on its own and has no torn-off side.
//! Two parts each tied to the world are joined through it, so a part bonded
//! to both hangs from either: neither bond is a bridge.

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
    Within(Span),
    Around { component: Span, hole: Span },
}

/// A run of positions in the visit order, `start..end`.
type Span = (usize, usize);

fn spans(span: Span, at: usize) -> bool {
    (span.0..span.1).contains(&at)
}

impl Run {
    fn contains(&self, at: usize) -> bool {
        match *self {
            Self::Within(span) => spans(span, at),
            Self::Around { component, hole } => spans(component, at) && !spans(hole, at),
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
        let parts = graph.parts();

        // Running sums of mass and mass moment over the visit order; the
        // ground weighs nothing.
        let mut mass = vec![0.0f32; search.order.len() + 1];
        let mut moment = vec![Vector3::zeros(); search.order.len() + 1];
        for (k, node) in search.order.iter().enumerate() {
            let (m, centre) = parts
                .get(*node)
                .and_then(|part| mass_of(part.id))
                .unwrap_or((0.0, Point3::origin()));
            mass[k + 1] = mass[k] + m;
            moment[k + 1] = moment[k] + centre.coords * m;
        }
        let sum = |(start, end): Span| (mass[end] - mass[start], moment[end] - moment[start]);
        let grounded = |span: Span| search.ground_at.is_some_and(|at| spans(span, at));

        let mut sides = FxHashMap::default();
        for bridge in &search.bridges {
            let (sub_mass, sub_moment) = sum(bridge.subtree);
            let (all_mass, all_moment) = sum(bridge.component);
            let (rest_mass, rest_moment) = (all_mass - sub_mass, all_moment - sub_moment);
            let subtree_torn = if grounded(bridge.component) {
                !grounded(bridge.subtree)
            } else {
                sub_mass <= rest_mass
            };
            let (run, mass, moment) = if subtree_torn {
                (Run::Within(bridge.subtree), sub_mass, sub_moment)
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
        let visited_at = search
            .order
            .iter()
            .enumerate()
            .filter_map(|(k, node)| Some((parts.get(*node)?.id, k)))
            .collect();
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

/// A bridge between two parts found by the search: the pair it joins, the
/// subtree beyond it and the component it is in, as runs of the visit
/// order.
struct Bridge {
    pair: (PartId, PartId),
    subtree: Span,
    component: Span,
}

/// Tarjan's bridge search over a part graph whose parallel bonds count as
/// one edge, with the ground as one more node, after the parts, joined to
/// every tied part.
struct BridgeSearch {
    /// The nodes in the order the search first reached them: a part's place
    /// in the graph's parts, or the ground's after them.
    order: Vec<usize>,
    /// Where the ground is in the visit order, if anything is tied.
    ground_at: Option<usize>,
    /// The bridges between two parts; one between a part and the ground is
    /// a tie, not a bond, and is left out.
    bridges: Vec<Bridge>,
}

impl BridgeSearch {
    fn of(graph: &PartGraph) -> Self {
        let parts = graph.parts();
        let ground = parts.len();
        let place: FxHashMap<PartId, usize> =
            parts.iter().enumerate().map(|(i, p)| (p.id, i)).collect();
        let bonded = graph
            .bonds()
            .iter()
            .filter_map(|bond| Some((*place.get(&bond.a)?, *place.get(&bond.b)?)));
        let tied = graph
            .ties()
            .iter()
            .filter_map(|tie| Some((*place.get(&tie.part)?, ground)));
        let mut edges: Vec<(usize, usize)> = bonded
            .chain(tied)
            .filter(|(a, b)| a != b)
            .map(|(a, b)| (a.min(b), a.max(b)))
            .collect();
        edges.sort_unstable();
        edges.dedup();
        let mut adjacent: Vec<Vec<(usize, usize)>> = vec![Vec::new(); parts.len() + 1];
        for (edge, (a, b)) in edges.iter().enumerate() {
            adjacent[*a].push((*b, edge));
            adjacent[*b].push((*a, edge));
        }

        const UNSEEN: usize = usize::MAX;
        let nodes = parts.len() + usize::from(!graph.ties().is_empty());
        let mut discovered = vec![UNSEEN; nodes];
        let mut low = vec![0usize; nodes];
        let mut order = Vec::with_capacity(nodes);
        let mut bridges = Vec::new();
        for root in 0..nodes {
            if discovered[root] != UNSEEN {
                continue;
            }
            let component_start = order.len();
            let first_bridge = bridges.len();
            // (node, the edge it was reached by, how many neighbours tried)
            let mut stack: Vec<(usize, usize, usize)> = vec![(root, UNSEEN, 0)];
            discovered[root] = order.len();
            low[root] = order.len();
            order.push(root);
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
                        order.push(next);
                        stack.push((next, edge, 0));
                    } else {
                        low[node] = low[node].min(discovered[next]);
                    }
                    continue;
                }
                stack.pop();
                if let Some(&(parent, _, _)) = stack.last() {
                    low[parent] = low[parent].min(low[node]);
                    if low[node] > discovered[parent] && parent != ground && node != ground {
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
        let ground_at = (nodes > parts.len()).then(|| discovered[ground]);
        Self {
            order,
            ground_at,
            bridges,
        }
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

    /// Two parts each tied to the world are joined through it: a part bonded
    /// to both hangs from either, so neither bond tears anything off. A part
    /// hanging from one of them alone still comes away, and the ground is
    /// never on a side.
    #[test]
    fn a_part_held_by_two_tied_parts_hangs_from_either() {
        let mut graph = PartGraph::default();
        let ids = parts(&mut graph, 4);
        let [left, pane, right, shard] = [ids[0], ids[1], ids[2], ids[3]];
        graph.tie(weld(), left);
        graph.tie(weld(), right);
        graph.tie(weld(), right);
        let (to_left, to_right, to_shard) =
            (bond(left, pane), bond(pane, right), bond(pane, shard));
        for b in [to_left, to_right, to_shard] {
            graph.bond(b);
        }
        let sides = sides(&graph, &[5.0, 1.0, 5.0, 0.5]);

        assert!(sides.of_bond(&to_left).is_none());
        assert!(sides.of_bond(&to_right).is_none());
        assert_eq!(held_parts(&sides, &to_shard, &ids), vec![shard]);
        assert_eq!(sides.of_bond(&to_shard).unwrap().mass, 0.5);
    }

    /// A tied part on the far side of a bridge keeps it: what comes away is
    /// the untied side, however heavy, and its mass leaves out the ground.
    #[test]
    fn the_side_away_from_the_ground_comes_away() {
        let mut graph = PartGraph::default();
        let ids = parts(&mut graph, 3);
        graph.tie(weld(), ids[2]);
        let (first, second) = (bond(ids[0], ids[1]), bond(ids[1], ids[2]));
        graph.bond(first);
        graph.bond(second);
        let sides = sides(&graph, &[40.0, 20.0, 1.0]);
        assert_eq!(held_parts(&sides, &first, &ids), vec![ids[0]]);
        assert_eq!(held_parts(&sides, &second, &ids), vec![ids[0], ids[1]]);
        assert_eq!(sides.of_bond(&second).unwrap().mass, 60.0);
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
