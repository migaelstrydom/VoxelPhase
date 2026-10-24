use std::cmp::Ordering;

/// A search node waiting to be expanded, in a `BinaryHeap` that pops the
/// cheapest first.
pub struct Pending {
    pub cost: f32,
    pub node: usize,
}

impl PartialEq for Pending {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Pending {}

impl PartialOrd for Pending {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Pending {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then(other.node.cmp(&self.node))
    }
}
