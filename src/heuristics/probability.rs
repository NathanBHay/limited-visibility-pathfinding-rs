use std::{ops::Add, sync::Arc};

use crate::{
    domains::samplegrids::samplegrid2d::SampleGrid2d,
    search::{astar::AStar, Search},
};
use ordered_float::OrderedFloat;

/// A cost node that stores the probability and a distance count
#[derive(Clone, Default, PartialEq, Eq, PartialOrd, Ord, Debug)]
struct ProbabilityNode(OrderedFloat<f32>, usize);
impl Add for ProbabilityNode {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0, self.1 + other.1)
    }
}

/// Compute the probability of coming landing upon this node. Used to get next best 
/// node to explore if the goal is not reachable.
pub fn compute_probability(
    grid: &SampleGrid2d,
    goal: (usize, usize),
) -> Arc<dyn Fn(&(usize, usize)) -> (usize, usize) + Send + Sync> { // \/ Below |_| 0 is bad design
    let astar = AStar::new(Arc::new(|_| ProbabilityNode::default()), Arc::new(|_| 0));
    let (map, _) = astar._search(
        |n| {
            grid.adjacent_probs(*n, false)
                .into_iter()
                .map(|(n, f)| (n, ProbabilityNode(f, 1)))
        },
        goal,
        |_| false,
    );
    Arc::new(move |n| {
        let x = (map[n].1 .0 * 1000.0).0 as usize;
        (x, map[n].1 .1)
        // x * map[n].1 .1 // This here is a hack to match with the current design
                        // A better approach would be to factor for different
                        // return type.
    })
}
