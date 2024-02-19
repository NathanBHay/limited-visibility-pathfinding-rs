use std::{ops::Add, sync::Arc};

use crate::{
    domains::samplegrids::samplegrid2d::SampleGrid2d,
    search::{astar::AStar, Search},
};
use ordered_float::OrderedFloat;

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord)]
struct ProbabilityNode(OrderedFloat<f32>, usize);
impl Add for ProbabilityNode {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self(self.0 + other.0, self.1 + other.1)
    }
}
impl Default for ProbabilityNode {
    fn default() -> Self {
        Self(OrderedFloat(0.0), 0)
    }
}

pub fn compute_probability(
    grid: &SampleGrid2d,
    goal: (usize, usize),
) -> Arc<dyn Fn(&(usize, usize)) -> usize + Send + Sync> {
    let astar = AStar::new(Arc::new(|_| ProbabilityNode(OrderedFloat(0.0), 0)));
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
        x
    })
}
