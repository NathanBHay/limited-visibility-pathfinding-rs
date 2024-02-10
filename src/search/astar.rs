//! # A-Star Search
//! An A-Star Implementation with drop-in heuristic, expander, and domain.
//! The implementation is similar to the approach used by
//! [Warthog](https://bitbucket.org/dharabor/pathfinding/src/master/), however
//! is not as optimized and lacks choice of a queue.

use super::{BestSearch, Search, SearchNode};
use std::{
    collections::{BinaryHeap, HashMap},
    hash::Hash,
    ops::Add, sync::Arc,
};

/// A-Star Search
pub struct AStar<N, C>
where
    N: Hash + Clone + Eq + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    pub heuristic: Arc<dyn Fn(&N) -> C + Sync + Send>,
}

impl <N, C> AStar<N, C>
where
    N: Hash + Clone + Eq + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    /// Create a new A-Star search
    pub fn new(heuristic: Arc<dyn Fn(&N) -> C + Sync + Send>) -> Self {
        AStar { heuristic }
    }
}

impl<N, C> Search<N, C> for AStar<N, C>
where
    N: Hash + Clone + Eq + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    fn _search<E, I, G>(
        &self,
        mut expander: E,
        start: N,
        goal: G,
    ) -> (HashMap<N, (Option<N>, C)>, Option<N>)
    where
        E: FnMut(&N) -> I,
        I: IntoIterator<Item = (N, C)>,
        G: Fn(&N) -> bool,
    {
        let mut open = BinaryHeap::from([SearchNode {
            node: start.clone(),
            cost: (self.heuristic)(&start),
        }]);
        let mut previous = HashMap::new();
        previous.insert(start.clone(), (None, C::default()));
        while let Some(SearchNode { node, .. }) = open.pop() {
            if goal(&node) {
                return (previous, Some(node));
            }
            for (child, cost) in expander(&node) {
                let new_cost = previous[&node].1.clone() + cost;
                if !previous.contains_key(&child) || new_cost < previous[&child].1 {
                    previous.insert(child.clone(), (Some(node.clone()), new_cost.clone()));
                    open.push(SearchNode {
                        node: child.clone(),
                        cost: new_cost.clone() + (self.heuristic)(&child),
                    });
                }
            }
        }
        (previous, None)
    }
}

impl<N, C> BestSearch<N, C> for AStar<N, C>
where
    N: Hash + Eq + Clone + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    fn get_best_heuristic(&self) -> &Arc<dyn Fn(&N) -> C + Sync + Send> {
        &self.heuristic
    }
}

#[cfg(test)]
mod tests {
    use super::AStar;
    use crate::{domains::{bitpackedgrid::BitPackedGrid, DomainCreate}, search::Search};
    use std::sync::Arc;

    #[test]
    fn test_astar() {
        let results = AStar::new(
            Arc::new(|x| *x),
        ).search(|x| vec![(x + 1, 1), (x + 2, 2)], 0, |x| *x == 2);
        assert_eq!(results.unwrap().0, vec![0, 2]);
    }

    #[test]
    fn test_astar_bitpacked_grid() {
        let grid = BitPackedGrid::new_from_string(".....\n.###.\n.#...\n.#.#.\n...#.".to_string());
        let path = AStar::new(
            Arc::new(|_| 0),
        ).search(
            |(x, y)| {
                grid.adjacent((x.clone(), y.clone()), false)
                    .map(|(x, y)| ((x, y), 1))
            },
            (0, 4),
            |(x, y)| *x == 4 && *y == 2,
        );
        assert_eq!(
            path.unwrap().0,
            vec![(0, 4), (1, 4), (2, 4), (2, 3), (2, 2), (3, 2), (4, 2)]
        );
    }

    #[test]
    fn test_astar_bitpacked_grid_with_heuristic() {
        let grid = BitPackedGrid::new_from_string(
            "........\n...###..\n.....#..\n.....#..\n........\n........".to_string(),
        );
        let path = AStar::new(
            Arc::new(|_| 0),
        ).search(
            |(x, y)| {
                grid.adjacent((x.clone(), y.clone()), false)
                    .map(|(x, y)| ((x, y), 1))
            },
            (0, 5),
            |(x, y)| *x == 7 && *y == 0,
        );
        assert_eq!(
            path.unwrap().0,
            vec![
                (0, 5),
                (1, 5),
                (2, 5),
                (3, 5),
                (4, 5),
                (5, 5),
                (6, 5),
                (6, 4),
                (7, 4),
                (7, 3),
                (7, 2),
                (7, 1),
                (7, 0)
            ]
        );
    }

}
