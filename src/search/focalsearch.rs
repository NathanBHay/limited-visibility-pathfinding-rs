use std::{
    collections::{BTreeMap, BinaryHeap},
    hash::Hash,
    ops::Add,
    sync::Arc,
};

use ahash::{AHashMap, AHashSet};
use rand::{rngs::SmallRng, Rng, SeedableRng};

use super::{BestSearch, Search, SearchNode};

/// Open list for Focal Search. Supports insertion, removal, and popping the best node
struct FSOpenList<N: Clone + Eq + Hash, C: Clone + Ord>(BTreeMap<C, AHashSet<N>>);

impl<N: Clone + Eq + Hash, C: Clone + Ord> FSOpenList<N, C> {
    fn insert(&mut self, node: N, cost: C) {
        self.0.entry(cost).or_insert_with(AHashSet::new).insert(node);
    }

    fn remove(&mut self, node: &N, cost: &C) -> bool {
        if let Some(nodes) = self.0.get_mut(cost) {
            nodes.remove(node);
            if nodes.is_empty() {
                self.0.remove(cost); // This may be slower as there will be
            } // cases where this map is recreated
            true
        } else {
            false
        }
    }

    fn peak(&self) -> Option<&C> {
        self.0.first_key_value().map(|(cost, _)| cost)
    }

    fn iter(&self) -> impl Iterator<Item = (&N, &C)> {
        self.0
            .iter()
            .flat_map(|(cost, nodes)| nodes.iter().map(move |node| (node, cost)))
    }
}

/// # Focal Search
/// Focal search is a bounded semi-admissible search. It operates similar to A*
/// but with the addition of a focal list. The focal list is a subset of the open
/// list that contains nodes within threshold of the current best node. This allows
/// the use of a second heuristic to further guide the search. This heuristic
/// doesn't need to be admissible.
/// ## Arguments
/// * `heuristic` - A function that returns the heuristic value of a given node
/// * `focal_heuristic` - A function tha evaluates nodes within a range of
/// (0, epsilon] of the best node on the open list
/// * ``focal_calc`` - A function that calculates epsilon combined with a cost
/// to find whether a node is within the focal range. Focal calc should produce
/// a value where focal_calc(f_min) >= f_min
/// Vaguely based upon: https://www.ijcai.org/proceedings/2018/0199.pdf
pub struct FocalSearch<N, C>
where
    N: Hash + Clone + Eq + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    heuristic: Arc<dyn Fn(&N) -> C + Sync + Send>,
    focal_heuristic: Arc<dyn Fn(&N) -> C + Sync + Send>,
    focal_calc: Arc<dyn Fn(&C) -> C + Sync + Send>,
}

impl<N, C> FocalSearch<N, C>
where
    N: Hash + Clone + Eq + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    /// Create a new Focal Search
    pub fn new(
        heuristic: Arc<dyn Fn(&N) -> C + Sync + Send>,
        focal_heuristic: Arc<dyn Fn(&N) -> C + Sync + Send>,
        focal_calc: Arc<dyn Fn(&C) -> C + Sync + Send>,
    ) -> Self {
        FocalSearch {
            heuristic,
            focal_heuristic,
            focal_calc,
        }
    }
}

impl<N, C> Search<N, C> for FocalSearch<N, C>
where
    N: Hash + Clone + Eq + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    fn _search<E, I, G>(
        &self,
        mut expander: E,
        start: N,
        goal: G,
    ) -> (AHashMap<N, (Option<N>, C)>, Option<N>)
    where
        E: FnMut(&N) -> I,
        I: IntoIterator<Item = (N, C)>,
        G: Fn(&N) -> bool,
    {
        let mut rng = SmallRng::from_entropy();
        let mut open = FSOpenList(BTreeMap::new());
        open.insert(start.clone(), (self.heuristic)(&start));
        // Focal can be implemented as a heap sorted with the focal heuristic or as
        // a tuple with both heuristics. The latter is more efficienct given an
        // expensive heuristic function or in cases where the tie breaking of the
        // focal heuristic is important. The former is more efficient in cases where
        // the amount of nodes in the focal list is large.
        let mut focal = BinaryHeap::from([SearchNode {
            node: start.clone(),
            cost: ((self.focal_heuristic)(&start), (self.heuristic)(&start)),
            random_key: rng.gen(),
        }]);
        let mut previous = AHashMap::new();
        previous.insert(start.clone(), (None, C::default()));

        while let Some(SearchNode {
            node,
            cost: (fcost, hcost),
            ..
        }) = focal.pop()
        {
            if goal(&node) {
                return (previous, Some(node));
            }
            let f_min = open.peak().unwrap_or(&fcost).clone();
            if !open.remove(&node, &(hcost)) {
                continue;
            }
            for (child, cost) in expander(&node) {
                let new_cost = previous[&node].1.clone() + cost;
                if !previous.contains_key(&child) || new_cost < previous[&child].1 {
                    previous.insert(child.clone(), (Some(node.clone()), new_cost.clone()));
                    let h = (self.heuristic)(&child);
                    let child_h = h.clone() + new_cost.clone();
                    open.insert(child.clone(), child_h.clone());
                    // Add to focal list given within the focal range
                    if new_cost <= (self.focal_calc)(&f_min) {
                        focal.push(SearchNode {
                            node: child.clone(),
                            cost: (new_cost.clone() + (self.focal_heuristic)(&child), child_h), // TODO: No use of previous
                            random_key: rng.gen(),
                    });
                    }
                }
            }
            // Update Lower Bound
            if let Some(hcost) = open.peak() {
                if f_min < *hcost {
                    for (node, cost) in open.iter() {
                        if *cost > (self.focal_calc)(&f_min) && *cost <= (self.focal_calc)(hcost) {
                            focal.push(SearchNode {
                                node: node.clone(),
                                cost: (
                                    previous[node].1.clone() + (self.focal_heuristic)(&node),
                                    cost.clone(),
                                ),
                                random_key: rng.gen(),
                        });
                        }
                    }
                }
            }
        }
        (previous, None)
    }
}

impl<N, C> BestSearch<N, C> for FocalSearch<N, C>
where
    N: Hash + Clone + Eq + Sync,
    C: Ord + Default + Clone + Add<Output = C> + Sync,
{
    fn get_best_heuristic(&self) -> &Arc<dyn Fn(&N) -> C + Sync + Send> {
        &self.heuristic
    }
}

#[cfg(test)]
mod test {
    use std::sync::Arc;

    use crate::{domains::GridCreate2d, heuristics::distance::manhattan_distance};

    use super::*;
    #[test]
    fn test_fssearchlist() {
        #[derive(Clone, PartialEq, Eq, Debug, Hash)]
        struct NoCompare(u32);
        let mut fsopenlist = FSOpenList(BTreeMap::new());
        fsopenlist.insert(NoCompare(0), 0);
        fsopenlist.insert(NoCompare(0), 0);
        fsopenlist.insert(NoCompare(1), 0);
        fsopenlist.insert(NoCompare(3), 1);
        fsopenlist.insert(NoCompare(1), 1);
        assert_eq!(fsopenlist.0.len(), 2);
        assert_eq!(0, *fsopenlist.peak().unwrap());
        assert!(fsopenlist.remove(&NoCompare(3), &1));
        assert_eq!(fsopenlist.0.get(&1).unwrap().len(), 1);
    }

    #[test]
    fn test_focal_search() {
        let results = FocalSearch::new(Arc::new(|x| *x), Arc::new(|x| *x), Arc::new(|x| *x))
            .search(|x| vec![(x + 1, 1), (x + 2, 2)], 0, |x| *x == 2);
        assert_eq!(results.unwrap().0, vec![0, 2]);
    }

    #[test]
    fn test_focal_search_bitpacked_grid() {
        // In this example the focal list == open list at each iteration
        let grid = crate::domains::bitpackedgrids::bitpackedgrid2d::BitPackedGrid2d::new_from_string(
            ".....\n.###.\n.#...\n.#.#.\n...#.".to_string(),
        );
        let path = FocalSearch::<(usize, usize), usize>::new(
            Arc::new(|n| manhattan_distance(*n, (4, 4))), // Fix: Dereference the reference to the tuple
            Arc::new(|_| 0),
            Arc::new(|x| *x),
        )
        .search(
            |(x, y)| {
                grid.adjacent((x.clone(), y.clone()), false)
                    .map(|(x, y)| ((x, y), 1))
            },
            (0, 4),
            |n| n == &(4, 4),
        );
        assert_eq!(
            path.unwrap().0,
            vec![
                (0, 4),
                (1, 4),
                (2, 4),
                (2, 3),
                (2, 2),
                (3, 2),
                (4, 2),
                (4, 3),
                (4, 4)
            ]
        );
    }
}
