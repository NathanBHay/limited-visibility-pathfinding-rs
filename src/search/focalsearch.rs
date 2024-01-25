use std::{
    collections::{BTreeMap, BinaryHeap, HashMap, HashSet},
    fmt::Debug,
    hash::Hash,
    ops::Add,
};

use super::{reconstruct_path_with_cost, SearchNode};

/// Open list for Focal Search. Supports insertion, removal, and popping the best node
struct FSOpenList<N: Clone + Eq + Hash, C: Clone + Ord>(BTreeMap<C, HashSet<N>>);

impl<N: Clone + Eq + Hash, C: Clone + Ord> FSOpenList<N, C> {
    fn insert(&mut self, node: N, cost: C) {
        self.0.entry(cost).or_insert_with(HashSet::new).insert(node);
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
/// * `expander` - A function that returns an iterator over the nodes adjacent to a given node
/// * `start` - The start node
/// * `goal` - A function that returns whether or not a given node is the goal
/// * `heuristic` - A function that returns the heuristic value of a given node
/// * `focal_heuristic` - A function tha evaluates nodes within a range of
/// (0, epsilon] of the best node on the open list
/// * ``focal_calc`` - A function that calculates epsilon combined with a cost
/// to find whether a node is within the focal range. Focal calc should produce
/// a value where focal_calc(f_min) >= f_min
/// Vaguely based upon: https://www.ijcai.org/proceedings/2018/0199.pdf
pub fn focal_search<E, I, C, N, G, H1, H2, F>(
    mut expander: E,
    start: N,
    goal: G,
    heuristic: H1,
    focal_heuristic: H2,
    focal_calc: F,
) -> Option<(Vec<N>, C)>
where
    E: FnMut(&N) -> I,
    I: IntoIterator<Item = (N, C)>,
    C: Ord + Default + Clone + Add<Output = C> + Debug,
    N: Hash + Clone + Eq + Debug,
    G: Fn(&N) -> bool,
    H1: Fn(&N) -> C,
    H2: Fn(&N) -> C,
    F: Fn(&C) -> C,
{
    let mut open = FSOpenList(BTreeMap::new());
    open.insert(start.clone(), heuristic(&start));
    // Focal can be implemented as a heap sorted with the focal heuristic or as
    // a tuple with both heuristics. The latter is more efficienct given an
    // expensive heuristic function or in cases where the tie breaking of the
    // focal heuristic is important. The former is more efficient in cases where
    // the amount of nodes in the focal list is large.
    let mut focal = BinaryHeap::from([SearchNode {
        node: start.clone(),
        cost: (focal_heuristic(&start), heuristic(&start)),
    }]);
    let mut previous = HashMap::new();
    previous.insert(start.clone(), (None, C::default()));
    while let Some(SearchNode {
        node,
        cost: (fcost, hcost),
    }) = focal.pop()
    {
        if goal(&node) {
            return Some(reconstruct_path_with_cost(previous, node.clone()));
        }
        let f_min = open.peak().unwrap_or(&fcost).clone();
        if !open.remove(&node, &(hcost)) {
            continue;
        } // Just to make sure :)
        for (child, cost) in expander(&node) {
            let new_cost = previous[&node].1.clone() + cost;
            if !previous.contains_key(&child) || new_cost < previous[&child].1 {
                previous.insert(child.clone(), (Some(node.clone()), new_cost.clone()));
                let child_h = heuristic(&child) + new_cost.clone();
                open.insert(child.clone(), child_h.clone());
                if new_cost <= focal_calc(&f_min) {
                    focal.push(SearchNode {
                        node: child.clone(),
                        cost: (new_cost.clone() + focal_heuristic(&child), child_h), // TODO: No use of previous
                    });
                }
            }
        }
        // Update Lower Bound
        if let Some(hcost) = open.peak() {
            if f_min < *hcost {
                for (node, cost) in open.iter() {
                    if *cost > focal_calc(&f_min) && *cost <= focal_calc(hcost) {
                        focal.push(SearchNode {
                            node: node.clone(),
                            cost: (
                                previous[node].1.clone() + focal_heuristic(&node),
                                cost.clone(),
                            ),
                        });
                    }
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod test {
    use crate::heuristics::distance::manhattan_distance;

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
        let results = focal_search(
            |x| vec![(x + 1, 1), (x + 2, 2)],
            0,
            |x| *x == 2,
            |x| *x,
            |x| *x,
            |x| *x,
        );
        assert_eq!(results.unwrap().0, vec![0, 2]);
    }

    #[test]
    fn test_focal_search_bitpacked_grid() {
        // In this example the focal list == open list at each iteration
        let grid = crate::domains::bitpackedgrid::BitPackedGrid::new_from_string(
            ".....\n.###.\n.#...\n.#.#.\n...#.".to_string(),
        );
        let path = focal_search(
            |(x, y)| {
                grid.adjacent((x.clone(), y.clone()), false)
                    .map(|(x, y)| ((x, y), 1))
            },
            (0, 4),
            |n| n == &(4, 4),
            |n| manhattan_distance(*n, (4, 4)), // Fix: Dereference the reference to the tuple
            |_| 0,
            |x| *x,
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
