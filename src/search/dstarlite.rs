//! D* Lite Search Algorithm
//! Currently not working as of the projects end date.

use ahash::AHashMap;

use super::SearchNode;
use std::{
    collections::BinaryHeap,
    fmt::Debug,
    hash::Hash,
    ops::Add,
    vec,
};

use super::RevSome;

/// D* Lite Search Algorithm
/// ## Arguments
/// * `expander` - A function that takes a node and returns an iterator over its successors,
/// where successors == predecessors. This means a bidirectional structure is required. This
/// could be changed to be split into two functions, one for successors and one for predecessors.
/// * `start` - The starting node of the search
/// * `goal` - The goal node of the search
/// * `heuristic` - A function that takes a node and returns the heuristic cost to the goal
/// * `mutator` - A function that takes node at the current iteration of dstar lite and applies
/// a mutation that changes the cost of the node, returning the old cost.
struct DStarLite<E, I, N, C, H, M, J>
where
    E: Fn(&N) -> I,
    I: IntoIterator<Item = (N, C)>,
    N: Hash + Eq + Clone,
    C: Ord + Default + Clone + Add<Output = C>,
    H: Fn(&N, &N) -> C,
    M: FnMut(&N) -> J,
    J: IntoIterator<Item = N>,
{
    expander: E,
    current: N,
    goal: N,
    final_path: Vec<N>,
    heuristic: H,
    mutator: M,
    /// k_m is the lookahead value
    k_m: C,
    /// RHS values are one-step ahead lookahead values
    rhs: AHashMap<N, C>,
    /// G values are an estimate of distance to nodes
    g_score: AHashMap<N, C>,
    /// The frontier priority queue of nodes to be expanded
    queue: BinaryHeap<SearchNode<N, Option<(C, C)>>>,
    /// The last node that was mutated
    s_last: N,
}

impl<E, I, N, C, H, M, J> DStarLite<E, I, N, C, H, M, J>
where
    E: Fn(&N) -> I,
    I: IntoIterator<Item = (N, C)>,
    N: Hash + Eq + Clone + Debug,
    C: Ord + Default + Clone + Add<Output = C> + Debug,
    H: Fn(&N, &N) -> C,
    M: FnMut(&N) -> J,
    J: IntoIterator<Item = N>,
{
    /// Create a new instance of D* Lite
    pub fn new(expander: E, start: N, goal: N, heuristic: H, mutator: M) -> Self {
        let start_h = (heuristic)(&start, &goal);
        let mut dstar = DStarLite {
            expander,
            current: start.clone(),
            goal: goal.clone(),
            final_path: vec![start.clone()],
            heuristic,
            mutator,
            rhs: AHashMap::from([(start.clone(), C::default())]),
            k_m: C::default(),
            g_score: AHashMap::new(),
            queue: BinaryHeap::from([SearchNode::new(
                goal,
                Some((start_h, C::default())),
            )]),
            s_last: start,
        };
        dstar.compute_shortest_dist();
        dstar
    }

    /// Run the algorithm for one step
    pub fn step(&mut self) -> bool {
        if self.current == self.goal {
            return true;
        }
        // Moves to new start position
        self.current = (self.expander)(&self.current)
            .into_iter()
            .min_by_key(|(child, cost)| {
                RevSome(self.g_score.get(&child).map(|g| g.clone() + cost.clone()))
            })
            .map(|(child, _)| child.clone())
            .unwrap_or(self.current.clone()); // Not safe
        self.final_path.push(self.current.clone());

        // This section is currently unoptimised due to problem constraints
        let mut mutated = (self.mutator)(&self.current).into_iter().peekable();
        if mutated.peek().is_some() {
            self.k_m = self.k_m.clone() + (self.heuristic)(&self.s_last, &self.current);
            self.s_last = self.current.clone();
            for node in mutated {
                self.update_vertex(node);
            }
            self.compute_shortest_dist();
        }
        false
    }

    /// Calculate the key for a vertex
    fn calculate_key(&self, node: &N) -> Option<(C, C)> {
        let min = std::cmp::min(RevSome(self.g_score.get(node)), RevSome(self.rhs.get(node)));
        match min.0 {
            Some(min) => Some((
                min.clone() + (self.heuristic)(&self.current, &node) + self.k_m.clone(),
                min.clone(),
            )),
            None => None,
        }
    }

    /// Update the vertex in the queue
    fn update_vertex(&mut self, node: N) {
        self.queue.retain(|x| x.node != node);
        if self.g_score.get(&node) != self.rhs.get(&node) {
            self.queue.push(SearchNode::new(
                node.clone(),
                self.calculate_key(&node),
            ));
        }
    }

    /// Compute the shortest path similar to a*
    fn compute_shortest_dist(&mut self) {
        while let Some(SearchNode{ node, cost, .. }) = self.queue.pop() {
            println!(
                "{:?} | {:?}",
                self.rhs.get(&self.current),
                self.g_score.get(&self.current)
            );
            if !(cost < self.calculate_key(&self.current)
                || RevSome(self.rhs.get(&self.current)) > RevSome(self.g_score.get(&self.current)))
            {
                break;
            }
            let new_cost = self.calculate_key(&node);
            // self.queue.retain(|x| x.node != node);
            if cost < new_cost {
                self.queue.retain(|x| x.node != node); // This might be able to be removed
                self.queue.push(SearchNode::new(
                    node.clone(),
                    new_cost,
                ));
            } else if RevSome(self.g_score.get(&node)) > RevSome(self.rhs.get(&node)) {
                self.g_score
                    .insert(node.clone(), self.rhs.get(&node).unwrap().clone());
                self.queue.retain(|x| x.node != node);
                for (child, cost) in (self.expander)(&node) {
                    if child != self.goal {
                        self.rhs.insert(
                            child.clone(),
                            std::cmp::min(
                                RevSome(self.rhs.get(&child)),
                                RevSome(
                                    self.g_score.get(&child).map(|x| x.clone() + cost).as_ref(),
                                ),
                            )
                            .0
                            .unwrap()
                            .clone(),
                        );
                    }
                    self.update_vertex(child);
                }
            } else {
                let g_old = self.g_score.remove(&node);
                for (child, cost) in (self.expander)(&node) {
                    if self.rhs.get(&child) == g_old.clone().map(|x| x.clone() + cost).as_ref()
                        && child != self.goal
                    {
                        let min =
                            (self.expander)(&child)
                                .into_iter()
                                .min_by_key(|(child, cost)| {
                                    RevSome(
                                        self.g_score.get(&child).map(|g| g.clone() + cost.clone()),
                                    )
                                });
                        if let Some((_, min)) = min {
                            self.rhs.insert(child.clone(), min);
                        }
                        self.update_vertex(child);
                    }
                }
            }
        }
    }

    /// Get the path from the start to the goal
    fn path(&self) -> Option<(Vec<N>, C)> {
        let mut path = vec![self.current.clone()];
        let mut cost = C::default();
        let mut current = self.current.clone();
        while current != self.goal {
            let mut min = RevSome(Some(C::default()));
            let mut next = current.clone();
            for (child, cost) in (self.expander)(&current) {
                let val = RevSome(self.g_score.get(&child).map(|g| g.clone() + cost));
                if val < min {
                    min = val;
                    next = child;
                }
            }
            if next == current {
                return None;
            }
            cost = cost + min.0.unwrap();
            path.push(next.clone());
            current = next;
        }
        Some((path, cost))
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::domains::bitpackedgrids::bitpackedgrid2d::BitPackedGrid2d;
    use crate::domains::{GridDomain, GridCreate2d};
    use crate::heuristics::distance::manhattan_distance;

    #[test]
    fn test_shortest_distance() {
        let grid = BitPackedGrid2d::new_from_string(
            "........\n...###..\n.....#..\n.....#..\n........\n........".to_string(),
        );
        let mut dstar = DStarLite::new(
            |(x, y)| {
                grid.adjacent((x.clone(), y.clone()), false)
                    .map(|(x, y)| ((x, y), 1))
            },
            (0, 5),
            (7, 0),
            |node1, node2| manhattan_distance(*node1, *node2),
            |_| vec![],
        );
        assert!(!dstar.step());
        println!("{:?}", dstar.path());
        // assert_eq!(
        //     dstar.path().unwrap().0,
        //     vec![
        //         (0, 5),
        //         (1, 5),
        //         (2, 5),
        //         (3, 5),
        //         (4, 5),
        //         (5, 5),
        //         (6, 5),
        //         (6, 4),
        //         (7, 4),
        //         (7, 3),
        //         (7, 2),
        //         (7, 1),
        //         (7, 0)
        //     ]
        // );
    }
}
