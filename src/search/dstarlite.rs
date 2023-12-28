use std::{ops::Add, collections::{BinaryHeap, HashMap}, hash::Hash, cmp::Ordering, vec};
use super::SearchNodeState;

/// Reverse the ordering of an option such that `None` is greater than `Some`
#[derive(Clone, Eq, PartialEq, Debug)]
struct RevSome<T>(Option<T>);

impl<T: Ord> Ord for RevSome<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        match (self.0.as_ref(), other.0.as_ref()) {
            (Some(a), Some(b)) => a.cmp(b),
            (Some(_), None) => Ordering::Less,
            (None, Some(_)) => Ordering::Greater,
            (None, None) => Ordering::Equal,
        }
    }
}

impl<T: Ord> PartialOrd for RevSome<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

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
struct DStarLite<E, I, N, C, H, M, J> where
    E: Fn(&N) -> I,
    I: IntoIterator<Item = (N, C)>,
    N: Hash + Eq + Clone,
    C: Ord + Default + Clone + Add<Output = C>,
    H: Fn(&N, &N) -> C,
    M: FnMut(&N) -> J,
    J: IntoIterator<Item = N>
{
    expander: E,
    start: N,
    goal: N,
    heuristic: H,
    mutator: M,
    /// k_m is the lookahead value
    k_m: C,
    /// RHS values are one-step ahead lookahead values
    rhs: HashMap<N, C>,
    /// G values are an estimate of distance to nodes
    g_score: HashMap<N, C>,
    /// The frontier priority queue of nodes to be expanded
    queue: BinaryHeap<SearchNodeState<N, Option<(C, C)>>>,
    /// The last node that was mutated
    s_last: N,
}

impl<E, I, N, C, H, M, J> DStarLite<E, I, N, C, H, M, J> where
    E: Fn(&N) -> I,
    I: IntoIterator<Item = (N, C)>,
    N: Hash + Eq + Clone,
    C: Ord + Default + Clone + Add<Output = C>,
    H: Fn(&N, &N) -> C,
    M: FnMut(&N) -> J,
    J: IntoIterator<Item = N>,
{
    pub fn new(
        expander: E,
        start: N,
        goal: N,
        heuristic: H,
        mutator: M,
    ) -> Self {
        let start_h = (heuristic)(&start, &goal);
        let mut dstar = DStarLite {
            expander,
            start: start.clone(), 
            goal: goal.clone(),
            heuristic,
            mutator,
            rhs: HashMap::from([(start.clone(), C::default())]),
            k_m: C::default(),
            g_score: HashMap::new(),
            queue: BinaryHeap::from(vec![SearchNodeState {
                node: goal,
                cost: Some((start_h, C::default())),
            }]),
            s_last: start
        };
        dstar.compute_shortest_dist();
        dstar
    }
    
    /// Run the algorithm for one step
    pub fn step(&mut self) {
        // Moves to new start position
        self.start = (self.expander)(&self.start)
            .into_iter()
            .min_by_key(|(child, cost)| RevSome(self.g_score.get(&child).map(|g| g.clone() + cost.clone())))
            .map(|(child, _)| child.clone())
            .unwrap_or(self.start.clone()); // Not safe
    
        // This section is currently unoptimised due to problem constraints
        let mut mutated = (self.mutator)(&self.start).into_iter().peekable();
        if mutated.peek().is_some() {
            self.k_m = self.k_m.clone() + (self.heuristic)(&self.s_last, &self.start);
            self.s_last = self.start.clone();
            for node in mutated {
                self.update_vertex(node);
            }
            self.compute_shortest_dist();
        }
        assert!(self.rhs.contains_key(&self.start), "There is no path to the goal");
    }

    /// Calculate the key for a vertex
    fn calculate_key(&self, node: &N) -> Option<(C, C)> {
        let min: RevSome<&C> = RevSome(self.g_score.get(node)).min(RevSome(self.rhs.get(node)));
        match min.0 {
            Some(min) => Some((
                min.clone() + (self.heuristic)(&self.start, &node) + self.k_m.clone(), 
                min.clone()
            )),
            None => None,
        }
    }

    /// Update the vertex in the queue
    fn update_vertex(&mut self, node: N) {
        self.queue.retain(|x| {
            x.node != node
        });
        if self.g_score.get(&node) != self.rhs.get(&node) {
            self.queue.push(SearchNodeState {
                node: node.clone(),
                cost: self.calculate_key(&node),
            });
        }
    }

    /// Compute the shortest path similar to a*
    fn compute_shortest_dist(&mut self) {
        while let Some(SearchNodeState { node, cost }) = self.queue.pop() {
            if !(cost < self.calculate_key(&self.start)
            || RevSome(self.rhs.get(&self.start)) > RevSome(self.g_score.get(&self.start))) {
                continue;
            }
            let new_cost = self.calculate_key(&node);
            // self.queue.retain(|x| x.node != node);
            if cost < new_cost {
                self.queue.retain(|x| x.node != node); // This might be able to be removed
                self.queue.push(SearchNodeState {
                    node: node.clone(),
                    cost: new_cost,
                });
            } else if RevSome(self.g_score.get(&node)) > RevSome(self.rhs.get(&node)) {
                self.g_score.insert(node.clone(), self.rhs.get(&node).unwrap().clone());
                self.queue.retain(|x| x.node != node); 
                for (child, cost) in (self.expander)(&node) {
                    if child != self.goal {
                        self.rhs.insert(
                            child.clone(), 
                            RevSome(self.rhs.get(&child))
                                .min(RevSome(self.g_score.get(&child).map(|x| x.clone() + cost).as_ref())).0.unwrap().clone()
                        );
                    }
                    self.update_vertex(child);
                }
            } else {
                let g_old = self.g_score.remove(&node);
                for (child, cost) in (self.expander)(&node) {
                    if self.rhs.get(&child) == g_old.clone().map(|x| x.clone() + cost).as_ref()
                    && child != self.goal {
                        let min = (self.expander)(&child)
                            .into_iter()
                            .min_by_key(|(child, cost)| RevSome(self.g_score.get(&child).map(|g| g.clone() + cost.clone())));
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
        let mut path = vec![self.start.clone()];
        let mut cost = C::default();
        let mut current = self.start.clone();
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
    use crate::domains::bitpackedgrid::BitPackedGrid;
    use crate::heuristics::distance::manhattan_distance;

    #[test]
    fn test_rev_some() {
        let a = RevSome(Some(1));
        let b = RevSome(Some(2));
        let c = RevSome(None);
        assert!(a < b);
        assert!(b < c);
        assert!(a < c);
    }

    #[test]
    fn test_shortest_distance() {
        let grid = BitPackedGrid::create_from_string("........\n...###..\n.....#..\n.....#..\n........\n........".to_string());
        let mut dstar = DStarLite::new(
            |(x, y)| grid.adjacent(x.clone(), y.clone(), false).map(|(x, y)| ((x, y), 1)), 
            (0, 5),
            (7, 0),
            |node1, node2| manhattan_distance(*node1, *node2),
            |_| vec![],
        );
        dstar.step();
        assert_eq!(dstar.path().unwrap().0, vec![(0, 5), (1, 5), (2, 5), (3, 5), (4, 5), (5, 5), (6, 5), (6, 4), (7, 4), (7, 3), (7, 2), (7, 1), (7, 0)]);
    }
}