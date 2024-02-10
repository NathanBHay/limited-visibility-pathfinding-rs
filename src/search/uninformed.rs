//! A series of uninformed search algorithms
//! These algorithms include:
//! * Breadth-First Search
//! * Depth-First Search

use std::{
    collections::{HashMap, VecDeque},
    hash::Hash, ops::Add,
};

use super::Search;

/// Breadth-First Search
struct Bfs;

impl Bfs {
    /// Create a new Breadth-First Search
    pub fn new() -> Self {
        Bfs
    }
}

impl<N, C> Search<N, C> for Bfs
where
    N: Hash + Eq + Clone,
    C: Ord + Clone + Default + Add<Output = C>,
{
    fn _search<E, I, G>(&self, mut expander: E, start: N, goal: G) -> (HashMap<N, (Option<N>, C)>, Option<N>)
        where
            E: FnMut(&N) -> I,
            I: IntoIterator<Item = (N, C)>,
            G: Fn(&N) -> bool 
        {
        let mut queue = VecDeque::new();
        let mut distance = HashMap::new();
        distance.insert(start.clone(), (None, C::default()));
        queue.push_back((start, C::default()));
        while let Some((node, cost)) = queue.pop_front() {
            if goal(&node) {
                return (distance, Some(node));
            }
            for (child, child_cost) in expander(&node) {
                if !distance.contains_key(&child) {
                    let new_cost = cost.clone() + child_cost;
                    distance.insert(child.clone(), (Some(node.clone()), new_cost.clone()));
                    queue.push_back((child, new_cost));
                }
            }
        }
        (distance, None)
    }
}

/// Iterative Depth-First Search
struct Dfs;

impl Dfs {
    /// Create a new Depth-First Search
    pub fn new() -> Self {
        Dfs
    }
}

impl<N, C> Search<N, C> for Dfs
where
    N: Hash + Eq + Clone,
    C: Ord + Clone + Default + Add<Output = C>,
{
    fn _search<E, I, G>(&self, mut expander: E, start: N, goal: G) -> (HashMap<N, (Option<N>, C)>, Option<N>)
        where
            E: FnMut(&N) -> I,
            I: IntoIterator<Item = (N, C)>,
            G: Fn(&N) -> bool 
        {
        let mut stack = Vec::new();
        let mut distance = HashMap::new();
        distance.insert(start.clone(), (None, C::default()));
        stack.push((start, C::default()));
        while let Some((node, cost)) = stack.pop() {
            if goal(&node) {
                return (distance, Some(node));
            }
            for (child, child_cost) in expander(&node) {
                if !distance.contains_key(&child) {
                    let new_cost = cost.clone() + child_cost;
                    distance.insert(child.clone(), (Some(node.clone()), new_cost.clone()));
                    stack.push((child, new_cost));
                }
            }
        }
        (distance, None)
    }
}

#[cfg(test)]
mod tests {

    use crate::search::uninformed::*;

    #[test]
    fn test_bfs() {
        let results = Bfs::new().search(|x| vec![(x + 1, 1), (x + 2, 1)], 0, |x| *x == 2);
        assert_eq!(results.unwrap().0, vec![0, 2]);
    }

    #[test]
    fn test_dfs() {
        let results = Dfs::new().search(|x| vec![(x + 1, 1), (x + 2, 1)], 0, |x| *x == 2);
        assert_eq!(results.unwrap().0, vec![0, 2]);
    }
}
