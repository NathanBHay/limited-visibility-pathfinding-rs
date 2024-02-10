#![allow(dead_code)]

use std::{cmp::Ordering, collections::HashMap, hash::Hash, sync::Arc};

pub mod astar;
pub mod dstarlite;
pub mod focalsearch;
pub mod pathstore;
pub mod samplestar;
pub mod uninformed;
pub mod samplestarstats;

/// Trait representation of a search algorithm, allows for polymorphic searches.
/// A search algorithm is therefore a structure that keeps its specific functions
/// heuristic function.
pub trait Search<N, C> 
where
    N: Hash + Eq + Clone,
    C: Ord + Clone
{
    /// Search function that finds a path from the start node to the goal node
    /// ## Arguments
    /// * `expander` - The expander function that returns the children of a node
    /// * `start` - The starting node
    /// * `goal` - The goal function that returns true if the node is the goal
    /// ## Returns
    /// The path to the goal or None if the goal isn't found
    fn search<E, I, G>(&self, expander: E, start: N, goal: G) -> Option<(Vec<N>, C)>
    where
        E: FnMut(&N) -> I,
        I: IntoIterator<Item = (N, C)>,
        G: Fn(&N) -> bool 
    {
        match self._search(expander, start, goal) {
            (distances, Some(goal)) => Some(reconstruct_path(&distances, goal)),
            _ => None,
        }
    }

    /// Search function that is implemented. Returns a HashMap to allow for optional path
    /// reconstruction and cost retrieval for any node.
    /// ## Arguments
    /// * `expander` - The expander function that returns the children of a node
    /// * `start` - The starting node
    /// * `goal` - The goal function that returns true if the node is the goal
    /// ## Returns
/// A HashMap of nodes to their parent nodes and the cost to reach them and the goal node
    fn _search<E, I, G>(&self, expander: E, start: N, goal: G) -> (HashMap<N, (Option<N>, C)>, Option<N>)
    where
        E: FnMut(&N) -> I,
        I: IntoIterator<Item = (N, C)>,
        G: Fn(&N) -> bool;
}

/// Trait that represents a search algorithm that can return the best next node if the goal isn't
/// found. This algorithm also assumes parallelism.
pub trait BestSearch<N, C>: Search<N, C> 
where
    N: Hash + Eq + Clone,
    C: Ord + Clone
{
    /// Search function that finds a path from the start node to the goal node. In the event the 
    /// goal isn't found, it returns the path of the node reasoned to be the best next node
    /// ## Arguments
    /// * `expander` - The expander function that returns the children of a node
    /// * `start` - The starting node
    /// * `goal` - The goal function that returns true if the node is the goal
    /// ## Returns
    /// The path to the goal or the best node
    fn best_search<E, I, G>(&self, expander: E, start: N, goal: G) -> (Vec<N>, C)
    where
        E: FnMut(&N) -> I,
        I: IntoIterator<Item = (N, C)>,
        G: Fn(&N) -> bool 
    {
        match self._search(expander, start, goal) {
            (distances, Some(goal)) => reconstruct_path(&distances, goal),
            (distances, _) => {
                let (best_node, _) = distances
                    .iter()
                    .filter_map(|(node, (parent, _))| 
                        match parent {
                            Some(_) => Some((node, self.get_best_heuristic()(node))),
                            None => None
                        }
                    )
                    .min_by_key(|(_, cost)| cost.clone())
                    .unwrap();
                reconstruct_path(&distances, best_node.clone())
            }
        }    
    }

    /// Returns the heuristic used to find the next best node rather than the goal node
    fn get_best_heuristic(&self) -> &Arc<dyn Fn(&N) -> C + Sync + Send>;
}

/// Reconstructs a path from a given node to the start node given a cost
/// ## Arguments
/// * `parent` - A map of nodes to their parent nodes
/// * `node` - The node to reconstruct the path from
/// ## Returns
/// A vector of nodes from the start to the given node and the cost of the path
fn reconstruct_path<N, C>(
    parent: &HashMap<N, (Option<N>, C)>,
    mut node: N,
) -> (Vec<N>, C)
where
    N: Hash + Eq + Clone,
    C: Clone
{
    let mut path = vec![node.clone()];
    let cost = parent[&node].1.clone();
    while let Some((Some(prev), _)) = parent.get(&node) {
        path.push(prev.clone());
        node = prev.clone();
    }
    path.reverse();
    (path, cost)
}

/// Search node used in A-Star/Focal/D-Star Binary Heap
#[derive(Clone, PartialEq, Eq, Debug)]
pub(crate) struct SearchNode<N: Eq, C: Ord> {
    node: N,
    cost: C,
}

impl<N: Eq, C: Ord> Ord for SearchNode<N, C> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.cost.cmp(&other.cost).reverse()
    }
}

impl<N: Eq, C: Ord> PartialOrd for SearchNode<N, C> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg(test)]
mod tests {

    use std::collections::BinaryHeap;

    use crate::search::{reconstruct_path, SearchNode};

    #[test]
    fn test_reconstruct_path() {
        let mut parent = std::collections::HashMap::new();
        parent.insert(1, (None, 0));
        parent.insert(2, (Some(1), 1));
        parent.insert(3, (Some(2), 2));
        parent.insert(4, (Some(3), 3));
        parent.insert(5, (Some(4), 4));
        let (path, cost) = reconstruct_path(&parent, 5);
        assert_eq!(path, vec![1, 2, 3, 4, 5]);
        assert_eq!(cost, 4);
    }

    #[test]
    fn test_search_node() {
        let mut open = BinaryHeap::new();
        open.push(SearchNode { node: 1, cost: 1 });
        open.push(SearchNode { node: 0, cost: 0 });
        open.push(SearchNode { node: 2, cost: 2 });
        assert_eq!(open.pop().unwrap().node, 0);
        assert_eq!(open.pop().unwrap().node, 1);
        assert_eq!(open.pop().unwrap().node, 2);
    }
}
