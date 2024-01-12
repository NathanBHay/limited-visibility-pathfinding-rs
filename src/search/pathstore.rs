//! A series of data structure used to store the paths taken by running multiple
//! searches on a given map. These structures use the `PathStore` trait which has
//! a function to add the found path and a function to get the next node to 
//! explore given that path.

use std::{collections::HashMap, hash::Hash};

/// PathStore is a trait that defines the interface for storing multiple paths
/// as to be able to find most commomly taken paths.
pub trait PathStore<N> {

    /// Reinitialize the store at the start of a new search,
    /// optional as some stores may not need to be reinitialized
    fn reinitialize(&mut self) {}

    /// Add a path to the store's internal data structure
    fn add_path(&mut self, path: Box<dyn Iterator<Item = N>>);

    /// Remove a path from the store's internal data structure
    // fn remove_path(&mut self, path: &Vec<(usize, usize)>);
    
    /// Get the best node to explore next
    fn next_node(&self, nodes: Box<dyn Iterator<Item = N>>) -> Option<N>;

    /// Check if node is in store
    fn contains(&self, node: &N) -> bool;

    /// Get the store's possible paths
    fn get_paths(&self) -> &HashMap<N, usize>;
}

/// A store which accumulates the number of times a node has been visited
pub struct AccStore<N: Eq + Hash> {
    pub paths: HashMap<N, usize>,
}

impl <N: Eq + Hash> AccStore<N> {
    pub fn new() -> Self {
        Self {
            paths: HashMap::new(),
        }
    }
}

impl<N: Eq + Hash> PathStore<N> for AccStore<N> {

    fn reinitialize(&mut self) {
        self.paths.clear();
    }

    fn add_path(&mut self, path: Box<dyn Iterator<Item = N>>) {
        for node in path {
            *self.paths.entry(node).or_insert(0) += 1;
        }
    }

    fn next_node(&self, nodes: Box<dyn Iterator<Item = N>>) -> Option<N> {
        nodes.filter(|n| self.paths.contains_key(n))
            .max_by_key(|n| self.paths[n])
    }

    fn contains(&self, node: &N) -> bool {
        self.paths.contains_key(node)
    }

    fn get_paths(&self) -> &HashMap<N, usize> {
        &self.paths
    }

}



// struct SmallStore


// Alternative approach could use a heuristic in storing
// struct Store {
//     paths: HashMap<Vec<(usize, usize)>, usize>,
// }

// impl Store {
//     fn add_path(&mut self, path: Vec<(usize, usize)>, heuristic: impl Fn(&(usize, usize)) -> usize) {
//         for node in path {
//             *self.paths.entry(node).or_insert(0) += 1 + heuristic(&node);
//         }
//     }
//// }
//