//! A series of data structure used to store the paths taken by running multiple
//! searches on a given map. These structures use the `PathStore` trait which has
//! a function to add the found path and a function to get the next node to
//! explore given that path.

use std::{cmp::Ord, collections::HashMap, hash::Hash, ops::Add};

/// PathStore is a trait that defines the interface for storing multiple paths
/// as to be able to find most commomly taken paths. `Send` is required to allow
/// for the store to be used in parallel.
pub trait PathStore<N: Send, W: Send>: Send {
    /// Reinitialize the store at the start of a new search,
    /// optional as some stores may not need to be reinitialized
    fn reinitialize(&mut self) {}

    /// Add a path to the store's internal data structure
    fn add_path(&mut self, path: Box<dyn Iterator<Item = N>>, weight: W);

    /// Remove a path from the store's internal data structure
    // fn remove_path(&mut self, path: &Vec<(usize, usize)>);

    /// Get the best node to explore next, reaturns none if no node is found
    fn next_node(&self, nodes: Box<dyn Iterator<Item = N>>) -> Option<N>;

    /// Check if node is in store
    fn contains(&self, node: &N) -> bool;

    /// Get the store's possible paths
    fn get_paths(&self) -> &HashMap<N, W>;
}

/// A store which accumulates the number of times a node has been visited,
/// using a heuristic to weight the value of each visit
pub struct AccStore<N: Eq + Hash + Send, W: Send> {
    store: HashMap<N, W>,
    heuristic: Box<dyn Fn(&W) -> W + Send>,
}

impl<N: Eq + Hash + Send, W: Send> AccStore<N, W> {
    pub fn new(heuristic: Box<dyn Fn(&W) -> W + Send>) -> Self {
        Self {
            store: HashMap::new(),
            heuristic,
        }
    }
}

impl<N, W> PathStore<N, W> for AccStore<N, W> 
where
    N: Eq + Hash + Send,
    W: Add<Output = W> + Clone + Default + Ord + Send
{
    fn reinitialize(&mut self) {
        self.store.clear();
    }

    fn add_path(&mut self, path: Box<dyn Iterator<Item = N>>, weight: W) {
        for node in path {
            let entry = self.store.entry(node).or_insert(W::default());
            *entry = entry.clone() + (self.heuristic)(&weight);
        }
    }

    fn next_node(&self, nodes: Box<dyn Iterator<Item = N>>) -> Option<N> {
        nodes
            .filter(|n| self.contains(n))
            .max_by_key(|n| self.store.get(n).unwrap())
    }

    fn contains(&self, node: &N) -> bool {
        self.store.contains_key(node)
    }

    fn get_paths(&self) -> &HashMap<N, W> {
        &self.store
    }
}

impl<N: Eq + Hash + Send> AccStore<N, usize> {
    /// Create a Acc Store that counts the number of times a node has been visited
    pub fn new_count_store() -> Self {
        AccStore::new(Box::new(|_| 1))
    }
}
