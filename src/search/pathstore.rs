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
    fn add_path(&mut self, path: Vec<N>, weight: W);

    /// Remove a path from the store's internal data structure
    // fn remove_path(&mut self, path: &Vec<(usize, usize)>);

    /// Get the best node to explore next, reaturns none if no node is found
    fn next_node(&self, nodes: Vec<N>) -> Option<N>;

    /// Get the weight of a given node
    fn get(&self, node: &N) -> Option<&W>;

    /// Get length of store
    fn len(&self) -> usize;

    /// Get the paths stored in the store
    fn visualise(&self) -> Vec<(N, W)>;
}

/// A store which accumulates the number of times a node has been visited,
/// using a heuristic to weight the value of each visit
pub struct AccStore<N: Eq + Hash + Send, W: Send> {
    store: HashMap<N, W>,
    heuristic: Box<dyn Fn(&W) -> W + Send>,
}

impl<N: Eq + Hash + Send, W: Send> AccStore<N, W> {
    /// Create a new accumulating store
    pub fn new(heuristic: Box<dyn Fn(&W) -> W + Send>) -> Self {
        Self {
            store: HashMap::new(),
            heuristic,
        }
    }
    /// Check if the store contains a given node
    fn contains(&self, node: &N) -> bool {
        self.store.contains_key(node)
    }
}

impl<N, W> PathStore<N, W> for AccStore<N, W> 
where
    N: Clone + Eq + Hash + Send,
    W: Add<Output = W> + Clone + Default + Ord + Send
{
    fn reinitialize(&mut self) {
        self.store.clear();
    }

    fn add_path(&mut self, path: Vec<N>, weight: W) {
        for node in path {
            let entry = self.store.entry(node).or_insert(W::default());
            *entry = entry.clone() + (self.heuristic)(&weight);
        }
    }

    fn next_node(&self, nodes: Vec<N>) -> Option<N> {
        nodes.into_iter()
            .filter(|n| self.contains(n))
            .max_by_key(|n| self.store.get(n).unwrap())
    }

    fn len(&self) -> usize {
        self.store.len()
    }

    fn get(&self, node: &N) -> Option<&W> {
        self.store.get(node)
    }

    fn visualise(&self) -> Vec<(N, W)> {
        self.store.iter().map(|(n, w)| (n.clone(), w.clone())).collect()
    }
}

impl<N: Eq + Hash + Send> AccStore<N, usize> {
    /// Create a Acc Store that counts the number of times a node has been visited
    pub fn new_count_store() -> Self {
        AccStore::new(Box::new(|_| 1))
    }
}

/// Keeps a store of only the best path to a given node 
pub struct GreedyStore<N: Eq + Hash + Send, W: Send> {
    store: Vec<N>,
    weight: Option<W>,
    heuristic: Box<dyn Fn(&N) -> W + Send>,
}

impl<N: Eq + Hash + Send, W: Send> GreedyStore<N, W> {
    /// Create a new greedy store
    pub fn new(heuristic: Box<dyn Fn(&N) -> W + Send>) -> Self {
        Self {
            store: Vec::new(),
            weight: None,
            heuristic,
        }
    }
}

impl<N, W> PathStore<N, W> for GreedyStore<N, W> 
where
    N: Clone + Eq + Hash + Send + core::fmt::Debug,
    W: Clone + Default + Ord + Send
{
    fn reinitialize(&mut self) {
        self.store.clear();
        self.weight = None;
    }

    fn add_path(&mut self, path: Vec<N>, _weight: W) {
        self.store = path;
        if let Some(node) = self.store.first() {
            println!("Node: {:?}", node);
            self.weight = Some((self.heuristic)(node));
        }
    }

    fn next_node(&self, nodes: Vec<N>) -> Option<N> {
        nodes.into_iter().find(|n| self.store.first() == Some(n))
    }

    fn get(&self, _node: &N) -> Option<&W> {
        self.weight.as_ref()
    }

    fn len(&self) -> usize {
        self.store.len()
    }

    fn visualise(&self) -> Vec<(N, W)> {
        self.store.iter().map(|n| (n.clone(), self.weight.clone().unwrap())).collect()
    }
}