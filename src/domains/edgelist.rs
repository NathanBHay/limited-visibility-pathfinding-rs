/// Edge List Representation of a Graph
pub struct EdgeList<N: Eq, W>(Vec<(N, N, W)>);

impl<N: Eq, W> EdgeList<N, W> {
    /// Create a new empty graph
    pub fn new() -> Self {
        Self(Vec::new())
    }

    /// Add a edge to the list
    pub fn add_edge(&mut self, from: N, to: N, weight: W) {
        self.0.push((from, to, weight));
    }

    /// Get an iterator over the edges
    pub fn iter(&self) -> impl Iterator<Item = &(N, N, W)> {
        self.0.iter()
    }

    /// Get a mutable iterator over the edges
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut (N, N, W)> {
        self.0.iter_mut()
    }

    /// Get the number of nodes in the graph
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Remove Edge
    pub fn remove_edge(&mut self, from: N, to: N) {
        self.0.retain(|(f, t, _)| f != &from || t != &to);
    }

    /// Remove Node
    pub fn remove_node(&mut self, node: N) {
        self.0.retain(|(f, t, _)| f != &node && t != &node);
    }

    /// Get the degree of a node
    pub fn degree(&self, node: N) -> usize {
        self.iter()
            .filter(|(f, t, _)| f == &node || t == &node)
            .count()
    }
}
