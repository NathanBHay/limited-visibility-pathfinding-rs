pub struct EdgeList<N: Eq, W>(Vec<(N, N, W)>);

impl<N: Eq, W> EdgeList<N, W> {

    pub fn new() -> Self {
        Self(Vec::new())
    }

    pub fn add_edge(&mut self, from: N, to: N, weight: W) {
        self.0.push((from, to, weight));
    }

    pub fn iter(&self) -> impl Iterator<Item = &(N, N, W)> {
        self.0.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut (N, N, W)> {
        self.0.iter_mut()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn remove_edge(&mut self, from: N, to: N) {
        self.0.retain(|(f, t, _)| f != &from || t != &to);
    }

    pub fn remove_node(&mut self, node: N) {
        self.0.retain(|(f, t, _)| f != &node && t != &node);
    }

    pub fn degree(&self, node: N) -> usize {
        self.iter().filter(|(f, t, _)| f == &node || t == &node).count()
    }
}