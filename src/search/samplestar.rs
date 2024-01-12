//! # SampleStar
//! A 
//! 
//! Possible Optimisations:
//! * Get min between epochs and amount of nodes in radius which can be sampled
//! * Cache paths
//! Heuristics could include ones that take into account probability of being an obstacle:
//! `self.grid.sample_grid[x][y].state * manhattan_distance(n*, self.goal)`


use std::collections::HashMap;

use crate::{domains::samplegrid::SampleGrid, heuristics::distance::manhattan_distance, util::visualiser::Visualiser, search::pathstore::AccStore};

use super::{astar::astar, pathstore::PathStore};

/// Sample Star Algorithm
/// ## Arguments
/// * `grid` - The sampling grid to search on.
/// * `start` - The starting node.
/// * `goal` - The goal node.
/// * `epoch` - The number of times to sample each node.
/// * `radius` - The radius to sample around each node.
pub struct SampleStar
{
    grid:  SampleGrid,
    previous: (usize, usize),
    current: (usize, usize),
    goal: (usize, usize),
    epoch: usize,
    radius: usize,
    final_path: Vec<(usize, usize)>,
    path_store: Box<dyn PathStore<(usize, usize)>>,
}

impl SampleStar {

    /// Creates a new SampleStar algorithm
    pub fn new(
        grid: SampleGrid,
        start: (usize, usize), 
        goal: (usize, usize),
        epoch:usize,
        radius: usize,
        path_store: Box<dyn PathStore<(usize, usize)>>
    ) -> Self {
        assert!(grid.bound_check(start) && grid.bound_check(goal));
        Self {
            grid,
            previous: start,
            current: start,
            goal,
            epoch,
            radius,
            final_path: vec![start],
            path_store,
        }
    }

    /// Run the algorithm for one step
    pub fn step(&mut self) -> bool {
        if self.current == self.goal {
            return true;
        }
        self.path_store.reinitialize();
        // let mut cached_paths = HashMap::new();
        self.grid.update_node_kern(self.current, self.radius);
        // self.grid.init_gridmap_radius(self.current, self.radius + 1); // +1 to account for previous update
        self.grid.init_gridmap_nearest();
        for _ in 0..self.epoch { // .min(1 << (self.radius * self.radius)) could be further optimised
            // self.grid.sample_all();
            self.grid.sample_radius(self.current, self.radius);
            if let Some((path, _)) = astar(
                |n| self.grid.gridmap.adjacent1(*n),
                self.current,
                |n| *n == self.goal,
                |n| manhattan_distance(*n, self.goal),
            ) {
                self.path_store.add_path(Box::new(path.into_iter()));
            }
        }
        self.previous = self.current;
        let adj: Box<dyn Iterator<Item = (usize, usize)>> = Box::new(self.grid.adjacent(self.current, false).collect::<Vec<_>>().into_iter());
        self.current = self.path_store.next_node(adj).unwrap_or(self.current);
        self.final_path.push(self.current);

        false
    }

}

/*
*/

#[cfg(test)]
mod tests {
    use super::*;

    

    #[test]
    fn test_samplestar() {
        // let file = "tests/basic.map";
        // let start = (1, 1);
        // let goal = (30, 30);
        // let file = "tests/map.map";
        // let start = (225, 225);
        // let goal = (70, 40);
        let file = "tests/wall/wall.map";
        let start = (3, 1);
        let goal = (3, 6);
        let mut grid = SampleGrid::new_from_file(file);
        grid.blur_samplegrid(5, 1.0);
        let path_store: Box<dyn PathStore<(usize, usize)>> = Box::new(AccStore::new());
        let mut samplestar = SampleStar::new(grid, start, goal, 10, 2, path_store);
        let visualiser = Visualiser::new("test", &samplestar.grid, Some(start), Some(goal));

        for i in 1..=100 {
            if samplestar.step() {
                break;
            }
            visualiser.visualise_iteration(&samplestar.grid, i, Some(samplestar.previous.clone()), Some(samplestar.current.clone()), samplestar.path_store.get_paths());
        }
        visualiser.visualise_final_path(&samplestar.final_path);
        assert!(false);
        // assert_eq!(samplestar.final_path.len(), 100);
    }
}