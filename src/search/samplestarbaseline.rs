//! # SampleStar
//! A
//!
//! Possible Optimisations:
//! * Get min between epochs and amount of nodes in radius which can be sampled
//! * Cache paths
//! Heuristics could include ones that take into account probability of being an obstacle:
//! `self.grid.sample_grid[x][y].state * manhattan_distance(n*, self.goal)`

use rayon::prelude::*;
use std::sync::{Arc, Mutex, MutexGuard};

use crate::{
    domains::{bitpackedgrids::{bitpackedgrid2d::BitPackedGrid2d, BitPackedGrid}, samplegrids::samplegrid2d::SampleGrid2d, Domain, Grid2d},
    util::matrix::Matrix,
};

use super::{
    pathstore::PathStore, samplestar::PathStoreT, samplestarstats::SampleStarStats, BestSearch,
};

/// Sample Star Algorithm
/// ## Arguments
/// * `grid` - The sampling grid to search on.
/// * `search` - The search algorithm to use.
/// * `start` - The start node.
/// * `goal` - The goal node.
/// * `epoch` - The number of times to sample each node.
/// * `kernel` - The kernel to sample with.
/// * `path_store` - The path store to store the paths.
/// * `no_path_store` - The path store to store the paths that don't reach the goal
/// * `stats` - The statistics store to store the stats. Currently built into the
/// algorithm however it could be moved out. This is due to the fact search results
/// aren't stored so stats are calculated on the fly.
pub struct SampleStarBaseline<S: BestSearch<(usize, usize), usize> + Sync> {
    pub grid: SampleGrid2d,
    search: S,
    pub previous: (usize, usize),
    pub current: (usize, usize),
    pub goal: (usize, usize),
    epoch: usize,
    kernel: Matrix<f32>,
    pub final_path: Vec<(usize, usize)>,
    path_store: Arc<Mutex<PathStoreT<(usize, usize)>>>,
    no_path_store: Arc<Mutex<PathStoreT<(usize, usize)>>>,
    pub stats: Arc<Mutex<SampleStarStats<(usize, usize)>>>,
    pub sampled_before: BitPackedGrid2d,
}

impl<S: BestSearch<(usize, usize), usize> + Sync> SampleStarBaseline<S> {
    /// Creates a new SampleStar algorithm
    pub fn new(
        grid: SampleGrid2d,
        search: S,
        start: (usize, usize),
        goal: (usize, usize),
        epoch: usize,
        kernel: Matrix<f32>,
        path_store: PathStoreT<(usize, usize)>,
        no_path_store: PathStoreT<(usize, usize)>,
        stats: SampleStarStats<(usize, usize)>,
    ) -> Self {
        assert!(grid.bounds_check(start) && grid.bounds_check(goal));
        let (width, height) = grid.shape();
        Self {
            grid,
            search,
            previous: start,
            current: start,
            goal,
            epoch,
            kernel,
            final_path: vec![start],
            path_store: Arc::new(Mutex::new(path_store)),
            no_path_store: Arc::new(Mutex::new(no_path_store)),
            stats: Arc::new(Mutex::new(stats)),
            sampled_before: BitPackedGrid2d::new((width, height)),
        }
    }

    /// Run the algorithm for one step
    pub fn step(&mut self) -> bool {
        if self.current == self.goal {
            return true;
        }
        self.path_store.lock().unwrap().reinitialize();
        self.no_path_store.lock().unwrap().reinitialize();
        self.stats.lock().unwrap().clear();
        self.grid.raycast_update(self.current, &self.kernel);
        self.sampled_before.raycast_set_radius(&self.grid, self.current, 2, true);
        // Keeping a seperate count should allow for less contention on the lock
        // as path_store.len() is unneccesary.
        let valid_paths = Arc::new(Mutex::new(0));
        (0..self.epoch).into_par_iter().for_each(|_| {
            let mut gridmap = BitPackedGrid2d::new((self.grid.width, self.grid.height));
            gridmap.invert();
            self.grid.sample_based_on_grid(&mut gridmap, &self.sampled_before);
            let (path, weight) = self.search.best_search(
                |n| gridmap.adjacent(*n, false).map(|n| (n, 1)).collect::<Vec<_>>(),
                self.current,
                |n| !self.sampled_before.get_value(*n) || *n == self.goal,
            );
            let found_path = path.last() == Some(&self.goal);
            let no_valid_paths = *valid_paths.lock().unwrap() == 0;
            if no_valid_paths && found_path {
                // This could be removed if you want to keep data
                self.no_path_store.lock().unwrap().reinitialize();
                self.stats.lock().unwrap().clear();
            }
            if found_path {
                *valid_paths.lock().unwrap() += 1;
            }
            {
                let mut stats = self.stats.lock().unwrap();
                stats.run_path_stats(&self.grid, &path);
                stats.add(1, self.sampled_before.count_ones() as f32);
                stats.add(2, weight as f32);
            }
            if found_path {
                self.path_store.lock().unwrap().add_path(path, weight);
            } else if no_valid_paths {
                self.no_path_store.lock().unwrap().add_path(path, weight);
            }
        });
        self.previous = self.current;
        let adj = self.grid.adjacent(self.current, false).collect();
        let valid_paths = valid_paths.lock().unwrap();
        let path_store = if *valid_paths > 0 {
            self.path_store.lock().unwrap()
        } else {
            self.no_path_store.lock().unwrap()
        };
        let mut stats = self.stats.lock().unwrap();
        stats.add(0, *valid_paths as f32);
        stats.collate_path_stats(*valid_paths);
        stats.run_step_stats(&path_store, &adj);
        self.current = path_store.next_node(adj).unwrap_or(self.current);

        // Bump mechanics are done to avoid walking into walls. This is necessary as the
        // kalman updating procedure doesn't overide the value of adjacenct cells. This
        // means there are cases where the path store will return a path that walks into
        // a wall. To change from bump mechanics to override mechanics delete this and
        // add code that sets the adjacent cells to the ground truth.
        if !self.grid.ground_truth.get_value(self.current) {
            self.current = self.previous;
        }
        self.final_path.push(self.current);
        false
    }

    /// Get the path store that should be used
    pub fn get_path_store(&self) -> MutexGuard<'_, Box<dyn PathStore<(usize, usize), usize>>> {
        if self.path_store.lock().unwrap().len() > 0 {
            self.path_store.lock().unwrap()
        } else {
            self.no_path_store.lock().unwrap()
        }
    }
}
