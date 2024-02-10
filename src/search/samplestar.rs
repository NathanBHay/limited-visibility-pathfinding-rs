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
    domains::{bitpackedgrid::BitPackedGrid, samplegrid::SampleGrid, Domain},
    util::{filter::KalmanNode, matrix::Matrix},
};

use super::{pathstore::PathStore, samplestarstats::SampleStarStats, BestSearch};

pub type PathStoreT = Box<dyn PathStore<(usize, usize), usize>>;

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
pub struct SampleStar<S: BestSearch<(usize, usize), usize> + Sync> {
    pub grid: SampleGrid,
    search: S,
    pub previous: (usize, usize),
    pub current: (usize, usize),
    pub goal: (usize, usize),
    epoch: usize,
    kernel: Matrix<f32>,
    pub final_path: Vec<(usize, usize)>,
    path_store: Arc<Mutex<PathStoreT>>,
    no_path_store: Arc<Mutex<PathStoreT>>,
    pub stats: Arc<Mutex<SampleStarStats>>,
}

impl<S: BestSearch<(usize, usize), usize> + Sync> SampleStar<S> {
    /// Creates a new SampleStar algorithm
    pub fn new(
        grid: SampleGrid,
        search: S,
        start: (usize, usize),
        goal: (usize, usize),
        epoch: usize,
        kernel: Matrix<f32>,
        path_store: PathStoreT,
        no_path_store: PathStoreT,
        stats: SampleStarStats,
    ) -> Self {
        assert!(grid.bound_check(start) && grid.bound_check(goal));
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
        // Keeping a seperate count should allow for less contention on the lock
        // as path_store.len() is unneccesary.
        let valid_paths = Arc::new(Mutex::new(0));
        (0..self.epoch).into_par_iter().for_each(|_| {
            let mut gridmap = BitPackedGrid::new(self.grid.width, self.grid.height);
            let mut sampled_before = gridmap.clone();
            let mut rng = rand::thread_rng(); // Incrementally improves performance
            let (path, weight) = self.search.best_search(
                |n| {
                    self.grid
                        .sample_adjacenct(&mut gridmap, &mut sampled_before, &mut rng, *n)
                },
                self.current,
                |n| *n == self.goal,
            );
            let found_path = path.last() == Some(&self.goal);
            let no_valid_paths = *valid_paths.lock().unwrap() == 0;
            if no_valid_paths && found_path { // This could be removed if you want to keep data
                self.no_path_store.lock().unwrap().reinitialize();
                self.stats.lock().unwrap().clear();
            }
            if found_path {
                *valid_paths.lock().unwrap() += 1;
            }
            {
                let mut stats = self.stats.lock().unwrap();
                stats.run_path_stats(&self.grid, &path);
                stats.add(1, sampled_before.count_ones() as f32);
                stats.add(2, weight as f32);
            }
            if found_path {
                self.path_store.lock().unwrap().add_path(path, weight);
            } else if no_valid_paths {
                self.no_path_store.lock().unwrap().add_path(path, weight);
            }
        });
        self.previous = self.current;
        let adj = self.grid.adjacent(self.current, false);
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

    /// Calculate the number of samples needed to achieve a confidence interval
    /// of Z and a margin of error of MARGIN_OF_ERROR. This possibly results
    /// in less samples than the statistical epoch.
    fn statistical_epoch(&self, sampling_states: Vec<KalmanNode>) -> usize {
        let (s, c) = sampling_states
            .iter()
            .map(|n| n.state)
            .filter(|n| n != &0.0 || n != &1.0)
            .fold((0.0, 0), |(s, c), x| (s + x, c + 1));
        let p: f32 = s / c as f32;
        (Self::DESIGN_EFFECT * p * (1.0 - p)) as usize
    }

    // Constant values for statistical epoch
    const Z: f32 = 1.96;
    const MARGIN_OF_ERROR: f32 = 0.05;
    const D: f32 = Self::Z / Self::MARGIN_OF_ERROR;
    const DESIGN_EFFECT: f32 = Self::D * Self::D;
}
