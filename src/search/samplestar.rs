//! # SampleStar
//! A
//!
//! Possible Optimisations:
//! * Get min between epochs and amount of nodes in radius which can be sampled
//! * Cache paths
//! Heuristics could include ones that take into account probability of being an obstacle:
//! `self.grid.sample_grid[x][y].state * manhattan_distance(n*, self.goal)`

use std::sync::{Arc, Mutex};
use rayon::prelude::*;

use crate::{
    domains::{bitpackedgrid::BitPackedGrid, samplegrid::SampleGrid}, heuristics::distance::manhattan_distance, util::{filter::KalmanNode, matrix::Matrix}
};

use super::{focalsearch::focal_search, pathstore::PathStore, samplestarstats::SampleStarStats};

pub type PathStoreT = Box<dyn PathStore<(usize, usize), usize>>;

/// Sample Star Algorithm
/// ## Arguments
/// * `grid` - The sampling grid to search on.
/// * `start` - The starting node.
/// * `goal` - The goal node.
/// * `epoch` - The number of times to sample each node.
/// * `radius` - The radius to sample around each node.
pub struct SampleStar {
    pub grid: SampleGrid,
    pub previous: (usize, usize),
    pub current: (usize, usize),
    pub goal: (usize, usize),
    epoch: usize,
    kernel: Matrix<f32>,
    pub final_path: Vec<(usize, usize)>,
    pub path_store: Arc<Mutex<PathStoreT>>,
    pub stats: Arc<Mutex<SampleStarStats>>,
    break_flag: usize,
}

impl SampleStar {
    /// Creates a new SampleStar algorithm
    pub fn new(
        grid: SampleGrid,
        start: (usize, usize),
        goal: (usize, usize),
        epoch: usize,
        kernel: Matrix<f32>,
        path_store: PathStoreT,
        stats: SampleStarStats,
    ) -> Self {
        assert!(grid.bound_check(start) && grid.bound_check(goal));
        Self {
            grid,
            previous: start,
            current: start,
            goal,
            epoch,
            kernel,
            final_path: vec![start],
            path_store: Arc::new(Mutex::new(path_store)),
            stats: Arc::new(Mutex::new(stats)),
            break_flag: 0,
        }
    }

    /// Run the algorithm for one step
    pub fn step(&mut self) -> bool {
        if self.current == self.goal {
            return true;
        }
        {
            self.path_store.lock().unwrap().reinitialize();
            self.stats.lock().unwrap().clear();
        }
        self.grid.raycast_update(self.current, &self.kernel);
        let valid_paths = Arc::new(Mutex::new(0));
        (0..self.epoch).into_par_iter().for_each(|_| {
            let mut gridmap = BitPackedGrid::new(self.grid.width, self.grid.height);
            let mut sampled_before = gridmap.clone();
            if let Some((path, weight)) = focal_search(
                |n| self.grid.sample_adjacenct(&mut gridmap, &mut sampled_before, *n),
                self.current,
                |n| *n == self.goal,
                |n| manhattan_distance(*n, self.goal),
                |(x, y)| 0,
                |n| 0,
            ) {
                {
                    let mut stats = self.stats.lock().unwrap();
                    stats.run_path_stats(&self.grid, &path);
                    stats.add(1, sampled_before.count_ones() as f32);
                    stats.add(2, weight as f32);
                    *valid_paths.lock().unwrap() += 1;
                }
                {
                    self.path_store.lock().unwrap().add_path(path, weight);
                }
            }
        });
        self.previous = self.current;
        let adj = self.grid
            .adjacent(self.current, false)
            .collect::<Vec<_>>();
        let path_store = self.path_store.lock().unwrap();
        if path_store.get(&self.current).is_none() {
            self.break_flag += 1;
            if self.break_flag > 10 {
                return true;
            }
        } else {
            self.break_flag = 0;
        }
        let mut stats = self.stats.lock().unwrap();
        let valid_paths = valid_paths.lock().unwrap();
        stats.add(0, *valid_paths as f32);
        stats.collate_path_stats(*valid_paths);
        stats.run_step_stats(&path_store, &adj);
        self.current = path_store.next_node(adj).unwrap_or(self.current);
        self.final_path.push(self.current);
        false
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
