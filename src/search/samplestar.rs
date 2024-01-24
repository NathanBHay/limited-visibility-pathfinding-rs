//! # SampleStar
//! A
//!
//! Possible Optimisations:
//! * Get min between epochs and amount of nodes in radius which can be sampled
//! * Cache paths
//! Heuristics could include ones that take into account probability of being an obstacle:
//! `self.grid.sample_grid[x][y].state * manhattan_distance(n*, self.goal)`

use crate::{
    domains::samplegrid::SampleGrid,
    heuristics::distance::manhattan_distance,
    util::{filter::KalmanNode, matrix::Matrix},
};

use super::{focalsearch::focal_search, pathstore::PathStore};

pub type SampleStratT = Box<dyn FnMut(&mut SampleGrid, (usize, usize), usize)>;
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
    pub path_store: PathStoreT,
    sample_stategy: SampleStratT,
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
        sample_stategy: SampleStratT,
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
            path_store,
            sample_stategy,
        }
    }

    /// Run the algorithm for one step
    pub fn step(&mut self) -> bool {
        if self.current == self.goal {
            return true;
        }
        self.path_store.reinitialize();
        // let mut cached_paths = HashMap::new();
        self.grid.raycast_update(self.current, &self.kernel);
        self.grid.init_gridmap_nearest(); // Currently more accurate than other methods
        for _ in 0..self.epoch {
            // (self.sample_stategy)(&mut self.grid, self.current, self.kernel.width /
            self.grid.init_sampled_before();
            if let Some((path, weight)) = focal_search(
                |n| self.grid.sample_adjacenct(*n).collect::<Vec<_>>(),
                self.current,
                |n| *n == self.goal,
                |n| manhattan_distance(*n, self.goal),
                |_| 0,
                |n| *n,
            ) {
                self.path_store.add_path(Box::new(path.into_iter()), weight);
            }
        }
        self.previous = self.current;
        let adj: Box<dyn Iterator<Item = (usize, usize)>> = Box::new(
            self.grid
                .adjacent(self.current, false)
                .collect::<Vec<_>>()
                .into_iter(),
        );
        self.current = self.path_store.next_node(adj).unwrap_or(self.current);
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

/*
Premade Sample Strategies
|grid, _, _| grid.sample_all()
|grid, current, radius| grid.sample_radius(current, radius)
*/
