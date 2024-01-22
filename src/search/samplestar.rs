//! # SampleStar
//! A 
//! 
//! Possible Optimisations:
//! * Get min between epochs and amount of nodes in radius which can be sampled
//! * Cache paths
//! Heuristics could include ones that take into account probability of being an obstacle:
//! `self.grid.sample_grid[x][y].state * manhattan_distance(n*, self.goal)`

use crate::{domains::samplegrid::SampleGrid, heuristics::distance::manhattan_distance, util::{filter::KalmanNode, matrix::Matrix}};

use super::{pathstore::PathStore, focalsearch::focal_search};

type SampleStratT = Box<dyn FnMut(&mut SampleGrid, (usize, usize), usize)>;
type PathStoreT = Box<dyn PathStore<(usize, usize), usize>>;

/// Sample Star Algorithm
/// ## Arguments
/// * `grid` - The sampling grid to search on.
/// * `start` - The starting node.
/// * `goal` - The goal node.
/// * `epoch` - The number of times to sample each node.
/// * `radius` - The radius to sample around each node.
pub struct SampleStar {
    grid:  SampleGrid,
    previous: (usize, usize),
    current: (usize, usize),
    goal: (usize, usize),
    epoch: usize,
    kernel: Matrix<f32>,
    final_path: Vec<(usize, usize)>,
    path_store: PathStoreT,
    sample_stategy: SampleStratT,
}

impl SampleStar {

    /// Creates a new SampleStar algorithm
    pub fn new(
        grid: SampleGrid,
        start: (usize, usize), 
        goal: (usize, usize),
        epoch:usize,
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
            (self.sample_stategy)(&mut self.grid, self.current, self.kernel.width / 2);
            if let Some((path, weight)) = focal_search(
                |n| self.grid.gridmap.adjacent1(*n),
                self.current,
                |n| *n == self.goal,
                |n| manhattan_distance(*n, self.goal),
                |n| manhattan_distance(*n, self.goal),
                |n| *n,
            ) {
                self.path_store.add_path(Box::new(path.into_iter()), weight);
            }
        }
        self.previous = self.current;
        let adj: Box<dyn Iterator<Item = (usize, usize)>> = Box::new(self.grid.adjacent(self.current, false)
            .collect::<Vec<_>>()
            .into_iter());
        self.current = self.path_store.next_node(adj).unwrap_or(self.current);
        self.final_path.push(self.current);
        false
    }

    /// Calculate the number of samples needed to achieve a confidence interval
    /// of Z and a margin of error of MARGIN_OF_ERROR. This possibly results
    /// in less samples than the statistical epoch.
    fn statistical_epoch(&self, sampling_states: Vec<KalmanNode>) -> usize {
        let (s, c) = sampling_states.iter()
            .map(|n| n.state)
            .filter(|n| n != &0.0 || n != &1.0)
            .fold((0.0, 0), |(s, c), x| (s + x, c + 1));
        let p: f32 = s / c as f32;
        (Self::DESIGN_EFFECT * p * (1.0 - p)) as usize
    }

    // Constant values for statistical epoch    
    const Z : f32 = 1.96;
    const MARGIN_OF_ERROR : f32 = 0.05;
    const D : f32 = Self::Z / Self::MARGIN_OF_ERROR;
    const DESIGN_EFFECT : f32 = Self::D * Self::D;
}

/*
Premade Sample Strategies
|grid, _, _| grid.sample_all()
|grid, current, radius| grid.sample_radius(current, radius)
*/

#[cfg(test)]
mod tests {

    use super::*;
    use std::collections::HashMap;
    use crate::{util::{visualiser::Visualiser, matrix::gaussian_kernal}, search::pathstore::AccStore,};
    
    #[test]
    fn test_samplestar() {
        let (file, start, goal) = maps::BASIC;
        let mut grid = SampleGrid::new_from_file(file);
        grid.blur_samplegrid(&gaussian_kernal(5, 1.0));
        let path_store: PathStoreT = Box::new(AccStore::new_count_store());
        let sample_strat: SampleStratT = Box::new(|grid: &mut SampleGrid, current: (usize, usize), radius: usize| grid.sample_radius(current, radius));
        let mut samplestar = SampleStar::new(grid, start, goal, 10, gaussian_kernal(5, 1.0), path_store, sample_strat);
        let visualiser = Visualiser::new("test", &samplestar.grid, Some(start), Some(goal));

        for i in 1..=10000 {
            if samplestar.step() {
                break;
            }
            visualiser.visualise_iteration(&samplestar.grid, i, Some(samplestar.previous.clone()), Some(samplestar.current.clone()), samplestar.path_store.get_paths());
        }
        visualiser.visualise_final_path(&samplestar.final_path);
    }
}

mod maps {
    type Map = (&'static str, (usize, usize), (usize, usize));
    pub const BASIC: Map =  ("tests/basic.map", (1, 1), (30, 30));
    pub const MAP: Map =  ("tests/map.map", (225, 225), (70, 40));
    pub const WALL: Map =  ("tests/wall/wall.map", (3, 1), (3, 6));
    pub const CACAVERNS: Map =  ("tests/ca_caverns1.map", (122, 595), (200, 15));
    pub const DRYWATER: Map =  ("tests/drywatergulch.map", (175, 315), (320, 125));
    pub const FLOODEDPLAINS: Map =  ("tests/FloodedPlains.map", (160, 100), (480, 330));
    pub const HRT: Map =  ("tests/hrt201d.map", (70, 28), (250, 235));
    pub const LAK: Map =  ("tests/lak201d.map", (30, 150), (100, 50));
    pub const MAZE: Map =  ("tests/maze512-8-4.map", (10, 10), (380, 325));
    pub const MEDUSA: Map =  ("tests/Medusa.map", (60, 250), (460, 20));
    pub const SIROCCO: Map =  ("tests/Sirocco.map", (10, 250), (750, 250));
    pub const TRISKELION: Map =  ("tests/Triskelion.map", (260, 500), (10, 10));
    pub const WAYPOINTJUNCTION: Map =  ("tests/WaypointJunction.map", (245, 20), (260, 500));
}