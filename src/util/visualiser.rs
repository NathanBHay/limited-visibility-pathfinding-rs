//! Visualiser tool for visualising `Sample Star` algorithm. This is used to output
//! the data in JSON which is usable by the python visualiser.
//! Future optimisation could be to output in BinCode format as it is interpertable by python.
//! This would allow for faster output of the data.
use ahash::AHashMap;
use serde_json::json;
use std::{
    fs::File,
    io::{BufWriter, Write},
};

use crate::domains::{bitpackedgrids::bitpackedgrid2d::BitPackedGrid2d, samplegrids::samplegrid2d::SampleGrid2d, GridDomain};

/// Visualiser tool for visualising `Sample Star` algorithm.
/// Outputs to JSON format for use with the python visualiser.
pub struct Visualiser {
    file_path: String,
    start: Option<(usize, usize)>,
    goal: Option<(usize, usize)>,
}

impl Visualiser {
    /// Create a new visualiser for a `SampleGrid` with a start and goal
    pub fn new(
        file_path: &str,
        sample_grid: &SampleGrid2d,
        start: Option<(usize, usize)>,
        goal: Option<(usize, usize)>,
    ) -> Self {
        let visualiser = Visualiser {
            file_path: file_path.to_string(),
            start,
            goal,
        };
        visualiser.visualise_ground_truth(&sample_grid.ground_truth);
        visualiser
    }

    pub fn visualise_bitpacked_grid(&self, grid: &BitPackedGrid2d, name: &str) {
        let mut bitpacked_grid = vec![vec![false; grid.height]; grid.width];
        for x in 0..grid.width {
            for y in 0..grid.height {
                bitpacked_grid[x][y] = grid.get_value((x, y));
            }
        }
        let bitpacked_grid = json!({
            "grid": bitpacked_grid,
            "start": self.start,
            "goal": self.goal,
        });
        let mut file = File::create(format!("{}_{}.json", self.file_path, name)).unwrap();
        serde_json::to_writer(&mut file, &bitpacked_grid).unwrap();
    }

    /// Visualise the ground truth of the grid
    fn visualise_ground_truth(&self, grid: &BitPackedGrid2d) {
        self.visualise_bitpacked_grid(grid, "ground_truth")
    }
    
    /// Visualise the current state of the grid and found paths
    pub fn visualise_iteration(
        &self,
        sample_grid: Option<&SampleGrid2d>,
        iteration: usize,
        current: Option<(usize, usize)>,
        next: Option<(usize, usize)>,
        paths: Vec<((usize, usize), usize)>,
        stats: Vec<String>,
    ) {
        let sample_grid = json!({
            "sample_grid": get_sample_grid(sample_grid),
            "current": current,
            "next": next,
            "paths": create_pathlist(paths),
            "stats": stats,
        });
        let mut file = BufWriter::new(
            File::create(format!("{}_step_{}.json", self.file_path, iteration)).unwrap(),
        );
        serde_json::to_writer(&mut file, &sample_grid).unwrap();
        let _ = file.flush();
    }

    /// Visualise the final path found by the algorithm
    pub fn visualise_final_path(&self, final_path: &Vec<(usize, usize)>) {
        let mut paths = AHashMap::new();
        for i in 0..final_path.len() - 1 {
            let node = (
                final_path[i].min(final_path[i + 1]),
                final_path[i].max(final_path[i + 1]),
            );
            *paths.entry(node).or_insert(0) += 1;
        }
        let sample_grid = json!({
            "path": create_pathlist(paths.into_iter().collect()),
            "length": final_path.len(),
        });
        let mut file = File::create(format!("{}_final_path.json", self.file_path)).unwrap();
        serde_json::to_writer(&mut file, &sample_grid).unwrap();
    }
}

fn get_sample_grid(grid: Option<&SampleGrid2d>) -> Vec<Vec<f32>> {
    match grid {
        Some(grid) => {
            // TODO: Optimize this to copy directly from the grid
            // Do this by changing python code to accept a array and format it using w & h
            let mut sample_grid = vec![vec![0.0; grid.height]; grid.width];
            for x in 0..grid.width {
                for y in 0..grid.height {
                    sample_grid[x][y] = grid.sample_grid[x][y].state;
                }
            }
            sample_grid
        }
        _ => Vec::new(),
    }
}

fn create_pathlist<K: Clone>(map: Vec<(K, usize)>) -> Vec<(K, f32)> {
    let max = map.iter().map(|(_, w)| *w).max().unwrap_or(1) as f32;
    map.iter()
        .map(|(k, w)| (k.clone(), *w as f32 / max))
        .collect()
}
