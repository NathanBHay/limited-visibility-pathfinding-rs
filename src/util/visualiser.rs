use serde_json::json;
use std::{collections::HashMap, fs::File};

use crate::domains::neighbors;
use crate::domains::{
    adjacencylist::AdjacencyList, bitpackedgrid::BitPackedGrid, samplegrid::SampleGrid,
};

/// Visualiser tool for `SampleGrid`
///
pub struct Visualiser {
    file_path: String,
    start: Option<(usize, usize)>,
    goal: Option<(usize, usize)>,
}

impl Visualiser {
    /// Create a new visualiser for a `SampleGrid` with a start and goal
    pub fn new(
        file_path: &str,
        sample_grid: &SampleGrid,
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

    /// Visualise the ground truth of the grid
    fn visualise_ground_truth(&self, grid: &BitPackedGrid) {
        let mut ground_truth = vec![vec![false; grid.original_height]; grid.original_width];
        for x in 0..grid.original_width {
            for y in 0..grid.original_height {
                ground_truth[x][y] = grid.get_bit_value((x, y));
            }
        }
        let ground_truth = json!({
            "grid": ground_truth,
            "start": self.start,
            "goal": self.goal,
        });
        let mut file = File::create(format!("{}_ground_truth.json", self.file_path)).unwrap();
        serde_json::to_writer_pretty(&mut file, &ground_truth).unwrap();
    }

    /// Visualise the current state of the grid and found paths
    pub fn visualise_iteration(
        &self,
        sample_grid: Option<&SampleGrid>,
        iteration: usize,
        current: Option<(usize, usize)>,
        next: Option<(usize, usize)>,
        paths: &HashMap<(usize, usize), usize>,
        stats: Option<Vec<(&str, &str)>>,
    ) {
        let sample_grid = json!({
            "sample_grid": get_sample_grid(sample_grid),
            "current": current,
            "next": next,
            "paths": hashmap_to_adjlist(paths).iter().collect::<Vec<_>>(),
            "stats": stats,
        });
        let mut file = File::create(format!("{}_step_{}.json", self.file_path, iteration)).unwrap();
        serde_json::to_writer_pretty(&mut file, &sample_grid).unwrap();
    }

    /// Visualise the final path found by the algorithm
    pub fn visualise_final_path(&self, final_path: &Vec<(usize, usize)>) {
        let mut paths = HashMap::new();
        for node in final_path {
            *paths.entry(*node).or_insert(0) += 1;
        }
        let sample_grid = json!({
            "path": hashmap_to_adjlist(&paths).iter().collect::<Vec<_>>(),
        });
        let mut file = File::create(format!("{}_final_path.json", self.file_path)).unwrap();
        serde_json::to_writer_pretty(&mut file, &sample_grid).unwrap();
    }
}

fn get_sample_grid(grid: Option<&SampleGrid>) -> Vec<Vec<f32>> {
    match grid {
        Some(grid) => {
            // TODO: Optimize this to copy directly from the grid
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

fn hashmap_to_adjlist(map: &HashMap<(usize, usize), usize>) -> AdjacencyList<(usize, usize), f32> {
    let mut adjlist = AdjacencyList::new();
    let mut max = 0;
    for ((x, y), w) in map.iter() {
        adjlist.add_node((*x, *y));
        for (dest_x, dest_y) in neighbors(*x, *y, false) {
            if let Some(dest_w) = map.get(&(dest_x, dest_y)) {
                adjlist.add_edge((*x, *y), (dest_x, dest_y), *w.min(dest_w) as f32);
                if *w > max {
                    max = *w;
                }
            }
        }
    }
    for node in adjlist.iter_mut() {
        for (_, w) in node.iter_mut() {
            *w = *w / max as f32;
        }
    }
    adjlist
}
