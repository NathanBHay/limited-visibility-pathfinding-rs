use domains::samplegrid::SampleGrid;
use maps::Problem;
use search::pathstore::AccStore;
use search::samplestar::{PathStoreT, SampleStar};
use util::{
    matrix::{gaussian_kernal, gaussian_kernel_rev},
    visualiser::Visualiser,
};
use std::time::Instant;
use crate::search::samplestarstats::SampleStarStats;

mod domains;
mod fov;
mod heuristics;
mod maps;
mod search;
mod util;

// Goal to Improve 6s for 50 & 30-35 for 200
fn main() {
    let now = Instant::now();
    for map in maps::MAP_PACK.into_iter() {
        run_sample_star(map, 10, 500);
    }
    println!("Time Taken: {}s", now.elapsed().as_secs_f32());
}

fn run_sample_star(map: Problem, epoch: usize, limit: usize) {
    let (name, file, start, goal) = map;
    let new_from_file = SampleGrid::new_from_file(file);
    let mut grid = new_from_file;
    grid.blur_samplegrid(&gaussian_kernal(5, 1.0));
    let new_count_store = AccStore::new_count_store();
    let path_store: PathStoreT = Box::new(new_count_store);
    let stats = SampleStarStats::new(
        vec![
            ("AVG State".to_string(), Box::new(|grid, (x, y)| grid.sample_grid[*x][*y].state)),
            ("AVG Var".to_string(), Box::new(|grid, (x, y)| grid.sample_grid[*x][*y].covariance)),
        ],
        vec![
            ("MAX ADJ".to_string(), Box::new(|path_store, adj| *adj.iter().map(|n| path_store.get(n)).filter(|n| n.is_some()).map(|n| n.unwrap()).max().unwrap_or(&0) as f32)),
            ("AVG ADJ".to_string(), Box::new(|path_store, adj| adj.iter().map(|n| path_store.get(n)).filter(|n| n.is_some()).map(|n| n.unwrap()).sum::<usize>() as f32 / adj.len() as f32))
        ],
    );
    let mut samplestar = SampleStar::new(
        grid,
        start,
        goal,
        epoch,
        gaussian_kernel_rev(5, 1.0),
        path_store,
        stats
    );
    let visualiser = Visualiser::new(name, &samplestar.grid, Some(start), Some(goal));
    for i in 1..=limit {
        if samplestar.step() {
            break;
        }
        visualiser.visualise_iteration(
            None,
            i,
            Some(samplestar.previous.clone()),
            Some(samplestar.current.clone()),
            samplestar.path_store.lock().unwrap().get_paths(),
            samplestar.stats.lock().unwrap().get_stats(),
        );
    }
    visualiser.visualise_final_path(&samplestar.final_path);
}
