use crate::search::samplestarstats::SampleStarStats;
use domains::samplegrid::SampleGrid;
use domains::DomainCreate;
use heuristics::distance::manhattan_distance;
use maps::Problem;
use search::pathstore::{AccStore, GreedyStore};
use search::samplestar::{PathStoreT, SampleStar};
use std::time::Instant;
use util::matrix::{manhattan_dist_matrix, Matrix};
use util::{matrix::gaussian_kernal, visualiser::Visualiser};

mod domains;
mod fov;
mod heuristics;
mod maps;
mod search;
mod util;

fn main() {
    let now = Instant::now();
    for map in maps::MAP_PACK.into_iter() {
        run_sample_star(map, 10, 500);
    }
    run_sample_star(maps::FILL, 50, 50);
    println!("Time Taken: {}s", now.elapsed().as_secs_f32());
}

fn run_sample_star(map: Problem, epoch: usize, limit: usize) {
    let (name, file, start, goal) = map;
    let path_store: PathStoreT = Box::new(AccStore::new_count_store());
    let no_path_store: PathStoreT = 
        Box::new(GreedyStore::new(Box::new(move |n| manhattan_distance(*n, goal))));
    let mut samplestar =
        SampleStar::new(init_grid(file), start, goal, epoch, init_update_kernel(), path_store, no_path_store, init_stats());
    let visualiser = Visualiser::new(name, &samplestar.grid, Some(start), Some(goal));
    for i in 1..=limit {
        if samplestar.step() {
            break;
        }
        visualiser.visualise_iteration(
            Some(&samplestar.grid),
            i,
            Some(samplestar.previous.clone()),
            Some(samplestar.current.clone()),
            samplestar.get_path_store().visualise(),
            samplestar.stats.lock().unwrap().get_stats(),
        );
    }
    visualiser.visualise_final_path(&samplestar.final_path);
}

/// The initial grid to search on
fn init_grid(file: &str) -> SampleGrid {
    let mut grid = SampleGrid::new_from_file(file);
    grid.blur_samplegrid(&gaussian_kernal(5, 1.0));
    grid.sample_grid[0][0].state = 1.0; // Just to make sure
    grid.sample_grid[4][4].state = 1.0;
    grid
}

/// The kernel used to update the sample grid at each step
fn init_update_kernel() -> Matrix<f32> {
    let update_kernel = manhattan_dist_matrix(5, 5)
        .data
        .into_iter()
        .map(|x| (1.0 - 1.0 / (*x as f32))
        .clamp(0.0, 1.0))
        .collect();
    Matrix {
        data: update_kernel,
        width: 5,
        height: 5,
    }
}

/// The statistics to be collected during the search
fn init_stats() -> SampleStarStats {
    SampleStarStats::new(
        vec![
            (
                "AVG State".to_string(),
                Box::new(|grid, (x, y)| grid.sample_grid[*x][*y].state),
            ),
            (
                "AVG Var".to_string(),
                Box::new(|grid, (x, y)| grid.sample_grid[*x][*y].covariance),
            ),
        ],
        vec![
            (
                "MAX ADJ".to_string(),
                Box::new(|path_store, adj| {
                    *adj.iter()
                        .map(|n| path_store.get(n))
                        .filter(|n| n.is_some())
                        .map(|n| n.unwrap())
                        .max()
                        .unwrap_or(&0) as f32
                }),
            ),
            (
                "AVG ADJ".to_string(),
                Box::new(|path_store, adj| {
                    adj.iter()
                        .map(|n| path_store.get(n))
                        .filter(|n| n.is_some())
                        .map(|n| n.unwrap())
                        .sum::<usize>() as f32
                        / adj.len() as f32
                }),
            ),
        ],
    )
}