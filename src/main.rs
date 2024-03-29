use crate::search::samplestarstats::SampleStarStats;
use domains::samplegrids::samplegrid2d::SampleGrid2d;
use domains::GridCreate2d;
use heuristics::distance::manhattan_distance;
use maps::Problem;
use search::astar::AStar;
use search::focalsearch::FocalSearch;
use search::pathstore::{AccStore, GreedyStore};
use search::samplestar::{PathStoreT, SampleStar};
use search::BestSearch;
use std::sync::Arc;
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
    // for map in maps::MAP_PACK.into_iter() {
    //     run_sample_star(map, 10, 500);
    // }
    run_sample_star(maps::LAK, 100, 100);
    println!("Time Taken: {}s", now.elapsed().as_secs_f32());
}

fn run_sample_star(map: Problem, epoch: usize, limit: usize) {
    let (name, file, start, goal) = map;
    let path_store: PathStoreT<(usize, usize)> = Box::new(AccStore::new_count_store());
    let no_path_store: PathStoreT<(usize, usize)> = Box::new(GreedyStore::new(Box::new(move |n| {
        manhattan_distance(*n, goal)
    })));
    // You can alternate betweeen AStar and FocalSearch
    let search = AStar::new(Arc::new(move |x| manhattan_distance(*x, goal)), Arc::new(|_| (0, 0)));
    // let search = FocalSearch::new(
    //     Arc::new(move |x| manhattan_distance(*x, goal)),
    //     Arc::new(|_| (0, 0))
    //     Arc::new(|_| 0),
    //     Arc::new(move |_| 0),
    // );
    let mut samplestar = SampleStar::new(
        init_grid(file, start, goal),
        search,
        start,
        goal,
        epoch,
        init_update_kernel(),
        path_store,
        no_path_store,
        init_stats(),
    );
    let visualiser = Visualiser::new(&format!("out/{}", name), &samplestar.grid, Some(start), Some(goal));
    for i in 1..=limit {
        if samplestar.step() {
            break;
        }
        // Specific visualisations for samplestar baseline that show visiblity
        // visualiser
        //     .visualise_bitpacked_grid(&samplestar.sampled_before, &format!("sampled_before_{}", i));
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
fn init_grid(
    file: &str,
    (start_x, start_y): (usize, usize),
    (goal_x, goal_y): (usize, usize),
) -> SampleGrid2d {
    let mut grid = SampleGrid2d::new_from_file(file);
    grid.blur_samplegrid(&gaussian_kernal(3, 1.0));
    grid.sample_grid[start_x][start_y].state = 1.0; // Just to make sure
    grid.sample_grid[goal_x][goal_y].state = 1.0;
    grid
}

/// The kernel used to update the sample grid at each step
fn init_update_kernel() -> Matrix<f32> {
    let update_kernel = manhattan_dist_matrix(5, 5)
        .data
        .into_iter()
        .map(|x| (1.0 - 1.0 / (*x as f32)).clamp(0.0, 1.0))
        .collect();
    Matrix {
        data: update_kernel,
        width: 5,
        height: 5,
    }
}

/// The statistics to be collected during the search
fn init_stats() -> SampleStarStats<(usize, usize)> {
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
