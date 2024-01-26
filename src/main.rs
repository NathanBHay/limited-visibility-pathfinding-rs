use domains::samplegrid::SampleGrid;
use search::pathstore::AccStore;
use search::samplestar::{PathStoreT, SampleStar};
use util::{
    matrix::{gaussian_kernal, gaussian_kernel_rev},
    visualiser::Visualiser,
};

mod domains;
mod fov;
mod heuristics;
mod maps;
mod search;
mod util;

// Goal to Improve 6s for 50 & 30-35 for 200
fn main() {
    let (file, start, goal) = maps::WALL;
    let new_from_file = SampleGrid::new_from_file(file);
    let mut grid = new_from_file;
    grid.blur_samplegrid(&gaussian_kernal(5, 1.0));
    let new_count_store = AccStore::new_count_store();
    let path_store: PathStoreT = Box::new(new_count_store);
    let mut samplestar = SampleStar::new(
        grid,
        start,
        goal,
        10,
        gaussian_kernel_rev(5, 1.0),
        path_store,
    );
    let visualiser = Visualiser::new("test", &samplestar.grid, Some(start), Some(goal));

    for i in 1..=50 {
        if samplestar.step() {
            break;
        }
        visualiser.visualise_iteration(
            None,
            i,
            Some(samplestar.previous.clone()),
            Some(samplestar.current.clone()),
            samplestar.path_store.get_paths(),
            None,
        );
    }
    visualiser.visualise_final_path(&samplestar.final_path);
}

#[test]
fn test_env() {
    use std::thread::available_parallelism;
    println!(
        "Available Parallelism: {}",
        available_parallelism().unwrap()
    );
    assert!(false);
}
