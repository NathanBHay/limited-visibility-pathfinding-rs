use domains::samplegrid::SampleGrid;
use search::pathstore::AccStore;
use search::samplestar::{PathStoreT, SampleStar, SampleStratT};
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

fn main() {
    let (file, start, goal) = maps::LAK;
    let new_from_file = SampleGrid::new_from_file(file);
    let mut grid = new_from_file;
    grid.blur_samplegrid(&gaussian_kernal(5, 1.0));
    let new_count_store = AccStore::new_count_store();
    let path_store: PathStoreT = Box::new(new_count_store);
    let sample_strat: SampleStratT = Box::new(
        |grid: &mut SampleGrid, current: (usize, usize), radius: usize| {
            grid.sample_radius(current, radius)
        },
    );
    let mut samplestar = SampleStar::new(
        grid,
        start,
        goal,
        10,
        gaussian_kernel_rev(5, 1.0),
        path_store,
        sample_strat,
    );
    let visualiser = Visualiser::new("test", &samplestar.grid, Some(start), Some(goal));

    for i in 1..=50 {
        if samplestar.step() {
            break;
        }
        visualiser.visualise_iteration(
            &samplestar.grid,
            i,
            Some(samplestar.previous.clone()),
            Some(samplestar.current.clone()),
            samplestar.path_store.get_paths(),
            None,
        );
    }
    visualiser.visualise_final_path(&samplestar.final_path);
}
