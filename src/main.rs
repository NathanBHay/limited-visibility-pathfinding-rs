mod domains;
mod heuristics;
mod search;
mod util;
mod fov;
mod gridpolicies;

fn main() {
}


#[cfg(test)]
mod tests {

    #[test]
    fn run_test() {
        let x = None;
        let y = Some(2);
        println!("{:?}", y.min(x));
    }

    // #[test]
    // fn run_basic_search() {
    //     let grid = BitPackedGrid::new_from_file("map.map");
    //     let mut expanded_nodes = HashSet::new();
    //     let path = astar_with_expanded_set(
    //         |(x, y)| grid.adjacent((x.clone(), y.clone()), false).map(|(x, y)| ((x, y), 1)), 
    //         (225, 225),
    //         |n| n == &(70, 40),
    //         Some(&mut expanded_nodes),
    //         |(x, y)| manhattan_distance((*x, *y), (70, 40)), 
    //     ).map(|(path, _)| path);
    //     let expanded_nodes = expanded_nodes.iter().map(|n| (*n, 0.1)).collect::<Vec<_>>();
    //     grid.plot_cells("test1.png", path, Some(expanded_nodes));
    // }
}
