//! # Domains
//! Domains that can be used with search algorithms. These domains include:
//! * BitPackedGrid, a grid map that uses 1 bit per cell
//! * HashedGrid, a grid map that uses a hash map to store the map
//! * AdjacencyList, a graph representation of a map
//! * SampleGrid, a grid map that uses a hash map to store the map and has a chance of being occupied

#![allow(dead_code)]
pub mod adjacencylist;
pub mod bitpackedgrid;
pub mod edgelist;
pub mod hashedgrid;
pub mod samplegrid;

/// A helper function that creates a map from a string given functions
/// to initialize the map and add obstacles
/// ## Arguments
/// * `s` - A string representing the grid map
/// * `initialize` - A function that initializes the map
/// * `add_obstacle` - A function that adds an obstacle to the map
/// ## Returns
/// A map with a domain equal to the return type of initialize
pub(crate) fn create_map_from_string<F, D, I>(
    s: String,
    mut initialize: I,
    mut add_obstacle: F,
) -> D
where
    I: FnMut(usize, usize) -> D,
    F: FnMut(&mut D, usize, usize) -> (),
{
    let s = s.trim();
    let start = s.find(|c: char| ['.', '@', 'T'].contains(&c));
    let s = match start {
        Some(start) => &s[start..],
        None => "",
    };
    let height = s.lines().count();
    let width = s.lines().next().map(|x| x.len()).unwrap();
    let mut domain = initialize(width, height);
    for (i, line) in s.lines().enumerate() {
        for (j, c) in line.chars().enumerate() {
            if c == '.' {
                add_obstacle(&mut domain, j, i);
            }
        }
    }
    domain
}

/// A helper function that prints a map given a function to get the value of a cell
/// ## Arguments
/// * `width` - The width of the map
/// * `height` - The height of the map
/// * `get_cell_value` - A function that returns the value of a cell given its x, y coordinates,
/// true if the cell is free and false if it is an obstacle
/// ## Returns
/// A string representing the map where . is a free cell and @ is an obstacle
pub(crate) fn print_cells(
    width: usize,
    height: usize,
    get_cell_value: impl Fn(usize, usize) -> bool,
    path: Option<Vec<(usize, usize)>>,
) -> String {
    let mut s = String::new();
    for y in 0..height {
        for x in 0..width {
            if get_cell_value(x, y) {
                s.push('.');
            } else {
                s.push('@');
            }
        }
        s.push('\n');
    }
    if let Some(path) = path {
        for (x, y) in path {
            s.replace_range(y * (width + 1) + x..y * (width + 1) + x + 1, "*");
        }
    }
    s
}

/// A debuging function which prints a vector of points
pub fn print_points(cells: Vec<(usize, usize)>) -> String {
    let width_max = cells.iter().map(|(x, _)| x).max().unwrap() + 1;
    let width_min = cells.iter().map(|(x, _)| x).min().unwrap();
    let height_max = cells.iter().map(|(_, y)| y).max().unwrap() + 1;
    let height_min = cells.iter().map(|(_, y)| y).min().unwrap();
    print_cells(
        width_max - width_min,
        height_max - height_min,
        |x, y| cells.contains(&(x + width_min, y + height_min)),
        None,
    )
}

/// Helper function to get a iterator of the neighbors of a cell
pub fn neighbors(x: usize, y: usize, diagonal: bool) -> impl Iterator<Item = (usize, usize)> {
    let direct = vec![
        (x.wrapping_add(1), y), // Theses are wrapping as to avoid branching
        (x, y.wrapping_add(1)), // on BitPacked Grids
        (x.wrapping_sub(1), y),
        (x, y.wrapping_sub(1)),
    ]
    .into_iter();

    let diagonal = if diagonal {
        vec![
            (x.wrapping_add(1), y.wrapping_add(1)),
            (x.wrapping_sub(1), y.wrapping_add(1)),
            (x.wrapping_add(1), y.wrapping_sub(1)),
            (x.wrapping_sub(1), y.wrapping_sub(1)),
        ]
        .into_iter()
    } else {
        Vec::new().into_iter()
    };

    direct.chain(diagonal)
}
