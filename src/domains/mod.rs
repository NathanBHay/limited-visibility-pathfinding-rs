//! # Domains
//! Domains that can be used with search algorithms. These domains include:
//! * BitPackedGrid, a grid map that uses 1 bit per cell
//! * HashedGrid, a grid map that uses a hash map to store the map
//! * AdjacencyList, a graph representation of a map
//! * SampleGrid, a grid map that uses a hash map to store the map and has a chance of being occupied

#![allow(dead_code)]

use std::{fs::read_to_string, ops::Range};

pub mod adjacencylist;
pub mod bitpackedgrid;
pub mod edgelist;
pub mod hashedgrid;
pub mod samplegrid;

/// Trait that represents a domain that can be used with search algorithms.
/// A domain is a map of cells where each cell can be traversable or an obstacle.
pub trait Domain {
    /// Creates a new map with a given width and height
    fn new(width: usize, height: usize) -> Self;

    /// Sets the value of a cell in a map. True if the cell is traversable and 
    /// false if it is an obstacle.
    fn set_value(&mut self, n: (usize, usize), value: bool);

    /// Gets the value of a cell in a map. True if the cell is traversable and
    /// false if it is an obstacle.
    fn get_value(&self, n: (usize, usize)) -> bool;

    /// Get shape of the data listed in width, height format.
    fn shape(&self) -> (usize, usize);
}

/// Trait used to create domains from files and strings.
pub trait DomainCreate: Domain + Sized {
    /// Create a new domain from a string, where . is used to represent traversable space
    /// and all other characters represent terrain.
    fn new_from_string(s: String) -> Self {
        let s = s.trim();
        let start = s.find(|c: char| ['.', '@', 'T'].contains(&c));
        let s = match start {
            Some(start) => &s[start..],
            None => "",
        };
        let height = s.lines().count();
        let width = s.lines().next().map(|x| x.len()).unwrap();
        let mut domain = Self::new(width, height);
        for (i, line) in s.lines().enumerate() {
            for (j, c) in line.chars().enumerate() {
                if c == '.' {
                    domain.set_value((j, i), true);
                }
            }
        }
        domain
    }

    /// Create a new domain from a file
    fn new_from_file(filename: &str) -> Self {
        let s = read_to_string(filename).expect("Unable to read file");
        Self::new_from_string(s)
    }
}

/// Trait for Printing Domains
pub trait DomainPrint: Domain {
    /// Prints the cells of the domain where . represents a free cell and @ represents a blocked
    /// cell. This will be printed with the given dimensions.
    fn print_cells_with_dims(&self, height: Range<usize>, width: Range<usize>, path: Option<Vec<(usize, usize)>>) -> String {
        let (grid_width, _) = self.shape();
        let mut s = String::new();
        for y in height.clone() {
            for x in width.clone() {
                if self.get_value((x, y)) {
                    s.push('.');
                } else {
                    s.push('@');
                }
            }
            s.push('\n');
        }
        if let Some(path) = path {
            for (x, y) in path {
                if width.contains(&x) && height.contains(&y) {
                    s.replace_range(y * (grid_width + 1) + x..y * (grid_width + 1) + x + 1, "*");
                }
            }
        }
        s
    }

    /// Prints the cells of the domain where . represents a free cell and @ represents a blocked
    /// cell. A path can be printed which is represented as *
    fn print_cells(&self, path: Option<Vec<(usize, usize)>>) -> String {
        let (width, height) = self.shape();
        self.print_cells_with_dims(0..height, 0..width, path)
    }
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
