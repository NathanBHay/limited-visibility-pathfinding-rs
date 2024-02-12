//! # Domains
//! Domains that can be used with search algorithms. These domains include:
//! * BitPackedGrid, a grid map that uses 1 bit per cell
//! * HashedGrid, a grid map that uses a hash map to store the map
//! * AdjacencyList, a graph representation of a map
//! * SampleGrid, a grid map that uses a hash map to store the map and has a chance of being occupied

#![allow(dead_code)]

use std::{fs::read_to_string, ops::Range};

use rand::{rngs::ThreadRng, seq::SliceRandom};

use crate::{fov::fieldofvision::raycast_matrix, util::matrix::Matrix};

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

    /// Check if a given coordinate is valid
    fn bounds_check(&self, n: (usize, usize)) -> bool {
        let (width, height) = self.shape();
        n.0 < width && n.1 < height
    }
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

/// Trait for calculating the radius of a given point
pub trait RadiusCalc: Domain {
    /// Calculate the radius of a given point
    fn radius_calc(&self, n: (usize, usize), radius: usize) -> ((usize, usize), usize, usize) {
        let radius = if radius % 2 == 0 { radius + 1 } else { radius};
        let x_min = n.0.saturating_sub(radius);
        let y_min = n.1.saturating_sub(radius);
        let (width, height) = self.shape();
        let x_max = (n.0 + radius).min(width);
        let y_max = (n.1 + radius).min(height);
        ((x_min, y_min), x_max - x_min, y_max - y_min)
    }
}

/// Trait for computing the visibility within a domain
pub trait DomainVisibility: Domain {
    /// Get the visibility of a given point
    fn visibility(&self, n: (usize, usize), radius: usize) -> Matrix<bool> {
        raycast_matrix(
            (n.0 as isize, n.1 as isize),
            radius,
            |(x, y)| self.get_value((x as usize, y as usize)),
            |(x, y)| self.bounds_check((x as usize, y as usize)),
        )
    }
}

/// Helper function to get a iterator of the neighbors of a cell
pub fn neighbors((x, y): (usize, usize), diagonal: bool) -> impl Iterator<Item = (usize, usize)> {
    neighbors_cached((x, y), diagonal, None)
}

/// Helper function to get a iterator of the neighbors of a cell caching the random number gen
pub fn neighbors_cached((x, y): (usize, usize), diagonal: bool, rng: Option<&mut ThreadRng>) -> impl Iterator<Item = (usize, usize)> {
    let mut neighbors = vec![
        (x.wrapping_add(1), y), // Theses are wrapping as to avoid branching
        (x, y.wrapping_add(1)), // on BitPacked Grids
        (x.wrapping_sub(1), y),
        (x, y.wrapping_sub(1)),
    ];

    if diagonal {
        neighbors.extend(vec![
            (x.wrapping_add(1), y.wrapping_add(1)),
            (x.wrapping_sub(1), y.wrapping_add(1)),
            (x.wrapping_add(1), y.wrapping_sub(1)),
            (x.wrapping_sub(1), y.wrapping_sub(1)),
        ]);
    }

    if let Some(rng) = rng {
        neighbors.shuffle(rng);
    }
    neighbors.into_iter()
}
