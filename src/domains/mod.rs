//! # Domains
//! Domains that can be used with search algorithms. These domains include:
//! * BitPackedGrid, a grid map that uses 1 bit per cell
//! * HashedGrid, a grid map that uses a hash map to store the map
//! * SampleGrid, a grid of probability filters that can be sampled

#![allow(dead_code)]

use std::{fs::read_to_string, ops::Range};

use crate::{fov::fieldofvision::raycast_matrix, util::matrix::Matrix};

pub mod bitpackedgrids;
pub mod hashedgrid;
pub mod samplegrids;

/// Trait that represents a grid domain that can be used with search algorithms.
/// This trait allows trait allows subtraits to do repeated operations.
pub trait GridDomain {
    type Node;
    /// Creates a new map with a given width and height
    fn new(dims: Self::Node) -> Self;

    /// Sets the value of a cell in a map. True if the cell is traversable and 
    /// false if it is an obstacle.
    fn set_value(&mut self, n: Self::Node, value: bool);

    /// Gets the value of a cell in a map. True if the cell is traversable and
    /// false if it is an obstacle.
    fn get_value(&self, n: Self::Node) -> bool;

    /// Get shape of the data listed in x, y, z, ... format
    fn shape(&self) -> Self::Node;

    /// Get the neighbors of a given cell
    fn adjacent(&self, n: Self::Node, diagonal: bool) -> impl Iterator<Item = Self::Node>;
}

/// Trait for 2d grid domains
pub trait Grid2d: GridDomain<Node = (usize, usize)> {
    /// Check if within 2d bounds
    fn bounds_check(&self, (x, y): Self::Node) -> bool {
        let (width, height) = self.shape();
        x < width && y < height
    }

    /// Calculate the radius of a given point
    fn radius_calc(&self, n: Self::Node, radius: usize) -> (Self::Node, usize, usize) {
        let radius = if radius % 2 == 0 { radius + 1 } else { radius};
        let x_min = n.0.saturating_sub(radius);
        let y_min = n.1.saturating_sub(radius);
        let (width, height) = self.shape();
        let x_max = (n.0 + radius).min(width);
        let y_max = (n.1 + radius).min(height);
        ((x_min, y_min), x_max - x_min, y_max - y_min)
    }
}

/// Trait for 3d grid domains
pub trait Grid3d: GridDomain<Node = (usize, usize, usize)> {
    /// Check if within 3d bounds
    fn bounds_check(&self, (x, y, z): (usize, usize, usize)) -> bool {
        let (width, height, depth) = self.shape();
        x < width && y < height && z < depth
    }
}
 
/// Trait used to create domains from files and strings.
pub trait GridCreate2d: Grid2d + Sized {
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
        let mut domain = Self::new((width, height));
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
pub trait GridPrint2d: Grid2d {
    /// Prints the cells of the domain where . represents a free cell and @ represents a blocked
    /// cell. This will be printed with the given dimensions.
    fn print_cells_with_dims(&self, height: Range<usize>, width: Range<usize>, path: Option<Vec<Self::Node>>) -> String {
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
    fn print_cells(&self, path: Option<Vec<Self::Node>>) -> String {
        let (width, height) = self.shape();
        self.print_cells_with_dims(0..height, 0..width, path)
    }
}

/// Trait for computing the visibility within a domain
pub trait GridVisibility2d: Grid2d {
    /// Get the visibility of a given point
    fn visibility(&self, n: Self::Node, radius: usize) -> Matrix<bool> {
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
    neighbors.into_iter()
}

/// Helper for 3-dimensional neighbours
pub fn neighbors3d((x, y, z): (usize, usize, usize), diagonal: bool) -> impl Iterator<Item = (usize, usize, usize)> {
    let mut neighbors = vec![
        (x.wrapping_add(1), y, z),
        (x, y.wrapping_add(1), z),
        (x.wrapping_sub(1), y, z),
        (x, y.wrapping_sub(1), z),
        (x, y, z.wrapping_add(1)),
        (x, y, z.wrapping_sub(1)),
    ];
    if diagonal {
        neighbors.extend(vec![
            (x.wrapping_add(1), y.wrapping_add(1), z),
            (x.wrapping_sub(1), y.wrapping_add(1), z),
            (x.wrapping_add(1), y.wrapping_sub(1), z),
            (x.wrapping_sub(1), y.wrapping_sub(1), z),
            (x.wrapping_add(1), y, z.wrapping_add(1)),
            (x.wrapping_sub(1), y, z.wrapping_add(1)),
            (x.wrapping_add(1), y, z.wrapping_sub(1)),
            (x.wrapping_sub(1), y, z.wrapping_sub(1)),
            (x, y.wrapping_add(1), z.wrapping_add(1)),
            (x, y.wrapping_sub(1), z.wrapping_add(1)),
            (x, y.wrapping_add(1), z.wrapping_sub(1)),
            (x, y.wrapping_sub(1), z.wrapping_sub(1)),
        ]);
    }
    neighbors.into_iter()
}