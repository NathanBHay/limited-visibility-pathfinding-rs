//! # Hashed Grid Maps
//! A grid implementation which uses a hashset to represent obstacles
//! This is the simplest grid implementation and is fairly slow due
//! to the possibility of hashing collisions.

use std::collections::HashSet;

use super::{neighbors, Domain, DomainCreate, DomainPrint};

/// A grid map whoch uses a hashset of obstacles to represent obstacles
/// This is the simplest grid map implementation
/// ## Fields
/// * `width` - The width of the grid map
/// * `height` - The height of the grid map
/// * `diagonal` - Whether or not diagonal movement is allowed
/// * `valid_cells` - A hashset of cells that are traversable
pub struct HashedGrid {
    pub width: usize,
    pub height: usize,
    pub valid_cells: HashSet<usize>,
}

impl Domain for HashedGrid {
    /// Creates a new grid map with a given width and height
    fn new(width: usize, height: usize) -> HashedGrid {
        HashedGrid {
            width,
            height,
            valid_cells: HashSet::new(),
        }
    }

    /// Sets the value of a cell in a map. True if the cell is traversable and 
    /// false if it is an obstacle.
    fn set_value(&mut self, (x, y): (usize, usize), value: bool) {
        if value {
            self.valid_cells.insert(x + y * self.width);
        } else {
            self.valid_cells.remove(&(x + y * self.width));
        }
    }

    fn get_value(&self, (x, y): (usize, usize)) -> bool {
        self.valid_cells.contains(&(x + y * self.width))
    }

    fn shape(&self) -> (usize, usize) {
        (self.width, self.height)
    }
}

impl DomainCreate for HashedGrid {}

impl DomainPrint for HashedGrid {}

impl HashedGrid {
    /// Inverts the grid map
    pub fn invert(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.set_value((x, y), !self.get_value((x, y)))
            }
        }
    }

    /// Checks if a given coordinate is valid and not an obstacle
    pub fn valid_map_value(&self, (x, y): (usize, usize)) -> bool {
        self.bounds_check((x, y)) && self.get_value((x, y))
    }

    /// Gets the neighbors of a given coordinate
    pub fn adjacent(
        &self,
        x: usize,
        y: usize,
        diagonal: bool,
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        neighbors((x, y), diagonal).filter(move |(x, y)| self.valid_map_value((*x, *y)))
    }
}

#[cfg(test)]
mod tests {

    use crate::domains::{Domain, DomainCreate, DomainPrint};

    use super::HashedGrid;

    #[test]
    fn test_grid_add_obstacle() {
        let mut grid = HashedGrid::new(5, 5);
        grid.set_value((0, 0), false);
        grid.set_value((2, 3), false);
        assert_eq!(grid.get_value((0, 0)), false);
        assert_eq!(grid.get_value((2, 3)), false);
    }

    #[test]
    fn test_grid_new_from_string() {
        let map_str = ".....\n.@.@.\n.@.@.\n.@.@.\n.....\n....@\n";
        let grid = HashedGrid::new_from_string(map_str.to_string());
        assert_eq!(grid.width, 5);
        assert_eq!(grid.height, 6);
        assert_eq!(grid.get_value((0, 0)), true);
        assert_eq!(grid.get_value((1, 1)), false);
        assert_eq!(grid.get_value((3, 1)), false);
        assert_eq!(grid.get_value((3, 3)), false);
        assert_eq!(grid.get_value((4, 4)), true);
        assert_eq!(grid.print_cells(None), map_str);
    }

    #[test]
    fn test_grid_adjacent() {
        let grid = HashedGrid::new_from_string(String::from("...\n@.@\n@.@"));
        let neighbors = grid.adjacent(2, 1, true).collect::<Vec<_>>();
        assert_eq!(neighbors, vec![(1, 1), (2, 0), (1, 2), (1, 0)]);
    }
}
