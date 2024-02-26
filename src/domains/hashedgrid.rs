//! # Hashed Grid Maps
//! A grid implementation which uses a hashset to represent valid cells. This is the 
//! simplest grid implementation and is fairly slow due to the possibility of
//! hashing collisions. Still within the repo for benchmarking purposes.

use ahash::AHashSet;

use super::{neighbors, GridDomain, Grid2d, GridCreate2d, GridPrint2d};

/// A grid map which uses a hashset to represent valid cells, this is a 
/// 2-dimensional grid map.
pub struct HashedGrid {
    pub width: usize,
    pub height: usize,
    pub valid_cells: AHashSet<usize>,
}

impl GridDomain for HashedGrid {
    type Node = (usize, usize);

    fn new((width, height): Self::Node) -> HashedGrid {
        HashedGrid {
            width,
            height,
            valid_cells: AHashSet::new(),
        }
    }

    fn set_value(&mut self, (x, y): Self::Node, value: bool) {
        if value {
            self.valid_cells.insert(x + y * self.width);
        } else {
            self.valid_cells.remove(&(x + y * self.width));
        }
    }

    fn get_value(&self, (x, y): Self::Node) -> bool {
        self.valid_cells.contains(&(x + y * self.width))
    }

    fn shape(&self) -> Self::Node {
        (self.width, self.height)
    }

    fn adjacent(
        &self,
        (x, y): Self::Node,
        diagonal: bool,
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        neighbors((x, y), diagonal).filter(move |(x, y)| self.get_value((*x, *y)))
    }
}

impl Grid2d for HashedGrid {}

impl GridCreate2d for HashedGrid {}

impl GridPrint2d for HashedGrid {}

impl HashedGrid {
    /// Inverts the grid map
    pub fn invert(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.set_value((x, y), !self.get_value((x, y)))
            }
        }
    }
}

#[cfg(test)]
mod tests {

    use crate::domains::{GridDomain, GridCreate2d, GridPrint2d};

    use super::HashedGrid;

    #[test]
    fn test_grid_add_obstacle() {
        let mut grid = HashedGrid::new((5, 5));
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
        let neighbors = grid.adjacent((2, 1), true).collect::<Vec<_>>();
        assert_eq!(neighbors, vec![(1, 1), (2, 0), (1, 2), (1, 0)]);
    }
}
