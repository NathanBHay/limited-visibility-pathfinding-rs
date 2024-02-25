//! # Bit-Packed Grid Maps
//! A grid map representation that uses bitpacking to store the map.
//! This is the fastest grid map implementation, as it uses bit manipulations
//! to change the map while also being the smallest as it compresses the map
//! into a bitpacked array. This array is stored as an array of numbers, where
//! each binary digit in the number represents a cell in the map. These cells
//! are additionally padded with 2 rows and columns to avoid travelling out of
//! bounds.
//!
//! This implementation is based upon Warthog's implementation of bitpacked grid
//! maps, which can be found: https://bitbucket.org/dharabor/pathfinding/

use std::vec;

use crate::util::matrix::matrix_overlay;

use super::{super::{neighbors, samplegrids::samplegrid2d::SampleGrid2d, Domain, Grid2d, GridCreate2d, GridPrint2d, GridVisibility2d}, BitPackedGrid};

/// A grid of bits packed into usize-bit words
#[derive(Clone, Debug)]
pub struct BitPackedGrid2d {
    pub height: usize,
    pub width: usize,
    /// The height of the map including padding
    map_height: usize,
    /// The width of the map including padding
    map_width: usize,
    map_cells: Box<[usize]>,
}

impl Domain for BitPackedGrid2d {
    type Node = (usize, usize);

    /// Creates a new grid map with a given width and height
    fn new((width, height): Self::Node) -> BitPackedGrid2d {
        let map_width_in_words = (width >> super::LOG2_BITS_PER_WORD) + 1;
        let map_width = map_width_in_words << super::LOG2_BITS_PER_WORD;
        let map_height = height + 2 * super::PADDING;
        let map_size = (map_width * map_height) >> super::LOG2_BITS_PER_WORD;
        let map_cells: Box<[usize]> = vec![0; map_size as usize].into_boxed_slice();
        BitPackedGrid2d {
            height,
            width,
            map_height,
            map_width,
            map_cells,
        }
    }

    /// Set the value if a but at a given x, y coordinate to be true or false
    fn set_value(&mut self, (x, y): Self::Node, value: bool) {
        let map_id = self.get_map_id((x, y));
        let word_index = map_id >> super::LOG2_BITS_PER_WORD;
        let mask = 1 << (map_id & super::INDEX_MASK);
        if value {
            self.map_cells[word_index as usize] |= mask;
        } else {
            self.map_cells[word_index as usize] &= !mask;
        }
    }

    /// Get the value of a bit at a given x, y coordinate
    fn get_value(&self, (x, y): Self::Node) -> bool {
        let map_id = self.get_map_id((x, y));
        let word_index = map_id >> super::LOG2_BITS_PER_WORD;
        let mask = 1 << (map_id & super::INDEX_MASK);
        (self.map_cells[word_index as usize] & mask) != 0
    }

    fn shape(&self) -> Self::Node {
        (self.width, self.height)
    }
    
    fn adjacent(
        &self,
        (x, y): (usize, usize),
        diagonal: bool,
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        neighbors((x, y), diagonal).filter(move |(x, y)| self.get_value((*x, *y)))
    }
}

impl Grid2d for BitPackedGrid2d {}

impl GridCreate2d for BitPackedGrid2d {}

impl GridPrint2d for BitPackedGrid2d {}

impl GridVisibility2d for BitPackedGrid2d {}

impl BitPackedGrid for BitPackedGrid2d {
    type Node = (usize, usize);
    
    fn get_map_id(&self, (x, y): Self::Node) -> usize {
        self.map_width * y.wrapping_add(super::PADDING)
            + x.wrapping_add(super::PADDING)
    }

    fn count_ones(&self) -> usize {
        self.map_cells
            .iter()
            .map(|word| word.count_ones() as usize)
            .sum()
    }

    fn invert(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.set_value((x, y), !self.get_value((x, y)))
            }
        }
    }
}

impl BitPackedGrid2d {

    /// Convert map id to x, y
    fn map_id_to_xy(&self, map_id: usize) -> (usize, usize) {
        let y = map_id / self.height;
        let x = map_id - y * self.width;
        (x, y)
    }

    /// Set Radius based upon a raycast
    pub fn raycast_set_radius(&mut self, gridmap: &SampleGrid2d, (x, y): (usize, usize), radius: usize, value: bool) {
        let kernel = gridmap.visibility((x, y), radius);
        for (n, (i, j)) in matrix_overlay(self.shape(), kernel.shape(), (x, y)) {
            if kernel[j][i] {
                self.set_value(n, value);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::domains::{bitpackedgrids::BitPackedGrid, Domain, GridCreate2d, GridPrint2d};

    use super::BitPackedGrid2d;

    #[test]
    fn test_bitpackedgrid_new() {
        let grid = BitPackedGrid2d::new((128, 128));
        assert_eq!(grid.height, 128);
        assert_eq!(grid.width, 128);
        assert_eq!(grid.map_height, 132);
        assert_eq!(grid.map_width, 192);
        assert_eq!(grid.map_cells.len(), 396);
    }

    #[test]
    fn test_bitpackedgrid_set_value() {
        let mut grid = BitPackedGrid2d::new((16, 16));
        grid.set_value((0, 0), true);
        assert_eq!(grid.get_value((0, 0)), true);
        grid.set_value((0, 0), false);
        assert_eq!(grid.get_value((0, 0)), false);
        grid.set_value((15, 15), true);
        assert_eq!(grid.get_value((15, 15)), true);
        grid.set_value((15, 5), true);
        assert_eq!(grid.get_value((15, 5)), true);
    }

    #[test]
    fn test_bitpackedgrid_create() {
        let map_str = ".....\n.@.@.\n.@.@.\n.@.@.\n.....\n....@\n";
        let grid = BitPackedGrid2d::new_from_string(map_str.to_string());
        assert_eq!(grid.print_cells(None), map_str);
    }

    #[test]
    fn test_bitpackedgrid_get_neighbours() {
        let grid = BitPackedGrid2d::new_from_string(
            ".....\n.@.@.\n.@.@.\n.@.@.\n.....\n....@\n".to_string(),
        );
        assert_eq!(
            grid.adjacent((0, 0), false).collect::<Vec<_>>(),
            vec![(1, 0), (0, 1)]
        );
    }

    #[test]
    fn test_bitpackedgrid_count_one() {
        let grid = BitPackedGrid2d::new_from_string(
            ".....\n.@.@.\n.@.@.\n.@.@.\n....@\n".to_string(),
        );
        assert_eq!(grid.count_ones(), 18);
    }
}
