use crate::domains::{neighbors3d, Domain};

use super::BitPackedGrid;

pub struct BitPackedGrid3d {
    pub height: usize,
    pub width: usize,
    pub depth: usize,
    map_height: usize,
    map_width: usize,
    map_area: usize,
    map_depth: usize,
    map_cells: Box<[usize]>,
}

impl Domain for BitPackedGrid3d {
    type Node = (usize, usize, usize);

    fn new((width, height, depth): Self::Node) -> Self {
        let map_width_in_words = (width >> super::LOG2_BITS_PER_WORD) + 1;
        let map_width = map_width_in_words << super::LOG2_BITS_PER_WORD;
        let map_height = height + 2 * BitPackedGrid3d::YPADDING;
        let map_depth = depth + 2 * super::PADDING;
        let map_area = map_width * map_height;
        let map_size = (map_area * map_depth) >> super::LOG2_BITS_PER_WORD;
        let map_cells: Box<[usize]> = vec![0; map_size as usize].into_boxed_slice();
        BitPackedGrid3d {
            height,
            width,
            depth,
            map_height,
            map_width,
            map_depth,
            map_area,
            map_cells,
        }
    }

    fn set_value(&mut self, (x, y, z): Self::Node, value: bool) {
        let map_id = self.get_map_id((x, y, z));
        let word_index = map_id >> super::LOG2_BITS_PER_WORD;
        let mask = 1 << (map_id & super::INDEX_MASK);
        if value {
            self.map_cells[word_index as usize] |= mask;
        } else {
            self.map_cells[word_index as usize] &= !mask;
        }
    }

    fn get_value(&self, n: Self::Node) -> bool {
        let map_id = self.get_map_id(n);
        let word_index = map_id >> super::LOG2_BITS_PER_WORD;
        let mask = 1 << (map_id & super::INDEX_MASK);
        (self.map_cells[word_index as usize] & mask) != 0
    }

    fn shape(&self) -> Self::Node {
        (self.width, self.height, self.depth)
    }

    fn adjacent(&self, (x, y, z): Self::Node, diagonal: bool) -> impl Iterator<Item = Self::Node> {
        neighbors3d((x, y, z), diagonal).filter(move |(x, y, z)| self.get_value((*x, *y, *z)))
    }
}

impl BitPackedGrid for BitPackedGrid3d {
    type Node = (usize, usize, usize);

    /// Get the map id of a cell
    fn get_map_id(&self, (x, y, z): (usize, usize, usize)) -> usize {
        self.map_area * z.wrapping_add(super::PADDING)
         + self.map_width * y.wrapping_add(BitPackedGrid3d::YPADDING)
            + x.wrapping_add(super::PADDING)
    }

    fn invert(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                for z in 0..self.height {
                    self.set_value((x, y, z), !self.get_value((x, y, z)))
                }
            }
        }
    }

    fn count_ones(&self) -> usize {
        self.map_cells
            .iter()
            .map(|word| word.count_ones() as usize)
            .sum()
    
    }
}

impl BitPackedGrid3d {
    const YPADDING: usize = super::PADDING - 1;
}

#[cfg(test)]
mod tests {
    use crate::domains::{bitpackedgrids::BitPackedGrid, Domain};

    use super::BitPackedGrid3d;

    #[test]
    fn test_bitpackedgrid_new() {
        let grid = BitPackedGrid3d::new((32, 32, 32));
        assert_eq!(grid.depth, 32);
        assert_eq!(grid.height, 32);
        assert_eq!(grid.width, 32);
        assert_eq!(grid.map_depth, 36);
        assert_eq!(grid.map_height, 34);
        assert_eq!(grid.map_width, 64);
        assert_eq!(grid.map_area, 2176);
    }

    #[test]
    fn test_bitpackedgrid_set_value() {
        let mut grid = BitPackedGrid3d::new((16, 16, 8));
        grid.set_value((0, 0, 0), true);
        assert_eq!(grid.get_value((0, 0, 0)), true);
        grid.set_value((0, 0, 0), false);
        assert_eq!(grid.get_value((0, 0, 0)), false);
        grid.set_value((15, 15, 7), true);
        assert_eq!(grid.get_value((15, 15, 7)), true);
        grid.set_value((15, 5, 5), true);
        assert_eq!(grid.get_value((15, 5, 5)), true);
    }

    #[test]
    fn test_bitpackedgrid_value() {
        let mut grid = BitPackedGrid3d::new((8, 8, 8));
        grid.invert();
        for i in grid.map_cells.iter() {
            println!("{:064b}", *i)
        }
    }

    #[test]
    fn test_bitpacked_grid_get_neighbours() {
        let mut grid = BitPackedGrid3d::new((8, 8, 8));
        grid.set_value((0, 0, 0), true);
        grid.set_value((1, 0, 0), true);
        grid.set_value((0, 1, 0), true);
        grid.set_value((0, 0, 1), true);
        let neighbours: Vec<_> = grid.adjacent((0, 0, 0), false).collect();
        assert_eq!(neighbours.len(), 3);
        assert_eq!(neighbours, vec![(1, 0, 0), (0, 1, 0), (0, 0, 1)]);
    }
}