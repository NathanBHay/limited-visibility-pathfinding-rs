const PADDING: usize = 2;
const BITS_PER_WORD: usize = usize::BITS as usize;
const LOG2_BITS_PER_WORD: usize = usize::BITS.trailing_zeros() as usize;
const INDEX_MASK: usize = BITS_PER_WORD - 1;

pub mod bitpackedgrid2d;
pub mod bitpackedgrid3d;

pub trait BitPackedGrid {
    type Node;
    /// Convert x, y to a map id
    fn get_map_id(&self, n: Self::Node) -> usize;

    /// Convert map id to x, y
    fn invert(&mut self);

    /// Get the number of 1s in the grid
    fn count_ones(&self) -> usize;

}