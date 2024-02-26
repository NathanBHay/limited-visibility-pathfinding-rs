//! # Bit-Packed Grid Maps
//! A grid map representation that uses bitpacking to store the map.
//! This is a very fast grid map implementation, as it uses bit manipulations
//! to change the map while also being the smallest as it compresses the map
//! into a bitpacked array. This array is stored as an array of numbers, where
//! each binary digit in the number represents a cell in the map. These cells
//! are additionally padded with 2 rows and columns to avoid travelling out of
//! bounds. This allows is instead of using a brach which slows down the code.
//! There are two dimensions of bitpacked grid maps, 2D and 3D. The 3d version
//! lacks adequent testing as of the project's end.
//! 
//! These implementations are based upon Warthog's implementation of bitpacked grid
//! maps, which can be found: https://bitbucket.org/dharabor/pathfinding/

// Constants used in both 2D and 3D bitpacked grid maps
const PADDING: usize = 2;
const BITS_PER_WORD: usize = usize::BITS as usize;
const LOG2_BITS_PER_WORD: usize = usize::BITS.trailing_zeros() as usize;
const INDEX_MASK: usize = BITS_PER_WORD - 1;

pub mod bitpackedgrid2d;
pub mod bitpackedgrid3d;

/// Trait that represents a bit-packed grid map of any dimension
pub trait BitPackedGrid {
    type Node;
    /// Convert x, y to a map id
    fn get_map_id(&self, n: Self::Node) -> usize;

    /// Convert map id to x, y
    fn invert(&mut self);

    /// Get the number of 1s in the grid
    fn count_ones(&self) -> usize;

}