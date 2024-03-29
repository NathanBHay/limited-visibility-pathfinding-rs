use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use ordered_float::OrderedFloat;

use super::super::bitpackedgrids::bitpackedgrid2d::BitPackedGrid2d;
use super::super::{GridDomain, Grid2d, GridCreate2d, GridPrint2d, GridVisibility2d};
use crate::domains::neighbors;
use crate::matrix;
use crate::util::filter::KalmanNode;
use crate::util::matrix::{convolve2d, matrix_overlay, ConvResolve, Matrix};

/// A grid that allows for the sampling of cells. It uses a series of Kalman filters for updates
/// to the individual cells.
pub struct SampleGrid2d {
    /// The sampling grid which determines the probability of a cell being occupied.
    /// It has a value between 0.0 and 1.0
    pub sample_grid: Matrix<KalmanNode>,

    /// The real values of the grid
    pub ground_truth: BitPackedGrid2d,

    // The width and height of the grid
    // can be removed for reduced space
    pub width: usize,
    pub height: usize,
}

impl GridDomain for SampleGrid2d {
    type Node = (usize, usize);
    
    fn new((width, height): Self::Node) -> Self {
        let node = KalmanNode::default();
        SampleGrid2d {
            sample_grid: matrix![node; height, width],
            ground_truth: BitPackedGrid2d::new((width, height)),
            width,
            height,
        }
    }

    fn set_value(&mut self, (x, y): Self::Node, value: bool) {
        self.sample_grid[x][y].state = if value { 1.0 } else { 0.0 };
        // This is a little counter intuitive but it is assumed that if you use set value you're
        // setting both values, this is to allow new_from_file without having to init_ground_truth
        self.ground_truth.set_value((x, y), value);
    }

    fn get_value(&self, (x, y): Self::Node) -> bool {
        self.bounds_check((x, y)) && self.sample_grid[x][y].state > Self::NEAREST_THRESHOLD
    }

    fn shape(&self) -> Self::Node {
        (self.width, self.height)
    }
    
    fn adjacent(&self, (x, y): (usize, usize), diagonal: bool) -> impl Iterator<Item = (usize, usize)> + '_{
        neighbors((x, y), diagonal)
            .filter(|(x, y)| self.bounds_check((*x, *y)))
    }
}


impl Grid2d for SampleGrid2d {}

impl GridCreate2d for SampleGrid2d {}

impl GridPrint2d for SampleGrid2d {}

impl GridVisibility2d for SampleGrid2d {}

impl SampleGrid2d {
    /// The threshold for a cell to be considered occupied
    const NEAREST_THRESHOLD: f32 = 0.5;

    /// Creates a new sampling grid from a sampling grid and a ground truth grid
    pub fn new_from_grid(grid: Matrix<f32>, ground_truth: BitPackedGrid2d) -> Self {
        let (width, height) = grid.shape();
        let mut sample_grid = matrix![KalmanNode::default(); height, width];
        for y in 0..width {
            // Swapped width and height here to match the matrix indexing
            for x in 0..height {
                sample_grid[x][y].state = grid[x][y];
            }
        }
        SampleGrid2d {
            sample_grid,
            ground_truth,
            width,
            height,
        }
    }

    /// Initializes an area of the bitfield where the kalman state is not 0.0
    pub fn init_gridmap_area<'a>(
        &self,
        gridmap: &'a mut BitPackedGrid2d,
        (x, y): (usize, usize),
        width: usize,
        height: usize,
    ) {
        for x in x..x + width {
            for y in y..y + height {
                gridmap.set_value((x, y), self.sample_grid[x][y].state != 0.0);
            }
        }
    }

    /// Initializes the gridmap within a radius
    pub fn init_gridmap_radius<'a>(
        &mut self,
        gridmap: &'a mut BitPackedGrid2d,
        (x, y): (usize, usize),
        radius: usize,
    ) {
        let (n, width, height) = self.radius_calc((x, y), radius);
        self.init_gridmap_area(gridmap, n, width, height);
    }

    /// Initializes the entire gridmap
    pub fn init_gridmap<'a>(&mut self, gridmap: &'a mut BitPackedGrid2d) {
        self.init_gridmap_area(gridmap, (0, 0), self.width, self.height);
    }

    /// Initializes the ground truth grid from the sampling grid
    pub fn init_ground_truth(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.ground_truth
                    .set_value((x, y), self.sample_grid[x][y].state != 0.0);
            }
        }
    }

    /// Blurs the sampling grid with a gaussian kernal.
    /// Note that this operation sets all covariances to 1.0
    pub fn blur_samplegrid(&mut self, kernel: &Matrix<f32>) {
        self.sample_grid = convolve2d(&self.sample_grid, kernel, ConvResolve::Nearest);
    }

    /// Samples a cell with a given chance
    pub fn sample<'a>(&self, gridmap: &'a mut BitPackedGrid2d, (x, y): (usize, usize)) -> bool {
        self.sample_cached(gridmap, &mut SmallRng::from_entropy(), (x, y))
    }

    /// Samples a cell with a given chance caching from a random number generator
    pub fn sample_cached<'a>(&self, gridmap: &'a mut BitPackedGrid2d, rng: &mut SmallRng, (x, y): (usize, usize)) -> bool {
        let value = self.sample_grid[x][y].state != 0.0
            && rng.gen::<f32>() < self.sample_grid[x][y].state;
        gridmap.set_value((x, y), value);
        value
    }

    /// Samples an area of the grid
    pub fn sample_area<'a>(
        &self,
        gridmap: &'a mut BitPackedGrid2d,
        (x, y): (usize, usize),
        width: usize,
        height: usize,
    ) {
        let mut rng = SmallRng::from_entropy();
        for x in x..x + width {
            for y in y..y + height {
                self.sample_cached(gridmap, &mut rng, (x, y));
            }
        }
    }

    /// Samples a cell around a radius
    pub fn sample_radius<'a>(
        &self,
        gridmap: &'a mut BitPackedGrid2d,
        (x, y): (usize, usize),
        radius: usize,
    ) {
        let (n, width, height) = self.radius_calc((x, y), radius);
        self.sample_area(gridmap, n, width, height);
    }

    /// Samples all cells in the grid
    pub fn sample_all<'a>(&self, gridmap: &'a mut BitPackedGrid2d) {
        self.sample_area(gridmap, (0, 0), self.width, self.height)
    }

    /// Sample the same cells that are found on the sampled before grid. Used in case where one
    /// wants to use the memory of previously sampled grids to sample new grids.
    pub fn sample_based_on_grid<'a>(
        &self,
        gridmap: &'a mut BitPackedGrid2d,
        sampled_before: &BitPackedGrid2d,
    ) {
        let mut rng = SmallRng::from_entropy();
        for x in 0..self.width {
            for y in 0..self.height {
                if sampled_before.get_value((x, y)) {
                    self.sample_cached(gridmap, &mut rng, (x, y));
                }
            }
        }
    }

    /// Updates a cell within the grid, where the `measurement_covariance` is the 
    /// variance of the measurement where 0.0 is a perfect measurement
    pub fn update_node(&mut self, (x, y): (usize, usize), measurement_covariance: f32) {
        let measurement = self.ground_truth.get_value((x, y)) as u8 as f32;
        self.sample_grid[x][y].update(measurement, measurement_covariance);
    }

    /// Creates a kernel for the adjacency of a point. Used to update nodes with
    /// matrix representing covariance.
    fn adjacency_kernel(kernel: &Matrix<f32>) -> Matrix<Option<f32>> {
        let mut kernel = Matrix {
            data: kernel.data.iter().map(|x| Some(*x)).collect(),
            width: kernel.width,
            height: kernel.height,
        };
        let dx = kernel.width / 2;
        let dy = kernel.height / 2;
        kernel[dx][dy] = Some(0.0);
        kernel[dx.saturating_sub(1)][dy] = Some(0.0); // This is to ensure that
        kernel[dx][dy.saturating_sub(1)] = Some(0.0); // the center's measurements
        kernel[dx.saturating_add(1)][dy] = Some(0.0); // are always correct
        kernel[dx][dy.saturating_add(1)] = Some(0.0);
        kernel
    }

    /// Updates nodes based upon a kernal
    fn update_kernel(&mut self, (x, y): (usize, usize), kernel: Matrix<Option<f32>>) {
        for (n, (i, j)) in matrix_overlay(self.shape(), kernel.shape(), (x, y)) {
            if kernel[j][i].is_some() {
                self.update_node(n, kernel[j][i].unwrap());
            }
        }
    }

    /// Updates nodes based on visibile nodes using a kernel
    pub fn raycast_update(&mut self, (x, y): (usize, usize), kernel: &Matrix<f32>) {
        let mut kernel = SampleGrid2d::adjacency_kernel(kernel);
        let visible = self.visibility((x, y), kernel.width / 2);
        for (k, v) in kernel.data.iter_mut().zip(visible.data.iter()) {
            if !*v {
                *k = None;
            }
        }
        self.update_kernel((x, y), kernel);
    }

    /// Get adjacent nodes with their probabilities, log them to make them able to
    /// be used in a search algorithm
    pub fn adjacent_probs(
        &self,
        (x, y): (usize, usize),
        diagonal: bool,
    ) -> Vec<((usize, usize), OrderedFloat<f32>)> {
        self.adjacent((x, y), diagonal)
            .map(|n| {
                (
                    n,
                    OrderedFloat(-1.0 * self.sample_grid[n.0][n.1].state.log2()),
                )
            })
            .collect()
    }

    /// Samples all adjacent cells on sampling grid, accounting for cells that 
    /// have been sampled before
    pub fn sample_adjacenct<'a>(
        &self,
        gridmap: &'a mut BitPackedGrid2d,
        sampled_before: &'a mut BitPackedGrid2d,
        rng: &mut SmallRng,
        (x, y): (usize, usize),
    ) -> Vec<((usize, usize), usize)> {
        neighbors((x, y), false)
            .filter(move |(x, y)| {
                if self.bounds_check((*x, *y)) && !sampled_before.get_value((*x, *y)) {
                    sampled_before.set_value((*x, *y), true);
                    self.sample_cached(gridmap, rng, (*x, *y))
                } else {
                    gridmap.get_value((*x, *y))
                }
            })
            .map(|n| (n, 1))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        domains::{
            bitpackedgrids::{bitpackedgrid2d::BitPackedGrid2d, BitPackedGrid}, GridDomain, GridCreate2d, GridPrint2d, GridVisibility2d,
        },
        matrix,
        util::matrix::{gaussian_kernal, Matrix},
    };

    use super::SampleGrid2d;

    #[test]
    fn test_samplegrid_new() {
        let grid = SampleGrid2d::new((128, 128));
        assert_eq!(grid.height, 128);
        assert_eq!(grid.width, 128);
        assert_eq!(grid.sample_grid.height, 128);
        assert_eq!(grid.sample_grid.width, 128);
    }

    #[test]
    fn test_samplegrid_new_from_grid() {
        let grid =
            SampleGrid2d::new_from_grid(matrix![[0.0, 1.0], [1.0, 0.0]], BitPackedGrid2d::new((2, 2)));
        assert_eq!(grid.sample_grid[0][0].state, 0.0);
        assert_eq!(grid.sample_grid[1][0].state, 1.0);
    }

    #[test]
    fn test_samplegrid_new_from_string() {
        let map_str = ".....\n@@.@.\n.@.@.\n.@.@.\n.....\n....@\n";
        let grid = SampleGrid2d::new_from_string(map_str.to_string());
        assert_eq!(grid.print_cells(None), map_str);
        assert_eq!(grid.ground_truth.print_cells(None), map_str);
        assert_eq!(grid.sample_grid[0][0].state, 1.0);
        assert_eq!(grid.sample_grid[1][0].state, 1.0);
        assert_eq!(grid.sample_grid[0][1].state, 0.0);
    }

    #[test]
    fn test_gridmap_init() {
        let mut grid = SampleGrid2d::new_from_string("@....\n.....\n.....\n.....\n".to_string());
        let mut gridmap = BitPackedGrid2d::new((grid.width, grid.height));
        grid.init_gridmap_radius(&mut gridmap, (0, 0), 2);
        assert_eq!(
            grid.ground_truth.print_cells(None),
            "@....\n.....\n.....\n.....\n"
        );
        assert_eq!(gridmap.print_cells(None), "@..@@\n...@@\n...@@\n@@@@@\n");
    }

    #[test]
    fn test_blur() {
        let mut grid =
            SampleGrid2d::new_from_string("@....\n@@...\n@@@..\n@@@..\n@@...\n".to_string());
        grid.blur_samplegrid(&gaussian_kernal(3, 1.0));
        let grid_sample: Vec<f32> = grid.sample_grid.data.iter().map(|x| x.state).collect();
        assert_eq!(
            grid_sample,
            vec![
                0.19895503, 0.07511361, 0.0, 0.0, 0.0, 0.60209, 0.32279643, 0.07511361, 0.07511361,
                0.19895503, 0.9248864, 0.6772036, 0.39791006, 0.39791006, 0.60209, 1.0, 0.9248864,
                0.801045, 0.801045, 0.9248864, 1.0, 1.0, 1.0, 1.0, 1.,
            ]
        );
    }

    #[test]
    fn test_sample() {
        let mut grid = SampleGrid2d::new((8, 8));
        grid.sample_grid[0][0].state = 1.0;
        grid.sample_grid[2][4].state = 1.0;
        grid.sample_grid[1][6].state = 1.0;
        let mut gridmap = BitPackedGrid2d::new((grid.width, grid.height));
        grid.sample(&mut gridmap, (0, 0));
        assert_eq!(gridmap.get_value((0, 0)), true);
        grid.sample_all(&mut gridmap);
        assert_eq!(gridmap.count_ones(), 3);
    }

    #[test]
    fn test_adjacency_kernel() {
        let kernel = SampleGrid2d::adjacency_kernel(&matrix![1.0; 3]);
        assert_eq!(
            kernel,
            matrix![
                [Some(1.0), Some(0.0), Some(1.0)],
                [Some(0.0), Some(0.0), Some(0.0)],
                [Some(1.0), Some(0.0), Some(1.0)],
            ]
        )
    }

    #[test]
    fn test_grid_visibility() {
        let grid = SampleGrid2d::new_from_string("\n@@@.\n@...\n@.@.\n".to_string());
        let visiblility = grid.visibility((1, 1), 2);
        assert_eq!(
            visiblility,
            matrix![
                [false, false, false, false, false],
                [false, true, true, true, true],
                [false, true, true, true, true],
                [false, true, true, true, true],
                [false, false, false, false, false],
            ]
        );
    }

    #[test]
    fn test_raycast_update() {
        let mut grid = SampleGrid2d::new_from_string(".@..\n.@.@\n.@.@\n".to_string());
        for n in grid.sample_grid.data.iter_mut() {
            if n.state == 1.0 {
                n.state = 0.6;
            }
        }
        grid.raycast_update((0, 0), &matrix![0.0; 5]);
        assert!(grid.sample_grid[0][0].state == 1.0);
        assert!(grid.sample_grid[0][1].state == 1.0);
        assert!(grid.sample_grid[1][0].state == 0.0);
        assert!(grid.sample_grid[2][0].state == 0.6);
    }
}
