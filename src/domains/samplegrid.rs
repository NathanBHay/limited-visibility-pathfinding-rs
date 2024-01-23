use super::bitpackedgrid::BitPackedGrid;
use super::{create_map_from_string, plot_cells, print_cells};
use crate::fov::fieldofvision::raycast_matrix;
use crate::matrix;
use crate::util::filter::KalmanNode;
use crate::util::matrix::{convolve2d, matrix_overlay, ConvResolve, Matrix};

pub struct SampleGrid {
    /// The sampling grid which determines the probability of a cell being occupied.
    /// It has a value between 0.0 and 1.0
    pub sample_grid: Matrix<KalmanNode>,

    /// The bitpacked grid which represents sampled cells
    /// TODO: This cam be simplified to a smaller sub grid
    pub gridmap: BitPackedGrid,

    /// Places where the grid has been sampled before
    pub sampled_before: BitPackedGrid,

    /// The real values of the grid
    pub ground_truth: BitPackedGrid,

    // The width and height of the grid
    // can be removed for reduced space
    pub width: usize,
    pub height: usize,
}

impl SampleGrid {
    const NEAREST_THRESHOLD: f32 = 0.5;

    /// Creates a new sampling grid from a sampling grid and a ground truth grid
    pub fn new_from_grid(grid: Matrix<f32>, ground_truth: BitPackedGrid) -> Self {
        let width = grid.height;
        let height = grid.width;
        let mut sample_grid = matrix![KalmanNode::default(); height, width];
        for y in 0..width {
            // Swapped width and height here
            for x in 0..height {
                // to match the matrix indexing
                sample_grid[x][y].state = grid[x][y];
            }
        }
        let mut grid = SampleGrid {
            sample_grid,
            gridmap: BitPackedGrid::new(width, height),
            ground_truth,
            width,
            height,
            sampled_before: BitPackedGrid::new(width, height),
        };
        grid.init_gridmap();
        grid
    }

    /// Creates a new sampling grid with a given size
    pub fn new_with_size(width: usize, height: usize) -> Self {
        let node = KalmanNode::default();
        SampleGrid {
            sample_grid: matrix![node; height, width],
            gridmap: BitPackedGrid::new(width, height),
            ground_truth: BitPackedGrid::new(width, height),
            width,
            height,
            sampled_before: BitPackedGrid::new(width, height),
        }
    }

    /// Creates a sampling grid from a string
    pub fn new_from_string(map: String) -> Self {
        let mut grid = create_map_from_string(map, SampleGrid::new_with_size, |grid, x, y| {
            grid.sample_grid[x][y].state = 1.0;
        });
        grid.init_ground_truth(); // These aren't good practice as they are an
        grid.init_gridmap(); // weird side effect
        grid
    }

    /// Creates a sampling grid from a file
    pub fn new_from_file(filename: &str) -> Self {
        let s = std::fs::read_to_string(filename).expect("Unable to read file");
        SampleGrid::new_from_string(s)
    }

    /// Initializes an area of the bitfield from the sampling grid values, where
    /// 0.0 indicates a guaranteed obstacles and (0,1) indicates a probability
    pub fn init_gridmap_area(&mut self, (x, y): (usize, usize), width: usize, height: usize) {
        for x in x..x + width {
            for y in y..y + height {
                self.gridmap
                    .set_bit_value((x, y), self.sample_grid[x][y].state != 0.0);
            }
        }
    }

    /// Initializes the gridmap from the sampling grid using the `NEAREST_THRESHOLD``
    pub fn init_gridmap_nearest(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.gridmap.set_bit_value(
                    (x, y),
                    self.sample_grid[x][y].state > Self::NEAREST_THRESHOLD,
                );
            }
        }
    }

    /// Initializes the gridmap from the sampling grid
    pub fn init_gridmap_radius(&mut self, (x, y): (usize, usize), radius: usize) {
        let radius = radius + 1;
        let x_min = x.saturating_sub(radius);
        let y_min = y.saturating_sub(radius);
        let x_max = (x + radius).min(self.width);
        let y_max = (y + radius).min(self.height);
        self.init_gridmap_area((x_min, y_min), x_max - x_min, y_max - y_min);
    }

    /// Initializes the gridmap from the sampling grid
    pub fn init_gridmap(&mut self) {
        self.init_gridmap_area((0, 0), self.width, self.height);
    }

    /// Initialize the sampled before grid
    pub fn init_sampled_before(&mut self) {
        // TODO: Calculate a mask that does this slightly faster
        self.sampled_before = BitPackedGrid::new(self.width, self.height)
    }

    /// Initializes the ground truth grid from the sampling grid
    pub fn init_ground_truth(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                self.ground_truth
                    .set_bit_value((x, y), self.sample_grid[x][y].state != 0.0);
            }
        }
    }

    /// Blurs the sampling grid with a gaussian kernal.
    /// Note that this operation sets all covariances to 1.0
    pub fn blur_samplegrid(&mut self, kernel: &Matrix<f32>) {
        self.sample_grid = convolve2d(&self.sample_grid, kernel, ConvResolve::Nearest);
    }

    /// Samples a cell with a given chance
    pub fn sample(&mut self, (x, y): (usize, usize)) -> bool {
        let value = self.sample_grid[x][y].state != 0.0
            && rand::random::<f32>() < self.sample_grid[x][y].state;
        self.gridmap.set_bit_value((x, y), value);
        value
    }

    /// Samples an area of the grid
    pub fn sample_area(&mut self, (x, y): (usize, usize), width: usize, height: usize) {
        for x in x..x + width {
            for y in y..y + height {
                self.sample((x, y));
            }
        }
    }

    // Calculates the radius of the sampling area
    pub fn radius_calc(
        &self,
        (x, y): (usize, usize),
        radius: usize,
    ) -> ((usize, usize), usize, usize) {
        let radius = radius + 1;
        let x_min = x.saturating_sub(radius);
        let y_min = y.saturating_sub(radius);
        let x_max = (x + radius).min(self.width);
        let y_max = (y + radius).min(self.height);
        ((x_min, y_min), x_max - x_min, y_max - y_min)
    }

    /// Samples a cell with a given chance
    pub fn sample_radius(&mut self, (x, y): (usize, usize), radius: usize) {
        let (n, width, height) = self.radius_calc((x, y), radius);
        self.sample_area(n, width, height);
    }

    /// Samples all cells in the grid
    pub fn sample_all(&mut self) {
        self.sample_area((0, 0), self.width, self.height)
    }

    /// Samples a cell with a given chance
    /// ## Arguments
    /// * `(x, y)` - The coordinate of the cell
    /// * `measurement_covariance` - The variance of the measurement where 0.0 is a perfect measurement
    pub fn update_node(&mut self, (x, y): (usize, usize), measurement_covariance: f32) {
        let measurement = self.ground_truth.get_bit_value((x, y)) as u8 as f32;
        self.sample_grid[x][y].update(measurement, measurement_covariance);
    }

    /// Creates a kernel for the adjacency of a point. Used to update nodes with
    /// matrix representing covariance.
    pub fn adjacency_kernel(&self, kernel: &Matrix<f32>) -> Matrix<f32> {
        let mut kernel = kernel.clone();
        let radius = kernel.width / 2;
        kernel[radius][radius] = 0.0;
        kernel[radius.saturating_sub(1)][radius] = 0.0; // This is to ensure that
        kernel[radius][radius.saturating_sub(1)] = 0.0; // the center's measurements
        kernel[radius.saturating_add(1)][radius] = 0.0; // are always correct
        kernel[radius][radius.saturating_add(1)] = 0.0;
        kernel
    }

    /// Updates nodes based upon a kernal
    fn update_kernel(&mut self, (x, y): (usize, usize), kernel: Matrix<f32>) {
        let kernel_size = (kernel.width, kernel.height);
        for (n, (i, j)) in matrix_overlay((self.width, self.height), kernel_size, (x, y)) {
            self.update_node(n, kernel[i][j]);
        }
    }

    /// Updates nodes based upon a radius
    pub fn update_radius(&mut self, (x, y): (usize, usize), kernel: &Matrix<f32>) {
        let kernel = self.adjacency_kernel(kernel);
        self.update_kernel((x, y), kernel);
    }

    /// Updates nodes based on visibile nodes
    pub fn raycast_update(&mut self, (x, y): (usize, usize), kernel: &Matrix<f32>) -> usize {
        let mut kernel = self.adjacency_kernel(kernel);

        let visible = raycast_matrix(
            (x, y),
            kernel.width / 2,
            |x, y| {
                self.sample_grid[x.min(self.width - 1)][y.min(self.height - 1)].state
                    > Self::NEAREST_THRESHOLD
            },
            |x, y| self.bound_check((x, y)),
        );
        for (k, v) in kernel.data.iter_mut().zip(visible.data.iter()) {
            if *v {
                *k = 1.0;
            }
        }
        self.update_kernel((x, y), kernel);
        visible.data.iter().filter(|x| **x).count()
    }

    /// Checks if within bounds
    pub fn bound_check(&self, (x, y): (usize, usize)) -> bool {
        x < self.width && y < self.height
    }

    /// Get all adjacenct cells on sampling grid
    pub fn adjacent(
        &self,
        (x, y): (usize, usize),
        diagonal: bool,
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        super::neighbors(x, y, diagonal).filter(|(x, y)| self.bound_check((*x, *y)))
    }

    /// Samples all adjacent cells on sampling grid
    pub fn sample_adjacenct(
        &mut self,
        (x, y): (usize, usize),
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        super::neighbors(x, y, false).filter(move |(x, y)| {
            if self.bound_check((*x, *y)) && !self.sampled_before.get_bit_value((*x, *y)) {
                self.sample((*x, *y))
            } else {
                self.gridmap.get_bit_value((*x, *y))
            }
        })
    }

    /// Prints the sampling cell grid for debugging
    pub fn print_sampling_cells(&self, path: Option<Vec<(usize, usize)>>) -> String {
        print_cells(
            self.width,
            self.height,
            |x, y| self.sample_grid[x][y].state != 0.0,
            path,
        )
    }

    /// Plots the sampling cell grid for debugging
    pub fn plot_sampling_cells(
        &self,
        output_file: &str,
        path: Option<Vec<(usize, usize)>>,
        heatmap: Option<Vec<((usize, usize), f64)>>,
    ) {
        plot_cells(
            self.width,
            self.height,
            output_file,
            |x, y| self.sample_grid[x][y].state != 0.0,
            path,
            heatmap,
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        domains::bitpackedgrid::BitPackedGrid,
        matrix,
        util::matrix::{gaussian_kernal, Matrix},
    };

    use super::SampleGrid;

    #[test]
    fn test_samplegrid_new() {
        let grid = SampleGrid::new_with_size(128, 128);
        assert_eq!(grid.height, 128);
        assert_eq!(grid.width, 128);
        assert_eq!(grid.sample_grid.height, 128);
        assert_eq!(grid.sample_grid.width, 128);
    }

    #[test]
    fn test_samplegrid_new_from_grid() {
        let grid =
            SampleGrid::new_from_grid(matrix![[0.0, 1.0], [1.0, 0.0]], BitPackedGrid::new(2, 2));
        assert_eq!(grid.sample_grid[0][0].state, 0.0);
        assert_eq!(grid.sample_grid[1][0].state, 1.0);
        assert_eq!(grid.gridmap.get_bit_value((0, 0)), false);
        assert_eq!(grid.gridmap.get_bit_value((1, 0)), true);
        assert_eq!(grid.gridmap.get_bit_value((0, 1)), true);
    }

    #[test]
    fn test_samplegrid_create() {
        let map_str = ".....\n@@.@.\n.@.@.\n.@.@.\n.....\n....@\n";
        let grid = SampleGrid::new_from_string(map_str.to_string());
        assert_eq!(grid.ground_truth.print_cells(None), map_str);
        assert_eq!(grid.sample_grid[0][0].state, 1.0);
        assert_eq!(grid.sample_grid[1][0].state, 1.0);
        assert_eq!(grid.sample_grid[0][1].state, 0.0);
        assert_eq!(grid.gridmap.get_bit_value((0, 0)), true);
        assert_eq!(grid.gridmap.get_bit_value((1, 0)), true);
        assert_eq!(grid.gridmap.get_bit_value((0, 1)), false);
    }

    #[test]
    fn test_gridmap_init() {
        let mut grid = SampleGrid::new_from_string("@....\n.....\n.....\n.....\n".to_string());
        grid.gridmap = BitPackedGrid::new(grid.width, grid.height);
        grid.init_gridmap_radius((0, 0), 2);
        grid.init_ground_truth();
        assert_eq!(
            grid.ground_truth.print_cells(None),
            "@....\n.....\n.....\n.....\n"
        );
        assert_eq!(
            grid.gridmap.print_cells(None),
            "@..@@\n...@@\n...@@\n@@@@@\n"
        );
    }

    #[test]
    fn test_blur() {
        let mut grid =
            SampleGrid::new_from_string("@....\n@@...\n@@@..\n@@@..\n@@...\n".to_string());
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
}
