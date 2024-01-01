//! Matrix utilities.

use std::ops::{Add, Mul};

/// Convolve a matrix with a kernel.
/// * `matrix` - The matrix to convolve.
/// * `kernel` - The kernel to convolve with.
/// * `resolution` - The resolution to use when resolving the matrix at the edges.
pub fn convolve2d<T: Add<Output = T> + Mul<Output = T> + Clone + Default>(
    matrix: &Vec<Vec<T>>,
    kernel: &Vec<Vec<T>>,
    resolution: ConvResolve<T>
) -> Vec<Vec<T>> {
    let mut result = vec![vec![T::default(); matrix[0].len()]; matrix.len()];
    let kernel_width = kernel.len();
    let kernel_height = kernel[0].len();
    let kernel_center_x = kernel_width / 2;
    let kernel_center_y = kernel_height / 2;
    for x in 0..matrix.len() {
        for y in 0..matrix[0].len() {
            let mut sum = T::default();
            for i in 0..kernel_width {
                for j in 0..kernel_height {
                    let matrix_x = (x + i) as i64 - kernel_center_x as i64;
                    let matrix_y = (y + j) as i64 - kernel_center_y as i64;
                    sum = sum + resolution.resolve(matrix, matrix_x, matrix_y) * kernel[i][j].clone();
                }
            }
            result[x][y] = sum;
        }
    }
    result
}

/// Resolution method for resolving the value of a matrix at the edges.
pub enum ConvResolve<T: Clone> {
    /// Fill the matrix with the given value.
    Fill(T),
    /// Wrap around the matrix.
    Wrap,
    Nearest,
    Reflect
}

impl<T: Clone> ConvResolve<T> {
    /// Resolve the value of a matrix at a given position.
    fn resolve(&self, matrix: &Vec<Vec<T>>, matrix_x: i64, matrix_y: i64) -> T {
        let (matrix_x, matrix_y) = match self {
            ConvResolve::Fill(fill) => 
                if matrix_x >= 0 && matrix_y >= 0 && matrix_x < matrix.len() as i64 && matrix_y < matrix[0].len() as i64 {
                    (matrix_x, matrix_y)
                } else {
                    return fill.clone()
                },
            ConvResolve::Wrap => (
                (matrix_x + matrix.len() as i64) % matrix.len() as i64,
                (matrix_y + matrix[0].len() as i64) % matrix[0].len() as i64
            ), 
            ConvResolve::Nearest => (
                matrix_x.clamp(0, matrix.len() as i64 - 1),
                matrix_y.clamp(0, matrix[0].len() as i64 - 1)
            ),
            ConvResolve::Reflect => (
                if matrix_x < 0 {
                    matrix_x.abs() - 1 
                } else if matrix_x >= matrix.len() as i64 {
                    matrix_x - matrix.len() as i64 + 1
                }else {
                    matrix_x
                },
                if matrix_y < 0 {
                    matrix_y.abs() - 1 
                } else if matrix_y >= matrix[0].len() as i64 {
                    matrix_y - matrix[0].len() as i64 + 1
                } else { 
                    matrix_y
                },
            ),
        };
        matrix[matrix_x as usize][matrix_y as usize].clone()
    }
}

/// Create a gaussian kernel.
/// * `size` - The size of the kernel.
/// * `sigma` - The sigma value of the gaussian.
pub fn gaussian_kernal(size: usize, sigma: f32) -> Vec<Vec<f32>> {
    let mut kernel = vec![vec![0.0; size]; size];
    let center = size / 2;
    let sigma = sigma * sigma;
    let mut sum = 0.0;
    for i in 0..size {
        for j in 0..size {
            let x = i as i32 - center as i32;
            let y = j as i32 - center as i32;
            let value = (-((x * x + y * y) as f32) / (2.0 * sigma)).exp();
            sum += value;
            kernel[i][j] = value;
        }
    }
    for i in 0..size {
        for j in 0..size {
            kernel[i][j] /= sum;
        }
    }
    kernel
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ones(n: usize) -> Vec<Vec<i32>> {
        vec![vec![1; n]; n]
    }

    fn count(n: usize, m: usize) -> Vec<Vec<i32>> {
        (1..=n).map(|i| (1..=m).map(|j| ((i - 1) * n + j) as i32).collect()).collect()
    }

    #[test]
    fn test_convolve2d_fill() {
        let matrix = count(3, 3);
        let kernel = ones(3);
        let result = convolve2d(&matrix, &kernel, ConvResolve::Fill(0));
        assert_eq!(result, vec![
            vec![12, 21, 16],
            vec![27, 45, 33],
            vec![24, 39, 28],
        ]);
    }

    #[test]
    fn test_convolve2d_wrap() {
        let matrix = count(2, 2);
        let kernel = ones(2);
        let result = convolve2d(&matrix, &kernel, ConvResolve::Wrap);
        assert_eq!(result, vec![
            vec![10, 10],
            vec![10, 10],
        ]);
    }

    #[test]
    fn test_convolve_reflect() {
        let matrix = count(3, 3);
        let kernel = ones(3);
        let result = convolve2d(&matrix, &kernel, ConvResolve::Reflect);
        assert_eq!(result, vec![
            vec![21, 27, 30],
            vec![39, 45, 48],
            vec![48, 54, 57],
        ]);
    }

    #[test]
    fn test_convolve2d_nearest() {
        let matrix = count(3, 3);
        let kernel = ones(3);
        let result = convolve2d(&matrix, &kernel, ConvResolve::Nearest);
        assert_eq!(result, vec![
            vec![21, 27, 33],
            vec![39, 45, 51],
            vec![57, 63, 69],
        ]);
    }

    #[test]
    fn test_gaussian_kernal() {
        let kernel = gaussian_kernal(3, 1.0);
        assert_eq!(kernel, vec![
            vec![0.07511361, 0.12384141, 0.07511361],
            vec![0.12384141, 0.20417996, 0.12384141],
            vec![0.07511361, 0.12384141, 0.07511361],
        ]);
    }

    #[test]
    fn test_convolve2d_gauss() {
        let matrix = vec![
            vec![0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0, 1.0],
        ];
        let kernel = gaussian_kernal(3, 1.0);
        let result = convolve2d(&matrix, &kernel, ConvResolve::Nearest);
        assert_eq!(result, vec![
            vec![0.19895503, 0.60209, 0.9248864, 1.0, 1.0], 
            vec![0.07511361, 0.32279643, 0.6772036, 0.9248864, 1.0],
            vec![0.0, 0.07511361, 0.39791006, 0.801045, 1.0],
            vec![0.0, 0.07511361, 0.39791006, 0.801045, 1.0],
            vec![0.0, 0.19895503, 0.60209, 0.9248864, 1.0],
        ]);
    }
}
