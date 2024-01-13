//! Matrix utilities.

use std::ops::{Add, Mul, Index, IndexMut};

#[derive(PartialEq, Eq, Debug, Clone)]
pub struct Matrix<T> {
    pub data: Box<[T]>,
    pub width: usize,
    pub height: usize,
}

#[macro_export]
#[allow(unused_assignments)]
macro_rules! matrix {
    ( $([$($x:expr),* $(,)?]),* $(,)?) => {
        {
            let mut data = Vec::new();
            let mut height = 0;
            let mut width = 0;
            $(
                height += 1;
                let row = [$($x),*];
                width = width.max(row.len());
                data.extend_from_slice(&row);
            )*
            Matrix {
                data: data.into_boxed_slice(),
                width,
                height,
            }
        }
    };
    ($x: expr; $s: expr) => {
        Matrix {
            data: vec![$x; $s * $s].into_boxed_slice(),
            width: $s,
            height: $s,
        }
    };
    ($x:expr; $w:expr, $h:expr) => {
        Matrix {
            data: vec![$x; $w * $h].into_boxed_slice(),
            width: $w,
            height: $h,
        }
    };
}

impl<T> Index<usize> for Matrix<T> {
    type Output = [T];
    fn index(&self, row: usize) -> &Self::Output {
        &self.data[row * self.width..(row + 1) * self.width]
    }
}

impl<T> IndexMut<usize> for Matrix<T> {
    fn index_mut(&mut self, row: usize) -> &mut Self::Output {
        &mut self.data[row * self.width..(row + 1) * self.width]
    }
}

/// Convolve a matrix with a kernel.
/// * `matrix` - The matrix to convolve.
/// * `kernel` - The kernel to convolve with.
/// * `resolution` - The resolution to use when resolving the matrix at the edges.
pub fn convolve2d<T, K>(
    matrix: &Matrix<T>,
    kernel: &Matrix<K>,
    resolution: ConvResolve<T>
) -> Matrix<T> 
where 
    T: Clone + Default + Add<Output = T> + Mul<K, Output = T>,
    K: Clone,
{
    let mut result = matrix![T::default(); matrix.width, matrix.height];
    let kernel_center_x = kernel.width / 2;
    let kernel_center_y = kernel.height / 2;
    for x in 0..matrix.width {
        for y in 0..matrix.height {
            let mut sum = T::default();
            for i in 0..kernel.width {
                for j in 0..kernel.height {
                    let matrix_x = (x + i) as i64 - kernel_center_x as i64;
                    let matrix_y = (y + j) as i64 - kernel_center_y as i64;
                    sum = sum + resolution.resolve(matrix, matrix_x, matrix_y) * kernel[i][j].clone();
                }
            }
            result[y][x] = sum;
        }
    }
    result
}

/// Resolution method for resolving the value of a matrix at the edges.
pub enum ConvResolve<T: Clone> {
    /// Fill the matrix with the given value. |T T T|a b c|T T T|
    Fill(T),
    /// Wrap around the matrix. |a b c|a b c|b c|
    Wrap,
    /// Use the nearest value. |a a a|a b c|c c c|
    Nearest,
    /// Reflect the matrix. |c b a|a b c|c b a|
    Reflect
}

impl<T: Clone> ConvResolve<T> {
    /// Resolve the value of a matrix at a given position.
    fn resolve(&self, matrix: &Matrix<T>, matrix_x: i64, matrix_y: i64) -> T {
        let (matrix_x, matrix_y) = match self {
            ConvResolve::Fill(fill) => 
                if matrix_x >= 0 && matrix_y >= 0 && matrix_x < matrix.width as i64 && matrix_y < matrix.height as i64 {
                    (matrix_x, matrix_y)
                } else {
                    return fill.clone()
                },
            ConvResolve::Wrap => (
                (matrix_x + matrix.width as i64) % matrix.width as i64,
                (matrix_y + matrix.height as i64) % matrix.height as i64
            ), 
            ConvResolve::Nearest => (
                matrix_x.clamp(0, matrix.width as i64 - 1),
                matrix_y.clamp(0, matrix.height as i64 - 1)
            ),
            ConvResolve::Reflect => (
                if matrix_x < 0 {
                    matrix_x.abs() - 1 
                } else if matrix_x >= matrix.width as i64 {
                    matrix_x - matrix.width as i64 + 1
                }else {
                    matrix_x
                },
                if matrix_y < 0 {
                    matrix_y.abs() - 1 
                } else if matrix_y >= matrix.height as i64 {
                    matrix_y - matrix.height as i64 + 1
                } else { 
                    matrix_y
                },
            ),
        };
        matrix[matrix_y as usize][matrix_x as usize].clone()
    }
}

pub fn matrix_overlay(
    (matrix_w, matrix_h): (usize, usize),
    (kernel_w, kernel_h): (usize, usize),
    (x, y): (usize, usize),
) -> impl Iterator<Item = ((usize, usize), (usize, usize))>
{
    let kernel_center_x = kernel_w / 2;
    let kernel_center_y = kernel_h / 2;
    (0..kernel_w).flat_map(move |i| (0..kernel_h).map(move |j| (i, j)))
        .filter_map(move |(i, j)| {
            let (matrix_x, underflow_x) = (x + i).overflowing_sub(kernel_center_x);
            let (matrix_y, underflow_y) = (y + j).overflowing_sub(kernel_center_y);
            if !underflow_x && !underflow_y && matrix_x < matrix_w && matrix_y < matrix_h {
                Some(((matrix_x, matrix_y), (i, j)))
            } else {
                None
            }
        })
}

/// Create a gaussian kernel.
/// * `size` - The size of the kernel.
/// * `sigma` - The sigma value of the gaussian.
pub fn gaussian_kernal(size: usize, sigma: f32) -> Matrix<f32> {
    let mut kernel = matrix![0.0; size];
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

    /// Create a matrix that count up from 1 to n * m.
    fn count(n: usize, m: usize) -> Matrix<i32> {
        let mut matrix = matrix![0; m, n];
        for i in 0..n {
            for j in 0..m {
                matrix[i][j] = (i * m + j + 1) as i32;
            }
        }
        matrix
    }

    #[test]
    fn test_macro() {
        let matrix = matrix![
            [1, 2, 3],
            [4, 5, 6],
        ];
        assert_eq!(matrix.width, 3);
        assert_eq!(matrix.height, 2);
        assert_eq!(matrix.data, vec![1, 2, 3, 4, 5, 6].into_boxed_slice());
    }

    #[test]
    fn test_matrix_access() {
        let matrix = Matrix {
            width: 3, 
            height: 3, 
            data: vec![1, 2, 3, 4, 5, 6, 7, 8, 9].into_boxed_slice(),
        };
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(matrix[i][j], (i * 3 + j + 1) as i32);
            }
        }
    }

    #[test]
    fn test_convolve2d() {
        let matrix = count(2, 3);
        let kernel = matrix![1; 3];
        let result = convolve2d(&matrix, &kernel, ConvResolve::Nearest);
        assert_eq!(result, matrix![ 
            [21, 27, 33], 
            [30, 36, 42]
        ]);
    }

    #[test]
    fn test_convolve2d_fill() {
        let matrix = count(3, 3);
        let kernel = matrix![1; 3];
        let result = convolve2d(&matrix, &kernel, ConvResolve::Fill(0));
        assert_eq!(result, matrix![
            [12, 21, 16],
            [27, 45, 33],
            [24, 39, 28],
        ]);
    }

    #[test]
    fn test_convolve2d_wrap() {
        let matrix = count(2, 2);
        let kernel = matrix![1; 2];
        let result = convolve2d(&matrix, &kernel, ConvResolve::Wrap);
        assert_eq!(result, matrix![
            [10, 10],
            [10, 10],
        ]);
    }

    #[test]
    fn test_convolve_reflect() {
        let matrix = count(3, 3);
        let kernel = matrix![1; 3];
        let result = convolve2d(&matrix, &kernel, ConvResolve::Reflect);
        assert_eq!(result, matrix![
            [21, 27, 30],
            [39, 45, 48],
            [48, 54, 57],
        ]);
    }

    #[test]
    fn test_convolve2d_nearest() {
        let matrix = count(3, 3);
        let kernel = matrix![1; 3];
        let result = convolve2d(&matrix, &kernel, ConvResolve::Nearest);
        assert_eq!(result, matrix![
            [21, 27, 33],
            [39, 45, 51],
            [57, 63, 69],
        ]);
    }

    #[test]
    fn test_matrix2d_combine() {
        let mut matrix = count(3, 3);
        let kernel = matrix![1; 5];
        for ((x, y), (i, j)) in matrix_overlay((matrix.width, matrix.height), (kernel.width, kernel.height), (1, 1)) {
            matrix[x][y] += kernel[i][j];
        }
        assert_eq!(matrix, matrix![
            [2, 3, 4],
            [5, 6, 7],
            [8, 9, 10],
        ]);
    }

    #[test]
    fn test_gaussian_kernal() {
        let kernel = gaussian_kernal(3, 1.0);
        assert_eq!(kernel, matrix![
            [0.07511361, 0.12384141, 0.07511361],
            [0.12384141, 0.20417996, 0.12384141],
            [0.07511361, 0.12384141, 0.07511361],
        ]);
    }

    #[test]
    fn test_convolve2d_gauss() {
        let matrix = matrix![
            [0.0, 1.0, 1.0, 1.0, 1.0],
            [0.0, 0.0, 1.0, 1.0, 1.0],
            [0.0, 0.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, 1.0, 1.0, 1.0],
        ];
        let kernel = gaussian_kernal(3, 1.0);
        let result = convolve2d(&matrix, &kernel, ConvResolve::Nearest);
        assert_eq!(result, matrix![
            [0.19895503, 0.60209, 0.9248864, 1.0, 1.0], 
            [0.07511361, 0.32279643, 0.6772036, 0.9248864, 1.0],
            [0.0, 0.07511361, 0.39791006, 0.801045, 1.0],
            [0.0, 0.07511361, 0.39791006, 0.801045, 1.0],
            [0.0, 0.19895503, 0.60209, 0.9248864, 1.0],
        ]);
    }
}
