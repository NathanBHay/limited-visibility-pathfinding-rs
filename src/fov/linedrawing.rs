//! Line drawing algorithms that draw a rasterized line between two points.
//! This has two approaches one basic one that uses floating points, here
//! as a baseline, and Bresenham's line algorithm the more efficient one.
//!
//! A possible expansion in the future is implementations that use chain
//! codes rather than a list. A chain code is a binary encoding of the
//! lines where 0 represents the next pixel being placed to the right,
//! while 1 represents the next pixel being placed to the right and up.
//! Source for codes: https://www.roguebasin.com/index.php/Digital_lines

/// Basic line drawing algorithm, inefficient but simple.
pub fn basic_line((x0, y0): (usize, usize), (x1, y1): (usize, usize)) -> Vec<(usize, usize)> {
    let (x0, y0, x1, y1) = (x0 as i32, y0 as i32, x1 as i32, y1 as i32);
    let mut line = Vec::new();
    let m = (y1 - y0) as f32 / (x1 - x0) as f32;
    let c = y0 as f32 - m * x0 as f32;
    let (l, r) = if x0 < x1 { (x0, x1) } else { (x1, x0) };
    for x in l..(r + 1) {
        let y = (m * x as f32 + c).round();
        line.push((x as usize, y as usize));
    }
    if x0 > x1 {
        line.reverse();
    }
    line
}

/// Bresenham's line algorithm for all 2D octants.
/// This implementation is adapted from the JS version found here:
/// https://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
/// This approach draws the line from the origin to the end point,
/// making it able to be used for raycasting.
/// Possibly could include a expansion function that allows the line
/// to work out whether line of sight is blocked thus terminating
/// the algorithm.
/// 
/// Note: The visibility check includes the point itself as it is assumed to be 
/// the object blocking visibility.
pub fn bresenham(
    (mut x0, mut y0): (isize, isize),
    (mut x1, mut y1): (isize, isize),
    visibility_check: impl Fn((isize, isize)) -> bool,
    bounds_check: impl Fn((isize, isize)) -> bool,
) -> Vec<(isize, isize)> {
    let steep = (y1 - y0).abs() > (x1 - x0).abs();
    if steep {
        (x0, y0, x1, y1) = (y0, x0, y1, x1)
    };
    let mut sign = 1;
    if x0 > x1 {
        (sign, x0, x1) = (-1, -1 * x0, -1 * x1)
    };
    let dx = x1 - x0;
    let dy = (y1 - y0).abs();
    let ystep = if y0 < y1 { 1 } else { -1 };
    let mut error = dx / 2;
    let mut y = y0;
    let mut line = Vec::new();

    for x in x0..=x1 {
        let point = if steep {
            (y, (sign * x))
        } else {
            ((sign * x), y)
        };
        if !bounds_check(point) {
            break;
        }
        line.push(point);
        if !visibility_check(point) {
            break;
        }
        error -= dy;
        if error < 0 {
            y += ystep;
            error += dx;
        }
    }
    line
}

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn test_basic_line() {
        let line_octant12 = basic_line((0, 0), (3, 3));
        assert_eq!(line_octant12, vec![(0, 0), (1, 1), (2, 2), (3, 3)]);
        let line_octant34 = basic_line((0, 3), (3, 0));
        assert_eq!(line_octant34, vec![(0, 3), (1, 2), (2, 1), (3, 0)]);
        let line_octant56 = basic_line((3, 3), (0, 0));
        assert_eq!(line_octant56, vec![(3, 3), (2, 2), (1, 1), (0, 0)]);
        let line_octant78 = basic_line((3, 0), (0, 3));
        assert_eq!(line_octant78, vec![(3, 0), (2, 1), (1, 2), (0, 3)]);
    }

    #[test]
    fn test_breseham() {
        let line_octant12 = bresenham((0, 0), (3, 3), |n| n != (1, 1), |_| true);
        assert_eq!(line_octant12, vec![(0, 0), (1, 1)]);
        let line_octant34 = bresenham((0, 3), (3, 0), |_| true, |_| true);
        assert_eq!(line_octant34, vec![(0, 3), (1, 2), (2, 1), (3, 0)]);
        let line_octant56 = bresenham((3, 3), (0, 0), |n| n != (1, 1), |_| true);
        assert_eq!(line_octant56, vec![(3, 3), (2, 2), (1,1)]);
        let line_octant78 = bresenham((3, 0), (0, 3), |n| n != (0, 3), |_| true);
        assert_eq!(line_octant78, vec![(3, 0), (2, 1), (1, 2), (0, 3)]);
    }
}
