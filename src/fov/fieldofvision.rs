//! These are RPG approaches to field of vision algorithms. These algorithms include:
//! * Raycasting
//! * Recursive Shadowcasting
//! These implementations are generally based upon code from RogueBasin.

use ahash::AHashMap;

use super::linedrawing::bresenham;
use crate::matrix;
use crate::util::matrix::Matrix;

/// Raycast to produce a matrix of visibility where 1 indicates visibility, uses 
/// Bresenham's line algorithm to calculate the raycast.
pub fn raycast_matrix(
    (x, y): (isize, isize),
    radius: usize,
    visibility_check: impl Fn((isize, isize)) -> bool,
    bounds_check: impl Fn((isize, isize)) -> bool
) -> Matrix<bool> {
    let mut visible: Vec<(isize, isize)> = Vec::new();
    let r = radius as isize;
    for i in 0..2 * r {
        let positions: [(isize, isize); 4] = [
            (x - r + i, y - r),
            (x + r, y - r + i),
            (x + r - i, y + r),
            (x - r, y + r - i),
        ];
        for (px, py) in positions.iter() {
            visible.extend(
                bresenham((x, y), (*px, *py), &visibility_check, &bounds_check).iter(),
            );
        }
    }
    let mut visible_matrix = matrix![false; 2*radius+1, 2*radius+1];
    for (y, x) in visible
        .iter()
        .map(|(xi, yi)| (xi + r - x, yi + r - y))
    {
        visible_matrix[x as usize][y as usize] = true;
    }
    visible_matrix
}

/// Shadowcasting implementation for field of view
/// 
/// Adapted from java versions found at:
/// https://www.roguebasin.com/index.php/Improved_Shadowcasting_in_Java
pub fn shadowcasting(
    (x, y): (usize, usize),
    radius: usize,
    bounds_check: impl Fn(usize, usize) -> bool,
    visibility_check: impl Fn(usize, usize) -> bool,
) -> Vec<((usize, usize), f32)> {
    let offsets = vec![(-1, -1), (1, -1), (-1, 1), (1, 1)];
    let mut light_map = AHashMap::new();
    light_map.insert((x, y), 1.0);
    for (dx, dy) in offsets {
        lightcast(
            (x, y),
            1,
            1.0,
            0.0,
            (0, dx, dy, 0),
            radius,
            &mut light_map,
            &bounds_check,
            &visibility_check,
        );
        lightcast(
            (x, y),
            1,
            1.0,
            0.0,
            (dx, 0, 0, dy),
            radius,
            &mut light_map,
            &bounds_check,
            &visibility_check,
        );
    }
    light_map.into_iter().collect()
}

/// This is a helper function for shadowcasting that recursively casts light.
fn lightcast(
    (x, y): (usize, usize),
    row: usize,
    mut start: f32,
    end: f32,
    (xx, xy, yx, yy): (isize, isize, isize, isize),
    radius: usize,
    light_map: &mut AHashMap<(usize, usize), f32>,
    bounds_check: &impl Fn(usize, usize) -> bool,
    visibility_check: &impl Fn(usize, usize) -> bool,
) {
    let mut new_start = 0.0;
    if start < end {
        return;
    }
    let mut blocked = false;
    for dist in row..=radius {
        if blocked {
            break;
        }
        let dist = dist as isize;
        let dy = -dist;
        for dx in -dist..=0 {
            let cur_x = x as isize + dx * xx + dy * xy;
            let cur_y = y as isize + dx * yx + dy * yy;
            let left_slope = (dx as f32 - 0.5) / (dy as f32 + 0.5);
            let right_slope = (dx as f32 + 0.5) / (dy as f32 - 0.5);
            if !(cur_x >= 0 && cur_y >= 0 && bounds_check(cur_x as usize, cur_y as usize))
                || start < right_slope
            {
                continue;
            } else if end > left_slope {
                break;
            }

            // Currently, this is a square light cast, but it could be changed to a square
            // light cast by changing the if statement to check if dx + dy <= radius as isize
            // circular if statement: if dx * dx + dy * dy <= radius * radius as isize
            if dx + dy <= radius as isize {
                let bright = (1.0 - (dx * dx + dy * dy) as f32 / (radius * radius) as f32).sqrt();
                light_map.insert((cur_x as usize, cur_y as usize), bright);
            }

            if blocked {
                if !visibility_check(cur_x as usize, cur_y as usize) {
                    new_start = right_slope;
                } else {
                    blocked = false;
                    start = new_start;
                }
            } else {
                if !visibility_check(cur_x as usize, cur_y as usize) && dist < radius as isize {
                    blocked = true;
                    lightcast(
                        (x, y),
                        usize::try_from(dist + 1).unwrap(),
                        start,
                        left_slope,
                        (xx, xy, yx, yy),
                        radius,
                        light_map,
                        bounds_check,
                        visibility_check,
                    );
                    new_start = right_slope;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{domains::{bitpackedgrids::bitpackedgrid2d::BitPackedGrid2d, Domain, Grid2d, GridCreate2d}, matrix, util::matrix::Matrix};

    use super::{raycast_matrix, shadowcasting};

    #[test]
    fn test_raycast_matrix() {
        let visibility_all = raycast_matrix((10, 10), 2, |_| true, |_| true);
        assert_eq!(visibility_all.data.iter().filter(|x| **x).count(), 25);
        let visibility_top_obstacle =
            raycast_matrix(
                (10, 10), 
                2, 
                |n| n != (10, 9),
                |n| n < (20, 20)
            );
        assert_eq!(
            visibility_top_obstacle.data.iter().filter(|x| **x).count(),
            22
        );
    }

    #[test]
    fn test_raycast_on_grid() {
        let grid = BitPackedGrid2d::new_from_string(
            ".....\n.###.\n.#...\n.#.#.\n...#.".to_string(),
        );
        let visibility = raycast_matrix(
            (2, 2), 
            2, 
            |(x, y)| grid.get_value((x as usize, y as usize)), 
            |(x, y)| grid.bounds_check((x as usize, y as usize))
        );
        assert_eq!(visibility, matrix![
            [false, false, false, false, false],
            [false, true, true, true, true],
            [false, true, true, true, true],
            [false, true, true, true, true],
            [false, true, true, true, false],
        ]);
    }

    #[test]
    fn test_shadowcasting() {
        let visibility_all = shadowcasting((10, 10), 2, |_, _| true, |_, _| true);
        assert_eq!(visibility_all.len(), 25);
        let visibility_top_obstacle =
            shadowcasting((10, 10), 2, |_, _| true, |x, y| (x, y) != (10, 9));
        assert_eq!(visibility_top_obstacle.len(), 24);
    }
}
