//! These are RPG approaches to field of vision algorithms. These algorithms include:
//! * Raycasting
//! * Recursive Shadowcasting
//! These implementations are generally based upon code from RogueBasin.

use std::collections::HashMap;

use super::linedrawing::bresenham;
use crate::matrix;
use crate::util::matrix::Matrix;

/// Raycast to produce a matrix of visibility where 1 indicates visibility
pub fn raycast_matrix(
    (x, y): (usize, usize),
    radius: usize,
    mut visibility_check: impl FnMut(usize, usize) -> bool,
    bounds_check: impl Fn(usize, usize) -> bool,
) -> Matrix<bool> {
    let mut visible: Vec<(usize, usize)> = Vec::new();
    let (ix, iy) = (x as i32, y as i32);
    let r = radius as i32;
    for i in 0..2 * r {
        let positions: [(i32, i32); 4] = [
            (ix - r + i, iy - r),
            (ix + r, iy - r + i),
            (ix + r - i, iy + r),
            (ix - r, iy + r - i),
        ];
        for (px, py) in positions
            .iter()
            .filter(|(x, y)| x >= &0 && y >= &0 && bounds_check(*x as usize, *y as usize))
        {
            visible.extend(
                bresenham((x, y), (*px as usize, *py as usize), &mut visibility_check).iter(),
            );
        }
    }
    let mut visible_matrix = matrix![false; 2*radius+1, 2*radius+1];
    for (x, y) in visible
        .iter()
        .map(|(xi, yi)| (xi + radius - x, yi + radius - y))
    {
        visible_matrix[y][x] = true;
    }
    visible_matrix
}

/// Raycasting approach to field of vision
pub fn raycasting(
    (x, y): (usize, usize),
    radius: usize,
    visibility_check: impl FnMut(usize, usize) -> bool,
) -> Vec<(usize, usize)> {
    raycasting_with_dist((x, y), radius, visibility_check, |_| 1)
        .iter()
        .map(|(p, _)| *p)
        .collect::<Vec<_>>()
}

pub fn raycasting_with_dist(
    (x, y): (usize, usize),
    radius: usize,
    mut visibility_check: impl FnMut(usize, usize) -> bool,
    visibility_func: impl Fn(usize) -> usize,
) -> Vec<((usize, usize), usize)> {
    let mut visible = HashMap::new();
    let visibility_kernal = (0..=radius).map(visibility_func).collect::<Vec<_>>();
    for i in 0..2 * radius {
        let positions = [
            (
                x.saturating_sub(radius).saturating_add(i),
                y.saturating_sub(radius),
            ),
            (
                x.saturating_add(radius),
                y.saturating_sub(radius).saturating_add(i),
            ),
            (
                x.saturating_add(radius).saturating_sub(i),
                y.saturating_add(radius),
            ),
            (
                x.saturating_sub(radius),
                y.saturating_add(radius).saturating_sub(i),
            ),
        ];
        for position in positions.iter() {
            visible.extend(
                bresenham((x, y), *position, &mut visibility_check)
                    .iter()
                    .zip(visibility_kernal.iter()),
            );
        }
    }
    visible.into_iter().collect::<Vec<_>>()
}

/// Shadowcasting implementation for field of view
/// Adapted from java versions found at:
/// https://www.roguebasin.com/index.php/Improved_Shadowcasting_in_Java
pub fn shadowcasting(
    (x, y): (usize, usize),
    radius: usize,
    bounds_check: impl Fn(usize, usize) -> bool,
    visibility_check: impl Fn(usize, usize) -> bool,
) -> Vec<((usize, usize), f32)> {
    let offsets = vec![(-1, -1), (1, -1), (-1, 1), (1, 1)];
    let mut light_map = HashMap::new();
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

/// This is a helper function for shadowcasting that recursively casts
/// light.
fn lightcast(
    (x, y): (usize, usize),
    row: usize,
    mut start: f32,
    end: f32,
    (xx, xy, yx, yy): (isize, isize, isize, isize),
    radius: usize,
    light_map: &mut HashMap<(usize, usize), f32>,
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

    use super::{raycast_matrix, raycasting, shadowcasting};

    #[test]
    fn test_raycasting() {
        let visibility_all = raycasting((10, 10), 2, |_, _| true);
        assert_eq!(visibility_all.len(), 25);
        let visibility_top_obstacle = raycasting((10, 10), 2, |x, y| (x, y) != (10, 9));
        assert_eq!(visibility_top_obstacle.len(), 21);
    }

    #[test]
    fn test_raycast_matrix() {
        let visibility_all = raycast_matrix((10, 10), 2, |_, _| true, |_, _| true);
        assert_eq!(visibility_all.data.iter().filter(|x| **x).count(), 25);
        let visibility_top_obstacle =
            raycast_matrix((10, 10), 2, |x, y| (x, y) != (10, 9), |_, _| true);
        assert_eq!(
            visibility_top_obstacle.data.iter().filter(|x| **x).count(),
            21
        );
    }

    #[test]
    fn test_shadowcasting() {
        let visibility_all = shadowcasting((10, 10), 2, |_, _| true, |_, _| true);
        assert_eq!(visibility_all.len(), 25);
        let visibility_top_obstacle =
            shadowcasting((10, 10), 2, |_, _| true, |x, y| (x, y) != (10, 9));
        assert_eq!(visibility_top_obstacle.len(), 24);
    }

    #[test]
    fn test_raycasting_with_func() {
        let visibility_all: usize = super::raycasting_with_dist((10, 10), 1, |_, _| true, |x| x)
            .iter()
            .map(|(_, c)| c)
            .sum();
        assert_eq!(visibility_all, 8);
    }
}
