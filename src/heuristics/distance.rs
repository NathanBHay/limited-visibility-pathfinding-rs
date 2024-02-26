//! Distance heuristics used to compute the distance between two points in a grid.
//! These heuristics are used on grid maps in pathfinding algorithms such as A*.
//! All of these heuristics are admissible, meaning they result in an optimal path.

/// Manhattan distance heuristic.
pub fn manhattan_distance((x1, y1): (usize, usize), (x2, y2): (usize, usize)) -> usize {
    x1.abs_diff(x2) + y1.abs_diff(y2)
}

/// 3D Manhattan distance heuristic.
pub fn manhattan_distance3d((x1, y1, z1): (usize, usize, usize), (x2, y2, z2): (usize, usize, usize)) -> usize {
    x1.abs_diff(x2) + y1.abs_diff(y2) + z1.abs_diff(z2)
}

/// Euclidean distance heuristic.
pub fn euclidean_distance((x1, y1): (i32, i32), (x2, y2): (i32, i32)) -> f32 {
    (((x1 as f32 - x2 as f32).powi(2) + (y1 as f32 - y2 as f32).powi(2)) as f32).sqrt()
}

/// Chebyshev distance heuristic.
pub fn chebyshev_distance((x1, y1): (i32, i32), (x2, y2): (i32, i32)) -> i32 {
    (x1 - x2).abs().max((y1 - y2).abs())
}

/// Octile distance heuristic.
pub fn octile_distance((x1, y1): (i32, i32), (x2, y2): (i32, i32)) -> f32 {
    let dx = (x1 - x2).abs();
    let dy = (y1 - y2).abs();
    let min = dx.min(dy);
    let max = dx.max(dy);
    (min as f32 * 1.4142135623730951) + (max - min) as f32
}
