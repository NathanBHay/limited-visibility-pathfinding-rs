#![allow(dead_code)]
/// Problem problems for testing. Contains a path to the map, the start and goal
pub type Problem = (&'static str, &'static str, (usize, usize), (usize, usize));
pub const BASIC: Problem = ("Basic", "tests/basic.map", (1, 1), (30, 30));
pub const BLOCK: Problem = ("Block", "tests/block.map", (8, 0), (8, 6));
pub const MAP: Problem = ("Map", "tests/map.map", (225, 225), (70, 40));
pub const FILL: Problem = ("Fill", "tests/fill.map", (4, 1), (4, 6));
pub const WALL: Problem = ("Wall", "tests/wall.map", (3, 1), (3, 6));
pub const CACAVERNS: Problem = ("Caverns", "tests/ca_caverns1.map", (122, 595), (200, 15));
pub const DRYWATER: Problem = ("Drywater", "tests/drywatergulch.map", (175, 315), (320, 125));
pub const FLOODEDPLAINS: Problem = ("Flood Plains", "tests/FloodedPlains.map", (160, 100), (480, 330));
pub const HRT: Problem = ("HRT", "tests/hrt201d.map", (70, 28), (250, 235));
pub const LAK: Problem = ("Lake", "tests/lak201d.map", (30, 150), (100, 50));
pub const MAZE: Problem = ("Maze", "tests/maze512-8-4.map", (10, 10), (380, 325));
pub const MEDUSA: Problem = ("Medusa", "tests/Medusa.map", (60, 250), (460, 20));
pub const SIROCCO: Problem = ("Sirocco", "tests/Sirocco.map", (10, 250), (750, 250));
pub const TRISKELION: Problem = ("Triskelion", "tests/Triskelion.map", (260, 500), (10, 10));
pub const WAYPOINTJUNCTION: Problem = ("Waypoint Junc", "tests/WaypointJunction.map", (245, 20), (260, 500));
pub const MAP_PACK: [Problem; 6] = [
    MAP,
    WALL,
    FILL,
    CACAVERNS,
    // DRYWATER,
    // FLOODEDPLAINS,
    HRT,
    LAK,
    // MAZE,
    // MEDUSA,
    // SIROCCO,
    // TRISKELION,
    // WAYPOINTJUNCTION,
];