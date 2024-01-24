#![allow(dead_code)]
/// Map problems for testing. Contains a path to the map, the start and goal
type Map = (&'static str, (usize, usize), (usize, usize));
pub const BASIC: Map = ("tests/basic.map", (1, 1), (30, 30));
pub const MAP: Map = ("tests/map.map", (225, 225), (70, 40));
pub const WALL: Map = ("tests/wall.map", (3, 1), (3, 6));
pub const CACAVERNS: Map = ("tests/ca_caverns1.map", (122, 595), (200, 15));
pub const DRYWATER: Map = ("tests/drywatergulch.map", (175, 315), (320, 125));
pub const FLOODEDPLAINS: Map = ("tests/FloodedPlains.map", (160, 100), (480, 330));
pub const HRT: Map = ("tests/hrt201d.map", (70, 28), (250, 235));
pub const LAK: Map = ("tests/lak201d.map", (30, 150), (100, 50));
pub const MAZE: Map = ("tests/maze512-8-4.map", (10, 10), (380, 325));
pub const MEDUSA: Map = ("tests/Medusa.map", (60, 250), (460, 20));
pub const SIROCCO: Map = ("tests/Sirocco.map", (10, 250), (750, 250));
pub const TRISKELION: Map = ("tests/Triskelion.map", (260, 500), (10, 10));
pub const WAYPOINTJUNCTION: Map = ("tests/WaypointJunction.map", (245, 20), (260, 500));
