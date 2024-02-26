# Limited Visibility Pathfinding
This is a limited visibility pathfinding project designed for my summer research project over the span of November 2023 to February 2024. The purpose of this README is to function as a general commentary on the design process while also being an overview of what is in the repository. Before I go onto that I must first thank Professor Peter Stuckey and Daniel Harabor, and future PHD Michael Chen for their creation of the project and supervision. Additionally, I would like to thank Anthony Zhoon for his recommendations such as probability filters and hashing optimisations. Without these people I wouldn't have been able to get nearly as far, so to them, thank you.

The goal of this project is to offer a solution to the *limited visibility pathfinding* problem. For the purpose of this repository the problem is defined as finding a robust path between two points, given an understanding of the area from sensor data and a satellite image. This means that the agent needs to find a path, updating the area every movement, accounting for the new information as it approaches walls. This has applications in fields such as drone delivery, or similar tasks.

## Repository Overview
The repository is built around two main parts, *grid domains* and *searches*. The searches are functions that allow for searching from a starting node to a goal function, using an expansion function. Due to this they don't have a dependency or association on grid domains which makes them a flexible search. This idea of attempting to minimise dependencies was a large part of the project design. To supplement these searches are predesigned heuristics. The other half is in the form of grid domains. These function as entities which can be searched on using the `adjacent` expansion function. These generally represent a uniform discrete grid. 

### Search Overview
There are a variety of searches found within the library which all operate by using a central `Search` trait. The searches which can be found within the repository are:
- A-Star, a flexible search implementation that uses a heuristic to improve its search order.
- Focal search, which allows for a semi-admissible heuristic to be used in addition to an admissible one.
- D-Star, currently not fully implemented, but offers a solution to lifelong pathfinding.
- Sample-Star, an algorithm used to solve the limited visibility pathfinding problem.
- Sample-Star Baseline, another weaker algorithm to solve limited visibility pathfinding which doesn't use satellite data.

### Domain Overview
The grid domains are unified under the `GridDomain` trait which has sub-traits for two and three-dimensional domains. Some domains have support for multiple dimensions and as such have an interface to represent the domain for polymorphism. The domains in the repository are:
- Bit-Packed Grids, a bit field which is used to represent a simple grid.
- Hashed Grids, a simple grid which uses a hash table.
- Sample Grids, a grid used to represent the likelihood of a space being an obstacle as well as the underlying truth.

## Design Commentary
Throughout the creation of this project a variety of decisions were made to achieve the set out goal while also ensuring a level of optimisation. I'll discuss decisions in order of importance and generally by aspect of the project. Before I get to that though I want to discuss the first decision made, that being the choice of Rust. Rust was chosen as the main language of the project as I had done a previous project with it and wanted to expand my knowledge on the language principally. Beyond these qualitative reasons I also was interested in the speed of the language and support for native multi-threading both traits which would allow for a fast codebase.

### Sample-Star
Sample-Star is an algorithm used to solve limited visibility pathfinding. It functions similar to a lifelong pathfinding problem with it computing a new path at each step however in contrast to lifelong pathfinding it samples from a grid of probabilities and runs multiple searches. These probabilities represent the chance of their being a free-space at that location. The multiple searches are done as to ensure a convergence on the best solution. These searches are run in parallel as to speed up execution. The goal of the algorithm is to be highly configurable with support of different searches and path stores. The algorithm also offers an alternative in the form of the baseline which only samples from locations it has visited, therefore assuming open space outside that area. This is optimised to jump to the goal.

An interesting aspect of the algorithm is that it is still able to move in the case of no paths successfully getting to the goal. This is done when a search fails to find the goal, it must then use a heuristic to find the next best node and move towards there. A heuristic was specifically made for this which calculates the best node as the one with the lowest accumulated log value, tie-broken on distance from the goal.

### Searches
The general goal of the searches was to be flexible and non-dependent on the domains. This was achieved through the use of generics and higher order functions. This choice while making the codebase confusing also made it flexible. The use of a trait to represent searching was another intriguing approach to the problem. This trait decreased repeated code while also enabling it Best Searches to be made. A search which settles upon the next best node given no path to the goal. 

### Domains
The domain's use of traits were mainly done to reduce redundant code rather allow for polymorphism. While I do like this idea in practice I do wonder if the code quality suffers due to this. The choice to make a separate trait for bit-packed and sample grids that doesn't depend on grid domains I believe was also a better abstraction that avoided an interface pyramid. Ray casting was chosen as the main visibility algorithm due to its calculation speed in comparison to other methods. This led to a few unique vision rules as well as the case on glitching at a larger vision radius.

### Path Stores
A path store functions as the structure which holds paths that successfully/unsuccessfully get to the goal. This abstraction to a trait allows for different types of stores. The basic store is a count store which will choose to advance to the next square based on the whatever square had the most successful searches go through it. This can be made more advance with an accumulator store which allows for unique voting heuristic that weights certain paths. The last approach is a greedy store which stores one best path. In general an accumulator store is best for the success case while for paths that don't find a path a greedy store is the best.

### Statistics & Visualisation
The visualiser is made up of two parts, a rust serialiser that converts the grid state to JSON, for the second part a Python visualiser. JSON while slow offered a human-readable format that had serialisation and deserialisation support by both Python and Rust. The visualiser was constructed in Python due to Matplotlib being a simpler and more supported alternative to any Rust visualisation library. The ability for the Rust part of the codebase to output results also allowed bug fixing for cases when the visualiser wasn't able to interpret results. The visualiser was made as a CLI utility with flags due to the need of different plotting methods.

The statistics part of the codebase function as a way of using a dynamic set of functions to record statistics from Sample-Star. This was a design decision I wasn't super happy with due to the association between the stats and sample-star. Ideally neither would know about each other however in order to properly record statistics like expanded nodes, and average path results this change was made. 

### Matrix Utilities
Throughout the codebase is the use of a matrix structure created by me. The lack of use of an external library is due to the desire to keep dependencies low while also learning how to program basic matrix operations. While efficient for 2d operations this has created an expensive factor if you wanted to create 3d matrices.

## Conclusion
This project has been very helpful in developing my skill set in both programming, and problem-solving. The unique process of creating optimised algorithms has given me a newfound appreciation to many of the algorithms I had once considered mundane. Once again I would like to thank all who helped me, and the University.