def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """
    Improved version of `find_best_route_v1` using a combination of local search and 2-opt heuristics.

    Parameters:
    _distances (np.ndarray): A square matrix of distances between cities, shape (n, n)

    Returns:
    tuple[int, ...]: The best valid route found.
    """

    # Generate an initial solution using the nearest neighbor heuristic
    initial_route = nearest_neighbor(_distances)

    # Perform local search to find a better solution
    best_route = local_search(initial_route, _distances)

    # Perform 2-opt operations to further improve the solution
    best_route = two_opt(best_route, _distances)

    return best_route


# Local search heuristic
def local_search(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # ... Local search logic here ...

# 2-opt heuristic
def two_opt(route: tuple[int, ...], distances: np.ndarray) -> tuple[int, ...]:
    # ... 2-opt logic here ...

# Nearest neighbor heuristic
def nearest_neighbor(distances: np.ndarray) -> tuple[int, ...]:
    # ... Nearest neighbor logic here ...
