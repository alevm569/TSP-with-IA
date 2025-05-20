def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""
    # Use a hybrid approach that combines the nearest neighbor heuristic with the 2-opt local search.
    # Initialize the route using the nearest neighbor heuristic.
    route = nearest_neighbor(_distances)

    # Apply the 2-opt local search algorithm to improve the route.
    route = local_search(route, _distances)

    return route


def nearest_neighbor(_distances: np.ndarray) -> tuple[int, ...]:
    # ... (Implementation of the nearest neighbor heuristic)

def local_search(route: tuple[int, ...], _distances: np.ndarray) -> tuple[int, ...]:
    # ... (Implementation of the 2-opt local search algorithm)
