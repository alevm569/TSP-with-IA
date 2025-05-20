def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`.

    Uses a combination of local search and 2-opt heuristics.
    """
    # Create an initial random route
    num_cities = len(_distances)
    initial_route = np.random.permutation(num_cities)

    # Perform local search
    best_route = local_search(_distances, initial_route)

    # Perform 2-opt optimization
    best_route = two_opt(_distances, best_route)

    return best_route

def local_search(_distances: np.ndarray, route: tuple[int, ...]) -> tuple[int, ...]:
    """Performs local search to improve a route."""
    # ... (Implementation of local search algorithm)

def two_opt(_distances: np.ndarray, route: tuple[int, ...]) -> tuple[int, ...]:
    """Performs 2-opt optimization to improve a route."""
    # ... (Implementation of 2-opt algorithm)
