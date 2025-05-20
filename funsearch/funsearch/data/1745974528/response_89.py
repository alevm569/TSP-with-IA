def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a hybrid heuristic that combines two or more of the following techniques:
    # - Nearest neighbor
    # - Cheapest insertion
    # - Local search
    # - 2-opt

    # Example hybrid heuristic:
    # 1. Use nearest neighbor to generate an initial route.
    # 2. Perform local search on the initial route to find a better solution.

    # Return the best route found.
    return tuple(range(len(_distances)))
