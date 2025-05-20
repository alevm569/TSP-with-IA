def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid approach that combines the nearest neighbor heuristic with the 2-opt local search algorithm.

    # Return the best route found.
    return tuple(range(len(_distances)))
