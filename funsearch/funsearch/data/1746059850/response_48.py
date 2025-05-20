def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2` using a hybrid heuristic."""
    # Implement a hybrid heuristic that combines different strategies.
    # For example, you could use the nearest neighbor heuristic to find an initial route,
    # and then apply the 2-opt heuristic to improve it.

    # Return the best route as a tuple of city indices.
    return tuple(range(len(_distances)))
