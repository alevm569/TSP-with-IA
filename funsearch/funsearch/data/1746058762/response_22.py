def find_best_route_v2(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v1`."""

    # Implement a new heuristic or strategy here.
    # For example, you could use a 2-opt heuristic to improve the route.

    # Ensure that the route includes all cities exactly once and returns to the starting point.
    return tuple(np.random.permutation(np.arange(len(_distances))))
