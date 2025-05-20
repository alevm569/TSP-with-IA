def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a hybrid of nearest neighbor and cheapest insertion.

    # Ensure that the returned route includes all cities exactly once and returns to the starting point.
    return tuple(range(len(_distances)))
