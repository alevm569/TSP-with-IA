def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or strategy here.
    # For example, you could use a combination of the nearest neighbor and cheapest insertion heuristics.

    # Return the best route as a tuple of integers.
    return tuple(range(len(_distances)))
