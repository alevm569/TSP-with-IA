def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement your new heuristic or combination of heuristics here.
    # For example, you could use a hybrid of the nearest neighbor and cheapest insertion heuristics.

    # Return the best route as a tuple of city indices.
    return tuple(range(len(_distances)))
