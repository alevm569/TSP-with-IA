def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement a new heuristic or combination of heuristics here.
    # For example, you could use a combination of nearest neighbor and cheapest insertion.

    # Return the best route as a tuple of city indices.
    return tuple(np.random.permutation(len(_distances)))
