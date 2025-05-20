def find_best_route_v3(_distances: np.ndarray) -> tuple[int, ...]:
    """Improved version of `find_best_route_v2`."""

    # Implement your new heuristic here.
    # For example, you could use a hybrid approach that combines several smaller heuristics.

    # Return the best route as a tuple of city indices.
    return tuple(np.random.permutation(len(_distances)))
